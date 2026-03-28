// SPDX-License-Identifier: GPL-2.0
/*
 * KPM OC Module for MediaTek MT8792 (Dimensity 8300)
 * Reads CPU DVFS LUT from performance-controller CSRAM domains.
 * Driver: mtk-cpufreq-hw — LUT bits[11:0] = freq_MHz.
 * GPU data is read from userspace via /proc/gpufreqv2.
 *
 * Kernel 6.1 GKI compatible.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mutex.h>

/* ─── CSRAM Configuration ───────────────────────────────────────────────── */
/*
 * From DTB cpuhvfs@114400 reg-names:
 *   reg[0] = 0x114400 (USRAM)  ← leakage / hybrid tables
 *   reg[1] = 0x11BC00 (CSRAM)  ← performance-controller LUT lives here
 *   reg[2] = 0x118000 (ESRAM)
 *
 * Performance-controller domains (from DT 11bc10.performance-controller reg):
 *   domain0 (L  policy0): 0x11BC10, size 0x120
 *   domain1 (B  policy4): 0x11BD30, size 0x120
 *   domain2 (P  policy7): 0x11BE50, size 0x120
 *
 * We map the whole CSRAM region and use domain offsets within it.
 */
#define CSRAM_PHYS_BASE   0x0011BC00UL
#define CSRAM_PHYS_SIZE   0x00001400UL

/* Domain LUT base offsets within CSRAM */
static const unsigned int domain_offsets[] = { 0x10, 0x130, 0x250 };
static const int cluster_policies[] = { 0, 4, 7 };
static const char * const cluster_names[] = { "L", "B", "P" };
#define NUM_CLUSTERS      3

/*
 * mtk-cpufreq-hw register offsets within each domain:
 *   REG_FREQ_LUT_TABLE = 0x00   (frequency LUT)
 *   REG_EM_POWER_TBL   = 0x90   (energy model power table)
 */
#define REG_FREQ_LUT  0x00
#define REG_EM_POWER  0x90

/* LUT entry format (mtk-cpufreq-hw on MT6897):
 *   bits[11:0]  = frequency in MHz
 *   bits[30:29] = gear selector (voltage domain switch)
 *   remaining upper bits >> 12 (with bits 30:29 cleared) = voltage in 10 µV units
 *   Voltage (µV) = ((raw & 0x9FFFFFFF) >> 12) * 10
 */
#define LUT_FREQ_MASK     0x00000FFFU
#define LUT_GEAR_MASK     0x60000000U
#define LUT_VOLT_DECODE(r) ((unsigned int)(((r) & 0x9FFFFFFFU) >> 12) * 10)
#define LUT_ROW_SIZE      4
#define LUT_MAX_ENTRIES   32

/* ─── Export Buffers ────────────────────────────────────────────────────── */
/* OPP table: CPU:<policy>:<freq_khz>:<volt_uv>|...  */
#define OPP_BUF_SIZE      8192
static char opp_table_export[OPP_BUF_SIZE];
module_param_string(opp_table, opp_table_export, sizeof(opp_table_export), 0444);
MODULE_PARM_DESC(opp_table, "CPU OPP table (CPU:policy:freq_khz:volt_uv|...)");

/* Raw hex dump for debugging: first 8 LUT entries per domain */
#define RAW_BUF_SIZE      4096
static char raw_dump[RAW_BUF_SIZE];
module_param_string(raw, raw_dump, sizeof(raw_dump), 0444);
MODULE_PARM_DESC(raw, "Raw hex dump of LUT entries for debugging");

static DEFINE_MUTEX(lock);
static void __iomem *csram_base;

/* ─── Scan CSRAM and Export CPU Freq Table ──────────────────────────────── */

static int set_apply(const char *val, const struct kernel_param *kp)
{
	int c, i;
	char buf[96];
	int opp_pos = 0;
	int raw_pos = 0;

	if (!csram_base) {
		pr_err("KPM_OC: CSRAM not mapped\n");
		return -ENOMEM;
	}

	mutex_lock(&lock);
	memset(opp_table_export, 0, sizeof(opp_table_export));
	memset(raw_dump, 0, sizeof(raw_dump));

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int prev_freq = 0;
		unsigned int dom_base = domain_offsets[c];
		int entry_count = 0;

		/* Raw dump header */
		raw_pos += snprintf(raw_dump + raw_pos,
				    RAW_BUF_SIZE - raw_pos,
				    "[%s/p%d LUT@0x%x EM@0x%x] ",
				    cluster_names[c],
				    cluster_policies[c],
				    dom_base + REG_FREQ_LUT,
				    dom_base + REG_EM_POWER);

		for (i = 0; i < LUT_MAX_ENTRIES; i++) {
			u32 lut_val = readl_relaxed(csram_base + dom_base +
						    REG_FREQ_LUT +
						    i * LUT_ROW_SIZE);
			u32 em_val  = readl_relaxed(csram_base + dom_base +
						    REG_EM_POWER +
						    i * LUT_ROW_SIZE);
			unsigned int freq_mhz = lut_val & LUT_FREQ_MASK;
			unsigned int volt_uv  = LUT_VOLT_DECODE(lut_val);

			/* End: freq==0 or repeated freq */
			if (freq_mhz == 0 || freq_mhz == prev_freq)
				break;

			/* Raw dump (first 8 per domain) */
			if (i < 8 && raw_pos < RAW_BUF_SIZE - 32)
				raw_pos += snprintf(raw_dump + raw_pos,
						    RAW_BUF_SIZE - raw_pos,
						    "%08x/%08x ",
						    lut_val, em_val);

			/* OPP export: freq KHz and voltage µV */
			snprintf(buf, sizeof(buf), "CPU:%d:%u:%u|",
				 cluster_policies[c],
				 freq_mhz * 1000,   /* KHz */
				 volt_uv);           /* Decoded voltage µV */

			if (opp_pos + (int)strlen(buf) < OPP_BUF_SIZE - 1) {
				memcpy(opp_table_export + opp_pos, buf,
				       strlen(buf));
				opp_pos += strlen(buf);
			}

			prev_freq = freq_mhz;
			entry_count++;
		}

		if (raw_pos < RAW_BUF_SIZE - 2)
			raw_pos += snprintf(raw_dump + raw_pos,
					    RAW_BUF_SIZE - raw_pos, "| ");

		pr_info("KPM_OC: Cluster %s (policy%d): %d LUT entries at CSRAM+0x%x\n",
			cluster_names[c], cluster_policies[c],
			entry_count, dom_base);
	}

	opp_table_export[opp_pos] = '\0';
	/* Remove trailing pipe */
	if (opp_pos > 0 && opp_table_export[opp_pos - 1] == '|')
		opp_table_export[opp_pos - 1] = '\0';

	mutex_unlock(&lock);
	return 0;
}

static const struct kernel_param_ops apply_ops = { .set = set_apply };
static int apply_dummy;
module_param_cb(apply, &apply_ops, &apply_dummy, 0220);
MODULE_PARM_DESC(apply, "Write 1 to scan CSRAM and export CPU OPP table");

/* ─── Module Init / Exit ────────────────────────────────────────────────── */

static int __init kpm_oc_init(void)
{
	csram_base = ioremap(CSRAM_PHYS_BASE, CSRAM_PHYS_SIZE);
	if (!csram_base) {
		pr_err("KPM_OC: Failed to ioremap CSRAM at 0x%lx\n",
		       CSRAM_PHYS_BASE);
		return -ENOMEM;
	}

	snprintf(opp_table_export, sizeof(opp_table_export), "READY");
	pr_info("KPM_OC: MT8792 CSRAM OPP reader v3.2 (base=0x%lx)\n",
		CSRAM_PHYS_BASE);

	/* Auto-scan on load */
	set_apply("1", NULL);

	return 0;
}

static void __exit kpm_oc_exit(void)
{
	if (csram_base)
		iounmap(csram_base);
	pr_info("KPM_OC: Unloaded.\n");
}

module_init(kpm_oc_init);
module_exit(kpm_oc_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zerofrip");
MODULE_DESCRIPTION("MT8792 CSRAM CPU OPP reader for KPM OC Manager v3.2");

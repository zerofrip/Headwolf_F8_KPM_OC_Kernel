// SPDX-License-Identifier: GPL-2.0
/*
 * KPM OC Module for MediaTek MT8792 (Dimensity 8300)
 * Reads CPU DVFS LUT from CPUHVFS CSRAM for voltage information.
 * GPU data is read from userspace via /proc/gpufreqv2.
 *
 * Kernel 6.1 GKI compatible.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mutex.h>

/* ─── CSRAM Configuration ───────────────────────────────────────────────── */
/* Physical base from DTB: cpuhvfs@114400 */
#define CSRAM_PHYS_BASE   0x00114400UL
#define CSRAM_PHYS_SIZE   0x00000C00UL

/* Table offsets within CSRAM from DTB tbl-off property:
 * [0x04, 0x4C, 0x94, 0xDC] → L, B, P, CCI */
static const int tbl_offsets[] = { 0x04, 0x4C, 0x94 };
static const int cluster_policies[] = { 0, 4, 7 };
static const char * const cluster_names[] = { "L", "B", "P" };
#define NUM_CLUSTERS      3

/* LUT entry format for mediatek,cpufreq-hybrid:
 *   bits[11:0]  = frequency in MHz
 *   bits[27:16] = voltage step (each step = 6.25 mV = 6250 µV)
 */
#define LUT_FREQ_MASK     0x00000FFFU
#define LUT_VOLT_SHIFT    16
#define LUT_VOLT_MASK     0x0FFF0000U
#define VOLT_STEP_UV      6250
#define LUT_MAX_ENTRIES   18
#define LUT_ENTRY_BYTES   4

/* ─── Export Buffer ─────────────────────────────────────────────────────── */
/* Format: CPU:<policy>:<freq_khz>:<volt_uv>|...  */
#define OPP_BUF_SIZE      4096
static char opp_table_export[OPP_BUF_SIZE];
module_param_string(opp_table, opp_table_export, sizeof(opp_table_export), 0444);
MODULE_PARM_DESC(opp_table, "CPU OPP table from CSRAM (CPU:policy:freq_khz:volt_uv|...)");

static DEFINE_MUTEX(lock);
static void __iomem *csram_base;

/* ─── Scan CSRAM and Export CPU Freq/Volt Table ─────────────────────────── */

static int set_apply(const char *val, const struct kernel_param *kp)
{
	int c, i;
	char buf[64];

	if (!csram_base) {
		pr_err("KPM_OC: CSRAM not mapped\n");
		return -ENOMEM;
	}

	mutex_lock(&lock);
	memset(opp_table_export, 0, sizeof(opp_table_export));

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int prev_freq = 0;
		int offset = tbl_offsets[c];
		int entry_count = 0;

		for (i = 0; i < LUT_MAX_ENTRIES; i++) {
			u32 raw = readl_relaxed(csram_base + offset +
						i * LUT_ENTRY_BYTES);
			unsigned int freq_mhz = raw & LUT_FREQ_MASK;
			unsigned int volt_raw = (raw & LUT_VOLT_MASK) >>
						LUT_VOLT_SHIFT;
			unsigned int volt_uv  = volt_raw * VOLT_STEP_UV;

			if (freq_mhz == 0 || freq_mhz == prev_freq)
				break;

			snprintf(buf, sizeof(buf), "CPU:%d:%u:%u|",
				 cluster_policies[c],
				 freq_mhz * 1000,   /* KHz */
				 volt_uv);

			if (strlen(opp_table_export) + strlen(buf) <
			    OPP_BUF_SIZE - 1)
				strlcat(opp_table_export, buf,
					sizeof(opp_table_export));

			prev_freq = freq_mhz;
			entry_count++;
		}

		pr_info("KPM_OC: Cluster %s (policy%d): %d LUT entries\n",
			cluster_names[c], cluster_policies[c], entry_count);
	}

	/* Remove trailing pipe */
	{
		size_t len = strlen(opp_table_export);
		if (len > 0 && opp_table_export[len - 1] == '|')
			opp_table_export[len - 1] = '\0';
	}

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
	pr_info("KPM_OC: MT8792 CSRAM OPP reader v3.0 initialized\n");

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
MODULE_DESCRIPTION("MT8792 CSRAM CPU OPP reader for KPM OC Manager v3.0");

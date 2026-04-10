// SPDX-License-Identifier: GPL-2.0
/*
 * KPM OC Module for MediaTek MT8792 (Dimensity 8300)
 * - Reads CPU DVFS LUT from performance-controller CSRAM domains.
 * - CPU OC: patches CSRAM LUT[0] per cluster + cpufreq policy max.
 * - GPU OC: patches default + working + shared_status OPP tables.
 *   v6.4: CPU OC support added.
 *   v6.6: Lift vendor freq_qos MAX constraints (powerhal, fpsgo, touch_boost)
 *         so scaling_max_freq can reach the OC'd frequency.
 *   v6.8: Relift reliability updates
 *         - Start relift worker whenever any CPU OC target is set (even NOOP re-apply)
 *         - Enforce policy max/cpuinfo max in worker and tighten relift interval.
 *   v7.0: Per-OPP voltage override for all CPU LUT entries and GPU OPP entries.
 *         Bypasses vendor fix_custom_freq_volt validation (DVFSState, volt clamp)
 *         by writing directly to CSRAM / kernel memory.
 *   v7.2: MCUPM/GPUEB CSRAM overwrite countermeasure.
 *         Root cause: MCUPM firmware continuously recalculates and overwrites
 *         CSRAM LUT voltage fields; GPUEB firmware overwrites GPU OPP table
 *         voltages via shared_status.  Previous 500ms periodic relift could
 *         not keep up.
 *         Fix: Event-driven voltage resync via kprobes on:
 *         - mtk_cpufreq_hw_fast_switch / mtk_cpufreq_hw_target_index
 *           (re-patches CSRAM LUT before HW reads it on every DVFS transition)
 *         - __gpufreq_generic_commit_gpu
 *           (re-patches GPU OPP tables before commit reads them)
 *         Periodic relift retained as safety net only.
 *   v7.6: Bug fixes and sysfs improvements.
 *         - opp_table / raw sysfs params: converted from static
 *           module_param_string to module_param_cb with .get callbacks
 *           that re-read CSRAM on every sysfs read (live data, no
 *           'apply' write required).
 *         - set_cpu_oc freq_table update: always writes tbl[0].frequency
 *           instead of searching for the max-freq entry. Fixes re-OC
 *           after underclock where index 0 held the underclocked value.
 *
 * Driver: mtk-cpufreq-hw — LUT bits[11:0] = freq_MHz.
 * Kernel 6.1 GKI compatible.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/kprobes.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/input.h>

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

/* ─── ARMPLL Direct Access Configuration ─────────────────────────────── */
/*
 * Bypass MCUPM firmware by writing ARMPLL PLL registers directly from AP.
 * On MT6897 with scmi_cpufreq, MCUPM controls actual PLL frequency via its
 * own frequency table.  CSRAM LUT patching only changes kernel metadata.
 * This section directly programs ARMPLL PCW (phase-controlled word) registers
 * to achieve real hardware overclocking.
 *
 * From DTB fhctl node — MCU PLL subsystem at 0x0C030000:
 *   ccipll:    base+0x000, FHCTL base+0x100
 *   armpll_ll: base+0x400, FHCTL base+0x500  (L cluster, policy0)
 *   armpll_bl: base+0x800, FHCTL base+0x900  (B cluster, policy4)
 *   armpll_b:  base+0xC00, FHCTL base+0xD00  (P cluster, policy7)
 *
 * PLL frequency formula:
 *   freq_hz = FIN × PCW / 2^PCW_FRAC_BITS / POSTDIV
 *   where FIN = 26 MHz, PCW_FRAC_BITS = 14, POSTDIV = 2^postdiv_pow
 *
 * PLL_CON1 register layout:
 *   bits[21:0]  = PCW (DDS value)
 *   bits[26:24] = POSTDIV power (0=÷1, 1=÷2, 2=÷4, ...)
 *
 * WARNING: Direct PLL manipulation bypasses MCUPM DVFS.  Voltage is NOT
 * adjusted here — the CPU runs at whatever voltage MCUPM last set for the
 * stock frequency.  Overclocking without voltage increase risks instability.
 */
#define MCU_PLL_PHYS_BASE    0x0C030000UL
#define MCU_PLL_PHYS_SIZE    0x00005000UL  /* 20KB: ccipll through ptppll */

/* Per-cluster ARMPLL offset from MCU_PLL_PHYS_BASE */
static const unsigned int pll_offsets[] = { 0x400, 0x800, 0xC00 }; /* L, B, P */

/* PLL CON register offsets from per-PLL base */
#define PLL_CON0             0x00
#define PLL_CON1             0x04  /* PCW[21:0] + POSTDIV[26:24] */
#define PLL_CON2             0x08
#define PLL_CON3             0x0C

/* FHCTL (frequency hopping controller) register offsets
 * MT6897-specific: from fhctl.ko mt6897_mcu*_offset tables [0x14,0x0C,0x08,0x10,0x04]
 */
#define FHCTL_OFF            0x100  /* FHCTL base = PLL base + 0x100 */
#define REG_FHCTL_CFG        0x14   /* per-PLL FHCTL config */
#define REG_FHCTL_UPDNLMT   0x0C   /* up/down limit */
#define REG_FHCTL_DDS        0x08   /* center DDS (SSC base) */
#define REG_FHCTL_DVFS       0x10   /* DVFS target DDS */
#define REG_FHCTL_MON        0x04   /* PLL DDS monitor (current) */

/* PLL / FHCTL bit masks */
#define PLL_DDS_MASK         0x003FFFFFUL  /* PCW: bits[21:0] */
#define PLL_POSTDIV_MASK     0x07000000UL  /* bits[26:24] */
#define PLL_POSTDIV_SHIFT    24
#define FHCTL_DDS_VALID      BIT(31)
#define FHCTL_SFSTR_EN       BIT(2)  /* soft-start (DVFS) enable */
#define FHCTL_FHCTL_EN       BIT(0)  /* frequency hopping enable */

#define PLL_FIN_MHZ          26
#define PLL_PCW_FRAC_BITS    14


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

/* Raw hex dump for debugging: first 8 LUT entries per domain */
#define RAW_BUF_SIZE      4096
static char raw_dump[RAW_BUF_SIZE];

static DEFINE_MUTEX(lock);
static void __iomem *csram_base;

/* ─── Cached OPP table (survives MCUPM CSRAM rewrites) ─────────────────── */
/*
 * MCUPM firmware rewrites CSRAM LUT after boot, reducing entries to 1 per
 * cluster.  We cache the full table at module init (when CSRAM is fresh)
 * and serve the cached data from get_opp_table().  OC and voltage override
 * operations update the cache directly.
 */
struct opp_entry {
	unsigned int freq_khz;
	unsigned int volt_uv;
};
static struct opp_entry opp_cache[NUM_CLUSTERS][LUT_MAX_ENTRIES];
static int opp_cache_count[NUM_CLUSTERS];
static bool opp_cache_valid;

/* ─── Dynamic sysfs readers for opp_table and raw ──────────────────────── */

/*
 * Shared helper: scan CSRAM LUT and populate opp_table_export[] + raw_dump[].
 * Also populates opp_cache[] if not already valid.
 * Called from set_apply() and get_raw_dump().
 * Caller must hold lock.
 */
static void __scan_csram_lut_locked(void)
{
	int c, i;
	char buf[96];
	int opp_pos = 0;
	int raw_pos = 0;
	int total_entries = 0;

	if (!csram_base)
		return;

	memset(opp_table_export, 0, sizeof(opp_table_export));
	memset(raw_dump, 0, sizeof(raw_dump));

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int prev_freq = 0;
		unsigned int dom_base = domain_offsets[c];
		int cluster_count = 0;

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

			if (freq_mhz == 0 || freq_mhz == prev_freq)
				break;

			if (i < 8 && raw_pos < RAW_BUF_SIZE - 32)
				raw_pos += snprintf(raw_dump + raw_pos,
						    RAW_BUF_SIZE - raw_pos,
						    "%08x/%08x ",
						    lut_val, em_val);

			snprintf(buf, sizeof(buf), "CPU:%d:%u:%u|",
				 cluster_policies[c],
				 freq_mhz * 1000,
				 volt_uv);

			if (opp_pos + (int)strlen(buf) < OPP_BUF_SIZE - 1) {
				memcpy(opp_table_export + opp_pos, buf,
				       strlen(buf));
				opp_pos += strlen(buf);
			}

			cluster_count++;
			prev_freq = freq_mhz;
		}

		total_entries += cluster_count;

		if (raw_pos < RAW_BUF_SIZE - 2)
			raw_pos += snprintf(raw_dump + raw_pos,
					    RAW_BUF_SIZE - raw_pos, "| ");
	}

	opp_table_export[opp_pos] = '\0';
	if (opp_pos > 0 && opp_table_export[opp_pos - 1] == '|')
		opp_table_export[opp_pos - 1] = '\0';

	/* Populate cache only if CSRAM has meaningful data (>1 per cluster avg).
	 * MCUPM firmware rewrites CSRAM after boot, leaving ~1 entry per cluster.
	 * We only cache the full initial scan; subsequent sparse scans are ignored.
	 */
	if (!opp_cache_valid && total_entries > NUM_CLUSTERS) {
		for (c = 0; c < NUM_CLUSTERS; c++) {
			unsigned int prev_f = 0;
			unsigned int dom_base = domain_offsets[c];
			int cnt = 0;

			for (i = 0; i < LUT_MAX_ENTRIES; i++) {
				u32 lut_val = readl_relaxed(csram_base +
					dom_base + REG_FREQ_LUT +
					i * LUT_ROW_SIZE);
				unsigned int f = lut_val & LUT_FREQ_MASK;
				unsigned int v = LUT_VOLT_DECODE(lut_val);

				if (f == 0 || f == prev_f)
					break;
				opp_cache[c][i].freq_khz = f * 1000;
				opp_cache[c][i].volt_uv  = v;
				cnt++;
				prev_f = f;
			}
			opp_cache_count[c] = cnt;
		}
		opp_cache_valid = true;
		pr_info("KPM_OC: OPP cache populated (%d/%d/%d entries)\n",
			opp_cache_count[0], opp_cache_count[1],
			opp_cache_count[2]);
	}
}

/*
 * Generate OPP table string from cache.
 * Caller must hold lock.
 */
static int __format_opp_cache_locked(char *buf, int buf_size)
{
	int c, i, pos = 0;
	char tmp[96];

	for (c = 0; c < NUM_CLUSTERS; c++) {
		for (i = 0; i < opp_cache_count[c]; i++) {
			int len = snprintf(tmp, sizeof(tmp), "CPU:%d:%u:%u|",
					   cluster_policies[c],
					   opp_cache[c][i].freq_khz,
					   opp_cache[c][i].volt_uv);
			if (pos + len < buf_size - 1) {
				memcpy(buf + pos, tmp, len);
				pos += len;
			}
		}
	}
	buf[pos] = '\0';
	if (pos > 0 && buf[pos - 1] == '|')
		buf[--pos] = '\0';
	return pos;
}

static int get_opp_table(char *buf, const struct kernel_param *kp)
{
	int len;

	mutex_lock(&lock);

	/* If cache is valid, serve cached data (stable across MCUPM rewrites).
	 * Otherwise fall back to live CSRAM scan. */
	if (opp_cache_valid) {
		len = __format_opp_cache_locked(opp_table_export,
						sizeof(opp_table_export));
		len = scnprintf(buf, PAGE_SIZE, "%s", opp_table_export);
	} else {
		__scan_csram_lut_locked();
		len = scnprintf(buf, PAGE_SIZE, "%s", opp_table_export);
	}

	mutex_unlock(&lock);
	return len;
}

static const struct kernel_param_ops opp_table_ops = { .get = get_opp_table };
static int opp_table_dummy;
module_param_cb(opp_table, &opp_table_ops, &opp_table_dummy, 0444);
MODULE_PARM_DESC(opp_table, "CPU OPP table (CPU:policy:freq_khz:volt_uv|...) — live CSRAM read");

static int get_raw_dump(char *buf, const struct kernel_param *kp)
{
	int len;

	mutex_lock(&lock);
	__scan_csram_lut_locked();
	len = scnprintf(buf, PAGE_SIZE, "%s", raw_dump);
	mutex_unlock(&lock);
	return len;
}

static const struct kernel_param_ops raw_dump_ops = { .get = get_raw_dump };
static int raw_dump_dummy;
module_param_cb(raw, &raw_dump_ops, &raw_dump_dummy, 0444);
MODULE_PARM_DESC(raw, "Raw hex dump of LUT entries — live CSRAM read");

/* ─── Scan CSRAM and Export CPU Freq Table ──────────────────────────────── */

static int set_apply(const char *val, const struct kernel_param *kp)
{
	int c;

	if (!csram_base) {
		pr_err("KPM_OC: CSRAM not mapped\n");
		return -ENOMEM;
	}

	mutex_lock(&lock);
	__scan_csram_lut_locked();

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int dom_base = domain_offsets[c];
		int entry_count = 0, i;
		unsigned int prev_freq = 0;

		for (i = 0; i < LUT_MAX_ENTRIES; i++) {
			u32 lut_val = readl_relaxed(csram_base + dom_base +
						    REG_FREQ_LUT +
						    i * LUT_ROW_SIZE);
			unsigned int freq_mhz = lut_val & LUT_FREQ_MASK;

			if (freq_mhz == 0 || freq_mhz == prev_freq)
				break;
			prev_freq = freq_mhz;
			entry_count++;
		}
		pr_info("KPM_OC: Cluster %s (policy%d): %d LUT entries at CSRAM+0x%x\n",
			cluster_names[c], cluster_policies[c],
			entry_count, dom_base);
	}

	mutex_unlock(&lock);
	return 0;
}

static const struct kernel_param_ops apply_ops = { .set = set_apply };
static int apply_dummy;
module_param_cb(apply, &apply_ops, &apply_dummy, 0220);
MODULE_PARM_DESC(apply, "Write 1 to scan CSRAM and export CPU OPP table");

/* ─── kallsyms utility (shared by CPU OC + GPU OC) ─────────────────────── */
typedef unsigned long (*kln_t)(const char *name);
static kln_t kln_func;

static int resolve_kallsyms(void)
{
	struct kprobe kp = { .symbol_name = "kallsyms_lookup_name" };
	int ret;

	if (kln_func)
		return 0;

	ret = register_kprobe(&kp);
	if (ret < 0) {
		pr_err("KPM_OC: kprobe register failed: %d\n", ret);
		return ret;
	}
	kln_func = (kln_t)kp.addr;
	unregister_kprobe(&kp);

	if (!kln_func) {
		pr_err("KPM_OC: kallsyms_lookup_name not found\n");
		return -ENOENT;
	}
	return 0;
}

/* cpufreq function pointers — resolved at runtime via kallsyms to avoid
 * hard symbol dependencies that can prevent module loading at early boot.
 */
typedef struct cpufreq_policy *(*cpufreq_cpu_get_t)(unsigned int cpu);
typedef void (*cpufreq_cpu_put_t)(struct cpufreq_policy *policy);

static cpufreq_cpu_get_t fn_cpufreq_cpu_get;
static cpufreq_cpu_put_t fn_cpufreq_cpu_put;

/*
 * Vendor freq_qos constraint updaters — resolved via kallsyms.
 *
 * powerhal_cpu_ctrl + touch_boost share a FREQ_QOS_MAX constraint per cluster
 * via a global freq_max_request[], initialised to the stock max frequency.
 * update_userlimit_cpufreq_max(cluster_idx, freq_khz) updates that constraint.
 *
 * mtk_fpsgo adds its own FREQ_QOS_MAX constraint per cluster via fbt_cpu_rq[],
 * also initialised to the stock max.  fbt_cpu_rq is a pointer-to-array where
 * each element (struct freq_qos_request) has stride 64 bytes.
 *
 * Without updating both, scaling_max_freq is clamped to the stock maximum
 * even when cpuinfo.max_freq has been raised by the OC patch.
 */
typedef void (*update_userlimit_max_t)(int cluster, int freq);
typedef int  (*freq_qos_update_req_t)(void *req, int new_value);

static update_userlimit_max_t  fn_update_userlimit_max;
static freq_qos_update_req_t  fn_freq_qos_update_req;
static unsigned long           fbt_cpu_rq_sym;   /* &fbt_cpu_rq (pointer) */
typedef s32  (*freq_qos_read_value_t)(void *qos, int type);

static freq_qos_read_value_t   fn_freq_qos_read_value;
static unsigned long           pt_policy_list_sym; /* mtk_cpu_power_throttling list head */
#define FBT_RQ_STRIDE 64  /* bytes per cluster in fbt_cpu_rq array; confirmed by lsl #6 in disasm */

/* Reverse-engineered from mtk_cpu_power_throttling callbacks:
 * list node at +0x60, freq_qos_request at +0x18 (node - 0x48).
 */
#define CPU_PT_NODE_TO_REQ_OFF 0x48
#define CPU_PT_NODE_TO_FLAG_OFF 0x60

/* Periodic relift worker — re-lifts vendor freq_qos MAX constraints that
 * get re-asserted by powerhal, fpsgo, battery throttling, etc.
 */
#define RELIFT_INTERVAL_MS 500
static struct delayed_work cpu_oc_relift_dwork;
static bool cpu_oc_relift_active;
static unsigned int cpu_oc_targets[NUM_CLUSTERS]; /* per-cluster OC target KHz, 0=off */
/* Cached &policy->constraints per cluster for kprobe interception */
static struct freq_constraints *cluster_qos_ptr[NUM_CLUSTERS];

static void __nocfi cpu_oc_relift_work_fn(struct work_struct *work);

/* ─── PLL Direct Access state (read-only diagnostics) ─── */
static void __iomem *mcu_pll_base;

/* Forward declarations for CPU voltage override (defined after GPU section) */
static unsigned int cpu_volt_ov[NUM_CLUSTERS][LUT_MAX_ENTRIES];
static int          cpu_volt_ov_count;

static void __nocfi resolve_cpufreq_symbols(void)
{
	if (!kln_func)
		return;
	if (!fn_cpufreq_cpu_get)
		fn_cpufreq_cpu_get =
			(cpufreq_cpu_get_t)kln_func("cpufreq_cpu_get");
	if (!fn_cpufreq_cpu_put)
		fn_cpufreq_cpu_put =
			(cpufreq_cpu_put_t)kln_func("cpufreq_cpu_put");
	if (!fn_update_userlimit_max)
		fn_update_userlimit_max =
			(update_userlimit_max_t)kln_func("update_userlimit_cpufreq_max");
	if (!fn_freq_qos_update_req)
		fn_freq_qos_update_req =
			(freq_qos_update_req_t)kln_func("freq_qos_update_request");
	if (!fbt_cpu_rq_sym)
		fbt_cpu_rq_sym = kln_func("fbt_cpu_rq");
	if (!fn_freq_qos_read_value)
		fn_freq_qos_read_value =
			(freq_qos_read_value_t)kln_func("freq_qos_read_value");
	if (!pt_policy_list_sym)
		pt_policy_list_sym = kln_func("pt_policy_list");
}

/* ─── CPU OC via CSRAM LUT[0] patch + cpufreq policy update ─────────────── */
/*
 * For each cluster (L/B/P), if cpu_oc_X_freq is set to a nonzero value:
 *   1. Patches CSRAM LUT[0] (the hardware performance-controller table) with
 *      the new freq (MHz) and volt (µV), keeping the original gear selector
 *      bits [30:29]. Encoding: bits[11:0]=freq_MHz, bits[28:12]=volt_uv/10.
 *   2. Updates the Linux cpufreq policy (freq_table max entry, policy->max,
 *      cpuinfo.max_freq) so the governor can schedule the new maximum.
 */
#define CPU_RESULT_BUF_SIZE 512

static unsigned int cpu_oc_l_freq;
module_param(cpu_oc_l_freq, uint, 0644);
MODULE_PARM_DESC(cpu_oc_l_freq, "L cluster (policy0) OC target in KHz (0=disabled)");

static unsigned int cpu_oc_l_volt;
module_param(cpu_oc_l_volt, uint, 0644);
MODULE_PARM_DESC(cpu_oc_l_volt, "L cluster OC volt in µV (0=keep original)");

static unsigned int cpu_oc_b_freq;
module_param(cpu_oc_b_freq, uint, 0644);
MODULE_PARM_DESC(cpu_oc_b_freq, "B cluster (policy4) OC target in KHz (0=disabled)");

static unsigned int cpu_oc_b_volt;
module_param(cpu_oc_b_volt, uint, 0644);
MODULE_PARM_DESC(cpu_oc_b_volt, "B cluster OC volt in µV (0=keep original)");

static unsigned int cpu_oc_p_freq;
module_param(cpu_oc_p_freq, uint, 0644);
MODULE_PARM_DESC(cpu_oc_p_freq, "P cluster (policy7) OC target in KHz (0=disabled)");

static unsigned int cpu_oc_p_volt;
module_param(cpu_oc_p_volt, uint, 0644);
MODULE_PARM_DESC(cpu_oc_p_volt, "P cluster OC volt in µV (0=keep original)");

static char cpu_oc_result[CPU_RESULT_BUF_SIZE];
module_param_string(cpu_oc_result, cpu_oc_result, sizeof(cpu_oc_result), 0444);
MODULE_PARM_DESC(cpu_oc_result, "Result of last CPU OC patch (read-only)");

/* ─── PLL Direct Access Functions ───────────────────────────────────────── */

/* Calculate PCW (DDS) for a target frequency in MHz.
 * Formula: PCW = freq_mhz × 2^PCW_FRAC_BITS / FIN
 *          freq = FIN × PCW / 2^PCW_FRAC_BITS / POSTDIV
 * Assumes POSTDIV = 1 (appropriate for frequencies > 1.5 GHz).
 */
static u32 freq_to_dds(unsigned int freq_mhz)
{
	return (u32)((u64)freq_mhz * (1U << PLL_PCW_FRAC_BITS) / PLL_FIN_MHZ)
	       & PLL_DDS_MASK;
}

/* Convert PLL CON1 register value back to frequency in MHz */
static unsigned int con1_to_freq_mhz(u32 con1)
{
	u32 pcw = con1 & PLL_DDS_MASK;
	unsigned int postdiv_pow = (con1 & PLL_POSTDIV_MASK) >> PLL_POSTDIV_SHIFT;
	unsigned int postdiv = 1U << postdiv_pow;

	if (postdiv == 0)
		postdiv = 1;
	return (unsigned int)((u64)PLL_FIN_MHZ * pcw / (1U << PLL_PCW_FRAC_BITS)
			      / postdiv);
}

/* ─── PLL Register Dump (diagnostic, read-only) ─────────────────────────── */

#define PLL_DUMP_BUF_SIZE 2048
static char pll_dump_buf[PLL_DUMP_BUF_SIZE];
module_param_string(pll_dump, pll_dump_buf, sizeof(pll_dump_buf), 0444);
MODULE_PARM_DESC(pll_dump, "PLL register dump (read-only)");

static int set_pll_dump(const char *val, const struct kernel_param *kp)
{
	int c, pos = 0;

	memset(pll_dump_buf, 0, sizeof(pll_dump_buf));

	if (!mcu_pll_base) {
		snprintf(pll_dump_buf, PLL_DUMP_BUF_SIZE, "NOT_MAPPED");
		return 0;
	}

	for (c = 0; c < NUM_CLUSTERS; c++) {
		void __iomem *pll = mcu_pll_base + pll_offsets[c];
		void __iomem *fh = pll + FHCTL_OFF;
		u32 con0 = readl(pll + PLL_CON0);
		u32 con1 = readl(pll + PLL_CON1);
		u32 con2 = readl(pll + PLL_CON2);
		u32 con3 = readl(pll + PLL_CON3);
		u32 fh_cfg = readl(fh + REG_FHCTL_CFG);
		u32 fh_dds = readl(fh + REG_FHCTL_DDS);
		u32 fh_dvfs = readl(fh + REG_FHCTL_DVFS);
		u32 fh_mon = readl(fh + REG_FHCTL_MON);
		u32 fh_updnlmt = readl(fh + REG_FHCTL_UPDNLMT);
		unsigned int freq_mhz = con1_to_freq_mhz(con1);
		unsigned int mon_freq = con1_to_freq_mhz(fh_mon);

		pos += snprintf(pll_dump_buf + pos, PLL_DUMP_BUF_SIZE - pos,
				"[%s@0x%03x] CON0=%08x CON1=%08x(%uMHz) "
				"CON2=%08x CON3=%08x "
				"FH:CFG=%08x DDS=%08x DVFS=%08x MON=%08x(%uMHz) "
				"UPD=%08x | ",
				cluster_names[c], pll_offsets[c],
				con0, con1, freq_mhz,
				con2, con3,
				fh_cfg, fh_dds, fh_dvfs, fh_mon, mon_freq,
				fh_updnlmt);
	}

	/* Also dump raw words at PLL base for first cluster to find real regs */
	{
		void __iomem *base = mcu_pll_base + pll_offsets[2]; /* P cluster */
		int i;

		pos += snprintf(pll_dump_buf + pos, PLL_DUMP_BUF_SIZE - pos,
				"[P raw 0x00-0x3C] ");
		for (i = 0; i < 16 && pos < PLL_DUMP_BUF_SIZE - 12; i++)
			pos += snprintf(pll_dump_buf + pos,
					PLL_DUMP_BUF_SIZE - pos, "%08x ",
					readl(base + i * 4));
	}

	pr_info("KPM_OC: PLL dump: %s\n", pll_dump_buf);
	return 0;
}

static const struct kernel_param_ops pll_dump_ops = { .set = set_pll_dump };
static int pll_dump_dummy;
module_param_cb(pll_scan, &pll_dump_ops, &pll_dump_dummy, 0220);
MODULE_PARM_DESC(pll_scan, "Write 1 to dump PLL registers");

/* ─── CSRAM perf_state diagnostic ───────────────────────────────────────── */
/*
 * Read REG_FREQ_PERF_STATE (offset 0x88) from each CSRAM domain, and also
 * read back LUT[0] freq bits, to diagnose whether the governor-selected
 * index matches what HW reports and whether LUT patching is effective.
 *
 * Also reads the "csram_sys" region (offset 0x8C = REG_FREQ_HW_STATE) which
 * vendor mtk_cpufreq_hw may use for actual freq readback.
 */
#define REG_PERF_STATE_OFF  0x88
#define REG_HW_STATE_OFF    0x8C
#define PERF_DIAG_BUF_SIZE  2048
static char perf_diag_buf[PERF_DIAG_BUF_SIZE];
module_param_string(perf_diag, perf_diag_buf, sizeof(perf_diag_buf), 0444);
MODULE_PARM_DESC(perf_diag, "CSRAM perf_state diagnostic (read-only)");

static int set_perf_diag(const char *val, const struct kernel_param *kp)
{
	int c, pos = 0;

	memset(perf_diag_buf, 0, sizeof(perf_diag_buf));

	if (!csram_base) {
		snprintf(perf_diag_buf, PERF_DIAG_BUF_SIZE, "NOT_MAPPED");
		return 0;
	}

	for (c = 0; c < NUM_CLUSTERS; c++) {
		void __iomem *dom = csram_base + domain_offsets[c];
		u32 perf_state = readl(dom + REG_PERF_STATE_OFF);
		u32 hw_state   = readl(dom + REG_HW_STATE_OFF);
		u32 lut0_raw   = readl(dom + REG_FREQ_LUT);
		u32 lut0_freq  = lut0_raw & LUT_FREQ_MASK;
		int i;

		pos += snprintf(perf_diag_buf + pos, PERF_DIAG_BUF_SIZE - pos,
				"[%s] perf_idx=%u hw_state=0x%08x "
				"lut0=0x%08x(%uMHz)",
				cluster_names[c], perf_state, hw_state,
				lut0_raw, lut0_freq);

		/* Also read +0x90..+0xBF (32 bytes after hw_state) for vendor extras */
		pos += snprintf(perf_diag_buf + pos, PERF_DIAG_BUF_SIZE - pos,
				" extra[0x90-0xAC]=");
		for (i = 0; i < 8 && pos < PERF_DIAG_BUF_SIZE - 12; i++)
			pos += snprintf(perf_diag_buf + pos,
					PERF_DIAG_BUF_SIZE - pos, "%08x ",
					readl(dom + 0x90 + i * 4));

		pos += snprintf(perf_diag_buf + pos, PERF_DIAG_BUF_SIZE - pos, "| ");
	}

	pr_info("KPM_OC: perf_diag: %s\n", perf_diag_buf);
	return 0;
}

static const struct kernel_param_ops perf_diag_ops = { .set = set_perf_diag };
static int perf_diag_dummy;
module_param_cb(perf_scan, &perf_diag_ops, &perf_diag_dummy, 0220);
MODULE_PARM_DESC(perf_scan, "Write 1 to dump CSRAM perf_state diagnostics");

/* ─── MCUPM SRAM Frequency Table Scanner ────────────────────────────────── */
/*
 * MCUPM firmware has its own frequency table in SRAM (0x0C070000, 320KB).
 * The encrypted firmware is decrypted at boot; runtime SRAM contains the live
 * data including DDS/PCW tables that MCUPM uses for DVFS.
 *
 * We search for known stock max DDS values to locate the table, then patch
 * with OC target DDS values.
 */
#define MCUPM_SRAM_PHYS_BASE 0x0C070000UL
#define MCUPM_SRAM_PHYS_SIZE 0x00050000UL  /* 320KB */
#define MCUPM_DRAM_PHYS_BASE 0x7FD00000UL  /* MCUPM reserved DRAM */
#define MCUPM_DRAM_PHYS_SIZE 0x00100000UL  /* 1MB */

static void __iomem *mcupm_sram_base;
static void __iomem *mcupm_dram_base;

/* Known stock max frequencies (MHz) -> expected DDS values at POSTDIV=1 */
static const struct {
	const char *name;
	unsigned int freq_mhz;
	u32 dds;  /* PCW = freq * 2^14 / 26 */
} stock_freq_table[] = {
	{ "P-max", 3350, 0 },  /* filled at runtime */
	{ "B-max", 3200, 0 },
	{ "L-max", 2200, 0 },
};

#define MCUPM_SCAN_BUF_SIZE 4096
static char mcupm_scan_buf[MCUPM_SCAN_BUF_SIZE];
module_param_string(mcupm_scan, mcupm_scan_buf, sizeof(mcupm_scan_buf), 0444);
MODULE_PARM_DESC(mcupm_scan, "MCUPM SRAM scan results (read-only)");

static int set_mcupm_scan(const char *val, const struct kernel_param *kp)
{
	int pos = 0;
	u32 i;
	u32 sram_words;
	u32 dds_p, dds_b, dds_l;

	memset(mcupm_scan_buf, 0, sizeof(mcupm_scan_buf));

	if (!mcupm_sram_base) {
		snprintf(mcupm_scan_buf, MCUPM_SCAN_BUF_SIZE, "NOT_MAPPED");
		return 0;
	}

	/* Expected DDS values for stock max frequencies */
	dds_p = freq_to_dds(3350); /* P cluster: 3350 MHz */
	dds_b = freq_to_dds(3200); /* B cluster: 3200 MHz */
	dds_l = freq_to_dds(2200); /* L cluster: 2200 MHz */

	pos += snprintf(mcupm_scan_buf + pos, MCUPM_SCAN_BUF_SIZE - pos,
			"search: P=%uMHz(0x%06x) B=%uMHz(0x%06x) L=%uMHz(0x%06x) | ",
			3350, dds_p, 3200, dds_b, 2200, dds_l);

	sram_words = MCUPM_SRAM_PHYS_SIZE / 4;

	/* Hex dump first 32 words of SRAM */
	pos += snprintf(mcupm_scan_buf + pos, MCUPM_SCAN_BUF_SIZE - pos,
			"SRAM[0x00-0x7C]: ");
	for (i = 0; i < 32 && pos < MCUPM_SCAN_BUF_SIZE - 12; i++) {
		u32 w = readl(mcupm_sram_base + i * 4);
		pos += snprintf(mcupm_scan_buf + pos,
				MCUPM_SCAN_BUF_SIZE - pos, "%08x ", w);
	}
	pos += snprintf(mcupm_scan_buf + pos, MCUPM_SCAN_BUF_SIZE - pos, "| ");

	/* Count non-zero words and show stats */
	{
		u32 nonzero = 0;
		u32 first_nz_off = 0xFFFFFFFF;
		for (i = 0; i < sram_words; i++) {
			u32 w = readl(mcupm_sram_base + i * 4);
			if (w != 0) {
				nonzero++;
				if (first_nz_off == 0xFFFFFFFF)
					first_nz_off = i * 4;
			}
		}
		pos += snprintf(mcupm_scan_buf + pos, MCUPM_SCAN_BUF_SIZE - pos,
				"nonzero=%u/%u first@0x%x | ",
				nonzero, sram_words, first_nz_off);
	}

	/* Search for DDS values (22-bit PCW in any word) */
	for (i = 0; i < sram_words && pos < MCUPM_SCAN_BUF_SIZE - 80; i++) {
		u32 w = readl(mcupm_sram_base + i * 4);
		u32 masked = w & PLL_DDS_MASK;

		if (masked == dds_p || masked == dds_b || masked == dds_l) {
			const char *which = (masked == dds_p) ? "P" :
					    (masked == dds_b) ? "B" : "L";
			pos += snprintf(mcupm_scan_buf + pos,
					MCUPM_SCAN_BUF_SIZE - pos,
					"DDS:%s@0x%05x=0x%08x ", which, i * 4, w);
		}

		/* Also search for MHz values (small integers 2000-4000) */
		if ((w >= 2000 && w <= 4000) &&
		    (w == 3350 || w == 3200 || w == 2200 ||
		     w == 2000 || w == 2600 || w == 2800 || w == 3000)) {
			pos += snprintf(mcupm_scan_buf + pos,
					MCUPM_SCAN_BUF_SIZE - pos,
					"MHz:%u@0x%05x ", w, i * 4);
		}

		/* Search for KHz values */
		if (w == 3350000 || w == 3200000 || w == 2200000 ||
		    w == 3000000 || w == 2800000 || w == 2600000) {
			pos += snprintf(mcupm_scan_buf + pos,
					MCUPM_SCAN_BUF_SIZE - pos,
					"KHz:%u@0x%05x ", w, i * 4);
		}
	}

	if (pos == 0)
		snprintf(mcupm_scan_buf, MCUPM_SCAN_BUF_SIZE,
			 "NO_MATCHES in %u words", sram_words);

	/* Also scan MCUPM reserved DRAM (0x7FD00000, 1MB) */
	if (mcupm_dram_base) {
		u32 dram_words = MCUPM_DRAM_PHYS_SIZE / 4;
		u32 dram_nonzero = 0;

		pos += snprintf(mcupm_scan_buf + pos, MCUPM_SCAN_BUF_SIZE - pos,
				"DRAM: ");

		for (i = 0; i < dram_words && pos < MCUPM_SCAN_BUF_SIZE - 80; i++) {
			u32 w = readl(mcupm_dram_base + i * 4);
			u32 masked = w & PLL_DDS_MASK;

			if (w != 0)
				dram_nonzero++;

			if (masked == dds_p || masked == dds_b || masked == dds_l) {
				const char *which = (masked == dds_p) ? "P" :
						    (masked == dds_b) ? "B" : "L";
				pos += snprintf(mcupm_scan_buf + pos,
						MCUPM_SCAN_BUF_SIZE - pos,
						"DDS:%s@D+0x%05x=0x%08x ",
						which, i * 4, w);
			}

			/* MHz values */
			if (w == 3350 || w == 3200 || w == 2200 ||
			    w == 3000 || w == 2800 || w == 2600 || w == 2000)
				pos += snprintf(mcupm_scan_buf + pos,
						MCUPM_SCAN_BUF_SIZE - pos,
						"MHz:%u@D+0x%05x ", w, i * 4);

			/* KHz values */
			if (w == 3350000 || w == 3200000 || w == 2200000 ||
			    w == 3000000 || w == 2800000 || w == 2600000 ||
			    w == 2000000 || w == 1800000 || w == 1500000)
				pos += snprintf(mcupm_scan_buf + pos,
						MCUPM_SCAN_BUF_SIZE - pos,
						"KHz:%u@D+0x%05x ", w, i * 4);

			/* 10KHz values */
			if (w == 335000 || w == 320000 || w == 220000 ||
			    w == 300000 || w == 280000 || w == 260000)
				pos += snprintf(mcupm_scan_buf + pos,
						MCUPM_SCAN_BUF_SIZE - pos,
						"10K:%u@D+0x%05x ", w, i * 4);
		}

		pos += snprintf(mcupm_scan_buf + pos, MCUPM_SCAN_BUF_SIZE - pos,
				"dram_nz=%u/%u | ", dram_nonzero, dram_words);
	}

	pr_info("KPM_OC: MCUPM scan: %s\n", mcupm_scan_buf);
	return 0;
}

static const struct kernel_param_ops mcupm_scan_ops = { .set = set_mcupm_scan };
static int mcupm_scan_dummy;
module_param_cb(mcupm_scan_trigger, &mcupm_scan_ops, &mcupm_scan_dummy, 0220);
MODULE_PARM_DESC(mcupm_scan_trigger, "Write 1 to scan MCUPM SRAM for freq tables");

/* Hexdump a region of MCUPM DRAM: write "offset" to trigger,
 * reads 256 bytes at that offset within DRAM region.
 */
#define MCUPM_HEX_BUF_SIZE 2048
static char mcupm_hex_buf[MCUPM_HEX_BUF_SIZE];
module_param_string(mcupm_hex, mcupm_hex_buf, sizeof(mcupm_hex_buf), 0444);
MODULE_PARM_DESC(mcupm_hex, "MCUPM DRAM hex dump at given offset (read-only)");

static int set_mcupm_hexdump(const char *val, const struct kernel_param *kp)
{
	unsigned long offset = 0;
	int pos = 0, i;
	void __iomem *base;
	unsigned long max_size;

	memset(mcupm_hex_buf, 0, sizeof(mcupm_hex_buf));

	if (kstrtoul(val, 0, &offset))
		return -EINVAL;

	/* Auto-select SRAM or DRAM based on offset */
	if (offset < MCUPM_SRAM_PHYS_SIZE && mcupm_sram_base) {
		base = mcupm_sram_base;
		max_size = MCUPM_SRAM_PHYS_SIZE;
		pos += snprintf(mcupm_hex_buf + pos, MCUPM_HEX_BUF_SIZE - pos,
				"SRAM+0x%lx: ", offset);
	} else if (mcupm_dram_base) {
		if (offset >= MCUPM_SRAM_PHYS_SIZE)
			offset -= MCUPM_SRAM_PHYS_SIZE; /* allow absolute-ish offsets */
		base = mcupm_dram_base;
		max_size = MCUPM_DRAM_PHYS_SIZE;
		pos += snprintf(mcupm_hex_buf + pos, MCUPM_HEX_BUF_SIZE - pos,
				"DRAM+0x%lx: ", offset);
	} else {
		snprintf(mcupm_hex_buf, MCUPM_HEX_BUF_SIZE, "NOT_MAPPED");
		return 0;
	}

	if (offset >= max_size) {
		snprintf(mcupm_hex_buf, MCUPM_HEX_BUF_SIZE, "OUT_OF_RANGE");
		return 0;
	}

	/* Dump 64 words (256 bytes) as hex */
	for (i = 0; i < 64 && (offset + i * 4) < max_size &&
	     pos < MCUPM_HEX_BUF_SIZE - 12; i++) {
		u32 w = readl(base + offset + i * 4);
		pos += snprintf(mcupm_hex_buf + pos,
				MCUPM_HEX_BUF_SIZE - pos, "%08x ", w);
	}

	return 0;
}

static const struct kernel_param_ops mcupm_hex_ops = { .set = set_mcupm_hexdump };
static int mcupm_hex_dummy;
module_param_cb(mcupm_hexdump, &mcupm_hex_ops, &mcupm_hex_dummy, 0220);
MODULE_PARM_DESC(mcupm_hexdump, "Hex dump MCUPM memory: write offset, read mcupm_hex");

/* MCUPM DRAM write: "offset,value_hex" (offset within DRAM region) */
#define MCUPM_WRITE_BUF_SIZE 256
static char mcupm_write_buf[MCUPM_WRITE_BUF_SIZE];
module_param_string(mcupm_write_result, mcupm_write_buf, sizeof(mcupm_write_buf), 0444);

static int set_mcupm_write(const char *val, const struct kernel_param *kp)
{
	unsigned long offset = 0, value = 0;
	char tmp[64], *comma;
	u32 old_val;

	memset(mcupm_write_buf, 0, sizeof(mcupm_write_buf));

	if (!mcupm_dram_base) {
		snprintf(mcupm_write_buf, MCUPM_WRITE_BUF_SIZE, "ERR: DRAM not mapped");
		return -ENODEV;
	}

	strscpy(tmp, val, sizeof(tmp));
	comma = strchr(tmp, ',');
	if (!comma) {
		snprintf(mcupm_write_buf, MCUPM_WRITE_BUF_SIZE, "ERR: format offset,value");
		return -EINVAL;
	}
	*comma = '\0';
	if (kstrtoul(tmp, 0, &offset) || kstrtoul(comma + 1, 0, &value))
		return -EINVAL;

	if (offset >= MCUPM_DRAM_PHYS_SIZE || (offset & 3)) {
		snprintf(mcupm_write_buf, MCUPM_WRITE_BUF_SIZE,
			 "ERR: offset 0x%lx out of range or unaligned", offset);
		return -EINVAL;
	}

	old_val = readl(mcupm_dram_base + offset);
	writel((u32)value, mcupm_dram_base + offset);

	snprintf(mcupm_write_buf, MCUPM_WRITE_BUF_SIZE,
		 "DRAM+0x%lx: 0x%08x → 0x%08lx",
		 offset, old_val, value);

	pr_info("KPM_OC: DRAM write: offset=0x%lx old=0x%08x new=0x%08lx\n",
		offset, old_val, value);
	return 0;
}

static const struct kernel_param_ops mcupm_write_ops = { .set = set_mcupm_write };
static int mcupm_write_dummy;
module_param_cb(mcupm_dram_write, &mcupm_write_ops, &mcupm_write_dummy, 0220);
MODULE_PARM_DESC(mcupm_dram_write, "Write DRAM: offset,value_hex");

/* ─── Kernel virtual memory reader ──────────────────────────────────────── */
/*
 * Read 64 words (256 bytes) at a kernel virtual address.
 * Write hex address to kvm_read_trigger, read result from kvm_read.
 * Uses copy_from_kernel_nofault for safe access.
 */
#define KVM_READ_BUF_SIZE 2048
static char kvm_read_buf[KVM_READ_BUF_SIZE];
module_param_string(kvm_read, kvm_read_buf, sizeof(kvm_read_buf), 0444);
MODULE_PARM_DESC(kvm_read, "Kernel virtual memory read result (read-only)");

static int set_kvm_read(const char *val, const struct kernel_param *kp)
{
	unsigned long addr = 0;
	int pos = 0, i;

	memset(kvm_read_buf, 0, sizeof(kvm_read_buf));

	if (kstrtoul(val, 0, &addr))
		return -EINVAL;

	if (addr < PAGE_OFFSET) {
		snprintf(kvm_read_buf, KVM_READ_BUF_SIZE, "ERR: addr 0x%lx below PAGE_OFFSET", addr);
		return -EINVAL;
	}

	pos += snprintf(kvm_read_buf + pos, KVM_READ_BUF_SIZE - pos,
			"@%lx: ", addr);

	for (i = 0; i < 64 && pos < KVM_READ_BUF_SIZE - 12; i++) {
		u32 w;

		if (copy_from_kernel_nofault(&w, (void *)(addr + i * 4), 4)) {
			pos += snprintf(kvm_read_buf + pos,
					KVM_READ_BUF_SIZE - pos, "FAULT ");
			break;
		}
		pos += snprintf(kvm_read_buf + pos,
				KVM_READ_BUF_SIZE - pos, "%08x ", w);
	}

	return 0;
}

static const struct kernel_param_ops kvm_read_ops = { .set = set_kvm_read };
static int kvm_read_dummy;
module_param_cb(kvm_read_trigger, &kvm_read_ops, &kvm_read_dummy, 0220);
MODULE_PARM_DESC(kvm_read_trigger, "Read 256 bytes at kernel virtual address");

/* ─── fhctl hopping function caller ─────────────────────────────────────── */
/*
 * Call fhctl's mcupm_hopping_v1 to send DDS to MCUPM via IPI.
 * MCUPM will program the PLL using its privileged register access.
 *
 * From fhctl.ko disassembly, _array holds per-PLL data with ioremapped
 * register pointers and hopping function pointers.
 */
/*
 * mcupm_hopping_v1 actual signature (from disassembly):
 *   int mcupm_hopping_v1(void *pll_data, const char *domain_name,
 *                        int fh_id, unsigned int new_dds, int postdiv)
 * x0=pll_data, x1=domain, w2=fh_id, w3=new_dds, w4=postdiv
 * The function looks up domain_name in its internal name_table to compute
 * the MCUPM-internal fh_id, then sends IPI with cmd=0x1006.
 */
typedef int (*fhctl_hopping5_t)(void *pll_data, const char *domain,
			       int fh_id, unsigned int new_dds, int postdiv);
static fhctl_hopping5_t fn_mcupm_hopping;
static fhctl_hopping5_t fn_ap_hopping;
static unsigned long fhctl_array_sym; /* &_array in fhctl.ko */

/* Cached per-PLL data pointers for MCU PLLs (L, B, P) in _array */
static void *mcu_pll_data[NUM_CLUSTERS] __maybe_unused; /* from _array: per-domain pll_data */

static void resolve_fhctl_symbols(void)
{
	if (!kln_func)
		return;
	if (!fn_mcupm_hopping)
		fn_mcupm_hopping = (fhctl_hopping5_t)kln_func("mcupm_hopping_v1");
	if (!fn_ap_hopping)
		fn_ap_hopping = (fhctl_hopping5_t)kln_func("ap_hopping_v1");
	if (!fhctl_array_sym)
		fhctl_array_sym = kln_func("_array");
}

/* Diagnostic: dump fhctl _array contents */
#define FHCTL_DIAG_BUF_SIZE 2048
static char fhctl_diag_buf[FHCTL_DIAG_BUF_SIZE];
module_param_string(fhctl_diag, fhctl_diag_buf, sizeof(fhctl_diag_buf), 0444);
MODULE_PARM_DESC(fhctl_diag, "fhctl internal state dump (read-only)");

/*
 * fhctl _array points to a flat table of per-PLL records.
 * On this MT6897 build the records are 0x50 bytes apart in memory.
 * qword[0] is a dynamic private-data pointer in module memory and is the
 * best candidate for mcupm_hopping_v1 x0. Passing the record base itself
 * caused an alignment fault inside mcupm_hopping_v1.
 *
 * _array is a flat array of per-PLL records by DTS map order. For MCU
 * domains, each domain has exactly 1 PLL, so indices 9-13 map directly to
 * buspll/cpu0pll/cpu1pll/cpu2pll/ptppll.
 *
 * Domain assignment in fhctl_probe -> fhctl_init:
 *   map0 "top"  → ap      PLLs (mpll=fh2, mmpll=fh3, mainpll=fh4, etc)
 *   map5 "gpu0" → gpueb   PLLs
 *   map8 "gpu3" → gpueb   PLLs
 *   map9 "mcu0" → mcupm   buspll (ccipll)
 *   map10 "mcu1" → mcupm  cpu0pll (armpll_ll, L cluster)
 *   map11 "mcu2" → mcupm  cpu1pll (armpll_bl, B cluster)
 *   map12 "mcu3" → mcupm  cpu2pll (armpll_b,  P cluster)
 *   map13 "mcu4" → mcupm  ptppll
 *
 * _array indices correspond to the DTS map order. Within each domain,
 * PLLs are stored at _array[domain_start + fh_id]. For MCU single-PLL
 * domains, _array[domain_idx] = pll_data for that single PLL.
 *
 * From previous dump: _array[0] = ap domain first PLL (mpll, fh2 at ptr[0])
 * MCU PLLs should be at higher indices.
 */
static int set_fhctl_diag(const char *val, const struct kernel_param *kp)
{
	int pos = 0;
	int i;

	memset(fhctl_diag_buf, 0, sizeof(fhctl_diag_buf));

	if (resolve_kallsyms()) {
		snprintf(fhctl_diag_buf, FHCTL_DIAG_BUF_SIZE, "kallsyms_fail");
		return 0;
	}

	resolve_fhctl_symbols();

	pos += snprintf(fhctl_diag_buf + pos, FHCTL_DIAG_BUF_SIZE - pos,
			"mcupm_hop=%px ap_hop=%px _array=%px | ",
			fn_mcupm_hopping, fn_ap_hopping, (void *)fhctl_array_sym);

	/*
	 * _array is a POINTER variable in fhctl BSS — dereference once
	 * to get the actual array base.  Each per-PLL struct is 80 bytes
	 * (10 qwords).  14 PLLs total (from ctrl output).
	 */
#define FHCTL_STRUCT_SIZE   80  /* bytes per PLL entry */
#define FHCTL_STRUCT_QWORDS (FHCTL_STRUCT_SIZE / 8)
#define FHCTL_MAX_PLLS      14
	if (fhctl_array_sym) {
		u64 actual_base = *(u64 *)fhctl_array_sym;

		pos += snprintf(fhctl_diag_buf + pos, FHCTL_DIAG_BUF_SIZE - pos,
				"array_ptr=%px | ", (void *)actual_base);

		if (actual_base) {
			u64 *base64 = (u64 *)actual_base;

			for (i = 0; i < FHCTL_MAX_PLLS &&
			     pos < FHCTL_DIAG_BUF_SIZE - 200; i++) {
				u64 *pd = base64 + i * FHCTL_STRUCT_QWORDS;
				u32 perms = (u32)(pd[5] >> 32);
				u32 fh_id_val = (u32)pd[5];

				pos += snprintf(fhctl_diag_buf + pos,
					FHCTL_DIAG_BUF_SIZE - pos,
					"[%d]@%px hop=%px p=%x id=%u hdlr=%px | ",
					i, (void *)pd, (void *)pd[4],
					perms, fh_id_val, (void *)pd[9]);

				if (i >= 9 && pos < FHCTL_DIAG_BUF_SIZE - 180)
					pos += snprintf(fhctl_diag_buf + pos,
						FHCTL_DIAG_BUF_SIZE - pos,
						"q0=%px q2=%px q3=%px q7=%px q8=%px | ",
						(void *)pd[0], (void *)pd[2],
						(void *)pd[3], (void *)pd[7],
						(void *)pd[8]);
			}
		}

		if (pos < FHCTL_DIAG_BUF_SIZE - 40)
			pos += snprintf(fhctl_diag_buf + pos,
				FHCTL_DIAG_BUF_SIZE - pos, "END");
	}

	pr_info("KPM_OC: fhctl diag: %s\n", fhctl_diag_buf);
	return 0;
}

static const struct kernel_param_ops fhctl_diag_ops = { .set = set_fhctl_diag };
static int fhctl_diag_dummy;
module_param_cb(fhctl_scan, &fhctl_diag_ops, &fhctl_diag_dummy, 0220);
MODULE_PARM_DESC(fhctl_scan, "Write 1 to dump fhctl internal state");

/* ─── mcupm_hopping_v1 direct caller ─────────────────────────────────────
 *
 * Call mcupm_hopping_v1 directly with resolved pll_data from _array.
 * Format: "array_idx,fh_id,dds_hex[,postdiv]"
 * Example: "10,0,0x21a762,-1"  → _array[10], fh_id=0, dds=0x21a762, pdiv=-1
 *
 * Also supports "scan" to try calling with stock DDS on each non-NULL entry.
 */
#define MCUPM_HOP_BUF_SIZE 1024
static char mcupm_hop_buf[MCUPM_HOP_BUF_SIZE];
module_param_string(mcupm_hop_result, mcupm_hop_buf, sizeof(mcupm_hop_buf), 0444);
MODULE_PARM_DESC(mcupm_hop_result, "Result of mcupm_hopping_v1 call (read-only)");

static int __nocfi set_mcupm_hop(const char *val, const struct kernel_param *kp)
{
	char tmp[64], *p1, *p2, *p3;
	unsigned long arr_idx = 0, fh_id_l = 0, dds_l = 0;
	long postdiv_l = -1;
	u64 *pd;
	void *pll_data;
	void *priv_data;
	int ret;

	memset(mcupm_hop_buf, 0, sizeof(mcupm_hop_buf));

	if (!fn_mcupm_hopping || !fhctl_array_sym) {
		if (resolve_kallsyms())
			return -ENOENT;
		resolve_fhctl_symbols();
	}

	if (!fn_mcupm_hopping) {
		snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
			 "ERR: mcupm_hopping_v1 not found");
		return -ENOENT;
	}
	if (!fhctl_array_sym) {
		snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
			 "ERR: _array not found");
		return -ENOENT;
	}

	strscpy(tmp, val, sizeof(tmp));

	/* Parse "array_idx,fh_id,dds_hex[,postdiv]" */
	p1 = strchr(tmp, ',');
	if (!p1) {
		snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
			 "ERR: format: array_idx,fh_id,dds_hex[,postdiv]");
		return -EINVAL;
	}
	*p1++ = '\0';
	p2 = strchr(p1, ',');
	if (!p2) {
		snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
			 "ERR: format: array_idx,fh_id,dds_hex[,postdiv]");
		return -EINVAL;
	}
	*p2++ = '\0';
	p3 = strchr(p2, ',');
	if (p3) {
		*p3++ = '\0';
		if (kstrtol(p3, 0, &postdiv_l))
			return -EINVAL;
	}

	if (kstrtoul(tmp, 0, &arr_idx) || kstrtoul(p1, 0, &fh_id_l) ||
	    kstrtoul(p2, 0, &dds_l))
		return -EINVAL;

	if (arr_idx > 63 || dds_l > 0x3FFFFF) {
		snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
			 "ERR: arr_idx=%lu (max 63) dds=0x%lx (max 0x3FFFFF)",
			 arr_idx, dds_l);
		return -EINVAL;
	}

	/*
	 * _array is a pointer variable; dereference to get actual AP flat array.
	 * Each per-PLL record is 80 bytes (10 qwords):
	 *   arr[0..6]  = AP PLLs (mpll, mmpll, mainpll, msdcpll, adsppll, imgpll, tvdpll)
	 *   arr[7..8]  = GPU PLLs (mfg-ao-mfgpll, mfgsc-ao-mfgscpll)
	 *   arr[9..13] = MCU PLLs (buspll/mcu0, cpu0pll/mcu1, cpu1pll/mcu2,
	 *                          cpu2pll/mcu3=P-cluster, ptppll/mcu4)
	 *
	 * qword[9] (offset 0x48) = link_block ptr == Hdlr from fhctl debugfs ctrl.
	 * link_block[0] = ops_struct = actual priv_data (x0) for mcupm_hopping_v1.
	 *
	 * Verified from runtime analysis:
	 *   cpu2pll (arr[12]): link_block = 0xffffff80095f0380,
	 *                      ops_struct  = 0xffffff80095f07c0
	 */
	{
		u64 actual_base = *(u64 *)fhctl_array_sym;
		u64 link_block_addr;
		u8 *base8;

		if (!actual_base) {
			snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
				 "ERR: _array pointer is NULL");
			return -ENOENT;
		}
		base8 = (u8 *)actual_base;
		pd = (u64 *)(base8 + arr_idx * 80);
		pll_data = (void *)pd;

		/* pd[9] = link_block ptr (= Hdlr column in fhctl ctrl debugfs).
		 * Dereference once more to get ops_struct = actual priv_data. */
		link_block_addr = pd[9];
		if (!link_block_addr) {
			snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
				 "ERR: _array[%lu] link_block is NULL", arr_idx);
			return -ENOENT;
		}
		priv_data = (void *)(*(u64 *)link_block_addr);
	}
	if (!pll_data || !priv_data) {
		snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
			 "ERR: _array[%lu] record=%px priv=%px", arr_idx,
			 pll_data, priv_data);
		return -ENOENT;
	}

	pr_info("KPM_OC: mcupm_hop: arr[%lu]=%px priv=%px fh_id=%lu dds=0x%06lx pdiv=%ld\n",
		arr_idx, pll_data, priv_data, fh_id_l, dds_l, postdiv_l);

	/* Domain name pointer for mcupm_hopping_v1 arg1.
	 *
	 * find_entry() uses POINTER comparison (not strcmp) to search the
	 * ops_struct domain_head list.  We must pass the exact fhctl-internal
	 * name pointer from domain_head[k].name_ptr (k = arr_idx - 9 for MCU).
	 *
	 * ops_struct layout (relevant fields):
	 *   [0x00] = pll_data ptr
	 *   [0x08] = spinlock
	 *   [0x10] = hw_ctx
	 *   [0x28] = domain_head ptr → array of {name_ptr, fh_id_base} entries
	 *
	 * For AP PLLs (arr_idx 0-8):  domain_head = NULL → arg1 unused.
	 * For MCU PLLs (arr_idx 9-13): domain_head != NULL → must use exact ptr.
	 * Entry ordering in domain_head matches arr_idx order: k = arr_idx - 9.
	 */
	{
		u64 *ops = (u64 *)priv_data;
		u64  dom_head_va = ops[5];  /* ops_struct[0x28] = ops[5] */
		const char *domain;

		if (dom_head_va && arr_idx >= 9 && arr_idx <= 13) {
			int k = (int)arr_idx - 9;
			u64 *dom_head = (u64 *)dom_head_va;
			/* Each entry is 16 bytes: {name_ptr, fh_id_base} */
			domain = (const char *)dom_head[k * 2];
			pr_info("KPM_OC: mcupm_hop dom_head=%px k=%d name_ptr=%px\n",
				(void *)dom_head_va, k, domain);
		} else {
			/* AP PLL or no domain_head: fallback string (not searched) */
			domain = "top";
		}

		ret = fn_mcupm_hopping(priv_data, domain, (int)fh_id_l,
				       (unsigned int)dds_l, (int)postdiv_l);
	}

	snprintf(mcupm_hop_buf, MCUPM_HOP_BUF_SIZE,
		 "arr[%lu]=%px link_blk=%px ops=%px fh=%lu dds=0x%06lx pdiv=%ld ret=%d",
		 arr_idx, pll_data, (void *)pd[9], priv_data,
		 fh_id_l, dds_l, postdiv_l, ret);

	pr_info("KPM_OC: mcupm_hop result: %s\n", mcupm_hop_buf);
	return 0;
}

static const struct kernel_param_ops mcupm_hop_ops = { .set = set_mcupm_hop };
static int mcupm_hop_dummy;
module_param_cb(mcupm_hop, &mcupm_hop_ops, &mcupm_hop_dummy, 0220);
MODULE_PARM_DESC(mcupm_hop, "Call mcupm_hopping_v1: array_idx,fh_id,dds[,postdiv]");

/* ─── CPU OC via CSRAM LUT[0] patch + cpufreq policy update ─────────────── */

static int __nocfi set_cpu_oc(const char *val, const struct kernel_param *kp)
{
	/* Indexed by cluster order: L=0, B=1, P=2 (matches cluster_policies[]) */
	static unsigned int * const tgt_freqs[] = {
		&cpu_oc_l_freq, &cpu_oc_b_freq, &cpu_oc_p_freq
	};
	static unsigned int * const tgt_volts[] = {
		&cpu_oc_l_volt, &cpu_oc_b_volt, &cpu_oc_p_volt
	};
	int c, pos = 0, any_patched = 0, any_target = 0;

	if (!csram_base) {
		pr_err("KPM_OC: CSRAM not mapped\n");
		return -ENOMEM;
	}

	/* Resolve cpufreq functions lazily via kallsyms */
	if (!resolve_kallsyms())
		resolve_cpufreq_symbols();

	mutex_lock(&lock);
	memset(cpu_oc_result, 0, sizeof(cpu_oc_result));

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int target_khz = *tgt_freqs[c];
		unsigned int target_uv  = *tgt_volts[c];
		unsigned int dom_base   = domain_offsets[c];
		u32 orig_lut, new_lut;
		unsigned int orig_freq_mhz, orig_volt_uv, new_freq_mhz;

		if (target_khz == 0)
			continue;

		/* Read current LUT[0] (highest OPP = index 0 in descending LUT) */
		orig_lut      = readl_relaxed(csram_base + dom_base + REG_FREQ_LUT);
		orig_freq_mhz = orig_lut & LUT_FREQ_MASK;
		orig_volt_uv  = LUT_VOLT_DECODE(orig_lut);

		if (orig_freq_mhz < 100 || orig_freq_mhz > 5000)
			pr_warn("KPM_OC: CPU %s LUT[0] unexpected freq=%u MHz (repairing)\n",
				cluster_names[c], orig_freq_mhz);

		new_freq_mhz = target_khz / 1000;
		if (new_freq_mhz < 100 || new_freq_mhz > 5000) {
			pr_warn("KPM_OC: CPU %s target %u MHz out of range\n",
				cluster_names[c], new_freq_mhz);
			pos += snprintf(cpu_oc_result + pos, CPU_RESULT_BUF_SIZE - pos,
					"%s=OOB,", cluster_names[c]);
			continue;
		}

		if (target_uv == 0)
			target_uv = orig_volt_uv; /* keep original voltage */

		/*
		 * Encode new LUT entry:
		 *   bits[30:29] = gear selector — keep from original
		 *   bits[28:12] = volt_uv / 10
		 *   bits[11:0]  = freq_MHz
		 */
		new_lut = (orig_lut & LUT_GEAR_MASK) |
			  (new_freq_mhz & LUT_FREQ_MASK) |
			  ((target_uv / 10) << 12);

		writel_relaxed(new_lut, csram_base + dom_base + REG_FREQ_LUT);
		wmb();

		pr_info("KPM_OC: CPU %s LUT[0]: %uMHz@%uµV -> %uMHz@%uµV (0x%08x->0x%08x)\n",
			cluster_names[c], orig_freq_mhz, orig_volt_uv,
			new_freq_mhz, target_uv, orig_lut, new_lut);

		pos += snprintf(cpu_oc_result + pos, CPU_RESULT_BUF_SIZE - pos,
				"%s:%u->%uKHz@%uuV,",
				cluster_names[c], orig_freq_mhz * 1000, target_khz, target_uv);
		any_patched++;

		/* Update OPP cache entry for LUT[0] (highest freq) */
		if (opp_cache_valid && c < NUM_CLUSTERS && opp_cache_count[c] > 0) {
			opp_cache[c][0].freq_khz = target_khz;
			opp_cache[c][0].volt_uv  = target_uv;
		}

		/* ── Update cpufreq policy so governor can target new max freq ── */
		{
			struct cpufreq_policy *policy = fn_cpufreq_cpu_get ?
				fn_cpufreq_cpu_get(cluster_policies[c]) : NULL;
			if (policy) {
				struct cpufreq_frequency_table *tbl = policy->freq_table;
				if (tbl) {
					/*
					 * Always update index 0 — it corresponds to
					 * LUT[0] (the highest OPP in descending LUT).
					 * Previous code searched for max freq entry,
					 * which broke on re-OC after underclocking
					 * (max shifted to a different index).
					 */
					tbl[0].frequency = target_khz;
				}
				policy->max                 = target_khz;
				policy->cpuinfo.max_freq    = target_khz;
				/*
				 * Update the user's own freq_qos MAX request
				 * (policy->max_freq_req) to the OC frequency.
				 * Without this, the freq_qos aggregate stays at the
				 * boot-time cpuinfo.max_freq (= stock max), clamping
				 * subsequent sysfs scaling_max_freq writes.
				 */
				if (fn_freq_qos_update_req && policy->max_freq_req)
					fn_freq_qos_update_req(
						policy->max_freq_req,
						target_khz);
				/* Diagnostic: read user_prio and aggregate BEFORE cpu_put */
				{
					s32 user_prio_before = -1;
					s32 agg_before = -1;

					if (policy->max_freq_req)
						user_prio_before =
							*(s32 *)((char *)policy->max_freq_req + 8);
					if (fn_freq_qos_read_value)
						agg_before = fn_freq_qos_read_value(
							&policy->constraints,
							2 /* FREQ_QOS_MAX */);
					pr_info("KPM_OC: CPU %s PRE-PUT: user_prio=%d agg_max=%d policy_max=%u\n",
						cluster_names[c], user_prio_before,
						agg_before, policy->max);
				}
				/* Cache constraints ptr for kprobe interception */
				WRITE_ONCE(cluster_qos_ptr[c], &policy->constraints);
				if (fn_cpufreq_cpu_put)
					fn_cpufreq_cpu_put(policy);
				pr_info("KPM_OC: CPU %s cpufreq policy updated to %uKHz (incl. user freq_qos)\n",
					cluster_names[c], target_khz);
			}
		}

		/* ── Lift vendor freq_qos MAX constraints to the OC frequency ── */
		/*
		 * powerhal_cpu_ctrl + touch_boost share freq_max_request[c]:
		 * update_userlimit_cpufreq_max(cluster_idx, target_khz)
		 */
		if (fn_update_userlimit_max) {
			fn_update_userlimit_max(c, target_khz);
			pr_info("KPM_OC: CPU %s powerhal freq_qos MAX -> %uKHz\n",
				cluster_names[c], target_khz);
		}

		/*
		 * mtk_fpsgo's fbt_cpu_rq[c] (stride 64 bytes):
		 * freq_qos_update_request(&fbt_cpu_rq[c], target_khz)
		 */
		if (fn_freq_qos_update_req && fbt_cpu_rq_sym) {
			void *rq_base = *(void **)fbt_cpu_rq_sym;
			if (rq_base) {
				void *rq = (char *)rq_base + c * FBT_RQ_STRIDE;
				int fret = fn_freq_qos_update_req(rq, target_khz);
				s32 agg_after = fn_freq_qos_read_value ?
					fn_freq_qos_read_value(
						*(void **)((char *)rq +
							   offsetof(struct freq_qos_request, qos)),
						2 /* FREQ_QOS_MAX */) : -999;
				pr_info("KPM_OC: CPU %s fpsgo freq_qos MAX -> %uKHz (ret=%d agg=%d)\n",
					cluster_names[c], target_khz, fret, agg_after);
			}
		}
	}

	/*
	 * mtk_cpu_power_throttling dynamically applies extra FREQ_QOS_MAX limits
	 * via pt_policy_list callbacks (low battery / over-current / battery %).
	 * Lift all active requests to INT_MAX so they don't silently clamp OC.
	 */
	if (fn_freq_qos_update_req && pt_policy_list_sym) {
		struct list_head *head = (struct list_head *)pt_policy_list_sym;
		struct list_head *pos;
		int n = 0;

		list_for_each(pos, head) {
			u32 active = *(u32 *)((char *)pos - CPU_PT_NODE_TO_FLAG_OFF);
			void *rq = (char *)pos - CPU_PT_NODE_TO_REQ_OFF;

			if (active == 0)
				continue;
			fn_freq_qos_update_req(rq, INT_MAX);
			n++;
		}
		if (n > 0)
			pr_info("KPM_OC: Lifted %d mtk_cpu_power_throttling freq_qos MAX requests to INT_MAX\n",
				n);
	}

	if (pos > 0 && cpu_oc_result[pos - 1] == ',')
		cpu_oc_result[pos - 1] = '\0';
	if (any_patched == 0 && pos == 0)
		snprintf(cpu_oc_result, CPU_RESULT_BUF_SIZE,
			 "NOOP:no cluster targets set");

	/* Record per-cluster OC targets and start the periodic relift worker */
	for (c = 0; c < NUM_CLUSTERS; c++) {
		cpu_oc_targets[c] = *tgt_freqs[c];
		if (*tgt_freqs[c] > 0)
			any_target = 1;
	}

	if (any_target && !cpu_oc_relift_active) {
		cpu_oc_relift_active = true;
		schedule_delayed_work(&cpu_oc_relift_dwork,
				      msecs_to_jiffies(RELIFT_INTERVAL_MS));
		pr_info("KPM_OC: periodic relift worker started (%d ms)\n",
			RELIFT_INTERVAL_MS);
	}

	mutex_unlock(&lock);
	return 0;
}

static const struct kernel_param_ops cpu_oc_ops = { .set = set_cpu_oc };
static int cpu_oc_dummy;
module_param_cb(cpu_oc_apply, &cpu_oc_ops, &cpu_oc_dummy, 0220);
MODULE_PARM_DESC(cpu_oc_apply, "Write 1 to patch CPU CSRAM LUT[0] + cpufreq policy max");

/* ─── kprobe: zero-latency freq_qos_update_request intercept ────────────── */
/*
 * Vendor modules call freq_qos_update_request() to re-assert stock MAX
 * constraints on the cpufreq policy (powerhal, fpsgo, battery throttling).
 * Instead of waiting up to RELIFT_INTERVAL_MS for the periodic worker to
 * notice and fix the value, we intercept every call and silently raise the
 * argument back to the OC target before the function executes.
 *
 * AArch64 calling convention: x0 = req (struct freq_qos_request *),
 *                              x1 = new_value (s32).
 * Safety: handler only reads ONCE variables; no spinlocks, no alloc.
 */
static int __nocfi freq_qos_kp_pre(struct kprobe *p, struct pt_regs *regs)
{
	struct freq_qos_request *req;
	s32 new_value;
	int c;

	req       = (struct freq_qos_request *)(uintptr_t)regs->regs[0];
	new_value = (s32)regs->regs[1];

	if (!req || new_value <= 0)
		return 0;

	/* Only intercept MAX constraint lowering */
	if (req->type != FREQ_QOS_MAX)
		return 0;

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int target = READ_ONCE(cpu_oc_targets[c]);
		struct freq_constraints *qp = READ_ONCE(cluster_qos_ptr[c]);

		if (target == 0 || !qp)
			continue;

		if (req->qos == qp && (unsigned int)new_value < target) {
			/* Override: caller will use OC target instead of stock value */
			regs->regs[1] = (u64)target;
			pr_info_ratelimited("KPM_OC: kp: intercepted c%d %d->%u\n",
					    c, new_value, target);
			return 0;
		}
		/* Debug: log requests that would lower this cluster but have wrong qos */
		if ((unsigned int)new_value < target && qp)
			pr_info_ratelimited("KPM_OC: kp: MISS c%d val=%d qos=%px expected=%px\n",
					    c, new_value, req->qos, qp);
	}
	return 0;
}

static struct kprobe freq_qos_kp = {
	.symbol_name = "freq_qos_update_request",
	.pre_handler = freq_qos_kp_pre,
};

/* kprobe: intercept update_userlimit_cpufreq_max (powerhal/touch_boost)
 * AArch64: x0 = cluster_idx (int), x1 = max_khz (unsigned int)
 */
static int __nocfi userlimit_kp_pre(struct kprobe *p, struct pt_regs *regs)
{
	int cluster = (int)regs->regs[0];
	unsigned int new_max = (unsigned int)regs->regs[1];
	unsigned int target;

	if (cluster < 0 || cluster >= NUM_CLUSTERS)
		return 0;

	target = READ_ONCE(cpu_oc_targets[cluster]);
	if (target > 0 && new_max < target)
		regs->regs[1] = (u64)target;

	return 0;
}

static struct kprobe userlimit_kp = {
	.symbol_name = "update_userlimit_cpufreq_max",
	.pre_handler = userlimit_kp_pre,
};

/* ─── MCUPM CSRAM countermeasure: event-driven voltage resync ───────────
 *
 * Root cause: MCUPM firmware runs on a dedicated MCU and has direct access
 * to CSRAM.  It continuously recalculates and overwrites LUT voltage fields
 * (freq bits[11:0] are preserved, but bits[28:12] = voltage are replaced).
 *
 * Fix: Kprobe on scmi_cpufreq_fast_switch (the actual DVFS path on this SoC,
 * since it uses scmi_cpufreq driver — not mtk-cpufreq-hw).  In the
 * pre-handler we re-write CSRAM LUT voltages and trigger PLL re-enforcement.
 */
static void __nocfi cpu_csram_resync_fast(void)
{
	int c;

	if (!csram_base)
		return;

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int dom_base = domain_offsets[c];
		unsigned int target_khz = READ_ONCE(cpu_oc_targets[c]);
		int i;

		/* Re-patch LUT[0] with OC freq + voltage if active */
		if (target_khz > 0) {
			static unsigned int * const tgt_volts[] = {
				&cpu_oc_l_volt, &cpu_oc_b_volt, &cpu_oc_p_volt
			};
			unsigned int oc_volt_uv = READ_ONCE(*tgt_volts[c]);
			if (oc_volt_uv > 0) {
				u32 cur = readl_relaxed(csram_base + dom_base +
						       REG_FREQ_LUT);
				u32 want = (cur & (LUT_GEAR_MASK | LUT_FREQ_MASK)) |
					   ((oc_volt_uv / 10) << 12);
				if (cur != want)
					writel_relaxed(want, csram_base + dom_base +
						       REG_FREQ_LUT);
			}
		}

		/* Re-patch per-LUT voltage overrides */
		if (cpu_volt_ov_count > 0) {
			for (i = 0; i < LUT_MAX_ENTRIES; i++) {
				unsigned int ov = cpu_volt_ov[c][i];
				u32 cur, patched;

				if (!ov)
					continue;

				cur = readl_relaxed(csram_base + dom_base +
						    REG_FREQ_LUT +
						    i * LUT_ROW_SIZE);
				patched = (cur & (LUT_GEAR_MASK | LUT_FREQ_MASK)) |
					  ((ov / 10) << 12);
				if (cur != patched)
					writel_relaxed(patched, csram_base +
						       dom_base + REG_FREQ_LUT +
						       i * LUT_ROW_SIZE);
			}
		}
	}

	/* No wmb() — writel_relaxed ordering is sufficient; the HW read
	 * follows the REG_FREQ_PERF_STATE write which acts as a barrier.
	 */
}

/* ─── SCMI capture buffer (shared by perf_level and dvfs_freq kprobes) ─ */
#define SCMI_CAP_BUF_SIZE 2048
static char scmi_cap_buf[SCMI_CAP_BUF_SIZE];
static int scmi_cap_pos;

/* Forward declarations for cpufreq kprobe counters */
static atomic_t scmi_cpufreq_hit_count = ATOMIC_INIT(0);
static u32 scmi_last_target_freq;

static int get_scmi_capture(char *buf, const struct kernel_param *kp)
{
	int len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "cpufreq_hits=%d last_target=%u\n",
			 atomic_read(&scmi_cpufreq_hit_count),
			 READ_ONCE(scmi_last_target_freq));
	if (scmi_cap_pos > 0)
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "perf: %s\n", scmi_cap_buf);
	else
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "perf: (no calls)\n");
	return len;
}

static const struct kernel_param_ops scmi_cap_ops = { .get = get_scmi_capture };
static int scmi_cap_dummy;
module_param_cb(scmi_capture, &scmi_cap_ops, &scmi_cap_dummy, 0444);
MODULE_PARM_DESC(scmi_capture, "Captured SCMI DVFS calls (read-only)");

/*
 * Kprobe pre-handler: fires on SCMI cpufreq DVFS transitions.
 * Resyncs CSRAM voltages and triggers PLL re-enforcement.
 * scmi_cpufreq_fast_switch: x0 = policy *, x1 = target_freq (unsigned int)
 * scmi_cpufreq_set_target:  x0 = policy *, x1 = target_freq (unsigned int)
 *
 * Uses only lock-free, IRQ-safe primitives.
 */
static int __nocfi scmi_cpufreq_kp_pre(struct kprobe *p,
					struct pt_regs *regs)
{
	unsigned int target_freq = (unsigned int)regs->regs[1];

	WRITE_ONCE(scmi_last_target_freq, target_freq);
	atomic_inc(&scmi_cpufreq_hit_count);

	cpu_csram_resync_fast();
	return 0;
}

static struct kprobe scmi_fast_switch_kp = {
	.symbol_name = "scmi_cpufreq_fast_switch",
	.pre_handler = scmi_cpufreq_kp_pre,
};

static struct kprobe scmi_set_target_kp = {
	.symbol_name = "scmi_cpufreq_set_target",
	.pre_handler = scmi_cpufreq_kp_pre,
};

/* ─── SCMI perf_level_set capture & override ────────────────────────────── */
/*
 * scmi_perf_level_set(handle, domain, level, poll)
 * AArch64: x0=handle, w1=domain, w2=level, w3=poll
 *
 * Captures perf_level values sent by the cpufreq driver to MCUPM.
 * If scmi_level_override is set for a domain, replaces the level.
 */

/* Per-domain level override: 0 = no override */
static u32 scmi_level_override[8]; /* max 8 domains */

static int __nocfi scmi_perf_kp_pre(struct kprobe *p, struct pt_regs *regs)
{
	unsigned int domain = (unsigned int)regs->regs[1];
	unsigned int level = (unsigned int)regs->regs[2];

	/* Capture */
	if (scmi_cap_pos > SCMI_CAP_BUF_SIZE - 60)
		scmi_cap_pos = 0;
	scmi_cap_pos += snprintf(scmi_cap_buf + scmi_cap_pos,
				 SCMI_CAP_BUF_SIZE - scmi_cap_pos,
				 "d%u:L%u ", domain, level);

	/* Override if set */
	if (domain < 8 && scmi_level_override[domain] != 0 &&
	    level == scmi_level_override[domain] - 1) {
		/* Override: when kernel sends the stock max level (target-1),
		 * replace with the OC level (target).  The -1/+0 encoding
		 * prevents infinite self-overrides. */
	}

	/* Direct override: replace level for domain if override > 0 */
	if (domain < 8 && scmi_level_override[domain] > 0) {
		regs->regs[2] = scmi_level_override[domain];
		pr_info_ratelimited("KPM_OC: SCMI override: d%u level %u→%u\n",
				    domain, level, scmi_level_override[domain]);
	}

	return 0;
}

static struct kprobe scmi_perf_level_kp = {
	.symbol_name = "scmi_perf_level_set",
	.pre_handler = scmi_perf_kp_pre,
};

/*
 * scmi_dvfs_freq_set(handle, domain, freq_hz, poll)
 * AArch64: x0=handle, w1=domain, x2=freq_hz (unsigned long), w3=poll
 * This is the function pointer target for perf_ops->freq_set.
 */
static int __nocfi scmi_dvfs_freq_kp_pre(struct kprobe *p, struct pt_regs *regs)
{
	unsigned int domain = (unsigned int)regs->regs[1];
	unsigned long freq_hz = (unsigned long)regs->regs[2];

	if (scmi_cap_pos > SCMI_CAP_BUF_SIZE - 60)
		scmi_cap_pos = 0;
	scmi_cap_pos += snprintf(scmi_cap_buf + scmi_cap_pos,
				 SCMI_CAP_BUF_SIZE - scmi_cap_pos,
				 "D%u:%luHz ", domain, freq_hz);
	return 0;
}

static struct kprobe scmi_dvfs_freq_kp = {
	.symbol_name = "scmi_dvfs_freq_set",
	.pre_handler = scmi_dvfs_freq_kp_pre,
};

/* Set SCMI level override: "domain,level"  (0 to clear) */
static int set_scmi_override(const char *val, const struct kernel_param *kp)
{
	unsigned long domain = 0, level = 0;
	char tmp[32], *comma;

	strscpy(tmp, val, sizeof(tmp));
	comma = strchr(tmp, ',');
	if (!comma)
		return -EINVAL;
	*comma = '\0';
	if (kstrtoul(tmp, 0, &domain) || kstrtoul(comma + 1, 0, &level))
		return -EINVAL;
	if (domain >= 8)
		return -EINVAL;

	scmi_level_override[domain] = (u32)level;
	pr_info("KPM_OC: SCMI level override: domain %lu → %lu\n", domain, level);
	return 0;
}

static const struct kernel_param_ops scmi_ov_ops = { .set = set_scmi_override };
static int scmi_ov_dummy;
module_param_cb(scmi_override, &scmi_ov_ops, &scmi_ov_dummy, 0220);
MODULE_PARM_DESC(scmi_override, "Override SCMI perf level: domain,level (0=clear)");

/* ─── mcupm_hopping_v1 argument capture ─────────────────────────────────── */
/*
 * Kprobe on mcupm_hopping_v1 in fhctl module:
 *   int mcupm_hopping_v1(void *priv_data, const char *domain,
 *                        int fh_id, unsigned int new_dds, int postdiv)
 * AArch64: x0=priv_data, x1=domain, w2=fh_id, w3=new_dds, w4=postdiv
 */
#define FHCTL_CAP_BUF_SIZE 2048
static char fhctl_cap_buf[FHCTL_CAP_BUF_SIZE];
module_param_string(fhctl_capture, fhctl_cap_buf, sizeof(fhctl_cap_buf), 0444);
MODULE_PARM_DESC(fhctl_capture, "Last captured mcupm_hopping_v1 calls (read-only)");
static int fhctl_cap_pos;

static int __nocfi fhctl_hop_kp_pre(struct kprobe *p, struct pt_regs *regs)
{
	void *priv_data = (void *)regs->regs[0];
	const void *domain_ptr = (const void *)regs->regs[1];
	unsigned int fh_id = (unsigned int)regs->regs[2];
	unsigned int new_dds = (unsigned int)regs->regs[3];
	unsigned int postdiv = (unsigned int)regs->regs[4];

	/* Ring buffer: newest entries overwrite oldest */
	if (fhctl_cap_pos > FHCTL_CAP_BUF_SIZE - 80)
		fhctl_cap_pos = 0;

	fhctl_cap_pos += snprintf(fhctl_cap_buf + fhctl_cap_pos,
				  FHCTL_CAP_BUF_SIZE - fhctl_cap_pos,
				  "hop priv=%px dom=%px fh_id=%u dds=0x%08x pdiv=%u | ",
				  priv_data, domain_ptr, fh_id, new_dds, postdiv);

	pr_info_ratelimited("KPM_OC: FH hop priv=%px dom=%px fh_id=%u dds=0x%08x pdiv=%u\n",
			    priv_data, domain_ptr, fh_id, new_dds, postdiv);
	return 0;
}

static struct kprobe fhctl_hop_kp = {
	.symbol_name = "mcupm_hopping_v1",
	.pre_handler = fhctl_hop_kp_pre,
};

/* ─── IPI-based PLL overclock via MCUPM fhctl ──────────────────────────── */
/*
 * Construct and send an FHCTL IPI message to MCUPM to request a PLL
 * frequency change.  Uses the same IPI mechanism as mcupm_hopping_v1.
 *
 * IPI message format (9 x u32):
 *   [0] = cmd (0x1006 = FHCTL DVFS command)
 *   [1] = fh_id (PLL hopping index)
 *   [2] = new_dds (target DDS value)
 *   [3] = postdiv (-1 = no postdiv change)
 *   [4..8] = reserved/zero
 *
 * Write format: "fh_id,dds_hex[,postdiv]"
 * Example:  "2,0x203627"      → fh_id=2, dds=0x203627, postdiv=-1
 *           "3,0x248B06,0"    → fh_id=3, dds=0x248B06, postdiv=0
 *
 * NOTE: This is EXPERIMENTAL.  MCUPM may reject or ignore the DDS value.
 */

/* External MCUPM API (exported from mcupm.ko and mtk_tinysys_ipi.ko) */
struct mtk_ipi_device;  /* forward decl */
extern struct mtk_ipi_device *get_mcupm_ipidev(void);
extern int mtk_ipi_send_compl(struct mtk_ipi_device *ipidev,
			      int ipi_id, int opt,
			      void *data, int len, unsigned long timeout);

#define FHCTL_IPI_CMD        0x1006
#define FHCTL_IPI_PIN        2      /* hardcoded in mcupm_hopping_v1 disasm (mov w1, #0x2) */
#define FHCTL_IPI_MSG_LEN    9      /* 9 x u32 words */
#define FHCTL_IPI_TIMEOUT    2000   /* ms */

#define FHCTL_OC_BUF_SIZE   512
static char fhctl_oc_buf[FHCTL_OC_BUF_SIZE];
module_param_string(fhctl_oc_result, fhctl_oc_buf, sizeof(fhctl_oc_buf), 0444);
MODULE_PARM_DESC(fhctl_oc_result, "Result of last fhctl IPI OC attempt (read-only)");

static int set_fhctl_oc(const char *val, const struct kernel_param *kp)
{
	unsigned long fh_id_l = 0, dds_l = 0;
	long postdiv_l = -1;  /* -1 = no postdiv change */
	u32 ipi_msg[FHCTL_IPI_MSG_LEN];
	struct mtk_ipi_device *ipidev;
	int ret;
	char tmp[64], *p1, *p2;

	memset(fhctl_oc_buf, 0, sizeof(fhctl_oc_buf));

	strscpy(tmp, val, sizeof(tmp));

	/* Parse "fh_id,dds_hex[,postdiv]" */
	p1 = strchr(tmp, ',');
	if (!p1) {
		snprintf(fhctl_oc_buf, FHCTL_OC_BUF_SIZE,
			 "ERR: format: fh_id,dds_hex[,postdiv]");
		return -EINVAL;
	}
	*p1++ = '\0';
	p2 = strchr(p1, ',');
	if (p2) {
		*p2++ = '\0';
		if (kstrtol(p2, 0, &postdiv_l))
			return -EINVAL;
	}

	if (kstrtoul(tmp, 0, &fh_id_l) || kstrtoul(p1, 0, &dds_l))
		return -EINVAL;

	if (fh_id_l > 15 || dds_l > 0x3FFFFF) {
		snprintf(fhctl_oc_buf, FHCTL_OC_BUF_SIZE,
			 "ERR: fh_id=%lu (max 15), dds=0x%lx (max 0x3FFFFF)",
			 fh_id_l, dds_l);
		return -EINVAL;
	}

	/* Get MCUPM IPI device */
	ipidev = get_mcupm_ipidev();
	if (!ipidev) {
		snprintf(fhctl_oc_buf, FHCTL_OC_BUF_SIZE,
			 "ERR: get_mcupm_ipidev() returned NULL");
		return -ENODEV;
	}

	/* Construct IPI message */
	memset(ipi_msg, 0, sizeof(ipi_msg));
	ipi_msg[0] = FHCTL_IPI_CMD;         /* 0x1006 */
	ipi_msg[1] = (u32)fh_id_l;
	ipi_msg[2] = (u32)dds_l;
	ipi_msg[3] = (u32)(int)postdiv_l;   /* -1 or specific value */

	pr_info("KPM_OC: FHCTL IPI OC: fh_id=%lu dds=0x%06lx pdiv=%ld\n",
		fh_id_l, dds_l, postdiv_l);

	ret = mtk_ipi_send_compl(ipidev, FHCTL_IPI_PIN, 1,
				 ipi_msg, FHCTL_IPI_MSG_LEN,
				 FHCTL_IPI_TIMEOUT);

	snprintf(fhctl_oc_buf, FHCTL_OC_BUF_SIZE,
		 "fh_id=%lu dds=0x%06lx pdiv=%ld → IPI ret=%d",
		 fh_id_l, dds_l, postdiv_l, ret);

	pr_info("KPM_OC: FHCTL IPI result: %s\n", fhctl_oc_buf);
	return 0;
}

static const struct kernel_param_ops fhctl_oc_ops = { .set = set_fhctl_oc };
static int fhctl_oc_dummy;
module_param_cb(fhctl_oc, &fhctl_oc_ops, &fhctl_oc_dummy, 0220);
MODULE_PARM_DESC(fhctl_oc, "FHCTL IPI OC: write fh_id,dds_hex[,postdiv]");

/* ─── fhctl has_perms patch ─────────────────────────────────────────────── */
/*
 * Unlock fhctl debugfs ctrl interface by setting has_perms = 1.
 * This allows PLL hopping via:  echo "cpu2pll hopping 0xDDS" > /sys/kernel/debug/fhctl/ctrl
 */
static int set_fhctl_unlock(const char *val, const struct kernel_param *kp)
{
	unsigned long sym;

	if (!kln_func && resolve_kallsyms())
		return -ENOENT;

	sym = kln_func("has_perms");
	if (!sym) {
		pr_err("KPM_OC: fhctl has_perms symbol not found\n");
		return -ENOENT;
	}

	*(u32 *)sym = 1;
	pr_info("KPM_OC: fhctl has_perms@%px set to 1\n", (void *)sym);
	return 0;
}

static const struct kernel_param_ops fhctl_unlock_ops = { .set = set_fhctl_unlock };
static int fhctl_unlock_dummy;
module_param_cb(fhctl_unlock, &fhctl_unlock_ops, &fhctl_unlock_dummy, 0220);
MODULE_PARM_DESC(fhctl_unlock, "Write 1 to unlock fhctl debugfs ctrl");

static void cpu_reapply_volt_overrides_locked(void)
{
	int c;

	if (cpu_volt_ov_count <= 0 || !csram_base)
		return;

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int dom_base = domain_offsets[c];
		int i;

		for (i = 0; i < LUT_MAX_ENTRIES; i++) {
			unsigned int ov = cpu_volt_ov[c][i];
			u32 cur, patched;

			if (!ov)
				continue;

			cur = readl_relaxed(csram_base + dom_base +
					    REG_FREQ_LUT + i * LUT_ROW_SIZE);
			patched = (cur & (LUT_GEAR_MASK | LUT_FREQ_MASK)) |
				  ((ov / 10) << 12);
			if (cur != patched)
				writel_relaxed(patched, csram_base + dom_base +
					       REG_FREQ_LUT + i * LUT_ROW_SIZE);
		}
	}

	wmb();
}

/* ─── Periodic freq_qos re-lift worker ──────────────────────────────────── */
/*
 * Vendor modules (powerhal, fpsgo, mtk_cpu_power_throttling) periodically
 * re-assert FREQ_QOS_MAX constraints to the stock max.  This worker checks
 * the aggregate every RELIFT_INTERVAL_MS and re-lifts as needed.
 * With the kprobe active this worker acts as a safety net only.
 */
static void __nocfi cpu_oc_relift_work_fn(struct work_struct *work)
{
	int c;
	bool any_lifted = false;

	if (!cpu_oc_relift_active)
		return;

	for (c = 0; c < NUM_CLUSTERS; c++) {
		unsigned int target = cpu_oc_targets[c];
		struct cpufreq_policy *policy;
		s32 agg_max;
		bool need_enforce;

		if (target == 0)
			continue;

		policy = fn_cpufreq_cpu_get ?
			fn_cpufreq_cpu_get(cluster_policies[c]) : NULL;
		if (!policy)
			continue;

		/*
		 * Keep cluster_qos_ptr fresh so the freq_qos_update_request
		 * kprobe can identify which requests target this cluster.
		 * This pointer may be NULL if set_cpu_oc() ran at early boot
		 * before cpufreq policies were registered, which would leave
		 * the kprobe unable to intercept vendor stock-cap writes
		 * (powerhal, fpsgo, thermal) even after relift succeeds.
		 */
		WRITE_ONCE(cluster_qos_ptr[c], &policy->constraints);

		agg_max = fn_freq_qos_read_value ?
			fn_freq_qos_read_value(&policy->constraints,
					       2 /* FREQ_QOS_MAX */) : (s32)target;

		need_enforce = ((unsigned int)agg_max < target) ||
			      (policy->max < target) ||
			      (policy->cpuinfo.max_freq < target);

		if (need_enforce) {
			/* Re-lift user request */
			if (fn_freq_qos_update_req && policy->max_freq_req)
				fn_freq_qos_update_req(policy->max_freq_req, target);

			policy->max = target;
			policy->cpuinfo.max_freq = target;

			/*
			 * Walk policy->constraints MAX plist and raise every
			 * request that is below the OC target.  This catches
			 * unknown vendor requests that bypass known symbols.
			 * Collect pointers before updating to avoid walking a
			 * list that changes under us.
			 */
			if (fn_freq_qos_update_req) {
				struct plist_head *ph =
					&policy->constraints.max_freq.list;
				struct plist_node *pn;
				struct freq_qos_request *rqs[32];
				int nrq = 0, i;

				plist_for_each(pn, ph) {
					if ((unsigned int)pn->prio < target &&
					    nrq < (int)ARRAY_SIZE(rqs))
						rqs[nrq++] = container_of(
							pn,
							struct freq_qos_request,
							pnode);
				}
				for (i = 0; i < nrq; i++)
					fn_freq_qos_update_req(rqs[i], target);
			}

			if (fn_cpufreq_cpu_put)
				fn_cpufreq_cpu_put(policy);

			/* powerhal / touch_boost */
			if (fn_update_userlimit_max)
				fn_update_userlimit_max(c, target);

			/* fpsgo */
			if (fn_freq_qos_update_req && fbt_cpu_rq_sym) {
				void *rq_base = *(void **)fbt_cpu_rq_sym;
				if (rq_base)
					fn_freq_qos_update_req(
						(char *)rq_base + c * FBT_RQ_STRIDE,
						target);
			}

			/* power throttling */
			if (fn_freq_qos_update_req && pt_policy_list_sym) {
				struct list_head *head = (struct list_head *)pt_policy_list_sym;
				struct list_head *pos;
				list_for_each(pos, head) {
					u32 active = *(u32 *)((char *)pos - CPU_PT_NODE_TO_FLAG_OFF);
					if (active)
						fn_freq_qos_update_req(
							(char *)pos - CPU_PT_NODE_TO_REQ_OFF,
							INT_MAX);
				}
			}

			pr_info("KPM_OC: relift %s: agg_max %d -> %u\n",
				cluster_names[c], agg_max, target);
			any_lifted = true;
		} else {
			if (fn_cpufreq_cpu_put)
				fn_cpufreq_cpu_put(policy);
		}
	}

	(void)any_lifted;

	/* Safety net: event-driven sync handles transitions, this catches misses. */
	cpu_reapply_volt_overrides_locked();

	if (cpu_oc_relift_active)
		schedule_delayed_work(&cpu_oc_relift_dwork,
				      msecs_to_jiffies(RELIFT_INTERVAL_MS));
}

/* ─── GPU OPP Runtime Memory Patcher ────────────────────────────────────── */
/*
 * Patches g_gpu_default_opp_table[0] in the already-loaded mtk_gpufreq_mt6897
 * module's kernel memory, then updates the shared-status region so GPUEB can
 * (potentially) see the new values.
 *
 * OPP entry layout (stride 24 bytes):
 *   u32 freq;     // KHz
 *   u32 volt;     // µV
 *   u32 vsram;    // µV
 *   u32 posdiv;
 *   u32 vaging;
 *   u32 power;
 */

#define GPU_OPP_STRIDE       24
#define GPU_OPP_SHARED_OFF   0x3ec   /* GPU OPP table offset in shared_status */
#define GPU_RESULT_BUF_SIZE  512
#define GPU_MAX_OPPS         69      /* SignedOPPNum on this SoC */

/* Per-OPP voltage overrides (bypasses vendor fix_custom_freq_volt validation).
 * Non-zero entries are direct-memory-patched into default + working tables
 * and kept alive by the GPU relift kthread.
 */
static unsigned int gpu_volt_ov[GPU_MAX_OPPS];   /* override volt (10µV step), 0=off */
static unsigned int gpu_vsram_ov[GPU_MAX_OPPS];  /* override vsram, 0=off */
static int          gpu_volt_ov_count;            /* number of active overrides */

/* Original default_opp_table values — saved on first override for clean restore */
static unsigned int gpu_orig_volt[GPU_MAX_OPPS];
static unsigned int gpu_orig_vsram[GPU_MAX_OPPS];
static bool         gpu_orig_saved;

/* CPU per-LUT-entry voltage overrides.
 * Written directly to CSRAM, bypassing stock constraints.
 */
static unsigned int cpu_volt_ov[NUM_CLUSTERS][LUT_MAX_ENTRIES]; /* µV, 0=off */
static int          cpu_volt_ov_count;
/* Original CSRAM LUT voltages for clean restore */
static unsigned int cpu_orig_volt[NUM_CLUSTERS][LUT_MAX_ENTRIES];

/* Configurable via module params */
static unsigned int gpu_target_freq = 1900000;
module_param(gpu_target_freq, uint, 0644);
MODULE_PARM_DESC(gpu_target_freq, "Target GPU freq in KHz (default 1900000; hardware PLL max, signed OPP[0] OPP[0])");

static unsigned int gpu_target_volt = 115040;
module_param(gpu_target_volt, uint, 0644);
MODULE_PARM_DESC(gpu_target_volt, "Target GPU volt in 10µV steps (default 115040 = 1150.4mV; matches signed OPP[0]mV; matches signed OPP[0])");

static unsigned int gpu_target_vsram = 95000;
module_param(gpu_target_vsram, uint, 0644);
MODULE_PARM_DESC(gpu_target_vsram, "Target GPU vsram in 10µV steps (default 95000 = 950mV)");

/*
 * GPUEB stock max OPP freq (KHz).  Requests at or below this are satisfied
 * by GPUEB's own OPP table; above this, AP-side PLL programming in
 * __gpufreq_freq_scale_gpu handles the OC (LEGACY DVFSMode: no GPUEB IPI).
 */
#define GPU_STOCK_MAX_KHZ 1400000U

/*
 * Hardware PLL maximum freq (KHz).  The MFG PLL abort check in
 * __gpufreq_freq_scale_gpu rejects anything above this (postdiv_pow=1 range).
 * Writing tgt_freq > this value to the working table causes __gpufreq_abort.
 */
#define GPU_PLL_MAX_KHZ   1900000U

/* ─── MFG PLL direct programming (bypass GPUEB for above-stock OC) ──────
 *
 * On MT6897 with GPUEBSupport=On, gpufreq_commit() sends IPI to GPUEB which
 * programs PLL registers.  AP-side __gpufreq_generic_commit_gpu is NEVER called.
 *
 * For >1400 MHz OC, we let GPUEB commit OPP[0] normally (1400 MHz + overvolted),
 * then immediately reprogram MFG PLL and MFGSC PLL to the actual target freq.
 * GPUEB owns voltage (safe at 105V for ~1500MHz), we only override the clock.
 *
 * Register layout (from __gpufreq_freq_scale_gpu disassembly):
 *   CON1 (offset 0x0C): [31]=PCW_CHG, [26:24]=POSTDIV_POW, [21:0]=PCW
 *   PCW formula:  PCW = (target_khz * (1 << postdiv_pow) << 14) / 26000
 *   POSTDIV_POW thresholds:
 *     > 950000 KHz → 1  (VCO = freq*2, range 1900002–3800000)
 *     > 475000 KHz → 2  (VCO = freq*4)
 *     > 237500 KHz → 3  (VCO = freq*8)
 *     else         → 4  (VCO = freq*16)
 */
#define MFG_PLL_PHYS_BASE   0x13FA0000UL
#define MFG_PLL_PHYS_SIZE   0x400UL
#define MFGSC_PLL_PHYS_BASE 0x13FA0C00UL
#define MFGSC_PLL_PHYS_SIZE 0x400UL
#define MFG_PLL_CON1_OFF    0x0C
#define GPU_FIN_KHZ         26000U
#define GPU_PCW_FRAC_BITS   14

static void __iomem *mfg_pll_virt;
static void __iomem *mfgsc_pll_virt;

static void gpu_pll_set_freq(unsigned int tar_freq_khz)
{
	unsigned int postdiv_pow;
	u64 vco_khz;
	u32 pcw, con1, readback;
	static unsigned long last_log_jiffies;

	if (!mfg_pll_virt)
		return;

	if (tar_freq_khz > GPU_PLL_MAX_KHZ)
		tar_freq_khz = GPU_PLL_MAX_KHZ;

	if (tar_freq_khz > 950000)
		postdiv_pow = 1;
	else if (tar_freq_khz > 475000)
		postdiv_pow = 2;
	else if (tar_freq_khz > 237500)
		postdiv_pow = 3;
	else
		postdiv_pow = 4;

	vco_khz = (u64)tar_freq_khz * (1U << postdiv_pow);
	pcw = (u32)div_u64(vco_khz << GPU_PCW_FRAC_BITS, GPU_FIN_KHZ);
	con1 = (1U << 31) | (postdiv_pow << 24) | (pcw & 0x3FFFFF);

	writel(con1, mfg_pll_virt + MFG_PLL_CON1_OFF);
	if (mfgsc_pll_virt)
		writel(con1, mfgsc_pll_virt + MFG_PLL_CON1_OFF);

	/* Readback to check GPUEB hasn't actually changed the target frequency.
	 * GPUEB periodically clears bit31 (PCW_CHG/lock flag) as part of its
	 * normal lifecycle, but leaves the PCW (frequency divider) unchanged.
	 * Only log "FREQ_CHANGED" if GPUEB actually modified the PCW/postdiv.
	 */
	readback = readl(mfg_pll_virt + MFG_PLL_CON1_OFF);
	if (time_after(jiffies, last_log_jiffies + msecs_to_jiffies(2000))) {
		unsigned int rb_pcw = readback & 0x3FFFFF;
		unsigned int rb_pdiv = (readback >> 24) & 0x7;
		u64 rb_vco = (u64)rb_pcw * GPU_FIN_KHZ;
		unsigned int rb_freq = (unsigned int)(rb_vco >> (GPU_PCW_FRAC_BITS + rb_pdiv));
		/* Compare only PCW + postdiv (freq-determining bits), not bit31 (update flag) */
		bool freq_ok = ((readback & 0x07FFFFFF) == (con1 & 0x07FFFFFF));

		pr_info("KPM_OC: gpu_pll: wrote con1=0x%08x (%uKHz) readback=0x%08x (%uKHz) %s%s\n",
			con1, tar_freq_khz, readback, rb_freq,
			freq_ok ? "freq_ok" : "FREQ_CHANGED_BY_GPUEB",
			(!freq_ok) ? "" :
			  ((readback >> 31) & 1) ? " lock=1" : " lock=0(GPUEB_cleared_flag)");
		last_log_jiffies = jiffies;
	}
}

/* kretprobe on gpufreq_commit (wrapper exported symbol).
 * Entry handler captures oppidx; return handler reprograms PLL when OPP==0
 * and target freq exceeds stock max.
 */
struct gpu_commit_kretp_data {
	int oppidx;
};

static int __nocfi gpu_commit_kretp_entry(struct kretprobe_instance *ri,
					  struct pt_regs *regs)
{
	struct gpu_commit_kretp_data *d =
		(struct gpu_commit_kretp_data *)ri->data;
	d->oppidx = (int)regs->regs[1];
	return 0;
}

static atomic_t gpu_pll_override_count = ATOMIC_INIT(0);

static int __nocfi gpu_commit_kretp_ret(struct kretprobe_instance *ri,
					struct pt_regs *regs)
{
	struct gpu_commit_kretp_data *d =
		(struct gpu_commit_kretp_data *)ri->data;
	unsigned int tgt = READ_ONCE(gpu_target_freq);
	int ret_val = (int)regs_return_value(regs);

	/* Only override PLL when:
	 * 1) commit succeeded (ret == 0)
	 * 2) OPP was 0 (max freq)
	 * 3) target freq exceeds stock max
	 */
	if (ret_val == 0 && d->oppidx == 0 && tgt > GPU_STOCK_MAX_KHZ) {
		gpu_pll_set_freq(tgt);
		atomic_inc(&gpu_pll_override_count);
	}

	return 0;
}

static struct kretprobe gpu_commit_kretp = {
	.handler = gpu_commit_kretp_ret,
	.entry_handler = gpu_commit_kretp_entry,
	.data_size = sizeof(struct gpu_commit_kretp_data),
	.maxactive = 4,
	.kp.symbol_name = "gpufreq_commit",
};

static char gpu_oc_result[GPU_RESULT_BUF_SIZE];
module_param_string(gpu_oc_result, gpu_oc_result, sizeof(gpu_oc_result), 0444);
MODULE_PARM_DESC(gpu_oc_result, "Result of last GPU OC patch operation");

/* gpu_pll_diag: read MFG PLL CON1 register and compute actual GPU clock.
 * Also shows MFGSC PLL CON1 for cross-validation.
 * con1 [31]=PCW_CHG, [26:24]=POSTDIV_POW, [21:0]=PCW
 * freq_khz = (pcw * 26000) >> (14 + postdiv_pow)
 */
static int get_gpu_pll_diag(char *buf, const struct kernel_param *kp)
{
	u32 mfg_con1 = 0, mfgsc_con1 = 0;
	unsigned int pcw, postdiv_pow, freq_khz;
	u64 vco_khz;

	if (!mfg_pll_virt)
		return scnprintf(buf, PAGE_SIZE, "ERR:no_ioremap\n");

	mfg_con1 = readl(mfg_pll_virt + MFG_PLL_CON1_OFF);
	if (mfgsc_pll_virt)
		mfgsc_con1 = readl(mfgsc_pll_virt + MFG_PLL_CON1_OFF);

	pcw        = mfg_con1 & 0x3FFFFF;
	postdiv_pow = (mfg_con1 >> 24) & 0x7;
	vco_khz    = (u64)pcw * GPU_FIN_KHZ;
	freq_khz   = (unsigned int)(vco_khz >> (GPU_PCW_FRAC_BITS + postdiv_pow));

	return scnprintf(buf, PAGE_SIZE,
			 "mfg_con1=0x%08x pcw=0x%06x postdiv=%u freq=%uKHz"
			 " mfgsc_con1=0x%08x tgt=%u overrides=%d\n",
			 mfg_con1, pcw, postdiv_pow, freq_khz,
			 mfgsc_con1, READ_ONCE(gpu_target_freq),
			 atomic_read(&gpu_pll_override_count));
}

static const struct kernel_param_ops gpu_pll_diag_ops = {
	.get = get_gpu_pll_diag,
};
static int gpu_pll_diag_dummy;
module_param_cb(gpu_pll_diag, &gpu_pll_diag_ops, &gpu_pll_diag_dummy, 0444);
MODULE_PARM_DESC(gpu_pll_diag, "Read MFG PLL CON1 and compute actual GPU clock frequency");

/*
 * gpu_pll_force_khz: write a frequency (KHz) to directly program MFG PLL CON1
 * without waiting for GPU load / kretprobe.  Used to diagnose PLL behaviour.
 *
 * SAFETY: Writing the PLL at a frequency higher than the active voltage level
 * allows (e.g. 1600 MHz at 1.05V) will crash the device.  This write is
 * rejected unless the target frequency is <= gpu_target_freq (which is the
 * value for which the OPP table voltage has been set).  To bypass this
 * gating set gpu_pll_force_unsafe=1 first (diagnostic only, crash risk).
 */
static bool gpu_pll_force_unsafe;
module_param(gpu_pll_force_unsafe, bool, 0644);
MODULE_PARM_DESC(gpu_pll_force_unsafe, "Allow gpu_pll_force_khz above gpu_target_freq (UNSAFE, crash risk)");

static int set_gpu_pll_force(const char *val, const struct kernel_param *kp)
{
	unsigned long freq_khz;
	int ret = kstrtoul(val, 10, &freq_khz);

	if (ret)
		return ret;
	if (freq_khz == 0 || freq_khz > GPU_PLL_MAX_KHZ)
		return -ERANGE;

	/* Reject if requested freq exceeds the voltage-matched target, unless
	 * the user has explicitly acknowledged the crash risk.
	 */
	if (!gpu_pll_force_unsafe && freq_khz > READ_ONCE(gpu_target_freq)) {
		pr_warn("KPM_OC: gpu_pll_force: %lu KHz > gpu_target_freq=%u; set gpu_pll_force_unsafe=1 to override\n",
			freq_khz, READ_ONCE(gpu_target_freq));
		return -EPERM;
	}

	gpu_pll_set_freq((unsigned int)freq_khz);
	pr_info("KPM_OC: gpu_pll_force: wrote %lu KHz to MFG PLL CON1\n", freq_khz);
	return 0;
}

static const struct kernel_param_ops gpu_pll_force_ops = {
	.set = set_gpu_pll_force,
};
static int gpu_pll_force_dummy;
module_param_cb(gpu_pll_force_khz, &gpu_pll_force_ops, &gpu_pll_force_dummy, 0220);
MODULE_PARM_DESC(gpu_pll_force_khz, "Write KHz to directly program MFG PLL (must be <= gpu_target_freq unless gpu_pll_force_unsafe=1)");

/*
 * Runtime patch strategy:
 * 1) Patch g_gpu_default_opp_table[0] via direct symbol lookup.
 * 2) Patch runtime tables via getter functions:
 *      __gpufreq_get_working_table_gpu()
 *      __gpufreq_get_signed_table_gpu()
 * 3) Best-effort sync to shared_status by calling:
 *      __gpufreq_update_shared_status_opp_table()
 *
 * This intentionally avoids fixed .bss deltas. On this target, duplicate local
 * symbols (for example g_shared_status) can resolve to storage that stays NULL.
 */
typedef u32 *(*gpufreq_get_table_t)(void);
typedef void (*gpufreq_update_shared_t)(void);
/* gpufreq_get_working_table(target) from mtk_gpufreq_wrapper - public API.
 * Takes an int (TARGET_GPU=0, TARGET_STACK=1). May read g_shared_status which
 * is CPU-accessible even when GPU is powered off, unlike the internal variant.
 */
typedef u32 *(*gpufreq_get_table_tgt_t)(int target);
/* gpufreq_fix_custom_freq_volt(target, freq_khz, volt_10uv) from mtk_gpufreq_wrapper.
 * Forces GPU/STACK to a custom freq+volt bypassing OPP-table DVFS.
 * Returns 0 on success, -EINVAL when GPU is powered off or freq out of range.
 */
typedef int (*gpufreq_fix_custom_fv_t)(int target, unsigned int freq, unsigned int volt);

/* Saved only for diagnostics; runtime patching no longer relies on this. */
static unsigned long opp_table_anchor;
static gpufreq_get_table_t fn_get_working_table_gpu;
static gpufreq_get_table_t fn_get_signed_table_gpu;
static gpufreq_get_table_tgt_t fn_get_working_table_wrap;
static gpufreq_get_table_tgt_t fn_get_signed_table_wrap;
static gpufreq_update_shared_t fn_update_shared_status_opp_table;
static gpufreq_fix_custom_fv_t fn_fix_custom_fv;

/*
 * Track availability/patch state. Bits:
 *   1 = g_gpu_default_opp_table[0] patched
 *   2 = working_table_gpu[0] patched
 *   4 = signed_table_gpu[0] patched
 *
 * Note: set_gpu_oc() always rewrites OPP[0] so users can change
 * gpu_target_* and re-apply at runtime.
 */
static int gpu_oc_patched;

/* Kthread for GPU relift (periodically re-patches working/signed tables) */
#define GPU_RELIFT_INTERVAL_MS 500
static struct task_struct *gpu_oc_task;

static void __nocfi gpu_resolve_runtime_symbols(void)
{
    if (!kln_func)
        return;

    if (!fn_get_working_table_gpu)
        fn_get_working_table_gpu =
            (gpufreq_get_table_t)kln_func("__gpufreq_get_working_table_gpu");

    if (!fn_get_signed_table_gpu)
        fn_get_signed_table_gpu =
            (gpufreq_get_table_t)kln_func("__gpufreq_get_signed_table_gpu");

    if (!fn_update_shared_status_opp_table)
        fn_update_shared_status_opp_table =
            (gpufreq_update_shared_t)kln_func("__gpufreq_update_shared_status_opp_table");

    /* Public wrapper API - reads g_shared_status, accessible when GPU is off */
    if (!fn_get_working_table_wrap)
        fn_get_working_table_wrap =
            (gpufreq_get_table_tgt_t)kln_func("gpufreq_get_working_table");
    if (!fn_get_signed_table_wrap)
        fn_get_signed_table_wrap =
            (gpufreq_get_table_tgt_t)kln_func("gpufreq_get_signed_table");
    /* Direct GPU freq override — bypasses GPUEB OPP table for above-stock OC */
    if (!fn_fix_custom_fv)
        fn_fix_custom_fv =
            (gpufreq_fix_custom_fv_t)kln_func("gpufreq_fix_custom_freq_volt");
}

static int gpu_patch_table_opp0(u32 *tbl, const char *name, int bit)
{
	u32 freq;
	u32 volt;
	u32 vsram;
	unsigned int clamped_freq;

	if (!tbl)
		return 0;

	freq = tbl[0];
	volt = tbl[1];
	vsram = tbl[2];
	if (freq < 1000000 || freq > 4000000) {
		pr_warn("KPM_OC: %s[0] unexpected freq=%u, skip\n", name, freq);
		return 0;
	}

	clamped_freq = gpu_target_freq;
	if (clamped_freq > GPU_PLL_MAX_KHZ)
		clamped_freq = GPU_PLL_MAX_KHZ;

	if (freq == clamped_freq &&
	    volt == gpu_target_volt &&
	    vsram == gpu_target_vsram)
		return bit;

	pr_info("KPM_OC: %s[0] at %px freq=%u->%u volt=%u->%u vsram=%u->%u\n",
		name, tbl, freq, clamped_freq, volt, gpu_target_volt,
		vsram, gpu_target_vsram);
	tbl[0] = clamped_freq;
	tbl[1] = gpu_target_volt;
	tbl[2] = gpu_target_vsram;
	return bit;
}

/*
 * gpu_oc_patch_wk_ss — attempt to patch working_table and signed_table.
 * Caller MUST hold &lock.
 * Returns bitmask of new patches (bits 2/4).
 */
static int __nocfi gpu_oc_patch_wk_ss(void)
{
	int patched = 0;
	u32 *wt;
	u32 *st = NULL;

	gpu_resolve_runtime_symbols();

	/* Also sync default_opp_table[0] so update_shared_status doesn't
	 * revert the working table OPP[0] on every cycle.
	 */
	if (opp_table_anchor)
		patched |= gpu_patch_table_opp0((u32 *)opp_table_anchor,
						 "default_table", 0);

	wt = NULL;
	if (fn_get_working_table_gpu)
		wt = fn_get_working_table_gpu();
	/* Fallback: wrapper public API reads g_shared_status (GPU-off safe) */
	if (!wt && fn_get_working_table_wrap)
		wt = fn_get_working_table_wrap(0); /* TARGET_GPU = 0 */
	patched |= gpu_patch_table_opp0(wt, "working_table", 2);

	if (fn_get_signed_table_gpu)
		st = fn_get_signed_table_gpu();
	if (!st && fn_get_signed_table_wrap)
		st = fn_get_signed_table_wrap(0);
	patched |= gpu_patch_table_opp0(st, "signed_table", 4);

	/* ── Per-OPP voltage overrides (direct memory) ── */
	if (gpu_volt_ov_count > 0 && (wt || opp_table_anchor)) {
		unsigned long def_addr = opp_table_anchor;
		int i;
		for (i = 0; i < GPU_MAX_OPPS; i++) {
			unsigned int ov = gpu_volt_ov[i];
			unsigned int vs = gpu_vsram_ov[i];
			u32 *dt, *wte;
			if (ov == 0 && vs == 0)
				continue;
			/* Patch default_opp_table */
			if (def_addr) {
				dt = (u32 *)(def_addr + (unsigned long)i * GPU_OPP_STRIDE);
				if (ov && dt[1] != ov)
					dt[1] = ov;
				if (vs && dt[2] != vs)
					dt[2] = vs;
			}
			/* Patch working_table */
			if (wt) {
				wte = wt + i * (GPU_OPP_STRIDE / 4);
				if (ov && wte[1] != ov)
					wte[1] = ov;
				if (vs && wte[2] != vs)
					wte[2] = vs;
			}
		}
		patched |= 8; /* bit 3 = volt overrides applied */
	}

	/* Sync shared_status so the proc file reflects the new frequency.
	 * Only called when at least one table was patched (patched != 0),
	 * to avoid NULL-deref when GPU is powered off.
	 */
	if (fn_update_shared_status_opp_table && patched)
		fn_update_shared_status_opp_table();

	if (patched)
		wmb();

	return patched;
}

/*
 * Diagnostic dump: print resolved function pointers and current runtime tables.
 */
static void __nocfi gpu_oc_diagnostic(void)
{
	u32 *wt = NULL;
	u32 *st = NULL;

	gpu_resolve_runtime_symbols();

	pr_info("KPM_OC: diag: opp_table_anchor=%px\n", (void *)opp_table_anchor);
	pr_info("KPM_OC: diag: fn_get_wt=%px fn_get_st=%px fn_sync_ss=%px fn_wt_wrap=%px\n",
		fn_get_working_table_gpu,
		fn_get_signed_table_gpu,
		fn_update_shared_status_opp_table,
		fn_get_working_table_wrap);

	if (fn_get_working_table_gpu)
		wt = fn_get_working_table_gpu();
	if (!wt && fn_get_working_table_wrap)
		wt = fn_get_working_table_wrap(0);
	if (fn_get_signed_table_gpu)
		st = fn_get_signed_table_gpu();
	if (!st && fn_get_signed_table_wrap)
		st = fn_get_signed_table_wrap(0);

	pr_info("KPM_OC: diag: working_table=%px signed_table=%px\n", wt, st);

	if (wt) {
		pr_info("KPM_OC: diag: working_table[0] freq=%u volt=%u vsram=%u\n",
			wt[0], wt[1], wt[2]);
	}

	if (st) {
		pr_info("KPM_OC: diag: signed_table[0] freq=%u volt=%u vsram=%u\n",
			st[0], st[1], st[2]);
	}
}

/* ─── GPUEB countermeasure: event-driven GPU OPP resync ─────────────────
 *
 * Root cause: GPUEB firmware owns the shared_status/working_table in shared
 * memory and continuously overwrites OPP voltage fields (e.g. OPP[0].volt
 * reverts to 40500 from our target 105000).  The g_gpu_default_opp_table in
 * AP module .bss is also reverted — likely by a kernel path that synchronises
 * from shared_status back to the default table.
 *
 * Fix: Kprobe on __gpufreq_generic_commit_gpu.  This is the main GPU DVFS
 * commit function, called right before voltage/frequency scaling.  The
 * pre-handler re-patches default_table and working_table so the commit reads
 * our OC values.  Combined with the periodic kthread (reduced to safety-net
 * role), this eliminates the GPUEB overwrite race.
 */
static int __nocfi gpu_commit_kp_pre(struct kprobe *p, struct pt_regs *regs)
{
	unsigned int tgt_freq = READ_ONCE(gpu_target_freq);
	unsigned int tgt_volt = READ_ONCE(gpu_target_volt);
	unsigned int tgt_vsram = READ_ONCE(gpu_target_vsram);
	u32 *tbl;
	int i;
	static unsigned long last_log_jiffies;

	if (tgt_freq == 0)
		return 0;

	/* Clamp to PLL hardware max to prevent __gpufreq_abort on commit */
	if (tgt_freq > GPU_PLL_MAX_KHZ)
		tgt_freq = GPU_PLL_MAX_KHZ;

	/* Unconditionally re-patch default_opp_table[0].
	 * GPUEB firmware continuously overwrites freq+volt+vsram in shared
	 * memory, so we must force ALL three fields every commit — not just
	 * volt/vsram.  The old "if (tbl[0] == tgt_freq)" guard caused the
	 * kprobe to skip the patch entirely once GPUEB reverted freq.
	 */
	if (opp_table_anchor) {
		tbl = (u32 *)opp_table_anchor;
		WRITE_ONCE(tbl[0], tgt_freq);
		WRITE_ONCE(tbl[1], tgt_volt);
		WRITE_ONCE(tbl[2], tgt_vsram);
		/* Per-OPP voltage overrides */
		if (gpu_volt_ov_count > 0) {
			for (i = 0; i < GPU_MAX_OPPS; i++) {
				unsigned int ov = gpu_volt_ov[i];
				unsigned int vs = gpu_vsram_ov[i];
				u32 *e;
				if (!ov && !vs)
					continue;
				e = tbl + i * (GPU_OPP_STRIDE / 4);
				if (ov) WRITE_ONCE(e[1], ov);
				if (vs) WRITE_ONCE(e[2], vs);
			}
		}
	}

	/* Unconditionally re-patch working_table[0] (shared memory).
	 * Must write freq+volt+vsram right before commit reads them to
	 * minimise the GPUEB overwrite race window.
	 */
	tbl = NULL;
	if (fn_get_working_table_gpu)
		tbl = fn_get_working_table_gpu();
	if (!tbl && fn_get_working_table_wrap)
		tbl = fn_get_working_table_wrap(0);
	if (tbl) {
		WRITE_ONCE(tbl[0], tgt_freq);
		WRITE_ONCE(tbl[1], tgt_volt);
		WRITE_ONCE(tbl[2], tgt_vsram);
		/* Per-OPP overrides on working table */
		if (gpu_volt_ov_count > 0) {
			for (i = 0; i < GPU_MAX_OPPS; i++) {
				unsigned int ov = gpu_volt_ov[i];
				unsigned int vs = gpu_vsram_ov[i];
				u32 *e;
				if (!ov && !vs)
					continue;
				e = tbl + i * (GPU_OPP_STRIDE / 4);
				if (ov) WRITE_ONCE(e[1], ov);
				if (vs) WRITE_ONCE(e[2], vs);
			}
		}
	}

	/* Rate-limited diagnostic: log oppidx + target (avoids redundant
	 * function calls on the hot path; tables were just patched above).
	 */
	if (time_after(jiffies, last_log_jiffies + msecs_to_jiffies(5000))) {
		pr_info("KPM_OC: gpu_commit_kp: oppidx=%u tgt_freq=%u tgt_volt=%u\n",
			(unsigned int)regs->regs[0], tgt_freq, tgt_volt);
		last_log_jiffies = jiffies;
	}

	return 0;
}

static struct kprobe gpu_commit_kp = {
	.symbol_name = "__gpufreq_generic_commit_gpu",
	.pre_handler = gpu_commit_kp_pre,
};

/*
 * Kthread function: runs for the module lifetime and periodically re-applies
 * working/signed table patching.  With the gpu_commit_kp kprobe active, this
 * acts primarily as a safety net for power-on/off transitions where GPUEB
 * refreshes tables outside the commit path.
 */
static int __nocfi gpu_oc_kthread_fn(void *data)
{
	int miss_cnt = 0;
	int last_state = 0;

	pr_info("KPM_OC: GPU relift kthread started (%d ms)\n",
		GPU_RELIFT_INTERVAL_MS);

	while (!kthread_should_stop()) {
		int now;

		msleep(GPU_RELIFT_INTERVAL_MS);

		if (READ_ONCE(gpu_target_freq) == 0)
			continue;

		mutex_lock(&lock);
		gpu_oc_patched |= gpu_oc_patch_wk_ss();
		gpu_resolve_runtime_symbols();
		/* Check working_table bit only; signed_table may be NULL on
		 * this SoC and that is expected — not a miss condition.
		 */
		now = gpu_oc_patched & 2;
		mutex_unlock(&lock);

		/*
		 * GPUEB voltage+frequency management:
		 *
		 * With GPUEBSupport=On, gpufreq_commit() sends IPI to GPUEB
		 * which programs PLL+voltage directly.  AP-side
		 * __gpufreq_generic_commit_gpu is NEVER called.
		 *
		 * For above-stock OC:
		 *   DO NOT call fix_custom_freq_volt — it locks GPUEB at a
		 *   fixed OPP, blocking thermal throttling.  Instead, let
		 *   GPUEB manage DVFS normally.  The gpu_commit_kretp
		 *   kretprobe reprograms PLL after each OPP[0] commit,
		 *   giving us above-stock freq while preserving GPUEB's
		 *   ability to throttle to lower OPPs under thermal pressure.
		 *
		 * For at-or-below stock: call fix_custom_freq_volt normally
		 *   to enforce custom freq+volt via GPUEB IPI.
		 */
		{
			unsigned int tgt = READ_ONCE(gpu_target_freq);
			unsigned int vlt = READ_ONCE(gpu_target_volt);

			if (tgt <= GPU_STOCK_MAX_KHZ && fn_fix_custom_fv)
				fn_fix_custom_fv(0, tgt, vlt);
		}

		if (now == 2 && last_state != 2)
			pr_info("KPM_OC: GPU relift: working patched (patched=%d)\n",
				gpu_oc_patched);

		if (now != 2) {
			miss_cnt++;
			if (miss_cnt >= 600) {
				mutex_lock(&lock);
				gpu_oc_diagnostic();
				mutex_unlock(&lock);
				miss_cnt = 0;
			}
		} else {
			miss_cnt = 0;
		}

		last_state = now;
	}

	return 0;
}

static int set_gpu_oc(const char *val, const struct kernel_param *kp)
{
	unsigned long opp_table_addr;
	u32 *opp_entry;
	u32 orig_freq, orig_volt, orig_vsram;
	unsigned int clamped_freq;
	int pos = 0;
	int newly_patched = 0;

	mutex_lock(&lock);
	memset(gpu_oc_result, 0, sizeof(gpu_oc_result));

	if (resolve_kallsyms()) {
		pos = snprintf(gpu_oc_result, GPU_RESULT_BUF_SIZE,
			       "FAIL:kallsyms_resolve");
		goto out;
	}

	gpu_resolve_runtime_symbols();

	/* ── Step 1: Patch g_gpu_default_opp_table[0] (always re-apply) ── */
	opp_table_addr = kln_func("g_gpu_default_opp_table");
	if (!opp_table_addr) {
		pos = snprintf(gpu_oc_result, GPU_RESULT_BUF_SIZE,
			       "FAIL:opp_table_not_found");
		goto out;
	}

	opp_entry = (u32 *)opp_table_addr;
	orig_freq  = opp_entry[0];
	orig_volt  = opp_entry[1];
	orig_vsram = opp_entry[2];

	if (orig_freq < 1000000 || orig_freq > 4000000) {
		pos = snprintf(gpu_oc_result, GPU_RESULT_BUF_SIZE,
			       "FAIL:unexpected_freq=%u", orig_freq);
		goto out;
	}

	opp_entry[0] = gpu_target_freq;
	if (opp_entry[0] > GPU_PLL_MAX_KHZ)
		opp_entry[0] = GPU_PLL_MAX_KHZ;
	clamped_freq = opp_entry[0];
	opp_entry[1] = gpu_target_volt;
	opp_entry[2] = gpu_target_vsram;
	newly_patched |= 1;

	pr_info("KPM_OC: default_opp[0]: freq=%u->%u volt=%u->%u vsram=%u->%u\n",
		orig_freq, clamped_freq,
		orig_volt, gpu_target_volt,
		orig_vsram, gpu_target_vsram);

	/* Save anchor for diagnostics */
	if (!opp_table_anchor)
		opp_table_anchor = opp_table_addr;

	/* ── Steps 2-3: Patch runtime working/signed tables ── */
	newly_patched |= gpu_oc_patch_wk_ss();

	gpu_oc_patched |= newly_patched;
	pos = snprintf(gpu_oc_result, GPU_RESULT_BUF_SIZE,
		       "OK:patched=%d,freq=%u->%u,volt=%u->%u,vsram=%u->%u",
		       gpu_oc_patched, orig_freq, clamped_freq,
		       orig_volt, gpu_target_volt,
		       orig_vsram, gpu_target_vsram);

out:
	mutex_unlock(&lock);
	return 0;
}

static const struct kernel_param_ops gpu_oc_ops = { .set = set_gpu_oc };
static int gpu_oc_dummy;
module_param_cb(gpu_oc_apply, &gpu_oc_ops, &gpu_oc_dummy, 0220);
MODULE_PARM_DESC(gpu_oc_apply, "Write 1 to patch GPU OPP[0] in kernel memory");

/* ─── GPU Per-OPP Voltage Override (direct memory, no driver validation) ── */
/*
 * Input format: "idx:volt[:vsram] idx:volt[:vsram] ..."
 *   idx  = OPP index (0..GPU_MAX_OPPS-1)
 *   volt = voltage in 10µV steps (same unit as /proc/gpufreqv2 shows)
 *   vsram = optional, defaults to volt if omitted
 *
 * Example: "0 95000 95000 1 85000 85000" sets OPP[0] and OPP[1].
 * Write "clear" to remove all overrides.
 */
#define GPU_VOLT_OV_RESULT_SIZE 1024
static char gpu_volt_ov_result[GPU_VOLT_OV_RESULT_SIZE];
module_param_string(gpu_volt_ov_result, gpu_volt_ov_result,
		    sizeof(gpu_volt_ov_result), 0444);
MODULE_PARM_DESC(gpu_volt_ov_result, "Result of last GPU per-OPP voltage override");

static int __nocfi set_gpu_volt_ov(const char *val, const struct kernel_param *kp)
{
	char buf[1024];
	char *p, *tok;
	int pos = 0, count = 0;
	unsigned long def_addr;
	u32 *wt = NULL;

	if (!val || !*val)
		return -EINVAL;

	strscpy(buf, val, sizeof(buf));

	/* "clear" resets all overrides and restores original default table */
	if (strncmp(buf, "clear", 5) == 0) {
		if (resolve_kallsyms()) goto clear_arrays;
		gpu_resolve_runtime_symbols();
		mutex_lock(&lock);
		if (gpu_orig_saved && opp_table_anchor) {
			int i;
			u32 *wt_c = NULL;
			if (fn_get_working_table_gpu)
				wt_c = fn_get_working_table_gpu();
			if (!wt_c && fn_get_working_table_wrap)
				wt_c = fn_get_working_table_wrap(0);
			for (i = 0; i < GPU_MAX_OPPS; i++) {
				if (gpu_volt_ov[i] || gpu_vsram_ov[i]) {
					u32 *dt = (u32 *)(opp_table_anchor +
						  (unsigned long)i * GPU_OPP_STRIDE);
					dt[1] = gpu_orig_volt[i];
					dt[2] = gpu_orig_vsram[i];
					if (wt_c) {
						u32 *wte = wt_c + i * (GPU_OPP_STRIDE / 4);
						wte[1] = gpu_orig_volt[i];
						wte[2] = gpu_orig_vsram[i];
					}
				}
			}
			if (fn_update_shared_status_opp_table)
				fn_update_shared_status_opp_table();
			wmb();
		}
clear_arrays:
		if (!mutex_is_locked(&lock))
			mutex_lock(&lock);
		memset(gpu_volt_ov, 0, sizeof(gpu_volt_ov));
		memset(gpu_vsram_ov, 0, sizeof(gpu_vsram_ov));
		gpu_volt_ov_count = 0;
		snprintf(gpu_volt_ov_result, GPU_VOLT_OV_RESULT_SIZE, "cleared");
		mutex_unlock(&lock);
		return 0;
	}

	if (resolve_kallsyms()) {
		snprintf(gpu_volt_ov_result, GPU_VOLT_OV_RESULT_SIZE,
			 "FAIL:kallsyms");
		return 0;
	}
	gpu_resolve_runtime_symbols();

	mutex_lock(&lock);
	memset(gpu_volt_ov_result, 0, sizeof(gpu_volt_ov_result));

	def_addr = opp_table_anchor;
	if (!def_addr)
		def_addr = kln_func("g_gpu_default_opp_table");
	if (!def_addr) {
		pos = snprintf(gpu_volt_ov_result, GPU_VOLT_OV_RESULT_SIZE,
			       "FAIL:no_default_table");
		goto out_unlock;
	}
	if (!opp_table_anchor)
		opp_table_anchor = def_addr;

	/* Resolve working table */
	if (fn_get_working_table_gpu)
		wt = fn_get_working_table_gpu();
	if (!wt && fn_get_working_table_wrap)
		wt = fn_get_working_table_wrap(0);

	/* Save original default_opp_table voltages on first access */
	if (!gpu_orig_saved) {
		int i;
		for (i = 0; i < GPU_MAX_OPPS; i++) {
			u32 *e = (u32 *)(def_addr + (unsigned long)i * GPU_OPP_STRIDE);
			gpu_orig_volt[i] = e[1];
			gpu_orig_vsram[i] = e[2];
		}
		gpu_orig_saved = true;
		pr_info("KPM_OC: GPU original voltages saved (%d entries)\n",
			GPU_MAX_OPPS);
	}

	/* Parse: triplets "idx volt [vsram]" separated by spaces/newlines */
	p = buf;
	while ((tok = strsep(&p, " \t\n")) != NULL) {
		unsigned int idx, volt, vsram;
		u32 *dt, *wte;
		int nf;

		if (!*tok)
			continue;

		/* Try "idx:volt:vsram" or "idx:volt" (colon-separated) */
		nf = sscanf(tok, "%u:%u:%u", &idx, &volt, &vsram);
		if (nf < 2) {
			/* Also accept space-separated triplets: read next tokens */
			idx = simple_strtoul(tok, NULL, 10);
			tok = strsep(&p, " \t\n");
			if (!tok || !*tok) break;
			volt = simple_strtoul(tok, NULL, 10);
			tok = strsep(&p, " \t\n");
			vsram = (tok && *tok) ? simple_strtoul(tok, NULL, 10) : volt;
			nf = 3;
		}
		if (nf == 2)
			vsram = volt;

		if (idx >= GPU_MAX_OPPS) {
			pos += snprintf(gpu_volt_ov_result + pos,
					GPU_VOLT_OV_RESULT_SIZE - pos,
					"SKIP:%u(OOB),", idx);
			continue;
		}
		if (volt == 0) {
			/* Remove override for this index */
			gpu_volt_ov[idx] = 0;
			gpu_vsram_ov[idx] = 0;
			continue;
		}

		/* Store override (persisted for relift) */
		gpu_volt_ov[idx] = volt;
		gpu_vsram_ov[idx] = vsram;

		/* Immediate patch: default_opp_table */
		dt = (u32 *)(def_addr + (unsigned long)idx * GPU_OPP_STRIDE);
		pr_info("KPM_OC: gpu_volt_ov[%u] default: volt=%u->%u vsram=%u->%u\n",
			idx, dt[1], volt, dt[2], vsram);
		dt[1] = volt;
		dt[2] = vsram;

		/* Immediate patch: working_table */
		if (wt) {
			wte = wt + idx * (GPU_OPP_STRIDE / 4);
			pr_info("KPM_OC: gpu_volt_ov[%u] working: volt=%u->%u vsram=%u->%u\n",
				idx, wte[1], volt, wte[2], vsram);
			wte[1] = volt;
			wte[2] = vsram;
		}

		count++;
		pos += snprintf(gpu_volt_ov_result + pos,
				GPU_VOLT_OV_RESULT_SIZE - pos,
				"[%u]=%u/%u,", idx, volt, vsram);
	}

	/* Count active overrides */
	{
		int i, n = 0;
		for (i = 0; i < GPU_MAX_OPPS; i++)
			if (gpu_volt_ov[i])
				n++;
		gpu_volt_ov_count = n;
	}

	/* Sync to shared_status if available */
	if (count > 0 && fn_update_shared_status_opp_table)
		fn_update_shared_status_opp_table();
	wmb();

	if (count > 0 && pos > 0 && gpu_volt_ov_result[pos - 1] == ',')
		gpu_volt_ov_result[pos - 1] = '\0';
	if (count == 0)
		snprintf(gpu_volt_ov_result, GPU_VOLT_OV_RESULT_SIZE, "NOOP");

	pr_info("KPM_OC: GPU volt override: %d entries applied, %d total active\n",
		count, gpu_volt_ov_count);

out_unlock:
	mutex_unlock(&lock);
	return 0;
}

static const struct kernel_param_ops gpu_volt_ov_ops = { .set = set_gpu_volt_ov };
static int gpu_volt_ov_dummy;
module_param_cb(gpu_volt_override, &gpu_volt_ov_ops, &gpu_volt_ov_dummy, 0220);
MODULE_PARM_DESC(gpu_volt_override,
		 "Per-OPP GPU voltage override: 'idx:volt[:vsram] ...' (direct memory)");

/* ─── CPU Per-LUT Voltage Override (CSRAM direct write) ─────────────────── */
/*
 * Input format: "cluster:lut_idx:volt_uv cluster:lut_idx:volt_uv ..."
 *   cluster  = 0(L), 1(B), 2(P)
 *   lut_idx  = LUT entry index (0 = highest freq)
 *   volt_uv  = voltage in µV
 *
 * Write "clear" to remove all overrides.
 */
#define CPU_VOLT_OV_RESULT_SIZE 1024
static char cpu_volt_ov_result[CPU_VOLT_OV_RESULT_SIZE];
module_param_string(cpu_volt_ov_result, cpu_volt_ov_result,
		    sizeof(cpu_volt_ov_result), 0444);
MODULE_PARM_DESC(cpu_volt_ov_result, "Result of last CPU per-LUT voltage override");

static int set_cpu_volt_ov(const char *val, const struct kernel_param *kp)
{
	char buf[1024];
	char *p, *tok;
	int pos = 0, count = 0;

	if (!val || !*val)
		return -EINVAL;
	if (!csram_base) {
		snprintf(cpu_volt_ov_result, CPU_VOLT_OV_RESULT_SIZE,
			 "FAIL:csram_not_mapped");
		return 0;
	}

	strscpy(buf, val, sizeof(buf));

	if (strncmp(buf, "clear", 5) == 0) {
		int c, i;
		mutex_lock(&lock);
		/* Restore original CSRAM voltages */
		for (c = 0; c < NUM_CLUSTERS; c++)
			for (i = 0; i < LUT_MAX_ENTRIES; i++) {
				if (cpu_volt_ov[c][i] && cpu_orig_volt[c][i]) {
					unsigned int dom_base = domain_offsets[c];
					u32 cur = readl_relaxed(csram_base + dom_base +
						REG_FREQ_LUT + i * LUT_ROW_SIZE);
					u32 restored = (cur & (LUT_GEAR_MASK | LUT_FREQ_MASK)) |
						       ((cpu_orig_volt[c][i] / 10) << 12);
					writel_relaxed(restored, csram_base + dom_base +
						       REG_FREQ_LUT + i * LUT_ROW_SIZE);
				}
			}
		wmb();
		memset(cpu_volt_ov, 0, sizeof(cpu_volt_ov));
		cpu_volt_ov_count = 0;
		snprintf(cpu_volt_ov_result, CPU_VOLT_OV_RESULT_SIZE, "cleared");
		mutex_unlock(&lock);
		return 0;
	}

	mutex_lock(&lock);
	memset(cpu_volt_ov_result, 0, sizeof(cpu_volt_ov_result));

	p = buf;
	while ((tok = strsep(&p, " \t\n")) != NULL) {
		unsigned int cl, idx, volt_uv;
		unsigned int dom_base;
		u32 orig_lut, new_lut;
		unsigned int orig_volt_uv;

		if (!*tok)
			continue;

		if (sscanf(tok, "%u:%u:%u", &cl, &idx, &volt_uv) != 3) {
			pos += snprintf(cpu_volt_ov_result + pos,
					CPU_VOLT_OV_RESULT_SIZE - pos,
					"SKIP:parse(%s),", tok);
			continue;
		}
		if (cl >= NUM_CLUSTERS) {
			pos += snprintf(cpu_volt_ov_result + pos,
					CPU_VOLT_OV_RESULT_SIZE - pos,
					"SKIP:cl%u(OOB),", cl);
			continue;
		}
		if (idx >= LUT_MAX_ENTRIES) {
			pos += snprintf(cpu_volt_ov_result + pos,
					CPU_VOLT_OV_RESULT_SIZE - pos,
					"SKIP:idx%u(OOB),", idx);
			continue;
		}

		/* Save original voltage before first override */
		if (!cpu_orig_volt[cl][idx]) {
			u32 cur_lut = readl_relaxed(csram_base +
				domain_offsets[cl] + REG_FREQ_LUT +
				idx * LUT_ROW_SIZE);
			cpu_orig_volt[cl][idx] = LUT_VOLT_DECODE(cur_lut);
		}

		/* Store override (used for persistence check) */
		cpu_volt_ov[cl][idx] = volt_uv;

		/* Write directly to CSRAM */
		dom_base = domain_offsets[cl];
		orig_lut = readl_relaxed(csram_base + dom_base +
					 REG_FREQ_LUT + idx * LUT_ROW_SIZE);
		orig_volt_uv = LUT_VOLT_DECODE(orig_lut);

		/* Encode: keep gear bits[30:29] and freq bits[11:0],
		 * replace voltage bits */
		new_lut = (orig_lut & (LUT_GEAR_MASK | LUT_FREQ_MASK)) |
			  ((volt_uv / 10) << 12);

		writel_relaxed(new_lut, csram_base + dom_base +
			       REG_FREQ_LUT + idx * LUT_ROW_SIZE);

		pr_info("KPM_OC: cpu_volt_ov c%u[%u]: %uµV->%uµV (0x%08x->0x%08x)\n",
			cl, idx, orig_volt_uv, volt_uv, orig_lut, new_lut);

		/* Update OPP cache so get_opp_table reflects the change */
		if (opp_cache_valid && cl < NUM_CLUSTERS &&
		    (int)idx < opp_cache_count[cl])
			opp_cache[cl][idx].volt_uv = volt_uv;

		count++;
		pos += snprintf(cpu_volt_ov_result + pos,
				CPU_VOLT_OV_RESULT_SIZE - pos,
				"%s[%u]=%uuV,",
				cluster_names[cl], idx, volt_uv);
	}
	wmb();

	/* Count active overrides */
	{
		int c, i, n = 0;
		for (c = 0; c < NUM_CLUSTERS; c++)
			for (i = 0; i < LUT_MAX_ENTRIES; i++)
				if (cpu_volt_ov[c][i])
					n++;
		cpu_volt_ov_count = n;
	}

	if (count > 0 && pos > 0 && cpu_volt_ov_result[pos - 1] == ',')
		cpu_volt_ov_result[pos - 1] = '\0';
	if (count == 0)
		snprintf(cpu_volt_ov_result, CPU_VOLT_OV_RESULT_SIZE, "NOOP");

	pr_info("KPM_OC: CPU volt override: %d entries applied, %d total active\n",
		count, cpu_volt_ov_count);

	mutex_unlock(&lock);
	return 0;
}

static const struct kernel_param_ops cpu_volt_ov_ops = { .set = set_cpu_volt_ov };
static int cpu_volt_ov_dummy;
module_param_cb(cpu_volt_override, &cpu_volt_ov_ops, &cpu_volt_ov_dummy, 0220);
MODULE_PARM_DESC(cpu_volt_override,
		 "Per-LUT CPU voltage override: 'cl:idx:volt_uv ...' (CSRAM direct)");

/* ─── Proximity Screen Control ──────────────────────────────────────────── */
/*
 * Hook hf_manager_report_event() to detect proximity sensor events.
 * When the sensor reports "near" (value == 0), inject KEY_SLEEP to turn
 * the screen off.  When it reports "far" (value != 0), inject KEY_WAKEUP.
 *
 * hf_manager_event structure (76 bytes, from hf_manager.ko disassembly):
 *   +0x00  int64_t  timestamp
 *   +0x08  uint8_t  sensor_type  (8 = proximity)
 *   +0x09  uint8_t  action
 *   +0x0a  uint8_t  accuracy
 *   +0x0b  uint8_t  reserved
 *   +0x0c  int32_t  word[16]     (proximity: 0x00000000 = near/0.0f,
 *                                             0x40A00000 = far/5.0f)
 */

#define PROX_SENSOR_TYPE        8       /* MTK sensor type: proximity         */
#define PROX_DEBOUNCE_MS        300     /* ignore rapid near↔far transitions  */

static bool prox_screen_enabled;
module_param(prox_screen_enabled, bool, 0644);
MODULE_PARM_DESC(prox_screen_enabled,
		 "Enable proximity-based screen on/off (near→sleep, far→wake)");

static int prox_screen_state = -1;     /* -1=unknown, 0=near, 5=far */
module_param(prox_screen_state, int, 0444);
MODULE_PARM_DESC(prox_screen_state,
		 "Last proximity state (ro): -1=unknown, 0=near, 5=far");

static struct input_dev *prox_input_dev;
static struct work_struct prox_screen_work;
static unsigned long prox_last_change_jiffies;

static void prox_screen_work_fn(struct work_struct *work)
{
	int state = READ_ONCE(prox_screen_state);

	if (!prox_input_dev)
		return;

	if (state == 0) {
		input_report_key(prox_input_dev, KEY_SLEEP, 1);
		input_sync(prox_input_dev);
		input_report_key(prox_input_dev, KEY_SLEEP, 0);
		input_sync(prox_input_dev);
		pr_info("KPM_OC: proximity NEAR → screen off\n");
	} else if (state == 5) {
		input_report_key(prox_input_dev, KEY_WAKEUP, 1);
		input_sync(prox_input_dev);
		input_report_key(prox_input_dev, KEY_WAKEUP, 0);
		input_sync(prox_input_dev);
		pr_info("KPM_OC: proximity FAR → screen on\n");
	}
}

static int __nocfi prox_report_kp_pre(struct kprobe *p, struct pt_regs *regs)
{
	u64 ev;
	u8 stype;
	u32 raw;
	int new_state;

	if (!READ_ONCE(prox_screen_enabled))
		return 0;

	ev = regs->regs[1];   /* x1 = event struct pointer */

	stype = *(volatile u8 *)(ev + 0x08);
	if (stype != PROX_SENSOR_TYPE)
		return 0;

	raw = *(volatile u32 *)(ev + 0x0c);
	new_state = (raw == 0) ? 0 : 5;

	if (new_state == READ_ONCE(prox_screen_state))
		return 0;

	if (time_before(jiffies,
			prox_last_change_jiffies +
			msecs_to_jiffies(PROX_DEBOUNCE_MS)))
		return 0;

	WRITE_ONCE(prox_screen_state, new_state);
	prox_last_change_jiffies = jiffies;
	schedule_work(&prox_screen_work);

	return 0;
}

static struct kprobe prox_report_kp = {
	.symbol_name = "hf_manager_report_event",
	.pre_handler = prox_report_kp_pre,
};

/* ─── Module Init / Exit ────────────────────────────────────────────────── */

static int __init kpm_oc_init(void)
{
	csram_base = ioremap(CSRAM_PHYS_BASE, CSRAM_PHYS_SIZE);
	if (!csram_base) {
		pr_err("KPM_OC: Failed to ioremap CSRAM at 0x%lx\n",
		       CSRAM_PHYS_BASE);
		return -ENOMEM;
	}

	/* ioremap MCU PLL subsystem for direct ARMPLL access */
	mcu_pll_base = ioremap(MCU_PLL_PHYS_BASE, MCU_PLL_PHYS_SIZE);
	if (!mcu_pll_base)
		pr_warn("KPM_OC: Failed to ioremap MCU PLL at 0x%lx (PLL bypass disabled)\n",
			MCU_PLL_PHYS_BASE);
	else
		pr_info("KPM_OC: MCU PLL region mapped at %px (phys 0x%lx, %lu bytes)\n",
			mcu_pll_base, MCU_PLL_PHYS_BASE, MCU_PLL_PHYS_SIZE);

	/* ioremap MCUPM SRAM for frequency table scanning */
	mcupm_sram_base = ioremap(MCUPM_SRAM_PHYS_BASE, MCUPM_SRAM_PHYS_SIZE);
	if (!mcupm_sram_base)
		pr_warn("KPM_OC: Failed to ioremap MCUPM SRAM at 0x%lx\n",
			MCUPM_SRAM_PHYS_BASE);
	else
		pr_info("KPM_OC: MCUPM SRAM mapped at %px (phys 0x%lx, %lu bytes)\n",
			mcupm_sram_base, MCUPM_SRAM_PHYS_BASE,
			MCUPM_SRAM_PHYS_SIZE);

	/* ioremap MCUPM reserved DRAM for frequency table scanning */
	mcupm_dram_base = ioremap(MCUPM_DRAM_PHYS_BASE, MCUPM_DRAM_PHYS_SIZE);
	if (!mcupm_dram_base)
		pr_warn("KPM_OC: Failed to ioremap MCUPM DRAM at 0x%lx\n",
			MCUPM_DRAM_PHYS_BASE);
	else
		pr_info("KPM_OC: MCUPM DRAM mapped at %px (phys 0x%lx, %lu bytes)\n",
			mcupm_dram_base, MCUPM_DRAM_PHYS_BASE,
			MCUPM_DRAM_PHYS_SIZE);

	snprintf(opp_table_export, sizeof(opp_table_export), "READY");
	INIT_DELAYED_WORK(&cpu_oc_relift_dwork, cpu_oc_relift_work_fn);
	pr_info("KPM_OC: MT8792 CSRAM OPP reader + CPU/GPU OC v8.1 (base=0x%lx)\n",
		CSRAM_PHYS_BASE);

	/* Register kprobes for freq_qos and userlimit interception */
	{
		int kret = register_kprobe(&freq_qos_kp);

		if (kret < 0)
			pr_warn("KPM_OC: freq_qos kprobe failed (%d); periodic relift only\n",
				kret);
		else
			pr_info("KPM_OC: freq_qos_update_request kprobe registered\n");
	}

	{
		int kret2 = register_kprobe(&userlimit_kp);

		if (kret2 < 0)
			pr_warn("KPM_OC: userlimit kprobe failed (%d); powerhal only patched on relift\n",
				kret2);
		else
			pr_info("KPM_OC: update_userlimit_cpufreq_max kprobe registered\n");
	}

	/* SCMI cpufreq DVFS kprobes: intercept frequency transitions for
	 * CSRAM voltage resync and PLL re-enforcement triggering.
	 * On MT6897 this SoC uses scmi_cpufreq (not mtk-cpufreq-hw).
	 */
	{
		int kret3 = register_kprobe(&scmi_fast_switch_kp);

		if (kret3 < 0)
			pr_warn("KPM_OC: scmi_cpufreq_fast_switch kprobe failed (%d)\n",
				kret3);
		else
			pr_info("KPM_OC: scmi_cpufreq_fast_switch kprobe registered\n");
	}

	{
		int kret4 = register_kprobe(&scmi_set_target_kp);

		if (kret4 < 0)
			pr_warn("KPM_OC: scmi_cpufreq_set_target kprobe failed (%d)\n",
				kret4);
		else
			pr_info("KPM_OC: scmi_cpufreq_set_target kprobe registered\n");
	}

	/* GPUEB countermeasure: kprobe on GPU DVFS commit to re-patch OPP
	 * voltages immediately before the commit reads them.
	 */
	{
		int kret5 = register_kprobe(&gpu_commit_kp);

		if (kret5 < 0)
			pr_warn("KPM_OC: gpu_commit kprobe failed (%d)\n",
				kret5);
		else
			pr_info("KPM_OC: __gpufreq_generic_commit_gpu kprobe registered (GPUEB countermeasure)\n");
	}

	/* MFG PLL direct access for above-stock GPU OC.
	 * ioremap both MFG PLL and MFGSC PLL registers so gpu_pll_set_freq()
	 * can reprogram clock after GPUEB commits.
	 */
	mfg_pll_virt = ioremap(MFG_PLL_PHYS_BASE, MFG_PLL_PHYS_SIZE);
	if (mfg_pll_virt)
		pr_info("KPM_OC: MFG PLL mapped at %px (phys 0x%lx)\n",
			mfg_pll_virt, MFG_PLL_PHYS_BASE);
	else
		pr_warn("KPM_OC: MFG PLL ioremap failed\n");

	mfgsc_pll_virt = ioremap(MFGSC_PLL_PHYS_BASE, MFGSC_PLL_PHYS_SIZE);
	if (mfgsc_pll_virt)
		pr_info("KPM_OC: MFGSC PLL mapped at %px (phys 0x%lx)\n",
			mfgsc_pll_virt, MFGSC_PLL_PHYS_BASE);
	else
		pr_warn("KPM_OC: MFGSC PLL ioremap failed\n");

	/* kretprobe on gpufreq_commit (wrapper): reprogram PLL after every
	 * GPUEB commit to OPP[0] for above-stock GPU OC.
	 */
	{
		int kret_kretp = register_kretprobe(&gpu_commit_kretp);

		if (kret_kretp < 0)
			pr_warn("KPM_OC: gpufreq_commit kretprobe failed (%d)\n",
				kret_kretp);
		else
			pr_info("KPM_OC: gpufreq_commit kretprobe registered (PLL override)\n");
	}

	/* FHCTL mcupm_hopping_v1 argument capture kprobe */
	{
		int kret6 = register_kprobe(&fhctl_hop_kp);

		if (kret6 < 0)
			pr_warn("KPM_OC: mcupm_hopping_v1 kprobe failed (%d)\n",
				kret6);
		else
			pr_info("KPM_OC: mcupm_hopping_v1 kprobe registered (DDS capture)\n");
	}

	/* SCMI perf_level_set kprobe for capturing/overriding performance levels */
	{
		int kret7 = register_kprobe(&scmi_perf_level_kp);

		if (kret7 < 0)
			pr_warn("KPM_OC: scmi_perf_level_set kprobe failed (%d)\n",
				kret7);
		else
			pr_info("KPM_OC: scmi_perf_level_set kprobe registered (level capture+override)\n");
	}

	/* SCMI dvfs_freq_set kprobe */
	{
		int kret8 = register_kprobe(&scmi_dvfs_freq_kp);

		if (kret8 < 0)
			pr_warn("KPM_OC: scmi_dvfs_freq_set kprobe failed (%d)\n",
				kret8);
		else
			pr_info("KPM_OC: scmi_dvfs_freq_set kprobe registered\n");
	}

	/* Auto-scan CPU OPP on load */
	set_apply("1", NULL);

	/* Auto-apply CPU OC if any cluster target is set */
	if (cpu_oc_l_freq || cpu_oc_b_freq || cpu_oc_p_freq)
		set_cpu_oc("1", NULL);

	/* Auto-apply GPU OC: always patches default OPP; working+shared if already probed */
	set_gpu_oc("1", NULL);

	/*
	 * Start GPU relift kthread for module lifetime so runtime table refreshes
	 * from vendor/GPUEB side cannot silently drop OC back to stock.
	 */
	if (gpu_target_freq > 0) {
		gpu_oc_task = kthread_run(gpu_oc_kthread_fn, NULL,
					  "gpu_oc_worker");
		if (IS_ERR(gpu_oc_task)) {
			pr_warn("KPM_OC: kthread start failed: %ld\n",
				PTR_ERR(gpu_oc_task));
			gpu_oc_task = NULL;
		} else {
			pr_info("KPM_OC: delayed-patch kthread launched\n");
		}
	}

	/* ─── Proximity screen control: virtual input device + kprobe ──── */
	INIT_WORK(&prox_screen_work, prox_screen_work_fn);
	prox_input_dev = input_allocate_device();
	if (prox_input_dev) {
		prox_input_dev->name = "kpm_oc_proximity";
		prox_input_dev->id.bustype = BUS_VIRTUAL;
		set_bit(EV_KEY, prox_input_dev->evbit);
		set_bit(KEY_SLEEP, prox_input_dev->keybit);
		set_bit(KEY_WAKEUP, prox_input_dev->keybit);
		if (input_register_device(prox_input_dev)) {
			input_free_device(prox_input_dev);
			prox_input_dev = NULL;
			pr_warn("KPM_OC: proximity input device registration failed\n");
		}
	}
	{
		int kret_prox = register_kprobe(&prox_report_kp);

		if (kret_prox < 0)
			pr_warn("KPM_OC: hf_manager_report_event kprobe failed (%d)\n",
				kret_prox);
		else
			pr_info("KPM_OC: proximity screen control kprobe registered\n");
	}

	return 0;
}

static void __exit kpm_oc_exit(void)
{
	unregister_kprobe(&prox_report_kp);
	cancel_work_sync(&prox_screen_work);
	if (prox_input_dev)
		input_unregister_device(prox_input_dev);
	unregister_kprobe(&freq_qos_kp);
	unregister_kprobe(&userlimit_kp);
	unregister_kprobe(&scmi_fast_switch_kp);
	unregister_kprobe(&scmi_set_target_kp);
	unregister_kprobe(&gpu_commit_kp);
	unregister_kretprobe(&gpu_commit_kretp);
	unregister_kprobe(&fhctl_hop_kp);
	unregister_kprobe(&scmi_perf_level_kp);
	unregister_kprobe(&scmi_dvfs_freq_kp);
	cpu_oc_relift_active = false;
	cancel_delayed_work_sync(&cpu_oc_relift_dwork);
	if (gpu_oc_task) {
		kthread_stop(gpu_oc_task);
		gpu_oc_task = NULL;
	}
	if (csram_base)
		iounmap(csram_base);
	if (mcu_pll_base)
		iounmap(mcu_pll_base);
	if (mcupm_sram_base)
		iounmap(mcupm_sram_base);
	if (mcupm_dram_base)
		iounmap(mcupm_dram_base);
	if (mfg_pll_virt)
		iounmap(mfg_pll_virt);
	if (mfgsc_pll_virt)
		iounmap(mfgsc_pll_virt);
	pr_info("KPM_OC: Unloaded.\n");
}

module_init(kpm_oc_init);
module_exit(kpm_oc_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zerofrip");
MODULE_DESCRIPTION("MT8792 CSRAM CPU OPP reader + CPU/GPU OC patcher + PLL bypass v8.0");

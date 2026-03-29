// SPDX-License-Identifier: GPL-2.0
/*
 * KPM OC Module for MediaTek MT8792 (Dimensity 8300)
 * - Reads CPU DVFS LUT from performance-controller CSRAM domains.
 * - CPU OC: patches CSRAM LUT[0] per cluster + cpufreq policy max.
 * - GPU OC: patches default + working + shared_status OPP tables.
 *   v6.4: CPU OC support added.
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

static int set_cpu_oc(const char *val, const struct kernel_param *kp)
{
	/* Indexed by cluster order: L=0, B=1, P=2 (matches cluster_policies[]) */
	static unsigned int * const tgt_freqs[] = {
		&cpu_oc_l_freq, &cpu_oc_b_freq, &cpu_oc_p_freq
	};
	static unsigned int * const tgt_volts[] = {
		&cpu_oc_l_volt, &cpu_oc_b_volt, &cpu_oc_p_volt
	};
	int c, pos = 0, any_patched = 0;

	if (!csram_base) {
		pr_err("KPM_OC: CSRAM not mapped\n");
		return -ENOMEM;
	}

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

		if (orig_freq_mhz < 100 || orig_freq_mhz > 5000) {
			pr_warn("KPM_OC: CPU %s LUT[0] unexpected freq=%u MHz\n",
				cluster_names[c], orig_freq_mhz);
			pos += snprintf(cpu_oc_result + pos, CPU_RESULT_BUF_SIZE - pos,
					"%s=BAD_ORIG,", cluster_names[c]);
			continue;
		}

		new_freq_mhz = target_khz / 1000;
		if (new_freq_mhz > 5000) {
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

		/* ── Update cpufreq policy so governor can target new max freq ── */
		{
			struct cpufreq_policy *policy =
				cpufreq_cpu_get(cluster_policies[c]);
			if (policy) {
				struct cpufreq_frequency_table *tbl = policy->freq_table;
				if (tbl) {
					int t, max_idx = 0;
					unsigned int tbl_max = 0;

					for (t = 0; tbl[t].frequency != CPUFREQ_TABLE_END; t++) {
						if (tbl[t].frequency != CPUFREQ_ENTRY_INVALID &&
						    tbl[t].frequency > tbl_max) {
							tbl_max = tbl[t].frequency;
							max_idx = t;
						}
					}
					tbl[max_idx].frequency = target_khz;
				}
				policy->max                 = target_khz;
				policy->cpuinfo.max_freq    = target_khz;
				cpufreq_cpu_put(policy);
				pr_info("KPM_OC: CPU %s cpufreq policy->max updated to %uKHz\n",
					cluster_names[c], target_khz);
			}
		}
	}

	if (pos > 0 && cpu_oc_result[pos - 1] == ',')
		cpu_oc_result[pos - 1] = '\0';
	if (any_patched == 0 && pos == 0)
		snprintf(cpu_oc_result, CPU_RESULT_BUF_SIZE,
			 "NOOP:no cluster targets set");

	mutex_unlock(&lock);
	return 0;
}

static const struct kernel_param_ops cpu_oc_ops = { .set = set_cpu_oc };
static int cpu_oc_dummy;
module_param_cb(cpu_oc_apply, &cpu_oc_ops, &cpu_oc_dummy, 0220);
MODULE_PARM_DESC(cpu_oc_apply, "Write 1 to patch CPU CSRAM LUT[0] + cpufreq policy max");

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

/* Configurable via module params */
static unsigned int gpu_target_freq = 1450000;
module_param(gpu_target_freq, uint, 0644);
MODULE_PARM_DESC(gpu_target_freq, "Target GPU freq for OPP[0] in KHz (default 1450000)");

static unsigned int gpu_target_volt = 87500;
module_param(gpu_target_volt, uint, 0644);
MODULE_PARM_DESC(gpu_target_volt, "Target GPU volt for OPP[0] in µV (default 87500)");

static unsigned int gpu_target_vsram = 87500;
module_param(gpu_target_vsram, uint, 0644);
MODULE_PARM_DESC(gpu_target_vsram, "Target GPU vsram for OPP[0] in µV (default 87500)");

static char gpu_oc_result[GPU_RESULT_BUF_SIZE];
module_param_string(gpu_oc_result, gpu_oc_result, sizeof(gpu_oc_result), 0444);
MODULE_PARM_DESC(gpu_oc_result, "Result of last GPU OC patch operation");

/* kallsyms_lookup_name via kprobes (GKI 6.1 compatible) */
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

/* Saved only for diagnostics; runtime patching no longer relies on this. */
static unsigned long opp_table_anchor;
static gpufreq_get_table_t fn_get_working_table_gpu;
static gpufreq_get_table_t fn_get_signed_table_gpu;
static gpufreq_get_table_tgt_t fn_get_working_table_wrap;
static gpufreq_get_table_tgt_t fn_get_signed_table_wrap;
static gpufreq_update_shared_t fn_update_shared_status_opp_table;

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

/* Kthread for delayed patch (polls until GPU probe completes) */
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
}

static int gpu_patch_table_opp0(u32 *tbl, const char *name, int bit)
{
    u32 freq;

    if (!tbl)
        return 0;

    freq = tbl[0];
    if (freq < 1000000 || freq > 2000000) {
        pr_warn("KPM_OC: %s[0] unexpected freq=%u, skip\n", name, freq);
        return 0;
    }

    pr_info("KPM_OC: %s[0] at %px freq=%u -> %u\n",
        name, tbl, freq, gpu_target_freq);
    tbl[0] = gpu_target_freq;
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

/*
 * Kthread function: polls every 500 ms until working_table and shared_status
 * are both patched, or until 120 seconds have elapsed.
 */
static int gpu_oc_kthread_fn(void *data)
{
	int tries = 0;
	bool diag_done = false;

	pr_info("KPM_OC: delayed-patch kthread started (max 120 s)\n");

	while (!kthread_should_stop() && tries < 240) {
		msleep(500);
		tries++;

		if ((gpu_oc_patched & 6) == 6)
			break;

		mutex_lock(&lock);
		gpu_oc_patched |= gpu_oc_patch_wk_ss();
		mutex_unlock(&lock);

		/* After 35 s, dump diagnostics once if still not fully patched */
		if (!diag_done && tries >= 70 && (gpu_oc_patched & 6) != 6) {
			mutex_lock(&lock);
			gpu_oc_diagnostic();
			mutex_unlock(&lock);
			diag_done = true;
		}
	}

	if ((gpu_oc_patched & 6) == 6) {
		pr_info("KPM_OC: kthread: GPU OC fully applied after %d*500ms (patched=%d)\n",
			tries, gpu_oc_patched);
		/* Update result string */
		mutex_lock(&lock);
		snprintf(gpu_oc_result, GPU_RESULT_BUF_SIZE,
			 "OK:patched=%d,freq=%u,volt=%u,vsram=%u (kthread)",
			 gpu_oc_patched, gpu_target_freq,
			 gpu_target_volt, gpu_target_vsram);
		mutex_unlock(&lock);
	} else {
		pr_warn("KPM_OC: kthread: timed out, patched=%d\n",
			gpu_oc_patched);
	}

	/* Block until kthread_stop() is called from module exit.
	 * Returning while gpu_oc_task is still set would cause
	 * kthread_stop() to dereference a freed task_struct.
	 */
	while (!kthread_should_stop())
		msleep(500);

	return 0;
}

static int set_gpu_oc(const char *val, const struct kernel_param *kp)
{
	unsigned long opp_table_addr;
	u32 *opp_entry;
	u32 orig_freq, orig_volt, orig_vsram;
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

	if (orig_freq < 1000000 || orig_freq > 2000000) {
		pos = snprintf(gpu_oc_result, GPU_RESULT_BUF_SIZE,
			       "FAIL:unexpected_freq=%u", orig_freq);
		goto out;
	}

	opp_entry[0] = gpu_target_freq;
	opp_entry[1] = gpu_target_volt;
	opp_entry[2] = gpu_target_vsram;
	newly_patched |= 1;

	pr_info("KPM_OC: default_opp[0]: freq=%u->%u volt=%u->%u vsram=%u->%u\n",
		orig_freq, gpu_target_freq,
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
		       gpu_oc_patched, orig_freq, gpu_target_freq,
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
	pr_info("KPM_OC: MT8792 CSRAM OPP reader + GPU OC v6.3 (base=0x%lx)\n",
		CSRAM_PHYS_BASE);

	/* Auto-scan CPU OPP on load */
	set_apply("1", NULL);

	/* Auto-apply CPU OC if any cluster target is set */
	if (cpu_oc_l_freq || cpu_oc_b_freq || cpu_oc_p_freq)
		set_cpu_oc("1", NULL);

	/* Auto-apply GPU OC: always patches default OPP; working+shared if already probed */
	set_gpu_oc("1", NULL);

	/*
	 * If runtime tables are still unavailable (GPU probe not yet done),
	 * start a kthread that polls until working+signed tables are patched.
	 */
	if ((gpu_oc_patched & 6) != 6) {
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

	return 0;
}

static void __exit kpm_oc_exit(void)
{
	if (gpu_oc_task) {
		kthread_stop(gpu_oc_task);
		gpu_oc_task = NULL;
	}
	if (csram_base)
		iounmap(csram_base);
	pr_info("KPM_OC: Unloaded.\n");
}

module_init(kpm_oc_init);
module_exit(kpm_oc_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zerofrip");
MODULE_DESCRIPTION("MT8792 CSRAM CPU OPP reader + CPU/GPU OC patcher v6.4");

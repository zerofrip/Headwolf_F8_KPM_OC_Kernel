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
			struct cpufreq_policy *policy = fn_cpufreq_cpu_get ?
				fn_cpufreq_cpu_get(cluster_policies[c]) : NULL;
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
}

static int gpu_patch_table_opp0(u32 *tbl, const char *name, int bit)
{
	u32 freq;
	u32 volt;
	u32 vsram;

	if (!tbl)
		return 0;

	freq = tbl[0];
	volt = tbl[1];
	vsram = tbl[2];
	if (freq < 1000000 || freq > 2000000) {
		pr_warn("KPM_OC: %s[0] unexpected freq=%u, skip\n", name, freq);
		return 0;
	}

	if (freq == gpu_target_freq &&
	    volt == gpu_target_volt &&
	    vsram == gpu_target_vsram)
		return bit;

	pr_info("KPM_OC: %s[0] at %px freq=%u->%u volt=%u->%u vsram=%u->%u\n",
		name, tbl, freq, gpu_target_freq, volt, gpu_target_volt,
		vsram, gpu_target_vsram);
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

/*
 * Kthread function: runs for the module lifetime and periodically re-applies
 * working/signed table patching.  This keeps GPU OC active even after GPU
 * power-cycle or vendor-side runtime table refresh.
 */
static int gpu_oc_kthread_fn(void *data)
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
		now = gpu_oc_patched & 6;
		mutex_unlock(&lock);

		if (now == 6 && last_state != 6)
			pr_info("KPM_OC: GPU relift: working+signed patched (patched=%d)\n",
				gpu_oc_patched);

		if (now != 6) {
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
	INIT_DELAYED_WORK(&cpu_oc_relift_dwork, cpu_oc_relift_work_fn);
	pr_info("KPM_OC: MT8792 CSRAM OPP reader + CPU/GPU OC v7.0 (base=0x%lx)\n",
		CSRAM_PHYS_BASE);

	/* Register kprobe to intercept freq_qos_update_request at call time */
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

	return 0;
}

static void __exit kpm_oc_exit(void)
{
	unregister_kprobe(&freq_qos_kp);
	unregister_kprobe(&userlimit_kp);
	cpu_oc_relift_active = false;
	cancel_delayed_work_sync(&cpu_oc_relift_dwork);
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
MODULE_DESCRIPTION("MT8792 CSRAM CPU OPP reader + CPU/GPU OC patcher v7.0");

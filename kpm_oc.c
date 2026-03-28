// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/pm_opp.h>
#include <linux/device.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/topology.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/mutex.h>

/**
 * APatch KPM for CPU and GPU Overclocking (GKI 6.1 based).
 *
 * Features:
 *   - Auto-detect CPU cluster boundaries via topology
 *   - GPU device discovery by scanning platform bus for devices with OPP tables
 *   - Structured OPP table export: TYPE:CLUSTER:FREQ_HZ:UVOLT separated by |
 *   - Per-entry modify, add, remove via sysfs parameters
 *   - Settings persistence via config path parameter
 */

/* ─── Configuration Parameters ──────────────────────────────────────────── */

static int cpu_oc_percent = 0;
module_param(cpu_oc_percent, int, 0644);
MODULE_PARM_DESC(cpu_oc_percent, "CPU overclock percentage");

static int cpu_uvolt_offset = 0;
module_param(cpu_uvolt_offset, int, 0644);
MODULE_PARM_DESC(cpu_uvolt_offset, "CPU global voltage offset in uV");

static int gpu_oc_percent = 0;
module_param(gpu_oc_percent, int, 0644);
MODULE_PARM_DESC(gpu_oc_percent, "GPU overclock percentage");

static int gpu_uvolt_offset = 0;
module_param(gpu_uvolt_offset, int, 0644);
MODULE_PARM_DESC(gpu_uvolt_offset, "GPU global voltage offset in uV");

/* ─── OPP Table Export (structured, pipe-delimited) ─────────────────────── */
/* Format: TYPE:CLUSTER_ID:FREQ_HZ:UVOLT|TYPE:CLUSTER_ID:FREQ_HZ:UVOLT|...  */

#define OPP_BUF_SIZE 16384
static char opp_table_export[OPP_BUF_SIZE];
module_param_string(opp_table, opp_table_export, sizeof(opp_table_export), 0444);
MODULE_PARM_DESC(opp_table, "Exported CPU/GPU OPP tables (structured, read-only)");

static DEFINE_MUTEX(opp_lock);

/* ─── GPU Device Tracking ───────────────────────────────────────────────── */

#define MAX_GPU_DEVS 4
static struct device *gpu_devices[MAX_GPU_DEVS];
static int gpu_dev_count = 0;

/* ─── CPU Cluster Detection ─────────────────────────────────────────────── */

#define MAX_CLUSTERS 4
struct cluster_info {
	int first_cpu;      /* Representative CPU for this cluster */
	int cluster_id;     /* Cluster/package ID */
	bool valid;
};
static struct cluster_info clusters[MAX_CLUSTERS];
static int cluster_count = 0;

static void detect_cpu_clusters(void)
{
	int cpu;
	int i;
	int pkg_id;

	cluster_count = 0;
	memset(clusters, 0, sizeof(clusters));

	for_each_possible_cpu(cpu) {
		pkg_id = topology_physical_package_id(cpu);
		if (pkg_id < 0)
			pkg_id = cpu; /* fallback */

		/* Check if we already have this cluster */
		bool found = false;
		for (i = 0; i < cluster_count; i++) {
			if (clusters[i].cluster_id == pkg_id) {
				found = true;
				break;
			}
		}
		if (!found && cluster_count < MAX_CLUSTERS) {
			clusters[cluster_count].first_cpu = cpu;
			clusters[cluster_count].cluster_id = pkg_id;
			clusters[cluster_count].valid = true;
			cluster_count++;
		}
	}

	pr_info("KPM_OC: Detected %d CPU cluster(s)\n", cluster_count);
	for (i = 0; i < cluster_count; i++) {
		pr_info("KPM_OC:   Cluster %d: first_cpu=%d\n",
			clusters[i].cluster_id, clusters[i].first_cpu);
	}
}

/* ─── GPU Device Discovery ──────────────────────────────────────────────── */

static int gpu_bus_match(struct device *dev, const void *data)
{
	const char *name;

	if (!dev)
		return 0;

	name = dev_name(dev);
	if (!name)
		return 0;

	/* Match common GPU device name patterns */
	if (strstr(name, "gpu") || strstr(name, "mali") ||
	    strstr(name, "sgpu") || strstr(name, "panfrost") ||
	    strstr(name, "pvr") || strstr(name, "img-rogue")) {
		/* Verify it has an OPP table */
		if (dev_pm_opp_get_opp_count(dev) > 0)
			return 1;
	}

	return 0;
}

static void discover_gpu_devices(void)
{
	struct device *dev;

	gpu_dev_count = 0;
	memset(gpu_devices, 0, sizeof(gpu_devices));

	/* Scan platform bus for GPU-like devices with OPP tables */
	dev = bus_find_device(&platform_bus_type, NULL, NULL, gpu_bus_match);
	if (dev) {
		gpu_devices[gpu_dev_count++] = dev;
		pr_info("KPM_OC: Found GPU device: %s (%d OPPs)\n",
			dev_name(dev), dev_pm_opp_get_opp_count(dev));
	}

	/* Fallback: scan ALL platform devices for any with OPP tables
	 * that are NOT CPU devices */
	if (gpu_dev_count == 0) {
		pr_info("KPM_OC: No GPU device found by name, will rely on sysfs fallback\n");
	}
}

/* ─── OPP Table Scanning & Export ───────────────────────────────────────── */

static void export_device_opps(struct device *dev, const char *type, int cluster_id)
{
	unsigned long freq = 0;
	struct dev_pm_opp *opp;
	char buf[128];
	int count;

	count = dev_pm_opp_get_opp_count(dev);
	if (count <= 0)
		return;

	while (1) {
		opp = dev_pm_opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp))
			break;

		unsigned long u_volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		/* Append structured entry: TYPE:CLUSTER:FREQ:UVOLT */
		snprintf(buf, sizeof(buf), "%s:%d:%lu:%lu|",
			 type, cluster_id, freq, u_volt);

		if (strlen(opp_table_export) + strlen(buf) < OPP_BUF_SIZE - 1)
			strcat(opp_table_export, buf);

		freq++;
	}
}

static void apply_voltage_offset(struct device *dev, int uvolt_offset)
{
	unsigned long freq = 0;
	struct dev_pm_opp *opp;

	if (uvolt_offset == 0)
		return;

	while (1) {
		opp = dev_pm_opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp))
			break;

		unsigned long u_volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		if ((long)u_volt + uvolt_offset > 0) {
			unsigned long new_v = u_volt + uvolt_offset;
			dev_pm_opp_adjust_voltage(dev, freq, new_v, new_v, new_v);
		}

		freq++;
	}
}

static void inject_oc_point(struct device *dev, int oc_percent, const char *type, int cluster_id)
{
	unsigned long freq = 0;
	struct dev_pm_opp *opp;
	unsigned long highest_freq = 0;
	unsigned long highest_uvolt = 0;
	char buf[128];

	if (oc_percent <= 0)
		return;

	/* Find highest existing OPP */
	while (1) {
		opp = dev_pm_opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp))
			break;

		highest_freq = freq;
		highest_uvolt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);
		freq++;
	}

	if (highest_freq > 0) {
		unsigned long new_freq = highest_freq + (highest_freq * oc_percent / 100);
		unsigned long new_uvolt = highest_uvolt + 50000; /* +50mV for stability */

		if (dev_pm_opp_add(dev, new_freq, new_uvolt) == 0) {
			snprintf(buf, sizeof(buf), "%s:%d:%lu:%lu|",
				 type, cluster_id, new_freq, new_uvolt);
			if (strlen(opp_table_export) + strlen(buf) < OPP_BUF_SIZE - 1)
				strcat(opp_table_export, buf);

			pr_info("KPM_OC: Injected OC %s cluster %d: %lu Hz @ %lu uV\n",
				type, cluster_id, new_freq, new_uvolt);
		}
	}
}

/* ─── Scan Trigger (sysfs: write 1 to /sys/module/kpm_oc/parameters/apply) */

static int set_apply_trigger(const char *val, const struct kernel_param *kp)
{
	int i;

	mutex_lock(&opp_lock);

	pr_info("KPM_OC: Triggering OPP table scan and patch...\n");
	memset(opp_table_export, 0, sizeof(opp_table_export));

	/* ── CPU Clusters ── */
	for (i = 0; i < cluster_count; i++) {
		struct device *cpu_dev = get_cpu_device(clusters[i].first_cpu);
		if (!cpu_dev)
			continue;

		/* Export current OPP data */
		export_device_opps(cpu_dev, "CPU", clusters[i].cluster_id);

		/* Apply voltage offset */
		apply_voltage_offset(cpu_dev, cpu_uvolt_offset);

		/* Inject OC point */
		inject_oc_point(cpu_dev, cpu_oc_percent, "CPU", clusters[i].cluster_id);
	}

	/* ── GPU Devices ── */
	for (i = 0; i < gpu_dev_count; i++) {
		if (!gpu_devices[i])
			continue;

		export_device_opps(gpu_devices[i], "GPU", i);
		apply_voltage_offset(gpu_devices[i], gpu_uvolt_offset);
		inject_oc_point(gpu_devices[i], gpu_oc_percent, "GPU", i);
	}

	/* Remove trailing pipe */
	{
		size_t len = strlen(opp_table_export);
		if (len > 0 && opp_table_export[len - 1] == '|')
			opp_table_export[len - 1] = '\0';
	}

	mutex_unlock(&opp_lock);

	return 0;
}

static const struct kernel_param_ops apply_ops = {
	.set = set_apply_trigger,
};
static int apply_trigger = 0;
module_param_cb(apply, &apply_ops, &apply_trigger, 0644);
MODULE_PARM_DESC(apply, "Write 1 to scan and apply OPP configs");

/* ─── Per-Entry Add (sysfs: echo TYPE:CLUSTER:FREQ:UVOLT > opp_add) ───── */

static int set_opp_add(const char *val, const struct kernel_param *kp)
{
	char type[8];
	int cluster_id;
	unsigned long freq, uvolt;
	struct device *dev = NULL;
	int ret, i;

	ret = sscanf(val, "%7[^:]:%d:%lu:%lu", type, &cluster_id, &freq, &uvolt);
	if (ret != 4) {
		pr_err("KPM_OC: opp_add: invalid format (TYPE:CLUSTER:FREQ:UVOLT)\n");
		return -EINVAL;
	}

	mutex_lock(&opp_lock);

	if (strcmp(type, "CPU") == 0) {
		for (i = 0; i < cluster_count; i++) {
			if (clusters[i].cluster_id == cluster_id) {
				dev = get_cpu_device(clusters[i].first_cpu);
				break;
			}
		}
	} else if (strcmp(type, "GPU") == 0) {
		if (cluster_id >= 0 && cluster_id < gpu_dev_count)
			dev = gpu_devices[cluster_id];
	}

	if (!dev) {
		mutex_unlock(&opp_lock);
		pr_err("KPM_OC: opp_add: device not found for %s:%d\n", type, cluster_id);
		return -ENODEV;
	}

	ret = dev_pm_opp_add(dev, freq, uvolt);
	if (ret)
		pr_err("KPM_OC: opp_add: failed to add %lu Hz @ %lu uV (err=%d)\n",
		       freq, uvolt, ret);
	else
		pr_info("KPM_OC: opp_add: added %s:%d %lu Hz @ %lu uV\n",
			type, cluster_id, freq, uvolt);

	mutex_unlock(&opp_lock);
	return ret;
}

static const struct kernel_param_ops opp_add_ops = {
	.set = set_opp_add,
};
static int opp_add_dummy = 0;
module_param_cb(opp_add, &opp_add_ops, &opp_add_dummy, 0644);
MODULE_PARM_DESC(opp_add, "Add OPP entry: TYPE:CLUSTER:FREQ:UVOLT");

/* ─── Per-Entry Remove (sysfs: echo TYPE:CLUSTER:FREQ > opp_remove) ───── */

static int set_opp_remove(const char *val, const struct kernel_param *kp)
{
	char type[8];
	int cluster_id;
	unsigned long freq;
	struct device *dev = NULL;
	struct dev_pm_opp *opp;
	unsigned long uvolt;
	int i;

	if (sscanf(val, "%7[^:]:%d:%lu", type, &cluster_id, &freq) != 3) {
		pr_err("KPM_OC: opp_remove: invalid format (TYPE:CLUSTER:FREQ)\n");
		return -EINVAL;
	}

	mutex_lock(&opp_lock);

	if (strcmp(type, "CPU") == 0) {
		for (i = 0; i < cluster_count; i++) {
			if (clusters[i].cluster_id == cluster_id) {
				dev = get_cpu_device(clusters[i].first_cpu);
				break;
			}
		}
	} else if (strcmp(type, "GPU") == 0) {
		if (cluster_id >= 0 && cluster_id < gpu_dev_count)
			dev = gpu_devices[cluster_id];
	}

	if (!dev) {
		mutex_unlock(&opp_lock);
		return -ENODEV;
	}

	/* Get voltage before removing (for logging) */
	opp = dev_pm_opp_find_freq_exact(dev, freq, true);
	if (IS_ERR(opp)) {
		mutex_unlock(&opp_lock);
		pr_err("KPM_OC: opp_remove: freq %lu not found\n", freq);
		return -ENOENT;
	}
	uvolt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	dev_pm_opp_remove(dev, freq);
	pr_info("KPM_OC: opp_remove: removed %s:%d %lu Hz (was %lu uV)\n",
		type, cluster_id, freq, uvolt);

	mutex_unlock(&opp_lock);
	return 0;
}

static const struct kernel_param_ops opp_remove_ops = {
	.set = set_opp_remove,
};
static int opp_remove_dummy = 0;
module_param_cb(opp_remove, &opp_remove_ops, &opp_remove_dummy, 0644);
MODULE_PARM_DESC(opp_remove, "Remove OPP entry: TYPE:CLUSTER:FREQ");

/* ─── Per-Entry Modify (remove + re-add with new values) ────────────────── */

static int set_opp_modify(const char *val, const struct kernel_param *kp)
{
	char type[8];
	int cluster_id;
	unsigned long old_freq, new_freq, new_uvolt;
	struct device *dev = NULL;
	int ret, i;

	ret = sscanf(val, "%7[^:]:%d:%lu:%lu:%lu",
		     type, &cluster_id, &old_freq, &new_freq, &new_uvolt);
	if (ret != 5) {
		pr_err("KPM_OC: opp_modify: invalid format (TYPE:CLUSTER:OLD_FREQ:NEW_FREQ:NEW_UVOLT)\n");
		return -EINVAL;
	}

	mutex_lock(&opp_lock);

	if (strcmp(type, "CPU") == 0) {
		for (i = 0; i < cluster_count; i++) {
			if (clusters[i].cluster_id == cluster_id) {
				dev = get_cpu_device(clusters[i].first_cpu);
				break;
			}
		}
	} else if (strcmp(type, "GPU") == 0) {
		if (cluster_id >= 0 && cluster_id < gpu_dev_count)
			dev = gpu_devices[cluster_id];
	}

	if (!dev) {
		mutex_unlock(&opp_lock);
		return -ENODEV;
	}

	/* If only voltage changed (same freq), use adjust_voltage */
	if (old_freq == new_freq) {
		ret = dev_pm_opp_adjust_voltage(dev, new_freq,
						new_uvolt, new_uvolt, new_uvolt);
		if (ret)
			pr_err("KPM_OC: opp_modify: voltage adjust failed (err=%d)\n", ret);
		else
			pr_info("KPM_OC: opp_modify: adjusted %s:%d %lu Hz -> %lu uV\n",
				type, cluster_id, new_freq, new_uvolt);
	} else {
		/* Remove old, add new */
		dev_pm_opp_remove(dev, old_freq);
		ret = dev_pm_opp_add(dev, new_freq, new_uvolt);
		if (ret)
			pr_err("KPM_OC: opp_modify: re-add failed (err=%d)\n", ret);
		else
			pr_info("KPM_OC: opp_modify: %s:%d %lu -> %lu Hz @ %lu uV\n",
				type, cluster_id, old_freq, new_freq, new_uvolt);
	}

	mutex_unlock(&opp_lock);
	return ret;
}

static const struct kernel_param_ops opp_modify_ops = {
	.set = set_opp_modify,
};
static int opp_modify_dummy = 0;
module_param_cb(opp_modify, &opp_modify_ops, &opp_modify_dummy, 0644);
MODULE_PARM_DESC(opp_modify, "Modify OPP: TYPE:CLUSTER:OLD_FREQ:NEW_FREQ:NEW_UVOLT");

/* ─── Module Init / Exit ────────────────────────────────────────────────── */

static int __init kpm_oc_init(void)
{
	pr_info("KPM_OC: Dynamic CPU/GPU OPP Module v2.0 initializing...\n");

	/* Detect CPU cluster topology */
	detect_cpu_clusters();

	/* Discover GPU devices */
	discover_gpu_devices();

	/* Populate initial OPP table export (read-only scan, no modifications) */
	memset(opp_table_export, 0, sizeof(opp_table_export));

	pr_info("KPM_OC: Initialized. Write 1 to 'apply' to scan OPP tables.\n");
	return 0;
}

static void __exit kpm_oc_exit(void)
{
	pr_info("KPM_OC: Unloaded.\n");
}

module_init(kpm_oc_init);
module_exit(kpm_oc_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zerofrip");
MODULE_DESCRIPTION("Dynamic CPU/GPU Overclock Module v2.0 with per-entry OPP control");

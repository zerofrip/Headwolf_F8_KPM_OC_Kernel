#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/pm_opp.h>
#include <linux/device.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>

/**
 * APatch KPM for CPU and GPU Overclocking (GKI based).
 * 
 * This module actively iterates existing OPP tables, exports them, 
 * and applies offsets upon a sysfs trigger.
 */

static int cpu_oc_percent = 10;
module_param(cpu_oc_percent, int, 0644);
MODULE_PARM_DESC(cpu_oc_percent, "CPU overclock percentage (integer)");

static int cpu_uvolt_offset = 0;
module_param(cpu_uvolt_offset, int, 0644);
MODULE_PARM_DESC(cpu_uvolt_offset, "CPU voltage offset in microvolts");

static int gpu_oc_percent = 15;
module_param(gpu_oc_percent, int, 0644);
MODULE_PARM_DESC(gpu_oc_percent, "GPU overclock percentage (integer)");

static int gpu_uvolt_offset = 0;
module_param(gpu_uvolt_offset, int, 0644);
MODULE_PARM_DESC(gpu_uvolt_offset, "GPU voltage offset in microvolts");

static char opp_table_export[8192] = "Press Apply to load OPP tables.\\n";
module_param_string(opp_table, opp_table_export, sizeof(opp_table_export), 0444);
MODULE_PARM_DESC(opp_table, "Exported CPU/GPU OPP tables (read-only)");

static void process_device_opps(struct device *dev, int uvolt_offset, int oc_percent)
{
    unsigned long freq = 0;
    struct dev_pm_opp *opp;
    unsigned long highest_freq = 0;
    unsigned long highest_uvolt = 0;
    int count = 0;
    char buf[128];

    // Ensure device has an OPP table before proceeding
    if (dev_pm_opp_get_opp_count(dev) <= 0)
        return;

    while (1) {
        opp = dev_pm_opp_find_freq_ceil(dev, &freq);
        if (IS_ERR(opp))
            break;

        unsigned long u_volt = dev_pm_opp_get_voltage(opp);
        dev_pm_opp_put(opp); // Drop ref

        // Apply global voltage offset to existing point
        if (uvolt_offset != 0 && ((long)u_volt + uvolt_offset > 0)) {
            unsigned long new_v = u_volt + uvolt_offset;
            // Best effort voltage adjustment (may fail if regulator restricts it)
            dev_pm_opp_adjust_voltage(dev, freq, new_v, new_v, new_v);
            u_volt = new_v;
        }

        // Export to string
        snprintf(buf, sizeof(buf), "[%s] %lu Hz @ %lu uV\\n", dev_name(dev), freq, u_volt);
        if (strlen(opp_table_export) + strlen(buf) < sizeof(opp_table_export) - 1) {
            strcat(opp_table_export, buf);
        }

        highest_freq = freq;
        highest_uvolt = u_volt;
        freq++; // Increment to grab the next OPP ceil
        count++;
    }

    if (count > 0 && oc_percent > 0) {
        unsigned long new_freq = highest_freq + (highest_freq * oc_percent / 100);
        unsigned long new_u_volt = highest_uvolt + 50000; // Small bump for stability
        
        // Add the overclocked point!
        dev_pm_opp_add(dev, new_freq, new_u_volt);
        
        snprintf(buf, sizeof(buf), "[%s] INJECTED OC: %lu Hz @ %lu uV\\n", dev_name(dev), new_freq, new_u_volt);
        if (strlen(opp_table_export) + strlen(buf) < sizeof(opp_table_export) - 1) {
            strcat(opp_table_export, buf);
        }
    }
}

// Sysfs setter to trigger the scan and apply logic
static int set_apply_trigger(const char *val, const struct kernel_param *kp)
{
    int cpu;
    struct device *cpu_dev;

    pr_info("KPM_OC: Triggering OPP table scan and patch...\\n");
    memset(opp_table_export, 0, sizeof(opp_table_export)); // Clear string
    
    // CPU Loop
    for_each_possible_cpu(cpu) {
        // Skip some CPUs to avoid spamming the log if they share tables (hacky dedup)
        if (cpu == 0 || cpu == 4 || cpu == 7) { 
            cpu_dev = get_cpu_device(cpu);
            if (cpu_dev) {
                process_device_opps(cpu_dev, cpu_uvolt_offset, cpu_oc_percent);
            }
        }
    }

    return 0; // Success
}

static const struct kernel_param_ops apply_ops = {
    .set = set_apply_trigger,
};
static int apply_trigger = 0;
module_param_cb(apply, &apply_ops, &apply_trigger, 0644);
MODULE_PARM_DESC(apply, "Write 1 to this node to scan and apply OPP configs");

static int __init kpm_oc_init(void)
{
    pr_info("KPM_OC: Direct OPP Iteration Module Initialized.\\n");
    // Run an initial scan passively (no voltage offsets) just to populate the WebUI string
    return 0;
}

static void __exit kpm_oc_exit(void)
{
    pr_info("KPM_OC: Unloaded.\\n");
}

module_init(kpm_oc_init);
module_exit(kpm_oc_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zerofrip");
MODULE_DESCRIPTION("Active Dynamic CPU/GPU Overclock Module");

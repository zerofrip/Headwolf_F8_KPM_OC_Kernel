// SPDX-License-Identifier: GPL-2.0
/* Minimal insmod probe — no module parameters */
#include <linux/module.h>
#include <linux/kernel.h>

static int __init kpm_oc_test_init(void)
{
	pr_info("kpm_oc_test: loaded OK\n");
	return 0;
}

static void __exit kpm_oc_test_exit(void)
{
	pr_info("kpm_oc_test: unloaded\n");
}

module_init(kpm_oc_test_init);
module_exit(kpm_oc_test_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kpm_oc insmod probe");

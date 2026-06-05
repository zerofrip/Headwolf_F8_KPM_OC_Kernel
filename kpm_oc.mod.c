#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

MODULE_INFO(scmversion, "g22ece82feb61-dirty");

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif


static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xaf56600a, "arm64_use_ng_mappings" },
	{ 0x40863ba1, "ioremap_prot" },
	{ 0x92997ed8, "_printk" },
	{ 0x253af43d, "delayed_work_timer_fn" },
	{ 0x79cf5987, "init_timer_key" },
	{ 0x9cc4f70a, "register_pm_notifier" },
	{ 0x472cf3b, "register_kprobe" },
	{ 0x79345cb9, "register_kretprobe" },
	{ 0xb0f4ab9f, "kthread_create_on_node" },
	{ 0x73e2bcf1, "wake_up_process" },
	{ 0x7681946c, "unregister_pm_notifier" },
	{ 0xeb78b1ed, "unregister_kprobe" },
	{ 0xce598ef2, "unregister_kretprobe" },
	{ 0xf8beca97, "cancel_delayed_work_sync" },
	{ 0x4fa9ae5c, "kthread_stop" },
	{ 0xedc03953, "iounmap" },
	{ 0xd5977bfb, "mutex_lock" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x4829a47e, "memcpy" },
	{ 0x96848186, "scnprintf" },
	{ 0xed55cabd, "mutex_unlock" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xc2c193d2, "__stack_chk_fail" },
	{ 0xdcb764ad, "memset" },
	{ 0xa916b694, "strnlen" },
	{ 0x6b722180, "log_read_mmio" },
	{ 0xc1a5ee3, "log_post_read_mmio" },
	{ 0x5c3c7387, "kstrtoull" },
	{ 0xdd64e639, "strscpy" },
	{ 0x349cba85, "strchr" },
	{ 0xc905402b, "log_write_mmio" },
	{ 0xfa4d3c24, "log_post_write_mmio" },
	{ 0x4531ab62, "copy_from_kernel_nofault" },
	{ 0x3854774b, "kstrtoll" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xbf57e89e, "queue_delayed_work_on" },
	{ 0x73e08ef8, "get_mcupm_ipidev" },
	{ 0x705006af, "mtk_ipi_send_compl" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x819bebd4, "mutex_is_locked" },
	{ 0x85df9b6c, "strsep" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0x20000329, "simple_strtoul" },
	{ 0x9e61bb05, "set_freezable" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0xd73653c4, "freezer_active" },
	{ 0xf9a482f9, "msleep" },
	{ 0x817dcb59, "freezing_slow_path" },
	{ 0x4482cdb, "__refrigerator" },
	{ 0x6091797f, "synchronize_rcu" },
	{ 0x1348649e, "alt_cb_patch_nops" },
	{ 0x1e6d26a8, "strstr" },
	{ 0x1d24c881, "___ratelimit" },
	{ 0x7d68eebb, "param_ops_uint" },
	{ 0x91a089b, "param_ops_bool" },
	{ 0xb1777b47, "param_ops_string" },
	{ 0xea759d7f, "module_layout" },
};

MODULE_INFO(depends, "");


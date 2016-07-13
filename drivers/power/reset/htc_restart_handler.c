/* linux/arch/arm/mach-msm/htc_restart_handler.c
 *
 * Copyright (C) 2012 HTC Corporation.
 * Author: Jimmy.CM Chen <jimmy.cm_chen@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/kdebug.h>
#include <linux/notifier.h>
#include <linux/kallsyms.h>

#include <soc/qcom/restart.h>

#include <soc/qcom/htc_restart_handler.h>

#define MAX_SZ_DIAG_ERR_MSG 	200

struct htc_reboot_params {
	unsigned abnrst;
	unsigned xbl_log_addr;
	unsigned ddr_vendor;
	unsigned lock_state;
	char msg;
};

static struct htc_reboot_params *reboot_params;
static atomic_t restart_counter = ATOMIC_INIT(0);
static void *restart_reason;
static unsigned rst_msg_size;

/*
   This function should not be called outside
   to ensure that others do not change restart reason.
   Use mode & cmd to set reason & msg in arch_reset().
*/
static inline void set_restart_msg(const char *msg)
{
	if (msg) {
		pr_notice("set restart msg = '%s'\n", msg);
		strncpy(&reboot_params->msg, msg, rst_msg_size - 1);
	} else {
		strncpy(&reboot_params->msg, "", rst_msg_size - 1);
	}
	mb();
}

unsigned get_restart_reason(void)
{
	return __raw_readl(restart_reason);
}
EXPORT_SYMBOL(get_restart_reason);

/*
   This function should not be called outside
   to ensure that others do not change restart reason.
   Use mode & cmd to set reason & msg in arch_reset().
*/
static inline void set_restart_reason(unsigned int reason)
{
	pr_notice("set restart reason = 0x%08X\n", reason);
	__raw_writel(reason, restart_reason);
	mb();
}

static struct die_args *tombstone = NULL;

int die_notify(struct notifier_block *self,
				       unsigned long val, void *data)
{
	static struct die_args args;
	memcpy(&args, data, sizeof(args));
	tombstone = &args;
	pr_debug("saving oops: %p\n", (void*) tombstone);
	return NOTIFY_DONE;
}

static struct notifier_block die_nb = {
	.notifier_call = die_notify,
};

static int panic_restart_action(struct notifier_block *this, unsigned long event, void *ptr)
{
	char kernel_panic_msg[MAX_SZ_DIAG_ERR_MSG] = "Kernel Panic";

	if (tombstone) { /* tamper the panic message for Oops */
		char pc_symn[KSYM_NAME_LEN] = "<unknown>";
		char lr_symn[KSYM_NAME_LEN] = "<unknown>";

#if defined(CONFIG_ARM)
		sprint_symbol(pc_symn, tombstone->regs->ARM_pc);
		sprint_symbol(lr_symn, tombstone->regs->ARM_lr);
#elif defined(CONFIG_ARM64)
		sprint_symbol(pc_symn, tombstone->regs->pc);
		sprint_symbol(lr_symn, tombstone->regs->regs[30]);
#endif

		snprintf(kernel_panic_msg, rst_msg_size - 1,
				"KP: %s PC:%s LR:%s",
				current->comm, pc_symn, lr_symn);
	} else
		snprintf(kernel_panic_msg, rst_msg_size - 1, "KP: %s", (char *)ptr);
	set_restart_to_ramdump(kernel_panic_msg);

	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call  = panic_restart_action,
};

int set_restart_action(unsigned int reason, const char *msg)
{
	/* only allow write msg before entering arch_rest */
	if (atomic_read(&restart_counter) != 0) {
		pr_warn("%s: someone call this function before\r\n", __func__);
		return 1;
	}

	atomic_set(&restart_counter, 1);

	set_restart_reason(reason);
	set_restart_msg(msg ? msg: "");
	return 0;
}
EXPORT_SYMBOL(set_restart_action);

int set_restart_to_oem(unsigned int code, const char *msg)
{
	char oem_msg[MAX_SZ_DIAG_ERR_MSG] = "";

	if (msg == NULL)
		snprintf(oem_msg, rst_msg_size - 1, "oem-%x", code);
	else
		strncpy(oem_msg, msg, (strlen(msg) >= rst_msg_size) ? (rst_msg_size - 1): strlen(msg));

	/* oem-93, 94, 95, 96, 97, 98, 99 are RIL fatal */
	if ((code >= 0x93) && (code <= 0x98))
		code = 0x99;
#ifdef CONFIG_HTC_HANG_DETECT_RESTART
	/* oem-aa, ab are hanging detector */
	else if ((code == 0xaa) || (code == 0xab))
		code = 0x99;
#endif

	return set_restart_action(RESTART_REASON_OEM_BASE | code, oem_msg);
}
int set_restart_to_ramdump(const char *msg)
{
	return set_restart_action(RESTART_REASON_RAMDUMP, msg);
}
EXPORT_SYMBOL(set_restart_to_ramdump);

int htc_restart_handler_init(void)
{
	struct device_node *np;
	unsigned htc_rst_info_size;
	int ret = 0;

	np = of_find_compatible_node(NULL, NULL,
				"qcom,msm-imem-restart_reason");
	if (!np) {
		pr_err("unable to find DT imem restart reason node\n");
		ret = -ENOENT;
	} else {
		restart_reason = of_iomap(np, 0);
		if (!restart_reason) {
			pr_err("unable to map imem restart reason offset\n");
			ret = -ENOMEM;
		}
	}
	if (ret)
		goto err_restart_reason;

	np = of_find_compatible_node(NULL, NULL,
				"htc,msm-imem-restart_info");
	if (!np) {
		pr_err("unable to find DT imem htc restart info node\n");
		ret = -ENOENT;
	} else {
		reboot_params = of_iomap(np, 0);
		if (!reboot_params) {
			pr_err("unable to map imem htc restart info offset\n");
			ret = -ENOMEM;
		} else {
			ret = of_property_read_u32(np, "info_size", &htc_rst_info_size);
			if (ret) {
				pr_err("%s: Failed to find info_size property in htc restart info device node %d\n"
					, __func__, ret);
				goto err_info_size;
			}
		}
	}
	if (ret)
		goto err_restart_msg;

	rst_msg_size = htc_rst_info_size - offsetof(struct htc_reboot_params, msg);
	if (rst_msg_size > MAX_SZ_DIAG_ERR_MSG)
		rst_msg_size = MAX_SZ_DIAG_ERR_MSG;

	set_restart_reason(RESTART_REASON_RAMDUMP);
	set_restart_msg("Unknown");

	register_die_notifier(&die_nb);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);

	return ret;

err_info_size:
	iounmap(reboot_params);
err_restart_msg:
	iounmap(restart_reason);
err_restart_reason:
	return ret;
}
EXPORT_SYMBOL(htc_restart_handler_init);

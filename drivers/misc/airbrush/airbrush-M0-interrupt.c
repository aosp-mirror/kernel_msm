/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Thiagu Ramalingam(thiagu.r@samsung.com)
 *
 * Airbrush PMU driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/mfd/abc-pcie.h>
#include "airbrush-sm-ctrl.h"

/* ABC AON config regisetr offsets */
#define SYSREG_AON				0x30000

int ab_interrupt_M0(int tar_dev)
{
	printk("Interrupting M0\n");
	aon_config_write(SYSREG_AON + 0x037C, 0x4, 1);
	aon_config_write(SYSREG_AON + 0x0380, 0x4, 1);
	return 0;
}
EXPORT_SYMBOL(ab_interrupt_M0);

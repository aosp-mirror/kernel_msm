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
#include <linux/mfd/syscon/exynos5-pmu.h>
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
#include <linux/airbrush-sm-ctrl.h>

/* IPU and TPU Status Timeout */
#define IPU_TPU_STATUS_TIMEOUT			20

/* ABC Power state error codes */
#define ERROR_POWER_STATE_CHANGE		0xFFFFFFFF
#define ERROR_STATUS_TIMEOUT			0xFFFFFFFE
#define ERROR_IPU_BLOCK_OFF			0xFFFFFFFD
#define ERROR_IPU_BLOCK_ON			0xFFFFFFFC
#define ERROR_TPU_BLOCK_OFF			0xFFFFFFFB
#define ERROR_TPU_BLOCK_ON			0xFFFFFFFA
#define ERROR_DRAM_SET_STATE			0xFFFFFFF9
#define ERROR_PCIE_SET_STATE			0xFFFFFFF8
#define ERROR_IPU_SET_STATE			0xFFFFFFF7
#define ERROR_TPU_SET_STATE			0xFFFFFFF6

/* ABC PMU config regisetr offsets */
#define PMU_CONTROL_OFFSET			0xa0004
#define PMU_STATUS_OFFSET			0xa0000
#define PMU_TIMEOUT_PWRSWITCH_HANDSHAKE_IPU	0xa3530
#define PMU_TIMEOUT_PWRSWITCH_HANDSHAKE_TPU	0xa3534
#define SYS_AON_REG_PCIE_INIT_OFFSET		0x30388

/* airbrush soc block for powerstates */
typedef enum ab_blocks {
	AB_BLK_IPU,
	AB_BLK_TPU,
	AB_BLK_DRAM,
	AB_BLK_PCIE,
} ab_blocks_t;

/* airbrush soc block status */
typedef enum ab_blk_status {
	BLOCK_ON,
	BLOCK_OFF
} ab_blk_status_t;

static ab_blk_status_t ab_block_status(ab_blocks_t block)
{
	uint32_t val;
#ifdef DEBUG
	pr_debug("%sn", __func__);
#endif
	switch (block) {
	case AB_BLK_IPU:
		aon_config_read(PMU_STATUS_OFFSET, 0x0, &val);
		if (!(val & 0x1)) /*PMU_STATUS[0:0]*/
			return BLOCK_OFF;
		break;
	case AB_BLK_TPU:
		aon_config_read(PMU_STATUS_OFFSET, 0x0, &val);
		if (!(val & 0x2)) /*PMU_STATUS[1:1]*/
			return BLOCK_OFF;
		break;
	case AB_BLK_DRAM:
		break;
	case AB_BLK_PCIE:
		break;
	}

	return BLOCK_ON;
}

static uint32_t poll_status(uint32_t device, uint32_t offset)
{
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;
	uint32_t val = 0;
#ifdef DEBUG
	pr_debug("%sn", __func__);
#endif
	do {
		switch (device) {
		case AB_BLK_IPU:
			ipu_config_read(offset, 0x0, &val);
			break;
		case AB_BLK_TPU:
			tpu_config_read(offset, 0x0, &val);
			break;
		default:
			pr_err("%sn", __func__);
		}

	} while (val && --timeout > 0);

	if (timeout == 0) {
		pr_debug("Timeout waiting for status\n");
		return ERROR_STATUS_TIMEOUT;
	}

	return 0;
}

static int ab_tpu_gating_off(void)
{
	int ret;
	uint32_t val;

#ifdef DEBUG
	pr_debug("%s\n", __func__);
#endif
	/* check for TPU core status */
	if (ab_block_status(AB_BLK_TPU) == BLOCK_ON)
		return ERROR_TPU_BLOCK_ON;
	/* tpu_aon.logicShutdownPreReg */
	tpu_config_read(0x20010, 0, &val);
	if (val != 0x1ffff)
		return ERROR_POWER_STATE_CHANGE;

	/* tpu_aon.logicShutdownPreReg */
	tpu_config_write(0x20010, 0x0, 0x0);
	/* tpu_aon.logicShutdownPreReg */
	tpu_config_write(0x20014, 0x0, 0x0);
	/* tpu_aon.logicShutdownPreReg */
	if (poll_status(AB_BLK_TPU, 0x20010))
		return ERROR_STATUS_TIMEOUT;

	if (poll_status(AB_BLK_TPU, 0x20014))
		return ERROR_STATUS_TIMEOUT;

	/* tpu_aon.logicShutdownAllReg */
	tpu_config_read(0x20018, 0x0, &val);
	if (val != 0x1ffff)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.logicShutdownAllReg */
	tpu_config_write(0x20018, 0x0, 0x0);
	/* tpu_aon.logicShutdownAllReg */
	tpu_config_write(0x2001c, 0x0, 0x0);
	/* tpu_aon.logicShutdownAllReg */
	if (poll_status(AB_BLK_TPU, 0x20018))
		return ERROR_STATUS_TIMEOUT;

	/* tpu_aon.logicShutdownAllReg */
	ret = poll_status(AB_BLK_TPU, 0x2001c);
	if (ret)
		return ERROR_STATUS_TIMEOUT;
	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x20008, 0x0, 0x3);
	/* tpu_aon.clockEnableReg */
	tpu_config_read(0x20008, 0x0, &val);
	if (val != 0x3)
		return ERROR_POWER_STATE_CHANGE;

	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x20008, 0x0, 0x2);
	/* tpu_aon.clockEnableReg */
	tpu_config_read(0x20008, 0x0, &val);
	if (val != 0x2)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x2000c, 0x0, 0x0);
	/* tpu_aon.clockEnableReg */
	tpu_config_read(0x2000c, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x20028, 0x0, 0x0);
	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x2002c, 0x0, 0x0);
	/* tpu_aon.clockEnableReg */
	if (poll_status(AB_BLK_TPU, 0x20028))
		return ERROR_STATUS_TIMEOUT;

	/* tpu_aon.memoryShutdownAckReg */
	if (poll_status(AB_BLK_TPU, 0x20030))
		return ERROR_STATUS_TIMEOUT;

	/* tpu_aon.tileMemoryRetnReg */
	tpu_config_write(0x20020, 0x0, 0x0);
	/* tpu_aon.tileMemoryRetnReg */
	tpu_config_write(0x20024, 0x0, 0x0);
	/* tpu_aon.tileMemoryRetnReg */
	tpu_config_read(0x20020, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.tileMemoryRetnReg */
	tpu_config_read(0x20024, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x20008, 0x0, 0x1);
	/* tpu_aon.clockEnableReg */
	tpu_config_read(0x20008, 0x0, &val);
	if (val != 0x1)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.clampEnableReg */
	tpu_config_write(0x20038, 0x0, 0x0);
	/* tpu_aon.clampEnableReg */
	tpu_config_write(0x2003c, 0x0, 0x0);
	/* tpu_aon.clampEnableReg */
	tpu_config_read(0x20038, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.clampEnableReg */
	tpu_config_read(0x2003c, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.resetReg */
	tpu_config_write(0x20000, 0x0, 0x0);
	/* tpu_aon.resetReg */
	tpu_config_write(0x20004, 0x0, 0x0);
	/* tpu_aon.forceQuiesceReg */
	tpu_config_read(0x20040, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.forceQuiesceReg */
	tpu_config_read(0x20044, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;

	return 0;
}

#if 0 //unused
static int ab_tpu_gating_on(void)
{
	uint32_t val;
#ifdef DEBUG
	pr_debug("%s\n", __func__);
#endif
	/* check for TPU core status */
	if (ab_block_status(AB_BLK_TPU) == BLOCK_OFF)
		return ERROR_TPU_BLOCK_OFF;
	/* tpu_aon.clampEnableReg */
	tpu_config_write(0x20038, 0x0, 0x1ffff);
	/* tpu_aon.clampEnableReg */
	tpu_config_write(0x2003c, 0x0, 0x0);
	/* tpu_aon.tileMemoryRetnReg */
	tpu_config_write(0x20020, 0x0, 0xffff);
	/* tpu_aon.tileMemoryRetnReg */
	tpu_config_write(0x20024, 0x0, 0x0);
	/* tpu_aon.tileMemoryRetnReg */
	tpu_config_read(0x20020, 0x0, &val);

	if (val != 0xffff)
		return ERROR_POWER_STATE_CHANGE;
	tpu_config_read(0x20024, 0x0, &val);

	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.memoryShutdownReg */
	tpu_config_write(0x20028, 0x0, 0x1ffff);
	/* tpu_aon.memoryShutdownReg */
	tpu_config_write(0x2002c, 0x0, 0x0);
	/* tpu_aon.memoryShutdownReg */
	tpu_config_read(0x20028, 0x0, &val);
	if (val != 0x1ffff)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.memoryShutdownReg */
	tpu_config_read(0x2002c, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.memoryShutdownAckReg */
	tpu_config_read(0x20030, 0x0, &val);
	if (val != 0x1ffff)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.logicShutdownAllReg */
	tpu_config_write(0x20018, 0x0, 0x1ffff);
	/* tpu_aon.logicShutdownAllReg */
	tpu_config_write(0x2001c, 0x0, 0x0);
	/* tpu_aon.logicShutdownAllReg */
	tpu_config_read(0x20018, 0x0, &val);
	if (val != 0x1ffff)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.logicShutdownAllReg */
	tpu_config_read(0x2001c, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.logicShutdownPreReg */
	tpu_config_write(0x20010, 0x0, 0x1ffff);
	/* tpu_aon.logicShutdownPreReg */
	tpu_config_write(0x20014, 0x0, 0x0);
	/* tpu_aon.logicShutdownPreReg */
	tpu_config_read(0x20010, 0x0, &val);
	if (val != 0x1ffff)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.logicShutdownPreReg */
	tpu_config_read(0x20014, 0x0, &val);
	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;

	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x20008, 0x0, 0x2);
	/* tpu_aon.clockEnableReg */
	tpu_config_write(0x2000c, 0x0, 0x0);
	tpu_config_read(0x20008, 0x0, &val);

	if (val != 0x2)
		return ERROR_POWER_STATE_CHANGE;
	/* tpu_aon.clockEnableReg */
	tpu_config_read(0x2000c, 0x0, &val);

	if (val != 0x0)
		return ERROR_POWER_STATE_CHANGE;

	return 0;
}
#endif

static int ab_ipu_gating_off(void)
{
#ifdef DEBUG
	pr_debug("%sn", __func__);
#endif
	/* Check for TPU core status */
	if (ab_block_status(AB_BLK_IPU) == BLOCK_ON)
		return ERROR_IPU_BLOCK_ON;
	/* SOFT_RESET */
	ipu_config_write(0x80, 0x0, 0x1);
	/* CLK_GATE_CONTROL_STP_IDLE_GATE_DIS */
	ipu_config_write(0x18, 0x0, 0x3fff);
	/* CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS */
	ipu_config_write(0x20, 0x0, 0xffff);
	/* CLK_GATE_CONTROL */
	ipu_config_write(0x28, 0x0, 0xf);
	/* Powering i/o Blocks */
	/* IO_POWER_ON_N[32:32] */
	ipu_config_write(0x6c, 0x0, 0x0);
	/* IO_POWER_ON_N[0:0] */
	ipu_config_write(0x68, 0x0, 0x0);
	/* IPU_IO_SWITCHED_CLK_EN[0:0] */
	ipu_config_write(0x38, 0x0, 0x1);
	/* IPU_IO_SWITCHED_CLK_EN[0:0] */
	ipu_config_write(0x38, 0x0, 0x0);
	/* IO_RAM_ON_N[0:0] */
	ipu_config_write(0x78, 0x0, 0x0);
	/* IPU_IO_SWITCHED_CLK_EN[0:0] */
	ipu_config_write(0x38, 0x0, 0x1);
	/* IO_ISO_ON[0:0] */
	ipu_config_write(0x70, 0x0, 0x0);
	/* SOFT_RESET[0:0] */
	ipu_config_write(0x80, 0x0, 0x0);
	/* JQS_CONTROL[0:0] */
	ipu_config_write(0x90, 0x0, 0x1);
	/* powering on core pair 0 */
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3ffe);
	/* CORE_POWER_ON_N[0:13] */
	ipu_config_write(0x50, 0x0, 0x3ffe);
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3ffc);
	/* CORE_POWER_ON_N[0:13] */
	ipu_config_write(0x50, 0x0, 0x3ffc);
	/* IPU_CORE_PAIRS_EN[10:8] */
	ipu_config_write(0x40, 0x0, 0x1);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x60, 0x0, 0x3ffc);
	/* IPU_CORE_PAIRS_EN[10:8] */
	ipu_config_write(0x40, 0x0, 0x1);
	/* CORE_ISO_ON[0:13] */
	ipu_config_write(0x58, 0x0, 0x3ffc);
	/* powering on core pair 1 */
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3ff8);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3ff8);
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3ff0);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3ff0);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x2);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x60, 0x0, 0x3ff0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x40, 0x0, 0x2);
	/* CORE_ISO_ON[0:13] */
	ipu_config_write(0x58, 0x0, 0x3ff0);
	/* powering on core pair 2 */
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3fe0);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3fe0);
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3fc0);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3fc0);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x3);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x60, 0x0, 0x3fc0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x40, 0x0, 0x3);
	/* CORE_ISO_ON[0:13] */
	ipu_config_write(0x58, 0x0, 0x3fc0);
	/* powering on core pair 3 */
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3f80);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3f80);
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3f00);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3f00);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x4);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x60, 0x0, 0x3f00);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x40, 0x0, 0x4);
	/* CORE_ISO_ON[0:13] */
	ipu_config_write(0x58, 0x0, 0x3f00);
	/* powering on core pair 4 */
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3e00);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3e00);
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3c00);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3c00);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x5);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x60, 0x0, 0x3c00);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x40, 0x0, 0x5);
	/* CORE_ISO_ON[0:13] */
	ipu_config_write(0x58, 0x0, 0x3c00);
	/* powering on core pair 5 */
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3800);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3800);
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x3000);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3000);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x6);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x60, 0x0, 0x3000);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x40, 0x0, 0x6);
	/* CORE_ISO_ON[0:13] */
	ipu_config_write(0x58, 0x0, 0x3000);
	/* powering on core pair 6 */
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x2000);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x2000);
	/* CORE_POWER_ON_N[32:45] */
	ipu_config_write(0x54, 0x0, 0x0);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x0);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x7);
	/* IPU_CORE_PAIRS_EN[2:0] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x60, 0x0, 0x0);
	/* CORE_RAM_ON_N[0:13] */
	ipu_config_write(0x40, 0x0, 0x7);
	/* CORE_ISO_ON[0:13] */
	ipu_config_write(0x58, 0x0, 0x0);
	/* CLK_GATE_CONTROL[0:0] */
	ipu_config_write(0x28, 0x0, 0x1);
	/* CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS[0:15] */
	ipu_config_write(0x20, 0x0, 0x0);

	return 0;
}

#if 0 //unused
static int ab_ipu_gating_on(void)
{
#ifdef DEBUG
	pr_debug("%sn", __func__);
#endif
	/* check for IPU core status */
	if (ab_block_status(AB_BLK_IPU) == BLOCK_OFF)
		return ERROR_IPU_BLOCK_OFF;
	/* CLK_GATE_CONTROL_STP_IDLE_GATE_DIS[13:0] */
	ipu_config_write(0x18, 0x0, 0x3fff);
	/* CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS[15:0] */
	ipu_config_write(0x20, 0x0, 0xffff);
	/* CLK_GATE_CONTROL[4:0] */
	ipu_config_write(0x28, 0x0, 0xf);
	/* powering off core pair 6 */
	/* CORE_ISO_ON[13:0] */
	ipu_config_write(0x58, 0x0, 0x3000);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[13:0] */
	ipu_config_write(0x60, 0x0, 0x3000);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x7);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x6);
	/* CORE_POWER_ON_N[45:32] */
	ipu_config_write(0x54, 0x0, 0x3000);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3000);
	/* powering off core pair 5 */
	/* CORE_ISO_ON[13:0] */
	ipu_config_write(0x58, 0x0, 0x3c00);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[13:0] */
	ipu_config_write(0x60, 0x0, 0x3c00);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x6);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x5);
	/* CORE_POWER_ON_N[45:32] */
	ipu_config_write(0x54, 0x0, 0x3c00);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3c00);
	/* powering off core pair 4 */
	/* CORE_ISO_ON[13:0] */
	ipu_config_write(0x58, 0x0, 0x3f00);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[13:0] */
	ipu_config_write(0x60, 0x0, 0x3f00);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x5);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x4);
	/* CORE_POWER_ON_N[45:32] */
	ipu_config_write(0x54, 0x0, 0x3f00);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3f00);
	/* powering off core pair 3 */
	/* CORE_ISO_ON[13:0] */
	ipu_config_write(0x58, 0x0, 0x3fc0);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[13:0] */
	ipu_config_write(0x60, 0x0, 0x3fc0);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x4);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x3);
	/* CORE_POWER_ON_N[45:32] */
	ipu_config_write(0x54, 0x0, 0x3fc0);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3fc0);
	/* powering off core pair 2 */
	/* CORE_ISO_ON[13:0] */
	ipu_config_write(0x58, 0x0, 0x3ff0);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[13:0] */
	ipu_config_write(0x60, 0x0, 0x3ff0);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x3);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x2);
	/* CORE_POWER_ON_N[45:32] */
	ipu_config_write(0x54, 0x0, 0x3ff0);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3ff0);
	/* powering off core pair 1 */
	/* CORE_ISO_ON[13:0] */
	ipu_config_write(0x58, 0x0, 0x3ffc);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[13:0] */
	ipu_config_write(0x60, 0x0, 0x3ffc);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x2);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x1);
	/* CORE_POWER_ON_N[45:32] */
	ipu_config_write(0x54, 0x0, 0x3ffc);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3ffc);
	/* powering off core pair 0 */
	/* CORE_ISO_ON[13:0] */
	ipu_config_write(0x58, 0x0, 0x3fff);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_RAM_ON_N[13:0] */
	ipu_config_write(0x60, 0x0, 0x3fff);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x1);
	/* IPU_CORE_PAIRS_EN[0:2] */
	ipu_config_write(0x40, 0x0, 0x0);
	/* CORE_POWER_ON_N[45:32] */
	ipu_config_write(0x54, 0x0, 0x3fff);
	/* CORE_POWER_ON_N[13:0] */
	ipu_config_write(0x50, 0x0, 0x3fff);
	/* powering off i/o block */
	/* JQS_CONTROL[0:0] */
	ipu_config_write(0x90, 0x0, 0x0);
	/* IO_ISO_ON */
	ipu_config_write(0x70, 0x0, 0x1);
	/* IPU_IO_SWITCHED_CLK_EN[0:0] */
	ipu_config_write(0x38, 0x0, 0x0);
	/* IO_RAM_ON_N[0:0] */
	ipu_config_write(0x78, 0x0, 0x1);
	/* IPU_IO_SWITCHED_CLK_EN[0:0] */
	ipu_config_write(0x38, 0x0, 0x0);
	/* IPU_IO_SWITCHED_CLK_EN[0:0] */
	ipu_config_write(0x38, 0x0, 0x0);
	/* IO_POWER_ON_N[32:32] */
	ipu_config_write(0x6c, 0x0, 0x1);
	/* IO_POWER_ON_N[0:0] */
	ipu_config_write(0x68, 0x0, 0x1);
	/* CLK_GATE_CONTROL[0:0] */
	ipu_config_write(0x28, 0x0, 0x1);
	/* CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS */
	ipu_config_write(0x20, 0x0, 0x1);

	return 0;
}
#endif

static int ab_power_on(ab_blocks_t block)
{
	uint32_t val;
	int ret = 0;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;
#ifdef DEBUG
	pr_debug("%s\n", __func__);
#endif
	switch (block) {
	case AB_BLK_IPU:
		/* check for TPU status */
		if (ab_block_status(AB_BLK_IPU) == BLOCK_ON)
			return ERROR_IPU_BLOCK_ON;

		/* TODO: PMU_TIMEOUT_PWRSWITCH_HANDSHAKE_IPU is hack
		 * disbale later.
		 */
		aon_config_write(PMU_TIMEOUT_PWRSWITCH_HANDSHAKE_IPU,
				 0x0, 0x1);
		aon_config_write(SYS_AON_REG_PCIE_INIT_OFFSET,
				 0x0, 0x1);

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		aon_config_read(PMU_CONTROL_OFFSET, 0x0, &val);
		val |= (1 << 0);

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		aon_config_write(PMU_CONTROL_OFFSET, 0x0, val);

		do {
			aon_config_read(PMU_STATUS_OFFSET, 0x0, &val);

		} while (!(val & 0x1) && --timeout > 0);

		if (timeout == 0) {
			pr_debug("Timeout - IPU up status\n");
			return ERROR_STATUS_TIMEOUT;
		}

		/* CMU ENABLE_POWER_MANAGEMENT[29:29] */
		ipu_config_write(0x40800, 0x0, 0x20000000);
		ret = ab_ipu_gating_off();
		break;

	case AB_BLK_TPU:
		/* check for TPU status */
		if (ab_block_status(AB_BLK_TPU) == BLOCK_ON)
			return ERROR_TPU_BLOCK_ON;

		/* TODO: PMU_TIMEOUT_PWRSWITCH_HANDSHAKE_IPU is hack
		 * disbale later.
		 */
		aon_config_write(PMU_TIMEOUT_PWRSWITCH_HANDSHAKE_TPU,
					0x0, 0x1);
		aon_config_write(SYS_AON_REG_PCIE_INIT_OFFSET,
					0x0, 0x1);

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		aon_config_read(PMU_CONTROL_OFFSET, 0x0, &val);
		val |= (1 << 1);

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		aon_config_write(PMU_CONTROL_OFFSET, 0x0, val);

		do {
			aon_config_read(PMU_STATUS_OFFSET, 0x0, &val);

		} while (!(val & 0x2) && --timeout > 0);

		if (timeout == 0) {
			pr_debug("Timeout - TPU up status\n");
			return ERROR_STATUS_TIMEOUT;
		}

		/* CMU ENABLE_POWER_MANAGEMENT[29:29] */
		tpu_config_write(0x40800, 0x0, 0x20000000);
		ret = ab_tpu_gating_off();
		break;

	case AB_BLK_DRAM:
		break;

	case AB_BLK_PCIE:
		break;
	}
	return ret;
}

#if 0 //unused
static int ab_power_off(ab_blocks_t block)
{
	int ret = 0;
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;
#ifdef DEBUG
	pr_debug("%sn", __func__);
#endif
	switch (block) {
	case AB_BLK_IPU:
		/* check for IPU status */
		if (ab_block_status(AB_BLK_IPU) == BLOCK_OFF)
			return ERROR_IPU_BLOCK_OFF;

		ret = ab_ipu_gating_on();
		 /* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		aon_config_read(PMU_CONTROL_OFFSET, 0, &val);
		val &= ~(1 << 0);
		aon_config_write(PMU_CONTROL_OFFSET, 0x0, val);
		do {
			/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
			aon_config_read(PMU_STATUS_OFFSET, 0x0, &val);
		} while ((val & 0x1) && --timeout > 0);

		if (timeout == 0) {
			pr_debug("Timeout waiting for IPU up status\n");
			return ERROR_STATUS_TIMEOUT;
		}

		/* TODO: cut off rail power */
		break;

	case AB_BLK_TPU:
		/* check for TPU status */
		if (ab_block_status(AB_BLK_TPU) == BLOCK_OFF)
			return ERROR_TPU_BLOCK_OFF;

		ret = ab_tpu_gating_on();
		 /* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
		aon_config_read(PMU_CONTROL_OFFSET, 0, &val);
		val &= ~(1 << 1);
		aon_config_write(PMU_CONTROL_OFFSET, 0x0, val);
		do {
			/* PMU_STATUS[1:1] BLK_TPU_UP_STATUS */
			aon_config_read(PMU_STATUS_OFFSET, 0x0, &val);
		} while ((val & 0x2) && --timeout > 0);

		if (timeout == 0) {
			pr_debug("Timeout waiting for IPU up status\n");
			return ERROR_STATUS_TIMEOUT;
		}

		/* TODO: cut off rail power */
		break;

	case AB_BLK_DRAM:
		break;

	case AB_BLK_PCIE:
		break;
	}
	return 0;
}
#endif

void init_pmu(void)
{
	ab_power_on(AB_BLK_IPU);
	ab_power_on(AB_BLK_TPU);
}
EXPORT_SYMBOL(init_pmu);

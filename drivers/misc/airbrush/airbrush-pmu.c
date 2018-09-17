/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Thiagu Ramalingam(thiagu.r@samsung.com)
 *	Raman Kumar Banka(raman.k2@samsung.com)
 *
 * Airbrush Power Management Unit Control driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/mfd/abc-pcie.h>
#include <linux/airbrush-sm-ctrl.h>
#include "airbrush-pmu.h"

/* IPU and TPU Status Timeout */
#define IPU_TPU_STATUS_TIMEOUT			20000

/* ABC PMU config regisetr offsets */
#define SYSREG_PMU_PMU_CONTROL				0x10ba0004
#define SYSREG_PMU_PMU_STATUS				0x10ba0000
#define CMU_IPU_IPU_CONTROLLER_OPTION			0x10240800
#define CMU_TPU_TPU_CONTROLLER_OPTION			0x10040800

bool is_ab_ipu_on(void)
{
	u32 val;

	ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	if (val & 0x1) /*PMU_STATUS[0:0]*/
		return true;
	return false;
}

bool is_ab_tpu_on(void)
{
	u32 val;

	ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	if (val & 0x2) /*PMU_STATUS[1:1]*/
		return true;
	return false;
}

int ab_block_power_off(block_name_t block)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	switch (block) {
	case BLK_IPU:
		/* check for IPU status */
		if (!is_ab_ipu_on())
			return E_IPU_BLOCK_OFF;

		/* IPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
		ABC_READ(CMU_IPU_IPU_CONTROLLER_OPTION, &val);
		val |= (1<<29);
		ABC_WRITE(CMU_IPU_IPU_CONTROLLER_OPTION, val);

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
		val &= ~(1<<0);
		ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
		do {
			/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
			ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
		} while ((val & 0x1) && --timeout > 0);

		if (timeout == 0) {
			pr_err("Timeout waiting for IPU up status\n");
			return E_STATUS_TIMEOUT;
		}

		break;

	case BLK_TPU:
		/* check for TPU status */
		if (!is_ab_tpu_on())
			return E_TPU_BLOCK_OFF;

		/* TPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
		ABC_READ(CMU_TPU_TPU_CONTROLLER_OPTION, &val);
		val |= (1<<29);
		ABC_WRITE(CMU_TPU_TPU_CONTROLLER_OPTION, val);

		/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
		ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
		val &= ~(1<<1);
		ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
		do {
			/* PMU_STATUS[1:1] BLK_TPU_UP_STATUS */
			ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
		} while ((val & 0x2) && --timeout > 0);

		if (timeout == 0) {
			pr_err("Timeout waiting for TPU up status\n");
			return E_STATUS_TIMEOUT;
		}
		break;

	default:
		break;
	}
	return 0;
}

int ab_block_power_on(block_name_t block)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;

	switch (block) {
	case BLK_IPU:
		/* check for IPU status */
		if (is_ab_ipu_on())
			return E_IPU_BLOCK_ON;

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
		val |= (1 << 0);

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

		do {
			ABC_READ(SYSREG_PMU_PMU_STATUS, &val);

		} while (!(val & 0x1) && --timeout > 0);

		if (timeout == 0) {
			pr_err("Timeout - IPU up status\n");
			return E_STATUS_TIMEOUT;
		}

		break;

	case BLK_TPU:
		/* check for TPU status */
		if (is_ab_tpu_on())
			return E_TPU_BLOCK_ON;

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
		val |= (1 << 1);

		/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
		ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

		do {
			ABC_READ(SYSREG_PMU_PMU_STATUS, &val);

		} while (!(val & 0x2) && --timeout > 0);

		if (timeout == 0) {
			pr_err("Timeout - TPU up status\n");
			return E_STATUS_TIMEOUT;
		}
		break;

	default:
		break;
	}
	return 0;
}

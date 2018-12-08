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
#define IPU_TPU_STATUS_TIMEOUT			1000

/* ABC PMU config regisetr offsets */
#define SYSREG_PMU_PMU_CONTROL				0x10ba0004
#define SYSREG_PMU_PMU_STATUS				0x10ba0000
#define CMU_IPU_IPU_CONTROLLER_OPTION			0x10240800
#define CMU_TPU_TPU_CONTROLLER_OPTION			0x10040800

static int ab_pmu_sleep_handler(void *ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab entering sleep\n");

	/* IPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
	ABC_READ(CMU_IPU_IPU_CONTROLLER_OPTION, &val);
	val |= (0x1 << 29);
	ABC_WRITE(CMU_IPU_IPU_CONTROLLER_OPTION, val);

	/* TPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
	ABC_READ(CMU_TPU_TPU_CONTROLLER_OPTION, &val);
	val |= (0x1 << 29);
	ABC_WRITE(CMU_TPU_TPU_CONTROLLER_OPTION, val);

	/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
	/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val &= ~0x3;
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

	do {
		/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
		/* PMU_STATUS[1:1] BLK_IPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((val & 0x3) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for IPU/TPU down status\n");
		return E_STATUS_TIMEOUT;
	}
	return 0;
}

static int ab_pmu_deep_sleep_handler(void *ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab entering deep sleep\n");

	ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	if (val & 0x3) {
		/* IPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
		ABC_READ(CMU_IPU_IPU_CONTROLLER_OPTION, &val);
		val |= (0x1 << 29);
		ABC_WRITE(CMU_IPU_IPU_CONTROLLER_OPTION, val);

		/* TPU_CONTROLLER_OPTION[29:29] ENABLE_POWER_MANAGEMENT */
		ABC_READ(CMU_TPU_TPU_CONTROLLER_OPTION, &val);
		val |= (0x1 << 29);
		ABC_WRITE(CMU_TPU_TPU_CONTROLLER_OPTION, val);
	}

	/* PMU_CONTROL[2:2] DEEP_SLEEP */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val |= (0x1 << 2);
	val &= ~(0x3 << 0);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

	do {
		/* PMU_STATUS[2:2] DEEP_SLEEP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while (((val & 0x7) != 0x4) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for deep_sleep set status\n");
		return E_STATUS_TIMEOUT;
	}

	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val &= ~(0x1 << 2);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);

	return 0;
}

#define CLK_CON_DIV_PLL_AON_CLK 0x10B1180C
#define CLK_CON_DIV_DIV4_PLLCLK_TPU 0x10041800
#define CLK_CON_DIV_DIV4_PLLCLK_IPU 0x10241800

static void abc_ipu_tpu_enable(void)
{
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_IPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_TPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x0);
}

static int ab_pmu_resume_handler(void *ctx)
{
	uint32_t val;
	uint32_t timeout = IPU_TPU_STATUS_TIMEOUT;
	struct ab_pmu_context *pmu_ctx = (struct ab_pmu_context *)ctx;

	dev_dbg(pmu_ctx->dev, "ab resuming\n");

	/* PMU_CONTROL[0:0] BLK_IPU_UP_REQ */
	/* PMU_CONTROL[1:1] BLK_TPU_UP_REQ */
	ABC_READ(SYSREG_PMU_PMU_CONTROL, &val);
	val |= (0x3 << 0);
	ABC_WRITE(SYSREG_PMU_PMU_CONTROL, val);
	do {
		/* PMU_STATUS[0:0] BLK_IPU_UP_STATUS */
		/* PMU_STATUS[1:1] BLK_IPU_UP_STATUS */
		ABC_READ(SYSREG_PMU_PMU_STATUS, &val);
	} while ((!(val & 0x2) || !(val & 0x1)) && --timeout > 0);

	if (timeout == 0) {
		dev_err(pmu_ctx->dev, "Timeout waiting for IPU/TPU up status\n");
		return E_STATUS_TIMEOUT;
	}

	abc_ipu_tpu_enable();

	return 0;
}

static struct ab_sm_pmu_ops pmu_ops = {
	.pmu_sleep = &ab_pmu_sleep_handler,
	.pmu_deep_sleep = &ab_pmu_deep_sleep_handler,
	.pmu_resume = &ab_pmu_resume_handler,
};

static int ab_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ab_pmu_context *pmu_ctx;

	pmu_ctx = devm_kzalloc(dev, sizeof(struct ab_pmu_context), GFP_KERNEL);
	if (!pmu_ctx)
		return -ENOMEM;

	pmu_ctx->dev = dev;

	pmu_ops.ctx = pmu_ctx;
	ab_sm_register_pmu_ops(&pmu_ops);

	ab_pmu_resume_handler(pmu_ctx);

	return 0;
}

static int ab_pmu_remove(struct platform_device *pdev)
{
	ab_sm_unregister_pmu_ops();
	return 0;
}

static const struct of_device_id ab_pmu_of_match[] = {
		{ .compatible = "abc,airbrush-pmu", },
		{ },
};

static struct platform_driver ab_pmu_driver = {
	.probe = ab_pmu_probe,
	.remove = ab_pmu_remove,
	.driver = {
		.name = "ab-pmu",
		.of_match_table = ab_pmu_of_match,
	},
};
module_platform_driver(ab_pmu_driver);

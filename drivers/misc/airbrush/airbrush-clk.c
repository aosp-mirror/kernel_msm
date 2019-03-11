/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * Author: Raman Kumar Banka <raman.k2@samsung.com>
 *
 * Clock controller for airbrush state manager
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/airbrush-sm-notifier.h>
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/mfd/abc-pcie.h>

#include "airbrush-clk.h"

#define GAT_CLK_BLK_IPU_UID_IPU_IPCLKPORT_CLK_IPU	0x1024202c
#define GAT_CLK_BLK_TPU_UID_TPU_IPCLKPORT_CLK_TPU	0x10042034

#define AB_SM_19_2_MHZ		19200000
#define AB_SM_466_MHZ		466000000
#define AB_SM_789_6_MHZ		789600000
#define AB_SM_921_6_MHZ		921600000
#define AON_CLK_RATE_REG		0x10B10100
#define AON_CLK_RATE_19_2_MHZ	0xA0F00500
#define AON_CLK_RATE_921_6_MHZ	0xA0F00510
#define TPU_CLK_RATE_REG		0x10040120
#define TPU_CLK_RATE_19_2_MHZ	0xA1490202
#define TPU_CLK_RATE_789_6_MHZ	0xA1490212

static void __ab_clk_restore_mainclk_freq(struct ab_clk_context *ctx);
static void __ab_clk_reduce_mainclk_freq(struct ab_clk_context *ctx);

static struct ab_sm_clk_ops clk_ops;
static int ab_clk_pcie_link_listener(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct ab_clk_context *clk_ctx = container_of(nb,
			struct ab_clk_context, pcie_link_blocking_nb);

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		mutex_lock(&clk_ctx->pcie_link_lock);
		clk_ctx->pcie_link_ready = true;
		mutex_unlock(&clk_ctx->pcie_link_lock);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_PRE_DISABLE) {
		mutex_lock(&clk_ctx->pcie_link_lock);
		clk_ctx->pcie_link_ready = false;
		mutex_unlock(&clk_ctx->pcie_link_lock);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;  /* Don't care */
}

static int ab_clk_ipu_pll_enable_handler(void *ctx)
{
	int ret = 0;

	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	dev_dbg(clk_ctx->dev, "%s: enable IPU PLL\n", __func__);

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = clk_prepare_enable(clk_ctx->ipu_pll);
		if (ret < 0)
			dev_err(clk_ctx->dev,
				"Unable to prepare_enable ipu clk (err %d)\n",
				ret);

	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

static int ab_clk_ipu_pll_disable_handler(void *ctx)
{
	int ret = 0;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	dev_dbg(clk_ctx->dev, "%s: disable IPU PLL\n", __func__);

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		clk_disable_unprepare(clk_ctx->ipu_pll);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_ipu_set_rate_handler(struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;

	dev_dbg(clk_ctx->dev,
		"%s: set IPU clock rate to %llu\n", __func__, new_rate);

	ab_sm_clk_notify(AB_IPU_PRE_RATE_CHANGE, old_rate, new_rate);

	if (new_rate == AB_SM_OSC_RATE) {
		ret = clk_set_parent(clk_ctx->ipu_pll_mux, clk_ctx->osc_clk);
		if (ret) {
			dev_err(clk_ctx->dev,
				"ipu_pll_mux: set_parent failed(err %d)\n",
				ret);
			goto error_abort;
		}

		ab_sm_clk_notify(AB_IPU_POST_RATE_CHANGE,
				old_rate, new_rate);
		return new_rate;
	}

	/* Switch to osc_clk during rate change so that
	 * change doesn't propagate to children
	 */
	ret = clk_set_parent(clk_ctx->ipu_pll_mux, clk_ctx->osc_clk);
	if (ret) {
		dev_err(clk_ctx->dev,
			"ipu_pll_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	/* force calculate current rate */
	clk_get_rate(clk_ctx->ipu_pll);

	ret = clk_set_rate(clk_ctx->ipu_pll, new_rate);
	if (ret) {
		dev_err(clk_ctx->dev,
			"ipu_pll: set_rate failed(err %d)\n", ret);
		goto error_abort;
	}

	ret = clk_set_parent(clk_ctx->ipu_pll_mux, clk_ctx->ipu_pll);
	if (ret) {
		dev_err(clk_ctx->dev,
			"ipu_pll_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	ab_sm_clk_notify(AB_IPU_POST_RATE_CHANGE, old_rate, new_rate);
	return new_rate;

error_abort:
	ab_sm_clk_notify(AB_IPU_ABORT_RATE_CHANGE, old_rate, new_rate);
	return ret;
}

static int64_t ab_clk_ipu_set_rate_handler(void *ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = __ab_clk_ipu_set_rate_handler(clk_ctx,
				old_rate, new_rate);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

static int ab_clk_tpu_pll_enable_handler(void *ctx)
{
	int ret = 0;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	dev_dbg(clk_ctx->dev, "%s: enable TPU PLL\n", __func__);

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = clk_prepare_enable(clk_ctx->tpu_pll);
		if (ret < 0)
			dev_err(clk_ctx->dev,
				"Unable to prepare_enable tpu clk (err %d)\n",
				ret);

	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

static int ab_clk_tpu_pll_disable_handler(void *ctx)
{
	int ret = 0;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	dev_dbg(clk_ctx->dev, "%s: disable TPU PLL\n", __func__);

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		clk_disable_unprepare(clk_ctx->tpu_pll);

	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_tpu_set_rate_handler(struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;

	dev_dbg(clk_ctx->dev,
		"%s: set TPU clock rate to %llu\n", __func__, new_rate);

	ab_sm_clk_notify(AB_TPU_PRE_RATE_CHANGE, old_rate, new_rate);

	if (new_rate == AB_SM_OSC_RATE) {
		ret = clk_set_parent(clk_ctx->tpu_pll_mux, clk_ctx->osc_clk);
		if (ret) {
			dev_err(clk_ctx->dev,
				"tpu_pll_mux: set_parent failed(err %d)\n",
				ret);
			goto error_abort;
		}

		ab_sm_clk_notify(AB_TPU_POST_RATE_CHANGE,
				old_rate, new_rate);
		return new_rate;
	}

	/* Switch to shared_div_aon_pll to prevent rail droop if transitioning
	 * during a workload (see b/126274013)
	 */
	clk_set_parent(clk_ctx->tpu_switch_mux, clk_ctx->shared_div_aon_pll);
	if (ret) {
		dev_err(clk_ctx->dev,
			"tpu_switch_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	/* Switch to osc_clk during rate change so that
	 * change doesn't propagate to children
	 */
	ret = clk_set_parent(clk_ctx->tpu_pll_mux, clk_ctx->osc_clk);
	if (ret) {
		dev_err(clk_ctx->dev,
			"tpu_pll_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	/* force calculate current rate */
	clk_get_rate(clk_ctx->tpu_pll);

	ret = clk_set_rate(clk_ctx->tpu_pll, new_rate);
	if (ret) {
		dev_err(clk_ctx->dev,
			"tpu_pll: set_rate failed(err %d)\n", ret);
		goto error_abort;
	}

	ret = clk_set_parent(clk_ctx->tpu_pll_mux, clk_ctx->tpu_pll);
	if (ret) {
		dev_err(clk_ctx->dev,
			"tpu_pll_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	clk_set_parent(clk_ctx->tpu_switch_mux, clk_ctx->tpu_pll_div);
	if (ret) {
		dev_err(clk_ctx->dev,
			"tpu_switch_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	ab_sm_clk_notify(AB_TPU_POST_RATE_CHANGE, old_rate, new_rate);
	return new_rate;

error_abort:
	ab_sm_clk_notify(AB_TPU_ABORT_RATE_CHANGE, old_rate, new_rate);
	return ret;
}

static int64_t ab_clk_tpu_set_rate_handler(void *ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = __ab_clk_tpu_set_rate_handler(clk_ctx,
				old_rate, new_rate);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

/*
 * NOTE: This method does not result in clk notifications for
 * mfd child drivers
 */
static int64_t ab_clk_tpu_set_rate_direct_handler(void *ctx,
		u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		switch (new_rate) {
		case AB_SM_19_2_MHZ:
			ABC_WRITE(TPU_CLK_RATE_REG, TPU_CLK_RATE_19_2_MHZ);
			ret = AB_SM_19_2_MHZ;
			break;
		case AB_SM_789_6_MHZ:
			ABC_WRITE(TPU_CLK_RATE_REG, TPU_CLK_RATE_789_6_MHZ);
			ret = AB_SM_789_6_MHZ;
			break;
		default:
			ret = -EINVAL;
			break;
		}

	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_aon_set_rate_handler(struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;

	dev_dbg(clk_ctx->dev,
		"%s: set AON clock rate to %llu\n", __func__, new_rate);

	if (old_rate == AB_SM_466_MHZ &&
			new_rate != AB_SM_466_MHZ)
		__ab_clk_restore_mainclk_freq(clk_ctx);

	if (new_rate == AB_SM_OSC_RATE) {
		ret = clk_set_parent(clk_ctx->aon_pll_mux, clk_ctx->osc_clk);
		if (ret) {
			dev_err(clk_ctx->dev,
				"aon_pll_mux: set_parent failed(err %d)\n",
				ret);
			goto error_abort;
		}

		return new_rate;
	}

	ret = clk_set_parent(clk_ctx->aon_pll_mux, clk_ctx->aon_pll);
	if (ret) {
		dev_err(clk_ctx->dev,
			"aon_pll_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	if (new_rate == AB_SM_466_MHZ)
		__ab_clk_reduce_mainclk_freq(clk_ctx);

	return new_rate;

error_abort:
	return ret;
}

static int64_t ab_clk_aon_set_rate_handler(void *ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = __ab_clk_aon_set_rate_handler(clk_ctx,
				old_rate, new_rate);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

/*
 * NOTE: This method does not result in clk notifications for
 * mfd child drivers
 */
static int64_t ab_clk_aon_set_rate_direct_handler(void *ctx,
		u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		switch (new_rate) {
		case AB_SM_19_2_MHZ:
			ABC_WRITE(AON_CLK_RATE_REG, AON_CLK_RATE_19_2_MHZ);
			ret = AB_SM_19_2_MHZ;
			break;
		case AB_SM_921_6_MHZ:
			ABC_WRITE(AON_CLK_RATE_REG, AON_CLK_RATE_921_6_MHZ);
			ret = AB_SM_921_6_MHZ;
			break;
		default:
			ret = -EINVAL;
			break;
		}

	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

#define CLK_CON_DIV_DIV2_PLLCLK_MIF 0x10511804
#define CLK_CON_DIV_DIV4_PLLCLK_TPU 0x10041800
#define CLK_CON_DIV_DIV4_PLLCLK_IPU 0x10241800
#define CLK_CON_DIV_DIV4_PLLCLK_FSYS 0x10711804
#define CLK_CON_DIV_DIV4_PLLCLK_CORE 0x10f11804
#define CLK_CON_DIV_PLL_AON_CLK 0x10b1180c

static void __ab_clk_reduce_mainclk_freq(struct ab_clk_context *ctx)
{
	dev_dbg(ctx->dev, "Reduce PLL_AON_CLK 2x\n");

	/* Multiply sub clocks by 2x to compensate /2 */
	ABC_WRITE(CLK_CON_DIV_DIV2_PLLCLK_MIF, 0x0);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_TPU, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_IPU, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_FSYS, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_CORE, 0x0);

	/* Divide PLL_AON_CLK by 2 */
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x1);
}

static int ab_clk_reduce_mainclk_freq(void *ctx)
{
	int ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		__ab_clk_reduce_mainclk_freq(clk_ctx);
		ret = 0;
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

static void __ab_clk_restore_mainclk_freq(struct ab_clk_context *ctx)
{
	dev_dbg(ctx->dev, "Restore PLL_AON_CLK\n");

	/* Restore default divider settings to undo 2x multiply */
	ABC_WRITE(CLK_CON_DIV_DIV2_PLLCLK_MIF, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_TPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_IPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_FSYS, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_CORE, 0x1);

	/* Restore default divider setting to undo 2x divide */
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x0);
}

static int ab_clk_restore_mainclk_freq(void *ctx)
{
	int ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		__ab_clk_restore_mainclk_freq(clk_ctx);
		ret = 0;
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

#define SHARED_DIV_AON_PLL_REG				0x10B11810
#define SHARED_DIV_AON_PLL_DIVRATIO_MASK	0xF
#define SHARED_DIV_AON_PLL_DIVRATIO_2		0x1

static void ab_clk_init(void *ctx)
{
	u32 val;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	dev_dbg(clk_ctx->dev, "AB clock init\n");

	/* Set SHARED_DIV_AON_PLL to PLL_AON / 2 (nominally 466MHz)
	 */
	ABC_READ(SHARED_DIV_AON_PLL_REG, &val);
	val &= ~SHARED_DIV_AON_PLL_DIVRATIO_MASK;
	val |= SHARED_DIV_AON_PLL_DIVRATIO_2;
	ABC_WRITE(SHARED_DIV_AON_PLL_REG, val);
}

static struct ab_sm_clk_ops clk_ops = {
	.init = ab_clk_init,

	.ipu_pll_enable = &ab_clk_ipu_pll_enable_handler,
	.ipu_pll_disable = &ab_clk_ipu_pll_disable_handler,
	.ipu_set_rate = &ab_clk_ipu_set_rate_handler,

	.tpu_pll_enable = &ab_clk_tpu_pll_enable_handler,
	.tpu_pll_disable = &ab_clk_tpu_pll_disable_handler,
	.tpu_set_rate = &ab_clk_tpu_set_rate_handler,
	.tpu_set_rate_direct = &ab_clk_tpu_set_rate_direct_handler,

	.aon_set_rate = &ab_clk_aon_set_rate_handler,
	.aon_set_rate_direct = &ab_clk_aon_set_rate_direct_handler,

	.reduce_mainclk_freq = &ab_clk_reduce_mainclk_freq,
	.restore_mainclk_freq = &ab_clk_restore_mainclk_freq,
};

static int ab_clk_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct device_node *ab_clk_nd;
	struct ab_clk_context *clk_ctx;

	clk_ctx = devm_kzalloc(dev, sizeof(struct ab_clk_context), GFP_KERNEL);
	if (!clk_ctx)
		return -ENOMEM;

	clk_ctx->dev = dev;

	ab_clk_nd = of_find_node_by_name(dev->of_node, "abc-clk");
	if (!ab_clk_nd)
		return -ENODEV;

	/* Registering CMUs to Common Clock Framework.
	 * Parse through the ab device node and scan for cmu nodes.
	 * once found, register the same with the common clock framework
	 */
	for_each_child_of_node(ab_clk_nd, child) {
		if (of_device_is_compatible(child,
					"diablo,abc-clock-aon")) {
			abc_clk_aon_init(child);
		} else if (of_device_is_compatible(child,
					"diablo,abc-clock-ipu")) {
			abc_clk_ipu_init(child);
		} else if (of_device_is_compatible(child,
					"diablo,abc-clock-tpu")) {
			abc_clk_tpu_init(child);
		} else {
			dev_err(clk_ctx->dev,
				"incompatible child node (%s)\n", child->name);
		}
	}

	clk_ctx->ipu_pll =
		of_clk_get_by_name(ab_clk_nd, "ipu_pll");
	clk_ctx->ipu_pll_mux =
		of_clk_get_by_name(ab_clk_nd, "ipu_pll_mux");
	clk_ctx->ipu_pll_div =
		of_clk_get_by_name(ab_clk_nd, "ipu_pll_div");
	clk_ctx->ipu_switch_mux =
		of_clk_get_by_name(ab_clk_nd, "ipu_switch_mux");

	clk_ctx->tpu_pll =
		of_clk_get_by_name(ab_clk_nd, "tpu_pll");
	clk_ctx->tpu_pll_mux =
		of_clk_get_by_name(ab_clk_nd, "tpu_pll_mux");
	clk_ctx->tpu_pll_div =
		of_clk_get_by_name(ab_clk_nd, "tpu_pll_div");
	clk_ctx->tpu_switch_mux =
		of_clk_get_by_name(ab_clk_nd, "tpu_switch_mux");

	clk_ctx->osc_clk =
		of_clk_get_by_name(ab_clk_nd, "osc_clk");
	clk_ctx->shared_div_aon_pll =
		of_clk_get_by_name(ab_clk_nd, "shared_div_aon_pll");
	clk_ctx->aon_pll =
		of_clk_get_by_name(ab_clk_nd, "aon_pll");
	clk_ctx->aon_pll_mux =
		of_clk_get_by_name(ab_clk_nd, "aon_pll_mux");

	if (IS_ERR(clk_ctx->ipu_pll) ||
			IS_ERR(clk_ctx->ipu_pll_mux) ||
			IS_ERR(clk_ctx->ipu_pll_div) ||
			IS_ERR(clk_ctx->ipu_switch_mux) ||
			IS_ERR(clk_ctx->tpu_pll) ||
			IS_ERR(clk_ctx->tpu_pll_mux) ||
			IS_ERR(clk_ctx->tpu_pll_div) ||
			IS_ERR(clk_ctx->tpu_switch_mux) ||
			IS_ERR(clk_ctx->osc_clk) ||
			IS_ERR(clk_ctx->shared_div_aon_pll) ||
			IS_ERR(clk_ctx->aon_pll) ||
			IS_ERR(clk_ctx->aon_pll_mux)) {
		dev_err(clk_ctx->dev, "could not register all clocks\n");

		// TODO: unregister clocks?
		return -ENODEV;
	}

	platform_set_drvdata(pdev, clk_ctx);

	mutex_init(&clk_ctx->pcie_link_lock);
	clk_ctx->pcie_link_ready = true;
	clk_ctx->pcie_link_blocking_nb.notifier_call =
			ab_clk_pcie_link_listener;
	ret = abc_register_pcie_link_blocking_event(
			&clk_ctx->pcie_link_blocking_nb);
	if (ret) {
		dev_err(dev,
				"failed to subscribe to PCIe blocking link event, ret %d\n",
				ret);
		return ret;
	}

	clk_ops.ctx = clk_ctx;
	ab_sm_register_clk_ops(&clk_ops);

	return 0;
}

static int ab_clk_remove(struct platform_device *pdev)
{
	struct ab_clk_context *clk_ctx = platform_get_drvdata(pdev);

	ab_sm_unregister_clk_ops();

	// TODO: unregister clocks?
	if (clk_ctx->ipu_pll)
		clk_put(clk_ctx->ipu_pll);
	if (clk_ctx->ipu_pll_mux)
		clk_put(clk_ctx->ipu_pll_mux);
	if (clk_ctx->ipu_pll_div)
		clk_put(clk_ctx->ipu_pll_div);
	if (clk_ctx->ipu_switch_mux)
		clk_put(clk_ctx->ipu_switch_mux);

	if (clk_ctx->tpu_pll)
		clk_put(clk_ctx->tpu_pll);
	if (clk_ctx->tpu_pll_mux)
		clk_put(clk_ctx->tpu_pll_mux);
	if (clk_ctx->tpu_pll_div)
		clk_put(clk_ctx->tpu_pll_div);
	if (clk_ctx->tpu_switch_mux)
		clk_put(clk_ctx->tpu_switch_mux);

	if (clk_ctx->osc_clk)
		clk_put(clk_ctx->osc_clk);
	if (clk_ctx->shared_div_aon_pll)
		clk_put(clk_ctx->shared_div_aon_pll);
	if (clk_ctx->aon_pll)
		clk_put(clk_ctx->aon_pll);
	if (clk_ctx->aon_pll_mux)
		clk_put(clk_ctx->aon_pll_mux);

	return 0;
}

static const struct of_device_id ab_clk_of_match[] = {
	{ .compatible = "abc,airbrush-clk", },
	{ }
};

static struct platform_driver ab_clk_driver = {
	.probe = ab_clk_probe,
	.remove = ab_clk_remove,
	.driver = {
		.name = "ab-clk",
		.of_match_table = ab_clk_of_match,
	},
};
module_platform_driver(ab_clk_driver);

/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <dt-bindings/clock/audio-ext-clk.h>
#include <linux/clk/msm-clk-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/qdsp6v2/apr.h>
#include <linux/slab.h>
#include <soc/qcom/msm-clock-controller.h>
#include <sound/apr_audio-v2.h>
#include <sound/msm-dai-q6-v2.h>
#include <sound/q6afe-v2.h>

struct q6_mi2s_mclk {
	struct clk c;
	struct afe_clk_set clk_set;
	int port_id;
};

static long q6_mi2s_mclk_supported_rates[] = {
	Q6AFE_LPASS_IBIT_CLK_12_P288_MHZ,
	Q6AFE_LPASS_IBIT_CLK_11_P2896_MHZ,
	Q6AFE_LPASS_IBIT_CLK_8_P192_MHZ,
	Q6AFE_LPASS_IBIT_CLK_6_P144_MHZ,
	Q6AFE_LPASS_IBIT_CLK_4_P096_MHZ,
	Q6AFE_LPASS_IBIT_CLK_3_P072_MHZ,
	Q6AFE_LPASS_IBIT_CLK_2_P8224_MHZ,
	Q6AFE_LPASS_IBIT_CLK_2_P048_MHZ,
	Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ,
	Q6AFE_LPASS_IBIT_CLK_1_P4112_MHZ,
	Q6AFE_LPASS_IBIT_CLK_1_P024_MHZ,
	Q6AFE_LPASS_IBIT_CLK_768_KHZ,
	Q6AFE_LPASS_IBIT_CLK_512_KHZ,
	Q6AFE_LPASS_IBIT_CLK_256_KHZ,
	Q6AFE_LPASS_IBIT_CLK_DISABLE,
	-1				/* Invalid rate. */
};

#define to_q6_mi2s_mclk(_clk) container_of(_clk, struct q6_mi2s_mclk, c)

/*
 * Update the AFE with our clock configuration.
 * NOTE: This function may sleep.
 */
static unsigned int q6_mi2s_mclk_update_afe(struct clk *clk)
{
	struct q6_mi2s_mclk *pdata = to_q6_mi2s_mclk(clk);
	int ret;

	pr_debug("%s: %sable, freq: %u", clk_name(clk),
		 pdata->clk_set.enable ? "en" : "dis",
		 pdata->clk_set.clk_freq_in_hz);
	ret = afe_set_lpass_clock_v2(pdata->port_id, &pdata->clk_set);
	if (ret)
		pr_err("%s: failure updating afe: %d", clk_name(clk), ret);
	return ret;
}

/*
 * Implement the clk_ops.round_rate by finding the nearest supported
 * clock rate.
 */
static long q6_mi2s_mclk_round_rate(struct clk *clk, unsigned long rate)
{
	int i = 1;
	long prev_rate = q6_mi2s_mclk_supported_rates[0];
	long supported_rate;

	if (rate >= prev_rate)
		supported_rate = prev_rate;
	else while ((supported_rate = q6_mi2s_mclk_supported_rates[i++]) > -1) {
		if (prev_rate > rate && rate >= supported_rate) {
			if (prev_rate - rate < rate - supported_rate)
				supported_rate = prev_rate;
			break;
		}
	}

	if (supported_rate < 0) {
		pr_err("%s: Could not round rate %ld", clk_name(clk), rate);
		return -EINVAL;
	}

	if (supported_rate != rate)
		pr_debug("%s: Rounded %ld to %ld", clk_name(clk), rate,
			 supported_rate);
	return supported_rate;
}

/*
 * Set the clock's rate.
 */
static int q6_mi2s_mclk_set_rate(struct clk *clk, unsigned long rate)
{
	struct q6_mi2s_mclk *pdata = to_q6_mi2s_mclk(clk);
	long supported_rate;

	if ((u32)rate == pdata->clk_set.clk_freq_in_hz)
		return 0;

	supported_rate = q6_mi2s_mclk_round_rate(clk, rate);
	if (supported_rate != rate)
		return -EINVAL;

	pdata->clk_set.clk_freq_in_hz = rate;
	pr_debug("%s: clock frequency: %ldHz", clk_name(clk), rate);

	return q6_mi2s_mclk_update_afe(clk);
}

/*
 * Get the clock's rate (use the value last set.).
 */
static unsigned long q6_mi2s_mclk_get_rate(struct clk *clk)
{
	struct q6_mi2s_mclk *pdata = to_q6_mi2s_mclk(clk);

	return pdata->clk_set.clk_freq_in_hz;
}

/*
 * This function must be provided to avoid an error in the base clk driver.
 */
static int q6_mi2s_mclk_set_flags(struct clk *clk, unsigned flags)
{
	return 0;
}

/*
 * Prepare (enable) the clock - this function will sleep.
 */
static int q6_mi2s_mclk_prepare(struct clk *clk)
{
	struct q6_mi2s_mclk *pdata = to_q6_mi2s_mclk(clk);

	pdata->clk_set.enable = 1;
	return q6_mi2s_mclk_update_afe(clk);
}

/*
 * Unprepare (disable) the clock - this function will sleep.
 */
static void q6_mi2s_mclk_unprepare(struct clk *clk)
{
	struct q6_mi2s_mclk *pdata = to_q6_mi2s_mclk(clk);
	int ret;

	pdata->clk_set.enable = 0;
	ret = q6_mi2s_mclk_update_afe(clk);
	if (ret)
		pr_err("%s: clock unprepare error: %d", clk_name(clk), ret);
}

struct clk_ops q6_mi2s_mclk_ops = {
	.prepare = q6_mi2s_mclk_prepare,
	.unprepare = q6_mi2s_mclk_unprepare,
	.set_rate = q6_mi2s_mclk_set_rate,
	.get_rate = q6_mi2s_mclk_get_rate,
	.round_rate = q6_mi2s_mclk_round_rate,
	.set_flags = q6_mi2s_mclk_set_flags,
};

static struct q6_mi2s_mclk pri_mi2s_mclk = {
	.c = {
		.dbg_name = "pri_mi2s_mclk",
		.ops = &q6_mi2s_mclk_ops,
		CLK_INIT(pri_mi2s_mclk.c),
	},
	.clk_set = {
		.clk_id = Q6AFE_LPASS_CLK_ID_MCLK_1,
		.clk_set_minor_version = AFE_API_VERSION_I2S_CONFIG,
		.clk_attri = Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
		.clk_root = Q6AFE_LPASS_CLK_ROOT_DEFAULT,
		.clk_freq_in_hz = Q6AFE_LPASS_IBIT_CLK_12_P288_MHZ,
	},
	.port_id = AFE_PORT_ID_PRIMARY_MI2S_RX,
};

static struct q6_mi2s_mclk sec_mi2s_mclk = {
	.c = {
		.dbg_name = "sec_mi2s_mclk",
		.ops = &q6_mi2s_mclk_ops,
		CLK_INIT(sec_mi2s_mclk.c),
	},
	.clk_set = {
		.clk_id = Q6AFE_LPASS_CLK_ID_MCLK_2,
		.clk_set_minor_version = AFE_API_VERSION_I2S_CONFIG,
		.clk_attri = Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
		.clk_root = Q6AFE_LPASS_CLK_ROOT_DEFAULT,
		.clk_freq_in_hz = Q6AFE_LPASS_IBIT_CLK_12_P288_MHZ,
	},
	.port_id = AFE_PORT_ID_SECONDARY_MI2S_RX,
};

static struct q6_mi2s_mclk tert_mi2s_mclk = {
	.c = {
		.dbg_name = "tert_mi2s_mclk",
		.ops = &q6_mi2s_mclk_ops,
		CLK_INIT(tert_mi2s_mclk.c),
	},
	.clk_set = {
		.clk_id = Q6AFE_LPASS_CLK_ID_MCLK_3,
		.clk_set_minor_version = AFE_API_VERSION_I2S_CONFIG,
		.clk_attri = Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
		.clk_root = Q6AFE_LPASS_CLK_ROOT_DEFAULT,
		.clk_freq_in_hz = Q6AFE_LPASS_IBIT_CLK_12_P288_MHZ,
	},
	.port_id = AFE_PORT_ID_TERTIARY_MI2S_RX,
};

static struct clk_lookup q6_mi2s_mclk_list[] = {
	CLK_LIST(pri_mi2s_mclk),
	CLK_LIST(sec_mi2s_mclk),
	CLK_LIST(tert_mi2s_mclk),
};

static int q6_mi2s_mclk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	enum apr_subsys_state adsp_state;

	adsp_state = apr_get_q6_state();
	if (adsp_state != APR_SUBSYS_LOADED) {
		dev_dbg(dev, "Waiting for ADSP to be UP.");
		return -EPROBE_DEFER;
	}

	ret = of_msm_clock_register(dev->of_node, q6_mi2s_mclk_list,
				    ARRAY_SIZE(q6_mi2s_mclk_list));
	if (ret)
		dev_err(dev, "Failed to register clocks: %d", ret);
	else
		dev_info(dev, "Registered clocks.");
	return ret;
}

static struct of_device_id q6_mi2s_mclk_match_table[] = {
	{ .compatible = "qcom,q6-mi2s-mclk" },
	{}
};

static struct platform_driver q6_mi2s_mclk_driver = {
	.probe = q6_mi2s_mclk_probe,
	.driver = {
		.name = "qcom,q6-mi2s-mclk",
		.of_match_table = q6_mi2s_mclk_match_table,
		.owner = THIS_MODULE,
	},
};

int __init q6_mi2s_mclk_init(void)
{
	return platform_driver_register(&q6_mi2s_mclk_driver);
}
arch_initcall(q6_mi2s_mclk_init);

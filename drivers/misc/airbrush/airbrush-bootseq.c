/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/segment.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/airbrush-sm-ctrl.h>
#include <linux/clk.h>

#include "airbrush-ddr.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"
#include "airbrush-thermal.h"

#define REG_AON_PCI_OTP	0x10b3000c
#define REG_SRAM_ADDR	0x10b30374
#define REG_DDR_INIT	0x10b30378
#define REG_UPL_CMPL	0x10b30384
#define RAM_CRC_CLR	0x10b30390
#define RAM_CRC_En	0x10b30394
#define RAM_CRC_VAL	0x10b30398
#define GPB0_DRV	0x10b4006c

#define ROM_BL_ADDR	0
#define SRAM_PCI_OTP	0x20414
#define SRAM_PCI_BAR	(SRAM_PCI_OTP + 4*253)

#define SOC_PWRGOOD_WAIT_TIMEOUT	msecs_to_jiffies(100)
#define AB_READY_WAIT_TIMEOUT		msecs_to_jiffies(100)
#define POLL_USLEEP_MIN			(100)

static int enable_ref_clk(struct device *dev)
{
	struct clk *ref_clk = clk_get(dev, "ab_ref");

	if (!IS_ERR(ref_clk))
		return clk_prepare_enable(ref_clk);
	else
		return PTR_ERR(ref_clk);
}

void parse_fw(uint32_t *image_dw_buf, const unsigned char *image_buf,
		int image_size_dw)
{
	int dw, byte;

	for (dw = 0; dw < image_size_dw; dw++) {
		byte = dw * 4;
		image_dw_buf[dw] = (image_buf[byte + 3] << 24 |
			image_buf[byte + 2] << 16 | image_buf[byte + 1] << 8 |
			image_buf[byte]);
	}
}

/* Caller must hold ab_ctx->state_lock */
int ab_bootsequence(struct ab_state_context *ab_ctx, enum chip_state prev_state)
{
	int ret;
	struct platform_device *plat_dev = ab_ctx->pdev;
	unsigned long timeout;
	enum ab_chip_id saved_chip_id, raw_chip_id;

	if (!ab_ctx)
		return -EINVAL;

	if (ab_ctx->cold_boot) {
		ab_disable_pgood(ab_ctx);
		msm_pcie_assert_perst(1);

		/* Disable PCIe equalization only for Airbrush A0 parts.
		 *   A0 => PCIe EQ disable
		 *   B0 => PCIe EQ enable
		 */
		saved_chip_id = ab_get_chip_id(ab_ctx);
		if (saved_chip_id == CHIP_ID_B0) {
			dev_info(ab_ctx->dev,
				 "AB version is B0\n");
			msm_pcie_eq_ctrl(1, /*enable=*/true);
		} else if (saved_chip_id == CHIP_ID_A0) {
			dev_err(ab_ctx->dev,
				"AB version is A0, which is not fully supported\n");
			msm_pcie_eq_ctrl(1, /*enable=*/false);
		} else {
			dev_warn(ab_ctx->dev,
				"WARNING: AB version is unknown\n");
			msm_pcie_eq_ctrl(1, /*enable=*/false);
		}
	}

	ret = ab_pmic_on(ab_ctx);
	if (ret) {
		dev_err(ab_ctx->dev, "ERROR!!! PMIC failure during ABC Boot");
		return ret;
	}

	ret = enable_ref_clk(ab_ctx->dev);
	if (ret) {
		dev_err(ab_ctx->dev,
			"Unable to enable reference clock (err %d)",
			ret);
		return ret;
	}

	if (prev_state == CHIP_STATE_100) {
		/* M0 samples DDR_SR for its ddr_train sequence */
		ab_gpio_enable_ddr_sr(ab_ctx);

		/*
		 * DDRCKE_ISO stays on during suspend in production.
		 * Disable it during resume before asserting pgood
		 * (b/128545111).
		 */
		if (!ab_ctx->ddrcke_iso_clamp_wr)
			ab_gpio_disable_ddr_iso(ab_ctx);
	}

	if (ab_ctx->alternate_boot)
		ab_gpio_enable_fw_patch(ab_ctx);

	msm_pcie_deassert_perst(1);
	ab_enable_pgood(ab_ctx);

	timeout = jiffies + SOC_PWRGOOD_WAIT_TIMEOUT;
	/* Wait until soc_pwrgood is set by PMIC */
	while (!gpiod_get_value_cansleep(ab_ctx->soc_pwrgood) &&
			time_before(jiffies, timeout))
		usleep_range(POLL_USLEEP_MIN, POLL_USLEEP_MIN + 1);

	if (!gpiod_get_value_cansleep(ab_ctx->soc_pwrgood)) {
		dev_err(&plat_dev->dev, "ABC PWRGOOD is not enabled");
		return -EIO;
	}

	if (ab_ctx->cold_boot) {
		ab_sm_start_ts(AB_SM_TS_PCIE_ENUM);
		ret = ab_sm_enumerate_pcie(ab_ctx);
		ab_sm_record_ts(AB_SM_TS_PCIE_ENUM);
		if (ret)
			return ret;

		ab_ctx->cold_boot = false;

		/* Cross-check chip id after cold boot. */
		raw_chip_id = ab_get_raw_chip_id(ab_ctx);
		if (saved_chip_id != raw_chip_id) {
			dev_warn(ab_ctx->dev,
				 "saved_chip_id %d does not match raw_chip_id %d\n",
				 saved_chip_id, raw_chip_id);

			if (raw_chip_id == CHIP_ID_UNKNOWN) {
				dev_warn(ab_ctx->dev,
					 "Keep %d as chip_id\n",
					 ab_ctx->chip_id);
			} else {
				dev_warn(ab_ctx->dev,
					 "Use %d as chip_id\n",
					 raw_chip_id);
				ab_ctx->chip_id = raw_chip_id;
			}
		}

		ab_sm_start_ts(AB_SM_TS_LVCC_INIT);
		ab_lvcc_init(&ab_ctx->asv_info);
		ab_sm_record_ts(AB_SM_TS_LVCC_INIT);
	} else {
		ab_sm_start_ts(AB_SM_TS_PCIE_ENUM);

		if (ab_ctx->debug_skip_pcie_link_init) {
			dev_warn(ab_ctx->dev,
				 "Skip PCIe link resume for testing\n");
			ab_sm_record_ts(AB_SM_TS_PCIE_ENUM);
			return -ENODEV;
		}

		ret = ab_sm_enable_pcie(ab_ctx);
		ab_sm_record_ts(AB_SM_TS_PCIE_ENUM);
		if (ret)
			return ret;
	}

	timeout = jiffies + AB_READY_WAIT_TIMEOUT;
	while (!gpiod_get_value_cansleep(ab_ctx->ab_ready) &&
			time_before(jiffies, timeout)) {
		usleep_range(POLL_USLEEP_MIN, POLL_USLEEP_MIN + 1);
	}

	if (!gpiod_get_value_cansleep(ab_ctx->ab_ready)) {
		dev_err(&plat_dev->dev,
			"ab_ready is not high after fw load\n");
		return -EIO;
	}

	ab_sm_start_ts(AB_SM_TS_AB_READY_NOTIFY);
	mutex_lock(&ab_ctx->mfd_lock);
	ret = ab_ctx->mfd_ops->ab_ready(ab_ctx->mfd_ops->ctx);
	mutex_unlock(&ab_ctx->mfd_lock);
	ab_sm_record_ts(AB_SM_TS_AB_READY_NOTIFY);

	/* b/129788388: configure PCIe PCS block to disable PLL at the start of
	 * P1.CPM state
	 */
	ABC_WRITE(PCS_OUT_VEC_4, 0x700DD);

	/* Enable schmitt trigger mode for SPI clk pad.
	 * This is to filter out any noise on SPI clk line.
	 * Also reset the SPI controller in case it's already
	 * in a glitched state.
	 */
	ABC_WRITE(GPB0_DRV, 0x22222262);
	ABC_WRITE(SYSREG_AON_SPI0_AHB_ENABLE, 0x0);
	ABC_WRITE(SYSREG_AON_SPI0_AHB_ENABLE, 0x1);

	/* Set clocks to usable states */
	ab_sm_start_ts(AB_SM_TS_CLK_INIT);
	ab_ctx->clk_ops->init(ab_ctx->clk_ops->ctx);
	ab_sm_record_ts(AB_SM_TS_CLK_INIT);

	ab_sm_start_ts(AB_SM_TS_DDR_INIT);
	/* Setup the function pointer to read DDR OTPs */
	ret = ab_ctx->dram_ops->setup(ab_ctx->dram_ops->ctx, ab_ctx);
	if (ret) {
		dev_err(ab_ctx->dev, "ddr setup failed\n");
		return ret;
	}
	/* Wait till the ddr init & training is completed in case of ddr
	 * initialization is done by BootROM
	 */
	if (IS_M0_DDR_INIT()) {
		if (ab_ctx->dram_ops->wait_for_m0_ddr_init(
					ab_ctx->dram_ops->ctx))
			return -EIO;
	}
	ret = ab_ctx->dram_ops->init(ab_ctx->dram_ops->ctx);
	if (ret) {
		dev_err(ab_ctx->dev, "ddr init failed\n");
		return ret;
	}
	ab_sm_record_ts(AB_SM_TS_DDR_INIT);

	/*
	 * Enable thermal after boot sequence finished successfully. Do not
	 * let thermal throttle intefere the bootsequence.
	 */
	ab_thermal_enable(ab_ctx->thermal);

	return 0;
}
EXPORT_SYMBOL(ab_bootsequence);

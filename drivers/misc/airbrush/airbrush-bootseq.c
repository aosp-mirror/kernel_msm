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
#include <linux/msm_pcie.h>

#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"
#include "airbrush-spi.h"

#define REG_SRAM_ADDR	0x10b30374
#define REG_DDR_INIT	0x10b30378
#define REG_UPL_CMPL	0x10b30384
#define RAM_CRC_CLR	0x10b30390
#define RAM_CRC_En	0x10b30394
#define RAM_CRC_VAL	0x10b30398
#define GPB0_DRV	0x10b4006c

#define SRAM_BL_ADDR	0x20000

#define SOC_PWRGOOD_WAIT_TIMEOUT	msecs_to_jiffies(100) /* TBD */
#define AB_READY_WAIT_TIMEOUT		msecs_to_jiffies(100) /* TBD */

#define M0_FIRMWARE_PATH "ab.fw"

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

int ab_bootsequence(struct ab_state_context *ab_ctx, bool patch_fw)
{
	/* Number of attempts to flash SRAM bootcode when CRC error happens */
	int num_attempts = 1;
	int gpio_cke_in, gpio_ddr_sr, crc_ok = 1;
	int state_suspend, state_off;
	unsigned int data;
	struct airbrush_spi_packet spi_packet;
	const struct firmware *fw_entry;
	uint32_t *image_dw_buf;
	int image_size_dw;
	int fw_status;
	struct pci_bus *pbus = 0;
	struct pci_dev *pdev = 0;
	int ret;
	struct platform_device *plat_dev = ab_ctx->pdev;
	unsigned long timeout;
	static int abc_clk_registered;

	if (!ab_ctx)
		return -EINVAL;

	if (!ab_ctx->ab_sm_ctrl_pmic) {
		dev_dbg(ab_ctx->dev, "%s: No ABC booting by state manager\n",
				__func__);

		/* Setup the function pointer to read DDR OTPs */
		ab_ddr_setup(ab_ctx);

		if (!abc_clk_registered) {
			abc_clk_register(ab_ctx);
			abc_clk_registered = 1;
		}

		return 0;
	}

	ret = ab_get_pmic_resources(ab_ctx);
	if (ret)
		return ret;

	/* Get the current state of CKE_IN, DDR_SR GPIOs */
	gpio_cke_in = ab_gpio_get_ddr_iso(ab_ctx);
	gpio_ddr_sr = ab_gpio_get_ddr_sr(ab_ctx);

	state_off = (ab_ctx->chip_substate_id == CHIP_STATE_6_0) ||
			    ((gpio_cke_in == 0) && (gpio_ddr_sr == 0));
	state_suspend = (ab_ctx->chip_substate_id == CHIP_STATE_5_0);

	if (state_suspend || state_off) {
		ab_disable_pgood(ab_ctx);

		ret = ab_pmic_on(ab_ctx);
		if (ret) {
			pr_err("ERROR!!! PMIC failure during ABC Boot\n");
			return ret;
		}

		ret = enable_ref_clk(ab_ctx->dev);
		if (ret) {
			dev_err(ab_ctx->dev,
				"Unable to enable reference clock (err %d)",
				ret);
			return ret;
		}

		if (patch_fw && !ab_ctx->otp_fw_patch_dis) {
			ab_gpio_enable_fw_patch(ab_ctx);

			fw_status =
				request_firmware(&fw_entry,
					M0_FIRMWARE_PATH, ab_ctx->dev);
			if (fw_status != 0) {
				pr_info("Airbrush Firmware Not Found: %d, %d\n",
						fw_status, __LINE__);
				return -EIO;
			}
			image_size_dw = fw_entry->size / 4;
			image_dw_buf = vmalloc(image_size_dw *
					sizeof(uint32_t));
			parse_fw(image_dw_buf, fw_entry->data, image_size_dw);
			release_firmware(fw_entry);
		}

		if (state_suspend)
			ab_gpio_enable_ddr_sr(ab_ctx);

		ab_enable_pgood(ab_ctx);

		timeout = jiffies + SOC_PWRGOOD_WAIT_TIMEOUT;
		/* Wait till the soc_pwrgood is put to high by PMIC */
		while (!gpiod_get_value_cansleep(ab_ctx->soc_pwrgood) &&
				time_before(jiffies, timeout))
			usleep_range(100, 105);

		if (!gpiod_get_value_cansleep(ab_ctx->soc_pwrgood)) {
			dev_err(&plat_dev->dev,
					"ABC PWRGOOD is not enabled");
			return -EIO;
		}

		/* If Airbrush OTP is read OTP_FW_PATCH_DIS=1, we should not
		 * go for secondary boot.
		 */
		if (patch_fw && !ab_ctx->otp_fw_patch_dis) {
			/* Wait for AB_READY = 1,
			 * this ensures the SPI FSM is initialized to flash the
			 * alternate bootcode to SRAM.
			 */
			timeout = jiffies + AB_READY_WAIT_TIMEOUT;
			while (!gpiod_get_value_cansleep(ab_ctx->ab_ready) &&
					time_before(jiffies, timeout))
				usleep_range(100, 105);

			if (!gpiod_get_value_cansleep(ab_ctx->ab_ready)) {
				dev_err(&plat_dev->dev,
					"ab_ready is not High");
				return -EIO;
			}

			/* Enable CRC via SPI-FSM */
			while (num_attempts) {
				/* [TBD] Reset CRC Register via SPI-FSM */

				/* [TBD] Enable CRC */

				/* Initiate FW patch upload via SPI-FSM */
				spi_packet.command = AB_SPI_CMD_FSM_BURST_WRITE;
				spi_packet.granularity = FOUR_BYTE;
				spi_packet.base_address = SRAM_BL_ADDR;
				spi_packet.data_length = image_size_dw;
				spi_packet.data = image_dw_buf;

				if (airbrush_spi_run_cmd(&spi_packet)) {
					vfree(image_dw_buf);
					return -EIO;
				}

				/* [TBD] Read CRC Register via SPI-FSM */

				/* if CRC is OK, break the loop */
				if (crc_ok)
					break;

				/* Decrement the number of attempts and retry */
				num_attempts--;

				if (!num_attempts) {
					vfree(image_dw_buf);
					return -EIO;
				}
			}

			/* set REG_SRAM_ADDR=SRAM_BL_ADDR via SPI-FSM */
			data = SRAM_BL_ADDR;
			spi_packet.command = AB_SPI_CMD_FSM_WRITE_SINGLE;
			spi_packet.granularity = FOUR_BYTE;
			spi_packet.base_address = REG_SRAM_ADDR;
			spi_packet.data_length = 1;
			spi_packet.data = &data;

			if (airbrush_spi_run_cmd(&spi_packet)) {
				vfree(image_dw_buf);
				return -EIO;
			}

			/* set REG_UPL_CMPL=1  via SPI-FSM */
			data = 0x1; // used to write REG_UPL_CMPL = 0x1
			spi_packet.command = AB_SPI_CMD_FSM_WRITE_SINGLE;
			spi_packet.granularity = FOUR_BYTE;
			spi_packet.base_address = REG_UPL_CMPL;
			spi_packet.data_length = 1;
			spi_packet.data = &data;

			if (airbrush_spi_run_cmd(&spi_packet)) {
				vfree(image_dw_buf);
				return -EIO;
			}

			/* Read the REG_UPL_COMPL register to make sure the
			 * AB_READY is deasserted by the Airbrush.
			 * Note: Assumption is that, by the time host reads the
			 * REG_UPL_CMPL register, Airbrush would have deasserted
			 * the AB_READY gpio.
			 */
			spi_packet.command = AB_SPI_CMD_FSM_READ_SINGLE;
			spi_packet.granularity = FOUR_BYTE;
			spi_packet.base_address = REG_UPL_CMPL;
			spi_packet.data_length = 1;
			spi_packet.data = NULL;


			if (airbrush_spi_run_cmd(&spi_packet)) {
				vfree(image_dw_buf);
				return -EIO;
			}
		}

		if (!ab_ctx->pcie_enumerated) {
			if (msm_pcie_enumerate(1)) {
				pr_err("PCIe enumeration failed\n");
				return -EIO;
			}

			ab_ctx->pcie_enumerated = true;

			pbus = pci_find_bus(1, 1);
			if (pbus) {
				pdev = pbus->self;
				while (!pci_is_root_bus(pbus)) {
					pdev = pbus->self;
					pbus = pbus->self->bus;
				}
				ab_ctx->pcie_dev = pdev;
			}
		} else {
			ret = msm_pcie_pm_control(MSM_PCIE_RESUME, 0,
				ab_ctx->pcie_dev, NULL,
				MSM_PCIE_CONFIG_NO_CFG_RESTORE);
			if (ret)
				pr_err("PCIe failed to enable link\n");
			else
				msm_pcie_recover_config(ab_ctx->pcie_dev);
		}

		/* Enable schmitt trigger mode for SPI clk pad.
		 * This is to filter out any noise on SPI clk line.
		 */
		ABC_WRITE(GPB0_DRV, 0x22222262);

		/* Wait for AB_READY = 1,
		 * this ensures the SPI FSM is initialized to flash the
		 * alternate bootcode to SRAM.
		 */
		timeout = jiffies + AB_READY_WAIT_TIMEOUT;
		while (!gpiod_get_value_cansleep(ab_ctx->ab_ready) &&
				time_before(jiffies, timeout)) {
			usleep_range(100, 105);
		}

		if (!gpiod_get_value_cansleep(ab_ctx->ab_ready)) {
			dev_err(&plat_dev->dev,
				"ab_ready is not High");
			return -EIO;
		}
	}

	/* Setup the function pointer to read DDR OTPs */
	ab_ddr_setup(ab_ctx);

	/* In case the M0_DDR_INIT (Renamed HOST_DDR_INIT) is 1,
	 * perform the DDR Initialization here.
	 */
	if (IS_HOST_DDR_INIT())
		ab_ddr_init(ab_ctx);

	if (!abc_clk_registered) {
		abc_clk_register(ab_ctx);
		abc_clk_registered = 1;
	}

	return 0;
}
EXPORT_SYMBOL(ab_bootsequence);

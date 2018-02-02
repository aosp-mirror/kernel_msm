/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * Airbrush State Manager Control driver..
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/debugfs.h>

#include "airbrush-sm-ctrl.h"
#include "airbrush-spi.h"

static struct ab_state_context *ab_sm_ctx;

#ifdef __DEBUG_FS
static int ab_debugfs_boot(void *data, u64 val)
{
	switch (val) {
	case 0:
		ab_bootsequence(ab_sm_ctx, 1);
		break;
	case 1:
		ab_interrupt_M0(0);
		break;
	default:
		printk("Unsupported value\n");

	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(airbrush_sm_ctrl_fops, NULL, ab_debugfs_boot, "%lli\n");
#endif

struct ab_state_context *ab_sm_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int error;

	/* allocate device memory */
	ab_sm_ctx = devm_kzalloc(dev, sizeof(struct ab_state_context),
							GFP_KERNEL);
	ab_sm_ctx->pdev = pdev;
	ab_sm_ctx->dev = &pdev->dev;

	/* Get the gpio_desc for all the gpios used */

	/* FW_PATCH_EN is used to inform Airbrush about host is interested in
	 * secondary SRAM boot. This will help Airbrush to put SPI in FSM Mode.
	 */
	ab_sm_ctx->fw_patch_en = devm_gpiod_get(&pdev->dev, "fw-patch-en",
					GPIOD_OUT_LOW);
	if (IS_ERR(ab_sm_ctx->fw_patch_en)) {
		dev_err(dev, "%s: could not get fw-patch-en gpio (%ld)\n",
			__func__, PTR_ERR(ab_sm_ctx->fw_patch_en));
		error = PTR_ERR(ab_sm_ctx->fw_patch_en);
		goto fail_fw_patch_en;
	}

	gpiod_set_value(ab_sm_ctx->fw_patch_en, __GPIO_ENABLE);
	/* AB_READY is used by host to understand that Airbrush SPI is now in
	 * FSM mode and host can start the SPI FSM commands to Airbrush.
	 */
	ab_sm_ctx->ab_ready = devm_gpiod_get(&pdev->dev, "ab-ready", GPIOD_IN);
	if (IS_ERR(ab_sm_ctx->ab_ready)) {
		dev_err(dev, "%s: could not get ab-ready gpio (%ld)\n",
			__func__, PTR_ERR(ab_sm_ctx->ab_ready));
		error = PTR_ERR(ab_sm_ctx->ab_ready);
		goto fail_ab_ready;
	}

	/* Get the otp-fw-patch-dis property from dt node. This property
	 * provides the OTP information of Airbrush for allowing the secondary
	 * boot via SRAM.
	 */
	if (of_property_read_u32(np, "otp-fw-patch-dis",
				&ab_sm_ctx->otp_fw_patch_dis))
		dev_info(dev, "otp-fw-patch-dis property not found\n");

	/* [TBD] Need DDR_SR, DDR_TRAIN, CKE_IN, CKE_IN_SENSE GPIOs for  */

#ifdef __DEBUG_FS

	ab_sm_ctx->d_entry = debugfs_create_dir("airbrush", NULL);
	if (!ab_sm_ctx->d_entry) {
		dev_err(dev, "failed to create debugfs entry for airbrush");
		return NULL;
	}
	debugfs_create_file("ab_boot", 0444, ab_sm_ctx->d_entry, NULL,
			&airbrush_sm_ctrl_fops);

#endif
	return ab_sm_ctx;

fail_fw_patch_en:
fail_ab_ready:
	devm_kfree(dev, (void *)ab_sm_ctx);
	ab_sm_ctx = NULL;

	return NULL;
}
EXPORT_SYMBOL(ab_sm_init);

int ab_sm_set_state(struct ab_state_context *sc,
			uint32_t device, enum ab_sm_state s)
{
	/* Look for the current state and depending on this try to move the
	 * ABC device state to the requested state
	 */

	/* Incase of power-on/resume requested state, follow the bootflow
	 * diagram. Need to control AP/HOST PMIC/PMU settings to control the
	 * ABC state
	 */

	return 0;
}
EXPORT_SYMBOL(ab_sm_set_state);


enum ab_sm_state ab_sm_get_state(struct ab_state_context *sc, uint32_t device)
{
	return 0;
}
EXPORT_SYMBOL(ab_sm_get_state);


int ab_sm_register_callback(struct ab_state_context *sc,
				ab_sm_callback_t cb, void *cookie)
{
	/* Update the context structure with the event callback information */
	sc->cb_event = cb;
	sc->cb_cookie = cookie;

	return 0;
}
EXPORT_SYMBOL(ab_sm_register_callback);

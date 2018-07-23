/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha <shaik.ameer@samsung.com>
 * Author: Raman Kumar Banka (raman.k2@samsung.com)
 *
 * Airbrush State Manager Control driver..
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>

#include "airbrush-sm-ctrl.h"
#include "airbrush-spi.h"
#include "airbrush-pmu.h"

static struct ab_state_context *ab_sm_ctx;

static struct block_property ipu_property_table[] = {
	{BLOCK_STATE_0_0,	"Normal",	"Ready",	on,	VOLTAGE_0_75,	off,	550000000,	14,	0,	0,	0},
	{BLOCK_STATE_0_1,	"Normal",	"AonCompute",	on,	VOLTAGE_0_75,	on,	50000000,	2,	2,	0,	0},
	{BLOCK_STATE_0_2,	"Normal",	"MinCompute",	on,	VOLTAGE_0_75,	on,	220000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_3,	"Normal",	"LowCompute",	on,	VOLTAGE_0_75,	on,	330000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_4,	"Normal",	"MidCompute",	on,	VOLTAGE_0_75,	on,	440000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_5,	"Normal",	"MaxCompute",	on,	VOLTAGE_0_75,	on,	550000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_6,	"Boost",	"MaxCompute",	on,	VOLTAGE_0_85,	on,	610000000,	14,	14,	0,	0},
	{BLOCK_STATE_1_0,	"PowerDown",	"PowerGated",	on,	VOLTAGE_0_85,	off,	610000000,	0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",	off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0}
};

static struct block_property tpu_property_table[] = {
	{BLOCK_STATE_0_0,	"Normal",	"Ready",	on,	VOLTAGE_0_75,	off,	766000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_1,	"Normal",	"AonCompute",	on,	VOLTAGE_0_75,	on,	50000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_2,	"Normal",	"MinCompute",	on,	VOLTAGE_0_75,	on,	306000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_3,	"Normal",	"LowCompute",	on,	VOLTAGE_0_75,	on,	460000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_4,	"Normal",	"MidCompute",	on,	VOLTAGE_0_75,	on,	612000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_5,	"Normal",	"MaxCompute",	on,	VOLTAGE_0_75,	on,	766000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_6,	"Boost",	"MaxCompute",	on,	VOLTAGE_0_85,	on,	962000000,	0,	0,	16,	0},
	{BLOCK_STATE_1_0,	"PowerDown",	"PowerGated",	on,	VOLTAGE_0_85,	off,	962000000,	0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",	off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0}
};

static struct block_property dram_property_table[] = {
	{BLOCK_STATE_0_0,	"PowerUp",	"Standby",		on,	VOLTAGE_0_60,	off,	1867000000,	0,	0,	0,	3733},
	{BLOCK_STATE_0_1,	"PowerUp",	"AonTransact",		on,	VOLTAGE_0_60,	on,	200000000,	0,	0,	0,	400},
	{BLOCK_STATE_0_2,	"PowerUp",	"HalfMidTransact",	on,	VOLTAGE_0_60,	on,	800000000,	0,	0,	0,	1600},
	{BLOCK_STATE_0_3,	"PowerUp",	"HalfMaxTransact",	on,	VOLTAGE_0_60,	on,	934000000,	0,	0,	0,	1867},
	{BLOCK_STATE_0_4,	"PowerUp",	"LowTransact",		on,	VOLTAGE_0_60,	on,	1200000000,	0,	0,	0,	2400},
	{BLOCK_STATE_0_5,	"PowerUp",	"MidTransact",		on,	VOLTAGE_0_60,	on,	1600000000,	0,	0,	0,	3200},
	{BLOCK_STATE_0_6,	"PowerUp",	"MaxTransact",		on,	VOLTAGE_0_60,	on,	1867000000,	0,	0,	0,	3733},
	{BLOCK_STATE_1_0,	"PowerDown",	"ClockOff",		on,	VOLTAGE_0_60,	off,	1867000000,	0,	0,	0,	3733},
	{BLOCK_STATE_1_1,	"PowerDown",	"ClockOn",		on,	VOLTAGE_0_60,	on,	1867000000,	0,	0,	0,	3733},
	{BLOCK_STATE_2_0,	"Retention",	"SelfRefresh",		on,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",		off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0}
};

static struct block_property mif_property_table[] = {
	{BLOCK_STATE_0_0,	"Normal",	"Ready",		on,	VOLTAGE_0_85,	off,	933000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_1,	"Normal",	"AonTransact",		on,	VOLTAGE_0_85,	on,	50000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_2,	"Normal",	"HalfMidTransact",	on,	VOLTAGE_0_85,	on,	200000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_3,	"Normal",	"HalfMaxTransact",	on,	VOLTAGE_0_85,	on,	233000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_4,	"Normal",	"LowTransact",		on,	VOLTAGE_0_85,	on,	300000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_5,	"Normal",	"MidTransact",		on,	VOLTAGE_0_85,	on,	400000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_6,	"Normal",	"MaxTransact",		on,	VOLTAGE_0_85,	on,	467000000,	0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",		off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0}
};

static struct block_property fsys_property_table[] = {
	{BLOCK_STATE_0_0,	"ElectricalIdle",	"L0s",		on,	VOLTAGE_0_85,	off,	4000000000,	0,	0,	0,	8000},
	{BLOCK_STATE_0_1,	"PowerUp",		"Gen1L0",	on,	VOLTAGE_0_85,	on,	1250000000,	0,	0,	0,	2000},
	{BLOCK_STATE_0_2,	"PowerUp",		"Gen2L0",	on,	VOLTAGE_0_85,	on,	2500000000,	0,	0,	0,	4000},
	{BLOCK_STATE_0_3,	"PowerUp",		"Gen3L0",	on,	VOLTAGE_0_85,	on,	4000000000,	0,	0,	0,	8000},
	{BLOCK_STATE_1_0,	"ElectricalIdle",	"Gen3L1",	on,	VOLTAGE_0_85,	on,	4000000000,	0,	0,	0,	0},
	{BLOCK_STATE_1_1,	"ElectricalIdle",	"Gen3L1.1",	on,	VOLTAGE_0_85,	on,	0,		0,	0,	0,	0},
	{BLOCK_STATE_1_2,	"ElectricalIdle",	"Gen3L1.2",	on,	VOLTAGE_0_85,	on,	0,		0,	0,	0,	0},
	{BLOCK_STATE_2_0,	"Hibernate",		"L2",		on,	VOLTAGE_0_85,	on,	0,		0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",		"L3",		off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0}
};

static struct block_property aon_property_table[] = {
	{BLOCK_STATE_0_0,	"PowerUp",	"WFI",		on,	VOLTAGE_0_85,	off,	233000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_1,	"PowerUp",	"Boot",		on,	VOLTAGE_0_85,	on,	19200000,	0,	0,	0,	0},
	{BLOCK_STATE_0_2,	"PowerUp",	"Compute",	on,	VOLTAGE_0_85,	on,	233000000,	0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",	off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0}
};

static struct chip_to_block_map chip_state_map[] = {
	{CHIP_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_2, BLOCK_STATE_0_2, BLOCK_STATE_0_2, BLOCK_STATE_0_2, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_3, BLOCK_STATE_0_3, BLOCK_STATE_0_3, BLOCK_STATE_0_4, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_4, BLOCK_STATE_0_4, BLOCK_STATE_0_4, BLOCK_STATE_0_5, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_5, BLOCK_STATE_0_5, BLOCK_STATE_0_2, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_6, BLOCK_STATE_0_5, BLOCK_STATE_0_2, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_7, BLOCK_STATE_0_2, BLOCK_STATE_0_5, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_8, BLOCK_STATE_0_2, BLOCK_STATE_0_5, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_0_9, BLOCK_STATE_0_5, BLOCK_STATE_0_5, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_1_0, BLOCK_STATE_0_0, BLOCK_STATE_1_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_1_1, BLOCK_STATE_0_1, BLOCK_STATE_1_0, BLOCK_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_1_2, BLOCK_STATE_0_2, BLOCK_STATE_1_0, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_1_3, BLOCK_STATE_0_2, BLOCK_STATE_1_0, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_1_4, BLOCK_STATE_0_4, BLOCK_STATE_1_0, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_1_5, BLOCK_STATE_0_5, BLOCK_STATE_1_0, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_1_6, BLOCK_STATE_0_6, BLOCK_STATE_1_0, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_2_0, BLOCK_STATE_1_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, BLOCK_STATE_0_0, TPU_POWER_CONTROL},
	{CHIP_STATE_2_1, BLOCK_STATE_1_0, BLOCK_STATE_0_1, BLOCK_STATE_0_6, BLOCK_STATE_0_1, BLOCK_STATE_0_1, BLOCK_STATE_0_0, TPU_POWER_CONTROL},
	{CHIP_STATE_2_2, BLOCK_STATE_1_0, BLOCK_STATE_0_2, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, TPU_POWER_CONTROL},
	{CHIP_STATE_2_3, BLOCK_STATE_1_0, BLOCK_STATE_0_3, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, TPU_POWER_CONTROL},
	{CHIP_STATE_2_4, BLOCK_STATE_1_0, BLOCK_STATE_0_4, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, TPU_POWER_CONTROL},
	{CHIP_STATE_2_5, BLOCK_STATE_1_0, BLOCK_STATE_0_5, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, TPU_POWER_CONTROL},
	{CHIP_STATE_2_6, BLOCK_STATE_1_0, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_6, BLOCK_STATE_0_2, BLOCK_STATE_0_0, TPU_POWER_CONTROL},
	{CHIP_STATE_3_0, BLOCK_STATE_1_0, BLOCK_STATE_1_0, BLOCK_STATE_2_0, BLOCK_STATE_0_0, BLOCK_STATE_1_1, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_4_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, BLOCK_STATE_2_0, BLOCK_STATE_0_0, BLOCK_STATE_1_1, BLOCK_STATE_0_0, IPU_POWER_CONTROL},
	{CHIP_STATE_5_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, BLOCK_STATE_2_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, IPU_POWER_CONTROL},
	{CHIP_STATE_6_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, BLOCK_STATE_3_0, IPU_POWER_CONTROL},
};

int ipu_set_state(struct device *dev, struct block *blk_ipu,
		  u32 to_block_state_id, bool power_control)
{
	struct block_property *desired_state;
	int i;

	desired_state = NULL;
	for (i = 0; i < (blk_ipu->nr_block_states); i++) {
		if (blk_ipu->block_property_table[i].id == to_block_state_id)
			desired_state = &(blk_ipu->block_property_table[i]);
	}

	if (!desired_state)
		return -EINVAL;

	/* clock related demo code */
	ipu_set_rate(dev, desired_state->clk_frequency);
	blk_ipu->current_state = desired_state;

	//TODO: change the properties from current_state to desired_state
	return 0;
}

int tpu_set_state(struct device *dev, struct block *blk_tpu,
		  u32 to_block_state_id, bool power_control)
{
	struct block_property *desired_state;
	int i;

	desired_state = NULL;
	for (i = 0; i < (blk_tpu->nr_block_states); i++) {
		if (blk_tpu->block_property_table[i].id == to_block_state_id)
			desired_state = &(blk_tpu->block_property_table[i]);
	}

	if (!desired_state)
		return -EINVAL;

	/* clock related demo code */
	tpu_set_rate(dev, desired_state->clk_frequency);
	blk_tpu->current_state = desired_state;
#if 0
	if (current_state->id == desired_state->id)
		//no change needed
#endif
	//TODO: change the properties from current_state to desired_state
	return 0;
}

int dram_set_state(struct device *dev, struct block *blk_dram,
		   u32 to_block_state_id, bool power_control)
{
	struct block_property *desired_state;
	int i;

	desired_state = NULL;
	for (i = 0; i < (blk_dram->nr_block_states); i++) {
		if (blk_dram->block_property_table[i].id == to_block_state_id)
			desired_state = &(blk_dram->block_property_table[i]);
	}

	if (!desired_state)
		return -EINVAL;

	//TODO: change the properties from current_state to desired_state
	return 0;
}

int mif_set_state(struct device *dev, struct block *blk_mif,
		  u32 to_block_state_id, bool power_control)
{
	struct block_property *desired_state;
	int i;

	desired_state = NULL;
	for (i = 0; i < (blk_mif->nr_block_states); i++) {
		if (blk_mif->block_property_table[i].id == to_block_state_id)
			desired_state = &(blk_mif->block_property_table[i]);
	}

	if (!desired_state)
		return -EINVAL;

	//TODO: change the properties from current_state to desired_state
	return 0;
}

int fsys_set_state(struct device *dev, struct block *blk_fsys,
		   u32 to_block_state_id, bool power_control)
{
	struct block_property *desired_state;
	int i;

	desired_state = NULL;
	for (i = 0; i < (blk_fsys->nr_block_states); i++) {
		if (blk_fsys->block_property_table[i].id == to_block_state_id)
			desired_state = &(blk_fsys->block_property_table[i]);
	}

	if (!desired_state)
		return -EINVAL;

	//TODO: change the properties from current_state to desired_state
	return 0;
}

int aon_set_state(struct device *dev, struct block *blk_aon,
		  u32 to_block_state_id, bool power_control)
{
	struct block_property *desired_state;
	int i;

	desired_state = NULL;
	for (i = 0; i < (blk_aon->nr_block_states); i++) {
		if (blk_aon->block_property_table[i].id == to_block_state_id)
			desired_state = &(blk_aon->block_property_table[i]);
	}

	if (!desired_state)
		return -EINVAL;

	//TODO: change the properties from current_state to desired_state
	return 0;
}

int ab_sm_set_state(struct ab_state_context *sc, u32 to_sw_state_id,
		    u32 to_chip_substate_id)
{
	int i;
	struct chip_to_block_map *map = NULL;

	for (i = 0; i < sc->nr_chip_states; i++) {
		if (sc->chip_state_table[i].chip_substate_id == to_chip_substate_id) {
			map = &(sc->chip_state_table[i]);
			break;
		}
	}

	if (!map)
		return -EINVAL;

	if (ipu_set_state(sc->dev, &(sc->blocks[BLK_IPU]),
		      map->ipu_block_state_id, (map->flags & IPU_POWER_CONTROL)))
		return -EINVAL;

	if (tpu_set_state(sc->dev, &(sc->blocks[BLK_TPU]),
		      map->tpu_block_state_id, (map->flags & TPU_POWER_CONTROL)))
		return -EINVAL;

	if (dram_set_state(sc->dev, &(sc->blocks[DRAM]),
		      map->dram_block_state_id, (map->flags & DRAM_POWER_CONTROL)))
		return -EINVAL;

	if (mif_set_state(sc->dev, &(sc->blocks[BLK_MIF]),
		      map->dram_block_state_id, (map->flags & MIF_POWER_CONTROL)))
		return -EINVAL;

	if (fsys_set_state(sc->dev, &(sc->blocks[BLK_FSYS]),
		      map->fsys_block_state_id, (map->flags & FSYS_POWER_CONTROL)))
		return -EINVAL;

	if (aon_set_state(sc->dev, &(sc->blocks[BLK_AON]),
		      map->aon_block_state_id,(map->flags & AON_POWER_CONTROL)))
		return -EINVAL;

	sc->chip_substate_id = to_chip_substate_id;
	return 0;
}
EXPORT_SYMBOL(ab_sm_set_state);

int ab_sm_register_callback(struct ab_state_context *sc,
				ab_sm_callback_t cb, void *cookie)
{
	/* Update the context structure with the event callback information */
	sc->cb_event = cb;
	sc->cb_cookie = cookie;

	return 0;
}
EXPORT_SYMBOL(ab_sm_register_callback);

struct ab_state_context *ab_sm_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int error, host_gpio_ctrl;

	/* allocate device memory */
	ab_sm_ctx = devm_kzalloc(dev, sizeof(struct ab_state_context),
							GFP_KERNEL);
	ab_sm_ctx->pdev = pdev;
	ab_sm_ctx->dev = &pdev->dev;

	/* When this "host-gpio-ctrl" property is set, host has control over the
	 * below gpios and can be controlled at kernel level. If this property
	 * is not set, all the gpios and their related functionality has to be
	 * triggered from application level (ex Juno Host)
	 */
	if (!of_property_read_u32(np, "host-gpio-ctrl", &host_gpio_ctrl)) {

		/* Get the gpio_desc for all the gpios used */
		/* SOC_PWRGOOD releases M0PLUS when its low */
		ab_sm_ctx->soc_pwrgood = devm_gpiod_get(&pdev->dev, "soc-pwrgood",
						GPIOD_OUT_LOW);
		if (IS_ERR(ab_sm_ctx->soc_pwrgood)) {
			dev_err(dev, "%s: could not get soc-pwrgood gpio (%ld)\n",
				__func__, PTR_ERR(ab_sm_ctx->soc_pwrgood));
			error = PTR_ERR(ab_sm_ctx->soc_pwrgood);
			goto fail_soc_pwrgood;
		}

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

        gpiod_set_value(ab_sm_ctx->fw_patch_en, __GPIO_DISABLE);
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
	}

	/* Intialize the default state of each block for state manager */
	ab_sm_ctx->blocks[BLK_IPU] = (struct block){"BLK_IPU",
			&ipu_property_table[8], ipu_property_table,
			ARRAY_SIZE(ipu_property_table)};
	ab_sm_ctx->blocks[BLK_TPU] = (struct block){"BLK_TPU",
			&tpu_property_table[8], tpu_property_table,
			ARRAY_SIZE(tpu_property_table)};
	ab_sm_ctx->blocks[DRAM] = (struct block){"DRAM",
			&dram_property_table[10], dram_property_table,
			ARRAY_SIZE(dram_property_table)};
	ab_sm_ctx->blocks[BLK_MIF] = (struct block){"BLK_MIF",
			&mif_property_table[7], mif_property_table,
			ARRAY_SIZE(mif_property_table)};
	ab_sm_ctx->blocks[BLK_FSYS] = (struct block){"BLK_FSYS",
			&fsys_property_table[8], fsys_property_table,
			ARRAY_SIZE(fsys_property_table)};
	ab_sm_ctx->blocks[BLK_AON] = (struct block){"BLK_AON",
			&aon_property_table[3], aon_property_table,
			ARRAY_SIZE(aon_property_table)};

	/* intitialize the default chip state */
	ab_sm_ctx->chip_state_table = chip_state_map;
	ab_sm_ctx->nr_chip_states = ARRAY_SIZE(chip_state_map);
	ab_sm_ctx->sw_state_id = S0;
	ab_sm_ctx->sw_state_name = "Active";
	ab_sm_ctx->chip_substate_id = 0;
	ab_sm_ctx->chip_substate_name = "Ready";

	/* Registering CMUs to Common Clock Framework */
	abc_clk_register(ab_sm_ctx);

	return ab_sm_ctx;

fail_soc_pwrgood:
fail_fw_patch_en:
fail_ab_ready:
	devm_kfree(dev, (void *)ab_sm_ctx);
	ab_sm_ctx = NULL;

	return NULL;
}
EXPORT_SYMBOL(ab_sm_init);


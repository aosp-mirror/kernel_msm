/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Shaik Ameer Basha <shaik.ameer@samsung.com>
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *
 * Airbrush State Manager Control driver..
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-clk.h>
#include <linux/airbrush-sm-ctrl.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/kernel.h>

#include "airbrush-spi.h"
#include "airbrush-pmu.h"
#include "airbrush-power-gating.h"
#include "airbrush-pmic-ctrl.h"

static struct ab_state_context *ab_sm_ctx;

static struct block_property ipu_property_table[] = {
	{BLOCK_STATE_0_0,	"Normal",	"Ready",	on,	VOLTAGE_0_75,	off,	550000000,	14,	0,	0,	0},
	{BLOCK_STATE_0_1,	"Normal",	"AonCompute",	on,	VOLTAGE_0_75,	on,	50000000,	2,	2,	0,	0},
	{BLOCK_STATE_0_2,	"Normal",	"MinCompute",	on,	VOLTAGE_0_75,	on,	220000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_3,	"Normal",	"LowCompute",	on,	VOLTAGE_0_75,	on,	330000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_4,	"Normal",	"MidCompute",	on,	VOLTAGE_0_75,	on,	440000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_5,	"Normal",	"MaxCompute",	on,	VOLTAGE_0_75,	on,	550000000,	14,	14,	0,	0},
	{BLOCK_STATE_0_6,	"Boost",	"MaxCompute",	on,	VOLTAGE_0_85,	on,	610000000,	14,	14,	0,	0},
	{BLOCK_STATE_1_0,	"Normal",	"PowerGated",	on,	VOLTAGE_0_75,	off,	550000000,	0,	0,	0,	0},
	{BLOCK_STATE_1_1,	"Boost",	"PowerGated",	on,	VOLTAGE_0_85,	off,	610000000,	0,	0,	0,	0},
	{
		BLOCK_STATE_1_2, "Normal", "Sleep", on, VOLTAGE_0_75, off, 0, 0,
		0, 0, 0
	},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",	off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{
		BLOCK_STATE_DEFAULT, "BootupState", "NoClock", on, VOLTAGE_0_75,
		off, 0, 0, 0, 0, 0
	}
};

static struct block_property tpu_property_table[] = {
	{BLOCK_STATE_0_0,	"Normal",	"Ready",	on,	VOLTAGE_0_75,	off,	766000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_1,	"Normal",	"AonCompute",	on,	VOLTAGE_0_75,	on,	50000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_2,	"Normal",	"MinCompute",	on,	VOLTAGE_0_75,	on,	307000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_3,	"Normal",	"LowCompute",	on,	VOLTAGE_0_75,	on,	460000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_4,	"Normal",	"MidCompute",	on,	VOLTAGE_0_75,	on,	613000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_5,	"Normal",	"MaxCompute",	on,	VOLTAGE_0_75,	on,	766000000,	0,	0,	16,	0},
	{BLOCK_STATE_0_6,	"Boost",	"MaxCompute",	on,	VOLTAGE_0_85,	on,	962000000,	0,	0,	16,	0},
	{BLOCK_STATE_1_0,	"Normal",	"PowerGated",	on,	VOLTAGE_0_75,	off,	766000000,	0,	0,	0,	0},
	{BLOCK_STATE_1_1,	"Boost",	"PowerGated",	on,	VOLTAGE_0_85,	off,	962000000,	0,	0,	0,	0},
	{
		BLOCK_STATE_1_2, "Normal", "Sleep", on, VOLTAGE_0_75, off, 0, 0,
		0, 0, 0
	},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",	off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{
		BLOCK_STATE_DEFAULT, "BootupState", "NoClock", on, VOLTAGE_0_75,
		off, 0, 0, 0, 0, 0
	}
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
	{BLOCK_STATE_2_0,	"Retention",	"SelfRefresh",		off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",		off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{
		BLOCK_STATE_DEFAULT, "BootupState", "MaxTransact", on,
		VOLTAGE_0_60, on, 1867000000, 0, 0, 0, 3733
	}
};

static struct block_property mif_property_table[] = {
	{BLOCK_STATE_0_0,	"Normal",	"Ready",		on,	VOLTAGE_0_85,	off,	933000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_1,	"Normal",	"AonTransact",		on,	VOLTAGE_0_85,	on,	50000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_2,	"Normal",	"HalfMidTransact",	on,	VOLTAGE_0_85,	on,	200000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_3,	"Normal",	"HalfMaxTransact",	on,	VOLTAGE_0_85,	on,	233000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_4,	"Normal",	"LowTransact",		on,	VOLTAGE_0_85,	on,	300000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_5,	"Normal",	"MidTransact",		on,	VOLTAGE_0_85,	on,	400000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_6,	"Normal",	"MaxTransact",		on,	VOLTAGE_0_85,	on,	467000000,	0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",		off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{
		BLOCK_STATE_DEFAULT, "BootupState", "MaxTransact", on,
		VOLTAGE_0_85, on, 467000000, 0, 0, 0, 0
	}
};

static struct block_property fsys_property_table[] = {
	{BLOCK_STATE_0_0,	"ElectricalIdle",	"L0s",		on,	VOLTAGE_0_85,	off,	4000000000,	0,	0,	0,	3},
	{BLOCK_STATE_0_1,	"PowerUp",		"L0",		on,	VOLTAGE_0_85,	on,	1250000000,	0,	0,	0,	1}, /*GEN1L0*/
	{BLOCK_STATE_0_2,	"PowerUp",		"L0",		on,	VOLTAGE_0_85,	on,	2500000000,	0,	0,	0,	2}, /*GEN2L0*/
	{BLOCK_STATE_0_3,	"PowerUp",		"L0",		on,	VOLTAGE_0_85,	on,	4000000000,	0,	0,	0,	3}, /*GEN3L0*/
	{BLOCK_STATE_1_0,	"ElectricalIdle",	"L1",		on,	VOLTAGE_0_85,	on,	4000000000,	0,	0,	0,	0},
	{BLOCK_STATE_1_1,	"ElectricalIdle",	"L1.1",		on,	VOLTAGE_0_85,	on,	0,		0,	0,	0,	0},
	{BLOCK_STATE_1_2,	"ElectricalIdle",	"L1.2",		on,	VOLTAGE_0_85,	on,	0,		0,	0,	0,	0},
	{BLOCK_STATE_2_0,	"Hibernate",		"L2",		on,	VOLTAGE_0_85,	on,	0,		0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",		"L3",		off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{
		BLOCK_STATE_DEFAULT, "BootupState", "L0", on, VOLTAGE_0_85, on,
		4000000000, 0, 0, 0, 3
	} /*GEN3L0*/
};

static struct block_property aon_property_table[] = {
	{BLOCK_STATE_0_0,	"PowerUp",	"WFI",		on,	VOLTAGE_0_85,	off,	233000000,	0,	0,	0,	0},
	{BLOCK_STATE_0_1,	"PowerUp",	"Boot",		on,	VOLTAGE_0_85,	on,	19200000,	0,	0,	0,	0},
	{BLOCK_STATE_0_2,	"PowerUp",	"Compute",	on,	VOLTAGE_0_85,	on,	233000000,	0,	0,	0,	0},
	{BLOCK_STATE_3_0,	"Disabled",	"NoRail",	off,	VOLTAGE_0_0,	off,	0,		0,	0,	0,	0},
	{
		BLOCK_STATE_DEFAULT, "BootupState", "Compute", on, VOLTAGE_0_85,
		on, 233000000, 0, 0, 0, 0
	}
};

#define CHIP_TO_BLOCK_MAP_INIT(cs, ipu, tpu, dram, mif, fsys, aon, core) \
	{								\
		CHIP_STATE_ ## cs,					\
		BLOCK_STATE_ ## ipu,					\
		BLOCK_STATE_ ## tpu,					\
		BLOCK_STATE_ ## dram,					\
		BLOCK_STATE_ ## mif,					\
		BLOCK_STATE_ ## fsys,					\
		BLOCK_STATE_ ## aon,					\
		core ## _POWER_CONTROL,					\
	}

static struct chip_to_block_map chip_state_map[] = {
	/*                     CS   IPU  TPU DRAM  MIF FSYS  AON  PC */
	CHIP_TO_BLOCK_MAP_INIT(0_0, 0_0, 0_0, 0_0, 0_0, 0_0, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_1, 0_1, 0_1, 0_1, 0_1, 0_1, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_2, 0_2, 0_2, 0_2, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_3, 0_3, 0_3, 0_4, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_4, 0_4, 0_4, 0_5, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_5, 0_5, 0_2, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_6, 0_2, 0_5, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_7, 0_5, 0_3, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_8, 0_3, 0_5, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(0_9, 0_5, 0_5, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(1_0, 0_0, 1_0, 0_0, 0_0, 0_0, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(1_1, 0_1, 1_0, 0_1, 0_1, 0_1, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(1_2, 0_2, 1_0, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(1_3, 0_3, 1_0, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(1_4, 0_4, 1_0, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(1_5, 0_5, 1_0, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(1_6, 0_6, 1_1, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(2_0, 1_0, 0_0, 0_0, 0_0, 0_0, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(2_1, 1_0, 0_1, 0_6, 0_1, 0_1, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(2_2, 1_0, 0_2, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(2_3, 1_0, 0_3, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(2_4, 1_0, 0_4, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(2_5, 1_0, 0_5, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(2_6, 1_1, 0_6, 0_6, 0_6, 0_3, 0_0, IPU),
	CHIP_TO_BLOCK_MAP_INIT(3_0, 1_2, 1_2, 2_0, 0_0, 1_2, 0_0, TPU),
	CHIP_TO_BLOCK_MAP_INIT(4_0, 3_0, 3_0, 2_0, 0_0, 1_2, 0_0, TPU),
	CHIP_TO_BLOCK_MAP_INIT(5_0, 3_0, 3_0, 2_0, 3_0, 3_0, 3_0, TPU),
	CHIP_TO_BLOCK_MAP_INIT(6_0, 3_0, 3_0, 3_0, 3_0, 3_0, 3_0, TPU),
	CHIP_TO_BLOCK_MAP_INIT(DEFAULT, DEFAULT, DEFAULT, DEFAULT, DEFAULT,
			       DEFAULT, DEFAULT, IPU),
};

struct block_property *get_desired_state(struct block *blk,
					 u32 to_block_state_id)
{
	int i;

	for (i = 0; i < (blk->nr_block_states); i++) {
		if (blk->block_property_table[i].id == to_block_state_id)
			return &(blk->block_property_table[i]);
	}
	return NULL;
}

void ab_sm_register_blk_callback(block_name_t name,
		ab_sm_set_block_state_t callback, void *data)
{
	ab_sm_ctx->blocks[name].set_state = callback;
	ab_sm_ctx->blocks[name].data = data;
}

int clk_set_frequency(struct device *dev, struct block *blk,
			 u64 frequency, enum states clk_status)
{
	switch (blk->name) {
	case BLK_IPU:
		if (blk->current_state->clk_frequency == 0 && frequency != 0)
			ipu_pll_enable(dev);
		if (blk->current_state->clk_status == off && clk_status == on)
			ipu_ungate(dev);
		if (blk->current_state->clk_frequency == 0 && !frequency)
			break;

		ipu_set_rate(dev, frequency);

		if (blk->current_state->clk_status == on && clk_status == off)
			ipu_gate(dev);
		if (!clk_status && !frequency)
			ipu_pll_disable(dev);
		break;
	case BLK_TPU:
		if (blk->current_state->clk_frequency == 0 && frequency != 0)
			tpu_pll_enable(dev);
		if (blk->current_state->clk_status == off && clk_status == on)
			tpu_ungate(dev);
		if (blk->current_state->clk_frequency == 0 && !frequency)
			break;

		tpu_set_rate(dev, frequency);

		if (blk->current_state->clk_status == on && clk_status == off)
			tpu_gate(dev);
		if (!clk_status && !frequency)
			tpu_pll_disable(dev);
		break;
	case BLK_MIF:
	case BLK_FSYS:
	case BLK_AON:
	case DRAM:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int blk_set_state(struct ab_state_context *sc, struct block *blk,
	u32 to_block_state_id, bool power_control, u32 to_chip_substate_id)
{
	struct device *dev = sc->dev;
	bool power_increasing;
	struct block_property *desired_state =
		get_desired_state(blk, to_block_state_id);
	if (!desired_state)
		return -EINVAL;

	if (blk->current_state->id == desired_state->id)
		return 0;

	power_increasing = (blk->current_state->logic_voltage
				< desired_state->logic_voltage);

	/* Regulator Settings */
	if (power_control && power_increasing) {
		if (blk->current_state->voltage_rail_status == off)
			ab_blk_pw_rails_enable(sc, blk->name,
			to_chip_substate_id);
		/*TODO: change regulator voltage*/
	}

	/* PMU settings */
	if (power_control && blk->name == BLK_IPU) {
		if (desired_state->id != BLOCK_STATE_1_2
				&& desired_state->id != BLOCK_STATE_3_0
				&& blk->current_state->id >= BLOCK_STATE_1_2) {
			if (ab_pmu_resume())
				return -EAGAIN;
			if (sc->chip_substate_id == CHIP_STATE_4_0)
				abc_ipu_tpu_enable();
		}
	}

	clk_set_frequency(dev, blk, desired_state->clk_frequency,
			desired_state->clk_status);

	/* Block specific hooks */
	if (blk->set_state)
		blk->set_state(blk->current_state, desired_state,
			       to_chip_substate_id, blk->data);

	/* PMU settings */
	if (power_control && blk->name == BLK_TPU) {
		if (desired_state->id == BLOCK_STATE_1_2 && ab_pmu_sleep())
			return -EAGAIN;
		if (desired_state->id == BLOCK_STATE_3_0 && ab_pmu_deep_sleep())
			return -EAGAIN;
	}

	/*Regulator Settings*/
	if (power_control && !power_increasing) {
		/*TODO: change regulator voltage*/
		if (desired_state->voltage_rail_status == off)
			ab_blk_pw_rails_disable(sc, blk->name,
			to_chip_substate_id);
	}

	blk->current_state = desired_state;

	return 0;
}

static bool is_valid_transition(u32 curr_chip_substate_id,
				u32 to_chip_substate_id)
{
	switch (curr_chip_substate_id) {
	case CHIP_STATE_4_0:
		if (to_chip_substate_id == CHIP_STATE_3_0)
			return false;
		break;
	case CHIP_STATE_5_0:
		if (to_chip_substate_id == CHIP_STATE_4_0)
			return false;
		if (to_chip_substate_id == CHIP_STATE_3_0)
			return false;
		break;
	case CHIP_STATE_6_0:
		if (to_chip_substate_id == CHIP_STATE_5_0)
			return false;
		if (to_chip_substate_id == CHIP_STATE_4_0)
			return false;
		if (to_chip_substate_id == CHIP_STATE_3_0)
			return false;
		break;
	}
	return true;
}

int ab_sm_set_state(struct ab_state_context *sc, u32 to_chip_substate_id)
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

	if (!is_valid_transition(sc->chip_substate_id, to_chip_substate_id)) {
		dev_err(sc->dev,
				"%s: invalid state change, current %u, requested %u\n",
				__func__, sc->chip_substate_id,
				to_chip_substate_id);
		return -EINVAL;
	}

	if ((sc->chip_substate_id == CHIP_STATE_6_0 ||
	   sc->chip_substate_id == CHIP_STATE_5_0) &&
	   to_chip_substate_id < CHIP_STATE_3_0) {
		if (sc->ab_alternate_boot)
			ab_bootsequence(sc, 1);
		else
			ab_bootsequence(sc, 0);
	}

	if (sc->chip_substate_id == to_chip_substate_id)
		return 0;

	if (blk_set_state(sc, &(sc->blocks[BLK_IPU]),
		      map->ipu_block_state_id, (map->flags & IPU_POWER_CONTROL),
			to_chip_substate_id))
		return -EINVAL;

	if (blk_set_state(sc, &(sc->blocks[BLK_TPU]),
		      map->tpu_block_state_id, (map->flags & TPU_POWER_CONTROL),
			to_chip_substate_id))
		return -EINVAL;

	if (blk_set_state(sc, &(sc->blocks[DRAM]),
		      map->dram_block_state_id, true, to_chip_substate_id))
		return -EINVAL;

	if (blk_set_state(sc, &(sc->blocks[BLK_MIF]),
		      map->mif_block_state_id, true, to_chip_substate_id))
		return -EINVAL;

	if (blk_set_state(sc, &(sc->blocks[BLK_FSYS]),
		      map->fsys_block_state_id, true, to_chip_substate_id))
		return -EINVAL;

	if (blk_set_state(sc, &(sc->blocks[BLK_AON]),
		      map->aon_block_state_id, true, to_chip_substate_id))
		return -EINVAL;

	if (to_chip_substate_id == CHIP_STATE_6_0) {
		ab_pmic_off(sc);
		ab_disable_pgood(sc);
	}
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

void ab_enable_pgood(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->soc_pwrgood, __GPIO_ENABLE);
}

void ab_disable_pgood(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->soc_pwrgood, __GPIO_DISABLE);
}

struct ab_state_context *ab_sm_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int error;
	u32 boot_time_block_state;

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

	ab_sm_ctx->ab_sm_ctrl_pmic = true;

	/* [TBD] Need DDR_SR, DDR_TRAIN, CKE_IN, CKE_IN_SENSE GPIOs for  */

	/* Intialize the default state of each block for state manager */
	boot_time_block_state = ARRAY_SIZE(ipu_property_table)-1;
	ab_sm_ctx->blocks[BLK_IPU] = (struct block){BLK_IPU,
			&ipu_property_table[boot_time_block_state], ipu_property_table,
			ARRAY_SIZE(ipu_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(tpu_property_table)-1;
	ab_sm_ctx->blocks[BLK_TPU] = (struct block){BLK_TPU,
			&tpu_property_table[boot_time_block_state], tpu_property_table,
			ARRAY_SIZE(tpu_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(dram_property_table)-1;
	ab_sm_ctx->blocks[DRAM] = (struct block){DRAM,
			&dram_property_table[boot_time_block_state], dram_property_table,
			ARRAY_SIZE(dram_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(mif_property_table)-1;
	ab_sm_ctx->blocks[BLK_MIF] = (struct block){BLK_MIF,
			&mif_property_table[boot_time_block_state], mif_property_table,
			ARRAY_SIZE(mif_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(fsys_property_table)-1;
	ab_sm_ctx->blocks[BLK_FSYS] = (struct block){BLK_FSYS,
			&fsys_property_table[boot_time_block_state], fsys_property_table,
			ARRAY_SIZE(fsys_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(aon_property_table)-1;
	ab_sm_ctx->blocks[BLK_AON] = (struct block){BLK_AON,
			&aon_property_table[boot_time_block_state], aon_property_table,
			ARRAY_SIZE(aon_property_table), NULL, NULL};

	/* intitialize the default chip state */
	ab_sm_ctx->chip_state_table = chip_state_map;
	ab_sm_ctx->nr_chip_states = ARRAY_SIZE(chip_state_map);
	ab_sm_ctx->chip_substate_id = CHIP_STATE_DEFAULT;

	ab_sm_create_debugfs(ab_sm_ctx);
	return ab_sm_ctx;

fail_fw_patch_en:
fail_ab_ready:
	devm_kfree(dev, (void *)ab_sm_ctx);
	ab_sm_ctx = NULL;

	return NULL;
}
EXPORT_SYMBOL(ab_sm_init);

void ab_sm_exit(struct platform_device *pdev)
{
	ab_sm_remove_debugfs(ab_sm_ctx);
}

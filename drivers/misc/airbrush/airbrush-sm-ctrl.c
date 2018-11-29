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

#include <linux/airbrush-sm-ctrl.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/kernel.h>
#include <linux/msm_pcie.h>
#include <uapi/ab-sm.h>

#include "airbrush-cooling.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-pmu.h"
#include "airbrush-power-gating.h"
#include "airbrush-regs.h"
#include "airbrush-spi.h"
#include "airbrush-thermal.h"

#define to_chip_substate_category(chip_substate_id) ((chip_substate_id) / 10)

static struct ab_state_context *ab_sm_ctx;

#define BLK_ENTRY(num, state, sub, rail, v, clk, freq, pwr, used, tiles, dr) \
	{\
		BLOCK_STATE_ ## num, \
		#state, \
		#sub, \
		rail, \
		VOLTAGE_ ## v, \
		clk, \
		(u32)(1000000. * freq), \
		pwr, \
		used, \
		tiles, \
		dr,\
	}

static struct block_property ipu_property_table[] = {
	BLK_ENTRY(0_0, Normal,   Ready,      on, 0_75, off, 549.6, 14, 0, 0, 0),
	BLK_ENTRY(0_1, Normal,   AonCompute, on, 0_75, on,  50,   2,  2,  0, 0),
	BLK_ENTRY(0_2, Normal,   MinCompute, on, 0_75, on,  220,  14, 14, 0, 0),
	BLK_ENTRY(0_3, Normal,   LowCompute, on, 0_75, on,  330,  14, 14, 0, 0),
	BLK_ENTRY(0_4, Normal,   MidCompute, on, 0_75, on,  440,  14, 14, 0, 0),
	BLK_ENTRY(0_5, Normal,   MaxCompute, on, 0_75, on, 549.6, 14, 14, 0, 0),
	BLK_ENTRY(0_6, Boost,    MaxCompute, on, 0_75, on, 577.6, 14, 14, 0, 0),
	BLK_ENTRY(1_0, Normal,   PowerGated, on, 0_75, off, 0,    0,  0,  0, 0),
	BLK_ENTRY(1_1, Boost,    PowerGated, on, 0_85, off, 0,    0,  0,  0, 0),
	BLK_ENTRY(1_2, Normal,   Sleep,      on, 0_75, off, 0,    0,  0,  0, 0),
	BLK_ENTRY(3_0, Disabled, NoRail,     off, 0_0, off, 0,    0,  0,  0, 0),
};

static struct block_property tpu_property_table[] = {
	BLK_ENTRY(0_0, Normal,   Ready,      on, 0_75, off, 765.6, 0, 0, 16, 0),
	BLK_ENTRY(0_1, Normal,   AonCompute, on, 0_75, on,  50,    0, 0, 16, 0),
	BLK_ENTRY(0_2, Normal,   MinCompute, on, 0_75, on,  303,   0, 0, 16, 0),
	BLK_ENTRY(0_3, Normal,   LowCompute, on, 0_75, on,  453.6, 0, 0, 16, 0),
	BLK_ENTRY(0_4, Normal,   MidCompute, on, 0_75, on,  606,   0, 0, 16, 0),
	BLK_ENTRY(0_5, Normal,   MaxCompute, on, 0_75, on,  765.6, 0, 0, 16, 0),
	BLK_ENTRY(0_6, Boost,    MaxCompute, on, 0_85, on,  961.6, 0, 0, 16, 0),
	BLK_ENTRY(1_0, Normal,   PowerGated, on, 0_75, off, 0,     0, 0, 0,  0),
	BLK_ENTRY(1_1, Boost,    PowerGated, on, 0_85, off, 0,     0, 0, 0,  0),
	BLK_ENTRY(1_2, Normal,   Sleep,      on, 0_75, off, 0,     0, 0, 0,  0),
	BLK_ENTRY(3_0, Disabled, NoRail,     off, 0_0, off, 0,     0, 0, 0,  0),
};

static struct block_property dram_property_table[] = {
	BLK_ENTRY(0_0, PowerUp,   Standby,  on, 0_60, off, 1867, 0, 0, 0, 3733),
	BLK_ENTRY(0_1, PowerUp,   AonTran,   on, 0_60, on,  800, 0, 0, 0, 1600),
	BLK_ENTRY(0_2, PowerUp, HalfMidTran, on, 0_60, on, 800,  0, 0, 0, 1600),
	BLK_ENTRY(0_3, PowerUp, HalfMaxTran, on, 0_60, on, 934,  0, 0, 0, 1867),
	BLK_ENTRY(0_4, PowerUp,   LowTran,   on, 0_60, on, 1200, 0, 0, 0, 2400),
	BLK_ENTRY(0_5, PowerUp,   MidTran,   on, 0_60, on, 1600, 0, 0, 0, 3200),
	BLK_ENTRY(0_6, PowerUp,   MaxTran,   on, 0_60, on, 1867, 0, 0, 0, 3733),
	BLK_ENTRY(1_0, PowerDown, ClockOff, on, 0_60, off, 1867, 0, 0, 0, 3733),
	BLK_ENTRY(1_1, PowerDown, ClockOn,   on, 0_60, on, 1867, 0, 0, 0, 3733),
	BLK_ENTRY(2_0, Retention, SelfRef,   off, 0_0, off, 0,   0, 0, 0, 0),
	BLK_ENTRY(3_0, Disabled,  NoRail,    off, 0_0, off, 0,   0, 0, 0, 0),
};

static struct block_property mif_property_table[] = {
	BLK_ENTRY(0_0, Normal,   Ready,       on, 0_85, off, 933, 0, 0, 0, 0),
	BLK_ENTRY(0_1, Normal,   AonTran,     on, 0_85, on,  200, 0, 0, 0, 0),
	BLK_ENTRY(0_2, Normal,   HalfMidTran, on, 0_85, on,  200, 0, 0, 0, 0),
	BLK_ENTRY(0_3, Normal,   HalfMaxTran, on, 0_85, on,  233, 0, 0, 0, 0),
	BLK_ENTRY(0_4, Normal,   LowTran,     on, 0_85, on,  300, 0, 0, 0, 0),
	BLK_ENTRY(0_5, Normal,   MidTran,     on, 0_85, on,  400, 0, 0, 0, 0),
	BLK_ENTRY(0_6, Normal,   MaxTran,     on, 0_85, on,  467, 0, 0, 0, 0),
	BLK_ENTRY(3_0, Disabled, NoRail,      off, 0_0, off, 0,   0, 0, 0, 0),
};

static struct block_property fsys_property_table[] = {
	BLK_ENTRY(0_0, ElectricalIdle, L0s, on,  0_85, off, 4000,  0, 0, 0, 3),
	/* GEN1L0 */
	BLK_ENTRY(0_1, PowerUp,        L0,  on,  0_85, on,  1250,  0, 0, 0, 1),
	/* GEN2L0 */
	BLK_ENTRY(0_2, PowerUp,        L0,  on,  0_85, on,  2500,  0, 0, 0, 2),
	/* GEN3L0 */
	BLK_ENTRY(0_3, PowerUp,        L0,  on,  0_85, on,  4000,  0, 0, 0, 3),
	BLK_ENTRY(1_0, ElectricalIdle, L1,  on,  0_85, on,  4000,  0, 0, 0, 0),
	BLK_ENTRY(1_1, ElectricalIdle, L1.1, on, 0_85, on,  0,     0, 0, 0, 0),
	BLK_ENTRY(1_2, ElectricalIdle, L1.2, on, 0_85, on,  0,     0, 0, 0, 0),
	BLK_ENTRY(2_0, Hibernate,      L2,  on,  0_85, on,  0,     0, 0, 0, 0),
	BLK_ENTRY(3_0, Disabled,       L3,  off, 0_0,  off, 0,     0, 0, 0, 0),
};

static struct block_property aon_property_table[] = {
	BLK_ENTRY(0_0, PowerUp,  WFI,     on,  0_85, off, 933.12, 0, 0, 0, 0),
	BLK_ENTRY(0_1, PowerUp,  Boot,    on,  0_85, on,  19.2,   0, 0, 0, 0),
	BLK_ENTRY(0_2, PowerUp,  Compute, on,  0_85, on,  933.12, 0, 0, 0, 0),
	BLK_ENTRY(3_0, Disabled, NoRail,  off, 0_0,  off, 0,      0, 0, 0, 0),
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
	CHIP_TO_BLOCK_MAP_INIT(3_0, 1_2, 1_2, 2_0, 0_0, 1_2, 0_1, TPU),
	CHIP_TO_BLOCK_MAP_INIT(4_0, 3_0, 3_0, 2_0, 0_0, 1_2, 0_1, TPU),
	CHIP_TO_BLOCK_MAP_INIT(5_0, 3_0, 3_0, 2_0, 3_0, 3_0, 3_0, TPU),
	CHIP_TO_BLOCK_MAP_INIT(6_0, 3_0, 3_0, 3_0, 3_0, 3_0, 3_0, TPU),
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

void ab_sm_register_blk_callback(enum block_name name,
		ab_sm_set_block_state_t callback, void *data)
{
	ab_sm_ctx->blocks[name].set_state = callback;
	ab_sm_ctx->blocks[name].data = data;
}

/* Caller must hold sc->op_lock */
int clk_set_frequency(struct ab_state_context *sc, struct block *blk,
			 u64 frequency, enum states clk_status)
{
	int ret = 0;
	struct ab_sm_clk_ops *clk = sc->clk_ops;

	switch (blk->name) {
	case BLK_IPU:
		if (blk->current_state->clk_frequency == 0 && frequency != 0)
			ret = clk->ipu_pll_enable(clk->ctx);
		if (blk->current_state->clk_status == off && clk_status == on)
			ret = clk->ipu_ungate(clk->ctx);
		if (blk->current_state->clk_frequency == 0 && !frequency)
			break;

		clk->ipu_set_rate(clk->ctx, frequency);

		if (blk->current_state->clk_status == on && clk_status == off)
			ret = clk->ipu_gate(clk->ctx);
		if (!clk_status && !frequency)
			ret = clk->ipu_pll_disable(clk->ctx);
		break;
	case BLK_TPU:
		if (blk->current_state->clk_frequency == 0 && frequency != 0)
			ret = clk->tpu_pll_enable(clk->ctx);
		if (blk->current_state->clk_status == off && clk_status == on)
			ret = clk->tpu_ungate(clk->ctx);
		if (blk->current_state->clk_frequency == 0 && !frequency)
			break;

		clk->tpu_set_rate(clk->ctx, frequency);

		if (blk->current_state->clk_status == on && clk_status == off)
			ret = clk->tpu_gate(clk->ctx);
		if (!clk_status && !frequency)
			ret = clk->tpu_pll_disable(clk->ctx);
		break;
	case BLK_MIF:
		break;
	case BLK_FSYS:
		break;
	case BLK_AON:
		if (blk->current_state->clk_frequency == 0 && !frequency)
			break;
		clk->aon_set_rate(clk->ctx, frequency);
		break;
	case DRAM:
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

int blk_set_state(struct ab_state_context *sc, struct block *blk,
	u32 to_block_state_id, bool power_control, u32 to_chip_substate_id)
{
	struct ab_sm_pmu_ops *pmu;
	bool power_increasing;
	struct block_property *desired_state =
		get_desired_state(blk, to_block_state_id);
	if (!desired_state)
		return -EINVAL;

	if (blk->current_state->id == desired_state->id)
		return 0;

	power_increasing = (blk->current_state->logic_voltage
				< desired_state->logic_voltage);

	mutex_lock(&sc->op_lock);
	pmu = sc->pmu_ops;
	/* PMU settings */
	if (power_control && blk->name == BLK_IPU) {
		if (desired_state->id != BLOCK_STATE_1_2
				&& desired_state->id != BLOCK_STATE_3_0
				&& blk->current_state->id >= BLOCK_STATE_1_2) {
			if (pmu->pmu_resume(pmu->ctx)) {
				mutex_unlock(&sc->op_lock);
				return -EAGAIN;
			}
		}
	}

	clk_set_frequency(sc, blk, desired_state->clk_frequency,
			desired_state->clk_status);

	/* Block specific hooks */
	if (blk->set_state)
		blk->set_state(blk->current_state, desired_state,
				   to_chip_substate_id, blk->data);

	/* PMU settings */
	if (power_control && blk->name == BLK_TPU) {
		if (desired_state->id == BLOCK_STATE_1_2 &&
				pmu->pmu_sleep(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
		if (desired_state->id == BLOCK_STATE_3_0 &&
				pmu->pmu_deep_sleep(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
	}

	/*Regulator Settings*/
	if (power_control && !power_increasing) {
		/*TODO: change regulator voltage*/
		if (desired_state->voltage_rail_status == off)
			ab_blk_pw_rails_disable(sc, blk->name,
			to_chip_substate_id);
	}

	blk->current_state = desired_state;

	mutex_unlock(&sc->op_lock);
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

static int disable_ref_clk(struct device *dev)
{
	struct clk *ref_clk = clk_get(dev, "ab_ref");

	if (!IS_ERR(ref_clk)) {
		clk_disable_unprepare(ref_clk);
		return 0;
	} else
		return PTR_ERR(ref_clk);
}

static const u32 chip_substate_throttler_map
		[][AIRBRUSH_COOLING_STATE_MAX + 1] = {
	{CHIP_STATE_0_9, CHIP_STATE_0_4, CHIP_STATE_0_3, CHIP_STATE_0_2},
	{CHIP_STATE_1_6, CHIP_STATE_1_4, CHIP_STATE_1_3, CHIP_STATE_1_2},
	{CHIP_STATE_2_6, CHIP_STATE_2_4, CHIP_STATE_2_3, CHIP_STATE_2_2},
};

static u32 ab_sm_throttled_chip_substate_id(
		u32 chip_substate_id, enum throttle_state throttle_state_id)
{
	u32 substate_category;
	u32 throttler_substate_id;

	if (chip_substate_id >= CHIP_STATE_3_0)
		return chip_substate_id;

	substate_category = to_chip_substate_category(chip_substate_id);
	throttler_substate_id = chip_substate_throttler_map
			[substate_category][throttle_state_id];
	return min(chip_substate_id, throttler_substate_id);
}

static const enum stat_state chip_state_to_stat_state(
		enum chip_state chip_state_id)
{
	if ((chip_state_id >= CHIP_STATE_0_0) &&
			(chip_state_id < CHIP_STATE_3_0)) {
		return STAT_STATE_ACTIVE;
	}
	switch (chip_state_id) {
	case CHIP_STATE_3_0:
		return STAT_STATE_SLEEP;
	case CHIP_STATE_4_0:
		return STAT_STATE_DEEP_SLEEP;
	case CHIP_STATE_5_0:
		return STAT_STATE_SUSPEND;
	case CHIP_STATE_6_0:
		return STAT_STATE_OFF;
	default:
		/* should never hit this code path */
		return STAT_STATE_UNKNOWN;
	}
}

/* Caller must hold sc->state_lock */
static void ab_sm_record_state_change(enum chip_state prev_state,
		enum chip_state new_state,
		struct ab_state_context *sc)
{
	enum stat_state prev_stat_state = chip_state_to_stat_state(prev_state);
	enum stat_state new_stat_state = chip_state_to_stat_state(new_state);
	ktime_t time, time_diff;

	if (new_stat_state == prev_stat_state)
		return;

	time = ktime_get_boottime();
	sc->state_stats[new_stat_state].counter++;
	sc->state_stats[new_stat_state].last_entry = time;
	sc->state_stats[prev_stat_state].last_exit = time;
	time_diff = ktime_sub(sc->state_stats[prev_stat_state].last_exit,
			sc->state_stats[prev_stat_state].last_entry);
	sc->state_stats[prev_stat_state].duration = ktime_add(
			sc->state_stats[prev_stat_state].duration, time_diff);
}

/* Caller must hold sc->state_lock */
static int ab_sm_update_chip_state(struct ab_state_context *sc)
{
	u32 to_chip_substate_id;
	int i;
	int ret;
	struct chip_to_block_map *map = NULL;
	enum chip_state prev_state = sc->curr_chip_substate_id;

	to_chip_substate_id = ab_sm_throttled_chip_substate_id(
			sc->dest_chip_substate_id,
			sc->throttle_state_id);

	if (sc->curr_chip_substate_id == to_chip_substate_id)
		return 0;

	for (i = 0; i < sc->nr_chip_states; i++) {
		if (sc->chip_state_table[i].chip_substate_id ==
				to_chip_substate_id) {
			map = &(sc->chip_state_table[i]);
			break;
		}
	}

	if (!map)
		return -EINVAL;

	dev_info(sc->dev, "AB state changing to %d\n", to_chip_substate_id);

	if ((sc->curr_chip_substate_id == CHIP_STATE_6_0 ||
	   sc->curr_chip_substate_id == CHIP_STATE_5_0) &&
	   to_chip_substate_id < CHIP_STATE_3_0)
		ab_bootsequence(sc);

	if ((sc->curr_chip_substate_id == CHIP_STATE_4_0 ||
	   sc->curr_chip_substate_id == CHIP_STATE_3_0) &&
	   to_chip_substate_id < CHIP_STATE_3_0)
		ab_pmic_on(sc);

	if ((sc->curr_chip_substate_id == CHIP_STATE_5_0 ||
			sc->curr_chip_substate_id == CHIP_STATE_4_0 ||
			sc->curr_chip_substate_id == CHIP_STATE_3_0) &&
			to_chip_substate_id < CHIP_STATE_3_0) {
		mutex_lock(&sc->op_lock);
		ret = sc->clk_ops->deattach_mif_clk_ref(sc->clk_ops->ctx);
		mutex_unlock(&sc->op_lock);
		if (ret)
			return ret;
	}

	/*
	 * TODO May need to roll-back the block states if only partial
	 * blocks are set to destination state.
	 */

	if (blk_set_state(sc, &(sc->blocks[BLK_IPU]),
			map->ipu_block_state_id,
			(map->flags & IPU_POWER_CONTROL),
			to_chip_substate_id)) {
		return -EINVAL;
	}

	if (blk_set_state(sc, &(sc->blocks[BLK_TPU]),
			map->tpu_block_state_id,
			(map->flags & TPU_POWER_CONTROL),
			to_chip_substate_id)) {
		return -EINVAL;
	}

	if (blk_set_state(sc, &(sc->blocks[DRAM]),
			map->dram_block_state_id, true, to_chip_substate_id)) {
		return -EINVAL;
	}

	if (blk_set_state(sc, &(sc->blocks[BLK_MIF]),
			map->mif_block_state_id, true, to_chip_substate_id)) {
		return -EINVAL;
	}

	if (blk_set_state(sc, &(sc->blocks[BLK_FSYS]),
			map->fsys_block_state_id, true, to_chip_substate_id)) {
		return -EINVAL;
	}

	if (blk_set_state(sc, &(sc->blocks[BLK_AON]),
			map->aon_block_state_id, true, to_chip_substate_id)) {
		return -EINVAL;
	}

	if ((to_chip_substate_id == CHIP_STATE_3_0 ||
			to_chip_substate_id == CHIP_STATE_4_0 ||
			to_chip_substate_id == CHIP_STATE_5_0) &&
			sc->curr_chip_substate_id < CHIP_STATE_3_0) {
		mutex_lock(&sc->op_lock);
		ret = sc->clk_ops->attach_mif_clk_ref(sc->clk_ops->ctx);
		mutex_unlock(&sc->op_lock);
		if (ret)
			return ret;
	}

	if (((to_chip_substate_id == CHIP_STATE_5_0) ||
			(to_chip_substate_id == CHIP_STATE_6_0)) &&
			(sc->curr_chip_substate_id < CHIP_STATE_5_0)) {
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->pcie_pre_disable(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);

		if (msm_pcie_pm_control(MSM_PCIE_SUSPEND, 0, sc->pcie_dev, NULL,
				MSM_PCIE_CONFIG_NO_CFG_RESTORE))
			pr_err("PCIe failed to disable link\n");

		ab_disable_pgood(sc);
		msm_pcie_assert_perst(1);
		ab_gpio_disable_fw_patch(sc);
		disable_ref_clk(sc->dev);
	}

	ab_pmic_off(sc);

	if (to_chip_substate_id == CHIP_STATE_5_0) {
		ab_gpio_disable_ddr_iso(sc);
		ab_gpio_disable_ddr_sr(sc);
	}

	sc->curr_chip_substate_id = to_chip_substate_id;

	/* record state change */
	ab_sm_record_state_change(prev_state, sc->curr_chip_substate_id, sc);

	mutex_lock(&sc->async_fifo_lock);
	if (sc->async_entries) {
		kfifo_in(sc->async_entries,
			&sc->curr_chip_substate_id,
			sizeof(sc->curr_chip_substate_id));
	}
	mutex_unlock(&sc->async_fifo_lock);

	complete_all(&sc->state_change_comp);

	dev_info(sc->dev, "AB state changed to %d\n", to_chip_substate_id);
	dev_dbg(sc->dev, "IPU clk -> %s %dHz",
		sc->blocks[BLK_IPU].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_IPU].current_state->clk_frequency);
	dev_dbg(sc->dev, "TPU clk -> %s %dHz",
		sc->blocks[BLK_TPU].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_TPU].current_state->clk_frequency);
	dev_dbg(sc->dev, "DRAM clk -> %s %dHz",
		sc->blocks[DRAM].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[DRAM].current_state->clk_frequency);
	dev_dbg(sc->dev, "MIF clk -> %s %dHz",
		sc->blocks[BLK_MIF].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_MIF].current_state->clk_frequency);
	dev_dbg(sc->dev, "FSYS clk -> %s %dHz",
		sc->blocks[BLK_FSYS].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_FSYS].current_state->clk_frequency);
	dev_dbg(sc->dev, "AON clk -> %s %dHz",
		sc->blocks[BLK_AON].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_AON].current_state->clk_frequency);

	return 0;
}

/* Caller must hold sc->state_lock */
static int _ab_sm_set_state(struct ab_state_context *sc,
		u32 dest_chip_substate_id)
{
	if (sc->dest_chip_substate_id == dest_chip_substate_id)
		return 0;

	if (!is_valid_transition(sc->curr_chip_substate_id,
			dest_chip_substate_id)) {
		dev_err(sc->dev,
				"%s: invalid state change, current %u, requested %u\n",
				__func__, sc->curr_chip_substate_id,
				dest_chip_substate_id);
		return -EINVAL;
	}

	sc->dest_chip_substate_id = dest_chip_substate_id;
	return ab_sm_update_chip_state(sc);
}

int ab_sm_set_state(struct ab_state_context *sc, u32 dest_chip_substate_id)
{
	int ret;

	mutex_lock(&sc->state_lock);
	ret = _ab_sm_set_state(sc, dest_chip_substate_id);
	mutex_unlock(&sc->state_lock);

	return ret;
}
EXPORT_SYMBOL(ab_sm_set_state);

enum chip_state ab_sm_get_state(struct ab_state_context *sc)
{
	enum chip_state ret;

	mutex_lock(&sc->state_lock);
	ret = sc->curr_chip_substate_id;
	mutex_unlock(&sc->state_lock);
	return ret;
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

/**
 * ab_sm_clk_notify - call Airbrush clk notifier chain
 * @event: clk notifier type (see include/linux/airbrush-sm-notifier.h)
 * @old_rate: old clk rate in Hz
 * @new_rate: new clk rate in Hz
 *
 * Intended to be called by Airbrush clk provider only.
 * Returns NOTIFY_DONE from the last driver called if all went well,
 * or NOTIFY_STOP or NOTIFY_BAD immediately if a driver returns that,
 * or -EAGAIN if ab_sm has not initialized.
 */
int ab_sm_clk_notify(unsigned long event,
		     unsigned long old_rate,
		     unsigned long new_rate)
{
	struct ab_clk_notifier_data clk_data;

	if (!ab_sm_ctx)
		return -EAGAIN;

	clk_data.old_rate = old_rate;
	clk_data.new_rate = new_rate;

	return blocking_notifier_call_chain(&ab_sm_ctx->clk_subscribers,
					    event,
					    &clk_data);
}
EXPORT_SYMBOL(ab_sm_clk_notify);

int ab_sm_register_clk_event(struct notifier_block *nb)
{
	if (!ab_sm_ctx)
		return -EAGAIN;

	return blocking_notifier_chain_register(
				&ab_sm_ctx->clk_subscribers, nb);
}
EXPORT_SYMBOL(ab_sm_register_clk_event);

int ab_sm_unregister_clk_event(struct notifier_block *nb)
{
	if (!ab_sm_ctx)
		return -EAGAIN;

	return blocking_notifier_chain_unregister(
				&ab_sm_ctx->clk_subscribers, nb);
}
EXPORT_SYMBOL(ab_sm_unregister_clk_event);

enum ab_chip_id ab_get_chip_id(struct ab_state_context *sc)
{
	uint32_t val;
	int ret;

	if (sc->chip_id == CHIP_ID_UNKNOWN) {
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->get_chip_id(sc->mfd_ops->ctx, &val);
		mutex_unlock(&sc->mfd_lock);

		if (ret < 0)
			return CHIP_ID_UNKNOWN;

		sc->chip_id = (enum ab_chip_id)val;
	}

	return sc->chip_id;
}

void ab_sm_register_pmu_ops(struct ab_sm_pmu_ops *ops)
{
	mutex_lock(&ab_sm_ctx->op_lock);
	ab_sm_ctx->pmu_ops = ops;
	mutex_unlock(&ab_sm_ctx->op_lock);
}
EXPORT_SYMBOL(ab_sm_register_pmu_ops);

void ab_sm_unregister_pmu_ops(void)
{
	mutex_lock(&ab_sm_ctx->op_lock);
	ab_sm_ctx->pmu_ops = &pmu_ops_stub;
	mutex_unlock(&ab_sm_ctx->op_lock);
}
EXPORT_SYMBOL(ab_sm_unregister_pmu_ops);

void ab_sm_register_clk_ops(struct ab_sm_clk_ops *ops)
{
	mutex_lock(&ab_sm_ctx->op_lock);
	ab_sm_ctx->clk_ops = ops;
	mutex_unlock(&ab_sm_ctx->op_lock);
}
EXPORT_SYMBOL(ab_sm_register_clk_ops);

void ab_sm_unregister_clk_ops(void)
{
	mutex_lock(&ab_sm_ctx->op_lock);
	ab_sm_ctx->clk_ops = &clk_ops_stub;
	mutex_unlock(&ab_sm_ctx->op_lock);
}
EXPORT_SYMBOL(ab_sm_unregister_clk_ops);

void ab_sm_register_dram_ops(struct ab_sm_dram_ops *ops)
{
	mutex_lock(&ab_sm_ctx->op_lock);
	ab_sm_ctx->dram_ops = ops;
	mutex_unlock(&ab_sm_ctx->op_lock);
}
EXPORT_SYMBOL(ab_sm_register_dram_ops);

void ab_sm_unregister_dram_ops(void)
{
	mutex_lock(&ab_sm_ctx->op_lock);
	ab_sm_ctx->dram_ops = &dram_ops_stub;
	mutex_unlock(&ab_sm_ctx->op_lock);
}
EXPORT_SYMBOL(ab_sm_unregister_dram_ops);

void ab_sm_register_mfd_ops(struct ab_sm_mfd_ops *ops)
{
	mutex_lock(&ab_sm_ctx->mfd_lock);
	ab_sm_ctx->mfd_ops = ops;
	mutex_unlock(&ab_sm_ctx->mfd_lock);
}
EXPORT_SYMBOL(ab_sm_register_mfd_ops);

void ab_sm_unregister_mfd_ops(void)
{
	mutex_lock(&ab_sm_ctx->mfd_lock);
	ab_sm_ctx->mfd_ops = &mfd_ops_stub;
	mutex_unlock(&ab_sm_ctx->mfd_lock);
}
EXPORT_SYMBOL(ab_sm_unregister_mfd_ops);

void ab_enable_pgood(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->soc_pwrgood, __GPIO_ENABLE);
}

void ab_disable_pgood(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->soc_pwrgood, __GPIO_DISABLE);
}

int ab_gpio_get_ddr_sr(struct ab_state_context *ab_ctx)
{
	return gpiod_get_value_cansleep(ab_ctx->ddr_sr);
}

void ab_gpio_enable_ddr_sr(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->ddr_sr, __GPIO_ENABLE);
}

void ab_gpio_disable_ddr_sr(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->ddr_sr, __GPIO_DISABLE);
}

int ab_gpio_get_ddr_iso(struct ab_state_context *ab_ctx)
{
	return gpiod_get_value_cansleep(ab_ctx->ddr_iso);
}

void ab_gpio_enable_ddr_iso(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->ddr_iso, __GPIO_ENABLE);
}

void ab_gpio_disable_ddr_iso(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->ddr_iso, __GPIO_DISABLE);
}

void ab_gpio_enable_fw_patch(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->fw_patch_en, __GPIO_ENABLE);
}

void ab_gpio_disable_fw_patch(struct ab_state_context *ab_ctx)
{
	gpiod_set_value_cansleep(ab_ctx->fw_patch_en, __GPIO_DISABLE);
}

static long ab_sm_async_notify(struct ab_sm_misc_session *sess,
		unsigned long arg)
{
	int ret;
	int chip_state;

	mutex_lock(&sess->sc->async_fifo_lock);
	sess->sc->async_entries = &sess->async_entries;
	mutex_unlock(&sess->sc->async_fifo_lock);

	if (kfifo_is_empty(&sess->async_entries)) {
		if (sess->first_entry) {
			sess->first_entry = false;
			if (copy_to_user((void __user *)arg,
					&sess->sc->curr_chip_substate_id,
					sizeof(chip_state)))
				return -EFAULT;

			reinit_completion(&sess->sc->state_change_comp);
			return 0;

		} else {
			ret = wait_for_completion_interruptible(
					&sess->sc->state_change_comp);
			if (ret < 0)
				return ret;
		}
	}

	reinit_completion(&sess->sc->state_change_comp);

	if (!kfifo_is_empty(&sess->async_entries)) {
		kfifo_out(&sess->async_entries, &chip_state,
				sizeof(chip_state));
		if (copy_to_user((void __user *)arg,
				&chip_state,
				sizeof(chip_state)))
			return -EFAULT;
	} else {
		/* Another ioctl may have closed causing a completion,
		 * can safely ignore
		 */
		return -EAGAIN;
	}

	sess->first_entry = false;
	return 0;
}

static int ab_sm_misc_open(struct inode *ip, struct file *fp)
{
	struct ab_sm_misc_session *sess;
	struct miscdevice *misc_dev = fp->private_data;
	struct ab_state_context *sc =
		container_of(misc_dev, struct ab_state_context, misc_dev);

	sess = kzalloc(sizeof(struct ab_sm_misc_session), GFP_KERNEL);
	if (!sess)
		return -ENOMEM;

	sess->sc = sc;
	sess->first_entry = true;
	kfifo_alloc(&sess->async_entries, 32 * sizeof(int), GFP_KERNEL);

	fp->private_data = sess;

	return 0;
}

static int ab_sm_misc_release(struct inode *ip, struct file *fp)
{
	struct ab_sm_misc_session *sess = fp->private_data;
	struct ab_state_context *sc = sess->sc;

	complete_all(&sc->state_change_comp);

	mutex_lock(&sc->async_fifo_lock);
	if (&sess->async_entries == sc->async_entries)
		sc->async_entries = NULL;
	kfree(sess);
	mutex_unlock(&sc->async_fifo_lock);
	return 0;
}

static long ab_sm_misc_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	long ret;
	struct ab_sm_misc_session *sess = fp->private_data;
	struct ab_state_context *sc = sess->sc;

	switch (cmd) {
	case AB_SM_ASYNC_NOTIFY:
		if (!atomic_cmpxchg(&sc->async_in_use, 0, 1)) {
			ret = ab_sm_async_notify(sess, arg);
			atomic_set(&sc->async_in_use, 0);
		} else {
			dev_dbg(sc->dev, "AB_SM_ASYNC_NOTIFY is in use\n");
			ret = -EBUSY;
		}
		break;

	case AB_SM_SET_STATE:
		ret = ab_sm_set_state(sc, (u32)arg);
		break;

	case AB_SM_GET_STATE:
		ret = ab_sm_get_state(sess->sc);
		if (copy_to_user((void __user *)arg, &ret, sizeof(int)))
			return -EFAULT;
		ret = 0;
		break;

	case AB_SM_ENTER_EL2:
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->enter_el2(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);
		break;

	case AB_SM_EXIT_EL2:
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->exit_el2(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);
		break;

	default:
		dev_err(sc->dev,
			"%s: Unknown ioctl cmd 0x%X\n", __func__, cmd);
		return -EINVAL;
	}

	return ret;
}

static const struct file_operations ab_misc_fops = {
	.owner = THIS_MODULE,
	.open = ab_sm_misc_open,
	.release = ab_sm_misc_release,
	.unlocked_ioctl = ab_sm_misc_ioctl,
};

static void ab_sm_thermal_throttle_state_updated(
		enum throttle_state throttle_state_id, void *op_data)
{
	struct ab_state_context *ctx = op_data;

	mutex_lock(&ctx->state_lock);
	ctx->throttle_state_id = throttle_state_id;
	dev_dbg(ctx->dev, "Throttle state updated to %lu", throttle_state_id);
	ab_sm_update_chip_state(ctx);
	mutex_unlock(&ctx->state_lock);
}

static const struct ab_thermal_ops ab_sm_thermal_ops = {
	.throttle_state_updated = ab_sm_thermal_throttle_state_updated,
};

static void ab_sm_state_stats_init(struct ab_state_context *sc)
{
	enum stat_state curr_stat_state =
		chip_state_to_stat_state(sc->curr_chip_substate_id);

	sc->state_stats[curr_stat_state].counter++;
	sc->state_stats[curr_stat_state].last_entry = ktime_get_boottime();
}

struct ab_state_context *ab_sm_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int error;
	int ret;
	u32 boot_time_block_state;

	ab_sm_ctx = devm_kzalloc(dev, sizeof(struct ab_state_context),
							GFP_KERNEL);
	if(ab_sm_ctx == NULL)
		goto fail_mem_alloc;

	ab_sm_ctx->pdev = pdev;
	ab_sm_ctx->dev = &pdev->dev;
	dev_set_drvdata(ab_sm_ctx->dev, ab_sm_ctx);

	ab_sm_ctx->misc_dev.minor = MISC_DYNAMIC_MINOR;
	ab_sm_ctx->misc_dev.name = "ab_sm";
	ab_sm_ctx->misc_dev.fops = &ab_misc_fops;

	ret = misc_register(&ab_sm_ctx->misc_dev);
	if (ret < 0) {
		dev_err(ab_sm_ctx->dev,
			"Failed to register misc device node (ret = %d)", ret);
		goto fail_misc_reg;
	}

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

	/* Get the alternate-boot property from dt node. This property
	 * allows secondary boot via SPI.
	 */
	if (of_property_read_u32(np, "alternate-boot",
			&ab_sm_ctx->alternate_boot))
		dev_dbg(dev, "alternate-boot property not found\n");

	/* Intialize the default state of each block for state manager */
	boot_time_block_state = ARRAY_SIZE(ipu_property_table)-1;
	ab_sm_ctx->blocks[BLK_IPU] = (struct block){BLK_IPU,
			&ipu_property_table[boot_time_block_state],
			ipu_property_table,
			ARRAY_SIZE(ipu_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(tpu_property_table)-1;
	ab_sm_ctx->blocks[BLK_TPU] = (struct block){BLK_TPU,
			&tpu_property_table[boot_time_block_state],
			tpu_property_table,
			ARRAY_SIZE(tpu_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(dram_property_table)-1;
	ab_sm_ctx->blocks[DRAM] = (struct block){DRAM,
			&dram_property_table[boot_time_block_state],
			dram_property_table,
			ARRAY_SIZE(dram_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(mif_property_table)-1;
	ab_sm_ctx->blocks[BLK_MIF] = (struct block){BLK_MIF,
			&mif_property_table[boot_time_block_state],
			mif_property_table,
			ARRAY_SIZE(mif_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(fsys_property_table)-1;
	ab_sm_ctx->blocks[BLK_FSYS] = (struct block){BLK_FSYS,
			&fsys_property_table[boot_time_block_state],
			fsys_property_table,
			ARRAY_SIZE(fsys_property_table), NULL, NULL};

	boot_time_block_state = ARRAY_SIZE(aon_property_table)-1;
	ab_sm_ctx->blocks[BLK_AON] = (struct block){BLK_AON,
			&aon_property_table[boot_time_block_state],
			aon_property_table,
			ARRAY_SIZE(aon_property_table), NULL, NULL};

	/* intitialize the default chip state */
	ab_sm_ctx->chip_state_table = chip_state_map;
	ab_sm_ctx->nr_chip_states = ARRAY_SIZE(chip_state_map);
	ab_sm_ctx->dest_chip_substate_id = CHIP_STATE_6_0;
	ab_sm_ctx->curr_chip_substate_id = CHIP_STATE_6_0;

	mutex_init(&ab_sm_ctx->pmic_lock);
	mutex_init(&ab_sm_ctx->state_lock);
	mutex_init(&ab_sm_ctx->async_fifo_lock);
	mutex_init(&ab_sm_ctx->op_lock);
	mutex_init(&ab_sm_ctx->mfd_lock);
	atomic_set(&ab_sm_ctx->clocks_registered, 0);
	atomic_set(&ab_sm_ctx->async_in_use, 0);
	init_completion(&ab_sm_ctx->state_change_comp);

	ab_sm_ctx->chip_id = CHIP_ID_UNKNOWN;
	ab_sm_ctx->cold_boot = true;

	/* initialize state stats */
	ab_sm_state_stats_init(ab_sm_ctx);

	/* Initialize stub ops */
	ab_sm_register_pmu_ops(&pmu_ops_stub);
	ab_sm_register_clk_ops(&clk_ops_stub);
	ab_sm_register_dram_ops(&dram_ops_stub);
	ab_sm_register_mfd_ops(&mfd_ops_stub);

	/*
	 * TODO error handle at airbrush-sm should return non-zero value to
	 * free this.
	 */
	devm_ab_thermal_create(ab_sm_ctx->dev, &ab_sm_thermal_ops, ab_sm_ctx);
	ab_sm_ctx->throttle_state_id = THROTTLE_NONE;

	ab_sm_create_debugfs(ab_sm_ctx);
	ab_sm_create_sysfs(ab_sm_ctx);

	BLOCKING_INIT_NOTIFIER_HEAD(&ab_sm_ctx->clk_subscribers);
	return ab_sm_ctx;

fail_fw_patch_en:
fail_ab_ready:
fail_misc_reg:
	misc_deregister(&ab_sm_ctx->misc_dev);
	devm_kfree(dev, (void *)ab_sm_ctx);
	ab_sm_ctx = NULL;

fail_mem_alloc:
	return NULL;
}
EXPORT_SYMBOL(ab_sm_init);

void ab_sm_exit(struct platform_device *pdev)
{
	ab_sm_remove_sysfs(ab_sm_ctx);
	ab_sm_remove_debugfs(ab_sm_ctx);
}

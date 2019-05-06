/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Shaik Ameer Basha <shaik.ameer@samsung.com>
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *
 * Airbrush State Manager Control driver.
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
#include <linux/kthread.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/pm_wakeup.h>
#include <soc/qcom/subsystem_notif.h>
#include <uapi/linux/ab-sm.h>
#include <uapi/linux/sched/types.h>

#include "airbrush-cooling.h"
#include "airbrush-ddr.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-pmu.h"
#include "airbrush-regs.h"
#include "airbrush-spi.h"
#include "airbrush-thermal.h"

#define CREATE_TRACE_POINTS
#include <trace/events/airbrush.h>

#ifdef CONFIG_AB_DDR_SANITY_TEST
#define AB_MAX_TRANSITION_TIME_MS	\
	(10000 + (150 * CONFIG_AB_DDR_SANITY_SZ_MBYTES))
#else
#define AB_MAX_TRANSITION_TIME_MS	10000
#endif
#define AB_KFIFO_ENTRY_SIZE	32
#define to_chip_substate_category(chip_substate_id) ((chip_substate_id) / 100)

static struct ab_state_context *ab_sm_ctx;

#define MAX_AON_FREQ 934000000

#define AB_THROTTLE_TIMEOUT_MS	500

static int pmu_ipu_sleep_stub(void *ctx)      { return -ENODEV; }
static int pmu_tpu_sleep_stub(void *ctx)      { return -ENODEV; }
static int pmu_ipu_tpu_sleep_stub(void *ctx)      { return -ENODEV; }
static int pmu_deep_sleep_stub(void *ctx) { return -ENODEV; }
static int pmu_ipu_resume_stub(void *ctx)     { return -ENODEV; }
static int pmu_tpu_resume_stub(void *ctx)     { return -ENODEV; }
static int pmu_ipu_tpu_resume_stub(void *ctx)     { return -ENODEV; }

static struct ab_sm_pmu_ops pmu_ops_stub = {
	.ctx = NULL,

	.pmu_ipu_sleep = &pmu_ipu_sleep_stub,
	.pmu_tpu_sleep = &pmu_tpu_sleep_stub,
	.pmu_ipu_tpu_sleep = &pmu_ipu_tpu_sleep_stub,
	.pmu_deep_sleep = &pmu_deep_sleep_stub,
	.pmu_ipu_resume = &pmu_ipu_resume_stub,
	.pmu_tpu_resume = &pmu_tpu_resume_stub,
	.pmu_ipu_tpu_resume = &pmu_ipu_tpu_resume_stub,
};

static void ab_clk_init_stub(void *ctx)   { return; }

static int64_t ipu_set_rate_stub(void *ctx, u64 old_rate, u64 new_rate)
{
	return -ENODEV;
}

static int64_t tpu_set_rate_stub(void *ctx, u64 old_rate, u64 new_rate)
{
	return -ENODEV;
}
static int64_t ipu_tpu_set_rate_stub(void *ctx,
		u64 old_ipu_rate, u64 new_ipu_rate,
		u64 old_tpu_rate, u64 new_tpu_rate)
{
	return -ENODEV;
}

static int64_t aon_set_rate_stub(void *ctx, u64 old_rate, u64 new_rate)
{
	return -ENODEV;
}

static struct ab_sm_clk_ops clk_ops_stub = {
	.ctx = NULL,

	.init = &ab_clk_init_stub,

	.ipu_set_rate = &ipu_set_rate_stub,
	.tpu_set_rate = &tpu_set_rate_stub,
	.ipu_tpu_set_rate = &ipu_tpu_set_rate_stub,
	.aon_set_rate = &aon_set_rate_stub,
};

static int ddr_setup_stub(void *ctx, void *ab_state_ctx) { return -ENODEV; }
static int ddr_wait_for_m0_ddr_init_stub(void *ctx) { return -ENODEV; }
static int ddr_init_stub(void *ctx) { return -ENODEV; }
static int ddr_train_all_stub(void *ctx) { return -ENODEV; }
static int ddr_get_freq_stub(void *ctx, u64 *val) { return -ENODEV; }
static int ddr_set_freq_stub(void *ctx, u64 val) { return -ENODEV; }
static int ddr_suspend_stub(void *ctx) { return -ENODEV; }
static int ddr_resume_stub(void *ctx) { return -ENODEV; }
static int ddr_sref_enter_stub(void *ctx) { return -ENODEV; }
static int ddr_sref_exit_stub(void *ctx) { return -ENODEV; }
static int ddr_read_write_test_stub(void *ctx, unsigned int read_write)
{
	return -ENODEV;
}
static int ddr_eye_margin_stub(void *ctx, unsigned int test_data)
{
	return -ENODEV;
}
static int ddr_eye_margin_plot_stub(void *ctx) { return -ENODEV; }
static int ddr_ppc_set_event_stub(void *ctx, unsigned int counter_idx,
				  unsigned int event)
{
	return -ENODEV;
}
static void ddr_ppc_ctrl_stub(void *ctx, int is_start) { return; }

static struct ab_sm_dram_ops dram_ops_stub = {
	.ctx = NULL,

	.setup = &ddr_setup_stub,
	.wait_for_m0_ddr_init = &ddr_wait_for_m0_ddr_init_stub,
	.init = &ddr_init_stub,
	.train_all = &ddr_train_all_stub,
	.get_freq = &ddr_get_freq_stub,
	.set_freq = &ddr_set_freq_stub,
	.suspend = &ddr_suspend_stub,
	.resume = &ddr_resume_stub,
	.sref_enter = &ddr_sref_enter_stub,
	.sref_exit = &ddr_sref_exit_stub,
	.rw_test = &ddr_read_write_test_stub,
	.eye_margin = &ddr_eye_margin_stub,
	.eye_margin_plot = &ddr_eye_margin_plot_stub,
	.ppc_set_event = &ddr_ppc_set_event_stub,
	.ppc_ctrl = &ddr_ppc_ctrl_stub,
};

static int enter_el2_stub(void *ctx) { return -ENODEV; }
static int exit_el2_stub(void *ctx)  { return -ENODEV; }
static int get_chip_id_stub(void *ctx, enum ab_chip_id *val)
{
	return -ENODEV;
}

static int ab_ready_stub(void *ctx)  { return -ENODEV; }
static int pcie_pre_disable_stub(void *ctx)  { return -ENODEV; }
static int pcie_linkdown_stub(void *ctx)  { return -ENODEV; }

static struct ab_sm_mfd_ops mfd_ops_stub = {
	.ctx = NULL,

	.enter_el2 = &enter_el2_stub,
	.exit_el2 = &exit_el2_stub,
	.get_chip_id = &get_chip_id_stub,
	.ab_ready = &ab_ready_stub,
	.pcie_pre_disable = &pcie_pre_disable_stub,
	.pcie_linkdown = &pcie_linkdown_stub,
};

/* Index 0 - A0 clk frequencies
 * Index 1 - B0 clk frequencies
 * Note: The block order here must match the ipu_property_table block order
 */
static const u64 blk_ipu_clk_tbl[][2] = {
	/* BLOCK_STATE_0   */ { 0, 0 },
	/* BLOCK_STATE_100 */ { 0, 0 },
	/* BLOCK_STATE_200 */ { 0, 0 },
	/* BLOCK_STATE_300 */ { AB_SM_OSC_RATE, AB_SM_OSC_RATE },
	/* BLOCK_STATE_301 */ { 50000000,  50000000  },
	/* BLOCK_STATE_302 */ { 220000000, 271800000 },
	/* BLOCK_STATE_303 */ { 330000000, 408000000 },
	/* BLOCK_STATE_304 */ { 440000000, 543600000 },
	/* BLOCK_STATE_305 */ { 549600000, 680000000 },
};

/* Index 0 - A0 clk frequencies
 * Index 1 - B0 clk frequencies
 * Note: The block order here must match the tpu_property_table block order
 */
static const u64 blk_tpu_clk_tbl[][2] = {
	/* BLOCK_STATE_0   */ { 0, 0 },
	/* BLOCK_STATE_100 */ { 0, 0 },
	/* BLOCK_STATE_200 */ { 0, 0 },
	/* BLOCK_STATE_300 */ { AB_SM_OSC_RATE, AB_SM_OSC_RATE },
	/* BLOCK_STATE_301 */ { 50000000,  50000000   },
	/* BLOCK_STATE_302 */ { 306400000, 316000000  },
	/* BLOCK_STATE_303 */ { 459600000, 474000000  },
	/* BLOCK_STATE_304 */ { 612800000, 632000000  },
	/* BLOCK_STATE_305 */ { 765600000, 789600000  },
};

#define BLK_(num, state, sub, pmu, rail, v, clk, freq, pwr, used, tiles, dr) \
	{							\
		BLOCK_STATE_ ## num,	\
		#state,			\
		#sub,			\
		pmu,			\
		rail,			\
		VOLTAGE_ ## v,	\
		clk,			\
		(u64)(1000000. * freq), \
		pwr,	\
		used,	\
		tiles,	\
		dr,		\
	}

static struct block_property ipu_property_table[] = {
	BLK_(0,   Disabled,  NoRail,    0, off, 0_0, off, 0, 0,  0,  0, 0),
	BLK_(100, DeepSleep, DeepSleep, 1, off, 0_0, off, 0, 0,  0,  0, 0),
	BLK_(200, Sleep,  Sleep,      2,  on,  0_75, off, 0, 0,  0,  0, 0),
	BLK_(300, Normal, Ready,      3,  on,  0_75, on,  0, 14, 0,  0, 0),
	BLK_(301, Normal, AonCompute, 3,  on,  0_75, on,  0, 2,  2,  0, 0),
	BLK_(302, Normal, MinCompute, 3,  on,  0_75, on,  0, 14, 14, 0, 0),
	BLK_(303, Normal, LowCompute, 3,  on,  0_75, on,  0, 14, 14, 0, 0),
	BLK_(304, Normal, MidCompute, 3,  on,  0_75, on,  0, 14, 14, 0, 0),
	BLK_(305, Normal, MaxCompute, 3,  on,  0_75, on,  0, 14, 14, 0, 0),
};

static struct block_property tpu_property_table[] = {
	BLK_(0,   Disabled,  NoRail,    0, off, 0_0, off, 0, 0, 0, 0,  0),
	BLK_(100, DeepSleep, DeepSleep, 1, off, 0_0, off, 0, 0, 0, 0,  0),
	BLK_(200, Sleep,  Sleep,      2,  on,  0_75, off, 0, 0, 0, 0,  0),
	BLK_(300, Normal, Ready,      3,  on,  0_75, on,  0, 0, 0, 16, 0),
	BLK_(301, Normal, AonCompute, 3,  on,  0_75, on,  0, 0, 0, 16, 0),
	BLK_(302, Normal, MinCompute, 3,  on,  0_75, on,  0, 0, 0, 16, 0),
	BLK_(303, Normal, LowCompute, 3,  on,  0_75, on,  0, 0, 0, 16, 0),
	BLK_(304, Normal, MidCompute, 3,  on,  0_75, on,  0, 0, 0, 16, 0),
	BLK_(305, Normal, MaxCompute, 3,  on,  0_75, on,  0, 0, 0, 16, 0),
};

static struct block_property dram_property_table[] = {
	BLK_(0, Disabled,    NoRail,    0, off, 0_0, off, 0,   0, 0, 0, 0),
	BLK_(100, Retention, Suspend,   0, off, 0_0, off, 0,   0, 0, 0, 0),
	BLK_(101, Retention, SelfRef,   0, on, 0_60, off, 0,   0, 0, 0, 0),
	BLK_(300, PowerUp,   AonTran,  0, on, 0_60, on,  934,  0, 0, 0, 1867),
	BLK_(301, PowerUp, HalfMidTran, 0, on, 0_60, on, 934,  0, 0, 0, 1867),
	BLK_(302, PowerUp, HalfMaxTran, 0, on, 0_60, on, 934,  0, 0, 0, 1867),
	BLK_(303, PowerUp,   LowTran,   0, on, 0_60, on, 1200, 0, 0, 0, 2400),
	BLK_(304, PowerUp,   MidTran,   0, on, 0_60, on, 1600, 0, 0, 0, 3200),
	BLK_(305, PowerUp,   MaxTran,   0, on, 0_60, on, 1867, 0, 0, 0, 3733),
};

static struct block_property mif_property_table[] = {
	BLK_(0,   Disabled, NoRail,      0, off, 0_0, off, 0,   0, 0, 0, 0),
	BLK_(100, Retention, Standby,    0, on, 0_85, off, 0,   0, 0, 0, 0),
	BLK_(300, Normal,   AonTran,     0, on, 0_85, on,  233, 0, 0, 0, 0),
	BLK_(301, Normal,   HalfMidTran, 0, on, 0_85, on,  233, 0, 0, 0, 0),
	BLK_(302, Normal,   HalfMaxTran, 0, on, 0_85, on,  233, 0, 0, 0, 0),
	BLK_(303, Normal,   LowTran,     0, on, 0_85, on,  300, 0, 0, 0, 0),
	BLK_(304, Normal,   MidTran,     0, on, 0_85, on,  400, 0, 0, 0, 0),
	BLK_(305, Normal,   MaxTran,     0, on, 0_85, on,  467, 0, 0, 0, 0),
};

static struct block_property fsys_property_table[] = {
	BLK_(0, Disabled,         L3,  0, off, 0_0,  off, 0,     0, 0, 0, 0),
	BLK_(100, DeepSleep,      L2,  0, on,  0_85, on,  0,     0, 0, 0, 0),
	BLK_(200, ElectricalIdle, L1,  0, on,  0_85, on,  4000,  0, 0, 0, 0),
	BLK_(201, ElectricalIdle, L1.2, 0, on, 0_85, on,  0,     0, 0, 0, 0),
	BLK_(202, ElectricalIdle, L1.2, 0, on, 0_85, on,  0,     0, 0, 0, 0),
	/* GEN3L1.2 */
	BLK_(300, Normal,        L1.2, 0, on, 0_85, on,  4000,  0, 0, 0, 3),
	/* GEN3L1 */
	BLK_(301, Normal,        L1,  0, on,  0_85, on,  4000,  0, 0, 0, 3),
	/* GEN1L0 */
	BLK_(302, Normal,        L0,  0, on,  0_85, on,  1250,  0, 0, 0, 1),
	/* GEN2L0 */
	BLK_(303, Normal,        L0,  0, on,  0_85, on,  2500,  0, 0, 0, 2),
	/* GEN3L0 */
	BLK_(304, Normal,        L0,  0, on,  0_85, on,  4000,  0, 0, 0, 3),
};

static struct block_property aon_property_table[] = {
	BLK_(0, Disabled, NoRail,  0, off, 0_0,  off, 0,     0, 0, 0, 0),
	BLK_(300, Normal,  Min,    0, on,  0_85, on,  19.2,  0, 0, 0, 0),
	BLK_(301, Normal,  Low,    0, on,  0_85, on,  93.312,   0, 0, 0, 0),
	BLK_(302, Normal,  Mid,    0, on,  0_85, on,  466,   0, 0, 0, 0),
	BLK_(303, Normal,  Max,    0, on,  0_85, on,  933.12, 0, 0, 0, 0),
};

#define CHIP_TO_BLOCK_MAP_INIT(cs, ipu, tpu, dram, mif, fsys, aon) \
	{							\
		CHIP_STATE_ ## cs,		\
		BLOCK_STATE_ ## ipu,	\
		BLOCK_STATE_ ## tpu,	\
		BLOCK_STATE_ ## dram,	\
		BLOCK_STATE_ ## mif,	\
		BLOCK_STATE_ ## fsys,	\
		BLOCK_STATE_ ## aon,	\
	}

static struct chip_to_block_map chip_state_map[] = {
	/*                     CS   IPU  TPU DRAM  MIF FSYS  AON */
	/* Off */
	CHIP_TO_BLOCK_MAP_INIT(0,    0,   0,   0,   0,   0,   0),
	/* Suspend */
	CHIP_TO_BLOCK_MAP_INIT(100,  0,   0,  100,  0,   0,   0),
	/* Deep Sleep */
	CHIP_TO_BLOCK_MAP_INIT(200, 100, 100, 101, 100, 304, 300),
	/* Sleep */
	CHIP_TO_BLOCK_MAP_INIT(300, 200, 200, 101, 100, 304, 300),

	/* Active */
	CHIP_TO_BLOCK_MAP_INIT(400, 300, 300, 305, 305, 300, 302),
	CHIP_TO_BLOCK_MAP_INIT(401, 301, 301, 305, 305, 300, 302),
	CHIP_TO_BLOCK_MAP_INIT(402, 302, 302, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(403, 303, 303, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(404, 304, 305, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(405, 305, 302, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(406, 302, 305, 305, 305, 301, 303),
	CHIP_TO_BLOCK_MAP_INIT(407, 305, 303, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(408, 303, 305, 305, 305, 301, 303),
	CHIP_TO_BLOCK_MAP_INIT(409, 305, 305, 305, 305, 301, 303),


	/* IPU Only */
	CHIP_TO_BLOCK_MAP_INIT(500, 300, 200, 305, 305, 300, 302),
	CHIP_TO_BLOCK_MAP_INIT(501, 301, 200, 305, 305, 300, 302),
	CHIP_TO_BLOCK_MAP_INIT(502, 302, 200, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(503, 303, 200, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(504, 304, 200, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(505, 305, 200, 305, 305, 300, 303),

	/* TPU Only */
	CHIP_TO_BLOCK_MAP_INIT(600, 200, 300, 305, 305, 300, 302),
	CHIP_TO_BLOCK_MAP_INIT(601, 200, 301, 305, 305, 300, 302),
	CHIP_TO_BLOCK_MAP_INIT(602, 200, 302, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(603, 200, 305, 305, 305, 300, 303),
	CHIP_TO_BLOCK_MAP_INIT(604, 200, 305, 305, 305, 301, 303),
	CHIP_TO_BLOCK_MAP_INIT(605, 200, 305, 305, 305, 304, 303),

	/* TPU Only - No DRAM */
	CHIP_TO_BLOCK_MAP_INIT(700, 200, 300, 101, 100, 300, 300),
	CHIP_TO_BLOCK_MAP_INIT(701, 200, 301, 101, 100, 300, 301),
	CHIP_TO_BLOCK_MAP_INIT(702, 200, 302, 101, 100, 300, 301),
	CHIP_TO_BLOCK_MAP_INIT(703, 200, 305, 101, 100, 300, 301),
	CHIP_TO_BLOCK_MAP_INIT(704, 200, 305, 101, 100, 301, 301),
	CHIP_TO_BLOCK_MAP_INIT(705, 200, 305, 101, 100, 304, 301),

	/* IPU Only - Low DRAM */
	CHIP_TO_BLOCK_MAP_INIT(800, 300, 200, 101, 100, 300, 300),
	CHIP_TO_BLOCK_MAP_INIT(801, 301, 200, 303, 303, 300, 300),
	CHIP_TO_BLOCK_MAP_INIT(802, 302, 200, 303, 303, 300, 301),
	CHIP_TO_BLOCK_MAP_INIT(803, 303, 200, 303, 303, 300, 301),
	CHIP_TO_BLOCK_MAP_INIT(804, 304, 200, 303, 303, 300, 301),
	CHIP_TO_BLOCK_MAP_INIT(805, 305, 200, 303, 303, 300, 301),

};

static int ab_update_block_prop_table(struct new_block_props *props,
		enum block_name block, struct ab_state_context *sc)
{
	int i;
	int rows;
	struct block_property *prop_table;

	switch (block) {
	case BLK_IPU:
		prop_table = ipu_property_table;
		rows = ARRAY_SIZE(ipu_property_table);
		break;
	case BLK_TPU:
		prop_table = tpu_property_table;
		rows = ARRAY_SIZE(tpu_property_table);
		break;
	case DRAM:
		prop_table = dram_property_table;
		rows = ARRAY_SIZE(dram_property_table);
		break;
	case BLK_MIF:
		prop_table = mif_property_table;
		rows = ARRAY_SIZE(mif_property_table);
		break;
	case BLK_FSYS:
		prop_table = fsys_property_table;
		rows = ARRAY_SIZE(fsys_property_table);
		break;
	case BLK_AON:
		prop_table = aon_property_table;
		rows = ARRAY_SIZE(aon_property_table);
		break;
	default:
		dev_warn(sc->dev,
			"%s: Should never reach default case!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&sc->state_transitioning_lock);
	for (i = 0; i < rows; i++) {
		if ((int)prop_table[i].id != (int)props->table[i].id) {
			mutex_unlock(&sc->state_transitioning_lock);
			return -EINVAL;
		}

		prop_table[i].pmu =
			(int)props->table[i].pmu;
		prop_table[i].rail_en =
			(int)props->table[i].rail_en;
		prop_table[i].logic_voltage =
			(int)props->table[i].logic_voltage;
		prop_table[i].clk_status =
			(int)props->table[i].clk_status;
		prop_table[i].clk_frequency =
			props->table[i].clk_frequency;
		prop_table[i].num_powered_cores =
			props->table[i].num_powered_cores;
		prop_table[i].num_computing_cores =
			props->table[i].num_computing_cores;
		prop_table[i].num_powered_tiles =
			props->table[i].num_powered_tiles;
		prop_table[i].data_rate =
			props->table[i].data_rate;
	}
	mutex_unlock(&sc->state_transitioning_lock);

	return 0;
}

struct block_property *get_desired_state(struct block *blk,
					 u32 to_block_state_id)
{
	int i;

	for (i = 0; i < (blk->nr_block_states); i++) {
		if (blk->prop_table[i].id == to_block_state_id)
			return &(blk->prop_table[i]);
	}
	return NULL;
}

void ab_sm_register_blk_callback(enum block_name name,
		ab_sm_set_block_state_t callback, void *data)
{
	ab_sm_ctx->blocks[name].set_state = callback;
	ab_sm_ctx->blocks[name].data = data;
}

int clk_set_ipu_tpu_freq(struct ab_state_context *sc,
		struct block_property *last_ipu_state,
		struct block_property *new_ipu_state,
		struct block_property *last_tpu_state,
		struct block_property *new_tpu_state)
{
	int ret = 0;
	int64_t ret_freq;
	u64 old_ipu_freq = last_ipu_state->clk_frequency;
	u64 new_ipu_freq = new_ipu_state->clk_frequency;
	u64 old_tpu_freq = last_tpu_state->clk_frequency;
	u64 new_tpu_freq = new_tpu_state->clk_frequency;

	struct ab_sm_clk_ops *clk = sc->clk_ops;

	if (old_ipu_freq != new_ipu_freq &&
			old_tpu_freq != new_tpu_freq) {
		ab_sm_start_ts(AB_SM_TS_IPU_TPU_CLK);
		ret = clk->ipu_tpu_set_rate(clk->ctx,
				old_ipu_freq, new_ipu_freq,
				old_tpu_freq, new_tpu_freq);
		ab_sm_record_ts(AB_SM_TS_IPU_TPU_CLK);
		if (ret < 0) {
			dev_err(sc->dev,
				"Tried to set ipu freq to %lld and tpu freq to %lld, ret=%d\n",
				new_ipu_freq, new_tpu_freq, ret);
			return ret;
		}

	} else if (old_ipu_freq != new_ipu_freq) {
		ab_sm_start_ts(AB_SM_TS_IPU_CLK);

		ret_freq = clk->ipu_set_rate(clk->ctx,
				old_ipu_freq, new_ipu_freq);
		if (ret_freq != new_ipu_freq) {
			dev_err(sc->dev, "Tried to set ipu freq to %lld but got %lld",
					new_ipu_freq, ret_freq);
			return -ENODEV;
		}

		ab_sm_record_ts(AB_SM_TS_IPU_CLK);

	} else if (old_tpu_freq != new_tpu_freq) {
		ab_sm_start_ts(AB_SM_TS_TPU_CLK);

		ret_freq = clk->tpu_set_rate(clk->ctx,
				old_tpu_freq, new_tpu_freq);
		if (ret_freq != new_tpu_freq) {
			dev_err(sc->dev, "Tried to set tpu freq to %lld but got %lld",
					new_tpu_freq, ret_freq);
			return -ENODEV;
		}

		ab_sm_record_ts(AB_SM_TS_TPU_CLK);
	}

	return 0;
}


/* Caller must hold sc->op_lock */
int clk_set_frequency(struct ab_state_context *sc, struct block *blk,
			 struct block_property *last_state,
			 u64 new_freq, enum states clk_status)
{
	int ret = 0;
	u64 old_freq = last_state->clk_frequency;
	int64_t ret_freq;
	struct ab_sm_clk_ops *clk = sc->clk_ops;

	switch (blk->name) {
	case BLK_IPU:
		ab_sm_start_ts(AB_SM_TS_IPU_CLK);
		if (old_freq == new_freq) {
			ab_sm_record_ts(AB_SM_TS_IPU_CLK);
			break;
		}

		ret_freq = clk->ipu_set_rate(clk->ctx, old_freq, new_freq);
		if (ret_freq != new_freq) {
			dev_err(sc->dev, "Tried to set ipu freq to %lld but got %lld",
					new_freq, ret_freq);
			return -ENODEV;
		}

		ab_sm_record_ts(AB_SM_TS_IPU_CLK);
		break;

	case BLK_TPU:
		ab_sm_start_ts(AB_SM_TS_TPU_CLK);
		if (old_freq == new_freq) {
			ab_sm_record_ts(AB_SM_TS_TPU_CLK);
			break;
		}

		ret_freq = clk->tpu_set_rate(clk->ctx, old_freq, new_freq);
		if (ret_freq != new_freq) {
			dev_err(sc->dev, "Tried to set tpu freq to %lld but got %lld",
					new_freq, ret_freq);
			return -ENODEV;
		}

		ab_sm_record_ts(AB_SM_TS_TPU_CLK);
		break;

	case BLK_MIF:
		break;
	case BLK_FSYS:
		break;
	case BLK_AON:
		ab_sm_start_ts(AB_SM_TS_AON_CLK);
		if (clk_status == off || old_freq == new_freq) {
			ab_sm_record_ts(AB_SM_TS_AON_CLK);
			break;
		}

		ret_freq = clk->aon_set_rate(clk->ctx, old_freq, new_freq);
		if (ret_freq != new_freq) {
			dev_err(sc->dev, "Tried to set aon freq to %lld but got %lld",
					new_freq, ret_freq);
			return -ENODEV;
		}

		ab_sm_record_ts(AB_SM_TS_AON_CLK);
		break;
	case DRAM:
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

int blk_set_state(struct ab_state_context *sc, struct block *blk,
	enum block_state to_block_state_id)
{
	struct ab_sm_pmu_ops *pmu;
	struct block_property *desired_state =
		get_desired_state(blk, to_block_state_id);
	struct block_property *last_state = blk->current_state;

	if (!desired_state)
		return -EINVAL;

	if (last_state->id == desired_state->id)
		return 0;

	if (blk->name == BLK_IPU)
		ab_sm_start_ts(AB_SM_TS_IPU);
	if (blk->name == BLK_TPU)
		ab_sm_start_ts(AB_SM_TS_TPU);

	/* Mark block as new state early in case rollback is needed */
	blk->current_state = desired_state;

	mutex_lock(&sc->op_lock);
	pmu = sc->pmu_ops;
	/* PMU settings - Resume */
	if (desired_state->pmu == PMU_STATE_ON &&
			last_state->pmu != PMU_STATE_ON) {
		if (blk->name == BLK_IPU) {
			ab_sm_start_ts(AB_SM_TS_IPU_PMU_RES);
			if (pmu->pmu_ipu_resume(pmu->ctx)) {
				mutex_unlock(&sc->op_lock);
				return -EAGAIN;
			}
			ab_sm_record_ts(AB_SM_TS_IPU_PMU_RES);
		}
		if (blk->name == BLK_TPU) {
			ab_sm_start_ts(AB_SM_TS_TPU_PMU_RES);
			if (pmu->pmu_tpu_resume(pmu->ctx)) {
				mutex_unlock(&sc->op_lock);
				return -EAGAIN;
			}
			ab_sm_record_ts(AB_SM_TS_TPU_PMU_RES);
		}
	}

	if (clk_set_frequency(sc, blk, last_state, desired_state->clk_frequency,
			desired_state->clk_status)) {
		mutex_unlock(&sc->op_lock);
		return -EAGAIN;
	}

	/* Block specific hooks */
	if (blk->set_state) {
		if (blk->name == DRAM)
			ab_sm_start_ts(AB_SM_TS_DDR_CB);
		if (blk->name == BLK_FSYS)
			ab_sm_start_ts(AB_SM_TS_PCIE_CB);

		blk->set_state(last_state, desired_state,
				   to_block_state_id, blk->data);

		if (blk->name == DRAM)
			ab_sm_record_ts(AB_SM_TS_DDR_CB);
		if (blk->name == BLK_FSYS)
			ab_sm_record_ts(AB_SM_TS_PCIE_CB);
	}

	/* PMU settings - Sleep */
	if (desired_state->pmu == PMU_STATE_SLEEP &&
			last_state->pmu == PMU_STATE_ON) {
		ab_sm_start_ts(AB_SM_TS_TPU_PMU_SLEEP);
		if (blk->name == BLK_TPU) {
			if (pmu->pmu_tpu_sleep(pmu->ctx)) {
				mutex_unlock(&sc->op_lock);
				return -EAGAIN;
			}
		}
		ab_sm_record_ts(AB_SM_TS_TPU_PMU_SLEEP);

		ab_sm_start_ts(AB_SM_TS_IPU_PMU_SLEEP);
		if (blk->name == BLK_IPU) {
			if (pmu->pmu_ipu_sleep(pmu->ctx)) {
				mutex_unlock(&sc->op_lock);
				return -EAGAIN;
			}
		}
		ab_sm_record_ts(AB_SM_TS_IPU_PMU_SLEEP);
	}

	/* PMU settings - Deep Sleep */
	ab_sm_start_ts(AB_SM_TS_PMU_DEEP_SLEEP);
	if (desired_state->pmu == PMU_STATE_DEEP_SLEEP &&
			last_state->pmu != PMU_STATE_DEEP_SLEEP &&
			blk->name == BLK_TPU) {
		if (pmu->pmu_deep_sleep(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
	}
	ab_sm_record_ts(AB_SM_TS_PMU_DEEP_SLEEP);

	mutex_unlock(&sc->op_lock);

	if (blk->name == BLK_IPU)
		ab_sm_record_ts(AB_SM_TS_IPU);
	if (blk->name == BLK_TPU)
		ab_sm_record_ts(AB_SM_TS_TPU);

	return 0;
}

int blk_set_ipu_tpu_states(struct ab_state_context *sc,
		struct block *ipu_blk, enum block_state to_ipu_state,
		struct block *tpu_blk, enum block_state to_tpu_state)
{
	struct ab_sm_pmu_ops *pmu;
	struct block_property *next_ipu_state =
		get_desired_state(ipu_blk, to_ipu_state);
	struct block_property *last_ipu_state = ipu_blk->current_state;
	struct block_property *next_tpu_state =
		get_desired_state(tpu_blk, to_tpu_state);
	struct block_property *last_tpu_state = tpu_blk->current_state;

	if (!next_ipu_state || !next_tpu_state)
		return -EINVAL;

	if (last_ipu_state->id == next_ipu_state->id &&
			last_tpu_state->id == next_tpu_state->id)
		return 0;
	else if (last_ipu_state->id == next_ipu_state->id)
		return blk_set_state(sc, tpu_blk, to_tpu_state);
	else if (last_tpu_state->id == next_tpu_state->id)
		return blk_set_state(sc, ipu_blk, to_ipu_state);

	ipu_blk->current_state = next_ipu_state;
	tpu_blk->current_state = next_tpu_state;

	mutex_lock(&sc->op_lock);
	pmu = sc->pmu_ops;
	/* PMU settings - Resume */
	if (next_ipu_state->pmu == PMU_STATE_ON &&
			next_tpu_state->pmu == PMU_STATE_ON &&
			(last_ipu_state->pmu != PMU_STATE_ON ||
			 last_tpu_state->pmu != PMU_STATE_ON)) {
		ab_sm_start_ts(AB_SM_TS_IPU_TPU_PMU_RES);
		if (pmu->pmu_ipu_tpu_resume(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
		ab_sm_record_ts(AB_SM_TS_IPU_TPU_PMU_RES);

	} else if (next_ipu_state->pmu == PMU_STATE_ON &&
			last_ipu_state->pmu != PMU_STATE_ON) {
		ab_sm_start_ts(AB_SM_TS_IPU_PMU_RES);
		if (pmu->pmu_ipu_resume(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
		ab_sm_record_ts(AB_SM_TS_IPU_PMU_RES);

	} else if (next_tpu_state->pmu == PMU_STATE_ON &&
			last_tpu_state->pmu != PMU_STATE_ON) {
		ab_sm_start_ts(AB_SM_TS_TPU_PMU_RES);
		if (pmu->pmu_tpu_resume(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
		ab_sm_record_ts(AB_SM_TS_TPU_PMU_RES);
	}

	if (clk_set_ipu_tpu_freq(sc,
			last_ipu_state, next_ipu_state,
			last_tpu_state, next_tpu_state)) {
		mutex_unlock(&sc->op_lock);
		return -EAGAIN;
	}

	if (ipu_blk->set_state) {
		ipu_blk->set_state(last_ipu_state, next_ipu_state,
				to_ipu_state, ipu_blk->data);
	}
	if (tpu_blk->set_state) {
		tpu_blk->set_state(last_tpu_state, next_tpu_state,
				to_tpu_state, tpu_blk->data);
	}

	/* PMU settings - Sleep */
	if (next_ipu_state->pmu == PMU_STATE_SLEEP &&
			next_tpu_state->pmu == PMU_STATE_SLEEP &&
			last_ipu_state->pmu == PMU_STATE_ON &&
			last_tpu_state->pmu == PMU_STATE_ON) {
		ab_sm_start_ts(AB_SM_TS_IPU_TPU_PMU_SLEEP);
		if (pmu->pmu_ipu_tpu_sleep(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
		ab_sm_record_ts(AB_SM_TS_IPU_TPU_PMU_SLEEP);

	} else if (next_ipu_state->pmu == PMU_STATE_SLEEP &&
			last_ipu_state->pmu == PMU_STATE_ON) {
		ab_sm_start_ts(AB_SM_TS_IPU_PMU_SLEEP);
		if (pmu->pmu_ipu_sleep(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
		ab_sm_record_ts(AB_SM_TS_IPU_PMU_SLEEP);

	} else if (next_tpu_state->pmu == PMU_STATE_SLEEP &&
			last_tpu_state->pmu == PMU_STATE_ON) {
		ab_sm_start_ts(AB_SM_TS_TPU_PMU_SLEEP);
		if (pmu->pmu_tpu_sleep(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
		ab_sm_record_ts(AB_SM_TS_TPU_PMU_SLEEP);
	}

	/* PMU settings - Deep Sleep */
	ab_sm_start_ts(AB_SM_TS_PMU_DEEP_SLEEP);
	if (next_ipu_state->pmu == PMU_STATE_DEEP_SLEEP &&
			last_ipu_state->pmu != PMU_STATE_DEEP_SLEEP) {
		if (pmu->pmu_deep_sleep(pmu->ctx)) {
			mutex_unlock(&sc->op_lock);
			return -EAGAIN;
		}
	}
	ab_sm_record_ts(AB_SM_TS_PMU_DEEP_SLEEP);

	mutex_unlock(&sc->op_lock);

	return 0;
}

static struct chip_to_block_map *ab_sm_get_block_map(
		struct ab_state_context *sc, int state)
{
	int i;

	for (i = 0; i < sc->nr_chip_states; i++) {
		if (sc->chip_state_table[i].chip_substate_id == state)
			return &(sc->chip_state_table[i]);
	}

	return NULL;
}

static bool is_valid_transition(struct ab_state_context *sc,
		u32 curr_chip_substate_id,
		u32 to_chip_substate_id)
{
	struct chip_to_block_map *curr_map;
	struct chip_to_block_map *to_map;

	switch (curr_chip_substate_id) {
	case CHIP_STATE_200:
		if (to_chip_substate_id == CHIP_STATE_300)
			return false;
		break;
	case CHIP_STATE_100:
		if (to_chip_substate_id == CHIP_STATE_200)
			return false;
		if (to_chip_substate_id == CHIP_STATE_300)
			return false;
		break;
	case CHIP_STATE_0:
		if (to_chip_substate_id == CHIP_STATE_100)
			return false;
		if (to_chip_substate_id == CHIP_STATE_200)
			return false;
		if (to_chip_substate_id == CHIP_STATE_300)
			return false;
		break;
	}

	/* Prevent direct DRAM active -> active clock rate changes.
	 * DRAM must go through intermediate low power state, i.e:
	 * 1867MHz -> self-refresh -> 800MHz
	 * See b/127677742 for more info
	 */
	curr_map = ab_sm_get_block_map(sc, curr_chip_substate_id);
	to_map = ab_sm_get_block_map(sc, to_chip_substate_id);
	if (curr_map == NULL || to_map == NULL)
		return false;

	if (curr_map->dram_block_state_id != to_map->dram_block_state_id) {
		if (curr_map->dram_block_state_id >= BLOCK_STATE_300 &&
				to_map->dram_block_state_id >= BLOCK_STATE_300)
			return false;
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

#define THROTTLER_MAP_INIT(cs0, cs1, cs2, cs3, cs4) \
	{ \
		CHIP_STATE_ ## cs0, \
		CHIP_STATE_ ## cs1, \
		CHIP_STATE_ ## cs2, \
		CHIP_STATE_ ## cs3, \
		CHIP_STATE_ ## cs4, \
	}

static const u32 chip_substate_throttler_map
		[][AIRBRUSH_COOLING_STATE_MAX + 1] = {
	[4] = THROTTLER_MAP_INIT(409, 404, 403, 402, 100),
	[5] = THROTTLER_MAP_INIT(505, 504, 503, 502, 100),
	[6] = THROTTLER_MAP_INIT(605, 604, 603, 602, 100),
	[7] = THROTTLER_MAP_INIT(705, 704, 703, 702, 100),
	[8] = THROTTLER_MAP_INIT(805, 804, 803, 802, 100),
};

static u32 ab_sm_throttled_chip_substate_id(
		u32 chip_substate_id, enum throttle_state throttle_state_id)
{
	u32 substate_category;
	u32 throttler_substate_id;

	if (chip_substate_id <= CHIP_STATE_300)
		return chip_substate_id;

	substate_category = to_chip_substate_category(chip_substate_id);
	throttler_substate_id = chip_substate_throttler_map
			[substate_category][throttle_state_id];
	if (throttler_substate_id < CHIP_STATE_400)
		return throttler_substate_id;
	return min(chip_substate_id, throttler_substate_id);
}

const enum stat_state ab_chip_state_to_stat_state(
		enum chip_state id)
{
	if (id >= CHIP_STATE_400)
		return STAT_STATE_ACTIVE;

	switch (id) {
	case CHIP_STATE_300:
		return STAT_STATE_SLEEP;
	case CHIP_STATE_200:
		return STAT_STATE_DEEP_SLEEP;
	case CHIP_STATE_100:
		return STAT_STATE_SUSPEND;
	case CHIP_STATE_0:
		return STAT_STATE_OFF;
	default:
		/* should never hit this code path */
		return STAT_STATE_UNKNOWN;
	}
}

/* Caller must hold sc->set_state_lock */
static void ab_sm_record_state_change(enum chip_state prev_state,
		enum chip_state new_state,
		struct ab_state_context *sc)
{
	enum stat_state prev = ab_chip_state_to_stat_state(prev_state);
	enum stat_state new = ab_chip_state_to_stat_state(new_state);
	ktime_t time, time_diff;

	if (new == prev)
		return;

	time = ktime_get_boottime();
	sc->state_stats[new].counter++;
	sc->state_stats[new].last_entry = time;
	sc->state_stats[prev].last_exit = time;
	time_diff = ktime_sub(sc->state_stats[prev].last_exit,
			sc->state_stats[prev].last_entry);
	sc->state_stats[prev].duration = ktime_add(
			sc->state_stats[prev].duration, time_diff);
}

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_PROFILE)
void ab_sm_start_ts(int ts)
{
	ab_sm_ctx->state_start_ts[ts] = ktime_get_ns();
}
EXPORT_SYMBOL(ab_sm_start_ts);

void ab_sm_record_ts(int ts)
{
	ab_sm_ctx->state_trans_ts[ts] +=
		ktime_get_ns() - ab_sm_ctx->state_start_ts[ts];
}
EXPORT_SYMBOL(ab_sm_record_ts);

void ab_sm_zero_ts(struct ab_state_context *sc)
{
	memset(sc->state_trans_ts, 0, sizeof(sc->state_trans_ts));
	memset(sc->state_trans_ts, 0, sizeof(sc->state_start_ts));
}

void ab_sm_print_ts(struct ab_state_context *sc)
{
	int i;
	int nr_ts = NUM_AB_SM_TS;

	static const char *ts_names[NUM_AB_SM_TS] = {
		"    boot sequence",
		"        alternate boot",
		"        PCIe Enumeration",
		"        DDR init",
		"    PMIC on",
		"    LVCC",
		"    AON state change for DRAM init",
		"    IPU/TPU state change",
		"    IPU/TPU PMU resume",
		"    IPU/TPU clock settings",
		"    IPU state change",
		"        IPU PMU resume",
		"        IPU clock settings",
		"            IPU pre rate change notify",
		"            IPU get clk",
		"            IPU set oscclk",
		"            IPU set clk rate",
		"            IPU clock lock",
		"            IPU finish clk rate",
		"            IPU post rate change notify",
		"        IPU PMU sleep",
		"    TPU state change",
		"        TPU PMU resume",
		"        TPU clock settings",
		"            TPU pre rate change notify",
		"            TPU get clk",
		"            TPU set oscclk",
		"            TPU set clk rate",
		"            TPU clock lock",
		"            TPU finish clk rate",
		"            TPU post rate change notify",
		"        TPU PMU sleep",
		"        PMU deep sleep",
		"    IPU/TPU PMU sleep",
		"    DDR state change",
		"        DDR callback",
		"            DDR set PLL",
		"            DDR set PLL poll",
		"    MIF state change",
		"    FSYS state change",
		"        PCIe callback",
		"            PCIe get linkspeed",
		"            PCIe get linkstate",
		"            PCIe set linkspeed",
		"            PCIe set linkstate",
		"    AON state change",
		"        AON clock settings",
		"    PMIC off",
		"full state change",
	};

	if (sc->ts_enabled) {
		for (i = 0; i < nr_ts; i++) {
			dev_warn(sc->dev, "latency for %s: %llu ns\n",
					ts_names[i], sc->state_trans_ts[i]);
		}
	}
}
#endif

/* Set the enable/disable flag for each pmic rail.
 * Only sets the flags, does not turn rails on or off
 */
static void ab_prep_pmic_settings(struct ab_state_context *sc,
	struct chip_to_block_map *dest_map)
{
	struct block_property *state;

	state = get_desired_state(&sc->blocks[BLK_IPU],
			dest_map->ipu_block_state_id);
	ab_mark_pmic_rail(sc, BLK_IPU, state->rail_en, state->id);

	state = get_desired_state(&sc->blocks[BLK_TPU],
			dest_map->tpu_block_state_id);
	ab_mark_pmic_rail(sc, BLK_TPU, state->rail_en, state->id);

	state = get_desired_state(&sc->blocks[DRAM],
			dest_map->dram_block_state_id);
	ab_mark_pmic_rail(sc, DRAM, state->rail_en, state->id);

	state = get_desired_state(&sc->blocks[BLK_MIF],
			dest_map->mif_block_state_id);
	ab_mark_pmic_rail(sc, BLK_MIF, state->rail_en, state->id);

	state = get_desired_state(&sc->blocks[BLK_FSYS],
			dest_map->fsys_block_state_id);
	ab_mark_pmic_rail(sc, BLK_FSYS, state->rail_en, state->id);

	state = get_desired_state(&sc->blocks[BLK_AON],
			dest_map->aon_block_state_id);
	ab_mark_pmic_rail(sc, BLK_AON, state->rail_en, state->id);
}

static void __ab_cleanup_state(struct ab_state_context *sc,
			       bool is_linkdown_event)
{
	struct chip_to_block_map *map = ab_sm_get_block_map(sc, CHIP_STATE_0);

	/*
	 * Mark PMIC rails to be disabled so that regulator_enable() and
	 * regualtor_disable() calls are balanced.
	 */
	ab_prep_pmic_settings(sc, map);

	dev_err(sc->dev, "Cleaning AB state\n");
	blk_set_ipu_tpu_states(sc,
			&(sc->blocks[BLK_IPU]), map->ipu_block_state_id,
			&(sc->blocks[BLK_TPU]), map->tpu_block_state_id);
	blk_set_state(sc, &(sc->blocks[DRAM]), map->dram_block_state_id);
	blk_set_state(sc, &(sc->blocks[BLK_MIF]), map->mif_block_state_id);
	blk_set_state(sc, &(sc->blocks[BLK_FSYS]), map->fsys_block_state_id);
	blk_set_state(sc, &(sc->blocks[BLK_AON]), map->aon_block_state_id);

	dev_err(sc->dev, "AB block states cleaned\n");

	if (!is_linkdown_event) {
		/*
		 * Disable thermal before all pcie subscribers getting
		 * disabled.
		 */
		ab_thermal_disable(sc->thermal);
		/* broadcast normal disable */
		mutex_lock(&sc->mfd_lock);
		sc->mfd_ops->pcie_pre_disable(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);
	}

	ab_sm_disable_pcie(sc);
	dev_err(sc->dev, "AB PCIE suspended\n");

	if (is_linkdown_event) {
		/*
		 * Disable thermal before all pcie subscribers getting
		 * disabled.
		 */
		ab_thermal_disable(sc->thermal);
		/* broadcast linkdown event */
		mutex_lock(&sc->mfd_lock);
		sc->mfd_ops->pcie_linkdown(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);
	}

	pm_relax(sc->dev);
	ab_disable_pgood(sc);
	msm_pcie_assert_perst(1);
	ab_gpio_disable_fw_patch(sc);
	disable_ref_clk(sc->dev);
	dev_err(sc->dev, "AB refclks disabled\n");

	ab_pmic_off(sc);
	dev_err(sc->dev, "AB PMIC off\n");

	sc->curr_chip_substate_id = CHIP_STATE_0;
}

/* Attempt to shutdown Airbrush and notify a PCIe linkdown
 * event to client drivers.
 * Caller must hold sc->state_transitioning_lock.
 */
static void ab_cleanup_state_linkdown(struct ab_state_context *sc)
{
	__ab_cleanup_state(sc, /*is_linkdown_event=*/true);
}

/* Attempt to get airbrush into a known state
 * Ignore errors and set to CHIP_STATE_0 (off)
 * Caller must hold sc->state_transitioning_lock
 */
static void ab_cleanup_state(struct ab_state_context *sc)
{
	__ab_cleanup_state(sc, /*is_linkdown_event=*/false);
}

static int ab_sm_update_chip_state(struct ab_state_context *sc)
{
	u32 to_chip_substate_id;
	int ret;
	struct chip_to_block_map *dest_map;
	struct chip_to_block_map *last_map;
	struct chip_to_block_map *active_map;
	enum chip_state prev_state = sc->curr_chip_substate_id;

	ab_sm_zero_ts(sc);

	if (sc->el2_mode || sc->el2_in_secure_context) {
		dev_err(sc->dev, "Cannot change state while in EL2 mode\n");
		return -ENODEV;
	}

	to_chip_substate_id = ab_sm_throttled_chip_substate_id(
			sc->dest_chip_substate_id,
			sc->throttle_state_id);

	if (prev_state == to_chip_substate_id) {
		dev_dbg(sc->dev, "Ignore state change, already at destination\n");
		return 0;
	}

	dest_map = ab_sm_get_block_map(sc, to_chip_substate_id);
	last_map = ab_sm_get_block_map(sc, prev_state);
	if (!is_valid_transition(sc, prev_state, to_chip_substate_id) ||
			!dest_map || !last_map) {
		dev_err(sc->dev,
			"Entered %s with invalid destination state\n",
			__func__);
		WARN_ON(1);

		return -EINVAL;
	}

	dev_info(sc->dev, "AB state changing to %d\n", to_chip_substate_id);
	/* Mark as new state early in case rollback is needed */
	sc->curr_chip_substate_id = to_chip_substate_id;
	ab_sm_start_ts(AB_SM_TS_FULL);

	if ((prev_state == CHIP_STATE_0 ||
			prev_state == CHIP_STATE_100) &&
			to_chip_substate_id >= CHIP_STATE_400) {

		/* Mark all PMIC rails to be enabled before calling
		 * ab_bootsequence, since ab_bootsequence will call
		 * ab_pmic_on
		 */
		if (to_chip_substate_id >= CHIP_STATE_500) {
			active_map = ab_sm_get_block_map(sc, CHIP_STATE_400);
			ab_prep_pmic_settings(sc, active_map);
		} else {
			ab_prep_pmic_settings(sc, dest_map);
		}

		/* System must not suspend while PCIe is enabled */
		pm_stay_awake(sc->dev);

		ab_sm_start_ts(AB_SM_TS_BOOT_SEQ);
		ret = ab_bootsequence(sc, prev_state);
		ab_sm_record_ts(AB_SM_TS_BOOT_SEQ);
		if (ret) {
			dev_err(sc->dev, "ab_bootsequence failed (%d)\n", ret);
			goto cleanup_state;
		}

		/* If we are going to a partially active state, first place
		 * IPU, TPU, and DRAM into fully active states
		 */
		if (to_chip_substate_id >= CHIP_STATE_500) {
			if (blk_set_ipu_tpu_states(sc,
					&(sc->blocks[BLK_IPU]),
					active_map->ipu_block_state_id,
					&(sc->blocks[BLK_TPU]),
					active_map->tpu_block_state_id)) {
				ret = -EINVAL;
				dev_err(sc->dev, "blk_set_ipu_tpu_state failed\n");
				goto cleanup_state;

			}
			if (blk_set_state(sc, &(sc->blocks[DRAM]),
					active_map->dram_block_state_id)) {
				ret = -EINVAL;
				dev_err(sc->dev, "blk_set_state failed for DRAM\n");
				goto cleanup_state;
			}
		}
	}

	/* Mark which PMIC rails will be enabled/disabled
	 * for destination state
	 */
	ab_prep_pmic_settings(sc, dest_map);

	ab_sm_start_ts(AB_SM_TS_PMIC_ON);
	ret = ab_pmic_on(sc);
	ab_sm_record_ts(AB_SM_TS_PMIC_ON);
	if (ret) {
		dev_err(sc->dev, "ab_pmic_on failed (%d)\n", ret);
		return ret;
	}

	ab_sm_start_ts(AB_SM_TS_LVCC);
	if (to_chip_substate_id >= CHIP_STATE_400) {
		ret = ab_lvcc(sc, to_chip_substate_id);
		if (ret) {
			dev_err(sc->dev, "ab_lvcc failed (%d)\n", ret);
			goto cleanup_state;
		}
	}
	ab_sm_record_ts(AB_SM_TS_LVCC);

	/* If DRAM is changing state, ensure AON
	 * clock rate is at maximum to speed up initialization
	 */
	if (dest_map->dram_block_state_id != last_map->dram_block_state_id) {
		active_map = ab_sm_get_block_map(sc, CHIP_STATE_409);

		ab_sm_start_ts(AB_SM_TS_AON_DRAM_INIT);
		if (blk_set_state(sc, &(sc->blocks[BLK_AON]),
					active_map->aon_block_state_id)) {
			ret = -EINVAL;
			if (to_chip_substate_id != CHIP_STATE_0)
				goto cleanup_state;
		}
		ab_sm_record_ts(AB_SM_TS_AON_DRAM_INIT);

	}

	ab_sm_start_ts(AB_SM_TS_IPU_TPU);
	if (blk_set_ipu_tpu_states(sc,
			&(sc->blocks[BLK_IPU]), dest_map->ipu_block_state_id,
			&(sc->blocks[BLK_TPU]), dest_map->tpu_block_state_id)) {
		ret = -EINVAL;
		dev_err(sc->dev, "blk_set_ipu_tpu_state failed\n");
		if (to_chip_substate_id != CHIP_STATE_0)
			goto cleanup_state;

	}
	ab_sm_record_ts(AB_SM_TS_IPU_TPU);

	ab_sm_start_ts(AB_SM_TS_DRAM);
	if (blk_set_state(sc, &(sc->blocks[DRAM]),
			dest_map->dram_block_state_id)) {
		ret = -EINVAL;
		dev_err(sc->dev, "blk_set_state failed for DRAM\n");
		if (to_chip_substate_id != CHIP_STATE_0)
			goto cleanup_state;
	}
	ab_sm_record_ts(AB_SM_TS_DRAM);

	ab_sm_start_ts(AB_SM_TS_MIF);
	if (blk_set_state(sc, &(sc->blocks[BLK_MIF]),
			dest_map->mif_block_state_id)) {
		ret = -EINVAL;
		dev_err(sc->dev, "blk_set_state failed for MIF\n");
		if (to_chip_substate_id != CHIP_STATE_0)
			goto cleanup_state;
	}
	ab_sm_record_ts(AB_SM_TS_MIF);


	ab_sm_start_ts(AB_SM_TS_FSYS);
	/* Save time by not updating FSYS block when going to
	 * SLEEP or DEEP-SLEEP
	 */
	if (to_chip_substate_id != CHIP_STATE_200 &&
			to_chip_substate_id != CHIP_STATE_300) {
		if (blk_set_state(sc, &(sc->blocks[BLK_FSYS]),
					dest_map->fsys_block_state_id)) {
			ret = -EINVAL;
			dev_err(sc->dev, "blk_set_state failed for FSYS\n");
			if (to_chip_substate_id != CHIP_STATE_0)
				goto cleanup_state;
		}
	}
	ab_sm_record_ts(AB_SM_TS_FSYS);

	ab_sm_start_ts(AB_SM_TS_AON);
	if (blk_set_state(sc, &(sc->blocks[BLK_AON]),
			dest_map->aon_block_state_id)) {
		ret = -EINVAL;
		if (to_chip_substate_id != CHIP_STATE_0)
			goto cleanup_state;
	}
	ab_sm_record_ts(AB_SM_TS_AON);

	if (((to_chip_substate_id == CHIP_STATE_100) ||
			(to_chip_substate_id == CHIP_STATE_0)) &&
			(prev_state >= CHIP_STATE_200)) {
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->pcie_pre_disable(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);

		ret = ab_sm_disable_pcie(sc);
		if (ret) {
			if (to_chip_substate_id != CHIP_STATE_0)
				goto cleanup_state;
		}

		pm_relax(sc->dev);
		ab_disable_pgood(sc);
		msm_pcie_assert_perst(1);
		ab_gpio_disable_fw_patch(sc);
		disable_ref_clk(sc->dev);
	}

	if (to_chip_substate_id == CHIP_STATE_100) {
		/* Disabling DDRCKE_ISO is a workaround to prevent
		 * back drive during suspend.
		 * This operation should be moved to after DDR resume for
		 * HW >= EVT1.1 (b/128545111).
		 */
		if (sc->ddrcke_iso_clamp_wr)
			ab_gpio_disable_ddr_iso(sc);
	}

	ab_sm_start_ts(AB_SM_TS_PMIC_OFF);
	ab_pmic_off(sc);
	ab_sm_record_ts(AB_SM_TS_PMIC_OFF);

	ab_sm_record_ts(AB_SM_TS_FULL);

	/* record state change */
	ab_sm_record_state_change(prev_state, sc->curr_chip_substate_id, sc);
	trace_ab_state_change(sc->curr_chip_substate_id);

	mutex_lock(&sc->async_fifo_lock);
	if (sc->async_entries) {
		kfifo_in(sc->async_entries,
			&sc->curr_chip_substate_id,
			sizeof(sc->curr_chip_substate_id));
	}
	mutex_unlock(&sc->async_fifo_lock);

	dev_info(sc->dev, "AB state changed to %d\n", to_chip_substate_id);
	ab_sm_print_ts(sc);

	dev_dbg(sc->dev, "IPU clk -> %s %lluHz",
		sc->blocks[BLK_IPU].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_IPU].current_state->clk_frequency);
	dev_dbg(sc->dev, "TPU clk -> %s %lluHz",
		sc->blocks[BLK_TPU].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_TPU].current_state->clk_frequency);
	dev_dbg(sc->dev, "DRAM clk -> %s %lluHz",
		sc->blocks[DRAM].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[DRAM].current_state->clk_frequency);
	dev_dbg(sc->dev, "MIF clk -> %s %lluHz",
		sc->blocks[BLK_MIF].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_MIF].current_state->clk_frequency);
	dev_dbg(sc->dev, "FSYS clk -> %s %lluHz",
		sc->blocks[BLK_FSYS].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_FSYS].current_state->clk_frequency);
	dev_dbg(sc->dev, "AON clk -> %s %lluHz",
		sc->blocks[BLK_AON].current_state->clk_status == on ?
			"on" : "off",
		sc->blocks[BLK_AON].current_state->clk_frequency);

	return 0;

cleanup_state:
	ab_cleanup_state(sc);

	/* record state change */
	ab_sm_record_state_change(prev_state, sc->curr_chip_substate_id, sc);
	trace_ab_state_change(sc->curr_chip_substate_id);

	mutex_lock(&sc->async_fifo_lock);
	if (sc->async_entries) {
		kfifo_in(sc->async_entries,
			&sc->curr_chip_substate_id,
			sizeof(sc->curr_chip_substate_id));
	}
	mutex_unlock(&sc->async_fifo_lock);

	dev_err(sc->dev, "AB state reverted to %d\n",
		sc->curr_chip_substate_id);
	return ret;
}

static int state_change_task(void *ctx)
{
	struct ab_state_context *sc = (struct ab_state_context *)ctx;
	struct sched_param sp = {
		.sched_priority = MAX_RT_PRIO - 1,
	};
	int ret = sched_setscheduler(current, SCHED_FIFO, &sp);

	if (ret)
		dev_warn(sc->dev,
			"Unable to set FIFO scheduling of state change task (%d)\n",
			ret);

	while (!kthread_should_stop()) {
		ret = wait_for_completion_interruptible(
				&sc->request_state_change_comp);
		reinit_completion(&sc->request_state_change_comp);

		if (kthread_should_stop())
			return 0;

		if (ret)
			continue;

		mutex_lock(&sc->state_transitioning_lock);
		sc->change_ret = ab_sm_update_chip_state(sc);
		mutex_unlock(&sc->state_transitioning_lock);

		complete_all(&sc->transition_comp);
		complete_all(&sc->notify_comp);
	}

	return 0;
}

/* Caller must hold sc->set_state_lock */
static int _ab_sm_set_state(struct ab_state_context *sc,
		u32 dest_chip_substate_id)
{
	int ret;
	struct chip_to_block_map *map =
		ab_sm_get_block_map(sc, dest_chip_substate_id);

	if (!is_valid_transition(sc, sc->curr_chip_substate_id,
			dest_chip_substate_id) ||
			!map) {
		dev_err(sc->dev,
				"%s: invalid state change, current %u, requested %u\n",
				__func__, sc->curr_chip_substate_id,
				dest_chip_substate_id);
		return -EINVAL;
	}

	mutex_lock(&sc->state_transitioning_lock);
	sc->dest_chip_substate_id = dest_chip_substate_id;
	mutex_unlock(&sc->state_transitioning_lock);

	reinit_completion(&sc->transition_comp);
	complete_all(&sc->request_state_change_comp);

	ret = wait_for_completion_timeout(
			&sc->transition_comp,
			msecs_to_jiffies(AB_MAX_TRANSITION_TIME_MS));
	if (ret == 0) {
		dev_info(sc->dev, "State change timed out\n");
		ret = -EAGAIN;
	} else {
		/* completion finished before timeout */
		mutex_lock(&sc->state_transitioning_lock);
		ret = sc->change_ret;
		mutex_unlock(&sc->state_transitioning_lock);
	}

	return ret;
}

/* TODO (b/127500645): Remove once old mappings are completely deprecated */
int ab_sm_map_state(u32 old_mapping, u32 *new_mapping)
{
	static const u32 remap_table[86] = {
		[0]  = CHIP_STATE_400,
		[1]  = CHIP_STATE_401,
		[2]  = CHIP_STATE_402,
		[3]  = CHIP_STATE_403,
		[4]  = CHIP_STATE_404,
		[5]  = CHIP_STATE_405,
		[6]  = CHIP_STATE_406,
		[7]  = CHIP_STATE_407,
		[8]  = CHIP_STATE_408,
		[9]  = CHIP_STATE_409,
		[10] = CHIP_STATE_500,
		[11] = CHIP_STATE_501,
		[12] = CHIP_STATE_502,
		[13] = CHIP_STATE_503,
		[14] = CHIP_STATE_504,
		[15] = CHIP_STATE_505,
		[20] = CHIP_STATE_600,
		[21] = CHIP_STATE_601,
		[22] = CHIP_STATE_602,
		[23] = CHIP_STATE_603,
		[24] = CHIP_STATE_604,
		[25] = CHIP_STATE_605,
		[30] = CHIP_STATE_300,
		[40] = CHIP_STATE_200,
		[50] = CHIP_STATE_100,
		[60] = CHIP_STATE_0,
		[70] = CHIP_STATE_700,
		[71] = CHIP_STATE_701,
		[72] = CHIP_STATE_702,
		[73] = CHIP_STATE_703,
		[74] = CHIP_STATE_704,
		[75] = CHIP_STATE_705,
		[80] = CHIP_STATE_800,
		[81] = CHIP_STATE_801,
		[82] = CHIP_STATE_802,
		[83] = CHIP_STATE_803,
		[84] = CHIP_STATE_804,
		[85] = CHIP_STATE_805,
	};

	if (old_mapping >= ARRAY_SIZE(remap_table))
		return -EINVAL;

	*new_mapping = remap_table[old_mapping];

	/* Checking case where input was eg. 34
	 * Only valid output of 0 is input of 60
	 */
	if (*new_mapping == 0 && old_mapping != 60)
		return -EINVAL;

	return 0;

}

int ab_sm_unmap_state(u32 new_mapping, u32 *old_mapping)
{
	switch (new_mapping) {
	case CHIP_STATE_0:
		*old_mapping = 60;
		break;
	case CHIP_STATE_100:
		*old_mapping = 50;
		break;
	case CHIP_STATE_200:
		*old_mapping = 40;
		break;
	case CHIP_STATE_300:
		*old_mapping = 30;
		break;
	case CHIP_STATE_400 ... CHIP_STATE_409:
		*old_mapping = (new_mapping - 400);
		break;
	case CHIP_STATE_500 ... CHIP_STATE_505:
		*old_mapping = (new_mapping - 490);
		break;
	case CHIP_STATE_600 ... CHIP_STATE_605:
		*old_mapping = (new_mapping - 580);
		break;
	case CHIP_STATE_700 ... CHIP_STATE_705:
		*old_mapping = (new_mapping - 630);
		break;
	case CHIP_STATE_800 ... CHIP_STATE_805:
		*old_mapping = (new_mapping - 720);
		break;
	default:
		pr_err("couldn't unmap %d\n", new_mapping);
		return -EINVAL;
	}

	return 0;
}

int ab_sm_set_state(struct ab_state_context *sc,
		u32 dest_chip_substate_id, bool mapped)
{
	int ret;
	u32 mapped_val;

	mutex_lock(&sc->set_state_lock);
	if (!mapped) {
		ret = ab_sm_map_state(dest_chip_substate_id, &mapped_val);
		if (ret) {
			mutex_unlock(&sc->set_state_lock);
			return ret;
		}
	} else {
		mapped_val = dest_chip_substate_id;
	}

	ret = _ab_sm_set_state(sc, mapped_val);
	mutex_unlock(&sc->set_state_lock);

	return ret;
}
EXPORT_SYMBOL(ab_sm_set_state);

u32 ab_sm_get_state(struct ab_state_context *sc, bool mapped)
{
	u32 state;

	mutex_lock(&sc->state_transitioning_lock);
	if (!mapped)
		ab_sm_unmap_state(sc->curr_chip_substate_id, &state);
	else
		state = sc->curr_chip_substate_id;

	mutex_unlock(&sc->state_transitioning_lock);
	return state;
}
EXPORT_SYMBOL(ab_sm_get_state);

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

/*
 * set_ipu_tpu_clk_freq_table - Sets index of clk frequency table for A0/B0.
 */
static void set_ipu_tpu_clk_freq_table(struct ab_state_context *sc,
		enum ab_chip_id chip_id)
{
	struct block_property *prop;
	int blk_idx = 0;

	if (chip_id == CHIP_ID_UNKNOWN) {
		dev_err(sc->dev, "%s called with CHIP_ID_UNKNOWN\n", __func__);
		return;
	}

	while (blk_idx < sc->blocks[BLK_IPU].nr_block_states) {
		prop = &sc->blocks[BLK_IPU].prop_table[blk_idx];
		prop->clk_frequency = blk_ipu_clk_tbl[blk_idx][chip_id];
		blk_idx++;
	}

	blk_idx = 0;
	while (blk_idx < sc->blocks[BLK_TPU].nr_block_states) {
		prop = &sc->blocks[BLK_TPU].prop_table[blk_idx];
		prop->clk_frequency = blk_tpu_clk_tbl[blk_idx][chip_id];
		blk_idx++;
	}
}

enum ab_chip_id ab_get_chip_id(struct ab_state_context *sc)
{
	enum ab_chip_id val;
	int ret;

	if (sc->chip_id == CHIP_ID_UNKNOWN) {
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->get_chip_id(sc->mfd_ops->ctx, &val);
		mutex_unlock(&sc->mfd_lock);

		if (ret < 0) {
			set_ipu_tpu_clk_freq_table(sc, CHIP_ID_UNKNOWN);
			return CHIP_ID_UNKNOWN;
		}

		sc->chip_id = val;
		set_ipu_tpu_clk_freq_table(sc, sc->chip_id);
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

/*
 * Calls host platform dependent APIs to enumerate PCIe bus.
 *
 * Returns 0 on success, and ab_ctx->pcie_dev is assigned a pointer to
 *         a pci device.
 * Returns non-zero on error.
 */
int ab_sm_enumerate_pcie(struct ab_state_context *ab_ctx)
{
	int ret;
	struct pci_bus *pbus = NULL;
	struct pci_dev *pdev = NULL;

	ret = msm_pcie_enumerate(1);
	if (ret) {
		dev_err(ab_ctx->dev, "PCIe enumeration failed (%d)\n", ret);
		return ret;
	}

	pbus = pci_find_bus(1, 1);
	if (!pbus) {
		dev_err(ab_ctx->dev, "Cannot locate PCIe bus\n");
		return -ENODEV;
	}

	pdev = pbus->self;
	while (!pci_is_root_bus(pbus)) {
		pdev = pbus->self;
		pbus = pbus->self->bus;
	}
	ab_ctx->pcie_dev = pdev;
	ab_sm_setup_pcie_event(ab_ctx);

	return 0;
}

/*
 * Calls host platform dependent APIs to resume PCIe link after PCIe
 * has been enumerated once.
 *
 * Returns 0 on success, non-zero on error.
 */
int ab_sm_enable_pcie(struct ab_state_context *ab_ctx)
{
	int ret;

	ret = msm_pcie_pm_control(MSM_PCIE_RESUME, 0,
				  ab_ctx->pcie_dev, NULL,
				  MSM_PCIE_CONFIG_NO_CFG_RESTORE);
	if (ret) {
		dev_err(ab_ctx->dev, "PCIe failed to enable link (%d)\n", ret);
		return ret;
	}

	ret = msm_pcie_recover_config(ab_ctx->pcie_dev);
	if (ret) {
		dev_err(ab_ctx->dev, "PCIe failed to recover config (%d)\n",
			ret);
		return ret;
	}

	return 0;
}

/*
 * Calls host platform dependent APIs to suspend PCIe link.
 *
 * Returns 0 on success, non-zero on error.
 */
int ab_sm_disable_pcie(struct ab_state_context *ab_ctx)
{
	int ret;

	ret = msm_pcie_pm_control(MSM_PCIE_SUSPEND, 0,
				  ab_ctx->pcie_dev, NULL,
				  MSM_PCIE_CONFIG_NO_CFG_RESTORE);
	if (ret) {
		dev_err(ab_ctx->dev, "PCIe failed to disable link\n");
		return ret;
	}

	return 0;
}

static void ab_sm_shutdown_work(struct work_struct *data)
{
	struct ab_state_context *sc =
			container_of(data,
				     struct ab_state_context,
				     shutdown_work);
	enum chip_state prev_state;

	/*
	 * When this work is scheduled, cleanup should have been marked
	 * as in progress, otherwise it indicates a programming error.
	 */
	WARN_ON(atomic_read(&sc->is_cleanup_in_progress) ==
		AB_SM_CLEANUP_NOT_IN_PROGRESS);

	mutex_lock(&sc->state_transitioning_lock);
	prev_state = sc->curr_chip_substate_id;
	if (prev_state == CHIP_STATE_0) {
		/* No need to emergency shutdown if already powered off */
		dev_info(sc->dev, "already shutdown; skip emergency shutdown work\n");
		mutex_unlock(&sc->state_transitioning_lock);
		return;
	}

	dev_warn(sc->dev, "begin emergency shutdown work\n");

	/* Force reset el2_mode (b/122619299#comment10) */
	sc->el2_mode = false;

	ab_cleanup_state_linkdown(sc);

	/* record state change */
	ab_sm_record_state_change(prev_state, sc->curr_chip_substate_id, sc);
	trace_ab_state_change(sc->curr_chip_substate_id);

	mutex_lock(&sc->async_fifo_lock);
	if (sc->async_entries) {
		kfifo_in(sc->async_entries,
			&sc->curr_chip_substate_id,
			sizeof(sc->curr_chip_substate_id));
	}
	mutex_unlock(&sc->async_fifo_lock);

	/*
	 * Intentionally skip complete_all(&sc->transition_comp) because
	 * this is not initiated by a state transition request.
	 */
	complete_all(&sc->notify_comp);
	mutex_unlock(&sc->state_transitioning_lock);

	/* This work is responsible for marking cleanup as completed. */
	WARN_ON(atomic_cmpxchg(&sc->is_cleanup_in_progress,
			       AB_SM_CLEANUP_IN_PROGRESS,
			       AB_SM_CLEANUP_NOT_IN_PROGRESS) ==
			       AB_SM_CLEANUP_NOT_IN_PROGRESS);
}

#if IS_ENABLED(CONFIG_PCI_MSM)
static void ab_sm_pcie_linkdown_cb(struct msm_pcie_notify *notify)
{
	struct ab_state_context *sc = notify->data;

	switch (notify->event) {
	case MSM_PCIE_EVENT_LINKDOWN:
		dev_err(sc->dev, "received PCIe linkdown event\n");
		if (atomic_cmpxchg(&sc->is_cleanup_in_progress,
				   AB_SM_CLEANUP_NOT_IN_PROGRESS,
				   AB_SM_CLEANUP_IN_PROGRESS) ==
				   AB_SM_CLEANUP_IN_PROGRESS) {
			dev_warn(sc->dev,
				 "%s: cleanup in progress; don't schedule work\n",
				 __func__);
			break;
		}
		dev_info(sc->dev, "%s: schedule a shutdown work\n", __func__);
		schedule_work(&sc->shutdown_work);
		sysfs_notify(&sc->dev->kobj, NULL, "error_event");
		break;
	default:
		dev_warn(sc->dev,
			 "%s: received invalid pcie event (%d)\n",
			 __func__, notify->event);
		break;
	}
}

int ab_sm_setup_pcie_event(struct ab_state_context *sc)
{
	int ret;

	sc->pcie_link_event.events = MSM_PCIE_EVENT_LINKDOWN;
	sc->pcie_link_event.user = sc->pcie_dev;
	sc->pcie_link_event.callback = ab_sm_pcie_linkdown_cb;
	sc->pcie_link_event.notify.data = sc;

	ret = msm_pcie_register_event(&sc->pcie_link_event);
	if (ret) {
		dev_err(sc->dev,
			"msm_pcie_register_event failed (%d)\n", ret);
		return ret;
	}

	return 0;
}
#endif /* CONFIG_PCI_MSM */

static int ab_sm_regulator_listener(struct notifier_block *nb,
				    unsigned long event, void *cookie)
{
	struct ab_state_context *sc =
			container_of(nb,
				     struct ab_state_context,
				     regulator_nb);

	dev_dbg(sc->dev, "received regulator event 0x%lx\n", event);

	if (event & REGULATOR_EVENT_FAIL) {
		dev_err(sc->dev, "received regulator failure 0x%lx\n", event);
		if (atomic_cmpxchg(&sc->is_cleanup_in_progress,
				   AB_SM_CLEANUP_NOT_IN_PROGRESS,
				   AB_SM_CLEANUP_IN_PROGRESS) ==
				   AB_SM_CLEANUP_IN_PROGRESS) {
			dev_warn(sc->dev,
				 "%s: cleanup in progress; don't schedule work\n",
				 __func__);
			return NOTIFY_DONE; /* Don't care */
		}
		sc->asv_info.last_volt = 0;
		dev_info(sc->dev, "%s: schedule a shutdown work\n", __func__);
		schedule_work(&sc->shutdown_work);
		sysfs_notify(&sc->dev->kobj, NULL, "error_event");
		return NOTIFY_OK;
	}

	return NOTIFY_DONE; /* Don't care */
}

static long ab_sm_async_notify(struct ab_sm_misc_session *sess,
		unsigned long arg, bool mapped)
{
	int ret;
	int chip_state;
	u32 unmapped_val;
	struct ab_state_context *sc;

	mutex_lock(&sess->sc->async_fifo_lock);
	sc = sess->sc;
	sc->async_entries = &sess->async_entries;

	while (kfifo_is_empty(sc->async_entries)) {
		mutex_unlock(&sc->async_fifo_lock);
		if (sess->first_entry) {
			sess->first_entry = false;
			if (!mapped) {
				ret = ab_sm_unmap_state(
						sc->curr_chip_substate_id,
						&unmapped_val);
				if (ret)
					return ret;
				if (copy_to_user((void __user *)arg,
						&unmapped_val,
						sizeof(unmapped_val)))
					return -EFAULT;
			} else if (copy_to_user((void __user *)arg,
					&sc->curr_chip_substate_id,
					sizeof(sc->curr_chip_substate_id))) {
				return -EFAULT;
			}

			reinit_completion(&sc->notify_comp);
			return 0;

		} else {
			ret = wait_for_completion_interruptible(
					&sc->notify_comp);
			if (ret < 0)
				return ret;

			mutex_lock(&sc->async_fifo_lock);
			if (sc->async_entries == NULL) {
				dev_warn(sc->dev,
					"Ioctl session closed during wait for notification");
				mutex_unlock(&sc->async_fifo_lock);
				return -ENODEV;
			}
		}
		reinit_completion(&sc->notify_comp);
	}

	kfifo_out(sc->async_entries, &chip_state, sizeof(chip_state));

	if (!mapped) {
		ret = ab_sm_unmap_state(chip_state, &unmapped_val);
		if (ret) {
			mutex_unlock(&sc->async_fifo_lock);
			return ret;
		}

		if (copy_to_user((void __user *)arg,
					&unmapped_val, sizeof(unmapped_val))) {
			mutex_unlock(&sc->async_fifo_lock);
			return -EFAULT;
		}
	} else {
		if (copy_to_user((void __user *)arg,
					&chip_state, sizeof(chip_state))) {
			mutex_unlock(&sc->async_fifo_lock);
			return -EFAULT;
		}
	}

	sess->first_entry = false;
	mutex_unlock(&sc->async_fifo_lock);
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
	kfifo_alloc(&sess->async_entries,
		AB_KFIFO_ENTRY_SIZE * sizeof(int), GFP_KERNEL);

	fp->private_data = sess;

	return 0;
}

static int ab_sm_misc_release(struct inode *ip, struct file *fp)
{
	struct ab_sm_misc_session *sess = fp->private_data;
	struct ab_state_context *sc = sess->sc;

	complete_all(&sc->notify_comp);

	mutex_lock(&sc->async_fifo_lock);
	if (&sess->async_entries == sc->async_entries)
		sc->async_entries = NULL;
	kfree(sess);
	mutex_unlock(&sc->async_fifo_lock);
	return 0;
}

int ab_sm_enter_el2(struct ab_state_context *sc)
{
	int ret = 0;
	bool thermal_disabled = false;

	mutex_lock(&sc->state_transitioning_lock);
	if (sc->throttle_state_id == THROTTLE_NOCOMPUTE) {
		ret = -EBUSY;
	} else {
		/*
		 * Disable thermal in state transitioning lock to make sure the
		 * state change to be applied is what we are waiting for.
		 */
		ab_thermal_disable(sc->thermal);
		thermal_disabled = true;
	}
	mutex_unlock(&sc->state_transitioning_lock);
	if (ret < 0)
		goto exit;

	/*
	 * Disable thermal may cause a state change. Wait for state change
	 * to be applied if needed.
	 */
	reinit_completion(&sc->transition_comp);
	complete_all(&sc->request_state_change_comp);
	ret = wait_for_completion_timeout(&sc->transition_comp,
			msecs_to_jiffies(AB_MAX_TRANSITION_TIME_MS));
	if (ret == 0) {
		dev_warn(sc->dev, "State change timed out\n");
		ret = -ETIMEDOUT;
		goto exit;
	}

	mutex_lock(&sc->state_transitioning_lock);

	/* Ensure PCIe is accessible */
	if (sc->el2_in_secure_context) {
		dev_warn(sc->dev,
			 "Can't enter EL2 while PCIe is mapped to the secure context\n");
		mutex_unlock(&sc->state_transitioning_lock);
		ret = -EINVAL;
		goto exit;
	}

	if (!sc->el2_mode) {
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->enter_el2(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);

		if (!ret)
			sc->el2_mode = true;
	} else {
		ret = -EINVAL;
		dev_warn(sc->dev, "Already in el2 mode\n");
	}
	mutex_unlock(&sc->state_transitioning_lock);

exit:
	if (ret && thermal_disabled)
		ab_thermal_enable(sc->thermal);
	return ret;
}

int ab_sm_exit_el2(struct ab_state_context *sc)
{
	int ret;

	mutex_lock(&sc->state_transitioning_lock);

	/* Ensure PCIe is accessible */
	if (sc->el2_in_secure_context) {
		dev_warn(sc->dev,
			 "Can't exit EL2 while PCIe is mapped to the secure context\n");
		mutex_unlock(&sc->state_transitioning_lock);
		return -EINVAL;
	}

	if (sc->el2_mode) {
		mutex_lock(&sc->mfd_lock);
		ret = sc->mfd_ops->exit_el2(sc->mfd_ops->ctx);
		mutex_unlock(&sc->mfd_lock);

		if (!ret) {
			sc->el2_mode = false;
			/*
			 * Enable thermal after all pcie subscribers getting
			 * enabled.
			 */
			ab_thermal_enable(sc->thermal);
		}
	} else {
		ret = -EINVAL;
		dev_warn(sc->dev, "Not in el2 mode\n");
	}
	mutex_unlock(&sc->state_transitioning_lock);

	return ret;
}

/* Caller must hold sc->throttle_ready_lock */
static inline void __complete_throttle_nocompute_ready(
		struct ab_state_context *sc)
{
	/* Allow thermal throttling to continue */
	sc->throttle_nocomp_waiting = false;
	reinit_completion(&sc->throttle_nocompute_event);
	complete_all(&sc->throttle_nocompute_ready);
}

static void __throttle_nocompute_notify(struct ab_state_context *sc)
{
	unsigned long ret;

	sc->req_thermal_listeners = sc->curr_thermal_listeners;

	/* If there is a userspace listener, notify them of pending change */
	if (sc->req_thermal_listeners > 0) {
		sc->throttle_nocomp_waiting = true;
		reinit_completion(&sc->throttle_nocompute_ready);
		sc->curr_thermal_listeners = 0;
		complete_all(&sc->throttle_nocompute_event);

		mutex_unlock(&sc->throttle_ready_lock);

		/* Wait for all listeners to be ready */
		ret = wait_for_completion_timeout(&sc->throttle_nocompute_ready,
				msecs_to_jiffies(AB_THROTTLE_TIMEOUT_MS));
		if (!ret) {
			dev_warn(sc->dev,
				"Timeout while waiting for userspace to be ready for nocompute\n");
			/* In case of timeout, reset things */
			mutex_lock(&sc->throttle_ready_lock);
			if (sc->throttle_nocomp_waiting)
				__complete_throttle_nocompute_ready(sc);
			mutex_unlock(&sc->throttle_ready_lock);
		}
	} else {
		mutex_unlock(&sc->throttle_ready_lock);
	}
}

static int __throttle_nocompute_wait_for_user(struct ab_state_context *sc)
{
	int ret;

	mutex_lock(&sc->throttle_ready_lock);
	sc->curr_thermal_listeners++;
	if (sc->throttle_nocomp_waiting) {
		if (sc->curr_thermal_listeners >= sc->req_thermal_listeners) {
			__complete_throttle_nocompute_ready(sc);
			mutex_unlock(&sc->throttle_ready_lock);
		} else {
			/* Ensure all listeners get the chance to wake up from
			 * throttle_nocompute_event before listening for the
			 * next event
			 */
			mutex_unlock(&sc->throttle_ready_lock);
			ret = wait_for_completion_timeout(
				&sc->throttle_nocompute_ready,
				msecs_to_jiffies(AB_THROTTLE_TIMEOUT_MS));
			/* In case of timeout, reset things */
			if (!ret) {
				mutex_lock(&sc->throttle_ready_lock);
				if (sc->throttle_nocomp_waiting)
					__complete_throttle_nocompute_ready(sc);
				mutex_unlock(&sc->throttle_ready_lock);
			}

		}
	} else {
		mutex_unlock(&sc->throttle_ready_lock);
	}

	/* Wait for next throttle to no compute event */
	ret = wait_for_completion_interruptible(&sc->throttle_nocompute_event);
	mutex_lock(&sc->throttle_ready_lock);
	if (ret == -ERESTARTSYS) {
		if (sc->throttle_nocomp_waiting) {
			sc->req_thermal_listeners--;
			if (sc->curr_thermal_listeners >=
					sc->req_thermal_listeners)
				__complete_throttle_nocompute_ready(sc);
		} else {
			sc->curr_thermal_listeners--;
		}
	}

	ret = sc->going_to_comp_ready;
	mutex_unlock(&sc->throttle_ready_lock);

	return ret;
}

static void ab_sm_thermal_throttle_state_updated(
		enum throttle_state throttle_state_id, void *op_data);

#ifdef CONFIG_AIRBRUSH_SM_DEBUG_IOCTLS
static long ab_sm_misc_ioctl_debug(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	long ret;
	struct ab_sm_misc_session *sess = fp->private_data;
	struct ab_state_context *sc = sess->sc;
	struct new_block_props props;
	int aspm_val;
	u32 clk_frequency;

	switch (cmd) {
	case AB_SM_SET_IPU_FREQUENCY:
		clk_frequency = (u32)arg;
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		ret = sc->clk_ops->ipu_set_rate(sc->clk_ops->ctx,
			sc->blocks[BLK_IPU].current_state->clk_frequency,
			arg);
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		break;

	case AB_SM_SET_TPU_FREQUENCY:
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		ret = sc->clk_ops->tpu_set_rate(sc->clk_ops->ctx,
			sc->blocks[BLK_TPU].current_state->clk_frequency,
			arg);
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		if (ret == arg)
			ret = 0;
		break;

	case AB_SM_SET_DDR_FREQUENCY:
	case AB_SM_SET_PCIE_FREQUENCY:
		dev_info(sc->dev,
			"%s: Unimplemented ioctl cmd 0x%X\n", __func__, cmd);
		break;

	case AB_SM_SET_AON_FREQUENCY:
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		ret = sc->clk_ops->aon_set_rate(sc->clk_ops->ctx,
			sc->blocks[BLK_AON].current_state->clk_frequency,
			arg);
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		if (ret == arg)
			ret = 0;
		break;

	case AB_SM_SET_IPU_STATE:
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		if (arg == 0)
			ret = sc->pmu_ops->pmu_ipu_sleep(sc->pmu_ops->ctx);
		else
			ret = sc->pmu_ops->pmu_ipu_resume(sc->pmu_ops->ctx);
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		break;

	case AB_SM_SET_TPU_STATE:
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		if (arg == 0)
			ret = sc->pmu_ops->pmu_tpu_sleep(sc->pmu_ops->ctx);
		else
			ret = sc->pmu_ops->pmu_tpu_resume(sc->pmu_ops->ctx);
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		break;

	case AB_SM_SET_DDR_STATE:
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		if (sc->curr_chip_substate_id < CHIP_STATE_200) {
			/* PCIe link is down. Return error code. */
			ret = -ENODEV;
		} else if (arg == 0) {
			ret = sc->dram_ops->sref_enter(sc->dram_ops->ctx);
			/* divide pll_aon_clk by 4*/
			/* TODO(b/123695099): do this via ops struct */
			ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x3);
			/* divide aon_pclk by 16*/
			/* TODO(b/123695099): do this via ops struct */
			ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK, 0xf);
		} else {
			/* divide aon_pclk by 4*/
			/* TODO(b/123695099): do this via ops struct */
			ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK, 0x3);
			/* divide pll_aon_clk by 1*/
			/* TODO(b/123695099): do this via ops struct */
			ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x0);
			ret = sc->dram_ops->sref_exit(sc->dram_ops->ctx);
		}
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		break;

	case AB_SM_SET_PCIE_STATE:
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		if (sc->curr_chip_substate_id < CHIP_STATE_200) {
			/* PCIe link is down. Return error code. */
			ret = -ENODEV;
		} else {
			switch (arg) {
			case 0:
				aspm_val = ASPM_L12;
				break;
			case 1:
				aspm_val = ASPM_L11;
				break;
			case 2:
				aspm_val = ASPM_L10;
				break;
			case 3:
				aspm_val = ASPM_L0s;
				break;
			default:
				aspm_val = NOASPM;
				break;
			}
			/* TODO(b/123695099): do this via ops struct */
			ret = 0;
			abc_pcie_set_linkstate(aspm_val);
		}
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		break;

	case AB_SM_SET_PCIE_L1SS_DELAY:
		mutex_lock(&sc->state_transitioning_lock);
		mutex_lock(&sc->op_lock);
		abc_set_l1_entry_delay(arg);
		mutex_unlock(&sc->op_lock);
		mutex_unlock(&sc->state_transitioning_lock);
		ret = 0;
		break;
	case AB_SM_UPDATE_IPU_STATE_PROPERTIES:
		if (copy_from_user(&props, (void __user *)arg,
					sizeof(struct new_block_props))) {
			return -EINVAL;
		}
		ret = ab_update_block_prop_table(&props, BLK_IPU, sc);
		dev_warn(sc->dev,
			"IPU property table has been changed! Airbrush may behave unexpectedly.\n");
		break;

	case AB_SM_UPDATE_TPU_STATE_PROPERTIES:
		if (copy_from_user(&props, (void __user *)arg,
					sizeof(struct new_block_props))) {
			return -EINVAL;
		}
		ret = ab_update_block_prop_table(&props, BLK_TPU, sc);
		dev_warn(sc->dev,
			"TPU property table has been changed! Airbrush may behave unexpectedly.\n");
		break;

	case AB_SM_UPDATE_DRAM_STATE_PROPERTIES:
		if (copy_from_user(&props, (void __user *)arg,
					sizeof(struct new_block_props))) {
			return -EINVAL;
		}
		ret = ab_update_block_prop_table(&props, DRAM, sc);
		dev_warn(sc->dev,
			"DRAM property table has been changed! Airbrush may behave unexpectedly.\n");
		break;

	case AB_SM_UPDATE_MIF_STATE_PROPERTIES:
		if (copy_from_user(&props, (void __user *)arg,
					sizeof(struct new_block_props))) {
			return -EINVAL;
		}
		ret = ab_update_block_prop_table(&props, BLK_MIF, sc);
		dev_warn(sc->dev,
			"MIF property table has been changed! Airbrush may behave unexpectedly.\n");
		break;

	case AB_SM_UPDATE_FSYS_STATE_PROPERTIES:
		if (copy_from_user(&props, (void __user *)arg,
					sizeof(struct new_block_props))) {
			return -EINVAL;
		}
		ret = ab_update_block_prop_table(&props, BLK_FSYS, sc);
		dev_warn(sc->dev,
			"FSYS property table has been changed! Airbrush may behave unexpectedly.\n");
		break;

	case AB_SM_UPDATE_AON_STATE_PROPERTIES:
		if (copy_from_user(&props, (void __user *)arg,
					sizeof(struct new_block_props))) {
			return -EINVAL;
		}
		ret = ab_update_block_prop_table(&props, BLK_AON, sc);
		dev_warn(sc->dev,
			"AON property table has been changed! Airbrush may behave unexpectedly.\n");
		break;

	case AB_SM_SET_THROTTLE_LEVEL:
		ab_sm_thermal_throttle_state_updated(arg, sc);
		ret = 0;
		break;

	case AB_SM_ENABLE_THERMAL:
		if (arg)
			ab_thermal_enable(sc->thermal);
		else
			ab_thermal_disable(sc->thermal);
		ret = 0;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}
#endif /* CONFIG_AIRBRUSH_SM_DEBUG_IOCTLS */

static long ab_sm_misc_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	long ret;
	struct ab_sm_misc_session *sess = fp->private_data;
	struct ab_state_context *sc = sess->sc;
	int state;

#ifdef CONFIG_AIRBRUSH_SM_DEBUG_IOCTLS
	ret = ab_sm_misc_ioctl_debug(fp, cmd, arg);
	if (ret != -EINVAL)
		return ret;
#endif /* CONFIG_AIRBRUSH_SM_DEBUG_IOCTLS */

	switch (cmd) {
	case AB_SM_ASYNC_NOTIFY:
		if (!atomic_cmpxchg(&sc->async_in_use, 0, 1)) {
			ret = ab_sm_async_notify(sess, arg, false);
			atomic_set(&sc->async_in_use, 0);
		} else {
			dev_warn(sc->dev, "AB_SM_ASYNC_NOTIFY is in use\n");
			ret = -EBUSY;
		}
		break;

	case AB_SM_SET_STATE:
		ret = ab_sm_set_state(sc, arg, false);
		break;

	case AB_SM_GET_STATE:
		state = ab_sm_get_state(sess->sc, false);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			return -EFAULT;
		ret = 0;
		break;

	case AB_SM_MAPPED_ASYNC_NOTIFY:
		if (!atomic_cmpxchg(&sc->async_in_use, 0, 1)) {
			ret = ab_sm_async_notify(sess, arg, true);
			atomic_set(&sc->async_in_use, 0);
		} else {
			dev_warn(sc->dev, "AB_SM_ASYNC_NOTIFY is in use\n");
			ret = -EBUSY;
		}
		break;

	case AB_SM_MAPPED_SET_STATE:
		ret = ab_sm_set_state(sc, arg, true);
		break;

	case AB_SM_MAPPED_GET_STATE:
		state = ab_sm_get_state(sess->sc, true);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			return -EFAULT;
		ret = 0;
		break;

	case AB_SM_ENTER_EL2:
		ret = ab_sm_enter_el2(sc);
		break;

	case AB_SM_EXIT_EL2:
		ret = ab_sm_exit_el2(sc);
		break;

	case AB_SM_GET_EL2_MODE:
		mutex_lock(&sc->state_transitioning_lock);
		if (copy_to_user((void __user *)arg,
				&sc->el2_mode, sizeof(sc->el2_mode))) {
			mutex_unlock(&sc->state_transitioning_lock);
			return -EFAULT;
		}
		mutex_unlock(&sc->state_transitioning_lock);
		break;

	case AB_SM_COMPUTE_READY_NOTIFY:
		state = __throttle_nocompute_wait_for_user(sc);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			return -EFAULT;
		ret = 0;
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
	struct ab_state_context *sc = op_data;

	/* In THROTTLE_NOCOMPUTE case, give userspace a
	 * chance to react to AB resource being removed
	 */
	if (throttle_state_id == THROTTLE_NOCOMPUTE &&
			sc->throttle_state_id != THROTTLE_NOCOMPUTE) {
		mutex_lock(&sc->throttle_ready_lock);
		sc->going_to_comp_ready = false;
		__throttle_nocompute_notify(sc);
	} else if (throttle_state_id != THROTTLE_NOCOMPUTE &&
			sc->throttle_state_id == THROTTLE_NOCOMPUTE) {
		/* Update sc->throttle_state_id immediately in case
		 * userspace attempts to immediately change state while
		 * we are still in __throttle_nocompute_notify
		 */
		mutex_lock(&sc->throttle_ready_lock);
		sc->going_to_comp_ready = true;
		sc->throttle_state_id = throttle_state_id;
		__throttle_nocompute_notify(sc);
	}

	sc->throttle_state_id = throttle_state_id;
	dev_info(sc->dev, "Throttle state updated to %lu", throttle_state_id);

	if (!sc->cold_boot)
		complete_all(&sc->request_state_change_comp);
}

static const struct ab_thermal_ops ab_sm_thermal_ops = {
	.throttle_state_updated = ab_sm_thermal_throttle_state_updated,
};

static void ab_sm_state_stats_init(struct ab_state_context *sc)
{
	enum stat_state curr_stat_state =
		ab_chip_state_to_stat_state(sc->curr_chip_substate_id);

	sc->state_stats[curr_stat_state].counter++;
	sc->state_stats[curr_stat_state].last_entry = ktime_get_boottime();
}

static int ab_sm_el2_notification_listener(struct notifier_block *nb,
					   unsigned long code, void *data)
{
	struct ab_state_context *sc =
		container_of(nb, struct ab_state_context, el2_notif_nb);

	switch (code) {
	case SUBSYS_BEFORE_POWERUP:
		/* PCIe is getting mapped to the secure context */
		mutex_lock(&sc->state_transitioning_lock);
		sc->el2_in_secure_context = true;
		mutex_unlock(&sc->state_transitioning_lock);
		return NOTIFY_OK;

	case SUBSYS_AFTER_SHUTDOWN:
	case SUBSYS_POWERUP_FAILURE:
		/* PCIe got mapped back to EL1 context */
		mutex_lock(&sc->state_transitioning_lock);
		sc->el2_in_secure_context = false;
		mutex_unlock(&sc->state_transitioning_lock);
		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static void ab_sm_el2_notif_init(struct work_struct *work)
{
	struct ab_state_context *sc = container_of(
		work, struct ab_state_context, el2_notif_init.work);
	void *notifier_handler;

	mutex_lock(&sc->el2_notif_init_lock);

	/* setup callback for EL2 context change */
	sc->el2_notif_nb.notifier_call = ab_sm_el2_notification_listener;
	notifier_handler =
		subsys_notif_register_notifier("faceauth", &sc->el2_notif_nb);

	/* retry if registration has failed */
	if (IS_ERR(notifier_handler)) {
		if (!sc->sm_exiting) {
			dev_info(sc->dev,
				 "Retry EL2 callback registration later\n");
			schedule_delayed_work(&sc->el2_notif_init,
					      msecs_to_jiffies(500));
		} else
			dev_info(sc->dev,
				 "SM exiting. EL2 registartion not needed\n");
	} else {
		sc->el2_notif_handle = notifier_handler;
		dev_info(sc->dev, "Successfully registered EL2 notification\n");
	}

	mutex_unlock(&sc->el2_notif_init_lock);
}

int ab_sm_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int error;
	int ret;

	ab_sm_ctx = devm_kzalloc(dev, sizeof(struct ab_state_context),
							GFP_KERNEL);
	if (ab_sm_ctx == NULL)
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

	ret = ab_get_pmic_resources(ab_sm_ctx);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(ab_sm_ctx->dev,
				"Failed to get PMIC resources (ret = %d)",
				ret);
		goto fail_pmic_resources;
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

	if (of_property_read_u32(np, "ddrcke-iso-clamp-wr",
				 &ab_sm_ctx->ddrcke_iso_clamp_wr))
		ab_sm_ctx->ddrcke_iso_clamp_wr = 0;

	if (of_property_read_u32(np, "otp-bypass", &ab_sm_ctx->otp_bypass)) {
		ab_sm_ctx->otp_bypass = 0;
		dev_dbg(dev,
			"otp-bypass property not found. Setting ab_sm_ctx->otp_bypass = 0 (default)\n");
	}

	/* Force alternate boot everytime Airbrush is powered up
	 * Required to bypass the loading of PCIe config from OTP (b/130528130)
	 */
	if (ab_sm_ctx->otp_bypass) {
		ab_sm_ctx->alternate_boot = 1;
		dev_warn(dev,
			 "OTP bypass needed. Override alternate_boot = 1\n");
	}

	/* Intialize the default state of each block for state manager */
	ab_sm_ctx->blocks[BLK_IPU] = (struct block){BLK_IPU,
			&ipu_property_table[0],
			ipu_property_table,
			ARRAY_SIZE(ipu_property_table), NULL, NULL};

	ab_sm_ctx->blocks[BLK_TPU] = (struct block){BLK_TPU,
			&tpu_property_table[0],
			tpu_property_table,
			ARRAY_SIZE(tpu_property_table), NULL, NULL};

	ab_sm_ctx->blocks[DRAM] = (struct block){DRAM,
			&dram_property_table[0],
			dram_property_table,
			ARRAY_SIZE(dram_property_table), NULL, NULL};

	ab_sm_ctx->blocks[BLK_MIF] = (struct block){BLK_MIF,
			&mif_property_table[0],
			mif_property_table,
			ARRAY_SIZE(mif_property_table), NULL, NULL};

	ab_sm_ctx->blocks[BLK_FSYS] = (struct block){BLK_FSYS,
			&fsys_property_table[0],
			fsys_property_table,
			ARRAY_SIZE(fsys_property_table), NULL, NULL};

	ab_sm_ctx->blocks[BLK_AON] = (struct block){BLK_AON,
			&aon_property_table[0],
			aon_property_table,
			ARRAY_SIZE(aon_property_table), NULL, NULL};

	/* intitialize the default chip state */
	ab_sm_ctx->chip_state_table = chip_state_map;
	ab_sm_ctx->nr_chip_states = ARRAY_SIZE(chip_state_map);
	ab_sm_ctx->dest_chip_substate_id = CHIP_STATE_0;
	ab_sm_ctx->curr_chip_substate_id = CHIP_STATE_0;

	mutex_init(&ab_sm_ctx->set_state_lock);
	mutex_init(&ab_sm_ctx->state_transitioning_lock);
	mutex_init(&ab_sm_ctx->async_fifo_lock);
	mutex_init(&ab_sm_ctx->op_lock);
	mutex_init(&ab_sm_ctx->mfd_lock);
	atomic_set(&ab_sm_ctx->clocks_registered, 0);
	atomic_set(&ab_sm_ctx->async_in_use, 0);
	init_completion(&ab_sm_ctx->request_state_change_comp);
	init_completion(&ab_sm_ctx->transition_comp);
	init_completion(&ab_sm_ctx->notify_comp);
	kfifo_alloc(&ab_sm_ctx->state_change_reqs,
		AB_KFIFO_ENTRY_SIZE * sizeof(struct ab_change_req), GFP_KERNEL);

	/* Assume chip_id to be B0 by default. */
	ab_sm_ctx->chip_id = CHIP_ID_B0;
	set_ipu_tpu_clk_freq_table(ab_sm_ctx, ab_sm_ctx->chip_id);

	ab_sm_ctx->cold_boot = true;
	ab_sm_ctx->el2_mode = false;

	/* initialize state stats */
	ab_sm_state_stats_init(ab_sm_ctx);

	/* Initialize stub ops */
	ab_sm_register_pmu_ops(&pmu_ops_stub);
	ab_sm_register_clk_ops(&clk_ops_stub);
	ab_sm_register_dram_ops(&dram_ops_stub);
	ab_sm_register_mfd_ops(&mfd_ops_stub);

	ab_sm_ctx->thermal = devm_ab_thermal_create(ab_sm_ctx->dev,
			&ab_sm_thermal_ops, ab_sm_ctx);
	if (IS_ERR(ab_sm_ctx->thermal))
		dev_warn(dev, "Failed to initialize thermal\n");
	ab_sm_ctx->throttle_state_id = THROTTLE_NONE;

	init_completion(&ab_sm_ctx->throttle_nocompute_event);
	init_completion(&ab_sm_ctx->throttle_nocompute_ready);
	ab_sm_ctx->curr_thermal_listeners = 0;
	mutex_init(&ab_sm_ctx->throttle_ready_lock);
	ab_sm_ctx->throttle_nocomp_waiting = false;

	ab_sm_ctx->state_change_task =
		kthread_run(&state_change_task, ab_sm_ctx, "ab-sm");

	ab_sm_create_debugfs(ab_sm_ctx);
	ab_sm_create_sysfs(ab_sm_ctx);

	atomic_set(&ab_sm_ctx->is_cleanup_in_progress,
		   AB_SM_CLEANUP_NOT_IN_PROGRESS);
	INIT_WORK(&ab_sm_ctx->shutdown_work, ab_sm_shutdown_work);
	BLOCKING_INIT_NOTIFIER_HEAD(&ab_sm_ctx->clk_subscribers);

	ab_sm_ctx->regulator_nb.notifier_call = ab_sm_regulator_listener;

	ab_sm_ctx->smps2_delay = SMPS2_DEFAULT_DELAY;
	ab_sm_ctx->ldo4_delay = LDO4_DEFAULT_DELAY;
	ab_sm_ctx->ldo5_delay = LDO5_DEFAULT_DELAY;
	ab_sm_ctx->s60_delay = S60_DEFAULT_DELAY;

	/* initialize device wakeup source */
	device_init_wakeup(ab_sm_ctx->dev, true);

	/* schedule the EL2 notification registration worker */
	INIT_DELAYED_WORK(&ab_sm_ctx->el2_notif_init, ab_sm_el2_notif_init);
	schedule_delayed_work(&ab_sm_ctx->el2_notif_init, 0);

	dev_info(ab_sm_ctx->dev, "%s: done\n", __func__);

	return 0;

fail_fw_patch_en:
fail_ab_ready:
fail_pmic_resources:
	misc_deregister(&ab_sm_ctx->misc_dev);
fail_misc_reg:
	devm_kfree(dev, (void *)ab_sm_ctx);
	ab_sm_ctx = NULL;
fail_mem_alloc:
	return ret;
}
EXPORT_SYMBOL(ab_sm_init);

void ab_sm_exit(struct platform_device *pdev)
{
	mutex_lock(&ab_sm_ctx->el2_notif_init_lock);
	ab_sm_ctx->sm_exiting = true;
	mutex_unlock(&ab_sm_ctx->el2_notif_init_lock);

	cancel_delayed_work_sync(&ab_sm_ctx->el2_notif_init);
	kthread_stop(ab_sm_ctx->state_change_task);
	complete_all(&ab_sm_ctx->request_state_change_comp);
	complete_all(&ab_sm_ctx->transition_comp);
	complete_all(&ab_sm_ctx->notify_comp);
	ab_sm_remove_sysfs(ab_sm_ctx);
	ab_sm_remove_debugfs(ab_sm_ctx);
	if (ab_sm_ctx->el2_notif_handle)
		subsys_notif_unregister_notifier(ab_sm_ctx->el2_notif_handle,
						 &ab_sm_ctx->el2_notif_nb);
}

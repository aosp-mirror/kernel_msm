/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Shaik Ameer Basha <shaik.ameer@samsung.com>
 *	Raman Kumar Banka (raman.k2@samsung.com)
 *
 * Airbrush State Manager Control driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>

#include <linux/airbrush-sm-ctrl.h>
#include "airbrush-regs.h"

int ab_interrupt_M0(int tar_dev);

static int chip_state_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;
	int ret;

	ret = ab_sm_set_state(sc, val);
	if (ret < 0)
		dev_err(sc->dev, "%s: State change failed, ret %d\n",
				__func__, ret);

	return ret;
}

static int chip_state_get(void *sc, u64 *val)
{
	*val = ab_sm_get_state((struct ab_state_context *)sc);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_chip_state, chip_state_get,
				chip_state_set, "%llu\n");

static int ab_sm_force_el2_set(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;
	int ret;

	mutex_lock(&sc->mfd_lock);

	sc->force_el2 = !!val;

	if (sc->force_el2)
		ret = sc->mfd_ops->enter_el2(sc->mfd_ops->ctx);
	else
		ret = sc->mfd_ops->exit_el2(sc->mfd_ops->ctx);

	mutex_unlock(&sc->mfd_lock);

	return ret;
}

static int ab_sm_force_el2_get(void *data, u64 *val)
{
	struct ab_state_context *sc = (struct ab_state_context *)data;

	*val = (uint64_t)sc->force_el2;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ab_sm_force_el2_fops, ab_sm_force_el2_get,
		ab_sm_force_el2_set, "%llu\n");

static int id_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->id;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_id, id_get, NULL, "%llu\n");

static int vrail_status_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->voltage_rail_status;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_vrail_status,
				vrail_status_get, NULL, "%llu\n");

static int logic_voltage_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->logic_voltage;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_logic_voltage,
				logic_voltage_get, NULL, "%llu\n");

static int clk_status_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->clk_status;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_clk_status, clk_status_get, NULL, "%llu\n");

static int clk_frequency_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->clk_frequency;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_clk_frequency,
				clk_frequency_get, NULL, "%llu\n");

static int num_powered_cores_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_powered_cores;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_powered_cores,
				num_powered_cores_get, NULL, "%llu\n");

static int num_computing_cores_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_computing_cores;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_computing_cores,
				num_computing_cores_get, NULL, "%llu\n");

static int num_powered_tiles_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_powered_tiles;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_powered_tiles,
				num_powered_tiles_get, NULL, "%llu\n");

static int data_rate_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->data_rate;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_data_rate, data_rate_get, NULL, "%llu\n");

static int ab_debugfs_m0_intr(void *data, u64 val)
{
	ab_interrupt_M0(0);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ab_m0_intr_fops, NULL, ab_debugfs_m0_intr, "%lli\n");

static int ab_debugfs_ddr_ctrl(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *) data;

	switch (val) {
	case 0:
		if (IS_HOST_DDR_INIT())
			ab_ddr_init(sc);
		break;
	case 1:
		ab_ddr_suspend(sc);
		break;
	case 2:
		ab_ddr_resume(sc);
		break;
	case 3:
		ab_ddr_selfrefresh_enter(sc);
		break;
	case 4:
		ab_ddr_selfrefresh_exit(sc);
		break;
	case 5:
		ab_ddr_setup(sc);
		break;
	case 6:
		ab_ddr_read_write_test(3);
		break;
	default:
		pr_err("ERROR!! Invalid DDR Control\n");
		break;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ab_ddr_ctrl_fops, NULL, ab_debugfs_ddr_ctrl, "%lli\n");

void create_block_debugfs(struct dentry *parent_dir, struct block *blk)
{
	struct dentry *d;

	d = debugfs_create_file("state_id", 0444, parent_dir,
			       blk, &fops_id);
	if (!d)
		goto err_out;

	d = debugfs_create_file("voltage_rail_status", 0444, parent_dir,
			       blk, &fops_vrail_status);
	if (!d)
		goto err_out;

	d = debugfs_create_file("logic_voltage", 0444, parent_dir,
			       blk, &fops_logic_voltage);
	if (!d)
		goto err_out;

	d = debugfs_create_file("clk_status", 0444, parent_dir,
			       blk, &fops_clk_status);
	if (!d)
		goto err_out;

	d = debugfs_create_file("clk_frequency", 0444, parent_dir,
			       blk, &fops_clk_frequency);
	if (!d)
		goto err_out;

	d = debugfs_create_file("num_powered_cores", 0444, parent_dir,
			       blk, &fops_num_powered_cores);
	if (!d)
		goto err_out;

	d = debugfs_create_file("num_computing_cores", 0444, parent_dir,
			       blk, &fops_num_computing_cores);
	if (!d)
		goto err_out;

	d = debugfs_create_file("num_powered_tiles", 0444, parent_dir,
			       blk, &fops_num_powered_tiles);
	if (!d)
		goto err_out;

	d = debugfs_create_file("data_rate", 0444, parent_dir,
			       blk, &fops_data_rate);
	if (!d)
		goto err_out;

	return;

err_out:
	pr_err("Some error occured, couldn't create debugfs entry for airbrush blocks\n");
}

void ab_sm_create_debugfs(struct ab_state_context *sc)
{
	struct dentry *d_chip, *d_force_el2, *d_block, *d;

	sc->d_entry = debugfs_create_dir("airbrush", NULL);
	if (!sc->d_entry) {
		dev_err(sc->dev, "failed to create debugfs entry for airbrush");
		goto err_out;
	}

	d = debugfs_create_file("m0_intr", 0666, sc->d_entry, sc,
					&ab_m0_intr_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("ab_ddr_ctrl", 0444, sc->d_entry, sc,
					&ab_ddr_ctrl_fops);
	if (!d)
		goto err_out;

	d_chip = debugfs_create_dir("airbrush_sm", sc->d_entry);
	if (!d_chip)
		goto err_out;

	d = debugfs_create_file("chip_state", 0666, d_chip, sc,
				&fops_chip_state);
	if (!d)
		goto err_out;

	d_force_el2 = debugfs_create_file("force_el2", 0666, sc->d_entry, sc,
				&ab_sm_force_el2_fops);
	if (!d_force_el2)
		goto err_out;

	d_block = debugfs_create_dir("ipu", d_chip);
	if (!d_block)
		goto err_out;
	create_block_debugfs(d_block, &(sc->blocks[BLK_IPU]));

	d_block = debugfs_create_dir("tpu", d_chip);
	if (!d_block)
		goto err_out;
	create_block_debugfs(d_block, &(sc->blocks[BLK_TPU]));

	d_block = debugfs_create_dir("dram", d_chip);
	if (!d_block)
		goto err_out;
	create_block_debugfs(d_block, &(sc->blocks[DRAM]));

	d_block = debugfs_create_dir("mif", d_chip);
	if (!d_block)
		goto err_out;
	create_block_debugfs(d_block, &(sc->blocks[BLK_MIF]));

	d_block = debugfs_create_dir("fsys", d_chip);
	if (!d_block)
		goto err_out;
	create_block_debugfs(d_block, &(sc->blocks[BLK_FSYS]));

	d_block = debugfs_create_dir("aon", d_chip);
	if (!d_block)
		goto err_out;
	create_block_debugfs(d_block, &(sc->blocks[BLK_AON]));

	return;

err_out:
	pr_err("Some error occured, couldn't create debugfs entry for ab_device states\n");
}

void ab_sm_remove_debugfs(struct ab_state_context *sc)
{
	debugfs_remove_recursive(sc->d_entry);
}

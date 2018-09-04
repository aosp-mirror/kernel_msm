/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha <shaik.ameer@samsung.com>
 * 	    Raman Kumar Banka (raman.k2@samsung.com)
 *
 * Airbrush State Manager Control driver..
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>

#include <linux/airbrush-sm-ctrl.h>

int ab_interrupt_M0(int tar_dev);

#ifdef CONFIG_DEBUGFS_AIRBRUSH

static int chip_state_set(void *sc, u64 val)
{
	if(ab_sm_set_state((struct ab_state_context *)sc, 0, val))
		pr_err("State change failed\n");
	return 0;
}

static int chip_state_get(void *sc, u64 *val)
{
	*val = ((struct ab_state_context *)sc)->chip_substate_id;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_chip_state, chip_state_get,
				chip_state_set, "%llu\n");

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
DEFINE_DEBUGFS_ATTRIBUTE(fops_vrail_status, vrail_status_get, NULL, "%llu\n");

static int logic_voltage_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->logic_voltage;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_logic_voltage, logic_voltage_get, NULL, "%llu\n");

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
DEFINE_DEBUGFS_ATTRIBUTE(fops_clk_frequency, clk_frequency_get, NULL, "%llu\n");

static int num_powered_cores_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_powered_cores;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_powered_cores, num_powered_cores_get, NULL, "%llu\n");

static int num_computing_cores_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_computing_cores;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_computing_cores, num_computing_cores_get, NULL, "%llu\n");

static int num_powered_tiles_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->num_powered_tiles;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_num_powered_tiles, num_powered_tiles_get, NULL, "%llu\n");

static int data_rate_get(void *blk_passed, u64 *val)
{
	struct block *blk = (struct block *)blk_passed;
	*val = blk->current_state->data_rate;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_data_rate, data_rate_get, NULL, "%llu\n");

static int ab_debugfs_boot(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *) data;

    switch (val) {
    case 0:
        ab_bootsequence(sc, 1);
        break;
    case 1:
        ab_interrupt_M0(0);
        break;
    case 2:
        gpiod_set_value(sc->fw_patch_en, __GPIO_ENABLE);
        break;
    default:
        pr_info("Unsupported value\n");
    }

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ab_bootsequence_fops, NULL, ab_debugfs_boot, "%lli\n");

static int ab_assign_resources(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *) data;

	ab_get_pmic_resources(sc);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ab_assign_resources_fops, NULL, ab_assign_resources, "%lli\n");

static int ab_debugfs_clk_register(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *) data;

	abc_clk_register(sc);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ab_clk_register_fops, NULL, ab_debugfs_clk_register, "%lli\n");


static int ab_debugfs_ddr_ctrl(void *data, u64 val)
{
	struct ab_state_context *sc = (struct ab_state_context *) data;

	switch (val) {
	case 0:
		ab_ddr_init(sc);
		break;
	case 1:
		ab_ddr_suspend(sc);
		break;
	case 2:
		ab_ddr_resume(sc);
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
	if(!d)
		goto err_out;

	d = debugfs_create_file("voltage_rail_status", 0444, parent_dir,
			       blk, &fops_vrail_status);
	if(!d)
		goto err_out;

	d = debugfs_create_file("logic_voltage", 0444, parent_dir,
			       blk, &fops_logic_voltage);
	if(!d)
		goto err_out;

	d = debugfs_create_file("clk_status", 0444, parent_dir,
			       blk, &fops_clk_status);
	if(!d)
		goto err_out;

	d = debugfs_create_file("clk_frequency", 0444, parent_dir,
			       blk, &fops_clk_frequency);
	if(!d)
		goto err_out;

	d = debugfs_create_file("num_powered_cores", 0444, parent_dir,
			       blk, &fops_num_powered_cores);
	if(!d)
		goto err_out;

	d = debugfs_create_file("num_computing_cores", 0444, parent_dir,
			       blk, &fops_num_computing_cores);
	if(!d)
		goto err_out;

	d = debugfs_create_file("num_powered_tiles", 0444, parent_dir,
			       blk, &fops_num_powered_tiles);
	if(!d)
		goto err_out;

	d = debugfs_create_file("data_rate", 0444, parent_dir,
			       blk, &fops_data_rate);
	if(!d)
		goto err_out;

	return;

err_out:
	pr_err("Some error occured, couldn't create debugfs entry for airbrush blocks\n");
}

void ab_sm_create_debugfs(struct ab_state_context *sc)
{
	struct dentry *d_chip, *d_block, *d;

	sc->d_entry = debugfs_create_dir("airbrush", NULL);
	if (!sc->d_entry) {
		dev_err(sc->dev, "failed to create debugfs entry for airbrush");
		goto err_out;
	}

	d = debugfs_create_file("ab_boot", 0444, sc->d_entry, sc,
					&ab_bootsequence_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("ab_clk_register", 0444, sc->d_entry, sc,
					&ab_clk_register_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("ab_ddr_ctrl", 0444, sc->d_entry, sc,
					&ab_ddr_ctrl_fops);
	if (!d)
		goto err_out;

	d_chip = debugfs_create_dir("airbrush_sm", sc->d_entry);
	if (!d_chip)
		goto err_out;

	d = debugfs_create_file("ab_assign_resources", 0664, d_chip, sc,
			&ab_assign_resources_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("chip_state", 0664, d_chip, sc,
				&fops_chip_state);
        if (!d)
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
#endif /* CONFIG_DEBUGFS_AIRBRUSH */

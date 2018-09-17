/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *	Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * Airbrush State Manager Control driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_SM_CTRL_H
#define _AIRBRUSH_SM_CTRL_H

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include "airbrush-clk.h"

#define __GPIO_ENABLE   0x1
#define __GPIO_DISABLE   0x0

#define NUM_BLOCKS 6

#define CONFIG_DEBUGFS_AIRBRUSH

#define __GPIO_ENABLE	0x1
#define __GPIO_DISABLE	0x0

typedef enum __block_names {
	BLK_IPU,
	BLK_TPU,
	DRAM,
	BLK_MIF,
	BLK_FSYS,
	BLK_AON,
} block_name_t;

enum states {
	off = 0,
	on = 1,
};


/*Should be in ascending order (for comparisons)*/
typedef enum __logic_voltage {
	VOLTAGE_0_0,
	VOLTAGE_0_60,
	VOLTAGE_0_75,
	VOLTAGE_0_85,
} logic_voltage_t;

typedef enum __chip_state {
	CHIP_STATE_0_0 = 0,
	CHIP_STATE_0_1,
	CHIP_STATE_0_2,
	CHIP_STATE_0_3,
	CHIP_STATE_0_4,
	CHIP_STATE_0_5,
	CHIP_STATE_0_6,
	CHIP_STATE_0_7,
	CHIP_STATE_0_8,
	CHIP_STATE_0_9,
	CHIP_STATE_1_0 = 10,
	CHIP_STATE_1_1,
	CHIP_STATE_1_2,
	CHIP_STATE_1_3,
	CHIP_STATE_1_4,
	CHIP_STATE_1_5,
	CHIP_STATE_1_6,
	CHIP_STATE_2_0 = 20,
	CHIP_STATE_2_1,
	CHIP_STATE_2_2,
	CHIP_STATE_2_3,
	CHIP_STATE_2_4,
	CHIP_STATE_2_5,
	CHIP_STATE_2_6,
	CHIP_STATE_3_0 = 30,
	CHIP_STATE_4_0 = 40,
	CHIP_STATE_5_0 = 50,
	CHIP_STATE_6_0 = 60,
} chip_state_t;

typedef enum __block_state {
	BLOCK_STATE_0_0 = 0,
	BLOCK_STATE_0_1,
	BLOCK_STATE_0_2,
	BLOCK_STATE_0_3,
	BLOCK_STATE_0_4,
	BLOCK_STATE_0_5,
	BLOCK_STATE_0_6,
	BLOCK_STATE_1_0 = 10,
	BLOCK_STATE_1_1,
	BLOCK_STATE_1_2,
	BLOCK_STATE_2_0 = 20,
	BLOCK_STATE_3_0 = 30,
	BLOCK_STATE_DEFAULT,
} block_state_t;

#define bit(x) (1<<x)
#define IPU_POWER_CONTROL	bit(0)
#define TPU_POWER_CONTROL	bit(1)
#define DRAM_POWER_CONTROL	bit(2)
#define MIF_POWER_CONTROL	bit(3)
#define FSYS_POWER_CONTROL	bit(4)
#define AON_POWER_CONTROL	bit(5)

/**
 * struct block_property - stores the information of a soc block's operating state.
 *
 * @id: The block state id of the SOC block.
 * @state_name: the name of the corresponing block state
 * @substate_name: the name of the corresponding substate.
 * @voltage_rate_status: status of the voltage rail, (on/off)
 * @logic_voltage: the voltage provoded to the block in volts(multiplied by 100).
 * @clock_status: status of the clock tree which provides the clock
 * @clk_frequency: frequency of the clock in Hz
 * @num_powered_cores: number of cores that are powered up.
 * @num_computing_cores: Number of cores that are used for computation.
 * @num_powered_tiles: Number of powered tiles.
 * @data_rate: Rate of data transfer.
 */
struct block_property {
	block_state_t id;
	char *state_name;
	char *substate_name;
	enum states voltage_rail_status;
	logic_voltage_t logic_voltage;
	enum states clk_status;
	u64 clk_frequency;
	u32 num_powered_cores;
	u32 num_computing_cores;
	u32 num_powered_tiles;
	u32 data_rate;
};


/**
 * struct block - stores the information about a SOC block
 *
 * @name: name of the block
 * @current_id: id of current state of the block
 * @current_state_category: category of the current state belongs.
 * @block_property_table: table containing details of all the states of the
 * @nr_block_states: number of possible states for this block
 */
struct block {
	block_name_t name;
	struct block_property *current_state;
	struct block_property *block_property_table;
	u32 nr_block_states;
	int (*set_state)(const struct block_property *, void *);
	void *data; /*IP specific data*/
};

struct chip_to_block_map {
	chip_state_t chip_substate_id;
	block_state_t ipu_block_state_id;
	block_state_t tpu_block_state_id;
	block_state_t dram_block_state_id;
	block_state_t mif_block_state_id;
	block_state_t fsys_block_state_id;
	block_state_t aon_block_state_id;
	u32 flags;
};

enum ab_sm_event {
	AB_SM_EV_THERMAL_MONITOR,	/* Thermal event */
	AB_SM_EV_DEVICE_ERROR,		/* Other device fail */
	AB_SM_EV_LINK_ERROR,		/* ... */
	// ...
};

typedef int (*ab_sm_callback_t)(enum ab_sm_event, uintptr_t data, void *cookie);

/**
 * struct ab_state_context - stores the context of airbrush soc
 *
 * @pdev: pointer to the platform device managing this context
 * @dev: pointer to the device managing this context
 * @sw_state_id: id of the current software state
 * @sw_state_name: name of the current software state
 * @chip_substate_id: id of the current chip substate
 * @chip_substate_name: name of the current chip substate
 * @chip_state_table: Table which contains information about all chip states
 * @nr_chip_states: Number of possible chip states
 * @d_entry: debugfs entry directory
 */
struct ab_state_context {
	struct platform_device *pdev;
	struct device *dev;
	struct block blocks[NUM_BLOCKS];
	enum {S0, S1, S2, S3, S4, S5, S6} sw_state_id;
	char *sw_state_name;
	chip_state_t chip_substate_id;
	char *chip_substate_name;
	struct chip_to_block_map *chip_state_table;
	u32 nr_chip_states;

	/* mutex for synchronization (if needed) */
	struct mutex lock;

	/* pins used in bootsequence */
	struct gpio_desc *soc_pwrgood;	/* output */
	struct gpio_desc *fw_patch_en;	/* output */
	struct gpio_desc *ab_ready;	/* input  */
	struct gpio_desc *ddr_sr;	/* output */
	struct gpio_desc *ddr_train;	/* output */
	struct gpio_desc *cke_in;	/* output */
	struct gpio_desc *cke_in_sense;	/* output */

	unsigned int ab_ready_irq;	/* ab_ready_gpio irq */

	int otp_fw_patch_dis;		/* OTP info from Airbrush (DT property) */

	ab_sm_callback_t cb_event;	/* Event callback registered by the SM */
	void *cb_cookie;		/* Private data sent by SM while registering event callback */

	/* regulator descriptors */
	struct regulator *smps1;
	struct regulator *smps2;
	struct regulator *smps3;
	struct regulator *ldo1;
	struct regulator *ldo2;
	struct regulator *ldo3;
	struct regulator *ldo4;
	struct regulator *ldo5;

#ifdef CONFIG_DEBUGFS_AIRBRUSH
	struct dentry *d_entry;
#endif
};

/*
 *  void ab_sm_register_blk_callback - register block specific state change callback
 *
 *  name: name of the block for which this callback should be called.
 *  set_state: callback function.
 *  block_property: block_property structure passed to callback.
 *  data: the cookie that is passed back to the callback.
 */
void ab_sm_register_blk_callback(block_name_t name,
				int (*set_state)(const struct block_property *, void *),
				void *data);

struct ab_state_context *ab_sm_init(struct platform_device *pdev);
int ab_sm_register_callback(struct ab_state_context *sc,
				ab_sm_callback_t cb, void *cookie);
int ab_sm_set_state(struct ab_state_context *sc, u32 to_sw_state_id,
			u32 to_chip_substate_id);
int ab_bootsequence(struct ab_state_context *ab_ctx, bool patch_fw);
int ab_get_pmic_resources(struct ab_state_context *ab_ctx);
void abc_clk_register(struct ab_state_context *ab_ctx);
int ab_ddr_init(struct ab_state_context *sc);
int ab_ddr_suspend(struct ab_state_context *sc);
int ab_ddr_resume(struct ab_state_context *sc);

int ab_pmic_on(struct ab_state_context *ab_ctx);
void ab_enable_pgood(struct ab_state_context *ab_ctx);
void ab_disable_pgood(struct ab_state_context *ab_ctx);

#ifdef CONFIG_DEBUGFS_AIRBRUSH
void ab_sm_create_debugfs(struct ab_state_context *sc);
#endif

#endif /* _AIRBRUSH_SM_CTRL_H */

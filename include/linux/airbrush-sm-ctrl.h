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

#include <linux/airbrush-sm-notifier.h>
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kfifo.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define __GPIO_ENABLE   0x1
#define __GPIO_DISABLE   0x0

#define NUM_BLOCKS 6

#define __GPIO_ENABLE	0x1
#define __GPIO_DISABLE	0x0

enum block_name {
	BLK_IPU,
	BLK_TPU,
	DRAM,
	BLK_MIF,
	BLK_FSYS,
	BLK_AON,
};

enum states {
	off = 0,
	on = 1,
};

/*
 * Read a register from the address,addr of ABC device and put it in to_addr,
 * all reads are currently happening through PCIe
 */
#define ABC_READ(addr, to_addr) \
	abc_pcie_config_read(addr & 0xffffff, 0x0, to_addr)

/*
 * Write value to an ABC register address, addr
 * All writes are currently happening through PCIe
 */
#define ABC_WRITE(addr, value) \
	abc_pcie_config_write(addr & 0xffffff, 0x0, value)

/*Should be in ascending order (for comparisons)*/
enum logic_voltage {
	VOLTAGE_0_0,
	VOLTAGE_0_60,
	VOLTAGE_0_75,
	VOLTAGE_0_85,
};

enum ddr_state {
	DDR_ON,
	DDR_SLEEP,
	DDR_SUSPEND,
	DDR_OFF,
};

enum throttle_state {
	THROTTLE_NONE = 0,
	THROTTLE_TO_MID,
	THROTTLE_TO_LOW,
	THROTTLE_TO_MIN,
};

enum chip_state {
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
};

enum block_state {
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
	NUM_BLOCK_STATES,
};

enum stat_state {
	STAT_STATE_ACTIVE = 0,
	STAT_STATE_SLEEP,
	STAT_STATE_DEEP_SLEEP,
	STAT_STATE_SUSPEND,
	STAT_STATE_OFF,
	STAT_STATE_UNKNOWN,
	STAT_STATE_SIZE,
};

#define bit(x) (1<<x)
#define IPU_POWER_CONTROL	bit(0)
#define TPU_POWER_CONTROL	bit(1)
#define DRAM_POWER_CONTROL	bit(2)
#define MIF_POWER_CONTROL	bit(3)
#define FSYS_POWER_CONTROL	bit(4)
#define AON_POWER_CONTROL	bit(5)

/**
 * struct block_property
 * stores the information of a soc block's operating state.
 *
 * @id: The block state id of the SOC block.
 * @state_name: the name of the corresponing block state
 * @substate_name: the name of the corresponding substate.
 * @voltage_rate_status: status of the voltage rail, (on/off)
 * @logic_voltage: the voltage provoded to the block in volts (mult by 100).
 * @clock_status: status of the clock tree which provides the clock
 * @clk_frequency: frequency of the clock in Hz
 * @num_powered_cores: number of cores that are powered up.
 * @num_computing_cores: Number of cores that are used for computation.
 * @num_powered_tiles: Number of powered tiles.
 * @data_rate: Rate of data transfer.
 */
struct block_property {
	enum block_state id;
	char *state_name;
	char *substate_name;
	enum states voltage_rail_status;
	enum logic_voltage logic_voltage;
	enum states clk_status;
	u64 clk_frequency;
	u32 num_powered_cores;
	u32 num_computing_cores;
	u32 num_powered_tiles;
	u32 data_rate;
};

typedef int (*ab_sm_set_block_state_t)(
		const struct block_property *current_property,
		const struct block_property *desired_property,
		enum chip_state chip_substate_id, void *data);

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
	enum block_name name;
	struct block_property *current_state;
	struct block_property *block_property_table;
	u32 nr_block_states;
	ab_sm_set_block_state_t set_state;
	void *data; /*IP specific data*/
};

struct chip_to_block_map {
	enum chip_state chip_substate_id;
	enum block_state ipu_block_state_id;
	enum block_state tpu_block_state_id;
	enum block_state dram_block_state_id;
	enum block_state mif_block_state_id;
	enum block_state fsys_block_state_id;
	enum block_state aon_block_state_id;
	u32 flags;
};

enum ab_error_codes {
	E_INVALID_CHIP_STATE, /* Chip state entered is invalid*/
	E_INVALID_BLOCK_STATE, /* Block state is invalid*/
	E_STATE_CHANGE, /*Chip State change failed*/
	E_STATUS_TIMEOUT, /* Timeout happened while checking status */
	E_IPU_BLOCK_OFF, /* IPU block is already off */
	E_IPU_BLOCK_ON, /* IPU block is already on */
	E_TPU_BLOCK_OFF, /* TPU block is already off */
	E_TPU_BLOCK_ON, /* TPU block is already on */
	E_IPU_CORES_ALREADY_OFF, /* All the IPU cores are already off */
	E_IPU_CORES_ALREADY_ON, /* All the IPU cores are on */
	E_TPU_TILES_ALREADY_OFF, /* TPU block is already on */
	E_TPU_TILES_ALREADY_ON, /* TPU block is already on */
	E_IPU_CORES_OFF, /* An error occurred in turning off IPU cores */
	E_IPU_CORES_ON, /* An error occurred in turning on IPU cores */
	E_TPU_TILES_OFF, /* An error occurred in turning off TPU tiles */
	E_TPU_TILES_ON, /* An error occurred in turning on TPU tiles */
};

enum ab_sm_event {
	AB_SM_EV_THERMAL_MONITOR,	/* Thermal event */
	AB_SM_EV_DEVICE_ERROR,		/* Other device fail */
	AB_SM_EV_LINK_ERROR,		/* ... */
	// ...
};

enum ab_chip_id {
	CHIP_ID_UNKNOWN = -1,
	CHIP_ID_A0 = 0,
	CHIP_ID_B0,
};

struct ab_sm_state_stat {
	u64 counter;		/* cumulative */
	ktime_t duration;	/* cumulative */
	ktime_t last_entry;
	ktime_t last_exit;
};

typedef int (*ab_sm_callback_t)(enum ab_sm_event, uintptr_t data, void *cookie);

struct ab_sm_pmu_ops {
	void *ctx;

	int (*pmu_sleep)(void *ctx);
	int (*pmu_deep_sleep)(void *ctx);
	int (*pmu_resume)(void *ctx);
};

static int pmu_sleep_stub(void *ctx)      { return -ENODEV; }
static int pmu_deep_sleep_stub(void *ctx) { return -ENODEV; }
static int pmu_resume_stub(void *ctx)     { return -ENODEV; }

static struct ab_sm_pmu_ops pmu_ops_stub = {
	.ctx = NULL,

	.pmu_sleep = &pmu_sleep_stub,
	.pmu_deep_sleep = &pmu_deep_sleep_stub,
	.pmu_resume = &pmu_resume_stub,
};

struct ab_sm_clk_ops {
	void *ctx;

	int (*ipu_pll_enable)(void *ctx);
	int (*ipu_pll_disable)(void *ctx);
	int (*ipu_gate)(void *ctx);
	int (*ipu_ungate)(void *ctx);
	u64 (*ipu_set_rate)(void *ctx, u64 rate);

	int (*tpu_pll_enable)(void *ctx);
	int (*tpu_pll_disable)(void *ctx);
	int (*tpu_gate)(void *ctx);
	int (*tpu_ungate)(void *ctx);
	u64 (*tpu_set_rate)(void *ctx, u64 rate);

	u64 (*aon_set_rate)(void *ctx, u64 rate);

	int (*attach_mif_clk_ref)(void *ctx);
	int (*deattach_mif_clk_ref)(void *ctx);
};

static int ipu_pll_enable_stub(void *ctx)   { return -ENODEV; }
static int ipu_pll_disable_stub(void *ctx)   { return -ENODEV; }
static int ipu_gate_stub(void *ctx)   { return -ENODEV; }
static int ipu_ungate_stub(void *ctx) { return -ENODEV; }
static u64 ipu_set_rate_stub(void *ctx, u64 rate) { return 0; }

static int tpu_pll_enable_stub(void *ctx)   { return -ENODEV; }
static int tpu_pll_disable_stub(void *ctx)   { return -ENODEV; }
static int tpu_gate_stub(void *ctx)   { return -ENODEV; }
static int tpu_ungate_stub(void *ctx) { return -ENODEV; }
static u64 tpu_set_rate_stub(void *ctx, u64 rate) { return 0; }

static u64 aon_set_rate_stub(void *ctx, u64 rate) { return 0; }

static int attach_mif_clk_ref_stub(void *ctx)   { return -ENODEV; }
static int deattach_mif_clk_ref_stub(void *ctx) { return -ENODEV; }

static struct ab_sm_clk_ops clk_ops_stub = {
	.ctx = NULL,

	.ipu_pll_enable = &ipu_pll_enable_stub,
	.ipu_pll_disable = &ipu_pll_disable_stub,
	.ipu_gate = &ipu_gate_stub,
	.ipu_ungate = &ipu_ungate_stub,
	.ipu_set_rate = &ipu_set_rate_stub,

	.tpu_pll_enable = &tpu_pll_enable_stub,
	.tpu_pll_disable = &tpu_pll_disable_stub,
	.tpu_gate = &tpu_gate_stub,
	.tpu_ungate = &tpu_ungate_stub,
	.tpu_set_rate = &tpu_set_rate_stub,

	.aon_set_rate = &aon_set_rate_stub,

	.attach_mif_clk_ref = &attach_mif_clk_ref_stub,
	.deattach_mif_clk_ref = &deattach_mif_clk_ref_stub,
};

struct ab_sm_dram_ops {
	void *ctx;

	// TODO: Define dram ops
};

static struct ab_sm_dram_ops dram_ops_stub = {
	.ctx = NULL,

	// TODO: Fill in with dram ops
};

struct ab_sm_mfd_ops {
	void *ctx;

	// TODO: Define mfd ops
	int (*enter_el2)(void *ctx);
	int (*exit_el2)(void *ctx);

	int (*get_chip_id)(void *ctx, enum ab_chip_id *val);
	int (*ab_ready)(void *ctx);
	int (*pcie_pre_disable)(void *ctx);
};

static int enter_el2_stub(void *ctx) { return -ENODEV; }
static int exit_el2_stub(void *ctx)  { return -ENODEV; }
static int get_chip_id_stub(void *ctx, enum ab_chip_id *val)
{
	return -ENODEV;
}

static int ab_ready_stub(void *ctx)  { return -ENODEV; }
static int pcie_pre_disable_stub(void *ctx)  { return -ENODEV; }

static struct ab_sm_mfd_ops mfd_ops_stub = {
	.ctx = NULL,

	.enter_el2 = &enter_el2_stub,
	.exit_el2 = &exit_el2_stub,
	.get_chip_id = &get_chip_id_stub,
	.ab_ready = &ab_ready_stub,
	.pcie_pre_disable = &pcie_pre_disable_stub,
};

struct ab_change_req {
	u32 new_state;
	int *ret_code;
	struct completion *comp;
};

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
	struct miscdevice misc_dev;

	struct task_struct *state_change_task;

	struct block blocks[NUM_BLOCKS];
	enum throttle_state throttle_state_id;
	enum chip_state dest_chip_substate_id;
	enum chip_state curr_chip_substate_id;
	struct chip_to_block_map *chip_state_table;
	u32 nr_chip_states;

	struct kfifo state_change_reqs;
	enum ab_chip_id chip_id;

	/* Synchronization structs */
	struct mutex pmic_lock;
	struct mutex state_lock;
	struct completion state_change_requested;
	struct completion state_change_comp;

	/* pins used in bootsequence */
	struct gpio_desc *soc_pwrgood;	/* output */
	struct gpio_desc *fw_patch_en;	/* output */
	struct gpio_desc *ab_ready;	/* input  */
	struct gpio_desc *ddr_sr;	/* output */
	struct gpio_desc *ddr_iso;	/* output */
	struct gpio_desc *ddr_train;	/* output */
	struct gpio_desc *cke_in;	/* output */
	struct gpio_desc *cke_in_sense;	/* output */

	unsigned int ab_ready_irq;	/* ab_ready_gpio irq */

	/* Check for alternate boot */
	int alternate_boot;

	/* Event callback registered by the SM */
	ab_sm_callback_t cb_event;
	/* Private data sent by SM while registering event callback */
	void *cb_cookie;

	/* regulator descriptors */
	struct regulator *smps1;
	struct regulator *smps2;
	struct regulator *smps3;
	struct regulator *ldo1;
	struct regulator *ldo2;
	struct regulator *ldo3;
	struct regulator *ldo4;
	struct regulator *ldo5;

	bool smps1_state;
	bool smps2_state;
	bool smps3_state;
	bool ldo1_state;
	bool ldo2_state;
	bool ldo3_state;
	bool ldo4_state;
	bool ldo5_state;

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_DEBUGFS)
	struct dentry *d_entry;
#endif
	atomic_t clocks_registered;
	enum ddr_state ddr_state;
	struct pci_dev *pcie_dev;
	bool cold_boot;

	atomic_t async_in_use;
	struct mutex async_fifo_lock;
	struct kfifo *async_entries;

	struct blocking_notifier_head clk_subscribers;

	/* power state stats */
	struct ab_sm_state_stat state_stats[STAT_STATE_SIZE];

	// MFD child operations
	struct mutex		op_lock;
	struct ab_sm_pmu_ops	*pmu_ops;
	struct ab_sm_clk_ops	*clk_ops;
	struct ab_sm_dram_ops	*dram_ops;

	/* NOTE: separate lock needed for mfd ops, as enter/exit_el2
	 * will likely cause mfd to call unregister methods for other ops.
	 * Would deadlock if using op_lock.
	 */
	struct mutex		mfd_lock;
	struct ab_sm_mfd_ops	*mfd_ops;
};

struct ab_sm_misc_session {
	struct ab_state_context *sc;
	bool first_entry;
	struct kfifo async_entries;
};

/*
 *  void ab_sm_register_blk_callback
 *  register block specific state change callback
 *
 *  name: name of the block for which this callback should be called.
 *  callback: set_state callback function.
 *  block_property: block_property structure passed to callback.
 *  data: the cookie that is passed back to the callback.
 */
void ab_sm_register_blk_callback(enum block_name name,
		ab_sm_set_block_state_t callback, void *data);

void ab_sm_register_pmu_ops(struct ab_sm_pmu_ops *ops);
void ab_sm_unregister_pmu_ops(void);
void ab_sm_register_clk_ops(struct ab_sm_clk_ops *ops);
void ab_sm_unregister_clk_ops(void);
void ab_sm_register_dram_ops(struct ab_sm_dram_ops *ops);
void ab_sm_unregister_dram_ops(void);
void ab_sm_register_mfd_ops(struct ab_sm_mfd_ops *ops);
void ab_sm_unregister_mfd_ops(void);

struct ab_state_context *ab_sm_init(struct platform_device *pdev);
void ab_sm_exit(struct platform_device *pdev);
int ab_sm_register_callback(struct ab_state_context *sc,
				ab_sm_callback_t cb, void *cookie);
int ab_sm_set_state(struct ab_state_context *sc, u32 to_chip_substate_id);
enum chip_state ab_sm_get_state(struct ab_state_context *sc);

int ab_bootsequence(struct ab_state_context *ab_ctx);
int ab_ddr_init(struct ab_state_context *sc);
int ab_ddr_suspend(struct ab_state_context *sc);
int ab_ddr_resume(struct ab_state_context *sc);
int ab_ddr_selfrefresh_enter(struct ab_state_context *sc);
int ab_ddr_selfrefresh_exit(struct ab_state_context *sc);
int ab_ddr_setup(struct ab_state_context *sc);
void ab_ddr_read_write_test(int read_write);

enum ab_chip_id ab_get_chip_id(struct ab_state_context *sc);

void ab_enable_pgood(struct ab_state_context *ab_ctx);
void ab_disable_pgood(struct ab_state_context *ab_ctx);
void ab_gpio_enable_ddr_sr(struct ab_state_context *ab_ctx);
void ab_gpio_disable_ddr_sr(struct ab_state_context *ab_ctx);
int  ab_gpio_get_ddr_sr(struct ab_state_context *ab_ctx);
void ab_gpio_enable_ddr_iso(struct ab_state_context *ab_ctx);
void ab_gpio_disable_ddr_iso(struct ab_state_context *ab_ctx);
int  ab_gpio_get_ddr_iso(struct ab_state_context *ab_ctx);
void ab_gpio_enable_fw_patch(struct ab_state_context *ab_ctx);
void ab_gpio_disable_fw_patch(struct ab_state_context *ab_ctx);

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_DEBUGFS)
void ab_sm_create_debugfs(struct ab_state_context *sc);
void ab_sm_remove_debugfs(struct ab_state_context *sc);
#else
static inline void ab_sm_create_debugfs(struct ab_state_context *sc) {}
static inline void ab_sm_remove_debugfs(struct ab_state_context *sc) {}
#endif

void ab_sm_create_sysfs(struct ab_state_context *sc);
void ab_sm_remove_sysfs(struct ab_state_context *sc);

#endif /* _AIRBRUSH_SM_CTRL_H */

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
#if IS_ENABLED(CONFIG_PCI_MSM)
#include <linux/msm_pcie.h>
#endif
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define NUM_BLOCKS 6

#define __GPIO_ENABLE	0x1
#define __GPIO_DISABLE	0x0

#define AB_SM_OSC_RATE 19200000

#define AB_SM_STATE_IN_RANGE(chip_state, state_range) \
	((chip_state / 100) == (state_range / 100))

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
	CHIP_STATE_0 = 0,
	CHIP_STATE_100 = 100,
	CHIP_STATE_200 = 200,
	CHIP_STATE_300 = 300,
	CHIP_STATE_400 = 400,
	CHIP_STATE_401,
	CHIP_STATE_402,
	CHIP_STATE_403,
	CHIP_STATE_404,
	CHIP_STATE_405,
	CHIP_STATE_406,
	CHIP_STATE_407,
	CHIP_STATE_408,
	CHIP_STATE_409,
	CHIP_STATE_500 = 500,
	CHIP_STATE_501,
	CHIP_STATE_502,
	CHIP_STATE_503,
	CHIP_STATE_504,
	CHIP_STATE_505,
	CHIP_STATE_600 = 600,
	CHIP_STATE_601,
	CHIP_STATE_602,
	CHIP_STATE_603,
	CHIP_STATE_604,
	CHIP_STATE_605,
	CHIP_STATE_700 = 700,
	CHIP_STATE_701,
	CHIP_STATE_702,
	CHIP_STATE_703,
	CHIP_STATE_704,
	CHIP_STATE_705,
};

enum block_state {
	BLOCK_STATE_0 = 0,
	BLOCK_STATE_100 = 100,
	BLOCK_STATE_101,
	BLOCK_STATE_200 = 200,
	BLOCK_STATE_201,
	BLOCK_STATE_202,
	BLOCK_STATE_300 = 300,
	BLOCK_STATE_301,
	BLOCK_STATE_302,
	BLOCK_STATE_303,
	BLOCK_STATE_304,
	BLOCK_STATE_305,
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

enum pmu_states {
	PMU_STATE_OFF = 0,
	PMU_STATE_DEEP_SLEEP,
	PMU_STATE_SLEEP,
	PMU_STATE_ON,
};

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
	enum pmu_states pmu;
	enum states rail_en;
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
		enum block_state block_substate_id, void *data);

/**
 * struct block - stores the information about a SOC block
 *
 * @name: name of the block
 * @current_id: id of current state of the block
 * @current_state_category: category of the current state belongs.
 * @prop_table: table containing details of all the states
 * @nr_block_states: number of possible states for this block
 */
struct block {
	enum block_name name;
	struct block_property *current_state;
	struct block_property *prop_table;
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

enum ab_sm_time_stamps {
	/*SM functions*/
	AB_SM_TS_FULL,
	AB_SM_TS_IPU,
	AB_SM_TS_TPU,
	AB_SM_TS_DRAM,
	AB_SM_TS_MIF,
	AB_SM_TS_FSYS,
	AB_SM_TS_AON,

	/*PMIC*/
	AB_SM_TS_PMIC_ON,
	AB_SM_TS_PMIC_OFF,

	/*Bootsequence*/
	AB_SM_TS_BOOT_SEQ,

	/*PMU*/
	AB_SM_TS_IPU_PMU_RES,
	AB_SM_TS_TPU_PMU_RES,
	AB_SM_TS_IPU_PMU_SLEEP,
	AB_SM_TS_TPU_PMU_SLEEP,
	AB_SM_TS_PMU_DEEP_SLEEP,

	/*Clock*/
	AB_SM_TS_IPU_CLK,
	AB_SM_TS_TPU_CLK,
	AB_SM_TS_AON_CLK,

	/*Callbacks*/
	AB_SM_TS_PCIE_CB,
	AB_SM_TS_DDR_CB,

	/*Others*/
	AB_SM_TS_PCIE_ENUM,
	AB_SM_TS_DDR_INIT,
	AB_SM_TS_ALT_BOOT,

	NUM_AB_SM_TS,
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

	int (*pmu_ipu_sleep)(void *ctx);
	int (*pmu_tpu_sleep)(void *ctx);
	int (*pmu_deep_sleep)(void *ctx);
	int (*pmu_ipu_resume)(void *ctx);
	int (*pmu_tpu_resume)(void *ctx);
};

struct ab_sm_clk_ops {
	void *ctx;

	void (*init)(void *ctx);

	int (*ipu_pll_enable)(void *ctx);
	int (*ipu_pll_disable)(void *ctx);
	int (*ipu_gate)(void *ctx);
	int (*ipu_ungate)(void *ctx);
	int64_t (*ipu_set_rate)(void *ctx, u64 old_rate, u64 new_rate);

	int (*tpu_pll_enable)(void *ctx);
	int (*tpu_pll_disable)(void *ctx);
	int (*tpu_gate)(void *ctx);
	int (*tpu_ungate)(void *ctx);
	int64_t (*tpu_set_rate)(void *ctx, u64 old_rate, u64 new_rate);
	int64_t (*tpu_set_rate_direct)(void *ctx, u64 new_rate);

	int64_t (*aon_set_rate)(void *ctx, u64 old_rate, u64 new_rate);
	int64_t (*aon_set_rate_direct)(void *ctx, u64 new_rate);

	int (*reduce_mainclk_freq)(void *ctx);
	int (*restore_mainclk_freq)(void *ctx);
};

struct ab_sm_dram_ops {
	void *ctx;

	int (*setup)(void *ctx, void *ab_state_ctx);
	int (*wait_for_init)(void *ctx);
	int (*init)(void *ctx);
	int (*get_freq)(void *ctx, u64 *val);
	int (*set_freq)(void *ctx, u64 val);
	int (*suspend)(void *ctx);
	int (*resume)(void *ctx);
	int (*sref_enter)(void *ctx);
	int (*sref_exit)(void *ctx);
	int (*rw_test)(void *ctx, unsigned int read_write);
	int (*eye_margin)(void *ctx, unsigned int test_data);
	int (*eye_margin_plot)(void *ctx);
	int (*ppc_set_event)(void *ctx,
			     unsigned int counter_idx, unsigned int event);
	void (*ppc_ctrl)(void *ctx, int is_start);
};

struct ab_sm_mfd_ops {
	void *ctx;

	// TODO: Define mfd ops
	int (*enter_el2)(void *ctx);
	int (*exit_el2)(void *ctx);

	int (*get_chip_id)(void *ctx, enum ab_chip_id *val);
	int (*ab_ready)(void *ctx);
	int (*pcie_pre_disable)(void *ctx);
	int (*pcie_linkdown)(void *ctx);

	/* TODO(b/122614252):  Temporarily provide a mechanism to allow for PCIe
	 * DMA from EL1 after the enter EL2 ioctl or debugfs file has been
	 * invoked.  This is a temporary mechanism to allow testing from EL1 and
	 * EL2 contexts.  This should be removed once EL2 based software is
	 * ready for use.
	 */
	void (*set_el2_dma_mode)(void *ctx, bool allow_el1_dma);
	bool (*get_el2_dma_mode)(void *ctx);
};

struct ab_thermal;

struct ab_change_req {
	u32 new_state;
	int *ret_code;
	struct completion *comp;
};

struct ab_asv_info {
	bool fusing_done;
	int asv_version;
	u32 ipu_volt;
	u32 tpu_volt;
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
 * @set_state_lock: locks calls to _ab_sm_set_state
 * @state_transitioning_lock: locks dest_chip_substate_id and
 *		curr_chip_substate_id during a transition
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
	struct mutex set_state_lock;
	struct mutex state_transitioning_lock;
	struct completion request_state_change_comp;
	struct completion transition_comp;
	struct completion notify_comp;

	int change_ret;

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

	struct ab_asv_info asv_info;

	/* regulator descriptors */
	struct regulator *smps1;
	struct regulator *smps2;
	struct regulator *smps3;
	struct regulator *ldo1;
	struct regulator *ldo2;
	struct regulator *ldo3;
	struct regulator *ldo4;
	struct regulator *ldo5;
	struct regulator *boost_smps1;
	struct regulator *boost_ldo3;

	bool smps1_state;
	bool smps2_state;
	bool smps3_state;
	bool ldo1_state;
	bool ldo2_state;
	bool ldo3_state;
	bool ldo4_state;
	bool ldo5_state;

	u64 smps2_delay;
	u64 ldo4_delay;
	u64 ldo5_delay;
	u64 s60_delay;

#if IS_ENABLED(CONFIG_PCI_MSM)
	struct msm_pcie_register_event pcie_link_event;
#endif

	struct notifier_block regulator_nb; /* single notifier */
	struct work_struct shutdown_work; /* emergency shutdown work */

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_DEBUGFS)
	struct dentry *d_entry;
#endif
	atomic_t clocks_registered;
	void *ddr_data; /* ddr private data */
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

	struct ab_thermal *thermal;

	bool force_el2;
	bool el2_mode; /* Guarded by state_transitioning_lock */

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_PROFILE)
	/* time stamps */
	bool ts_enabled;
	u64 state_trans_ts[NUM_AB_SM_TS];
#endif
	int clkout_idx;
	int clkout_blk_idx;
	int clkout_clk_idx;
};

struct ab_sm_misc_session {
	struct ab_state_context *sc;
	bool first_entry;
	struct kfifo async_entries;
};

#if IS_ENABLED(CONFIG_PCI_MSM)
/*
 *  Set up listener to pcie linkdown event.
 *  Call only once after enumeration is done.
 */
int ab_sm_setup_pcie_event(struct ab_state_context *sc);
#else
static inline int ab_sm_setup_pcie_event(struct ab_state_context *sc)
{
	return 0;
}
#endif /* CONFIG_PCI_MSM */

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
int ab_sm_set_state(struct ab_state_context *sc,
	u32 to_chip_substate_id, bool mapped);
u32 ab_sm_get_state(struct ab_state_context *sc, bool mapped);
int ab_sm_map_state(u32 old_mapping, u32 *new_mapping);
int ab_sm_unmap_state(u32 new_mapping, u32 *old_mapping);

int ab_bootsequence(struct ab_state_context *ab_ctx,
		enum chip_state prev_state);
enum ab_chip_id ab_get_chip_id(struct ab_state_context *sc);
const enum stat_state ab_chip_state_to_stat_state(enum chip_state id);

int ab_sm_enter_el2(struct ab_state_context *sc);
int ab_sm_exit_el2(struct ab_state_context *sc);

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

void ab_clkout_sel(struct ab_state_context *sc, unsigned int clkout_idx);
void ab_clkout_blksel(struct ab_state_context *sc, unsigned int blk_idx);
void ab_clkout_clksel(struct ab_state_context *sc, unsigned int clk_idx);
int ab_clkout_enable(struct ab_state_context *sc, unsigned int enable);
int ab_clkout_freq(struct ab_state_context *sc, u64 *val);

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_DEBUGFS)
void ab_sm_create_debugfs(struct ab_state_context *sc);
void ab_sm_remove_debugfs(struct ab_state_context *sc);
#else
static inline void ab_sm_create_debugfs(struct ab_state_context *sc) {}
static inline void ab_sm_remove_debugfs(struct ab_state_context *sc) {}
#endif

#if IS_ENABLED(CONFIG_AIRBRUSH_SM_PROFILE)
void ab_sm_start_ts(struct ab_state_context *sc, int ts);
void ab_sm_record_ts(struct ab_state_context *sc, int ts);
void ab_sm_zero_ts(struct ab_state_context *sc);
void ab_sm_print_ts(struct ab_state_context *sc);
#else
static inline void ab_sm_start_ts(struct ab_state_context *sc, int ts) {}
static inline void ab_sm_record_ts(struct ab_state_context *sc, int ts) {}
static inline void ab_sm_zero_ts(struct ab_state_context *sc) {}
static inline void ab_sm_print_ts(struct ab_state_context *sc) {}
#endif

void ab_sm_create_sysfs(struct ab_state_context *sc);
void ab_sm_remove_sysfs(struct ab_state_context *sc);

void ab_lvcc_init(struct ab_asv_info *info);
void set_asv_version(struct ab_asv_info *info, int asv_version);
int ab_lvcc(struct ab_state_context *sc, int chip_state);
#endif /* _AIRBRUSH_SM_CTRL_H */

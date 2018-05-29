
/*
 * iaxxx-core.h  --  Knowles ALSA SoC Audio PCM driver header
 *
 * Copyright 2017 Knowles Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IAXXX_CORE_H__
#define __IAXXX_CORE_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/kthread.h>

typedef int (*iaxxx_cb_func_ptr_t)(struct device *dev);
typedef int (*iaxxx_cb_bc_func_ptr_t)(struct device *dev, u32 iaxxx_spi_speed);
struct workqueue_struct;
/* SRB is at the end of the memory map */
#define IAXXX_RBDT_NUM_ENTRIES	32
#define IAXXX_BLOCK_0 0 /* Update for CM4 PROC */
#define IAXXX_BLOCK_1 1 /* Update for HMD PROC */
#define IAXXX_BLOCK_2 2 /* Update for DMX PROC */
#define IAXXX_MAX_MODELS 3

#define IAXXX_PKG_ID_MASK	0x000F
#define IAXXX_PLGIN_ID_MASK	0x001F

#define IAXXX_FLG_STARTUP		1

enum {
	IAXXX_NORMAL,
	IAXXX_SUSPEND,
	IAXXX_POWER_TRANSITION,
	IAXXX_RESUME_FAIL
};

struct iaxxx_client {
	struct regmap *regmap;
};

/* Probed Bus */
enum iaxxx_bus {
	IAXXX_SPI = 0,
	IAXXX_I2C,
	IAXXX_UART,
};

enum iaxxx_fw_state {
	FW_SBL_MODE = 0,
	FW_APP_MODE,
	FW_CRASH,
	FW_RECOVERY,
};

enum iaxxx_plugin_state {
	IAXXX_PLUGIN_UNLOADED = 0,
	IAXXX_PLUGIN_LOADED,
	IAXXX_PLUGIN_ACTIVE,
	IAXXX_PLUGIN_RESET,
};

enum iaxxx_pkg_state {
	IAXXX_PKG_UNLOADED = 0,
	IAXXX_PKG_LOADED,
};

struct iaxxx_plug_inst_data {
	int proc_id;
	enum iaxxx_plugin_state plugin_inst_state;
};

struct iaxxx_pkg_data {
	int proc_id;
	enum iaxxx_pkg_state pkg_state;
};

struct iaxxx_block_err {
	uint32_t plg_inst_id[3];
	uint32_t error[3];
};

struct kw_model_info {
	uint32_t kw_loaded_bitmap;
	uint32_t kw_recognize_bitmap;
	uint32_t kw_id[IAXXX_MAX_MODELS];
};

struct iaxxx_system_state {
	enum iaxxx_fw_state fw_state;
	struct iaxxx_plug_inst_data plgin[IAXXX_PLGIN_ID_MASK + 1];
	struct iaxxx_pkg_data pkg[IAXXX_PKG_ID_MASK + 1];
	struct iaxxx_block_err err;
	struct kw_model_info kw_info;
};

enum {
	IAXXX_DBGLOG_CM4,
	IAXXX_DBGLOG_HMD,
	IAXXX_DBGLOG_DMX,
	IAXXX_CRASHLOG_CM4,
	IAXXX_CRASHLOG_HMD,
	IAXXX_CRASHLOG_DMX,
	IAXXX_MAX_LOG
};

struct iaxxx_crashlog_header {
	uint32_t log_type;
	uint32_t log_addr;
	uint32_t log_size;
};

struct iaxxx_crashlog {
	struct iaxxx_crashlog_header header[IAXXX_MAX_LOG];
	struct iaxxx_crashlog_header cm4header;
	char *log_buffer;
	uint32_t log_buffer_size;
	uint32_t logs_read;
};

/**
 * Description of driver private data
 *
 * @dev: device pointer
 * @regmap: the device register map
 * @regmap_config: regmap configuration data
 * @regmap_init: bus specific regmap init function
 * @sys_rbdt: cached copy of the Register Block Descriptor Table
 */
struct iaxxx_priv {
	struct device *dev;
	struct device *codec_dev;
	struct regmap *regmap;
	struct regmap_config *regmap_config;
	int (*regmap_init_bus)(struct iaxxx_priv *priv);
	int (*bulk_read)(struct device *dev, uint32_t address,
				void *buf, size_t len);
	int (*raw_write)(void *context,
			const void *reg,
			 const void *val, size_t val_len);

	uint32_t sys_rbdt[2*IAXXX_RBDT_NUM_ENTRIES];

	/* GPIOs */
	int reset_gpio;
	int event_gpio;
	u32 spi_app_speed;

	/* Regulators */
	struct regulator *vdd_io;
	struct regulator *vdd_core;

	/* External Clock */
	struct clk *ext_clk;

	/* Update block lock */
	struct mutex update_block_lock;

	/* Register plugin locks */
	struct mutex plugin_lock;

	/* Event work queue */
	struct mutex event_work_lock;
	struct mutex event_queue_lock;
	struct work_struct event_work_struct;
	struct workqueue_struct *event_workq;

	/* Core kthread */
	struct task_struct *thread;
	struct kthread_worker worker;

	/* Work for device init */
	struct kthread_work dev_init_work;
	struct kthread_work crash_work;
	struct work_struct crash_recover_work;

	void *tunnel_data;
	/* Event Manager */
	struct mutex event_lock;

	enum iaxxx_bus bus;

	/* For factory testing */
	struct work_struct dev_fw_update_test_work;
	struct work_struct dev_cmem_test_work;
	struct completion bootup_done;
	struct completion cmem_done;
	bool test_result;
	iaxxx_cb_bc_func_ptr_t spi_speed_setup;
	iaxxx_cb_func_ptr_t reset_cb;
	uint32_t fw_retry_count;
	struct mutex test_mutex;
	struct iaxxx_evt_queue *event_queue;

	/* Showing mode, boot (0) or app (1) */
	bool is_application_mode;
	struct iaxxx_raw_bus_ops *raw_ops;
	struct iaxxx_reg_dump_priv *reg_dump;
	void *intf_priv;
	bool dump_log;

	void *dfs_node;

	/* iaxxx core flags for atomic bit field operations */
	unsigned long flags;

	/* Synchronize suspend/resume on this */
	atomic_t power_state;
	struct notifier_block notifier_fbp;
	struct iaxxx_system_state *iaxxx_state;
	uint32_t crash_count;
	bool cm4_crashed;
	bool route_status;
	int (*crash_handler)(struct iaxxx_priv *priv);
	struct iaxxx_crashlog *crashlog;
	struct mutex crashdump_lock;
	bool   debug_isr_disable;
};

static inline struct iaxxx_priv *to_iaxxx_priv(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

int iaxxx_send_update_block_request(struct device *dev, uint32_t *status,
							int id);
int iaxxx_core_plg_get_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t *param_val, uint32_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id);
int iaxxx_core_create_plg(struct device *dev, uint32_t inst_id,
				uint32_t priority, uint32_t pkg_id,
				uint32_t plg_idx, uint8_t block_id);
int iaxxx_core_change_plg_state(struct device *dev, uint32_t inst_id,
				uint8_t is_enable, uint8_t block_id);
int iaxxx_core_destroy_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id);
int iaxxx_core_reset_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id);
int iaxxx_core_set_create_cfg(struct device *dev, uint32_t inst_id,
		uint32_t cfg_size, uint64_t cfg_val, uint32_t block_id,
		char *file);
int iaxxx_core_set_param_blk(struct device *dev, uint32_t inst_id,
			uint32_t blk_size, void *ptr_blk, uint32_t block_id,
			uint32_t slot_id);
int iaxxx_core_evt_subscribe(struct device *dev, uint16_t src_id,
		uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque);
int iaxxx_core_evt_unsubscribe(struct device *dev, uint16_t src_id,
		uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque);
int iaxxx_core_retrieve_event(struct device *dev, uint16_t *event_id,
		uint32_t *data);
int iaxxx_core_set_event(struct device *dev, uint8_t inst_id,
			uint32_t event_enable_mask, uint32_t block_id);
int iaxxx_package_load(struct device *dev, const char *pkg_name,
					uint32_t pkg_id, uint32_t *proc_id);
int iaxxx_package_unload(struct device *dev, const char *pkg_name,
					     uint32_t proc_pkg_id);
void iaxxx_tunnel_stop(struct iaxxx_priv *priv);
int iaxxx_tunnel_recovery(struct iaxxx_priv *priv);
int iaxxx_unload_kw_model(struct device *dev, uint32_t inst_id,
		uint32_t blk_id, uint32_t id);
int iaxxx_start_recognition(struct device *dev, uint32_t id);
int iaxxx_stop_recognition(struct device *dev, uint32_t id);
int iaxxx_get_kw_recognize_bitmap(struct device *dev, uint32_t *bitmap);
void iaxxx_reset_codec_params(struct iaxxx_priv *priv);
#endif /*__IAXXX_CORE_H__ */

/*
 * iaxxx-core.c -- IAxxx Multi-Function Device driver
 *
 * Copyright 2018 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-af.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-debuglog.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include "iaxxx.h"
#include "iaxxx-dbgfs.h"
#include "iaxxx-tunnel.h"
#include "iaxxx-debug.h"
#include "iaxxx-cdev.h"
#include "iaxxx-build-info.h"
#include <linux/circ_buf.h>
#include <linux/clk.h>

#define IAXXX_RESET_RETRIES		5		/* retry attempts */
#define IAXXX_RESET_HOLD_TIME		(20*1000)	/* 20 ms */
#define IAXXX_RESET_READY_DELAY		(20*1000)	/* 20 ms */
#define IAXXX_RESET_RANGE_INTERVAL	100		/* 100 us */

#define IAXXX_RESET_PWR_VLD_DELAY		(3*1000)
#define IAXXX_RESET_PWR_VLD_RANGE_INTERVAL	100

#define IAXXX_FW_RETRY_COUNT		5		/* 5 retry if failed */
#define IAXXX_FW_DOWNLOAD_TIMEOUT	10000		/* 10 secs */
#define IAXXX_VER_STR_SIZE		60
#define IAXXX_BYTES_IN_A_WORD		4
#define IAXXX_MAX_PLUGIN		(IAXXX_PLGIN_ID_MASK+1)
#define IAXXX_MAX_PACKAGE		(IAXXX_PKG_ID_MASK+1)

#define IAXXX_BYPASS_ON_VAL 0x00C199BB
#define IAXXX_BYPASS_OFF_VAL 0x0C099BB
#define IAXXX_PWR_DWN_VAL 0x01C00050
#define IAXXX_PWR_ON_VAL 0x845
#define IAXXX_PWR_STATE_RETRY	0x5

#define IAXXX_PM_AUTOSUSPEND_DELAY 3000
#define iaxxx_ptr2priv(ptr, item) container_of(ptr, struct iaxxx_priv, item)

/* Linux kthread APIs have changes in version 4.9 */
#if defined(init_kthread_worker)
#define iaxxx_work_flush(priv, work)	flush_kthread_work(&priv->work)
#define iaxxx_work(priv, work) queue_kthread_work(&priv->worker, &priv->work)
#define iaxxx_flush_kthread_worker(worker) flush_kthread_worker(worker)
#define iaxxx_init_kthread_worker(worker)  init_kthread_worker(worker)
#define iaxxx_init_kthread_work(work, fn)  init_kthread_work(work, fn)
#elif defined(kthread_init_worker)
#define iaxxx_work_flush(priv, work)	kthread_flush_work(&priv->work)
#define iaxxx_work(priv, work) kthread_queue_work(&priv->worker, &priv->work)
#define iaxxx_flush_kthread_worker(worker) kthread_flush_worker(worker)
#define iaxxx_init_kthread_worker(worker)  kthread_init_worker(worker)
#define iaxxx_init_kthread_work(work, fn)  kthread_init_work(work, fn)
#else
#error kthread functions not defined
#endif
struct iaxxx_port_clk_settings clk;
static const u32 iaxxx_port_clk_addr[] = {
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
	IAXXX_IO_CTRL_PORTD_CLK_ADDR,
	IAXXX_IO_CTRL_PORTE_CLK_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
};

static const u32 iaxxx_port_fs_addr[] = {
	IAXXX_IO_CTRL_PORTA_FS_ADDR,
	IAXXX_IO_CTRL_PORTB_FS_ADDR,
	IAXXX_IO_CTRL_PORTC_FS_ADDR,
	IAXXX_IO_CTRL_PORTD_FS_ADDR,
	IAXXX_IO_CTRL_PORTE_FS_ADDR,
	IAXXX_IO_CTRL_COMMB_1_ADDR,
};

static const u32 iaxxx_port_di_addr[] = {
	IAXXX_IO_CTRL_PORTA_DI_ADDR,
	IAXXX_IO_CTRL_PORTB_DI_ADDR,
	IAXXX_IO_CTRL_PORTC_DI_ADDR,
	IAXXX_IO_CTRL_PORTD_DI_ADDR,
	IAXXX_IO_CTRL_PORTE_DI_ADDR,
	IAXXX_IO_CTRL_COMMB_2_ADDR,
};

static const u32 iaxxx_port_do_addr[] = {
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
	IAXXX_IO_CTRL_PORTD_DO_ADDR,
	IAXXX_IO_CTRL_PORTE_DO_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,
};

/* Firmware download error code */
enum {
	E_IAXXX_REGMAP_ERROR = -1,
	E_IAXXX_BOOTUP_ERROR = -2,
	E_IAXXX_RESET_SYNC_ERROR = -3,
};

/*===========================================================================
 * MFD Driver
 *===========================================================================
 */

static struct mfd_cell iaxxx_devices[] = {
	{
		.name = "iaxxx-codec",
	},
	{
		.name = "iaxxx-odsp-celldrv",
	},
	{
		.name = "iaxxx-tunnel-celldrv",
	},
	{
		.name = "iaxxx-module-celldrv",
	},
};

static const char *iaxxx_crash_err2str(int error)
{
	switch (error) {
	case IAXXX_FW_CRASH_EVENT:
		return "crash event";
	case IAXXX_FW_CRASH_ON_FLUSH_EVENTS:
		return "crash event when flush events";
	case IAXXX_FW_CRASH_REG_MAP_WAIT_CLEAR:
		return "crash when wait clear";
	case IAXXX_FW_CRASH_UPDATE_BLOCK_REQ:
		return "crash during update block req";
	default:
		return "unknown error";
	}
}

static int iaxxx_send_uevent(struct iaxxx_priv *priv, char *type);

static int iaxxx_fw_bootup_regmap_init(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	int rc;

	/* Initialize "application" regmap */
	rc = iaxxx_application_regmap_init(priv);
	if (rc)
		goto err_regmap;
	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_switch_regmap(dev, priv->regmap, priv->dfs_node);
	if (rc)
		dev_err(dev, "Failed to create debugfs entry\n");
	else
		dev_info(dev, "%s: done\n", __func__);
	return rc;

err_regmap:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}
	return rc;
}

static int iaxxx_fw_recovery_regmap_init(struct iaxxx_priv *priv)
{
	int rc;

	/* Reinitialize the regmap cache */
	rc = regmap_reinit_cache(priv->regmap, priv->regmap_config);
	if (rc) {
		dev_err(priv->dev,
			"regmap cache can not be reinitialized %d\n", rc);
		regcache_cache_bypass(priv->regmap, true);
	}

	/* Clear system state */
	memset(priv->iaxxx_state, 0, sizeof(struct iaxxx_system_state));

	dev_info(priv->dev, "%s: Recovery done\n", __func__);

	return 0;
}

static int iaxxx_cell_force_resume(struct device *dev, void *data)
{
	pm_runtime_force_resume(dev);
	return 0;
}

static int iaxxx_cell_force_suspend(struct device *dev, void *data)
{
	pm_runtime_force_suspend(dev);
	return 0;
}

static void iaxxx_pm_enable(struct iaxxx_priv *priv)
{
	int ret = 0;

	priv->in_suspend = 0;
	priv->in_resume = 0;
	ret = pm_runtime_set_active(priv->dev);
	if (ret < 0)
		pr_err("pm_runtime_set_active fail %d\n", ret);
	pm_runtime_enable(priv->dev);

	pm_runtime_set_autosuspend_delay(priv->dev, IAXXX_PM_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(priv->dev);
	pm_runtime_mark_last_busy(priv->dev);

}

static int iaxxx_do_suspend(struct iaxxx_priv *priv,
			bool is_fw_crash)
{
	struct device *dev = priv->dev;
	unsigned long *flags = &priv->flags;
	bool suspended = false;

	/* Send broadcast event to all device children for crash */
	if (is_fw_crash)
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_CRASH, NULL);

	/* This means suspend is forced, for eg: pm_enable attribute */
	if (!(test_and_set_bit(IAXXX_FLG_PM_SUSPEND, flags))) {
		device_for_each_child(dev, priv, iaxxx_cell_force_suspend);
		pm_runtime_force_suspend(dev);
		suspended = true;
	}

	/* Start FW recovery if pending and is allowed, otherwise remember */
	if (is_fw_crash)
		iaxxx_work(priv, fw_crash_work);

	/* Send suspend event to user space */
	if (suspended)
		iaxxx_send_uevent(priv, "ACTION=IAXXX_SUSPEND_EVENT");

	return 0;
}

static int iaxxx_do_resume(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	unsigned long *flags = &priv->flags;
	bool resumed = false;

	if (test_and_clear_bit(IAXXX_FLG_PM_SUSPEND, flags)) {
		pm_runtime_force_resume(dev);
		device_for_each_child(dev, priv, iaxxx_cell_force_resume);
		resumed = true;
	}

	if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_STARTUP, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_STARTUP, NULL);

		/* Send firmware startup event to HAL */
		iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_STARTUP_EVENT");
		iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_DWNLD_SUCCESS");
	} else if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_RECOVERY, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_RECOVERY, NULL);

		/* Send recovery event to HAL */
		iaxxx_send_uevent(priv, "ACTION=IAXXX_RECOVERY_EVENT");
	}

	/* Send resume event to user space */
	if (resumed)
		iaxxx_send_uevent(priv, "ACTION=IAXXX_RESUME_EVENT");

	return 0;
}

int iaxxx_pm_get_sync(struct device *dev)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int ret = 0;

	if (priv == NULL) {
		dev_err(dev, "%s dev is NULL here\n", __func__);
		return -EINVAL;
	}

	if (!pm_runtime_enabled(dev))
		return 0;
	if (priv->in_suspend || priv->in_resume)
		return 0;
	if (mutex_trylock(&priv->pm_mutex))
		ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		dev_err(dev, "%s() Fail. %d\n", __func__, ret);

	mutex_unlock(&priv->pm_mutex);

	return ret;
}

int iaxxx_pm_put_autosuspend(struct device *dev)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int ret = 0;

	if (priv == NULL) {
		dev_err(dev, "%s dev is NULL here\n", __func__);
		return -EINVAL;
	}

	if (!pm_runtime_enabled(dev))
		return 0;
	if (priv->in_suspend || priv->in_resume)
		return 0;
	if (mutex_trylock(&priv->pm_mutex)) {
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_sync_autosuspend(dev);
		if (ret && ret != -EBUSY)
			dev_err(dev, "%s(): fail %d\n", __func__, ret);
	}
	mutex_unlock(&priv->pm_mutex);
	if (ret == -EBUSY)
		ret = 0;
	return ret;
}

int iaxxx_pm_put_sync_suspend(struct device *dev)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int ret = 0;

	if (priv == NULL) {
		dev_err(dev, "%s dev is NULL here\n", __func__);
		return -EINVAL;
	}

	if (!pm_runtime_enabled(dev))
		return 0;

	if (mutex_trylock(&priv->pm_mutex)) {
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_sync_suspend(dev);
		if (ret)
			dev_err(dev, "%s(): fail %d\n", __func__, ret);
		mutex_unlock(&priv->pm_mutex);
	}
	return ret;
}
/**
 * iaxxx_gpio_free - free a gpio for a managed device
 * @dev: device to free the gpio for
 * @gpio: GPIO to free
 */
static inline void iaxxx_gpio_free(struct device *dev, unsigned int gpio)
{
	devm_gpio_free(dev, gpio);
	dev_dbg(dev, "%s(): %d\n", __func__, gpio);
}

/**
 * get_named_gpio - Gets a GPIO property from a device tree node
 */
static inline int get_named_gpio(struct device *dev, const char *name)
{
	int rc;

	rc = of_get_named_gpio(dev->of_node, name, 0);
	if (rc < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			name, dev->of_node->full_name, rc);
		return rc;
	}
	dev_dbg(dev, "%s: %s %d\n", __func__, name, rc);
	return rc;
}

/**
 * iaxxx_populate_dt_gpios - populates GPIO data from device tree
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_populate_dt_gpios(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	rc = get_named_gpio(dev, "adnc,reset-gpio");
	if (rc < 0) {
		priv->reset_gpio = -EINVAL;
		dev_err(dev, "Failed to read reset-gpio, rc = %d\n", rc);
		return rc;
	}
	priv->reset_gpio = rc;

	rc = get_named_gpio(dev, "adnc,event-gpio");
	if (rc < 0) {
		priv->event_gpio = -EINVAL;
		dev_err(dev, "Failed to read event-gpio gpio, rc = %d\n", rc);
		return rc;
	}
	priv->event_gpio = rc;

	rc = get_named_gpio(dev, "adnc,pwr-vld-gpio");
	if (rc < 0) {
		priv->pwr_vld_gpio = -EINVAL;
		dev_err(dev, "Failed to read pwr-vld-gpio gpio, rc = %d\n", rc);
		return rc;
	}
	priv->pwr_vld_gpio = rc;

	return 0;
}

/**
 * iaxxx_populate_dt_regulator - populates regulator data from device tree
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_populate_dt_regulator(struct iaxxx_priv *priv,
		struct regulator **vreg, const char *reg_name)
{
	int rc;
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct regulator *reg;
	char prop_name[32];
	u32 volt_uV[2], load_uA;

	dev_dbg(dev, "%s()\n", __func__);

	if (!reg_name)
		return -EINVAL;

	/* check if the regulator consumer node exists */
	snprintf(prop_name, sizeof(prop_name), "%s-supply", reg_name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		dev_info(dev, "%s() property %s-supply not found.\n",
			__func__, reg_name);
		return 0;
	}

	/* get regulator */
	reg = devm_regulator_get(dev, reg_name);
	if (IS_ERR(reg)) {
		rc = PTR_ERR(reg);
		dev_err(dev, "%s: Failed to get regulator %s. rc=%d\n",
			__func__, reg_name, rc);
		goto err;
	}

	/* Read and set min/max regulator voltage */
	snprintf(prop_name, sizeof(prop_name), "%s-voltage-uV", reg_name);
	if (!of_property_read_u32_array(np, prop_name, volt_uV, 2)) {
		rc = regulator_set_voltage(reg, volt_uV[0], volt_uV[1]);
		if (rc < 0) {
			dev_err(dev, "%s: Failed to set voltage %s. rc=%d\n",
				__func__, reg_name, rc);
			goto err_put;
		}
	}

	/* Read and set max current load for this consumer */
	snprintf(prop_name, sizeof(prop_name), "%s-maxload-uA", reg_name);
	if (!of_property_read_u32(np, prop_name, &load_uA)) {
		rc = regulator_set_load(reg, load_uA);
		if (rc < 0) {
			dev_err(dev, "%s: Failed to set load %s. rc=%d\n",
				__func__, reg_name, rc);
			goto err_put;
		}
	}

	rc = regulator_enable(reg);
	if (rc < 0) {
		dev_err(dev,
			"Failed to enable regulator %s: %d\n", reg_name, rc);
		goto err_put;
	}

	dev_dbg(dev, "%s: regulator %s ready\n",
		__func__, reg_name);

	*vreg = reg;
	return 0;

err_put:
	devm_regulator_put(reg);
err:
	*vreg = NULL;
	return rc;
}

static int iaxxx_config_regulators(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	/* check and config vdd-io */
	rc = iaxxx_populate_dt_regulator(
			priv, &priv->vdd_io, "adnc,vdd-io");
	if (rc) {
		dev_err(dev,
			"Failed to configure regulator vdd_io, rc %d\n", rc);
		return rc;
	}

	/* Sleep for vdd_io to be stable before enabling vdd_core */
	if (priv->vdd_io)
		usleep_range(1000, 2000);

	/* check and config vdd-core */
	rc = iaxxx_populate_dt_regulator(
			priv, &priv->vdd_core, "adnc,vdd-core");
	if (rc) {
		dev_err(dev,
			"Failed to configure regulator vdd_core, rc %d\n", rc);
		return rc;
	}

	/* check and config vdd-oslo */
	rc = iaxxx_populate_dt_regulator(
			priv, &priv->vdd_oslo, "adnc,vdd-oslo");
	if (rc) {
		dev_err(dev,
			"Failed to configure regulator vdd_oslo, rc %d\n", rc);
		return rc;
	}

	return 0;
}
/**
 * iaxxx_populate_dt_pdata - populate platform data from device tree
 */
static int iaxxx_populate_dt_pdata(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	if (!dev->of_node) {
		dev_err(dev, "Missing device tree information\n");
		return -ENODEV;
	}

	rc = iaxxx_populate_dt_gpios(priv);
	if (rc) {
		dev_err(dev, "Failed to read GPIO data, rc = %d\n", rc);
		return rc;
	}

	return 0;
}

/**
 * iaxxx_gpio_init(): Requests the GPIO for the device
 */
static int iaxxx_gpio_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	/* GPIO: event (input used for interrupts) */
	/* TODD: interrupt is active HIGH so the GPIO needs pull-down */
	rc = devm_gpio_request_one(dev, priv->event_gpio,
						GPIOF_DIR_IN, "EVENT");
	if (rc < 0)
		goto err_missing_event_gpio;

	/* GPIO: reset (output used for chip reset) */
	rc = devm_gpio_request_one(dev, priv->reset_gpio,
						GPIOF_OUT_INIT_HIGH, "RESET");
	if (rc < 0)
		goto err_missing_reset_gpio;

	/* GPIO: pwr_vld (output used for chip reset) */
	rc = devm_gpio_request_one(dev, priv->pwr_vld_gpio,
						GPIOF_OUT_INIT_HIGH, "RESET");
	if (rc < 0)
		goto err_missing_pwr_vld_gpio;

	return 0;
err_missing_pwr_vld_gpio:
	iaxxx_gpio_free(dev, priv->reset_gpio);
err_missing_reset_gpio:
	iaxxx_gpio_free(dev, priv->event_gpio);
err_missing_event_gpio:
	return rc;
}

static void dump_to_log(struct device *dev,
		struct iaxxx_reg_dump_priv *reg_dump,
		struct iaxxx_register_log *log)
{
	spin_lock(&reg_dump->ring_lock);
	/* Check if Log buffer has space, if not increment the tail index to
	 * get buffer to overwrite oldest data
	 */
	if (!(CIRC_SPACE(reg_dump->head, reg_dump->tail, IAXXX_BUF_MAX_LEN))) {
		dev_dbg(dev, "%s() Register log buffer is full\n",
				__func__);
		reg_dump->tail++;
		reg_dump->tail %= IAXXX_BUF_MAX_LEN;
	}
	reg_dump->log[reg_dump->head] = *log;
	reg_dump->head++;
	/* Align with the Ring Buffer boundary */
	reg_dump->head %= IAXXX_BUF_MAX_LEN;
	spin_unlock(&reg_dump->ring_lock);
}

void register_transac_log(struct device *dev, uint32_t reg, uint32_t val,
		bool op)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_reg_dump_priv *reg_dump = priv->reg_dump;
	struct iaxxx_register_log log;

	if (!priv->reg_dump)
		return;
	if (!pm_runtime_active(dev))
		return;
	log.val = val;
	log.addr = reg;
	log.op = op;
	get_monotonic_boottime(&log.timestamp);
	/* Add the log into circular buffer */
	dump_to_log(dev, reg_dump, &log);
}

/**
 * iaxxx_reset(): Reset IAxxx device
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
static int iaxxx_reset(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s(): reset iaxxx\n", __func__);

	if (!gpio_is_valid(priv->reset_gpio)) {
		dev_err(dev, "%s(): reset_gpio(%d) is an invalid gpio.\n",
						__func__, priv->reset_gpio);
		return -EIO;
	}
	if (!gpio_is_valid(priv->pwr_vld_gpio)) {
		dev_err(dev, "%s(): pwr_vld_gpio(%d) is an invalid gpio.\n",
						__func__, priv->pwr_vld_gpio);
		return -EIO;
	}

	gpio_set_value(priv->pwr_vld_gpio, 0);

	gpio_set_value(priv->reset_gpio, 0);
	usleep_range(IAXXX_RESET_HOLD_TIME,
		IAXXX_RESET_HOLD_TIME + IAXXX_RESET_RANGE_INTERVAL);

	gpio_set_value(priv->reset_gpio, 1);
	usleep_range(IAXXX_RESET_PWR_VLD_DELAY, IAXXX_RESET_PWR_VLD_DELAY +
		     IAXXX_RESET_PWR_VLD_RANGE_INTERVAL);

	gpio_set_value(priv->pwr_vld_gpio, 1);
	usleep_range(IAXXX_RESET_READY_DELAY, IAXXX_RESET_READY_DELAY +
		IAXXX_RESET_RANGE_INTERVAL);

	return 0;
}

/**
 * iaxxx_reset_to_sbl - Boots the chip hardware
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
static int iaxxx_reset_to_sbl(struct iaxxx_priv *priv)
{
	int rc = 0;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	/* Reset the chip */
	rc = iaxxx_reset(priv);
	if (rc)
		dev_err(dev, "%s: device reset failed\n", __func__);

	return rc;
}

/**
 * iaxxx_event_isr - Interrupt / Event handler
 *
 * @irq	 : interrupt number
 * @data : iaxxx private data
 */
static irqreturn_t iaxxx_event_isr(int irq, void *data)
{
	int rc;
	uint32_t count;
	uint32_t status;
	int mode;
	bool handled = false;
	bool is_startup;
	struct iaxxx_priv *priv = (struct iaxxx_priv *)data;

	/* If ISR is disabled, return as handled */
	if (priv->debug_isr_disable)
		return IRQ_HANDLED;

	dev_dbg(priv->dev, "%s: IRQ %d\n", __func__, irq);

	if (!priv->boot_completed) {
		is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP,
						&priv->flags);
		rc = is_startup ? iaxxx_fw_bootup_regmap_init(priv) :
					iaxxx_fw_recovery_regmap_init(priv);
		if (rc)
			goto out;
	}

	/* Any events in the event queue? */
	rc = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read EVENT_COUNT, rc = %d\n", rc);
		/* Read should not fail recover the chip */
		count = 0;
	}

	if (count > 0 && priv->event_workq) {
		dev_dbg(priv->dev, "%s: %d event(s) avail\n", __func__, count);
		queue_work(priv->event_workq, &priv->event_work_struct);
		handled = true;
	} else {
		/* Read SYSTEM_STATUS to ensure that device is in App Mode */
		rc = regmap_read(priv->regmap,
					IAXXX_SRB_SYS_STATUS_ADDR, &status);
		if (rc)
			dev_err(priv->dev,
				"Failed to read SYSTEM_STATUS, rc = %d\n", rc);

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		if (mode != SYSTEM_STATUS_MODE_APPS) {
			dev_err(priv->dev,
				"Not in app mode CM4 might crashed, mode = %d\n",
				mode);
			priv->cm4_crashed = true;
			queue_work(priv->event_workq, &priv->event_work_struct);
			return IRQ_HANDLED;
		}
	}

	complete_all(&priv->cmem_done);
out:
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/**
 * Sysfs save isr_disable option
 */
static ssize_t iaxxx_debug_isr_disable_save(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		dev_info(dev, "%s() ISR Disabled\n", __func__);
		priv->debug_isr_disable  = true;
	} else {
		priv->debug_isr_disable = false;
		dev_info(dev, "%s() ISR Enabled\n", __func__);
	}
	return count;
}

/**
 * sysfs show isr disable option
 */
static ssize_t iaxxx_debug_isr_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
		priv->debug_isr_disable?"ISR Event Handling Disabled\n" :
		"ISR Event Handling Enabled\n");
}

static DEVICE_ATTR(debug_isr_disable, 0600,
		iaxxx_debug_isr_disable_show, iaxxx_debug_isr_disable_save);


/**
 * iaxxx_irq_init - Initialize interrupt handling
 *
 * @priv : iaxxx private data
 */
static int iaxxx_irq_init(struct iaxxx_priv *priv)
{
	int rc;

	if (!gpio_is_valid(priv->event_gpio))
		return -ENXIO;

	rc = request_threaded_irq(gpio_to_irq(priv->event_gpio), NULL,
			iaxxx_event_isr,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
			"iaxxx-event-irq", priv);
	if (rc)
		return rc;
	rc = enable_irq_wake(gpio_to_irq(priv->event_gpio));
	if (rc < 0)
		pr_err("%s: enable_irq_wake() failed on %d\n", __func__, rc);

	/* disable the irq until fw is loaded */
	disable_irq(gpio_to_irq(priv->event_gpio));
	priv->is_irq_enabled = false;

	return rc;
}

/**
 * iaxxx_irq_exit - Frees the event IRQ
 *
 * @priv	: iaxxx private data
 */
static void iaxxx_irq_exit(struct iaxxx_priv *priv)
{
	if (gpio_is_valid(priv->event_gpio))
		free_irq(gpio_to_irq(priv->event_gpio), priv);
}

static int iaxxx_regdump_init(struct iaxxx_priv *priv)
{
	dev_dbg(priv->dev, "%s()", __func__);

	priv->dump_log = true;
	priv->reg_dump = kzalloc(sizeof(struct iaxxx_reg_dump_priv),
			GFP_KERNEL);
	if (!priv->reg_dump)
		return -ENOMEM;

	priv->reg_dump->log = kzalloc(sizeof(struct iaxxx_register_log)
			* IAXXX_BUF_MAX_LEN, GFP_KERNEL);
	if (!priv->reg_dump->log) {
		kfree(priv->reg_dump);
		return -ENOMEM;
	}

	spin_lock_init(&priv->reg_dump->ring_lock);
	return 0;
}

void iaxxx_regdump_exit(struct iaxxx_priv *priv)
{
	kfree(priv->reg_dump->log);
	kfree(priv->reg_dump);
}

static int iaxxx_reset_check_sbl_mode(struct iaxxx_priv *priv)
{
	uint32_t status;
	int ret = 0;
	int mode = 0;
	int mode_retry = 5;

	if (priv->reset_cb)
		priv->reset_cb(priv->dev);

	do {
		/* Verify that the device is in bootloader mode */
		ret = regmap_read(priv->regmap,
				IAXXX_SRB_SYS_STATUS_ADDR, &status);
		if (ret)
			dev_err(priv->dev,
				"regmap_read failed, ret = %d\n", ret);

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_dbg(priv->dev,
			"System Status: 0x%.08X mode: 0x%.08X\n",
			status, mode);

		/* Give some time to device before read */
		usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);
	} while (!mode && mode_retry--);

	priv->boot_completed =  false;
	if (!mode && !mode_retry) {
		dev_err(priv->dev,
			"SBL SYS MODE retry expired in crash dump\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

/**
 * iaxxx_send_uevent - Send uevent KOBJ_CHANGE
 *
 * @priv	: iaxxx private data
 * @type	  Type of event
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_send_uevent(struct iaxxx_priv *priv, char *type)
{
	char *event[2] = {type, NULL};

	/* Send recovery event to HAL */
	return kobject_uevent_env(&priv->dev->kobj, KOBJ_CHANGE, event);
}

/**
 * iaxxx_do_fw_update - reset and sync target and do firmware update
 *
 * @priv	: iaxxx private data
 * Return
 *       0                    Success
 *       E_IAXXX_REGMAP_ERROR error accessing regmap
 *       E_IAXXX_BOOTUP_ERROR target bootup failure
 *
 */
static int iaxxx_do_fw_update(struct iaxxx_priv *priv)
{
	int rc;
	uint32_t reg, mem_elec_ctrl_val;
	struct device *dev = priv->dev;

	rc = iaxxx_reset_check_sbl_mode(priv);
	if (rc) {
		dev_err(dev, "SBL sysmode check failed, rc = %d\n", rc);
		return rc;
	}

	/* Get and log the Device ID */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_DEVICE_ID_ADDR, &reg);
	if (rc) {
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_dbg(dev, "Device ID: 0x%.08X\n", reg);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_ROM_VER_NUM_ADDR, &reg);
	if (rc) {
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_dbg(dev, "ROM Version: 0x%.08X\n", reg);


	/* Electrical control for Memory
	 * Value 2(Normal-Read 2) for read and write margins for ROMs.
	 * Value 0(Slowest Read, Slowest Write) for read and write margins
	 * for SRAMs
	 * Value 8(Medium Leakage) for Retention Voltage for PD8 memories.
	 *
	 *  This fix is required for FW download on FF part or else we get
	 *  checksum error after downloading it. It fixes chip
	 *  stability issues due to clock changes and heating issues.
	 *
	 */
	mem_elec_ctrl_val =
		((2 << IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_ROM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_ROM_MASK) |
		((0 << IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_RAM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_RD_WR_MARGIN_ADJ_RAM_MASK) |
		((8 << IAXXX_AO_MEM_ELEC_CTRL_PD8_BTRIM_POS) &
		IAXXX_AO_MEM_ELEC_CTRL_PD8_BTRIM_MASK);

	rc = regmap_write(priv->regmap, IAXXX_AO_MEM_ELEC_CTRL_ADDR,
			mem_elec_ctrl_val);
	if (rc) {
		dev_err(dev, "Electrical control register failed, rc = %d\n",
				rc);
		return E_IAXXX_REGMAP_ERROR;
	}

	/* Boot the device into application mode */
	rc = iaxxx_bootup(priv);
	if (rc) {
		dev_err(dev, "bootup failed\n");
		return E_IAXXX_BOOTUP_ERROR;
	}

	/* Call SPI speed setup callback if exists */
	if (priv->spi_speed_setup)
		priv->spi_speed_setup(dev, priv->spi_app_speed);

	return 0;
}

static int iaxxx_dump_crashlogs(struct iaxxx_priv *priv)
{
	uint32_t buf_size;
	uint32_t data_written = 0;
	int i;
	uint32_t log_addr;
	uint32_t log_size;
	int ret;

	/* If memory already allocated */
	kfree(priv->crashlog->log_buffer);
	priv->crashlog->log_buffer = NULL;
	/* Calculate total crash log dump size */
	buf_size = sizeof(struct iaxxx_crashlog_header) * IAXXX_MAX_LOG;
	for (i = 0; i < IAXXX_MAX_LOG; i++)
		buf_size += priv->crashlog->header[i].log_size;
	priv->crashlog->log_buffer_size = buf_size;
	/* Allocate the memory */
	priv->crashlog->log_buffer = kzalloc(buf_size, GFP_KERNEL);
	if (!priv->crashlog->log_buffer)
		return -ENOMEM;

	/* Collect the crashlogs into log buffer */
	for (i = 0; i < IAXXX_MAX_LOG; i++) {
		/* Copy header information */
		memcpy(priv->crashlog->log_buffer + data_written,
				&priv->crashlog->header[i],
				sizeof(struct iaxxx_crashlog_header));
		data_written += sizeof(struct iaxxx_crashlog_header);
		log_addr = priv->crashlog->header[i].log_addr;
		log_size = priv->crashlog->header[i].log_size;
		/* If size of the log is 0 */
		if (!log_size)
			continue;
		/* Read the logs */
		ret = priv->bulk_read(priv->dev,
			log_addr, priv->crashlog->log_buffer + data_written,
			log_size / sizeof(uint32_t));
		if (ret != log_size / sizeof(uint32_t)) {
			dev_err(priv->dev, "Not able to read Debug logs %d\n",
					ret);
			return ret;
		}
		data_written += log_size;
	}
	dev_dbg(priv->dev, "Data written 0x%x\n", data_written);
	return 0;
}

static void iaxxx_crashlog_header_read(struct iaxxx_priv *priv)
{
	int i;
	int j = 0;
	int ret;

	/* Reading the debug log address and size */
	for (i = 0; i < IAXXX_MAX_LOG / 2 ; i++) {
		priv->crashlog->header[i].log_type = i;
		ret = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUGLOG_BLOCK_0_DEBUGLOG_ADDR_ADDR + i * 8,
			&priv->crashlog->header[i].log_addr, 2);
		if (ret) {
			dev_err(priv->dev, "Log %d address fail %d\n", i, ret);
			priv->crashlog->header[i].log_addr = 0;
			priv->crashlog->header[i].log_size = 0;
		}
	}
	/* Reading the crash log address and size */
	for (i = IAXXX_CRASHLOG_CM4; i < IAXXX_MAX_LOG; i++) {
		priv->crashlog->header[i].log_type = i;
		ret = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUGLOG_BLOCK_0_CRASHLOG_ADDR_ADDR + j * 8,
			&priv->crashlog->header[i].log_addr, 2);
		j++;
		if (ret) {
			dev_err(priv->dev, "Log %d address fail %d\n", i, ret);
			priv->crashlog->header[i].log_addr = 0;
			priv->crashlog->header[i].log_size = 0;
		}
	}
	for (i = 0; i < IAXXX_MAX_LOG; i++)
		dev_dbg(priv->dev, "addr 0x%x size 0x%x\n",
				priv->crashlog->header[i].log_addr,
				priv->crashlog->header[i].log_size);
}


/**
 * iaxxx_fw_update_work - worker thread to download firmware.
 *
 * @work : used to retrieve private structure
 *
 * Firmware download and switch to application mode of firmware.
 *
 */

static void iaxxx_fw_update_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_update_work);
	struct device *dev = priv->dev;
	bool is_startup;
	int rc;

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);
	clear_bit(IAXXX_FLG_BUS_BLOCK_CORE, &priv->flags);

	rc = iaxxx_do_fw_update(priv);
	if (rc == E_IAXXX_REGMAP_ERROR) {
		goto exit_fw_fail;
	} else if (rc == E_IAXXX_BOOTUP_ERROR) {
		/* If there's device reset cb, retry */
		if (!priv->reset_cb) {
			dev_err(dev, "%s: Chip failed to boot up\n", __func__);
			goto exit_fw_fail;
		}

		if (++priv->try_count < IAXXX_FW_RETRY_COUNT) {
			dev_err(dev, "%s: Bootup error. retrying... %d\n",
				__func__, priv->try_count);
			iaxxx_work(priv, fw_update_work);
			return;
		}

		dev_err(dev, "%s: %d retry failed! EXIT\n",
			__func__, IAXXX_FW_RETRY_COUNT);
		goto exit_fw_fail;
	}

	priv->try_count = 0;

	if (!priv->boot_completed) {
		is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP,
						&priv->flags);
		rc = is_startup ? iaxxx_fw_bootup_regmap_init(priv) :
					iaxxx_fw_recovery_regmap_init(priv);
		if (rc)
			goto exit_fw_fail;
		priv->boot_completed = true;
	}

	/* Send firmware ready event to HAL */
	iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_READY_EVENT");

	iaxxx_crashlog_header_read(priv);

	/* Subscribing for FW crash event */
	rc = iaxxx_core_evt_subscribe(dev, IAXXX_CM4_CTRL_MGR_SRC_ID,
				IAXXX_CRASH_EVENT_ID, IAXXX_SYSID_HOST, 0);
	if (rc) {
		dev_err(dev, "%s: failed to subscribe for crash event\n",
			__func__);
		goto exit_fw_fail;
	}

	set_bit(IAXXX_FLG_FW_READY, &priv->flags);

	iaxxx_fw_notifier_call(dev, IAXXX_EV_APP_MODE, NULL);

	if (test_and_clear_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		set_bit(IAXXX_FLG_RESUME_BY_RECOVERY, &priv->flags);
	else
		set_bit(IAXXX_FLG_RESUME_BY_STARTUP, &priv->flags);

	dev_info(dev, "%s: done\n", __func__);
	iaxxx_work(priv, runtime_work);
	iaxxx_pm_enable(priv);
	return;

exit_fw_fail:
	/* Send firmware fail uevent to HAL */
	iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_FAIL_EVENT");

	/* Clear try counter */
	priv->try_count = 0;

}

static void iaxxx_fw_crash_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_crash_work);
	int ret;
	uint32_t core_crashed = 0;

	priv->crash_count++;
	dev_info(priv->dev, "iaxxx %d time crashed\n",
			priv->crash_count);

	/* Clear event queue */
	if (gpio_is_valid(priv->event_gpio) && priv->is_irq_enabled) {
		disable_irq(gpio_to_irq(priv->event_gpio));
		priv->is_irq_enabled = false;
	}

	mutex_lock(&priv->event_queue_lock);
	priv->event_queue->w_index = -1;
	priv->event_queue->r_index = -1;
	mutex_unlock(&priv->event_queue_lock);

	priv->route_status = 0;

	if (priv->cm4_crashed) {
		dev_info(priv->dev, "CM4 Core crashed\n");
		priv->cm4_crashed = false;
	} else {
		ret = regmap_read(priv->regmap,
				IAXXX_SRB_PROCESSOR_CRASH_STATUS_ADDR,
				&core_crashed);
		/* Crash status read fails, means CM4 core crashed */
		if (ret) {
			dev_info(priv->dev,
				"Crash status read fail %s()\n", __func__);
			core_crashed = IAXXX_CM4_ID;
		}
		if (core_crashed == IAXXX_CM4_ID) {
			dev_info(priv->dev, "D4100S CM4 core crashed\n");
			priv->cm4_crashed = false;
		} else if (core_crashed == 1 << IAXXX_HMD_ID)
			dev_info(priv->dev, "D4100S HMD core crashed\n");
		else if (core_crashed == 1 << IAXXX_DMX_ID)
			dev_info(priv->dev, "D4100S DMX Core Crashed\n");
		else
			dev_info(priv->dev, "D4100S Update block failed\n");
	}

	mutex_lock(&priv->crashdump_lock);
	iaxxx_reset_check_sbl_mode(priv);
	iaxxx_dump_crashlogs(priv);
	mutex_unlock(&priv->crashdump_lock);

	/* Notify the user about crash and read crash dump log*/
	iaxxx_send_uevent(priv, "ACTION=IAXXX_CRASH_EVENT");
	/* Bypass regmap cache */
	regcache_cache_bypass(priv->regmap, true);
	iaxxx_work(priv, fw_update_work);
}

static void iaxxx_runtime_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, runtime_work);
	unsigned long *flags = &priv->flags;
	bool is_fw_crash, do_suspend;

	/* Suspend due FW crash pending or aborted recovery */
	is_fw_crash = test_bit(IAXXX_FLG_FW_CRASH, flags);

	do_suspend = is_fw_crash;

	if (do_suspend)
		iaxxx_do_suspend(priv, is_fw_crash);
	else
		iaxxx_do_resume(priv);
}

/**
 * iaxxx_fw_update_test_work - work thread for running firmware test
 *
 * @work : used to retrieve Transport Layer private structure
 * Set the regmap to sbl mode and firmware update
 * If firmware update successful, then set bootup_done
 *
 */
static void iaxxx_fw_update_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_fw_update_test_work);
	int rc;

	priv->test_result = false;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);

	if (!rc)
		priv->test_result = true;

	complete_all(&priv->bootup_done);
}

int iaxxx_fw_crash(struct device *dev, enum iaxxx_fw_crash_reasons reasons)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	dev_err(priv->dev, "FW Crash occurred, reasons %d (%s)\n",
		reasons, iaxxx_crash_err2str(reasons));

	/* Avoid second times if currently is handled */
	if (test_and_set_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EBUSY;

	priv->fw_crash_reasons = reasons;
	iaxxx_work(priv, runtime_work);
	return 0;
}

/*
 * iaxxx_abort_fw_recovery - abort current FW loading works
 *
 * @priv - context structure of driver
 *
 * FW Recovery procedure take more then 4sec.  When entering privacy mode,
 * we block the notifier call chain for the entire suspend time. Current
 * FW load procedure must be aborted. And started again when privacy complete
 *
 */
static int iaxxx_abort_fw_recovery(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	/* Not need abort if device is in active state */
	if (pm_runtime_enabled(dev) && pm_runtime_active(dev))
		return -EPERM;

	if (!test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EPERM;

	dev_info(dev, "Aborting FW recovery...\n");

	set_bit(IAXXX_FLG_BUS_BLOCK_CORE, &priv->flags);
	iaxxx_work_flush(priv, runtime_work);
	iaxxx_work_flush(priv, fw_crash_work);
	iaxxx_work_flush(priv, fw_update_work);
	clear_bit(IAXXX_FLG_BUS_BLOCK_CORE, &priv->flags);

	dev_info(dev, "FW recovery aborted!\n");

	return 0;
}

/**
 * iaxxx_cmem_test_work - work thread for running memory test
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * Wait for an event interrupt happening with timeout
 *
 */
static void iaxxx_cmem_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_cmem_test_work);
	int rc;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);
}

/**
 * iaxxx_test_init - initialize work items and mutex for test environment
 *
 * @priv        : iaxxx private data
 *
 */
static void iaxxx_test_init(struct iaxxx_priv *priv)
{
	INIT_WORK(&priv->dev_fw_update_test_work, iaxxx_fw_update_test_work);
	INIT_WORK(&priv->dev_cmem_test_work, iaxxx_cmem_test_work);
	mutex_init(&priv->test_mutex);
	init_completion(&priv->bootup_done);
	init_completion(&priv->cmem_done);
}

static int get_version_str(struct iaxxx_priv *priv, uint32_t reg, char *verbuf,
								uint32_t len)
{
	int rc;
	uint32_t addr;
	uint32_t i = 0;

	if (priv == NULL || priv->regmap == NULL) {
		dev_err(priv->dev, "%s() regmap is not initialized yet.\n",
			__func__);
		return -EIO;
	}

	/* Read the version string address */
	rc = regmap_read(priv->regmap, reg, &addr);
	if (rc) {
		dev_err(priv->dev, "%s() String address read failed %d\n",
							__func__, rc);
		return rc;
	}
	pr_debug("%s() String address 0x%x\n", __func__, addr);

	/* Read the FW string from address read above */
	while (len > 0) {
		rc = priv->bulk_read(priv->dev, addr + i, &verbuf[i], 1);
		if (rc != 1) {
			dev_err(priv->dev, "String Read fail addr 0x%x:%d\n",
							addr + i, rc);
			return -EIO;
		}
		/* Reached NULL character, FW version string ends here */
		if ((!verbuf[i]) || (!verbuf[i + 1]) ||
				(!verbuf[i + 2]) || (!verbuf[i + 3])) {
			pr_debug("%s() String ends here\n", __func__);
			i += IAXXX_BYTES_IN_A_WORD;
			break;
		}
		/* 4 characters read, go for next 4 bytes to read */
		len -= IAXXX_BYTES_IN_A_WORD;
		i += IAXXX_BYTES_IN_A_WORD;
	}

	verbuf[IAXXX_VER_STR_SIZE - 1] = '\0';
	print_hex_dump(KERN_INFO, "Version: ", DUMP_PREFIX_OFFSET, 32, 4,
			(void *)verbuf, i, true);
	/*
	 * If not reached end of buffer and buffer is not empty,
	 * then print Firmware version.
	 */
	if (len > 0 && verbuf[0] != '\0')
		return 0;
	return -EIO;
}

/**
 * iaxxx_firmware_version_show - sys node show function for firmware version
 */
static ssize_t iaxxx_firmware_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}

	rc = get_version_str(priv, IAXXX_SRB_SYS_APP_VER_STR_ADDR, verbuf,
									len);
	if (rc) {
		dev_err(dev, "%s() FW version read fail\n", __func__);
		return -EIO;
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", verbuf);
}
static DEVICE_ATTR(fw_version, 0400, iaxxx_firmware_version_show, NULL);


/**
 * iaxxx_host_version_show - sys node show function hsw veriso
 * and fw version built with
 */
static ssize_t iaxxx_host_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"HSW v%s built with Firmware v%s\n",
			HOST_SOFTWARE_VERSION_STR, FW_VERSION_IN_HOST_STR);
}
static DEVICE_ATTR(host_version, 0400, iaxxx_host_version_show, NULL);

/*
 * Sysfs - firmware update
 */
static ssize_t iaxxx_fw_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	/* Kick work queue for firmware loading */
	iaxxx_work(priv, fw_update_work);
	return count;
}
static DEVICE_ATTR(fw_update, 0200, NULL, iaxxx_fw_update);


static ssize_t iaxxx_plugin_version_plugin_index_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val >= IAXXX_MAX_PLUGIN)
		return -EINVAL;

	priv->plugin_version_plugin_index = val;
	dev_info(dev, "%s() plugin_version_plugin_index:%d\n",
		__func__, priv->plugin_version_plugin_index);

	return count;
}

/**
 * iaxxx_plugin_version_show - sys node show function for plugin version
 */
static ssize_t iaxxx_plugin_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	uint32_t buf_len = 0;

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}

	if (priv->plugin_version_plugin_index != -EINVAL) {
		rc = get_version_str(
				priv,
				IAXXX_PLUGIN_INS_GRP_PLUGIN_VER_STR_REG(
					priv->plugin_version_plugin_index),
				verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Plugin version read fail\n",
				__func__);
			return -EIO;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE,
				"plugin-%d:\t%s\n",
				priv->plugin_version_plugin_index, verbuf);
		priv->plugin_version_plugin_index = -EINVAL;
	}
	return buf_len;
}
static DEVICE_ATTR(plugin_version, 0600, iaxxx_plugin_version_show,
		iaxxx_plugin_version_plugin_index_store);

static ssize_t iaxxx_package_version_package_index_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val >= IAXXX_MAX_PACKAGE)
		return -EINVAL;

	priv->package_version_package_index = val;
	dev_info(dev, "%s() package_version_package_index:%d\n",
			__func__, priv->package_version_package_index);

	return count;
}

/**
 * iaxxx_package_version_show - sys node show function for package version
 */
static ssize_t iaxxx_package_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	uint32_t buf_len = 0;

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}

	if (priv->package_version_package_index != -EINVAL) {
		rc = get_version_str(
			priv, IAXXX_PLUGIN_INS_GRP_PACKAGE_VER_STR_REG(
				priv->package_version_package_index),
			verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Package version read fail\n",
						__func__);
			return -EIO;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE,
				"package-%d:\t%s\n",
				priv->package_version_package_index, verbuf);
		priv->package_version_package_index = -EINVAL;
	}
	return buf_len;
}
static DEVICE_ATTR(package_version, 0600, iaxxx_package_version_show,
		iaxxx_package_version_package_index_store);

/**
 * iaxxx_firmware_timestamp_show - sys node show function for firmware timestamp
 */
static ssize_t iaxxx_firmware_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	uint32_t fw_clk_rd[2];
	uint64_t timestamp;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "Invalid device\n");

	/* Read Firmware wall clock timestamp */
	rc = regmap_bulk_read(priv->regmap,
			IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR,
			fw_clk_rd, ARRAY_SIZE(fw_clk_rd));
	if (rc) {
		dev_err(dev,
			"Failed IAXXX_AF_WCPT_WALL_CLOCK_RD, rc:%d\n", rc);
		return rc;
	}
	timestamp = (((long)((fw_clk_rd[1] & 0xFFFF)) << 32) | fw_clk_rd[0]);

	return scnprintf(buf, PAGE_SIZE, "0x%llx\n", timestamp);
}
static DEVICE_ATTR(fw_timestamp, 0400, iaxxx_firmware_timestamp_show, NULL);

/**
 * iaxxx_firmware_update_test_show - sys node show function for firmware test
 *
 * Trigger firmware update test work and return results
 *
 */
static ssize_t iaxxx_firmware_update_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	long rc;
	ssize_t count;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	mutex_lock(&priv->test_mutex);

	cancel_work_sync(&priv->dev_fw_update_test_work);
	init_completion(&priv->bootup_done);
	schedule_work(&priv->dev_fw_update_test_work);

	rc = wait_for_completion_interruptible_timeout(&priv->bootup_done,
			msecs_to_jiffies(IAXXX_FW_DOWNLOAD_TIMEOUT));

	if (rc > 0 && priv->test_result)
		count = scnprintf(buf, PAGE_SIZE, "SUCCESS\n");
	else
		count = scnprintf(buf, PAGE_SIZE, "FAIL\n");

	mutex_unlock(&priv->test_mutex);

	return count;
}
static DEVICE_ATTR(fw_update_test, 0400, iaxxx_firmware_update_test_show,
			NULL);

/**
 * iaxxx_cmem_test_show - sys node show function for memory test
 *
 * Trigger firmware update test work and return results
 *
 */
static ssize_t iaxxx_cmem_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	long rc;
	ssize_t count;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	mutex_lock(&priv->test_mutex);

	cancel_work_sync(&priv->dev_cmem_test_work);
	init_completion(&priv->cmem_done);
	schedule_work(&priv->dev_cmem_test_work);

	rc = wait_for_completion_interruptible_timeout(&priv->cmem_done,
			msecs_to_jiffies(IAXXX_FW_DOWNLOAD_TIMEOUT));
	if (rc > 0)
		count = scnprintf(buf, PAGE_SIZE, "SUCCESS\n");
	else
		count = scnprintf(buf, PAGE_SIZE, "FAIL\n");

	mutex_unlock(&priv->test_mutex);

	return count;
}
static DEVICE_ATTR(cmem_test, 0400, iaxxx_cmem_test_show, NULL);

static int iaxxx_event_flush_and_enable(struct iaxxx_priv *priv)
{
	mutex_lock(&priv->event_queue_lock);
	/* Clear event queue */
	priv->event_queue->w_index = -1;
	priv->event_queue->r_index = -1;
	mutex_unlock(&priv->event_queue_lock);

	if (gpio_is_valid(priv->event_gpio) && !priv->is_irq_enabled) {
		enable_irq(gpio_to_irq(priv->event_gpio));
		priv->is_irq_enabled = true;
	}

	/* Clients regmap access must be enabled here */
	queue_work(priv->event_workq, &priv->event_work_struct);

	return 0;
}

static int iaxxx_wakeup_chip(struct iaxxx_priv *priv)
{
	int rc, reg_val, reg_addr;

	if (priv->disable_chip_pm) {
		dev_err(priv->dev, "%s chip pm disabled\n", __func__);
		return 0;
	}

	/* SPI_CS is being would act as wake up source. So making an
	 * SPI transaction to wake up the chip and then wait for 20ms
	 */
	reg_addr = IAXXX_SRB_SYS_STATUS_ADDR;
	rc = priv->read_no_pm(priv->dev, &reg_addr, sizeof(uint32_t),
			&reg_val, sizeof(uint32_t));
	msleep(40);

	reg_addr = IAXXX_SRB_SYS_POWER_CTRL_ADDR;
	rc = priv->read_no_pm(priv->dev, &reg_addr, sizeof(uint32_t),
			&reg_val, sizeof(uint32_t));

	reg_addr = IAXXX_SRB_SYS_STATUS_ADDR;
	rc = priv->read_no_pm(priv->dev, &reg_addr, sizeof(uint32_t),
			&reg_val, sizeof(uint32_t));

	/* if read failed or SYS mode is not in APP mode, flag error*/
	reg_val &= IAXXX_SRB_SYS_STATUS_MODE_MASK;
	dev_err(priv->dev, "%s chip wake up reg_val%d\n", __func__, reg_val);
	if (rc || (reg_val != SYSTEM_STATUS_MODE_APPS)) {
		dev_err(priv->dev, "%s chip wake up failed %d rc %d\n",
				__func__, reg_val, rc);
		return -EIO;
	}

	/*setting up the normal  power mode*/
	reg_addr = IAXXX_SRB_SYS_POWER_CTRL_ADDR;
	rc = priv->read_no_pm(priv->dev, &reg_addr, sizeof(uint32_t), &reg_val,
			sizeof(uint32_t));
	reg_val &= ~IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_MASK;
	reg_val |= (0x4 << IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_POS) &
			IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_MASK;

	rc = priv->write_no_pm(priv->dev, &reg_addr, sizeof(uint32_t),
			&reg_val, sizeof(uint32_t));
	if (rc) {
		dev_err(priv->dev, "%s() Fail\n", __func__);
		/*return rc; */
	}

	iaxxx_send_update_block_no_wait_no_pm(priv->dev, HOST_0);
	msleep(40);
	return 0;
}

static int iaxxx_suspend_chip(struct iaxxx_priv *priv)
{
	int rc, reg_addr, reg_val;

	if (priv->disable_chip_pm) {
		dev_err(priv->dev, "%s chip pm disabled\n", __func__);
		return 0;
	}

	/* system should go to sleep if there are no route active. But dynamic
	 * load/unload of plugins is not been verified yet. So for the time
	 * beging suspend would only moves to optimal low power mode
	 * (after disabling the control interface by both hosts)
	 */
	reg_addr = IAXXX_SRB_ARB_14_BASE_ADDR_ADDR;
	rc = priv->read_no_pm(priv->dev, &reg_addr, sizeof(uint32_t), &reg_val,
			sizeof(uint32_t));

	if (rc) {
		dev_err(priv->dev, "%s() Fail\n", __func__);
		return rc;
	}
	/* set up the SPI speed thats expected when the system is wake up */
	reg_addr = reg_val + (IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_ADDR & 0xffffff);
	reg_val =  priv->spi_app_speed;
	rc = priv->write_no_pm(priv->dev, &reg_addr, sizeof(uint32_t),
			&reg_val, sizeof(uint32_t));
	if (rc) {
		dev_err(priv->dev,
			"Failed to set Max SPI speed in %s\n", __func__);
		return rc;
	}

	/* Disable the control interface */
	reg_addr = IAXXX_SRB_SYS_POWER_CTRL_ADDR;
	rc = priv->read_no_pm(priv->dev, &reg_addr, sizeof(uint32_t), &reg_val,
			sizeof(uint32_t));
	reg_val &= ~IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK;
	reg_val |= (0x1 <<
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_POS) &
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK;
	rc = priv->write_no_pm(priv->dev, &reg_addr, sizeof(uint32_t),
			&reg_val, sizeof(uint32_t));

	if (rc) {
		dev_err(priv->dev, "%s() Fail\n", __func__);
		return rc;
	}

	iaxxx_send_update_block_no_wait_no_pm(priv->dev, HOST_0);
	msleep(20);
	dev_info(priv->dev, "%s() Success\n", __func__);

	return 0;
}


int iaxxx_core_suspend_rt(struct device *dev)
{
	int rc = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!test_bit(IAXXX_FLG_FW_READY, &priv->flags))
		/* return -EBUSY; */
		return 0;
	if (!pm_runtime_enabled(dev) && !test_bit(IAXXX_FLG_FW_CRASH,
		&priv->flags)) {
		dev_err(dev, "RT suspend requested while PM is not enabled\n");
		return -EINVAL;
	}
	/* this flag is to ensure that resume is not invoked from inside
	 * the suspend call back. Or it would cause a deadlock
	 */
	priv->in_suspend = 1;
	/* Chip suspend/optimal power switch happens here
	 * and they shouldn't be done in case of fw_crash
	 */
	if (!test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		rc = iaxxx_suspend_chip(priv);

	if  (rc) {
		dev_err(dev, " Chip suspend failed in %s\n", __func__);
		priv->in_suspend = 0;
		return rc;
	}
	set_bit(IAXXX_FLG_BUS_BLOCK_CLIENTS, &priv->flags);

	/* Block SPI transaction for iaxxx-core while being suspended */
	set_bit(IAXXX_FLG_BUS_BLOCK_CORE, &priv->flags);
	priv->in_suspend = 0;
	return 0;
}

int iaxxx_core_resume_rt(struct device *dev)
{
	int rc;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!test_bit(IAXXX_FLG_FW_READY, &priv->flags))
		/*return -EBUSY;*/
		return 0;

	priv->in_resume = 1;
	/* Regmap access must be enabled before enable event interrupt */
	clear_bit(IAXXX_FLG_BUS_BLOCK_CORE, &priv->flags);
	clear_bit(IAXXX_FLG_BUS_BLOCK_CLIENTS, &priv->flags);

	iaxxx_event_flush_and_enable(priv);

	rc = iaxxx_wakeup_chip(priv);
	if  (rc) {
		dev_err(dev, " Chip wakeup failed in %s\n", __func__);
		priv->in_resume = 0;
		return rc;
	}

	if (!pm_runtime_enabled(dev)) {
		/* This can happen during crash recovery, it's not an error
		 * condition
		 */
		dev_info(dev, "Resume requested while PM is not enabled\n");
	}

	priv->in_resume = 0;
	return 0;
}

int iaxxx_core_dev_suspend(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	iaxxx_flush_kthread_worker(&priv->worker);

	return iaxxx_core_suspend_rt(dev);
}

int iaxxx_core_dev_resume(struct device *dev)
{
	return iaxxx_core_resume_rt(dev);
}

/*
 * iaxxx_pm_enable_attr - store function for suspend and resume
 */
static ssize_t iaxxx_pm_enable_attr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		dev_dbg(dev, "%s() PM Resume\n", __func__);
		clear_bit(IAXXX_FLG_PM_SUSPEND, &priv->flags);
		/* Async trying to resume */
		iaxxx_work(priv, runtime_work);
	} else {
		dev_dbg(dev, "%s() PM Suspend\n", __func__);
		set_bit(IAXXX_FLG_PM_SUSPEND, &priv->flags);

		/* Abort current FW recovery procedure */
		iaxxx_abort_fw_recovery(priv);

		/* Start suspending and block until complete */
		iaxxx_work(priv, runtime_work);
		iaxxx_work_flush(priv, runtime_work);
	}

	return count;
}
static DEVICE_ATTR(pm_enable, 0600, NULL, iaxxx_pm_enable_attr);

/*
 * iaxxx_set_spi_speed
 */
static ssize_t iaxxx_set_spi_speed(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val > priv->spi_app_speed)
		return -EINVAL;
	if (priv->spi_speed_setup)
		priv->spi_speed_setup(dev, val);
	priv->spi_app_speed = val;

	dev_info(dev, "%s() Success\n", __func__);
	return count;
}
static DEVICE_ATTR(set_spi_speed, 0200, NULL, iaxxx_set_spi_speed);

/*
 * iaxxx_set_speed_spi2
 */
static ssize_t iaxxx_set_speed_spi2(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc, status;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&priv->test_mutex);

	/* Write speed for SPI2 interface */
	rc = regmap_write(priv->regmap,
			IAXXX_PWR_MGMT_MAX_SPI2_MASTER_SPEED_REQ_ADDR,
			val);

	if (rc) {
		dev_info(dev, "%s() Fail\n", __func__);
		return count;
	}

	rc = iaxxx_send_update_block_request(dev, &status,
			IAXXX_BLOCK_0);

	mutex_unlock(&priv->test_mutex);

	dev_info(dev, "%s() Success\n", __func__);
	return count;
}
static DEVICE_ATTR(set_speed_spi2, 0200, NULL, iaxxx_set_speed_spi2);

/*
 * iaxxx_pm_wakeup_chip
 */
static ssize_t iaxxx_pm_wakeup_chip(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	/* SPI transcation should wake up the chip
	 * reading SYS_STATUS reg
	 */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &val);
	msleep(50);
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &val);
	if (rc)
		dev_info(dev, "%s() Fail\n", __func__);
	else
		dev_info(dev, "%s() Success\n", __func__);
	return rc;
}
static DEVICE_ATTR(pm_wakeup_chip, 0200, NULL, iaxxx_pm_wakeup_chip);

/*
 * iaxxx_pm_set_aclk, input value as follows.
 * 0 - 3.072 MHz
 * 1 - 6.144 MHz
 * 2 - 12.288 MHz
 * 3 - 24.576 MHz
 * 4 - 49.152 MHz
 * 5 - 98.304 MHz
 * 6 - 368.640 MHz
 * otherwise - Invalid
 */
static ssize_t iaxxx_pm_set_aclk(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc = -1, status;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val > 6)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	rc = regmap_update_bits(priv->regmap,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_MASK,
			val << IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_POS);
	if (!rc)
		rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_SET_AUDIO_POWER_MODE_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_SET_AUDIO_POWER_MODE_MASK);

	if (!rc)
		rc = iaxxx_send_update_block_request(dev, &status,
							IAXXX_BLOCK_0);
	mutex_unlock(&priv->test_mutex);

	if (rc)
		dev_info(dev, "%s() Fail\n", __func__);
	else
		dev_info(dev, "%s() Success\n", __func__);
	return rc;
}
static DEVICE_ATTR(pm_set_aclk, 0200, NULL, iaxxx_pm_set_aclk);

/*
 * iaxxx_pm_set_optimal_power_mode (host0)
 * Need to do wake up to come out
 */
static ssize_t iaxxx_pm_set_optimal_power_mode_host0(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = regmap_write(priv->regmap, IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_ADDR,
			priv->spi_app_speed);


	rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK,
			0x1 <<
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_POS);
	if (rc) {
		dev_info(dev, "%s() Fail\n", __func__);
		return rc;
	}

	iaxxx_send_update_block_no_wait(dev, HOST_0);

	msleep(20);
	dev_info(dev, "%s() Success\n", __func__);
	mutex_unlock(&priv->test_mutex);
	return count;
}
static DEVICE_ATTR(pm_set_optimal_power_mode_host0, 0200, NULL,
		iaxxx_pm_set_optimal_power_mode_host0);
/*
 * iaxxx_pm_set_optimal_power_mode (host1)
 * Need to do wake up to come out
 */
static ssize_t iaxxx_pm_set_optimal_power_mode_host1(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = regmap_write(priv->regmap, IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_1_ADDR,
	    priv->spi_app_speed);
	if (!rc)
		rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_1_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_MASK,
			0x1 <<
			IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_POS);
	if (rc) {
		dev_info(dev, "%s() Fail\n", __func__);
		return count;
	}

	iaxxx_send_update_block_no_wait(dev, HOST_1);
	msleep(20);
	dev_info(dev, "%s() Success\n", __func__);
	mutex_unlock(&priv->test_mutex);
	return count;
}
static DEVICE_ATTR(pm_set_optimal_power_mode_host1, 0200, NULL,
		iaxxx_pm_set_optimal_power_mode_host1);

/*
 * Set power mode to sleep
 */
static ssize_t iaxxx_pm_set_sleep_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);

	/*setting up the sleep mode*/
	rc = regmap_update_bits(priv->regmap,
		IAXXX_SRB_SYS_POWER_CTRL_ADDR,
		IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_MASK,
		0x2 <<
		IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_POS);

	iaxxx_send_update_block_no_wait(dev, HOST_0);

	msleep(40);
	dev_info(dev, "%s() Success\n", __func__);
	mutex_unlock(&priv->test_mutex);
	return count;
}
static DEVICE_ATTR(pm_set_sleep_mode, 0200, NULL,
		iaxxx_pm_set_sleep_mode);

/*
 * Disable chip power management
 */
static ssize_t iaxxx_pm_disable_chip_pm(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		iaxxx_wakeup_chip(priv);
		priv->disable_chip_pm = true;
	} else
		priv->disable_chip_pm = false;
	dev_info(dev, "%s() chip_pm is %s\n", __func__,
			priv->disable_chip_pm ? "disabled" : "enabled");
	return count;
}
static DEVICE_ATTR(pm_disable_chip_pm, 0200, NULL, iaxxx_pm_disable_chip_pm);

/* sysfs chip_reset function
 */
static ssize_t iaxxx_chip_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;
	iaxxx_reset_to_sbl(priv);
	return count;
}
static DEVICE_ATTR(chip_reset, 0200, NULL, iaxxx_chip_reset_store);

/*
 * sysfs attr info
 */
static struct attribute *iaxxx_attrs[] = {
	&dev_attr_fw_update_test.attr,
	&dev_attr_cmem_test.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_host_version.attr,
	&dev_attr_fw_timestamp.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_plugin_version.attr,
	&dev_attr_package_version.attr,
	&dev_attr_debug_isr_disable.attr,
	&dev_attr_pm_enable.attr,
	&dev_attr_pm_wakeup_chip.attr,
	&dev_attr_pm_set_aclk.attr,
	&dev_attr_pm_set_optimal_power_mode_host0.attr,
	&dev_attr_pm_set_optimal_power_mode_host1.attr,
	&dev_attr_pm_set_sleep_mode.attr,
	&dev_attr_pm_disable_chip_pm.attr,
	&dev_attr_set_spi_speed.attr,
	&dev_attr_set_speed_spi2.attr,
	&dev_attr_chip_reset.attr,
	NULL,
};

/*
 * sysfs attr group info
 */
static const struct attribute_group iaxxx_attr_group = {
	.attrs = iaxxx_attrs,
	.name	= "iaxxx"
};

/**
 * iaxxx_device_power_init - init power
 *
 * @priv: iaxxx private data
 */
static int iaxxx_device_power_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	/* Initialize the platform data */
	rc = iaxxx_populate_dt_pdata(priv);
	if (rc) {
		dev_err(dev,
			"Failed to initialize platform data: %d\n", rc);
		goto err_populate_pdata;
	}

	rc = iaxxx_config_regulators(priv);
	if (rc) {
		dev_err(dev,
			"Failed to configure regulators %d\n", rc);
		goto err_enable_regulator;
	}

	/* Initialize the GPIOs */
	rc = iaxxx_gpio_init(priv);
	if (rc) {
		dev_err(dev, "Failed to initialize GPIOs: %d\n", rc);
		goto err_gpio_init;
	}

	/* Initialize interrupts */
	rc = iaxxx_irq_init(priv);
	if (rc) {
		dev_err(dev,
			"Failed to initialize interrupts: %d\n", rc);
		goto err_irq_init;
	}

	return 0;

err_irq_init:
err_gpio_init:
err_enable_regulator:
err_populate_pdata:
	return rc;
}

/*===========================================================================
 * Exported APIs
 *===========================================================================
 */

/**
 * iaxxx_device_reset - called from probe to perform chip reset
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_reset(struct iaxxx_priv *priv)
{
	/* Pull the device out of reset. */
	return iaxxx_reset_to_sbl(priv);
}

int iaxxx_fw_notifier_register(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_register(&priv->core_notifier_list, nb);
	return ret;
}

int iaxxx_fw_notifier_unregister(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_unregister(&priv->core_notifier_list, nb);
	return ret;
}

int iaxxx_fw_notifier_call(struct device *dev, unsigned long val, void *v)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	return srcu_notifier_call_chain(&priv->core_notifier_list, val, v);
}

/**
 * iaxxx_device_init - called from probe to perform device initialization
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_init(struct iaxxx_priv *priv)
{
	int rc;

	/* Init mutexes */
	mutex_init(&priv->update_block_lock);
	mutex_init(&priv->event_work_lock);
	mutex_init(&priv->event_queue_lock);
	mutex_init(&priv->plugin_lock);
	mutex_init(&priv->module_lock);
	mutex_init(&priv->crashdump_lock);
	mutex_init(&priv->pm_mutex);

	iaxxx_init_kthread_worker(&priv->worker);
	init_waitqueue_head(&priv->boot_wq);
	priv->thread = kthread_run(kthread_worker_fn, &priv->worker,
				   "iaxxx-core");
	if (IS_ERR(priv->thread)) {
		dev_err(priv->dev, "Can't create kthread worker: %ld\n",
			PTR_ERR(priv->thread));
		return PTR_ERR(priv->thread);
	}

	iaxxx_init_kthread_work(&priv->fw_update_work, iaxxx_fw_update_work);
	iaxxx_init_kthread_work(&priv->fw_crash_work, iaxxx_fw_crash_work);
	iaxxx_init_kthread_work(&priv->runtime_work, iaxxx_runtime_work);

	/* Initialize regmap for SBL */
	rc = iaxxx_regmap_init(priv);
	if (rc)
		return rc;

	/* Initialize the register dump */
	rc = iaxxx_regdump_init(priv);
	if (rc)
		goto err_regdump_init;


	/* Create sysfs */

	if (sysfs_create_group(&priv->dev->kobj, &iaxxx_attr_group)) {
		dev_err(priv->dev,
			"%s [ERROR] sysfs_create_group\n", __func__);
	}

	/* TODO: SYSTEM_ROM_VER_STR */

	/* Initialize test environment */
	iaxxx_test_init(priv);

	/* Initialize the cdev interface */
	rc = iaxxx_cdev_init();
	if (rc) {
		dev_err(priv->dev,
			"Failed to initialize cdev interface: %d\n", rc);
		goto err_debug_init;
	}

	/* Initialize the debug interface */
	rc = iaxxx_debug_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"Failed to initialize debug interface: %d\n", rc);
		goto err_debug_init;
	}

	srcu_init_notifier_head(&priv->core_notifier_list);

	/* Init early stage for tunneling */
	rc = iaxxx_tunnel_dev_init_early(priv);
	if (rc) {
		dev_err(priv->dev,
			"%s: Failed to create debugfs entry\n", __func__);
		goto err_debug_init;
	}

	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_add_regmap(priv->dev, priv->regmap, &priv->dfs_node);
	if (rc)
		dev_err(priv->dev,
			"%s: Failed to create debugfs entry\n", __func__);

	rc = iaxxx_event_init(priv);
	if (rc) {
		dev_err(priv->dev, "Failed to initialize the event\n");
		goto err_event_init;
	}


	/*
	 * Make the device power up the chip first so that
	 * the knowles chip won't cause any i2c communication error
	 * for other devices on the same bus
	 */
	rc = iaxxx_device_power_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"Failed to power up device: %d\n", rc);
		goto err_power_init;
	}

	rc = mfd_add_devices(priv->dev, -1, iaxxx_devices,
			ARRAY_SIZE(iaxxx_devices), NULL, 0, NULL);
	if (rc) {
		dev_err(priv->dev, "Failed to add cell devices\n");
		goto err_add_devices;
	}


	/* Initialize indexes uses for dumping
	 * plugin version and package versions
	 */
	priv->plugin_version_plugin_index = -EINVAL;
	priv->package_version_package_index = -EINVAL;

	iaxxx_work(priv, fw_update_work);
	return 0;

err_add_devices:
err_power_init:
	iaxxx_event_exit(priv);
err_event_init:
err_debug_init:
	iaxxx_regdump_exit(priv);
err_regdump_init:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(priv->dev, priv->regmap);
		regmap_exit(priv->regmap);
	}
	mutex_destroy(&priv->update_block_lock);
	mutex_destroy(&priv->event_work_lock);
	mutex_destroy(&priv->event_queue_lock);
	mutex_destroy(&priv->plugin_lock);
	mutex_destroy(&priv->module_lock);
	mutex_destroy(&priv->crashdump_lock);
	return -EINVAL;
}

void iaxxx_device_exit(struct iaxxx_priv *priv)
{
	pm_runtime_disable(priv->dev);
	if (test_and_clear_bit(IAXXX_FLG_STARTUP, &priv->flags))
		mfd_remove_devices(priv->dev);

	if (priv->vdd_io) {
		regulator_disable(priv->vdd_io);
		devm_regulator_put(priv->vdd_io);
	}

	if (priv->vdd_core) {
		regulator_disable(priv->vdd_core);
		devm_regulator_put(priv->vdd_core);
	}

	if (priv->vdd_oslo) {
		regulator_disable(priv->vdd_oslo);
		devm_regulator_put(priv->vdd_oslo);
	}

	/* Delete the work queue */
	flush_work(&priv->event_work_struct);
	destroy_workqueue(priv->event_workq);

	iaxxx_flush_kthread_worker(&priv->worker);
	kthread_stop(priv->thread);

	mutex_destroy(&priv->update_block_lock);
	mutex_destroy(&priv->event_work_lock);
	mutex_destroy(&priv->event_queue_lock);
	mutex_destroy(&priv->plugin_lock);
	mutex_destroy(&priv->module_lock);
	mutex_destroy(&priv->crashdump_lock);
	mutex_destroy(&priv->pm_mutex);

	iaxxx_regdump_exit(priv);
	iaxxx_irq_exit(priv);
	iaxxx_debug_exit(priv);
	iaxxx_cdev_exit();
	iaxxx_event_exit(priv);

	if (priv->regmap_config->ranges) {
		devm_kfree(priv->dev, (void *)priv->regmap_config->ranges);
		priv->regmap_config->ranges = NULL;
	}

	if (priv->regmap) {
		iaxxx_dfs_del_regmap(priv->dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}
}

/*
 * iaxxx-core.c -- IAxxx Multi-Function Device driver
 *
 * Copyright 2016 Knowles Corporation
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


/*
 * Driver To Do Items
 *
 * - Add mutex for API calls: Memory management, Event Management, etc.
 * - Define the volatile and read-only registers
 * - Use two regmaps; Core driver owns SRB, cell drivers have ARB access
 *	any SRB access for cell drivers needs to go througn an API call
 * - Enable regmap caching
 * - Verify that virtual addresses can only be accessed when mapped
 * - pull in the event manager from Athens (document on wiki)
 */

#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
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

#define IAXXX_FW_RETRY_COUNT		5		/* 0 retry if failed */
#define IAXXX_FW_DOWNLOAD_TIMEOUT	10000		/* 10 secs */
#define IAXXX_VER_STR_SIZE		60
#define IAXXX_BYTES_IN_A_WORD		4
#define IAXXX_MAX_PLUGIN		6
#define IAXXX_MAX_PACKAGE		3

#define IAXXX_BYPASS_ON_VAL 0x00C199BB
#define IAXXX_BYPASS_OFF_VAL 0x0C099BB
#define IAXXX_PWR_DWN_VAL 0x01C00050
#define IAXXX_PWR_ON_VAL 0x845
#define IAXXX_PWR_STATE_RETRY	0x5

#define iaxxx_ptr2priv(ptr, item) container_of(ptr, struct iaxxx_priv, item)

/* Linux kthread APIs have changes in version 4.9 */
#if defined(init_kthread_worker)
#define iaxxx_work(priv, work) queue_kthread_work(&priv->worker, &priv->work)
#define iaxxx_flush_kthread_worker(worker) flush_kthread_worker(worker)
#define iaxxx_init_kthread_worker(worker)  init_kthread_worker(worker)
#define iaxxx_init_kthread_work(work, fn)  init_kthread_work(work, fn)
#elif defined(kthread_init_worker)
#define iaxxx_work(priv, work) kthread_queue_work(&priv->worker, &priv->work)
#define iaxxx_flush_kthread_worker(worker) kthread_flush_worker(worker)
#define iaxxx_init_kthread_worker(worker)  kthread_init_worker(worker)
#define iaxxx_init_kthread_work(work, fn)  kthread_init_work(work, fn)
#else
#error kthread functions not defined
#endif

static int iaxxx_fw_dl_complete_notify(struct device *dev);

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
	E_IAXXX_BOOTUP_ERROR = -2
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
};

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
	if (priv->iaxxx_state->fw_state == FW_SBL_MODE ||
		priv->iaxxx_state->fw_state == FW_CRASH ||
		priv->iaxxx_state->fw_state == FW_RECOVERY)
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
	if (priv->ext_clk) {
		dev_info(dev, "%s(): enable clk\n", __func__);
		clk_prepare_enable(priv->ext_clk);
		usleep_range(10000, 12000);
	}
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
	struct iaxxx_priv *priv = (struct iaxxx_priv *)data;

	/* If ISR is disabled, return as handled */
	if (priv->debug_isr_disable)
		return IRQ_HANDLED;

	dev_dbg(priv->dev, "%s: IRQ %d\n", __func__, irq);

	if (priv->iaxxx_state->fw_state != FW_APP_MODE)
		return IRQ_HANDLED;

	/* Read SYSTEM_STATUS to ensure that device is in Application Mode */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &status);
	if (rc)
		dev_err(priv->dev,
			"Failed to read SYSTEM_STATUS, rc = %d\n", rc);

	mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
	if (mode != SYSTEM_STATUS_MODE_APPS) {
		dev_err(priv->dev,
			"Not in app mode CM4 might crashed, mode = %d\n", mode);
		priv->cm4_crashed = true;
		queue_work(priv->event_workq, &priv->event_work_struct);
		return IRQ_HANDLED;
	}

	/* Any events in the event queue? */
	rc = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(priv->dev,
			"Failed to read EVENT_COUNT, rc = %d\n", rc);
		goto out;
	}

	if (count > 0 && priv->event_workq) {
		dev_dbg(priv->dev, "%s: %d event(s) avail\n", __func__, count);
		queue_work(priv->event_workq, &priv->event_work_struct);
		handled = true;
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
	if (!gpio_is_valid(priv->event_gpio))
		return -ENXIO;

	return request_threaded_irq(gpio_to_irq(priv->event_gpio), NULL,
		iaxxx_event_isr, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
		"IAxxx-event-irq", priv);
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
	uint32_t reg, mode, status, mem_elec_ctrl_val;
	struct device *dev = priv->dev;
	int mode_retry = 5;

	if (priv->reset_cb)
		priv->reset_cb(dev);

	do {
		/* Verify that the device is in bootloader mode */
		rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR,
				&status);
		if (rc) {
			dev_err(dev, "regmap_read failed, rc = %d\n", rc);
			return E_IAXXX_REGMAP_ERROR;
		}

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_dbg(dev, "System Status: 0x%.08X mode: 0x%.08X\n", status,
				mode);
	} while (!mode && mode_retry--);

	if (!mode && !mode_retry) {
		WARN_ON(mode != SYSTEM_STATUS_MODE_SBL);
		dev_err(dev, "SBL SYS MODE retry expired\n");
		return -ETIMEDOUT;
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
	priv->iaxxx_state->fw_state = FW_APP_MODE;

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

static int iaxxx_dump_cm4crashlog(struct iaxxx_priv *priv)
{
	uint32_t buf_size;
	uint32_t data_written = 0;
	uint32_t log_addr;
	uint32_t log_size;
	int ret;

	if (!priv->crashlog->cm4header.log_size) {
		dev_err(priv->dev,
			"Nothing to read from CM4 crash log\n");
		return 0;
	}
	/* If memory already allocated */
	kfree(priv->crashlog->log_buffer);
	priv->crashlog->log_buffer = NULL;

	/* Calculate total crash log dump size */
	buf_size = sizeof(struct iaxxx_crashlog_header);
	buf_size += priv->crashlog->cm4header.log_size;
	priv->crashlog->log_buffer_size = buf_size;
	/* Allocate the memory */
	priv->crashlog->log_buffer = kzalloc(buf_size, GFP_KERNEL);
	if (!priv->crashlog->log_buffer)
		return -ENOMEM;

	/* Copy header information */
	memcpy(priv->crashlog->log_buffer + data_written,
				&priv->crashlog->cm4header,
				sizeof(struct iaxxx_crashlog_header));
	data_written += sizeof(struct iaxxx_crashlog_header);
	log_addr = priv->crashlog->cm4header.log_addr;
	log_size = priv->crashlog->cm4header.log_size;

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

	dev_dbg(priv->dev, "Data written 0x%x\n", data_written);
	return 0;
}

static void iaxxx_cm4_crashlog_header_read(struct iaxxx_priv *priv)
{
	int ret;

	priv->crashlog->cm4header.log_type = IAXXX_CRASHLOG_CM4;
	ret = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUGLOG_BLOCK_0_CRASHLOG_ADDR_ADDR,
			&priv->crashlog->cm4header.log_addr, 2);
	if (ret) {
		dev_err(priv->dev,
			"IAXXX_CRASHLOG_CM4 address read fail %d\n", ret);
		priv->crashlog->cm4header.log_addr = 0;
		priv->crashlog->cm4header.log_size = 0;
	}

	dev_dbg(priv->dev, "CM4: addr 0x%x size 0x%x\n",
				priv->crashlog->cm4header.log_addr,
				priv->crashlog->cm4header.log_size);
}

static int iaxxx_fw_recovery(struct iaxxx_priv *priv)
{
	int rc;
	char action[] = "ACTION=IAXXX_RECOVERY_EVENT";
	char *event[] = {action, NULL};
	uint32_t kw_bitmap;
	int try_count = 0;

	priv->iaxxx_state->fw_state = FW_RECOVERY;
	/* Bypass regmap cache */
	regcache_cache_bypass(priv->regmap, true);

	do {
		/* Firmware download */
		rc = iaxxx_do_fw_update(priv);
	} while (rc && ++try_count < IAXXX_FW_RETRY_COUNT);
	if (rc) {
		dev_err(priv->dev,
			"Recovery retry's expired with reason:%d\n", rc);
		priv->iaxxx_state->fw_state = FW_CRASH;
		return rc;
	}
	/* Reinitialize the regmap cache */
	rc = regmap_reinit_cache(priv->regmap, priv->regmap_config);
	if (rc) {
		dev_err(priv->dev,
			"regmap cache can not be reinitialized %d\n", rc);
		regcache_cache_bypass(priv->regmap, true);
	}

	atomic_set(&priv->power_state, IAXXX_NORMAL);
	if (gpio_is_valid(priv->event_gpio))
		enable_irq(gpio_to_irq(priv->event_gpio));
	/* Subscribing for FW crash event */
	rc = iaxxx_core_evt_subscribe(priv->dev, IAXXX_CM4_CTRL_MGR_SRC_ID,
			IAXXX_CRASH_EVENT_ID, IAXXX_SYSID_HOST, 0);
	if (rc)
		dev_err(priv->dev, "%s: failed to subscribe for crash event\n",
				__func__);

	/* HAL needs this info for loading already loaded KWs */
	kw_bitmap = priv->iaxxx_state->kw_info.kw_recognize_bitmap;
	/* Clear system state */
	memset(priv->iaxxx_state, 0, sizeof(struct iaxxx_system_state));
	priv->iaxxx_state->kw_info.kw_recognize_bitmap = kw_bitmap;
	priv->iaxxx_state->fw_state = FW_APP_MODE;
	/* Send recovery event to HAL */
	kobject_uevent_env(&priv->dev->kobj, KOBJ_CHANGE, event);
	dev_info(priv->dev, "%s: Recovery done\n", __func__);
	return rc;
}

static void iaxxx_crash_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, crash_work);
	int ret;
	char action[] = "ACTION=IAXXX_CRASH_EVENT";
	char *event[] = {action, NULL};
	uint32_t core_crashed = 0;
	uint32_t mode, status;
	int mode_retry = 5;

	priv->crash_count++;
	dev_info(priv->dev, "iaxxx %d time crashed\n",
			priv->crash_count);

	/* Clear event queue */
	if (gpio_is_valid(priv->event_gpio))
		disable_irq(gpio_to_irq(priv->event_gpio));
	mutex_lock(&priv->event_queue_lock);
	priv->event_queue->w_index = -1;
	priv->event_queue->r_index = -1;
	mutex_unlock(&priv->event_queue_lock);
	priv->route_status = 0;
	if (!priv->cm4_crashed) {
		ret = regmap_read(priv->regmap,
				IAXXX_SRB_PROCESSOR_CRASH_STATUS_ADDR,
				&core_crashed);
		/* Crash status read fails, means CM4 core crashed */
		if (ret) {
			dev_info(priv->dev,
					"Crash status read fail %s()\n",
					__func__);
		}
		if (core_crashed == 1 << IAXXX_HMD_ID)
			dev_info(priv->dev, "D4100S HMD core crashed\n");
		else if (core_crashed == 1 << IAXXX_DMX_ID)
			dev_info(priv->dev, "D4100S DMX Core Crashed\n");
		else
			dev_info(priv->dev, "D4100S Update block failed recovery\n");

		mutex_lock(&priv->crashdump_lock);
		iaxxx_crashlog_header_read(priv);
		iaxxx_dump_crashlogs(priv);
		mutex_unlock(&priv->crashdump_lock);
	} else {
		dev_info(priv->dev, "D4100S CM4 core crashed\n");
		if (priv->reset_cb)
			priv->reset_cb(priv->dev);

		do {
			/* Verify that the device is in bootloader mode */
			ret = regmap_read(priv->regmap,
					IAXXX_SRB_SYS_STATUS_ADDR, &status);
			if (ret) {
				dev_err(priv->dev,
					"regmap_read failed, ret = %d\n", ret);
			}

			mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
			dev_dbg(priv->dev,
				"System Status: 0x%.08X mode: 0x%.08X\n",
				status, mode);
		} while (!mode && mode_retry--);

		if (!mode && !mode_retry) {
			WARN_ON(mode != SYSTEM_STATUS_MODE_SBL);
			dev_err(priv->dev,
				"SBL SYS MODE retry expired in crash dump\n");
			return;
		}

		mutex_lock(&priv->crashdump_lock);
		iaxxx_dump_cm4crashlog(priv);
		priv->cm4_crashed = false;
		mutex_unlock(&priv->crashdump_lock);
	}
	iaxxx_tunnel_stop(priv);
	iaxxx_reset_codec_params(priv);
	/* Collect the crash logs */
	kobject_uevent_env(&priv->dev->kobj, KOBJ_CHANGE, event);
	ret = iaxxx_fw_recovery(priv);
	if (ret)
		dev_err(priv->dev, "Recovery fail\n");
}

static int iaxxx_crash_handler(struct iaxxx_priv *priv)
{
	iaxxx_work(priv, crash_work);
	return 0;
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

static int iaxxx_device_suspend(struct iaxxx_priv *priv);
static int iaxxx_device_resume(struct iaxxx_priv *priv);


/**
 * iaxxx_dev_init_work - work thread for initializing target
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * Firmware update, switch to application mode, and install codec driver
 *
 */

static void iaxxx_crash_recovery_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, crash_recover_work);

	if (priv->crash_handler)
		priv->crash_handler(priv);
}

static void iaxxx_dev_init_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, dev_init_work);
	struct device *dev = priv->dev;
	int rc;
	rc = iaxxx_do_fw_update(priv);
	if (rc == E_IAXXX_REGMAP_ERROR) {
		goto err_chip_reset;
	} else if (rc == E_IAXXX_BOOTUP_ERROR) {
		static int try_count;

		/* If there's device init cb, retry */
		if (!priv->reset_cb) {
			dev_err(dev, "%s: Chip failed to boot up\n", __func__);
			goto err_app_bootup;
		}

		if (++try_count < IAXXX_FW_RETRY_COUNT) {
			dev_err(dev, "%s: bootup error. retry... %d\n",
						__func__, try_count);
			iaxxx_work(priv, dev_init_work);
			return;
		}

		dev_err(dev, "%s: %d retry failed! EXIT\n",
				__func__, IAXXX_FW_RETRY_COUNT);
		goto err_app_bootup;
	}

	/* Initialize "application" regmap */
	rc = iaxxx_application_regmap_init(priv);
	if (rc)
		goto err_app_bootup;

	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_switch_regmap(priv->dev, priv->regmap, priv->dfs_node);
	if (rc)
		dev_err(priv->dev,
			"%s: Failed to create debugfs entry\n", __func__);

	/* Add the sub-devices */
	if (!test_and_set_bit(IAXXX_FLG_STARTUP, &priv->flags)) {
		rc = mfd_add_devices(dev, -1,
			iaxxx_devices, ARRAY_SIZE(iaxxx_devices), NULL,
				0, NULL);
		if (rc) {
			dev_err(dev, "%s: Failed to add cell devices\n",
					__func__);
			goto err_add_devices;
		}
	}

	priv->crash_handler = iaxxx_crash_handler;
	INIT_WORK(&priv->crash_recover_work, iaxxx_crash_recovery_work);

	atomic_set(&priv->power_state, IAXXX_NORMAL);

	iaxxx_cm4_crashlog_header_read(priv);
	/* Subscribing for FW crash event */
	rc = iaxxx_core_evt_subscribe(dev, IAXXX_CM4_CTRL_MGR_SRC_ID,
			IAXXX_CRASH_EVENT_ID, IAXXX_SYSID_HOST, 0);
	if (rc) {
		dev_err(dev, "%s: failed to subscribe for crash event\n",
				__func__);
		goto err_add_devices;
	}

	rc = iaxxx_fw_dl_complete_notify(dev);
	if (rc)
		dev_err(dev, "unable to write to fw dl node\n");

	dev_info(dev, "%s: done\n", __func__);
	return;

err_add_devices:
err_app_bootup:
err_chip_reset:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}
	if (priv->reg_dump)
		iaxxx_regdump_exit(priv);
}

/**
 * iaxxx_test_init - initialize work items and mutex for test environment
 *
 * @priv	: iaxxx private data
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
	iaxxx_work(priv, dev_init_work);
	return count;
}
static DEVICE_ATTR(fw_update, 0200, NULL, iaxxx_fw_update);

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
	int i;
	uint32_t buf_len = 0;
	static const char * const plugin[] = {
		"VQ", "VP", "Buffer", "Mixer", "MBC", "PEQ"};

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < IAXXX_MAX_PLUGIN; i++) {
		rc = get_version_str(
			priv, IAXXX_PLUGIN_INS_GRP_PLUGIN_VER_STR_REG(i),
			verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Plugin version read fail\n",
								__func__);
			return -EIO;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE, "%s:\t%s\n",
							plugin[i], verbuf);
	}
	return buf_len;
}
static DEVICE_ATTR(plugin_version, 0400, iaxxx_plugin_version_show, NULL);

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
	int i;
	uint32_t buf_len = 0;
	static const char * const package[] = {"Pepperoni", "Buffer", "Mixer"};
	uint32_t inst_id[] = {0, 2, 3};

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < IAXXX_MAX_PACKAGE; i++) {
		rc = get_version_str(priv,
			IAXXX_PLUGIN_INS_GRP_PACKAGE_VER_STR_REG(inst_id[i]),
			verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Package version read fail\n",
								__func__);
			return -EIO;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE, "%s:\t%s\n",
							package[i], verbuf);
	}
	return buf_len;
}
static DEVICE_ATTR(package_version, 0400, iaxxx_package_version_show, NULL);

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

/*
 * Sysfs - firmware download complete
 */
static ssize_t iaxxx_fw_dl_complete_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* Node is created following completion of firmware download */
	return scnprintf(buf, PAGE_SIZE, "1\n");
}
static DEVICE_ATTR(fw_dl_complete, 0400, iaxxx_fw_dl_complete_show, NULL);

static int iaxxx_pdm_clk_stop(struct iaxxx_priv *priv, int port)
{
	int ret;
	int mask;
	int status;

	/* Disable Clock output */
	mask = (IAXXX_AO_CLK_CFG_PORTA_CLK_OE_MASK << port);
	ret = regmap_update_bits(priv->regmap, IAXXX_AO_CLK_CFG_ADDR, mask, 0);
	if (ret) {
		dev_err(priv->dev, "update failed %s()\n", __func__);
		goto clk_stop_err;
	}
	/* Stop the clock generation */
	mask = (1 << port);
	ret = regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
					mask, 0);
	if (ret) {
		dev_err(priv->dev, "update failed %s()\n", __func__);
		goto clk_stop_err;
	}
	ret = regmap_write(priv->regmap,
		IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_TRIGGER_MASK);
	if (ret) {
		dev_err(priv->dev, "write failed %s()\n", __func__);
		goto clk_stop_err;
	}

	/* Power disable for the PCM port */
	mask = (1 << port);
	ret = regmap_update_bits(priv->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
			mask, 0);
	if (ret) {
		dev_err(priv->dev, "update failed %s()\n", __func__);
		goto clk_stop_err;
	}
	ret = iaxxx_send_update_block_request(priv->dev, &status,
						IAXXX_BLOCK_0);
	if (ret) {
		dev_err(priv->dev, "Update blk failed %s()\n", __func__);
		goto clk_stop_err;
	}
	return 0;
clk_stop_err:
	dev_err(priv->dev, "%s failed\n", __func__);
	return ret;
}

static int iaxxx_pdm_clk_start(struct iaxxx_priv *iaxxx, int port)
{
	u32 period, div_val, nr_val, fs_sync_active;
	u32 port_bits_per_frame;
	u32 clk_ctrl_val;
	u32 ao_clk_cfg_val;
	u32 ao_clk_cfg_mask, status;
	u32 ret;

	/* Use the values set in the codec while enabling PDM clock */
	port_bits_per_frame = clk.port_bits_per_frame;
	nr_val = clk.nr_val;
	div_val = clk.div_val;
	period = clk.period;

	/*
	 * 1. Configure I2S master if chip is master
	 * 2. Enable AO CLK CFG
	 * 3. Configure ports registers
	 * 4. Configure CIC filter register
	 * 5. Enable Dmic clk
	 */
	if (port == PDM_PORTB) {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_PORTB_CLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_PORTB_CLK_OE_MASK;
	} else {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_PORTC_CLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_PORTC_CLK_OE_MASK;
	}

	ret = regmap_update_bits(iaxxx->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
		IAXXX_SRB_I2S_PORT_PWR_EN_MASK_VAL, (0x1 << port));
	if (ret)
		goto start_clk_err;
	ret = iaxxx_send_update_block_request(iaxxx->dev, &status,
			IAXXX_BLOCK_0);
	if (ret)
		goto start_clk_err;
	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
			1 << (port), 0 << port);
	if (ret)
		goto start_clk_err;
	/* I2S Trigger - Disable I2S */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, 1);
	if (ret)
		goto start_clk_err;
	/*Bit 0 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S2_GEN_CFG_PCM_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	if (ret)
		goto start_clk_err;
	/* Bit 1 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_MASK,
		IAXXX_I2S_GEN_CFG_CLK_POL_LOW);
	if (ret)
		goto start_clk_err;
	/* Bit 2*/
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	if (ret)
		goto start_clk_err;
	/* Bit 3 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_MASK,
		IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE);
	if (ret)
		goto start_clk_err;
	/* Bit 11:4 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_MASK,
		((port_bits_per_frame) <<
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS));
	if (ret)
		goto start_clk_err;
	/* For PDM FS is assumed 0 */
	fs_sync_active = (0 << IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS);
	/* Bit 19:12 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK, fs_sync_active);
	if (ret)
		goto start_clk_err;
	/* Bit 20 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_MASK,
		IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE);
	if (ret)
		goto start_clk_err;
	/* FS Align */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_FS_ALIGN_ADDR(port),
		IAXXX_I2S_I2S0_FS_ALIGN_WMASK_VAL,
		IAXXX_I2S_FS_ALIGN_MASTER_MODE);
	if (ret)
		goto start_clk_err;
	/* disable hl divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_DISABLE);
	if (ret)
		goto start_clk_err;
	/* Set HL value */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_P_MASK,
		div_val);
	if (ret)
		goto start_clk_err;
	/* enable hl divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_ENABLE);
	if (ret)
		goto start_clk_err;
	/* disable NR divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_DISABLE);
	if (ret)
		goto start_clk_err;
	/* Set NR value */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_WMASK_VAL, nr_val);
	if (ret)
		goto start_clk_err;
	/* enable NR divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_ENABLE);
	if (ret)
		goto start_clk_err;
	/* Clk control */
	clk_ctrl_val = (((period/2 - 1) <<
		IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS)|((period - 1)
		<< IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS));

	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_CLK_CTRL_ADDR(port),
		IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL, clk_ctrl_val);
	if (ret)
		goto start_clk_err;
	/* AO CLK Config */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_AO_CLK_CFG_ADDR,
		ao_clk_cfg_mask, ao_clk_cfg_val);
	if (ret)
		goto start_clk_err;
	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(port), 1 << port);
	if (ret)
		goto start_clk_err;
	/* I2S Trigger - Disable I2S */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
		IAXXX_SRB_PDMI_PORT_PWR_EN_MASK_VAL, 0xFF);
	if (ret)
		goto start_clk_err;
	ret = iaxxx_send_update_block_request(iaxxx->dev, &status,
			IAXXX_BLOCK_0);
	if (ret)
		goto start_clk_err;
	/* Configure IO CTRL ports */
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK,
		 IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK,
		IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK,
		IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK,
		IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK, IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK,
		IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK,
		IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK,
		IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_do_addr[port],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, IAXXX_IO_CTRL_DO);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK, IAXXX_IO_CTRL_DO);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, IAXXX_IO_CTRL_DO);
	if (ret)
		goto start_clk_err;
	/* CIC filter config */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_CNR0_CIC_RX_HOS_ADDR,
		IAXXX_CNR0_CIC_RX_HOS_MASK_VAL, 1);
	if (ret)
		goto start_clk_err;
	return 0;
start_clk_err:
	pr_err("%s() failed\n", __func__);
	return ret;
}

static int iaxxx_device_pwr_up(struct iaxxx_priv *priv)
{
	int ret;
	int retry = IAXXX_PWR_STATE_RETRY;

	do {
		retry--;
		ret = regmap_write(priv->regmap, IAXXX_CNR_CLK_EN_OVRRD_ADDR,
				IAXXX_PWR_ON_VAL);
		if (ret) {
			dev_err(priv->dev, "write failed %s()\n", __func__);
			continue;
		}
		ret = regmap_write(priv->regmap, IAXXX_CNR_CLK_SRC_SEL_ADDR,
				IAXXX_BYPASS_OFF_VAL);
		if (ret) {
			dev_err(priv->dev, "write failed %s()\n", __func__);
			continue;
		}
		ret = iaxxx_pdm_clk_start(priv, PDM_PORTC);
		if (ret) {
			dev_err(priv->dev, "start clk failed %s()\n", __func__);
			continue;
		}
	} while ((retry > 0) && ret);

	if (ret) {
		dev_err(priv->dev, "resume fail\n");
		atomic_set(&priv->power_state, IAXXX_RESUME_FAIL);
		return ret;
	}

	if (priv->spi_speed_setup)
		priv->spi_speed_setup(priv->dev, priv->spi_app_speed);

	atomic_set(&priv->power_state, IAXXX_NORMAL);
	return ret;
}

static int iaxxx_device_resume(struct iaxxx_priv *priv)
{
	int power_state = atomic_read(&priv->power_state);

	if (priv->iaxxx_state->fw_state != FW_APP_MODE) {
		dev_err(priv->dev, "FW mode is invalid\n");
		return -EIO;
	}

	if (power_state == IAXXX_NORMAL)
		return 0;

	if (power_state == IAXXX_POWER_TRANSITION)
		return -EBUSY;

	if (power_state == IAXXX_RESUME_FAIL) {
		dev_err(priv->dev, "chip is in invalid state\n");
		return -EINVAL;
	}

	if (atomic_cmpxchg(&priv->power_state,
			power_state, IAXXX_POWER_TRANSITION) != power_state)
		return -EBUSY;

	return iaxxx_device_pwr_up(priv);
}

static int iaxxx_device_suspend(struct iaxxx_priv *priv)
{
	int ret;
	int retry = IAXXX_PWR_STATE_RETRY;
	int power_state = atomic_read(&priv->power_state);
	int rc;

	if (priv->iaxxx_state->fw_state != FW_APP_MODE) {
		dev_err(priv->dev, "FW mode is invalid\n");
		return -EIO;
	}

	if (power_state == IAXXX_SUSPEND)
		return 0;

	if (power_state == IAXXX_POWER_TRANSITION)
		return -EBUSY;

	if (power_state == IAXXX_RESUME_FAIL) {
		dev_err(priv->dev, "chip is in invalid state\n");
		return -EINVAL;
	}

	if (atomic_cmpxchg(&priv->power_state,
			power_state, IAXXX_POWER_TRANSITION) != power_state)
		return -EBUSY;

	if (priv->spi_speed_setup)
		priv->spi_speed_setup(priv->dev, IAXXX_SPI_SBL_SPEED);

	do {
		retry--;
		/* I2S Clock Stop */
		ret = iaxxx_pdm_clk_stop(priv, PDM_PORTC);
		if (ret) {
			dev_err(priv->dev, "Clk stop failed %s()\n", __func__);
			continue;
		}
		ret = regmap_write(priv->regmap, IAXXX_CNR_CLK_SRC_SEL_ADDR,
				IAXXX_BYPASS_ON_VAL);
		if (ret) {
			dev_err(priv->dev, "write failed %s()\n", __func__);
			continue;
		}
		ret = regmap_write(priv->regmap, IAXXX_CNR_CLK_EN_OVRRD_ADDR,
				IAXXX_PWR_DWN_VAL);
		if (ret) {
			dev_err(priv->dev, "write failed %s()\n", __func__);
			continue;
		}
	} while ((retry > 0) && ret);

	if (ret) {
		dev_err(priv->dev, "suspend fail\n");
		rc = iaxxx_device_pwr_up(priv);
		if (rc) {
			dev_err(priv->dev, "not able to resume\n");
			return rc;
		}

		atomic_set(&priv->power_state, IAXXX_NORMAL);
		return ret;
	}

	atomic_set(&priv->power_state, IAXXX_SUSPEND);
	return 0;
}

/**
 * iaxxx_pm_enable - store function for suspend and resume
 */
static ssize_t iaxxx_pm_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int ret;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		dev_dbg(dev, "%s() PM Resume\n", __func__);
		ret = iaxxx_device_resume(priv);
	} else {
		dev_dbg(dev, "%s() PM Suspend\n", __func__);
		ret = iaxxx_device_suspend(priv);
	}
	if (ret) {
		pr_err("iaxxx pm enable failed\n");
		return -EIO;
	}
	return count;
}
static DEVICE_ATTR(pm_enable, 0600, NULL, iaxxx_pm_enable);

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


static int iaxxx_fw_dl_complete_notify(struct device *dev)
{
	return sysfs_add_file_to_group(&dev->kobj,
		&dev_attr_fw_dl_complete.attr, iaxxx_attr_group.name);
}

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
	mutex_init(&priv->crashdump_lock);

	iaxxx_init_kthread_worker(&priv->worker);
	priv->thread = kthread_run(kthread_worker_fn, &priv->worker,
				   "iaxxx-core");
	if (IS_ERR(priv->thread)) {
		dev_err(priv->dev, "Can't create kthread worker: %ld\n",
			PTR_ERR(priv->thread));
		return PTR_ERR(priv->thread);
	}

	iaxxx_init_kthread_work(&priv->dev_init_work, iaxxx_dev_init_work);
	iaxxx_init_kthread_work(&priv->crash_work, iaxxx_crash_work);

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

	iaxxx_work(priv, dev_init_work);

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

	priv->iaxxx_state->fw_state = FW_SBL_MODE;

	pm_runtime_enable(priv->dev);
	pm_runtime_idle(priv->dev);

	return 0;

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
	return -EINVAL;
}

void iaxxx_device_exit(struct iaxxx_priv *priv)
{
	pm_runtime_disable(priv->dev);
	if (test_and_clear_bit(IAXXX_FLG_STARTUP, &priv->flags))
		mfd_remove_devices(priv->dev);

	/* Delete the work queue */
	flush_work(&priv->event_work_struct);
	destroy_workqueue(priv->event_workq);

	iaxxx_flush_kthread_worker(&priv->worker);
	kthread_stop(priv->thread);

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

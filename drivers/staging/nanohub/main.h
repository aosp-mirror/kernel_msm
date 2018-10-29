/*
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _NANOHUB_MAIN_H
#define _NANOHUB_MAIN_H

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/wakelock.h>

#include "comms.h"
#include "bl.h"

#define NANOHUB_NAME "nanohub"

#define NANOHUB_WAKEUP_TRACE_ENABLE  (0)

struct nanohub_buf {
	struct list_head list;
	uint8_t buffer[255];
	uint8_t length;
};

struct nanohub_data;

struct nanohub_io {
	struct device *dev;
	struct nanohub_data *data;
	wait_queue_head_t buf_wait;
	struct list_head buf_list;
};

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
struct nanohub_trace_event {
	uint32_t event_id;
	uint32_t event_sub_id;
	char *event_name;
	uint8_t interrupt;
	int event_count;
};

struct wakeup_trace_listnode {
	struct list_head event_list;
	struct nanohub_trace_event trace_event;
	uint8_t buffer[255];
	uint8_t length;
};

struct irq_nums_during_wakeup {
	uint32_t nums_irq1;
	uint32_t nums_irq2;
	uint32_t nums_irq3;
};

#endif

static inline struct nanohub_data *dev_get_nanohub_data(struct device *dev)
{
	struct nanohub_io *io = dev_get_drvdata(dev);

	return io->data;
}

struct nanohub_data {
	/* indices for io[] array */
	#define ID_NANOHUB_SENSOR 0
	#define ID_NANOHUB_COMMS 1
	#define ID_NANOHUB_HR_LOG 2
	#define ID_NANOHUB_CUSTOM_FLASH 3
	#define ID_NANOHUB_MAX 4

	struct iio_dev *iio_dev;
	struct nanohub_io io[ID_NANOHUB_MAX];

	struct nanohub_comms comms;
	struct nanohub_bl bl;
	const struct nanohub_platform_data *pdata;
	int irq1;
	int irq2;
	int irq3;

	atomic_t kthread_run;
	atomic_t thread_state;
	wait_queue_head_t kthread_wait;

	struct wake_lock wakelock_read;

	struct nanohub_io free_pool;

	atomic_t lock_mode;
	/* these 3 vars should be accessed only with wakeup_wait.lock held */
	atomic_t wakeup_cnt;
	atomic_t wakeup_lock_cnt;
	atomic_t wakeup_acquired;
	wait_queue_head_t wakeup_wait;

	uint32_t interrupts[8];

	ktime_t wakeup_err_ktime;
	int wakeup_err_cnt;

	ktime_t kthread_err_ktime;
	int kthread_err_cnt;

	void *vbuf;
	struct task_struct *thread;
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
	atomic_t download_bl_status;
	atomic_t hub_mode_ap_pwr_down;
	atomic_t hub_mode_ap_active;
	atomic_t lcd_mutex;
	atomic_t sensor_hal_alive;
#if (NANOHUB_WAKEUP_TRACE_ENABLE)
	atomic_t suspend_status;
	atomic_t st_wakeup_trace;
	struct irq_nums_during_wakeup wakeup_trace_irqs;
	struct wakeup_trace_listnode wakeup_trace;
#endif
	struct mutex hub_mode_set_lock;
	struct mutex nanohub_write_lock;
	struct Nanohub_FuelGauge_Info *fg_info;
	uint32_t nanohub_variant_version;
	uint16_t nanohub_hw_type;
};

enum {
	LCD_MUTEX_OFF = 0,
	LCD_MUTEX_ON,
};

enum {
	SENSOR_HAL_DEAD = 0,
	SENSOR_HAL_ALIVED,
};

enum NRESET_POLARITY {
	NRESET_ACTIVE_LOW,
	NRESET_ACTIVE_HIGH,
};

enum {
	KEY_WAKEUP_NONE,
	KEY_WAKEUP,
	KEY_WAKEUP_LOCK,
};

enum {
	LOCK_MODE_NONE,
	LOCK_MODE_NORMAL,
	LOCK_MODE_IO,
	LOCK_MODE_IO_BL,
	LOCK_MODE_RESET,
	LOCK_MODE_SUSPEND_RESUME,
};

enum AP_GPIO_CMD {
	GPIO_CMD_POWEROFF = 0,
	GPIO_CMD_BAND,
	GPIO_CMD_AMBIENT,
	GPIO_CMD_NORMAL,
	GPIO_CMD_FLASH_ERASE,
	GPIO_CMD_REQUEST_FUELGAUGE,
	GPIO_CMD_SUSPEND,
	GPIO_CMD_RESUME,
	GPIO_CMD_FORCE_REQUEST_FUELGAUGE,
	GPIO_CMD_TEST = 15,
	GPIO_CMD_RESEND,
};

enum DOWNLOAD_BL_STATUS {
	DOWNLOAD_BL_NOT_START = 0,
	DOWNLOAD_BL_RUNNING,
	DOWNLOAD_BL_SUCCESS,
	DOWNLOAD_BL_FAILED,
	DOWNLOAD_BL_TIMEOUT,
};

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
enum NANOHUB_SUSPEND_STATUS {
	NANOHUB_SUSPEND_ENTRY = 0,
	NANOHUB_SUSPEND_EXIT,
};

enum NANOHUB_WAKEUP_TRACE {
	NANOHUB_WAKEUP_TRACE_OFF = 0,
	NANOHUB_WAKEUP_TRACE_ON,
};

enum DUMP_WAKEUP_TRACE_REASON {
	DUMP_TRACE_REASON_QUEUE_EMPTY,
	DUMP_TRACE_REASON_DUMP_QUEUE,
	DUMP_TRACE_REASON_NO_BUFFER,
	DUMP_TRACE_REASON_CLEAR_IRQS,
};
#endif

int request_wakeup_ex(struct nanohub_data *data, long timeout,
		      int key, int lock_mode);
void release_wakeup_ex(struct nanohub_data *data, int key, int lock_mode);
int nanohub_wait_for_interrupt(struct nanohub_data *data);
int nanohub_wakeup_eom(struct nanohub_data *data, bool repeat);
struct iio_dev *nanohub_probe(struct device *dev, struct iio_dev *iio_dev);
void nanohub_start(struct nanohub_data *data);
int nanohub_reset(struct nanohub_data *data);
int nanohub_remove(struct iio_dev *iio_dev);
int nanohub_shutdown(struct iio_dev *iio_dev);

int nanohub_suspend(struct iio_dev *iio_dev);
int nanohub_resume(struct iio_dev *iio_dev);
int __nanohub_send_AP_cmd(struct nanohub_data *data, enum AP_GPIO_CMD mode);

static inline int nanohub_irq1_fired(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	return !gpio_get_value(pdata->irq1_gpio);
}

static inline int nanohub_irq2_fired(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	return data->irq2 && !gpio_get_value(pdata->irq2_gpio);
}

static inline int request_wakeup_timeout(struct nanohub_data *data, int timeout)
{
	return request_wakeup_ex(data, timeout, KEY_WAKEUP, LOCK_MODE_NORMAL);
}

static inline int request_wakeup(struct nanohub_data *data)
{
	return request_wakeup_ex(data, MAX_SCHEDULE_TIMEOUT, KEY_WAKEUP,
				 LOCK_MODE_NORMAL);
}

static inline void release_wakeup(struct nanohub_data *data)
{
	release_wakeup_ex(data, KEY_WAKEUP, LOCK_MODE_NORMAL);
}

#endif

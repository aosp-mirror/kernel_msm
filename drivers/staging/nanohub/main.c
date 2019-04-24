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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/time.h>
#include <linux/platform_data/nanohub.h>
#include <linux/reboot.h>

#include "main.h"
#include "comms.h"
#include "bl.h"
#include "spi.h"
#include "bq27xxx_fuelgauge.h"
#include "custom_app_event.h"

#define READ_QUEUE_DEPTH        20
#define APP_FROM_HOST_EVENTID   0x000000F8
#define FIRST_SENSOR_EVENTID    0x00000200
#define LAST_SENSOR_EVENTID     0x000002FF
#define APP_TO_HOST_EVENTID     0x00000401

enum APP_TO_HOST_EVENT_SUBID {
	APP_TO_HOST_EVENT_SUBID_OTHERS = 0,
	APP_TO_HOST_EVENT_SUBID_FUELGAUGE = 1,
	APP_TO_HOST_EVENT_SUBID_HR_LOG,
	APP_TO_HOST_EVENT_SUBID_FLASH,
	APP_TO_HOST_EVENT_SUBID_CALIBRATE,
	APP_TO_HOST_EVENT_SUBID_SELFTEST,
};

#define OS_LOG_EVENTID          0x3B474F4C    /* ;LOG */
#define OS_LOG_TO_HAL_EVENTID   0x474F4C41    /* ALOG */
#define WAKEUP_INTERRUPT        1
#define WAKEUP_TIMEOUT_MS       2000
#define SUSPEND_TIMEOUT_MS      200
#define KTHREAD_ERR_TIME_NS     (60LL * NSEC_PER_SEC)
#define KTHREAD_ERR_CNT         70
#define KTHREAD_WARN_CNT        10
#define WAKEUP_ERR_TIME_NS      (60LL * NSEC_PER_SEC)
#define WAKEUP_ERR_CNT          4

/**
 * struct gpio_config - this is a binding between platform data and driver data
 * @label:     for diagnostics
 * @flags:     to pass to gpio_request_one()
 * @options:   one or more of GPIO_OPT_* flags, below
 * @pdata_off: offset of u32 field in platform data with gpio #
 * @data_off:  offset of int field in driver data with irq # (optional)
 */
struct gpio_config {
	const char *label;
	u16 flags;
	u16 options;
	u16 pdata_off;
	u16 data_off;
};

enum gpio_status {
	STATUS_LOW,
	STATUS_TOGGLE,
	STATUS_HIGH,
	STATUS_START_TO_CHECK_TIMEOUT,
};

#define GPIO_OPT_HAS_IRQ	0x0001
#define GPIO_OPT_OPTIONAL	0x8000

#define PLAT_GPIO_DEF(name, _flags) \
	.pdata_off = offsetof(struct nanohub_platform_data, name ## _gpio), \
	.label = "nanohub_" #name, \
	.flags = _flags \

#define PLAT_GPIO_DEF_IRQ(name, _flags, _opts) \
	PLAT_GPIO_DEF(name, _flags), \
	.data_off = offsetof(struct nanohub_data, name), \
	.options = GPIO_OPT_HAS_IRQ | (_opts) \

static int nanohub_open(struct inode *, struct file *);
static ssize_t nanohub_read(struct file *, char *, size_t, loff_t *);
static ssize_t nanohub_write(struct file *, const char *, size_t, loff_t *);
static unsigned int nanohub_poll(struct file *, poll_table *);
static int nanohub_release(struct inode *, struct file *);
static int nanohub_hw_reset(struct nanohub_data *data);

static struct class *sensor_class;
static int major;

static const struct gpio_config gconf[] = {
	{ PLAT_GPIO_DEF(nreset, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(wakeup, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(int, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(mode1, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(mode2, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(mode3, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(mode4, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(boot0, GPIOF_OUT_INIT_LOW) },
	{ PLAT_GPIO_DEF_IRQ(irq1, GPIOF_DIR_IN, 0) },
	{ PLAT_GPIO_DEF_IRQ(irq2, GPIOF_DIR_IN, GPIO_OPT_OPTIONAL) },
	{ PLAT_GPIO_DEF_IRQ(irq3, GPIOF_DIR_IN, GPIO_OPT_OPTIONAL) },
};

static const struct iio_info nanohub_iio_info = {
	.driver_module = THIS_MODULE,
};

static const struct file_operations nanohub_fileops = {
	.owner = THIS_MODULE,
	.open = nanohub_open,
	.read = nanohub_read,
	.write = nanohub_write,
	.poll = nanohub_poll,
	.release = nanohub_release,
};

enum {
	ST_RESET,
	ST_IDLE,
	ST_ERROR,
	ST_RUNNING
};

static inline bool gpio_is_optional(const struct gpio_config *_cfg)
{
	return _cfg->options & GPIO_OPT_OPTIONAL;
}

static inline bool gpio_has_irq(const struct gpio_config *_cfg)
{
	return _cfg->options & GPIO_OPT_HAS_IRQ;
}

static inline bool nanohub_has_priority_lock_locked(struct nanohub_data *data)
{
	return  atomic_read(&data->wakeup_lock_cnt) >
		atomic_read(&data->wakeup_cnt);
}

static inline void nanohub_notify_thread(struct nanohub_data *data)
{
	atomic_set(&data->kthread_run, 1);
	/* wake_up implementation works as memory barrier */
	wake_up_interruptible_sync(&data->kthread_wait);
}

static inline void nanohub_io_init(struct nanohub_io *io,
				   struct nanohub_data *data,
				   struct device *dev)
{
	init_waitqueue_head(&io->buf_wait);
	INIT_LIST_HEAD(&io->buf_list);
	io->data = data;
	io->dev = dev;
}

static inline bool nanohub_io_has_buf(struct nanohub_io *io)
{
	return !list_empty(&io->buf_list);
}

static struct nanohub_buf *nanohub_io_get_buf(struct nanohub_io *io,
					      bool wait)
{
	struct nanohub_buf *buf = NULL;
	int ret;

	spin_lock(&io->buf_wait.lock);
	if (wait) {
		ret = wait_event_interruptible_locked(io->buf_wait,
						      nanohub_io_has_buf(io));
		if (ret < 0) {
			spin_unlock(&io->buf_wait.lock);
			return ERR_PTR(ret);
		}
	}

	if (nanohub_io_has_buf(io)) {
		buf = list_first_entry(&io->buf_list, struct nanohub_buf, list);
		list_del(&buf->list);
	}
	spin_unlock(&io->buf_wait.lock);

	return buf;
}

static void nanohub_io_put_buf(struct nanohub_io *io,
			       struct nanohub_buf *buf)
{
	bool was_empty;

	spin_lock(&io->buf_wait.lock);
	was_empty = !nanohub_io_has_buf(io);
	list_add_tail(&buf->list, &io->buf_list);
	spin_unlock(&io->buf_wait.lock);

	if (was_empty) {
		if (&io->data->free_pool == io)
			nanohub_notify_thread(io->data);
		else
			wake_up_interruptible(&io->buf_wait);
	}
}

static inline int plat_gpio_get(struct nanohub_data *data,
				const struct gpio_config *_cfg)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	return *(u32 *)(((char *)pdata) + (_cfg)->pdata_off);
}

static inline void nanohub_set_irq_data(struct nanohub_data *data,
					const struct gpio_config *_cfg, int val)
{
	int *data_addr = ((int *)(((char *)data) + _cfg->data_off));

	if ((void *)data_addr > (void *)data &&
	    (void *)data_addr < (void *)(data + 1))
		*data_addr = val;
	else
		WARN(1, "No data binding defined for %s", _cfg->label);
}

static inline void mcu_wakeup_gpio_set_value(struct nanohub_data *data,
					     int val)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	gpio_set_value(pdata->wakeup_gpio, val);
}

static inline void mcu_wakeup_gpio_get_locked(struct nanohub_data *data,
					      int priority_lock)
{
	atomic_inc(&data->wakeup_lock_cnt);
	if (!priority_lock && atomic_inc_return(&data->wakeup_cnt) == 1 &&
	    !nanohub_has_priority_lock_locked(data))
		mcu_wakeup_gpio_set_value(data, 0);
}

static inline bool mcu_wakeup_gpio_put_locked(struct nanohub_data *data,
					      int priority_lock)
{
	bool gpio_done = priority_lock ?
			 atomic_read(&data->wakeup_cnt) == 0 :
			 atomic_dec_and_test(&data->wakeup_cnt);
	bool done = atomic_dec_and_test(&data->wakeup_lock_cnt);

	if (!nanohub_has_priority_lock_locked(data))
		mcu_wakeup_gpio_set_value(data, gpio_done ? 1 : 0);

	return done;
}

static inline bool mcu_wakeup_gpio_is_locked(struct nanohub_data *data)
{
	return atomic_read(&data->wakeup_lock_cnt) != 0;
}

static inline void nanohub_handle_irq1(struct nanohub_data *data)
{
	bool locked;

	spin_lock(&data->wakeup_wait.lock);
	locked = mcu_wakeup_gpio_is_locked(data);
	spin_unlock(&data->wakeup_wait.lock);
	if (!locked)
		nanohub_notify_thread(data);
	else
		wake_up_interruptible_sync(&data->wakeup_wait);
}

static inline void nanohub_handle_irq2(struct nanohub_data *data)
{
	nanohub_notify_thread(data);
}

static inline bool mcu_wakeup_try_lock(struct nanohub_data *data, int key)
{
	/* implementation contains memory barrier */
	return atomic_cmpxchg(&data->wakeup_acquired, 0, key) == 0;
}

static inline void mcu_wakeup_unlock(struct nanohub_data *data, int key)
{
	WARN(atomic_cmpxchg(&data->wakeup_acquired, key, 0) != key,
	     "%s: failed to unlock with key %d; current state: %d",
	     __func__, key, atomic_read(&data->wakeup_acquired));
}

static inline void nanohub_set_state(struct nanohub_data *data, int state)
{
	atomic_set(&data->thread_state, state);
	smp_mb__after_atomic(); /* updated thread state is now visible */
}

static inline int nanohub_get_state(struct nanohub_data *data)
{
	smp_mb__before_atomic(); /* wait for all updates to finish */
	return atomic_read(&data->thread_state);
}

static inline void nanohub_clear_err_cnt(struct nanohub_data *data)
{
	data->kthread_err_cnt = data->wakeup_err_cnt = 0;
}

/* the following fragment is based on wait_event_* code from wait.h */
#define wait_event_interruptible_timeout_locked(q, cond, tmo)		\
({									\
	long __ret = (tmo);						\
	DEFINE_WAIT(__wait);						\
	if (!(cond)) {							\
		for (;;) {						\
			__wait.flags &= ~WQ_FLAG_EXCLUSIVE;		\
			if (list_empty(&__wait.task_list))		\
				__add_wait_queue_tail(&(q), &__wait);	\
			set_current_state(TASK_INTERRUPTIBLE);		\
			if ((cond))					\
				break;					\
			if (signal_pending(current)) {			\
				__ret = -ERESTARTSYS;			\
				break;					\
			}						\
			spin_unlock(&(q).lock);				\
			__ret = schedule_timeout(__ret);		\
			spin_lock(&(q).lock);				\
			if (!__ret) {					\
				if ((cond))				\
					__ret = 1;			\
				break;					\
			}						\
		}							\
		__set_current_state(TASK_RUNNING);			\
		if (!list_empty(&__wait.task_list))			\
			list_del_init(&__wait.task_list);		\
		else if (__ret == -ERESTARTSYS &&			\
			 /*reimplementation of wait_abort_exclusive() */\
			 waitqueue_active(&(q)))			\
			__wake_up_locked_key(&(q), TASK_INTERRUPTIBLE,	\
			NULL);						\
	} else {							\
		__ret = 1;						\
	}								\
	__ret;								\
})									\

int request_wakeup_ex(struct nanohub_data *data, long timeout_ms,
		      int key, int lock_mode)
{
	long timeout;
	bool priority_lock = lock_mode > LOCK_MODE_NORMAL;
	struct device *sensor_dev = data->io[ID_NANOHUB_SENSOR].dev;
	int ret;
	ktime_t ktime_delta;
	ktime_t wakeup_ktime;

	spin_lock(&data->wakeup_wait.lock);
	mcu_wakeup_gpio_get_locked(data, priority_lock);
	timeout = (timeout_ms != MAX_SCHEDULE_TIMEOUT) ?
		   msecs_to_jiffies(timeout_ms) :
		   MAX_SCHEDULE_TIMEOUT;

	if (!priority_lock && !data->wakeup_err_cnt)
		wakeup_ktime = ktime_get_boottime();
	timeout = wait_event_interruptible_timeout_locked(
			data->wakeup_wait,
			((priority_lock || nanohub_irq1_fired(data)) &&
			 mcu_wakeup_try_lock(data, key)),
			timeout
		  );

	if (timeout <= 0) {
		if (!timeout && !priority_lock) {
			if (!data->wakeup_err_cnt)
				data->wakeup_err_ktime = wakeup_ktime;
			ktime_delta = ktime_sub(ktime_get_boottime(),
						data->wakeup_err_ktime);
			data->wakeup_err_cnt++;
			if (ktime_to_ns(ktime_delta) > WAKEUP_ERR_TIME_NS
				&& data->wakeup_err_cnt > WAKEUP_ERR_CNT) {
				mcu_wakeup_gpio_put_locked(data, priority_lock);
				spin_unlock(&data->wakeup_wait.lock);
				dev_info(sensor_dev,
					"wakeup: hard reset due to consistent error\n");
				ret = nanohub_hw_reset(data);
				if (ret) {
					dev_info(sensor_dev,
						"%s: failed to reset nanohub: ret=%d\n",
						__func__, ret);
				}
				return -ETIME;
			}
		}
		mcu_wakeup_gpio_put_locked(data, priority_lock);

		if (timeout == 0)
			timeout = -ETIME;
	} else {
		data->wakeup_err_cnt = 0;
		timeout = 0;
	}
	spin_unlock(&data->wakeup_wait.lock);

	return timeout;
}

void release_wakeup_ex(struct nanohub_data *data, int key, int lock_mode)
{
	bool done;
	bool priority_lock = lock_mode > LOCK_MODE_NORMAL;

	spin_lock(&data->wakeup_wait.lock);
	done = mcu_wakeup_gpio_put_locked(data, priority_lock);
	mcu_wakeup_unlock(data, key);
	spin_unlock(&data->wakeup_wait.lock);

	if (!done)
		wake_up_interruptible_sync(&data->wakeup_wait);
	else if (nanohub_irq1_fired(data) || nanohub_irq2_fired(data))
		nanohub_notify_thread(data);
}

int nanohub_wait_for_interrupt(struct nanohub_data *data)
{
	int ret = -EFAULT;

	/* release the wakeup line, and wait for nanohub to send
	 * us an interrupt indicating the transaction completed.
	 */
	spin_lock(&data->wakeup_wait.lock);
	if (mcu_wakeup_gpio_is_locked(data)) {
		mcu_wakeup_gpio_set_value(data, 1);
		ret = wait_event_interruptible_locked(data->wakeup_wait,
						      nanohub_irq1_fired(data));
		mcu_wakeup_gpio_set_value(data, 0);
	}
	spin_unlock(&data->wakeup_wait.lock);

	return ret;
}

int nanohub_wakeup_eom(struct nanohub_data *data, bool repeat)
{
	int ret = -EFAULT;

	spin_lock(&data->wakeup_wait.lock);
	if (mcu_wakeup_gpio_is_locked(data)) {
		mcu_wakeup_gpio_set_value(data, 1);
		if (repeat)
			mcu_wakeup_gpio_set_value(data, 0);
		ret = 0;
	}
	spin_unlock(&data->wakeup_wait.lock);

	return ret;
}

static void __nanohub_interrupt_cfg(struct nanohub_data *data,
				    u8 interrupt, bool mask)
{
	int ret;
	uint8_t mask_ret;
	int cnt = 10;
	struct device *dev = data->io[ID_NANOHUB_SENSOR].dev;
	int cmd = mask ? CMD_COMMS_MASK_INTR : CMD_COMMS_UNMASK_INTR;

	do {
		ret = request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS);
		if (ret) {
			dev_err(dev,
				"%s: interrupt %d %smask failed: ret=%d\n",
				__func__, interrupt, mask ? "" : "un", ret);
			return;
		}

		ret =
		    nanohub_comms_tx_rx_retrans(data, cmd,
						&interrupt, sizeof(interrupt),
						&mask_ret, sizeof(mask_ret),
						false, 10, 0);
		release_wakeup(data);
		dev_dbg(dev,
			"%smasking interrupt %d, ret=%d, mask_ret=%d\n",
			mask ? "" : "un",
			interrupt, ret, mask_ret);
	} while ((ret != 1 || mask_ret != 1) && --cnt > 0);
}

static inline void nanohub_mask_interrupt(struct nanohub_data *data,
					  u8 interrupt)
{
	__nanohub_interrupt_cfg(data, interrupt, true);
}

static inline void nanohub_unmask_interrupt(struct nanohub_data *data,
					    u8 interrupt)
{
	__nanohub_interrupt_cfg(data, interrupt, false);
}

static ssize_t nanohub_wakeup_query(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	const struct nanohub_platform_data *pdata = data->pdata;

	nanohub_clear_err_cnt(data);
	if (nanohub_irq1_fired(data) || nanohub_irq2_fired(data))
		wake_up_interruptible(&data->wakeup_wait);

	return scnprintf(buf, PAGE_SIZE, "WAKEUP: %d INT1: %d "
			 "INT2: %d INT3: %d\n",
			 gpio_get_value(pdata->wakeup_gpio),
			 gpio_get_value(pdata->irq1_gpio),
			 data->irq2 ? gpio_get_value(pdata->irq2_gpio) : -1,
			 data->irq3 ? gpio_get_value(pdata->irq3_gpio) : -1);
}

static ssize_t nanohub_app_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	struct {
		uint64_t appId;
		uint32_t appVer;
		uint32_t appSize;
	} __packed buffer;
	uint32_t i = 0;
	int ret;
	ssize_t len = 0;

	do {
		if (request_wakeup(data))
			return -ERESTARTSYS;

		if (nanohub_comms_tx_rx_retrans
		    (data, CMD_COMMS_QUERY_APP_INFO, (uint8_t *)&i,
		     sizeof(i), (u8 *)&buffer, sizeof(buffer),
		     false, 10, 10) == sizeof(buffer)) {
			ret =
			    scnprintf(buf + len, PAGE_SIZE - len,
				      "app: %d id: %016llx ver: %08x size: %08x\n",
				      i, buffer.appId, buffer.appVer,
				      buffer.appSize);
			if (ret > 0) {
				len += ret;
				i++;
			}
		} else {
			ret = -1;
		}

		release_wakeup(data);
	} while (ret > 0);

	return len;
}

static ssize_t nanohub_firmware_query(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	uint16_t buffer[6];

	if (request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS))
		return -ERESTARTSYS;

	if (nanohub_comms_tx_rx_retrans
	    (data, CMD_COMMS_GET_OS_HW_VERSIONS, NULL, 0, (uint8_t *)&buffer,
	     sizeof(buffer), false, 10, 10) == sizeof(buffer)) {
		release_wakeup(data);
		data->nanohub_hw_type = buffer[0];
		data->nanohub_variant_version = buffer[5] << 16 | buffer[4];
		pr_info("nanohub_firmware_query: hw type 0x%04x, variant vesion 0x%08x\n",
				data->nanohub_hw_type,
				data->nanohub_variant_version);
		return scnprintf(buf, PAGE_SIZE,
				 "hw type: %04x hw ver: %04x bl ver: %04x os ver: %04x variant ver: %08x\n",
				 buffer[0], buffer[1], buffer[2], buffer[3],
				 buffer[5] << 16 | buffer[4]);
	} else {
		release_wakeup(data);
		return 0;
	}
}

static inline int nanohub_wakeup_lock(struct nanohub_data *data, int mode)
{
	int ret;

	if (data->irq2)
		disable_irq(data->irq2);
	else
		nanohub_mask_interrupt(data, 2);

	ret = request_wakeup_ex(data,
				mode == LOCK_MODE_SUSPEND_RESUME ?
				SUSPEND_TIMEOUT_MS : WAKEUP_TIMEOUT_MS,
				KEY_WAKEUP_LOCK, mode);
	if (ret < 0) {
		if (data->irq2)
			enable_irq(data->irq2);
		else
			nanohub_unmask_interrupt(data, 2);
		return ret;
	}

	if (mode == LOCK_MODE_IO || mode == LOCK_MODE_IO_BL)
		ret = nanohub_bl_open(data);
	if (ret < 0) {
		release_wakeup_ex(data, KEY_WAKEUP_LOCK, mode);
		return ret;
	}
	if (mode != LOCK_MODE_SUSPEND_RESUME)
		disable_irq(data->irq1);

	atomic_set(&data->lock_mode, mode);
	mcu_wakeup_gpio_set_value(data, mode != LOCK_MODE_IO_BL);

	return 0;
}

/* returns lock mode used to perform this lock */
static inline int nanohub_wakeup_unlock(struct nanohub_data *data)
{
	int mode = atomic_read(&data->lock_mode);

	atomic_set(&data->lock_mode, LOCK_MODE_NONE);
	if (mode != LOCK_MODE_SUSPEND_RESUME)
		enable_irq(data->irq1);
	if (mode == LOCK_MODE_IO || mode == LOCK_MODE_IO_BL)
		nanohub_bl_close(data);
	if (data->irq2)
		enable_irq(data->irq2);
	release_wakeup_ex(data, KEY_WAKEUP_LOCK, mode);
	if (!data->irq2)
		nanohub_unmask_interrupt(data, 2);
	nanohub_notify_thread(data);

	return mode;
}

static void __nanohub_hw_reset(struct nanohub_data *data, int boot0)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	if (pdata->nreset_polarity == NRESET_ACTIVE_LOW)
		gpio_set_value(pdata->nreset_gpio, 0);
	else
		gpio_set_value(pdata->nreset_gpio, 1);

	gpio_set_value(pdata->boot0_gpio, boot0 > 0);
	usleep_range(30, 40);
	if (pdata->nreset_polarity == NRESET_ACTIVE_LOW)
		gpio_set_value(pdata->nreset_gpio, 1);
	else
		gpio_set_value(pdata->nreset_gpio, 0);

	if (boot0 > 0)
		usleep_range(70000, 75000);
	else if (!boot0)
		usleep_range(750000, 800000);
	nanohub_clear_err_cnt(data);
}

static int nanohub_hw_reset(struct nanohub_data *data)
{
	int ret;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_RESET);

	if (!ret) {
		__nanohub_hw_reset(data, 0);
		nanohub_wakeup_unlock(data);
	}

	return ret;
}

static ssize_t nanohub_try_hw_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	int ret;

	ret = nanohub_hw_reset(data);

	return ret < 0 ? ret : count;
}

int __nanohub_set_mode_pin(struct nanohub_data *data, enum AP_GPIO_CMD mode)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	gpio_set_value(pdata->mode1_gpio, (mode & 0x08) ? 1 : 0);
	gpio_set_value(pdata->mode2_gpio, (mode & 0x04) ? 1 : 0);
	gpio_set_value(pdata->mode3_gpio, (mode & 0x02) ? 1 : 0);
	gpio_set_value(pdata->mode4_gpio, (mode & 0x01) ? 1 : 0);
	usleep_range(1000, 1500);

	if (gpio_get_value(pdata->int_gpio) != 0) {
		gpio_set_value(pdata->int_gpio, 0);
		usleep_range(1000, 1500);
	}
	gpio_set_value(pdata->int_gpio, 1);
		/*creat a intterupt(high rise edge) to mcu*/
	usleep_range(10000, 15000);
	gpio_set_value(pdata->int_gpio, 0);

	gpio_set_value(pdata->mode1_gpio, 0);
	gpio_set_value(pdata->mode2_gpio, 0);
	gpio_set_value(pdata->mode3_gpio, 0);
	gpio_set_value(pdata->mode4_gpio, 0);
	return 0;
}

int __nanohub_send_AP_cmd(struct nanohub_data *data, enum AP_GPIO_CMD mode)
{
	int ret = 0;

	if ((mode < GPIO_CMD_POWEROFF) || (mode > GPIO_CMD_RESEND)) {
		pr_err("nanohub: invalid mode = %d\n", mode);
		return -EINVAL;
	}

	mutex_lock(&(data->hub_mode_set_lock));

	switch (mode) {
	case GPIO_CMD_POWEROFF: /*0000*/
	case GPIO_CMD_BAND:     /*0001*/
		atomic_set(&data->hub_mode_ap_pwr_down, mode);    /* Only remember the state after AP pwr off */
		atomic_set(&data->hub_mode_ap_active, mode);
		ret = __nanohub_set_mode_pin(data, mode);
		break;

	case GPIO_CMD_AMBIENT:	/*0010*/
	case GPIO_CMD_NORMAL:   /*0011*/
	case GPIO_CMD_SUSPEND:  /*0110*/
	case GPIO_CMD_RESUME:   /*0111*/
		ret = __nanohub_set_mode_pin(data, mode);
		atomic_set(&data->hub_mode_ap_active, mode);
		break;

	case GPIO_CMD_FLASH_ERASE:       /*0100*/
	case GPIO_CMD_REQUEST_FUELGAUGE: /*0101*/
	case GPIO_CMD_FORCE_REQUEST_FUELGAUGE: /*1000*/
	case GPIO_CMD_TEST:              /*1111*/
		ret = __nanohub_set_mode_pin(data, mode);
		break;

	case GPIO_CMD_RESEND:   /*send mode again if mcu needed*/
		ret = __nanohub_set_mode_pin(data,
				atomic_read(&data->hub_mode_ap_active));
		break;

	default:
		break;
	}

	mutex_unlock(&(data->hub_mode_set_lock));

	return ret;
}

static ssize_t nanohub_mode_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	int mode = 0;

	if (sscanf(buf, "%d\n", &mode) > 0)
		__nanohub_send_AP_cmd(data, mode);

	return count;
}

static ssize_t nanohub_lcd_mutex(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	int lcd_mutex = 0;

	if (sscanf(buf, "%d\n", &lcd_mutex) > 0)
		atomic_set(&data->lcd_mutex,
			lcd_mutex?LCD_MUTEX_ON:LCD_MUTEX_OFF);

	return count;
}

static ssize_t nanohub_lcd_mutex_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	return scnprintf(buf, PAGE_SIZE,
		"%d\n", atomic_read(&data->lcd_mutex));
}

static ssize_t nanohub_sensorhal_status_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	int sensorhal_alive = 0;

	if (sscanf(buf, "%d\n", &sensorhal_alive) > 0)
		atomic_set(&data->sensor_hal_alive,
			sensorhal_alive?SENSOR_HAL_ALIVED:SENSOR_HAL_DEAD);

	return count;
}

static ssize_t nanohub_sensorhal_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);

	return scnprintf(buf, PAGE_SIZE,
		"%d\n", atomic_read(&data->sensor_hal_alive));
}

static ssize_t nanohub_get_cmdline(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"%s\n", saved_command_line);
}

static ssize_t fuelgauge_wakelock_time_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);

	return scnprintf(buf, PAGE_SIZE,
		"%lld\n", data->fg_info->wakelock_active_time);
}

static ssize_t nanohub_erase_shared(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	uint8_t status = CMD_ACK;
	int ret;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, 1);

	status = nanohub_bl_erase_shared(data);
	dev_info(dev, "nanohub_bl_erase_shared: status=%02x\n",
		 status);

	__nanohub_hw_reset(data, 0);
	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_erase_shared_bl(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	uint8_t status = CMD_ACK;
	int ret;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO_BL);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, -1);

	status = nanohub_bl_erase_shared_bl(data);
	dev_info(dev, "%s: status=%02x\n", __func__, status);

	__nanohub_hw_reset(data, 0);
	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_download_bl(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	const struct firmware *fw_entry;
	const char *firmware_name;
	int ret;
	uint8_t status = CMD_ACK;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0) {
		atomic_set(&data->download_bl_status,
						DOWNLOAD_BL_TIMEOUT);
		return ret;
	}

	atomic_set(&data->download_bl_status,
				DOWNLOAD_BL_RUNNING);

	__nanohub_hw_reset(data, 1);

	if (!strncmp(buf, "nanohub.full.", 13))
		firmware_name = buf;
	else
		firmware_name = "nanohub.full.bin";

	ret = request_firmware(&fw_entry, firmware_name, dev);
	if (ret) {
		dev_err(dev, "%s: request fw for nanohub: %s failed. err=%d\n",
			__func__, firmware_name, ret);
	} else {
		dev_info(dev, "%s: request fw for nanohub: %s successfully.\n",
			__func__, firmware_name);
		status = nanohub_bl_download(data, pdata->bl_addr,
					     fw_entry->data, fw_entry->size);
		dev_info(dev, "%s: status=%02x\n", __func__, status);
		if (status == CMD_ACK)
			atomic_set(&data->download_bl_status,
				DOWNLOAD_BL_SUCCESS);
		else
			atomic_set(&data->download_bl_status,
				DOWNLOAD_BL_FAILED);

		release_firmware(fw_entry);
	}
	__nanohub_hw_reset(data, 0);
	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_download_bl_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	char status[][10] = {"Not Start",
						"Running",
						"Success",
						"Failed",
						"Time Out"};
	return scnprintf(buf, PAGE_SIZE,
		"%s\n", status[data->download_bl_status.counter]);
}

static ssize_t nanohub_download_kernel(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	const struct firmware *fw_entry;
	int ret;
	int ret_val;

	ret = request_firmware(&fw_entry, "nanohub.update.bin", dev);
	if (ret) {
		dev_err(dev, "nanohub_download_kernel: err=%d\n", ret);
		ret_val = -EIO;
	} else {
		ret =
		    nanohub_comms_kernel_download(data, fw_entry->data,
						  fw_entry->size);

		release_firmware(fw_entry);

		ret_val = count;
	}

	return ret_val;
}

static ssize_t nanohub_download_kernel_bl(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{

	struct nanohub_data *data = dev_get_nanohub_data(dev);
	const struct firmware *fw_entry;
	int ret;
	uint8_t status = CMD_ACK;

	ret = request_firmware(&fw_entry, "nanohub.kernel.signed", dev);
	if (ret) {
		dev_err(dev, "%s: err=%d\n", __func__, ret);
	} else {
		ret = nanohub_wakeup_lock(data, LOCK_MODE_IO_BL);
		if (ret < 0)
			return ret;

		__nanohub_hw_reset(data, -1);

		status = nanohub_bl_erase_shared_bl(data);
		dev_info(dev, "%s: (erase) status=%02x\n", __func__, status);
		if (status == CMD_ACK) {
			status = nanohub_bl_write_memory(data, 0x50000000,
							 fw_entry->size,
							 fw_entry->data);
			mcu_wakeup_gpio_set_value(data, 1);
			dev_info(dev, "%s: (write) status=%02x\n",
						__func__, status);
			if (status == CMD_ACK) {
				status = nanohub_bl_update_finished(data);
				dev_info(dev, "%s: (finish) status=%02x\n",
						__func__, status);
			}
		} else {
			mcu_wakeup_gpio_set_value(data, 1);
		}

		__nanohub_hw_reset(data, 0);
		nanohub_wakeup_unlock(data);

		release_firmware(fw_entry);
	}

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_download_app(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	const struct firmware *fw_entry;
	char buffer[70];
	int i, ret, ret1, ret2, file_len = 0, appid_len = 0, ver_len = 0;
	const char *appid = NULL, *ver = NULL;
	unsigned long version;
	uint64_t id;
	uint32_t cur_version;
	bool update = true;

	for (i = 0; i < count; i++) {
		if (buf[i] == ' ') {
			if (i + 1 == count) {
				break;
			} else {
				if (appid == NULL)
					appid = buf + i + 1;
				else if (ver == NULL)
					ver = buf + i + 1;
				else
					break;
			}
		} else if (buf[i] == '\n' || buf[i] == '\r') {
			break;
		} else {
			if (ver)
				ver_len++;
			else if (appid)
				appid_len++;
			else
				file_len++;
		}
	}

	if (file_len > 64 || appid_len > 16 || ver_len > 8 || file_len < 1)
		return -EIO;

	memcpy(buffer, buf, file_len);
	memcpy(buffer + file_len, ".napp", 5);
	buffer[file_len + 5] = '\0';

	ret = request_firmware(&fw_entry, buffer, dev);
	if (ret) {
		dev_err(dev, "nanohub_download_app(%s): err=%d\n",
			buffer, ret);
		return -EIO;
	}
	if (appid_len > 0 && ver_len > 0) {
		memcpy(buffer, appid, appid_len);
		buffer[appid_len] = '\0';

		ret1 = kstrtoull(buffer, 16, &id);

		memcpy(buffer, ver, ver_len);
		buffer[ver_len] = '\0';

		ret2 = kstrtoul(buffer, 16, &version);

		if (ret1 == 0 && ret2 == 0) {
			if (request_wakeup(data))
				return -ERESTARTSYS;
			if (nanohub_comms_tx_rx_retrans
			    (data, CMD_COMMS_GET_APP_VERSIONS,
			     (uint8_t *)&id, sizeof(id),
			     (uint8_t *)&cur_version,
			     sizeof(cur_version), false, 10,
			     10) == sizeof(cur_version)) {
				if (cur_version == version)
					update = false;
			}
			release_wakeup(data);
		}
	}

	if (update)
		ret =
		    nanohub_comms_app_download(data, fw_entry->data,
					       fw_entry->size);

	release_firmware(fw_entry);

	return count;
}

static ssize_t nanohub_lock_bl(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	int ret;
	uint8_t status = CMD_ACK;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, 1);

	gpio_set_value(data->pdata->boot0_gpio, 0);
	/* this command reboots itself */
	status = nanohub_bl_lock(data);
	dev_info(dev, "%s: status=%02x\n", __func__, status);
	msleep(350);

	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_unlock_bl(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_nanohub_data(dev);
	int ret;
	uint8_t status = CMD_ACK;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, 1);

	gpio_set_value(data->pdata->boot0_gpio, 0);
	/* this command reboots itself (erasing the flash) */
	status = nanohub_bl_unlock(data);
	dev_info(dev, "%s: status=%02x\n", __func__, status);
	msleep(20);

	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static struct device_attribute attributes[] = {
	__ATTR(wakeup, 0440, nanohub_wakeup_query, NULL),
	__ATTR(app_info, 0440, nanohub_app_info, NULL),
	__ATTR(firmware_version, 0440, nanohub_firmware_query, NULL),
	__ATTR(download_bl, 0220, NULL, nanohub_download_bl),
	__ATTR(download_kernel, 0220, NULL, nanohub_download_kernel),
	__ATTR(download_kernel_bl, 0220, NULL, nanohub_download_kernel_bl),
	__ATTR(download_app, 0220, NULL, nanohub_download_app),
	__ATTR(erase_shared, 0220, NULL, nanohub_erase_shared),
	__ATTR(erase_shared_bl, 0220, NULL, nanohub_erase_shared_bl),
	__ATTR(reset, 0220, NULL, nanohub_try_hw_reset),
	__ATTR(lock, 0220, NULL, nanohub_lock_bl),
	__ATTR(unlock, 0220, NULL, nanohub_unlock_bl),
	__ATTR(mode, 0220, NULL, nanohub_mode_set),
	__ATTR(download_bl_status, 0444, nanohub_download_bl_status, NULL),
	__ATTR(lcd_mutex, 0660, nanohub_lcd_mutex_status, nanohub_lcd_mutex),
	__ATTR(sensorhal_alive, 0660, nanohub_sensorhal_status_get,
		nanohub_sensorhal_status_set),
	__ATTR(fuelgauge_wakelock_time, 0440, fuelgauge_wakelock_time_get,
		NULL),
	__ATTR(cmdline, 0440, nanohub_get_cmdline, NULL),
};

static inline int nanohub_create_sensor(struct nanohub_data *data)
{
	int i, ret;
	struct device *sensor_dev = data->io[ID_NANOHUB_SENSOR].dev;

	for (i = 0, ret = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(sensor_dev, &attributes[i]);
		if (ret) {
			dev_err(sensor_dev,
				"create sysfs attr %d [%s] failed; err=%d\n",
				i, attributes[i].attr.name, ret);
			goto fail_attr;
		}
	}

	ret = sysfs_create_link(&sensor_dev->kobj,
				&data->iio_dev->dev.kobj, "iio");
	if (ret) {
		dev_err(sensor_dev,
			"sysfs_create_link failed; err=%d\n", ret);
		goto fail_attr;
	}
	goto done;

fail_attr:
	for (i--; i >= 0; i--)
		device_remove_file(sensor_dev, &attributes[i]);
done:
	return ret;
}

static int nanohub_create_devices(struct nanohub_data *data)
{
	int i, ret;
	static const char *names[ID_NANOHUB_MAX] = {
			"nanohub", "nanohub_comms",
			"nanohub_hr_log", "nanohub_custom_flash"
	};

	for (i = 0; i < ID_NANOHUB_MAX; ++i) {
		struct nanohub_io *io = &data->io[i];

		nanohub_io_init(io, data, device_create(sensor_class, NULL,
							MKDEV(major, i),
							io, names[i]));
		if (IS_ERR(io->dev)) {
			ret = PTR_ERR(io->dev);
			pr_err("nanohub: device_create failed for %s; err=%d\n",
			       names[i], ret);
			goto fail_dev;
		}
	}

	ret = nanohub_create_sensor(data);
	if (!ret)
		goto done;

fail_dev:
	for (--i; i >= 0; --i)
		device_destroy(sensor_class, MKDEV(major, i));
done:
	return ret;
}

static int nanohub_match_devt(struct device *dev, const void *data)
{
	const dev_t *devt = data;

	return dev->devt == *devt;
}

static int nanohub_open(struct inode *inode, struct file *file)
{
	dev_t devt = inode->i_rdev;
	struct device *dev;

	dev = class_find_device(sensor_class, NULL, &devt, nanohub_match_devt);
	if (dev) {
		file->private_data = dev_get_drvdata(dev);
		nonseekable_open(inode, file);
		return 0;
	}

	return -ENODEV;
}

static ssize_t nanohub_read(struct file *file, char *buffer, size_t length,
			    loff_t *offset)
{
	struct nanohub_io *io = file->private_data;
	struct nanohub_data *data = io->data;
	struct nanohub_buf *buf;
	int ret;

	if (!nanohub_io_has_buf(io) && (file->f_flags & O_NONBLOCK))
		return -EAGAIN;

	buf = nanohub_io_get_buf(io, true);
	if (IS_ERR_OR_NULL(buf))
		return PTR_ERR(buf);

	ret = copy_to_user(buffer, buf->buffer, buf->length);
	if (ret != 0)
		ret = -EFAULT;
	else
		ret = buf->length;

	nanohub_io_put_buf(&data->free_pool, buf);

	return ret;
}

static ssize_t nanohub_write(struct file *file, const char *buffer,
			     size_t length, loff_t *offset)
{
	struct nanohub_io *io = file->private_data;
	struct nanohub_data *data = io->data;
	int ret;

	mutex_lock(&(data->nanohub_write_lock));
	ret = request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS);
	if (ret) {
		pr_err("nanohub: error to request wakeup, ret = %d\n", ret);
		mutex_unlock(&(data->nanohub_write_lock));
		return ret;
	}

	ret = nanohub_comms_write(data, buffer, length);

	release_wakeup(data);
	mutex_unlock(&(data->nanohub_write_lock));

	return ret;
}

static unsigned int nanohub_poll(struct file *file, poll_table *wait)
{
	struct nanohub_io *io = file->private_data;
	unsigned int mask = POLLOUT | POLLWRNORM;

	poll_wait(file, &io->buf_wait, wait);

	if (nanohub_io_has_buf(io))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int nanohub_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static void nanohub_destroy_devices(struct nanohub_data *data)
{
	int i;
	struct device *sensor_dev = data->io[ID_NANOHUB_SENSOR].dev;

	sysfs_remove_link(&sensor_dev->kobj, "iio");
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(sensor_dev, &attributes[i]);
	for (i = 0; i < ID_NANOHUB_MAX; ++i)
		device_destroy(sensor_class, MKDEV(major, i));
}

static irqreturn_t nanohub_irq1(int irq, void *dev_id)
{
	struct nanohub_data *data = (struct nanohub_data *)dev_id;

	nanohub_handle_irq1(data);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
	if (NANOHUB_WAKEUP_TRACE_ON ==
		atomic_read(&data->st_wakeup_trace)) {
		data->wakeup_trace_irqs.nums_irq1++;
	}
#endif

	return IRQ_HANDLED;
}

static irqreturn_t nanohub_irq2(int irq, void *dev_id)
{
	struct nanohub_data *data = (struct nanohub_data *)dev_id;

	nanohub_handle_irq2(data);

	if (data->nanohub_hw_type == 0x4d70	&&
		data->nanohub_variant_version < 0x0000001b)
		__nanohub_send_AP_cmd(data, GPIO_CMD_RESEND);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
	if (NANOHUB_WAKEUP_TRACE_ON ==
		atomic_read(&data->st_wakeup_trace)) {
		data->wakeup_trace_irqs.nums_irq2++;
	}
#endif

	return IRQ_HANDLED;
}


static irqreturn_t nanohub_irq3(int irq, void *dev_id)
{
	struct nanohub_data *data = (struct nanohub_data *)dev_id;

	__nanohub_send_AP_cmd(data, GPIO_CMD_RESEND);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
	if (NANOHUB_WAKEUP_TRACE_ON ==
		atomic_read(&data->st_wakeup_trace)) {
		data->wakeup_trace_irqs.nums_irq3++;
	}
#endif

	return IRQ_HANDLED;
}

static bool nanohub_os_log(struct nanohub_data *data, char *buffer, int len)
{
	uint32_t event_id =
		le32_to_cpu((((uint32_t *)buffer)[0]) & 0x7FFFFFFF);

	if (event_id == OS_LOG_EVENTID ||
		(event_id == OS_LOG_TO_HAL_EVENTID &&
		SENSOR_HAL_DEAD ==
		atomic_read(&data->sensor_hal_alive))) {
		char *mtype, *mdata = &buffer[5];

		buffer[len] = 0x00;

		switch (buffer[4]) {
		case 'E':
			mtype = KERN_ERR;
			break;
		case 'W':
			mtype = KERN_WARNING;
			break;
		case 'I':
			mtype = KERN_INFO;
			break;
		case 'D':
			mtype = KERN_DEBUG;
			break;
		default:
			mtype = KERN_DEFAULT;
			mdata--;
			break;
		}
		printk("%snanohub: %s", mtype, mdata);
		return true;
	} else {
		return false;
	}
}

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
static const char * const interrupt_name[] = {
					"INVALID",
					"SENSOR_WAKEUP",
					"SENSOR_NON_WAKEUP"};

static char *nanohub_get_event_name_from_id(
					uint32_t event_id,
					uint32_t event_sub_id)
{
	char *evt_name;

	if (event_id > FIRST_SENSOR_EVENTID &&
	    event_id <= LAST_SENSOR_EVENTID)
		evt_name = "EVENT_SENSOR";
	else if (event_id == APP_TO_HOST_EVENTID) {
		if (event_sub_id == APP_TO_HOST_EVENT_SUBID_FUELGAUGE)
			evt_name = "EVENT_FUELGAUGE";
		else if (event_sub_id == APP_TO_HOST_EVENT_SUBID_HR_LOG)
			evt_name = "EVENT_HR_LOG";
		else if (event_sub_id == APP_TO_HOST_EVENT_SUBID_FLASH)
			evt_name = "EVENT_CUSTOM_FLASH";
		else if (event_sub_id == APP_TO_HOST_EVENT_SUBID_CALIBRATE)
			evt_name = "EVENT_CALIBRATE_RESULT";
		else if (event_sub_id == APP_TO_HOST_EVENT_SUBID_SELFTEST)
			evt_name = "EVENT_SELFTEST_RESULT";
		else
			evt_name = "EVENT_OTHERS";
	} else if (event_id == OS_LOG_EVENTID)
		evt_name = "EVENT_LOG";
	else if (event_id == APP_FROM_HOST_EVENTID)
		evt_name = "EVENT_APP_FROM_HOST";
	else
		evt_name = "EVENT_UNKNOWN";

	return evt_name;
}

static bool nanohub_wakeup_trace_event_same(
				struct wakeup_trace_listnode *new_event,
				struct wakeup_trace_listnode *cur_event)
{
	if (new_event->trace_event.event_id != cur_event->trace_event.event_id)
		return false;
	if (new_event->trace_event.event_sub_id	!=
		cur_event->trace_event.event_sub_id)
			return false;

	return true;
}

static void nanohub_wakeup_trace_add_to_list(
				struct wakeup_trace_listnode *trace_head,
				struct nanohub_buf **buf, int len)
{
	struct wakeup_trace_listnode *new_trace_event =
			kzalloc(sizeof(struct wakeup_trace_listnode),
				GFP_KERNEL);

	struct wakeup_trace_listnode *tmp;
	bool is_same_type = false;

	memset(new_trace_event, 0, sizeof(new_trace_event));
	new_trace_event->length =
		len > sizeof(new_trace_event->buffer) ?
		sizeof(new_trace_event->buffer) : len;

	memcpy(new_trace_event->buffer,
		(*buf)->buffer, new_trace_event->length);
	new_trace_event->length = len;
	new_trace_event->trace_event.event_id =
		le32_to_cpu((((uint32_t *)(*buf)->buffer)[0]) & 0x7FFFFFFF);

	if (new_trace_event->trace_event.event_id > FIRST_SENSOR_EVENTID &&
		new_trace_event->trace_event.event_id <= LAST_SENSOR_EVENTID) {
			new_trace_event->trace_event.interrupt =
				(*buf)->buffer[sizeof(uint32_t) +
				sizeof(uint64_t) + 3];
	} else if (new_trace_event->trace_event.event_id ==
				APP_TO_HOST_EVENTID) {
		struct SensorAppEventHeader *p_SensorAppEventHeader;

		p_SensorAppEventHeader =
			(struct SensorAppEventHeader *)
			&((*buf)->buffer[sizeof(uint32_t)
			+ sizeof(struct HostHubRawPacket)]);
		if (!is_fuel_gauge_data(*buf, len))
			new_trace_event->trace_event.event_sub_id =
				APP_TO_HOST_EVENT_SUBID_FUELGAUGE;
		else if (!is_hr_log_data(*buf, len))
			new_trace_event->trace_event.event_sub_id =
				APP_TO_HOST_EVENT_SUBID_HR_LOG;
		else if (!is_custom_flash_data(*buf, len))
			new_trace_event->trace_event.event_sub_id =
				APP_TO_HOST_EVENT_SUBID_FLASH;
		else if (p_SensorAppEventHeader->msgId ==
				SENSOR_APP_MSG_ID_CAL_RESULT)
			new_trace_event->trace_event.event_sub_id =
				APP_TO_HOST_EVENT_SUBID_CALIBRATE;
		else if (p_SensorAppEventHeader->msgId ==
				SENSOR_APP_MSG_ID_TEST_RESULT)
			new_trace_event->trace_event.event_sub_id =
				APP_TO_HOST_EVENT_SUBID_CALIBRATE;
		else
			new_trace_event->trace_event.event_sub_id =
				APP_TO_HOST_EVENT_SUBID_OTHERS;
	}
	new_trace_event->trace_event.event_name =
		nanohub_get_event_name_from_id(
				new_trace_event->trace_event.event_id,
				new_trace_event->trace_event.event_sub_id);

	pr_debug("nanohub: wakeup trace: get new event id 0x%08x, "
			"event name %s, interrupt %s\n",
			new_trace_event->trace_event.event_id,
			new_trace_event->trace_event.event_name,
			interrupt_name[
				new_trace_event->trace_event.interrupt%3]);

	list_for_each_entry(tmp, &(trace_head->event_list), event_list) {
		is_same_type = nanohub_wakeup_trace_event_same(
							new_trace_event, tmp);
		if (is_same_type) {
			tmp->trace_event.event_count++;
			pr_debug("nanohub: wakeup trace: stored.\n");
			break;
		}
	}
	if (!is_same_type) {
		new_trace_event->trace_event.event_count++;
		list_add_tail(&(new_trace_event->event_list),
				&(trace_head->event_list));
		pr_debug("nanohub: wakeup trace: add to tail.\n");
	} else
		kzfree(new_trace_event);
}

static void nanohub_wakeup_trace_dump_list(
				struct wakeup_trace_listnode *trace_head,
				int call_reason)
{
	struct list_head *pos, *q;
	struct wakeup_trace_listnode *tmp;
	struct nanohub_data *data =
		container_of(trace_head, struct nanohub_data,
		wakeup_trace);
	static const char * const dump_reason_name[] = {
					"queue empty",
					"dump data",
					"ap no buffer",
					"clear irqs"};

	pr_info(
		"nanohub: wakeup trace: dump_reason: %s,"
		"irq_nums: irq1 %u, irq2 %u, irq3 %u\n",
		dump_reason_name[call_reason%4],
		data->wakeup_trace_irqs.nums_irq1,
		data->wakeup_trace_irqs.nums_irq2,
		data->wakeup_trace_irqs.nums_irq3);

	if (list_empty(&(trace_head->event_list)))
		return;

	list_for_each_safe(pos, q, &(trace_head->event_list)) {
		tmp = list_entry(pos, struct wakeup_trace_listnode, event_list);
		pr_info("nanohub: wakeup trace: event id 0x%08x, "
			"event name %s, interrupt %s, event_cnt %d\n",
			tmp->trace_event.event_id, tmp->trace_event.event_name,
			interrupt_name[tmp->trace_event.interrupt%3],
			tmp->trace_event.event_count);
		list_del(pos);
		kzfree(tmp);
	}
}
#endif

static void nanohub_process_buffer(struct nanohub_data *data,
				   struct nanohub_buf **buf,
				   int ret)
{
	uint32_t event_id;
	uint8_t interrupt;
	bool wakeup = false;
	struct nanohub_io *io = &data->io[ID_NANOHUB_SENSOR];

	data->kthread_err_cnt = 0;

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
	if (NANOHUB_WAKEUP_TRACE_ON ==
		atomic_read(&data->st_wakeup_trace))
		nanohub_wakeup_trace_add_to_list(
				&data->wakeup_trace, buf, ret);
#endif

	if (ret < 4 || nanohub_os_log(data, (*buf)->buffer, ret)) {
		release_wakeup(data);
		return;
	}

	(*buf)->length = ret;

	event_id = le32_to_cpu((((uint32_t *)(*buf)->buffer)[0]) & 0x7FFFFFFF);
	if (ret >= sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint32_t) &&
	    event_id > FIRST_SENSOR_EVENTID &&
	    event_id <= LAST_SENSOR_EVENTID) {
		interrupt = (*buf)->buffer[sizeof(uint32_t) +
					   sizeof(uint64_t) + 3];
		if (interrupt == WAKEUP_INTERRUPT)
			wakeup = true;
	}
	if (event_id == APP_TO_HOST_EVENTID) {
		wakeup = true;
		if (!is_fuel_gauge_data(*buf, ret)) {
			handle_fuelgauge_data(*buf, ret);
			release_wakeup(data);
			return;
		} else if (!is_hr_log_data(*buf, ret))
			io = &data->io[ID_NANOHUB_HR_LOG];
		else if (!is_custom_flash_data(*buf, ret))
			io = &data->io[ID_NANOHUB_CUSTOM_FLASH];
		else
			io = &data->io[ID_NANOHUB_COMMS];
	}

	nanohub_io_put_buf(io, *buf);

	*buf = NULL;
	/* (for wakeup interrupts): hold a wake lock for 250ms so the sensor hal
	 * has time to grab its own wake lock */
	if (wakeup)
		wake_lock_timeout(&data->wakelock_read, msecs_to_jiffies(250));
	release_wakeup(data);
}

static int nanohub_kthread(void *arg)
{
	struct nanohub_data *data = (struct nanohub_data *)arg;
	struct nanohub_buf *buf = NULL;
	int ret;
	ktime_t ktime_delta;
	uint32_t clear_interrupts[8] = { 0x00000006 };
	struct device *sensor_dev = data->io[ID_NANOHUB_SENSOR].dev;
	static const struct sched_param param = {
		.sched_priority = (MAX_USER_RT_PRIO/2)-1,
	};

	data->kthread_err_cnt = 0;
	sched_setscheduler(current, SCHED_FIFO, &param);
	nanohub_set_state(data, ST_RESET);

	while (!kthread_should_stop()) {
		switch (nanohub_get_state(data)) {
		case ST_RESET:
			nanohub_reset(data);
			nanohub_wakeup_unlock(data);
			nanohub_set_state(data, ST_IDLE);
			break;
		case ST_IDLE:
			wait_event_interruptible(data->kthread_wait,
						 atomic_read(&data->kthread_run)
						 );
			nanohub_set_state(data, ST_RUNNING);
			break;
		case ST_ERROR:
			ktime_delta = ktime_sub(ktime_get_boottime(),
						data->kthread_err_ktime);
			if (ktime_to_ns(ktime_delta) > KTHREAD_ERR_TIME_NS
				&& data->kthread_err_cnt > KTHREAD_ERR_CNT) {
				dev_info(sensor_dev,
					"kthread: hard reset due to consistent error\n");
				ret = nanohub_hw_reset(data);
				if (ret) {
					dev_info(sensor_dev,
						"%s: failed to reset nanohub: ret=%d\n",
						__func__, ret);
				}
				if (DOWNLOAD_BL_SUCCESS == atomic_read(&data->download_bl_status))
					atomic_set(&data->download_bl_status,
						DOWNLOAD_BL_FAILED);
			}
			msleep_interruptible(WAKEUP_TIMEOUT_MS);
			nanohub_set_state(data, ST_RUNNING);
			break;
		case ST_RUNNING:
			break;
		}
		atomic_set(&data->kthread_run, 0);
		if (!buf)
			buf = nanohub_io_get_buf(&data->free_pool,
						 false);
		if (buf) {
			ret = request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS);
			if (ret) {
				dev_info(sensor_dev,
					 "%s: request_wakeup_timeout: ret=%d\n",
					 __func__, ret);
				continue;
			}

			ret = nanohub_comms_rx_retrans_boottime(
			    data, CMD_COMMS_READ, buf->buffer,
			    sizeof(buf->buffer), 10, 10);

			if (ret > 0) {
				nanohub_process_buffer(data, &buf, ret);
				if (!nanohub_irq1_fired(data) &&
				    !nanohub_irq2_fired(data)) {
					nanohub_set_state(data, ST_IDLE);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
				if (NANOHUB_WAKEUP_TRACE_ON ==
					atomic_read(&data->st_wakeup_trace)) {
					atomic_set(&data->st_wakeup_trace,
						NANOHUB_WAKEUP_TRACE_OFF);
					nanohub_wakeup_trace_dump_list(
						&data->wakeup_trace,
						DUMP_TRACE_REASON_DUMP_QUEUE);
				}
#endif
					continue;
				}
			} else if (ret == 0) {
				/* queue empty, go to sleep */
				data->kthread_err_cnt = 0;
				data->interrupts[0] &= ~0x00000006;
				release_wakeup(data);
				nanohub_set_state(data, ST_IDLE);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
				if (NANOHUB_WAKEUP_TRACE_ON ==
					atomic_read(&data->st_wakeup_trace)) {

					atomic_set(&data->st_wakeup_trace,
						NANOHUB_WAKEUP_TRACE_OFF);
					nanohub_wakeup_trace_dump_list(
						&data->wakeup_trace,
						DUMP_TRACE_REASON_QUEUE_EMPTY);
				}
#endif
				continue;
			} else {
				release_wakeup(data);
				if (data->kthread_err_cnt == 0)
					data->kthread_err_ktime =
						ktime_get_boottime();

				data->kthread_err_cnt++;
				if (data->kthread_err_cnt >= KTHREAD_WARN_CNT) {
					dev_err(sensor_dev,
						"%s: kthread_err_cnt=%d, ret = %d\n",
						__func__,
						data->kthread_err_cnt, ret);
					nanohub_set_state(data, ST_ERROR);
					continue;
				}
			}
		} else {
			if (!nanohub_irq1_fired(data) &&
			    !nanohub_irq2_fired(data)) {
				nanohub_set_state(data, ST_IDLE);
#if (NANOHUB_WAKEUP_TRACE_ENABLE)
				if (NANOHUB_WAKEUP_TRACE_ON ==
					atomic_read(&data->st_wakeup_trace)) {

					atomic_set(&data->st_wakeup_trace,
						NANOHUB_WAKEUP_TRACE_OFF);
					nanohub_wakeup_trace_dump_list(
						&data->wakeup_trace,
						DUMP_TRACE_REASON_NO_BUFFER);
				}
#endif
				continue;
			}
			/* pending interrupt, but no room to read data -
			 * clear interrupts */
			if (request_wakeup(data))
				continue;

			nanohub_comms_tx_rx_retrans(data,
						    CMD_COMMS_CLR_GET_INTR,
						    (uint8_t *)
						    clear_interrupts,
						    sizeof(clear_interrupts),
						    (uint8_t *) data->
						    interrupts,
						    sizeof(data->interrupts),
						    false, 10, 0);
			release_wakeup(data);
			nanohub_set_state(data, ST_IDLE);
#if (NANOHUB_WAKEUP_TRACE_ENABLE)
				if (NANOHUB_WAKEUP_TRACE_ON ==
					atomic_read(&data->st_wakeup_trace)) {

					atomic_set(&data->st_wakeup_trace,
						NANOHUB_WAKEUP_TRACE_OFF);
					nanohub_wakeup_trace_dump_list(
						&data->wakeup_trace,
						DUMP_TRACE_REASON_CLEAR_IRQS);
				}
#endif
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static struct nanohub_platform_data *nanohub_parse_dt(struct device *dev)
{
	struct nanohub_platform_data *pdata;
	struct device_node *dt = dev->of_node;
	const uint32_t *tmp;
	struct property *prop;
	uint32_t u, i;
	int ret;

	if (!dt)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = pdata->irq1_gpio =
	    of_get_named_gpio(dt, "sensorhub,irq1-gpio", 0);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,irq1-gpio in device tree\n");
		goto free_pdata;
	}

	/* optional (strongly recommended) */
	pdata->irq2_gpio = of_get_named_gpio(dt, "sensorhub,irq2-gpio", 0);

	pdata->irq3_gpio = of_get_named_gpio(dt, "sensorhub,irq3-gpio", 0);

	ret = pdata->wakeup_gpio =
	    of_get_named_gpio(dt, "sensorhub,wakeup-gpio", 0);
	if (ret < 0) {
		pr_err
		    ("nanohub: missing sensorhub,wakeup-gpio in device tree\n");
		goto free_pdata;
	}

	ret = pdata->nreset_gpio =
	    of_get_named_gpio(dt, "sensorhub,nreset-gpio", 0);
	if (ret < 0) {
		pr_err
		    ("nanohub: missing sensorhub,nreset-gpio in device tree\n");
		goto free_pdata;
	}

	ret = pdata->nreset_polarity =
		of_property_read_bool(dt, "sensorhub,nreset_active_high");
	if (ret) {
		pdata->nreset_polarity = NRESET_ACTIVE_HIGH;
		pr_info("nanohub: nreset_active_high, %d\n",
			pdata->nreset_polarity);
	} else {
		pdata->nreset_polarity = NRESET_ACTIVE_LOW;
		pr_info("nanohub: nreset_active_low, %d\n",
			pdata->nreset_polarity);
	}

	/* optional (stm32f bootloader) */
	pdata->boot0_gpio = of_get_named_gpio(dt, "sensorhub,boot0-gpio", 0);

	/* optional (stm32f mode) */
	pdata->int_gpio   =
		of_get_named_gpio(dt, "sensorhub,hub-flag-int-gpio", 0);
	pdata->mode1_gpio =
		of_get_named_gpio(dt, "sensorhub,mode-flag1-gpio", 0);
	pdata->mode2_gpio =
		of_get_named_gpio(dt, "sensorhub,mode-flag2-gpio", 0);
	pdata->mode3_gpio =
		of_get_named_gpio(dt, "sensorhub,mode-flag3-gpio", 0);
	pdata->mode4_gpio =
		of_get_named_gpio(dt, "sensorhub,mode-flag4-gpio", 0);
	/* optional (spi) */
	pdata->spi_cs_gpio =
		of_get_named_gpio(dt, "sensorhub,spi-cs-gpio", 0);

	/* custom flash memory define */
	of_property_read_u32(dt, "sensorhub,custom-flash-addr",
		&pdata->custom_flash_addr);
	of_property_read_u32(dt, "sensorhub,custom-flash-len",
		&pdata->custom_flash_len);

	/* optional (stm32f bootloader) */
	of_property_read_u32(dt, "sensorhub,bl-addr", &pdata->bl_addr);

	/* optional (stm32f bootloader) */
	tmp = of_get_property(dt, "sensorhub,num-flash-banks", NULL);
	if (tmp) {
		pdata->num_flash_banks = be32_to_cpup(tmp);
		pdata->flash_banks =
		    devm_kzalloc(dev,
				 sizeof(struct nanohub_flash_bank) *
				 pdata->num_flash_banks, GFP_KERNEL);
		if (!pdata->flash_banks)
			goto no_mem;

		/* TODO: investigate replacing with of_property_read_u32_array
		 */
		i = 0;
		of_property_for_each_u32(dt, "sensorhub,flash-banks", prop, tmp,
					 u) {
			if (i / 3 >= pdata->num_flash_banks)
				break;
			switch (i % 3) {
			case 0:
				pdata->flash_banks[i / 3].bank = u;
				break;
			case 1:
				pdata->flash_banks[i / 3].address = u;
				break;
			case 2:
				pdata->flash_banks[i / 3].length = u;
				break;
			}
			i++;
		}
	}

	/* optional (stm32f bootloader) */
	tmp = of_get_property(dt,
			"sensorhub,num-shared-flash-banks", NULL);
	if (tmp) {
		pdata->num_shared_flash_banks = be32_to_cpup(tmp);
		pdata->shared_flash_banks =
		    devm_kzalloc(dev,
				 sizeof(struct nanohub_flash_bank) *
				 pdata->num_shared_flash_banks, GFP_KERNEL);
		if (!pdata->shared_flash_banks)
			goto no_mem_shared;

		/* TODO: investigate replacing with of_property_read_u32_array
		 */
		i = 0;
		of_property_for_each_u32(dt, "sensorhub,shared-flash-banks",
					 prop, tmp, u) {
			if (i / 3 >= pdata->num_shared_flash_banks)
				break;
			switch (i % 3) {
			case 0:
				pdata->shared_flash_banks[i / 3].bank = u;
				break;
			case 1:
				pdata->shared_flash_banks[i / 3].address = u;
				break;
			case 2:
				pdata->shared_flash_banks[i / 3].length = u;
				break;
			}
			i++;
		}
	}

	return pdata;

no_mem_shared:
	devm_kfree(dev, pdata->flash_banks);
no_mem:
	ret = -ENOMEM;
free_pdata:
	devm_kfree(dev, pdata);
	return ERR_PTR(ret);
}
#else
static struct nanohub_platform_data *nanohub_parse_dt(struct device *dev)
{
	struct nanohub_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	return pdata;
}
#endif

static int nanohub_request_irqs(struct nanohub_data *data)
{
	int ret;

	ret = request_threaded_irq(data->irq1, NULL, nanohub_irq1,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "nanohub-irq1", data);
	if (ret < 0)
		data->irq1 = 0;
	else
		disable_irq(data->irq1);

	ret = request_threaded_irq(data->irq3, NULL, nanohub_irq3,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "nanohub-irq3", data);
	if (ret < 0) {
		data->irq3 = 0;
		WARN(1, "failed to request optional IRQ %d; err=%d",
		     data->irq3, ret);
	} else {
		disable_irq(data->irq3);
	}

	if (data->irq2 <= 0 || ret < 0) {
		data->irq2 = 0;
		return ret;
	}

	ret = request_threaded_irq(data->irq2, NULL, nanohub_irq2,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "nanohub-irq2", data);
	if (ret < 0) {
		data->irq2 = 0;
		WARN(1, "failed to request optional IRQ %d; err=%d",
		     data->irq2, ret);
	} else {
		disable_irq(data->irq2);
	}
	/* if 2d request fails, hide this; it is optional IRQ,
	 * and failure should not interrupt driver init sequence.
	 */
	return 0;
}

static int nanohub_request_gpios(struct nanohub_data *data)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(gconf); ++i) {
		const struct gpio_config *cfg = &gconf[i];
		unsigned int gpio = plat_gpio_get(data, cfg);
		const char *label;
		bool optional = gpio_is_optional(cfg);

		ret = 0; /* clear errors on optional pins, if any */

		if (!gpio_is_valid(gpio) && optional)
			continue;

		label = cfg->label;
		ret = gpio_request_one(gpio, cfg->flags, label);
		if (ret && !optional) {
			pr_err("nanohub: gpio %d[%s] request failed;err=%d\n",
			       gpio, label, ret);
			break;
		}
		if (gpio_has_irq(cfg)) {
			int irq = gpio_to_irq(gpio);

			if (irq > 0) {
				nanohub_set_irq_data(data, cfg, irq);
			} else if (!optional) {
				ret = -EINVAL;
				pr_err("nanohub: no irq; gpio %d[%s];err=%d\n",
				       gpio, label, irq);
				break;
			}
		}
	}
	if (i < ARRAY_SIZE(gconf)) {
		for (--i; i >= 0; --i)
			gpio_free(plat_gpio_get(data, &gconf[i]));
	}

	return ret;
}

static void nanohub_release_gpios_irqs(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	if (data->irq3)
		free_irq(data->irq3, data);
	if (data->irq2)
		free_irq(data->irq2, data);
	if (data->irq1)
		free_irq(data->irq1, data);
	if (gpio_is_valid(pdata->irq3_gpio))
		gpio_free(pdata->irq3_gpio);
	if (gpio_is_valid(pdata->irq2_gpio))
		gpio_free(pdata->irq2_gpio);
	gpio_free(pdata->irq1_gpio);
	if (pdata->nreset_polarity == NRESET_ACTIVE_LOW)
		gpio_set_value(pdata->nreset_gpio, 0);
	else
		gpio_set_value(pdata->nreset_gpio, 1);

	gpio_free(pdata->nreset_gpio);
	mcu_wakeup_gpio_set_value(data, 1);
	gpio_free(pdata->wakeup_gpio);
	gpio_set_value(pdata->boot0_gpio, 0);
	gpio_free(pdata->boot0_gpio);

	gpio_set_value(pdata->int_gpio, 0);
	gpio_free(pdata->int_gpio);
	gpio_set_value(pdata->mode1_gpio, 0);
	gpio_free(pdata->mode1_gpio);
	gpio_set_value(pdata->mode2_gpio, 0);
	gpio_free(pdata->mode2_gpio);
	gpio_set_value(pdata->mode3_gpio, 0);
	gpio_free(pdata->mode3_gpio);
	gpio_set_value(pdata->mode4_gpio, 0);
	gpio_free(pdata->mode4_gpio);
}

struct iio_dev *nanohub_probe(struct device *dev, struct iio_dev *iio_dev)
{
	int ret, i;
	const struct nanohub_platform_data *pdata;
	struct nanohub_data *data;
	struct nanohub_buf *buf;
	bool own_iio_dev = !iio_dev;

	pr_info("nanohub: start to probe nanohub\n");

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = nanohub_parse_dt(dev);
		if (IS_ERR(pdata))
			return ERR_PTR(PTR_ERR(pdata));
	}

	if (own_iio_dev) {
		iio_dev = iio_device_alloc(sizeof(struct nanohub_data));
		if (!iio_dev)
			return ERR_PTR(-ENOMEM);
	}

	iio_dev->name = "nanohub";
	iio_dev->dev.parent = dev;
	iio_dev->info = &nanohub_iio_info;
	iio_dev->channels = NULL;
	iio_dev->num_channels = 0;

	data = iio_priv(iio_dev);
	data->iio_dev = iio_dev;
	data->pdata = pdata;

	mutex_init(&(data->hub_mode_set_lock));
	mutex_init(&(data->nanohub_write_lock));
	init_waitqueue_head(&data->kthread_wait);

	nanohub_io_init(&data->free_pool, data, dev);

	buf = vmalloc(sizeof(*buf) * READ_QUEUE_DEPTH);
	data->vbuf = buf;
	if (!buf) {
		ret = -ENOMEM;
		goto fail_vma;
	}

	for (i = 0; i < READ_QUEUE_DEPTH; i++)
		nanohub_io_put_buf(&data->free_pool, &buf[i]);
	atomic_set(&data->kthread_run, 0);
	wake_lock_init(&data->wakelock_read, WAKE_LOCK_SUSPEND,
		       "nanohub_wakelock_read");

	/* hold lock until reset completes */
	atomic_set(&data->lock_mode, LOCK_MODE_RESET);
	atomic_set(&data->wakeup_cnt, 0);
	atomic_set(&data->wakeup_lock_cnt, 1);
	atomic_set(&data->wakeup_acquired, KEY_WAKEUP_LOCK);
	atomic_set(&data->download_bl_status, DOWNLOAD_BL_NOT_START);
	atomic_set(&data->hub_mode_ap_active, GPIO_CMD_NORMAL);
	atomic_set(&data->hub_mode_ap_pwr_down, GPIO_CMD_POWEROFF);
	atomic_set(&data->lcd_mutex, LCD_MUTEX_OFF);
	atomic_set(&data->sensor_hal_alive, SENSOR_HAL_DEAD);
	init_waitqueue_head(&data->wakeup_wait);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
	INIT_LIST_HEAD(&data->wakeup_trace.event_list);
#endif

	ret = nanohub_request_gpios(data);
	if (ret)
		goto fail_gpio;

	ret = nanohub_request_irqs(data);
	if (ret)
		goto fail_irq;

	ret = iio_device_register(iio_dev);
	if (ret) {
		pr_err("nanohub: iio_device_register failed\n");
		goto fail_irq;
	}

	ret = nanohub_create_devices(data);
	if (ret)
		goto fail_dev;

	bq27x00_powersupply_init(dev, data);

	__nanohub_send_AP_cmd(data, GPIO_CMD_NORMAL);

	if (data->irq3)
		enable_irq(data->irq3);

	data->thread = kthread_create(nanohub_kthread, data, "nanohub");

	if (!IS_ERR(data->thread)) {
		pr_info("nanohub: nanohub_probe end successfully\n");
		return iio_dev;
	}

fail_dev:
	iio_device_unregister(iio_dev);
fail_irq:
	nanohub_release_gpios_irqs(data);
fail_gpio:
	wake_lock_destroy(&data->wakelock_read);
	vfree(buf);
fail_vma:
	if (own_iio_dev)
		iio_device_free(iio_dev);
	pr_err("nanohub: nanohub_probe fail\n");

	return ERR_PTR(ret);
}

void nanohub_start(struct nanohub_data *data)
{
	wake_up_process(data->thread);
}

int nanohub_reset(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	if (pdata->nreset_polarity == NRESET_ACTIVE_LOW) {
		gpio_set_value(pdata->nreset_gpio, 1);
	} else {
		gpio_set_value(pdata->nreset_gpio, 1);
		usleep_range(30, 40);
		gpio_set_value(pdata->nreset_gpio, 0);
	}
	usleep_range(650000, 700000);
	__nanohub_send_AP_cmd(data, GPIO_CMD_RESEND);
	return 0;
}

int nanohub_remove(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);

	bq27x00_powersupply_unregister();

	nanohub_notify_thread(data);
	kthread_stop(data->thread);

	__nanohub_send_AP_cmd(data, GPIO_CMD_POWEROFF);

	nanohub_destroy_devices(data);
	iio_device_unregister(iio_dev);
	nanohub_release_gpios_irqs(data);
	wake_lock_destroy(&data->wakelock_read);
	vfree(data->vbuf);
	iio_device_free(iio_dev);

	return 0;
}

int nanohub_shutdown(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);
	if (GPIO_CMD_BAND != atomic_read(&data->hub_mode_ap_pwr_down))
		__nanohub_set_mode_pin(data, GPIO_CMD_POWEROFF);
	else
		__nanohub_set_mode_pin(data, GPIO_CMD_BAND);
	return 0;
}

int nanohub_suspend(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);
	int ret;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_SUSPEND_RESUME);
	if (!ret) {
		int cnt;
		const int max_cnt = 10;

		for (cnt = 0; cnt < max_cnt; ++cnt) {
			if (!nanohub_irq1_fired(data))
				break;
			usleep_range(10, 15);
		}
		if (cnt < max_cnt) {
			dev_dbg(&iio_dev->dev, "nanohub: %s: cnt=%d\n",
				__func__, cnt);
			enable_irq_wake(data->irq1);
			enable_irq_wake(data->irq3);
			__nanohub_send_AP_cmd(data, GPIO_CMD_SUSPEND);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
			atomic_set(&data->suspend_status,
					NANOHUB_SUSPEND_ENTRY);
			atomic_set(&data->st_wakeup_trace,
					NANOHUB_WAKEUP_TRACE_ON);
			memset(&data->wakeup_trace_irqs, 0,
					sizeof(struct irq_nums_during_wakeup));
#endif
			return 0;
		}
		ret = -EBUSY;
		dev_info(&iio_dev->dev,
			 "nanohub: %s: failed to suspend: IRQ1=%d, state=%d\n",
			 __func__, nanohub_irq1_fired(data),
			 nanohub_get_state(data));
		nanohub_wakeup_unlock(data);
	} else {
		dev_info(&iio_dev->dev, "nanohub: %s: could not take wakeup lock\n",
			 __func__);
	}

	return ret;
}

int nanohub_resume(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);

	__nanohub_send_AP_cmd(data, GPIO_CMD_RESUME);

#if (NANOHUB_WAKEUP_TRACE_ENABLE)
	atomic_set(&data->suspend_status, NANOHUB_SUSPEND_EXIT);
#endif

	disable_irq_wake(data->irq1);
	disable_irq_wake(data->irq3);
	nanohub_wakeup_unlock(data);

	return 0;
}

static int __init nanohub_init(void)
{
	int ret = 0;

	sensor_class = class_create(THIS_MODULE, "nanohub");
	if (IS_ERR(sensor_class)) {
		ret = PTR_ERR(sensor_class);
		pr_err("nanohub: class_create failed; err=%d\n", ret);
	}
	if (!ret)
		major = __register_chrdev(0, 0, ID_NANOHUB_MAX, "nanohub",
					  &nanohub_fileops);

	if (major < 0) {
		ret = major;
		major = 0;
		pr_err("nanohub: can't register; err=%d\n", ret);
	}

#ifdef CONFIG_NANOHUB_I2C
	if (ret == 0)
		ret = nanohub_i2c_init();
#endif
#ifdef CONFIG_NANOHUB_SPI
	if (ret == 0)
		ret = nanohub_spi_init();
#endif
	pr_info("nanohub: loaded; ret=%d\n", ret);
	return ret;

}

static void __exit nanohub_cleanup(void)
{

#ifdef CONFIG_NANOHUB_I2C
	nanohub_i2c_cleanup();
#endif
#ifdef CONFIG_NANOHUB_SPI
	nanohub_spi_cleanup();
#endif
	__unregister_chrdev(major, 0, ID_NANOHUB_MAX, "nanohub");
	class_destroy(sensor_class);
	major = 0;
	sensor_class = 0;
}

module_init(nanohub_init);
module_exit(nanohub_cleanup);

MODULE_AUTHOR("Ben Fennema");
MODULE_LICENSE("GPL");

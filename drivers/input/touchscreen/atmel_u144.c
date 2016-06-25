/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2012 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 * Copyright (C) 2013-2016 LG Electronics, Inc.
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#include <linux/file.h>
#include <linux/irq.h>

#include "atmel_u144.h"
#include "atmel_u144_patch.h"

static struct mutex i2c_suspend_lock;
static struct mutex irq_lock;

static bool selftest_enable;
static bool selftest_show;
static struct wake_lock touch_wake_lock;
static struct wake_lock pm_touch_wake_lock;
static struct mxt_data *global_mxt_data;
static struct workqueue_struct	*touch_wq = NULL;

static bool touch_irq_mask = 1;
static bool touch_irq_wake_mask = 0;
static unsigned char touched_finger_count = 0;
static unsigned char patchevent_mask = 0;
static unsigned char power_block_mask = 0;

static struct bus_type touch_subsys = {
	 .name = MXT_TOUCH_NAME,
	 .dev_name = "mxt_touch",
};

static struct device device_touch = {
	.id = 0,
	.bus = &touch_subsys,
};

struct mxt_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct mxt_data *ts, char *buf);
	ssize_t (*store)(struct mxt_data *ts, const char *buf, size_t count);
};

#define MXT_TOUCH_ATTR(_name, _mode, _show, _store) \
	struct mxt_touch_attribute mxt_touch_attr_##_name = \
		__ATTR(_name, _mode, _show, _store)

#define jitter_abs(x)		(x > 0 ? x : -x)
#define jitter_sub(x, y)	(x > y ? x - y : y - x)
#define get_time_interval(a,b)	a>=b ? a-b : 1000000+a-b

static int g_usb_type = 0;

static int mxt_soft_reset(struct mxt_data *data);
static int mxt_hw_reset(struct mxt_data *data);
static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset, u8 value, bool wait);
static int mxt_t109_command(struct mxt_data *data, u16 cmd_offset, u8 value);
static void mxt_reset_slots(struct mxt_data *data);
static void mxt_start(struct mxt_data *data);
static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep);
static void mxt_regulator_enable(struct mxt_data *data);
static void mxt_stop(struct mxt_data *data);
static int mxt_read_config_crc(struct mxt_data *data, u32 *crc);
static int mxt_command_backup(struct mxt_data *data, u8 value);
static int mxt_command_reset(struct mxt_data *data, u8 value);
static void mxt_regulator_disable(struct mxt_data *data);
static void mxt_regulator_enable(struct mxt_data *data);
void trigger_usb_state_from_otg(int usb_type);
static void mxt_read_fw_version(struct mxt_data *data);
static int mxt_read_t100_config(struct mxt_data *data);

char *knockon_event[2] = { "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL };

static int touch_enable_irq_wake(unsigned int irq)
{
	int ret = 0;

	mutex_lock(&irq_lock);
	if (!touch_irq_wake_mask) {
		touch_irq_wake_mask = 1;
		ret = enable_irq_wake(irq);
		if (ret)
			TOUCH_ERR_MSG("%s : %d \n", __func__, ret);
		else
			TOUCH_DEBUG_MSG("%s : %d \n", __func__, ret);
	}
	mutex_unlock(&irq_lock);
	return ret;
}

static int touch_disable_irq_wake(unsigned int irq)
{
	int ret = 0;

	mutex_lock(&irq_lock);
	if (touch_irq_wake_mask) {
		touch_irq_wake_mask = 0;
		ret = disable_irq_wake(irq);
		if (ret)
			TOUCH_ERR_MSG("%s : %d \n", __func__, ret);
		else
			TOUCH_DEBUG_MSG("%s : %d \n", __func__, ret);
	}
	mutex_unlock(&irq_lock);
	return ret;
}

static void touch_enable_irq(unsigned int irq)
{
	mutex_lock(&irq_lock);
	if (!touch_irq_mask) {
		touch_irq_mask = 1;
		enable_irq(irq);
		TOUCH_DEBUG_MSG("%s()\n", __func__);
	}
	mutex_unlock(&irq_lock);
}

static void touch_disable_irq(unsigned int irq)
{
	mutex_lock(&irq_lock);
	if (touch_irq_mask) {
		touch_irq_mask = 0;
		disable_irq_nosync(irq);
		TOUCH_DEBUG_MSG("%s()\n", __func__);
	}
	mutex_unlock(&irq_lock);
}

static char mxt_power_block_get(void)
{
	return power_block_mask;
}

static void mxt_power_block(char value)
{
	power_block_mask |= value;
}

static void mxt_power_unblock(char value)
{
	power_block_mask &= ~(value);
}

static char mxt_patchevent_get(char value)
{
	return patchevent_mask & value;
}

static void mxt_patchevent_set(char value)
{
	patchevent_mask |= value;
}

static void mxt_patchevent_unset(char value)
{
	patchevent_mask &= ~(value);
}

static inline u16 mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static inline unsigned int mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
		case MXT_GEN_COMMAND_T6:
		case MXT_GEN_POWER_T7:
		case MXT_GEN_ACQUIRE_T8:
		case MXT_SPT_COMMSCONFIG_T18:
		case MXT_TOUCH_PROXIMITY_T23:
		case MXT_PROCI_ONETOUCH_T24:
		case MXT_SPT_SELFTEST_T25:
		case MXT_SPT_USERDATA_T38:
		case MXT_PROCI_GRIPSUPPRESSION_T40:
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
		case MXT_SPT_CTECONFIG_T46:
		case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
		case MXT_PROCI_SHIELDLESS_T56:
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	print_hex_dump(KERN_ERR, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
		       message, data->T5_msg_size, false);
}

static int mxt_wait_for_completion(struct mxt_data *data,
			struct completion *comp, unsigned int timeout_ms)
{
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret = 0;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		TOUCH_ERR_MSG("Wait for completion interrupted.\n");
		return -EINTR;
	} else if (ret == 0) {
		TOUCH_ERR_MSG("Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data, u8 *val,
		unsigned int count)
{
	int ret = 0;
	struct i2c_msg msg = {0};

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		TOUCH_ERR_MSG("i2c recv failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data, const u8 * const val,
		unsigned int count)
{
	int ret = 0;
	struct i2c_msg msg = {0};

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		TOUCH_ERR_MSG("i2c send failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, u8 retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader = 0;
	u8 family_id = 0;

	if (data->info)
		family_id = data->info->family_id;

	TOUCH_DEBUG_MSG("%s appmode=0x%x\n", __func__, appmode);

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if ((retry % 2) || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		TOUCH_ERR_MSG("Appmode i2c address 0x%02x not found\n",
				appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	TOUCH_DEBUG_MSG("%s bootloader_addr=0x%x\n", __func__, bootloader);
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, u8 retry)
{
	int ret = 0;
	u8 val = 0;
	bool crc_failure = false;

	TOUCH_DEBUG_MSG("%s\n", __func__);

	ret = mxt_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	TOUCH_DEBUG_MSG("Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	u8 buf[3] = {0};

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			TOUCH_ERR_MSG("%s: i2c failure\n", __func__);
			return -EIO;
		}

		TOUCH_DEBUG_MSG("Bootloader ID:%d Version:%d\n",
				buf[1], buf[2]);

		return buf[0];
	}

	TOUCH_DEBUG_MSG("Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

	return val;
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state)
{
	u8 val = 0;
	int ret = 0;

recheck:
	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			TOUCH_ERR_MSG("Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		TOUCH_ERR_MSG("Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret = 0;
	u8 buf[2] = {0};

	TOUCH_DEBUG_MSG("%s : %d\n", __func__, unlock);

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client, u16 reg, u16 len,
		void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2] = {0};
	int i = 0;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	do {
		if (i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer))==2)
			return 0;
		TOUCH_ERR_MSG("%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 10);

	TOUCH_ERR_MSG("%s: i2c transfer failed\n", __func__);
	return -EIO;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
		const void *val)
{
	u8 data[288] = {0};
	u8 *buf = NULL;
	int count = 0;
	int i = 0;
	bool alloced = false;
	int ret = 0;

	count = len + 2;

	if (unlikely(count > 288)) {
		buf = kzalloc(count, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
		alloced = true;
	} else {
		buf = data;
	}

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);
	do {
		if (i2c_master_send(client, buf, count)==count) {
			ret = 0;
			goto out;
		}
		TOUCH_ERR_MSG("%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 10);
	TOUCH_ERR_MSG("%s: i2c transfer failed\n", __func__);
	ret = -EIO;

out:
	if (unlikely(alloced))
		kfree(buf);

	return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

struct mxt_object *mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object = NULL;
	int i = 0;

	if (!data->info || !data->object_table)
		return NULL;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	TOUCH_ERR_MSG("Invalid object type T%u\n", type);
	return NULL;
}

int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object = NULL;
	int error = 0;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(data->client,
			object->start_address + offset, 1, val);
	if (error)
		TOUCH_ERR_MSG("Error to read T[%d] offset[%d] val[%d]\n",
				type, offset, *val);

	return error;
}

int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, u8 val)
{
	struct mxt_object *object = NULL;
	int error = 0;
	u16 reg = 0;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	error = __mxt_write_reg(data->client, reg + offset, 1, &val);
	if (error) {
		TOUCH_ERR_MSG("Error to write T[%d] offset[%d] val[%d]\n",
				type, offset, val);
		mxt_hw_reset(data);
	}
	return error;
}

static int mxt_set_diagnostic_mode(struct mxt_data *data, u8 dbg_mode)
{
	u8 cur_mode = 0;
	int ret = 0;
	int retry_cnt = 0;

	ret = mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_DIAGNOSTIC, dbg_mode);

	if (ret) {
		TOUCH_ERR_MSG("Failed change diagnositc mode to %d\n",
			 dbg_mode);
		goto out;
	}

	if (dbg_mode & MXT_DIAG_MODE_MASK) {
		do {
			ret = mxt_read_object(data, MXT_DEBUG_DIAGNOSTIC_T37,
				MXT_DIAGNOSTIC_MODE, &cur_mode);
			if (ret || retry_cnt++ >= 4) {
				TOUCH_ERR_MSG(
					"Failed getting diagnositc mode(%d)\n",
					retry_cnt);
				goto out;
			}
			msleep(20);
		} while (cur_mode != dbg_mode);

		TOUCH_ERR_MSG("current dianostic chip mode is %d\n", cur_mode);
	}

out:
	return ret;
}

int mxt_get_self_reference_chk(struct mxt_data *data)
{
	u8 cur_page = 0;
	u8 read_page = 0;
	struct mxt_object *dbg_object = NULL;
	struct mxt_object *object = NULL;

	int ret = 0;
	int i = 0;
	u8 ret_buf[NODE_PER_PAGE * DATA_PER_NODE] = {0};
	u8 comms_chk[2] = {0};
	u8 loop_chk = 0;
	u8 self_chk_thr = 0;

	s16 curr_ref;

	//bool self_tune = false;
	u8 err_cnt = 0;

	ret = mxt_t6_command(data, MXT_COMMAND_DIAGNOSTIC,
			MXT_COMMAND_SELF_REFERENCE, false);
	if (ret) {
		return 0;
	}

	object = mxt_get_object(global_mxt_data, MXT_SPT_USERDATA_T38);
	if (!object) {
		TOUCH_ERR_MSG("Failed to get object\n");
		return 1;
	}

	ret = __mxt_read_reg(data->client,
			object->start_address + 6, 1, &self_chk_thr);

	dbg_object = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!dbg_object) {
		TOUCH_ERR_MSG("Failed to get object_info\n");
		return 1;
	}

	for (i=0; i < 7; i++) 	{
		msleep(20);
		__mxt_read_reg(data->client,
				dbg_object->start_address, 2, comms_chk);

		if (comms_chk[0] == MXT_COMMAND_SELF_REFERENCE &&
		    comms_chk[1] == 0) {
			TOUCH_DEBUG_MSG("%s Enter success in Self Reference"
					" mode\n", __func__);
			break;
		} else if (i == 6) {
			TOUCH_DEBUG_MSG("%s Enter fail in Self Reference"
					" mode\n", __func__);
			return 0; // Don't check self Reference no more!!
		}
	}

	for (read_page = 0; read_page < 7; read_page++) {
		__mxt_read_reg(data->client, dbg_object->start_address + 2,
				NODE_PER_PAGE * DATA_PER_NODE, ret_buf);

		TOUCH_DEBUG_MSG("CURR SELF REFERENCE read page %u :",
				read_page);
		if (read_page == 0 || read_page == 1 || read_page == 6) {
			for (i = 0; i < 6; i++) {
				TOUCH_DEBUG_MSG(
					"%6hd %6hd %6hd %6hd %6hd"
					" %6hd %6hd %6hd %6hd %6hd",
					( ((u16)ret_buf[(i*10)*2+1] << 8) +
					  (u16)ret_buf[(i*10)*2] ),
					( ((u16)ret_buf[(i*10+1)*2+1] << 8) +
					  (u16)ret_buf[(i*10+1)*2] ),
					( ((u16)ret_buf[(i*10+2)*2+1] << 8) +
					  (u16)ret_buf[(i*10+2)*2] ),
					( ((u16)ret_buf[(i*10+3)*2+1] << 8) +
					  (u16)ret_buf[(i*10+3)*2] ),
					( ((u16)ret_buf[(i*10+4)*2+1] << 8) +
					  (u16)ret_buf[(i*10+4)*2] ),
					( ((u16)ret_buf[(i*10+5)*2+1] << 8) +
					  (u16)ret_buf[(i*10+5)*2] ),
					( ((u16)ret_buf[(i*10+6)*2+1] << 8) +
					  (u16)ret_buf[(i*10+6)*2] ),
					( ((u16)ret_buf[(i*10+7)*2+1] << 8) +
					  (u16)ret_buf[(i*10+7)*2] ),
					( ((u16)ret_buf[(i*10+8)*2+1] << 8) +
					  (u16)ret_buf[(i*10+8)*2] ),
					( ((u16)ret_buf[(i*10+9)*2+1] << 8) +
					  (u16)ret_buf[(i*10+9)*2] ));
			}
		}

		for (i = 0; i < NODE_PER_PAGE; i++) {
			curr_ref =( ((u16)ret_buf[i*2+1] << 8) + (u16)ret_buf[i*2] );

			if (read_page == 0 && i > 40) {
				/* Self Hover */
				break;
			} else if (read_page == 1 && i < 8) {
				continue;
			} else if (read_page == 1 && i > 33) {
			    break;
			} else if (read_page > 1 && read_page < 6) {
			    break;
			} else if (read_page == 6 && i > 35) {
			    break;
			} else {
				/* Self Touch & Proc reference check */
				if ( curr_ref > self_chk_thr * 500) {
					break;
				} else {
					/* Need to Self-Tune. */
					TOUCH_DEBUG_MSG(
						"CURR SELF REFERENCE Error"
						" page %u, numger %d :",
						read_page, i);
					err_cnt++;
				}
			}
		}

		ret = mxt_set_diagnostic_mode(data, MXT_DIAG_PAGE_UP);
		if (ret) {
			TOUCH_ERR_MSG("Failed to set self reference mode!\n");
			/* Don't check self reference no more!! */
			return 0;
		}

		loop_chk = 0;
		do {
			msleep(20);
			ret = __mxt_read_reg(data->client,
				dbg_object->start_address +
				MXT_DIAGNOSTIC_PAGE, 1, &cur_page);
			if (ret  || loop_chk++ >= 4) {
				TOUCH_ERR_MSG("%s Read fail page(%d)\n",
						__func__, loop_chk);
				/* Don't check self reference no more!! */
				return 0;
			}
		} while (cur_page != read_page + 1);
	}

	TOUCH_ERR_MSG("Reference Error Count: %d\n", err_cnt);

	if (err_cnt > 0) {
		TOUCH_ERR_MSG("Need to Self Cap Re tune!\n");

		if (object) {
		    mxt_write_reg(global_mxt_data->client,
				    object->start_address + 2, err_cnt);
		}

		return 1;
	} else {
		mxt_write_object(data, MXT_SPT_USERDATA_T38, 1, 1);

		/* Backup to memory */
		ret = mxt_command_backup(data, MXT_BACKUP_VALUE);
		if (ret) {
			TOUCH_ERR_MSG("Failed backup NV data\n");
		}
	}

	TOUCH_DEBUG_MSG("Self Reference Check Success!n");
	return 0;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);
	struct mxt_object *object = NULL;
	u8 stylus_in_a_row_limit[6] = {0};
	int ret = 0;

	if (crc != data->config_crc) {
		data->config_crc = crc;
		TOUCH_INFO_MSG("T6 Config Checksum: 0x%06X\n", crc);
		complete(&data->crc_completion);
	}

	/* Detect transition out of reset */
	if ((data->t6_status & MXT_T6_STATUS_RESET) &&
	    !(status & MXT_T6_STATUS_RESET))
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		TOUCH_INFO_MSG("T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			(status == 0) ? " OK" : "",
			(status & MXT_T6_STATUS_RESET) ? " RESET" : "",
			(status & MXT_T6_STATUS_OFL) ? " OFL" : "",
			(status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
			(status & MXT_T6_STATUS_CAL) ? " CAL" : "",
			(status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
			(status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;

	if (status & MXT_T6_STATUS_CAL || status & MXT_T6_STATUS_RESET) {
		mxt_reset_slots(data);
		if (data->delayed_cal) {
			TOUCH_DEBUG_MSG("delayed calibration call\n");
			queue_delayed_work(touch_wq,
					&data->work_delay_cal,
					msecs_to_jiffies(200));
			data->delayed_cal = false;
		}
		data->patch.src_item[MXT_PATCH_ITEM_USER4] = 0;
		object = mxt_get_object(data, MXT_SPT_USERDATA_T38);

		if(!object) {
			TOUCH_INFO_MSG("fail to get object\n");
			return;
		}

		ret = mxt_read_mem(data, object->start_address+2, 6,
				stylus_in_a_row_limit);

		if (ret)
			TOUCH_INFO_MSG("T38 stylus_in_a_row_limit read fail\n");

		data->stylus_in_a_row_cnt_thr = stylus_in_a_row_limit[0] << 8 |
			stylus_in_a_row_limit[1];
		data->x_zitter = stylus_in_a_row_limit[2] << 8 |
			stylus_in_a_row_limit[3];
		data->y_zitter = stylus_in_a_row_limit[4] << 8 |
			stylus_in_a_row_limit[5];
		TOUCH_INFO_MSG("Set Stylus limit thr %d x_zitter %d y_zitter %d \n",
				data->stylus_in_a_row_cnt_thr, data->x_zitter, data->y_zitter);
	}

	/* Set KnockCode Delay after RESET */
	if (!data->mfts_enable) {
		if (status & MXT_T6_STATUS_RESET && data->is_knockCodeDelay) {
			mxt_write_object(data,
				MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 19, 43);
			TOUCH_DEBUG_MSG("Set Knock Code delay after RESET (700ms)\n");
		} else if (status & MXT_T6_STATUS_RESET &&
				!data->is_knockCodeDelay) {
			mxt_write_object(data,
				MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 19, 0);
			TOUCH_DEBUG_MSG("Set Knock Code delay after RESET (0ms)\n");
		}
	}

	if (status & MXT_T6_STATUS_RESET && data->suspended) {
		TOUCH_DEBUG_MSG("RESET Detected. Start Recover \n");

		if (mxt_patchevent_get(PATCH_EVENT_TA)) {
			TOUCH_DEBUG_MSG("   Stage 1 : USB/TA \n");
			global_mxt_data->knock_on_mode = CHARGER_PLUGGED;
			mxt_patch_event(global_mxt_data,
					global_mxt_data->knock_on_mode);
		}

		if (mxt_patchevent_get(PATCH_EVENT_KNOCKON)) {
			TOUCH_DEBUG_MSG("   Stage 2 : Knock On \n");
			if (mxt_patchevent_get(PATCH_EVENT_TA)) {
				mxt_patch_event(global_mxt_data,
						CHARGER_KNOCKON_SLEEP);
			} else {
				mxt_patch_event(global_mxt_data,
						NOCHARGER_KNOCKON_SLEEP);
			}
		}
		TOUCH_DEBUG_MSG("Recover Complete\n");
	}
}

static void mxt_input_sync(struct input_dev *input_dev)
{
	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);
}

static void mxt_firmware_update_func(struct work_struct *work_firmware_update)
{
	struct mxt_data *data = container_of(
			to_delayed_work(work_firmware_update),
			struct mxt_data, work_firmware_update);
	int error = 0;

	TOUCH_DEBUG_MSG("%s \n", __func__);

	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(2000));

	touch_disable_irq(data->irq);

	error = mxt_update_firmware(data, data->pdata->fw_name);
	if (error) {
		TOUCH_ERR_MSG("%s error \n", __func__);
		goto exit;
	}

	if (data->T100_reportid_min) {
		error = mxt_read_t100_config(data);
		if (error) {
			TOUCH_ERR_MSG("Failed to initialize T100 resolution\n");
			goto exit;
		}
	} else {
		TOUCH_ERR_MSG("Failed to read touch object\n");
		goto exit;
	}

exit:
	if (mxt_patchevent_get(PATCH_EVENT_TA))
		trigger_usb_state_from_otg(1);
	else
		trigger_usb_state_from_otg(0);

	touch_enable_irq(data->irq);

	mxt_read_fw_version(data);
}

static void mxt_button_lock_func(struct work_struct *work_button_lock)
{
	struct mxt_data *data = container_of(
			to_delayed_work(work_button_lock),
			struct mxt_data, work_button_lock);

	mutex_lock(&data->input_dev->mutex);
	data->button_lock = false;
	mutex_unlock(&data->input_dev->mutex);
}

static void mxt_palm_unlock_func(struct work_struct *work_palm_unlock)
{
	struct mxt_data *data = container_of(
			to_delayed_work(work_palm_unlock),
			struct mxt_data, work_palm_unlock);

	mutex_lock(&data->input_dev->mutex);
	data->palm = false;
	mutex_unlock(&data->input_dev->mutex);
}

static void mxt_delay_cal_func(struct work_struct *work_delay_cal)
{
	struct mxt_data *data = container_of(
			to_delayed_work(work_delay_cal),
			struct mxt_data, work_delay_cal);

	TOUCH_INFO_MSG("Delayed work Calibration\n");
	/* Recalibrate since chip has been in deep sleep */
	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
	data->enable_reporting = true;
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct input_dev *input_dev = data->input_dev;
	int id = 0;
	u8 status = 0;
	int x = 0;
	int y = 0;
	int area = 0;
	int amplitude = 0;
	u8 vector = 0;
	int i = 0;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];
	amplitude = message[6];
	vector = message[7];

	if (unlikely( id < 0 || id >= MXT_MAX_FINGER)) {
		TOUCH_INFO_MSG("%s wrong id:%d \n", __func__, id);
		return;
	}

	if (status & MXT_T9_SUPPRESS) {
		if (touched_finger_count) {
			TOUCH_INFO_MSG(" MXT_T9_SUPPRESS\n");
			mxt_reset_slots(data);
		}
		if (data->palm) {
			cancel_delayed_work_sync(&data->work_palm_unlock);
		}
		data->palm = true;
		return;
	}

	if (status & MXT_T9_DETECT) {
		if ((status & MXT_T9_PRESS ||
		    (status & MXT_T9_MOVE)) &&
		    data->ts_data.prev_data[id].status < MXT_STATE_PRESS) {
			if (data->reported_keycode) {
				data->reported_keycode = 0;
				return ;
			}

			TOUCH_INFO_MSG("%d finger pressed <%d> : x[%3d] y[%3d] z[%3d] area[%3d]\n",
					++touched_finger_count, id, x, y, amplitude, area);
		}

		data->fingers[id].state = MXT_STATE_PRESS;
		data->ts_data.curr_data[id].status = MXT_STATE_PRESS;

		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
		{
			int tmp;
			tmp = x;
			x = y;
			y = tmp;
			TOUCH_INFO_MSG("###t9## x[%d], y[%d]\n", x, y);
		}
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);

		data->button_lock = true;
	}

	if (status & MXT_T9_RELEASE) {
		if (touched_finger_count &&
		    data->ts_data.prev_data[id].status < MXT_STATE_PRESS) {
			for (i = MXT_MAX_FINGER - 1; i >= 0; i--) {
				if (data->ts_data.prev_data[i].status>= MXT_STATE_PRESS) {
					TOUCH_INFO_MSG("finger id changed <%d> -> <%d> \n", id, i);
					id = i;
					data->ts_data.prev_data[id].status = MXT_STATE_PRESS;
					break;
				}
			}
		}

		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		TOUCH_INFO_MSG("touch_release    <%d> : x[%3d] y[%3d]\n", id, x, y);
		data->fingers[id].state = MXT_STATE_RELEASE;
		data->ts_data.curr_data[id].status= MXT_STATE_RELEASE;
		data->ts_data.curr_data[id].pressure = 0;
		if (touched_finger_count)
			data->ts_data.total_num = --touched_finger_count;

		if (!touched_finger_count)
			queue_delayed_work(touch_wq,
				&data->work_button_lock,
				msecs_to_jiffies(200));
	}

	input_sync(input_dev);

	data->ts_data.curr_data[id].x_position = x;
	data->ts_data.curr_data[id].y_position = y;
	data->ts_data.curr_data[id].pressure = amplitude;
	data->ts_data.curr_data[id].width_major = area;
	data->ts_data.curr_data[id].width_orientation = vector;
	data->ts_data.total_num = touched_finger_count;
	data->ts_data.prev_data[id]= data->ts_data.curr_data[id];
	data->ts_data.prev_total_num = data->ts_data.total_num;
	data->update_input = true;
}

/* T-series of Atmel Touch IC
 * The Touch Suppression T42 does not report its own messages.
 * Screen suppression messages are reported through the linked
 * Multiple Touch Touchscreen T100 object. */
static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP) {
		TOUCH_DEBUG_MSG("Palm detected %d\n", touched_finger_count);
		data->button_lock = true;
	} else {
		TOUCH_DEBUG_MSG("Palm released \n");
		queue_delayed_work(touch_wq, &data->work_button_lock, msecs_to_jiffies(200));
	}
	mxt_reset_slots(data);
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	u8 status = 0, state = 0;

	status = msg[1];
	state  = msg[4];

	TOUCH_DEBUG_MSG("T48 state %d status %02X %s%s%s%s%s\n",
			state,
			status,
			(status & 0x01) ? "FREQCHG " : "",
			(status & 0x02) ? "APXCHG " : "",
			(status & 0x04) ? "ALGOERR " : "",
			(status & 0x10) ? "STATCHG " : "",
			(status & 0x20) ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t57_messages(struct mxt_data *data, u8 *message)
{
	u16 area = 0;
	u16 touch_area = 0;
	u16 anti_touch_area = 0;

	area = (message[2] << 8 | message[1]);
	touch_area = (message[4] << 8 | message[3]);
	anti_touch_area = (message[6] << 8 | message[5]);

	if (data->t57_debug_enabled)
		TOUCH_DEBUG_MSG("T57 :%3d %3d %3d\n",
				area, touch_area, anti_touch_area);
}

static int mxt_proc_t25_message(struct mxt_data *data, u8 *message)
{
	u8 status = message[1];

	if (!selftest_enable)
		return 0;

	TOUCH_DEBUG_MSG("T25 Self Test completed %u\n",status);

	memset(data->self_test_status, 0, sizeof(data->self_test_status));

	if (selftest_show)
		data->self_test_status[0] = status;

	if ( status == 0xFE ) {
		TOUCH_INFO_MSG("[SUCCESS] All tests passed\n");
		data->self_test_result = true;
	} else {
		if (status == 0xFD) {
			TOUCH_ERR_MSG("[FAIL] Invalid test code\n");
		} else if (status == 0xFC)  {
			TOUCH_ERR_MSG("[FAIL] Unrelated fault\n");
		} else if (status == 0x01) {
			TOUCH_ERR_MSG("[FAIL] AVdd or XVdd is not present\n");
		} else if (status == 0x12) {
			TOUCH_ERR_MSG("[FAIL] Pin fault (SEQ_NUM %u, X_PIN %u, Y_PIN %u)\n", message[2], message[3], message[4]);
			if (selftest_show) {
				data->self_test_status[1] = message[2];
				data->self_test_status[2] = message[3];
				data->self_test_status[3] = message[4];
			}
		} else if (status == 0x17) {
			TOUCH_ERR_MSG("[FAIL] Signal limit fault (TYPE_NUM %u, TYPE_INSTANCE %u)\n", message[2], message[3]);
			if (selftest_show) {
				data->self_test_status[1] = message[2];
				data->self_test_status[2] = message[3];
			}
		} else;
		data->self_test_result = false;
	}

	selftest_enable = false;
	complete(&data->t25_completion);
	return 0;
}

static void mxt_proc_t24_messages(struct mxt_data *data, u8 *message)
{
	u8 msg = 0;
	int x = 0;
	int y = 0;

	if (data->in_bootloader)
		return;

	msg = message[1];

	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	if (msg == 0x04) {
		wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(2000));
		TOUCH_DEBUG_MSG("Knock On detected x[%3d] y[%3d] \n", x, y);
		kobject_uevent_env(&device_touch.kobj, KOBJ_CHANGE, knockon_event);
	} else {
		TOUCH_DEBUG_MSG("%s msg = %d \n", __func__, msg);
	}
}

static void mxt_proc_t80_messages(struct mxt_data *data, u8 *message)
{
	if (data->debug_enabled)
		print_hex_dump(KERN_ERR, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
		       message, data->T5_msg_size, false);
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	int id;
	u8 status;
	int x, y;
	int area;
	int amplitude;
	u8 vector, height, width;
	static int pre_id = 0, pre_x = 0, pre_y = 0;
	static int return_cnt = 0;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting){
		TOUCH_DEBUG_MSG( "return event\n");
		if (return_cnt++ > 30) {
			TOUCH_ERR_MSG( "recalibration\n");
			touch_disable_irq(data->irq);
			queue_delayed_work(touch_wq, &data->work_delay_cal,
					msecs_to_jiffies(10));
			data->delayed_cal = false;
			msleep(50);
			touch_enable_irq(data->irq);
			return_cnt = 0;
		}
		return;
	}

	return_cnt = 0;
	id = message[0] - data->T100_reportid_min - 2;

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	vector =  message[data->t100_aux_vect];
	amplitude = message[data->t100_aux_ampl];	/* message[6] */
	area = message[data->t100_aux_area];

	height = message[data->t100_aux_resv];
	width = message[data->t100_aux_resv+1];

	if (status & MXT_T100_DETECT) {
		/* Multiple bits may be set if the host is slow to read the
		* status messages, indicating all the events that have
		* happened */

		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS) {
			if (data->stylus_in_a_row_cnt == 0)
				pre_id = id;

			if (data->patch.src_item[MXT_PATCH_ITEM_USER4] == 0) {
				if (pre_id == id) {
					if(data->stylus_in_a_row_cnt_thr < data->stylus_in_a_row_cnt++) {
						data->patch.src_item[MXT_PATCH_ITEM_USER4] = 1;
						TOUCH_INFO_MSG("Stylus Continually Mode Enter \n");
						data->stylus_in_a_row_cnt = 0;
					}
				} else {
					data->stylus_in_a_row_cnt = 0;
				}
			} else if(data->patch.src_item[MXT_PATCH_ITEM_USER4] == 1) {
				if(data->stylus_in_a_row_cnt == 0) {
					pre_x = x;
					pre_y = y;
				}
				if(pre_id == id && pre_x <= (x + data->x_zitter) && pre_x >= (x - data->x_zitter) && pre_y <= (y + data->y_zitter) && pre_y >= (y - data->y_zitter))
				{
					if(data->stylus_in_a_row_cnt_thr < data->stylus_in_a_row_cnt++) {
						data->patch.src_item[MXT_PATCH_ITEM_USER4] = 2;
						TOUCH_INFO_MSG("Stylus Continually Mode Exit \n");
						data->stylus_in_a_row_cnt = 0;
					}
				}
				else
				{
					data->stylus_in_a_row_cnt = 0;
				}
			}
			else if(data->patch.src_item[MXT_PATCH_ITEM_USER4] == 2)
			{
				if(data->stylus_in_a_row_cnt++ > 100) {
					TOUCH_INFO_MSG("Stylus Continually Mode Re-Chk \n");
					data->patch.src_item[MXT_PATCH_ITEM_USER4] = 0;
                            data->stylus_in_a_row_cnt = 0;
				}
			}
		} else {
			data->stylus_in_a_row_cnt = 0;
		}

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE ||
		    (status & MXT_T100_STATUS_MASK) == MXT_T100_SUPPRESSION) {
			data->ts_data.curr_data[id].id = id;
			data->ts_data.curr_data[id].status = FINGER_RELEASED;
			if ((status & MXT_T100_STATUS_MASK) ==
					MXT_T100_SUPPRESSION)
				TOUCH_ERR_MSG("T100_message[%u] ###DETECT &&"
					" SUPPRESSION (%02X)\n", id, status);
		}

		data->ts_data.curr_data[id].id = id;
		data->ts_data.curr_data[id].x_position = x;
		data->ts_data.curr_data[id].y_position = y;
		data->ts_data.curr_data[id].area = area;
		data->ts_data.curr_data[id].tool = MT_TOOL_FINGER;

		if (amplitude == 255 &&
		    ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_FINGER ||
		     (status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)) {
			data->ts_data.curr_data[id].pressure = 240;
		} else if ((status & MXT_T100_TYPE_MASK) ==
				MXT_T100_TYPE_PALM) {
			data->ts_data.curr_data[id].pressure = 255;
			TOUCH_ERR_MSG("Palm Detected\n");
		} else {
			data->ts_data.curr_data[id].pressure = amplitude;
		}

		if (height >= width) {
			data->ts_data.curr_data[id].touch_major = height;
			data->ts_data.curr_data[id].touch_minor = width;
		} else {
			data->ts_data.curr_data[id].touch_major = width;
			data->ts_data.curr_data[id].touch_minor = height;
		}

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS)
			data->ts_data.curr_data[id].status = FINGER_PRESSED;
		else if ((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE)
			data->ts_data.curr_data[id].status = FINGER_MOVED;

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS)
#ifdef CONFIG_TOUCHSCREEN_MSG_INFO_PRINT
			TOUCH_DEBUG_MSG("touch_pressed    <%d> :"
				" x[%4d] y[%4d] A[%3d] P[%3d]"
				" WM[%3d] Wm[%3d]\n",
				id, x, y, area, amplitude,
				data->ts_data.curr_data[id].touch_major,
				data->ts_data.curr_data[id].touch_minor);
#else
			TOUCH_INFO_MSG("touch_pressed    <%d> :"
				" x[***] y[***] A[%3d] P[%3d]"
				" WM[%3d] Wm[%3d]\n",
				id, area, amplitude,
				data->ts_data.curr_data[id].touch_major,
				data->ts_data.curr_data[id].touch_minor);
#endif
	} else if ((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE){
		/* Touch Release */
		data->ts_data.curr_data[id].id = id;
		data->ts_data.curr_data[id].status = FINGER_RELEASED;
#ifdef CONFIG_TOUCHSCREEN_MSG_INFO_PRINT
		TOUCH_DEBUG_MSG("touch_release    <%d> : x[%4d] y[%4d]\n",
				id, x, y);
#else
		TOUCH_INFO_MSG("touch_release    <%d> : x[***] y[***]\n", id);
#endif
	}

	if (data->debug_enabled) {
		TOUCH_INFO_MSG( "T100_message[%u] %s%s%s%s%s%s%s%s%s"
			" %s%s%s%s%s (0x%02X) x:%u y:%u z:%u area:%u amp:%u"
			" vec:%u h:%u w:%u\n",
			id,
			((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE)?
				"MOVE" : "",
			((status & MXT_T100_STATUS_MASK) == 2)? "UNSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 3)? "SUP" : "",
			((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS)?
				"PRESS" : "",
			((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE)?
				"RELEASE" : "",
			((status & MXT_T100_STATUS_MASK) == 6)? "UNSUPSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 7)? "UNSUPUP" : "",
			((status & MXT_T100_STATUS_MASK) == 8)? "DOWNSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 9)? "DOWNUP" : "",
			((status & MXT_T100_TYPE_MASK) ==
				 MXT_T100_TYPE_FINGER)? "FIN" : ".",
			((status & MXT_T100_TYPE_MASK) ==
				 MXT_T100_TYPE_STYLUS)? "PEN" : ".",
			((status & MXT_T100_TYPE_MASK) ==
				 MXT_T100_TYPE_PALM)? "PALM" : ".",
			((status & MXT_T100_TYPE_MASK) == 0x40)? "HOVER" : ".",
			((status & MXT_T100_TYPE_MASK) == 0x30)? "ACTSTY" : ".",
			status, x, y, data->ts_data.curr_data[id].pressure,
			area, amplitude, vector,
			height, width);
	}

	data->update_input = true;
}

static void mxt_proc_t109_messages(struct mxt_data *data, u8 *msg)
{
	u8 command = msg[1];
	int ret = 0;

	TOUCH_DEBUG_MSG("%s CMD: %u\n", __func__, command);

	if (command == MXT_T109_CMD_TUNE) {
		TOUCH_DEBUG_MSG("%s Store to Configuration RAM.\n", __func__);
		mxt_t109_command(data, MXT_T109_CMD, MXT_T109_CMD_STORE_RAM);
	} else if (command == MXT_T109_CMD_STORE_RAM) {
		/* Self Tune Completed.. */
		mxt_write_object(data, MXT_SPT_USERDATA_T38, 1, 1);

		/* Backup to memory */
		ret = mxt_command_backup(data, MXT_BACKUP_VALUE);
		if (ret) {
			TOUCH_ERR_MSG("Failed backup NV data\n");
			return;
		}

		/* Soft reset */
		ret = mxt_command_reset(data, MXT_RESET_VALUE);
		if (ret) {
			TOUCH_ERR_MSG("Failed Reset IC\n");
			return;
		}

		TOUCH_DEBUG_MSG("%s Self Tune Complete.\n", __func__);
	}
}

static int mxt_update_file_name(struct device *dev, char **file_name,
		const char *buf, size_t count)
{
	char *file_name_tmp = NULL;

	/* Simple sanity check */
	if (count < 1 || count > 128) {
		TOUCH_ERR_MSG("File name too short or long \n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		TOUCH_ERR_MSG("no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t mxt_update_patch_store(struct mxt_data *data, const char *buf,
		size_t count)
{
	u8 *patch_data = NULL;
	const struct firmware *fw = NULL;
	char *name = NULL;
	int ret = 0;

	ret = mxt_update_file_name(&data->client->dev, &name, buf, count);
	if (ret) {
		TOUCH_ERR_MSG("%s error patch name [%s] \n", __func__, name);
		goto out;
	}

	ret = request_firmware(&fw, name, &data->client->dev);
	if (ret < 0) {
		TOUCH_ERR_MSG("Fail to request firmware(%s)\n", name);
		goto out;
	}

	patch_data = kzalloc(fw->size, GFP_KERNEL);
	if (!patch_data) {
		TOUCH_ERR_MSG("Failed to alloc buffer for fw\n");
		ret = -ENOMEM;
		goto out;
	}

	memcpy(patch_data, fw->data, fw->size);
	if (data->patch.patch) {
		kfree(data->patch.patch);
		data->patch.patch = NULL;
	}
	data->patch.patch = patch_data;
	
	TOUCH_DEBUG_MSG("%s ppatch:%p %p\n", __func__,
			patch_data, data->patch.patch);

	ret = mxt_patch_init(data, data->patch.patch);
	if (ret) {
		global_mxt_data = NULL;
		TOUCH_ERR_MSG("%s global_mxt_data is NULL \n", __func__);
		goto out;
	}

	global_mxt_data = data;

	release_firmware(fw);

	return count;

out:
	if (fw)
		release_firmware(fw);

	if (patch_data)
		kfree(patch_data);

	data->patch.patch = NULL;

	return ret;
}

void trigger_usb_state_from_otg(int usb_type)
{
	TOUCH_INFO_MSG("USB trigger USB_type: %d \n", usb_type);
	g_usb_type = usb_type;

	if (!global_mxt_data) {
		TOUCH_ERR_MSG("global_mxt_data is null\n");
		return;
	}

	if (!global_mxt_data->patch.event_cnt) {
		TOUCH_DEBUG_MSG("patch.event_cnt = 0\n");
		return;
	}

	global_mxt_data->global_object =
		mxt_get_object(global_mxt_data,
				MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (!global_mxt_data->global_object)
		return;

	if (global_mxt_data->regulator_status == 0) {
		TOUCH_INFO_MSG("IC Regulator Disabled. Do nothing\n");
		if (usb_type == 0) {
			mxt_patchevent_unset(PATCH_EVENT_TA);
			global_mxt_data->charging_mode = 0;
		} else {
			mxt_patchevent_set(PATCH_EVENT_TA);
			global_mxt_data->charging_mode = 1;
		}
		return;
	}

	if (global_mxt_data->mxt_mode_changed) {
		TOUCH_INFO_MSG("Do not change when mxt mode changed\n");
		if (usb_type == 0) {
			mxt_patchevent_unset(PATCH_EVENT_TA);
			global_mxt_data->charging_mode = 0;
		} else {
			mxt_patchevent_set(PATCH_EVENT_TA);
			global_mxt_data->charging_mode = 1;
		}
		return;
	}

	if (global_mxt_data->mfts_enable && global_mxt_data->pdata->use_mfts) {
		TOUCH_INFO_MSG("MFTS : Not support USB trigger \n");
		return;
	}

	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(2000));

	if (mutex_is_locked(&i2c_suspend_lock))
		TOUCH_DEBUG_MSG("%s mutex_is_locked \n", __func__);

	mutex_lock(&i2c_suspend_lock);
	if (usb_type == 0) {
		if (mxt_patchevent_get(PATCH_EVENT_TA)) {
			if (mxt_patchevent_get(PATCH_EVENT_KNOCKON)) {
				mxt_patchevent_unset(PATCH_EVENT_KNOCKON);
				mxt_patch_event(global_mxt_data, CHARGER_KNOCKON_WAKEUP);
			}
			global_mxt_data->charging_mode = 0;
			mxt_patch_event(global_mxt_data, CHARGER_UNPLUGGED);
			mxt_patchevent_unset(PATCH_EVENT_TA);
		}
	} else {
		if (mxt_patchevent_get(PATCH_EVENT_KNOCKON)) {
			mxt_patch_event(global_mxt_data, NOCHARGER_KNOCKON_WAKEUP);
			mxt_patchevent_unset(PATCH_EVENT_KNOCKON);
		}
		global_mxt_data->charging_mode = 1;


		mxt_patch_event(global_mxt_data, CHARGER_PLUGGED);

		mxt_patchevent_set(PATCH_EVENT_TA);
	}

	mutex_unlock(&i2c_suspend_lock);
}

static void mxt_proc_message_log(struct mxt_data *data, u8 type)
{
	if (mxt_patchevent_get(PATCH_EVENT_KNOCKON) &&
	    (type != MXT_GEN_COMMAND_T6 &&
	     type != MXT_SPT_CTECONFIG_T46 &&
	     type != MXT_PROCI_SHIELDLESS_T56 &&
	     type != MXT_SPT_TIMER_T61 &&
	     type != MXT_PROCG_NOISESUPPRESSION_T72))
			TOUCH_DEBUG_MSG("mxt_interrupt T%d \n", type);
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	u8 type = 0;
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	type = data->reportids[report_id].type;

	mxt_proc_message_log(data, type);

	switch (type) {
		case MXT_GEN_COMMAND_T6:
			mxt_proc_t6_messages(data, message);
			break;
		case MXT_TOUCH_MULTI_T9:
			mxt_proc_t9_message(data, message);
			break;
		case MXT_PROCI_ONETOUCH_T24:
			if (data->mxt_knock_on_enable)
				mxt_proc_t24_messages(data, message);
			break;
		case MXT_SPT_SELFTEST_T25:
			mxt_proc_t25_message(data, message);
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			mxt_proc_t42_messages(data, message);
			break;
		case MXT_SPT_CTECONFIG_T46:
			TOUCH_DEBUG_MSG("MXT_SPT_CTECONFIG_T46");
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			mxt_proc_t48_messages(data, message);
			break;
		case MXT_PROCI_SHIELDLESS_T56:
			TOUCH_DEBUG_MSG("MXT_PROCI_SHIELDLESS_T56");
			break;
		case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
			mxt_proc_t57_messages(data, message);
			break;
		case MXT_PROCG_NOISESUPPRESSION_T72:
			TOUCH_DEBUG_MSG("MXT_PROCG_NOISESUPPRESSION_T72"
					" Noise Status:%d\n",
					message[2] & 0x07);
			break;
		case MXT_RETRANSMISSIONCOMPENSATION_T80:
			mxt_proc_t80_messages(data, message);
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			mxt_proc_t100_message(data, message);
			break;
		case MXT_SPT_SELFCAPGLOBALCONFIG_T109:
			mxt_proc_t109_messages(data, message);
			break;
		case MXT_SPT_TIMER_T61:
			TOUCH_DEBUG_MSG("MXT_SPT_TIMER_T61\n");
			break;
		default:
			TOUCH_ERR_MSG("%s : Unknown T%d \n", __func__, type);
			dump = true;
			break;
	}

	if (dump)
		mxt_dump_message(data, message);

	if (!mxt_power_block_get())
		mxt_patch_message(data, (struct mxt_message*)message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	int ret = 0;
	int i = 0;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	if (!data->msg_buf) {
		TOUCH_ERR_MSG("%s data->msg_buf = NULL \n", __func__);
		return -EINVAL;
	}

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
			data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		TOUCH_ERR_MSG("Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
				data->msg_buf + data->T5_msg_size * i);
		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->pdata;
	int ret;
	int report_num = 0;
	int i = 0;
	u8 count, num_left;

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		TOUCH_ERR_MSG( "Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];
	if (count == 0) {
		/* Zero message is too much occured.
		 * Remove this log until firmware fixed */
		return IRQ_HANDLED;
	} else if (count > data->max_reportid) {
		TOUCH_ERR_MSG("T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	data->ts_data.total_num = 0;

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		TOUCH_ERR_MSG( "Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;
	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			TOUCH_ERR_MSG( "Unexpected invalid message\n");
	}

	for (i = 0; i < data->pdata->numtouch; i++) {
		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE &&
		    data->ts_data.prev_data[i].status != FINGER_INACTIVE &&
		    data->ts_data.prev_data[i].status != FINGER_RELEASED) {
			memcpy(&data->ts_data.curr_data[i],
			       &data->ts_data.prev_data[i],
			       sizeof(data->ts_data.prev_data[i]));
			data->ts_data.curr_data[i].skip_report = true;
		} else if (data->ts_data.curr_data[i].status ==
				FINGER_INACTIVE) {
			continue;
		}

		if (data->ts_data.curr_data[i].status == FINGER_PRESSED ||
		    data->ts_data.curr_data[i].status == FINGER_MOVED) {
			data->ts_data.total_num++;
		}
		report_num++;
	}

	if (!data->enable_reporting || !report_num)
		goto out;

	for (i = 0; i < data->pdata->numtouch; i++) {
		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE ||
		    data->ts_data.curr_data[i].skip_report) {
			continue;
		}
		if (data->ts_data.curr_data[i].pressure == 255 &&
		    pdata->palm_enabled && !data->palm) {
			mxt_reset_slots(data);
			input_report_key(data->input_dev, KEY_SLEEP, 1);
			input_sync(data->input_dev);
			data->palm = true;
			goto out;
		}
		if (data->ts_data.curr_data[i].status == FINGER_RELEASED) {
			if (pdata->palm_enabled && data->palm) {
				input_report_key(data->input_dev, KEY_SLEEP, 0);
				input_sync(data->input_dev);
				data->palm = false;
				goto out;
			} else {
				input_mt_slot(data->input_dev,
						data->ts_data.curr_data[i].id);
				input_mt_report_slot_state(data->input_dev,
						MT_TOOL_FINGER, 0);
			}
		} else if (data->palm == false) {
			input_mt_slot(data->input_dev,
					data->ts_data.curr_data[i].id);
			input_mt_report_slot_state(data->input_dev,
					MT_TOOL_FINGER, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					data->ts_data.curr_data[i].id);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					data->ts_data.curr_data[i].x_position);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					data->ts_data.curr_data[i].y_position);

			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					data->ts_data.curr_data[i].pressure);
			/* Report Palm event end */

			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					data->ts_data.curr_data[i].touch_major);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR,
					data->ts_data.curr_data[i].touch_minor);
		}
	}

	if(data->ts_data.total_num < data->ts_data.prev_total_num)
		TOUCH_DEBUG_MSG( "[finger] Total_num(move+press)= %d\n",
				data->ts_data.total_num);

	if (data->ts_data.total_num) {
		data->ts_data.prev_total_num = data->ts_data.total_num;
		memcpy(data->ts_data.prev_data, data->ts_data.curr_data,
				sizeof(data->ts_data.curr_data));
	} else {
		data->ts_data.prev_total_num = 0;
		memset(data->ts_data.prev_data, 0,
				sizeof(data->ts_data.prev_data));
	}
	memset(data->ts_data.curr_data, 0, sizeof(data->ts_data.curr_data));

end:
	if (data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

out:
	return IRQ_HANDLED;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled = 0, num_handled = 0;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count);
	if (total_handled < 0)
		return IRQ_HANDLED;
	/* if there were invalid messages, then we are done */

	if (total_handled <= count)
		goto update_count;

	/* read two at a time until an invalid message or else we reach
	 * reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_HANDLED;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->enable_reporting && data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

/* touch_irq_handler
 *
 * When Interrupt occurs, it will be called before touch_thread_irq_handler.
 *
 * return
 * IRQ_HANDLED: touch_thread_irq_handler will not be called.
 * IRQ_WAKE_THREAD: touch_thread_irq_handler will be called.
 */
static irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
	struct mxt_data *data = (struct mxt_data *)dev_id;
	if (data->pm_state >= PM_SUSPEND) {
		TOUCH_INFO_MSG("interrupt in suspend[%d]\n", data->pm_state);
		data->pm_state = PM_SUSPEND_IRQ;
		wake_lock_timeout(&pm_touch_wake_lock, msecs_to_jiffies(1000));
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t mxt_interrupt_thread(int irq, void *dev_id)
{
	struct mxt_data *data = (struct mxt_data*)dev_id;
	irqreturn_t ret = IRQ_NONE;

	if (data->in_bootloader) {
		/* bootloader state transition completion */
		complete(&data->bl_completion);
		return IRQ_HANDLED;
	}

	if (!data->object_table)
		return IRQ_HANDLED;

	mutex_lock(&i2c_suspend_lock);
	if (data->T44_address)
		ret = mxt_process_messages_t44(data);
	else
		ret = mxt_process_messages(data);
	mutex_unlock(&i2c_suspend_lock);

	return ret;
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset, u8 value,
		bool wait)
{
	u16 reg = 0;
	u8 command_register = 0;
	int timeout_counter = 0;
	int ret = 0;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		TOUCH_ERR_MSG("%s Command failed!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_t109_command(struct mxt_data *data, u16 cmd_offset, u8 value)
{
	u16 reg = 0;
	int ret = 0;

	reg = data->T109_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	reinit_completion(&data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	msleep(MXT_RESET_TIME);

	return 0;
}

static int mxt_hw_reset(struct mxt_data *data)
{
	TOUCH_INFO_MSG("%s \n", __func__);

	if (!gpio_is_valid(data->pdata->gpio_reset))
		return 0;

	gpio_set_value(data->pdata->gpio_reset, 0);
	usleep_range(5000, 10000);

	gpio_set_value(data->pdata->gpio_reset, 1);
	msleep(MXT_RESET_TIME);

	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/* on failure, CRC is set to 0 and config will always be downloaded */
	data->config_crc = 0;
	reinit_completion(&data->crc_completion);

	mxt_t6_command(data, cmd, value, true);
	/* Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded */

	msleep(MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result = 0;
	u32 data_word = 0;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	int error = 0;
	struct t7_config *new_config = NULL;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };
	u16 tmp;

	tmp = (u16)sizeof(data->t7_cfg);

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &data->t7_cfg;

	error = __mxt_write_reg(data->client,
			data->T7_address, tmp, new_config);
	if (error)
		return error;

	TOUCH_DEBUG_MSG("Set T7 ACTV:%d IDLE:%d\n",
			new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	int error = 0;
	bool retry = false;
	u16 tmp;

	tmp = (u16)sizeof(data->t7_cfg);
recheck:
	error = __mxt_read_reg(data->client, data->T7_address, tmp,
			&data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			TOUCH_DEBUG_MSG("T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			TOUCH_DEBUG_MSG("T7 cfg zero after reset,"
					" overriding\n");
			data->t7_cfg.active = 20;
			data->t7_cfg.idle = 100;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	} else {
		TOUCH_ERR_MSG("Initialised power cfg: ACTV %d, IDLE %d\n",
				data->t7_cfg.active, data->t7_cfg.idle);
		return 0;
	}
}

static int mxt_check_reg_init(struct mxt_data *data, const char *name)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info = {0};
	struct mxt_object *object = NULL;
	const struct firmware *cfg = NULL;
	int ret = 0;
	int offset = 0;
	int data_pos = 0;
	int byte_offset = 0;
	int i = 0;
	int cfg_start_ofs = 0;
	u32 info_crc = 0, config_crc = 0, calculated_crc = 0;
	u8 *config_mem = 0;
	u32 config_mem_size = 0;
	unsigned int type = 0, instance, size = 0;
	u8 val = 0;
	u16 reg = 0;

	TOUCH_DEBUG_MSG("%s \n", __func__);

	if (!name) {
		TOUCH_ERR_MSG("Skipping cfg download \n");
		return 0;
	}

	ret = request_firmware(&cfg, name, dev);
	if (ret < 0) {
		TOUCH_ERR_MSG("Failure to request config file [%s]\n", name);
		return -EINVAL;
	}

	TOUCH_DEBUG_MSG("Open [%s] configuration file \n", name);

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		TOUCH_ERR_MSG("Unrecognised config file\n");
		ret = -EINVAL;
		goto release;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n",
				(unsigned char *)&cfg_info + i, &offset);
		if (ret != 1) {
			TOUCH_ERR_MSG("Bad format\n");
			ret = -EINVAL;
			goto release;
		}

		data_pos += offset;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		TOUCH_ERR_MSG("Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		TOUCH_ERR_MSG("Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	TOUCH_DEBUG_MSG("RAW Config CRC is 0x%06X \n", config_crc);

	if (data->config_crc == config_crc) {
		TOUCH_INFO_MSG("Config already applied \n");
		ret = 0;
		goto release_mem;
	}

	if (memcmp((char *)data->info, (char *)&cfg_info,
				sizeof(struct mxt_info)) != 0) {
		TOUCH_DEBUG_MSG("Compatibility Error. Could not apply\n");
		TOUCH_DEBUG_MSG("Info Block [IC]   %02X %02X %02X"
				" %02X %02X %02X %02X \n",
			data->info->family_id, data->info->variant_id,
			data->info->version, data->info->build,
			data->info->matrix_xsize, data->info->matrix_ysize,
			data->info->object_num);

		TOUCH_DEBUG_MSG("Info Block [File] %02X %02X %02X"
				" %02X %02X %02X %02X \n",
			cfg_info.family_id, cfg_info.variant_id,
			cfg_info.version, cfg_info.build,
			cfg_info.matrix_xsize, cfg_info.matrix_ysize,
			cfg_info.object_num);

		ret = -EINVAL;
		goto release_mem;
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START + data->info->object_num *
		sizeof(struct mxt_object) + MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		TOUCH_ERR_MSG("Failed to allocate memory\n");
		ret = -ENOMEM;
		goto release;
	}

	while (data_pos < cfg->size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
				&type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			TOUCH_ERR_MSG("Bad format: failed to parse object\n");
			ret = -EINVAL;
			goto release_mem;
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n",
						&val, &offset);
				data_pos += offset;
			}
			continue;
		}

		if (instance >= mxt_obj_instances(object)) {
			TOUCH_ERR_MSG("Object instances exceeded!\n");
			ret = -EINVAL;
			goto release_mem;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		TOUCH_INFO_MSG("\t %04X %04X %04X \n", type, instance, size);

		if (size != mxt_obj_size(object)) {
			TOUCH_ERR_MSG("Size mismatched \n");
			ret = -EINVAL;
			goto release_mem;
		}

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->data + data_pos, "%hhx%n", &val, &offset);
			if (ret != 1) {
				TOUCH_ERR_MSG("Bad format in T%d\n", type);
				ret = -EINVAL;
				goto release_mem;
			}

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if ((byte_offset >= 0) && (byte_offset <= config_mem_size)) {
				*(config_mem + byte_offset) = val;
			} else {
				TOUCH_ERR_MSG("Bad object: reg:%d, T%d, ofs=%d\n", reg, object->type, byte_offset);
				ret = -EINVAL;
				goto release_mem;
			}

			data_pos += offset;
		}
	}

	/* calculate crc of the received configs (not the raw config file) */
	if (data->T7_address < cfg_start_ofs) {
		TOUCH_ERR_MSG("Bad T7 address, T7addr = %x, config offset %x\n", data->T7_address, cfg_start_ofs);
		ret = -EINVAL;
		goto release_mem;
	}

	calculated_crc = mxt_calculate_crc(config_mem, data->T7_address - cfg_start_ofs, config_mem_size);

	/* Check the crc, calculated should match what is in file */
	if (config_crc > 0 && (config_crc != calculated_crc)) {
		TOUCH_ERR_MSG("CRC mismatch in config file, calculated=0x%06X, file=0x%06X\n", calculated_crc, config_crc);
		TOUCH_ERR_MSG("Config not apply \n");
		ret = -EINVAL;
		goto release_mem;
	}

	/* Write configuration as blocks */
	byte_offset = 0;
	while (byte_offset < config_mem_size) {
		size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		ret = __mxt_write_reg(data->client, cfg_start_ofs + byte_offset, size, config_mem + byte_offset);
		if (ret != 0) {
			TOUCH_ERR_MSG("Config write error, ret=%d\n", ret);
			ret = -EINVAL;
			goto release_mem;
		}

		byte_offset += size;
	}

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	if ((config_crc > 0) && (config_crc != data->config_crc)) {
		TOUCH_ERR_MSG("Config CRC is mismatched 0x%06X \n", data->config_crc);
	}

	ret = mxt_soft_reset(data);
	if (ret)
		goto release_mem;

	TOUCH_INFO_MSG("Config written\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	TOUCH_INFO_MSG("kfree in %s \n", __func__);
	kfree(config_mem);
release:
	if(cfg)
		release_firmware(cfg);

	return ret;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	touch_enable_irq(data->irq);

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		TOUCH_DEBUG_MSG("mxt_free_input_device\n");
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	TOUCH_DEBUG_MSG("%s \n", __func__);

	if (data->raw_info_block)
		kfree(data->raw_info_block);

	data->info = NULL;
	data->raw_info_block = NULL;

	mxt_free_input_device(data);
	data->enable_reporting = false;

	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T9_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T18_address = 0;
	data->T24_reportid = 0;
	data->T35_reportid = 0;
	data->T25_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T42_address = 0;
	data->T44_address = 0;
	data->T46_address = 0;
	data->T48_reportid = 0;
	data->T56_address = 0;
	data->T65_address = 0;
	data->T72_address = 0;
	data->T80_address = 0;
	data->T80_reportid = 0;
	data->T93_address = 0;
	data->T93_reportid = 0;
	data->T100_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->T109_address = 0;
	data->T109_reportid = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data)
{
	int i = 0;
	u8 reportid = 0;
	u16 end_address = 0;
	struct mxt_object *object = NULL;
	u8 min_id = 0, max_id = 0;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids * mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		TOUCH_DEBUG_MSG("\t T%02u Start:%u Size:%03u Instances:%u"
			" Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80) {
				/* On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			/* Only handle messages from first T9 instance */
			data->T9_address = object->start_address;
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
				object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_PROCI_ONETOUCH_T24:
			data->T24_reportid = min_id;
			break;
		case MXT_SPT_PROTOTYPE_T35:
			data->T35_reportid = min_id;
			break;
		case MXT_SPT_SELFTEST_T25:
			data->T25_reportid = min_id;
			data->T25_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_address = object->start_address;
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_CTECONFIG_T46:
			data->T46_address = object->start_address;
			break;
		case MXT_PROCI_STYLUS_T47:
			data->T47_address = object->start_address;
			break;
		case MXT_SPT_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_SHIELDLESS_T56:
			data->T56_address = object->start_address;
			break;
		case MXT_PROCI_LENSBENDING_T65:
			data->T65_address = object->start_address;
			break;
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
			data->T71_address = object->start_address;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T72:
			data->T72_address = object->start_address;
			break;
		case MXT_RETRANSMISSIONCOMPENSATION_T80:
			data->T80_address = object->start_address;
			data->T80_reportid = min_id;
			break;
		case MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93:
			data->T93_reportid = min_id;
			data->T93_address = object->start_address;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			/* Only handle messages from first T100 instance */
			data->T100_address = object->start_address;
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = min_id +
				object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids - 2;
			TOUCH_DEBUG_MSG("T100_reportid_min:%d"
					" T100_reportid_max:%d\n",
					data->T100_reportid_min,
					data->T100_reportid_max);
			break;
		case MXT_SPT_SELFCAPGLOBALCONFIG_T109:
			data->T109_address = object->start_address;
			data->T109_reportid = min_id;
			break;
		}

		end_address = object->start_address + mxt_obj_size(object) *
			mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	if (data->msg_buf)
		kfree(data->msg_buf);

	data->msg_buf = kzalloc((data->max_reportid * data->T5_msg_size),
			GFP_KERNEL);
	if (!data->msg_buf) {
		TOUCH_ERR_MSG("%s d Failed to allocate message buffer\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error = 0;
	size_t size = 0;
	void *buf = NULL;
	struct mxt_info *info = NULL;

	TOUCH_INFO_MSG("%s \n", __func__);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		TOUCH_ERR_MSG("%s Failed to allocate memory 1\n", __func__);
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, 0, size, buf);
	if (error) {
		TOUCH_ERR_MSG("%s __mxt_read_reg error \n", __func__);
		goto err_free_mem;
	}

	/* Resize buffer to give space for rest of info block */
	info = (struct mxt_info *)buf;
	size += (MXT_OBJECT_NUM_MAX * sizeof(struct mxt_object)) +
		MXT_INFO_CHECKSUM_SIZE;
	buf = krealloc(buf, size, GFP_KERNEL);
	if (!buf) {
		TOUCH_ERR_MSG("%s Failed to allocate memory 2\n", __func__);
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = __mxt_read_reg(client, MXT_OBJECT_START,
			       size - MXT_OBJECT_START, buf + MXT_OBJECT_START);
	if (error) {
		TOUCH_ERR_MSG("%s __mxt_read_reg error \n", __func__);
		goto err_free_mem;
	}

	/* Save pointers in device data structure */
	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

	if (data->object_table == NULL)
		data->object_table = (struct mxt_object *)(
				buf + MXT_OBJECT_START);

	TOUCH_INFO_MSG("Family:%02X Variant:%02X Binary:%u.%u.%02X"
			" TX:%d RX:%d Objects:%d\n",
			data->info->family_id, data->info->variant_id,
			data->info->version >> 4, data->info->version & 0xF,
			data->info->build, data->info->matrix_xsize,
			data->info->matrix_ysize, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data);
	if (error) {
		TOUCH_ERR_MSG("%s Error %d reading object table\n",
				__func__, error);
		mxt_free_object_table(data);
		return error;
	}

	return 0;

err_free_mem:
	kfree(buf);
	data->raw_info_block = NULL;
	data->info = NULL;
	if(data->object_table)
		kfree(data->object_table);
	data->object_table = NULL;
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error = 0;
	struct t9_range range = {0};
	unsigned char orient = 0;
	struct mxt_object *object = NULL;
	u16 tmp = 0;

	memset(&range, 0, sizeof(range));

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	tmp = (u16)sizeof(range);
	error = __mxt_read_reg(client, object->start_address + MXT_T9_RANGE,
			tmp, &range);
	if (error)
		return error;

	le16_to_cpus(range.x);
	le16_to_cpus(range.y);

	error =  __mxt_read_reg(client, object->start_address + MXT_T9_ORIENT,
			1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 2159;

	if (range.y == 0)
		range.y = 3839;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	TOUCH_INFO_MSG("Touchscreen size X:%u Y:%u\n",
			data->max_x, data->max_y);

	return 0;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error = 0;

	if (data->regulator_status == 1)
		return;

	/* RST PIN is Set to LOW */
	if (gpio_is_valid(data->pdata->gpio_reset)) {
		gpio_set_value(data->pdata->gpio_reset, 0);
		usleep_range(5000, 10000);
	}

	/* vcc_dig enable */
	if (data->vcc_dig) {
		error = regulator_enable(data->vcc_dig);
		if (error < 0) {
			TOUCH_ERR_MSG("vcc_dig regulator enable fail\n");
			return;
		}
	}
	usleep_range(5000, 10000);

	data->regulator_status = 1;

	reinit_completion(&data->bl_completion);

	usleep_range(5000, 10000);

	/* RST PIN is Set to HIGH */
	if (gpio_is_valid(data->pdata->gpio_reset))
		gpio_set_value(data->pdata->gpio_reset, 1);
	msleep(MXT_RESET_TIME);
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	int error = 0;

	if (data->regulator_status == 0)
		return;

	/* RST PIN is Set to HIGH */
	if (gpio_is_valid(data->pdata->gpio_reset))
		gpio_set_value(data->pdata->gpio_reset, 0);

	/* vdd_ana disable */
	if (data->vcc_dig) {
		error = regulator_disable(data->vcc_dig);
		if (error < 0) {
			TOUCH_ERR_MSG("vcc_dig regulator disable fail\n");
			return;
		}
	}

	data->regulator_status = 0;
}

static int mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error = 0;

	/* According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage */
	data->regulator_status = 0;
	data->vcc_dig = NULL;

	data->vcc_dig = devm_regulator_get(dev, "vcc_dig");
	if (IS_ERR(data->vcc_dig)) {
		TOUCH_ERR_MSG("Error %d getting ana regulator\n", error);
		return PTR_ERR(data->vcc_dig);
	}

	error = regulator_set_voltage(data->vcc_dig, 3300000, 3300000);
	if (error < 0) {
		TOUCH_ERR_MSG("Error %d cannot control DVDD regulator\n",
				error);
		return error;
	}

	data->use_regulator = true;
	return 0;
}

static int mxt_configure_objects(struct mxt_data *data)
{
	int error = 0;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		TOUCH_ERR_MSG("Failed to initialize power cfg\n");
		return error;
	}

	/* Check register init values */
	error = mxt_check_reg_init(data, NULL);
	if (error) {
		TOUCH_ERR_MSG("Error %d initialising configuration\n", error);
		return error;
	}

	if (data->T9_reportid_min) {
		error = mxt_initialize_t9_input_device(data);
		if (error)
			return error;
	} else {
		TOUCH_ERR_MSG("No touch object detected\n");
	}
	return 0;
}

static int mxt_rest_init(struct mxt_data *data)
{
	int error = 0;

	error = mxt_acquire_irq(data);
	if (error)
		return error;

	error = mxt_configure_objects(data);
	if (error)
		return error;

	return 0;
}

static void mxt_read_fw_version(struct mxt_data *data)
{
	TOUCH_INFO_MSG("==================================\n");
	TOUCH_INFO_MSG("Firmware Version = %d.%02d.%02d \n",
			data->pdata->fw_ver[0],
			data->pdata->fw_ver[1],
			data->pdata->fw_ver[2]);
	TOUCH_INFO_MSG("FW Product       = %s \n", data->pdata->product);
	TOUCH_INFO_MSG("Binary Version   = %u.%u.%02X \n",
			data->info->version >> 4,
			data->info->version & 0xF,
			data->info->build);
	TOUCH_INFO_MSG("Config CRC       = 0x%X  \n", data->config_crc);
	TOUCH_INFO_MSG("Family Id        = 0x%02X \n", data->info->family_id);
	TOUCH_INFO_MSG("Variant          = 0x%02X \n", data->info->variant_id);
	TOUCH_INFO_MSG("Panel Type       = 0x%02X \n", data->panel_type);
	TOUCH_INFO_MSG("==================================\n");
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct mxt_data *data, char *buf)
{
	return (ssize_t)scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct mxt_data *data, char *buf)
{
	return (ssize_t)scnprintf(buf, PAGE_SIZE, "%02X.%02X\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_testmode_ver_show(struct mxt_data *data, char *buf)
{
	ssize_t ret = 0;

	ret += sprintf(buf+ret, "%d.%02d.%02d (panel_type:0x%02X)",
			data->pdata->fw_ver[0],
			data->pdata->fw_ver[1],
			data->pdata->fw_ver[2],
			data->panel_type);

	return ret;
}

static ssize_t mxt_info_show(struct mxt_data *data, char *buf)
{
	ssize_t ret = 0;

	mxt_read_fw_version(data);

	ret += sprintf(buf+ret, "Firmware Version = %d.%02d.%02d\n",
			data->pdata->fw_ver[0],
			data->pdata->fw_ver[1],
			data->pdata->fw_ver[2]);
	ret += sprintf(buf+ret, "FW Product       = %s\n",
			data->pdata->product);
	ret += sprintf(buf+ret, "Binary Version   = %u.%u.%02X\n",
			data->info->version >> 4,
			data->info->version & 0xF,
			data->info->build);
	ret += sprintf(buf+ret, "Config CRC       = 0x%X\n",
			data->config_crc);
	ret += sprintf(buf+ret, "Family Id        = 0x%02X\n",
			data->info->family_id);
	ret += sprintf(buf+ret, "Variant          = 0x%02X\n",
			data->info->variant_id);
	ret += sprintf(buf+ret, "Patch Date       = %d\n",
			data->patch.date);
	ret += sprintf(buf+ret, "Panel type       = 0x%02X\n",
			data->panel_type);

	return ret;
}

static int mxt_show_instance(char *buf, int count, struct mxt_object *object,
		int instance, const u8 *val)
{
	int i = 0;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct mxt_data *data, char *buf)
{
	struct mxt_object *object = NULL;
	size_t count = 0;
	int i = 0, j = 0;
	int error = 0;
	u8 *obuf = NULL;
	u16 size = 0;
	u16 addr = 0;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kzalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			size = mxt_obj_size(object);
			addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto error;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}
	error = count;

error:
	kfree(obuf);
	return error;
}

static ssize_t mxt_object_control(struct mxt_data *data, const char *buf,
		size_t count)
{
	struct mxt_object *object = NULL;
	unsigned char command[6] = {0};
	int type = 0;
	int addr_offset = 0;
	int value = 0;
	int error = 0;
	int i = 0, j = 0;
	u8 *obuf = NULL;
	u16 size = 0;
	u16 addr = 0;

	sscanf(buf, "%s %d %d %d", command, &type, &addr_offset, &value);

	if (!strncmp(command, "mode", 4)) { /*mode*/
		TOUCH_INFO_MSG("Mode changed MODE: %d\n", type);
		data->mxt_mode_changed = type;
		if (data->mxt_mode_changed)
			mxt_write_object(data,
					MXT_PROCG_NOISESUPPRESSION_T72, 0, 1);
		else
			mxt_write_object(data,
					MXT_PROCG_NOISESUPPRESSION_T72, 0, 11);
		return count;
	}

	obuf = kzalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	if (type == 25)
		selftest_enable = true;

	object = mxt_get_object(data, type);
	if (!object) {
		TOUCH_ERR_MSG("error Cannot get object_type T%d\n", type);
		error = -EINVAL;
		goto error;
	}

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		TOUCH_ERR_MSG("error object_type T%d\n", type);
		error = -ENODEV;
		goto error;
	}

	if (!strncmp(command, "read", 4)) {	/*read*/
		TOUCH_DEBUG_MSG("Object Read T%d: start_addr=%d,"
				" size=%d * instance=%d\n",
				type, object->start_address,
				mxt_obj_size(object),
				mxt_obj_instances(object));

		for (j = 0; j < mxt_obj_instances(object); j++) {
			size = mxt_obj_size(object);
			addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error) {
				TOUCH_ERR_MSG("Object Read Fail\n");
				goto error;
			}
		}

		for (i = 0; i < mxt_obj_size(object)*mxt_obj_instances(object); i++)
			TOUCH_DEBUG_MSG("T%d [%d] %d[0x%x]\n",
					type, i, obuf[i], obuf[i]);

	} else if (!strncmp(command, "write", 4)) {	/*write*/
		TOUCH_DEBUG_MSG("Object Write T%d: start_addr=%d,"
				" size=%d * instance=%d\n",
				type, object->start_address,
				mxt_obj_size(object),
				mxt_obj_instances(object));

		error = mxt_write_reg(data->client,
				object->start_address+addr_offset, value);
		if (error) {
			TOUCH_ERR_MSG("Object Write Fail\n");
			goto error;
		}

		TOUCH_INFO_MSG("Object Write Success."
			       " Execute Read Object and Check Value.\n");
	} else {
		TOUCH_ERR_MSG("Usage: echo [read | write] object cmd_field"
			      " value > object_ctrl\n");
		error = -EINVAL;
		goto error;
	}

	error = count;

error:
	kfree(obuf);
	return error;
}

static int mxt_check_firmware_format(struct device *dev,
		const struct firmware *fw)
{
	unsigned int pos = 0;
	char c = 0;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/* To convert file try
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw */
	TOUCH_ERR_MSG("Aborting: firmware file must be in binary format\n");

	return -EINVAL;
}

static int mxt_load_bin(struct device *dev, const char *name)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	unsigned int frame_size = 0;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret = 0;

	if (!name) {
		TOUCH_ERR_MSG("Skipping bin download \n");
		return 0;
	}

	ret = request_firmware(&fw, name, dev);
	if (ret) {
		TOUCH_ERR_MSG("Unable to open bin [%s]  ret %d\n", name, ret);
		return ret;
	}
	TOUCH_DEBUG_MSG("Open bin [%s]\n", name);

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret)
		goto release_firmware;

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);

		touch_enable_irq(data->irq);
		data->suspended = false;
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_BOOT_VALUE,
				false);
		if (ret)
			goto release_firmware;

		msleep(MXT_RESET_TIME);

		/* At this stage, do not need to scan since we know
		 * family ID */
		ret = mxt_lookup_bootloader_address(data, 0);
		if (ret)
			goto release_firmware;
	}

	mxt_free_object_table(data);
	reinit_completion(&data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD);
	if (ret) {
		/* Bootloader may still be unlocked from previous update
		 * attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;
	} else {
		TOUCH_INFO_MSG("Unlocking bootloader\n");

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			goto disable_irq;
	}

	while (pos < fw->size) {
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, fw->data + pos, frame_size);
		if (ret)
			goto disable_irq;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS);
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				TOUCH_ERR_MSG("Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (frame % 50 == 0)
			TOUCH_DEBUG_MSG("Sent %d frames, %d/%zd bytes\n",
				 frame, pos, fw->size);
	}

	/* Wait for flash */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
				      MXT_FW_RESET_TIME);
	if (ret)
		goto disable_irq;
	TOUCH_DEBUG_MSG("Sent %d frames, %u bytes\n", frame, pos);

	data->in_bootloader = false;

disable_irq:
	touch_disable_irq(data->irq);
release_firmware:
	if (fw)
		release_firmware(fw);
	return ret;
}

static ssize_t mxt_update_bin_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int error = 0;
	char *name = NULL;

	error = mxt_update_file_name(&data->client->dev, &name, buf, count);
	if (error)
		return error;

	error = mxt_load_bin(&data->client->dev, name);
	if (error) {
		TOUCH_ERR_MSG("The bin update failed(%d)\n", error);
		return error;
	}

	data->suspended = false;

	mxt_hw_reset(data);

	error = mxt_read_info_block(data);
	if (error)
		return error;

	error = mxt_rest_init(data);
	if (error)
		return error;

	TOUCH_DEBUG_MSG("The bin update succeeded\n");
	return count;
}

static ssize_t mxt_update_raw_store(struct mxt_data *data, const char *buf,
		size_t count)
{
	int ret = 0;
	int value = 0;
	char *name = NULL;

	sscanf(buf, "%d", &value);
	TOUCH_DEBUG_MSG("Update mxt Configuration.\n");

	if (data->in_bootloader) {
		TOUCH_ERR_MSG("Not in appmode\n");
		return -EINVAL;
	}

	ret = mxt_update_file_name(&data->client->dev, &name, buf, count);
	if (ret)
		return ret;

	data->enable_reporting = false;

	ret = mxt_check_reg_init(data, name);
	if (ret < 0) {
		TOUCH_ERR_MSG("Error mxt_check_reg_init ret=%d\n", ret);
		goto out;
	}

	TOUCH_DEBUG_MSG("Update mxt Configuration Success.\n");

	ret = count;
out:
	data->enable_reporting = true;

	return ret;
}

static ssize_t mxt_update_fw_store(struct mxt_data *data, const char *buf,
		size_t count)
{
	char *package_name = NULL;
	int error = 0;
	int wait_cnt = 0;

	TOUCH_DEBUG_MSG("%s \n", __func__);

	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(2000));

	if (data->suspended) {
		TOUCH_DEBUG_MSG("Try LCD On\n");
		kobject_uevent_env(&device_touch.kobj, KOBJ_CHANGE,
				knockon_event);
		while (1) {
			if (data->suspended) {
				mdelay(100);
				wait_cnt++;
			}

			if (!data->suspended || wait_cnt > 50)
				break;
		}
	}

	TOUCH_DEBUG_MSG("wait_cnt = %d\n", wait_cnt);

	touch_disable_irq(data->irq);

	error = mxt_update_file_name(&data->client->dev,
			&package_name, buf, count);
	if (error) {
		TOUCH_ERR_MSG("%s error package_name [%s] \n", __func__,
				package_name);
		goto exit;
	}

	error = mxt_update_firmware(data, package_name);
	if (error) {
		TOUCH_ERR_MSG("%s error \n", __func__);
		goto exit;
	}

	if (data->T100_reportid_min) {
		error = mxt_read_t100_config(data);
		if (error) {
			TOUCH_ERR_MSG("Failed to initialize T100 resolution\n");
			goto exit;
		}
	} else {
		TOUCH_ERR_MSG("Failed to read touch object\n");
		goto exit;
	}

	error = count;
exit:
	if (package_name)
		kfree(package_name);

	if (mxt_patchevent_get(PATCH_EVENT_TA))
		trigger_usb_state_from_otg(1);

	touch_enable_irq(data->irq);

	mxt_read_fw_version(data);

	return error;
}

static ssize_t mxt_check_fw_store(struct mxt_data *data, const char *buf,
		size_t count)
{
	unsigned int input = 0;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input)
		queue_delayed_work(touch_wq, &data->work_firmware_update, 0);

	return count;
}


static ssize_t mxt_debug_enable_show(struct mxt_data *data, char *buf)
{
	int count = 0;
	char c = 0;

	c = data->debug_enabled ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_debug_enable_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		TOUCH_INFO_MSG("%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	}

	TOUCH_ERR_MSG("debug_enabled write error\n");
	return -EINVAL;
}

static ssize_t mxt_t57_debug_enable_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->t57_debug_enabled = (i == 1);

		TOUCH_DEBUG_MSG("%s\n", i ?
				"t57 debug enabled" : "t57 debug disabled");
		return count;
	}

	TOUCH_ERR_MSG("t57_debug_enabled write error\n");
	return -EINVAL;
}

static ssize_t mxt_patch_debug_enable_show(struct mxt_data *data, char *buf)
{
	int count = 0;
	char c = 0;

	if (data->patch.patch == NULL) {
		TOUCH_ERR_MSG("patch not support \n");
		return -ENOTSUPP;
	}

	c = data->patch.debug ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_patch_debug_enable_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int i = 0;

	if (data->patch.patch == NULL) {
		TOUCH_ERR_MSG("patch not support \n");
		return -ENOTSUPP;
	}

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->patch.debug = (i == 1);

		TOUCH_INFO_MSG("%s\n", i ?
			"patch debug enabled" : "patch debug disabled");
		return count;
	}

	TOUCH_ERR_MSG("patch_debug_enabled write error\n");
	return -EINVAL;
}

static ssize_t mxt_power_control_show(struct mxt_data *data, char *buf)
{
	size_t ret = 0;

	ret += sprintf(buf+ret, "usage: echo [0|1|2|3] > power_control \n");
	ret += sprintf(buf+ret, "  0 : power off \n");
	ret += sprintf(buf+ret, "  1 : power on \n");
	ret += sprintf(buf+ret, "  2 : reset by I2C \n");
	ret += sprintf(buf+ret, "  3 : reset by reset_gpio \n");

	return ret;
}

static ssize_t mxt_power_control_store(struct mxt_data *data, const char *buf,
		size_t count)
{
	int cmd = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;
	switch (cmd) {
		case 0:
			mxt_regulator_disable(data);
			break;
		case 1:
			mxt_regulator_enable(data);
			mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
			break;
		case 2:
			mxt_soft_reset(data);
			break;
		case 3:
			mxt_hw_reset(data);
			break;
		default:
			TOUCH_INFO_MSG("usage: echo [0|1|2|3] > power_control \n");
			TOUCH_INFO_MSG("  0 : power off \n");
			TOUCH_INFO_MSG("  1 : power on \n");
			TOUCH_INFO_MSG("  2 : reset by I2C \n");
			TOUCH_INFO_MSG("  3 : reset by reset_gpio \n");
			break;
	}
	return count;

}

static ssize_t mxt_global_access_pixel_show(struct mxt_data *data, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"%d\n", data->pdata->global_access_pixel);

	return len;
}

static ssize_t mxt_global_access_pixel_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	TOUCH_DEBUG_MSG("%s = %d \n", __func__, value);

	data->pdata->global_access_pixel = value;

	return count;
}

static ssize_t mxt_force_rebase_show(struct mxt_data *data, char *buf)
{
	ssize_t len = 0;

	if (data->pdata->panel_on == POWER_OFF) {
		wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(2000));
	}

	TOUCH_DEBUG_MSG("MXT_COMMAND_CALIBRATE \n");
	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);

	return len;
}

static ssize_t mxt_mfts_enable_show(struct mxt_data *data, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", data->mfts_enable);

	return len;
}

static ssize_t mxt_mfts_enable_store(struct mxt_data *data, const char *buf,
		size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	TOUCH_INFO_MSG("%s = %d \n", __func__, value);

	data->mfts_enable = value;

	/* Touch IC Reset for Initial configration. */
	mxt_soft_reset(data);

	/* Calibrate for Active touch IC */
	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);

	return count;
}

static ssize_t mxt_show_self_ref(struct mxt_data *data, char *buf)
{
	mxt_get_self_reference_chk(data);
	return 0;
}

static ssize_t mxt_self_cap_show(struct mxt_data *data, char *buf)
{
	int len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", data->self_cap);
	return len;
}

static ssize_t mxt_self_cap_store(struct mxt_data *data, const char *buf,
		size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);
	TOUCH_DEBUG_MSG("%s : %d \n", __func__, value);
	if (value  == 1){
		data->self_cap = value;
		TOUCH_DEBUG_MSG(" Noise suppression \n");
		mxt_patch_event(global_mxt_data,SELF_CAP_OFF_NOISE_SUPPRESSION );
	} else if (value  == 0){
		data->self_cap = value;
		TOUCH_DEBUG_MSG(" Noise recover \n");
		mxt_patch_event(global_mxt_data,SELF_CAP_ON_NOISE_RECOVER );
	} else {
		TOUCH_DEBUG_MSG(" Do nothing\n");
	}
	return count;
}

static ssize_t mxt_noise_suppression_show(struct mxt_data *data, char *buf)
{
	int len = 0;

	data->self_cap = 1;
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", data->self_cap);
	TOUCH_DEBUG_MSG("%s : %d \n", __func__, data->self_cap);
	TOUCH_DEBUG_MSG(" Noise suppression \n");
	mxt_patch_event(global_mxt_data,SELF_CAP_OFF_NOISE_SUPPRESSION );

	return len;
}

static ssize_t mxt_noise_suppression_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);
	TOUCH_DEBUG_MSG("%s : %d \n", __func__, value);
	if (value  == 1){
		data->self_cap = value;
		TOUCH_DEBUG_MSG(" Noise suppression \n");
		mxt_patch_event(global_mxt_data,SELF_CAP_OFF_NOISE_SUPPRESSION );
	} else if (value  == 0){
		data->self_cap = value;
		TOUCH_DEBUG_MSG(" Noise recover \n");
		mxt_patch_event(global_mxt_data,SELF_CAP_ON_NOISE_RECOVER );
	} else {
		TOUCH_DEBUG_MSG(" Do nothing\n");
	}
	return count;
}

static ssize_t mxt_noise_recover_show(struct mxt_data *data, char *buf)
{
	int len = 0;
	data->self_cap = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", data->self_cap);
	TOUCH_DEBUG_MSG("%s : %d \n", __func__, data->self_cap);
	TOUCH_DEBUG_MSG(" Noise suppression recovered \n");
	mxt_patch_event(global_mxt_data,SELF_CAP_ON_NOISE_RECOVER );
	return len;
}

static ssize_t mxt_noise_recover_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);
	data->self_cap = value;
	TOUCH_DEBUG_MSG("%s : %d \n", __func__, value);
	if (value  == 1){
		data->self_cap = value;
		TOUCH_DEBUG_MSG(" Noise suppression \n");
		mxt_patch_event(global_mxt_data,SELF_CAP_OFF_NOISE_SUPPRESSION );
	} else if (value  == 0){
		data->self_cap = value;
		TOUCH_DEBUG_MSG(" Noise recover \n");
		mxt_patch_event(global_mxt_data,SELF_CAP_ON_NOISE_RECOVER );
	} else {
		TOUCH_DEBUG_MSG(" Do nothing\n");
	}
	return count;
}

static MXT_TOUCH_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static MXT_TOUCH_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static MXT_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, mxt_testmode_ver_show, NULL);
static MXT_TOUCH_ATTR(version, S_IRUGO, mxt_info_show, NULL);
static MXT_TOUCH_ATTR(mxt_info, S_IRUGO, mxt_info_show, NULL);
static MXT_TOUCH_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static MXT_TOUCH_ATTR(object_ctrl, S_IWUSR, NULL, mxt_object_control);
static MXT_TOUCH_ATTR(update_bin, S_IWUSR, NULL, mxt_update_bin_store);
static MXT_TOUCH_ATTR(update_raw, S_IWUSR, NULL, mxt_update_raw_store);
static MXT_TOUCH_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show, mxt_debug_enable_store);
static MXT_TOUCH_ATTR(t57_debug_enable, S_IWUSR | S_IRUSR, NULL, mxt_t57_debug_enable_store);
static MXT_TOUCH_ATTR(patch_debug_enable, S_IWUSR | S_IRUSR, mxt_patch_debug_enable_show, mxt_patch_debug_enable_store);
static MXT_TOUCH_ATTR(update_patch, S_IWUSR, NULL, mxt_update_patch_store);
static MXT_TOUCH_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static MXT_TOUCH_ATTR(check_fw, S_IWUSR, NULL, mxt_check_fw_store);
static MXT_TOUCH_ATTR(power_control, S_IRUGO | S_IWUSR, mxt_power_control_show, mxt_power_control_store);
static MXT_TOUCH_ATTR(global_access_pixel, S_IWUSR | S_IRUSR, mxt_global_access_pixel_show, mxt_global_access_pixel_store);
static MXT_TOUCH_ATTR(rebase, S_IWUSR | S_IRUGO, mxt_force_rebase_show, NULL);
static MXT_TOUCH_ATTR(mfts, S_IWUSR | S_IRUSR, mxt_mfts_enable_show, mxt_mfts_enable_store);
static MXT_TOUCH_ATTR(self_ref_check, S_IRUGO | S_IWUSR, mxt_show_self_ref, NULL);
static MXT_TOUCH_ATTR(self_cap, S_IWUSR | S_IRUGO, mxt_self_cap_show, mxt_self_cap_store);
static MXT_TOUCH_ATTR(noise_suppression, S_IWUSR | S_IRUGO, mxt_noise_suppression_show, mxt_noise_suppression_store);
static MXT_TOUCH_ATTR(noise_recover, S_IWUSR | S_IRUGO, mxt_noise_recover_show, mxt_noise_recover_store);
static struct attribute *mxt_touch_attribute_list[] = {
	&mxt_touch_attr_fw_version.attr,
	&mxt_touch_attr_hw_version.attr,
	&mxt_touch_attr_testmode_ver.attr,
	&mxt_touch_attr_version.attr,
	&mxt_touch_attr_mxt_info.attr,
	&mxt_touch_attr_object.attr,
	&mxt_touch_attr_object_ctrl.attr,
	&mxt_touch_attr_update_bin.attr,
	&mxt_touch_attr_update_raw.attr,
	&mxt_touch_attr_debug_enable.attr,
	&mxt_touch_attr_t57_debug_enable.attr,
	&mxt_touch_attr_patch_debug_enable.attr,
	&mxt_touch_attr_update_patch.attr,
	&mxt_touch_attr_update_fw.attr,
	&mxt_touch_attr_check_fw.attr,
	&mxt_touch_attr_power_control.attr,
	&mxt_touch_attr_global_access_pixel.attr,
	&mxt_touch_attr_rebase.attr,
	&mxt_touch_attr_mfts.attr,
	&mxt_touch_attr_self_ref_check.attr,
	&mxt_touch_attr_self_cap.attr,
	&mxt_touch_attr_noise_suppression.attr,
	&mxt_touch_attr_noise_recover.attr,
	NULL
};

static ssize_t mxt_touch_attr_show(struct kobject *mxt_touch_kobj,
		struct attribute *attr, char *buf)
{
	struct mxt_data *ts = container_of(
			mxt_touch_kobj, struct mxt_data, mxt_touch_kobj);
	struct mxt_touch_attribute *mxt_touch_priv =
		container_of(attr, struct mxt_touch_attribute, attr);
	ssize_t ret = 0;

	if (mxt_touch_priv->show)
		ret = mxt_touch_priv->show(ts, buf);

	return ret;
}

static ssize_t mxt_touch_attr_store(struct kobject *mxt_touch_kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *ts = container_of(
			mxt_touch_kobj, struct mxt_data, mxt_touch_kobj);
	struct mxt_touch_attribute *mxt_touch_priv =
		container_of(attr, struct mxt_touch_attribute, attr);
	ssize_t ret = 0;

	if (mxt_touch_priv->store)
		ret = mxt_touch_priv->store(ts, buf, count);

	return ret;
}

static const struct sysfs_ops mxt_touch_sysfs_ops = {
	.show	= mxt_touch_attr_show,
	.store	= mxt_touch_attr_store,
};

static struct kobj_type mxt_touch_kobj_type = {
	.sysfs_ops		= &mxt_touch_sysfs_ops,
	.default_attrs 	= mxt_touch_attribute_list,
};

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	int id = 0;

	for (id = 0; id < data->pdata->numtouch; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	mxt_input_sync(input_dev);
	TOUCH_INFO_MSG("Release all event \n");

	memset(data->ts_data.prev_data, 0x0, sizeof(data->ts_data.prev_data));
	memset(data->ts_data.curr_data, 0x0, sizeof(data->ts_data.curr_data));
	touched_finger_count = 0;
	data->button_lock = false;
	data->palm = false;
}

static void mxt_gesture_mode_start(struct mxt_data *data)
{
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (!object)
		return;

	mxt_patchevent_set(PATCH_EVENT_KNOCKON);

	if (mxt_patchevent_get(PATCH_EVENT_TA))
		mxt_patch_event(data, CHARGER_KNOCKON_SLEEP);
	else
		mxt_patch_event(data, NOCHARGER_KNOCKON_SLEEP);
}

static void mxt_active_mode_start(struct mxt_data *data)
{

	struct mxt_object *object;

	object = mxt_get_object(data, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (!object)
		return;

	if (mxt_patchevent_get(PATCH_EVENT_TA)) {
		if (data->mxt_knock_on_enable ||
		    mxt_patchevent_get(PATCH_EVENT_KNOCKON)) {
			mxt_patch_event(data, CHARGER_KNOCKON_WAKEUP);
		}
	} else {
		if (data->mxt_knock_on_enable ||
		    mxt_patchevent_get(PATCH_EVENT_KNOCKON)) {
			mxt_patch_event(data, NOCHARGER_KNOCKON_WAKEUP);
		}
	}

	mxt_patchevent_unset(PATCH_EVENT_KNOCKON);
}

static void mxt_start(struct mxt_data *data)
{
	if (!data->suspended || data->in_bootloader) {
		if(data->regulator_status == 1 && !data->in_bootloader) {
			TOUCH_DEBUG_MSG("%s suspended flag is false."
				" Calibration after System Shutdown.\n",
				__func__);
			mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
		}
		return;
	}

	TOUCH_INFO_MSG("%s \n", __func__);

	touch_disable_irq(data->irq);

	mxt_regulator_enable(data);

	mxt_active_mode_start(data);
	data->delayed_cal = true;

	mxt_reset_slots(data);
	data->suspended = false;
	data->button_lock = false;
	touch_enable_irq(data->irq);
}

static void mxt_stop(struct mxt_data *data)
{
	if (data->suspended || data->in_bootloader)
		return;

	TOUCH_INFO_MSG("%s \n", __func__);

	touch_disable_irq(data->irq);
	touch_disable_irq_wake(data->irq);

	if (data->mxt_knock_on_enable) {
		mxt_gesture_mode_start(data);
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
	} else {
		TOUCH_INFO_MSG("%s MXT_POWER_CFG_DEEPSLEEP\n", __func__);
		// mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
		mxt_regulator_disable(data);
	}

	mxt_reset_slots(data);
	data->suspended = true;
	data->button_lock = false;

	if (data->mxt_knock_on_enable) {
		touch_enable_irq(data->irq);
		touch_enable_irq_wake(data->irq);
	}
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}

static int mxt_parse_dt(struct device *dev, struct mxt_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	int rc = 0;
	u32 temp_val = 0;

	/* reset, irq gpio info */
	pdata->gpio_reset = of_get_named_gpio_flags(node,
			"atmel,reset-gpio", 0, NULL);
	if (pdata->gpio_reset < 0) {
		TOUCH_ERR_MSG("DT get gpio_reset error \n");
		return pdata->gpio_reset;
	}
	TOUCH_DEBUG_MSG("DT : gpio_reset = %lu\n", pdata->gpio_reset);

	pdata->gpio_int = of_get_named_gpio_flags(node,
			"atmel,irq-gpio", 0, NULL);
	if (pdata->gpio_int < 0) {
		TOUCH_ERR_MSG("DT get gpio_int error \n");
		return pdata->gpio_int;
	}
	TOUCH_DEBUG_MSG("DT : gpio_int = %lu\n", pdata->gpio_int);

	pdata->gpio_id = of_get_named_gpio_flags(node,
			"atmel,id-gpio", 0, NULL);
	if (pdata->gpio_id < 0)
		TOUCH_ERR_MSG("DT get gpio_id error \n");
	else
		TOUCH_DEBUG_MSG("DT : gpio_id = %lu\n", pdata->gpio_id);

	rc = of_property_read_u32(node, "atmel,numtouch", &temp_val);
	if (rc) {
		TOUCH_ERR_MSG("DT : Unable to read numtouch\n");
	} else {
		pdata->numtouch = temp_val;
		TOUCH_DEBUG_MSG("DT : numtouch = %d\n", pdata->numtouch);
	}

	rc = of_property_read_string(node, "atmel,fw_name",  &pdata->fw_name);
	if (rc) {
		TOUCH_ERR_MSG("DT : atmel,fw_name error \n");
		return rc;
	}
	TOUCH_DEBUG_MSG("DT : fw_name : %s \n", pdata->fw_name);

	rc = of_property_read_u32(node, "atmel,ref_reg_weight_val", &temp_val);
	if (rc) {
		pdata->ref_reg_weight_val = 16;
	} else {
		pdata->ref_reg_weight_val = temp_val;
		TOUCH_DEBUG_MSG("DT : ref_reg_weight_val = %d\n",
				pdata->ref_reg_weight_val);
	}

	rc = of_property_read_u32(node, "atmel,global_access_pixel",
			&temp_val);
	if (rc) {
		TOUCH_ERR_MSG("DT : Unable to read global_access_pixel - set as 0\n");
		pdata->global_access_pixel = 0;
	} else {
		pdata->global_access_pixel = temp_val;
		TOUCH_DEBUG_MSG("DT : global_access_pixel = %d \n",
				pdata->global_access_pixel);
	}

	rc = of_property_read_u32(node, "atmel,use_mfts",  &temp_val);
	if (rc) {
		TOUCH_ERR_MSG("DT : Unable to read use_mfts - set as false \n" );
		pdata->use_mfts = 0;
	} else {
		pdata->use_mfts = temp_val;
		TOUCH_DEBUG_MSG("DT : use_mfts = %d \n",pdata->use_mfts);
	}

	rc = of_property_read_u32(node, "atmel,lcd_x", &temp_val);
	if (rc) {
		TOUCH_ERR_MSG( "DT : Unable to read lcd_x\n");
		pdata->lcd_x = 540;
	} else {
		pdata->lcd_x = temp_val;
		TOUCH_DEBUG_MSG("DT : lcd_x: %d\n",pdata->lcd_x);
	}

	rc = of_property_read_u32(node, "atmel,lcd_y", &temp_val);
	if (rc) {
		TOUCH_ERR_MSG( "DT : Unable to read lcd_y\n");
		pdata->lcd_y = 960;
	} else {
		pdata->lcd_y = temp_val;
		TOUCH_DEBUG_MSG("DT : lcd_y: %d\n",pdata->lcd_y);
	}

	pdata->palm_enabled = of_property_read_bool(node,
			"atmel,palm-enabled");
	TOUCH_DEBUG_MSG("DT : palm %s\n",
			pdata->palm_enabled? "enabled" : "disabled");

	return 0;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x = 0, range_y = 0;
	u8 cfg = 0, tchaux = 0;
	u8 aux = 0;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object) {
		TOUCH_ERR_MSG("Couldn't get object\n");
		return -EINVAL;
	}

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       (u16)sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       (u16)sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	error = __mxt_read_reg(data->client, data->T100_address + 8, 1,
			&data->channel_size.start_x);

	error |= __mxt_read_reg(data->client, data->T100_address + 9, 1,
			&data->channel_size.size_x);

	error |= __mxt_read_reg(data->client, data->T100_address + 19, 1,
			&data->channel_size.start_y);

	error |= __mxt_read_reg(data->client, data->T100_address + 20, 1,
			&data->channel_size.size_y);

	if (!error) {
		TOUCH_DEBUG_MSG("Succeed to read channel_size %d %d %d %d \n",
			data->channel_size.start_x,
			data->channel_size.start_y,
			data->channel_size.size_x,
			data->channel_size.size_y);
	}

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;
	else
		data->t100_aux_ampl = aux;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	if (tchaux & MXT_T100_TCHAUX_RESV)
		data->t100_aux_resv = aux++;

	TOUCH_DEBUG_MSG("T100 Touchscreen size X%uY%u\n",
			data->max_x, data->max_y);

	if (data->input_dev) {
		struct input_dev *input_dev = data->input_dev;

		/* For multi touch */
		error = input_mt_init_slots(input_dev, data->num_touchids,
				INPUT_MT_DIRECT);
		if (error) {
			TOUCH_ERR_MSG("Error %d initialising slots\n", error);
			return error;
		}

		input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
				data->pdata->numtouch, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0,
				MXT_MAX_AREA, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0,
				MXT_MAX_AREA, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
				data->max_x, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
				data->max_y, 0, 0);

		if (data->t100_aux_area)
			input_set_abs_params(input_dev,
				ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_AREA, 0, 0);

		if (data->t100_aux_ampl)
			input_set_abs_params(input_dev,
				ABS_MT_PRESSURE, 0, 255, 0, 0);

		if (data->t100_aux_vect)
			input_set_abs_params(input_dev,
				ABS_MT_ORIENTATION, 0, 255, 0, 0);
	}

	return 0;
}

int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	struct input_dev *input_dev;
	int error;

	if (data->input_dev) {
		TOUCH_WARN_MSG("Input device already registered\n");
		return 0;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		TOUCH_ERR_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = MXT_TOUCH_NAME;
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_SLEEP, input_dev->keybit);

	/* For multi touch */
	error = input_mt_init_slots(input_dev, data->num_touchids,
			INPUT_MT_DIRECT);
	if (error) {
		TOUCH_ERR_MSG("Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
			data->pdata->numtouch, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0,
			MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0,
			MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			data->max_y, 0, 0);

	if (data->t100_aux_area)
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0,
				MXT_MAX_AREA, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (data->t100_aux_vect)
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		TOUCH_ERR_MSG("Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct input_dev *input_dev = NULL;
	int error = 0;
	unsigned int num_mt_slots = 0;

	if (data->input_dev) {
		TOUCH_ERR_MSG("ignore %s \n", __func__);
		return 0;
	}

	error = mxt_read_t9_resolution(data);
	if (error)
		TOUCH_ERR_MSG("Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		TOUCH_ERR_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = MXT_TOUCH_NAME;
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	num_mt_slots = data->num_touchids;
	error = input_mt_init_slots(input_dev, num_mt_slots, 0);
	if (error) {
		TOUCH_ERR_MSG("Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y, 0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION, 0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		TOUCH_ERR_MSG("Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	TOUCH_INFO_MSG("input_register_device done\n");

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_command_reset(struct mxt_data *data, u8 value)
{
	int error = 0;

	error = mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_RESET, value);
	msleep(MXT_RESET_TIME);
	if (error)
		TOUCH_ERR_MSG("Not respond after reset command[%d]\n", value);

	return error;
}

static int mxt_command_backup(struct mxt_data *data, u8 value)
{
	mxt_write_object(data, MXT_GEN_COMMAND_T6,MXT_COMMAND_BACKUPNV, value);
	msleep(MXT_BACKUP_TIME);

	return 0;
}

int mxt_read_mem(struct mxt_data *data, u16 reg, u16 len, void *buf)
{
	return __mxt_read_reg(data->client, reg, len, buf);
}

int mxt_write_mem(struct mxt_data *data, u16 reg, u16 len, const u8 *buf)
{
	return __mxt_write_reg(data->client, reg, len, buf);
}

int mxt_verify_fw(struct mxt_fw_info *fw_info, const struct firmware *fw)
{
	struct mxt_data *data = fw_info->data;
	struct mxt_fw_image *fw_img = NULL;
	char *extra_info = NULL;
	struct patch_header* ppheader = NULL;
	u8* patch = NULL;
	u32 ppos = 0;

	if (!fw) {
		TOUCH_ERR_MSG("could not find firmware file\n");
		return -ENOENT;
	}

	fw_img = (struct mxt_fw_image *)fw->data;

	if (le32_to_cpu(fw_img->magic_code) != MXT_FW_MAGIC) {
		/* In case, firmware file only consist of firmware */
		TOUCH_ERR_MSG("Firmware file only consist of raw firmware\n");
		fw_info->fw_len = fw->size;
		fw_info->fw_raw_data = fw->data;
	} else {
		/*
		 * In case, firmware file consist of header,
		 * configuration, firmware.
		 */
		TOUCH_INFO_MSG("Firmware file consist of header,"
			       " configuration, firmware\n");
		fw_info->bin_ver = fw_img->bin_ver;
		fw_info->build_ver = fw_img->build_ver;
		fw_info->hdr_len = le32_to_cpu(fw_img->hdr_len);
		fw_info->cfg_len = le32_to_cpu(fw_img->cfg_len);
		fw_info->fw_len = le32_to_cpu(fw_img->fw_len);
		fw_info->cfg_crc = le32_to_cpu(fw_img->cfg_crc);

		extra_info = fw_img->extra_info;
		fw_info->data->pdata->fw_ver[0] = extra_info[0];
		fw_info->data->pdata->fw_ver[1] = extra_info[1];
		fw_info->data->pdata->fw_ver[2] = extra_info[2];
		memcpy(fw_info->data->pdata->product, &extra_info[4], 10);

		/* Check the firmware file with header */
		if (fw_info->hdr_len != sizeof(struct mxt_fw_image)
			|| fw_info->hdr_len + fw_info->cfg_len +
			   fw_info->fw_len != fw->size) {

			ppos = fw_info->hdr_len + fw_info->cfg_len +
				fw_info->fw_len;
			ppheader = (struct patch_header*)(fw->data + ppos);
			if (ppheader->magic == MXT_PATCH_MAGIC) {
				TOUCH_INFO_MSG("Firmware file has patch size:"
					       " %d\n", ppheader->size);
				if (ppheader->size) {
					patch = NULL;
					if (data->patch.patch) {
						kfree(data->patch.patch);
						data->patch.patch = NULL;
					}
					patch = kzalloc(ppheader->size,
							GFP_KERNEL);
					if (patch) {
						memcpy(patch, (u8*)ppheader,
								ppheader->size);
						data->patch.patch = patch;
						TOUCH_INFO_MSG(
							"%s Patch Updated\n",
							__func__);
					} else {
						data->patch.patch = NULL;
						TOUCH_ERR_MSG(
							"%s Patch Update"
							" Failed\n", __func__);
					}
				}
			} else {
				TOUCH_ERR_MSG("Firmware file is invaild !\n");
				return -EINVAL;
			}
		}

		if (!fw_info->cfg_len) {
			TOUCH_ERR_MSG("Firmware file dose not include"
				      " configuration data\n");
			return -EINVAL;
		}

		if (!fw_info->fw_len) {
			TOUCH_ERR_MSG("Firmware file dose not include raw"
				      " firmware data\n");
			return -EINVAL;
		}

		/* Get the address of configuration data */
		fw_info->cfg_raw_data = fw_img->data;

		/* Get the address of firmware data */
		fw_info->fw_raw_data = fw_img->data + fw_info->cfg_len;
	}

	return 0;
}

static int mxt_read_id_info(struct mxt_data *data)
{
	int ret = 0;
	u8 id[MXT_INFOMATION_BLOCK_SIZE] = {0};

	/* Read IC information */
	ret = mxt_read_mem(data, 0, MXT_INFOMATION_BLOCK_SIZE, id);
	if (ret) {
		TOUCH_ERR_MSG("Read fail. IC information\n");
		return ret;
	}

	if (data->info)
		kfree(data->info);

	data->info = kzalloc(sizeof(struct mxt_info), GFP_KERNEL);
	if (data->info) {
		data->info->family_id = id[0];
		data->info->variant_id = id[1];
		data->info->version = id[2];
		data->info->build = id[3];
		data->info->matrix_xsize = id[4];
		data->info->matrix_ysize = id[5];
		data->info->object_num = id[6];
	}
	TOUCH_INFO_MSG("Family:%02X Variant:%02X Binary:%u.%u.%02X"
		       " TX:%d RX:%d Objects:%d\n",
		data->info->family_id,
		data->info->variant_id,
		data->info->version >> 4,
		data->info->version & 0xF,
		data->info->build,
		data->info->matrix_xsize,
		data->info->matrix_ysize,
		data->info->object_num);

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error = 0;
	int i = 0;
	u16 reg = 0;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_TABLE_ELEMENT_SIZE] = {0};
	struct mxt_object *object = NULL;

	if (!data->info || !data->object_table) {
		TOUCH_ERR_MSG("%s() info, object_table is NULL \n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		reg = MXT_OBJECT_TABLE_START_ADDRESS +
			(MXT_OBJECT_TABLE_ELEMENT_SIZE * i);
		error = mxt_read_mem(data, reg,
				MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);
		if (error) {
			TOUCH_ERR_MSG("%s mxt_read_mem error \n", __func__);
			return error;
		}

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		/* the real size of object is buf[3]+1 */
		object->size_minus_one = buf[3];
		/* the real instances of object is buf[4]+1 */
		object->instances_minus_one= buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
				(object->instances_minus_one+1);
			data->max_reportid = reportid;
		}
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	return 0;
}

static int mxt_enter_bootloader(struct mxt_data *data)
{
	int error = 0;

	if (data->object_table) {
		memset(data->object_table, 0x0,
			(MXT_OBJECT_NUM_MAX * sizeof(struct mxt_object)));
	} else {
		data->object_table = kzalloc(
			(MXT_OBJECT_NUM_MAX * sizeof(struct mxt_object)),
			GFP_KERNEL);
		if (!data->object_table) {
			TOUCH_ERR_MSG("%s Failed to allocate memory\n",
					__func__);
			return -ENOMEM;
		} else {
			memset(data->object_table, 0x0,
			(MXT_OBJECT_NUM_MAX * sizeof(struct mxt_object)));
		}
	}

	/* Get object table information*/
	error = mxt_get_object_table(data);
	if (error)
		goto err_free_mem;

	/* Change to the bootloader mode */
	error = mxt_command_reset(data, MXT_BOOT_VALUE);
	if (error)
		goto err_free_mem;

err_free_mem:
	kfree(data->object_table);
	data->object_table = NULL;
	return error;
}

static int mxt_flash_fw(struct mxt_fw_info *fw_info)
{
	struct mxt_data *data = fw_info->data;
	const u8 *fw_data = fw_info->fw_raw_data;
	u32 fw_size = fw_info->fw_len;
	unsigned int frame_size = 0;
	unsigned int frame = 0;
	unsigned int pos = 0;
	int ret = 0;

	if (!fw_data) {
		TOUCH_ERR_MSG("%s firmware data is Null\n", __func__);
		return -ENOMEM;
	}

	/* T1664 use 0x26 bootloader addr */
	ret = mxt_lookup_bootloader_address(data, 1);
	if (ret) {
		TOUCH_ERR_MSG("Failed to lookup bootloader address\n");
		return ret;
	}

	reinit_completion(&data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD);
	if (ret) {
		/*may still be unlocked from previous update attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret) {
			TOUCH_ERR_MSG("Failed to check bootloader\n");
			goto out;
		}
	} else {
		TOUCH_INFO_MSG("Unlocking bootloader\n");
		/* Unlock bootloader */
		//mxt_unlock_bootloader(client);
		mxt_send_bootloader_cmd(data, true);
	}

	while (pos < fw_size) {
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret) {
			TOUCH_ERR_MSG("Fail updating firmware."
				      " wating_frame_data err\n");
			goto out;
		}

		frame_size = ((*(fw_data + pos) << 8) | *(fw_data + pos + 1));

		/*
		* We should add 2 at frame size as the the firmware data is not
		* included the CRC bytes.
		*/

		frame_size += 2;

		/* Write one frame to device */
		mxt_bootloader_write(data, fw_data + pos, frame_size);

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS);
		if (ret) {
			TOUCH_INFO_MSG("Fail updating firmware."
				       " frame_crc err\n");
			goto out;
		}

		pos += frame_size;
		frame++;

		if (frame % 50 == 0)
			TOUCH_DEBUG_MSG("\t Updated %5d / %5d bytes\n",
					pos, fw_size);

		msleep(20);
	}

	msleep(MXT_FW_RESET_TIME);

out:
	return ret;
}

static int mxt_flash_fw_on_probe(struct mxt_fw_info *fw_info)
{
	struct mxt_data *data = fw_info->data;
	int error = 0;

	printk("[touch] mxt_flash_fw_on_probe\n");
	error = mxt_read_id_info(data);

	if (error) {
		/* need to check IC is in boot mode */
		/* T1664 use 0x26 bootloader Addr */
		error = mxt_probe_bootloader(data, 1);
		if (error) {
			TOUCH_ERR_MSG("Failed to verify bootloader's status\n");
			goto out;
		}

		TOUCH_INFO_MSG("Updating firmware from boot-mode\n");
		goto load_fw;
	}

	/* compare the version to verify necessity of firmware updating */
	TOUCH_INFO_MSG("Binary Version [IC:%u.%u.%02X] [FW:%u.%u.%02X]\n",
		data->info->version >> 4, data->info->version & 0xF,
		data->info->build, fw_info->bin_ver >> 4,
		fw_info->bin_ver & 0xF, fw_info->build_ver);

	if (data->info->version == fw_info->bin_ver &&
	    data->info->build == fw_info->build_ver) {
		TOUCH_INFO_MSG("Binary Version is same \n");
		goto out;
	}

	error = mxt_enter_bootloader(data);
	if (error) {
		TOUCH_ERR_MSG("Failed enter bootloader mode\n");
		goto out;
	}

load_fw:
	error = mxt_flash_fw(fw_info);
	if (error)
		TOUCH_ERR_MSG("Failed updating firmware\n");
	else
		TOUCH_INFO_MSG("succeeded updating firmware\n");
out:
	return error;
}

static void mxt_make_reportid_table(struct mxt_data *data)
{
	struct mxt_object *objects = data->object_table;
	struct mxt_reportid *reportids = data->reportids;
	int i = 0, j = 0;
	int id = 0;

	for (i = 0; i < data->info->object_num; i++) {
		for (j = 0; j < objects[i].num_report_ids *
				(objects[i].instances_minus_one+1); j++) {
			id++;

			reportids[id].type = objects[i].type;
			reportids[id].index = j;

		}
	}
}

static int mxt_read_info_crc(struct mxt_data *data, u32 *crc_pointer)
{
	u16 crc_address = 0;
	u8 msg[3] = {0};
	int ret = 0;

	/* Read Info block CRC address */
	crc_address = MXT_OBJECT_TABLE_START_ADDRESS +
			data->info->object_num * MXT_OBJECT_TABLE_ELEMENT_SIZE;

	ret = mxt_read_mem(data, crc_address, 3, msg);
	if (ret)
		return ret;

	*crc_pointer = msg[0] | (msg[1] << 8) | (msg[2] << 16);

	return 0;
}

static int mxt_table_initialize(struct mxt_data *data)
{
	u32 read_info_crc = 0;
	int ret = 0;

	ret = mxt_read_id_info(data);
	if (ret)
		return ret;

	if (data->object_table)
		memset(data->object_table, 0x0,
			(MXT_OBJECT_NUM_MAX * sizeof(struct mxt_object)));
	else {
		data->object_table = kzalloc((MXT_OBJECT_NUM_MAX *
					sizeof(struct mxt_object)), GFP_KERNEL);
		if (!data->object_table) {
			TOUCH_ERR_MSG("%s Failed to allocate memory\n",
					__func__);
			ret = 1;
			goto out;
		} else {
			memset(data->object_table, 0x0,
				(MXT_OBJECT_NUM_MAX *
				 sizeof(struct mxt_object)));
		}
	}

	/* Get object table infomation */
	ret = mxt_get_object_table(data);
	if (ret)
		goto out;

	if (data->reportids)
		kfree(data->reportids);

	data->reportids = kzalloc(((data->max_reportid + 1) *
				sizeof(struct mxt_reportid)), GFP_KERNEL);
	if (!data->reportids) {
		TOUCH_ERR_MSG("%s Failed to allocate memory 2\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	/* Make report id table */
	mxt_make_reportid_table(data);

	/* Verify the info CRC */
	ret = mxt_read_info_crc(data, &read_info_crc);
	if (ret)
		goto out;

	return 0;
out:
	return ret;
}

static int mxt_read_message(struct mxt_data *data, struct mxt_message *message)
{
	struct mxt_object *object = NULL;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object) {
		TOUCH_ERR_MSG("mxt_read_message-mxt_get_object error\n");
		return -EINVAL;
	}

	return mxt_read_mem(data, object->start_address,
			sizeof(struct mxt_message), message);
}

static int mxt_read_message_reportid(struct mxt_data *data, struct mxt_message *message, u8 reportid)
{
	int try = 0;
	int error = 0;
	int fail_count = 0;

	fail_count = data->max_reportid * 2;

	while (++try < fail_count) {
		error = mxt_read_message(data, message);
		if (error) {
			TOUCH_ERR_MSG("mxt_read_message error\n");
			print_hex_dump(KERN_DEBUG, "CRC : ", DUMP_PREFIX_NONE, 16, 1,
				   message, sizeof(struct mxt_message), false);
			return error;
		}

		if (message->reportid == 0xff)
			continue;

		if (message->reportid == reportid)
			return 0;
	}

	return -EINVAL;
}

static int mxt_read_config_crc(struct mxt_data *data, u32 *crc)
{
	struct mxt_message message = {0};
	struct mxt_object *object = NULL;
	int error = 0;

	object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	if (!object)
		return -EIO;

	/* Try to read the config checksum of the existing cfg */
	mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_REPORTALL, 1);

	/* Read message from command processor, which only has one report ID */
	error = mxt_read_message_reportid(data, &message, 1);//data->max_reportid);
	if (error) {
		TOUCH_ERR_MSG("Failed to retrieve CRC\n");
		return error;
	}

	/* Bytes 1-3 are the checksum. */
	*crc = message.message[1] | (message.message[2] << 8) | (message.message[3] << 16);

	return 0;
}

static int mxt_write_config(struct mxt_fw_info *fw_info)
{
	struct mxt_data *data = fw_info->data;
	struct mxt_object *object = NULL;
	struct mxt_cfg_data *cfg_data = NULL;
	u32 current_crc = 0;
	u32 t38_cfg_crc = 0;
	u8 buf_crc_t38[3] = {0};
	u8 i = 0, val = 0;
	u16 reg = 0, index = 0;
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (!fw_info->cfg_raw_data) {
		TOUCH_INFO_MSG("No cfg data in file\n");
		return ret;
	}

	/* Get config CRC from device */
	ret = mxt_read_config_crc(data, &current_crc);
	if (ret) {
		TOUCH_INFO_MSG("fail to read config crc\n");
		return ret;
	}

	/* Check Version information */
	if (fw_info->bin_ver != data->info->version) {
		TOUCH_ERR_MSG("Warning: version mismatch! %s\n", __func__);
		return 0;
	}
	if (fw_info->build_ver != data->info->build) {
		TOUCH_ERR_MSG("Warning: build num mismatch! %s\n", __func__);
		return 0;
	}

	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);

	if(!object) {
		TOUCH_ERR_MSG("fail to get object\n");
		return 0;
	}

	ret = mxt_read_mem(data, object->start_address+3, 3, buf_crc_t38);
	if (ret) {
		TOUCH_ERR_MSG("T38 CRC read fail \n");
		return 0;
	}

	t38_cfg_crc = buf_crc_t38[2] << 16 |
		      buf_crc_t38[1] << 8 |
		      buf_crc_t38[0];
	TOUCH_DEBUG_MSG("T38 CRC[%06X] FW CRC[%06X]\n",
			t38_cfg_crc, fw_info->cfg_crc);

	/* Check config CRC */
	if (current_crc == fw_info->cfg_crc ||
	    t38_cfg_crc == fw_info->cfg_crc) {
		TOUCH_INFO_MSG("Same Config[%06X] Skip Writing\n",
				current_crc);
		return 0;
	}

	/* Restore memory and stop event handing */
	ret = mxt_command_backup(data, MXT_DISALEEVT_VALUE);
	if (ret) {
		TOUCH_ERR_MSG("Failed Restore NV and stop event\n");
		return -EINVAL;
	}

	TOUCH_INFO_MSG("Writing Config:[FW:%06X] [IC:%06X]\n",
			fw_info->cfg_crc, current_crc);
	/* Write config info */
	for (index = 0; index < fw_info->cfg_len; ) {
		if (index + (u16)sizeof(struct mxt_cfg_data) >=
				fw_info->cfg_len) {
			TOUCH_DEBUG_MSG("index(%d) of cfg_data exceeded total"
					" size(%d)!!\n",
				index + (u16)sizeof(struct mxt_cfg_data),
				fw_info->cfg_len);
			return -EINVAL;
		}

		/* Get the info about each object */
		cfg_data =
			(struct mxt_cfg_data *)(&fw_info->cfg_raw_data[index]);

		index += (u16)sizeof(struct mxt_cfg_data) + cfg_data->size;
		if (index > fw_info->cfg_len) {
			TOUCH_DEBUG_MSG("index(%d) of cfg_data exceeded total"
					" size(%d) in T%d object!!\n",
				index, fw_info->cfg_len, cfg_data->type);
			return -EINVAL;
		}

		object = mxt_get_object(data, cfg_data->type);
		if (!object) {
			TOUCH_ERR_MSG("T%d is Invalid object type\n",
					cfg_data->type);
			return -EINVAL;
		}

		/* Check and compare the size, instance of each object */
		if (cfg_data->size > (object->size_minus_one+1)) {
			TOUCH_ERR_MSG("T%d Object length exceeded!\n",
					cfg_data->type);
			return -EINVAL;
		}
		if (cfg_data->instance >= (object->instances_minus_one+1)) {
			TOUCH_ERR_MSG("T%d Object instances exceeded!\n",
					cfg_data->type);
			return -EINVAL;
		}

		TOUCH_DEBUG_MSG("\t Writing config for T%02d len %3d"
				" instance %d (%3d/%3d)\n",
			cfg_data->type, object->size_minus_one,
			cfg_data->instance, index, fw_info->cfg_len);

		reg = object->start_address + (object->size_minus_one+1) *
			cfg_data->instance;

		/* Write register values of each object */
		ret = mxt_write_mem(data, reg, cfg_data->size,
				cfg_data->register_val);
		if (ret) {
			TOUCH_ERR_MSG("Write T%d Object failed\n",
					object->type);
			return ret;
		}

		/*
		 * If firmware is upgraded, new bytes may be added to end of
		 * objects. It is generally forward compatible to zero these
		 * bytes - previous behaviour will be retained. However
		 * this does invalidate the CRC and will force a config
		 * download every time until the configuration is updated.
		 */
		if (cfg_data->size < (object->size_minus_one+1)) {
			TOUCH_ERR_MSG("Warning: zeroing %d byte(s) in T%d\n",
				 (object->size_minus_one+1) - cfg_data->size,
				 cfg_data->type);

			for (i = cfg_data->size + 1;
					i < (object->size_minus_one+1); i++) {
				ret = mxt_write_mem(data, reg + i, 1, &val);
				if (ret)
					return ret;
			}
		}
	}

	TOUCH_INFO_MSG("Configuration Updated \n");

	TOUCH_DEBUG_MSG("Restore CRC value \n");
	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!object) {
		TOUCH_ERR_MSG("fail to get object\n");
		return 0;
	}

	buf_crc_t38[0] = (u8)(fw_info->cfg_crc & 0x000000FF);
	buf_crc_t38[1] = (u8)((fw_info->cfg_crc & 0x0000FF00) >> 8);
	buf_crc_t38[2] = (u8)((fw_info->cfg_crc & 0x00FF0000) >> 16);

	ret = mxt_write_mem(data, object->start_address+3, 3, buf_crc_t38);
	if (ret) {
		TOUCH_ERR_MSG("Reference CRC Restore fail\n");
		return ret;
	}

	/* Backup to memory */
	ret = mxt_command_backup(data, MXT_BACKUP_VALUE);
	if (ret) {
		TOUCH_ERR_MSG("Failed backup NV data\n");
		return -EINVAL;
	}

	/* Soft reset */
	ret = mxt_command_reset(data, MXT_RESET_VALUE);
	if (ret) {
		TOUCH_ERR_MSG("Failed Reset IC\n");
		return -EINVAL;
	}

	return ret;
}

static int mxt_config_initialize(struct mxt_fw_info *fw_info)
{
	struct mxt_data *data = fw_info->data;
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	ret = mxt_write_config(fw_info);
	if (ret) {
		TOUCH_ERR_MSG("Failed to write config from file\n");
		goto out;
	}

	if (data->patch.patch) {
		ret = mxt_patch_init(data, data->patch.patch);
	}

	if (ret == 0) {
		global_mxt_data = data;
	} else {
		global_mxt_data = NULL;
		TOUCH_ERR_MSG("Failed to get global_mxt_data(NULL) \n");
	}

out:
	return ret;
}

int mxt_request_firmware_work(const struct firmware *fw, void *context)
{
	struct mxt_data *data = context;
	int error = 0;

	mxt_power_block(POWERLOCK_FW_UP);

	data->fw_info.data = data;
	if (fw) {
		error = mxt_verify_fw(&data->fw_info, fw);
		if (error)
			goto ts_rest_init;
	} else {
		goto out;
	}

	/* Skip update on boot up if firmware file does not have a header */
	if (!data->fw_info.hdr_len)
		goto ts_rest_init;

	error = mxt_flash_fw_on_probe(&data->fw_info);
	if (error)
		goto out;

ts_rest_init:
	error = mxt_table_initialize(data);
	if (error) {
		TOUCH_ERR_MSG("Failed to initialize\n");
		goto out;
	}

	error = mxt_config_initialize(&data->fw_info);
	if (error) {
		TOUCH_ERR_MSG("Failed to rest initialize\n");
		goto out;
	}

out:
	if(fw)
		release_firmware(fw);
	mxt_power_unblock(POWERLOCK_FW_UP);
	return error;
}

int mxt_update_firmware(struct mxt_data *data, const char *fwname)
{
	int error = 0;
	const struct firmware *fw = NULL;

	TOUCH_INFO_MSG("%s [%s]\n", __func__, fwname);

	if (fwname) {
		error = request_firmware(&fw, fwname, &data->client->dev);
		if (error) {
			TOUCH_ERR_MSG("%s error request_firmware \n", __func__);
			return 1;
		}
	}

	error = mxt_request_firmware_work(fw, data);
	if (error) {
		TOUCH_ERR_MSG("%s error mxt_request_firmware_work \n", __func__);
		return 1;
	}

	error = mxt_read_info_block(data);
	if (error) {
		TOUCH_ERR_MSG("%s error mxt_read_info_block \n", __func__);
		return 1;
	}

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		TOUCH_ERR_MSG("%s error mxt_init_t7_power_cfg \n", __func__);
		return 1;
	}

	return 0;
}

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mxt_data *data = NULL;
	struct mxt_platform_data *pdata;
	int error = 0;

	TOUCH_DEBUG_MSG("%s \n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TOUCH_ERR_MSG("i2c functionality check error\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		TOUCH_ERR_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	data->object_table = devm_kzalloc(&client->dev,
			(MXT_OBJECT_NUM_MAX * sizeof(struct mxt_object)),
			GFP_KERNEL);
	if (!data->object_table) {
		TOUCH_ERR_MSG("%s Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	/* read device tree */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct mxt_platform_data), GFP_KERNEL);
		if (!pdata) {
			TOUCH_ERR_MSG("Failed to allocate pdata memory\n");
			return -ENOMEM;
		}

		error = mxt_parse_dt(&client->dev, pdata);
		if (error)
			return error;
	} else {
		TOUCH_ERR_MSG("OF support required\n");
		return -ENODEV;
	}

	if (!pdata->fw_name) {
		TOUCH_ERR_MSG("FW_NAME is NULL\n");
		return -EINVAL;
	}

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
			client->adapter->nr, client->addr);
	data->client = client;
	data->irq = client->irq;
	data->pdata = pdata;
	i2c_set_clientdata(client, data);

	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	init_completion(&data->t25_completion);

	/* request reset pin */
	if (gpio_is_valid(pdata->gpio_reset)) {
		error = devm_gpio_request_one(&client->dev, pdata->gpio_reset,
				GPIOF_OUT_INIT_LOW, "touch_reset");
		if (error < 0) {
			TOUCH_ERR_MSG("FAIL: touch_reset gpio_request\n");
			return error;
		}
	}

	/* request interrupt pin */
	if (gpio_is_valid(pdata->gpio_int)) {
		error = devm_gpio_request_one(&client->dev, pdata->gpio_int,
				GPIOF_IN, "touch_int");
		if (error < 0) {
			TOUCH_ERR_MSG("FAIL: touch_int gpio_request\n");
			return error;
		}
	}

	mutex_init(&i2c_suspend_lock);
	mutex_init(&irq_lock);

	error = devm_request_threaded_irq(&client->dev, data->irq,
			touch_irq_handler, mxt_interrupt_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			client->name, data);
	if (error) {
		TOUCH_ERR_MSG("Failed to register interrupt\n");
		return error;
	}
	touch_disable_irq(data->irq);

	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND,
			"touch_wakelock");
	wake_lock_init(&pm_touch_wake_lock, WAKE_LOCK_SUSPEND,
			"pm_touch_wakelock");

	error = mxt_probe_regulators(data);
	if (error)
		goto err_probe_regulators;
	mxt_regulator_enable(data);

	error = mxt_initialize_t100_input_device(data);
	if (error){
		TOUCH_ERR_MSG("Failed to init t100\n");
		goto err_init_t100_input_device;
	}

	INIT_DELAYED_WORK(&data->work_button_lock, mxt_button_lock_func);
	INIT_DELAYED_WORK(&data->work_palm_unlock, mxt_palm_unlock_func);
	INIT_DELAYED_WORK(&data->work_delay_cal, mxt_delay_cal_func);
	INIT_DELAYED_WORK(&data->work_firmware_update,
			mxt_firmware_update_func);

	/* disabled report touch event to prevent unnecessary event.
	 * it will be enabled in open function
	 */
	data->suspended = true;
	data->enable_reporting = false;

	/* Register sysfs for making fixed communication path to
	 * framework layer
	 */
	error = subsys_system_register(&touch_subsys, NULL);
	if (error < 0) {
		TOUCH_ERR_MSG("%s, bus is not registered, error : %d\n",
				__func__, error);
		goto err_init_t100_input_device;
	}

	error = device_register(&device_touch);
	if (error < 0) {
		TOUCH_ERR_MSG("%s, device is not registered, error : %d\n",
				__func__, error);
		goto err_init_t100_input_device;
	}

	error = kobject_init_and_add(&data->mxt_touch_kobj,
			&mxt_touch_kobj_type,
			data->input_dev->dev.kobj.parent,
			"%s", MXT_TOUCH_NAME);
	if (error < 0) {
		TOUCH_ERR_MSG("kobject_init_and_add is failed\n");
		goto err_mxt_touch_sysfs_init_and_add;
	}

	TOUCH_INFO_MSG("%s success...\n", __func__);

	return 0;

err_mxt_touch_sysfs_init_and_add:
	kobject_del(&data->mxt_touch_kobj);
	device_unregister(&device_touch);
err_init_t100_input_device:
	mxt_regulator_disable(data);
err_probe_regulators:
	mutex_destroy(&i2c_suspend_lock);
	mutex_destroy(&irq_lock);

	wake_lock_destroy(&pm_touch_wake_lock);
	wake_lock_destroy(&touch_wake_lock);

	mxt_free_object_table(data);
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	kobject_del(&data->mxt_touch_kobj);
	device_unregister(&device_touch);

	mxt_regulator_disable(data);
	mxt_free_object_table(data);

	mutex_destroy(&i2c_suspend_lock);
	mutex_destroy(&irq_lock);

	wake_lock_destroy(&pm_touch_wake_lock);
	wake_lock_destroy(&touch_wake_lock);

	return 0;
}

static int mxt_suspend(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	TOUCH_DEBUG_MSG("%s int status:%d\n", __func__,
			gpio_get_value(data->pdata->gpio_int));
	data->pm_state = PM_SUSPEND;
	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	TOUCH_DEBUG_MSG("%s int status:%d\n", __func__,
			gpio_get_value(data->pdata->gpio_int));

	if (data->pm_state == PM_SUSPEND_IRQ) {
		struct irq_desc *desc = irq_to_desc(data->irq);

		if (desc == NULL) {
			TOUCH_ERR_MSG("Null Pointer from irq_to_desc\n");
			return -ENOMEM;
		}

		data->pm_state = PM_RESUME;

		irq_set_pending(data->irq);
		check_irq_resend(desc, data->irq);

		TOUCH_DEBUG_MSG("resend interrupt\n");

		return 0;
	}

	data->pm_state = PM_RESUME;
	return 0;
}

static void mxt_shutdown(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	touch_disable_irq(data->irq);
}

static struct of_device_id mxt_match_table[] = {
	{ .compatible = "atmel,u144",},
	{ },
};

static const struct i2c_device_id mxt_id[] = {
	{ MXT_TOUCH_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct dev_pm_ops touch_pm_ops = {
	.suspend 	= mxt_suspend,
	.resume 	= mxt_resume,
};

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "touch_atmel",
		.of_match_table = mxt_match_table,
		.owner	= THIS_MODULE,
		.pm	= &touch_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.shutdown	= mxt_shutdown,
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
	int ret;

	touch_wq = create_singlethread_workqueue("touch_wq");
	if (!touch_wq) {
		TOUCH_ERR_MSG("touch_wq error\n");
		ret = -ENOMEM;
	}

	ret = i2c_add_driver(&mxt_driver);
	if (ret < 0) {
		TOUCH_ERR_MSG("can't add i2c driver\n");
		goto error;
	}

	return 0;

error:
	if (touch_wq)
		destroy_workqueue(touch_wq);

	return ret;
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);

	if (touch_wq) {
		destroy_workqueue(touch_wq);
		touch_wq = NULL;
	}
}

late_initcall(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_DESCRIPTION("Atmel Touchscreen driver");
MODULE_LICENSE("GPL");

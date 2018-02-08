/*
** =============================================================================
** Copyright (c) 2014  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
** File:
**     drv2605.c
**
** Description:
**     DRV2605 chip driver
**
** =============================================================================
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include "drv2605.h"

#define C_I2C_FIFO_SIZE 8
#define CONFIG_HAPTICS_DRV2605
#define CONFIG_HAPTICS_LRA_SEMCO1030

#define LOG_TAG "[DRV2605]"

int back_cover = 0x31;

static void drv2605_change_mode(struct drv2605_data *pDrv2605data,
				char work_mode, char dev_mode);
static ssize_t drv2605_ID_show(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t drv2605_vol_show(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t drv2605_mode_help_show(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t drv2605_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len);
static ssize_t drv2605_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static int drv2605_init_dev(struct drv2605_data *data,
			int user_prefer, int index);

static ssize_t drv2605_reg_ctrl_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len);
static ssize_t drv2605_reg_ctrl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf);

static ssize_t drv2605_pwm_mode_shutdown_show(struct device *dev,
			struct device_attribute *attr,
			char *buf);
static ssize_t drv2605_pwm_mode_shutdown_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len);

static ssize_t drv2605_en_ctrl_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len);

static struct drv2605_data *pDRV2605data;

static DEVICE_ATTR(ID, S_IRUSR, drv2605_ID_show, NULL);
static DEVICE_ATTR(vol, S_IRUSR, drv2605_vol_show, NULL);
static DEVICE_ATTR(mode, 0660, drv2605_mode_show, drv2605_mode_store);
static DEVICE_ATTR(mode_help, S_IRUSR, drv2605_mode_help_show, NULL);
static DEVICE_ATTR(reg_ctrl, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
			drv2605_reg_ctrl_show, drv2605_reg_ctrl_store);

static DEVICE_ATTR(pwm_mode_shutdown,
			S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
			drv2605_pwm_mode_shutdown_show,
			drv2605_pwm_mode_shutdown_store);

static DEVICE_ATTR(en_ctrl, S_IWUSR | S_IWGRP,
			NULL, drv2605_en_ctrl_store);

static struct attribute *drv2605_attrs[] = {
	&dev_attr_ID.attr,
	&dev_attr_vol.attr,
	&dev_attr_mode.attr,
	&dev_attr_mode_help.attr,
	&dev_attr_reg_ctrl.attr,
	&dev_attr_pwm_mode_shutdown.attr,
	&dev_attr_en_ctrl.attr,
	NULL
};

static const struct attribute_group drv2605_attr_group = {
	.attrs = drv2605_attrs,
};


#if defined(CONFIG_HAPTICS_DRV2605)
static struct drv2605_platform_data  drv2605_plat_data[2] = {
	[0] = {
		/*enable the chip*/
		.GpioEnable = 0,
		/*external trigger pin, (0: internal trigger)*/
		.GpioTrigger = 0,
		/*rated = 1.5Vrms, ov=2.1Vrms, f=200hz*/
		.loop = CLOSE_LOOP,
		.RTPFormat = Signed,
		.BIDIRInput = BiDirectional,
		.actuator = {
			.device_type = ERM,
			.rated_vol = 0x6c,
			.g_effect_bank = LIBRARY_D,
			.over_drive_vol = 0x75,
			.LRAFreq = 200,
		},
		.a2h = {
			.a2h_min_input = AUDIO_HAPTICS_MIN_INPUT_VOLTAGE,
			.a2h_max_input = AUDIO_HAPTICS_MAX_INPUT_VOLTAGE,
			.a2h_min_output = AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE,
			.a2h_max_output = AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE,
		},
	},
	[1] = {
		/*enable the chip*/
		.GpioEnable = 0,
		/*external trigger pin, (0: internal trigger)*/
		.GpioTrigger = 0,
		/*rated = 1.5Vrms, ov=2.1Vrms, f=200hz*/
		.loop = CLOSE_LOOP,
		.RTPFormat = Signed,
		.BIDIRInput = BiDirectional,
		.actuator = {
			.device_type = ERM,
			.rated_vol = 0x4b,
			.g_effect_bank = LIBRARY_D,
			.over_drive_vol = 0x54,
			.LRAFreq = 200,
		},
		.a2h = {
			.a2h_min_input = AUDIO_HAPTICS_MIN_INPUT_VOLTAGE,
			.a2h_max_input = AUDIO_HAPTICS_MAX_INPUT_VOLTAGE,
			.a2h_min_output = AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE,
			.a2h_max_output = AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE,
		},
	},
};
#endif

#if defined(CONFIG_REGMAP)
static int drv2605_reg_read(struct drv2605_data *pDrv2605data, unsigned int reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pDrv2605data->regmap, reg, &val);

	if (ret < 0)
		return ret;
	else
		return val;
}

static int drv2605_reg_write(struct drv2605_data *pDrv2605data,
			unsigned char reg, char val)
{
	return regmap_write(pDrv2605data->regmap, reg, val);
}

static int drv2605_bulk_read(struct drv2605_data *pDrv2605data,
			unsigned char reg, unsigned int count,
			u8 *buf)
{
	return regmap_bulk_read(pDrv2605data->regmap, reg, buf, count);
}

static int drv2605_bulk_write(struct drv2605_data *pDrv2605data,
			unsigned char reg, unsigned int count,
			const u8 *buf)
{
	return regmap_bulk_write(pDrv2605data->regmap, reg, buf, count);
}

static int drv2605_set_bits(struct drv2605_data *pDrv2605data,
			unsigned char reg, unsigned char mask,
			unsigned char val)
{
	return regmap_update_bits(pDrv2605data->regmap, reg, mask, val);
}

#else

static int drv2605_reg_write_block(struct i2c_client *client, u8 addr,
			u8 *data, u8 len)
{
	/*because address also occupies one byte,
	 *the maximum length for write is 7 bytes*/
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;

	if (!client) {
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		pr_err(LOG_TAG" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		pr_err(LOG_TAG"send command error!!\n");
		return -EFAULT;
	}
	return err;
}


static int drv2605_reg_read_block(struct i2c_client *client,
			u8 reg, bool single)
{
	struct i2c_msg msg[2] = { { 0 }, { 0 } };
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	ret = data[0];

	return ret;
}

static int drv2605_reg_write(struct drv2605_data *pDrv2605data,
			unsigned char reg, char val)
{
	unsigned int ret;

	ret = drv2605_reg_write_block(pDrv2605data->i2c_client, reg, &val, 1);

	return ret;
}

static int drv2605_reg_read(struct drv2605_data *pDrv2605data, u8 reg)
{
	unsigned int val;

	val = drv2605_reg_read_block(pDrv2605data->i2c_client, reg, 1);

	return val;
}

static int drv2605_bulk_read(struct drv2605_data *pDrv2605data,
			unsigned char reg, unsigned int count, u8 *buf)
{
	int i = 0;

	for (i = 0; i < count; i++)
		buf[i * 1] = drv2605_reg_read(pDrv2605data, reg + (i * 1));

	return 0;
}

static int drv2605_bulk_write(struct drv2605_data *pDrv2605data,
			unsigned char reg,
			unsigned int count, const u8 *buf)
{
	int i = 0;

	for (i = 0; i < count; i++)
		drv2605_reg_write(pDrv2605data, reg + (i * 1), buf[i * 1]);

	return 0;
}

static int drv2605_set_bits(struct drv2605_data *pDrv2605data,
			unsigned char reg, unsigned char mask,
			unsigned char val)
{
	unsigned int value, tmp;
	int ret = 0;

	value = drv2605_reg_read(pDrv2605data, reg);
	tmp = value & ~mask;
	tmp |= val & mask;

	if (tmp != value)
		ret = drv2605_reg_write(pDrv2605data, reg, tmp);
	return ret;
}
#endif

static int drv2605_set_go_bit(struct drv2605_data *pDrv2605data,
			unsigned char val)
{
	return drv2605_reg_write(pDrv2605data, GO_REG, (val & 0x01));
}

static void drv2605_poll_go_bit(struct drv2605_data *pDrv2605data)
{
	while (drv2605_reg_read(pDrv2605data, GO_REG) == GO)
		schedule_timeout_interruptible(
			msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
}

static int drv2605_select_library(struct drv2605_data *pDrv2605data,
			unsigned char lib)
{
	return drv2605_reg_write(pDrv2605data,
			LIBRARY_SELECTION_REG, (lib & 0x07));
}

static int drv2605_set_rtp_val(struct drv2605_data *pDrv2605data, char value)
{
	/* please be noted: in unsigned mode, maximum is 0xff,
	 * in signed mode, maximum is 0x7f */
	return drv2605_reg_write(pDrv2605data, REAL_TIME_PLAYBACK_REG, value);
}

static int drv2605_set_waveform_sequence(struct drv2605_data *pDrv2605data,
		unsigned char *seq, unsigned int size)
{
	return drv2605_bulk_write(pDrv2605data, WAVEFORM_SEQUENCER_REG,
		  (size > WAVEFORM_SEQUENCER_MAX) ?
		  WAVEFORM_SEQUENCER_MAX : size, seq);
}

static ssize_t drv2605_ID_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int err = 0;
	ssize_t ret = 0;

	err = drv2605_reg_read(pDRV2605data, STATUS_REG);
	if (err < 0) {
		pr_err(LOG_TAG"%s, i2c bus fail (%d)\n", __func__, err);
		ret = snprintf(buf, 16, "read error\n");
	} else {
		pr_err(LOG_TAG"%s, i2c status (0x%x)\n", __func__, err);
		ret = snprintf(buf, 16, "%d\n", (err & DEV_ID_MASK));
	}
	return ret;
}

static ssize_t drv2605_mode_help_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;

	return snprintf(buf, 1024,
		       "work_mode:\n"
		       "    0 - WORK_IDLE\n"
		       "    4 - WORK_PWM\n"
		       "    5 - WORK_AUDIO2HAPTIC\n"
		       "    6 - WORK_RTP\n"
		       "    7 - WORK_CALIBRATION\n"
		       "    8 - WORK_VIBRATOR\n"
		       "    9 - WORK_PATTERN_RTP_ON\n"
		       "    10 - WORK_PATTERN_RTP_OFF\n"
		       "    11 - WORK_SEQ_RTP_ON\n"
		       "    12 - WORK_SEQ_RTP_OFF\n"
		       "    13 - WORK_SEQ_PLAYBACK\n"
		       "\n"
		       "dev_mode:\n"
		       "    0 - DEV_IDLE\n"
		       "    1 - DEV_STANDBY\n"
		       "    2 - DEV_READY\n"
		       "\n"
		       "work_mode: %d, dev_mode: %d\n",
		       pDrv2605data->work_mode, pDrv2605data->dev_mode);
}

static ssize_t drv2605_mode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;

	return snprintf(buf, 10, "%d, %d\n",
			pDrv2605data->work_mode,
			pDrv2605data->dev_mode);
}

static int chr_is_dec_data(const char *pdata)
{
	if (
		(
			(*pdata >= '0') && (*pdata <= '9')
		)
	)
		return 1;
	else
		return 0;

}

static int chr_is_hex_data(const char *pdata)
{
	if (
		(
			(*pdata >= '0') && (*pdata <= '9')
		) ||
		(
			(*pdata >= 'a') && (*pdata <= 'f')
		) ||
		(
			(*pdata >= 'A') && (*pdata <= 'F')
		)
	)
		return 1;
	else
		return 0;

}

static ssize_t drv2605_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len)
{
	char *tmp_buf = NULL;
	int i = 0, j = 0;
	int index_buf = 0;
	int need_break = 0;
	unsigned long data_buf_t[2] = {0};
	int data_buf_index_t = 0;
	int ret = 0;
	char work_mode, dev_mode;
	struct drv2605_data *pDrv2605data = pDRV2605data;

	tmp_buf = kmalloc(sizeof(char) * 5, GFP_KERNEL);
	if (!tmp_buf)
		return -EFAULT;

	memset(tmp_buf, 0, sizeof(char) * 5);

	for (i = 0; i < 2; i++) {
		memset(tmp_buf, 0, sizeof(char) * 5);
		j = 0;

		while (chr_is_dec_data(&buf[index_buf])
		       && (index_buf < len)
		       && (j < 5)) {
			tmp_buf[j] = buf[index_buf];
			j++;
			index_buf++;
		}
		while ((!chr_is_dec_data(&buf[index_buf]))
		       && (index_buf < len)) {
			if (buf[index_buf] == '\0'
			|| buf[index_buf] == '\n'
			|| buf[index_buf] == '\r') {
				need_break = 1;
				break;
			}
			index_buf++;
		}
		/*transfer tmp_buf to int*/
		ret = kstrtoul(tmp_buf, 10, &data_buf_t[i]);

		data_buf_index_t++;

		if (need_break)
			break;
	}

	work_mode = data_buf_t[0];
	dev_mode = data_buf_t[1];
	pr_info(LOG_TAG"%s: work_mode = %d, dev_mode = %d\n", __func__,
			work_mode, dev_mode);
	if (work_mode < WORK_IDLE || work_mode > WORK_SEQ_PLAYBACK) {
		pr_err(LOG_TAG"%s: invalid workmode %d\n",
			__func__, work_mode);
		return -EFAULT;
	}
	if (dev_mode < DEV_IDLE || dev_mode > DEV_READY) {
		pr_err(LOG_TAG"%s: invalid devmode %d\n",
			__func__, dev_mode);
		return -EFAULT;
	}

	kfree(tmp_buf);
	tmp_buf = NULL;
	drv2605_change_mode(pDrv2605data, work_mode, dev_mode);

	if (!!ret)
		return ret;

	return len;
}

static ssize_t drv2605_reg_ctrl_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len)
{
	char *tmp_buf = NULL;
	int i = 0, j = 0;
	int index_buf = 0;
	int need_break = 0;
	unsigned long data_buf_t[2] = {0};
	int data_buf_index_t = 0;
	int ret = 0;
	char reg_addr, reg_value;
	struct drv2605_data *pDrv2605data = pDRV2605data;

	tmp_buf = kmalloc(sizeof(char) * 5, GFP_KERNEL);
	if (!tmp_buf)
		return -EFAULT;

	memset(tmp_buf, 0, sizeof(char) * 5);

	for (i = 0; i < 2; i++) {
		memset(tmp_buf, 0, sizeof(char) * 5);
		j = 0;

		while (chr_is_hex_data(&buf[index_buf])
		       && (index_buf < len)
		       && (j < 5)) {
			tmp_buf[j] = buf[index_buf];
			j++;
			index_buf++;
		}
		while ((!chr_is_hex_data(&buf[index_buf]))
		       && (index_buf < len)) {
			if (buf[index_buf] == '\0'
			|| buf[index_buf] == '\n'
			|| buf[index_buf] == '\r') {
				need_break = 1;
				break;
			}
			index_buf++;
		}
		/*transfer tmp_buf to int*/
		ret = kstrtoul(tmp_buf, 16, &data_buf_t[i]);

		data_buf_index_t++;

		if (need_break)
			break;
	}

	reg_addr = data_buf_t[0];
	reg_value = data_buf_t[1];
	if (data_buf_index_t == 1) {
		pDrv2605data->sysfs_reg_ctrl.is_read_reg_cmd = 1;
		pDrv2605data->sysfs_reg_ctrl.read_reg_addr = reg_addr;
	} else {
		pDrv2605data->sysfs_reg_ctrl.is_read_reg_cmd = 0;
		pDrv2605data->sysfs_reg_ctrl.read_reg_addr = reg_addr;
	}

	pr_info(LOG_TAG"%s: is_read_ops = %d, reg_addr = 0x%02x, reg_value = 0x%02x\n",
			__func__, pDrv2605data->sysfs_reg_ctrl.is_read_reg_cmd,
			reg_addr, reg_value);

	if (!pDrv2605data->sysfs_reg_ctrl.is_read_reg_cmd) {
		pr_info(LOG_TAG"%s: write 0x%02x to reg 0x%02x.\n", __func__,
			reg_value, reg_addr);
		drv2605_reg_write(pDrv2605data, reg_addr, reg_value);
	}

	kfree(tmp_buf);
	tmp_buf = NULL;

	if (!!ret)
		return ret;

	return len;
}

static ssize_t drv2605_reg_ctrl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;
	int reg_value, ret;
	char reg_addr = pDrv2605data->sysfs_reg_ctrl.read_reg_addr;

	if (pDrv2605data->sysfs_reg_ctrl.is_read_reg_cmd) {
		reg_value = drv2605_reg_read(pDrv2605data, reg_addr);
		pr_info(LOG_TAG"%s: read reg 0x%02x result is 0x%02x.\n",
			__func__, reg_addr, reg_value);
		ret = snprintf(buf, 32, "reg: 0x%02x, value: 0x%02x\n",
			reg_addr, reg_value);
	} else {
		pr_info(LOG_TAG"%s: no read reg_addr has been set.\n",
			__func__);
		ret = snprintf(buf, 32, "no read reg_addr has been set.\n");
	}
	return ret;
}

static ssize_t drv2605_pwm_mode_shutdown_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;

	return snprintf(buf, 32, "%d\n",
				pDrv2605data->will_switch_pwm_mode_shutdown);
}

static ssize_t drv2605_pwm_mode_shutdown_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;
	int setting = 0;

	if (sscanf(buf, "%d\n", &setting) > 0)
		pDrv2605data->will_switch_pwm_mode_shutdown = setting;

	return len;
}

static ssize_t drv2605_en_ctrl_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t len)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;
	int status = 0;

	if (sscanf(buf, "%d\n", &status) > 0) {
		if (pDrv2605data->PlatData[back_cover].GpioEnable) {
			/* Enable power to the chip */
			gpio_direction_output(
				pDrv2605data->PlatData[back_cover].GpioEnable,
				status);
			/* Wait 30 us */
			udelay(30);
		}
	}

	return len;
}

static void drv2605_change_mode(struct drv2605_data *pDrv2605data,
			char work_mode, char dev_mode)
{
	/* please be noted : LRA open loop cannot be used with
	 * analog input mode */
	if (dev_mode == DEV_IDLE) {
		pDrv2605data->dev_mode = dev_mode;
		pDrv2605data->work_mode = work_mode;
	} else if (dev_mode == DEV_STANDBY) {
		if (pDrv2605data->dev_mode != DEV_STANDBY) {
			pDrv2605data->dev_mode = DEV_STANDBY;
			drv2605_reg_write(pDrv2605data, MODE_REG,
					 MODE_STANDBY);
			schedule_timeout_interruptible(
				msecs_to_jiffies(WAKE_STANDBY_DELAY));
		}
		pDrv2605data->work_mode = WORK_IDLE;
	} else if (dev_mode == DEV_READY) {
		if ((work_mode != pDrv2605data->work_mode)
		    || (dev_mode != pDrv2605data->dev_mode)) {
			pDrv2605data->work_mode = work_mode;
			pDrv2605data->dev_mode = dev_mode;
			if ((pDrv2605data->work_mode == WORK_VIBRATOR)
			    || (pDrv2605data->work_mode == WORK_PATTERN_RTP_ON)
			    || (pDrv2605data->work_mode == WORK_SEQ_RTP_ON)
			    || (pDrv2605data->work_mode == WORK_RTP)) {
				drv2605_reg_write(pDrv2605data, MODE_REG,
						 MODE_REAL_TIME_PLAYBACK);
			} else if (pDrv2605data->work_mode ==
					WORK_AUDIO2HAPTIC) {
				drv2605_reg_write(pDrv2605data, MODE_REG,
						 MODE_AUDIOHAPTIC);
			} else if (pDrv2605data->work_mode ==
					WORK_CALIBRATION) {
				drv2605_reg_write(pDrv2605data, MODE_REG,
						 AUTO_CALIBRATION);
			} else if (pDrv2605data->work_mode == WORK_PWM) {
				drv2605_reg_write(pDrv2605data, MODE_REG,
						 MODE_PWM_OR_ANALOG_INPUT);
				drv2605_set_bits(pDrv2605data,
						 Control3_REG,
						 Control3_REG_PWMANALOG_MASK,
						 INPUT_PWM);
			} else {
				drv2605_reg_write(pDrv2605data, MODE_REG,
						 MODE_INTERNAL_TRIGGER);
				schedule_timeout_interruptible(
							msecs_to_jiffies(
							STANDBY_WAKE_DELAY));
			}
		}
	}
}

static void setAudioHapticsEnabled(struct drv2605_data *pDrv2605data,
			int enable)
{
	if (enable) {
		if (pDrv2605data->work_mode != WORK_AUDIO2HAPTIC) {
			pDrv2605data->vibrator_is_playing = YES;
			drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_READY);

			drv2605_set_bits(pDrv2605data,
					 Control1_REG,
					 Control1_REG_AC_COUPLE_MASK,
					 AC_COUPLE_ENABLED);

			drv2605_set_bits(pDrv2605data,
					 Control3_REG,
					 Control3_REG_PWMANALOG_MASK,
					 INPUT_ANALOG);

			drv2605_change_mode(pDrv2605data,
					WORK_AUDIO2HAPTIC, DEV_READY);
			switch_set_state(&pDrv2605data->sw_dev,
					SW_STATE_AUDIO2HAPTIC);
		}
	} else {
		/* Chip needs to be brought out of standby to
		 * change the registers */
		if (pDrv2605data->work_mode == WORK_AUDIO2HAPTIC) {
			pDrv2605data->vibrator_is_playing = NO;
			drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_READY);

			drv2605_set_bits(pDrv2605data,
					 Control1_REG,
					 Control1_REG_AC_COUPLE_MASK,
					 AC_COUPLE_DISABLED);

			drv2605_set_bits(pDrv2605data,
					 Control3_REG,
					 Control3_REG_PWMANALOG_MASK,
					 INPUT_PWM);

			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
			/* Disable audio-to-haptics*/
			drv2605_change_mode(pDrv2605data,
					WORK_IDLE, DEV_STANDBY);
		}
	}
}

static void play_effect(struct drv2605_data *pDrv2605data)
{
	switch_set_state(&pDrv2605data->sw_dev, SW_STATE_SEQUENCE_PLAYBACK);
	drv2605_change_mode(pDrv2605data, WORK_SEQ_PLAYBACK, DEV_READY);
	drv2605_set_waveform_sequence(pDrv2605data, pDrv2605data->sequence,
				      WAVEFORM_SEQUENCER_MAX);
	pDrv2605data->vibrator_is_playing = YES;
	drv2605_set_go_bit(pDrv2605data, GO);

	while ((drv2605_reg_read(pDrv2605data, GO_REG) == GO) &&
	       (pDrv2605data->should_stop == NO)) {
		schedule_timeout_interruptible(
				msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
	}

	if (pDrv2605data->should_stop == YES)
		drv2605_set_go_bit(pDrv2605data, STOP);

	if (pDrv2605data->audio_haptics_enabled)
		setAudioHapticsEnabled(pDrv2605data, YES);
	else {
		drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
		pDrv2605data->vibrator_is_playing = NO;
		wake_unlock(&pDrv2605data->wklock);
	}
	wake_unlock(&pDrv2605data->wklock);
}

static void play_Pattern_RTP(struct drv2605_data *pDrv2605data)
{
	if (pDrv2605data->work_mode == WORK_PATTERN_RTP_ON) {
		drv2605_change_mode(pDrv2605data,
				WORK_PATTERN_RTP_OFF, DEV_READY);
		if (pDrv2605data->repeat_times == 0) {
			drv2605_change_mode(pDrv2605data,
					WORK_IDLE, DEV_STANDBY);
			pDrv2605data->vibrator_is_playing = NO;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
			wake_unlock(&pDrv2605data->wklock);
		} else {
			hrtimer_start(&pDrv2605data->timer,
					ns_to_ktime(
					(u64)pDrv2605data->silience_time *
					NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	} else if (pDrv2605data->work_mode == WORK_PATTERN_RTP_OFF) {
		pDrv2605data->repeat_times--;
		drv2605_change_mode(pDrv2605data,
				WORK_PATTERN_RTP_ON, DEV_READY);
		hrtimer_start(&pDrv2605data->timer,
				ns_to_ktime((u64)pDrv2605data->vibration_time *
				NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
	}
}

static void play_Seq_RTP(struct drv2605_data *pDrv2605data)
{
	if (pDrv2605data->RTPSeq.RTPindex < pDrv2605data->RTPSeq.RTPCounts) {
		int RTPTime =
			pDrv2605data->
			RTPSeq.RTPData[pDrv2605data->RTPSeq.RTPindex] >> 8;
		int RTPVal =
			pDrv2605data->
			RTPSeq.RTPData[pDrv2605data->RTPSeq.RTPindex] &
			0x00ff;

		pDrv2605data->vibrator_is_playing = YES;
		pDrv2605data->RTPSeq.RTPindex++;
		drv2605_change_mode(pDrv2605data, WORK_SEQ_RTP_ON, DEV_READY);
		drv2605_set_rtp_val(pDrv2605data,  RTPVal);

		hrtimer_start(&pDrv2605data->timer,
				ns_to_ktime((u64)RTPTime * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
	} else {
		drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);
		pDrv2605data->vibrator_is_playing = NO;
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
		wake_unlock(&pDrv2605data->wklock);
	}
}

static void vibrator_off(struct drv2605_data *pDrv2605data)
{
	if (pDrv2605data->vibrator_is_playing) {
		if (pDrv2605data->audio_haptics_enabled == YES) {
			setAudioHapticsEnabled(pDrv2605data, YES);
		} else {
			pDrv2605data->vibrator_is_playing = NO;
			drv2605_set_go_bit(pDrv2605data, STOP);
			drv2605_change_mode(pDrv2605data,
					WORK_IDLE, DEV_STANDBY);
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
			wake_unlock(&pDrv2605data->wklock);
		}
	}
}

static void drv2605_stop(struct drv2605_data *pDrv2605data)
{
	if (pDrv2605data->vibrator_is_playing) {
		if (pDrv2605data->work_mode == WORK_AUDIO2HAPTIC) {
			setAudioHapticsEnabled(pDrv2605data, NO);
		} else if ((pDrv2605data->work_mode == WORK_VIBRATOR)
			   || (pDrv2605data->work_mode == WORK_PATTERN_RTP_ON)
			   || (pDrv2605data->work_mode == WORK_PATTERN_RTP_OFF)
			   || (pDrv2605data->work_mode == WORK_SEQ_RTP_ON)
			   || (pDrv2605data->work_mode == WORK_SEQ_RTP_OFF)
			   || (pDrv2605data->work_mode == WORK_RTP)) {
			vibrator_off(pDrv2605data);
		} else if (pDrv2605data->work_mode == WORK_SEQ_PLAYBACK)
			;
		else
			pr_err(LOG_TAG"%s, err mode=%d\n", __func__,
				pDrv2605data->work_mode);
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct drv2605_data *pDrv2605data =
			container_of(dev, struct drv2605_data, to_dev);

	if (hrtimer_active(&pDrv2605data->timer)) {
		ktime_t r = hrtimer_get_remaining(&pDrv2605data->timer);

		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct drv2605_data *pDrv2605data =
			container_of(dev, struct drv2605_data, to_dev);

	pDrv2605data->should_stop = YES;
	hrtimer_cancel(&pDrv2605data->timer);
	cancel_work_sync(&pDrv2605data->vibrator_work);

	mutex_lock(&pDrv2605data->lock);

	if (value < 0) {
		pr_err(LOG_TAG"Error enable value: %d.\n", value);
		return;
	}
	if (pDrv2605data->work_mode != WORK_PWM) {
		drv2605_stop(pDrv2605data);

		if (value > 0) {
			if (pDrv2605data->audio_haptics_enabled == NO)
				wake_lock(&pDrv2605data->wklock);

			drv2605_change_mode(pDrv2605data,
						WORK_VIBRATOR, DEV_READY);
			pDrv2605data->vibrator_is_playing = YES;
			switch_set_state(&pDrv2605data->sw_dev,
							SW_STATE_RTP_PLAYBACK);

			value = (value > MAX_TIMEOUT) ? MAX_TIMEOUT : value;
			hrtimer_start(&pDrv2605data->timer,
					ns_to_ktime((u64)value * NSEC_PER_MSEC),
					HRTIMER_MODE_REL);
		}
	} else {
		pr_info(LOG_TAG"vibrator in pwm mode, fefuse to enable req.\n");
	}
	mutex_unlock(&pDrv2605data->lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct drv2605_data *pDrv2605data =
			container_of(timer, struct drv2605_data, timer);

	schedule_work(&pDrv2605data->vibrator_work);

	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2605_data *pDrv2605data =
			container_of(work, struct drv2605_data,
					vibrator_work);

	mutex_lock(&pDrv2605data->lock);

	if ((pDrv2605data->work_mode == WORK_VIBRATOR)
	    || (pDrv2605data->work_mode == WORK_RTP)) {
		vibrator_off(pDrv2605data);
	} else if (pDrv2605data->work_mode == WORK_SEQ_PLAYBACK) {
		play_effect(pDrv2605data);
	} else if ((pDrv2605data->work_mode == WORK_PATTERN_RTP_ON) ||
		   (pDrv2605data->work_mode == WORK_PATTERN_RTP_OFF)) {
		play_Pattern_RTP(pDrv2605data);
	} else if ((pDrv2605data->work_mode == WORK_SEQ_RTP_ON) ||
		   (pDrv2605data->work_mode == WORK_SEQ_RTP_OFF)) {
		play_Seq_RTP(pDrv2605data);
	}

	mutex_unlock(&pDrv2605data->lock);
}

static int dev2605_open(struct inode *i_node, struct file *filp)
{
	if (pDRV2605data == NULL)
		return -ENODEV;

	filp->private_data = pDRV2605data;
	return 0;
}

static ssize_t dev2605_read(struct file *filp, char *buff,
			size_t length, loff_t *offset)
{
	struct drv2605_data *pDrv2605data =
			(struct drv2605_data *)filp->private_data;
	int ret = 0;

	if (pDrv2605data->ReadLen > 0) {
		ret = copy_to_user(buff, pDrv2605data->ReadBuff,
				pDrv2605data->ReadLen);
		if (ret != 0)
			pr_err(LOG_TAG"%s, copy_to_user err=%d\n",
				__func__, ret);
		else
			ret = pDrv2605data->ReadLen;

		pDrv2605data->ReadLen = 0;
	}

	return ret;
}

static bool isforDebug(int cmd)
{
	return ((cmd == HAPTIC_CMDID_REG_WRITE)
		|| (cmd == HAPTIC_CMDID_REG_READ)
		|| (cmd == HAPTIC_CMDID_REG_SETBIT));
}

static ssize_t dev2605_write(struct file *filp, const char *buff,
			size_t len, loff_t *off)
{
	struct drv2605_data *pDrv2605data =
			(struct drv2605_data *)filp->private_data;
	char buffer[4] = {0};

	if (!isforDebug(buff[0])) {
		pDrv2605data->should_stop = YES;
		hrtimer_cancel(&pDrv2605data->timer);
		cancel_work_sync(&pDrv2605data->vibrator_work);
	}

	mutex_lock(&pDrv2605data->lock);

	if (!isforDebug(buff[0]))
		drv2605_stop(pDrv2605data);


	switch (buff[0]) {
	case HAPTIC_CMDID_PLAY_SINGLE_EFFECT:
	case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE: {
		memset(&pDrv2605data->sequence, 0, WAVEFORM_SEQUENCER_MAX);
		if (!copy_from_user(&pDrv2605data->sequence,
				&buff[1], len - 1)) {
			if (pDrv2605data->audio_haptics_enabled == NO)
				wake_lock(&pDrv2605data->wklock);

			pDrv2605data->should_stop = NO;
			drv2605_change_mode(pDrv2605data,
					WORK_SEQ_PLAYBACK, DEV_IDLE);
			schedule_work(&pDrv2605data->vibrator_work);
		}
		break;
	}
	case HAPTIC_CMDID_PLAY_TIMED_EFFECT: {
		unsigned int value = 0;

		value = buff[2];
		value <<= 8;
		value |= buff[1];

		if (value > 0) {
			if (pDrv2605data->audio_haptics_enabled == NO)
				wake_lock(&pDrv2605data->wklock);

			switch_set_state(&pDrv2605data->sw_dev,
					SW_STATE_RTP_PLAYBACK);
			pDrv2605data->vibrator_is_playing = YES;
			value = (value > MAX_TIMEOUT) ? MAX_TIMEOUT : value;
			drv2605_change_mode(pDrv2605data, WORK_RTP, DEV_READY);

			hrtimer_start(&pDrv2605data->timer,
					ns_to_ktime((u64)value * NSEC_PER_MSEC),
					HRTIMER_MODE_REL);
		}
		break;
	}

	case HAPTIC_CMDID_PATTERN_RTP: {
		unsigned char strength = 0;

		pDrv2605data->vibration_time =
				(int)((((int)buff[2]) << 8) | (int)buff[1]);
		pDrv2605data->silience_time =
				(int)((((int)buff[4]) << 8) | (int)buff[3]);
		strength = buff[5];
		pDrv2605data->repeat_times = buff[6];

		if (pDrv2605data->vibration_time > 0) {
			if (pDrv2605data->audio_haptics_enabled == NO)
				wake_lock(&pDrv2605data->wklock);

			switch_set_state(&pDrv2605data->sw_dev,
					SW_STATE_RTP_PLAYBACK);
			pDrv2605data->vibrator_is_playing = YES;
			if (pDrv2605data->repeat_times > 0)
				pDrv2605data->repeat_times--;
			if (pDrv2605data->vibration_time > MAX_TIMEOUT)
				pDrv2605data->vibration_time = MAX_TIMEOUT;
			drv2605_change_mode(pDrv2605data,
					WORK_PATTERN_RTP_ON, DEV_READY);
			drv2605_set_rtp_val(pDrv2605data, strength);

			hrtimer_start(&pDrv2605data->timer,
				ns_to_ktime((u64)pDrv2605data->vibration_time *
				NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
		}
		break;
	}

	case HAPTIC_CMDID_RTP_SEQUENCE: {
		memset(&pDrv2605data->RTPSeq, 0, sizeof(struct RTP_Seq));
		if (((len - 1) % 2) == 0) {
			pDrv2605data->RTPSeq.RTPCounts = (len - 1) / 2;
			if ((pDrv2605data->RTPSeq.RTPCounts <= MAX_RTP_SEQ) &&
			    (pDrv2605data->RTPSeq.RTPCounts > 0)) {
				if (copy_from_user(pDrv2605data->RTPSeq.RTPData,
					&buff[1],
					pDrv2605data->RTPSeq.RTPCounts * 2)
					!= 0) {
					pr_err(LOG_TAG"%s, rtp_seq copy seq err\n",
						__func__);
					break;
				}

				if (pDrv2605data->audio_haptics_enabled == NO)
					wake_lock(&pDrv2605data->wklock);

				switch_set_state(&pDrv2605data->sw_dev,
						SW_STATE_RTP_PLAYBACK);
				drv2605_change_mode(pDrv2605data,
						WORK_SEQ_RTP_OFF, DEV_IDLE);
				schedule_work(&pDrv2605data->vibrator_work);
			} else
				pr_err(LOG_TAG"%s, rtp_seq count error,maximum=%d\n",
						__func__, MAX_RTP_SEQ);
		} else
			pr_err(LOG_TAG"%s, rtp_seq len error\n", __func__);
		break;
	}
	case HAPTIC_CMDID_USER_PREFER: {
		drv2605_init_dev(pDrv2605data, 1, (int)buff[1]);
		pDrv2605data->user_prefer = (int)buff[1];
		break;
	}
	case HAPTIC_CMDID_USER_PREFER_READ: {
		snprintf(buffer, 4, "#%d\n", pDrv2605data->user_prefer);
		if (0 != copy_to_user((void __user *)buff, buffer, 2))
			pr_err(LOG_TAG"%s, USER_PREFER_READ error.\n",
				__func__);
		break;
	}
	case HAPTIC_CMDID_STOP: {
		break;
	}

	case HAPTIC_CMDID_AUDIOHAPTIC_ENABLE: {
		if (pDrv2605data->audio_haptics_enabled == NO)
			wake_lock(&pDrv2605data->wklock);
		pDrv2605data->audio_haptics_enabled = YES;
		setAudioHapticsEnabled(pDrv2605data, YES);
		break;
	}

	case HAPTIC_CMDID_AUDIOHAPTIC_DISABLE: {
		if (pDrv2605data->audio_haptics_enabled == YES) {
			pDrv2605data->audio_haptics_enabled = NO;
			wake_unlock(&pDrv2605data->wklock);
		}
		break;
	}

	case HAPTIC_CMDID_REG_READ: {
		if (len == 2) {
			pDrv2605data->ReadLen = 1;
			pDrv2605data->ReadBuff[0] =
					drv2605_reg_read(pDrv2605data, buff[1]);
		} else if (len == 3) {
			pDrv2605data->ReadLen = (buff[2] > MAX_READ_BYTES) ?
						MAX_READ_BYTES : buff[2];
			drv2605_bulk_read(pDrv2605data, buff[1],
					pDrv2605data->ReadLen,
					pDrv2605data->ReadBuff);
		} else
			pr_err(LOG_TAG"%s, reg_read len error\n", __func__);

		break;
	}

	case HAPTIC_CMDID_REG_WRITE: {
		if ((len - 1) == 2) {
			drv2605_reg_write(pDrv2605data, buff[1], buff[2]);
		} else if ((len - 1) > 2) {
			unsigned char *data = kzalloc(len - 2, GFP_KERNEL);

			if (data != NULL) {
				if (copy_from_user(data, &buff[2], len - 2)
					!= 0)
					pr_err(LOG_TAG"%s, reg copy err\n",
						__func__);
				else
					drv2605_bulk_write(pDrv2605data,
						buff[1], len - 2, data);
				kfree(data);
			}
		} else
			pr_err(LOG_TAG"%s, reg_write len error\n", __func__);

		break;
	}

	case HAPTIC_CMDID_REG_SETBIT: {
		int i = 1;

		for (i = 1; i < len;) {
			drv2605_set_bits(pDrv2605data, buff[i],
				buff[i + 1], buff[i + 2]);
			i += 3;
		}
		break;
	}
	default:
		pr_err(LOG_TAG"%s, unknown HAPTIC cmd\n", __func__);
		break;
	}

	mutex_unlock(&pDrv2605data->lock);

	return len;
}


static const struct file_operations fops = {
	.open = dev2605_open,
	.read = dev2605_read,
	.write = dev2605_write,
};

static int drv2605_suspend(struct device *dev)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;

	pDrv2605data->should_stop = YES;
	hrtimer_cancel(&pDrv2605data->timer);
	cancel_work_sync(&pDrv2605data->vibrator_work);

	mutex_lock(&pDrv2605data->lock);

	drv2605_stop(pDrv2605data);
	if (pDrv2605data->audio_haptics_enabled == YES)
		wake_unlock(&pDrv2605data->wklock);

	mutex_unlock(&pDrv2605data->lock);
	return 0;
}

static int drv2605_resume(struct device *dev)
{
	struct drv2605_data *pDrv2605data = pDRV2605data;

	mutex_lock(&pDrv2605data->lock);
	if (pDrv2605data->audio_haptics_enabled == YES) {
		wake_lock(&pDrv2605data->wklock);
		setAudioHapticsEnabled(pDrv2605data, YES);
	}
	mutex_unlock(&pDrv2605data->lock);
	return 0;
}

static int Haptics_init(struct drv2605_data *pDrv2605data)
{
	int reval = -ENOMEM;

	pDrv2605data->version = MKDEV(0, 0);
	reval = alloc_chrdev_region(&pDrv2605data->version,
		0, 1, HAPTICS_DEVICE_NAME);
	if (reval < 0) {
		pr_err(LOG_TAG"error getting major number %d\n",
			reval);
		goto fail0;
	}

	pDrv2605data->class = class_create(THIS_MODULE, HAPTICS_DEVICE_NAME);
	if (!pDrv2605data->class) {
		pr_err(LOG_TAG"error creating class\n");
		goto fail1;
	}

	pDrv2605data->device = device_create(pDrv2605data->class,
					NULL, pDrv2605data->version,
					NULL, HAPTICS_DEVICE_NAME);
	if (!pDrv2605data->device) {
		pr_err(LOG_TAG"error creating device 2605\n");
		goto fail2;
	}

	cdev_init(&pDrv2605data->cdev, &fops);
	pDrv2605data->cdev.owner = THIS_MODULE;
	pDrv2605data->cdev.ops = &fops;
	reval = cdev_add(&pDrv2605data->cdev, pDrv2605data->version, 1);
	if (reval) {
		pr_err(LOG_TAG"fail to add cdev\n");
		goto fail3;
	}

	pDrv2605data->sw_dev.name = "haptics";
	reval = switch_dev_register(&pDrv2605data->sw_dev);
	if (reval < 0) {
		pr_err(LOG_TAG"fail to register switch\n");
		goto fail4;
	}

	pDrv2605data->to_dev.name = "vibrator";
	pDrv2605data->to_dev.get_time = vibrator_get_time;
	pDrv2605data->to_dev.enable = vibrator_enable;

	if (timed_output_dev_register(&(pDrv2605data->to_dev)) < 0) {
		pr_err(LOG_TAG"fail to create timed output dev\n");
		goto fail3;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	/*pDrv2605data->early_suspend.suspend = drv2605_early_suspend;*/
	/*pDrv2605data->early_suspend.resume = drv2605_late_resume;*/
	/*pDrv2605data->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;*/
	/*register_early_suspend(&pDrv2605data->early_suspend);*/
#endif

	hrtimer_init(&pDrv2605data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pDrv2605data->timer.function = vibrator_timer_func;
	INIT_WORK(&pDrv2605data->vibrator_work, vibrator_work_routine);

	wake_lock_init(&pDrv2605data->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&pDrv2605data->lock);

	return 0;

fail4:
	switch_dev_unregister(&pDrv2605data->sw_dev);
fail3:
	device_destroy(pDrv2605data->class, pDrv2605data->version);
fail2:
	class_destroy(pDrv2605data->class);
fail1:
	unregister_chrdev_region(pDrv2605data->version, 1);
fail0:
	return reval;
}

static void dev_init_platform_data(struct drv2605_data *pDrv2605data,
			int user_prefer, int index)
{
	struct drv2605_platform_data *pDrv2605Platdata =
			&pDrv2605data->PlatData[index];
	struct actuator_data actuator = pDrv2605Platdata->actuator;
	struct audio2haptics_data a2h = pDrv2605Platdata->a2h;
	unsigned char temp = 0;

	drv2605_select_library(pDrv2605data, actuator.g_effect_bank);
	/*OTP memory saves data from 0x16 to 0x1a*/
	pr_info(LOG_TAG"rated_vol:%x over_drive_vol:%x\n",
		actuator.rated_vol, actuator.over_drive_vol);
	if (pDrv2605data->OTP == 0 || user_prefer) {
		if (actuator.rated_vol != 0)
			drv2605_reg_write(pDrv2605data,
				 RATED_VOLTAGE_REG,
				 actuator.rated_vol);
		else
			pr_err(LOG_TAG"%s, ERROR Rated ZERO\n", __func__);

		if (actuator.over_drive_vol != 0)
			drv2605_reg_write(pDrv2605data,
				 OVERDRIVE_CLAMP_VOLTAGE_REG,
				 actuator.over_drive_vol);
		else
			pr_err(LOG_TAG"%s, ERROR OverDriveVol ZERO\n",
				__func__);

		drv2605_set_bits(pDrv2605data,
				 FEEDBACK_CONTROL_REG,
				 FEEDBACK_CONTROL_DEVICE_TYPE_MASK,
				 (actuator.device_type == LRA) ?
				 FEEDBACK_CONTROL_MODE_LRA :
				 FEEDBACK_CONTROL_MODE_ERM);
	} else
		pr_info(LOG_TAG"%s, OTP programmed\n", __func__);


	if (pDrv2605Platdata->loop == OPEN_LOOP)
		temp = BIDIR_INPUT_BIDIRECTIONAL;
	else {
		if (pDrv2605Platdata->BIDIRInput == UniDirectional)
			temp = BIDIR_INPUT_UNIDIRECTIONAL;
		else
			temp = BIDIR_INPUT_BIDIRECTIONAL;
	}

	if (actuator.device_type == LRA) {
		unsigned char DriveTime = 5 * (1000 - actuator.LRAFreq) /
						actuator.LRAFreq;
		drv2605_set_bits(pDrv2605data,
				 Control1_REG,
				 Control1_REG_DRIVE_TIME_MASK,
				 DriveTime);
		pr_info(LOG_TAG"%s, LRA = %d, DriveTime=0x%x\n", __func__,
				actuator.LRAFreq, DriveTime);
	}

	drv2605_set_bits(pDrv2605data,
			 Control2_REG,
			 Control2_REG_BIDIR_INPUT_MASK,
			 temp);

	if ((pDrv2605Platdata->loop == OPEN_LOOP) &&
		(actuator.device_type == LRA))
		temp = LRA_OpenLoop_Enabled;
	else if ((pDrv2605Platdata->loop == OPEN_LOOP) &&
		(actuator.device_type == ERM))
		temp = ERM_OpenLoop_Enabled;
	else
		temp = ERM_OpenLoop_Disable | LRA_OpenLoop_Disable;

	if ((pDrv2605Platdata->loop == CLOSE_LOOP) &&
	    (pDrv2605Platdata->BIDIRInput == UniDirectional)) {
		temp |= RTP_FORMAT_UNSIGNED;
		drv2605_reg_write(pDrv2605data,
				REAL_TIME_PLAYBACK_REG, 0xff);
	} else {
		if (pDrv2605Platdata->RTPFormat == Signed) {
			temp |= RTP_FORMAT_SIGNED;
			drv2605_reg_write(pDrv2605data,
				REAL_TIME_PLAYBACK_REG, 0x7f);
		} else {
			temp |= RTP_FORMAT_UNSIGNED;
			drv2605_reg_write(pDrv2605data,
				REAL_TIME_PLAYBACK_REG, 0xff);
		}
	}
	drv2605_set_bits(pDrv2605data,
			 Control3_REG,
			 Control3_REG_LOOP_MASK | Control3_REG_FORMAT_MASK,
			 temp);

	/* for audio to haptics */
	if (pDrv2605Platdata->GpioTrigger == 0) {
		/* not used as external trigger */
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MIN_INPUT_REG,
				a2h.a2h_min_input);
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MAX_INPUT_REG,
				a2h.a2h_max_input);
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MIN_OUTPUT_REG,
				a2h.a2h_min_output);
		drv2605_reg_write(pDrv2605data, AUDIO_HAPTICS_MAX_OUTPUT_REG,
				a2h.a2h_max_output);
	}
}

static int dev_auto_calibrate(struct drv2605_data *pDrv2605data)
{
	int err = 0, status = 0;

	drv2605_change_mode(pDrv2605data, WORK_CALIBRATION, DEV_READY);
	/* drv2605_set_go_bit(pDrv2605data, GO); */

	/* Wait until the procedure is done */
	drv2605_poll_go_bit(pDrv2605data);
	/* Read status */
	status = drv2605_reg_read(pDrv2605data, STATUS_REG);

	pr_info(LOG_TAG"%s, calibration status =0x%x\n", __func__, status);

	/* Read calibration results */
	drv2605_reg_read(pDrv2605data, AUTO_CALI_RESULT_REG);
	drv2605_reg_read(pDrv2605data, AUTO_CALI_BACK_EMF_RESULT_REG);
	drv2605_reg_read(pDrv2605data, FEEDBACK_CONTROL_REG);

	return err;
}

#if defined(CONFIG_REGMAP)
static struct regmap_config drv2605_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};
#endif

static int drv2605_init_dev(struct drv2605_data *data,
			int user_prefer, int index)
{
	int err = 0;

	if (user_prefer == 1)
		drv2605_change_mode(data, WORK_IDLE, DEV_READY);
	dev_init_platform_data(data, user_prefer, index);

	if (data->OTP == 0 || user_prefer) {
		err = dev_auto_calibrate(data);
		if (err < 0)
			pr_err(LOG_TAG"%s, ERROR, calibration fail\n",
				__func__);
	}

	/* Put hardware in standby */
	drv2605_change_mode(data, WORK_IDLE, DEV_STANDBY);

	return err;
}

static ssize_t drv2605_vol_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct drv2605_data *data = dev_get_drvdata(dev);

	if (data->user_prefer == 0)
		return snprintf(buf, 10, "high\n");
	else if (data->user_prefer == 1)
		return snprintf(buf, 10, "low\n");
	else
		return snprintf(buf, 10, "unkown\n");
}

#ifdef CONFIG_OF
static int drv2605_parse_dt(struct drv2605_data *data, const int back_cover)
{
	struct device_node *dt = data->i2c_client->dev.of_node;
	int ret;

	if (!dt)
		return -ENODEV;

	ret = data->PlatData[back_cover].GpioEnable =
	    of_get_named_gpio(dt, "drv2605,en_gpio", 0);
	if (ret < 0) {
		pr_err(LOG_TAG"missing drv2605,en_gpio in device tree\n");
		data->PlatData[back_cover].GpioEnable = 0;
	}

	ret = data->PlatData[back_cover].GpioTrigger =
		of_get_named_gpio(dt, "drv2605,trigger_gpio", 0);
	if (ret < 0) {
		pr_err(LOG_TAG"missing drv2605,trigger_gpio in device tree\n");
		data->PlatData[back_cover].GpioTrigger = 0;
	}
	return 0;
}
#endif

static int drv2605_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct drv2605_data *pDrv2605data;
	struct drv2605_platform_data *pDrv2605Platdata = drv2605_plat_data;

	int err = 0;
	int status = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err(LOG_TAG"%s:I2C check failed\n", __func__);
		return -ENODEV;
	}
	pDrv2605data = devm_kzalloc(&client->dev,
			sizeof(struct drv2605_data), GFP_KERNEL);
	if (pDrv2605data == NULL)
		return -ENOMEM;

	pDrv2605data->i2c_client = client;
#if defined(CONFIG_REGMAP)
	pDrv2605data->regmap =
		devm_regmap_init_i2c(client, &drv2605_i2c_regmap);
	if (IS_ERR(pDrv2605data->regmap)) {
		err = PTR_ERR(pDrv2605data->regmap);
		pr_err(LOG_TAG"%s:Failed to allocate register map: %d\n",
			__func__, err);
		return err;
	}
#endif

	memcpy(pDrv2605data->PlatData, pDrv2605Platdata,
		sizeof(struct drv2605_platform_data) *
		ARRAY_SIZE(drv2605_plat_data));
	i2c_set_clientdata(client, pDrv2605data);

	pr_info(LOG_TAG"@back_cover = %d\n", back_cover);

	if (back_cover  == 0x31) {
		/* plastic back cover */
		pDrv2605data->user_prefer = back_cover = 1;
	} else if (back_cover == 0x32) {
		/* ceramic back cover */
		pDrv2605data->user_prefer = back_cover = 0;
	} else {
		pr_err(LOG_TAG"#error to goto here.\n");
	}

	drv2605_parse_dt(pDrv2605data, back_cover);

	if (pDrv2605data->PlatData[back_cover].GpioTrigger) {
		err = gpio_request(
			pDrv2605data->PlatData[back_cover].GpioTrigger,
			HAPTICS_DEVICE_NAME"Trigger");
		if (err < 0) {
			pr_err(LOG_TAG"%s: GPIO request Trigger error\n",
				__func__);
			goto exit_gpio_request_failed;
		}
	}

	if (pDrv2605data->PlatData[back_cover].GpioEnable) {
		err = gpio_request(
			pDrv2605data->PlatData[back_cover].GpioEnable,
			HAPTICS_DEVICE_NAME"Enable");
		if (err < 0) {
			pr_err(LOG_TAG"%s: GPIO request enable error\n",
				__func__);
			goto exit_gpio_request_failed;
		}

		/* Enable power to the chip */
		gpio_direction_output(
			pDrv2605data->PlatData[back_cover].GpioEnable, 1);

		/* Wait 30 us */
		udelay(30);
	}

	err = drv2605_reg_read(pDrv2605data, STATUS_REG);
	if (err < 0) {
		pr_err(LOG_TAG"%s, i2c bus fail (%d)\n", __func__, err);
		goto exit_gpio_request_failed;
	} else {
		pr_info(LOG_TAG"%s, i2c status (0x%x)\n", __func__, err);
		status = err;
	}

	/* Read device ID */
	pDrv2605data->device_id = (status & DEV_ID_MASK);
	switch (pDrv2605data->device_id) {
	case DRV2605_VER_1DOT1:
		pr_info(LOG_TAG"drv2605 driver found: drv2605 v1.1.\n");
		break;
	case DRV2605_VER_1DOT0:
		pr_info(LOG_TAG"drv2605 driver found: drv2605 v1.0.\n");
		break;
	case DRV2604:
		pr_info(LOG_TAG"drv2605 driver found: drv2604.\n");
		break;
	case DRV2604L:
		pr_info(LOG_TAG"drv2605 driver found: drv2604L.\n");
		break;
	case DRV2605L:
		pr_info(LOG_TAG"drv2605 driver found: drv2605L.\n");
		break;
	default:
		pr_err(LOG_TAG"drv2605 driver found: unknown.\n");
		break;
	}

	if ((pDrv2605data->device_id != DRV2605_VER_1DOT1)
	    && (pDrv2605data->device_id != DRV2605_VER_1DOT0)
	    && (pDrv2605data->device_id != DRV2605L)) {
		pr_err(LOG_TAG"%s, status(0x%x),device_id(%d) fail\n",
		       __func__, status, pDrv2605data->device_id);
		goto exit_gpio_request_failed;
	}

	drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_READY);
	schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

	pDrv2605data->OTP = drv2605_reg_read(pDrv2605data,
			AUTOCAL_MEM_INTERFACE_REG) &
			AUTOCAL_MEM_INTERFACE_REG_OTP_MASK;


	dev_init_platform_data(pDrv2605data, pDrv2605data->user_prefer, 0);

	if (pDrv2605data->OTP == 0) {
		err = dev_auto_calibrate(pDrv2605data);
		if (err < 0)
			pr_err(LOG_TAG"%s, calibration fail\n", __func__);
	}

	/* Put hardware in standby */
	drv2605_change_mode(pDrv2605data, WORK_IDLE, DEV_STANDBY);

	drv2605_init_dev(pDrv2605data, 0, back_cover);

	Haptics_init(pDrv2605data);
	pDrv2605data->audio_haptics_enabled = NO;
	pDRV2605data = pDrv2605data;

	err = sysfs_create_group(&client->dev.kobj, &drv2605_attr_group);
	if (err)
		pr_err(LOG_TAG"failure %d create sysfs group\n", err);

	pr_info(LOG_TAG"drv2605 probe succeeded\n");

	return 0;

exit_gpio_request_failed:
	if (pDrv2605data->PlatData[back_cover].GpioTrigger)
		gpio_free(pDrv2605data->PlatData[back_cover].GpioTrigger);

	if (pDrv2605data->PlatData[back_cover].GpioEnable)
		gpio_free(pDrv2605data->PlatData[back_cover].GpioEnable);

	pr_err(LOG_TAG"%s failed, err=%d\n", __func__, err);
	return err;
}

static void drv2605_shutdown(struct i2c_client *client)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	if (pDrv2605data->will_switch_pwm_mode_shutdown) {
		pr_info(LOG_TAG"shutting down, try to change to pwm mode.\n");
		drv2605_change_mode(pDrv2605data, WORK_PWM, DEV_READY);
	} else {
		pr_info(LOG_TAG"shutting down, no change mode.\n");
	}
	gpio_direction_output(
		pDrv2605data->PlatData[back_cover].GpioEnable, 0);
}

static int drv2605_remove(struct i2c_client *client)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	device_destroy(pDrv2605data->class, pDrv2605data->version);
	class_destroy(pDrv2605data->class);
	unregister_chrdev_region(pDrv2605data->version, 1);

	if (pDrv2605data->PlatData[back_cover].GpioTrigger)
		gpio_free(pDrv2605data->PlatData[back_cover].GpioTrigger);

	if (pDrv2605data->PlatData[back_cover].GpioEnable)
		gpio_free(pDrv2605data->PlatData[back_cover].GpioEnable);

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* unregister_early_suspend(&pDrv2605data->early_suspend); */
#endif

	pr_err(LOG_TAG"drv2605 remove");

	return 0;
}

static const struct dev_pm_ops drv2605_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = drv2605_suspend,
	.resume = drv2605_resume,
#endif
};


static struct i2c_device_id drv2605_id_table[] = {
	{ HAPTICS_DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, drv2605_id_table);

#ifdef CONFIG_OF
static const struct of_device_id drv2605_of_id_table[] = {
	{.compatible = "ti,drv2605"},
	{ },
};
#else
#define drv2667_of_id_table NULL
#endif

static struct i2c_driver drv2605_driver = {
	.driver = {
		.name = HAPTICS_DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = drv2605_of_id_table,
		.pm = &drv2605_pm_ops,
	},
	.id_table = drv2605_id_table,
	.probe = drv2605_probe,
	.remove = drv2605_remove,
	.shutdown = drv2605_shutdown,
};

module_i2c_driver(drv2605_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
MODULE_LICENSE("GPL v2");

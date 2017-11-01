/* Copyright (c) 2017, LGE Inc. All rights reserved.
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/rtc.h>

#include "sop716_firmware.h"

#define SOP716_I2C_DATA_LENGTH               8

#define CMD_SOP716_SET_CURRENT_TIME          0
#define CMD_SOP716_MOTOR_MOVE_ONE            1
#define CMD_SOP716_MOTOR_MOVE_ALL            2
#define CMD_SOP716_MOTOR_INIT                3
#define CMD_SOP716_READ_CURRENT_TIME         4
#define CMD_SOP716_READ_FW_VERSION           5
#define CMD_SOP716_BATTERY_CHECK_PERIOD      7
#define CMD_SOP716_READ_BATTERY_LEVEL        8
#define CMD_SOP716_READ_BATTERY_CHECK_PERIOD 9
#define CMD_SOP716_MAX                       10

#define SOP716_CMD_READ_RETRY                10

#define SOP716_RESET_DELAY_MS                400

struct sop716_command {
	char *desc;
	u8 size;
};

struct sop716_info {
	struct i2c_client *client;
	struct device *dev;

	int gpio_reset;
	int gpio_interrupt;
	int gpio_bslen;
	int gpio_temp_1;

	int reset_status;
	int tz_minuteswest;
	int batt_check_interval;

	bool update_sysclock_on_boot;
	bool watch_mode;

	struct sop716_command *cmds;

	struct work_struct fw_work;
	struct work_struct sysclock_work;
	struct work_struct init_work;
	struct mutex lock;
};

struct sop716_info *sop716_info;

static inline int REG_CMD(struct sop716_command *cmd, u8 reg, char *desc,
		u8 size)
{
	if (reg >= CMD_SOP716_MAX) {
		pr_err("%s: unknown command %d\n", __func__, reg);
		return -EINVAL;
	}

	cmd[reg].desc = desc;
	cmd[reg].size = size;

	return 0;
}

static int sop716_register_commands(struct sop716_info *si)
{
	int err;

	si->cmds = devm_kzalloc(si->dev,
			sizeof(struct sop716_command) * CMD_SOP716_MAX,
			GFP_KERNEL);
	if (!si->cmds) {
		pr_err("%s: no mem\n", __func__);
		return -ENOMEM;
	}

	err = REG_CMD(si->cmds, CMD_SOP716_SET_CURRENT_TIME,
			"set time", 8);
	err |= REG_CMD(si->cmds, CMD_SOP716_MOTOR_MOVE_ONE,
			"motor move one", 4);
	err |= REG_CMD(si->cmds, CMD_SOP716_MOTOR_MOVE_ALL,
			"motor move all", 4);
	err |= REG_CMD(si->cmds, CMD_SOP716_MOTOR_INIT,
			"motor init", 4);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_CURRENT_TIME,
			"read time", 6);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_FW_VERSION,
			"read fw version", 2);
	err |= REG_CMD(si->cmds, CMD_SOP716_BATTERY_CHECK_PERIOD,
			"battery check period", 3);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_BATTERY_LEVEL,
			"read battery level" , 2);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_BATTERY_CHECK_PERIOD,
			"read battery check period", 2);

	return err;
}

static int sop716_write(struct sop716_info *si, u8 reg, u8 *val)
{
	int ret;

	pr_debug("%s: reg:%d val:%d\n", __func__, val[0], *val);

	if (reg >= CMD_SOP716_MAX) {
		pr_err("%s: unknown command %d\n", __func__, reg);
		return -EINVAL;
	}

	/* sop716 writes size instead of reg */
	ret = i2c_smbus_write_i2c_block_data(si->client,
			si->cmds[reg].size, si->cmds[reg].size, val);
	if (ret < 0)
		pr_err("%s: cannot write i2c: reg %d (%s)\n",
				__func__, reg, si->cmds[reg].desc);

	return ret;
}

static int sop716_read(struct sop716_info *si, u8 reg, u8 *val)
{
	int ret;

	pr_debug("%s: reg:%d val:%d\n", __func__, reg, *val);

	if (reg >= CMD_SOP716_MAX) {
		pr_err("%s: unknown command %d\n", __func__, reg);
		return -EINVAL;
	}

	/* read size + data, So "size + 1" need to be added  */
	ret = i2c_smbus_read_i2c_block_data(si->client,
			reg, si->cmds[reg].size + 1, val);
	if (ret < 0) {
		pr_err("%s: cannot read i2c: reg %d (%s)\n",
				__func__, reg, si->cmds[reg].desc);
		return ret;
	}

	if ((ret - 1) != val[0]) {
		pr_err("%s: error: cmd %d, size %d, ret %d, val[0] %d\n",
				__func__, reg, si->cmds[reg].size, ret, val[0]);
		return -EINVAL;

	}
	return ret;
}

static void sop716_hw_reset(struct sop716_info *si)
{
	gpio_set_value(si->gpio_reset, 1);
	usleep_range(1, 1000);
	gpio_set_value(si->gpio_reset, 0);
	msleep(SOP716_RESET_DELAY_MS);
}

static ssize_t sop716_reset_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	int reset;
	int rc;

	rc = kstrtoint(buf, 10, &reset);
	if (rc) {
		pr_err("%s: invalid value\n", __func__);
		return rc;
	}

	if (reset) {
		mutex_lock(&si->lock);
		sop716_hw_reset(si);
		mutex_unlock(&si->lock);
	}

	pr_debug("%s: reset %d\n", __func__, reset);
	return count;
}

/* Code for CMD_SOP716_SET_CURRENT_TIME */
static ssize_t sop716_time_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	u8 tmp[4];
	int hour, minute, second, year, month, day, option;
	int rc = 0;

	if (count != 18 && count != 19) {
		pr_err("%s: Error!!! invalid input count!\n", __func__);
		return -EINVAL;
	 }

	snprintf(tmp, 4, buf);
	rc = kstrtoint(tmp, 10, &hour);
	snprintf(tmp, 4, buf + 3);
	rc |= kstrtoint(tmp, 10, &minute);
	snprintf(tmp, 4, buf + 6);
	rc |= kstrtoint(tmp, 10, &second);
	snprintf(tmp, 3, buf + 9);
	rc |= kstrtoint(tmp, 10, &year);
	snprintf(tmp, 3, buf + 11);
	rc |= kstrtoint(tmp, 10, &month);
	snprintf(tmp, 3, buf + 13);
	rc |= kstrtoint(tmp, 10, &day);
	snprintf(tmp, 4, buf + 15);
	rc |= kstrtoint(tmp, 10, &option);
	if (rc) {
		pr_err("%s: Error!!! invalid input format! rc:%d\n",
				__func__, rc);
		return rc;
	 }

	if ((hour < 0 || hour > 23) || (minute < 0 || minute > 59) ||
	    (second < 0 || second > 59) || (option < 0 || option > 255)) {
		if (!(hour == 0xFF && minute == 0xFF)) {
			pr_err("%s: Error!!! invalid input format! "
			       "op:%d h:%d m:%d s:%d\n", __func__,
			       option, hour, minute, second);
			return -EINVAL;
		}
	} else if ((year < 0 || year > 99) || (month < 1 || month > 12) ||
		   (day < 1 || day > 31)) {
		pr_err("%s: Error!!! invalid input format! y:%d m:%d d:%d\n",
				__func__, year, month, day);
		return -EINVAL;
	}

	pr_info("sop set time: %04d-%02d-%02d %02d:%02d:%02d\n",
			year + 2000, month, day, hour, minute, second);

	data[0] = CMD_SOP716_SET_CURRENT_TIME;
	data[1] = hour;
	data[2] = minute;
	data[3] = second;
	data[4] = year;
	data[5] = month;
	data[6] = day;
	data[7] = option;

	mutex_lock(&si->lock);
	rc = sop716_write(si, CMD_SOP716_SET_CURRENT_TIME, data);
	if (rc < 0)
		pr_err("%s: cannot set time\n", __func__);

	si->watch_mode = true;
	mutex_unlock(&si->lock);

	return rc < 0? rc : count;
}

/* Code for CMD_SOP716_MOTOR_MOVE_ONE */
static ssize_t sop716_motor_move_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	u8 tmp[4];
	int motor_type, motor_dest, option;
	int rc = 0;

	if (count != 9 && count != 10) {
		pr_err("%s: Error!!! invalid input count!\n", __func__);
		return -EINVAL;
	 }

	snprintf(tmp, 4, buf);
	rc = kstrtoint(tmp, 10, &motor_type);
	snprintf(tmp, 4, buf + 3);
	rc |= kstrtoint(tmp, 10, &motor_dest);
	snprintf(tmp, 4, buf + 6);
	rc |= kstrtoint(tmp, 10, &option);
	if (rc) {
		pr_err("%s: Error!!! invalid input format! rc:%d\n",
				__func__, rc);
		return rc;
	 }

	if ((motor_dest < 0 || motor_dest > 179) ||
	    (option < 0 || option > 255) ||
	    (motor_type < 0 || motor_type> 1 )) {
		pr_err("%s: Error!!! invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d motortype:%d destination:%d option:%d\n",
			__func__, count, motor_type, motor_dest, option);

	data[0] = CMD_SOP716_MOTOR_MOVE_ONE;
	data[1] = motor_type;
	data[2] = motor_dest;
	data[3] = option;

	mutex_lock(&si->lock);
	sop716_write(si, CMD_SOP716_MOTOR_MOVE_ONE, data);

	si->watch_mode = false;
	mutex_unlock(&si->lock);

	return count;
}

/* Code for CMD_SOP716_MOTOR_MOVE_ALL */
static ssize_t sop716_motor_move_all_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	u8 tmp[4];
	int motor1, motor2, option;
	int rc = 0;

	if (count != 9 && count != 10) {
		pr_err("%s: Error!!! invalid input count!\n", __func__);
		return -EINVAL;
	 }

	snprintf(tmp, 4, buf);
	rc = kstrtoint(tmp, 10, &motor1);
	snprintf(tmp, 4, buf + 3);
	rc |= kstrtoint(tmp, 10, &motor2);
	snprintf(tmp, 4, buf + 6);
	rc |= kstrtoint(tmp, 10, &option);
	if (rc) {
		pr_err("%s: Error!!! invalid input format! rc:%d\n",
				__func__, rc);
		return rc;
	 }

	if ((motor1 < 0 || motor1 > 179) || (motor2 < 0 || motor2 > 179) ||
	    (option < 0 || option > 255)) {
		pr_err("%s: Error!!! invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d motor1:%d motor2:%d op:%d\n",
			__func__, count, motor1, motor2, option);

	data[0] = CMD_SOP716_MOTOR_MOVE_ALL;
	data[1] = motor1;
	data[2] = motor2;
	data[3] = option;

	mutex_lock(&si->lock);
	sop716_write(si, CMD_SOP716_MOTOR_MOVE_ALL, data);

	si->watch_mode = false;
	mutex_unlock(&si->lock);

	return count;
}

/* Code for CMD_SOP716_MOTOR_INIT */
static ssize_t sop716_motor_init_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	u8 tmp[4];
	int motor_type, direction, num_of_steps;
	int rc = 0;

	if (count != 9 && count != 10) {
		pr_err("%s: Error!!! invalid input count!\n", __func__);
		return -EINVAL;
	 }

	snprintf(tmp, 4, buf);
	rc = kstrtoint(tmp, 10, &motor_type);
	snprintf(tmp, 4, buf + 3);
	rc |= kstrtoint(tmp, 10, &direction);
	snprintf(tmp, 4, buf + 6);
	rc |= kstrtoint(tmp, 10, &num_of_steps);
	if (rc) {
		pr_err("%s: Error!!! invalid input format! rc:%d\n",
				__func__, rc);
		return -EINVAL;
	 }

	if ((motor_type < 0 || motor_type > 1) ||
	    (direction < 0 || direction > 1) ||
	    (num_of_steps < 0 || num_of_steps > 255)) {
		pr_err("%s: Error!!! invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d motor_type:%d direction:%d num_of_steps:%d\n",
			__func__, count, motor_type, direction, num_of_steps);

	data[0] = CMD_SOP716_MOTOR_INIT;
	data[1] = motor_type;
	data[2] = direction;
	data[3] = num_of_steps;

	mutex_lock(&si->lock);
	sop716_write(si, CMD_SOP716_MOTOR_INIT, data);

	si->watch_mode = false;
	mutex_unlock(&si->lock);

	return count;
}

/* Code for CMD_SOP716_READ_CURRENT_TIME */
static ssize_t sop716_get_time_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	struct sop716_info *si = dev_get_drvdata(dev);
	int err;

	mutex_lock(&si->lock);
	err = sop716_read(si, CMD_SOP716_READ_CURRENT_TIME, data);
	mutex_unlock(&si->lock);
	if (err < 0) {
		pr_err("%s: cannot read time from sop716\n", __func__);
		return err;
	}

	pr_debug("sop get time: %04d-%02d-%02d %02d:%02d:%02d\n",
			data[4] + 2000,
			data[5],
			data[6],
			data[1],
			data[2],
			data[3]);

	return snprintf(buf, PAGE_SIZE, "%04d-%02d-%02d %02d:%02d:%02d\n",
			data[4] + 2000, data[5], data[6],
			data[1], data[2], data[3]);
}

/* Code for CMD_SOP716_READ_FW_VERSION */
static ssize_t sop716_get_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	struct sop716_info *si = dev_get_drvdata(dev);

	mutex_lock(&si->lock);
	sop716_read(si, CMD_SOP716_READ_FW_VERSION, data);
	mutex_unlock(&si->lock);

	return snprintf(buf, PAGE_SIZE, "v%d.%d\n", data[1], data[2]);
}

/* Code for CMD_SOP716_BATTERY_CHECK_PERIOD */
static ssize_t sop716_battery_check_period_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sop716_info *si = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d sec\n", si->batt_check_interval);
}

static ssize_t sop716_battery_check_period_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	int interval;
	int rc = 0;

	if (!count) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	 }

	rc = kstrtoint(buf, 10, &interval);
	if (rc) {
		pr_err("%s: invalid value\n", __func__);
		return rc;
	 }

	if ((interval < 1 || interval > 7200)) {
		pr_err("%s: out of range %d\n", __func__, interval);
		return -EINVAL;
	}

	pr_debug("%s: interval:%d\n", __func__, interval);

	data[0] = CMD_SOP716_BATTERY_CHECK_PERIOD;
	data[1] = (interval >> 8) & 0xff;
	data[2] = interval & 0xff;

	mutex_lock(&si->lock);
	sop716_write(si, CMD_SOP716_BATTERY_CHECK_PERIOD, data);

	sop716_read(si, CMD_SOP716_READ_BATTERY_CHECK_PERIOD, data);
	mutex_unlock(&si->lock);

	if (interval != ((data[1] << 8) + data[2])) {
		pr_err("%s: cannot update the interval\n", __func__);
		return -EINVAL;
	}
	si->batt_check_interval = interval;

	return count;
}

/* Code for CMD_SOP716_READ_BATTERY_LEVEL */
static ssize_t sop716_get_battery_level_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	struct sop716_info *si = dev_get_drvdata(dev);

	mutex_lock(&si->lock);
	sop716_read(si, CMD_SOP716_READ_BATTERY_LEVEL, data);
	mutex_unlock(&si->lock);

	return snprintf(buf, PAGE_SIZE, "%d mV\n", (data[1] << 8) + data[2]);
}

static ssize_t sop716_update_fw_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	int err, value;

	err = kstrtoint(buf, 10, &value);
	if (err) {
		pr_err("%s: invalid value\n", __func__);
		return err;
	}

	if (value)
		schedule_work(&si->fw_work);

	return count;
}

static int sop716_hctosys(struct sop716_info *si)
{
	struct rtc_time tm;
	struct timespec tv = {
		.tv_nsec = NSEC_PER_SEC >> 1,
	};
	u8 time_data[SOP716_I2C_DATA_LENGTH] = {0, };
	int err;

	mutex_lock(&si->lock);
	err = sop716_read(si, CMD_SOP716_READ_CURRENT_TIME, time_data);
	if (err < 0) {
		pr_err("%s: cannot read time from device\n", __func__);
		goto out;
	}

	tm.tm_hour = time_data[1];
	tm.tm_min  = time_data[2];
	tm.tm_sec  = time_data[3];
	tm.tm_year = time_data[4] + 100;
	tm.tm_mon  = time_data[5] - 1;
	tm.tm_mday = time_data[6];

	err = rtc_valid_tm(&tm);
	if (err) {
		pr_err("%s: invalid date/time\n", __func__);
		goto out;
	}

	rtc_tm_to_time(&tm, &tv.tv_sec);

	/* Convert to UTC */
	tv.tv_sec += si->tz_minuteswest * 60;

	/* It will override the system clock */
	err = do_settimeofday(&tv);

	rtc_time_to_tm(tv.tv_sec, &tm);

	dev_info(si->dev, "setting system clock to "
			  "%d-%02d-%02d %02d:%02d:%02d UTC (%u)\n",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec,
			(unsigned int) tv.tv_sec);

out:
	mutex_unlock(&si->lock);
	return err;
}

static ssize_t sop716_update_sysclock_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	int err, value;

	err = kstrtoint(buf, 10, &value);
	if (err) {
		pr_err("%s: invalid value\n", __func__);
		return err;
	}

	if (value) {
		/* read the sys_tz and use it */
		si->tz_minuteswest = sys_tz.tz_minuteswest;

		schedule_work(&si->sysclock_work);
	}

	return count;
}

static ssize_t sop716_watch_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sop716_info *si = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", si->watch_mode);
}

static ssize_t sop716_watch_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	int value = 0;
	int err;
	u8 data[SOP716_I2C_DATA_LENGTH] = {
		CMD_SOP716_SET_CURRENT_TIME,
		0xFF,
		0xFF,
		0,
	};


	if (si->watch_mode)
		return count;

	err = kstrtoint(buf, 10, &value);
	if (err || !value) {
		pr_err("%s: invalid value\n", __func__);
		return err;
	}

	mutex_lock(&si->lock);
	err = sop716_write(si, CMD_SOP716_SET_CURRENT_TIME, data);
	mutex_unlock(&si->lock);
	if (err < 0) {
		pr_err("%s: cannot set watch mode\n", __func__);
		return err;
	}

	si->watch_mode = true;

	return count;
}

static DEVICE_ATTR(reset, S_IWUSR, NULL, sop716_reset_store);
static DEVICE_ATTR(set_time, S_IWUSR, NULL, sop716_time_store);
static DEVICE_ATTR(motor_init, S_IWUSR, NULL, sop716_motor_init_store);
static DEVICE_ATTR(motor_move, S_IWUSR, NULL, sop716_motor_move_store);
static DEVICE_ATTR(motor_move_all, S_IWUSR, NULL, sop716_motor_move_all_store);
static DEVICE_ATTR(get_time, S_IRUGO, sop716_get_time_show, NULL);
static DEVICE_ATTR(get_version, S_IRUGO, sop716_get_version_show, NULL);
static DEVICE_ATTR(battery_check_period, S_IRUGO | S_IWUSR,
		sop716_battery_check_period_show,
		sop716_battery_check_period_store);
static DEVICE_ATTR(get_battery_level, S_IRUGO,
		sop716_get_battery_level_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, sop716_update_fw_store);
static DEVICE_ATTR(update_sysclock, S_IWUSR, NULL,
		sop716_update_sysclock_store);
static DEVICE_ATTR(watch_mode, S_IWUSR | S_IRUGO, sop716_watch_mode_show,
		sop716_watch_mode_store);

static struct attribute *sop716_dev_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_set_time.attr,
	&dev_attr_motor_init.attr,
	&dev_attr_motor_move.attr,
	&dev_attr_motor_move_all.attr,
	&dev_attr_get_time.attr,
	&dev_attr_get_version.attr,
	&dev_attr_battery_check_period.attr,
	&dev_attr_get_battery_level.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_sysclock.attr,
	&dev_attr_watch_mode.attr,
	NULL
};

static struct attribute_group sop716_dev_attr_group = {
	.attrs = sop716_dev_attrs,
};

static int sop716_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sop716_info *si = i2c_get_clientdata(client);

	si->update_sysclock_on_boot = of_property_read_bool(node,
			"lge,update-syslock-on-boot");

	return 0;
}

static int sop716_read_and_request_gpio(struct device *dev,
		const char *dt_name, int *gpio,
		unsigned long gpio_flags, const char *label)
{
	struct device_node *node = dev->of_node;
	int err;

	*gpio = of_get_named_gpio(node, dt_name, 0);
	if (!gpio_is_valid(*gpio)) {
		pr_err("%s: %s not in device tree\n", __func__, dt_name);
		return -EINVAL;
	}

	err = devm_gpio_request_one(dev, *gpio, gpio_flags, label);
	if (err < 0) {
		pr_err("%s: cannot request gpio %d (%s)\n", __func__,
				*gpio, label);
		return err;
	}
	return 0;
}

static int sop716_config_gpios(struct i2c_client *client)
{
	struct sop716_info *si = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int err;

	/* Interrupt pin : input */
	err = sop716_read_and_request_gpio(dev, "lge,gpio-interrupt",
			&si->gpio_interrupt, GPIOF_IN, "interrupt");
	if (err)
		return err;

	/* bslen */
	err = sop716_read_and_request_gpio(dev, "lge,gpio-bslen",
			&si->gpio_bslen, GPIOF_OUT_INIT_LOW, "bslen");
	if (err)
		return err;

	/* gpio temp 1 */
	err = sop716_read_and_request_gpio(dev, "lge,gpio-temp-1",
			&si->gpio_temp_1, GPIOF_OUT_INIT_LOW,
			"temp_1");
	if (err)
		return err;

	/* reset signal high --> low */
	err = sop716_read_and_request_gpio(dev, "lge,gpio-reset",
			&si->gpio_reset, GPIOF_OUT_INIT_LOW, "reset");
	if (err)
		return err;

	return 0;
}

static void sop716_firmware_update(struct sop716_info *si, const u8 *fw)
{
	int err;

	usleep_range(1, 1000);
	gpio_set_value(si->gpio_reset, 1);
	usleep_range(1, 1000);
	gpio_set_value(si->gpio_bslen, 1);
	usleep_range(1, 1000);
	gpio_set_value(si->gpio_bslen, 0);
	usleep_range(1, 1000);
	gpio_set_value(si->gpio_bslen, 1);
	usleep_range(1, 1000);
	gpio_set_value(si->gpio_reset, 0); /* bootloader starts */
	/*
	 * BSLEN must remain logically high long enough for the
	 * boot code to detect its level and enter the BSL sequence
	 */
	msleep(100);
	gpio_set_value(si->gpio_bslen, 0);
	usleep_range(1, 1000);

	err = sop716fw_update_firmware(si->client, fw);
	if (err) {
		pr_warn("%s: failed to update firmware\n", __func__);
		gpio_set_value(si->gpio_bslen, 0);
		gpio_set_value(si->gpio_reset, 1);
		usleep_range(1, 1000);
		gpio_set_value(si->gpio_reset, 0); /* user program starts */
		msleep(100);
	}
}

static void sop716_update_fw_work(struct work_struct *work)
{
	struct sop716_info *si = container_of(work,
			struct sop716_info, fw_work);
	u8 data[SOP716_I2C_DATA_LENGTH] = {0, };
	u8 major, minor, img_major, img_minor;
	int err;
	int retry = SOP716_CMD_READ_RETRY;
	const struct firmware *fw_entry = NULL;

	err = request_firmware(&fw_entry, "sop716.fw", si->dev);
	if (err) {
		pr_err("%s: cannot read firmware\n", __func__);
		return;
	};

	sop716fw_validate_firmware(fw_entry->data, &img_major, &img_minor);
	if (!img_major && !img_minor) {
		pr_err("%s: wrong firmware file\n", __func__);
	}

	mutex_lock(&si->lock);
	sop716_read(si, CMD_SOP716_READ_FW_VERSION, data);

	major = data[1];
	minor = data[2];

	pr_info("sop firmware version: device v%d.%d, image v%d.%d\n",
			major, minor, img_major, img_minor);
	if (major == img_major && minor == img_minor)
		goto out;

	/* Update FW */
	si->watch_mode = false;

	/* Set the position 0 */
	memset(data, 0, sizeof(data));
	data[0] = CMD_SOP716_MOTOR_MOVE_ALL;
	sop716_write(si, CMD_SOP716_MOTOR_MOVE_ALL, data);

	/* save current time */
	err = sop716_read(si, CMD_SOP716_READ_CURRENT_TIME, data);
	if (err < 0)
		pr_err("%s: cannot read current time\n", __func__);

	pr_info("sop get time: %04d-%02d-%02d %02d:%02d:%02d\n",
		data[4] + 2000,
		data[5],
		data[6],
		data[1],
		data[2],
		data[3]);

	/* wait for hands move */
	msleep(2000); /* 2 seconds */

	sop716_firmware_update(si, fw_entry->data);

	/* boot up time */
	msleep(SOP716_RESET_DELAY_MS);

	/* restore time */
	data[0] = CMD_SOP716_SET_CURRENT_TIME;
	data[7] = 0;
	while (retry--) {
		err = sop716_write(si, CMD_SOP716_SET_CURRENT_TIME, data);
		if (!err)
			break;

		msleep(100);
	}

	if (err < 0)
		pr_err("%s: cannot set time\n", __func__);

	si->watch_mode = true;

	/* FIXME: delay */
	msleep(100);
	sop716_read(si, CMD_SOP716_READ_FW_VERSION, data);

	/* FIXME: delay */
	msleep(100);

	pr_info("sop firmware update: v%d.%d -> v%d.%d\n",
			major, minor, data[1], data[2]);
out:
	mutex_unlock(&si->lock);
	release_firmware(fw_entry);
}

static void sop716_update_sysclock_work(struct work_struct *work)
{
	struct sop716_info *si = container_of(work,
			struct sop716_info, sysclock_work);
	int err;

	err = sop716_hctosys(si);
	if (err) {
		/* try it again with reset */
		mutex_lock(&si->lock);
		sop716_hw_reset(si);
		mutex_unlock(&si->lock);
		sop716_hctosys(si);
	}
}

static void sop716_init_work(struct work_struct *work)
{
	struct sop716_info *si = container_of(work,
			struct sop716_info, init_work);

	mutex_lock(&si->lock);
	sop716_hw_reset(si);
	mutex_unlock(&si->lock);
}

static int sop716_movement_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct sop716_info *si;
	int err;

	pr_debug("%s\n", __func__);

	si = devm_kzalloc(&client->dev, sizeof(struct sop716_info),
			GFP_KERNEL);
	if (!si) {
		pr_err("%s: no mem\n", __func__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s: error i2c_check_functionality\n", __func__);
		return  -ENODEV;
	}

	si->client = client;
	si->dev = &client->dev;

	i2c_set_clientdata(client, si);

	err = sop716_parse_dt(client);
	if (err)
		return err;

	err = sop716_config_gpios(client);
	if (err)
		return err;

	err = sop716_register_commands(si);
	if (err)
		return err;

	INIT_WORK(&si->fw_work, sop716_update_fw_work);
	INIT_WORK(&si->sysclock_work, sop716_update_sysclock_work);
	INIT_WORK(&si->init_work, sop716_init_work);

	mutex_init(&si->lock);

	err = sysfs_create_group(&client->dev.kobj, &sop716_dev_attr_group);
	if (err) {
		pr_err("%s: cannot create sysfs\n", __func__);
		goto err_sysfs_create;
	}

	sop716_info = si;

	/* hw init */
	schedule_work(&si->init_work);

	pr_info("sop716 movement probed\n");
	return 0;

err_sysfs_create:
	mutex_destroy(&si->lock);
	i2c_set_clientdata(client, NULL);

	return err;
}

static int sop716_movement_remove(struct i2c_client *client)
{
	struct sop716_info *si = i2c_get_clientdata(client);

	mutex_destroy(&si->lock);
	sysfs_remove_group(&client->dev.kobj, &sop716_dev_attr_group);
	i2c_set_clientdata(client, NULL);
	sop716_info = NULL;

	return 0;
}

static void sop716_movement_shutdown(struct i2c_client *client)
{
	struct sop716_info *si = i2c_get_clientdata(client);
	int err;
	u8 data[SOP716_I2C_DATA_LENGTH] = {
		CMD_SOP716_SET_CURRENT_TIME,
		0xFF,
		0xFF,
		0,
	};

	if (si->watch_mode)
		return;

	err = sop716_write(si, CMD_SOP716_SET_CURRENT_TIME, data);
	if (err < 0)
		pr_err("%s: cannot set watch mode\n", __func__);
}

static struct of_device_id sop716_match_table[] = {
	{ .compatible = "lge,soprod_movement", },
	{},
};
MODULE_DEVICE_TABLE(of, sop716_match_table);

static const struct i2c_device_id sop716_id[] = {
	{"sop716", -1},
	{ },
};

static struct i2c_driver sop716_movement_driver = {
	.driver = {
		.name = "soprod716_movement",
		.owner = THIS_MODULE,
		.of_match_table = sop716_match_table,
	},
	.probe    = sop716_movement_probe,
	.remove   = sop716_movement_remove,
	.shutdown = sop716_movement_shutdown,
	.id_table = sop716_id,
};

/**
 * sop716_movement_init() - initialized sop716 movement driver
 * This function registers the sop716 movement platform driver.
 *
 * Returns 0 on success or errno on failure.
 */
static int __init sop716_movement_init(void)
{
	return i2c_add_driver(&sop716_movement_driver);
}

static void __exit sop716_movement_exit(void)
{
	i2c_del_driver(&sop716_movement_driver);
}

module_init(sop716_movement_init);
module_exit(sop716_movement_exit);

static int __init sop716_hctosys_init(void)
{
	struct sop716_info *si = sop716_info;
	int err;

	if (!si) {
		pr_err("%s: unable to open sop716 device\n", __func__);
		return -ENODEV;
	}

	if (!si->update_sysclock_on_boot)
		return 0;

	/* TODO: update tz_minuteswest before sop716_hctosys() */

	err = sop716_hctosys(si);

	return err;
}

late_initcall_sync(sop716_hctosys_init);

MODULE_DESCRIPTION("Soprod 716 movement");
MODULE_LICENSE("GPL v2");

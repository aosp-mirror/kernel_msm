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
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/syscalls.h>

#define SOP716_I2C_DATA_LENGTH       4
#define SOP716_I2C_DATA_LENGTH_TIME  8

#define CMD_SOP716_SET_CURRENT_TIME          0
#define CMD_SOP716_MOTOR_MOVE_ONE            1
#define CMD_SOP716_MOTOR_MOVE_ALL            2
#define CMD_SOP716_MOTOR_INIT                3
#define CMD_SOP716_READ_CURRENT_TIME         4
#define CMD_SOP716_READ_FW_VERSION           5
#define CMD_SOP716_BATTERY_CHECK_PERIOD      7
#define CMD_SOP716_READ_BATTERY_LEVEL        8
#define CMD_SOP716_READ_BATTERY_CHECK_PERIOD 9

#define MAJOR_VER 1
#define MINOR_VER 10


struct timer_list sop_demo_timer;

struct sop716_info {
	struct i2c_client *client;
	struct device *dev;

	int reset_pin;
	int interrupt_pin;
	int i2c_ldo_pin;
	int bslen_pin;
	int gpio_temp_1_pin;

	struct mutex lock;
	struct delayed_work movement_work;
};

struct sop716_info *sop716;

static int hands_alignment_status = 1;
static int sop_align_position = 0;
static int reset_status = 0;

static ssize_t sop716_hands_alignment_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", hands_alignment_status);
}

static ssize_t sop716_hands_alignment_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int status, rc;

	rc = kstrtoint(buf, 10, &status);
	if (rc) {
		pr_err("%s: kstrtoint failed. rc:%d\n", __func__, rc);
		return rc;
	}

	if (status > 1 || status < 0) {
		pr_err("%s: Error!!! hands_alignment:%d\n", __func__, status);
		return -EINVAL;
	}

	hands_alignment_status = status;
	pr_debug("%s: count:%d hands_alignment:%d\n", __func__, count, status);

	return count;
}

static ssize_t sop716_hands_position_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", sop_align_position);
}

static ssize_t sop716_reset_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
       return snprintf(buf, PAGE_SIZE, "%d", reset_status);
}

static ssize_t sop716_reset_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sop716_info *dd = dev_get_drvdata(dev);
	int status, rc;

	rc = kstrtoint(buf, 10, &status);
	if (rc) {
		pr_err("%s: kstrtoint failed. rc:%d\n", __func__, rc);
		return rc;
	}

	if (status > 1 || status < 0) {
		pr_err("%s: Error!!! reset_status:%d\n", __func__, status);
		return -EINVAL;
	}

	reset_status = status;
	gpio_set_value(dd->reset_pin, reset_status);

	pr_debug("%s: count:%d reset:%d\n", __func__, count, reset_status);

	return count;
}

/* Code for CMD_SOP716_SET_CURRENT_TIME */
static int sop716_write(struct sop716_info *dd, u8 reg, u8 length, u8 *val);
static int sop716_read(struct sop716_info *dd, u8 reg, u8 length, u8 *val);

static ssize_t sop716_time_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sop716_info *dd = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH_TIME];
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
		pr_err("%s: Error!!! invalid input format! rc:%d\n", __func__, rc);
		return rc;
	 }

	if ((hour < 0 || hour > 23) || (minute < 0 || minute > 59) ||
		(second < 0 || second > 59) || (option < 0 || option > 255)) {
		if (!(hour == 0xFF && minute == 0xFF)) {
			pr_err("%s: Error!!! invalid input format! op:%d h:%d m:%d s:%d\n",
					__func__, option, hour, minute, second);
			return -EINVAL;
		}
	} else if ((year < 0 || year > 99) || (month < 1 || month > 12) ||
		(day < 1 || day > 31)) {
		pr_err("%s: Error!!! invalid input format! y:%d m:%d d:%d\n",
				__func__, year, month, day);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d op:%d %02d:%02d:%02d %02d-%02d-%02d\n",
		__func__, count, option, hour, minute, second, year, month, day);

	data[0] = CMD_SOP716_SET_CURRENT_TIME;
	data[1] = hour;
	data[2] = minute;
	data[3] = second;
	data[4] = year;
	data[5] = month;
	data[6] = day;
	data[7] = option;

	sop716_write(dd, SOP716_I2C_DATA_LENGTH_TIME,
			SOP716_I2C_DATA_LENGTH_TIME, data);

	return count;
}

/* Code for CMD_SOP716_MOTOR_MOVE_ONE */
static ssize_t sop716_motor_move_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sop716_info *dd = dev_get_drvdata(dev);
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
		pr_err("%s: Error!!! invalid input format! rc:%d\n", __func__, rc);
		return rc;
	 }

	if ((motor_dest < 0 || motor_dest > 179) || (option < 0 || option > 255) ||
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

	sop716_write(dd, SOP716_I2C_DATA_LENGTH, SOP716_I2C_DATA_LENGTH, data);

	return count;
}

/* Code for CMD_SOP716_MOTOR_MOVE_ALL */
static ssize_t sop716_motor_move_all_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sop716_info *dd = dev_get_drvdata(dev);
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
		pr_err("%s: Error!!! invalid input format! rc:%d\n", __func__, rc);
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

	sop716_write(dd, SOP716_I2C_DATA_LENGTH, SOP716_I2C_DATA_LENGTH, data);

	return count;
}

/* Code for CMD_SOP716_MOTOR_INIT */
static ssize_t sop716_motor_init_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sop716_info *dd = dev_get_drvdata(dev);

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
		pr_err("%s: Error!!! invalid input format! rc:%d\n", __func__, rc);
		return -EINVAL;
	 }

	if ((motor_type < 0 || motor_type > 1) || (direction < 0 || direction > 1)
		|| (num_of_steps < 0 || num_of_steps > 255)) {
		pr_err("%s: Error!!! invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d motor_type:%d direction:%d num_of_steps:%d\n",
			__func__, count, motor_type, direction, num_of_steps);

	data[0] = CMD_SOP716_MOTOR_INIT;
	data[1] = motor_type;
	data[2] = direction;
	data[3] = num_of_steps;

	sop716_write(dd, SOP716_I2C_DATA_LENGTH, SOP716_I2C_DATA_LENGTH, data);

	return count;
}

/* Code for CMD_SOP716_READ_CURRENT_TIME */
static ssize_t sop716_get_time_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH_TIME-1];
	struct sop716_info *dd = dev_get_drvdata(dev);

	sop716_read(dd, CMD_SOP716_READ_CURRENT_TIME,
			SOP716_I2C_DATA_LENGTH_TIME-1, data);

	return snprintf(buf, PAGE_SIZE, "%dh%dm%ds, 20%d-%d-%d\n",
			data[1], data[2], data[3], data[4], data[5], data[6]);
}

/* Code for CMD_SOP716_READ_FW_VERSION */
static ssize_t sop716_get_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH-1];
	struct sop716_info *dd = dev_get_drvdata(dev);

	sop716_read(dd, CMD_SOP716_READ_FW_VERSION,
			SOP716_I2C_DATA_LENGTH-1, data);

	return snprintf(buf, PAGE_SIZE, "v%d.%d\n", data[1], data[2]);
}

/* Code for CMD_SOP716_BATTERY_CHECK_PERIOD */
static ssize_t sop716_battery_check_period_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH-2];
	struct sop716_info *dd = dev_get_drvdata(dev);

	sop716_read(dd, CMD_SOP716_READ_BATTERY_CHECK_PERIOD,
			SOP716_I2C_DATA_LENGTH-2, data);

	return snprintf(buf, PAGE_SIZE, "%d min\n", data[1]);
}

static ssize_t sop716_battery_check_period_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct sop716_info *dd = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH-2];
	u8 tmp[4];
	int interval;
	int rc = 0;

	if (count != 3 && count != 4) {
		pr_err("%s: Error!!! invalid input count!\n", __func__);
		return -EINVAL;
	 }

	snprintf(tmp, 4, buf);
	rc = kstrtoint(tmp, 10, &interval);

	if (rc) {
		pr_err("%s: Error!!! invalid input format! rc:%d\n", __func__, rc);
		return rc;
	 }

	if ((interval < 0 || interval > 60)) {
		pr_err("%s: Error!!! invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d interval:%d\n",
			__func__, count, interval);

	data[0] = CMD_SOP716_BATTERY_CHECK_PERIOD;
	data[1] = interval;

	sop716_write(dd, SOP716_I2C_DATA_LENGTH-2, SOP716_I2C_DATA_LENGTH-2, data);

	return count;
}

/* Code for CMD_SOP716_READ_BATTERY_LEVEL */
static ssize_t sop716_get_battery_level_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH-1];
	struct sop716_info *dd = dev_get_drvdata(dev);

	sop716_read(dd, CMD_SOP716_READ_BATTERY_LEVEL,
			SOP716_I2C_DATA_LENGTH-1, data);

	return snprintf(buf, PAGE_SIZE, "%d mV\n", data[1]*256 + data[2]);
}

static struct device_attribute sop716_device_attrs[] = {
	__ATTR(set_hands_alignment, S_IRUGO | S_IWUSR,
				sop716_hands_alignment_show, sop716_hands_alignment_store),
	__ATTR(set_reset, S_IRUGO | S_IWUSR,
				sop716_reset_show, sop716_reset_store),
	__ATTR(set_time, S_IWUSR, NULL, sop716_time_store),
	__ATTR(motor_init, S_IWUSR, NULL, sop716_motor_init_store),
	__ATTR(motor_move, S_IWUSR, NULL, sop716_motor_move_store),
	__ATTR(motor_move_all, S_IWUSR, NULL, sop716_motor_move_all_store),
	__ATTR(get_hands_position, S_IRUGO, sop716_hands_position_show, NULL),
	__ATTR(get_time, S_IRUGO, sop716_get_time_show, NULL),
	__ATTR(get_version, S_IRUGO, sop716_get_version_show, NULL),
	__ATTR(battery_check_period, S_IRUGO | S_IWUSR,
				sop716_battery_check_period_show, sop716_battery_check_period_store),
	__ATTR(get_battery_level, S_IRUGO, sop716_get_battery_level_show, NULL),
};

static int sop716_write(struct sop716_info *dd, u8 reg, u8 length, u8 *val)
{
	int ret;
	pr_info("%s: reg:%d val:%d\n", __func__, val[0], *val);

	ret = i2c_smbus_write_i2c_block_data(dd->client, reg, length, val);

	if (ret < 0)
		pr_err("%s: i2c_smbus_write_byte failed error %d\n", __func__, ret);

	return ret;
}

static int sop716_read(struct sop716_info *dd, u8 reg, u8 length, u8 *val)
{
	int ret;
	pr_info("%s: reg:%d val:%d\n", __func__, reg, *val);

	ret = i2c_smbus_read_i2c_block_data(dd->client, reg, length, val);

	if (ret < 0)
		pr_err("%s: i2c_smbus_read_byte failed error:%d register:%d\n",
				__func__, ret, reg);

	return ret;
}

static int sop716_parse_dt(struct device_node *node, struct sop716_info *dd)
{
	int rc = 0;

	return rc;
}

static int sop716_parse_gpio(struct device_node *node, struct sop716_info *dd)
{
	int ret = 0;
	int error = 0;

	/* Interrupt pin : input */
	if ((dd->interrupt_pin =
			of_get_named_gpio_flags(node, "soprod,interrupt", 0, NULL)) > 0) {
		error = gpio_request(dd->interrupt_pin, "soprod,interrupt");
		if (error < 0) {
			pr_err("FAIL: soprod,interrupt_pin gpio_request\n");
			return -1;
		}
		gpio_direction_input(dd->interrupt_pin);
	} else {
		pr_err("%s: interrupt gpio error:%d\n", __func__, dd->interrupt_pin);
		return -1;
	}

	/* i2c ldo enable */
	if ((dd->i2c_ldo_pin =
			of_get_named_gpio_flags(node, "soprod,i2c_ldo", 0, NULL)) > 0) {
		error = gpio_request(dd->i2c_ldo_pin, "soprod,i2c_ldo");
		if (error < 0) {
			pr_err("%s: FAIL: soprod,i2c_ldo gpio_request\n", __func__);
			return -1;
		}
		gpio_direction_output(dd->i2c_ldo_pin, 1);

		msleep(100);
		gpio_set_value(dd->i2c_ldo_pin, 1);
	} else {
		pr_err("%s: i2c_ldo gpio error:%d\n",__func__, dd->i2c_ldo_pin);
		return -1;
	}

	/* bslen */
	if ((dd->bslen_pin =
			of_get_named_gpio_flags(node, "soprod,bslen", 0, NULL)) > 0) {
		error = gpio_request(dd->bslen_pin, "soprod,bslen");
		if (error < 0) {
			pr_err("%s: FAIL: soprod,bslen gpio_request\n", __func__);
			return -1;
		}
		gpio_direction_output(dd->bslen_pin, 0);

		msleep(100);
		gpio_set_value(dd->bslen_pin, 0);
	} else {
		pr_err("%s: bslen gpio error:%d\n",__func__, dd->bslen_pin);
		return -1;
	}

	/* gpio temp 1 */
	if ((dd->gpio_temp_1_pin =
			of_get_named_gpio_flags(node, "soprod,gpio_temp_1", 0, NULL)) > 0) {
		error = gpio_request(dd->gpio_temp_1_pin, "soprod,gpio_temp_1");
		if (error < 0) {
			pr_err("%s: FAIL: soprod,gpio_temp_1 gpio_request\n", __func__);
			return -1;
		}
		gpio_direction_output(dd->gpio_temp_1_pin, 0);

		msleep(100);
		gpio_set_value(dd->gpio_temp_1_pin, 0);
	} else {
		pr_err("%s: gpio_temp_1_pin gpio error:%d\n",
				__func__, dd->gpio_temp_1_pin);
		return -1;
	}

	/* reset signal high --> low */
	if ((dd->reset_pin =
			of_get_named_gpio_flags(node, "soprod,reset", 0, NULL)) > 0) {
		error = gpio_request(dd->reset_pin, "soprod,reset");
		if (error < 0) {
			pr_err("%s: FAIL: soprod,reset gpio_request\n", __func__);
			return -1;
		}
		gpio_direction_output(dd->reset_pin, 0);
		msleep(100);
		gpio_set_value(dd->reset_pin, 0);
		msleep(100);
		gpio_set_value(dd->reset_pin, 1);
		msleep(100);
		gpio_set_value(dd->reset_pin, 0);
	} else {
		pr_err("%s: reset gpio error:%d\n", __func__, dd->reset_pin);
		return -1;
	}

	pr_info("%s: reset gpio:%d interrupt gpio:%d\n",
			__func__, dd->reset_pin, dd->interrupt_pin);
	return ret;
}

int init_sop_demo_first = 0;
static int align_key_state = 0;
static int align_hand_state = 0;

static void sop_firmware_update(struct sop716_info *chip)
{
	extern int msp430_firmware_update_start(struct device *dev);

	gpio_direction_output(sop716->bslen_pin, 1);
	msleep(100);
	gpio_set_value(sop716->reset_pin, 1);
	gpio_set_value(sop716->bslen_pin, 1);
	msleep(100);
	gpio_set_value(sop716->bslen_pin, 0);
	msleep(100);
	gpio_set_value(sop716->bslen_pin, 1);
	msleep(100);
	gpio_set_value(sop716->reset_pin, 0);
	msleep(100);
	gpio_set_value(sop716->bslen_pin, 0);
	msleep(100);

	msp430_firmware_update_start(chip->dev);
	gpio_set_value(sop716->bslen_pin, 1);
	msleep(100);
	gpio_set_value(sop716->bslen_pin, 0);
	msleep(100);
	gpio_set_value(sop716->reset_pin, 1);
	msleep(100);
	gpio_set_value(sop716->reset_pin, 0);
	msleep(100);
}

static void sop_movement_work(struct work_struct *work)
{
	struct timespec my_time;
	struct tm my_date;
	struct delayed_work *delayed_work;
	struct sop716_info *chip;

	delayed_work = container_of(work, struct delayed_work, work);
	chip = container_of(delayed_work, struct sop716_info, movement_work);


	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);

	pr_info("sop kernel time %02d:%02d:%02d, date %02d-%02d-%02d\n",
			my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
			(u8)my_date.tm_year, my_date.tm_mon+1, my_date.tm_mday);

	if (sop_align_position == 0) {
		u8 align_data[SOP716_I2C_DATA_LENGTH] =
				{CMD_SOP716_MOTOR_MOVE_ALL, 45, 135, 64};
		sop_align_position++;
		sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
	} else if(sop_align_position == 1) {
		u8 align_data[SOP716_I2C_DATA_LENGTH_TIME] =
				{CMD_SOP716_SET_CURRENT_TIME, 0xFF, 0xFF, 0, 0, 0, 0, 0};
		sop_align_position = 0;
		sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
	} else if(sop_align_position == 2) {
		u8 align_data[SOP716_I2C_DATA_LENGTH] =
				{CMD_SOP716_MOTOR_MOVE_ALL, 0, 0, 0};
		unsigned long timeout = 2;

		del_timer(&sop_demo_timer);
		sop_demo_timer.expires = jiffies + (timeout * HZ);
		sop_demo_timer.data = timeout;
		add_timer(&sop_demo_timer);

		sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
	} else if(sop_align_position == 3) {
		unsigned long timeout = 4;
		u8 align_data[SOP716_I2C_DATA_LENGTH] =
				{CMD_SOP716_MOTOR_MOVE_ALL, 179, 179, 96};
		u8 align_data2[SOP716_I2C_DATA_LENGTH] =
				{CMD_SOP716_MOTOR_MOVE_ALL, 0, 0, 96};

		del_timer(&sop_demo_timer);
		sop_demo_timer.expires = jiffies + (timeout * HZ);
		sop_demo_timer.data = timeout;
		add_timer(&sop_demo_timer);

		sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
		msleep(200);
		sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data2);
	} else if(sop_align_position == 4) {
		unsigned long timeout = 33;
		u8 align_data[SOP716_I2C_DATA_LENGTH] =
				{CMD_SOP716_MOTOR_MOVE_ALL, 179, 179, 224};
		u8 align_data2[SOP716_I2C_DATA_LENGTH] =
				{CMD_SOP716_MOTOR_MOVE_ALL, 0, 0, 96};

		del_timer(&sop_demo_timer);
		sop_demo_timer.expires = jiffies + (timeout * HZ);
		sop_demo_timer.data = timeout;
		add_timer(&sop_demo_timer);

		sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
		msleep(200);
		sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data2);
	} else if(sop_align_position == 5) {
		unsigned long timeout = 13;
		u8 data[SOP716_I2C_DATA_LENGTH-1];

		sop716_read(sop716, CMD_SOP716_READ_FW_VERSION,
				SOP716_I2C_DATA_LENGTH-1, data);
		pr_info("sop firmware version: v%d.%d\n", data[1], data[2]);

		if( !(data[1] == MAJOR_VER && data[2] == MINOR_VER)) {
			struct timespec system_time;
			struct tm movement_time;
			pr_info("sop_firmware_update : v%d.%d -> v%d.%d\n",
					data[1], data[2], MAJOR_VER, MINOR_VER);
			sop_firmware_update(chip);
			system_time = __current_kernel_time();
			time_to_tm(system_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1),
					&movement_time);
			{
				u8 time_data[SOP716_I2C_DATA_LENGTH_TIME] =
					{ CMD_SOP716_SET_CURRENT_TIME, movement_time.tm_hour,
					movement_time.tm_min, movement_time.tm_sec,
					movement_time.tm_year, movement_time.tm_mon,
					movement_time.tm_mday };
				sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
						SOP716_I2C_DATA_LENGTH, time_data);
			}
		} else {
			u8 time_data[SOP716_I2C_DATA_LENGTH_TIME-1];
			struct tm movement_time;
			struct timespec system_time;
			{
				u8 time_data[SOP716_I2C_DATA_LENGTH_TIME] =
					{ CMD_SOP716_SET_CURRENT_TIME, 0xFF, 0xFF, 0, 0, 0, 0, 0 };
				sop716_write(sop716, SOP716_I2C_DATA_LENGTH,
						SOP716_I2C_DATA_LENGTH, time_data);
			}
			sop716_read(sop716, CMD_SOP716_READ_CURRENT_TIME,
					SOP716_I2C_DATA_LENGTH_TIME-1, time_data);
			movement_time.tm_hour = time_data[1];
			movement_time.tm_min = time_data[2];
			movement_time.tm_sec = time_data[3];
			movement_time.tm_year = time_data[4] + 2000;
			movement_time.tm_mon = time_data[5];
			movement_time.tm_mday = time_data[6];
			system_time.tv_sec = mktime(movement_time.tm_year,
					movement_time.tm_mon, movement_time.tm_mday,
					movement_time.tm_hour, movement_time.tm_min,
					movement_time.tm_sec) + sys_tz.tz_minuteswest * 60;
			system_time.tv_nsec = 0;
			do_settimeofday(&system_time);
		}

		del_timer(&sop_demo_timer);
		sop_demo_timer.expires = jiffies + (timeout * HZ);
		sop_demo_timer.data = timeout;
		add_timer(&sop_demo_timer);
	}
}

static void sop_demo_start(unsigned long timeout)
{
	pr_info("+sop_demo_start\n");

	if (init_sop_demo_first == 0) {
		init_sop_demo_first++;
		timeout = 6;
		sop_demo_timer.expires = jiffies + (timeout * HZ);
		sop_demo_timer.data = timeout;
		INIT_DELAYED_WORK(&sop716->movement_work, sop_movement_work);
		add_timer(&sop_demo_timer);
	} else if (init_sop_demo_first == 1) {
		init_sop_demo_first++;
		sop_align_position = 2;
		schedule_delayed_work(&sop716->movement_work, msecs_to_jiffies(0));
	} else if (init_sop_demo_first == 2) {
		init_sop_demo_first++;
		sop_align_position = 3;
		schedule_delayed_work(&sop716->movement_work, msecs_to_jiffies(0));
	} else if (init_sop_demo_first == 3) {
		init_sop_demo_first++;
		sop_align_position = 4;
		schedule_delayed_work(&sop716->movement_work, msecs_to_jiffies(0));
	} else if (init_sop_demo_first == 4) {
		init_sop_demo_first++;
		sop_align_position = 5;
		schedule_delayed_work(&sop716->movement_work, msecs_to_jiffies(0));
	} else {
		if (align_key_state == 0) {
			sop_align_position = 1;
			schedule_delayed_work(&sop716->movement_work, msecs_to_jiffies(0));
		} else {
			sop_align_position = 0;
			schedule_delayed_work(&sop716->movement_work, msecs_to_jiffies(0));
			align_hand_state = 1;
		}
	}
	pr_info("-sop_demo_start\n");
}

void sop_key_align(int state)
{
	if((hands_alignment_status == 1) && (init_sop_demo_first > 4)) {
		if (state == 1) {
			unsigned long timeout = 250; // in milliseconds
			align_key_state = 1;
			del_timer(&sop_demo_timer);

			sop_demo_timer.expires = jiffies + (timeout * HZ / 1000);
			sop_demo_timer.data = timeout;
			add_timer(&sop_demo_timer);
		} else if (state == 0 && align_key_state == 1) {
			del_timer(&sop_demo_timer);
			align_key_state = 0;
			if (align_hand_state == 1) {
				align_hand_state = 0;
				sop_align_position = 1;
				schedule_delayed_work(&sop716->movement_work,
						msecs_to_jiffies(0));
			}
		}
	}
}

static int sop716_init(struct i2c_client *client, struct sop716_info *dd)
{
	int rc;

	rc = sop716_parse_dt(client->dev.of_node, dd);
	if (rc) {
		pr_err("%s: error1 rc(%d)\n", __func__, rc);
		return rc;
	}

	rc = sop716_parse_gpio(client->dev.of_node, dd);
	if (rc) {
		pr_err("%s: error2 rc(%d)\n", __func__, rc);
		return rc;
	}

	sop_demo_start(30);

	return rc;
}

static int sop716_movement_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int i;
	int err;

	pr_info("+sop716_movement_probe\n");

	sop716 = devm_kzalloc(&client->dev, sizeof(struct sop716_info), GFP_KERNEL);
	if (!sop716)
		return -ENOMEM;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s: error i2c_check_functionality\n", __func__);
		return  -ENODEV;
	}

	sop716->client = client;
	sop716->dev = &client->dev;

	for (i = 0; i < ARRAY_SIZE(sop716_device_attrs); i++) {
		err = device_create_file(&client->dev, &sop716_device_attrs[i]);
		if (err) {
			pr_err("%s: cannot create device file\n", __func__);
			return err;
		}
	}

	i2c_set_clientdata(client, sop716);

	sop716_init(client, sop716);

	pr_info("-sop716_movement_probe\n");
	return 0;
}

static int sop716_movement_remove(struct i2c_client *client)
{
	//struct sop716_info *dd = i2c_get_clientdata(client);

	return 0;
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
	.probe = sop716_movement_probe,
	.remove = sop716_movement_remove,
	.id_table = sop716_id,
};

/**
 * sop716_movement_init() - initialized sop716 movement driver
 * This function registers the sop716 movement platform driver.
 *
 * Returns 0 on success or errno on failure.
 */
int __init sop716_movement_init(void)
{
	static bool initialized;

	init_timer(&sop_demo_timer);
	sop_demo_timer.function = sop_demo_start;

	pr_info("+sop716_movement_init\n");

	if (initialized)
		return 0;
	else
		initialized = true;

	return i2c_add_driver(&sop716_movement_driver);
}
EXPORT_SYMBOL(sop716_movement_init);
subsys_initcall(sop716_movement_init);

static void __exit sop716_movement_exit(void)
{
	i2c_del_driver(&sop716_movement_driver);
	del_timer_sync(&sop_demo_timer);
}

module_exit(sop716_movement_exit);
MODULE_DESCRIPTION("Soprod 716 movement");
MODULE_LICENSE("GPL v2");

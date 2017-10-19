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
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/time.h>

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
#define MINOR_VER 12

struct sop716_info {
	struct i2c_client *client;
	struct device *dev;

	int reset_pin;
	int interrupt_pin;
	int bslen_pin;
	int gpio_temp_1_pin;

	struct mutex lock;
	struct work_struct movement_work;

	int hands_alignment_status;
	int sop_align_position;
	int init_sop_demo_first;
	int reset_status;
	int align_key_state;
	int align_hand_state;

	struct timer_list demo_timer;
};

static int sop716_write(struct sop716_info *si, u8 reg, u8 length, u8 *val);
static int sop716_read(struct sop716_info *si, u8 reg, u8 length, u8 *val);

static ssize_t sop716_hands_alignment_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sop716_info *si = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d", si->hands_alignment_status);
}

static ssize_t sop716_hands_alignment_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
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

	si->hands_alignment_status = status;
	pr_debug("%s: count:%d hands_alignment:%d\n", __func__, count, status);

	return count;
}

static ssize_t sop716_hands_position_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sop716_info *si = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d", si->sop_align_position);
}

static ssize_t sop716_reset_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sop716_info *si = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d", si->reset_status);
}

static ssize_t sop716_reset_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
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

	si->reset_status = status;
	gpio_set_value(si->reset_pin, si->reset_status);

	pr_debug("%s: count:%d reset:%d\n", __func__, count, si->reset_status);

	return count;
}

/* Code for CMD_SOP716_SET_CURRENT_TIME */
static ssize_t sop716_time_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
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

	pr_debug("%s: cnt:%d op:%d %02d:%02d:%02d %02d-%02d-%02d\n", __func__,
			count, option, hour, minute, second, year, month, day);

	data[0] = CMD_SOP716_SET_CURRENT_TIME;
	data[1] = hour;
	data[2] = minute;
	data[3] = second;
	data[4] = year;
	data[5] = month;
	data[6] = day;
	data[7] = option;

	sop716_write(si, SOP716_I2C_DATA_LENGTH_TIME,
			SOP716_I2C_DATA_LENGTH_TIME, data);

	return count;
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

	sop716_write(si, SOP716_I2C_DATA_LENGTH, SOP716_I2C_DATA_LENGTH, data);

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

	sop716_write(si, SOP716_I2C_DATA_LENGTH, SOP716_I2C_DATA_LENGTH, data);

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

	sop716_write(si, SOP716_I2C_DATA_LENGTH, SOP716_I2C_DATA_LENGTH, data);

	return count;
}

/* Code for CMD_SOP716_READ_CURRENT_TIME */
static ssize_t sop716_get_time_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH_TIME-1];
	struct sop716_info *si = dev_get_drvdata(dev);

	sop716_read(si, CMD_SOP716_READ_CURRENT_TIME,
			SOP716_I2C_DATA_LENGTH_TIME-1, data);

	return snprintf(buf, PAGE_SIZE, "%dh%dm%ds, 20%d-%d-%d\n",
			data[1], data[2], data[3], data[4], data[5], data[6]);
}

/* Code for CMD_SOP716_READ_FW_VERSION */
static ssize_t sop716_get_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH-1];
	struct sop716_info *si = dev_get_drvdata(dev);

	sop716_read(si, CMD_SOP716_READ_FW_VERSION,
			SOP716_I2C_DATA_LENGTH-1, data);

	return snprintf(buf, PAGE_SIZE, "v%d.%d\n", data[1], data[2]);
}

/* Code for CMD_SOP716_BATTERY_CHECK_PERIOD */
static ssize_t sop716_battery_check_period_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH-1];
	struct sop716_info *si = dev_get_drvdata(dev);

	sop716_read(si, CMD_SOP716_READ_BATTERY_CHECK_PERIOD,
			SOP716_I2C_DATA_LENGTH-1, data);

	return snprintf(buf, PAGE_SIZE, "%d sec\n", data[1]*256 + data[2]);
}

static ssize_t sop716_battery_check_period_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH-1];
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
		pr_err("%s: Error!!! invalid input format! rc:%d\n",
				__func__, rc);
		return rc;
	 }

	if ((interval < 0 || interval > 240)) {
		pr_err("%s: Error!!! invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d interval:%d\n",
			__func__, count, interval);

	data[0] = CMD_SOP716_BATTERY_CHECK_PERIOD;
	data[1] = 0;
	data[2] = interval;

	sop716_write(si, SOP716_I2C_DATA_LENGTH-1, SOP716_I2C_DATA_LENGTH-1,
			data);

	return count;
}

/* Code for CMD_SOP716_READ_BATTERY_LEVEL */
static ssize_t sop716_get_battery_level_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH-1];
	struct sop716_info *si = dev_get_drvdata(dev);

	sop716_read(si, CMD_SOP716_READ_BATTERY_LEVEL,
			SOP716_I2C_DATA_LENGTH-1, data);

	return snprintf(buf, PAGE_SIZE, "%d mV\n", (data[1] * 256) + data[2]);
}

static DEVICE_ATTR(set_hands_alignment, S_IRUGO | S_IWUSR,
		sop716_hands_alignment_show, sop716_hands_alignment_store);
static DEVICE_ATTR(set_reset, S_IRUGO | S_IWUSR,
		sop716_reset_show, sop716_reset_store);
static DEVICE_ATTR(set_time, S_IWUSR, NULL, sop716_time_store);
static DEVICE_ATTR(motor_init, S_IWUSR, NULL, sop716_motor_init_store);
static DEVICE_ATTR(motor_move, S_IWUSR, NULL, sop716_motor_move_store);
static DEVICE_ATTR(motor_move_all, S_IWUSR, NULL, sop716_motor_move_all_store);
static DEVICE_ATTR(get_hands_position, S_IRUGO,
		sop716_hands_position_show, NULL);
static DEVICE_ATTR(get_time, S_IRUGO, sop716_get_time_show, NULL);
static DEVICE_ATTR(get_version, S_IRUGO, sop716_get_version_show, NULL);
static DEVICE_ATTR(battery_check_period, S_IRUGO | S_IWUSR,
		sop716_battery_check_period_show,
		sop716_battery_check_period_store);
static DEVICE_ATTR(get_battery_level, S_IRUGO,
		sop716_get_battery_level_show, NULL);

static struct attribute *sop716_dev_attrs[] = {
	&dev_attr_set_hands_alignment.attr,
	&dev_attr_set_reset.attr,
	&dev_attr_set_time.attr,
	&dev_attr_motor_init.attr,
	&dev_attr_motor_move.attr,
	&dev_attr_motor_move_all.attr,
	&dev_attr_get_hands_position.attr,
	&dev_attr_get_time.attr,
	&dev_attr_get_version.attr,
	&dev_attr_battery_check_period.attr,
	&dev_attr_get_battery_level.attr,
	NULL
};

static struct attribute_group sop716_dev_attr_group = {
	.attrs = sop716_dev_attrs,
};

static int sop716_write(struct sop716_info *si, u8 reg, u8 length, u8 *val)
{
	int ret;

	pr_debug("%s: reg:%d val:%d\n", __func__, val[0], *val);

	ret = i2c_smbus_write_i2c_block_data(si->client, reg, length, val);
	if (ret < 0)
		pr_err("%s: i2c_smbus_write_byte failed error %d\n",
				__func__, ret);

	return ret;
}

static int sop716_read(struct sop716_info *si, u8 reg, u8 length, u8 *val)
{
	int ret;

	pr_debug("%s: reg:%d val:%d\n", __func__, reg, *val);

	ret = i2c_smbus_read_i2c_block_data(si->client, reg, length, val);
	if (ret < 0)
		pr_err("%s: i2c_smbus_read_byte failed error:%d register:%d\n",
				__func__, ret, reg);

	return ret;
}

static int sop716_parse_dt(struct i2c_client *client)
{
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
	err = sop716_read_and_request_gpio(dev, "soprod,interrupt",
			&si->interrupt_pin, GPIOF_IN, "interrupt");
	if (err)
		return err;

	/* bslen */
	err = sop716_read_and_request_gpio(dev, "soprod,bslen",
			&si->bslen_pin, GPIOF_OUT_INIT_LOW, "bslen");
	if (err)
		return err;

	msleep(100);

	/* gpio temp 1 */
	err = sop716_read_and_request_gpio(dev, "soprod,gpio_temp_1",
			&si->gpio_temp_1_pin, GPIOF_OUT_INIT_LOW,
			"gpio_temp_1");
	if (err)
		return err;

	msleep(100);

	/* reset signal high --> low */
	err = sop716_read_and_request_gpio(dev, "soprod,reset",
			&si->reset_pin, GPIOF_OUT_INIT_LOW, "reset");
	if (err)
		return err;

	msleep(100);

	return 0;
}

static void sop_firmware_update(struct sop716_info *si)
{
	extern int msp430_firmware_update_start(struct device *dev);

	gpio_direction_output(si->bslen_pin, 1);
	msleep(100);
	gpio_set_value(si->reset_pin, 1);
	gpio_set_value(si->bslen_pin, 1);
	msleep(100);
	gpio_set_value(si->bslen_pin, 0);
	msleep(100);
	gpio_set_value(si->bslen_pin, 1);
	msleep(100);
	gpio_set_value(si->reset_pin, 0);
	msleep(100);
	gpio_set_value(si->bslen_pin, 0);
	msleep(100);

	msp430_firmware_update_start(si->dev);
	gpio_set_value(si->bslen_pin, 1);
	msleep(100);
	gpio_set_value(si->bslen_pin, 0);
	msleep(100);
	gpio_set_value(si->reset_pin, 1);
	msleep(100);
	gpio_set_value(si->reset_pin, 0);
	msleep(100);
}

static void sop716_movement_work(struct work_struct *work)
{
	struct sop716_info *si = container_of(work,
			struct sop716_info, movement_work);
	struct timespec my_time;
	struct tm my_date;

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);

	pr_info("sop kernel time: %04d-%02d-%02d %02d:%02d:%02d\n",
			1900+(u8)my_date.tm_year, my_date.tm_mon+1,
			my_date.tm_mday, my_date.tm_hour, my_date.tm_min,
			my_date.tm_sec);

	if (si->sop_align_position == 0) {
		u8 align_data[SOP716_I2C_DATA_LENGTH] = {
			CMD_SOP716_MOTOR_MOVE_ALL,
			45,
			135,
			64
		};

		si->sop_align_position++;
		sop716_write(si, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
	} else if (si->sop_align_position == 1) {
		u8 align_data[SOP716_I2C_DATA_LENGTH_TIME] = {
			CMD_SOP716_SET_CURRENT_TIME,
			0xFF,
			0xFF,
			0,
			0,
			0,
			0,
			0
		};

		si->sop_align_position = 0;
		sop716_write(si, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
	} else if (si->sop_align_position == 2) {
		u8 align_data[SOP716_I2C_DATA_LENGTH] = {
			CMD_SOP716_MOTOR_MOVE_ALL,
			0,
			0,
			0
		};

		mod_timer(&si->demo_timer, jiffies + (2 * HZ));

		sop716_write(si, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
	} else if (si->sop_align_position == 3) {
		u8 align_data[SOP716_I2C_DATA_LENGTH] = {
			CMD_SOP716_MOTOR_MOVE_ALL,
			179,
			179,
			96
		};
		u8 align_data2[SOP716_I2C_DATA_LENGTH] = {
			CMD_SOP716_MOTOR_MOVE_ALL,
			0,
			0,
			96
		};

		mod_timer(&si->demo_timer, jiffies + (4 * HZ));

		sop716_write(si, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
		msleep(200);
		sop716_write(si, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data2);
	} else if (si->sop_align_position == 4) {
		u8 align_data[SOP716_I2C_DATA_LENGTH] = {
			CMD_SOP716_MOTOR_MOVE_ALL,
			179,
			179,
			224
		};
		u8 align_data2[SOP716_I2C_DATA_LENGTH] = {
			CMD_SOP716_MOTOR_MOVE_ALL,
			0,
			0,
			96
		};


		mod_timer(&si->demo_timer, jiffies + (33 * HZ));

		sop716_write(si, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data);
		msleep(200);
		sop716_write(si, SOP716_I2C_DATA_LENGTH,
				SOP716_I2C_DATA_LENGTH, align_data2);
	} else if (si->sop_align_position == 5) {
		u8 data[SOP716_I2C_DATA_LENGTH-1];

		sop716_read(si, CMD_SOP716_READ_FW_VERSION,
				SOP716_I2C_DATA_LENGTH-1, data);
		pr_info("sop firmware version: v%d.%d\n", data[1], data[2]);

		if (!(data[1] == MAJOR_VER && data[2] == MINOR_VER)) {
			struct timespec system_time;
			struct tm movement_time;
			u8 time_data[SOP716_I2C_DATA_LENGTH_TIME] = {0, };
			pr_info("sop_firmware_update : v%d.%d -> v%d.%d\n",
					data[1], data[2], MAJOR_VER, MINOR_VER);
			sop_firmware_update(si);
			system_time = __current_kernel_time();
			time_to_tm(system_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&movement_time);
			time_data[0] = CMD_SOP716_SET_CURRENT_TIME;
			time_data[1] = movement_time.tm_hour;
			time_data[2] = movement_time.tm_min;
			time_data[3] = movement_time.tm_sec;
			time_data[4] = movement_time.tm_year;
			time_data[5] = movement_time.tm_mon;
			time_data[6] = movement_time.tm_mday;
			sop716_write(si, SOP716_I2C_DATA_LENGTH,
					SOP716_I2C_DATA_LENGTH, time_data);
		} else {
			struct tm movement_time;
			struct timespec system_time;
			u8 time_data[SOP716_I2C_DATA_LENGTH_TIME] = {
				CMD_SOP716_SET_CURRENT_TIME,
				0xFF,
				0xFF,
				0,
				0,
				0,
				0,
				0
			};

			sop716_write(si, SOP716_I2C_DATA_LENGTH,
					SOP716_I2C_DATA_LENGTH, time_data);

			sop716_read(si, CMD_SOP716_READ_CURRENT_TIME,
					SOP716_I2C_DATA_LENGTH_TIME-1, time_data);
			movement_time.tm_hour = time_data[1];
			movement_time.tm_min  = time_data[2];
			movement_time.tm_sec  = time_data[3];
			movement_time.tm_year = time_data[4] + 2000;
			movement_time.tm_mon  = time_data[5];
			movement_time.tm_mday = time_data[6];
			system_time.tv_sec = mktime(movement_time.tm_year,
					movement_time.tm_mon,
					movement_time.tm_mday,
					movement_time.tm_hour,
					movement_time.tm_min,
					movement_time.tm_sec) +
				sys_tz.tz_minuteswest * 60;
			system_time.tv_nsec = 0;
			do_settimeofday(&system_time);
		}

		mod_timer(&si->demo_timer, jiffies + (13 * HZ));
	}
}

static void sop716_demo_start(unsigned long data)
{
	struct sop716_info *si = (struct sop716_info *)data;

	pr_debug("+sop_demo_start\n");

	if (si->init_sop_demo_first == 0) {
		si->init_sop_demo_first++;
		mod_timer(&si->demo_timer, jiffies + (6 * HZ));
	} else if (si->init_sop_demo_first == 1) {
		si->init_sop_demo_first++;
		si->sop_align_position = 2;
		schedule_work(&si->movement_work);
	} else if (si->init_sop_demo_first == 2) {
		si->init_sop_demo_first++;
		si->sop_align_position = 3;
		schedule_work(&si->movement_work);
	} else if (si->init_sop_demo_first == 3) {
		si->init_sop_demo_first++;
		si->sop_align_position = 4;
		schedule_work(&si->movement_work);
	} else if (si->init_sop_demo_first == 4) {
		si->init_sop_demo_first++;
		si->sop_align_position = 5;
		schedule_work(&si->movement_work);
	} else {
		if (si->align_key_state == 0) {
			si->sop_align_position = 1;
			schedule_work(&si->movement_work);
		} else {
			si->sop_align_position = 0;
			schedule_work(&si->movement_work);
			si->align_hand_state = 1;
		}
	}
	pr_debug("-sop_demo_start\n");
}

static int sop716_movement_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct sop716_info *si;
	int err;

	pr_debug("+sop716_movement_probe\n");

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
	si->hands_alignment_status = 1;

	i2c_set_clientdata(client, si);

	err = sop716_parse_dt(client);
	if (err)
		return err;

	err = sop716_config_gpios(client);
	if (err)
		return err;

	err = sysfs_create_group(&client->dev.kobj, &sop716_dev_attr_group);
	if (err) {
		pr_err("%s: cannot create sysfs\n", __func__);
		goto err_sysfs_create;
	}

	INIT_WORK(&si->movement_work, sop716_movement_work);

	/* demo start */
	init_timer(&si->demo_timer);
	si->demo_timer.function = sop716_demo_start;
	si->demo_timer.data = (unsigned long)si;
	si->demo_timer.expires = jiffies + 1;
	add_timer(&si->demo_timer);

	pr_info("SOP716 movement probed\n");
	return 0;

err_sysfs_create:
	i2c_set_clientdata(client, NULL);

	return err;
}

static int sop716_movement_remove(struct i2c_client *client)
{
	struct sop716_info *si = i2c_get_clientdata(client);

	del_timer_sync(&si->demo_timer);
	sysfs_remove_group(&client->dev.kobj, &sop716_dev_attr_group);
	i2c_set_clientdata(client, NULL);

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
static int __init sop716_movement_init(void)
{
	return i2c_add_driver(&sop716_movement_driver);
}

static void __exit sop716_movement_exit(void)
{
	i2c_del_driver(&sop716_movement_driver);
}

subsys_initcall(sop716_movement_init);
module_exit(sop716_movement_exit);

MODULE_DESCRIPTION("Soprod 716 movement");
MODULE_LICENSE("GPL v2");

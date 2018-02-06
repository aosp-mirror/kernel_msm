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
#include <linux/sysfs.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/rtc.h>

#include "sop716_firmware.h"

#define SOP716_I2C_DATA_LENGTH              8

#define SOP716_CMD_READ_RETRY               10

#define SOP716_RESET_DELAY_MS               400
#define SOP716_SETTLE_TIME_MS               2000
#define SOP716_EOL_MIN_MV                   1500
#define SOP716_EOL_MAX_MV                   4200
#define SOP716_BATTERY_CHECK_INTERVAL       30
#define SOP716_EOL_LIMIT_MV                 3700
#define SOP716_EOL_THRESHOLD_MV             3900

#define SOP716_MOTOR_MAX_POS                179
#define SOP716_MOTOR_TIME_PER_STEP_MS       17

/* FIXME: i2c error on subsequent i2c reads in 100ms */
#define SOP716_I2C_READ_POST_DELAY_MS       100

/* SOP716 COMMANDS SUB OPTIONS */
#define SOP716_OPT_ACC_DEC       0x80
#define SOP716_OPT_STACKED       0x40
#define SOP716_OPT_DIRECTION     0x20
#define SOP716_OPT_DIR_CCW       0x10
#define SOP716_OPT_NO_HANDS_MOVE 0x08

enum sop716_motor_ids {
	MOTOR1 = 0,
	MOTOR2,
};

enum sop716_cmd_ids {
	CMD_SOP716_SET_CURRENT_TIME = 0,
	CMD_SOP716_MOTOR_MOVE_ONE,
	CMD_SOP716_MOTOR_MOVE_ALL,
	CMD_SOP716_MOTOR_INIT,
	CMD_SOP716_READ_CURRENT_TIME,
	CMD_SOP716_READ_FW_VERSION, /* 5 */
	CMD_SOP716_RESERVED_6,
	CMD_SOP716_BATTERY_CHECK_INTERVAL,
	CMD_SOP716_READ_BATTERY_LEVEL,
	CMD_SOP716_READ_BATTERY_CHECK_INTERVAL,
	CMD_SOP716_SET_TIMEZONE, /* 10 */
	CMD_SOP716_READ_TIMEZONE,
	CMD_SOP716_SET_EOL_BATTERY_LEVEL,
	CMD_SOP716_READ_EOL_BATTERY_LEVEL,
	CMD_SOP716_READ_MOTORS_POSITION,
	CMD_SOP716_READ_MODE, /* 15 */
	CMD_SOP716_SET_AGING_TEST,
	CMD_SOP716_MAX
};

enum sop716_mode_ids {
	MODE_IDLE,
	MODE_START_TURN_CW,
	MODE_START_TURN_CCW,
	MODE_RETURN_POS_0,
	MODE_WATCH = 5,
	MODE_MOTOR,
	MODE_MAX = 10
};

struct sop716_command {
	char *desc;
	u8 size;
	u8 rev;
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

	int reset_delay_ms;
	int hctosys_pre_delay_ms;
	int hctosys_post_delay_ms;
	int fw_ver_check_delay_ms;
	int hands_settle_time_ms;
	int battery_check_interval;
	int eol_limit_mv;
	int eol_threshold_mv;

	u8 fw_ver;
	u8 fw_rev;

	bool update_sysclock_on_boot;
	bool watch_mode;

	struct sop716_command *cmds;

	struct work_struct fw_work;
	struct work_struct sysclock_work;
	struct work_struct init_work;
	struct mutex lock;

	u64 saved_ts_ns; /* workaround for i2c read errors */
};

struct sop716_error {
	struct sop716_info *si;
	int errno;
	u32 ts; /* in seconds */
	u8 hour;
	u8 minute;
	u16 code; /* (type << 8)|param */
};

static struct sop716_info *sop716_info;

static void sop716_hw_reset_locked(struct sop716_info *si, bool need_dump);
static int sop716_restore_default_config(struct sop716_info *si);
static void sop716_print_errors(struct sop716_info *si);

static inline int REG_CMD(struct sop716_command *cmd, u8 reg, char *desc,
		u8 size, u8 rev)
{
	if (reg >= CMD_SOP716_MAX) {
		pr_err("%s: unknown command %d\n", __func__, reg);
		return -EINVAL;
	}

	cmd[reg].desc = desc;
	cmd[reg].size = size;
	cmd[reg].rev = rev;

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
			"set time", 8, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_MOTOR_MOVE_ONE,
			"motor move one", 4, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_MOTOR_MOVE_ALL,
			"motor move all", 4, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_MOTOR_INIT,
			"motor init", 4, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_CURRENT_TIME,
			"read time", 6, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_FW_VERSION,
			"read fw version", 2, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_BATTERY_CHECK_INTERVAL,
			"battery check interval", 3, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_BATTERY_LEVEL,
			"read battery level" , 2, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_BATTERY_CHECK_INTERVAL,
			"read battery check interval", 2, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_SET_TIMEZONE,
			"set timezone", 3, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_TIMEZONE,
			"read timezone", 2, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_SET_EOL_BATTERY_LEVEL,
			"set eol battery level" , 5, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_EOL_BATTERY_LEVEL,
			"read eol battery level", 4, 0);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_MOTORS_POSITION,
			"read motors position", 2, 16);
	err |= REG_CMD(si->cmds, CMD_SOP716_READ_MODE,
			"read mode", 1, 16);
	err |= REG_CMD(si->cmds, CMD_SOP716_SET_AGING_TEST,
			"set aging test", 3, 16);

	return err;
}

static int sop716_write(struct sop716_info *si, u8 reg, u8 *val)
{
	int ret;
	int retry = 1;

	pr_debug("%s: reg:%d val:%d\n", __func__, val[0], *val);

	if (reg >= CMD_SOP716_MAX) {
		pr_err("%s: unknown command %d\n", __func__, reg);
		return -EINVAL;
	}

	if (si->cmds[reg].rev > si->fw_rev) {
		pr_warn("%s: cmd %s(%d) not supported by fw v%d.%d\n",
				__func__,
				si->cmds[reg].desc, reg,
				si->fw_ver, si->fw_rev);
		return -ENOTSUPP;
	}

	/* sop716 writes size instead of reg */
	do {
		ret = i2c_smbus_write_i2c_block_data(si->client,
				si->cmds[reg].size, si->cmds[reg].size, val);
		if (ret < 0) {
			pr_warn("%s: cannot write i2c: cmd %s(%d)\n"
				"and force reset to recover\n",
				__func__, si->cmds[reg].desc, reg);
			sop716_hw_reset_locked(si, true);
		}
	} while (retry-- && ret < 0);

	if (ret < 0)
		pr_err("%s: cannot write i2c: cmd %s(%d)\n",
				__func__, si->cmds[reg].desc, reg);

	return ret;
}

static int sop716_read(struct sop716_info *si, u8 reg, u8 *val)
{
	int ret;
	int retry = 1;
	u64 saved_ts_ns = si->saved_ts_ns;
	u64 off_ms;

	pr_debug("%s: reg:%d\n", __func__, reg);

	if (reg >= CMD_SOP716_MAX) {
		pr_err("%s: unknown command %d\n", __func__, reg);
		return -EINVAL;
	}

	if (si->cmds[reg].rev > si->fw_rev) {
		pr_warn("%s: cmd %s(%d) not supported by FW %d.%d\n",
				__func__,
				si->cmds[reg].desc, reg,
				si->fw_ver, si->fw_rev);
		return -ENOTSUPP;
	}

	/* FIXME: i2c error on subsequent i2c reads */
	off_ms = div_u64(ktime_get_boot_ns() - saved_ts_ns, NSEC_PER_MSEC);
	if (off_ms < SOP716_I2C_READ_POST_DELAY_MS)
		msleep(SOP716_I2C_READ_POST_DELAY_MS - off_ms);

	/* read size + data, So "size + 1" need to be added  */
	do {
		ret = i2c_smbus_read_i2c_block_data(si->client,
				reg, si->cmds[reg].size + 1, val);
		if ((ret < 0) || ((ret - 1) != val[0])) {
			pr_warn("%s: cannot read i2c: cmd %s(%d)\n"
				"and force reset to recover\n",
				__func__, si->cmds[reg].desc, reg);
			sop716_hw_reset_locked(si, true);
		}
	} while (retry-- && ret < 0);

	saved_ts_ns = ktime_get_boot_ns();

	if (ret < 0) {
		pr_err("%s: cannot read i2c: cmd %s(%d)\n",
				__func__, si->cmds[reg].desc, reg);
		return ret;
	}

	if ((ret - 1) != val[0]) {
		pr_err("%s: error: cmd %s(%d), size %d, ret %d, val[0] %d\n",
				__func__,
				si->cmds[reg].desc, reg,
				si->cmds[reg].size, ret, val[0]);
		return -EINVAL;

	}
	return ret;
}

static int sop716_read_error(struct sop716_error *serr)
{
	struct sop716_info *si = serr->si;
	u8 errno = serr->errno;
	u64 saved_ts_ns = si->saved_ts_ns;
	u64 off_ms;
	u8 val[8] = {0, };
	int ret;
	int retry = 1;

	pr_debug("%s: errno:%d\n", __func__, errno);

	if (errno > 0xF) {
		pr_err("%s: unknown errno %d\n", __func__, errno);
		return -EINVAL;
	}

	/* FIXME: i2c error on subsequent i2c reads */
	off_ms = div_u64(ktime_get_boot_ns() - saved_ts_ns, NSEC_PER_MSEC);
	if (off_ms < SOP716_I2C_READ_POST_DELAY_MS)
		msleep(SOP716_I2C_READ_POST_DELAY_MS - off_ms);

	do {
		ret = i2c_smbus_read_i2c_block_data(si->client,
				0x80 + errno, 8, val);
		if (ret < 0)
			sop716_hw_reset_locked(si, false);
	} while (retry-- && ret < 0);

	saved_ts_ns = ktime_get_boot_ns();

	if (ret < 0) {
		pr_err("%s: cannot read i2c: errno:%d\n", __func__, errno);
		return ret;
	}

	serr->errno = errno;
	serr->ts = (val[0] << 24) | (val[1] << 16) | (val[2] << 8) | val[3];
	serr->hour = val[4];
	serr->minute = val[5];
	serr->code = (val[6] << 8) | val[7];

	return ret;
}

static void sop716_print_error_single(struct sop716_error *serr)
{
	char *reason = "Unknown";

	switch (serr->code) {
		/* system reset */
		case 0x100:
			reason = "No interrupt pending";
			break;
		case 0x102:
			reason = "Brownout (BOR)";
			break;
		case 0x104:
			reason = "RST/NMI (BOR)";
			break;
		case 0x106:
			reason = "PMMSWBOR (BOR)";
			break;
		case 0x108:
			reason = "Wakeup from LPMx.5";
			break;
		case 0x10A:
			reason = "Security violation";
			break;
		case 0x10C:
			reason = "SVSL (POR)";
			break;
		case 0x10E:
			reason = "SVSH (POR)";
			break;
		case 0x110:
			reason = "SVML_OVP (POR)";
			break;
		case 0x112:
			reason = "SVMH_OVP (POR)";
			break;
		case 0x114:
			reason = "PMMSWPOR (POR)";
			break;
		case 0x116:
			reason = "WDT time out (PUC)";
			break;
		case 0x118:
			reason = "WDT password violation (PUC)";
			break;
		case 0x11A:
			reason = "Flash password violation (PUC)";
			break;
		case 0x11E:
			reason = "PERF peripheral/configuration area fetch (PUC)";
			break;
		case 0x120:
			reason = "PMM password vioation (PUC)";
			break;
		/* A/D converter error */
		case 0x202:
			reason = "Register Overflow";
			break;
		case 0x204:
			reason = "Conversion time Overflow";
			break;
		/* serial error */
		case 0x302:
			reason = "Too few byte received";
			break;
		case 0x304:
			reason = "Too many byte received";
			break;
		case 0x306:
			reason = "Stop bit not received";
			break;
		case 0x308:
			reason = "I2C timeout";
			break;
		/* motor error */
		case 0x402:
			reason = "Wrong motor position";

			break;
		case 0x200:
		case 0x300:
		case 0x400:
			reason = "Nothing";
			break;
		default:
			break;
	}

	printk("%2u [%10u] %2u:%-2u %s (%3x)\n",
		serr->errno, serr->ts, serr->hour, serr->minute,
		reason, serr->code);
}

static void sop716_print_errors(struct sop716_info *si)
{
	u8 errno;
	struct sop716_error serr;
	int ret;

	mutex_lock(&si->lock);
	printk("sop716 dump errors:\n"
	       "no [ts in second] h:m reason_string (code)\n"
	       "---------------------------------------\n");
	for (errno = 0; errno < 0x10; errno++) {
		memset(&serr, 0, sizeof(struct sop716_error));

		serr.si = si;
		serr.errno = errno;

		ret = sop716_read_error(&serr);
		if (ret < 0)
			break;
		sop716_print_error_single(&serr);
	}
	printk("---------------------------------------\n");
	mutex_unlock(&si->lock);
}

static void sop716_hw_reset_locked(struct sop716_info *si, bool need_dump)
{
	gpio_set_value(si->gpio_reset, 1);
	msleep(10);
	gpio_set_value(si->gpio_reset, 0);
	msleep(si->reset_delay_ms);

	if (need_dump) {
		mutex_unlock(&si->lock);
		sop716_print_errors(si);
		mutex_lock(&si->lock);
	}
}

static int sop716_read_tz_minutes(struct sop716_info *si)
{
	int ret;
	int tz_minutes;
	s8 data[SOP716_I2C_DATA_LENGTH] = {0, };

	mutex_lock(&si->lock);
	ret = sop716_read(si, CMD_SOP716_READ_TIMEZONE, data);
	mutex_unlock(&si->lock);
	if (ret < 0) {
		pr_err("%s: error: read timezone\n", __func__);
		return 0;
	}

	tz_minutes = (int)data[2]; /* hour */
	tz_minutes = (tz_minutes * 60) + data[1];

	return tz_minutes;
}

static int sop716_set_battery_check_interval(struct sop716_info *si,
		int interval)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	int rc;

	pr_debug("%s: interval:%d\n", __func__, interval);

	if ((interval < 1 || interval > 7200)) {
		pr_err("%s: out of range %d\n", __func__, interval);
		return -EINVAL;
	}

	data[0] = CMD_SOP716_BATTERY_CHECK_INTERVAL;
	data[1] = (interval >> 8) & 0xff;
	data[2] = interval & 0xff;

	mutex_lock(&si->lock);
	rc = sop716_write(si, CMD_SOP716_BATTERY_CHECK_INTERVAL, data);
	if (rc < 0)
		goto out;

	rc = sop716_read(si, CMD_SOP716_READ_BATTERY_CHECK_INTERVAL, data);
	if (rc < 0)
		goto out;

	if (interval != ((data[1] << 8) + data[2])) {
		pr_err("%s: cannot update the interval\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	si->battery_check_interval = interval;
	rc = 0;

out:
	mutex_unlock(&si->lock);
	return rc;
}

static int sop716_set_eol_limit(struct sop716_info *si, int eol_limit)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	int rc = 0;

	if (eol_limit < SOP716_EOL_MIN_MV || eol_limit > SOP716_EOL_MAX_MV ) {
		pr_err("%s: out of range!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&si->lock);
	rc = sop716_read(si, CMD_SOP716_READ_EOL_BATTERY_LEVEL, data);
	if (rc < 0) {
		pr_err("%s: failed to read EOL info\n", __func__);
		goto out;
	}

	data[0] = CMD_SOP716_SET_EOL_BATTERY_LEVEL;
	data[1] = (eol_limit >> 8) & 0xff;
	data[2] = eol_limit & 0xff;

	rc = sop716_write(si, CMD_SOP716_SET_EOL_BATTERY_LEVEL, data);
	if (rc < 0)
		goto out;

	si->eol_limit_mv = eol_limit;
	rc = 0;
out:
	mutex_unlock(&si->lock);
	return rc;
}

static int sop716_set_eol_threshold(struct sop716_info *si, int eol_threshold)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	int rc = 0;

	if (eol_threshold < SOP716_EOL_MIN_MV ||
	    eol_threshold > SOP716_EOL_MAX_MV) {
		pr_err("%s: out of range!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&si->lock);
	rc = sop716_read(si, CMD_SOP716_READ_EOL_BATTERY_LEVEL, data);
	if (rc < 0) {
		pr_err("%s: failed to read EOL info\n", __func__);
		goto out;
	}

	data[0] = CMD_SOP716_SET_EOL_BATTERY_LEVEL;
	data[3] = (eol_threshold >> 8) & 0xff;
	data[4] = eol_threshold & 0xff;

	rc = sop716_write(si, CMD_SOP716_SET_EOL_BATTERY_LEVEL, data);
	if (rc < 0)
		goto out;

	si->eol_threshold_mv = eol_threshold;
	rc = 0;
out:
	mutex_unlock(&si->lock);
	return rc;
}

static int sop716_read_mode(struct sop716_info *si)
{
	u8 data[SOP716_I2C_DATA_LENGTH] = {0, };
	int rc;
	int mode;

	mutex_lock(&si->lock);
	rc = sop716_read(si, CMD_SOP716_READ_MODE, data);
	mutex_unlock(&si->lock);

	mode = data[1];

	/* if mode is not determined, set the mode to WATCH by default */
	if (rc < 0 || mode > MODE_WATCH)
		mode = MODE_WATCH;

	return mode;
}

static void sop716_read_fw_version(struct sop716_info *si)
{
	u8 data[SOP716_I2C_DATA_LENGTH] = {0, };

	mutex_lock(&si->lock);
	sop716_read(si, CMD_SOP716_READ_FW_VERSION, data);
	mutex_unlock(&si->lock);

	si->fw_ver = data[1];
	si->fw_rev = data[2];
}

static void sop716_read_motors_position_locked(struct sop716_info *si,
		u8 motor_pos[])
{
	u8 data[SOP716_I2C_DATA_LENGTH] = {0, };

	sop716_read(si, CMD_SOP716_READ_MOTORS_POSITION, data);

	motor_pos[MOTOR1] = min(data[1], (u8)SOP716_MOTOR_MAX_POS);
	motor_pos[MOTOR2] = min(data[2], (u8)SOP716_MOTOR_MAX_POS);
	pr_debug("%s: motor1 pos %d motor2 pos %d\n", __func__,
			motor_pos[MOTOR1], motor_pos[MOTOR2]);
}

static u8 sop716_get_distance_locked(struct sop716_info *si,
		u8 motor, u8 pos, u8 opt)
{
	u8 val[2];
	int distance;

	if (pos > SOP716_MOTOR_MAX_POS) {
		pr_err("%s: invalid vlaue (%d)\n", __func__, pos);
		return -EINVAL;
	}

	if (opt & SOP716_OPT_STACKED) /* stacked request */
		return 0;

	/* get current motors position */
	sop716_read_motors_position_locked(si, val);

	/* calculate distance */
	distance = SOP716_MOTOR_MAX_POS + 1 - val[motor] + pos;
	distance = distance % (SOP716_MOTOR_MAX_POS + 1);

	distance = clamp_val(distance, 0, SOP716_MOTOR_MAX_POS);
	if (opt & SOP716_OPT_DIRECTION) { /* force direction */
		if (opt & SOP716_OPT_DIR_CCW) /* counter clockwise */
			distance = SOP716_MOTOR_MAX_POS - distance + 1;
	} else { /* shortest way */
		if (distance > ((SOP716_MOTOR_MAX_POS + 1) >> 1))
			distance = SOP716_MOTOR_MAX_POS - distance + 1;
	}

	return distance;
}

static inline void sop716_wait_for_motor_move_locked(u8 distance)
{
	pr_debug("%s: distance %d\n", __func__, distance);
	msleep(SOP716_MOTOR_TIME_PER_STEP_MS * distance);
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
		sop716_hw_reset_locked(si, false);
		mutex_unlock(&si->lock);
	}

	pr_debug("%s: reset %d\n", __func__, reset);
	return count;
}

static int convert_to_integers(char **str, const char *delimit,
			      int result[], int items)
{
	char *p;
	int rc = 0;
	int i = 0;

	while (i < items && (p = strsep(str, delimit)) != NULL) {
		rc = kstrtoint(p, 10, &result[i++]);
		if (rc) {
			pr_err("%s: Invalid value\n", __func__);
			return rc;
		}
	}

	if (i != items) {
		pr_err("%s: data is too short: %s\n", __func__, *str);
		return -EINVAL;
	}

	return 0;
}

/*
 * Code for CMD_SOP716_SET_CURRENT_TIME
 * input format: YYYY-MM-DD hh:mm:ss [option]
 */
static ssize_t sop716_time_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	char tmp[32];
	char *date = NULL;
	char *time = NULL;
	char *opt = NULL;
	char *p;
	int user_data[7] = {0, };
	int rc = 0;
	u8 pos[2];
	u8 dist[2];

	if (!count) {
		pr_err("%s: invalid input count!\n", __func__);
		return -EINVAL;
	}

	strlcpy(tmp, buf, sizeof(tmp));
	p = tmp;

	date = strsep(&p, " ");
	if (!date) {
		pr_err("%s: No date info!\n", __func__);
		return -EINVAL;
	}
	time = strsep(&p, " ");
	if (!time) {
		pr_err("%s: No time info\n", __func__);
		return -EINVAL;
	}
	opt = strsep(&p, " ");

	/* parse date */
	rc = convert_to_integers(&date, "-", user_data, 3);
	if (rc < 0) {
		pr_err("%s: Invalid date info: %s\n", __func__, buf);
		return -EINVAL;
	}

	/* parse time */
	rc = convert_to_integers(&time, ":", &user_data[3], 3);
	if (rc < 0) {
		pr_err("%s: Invalid time info: %s\n", __func__, buf);
		return -EINVAL;
	}

	if (opt) {
		rc = kstrtoint(opt, 0, &user_data[6]);
		if (rc < 0) {
			pr_err("%s: Invalid opt: %s\n", __func__, buf);
			return -EINVAL;
		}
	}

	user_data[0] -= 2000;
	if (user_data[0] < 0 || user_data[0] > 99 || // year
	    user_data[1] < 1 || user_data[1] > 12 || // month
	    user_data[2] < 1 || user_data[2] > 31 || // day
	    user_data[3] < 0 || user_data[3] > 23 || // hour
	    user_data[4] < 0 || user_data[4] > 59 || // minute
	    user_data[5] < 0 || user_data[5] > 59 || // second
	    user_data[6] < 0 || user_data[6] > 255) {
		pr_err("%s: out of range: %s\n", __func__, buf);
		return -EINVAL;
	}

	pr_info("sop set time: %04d-%02d-%02d %02d:%02d:%02d opt:%02x\n",
			user_data[0] + 2000,
			user_data[1],
			user_data[2],
			user_data[3],
			user_data[4],
			user_data[5],
			user_data[6]);

	data[0] = CMD_SOP716_SET_CURRENT_TIME;
	data[1] = user_data[3]; //hour
	data[2] = user_data[4]; //minute
	data[3] = user_data[5]; //second
	data[4] = user_data[0]; //year
	data[5] = user_data[1]; //month
	data[6] = user_data[2]; //day
	data[7] = user_data[6]; //opt

	mutex_lock(&si->lock);
	/* covert h:m to motors position */
	pos[MOTOR1] = 3 * data[2];
	pos[MOTOR2] = (15 * (data[1] % 12)) + (data[2] >> 2);
	dist[MOTOR1] = sop716_get_distance_locked(si, MOTOR1, pos[MOTOR1],
			user_data[6]);
	dist[MOTOR2] = sop716_get_distance_locked(si, MOTOR2, pos[MOTOR2],
			user_data[6]);

	rc = sop716_write(si, CMD_SOP716_SET_CURRENT_TIME, data);
	if (rc < 0)
		pr_err("%s: cannot set time\n", __func__);

	/* wait for motor move */
	dist[0] = max(dist[MOTOR1], dist[MOTOR2]);
	sop716_wait_for_motor_move_locked(dist[0]);
	/*
	 * Do not set the watch mode if opt is not moving hands
	 */
	if (!(user_data[6] & SOP716_OPT_NO_HANDS_MOVE))
		si->watch_mode = true;
	mutex_unlock(&si->lock);

	return rc < 0? rc : count;
}

/*
 * CMD_SOP716_MOTOR_MOVE_ONE
 * input format: type:position[:option]
 */
static ssize_t sop716_motor_move_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	char tmp[16] = {0, };
	char *str, *p;
	int result[2];
	int option = 0;
	int rc = 0;
	u8 dist;

	if (!count) {
		pr_err("%s: invalid input count!\n", __func__);
		return -EINVAL;
	}

	strlcpy(tmp, buf, sizeof(tmp));
	str = tmp;

	rc = convert_to_integers(&str, ":", result, 2);
	if (rc < 0) {
		pr_err("%s: Invalid arguments\n", __func__);
		return rc;
	}

	p = strsep(&str, ":");
	if (p) {
		rc = kstrtoint(p, 0, &option);
		if (rc) {
			pr_err("%s: Invalid value: %s\n", __func__, buf);
			return -EINVAL;
		}
	}

	if (result[0] < 0 || result[0]> 1  ||   // motor type
	    result[1] < 0 || result[1] > SOP716_MOTOR_MAX_POS || // motor position
	    option < 0 || option > 255) {
		pr_err("%s: out of range: %s\n", __func__, buf);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d motortype:%d destination:%d option:%d\n",
			__func__, count, result[0], result[1], option);

	data[0] = CMD_SOP716_MOTOR_MOVE_ONE;
	data[1] = result[0];
	data[2] = result[1];
	data[3] = option;

	mutex_lock(&si->lock);
	dist = sop716_get_distance_locked(si, data[1], data[2], data[3]);

	sop716_write(si, CMD_SOP716_MOTOR_MOVE_ONE, data);

	/* wait for motor move */
	sop716_wait_for_motor_move_locked(dist);

	si->watch_mode = false;
	mutex_unlock(&si->lock);

	return count;
}

/*
 * CMD_SOP716_MOTOR_MOVE_ALL
 * input format: motor1:motor2[:option]
 */
static ssize_t sop716_motor_move_all_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	char tmp[16] = {0, };
	char *str, *p;
	int result[2];
	int option = 0;
	int rc = 0;
	u8 dist[2];

	if (!count) {
		pr_err("%s: invalid input count!\n", __func__);
		return -EINVAL;
	}

	strlcpy(tmp, buf, sizeof(tmp));
	str = tmp;

	rc = convert_to_integers(&str, ":", result, 2);
	if (rc < 0) {
		pr_err("%s: invalid arguments\n", __func__);
		return rc;
	}

	p = strsep(&str, ":");
	if (p) {
		rc = kstrtoint(p, 0, &option);
		if (rc) {
			pr_err("%s: Invalid value: %s\n", __func__, buf);
			return -EINVAL;
		}
	}

	if (result[0] < 0 || result[0] > SOP716_MOTOR_MAX_POS || // motor1
	    result[1] < 0 || result[1] > SOP716_MOTOR_MAX_POS || // motor2
	    option < 0 || option > 255) {
		pr_err("%s: invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d motor1:%d motor2:%d op:%d\n",
			__func__, count, result[0], result[1], option);

	data[0] = CMD_SOP716_MOTOR_MOVE_ALL;
	data[1] = result[0];
	data[2] = result[1];
	data[3] = option;

	mutex_lock(&si->lock);
	dist[MOTOR1] = sop716_get_distance_locked(si, MOTOR1, data[1], data[3]);
	dist[MOTOR2] = sop716_get_distance_locked(si, MOTOR2, data[2], data[3]);

	sop716_write(si, CMD_SOP716_MOTOR_MOVE_ALL, data);

	/* wait for motor move */
	dist[0] = max(dist[MOTOR1], dist[MOTOR2]);
	sop716_wait_for_motor_move_locked(dist[0]);

	si->watch_mode = false;
	mutex_unlock(&si->lock);

	return count;
}

/*
 * CMD_SOP716_MOTOR_INIT
 * input format: type:dir:steps
 */
static ssize_t sop716_motor_init_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	char tmp[16] = {0, };
	char *str;
	int result[3];
	int rc = 0;
	u8 dist, opt;

	if (!count) {
		pr_err("%s: invalid input count!\n", __func__);
		return -EINVAL;
	}

	strlcpy(tmp, buf, sizeof(tmp));
	str = tmp;

	rc = convert_to_integers(&str, ":", result, 3);
	if (rc < 0) {
		pr_err("%s: invalid arguments\n", __func__);
		return rc;
	}

	if (result[0] < 0 || result[0] > 1 || // motor type
	    result[1] < 0 || result[1] > 1 || // motor direction
	    result[2] < 0 || result[2] > 179) { // motor steps
		pr_err("%s: invalid input format!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: cnt:%d motor_type:%d direction:%d num_of_steps:%d\n",
			__func__, count, result[0], result[1], result[2]);

	data[0] = CMD_SOP716_MOTOR_INIT;
	data[1] = result[0];
	data[2] = result[1];
	data[3] = result[2];

	mutex_lock(&si->lock);
	opt = SOP716_OPT_DIRECTION;
	if (data[2]) /* counter clockwise */
		opt |= SOP716_OPT_DIR_CCW;

	sop716_write(si, CMD_SOP716_MOTOR_INIT, data);

	/* wait for motor move */
	dist = data[3];
	sop716_wait_for_motor_move_locked(dist);

	si->watch_mode = false;
	mutex_unlock(&si->lock);

	return count;
}

/* Code for CMD_SOP716_READ_CURRENT_TIME */
static ssize_t sop716_time_show(struct device *dev,
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
static ssize_t sop716_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	struct sop716_info *si = dev_get_drvdata(dev);

	mutex_lock(&si->lock);
	sop716_read(si, CMD_SOP716_READ_FW_VERSION, data);
	mutex_unlock(&si->lock);

	return snprintf(buf, PAGE_SIZE, "%d.%d\n", data[1], data[2]);
}

/* Code for CMD_SOP716_BATTERY_CHECK_INTERVAL */
static ssize_t sop716_battery_check_interval_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	int interval;
	int rc = 0;

	mutex_lock(&si->lock);
	rc = sop716_read(si, CMD_SOP716_READ_BATTERY_CHECK_INTERVAL, data);
	mutex_unlock(&si->lock);
	if (rc < 0)
		return rc;

	interval = ((data[1] << 8) + data[2]);

	return snprintf(buf, PAGE_SIZE, "%d second(s)\n", interval);
}

static ssize_t sop716_battery_check_interval_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
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

	rc = sop716_set_battery_check_interval(si, interval);
	if (rc < 0) {
		pr_err("%s: failed to set battery check interval\n", __func__);
		return rc;
	}

	return count;
}

/* Code for CMD_SOP716_READ_BATTERY_LEVEL */
static ssize_t sop716_battery_level_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	struct sop716_info *si = dev_get_drvdata(dev);

	mutex_lock(&si->lock);
	sop716_read(si, CMD_SOP716_READ_BATTERY_LEVEL, data);
	mutex_unlock(&si->lock);

	return snprintf(buf, PAGE_SIZE, "%d mV\n", (data[1] << 8) + data[2]);
}

static ssize_t sop716_tz_minutes_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	s8 data[SOP716_I2C_DATA_LENGTH];
	int tz_minutes;
	int rc = 0;

	if (!count) {
		pr_err("%s: invalid input count\n", __func__);
		return -EINVAL;
	 }

	rc = kstrtoint(buf, 10, &tz_minutes);
	if (rc) {
		pr_err("%s: invalid input! rc:%d\n",
				__func__, rc);
		return -EINVAL;
	 }

	if (tz_minutes < -720 || tz_minutes > 840) {
		pr_err("%s: out of range!\n", __func__);
		return -EINVAL;
	}

	data[0] = CMD_SOP716_SET_TIMEZONE;
	data[2] = tz_minutes / 60; /* hour */
	data[1] = tz_minutes - (((int)data[2]) * 60);

	mutex_lock(&si->lock);
	sop716_write(si, CMD_SOP716_SET_TIMEZONE, data);
	mutex_unlock(&si->lock);

	return count;
}

static ssize_t sop716_tz_minutes_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	int tz_minutes;

	tz_minutes = sop716_read_tz_minutes(si);

	return snprintf(buf, PAGE_SIZE, "%d\n", tz_minutes);
}

static ssize_t sop716_eol_limit_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	int eol_limit;
	int rc = 0;

	if (!count) {
		pr_err("%s: invalid input count\n", __func__);
		return -EINVAL;
	 }

	rc = kstrtoint(buf, 10, &eol_limit);
	if (rc) {
		pr_err("%s: invalid input! rc:%d\n", __func__, rc);
		return -EINVAL;
	 }

	rc = sop716_set_eol_limit(si, eol_limit);
	if (rc) {
		pr_err("%s: failed to set eol limit\n", __func__);
		return rc;
	}

	return count;
}

static ssize_t sop716_eol_limit_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	struct sop716_info *si = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&si->lock);
	rc = sop716_read(si, CMD_SOP716_READ_EOL_BATTERY_LEVEL, data);
	mutex_unlock(&si->lock);
	if (rc < 0)
		return rc;

	return snprintf(buf, PAGE_SIZE, "%d mV\n", (data[1] << 8) + data[2]);
}

static ssize_t sop716_eol_threshold_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	int eol_threshold;
	int rc = 0;

	if (!count) {
		pr_err("%s: invalid input count\n", __func__);
		return -EINVAL;
	 }

	rc = kstrtoint(buf, 10, &eol_threshold);
	if (rc) {
		pr_err("%s: invalid input! rc:%d\n", __func__, rc);
		return -EINVAL;
	 }

	rc = sop716_set_eol_threshold(si, eol_threshold);
	if (rc) {
		pr_err("%s: failed to set eol threshold\n", __func__);
		return rc;
	}

	return count;
}

static ssize_t sop716_eol_threshold_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 data[SOP716_I2C_DATA_LENGTH];
	struct sop716_info *si = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&si->lock);
	rc = sop716_read(si, CMD_SOP716_READ_EOL_BATTERY_LEVEL, data);
	mutex_unlock(&si->lock);
	if (rc < 0)
		return rc;

	return snprintf(buf, PAGE_SIZE, "%d mV\n", (data[3] << 8) + data[4]);
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
	msleep(si->hctosys_pre_delay_ms);
	err = sop716_read(si, CMD_SOP716_READ_CURRENT_TIME, time_data);
	msleep(si->hctosys_post_delay_ms);
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
	struct rtc_time tm;
	struct timeval tv;
	u8 pos[2];
	u8 dist[2];
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
	/* Get local time */
	do_gettimeofday(&tv);
	tv.tv_sec -= si->tz_minuteswest * 60;
	rtc_time_to_tm(tv.tv_sec, &tm);

	pr_debug("%s: current time: %d-%02d-%02d %02d:%02d:%02d\n",
			__func__,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec);

	/* covert h:m to motors position */
	pos[MOTOR1] = 3 * tm.tm_min;
	pos[MOTOR2] = (15 * (tm.tm_hour % 12)) + (tm.tm_min >> 2);
	dist[MOTOR1] = sop716_get_distance_locked(si, MOTOR1, pos[MOTOR1], 0);
	dist[MOTOR2] = sop716_get_distance_locked(si, MOTOR2, pos[MOTOR2], 0);

	err = sop716_write(si, CMD_SOP716_SET_CURRENT_TIME, data);

	/* wait for motor move */
	dist[0] = max(dist[MOTOR1], dist[MOTOR2]);
	sop716_wait_for_motor_move_locked(dist[0]);
	mutex_unlock(&si->lock);
	if (err < 0) {
		pr_err("%s: cannot set watch mode\n", __func__);
		return err;
	}

	si->watch_mode = true;

	return count;
}

/*
 * CMD_SOP716_SET_AGING_TEST
 * intput format: 0~255 (0: stop, 1~255: start the aging test
 */
static ssize_t sop716_aging_test_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	u8 data[SOP716_I2C_DATA_LENGTH];
	unsigned int v = -1;
	int rc;

	if (!count) {
		pr_err("%s: invalid input count!\n", __func__);
		return -EINVAL;
	}

	rc = kstrtouint(buf, 0, &v);
	if (rc < 0 || v > 255) {
		pr_err("%s: invalid input %u\n", __func__, v);
		return -EINVAL;
	}

	pr_debug("%s: No. of sequence:%u\n", __func__, v);

	data[0] = CMD_SOP716_SET_AGING_TEST;
	data[1] = v? 1 : 0;
	data[2] = v;

	mutex_lock(&si->lock);
	sop716_write(si, CMD_SOP716_SET_AGING_TEST, data);
	si->watch_mode = false;
	mutex_unlock(&si->lock);

	return count;
}

static ssize_t sop716_dump_errors(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sop716_info *si = dev_get_drvdata(dev);
	sop716_print_errors(si);
	return count;
}

static DEVICE_ATTR(reset, S_IWUSR, NULL, sop716_reset_store);
static DEVICE_ATTR(time, S_IWUSR | S_IRUGO,
		sop716_time_show, sop716_time_store);
static DEVICE_ATTR(motor_init, S_IWUSR, NULL, sop716_motor_init_store);
static DEVICE_ATTR(motor_move, S_IWUSR, NULL, sop716_motor_move_store);
static DEVICE_ATTR(motor_move_all, S_IWUSR, NULL, sop716_motor_move_all_store);
static DEVICE_ATTR(version, S_IRUGO, sop716_version_show, NULL);
static DEVICE_ATTR(battery_check_interval, S_IRUGO | S_IWUSR,
		sop716_battery_check_interval_show,
		sop716_battery_check_interval_store);
static DEVICE_ATTR(battery_level, S_IRUGO,
		sop716_battery_level_show, NULL);
static DEVICE_ATTR(tz_minutes, S_IWUSR | S_IRUGO,
		sop716_tz_minutes_show, sop716_tz_minutes_store);
static DEVICE_ATTR(eol_limit, S_IWUSR | S_IRUGO,
		sop716_eol_limit_show, sop716_eol_limit_store);
static DEVICE_ATTR(eol_threshold, S_IWUSR | S_IRUGO,
		sop716_eol_threshold_show, sop716_eol_threshold_store);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, sop716_update_fw_store);
static DEVICE_ATTR(update_sysclock, S_IWUSR, NULL,
		sop716_update_sysclock_store);
static DEVICE_ATTR(watch_mode, S_IWUSR | S_IRUGO, sop716_watch_mode_show,
		sop716_watch_mode_store);
static DEVICE_ATTR(aging_test, S_IWUSR, NULL, sop716_aging_test_store);
static DEVICE_ATTR(dump_errors, S_IWUSR, NULL, sop716_dump_errors);

static struct attribute *sop716_dev_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_time.attr,
	&dev_attr_motor_init.attr,
	&dev_attr_motor_move.attr,
	&dev_attr_motor_move_all.attr,
	&dev_attr_version.attr,
	&dev_attr_battery_check_interval.attr,
	&dev_attr_battery_level.attr,
	&dev_attr_tz_minutes.attr,
	&dev_attr_eol_limit.attr,
	&dev_attr_eol_threshold.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_sysclock.attr,
	&dev_attr_watch_mode.attr,
	&dev_attr_aging_test.attr,
	&dev_attr_dump_errors.attr,
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
	u32 val;
	int ret;

	si->update_sysclock_on_boot = of_property_read_bool(node,
			"lge,update-syslock-on-boot");

	ret = of_property_read_u32(node, "lge,reset-delay-ms", &val);
	si->reset_delay_ms = ret? SOP716_RESET_DELAY_MS : val;

	ret = of_property_read_u32(node, "lge,hctosys-pre-delay-ms", &val);
	si->hctosys_pre_delay_ms = ret? 0 : val;

	ret = of_property_read_u32(node, "lge,hctosys-post-delay-ms", &val);
	si->hctosys_post_delay_ms = ret? 0 : val;

	ret = of_property_read_u32(node, "lge,fw-ver-check-delay-ms", &val);
	si->fw_ver_check_delay_ms = ret? 0 : val;

	ret = of_property_read_u32(node, "lge,hands-settle-time-ms", &val);
	si->hands_settle_time_ms = ret? SOP716_SETTLE_TIME_MS : val;

	pr_info("sop716: hctosys pre-delay: %dms, hctosys post-delay: %dms\n",
			si->hctosys_pre_delay_ms,
			si->hctosys_post_delay_ms);
	pr_info("sop716: reset-delay: %dms, fw ver check delay: %dms, "
		"hands settle time: %dms\n",
		 si->reset_delay_ms,
		 si->fw_ver_check_delay_ms,
		 si->hands_settle_time_ms);

	ret = of_property_read_u32(node, "lge,battery-check-interval", &val);
	si->battery_check_interval = ret? SOP716_BATTERY_CHECK_INTERVAL : val;
	pr_info("sop716: battery check interval %d second(s)\n",
			si->battery_check_interval);

	ret = of_property_read_u32(node, "lge,eol-limit-mv", &val);
	si->eol_limit_mv = ret? SOP716_EOL_LIMIT_MV : val;

	ret = of_property_read_u32(node, "lge,eol-threshold-mv", &val);
	si->eol_threshold_mv = ret? SOP716_EOL_THRESHOLD_MV : val;
	pr_info("sop716: EOL limit %dmv threshold %dmv\n",
			si->eol_limit_mv, si->eol_threshold_mv);
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
	u8 img_ver, img_rev;
	int err;
	int retry = SOP716_CMD_READ_RETRY;
	const struct firmware *fw_entry = NULL;

	err = request_firmware(&fw_entry, "sop716.fw", si->dev);
	if (err) {
		pr_err("%s: cannot read firmware\n", __func__);
		return;
	};

	sop716fw_validate_firmware(fw_entry->data, &img_ver, &img_rev);
	if (!img_ver && !img_rev) {
		pr_err("%s: wrong firmware file\n", __func__);
	}

	pr_info("sop firmware version: device v%d.%d, image v%d.%d\n",
			si->fw_ver, si->fw_rev, img_ver, img_rev);
	if (si->fw_ver == img_ver && si->fw_rev == img_rev) {
		release_firmware(fw_entry);
		return;
	}

	mutex_lock(&si->lock);

	/* Set the position 0 */
	memset(data, 0, sizeof(data));
	data[0] = CMD_SOP716_MOTOR_MOVE_ALL;
	sop716_write(si, CMD_SOP716_MOTOR_MOVE_ALL, data);

	/* wait for hands move */
	msleep(si->hands_settle_time_ms);

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

	sop716_firmware_update(si, fw_entry->data);

	/* boot up time */
	msleep(SOP716_RESET_DELAY_MS);

	/* restore time */
	data[0] = CMD_SOP716_SET_CURRENT_TIME;
	if (!si->watch_mode)
		data[7] = SOP716_OPT_NO_HANDS_MOVE;
	do {
		err = sop716_write(si, CMD_SOP716_SET_CURRENT_TIME, data);
		msleep(100);
	} while(retry-- && err);

	if (err < 0)
		pr_err("%s: cannot set time\n", __func__);

	mutex_unlock(&si->lock);

	/* save old fw version info */
	img_ver = si->fw_ver;
	img_rev = si->fw_rev;

	/* read new fw version */
	sop716_read_fw_version(si);
	msleep(si->fw_ver_check_delay_ms);

	pr_info("sop firmware update: v%d.%d -> v%d.%d\n",
			img_ver, img_rev, si->fw_ver, si->fw_rev);

	release_firmware(fw_entry);

	sop716_restore_default_config(si);
}

static void sop716_update_sysclock_work(struct work_struct *work)
{
	struct sop716_info *si = container_of(work,
			struct sop716_info, sysclock_work);

	if (sop716_hctosys(si))
		sop716_hctosys(si); /* again one more */
}

static int sop716_restore_default_config(struct sop716_info *si)
{
	int rc;

	rc = sop716_set_battery_check_interval(si, si->battery_check_interval);
	if (rc) {
		pr_err("%s: failed to restore the battery check interval\n",
				__func__);
		return rc;
	}

	rc = sop716_set_eol_limit(si, si->eol_limit_mv);
	if (rc) {
		pr_err("%s: failed to restore eol limit\n", __func__);
		return rc;
	}

	rc = sop716_set_eol_threshold(si, si->eol_threshold_mv);
	if (rc) {
		pr_err("%s: failed to restore eol threshold\n", __func__);
		return rc;
	}

	return 0;
}

static void sop716_init_work(struct work_struct *work)
{
	struct sop716_info *si = container_of(work,
			struct sop716_info, init_work);

	sop716_read_fw_version(si);
	if (sop716_read_mode(si) == MODE_WATCH)
		si->watch_mode = true;

	sop716_restore_default_config(si);
}

static int sop716_movement_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct sop716_info *si;
	int err;
	struct kobject *kobj = NULL;

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

	if (client->dev.kobj.kset)
		kobj = &client->dev.kobj.kset->kobj;

	err = sysfs_create_link(kobj, &client->dev.kobj, "sop716");
	if (err < 0) {
		pr_err("%s: cannot create link\n", __func__);
		goto err_sysfs_create_link;
	}

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
	sysfs_remove_link(&client->dev.kobj, "sop716");

err_sysfs_create_link:
	mutex_destroy(&si->lock);
	i2c_set_clientdata(client, NULL);

	return err;
}

static int sop716_movement_remove(struct i2c_client *client)
{
	struct sop716_info *si = i2c_get_clientdata(client);

	mutex_destroy(&si->lock);
	sysfs_remove_group(&client->dev.kobj, &sop716_dev_attr_group);
	sysfs_remove_link(&client->dev.kobj, "sop716");
	i2c_set_clientdata(client, NULL);
	sop716_info = NULL;

	return 0;
}

static void sop716_movement_shutdown(struct i2c_client *client)
{
	return;
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

	/* Update tz_minuteswest before sop716_hctosys() */
	si->tz_minuteswest = sop716_read_tz_minutes(si) * -1;
	sys_tz.tz_minuteswest = si->tz_minuteswest;

	err = sop716_hctosys(si);

	return err;
}

late_initcall_sync(sop716_hctosys_init);

MODULE_DESCRIPTION("Soprod 716 movement");
MODULE_AUTHOR("LGE");
MODULE_LICENSE("GPL v2");

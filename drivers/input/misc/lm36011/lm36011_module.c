/*
 *  lm36011_module.c - Linux kernel module for led laser
 *
 *  Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include "cam_sensor_dev.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <media/cam_sensor.h>

#define LM36011_DEV_NAME "lm36011"

#define ENABLE_REG 0x01
#define CONFIGURATION_REG 0x02
#define LED_FLASH_BRIGHTNESS_REG 0x03
#define LED_TORCH_BRIGHTNESS_REG 0x04
#define FLAG_REG 0x05
#define DEVICE_ID_REG 0x06

#define IR_STANDBY_MODE 0x04
#define IR_ENABLE_MODE 0x05
#define DEVICE_ID 0x01

#define PHASENUM 4

#define MAX_RETRY_COUNT 3

#define PHASE_SELECT_REG 0x60
#define PROXAVG_REG 0x63
#define PROXOFFSET_REG 0x67
/* TODO: replace this by actual crack threshold
 * Bug: b/124409695
 */
#define CRACK_THRES ~(1<<16)

#define VIO_VOLTAGE_MIN 1800000
#define VIO_VOLTAGE_MAX 1800000
#define SLIEGO_VDD_VOlTAGE_MIN 2800000
#define SLIEGO_VDD_VOlTAGE_MAX 2800000
#define SX9320_VDD_VOlTAGE_MIN 1800000
#define SX9320_VDD_VOlTAGE_MAX 1800000
#define PMIC_BUCK1_VOlTAGE_MIN 1350000
#define PMIC_BUCK1_VOlTAGE_MAX 1350000
#define PMIC_BUCK2_VOlTAGE_MIN 4100000
#define PMIC_BUCK2_VOlTAGE_MAX 4100000

#define BUILD_DEV 0
#define BUILD_PROTO 1
#define BUILD_EVT1_0 2
#define BUILD_EVT1_1 3

enum SILEGO_GPIO {
	IR_VCSEL_FAULT,
	IR_VCSEL_TEST,
	SILEGO_SW_HALT,
	SILEGO_GPIO_MAX
};

enum CAP_SENSE_GPIO {
	CSENSE_PROXAVG_READ,
	CAP_SENSE_GPIO_MAX
};

enum LASER_TYPE {
	LASER_FLOOD,
	LASER_DOT,
	LASER_TYPE_MAX
};

struct reg_setting {
	uint32_t addr;
	uint32_t data;
};

struct led_laser_ctrl_t {
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct mutex cam_sensor_mutex;
	struct camera_io_master io_master_info;
	bool is_power_up;
	bool is_cci_init;
	bool is_probed;
	bool is_certified;
	struct regulator *vio;
	struct regulator *buck1;
	struct regulator *buck2;
	enum LASER_TYPE type;
	uint32_t read_addr;
	uint32_t read_data;
	dev_t dev;
	struct cdev c_dev;
	struct class *cl;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_active_state;
	struct pinctrl_state *gpio_suspend_state;
	int gpio_count;
	struct work_struct work;
	struct workqueue_struct *work_queue;
	uint32_t hw_version;
	struct {
		bool is_validated;
		bool is_power_up;
		bool is_vcsel_fault;
		bool is_csense_halt;
		uint32_t vcsel_fault_count;
		struct regulator *vdd;
		struct gpio *gpio_array;
		unsigned int irq[SILEGO_GPIO_MAX];
	} silego;
	struct {
		bool is_power_up;
		bool is_validated;
		struct regulator *vdd;
		uint32_t sid;
		struct gpio *gpio_array;
		unsigned int irq[CAP_SENSE_GPIO_MAX];
		int16_t proxavg[PHASENUM];
		uint16_t proxoffset[PHASENUM];
		struct camera_io_master io_master_info;
		bool is_cci_init;
		uint16_t calibraion_data[PHASENUM];
		bool is_crack_detected[LASER_TYPE_MAX];
	} cap_sense;
};

static bool read_proxoffset;
module_param(read_proxoffset, bool, 0644);

static const struct reg_setting silego_reg_settings_ver1[] = {
	{0xc0, 0x09}, {0xc1, 0xf9}, {0xc2, 0x09}, {0xc3, 0xf9}, {0xcb, 0x93},
	{0xcc, 0x81}, {0xcd, 0x93}, {0xce, 0x83}, {0x92, 0x00}, {0x93, 0x00}
};

static const struct reg_setting silego_reg_settings_ver2[] = {
	{0xc0, 0x09}, {0xc1, 0xf9}, {0xc2, 0x09}, {0xc3, 0xf9}, {0xcb, 0x00},
	{0xcc, 0x9a}, {0xcd, 0x13}, {0xce, 0x9b}, {0x92, 0x00}, {0x93, 0x00},
	{0xa0, 0x0e}, {0xa2, 0x0e}, {0x80, 0xb0}, {0x81, 0x24}, {0x82, 0x3c},
	{0x83, 0x2c}, {0x84, 0x58}, {0x85, 0x80}, {0x86, 0x40}, {0x87, 0x40},
	{0x88, 0x3e}, {0x89, 0x60}, {0x8a, 0x40}, {0x8b, 0x30}, {0x8c, 0x7c},
	{0x8d, 0x24}, {0x8e, 0xa0}, {0x8f, 0xc0}, {0x90, 0x40}, {0x91, 0xa0},
};

static const struct reg_setting silego_reg_settings_ver3[] = {
	{0xc0, 0x01}, {0xc1, 0xbb}, {0xc2, 0x01}, {0xc3, 0xbb}, {0xcb, 0x93},
	{0xcc, 0x9a}, {0xcd, 0x00}, {0xce, 0x9b}, {0x92, 0x00}, {0x93, 0x00},
	{0xa0, 0x12}, {0xa2, 0x12}, {0x80, 0xb0}, {0x81, 0x24}, {0x82, 0x58},
	{0x83, 0x2c}, {0x84, 0x68}, {0x85, 0x80}, {0x86, 0x40}, {0x87, 0x40},
	{0x88, 0x3e}, {0x89, 0x60}, {0x8a, 0x40}, {0x8b, 0x30}, {0x8c, 0x7c},
	{0x8d, 0x24}, {0x8e, 0xa0}, {0x8f, 0xc0}, {0x90, 0x40}, {0x91, 0xa0},
};

static const struct reg_setting cap_sense_init_reg_settings[] = {
	{0x10, 0x0F},
	{0x11, 0x2E},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x20, 0x20},
	{0x23, 0x01},
	{0x24, 0x47},
	{0x26, 0x01},
	{0x27, 0x47},
	{0x28, 0x3D},
	{0x29, 0x37},
	{0x2A, 0x1F},
	{0x2B, 0x40},
	{0x2C, 0x12},
	{0x2D, 0x0F},
	{0x30, 0x09},
	{0x31, 0x09},
	{0x32, 0x3F},
	{0x33, 0xC0},
	{0x34, 0x00},
	{0x35, 0x00},
	{0x36, 0x69},
	{0x37, 0x69},
	{0x40, 0x00},
	{0x41, 0x00},
	{0x42, 0x17},
	{0x43, 0x00},
	{0x44, 0x00},
	{0x45, 0x05},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x48, 0x00},
	{0x49, 0x00},
	{0x4A, 0x40},
	{0x4B, 0x31},
	{0x4C, 0x00},
	{0x4D, 0x00},
	{0x4E, 0x80},
	{0x4F, 0x0C},
	{0x50, 0x34},
	{0x51, 0x77},
	{0x52, 0x22},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x02, 0x02},
	{0x05, 0x08},
	{0x06, 0x04},
	{0x07, 0x80},
	{0x08, 0x01},
	{0x00, 0x08},
};

static int sx9320_write_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t data)
{
	int rc;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	reg_settings.reg_addr = addr;
	reg_settings.reg_data = data;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;
	rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
		&write_setting);
	if (rc < 0)
		dev_err(ctrl->soc_info.dev,
			"write 0x%x to cap sense 0x%x failed", data, addr);

	return rc;
}

static int sx9320_cleanup_nirq(struct led_laser_ctrl_t *ctrl)
{
	int rc;
	uint32_t data;

	rc = camera_io_dev_read(
		&ctrl->cap_sense.io_master_info,
		0x00,
		&data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
		dev_err(ctrl->soc_info.dev, "clean up NIRQ failed");

	return rc;
}

static void sx9320_crack_detection(struct led_laser_ctrl_t *ctrl)
{
	int gpio_level, flood_idx, dot_idx;
	bool is_sw_halt_required;

	if (ctrl->cap_sense.calibraion_data[1] == 0 ||
		ctrl->cap_sense.calibraion_data[2] == 0)
		return;

	flood_idx = 1;
	dot_idx = 2;
	gpio_level =
		gpio_get_value(ctrl->silego.gpio_array[SILEGO_SW_HALT].gpio);

	ctrl->cap_sense.is_crack_detected[LASER_FLOOD] =
		ctrl->cap_sense.proxavg[flood_idx] < CRACK_THRES ?
		true : false;

	ctrl->cap_sense.is_crack_detected[LASER_DOT] =
		ctrl->cap_sense.proxavg[dot_idx] < CRACK_THRES ?
		true : false;

	is_sw_halt_required =
		(ctrl->cap_sense.is_crack_detected[LASER_FLOOD] |
		ctrl->cap_sense.is_crack_detected[LASER_DOT]);

	if (!gpio_level && is_sw_halt_required) {
		dev_err(ctrl->soc_info.dev,
			"detected laser lens crack %d:%d(Flood:Dot), "
			"kill laser now",
			ctrl->cap_sense.is_crack_detected[LASER_FLOOD],
			ctrl->cap_sense.is_crack_detected[LASER_DOT]);
		gpio_set_value(
			ctrl->silego.gpio_array[SILEGO_SW_HALT].gpio,
			1);
	}

	if (gpio_level && !is_sw_halt_required) {
		dev_info(ctrl->soc_info.dev,
			"flood and dot crack are solved, release sw halt now");
		gpio_set_value(
			ctrl->silego.gpio_array[SILEGO_SW_HALT].gpio,
			0);
	}
}

static int sx9320_manual_compensation(struct led_laser_ctrl_t *ctrl)
{
	int rc = 0, retry_cnt;
	uint32_t data;
	uint32_t i, PH_start, PH_end;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	if (ctrl->cap_sense.calibraion_data[1] == 0 ||
		ctrl->cap_sense.calibraion_data[2] == 0) {
		dev_info(ctrl->soc_info.dev,
			"calibration data is not present, PH1: %d PH2; %d",
			ctrl->cap_sense.calibraion_data[1],
			ctrl->cap_sense.calibraion_data[2]);
		goto out;
	}
	PH_start = 1;
	PH_end = 2;

	reg_settings.reg_addr = PHASE_SELECT_REG;
	reg_settings.reg_data = 0x00;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 10;

	for (i = PH_start; i <= PH_end; i++) {
		reg_settings.reg_data = i;
		rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
			&write_setting);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"failed to select PH %d", i);
			goto out;
		}

		reg_settings.reg_addr = PROXOFFSET_REG;
		reg_settings.reg_data =
			(ctrl->cap_sense.calibraion_data[i] >> 8) & 0x3F;
		rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
			&write_setting);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"failed to write PH %d MLB", i);
			goto out;
		}

		reg_settings.reg_addr = PROXOFFSET_REG+1;
		reg_settings.reg_data =
			ctrl->cap_sense.calibraion_data[i] & 0x00FF;
		rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
			&write_setting);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"failed to write PH %d LSB", i);
			goto out;
		}

		retry_cnt = 0;
		while (retry_cnt < MAX_RETRY_COUNT) {
			camera_io_dev_read(
				&ctrl->cap_sense.io_master_info,
				PROXOFFSET_REG,
				&data,
				CAMERA_SENSOR_I2C_TYPE_BYTE,
				CAMERA_SENSOR_I2C_TYPE_WORD);

			ctrl->cap_sense.proxoffset[i] = data & 0x3FFF;
			if (ctrl->cap_sense.proxoffset[i]
				== ctrl->cap_sense.calibraion_data[i]) {
				break;
			}
			retry_cnt++;
		}

		if (retry_cnt == MAX_RETRY_COUNT) {
			dev_err(ctrl->soc_info.dev,
				"read back offset %d mismatch to cali data %d",
				ctrl->cap_sense.proxoffset[i],
				ctrl->cap_sense.calibraion_data[i]);
		}
	}

out:
	return rc;

}

int sx9320_init_setting(struct led_laser_ctrl_t *ctrl)
{
	int rc, i;
	size_t settings_size = ARRAY_SIZE(cap_sense_init_reg_settings);
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings[settings_size];

	for (i = 0; i < settings_size; i++) {
		reg_settings[i].reg_addr = cap_sense_init_reg_settings[i].addr;
		reg_settings[i].reg_data = cap_sense_init_reg_settings[i].data;
		reg_settings[i].delay = 0;
		reg_settings[i].data_mask = 0;
	}
	write_setting.reg_setting = reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = settings_size;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
		&write_setting);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: i2c write failed: rc: %d",
			__func__, rc);
		return rc;
	}

	rc = sx9320_manual_compensation(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: manual compensation failed: rc: %d",
			__func__, rc);
	}

	sx9320_cleanup_nirq(ctrl);

	return rc;
}

static int silego_override_setting(
	struct led_laser_ctrl_t *ctrl, uint32_t addr, uint32_t data)
{
	int rc;
	uint32_t check_value;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	reg_settings.reg_addr = addr;
	reg_settings.reg_data = data;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->io_master_info, &write_setting);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed to overwrite setting: rc: %d",
			__func__, rc);
		return rc;
	}

	rc = camera_io_dev_read(
		&ctrl->io_master_info,
		reg_settings.reg_addr,
		&check_value,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed to read back setting: rc: %d",
			__func__, rc);
		return rc;
	}

	if (check_value != data) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed, expected 0x%x, got: 0x%x: rc: %d",
			__func__, data, check_value, rc);
		return -EINVAL;
	}
	return rc;
};

static int32_t silego_verify_settings(struct led_laser_ctrl_t *ctrl)
{
	uint32_t data;
	int rc, io_release_rc;
	size_t i, settings_size;
	uint32_t old_sid, old_cci_master;
	const struct reg_setting *reg_map;

	old_sid = ctrl->io_master_info.cci_client->sid;
	old_cci_master = ctrl->io_master_info.cci_client->cci_i2c_master;
	ctrl->io_master_info.cci_client->sid = 0x08;
	ctrl->io_master_info.cci_client->cci_i2c_master = 0;

	rc = camera_io_init(&(ctrl->io_master_info));
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: cam io init for silego failed: rc: %d",
			__func__, rc);
		ctrl->io_master_info.cci_client->sid = old_sid;
		ctrl->io_master_info.cci_client->cci_i2c_master =
			old_cci_master;
		return rc;
	}

	switch (ctrl->hw_version) {
	case BUILD_PROTO:
	case BUILD_DEV:
		reg_map = silego_reg_settings_ver1;
		settings_size = ARRAY_SIZE(silego_reg_settings_ver1);
		break;
	case BUILD_EVT1_0:
		reg_map = silego_reg_settings_ver2;
		settings_size = ARRAY_SIZE(silego_reg_settings_ver2);
		break;
	case BUILD_EVT1_1:
	default:
		reg_map = silego_reg_settings_ver3;
		settings_size = ARRAY_SIZE(silego_reg_settings_ver3);
		break;
	}

	for (i = 0; i < settings_size; i++) {
		rc = camera_io_dev_read(
			&ctrl->io_master_info,
			reg_map[i].addr,
			&data,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev, "%s failed on read 0x%x",
				__func__, reg_map[i].addr);
			goto out;
		}

		if (ctrl->hw_version == BUILD_PROTO &&
			reg_map[i].addr == 0xcd && data == 0xb3) {
			/* Some of Silego part isn't store final version value.
			 *  Need to overwrite to correct value to provide
			 *  proper functionality.
			 */
			rc = silego_override_setting(ctrl,
				reg_map[i].addr, reg_map[i].data);
			if (rc < 0)
				goto out;
		} else if (ctrl->hw_version < BUILD_EVT1_1 &&
			(reg_map[i].addr == 0xc1 || reg_map[i].addr == 0xc3)) {
			/* Update pulse width limitation to 3 ms */
			rc = silego_override_setting(ctrl,
				reg_map[i].addr, reg_map[i].data);
			if (rc < 0)
				goto out;
		} else if (data != reg_map[i].data) {
			dev_err(ctrl->soc_info.dev,
				"address 0x%x mismatch,"
				" expected 0x%x but got 0x%x",
				reg_map[i].addr,
				reg_map[i].data,
				data);
			rc = -EINVAL;
			goto out;
		}
	}
out:
	io_release_rc = camera_io_release(&(ctrl->io_master_info));
	if (io_release_rc < 0) {
		dev_err(ctrl->soc_info.dev, "%s: silego cci_release failed",
			__func__);
		if (!rc)
			rc = io_release_rc;
	}
	ctrl->io_master_info.cci_client->sid = old_sid;
	ctrl->io_master_info.cci_client->cci_i2c_master = old_cci_master;
	return rc;
}

static int lm36011_read_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t *data)
{
	int rc = 0;
	rc = camera_io_dev_read(
		&ctrl->io_master_info,
		addr,
		data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (rc != 0)
		pr_err("%s failed rc = %d", __func__, rc);
	else
		pr_debug("%s: got data 0x%x from 0x%x rc %d",
			__func__,
			*data,
			addr,
			rc);
	return rc;
}

static int lm36011_write_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t data)
{
	int rc;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;
	reg_settings.reg_addr = addr;
	reg_settings.reg_data = data;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->io_master_info, &write_setting);

	if (rc != 0)
		pr_err("%s failed rc = %d", __func__, rc);
	else
		pr_debug("%s: set data 0x%x to 0x%x rc %d",
			__func__,
			data,
			addr,
			rc);
	return rc;
}

static int lm36011_power_up(struct led_laser_ctrl_t *ctrl)
{
	int rc;

	if (!ctrl->is_power_up) {
		rc = regulator_set_voltage(ctrl->buck1,
			PMIC_BUCK1_VOlTAGE_MIN, PMIC_BUCK1_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set pmic buck1 voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->buck1);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"buck1 regulator_enable failed: rc: %d", rc);
			return rc;
		}

		rc = regulator_set_voltage(ctrl->buck2,
			PMIC_BUCK2_VOlTAGE_MIN, PMIC_BUCK2_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set pmic buck2 voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->buck2);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"buck2 regulator_enable failed: rc: %d", rc);
			return rc;
		}

		rc = regulator_set_voltage(ctrl->vio,
			VIO_VOLTAGE_MIN, VIO_VOLTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set vio voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->vio);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"vio regulator_enable failed: rc: %d", rc);
			return rc;
		}
		ctrl->is_power_up = true;
	}

	if (!ctrl->cap_sense.is_power_up) {
		rc = regulator_set_voltage(ctrl->cap_sense.vdd,
			SX9320_VDD_VOlTAGE_MIN, SX9320_VDD_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set cap sense vdd voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->cap_sense.vdd);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cap sense regulator_enable failed: rc: %d",
				rc);
			return rc;
		}
		ctrl->cap_sense.is_power_up = true;
	}

	if (!ctrl->silego.is_power_up) {
		rc = regulator_set_voltage(ctrl->silego.vdd,
			SLIEGO_VDD_VOlTAGE_MIN, SLIEGO_VDD_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set silego vdd voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->silego.vdd);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"silego regulator_enable failed: rc: %d", rc);
			return rc;
		}
		ctrl->silego.is_power_up = true;
	}

	if (!ctrl->is_cci_init) {
		rc = camera_io_init(&(ctrl->io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cam io init failed: rc: %d", rc);
			return rc;
		}
		ctrl->is_cci_init = true;
	}

	if (!ctrl->cap_sense.is_cci_init) {
		rc = camera_io_init(&(ctrl->cap_sense.io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cam io init for cap sense failed: rc: %d",
				rc);
			return rc;
		}
		ctrl->cap_sense.is_cci_init = true;
	}

	/* Silego i2c need at least 3 ms after power up */
	usleep_range(3000, 6000);

	rc = silego_verify_settings(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"verify silego reg setting failed: rc: %d",
			rc);
		ctrl->silego.is_validated = false;
		return rc;
	}
	ctrl->silego.is_validated = true;

	if (ctrl->type == LASER_FLOOD &&
		!IS_ERR_OR_NULL(ctrl->gpio_active_state)) {
		rc = pinctrl_select_state(ctrl->pinctrl,
			ctrl->gpio_active_state);
		if (rc < 0)
			dev_err(ctrl->soc_info.dev,
				"failed to set pin ctrl to active");
	}

	return rc;
}

static int lm36011_power_down(struct led_laser_ctrl_t *ctrl)
{
	int rc = 0, is_error;

	if (ctrl->is_cci_init) {
		is_error = camera_io_release(&(ctrl->io_master_info));
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"laser cci_release failed: rc: %d", rc);
		} else
			ctrl->is_cci_init = false;
	}

	if (ctrl->cap_sense.is_cci_init) {
		is_error = camera_io_release(&(ctrl->cap_sense.io_master_info));
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"laser cci_release failed: rc: %d", rc);
		} else
			ctrl->cap_sense.is_cci_init = false;
	}

	if (ctrl->silego.is_power_up) {
		is_error = regulator_disable(ctrl->silego.vdd);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"silego regulator_disable failed: rc: %d", rc);
		} else
			ctrl->silego.is_power_up = false;
	}

	if (ctrl->cap_sense.is_power_up) {
		is_error = regulator_disable(ctrl->cap_sense.vdd);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"cap sense regulator_disable failed: rc: %d",
				rc);
		} else
			ctrl->cap_sense.is_power_up = false;
	}

	if (ctrl->is_power_up) {
		is_error = regulator_disable(ctrl->vio) +
			regulator_set_voltage(ctrl->buck2,
				0, PMIC_BUCK2_VOlTAGE_MAX) +
			regulator_disable(ctrl->buck2) +
			regulator_disable(ctrl->buck1);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"laser regulator_disable failed: rc: %d", rc);
		} else
			ctrl->is_power_up = false;
	}

	if (ctrl->type == LASER_FLOOD &&
		!IS_ERR_OR_NULL(ctrl->gpio_suspend_state)) {
		rc = pinctrl_select_state(ctrl->pinctrl,
			ctrl->gpio_suspend_state);
		if (rc < 0)
			dev_err(ctrl->soc_info.dev,
				"failed to set pin ctrl to suspend");
	}

	return rc;
}

static irqreturn_t silego_ir_vcsel_fault_handler(int irq, void *dev_id)
{
	struct device *dev = (struct device *)dev_id;
	struct led_laser_ctrl_t *ctrl;

	if (!dev)
		return IRQ_NONE;

	dev_err(dev, "Silego under fault condition, irq: %d", irq);
	ctrl = dev_get_drvdata(dev);
	ctrl->silego.is_vcsel_fault = true;

	return IRQ_HANDLED;
}

static irqreturn_t silego_ir_vcsel_fault_count_handler(int irq, void *dev_id)
{
	struct device *dev = (struct device *)dev_id;
	struct led_laser_ctrl_t *ctrl;

	if (!dev)
		return IRQ_NONE;

	ctrl = dev_get_drvdata(dev);
	ctrl->silego.vcsel_fault_count++;
	dev_dbg(dev, "Silego fault count: %d, irq: %d",
		ctrl->silego.vcsel_fault_count, irq);

	return IRQ_HANDLED;
}

static irqreturn_t cap_sense_irq_handler(int irq, void *dev_id)
{
	struct device *dev = (struct device *)dev_id;
	struct led_laser_ctrl_t *ctrl;

	if (!dev)
		return IRQ_NONE;

	ctrl = dev_get_drvdata(dev);

	queue_work(ctrl->work_queue, &ctrl->work);

	return IRQ_HANDLED;
}

static void cap_sense_workq_job(struct work_struct *work)
{
	struct led_laser_ctrl_t *ctrl;
	uint32_t data;
	int rc;
	uint32_t i;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	ctrl = container_of(work, struct led_laser_ctrl_t, work);
	if (!ctrl) {
		dev_err(ctrl->soc_info.dev, "failed to get driver struct");
		return;
	}

	rc = sx9320_cleanup_nirq(ctrl);
	if (rc < 0)
		return;

	reg_settings.reg_addr = PHASE_SELECT_REG;
	reg_settings.reg_data = 0x00;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	for (i = 1; i < PHASENUM; i++) {
		reg_settings.reg_data = i;

		camera_io_dev_write(&ctrl->cap_sense.io_master_info,
			&write_setting);

		camera_io_dev_read(
			&ctrl->cap_sense.io_master_info,
			PROXAVG_REG,
			&data,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_WORD);

		ctrl->cap_sense.proxavg[i] = data & 0xFFFF;

		if (i == 2)
			sx9320_crack_detection(ctrl);

		if (read_proxoffset) {
			camera_io_dev_read(
				&ctrl->cap_sense.io_master_info,
				PROXOFFSET_REG,
				&data,
				CAMERA_SENSOR_I2C_TYPE_BYTE,
				CAMERA_SENSOR_I2C_TYPE_WORD);

			ctrl->cap_sense.proxoffset[i] = data & 0x3FFF;
			dev_info(ctrl->soc_info.dev, "proxoffset PH%d: %d",
					i, ctrl->cap_sense.proxoffset[i]);
			dev_info(ctrl->soc_info.dev, "proxavg PH%d: %d",
					i, ctrl->cap_sense.proxavg[i]);
		}
	}
}

static int lm36011_set_up_silego_irq(
	struct device *dev,
	unsigned int irq,
	int gpio_index)
{
	int rc;
	char *irq_name;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	switch (gpio_index) {
	case IR_VCSEL_FAULT:
		dev_dbg(dev, "setup irq IR_VCSEL_FAULT");
		irq_name = (ctrl->type == LASER_FLOOD) ?
			"ir_vcsel_fault_flood" : "ir_vcsel_fault_dot";
		rc = request_irq(irq, silego_ir_vcsel_fault_handler,
			IRQF_TRIGGER_FALLING, irq_name, dev);
		break;
	case IR_VCSEL_TEST:
		dev_dbg(dev, "setup irq IR_VCSEL_TEST");
		irq_name = (ctrl->type == LASER_FLOOD) ?
			"ir_vcsel_fault_count_flood" :
			"ir_vcsel_fault_count_dot";
		rc = request_irq(irq,
			silego_ir_vcsel_fault_count_handler,
			IRQF_TRIGGER_RISING,
			irq_name, dev);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int lm36011_set_up_cap_sense_irq(
	struct device *dev,
	unsigned int irq,
	int gpio_index)
{
	int rc;
	char *irq_name;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	switch (gpio_index) {
	case CSENSE_PROXAVG_READ:
		dev_dbg(dev, "setup irq CSENSE_PROXAVG_READ as IRQF_TRIGGER_FALLING");
		irq_name = "cap_sense_irq";
		irq_name = (ctrl->type == LASER_FLOOD) ?
			"cap_sense_irq_flood" : "cap_sense_irq_dot";
		rc = request_irq(irq, cap_sense_irq_handler,
			IRQF_TRIGGER_FALLING, irq_name, dev);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static void lm36011_enable_gpio_irq(struct device *dev)
{
	int index;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	if (ctrl->gpio_count <= 0)
		return;

	for (index = 0; index < SILEGO_GPIO_MAX; index++) {
		if (ctrl->silego.irq[index] <= 0) {
			dev_warn(dev, "%s no available irq for gpio: %d",
				__func__, ctrl->silego.gpio_array[index].gpio);
		} else if (index == SILEGO_SW_HALT) {
			gpio_request(ctrl->silego.gpio_array[index].gpio,
				"SILEGO_SW_HALT");
		} else
			lm36011_set_up_silego_irq(
				dev, ctrl->silego.irq[index], index);
	}

	for (index = 0; index < CAP_SENSE_GPIO_MAX; index++) {
		if (ctrl->cap_sense.irq[index] <= 0) {
			dev_warn(dev, "%s no available irq for gpio: %d",
				__func__, ctrl->cap_sense.irq[index]);
		} else
			lm36011_set_up_cap_sense_irq(
				dev, ctrl->cap_sense.irq[index], index);
	}
}

static void lm36011_disable_gpio_irq(struct device *dev)
{
	int index, gpio_level;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	if (ctrl->gpio_count <= 0)
		return;

	for (index = 0; index < SILEGO_GPIO_MAX; index++) {
		if (ctrl->silego.irq[index] <= 0) {
			dev_warn(dev, "%s no available irq for gpio: %d",
				__func__, ctrl->silego.gpio_array[index].gpio);
		} else if (index == SILEGO_SW_HALT) {
			gpio_level =
				gpio_get_value(
					ctrl->silego.gpio_array[index].gpio);
			if (gpio_level)
				gpio_set_value(
					ctrl->silego.gpio_array[index].gpio,
					0);
			gpio_free(ctrl->silego.gpio_array[index].gpio);
		} else
			free_irq(ctrl->silego.irq[index], dev);
	}

	for (index = 0; index < CAP_SENSE_GPIO_MAX; index++) {
		if (ctrl->cap_sense.irq[index] <= 0) {
			dev_warn(dev, "%s no available irq for gpio: %d",
				__func__, ctrl->cap_sense.irq[index]);
		} else
			free_irq(ctrl->cap_sense.irq[index], dev);
	}
}

static int lm36011_parse_pinctrl(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	if (ctrl->type == LASER_DOT)
		return 0;

	ctrl->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(ctrl->pinctrl)) {
		dev_err(dev, "getting pinctrl handle failed");
		return -EINVAL;
	}

	ctrl->gpio_active_state =
		pinctrl_lookup_state(ctrl->pinctrl,
				"lm36011_active");
	if (IS_ERR_OR_NULL(ctrl->gpio_active_state)) {
		dev_err(dev,
			"failed to get the gpio active state pinctrl handle");
		return -EINVAL;
	}

	ctrl->gpio_suspend_state =
		pinctrl_lookup_state(ctrl->pinctrl,
				"lm36011_suspend");
	if (IS_ERR_OR_NULL(ctrl->gpio_suspend_state)) {
		dev_err(dev,
			"failed to get the gpio suspend state pinctrl handle");
		return -EINVAL;
	}

	return 0;
}

static int lm36011_get_gpio_info(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int16_t gpio_array_size = 0, index;

	gpio_array_size = of_gpio_count(dev->of_node);
	ctrl->gpio_count = gpio_array_size;

	if (gpio_array_size <= 0)
		return 0;

	if (gpio_array_size > (SILEGO_GPIO_MAX + CAP_SENSE_GPIO_MAX)) {
		dev_err(dev, "too many gpio defined, max num: %d",
			(SILEGO_GPIO_MAX + CAP_SENSE_GPIO_MAX));
		return -EINVAL;
	}

	ctrl->silego.gpio_array = devm_kzalloc(dev,
		sizeof(struct gpio)*SILEGO_GPIO_MAX, GFP_KERNEL);
	if (!ctrl->silego.gpio_array) {
		dev_err(dev, "no memory for silego gpio");
		return -ENOMEM;
	}

	ctrl->cap_sense.gpio_array = devm_kzalloc(dev,
		sizeof(struct gpio)*CAP_SENSE_GPIO_MAX, GFP_KERNEL);
	if (!ctrl->silego.gpio_array) {
		dev_err(dev, "no memory for cap sense gpio");
		return -ENOMEM;
	}

	if (lm36011_parse_pinctrl(dev) < 0) {
		dev_err(dev, "failed to init gpio state");
		return -EINVAL;
	}

	for (index = 0; index < gpio_array_size; index++) {
		if (index < SILEGO_GPIO_MAX) {
			ctrl->silego.gpio_array[index].gpio =
				of_get_gpio(dev->of_node, index);
			ctrl->silego.irq[index] =
				gpio_to_irq(
					ctrl->silego.gpio_array[index].gpio);
		} else {
			ctrl->cap_sense.gpio_array[0].gpio =
				of_get_gpio(dev->of_node, index);
			ctrl->cap_sense.irq[0] =
				gpio_to_irq(
					ctrl->cap_sense.gpio_array[0].gpio);
		}
	}

	return 0;
}

static int lm36011_parse_dt(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int value = 0;

	ctrl->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(ctrl->vio)) {
		ctrl->vio = NULL;
		dev_err(dev, "unable to get vio");
		return -ENOENT;
	}

	ctrl->silego.vdd = devm_regulator_get(dev, "silego_vdd");
	if (IS_ERR(ctrl->silego.vdd)) {
		ctrl->silego.vdd = NULL;
		dev_err(dev, "unable to get silego vdd");
		return -ENOENT;
	}

	ctrl->cap_sense.vdd = devm_regulator_get(dev, "sx9320_vdd");
	if (IS_ERR(ctrl->cap_sense.vdd)) {
		ctrl->cap_sense.vdd = NULL;
		dev_err(dev, "unable to get cap sense vdd");
		return -ENOENT;
	}

	ctrl->buck1 = devm_regulator_get(dev, "pmic_buck1");
	if (IS_ERR(ctrl->buck1)) {
		ctrl->buck1 = NULL;
		dev_err(dev, "unable to get pmic buck1");
		return -ENOENT;
	}

	ctrl->buck2 = devm_regulator_get(dev, "pmic_buck2");
	if (IS_ERR(ctrl->buck2)) {
		ctrl->buck2 = NULL;
		dev_err(dev, "unable to get pmic buck2");
		return -ENOENT;
	}

	if (of_property_read_u32(dev->of_node, "laser-type", &value)) {
		dev_err(dev, "laser-type not specified in dt");
		return -ENOENT;
	}
	ctrl->type = value;

	if (of_property_read_u32(dev->of_node, "sx9320_sid", &value)) {
		dev_err(dev, "cap sense slave address not specified in dt");
		return -ENOENT;
	}
	ctrl->cap_sense.sid = value;
	if (lm36011_get_gpio_info(dev)) {
		dev_err(dev, "failed to parse gpio and irq info");
		return -ENOENT;
	}

	if (of_property_read_u32(dev->of_node, "hw-version", &value)) {
		dev_warn(dev, "hw version not specified in dt");
		ctrl->hw_version = 0;
	} else
		ctrl->hw_version = value;

	return 0;
}

static int32_t lm36011_update_i2c_info(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int32_t value = 0;

	if (of_property_read_u32(dev->of_node, "cci-master", &value)) {
		dev_err(dev, "cci master not specified in dt");
		return -EINVAL;
	}
	ctrl->io_master_info.cci_client->cci_i2c_master = value;

	if (of_property_read_u32(dev->of_node, "reg", &value)) {
		dev_err(dev, "slave address is not specified in dt");
		return -EINVAL;

	}
	ctrl->io_master_info.cci_client->sid = value;

	if (of_property_read_u32(dev->of_node, "cci-device", &value) ||
		value >= CCI_DEVICE_MAX) {
		dev_err(dev, "cci device is not specified in dt");
		return -EINVAL;

	}
	ctrl->io_master_info.cci_client->cci_device = value;
	ctrl->io_master_info.cci_client->retries = 3;
	ctrl->io_master_info.cci_client->id_map = 0;
	ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_MODE;

	/* Fill up cap sense io info */
	ctrl->cap_sense.io_master_info.cci_client->cci_device = value;
	ctrl->cap_sense.io_master_info.cci_client->retries = 3;
	ctrl->cap_sense.io_master_info.cci_client->id_map = 0;
	ctrl->cap_sense.io_master_info.cci_client->i2c_freq_mode =
		I2C_FAST_MODE;
	ctrl->cap_sense.io_master_info.cci_client->sid = ctrl->cap_sense.sid;
	ctrl->cap_sense.io_master_info.cci_client->cci_i2c_master = 0;
	ctrl->cap_sense.io_master_info.master_type = CCI_MASTER;

	return 0;
}

static ssize_t led_laser_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	bool is_enabled;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	is_enabled = (ctrl->is_power_up == true && ctrl->is_cci_init == true);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", is_enabled);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;
	bool value;

	if (!ctrl->silego.is_validated || !ctrl->cap_sense.is_validated) {
		dev_err(dev, "Safety ic not valid, silego:cap sense (%d, %d)",
			ctrl->silego.is_validated,
			ctrl->cap_sense.is_validated);
		return -EINVAL;
	}

	if (!ctrl->is_certified) {
		dev_err(dev, "Cannot enable laser due to uncertified");
		return -EINVAL;
	}

	rc = kstrtobool(buf, &value);
	if (rc != 0)
		return rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (value == true) {
		rc = lm36011_power_up(ctrl);
		if (rc != 0) {
			lm36011_power_down(ctrl);
			mutex_unlock(&ctrl->cam_sensor_mutex);
			return rc;
		}
		/* Prepare safety ic IRQ */
		if (ctrl->type == LASER_FLOOD) {
			if (ctrl->hw_version >= 2) {
				rc = sx9320_init_setting(ctrl);
				if (rc < 0) {
					dev_err(ctrl->soc_info.dev,
						"initialize cap sense failed");
					lm36011_power_down(ctrl);
					mutex_unlock(&ctrl->cam_sensor_mutex);
					return rc;
				}
				sx9320_crack_detection(ctrl);
			}
			lm36011_enable_gpio_irq(dev);
		}

		/* Clean up IRQ for PROTO and DEV device */
		if (ctrl->hw_version < 2)
			sx9320_cleanup_nirq(ctrl);

		rc = lm36011_write_data(ctrl,
			ENABLE_REG, IR_ENABLE_MODE);
		if (rc == 0)
			dev_info(dev, "Laser enabled");
	} else {
		if (ctrl->type == LASER_FLOOD)
			lm36011_disable_gpio_irq(dev);
		rc = lm36011_power_down(ctrl);
		if (rc == 0)
			dev_info(dev, "Laser disabled");
	}
	ctrl->silego.vcsel_fault_count = 0;
	ctrl->silego.is_vcsel_fault = false;
	ctrl->silego.is_csense_halt = false;
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return rc < 0 ? rc : count;
}

static ssize_t led_laser_read_byte_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%x\n", ctrl->read_data);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_read_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t addr = 0;
	uint32_t read_data = 0;
	int rc = 0;

	if (!ctrl->is_certified) {
		dev_err(dev, "Cannot enable laser due to uncertified");
		return -EINVAL;
	}

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &addr);
	if (rc)
		goto error_out;

	addr &= 0xFF;

	rc = lm36011_read_data(ctrl, addr, &read_data);
	if (rc < 0) {
		dev_err(dev, "i2c read failed, rc = %d", rc);
		goto error_out;
	} else {
		ctrl->read_addr = addr;
		ctrl->read_data = read_data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_write_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t addr;
	uint32_t data;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);

	if (rc)
		goto error_out;

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = lm36011_write_data(ctrl, addr, data);
	if (rc < 0) {
		dev_err(dev, "%s i2c write failed: %d.", __func__, rc);
		goto error_out;
	} else {
		if (addr == ctrl->read_addr)
			ctrl->read_data = data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t is_silego_validated_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.is_validated);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t silego_vcsel_fault_detected_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.is_vcsel_fault);
	return rc;
}

static ssize_t silego_vcsel_fault_count_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.vcsel_fault_count);
	return rc;
}

static ssize_t silego_is_cap_sense_halt_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.is_csense_halt);
	return rc;
}

static ssize_t is_certified_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->is_certified);
}

static ssize_t is_cap_sense_validated_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->cap_sense.is_validated);
}

static ssize_t cap_sense_proxvalue_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	if (!ctrl->cap_sense.is_power_up || !ctrl->is_power_up) {
		dev_warn(dev, "try to enable laser first");
		return -EINVAL;
	}

	rc = scnprintf(buf, PAGE_SIZE,
		"proxoffset PH1: %d, PH2: %d, PH3: %d\nproxavg PH1: %d, PH2: %d, PH3: %d\n",
			ctrl->cap_sense.proxoffset[1],
			ctrl->cap_sense.proxoffset[2],
			ctrl->cap_sense.proxoffset[3],
			ctrl->cap_sense.proxavg[1],
			ctrl->cap_sense.proxavg[2],
			ctrl->cap_sense.proxavg[3]);

	return rc;
}

static ssize_t cap_sense_write_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t addr;
	uint32_t data;
	int rc;

	if (!ctrl->cap_sense.is_power_up || !ctrl->cap_sense.is_cci_init) {
		dev_warn(dev, "try to enable laser first");
		return -EINVAL;
	}

	rc = kstrtouint(buf, 0, &value);

	if (rc < 0)
		return rc;
	if ((value & 0xFFFF0000) != 0) {
		dev_err(dev, "value %x out of boundary", value);
		return -EINVAL;
	}

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = sx9320_write_data(ctrl, addr, data);
	if (rc < 0) {
		dev_err(dev, "%s i2c write failed: %d.", __func__, rc);
		return rc;
	}

	return count;
}

static ssize_t itoc_cali_data_store_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	int rc;

	rc = kstrtouint(buf, 0, &value);

	if (rc < 0)
		return rc;

	ctrl->cap_sense.calibraion_data[1] = (value >> 16);
	ctrl->cap_sense.calibraion_data[2] = (value & 0x0000FFFF);

	return count;
}


static DEVICE_ATTR_RW(led_laser_enable);
static DEVICE_ATTR_RW(led_laser_read_byte);
static DEVICE_ATTR_WO(led_laser_write_byte);
static DEVICE_ATTR_RO(is_silego_validated);
static DEVICE_ATTR_RO(silego_vcsel_fault_detected);
static DEVICE_ATTR_RO(silego_vcsel_fault_count);
static DEVICE_ATTR_RO(silego_is_cap_sense_halt);
static DEVICE_ATTR_RO(is_certified);
static DEVICE_ATTR_RO(is_cap_sense_validated);
static DEVICE_ATTR_RO(cap_sense_proxvalue);
static DEVICE_ATTR_WO(cap_sense_write_byte);
static DEVICE_ATTR_WO(itoc_cali_data_store);

static struct attribute *led_laser_dev_attrs[] = {
	&dev_attr_led_laser_enable.attr,
	&dev_attr_led_laser_read_byte.attr,
	&dev_attr_led_laser_write_byte.attr,
	&dev_attr_is_silego_validated.attr,
	&dev_attr_silego_vcsel_fault_detected.attr,
	&dev_attr_silego_vcsel_fault_count.attr,
	&dev_attr_silego_is_cap_sense_halt.attr,
	&dev_attr_is_certified.attr,
	&dev_attr_is_cap_sense_validated.attr,
	&dev_attr_cap_sense_proxvalue.attr,
	&dev_attr_cap_sense_write_byte.attr,
	&dev_attr_itoc_cali_data_store.attr,
	NULL
};

ATTRIBUTE_GROUPS(led_laser_dev);

static int32_t lm36011_platform_remove(struct platform_device *pdev)
{
	struct led_laser_ctrl_t *ctrl;

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		dev_err(&pdev->dev, "led laser device is NULL");
		return 0;
	}

	if (!ctrl->is_probed)
		return 0;

	flush_workqueue(ctrl->work_queue);
	destroy_workqueue(ctrl->work_queue);
	class_destroy(ctrl->cl);
	cdev_del(&ctrl->c_dev);
	unregister_chrdev_region(ctrl->dev, 1);
	sysfs_remove_groups(&pdev->dev.kobj, led_laser_dev_groups);
	mutex_destroy(&ctrl->cam_sensor_mutex);
	lm36011_power_down(ctrl);
	return 0;
}

static int lm36011_open(struct inode *inode, struct file *file)
{
	struct led_laser_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct led_laser_ctrl_t, c_dev);
	get_device(ctrl->soc_info.dev);
	file->private_data = ctrl;
	return 0;
}

static int lm36011_release(struct inode *inode, struct file *file)
{
	struct led_laser_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct led_laser_ctrl_t, c_dev);
	put_device(ctrl->soc_info.dev);
	return 0;
}

static long lm36011_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int rc = 0;
	struct led_laser_ctrl_t *ctrl = file->private_data;

	switch (cmd) {
	case LM36011_SET_CERTIFICATION_STATUS:
		ctrl->is_certified = (arg == 1 ? true : false);
		break;
	default:
		dev_err(ctrl->soc_info.dev,
			"%s: Unsupported ioctl command %u", __func__, cmd);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static const struct file_operations lm36011_fops = {
	.owner		= THIS_MODULE,
	.open		= lm36011_open,
	.release	= lm36011_release,
	.unlocked_ioctl	= lm36011_ioctl,
};

static int32_t lm36011_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc;
	uint32_t device_id = 0;
	uint32_t i;
	struct device *dev_ret;
	struct led_laser_ctrl_t *ctrl;
	char *class_name, *device_name;

	if (cam_cci_get_subdev(CCI_DEVICE_0) == NULL ||
		cam_cci_get_subdev(CCI_DEVICE_1) == NULL) {
		dev_warn(&pdev->dev, "wait for cci driver probe");
		return -EPROBE_DEFER;
	}

	/* Create sensor control structure */
	ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct led_laser_ctrl_t), GFP_KERNEL);
	if (!ctrl) {
		dev_err(&pdev->dev, "no memory for driver ctrl");
		return -ENOMEM;
	}

	/*fill in platform device*/
	ctrl->soc_info.pdev = pdev;
	ctrl->soc_info.dev = &pdev->dev;
	ctrl->soc_info.dev_name = pdev->name;
	ctrl->io_master_info.master_type = CCI_MASTER;
	ctrl->is_power_up = false;
	ctrl->is_cci_init = false;
	ctrl->is_probed = false;
	ctrl->silego.is_power_up = false;
	ctrl->silego.is_validated = false;
	ctrl->silego.is_vcsel_fault = false;
	ctrl->silego.is_csense_halt = false;
	ctrl->silego.vcsel_fault_count = 0;
	ctrl->cap_sense.is_validated = false;
	for (i = 0; i < LASER_TYPE_MAX; i++)
		ctrl->cap_sense.is_crack_detected[i] = false;
	for (i = 0; i < PHASENUM; i++)
		ctrl->cap_sense.calibraion_data[i] = 0;

	ctrl->io_master_info.cci_client = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->io_master_info.cci_client)) {
		dev_err(&pdev->dev, "no memory for cci client");
		return -ENOMEM;
	}

	ctrl->cap_sense.io_master_info.cci_client = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->cap_sense.io_master_info.cci_client)) {
		dev_err(&pdev->dev, "no memory for cap sense cci client");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ctrl);
	dev_set_drvdata(&pdev->dev, ctrl);

	rc = lm36011_parse_dt(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "paring led laser dt failed rc %d", rc);
		return rc;
	}

	rc = lm36011_update_i2c_info(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "update i2c info failed rc %d", rc);
		return rc;
	}

	/* Fill platform device id*/
	pdev->id = ctrl->soc_info.index;

	mutex_init(&ctrl->cam_sensor_mutex);

	rc = sysfs_create_groups(&pdev->dev.kobj, led_laser_dev_groups);
	if (rc != 0) {
		dev_err(&pdev->dev, "failed to create sysfs files");
		goto error_destroy_mutex;
	}

	rc = alloc_chrdev_region(&ctrl->dev, 0, 1, "lm36011_ioctl");
	if (rc)
		goto error_remove_sysfs;

	cdev_init(&ctrl->c_dev, &lm36011_fops);

	rc = cdev_add(&ctrl->c_dev, ctrl->dev, 1);
	if (rc)
		goto error_unregister_chrdev;

	class_name = (ctrl->type == LASER_FLOOD ? "char_flood" : "char_dot");
	ctrl->cl = class_create(THIS_MODULE, class_name);
	rc = IS_ERR(ctrl->cl);
	if (rc)
		goto error_del_cdev;

	device_name =
		(ctrl->type == LASER_FLOOD ? "lm36011_flood" : "lm36011_dot");
	dev_ret = device_create(ctrl->cl, NULL, ctrl->dev, NULL, device_name);
	rc = IS_ERR(dev_ret);
	if (rc)
		goto error_destroy_class;

	INIT_WORK(&ctrl->work, cap_sense_workq_job);
	device_name =
		(ctrl->type == LASER_FLOOD ? "flood_workq" : "dot_workq");
	ctrl->work_queue = create_workqueue(device_name);

	/* Read device id */
	lm36011_power_up(ctrl);

	rc = sx9320_cleanup_nirq(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"clean up cap sense irq failed: rc: %d", rc);
		ctrl->cap_sense.is_validated = false;
	} else
		ctrl->cap_sense.is_validated = true;

	rc = lm36011_read_data(ctrl,
		DEVICE_ID_REG, &device_id);
	if (rc != 0) {
		lm36011_power_down(ctrl);
		goto error_destroy_device;
	}

	rc = lm36011_power_down(ctrl);
	if (rc != 0)
		goto error_destroy_device;

	if (device_id == DEVICE_ID)
		_dev_info(&pdev->dev, "probe success, device id 0x%x rc = %d",
			device_id, rc);
	else
		dev_warn(&pdev->dev, "Device id mismatch, got 0x%x,"
			" expected 0x%x rc = %d", device_id, DEVICE_ID, rc);

	ctrl->is_probed = true;
	return rc;

error_destroy_device:
	device_destroy(ctrl->cl, ctrl->dev);
error_destroy_class:
	class_destroy(ctrl->cl);
error_del_cdev:
	cdev_del(&ctrl->c_dev);
error_unregister_chrdev:
	unregister_chrdev_region(ctrl->dev, 1);
error_remove_sysfs:
	sysfs_remove_groups(&pdev->dev.kobj, led_laser_dev_groups);
error_destroy_mutex:
	mutex_destroy(&ctrl->cam_sensor_mutex);
	return rc;
}


static const struct of_device_id lm36011_driver_dt_match[] = {
	{.compatible = "qcom,cam-led-laser"},
	{}
};

MODULE_DEVICE_TABLE(of, lm36011_driver_dt_match);

static struct platform_driver lm36011_platform_driver = {
	.probe = lm36011_driver_platform_probe,
	.driver = {
		.name = LM36011_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm36011_driver_dt_match,
	},
	.remove = lm36011_platform_remove,
};

static int __init lm36011_init(void)
{
	return platform_driver_register(&lm36011_platform_driver);
}

static void __exit lm36011_exit(void)
{
	platform_driver_unregister(&lm36011_platform_driver);
}

MODULE_DESCRIPTION("Led laser driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Speth Chang <spethchang@google.com>");

module_init(lm36011_init);
module_exit(lm36011_exit);

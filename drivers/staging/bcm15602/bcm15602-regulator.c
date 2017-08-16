/*
 * Copyright (C) 2016 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
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

/* #define DEBUG */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "bcm15602-regulator.h"

#define DRIVER_NAME "bcm15602"

/* defines the number of tries to repeat an I2C transaction */
#define BCM15602_I2C_RETRY_COUNT 10

/* defines the delay in ms for reset completion */
#define BCM15602_PON_RESET_DELAY 35

/* defines the timeout in jiffies for reset completion */
#define BCM15602_PON_RESET_TIMEOUT msecs_to_jiffies(15)

/* defines the number of retries for powering on */
#define BCM15602_PON_RETRY_CNT 4

/* defines the timeout in jiffies for reset completion after shutdown */
#define BCM15602_SHUTDOWN_RESET_TIMEOUT msecs_to_jiffies(10000)

/* defines the timeout in jiffies for ADC conversion completion */
#define BCM15602_ADC_CONV_TIMEOUT  msecs_to_jiffies(100)

static int bcm15602_chip_init(struct bcm15602_chip *ddata);
static int bcm15602_regulator_set_voltage(struct regulator_dev *rdev,
					  int min_uV, int max_uV,
					  unsigned *selector);
static int bcm15602_regulator_get_voltage(struct regulator_dev *rdev);
static int bcm15602_regulator_enable(struct regulator_dev *rdev);
static int bcm15602_regulator_disable(struct regulator_dev *rdev);
static int bcm15602_regulator_is_enabled(struct regulator_dev *rdev);
static void bcm15602_enable_wdt(struct bcm15602_chip *ddata);
static void bcm15602_disable_wdt(struct bcm15602_chip *ddata);

static const struct mfd_cell bcm15602_devs[] = {
	{
		.name = "bcm15602-gpio",
		.of_compatible = "brcm,bcm15602-gpio",
	},
	{
		.name = "bcm15602-thermal",
		.of_compatible = "brcm,bcm15602-thermal",
	},
};

/* adc slot configurations */
static const struct bcm15602_adc_ctrl adc_ctrls[] = {
	[BCM15602_HK_ASR_MSTR_CURR_OVERI] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_ASR_MSTR_CURR,
		.threshold = BCM15602_HK_TH_ASR_MSTR_CURR,
		.ot_ut = BCM15602_ADC_OVER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_ASR_SLV_CURR_OVERI] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_ASR_SLV_CURR,
		.threshold = BCM15602_HK_TH_ASR_SLV_CURR,
		.ot_ut = BCM15602_ADC_OVER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_SDSR_CURR_OVERI] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_SDSR_CURR,
		.threshold = BCM15602_HK_TH_SDSR_CURR,
		.ot_ut = BCM15602_ADC_OVER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_SDLDO_CURR_OVERI] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_SDLDO_CURR,
		.threshold = BCM15602_HK_TH_SDLDO_CURR,
		.ot_ut = BCM15602_ADC_OVER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_IOLDO_CURR_OVERI] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_IOLDO_CURR,
		.threshold = BCM15602_HK_TH_IOLDO_CURR,
		.ot_ut = BCM15602_ADC_OVER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_VBAT_UNDERV0] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_VBAT,
		.threshold = BCM15602_HK_TH_VBAT_UNDERV0,
		.ot_ut = BCM15602_ADC_UNDER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_VBAT_UNDERV1] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_VBAT,
		.threshold = BCM15602_HK_TH_VBAT_UNDERV1,
		.ot_ut = BCM15602_ADC_UNDER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_VBAT_UNDERV2] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_VBAT,
		.threshold = BCM15602_HK_TH_VBAT_UNDERV2,
		.ot_ut = BCM15602_ADC_UNDER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_PTAT_OVERT] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_PTAT,
		.threshold = BCM15602_HK_TH_PTAT_OVERT,
		.ot_ut = BCM15602_ADC_OVER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
	[BCM15602_HK_PTAT_UNDERT] = {
		.state = BCM15602_ADC_STATE_ACTIVE,
		.ovsp = BCM15602_ADC_OVSP_1,
		.chan_num = BCM15602_ADC_PTAT,
		.threshold = BCM15602_HK_TH_PTAT_UNDERT,
		.ot_ut = BCM15602_ADC_UNDER_TH,
		.deb_en = 0,
		.th_en = 1,
	},
};

static const struct regmap_config bcm15602_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static struct regulator_ops bcm15602_regulator_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_ascend,
	.set_voltage = bcm15602_regulator_set_voltage,
	.get_voltage = bcm15602_regulator_get_voltage,
	.enable = bcm15602_regulator_enable,
	.disable = bcm15602_regulator_disable,
	.is_enabled = bcm15602_regulator_is_enabled,
};

/* No support for DVS so just a single voltage level */
static const unsigned int bcm15602_ldo_vtbl[] = { 1800000 };
static const unsigned int bcm15602_asr_vtbl[] = { 900000, 1000000 };
static const unsigned int bcm15602_sdsr_vtbl[] = { 1100000 };

static struct regulator_desc
	bcm15602_regulator_desc[BCM15602_NUM_REGULATORS] = {
	[BCM15602_ID_SDLDO] = {
		.name = BCM15602_REGLTR_NAME_SDLDO,
		.id = BCM15602_ID_SDLDO,
		.ops = &bcm15602_regulator_ops,
		.n_voltages = ARRAY_SIZE(bcm15602_ldo_vtbl),
		.volt_table = bcm15602_ldo_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[BCM15602_ID_IOLDO] = {
		.name = BCM15602_REGLTR_NAME_IOLDO,
		.id = BCM15602_ID_IOLDO,
		.ops = &bcm15602_regulator_ops,
		.n_voltages = ARRAY_SIZE(bcm15602_ldo_vtbl),
		.volt_table = bcm15602_ldo_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[BCM15602_ID_ASR] = {
		.name = BCM15602_REGLTR_NAME_ASR,
		.id = BCM15602_ID_ASR,
		.ops = &bcm15602_regulator_ops,
		.n_voltages = ARRAY_SIZE(bcm15602_asr_vtbl),
		.volt_table = bcm15602_asr_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[BCM15602_ID_SDSR] = {
		.name = BCM15602_REGLTR_NAME_SDSR,
		.id = BCM15602_ID_SDSR,
		.ops = &bcm15602_regulator_ops,
		.n_voltages = ARRAY_SIZE(bcm15602_sdsr_vtbl),
		.volt_table = bcm15602_sdsr_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static struct regulator_init_data
	bcm15602_regulator_init_data[BCM15602_NUM_REGULATORS] = {
	[BCM15602_ID_SDLDO] = {
		.constraints = {
			.name = "bcm15602_sdldo",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 1800000,
			.max_uV = 1800000,
		},
	},
	[BCM15602_ID_IOLDO] = {
		.constraints = {
			.name = "bcm15602_ioldo",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 1800000,
			.max_uV = 1800000,
		},
	},
	[BCM15602_ID_ASR] = {
		.constraints = {
			.name = "bcm15602_asr",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 900000,
			.max_uV = 1000000,
		},
	},
	[BCM15602_ID_SDSR] = {
		.constraints = {
			.name = "bcm15602_sdsr",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 1100000,
			.max_uV = 1100000,
		},
	},
};

static int bcm15602_toggle_pon(struct bcm15602_chip *ddata)
{
	dev_err(ddata->dev, "%s: device is stuck, toggling PON\n", __func__);

	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 0);
	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 1);

	return 0;
}

static inline bool bcm15602_is_ready(struct bcm15602_chip *ddata)
{
	return (gpio_get_value(ddata->pdata->resetb_gpio) == 1);
}

int bcm15602_read_byte(struct bcm15602_chip *ddata, u8 addr, u8 *data)
{
	int ret;
	unsigned int val;
	int retry_cnt = 0;

	if (!bcm15602_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_read(ddata->regmap, addr, &val);
		if (!ret) {
			*data = (u8)val;
			return 0;
		}

		dev_err(ddata->dev,
			"failed to read addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < BCM15602_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, BCM15602_I2C_RETRY_COUNT);
	bcm15602_toggle_pon(ddata);

	return -EIO;
}
EXPORT_SYMBOL_GPL(bcm15602_read_byte);

int bcm15602_read_bytes(struct bcm15602_chip *ddata, u8 addr, u8 *data,
		       size_t count)
{
	int ret;
	int retry_cnt = 0;

	if (!bcm15602_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_bulk_read(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev, "%s: failed to read %zd bytes from addr 0x%.2x (%d), retry %d\n",
			__func__, count, addr, ret, retry_cnt);
	} while (++retry_cnt < BCM15602_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, BCM15602_I2C_RETRY_COUNT);
	bcm15602_toggle_pon(ddata);

	return -EIO;
}
EXPORT_SYMBOL_GPL(bcm15602_read_bytes);

int bcm15602_write_byte(struct bcm15602_chip *ddata, u8 addr, u8 data)
{
	int ret;
	int retry_cnt = 0;

	if (!bcm15602_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_write(ddata->regmap, addr, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < BCM15602_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, BCM15602_I2C_RETRY_COUNT);
	bcm15602_toggle_pon(ddata);

	return -EIO;
}
EXPORT_SYMBOL_GPL(bcm15602_write_byte);

int bcm15602_write_bytes(struct bcm15602_chip *ddata, u8 addr, u8 *data,
			 size_t count)
{
	int ret;
	int retry_cnt = 0;

	if (!bcm15602_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_bulk_write(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write %zd bytes to addr 0x%.2x (%d), retry %d\n",
			count, addr, ret, retry_cnt);
	} while (++retry_cnt < BCM15602_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, BCM15602_I2C_RETRY_COUNT);
	bcm15602_toggle_pon(ddata);

	return -EIO;
}
EXPORT_SYMBOL_GPL(bcm15602_write_bytes);

int bcm15602_update_bits(struct bcm15602_chip *ddata, u8 addr,
			 unsigned int mask, u8 data)
{
	int ret;
	int retry_cnt = 0;

	if (!bcm15602_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_update_bits(ddata->regmap, addr, mask, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to update addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < BCM15602_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, BCM15602_I2C_RETRY_COUNT);
	bcm15602_toggle_pon(ddata);

	return -EIO;
}
EXPORT_SYMBOL_GPL(bcm15602_update_bits);

int bcm15602_read_adc_chan(struct bcm15602_chip *ddata,
			   int chan_num, u16 *chan_data)
{
	u8 bytes[2];
	int ret = 0;
	unsigned long timeout;

	/* serialize manual adc conversions */
	if (test_and_set_bit(0, &ddata->adc_conv_busy))
		return -EAGAIN;

	/* clear adc conversion completion before starting conversion */
	reinit_completion(&ddata->adc_conv_complete);

	/* enable the ADC clock and write the channel number to trigger
	 * the conversion */
	bytes[0] = 0x1;
	bytes[1] = BCM15602_ADC_OVSP_4 | chan_num;
	ret = bcm15602_write_bytes(ddata, BCM15602_REG_ADC_MAN_CTRL, bytes, 2);
	if (ret)
		goto adc_cleanup;
	dev_dbg(ddata->dev, "%s: started ADC converstion\n", __func__);

	/* wait for completion signaled by interrupt */
	timeout = wait_for_completion_timeout(&ddata->adc_conv_complete,
					  BCM15602_ADC_CONV_TIMEOUT);
	if (!timeout) {
		ret = -ETIMEDOUT;
		dev_err(ddata->dev, "%s: ADC converstion timeout\n", __func__);
		goto finish;
	}

	/* read and format the conversion result */
	ret = bcm15602_read_bytes(ddata, BCM15602_REG_ADC_MAN_RESULT_L, bytes,
				  2);
	if (ret)
		goto adc_cleanup;
	*chan_data = ((bytes[1] & 0x7F) << 3) | (bytes[0] & 0x7);

finish:
	/* disable the ADC clock */
	bcm15602_write_byte(ddata, BCM15602_REG_ADC_MAN_CTRL, 0x0);

adc_cleanup:
	/* release adc conversion */
	clear_bit(0, &ddata->adc_conv_busy);

	return ret;
}
EXPORT_SYMBOL_GPL(bcm15602_read_adc_chan);

int bcm15602_read_hk_slot(struct bcm15602_chip *ddata,
			  int slot_num, u16 *slot_data)
{
	u8 reading_mask[2];
	u8 zeros[2] = {0, 0};
	u8 byte = 0;
	int ret = 0;

	/* serialize reads to the hk slots */
	if (test_and_set_bit(0, &ddata->hk_read_busy))
		return -EAGAIN;

	/*
	 * set the reading mask so the adc does not update the slot data while
	 * we are performing a read
	 */
	reading_mask[0] = (1 << slot_num) & 0xFF;
	reading_mask[1] = ((1 << slot_num) >> 8) & 0xF;
	ret = bcm15602_write_bytes(ddata, BCM15602_REG_ADC_SLOTDATA_READINGL,
				   reading_mask, 2);
	if (ret)
		goto read_hk_cleanup;

	ret = bcm15602_read_byte(ddata, BCM15602_REG_ADC_SLOTDATA0 + slot_num,
				 &byte);
	if (ret)
		goto read_hk_cleanup;
	*slot_data = (byte << 2);
	ret = bcm15602_read_byte(ddata, BCM15602_REG_ADC_SLOTDATA3_0_LSB +
				 (slot_num / 4), &byte);
	if (ret)
		goto read_hk_cleanup;
	*slot_data |= (byte >> (slot_num % 4)) & 0x3;

	/* unset the reading mask */
	ret = bcm15602_write_bytes(ddata, BCM15602_REG_ADC_SLOTDATA_READINGL,
				   zeros, 2);
	if (ret)
		goto read_hk_cleanup;

read_hk_cleanup:
	/* release hk read */
	clear_bit(0, &ddata->hk_read_busy);

	return ret;
}
EXPORT_SYMBOL_GPL(bcm15602_read_hk_slot);

/* reset the watchdog timer */
static void bcm15602_clear_wdt(struct bcm15602_chip *ddata)
{
	struct device *dev = ddata->dev;

	dev_dbg(dev, "clearing watchdog timer alarm\n");

	bcm15602_write_byte(ddata, BCM15602_REG_WDT_WDTCTRL2, 0x1E);
}

#define NOTIFY(id, event) regulator_notifier_call_chain(ddata->rdevs[id], \
	event, NULL)

/* read housekeeping adc slot status and notify any polling threads */
static int bcm15602_handle_hk_int(struct bcm15602_chip *ddata)
{
	struct device *dev = ddata->dev;
	u16 status;
	u8 bytes[2];
	int ret;

	/* read interrupt status flags */
	ret = bcm15602_read_bytes(ddata, BCM15602_REG_ADC_SLOTSTAL, bytes, 2);
	if (ret)
		return ret;
	status = ((bytes[1] & 0x3) << 8) | bytes[0];
	dev_dbg(dev, "%s: ADC_SLOTSTA: 0x%04x\n", __func__, status);

	/* clear handled interrupts */
	ret = bcm15602_write_bytes(ddata, BCM15602_REG_ADC_SLOTSTAL, bytes, 2);
	if (ret)
		return ret;

	ddata->hk_status = status;

	/* notify the userspace thread that is polling on the status */
	sysfs_notify(&dev->kobj, NULL, "hk_status");

	return 0;
}

/* print the device id */
static void bcm15602_print_id(struct bcm15602_chip *ddata)
{
	struct device *dev = ddata->dev;
	u8 bytes[4];
	int ret;

	ret = bcm15602_read_bytes(ddata, BCM15602_REG_SYS_PMIC_ID_LSB, bytes, 4);

	if (!ret) {
		dev_info(dev,
			 "Part: 0x%02x%02x, Rev: %d, Vendor Rev: 0x%02x\n",
			 bytes[1], bytes[0], bytes[3], bytes[2]);

		/* byte 3 is the customer supplied id */
		ddata->rev_id = bytes[3];
	} else {
		dev_err(dev, "%s: Could not read PMIC ID (%d)\n", __func__,
			ret);
	}
}

/* print the power state machine status register */
static void bcm15602_print_psm_status(struct bcm15602_chip *ddata)
{
	struct device *dev = ddata->dev;
	u8 byte;
	int ret;

	ret = bcm15602_read_byte(ddata, BCM15602_REG_PSM_PSM_ENV, &byte);

	if (!ret)
		dev_err(dev, "%s: PSM Status: 0x%02x\n", __func__, byte);
}

#define BCM15602_PSM_ENV_THERM_SHDWN_BIT         0x20
#define BCM15602_PSM_ENV_SWCMD_EMSHDWN_BIT       0x10
#define BCM15602_PSM_ENV_SWCMD_SHDWN_BIT         0x08
#define BCM15602_PSM_ENV_WDTEXP_SHDWN            0x04
#define BCM15602_PSM_ENV_TOOHOT_SHDWN            0x02
#define BCM15602_PSM_ENV_VBATUVB_FALL_SHDWN_BIT  0x01

/* print the shutdown event register */
static void bcm15602_print_psm_event(struct bcm15602_chip *ddata)
{
	struct device *dev = ddata->dev;
	char buffer[80] = "normal";
	u8 byte;
	int ret;

	ret = bcm15602_read_byte(ddata, BCM15602_REG_PSM_PSM_ENV, &byte);

	if (!ret) {
		if (byte & BCM15602_PSM_ENV_THERM_SHDWN_BIT)
			strncpy(buffer, "downstream thermal trip", 80);
		else if (byte & BCM15602_PSM_ENV_SWCMD_EMSHDWN_BIT)
			strncpy(buffer, "software emergency shutdown", 80);
		else if (byte & BCM15602_PSM_ENV_SWCMD_SHDWN_BIT)
			strncpy(buffer, "software shutdown", 80);
		else if (byte & BCM15602_PSM_ENV_WDTEXP_SHDWN)
			strncpy(buffer, "watchdog timer expiration", 80);
		else if (byte & BCM15602_PSM_ENV_TOOHOT_SHDWN)
			strncpy(buffer, "over temperature", 80);
		else if (byte & BCM15602_PSM_ENV_VBATUVB_FALL_SHDWN_BIT)
			strncpy(buffer, "supply under voltage", 80);
		dev_info(dev, "Last reboot reason: %s\n", buffer);
	}
}

/* handle an interrupt flag */
static int bcm15602_handle_int(struct bcm15602_chip *ddata,
			       unsigned int flag_num)
{
	struct device *dev = ddata->dev;

	dev_dbg(dev, "%s: flag %d\n", __func__, flag_num);

	switch (flag_num) {
	case BCM15602_INT_ASR_OVERI:
		dev_err(dev, "Detected ASR over-current event\n");
		NOTIFY(BCM15602_ID_ASR, REGULATOR_EVENT_OVER_CURRENT);
		break;

	case BCM15602_INT_ASR_SHUTDOWN:
		dev_err(dev, "Detected ASR shutdown event\n");
		NOTIFY(BCM15602_ID_ASR, REGULATOR_EVENT_FAIL);
		break;

	case BCM15602_INT_ASR_UV:
		dev_err(dev, "Detected ASR under-voltage event\n");
		NOTIFY(BCM15602_ID_ASR, REGULATOR_EVENT_UNDER_VOLTAGE);
		break;

	case BCM15602_INT_SDSR_OVERI:
		dev_err(dev, "Detected SDSR over-current event\n");
		NOTIFY(BCM15602_ID_SDSR, REGULATOR_EVENT_OVER_CURRENT);
		break;

	case BCM15602_INT_SDSR_SHUTDOWN:
		dev_err(dev, "Detected SDSR shutdown event\n");
		NOTIFY(BCM15602_ID_SDSR, REGULATOR_EVENT_FAIL);
		break;

	case BCM15602_INT_IOLDO_OVERI:
		dev_err(dev, "Detected IOLDO over-current event\n");
		NOTIFY(BCM15602_ID_IOLDO, REGULATOR_EVENT_OVER_CURRENT);
		break;

	case BCM15602_INT_IOLDO_SHUTDOWN:
		dev_err(dev, "Detected IOLDO shutdown event\n");
		NOTIFY(BCM15602_ID_IOLDO, REGULATOR_EVENT_FAIL);
		break;

	case BCM15602_INT_SDLDO_OVERI:
		dev_err(dev, "Detected SDLDO over-current event\n");
		NOTIFY(BCM15602_ID_SDLDO, REGULATOR_EVENT_OVER_CURRENT);
		break;

	case BCM15602_INT_SDLDO_SHUTDOWN:
		dev_err(dev, "Detected SDLDO shutdown event\n");
		NOTIFY(BCM15602_ID_SDLDO, REGULATOR_EVENT_FAIL);
		break;

	case BCM15602_INT_ADC_HK_INT:
		bcm15602_handle_hk_int(ddata);
		break;

	case BCM15602_INT_ADC_CONV_DONE:
		dev_dbg(dev, "%s: completing adc conversion\n", __func__);
		complete(&ddata->adc_conv_complete);
		break;

	case BCM15602_INT_OTP_ECC_FAULT:
		dev_dbg(dev, "Ignoring interrupt flag %d\n", flag_num);
		break;

	case BCM15602_INT_WDT_ALARM:
		bcm15602_clear_wdt(ddata);
		break;

	case BCM15602_INT_WDT_EXP:
	case BCM15602_INT_THERM_TRIP:
	case BCM15602_INT_THERM_TRIP_DONE:
		if (flag_num == BCM15602_INT_WDT_EXP)
			dev_err(dev, "Watchdog timer expired\n");
		else if (flag_num == BCM15602_INT_THERM_TRIP)
			dev_err(dev, "Received thermal trip event from device\n");
		else
			dev_err(dev, "Thermal condition has returned to normal\n");

		/* notify regulator clients of failures */
		NOTIFY(BCM15602_ID_ASR, REGULATOR_EVENT_FAIL);
		NOTIFY(BCM15602_ID_SDSR, REGULATOR_EVENT_FAIL);
		NOTIFY(BCM15602_ID_SDLDO, REGULATOR_EVENT_FAIL);
		NOTIFY(BCM15602_ID_IOLDO, REGULATOR_EVENT_FAIL);
		bcm15602_print_psm_status(ddata);
		break;

	default:
		dev_err(dev, "%s: Unknown flag %d\n", __func__, flag_num);
		break;
	}

	return 0;
}

/* find pending interrupt flags */
static int bcm15602_check_int_flags(struct bcm15602_chip *ddata)
{
	u8 flags[4], flags_dup[4], flag_mask, flag_clr_mask[4];
	unsigned int first_bit, flag_num;
	int ret = 0;
	int i;

	/* read interrupt status flags */
	ret = bcm15602_read_bytes(ddata, BCM15602_REG_INT_INTFLAG1, flags, 4);
	if (ret)
		return ret;
	ret = bcm15602_read_bytes(ddata, BCM15602_REG_INT_INTFLAG1, flags_dup,
				  4);
	if (ret)
		return ret;

	dev_dbg(ddata->dev,
		"%s: [0] = 0x%02x, [1] = 0x%02x, [2] = 0x%02x, [3] = 0x%02x\n",
		__func__, flags[0], flags[1], flags[2], flags[3]);

	/* iterate through each interrupt */
	for (i = 0; i < 4; i++) {
		flag_clr_mask[i] = 0;

		/*
		 * Workaround for b/63872924; bitwise AND the flags to reject
		 * false interrupts
		 */
		if (flags[i] != flags_dup[i])
			dev_err(ddata->dev,
				"%s: mismatch irq for flag %d, 0x%02x != 0x%02x\n",
				__func__, i, flags[i], flags_dup[i]);
		flags[i] &= flags_dup[i];

		while (flags[i]) {
			/* find first set interrupt flag */
			first_bit = ffs(flags[i]);
			flag_mask = 1 << (first_bit - 1);
			flag_num = (i * 8) + (first_bit - 1);

			/* handle interrupt */
			ret = bcm15602_handle_int(ddata, flag_num);

			flags[i] &= ~flag_mask;
			flag_clr_mask[i] |=  flag_mask;
		}
	}

	/* clear handled interrupts */
	ret = bcm15602_write_bytes(ddata, BCM15602_REG_INT_INTFLAG1,
				   flag_clr_mask, 4);

	return ret;
}

/* kernel thread for waiting for chip to come out of reset */
static void bcm15602_reset_work(struct work_struct *data)
{
	struct bcm15602_chip *ddata = container_of(data, struct bcm15602_chip,
						   reset_work);
	unsigned long timeout;

	/* notify regulators of shutdown event */
	dev_err(ddata->dev,
		"%s: Notifying regulators of shutdown event\n", __func__);
	NOTIFY(BCM15602_ID_ASR, REGULATOR_EVENT_FAIL);
	NOTIFY(BCM15602_ID_SDSR, REGULATOR_EVENT_FAIL);
	NOTIFY(BCM15602_ID_IOLDO, REGULATOR_EVENT_FAIL);
	NOTIFY(BCM15602_ID_SDLDO, REGULATOR_EVENT_FAIL);

	dev_err(ddata->dev,
		"%s: waiting for chip to come out of reset\n", __func__);

	/* wait for chip to come out of reset, signaled by resetb interrupt */
	timeout = wait_for_completion_timeout(&ddata->reset_complete,
					      BCM15602_SHUTDOWN_RESET_TIMEOUT);
	if (!timeout)
		dev_err(ddata->dev,
			"%s: timeout waiting for device to return from reset\n",
			__func__);
	else
		bcm15602_chip_init(ddata);
}

/* irq handler for resetb pin */
static irqreturn_t bcm15602_resetb_irq_handler(int irq, void *cookie)
{
	struct bcm15602_chip *ddata = (struct bcm15602_chip *)cookie;

	if (gpio_get_value(ddata->pdata->resetb_gpio)) {
		dev_dbg(ddata->dev, "%s: completing reset\n", __func__);
		complete(&ddata->reset_complete);
	} else {
		dev_err(ddata->dev, "%s: unexpected device reset\n", __func__);
		schedule_work(&ddata->reset_work);
	}

	return IRQ_HANDLED;
}

/* irq handler for intb pin */
static irqreturn_t bcm15602_intb_irq_handler(int irq, void *cookie)
{
	struct bcm15602_chip *ddata = (struct bcm15602_chip *)cookie;
	int ret;

	dev_dbg(ddata->dev, "%s: observed irq\n", __func__);

	while (!gpio_get_value(ddata->pdata->intb_gpio)) {
		ret = bcm15602_check_int_flags(ddata);
		if (ret)
			return IRQ_RETVAL(ret);
	}

	return IRQ_HANDLED;
}

#define BCM15602_ASR_VOCTRL_VBASE 565000 /* microvolts */
#define BCM15602_ASR_VOCTRL_VSTEP 5000   /* microvolts */

/* set the current voltage of the regulator in microvolts */
static int bcm15602_regulator_set_voltage(struct regulator_dev *rdev,
					  int min_uV, int max_uV,
					  unsigned *selector)
{
	struct bcm15602_chip *ddata = rdev_get_drvdata(rdev);
	enum bcm15602_regulator_ids rid = rdev_get_id(rdev);
	int vsel, vout;
	int i;

	if (rid != BCM15602_ID_ASR)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(bcm15602_asr_vtbl); i++) {
		if ((min_uV <= bcm15602_asr_vtbl[i]) &&
		    (bcm15602_asr_vtbl[i] <= max_uV)) {
			*selector = i;
			vout = bcm15602_asr_vtbl[i];
			break;
		}
	}

	if (i == ARRAY_SIZE(bcm15602_asr_vtbl))
		return -EINVAL;

	vsel = (vout - BCM15602_ASR_VOCTRL_VBASE) / BCM15602_ASR_VOCTRL_VSTEP;

	dev_dbg(ddata->dev, "%s: rid %d, setting vsel to 0x%02x\n", __func__,
		rid, vsel);

	return bcm15602_write_byte(ddata, BCM15602_REG_BUCK_ASR_VOCTRL,
				   vsel & 0x7F);
}

/* get the current voltage of the regulator in microvolts */
static int bcm15602_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct bcm15602_chip *ddata = rdev_get_drvdata(rdev);
	enum bcm15602_regulator_ids rid = rdev_get_id(rdev);
	u8 reg_data;
	int vsel, vstep, vbase;
	int ret;

	switch (rid) {
	case BCM15602_ID_SDLDO:
		ret = bcm15602_read_byte(ddata, BCM15602_REG_LDO_SDLDO_VOCTRL,
					 &reg_data);
		if (ret)
			return ret;
		vbase = (reg_data & 0x80) ? 1725000 : 1380000;
		vstep = (reg_data & 0x80) ? 25000 : 10000;
		vsel = reg_data & 0x3F;
		break;
	case BCM15602_ID_IOLDO:
		ret = bcm15602_read_byte(ddata, BCM15602_REG_LDO_IOLDO_VOCTRL,
					 &reg_data);
		if (ret)
			return ret;
		vbase = (reg_data & 0x80) ? 1725000 : 1380000;
		vstep = (reg_data & 0x80) ? 25000 : 10000;
		vsel = reg_data & 0x3F;
		break;
	case BCM15602_ID_ASR:
		ret = bcm15602_read_byte(ddata, BCM15602_REG_BUCK_ASR_VOCTRL,
					 &reg_data);
		if (ret)
			return ret;
		vbase = 565000;
		vstep = 5000;
		vsel = reg_data & 0x7F;
		break;
	case BCM15602_ID_SDSR:
		ret = bcm15602_read_byte(ddata, BCM15602_REG_BUCK_SDSR_CTRL1,
					 &reg_data);
		if (ret)
			return ret;
		switch ((reg_data & 0xC0) >> 6) {
		case 0:
			vbase = 565000;
			vstep = 5000;
			break;
		case 1:
			vbase = 678000;
			vstep = 6000;
			break;
		case 2:
			vbase = 1130000;
			vstep = 10000;
			break;
		case 3:
			vbase = 1695000;
			vstep = 15000;
			break;
		default: // make compiler happy
			return -EINVAL;
		}
		ret = bcm15602_read_byte(ddata, BCM15602_REG_BUCK_SDSR_VOCTRL,
					 &reg_data);
		if (ret)
			return ret;
		vsel = reg_data & 0x7F;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(ddata->dev, "%s: rid %d, returning voltage %d\n", __func__, rid,
		vbase + vsel * vstep);

	return vbase + vsel * vstep;
}

/* enable the regulator */
static int bcm15602_regulator_enable(struct regulator_dev *rdev)
{
	struct bcm15602_chip *ddata = rdev_get_drvdata(rdev);
	enum bcm15602_regulator_ids rid = rdev_get_id(rdev);
	int ret;

	switch (rid) {
	case BCM15602_ID_SDLDO:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_LDO_SDLDO_ENCTRL,
					  0x3);
		break;
	case BCM15602_ID_IOLDO:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_LDO_IOLDO_ENCTRL,
					  0x3);
		break;
	case BCM15602_ID_ASR:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_BUCK_ASR_CTRL0,
					  0xC5);
		break;
	case BCM15602_ID_SDSR:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_BUCK_SDSR_CTRL0,
					  0xDB);
		break;
	default:
		return -EINVAL;
	}

	if (!ret) {
		set_bit(rid, &ddata->reg_enabled_mask);
		bcm15602_enable_wdt(ddata);
	}

	return ret;
}

/* disable the regulator */
static int bcm15602_regulator_disable(struct regulator_dev *rdev)
{
	struct bcm15602_chip *ddata = rdev_get_drvdata(rdev);
	enum bcm15602_regulator_ids rid = rdev_get_id(rdev);
	int ret;

	switch (rid) {
	case BCM15602_ID_SDLDO:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_LDO_SDLDO_ENCTRL,
					  0x2);
		break;
	case BCM15602_ID_IOLDO:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_LDO_IOLDO_ENCTRL,
					  0x2);
		break;
	case BCM15602_ID_ASR:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_BUCK_ASR_CTRL0,
					  0xC4);
		break;
	case BCM15602_ID_SDSR:
		ret = bcm15602_write_byte(ddata, BCM15602_REG_BUCK_SDSR_CTRL0,
					  0xDA);
		break;
	default:
		return -EINVAL;
	}

	if (!ret) {
		clear_bit(rid, &ddata->reg_enabled_mask);
		if (!ddata->reg_enabled_mask)
			bcm15602_disable_wdt(ddata);
	}

	return ret;
}

/* get regulator enable status */
static int bcm15602_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct bcm15602_chip *ddata = rdev_get_drvdata(rdev);
	enum bcm15602_regulator_ids rid = rdev_get_id(rdev);

	if ((rid >= 0) && (rid < BCM15602_NUM_REGULATORS))
		return !!(ddata->reg_enabled_mask & (1 << rid));
	else
		return -EINVAL;
}

/* get platform data from the device tree */
static struct bcm15602_platform_data *bcm15602_get_platform_data_from_dt
	(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct bcm15602_platform_data *pdata;

	if (!np)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->pon_gpio = of_get_named_gpio(np, "bcm,pon-gpio", 0);
	pdata->resetb_gpio = of_get_named_gpio(np, "bcm,resetb-gpio", 0);
	pdata->intb_gpio = of_get_named_gpio(np, "bcm,intb-gpio", 0);
	pdata->resetb_irq = gpio_to_irq(pdata->resetb_gpio);
	pdata->intb_irq = gpio_to_irq(pdata->intb_gpio);

	return pdata;
}

/* register a regulator with the kernel regulator framework */
static int bcm15602_regulator_register(struct bcm15602_chip *ddata)
{
	struct device *dev = ddata->dev;
	struct regulator_config cfg = {
		.dev = dev,
		.driver_data = ddata,
		.regmap = ddata->regmap
	};
	struct regulator_dev *rdev;
	int i;

	for (i = 0; i < BCM15602_NUM_REGULATORS; i++) {
		cfg.init_data = &bcm15602_regulator_init_data[i];
		rdev = devm_regulator_register(dev,
					       &bcm15602_regulator_desc[i],
					       &cfg);
		if (IS_ERR(rdev)) {
			dev_err(dev,
				"%s: failed to register regulator %d\n",
				__func__, i);
			return PTR_ERR(rdev);
		}

		*(ddata->rdevs + i) = rdev;
	}

	return 0;
}

/* configure the adc and housekeeping slots */
static void bcm15602_config_adc(struct bcm15602_chip *ddata)
{
	u32 slot_mask = 0;
	int i;

	for (i = 0; i < BCM15602_HK_NUM_SLOTS; i++) {
		bcm15602_write_byte(ddata,
				    BCM15602_REG_ADC_SLOTCTRL0 + i,
				    adc_ctrls[i].ctrl);
		bcm15602_write_byte(ddata,
				    BCM15602_REG_ADC_SLOTTHL0 + (i * 2),
				    adc_ctrls[i].th_ctrl & 0xFF);
		bcm15602_write_byte(ddata,
				    BCM15602_REG_ADC_SLOTTHL0 + (i * 2) + 1,
				    (adc_ctrls[i].th_ctrl >> 8) & 0xFF);

		if (!adc_ctrls[i].th_en)
			slot_mask |= 1 << i;
	};

	for (i = 0; i < ((BCM15602_HK_NUM_SLOTS + 7) / 8); i++) {
		bcm15602_write_byte(ddata,
				    BCM15602_REG_ADC_SLOTMSKL + i,
				    (slot_mask >> (i * 8)) & 0xFF);
	}

	/* update hk every 10 ms */
	bcm15602_write_byte(ddata, BCM15602_REG_ADC_HKCTRL, 0x8);
}

/* enable all of the interrupts */
static void bcm15602_config_ints(struct bcm15602_chip *ddata)
{
	/* clear any pending interrupts */
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTFLAG1, 0x1F);
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTFLAG2, 0x1B);
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTFLAG3, 0x03);
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTFLAG4, 0x1F);

	/* enable interrupts we care about */
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTEN1, 0x1F);
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTEN2, 0x1B);
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTEN3, 0x02);
	bcm15602_write_byte(ddata, BCM15602_REG_INT_INTEN4, 0x1F);
}

/* enable the watchdog timer and reset the count */
static void bcm15602_enable_wdt(struct bcm15602_chip *ddata)
{
	if (!ddata->wdt_enabled) {
		bcm15602_write_byte(ddata, BCM15602_REG_WDT_WDTCTRL1, 0x01);
		bcm15602_write_byte(ddata, BCM15602_REG_WDT_WDTCTRL2, 0x1E);
		ddata->wdt_enabled = true;
	}
}

/* disable the watchdog timer */
static void bcm15602_disable_wdt(struct bcm15602_chip *ddata)
{
	if (ddata->wdt_enabled) {
		bcm15602_write_byte(ddata, BCM15602_REG_WDT_WDTCTRL1, 0x00);
		ddata->wdt_enabled = false;
	}
}

/* some changes to default configuration based on bringup */
static int bcm15602_chip_fixup(struct bcm15602_chip *ddata)
{
	dev_dbg(ddata->dev, "%s: rev %d\n", __func__, ddata->rev_id);

	if (ddata->rev_id < BCM15602_REV_A1) {
		/* enable bandgap curvature correction for improved accuracy */
		bcm15602_update_bits(ddata, BCM15602_REG_ADC_BGCTRL, 0x40,
				     0x40);

		/* unlock register, then set ASR switching frequency trim */
		bcm15602_write_byte(ddata, BCM15602_REG_SYS_WRLOCKEY, 0x38);
		bcm15602_write_byte(ddata, BCM15602_REG_BUCK_ASR_CTRL2, 0x04);

		/* set ASR undervoltage threshold */
		bcm15602_update_bits(ddata, BCM15602_REG_BUCK_ASR_CTRL5, 0x0C,
				     0x04);

		/* set ASR comparator input filter select */
		bcm15602_update_bits(ddata, BCM15602_REG_BUCK_ASR_CTRL11, 0x03,
				     0x03);

		/* set ASR feedback network R2 adjustment */
		bcm15602_update_bits(ddata, BCM15602_REG_BUCK_ASR_TSET_CTRL2,
				     0x03, 0x02);

		/* set ASR rail to 0.9V */
		bcm15602_write_byte(ddata, BCM15602_REG_BUCK_ASR_VOCTRL, 0x43);

		/* set ASR to single phase */
		bcm15602_update_bits(ddata, BCM15602_REG_BUCK_ASR_TSET_CTRL2,
				     0x10, 0x00);
	} else if (ddata->rev_id == BCM15602_REV_A1) {
		/* enable bandgap curvature correction for improved accuracy */
		bcm15602_update_bits(ddata, BCM15602_REG_ADC_BGCTRL, 0x40,
				     0x40);
	}

	return 0;
}

/* initialize the chip */
static int bcm15602_chip_init(struct bcm15602_chip *ddata)
{
	bcm15602_print_id(ddata);
	bcm15602_print_psm_event(ddata);

	bcm15602_chip_fixup(ddata);

	bcm15602_config_adc(ddata);
	bcm15602_config_ints(ddata);

	return 0;
}

static int bcm15602_probe(struct i2c_client *client,
			  const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	struct bcm15602_chip *ddata;
	struct bcm15602_platform_data *pdata;
	unsigned long timeout;
	int ret;
	int i;

	/* allocate memory for chip structure */
	ddata = devm_kzalloc(dev, sizeof(struct bcm15602_chip),
			     GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	/* set client data */
	i2c_set_clientdata(client, ddata);

	/* get platform data */
	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = bcm15602_get_platform_data_from_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	/* initialize chip structure */
	ddata->dev = dev;
	ddata->pdata = pdata;
	dev->platform_data = pdata;

	/* initialize some structures */
	INIT_WORK(&ddata->reset_work, bcm15602_reset_work);

	/* initialize completions */
	init_completion(&ddata->reset_complete);
	init_completion(&ddata->adc_conv_complete);

	/* initialize regmap */
	ddata->regmap = devm_regmap_init_i2c(client, &bcm15602_regmap_config);
	if (IS_ERR(ddata->regmap)) {
		ret = PTR_ERR(ddata->regmap);
		dev_err(dev,
			"%s: could not initialize regmap (%d)\n",
			__func__, ret);
		return -ENOMEM;
	}

	/* create sysfs attributes */
	bcm15602_config_sysfs(dev);

	/* request GPIOs and IRQs */
	devm_gpio_request_one(dev, pdata->pon_gpio, GPIOF_OUT_INIT_LOW,
			      "BCM15602 PON");
	devm_gpio_request_one(dev, pdata->resetb_gpio, GPIOF_IN,
			      "BCM15602 RESETB");
	devm_gpio_request_one(dev, pdata->intb_gpio, GPIOF_IN,
			      "BCM15602 INTB");
	ret = devm_request_threaded_irq(dev, pdata->resetb_irq, NULL,
					bcm15602_resetb_irq_handler,
					IRQF_TRIGGER_FALLING |
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"bcm15602-resetb", ddata);
	ret = devm_request_threaded_irq(dev, pdata->intb_irq, NULL,
					bcm15602_intb_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"bcm15602-intb", ddata);

	/* disable the irq while doing initial power on */
	disable_irq(pdata->resetb_irq);

	for (i = 0; i < BCM15602_PON_RETRY_CNT; i++) {
		dev_dbg(dev, "%s: powering on bcm15602\n", __func__);
		gpio_set_value_cansleep(pdata->pon_gpio, 1);

		/* give the chip some time to power on */
		msleep(BCM15602_PON_RESET_DELAY);

		/* poll on the gpio until it goes high or until we timeout */
		timeout = jiffies + BCM15602_PON_RESET_TIMEOUT;
		while (!gpio_get_value(pdata->resetb_gpio) &&
		       time_before(jiffies, timeout)) {
			usleep_range(100, 105);
		}

		if (gpio_get_value(pdata->resetb_gpio))
			break;

		dev_err(dev, "%s: powering on timed out, try (%d/%d)\n",
			__func__, i + 1, BCM15602_PON_RETRY_CNT);

		gpio_set_value_cansleep(pdata->pon_gpio, 0);
		usleep_range(100, 105);
	}

	/* check for failure */
	if (i == BCM15602_PON_RETRY_CNT) {
		ret = -ETIMEDOUT;
		dev_err(dev, "%s: powering on failed\n", __func__);
		goto error_reset;
	}

	/* enable the irq after power on */
	enable_irq(pdata->resetb_irq);

	/* initialize chip */
	bcm15602_chip_init(ddata);

	/* initialize and register device regulators */
	ddata->rdevs =
		devm_kzalloc(dev,
			     BCM15602_NUM_REGULATORS *
			     sizeof(struct regulator_dev *),
			     GFP_KERNEL);
	if (!ddata->rdevs) {
		dev_err(dev,
			"%s: could not initialize rdevs array\n",
			__func__);
		return -ENOMEM;
	}
	bcm15602_regulator_register(ddata);

	return mfd_add_devices(dev, -1, bcm15602_devs,
			       ARRAY_SIZE(bcm15602_devs),
			       NULL, 0, NULL);

error_reset:
	return ret;
}

#ifdef CONFIG_PM
static int bcm15602_suspend(struct device *dev)
{
	struct bcm15602_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		enable_irq_wake(pdata->intb_irq);
	return 0;
}

static int bcm15602_resume(struct device *dev)
{
	struct bcm15602_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		disable_irq_wake(pdata->intb_irq);
	return 0;
}

static const struct dev_pm_ops bcm15602_dev_pm_ops = {
	.suspend = bcm15602_suspend,
	.resume  = bcm15602_resume,
};
#endif

static const struct of_device_id bcm15602_dt_ids[] = {
	{ .compatible = "brcm,bcm15602", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm15602_dt_ids);

static const struct i2c_device_id bcm15602_id_table[] = {
	{ .name = DRIVER_NAME, .driver_data = 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bcm15602_id_table);

static struct i2c_driver bcm15602_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &bcm15602_dev_pm_ops,
#endif
	},
	.probe = bcm15602_probe,
	.id_table = bcm15602_id_table,
};

module_i2c_driver(bcm15602_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("BCM15602 Regulator Device Driver");
MODULE_LICENSE("GPL");

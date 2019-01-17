/*
 * Copyright (C) 2017 Google, Inc.
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

#include "s2mpb04-core.h"

#define DRIVER_NAME "s2mpb04"

/* defines the number of tries to repeat an I2C transaction */
#define S2MPB04_I2C_RETRY_COUNT 10

/* defines the delay in ms for reset completion */
#define S2MPB04_PON_RESET_DELAY 2

/* defines the timeout in jiffies for reset completion */
#define S2MPB04_PON_RESET_TIMEOUT msecs_to_jiffies(15)

/* defines the number of retries for powering on */
#define S2MPB04_PON_RETRY_CNT 4

/* defines the timeout in jiffies for reset completion after shutdown */
#define S2MPB04_SHUTDOWN_RESET_TIMEOUT msecs_to_jiffies(10000)

/* defines the timeout in jiffies for ADC conversion completion */
#define S2MPB04_ADC_CONV_TIMEOUT  msecs_to_jiffies(100)

/* defines the number of tries to recover chip */
#define S2MPB04_RECOVER_RETRY_COUNT 3

static int s2mpb04_pdn_seq_en(struct s2mpb04_core *ddata);
static int s2mpb04_pdn_seq_dis(struct s2mpb04_core *ddata);
static int s2mpb04_prepare_pon(struct s2mpb04_core *ddata);
static int s2mpb04_chip_init(struct s2mpb04_core *ddata);
static int s2mpb04_core_fixup(struct s2mpb04_core *ddata);
static void s2mpb04_print_status(struct s2mpb04_core *ddata);

static const struct mfd_cell s2mpb04_devs[] = {
	{
		.name = "s2mpb04-regulator",
		.of_compatible = "samsung,s2mpb04-regulator",
	},
	{
		.name = "s2mpb04-gpio",
		.of_compatible = "samsung,s2mpb04-gpio",
	},
	/*
	 * Disable s2mpb04 thermal zone to reduce overall i2c xfer.
	 * See b/122984225.
	 */
#if 0
	{
		.name = "s2mpb04-thermal",
		.of_compatible = "samsung,s2mpb04-thermal",
	},
#endif
};

static const struct regmap_config s2mpb04_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

int s2mpb04_toggle_pon(struct s2mpb04_core *ddata)
{
	unsigned long timeout;

	dev_info(ddata->dev, "%s: toggling PON\n", __func__);

	reinit_completion(&ddata->init_complete);

	/* disable hardware sequence when SW is initiating power-down.
	 * hw-driven power-down sequence is required only in case of a
	 * TSD event and has 4ms extra latency.
	 */
	s2mpb04_pdn_seq_dis(ddata);

	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 0);

	/* force state machine in low power state */
	s2mpb04_prepare_pon(ddata);

	usleep_range(20, 25);
	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 1);

	/* wait for chip to come out of reset, signaled by resetb interrupt */
	timeout = wait_for_completion_timeout(&ddata->init_complete,
					      S2MPB04_SHUTDOWN_RESET_TIMEOUT);
	if (!timeout)
		dev_err(ddata->dev,
			"%s: timeout waiting for device to return from reset\n",
			__func__);

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpb04_toggle_pon);

/* When chip is hung resetb is stuck high. Function to toggle PON to recover
 * the chip but don't wait for resetb transition
 */
int s2mpb04_toggle_pon_no_resetb_wait(struct s2mpb04_core *ddata)
{
	dev_dbg(ddata->dev, "%s: toggling PON\n", __func__);

	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 0);

	/* force state machine in low power state */
	s2mpb04_prepare_pon(ddata);

	usleep_range(20, 25);
	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 1);

	/* give the chip some time to power on */
	msleep(S2MPB04_PON_RESET_DELAY);

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpb04_toggle_pon_no_resetb_wait);

int s2mpb04_dump_regs(struct s2mpb04_core *ddata)
{
	u8 reg_data;
	int i;

	for (i = 0; i <= S2MPB04_REG_SEQ2; i++) {
		s2mpb04_read_byte(ddata, i, &reg_data);
		dev_info(ddata->dev, "[0x%02x] = 0x%02x\n", i, reg_data);
	}
	for (i = S2MPB04_REG_ADC_CTRL; i <= S2MPB04_REG_ADC_DBNC_CTRL2; i++) {
		s2mpb04_read_byte(ddata, i, &reg_data);
		dev_info(ddata->dev, "[0x%02x] = 0x%02x\n", i, reg_data);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpb04_dump_regs);

int s2mpb04_read_byte(struct s2mpb04_core *ddata, u8 addr, u8 *data)
{
	int ret;
	unsigned int val;
	int retry_cnt = 0;

	do {
		ret = regmap_read(ddata->regmap, addr, &val);
		if (!ret) {
			*data = (u8)val;
			return 0;
		}

		dev_err(ddata->dev,
			"failed to read addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPB04_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPB04_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpb04_read_byte);

int s2mpb04_read_bytes(struct s2mpb04_core *ddata, u8 addr, u8 *data,
		       size_t count)
{
	int ret;
	int retry_cnt = 0;

	do {
		ret = regmap_bulk_read(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev, "%s: failed to read %zd bytes from addr 0x%.2x (%d), retry %d\n",
			__func__, count, addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPB04_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPB04_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpb04_read_bytes);

int s2mpb04_write_byte(struct s2mpb04_core *ddata, u8 addr, u8 data)
{
	int ret;
	int retry_cnt = 0;

	do {
		ret = regmap_write(ddata->regmap, addr, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPB04_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPB04_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpb04_write_byte);

int s2mpb04_write_bytes(struct s2mpb04_core *ddata, u8 addr, u8 *data,
			size_t count)
{
	int ret;
	int retry_cnt = 0;

	do {
		ret = regmap_bulk_write(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write %zd bytes to addr 0x%.2x (%d), retry %d\n",
			count, addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPB04_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPB04_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpb04_write_bytes);

int s2mpb04_update_bits(struct s2mpb04_core *ddata, u8 addr,
			unsigned int mask, u8 data)
{
	int ret;
	int retry_cnt = 0;

	do {
		ret = regmap_update_bits(ddata->regmap, addr, mask, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to update addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPB04_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPB04_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpb04_update_bits);

int s2mpb04_read_adc_chan(struct s2mpb04_core *ddata,
			  int chan_num, u8 *chan_data)
{
	int ret = 0;
	unsigned long timeout;

	/* serialize manual adc conversions */
	if (test_and_set_bit(0, &ddata->adc_conv_busy))
		return -EAGAIN;

	/* clear adc conversion completion before starting conversion */
	reinit_completion(&ddata->adc_conv_complete);

	/* write the channel number */
	ret = s2mpb04_write_byte(ddata, S2MPB04_REG_MUXSEL1, chan_num);
	if (ret)
		goto adc_cleanup;

	/* enable the clock at 125 kHz, 1 channel, and 8 samples */
	ret = s2mpb04_write_byte(ddata, S2MPB04_REG_ADC_CTRL, 0xC2);
	if (ret)
		goto adc_cleanup;

	dev_dbg(ddata->dev, "%s: started ADC conversion\n", __func__);

	/* wait for completion signaled by interrupt */
	timeout = wait_for_completion_timeout(&ddata->adc_conv_complete,
					      S2MPB04_ADC_CONV_TIMEOUT);
	if (!timeout) {
		ret = -ETIMEDOUT;
		dev_err(ddata->dev, "%s: ADC conversion timeout\n", __func__);
		goto adc_cleanup;
	}

	/* read and format the conversion result */
	ret = s2mpb04_read_byte(ddata, S2MPB04_REG_ADC0DATA, chan_data);
	if (ret)
		goto adc_cleanup;

	dev_dbg(ddata->dev, "%s: chan_data 0x%02x\n", __func__, *chan_data);

adc_cleanup:

	/* set ADC muxsel to 0 (no monitoring). This is a workaround to prevent
	 * disabling ADC when it's connected to temperature sensor */
	s2mpb04_write_byte(ddata, S2MPB04_REG_MUXSEL1, 0x0);

	/*
	 * Disable thermal shutdown when disabling the ADC. This is a workaround
	 * for a silicon bug that causes thermal shutdown comparator input to be
	 * shorted to ground for a short duration when the ADC is enabled.
	 */
	s2mpb04_write_byte(ddata, S2MPB04_REG_TSD_CTRL, 0x20);

	/* disable the ADC clock */
	s2mpb04_write_byte(ddata, S2MPB04_REG_ADC_CTRL, 0x42);

	/* Re-enable thermal shutdown */
	s2mpb04_write_byte(ddata, S2MPB04_REG_TSD_CTRL, 0xA0);

	/* release adc conversion */
	clear_bit(0, &ddata->adc_conv_busy);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpb04_read_adc_chan);

#define NOTIFY(id, event) s2mpb04_regulator_notify(id, event)

/* print the device id */
static void s2mpb04_print_id(struct s2mpb04_core *ddata)
{
	struct device *dev = ddata->dev;
	u8 reg_data;
	int ret;

	ret = s2mpb04_read_byte(ddata, S2MPB04_REG_CHIP_ID, &reg_data);
	if (!ret) {
		dev_info(dev, "ID: 0x%02X\n", reg_data);
		ddata->rev_id = reg_data;
	} else {
		dev_err(dev, "%s: Could not read PMIC ID (%d)\n", __func__,
			ret);
	}
}

/* print the status register */
static void s2mpb04_print_status(struct s2mpb04_core *ddata)
{
	struct device *dev = ddata->dev;
	u8 flags[3], status[2];
	int ret;

	ret = s2mpb04_read_bytes(ddata, S2MPB04_REG_INT1, flags, 3);
	if (!ret)
		dev_info(dev, "%s: Flags: 0x%02x, 0x%02x, 0x%02x\n",
			 __func__, flags[0], flags[1], flags[2]);

	ret = s2mpb04_read_bytes(ddata, S2MPB04_REG_STATUS1, status, 2);
	if (!ret)
		dev_info(dev, "%s: Status: 0x%02x, 0x%02x\n", __func__,
			 status[0], status[1]);
}

/* handle an interrupt flag */
static int s2mpb04_handle_int(struct s2mpb04_core *ddata,
			      unsigned int flag_num)
{
	struct device *dev = ddata->dev;

	dev_dbg(dev, "%s: flag %d\n", __func__, flag_num);

	switch (flag_num) {
	case S2MPB04_INT_PONR:
		dev_dbg(dev, "%s: Observed PON rising edge\n", __func__);
		break;

	case S2MPB04_INT_PONF:
		dev_dbg(dev, "%s: Observed PON falling edge\n", __func__);
		break;

	case S2MPB04_INT_ADC_DONE:
		dev_dbg(dev, "%s: completing adc conversion\n", __func__);
		complete(&ddata->adc_conv_complete);
		break;

	case S2MPB04_INT_SMPS1_OI:
		dev_err(dev, "Detected SMPS1 over-current event\n");
		NOTIFY(S2MPB04_ID_SMPS1,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;

	case S2MPB04_INT_SMPS2_OI:
		dev_err(dev, "Detected SMPS2 over-current event\n");
		NOTIFY(S2MPB04_ID_SMPS2,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;

	case S2MPB04_INT_SMPS1_UV:
		dev_err(dev, "Detected SMPS1 under-voltage event\n");
		NOTIFY(S2MPB04_ID_SMPS1,
		       REGULATOR_EVENT_UNDER_VOLTAGE | REGULATOR_EVENT_FAIL);
		break;

	case S2MPB04_INT_LDO1_OI:
		dev_err(dev, "Detected LDO1 over-current event\n");
		NOTIFY(S2MPB04_ID_LDO1,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;

	case S2MPB04_INT_LDO2_OI:
		dev_err(dev, "Detected LDO2 over-current event\n");
		NOTIFY(S2MPB04_ID_LDO2,
		       REGULATOR_EVENT_OVER_CURRENT | REGULATOR_EVENT_FAIL);
		break;

	case S2MPB04_INT_TH_TINT:
		dev_err(dev, "%s: unhandled thermal warm interrupt\n",
			__func__);
		break;

	case S2MPB04_INT_TH_TRIPF:
		dev_err(dev, "%s: therm_trip has returned to normal\n",
			__func__);
		break;

	case S2MPB04_INT_WATCHDOG:
	case S2MPB04_INT_TH_TRIPR:
	case S2MPB04_INT_TSD:
		if (flag_num == S2MPB04_INT_WATCHDOG)
			dev_err(dev, "%s: Watchdog timer expired\n", __func__);
		else if (flag_num == S2MPB04_INT_TH_TRIPR)
			dev_err(dev, "%s: therm_trip asserted\n", __func__);
		else
			dev_err(dev, "%s: thermal shutdown\n", __func__);

		/* notify regulator clients of failures */
		NOTIFY(S2MPB04_ID_SMPS1, REGULATOR_EVENT_FAIL);
		NOTIFY(S2MPB04_ID_SMPS2, REGULATOR_EVENT_FAIL);
		NOTIFY(S2MPB04_ID_LDO2, REGULATOR_EVENT_FAIL);
		NOTIFY(S2MPB04_ID_LDO1, REGULATOR_EVENT_FAIL);
		break;

	default:
		dev_dbg(dev, "%s: Reserved flag %d\n", __func__, flag_num);
		break;
	}

	return 0;
}

/* find pending interrupt flags */
static int s2mpb04_check_int_flags(struct s2mpb04_core *ddata)
{
	u8 flags[3], flag_mask;
	unsigned int first_bit, flag_num;
	int ret = 0;
	int i;

	/* read interrupt status flags */
	ret = s2mpb04_read_bytes(ddata, S2MPB04_REG_INT1, flags, 3);
	if (ret)
		return ret;

	dev_dbg(ddata->dev,
		"%s: [0] = 0x%02x, [1] = 0x%02x, [2] = 0x%02x\n",
		__func__, flags[0], flags[1], flags[2]);

	/* iterate through each interrupt */
	for (i = 0; i < 3; i++) {
		while (flags[i]) {
			/* find first set interrupt flag */
			first_bit = ffs(flags[i]);
			flag_mask = 1 << (first_bit - 1);
			flag_num = (i * 8) + (first_bit - 1);

			/* handle interrupt */
			ret = s2mpb04_handle_int(ddata, flag_num);

			flags[i] &= ~flag_mask;
		}
	}

	return ret;
}

/* kernel thread to notify regulator fail event and clear any pending
 * interrupt status
 */
static void s2mpb04_reset_work(struct work_struct *data)
{
	struct s2mpb04_core *ddata = container_of(data, struct s2mpb04_core,
						   reset_work);

	/* notify regulators of shutdown event */
	dev_err(ddata->dev,
		"%s: Notifying regulators of shutdown event\n", __func__);
	NOTIFY(S2MPB04_ID_SMPS1, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPB04_ID_SMPS2, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPB04_ID_LDO1, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPB04_ID_LDO2, REGULATOR_EVENT_FAIL);
}

/* kernel thread to detect if PMIC is hung and run the recovery sequence */
static void s2mpb04_recover_work(struct work_struct *data)
{
	struct s2mpb04_core *ddata = container_of(data, struct s2mpb04_core,
						  recover_work);
	struct s2mpb04_platform_data *pdata = ddata->pdata;
	struct device *dev = ddata->dev;
	unsigned long timeout;

	/* disable interrupts during recovery sequence */
	disable_irq(pdata->resetb_irq);
	disable_irq(pdata->intb_irq);

	/* when chip is hung resetb is stuck high. Toggle PON to recover chip
	 * but don't wait for resetb transition
	 */
	s2mpb04_toggle_pon_no_resetb_wait(ddata);

	/* check if recovery sequence worked. resetb should now become low
	 * when PON is low
	 */
	gpio_set_value_cansleep(pdata->pon_gpio, 0);
	usleep_range(100, 105);

	/* re-enable interrupts */
	enable_irq(pdata->resetb_irq);
	enable_irq(pdata->intb_irq);

	if (gpio_get_value(pdata->resetb_gpio)) {
		dev_err(dev, "%s: recovery sequence failed", __func__);
		return;
	}

	dev_info(dev, "%s: recovery sequence success", __func__);

	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 1);

	/* wait for chip to come out of reset and get initialized,
	 * signaled by resetb interrupt handler
	 */
	timeout = wait_for_completion_timeout(&ddata->init_complete,
					      S2MPB04_SHUTDOWN_RESET_TIMEOUT);
	if (!timeout)
		dev_err(ddata->dev,
			"%s: timeout waiting for device to return from reset\n",
			__func__);
}

/* check if chip is hung */
static bool s2mpb04_is_chip_hung(struct s2mpb04_core *ddata)
{
	int ret;
	u8 chan_data;

	/* Detect if chip is hung by reading ADC */
	do {
		ret = s2mpb04_read_adc_chan(ddata,
					    S2MPB04_ADC_VBAT,
					    &chan_data);
	} while (ret == -EAGAIN);

	/* chip is hung if ADC read timesout or raw vbat data
	 * is 0x10
	 */
	return ((ret == -ETIMEDOUT) || (chan_data == 0x10));
}

/* irq handler for resetb pin.
 * It may not catch falling edge. But, that will not be an issue as intb
 * interrupt handler will notify regulator fail event.
 */
static irqreturn_t s2mpb04_resetb_irq_handler(int irq, void *cookie)
{
	struct s2mpb04_core *ddata = (struct s2mpb04_core *)cookie;
	struct device *dev = ddata->dev;
	bool is_chip_hung;

	if (gpio_get_value(ddata->pdata->resetb_gpio)) {
		dev_dbg(dev, "%s: completing reset\n", __func__);

		/* initialize chip */
		s2mpb04_chip_init(ddata);

		/* check if chip is hung */
		is_chip_hung = s2mpb04_is_chip_hung(ddata);

		if (!is_chip_hung) {
			/* Fixup some chip settings that are different than
			 * reset values.
			 */
			s2mpb04_core_fixup(ddata);

			complete(&ddata->init_complete);

			/* clear recovery attempt counter */
			ddata->recover_count = 0;
		} else if (ddata->recover_count < S2MPB04_RECOVER_RETRY_COUNT) {
			dev_err(dev,
				"%s: lockup detected, trying recovery: (%d/%d)",
				__func__,
				(int)++ddata->recover_count,
				S2MPB04_RECOVER_RETRY_COUNT);
			schedule_work(&ddata->recover_work);
		} else {
			dev_err(dev, "%s: max recovery attempts reached: %d",
				__func__, S2MPB04_RECOVER_RETRY_COUNT);
		}
	} else {
		dev_err(dev, "%s: device reset\n", __func__);
		reinit_completion(&ddata->init_complete);
		schedule_work(&ddata->reset_work);
	}

	return IRQ_HANDLED;
}

/* irq handler for intb pin */
static irqreturn_t s2mpb04_intb_irq_handler(int irq, void *cookie)
{
	struct s2mpb04_core *ddata = (struct s2mpb04_core *)cookie;
	int ret;

	dev_dbg(ddata->dev, "%s: observed irq\n", __func__);

	while (!gpio_get_value(ddata->pdata->intb_gpio)) {
		ret = s2mpb04_check_int_flags(ddata);
		if (ret)
			return IRQ_RETVAL(ret);
	}

	return IRQ_HANDLED;
}

/* get platform data from the device tree */
static struct s2mpb04_platform_data *s2mpb04_get_platform_data_from_dt
	(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct s2mpb04_platform_data *pdata;

	if (!np)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->pon_gpio = of_get_named_gpio(np, "samsung,pon-gpio", 0);
	pdata->resetb_gpio = of_get_named_gpio(np, "samsung,resetb-gpio", 0);
	pdata->intb_gpio = of_get_named_gpio(np, "samsung,intb-gpio", 0);
	pdata->resetb_irq = gpio_to_irq(pdata->resetb_gpio);
	pdata->intb_irq = gpio_to_irq(pdata->intb_gpio);

	return pdata;
}

/* enable all of the interrupts */
static void s2mpb04_config_ints(struct s2mpb04_core *ddata)
{
	u8 bytes[3];

	/* clear any pending interrupts */
	s2mpb04_read_bytes(ddata, S2MPB04_REG_INT1, bytes, 3);

	/* unmask all (non-reserved) interrupts */
	s2mpb04_write_byte(ddata, S2MPB04_REG_INT1M, 0x00);
	s2mpb04_write_byte(ddata, S2MPB04_REG_INT2M, 0x00);
	s2mpb04_write_byte(ddata, S2MPB04_REG_INT3M, 0x7F);
}

/* some changes to default configuration based on bringup */
static int s2mpb04_core_fixup(struct s2mpb04_core *ddata)
{
	dev_dbg(ddata->dev, "%s: rev %d\n", __func__, ddata->rev_id);

	s2mpb04_config_ints(ddata);

	/* set SMPS1 output voltage to 0.9125V */
	s2mpb04_write_byte(ddata, S2MPB04_REG_BUCK1_OUT, 0x62);

	/* set SMPS2 output voltage to 1.15V */
	s2mpb04_write_byte(ddata, S2MPB04_REG_BUCK2_OUT, 0x88);

	/* set LDO1 output voltage to 1.87V */
	/* also modify s2mpb04_regulator_enable to match LDO1 fixup */
	s2mpb04_write_byte(ddata, S2MPB04_REG_LDO1_CTRL, 0x2F);

	/* set LDO2 output voltage to 1.87V */
	/* also modify s2mpb04_regulator_enable to match LDO2 fixup */
	s2mpb04_write_byte(ddata, S2MPB04_REG_LDO2_CTRL, 0x2F);

	/* disable watchdog timer */
	s2mpb04_write_byte(ddata, S2MPB04_REG_WTSR_CTRL, 0x1D);

	/* Disable temprature compensation. This is a workaround to prevent
	 * disabling ADC when it's connected to temperature sensor */
	s2mpb04_write_byte(ddata, S2MPB04_REG_ADC_CTRL2, 0x5);

	/* enable hardware power-down sequence */
	s2mpb04_pdn_seq_en(ddata);

	return 0;
}

/* Enable hardware power down sequence.
 * PMIC state machine locks up when BGROK drops low in Standby state.
 * When hardware power down sequence is enabled, on TSD event,
 * PMIC state machine will first go to power-down state before
 * going to low-power state. This will make BGROK to drop low
 * on power-down -> low-power state transition and will prevent
 * the lockup.
 */
static int s2mpb04_pdn_seq_en(struct s2mpb04_core *ddata)
{
	dev_dbg(ddata->dev, "%s: enable power down sequence\n",
		__func__);

	s2mpb04_write_byte(ddata, S2MPB04_REG_OFF_SEQ_CTRL, 0x80);
	s2mpb04_write_byte(ddata, S2MPB04_REG_SEQ1, 0x44);
	s2mpb04_write_byte(ddata, S2MPB04_REG_SEQ2, 0x44);

	return 0;
}

/* Disable hardware power down sequence */
static int s2mpb04_pdn_seq_dis(struct s2mpb04_core *ddata)
{
	dev_dbg(ddata->dev, "%s: disable power down sequence\n",
		__func__);

	s2mpb04_write_byte(ddata, S2MPB04_REG_OFF_SEQ_CTRL, 0x0);

	return 0;
}

/* Force state machine in low power state. This should be state machine's
 * state when PON=0 and BUV (battery under volatge)=0. But, when LDO OI
 * issue happens, it's stuck in power-down state.
 *
 * In low power state, PON 0 -> 1 toggle will initiate power-up sequence
 * and reload register values from OTP. So, this function should be called
 * when PON is set to 0.
 */
static int s2mpb04_prepare_pon(struct s2mpb04_core *ddata)
{
	dev_info(ddata->dev, "%s: applying workaround for LDO OI issue\n",
		 __func__);

	s2mpb04_write_byte(ddata, S2MPB04_REG_SEQ1, 0x66);
	s2mpb04_write_byte(ddata, S2MPB04_REG_SEQ2, 0x66);

	return 0;
}

/* initialize the chip */
static int s2mpb04_chip_init(struct s2mpb04_core *ddata)
{
	s2mpb04_print_id(ddata);
	s2mpb04_print_status(ddata);

	return 0;
}

static int s2mpb04_probe(struct i2c_client *client,
			 const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	struct s2mpb04_core *ddata;
	struct s2mpb04_platform_data *pdata;
	unsigned long timeout;
	int ret;
	int i;

	/* allocate memory for chip structure */
	ddata = devm_kzalloc(dev, sizeof(struct s2mpb04_core),
			     GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	/* set client data */
	i2c_set_clientdata(client, ddata);
	dev_set_drvdata(dev, ddata);

	/* get platform data */
	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = s2mpb04_get_platform_data_from_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	/* initialize chip structure */
	ddata->dev = dev;
	ddata->pdata = pdata;
	dev->platform_data = pdata;

	/* initialize some structures */
	INIT_WORK(&ddata->reset_work, s2mpb04_reset_work);
	INIT_WORK(&ddata->recover_work, s2mpb04_recover_work);

	/* initialize completions */
	init_completion(&ddata->init_complete);
	init_completion(&ddata->adc_conv_complete);

	/* initialize regmap */
	ddata->regmap = devm_regmap_init_i2c(client, &s2mpb04_regmap_config);
	if (IS_ERR(ddata->regmap)) {
		ret = PTR_ERR(ddata->regmap);
		dev_err(dev,
			"%s: could not initialize regmap (%d)\n",
			__func__, ret);
		return -ENOMEM;
	}

	/* request GPIOs and IRQs */
	devm_gpio_request_one(dev, pdata->pon_gpio, GPIOF_OUT_INIT_LOW,
			      "S2MPB04 PON");
	devm_gpio_request_one(dev, pdata->resetb_gpio, GPIOF_IN,
			      "S2MPB04 RESETB");
	devm_gpio_request_one(dev, pdata->intb_gpio, GPIOF_IN,
			      "S2MPB04 INTB");
	ret = devm_request_threaded_irq(dev, pdata->intb_irq, NULL,
					s2mpb04_intb_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"s2mpb04-intb", ddata);

	/* initialize chip */
	s2mpb04_chip_init(ddata);

	/* force state machine in low power state */
	s2mpb04_prepare_pon(ddata);

	for (i = 0; i < S2MPB04_PON_RETRY_CNT; i++) {
		dev_dbg(dev, "%s: powering on s2mpb04\n", __func__);
		gpio_set_value_cansleep(pdata->pon_gpio, 1);

		/* give the chip some time to power on */
		msleep(S2MPB04_PON_RESET_DELAY);

		/* poll on the gpio until it goes high or until we timeout */
		timeout = jiffies + S2MPB04_PON_RESET_TIMEOUT;
		while (!gpio_get_value(pdata->resetb_gpio) &&
		       time_before(jiffies, timeout)) {
			usleep_range(100, 105);
		}

		if (gpio_get_value(pdata->resetb_gpio))
			break;

		dev_err(dev, "%s: powering on timed out, try (%d/%d)\n",
			__func__, i + 1, S2MPB04_PON_RETRY_CNT);

		gpio_set_value_cansleep(pdata->pon_gpio, 0);
		usleep_range(100, 105);
	}

	/* check for failure */
	if (i == S2MPB04_PON_RETRY_CNT) {
		ret = -ETIMEDOUT;
		dev_err(dev, "%s: powering on failed\n", __func__);
		goto error_reset;
	}

	/* create sysfs attributes */
	s2mpb04_config_sysfs(dev);

	/* Register resetb irq handler after setting PON to avoid IRQ
	 * firing unnecessarily.
	 */
	ret = devm_request_threaded_irq(dev, pdata->resetb_irq, NULL,
					s2mpb04_resetb_irq_handler,
					IRQF_TRIGGER_FALLING |
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"s2mpb04-resetb", ddata);

	/* fixup some chip settings that are different than reset values */
	s2mpb04_core_fixup(ddata);

	return mfd_add_devices(dev, -1, s2mpb04_devs, ARRAY_SIZE(s2mpb04_devs),
			       NULL, 0, NULL);

error_reset:
	return ret;
}

#ifdef CONFIG_PM
static int s2mpb04_suspend(struct device *dev)
{
	struct s2mpb04_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		enable_irq_wake(pdata->intb_irq);
	return 0;
}

static int s2mpb04_resume(struct device *dev)
{
	struct s2mpb04_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		disable_irq_wake(pdata->intb_irq);
	return 0;
}

static const struct dev_pm_ops s2mpb04_dev_pm_ops = {
	.suspend = s2mpb04_suspend,
	.resume  = s2mpb04_resume,
};
#endif

static const struct of_device_id s2mpb04_dt_ids[] = {
	{ .compatible = "samsung,s2mpb04", },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpb04_dt_ids);

static const struct i2c_device_id s2mpb04_id_table[] = {
	{ .name = DRIVER_NAME, .driver_data = 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s2mpb04_id_table);

static struct i2c_driver s2mpb04_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &s2mpb04_dev_pm_ops,
#endif
	},
	.probe = s2mpb04_probe,
	.id_table = s2mpb04_id_table,
};

module_i2c_driver(s2mpb04_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPB04 Device Driver");
MODULE_LICENSE("GPL");

/*
 * Copyright (C) 2017-2018 Google, Inc.
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


#include "s2mpg01-core.h"

#define DRIVER_NAME "s2mpg01"

/* defines the number of tries to repeat an I2C transaction */
#define S2MPG01_I2C_RETRY_COUNT 2

/* defines the delay in ms for reset completion */
#define S2MPG01_PON_RESET_DELAY 2

/* defines the timeout in jiffies for reset completion */
#define S2MPG01_PON_RESET_TIMEOUT msecs_to_jiffies(15)

/* defines the number of retries for powering on */
#define S2MPG01_PON_RETRY_CNT 1

/* defines the timeout in jiffies for reset completion after shutdown */
#define S2MPG01_SHUTDOWN_RESET_TIMEOUT msecs_to_jiffies(1000)

/* defines the timeout in jiffies for ADC conversion completion */
#define S2MPG01_ADC_CONV_TIMEOUT  msecs_to_jiffies(100)

static int s2mpg01_chip_init(struct s2mpg01_core *ddata);
static void s2mpg01_print_status(struct s2mpg01_core *ddata);

static const struct mfd_cell s2mpg01_devs[] = {
	/* TODO(b118705469): change name to s2mpg01 with dts change */
	{
		.name = "s2mpb04-regulator",
		.of_compatible = "samsung,s2mpb04-regulator",
	},
	{
		.name = "s2mpb04-gpio",
		.of_compatible = "samsung,s2mpb04-gpio",
	},
#if 0  /* STOPSHIP: b/120006694 */
	{
		.name = "s2mpb04-thermal",
		.of_compatible = "samsung,s2mpb04-thermal",
	},
#endif
};

static const struct regmap_config s2mpg01_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

int s2mpg01_toggle_pon(struct s2mpg01_core *ddata)
{
	unsigned long timeout;

	dev_info(ddata->dev, "%s: toggling PON\n", __func__);

	reinit_completion(&ddata->init_complete);

	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 0);
	usleep_range(20, 25);
	gpio_set_value_cansleep(ddata->pdata->pon_gpio, 1);

	/* wait for chip to come out of reset, signaled by resetb interrupt */
	timeout = wait_for_completion_timeout(&ddata->init_complete,
					      S2MPG01_SHUTDOWN_RESET_TIMEOUT);
	if (!timeout)
		dev_err(ddata->dev,
			"%s: timeout waiting for device to return from reset\n",
			__func__);

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg01_toggle_pon);

int s2mpg01_dump_regs(struct s2mpg01_core *ddata)
{
	u8 reg_data;
	int i;

	for (i = 0; i <= 64; i++) {
		s2mpg01_read_byte(ddata, i, &reg_data);
		dev_info(ddata->dev, "[0x%02x] = 0x%02x\n", i, reg_data);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpg01_dump_regs);

static inline bool s2mpg01_is_ready(struct s2mpg01_core *ddata)
{
#if 0
	return (gpio_get_value(ddata->pdata->pmic_ready_gpio) == 1);
#else
	return true;
#endif
}

int s2mpg01_read_byte(struct s2mpg01_core *ddata, u8 addr, u8 *data)
{
	int ret;
	unsigned int val;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
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
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_read_byte);

int s2mpg01_read_bytes(struct s2mpg01_core *ddata, u8 addr, u8 *data,
		       size_t count)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_bulk_read(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev, "%s: failed to read %zd bytes from addr 0x%.2x (%d), retry %d\n",
			__func__, count, addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_read_bytes);

int s2mpg01_write_byte(struct s2mpg01_core *ddata, u8 addr, u8 data)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_write(ddata->regmap, addr, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_write_byte);

int s2mpg01_write_bytes(struct s2mpg01_core *ddata, u8 addr, u8 *data,
			size_t count)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_bulk_write(ddata->regmap, addr, data, count);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to write %zd bytes to addr 0x%.2x (%d), retry %d\n",
			count, addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_write_bytes);

int s2mpg01_update_bits(struct s2mpg01_core *ddata, u8 addr,
			unsigned int mask, u8 data)
{
	int ret;
	int retry_cnt = 0;

	if (!s2mpg01_is_ready(ddata))
		return -EBUSY;

	do {
		ret = regmap_update_bits(ddata->regmap, addr, mask, data);
		if (!ret)
			return 0;

		dev_err(ddata->dev,
			"failed to update addr 0x%.2x (%d), retry %d\n",
			addr, ret, retry_cnt);
	} while (++retry_cnt < S2MPG01_I2C_RETRY_COUNT);

	dev_err(ddata->dev, "%s: failed with %d retries, power cycling device\n",
		__func__, S2MPG01_I2C_RETRY_COUNT);

	return -EIO;
}
EXPORT_SYMBOL_GPL(s2mpg01_update_bits);

int s2mpg01_read_adc_chan(struct s2mpg01_core *ddata,
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
	ret = s2mpg01_write_byte(ddata, S2MPG01_REG_MUXSEL1, chan_num);
	if (ret)
		goto adc_cleanup;

	/* enable the clock at 125 kHz, 1 channel, and 8 samples */
	ret = s2mpg01_write_byte(ddata, S2MPG01_REG_ADC_CTRL, 0xC2);
	if (ret)
		goto adc_cleanup;

	dev_dbg(ddata->dev, "%s: started ADC conversion\n", __func__);

	/* wait for completion signaled by interrupt */
	timeout = wait_for_completion_timeout(&ddata->adc_conv_complete,
					      S2MPG01_ADC_CONV_TIMEOUT);
	if (!timeout) {
		ret = -ETIMEDOUT;
		dev_err(ddata->dev, "%s: ADC conversion timeout\n", __func__);
		goto adc_cleanup;
	}

	/* read and format the conversion result */
	ret = s2mpg01_read_byte(ddata, S2MPG01_REG_ADC0DATA, chan_data);
	if (ret)
		goto adc_cleanup;

	dev_dbg(ddata->dev, "%s: chan_data 0x%02x\n", __func__, *chan_data);

adc_cleanup:
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpg01_read_adc_chan);

#define NOTIFY(id, event) s2mpg01_regulator_notify(id, event)

/* print the device id */
static int s2mpg01_print_id(struct s2mpg01_core *ddata)
{
	struct device *dev = ddata->dev;
	u8 reg_data;
	int ret;

	ret = s2mpg01_read_byte(ddata, S2MPG01_REG_CHIP_ID, &reg_data);
	if (!ret) {
		dev_info(dev, "ID: 0x%02X\n", reg_data);
		ddata->rev_id = reg_data;
	} else {
		dev_err(dev, "%s: Could not read PMIC ID (%d)\n", __func__,
			ret);
		return ret;
	}
	return 0;
}

/* print the status register */
static void s2mpg01_print_status(struct s2mpg01_core *ddata)
{
	struct device *dev = ddata->dev;
	u8 status[3];
	int ret;

	ret = s2mpg01_read_bytes(ddata, S2MPG01_REG_STATUS1, status, 3);
	if (!ret)
		dev_err(dev, "%s: Status: 0x%02x, 0x%02x, 0x%02x\n", __func__,
			status[0], status[1], status[2]);
}

/* kernel thread for waiting for chip to come out of reset */
static void s2mpg01_reset_work(struct work_struct *data)
{
	struct s2mpg01_core *ddata = container_of(data, struct s2mpg01_core,
						   reset_work);
	unsigned long timeout;

	/* notify regulators of shutdown event */
	dev_err(ddata->dev,
		"%s: Notifying regulators of shutdown event\n", __func__);
	NOTIFY(S2MPG01_ID_SMPS1, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_SMPS2, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_LDO1, REGULATOR_EVENT_FAIL);
	NOTIFY(S2MPG01_ID_LDO2, REGULATOR_EVENT_FAIL);

	dev_err(ddata->dev,
		"%s: waiting for chip to come out of reset\n", __func__);

	/* wait for chip to come out of reset, signaled by resetb interrupt */
	timeout = wait_for_completion_timeout(&ddata->reset_complete,
					      S2MPG01_SHUTDOWN_RESET_TIMEOUT);
	if (!timeout)
		dev_err(ddata->dev,
			"%s: timeout waiting for device to return from reset\n",
			__func__);
	else
		s2mpg01_chip_init(ddata);

	complete(&ddata->init_complete);
}

/* get platform data from the device tree */
static struct s2mpg01_platform_data *s2mpg01_get_platform_data_from_dt
	(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct s2mpg01_platform_data *pdata;

	if (!np)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->pon_gpio = of_get_named_gpio(np, "samsung,pon-gpio", 0);
	pdata->pmic_ready_gpio = of_get_named_gpio(np,
						   "samsung,pmic-ready-gpio",
						   0);
	pdata->intb_gpio = of_get_named_gpio(np, "samsung,intb-gpio", 0);
	pdata->resetb_irq = gpio_to_irq(pdata->pmic_ready_gpio);
	pdata->intb_irq = gpio_to_irq(pdata->intb_gpio);

	return pdata;
}

/* enable all of the interrupts */
static void s2mpg01_config_ints(struct s2mpg01_core *ddata)
{
	u8 bytes[3];

	/* clear any pending interrupts */
	s2mpg01_read_bytes(ddata, S2MPG01_REG_INT1, bytes, 4);

	/* unmask all (non-reserved) interrupts */
	s2mpg01_write_byte(ddata, S2MPG01_REG_INT1M, 0xFF);
}

/* initialize the chip */
static int s2mpg01_chip_init(struct s2mpg01_core *ddata)
{
	s2mpg01_print_status(ddata);

	s2mpg01_config_ints(ddata);

	return 0;
}

static int s2mpg01_probe(struct i2c_client *client,
			 const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	struct s2mpg01_core *ddata;
	struct s2mpg01_platform_data *pdata;
	unsigned long timeout;
	int ret;
	int i;

	/* allocate memory for chip structure */
	ddata = devm_kzalloc(dev, sizeof(struct s2mpg01_core),
			     GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	/* set client data */
	i2c_set_clientdata(client, ddata);
	dev_set_drvdata(dev, ddata);

	/* get platform data */
	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = s2mpg01_get_platform_data_from_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	/* initialize chip structure */
	ddata->dev = dev;
	ddata->pdata = pdata;
	dev->platform_data = pdata;

	/* initialize some structures */
	INIT_WORK(&ddata->reset_work, s2mpg01_reset_work);

	/* initialize completions */
	init_completion(&ddata->init_complete);
	init_completion(&ddata->reset_complete);
	init_completion(&ddata->adc_conv_complete);

	/* initialize regmap */
	ddata->regmap = devm_regmap_init_i2c(client, &s2mpg01_regmap_config);
	if (IS_ERR(ddata->regmap)) {
		ret = PTR_ERR(ddata->regmap);
		dev_err(dev,
			"%s: could not initialize regmap (%d)\n",
			__func__, ret);
		return -ENOMEM;
	}

	/* request GPIOs and IRQs */
	devm_gpio_request_one(dev, pdata->pon_gpio, GPIOF_OUT_INIT_LOW,
			      "S2MPG01 PON");
	devm_gpio_request_one(dev, pdata->pmic_ready_gpio, GPIOF_IN,
			      "S2MPG01 PMIC READY");
	devm_gpio_request_one(dev, pdata->intb_gpio, GPIOF_IN,
			      "S2MPG01 INTB");

	for (i = 0; i < S2MPG01_PON_RETRY_CNT; i++) {
		dev_dbg(dev, "%s: powering on s2mpg01\n", __func__);
		gpio_set_value_cansleep(pdata->pon_gpio, 1);

		/* give the chip some time to power on */
		msleep(S2MPG01_PON_RESET_DELAY);

		/* poll on the gpio until it goes high or until we timeout */
		timeout = jiffies + S2MPG01_PON_RESET_TIMEOUT;
		while (!gpio_get_value(pdata->pmic_ready_gpio) &&
		       time_before(jiffies, timeout)) {
			usleep_range(100, 105);
		}

		if (gpio_get_value(pdata->pmic_ready_gpio))
			break;

		dev_err(dev, "%s: powering on timed out, try (%d/%d)\n",
			__func__, i + 1, S2MPG01_PON_RETRY_CNT);

		gpio_set_value_cansleep(pdata->pon_gpio, 0);
		usleep_range(100, 105);
	}

	/* check for failure */
	if (i == S2MPG01_PON_RETRY_CNT) {
		ret = -ETIMEDOUT;
		dev_err(dev, "%s: powering on failed\n", __func__);
		goto error_reset;
	}

	if (s2mpg01_print_id(ddata)) {
		ret = -ENODEV;
		dev_err(dev, "%s: couldn't communicate over I2C, giving up",
			__func__);
		goto error_reset;
	}

	/* create sysfs attributes */
	s2mpg01_config_sysfs(dev);

	/* enable the irq after power on */
	enable_irq(pdata->resetb_irq);

	/* initialize chip */
	s2mpg01_chip_init(ddata);

	return mfd_add_devices(dev, -1, s2mpg01_devs, ARRAY_SIZE(s2mpg01_devs),
			       NULL, 0, NULL);

error_reset:
	return ret;
}

#ifdef CONFIG_PM
static int s2mpg01_suspend(struct device *dev)
{
	struct s2mpg01_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		enable_irq_wake(pdata->intb_irq);
	return 0;
}

static int s2mpg01_resume(struct device *dev)
{
	struct s2mpg01_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	if (pdata)
		disable_irq_wake(pdata->intb_irq);
	return 0;
}

static const struct dev_pm_ops s2mpg01_dev_pm_ops = {
	.suspend = s2mpg01_suspend,
	.resume  = s2mpg01_resume,
};
#endif

static const struct of_device_id s2mpg01_dt_ids[] = {
	/* TODO(b/118705469): Remove s2mpb04 once dts catches up */
	{ .compatible = "samsung,s2mpb04", },
	{ .compatible = "samsung,s2mpg01", },
	{ },
};
MODULE_DEVICE_TABLE(of, s2mpg01_dt_ids);

static const struct i2c_device_id s2mpg01_id_table[] = {
	{ .name = DRIVER_NAME, .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mpg01_id_table);

static struct i2c_driver s2mpg01_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(s2mpg01_dt_ids),
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &s2mpg01_dev_pm_ops,
#endif
	},
	.probe = s2mpg01_probe,
	.id_table = s2mpg01_id_table,
};

module_i2c_driver(s2mpg01_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPG01 Device Driver");
MODULE_LICENSE("GPL");

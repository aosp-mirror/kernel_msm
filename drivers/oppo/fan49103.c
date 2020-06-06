/*
 * FAN49103 Fairchild Digitally Programmable TinyBuck Regulator Driver.
i*
 * Copyright (C), 2008-2019, OPPO Mobile Comm Corp., Ltd.
 * Author: Zeng Zhaoxiu <zengzhaoxiu@oppo.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <include/of_regulator318.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <include/fan49103.h>

#define REG_SOFT_RESET		0x00
#define REG_VOUT_REF		0x01
#define REG_CTRL			0x02
#define REG_MANUFACTURER_ID	0x40
#define REG_DEVICE_ID		0x41

#define VOUT_MASK			0x7F
#define VOUT_MIN			0x2F
#define VOUT_MAX			0x5F
#define VOUT_DEFAULT		0x47

#define mV_VOUT_MIN			2800
#define mV_VOUT_MAX			4000
#define mV_VOUT_STEP		25


#define	CTRL_PASS_THROUGH_EN	BIT(3)
#define CTRL_FORCE_PWM			BIT(2)

#define FAN49103_MANUFACTURER_ID	0x83
#define FAN49103_DEVICE_ID1			0x06
#define FAN49103_DEVICE_ID2			0x07

struct fan49103_device_info {
	struct i2c_client *client;
	struct fan49103_platform_data *pdata;
	struct regulator_dev *rdev;
	struct mutex lock;
	u8 reg_cache[2];
	int irq_pg; 
};

static int fan49103_write_reg(struct fan49103_device_info *di, u8 reg, u8 mask, u8 val)
{
	int ret = 0;

	if (reg < REG_VOUT_REF || reg > REG_CTRL)
		return -EINVAL;

	mutex_lock(&di->lock);
	if ((di->reg_cache[reg - 1] & mask) != val) {
		val |= (di->reg_cache[reg - 1] & ~mask);
		ret = i2c_smbus_write_byte_data(di->client, reg, val);
		if (!ret)
			di->reg_cache[reg - 1] = val;
	}
	mutex_unlock(&di->lock);

	if (ret < 0)
		dev_err(&di->client->dev, "%s (%d) failed: %d\n", __func__, reg, ret);
	else
		dev_dbg(&di->client->dev, "REG%d = %x\n", reg, di->reg_cache[reg - 1]);
	return ret;
}

static int fan49103_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	struct fan49103_device_info *di = rdev_get_drvdata(rdev);

	selector += VOUT_MIN;
	if (selector < VOUT_MIN || selector > VOUT_MAX)
		return -ERANGE;

	return fan49103_write_reg(di, REG_VOUT_REF, 0x7f, selector);
}

static int fan49103_get_voltage_sel(struct regulator_dev *rdev)
{
	struct fan49103_device_info *di = rdev_get_drvdata(rdev);
	return (di->reg_cache[REG_VOUT_REF - 1] & 0x7f) - VOUT_MIN;
}

static int fan49103_set_bypass(struct regulator_dev *rdev, bool enable)
{
	struct fan49103_device_info *di = rdev_get_drvdata(rdev);
	return fan49103_write_reg(di, REG_CTRL, CTRL_PASS_THROUGH_EN, enable ? CTRL_PASS_THROUGH_EN : 0);
}

static int fan49103_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct fan49103_device_info *di = rdev_get_drvdata(rdev);
	int ret;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		ret = fan49103_write_reg(di, REG_CTRL, CTRL_FORCE_PWM, CTRL_FORCE_PWM);
		break;
	case REGULATOR_MODE_NORMAL:
		ret = fan49103_write_reg(di, REG_CTRL, CTRL_FORCE_PWM, 0);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static unsigned int fan49103_get_mode(struct regulator_dev *rdev)
{
	struct fan49103_device_info *di = rdev_get_drvdata(rdev);
	if (di->reg_cache[REG_CTRL - 1] & CTRL_FORCE_PWM)
		return REGULATOR_MODE_FAST;
	else
		return REGULATOR_MODE_NORMAL;
}

static struct regulator_ops fan49103_reg_ops = {
	.set_voltage_sel = fan49103_set_voltage_sel,
	.get_voltage_sel = fan49103_get_voltage_sel,

	.map_voltage = regulator_map_voltage_linear,
	.list_voltage = regulator_list_voltage_linear,

	.set_bypass = fan49103_set_bypass,
	.set_mode = fan49103_set_mode,
	.get_mode = fan49103_get_mode,
};

static const struct regulator_desc fan49103_reg_desc = {
	.name = "fan49103-reg",
	.supply_name = "vin",
	.ops = &fan49103_reg_ops,
	.type = REGULATOR_VOLTAGE,
	.n_voltages = VOUT_MAX - VOUT_MIN + 1,
	.min_uV = mV_VOUT_MIN * 1000,
	.uV_step = mV_VOUT_STEP * 1000,
	.owner = THIS_MODULE,
};

static irqreturn_t fan49103_power_fail(int irq, void *data)
{
	struct fan49103_device_info *di = data;

	regulator_notifier_call_chain(di->rdev,
					REGULATOR_EVENT_FAIL, NULL);

	return IRQ_HANDLED;
}

static struct fan49103_platform_data *fan49103_parse_dt(struct device *dev,
					      struct device_node *np)
{
	struct fan49103_platform_data *pdata;
	int rc;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->regulator = of_get_regulator_init_data(dev, np);

	pdata->gpio_en = of_get_named_gpio(np, "enable-gpio", 0);
	pdata->gpio_pg = of_get_named_gpio(np, "pg-gpio", 0);

	rc = of_property_read_u32(np, "fan,initial-microvolt", &pdata->init_uv);
	if (rc < 0)
		pdata->init_uv = 0;

	return pdata;
}

static int fan49103_setup(struct fan49103_device_info *di)
{
	struct regulator_config config = {
		.dev = &di->client->dev,
		.init_data = di->pdata->regulator,
		.driver_data = di,
		.of_node = di->client->dev.of_node,
		.ena_gpio = di->pdata->gpio_en,
		.ena_gpio_flags = di->pdata->regulator->constraints.boot_on ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
		.ena_gpio_invert = 0,
	};
	int ret;

	mutex_init(&di->lock);

	ret = i2c_smbus_read_byte_data(di->client, REG_VOUT_REF);
	if (ret < 0) {
		dev_err(&di->client->dev, "Failed to read REG_VOUT_REF!\n");
		return ret;
	}
	di->reg_cache[REG_VOUT_REF - 1] = ret;
	dev_dbg(&di->client->dev, "REG_VOUT_REF = %x\n", ret);

	ret = i2c_smbus_read_byte_data(di->client, REG_CTRL);
	if (ret < 0) {
		dev_err(&di->client->dev, "Failed to read REG_CTRL!\n");
		return ret;
	}
	di->reg_cache[REG_CTRL - 1] = ret;
	dev_dbg(&di->client->dev, "REG_CTRL = %x\n", ret);

	if (di->pdata->init_uv != 0) {
		u8 selector = (di->pdata->init_uv - mV_VOUT_MIN * 1000) / (mV_VOUT_STEP * 1000) + VOUT_MIN;
		ret = fan49103_write_reg(di, REG_VOUT_REF, 0x7f, selector);
		if (ret < 0)
			return ret;
	}

	if (gpio_is_valid(di->pdata->gpio_pg)) {
		ret = devm_gpio_request_one(&di->client->dev, di->pdata->gpio_pg, GPIOF_IN, "PG");
		if (ret)
			return ret;

		di->irq_pg = gpio_to_irq(di->pdata->gpio_pg);

		if (di->irq_pg >= 0) {
			ret = request_threaded_irq(di->irq_pg, NULL, fan49103_power_fail,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT, "PG", di);
			if (ret) {
				dev_err(&di->client->dev,
						"Failed to request PG IRQ.\n");
				di->irq_pg = -ENXIO;
			}
		}
	}

	/* Register regulator */
	di->rdev = devm_regulator_register(&di->client->dev, &fan49103_reg_desc, &config);
	if (IS_ERR(di->rdev)) {
		ret = PTR_ERR(di->rdev);
		dev_err(&di->client->dev, "Failed to register regulator!\n");
	}

	return 0;
}

static const struct of_device_id fan49103_dt_ids[] = {
	{ .compatible = "fcs,fan49103", },
	{ }
};
MODULE_DEVICE_TABLE(of, fan49103_dt_ids);

static int fan49103_regulator_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct fan49103_device_info *di;
	struct fan49103_platform_data *pdata;
	int ret;

	ret = i2c_smbus_read_byte_data(client, REG_MANUFACTURER_ID);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to get manufacturer ID!\n");
		return ret;
	}
	if (ret != FAN49103_MANUFACTURER_ID) {
		dev_err(&client->dev, "Wrong manufacturer ID (%d)!\n", ret);
		return -ENOENT;
	}

	ret = i2c_smbus_read_byte_data(client, REG_DEVICE_ID);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to get device ID!\n");
		return ret;
	}

	if (ret != FAN49103_DEVICE_ID1 && ret != FAN49103_DEVICE_ID2) {
		dev_err(&client->dev, "Wrong device ID (%d)!\n", ret);
		return -ENODEV;
	}

	dev_info(&client->dev, "FAN49103 Detected!\n");

	pdata = dev_get_platdata(&client->dev);
	if (!pdata)
		pdata = fan49103_parse_dt(&client->dev, client->dev.of_node);
	if (!pdata || !pdata->regulator) {
		dev_err(&client->dev, "Platform data not found!\n");
		return -ENODEV;
	} 

	di = devm_kzalloc(&client->dev, sizeof(struct fan49103_device_info),
					GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->client = client;
	di->pdata = pdata;

	ret = fan49103_setup(di);
	if (ret)
		return ret;

	i2c_set_clientdata(client, di);

	return 0;
}

static const struct i2c_device_id fan49103_id[] = {
	{ .name = "fan49103", }, 
	{ },
};

MODULE_DEVICE_TABLE(i2c, fan49103_id);

static struct i2c_driver fan49103_regulator_driver = {
	.driver = {
		.name = "fan49103-regulator",
		.of_match_table = of_match_ptr(fan49103_dt_ids),
	},
	.probe = fan49103_regulator_probe,
	.id_table = fan49103_id,
};



module_i2c_driver(fan49103_regulator_driver);
MODULE_AUTHOR("Zeng Zhaoxiu <zengzhaoxiu@oppo.com>");
MODULE_DESCRIPTION("FAN49103 regulator driver");
MODULE_LICENSE("GPL v2");

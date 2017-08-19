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

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "s2mpb04-core.h"

#define DRIVER_NAME "s2mpb04-regulator"

struct s2mpb04_regulator {
	struct device *dev;
	struct s2mpb04_core *s2mpb04_core;
	struct regulator_dev **rdevs;

	/* bitmask for tracking enabled regulators */
	unsigned long reg_enabled_mask;
};

static struct s2mpb04_regulator *_s2mpb04_regulator;

static int s2mpb04_regulator_get_voltage(struct regulator_dev *rdev);
static int s2mpb04_regulator_enable(struct regulator_dev *rdev);
static int s2mpb04_regulator_disable(struct regulator_dev *rdev);
static int s2mpb04_regulator_is_enabled(struct regulator_dev *rdev);

static struct regulator_ops s2mpb04_regulator_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_ascend,
	.get_voltage = s2mpb04_regulator_get_voltage,
	.enable = s2mpb04_regulator_enable,
	.disable = s2mpb04_regulator_disable,
	.is_enabled = s2mpb04_regulator_is_enabled,
};

/* No support for DVS so just a single voltage level */
static const unsigned int s2mpb04_ldo_vtbl[] = { 1800000 };
static const unsigned int s2mpb04_smps1_vtbl[] = { 900000 };
static const unsigned int s2mpb04_smps2_vtbl[] = { 1100000 };

static struct regulator_desc
	s2mpb04_regulator_desc[S2MPB04_NUM_REGULATORS] = {
	[S2MPB04_ID_SMPS1] = {
		.name = S2MPB04_REGLTR_NAME_SMPS1,
		.id = S2MPB04_ID_SMPS1,
		.ops = &s2mpb04_regulator_ops,
		.n_voltages = ARRAY_SIZE(s2mpb04_smps1_vtbl),
		.volt_table = s2mpb04_smps1_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPB04_ID_SMPS2] = {
		.name = S2MPB04_REGLTR_NAME_SMPS2,
		.id = S2MPB04_ID_SMPS2,
		.ops = &s2mpb04_regulator_ops,
		.n_voltages = ARRAY_SIZE(s2mpb04_smps2_vtbl),
		.volt_table = s2mpb04_smps2_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPB04_ID_LDO1] = {
		.name = S2MPB04_REGLTR_NAME_LDO1,
		.id = S2MPB04_ID_LDO1,
		.ops = &s2mpb04_regulator_ops,
		.n_voltages = ARRAY_SIZE(s2mpb04_ldo_vtbl),
		.volt_table = s2mpb04_ldo_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPB04_ID_LDO2] = {
		.name = S2MPB04_REGLTR_NAME_LDO2,
		.id = S2MPB04_ID_LDO2,
		.ops = &s2mpb04_regulator_ops,
		.n_voltages = ARRAY_SIZE(s2mpb04_ldo_vtbl),
		.volt_table = s2mpb04_ldo_vtbl,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static struct regulator_init_data
	s2mpb04_regulator_init_data[S2MPB04_NUM_REGULATORS] = {
	[S2MPB04_ID_SMPS1] = {
		.constraints = {
			.name = "s2mpb04_smps1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 900000,
			.max_uV = 900000,
		},
	},
	[S2MPB04_ID_SMPS2] = {
		.constraints = {
			.name = "s2mpb04_smps2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 1100000,
			.max_uV = 1100000,
		},
	},
	[S2MPB04_ID_LDO1] = {
		.constraints = {
			.name = "s2mpb04_ldo1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 1800000,
			.max_uV = 1800000,
		},
	},
	[S2MPB04_ID_LDO2] = {
		.constraints = {
			.name = "s2mpb04_ldo2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 1800000,
			.max_uV = 1800000,
		},
	},
};

/* get the current voltage of the regulator in microvolts */
static int s2mpb04_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct s2mpb04_regulator *s2mpb04_regulator = rdev_get_drvdata(rdev);
	struct s2mpb04_core *s2mpb04_core = s2mpb04_regulator->s2mpb04_core;
	enum s2mpb04_regulator_ids rid = rdev_get_id(rdev);
	u8 reg_data;
	int vsel, vstep, vbase;
	int ret;

	dev_dbg(s2mpb04_regulator->dev, "%s: rid %d\n", __func__, rid);

	switch (rid) {
	case S2MPB04_ID_SMPS1:
		ret = s2mpb04_read_byte(s2mpb04_core, S2MPB04_REG_BUCK1_OUT,
					&reg_data);
		if (ret)
			return ret;
		vbase = 300000;
		vstep = 6250;
		vsel = reg_data;
		break;
	case S2MPB04_ID_SMPS2:
		ret = s2mpb04_read_byte(s2mpb04_core, S2MPB04_REG_BUCK2_OUT,
					&reg_data);
		vbase = 300000;
		vstep = 6250;
		if (ret)
			return ret;
		vsel = reg_data;
		break;
	case S2MPB04_ID_LDO1:
		ret = s2mpb04_read_byte(s2mpb04_core, S2MPB04_REG_LDO1_CTRL,
					&reg_data);
		if (ret)
			return ret;
		vbase = 700000;
		vstep = 25000;
		vsel = reg_data & 0x3F;
		break;
	case S2MPB04_ID_LDO2:
		ret = s2mpb04_read_byte(s2mpb04_core, S2MPB04_REG_LDO2_CTRL,
					&reg_data);
		if (ret)
			return ret;
		vbase = 700000;
		vstep = 25000;
		vsel = reg_data & 0x3F;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(s2mpb04_regulator->dev, "%s: rid %d, returning voltage %d\n",
		__func__, rid, vbase + vsel * vstep);

	return vbase + vsel * vstep;
}

/* enable the regulator */
static int s2mpb04_regulator_enable(struct regulator_dev *rdev)
{
	struct s2mpb04_regulator *s2mpb04_regulator = rdev_get_drvdata(rdev);
	struct s2mpb04_core *s2mpb04_core = s2mpb04_regulator->s2mpb04_core;
	enum s2mpb04_regulator_ids rid = rdev_get_id(rdev);
	int ret;

	dev_dbg(s2mpb04_regulator->dev, "%s: rid %d\n", __func__, rid);

	switch (rid) {
	case S2MPB04_ID_SMPS1:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_BUCK1_CTRL,
					 0xF8);
		break;
	case S2MPB04_ID_SMPS2:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_BUCK2_CTRL,
					 0xD8);
		break;
	case S2MPB04_ID_LDO1:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_LDO1_CTRL,
					 0xEC);
		break;
	case S2MPB04_ID_LDO2:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_LDO2_CTRL,
					 0xEC);
		break;
	default:
		return -EINVAL;
	}

	if (!ret)
		set_bit(rid, &s2mpb04_regulator->reg_enabled_mask);

	return ret;
}

/* disable the regulator */
static int s2mpb04_regulator_disable(struct regulator_dev *rdev)
{
	struct s2mpb04_regulator *s2mpb04_regulator = rdev_get_drvdata(rdev);
	struct s2mpb04_core *s2mpb04_core = s2mpb04_regulator->s2mpb04_core;
	enum s2mpb04_regulator_ids rid = rdev_get_id(rdev);
	int ret;

	dev_dbg(s2mpb04_regulator->dev, "%s: rid %d\n", __func__, rid);

	switch (rid) {
	case S2MPB04_ID_SMPS1:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_BUCK1_CTRL,
					 0x38);
		break;
	case S2MPB04_ID_SMPS2:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_BUCK2_CTRL,
					 0x18);
		break;
	case S2MPB04_ID_LDO1:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_LDO1_CTRL,
					 0x2C);
		break;
	case S2MPB04_ID_LDO2:
		ret = s2mpb04_write_byte(s2mpb04_core, S2MPB04_REG_LDO2_CTRL,
					 0x2C);
		break;
	default:
		return -EINVAL;
	}

	if (!ret)
		clear_bit(rid, &s2mpb04_regulator->reg_enabled_mask);

	return ret;
}

/* get regulator enable status */
static int s2mpb04_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct s2mpb04_regulator *s2mpb04_regulator = rdev_get_drvdata(rdev);
	enum s2mpb04_regulator_ids rid = rdev_get_id(rdev);

	dev_dbg(s2mpb04_regulator->dev, "%s: rid %d\n", __func__, rid);

	if ((rid >= 0) && (rid < S2MPB04_NUM_REGULATORS))
		return !!(s2mpb04_regulator->reg_enabled_mask & (1 << rid));
	else
		return -EINVAL;
}

void s2mpb04_regulator_notify(enum s2mpb04_regulator_ids rid,
			      unsigned long event)
{
	if (!_s2mpb04_regulator || !_s2mpb04_regulator->rdevs ||
	    !_s2mpb04_regulator->rdevs[rid])
		return;

	dev_err(_s2mpb04_regulator->dev, "%s: rid %d, event 0x%lx\n", __func__,
		rid, event);

	s2mpb04_regulator_disable(_s2mpb04_regulator->rdevs[rid]);

	regulator_notifier_call_chain(_s2mpb04_regulator->rdevs[rid], event,
				      NULL);
}
EXPORT_SYMBOL_GPL(s2mpb04_regulator_notify);

/* register a regulator with the kernel regulator framework */
static int
s2mpb04_regulator_register(struct s2mpb04_regulator *s2mpb04_regulator)
{
	struct device *dev = s2mpb04_regulator->dev;
	struct s2mpb04_core *s2mpb04_core = s2mpb04_regulator->s2mpb04_core;
	struct regulator_config cfg = {
		.dev = dev,
		.driver_data = s2mpb04_regulator,
		.regmap = s2mpb04_core->regmap
	};
	struct regulator_dev *rdev;
	int i;

	for (i = 0; i < S2MPB04_NUM_REGULATORS; i++) {
		cfg.init_data = &s2mpb04_regulator_init_data[i];
		rdev = devm_regulator_register(dev,
					       &s2mpb04_regulator_desc[i],
					       &cfg);
		if (IS_ERR(rdev)) {
			dev_err(dev,
				"%s: failed to register regulator %d\n",
				__func__, i);
			return PTR_ERR(rdev);
		}

		*(s2mpb04_regulator->rdevs + i) = rdev;
	}

	return 0;
}

static int s2mpb04_regulator_probe(struct platform_device *pdev)
{
	struct s2mpb04_core *s2mpb04_core = dev_get_drvdata(pdev->dev.parent);
	struct s2mpb04_regulator *s2mpb04_regulator;
	struct device *dev = &pdev->dev;

	s2mpb04_regulator = devm_kzalloc(dev, sizeof(*s2mpb04_regulator),
					 GFP_KERNEL);
	if (!s2mpb04_regulator)
		return -ENOMEM;

	s2mpb04_regulator->dev = dev;
	s2mpb04_regulator->s2mpb04_core = s2mpb04_core;
	_s2mpb04_regulator = s2mpb04_regulator;

	platform_set_drvdata(pdev, s2mpb04_regulator);

	/* initialize and register device regulators */
	s2mpb04_regulator->rdevs =
		devm_kzalloc(dev,
			     S2MPB04_NUM_REGULATORS *
			     sizeof(struct regulator_dev *),
			     GFP_KERNEL);
	if (!s2mpb04_regulator->rdevs) {
		dev_err(dev,
			"%s: could not initialize rdevs array\n",
			__func__);
		return -ENOMEM;
	}

	return s2mpb04_regulator_register(s2mpb04_regulator);
}

static int s2mpb04_regulator_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id s2mpb04_regulator_of_match[] = {
	{ .compatible = "samsung,s2mpb04-regulator", },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpb04_regulator_of_match);

static struct platform_driver s2mpb04_regulator_driver = {
	.probe = s2mpb04_regulator_probe,
	.remove = s2mpb04_regulator_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = s2mpb04_regulator_of_match,
	},
};
module_platform_driver(s2mpb04_regulator_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPB04 Regulator Driver");
MODULE_LICENSE("GPL");

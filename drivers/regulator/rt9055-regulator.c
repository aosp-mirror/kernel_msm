/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/types.h>

#define RT9055_VOLTAGE_VERSION_G 1800000
#define RT9055_VOLTAGE_LEVELS 1

struct rt9055_regulator {
	struct regulator_dev *rdev;
	struct device_node *node;
	struct regulator_desc rdesc;
	const char *name;
	int en_gpio;
	int u_volt;
	bool is_enabled;
};

struct rt9055_chip {
	struct rt9055_regulator *vreg;
	struct device *dev;
	int num_regulators;
};

/* RT9055-GG: both out1 and out2 are fixed to 1.8V */
static struct of_regulator_match rt9055_matches[] = {
	{ .name = "rt9055_gg_vout1",
		.driver_data = (void *)RT9055_VOLTAGE_VERSION_G },
	{ .name = "rt9055_gg_vout2",
		.driver_data = (void *)RT9055_VOLTAGE_VERSION_G },
};

static int rt9055_regulator_disable(struct regulator_dev *rdev)
{
	struct rt9055_regulator *vreg = rdev_get_drvdata(rdev);

	pr_info("%s: %s\n", __func__, vreg->name);

	gpio_set_value(vreg->en_gpio, 0);
	vreg->is_enabled = false;

	return 0;
}

static int rt9055_regulator_enable(struct regulator_dev *rdev)
{
	struct rt9055_regulator *vreg = rdev_get_drvdata(rdev);

	pr_info("%s: %s\n", __func__, vreg->name);

	gpio_set_value(vreg->en_gpio, 1);
	vreg->is_enabled = true;

	return 0;
}

static int rt9055_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct rt9055_regulator *vreg = rdev_get_drvdata(rdev);

	return vreg->u_volt;
}

static int rt9055_regulator_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned int *selector)
{
	return 0;
}

static int rt9055_regulator_list_voltage(struct regulator_dev *rdev,
							unsigned int selector)
{
	struct rt9055_regulator *vreg = rdev_get_drvdata(rdev);

	return vreg->u_volt;
}

static int rt9055_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct rt9055_regulator *vreg = rdev_get_drvdata(rdev);

	return vreg->is_enabled ? 1 : 0;
}

static struct regulator_ops rt9055_ops = {
	.set_voltage = rt9055_regulator_set_voltage,
	.get_voltage = rt9055_regulator_get_voltage,
	.list_voltage = rt9055_regulator_list_voltage,
	.enable = rt9055_regulator_enable,
	.disable = rt9055_regulator_disable,
	.is_enabled = rt9055_regulator_is_enabled,
};

static int rt9055_regulator_gpio_init(struct rt9055_chip *chip)
{
	struct rt9055_regulator *vreg;
	int i, rc = 0;

	/* TBD */
	for (i = 0; i < chip->num_regulators; i++) {
		vreg = &chip->vreg[i];
		vreg->is_enabled = true;
	}

	return rc;
}

static int rt9055_parse_dt(struct rt9055_chip *chip,
					struct platform_device *pdev)
{
	struct device_node *node;
	struct of_regulator_match *match;
	int i, rc, volt_ver;

	if (!pdev->dev.of_node) {
		pr_err("%s: device node missing\n", __func__);
		return -EINVAL;
	}

	node = of_find_node_by_name(pdev->dev.of_node, "regulators");
	if (!node) {
		pr_err("%s: get regulators node failed\n", __func__);
		return -EINVAL;
	}

	rc = of_regulator_match(&pdev->dev, node, rt9055_matches,
					ARRAY_SIZE(rt9055_matches));
	if (rc < 0) {
		pr_err("%s: regulator match failed, rc = %d\n", __func__, rc);
		return rc;
	}

	chip->num_regulators = rc;

	chip->vreg = devm_kzalloc(&pdev->dev, chip->num_regulators *
					sizeof(struct rt9055_regulator),
					GFP_KERNEL);
	if (!chip->vreg) {
		pr_err("%s: memory allocation failed for vreg\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < chip->num_regulators; i++) {
		match = &rt9055_matches[i];
		chip->vreg[i].node = match->of_node;

		volt_ver = (uintptr_t)match->driver_data;
		if (volt_ver == RT9055_VOLTAGE_VERSION_G) {
			chip->vreg[i].u_volt = RT9055_VOLTAGE_VERSION_G;
		} else {
			pr_err("%s: unknown voltage version: %d\n", __func__,
							volt_ver);
			return -EINVAL;
		}

		chip->vreg[i].en_gpio = of_get_named_gpio(
					match->of_node, "richtek,en-gpio", 0);
		if (chip->vreg[i].en_gpio < 0) {
			pr_err("%s: get richtek,en-gpio failed, rc = %d\n",
				__func__, chip->vreg[i].en_gpio);
			return chip->vreg[i].en_gpio;
		}

		pr_info("%s: en-gpio = %d\n", __func__,
					chip->vreg[i].en_gpio);

		rc = 0;
	}

	return rc;
}

static int rt9055_regulator_probe(struct platform_device *pdev)
{
	struct rt9055_chip *chip;
	struct regulator_config config = {};
	struct regulator_desc *rdesc;
	int rc, i, j;

	pr_info("%s\n", __func__);

	chip = devm_kzalloc(&pdev->dev, sizeof(struct rt9055_chip),
				GFP_KERNEL);

	if (!chip) {
		pr_err("%s: memory allocation failed for rt9055 chip\n",
							__func__);
		return -ENOMEM;
	}

	rc = rt9055_parse_dt(chip, pdev);
	if (rc) {
		pr_err("%s: parse device tree failed for rt9055, rc = %d\n",
							__func__, rc);
		return rc;
	}

	chip->dev = &pdev->dev;

	rc = rt9055_regulator_gpio_init(chip);
	if (rc) {
		pr_err("%s: gpios initialize failed for rt9055, rc = %d\n",
							__func__, rc);
		return rc;
	}

	dev_set_drvdata(&pdev->dev, chip);

	for (i = 0; i < chip->num_regulators; i++) {
		config.dev = &pdev->dev;
		config.driver_data = &chip->vreg[i];
		config.of_node = chip->vreg[i].node;

		rdesc = &chip->vreg[i].rdesc;
		rdesc->name = chip->vreg[i].name;
		rdesc->type = REGULATOR_VOLTAGE;
		rdesc->owner = THIS_MODULE;
		rdesc->n_voltages = RT9055_VOLTAGE_LEVELS;
		rdesc->ops = &rt9055_ops;
		chip->vreg[i].rdev = regulator_register(rdesc, &config);
		if (IS_ERR(chip->vreg[i].rdev)) {
			pr_err("%s: regulator register failed, rc = %ld\n",
				__func__, PTR_ERR(chip->vreg[i].rdev));
			for (j = i - 1; j >= 0; j--)
				regulator_unregister(chip->vreg[j].rdev);

			return PTR_ERR(chip->vreg[i].rdev);
		}
	}

	return 0;
}

static int rt9055_regulator_remove(struct platform_device *pdev)
{
	struct rt9055_chip *chip = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < chip->num_regulators; i++)
		regulator_unregister(chip->vreg[i].rdev);

	return 0;
}

static const struct of_device_id rt9055_match_table[] = {
	{ .compatible = "richtek,rt9055", },
	{},
};
MODULE_DEVICE_TABLE(of, rt9055_match_table);

static struct platform_driver rt9055_regulator_driver = {
	.driver = {
		.name = "rt9055",
		.owner = THIS_MODULE,
		.of_match_table	= rt9055_match_table,
	},
	.probe = rt9055_regulator_probe,
	.remove = rt9055_regulator_remove,
};

static int __init rt9055_init(void)
{
	static int registered;

	if (registered)
		return 0;

	registered = 1;

	return platform_driver_register(&rt9055_regulator_driver);
}
arch_initcall(rt9055_init);

static void __exit rt9055_exit(void)
{
	platform_driver_unregister(&rt9055_regulator_driver);
}
module_exit(rt9055_exit);

MODULE_DESCRIPTION("Richteck RT9055 dual LDO regulator driver");
MODULE_LICENSE("GPL v2");

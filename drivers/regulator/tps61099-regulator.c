/*
 * Copyright 2016-2017 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>

struct tps61099 {
	struct device		*dev;
	struct regulator_dev	*rdev;
	struct regulator_desc	rdesc;
	struct regulator_ops	*reg_ops;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pinctrl_active_state;
	struct pinctrl_state	*pinctrl_sleep_state;

	struct gpio_desc	*enable_gpio;

	bool enabled;
};

static int tps61099_regulator_enable_pinctrl(struct regulator_dev *rdev)
{
	struct tps61099 *tps61099 = rdev_get_drvdata(rdev);
	int rc;

	rc = pinctrl_select_state(tps61099->pinctrl,
				  tps61099->pinctrl_active_state);
	if (rc < 0) {
		dev_err(tps61099->dev, "cannot select active state rc=%d\n",
			rc);
		return rc;
	}
	tps61099->enabled = true;

	return rc;
}

static int tps61099_regulator_disable_pinctrl(struct regulator_dev *rdev)
{
	struct tps61099 *tps61099 = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = pinctrl_select_state(tps61099->pinctrl,
				  tps61099->pinctrl_sleep_state);
	if (rc < 0) {
		dev_err(tps61099->dev, "cannot select sleep state rc=%d\n", rc);
		return rc;
	}
	tps61099->enabled = false;

	return rc;
}

static int tps61099_regulator_enable_gpio(struct regulator_dev *rdev)
{
	struct tps61099 *tps61099 = rdev_get_drvdata(rdev);

	gpiod_set_value(tps61099->enable_gpio, true);

	tps61099->enabled = true;

	return 0;
}

static int tps61099_regulator_disable_gpio(struct regulator_dev *rdev)
{
	struct tps61099 *tps61099 = rdev_get_drvdata(rdev);

	gpiod_set_value(tps61099->enable_gpio, false);

	tps61099->enabled = false;

	return 0;
}

static int tps61099_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct tps61099 *tps61099 = rdev_get_drvdata(rdev);

	return tps61099->enabled ? 1 : 0;
}

static struct regulator_ops tps61099_reg_ops_pinctrl = {
	.enable = tps61099_regulator_enable_pinctrl,
	.disable = tps61099_regulator_disable_pinctrl,
	.is_enabled = tps61099_regulator_is_enabled,
};

static struct regulator_ops tps61099_reg_ops_gpio = {
	.enable = tps61099_regulator_enable_gpio,
	.disable = tps61099_regulator_disable_gpio,
	.is_enabled = tps61099_regulator_is_enabled,
};

static struct tps61099 *tps61099_init_pinctrl(struct platform_device *pdev)
{
	struct tps61099 *tps61099;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_active_state;
	struct pinctrl_state *pinctrl_sleep_state;
	int rc = 0;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		rc = PTR_ERR(pinctrl);
		dev_err(&pdev->dev, "cannot get pinctrl rc=%d\n", rc);
		return ERR_PTR(rc);
	}

	pinctrl_active_state = pinctrl_lookup_state(pinctrl, "tps61099_active");
	if (IS_ERR(pinctrl_active_state)) {
		rc = PTR_ERR(pinctrl_active_state);
		dev_err(&pdev->dev,
			"cannot lookup pinctrl active state rc=%d\n",
			rc);
		return ERR_PTR(rc);
	}

	pinctrl_sleep_state = pinctrl_lookup_state(pinctrl, "tps61099_sleep");
	if (IS_ERR(pinctrl_sleep_state)) {
		rc = PTR_ERR(pinctrl_sleep_state);
		dev_err(&pdev->dev,
			"cannot lookup pinctrl sleep state rc=%d\n",
			rc);
		return ERR_PTR(rc);
	}

	rc = pinctrl_select_state(pinctrl, pinctrl_sleep_state);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"cannot select sleep pinctrl during driver probe rc=%d\n",
			rc);
		return ERR_PTR(rc);
	}

	tps61099 = devm_kzalloc(&pdev->dev, sizeof(*tps61099),
				GFP_KERNEL);
	if (!tps61099)
		return ERR_PTR(-ENOMEM);

	tps61099->dev = &pdev->dev;
	tps61099->pinctrl = pinctrl;
	tps61099->pinctrl_active_state = pinctrl_active_state;
	tps61099->pinctrl_sleep_state = pinctrl_sleep_state;
	tps61099->reg_ops = &tps61099_reg_ops_pinctrl;

	return tps61099;
}

static void tps61099_deinit_pinctrl(struct tps61099 *tps61099)
{}

static struct tps61099 *tps61099_init_gpio(struct platform_device *pdev)
{
	struct tps61099 *tps61099;
	struct gpio_desc *enable_gpio;
	int rc = 0;

	enable_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(enable_gpio)) {
		rc = PTR_ERR(enable_gpio);
		dev_err(&pdev->dev, "cannot get gpio rc=%d\n", rc);
		return ERR_PTR(rc);
	}

	tps61099 = devm_kzalloc(&pdev->dev, sizeof(*tps61099),
				GFP_KERNEL);
	if (!tps61099)
		return ERR_PTR(-ENOMEM);

	tps61099->dev = &pdev->dev;
	tps61099->enable_gpio = enable_gpio;
	tps61099->reg_ops = &tps61099_reg_ops_gpio;

	return tps61099;
}

static void tps61099_deinit_gpio(struct tps61099 *tps61099)
{}

static struct tps61099 *tps61099_init(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	bool uses_gpio = of_property_read_bool(node, "uses_gpio");

	if (uses_gpio)
		return tps61099_init_gpio(pdev);
	else
		return tps61099_init_pinctrl(pdev);
}

static void tps61099_deinit(struct tps61099 *tps61099)
{
	struct device_node *node = tps61099->dev->of_node;
	bool uses_gpio = of_property_read_bool(node, "uses_gpio");

	if (uses_gpio)
		tps61099_deinit_gpio(tps61099);
	else
		tps61099_deinit_pinctrl(tps61099);
}

static int tps61099_probe(struct platform_device *pdev)
{
	struct tps61099 *tps61099;
	struct regulator_config cfg = {};
	int rc = 0;

	tps61099 = tps61099_init(pdev);

	if (IS_ERR(tps61099))
		return PTR_ERR(tps61099);

	cfg.dev = tps61099->dev;
	cfg.driver_data = tps61099;
	tps61099->rdesc.owner = THIS_MODULE;
	tps61099->rdesc.type = REGULATOR_VOLTAGE;
	tps61099->rdesc.ops = tps61099->reg_ops;
	tps61099->rdesc.of_match = "ti,tps61099";
	tps61099->rdesc.name = "ti,tps61099";
	tps61099->rdev = devm_regulator_register(tps61099->dev,
						 &tps61099->rdesc,
						 &cfg);
	if (IS_ERR(tps61099->rdev)) {
		rc = PTR_ERR(tps61099->rdev);
		tps61099->rdev = NULL;
		dev_err(tps61099->dev, "cannot register regulator rc=%d\n", rc);
		goto deinit_tps;
	}

	platform_set_drvdata(pdev, tps61099);

	dev_dbg(tps61099->dev, "TPS61099 driver successfully probed\n");

	return rc;

deinit_tps:
	tps61099_deinit(tps61099);
	return rc;
}

static int tps61099_remove(struct platform_device *pdev)
{
	struct tps61099 *tps61099 = platform_get_drvdata(pdev);

	regulator_unregister(tps61099->rdev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void tps61099_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id match_table[] = {
	{ .compatible = "ti,tps61099", },
	{ },
};

static struct platform_driver tps61099_driver = {
	.driver		= {
		.name		= "ti,tps61099",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
	},
	.probe		= tps61099_probe,
	.remove		= tps61099_remove,
	.shutdown	= tps61099_shutdown,
};
module_platform_driver(tps61099_driver);

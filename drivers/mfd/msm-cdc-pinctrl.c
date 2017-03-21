/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mfd/msm-cdc-pinctrl.h>

struct msm_cdc_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_active;
	struct pinctrl_state *pinctrl_sleep;
	struct regulator *pinctrl_supply;
	int gpio;
	bool state;
};

static struct msm_cdc_pinctrl_info *msm_cdc_pinctrl_get_gpiodata(
						struct device_node *np)
{
	struct platform_device *pdev;
	struct msm_cdc_pinctrl_info *gpio_data;

	if (!np) {
		pr_err("%s: device node is null\n", __func__);
		return NULL;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		pr_err("%s: platform device not found!\n", __func__);
		return NULL;
	}

	gpio_data = dev_get_drvdata(&pdev->dev);
	if (!gpio_data)
		dev_err(&pdev->dev, "%s: cannot find cdc gpio info\n",
			__func__);

	return gpio_data;
}

/*
 * msm_cdc_get_gpio_state: select pinctrl sleep state
 * @np: pointer to struct device_node
 *
 * Returns error code for failure and GPIO value on success
 */
int msm_cdc_get_gpio_state(struct device_node *np)
{
	struct msm_cdc_pinctrl_info *gpio_data;
	int value = -EINVAL;

	gpio_data = msm_cdc_pinctrl_get_gpiodata(np);
	if (!gpio_data)
		return value;

	if (gpio_is_valid(gpio_data->gpio))
		value = gpio_get_value_cansleep(gpio_data->gpio);

	return value;
}
EXPORT_SYMBOL(msm_cdc_get_gpio_state);

/*
 * msm_cdc_pinctrl_select_sleep_state: select pinctrl sleep state
 * @np: pointer to struct device_node
 *
 * Returns error code for failure
 */
int msm_cdc_pinctrl_select_sleep_state(struct device_node *np)
{
	struct msm_cdc_pinctrl_info *gpio_data;

	gpio_data = msm_cdc_pinctrl_get_gpiodata(np);
	if (!gpio_data)
		return -EINVAL;

	if (!gpio_data->pinctrl_sleep) {
		pr_err("%s: pinctrl sleep state is null\n", __func__);
		return -EINVAL;
	}
	gpio_data->state = false;

	return pinctrl_select_state(gpio_data->pinctrl,
				    gpio_data->pinctrl_sleep);
}
EXPORT_SYMBOL(msm_cdc_pinctrl_select_sleep_state);

/*
 * msm_cdc_pinctrl_select_active_state: select pinctrl active state
 * @np: pointer to struct device_node
 *
 * Returns error code for failure
 */
int msm_cdc_pinctrl_select_active_state(struct device_node *np)
{
	struct msm_cdc_pinctrl_info *gpio_data;

	gpio_data = msm_cdc_pinctrl_get_gpiodata(np);
	if (!gpio_data)
		return -EINVAL;

	if (!gpio_data->pinctrl_active) {
		pr_err("%s: pinctrl active state is null\n", __func__);
		return -EINVAL;
	}
	gpio_data->state = true;

	return pinctrl_select_state(gpio_data->pinctrl,
				    gpio_data->pinctrl_active);
}
EXPORT_SYMBOL(msm_cdc_pinctrl_select_active_state);

/*
 * msm_cdc_pinctrl_get_state: get curren pinctrl state
 * @np: pointer to struct device_node
 *
 * Returns 0 for sleep state, 1 for active state
 */
bool msm_cdc_pinctrl_get_state(struct device_node *np)
{
	struct msm_cdc_pinctrl_info *gpio_data;

	gpio_data = msm_cdc_pinctrl_get_gpiodata(np);
	if (!gpio_data)
		return -EINVAL;

	return gpio_data->state;
}
EXPORT_SYMBOL(msm_cdc_pinctrl_get_state);

static int msm_cdc_pinctrl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_cdc_pinctrl_info *gpio_data;

	gpio_data = devm_kzalloc(&pdev->dev,
				 sizeof(struct msm_cdc_pinctrl_info),
				 GFP_KERNEL);
	if (!gpio_data)
		return -ENOMEM;

	gpio_data->pinctrl_supply = devm_regulator_get_optional(&pdev->dev,
								"pinctrl");
	ret = PTR_ERR(gpio_data->pinctrl_supply);
	if (ret == -ENODEV) {
		gpio_data->pinctrl_supply = NULL;
	} else if (ret == -EPROBE_DEFER) {
		dev_dbg(&pdev->dev, "Defer due to pinctrl-supply.");
		goto err_pctrl_get;
	} else if (ret == -ENOENT) {
		dev_err(&pdev->dev,
			"Could not find regulator for pinctrl-supply.");
		goto err_pctrl_get;
	} else if (IS_ERR(gpio_data->pinctrl_supply)) {
		dev_err(&pdev->dev,
			"Unhandled error %d getting pinctrl-supply.",
			ret);
		goto err_pctrl_get;
	}

	if (gpio_data->pinctrl_supply) {
		ret = regulator_enable(gpio_data->pinctrl_supply);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to enable pinctrl-supply regulator: %d",
				ret);
			goto err_pctrl_get;
		}
	}

	gpio_data->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(gpio_data->pinctrl)) {
		dev_err(&pdev->dev, "%s: Cannot get cdc gpio pinctrl:%ld\n",
			__func__, PTR_ERR(gpio_data->pinctrl));
		ret = PTR_ERR(gpio_data->pinctrl);
		goto err_pctrl_get;
	}

	gpio_data->pinctrl_active = pinctrl_lookup_state(
					gpio_data->pinctrl, "aud_active");
	if (IS_ERR_OR_NULL(gpio_data->pinctrl_active)) {
		dev_err(&pdev->dev, "%s: Cannot get aud_active pinctrl state:%ld\n",
			__func__, PTR_ERR(gpio_data->pinctrl_active));
		ret = PTR_ERR(gpio_data->pinctrl_active);
		goto err_lookup_state;
	}

	gpio_data->pinctrl_sleep = pinctrl_lookup_state(
					gpio_data->pinctrl, "aud_sleep");
	if (IS_ERR_OR_NULL(gpio_data->pinctrl_sleep)) {
		dev_err(&pdev->dev, "%s: Cannot get aud_sleep pinctrl state:%ld\n",
			__func__, PTR_ERR(gpio_data->pinctrl_sleep));
		ret = PTR_ERR(gpio_data->pinctrl_sleep);
		goto err_lookup_state;
	}
	/* skip setting to sleep state for LPI_TLMM GPIOs */
	if (!of_property_read_bool(pdev->dev.of_node, "qcom,lpi-gpios")) {
		/* Set pinctrl state to aud_sleep by default */
		ret = pinctrl_select_state(gpio_data->pinctrl,
					   gpio_data->pinctrl_sleep);
		if (ret)
			dev_err(&pdev->dev, "%s: set cdc gpio sleep state fail: %d\n",
				__func__, ret);
	}

	gpio_data->gpio = of_get_named_gpio(pdev->dev.of_node,
					    "qcom,cdc-rst-n-gpio", 0);
	if (gpio_is_valid(gpio_data->gpio)) {
		ret = gpio_request(gpio_data->gpio, "MSM_CDC_RESET");
		if (ret) {
			dev_err(&pdev->dev, "%s: Failed to request gpio %d\n",
				__func__, gpio_data->gpio);
			goto err_lookup_state;
		}
	}

	dev_set_drvdata(&pdev->dev, gpio_data);
	return 0;

err_lookup_state:
	devm_pinctrl_put(gpio_data->pinctrl);
	if (gpio_data->pinctrl_supply)
		regulator_disable(gpio_data->pinctrl_supply);
err_pctrl_get:
	devm_kfree(&pdev->dev, gpio_data);
	return ret;
}

static int msm_cdc_pinctrl_remove(struct platform_device *pdev)
{
	struct msm_cdc_pinctrl_info *gpio_data;

	gpio_data = dev_get_drvdata(&pdev->dev);

	if (gpio_data && gpio_data->pinctrl)
		devm_pinctrl_put(gpio_data->pinctrl);

	if (gpio_data && gpio_data->pinctrl_supply &&
	    regulator_is_enabled(gpio_data->pinctrl_supply))
		regulator_disable(gpio_data->pinctrl_supply);

	devm_kfree(&pdev->dev, gpio_data);

	return 0;
}

#ifdef CONFIG_PM
static int msm_cdc_pinctrl_suspend_late(struct device *dev)
{
	struct msm_cdc_pinctrl_info *gpio_data = dev_get_drvdata(dev);
	int ret = 0;

	if (gpio_data->pinctrl_supply)
		ret = regulator_disable(gpio_data->pinctrl_supply);
	return ret;
}

static int msm_cdc_pinctrl_resume_early(struct device *dev)
{
	struct msm_cdc_pinctrl_info *gpio_data = dev_get_drvdata(dev);
	int ret = 0;

	if (gpio_data->pinctrl_supply) {
		ret = regulator_enable(gpio_data->pinctrl_supply);
		if (ret)
			dev_err(dev, "failed to enable pinctrl-supply: %d", ret);
	}
	return ret;
}
#endif

static const struct dev_pm_ops msm_cdc_pinctrl_pm_ops = {
#ifdef CONFIG_PM
	.suspend_late = msm_cdc_pinctrl_suspend_late,
	.resume_early = msm_cdc_pinctrl_resume_early,
#endif
};

static const struct of_device_id msm_cdc_pinctrl_match[] = {
	{.compatible = "qcom,msm-cdc-pinctrl"},
	{}
};

static struct platform_driver msm_cdc_pinctrl_driver = {
	.driver = {
		.name = "msm-cdc-pinctrl",
		.owner = THIS_MODULE,
		.pm = &msm_cdc_pinctrl_pm_ops,
		.of_match_table = msm_cdc_pinctrl_match,
	},
	.probe = msm_cdc_pinctrl_probe,
	.remove = msm_cdc_pinctrl_remove,
};
module_platform_driver(msm_cdc_pinctrl_driver);

MODULE_DESCRIPTION("MSM CODEC pin control platform driver");
MODULE_LICENSE("GPL v2");

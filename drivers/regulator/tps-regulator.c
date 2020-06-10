// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019-2020 Google, Inc
 *
 * TPS Voltage Regulator for USB and Wireless RTx
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <../gpio/gpiolib.h>

/* TODO: revert this param b/149543749 */
static int delay;
module_param(delay, int, 0644);
MODULE_PARM_DESC(delay, "GPIO delay");

// used for selector, must be aligned to tps_volt_table
enum {
	TPS_CONFIG_5V,
	TPS_CONFIG_7V,
	TPS_CONFIG_MAX,
};

static const unsigned int tps_volt_table[] = { 5000000, 7000000 };

struct tps_config {
	unsigned int nr_control_gpios;
	struct gpio_desc **control_gpio_descs;
	unsigned int nr_gpio_en;
	unsigned int *gpio_enable_sequence;
	unsigned int *gpio_enable_delay;
	unsigned int nr_gpio_dis;
	unsigned int *gpio_disable_sequence;
	unsigned int *gpio_disable_delay;
};

struct regulator_tps_data {
	struct regulator_desc desc;
	struct regulator_dev *dev;
	struct regulator_init_data *init_data;
	int voltage_selector;
	int nr_voltages;
	struct tps_config config[ARRAY_SIZE(tps_volt_table)];

	struct gpio_desc *en_gpio_desc;
	unsigned int en_delay;
	unsigned int dis_delay;
	bool en_pin_only;
	bool enabled;
	struct mutex lock;
	const char *supply_name;
};

static int of_get_regulator_tps_config(struct device *dev,
				       struct device_node *np,
				       struct tps_config *config)
{
	int i, ret;

	ret = of_property_read_u32(np, "control-gpio-count",
				   &config->nr_control_gpios);
	if (ret || config->nr_control_gpios <= 0) {
		dev_err(dev, "Incorrect value for control-gpio-count\n");
		return -EINVAL;
	}

	config->control_gpio_descs = devm_kcalloc(dev,
						  config->nr_control_gpios,
						  sizeof(struct gpio_desc *),
						  GFP_KERNEL);
	if (!config->control_gpio_descs) {
		dev_err(dev, "nr_control_gpio allocation failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < config->nr_control_gpios; i++) {
		config->control_gpio_descs[i] = gpiod_get_from_of_node(np,
							"control-gpios",
							i,
							GPIOD_OUT_LOW,
							"control");
		/* TODO: for debug only; remove this export. b/149543749 */
		ret = gpiod_export(config->control_gpio_descs[i], true);
		if (ret)
			dev_err(dev, "couldn't export gpio\n");
	}

	config->nr_gpio_en = of_property_count_elems_of_size(np,
							"gpio-enable-sequence",
							sizeof(u32));
	if (config->nr_gpio_en <= 0) {
		dev_err(dev, "gpio-enable-sequence not set\n");
		return -EINVAL;
	}

	config->gpio_enable_sequence = devm_kcalloc(dev,
						    config->nr_gpio_en,
						    sizeof(unsigned int),
						    GFP_KERNEL);
	if (!config->gpio_enable_sequence) {
		dev_err(dev, "gpio_enable_sequence allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-enable-sequence",
					 config->gpio_enable_sequence,
					 config->nr_gpio_en);
	if (ret) {
		dev_err(dev, "Failed to read gpio-enable-sequence\n");
		return ret;
	}

	ret = of_property_count_elems_of_size(np,
					      "gpio-enable-delay",
					      sizeof(u32));
	if (ret != config->nr_gpio_en) {
		dev_err(dev,
			"gpio-enable-delay should have same length as gpio-enable-sequence\n");
		return -EINVAL;
	}

	config->gpio_enable_delay = devm_kcalloc(dev, config->nr_gpio_en,
						 sizeof(unsigned int),
						 GFP_KERNEL);
	if (!config->gpio_enable_delay) {
		dev_err(dev, "gpio_enable_delay allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-enable-delay",
					 config->gpio_enable_delay,
					 config->nr_gpio_en);
	if (ret) {
		dev_err(dev, "Failed to read gpio-enable-delay\n");
		return ret;
	}

	config->nr_gpio_dis = of_property_count_elems_of_size(np,
						"gpio-disable-sequence",
						sizeof(u32));
	if (config->nr_gpio_dis <= 0) {
		dev_err(dev, "gpio-disable-sequence not set\n");
		return -EINVAL;
	}

	config->gpio_disable_sequence = devm_kcalloc(dev,
						     config->nr_gpio_dis,
						     sizeof(unsigned int),
						     GFP_KERNEL);
	if (!config->gpio_disable_sequence) {
		dev_err(dev, "gpio_disable_sequence allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-disable-sequence",
					 config->gpio_disable_sequence,
					 config->nr_gpio_dis);
	if (ret) {
		dev_err(dev, "Failed to read gpio-disable-sequence\n");
		return ret;
	}

	ret = of_property_count_elems_of_size(np, "gpio-disable-delay",
					      sizeof(u32));
	if (ret != config->nr_gpio_dis) {
		dev_err(dev,
			"gpio-disable-delay should have same length as gpio-disable-sequence\n");
		return -EINVAL;
	}

	config->gpio_disable_delay = devm_kcalloc(dev,
						  config->nr_gpio_dis,
						  sizeof(unsigned int),
						  GFP_KERNEL);
	if (!config->gpio_disable_delay) {
		dev_err(dev, "gpio_disable_delay allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-disable-delay",
					 config->gpio_disable_delay,
					 config->nr_gpio_dis);
	if (ret) {
		dev_err(dev, "Failed to read gpio-disable-delay\n");
		return ret;
	}

	return 0;
}

static int tps_regulator_enable(struct regulator_dev *rdev)
{
	struct regulator_tps_data *rdata;
	struct tps_config *config;
	int i = 0;

	rdata = rdev_get_drvdata(rdev);

	mutex_lock(&rdata->lock);

	if (rdata->enabled)
		goto exit;

	gpiod_set_value_cansleep(rdata->en_gpio_desc, 1);

	if (delay)
		udelay(delay);
	else
		udelay(rdata->en_delay);

	if (rdata->en_pin_only) {
		rdata->enabled = true;
		goto exit;
	}

	config = &rdata->config[rdata->voltage_selector];
	for (i = 0; i < config->nr_gpio_en; i++) {
		gpiod_set_value_cansleep(config->control_gpio_descs[
					 config->gpio_enable_sequence[i] - 1],
					 1);
		udelay(config->gpio_enable_delay[i]);
	}

	rdata->enabled = true;
exit:
	mutex_unlock(&rdata->lock);
	return 0;
}

static int tps_regulator_disable(struct regulator_dev *rdev)
{
	struct regulator_tps_data *rdata;
	struct tps_config *config;
	int i = 0;

	rdata = rdev_get_drvdata(rdev);

	mutex_lock(&rdata->lock);

	if (!rdata->enabled)
		goto exit;

	if (rdata->en_pin_only)
		goto disable;

	config = &rdata->config[rdata->voltage_selector];

	for (i = 0; i < config->nr_gpio_dis; i++) {
		gpiod_set_value_cansleep(config->control_gpio_descs[
					 config->gpio_disable_sequence[i] - 1],
					 0);
		udelay(config->gpio_disable_delay[i]);
	}

disable:
	gpiod_set_value_cansleep(rdata->en_gpio_desc, 0);
	udelay(rdata->dis_delay);

	rdata->enabled = false;
exit:
	mutex_unlock(&rdata->lock);
	return 0;
}

static int tps_regulator_is_enabled(struct regulator_dev *dev)
{
	struct regulator_tps_data *rdata;

	rdata = rdev_get_drvdata(dev);

	return rdata->enabled ? 1 : 0;
}

static int tps_regulator_get_voltage_sel(struct regulator_dev *dev)
{
	struct regulator_tps_data *rdata;

	rdata = rdev_get_drvdata(dev);

	return rdata->voltage_selector;
}

static int tps_regulator_set_voltage_sel(struct regulator_dev *dev,
					 unsigned selector)
{
	struct regulator_tps_data *rdata;

	rdata = rdev_get_drvdata(dev);

	// TODO: verify the value of selector
	rdata->voltage_selector = selector;

	return 0;
}

static struct regulator_ops regulator_tps_ops = {
	.is_enabled = tps_regulator_is_enabled,
	.enable = tps_regulator_enable,
	.disable = tps_regulator_disable,
	.get_voltage_sel = tps_regulator_get_voltage_sel,
	.set_voltage_sel = tps_regulator_set_voltage_sel,
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_ascend,
};

static struct regulator_ops regulator_tps_single_ops = {
	.is_enabled = tps_regulator_is_enabled,
	.enable = tps_regulator_enable,
	.disable = tps_regulator_disable,
};

static int reg_tps_probe(struct platform_device *pdev)
{
	struct regulator_tps_data *rdata;
	struct regulator_config cfg = { };
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	int ret;

	rdata = devm_kzalloc(&pdev->dev, sizeof(struct regulator_tps_data),
			     GFP_KERNEL);
	if (!rdata) {
		dev_err(&pdev->dev, "Failed to alloc rdata\n");
		return -ENOMEM;
	}

	rdata->init_data = of_get_regulator_init_data(&pdev->dev, np,
						      &rdata->desc);
	if (!rdata->init_data)
		return -EINVAL;

	rdata->supply_name = rdata->init_data->constraints.name; // need supply_name?

	of_property_read_u32(np, "startup-delay-us", &rdata->desc.enable_time);

	rdata->en_pin_only = of_property_read_bool(np, "en-pin-only");

	ret = of_property_read_u32(np, "voltage-count", &rdata->nr_voltages);
	if (ret || rdata->nr_voltages > ARRAY_SIZE(tps_volt_table)) {
		dev_err(&pdev->dev, "Incorrect value for voltage-count\n");
		return -EINVAL;
	}

	for_each_child_of_node(np, child) {
		u32 voltage_level;

		if (rdata->en_pin_only)
			break;

		ret = of_property_read_u32(child, "voltage-level-uv", &voltage_level);
		if (ret) {
			dev_err(&pdev->dev, "Couldn't find voltage-level-uv\n");
			return -EINVAL;
		}
		/* TODO: implement a table to match the device tree node */
		if (voltage_level == tps_volt_table[TPS_CONFIG_5V]) {
			ret = of_get_regulator_tps_config(&pdev->dev, child,
						&rdata->config[TPS_CONFIG_5V]);
			if (ret)
				return -EINVAL;
		} else if (voltage_level == tps_volt_table[TPS_CONFIG_7V]) {
			ret = of_get_regulator_tps_config(&pdev->dev, child,
						&rdata->config[TPS_CONFIG_7V]);
			if (ret)
				return -EINVAL;
		}
	}

	/* Default selector set to 5V */
	/* TODO: move the init config to regulator_init_data->regulator_init */
	rdata->voltage_selector = TPS_CONFIG_5V;
	rdata->en_gpio_desc = devm_gpiod_get(&pdev->dev, "en", GPIOD_OUT_LOW);
	ret = of_property_read_u32(np, "en-delay", &rdata->en_delay);
	ret = of_property_read_u32(np, "dis-delay", &rdata->dis_delay);

	rdata->desc.name = devm_kstrdup(&pdev->dev, rdata->supply_name,
					GFP_KERNEL);
	rdata->desc.type = REGULATOR_VOLTAGE;
	rdata->desc.n_voltages = rdata->nr_voltages; // do we need this?
	rdata->desc.owner = THIS_MODULE;
	if (rdata->en_pin_only) {
		rdata->init_data->constraints.apply_uV = 0;
		rdata->desc.fixed_uV = rdata->init_data->constraints.min_uV ?
				       : tps_volt_table[0];
		rdata->desc.ops = &regulator_tps_single_ops;
	} else {
		rdata->desc.volt_table = tps_volt_table;
		rdata->desc.ops = &regulator_tps_ops;
	}

	cfg.dev = &pdev->dev;
	cfg.init_data = rdata->init_data;
	cfg.driver_data = rdata;
	cfg.of_node = pdev->dev.of_node;

	rdata->dev = devm_regulator_register(&pdev->dev, &rdata->desc,
					     &cfg);
	if (IS_ERR(rdata->dev)) {
		ret = PTR_ERR(rdata->dev);
		dev_err(&pdev->dev,
			"Failed to register regulator: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, rdata);

	return 0;
	}

#if defined(CONFIG_OF)
static const struct of_device_id regulator_tps_of_match[] = {
	{ .compatible = "regulator-tps", },
	{},
};
MODULE_DEVICE_TABLE(of, regulator_tps_of_match);
#endif

static struct platform_driver regulator_tps_driver = {
	.probe          = reg_tps_probe,
	.driver         = {
		.name           = "reg-dual-voltage-tps",
		.of_match_table = of_match_ptr(regulator_tps_of_match),
	},
};

static int __init regulator_tps_init(void)
{
	return platform_driver_register(&regulator_tps_driver);
}
subsys_initcall(regulator_tps_init);

static void __exit regulator_tps_exit(void)
{
	platform_driver_unregister(&regulator_tps_driver);
}
module_exit(regulator_tps_exit);

MODULE_AUTHOR("Badhri Jagan Sridharan<badhri@google.com>");
MODULE_AUTHOR("Kyle Tso<kyletso@google.com>");
MODULE_DESCRIPTION("tps voltage regulator");
MODULE_LICENSE("GPL");


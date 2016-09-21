/*
 * Driver for the TCA6418E GPIO Pin controller
 *
 * Copyright (C) 2016 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_fdt.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include "core.h"
#include "pinconf.h"
#include "pinctrl-utils.h"

#define TCA6418_I2C_NAME "tca6418-i2c"

#define TCA6418E_REGBASE_DAT_STAT          (0x14)
#define TCA6418E_REGBASE_DAT_OUT           (0x17)
#define TCA6418E_REGBASE_OUTPUT_ENABLE     (0x23)
#define TCA6418E_REGBASE_PULLDOWN_DISABLE  (0x2C)

#define pin_offset(pin) \
	((uint8_t) ((uint8_t) pin / 8))

#define pin_bit(pin) \
	((uint8_t) (pin > 7 ? pin % 8 : 7 - (pin % 8)))

struct devref {
	atomic_t count; /* -1: disabled, 0: unused, 1+: inuse */
	struct device_node *pnode;
	struct i2c_client *client;
	struct devref *next;
};

static struct devref *devrefs = NULL;
static DEFINE_SPINLOCK(devrefs_lock);

static struct devref *devref_get(struct devref *dref)
{
	return atomic_add_unless(&dref->count, 1, -1) ?
		dref : NULL;
}

static void devref_put(struct devref *dref)
{
	atomic_dec(&dref->count);
}

static struct devref *find_devref_by_node(struct device_node *pnode)
{
	unsigned long flags;
	struct devref *dref;

	spin_lock_irqsave(&devrefs_lock, flags);
	dref = devrefs;
	spin_unlock_irqrestore(&devrefs_lock, flags);

	for (; dref != NULL; dref = dref->next)
		if (dref->pnode == pnode)
			return dref;

	return NULL;
}

static struct devref *register_devref(struct i2c_client *client,
		struct device_node *pnode)
{
	struct devref *dref;

	dref = find_devref_by_node(pnode);

	if (!dref) {
		unsigned long flags;

		dref = devm_kmalloc(&client->dev, sizeof(*dref), GFP_KERNEL);
		if (!dref)
			return ERR_PTR(-ENOMEM);

		dref->pnode = pnode;
		atomic_set(&dref->count, -1);

		spin_lock_irqsave(&devrefs_lock, flags);
		dref->next = devrefs;
		devrefs = dref;
		spin_unlock_irqrestore(&devrefs_lock, flags);
	}

	dref->client = client;
	atomic_set(&dref->count, 0);
	return dref;
}

static void unregister_devref(struct devref *dref)
{
	while (atomic_cmpxchg(&dref->count, 0, -1))
		schedule_timeout(msecs_to_jiffies(200));

	/* The devref shall be re-used next time the same i2c client
	 * is probed, so not going to free it nor remove it from list.
	 */
}

/*
 * tca6418: Accessing GPIO registers
 */
static int tca6418_writeb(struct i2c_client *client, uint8_t addr, uint8_t *buf)
{
	int ret;
	uint8_t out_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = out_buf,
		}
	};

	out_buf[0] = addr;
	out_buf[1] = *buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EAGAIN;
	return 0;
}

static int tca6418_readb(struct i2c_client *client, uint8_t addr, uint8_t *buf)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(addr),
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = buf,
		}
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EAGAIN;
	return 0;
}

struct tca6418_data {
	struct gpio_chip gpioc;
	struct pinctrl_dev *pctrl;
	struct devref *devref;
	struct mutex reglock;
};

static const struct pinctrl_pin_desc tca6418_pins[] = {
	PINCTRL_PIN(0,  "gpio0"),
	PINCTRL_PIN(1,  "gpio1"),
	PINCTRL_PIN(2,  "gpio2"),
	PINCTRL_PIN(3,  "gpio3"),
	PINCTRL_PIN(4,  "gpio4"),
	PINCTRL_PIN(5,  "gpio5"),
	PINCTRL_PIN(6,  "gpio6"),
	PINCTRL_PIN(7,  "gpio7"),
	PINCTRL_PIN(8,  "gpio8"),
	PINCTRL_PIN(9,  "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"),
	PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"),
	PINCTRL_PIN(17, "gpio17"),
};

static int tca6418_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *tca6418_get_group_name(struct pinctrl_dev *pctldev,
				      unsigned group)
{
	return "n/a";
}

static const struct pinctrl_ops tca6418_pinctrl_ops = {
	.get_groups_count	= tca6418_get_groups_count,
	.get_group_name		= tca6418_get_group_name,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_pin,
	.dt_free_map		= pinctrl_utils_dt_free_map,
};

static int tca6418_reg_bitop(struct tca6418_data *pdata, uint8_t regbase,
		int pin, int bitop_set)
{
	int ret;
	uint8_t rdata = 0;
	uint8_t reg = regbase + pin_offset(pin);
	int bit = pin_bit(pin);

	struct devref *dref = devref_get(pdata->devref);

	if (!dref)
		return -EAGAIN;

	mutex_lock(&pdata->reglock);

	ret = tca6418_readb(dref->client, reg, &rdata);
	if (!ret) {
		if (bitop_set)
			rdata |= 1 << bit;
		else
			rdata &= ~(1 << bit);
		ret = tca6418_writeb(dref->client, reg, &rdata);
	}

	mutex_unlock(&pdata->reglock);
	devref_put(dref);

	return ret;
}

static uint8_t tca6418_reg_bit(struct tca6418_data *pdata, uint8_t regbase,
		int pin)
{
	uint8_t rdata = 0;
	uint8_t reg = regbase + pin_offset(pin);
	int bit = pin_bit(pin);

	struct devref *dref = devref_get(pdata->devref);

	if (!dref)
		return -EAGAIN;

	mutex_lock(&pdata->reglock);

	tca6418_readb(dref->client, reg, &rdata);

	mutex_unlock(&pdata->reglock);
	devref_put(dref);

	return (rdata >> bit) & 0x1;
}

static int tca6418_pmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *tca6418_pmux_get_function_name(struct pinctrl_dev *pctldev,
					       unsigned selector)
{
	return "n/a";
}

static int tca6418_pmux_get_function_groups(struct pinctrl_dev *pctldev,
					 unsigned selector,
					 const char * const **groups,
					 unsigned * const num_groups)
{
	return 0;
}

static int tca6418_pinmux_set_mux(struct pinctrl_dev *pctldev,
			       unsigned function,
			       unsigned group)
{
	return -ENOTSUPP;
}

static const struct pinmux_ops tca6418_pinmux_ops = {
	.get_functions_count = tca6418_pmux_get_functions_count,
	.get_function_name = tca6418_pmux_get_function_name,
	.get_function_groups = tca6418_pmux_get_function_groups,
	.set_mux = tca6418_pinmux_set_mux,
};

static int tca6418_pinconf_cfg_get(struct pinctrl_dev *pctldev,
				unsigned pin,
				unsigned long *config)
{
	int pulldown, input_en, output;
	struct tca6418_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);

	pulldown = !(tca6418_reg_bit(pdata,
			TCA6418E_REGBASE_PULLDOWN_DISABLE, pin));
	input_en = !(tca6418_reg_bit(pdata,
			TCA6418E_REGBASE_OUTPUT_ENABLE, pin));
	output = tca6418_reg_bit(pdata,
			TCA6418E_REGBASE_DAT_OUT, pin);

	if (param == PIN_CONFIG_BIAS_PULL_DOWN && pulldown)
		return 0;
	if (param == PIN_CONFIG_BIAS_DISABLE && !pulldown)
		return 0;
	if (param == PIN_CONFIG_INPUT_ENABLE && input_en)
		return 0;
	if (param == PIN_CONFIG_OUTPUT && !input_en) {
		*config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, output);
		return 0;
	}

	return -ENOTSUPP;
}

static int tca6418_pinconf_cfg_set_pin(struct tca6418_data *pdata,
		unsigned pin, enum pin_config_param param, unsigned int arg)
{
	int ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		ret = tca6418_reg_bitop(pdata,
				TCA6418E_REGBASE_PULLDOWN_DISABLE, pin, 1);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		ret = tca6418_reg_bitop(pdata,
				TCA6418E_REGBASE_PULLDOWN_DISABLE, pin, 0);
		break;
	case PIN_CONFIG_OUTPUT:
		ret = tca6418_reg_bitop(pdata,
				TCA6418E_REGBASE_DAT_OUT, pin, arg ? 1 : 0);
		if (ret)
			break;
		arg = 0;
		/* no-break: all output request implies input disable */
	case PIN_CONFIG_INPUT_ENABLE:
		ret = tca6418_reg_bitop(pdata,
				TCA6418E_REGBASE_OUTPUT_ENABLE, pin, arg ? 0 : 1);
		break;
	default:
		return -ENOTSUPP;
	}

	return ret;
}

static int tca6418_pinconf_cfg_set(struct pinctrl_dev *pctldev,
				unsigned pin,
				unsigned long *configs,
				unsigned num_configs)
{
	int i;
	struct tca6418_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	for (i = 0; i < num_configs; i++) {
		int ret;
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		ret = tca6418_pinconf_cfg_set_pin(pdata, pin, param, arg);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops tca6418_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = tca6418_pinconf_cfg_get,
	.pin_config_set = tca6418_pinconf_cfg_set,
	.pin_config_config_dbg_show = pinconf_generic_dump_config,
};

static struct pinctrl_desc tca6418_pinctrl_desc = {
	.name = "tca6418_pinctrl",
	.pins = tca6418_pins,
	.npins = ARRAY_SIZE(tca6418_pins),
	.pctlops = &tca6418_pinctrl_ops,
	.pmxops = &tca6418_pinmux_ops,
	.confops = &tca6418_pinconf_ops,
	.owner = THIS_MODULE,
};

static int tca6418_gpio_direction_input(struct gpio_chip *chip,
		unsigned offset)
{
	struct tca6418_data *pdata = container_of(chip, struct tca6418_data, gpioc);
	int pin = offset;

	return tca6418_reg_bitop(pdata,
			TCA6418E_REGBASE_OUTPUT_ENABLE, pin, 0);
}

static int tca6418_gpio_direction_output(struct gpio_chip *chip,
		unsigned offset, int value)
{
	struct tca6418_data *pdata = container_of(chip, struct tca6418_data, gpioc);
	int pin = offset;

	return tca6418_reg_bitop(pdata,
			TCA6418E_REGBASE_OUTPUT_ENABLE, pin, 1);
}

static int tca6418_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct tca6418_data *pdata = container_of(chip, struct tca6418_data, gpioc);
	int pin = offset;

	return tca6418_reg_bit(pdata,
			TCA6418E_REGBASE_DAT_STAT, pin);
}

static void tca6418_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct tca6418_data *pdata = container_of(chip, struct tca6418_data, gpioc);
	int pin = offset;

	tca6418_reg_bitop(pdata,
			TCA6418E_REGBASE_DAT_OUT, pin, value ? 1 : 0);
}

static void tca6418_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	int i;
	struct tca6418_data *pdata = container_of(chip, struct tca6418_data, gpioc);

	for (i = 0; i < ARRAY_SIZE(tca6418_pins); i++) {
		const struct pinctrl_pin_desc *desc = &tca6418_pins[i];
		int pin = desc->number;

		seq_printf(s,
			"%7.s: input=%d output=%d output_enable=%d pulldown_disable=%d\n",
			desc->name,
			tca6418_reg_bit(pdata,
					TCA6418E_REGBASE_DAT_STAT, pin),
			tca6418_reg_bit(pdata,
					TCA6418E_REGBASE_DAT_OUT, pin),
			tca6418_reg_bit(pdata,
					TCA6418E_REGBASE_OUTPUT_ENABLE, pin),
			tca6418_reg_bit(pdata,
					TCA6418E_REGBASE_PULLDOWN_DISABLE, pin));
	}
}

static struct gpio_chip tca6418_chip = {
	.direction_input  = tca6418_gpio_direction_input,
	.direction_output = tca6418_gpio_direction_output,
	.get              = tca6418_gpio_get,
	.set              = tca6418_gpio_set,
	.dbg_show         = tca6418_gpio_dbg_show,
	.request          = gpiochip_generic_request,
	.free             = gpiochip_generic_free,
};

static int tca6418_pinctrl_probe(struct platform_device *pdev)
{
	struct tca6418_data *pdata;
	int ret;

	dev_info(&pdev->dev, "Probing tca6418 pinctrl driver\n");

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	mutex_init(&pdata->reglock);
	pdata->devref = find_devref_by_node(pdev->dev.of_node);
	if (!pdata->devref) {
		dev_warn(&pdev->dev, "No available devref found, defer probe\n");
		ret = -EPROBE_DEFER;
		goto fail_devref;
	}

	pdata->pctrl = pinctrl_register(&tca6418_pinctrl_desc, &pdev->dev, pdata);
	if (IS_ERR(pdata->pctrl)) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		ret = PTR_ERR(pdata->pctrl);
		goto fail_pctrl;
	}

	memcpy(&pdata->gpioc, &tca6418_chip, sizeof(pdata->gpioc));
	pdata->gpioc.dev = &pdev->dev;
	pdata->gpioc.base = -1;
	pdata->gpioc.ngpio = tca6418_pinctrl_desc.npins;
	pdata->gpioc.label = tca6418_pinctrl_desc.name;
	pdata->gpioc.owner = THIS_MODULE;
	pdata->gpioc.of_node = pdev->dev.of_node;

	ret = gpiochip_add(&pdata->gpioc);
	if (ret) {
		dev_err(&pdev->dev, "Failed register gpiochip\n");
		goto fail_gpioc;
	}

	ret = gpiochip_add_pin_range(&pdata->gpioc,
			dev_name(&pdev->dev),
			0,
			0,
			pdata->gpioc.ngpio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add pin range\n");
		goto fail_gpioc_pins;
	}

	platform_set_drvdata(pdev, pdata);

	dev_info(&pdev->dev, "Probed tca6418 pinctrl driver\n");
	return 0;

fail_gpioc_pins:
	gpiochip_remove(&pdata->gpioc);
fail_gpioc:
	pinctrl_unregister(pdata->pctrl);
fail_pctrl:
fail_devref:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int tca6418_pinctrl_remove(struct platform_device *pdev)
{
	struct tca6418_data *pdata = platform_get_drvdata(pdev);

	gpiochip_remove(&pdata->gpioc);
	pinctrl_unregister(pdata->pctrl);
	devm_kfree(&pdev->dev, pdata);
	return 0;
}

static const struct of_device_id tca6418_pinctrl_of_match[] = {
	{ .compatible = "ti,tca6418-pinctrl", },
	{ },
};

static struct platform_driver tca6418_pinctrl_driver = {
	.driver = {
		.name = "tca6418-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = tca6418_pinctrl_of_match,
	},
	.probe = tca6418_pinctrl_probe,
	.remove = tca6418_pinctrl_remove,
};
module_platform_driver(tca6418_pinctrl_driver);

static int tca6418_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const __be32 *pprop;
	struct device_node *pnode;
	struct devref *dref;

	dev_info(&client->dev, "Probing tca6418 i2c driver\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EINVAL;
	}

	pprop = of_get_property(client->dev.of_node, "pinctrl", NULL);
	if (!pprop) {
		dev_err(&client->dev, "Couldn't get property `pinctrl'\n");
		return -EINVAL;
	}

	pnode = of_find_node_by_phandle(be32_to_cpup(pprop));
	if (!pnode) {
		dev_err(&client->dev, "Couldn't find phandle\n");
		return -EINVAL;
	}

	/* Validation */
	if (client->dev.of_node != of_find_node_by_phandle(
				be32_to_cpup(of_get_property(pnode, "i2c-client", NULL)))) {
		dev_err(&client->dev, "Couldn't get a paired i2c-client\n");
		return -EINVAL;
	}

	dref = register_devref(client, pnode);
	if (IS_ERR(dref)) {
		dev_err(&client->dev, "failed to register_devref: err=%ld\n",
				PTR_ERR(dref));
		return PTR_ERR(dref);
	}

	i2c_set_clientdata(client, dref);

	dev_info(&client->dev, "Probed tca6418 i2c driver\n");
	return 0;
}

static int tca6418_i2c_remove(struct i2c_client *client)
{
	struct devref *dref = i2c_get_clientdata(client);

	BUG_ON(!dref);
	unregister_devref(dref);
	return 0;
}

static const struct i2c_device_id tca6418_i2c_id[] = {
	{ TCA6418_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tca6418_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id tca6418_i2c_match_table[] = {
	{.compatible = "ti,tca6418-i2c" },
	{},
};
#else
#define tca6418_i2c_match_table NULL
#endif /* CONFIG_OF */

static struct i2c_driver tca6418_i2c_driver = {
	.driver = {
		.name		= TCA6418_I2C_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = tca6418_i2c_match_table,
	},
	.probe		= tca6418_i2c_probe,
	.remove		= tca6418_i2c_remove,
	.id_table	= tca6418_i2c_id,
};
module_i2c_driver(tca6418_i2c_driver);

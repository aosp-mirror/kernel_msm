/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Copyright (C) 2011 Broadcom Corporation.
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/of_gpio.h>

/*The reset on pin is the same with reg on*/

int  bt_reg_on_gpio;

/* LDO for BT PA*/
struct regulator *reg_ldo;
static struct rfkill *bt_rfk;
static const char bt_name[] = "brcm_Bluetooth_rfkill";

static int bluetooth_set_power(void *data, bool blocked)
{
	int rc = 0;
	printk(KERN_ERR "bluetooth_set_power set blocked=%d\n", blocked);
	if (!blocked) {
		gpio_direction_output(bt_reg_on_gpio, 0);
		msleep(30);
		gpio_direction_output(bt_reg_on_gpio, 1);
		rc = regulator_enable(reg_ldo);
	} else {
		gpio_direction_output(bt_reg_on_gpio, 0);
		regulator_disable(reg_ldo);
	}
	return 0;
}

static struct rfkill_ops bluetooth_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int bluetooth_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	int len = 0;
	bool default_state = true;  /* off */
	const __be32 *prop;
	u32 low_vol_level;  /*regulator-min-microvolt*/
	u32 high_vol_level; /*regulator-max-microvolt*/

	struct device *bluetooth_power_dev = &pdev->dev;
	printk(KERN_INFO "bluetooth_rfkill_probe\n");
	bt_reg_on_gpio = of_get_named_gpio(bluetooth_power_dev->of_node , "huawei,reg_on_gpio" , 0);
	rc = gpio_is_valid(bt_reg_on_gpio);
	if (!rc) {
		printk(KERN_ERR "gpio port %d is invalid\n", bt_reg_on_gpio);
		return -EINVAL;
	}

	rc = gpio_request(bt_reg_on_gpio, "bt_reset");
	if (rc) {
		gpio_free(bt_reg_on_gpio);
		rc = gpio_request(bt_reg_on_gpio, "bt_reset");
		if(rc) {
			printk(KERN_ERR "can't request gpio port %d", bt_reg_on_gpio);
			return -EINVAL;
		}
	}
	gpio_direction_output(bt_reg_on_gpio, 0);

	/*Get vdd-voltage-level from dts for BT*/
	prop = of_get_property(bluetooth_power_dev->of_node, "vdd-voltage-level", &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		printk(KERN_ERR "can't get vdd-voltage-level\n");
	} else {
		low_vol_level = be32_to_cpup(&prop[0]);
		high_vol_level = be32_to_cpup(&prop[1]);
		printk(KERN_ERR "low_vol_level=%d,high_vol_level=%d\n",low_vol_level,high_vol_level);
	}

	reg_ldo = regulator_get(bluetooth_power_dev, "vdd");
	if(IS_ERR(reg_ldo))
	{
		printk(KERN_ERR "Fialed to get  regulator vdd\n");
		regulator_put(reg_ldo);
		return -EINVAL;
	}

	bluetooth_set_power(NULL, default_state);

	/*Set vdd-voltage-level for BT*/
	if (regulator_count_voltages(reg_ldo) > 0)
	{
		rc = regulator_set_voltage(reg_ldo,low_vol_level, high_vol_level);
		if (rc)
		{
			printk(KERN_ERR "regulator set_vtg failed rc=%d\n", rc);
			regulator_put(reg_ldo);
		}
	}

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&bluetooth_rfkill_ops, NULL);
	if (!bt_rfk) {
		printk(KERN_ERR "rfkill alloc failed.\n");
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;


err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	gpio_free(bt_reg_on_gpio);
	printk(KERN_ERR "bluetooth_rfkill_probe error!\n");
	return rc;
}

static int bluetooth_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	gpio_free(bt_reg_on_gpio);
	regulator_disable(reg_ldo);
	regulator_put(reg_ldo);
	return 0;
}

static const struct of_device_id bluetooth_power_match_table[] = {
	{
		.compatible = "huawei,bluetooth_rfkill",
		.data = NULL,
	},
	{
	},
};
MODULE_DEVICE_TABLE(of, bluetooth_power_match_table);

static struct platform_driver bluetooth_rfkill_driver = {
	.probe = bluetooth_rfkill_probe,
	.remove = bluetooth_rfkill_remove,
	.driver = {
		.name = "huawei,bluetooth_rfkill",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bluetooth_power_match_table),
	},
};

static int __init bluetooth_rfkill_init(void)
{
	int ret;

	printk(KERN_ERR "bluetooth_rfkill_init\n");
	ret = platform_driver_register(&bluetooth_rfkill_driver);
	if (ret)
		printk(KERN_ERR "Fail to register rfkill platform driver\n");
	printk(KERN_INFO "bluetooth_rfkill_init done\n");
	return ret;
}

static void __exit bluetooth_rfkill_exit(void)
{
	printk(KERN_ERR "bluetooth_rfkill_exit\n");
	platform_driver_unregister(&bluetooth_rfkill_driver);
}

module_init(bluetooth_rfkill_init);
module_exit(bluetooth_rfkill_exit);

MODULE_DESCRIPTION("bluetooth rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");

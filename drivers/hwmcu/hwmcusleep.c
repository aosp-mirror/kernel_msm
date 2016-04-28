/* Copyright (c) 2015, Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#define pr_fmt(fmt) "Mcusleep: %s: " fmt, __func__

#include <linux/module.h> /* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

/*mcu sleep info struct*/
struct mcusleep_info
{
	unsigned int ap_status_gpio;
};

static struct mcusleep_info *mcuinfo = NULL;

/**
* mcu sleep module  drv probe
*/
static int mcusleep_probe(struct platform_device *pdev)
{
	if ((NULL == pdev) || (NULL == pdev->dev.of_node))
	{
		pr_err("mcusleep_probe, invalide parameter\n");
		return -EINVAL;
	}

	mcuinfo = kzalloc(sizeof(struct mcusleep_info), GFP_KERNEL);
	if (NULL == mcuinfo)
	{
		pr_err("mcusleep_probe, alloc failed\n");
		return -ENOMEM;
	}

	mcuinfo->ap_status_gpio = of_get_named_gpio(pdev->dev.of_node, "ap_status", 0);
	if (mcuinfo->ap_status_gpio < 0)
	{
		pr_err("mcusleep_probe, ap_status gpio not found\n");
		return -ENODEV;
	}

	if (gpio_request(mcuinfo->ap_status_gpio, "ap_status"))
	{
		pr_err("mcusleep_probe, gpio_request failed, gpio=%d\n", mcuinfo->ap_status_gpio);
		return -ENODEV;
	}

	pr_info("mcu sleep probe, ap status gpio=%d\n", mcuinfo->ap_status_gpio);

	gpio_direction_output(mcuinfo->ap_status_gpio, 1);

	return 0;

}

/**
* mcu sleep module  drv remove
*/
static int mcusleep_remove(struct platform_device *pdev)
{
	gpio_free(mcuinfo->ap_status_gpio);
	if(NULL != mcuinfo)
	{
		kfree(mcuinfo);
		mcuinfo = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM
/**
* mcu pm sleep module  drv resume  donothing
*/
static int mcusleep_resume(struct device *dev)
{
	pr_info("ap_status high level\n");
	gpio_direction_output(mcuinfo->ap_status_gpio, 1);
	return 0;
}

/**
* mcu mcusleep pm drv suspend
*/
static int mcusleep_suspend(struct device *dev)
{
	pr_info("ap_status low level\n");
	gpio_direction_output(mcuinfo->ap_status_gpio, 0);
	return 0;
}

/**
* mcu mcusleep pm drv
*/
static const struct dev_pm_ops mcusleep_pmops = {
	.suspend = mcusleep_suspend,
	.resume = mcusleep_resume,
};

#define MCUSLEEP_PMOPS (&mcusleep_pmops)
#else
#define MCUSLEEP_PMOPS NULL
#endif

/**
* mcu mcusleep_match_table
*/
static struct of_device_id mcusleep_match_table[] =
{
	{.compatible = "mcu,mcusleep"},
	{},
};

/**
* mcu platform_driver struct
*/
static struct platform_driver mcusleep_driver =
{
	.probe = mcusleep_probe,
	.remove = mcusleep_remove,
	.driver = {
		.name = "mcusleep",
		.owner = THIS_MODULE,
		.of_match_table = mcusleep_match_table,
		.pm = MCUSLEEP_PMOPS,
	},
};

/**
* Initializes the module.
* @return On success, 0. On error, -1, and <code>errno</code> is set
* appropriately.
*/
static int __init mcusleep_init(void)
{
	int retval;

	retval = platform_driver_register(&mcusleep_driver);

	return retval;
}

/**
* Cleans up the module.
*/
static void __exit mcusleep_exit(void)
{
	if (NULL != mcuinfo)
	{
		kfree(mcuinfo);
		mcuinfo = NULL;
    }

	/*deregister dev*/
	platform_driver_unregister(&mcusleep_driver);
}

module_init(mcusleep_init);
module_exit(mcusleep_exit);

MODULE_DESCRIPTION("Mcu Sleep Mode Driver");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

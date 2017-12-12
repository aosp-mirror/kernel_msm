/*
 * Copyright (C) 2017 LGE, Inc.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define BT_ADDR_PROP_NAME "linux,bt-dev-address"

static char bdaddress[18];
module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IRUSR | S_IRGRP);
MODULE_PARM_DESC(bdaddress, "bluetooth device address");

static struct of_device_id bt_addr_match_table[] = {
	{.compatible = "lge,bt_addr"},
	{}
};

static int bt_addr_probe(struct platform_device *pdev)
{
	const char *str;

	if (!of_chosen) {
		pr_err("%s: No chosen node!\n", __func__);
		return -ENOENT;
	}

	if (of_property_read_string(of_chosen, BT_ADDR_PROP_NAME, &str)) {
		pr_err("%s: No %s in device tree\n", __func__,
				BT_ADDR_PROP_NAME);
		return -ENOENT;
	}
	strlcpy(bdaddress, str, sizeof(bdaddress));
	bdaddress[17] = '\0';

	return 0;
}

static struct platform_driver bt_addr_platform_driver = {
	.probe = bt_addr_probe,
	.driver = {
		.name = "bt_addr",
		.owner = THIS_MODULE,
		.of_match_table = bt_addr_match_table,
	},
};

static int __init bt_addr_init(void)
{
	return platform_driver_register(&bt_addr_platform_driver);
}

static void __exit bt_addr_exit(void)
{
	platform_driver_unregister(&bt_addr_platform_driver);
}

module_init(bt_addr_init);
module_exit(bt_addr_exit);
MODULE_DESCRIPTION("BT device address populate");
MODULE_AUTHOR("LGE");
MODULE_LICENSE("GPL");

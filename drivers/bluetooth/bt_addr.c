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
#include <linux/random.h>
#include <linux/time.h>

#define BT_ADDR_PROP_NAME "linux,bt-dev-address"

static char bdaddress[18];
module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IRUSR | S_IRGRP);
MODULE_PARM_DESC(bdaddress, "bluetooth device address");

static struct of_device_id bt_addr_match_table[] = {
	{.compatible = "lge,bt_addr"},
	{}
};

static void get_random_addr(char *buf)
{
	unsigned char addrs[6] = {0x00, 0x90, 0x4c, 0, 0, 0};
	uint rand_addr;

	prandom_seed((uint)ktime_get_ns());
	rand_addr = prandom_u32();
	addrs[3] = (unsigned char)rand_addr;
	addrs[4] = (unsigned char)(rand_addr >> 8);
	addrs[5] = (unsigned char)(rand_addr >> 16);

	sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
			addrs[0], addrs[1], addrs[2],
			addrs[3], addrs[4], addrs[5]);
	buf[17] = '\0';
	WARN(1, "%s: Random BT DEV ADDRESS %s\n", __func__, buf);
}

static int bt_addr_probe(struct platform_device *pdev)
{
	const char *str;
	bool allow_random_addr = false;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	allow_random_addr = of_property_read_bool(np, "allow-random-addr");

	if (!of_chosen) {
		pr_err("%s: No chosen node!\n", __func__);
		ret = -ENOENT;
		goto out;
	}

	if (of_property_read_string(of_chosen, BT_ADDR_PROP_NAME, &str)) {
		pr_err("%s: No %s in device tree\n", __func__,
				BT_ADDR_PROP_NAME);
		ret = -ENOENT;
		goto out;
	}

	strlcpy(bdaddress, str, sizeof(bdaddress));
	bdaddress[17] = '\0';

	return 0;

out:
	if (allow_random_addr) {
		get_random_addr(bdaddress);
		return 0;
	}
	return ret;
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

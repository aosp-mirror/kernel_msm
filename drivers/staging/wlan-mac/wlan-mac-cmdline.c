/*
 * Copyright (c) 2017, Google, Inc. All rights reserved.
 * Author: muirj@
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <net/cnss_utils.h>

/* MAC addresses are 6 bytes. */
#define MAC_ADDR_LEN 6
/* We can have two MAC addresses specified on the command-line. */
#define NUM_MAC_ADDRS 2

static const char *k_wifi_mac_property_names[NUM_MAC_ADDRS] = {
	"goog,wlan-mac-0-cmdline-arg",
	"goog,wlan-mac-1-cmdline-arg"
};
static const char *g_wifi_mac_cmdline_arg[NUM_MAC_ADDRS] = {0};
static u8 g_mac_addrs[NUM_MAC_ADDRS * MAC_ADDR_LEN] = {0};
static int g_mac_addr_set[NUM_MAC_ADDRS] = {0};

static int wlan_mac_cmdline_param_cb(char *param, char *val,
				     const char *unused, void *arg)
{
	int index;
	u8 *mac;

	for (index = 0; index < NUM_MAC_ADDRS; index++)
		if (g_wifi_mac_cmdline_arg[index] &&
		    !strcmp(param, g_wifi_mac_cmdline_arg[index]))
			break;
	if (index == NUM_MAC_ADDRS) {
		/* No string match, so skip this parameter (continue to
		 * the next. */
		return 0;
	}

	mac = g_mac_addrs + (index * MAC_ADDR_LEN);
	if (sscanf(val, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
		   mac, mac + 1, mac + 2, mac + 3, mac + 4, mac + 5) !=
	    MAC_ADDR_LEN)
		return -EINVAL;
	g_mac_addr_set[index] = 1;
	return 0;
}

static void wlan_mac_cmdline_setup(void)
{
	struct device_node *board_info_node;
	char *cmdline;
	char *parse_ret;
	int ret;
	int i;
	int max_index = 0;

	board_info_node = of_find_node_by_path("/board-info");
	if (!board_info_node) {
		pr_err("wlan-mac-cmdline: no board-info found\n");
		return;
	}

	for (i = 0; i < NUM_MAC_ADDRS; i++) {
		ret = of_property_read_string(
				board_info_node,
				k_wifi_mac_property_names[i],
				&g_wifi_mac_cmdline_arg[i]);
		if (i == 0 && ret) {
			/* The first MAC address must be specified. */
			pr_err("wlan-mac-cmdline: missing %s\n",
			       k_wifi_mac_property_names[i]);
			return;
		}
	}

	cmdline = kstrdup(saved_command_line, GFP_KERNEL);
	parse_ret = parse_args("wlan-mac-cmdline", cmdline, NULL,
			       0, 0, 0, NULL, &wlan_mac_cmdline_param_cb);
	kfree(cmdline);
	if (IS_ERR(parse_ret))
		return;
	for (i = 0; i < NUM_MAC_ADDRS; i++) {
		if (g_mac_addr_set[i]) {
			max_index = i;
		} else if(g_wifi_mac_cmdline_arg[i]) {
			pr_err("wlan-mac-cmdline: missing '%s'\n",
			       k_wifi_mac_property_names[i]);
			return;
		}
	}

	if (cnss_utils_set_wlan_mac_address(
			g_mac_addrs, MAC_ADDR_LEN * (max_index + 1)) != 0)
		pr_err("wlan-mac-cmdline: set wlan mac address failed\n");
	else
		pr_info("wlan-mac-cmdline: done setting address(es)\n");
}

static int __init wlan_mac_cmdline_init(void)
{
	wlan_mac_cmdline_setup();

	return 0;
}

late_initcall(wlan_mac_cmdline_init);

MODULE_AUTHOR("Google, Inc.");
MODULE_DESCRIPTION("Set WLAN MAC addresses from commandline parameters.");
MODULE_LICENSE("GPL v2");

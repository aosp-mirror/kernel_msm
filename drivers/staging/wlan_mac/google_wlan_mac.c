/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <asm/setup.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <net/cnss_utils.h>
#include <soc/qcom/icnss.h>

#define CDB_PATH "/chosen/cdt/cdb2"
#define WIFI_MAC_1 "wlan_mac1"
#define WIFI_MAC_2 "wlan_mac2"

static void set_wifi_mac(void)
{
	u8 mac[12] = {0};
	unsigned int i, size;
	unsigned char *mac_addr = NULL;
	struct device_node *node;
	unsigned int mac_found = 0;

	node = of_find_node_by_path(CDB_PATH);
	if (!node)
		pr_err("[WLAN] CDB Node not created under %s", CDB_PATH);
	else
		mac_addr = (unsigned char *)
				of_get_property(node, WIFI_MAC_1, &size);

	if (mac_addr) {
		if (sscanf(mac_addr,
			   "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
			   &mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
			   &mac[5]) == 6) {
			mac_found = 1;
		} else if (sscanf(mac_addr,
				  "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
				  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
				  &mac[5]) == 6) {
			mac_found = 1;
		}
	}

	if (!mac_found) {
		pr_err("[WLAN] No Default MAC !!!!! Use Random Mac\n");

		get_random_bytes(mac, 6);

		/*To avoid invalid mac, set mac[0] and mac[1]*/
		mac[0] = 0x00;
		mac[1] = 0x88;
	}

	/* Now derive the second mac address by flipping the locally administred bit */
	/* TODO(arabawy) We should use the second MAC Address once driver is ready */

	mac[6] = mac[0] | 2;
	for (i = 1;  i < 6; i++) {
		mac[i + 6] = mac[i];
	}

	if (cnss_utils_set_wlan_mac_address(mac, sizeof(mac)) != 0) {
		pr_err("[WLAN] set wlan mac address failed\n");
	}
}

static int __init wifi_mac_init(void)
{
	set_wifi_mac();

	return 0;
}

late_initcall(wifi_mac_init);

MODULE_AUTHOR("Google");
MODULE_DESCRIPTION("Google set wlan mac");
MODULE_LICENSE("GPL v2");


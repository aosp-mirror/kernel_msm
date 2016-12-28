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
#include <soc/qcom/icnss.h>

#define WLAN_MAC_OFFSET 0x40
#define CALIBRATION_DATA_PATH "/calibration_data"
#define WIFI_FLASH_DATA "wifi_eeprom"

static void set_wifi_mac(void)
{
	u8 mac[12] = {0};
	unsigned int i, p_size;
	unsigned char *wifi_nvs_ram = NULL;
	struct device_node *offset;

	offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	if (offset)
		wifi_nvs_ram = (unsigned char *)
			of_get_property(offset, WIFI_FLASH_DATA, &p_size);

	/*Intf0MacAddress*/
	if (!wifi_nvs_ram ||
	    sscanf((const char *)(wifi_nvs_ram + WLAN_MAC_OFFSET),
		   "macaddr=%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
		   &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
		pr_err("[WLAN] No Default MAC !!!!! Use Random Mac\n");
		get_random_bytes(mac, 6);
		/*To avoid invalid mac, set mac[0] and mac[1]*/
		mac[0] = 0x00;
		mac[1] = 0x88;
	}

	/*Intf1MacAddress*/
	mac[6] = mac[0] | 2;
	for (i = 1;  i < 6; i++)
		mac[i + 6] = mac[i];

	if (icnss_set_wlan_mac_address(mac, sizeof(mac)) != 0)
		pr_err("[WLAN] set wlan mac address failed\n");
}

static int __init wifi_nvs_init(void)
{
	set_wifi_mac();

	return 0;
}

late_initcall(wifi_nvs_init);

MODULE_AUTHOR("hTC, Inc.");
MODULE_DESCRIPTION("hTC set wlan mac");
MODULE_LICENSE("GPL v2");

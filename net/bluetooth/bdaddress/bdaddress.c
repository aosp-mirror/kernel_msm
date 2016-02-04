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

/* net/bluetooth/bdaddress.c
 *
 * Code to extract Bluetooth bd_address information
 * from ATAG set up by the bootloader.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include <linux/of.h>

#define ATAG_BT_DEBUG

/* configuration tags specific to Bluetooth*/
#define MAX_BT_SIZE 0x8U

#define CALIBRATION_DATA_PATH "/calibration_data"
#define BT_FLASH_DATA "bt_flash"

static unsigned char bt_bd_ram[MAX_BT_SIZE];
static char bdaddress[20];

static unsigned char *get_bt_bd_ram(void)
{
	struct device_node *offset =
		of_find_node_by_path(CALIBRATION_DATA_PATH);
	int p_size;
	unsigned char *p_data;
#ifdef ATAG_BT_DEBUG
	unsigned int i;
#endif

	p_size = 0;

	if (offset == NULL)
		goto no_data;

	p_data = (unsigned char *)of_get_property(offset, BT_FLASH_DATA, &p_size);

	if (p_data == NULL)
		goto no_data;
	if (p_size > MAX_BT_SIZE)
		goto no_data;
#ifdef ATAG_BT_DEBUG
	for (i = 0; i < p_size; i++)
		pr_debug("%02x ", p_data[i]);
	pr_debug("\n");
#endif
	memcpy(bt_bd_ram, p_data, p_size);
no_data:
	return bt_bd_ram;
}

void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	snprintf(bdaddress, sizeof(bdaddress), "%02x:%02x:%02x:%02x:%02x:%02x",
			cTemp[0], cTemp[1], cTemp[2],
			cTemp[3], cTemp[4], cTemp[5]);

	pr_debug(KERN_INFO "BD_ADDRESS=%s\n", bdaddress);
}

static int __init bdaddress_init(void)
{
	bt_export_bd_address();
	return 0;
}

module_init(bdaddress_init);

module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT DEVICE ADDRESS");


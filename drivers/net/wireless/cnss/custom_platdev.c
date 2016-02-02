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

/* drivers/net/wireless/cnss/custom_pltdev.c
 *
 * Code to extract WiFi calibration information from ATAG set up
 * by the bootloader.
 *
 */

#include <asm/setup.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "../../../fs/proc/internal.h"

#define NVS_MAX_SIZE	0x800U
#define NVS_DATA_OFFSET	0x40
#define WLAN_COUNTRY_OFFSET	0x24
#define WLAN_COUNTRY_SIZE	0x4
#define MAC_TEMPLATE	"Intf0MacAddress=112233445566\n"
#define MAC_END	"END\n"
#define CALIBRATION_DATA_PATH "/calibration_data"
#define WIFI_FLASH_DATA "wifi_eeprom"

static unsigned char wifi_nvs_ram[NVS_MAX_SIZE];
static struct proc_dir_entry *wifi_mac;
static struct proc_dir_entry *wifi_country;

unsigned char *get_wifi_nvs_ram(void)
{
	struct device_node *offset =
		of_find_node_by_path(CALIBRATION_DATA_PATH);
	int p_size;
	unsigned char *p_data;
#ifdef MSM_WIFI_DEBUG
	unsigned int i;
#endif

	p_size = 0;
	if (offset == NULL)
		goto no_data;
	p_data = (unsigned char *)
		of_get_property(offset, WIFI_FLASH_DATA, &p_size);
	if (p_data == NULL)
		goto no_data;
	if (p_size > NVS_MAX_SIZE)
		goto no_data;
#ifdef MSM_WIFI_DEBUG
	for (i = 0; i < p_size; i++)
		pr_debug("%02x ", p_data[i]);
	pr_debug("\n");
#endif
	memcpy(wifi_nvs_ram, p_data, p_size);
no_data:
	return wifi_nvs_ram;
}
EXPORT_SYMBOL(get_wifi_nvs_ram);

static ssize_t wifi_mac_read_proc
	(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char *ptr;
	unsigned int count = size;
	unsigned long p = *ppos;
	const char *src;
	int ret = 0;
	u8 mac[6] = { 0 };
	static unsigned int mlen;
	static int read_mac;
	/* data format:
	* Intf0MacAddress=00AA00BB00CC
	* Intf1MacAddress=00AA00BB00CD
	* END
	*/
	static char mbuf[2*sizeof(MAC_TEMPLATE)+sizeof(MAC_END)];

	if (!read_mac) {
		ptr = get_wifi_nvs_ram();
		src = (const char *)(ptr+NVS_DATA_OFFSET);
		if (ptr == NULL || sscanf(src, "macaddr=%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
					&mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
			pr_debug("[WLAN] No Default MAC !!!!! Use Random Mac ");
			get_random_bytes(mac, 6);
			/*To avoid invalid mac, set mac[0] and mac[1]*/
			mac[0] = 0x00;
			mac[1] = 0x88;
		}
		mlen += scnprintf(mbuf+mlen, sizeof(MAC_TEMPLATE),
			"Intf0MacAddress=%02X%02X%02X%02X%02X%02X\n",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		mlen += scnprintf(mbuf+mlen, sizeof(MAC_TEMPLATE),
			"Intf1MacAddress=%02X%02X%02X%02X%02X%02X\n",
			mac[0]|2, mac[1], mac[2], mac[3], mac[4], mac[5]);
		mlen += scnprintf(mbuf+mlen, sizeof(MAC_END), "END\n");
		read_mac = 1;
	}

	if (p >= mlen)
		return 0;

	if (count > mlen - p)
		count = mlen - p;

	if (copy_to_user(buf, (void *)(mbuf+p), count)) {
		ret =  -EFAULT;
		pr_debug("wifi_calibration_read_proc: FAIL to copy\n");
	} else {
		*ppos += count;
		ret = count;
	}

	return ret;
}

static ssize_t wifi_country_read_proc
(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char *ptr;
	unsigned int count = size;
	unsigned long p = *ppos;
	int ret = 0;

	if (p >= WLAN_COUNTRY_SIZE)
		return 0;

	if (count > WLAN_COUNTRY_SIZE - p)
		count = WLAN_COUNTRY_SIZE - p;

	ptr = get_wifi_nvs_ram();

	if (ptr != NULL) {
		if (copy_to_user(buf, (void *)
		   (ptr+WLAN_COUNTRY_OFFSET+p), count)) {
			ret =  -EFAULT;
			pr_debug("wifi_data_read_proc: FAIL to copy\n");
		} else {
			*ppos += count;
			ret = count;
		}
	} else {
		ret =  -EFAULT;
		pr_debug("wifi_data_read_proc: ptr is null\n");
	}

	return ret;
}

static const struct file_operations wifi_mac_fops = {
	.write      = NULL,
	.read       = wifi_mac_read_proc,
};

static const struct file_operations wifi_country_fops = {
	.write      = NULL,
	.read       = wifi_country_read_proc,
};

static int __init wifi_nvs_init(void)
{
	wifi_mac = proc_create_data("wifi_mac", 0444, NULL,
					&wifi_mac_fops, NULL);
	if (wifi_mac == NULL)
		pr_debug("%s: unable to create /proc/wifi_mac entry\n",
		 __func__);
	else
		wifi_mac->size = 2*sizeof(MAC_TEMPLATE)+sizeof(MAC_END);

	wifi_country = proc_create_data("wifi_country", 0444, NULL,
			&wifi_country_fops, NULL);
	if (wifi_country == NULL)
		pr_debug("%s: unable to create /proc/wifi_country entry\n"
		, __func__);
	else
		wifi_country->size = WLAN_COUNTRY_SIZE;

	return 0;
}

late_initcall(wifi_nvs_init);


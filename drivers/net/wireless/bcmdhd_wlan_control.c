/*
 * Copyright (C) 2017 LGE, Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>

#define WLAN_MAC_ADDRS_FILE    "mac_addrs.bin"
#define ETHER_ADDR_LEN         6

#define WLAN_STATIC_SCAN_BUF0           5
#define WLAN_STATIC_SCAN_BUF1           6
#define PREALLOC_WLAN_SEC_NUM           4
#define PREALLOC_WLAN_BUF_NUM           160
#define PREALLOC_WLAN_SECTION_HEADER    24

#define WLAN_SECTION_SIZE_0     (PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1     (PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2     (PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3     (PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE         336
#define DHD_SKB_1PAGE_BUFSIZE   ((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE   ((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE   ((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM        17

#define WLAN_SCAN_BUF_SIZE      65536

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

#define COUNTRY_BUF_SZ          4
struct custom_locales {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char locale[COUNTRY_BUF_SZ];
	int  locale_rev;
};

struct bcm_wlan_control {
	struct device *dev;
	int gpio_power;
	int gpio_hostwake;
	int power_enabled;
	struct custom_locales *custom_locales;
	int custom_locales_size;
};

static struct bcm_wlan_control *wlan_ctrl = NULL;

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

static void *wlan_static_scan_buf0 = NULL;
static void *wlan_static_scan_buf1 = NULL;

static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;

	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int bcm_init_wlan_mem(void)
{
	int i, num;

	for (i = 0; i < WLAN_SKB_BUF_NUM; i++)
		wlan_static_skb[i] = NULL;

	num = (WLAN_SKB_BUF_NUM - 1) >> 1;
	for (i = 0; i < num; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	num = WLAN_SKB_BUF_NUM -1;
	for (; i < num; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM; i++) {
		wlan_mem_array[i].mem_ptr =
			kmalloc(wlan_mem_array[i].size, GFP_KERNEL);
		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}

	wlan_static_scan_buf0 = kmalloc(WLAN_SCAN_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf0)
		goto err_mem_alloc;

	wlan_static_scan_buf1 = kmalloc(WLAN_SCAN_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf1)
		goto err_static_scan_buf;

	pr_info("%s: WIFI: MEM is pre-allocated\n", __func__);
	return 0;

err_static_scan_buf:
	pr_err("%s: failed to allocate scan_buf0\n", __func__);
	kfree(wlan_static_scan_buf0);
	wlan_static_scan_buf0 = NULL;

err_mem_alloc:
	pr_err("%s: failed to allocate mem_alloc\n", __func__);
	for (i -= 1; i >= 0; i--) {
		kfree(wlan_mem_array[i].mem_ptr);
		wlan_mem_array[i].mem_ptr = NULL;
	}

	i = WLAN_SKB_BUF_NUM;
err_skb_alloc:
	pr_err("%s: failed to allocate skb_alloc\n", __func__);
	for (i -= 1; i >= 0 ; i--) {
		dev_kfree_skb(wlan_static_skb[i]);
		wlan_static_skb[i] = NULL;
	}

	return -ENOMEM;
}

static int bcm_wifi_set_power(int enable)
{
	struct bcm_wlan_control *ctrl = wlan_ctrl;

	if (!ctrl)
		return -ENODEV;

	enable = !!enable;
	if (!(ctrl->power_enabled ^ enable))
		return 0;

	if (enable) {
		gpio_set_value(ctrl->gpio_power, 1);
		ctrl->power_enabled = 1;

		/* WLAN chip to reset */
		msleep(150);
		pr_info("%s: WIFI ON\n", __func__);
	} else {
		gpio_set_value(ctrl->gpio_power, 0);
		ctrl->power_enabled = 0;

		/* WLAN chip down */
		msleep(100);
		pr_info("%s: WiFi OFF\n", __func__);
	}

	return 0;
}

static int bcm_wifi_reset(int on)
{
	return 0;
}

static int bcm_wifi_carddetect(int val)
{
	return 0;
}

static int bcm_wifi_get_mac_addr(unsigned char *buf)
{
	struct bcm_wlan_control *ctrl = wlan_ctrl;
	char *file = WLAN_MAC_ADDRS_FILE;
	const struct firmware *firmware;
	uint rand_mac;
	static unsigned char mymac[ETHER_ADDR_LEN] = {0,};
	const unsigned char nullmac[ETHER_ADDR_LEN] = {0,};
	int ret = 0;

	if (!ctrl || !buf)
		return -EAGAIN;

	memset(buf, 0x00, ETHER_ADDR_LEN);

	ret = request_firmware(&firmware, file, ctrl->dev);
	if (ret) {
		pr_err("%s: Failed to request firmware %s\n", __func__,
				file);
		goto random_mac;
	}

	if (firmware->size < ETHER_ADDR_LEN) {
		pr_err("%s: %s file too short, %zu bytes\n", __func__,
				file, firmware->size);
		release_firmware(firmware);
		goto random_mac;
	}

	memcpy(buf, &firmware->data[0], ETHER_ADDR_LEN);
	pr_info("%s: MAC ADDRESS %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	release_firmware(firmware);
	return ret;

random_mac:
	if (memcmp(mymac, nullmac, ETHER_ADDR_LEN) != 0) {
		/* Mac displayed from UI is never updated..
		   So, mac obtained on initial time is used */
		memcpy(buf, mymac, ETHER_ADDR_LEN);
		return 0;
	}

	prandom_seed((uint)jiffies);
	rand_mac = prandom_u32();
	buf[0] = 0x00;
	buf[1] = 0x90;
	buf[2] = 0x4c;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);

	memcpy(mymac, buf, ETHER_ADDR_LEN);

	WARN(1, "%s; Random MAC ADDRESS %02X:%02X:%02X:%02X:%02X:%02X\n",
			__func__,
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5] );

	return 0;
}

static void *bcm_wifi_get_country_code(char *ccode, u32 flags)
{
	int i;
	struct bcm_wlan_control *ctrl = wlan_ctrl;
	static struct custom_locales country_code;

	if (!ctrl || !ccode)
		return NULL;

	if (!ctrl->custom_locales) {
		pr_err("%s: No custom locales\n", __func__);
		return NULL;
	}

	for (i = 0; i < ctrl->custom_locales_size; i++) {
		if (!strcmp(ccode, ctrl->custom_locales[i].iso_abbrev))
			return &ctrl->custom_locales[i];
	}

	memset(&country_code, 0, sizeof(struct custom_locales));
	strlcpy(country_code.locale, ccode, COUNTRY_BUF_SZ);

	return &country_code;
}

static struct wifi_platform_data bcm_platform_data = {
	.mem_prealloc   = bcm_wifi_mem_prealloc,
	.set_power      = bcm_wifi_set_power,
	.set_reset      = bcm_wifi_reset,
	.set_carddetect = bcm_wifi_carddetect,
	.get_mac_addr   = bcm_wifi_get_mac_addr,
	.get_country_code = bcm_wifi_get_country_code,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name = "bcmdhd_wlan_irq",
		.start = 0,  /* assigned later */
		.end   = 0,  /* assigned later */
		.flags = IORESOURCE_IRQ |
			 IORESOURCE_IRQ_HIGHLEVEL |
			 IORESOURCE_IRQ_SHAREABLE, /* for HW_OOB */
	},
};

static struct platform_device bcm_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(wifi_resource),
	.resource       = wifi_resource,
	.dev            = {
		.platform_data = &bcm_platform_data,
	},
};

static int bcm_wifi_read_country_codes_from_dt(struct bcm_wlan_control *ctrl)
{
	struct device *dev = ctrl->dev;
	struct device_node *np = dev->of_node;
	const char *prop_name = "custom-locales";
	const char **out_strs;
	int i, j, counts;
	int ret;

	counts = of_property_count_strings(np, prop_name);
	if (counts < 0) {
		pr_err("%s: %s not in device tree\n", __func__, prop_name);
		return -EINVAL;
	}

	if (counts % 3) {
		pr_err("%s: the number of locales should be aligned to 3\n",
				__func__);
		return -EINVAL;
	}

	out_strs = devm_kmalloc(dev, sizeof(void *) * counts, GFP_KERNEL);
	if (!out_strs) {
		pr_err("%s: failed to allocate memory for out_strs\n",
				__func__);
		return -ENOMEM;
	}

	ctrl->custom_locales_size = counts / 3;
	ctrl->custom_locales = devm_kzalloc(dev,
			sizeof(struct custom_locales) *
			ctrl->custom_locales_size,
			GFP_KERNEL);
	if (!ctrl->custom_locales) {
		pr_err("%s: no mem\n", __func__);
		return -ENOMEM;
	}

	ret = of_property_read_string_array(np, prop_name, out_strs, counts);
	if (ret < 0) {
		pr_err("%s: failed to read custom locales\n", __func__);
		return ret;
	}

	for (i = 0, j = 0; i < counts; i += 3) {
		struct custom_locales *locale =
			&ctrl->custom_locales[j++];

		strlcpy(locale->iso_abbrev, out_strs[i], COUNTRY_BUF_SZ);
		strlcpy(locale->locale, out_strs[i+1], COUNTRY_BUF_SZ);
		ret = kstrtoint(out_strs[i+2], 0, &locale->locale_rev);
		if (ret < 0) {
			pr_err("%s: locale_rev is invalid(%d %s %s %s)\n",
					__func__, (j - 1),
					out_strs[i],
					out_strs[i+1],
					out_strs[i+2]);
			return ret;
		}
	}

	pr_info("%s: read custom locales %d\n", __func__,
			ctrl->custom_locales_size);
	devm_kfree(dev, out_strs);
	return 0;
}

static int read_and_request_gpio(struct device *dev,
		const char *dt_name, int *gpio,
		unsigned long gpio_flags, const char *label)
{
	struct device_node *np = dev->of_node;
	int rc;

	*gpio = of_get_named_gpio(np, dt_name, 0);
	if (*gpio < 0) {
		pr_err("%s: %s not in device tree\n", __func__, dt_name);
		return -EINVAL;
	}

	if (!gpio_is_valid(*gpio)) {
		pr_err("%s: %s is invalid\n", __func__, dt_name);
		return -EINVAL;
	}

	rc = devm_gpio_request_one(dev, *gpio, gpio_flags, label);
	if (rc < 0) {
		pr_err("%s: failed to request gpio: %d (%s)\n",
				__func__, *gpio, label);
		return rc;
	}

	return 0;
}

static int bcm_wifi_populate_dt(struct bcm_wlan_control *ctrl)
{
	struct device *dev = ctrl->dev;
	int rc;

	rc = read_and_request_gpio(dev, "power-gpio", &ctrl->gpio_power,
			GPIOF_OUT_INIT_HIGH, "wlan-power");
	if (rc < 0)
		return rc;

	ctrl->power_enabled = 1;

	rc = read_and_request_gpio(dev, "host-wake-gpio", &ctrl->gpio_hostwake,
			GPIOF_IN, "wlan_host_wake");
	if (rc < 0)
		return rc;

	rc = bcm_wifi_read_country_codes_from_dt(ctrl);
	if (rc < 0)
		return rc;

	return 0;
}

static int bcm_wifi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm_wlan_control *ctrl;
	int rc = 0;

	ctrl = devm_kzalloc(dev, sizeof(struct bcm_wlan_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = dev;

	rc = bcm_wifi_populate_dt(ctrl);
	if (rc)
		return rc;

	platform_set_drvdata(pdev, ctrl);
	wlan_ctrl = ctrl;

	rc = bcm_init_wlan_mem();
	if (rc)
		return rc;

	/* register wifi platform device */
	wifi_resource[0].start = gpio_to_irq(ctrl->gpio_hostwake);
	wifi_resource[0].end   = gpio_to_irq(ctrl->gpio_hostwake);
	rc = platform_device_register(&bcm_wifi_device);
	if (rc) {
		pr_err("%s: failed to register bcm_wifi_device\n", __func__);
		goto err_bcm_wifi_device;
	}

	pr_info("WLAN control for BCMDHD probed!\n");
	return 0;

err_bcm_wifi_device:
	return rc;
}

static int bcm_wifi_remove(struct platform_device *pdev)
{
	wlan_ctrl = NULL;

	return 0;
}

static struct of_device_id bcm_wifi_match_table[] = {
	{ .compatible = "lge,bcmdhd_wlan" },
	{},
};

static struct platform_driver bcm_wifi_driver = {
	.probe = bcm_wifi_probe,
	.remove = bcm_wifi_remove,
	.driver = {
		.name = "bcmdhd_wlan_control",
		.owner = THIS_MODULE,
		.of_match_table = bcm_wifi_match_table,
	},
};

static int __init bcm_wifi_init(void)
{
	return platform_driver_register(&bcm_wifi_driver);
}

subsys_initcall(bcm_wifi_init);

MODULE_DESCRIPTION("WLAN control for BCMDHD");
MODULE_AUTHOR("LGE");
MODULE_LICENSE("GPL");

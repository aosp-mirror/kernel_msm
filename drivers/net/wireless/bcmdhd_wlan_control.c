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

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
/* dhd.h */
enum dhd_prealloc_index {
	DHD_PREALLOC_PROT = 0,
	DHD_PREALLOC_RXBUF,
	DHD_PREALLOC_DATABUF,
	DHD_PREALLOC_OSL_BUF,
	DHD_PREALLOC_WIPHY_ESCAN0 = 5,
	DHD_PREALLOC_WIPHY_ESCAN1,
	DHD_PREALLOC_DHD_INFO = 7,
	DHD_PREALLOC_MAX_INDEX
};

#define PREALLOC_WLAN_BUF_NUM           160
#define PREALLOC_WLAN_SECTION_HEADER    24

#define DHD_SKB_HDRSIZE         336
#define DHD_SKB_1PAGE_BUFSIZE   ((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE   ((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE   ((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)
#define WLAN_SKB_BUF_NUM        17
#define WLAN_SECTION_SKBUFF_IDX 4

#define WLAN_SECTION_SIZE_0     (PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1     (PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2     (PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3     (PREALLOC_WLAN_BUF_NUM * 1024)
#define WLAN_SECTION_SIZE_4     0 /* static socket buffer */
#define WLAN_SECTION_SIZE_5     65536
#define WLAN_SECTION_SIZE_6     65536
#define WLAN_SECTION_SIZE_7     (16 * 1024)

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[DHD_PREALLOC_MAX_INDEX] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1)},
	{NULL, (WLAN_SECTION_SIZE_2)},
	{NULL, (WLAN_SECTION_SIZE_3)},
	{NULL, (WLAN_SECTION_SIZE_4)},
	{NULL, (WLAN_SECTION_SIZE_5)},
	{NULL, (WLAN_SECTION_SIZE_6)},
	{NULL, (WLAN_SECTION_SIZE_7)},
};
#endif

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
	unsigned char mac_addrs[ETHER_ADDR_LEN];
	bool valid_mac_addrs;
	bool auto_read_mac;
	struct work_struct fw_work;
};

static struct bcm_wlan_control *wlan_ctrl = NULL;

static int bcm_wifi_read_mac_file(struct bcm_wlan_control *ctrl);

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (WLAN_SECTION_SKBUFF_IDX == section)
		return wlan_static_skb;

	if ((section < 0) || (section > DHD_PREALLOC_MAX_INDEX))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int bcm_init_wlan_mem(void)
{
	int i, num;
	size_t total = 0;

	for (i = 0; i < WLAN_SKB_BUF_NUM; i++)
		wlan_static_skb[i] = NULL;

	num = (WLAN_SKB_BUF_NUM - 1) >> 1;
	for (i = 0; i < num; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		total += DHD_SKB_1PAGE_BUFSIZE;
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	num = WLAN_SKB_BUF_NUM -1;
	for (; i < num; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		total += DHD_SKB_2PAGE_BUFSIZE;
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	total += DHD_SKB_4PAGE_BUFSIZE;
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < DHD_PREALLOC_MAX_INDEX; i++) {
		wlan_mem_array[i].mem_ptr =
			kmalloc(wlan_mem_array[i].size, GFP_KERNEL);
		total += wlan_mem_array[i].size;
		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}

	pr_info("BCMDHD: preallocated %u bytes pool\n", total);
	return 0;

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

static void bcm_deinit_wlan_mem(void)
{
	int i;

	for (i = 0; i < DHD_PREALLOC_MAX_INDEX; i++) {
		kfree(wlan_mem_array[i].mem_ptr);
		wlan_mem_array[i].mem_ptr = NULL;
	}

	for (i = 0; i < WLAN_SKB_BUF_NUM; i++) {
		dev_kfree_skb(wlan_static_skb[i]);
		wlan_static_skb[i] = NULL;
	}
}
#endif

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
		pr_info("%s: WiFi ON\n", __func__);
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

	if (!ctrl || !buf)
		return -EAGAIN;

	if (!ctrl->valid_mac_addrs && ctrl->auto_read_mac) {
		if (bcm_wifi_read_mac_file(ctrl))
			return -EAGAIN;
	}

	memcpy(buf, ctrl->mac_addrs, ETHER_ADDR_LEN);
	return 0;
}

static int bcm_wifi_read_mac_file(struct bcm_wlan_control *ctrl)
{
	char *file = WLAN_MAC_ADDRS_FILE;
	const struct firmware *firmware;
	uint rand_mac;
	int ret = 0;

	if (!ctrl)
		return -EAGAIN;

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

	memcpy(ctrl->mac_addrs, &firmware->data[0], ETHER_ADDR_LEN);
	ctrl->valid_mac_addrs = true;

	pr_info("%s: MAC ADDRESS %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
			ctrl->mac_addrs[0],
			ctrl->mac_addrs[1],
			ctrl->mac_addrs[2],
			ctrl->mac_addrs[3],
			ctrl->mac_addrs[4],
			ctrl->mac_addrs[5]);
	ctrl->valid_mac_addrs = true;
	release_firmware(firmware);
	return 0;

random_mac:
	prandom_seed((uint)jiffies);
	rand_mac = prandom_u32();
	ctrl->mac_addrs[0] = 0x00;
	ctrl->mac_addrs[1] = 0x90;
	ctrl->mac_addrs[2] = 0x4c;
	ctrl->mac_addrs[3] = (unsigned char)rand_mac;
	ctrl->mac_addrs[4] = (unsigned char)(rand_mac >> 8);
	ctrl->mac_addrs[5] = (unsigned char)(rand_mac >> 16);
	ctrl->valid_mac_addrs = true;

	WARN(1, "%s; Random MAC ADDRESS %02X:%02X:%02X:%02X:%02X:%02X\n",
			__func__,
			ctrl->mac_addrs[0],
			ctrl->mac_addrs[1],
			ctrl->mac_addrs[2],
			ctrl->mac_addrs[3],
			ctrl->mac_addrs[4],
			ctrl->mac_addrs[5]);
	return 0;
}

static void bcm_wifi_fw_work_func(struct work_struct *work)
{
	struct bcm_wlan_control *ctrl =
		container_of(work, struct bcm_wlan_control, fw_work);

	bcm_wifi_read_mac_file(ctrl);
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
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc   = bcm_wifi_mem_prealloc,
#endif
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

static ssize_t bcm_wifi_read_mac_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct bcm_wlan_control *ctrl = platform_get_drvdata(pdev);
	int enable, rc;

	rc = kstrtoint(buf, 0, &enable);
	if (rc) {
		pr_err("%s: invalid input for read_mac\n", __func__);
		return rc;
	}

	if (enable)
		schedule_work(&ctrl->fw_work);

	return count;
}

static ssize_t bcm_wifi_power_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct bcm_wlan_control *ctrl = platform_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ctrl->power_enabled);
}

static ssize_t bcm_wifi_power_on_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct bcm_wlan_control *ctrl = platform_get_drvdata(pdev);
	int enable, rc;

	rc = kstrtoint(buf, 10, &enable);
	if (rc) {
		pr_err("%s: invalid input for power_on\n", __func__);
		return rc;
	}

	enable = !!enable;
	if (enable == ctrl->power_enabled) {
		pr_debug("%s: already WiFi %s\n", __func__,
				enable? "ON" : "OFF");
		return count;
	}

	gpio_set_value(ctrl->gpio_power, enable);
	ctrl->power_enabled = enable;
	pr_info("%s: WiFi %s\n", __func__, enable? "ON" : "OFF");

	return count;
}

static DEVICE_ATTR(read_mac, 0440, NULL, bcm_wifi_read_mac_store);
static DEVICE_ATTR(power_on, 0660,
		bcm_wifi_power_on_show, bcm_wifi_power_on_store);

static struct attribute *bcm_wifi_attrs[] = {
	&dev_attr_read_mac.attr,
	&dev_attr_power_on.attr,
	NULL
};

static struct attribute_group bcm_wifi_attr_group = {
	.attrs = bcm_wifi_attrs,
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

	pr_info("%s: read %d custom locales\n", __func__,
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
	struct device_node *np = dev->of_node;
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

	ctrl->auto_read_mac = of_property_read_bool(np, "auto-read-mac");

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

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	rc = bcm_init_wlan_mem();
	if (rc)
		return rc;
#endif

	INIT_WORK(&ctrl->fw_work, bcm_wifi_fw_work_func);
	rc = sysfs_create_group(&dev->kobj, &bcm_wifi_attr_group);
	if (rc) {
		pr_err("%s: failed to create sysfs\n", __func__);
		goto err_sysfs_create;
	}

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

err_sysfs_create:
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	bcm_deinit_wlan_mem();
#endif
err_bcm_wifi_device:
	sysfs_remove_group(&dev->kobj, &bcm_wifi_attr_group);
	return rc;
}

static int bcm_wifi_remove(struct platform_device *pdev)
{
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	bcm_deinit_wlan_mem();
#endif
	sysfs_remove_group(&pdev->dev.kobj, &bcm_wifi_attr_group);
	wlan_ctrl = NULL;
	platform_set_drvdata(pdev, NULL);

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

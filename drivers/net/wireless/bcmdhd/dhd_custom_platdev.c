/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/mmc/host.h>
#include <linux/if.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdhci.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/fcntl.h>
#include <linux/regulator/consumer.h>

#include <linux/random.h>
#include <linux/ctype.h>
#include <dhd_linux.h>
#include <dhd_dbg.h>

/* wifi mac custom */
#define TEMP_BUFFER_LEN 3
#define ETHER_ADDR_LEN 6
#define ETHER_STR_LEN 12
#define ETHER_BUFFER_LEN 16

/* wifi chip vendor */
#define SEMCO_CHIP_VENDOR_ID 0X3333
#define MURATA_CHIP_VENDOR_ID 0X2200

struct pinctrl_data {
	struct pinctrl          *pctrl;
	struct pinctrl_state    *pins_active;
	struct pinctrl_state    *pins_sleep;
};

/* wifi reset gpio */
static int gpio_wl_reg_on = -1;
/* wifi wake up ap gpio */
static int brcm_wake_irq = -1;
/* for wifi card detect */
static struct mmc_host * wlan_mmc =NULL;
/* regulator for wifi rf */
static struct regulator *reg_rf = NULL;
/* pinctrl data of android,bcmdhd_wlan */
static struct pinctrl_data bcmdhd_pinctrl = {0};
/* vendor id for combo chip */
static int wifi_vendor_id = 0x00;
/* wifi mac custom */
char g_wlan_ether_addr[] = {0x00,0x00,0x00,0x00,0x00,0x00};

typedef enum
{
	WLAN_BASE10		=	10,
	WLAN_BASE16		=	16,
}WLAN_BASE_TYPE;

#ifdef CONFIG_DHD_USE_STATIC_BUF

#define STATIC_BUF_MAX_NUM	20
#define STATIC_BUF_SIZE	(PAGE_SIZE*2)

#define DHD_PREALLOC_PROT_SIZE   	(512)
#define DHD_PREALLOC_WIPHY_ESCAN0_SIZE	(64 * 1024)
#define DHD_PREALLOC_DHD_INFO_SIZE		(24 * 1024)
#ifdef CONFIG_64BIT
#define DHD_PREALLOC_IF_FLOW_LKUP_SIZE	(20 * 1024 * 2)
#else
#define DHD_PREALLOC_IF_FLOW_LKUP_SIZE	(20 * 1024)
#endif
#define DHD_PREALLOC_OSL_BUF_SIZE      (STATIC_BUF_MAX_NUM * STATIC_BUF_SIZE)

#define WLAN_SCAN_BUF_SIZE		(64 * 1024)

#if defined(CONFIG_64BIT)
#define WLAN_DHD_INFO_BUF_SIZE		(24 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE		(64 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE	(64 * 1024)
#else
#define WLAN_DHD_INFO_BUF_SIZE		(16 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE		(16 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE	(20 * 1024)
#endif /* CONFIG_64BIT */
#define WLAN_DHD_MEMDUMP_SIZE		(800 * 1024)

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#ifdef CONFIG_BCMDHD_PCIE
#define DHD_SKB_1PAGE_BUFSIZE	(PAGE_SIZE*1)
#define DHD_SKB_2PAGE_BUFSIZE	(PAGE_SIZE*2)
#define DHD_SKB_4PAGE_BUFSIZE	(PAGE_SIZE*4)

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	0
#define WLAN_SECTION_SIZE_2	0
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_1PAGE_BUF_NUM	0
#define DHD_SKB_2PAGE_BUF_NUM	64
#define DHD_SKB_4PAGE_BUF_NUM	0

#else
#define DHD_SKB_HDRSIZE		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_1PAGE_BUF_NUM	8
#define DHD_SKB_2PAGE_BUF_NUM	8
#define DHD_SKB_4PAGE_BUF_NUM	1
#endif /* CONFIG_BCMDHD_PCIE */

#define WLAN_SKB_1_2PAGE_BUF_NUM	((DHD_SKB_1PAGE_BUF_NUM) + \
		(DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM	((WLAN_SKB_1_2PAGE_BUF_NUM) + \
		(DHD_SKB_4PAGE_BUF_NUM))

void *wlan_static_prot = NULL;
void *wlan_static_scan_buf0 = NULL;
void *wlan_static_scan_buf1 = NULL;
void *wlan_static_dhd_info_buf = NULL;
void *wlan_static_if_flow_lkup = NULL;
void *wlan_static_osl_buf = NULL;
static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

static void *dhd_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == DHD_PREALLOC_PROT)
		return wlan_static_prot;

	if (section == DHD_PREALLOC_SKB_BUF)
		return wlan_static_skb;

	if (section == DHD_PREALLOC_WIPHY_ESCAN0)
		return wlan_static_scan_buf0;

	if (section == DHD_PREALLOC_WIPHY_ESCAN1)
		return wlan_static_scan_buf1;

	if (section == DHD_PREALLOC_OSL_BUF) {
		if (size > DHD_PREALLOC_OSL_BUF_SIZE) {
			pr_err("request OSL_BUF(%lu) is bigger than static size(%ld).\n",
				size, DHD_PREALLOC_OSL_BUF_SIZE);
			return NULL;
		}
		return wlan_static_osl_buf;
	}

	if (section == DHD_PREALLOC_DHD_INFO) {
		if (size > DHD_PREALLOC_DHD_INFO_SIZE) {
			pr_err("request DHD_INFO size(%lu) is bigger than static size(%d).\n",
				size, DHD_PREALLOC_DHD_INFO_SIZE);
			return NULL;
		}
		return wlan_static_dhd_info_buf;
	}
	if (section == DHD_PREALLOC_IF_FLOW_LKUP)  {
		if (size > DHD_PREALLOC_IF_FLOW_LKUP_SIZE) {
			pr_err("request DHD_IF_FLOW_LKUP size(%lu) is bigger than static size(%d).\n",
				size, DHD_PREALLOC_IF_FLOW_LKUP_SIZE);
			return NULL;
		}

		return wlan_static_if_flow_lkup;
	}
	if ((section < 0) || (section > DHD_PREALLOC_MAX))
		pr_err("request section id(%d) is out of max index %d\n",
				section, DHD_PREALLOC_MAX);

	return NULL;
}

static int dhd_init_wlan_mem(void)
{

	int i;
	int j;

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i]) {
			goto err_skb_alloc;
		}
	}

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i]) {
			goto err_skb_alloc;
		}
	}

#if !defined(CONFIG_BCMDHD_PCIE)
	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i]) {
		goto err_skb_alloc;
	}
#endif /* !CONFIG_BCMDHD_PCIE */

	wlan_static_prot = kmalloc(DHD_PREALLOC_PROT_SIZE, GFP_KERNEL);
	if (!wlan_static_prot) {
		pr_err("Failed to alloc wlan_static_prot\n");
		goto err_mem_alloc;
	}

	wlan_static_osl_buf = kmalloc(DHD_PREALLOC_OSL_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_osl_buf) {
		pr_err("Failed to alloc wlan_static_osl_buf\n");
		goto err_mem_alloc;
	}

	wlan_static_scan_buf0 = kmalloc(DHD_PREALLOC_WIPHY_ESCAN0_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf0) {
		pr_err("Failed to alloc wlan_static_scan_buf0\n");
		goto err_mem_alloc;
	}


	wlan_static_dhd_info_buf = kmalloc(DHD_PREALLOC_DHD_INFO_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_info_buf) {
		pr_err("Failed to alloc wlan_static_dhd_info_buf\n");
		goto err_mem_alloc;
	}
#ifdef CONFIG_BCMDHD_PCIE
	wlan_static_if_flow_lkup = kmalloc(DHD_PREALLOC_IF_FLOW_LKUP_SIZE, GFP_KERNEL);
	if (!wlan_static_if_flow_lkup) {
		pr_err("Failed to alloc wlan_static_if_flow_lkup\n");
		goto err_mem_alloc;
	}
#endif /* CONFIG_BCMDHD_PCIE */

	return 0;

err_mem_alloc:

	if (wlan_static_prot)
		kfree(wlan_static_prot);

	if (wlan_static_dhd_info_buf)
		kfree(wlan_static_dhd_info_buf);

	if (wlan_static_scan_buf1)
		kfree(wlan_static_scan_buf1);

	if (wlan_static_scan_buf0)
		kfree(wlan_static_scan_buf0);

	if (wlan_static_osl_buf)
		kfree(wlan_static_osl_buf);

#ifdef CONFIG_BCMDHD_PCIE
	if (wlan_static_if_flow_lkup)
		kfree(wlan_static_if_flow_lkup);
#endif
	pr_err("Failed to mem_alloc for WLAN\n");

	i = WLAN_SKB_BUF_NUM;

err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0; j < i; j++) {
		dev_kfree_skb(wlan_static_skb[j]);
	}

	return -ENOMEM;
}
#endif /* CONFIG_DHD_USE_STATIC_BUF */

int dhd_wlan_power(int on)
{
	DHD_INFO(("%s Enter: power %s\n", __FUNCTION__, on ? "on" : "off"));

	if (on) {
		if (reg_rf)
		{
			if(regulator_enable(reg_rf))
			{
				DHD_ERROR(("%s Enable wlan rf regulator fail\n", __FUNCTION__));
				return -EIO;
			}
		}

		if (gpio_direction_output(gpio_wl_reg_on, 1)) {
			DHD_ERROR(("%s WL_REG_ON power on fail\n", __FUNCTION__));
			return -EIO;
		}
		if (!gpio_get_value(gpio_wl_reg_on)) {
			DHD_ERROR(("%s WL_REG_ON pull up fail.\n", __FUNCTION__));
			return -EIO;
		} else {
			DHD_INFO(("%s: WL_REG_ON pull up success\n", __FUNCTION__));
		}
	} else {
		if (gpio_direction_output(gpio_wl_reg_on, 0)) {
			DHD_ERROR(("%s: WL_REG_ON power off fail\n", __FUNCTION__));
			return -EIO;
		}

		regulator_disable(reg_rf);
	}

	return 0;
}
EXPORT_SYMBOL(dhd_wlan_power);

static int dhd_wlan_reset(int onoff)
{
	return 0;
}

static int dhd_wlan_set_carddetect(int val)
{
	if(wlan_mmc) {
		mmc_detect_change(wlan_mmc, 0);
	} else {
		DHD_ERROR(("%s wlan_mmc is null,carddetect failed \n ",__FUNCTION__));
	}
	return 0;
}

#define WLC_CNTRY_BUF_SZ        4
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
};

static struct cntry_locales_custom brcm_wlan_translate_custom_table[] = {
	/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XT", 49},  /* Universal if Country code is unknown or empty */
	{"US", "US", 176},
	{"AE", "AE", 1},
	{"AR", "AR", 21},
	{"AT", "AT", 4},
	{"AU", "AU", 40},
	{"BE", "BE", 4},
	{"BG", "BG", 4},
	{"BN", "BN", 4},
	{"BR", "BR", 4},
	{"CA", "US", 176},   /* Previousely was CA/31 */
	{"CH", "CH", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DE", "DE", 7},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ES", "ES", 4},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GB", "GB", 6},
	{"GR", "GR", 4},
	{"HK", "HK", 2},
	{"HR", "HR", 4},
	{"HU", "HU", 4},
	{"IE", "IE", 5},
	{"IN", "IN", 28},
	{"IS", "IS", 4},
	{"IT", "IT", 4},
	{"ID", "ID", 5},
	{"JP", "JP", 86},
	{"KR", "KR", 57},
	{"KW", "KW", 5},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "MT", 4},
	{"MX", "MX", 20},
	{"MY", "MY", 16},
	{"NL", "NL", 4},
	{"NO", "NO", 4},
	{"NZ", "NZ", 4},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PY", "PY", 2},
	{"RO", "RO", 4},
	{"RU", "RU", 13},
	{"SE", "SE", 4},
	{"SG", "SG", 19},
	{"SI", "SI", 4},
	{"SK", "SK", 4},
	{"TH", "TH", 5},
	{"TR", "TR", 7},
	{"TW", "TW", 1},
	{"VN", "VN", 4},
};

struct cntry_locales_custom brcm_wlan_translate_nodfs_table[] = {
	{"",   "XT", 50},  /* Universal if Country code is unknown or empty */
	{"US", "US", 177},
	{"AU", "AU", 41},
	{"BR", "BR", 18},
	{"CA", "US", 177},
	{"CH", "E0", 33},
	{"CY", "E0", 33},
	{"CZ", "E0", 33},
	{"DE", "E0", 33},
	{"DK", "E0", 33},
	{"EE", "E0", 33},
	{"ES", "E0", 33},
	{"EU", "E0", 33},
	{"FI", "E0", 33},
	{"FR", "E0", 33},
	{"GB", "E0", 33},
	{"GR", "E0", 33},
	{"HK", "SG", 20},
	{"HR", "E0", 33},
	{"HU", "E0", 33},
	{"IE", "E0", 33},
	{"IN", "IN", 29},
	{"ID", "ID", 5},
	{"IS", "E0", 33},
	{"IT", "E0", 33},
	{"JP", "JP", 87},
	{"KR", "KR", 79},
	{"KW", "KW", 5},
	{"LI", "E0", 33},
	{"LT", "E0", 33},
	{"LU", "E0", 33},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "E0", 33},
	{"MY", "MY", 17},
	{"MX", "US", 177},
	{"NL", "E0", 33},
	{"NO", "E0", 33},
	{"PL", "E0", 33},
	{"PT", "E0", 33},
	{"RO", "E0", 33},
	{"SE", "E0", 33},
	{"SG", "SG", 20},
	{"SI", "E0", 33},
	{"SK", "E0", 33},
	{"SZ", "E0", 33},
	{"TH", "TH", 9},
	{"TW", "TW", 60},
};

static void *dhd_wlan_get_country_code(char *ccode, u32 flags)
{
	struct cntry_locales_custom *locales;
	int size;
	int i;

	if (!ccode)
		return NULL;

	if (flags & WLAN_PLAT_NODFS_FLAG) {
		locales = brcm_wlan_translate_nodfs_table;
		size = ARRAY_SIZE(brcm_wlan_translate_nodfs_table);
	} else {
		locales = brcm_wlan_translate_custom_table;
		size = ARRAY_SIZE(brcm_wlan_translate_custom_table);
	}

	for (i = 0; i < size; i++)
		if (strcmp(ccode, locales[i].iso_abbrev) == 0)
			return &locales[i];
	return &locales[0];
}

/* wifi combo chip vendor id */
void wlan_vendor_set(int vendorId)
{
	wifi_vendor_id = vendorId;
	DHD_INFO(("wlan_vendor_set: vendor_id=%x\n", wifi_vendor_id));
}
EXPORT_SYMBOL(wlan_vendor_set);

int get_wlan_chip_vendor_id(void)
{
	return wifi_vendor_id;
}

static ssize_t wifi_vendor_read(struct file *file, char __user *userbuf,
							size_t bytes, loff_t *off)
{
	char vendor_id[5] = {0};
	int len = 0;

	if (NULL == userbuf)
	{
		return -EFAULT;
	}

	snprintf(vendor_id, 5, "%x", wifi_vendor_id);
	len = strlen(vendor_id);

	if(copy_to_user(userbuf, vendor_id, len))
	{
		return -EFAULT;
	}

	return len;
}

static const struct file_operations proc_fops_wifi_vendor = {
	.owner = THIS_MODULE,
	.read = wifi_vendor_read,
};

void dhd_wlan_get_fw_nv_path(const char **fw, const char **nv)
{
	int vendor_id = -1;
	vendor_id = get_wlan_chip_vendor_id();
	if (SEMCO_CHIP_VENDOR_ID == vendor_id)
	{
		*fw = CONFIG_BCMDHD_SEMCO_FW_PATH;
		*nv = CONFIG_BCMDHD_SEMCO_NVRAM_PATH;

		printk(KERN_INFO "BCMDHD:wlan vendor: SEMCO\n");
	}
	else if (MURATA_CHIP_VENDOR_ID == vendor_id)
	{
		*fw = CONFIG_BCMDHD_MURATA_FW_PATH;
		*nv = CONFIG_BCMDHD_MURATA_NVRAM_PATH;

		printk(KERN_INFO "BCMDHD:wlan vendor: MURATA\n");
	}
	else
	{
		*fw = CONFIG_BCMDHD_SEMCO_FW_PATH;
		*nv = CONFIG_BCMDHD_SEMCO_NVRAM_PATH;
		printk(KERN_WARNING "BCMDHD:Warning: wlan vendor id ERROR!!! default use semco\n");
	}

	return;
}

/* wifi mac custom */
static int dhd_wifi_get_mac_addr(unsigned char *buf)
{
	const char null_addr[] = {0x00,0x00,0x00,0x00,0x00,0x00};

	if (NULL == buf)
	{
		return -1;
	}

	if (0 == memcmp(g_wlan_ether_addr, null_addr, ETHER_ADDR_LEN))
	{
		get_random_bytes(buf, ETHER_ADDR_LEN);
		buf[0] = 0x00;
		return 0;
	}

	memcpy(buf, g_wlan_ether_addr, ETHER_ADDR_LEN);

	printk(KERN_INFO"%s:MAC:%02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	return 0;
}

int wlan_strtoi(const char *nptr, const char **endptr, WLAN_BASE_TYPE base)
{
	int ret = 0;
	if (NULL == nptr)
	{
		return ret;
	}

	while (isspace(*nptr))
	{
		nptr++;
	}

	while ((isdigit(*nptr) && (WLAN_BASE10 == base || WLAN_BASE16 == base))
		|| (isalpha(*nptr) && (WLAN_BASE16 == base)))
	{
		if (isdigit(*nptr))
		{
			ret = (ret * base) + (*nptr - '0');
		}
		else
		{
			if (isupper(*nptr))
			{
				ret = (ret * base) + (*nptr - 'A');
			}
			else
			{
				ret = (ret * base) + (*nptr - 'a');
			}
			ret += WLAN_BASE10;
		}
		nptr++;
	}

	if (NULL != endptr)
	{
		*endptr = nptr;
	}
	return ret;
}

static ssize_t wifi_mac_write(struct file *file, const char __user *buffer,
							size_t count, loff_t *pos)
{
		int i = 0;
		char temp[TEMP_BUFFER_LEN];
		char temp_buf[ETHER_BUFFER_LEN] = {0};

		if (count < ETHER_STR_LEN)
		{
			return -EINVAL;
		}

		if (copy_from_user(temp_buf, buffer, ETHER_STR_LEN))
		{
			return -EFAULT;
		}

		for (i = 0; i < ETHER_STR_LEN; i++)
		{
			if (((temp_buf[i] < '0')|| (temp_buf[i] > '9')) &&
				((temp_buf[i] < 'a') || (temp_buf[i] > 'f')) &&
				((temp_buf[i] < 'A') || (temp_buf[i] > 'F')))
			{
				DHD_ERROR(("wlan:invalid mac address\n"));
				return -EINVAL;
			}
		}

		for (i = 0; i < ETHER_ADDR_LEN; i++)
		{
			memset(temp, 0, TEMP_BUFFER_LEN);
			memcpy(temp, temp_buf + 2*i, TEMP_BUFFER_LEN-1);
			g_wlan_ether_addr[i] = (char)wlan_strtoi(temp, NULL, WLAN_BASE16);
		}

	return count;
}

static const struct file_operations proc_fops_wifi_mac = {
	.owner = THIS_MODULE,
	.write = wifi_mac_write,
};

int dhd_wlan_proc_create(void)
{
	int retval = 0;
	struct proc_dir_entry *ent;
	struct proc_dir_entry *wifi_dir;

	wifi_dir = proc_mkdir("wifi", NULL);
	if (wifi_dir == NULL)
	{
		DHD_ERROR(("Unable to create /proc/wifi directory.\n"));
		return -ENOMEM;
	}

	/* read/write wifi mac entries */
	ent = proc_create("mac", 0660, wifi_dir, &proc_fops_wifi_mac);
	if (ent == NULL)
	{
		DHD_ERROR(("Unable to create /proc/wifi/mac entry.\n"));
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write wifi vendor entries */
	ent = proc_create("vendor", 0660, wifi_dir, &proc_fops_wifi_vendor);
	if (ent == NULL)
	{
		DHD_ERROR(("Unable to create /proc/wifi/vendor entry.\n"));
		retval = -ENOMEM;
		goto fail;
	}

	return retval;

fail:
	remove_proc_entry("wifi", 0);
	return retval;

}

struct resource dhd_wlan_resources[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= 0, /* Dummy */
		.end	= 0, /* Dummy */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE
			| IORESOURCE_IRQ_HIGHLEVEL, /* Dummy */
	},
};
EXPORT_SYMBOL(dhd_wlan_resources);


struct wifi_platform_data dhd_wlan_control = {
        .set_power	= dhd_wlan_power,
	.set_reset	= dhd_wlan_reset,
	.set_carddetect	= dhd_wlan_set_carddetect,
	.get_mac_addr	= dhd_wifi_get_mac_addr,
#ifdef CONFIG_DHD_USE_STATIC_BUF
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif
	.get_country_code = dhd_wlan_get_country_code,
	.get_fw_nv_path = dhd_wlan_get_fw_nv_path,
};
EXPORT_SYMBOL(dhd_wlan_control);

int dhd_wlan_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	struct pinctrl *pctrl = NULL;
	struct pinctrl_state *pins_active = NULL;
	struct pinctrl_state *pins_sleep = NULL;

	if (NULL == pdev){
		DHD_ERROR(("pdev is NULL.\n"));
		return -1;
	}

	pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pctrl)) {
		ret = PTR_ERR(pctrl);
		DHD_ERROR(("Could not get pinsctrl info, err:%d\n", ret));
		goto out;
	}

	pins_active = pinctrl_lookup_state(pctrl, "active");
	if (IS_ERR(pins_active)) {
		ret = PTR_ERR(pins_active);
		DHD_ERROR(("Could not get active pinstates, err:%d\n", ret));
		goto out;
	}

	pins_sleep = pinctrl_lookup_state(pctrl, "sleep");
	if (IS_ERR(pins_sleep)) {
		ret = PTR_ERR(pins_sleep);
		DHD_ERROR(( "Could not get sleep pinstates, err:%d\n", ret));
		goto out;
	}

	bcmdhd_pinctrl.pctrl = pctrl;
	bcmdhd_pinctrl.pins_active = pins_active;
	bcmdhd_pinctrl.pins_sleep = pins_sleep;
	return ret;

out:
	bcmdhd_pinctrl.pctrl = NULL;
	bcmdhd_pinctrl.pins_active = NULL;
	bcmdhd_pinctrl.pins_sleep = NULL;
	return ret;
}

int dhd_wlan_pinctrl_setup(bool enable)
{
	int ret = 0;

	if (enable)
		ret = pinctrl_select_state(bcmdhd_pinctrl.pctrl, bcmdhd_pinctrl.pins_active);
	else
		ret = pinctrl_select_state(bcmdhd_pinctrl.pctrl, bcmdhd_pinctrl.pins_sleep);
	if (ret < 0){
		DHD_ERROR(( "Could not select pinstates:%s, err:%d\n", enable?"active":"sleep", ret));
	}

	return ret;
}

int dhd_wlan_gpio_init(void)
{
	int ret = 0;
	int wl_reg_on = -1;
	int wl_host_wake = -1;
	char *wlan_node = "android,bcmdhd_wlan";
	struct device_node *np = NULL;
	struct device_node *sdio_node = NULL;
	struct platform_device *bcmdhd_pdev = NULL;
	struct platform_device *mmc_host_pdev = NULL;
	struct sdhci_host *host = NULL;

	/* Get bcmdhd node */
	np = of_find_compatible_node(NULL, NULL, wlan_node);
	if (!np) {
		DHD_ERROR(("%s:Failed to get mmc host node\n", __FUNCTION__));
		return -ENODEV;
	}
	bcmdhd_pdev = of_find_device_by_node(np);
	if(NULL == bcmdhd_pdev)
	{
		DHD_ERROR(("Failed to find bcmdhd device by node\n"));
		return -ENODEV;
	}

	/* bcmdhd_wlan pinctrl init  */
	ret = dhd_wlan_pinctrl_init(bcmdhd_pdev);
	if (ret){
		DHD_ERROR(("Failed to init pinctrl state\n"));
		return ret;
	}
	ret = dhd_wlan_pinctrl_setup(true);
	if (ret){
		DHD_ERROR(("Failed to setup pinctrl state\n"));
		return ret;
	}

	/* Get mmc host */
	sdio_node = of_parse_phandle(np, "sdhci-name", 0);
	if (NULL == sdio_node)
	{
		DHD_ERROR(("%s:Failed to get sdhci-name node\n", __FUNCTION__));
		return -ENODEV;
	}
	mmc_host_pdev = of_find_device_by_node(sdio_node);
	if (mmc_host_pdev == NULL) {
		DHD_ERROR(("%s:Failed to get platform device\n", __FUNCTION__));
		return -ENODEV;
	}
	host = platform_get_drvdata(mmc_host_pdev);
	wlan_mmc = host->mmc;

	/* Get regulator of rf */
	reg_rf = regulator_get(&bcmdhd_pdev->dev, "vdd-rf");
	if(IS_ERR(reg_rf))
	{
		DHD_ERROR(("Failed to get vdd-rf regulator\n"));
		regulator_put(reg_rf);
		return -ENODEV;
	}

	/* Get wl_reg_on gpio */
	wl_reg_on = of_get_named_gpio(np, "wl_reg_on", 0);
	if (wl_reg_on >= 0) {
		gpio_wl_reg_on = wl_reg_on;
	}

	/* Get wl_host_wake gpio */
	wl_host_wake = of_get_named_gpio(np, "wl_host_wake", 0);
	if (wl_host_wake >= 0) {
		gpio_request(wl_host_wake, "WL_HOST_WAKE");
		gpio_direction_input(wl_host_wake);
		brcm_wake_irq = gpio_to_irq(wl_host_wake);
	}

	if (gpio_request(gpio_wl_reg_on, "WL_REG_ON"))
		DHD_ERROR(("%s: Failed to request gpio %d for WL_REG_ON\n",
			 __FUNCTION__, gpio_wl_reg_on));
	else
		DHD_INFO(("%s: gpio_request WL_REG_ON done\n", __FUNCTION__));

	if (gpio_direction_output(gpio_wl_reg_on, 1))
		DHD_ERROR(("%s: WL_REG_ON failed to pull up\n", __FUNCTION__));
	else
		DHD_INFO(("%s: WL_REG_ON is pulled up\n", __FUNCTION__));

	if (gpio_get_value(gpio_wl_reg_on))
		DHD_INFO(("%s: Initial WL_REG_ON: [%d]\n",
			__FUNCTION__, gpio_get_value(gpio_wl_reg_on)));
	return 0;
}

int dhd_wlan_init(void)
{
	int ret = -1;
#ifdef CONFIG_DHD_USE_STATIC_BUF
	dhd_init_wlan_mem();
#endif
	ret = dhd_wlan_proc_create();
	if (ret)
		return ret;

	ret = dhd_wlan_gpio_init();
	if (ret)
		return ret;

	dhd_wlan_resources[0].start = dhd_wlan_resources[0].end =
		brcm_wake_irq;

	return 0;
}
EXPORT_SYMBOL_GPL(dhd_wlan_init);

void __exit dhd_wlan_exit(void)
{
#ifdef CONFIG_DHD_USE_STATIC_BUF
	int i;

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		if (wlan_static_skb[i])
			dev_kfree_skb(wlan_static_skb[i]);
	}

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		if (wlan_static_skb[i])
			dev_kfree_skb(wlan_static_skb[i]);
	}

#if !defined(CONFIG_BCMDHD_PCIE)
	if (wlan_static_skb[i])
		dev_kfree_skb(wlan_static_skb[i]);
#endif /* !CONFIG_BCMDHD_PCIE */

	if (wlan_static_prot)
		kfree(wlan_static_prot);

	if (wlan_static_osl_buf)
		kfree(wlan_static_osl_buf);

	if (wlan_static_scan_buf0)
		kfree(wlan_static_scan_buf0);

	if (wlan_static_dhd_info_buf)
		kfree(wlan_static_dhd_info_buf);

	if (wlan_static_scan_buf1)
		kfree(wlan_static_scan_buf1);

#ifdef CONFIG_BCMDHD_PCIE
	if (wlan_static_if_flow_lkup)
		kfree(wlan_static_if_flow_lkup);
#endif
#endif /* CONFIG_BCMDHD_USE_STATIC_BUF */

	if (NULL != reg_rf) {
		regulator_put(reg_rf);
	}

	(void)dhd_wlan_pinctrl_setup(false);

	return;
}

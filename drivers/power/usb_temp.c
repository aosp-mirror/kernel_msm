/* Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "USBTEMP: %s: " fmt, __func__
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/usb_temp.h>

struct usb_temp_device_info {
	struct device   *dev;
	struct workqueue_struct *usb_temp_wq;
	struct work_struct usb_temp_check_wk;
	struct notifier_block   usb_nb;
	struct power_supply     *usb_psy;
	struct power_supply     *batt_psy;
	struct power_supply     *bms_psy;
	struct hrtimer timer;
	int gpio_usb_temp;
	int open_mosfet_temp;
	int close_mosfet_temp;
	int interval_switch_temp;
	int check_interval;
	int keep_check_cnt;
	struct regulator	*vdd;
	struct qpnp_vadc_chip	*vadc_dev;
	int no_need_usb_temp;
};

#define		INVALID_DELAY_TIME			0
#define		USB_TEMP_INSERT_CHG_CNT			1100
#define		USB_TEMP_START_CHK_CNT			0
#define		USB_TEMP_END_CHK_CNT			1001
#define		USB_TEMP_CNT				2
#define		FAST_MONITOR_INTERVAL			300
#define		NORMAL_MONITOR_INTERVAL			1000
#define		INVALID_DELTA_TEMP			0
#define		USB_TEMP_CHK_CNT_STEP			1
#define		USB_TEMP_DEFAULT_CHK_CNT		(-1)
#define		GPIO_HIGH				1
#define		GPIO_LOW				0
#define		INTERVAL_0				0
#define		INVALID_BATT_TEMP			(-255)
#define		COVERSE_TEMP_UNIT			10
#define		TUSB_TEMP_UPPER_LIMIT			100
#define		TUSB_TEMP_LOWER_LIMIT			(-30)
#define		TRUE					1
#define		FALSE					0

static int protect_enable = FALSE;
static int protect_dmd_notify_enable = TRUE;
static int is_usb_protect_mode;
#ifdef SUPPORT_TEST_MODE
static int utp_test_temp;
#endif

static struct usb_temp_device_info *g_di;
static struct wake_lock usb_temp_wakelock;

int get_power_supply_info(struct power_supply *psy, int prop)
{
	union power_supply_propval ret = {0, };

	if ((g_di == NULL) || (g_di->batt_psy == NULL)
		|| (g_di->bms_psy == NULL) || (g_di->usb_psy == NULL)) {
		pr_err("g_di is NULL!\n");
		return -EINVAL;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_TYPE:
		g_di->usb_psy->get_property(g_di->usb_psy, prop, &ret);
		pr_debug("charger type = %d\n", ret.intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		g_di->batt_psy->get_property(g_di->batt_psy, prop, &ret);
		pr_debug("battery persent = %d\n", ret.intval);
		break;
	default:
		return -EINVAL;
	}
	return ret.intval;
}

#define UTP_VTG_MIN_UV		1800000
#define UTP_VTG_MAX_UV		1800000
static bool enable_vdd_for_adc(struct usb_temp_device_info *di)
{
	int rc;
	static int set_once;

	if (set_once)
		return 1;

	di->vdd = devm_regulator_get(di->dev, "vdd");
	if (IS_ERR(di->vdd)) {
		pr_err("Regulator get failed vdd rc=%d\n", rc);
		goto get_vdd_failed;
	}

	if (regulator_count_voltages(di->vdd) > 0) {
		rc = regulator_set_voltage(di->vdd, UTP_VTG_MIN_UV,
					   UTP_VTG_MAX_UV);
		if (rc)
			pr_err("Regulator set_vtg failed vdd rc=%d\n", rc);
	}

	rc = regulator_enable(di->vdd);
	if (rc)
		pr_err("Regulator vdd enable failed rc=%d\n", rc);

	set_once++;
	pr_err("enable vdd for adc success\n");
	return 1;

get_vdd_failed:
	return 0;
}

static bool is_factory_mode;
static int __init early_parse_factory_mode(char *cmdline)
{
	if ((cmdline) && !strncmp(cmdline, "ffbm", strlen("ffbm")))
		is_factory_mode = true;

	return 0;
}
early_param("androidboot.mode", early_parse_factory_mode);

static bool is_new_mainboard;
static int __init early_parse_mainboard_id(char *cmdline)
{
	if ((cmdline) && !strncmp(cmdline, "2", strlen("2")))
		is_new_mainboard = true;

	return 0;
}
early_param("androidboot.boardid.version", early_parse_mainboard_id);

static void usb_temp_wake_lock(void)
{
	if (!wake_lock_active(&usb_temp_wakelock)) {
		pr_info("wake lock\n");
		wake_lock(&usb_temp_wakelock);
	}
}

static void usb_temp_wake_unlock(void)
{
	if (wake_lock_active(&usb_temp_wakelock)) {
		pr_info("wake unlock\n");
		wake_unlock(&usb_temp_wakelock);
	}
}

static void charge_type_handler(struct usb_temp_device_info *di,
						enum power_supply_type type)
{
	int interval = 0;

	if ((!protect_enable) || (NULL == di))
		return;

	if ((POWER_SUPPLY_TYPE_USB_DCP == type)
			|| (POWER_SUPPLY_TYPE_USB == type)
			|| (POWER_SUPPLY_TYPE_USB_CDP == type)
			|| (POWER_SUPPLY_TYPE_USB_HVDCP == type)
			|| (POWER_SUPPLY_TYPE_USB_HVDCP_3 == type)) {
		if (hrtimer_active(&(di->timer))) {
			pr_info("timer already working , do nothing\n");
		} else {
			pr_info("start usb_temp check\n");
			interval = INTERVAL_0;
			/* record 30 seconds after the charger just insert;
			 * 30s = (1100 - 1001 + 1)*300ms */
			di->keep_check_cnt = USB_TEMP_INSERT_CHG_CNT;
			hrtimer_start(&di->timer,
				ktime_set(interval/MSEC_PER_SEC,
				(interval % MSEC_PER_SEC) * USEC_PER_SEC),
				HRTIMER_MODE_REL);
		}
	} else {
		pr_info("charger type = %d, do nothing\n", type);
	}
}

static int usb_notifier_call(struct notifier_block *usb_nb,
						unsigned long event, void *data)
{
	struct usb_temp_device_info *di = container_of(usb_nb,
					struct usb_temp_device_info, usb_nb);
	enum power_supply_type type = ((enum power_supply_type)event);

	pr_err("%s:%d\n", __func__, type);
	charge_type_handler(di, type);
	return NOTIFY_OK;
}

static int get_usb_temp_value(struct usb_temp_device_info *di)
{
	int ret = 0;
	struct qpnp_vadc_result results;

#ifdef SUPPORT_TEST_MODE
	if (utp_test_temp > 0)
		return utp_test_temp;
#endif

	ret = qpnp_vadc_read(di->vadc_dev, P_MUX2_1_1, &results);
	if (ret) {
		pr_err("Unable to read usb temperature rc=%d\n", ret);
		return ret;
	}
	pr_debug("get_usb_temp_value %d, %lld\n", results.adc_code,
							results.physical);

	return (int)results.physical;
}

static int get_batt_temp_value(void)
{
	int rc = 0;
	union power_supply_propval ret = {0, };

	if ((g_di == NULL) || (g_di->batt_psy == NULL)
			|| (g_di->bms_psy == NULL)) {
		pr_err(" %s g_di is NULL!\n", __func__);
		return INVALID_BATT_TEMP;
	}

	rc = g_di->batt_psy->get_property(g_di->batt_psy,
						POWER_SUPPLY_PROP_TEMP, &ret);
	if (rc) {
		pr_err(" %s  get temp error!\n", __func__);
		return INVALID_BATT_TEMP;
	}

	pr_debug("the battery temperature is %d\n", ret.intval);
	return ret.intval/COVERSE_TEMP_UNIT;

}

static void set_interval(struct usb_temp_device_info *di, int temp)
{
	if (NULL == di) {
		pr_err(" %s di is NULL!\n", __func__);
		return;
	}

	if (temp > di->interval_switch_temp) {
		di->check_interval = FAST_MONITOR_INTERVAL;
		di->keep_check_cnt = USB_TEMP_START_CHK_CNT;
		is_usb_protect_mode = TRUE;
		pr_debug("cnt = %d!\n", di->keep_check_cnt);
	} else {
		if (di->keep_check_cnt > USB_TEMP_END_CHK_CNT) {
			/*check the temperature per 0.3 second for 100 times,
			 *when the charger just insert.
			 */
			pr_debug("cnt = %d!\n", di->keep_check_cnt);
			di->keep_check_cnt -= USB_TEMP_CHK_CNT_STEP;
			di->check_interval = FAST_MONITOR_INTERVAL;
			is_usb_protect_mode = FALSE;
		} else if (di->keep_check_cnt == USB_TEMP_END_CHK_CNT) {
			/* reset the flag when the temp status is stable*/
			pr_debug("cnt = %d!\n", di->keep_check_cnt);
			di->keep_check_cnt = USB_TEMP_DEFAULT_CHK_CNT;
			di->check_interval = NORMAL_MONITOR_INTERVAL;
			is_usb_protect_mode = FALSE;
			usb_temp_wake_unlock();
		} else if (di->keep_check_cnt >= USB_TEMP_START_CHK_CNT) {
			pr_debug("cnt = %d!\n", di->keep_check_cnt);
			di->keep_check_cnt =
				di->keep_check_cnt + USB_TEMP_CHK_CNT_STEP;
			di->check_interval = FAST_MONITOR_INTERVAL;
			is_usb_protect_mode = TRUE;
		} else {
			di->check_interval = NORMAL_MONITOR_INTERVAL;
			is_usb_protect_mode = FALSE;
		}
	}
}

static void protection_process(struct usb_temp_device_info *di, int temp)
{
	int gpio_value = 0;

	if (NULL == di) {
		pr_err(" %s di is NULL!\n", __func__);
		return;
	}

	gpio_value = gpio_get_value(di->gpio_usb_temp);
	if (temp >= di->open_mosfet_temp) {
		usb_temp_wake_lock();
		gpio_set_value(di->gpio_usb_temp, GPIO_HIGH);/*open mosfet*/
		pr_info("pull up, org gpio value is:%d\n", gpio_value);
	} else if (temp <= di->close_mosfet_temp) {
		gpio_set_value(di->gpio_usb_temp, GPIO_LOW);/*close mosfet*/
		pr_debug("pull down, org gpio value is:%d\n", gpio_value);
	} else {
		/*do nothing*/
	}
}

static void check_temperature(struct usb_temp_device_info *di)
{
	int tusb = 0;
	int tbatt = 0;
	int tdiff = 0;

	if (NULL == di) {
		pr_err(" %s di is NULL!\n", __func__);
		return;
	}

	tusb = get_usb_temp_value(di);
	tbatt = get_batt_temp_value();

	pr_debug("tusb = %d, tbatt = %d\n", tusb, tbatt);
	tdiff = tusb - tbatt;
	if (INVALID_BATT_TEMP == tbatt) {
		tdiff = INVALID_DELTA_TEMP;
		pr_err("get battery adc temp err, not care!!!\n");
	}

	set_interval(di, tdiff);
	protection_process(di, tdiff);
}

static void usb_temp_check_work(struct work_struct *work)
{
	struct usb_temp_device_info *di = container_of(work,
				struct usb_temp_device_info, usb_temp_check_wk);
	int interval = 0;
	int type = 0;

	type = get_power_supply_info(di->usb_psy, POWER_SUPPLY_PROP_TYPE);

	if (((USB_TEMP_DEFAULT_CHK_CNT == di->keep_check_cnt)
				&& (POWER_SUPPLY_TYPE_UNKNOWN == type))) {
		protect_dmd_notify_enable = TRUE;
		gpio_set_value(di->gpio_usb_temp, GPIO_LOW);
		di->keep_check_cnt = USB_TEMP_DEFAULT_CHK_CNT;
		di->check_interval = NORMAL_MONITOR_INTERVAL;
		is_usb_protect_mode = FALSE;
		di->keep_check_cnt = USB_TEMP_INSERT_CHG_CNT;
		pr_info("charger type is %d, stop checking\n", type);
		return;
	}

	check_temperature(di);
	interval = di->check_interval;

	hrtimer_start(&di->timer,
			ktime_set(interval/MSEC_PER_SEC,
				(interval % MSEC_PER_SEC) * USEC_PER_SEC),
							HRTIMER_MODE_REL);

}

static enum hrtimer_restart usb_temp_timer_func(struct hrtimer *timer)
{
	struct usb_temp_device_info *di = NULL;

	di = container_of(timer, struct usb_temp_device_info, timer);
	queue_work(di->usb_temp_wq, &di->usb_temp_check_wk);

	return HRTIMER_NORESTART;
}

static void check_ntc_can_worked(struct usb_temp_device_info *di)
{
	int temp = 0;
	int sum = 0;
	int i = 0;

	for (i = 0; i < USB_TEMP_CNT; ++i)
		sum += get_usb_temp_value(di);

	temp = sum / USB_TEMP_CNT;
	if (temp > TUSB_TEMP_UPPER_LIMIT || temp < TUSB_TEMP_LOWER_LIMIT) {
		protect_enable = FALSE;
		pr_info("usb temp is not right, do not need this function\n");
	} else {
		pr_info("enable usb temp protect\n");
		protect_enable = TRUE;
	}
}

#ifdef SUPPORT_TEST_MODE
static ssize_t utp_test_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(int), "%d\n", utp_test_temp);
}

static ssize_t utp_test_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int input;

	if (kstrtouint(buf, 0, &input))
		ret = -EINVAL;
	else
		utp_test_temp = input;

	pr_err("set the usb temperatue to %d\n", utp_test_temp);

	return ret;
}

static struct device_attribute attrs[] = {
	__ATTR(utp_test_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			utp_test_mode_show,
			utp_test_mode_store),
};
static struct kobject *utp_test_kobj;
#endif

static int usb_temp_probe(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	struct usb_temp_device_info *di = NULL;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;
	struct power_supply *usb_psy = NULL;
	struct power_supply *batt_psy = NULL;
	struct power_supply *bms_psy = NULL;
	int ret = 0;
	int batt_present = TRUE;
	int need_usb_temp = TRUE;

	pr_info("enter into usb_temp probe\n");

	if (!is_new_mainboard) {
		pr_err("Not new version mainboard, not support usb temp\n");
		return -ENODEV;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\\n");
		return -EPROBE_DEFER;
	}
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("batt supply not found deferring probe\\n");
		return -EPROBE_DEFER;
	}
	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("bms supply not found deferring probe\\n");
		return -EPROBE_DEFER;
	}

	np = pdev->dev.of_node;
	if (NULL == np) {
		pr_err("np is NULL\\n");
		return -EINVAL;
	}
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	pr_info("alloc di success, let's go on\n");

	di->dev = &pdev->dev;
	dev_set_drvdata(&(pdev->dev), di);
	g_di = di;

	di->usb_psy = usb_psy;
	di->batt_psy = batt_psy;
	di->bms_psy = bms_psy;
	is_usb_protect_mode = FALSE;
	di->keep_check_cnt = USB_TEMP_INSERT_CHG_CNT;

	enable_vdd_for_adc(di);
	di->vadc_dev = qpnp_get_vadc(di->dev, "usb_temp");
	if (IS_ERR(di->vadc_dev)) {
		pr_err("vadc is not valid\n");
		return ret;
	}

	di->gpio_usb_temp = of_get_named_gpio(np,
					"qcom,gpio_usb_temp_protect", 0);
	if (!gpio_is_valid(di->gpio_usb_temp)) {
		pr_err("gpio_usb_temp is not valid\n");
		ret = -EINVAL;
		goto free_mem;
	}
	pr_info("gpio_usb_temp = %d\n", di->gpio_usb_temp);

	ret = gpio_request(di->gpio_usb_temp, "usb_temp_protect");
	if (ret) {
		pr_err("could not request gpio_usb_temp\n");
		ret = -EINVAL;
		goto free_mem;
	}
	gpio_direction_output(di->gpio_usb_temp, GPIO_LOW);

	ret = of_property_read_u32(np, "qcom,no_need_usb_temp",
						&(di->no_need_usb_temp));
	if (ret)
		pr_err("get qcom,no_need_usb_temp fail!\n");
	pr_info("no_need_usb_temp = %d\n", di->no_need_usb_temp);

	ret = of_property_read_u32(np, "qcom,open_mosfet_temp",
						&(di->open_mosfet_temp));
	if (ret) {
		pr_err("get open_mosfet_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("open_mosfet_temp = %d\n", di->open_mosfet_temp);

	ret = of_property_read_u32(np, "qcom,close_mosfet_temp",
						&(di->close_mosfet_temp));
	if (ret) {
		pr_err("get close_mosfet_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("close_mosfet_temp = %d\n", di->close_mosfet_temp);

	ret = of_property_read_u32(np, "qcom,interval_switch_temp",
						&(di->interval_switch_temp));
	if (ret) {
		pr_err("get interval_switch_temp info fail!\n");
		ret = -EINVAL;
		goto free_gpio;
	}
	pr_info("interval_switch_temp = %d\n", di->interval_switch_temp);

	check_ntc_can_worked(di);
	batt_present = get_power_supply_info(batt_psy,
						POWER_SUPPLY_PROP_PRESENT);

	if (is_factory_mode || (TRUE == di->no_need_usb_temp))
		need_usb_temp = FALSE;

	if ((!batt_present) || (FALSE == need_usb_temp)) {
		pr_err("no battery or no need usb_temp in factory mode\n");
		protect_enable = FALSE;
	}

	if (!protect_enable)
		goto free_gpio;

	wake_lock_init(&usb_temp_wakelock, WAKE_LOCK_SUSPEND,
						"usb_temp_protect_wakelock");
	di->usb_temp_wq = create_singlethread_workqueue("usb_temp_protect_wq");
	INIT_WORK(&di->usb_temp_check_wk, usb_temp_check_work);
	hrtimer_init(&di->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	di->timer.function = usb_temp_timer_func;
	di->usb_nb.notifier_call = usb_notifier_call;
	ret = charger_register_notifier(&di->usb_nb);
	if (ret < 0) {
		pr_err("charger_type_notifier_register failed\\n");
		ret = -EINVAL;
		goto free_gpio;
	}

	type = get_power_supply_info(usb_psy, POWER_SUPPLY_PROP_TYPE);

	pr_info("usb type = %d\n", type);
	charge_type_handler(di, type);

#ifdef SUPPORT_TEST_MODE
	utp_test_kobj = kobject_create_and_add("utp_test", NULL);
	ret = sysfs_create_file(utp_test_kobj, &attrs[0].attr);
	if (ret < 0) {
		pr_err("Failed to create sysfs attributes\n");
		sysfs_remove_file(utp_test_kobj, &attrs[0].attr);
	}
#endif

	pr_info("usb_temp probe ok!\n");
	return 0;

free_gpio:
	gpio_free(di->gpio_usb_temp);
free_mem:
	kfree(di);
	g_di = NULL;

	return ret;
}

static int usb_temp_remove(struct platform_device *pdev)
{
	struct usb_temp_device_info *di = dev_get_drvdata(&pdev->dev);

#ifdef SUPPORT_TEST_MODE
	sysfs_remove_file(utp_test_kobj, &attrs[0].attr);
#endif
	gpio_free(di->gpio_usb_temp);
	kfree(di);
	g_di = NULL;

	return 0;
}
static struct of_device_id usb_temp_match_table[] = {
	{
		.compatible = "qcom,usb_temp_protect",
		.data = NULL,
	},
	{
	},
};
static struct platform_driver usb_temp_driver = {
	.probe = usb_temp_probe,
	.remove = usb_temp_remove,
	.driver = {
		.name = "qcom,usb_temp_protect",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(usb_temp_match_table),
},
};

static int __init usb_temp_init(void)
{
	return platform_driver_register(&usb_temp_driver);
}

device_initcall_sync(usb_temp_init);

static void __exit usb_temp_exit(void)
{
	platform_driver_unregister(&usb_temp_driver);
}

module_exit(usb_temp_exit);

MODULE_DESCRIPTION("USB TEMPERATURE PROTECT");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:usb-temp");

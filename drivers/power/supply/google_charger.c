/*
 * Copyright 2018 Google, Inc
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
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s " fmt, __func__

#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>

#define CHG_TEMP_NB_LIMITS_MAX 10
#define CHG_VOLT_NB_LIMITS_MAX 5

#define CV_RANGE_ACCURACY 10000	/* in microVolt */
#define CV_RANGE_ACCURACY_CNT 3	/* 3 continuous updates valid */

struct chg_profile {
	u32 update_interval;
	u32 battery_capacity;
	u32 temp_nb_limits;
	s32 temp_limits[CHG_TEMP_NB_LIMITS_MAX];
	u32 volt_nb_limits;
	s32 volt_limits[CHG_VOLT_NB_LIMITS_MAX];
	/* Array of constant current limits */
	s32 *cccm_limits;
};

struct chg_drv {
	struct device *device;
	struct power_supply *batt_psy;
	struct power_supply *bms_psy;
	struct power_supply *usb_psy;
	struct chg_profile chg_profile;
	struct notifier_block psy_nb;

	struct delayed_work chg_work;
	struct wakeup_source chg_ws;

	bool chg_work_running;
	bool stop_charging;
	int temp_idx;
	int vbatt_idx;
	int chg_type;
	int checked_cv_cnt;
};

/* Return the constant charge current in uA given the indexes  */
static s32 get_cccm_limits(struct chg_profile *profile,
			   unsigned int ti, unsigned int vi)
{
	s32 res = profile->cccm_limits[(ti * profile->volt_nb_limits) + vi];
	/* chg-battery-capacity is in mAh, chg-cc-limits relative to 100 */
	res = res * profile->battery_capacity * 10;
	/* CCC has a 25mA resolution */
	res = (res / 25000) * 25000;
	return res;
}

static int psy_changed(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct power_supply *psy = data;
	struct chg_drv *chg_drv = container_of(nb, struct chg_drv, psy_nb);

	pr_debug("name=%s evt=%lu\n", psy->desc->name, action);

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    (!strcmp(psy->desc->name, "battery") ||
	     !strcmp(psy->desc->name, "usb"))) {
		cancel_delayed_work(&chg_drv->chg_work);
		schedule_delayed_work(&chg_drv->chg_work, 0);
	}
	return NOTIFY_OK;
}

static char *psy_charge_type_str[] = {
	"Unknown", "N/A", "Trickle", "Fast", "Taper"
};

static char *psy_usb_type_str[] = {
	"Unknown", "Battery", "UPS", "Mains", "USB", "USB_DCP",
	"USB_CDP", "USB_ACA", "USB_HVDCP", "USB_HVDCP_3", "USB_PD",
	"Wireless", "USB_FLOAT", "BMS", "Parallel", "Main", "Wipower",
	"TYPEC", "TYPEC_UFP", "TYPEC_DFP"
};

#define PSY_GET_PROP(psy, psp) psy_get_prop(psy, psp, #psp)
static inline int psy_get_prop(struct power_supply *psy,
			       enum power_supply_property psp, char *prop_name)
{
	union power_supply_propval val;
	int ret = 0;

	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, psp, &val);
	if (ret < 0) {
		pr_err("failed to get %s from '%s', ret=%d\n",
		       prop_name, psy->desc->name, ret);
		return -EINVAL;
	}
	pr_debug("get %s for '%s' => %d\n",
		 prop_name, psy->desc->name, val.intval);
	return val.intval;
}

#define PSY_SET_PROP(psy, psp, val) psy_set_prop(psy, psp, val, #psp)
static inline int psy_set_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       int intval, char *prop_name)
{
	union power_supply_propval val;
	int ret = 0;

	if (!psy)
		return -EINVAL;
	val.intval = intval;
	pr_debug("set %s for '%s' to %d\n", prop_name, psy->desc->name, intval);
	ret = power_supply_set_property(psy, psp, &val);
	if (ret < 0) {
		pr_err("failed to set %s for '%s', ret=%d\n",
		       prop_name, psy->desc->name, ret);
		return -EINVAL;
	}
	return 0;
}

#define CHG_WORK_ERROR_RETRY_MS 1000
static void chg_work(struct work_struct *work)
{
	struct chg_drv *chg_drv =
	    container_of(work, struct chg_drv, chg_work.work);
	struct chg_profile *profile = &chg_drv->chg_profile;
	union power_supply_propval val;
	struct power_supply *batt_psy = chg_drv->batt_psy;
	struct power_supply *usb_psy = chg_drv->usb_psy;
	int vbatt, temp, ibatt, chg_type, usb_present, usb_type, batt_status;
	int vbatt_idx = 0, temp_idx = 0, fv_uv, cc_max, cc_next_max;
	int cv_delta;
	bool rerun_work = false;

	pr_debug("battery charging work item\n");

	if (!chg_drv->chg_work_running) {
		chg_drv->chg_work_running = true;
		__pm_stay_awake(&chg_drv->chg_ws);
	}

	temp = PSY_GET_PROP(batt_psy, POWER_SUPPLY_PROP_TEMP);
	ibatt = PSY_GET_PROP(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	vbatt = PSY_GET_PROP(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	chg_type = PSY_GET_PROP(batt_psy, POWER_SUPPLY_PROP_CHARGE_TYPE);

	usb_present = PSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_PRESENT);
	usb_type = PSY_GET_PROP(usb_psy, POWER_SUPPLY_PROP_REAL_TYPE);

	if (temp == -EINVAL || ibatt == -EINVAL ||
	    vbatt == -EINVAL || chg_type == -EINVAL ||
	    usb_type == -EINVAL || usb_present == -EINVAL)
		goto error_rerun;

	pr_info("vbatt=%d, ibatt=%d, temp=%d, chg_type=%s, usb=%d usb_type=%s\n",
		vbatt, ibatt, temp, psy_charge_type_str[chg_type], usb_present,
		psy_usb_type_str[usb_type]);

	if (!usb_present)
		goto check_rerun;

	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1]) {
		if (!chg_drv->stop_charging) {
			pr_info("batt. temp. too high, disabling charging\n");
			PSY_SET_PROP(batt_psy,
				     POWER_SUPPLY_PROP_INPUT_SUSPEND, 1);
			chg_drv->stop_charging = true;
			chg_drv->temp_idx = -1;
			chg_drv->vbatt_idx = -1;
			chg_drv->chg_type = -1;
			chg_drv->checked_cv_cnt = 0;
		}
		/* status will be discharging when disabled but we want to keep
		 * monitoring temperature to re-enable charging
		 */
		rerun_work = true;
		goto handle_rerun;
	} else if (chg_drv->stop_charging) {
		pr_info("batt. temp. ok, enabling charging\n");
		PSY_SET_PROP(batt_psy, POWER_SUPPLY_PROP_INPUT_SUSPEND, 0);
		chg_drv->stop_charging = false;
	}

	/* 1. charge profile idx based on the battery temperature */
	while (temp_idx < profile->temp_nb_limits - 1 &&
	       temp >= profile->temp_limits[temp_idx + 1])
		temp_idx++;

	/* 2. compute the step index given the battery voltage  */
	while (vbatt_idx < profile->volt_nb_limits &&
	       vbatt > profile->volt_limits[vbatt_idx])
		vbatt_idx++;
	/* voltage index cannot go down once a CV step is crossed */
	if (vbatt_idx < chg_drv->vbatt_idx)
		vbatt_idx = chg_drv->vbatt_idx;

	/* 3. get our current step CC CV limits */
	cc_max = get_cccm_limits(profile, temp_idx, vbatt_idx);
	fv_uv = profile->volt_limits[vbatt_idx];

	/* 4. get next step CC value */
	if (vbatt_idx == profile->volt_nb_limits - 1)
		cc_next_max = 0;
	else
		cc_next_max = get_cccm_limits(profile, temp_idx, vbatt_idx + 1);

	/* 5. check if CV step limit has been reached */
	if (chg_drv->vbatt_idx != -1) {
		cv_delta = abs(fv_uv - vbatt);
		if (cv_delta <= CV_RANGE_ACCURACY)
			chg_drv->checked_cv_cnt++;
		else
			chg_drv->checked_cv_cnt = 0;
	}

	/* 5. in case CC has lowered to next step, move to next CV step */
	if (chg_drv->checked_cv_cnt >= CV_RANGE_ACCURACY_CNT &&
	    cc_next_max != 0 && ibatt < 0 && -ibatt <= cc_next_max) {
		pr_debug("CV step done: vbatt_idx:%d->%d -ibatt=%d <= cc_next_max=%d\n",
			vbatt_idx, vbatt_idx + 1, -ibatt, cc_next_max);
		vbatt_idx++;
		chg_drv->checked_cv_cnt = 0;
		cc_max = cc_next_max;
		cc_next_max = -1;
		fv_uv = profile->volt_limits[vbatt_idx];
	}

	pr_info("temp_idx:%d->%d, vbatt_idx:%d->%d, fv=%d, cc_max=%d, cc_next_max=%d\n",
		chg_drv->temp_idx, temp_idx, chg_drv->vbatt_idx, vbatt_idx,
		fv_uv, cc_max, cc_next_max);

	if (chg_drv->vbatt_idx != vbatt_idx || chg_drv->temp_idx != temp_idx) {
		if (PSY_SET_PROP(batt_psy,
				 POWER_SUPPLY_PROP_VOLTAGE_MAX,
				 fv_uv))
			goto error_rerun;

		if (PSY_SET_PROP(batt_psy,
				 POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				 cc_max))
			goto error_rerun;
		chg_drv->temp_idx = temp_idx;
		chg_drv->vbatt_idx = vbatt_idx;
		chg_drv->checked_cv_cnt = 0;
	}

check_rerun:
	batt_status = PSY_GET_PROP(batt_psy, POWER_SUPPLY_PROP_STATUS);

	switch (batt_status) {
	case POWER_SUPPLY_STATUS_DISCHARGING:
		rerun_work = false;
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
	case POWER_SUPPLY_STATUS_FULL:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		rerun_work = true;
		break;
	case POWER_SUPPLY_STATUS_UNKNOWN:
		pr_err("chg_work charging status UNKNOWN\n");
		goto error_rerun;
	default:
		pr_err("chg_work invalid charging status %d\n", val.intval);
		goto error_rerun;
	}

handle_rerun:
	if (rerun_work) {
		pr_debug("rerun battery charging work in %d ms\n",
			 chg_drv->chg_profile.update_interval);
		schedule_delayed_work(&chg_drv->chg_work,
			msecs_to_jiffies(chg_drv->
					 chg_profile.update_interval));
	} else {
		pr_info("stop battery charging work\n");
		chg_drv->temp_idx = -1;
		chg_drv->vbatt_idx = -1;
		chg_drv->chg_type = -1;
		chg_drv->checked_cv_cnt = 0;
		chg_drv->chg_work_running = false;
		__pm_relax(&chg_drv->chg_ws);
	}

	return;

error_rerun:
	pr_err("error occurred, rerun battery charging work in %d ms\n",
	       CHG_WORK_ERROR_RETRY_MS);
	schedule_delayed_work(&chg_drv->chg_work,
			      msecs_to_jiffies(CHG_WORK_ERROR_RETRY_MS));
}

static void dump_profile(struct chg_profile *profile)
{
	int ti, vi, count, len = 256;
	char buff[len];

	count = 0;
	for (ti = 0; ti < profile->temp_nb_limits; ti++) {
		count += scnprintf(buff + count, len - count, " %3d",
				   profile->temp_limits[ti]);
	}
	pr_info("Profile temperature limits: %s\n", buff);

	count = 0;
	for (vi = 0; vi < profile->volt_nb_limits; vi++) {
		count += scnprintf(buff + count, len - count, " %d",
				   profile->volt_limits[vi]);
	}
	pr_info("Profile voltage limits: %s\n", buff);

	pr_info("Profile constant charge limits:\n");
	for (ti = 0; ti < profile->temp_nb_limits - 1; ti++) {
		count = 0;
		for (vi = 0; vi < profile->volt_nb_limits; vi++) {
			count += scnprintf(buff + count, len - count, " %7d",
					   get_cccm_limits(profile, ti, vi));
		}
		pr_info("%s\n", buff);
	}
}

static int chg_init_chg_profile(struct chg_drv *chg_drv)
{
	struct device *dev = chg_drv->device;
	struct device_node *node = dev->of_node;
	struct chg_profile *profile = &chg_drv->chg_profile;
	u32 cccm_array_size;
	int ret = 0;

	ret = of_property_read_u32(node, "google,chg-update-interval",
				   &profile->update_interval);
	if (ret < 0) {
		pr_err("cannot read chg-update-interval, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &profile->battery_capacity);
	if (ret < 0) {
		pr_err("cannot read chg-battery-capacity, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,chg-temp-nb-limits",
				   &profile->temp_nb_limits);
	if (ret < 0) {
		pr_err("cannot read chg-temp-nb-limits, ret=%d\n", ret);
		return ret;
	}
	if (profile->temp_nb_limits > CHG_TEMP_NB_LIMITS_MAX) {
		pr_err("chg-temp-nb-limits exceeds driver max: %d\n",
		       CHG_TEMP_NB_LIMITS_MAX);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(node, "google,chg-temp-limits",
					 profile->temp_limits,
					 profile->temp_nb_limits);
	if (ret < 0) {
		pr_err("cannot read chg-temp-limits table, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,chg-cv-nb-limits",
				   &profile->volt_nb_limits);
	if (ret < 0) {
		pr_err("cannot read chg-cv-nb-limits, ret=%d\n", ret);
		return ret;
	}
	if (profile->volt_nb_limits > CHG_VOLT_NB_LIMITS_MAX) {
		pr_err("chg-cv-nb-limits exceeds driver max: %d\n",
		       CHG_VOLT_NB_LIMITS_MAX);
		return -EINVAL;
	}
	ret = of_property_read_u32_array(node, "google,chg-cv-limits",
					 profile->volt_limits,
					 profile->volt_nb_limits);
	if (ret < 0) {
		pr_err("cannot read chg-cv-limits table, ret=%d\n", ret);
		return ret;
	}

	cccm_array_size =
	    (profile->temp_nb_limits - 1) * profile->volt_nb_limits;
	profile->cccm_limits = devm_kzalloc(dev,
					    sizeof(s32) * cccm_array_size,
					    GFP_KERNEL);

	ret = of_property_read_u32_array(node, "google,chg-cc-limits",
					 profile->cccm_limits, cccm_array_size);
	if (ret < 0) {
		pr_err("cannot read chg-cc-limits table, ret=%d\n", ret);
		return ret;
	}

	pr_info("successfully read charging profile:\n");
	dump_profile(profile);

	return ret;
}

static int google_charger_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct chg_drv *chg_drv;
	struct power_supply *batt_psy, *bms_psy, *usb_psy;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		dev_info(&pdev->dev,
			 "failed to get \"battery\" power supply\n");
		return -EPROBE_DEFER;
	}
	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_info(&pdev->dev, "failed to get \"bms\" power supply\n");
		return -EPROBE_DEFER;
	}
	usb_psy = power_supply_get_by_name("usb");
	if (!bms_psy) {
		dev_info(&pdev->dev, "failed to get \"usb\" power supply\n");
		return -EPROBE_DEFER;
	}

	chg_drv = devm_kzalloc(&pdev->dev, sizeof(*chg_drv), GFP_KERNEL);
	if (!chg_drv)
		return -ENOMEM;
	chg_drv->device = &pdev->dev;
	chg_drv->batt_psy = batt_psy;
	chg_drv->bms_psy = bms_psy;
	chg_drv->usb_psy = usb_psy;
	ret = chg_init_chg_profile(chg_drv);
	if (ret < 0) {
		pr_err("cannot read charging profile from dt, ret=%d\n", ret);
		return ret;
	}
	INIT_DELAYED_WORK(&chg_drv->chg_work, chg_work);
	wakeup_source_init(&chg_drv->chg_ws, "google-charger");
	chg_drv->temp_idx = -1;
	chg_drv->vbatt_idx = -1;
	chg_drv->chg_type = -1;

	chg_drv->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chg_drv->psy_nb);
	if (ret < 0) {
		pr_err("Cannot register power supply notifer, ret=%d\n", ret);
		return ret;
	}
	dev_info(&pdev->dev, "probe done\n");

	return 0;
}

static int google_charger_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id match_table[] = {
	{.compatible = "google,charger"},
	{},
};

static struct platform_driver charger_driver = {
	.driver = {
		   .name = "google,charger",
		   .owner = THIS_MODULE,
		   .of_match_table = match_table,
		   },
	.probe = google_charger_probe,
	.remove = google_charger_remove,
};

static int __init google_charger_init(void)
{
	int ret;

	ret = platform_driver_register(&charger_driver);
	if (ret < 0) {
		pr_err("device registration failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static void __init google_charger_exit(void)
{
	platform_driver_unregister(&charger_driver);
	pr_info("unregistered platform driver\n");
}

module_init(google_charger_init);
module_exit(google_charger_exit);
MODULE_DESCRIPTION("Multi-step battery charger driver");
MODULE_AUTHOR("Thierry Strudel <tstrudel@google.com>");
MODULE_LICENSE("GPL");

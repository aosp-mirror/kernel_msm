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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/thermal.h>
#include <linux/time.h>
#include <linux/pm_wakeup.h>

#define USB_OVERHEAT_MITIGATION_VOTER	"USB_OVERHEAT_MITIGATION_VOTER"

static int fake_port_temp = -1;
module_param_named(
	fake_port_temp, fake_port_temp, int, 0600
);

static bool enable = true;
module_param_named(
	enable, enable, bool, 0600
);

struct overheat_info {
	struct device              *dev;
	struct power_supply        *usb_psy;
	struct votable             *usb_icl_votable;
	struct votable             *disable_power_role_switch;
	struct notifier_block      psy_nb;
	struct delayed_work        port_overheat_work;
	struct wakeup_source	   overheat_ws;

	bool usb_connected;
	bool accessory_connected;
	bool usb_replug;
	bool overheat_mitigation;
	bool overheat_work_running;

	int begin_temp;
	int clear_temp;
	int overheat_work_delay_ms;
	int polling_freq;
	int monitor_accessory_s;
	time_t accessory_connect_time;
	int check_status;
};

#define PSY_GET_PROP(psy, psp) psy_get_prop(psy, psp, #psp)
static inline int psy_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       char *prop_name)
{
	union power_supply_propval val;
	int ret = 0;

	if (!psy)
		return -EINVAL;
	pr_debug("get %s for '%s'...\n", prop_name, psy->desc->name);
	ret = power_supply_get_property(psy, psp, &val);
	if (ret < 0) {
		pr_err("failed to get %s from '%s', ret=%d\n", prop_name,
		       psy->desc->name, ret);
		return ret;
	}
	pr_debug("get %s for '%s' => %d\n", prop_name, psy->desc->name,
		 val.intval);
	return val.intval;
}

static inline int get_dts_vars(struct overheat_info *ovh_info)
{
	struct device *dev = ovh_info->dev;
	struct device_node *node = dev->of_node;
	int ret;

	ret = of_property_read_u32(node, "google,begin-mitigation-temp",
				   &ovh_info->begin_temp);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read begin-mitigation-temp, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,end-mitigation-temp",
				   &ovh_info->clear_temp);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read end-mitigation-temp, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,port-overheat-work-interval",
				   &ovh_info->overheat_work_delay_ms);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read port-overheat-work-interval, ret=%d\n",
			ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,polling-freq",
				   &ovh_info->polling_freq);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read polling-freq, ret=%d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(node, "google,accessory-monitoring-period",
				   &ovh_info->monitor_accessory_s);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"cannot read accessory-monitoring-period, ret=%d\n",
			ret);
		ovh_info->monitor_accessory_s = 5;
	}

	return 0;
}

static int suspend_usb(struct overheat_info *ovh_info)
{
	int ret;

	ovh_info->usb_replug = false;

	/* disable USB */
	ret = vote(ovh_info->disable_power_role_switch,
		   USB_OVERHEAT_MITIGATION_VOTER, true, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't vote for disable_power_role_switch ret=%d\n",
			ret);
		return ret;
	}

	/* suspend charging */
	ret = vote(ovh_info->usb_icl_votable,
		  USB_OVERHEAT_MITIGATION_VOTER, true, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't vote for USB ICL ret=%d\n", ret);
		return ret;
	}

	ovh_info->overheat_mitigation = true;
	return ret;
}

static int resume_usb(struct overheat_info *ovh_info)
{
	int ret;

	/* enable charging */
	ret = vote(ovh_info->usb_icl_votable,
		  USB_OVERHEAT_MITIGATION_VOTER, false, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't un-vote for USB ICL ret=%d\n", ret);
		return ret;
	}

	/* enable USB */
	ret = vote(ovh_info->disable_power_role_switch,
		   USB_OVERHEAT_MITIGATION_VOTER, false, 0);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Couldn't un-vote for disable_power_role_switch ret=%d\n",
			ret);
		return ret;
	}

	ovh_info->overheat_mitigation = false;
	ovh_info->usb_replug = false;
	return ret;
}

static inline time_t get_seconds_since_boot(void)
{
	struct timespec boot;

	getboottime(&boot);
	return get_seconds() - boot.tv_sec;
}

/*
 * Update usb_connected, accessory_connected, and usb_replug status in
 * overheat_info struct.
 */
static int update_usb_status(struct overheat_info *ovh_info)
{
	int ret;
	bool prev_state = ovh_info->usb_connected ||
		ovh_info->accessory_connected;
	bool prev_accessory_state = ovh_info->accessory_connected;
	bool curr_state;
	int *check_status = &ovh_info->check_status;

	if (ovh_info->overheat_mitigation) {
		// Only check USB status every polling_freq instances
		*check_status = (*check_status + 1) % ovh_info->polling_freq;
		if (*check_status > 0)
			return 0;
		ret = vote(ovh_info->disable_power_role_switch,
			   USB_OVERHEAT_MITIGATION_VOTER, false, 0);
		if (ret < 0) {
			dev_err(ovh_info->dev,
				"Couldn't un-vote for disable_power_role_switch ret=%d\n",
				ret);
			return ret;
		}
		msleep(200);
	}

	dev_dbg(ovh_info->dev, "Updating USB connected status\n");
	ret = PSY_GET_PROP(ovh_info->usb_psy, POWER_SUPPLY_PROP_ONLINE);
	if (ret < 0)
		return ret;
	ovh_info->usb_connected = ret;

	ret = PSY_GET_PROP(ovh_info->usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE);
	if (ret < 0)
		return ret;
	ovh_info->accessory_connected = (ret == POWER_SUPPLY_TYPEC_SINK) ||
			(ret == POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE);

	if (!prev_accessory_state && ovh_info->accessory_connected)
		ovh_info->accessory_connect_time = get_seconds_since_boot();
	curr_state = ovh_info->usb_connected || ovh_info->accessory_connected;

	if (ovh_info->overheat_mitigation) {
		ret = vote(ovh_info->disable_power_role_switch,
			   USB_OVERHEAT_MITIGATION_VOTER, true, 0);
		if (ret < 0) {
			dev_err(ovh_info->dev,
				"Couldn't un-vote for disable_power_role_switch ret=%d\n",
				ret);
			return ret;
		}
	}

	if (curr_state != prev_state)
		dev_info(ovh_info->dev,
			 "USB is %sconnected",
			 curr_state ? "" : "dis");

	// USB should be disconnected for two cycles before replug is acked
	if (ovh_info->overheat_mitigation && !curr_state && !prev_state)
		ovh_info->usb_replug = true;

	return 0;
}

static inline int get_usb_port_temp(struct overheat_info *ovh_info)
{
	int temp;

	temp = PSY_GET_PROP(ovh_info->usb_psy, POWER_SUPPLY_PROP_TEMP);

	if (fake_port_temp > 0)
		temp = fake_port_temp;

	if (ovh_info->overheat_mitigation || temp >= ovh_info->begin_temp)
		dev_info(ovh_info->dev, "Overheat triggered: USB port temp is %d\n",
			 temp);
	return temp;
}

static int psy_changed(struct notifier_block *nb, unsigned long action,
		       void *data)
{
	struct power_supply *psy = data;
	struct overheat_info *ovh_info =
			container_of(nb, struct overheat_info, psy_nb);

	if ((action != PSY_EVENT_PROP_CHANGED) || (psy == NULL) ||
	    (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    !strcmp(psy->desc->name, "usb")) {
		dev_dbg(ovh_info->dev, "name=usb evt=%lu\n", action);
		if (!ovh_info->overheat_work_running)
			schedule_delayed_work(&ovh_info->port_overheat_work, 0);
	}
	return NOTIFY_OK;
}

static bool should_check_accessory(struct overheat_info *ovh_info)
{
	time_t connected;
	bool ret;

	if (!ovh_info->accessory_connected)
		return false;

	connected =
		get_seconds_since_boot() - ovh_info->accessory_connect_time;
	ret = (connected < ovh_info->monitor_accessory_s);
	if (!ret)
		dev_info(ovh_info->dev,
			 "Stop monitoring: %d sec since USB accessory connected",
			 (int) connected);
	return ret;
}

static void port_overheat_work(struct work_struct *work)
{
	struct overheat_info *ovh_info =
			container_of(work, struct overheat_info,
				     port_overheat_work.work);
	int temp = 0, ret = 0;

	// Take a wake lock to ensure we poll the temp regularly
	if (!ovh_info->overheat_work_running)
		__pm_stay_awake(&ovh_info->overheat_ws);
	ovh_info->overheat_work_running = true;

	if (enable) {
		temp = get_usb_port_temp(ovh_info);
		if (temp < 0)
			goto rerun;

		// Check USB connection status if it's safe to do so
		if (!ovh_info->overheat_mitigation ||
		    temp < ovh_info->clear_temp) {
			ret = update_usb_status(ovh_info);
			if (ret < 0)
				goto rerun;
		}
	}

	if (ovh_info->overheat_mitigation && (!enable ||
	    (temp < ovh_info->clear_temp && ovh_info->usb_replug))) {
		dev_err(ovh_info->dev, "Port overheat mitigated\n");
		resume_usb(ovh_info);
	} else if (!ovh_info->overheat_mitigation &&
		 enable && temp > ovh_info->begin_temp) {
		dev_err(ovh_info->dev, "Port overheat triggered\n");
		suspend_usb(ovh_info);
		goto rerun;
	}

	if (ovh_info->overheat_mitigation || ovh_info->usb_connected ||
	    should_check_accessory(ovh_info))
		goto rerun;
	// Do not run again, USB port isn't overheated or connected to something
	ovh_info->overheat_work_running = false;
	__pm_relax(&ovh_info->overheat_ws);
	return;

rerun:
	schedule_delayed_work(
			&ovh_info->port_overheat_work,
			msecs_to_jiffies(ovh_info->overheat_work_delay_ms));
}

static int ovh_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct overheat_info *ovh_info;
	struct power_supply  *usb_psy;
	struct votable       *usb_icl_votable;
	struct votable       *disable_power_role_switch;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy)
		return -EPROBE_DEFER;

	usb_icl_votable = find_votable("USB_ICL");
	if (usb_icl_votable == NULL) {
		pr_err("Couldn't find USB_ICL votable\n");
		return -EPROBE_DEFER;
	}

	disable_power_role_switch = find_votable("DISABLE_POWER_ROLE_SWITCH");
	if (disable_power_role_switch == NULL) {
		pr_err("Couldn't find DISABLE_POWER_ROLE_SWITCH votable\n");
		return -EPROBE_DEFER;
	}

	ovh_info = devm_kzalloc(&pdev->dev, sizeof(*ovh_info), GFP_KERNEL);
	if (!ovh_info)
		return -ENOMEM;

	ovh_info->dev = &pdev->dev;
	ovh_info->usb_icl_votable = usb_icl_votable;
	ovh_info->disable_power_role_switch = disable_power_role_switch;
	ovh_info->usb_psy = usb_psy;
	ovh_info->overheat_mitigation = false;
	ovh_info->usb_replug = false;
	ovh_info->usb_connected = false;
	ovh_info->overheat_work_running = false;

	ret = get_dts_vars(ovh_info);
	if (ret < 0)
		return -ENODEV;

	// initialize votables
	vote(ovh_info->usb_icl_votable,
	     USB_OVERHEAT_MITIGATION_VOTER, false, 0);
	vote(ovh_info->disable_power_role_switch,
	     USB_OVERHEAT_MITIGATION_VOTER, false, 0);

	// register power supply change notifier
	ovh_info->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&ovh_info->psy_nb);
	if (ret < 0) {
		dev_err(ovh_info->dev,
			"Cannot register power supply notifer, ret=%d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, ovh_info);
	wakeup_source_init(&ovh_info->overheat_ws, "overheat_mitigation");
	INIT_DELAYED_WORK(&ovh_info->port_overheat_work, port_overheat_work);
	schedule_delayed_work(&ovh_info->port_overheat_work, 0);

	return ret;
}

static int ovh_remove(struct platform_device *pdev)
{
	struct overheat_info *ovh_info = platform_get_drvdata(pdev);
	if (ovh_info)
		wakeup_source_trash(&ovh_info->overheat_ws);

	return 0;
}

static const struct of_device_id match_table[] = {
	{
		.compatible = "google,overheat_mitigation",
	},
	{},
};

static struct platform_driver ovh_driver = {
	.driver = {
		.name = "google,overheat_mitigation",
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
	.probe = ovh_probe,
	.remove = ovh_remove,
};

module_platform_driver(ovh_driver);
MODULE_DESCRIPTION("USB port overheat mitigation driver");
MODULE_AUTHOR("Maggie White <maggiewhite@google.com>");
MODULE_LICENSE("GPL v2");

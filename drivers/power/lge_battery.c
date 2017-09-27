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

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#define BATT_DRV_NAME	"lge_battery"

#define pr_bm(reason, format, ...)					\
	do {								\
		if (debug_mask & (reason))				\
			pr_info("%s: %s: " format, BATT_DRV_NAME,	\
					__func__, ##__VA_ARGS__);	\
		else							\
			pr_debug("%s: %s: " format, BATT_DRV_NAME,	\
					__func__, ##__VA_ARGS__);	\
	} while (0)

#define NORM_VOLT			4400000
#define LIM_VOLT			4100000
#define PARALLEL_VOLT			4450000
#define SC_VOLT				4200000
#define CHG_CURRENT_MAX			3550000
#define SC_CURRENT			2400000
#define LCD_ON_CURRENT			1000000
#define WATCH_DELAY			30000
#define DEMO_MODE_MAX			35
#define DEMO_MODE_MIN			30

enum debug_mask_print {
	ASSERT = BIT(0),
	ERROR = BIT(1),
	INTERRUPT = BIT(2),
	REGISTER = BIT(3),
	MISC = BIT(4),
	VERBOSE = BIT(5),
};

enum bm_vote_reason {
	BM_REASON_DEFAULT,
	BM_REASON_LCD,
	BM_REASON_STEP,
	BM_REASON_THERM,
	BM_REASON_DEMO,
	BM_REASON_MAX,
};

enum bm_therm_states {
	BM_HEALTH_COLD,
	BM_HEALTH_COOL,
	BM_HEALTH_GOOD,
	BM_HEALTH_WARM,
	BM_HEALTH_HOT,
	BM_HEALTH_WARM_LIM,
	BM_HEALTH_MAX,
};

struct battery_manager {
	struct device			*dev;
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*pl_psy;
	struct notifier_block		ps_nb;
	struct notifier_block		fb_nb;
	struct work_struct		bm_batt_update;
	struct work_struct		bm_usb_update;
	struct work_struct		bm_fb_update;
	struct delayed_work		bm_watch;
	struct wake_lock		chg_wake_lock;
	struct mutex			work_lock;

	enum bm_therm_states		therm_stat;
	int		chg_present;
	int		chg_status;
	int		batt_soc;
	int		fb_state;
	int		bm_vote_fcc_reason;
	int		bm_vote_fcc_value;
	int		demo_iusb;
	int		demo_ibat;
	int		demo_enable;
	bool		bm_active;
	bool		sc_status;
};

struct bm_therm_table {
	int		min;
	int		max;
	int		cur;
};

static struct bm_therm_table therm_table[BM_HEALTH_MAX] = {
	{  INT_MIN,       20,        0},
	{        0,      220,   710000},
	{      200,      450,  -EINVAL},
	{      430,      550,   710000},
	{      530,  INT_MAX,        0},
	{      430,      550,        0},
};

static int bm_vote_fcc_table[BM_REASON_MAX] = {
	CHG_CURRENT_MAX,
	-EINVAL,
	-EINVAL,
	-EINVAL,
	-EINVAL,
};

static int debug_mask = ERROR | INTERRUPT | MISC | VERBOSE;
static int demo_mode;

static int bm_get_property(struct power_supply *psy,
			   enum power_supply_property prop, int *value)
{
	union power_supply_propval val = {0, };
	int rc = 0;

	if (!psy) {
		pr_bm(ERROR, "Couldn't get psy\n");
		return -EINVAL;
	}

	rc = power_supply_get_property(psy, prop, &val);
	if (rc < 0) {
		pr_bm(ERROR, "Couldn't get property %d, rc=%d\n", prop, rc);
		return rc;
	}

	*value = val.intval;
	return rc;
}

static int bm_set_property(struct power_supply *psy,
			   enum power_supply_property prop, int value)
{
	union power_supply_propval val = {0, };
	int rc = 0;

	if (!psy) {
		pr_bm(ERROR, "Couldn't get psy\n");
		return -EINVAL;
	}

	val.intval = value;
	rc = power_supply_set_property(psy, prop, &val);
	if (rc < 0)
		pr_bm(ERROR, "Couldn't set property %d, rc=%d\n", prop, rc);

	return rc;
}

static int bm_vote_fcc_update(struct battery_manager *bm)
{
	int fcc = INT_MAX;
	int reason = -EINVAL;
	int i, rc = 0;

	for (i = 0; i < BM_REASON_MAX; i++) {
		if (bm_vote_fcc_table[i] == -EINVAL)
			continue;
		if (fcc > bm_vote_fcc_table[i]) {
			fcc = bm_vote_fcc_table[i];
			reason = i;
		}
	}

	if (reason != bm->bm_vote_fcc_reason || fcc != bm->bm_vote_fcc_value) {
		if (fcc != bm->bm_vote_fcc_value) {
			rc = bm_set_property(
				bm->batt_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				fcc);
			if (rc < 0) {
				pr_bm(ERROR,
				      "Couldn't set current, rc=%d\n", rc);
				return rc;
			}
			rc = bm_set_property(
				bm->batt_psy,
				POWER_SUPPLY_PROP_CHARGE_DISABLE,
				(fcc == 0 ? 1 : 0));
			if (rc < 0) {
				pr_bm(ERROR,
				      "Couldn't charge disable, rc=%d\n", rc);
				return rc;
			}
		}
		bm->bm_vote_fcc_reason = reason;
		bm->bm_vote_fcc_value = fcc;
		pr_bm(MISC, "vote id[%d], set cur[%d]\n", reason, fcc);
	}

	return rc;
}

static int bm_vote_fcc(struct battery_manager *bm, int reason, int fcc)
{
	int rc = 0;

	bm_vote_fcc_table[reason] = fcc;
	rc = bm_vote_fcc_update(bm);
	if (rc < 0) {
		pr_bm(ERROR, "Couldn't vote id[%d] set cur[%d], rc=%d\n",
		      reason, fcc, rc);
		bm_vote_fcc_table[reason] = -EINVAL;
	}

	return rc;
}

static int bm_vote_fcc_get(struct battery_manager *bm)
{
	if (bm->bm_vote_fcc_reason == -EINVAL)
		return -EINVAL;

	return bm_vote_fcc_table[bm->bm_vote_fcc_reason];
}

void bm_check_demo_mode(struct battery_manager *bm)
{
	int rc = 0;
	int before_demo_iusb = bm->demo_iusb;
	int before_demo_ibat = bm->demo_ibat;

	if (!demo_mode || !bm->bm_active) {
		bm->demo_iusb = 1;
		bm->demo_ibat = 1;
	} else {
		if (bm->batt_soc > DEMO_MODE_MAX) {
			bm->demo_iusb = 0;
			bm->demo_ibat = 0;
		} else if (bm->batt_soc >= DEMO_MODE_MAX) {
			bm->demo_iusb = 1;
			bm->demo_ibat = 0;
		} else if (bm->batt_soc < DEMO_MODE_MIN) {
			bm->demo_iusb = 1;
			bm->demo_ibat = 1;
		}
	}

	if (bm->demo_ibat != before_demo_ibat) {
		if (!bm->demo_ibat)
			rc = bm_vote_fcc(bm, BM_REASON_DEMO, 0);
		else
			rc = bm_vote_fcc(bm, BM_REASON_DEMO, -EINVAL);

		if (rc < 0) {
			bm->demo_iusb = before_demo_iusb;
			bm->demo_ibat = before_demo_ibat;
			pr_bm(ERROR, "Couldn't set ibat for demo rc=%d\n", rc);
			return;
		}
	}

	if (bm->demo_iusb != before_demo_iusb) {
		if (!bm->demo_iusb)
			rc = bm_set_property(bm->batt_psy,
					     POWER_SUPPLY_PROP_INPUT_SUSPEND,
					     1);
		else
			rc = bm_set_property(bm->batt_psy,
					     POWER_SUPPLY_PROP_INPUT_SUSPEND,
					     0);
		if (rc < 0) {
			bm->demo_iusb = before_demo_iusb;
			pr_bm(ERROR, "Couldn't set iusb for demo rc=%d\n", rc);
			return;
		}
	}

	bm->demo_enable = demo_mode;
}

void bm_check_therm_charging(struct battery_manager *bm,
			     int batt_temp, int batt_volt)
{
	enum bm_therm_states stat;
	int i, rc = 0;

	if (bm->therm_stat == BM_HEALTH_WARM_LIM)
		stat = BM_HEALTH_WARM;
	else
		stat = bm->therm_stat;

	for (i = 0; i < BM_HEALTH_MAX - 1; i++) {
		if (batt_temp < therm_table[stat].min)
			stat--;
		else if (batt_temp >= therm_table[stat].max)
			stat++;
		else
			break;
	}

	if (stat == BM_HEALTH_WARM && batt_volt >= LIM_VOLT) {
		stat = BM_HEALTH_WARM_LIM;
	}

	if (bm->therm_stat != stat) {
		pr_bm(MISC, "STATE[%d->%d] TEMP[%d] CUR[%d] VOL[%d]\n",
		      bm->therm_stat, stat, batt_temp,
		      therm_table[stat].cur, batt_volt);
		rc = bm_vote_fcc(bm, BM_REASON_THERM, therm_table[stat].cur);
		if (rc < 0) {
			pr_bm(ERROR, "Couldn't set ibat current rc=%d\n", rc);
			return;
		}
		bm->therm_stat = stat;
	}
}

void bm_check_step_charging(struct battery_manager *bm, int volt)
{
	int rc = 0;

	if (!bm->bm_active) {
		if (bm->sc_status) {
			rc = bm_vote_fcc(bm, BM_REASON_STEP, -EINVAL);
			if (rc < 0) {
				pr_bm(ERROR,
				      "Couldn't set ibat curr rc=%d\n", rc);
				return;
			}
			bm->sc_status = false;
		}
		return;
	}

	if (!bm->sc_status && volt >= SC_VOLT) {
		rc = bm_vote_fcc(bm, BM_REASON_STEP, SC_CURRENT);
		if (rc < 0) {
			pr_bm(ERROR, "Couldn't set ibat curr rc=%d\n", rc);
			return;
		}
		bm->sc_status = true;
	}
}

static void bm_check_status(struct battery_manager *bm)
{
	int rc, vbus_out = 0;
	int chg_active = bm->bm_active;

	rc = bm_get_property(bm->usb_psy,
			     POWER_SUPPLY_PROP_USE_EXTERNAL_VBUS_OUTPUT,
			     &vbus_out);
	if (rc < 0)
		pr_bm(ERROR, "Couldn't get use_external_vbus_output=%d\n", rc);

	if (!bm->chg_present || (bm->chg_present && vbus_out) ||
	    (bm->chg_present && bm->chg_status == POWER_SUPPLY_STATUS_FULL)) {
		if (wake_lock_active(&bm->chg_wake_lock)) {
			bm->bm_active = false;
			wake_unlock(&bm->chg_wake_lock);
		}
	} else if (bm->chg_present &&
		   bm->chg_status != POWER_SUPPLY_STATUS_FULL) {
		if (!vbus_out && !wake_lock_active(&bm->chg_wake_lock)) {
			bm->bm_active = true;
			wake_lock(&bm->chg_wake_lock);
		}
	}

	if (bm->bm_active != chg_active)
		pr_bm(MISC, "wake_%s: present[%d] chg_state[%d] vbus[%d]",
		      bm->bm_active ? "locked" : "unlocked", bm->chg_present,
		      bm->chg_status, vbus_out);
}

static void bm_watch_work(struct work_struct *work)
{
	struct battery_manager *bm = container_of(work,
						struct battery_manager,
						bm_watch.work);
	int rc, batt_volt, batt_temp, ibat_max = 0;

	mutex_lock(&bm->work_lock);

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_volt);
	if (rc < 0) {
		pr_bm(ERROR, "Couldn't do bm_check_step_charging=%d\n", rc);
		goto error;
	}

	if (bm->bm_active)
		bm_check_step_charging(bm, batt_volt);

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_TEMP, &batt_temp);
	if (rc < 0) {
		pr_bm(ERROR, "Couldn't do bm_check_therm_charging=%d\n", rc);
		goto error;
	}

	if (bm->bm_active)
		bm_check_therm_charging(bm, batt_temp, batt_volt);

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
			     &ibat_max);

	pr_bm(VERBOSE, "PRESENT:%d, CHG_STAT:%d, THM_STAT:%d, " \
		       "BAT_TEMP:%d, BAT_VOLT:%d, VOTE_CUR:%d, SET_CUR:%d,\n",
	      bm->chg_present, bm->chg_status, bm->therm_stat,
	      batt_temp, batt_volt, bm_vote_fcc_get(bm), ibat_max);

error:
	mutex_unlock(&bm->work_lock);

	if (bm->therm_stat >= BM_HEALTH_WARM)
		schedule_delayed_work(&bm->bm_watch,
				      msecs_to_jiffies(WATCH_DELAY / 3));
	else
		schedule_delayed_work(&bm->bm_watch,
				      msecs_to_jiffies(WATCH_DELAY));
}

static void bm_batt_update_work(struct work_struct *work)
{
	struct battery_manager *bm = container_of(work,
						struct battery_manager,
						bm_batt_update);
	int rc, batt_soc = bm->batt_soc;

	mutex_lock(&bm->work_lock);

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_STATUS, &bm->chg_status);
	if (rc < 0)
		goto error;

	bm_check_status(bm);

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_CAPACITY, &bm->batt_soc);
	if (rc < 0)
		goto error;

	if ((bm->demo_enable != demo_mode) ||
	    (demo_mode && (bm->batt_soc != batt_soc)))
		bm_check_demo_mode(bm);

error:
	mutex_unlock(&bm->work_lock);
}

static void bm_usb_update_work(struct work_struct *work)
{
	struct battery_manager *bm = container_of(work,
						struct battery_manager,
						bm_usb_update);
	int rc, chg_active = bm->bm_active;

	mutex_lock(&bm->work_lock);

	rc = bm_get_property(bm->usb_psy,
			     POWER_SUPPLY_PROP_PRESENT, &bm->chg_present);
	if (rc < 0)
		goto error;

	bm_check_status(bm);

	if (bm->bm_active != chg_active) {
		if (!bm->bm_active) {
			bm_check_step_charging(bm, 0);
			bm_check_therm_charging(bm, 250, 0);
		}

		if (bm->demo_enable)
			bm_check_demo_mode(bm);
	}

error:
	mutex_unlock(&bm->work_lock);
}

static int bm_ps_notifier_call(struct notifier_block *nb,
			       unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct battery_manager *bm = container_of(nb,
						struct battery_manager,
						ps_nb);

	if (!strcmp(psy->desc->name, "battery")) {
		if (!bm->batt_psy)
			bm->batt_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED && bm->batt_psy)
			schedule_work(&bm->bm_batt_update);
	}

	if (!strcmp(psy->desc->name, "usb")) {
		if (!bm->usb_psy)
			bm->usb_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED && bm->usb_psy)
			schedule_work(&bm->bm_usb_update);
	}
	return NOTIFY_OK;
}

static int bm_ps_register_notifier(struct battery_manager *bm)
{
	int rc = 0;

	bm->ps_nb.notifier_call = bm_ps_notifier_call;
	rc = power_supply_reg_notifier(&bm->ps_nb);
	if (rc < 0)
		pr_bm(ERROR, "Couldn't register bm notifier = %d", rc);

	return rc;
}

static void bm_fb_update_work(struct work_struct *work)
{
	struct battery_manager *bm = container_of(work,
						struct battery_manager,
						bm_fb_update);
	mutex_lock(&bm->work_lock);

	if (!(bm->fb_state & BL_CORE_FBBLANK))
		bm_vote_fcc(bm, BM_REASON_LCD, LCD_ON_CURRENT);
	else
		bm_vote_fcc(bm, BM_REASON_LCD, -EINVAL);

	mutex_unlock(&bm->work_lock);
}

static int bm_fb_notifier_call(struct notifier_block *nb,
			       unsigned long ev, void *v)
{
	struct fb_event *evdata = v;
	struct battery_manager *bm = container_of(nb,
						struct battery_manager,
						fb_nb);
	int fb_blank = 0;

	if (ev != FB_EVENT_BLANK)
		return NOTIFY_OK;

	if (evdata && evdata->data) {
		fb_blank = *(int *)evdata->data;
		switch (fb_blank) {
		case FB_BLANK_UNBLANK:
			bm->fb_state &= ~BL_CORE_FBBLANK;
			break;
		case FB_BLANK_NORMAL:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_POWERDOWN:
			bm->fb_state |= BL_CORE_FBBLANK;
			break;
		default:
			pr_bm(ERROR, "not used evdata=%d\n", fb_blank);
			break;
		}
		schedule_work(&bm->bm_fb_update);
	}
	return NOTIFY_OK;
}

static int bm_fb_register_notifier(struct battery_manager *bm)
{
	int rc = 0;

	bm->fb_nb.notifier_call = bm_fb_notifier_call;
	rc = fb_register_client(&bm->fb_nb);
	if (rc < 0)
		pr_bm(ERROR, "Couldn't register bm notifier = %d\n", rc);

	return rc;
}

static int bm_init(struct battery_manager *bm)
{
	int rc, batt_temp, batt_volt = 0;

	bm->fb_state = 0;
	bm->therm_stat = BM_HEALTH_GOOD;
	bm->bm_vote_fcc_reason = -EINVAL;
	bm->bm_vote_fcc_value = -EINVAL;

	bm->batt_psy = power_supply_get_by_name("battery");
	if (!bm->batt_psy) {
		pr_bm(ERROR, "Couldn't get batt_psy\n");
		return -ENODEV;
	}

	bm->usb_psy = power_supply_get_by_name("usb");
	if (!bm->usb_psy) {
		pr_bm(ERROR, "Couldn't get usb_psy\n");
		return -ENODEV;
	}

	bm->pl_psy = power_supply_get_by_name("parallel");
	if (!bm->pl_psy) {
		pr_bm(ERROR, "Couldn't get pl_psy\n");
		return -ENODEV;
	}

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_STATUS, &bm->chg_status);
	if (rc < 0)
		bm->chg_status = 0;

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_TEMP, &batt_temp);
	if (rc < 0)
		batt_temp = 25;

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_volt);
	if (rc < 0)
		batt_volt = 4000000;

	rc = bm_get_property(bm->batt_psy,
			     POWER_SUPPLY_PROP_CAPACITY, &bm->batt_soc);
	if (rc < 0)
		bm->batt_soc = 50;

	rc = bm_get_property(bm->usb_psy,
			     POWER_SUPPLY_PROP_PRESENT, &bm->chg_present);
	if (rc < 0)
		bm->chg_present = 0;

	if (bm->chg_present) {
		bm->demo_iusb = 1;
		bm->demo_ibat = 1;
	} else {
		bm->demo_iusb = 0;
		bm->demo_ibat = 0;
	}

	rc = bm_set_property(bm->pl_psy,
			     POWER_SUPPLY_PROP_VOLTAGE_MAX,
			     PARALLEL_VOLT);
	if (rc < 0)
		pr_bm(ERROR, "Couldn't set pl float voltage, rc=%d", rc);

	INIT_WORK(&bm->bm_fb_update, bm_fb_update_work);
	INIT_WORK(&bm->bm_batt_update, bm_batt_update_work);
	INIT_WORK(&bm->bm_usb_update, bm_usb_update_work);
	INIT_DELAYED_WORK(&bm->bm_watch, bm_watch_work);

	mutex_init(&bm->work_lock);
	wake_lock_init(&bm->chg_wake_lock,
		       WAKE_LOCK_SUSPEND, "bm_wake_lock");

	if (bm->chg_present)
		bm_check_status(bm);

	if (bm->bm_active)
		bm_check_therm_charging(bm, batt_temp, batt_volt);

	schedule_delayed_work(&bm->bm_watch,
			      msecs_to_jiffies(WATCH_DELAY));

	return 0;
}

static int lge_battery_probe(struct platform_device *pdev)
{
	struct battery_manager *bm;
	int rc = 0;

	bm = devm_kzalloc(&pdev->dev, sizeof(struct battery_manager),
			  GFP_KERNEL);
	if (!bm) {
		pr_bm(ERROR, "no memory\n");
		return -ENOMEM;
	}

	bm->dev = &pdev->dev;
	rc = bm_init(bm);
	if (rc < 0) {
		pr_bm(ERROR, "bm_init fail\n");
		return rc;
	}

	platform_set_drvdata(pdev, bm);

	rc = bm_ps_register_notifier(bm);
	if (rc < 0) {
		pr_bm(ERROR, "bm_power_register_notifier fail\n");
		goto error;
	}

	rc = bm_fb_register_notifier(bm);
	if (rc < 0) {
		pr_bm(ERROR, "bm_fb_register_notifier fail!\n");
		goto error;
	}

	pr_bm(VERBOSE, "Battery manager driver probe success!\n");
	return 0;

error:
	mutex_destroy(&bm->work_lock);
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int lge_battery_suspend(struct device *dev)
{
	struct battery_manager *bm = dev_get_drvdata(dev);

	if (!bm) {
		pr_bm(ERROR, "There is no battery manager\n");
		return -ENODEV;
	}
	cancel_delayed_work_sync(&bm->bm_watch);

	return 0;
}

static int lge_battery_resume(struct device *dev)
{
	struct battery_manager *bm = dev_get_drvdata(dev);

	if (!bm) {
		pr_bm(ERROR, "There is no battery manager\n");
		return -ENODEV;
	}
	schedule_delayed_work(&bm->bm_watch, 0);

	return 0;
}

static int lge_battery_remove(struct platform_device *pdev)
{
	struct battery_manager *bm = dev_get_drvdata(&pdev->dev);

	mutex_destroy(&bm->work_lock);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct dev_pm_ops lge_battery_pm_ops = {
	.suspend	= lge_battery_suspend,
	.resume		= lge_battery_resume,
};

static struct platform_device lge_battery_pdev = {
	.name = BATT_DRV_NAME,
	.id = -1,
};

static struct platform_driver lge_battery_driver = {
	.probe = lge_battery_probe,
	.remove = lge_battery_remove,
	.driver = {
		.name = BATT_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &lge_battery_pm_ops,
	},
};

static int __init lge_battery_init(void)
{
	int ret;

	ret = platform_device_register(&lge_battery_pdev);
	if (ret < 0) {
		pr_bm(ERROR, "device register fail\n");
		return ret;
	}

	ret = platform_driver_register(&lge_battery_driver);
	if (ret < 0) {
		pr_bm(ERROR, "driver register fail\n");
		platform_device_unregister(&lge_battery_pdev);
		return ret;
	}
	return 0;
}

static void __exit lge_battery_exit(void)
{
	platform_device_unregister(&lge_battery_pdev);
	platform_driver_unregister(&lge_battery_driver);
}

static int set_demo_mode(const char *val, const struct kernel_param *kp)
{
	struct power_supply *batt_psy;
	int rc = 0;
	int old_val = demo_mode;

	rc = param_set_int(val, kp);
	if (rc) {
		pr_bm(ERROR, "Unable to set demo mode = %d\n", rc);
		return rc;
	}

	if (demo_mode == old_val)
		return 0;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_bm(ERROR, "Couldn't get batt_psy\n");
		return -ENODEV;
	}

	power_supply_changed(batt_psy);
	return 0;
}

static struct kernel_param_ops demo_mode_ops = {
	.set = set_demo_mode,
	.get = param_get_int,
};

module_param_cb(demo_mode, &demo_mode_ops, &demo_mode, 0644);
MODULE_PARM_DESC(demo_mode, "VZW Demo mode <on|off>");

module_init(lge_battery_init);
module_exit(lge_battery_exit);
MODULE_LICENSE("GPL");

/*
 * Copyright (c) 2010 Google, Inc.
 *
 * Author: Dima Zavin <dima@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/pm8058.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>


struct pm8058_charger {
	struct device				*pmic_dev;
	int					chgval_irq;
	int					fastchg_irq;

	struct power_supply			ac_supply;
	struct power_supply			usb_supply;

	struct pm8058_charger_platform_data	*pdata;

	spinlock_t				lock;
	bool					can_charge;
	bool					is_ac;
	bool					is_online;
	bool					vbus_present;
	int					charge_type;
	u32					max_current;
};

static struct pm8058_charger *the_pm8058_charger;

/* TODO: the usb core driver should provide the maximum current draw value to us
 * for charging */

void pm8058_notify_charger_connected(int status)
{
	struct pm8058_charger *charger = the_pm8058_charger;
	u32 max_current = 0;
	bool is_ac;
	bool is_online;
	bool change = false;
	unsigned long flags;

	if (!charger)
		return;

	printk("### %s(%d) ###\n", __func__, status);
	if (status && !charger->vbus_present)
		pr_warning("%s: cable status mismatch %d %d\n", __func__,
			   status, charger->vbus_present);

	switch (status) {
	case 1:
		/* usb (pc) charging */
		max_current = 500;
		is_ac = false;
		is_online = true;
		break;
	case 2:
		/* wall charger */
		max_current = 1500;
		is_ac = true;
		is_online = true;
		break;
	case 0:
	default:
		/* disable charging */
		max_current = 0;
		is_ac = false;
		is_online = false;
		break;
	}
	spin_lock_irqsave(&charger->lock, flags);
	if (max_current != charger->max_current ||
	    is_ac != charger->is_ac || is_online != charger->is_online) {
		charger->max_current = max_current;
		charger->is_ac = is_ac;
		charger->is_online = is_online;
		change = true;
	}
	spin_unlock_irqrestore(&charger->lock, flags);
	/* for now, charge control is done on the modem side, so we have to
	 * delegate to the board file. Eventually, all charge control will
	 * be done in this driver */
	if (change && charger->pdata->charge)
		charger->pdata->charge(max_current, is_ac);

	power_supply_changed(&charger->ac_supply);
	power_supply_changed(&charger->usb_supply);
}
EXPORT_SYMBOL_GPL(pm8058_notify_charger_connected);

static void check_chgval(struct pm8058_charger *charger)
{
	int ret;
	unsigned long flags;

	ret = pm8058_irq_get_status(charger->pmic_dev, PM8058_CHGVAL_IRQ);
	if (ret >= 0) {
		spin_lock_irqsave(&charger->lock, flags);
		charger->vbus_present = !!ret;
		spin_unlock_irqrestore(&charger->lock, flags);
		charger->pdata->vbus_present(ret);
	} else {
		pr_err("%s: can't read status!! ignoring event?!\n", __func__);
	}
}

static irqreturn_t chgval_irq_handler(int irq, void *dev_id)
{
	struct pm8058_charger *charger = dev_id;

	check_chgval(charger);
	return IRQ_HANDLED;
}

/* should only get this irq when we are plugged in */
static irqreturn_t fastchg_irq_handler(int irq, void *dev_id)
{
	struct pm8058_charger *charger = dev_id;
	int ret;
	bool fast_charging;
	unsigned long flags;

	ret = pm8058_irq_get_status(charger->pmic_dev, PM8058_FASTCHG_IRQ);
	if (ret < 0)
		return IRQ_HANDLED;
	fast_charging = !!ret;

	spin_lock_irqsave(&charger->lock, flags);
	if (fast_charging) {
		if (!charger->vbus_present) {
			pr_err("%s: charging without vbus?!\n", __func__);
			goto done;
		}
		charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
	} else {
		/* charging is either stopped (done/overtemp/etc.), or we
		 * are trickle charging. */
		/* TODO: detect trickle charging mode */
		if (charger->is_online)
			charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		else
			charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

done:
	spin_unlock_irqrestore(&charger->lock, flags);

	power_supply_changed(&charger->ac_supply);
	power_supply_changed(&charger->usb_supply);
	return IRQ_HANDLED;
}

static int power_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct pm8058_charger *charger;

	if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		charger = container_of(psy, struct pm8058_charger, ac_supply);
	else
		charger = container_of(psy, struct pm8058_charger, usb_supply);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = charger->is_online && charger->is_ac;
		else
			val->intval = charger->is_online && !charger->is_ac;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		/* for now, fake fast charge all the time if we're on */
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = charger->is_ac ? charger->charge_type :
				POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		else
			val->intval = charger->is_online && !charger->is_ac ?
				charger->charge_type :
				POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static int __init pm8058_charger_probe(struct platform_device *pdev)
{
	struct pm8058_charger_platform_data *pdata = pdev->dev.platform_data;
	struct pm8058_charger *charger;
	int chgval_irq;
	int fastchg_irq;
	int ret;

	chgval_irq = platform_get_irq_byname(pdev, "chgval_irq");
	fastchg_irq = platform_get_irq_byname(pdev, "fastchg_irq");

	if (!pdata || chgval_irq < 0 || fastchg_irq < 0) {
		pr_err("%s: missing platform data/resources\n", __func__);
		return -EINVAL;
	}

	charger = kzalloc(sizeof(struct pm8058_charger), GFP_KERNEL);
	if (!charger) {
		pr_err("%s: can't alloc mem for charger struct\n", __func__);
		return -ENOMEM;
	}

	charger->pmic_dev = pdev->dev.parent;
	charger->pdata = pdata;
	platform_set_drvdata(pdev, charger);
	spin_lock_init(&charger->lock);

	the_pm8058_charger = charger;

	charger->ac_supply.name = "ac";
	charger->ac_supply.type = POWER_SUPPLY_TYPE_MAINS;
	charger->ac_supply.supplied_to = pdata->supplied_to;
	charger->ac_supply.num_supplicants = pdata->num_supplicants;
	charger->ac_supply.properties = power_properties;
	charger->ac_supply.num_properties = ARRAY_SIZE(power_properties);
	charger->ac_supply.get_property = power_get_property;

	charger->usb_supply.name = "usb";
	charger->usb_supply.type = POWER_SUPPLY_TYPE_USB;
	charger->usb_supply.supplied_to = pdata->supplied_to;
	charger->usb_supply.num_supplicants = pdata->num_supplicants;
	charger->usb_supply.properties = power_properties;
	charger->usb_supply.num_properties = ARRAY_SIZE(power_properties);
	charger->usb_supply.get_property = power_get_property;

	ret = power_supply_register(&pdev->dev, &charger->ac_supply);
	if (ret)
		goto err_reg_ac_supply;
	ret = power_supply_register(&pdev->dev, &charger->usb_supply);
	if (ret)
		goto err_reg_usb_supply;

	ret = request_threaded_irq(chgval_irq, NULL, chgval_irq_handler,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "pm8058-charger-valid", charger);
	if (ret) {
		pr_err("%s: can't request chgval_irq\n", __func__);
		goto err_req_chgval_irq;
	}
	charger->chgval_irq = chgval_irq;

	ret = request_threaded_irq(fastchg_irq, NULL, fastchg_irq_handler,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "pm8058-charger-fastchg", charger);
	if (ret) {
		pr_err("%s: can't request stuck\n", __func__);
		goto err_req_fastchg_irq;
	}
	charger->fastchg_irq = fastchg_irq;
	enable_irq_wake(charger->chgval_irq);

	pr_info("%s: driver initialized\n", __func__);
	check_chgval(charger);

	return 0;

err_req_fastchg_irq:
	free_irq(chgval_irq, charger);
err_req_chgval_irq:
	power_supply_unregister(&charger->usb_supply);
err_reg_usb_supply:
	power_supply_unregister(&charger->ac_supply);
err_reg_ac_supply:
	platform_set_drvdata(pdev, NULL);
	the_pm8058_charger = NULL;
	kfree(charger);
	return ret;
}

static struct platform_driver pm8058_charger_driver = {
	.probe	= pm8058_charger_probe,
	.driver	= {
		.name	= "pm8058-charger",
		.owner	= THIS_MODULE,
	},
};

static int __init pm8058_charger_init(void)
{
	return platform_driver_register(&pm8058_charger_driver);
}

module_init(pm8058_charger_init);
MODULE_DESCRIPTION("PM8058 Charger Driver");
MODULE_AUTHOR("Dima Zavin <dima@android.com>");
MODULE_LICENSE("GPL");


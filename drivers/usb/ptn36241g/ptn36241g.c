/*
 * Driver for the NXP,PTN36241G USB3.0 Redriver
 *
 * Copyright (C) 2017 HTC Corporation.
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

#include <linux/of.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/init.h>

static char boot_mode[9] = {0};

/* must align with smblib's integer representation of cc orientation. */
enum cc_orientation {
	CC_ORIENTATION_NONE = 0,
	CC_ORIENTATION_CC1 = 1,
	CC_ORIENTATION_CC2 = 2,
};

struct ptn36241g {
	struct device		*dev;

	struct pinctrl		*nxp_pinctrl;
	struct pinctrl_state    *nxp_vdd_active;
	struct pinctrl_state    *nxp_vdd_sleep;
	struct pinctrl_state	*nxp_cc1_active_state;
	struct pinctrl_state	*nxp_cc1_sleep_state;
	struct pinctrl_state	*nxp_cc2_active_state;
	struct pinctrl_state	*nxp_cc2_sleep_state;

	struct work_struct	psy_changed_handler_work;

	struct power_supply	*usb_psy;
	struct notifier_block	psy_nb;

	bool ptn_active;

	struct mutex		lock; /* lock for chip struct */
};

static const char * const get_orientation_string(enum cc_orientation cc)
{
	switch (cc) {
	case CC_ORIENTATION_NONE:
		return "none";
	case CC_ORIENTATION_CC1:
		return "cc1";
	case CC_ORIENTATION_CC2:
		return "cc2";
	default:
		return "invalid";
	}
}

static void psy_changed_handler(struct work_struct *work)
{
	struct ptn36241g *chip = container_of(work, struct ptn36241g,
					      psy_changed_handler_work);
	int ret;
	union power_supply_propval val;
	enum cc_orientation cc_orientation;
	bool ptn_enable = false;

	ret = power_supply_get_property(chip->usb_psy,
					POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
					&val);
	if (ret < 0) {
		dev_err(chip->dev,
			"%s - unable to get power_supply cc orientation, ret=%d\n",
			__func__,
			ret);
		return;
	}

	cc_orientation = val.intval;
	switch (cc_orientation) {
	case CC_ORIENTATION_NONE:
		ptn_enable = false;
		break;
	case CC_ORIENTATION_CC1:
	case CC_ORIENTATION_CC2:
		ptn_enable = true;
		break;
	default:
		dev_err(chip->dev,
			"%s - invalid cc_orientation: %d\n", __func__,
			cc_orientation);
		return;
	}

	mutex_lock(&chip->lock);

	if (ptn_enable == chip->ptn_active)
		goto done;
	else
		chip->ptn_active = ptn_enable;

	if (chip->ptn_active) {
		/* Power on redriver ic */
		ret = pinctrl_select_state(chip->nxp_pinctrl,
					   chip->nxp_vdd_active);
		if (ret) {
			dev_err(chip->dev,
				"%s - failed to power on ptn36241g, ret=%d\n",
				__func__,
				ret);
			goto done;
		}

		switch (cc_orientation) {
		case CC_ORIENTATION_CC1:
			ret = pinctrl_select_state(chip->nxp_pinctrl,
						   chip->nxp_cc1_active_state);
			if (ret < 0)
				dev_err(chip->dev,
					"%s - failed to config cc1 active, ret=%d\n",
					__func__,
					ret);
			break;
		case CC_ORIENTATION_CC2:
			ret = pinctrl_select_state(chip->nxp_pinctrl,
						   chip->nxp_cc2_active_state);
			if (ret < 0)
				dev_err(chip->dev,
					"%s - failed to config cc2 active, ret=%d\n",
					__func__,
					ret);
			break;
		default:
			dev_err(chip->dev,
				"%s - invalid cc_orientation to set active state\n",
				__func__);
			ret = -EINVAL;
			break;
		}

		if (ret < 0)
			goto done;
	} else {
		ret = pinctrl_select_state(chip->nxp_pinctrl,
					   chip->nxp_cc1_sleep_state);
		if (ret < 0) {
			dev_err(chip->dev,
				"%s - failed to config cc1 sleep, ret=%d\n",
				__func__, ret);
			goto done;
		}

		ret = pinctrl_select_state(chip->nxp_pinctrl,
					   chip->nxp_cc2_sleep_state);
		if (ret < 0) {
			dev_err(chip->dev,
				"%s - failed to config cc2 sleep, ret=%d\n",
				__func__, ret);
			goto done;
		}

		/* Power off redriver ic */
		ret = pinctrl_select_state(chip->nxp_pinctrl,
					   chip->nxp_vdd_sleep);
		if (ret < 0) {
			dev_err(chip->dev,
				"%s - failed to power off ptn36241g, ret=%d\n",
				__func__, ret);
			goto done;
		}
	}

	dev_info(chip->dev, "%s - state := %s, orientation := %s\n",
		 __func__, chip->ptn_active ? "active" : "sleep",
		 get_orientation_string(cc_orientation));
done:
	mutex_unlock(&chip->lock);
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct ptn36241g *chip;

	chip = container_of(nb, struct ptn36241g, psy_nb);

	if (ptr != chip->usb_psy || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	schedule_work(&chip->psy_changed_handler_work);

	return 0;
}

static int ptn36241g_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ptn36241g *chip;
	struct pinctrl *pinctrl;
	struct pinctrl_state *nxp_vdd_active;
	struct pinctrl_state *nxp_vdd_sleep;
	struct pinctrl_state *nxp_cc1_active_state;
	struct pinctrl_state *nxp_cc1_sleep_state;
	struct pinctrl_state *nxp_cc2_active_state;
	struct pinctrl_state *nxp_cc2_sleep_state;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(&pdev->dev, "failed to get nxp pinctrl, ret=%d\n", ret);
		return ret;
	}

	nxp_vdd_active = pinctrl_lookup_state(pinctrl, "nxp_vdd_active");
	if (IS_ERR(nxp_vdd_active)) {
		ret = PTR_ERR(nxp_vdd_active);
		dev_err(&pdev->dev,
			"failed to get nxp_vdd_active, ret=%d\n", ret);
		return ret;
	}

	nxp_vdd_sleep = pinctrl_lookup_state(pinctrl, "nxp_vdd_sleep");
	if (IS_ERR(nxp_vdd_sleep)) {
		ret = PTR_ERR(nxp_vdd_sleep);
		dev_err(&pdev->dev,
			"failed to get nxp_vdd_sleep, ret=%d\n", ret);
		return ret;
	}

	nxp_cc1_active_state = pinctrl_lookup_state(pinctrl,
						    "nxp_cc1_active-HH");
	if (IS_ERR(nxp_cc1_active_state)) {
		ret = PTR_ERR(nxp_cc1_active_state);
		dev_err(&pdev->dev,
			"failed to get nxp_cc1_active_state, ret=%d\n", ret);
		return ret;
	}

	nxp_cc1_sleep_state = pinctrl_lookup_state(pinctrl,
						   "nxp_cc1_sleep-LL");
	if (IS_ERR(nxp_cc1_sleep_state)) {
		ret = PTR_ERR(nxp_cc1_sleep_state);
		dev_err(&pdev->dev,
			"failed to get nxp_cc1_sleep_state, ret=%d\n", ret);
		return ret;
	}

	nxp_cc2_active_state = pinctrl_lookup_state(pinctrl,
						    "nxp_cc2_active-HH");
	if (IS_ERR(nxp_cc2_active_state)) {
		ret = PTR_ERR(nxp_cc2_active_state);
		dev_err(&pdev->dev,
			"failed to get nxp_cc2_active_state, ret=%d\n", ret);
		return ret;
	}

	nxp_cc2_sleep_state = pinctrl_lookup_state(pinctrl,
						   "nxp_cc2_sleep-LL");
	if (IS_ERR(nxp_cc2_sleep_state)) {
		ret = PTR_ERR(nxp_cc2_sleep_state);
		dev_err(&pdev->dev,
			"failed to get nxp_cc2_sleep_state, ret=%d\n", ret);
		return ret;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->nxp_pinctrl = pinctrl;
	chip->nxp_vdd_active = nxp_vdd_active;
	chip->nxp_vdd_sleep = nxp_vdd_sleep;
	chip->nxp_cc1_active_state = nxp_cc1_active_state;
	chip->nxp_cc1_sleep_state = nxp_cc1_sleep_state;
	chip->nxp_cc2_active_state = nxp_cc2_active_state;
	chip->nxp_cc2_sleep_state = nxp_cc2_sleep_state;
	chip->ptn_active = false;
	mutex_init(&chip->lock);

	INIT_WORK(&chip->psy_changed_handler_work, psy_changed_handler);

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		dev_err(&pdev->dev,
			"%s - cannot get usb power supply, deferring probe\n",
			__func__);
		return -EPROBE_DEFER;
	}

	/* Initial power off redriver ic */
	ret = pinctrl_select_state(pinctrl, nxp_vdd_sleep);
	if (ret) {
		dev_err(&pdev->dev,
			"%s - failed to power off ptn36241g, ret=%d\n",
			__func__,
			ret);
		return ret;
	}

	/* Initial configuration for cc1/cc2 pin state */
	ret = pinctrl_select_state(chip->nxp_pinctrl,
				   chip->nxp_cc1_sleep_state);
	if (ret < 0) {
		dev_err(chip->dev,
			"%s - failed to config cc1 sleep, ret=%d\n",
			__func__, ret);
		return ret;
	}
	ret = pinctrl_select_state(chip->nxp_pinctrl,
				   chip->nxp_cc2_sleep_state);
	if (ret < 0) {
		dev_err(chip->dev,
			"%s - failed to config cc2 sleep, ret=%d\n",
			__func__, ret);
		return ret;
	}

	/* Do not enable ptn36241g redriver for 'usbradio' boot mode.
	 * End probe here after initializing the chip to inactive
	 * state.
	 */
	if (!strncmp(boot_mode, "usbradio", 8)) {
		dev_info(&pdev->dev,
			 "Do not register power notifier in usbradio mode\n");
		return ret;
	}

	/* Initial Redriver state */
	psy_changed(&chip->psy_nb, PSY_EVENT_PROP_CHANGED, chip->usb_psy);

	chip->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_nb);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"%s - cannot register notifier, ret=%d\n",
			__func__, ret);
		return ret;
	}

	dev_info(&pdev->dev, "probing PTN36241G driver done\n");

	return ret;
}

static int ptn36241g_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev,
		 "%s - remove ptn36241g driver\n", __func__);
	return 0;
}

static void ptn36241g_shutdown(struct platform_device *pdev)
{
	dev_info(&pdev->dev,
		 "%s - shutdown ptn36241g\n", __func__);
}

static int __init get_boot_mode(char *str)
{
	strlcpy(boot_mode, str, sizeof(boot_mode));
	pr_info("androidboot.mode:[%s], str:[%s]\n", boot_mode, str);
	return 0;
} early_param("androidboot.mode", get_boot_mode);

static const struct of_device_id ptn36241g_match_table[] = {
	{.compatible = "nxp,ptn36241g"},
	{},
};

static struct platform_driver ptn36241g_driver = {
	.driver = {
		.name		= "nxp,ptn36241g",
		.owner		= THIS_MODULE,
		.of_match_table = ptn36241g_match_table,
	},
	.probe		= ptn36241g_probe,
	.remove		= ptn36241g_remove,
	.shutdown	= ptn36241g_shutdown,
};
module_platform_driver(ptn36241g_driver);

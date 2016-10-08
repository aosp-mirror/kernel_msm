 /*
 * Copyright(c) 2013-2014, LGE Inc. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/suspend.h>

struct batt_tm_data {
	struct device *dev;
	struct qpnp_adc_tm_chip *adc_tm_dev;
	struct power_supply *batt_psy;
	struct power_supply *usb_psy;
	struct power_supply *wlc_psy;
	struct power_supply psy;
	struct qpnp_adc_tm_btm_param adc_param;
	struct tm_ctrl_data *warm_cfg;
	struct tm_ctrl_data *cool_cfg;
	struct tm_ctrl_data *warm_cfg_wlc;
	struct tm_ctrl_data *cool_cfg_wlc;
	struct wakeup_source ws_noti;
	bool tm_disabled_in_suspend;
	unsigned int warm_cfg_size;
	unsigned int cool_cfg_size;
	unsigned int warm_cfg_size_wlc;
	unsigned int cool_cfg_size_wlc;
	int batt_vreg_mv;
	int low_batt_vreg_mv;
	int current_ma;
	int low_current_ma;
	int wlc_online;
	int usb_online;
	bool is_wlc_param;
	bool wlc_config_use;
};

struct tm_ctrl_data {
	int thr;
	int next_cool_thr;
	int next_warm_thr;
	unsigned int action;
	unsigned int health;
};

enum {
	CHG_ENABLE,
	CHG_DECREASE,
	CHG_STOP,
	CHG_SHUTDOWN
};

static int power_supply_set_max_voltage(struct power_supply *psy, int limit)
{
	const union power_supply_propval ret = {limit,};

	if (psy->set_property)
		return psy->set_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX,
								&ret);

	return -ENXIO;
}

static int power_supply_set_chg_enable(struct power_supply *psy, int enable)
{
	const union power_supply_propval ret = {enable,};

	if (psy->set_property)
		return psy->set_property(psy,
			POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &ret);

	return -ENXIO;
}

static void batt_tm_notification(enum qpnp_tm_state state, void *ctx)
{
	struct batt_tm_data *batt_tm = ctx;
	int temp;
	int i;
	int tm_action;
	int batt_health;
	union power_supply_propval ret = {0,};
	struct tm_ctrl_data *warm;
	struct tm_ctrl_data *cool;
	unsigned int warm_size;
	unsigned int cool_size;
	int rc;

	__pm_stay_awake(&batt_tm->ws_noti);
	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invalid notification %d\n", state);
		__pm_relax(&batt_tm->ws_noti);
		return;
	}

	batt_tm->batt_psy->get_property(batt_tm->batt_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
	temp = ret.intval;

	pr_debug("temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "warm" : "cool");

	if (batt_tm->wlc_config_use && batt_tm->is_wlc_param) {
		warm = batt_tm->warm_cfg_wlc;
		cool = batt_tm->cool_cfg_wlc;
		warm_size = batt_tm->warm_cfg_size_wlc;
		cool_size = batt_tm->cool_cfg_size_wlc;
	} else {
		warm = batt_tm->warm_cfg;
		cool = batt_tm->cool_cfg;
		warm_size = batt_tm->warm_cfg_size;
		cool_size = batt_tm->cool_cfg_size;
	}

	if (state == ADC_TM_WARM_STATE) {
		i = warm_size - 1;
		while ((batt_tm->adc_param.high_temp !=
					warm[i].thr) && (i > 0))
			i--;

		batt_tm->adc_param.low_temp = warm[i].next_cool_thr;
		batt_tm->adc_param.high_temp = warm[i].next_warm_thr;
		tm_action = warm[i].action;
		batt_health = warm[i].health;
	} else {
		i = cool_size - 1;
		while ((batt_tm->adc_param.low_temp !=
					cool[i].thr) && (i > 0))
			i--;

		batt_tm->adc_param.low_temp = cool[i].next_cool_thr;
		batt_tm->adc_param.high_temp = cool[i].next_warm_thr;
		tm_action = cool[i].action;
		batt_health = cool[i].health;
	}

	power_supply_set_health_state(batt_tm->batt_psy, batt_health);

	switch (tm_action) {
	case CHG_ENABLE:
		pr_debug("Enable charging. vbatt_max = %d\n",
					batt_tm->batt_vreg_mv);
		power_supply_set_chg_enable(batt_tm->batt_psy, true);
		power_supply_set_current_limit(batt_tm->batt_psy,
					batt_tm->current_ma * 1000);
		power_supply_set_max_voltage(batt_tm->batt_psy,
					batt_tm->batt_vreg_mv * 1000);
		break;
	case CHG_DECREASE:
		pr_debug("Decrease current to %d, vbatt_max to %d\n",
						batt_tm->low_current_ma,
						batt_tm->low_batt_vreg_mv);
		power_supply_set_chg_enable(batt_tm->batt_psy, true);
		power_supply_set_current_limit(batt_tm->batt_psy,
					batt_tm->low_current_ma * 1000);
		power_supply_set_max_voltage(batt_tm->batt_psy,
					batt_tm->low_batt_vreg_mv * 1000);
		break;
	case CHG_STOP:
		pr_debug("Stop charging!\n");
		power_supply_set_chg_enable(batt_tm->batt_psy, false);
		break;
	case CHG_SHUTDOWN:
		pr_debug("Shutdown!\n");
		power_supply_changed(batt_tm->batt_psy);
		break;
	default:
		break;
	}

	pr_info("[%s] action : %d next low temp = %d next high temp = %d\n",
					batt_tm->is_wlc_param? "WLC" : "USB",
					tm_action,
					batt_tm->adc_param.low_temp,
					batt_tm->adc_param.high_temp);

	rc = qpnp_adc_tm_channel_measure(batt_tm->adc_tm_dev,
						&batt_tm->adc_param);
	if (rc)
		pr_err("request adc_tm error\n");

	__pm_relax(&batt_tm->ws_noti);
}

static int batt_tm_notification_init_legacy(struct batt_tm_data *batt_tm)
{
	int rc;

	batt_tm->adc_param.low_temp =
				batt_tm->warm_cfg[0].next_cool_thr;
	batt_tm->adc_param.high_temp =
				batt_tm->warm_cfg[0].next_warm_thr;
	batt_tm->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
	batt_tm->adc_param.timer_interval = ADC_MEAS1_INTERVAL_8S;
	batt_tm->adc_param.btm_ctx = batt_tm;
	batt_tm->adc_param.threshold_notification =
					batt_tm_notification;
	batt_tm->adc_param.channel = LR_MUX1_BATT_THERM;

	rc = qpnp_adc_tm_channel_measure(batt_tm->adc_tm_dev,
						&batt_tm->adc_param);
	if (rc)
		pr_err("request adc_tm error %d\n", rc);

	return rc;
}

static int batt_tm_notification_init(struct batt_tm_data *batt_tm)
{
	int rc = 0;

	if (batt_tm->wlc_online || batt_tm->usb_online) {
		/* Normal setting */
		power_supply_set_chg_enable(batt_tm->batt_psy, true);
		power_supply_set_current_limit(batt_tm->batt_psy,
					batt_tm->current_ma * 1000);
		power_supply_set_max_voltage(batt_tm->batt_psy,
					batt_tm->batt_vreg_mv * 1000);

		if (batt_tm->wlc_online) {
			batt_tm->adc_param.low_temp =
						batt_tm->warm_cfg_wlc[0].next_cool_thr;
			batt_tm->adc_param.high_temp =
						batt_tm->warm_cfg_wlc[0].next_warm_thr;
			batt_tm->is_wlc_param = true;
		} else if (batt_tm->usb_online) {
			batt_tm->adc_param.low_temp =
						batt_tm->warm_cfg[0].next_cool_thr;
			batt_tm->adc_param.high_temp =
						batt_tm->warm_cfg[0].next_warm_thr;
			batt_tm->is_wlc_param = false;
		}
		batt_tm->adc_param.threshold_notification =
					batt_tm_notification;
		pr_info("[%s] next low temp = %d next high temp = %d\n",
					batt_tm->is_wlc_param? "WLC" : "USB",
					batt_tm->adc_param.low_temp,
					batt_tm->adc_param.high_temp);
		rc = qpnp_adc_tm_channel_measure(batt_tm->adc_tm_dev,
							&batt_tm->adc_param);

		if (rc)
			pr_err("request adc_tm error %d\n", rc);
	}

	return rc;
}

static int batt_tm_parse_dt(struct device_node *np,
				struct batt_tm_data *batt_tm)
{
	int ret;

	batt_tm->adc_tm_dev = qpnp_get_adc_tm(batt_tm->dev, "batt-tm");
	if (IS_ERR(batt_tm->adc_tm_dev)) {
		ret = PTR_ERR(batt_tm->adc_tm_dev);
		pr_err("adc-tm not ready\n");
		goto out;
	}

	ret = of_property_read_u32(np, "tm,current-ma",
					&batt_tm->current_ma);
	if (ret) {
		pr_err("failed to get tm,current-ma\n");
		goto out;
	}

	ret = of_property_read_u32(np, "tm,batt-vreg-mv",
					&batt_tm->batt_vreg_mv);
	if (ret) {
		pr_err("failed to get tm,batt-vreg-mv\n");
		goto out;
	}

	ret = of_property_read_u32(np, "tm,low-vreg-mv",
				&batt_tm->low_batt_vreg_mv);
	if (ret) {
		pr_err("failed to get tm,low-vbatt-mv\n");
		goto out;
	}

	ret = of_property_read_u32(np, "tm,low-current-ma",
				&batt_tm->low_current_ma);
	if (ret) {
		pr_err("failed to get tm,low-current-ma\n");
		goto out;
	}

	if (!of_get_property(np, "tm,warm-cfg",
				&batt_tm->warm_cfg_size)) {
		pr_err("failed to get warm_cfg\n");
		ret = -EINVAL;
		goto out;
	}

	batt_tm->warm_cfg = devm_kzalloc(batt_tm->dev,
				batt_tm->warm_cfg_size, GFP_KERNEL);
	if (!batt_tm->warm_cfg) {
		pr_err("Unable to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = of_property_read_u32_array(np, "tm,warm-cfg",
					(u32 *)batt_tm->warm_cfg,
					batt_tm->warm_cfg_size / sizeof(u32));
	if (ret) {
		pr_err("failed to get tm,warm-cfg\n");
		goto out;
	}

	batt_tm->warm_cfg_size /= sizeof(struct tm_ctrl_data);

	if (!of_get_property(np, "tm,cool-cfg",
			&batt_tm->cool_cfg_size)) {
		pr_err("failed to get cool_cfg\n");
		ret = -EINVAL;
		goto out;
	}

	batt_tm->cool_cfg = devm_kzalloc(batt_tm->dev,
				batt_tm->cool_cfg_size, GFP_KERNEL);
	if (!batt_tm->cool_cfg) {
		pr_err("Unable to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = of_property_read_u32_array(np, "tm,cool-cfg",
					(u32 *)batt_tm->cool_cfg,
					batt_tm->cool_cfg_size / sizeof(u32));
	if (ret) {
		pr_err("failed to get tm,cool-cfg\n");
		goto out;
	}

	batt_tm->cool_cfg_size /= sizeof(struct tm_ctrl_data);

	batt_tm->wlc_config_use = of_property_read_bool(np, "tm,wlc-config-use");
	if (batt_tm->wlc_config_use) {
		if (!of_get_property(np, "tm,warm-cfg-wlc",
					&batt_tm->warm_cfg_size_wlc)) {
			pr_err("failed to get warm_cfg_wlc\n");
			ret = -EINVAL;
			goto out;
		}

		batt_tm->warm_cfg_wlc = devm_kzalloc(batt_tm->dev,
					batt_tm->warm_cfg_size_wlc, GFP_KERNEL);
		if (!batt_tm->warm_cfg_wlc) {
			pr_err("Unable to allocate memory\n");
			ret = -ENOMEM;
			goto out;
		}

		ret = of_property_read_u32_array(np, "tm,warm-cfg-wlc",
						(u32 *)batt_tm->warm_cfg_wlc,
						batt_tm->warm_cfg_size_wlc / sizeof(u32));
		if (ret) {
			pr_err("failed to get tm,warm-cfg\n");
			goto out;
		}

		batt_tm->warm_cfg_size_wlc /= sizeof(struct tm_ctrl_data);

		if (!of_get_property(np, "tm,cool-cfg-wlc",
				&batt_tm->cool_cfg_size_wlc)) {
			pr_err("failed to get cool_cfg_wlc\n");
			ret = -EINVAL;
			goto out;
		}

		batt_tm->cool_cfg_wlc = devm_kzalloc(batt_tm->dev,
					batt_tm->cool_cfg_size_wlc, GFP_KERNEL);
		if (!batt_tm->cool_cfg_wlc) {
			pr_err("Unable to allocate memory\n");
			ret = -ENOMEM;
			goto out;
		}

		ret = of_property_read_u32_array(np, "tm,cool-cfg",
						(u32 *)batt_tm->cool_cfg_wlc,
						batt_tm->cool_cfg_size_wlc / sizeof(u32));
		if (ret) {
			pr_err("failed to get tm,cool-cfg\n");
			goto out;
		}

		batt_tm->cool_cfg_size_wlc /= sizeof(struct tm_ctrl_data);
	}
out:
	return ret;
}

static void batt_tm_external_power_changed(struct power_supply *psy)
{
	struct batt_tm_data *batt_tm =
			container_of(psy, struct batt_tm_data, psy);
	union power_supply_propval res = {0, };
	int online_changed = 0;

	batt_tm->wlc_psy->get_property(
			batt_tm->wlc_psy, POWER_SUPPLY_PROP_ONLINE, &res);
	if (batt_tm->wlc_online ^ res.intval) {
		batt_tm->wlc_online = res.intval;
		online_changed = 1;
	}

	/* in case of using both cable wlc and usb charger
	 * If wlc charger is fake_online state,
	 * usb plugged state will be changed. */
	if (!batt_tm->wlc_online) {
		batt_tm->usb_psy->get_property(
				batt_tm->usb_psy, POWER_SUPPLY_PROP_ONLINE, &res);
		if (batt_tm->usb_online ^ res.intval) {
			batt_tm->usb_online = res.intval;
			online_changed = 1;
		}
	}

	if (online_changed) {
		pr_debug("wlc=%d, usb=%d\n", batt_tm->wlc_online,
				batt_tm->usb_online);
		batt_tm_notification_init(batt_tm);
	}
}

static int batt_tm_get_psy_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	return -EINVAL;
}

static char *batt_tm_supplied_from[] = {
	"usb",
	"wireless",
};

static int batt_tm_probe(struct platform_device *pdev)
{
	struct batt_tm_data *batt_tm;
	struct device_node *dev_node = pdev->dev.of_node;
	int ret = 0;

	batt_tm = devm_kzalloc(&pdev->dev,
			sizeof(struct batt_tm_data), GFP_KERNEL);
	if (!batt_tm) {
		pr_err("falied to alloc memory\n");
		return -ENOMEM;
	}
	batt_tm->dev = &pdev->dev;
	platform_set_drvdata(pdev, batt_tm);

	if (dev_node) {
		ret = batt_tm_parse_dt(dev_node, batt_tm);
		if (ret) {
			pr_err("failed to parse dt\n");
			goto out;
		}
	} else {
		pr_err("not supported for non OF\n");
		ret = -ENODEV;
		goto out;
	}

	batt_tm->batt_psy = power_supply_get_by_name("battery");
	if (!batt_tm->batt_psy) {
		pr_err("battery supply not found\n");
		ret =  -EPROBE_DEFER;
		goto out;
	}

	wakeup_source_init(&batt_tm->ws_noti, "batt_tm_noti");

	if (!batt_tm->wlc_config_use) {
		ret = batt_tm_notification_init_legacy(batt_tm);
		if (ret) {
			pr_err("failed to init adc tm\n");
			goto out;
		}
	} else {
		batt_tm->usb_psy = power_supply_get_by_name("usb");
		if (!batt_tm->usb_psy) {
			pr_err("usb supply not found\n");
			ret =  -EPROBE_DEFER;
			goto out;
		}

		batt_tm->wlc_psy = power_supply_get_by_name("wireless");
		if (!batt_tm->wlc_psy) {
			pr_err("wireless supply not found\n");
			ret =  -EPROBE_DEFER;
			goto out;
		}

		/* pre-initialize adc_tm */
		batt_tm->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		batt_tm->adc_param.timer_interval = ADC_MEAS1_INTERVAL_8S;
		batt_tm->adc_param.btm_ctx = batt_tm;
		batt_tm->adc_param.channel = LR_MUX1_BATT_THERM;

		/* register psy */
		batt_tm->psy.name = "batt_tm";
		batt_tm->psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
		batt_tm->psy.get_property = batt_tm_get_psy_property;
		batt_tm->psy.external_power_changed =
			batt_tm_external_power_changed;
		batt_tm->psy.of_node = pdev->dev.of_node;
		batt_tm->psy.supplied_from = batt_tm_supplied_from;
		batt_tm->psy.num_supplies = ARRAY_SIZE(batt_tm_supplied_from);
		ret = power_supply_register(batt_tm->dev, &batt_tm->psy);
		if (ret < 0) {
			pr_err("fail to register psy ret=%d\n", ret);
			goto out;
		}
		batt_tm_external_power_changed(&batt_tm->psy);
	}

out:
	return ret;
}

static int batt_tm_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int batt_tm_pm_prepare(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct batt_tm_data *batt_tm = platform_get_drvdata(pdev);

	if (!power_supply_is_system_supplied()) {
		qpnp_adc_tm_disable_chan_meas(batt_tm->adc_tm_dev,
					&batt_tm->adc_param);
		batt_tm->tm_disabled_in_suspend = true;
	}
	return 0;
}

static void batt_tm_pm_complete(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct batt_tm_data *batt_tm = platform_get_drvdata(pdev);

	if (batt_tm->tm_disabled_in_suspend) {
		qpnp_adc_tm_channel_measure(batt_tm->adc_tm_dev,
					&batt_tm->adc_param);
		batt_tm->tm_disabled_in_suspend = false;
	}
}

static const struct dev_pm_ops batt_tm_pm_ops = {
	.prepare = batt_tm_pm_prepare,
	.complete = batt_tm_pm_complete,
};

static struct of_device_id batt_tm_match[] = {
	{.compatible = "battery_tm", },
	{}
};

static struct platform_driver batt_tm_driver = {
	.probe = batt_tm_probe,
	.remove = batt_tm_remove,
	.driver = {
		.name = "batt_tm",
		.owner = THIS_MODULE,
		.of_match_table = batt_tm_match,
		.pm = &batt_tm_pm_ops,
	},
};

static int __init batt_tm_init(void)
{
	return platform_driver_register(&batt_tm_driver);
}
late_initcall(batt_tm_init);

static void __exit batt_tm_exit(void)
{
	platform_driver_unregister(&batt_tm_driver);
}
module_exit(batt_tm_exit);

MODULE_DESCRIPTION("Battery Temp Monitor Driver");
MODULE_AUTHOR("ChoongRyeol Lee <choongryeol.lee@lge.com>");
MODULE_LICENSE("GPL");

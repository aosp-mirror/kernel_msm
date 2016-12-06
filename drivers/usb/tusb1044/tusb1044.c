/*
 * Driver for the TUSB1044 USB3.0 Redriver
 *
 * Copyright (C) 2016 HTC Corporation.
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

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include "tusb1044.h"

enum cc_state {
	CC_STATE_OPEN = 0,
	CC_STATE_USB3,
	CC_STATE_DP,
};

/* must align with smblib's integer representation of cc orientation. */
enum cc_orientation {
	CC_ORIENTATION_NONE = 0,
	CC_ORIENTATION_CC1 = 1,
	CC_ORIENTATION_CC2 = 2,
};

static const char * const cc_state_name[] = {
	[CC_STATE_USB3]		= "USB3",
	[CC_STATE_DP]		= "Display Port",
	[CC_STATE_OPEN]		= "Open",
};

static const char * const cc_orientation_name[] = {
	[CC_ORIENTATION_NONE]	= "None",
	[CC_ORIENTATION_CC1]	= "CC1",
	[CC_ORIENTATION_CC2]	= "CC2",
};

struct tusb1044_chip {
	struct device		*device;
	struct i2c_client	*i2c_client;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pinctrl_active_state;
	struct pinctrl_state	*pinctrl_sleep_state;

	struct power_supply	*usb_psy;
	struct notifier_block	psy_nb;

	enum cc_state		cc_state;
	enum cc_orientation	cc_orientation;

	struct workqueue_struct *wq;
};

static int tusb1044_1v8_switch(struct tusb1044_chip *chip, bool enabled)
{
	int ret = 0;
	struct pinctrl_state *pinctrl_state = enabled ?
					      chip->pinctrl_active_state :
					      chip->pinctrl_sleep_state;

	ret = pinctrl_select_state(chip->pinctrl, pinctrl_state);
	if (ret < 0) {
		dev_err(chip->device,
			"TUSB1044 %s - Error: pinctrl enable %s state failed, ret=%d\n",
			__func__, enabled ? "active" : "sleep", ret);
		return ret;
	}

	dev_info(chip->device, "TUSB1044 %s - pinctrl enable %s state done\n",
		 __func__, enabled ? "active" : "sleep");
	return ret;
}

static int tusb1044_aux_snoop(struct tusb1044_chip *chip, bool enabled)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_AUX,
					enabled ?
					ENABLE_AUX_SNOOP :
					DISABLE_AUX_SNOOP);
	if (ret < 0) {
		dev_err(chip->device,
			"TUSB1044 %s - cannot %s aux snoop, ret=%d\n",
			__func__, enabled ? "enable" : "disable", ret);
		return ret;
	}

	dev_info(chip->device, "TUSB1044 %s - %s aux snoop\n",
		 __func__, enabled ? "enable" : "disable");
	return ret;
}

static int tusb1044_usb3_update_state(struct tusb1044_chip *chip,
				      enum cc_state cc_state,
				      enum cc_orientation cc_orientation)
{
	int ret = 0;
	bool switch_1v8_enabled;
	bool aux_snoop_enabled;
	u8 config_ctrl_value = 0;

	switch (cc_state) {
	case CC_STATE_USB3:
		switch_1v8_enabled = true;
		aux_snoop_enabled = true;
		if (cc_orientation == CC_ORIENTATION_CC1)
			config_ctrl_value = USB3_ON_CC1;
		else if (cc_orientation == CC_ORIENTATION_CC2)
			config_ctrl_value = USB3_ON_CC2;
		break;
	case CC_STATE_DP:
		switch_1v8_enabled = true;
		aux_snoop_enabled = false;
		if (cc_orientation == CC_ORIENTATION_CC1)
			config_ctrl_value = DP_ON_CC1;
		else if (cc_orientation == CC_ORIENTATION_CC2)
			config_ctrl_value = DP_ON_CC2;
		break;
	case CC_STATE_OPEN:
	default:
		switch_1v8_enabled = false;
		aux_snoop_enabled = true;
		config_ctrl_value = DISABLE_TX_RX;
		break;
	}

	ret = tusb1044_1v8_switch(chip, switch_1v8_enabled);
	if (ret < 0)
		return ret;

	ret = tusb1044_aux_snoop(chip, aux_snoop_enabled);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_CONFIG_CTRL,
					config_ctrl_value);
	if (ret < 0)
		return ret;

	dev_info(chip->device,
		 "TUSB1044 %s - state := %s, orientation := %s, switch := %s, aux_snoop := %s, config_ctrl := 0x%x\n",
		 __func__, cc_state_name[cc_state],
		 cc_orientation_name[cc_orientation],
		 switch_1v8_enabled ? "enabled" : "disabled",
		 aux_snoop_enabled ? "enabled" : "disabled",
		 config_ctrl_value);
	return ret;
}

struct psy_changed_event {
	struct work_struct work;
	struct tusb1044_chip *chip;
};

static void psy_changed_handler(struct work_struct *work)
{
	struct psy_changed_event *event = container_of(work,
						      struct psy_changed_event,
						      work);
	struct tusb1044_chip *chip = event->chip;
	int ret = 0;
	union power_supply_propval val;
	enum cc_state cc_state;
	enum cc_orientation cc_orientation;

	ret = power_supply_get_property(chip->usb_psy,
					POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
					&val);
	if (ret < 0) {
		dev_err(chip->device,
			"TUSB1044 %s - unable to get power_supply cc orientation, ret=%d\n",
			__func__,
			ret);
		goto done;
	}

	cc_orientation = val.intval;
	switch (cc_orientation) {
	case CC_ORIENTATION_NONE:
		cc_state = CC_STATE_OPEN;
		break;
	case CC_ORIENTATION_CC1:
		cc_state = CC_STATE_USB3;
		break;
	case CC_ORIENTATION_CC2:
		cc_state = CC_STATE_USB3;
		break;
	default:
		dev_err(chip->device,
			"TUSB1044 %s - invalid cc_orientation: %d\n",
			__func__,
			cc_orientation);
		goto done;
	}

	if (cc_orientation == chip->cc_orientation &&
	    cc_state == chip->cc_state)
		goto done;

	ret = tusb1044_usb3_update_state(chip, cc_state, cc_orientation);
	if (ret < 0)
		goto done;

	chip->cc_orientation = cc_orientation;
	chip->cc_state = cc_state;

done:
	kfree(event);
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct tusb1044_chip *chip;
	struct psy_changed_event *event;

	chip = container_of(nb, struct tusb1044_chip, psy_nb);
	if (ptr != chip->usb_psy || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	event = kzalloc(sizeof(*event), GFP_ATOMIC);
	if (!event)
		return -ENOMEM;

	INIT_WORK(&event->work, psy_changed_handler);
	event->chip = chip;
	queue_work(chip->wq, &event->work);

	return 0;
}

static int tusb1044_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	int ret = 0;
	struct tusb1044_chip *chip;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_sleep_state;
	struct pinctrl_state *pinctrl_active_state;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
			"TUSB1044 %s - i2c_check_functionality error\n",
			__func__);
		return -EIO;
	}

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: pinctrl not ready\n",
			__func__);
		return PTR_ERR(pinctrl);
	}

	pinctrl_active_state = pinctrl_lookup_state(pinctrl, "pin_active");
	if (IS_ERR(pinctrl_active_state)) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: no pin_active pinctrl state, ret=%ld\n",
			__func__, PTR_ERR(pinctrl_active_state));
		return PTR_ERR(pinctrl_active_state);
	}

	pinctrl_sleep_state = pinctrl_lookup_state(pinctrl, "pin_sleep");
	if (IS_ERR(pinctrl_sleep_state)) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: no pin_sleep pinctrl state, ret=%ld\n",
			__func__, PTR_ERR(pinctrl_sleep_state));
		return PTR_ERR(pinctrl_sleep_state);
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->device = &client->dev;
	chip->i2c_client = client;
	i2c_set_clientdata(client, chip);
	chip->pinctrl = pinctrl;
	chip->pinctrl_active_state = pinctrl_active_state;
	chip->pinctrl_sleep_state = pinctrl_sleep_state;

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: cannot get usb power supply, deferring probe",
			__func__);
		return -EPROBE_DEFER;
	}

	chip->wq = create_singlethread_workqueue(dev_name(&client->dev));
	if (!chip->wq) {
		ret = -ENOMEM;
		goto put_psy;
	}

	ret = tusb1044_usb3_update_state(chip, CC_STATE_OPEN,
					 CC_ORIENTATION_NONE);
	if (ret < 0) {
		dev_err(&client->dev,
			"TUSB1044 %s - Error: REG_CONFIG_CTRL := DISABLE_TX_RX failed, ret=%d\n",
			__func__, ret);
		goto del_wq;
	}
	chip->cc_orientation = CC_ORIENTATION_NONE;
	chip->cc_state = CC_STATE_OPEN;

	psy_changed(&chip->psy_nb, PSY_EVENT_PROP_CHANGED, chip->usb_psy);

	chip->psy_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_nb);
	if (ret < 0)
		goto del_wq;

	dev_info(&client->dev,
		 "TUSB1044 %s - probing TUSB1044 i2c driver done\n", __func__);
	return ret;

del_wq:
	destroy_workqueue(chip->wq);
put_psy:
	power_supply_put(chip->usb_psy);
	return ret;
}

static int tusb1044_i2c_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "TUSB1044 %s - remove TUSB1044 i2c driver\n",
		 __func__);
	return 0;
}

static const struct of_device_id tusb1044_i2c_match_table[] = {
	{.compatible = "ti,tusb1044-i2c" },
	{},
};

static const struct i2c_device_id tusb1044_i2c_id[] = {
	{ "tusb1044-i2c", 0 },
	{ }
};

static struct i2c_driver tusb1044_i2c_driver = {
	.driver = {
		.name		= "tusb1044-i2c",
		.owner		= THIS_MODULE,
		.of_match_table = tusb1044_i2c_match_table,
	},
	.probe		= tusb1044_i2c_probe,
	.remove		= tusb1044_i2c_remove,
	.id_table	= tusb1044_i2c_id,
};
module_i2c_driver(tusb1044_i2c_driver);


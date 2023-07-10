/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 Google LLC */

#define pr_fmt(fmt) "smblite-shim:%s: " fmt, __func__

#include <linux/jiffies.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include "smblite-shim.h"

static struct power_supply_desc usb_psy_desc;

static int smblite_shim_usb_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct smblite_shim *shim = power_supply_get_drvdata(psy);
	struct smb_charger *chg = shim->chg;
	const struct power_supply_desc *real_usb_desc = chg->usb_psy->desc;

	return real_usb_desc->get_property(chg->usb_psy, psp, val);
}

static int smblite_shim_usb_set_prop(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct smblite_shim *shim = power_supply_get_drvdata(psy);
	struct smb_charger *chg = shim->chg;
	const struct power_supply_desc *real_usb_desc = chg->usb_psy->desc;

	return real_usb_desc->set_property(chg->usb_psy, psp, val);
}

static int smblite_shim_usb_prop_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	struct smblite_shim *shim = power_supply_get_drvdata(psy);
	struct smb_charger *chg = shim->chg;
	const struct power_supply_desc *real_usb_desc = chg->usb_psy->desc;

	return real_usb_desc->property_is_writeable(chg->usb_psy, psp);
}

static void smblite_shim_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

struct smblite_shim *smblite_shim_init(struct smb_charger *chg)
{
	struct smblite_shim *shim;

	shim = devm_kzalloc(chg->dev, sizeof(*shim), GFP_KERNEL);

	if (!shim)
		return NULL;

	shim->chg = chg;
	return shim;
}

int smblite_shim_on_usb_psy_created(struct smblite_shim *shim,
				struct power_supply_desc *existing_usb_desc)
{
	struct power_supply_config usb_cfg = {};

	memcpy(&usb_psy_desc, existing_usb_desc, sizeof(usb_psy_desc));

	usb_psy_desc.name = "usb";
	usb_psy_desc.get_property = smblite_shim_usb_get_prop;
	usb_psy_desc.set_property = smblite_shim_usb_set_prop;
	usb_psy_desc.property_is_writeable = smblite_shim_usb_prop_is_writeable;
	usb_psy_desc.external_power_changed = smblite_shim_external_power_changed;

	usb_cfg.drv_data = shim;
	usb_cfg.of_node = shim->chg->dev->of_node;
	shim->psy = devm_power_supply_register(shim->chg->dev,
						&usb_psy_desc, &usb_cfg);
	if (IS_ERR(shim->psy)) {
		pr_err("Couldn't register smblite shim USB power supply\n");
		return PTR_ERR(shim->psy);
	}

	return 0;
}

void smblite_shim_on_usb_type_updated(struct smblite_shim *shim,
				enum power_supply_type type)
{
	usb_psy_desc.type = type;
}

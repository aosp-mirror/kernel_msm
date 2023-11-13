/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 Google LLC */

#ifndef __SMBLITE_SHIM_H__
#define __SMBLITE_SHIM_H__

#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include "misc/gvotable.h"
#include "smblite-lib.h"

enum smblite_shim_plug_sts {
	SMBLITE_SHIM_UNPLUGGED,
	SMBLITE_SHIM_PLUGGED_IN
};

enum smblite_shim_boost_sts {
	SMBLITE_SHIM_BOOST_DIS,
	SMBLITE_SHIM_BOOST_EN
};

struct smblite_shim {
	struct mutex lock;
	struct blocking_notifier_head hvdcp_req_nh;
	struct blocking_notifier_head usbin_plugin_nh;
	struct blocking_notifier_head boost_nh;
	struct smb_charger *chg;
	struct power_supply *psy;
	struct gvotable_election *fake_psy_online_votable;
	struct gvotable_election *vmax_votable;
	unsigned int sdp_icl_req_ignored;
	unsigned int real_sdp_icl;
};

struct smblite_shim *smblite_shim_init(struct smb_charger *chg);

int smblite_shim_on_usb_psy_created(struct smblite_shim *shim,
				struct power_supply_desc *existing_usb_desc);

void smblite_shim_on_usb_type_updated(struct smblite_shim *shim,
				enum power_supply_type type);

int smblite_shim_update_sw_icl_max(struct smblite_shim *shim, int type);

void smblite_shim_notify_hvdcp_req(struct smblite_shim *shim);
void smblite_shim_notify_plugin(struct smblite_shim *shim,
				enum smblite_shim_plug_sts plugged);
void smblite_shim_notify_boost_sw(struct smblite_shim *shim,
				enum smblite_shim_boost_sts boost);

int smblite_shim_hvdcp_req_register_notifier(struct smblite_shim *shim,
					struct notifier_block *nb);
int smblite_shim_hvdcp_req_unregister_notifier(struct smblite_shim *shim,
					struct notifier_block *nb);
int smblite_shim_plugin_register_notifier(struct smblite_shim *shim,
					struct notifier_block *nb);
int smblite_shim_plugin_unregister_notifier(struct smblite_shim *shim,
					struct notifier_block *nb);
int smblite_shim_boost_register_notifier(struct smblite_shim *shim,
					struct notifier_block *nb);
int smblite_shim_boost_unregister_notifier(struct smblite_shim *shim,
					struct notifier_block *nb);
#endif

/* arch/arm/mach-msm/htc_battery.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>

static char *supply_list[] = {
	"battery",
};

static int vbus_present;
static int usb_status;

static int power_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
		val->intval = (vbus_present && (usb_status == 2));
	} else {
		val->intval = (vbus_present && (usb_status == 1));
	}
	return 0;
}

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply ac_supply = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = supply_list,
	.num_supplicants = ARRAY_SIZE(supply_list),
	.properties = power_properties,
	.num_properties = ARRAY_SIZE(power_properties),
	.get_property = power_get_property,
};

static struct power_supply usb_supply = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = supply_list,
	.num_supplicants = ARRAY_SIZE(supply_list),
	.properties = power_properties,
	.num_properties = ARRAY_SIZE(power_properties),
	.get_property = power_get_property,
};

/* rpc related */
#define APP_BATT_PDEV_NAME		"rs30100001:00000000"
#define APP_BATT_PROG			0x30100001
#define APP_BATT_VER			MSM_RPC_VERS(0,0)
#define HTC_PROCEDURE_BATTERY_NULL	0
#define HTC_PROCEDURE_GET_BATT_LEVEL	1
#define HTC_PROCEDURE_GET_BATT_INFO	2
#define HTC_PROCEDURE_GET_CABLE_STATUS	3
#define HTC_PROCEDURE_SET_BATT_DELTA	4

static struct msm_rpc_endpoint *endpoint;

struct battery_info_reply {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C) from formula and ADC */
	u32 batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 full_bat;		/* Full capacity of battery (mAh) */
};

static int htc_battery_probe(struct platform_device *pdev)
{
	struct rpc_request_hdr req;	
	struct htc_get_batt_info_rep {
		struct rpc_reply_hdr hdr;
		struct battery_info_reply info;
	} rep;

	int rc;

	endpoint = msm_rpc_connect(APP_BATT_PROG, APP_BATT_VER, 0);
	if (IS_ERR(endpoint)) {
		printk(KERN_ERR "%s: init rpc failed! rc = %ld\n",
		       __FUNCTION__, PTR_ERR(endpoint));
		return PTR_ERR(endpoint);
	}

	/* must do this or we won't get cable status updates */
	rc = msm_rpc_call_reply(endpoint, HTC_PROCEDURE_GET_BATT_INFO,
				&req, sizeof(req),
				&rep, sizeof(rep),
				5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: get info failed\n", __FUNCTION__);

	power_supply_register(&pdev->dev, &ac_supply);
	power_supply_register(&pdev->dev, &usb_supply);

	return 0;
}

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= APP_BATT_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};

/* batt_mtoa server definitions */
#define BATT_MTOA_PROG				0x30100000
#define BATT_MTOA_VERS				0
#define RPC_BATT_MTOA_NULL			0
#define RPC_BATT_MTOA_SET_CHARGING_PROC		1
#define RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC	2
#define RPC_BATT_MTOA_LEVEL_UPDATE_PROC		3

struct rpc_batt_mtoa_cable_status_update_args {
	int status;
};

static int handle_battery_call(struct msm_rpc_server *server,
			       struct rpc_request_hdr *req, unsigned len)
{	
	struct rpc_batt_mtoa_cable_status_update_args *args;

	if (req->procedure != RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC)
		return 0;

	args = (struct rpc_batt_mtoa_cable_status_update_args *)(req + 1);
	args->status = be32_to_cpu(args->status);
	pr_info("cable_status_update: status=%d\n",args->status);

	args->status = !!args->status;

	if (vbus_present != args->status) {
		vbus_present = args->status;
		msm_hsusb_set_vbus_state(vbus_present);
		power_supply_changed(&ac_supply);
		power_supply_changed(&usb_supply);
	}
	return 0;
}

void notify_usb_connected(int status)
{
	printk("### notify_usb_connected(%d) ###\n", status);
	usb_status = status;
	power_supply_changed(&ac_supply);
	power_supply_changed(&usb_supply);
}

int is_ac_power_supplied(void)
{
	return vbus_present && (usb_status == 2);
}

static struct msm_rpc_server battery_server = {
	.prog = BATT_MTOA_PROG,
	.vers = BATT_MTOA_VERS,
	.rpc_call = handle_battery_call,
};

static int __init htc_battery_init(void)
{
	platform_driver_register(&htc_battery_driver);
	msm_rpc_create_server(&battery_server);
	return 0;
}

module_init(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");


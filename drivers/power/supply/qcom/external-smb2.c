#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/irq.h>
#include <linux/pmic-voter.h>
#include "smb-lib.h"
#include "smb-reg.h"
#include "battery.h"
#include "external-smb2.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define DEFAULT_INPUT_CURRENT_LIMIT 500000
#define DEFAULT_WPC_BOOT_DELAY 20
#define DEFAULT_WPC_FASTCHG_CURRENT 125000
#define DEFAULT_WPC_NORMAL_TEMP 420
#define DEFAULT_WPC_OVERHEAT_TEMP 450
#define DEFAULT_WPC_RETAILMODE_TEMP_DIFF 50
#define DEFAULT_WPC_VOTE_CURRENT 100000
void ext_smb2_read_dt(struct smb_charger *chg)
{
	struct device_node *node = chg->dev->of_node;
	u32 temp;
	int rc;

	chg->external_smb2 = of_property_read_bool(node,
						"qcom,external-chg-smb2");

	chg->two_pin_charger = of_property_read_bool(node,
						"qcom,two-pin-charger");

	rc = of_property_read_u32(node, "qcom,input-current_limit", &temp);
	if (rc < 0)
		chg->input_current_limit = DEFAULT_INPUT_CURRENT_LIMIT;
	else
		chg->input_current_limit = temp;

	chg->is_wpc_charger = of_property_read_bool(node,
						"qcom,is-wpc-charger");

	if (chg->is_wpc_charger) {
		chg->wpc_en_gpio = of_get_named_gpio(node,
						"qcom,wpc-en-gpio", 0);
		if (chg->wpc_en_gpio < 0)
			pr_err("wpc_en_gpio not available\n");

		rc = of_property_read_u32(node, "qcom,wpc-boot-delay",
						&temp);
		if (rc < 0)
			chg->wpc_boot_delay = DEFAULT_WPC_BOOT_DELAY;
		else
			chg->wpc_boot_delay = temp;

		rc = of_property_read_u32(node, "qcom,wpc-fastchg-current",
						&temp);
		if (rc < 0)
			chg->wpc_fastchg_current = DEFAULT_WPC_FASTCHG_CURRENT;
		else
			chg->wpc_fastchg_current = temp;

		rc = of_property_read_u32(node, "qcom,wpc-normal-temp",
						&temp);
		if (rc < 0)
			chg->wpc_normal_temp = DEFAULT_WPC_NORMAL_TEMP;
		else
			chg->wpc_normal_temp = temp;

		rc = of_property_read_u32(node, "qcom,wpc-overheat-temp",
						&temp);
		if (rc < 0)
			chg->wpc_overheat_temp = DEFAULT_WPC_OVERHEAT_TEMP;
		else
			chg->wpc_overheat_temp = temp;

		rc = of_property_read_u32(node,
				"qcom,wpc-retailmode-temp-diff", &temp);
		if (rc < 0)
			chg->wpc_retailmode_temp_diff =
					DEFAULT_WPC_RETAILMODE_TEMP_DIFF;
		else
			chg->wpc_retailmode_temp_diff = temp;

		rc = of_property_read_u32(node, "qcom,wpc-vote-current",
								&temp);
		if (rc < 0)
			chg->wpc_vote_current = DEFAULT_WPC_VOTE_CURRENT;
		else
			chg->wpc_vote_current = temp;
	}
}

//TODO:Remove the debug info after charge function is ok
static void smblib_debug_info(struct smb_charger *chg)
{
	int rc = -EINVAL;
	u8 data1, data2, data3;

	rc = smblib_read(chg, 0x00001061, &data1);
	rc = smblib_read(chg, 0x00001370, &data2);
	rc = smblib_read(chg, 0x00001607, &data3);
	pr_err("FSC=%d mA, USB limit=%d mA, MISC ICL=%d mA\n",
					data1*25, data2*25, data3*25);
}

static void smblib_debug_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							debug_work.work);

	smblib_debug_info(chg);
	schedule_delayed_work(&chg->debug_work,
			msecs_to_jiffies(30000));
}

static u8 usbicl_rerun_time;
#define AICL_RERUN_TIME_3S 0
#define AICL_RERUN_TIME_12S 1
#define AICL_RERUN_TIME_45S 2
#define AICL_RERUN_TIME_3min 3
static char *usbicl_rerun_time_text[] = {
		"3s", "12s", "45s", "3min"
};

static void smblib_usbicl_rerun_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							usbicl_rerun_work.work);
	int rc = -EINVAL;

	pr_info("Set AICL rerun timer to %s\n",
	usbicl_rerun_time_text[usbicl_rerun_time]);

	rc = smblib_masked_write(chg, AICL_RERUN_TIME_CFG_REG,
		AICL_RERUN_TIME_MASK, usbicl_rerun_time);
	if (rc < 0) {
		pr_err("Couldn't config AICL rerun time\n");
		return;
	}

	if (usbicl_rerun_time == AICL_RERUN_TIME_3S) {
		//delay 90s then set to rerun time to 3min
		schedule_delayed_work(&chg->usbicl_rerun_work,
				msecs_to_jiffies(90000));
		usbicl_rerun_time = AICL_RERUN_TIME_3min;
	}
}

void ext_smb2_init_hw(struct smb_charger *chg)
{
	int rc = 0;

	if (!chg->external_smb2)
		return;

	//Set usbin collapse time to 30us
	smblib_masked_write(chg, USBIN_LOAD_CFG_REG, 0x3, 0x3);

	if (chg->is_wpc_charger) {
		if (gpio_is_valid(chg->wpc_en_gpio)) {
			rc = gpio_request(chg->wpc_en_gpio,
					"qcom,wpc-en-gpio");
			if (rc < 0) {
				pr_err("gpio req failed for wpc-en-gpio\n");
				chg->wpc_en_gpio = 0;
			} else {
				gpio_direction_output(chg->wpc_en_gpio, 0);
			}
		}
		//Set PMIC WD reset disable
		smblib_write(chg, 0x857, 0x1);
		smblib_write(chg, 0x858, 0x1);
	}
}

static int batt_temp;
static bool wpc_normal_thermal;
static bool wpc_reset_done;
static bool wpc_enable;
static int boot_count;
static bool icl_reset_wpc_flag;

static void smblib_fake_charger_icon_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						fake_charger_icon_work.work);

	chg->fake_charger_icon_flag = 0;
	power_supply_changed(chg->usb_psy);
}

void ext_smblib_usb_plugin(struct smb_charger *chg, bool enable)
{
	if (!chg->external_smb2)
		return;

	if (enable) {
		usbicl_rerun_time = AICL_RERUN_TIME_3S;
		schedule_delayed_work(&chg->usbicl_rerun_work,
						msecs_to_jiffies(3000));
		schedule_delayed_work(&chg->debug_work,
						msecs_to_jiffies(5000));
		if (chg->is_wpc_charger) {
			cancel_delayed_work(&chg->fake_charger_icon_work);
			chg->fake_charger_icon_flag = 0;
			if (boot_count > 9 && wpc_reset_done)
				cancel_delayed_work(&chg->wpc_enable_work);

			schedule_delayed_work(&chg->alg_work,
						msecs_to_jiffies(33000));
		}
	} else {
		cancel_delayed_work(&chg->usbicl_rerun_work);
		cancel_delayed_work(&chg->debug_work);
		if (chg->is_wpc_charger) {
			cancel_delayed_work(&chg->alg_work);
			cancel_delayed_work(&chg->fake_charger_icon_work);
			chg->fake_charger_icon_flag = 1;
			schedule_delayed_work(&chg->fake_charger_icon_work,
						msecs_to_jiffies(2500));
			smblib_set_charge_param(chg, &chg->param.fcc, 50000);
			if (boot_count > 9) {
				cancel_delayed_work(&chg->wpc_enable_work);
				schedule_delayed_work(&chg->wpc_enable_work,
						msecs_to_jiffies(3000));
			}
		}
	}
}

void ext_smblib_usbicl_restart(struct smb_charger *chg)
{
	cancel_delayed_work(&chg->usbicl_rerun_work);
	usbicl_rerun_time = AICL_RERUN_TIME_3S;
	schedule_delayed_work(&chg->usbicl_rerun_work, msecs_to_jiffies(3000));
}

int ext_smb2_fake_charger_icon(struct smb_charger *chg)
{
	if (!chg->external_smb2)
		return 0;

	if (chg->fake_charger_icon_flag)
		return 1;
	else
		return 0;
}

static void smblib_set_WPC_enable(struct smb_charger *chg)
{
	if (!wpc_normal_thermal || !wpc_reset_done || !icl_reset_wpc_flag) {
		if (wpc_enable) {
			gpio_set_value(chg->wpc_en_gpio, 1);
			wpc_enable = 0;
			pr_info("smblib_set_wpc_enable: disable WPC\n");
		}
	} else {
		if (!wpc_enable) {
			gpio_set_value(chg->wpc_en_gpio, 0);
			wpc_enable = 1;
			pr_info("smblib_set_wpc_enable: enable WPC\n");
		}
	}
}

static void smblib_wpc_enable_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							wpc_enable_work.work);

	if (boot_count < chg->boot_delay_count) {
		boot_count++;
		schedule_delayed_work(&chg->wpc_enable_work,
				msecs_to_jiffies(2000));
	} else {
		if (wpc_reset_done) {
			wpc_reset_done = 0;
			smblib_set_WPC_enable(chg);
			schedule_delayed_work(&chg->wpc_enable_work,
					msecs_to_jiffies(1000));
		} else {
			wpc_reset_done = 1;
			smblib_set_WPC_enable(chg);
		}
	}
}

static void smblib_vbatt_therm_icl_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
				vbatt_therm_icl_work.work);

	int rc;
	u8 data;
	bool isretailmode;
	int retailmode_tuning;
	union power_supply_propval val;

	power_supply_get_property(chg->bms_psy,
				POWER_SUPPLY_PROP_RESTRICTED_CHARGING, &val);
	isretailmode = val.intval;
	if (isretailmode)
		retailmode_tuning = chg->wpc_retailmode_temp_diff;
	else
		retailmode_tuning = 0;

	power_supply_get_property(chg->bms_psy, POWER_SUPPLY_PROP_TEMP,
							&val);
	batt_temp = val.intval;
	if (batt_temp >= chg->wpc_overheat_temp +  retailmode_tuning
			&& wpc_normal_thermal) {
		wpc_normal_thermal = 0;
		pr_info("temperature high, disable WPC\n");
		smblib_set_WPC_enable(chg);
	} else if (batt_temp <= chg->wpc_normal_temp + retailmode_tuning
					&& !wpc_normal_thermal) {
		wpc_normal_thermal = 1;
		pr_info("temperature normal, enable WPC\n");
		smblib_set_WPC_enable(chg);
	}

	//ICL check
	rc = smblib_read(chg, 0x00001607, &data);
	pr_err("MISC ICL=%d mA\n", data*25);

	if (!data && icl_reset_wpc_flag) {
		icl_reset_wpc_flag = 0;
		smblib_set_WPC_enable(chg);
		schedule_delayed_work(&chg->vbatt_therm_icl_work,
			msecs_to_jiffies(1000));
	} else {
		if (!icl_reset_wpc_flag) {
			icl_reset_wpc_flag = 1;
			smblib_set_WPC_enable(chg);
		}
		schedule_delayed_work(&chg->vbatt_therm_icl_work,
			msecs_to_jiffies(30000));
	}
}

static void smblib_alg_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							alg_work.work);

	smblib_set_charge_param(chg, &chg->param.fcc,
					chg->wpc_fastchg_current);
}

int ext_smblib_input_current_limit(struct smb_charger *chg,
			struct smb_chg_param *param, int val_u)
{
	if (!chg->external_smb2)
		return val_u;

	if (chg->two_pin_charger || chg->is_wpc_charger) {
		if (!strcmp(param->name, "usb input current limit")) {
			pr_info("set usb input current to %d mA.\n",
						chg->input_current_limit);
			return chg->input_current_limit;
		}
	}
	return val_u;
}

void ext_smblib_init(struct smb_charger *chg)
{
	INIT_DELAYED_WORK(&chg->usbicl_rerun_work,
				smblib_usbicl_rerun_work);

	INIT_DELAYED_WORK(&chg->debug_work,
				smblib_debug_work);

	if (!chg->external_smb2)
		return;

	if (chg->is_wpc_charger) {
		batt_temp = 0;
		wpc_normal_thermal = 1;
		wpc_reset_done = 1;
		wpc_enable = 1;
		boot_count = 0;
		icl_reset_wpc_flag = 1;
		INIT_DELAYED_WORK(&chg->alg_work, smblib_alg_work);
		INIT_DELAYED_WORK(&chg->vbatt_therm_icl_work,
					smblib_vbatt_therm_icl_work);
		INIT_DELAYED_WORK(&chg->wpc_enable_work,
					smblib_wpc_enable_work);
		INIT_DELAYED_WORK(&chg->fake_charger_icon_work,
					smblib_fake_charger_icon_work);
		schedule_delayed_work(&chg->wpc_enable_work,
				msecs_to_jiffies(2000));
		schedule_delayed_work(&chg->vbatt_therm_icl_work,
				msecs_to_jiffies(30000));
		chg->fake_charger_icon_flag = 0;
		chg->boot_delay_count = chg->wpc_boot_delay / 2;
	}
}

void ext_smblib_power_ok(struct smb_charger *chg)
{
	if (!chg->external_smb2)
		return;

	pr_err("Reverse boost detected: voting 100mA to suspend input\n");
	vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true,
							chg->wpc_vote_current);
}

void ext_smb2_force_disable_hvdcp(struct smb_charger *chg, u8 *val)
{
	if (!chg->external_smb2)
		return;
	*val = 0;
}

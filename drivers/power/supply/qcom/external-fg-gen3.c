#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/alarmtimer.h>
#include <linux/of_platform.h>
#include <linux/of_batterydata.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/iio/consumer.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/qpnp/qpnp-misc.h>
#include "fg-core.h"
#include "fg-reg.h"
#include "external-fg-gen3.h"

#define BATT_CURRENT_NUMR	488281
#define BATT_CURRENT_DENR	1000
static int fg_get_battery_current(struct fg_chip *chip, int *val)
{
	int rc = 0;
	int64_t temp = 0;
	u8 buf[2];

	rc = fg_read(chip, BATT_INFO_IBATT_LSB(chip), buf, 2);
	if (rc < 0) {
		pr_err("failed to read addr=0x%04x, rc=%d\n",
			BATT_INFO_IBATT_LSB(chip), rc);
		return rc;
	}

	if (chip->wa_flags & PMI8998_V1_REV_WA)
		temp = buf[0] << 8 | buf[1];
	else
		temp = buf[1] << 8 | buf[0];

	pr_debug("buf: %x %x temp: %llx\n", buf[0], buf[1], temp);
	/* Sign bit is bit 15 */
	temp = twos_compliment_extend(temp, 15);
	*val = div_s64((s64)temp * BATT_CURRENT_NUMR, BATT_CURRENT_DENR);
	return 0;
}

static int fg_get_usb_online(struct fg_chip *chip, bool *usb_online)
{
	ssize_t ret = 0;
	struct power_supply *psy = power_supply_get_by_name("usb");
	union power_supply_propval value;

	value.intval = 0;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
						&value);
	if (ret < 0)
		pr_err("driver failed to report USB property\n");
	*usb_online = value.intval;

	return ret;
}

#define DEFAULT_COLD_FV			4200
#define DEFAULT_COLD_CC			100
#define DEFAULT_HOT_FV			4305
#define DEFAULT_HOT_CC			100
#define JEITA_DYNAMIC_HOT_THRES			350
#define JEITA_DYNAMIC_COLD_THRES			250
#define CHGR_FLOAT_VOLTAGE_CFG_REG		0x1070
#define CHGR_FAST_CHARGE_CURRENT_CFG_REG	0x1061
#define CHGR_JEITA_EN_CFG					0x1090
#define CHGR_JEITA_FVCOMP_CFG_REG			0x1091
#define CHGR_JEITA_CCCOMP_CFG_REG			0x1092
#define FV_STEP							75
#define CC_STEP							25
//JEITA FV = Float Voltage - (DATA x 7.5mV)
static void fg_set_fv_compensation(struct fg_chip *chip, int vfloat_comp)
{
	u8 stat = 0, delta = 0;
	int rc = 0, vfloat = 0;

	rc = fg_read(chip, CHGR_FLOAT_VOLTAGE_CFG_REG, &stat, 1);
	if (rc < 0) {
		pr_err("failed to read FLOAT_VOLTAGE_CFG_REG\n");
		return;
	}

	// float voltage unit is 0.1 mV
	vfloat = 34875 + stat * FV_STEP;
	pr_info("vfloat_cfg: 0x%x (%d mv), vfloat_comp: %d\n", stat,
			vfloat, vfloat_comp);

	vfloat_comp = vfloat_comp * 10;
	if (vfloat - vfloat_comp >= 0) {
		delta = (vfloat - vfloat_comp) / FV_STEP;
		if (((vfloat - vfloat_comp) % FV_STEP) != 0)
			delta++;
	}

	pr_debug("delta = %d, fv_comp_formula: %d - %d = %d\n", delta,
		vfloat, vfloat_comp, vfloat - vfloat_comp);
	rc = fg_write(chip, CHGR_JEITA_FVCOMP_CFG_REG, &delta, 1);
	if (rc < 0)
		pr_err("Couldn't write CHGR_JEITA_FVCOMP_CFG_REG\n");
}

//JEITA CC = Fast Charge Current - DATA x 25mA
static void fg_set_cc_compensation(struct fg_chip *chip, int current_comp)
{
	u8 stat = 0, delta = 0;
	int rc = 0, fast_curret = 0;

	rc = fg_read(chip, CHGR_FAST_CHARGE_CURRENT_CFG_REG, &stat, 1);
	if (rc < 0) {
		pr_err("failed to read FAST_CHARGE_CURRENT_CFG_REG\n");
		return;
	}

	// fast current comp ( mA)
	fast_curret = stat * CC_STEP;
	pr_info("fast_current_cfg: 0x%x (%d mA), current_comp: %d mA\n", stat,
			fast_curret, current_comp);

	if (fast_curret - current_comp >= 0) {
		delta = (fast_curret - current_comp) / (CC_STEP);
		if (((fast_curret - current_comp) % CC_STEP) != 0)
			delta++;
	}

	pr_debug("delta = %d, cc_comp_formula: %d - %d = %d\n", delta,
			fast_curret, current_comp,
			fast_curret - current_comp);
	rc = fg_write(chip, CHGR_JEITA_CCCOMP_CFG_REG, &delta, 1);
	if (rc < 0)
		pr_err("Couldn't write CHGR_JEITA_CCCOMP_CFG_REG\n");
}

static void fg_set_jeita_fv_cc_compensation(struct fg_chip *chip, int temp)
{
	pr_debug("smblib_set_jeita_fv_cc: temp = %d, model = %d\n", temp,
			chip->dt.jeita_dynamic_model);
	if ((temp > JEITA_DYNAMIC_HOT_THRES) &&
			(chip->dt.jeita_dynamic_model == MODEL_COLD)) {
		chip->dt.jeita_dynamic_model = MODEL_HOT;
		fg_set_fv_compensation(chip,
				chip->dt.jeita_soft_hot_fv_cc[JEITA_FV]);
		fg_set_cc_compensation(chip,
				chip->dt.jeita_soft_hot_fv_cc[JEITA_CC]);
	} else if ((temp < JEITA_DYNAMIC_COLD_THRES) &&
			(chip->dt.jeita_dynamic_model == MODEL_HOT)) {
		chip->dt.jeita_dynamic_model = MODEL_COLD;
		fg_set_fv_compensation(chip,
				chip->dt.jeita_soft_cold_fv_cc[JEITA_FV]);
		fg_set_cc_compensation(chip,
				chip->dt.jeita_soft_cold_fv_cc[JEITA_CC]);
	}
}

void ext_fg_jeita_compensation(struct fg_chip *chip, int temp)
{
	if (!chip->external_fg_gen3)
		return;

	//check the jeita model or not
	if (chip->dt.jeita_dynamic_model != MODEL_DISABLE)
		fg_set_jeita_fv_cc_compensation(chip, temp);
}

void ext_fg_jeita_compensation_init(struct fg_chip *chip)
{
	if (!chip->external_fg_gen3)
		return;

	if (chip->dt.jeita_en) {
		//wait fcc applied, then set jeita
		fg_set_fv_compensation(chip,
				chip->dt.jeita_soft_cold_fv_cc[JEITA_FV]);
		fg_set_cc_compensation(chip,
				chip->dt.jeita_soft_cold_fv_cc[JEITA_CC]);
		chip->dt.jeita_dynamic_model = MODEL_COLD;
	}
}

void ext_fg_jeita_enable(struct fg_chip *chip)
{
	int rc;
	u8 val;

	if (!chip->external_fg_gen3)
		return;

	if (chip->dt.jeita_en)
		val = 0x1f;
	else
		val = 0;
	rc = fg_write(chip, CHGR_JEITA_EN_CFG, &val, 1);
	if (rc < 0)
		pr_err("Error in writing CHGR_JEITA_EN_CFG\n");
}

#define DEFAULT_RECHARGE_COUNT	10
void fg_recharge_mode_detection(struct fg_chip *chip)
{
	int rc;
	int ibat;
	static int auto_recharger_counter;
	union power_supply_propval pval = {0, };

	if (!chip->external_fg_gen3)
		return;

	if (chip->charge_status == POWER_SUPPLY_STATUS_FULL &&
			auto_recharger_counter < DEFAULT_RECHARGE_COUNT) {
		fg_get_battery_current(chip, &ibat);
		if (ibat > 0)
			auto_recharger_counter = DEFAULT_RECHARGE_COUNT + 1;
		else
			auto_recharger_counter++;

		if (auto_recharger_counter == DEFAULT_RECHARGE_COUNT &&
			ibat < 1) {
			chip->dt.auto_recharge_soc = 0;

			pval.intval = 0;
			rc = power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_CALIBRATE,	&pval);

			fg_ext_set_recharge_voltage(chip);

		}
	}
}

static void twm_improve_work(struct work_struct *work)
{
	struct fg_chip *chip = container_of(work,
				struct fg_chip,
				twm_improve_work.work);
	bool usb_online;

	fg_get_usb_online(chip, &usb_online);

	if (chip->twm_improve_count > 20 || !chip->twm_improve_work_flag) {
		chip->twm_improve_work_flag = 0;
		cancel_delayed_work_sync(&chip->twm_improve_work);
	} else
		schedule_delayed_work(&chip->twm_improve_work,
				msecs_to_jiffies(3000));

	if (usb_online)
		chip->twm_improve_count = 0;
	else
		chip->twm_improve_count++;
}

#define FULL_CAPACITY	100
void ext_fg_soc_compensation(struct fg_chip *chip, int *msoc)
{
	bool usb_online;
	int twm_ibat;
	int upper_reserve_soc;
	int lower_reserve_soc;
	int scale_soc;

	if (!chip->external_fg_gen3) {
		chip->last_soc = *msoc;
		return;
	}

	upper_reserve_soc = FULL_CAPACITY - chip->full_soc_scale;
	lower_reserve_soc = chip->twm_soc_reserve;
	scale_soc = upper_reserve_soc + lower_reserve_soc;

	if (chip->fg_can_restart_flag == 1)
		chip->last_soc = *msoc;
	else
		*msoc = chip->last_soc;

	if (*msoc > chip->twm_soc_reserve && chip->twm_improve_work_flag) {
		chip->twm_improve_work_flag = 0;
	}

	if (chip->last_soc <= chip->twm_soc_reserve) {
		fg_get_usb_online(chip, &usb_online);
		fg_get_battery_current(chip, &twm_ibat);
		if (usb_online || twm_ibat < 0 || chip->twm_improve_work_flag)
			*msoc = 1;
		else
			*msoc = 0;
	} else if (chip->last_soc > chip->full_soc_scale) {
		*msoc = 100;
	} else
		*msoc = DIV_ROUND_CLOSEST(
				(*msoc-lower_reserve_soc) * FULL_CAPACITY,
				(FULL_CAPACITY - scale_soc));

}

static void fg_restart_work(struct work_struct *work)
{
	struct fg_chip *chip = container_of(work, struct fg_chip,
					    fg_restart_work.work);

	chip->fg_can_restart_flag = 1;
}

bool ext_fg_restart_need(struct fg_chip *chip)
{
	int msoc;
	bool usb_online;
	static bool fg_restart_once;

	if (!chip->external_fg_gen3)
		return 0;

	msoc = chip->last_soc;
	fg_get_usb_online(chip, &usb_online);
	if (!usb_online && fg_restart_once)
		fg_restart_once = 0;

	if (chip->charge_status == POWER_SUPPLY_STATUS_FULL &&
			msoc < 99 && !fg_restart_once) {
		if (chip->fg_can_restart_flag) {
			fg_restart_once = 1;
			chip->fg_can_restart_flag = 0;
			schedule_delayed_work(&chip->fg_restart_work,
					msecs_to_jiffies(3000));
			return 1;
		}
		return 0;
	} else
		return 0;
}

#define DEFAULT_TWM_SOC_RESERVER	0
#define DEFAULT_FULL_SOC_SCALE 100
void ext_fg_read_dt(struct fg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	u32 temp;
	int rc;

	chip->external_fg_gen3 = of_property_read_bool(node,
						"qcom,external-fg-gen3");

	rc = of_property_read_u32(node, "qcom,fg-jeita-en", &temp);
	if (rc < 0)
		chip->dt.jeita_en = 0;
	else
		chip->dt.jeita_en = temp;

	chip->dt.jeita_soft_hot_fv_cc[JEITA_FV] = DEFAULT_HOT_FV;
	chip->dt.jeita_soft_hot_fv_cc[JEITA_CC] = DEFAULT_HOT_CC;
	chip->dt.jeita_soft_cold_fv_cc[JEITA_FV] = DEFAULT_COLD_FV;
	chip->dt.jeita_soft_cold_fv_cc[JEITA_CC] = DEFAULT_COLD_CC;
	if (of_property_count_elems_of_size(node,
		"qcom,fg-jeita-soft-hot-fv-cc",
		sizeof(u32)) == JEITA_FV_CC_COUNT) {
		rc = of_property_read_u32_array(node,
				"qcom,fg-jeita-soft-hot-fv-cc",
				chip->dt.jeita_soft_hot_fv_cc,
				JEITA_FV_CC_COUNT);
		if (rc < 0)
			pr_err("Error reading fg-jeita-soft-hot-fv-cc, use default value\n");
	}

	if (of_property_count_elems_of_size(node,
		"qcom,fg-jeita-soft-cold-fv-cc",
		sizeof(u32)) == JEITA_FV_CC_COUNT) {
		rc = of_property_read_u32_array(node,
				"qcom,fg-jeita-soft-cold-fv-cc",
				chip->dt.jeita_soft_cold_fv_cc,
				JEITA_FV_CC_COUNT);
		if (rc < 0)
			pr_err("Error reading fg-jeita-soft-cold-fv-cc,	use default value\n");
	}

	rc = of_property_read_u32(node, "qcom,twm-soc-reserve", &temp);
	if (rc < 0)
		chip->twm_soc_reserve = DEFAULT_TWM_SOC_RESERVER;
	else
		chip->twm_soc_reserve = temp;

	rc = of_property_read_u32(node, "qcom,full-soc-scale", &temp);
	if (rc < 0)
		chip->full_soc_scale = DEFAULT_FULL_SOC_SCALE;
	else
		chip->full_soc_scale = temp;
}

void ext_fg_init(struct fg_chip *chip)
{
	chip->fg_can_restart_flag = 1;
	chip->g_isretailmode = false;
	chip->twm_improve_work_flag = 1;
	chip->twm_improve_count = 0;

	if (!chip->external_fg_gen3)
		return;

	INIT_DELAYED_WORK(&chip->fg_restart_work, fg_restart_work);
	INIT_DELAYED_WORK(&chip->twm_improve_work, twm_improve_work);

	chip->dt.jeita_dynamic_model = MODEL_DISABLE;
	schedule_delayed_work(&chip->twm_improve_work, 0);
}


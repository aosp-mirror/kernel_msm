/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "SMB348 %s: " fmt, __func__
#define FEATURE_THERMAL_MITIGATION_ALGO

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/workqueue.h>

#include <linux/switch.h>

#define _SMB348_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB348_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB348_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))

/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define CHG_OTH_CURRENT_CTRL_REG	0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define OTHER_CTRL_REG			0x9
#define TEMP_REG			0xB
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD

/* Command registers */
#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

/* IRQ status registers */
#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_F_REG			0x3A

/* Status registers */
#define STATUS_C_REG			0x3D
#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F

/* Config bits */
#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	SMB348_MASK(5, 4)
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_AUTO_RECHARGE_DIS_BIT		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		SMB348_MASK(5, 4)
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
/* This is to select if use external pin EN to control CHG */
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		SMB348_MASK(6, 5)
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		SMB348_MASK(6, 5)

#define CHG_LOW_BATT_THRESHOLD \
				SMB348_MASK(3, 0)
#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4)
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2)
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define VFLOAT_MASK				0x3F

/* IRQ status bits */
#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)

/* Status  bits */
#define STATUS_C_CHARGING_MASK			SMB348_MASK(2, 1)
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			SMB348_MASK(2, 1)
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_CHARGING_PORT_MASK \
				SMB348_MASK(3, 0)
#define STATUS_D_PORT_ACA_DOCK			BIT(3)
#define STATUS_D_PORT_SDP			BIT(2)
#define STATUS_D_PORT_DCP			BIT(1)
#define STATUS_D_PORT_CDP			BIT(0)
#define STATUS_D_PORT_OTHER			SMB348_MASK(1, 0)
#define STATUS_D_PORT_ACA_A			(BIT(2) | BIT(0))
#define STATUS_D_PORT_ACA_B			SMB348_MASK(2, 1)
#define STATUS_D_PORT_ACA_C			SMB348_MASK(2, 0)

/* constants */
#define DC_MA_MIN			100
#define DC_MA_MAX			1500
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MIN_CURRENT_MA		150
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0x70
#define AC_CHG_CURRENT_SHIFT		4
#define SMB348_IRQ_REG_COUNT		6
#define SMB348_FAST_CHG_MIN_MA		100
#define SMB348_FAST_CHG_MAX_MA		1500
#define SMB348_FAST_CHG_SHIFT		5
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB348_DEFAULT_BATT_CAPACITY	50
#define SMB348_DEFAULT_VOLTAGE		3760000
#define SMB348_DEFAULT_CURRENT		99999
#define SMB348_BATT_GOOD_THRE_2P5	0x1
#ifdef FEATURE_THERMAL_MITIGATION_ALGO
#define SMB348_SW_MITIGATION_SOC	75
#define SMB348_SW_RECHG_SOC		99
#define SMB348_SW_RECHG_VOLTAGE		4330000
#endif /* FEATURE_THERMAL_MITIGATION_ALGO */

/* Quanta { 3 mins delay for WPC charge */
#define WPC_CHG_RESUME_CHG_MS		180000
bool WPC_CHG_RESUME_ALLOW_GOTA = 1;
bool RECOVERY_WPC_DISABLE_FLAG = 0;
/* } Quanta */

extern char *saved_command_line;
// Quanta: export fo gauge driver
bool chg_full_flag = false;
bool force_full_flag = false;
bool recharge_flag = false;
#define SMB_MAX_SOC 100

enum {
	USER		= BIT(0),
	THERMAL		= BIT(1),
	CURRENT		= BIT(2),
	SOC		= BIT(3),
	FAKE_BATTERY	= BIT(4),
};

typedef enum {
	HW_VER_H	= 1,
	HW_VER_I	= 2,
	HW_VER_J	= 3,
	HW_VER_K	= 4,
	HW_VER_L	= 5,
} XU1_HW_VER;

struct smb348_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct smb348_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			inhibit_disabled;
	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_end_gpio;
	int			wpc_dock_detect_gpio;
	int			wpc_stat_gpio;
	int			wpc_enable_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			wpc_state;
	int			wpc_dock_present;
	int			fake_battery_soc;
	int			dc_psy_type;
	int			dc_psy_ma;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			pmic_vbat_sns;
	bool			battery_missing;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			irq_waiting;
	bool			bms_controlled_charging;
	bool			skip_usb_suspend_for_fake_battery;
	struct mutex		read_write_lock;
	struct mutex		path_suspend_lock;
	struct mutex		irq_complete;
	struct mutex		wpc_chg_complete;
	/* Quanta { 3 mins delay for WPC charge */
	struct delayed_work wpc_resume_chg_wq;
	/* } Quanta */
	struct work_struct	wpc_dock_wq;
	u8			irq_cfg_mask[2];
	int			irq_gpio;
	int			charging_disabled;
	int			fastchg_current_max_ma;
#ifdef FEATURE_THERMAL_MITIGATION_ALGO
	int			fastchg_current_mitigation_ma;
	int			prop_batt_capacity;
	int			prop_batt_temp;
#endif /* FEATURE_THERMAL_MITIGATION_ALGO */
	unsigned int		cool_bat_ma;
	unsigned int		warm_bat_ma;
	unsigned int		cool_bat_mv;
	unsigned int		warm_bat_mv;
	unsigned int		connected_rid;
	XU1_HW_VER		chip_hw_id;

	/* debugfs related */
#if defined(CONFIG_DEBUG_FS)
	struct dentry		*debug_root;
	u32			peek_poke_address;
#endif
	/* status tracking */
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	bool			jeita_supported;
	int			charging_disabled_status;
	int			usb_suspended;

	/* power supply */
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	batt_psy;
	struct power_supply	dc_psy;

	/* switch */
	struct switch_dev	dock_sdev;

	/* otg 5V regulator */
	struct smb348_regulator	otg_vreg;

	/* adc_tm paramters */
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	int			cold_bat_decidegc;
	int			hot_bat_decidegc;
	int			cool_bat_decidegc;
	int			warm_bat_decidegc;
	int			bat_present_decidegc;
	/* i2c pull up regulator */
	struct regulator	*vcc_i2c;
};

struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb348_charger *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

static int chg_current[] = {
	300, 500, 700, 900, 1000, 1200, 1500, 1800,
};

static int fast_chg_current[] = {
	100, 200, 375, 450, 600, 800, 1300, 1500,
};

/* add supplied to "bms" function */
static char *pm_batt_supplied_to[] = {
	"bms",
};

#define MAX_RW_RETRIES		3
static int __smb348_read_reg(struct smb348_charger *chip, u8 reg, u8 *val)
{
	s32 ret, i;

	for (i = 0; i < MAX_RW_RETRIES; i++) {
		ret = i2c_smbus_read_byte_data(chip->client, reg);
		if (ret >= 0)
			break;
		/* delay between i2c retries */
		msleep(20);
	}
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	}

	*val = ret;
	return 0;
}

static int __smb348_write_reg(struct smb348_charger *chip, int reg, u8 val)
{
	s32 ret, i;

	for (i = 0; i < MAX_RW_RETRIES; i++) {
		ret = i2c_smbus_write_byte_data(chip->client, reg, val);
		if (!ret)
			break;
		/* delay between i2c retries */
		msleep(20);
	}
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb348_read_reg(struct smb348_charger *chip, int reg,
						u8 *val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb348_read_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb348_write_reg(struct smb348_charger *chip, int reg,
						u8 val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb348_write_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb348_masked_write(struct smb348_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = __smb348_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev,
			"smb348_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb348_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev,
			"smb348_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb348_enable_volatile_writes(struct smb348_charger *chip)
{
	int rc;

	rc = smb348_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n",
				rc);

	return rc;
}

static XU1_HW_VER smb348_check_hw_id(void)
{
	char *pos = NULL;
	char hw_id;
	size_t len = 0;

	pos = strnstr(saved_command_line, "chip_hw_id", strlen(saved_command_line));
	len = strlen("chip_hw_id");
	pos += (len + 1);

	sscanf(pos, "%c", &hw_id);
	if (hw_id == 'H')
		return HW_VER_H;
	else if (hw_id == 'I')
		return HW_VER_I;
	else if (hw_id == 'J')
		return HW_VER_J;
	else if (hw_id == 'K')
		return HW_VER_K;
	else if (hw_id == 'L')
		return HW_VER_L;
	else
		return 0;
}

static int smb348_fastchg_current_set(struct smb348_charger *chip,
					unsigned int fastchg_current)
{
	int i;

	if ((fastchg_current < SMB348_FAST_CHG_MIN_MA) ||
		(fastchg_current >  SMB348_FAST_CHG_MAX_MA)) {
		dev_dbg(chip->dev, "bad fastchg current mA=%d asked to set\n",
						fastchg_current);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= fastchg_current)
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Invalid current setting %dmA\n",
						fastchg_current);
		i = 0;
	}

	i = i << SMB348_FAST_CHG_SHIFT;
	dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n",
					fastchg_current, i);

	return smb348_masked_write(chip, CHG_CURRENT_CTRL_REG,
				SMB_FAST_CHG_CURRENT_MASK, i);
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
#define VFLOAT_4350MV		4350
static int smb348_float_voltage_set(struct smb348_charger *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (VFLOAT_4350MV == vfloat_mv)
		temp = 0x2B;
	else if (vfloat_mv > VFLOAT_4350MV)
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV + 1;
	else
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb348_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

#define CHG_ITERM_30MA			0x00
#define CHG_ITERM_40MA			0x01
#define CHG_ITERM_60MA			0x02
#define CHG_ITERM_80MA			0x03
#define CHG_ITERM_100MA			0x04
#define CHG_ITERM_125MA			0x05
#define CHG_ITERM_150MA			0x06
#define CHG_ITERM_200MA			0x07
static int smb348_term_current_set(struct smb348_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled)
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");

		if (chip->iterm_ma <= 30)
			reg = CHG_ITERM_30MA;
		else if (chip->iterm_ma <= 40)
			reg = CHG_ITERM_40MA;
		else if (chip->iterm_ma <= 60)
			reg = CHG_ITERM_60MA;
		else if (chip->iterm_ma <= 80)
			reg = CHG_ITERM_80MA;
		else if (chip->iterm_ma <= 100)
			reg = CHG_ITERM_100MA;
		else if (chip->iterm_ma <= 125)
			reg = CHG_ITERM_125MA;
		else if (chip->iterm_ma <= 150)
			reg = CHG_ITERM_150MA;
		else
			reg = CHG_ITERM_200MA;

		rc = smb348_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = smb348_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	} else {
		rc = smb348_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK, 0);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't enable iterm rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define VFLT_300MV			0x0C
#define VFLT_200MV			0x08
#define VFLT_100MV			0x04
#define VFLT_50MV			0x00
#define VFLT_MASK			0x0C
static int smb348_recharge_and_inhibit_set(struct smb348_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->recharge_disabled)
		rc = smb348_masked_write(chip, CHG_CTRL_REG,
		CHG_CTRL_AUTO_RECHARGE_MASK, CHG_AUTO_RECHARGE_DIS_BIT);
	else
		rc = smb348_masked_write(chip, CHG_CTRL_REG,
			CHG_CTRL_AUTO_RECHARGE_MASK, 0x0);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set auto recharge en reg rc = %d\n", rc);
	}

	if (chip->inhibit_disabled)
		rc = smb348_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, 0x0);
	else
		rc = smb348_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set inhibit en reg rc = %d\n", rc);
	}

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv <= 50)
			reg = VFLT_50MV;
		else if (chip->recharge_mv <= 100)
			reg = VFLT_100MV;
		else if (chip->recharge_mv <= 200)
			reg = VFLT_200MV;
		else
			reg = VFLT_300MV;

		rc = smb348_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int smb348_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb348_charger *chip = rdev_get_drvdata(rdev);

	rc = smb348_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
							CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	return rc;
}

static int smb348_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb348_charger *chip = rdev_get_drvdata(rdev);

	rc = smb348_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	return rc;
}

static int smb348_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb348_charger *chip = rdev_get_drvdata(rdev);

	rc = smb348_read_reg(chip, CMD_A_REG, &reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't read OTG enable bit rc=%d, reg=%2x\n",
							rc, CMD_A_REG);
		return rc;
	}

	return  (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}

struct regulator_ops smb348_chg_otg_reg_ops = {
	.enable		= smb348_chg_otg_regulator_enable,
	.disable	= smb348_chg_otg_regulator_disable,
	.is_enabled	= smb348_chg_otg_regulator_is_enable,
};

#if 0 /* No OTG regulater*/
static int smb348_regulator_init(struct smb348_charger *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Allocate memory failed\n");
		return -ENOMEM;
	}

	/* Give the name, then will register */
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb348_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}
	return rc;
}
#endif

static int __smb348_path_suspend(struct smb348_charger *chip, bool suspend)
{
	int rc;

	rc = smb348_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK,
					suspend ? CMD_A_CHG_SUSP_EN_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set CMD_A reg, rc = %d\n", rc);

	return rc;
}

static int smb348_path_suspend(struct smb348_charger *chip, int reason,
								bool suspend)
{
	int rc = 0;
	int suspended;

	mutex_lock(&chip->path_suspend_lock);
	suspended = chip->usb_suspended;

	if (suspend == false)
		suspended &= ~reason;
	else
		suspended |= reason;

	if (!chip->usb_suspended && suspended) {
		rc = __smb348_path_suspend(chip, true);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	} else if (chip->usb_suspended && !suspended) {
		rc = __smb348_path_suspend(chip, false);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	}

	if (rc)
		dev_err(chip->dev, "Couldn't set/unset suspend rc = %d\n", rc);

	mutex_unlock(&chip->path_suspend_lock);

	return rc;
}


static int __smb348_charging_disable(struct smb348_charger *chip, bool disable)
{
	int rc;

	rc = smb348_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
			disable ? 0 : CMD_A_CHG_ENABLE_BIT);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT diable = %d, rc = %d\n",
				disable, rc);
	return rc;
}

static int smb348_charging_disable(struct smb348_charger *chip,
						int reason, int disable)
{
	int rc = 0;
	int disabled;

	disabled = chip->charging_disabled_status;

	pr_debug("reason = %d requested_disable = %d disabled_status = %d\n",
						reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (!!disabled == !!chip->charging_disabled_status)
		goto skip;

	rc = __smb348_charging_disable(chip, !!disabled);
	if (rc) {
		pr_err("Failed to disable charging rc = %d\n", rc);
		return rc;
	} else {
	/* will not modify online status in this condition */
		power_supply_changed(&chip->batt_psy);
	}

skip:
	chip->charging_disabled_status = disabled;
	return rc;
}

#define MAX_INV_BATT_ID		7700
#define MIN_INV_BATT_ID		7300
static int smb348_hw_init(struct smb348_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additonal settings
	 */
	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb348_enable_volatile_writes(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n",
				rc);
		return rc;
	}

	/* setup defaults for CHG_CNTRL_REG */
	reg = CHG_CTRL_BATT_MISSING_DET_THERM_IO;
	mask = CHG_CTRL_BATT_MISSING_DET_MASK;
	rc = smb348_masked_write(chip, CHG_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup defaults for PIN_CTRL_REG */
	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
	rc = smb348_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* setup USB suspend and APSD  */
	rc = smb348_masked_write(chip, VARIOUS_FUNC_REG,
		VARIOUS_FUNC_USB_SUSP_MASK, VARIOUS_FUNC_USB_SUSP_EN_REG_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n",
				rc);
		return rc;
	}

	if (!chip->disable_apsd)
		reg = CHG_CTRL_APSD_EN_BIT;
	else
		reg = 0;

	rc = smb348_masked_write(chip, CHG_CTRL_REG,
				CHG_CTRL_APSD_EN_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	/* Fault and Status IRQ configuration */
	reg = FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_HOT_COLD_SOFT_BIT
		| FAULT_INT_INPUT_UV_BIT | FAULT_INT_AICL_COMPLETE_BIT
		| FAULT_INT_INPUT_OV_BIT;
	rc = smb348_write_reg(chip, FAULT_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}
	reg = STATUS_INT_CHG_TIMEOUT_BIT | STATUS_INT_OTG_DETECT_BIT |
		STATUS_INT_BATT_OV_BIT | STATUS_INT_CHGING_BIT |
		STATUS_INT_CHG_INHI_BIT | STATUS_INT_INOK_BIT |
		STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT;
	rc = smb348_write_reg(chip, STATUS_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}
	/* setup THERM Monitor */
	rc = smb348_masked_write(chip, THERM_A_CTRL_REG,
		THERM_A_THERM_MONITOR_EN_MASK, THERM_A_THERM_MONITOR_EN_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	/* set the fast charge current limit */
	rc = smb348_fastchg_current_set(chip, chip->fastchg_current_max_ma);
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/* set the float voltage */
	rc = smb348_float_voltage_set(chip, chip->vfloat_mv);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}

	/* set iterm */
	rc = smb348_term_current_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set term current rc=%d\n", rc);

	/* set recharge */
	rc = smb348_recharge_and_inhibit_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set recharge para rc=%d\n", rc);

	/* suspend USB path for fake battery */
	if (!chip->skip_usb_suspend_for_fake_battery) {
		if ((chip->connected_rid >= MIN_INV_BATT_ID) &&
				(chip->connected_rid <= MAX_INV_BATT_ID)) {
			rc = smb348_path_suspend(chip, FAKE_BATTERY, true);
			if (!rc)
				dev_info(chip->dev,
					"Suspended USB path reason FAKE_BATTERY\n");
		}
	}

	/* enable/disable charging */
	if (chip->charging_disabled) {
		rc = smb348_charging_disable(chip, USER, 1);
		if (rc)
			dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
	} else {
		/*
		 * Enable charging explictly,
		 * because not sure the default behavior.
		 */
		rc = __smb348_charging_disable(chip, 0);
		if (rc)
			dev_err(chip->dev, "Couldn't enable charging\n");
	}

	/*
	* Workaround for recharge frequent issue: When battery is
	* greater than 4.2v, and charging is disabled, charger
	* stops switching. In such a case, system load is provided
	* by battery rather than input, even though input is still
	* there. Make reg09[0:3] to be a non-zero value which can
	* keep the switcher active
	*/
	rc = smb348_masked_write(chip, OTHER_CTRL_REG, CHG_LOW_BATT_THRESHOLD,
						SMB348_BATT_GOOD_THRE_2P5);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTHER_CTRL_REG, rc = %d\n",
								rc);

	return rc;
}

static enum power_supply_property smb348_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int smb348_get_prop_batt_status(struct smb348_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smb348_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	if (reg & STATUS_C_CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if ((reg & STATUS_C_CHARGING_MASK) &&
			!(reg & STATUS_C_CHG_ERR_STATUS_BIT)) {
		if ((chg_full_flag) && (chip->chip_hw_id >= HW_VER_K)) {
			chg_full_flag = false;
			force_full_flag = false;
		}
		return POWER_SUPPLY_STATUS_CHARGING;
	}

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb348_get_prop_batt_present(struct smb348_charger *chip)
{
	return !chip->battery_missing;
}

static int smb348_get_prop_batt_capacity(struct smb348_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		/* Avoid battery report negative value of capacity. if happened, report 0%. */
		if (ret.intval < 0) {
			pr_err("BMS capacity is mess, ret.intval=%d\n", ret.intval);
			chip->prop_batt_capacity = 0;
			return 0;
		}
#ifdef FEATURE_THERMAL_MITIGATION_ALGO
		// 99% recharged check is moved to smb348_get_prop_battery_voltage_now() due to need to check voltage is lower
		chip->prop_batt_capacity = ret.intval;  // smb348_get_prop_battery_voltage_now() is always called after this func

		// return 100% once charging-end is triggered.
		if (chg_full_flag || recharge_flag) {
			pr_err("bms soc=%d, chg_end=%d, chg_full_flag=%d, recharge_flag=%d\n", ret.intval, gpio_get_value(chip->chg_end_gpio), chg_full_flag, recharge_flag);
			return SMB_MAX_SOC;
		}

		pr_err("soc=%d, chg_end=%d, chg_full_flag=%d, recharge_flag=%d\n", ret.intval, gpio_get_value(chip->chg_end_gpio), chg_full_flag, recharge_flag);
#endif /* FEATURE_THERMAL_MITIGATION_ALGO */
		return ret.intval;
	}

	dev_dbg(chip->dev,
		"Couldn't get bms_psy, return default capacity\n");
	return SMB348_DEFAULT_BATT_CAPACITY;
}

static int smb348_get_prop_charge_type(struct smb348_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb348_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	reg &= STATUS_C_CHARGING_MASK;

	if (reg == STATUS_C_FAST_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_TAPER_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_C_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb348_get_prop_batt_health(struct smb348_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

#define DEFAULT_BATT_HOT_TEMP	650
#define DEFAULT_BATT_WARM_TEMP	450
#define DEFAULT_BATT_NORM_TEMP	250
#define DEFAULT_BATT_COOL_TEMP	150
#define DEFAULT_BATT_COLD_TEMP	0

#define DEFAULT_BATT_UPPER_TEMP	580
#define DEFAULT_BATT_LOWER_TEMP	20
static int smb348_get_prop_batt_temp(struct smb348_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
		POWER_SUPPLY_PROP_TEMP, &ret);
		pr_debug("Gauge temp:%d\n", ret.intval);
		chip->prop_batt_temp = ret.intval;  // Use variable to keep current temperature
		if (chip->wpc_state) {
			if (ret.intval >= 500) {
				pr_err("Temp too high, wpc disable\n");
				gpio_set_value(chip->wpc_enable_gpio, 1);
			}
		} else {
			if (ret.intval <= 450 &&
				gpio_get_value(chip->wpc_enable_gpio) &&
				WPC_CHG_RESUME_ALLOW_GOTA &&
				!RECOVERY_WPC_DISABLE_FLAG) {
				pr_err("Temp normal, wpc enable\n");
				gpio_set_value(chip->wpc_enable_gpio, 0);
			}
		}
	}

	if (chip->batt_hot) {
		if (chip->bms_psy)
			pr_err("Battery hot irq trigger, gauge temp:%d\n", ret.intval);
		else
			pr_err("Battery hot irq trigger\n");

		return DEFAULT_BATT_HOT_TEMP;
	}
	if (chip->batt_warm)
		return DEFAULT_BATT_WARM_TEMP;
	if (chip->batt_cold)
		return DEFAULT_BATT_COLD_TEMP;
	if (chip->batt_cool)
		return DEFAULT_BATT_COOL_TEMP;

	if (chip->bms_psy) {
		if (ret.intval >= DEFAULT_BATT_UPPER_TEMP) {
			pr_err("*** Temp close shutdown point, but didn't receive batt hot irq ***\n");
			pr_err("*** Keep current temp(%d) to 58\n", ret.intval);
			return DEFAULT_BATT_UPPER_TEMP;
		} else if (ret.intval <= DEFAULT_BATT_LOWER_TEMP) {
			pr_err("*** Temp close cold point, but didn't receive batt cold irq ***\n");
			pr_err("*** Keep current temp(%d) to 2\n", ret.intval);
			return DEFAULT_BATT_LOWER_TEMP;
		} else {
			return ret.intval;
		}
	} else {
		pr_err("No bms power supply, report normal temp\n");
		return DEFAULT_BATT_NORM_TEMP;
	}
}

static int
smb348_get_prop_battery_voltage_now(struct smb348_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
#ifdef FEATURE_THERMAL_MITIGATION_ALGO
		pr_err("chg_end=%d, chg_full_flag=%d\n", gpio_get_value(chip->chg_end_gpio), chg_full_flag);
		if (chg_full_flag) {
			/*******************Release CHG_END critera:*********************
			 *	1. SOC		<= 99%					*
			 *	2. voltage	< 4330 mV				*
			 * If temperature lower than 10 degree C, only check SOC <= 99	*
			 ****************************************************************/
			if ((chip->prop_batt_capacity <= SMB348_SW_RECHG_SOC && ret.intval < 4350000) ||
				(ret.intval < SMB348_SW_RECHG_VOLTAGE && chip->prop_batt_temp > 100 )) {
				u8 reg;
				int rc;
				rc = smb348_read_reg(chip, IRQ_C_REG, &reg);
				/* Check the charge full status bit(REG 37h, bit(0)) has been clear */
				if (!(reg & IRQ_C_TERM_BIT)) {
					pr_err("SW recharge is due to batt capacity=%d or voltage=%d lower than %d\n", chip->prop_batt_capacity, ret.intval, SMB348_SW_RECHG_VOLTAGE);
					gpio_set_value(chip->chg_end_gpio, 0);
					chg_full_flag = false;
					force_full_flag = false;
					recharge_flag = true;
				} else {
					pr_err("Cannot release CHG_END, rc = %d and reg = 0x%02x\n", rc, reg);
				}
			}
		}
#endif /* FEATURE_THERMAL_MITIGATION_ALGO */
		return ret.intval;
	}

	dev_dbg(chip->dev,
		"Couldn't get bms_psy, return default voltage\n");
		return SMB348_DEFAULT_VOLTAGE;
}

static int
smb348_get_prop_battery_current_now(struct smb348_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	}

	dev_dbg(chip->dev,
		"Couldn't get bms_psy, return default current\n");
		return SMB348_DEFAULT_CURRENT;
}


static int smb348_set_usb_chg_current(struct smb348_charger *chip,
		int current_ma)
{
	int i, rc = 0;
	u8 reg1 = 0, reg2 = 0, mask = 0;

	dev_dbg(chip->dev, "%s: USB current_ma = %d\n", __func__, current_ma);

	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "%s: Charger in autonmous mode\n", __func__);
		return 0;
	}

	if (current_ma < USB3_MIN_CURRENT_MA && current_ma != 2)
		current_ma = USB2_MIN_CURRENT_MA;

	if (current_ma == USB2_MIN_CURRENT_MA) {
		/* USB 2.0 - 100mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 &= ~CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB2_MAX_CURRENT_MA) {
		/* USB 2.0 - 500mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB3_MAX_CURRENT_MA) {
		/* USB 3.0 - 900mA */
		reg1 |= USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma > USB2_MAX_CURRENT_MA) {
		/* HC mode  - if none of the above */
		reg2 |= CMD_B_CHG_HC_ENABLE_BIT;

		for (i = ARRAY_SIZE(chg_current) - 1; i >= 0; i--) {
			if (chg_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
			i = 0;
		}

		i = i << AC_CHG_CURRENT_SHIFT;
		rc = smb348_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						AC_CHG_CURRENT_MASK, i);
		if (rc)
			dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
	}

	mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
	rc = smb348_masked_write(chip, CMD_B_REG, mask, reg2);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set charging mode rc = %d\n", rc);

	mask = USB3_ENABLE_MASK;
	rc = smb348_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set USB3 mode rc = %d\n", rc);

	/* Only set suspend bit when chg present and current_ma = 2 */
	if (current_ma == 2 && chip->chg_present) {
		rc = smb348_path_suspend(chip, CURRENT, true);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);
	} else {
		rc = smb348_path_suspend(chip, CURRENT, false);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set susp rc = %d\n", rc);
	}

	return rc;
}

static enum power_supply_property smb348_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};


static int smb348_dc_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb348_charger *chip = container_of(psy,
				struct smb348_charger, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (chip->chip_hw_id >= HW_VER_K)
			val->intval = chip->wpc_dock_present;
		else
			val->intval = chip->wpc_state;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (chip->chip_hw_id >= HW_VER_K)
			val->intval = chip->wpc_dock_present;
		else {
			/* return if dc is charging the battery */
			val->intval = !chip->charging_disabled_status ? chip->wpc_state : 0;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chip->chip_hw_id >= HW_VER_K)
			val->intval = chip->wpc_dock_present;
		else
			val->intval = chip->wpc_state;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->dc_psy_ma * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int
smb348_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
		return 1;
	default:
		break;
	}

	return 0;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(soc, SMB_MAX_SOC);
	return soc;
}

static int smb348_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int rc;
	struct smb348_charger *chip = container_of(psy,
				struct smb348_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->bms_controlled_charging)
			return -EINVAL;
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
			rc = smb348_charging_disable(chip, SOC, true);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set charging disable rc = %d\n",
					rc);
			} else {
				chip->batt_full = true;
				dev_dbg(chip->dev, "status = FULL, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			chip->batt_full = false;
			power_supply_changed(&chip->batt_psy);
			dev_dbg(chip->dev, "status = DISCHARGING, batt_full = %d\n",
							chip->batt_full);
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			rc = smb348_charging_disable(chip, SOC, false);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set charging disable rc = %d\n",
								rc);
			} else {
				chip->batt_full = false;
				dev_dbg(chip->dev, "status = CHARGING, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb348_charging_disable(chip, USER, !val->intval);
		smb348_path_suspend(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = bound_soc(val->intval);
		power_supply_changed(&chip->batt_psy);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb348_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb348_charger *chip = container_of(psy,
				struct smb348_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb348_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb348_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb348_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !(chip->charging_disabled_status & USER);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb348_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb348_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB348";
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb348_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb348_get_prop_battery_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb348_get_prop_battery_current_now(chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int apsd_complete(struct smb348_charger *chip, u8 status)
{
	int rc;
	u8 reg = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	/*
	 * If apsd is disabled, charger detection is done by
	 * DCIN UV irq.
	 * status = ZERO - indicates charger removed, handled
	 * by DCIN UV irq
	 */
	if (chip->disable_apsd || status == 0) {
		dev_dbg(chip->dev, "APSD %s, status = %d\n",
			chip->disable_apsd ? "disabled" : "enabled", !!status);
		return 0;
	}

	rc = smb348_read_reg(chip, STATUS_D_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STATUS D rc = %d\n", rc);
		return rc;
	}

	dev_dbg(chip->dev, "%s: STATUS_D_REG=%x\n", __func__, reg);

	switch (reg & STATUS_D_CHARGING_PORT_MASK) {
	case STATUS_D_PORT_ACA_DOCK:
	case STATUS_D_PORT_ACA_C:
	case STATUS_D_PORT_ACA_B:
	case STATUS_D_PORT_ACA_A:
		type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_D_PORT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_D_PORT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_D_PORT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_D_PORT_OTHER:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}

	chip->chg_present = !!status;

	dev_dbg(chip->dev, "APSD complete. USB type detected=%d chg_present=%d",
						type, chip->chg_present);
	power_supply_set_supply_type(chip->usb_psy, type);

	 /* SMB is now done sampling the D+/D- lines, indicate USB driver */
	dev_dbg(chip->dev, "%s updating usb_psy present=%d", __func__,
			chip->chg_present);
	power_supply_set_present(chip->usb_psy, chip->chg_present);

	return 0;
}

static int chg_uv(struct smb348_charger *chip, u8 status)
{
	int rc;

	/* use this to detect USB insertion only if !apsd */
	if (chip->disable_apsd && status == 0) {
		chip->chg_present = true;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		if (!chip->wpc_state) {
			pr_err("TYPE: USB\n");
			power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB);
		} else {
			pr_err("TYPE: USB DCP\n");
			power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB_DCP);
		}
		power_supply_set_present(chip->usb_psy, chip->chg_present);

		if (chip->bms_controlled_charging) {
			/*
			* Disable SOC based USB suspend to enable charging on
			* USB insertion.
			*/
			rc = smb348_charging_disable(chip, SOC, false);
			if (rc < 0)
				dev_err(chip->dev,
				"Couldn't disable usb suspend rc = %d\n",
								rc);
		}
	}

	if (status != 0) {
		if (chip->chip_hw_id >= HW_VER_K) {
			if (chip->wpc_dock_present)
				return 0;
		}
		chip->chg_present = false;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
	/* we can't set usb_psy as UNKNOWN here, will lead USERSPACE issue */
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	power_supply_changed(chip->usb_psy);

	dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

	return 0;
}

static int chg_ov(struct smb348_charger *chip, u8 status)
{
	u8 psy_health_sts;
	if (status)
		psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;

	power_supply_set_health_state(
				chip->usb_psy, psy_health_sts);
	power_supply_changed(chip->usb_psy);

	return 0;
}

#define STATUS_FAST_CHARGING BIT(6)
static int fast_chg(struct smb348_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s\n", __func__);

	if (status & STATUS_FAST_CHARGING)
		chip->batt_full = false;
	return 0;
}

static int chg_term(struct smb348_charger *chip, u8 status)
{
	bool check_dock = 0;
	/* Pull up the CHG_END gpio to stop blink LED,
	   And set chg_full_flag. */
	mutex_lock(&chip->wpc_chg_complete);

	if (chip->chip_hw_id >= HW_VER_K)
		check_dock = chip->wpc_dock_present;
	else
		check_dock = chip->wpc_state;
	/* chg_full_flag avoid device enter charge <-> charge full loop */
	if (!chg_full_flag && check_dock) {
		pr_err("Full-Charged\n");
		gpio_set_value(chip->chg_end_gpio, 1);
		chg_full_flag = true;
		force_full_flag = true;
		recharge_flag = false;
#ifndef FEATURE_THERMAL_MITIGATION_ALGO
		msleep(100);
		gpio_set_value(chip->chg_end_gpio, 0);
#endif /* FEATURE_THERMAL_MITIGATION_ALGO */
	}
	mutex_unlock(&chip->wpc_chg_complete);

	pr_err("Full-Charged\n");
	if (chip->chip_hw_id < HW_VER_K) {
		if (!chip->iterm_disabled) {
			chip->batt_full = !!status;
		}
	} else {
		printk("status = %d\n",status);
		if (chg_full_flag)
			chip->batt_full = true;
	}
	return 0;
}

static int taper_chg(struct smb348_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s\n", __func__);
	return 0;
}

static int chg_recharge(struct smb348_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s, status = %d\n", __func__, !!status);
	/* to check the status mean */
	chip->batt_full = !status;
	return 0;
}

static void smb348_wpc_chg_gpio_clear(struct smb348_charger *chip)
{
	if (gpio_is_valid(chip->chg_end_gpio)) {
		if (gpio_get_value(chip->chg_end_gpio)) {
			pr_err("Clear chg_end_gpio\n");
			gpio_set_value(chip->chg_end_gpio, 0);
			chg_full_flag = false;
			force_full_flag = false;
		}
	}
	if (gpio_is_valid(chip->wpc_enable_gpio)) {
		if (gpio_get_value(chip->wpc_enable_gpio)) {
			pr_err("Clear wpc_enable_gpio\n");
			gpio_set_value(chip->wpc_enable_gpio, 0);
		}
	}
}

static void smb348_chg_set_appropriate_battery_current(
				struct smb348_charger *chip)
{
	int rc;
	unsigned int current_max = chip->fastchg_current_max_ma;

	if (chip->batt_cool)
		current_max =
			min(current_max, chip->cool_bat_ma);
	if (chip->batt_warm)
		current_max =
			min(current_max, chip->warm_bat_ma);
	dev_dbg(chip->dev, "setting %dmA", current_max);
	rc = smb348_fastchg_current_set(chip, current_max);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);
}

static void smb348_chg_set_appropriate_vddmax(
				struct smb348_charger *chip)
{
	int rc;
	unsigned int vddmax = chip->vfloat_mv;

	if (chip->batt_cool)
		vddmax = min(vddmax, chip->cool_bat_mv);
	if (chip->batt_warm)
		vddmax = min(vddmax, chip->warm_bat_mv);

	dev_dbg(chip->dev, "setting %dmV\n", vddmax);
	rc = smb348_float_voltage_set(chip, vddmax);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);
}

#define HYSTERESIS_DECIDEGC 20
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb348_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0, bat_warm = 0,
							bat_cool = 0;
	int temp;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invallid state parameter %d\n", state);
		return;
	}

	temp = smb348_get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->hot_bat_decidegc) {
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
			chip->warm_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->warm_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc;
		} else if (temp >=
			chip->cool_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cool_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
		} else if (temp >=
			chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cold_bat_decidegc - HYSTERESIS_DECIDEGC;
			if (chip->jeita_supported)
				chip->adc_param.high_temp =
						chip->cool_bat_decidegc;
			else
				chip->adc_param.high_temp =
						chip->hot_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >= chip->bat_present_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp = chip->cold_bat_decidegc;
			chip->adc_param.low_temp = chip->bat_present_decidegc
							- HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->bat_present_decidegc) {
			bat_cold = true;
			bat_cool = false;
			bat_hot = false;
			bat_warm = false;
			bat_present = false;
			chip->adc_param.high_temp = chip->bat_present_decidegc
							+ HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (temp <= chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + HYSTERESIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->cool_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cold_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->warm_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->hot_bat_decidegc) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			if (chip->jeita_supported)
				chip->adc_param.low_temp =
					chip->warm_bat_decidegc;
			else
				chip->adc_param.low_temp =
					chip->cold_bat_decidegc;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present)
		chip->battery_missing = false;
	else
		chip->battery_missing = true;

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
		/* stop charging explicitly since we use PMIC thermal pin*/
		if (bat_hot || bat_cold || chip->battery_missing)
			smb348_charging_disable(chip, THERMAL, 1);
		else
			smb348_charging_disable(chip, THERMAL, 0);
	}

	if ((chip->batt_warm ^ bat_warm || chip->batt_cool ^ bat_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = bat_warm;
		chip->batt_cool = bat_cool;
		smb348_chg_set_appropriate_battery_current(chip);
		smb348_chg_set_appropriate_vddmax(chip);
	}

	pr_debug("hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported, chip->battery_missing,
		chip->adc_param.low_temp, chip->adc_param.high_temp);
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

/* only for SMB thermal */
static int hot_hard_handler(struct smb348_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_hot = !!status;
	return 0;
}
static int cold_hard_handler(struct smb348_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cold = !!status;
	return 0;
}
static int hot_soft_handler(struct smb348_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_warm = !!status;
	return 0;
}
static int cold_soft_handler(struct smb348_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cool = !!status;
	return 0;
}

static int battery_missing(struct smb348_charger *chip, u8 status)
{
	chip->battery_missing = !!status;
	return 0;
}

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "cold_soft",
				.smb_irq	= cold_soft_handler,
			},
			{
				.name		= "hot_soft",
				.smb_irq	= hot_soft_handler,
			},
			{
				.name		= "cold_hard",
				.smb_irq	= cold_hard_handler,
			},
			{
				.name		= "hot_hard",
				.smb_irq	= hot_hard_handler,
			},
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_hot",
			},
			{
				.name		= "vbat_low",
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing
			},
			{
				.name		= "battery_ov",
			},
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_term",
				.smb_irq	= chg_term,
			},
			{
				.name		= "taper",
				.smb_irq	= taper_chg,
			},
			{
				.name		= "recharge",
				.smb_irq	= chg_recharge,
			},
			{
				.name		= "fast_chg",
				.smb_irq	= fast_chg,
			},
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
			},
			{
				.name		= "aicl_complete",
			},
			{
				.name		= "src_detect",
				.smb_irq	= apsd_complete,
			},
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "usbin_uv",
				.smb_irq        = chg_uv,
			},
			{
				.name		= "usbin_ov",
				.smb_irq	= chg_ov,
			},
			{
				.name		= "unknown",
			},
			{
				.name		= "unknown",
			},
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "power_ok",
			},
			{
				.name		= "otg_det",
			},
			{
				.name		= "otg_batt_uv",
			},
			{
				.name		= "otg_oc",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb348_chg_stat_handler(int irq, void *dev_id)
{
	struct smb348_charger *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb348_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static irqreturn_t smb348_chg_valid_handler(int irq, void *dev_id)
{
	struct smb348_charger *chip = dev_id;
	int present;

	present = gpio_get_value_cansleep(chip->chg_valid_gpio);
	if (present < 0) {
		dev_err(chip->dev, "Couldn't read chg_valid gpio=%d\n",
						chip->chg_valid_gpio);
		return IRQ_HANDLED;
	}
	present ^= chip->chg_valid_act_low;

	dev_dbg(chip->dev, "%s: chg_present = %d\n", __func__, present);

	if (present != chip->chg_present) {
		chip->chg_present = present;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	return IRQ_HANDLED;
}

/* smb348_wpc_handler to monitor WPC in/out interrupt */
#define WPC_CHG_FULL_MS		2000
static irqreturn_t smb348_wpc_state_handler(int irq, void *dev_id)
{
	struct smb348_charger *chip = dev_id;

	chip->wpc_state = !gpio_get_value(chip->wpc_stat_gpio);
	pr_err("wpc status pin changed!! chg_end=%d state=%d enable=%d, detect=%d\n", gpio_get_value(chip->chg_end_gpio), chip->wpc_state, gpio_get_value(chip->wpc_enable_gpio), gpio_get_value(chip->wpc_dock_detect_gpio));

	if (gpio_get_value(chip->chg_end_gpio) | gpio_get_value(chip->wpc_enable_gpio)) {
		pr_err("Stop Charging: wpc_state=%d,  wpc_dock_present=%d\n", chip->wpc_state, gpio_get_value(chip->wpc_dock_detect_gpio));
		return IRQ_HANDLED;
	} else if (0) {  // toDO
		// stop charging (not caused by chg_eng or enable pin). Measns remove from dock
		// toDO: device may be still on dock but 
		//         1. something wrong in dock or
		//         2. JEITA protection

		return IRQ_HANDLED;
	}
	else {
		chip->wpc_dock_present = chip->wpc_state;
	}

	pr_err("Updated wpc_dock_present = %d wpc_state=%d\n", chip->wpc_dock_present, chip->wpc_state);
	schedule_work(&chip->wpc_dock_wq);
	return IRQ_HANDLED;
}

/* smb348_wpc_dock_handler to monitor dock mode in/out interrupt */
/* only called by HW rev. K or later */
static irqreturn_t smb348_wpc_dock_handler(int irq, void *dev_id)
{
	struct smb348_charger *chip = dev_id;
	// chg eng (L: charging/blink; H: discharging/solid on)
	// state   (L: charging; H: discharging)
	// enable  (L: charging; H: discharging)
	// detect  (L: on-dock;  H: off-dock)
	chip->wpc_dock_present = !gpio_get_value(chip->wpc_dock_detect_gpio);
	pr_err("wpc detect pin changed!! chg_end=%d state=%d enable=%d, detect=%d\n",
			gpio_get_value(chip->chg_end_gpio), !gpio_get_value(chip->wpc_stat_gpio), gpio_get_value(chip->wpc_enable_gpio), chip->wpc_dock_present);
	schedule_work(&chip->wpc_dock_wq);
	return IRQ_HANDLED;
}

/* Quanta { 3 mins delay for WPC charge */
static void smb348_wpc_resume_to_charge(struct work_struct *work)
{
	struct smb348_charger *chip = container_of(work,
			struct smb348_charger, wpc_resume_chg_wq.work);

	pr_err("[DDDBG] resume charging - chg_full_flag = %d, wpc_state= %d\n",
			 chg_full_flag, chip->wpc_state);
	gpio_set_value(chip->wpc_enable_gpio, 0);
	WPC_CHG_RESUME_ALLOW_GOTA = 1;
}
/* } Quanta */

static void smb348_wpc_dock_work(struct work_struct *work)
{
	int state = 0;
	struct smb348_charger *chip = container_of(work,
			struct smb348_charger, wpc_dock_wq);

	state = chip->dock_sdev.state;
	pr_err("current dock state = %d, chip->wpc_dock_present = %d\n", state, chip->wpc_dock_present);
	if (chip->wpc_dock_present != state) {
		switch_set_state(&chip->dock_sdev, chip->wpc_dock_present);
		power_supply_changed(&chip->dc_psy);
		pr_err("Dock state changed - %s\n", chip->dock_sdev.state?"PRESENT":"REMOVED");
	}
	/* removed from dock so clear the discharge gpio */
	if (!chip->wpc_dock_present) {
		smb348_wpc_chg_gpio_clear(chip);
		recharge_flag = false;
		chip->chg_present = false;
		if (chip->batt_full) {
			chip->batt_full = false;
		}
		power_supply_set_present(chip->usb_psy, chip->chg_present);
		power_supply_changed(chip->usb_psy);
	}
}

static void smb348_external_power_changed(struct power_supply *psy)
{
	struct smb348_charger *chip = container_of(psy,
				struct smb348_charger, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc)
		dev_err(chip->dev,
			"Couldn't read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	/* Hardcode 500mA if WPC present */
	if (chip->wpc_state || chip->wpc_dock_present || chip->chg_present) {
		pr_debug("USB psy current max=%d!! hardcode 500mA\n", current_limit);
		current_limit = 500;
	}

	smb348_enable_volatile_writes(chip);
	smb348_set_usb_chg_current(chip, current_limit);

	dev_dbg(chip->dev, "current_limit = %d\n", current_limit);
}

#if defined(CONFIG_DEBUG_FS)
#define LAST_CNFG_REG	0x13
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb348_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb348_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb348_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x33
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb348_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb348_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb348_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x35
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb348_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb348_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb348_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb348_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb348_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb348_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb348_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb348_write_reg(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb348_charger *chip = data;

	smb348_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");
#endif


#ifdef DEBUG
static void dump_regs(struct smb348_charger *chip)
{
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb348_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb348_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb348_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}
}
#else
static void dump_regs(struct smb348_charger *chip)
{
}
#endif

static int smb_parse_batt_id(struct smb348_charger *chip)
{
	int rc = 0, rpull = 0, vref = 0;
	int64_t denom, batt_id_uv, numerator;
	struct device_node *node = chip->dev->of_node;
	struct qpnp_vadc_result result;

	rc = of_property_read_u32(node, "qcom,batt-id-vref-uv", &vref);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read batt-id-vref-uv rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,batt-id-rpullup-kohm", &rpull);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read batt-id-rpullup-kohm rc=%d\n", rc);
		return rc;
	}

	/* read battery ID */
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't read batt id channel=%d, rc=%d\n",
			LR_MUX2_BAT_ID, rc);
		return rc;
	}
	batt_id_uv = result.physical;

	if (batt_id_uv == 0) {
		/*vadc not correct or batt id line grounded, report 0 kohms */
		dev_warn(chip->dev, "batt_id_uv=0, batt-id grounded\n");
		return 0;
	}

	numerator = batt_id_uv * rpull * 1000;
	denom = vref  - batt_id_uv;

	/* batt id connector might be open, return 0 kohms */
	if (denom == 0)
		return 0;

	chip->connected_rid = div64_s64(numerator, denom);

	dev_dbg(chip->dev,
		"batt_id_voltage=%lld numerator=%lld denom=%lld connected_rid=%d\n",
		batt_id_uv, numerator, denom, chip->connected_rid);

	return 0;
}

static int smb_parse_dt(struct smb348_charger *chip)
{
	int rc;
	enum of_gpio_flags gpio_flags;
	struct device_node *node = chip->dev->of_node;
	int batt_present_degree_negative;
	const char *dc_psy_type;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->charging_disabled = of_property_read_bool(node,
					"qcom,charger-disabled");

	chip->inhibit_disabled = of_property_read_bool(node,
					"qcom,chg-inhibit-disabled");
	chip->chg_autonomous_mode = of_property_read_bool(node,
					"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	/*chip->using_pmic_therm = of_property_read_bool(node,
						"qcom,using-pmic-therm");*/
	chip->pmic_vbat_sns = of_property_read_bool(node,
					"qcom,using-vbat-sns");
	chip->bms_controlled_charging = of_property_read_bool(node,
						"qcom,bms-controlled-charging");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	chip->chg_valid_gpio = of_get_named_gpio_flags(node,
				"qcom,chg-valid-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(chip->chg_valid_gpio))
		dev_dbg(chip->dev, "Invalid chg-valid-gpio");
	else
		chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;

	chip->chg_end_gpio = of_get_named_gpio(node,
				"qcom,chg-end-gpio", 0);
	chip->wpc_stat_gpio = of_get_named_gpio(node,
				"qcom,wpc-stat-gpio", 0);

	chip->wpc_enable_gpio = of_get_named_gpio(node,
				"qcom,wpc-enable-gpio", 0);

	if (chip->chip_hw_id >= HW_VER_K) {
		chip->wpc_dock_detect_gpio = of_get_named_gpio(node,
				"qcom,wpc-dock-detect-gpio", 0);
	}

	/* parse the dc power supply configuration */
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}

	if (chip->dc_psy_type != -EINVAL) {
		rc = of_property_read_u32(node, "qcom,dc-psy-ma",
							&chip->dc_psy_ma);
		if (rc < 0) {
			dev_err(chip->dev,
					"no mA current for dc rc = %d\n", rc);
			return rc;
		}

		if (chip->dc_psy_ma < DC_MA_MIN
				|| chip->dc_psy_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n", chip->dc_psy_ma);
			return -EINVAL;
		}
	}

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
						&chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = SMB348_FAST_CHG_MAX_MA;

#ifdef FEATURE_THERMAL_MITIGATION_ALGO
	rc = of_property_read_u32(node, "qcom,fastchg-current-mitigation-ma",
						&chip->fastchg_current_mitigation_ma);
	if (rc)
		chip->fastchg_current_mitigation_ma = SMB348_FAST_CHG_MIN_MA;
#endif /* FEATURE_THERMAL_MITIGATION_ALGO */

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0) {
		chip->vfloat_mv = -EINVAL;
		pr_err("float-voltage-mv property missing, exit\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc < 0)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
					"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,cold-bat-decidegc",
						&chip->cold_bat_decidegc);
	if (rc < 0)
		chip->cold_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,hot-bat-decidegc",
						&chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,warm-bat-decidegc",
						&chip->warm_bat_decidegc);

	rc |= of_property_read_u32(node, "qcom,cool-bat-decidegc",
						&chip->cool_bat_decidegc);

	if (!rc) {
		rc = of_property_read_u32(node, "qcom,cool-bat-mv",
						&chip->cool_bat_mv);

		rc |= of_property_read_u32(node, "qcom,warm-bat-mv",
						&chip->warm_bat_mv);

		rc |= of_property_read_u32(node, "qcom,cool-bat-ma",
						&chip->cool_bat_ma);

		rc |= of_property_read_u32(node, "qcom,warm-bat-ma",
						&chip->warm_bat_ma);
		if (rc)
			chip->jeita_supported = false;
		else
			chip->jeita_supported = true;
	}

	pr_debug("jeita_supported = %d", chip->jeita_supported);

	rc = of_property_read_u32(node, "qcom,bat-present-decidegc",
						&batt_present_degree_negative);
	if (rc < 0)
		chip->bat_present_decidegc = -EINVAL;
	else
		chip->bat_present_decidegc = -batt_present_degree_negative;

	if (of_get_property(node, "qcom,vcc-i2c-supply", NULL)) {
		chip->vcc_i2c = devm_regulator_get(chip->dev, "vcc-i2c");
		if (IS_ERR(chip->vcc_i2c)) {
			dev_err(chip->dev,
				"%s: Failed to get vcc_i2c regulator\n",
								__func__);
			return PTR_ERR(chip->vcc_i2c);
		}
	}

	chip->skip_usb_suspend_for_fake_battery = of_property_read_bool(node,
				"qcom,skip-usb-suspend-for-fake-battery");
	if (!chip->skip_usb_suspend_for_fake_battery) {
		if (!chip->vadc_dev) {
			dev_err(chip->dev,
				"VADC device not present with usb suspend on fake battery\n");
			return -EINVAL;
		}

		rc = smb_parse_batt_id(chip);
		if (rc) {
			dev_err(chip->dev,
				"failed to read batt-id rc=%d\n", rc);
			return rc;
		}
	}

	pr_debug("inhibit-disabled = %d, recharge-disabled = %d, recharge-mv = %d,",
		chip->inhibit_disabled, chip->recharge_disabled,
						chip->recharge_mv);
	pr_debug("vfloat-mv = %d, iterm-disabled = %d,",
			chip->vfloat_mv, chip->iterm_disabled);
	pr_debug("fastchg-current = %d, charging-disabled = %d,",
			chip->fastchg_current_max_ma,
					chip->charging_disabled);
	pr_debug("disable-apsd = %d bms = %s cold-bat-degree = %d,",
		chip->disable_apsd, chip->bms_psy_name,
					chip->cold_bat_decidegc);
	pr_debug("hot-bat-degree = %d, bat-present-decidegc = %d\n",
		chip->hot_bat_decidegc, chip->bat_present_decidegc);

	chip->prop_batt_capacity = SMB_MAX_SOC; // make sure recharging design can only take effect after reading capacity from gauge.
	return 0;
}

static int determine_initial_state(struct smb348_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb348_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}

	rc = smb348_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;

	rc = smb348_read_reg(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

	/* For current design, can ignore this */
	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb348_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(chip, 1);
	} else {
		chg_uv(chip, 0);
		apsd_complete(chip, 1);
	}

	return 0;

fail_init_status:
	dev_err(chip->dev, "Couldn't determine initial status\n");
	return rc;
}

#if defined(CONFIG_DEBUG_FS)
static void smb348_debugfs_init(struct smb348_charger *chip)
{
	int rc;
	chip->debug_root = debugfs_create_dir("smb348", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cmd debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("force_irq",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &force_irq_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create force_irq debug file rc =%d\n",
				rc);
		}

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg irq_count file rc = %d\n",
				rc);
		}

	}
}
#else
static void smb348_debugfs_init(struct smb348_charger *chip)
{
}
#endif

#define SMB_I2C_VTG_MIN_UV 1800000
#define SMB_I2C_VTG_MAX_UV 1800000
static int smb348_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc, irq;
	struct smb348_charger *chip;
	struct power_supply *usb_psy;
	u8 reg = 0;
#if 0
	char *str_loc = NULL;  /* Avery added for turn off WPC charger in recovery mode */
	char *firstboot_str_loc = NULL;  /* Alan added for turn off WPC charger in first boot */
#endif

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;

	mutex_init(&chip->wpc_chg_complete);
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->path_suspend_lock);

	/* Quanta { 3 mins delay for WPC charge */
	INIT_DELAYED_WORK(&chip->wpc_resume_chg_wq, smb348_wpc_resume_to_charge);
	/* } Quanta */
	INIT_WORK(&chip->wpc_dock_wq, smb348_wpc_dock_work);

	if (of_find_property(chip->dev->of_node, "qcom,chg-vadc", NULL)) {
		/* early for VADC get, defer probe if needed */
		chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
		if (IS_ERR(chip->vadc_dev)) {
			rc = PTR_ERR(chip->vadc_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("vadc property configured incorrectly\n");
			goto destroy_mutex;
		}
	}

	chip->chip_hw_id = smb348_check_hw_id();
	pr_err("%s HW_ID: %d\n", __func__, chip->chip_hw_id);

	rc = smb_parse_dt(chip);
	if (rc) {
		dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", rc);
		goto destroy_mutex;
	}

#if 0 /* No need to pull up regulator manually */
	/* i2c pull up regulator configuration */
	if (chip->vcc_i2c) {
		if (regulator_count_voltages(chip->vcc_i2c) > 0) {
			rc = regulator_set_voltage(chip->vcc_i2c,
				SMB_I2C_VTG_MIN_UV, SMB_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
				"regulator vcc_i2c set failed, rc = %d\n",
								rc);
				return rc;
			}
		}

		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_i2c enable failed rc = %d\n",
									rc);
			goto err_set_vtg_i2c;
		}
	}
#endif

	/* probe the device to check if its actually connected */
	rc = smb348_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB348, device absent, rc = %d\n", rc);
		goto destroy_mutex;
	}

	/* using adc_tm for implementing pmic therm */
	if (chip->using_pmic_therm) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("adc_tm property missing\n");
			goto destroy_mutex;
		}
	}

	i2c_set_clientdata(client, chip);

	if (chip->dc_psy_type != -EINVAL) {
		chip->dc_psy.name		= "dc";
		chip->dc_psy.type		= chip->dc_psy_type;
		chip->dc_psy.get_property	= smb348_dc_get_property;
		chip->dc_psy.properties		= smb348_dc_properties;
		chip->dc_psy.num_properties = ARRAY_SIZE(smb348_dc_properties);

		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			dev_err(&client->dev,
				"Unable to register dc_psy rc = %d\n", rc);
			goto destroy_mutex;
		}
	}

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb348_battery_get_property;
	chip->batt_psy.set_property	= smb348_battery_set_property;
	chip->batt_psy.property_is_writeable =
					smb348_batt_property_is_writeable;
	chip->batt_psy.properties	= smb348_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smb348_battery_properties);
	chip->batt_psy.external_power_changed = smb348_external_power_changed;
	chip->batt_psy.supplied_to = pm_batt_supplied_to;
	chip->batt_psy.num_supplicants = ARRAY_SIZE(pm_batt_supplied_to);

	chip->resume_completed = true;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register batt psy rc = %d\n",
				rc);
		goto fail_dc_psy_unregister;
	}

	dump_regs(chip);

	chip->dock_sdev.name = "dock";

	rc = switch_dev_register(&chip->dock_sdev);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register dock switch rc = %d\n",
				rc);
		goto fail_psy_unregister;
	}

#if 0 /* No OTG regulater*/
	rc = smb348_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb348 ragulator rc=%d\n", rc);
		goto fail_regulator_register;
	}
#endif
	rc = smb348_hw_init(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't intialize hardware rc=%d\n", rc);
		goto fail_switch_dev_unregister;
	}

	if (gpio_is_valid(chip->chg_end_gpio)) {
		rc = gpio_request(chip->chg_end_gpio, "smb348_chg_end");
		if (rc) {
			dev_err(&client->dev,
				"Invalid CHG_END(%d) gpio_request, rc = %d\n",
				chip->chg_end_gpio, rc);
			goto fail_switch_dev_unregister;
		}

		rc = gpio_direction_output(chip->chg_end_gpio, 0);
		if (rc) {
			dev_err(&client->dev,
				"Invalid CHG_END(%d) gpio_direction_output, rc = %d\n",
				chip->chg_end_gpio, rc);
			goto fail_chg_end_gpio;
		}
	}

	if (gpio_is_valid(chip->wpc_stat_gpio)) {
		rc = request_threaded_irq(gpio_to_irq(chip->wpc_stat_gpio), smb348_wpc_state_handler, NULL,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "smb348_wpc_stat", chip);
		if (rc) {
			dev_err(&client->dev,
				"Invalid WPC_STAT(%d) request irq failed, rc = %d\n",
				chip->wpc_stat_gpio, rc);
			goto fail_wpc_stat_gpio;
		}
		chip->wpc_state = !gpio_get_value(chip->wpc_stat_gpio);  // state pin: low,  power transfer is established.
		dev_err(&client->dev, "WPC status=%d\n", chip->wpc_state);
	}

	if (chip->chip_hw_id >= HW_VER_K) {
		if (gpio_is_valid(chip->wpc_dock_detect_gpio)) {
			rc = request_threaded_irq(gpio_to_irq(chip->wpc_dock_detect_gpio), smb348_wpc_dock_handler, NULL,
						IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "smb348_wpc_dock_detect", chip);
			if (rc) {
				dev_err(&client->dev,
					"Invalid WPC_DETECT(%d) request irq failed, rc = %d\n",
					chip->wpc_dock_detect_gpio, rc);
				goto fail_wpc_dock_detect_gpio;
			}
			pr_err("detect_pin=%d, stat_pin=%d\n", !gpio_get_value(chip->wpc_dock_detect_gpio), !gpio_get_value(chip->wpc_stat_gpio));
			chip->wpc_dock_present = !gpio_get_value(chip->wpc_dock_detect_gpio);
			switch_set_state(&chip->dock_sdev, chip->wpc_dock_present);
			power_supply_changed(&chip->dc_psy);
			pr_err("Dock state is %s\n", chip->dock_sdev.state?"PRESENT":"REMOVED");
		}
	}

	if (gpio_is_valid(chip->wpc_enable_gpio)) {
		rc = gpio_request(chip->wpc_enable_gpio, "smb348_wpc_enable");
		if (rc) {
			dev_err(&client->dev,
				"Invalid WPC_ENABLE(%d) gpio_request, rc = %d\n",
				chip->wpc_enable_gpio, rc);
			goto fail_wpc_enable_gpio;
		}

		rc = gpio_direction_output(chip->wpc_enable_gpio, 0);
		if (rc) {
			dev_err(&client->dev,
				"Invalid WPC_ENABLE(%d) gpio_direction_output, rc = %d\n",
				chip->wpc_enable_gpio, rc);
			goto fail_wpc_enable_gpio;
		}
	}

	rc = determine_initial_state(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't determine initial state rc=%d\n", rc);
		goto fail_wpc_enable_gpio;
	}

	/* We will not use it by default */
	if (gpio_is_valid(chip->chg_valid_gpio)) {
		rc = gpio_request(chip->chg_valid_gpio, "smb348_chg_valid");
		if (rc) {
			dev_err(&client->dev,
				"gpio_request for %d failed rc=%d\n",
				chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		irq = gpio_to_irq(chip->chg_valid_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid chg_valid irq = %d\n", irq);
			goto fail_chg_valid_irq;
		}
		rc = devm_request_threaded_irq(&client->dev, irq,
				NULL, smb348_chg_valid_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"smb348_chg_valid_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed request_irq irq=%d, gpio=%d rc=%d\n",
				irq, chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		smb348_chg_valid_handler(irq, chip);
		enable_irq_wake(irq);
	}
	/* Register IRQ */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb348_chg_stat_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"smb348_chg_stat_irq", chip);
		if (rc < 0) {
			dev_err(&client->dev,
				"Request IRQ(%d) failed, rc = %d\n",
				client->irq, rc);
			goto fail_chg_valid_irq;
		}
		enable_irq_wake(client->irq);
	}

	if (chip->using_pmic_therm) {
		if (!chip->jeita_supported) {
			/* add hot/cold temperature monitor */
			chip->adc_param.low_temp = chip->cold_bat_decidegc;
			chip->adc_param.high_temp = chip->hot_bat_decidegc;
		} else {
			chip->adc_param.low_temp = chip->cool_bat_decidegc;
			chip->adc_param.high_temp = chip->warm_bat_decidegc;
		}
		chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
				smb_chg_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		/* update battery missing info in tm_channel_measure*/
		rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
							&chip->adc_param);
		if (rc)
			pr_err("requesting ADC error %d\n", rc);
	}

	/* Temperature Monitor addr: 0Bh, bit7:bit6 */
	if (chip->cold_bat_decidegc == 100)
		rc = smb348_masked_write(chip, TEMP_REG, 0xC0, 0x00);
	else if (chip->cold_bat_decidegc == 50)
		rc = smb348_masked_write(chip, TEMP_REG, 0xC0, 0x40);
	else if (chip->cold_bat_decidegc == 0)
		rc = smb348_masked_write(chip, TEMP_REG, 0xC0, 0x80);
	else if (chip->cold_bat_decidegc == -50)
		rc = smb348_masked_write(chip, TEMP_REG, 0xC0, 0xC0);
	smb348_debugfs_init(chip);

	dump_regs(chip);

#if 0
	/* Avery added for turn off WPC charger in recovery mode { */
	str_loc = strstr(saved_command_line, "gpt");
	if (str_loc != NULL) {
		pr_err("Quanta: device boots into recovery mode, disable wpc charging");
		RECOVERY_WPC_DISABLE_FLAG = 1;
		gpio_set_value(chip->wpc_enable_gpio, 1);
	} else {
		pr_err("Quanta: device boots into normal mode");

		firstboot_str_loc = strstr(saved_command_line, "firstboot");
		if (firstboot_str_loc != NULL) { /* TODO: neeed to check - first bootup after GOTA flash */
			/* Quanta { 3 mins delay for WPC charge */
			WPC_CHG_RESUME_ALLOW_GOTA = 0;
			gpio_set_value(chip->wpc_enable_gpio, 1);
			schedule_delayed_work(&chip->wpc_resume_chg_wq, msecs_to_jiffies(WPC_CHG_RESUME_CHG_MS));
			pr_err("[DDDBG] set timer for 3 mins\n");
			/* } Quanta */
		}
	}
	/* Avery } */
#endif

	dev_info(chip->dev, "SMB348 successfully probed. charger=%d, batt=%d\n",
			chip->chg_present, smb348_get_prop_batt_present(chip));

	pr_err("wpc detect value 2: %d\n", gpio_get_value(chip->wpc_dock_detect_gpio));
	return 0;

fail_chg_valid_irq:
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);
fail_wpc_enable_gpio:
	if (gpio_is_valid(chip->wpc_enable_gpio))
		gpio_free(chip->wpc_enable_gpio);
fail_wpc_dock_detect_gpio:
	if (gpio_is_valid(chip->wpc_dock_detect_gpio))
		gpio_free(chip->wpc_dock_detect_gpio);
fail_wpc_stat_gpio:
	if (gpio_is_valid(chip->wpc_stat_gpio))
		gpio_free(chip->wpc_stat_gpio);

fail_chg_end_gpio:
	if (gpio_is_valid(chip->chg_end_gpio))
		gpio_free(chip->chg_end_gpio);
fail_switch_dev_unregister:
	switch_dev_unregister(&chip->dock_sdev);
fail_psy_unregister:
	power_supply_unregister(&chip->batt_psy);
fail_dc_psy_unregister:
	if (chip->dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->dc_psy);
destroy_mutex:
	mutex_destroy(&chip->wpc_chg_complete);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->read_write_lock);
	mutex_destroy(&chip->path_suspend_lock);

	return rc;
}

static int smb348_charger_remove(struct i2c_client *client)
{
	struct smb348_charger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->batt_psy);
	switch_dev_unregister(&chip->dock_sdev);
	if (gpio_is_valid(chip->wpc_enable_gpio))
		gpio_free(chip->wpc_enable_gpio);

	if (gpio_is_valid(chip->wpc_stat_gpio))
		gpio_free(chip->wpc_stat_gpio);

	if (gpio_is_valid(chip->chg_end_gpio))
		gpio_free(chip->chg_end_gpio);

	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);

	if (chip->chip_hw_id >= HW_VER_K) {
		if (gpio_is_valid(chip->wpc_dock_detect_gpio))
			gpio_free(chip->wpc_dock_detect_gpio);
	}

	if (chip->vcc_i2c)
		regulator_disable(chip->vcc_i2c);

	mutex_destroy(&chip->wpc_chg_complete);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->read_write_lock);
	mutex_destroy(&chip->path_suspend_lock);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static int smb348_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb348_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	for (i = 0; i < 2; i++) {
		rc = smb348_read_reg(chip, FAULT_INT_REG + i,
					&chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't save irq cfg regs rc = %d\n", rc);
	}

	/* enable wake up IRQs */
	rc = smb348_write_reg(chip, FAULT_INT_REG,
			FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_INPUT_UV_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set fault_irq_cfg rc = %d\n", rc);

	rc = smb348_write_reg(chip, STATUS_INT_REG,
			STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT |
			STATUS_INT_CHGING_BIT | STATUS_INT_INOK_BIT |
			STATUS_INT_OTG_DETECT_BIT | STATUS_INT_CHG_INHI_BIT);
	if (rc < 0)
		dev_err(chip->dev,
			"Couldn't set status_irq_cfg rc = %d\n", rc);

	mutex_lock(&chip->irq_complete);
	if (chip->vcc_i2c) {
		rc = regulator_disable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c disable failed rc=%d\n", rc);
			mutex_unlock(&chip->irq_complete);
			return rc;
		}
	}

	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);
	return 0;
}

static int smb348_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb348_charger *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb348_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb348_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	if (chip->vcc_i2c) {
		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}
	/* Restore IRQ config */
	for (i = 0; i < 2; i++) {
		rc = smb348_write_reg(chip, FAULT_INT_REG + i,
					chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't restore irq cfg regs rc=%d\n", rc);
	}

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	mutex_unlock(&chip->irq_complete);
	if (chip->irq_waiting) {
		smb348_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	}
	return 0;
}

static const struct dev_pm_ops smb348_pm_ops = {
	.suspend	= smb348_suspend,
	.suspend_noirq	= smb348_suspend_noirq,
	.resume		= smb348_resume,
};

static struct of_device_id smb348_match_table[] = {
	{ .compatible = "qcom,smb348-charger",},
	{ },
};

static const struct i2c_device_id smb348_charger_id[] = {
	{"smb348-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb348_charger_id);

static struct i2c_driver smb348_charger_driver = {
	.driver		= {
		.name		= "smb348-charger",
		.owner		= THIS_MODULE,
		.of_match_table = smb348_match_table,
		.pm		= &smb348_pm_ops,
	},
	.probe		= smb348_charger_probe,
	.remove		= smb348_charger_remove,
	.id_table	= smb348_charger_id,
};

module_i2c_driver(smb348_charger_driver);

MODULE_DESCRIPTION("SMB348 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb348-charger");

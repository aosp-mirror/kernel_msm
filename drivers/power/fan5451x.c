/*
 * External charger driver for Fan5451X
 * 3.2A Dual Input, Switch Mode Charger
 *
 * Copyright (C) 2016 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define pr_fmt(fmt)	"FC_CHG: %s: " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>

#include <linux/qpnp/qpnp-adc.h>
#include <linux/power/fan5451x.h>
#include <linux/wakelock.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#define FAN5451x_DEV_NAME "fan5451x"
#define RECHG_CHECK_DELAY_MS      30
#define RECHG_CHECK_DELAY_FAST_MS 10

static unsigned int att_addr = REG_IOCHRG;

struct fan5451x_chip {
	struct i2c_client *client;
	struct power_supply batt_psy;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply *wlc_psy;
	struct qpnp_vadc_chip *vadc_dev;

	int stat_gpio;
	int disable_gpio;
	/* charger param */
	int vbatmin;
	int vfloat;
	int ibus;
	int iin;
	int iochrg;
	int iterm;
	int prechg;
	int topoff_enable;
	int vbusovp;
	int vbuslim;
	int vinovp;
	int vinlim;
	int fctmr;
	/* charger status */
	int health;
	int enable;
	int usb_present;
	int wlc_present;
	bool eoc;
	/* battery temp scenario */
	int prev_ibus_ma;
	unsigned int set_ibat_ma;
	unsigned int ext_set_ibat_ma;
	unsigned int set_vddmax_mv;
	unsigned int ext_set_vddmax_mv;
	unsigned int ext_batt_health;
	/* step charging */
	struct delayed_work vbat_measure_work;
	struct wake_lock chg_wake_lock;
	int ibat_offset_ma;
	int step_dwn_offset_ma;
	unsigned int step_dwn_thr_mv;
	/* debugfs */
	struct dentry *dent;
	int fake_capacity;
	/* wlc fake online */
	int wlc_fake_online;
	struct delayed_work rechg_check_work;
	bool thermal_chg_stop;
};

static void fan5451x_wlc_fake_online(struct fan5451x_chip *chip, int set);

static int fan5451x_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		pr_err("err read [0x%02x] ret(%d)\n", reg, ret);

	pr_debug("read [0x%02x]=%02x\n", reg, ret);

	return ret;
}

static int fan5451x_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		pr_err("err write [0x%02x]=0x%02x ret(%d)\n", reg, val, ret);

	pr_debug("write [0x%02x]=0x%02x\n", reg, val);

	return ret;
}

static int fan5451x_update_reg(struct i2c_client *client,
		u8 reg, u8 val, u8 mask)
{
	int ret;
	u8 reg_val;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		pr_err("err read [0x%02x]=0x%02x ret(%d)\n",
				reg, val & mask, ret);
		return ret;
	}

	reg_val = ret & 0xff;
	reg_val &= ~mask;
	reg_val |= val & mask;

	ret = i2c_smbus_write_byte_data(client, reg, reg_val);
	if (ret < 0) {
		pr_err("err write [0x%02x]=0x%02x ret(%d)\n",
				reg, val & mask, ret);
		return ret;
	}

	pr_debug("update [0x%02x]=0x%02x ret(%d)\n",
				reg, val & mask, ret);

	return ret;
}

static int fan5451x_usb_chg_plugged_in(struct fan5451x_chip *chip)
{
	int ret;

	ret = fan5451x_read_reg(chip->client, REG_STAT0);
	if (ret < 0)
		return 0;

	return (ret & STAT0_VBUSPWR) ? 1 : 0;
}

static int fan5451x_wlc_chg_plugged_in(struct fan5451x_chip *chip)
{
	int ret;

	ret = fan5451x_read_reg(chip->client, REG_STAT0);
	if (ret < 0)
		return 0;

	return (ret & STAT0_VINPWR) ? 1 : 0;
}

static int fan5451x_enable(struct fan5451x_chip *chip, int enable)
{
	if (chip->enable ^ enable) {
		chip->enable = enable;
		if (enable) {
			gpio_direction_output(chip->disable_gpio, 0);
			fan5451x_update_reg(chip->client,
					REG_CON2, 0, CON2_HZMOD);
		} else {
			gpio_direction_output(chip->disable_gpio, 1);
			fan5451x_update_reg(chip->client,
					REG_CON2, CON2_HZMOD, CON2_HZMOD);
		}
	}

	return 0;
}

#define VFLOAT_MIN_MV 3300
#define VFLOAT_MAX_MV 4720
#define VFLOAT_STEP_MV 10
static int fan5451x_set_vfloat(struct fan5451x_chip *chip, int mv)
{
	u8 reg_val;

	if (mv < VFLOAT_MIN_MV)
		mv = VFLOAT_MIN_MV;
	else if (mv > VFLOAT_MAX_MV)
		mv = VFLOAT_MAX_MV;

	reg_val = (mv - VFLOAT_MIN_MV) / VFLOAT_STEP_MV;
	chip->vfloat = reg_val * VFLOAT_STEP_MV + VFLOAT_MIN_MV;
	return fan5451x_write_reg(chip->client, REG_VFLOAT, reg_val);
}

static int fan5451x_set_appropriate_vddmax(struct fan5451x_chip *chip)
{
	int ret;

	if (chip->ext_set_vddmax_mv)
		ret = fan5451x_set_vfloat(chip, chip->ext_set_vddmax_mv);
	else
		ret = fan5451x_set_vfloat(chip, chip->set_vddmax_mv);
	if (ret)
		pr_err("Failed to set appropriate vddmax ret=%d\n", ret);

	return ret;
}

#define IOCHRG_MIN_MA 200
#define IOCHRG_MAX_MA 3200
#define IOCHRG_STEP_MA 50
static int fan5451x_set_iochrg(struct fan5451x_chip *chip, int ma)
{
	u8 reg_val;

	if (ma < IOCHRG_MIN_MA)
		ma = IOCHRG_MIN_MA;
	else if (ma > IOCHRG_MAX_MA)
		ma = IOCHRG_MAX_MA;

	reg_val = (ma - IOCHRG_MIN_MA) / IOCHRG_STEP_MA;
	chip->iochrg = reg_val * IOCHRG_STEP_MA + IOCHRG_MIN_MA;

	return fan5451x_write_reg(chip->client, REG_IOCHRG, reg_val);
}

static void fan5451x_set_appropriate_current(struct fan5451x_chip *chip)
{
	static unsigned int prev_chg_current;
	unsigned int chg_current = chip->set_ibat_ma - chip->ibat_offset_ma;

	if (chip->ext_set_ibat_ma)
		chg_current = min(chg_current, chip->ext_set_ibat_ma);

	if (prev_chg_current != chg_current) {
		prev_chg_current = chg_current;
		pr_info("setting charger current %d mA\n", chg_current);
		fan5451x_set_iochrg(chip, chg_current);
	}
}

#define IBUS_MIN_MA 100
#define IBUS_MAX_MA 3000
#define IBUS_STEP_MA 25
static int fan5451x_set_ibus(struct fan5451x_chip *chip, int ma)
{
	u8 reg_val;

	if (ma < IBUS_MIN_MA)
		ma = IBUS_MIN_MA;
	else if (ma > IBUS_MAX_MA)
		ma = IBUS_MAX_MA;

	reg_val = (ma - IBUS_MIN_MA) / IBUS_STEP_MA;
	chip->ibus = reg_val * IBUS_STEP_MA + IBUS_MIN_MA;

	return fan5451x_write_reg(chip->client, REG_IBUS, reg_val);
}

#define IIN_MIN_MA 325
#define IIN_MAX_MA 2000
#define IIN_STEP_MA 25
static int fan5451x_set_iin(struct fan5451x_chip *chip, int ma)
{
	u8 reg_val;

	if (ma < IIN_MIN_MA)
		ma = IIN_MIN_MA;
	else if (ma > IIN_MAX_MA)
		ma = IIN_MAX_MA;

	reg_val = (ma - IIN_MIN_MA) / IIN_STEP_MA;
	chip->iin = reg_val * IIN_STEP_MA + IIN_MIN_MA;

	return fan5451x_write_reg(chip->client, REG_IIN, reg_val);
}

#define VBATMIN_MIN_MA 2700
#define VBATMIN_MAX_MA 3400
#define VBATMIN_STEP_MA 100
static int fan5451x_set_vbatmin(struct fan5451x_chip *chip, int ma)
{
	u8 reg_val;

	if (ma < VBATMIN_MIN_MA)
		ma = VBATMIN_MIN_MA;
	else if (ma > VBATMIN_MAX_MA)
		ma = VBATMIN_MAX_MA;

	reg_val = (ma - VBATMIN_MIN_MA) / VBATMIN_STEP_MA;
	chip->vbatmin = reg_val * VBATMIN_STEP_MA + VBATMIN_MIN_MA;

	return fan5451x_update_reg(chip->client, REG_CON0, reg_val, CON0_VBATMIN);
}

#define ITERM_MIN_MA 25
#define ITERM_MAX_MA 600
#define ITERM_STEP1_MA 25
#define ITERM_STEP2_MA 50
#define ITERM_STEP_SEPARATE_MA 200
#define ITERM_STEP2_BASE_IDX 7
static int fan5451x_set_iterm(struct fan5451x_chip *chip, int ma)
{
	u8 reg_val;

	if (ma < ITERM_MIN_MA)
		ma = ITERM_MIN_MA;
	else if (ma > ITERM_MAX_MA)
		ma = ITERM_MAX_MA;

	if (ma <= ITERM_STEP_SEPARATE_MA) {
		reg_val = (ma - ITERM_MIN_MA) / ITERM_STEP1_MA;
		chip->iterm = reg_val * ITERM_STEP1_MA + ITERM_MIN_MA;
	} else {
		reg_val = (ma - ITERM_STEP_SEPARATE_MA) / ITERM_STEP2_MA;
		chip->iterm =
			reg_val * ITERM_STEP2_MA + ITERM_STEP_SEPARATE_MA;

		reg_val += ITERM_STEP2_BASE_IDX;
	}

	return fan5451x_update_reg(chip->client, REG_IBAT,
			reg_val << IBAT_ITERM_SHIFT, IBAT_ITERM);
}

#define PRECHG_MIN_MA 50
#define PRECHG_MAX_MA 800
#define PRECHG_STEP_MA 50
static int fan5451x_set_prechg(struct fan5451x_chip *chip, int ma)
{
	u8 reg_val;

	if (ma < PRECHG_MIN_MA)
		ma = PRECHG_MIN_MA;
	else if (ma > PRECHG_MAX_MA)
		ma = PRECHG_MAX_MA;

	reg_val = (ma - PRECHG_MIN_MA) / PRECHG_STEP_MA;
	chip->prechg = reg_val * PRECHG_STEP_MA + PRECHG_MIN_MA;

	return fan5451x_update_reg(chip->client, REG_IBAT,
			reg_val, IBAT_PRECHG);
}

struct fan5451x_map {
	int x;
	int y;
};

#define OVD_MIN_MV 6500
#define OVD_MAX_MV 13700
struct fan5451x_map fan5451x_map_ovd[] = {
	/* reg_val, mV */
	{ 0,  6500 },
	{ 1, 10500 },
	{ 2, 13700 }
};

#define UVD_MIN_MV 4240
#define UVD_MAX_MV 8640
struct fan5451x_map fan5451x_map_uvd[] = {
	/* reg_val, mV */
	{ 0x0, 4240 },
	{ 0x1, 4320 },
	{ 0x2, 4400 },
	{ 0x3, 4480 },
	{ 0x4, 4560 },
	{ 0x5, 4640 },
	{ 0x6, 4720 },
	{ 0x7, 4800 },
	{ 0x8, 7632 },
	{ 0x9, 7776 },
	{ 0xa, 7920 },
	{ 0xb, 8064 },
	{ 0xc, 8208 },
	{ 0xd, 8352 },
	{ 0xe, 8496 },
	{ 0xf, 8640 }
};

enum protection_idx {
	VBUS,
	VIN,
	MAX_PROTECTION_IDX
};

static int fan5451x_search_map(struct fan5451x_map *map, int size, int mv)
{
	int i;

	for (i = 0; i < size; i++) {
		if (mv <= map[i].y)
			break;
	}

	return i;
}

static int fan5451x_set_protection(struct fan5451x_chip *chip,
		enum protection_idx id, int ovd, int uvd)
{
	int idx;
	u8 ovd_reg, uvd_reg;
	int ovd_mv, uvd_mv;
	int ret = 0;

	if (id < VBUS || id >= MAX_PROTECTION_IDX) {
		pr_err("undefined protection id\n");
		return -EINVAL;
	}

	if (ovd < OVD_MIN_MV)
		ovd = OVD_MIN_MV;
	else if (ovd > OVD_MAX_MV)
		ovd = OVD_MAX_MV;

	if (uvd < UVD_MIN_MV)
		uvd = UVD_MIN_MV;
	else if (uvd > UVD_MAX_MV)
		uvd = UVD_MAX_MV;

	/* Search OVD */
	idx = fan5451x_search_map(fan5451x_map_ovd,
			ARRAY_SIZE(fan5451x_map_ovd), ovd);
	ovd_reg = fan5451x_map_ovd[idx].x;
	ovd_mv = fan5451x_map_ovd[idx].y;

	/* Search UVD */
	idx = fan5451x_search_map(fan5451x_map_uvd,
			ARRAY_SIZE(fan5451x_map_uvd), uvd);
	uvd_reg = fan5451x_map_uvd[idx].x;
	uvd_mv = fan5451x_map_uvd[idx].y;

	if (id == VBUS) {
		ret = fan5451x_write_reg(chip->client, REG_VBUS,
			(ovd_reg << VBUS_OVP_SHIFT) | (uvd_reg & VBUS_LIM));
		chip->vbusovp = ovd_mv;
		chip->vbuslim = uvd_mv;
	} else if (id == VIN) {
		ret = fan5451x_write_reg(chip->client, REG_VIN,
			(ovd_reg << VIN_OVP_SHIFT) | (uvd_reg & VIN_LIM));
		chip->vinovp = ovd_mv;
		chip->vinlim = uvd_mv;
	}

	return ret;
}

static inline void fan5451x_vbat_measure(struct fan5451x_chip *chip)
{
	if (chip->step_dwn_offset_ma && chip->step_dwn_thr_mv) {
		if (chip->usb_present || chip->wlc_present) {
			if (!wake_lock_active(&chip->chg_wake_lock))
				wake_lock(&chip->chg_wake_lock);
			schedule_delayed_work(&chip->vbat_measure_work, 0);
		} else {
			cancel_delayed_work_sync(&chip->vbat_measure_work);
			chip->ibat_offset_ma = 0;
			fan5451x_set_appropriate_current(chip);
			if (wake_lock_active(&chip->chg_wake_lock))
				wake_unlock(&chip->chg_wake_lock);
		}
	}
}

static irqreturn_t fan5451x_irq_thread(int irq, void *handle)
{
	u8 intr[3];
	u8 statr[3];
	int ret;
	struct fan5451x_chip *chip = (struct fan5451x_chip *)handle;
	int usb_present;
	int wlc_present;
	bool found = false;

	ret = i2c_smbus_read_i2c_block_data(chip->client, REG_INT0, 3, intr);
	if (ret < 0) {
		pr_err("error i2c read REG_INT block\n");
		return ret;
	}

	ret = i2c_smbus_read_i2c_block_data(chip->client, REG_STAT0, 3, statr);
	if (ret < 0) {
		pr_err("error i2c read REG_STAT block\n");
		return ret;
	}

	if (!chip->wlc_fake_online) {
		pr_info("IRQ INT0:0x%02x, INT1:0x%02x, INT2:0x%02x, "
				"ST0:0x%02x, ST1:0x%02x, ST2:0x%02x\n",
				intr[0], intr[1], intr[2],
				statr[0], statr[1], statr[2]);
	}

	if (intr[0] & INT0_VBUSINT) {
		usb_present = fan5451x_usb_chg_plugged_in(chip);

		if (chip->usb_present ^ usb_present) {
			chip->usb_present = usb_present;
			pr_info("usb present %d\n", usb_present);
			power_supply_set_present(chip->usb_psy,
					chip->usb_present);
			found = true;
		}
	}

	if (intr[0] & INT0_VININT) {
		wlc_present = fan5451x_wlc_chg_plugged_in(chip);

		if (chip->wlc_present ^ wlc_present) {
			chip->wlc_present = wlc_present;
			if (chip->wlc_psy && !chip->wlc_fake_online) {
				pr_info("wlc present %d\n", wlc_present);
				power_supply_set_present(chip->wlc_psy,
						chip->wlc_present);
				if (chip->wlc_present && chip->thermal_chg_stop)
					fan5451x_wlc_fake_online(chip, 1);
			}
			found = true;
		}
	}

	if (found)
		fan5451x_vbat_measure(chip);

	if (intr[0] & INT0_CHGEND) {
		pr_info("EOC IRQ\n");
		fan5451x_wlc_fake_online(chip, 1);
		chip->eoc = true;
		schedule_delayed_work(&chip->rechg_check_work,
				round_jiffies_relative(msecs_to_jiffies(
				RECHG_CHECK_DELAY_MS)));
	} else if (intr[1] & INT1_RCHGN)
		pr_info("Rechg IRQ\n");

	if (!chip->wlc_fake_online)
		power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

static ssize_t show_addr_register(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 6, "0x%02x\n", att_addr);
}

static ssize_t store_addr_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int val;
	int error;

	if (!buf || !size)
		return 0;

	error = kstrtouint(buf, 16, &val);
	if (error)
		return error;

	att_addr = val;

	return size;
}
static DEVICE_ATTR(addr_register,
		S_IRUGO | S_IWUSR, show_addr_register, store_addr_register);

static ssize_t show_status_register(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct fan5451x_chip *chip = dev_get_drvdata(dev);

	ret = fan5451x_read_reg(chip->client, att_addr);
	return snprintf(buf, 6, "0x%02x\n", ret);
}

static ssize_t store_status_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct fan5451x_chip *chip = dev_get_drvdata(dev);
	unsigned int val;
	int ret;
	u8 reg_val;

	if (!buf || !size)
		return 0;

	ret = kstrtouint(buf, 16, &val);
	if (ret)
		return ret;

	reg_val = val & 0xFF;

	ret = fan5451x_write_reg(chip->client, att_addr, reg_val);
	if (ret < 0)
		return ret;

	return size;
}
static DEVICE_ATTR(status_register,
		S_IRUGO | S_IWUSR, show_status_register, store_status_register);

static ssize_t show_chip_param(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fan5451x_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, INT_MAX,
		"vbatmin:%d\n"
		"vfloat:%d\n"
		"ibus:%d\n"
		"iin:%d\n"
		"iochrg:%d\n"
		"iterm:%d\n"
		"prechg:%d\n"
		"topoff-enable:%d\n"
		"vbusovp:%d\n"
		"vbuslim:%d\n"
		"vinovp:%d\n"
		"vinlim:%d\n"
		"fctmr:%d\n"
		"set_ibat_ma:%d\n"
		"ext_set_ibat_ma:%d\n"
		"set_vddmax_mv:%d\n"
		"ext_set_vddmax_mv:%d\n"
		"ext_batt_health:%d\n"
		"step_dwn_offset_ma:%d\n"
		"step_dwn_thr_mv:%d\n",
		chip->vbatmin, chip->vfloat, chip->ibus, chip->iin,
		chip->iochrg, chip->iterm, chip->prechg, chip->topoff_enable,
		chip->vbusovp, chip->vbuslim, chip->vinovp, chip->vinlim,
		chip->fctmr, chip->set_ibat_ma, chip->ext_set_ibat_ma,
		chip->set_vddmax_mv, chip->ext_set_vddmax_mv,
		chip->ext_batt_health,
		chip->step_dwn_offset_ma, chip->step_dwn_thr_mv);
}

static DEVICE_ATTR(chip_param, S_IRUGO, show_chip_param, NULL);

static int fan5451x_debugfs_get_fake_capacity(void *data, u64 *val)
{
	struct fan5451x_chip *chip = data;

	*val = (u64)chip->fake_capacity;

	return 0;
}

static int fan5451x_debugfs_set_fake_capacity(void *data, u64 val)
{
	struct fan5451x_chip *chip = data;

	chip->fake_capacity = (int)val;
	chip->fake_capacity = max(chip->fake_capacity, 0);
	chip->fake_capacity = min(chip->fake_capacity, 100);

	power_supply_changed(&chip->batt_psy);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fan5451x_debugfs_fake_capacity_fops,
		fan5451x_debugfs_get_fake_capacity,
		fan5451x_debugfs_set_fake_capacity,
		 "%llu\n");

static void fan5451x_create_debugfs_entries(
		struct fan5451x_chip *chip)
{
	struct dentry *file;

	chip->dent = debugfs_create_dir("fan5451x", NULL);
	if (IS_ERR(chip->dent)) {
		pr_err("fan5451x driver couldn't create debugfs\n");
		return;
	}

	file = debugfs_create_file("fake_capacity", S_IRUSR | S_IWUSR,
			chip->dent, (void *)chip,
			&fan5451x_debugfs_fake_capacity_fops);
	if (IS_ERR(file)) {
		pr_err("fan5451x couldn't create fake_capacity node\n");
		return;
	}
}

static int fan5451x_gpio_init(struct fan5451x_chip *chip)
{
	int ret;

	ret = gpio_request(chip->disable_gpio, "fan5451x_disable_gpio");
	if (ret) {
		pr_err("failed to request disable_gpio ret(%d)\n", ret);
		return ret;
	}

	ret = gpio_request_one(chip->stat_gpio,
			GPIOF_IN, "fan5451x_stat_gpio");
	if (ret) {
		gpio_free(chip->disable_gpio);
		pr_err("failed to request stat_gpio ret(%d)\n", ret);
		return ret;
	}

	return 0;
}

static void fan5451x_initialization(struct fan5451x_chip *chip)
{
	int ret;

	ret = fan5451x_set_vbatmin(chip, chip->vbatmin);

	chip->set_vddmax_mv = chip->vfloat;
	ret |= fan5451x_set_vfloat(chip, chip->vfloat);

	ret |= fan5451x_set_ibus(chip, chip->ibus);
	ret |= fan5451x_set_iin(chip, chip->iin);

	chip->set_ibat_ma = chip->iochrg;
	ret |= fan5451x_set_iochrg(chip, chip->iochrg);

	ret |= fan5451x_set_iterm(chip, chip->iterm);
	ret |= fan5451x_set_prechg(chip, chip->prechg);
	ret |= fan5451x_set_protection(chip,
			VBUS, chip->vbusovp, chip->vbuslim);
	ret |= fan5451x_set_protection(chip,
			VIN, chip->vinovp, chip->vinlim);
	ret |= fan5451x_update_reg(chip->client, REG_CON2,
			!!chip->topoff_enable << CON2_TOEN_SHIFT, CON2_TOEN);
	ret |= fan5451x_update_reg(chip->client,
			REG_TIMER, chip->fctmr, TIMER_FCTMR);

	if (ret)
		pr_err("failed to set initial configuration\n");
}

static int power_supply_get_online(struct power_supply *psy)
{
	union power_supply_propval val;

	if (psy->get_property) {
		psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
		return val.intval;
	}

	return -ENXIO;
}

#define USB_CHG_I_MAX_MIN_100  100
static void fan5451x_batt_external_power_changed(struct power_supply *psy)
{
	struct fan5451x_chip *chip =
		container_of(psy, struct fan5451x_chip, batt_psy);
	union power_supply_propval ret = {0,};
	int current_ma;
	int wlc_present;

	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("bms");

	if (!chip->wlc_psy) {
		chip->wlc_psy = power_supply_get_by_name("wireless");
		if (chip->wlc_psy) {
			chip->wlc_present = fan5451x_wlc_chg_plugged_in(chip);
			power_supply_set_present(chip->wlc_psy, chip->wlc_present);
			pr_info("wlc present %d\n", chip->wlc_present);
		}
	}

	if (chip->wlc_fake_online) {
		wlc_present = power_supply_get_online(chip->wlc_psy);
		if (!wlc_present) {
			pr_info("wlc present 0, fake online off\n");
			fan5451x_wlc_fake_online(chip, 0);
		}
	}

	if (fan5451x_usb_chg_plugged_in(chip)) {
		chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		current_ma = ret.intval / 1000;

		if (current_ma == chip->prev_ibus_ma)
			goto skip_current_config;

		chip->prev_ibus_ma = current_ma;

		/* Disable charger in case of reset or suspend event */
		if (current_ma <= 2)
			fan5451x_set_ibus(chip, USB_CHG_I_MAX_MIN_100);
		else
			fan5451x_set_ibus(chip, current_ma);
	}

skip_current_config:
	power_supply_changed(&chip->batt_psy);
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
};

static char *pm_batt_supplied_to[] = {
	"bms",
};

static int fan5451x_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int get_prop_batt_status(struct fan5451x_chip *chip)
{
	u8 stat[2];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(chip->client, REG_STAT0, 2, stat);
	if (ret < 0) {
		pr_err("error to read stat ret=%d\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (!(stat[0] & (STAT0_VINPWR | STAT0_VBUSPWR)))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	if (stat[0] & STAT0_STAT)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (stat[1] & STAT1_CHGCMP)
		return POWER_SUPPLY_STATUS_FULL;

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int get_prop_charge_type(struct fan5451x_chip *chip)
{
	int stat, mon;

	stat = fan5451x_read_reg(chip->client, REG_STAT0);
	if (stat < 0) {
		pr_err("error to read stat0 ret=%d\n", stat);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	mon = fan5451x_read_reg(chip->client, REG_MNT0);
	if (mon < 0) {
		pr_err("error to read mon0 ret=%d\n", mon);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	if (!(stat & (STAT0_VINPWR | STAT0_VBUSPWR)))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (!(stat & STAT0_STAT))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (stat & STAT0_PRE)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else if (mon & MNT0_CV)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
}

static int get_prop_batt_present(struct fan5451x_chip *chip)
{
	int stat;

	stat = fan5451x_read_reg(chip->client, REG_STAT0);
	if (stat < 0) {
		pr_err("error to read stat0 ret=%d\n", stat);
		return 1;
	}

	return (stat & STAT0_NOBAT) ? 0 : 1;
}

static int get_prop_battery_voltage_now(struct fan5451x_chip *chip)
{
	int ret = 0;
	struct qpnp_vadc_result results;

	ret = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (ret) {
		pr_err("Unable to read vbat ret=%d\n", ret);
		return 0;
	}

	return results.physical;
}

#define DEFAULT_TEMP		250
static int get_prop_batt_temp(struct fan5451x_chip *chip)
{
	int ret = 0;
	struct qpnp_vadc_result results;

	ret = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (ret) {
		pr_debug("Unable to read batt temperature ret=%d\n", ret);
		return DEFAULT_TEMP;
	}

	pr_debug("get_bat_temp %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

#define BATT_OVERHEAT_TEMP 550
#define BATT_COLD_TEMP -100
static int get_prop_batt_health(struct fan5451x_chip *chip)
{
	int batt_temp;

	if (chip->ext_batt_health)
		return chip->ext_batt_health;

	batt_temp = get_prop_batt_temp(chip);
	if (batt_temp >= BATT_OVERHEAT_TEMP)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (batt_temp <= BATT_COLD_TEMP)
		return POWER_SUPPLY_HEALTH_COLD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

#define DEFAULT_CAPACITY	50
static int get_prop_capacity(struct fan5451x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->fake_capacity)
		return chip->fake_capacity;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);

		return ret.intval;
	}

	pr_debug("No BMS supply registered return %d\n",
						DEFAULT_CAPACITY);

	return DEFAULT_CAPACITY;
}

static int get_prop_current_now(struct fan5451x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	}

	pr_debug("No BMS supply registered return 0\n");

	return 0;
}

static int get_prop_current_avg(struct fan5451x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_AVG, &ret);
		return ret.intval;
	}

	pr_debug("No BMS supply registered return 0\n");

	return 0;
}

static int get_prop_charge_counter(struct fan5451x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CHARGE_COUNTER, &ret);
		return ret.intval;
	}

	pr_debug("No BMS supply registered return 0\n");

	return 0;
}

static int fan5451x_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct fan5451x_chip *chip =
		container_of(psy, struct fan5451x_chip, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->set_vddmax_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->vbuslim * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_battery_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->ext_set_ibat_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = get_prop_current_avg(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = get_prop_charge_counter(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->enable;
		break;
	default:
		return -EINVAL;
	}

	return 0;
};

static int fan5451x_batt_power_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct fan5451x_chip *chip =
		container_of(psy, struct fan5451x_chip, batt_psy);
	int enable;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		chip->ext_batt_health = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		/* In eoc case, TX does not turn on until below Rechg voltage */
		enable = !!val->intval;
		chip->thermal_chg_stop = !enable;
		if (chip->eoc)
			pr_info("EOC!! skip charging_enable to %d.\n", enable);
		else
			fan5451x_wlc_fake_online(chip, !enable);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		chip->ext_set_vddmax_mv = val->intval / 1000;
		fan5451x_set_appropriate_vddmax(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		chip->ext_set_ibat_ma = val->intval / 1000;
		fan5451x_set_appropriate_current(chip);
		break;
	default:
		return -EINVAL;
	}

	power_supply_changed(&chip->batt_psy);
	return ret;
};

#define VBAT_MEASURE_DELAY (10 * HZ)
#define STEPUP_OFFSET_MV 100
#define STEPDOWN_OFFSET_MV 100
static void
fan5451x_vbat_measure_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan5451x_chip *chip = container_of(dwork,
				struct fan5451x_chip, vbat_measure_work);
	int batt_volt;

	batt_volt = get_prop_battery_voltage_now(chip);
	if (batt_volt > (chip->step_dwn_thr_mv + STEPDOWN_OFFSET_MV) * 1000)
		chip->ibat_offset_ma = chip->step_dwn_offset_ma;
	else if (batt_volt < (chip->step_dwn_thr_mv - STEPUP_OFFSET_MV) * 1000)
		chip->ibat_offset_ma = 0;

	fan5451x_set_appropriate_current(chip);

	schedule_delayed_work(&chip->vbat_measure_work, VBAT_MEASURE_DELAY);
}

static void fan5451x_wlc_fake_online(struct fan5451x_chip *chip, int set)
{
	static int prev_state;

	if (prev_state == set)
		return;

	prev_state = set;

	fan5451x_enable(chip, !set);

	if (!chip->wlc_psy)
		return;

	pr_debug("fake wlc present: %d\n", set);

	if (set) {
		/* fake state */
		chip->wlc_fake_online = 1;
		power_supply_set_charging_enabled(chip->wlc_psy, 0); //WLC TX off
		disable_irq_wake(chip->client->irq);
	} else {
		/* normal state */
		enable_irq_wake(chip->client->irq);
		power_supply_set_charging_enabled(chip->wlc_psy, 1); //WLC TX on
		chip->wlc_fake_online = 0;
	}
}

#define RECHG_VOLTAGE_OFFSET_MV 150
#define RECHG_CHECK_CNT 3
static void fan5451x_rechg_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan5451x_chip *chip = container_of(dwork,
				struct fan5451x_chip, rechg_check_work);
	static int rechg_cnt;
	unsigned long delay;
	int volt;

	volt = get_prop_battery_voltage_now(chip);
	if (volt < (chip->vfloat - RECHG_VOLTAGE_OFFSET_MV) * 1000) {
		rechg_cnt++;
		pr_debug("Rechg check cnt(%d/3) volt(%d)\n", rechg_cnt, volt);
		delay = RECHG_CHECK_DELAY_FAST_MS;
	} else {
		rechg_cnt = 0;
		delay = RECHG_CHECK_DELAY_MS;
	}

	if (rechg_cnt > RECHG_CHECK_CNT) {
		pr_info("Rechg occur!! volt(%d)\n", volt);
		chip->eoc = false;
		fan5451x_wlc_fake_online(chip, 0);
		return;
	}

	schedule_delayed_work(&chip->rechg_check_work,
			round_jiffies_relative(msecs_to_jiffies(delay)));
}

#define OF_PROP_READ(chip, prop, dt_property, retval, optional)	\
do { \
	if (retval) \
		break; \
\
	retval = of_property_read_u32(chip->client->dev.of_node, \
					dt_property, &chip->prop); \
\
	if (optional && (retval == -EINVAL)) \
		retval = 0; \
	else if (retval) \
		pr_err("Error reading " #dt_property \
				" property ret = %d\n", retval); \
} while (0)

static int fan5451x_parse_dt(struct fan5451x_chip *chip)
{
	struct device_node *np = chip->client->dev.of_node;
	int ret;

	ret = of_get_named_gpio_flags(np, "stat-gpio", 0, NULL);
	if (ret < 0) {
		pr_err("failed to read stat-gpio\n");
		return ret;
	}
	chip->stat_gpio = ret;

	ret = of_get_named_gpio_flags(np, "disable-gpio", 0, NULL);
	if (ret < 0) {
		pr_err("failed to read disable-gpio\n");
		return ret;
	}
	chip->disable_gpio = ret;

	ret = 0;
	OF_PROP_READ(chip, vbatmin, "vbatmin", ret, 0);
	OF_PROP_READ(chip, vfloat, "vfloat", ret, 0);
	OF_PROP_READ(chip, ibus, "ibus", ret, 0);
	OF_PROP_READ(chip, iin, "iin", ret, 0);
	OF_PROP_READ(chip, iochrg, "iochrg", ret, 0);
	OF_PROP_READ(chip, iterm, "iterm", ret, 0);
	OF_PROP_READ(chip, prechg, "prechg", ret, 0);
	OF_PROP_READ(chip, prechg, "prechg", ret, 0);
	OF_PROP_READ(chip, topoff_enable, "topoff-enable", ret, 0);
	OF_PROP_READ(chip, vbusovp, "vbusovp", ret, 0);
	OF_PROP_READ(chip, vbuslim, "vbuslim", ret, 0);
	OF_PROP_READ(chip, vinovp, "vinovp", ret, 0);
	OF_PROP_READ(chip, vinlim, "vinlim", ret, 0);
	OF_PROP_READ(chip, fctmr, "fctmr", ret, 0);
	OF_PROP_READ(chip, step_dwn_offset_ma, "step-dwn-offset-ma", ret, 1);
	OF_PROP_READ(chip, step_dwn_thr_mv, "step-dwn-thr-mv", ret, 1);

	return ret;
}

static int fan5451x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct fan5451x_chip *chip;
	struct power_supply *usb_psy;
	struct qpnp_vadc_chip *vadc_dev;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("err not support i2c function\n");
		return -EIO;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	vadc_dev = qpnp_get_vadc(&client->dev, "chg");
	if (IS_ERR(vadc_dev)) {
		ret = PTR_ERR(vadc_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("vadc property missing\n");
		else
			pr_err("vadc not found deferring probe\n");
		return ret;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->usb_psy = usb_psy;
	chip->vadc_dev = vadc_dev;
	i2c_set_clientdata(client, chip);

	if (client->dev.of_node) {
		ret = fan5451x_parse_dt(chip);
		if (ret) {
			pr_err("Failed to parse dt\n");
			goto err_chip_clear;
		}
	}

	ret = fan5451x_gpio_init(chip);
	if (ret)
		goto err_chip_clear;

	fan5451x_initialization(chip);

	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = msm_batt_power_props;
	chip->batt_psy.num_properties =
		ARRAY_SIZE(msm_batt_power_props);
	chip->batt_psy.get_property = fan5451x_batt_power_get_property;
	chip->batt_psy.set_property = fan5451x_batt_power_set_property;
	chip->batt_psy.property_is_writeable =
		fan5451x_batt_property_is_writeable;
	chip->batt_psy.external_power_changed =
		fan5451x_batt_external_power_changed;
	chip->batt_psy.supplied_to = pm_batt_supplied_to;
	chip->batt_psy.num_supplicants =
		ARRAY_SIZE(pm_batt_supplied_to);
	ret = power_supply_register(&chip->client->dev, &chip->batt_psy);
	if (ret < 0) {
		pr_err("batt failed to register ret=%d\n", ret);
		goto err_gpio_free;
	}

	wake_lock_init(&chip->chg_wake_lock,
			WAKE_LOCK_SUSPEND, "fan5451x_chg");

	if (chip->step_dwn_offset_ma && chip->step_dwn_thr_mv) {
		INIT_DELAYED_WORK(&chip->vbat_measure_work,
				fan5451x_vbat_measure_work);
	}

	INIT_DELAYED_WORK(&chip->rechg_check_work,
			fan5451x_rechg_check_work);

	ret = request_threaded_irq(client->irq, NULL, fan5451x_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "fan5451x_irq", chip);
	if (ret) {
		pr_err("failed to reqeust IRQ\n");
		goto err_psy_unreg;
	}
	enable_irq_wake(client->irq);

	/* Set initial state */
	chip->usb_present = fan5451x_usb_chg_plugged_in(chip);
	power_supply_set_present(chip->usb_psy, chip->usb_present);
	chip->wlc_present = fan5451x_wlc_chg_plugged_in(chip);
	fan5451x_enable(chip, 1);
	fan5451x_vbat_measure(chip);

	ret = device_create_file(&client->dev, &dev_attr_addr_register);
	if (ret)
		goto err_sysfs_addr_register;
	ret = device_create_file(&client->dev, &dev_attr_status_register);
	if (ret)
		goto err_sysfs_status_register;
	ret = device_create_file(&client->dev, &dev_attr_chip_param);
	if (ret)
		goto err_sysfs_chip_param;

	fan5451x_create_debugfs_entries(chip);

	pr_info("FAN5451x initialized\n");

	return 0;

err_sysfs_chip_param:
	device_remove_file(&client->dev, &dev_attr_status_register);
err_sysfs_status_register:
	device_remove_file(&client->dev, &dev_attr_addr_register);
err_sysfs_addr_register:
	disable_irq_wake(client->irq);
	free_irq(client->irq, chip);
err_psy_unreg:
	power_supply_unregister(&chip->batt_psy);
	wake_lock_destroy(&chip->chg_wake_lock);
err_gpio_free:
	gpio_free(chip->disable_gpio);
	gpio_free(chip->stat_gpio);
err_chip_clear:
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);

	return ret;
}

static int fan5451x_remove(struct i2c_client *client)
{
	struct fan5451x_chip *chip = i2c_get_clientdata(client);

	debugfs_remove_recursive(chip->dent);

	cancel_delayed_work_sync(&chip->rechg_check_work);
	cancel_delayed_work_sync(&chip->vbat_measure_work);

	device_remove_file(&client->dev, &dev_attr_addr_register);
	device_remove_file(&client->dev, &dev_attr_status_register);
	device_remove_file(&client->dev, &dev_attr_chip_param);

	disable_irq_wake(client->irq);
	free_irq(client->irq, chip);

	power_supply_unregister(&chip->batt_psy);
	wake_lock_destroy(&chip->chg_wake_lock);

	gpio_free(chip->disable_gpio);
	gpio_free(chip->stat_gpio);

	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, chip);

	return 0;
}

#define WLC_TX_ON_LATENCY_MS 3000
/* For Wireless charger detection in SBL1 */
static void fan5451x_shutdown(struct i2c_client *client)
{
	struct fan5451x_chip *chip = i2c_get_clientdata(client);

	if (chip->wlc_fake_online) {
		disable_irq(client->irq);
		fan5451x_wlc_fake_online(chip, 0);
		msleep(WLC_TX_ON_LATENCY_MS);
	}
}

static int fan5451x_pm_prepare(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fan5451x_chip *chip = i2c_get_clientdata(client);

	if (chip->eoc)
		cancel_delayed_work_sync(&chip->rechg_check_work);

	return 0;
}

static void fan5451x_pm_complete(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fan5451x_chip *chip = i2c_get_clientdata(client);

	if (chip->eoc)
		schedule_delayed_work(&chip->rechg_check_work, 0);
}

static const struct dev_pm_ops fan5451x_pm_ops = {
	.prepare = fan5451x_pm_prepare,
	.complete = fan5451x_pm_complete,
};

static const struct of_device_id fan5451x_of_match[] = {
	{ .compatible = "fcs,fan5451x" },
	{ },
};
MODULE_DEVICE_TABLE(of, fan5451x_of_match);

static const struct i2c_device_id fan5451x_id[] = {
	{ FAN5451x_DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fan5451x_id);

static struct i2c_driver fan5451x_i2c_driver = {
	.driver = {
		.name = "fan5451x",
		.of_match_table = of_match_ptr(fan5451x_of_match),
		.pm = &fan5451x_pm_ops,
	},
	.probe = fan5451x_probe,
	.remove = fan5451x_remove,
	.shutdown = fan5451x_shutdown,
	.id_table = fan5451x_id,
};
module_i2c_driver(fan5451x_i2c_driver);

MODULE_AUTHOR("Kinam Kim <kinam119.kim@lge.com>");
MODULE_DESCRIPTION("FAN5451X External Charger");
MODULE_LICENSE("GPL");

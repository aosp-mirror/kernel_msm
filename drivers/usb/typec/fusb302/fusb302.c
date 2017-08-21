/*
 * Copyright 2016-2017 Google, Inc
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
 * Fairchild FUSB302 Type-C Chip Driver
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ipc_logging.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/power/htc_battery.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/usb/typec.h>
#include <linux/usb/usb_controller.h>
#include <linux/usb/usb_typec.h>
#include <linux/workqueue.h>

#include "fusb302_reg.h"
#include "../pd.h"
#include "../tcpm.h"

#define PM_WAKE_DELAY_MS 2000

bool IsPRSwap;
bool PolicyIsDFP;
bool PolicyIsSource;

static void *fusb302_log;
#define fusb302_log(fmt, ...) ipc_log_string(fusb302_log, "%s: " fmt, \
					     __func__, ##__VA_ARGS__)
#define NUM_LOG_PAGES		20

/*
 * When the device is SNK, BC_LVL interrupt is used to monitor cc pins
 * for the current capability offered by the SRC. As FUSB302 chip fires
 * the BC_LVL interrupt on PD signalings, cc lvl should be handled after
 * a delay to avoid measuring on PD activities. The delay is slightly
 * longer than PD_T_PD_DEBPUNCE (10-20ms).
 */
#define T_BC_LVL_DEBOUNCE_DELAY_MS 30

enum toggling_mode {
	TOGGLINE_MODE_OFF,
	TOGGLING_MODE_DRP,
	TOGGLING_MODE_SNK,
	TOGGLING_MODE_SRC,
};

static const char * const toggling_mode_name[] = {
	[TOGGLINE_MODE_OFF]	= "toggling_OFF",
	[TOGGLING_MODE_DRP]	= "toggling_DRP",
	[TOGGLING_MODE_SNK]	= "toggling_SNK",
	[TOGGLING_MODE_SRC]	= "toggling_SRC",
};

enum src_current_status {
	SRC_CURRENT_DEFAULT,
	SRC_CURRENT_MEDIUM,
	SRC_CURRENT_HIGH,
};

static const u8 ra_mda_value[] = {
	[SRC_CURRENT_DEFAULT] = 4,	/* 210mV */
	[SRC_CURRENT_MEDIUM] = 9,	/* 420mV */
	[SRC_CURRENT_HIGH] = 18,	/* 798mV */
};

static const u8 rd_mda_value[] = {
	[SRC_CURRENT_DEFAULT] = 38,	/* 1638mV */
	[SRC_CURRENT_MEDIUM] = 38,	/* 1638mV */
	[SRC_CURRENT_HIGH] = 61,	/* 2604mV */
};

struct fusb302_chip {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct tcpm_port *tcpm_port;
	struct tcpc_dev tcpc_dev;

	struct regulator *vdd;
	struct regulator *switch_vdd;
	struct regulator *vbus;
	struct regulator *vconn;

	struct pinctrl *gpio_pinctrl;
	int gpio_int_n;
	int gpio_int_n_irq;

	struct workqueue_struct *wq;
	struct delayed_work bc_lvl_handler;

	atomic_t pm_suspend;
	atomic_t i2c_busy;

	/* lock for sharing chip states */
	struct mutex lock;

	/* chip status */
	enum toggling_mode toggling_mode;
	enum src_current_status src_current_status;
	bool intr_togdone;
	bool intr_bc_lvl;
	bool intr_comp_chng;

	/* port status */
	bool pull_up;
	bool vconn_on;
	bool vbus_on;
	bool charge_on;
	bool vbus_present;
	enum typec_cc_polarity cc_polarity;
	enum typec_cc_status cc1;
	enum typec_cc_status cc2;

	struct usb_controller *uc;
	struct usb_typec_ctrl *utc;
	struct power_supply *batt_psy;
};

static struct fusb302_chip *__fusb302_chip;

#define FUSB302_RESUME_RETRY 10
#define FUSB302_RESUME_RETRY_SLEEP 50
static int fusb302_i2c_write(struct fusb302_chip *chip,
			     u8 address, u8 data)
{
	int retry_cnt;
	int ret = 0;

	atomic_set(&chip->i2c_busy, 1);
	for (retry_cnt = 0; retry_cnt < FUSB302_RESUME_RETRY; retry_cnt++) {
		if (atomic_read(&chip->pm_suspend)) {
			pr_err("fusb302_i2c: pm suspend, retry %d/%d\n",
			       retry_cnt + 1, FUSB302_RESUME_RETRY);
			msleep(FUSB302_RESUME_RETRY_SLEEP);
		} else {
			break;
		}
	}
	ret = i2c_smbus_write_byte_data(chip->i2c_client, address, data);
	if (ret < 0)
		fusb302_log("cannot write 0x%02x to 0x%02x, ret=%d\n",
			    data, address, ret);
	else
		fusb302_log("0x%02x := 0x%02x\n", address, data);
	atomic_set(&chip->i2c_busy, 0);

	return ret;
}

static int fusb302_i2c_block_write(struct fusb302_chip *chip, u8 address,
				   u8 length, const u8 *data)
{
	int retry_cnt;
	int ret = 0;

	if (length <= 0)
		return ret;
	atomic_set(&chip->i2c_busy, 1);
	for (retry_cnt = 0; retry_cnt < FUSB302_RESUME_RETRY; retry_cnt++) {
		if (atomic_read(&chip->pm_suspend)) {
			pr_err("fusb302_i2c: pm suspend, retry %d/%d\n",
			       retry_cnt + 1, FUSB302_RESUME_RETRY);
			msleep(FUSB302_RESUME_RETRY_SLEEP);
		} else {
			break;
		}
	}
	ret = i2c_smbus_write_i2c_block_data(chip->i2c_client, address,
					     length, data);
	if (ret < 0) {
		fusb302_log("cannot block write 0x%02x, len=%d, ret=%d\n",
			    address, length, ret);
	} else {
		fusb302_log("write %d bytes to 0x%02x\n", length, address);
		while (length > 0) {
			fusb302_log("%02x", *data);
			data++;
			length--;
		}
	}
	atomic_set(&chip->i2c_busy, 0);

	return ret;
}

static int fusb302_i2c_read(struct fusb302_chip *chip,
			    u8 address, u8 *data)
{
	int retry_cnt;
	int ret = 0;

	atomic_set(&chip->i2c_busy, 1);
	for (retry_cnt = 0; retry_cnt < FUSB302_RESUME_RETRY; retry_cnt++) {
		if (atomic_read(&chip->pm_suspend)) {
			pr_err("fusb302_i2c: pm suspend, retry %d/%d\n",
			       retry_cnt + 1, FUSB302_RESUME_RETRY);
			msleep(FUSB302_RESUME_RETRY_SLEEP);
		} else {
			break;
		}
	}
	ret = i2c_smbus_read_byte_data(chip->i2c_client, address);
	*data = (u8)ret;
	if (ret < 0)
		fusb302_log("cannot read %02x, ret=%d\n", address, ret);
	else
		fusb302_log("0x%02x: 0x%02x\n", address, ret);
	atomic_set(&chip->i2c_busy, 0);

	return ret;
}

static int fusb302_i2c_block_read(struct fusb302_chip *chip, u8 address,
				  u8 length, u8 *data)
{
	int retry_cnt;
	int ret = 0;

	if (length <= 0)
		return ret;
	atomic_set(&chip->i2c_busy, 1);
	for (retry_cnt = 0; retry_cnt < FUSB302_RESUME_RETRY; retry_cnt++) {
		if (atomic_read(&chip->pm_suspend)) {
			pr_err("fusb302_i2c: pm suspend, retry %d/%d\n",
			       retry_cnt + 1, FUSB302_RESUME_RETRY);
			msleep(FUSB302_RESUME_RETRY_SLEEP);
		} else {
			break;
		}
	}
	ret = i2c_smbus_read_i2c_block_data(chip->i2c_client, address,
					    length, data);
	if (ret < 0) {
		fusb302_log("cannot block read 0x%02x, len=%d, ret=%d\n",
			    address, length, ret);
		return ret;
	}
	if (ret != length) {
		fusb302_log("only read %d/%d bytes from 0x%02x\n",
			    ret, length, address);
		return -EIO;
	}
	fusb302_log("read %d bytes from 0x%02x\n", length, address);
	while (length > 0) {
		fusb302_log("%02x", *data);
		data++;
		length--;
	}
	atomic_set(&chip->i2c_busy, 0);

	return ret;
}

static int fusb302_i2c_mask_write(struct fusb302_chip *chip, u8 address,
				  u8 mask, u8 value)
{
	int ret = 0;
	u8 data;

	ret = fusb302_i2c_read(chip, address, &data);
	if (ret < 0)
		return ret;
	data &= ~mask;
	data |= value;
	ret = fusb302_i2c_write(chip, address, data);
	if (ret < 0)
		return ret;

	return ret;
}

static int fusb302_i2c_set_bits(struct fusb302_chip *chip, u8 address,
				u8 set_bits)
{
	return fusb302_i2c_mask_write(chip, address, 0x00, set_bits);
}

static int fusb302_i2c_clear_bits(struct fusb302_chip *chip, u8 address,
				  u8 clear_bits)
{
	return fusb302_i2c_mask_write(chip, address, clear_bits, 0x00);
}

static int fusb302_sw_reset(struct fusb302_chip *chip)
{
	int ret = 0;

	ret = fusb302_i2c_write(chip, FUSB_REG_RESET,
				FUSB_REG_RESET_SW_RESET);
	if (ret < 0)
		fusb302_log("cannot sw reset the chip, ret=%d\n", ret);
	else
		fusb302_log("sw reset\n");

	return ret;
}

static int fusb302_enable_tx_auto_retries(struct fusb302_chip *chip)
{
	int ret = 0;

	ret = fusb302_i2c_set_bits(chip, FUSB_REG_CONTROL3,
				   FUSB_REG_CONTROL3_N_RETRIES_3 |
				   FUSB_REG_CONTROL3_AUTO_RETRY);

	return ret;
}

/*
 * initialize interrupt on the chip
 * - unmasked interrupt: VBUS_OK
 */
static int fusb302_init_interrupt(struct fusb302_chip *chip)
{
	int ret = 0;

	ret = fusb302_i2c_write(chip, FUSB_REG_MASK,
				0xFF & ~FUSB_REG_MASK_VBUSOK);
	if (ret < 0)
		return ret;
	ret = fusb302_i2c_write(chip, FUSB_REG_MASKA, 0xFF);
	if (ret < 0)
		return ret;
	ret = fusb302_i2c_write(chip, FUSB_REG_MASKB, 0xFF);
	if (ret < 0)
		return ret;
	ret = fusb302_i2c_clear_bits(chip, FUSB_REG_CONTROL0,
				     FUSB_REG_CONTROL0_INT_MASK);
	if (ret < 0)
		return ret;

	return ret;
}

static int fusb302_set_power_mode(struct fusb302_chip *chip, u8 power_mode)
{
	int ret = 0;

	ret = fusb302_i2c_write(chip, FUSB_REG_POWER, power_mode);

	return ret;
}

static int tcpm_init(struct tcpc_dev *dev)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;
	u8 data;

	ret = fusb302_sw_reset(chip);
	if (ret < 0)
		return ret;
	ret = fusb302_enable_tx_auto_retries(chip);
	if (ret < 0)
		return ret;
	ret = fusb302_init_interrupt(chip);
	if (ret < 0)
		return ret;
	ret = fusb302_set_power_mode(chip, FUSB_REG_POWER_PWR_ALL);
	if (ret < 0)
		return ret;
	ret = fusb302_i2c_read(chip, FUSB_REG_STATUS0, &data);
	if (ret < 0)
		return ret;
	chip->vbus_present = !!(FUSB_REG_STATUS0 & FUSB_REG_STATUS0_VBUSOK);
	ret = fusb302_i2c_read(chip, FUSB_REG_DEVICE_ID, &data);
	if (ret < 0)
		return ret;
	fusb302_log("fusb302 device ID: 0x%02x\n", data);

	return ret;
}

static int tcpm_get_vbus(struct tcpc_dev *dev)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;

	mutex_lock(&chip->lock);
	ret = chip->vbus_present ? 1 : 0;
	mutex_unlock(&chip->lock);

	return ret;
}

static int fusb302_set_cc_pull(struct fusb302_chip *chip,
			       bool pull_up, bool pull_down)
{
	int ret = 0;
	u8 data = 0x00;
	u8 mask = FUSB_REG_SWITCHES0_CC1_PU_EN |
		  FUSB_REG_SWITCHES0_CC2_PU_EN |
		  FUSB_REG_SWITCHES0_CC1_PD_EN |
		  FUSB_REG_SWITCHES0_CC2_PD_EN;

	if (pull_up)
		data |= (chip->cc_polarity == TYPEC_POLARITY_CC1) ?
			FUSB_REG_SWITCHES0_CC1_PU_EN :
			FUSB_REG_SWITCHES0_CC2_PU_EN;
	if (pull_down)
		data |= FUSB_REG_SWITCHES0_CC1_PD_EN |
			FUSB_REG_SWITCHES0_CC2_PD_EN;
	ret = fusb302_i2c_mask_write(chip, FUSB_REG_SWITCHES0,
				     mask, data);
	if (ret < 0)
		return ret;
	chip->pull_up = pull_up;

	return ret;
}

static int fusb302_set_src_current(struct fusb302_chip *chip,
				   enum src_current_status status)
{
	int ret = 0;

	chip->src_current_status = status;
	switch (status) {
	case SRC_CURRENT_DEFAULT:
		ret = fusb302_i2c_mask_write(chip, FUSB_REG_CONTROL0,
					     FUSB_REG_CONTROL0_HOST_CUR_MASK,
					     FUSB_REG_CONTROL0_HOST_CUR_DEF);
		break;
	case SRC_CURRENT_MEDIUM:
		ret = fusb302_i2c_mask_write(chip, FUSB_REG_CONTROL0,
					     FUSB_REG_CONTROL0_HOST_CUR_MASK,
					     FUSB_REG_CONTROL0_HOST_CUR_MED);
		break;
	case SRC_CURRENT_HIGH:
		ret = fusb302_i2c_mask_write(chip, FUSB_REG_CONTROL0,
					     FUSB_REG_CONTROL0_HOST_CUR_MASK,
					     FUSB_REG_CONTROL0_HOST_CUR_HIGH);
		break;
	default:
		break;
	}

	return ret;
}

static int fusb302_set_toggling(struct fusb302_chip *chip,
				enum toggling_mode mode)
{
	int ret = 0;

	/* first disable toggling */
	ret = fusb302_i2c_clear_bits(chip, FUSB_REG_CONTROL2,
				     FUSB_REG_CONTROL2_TOGGLE);
	if (ret < 0)
		return ret;
	/* mask interrupts for SRC or SNK */
	ret = fusb302_i2c_set_bits(chip, FUSB_REG_MASK,
				   FUSB_REG_MASK_BC_LVL |
				   FUSB_REG_MASK_COMP_CHNG);
	if (ret < 0)
		return ret;
	chip->intr_bc_lvl = false;
	chip->intr_comp_chng = false;
	/* configure toggling mode: none/snk/src/drp */
	switch (mode) {
	case TOGGLINE_MODE_OFF:
		ret = fusb302_i2c_mask_write(chip, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_NONE);
		if (ret < 0)
			return ret;
		break;
	case TOGGLING_MODE_SNK:
		ret = fusb302_i2c_mask_write(chip, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_UFP);
		if (ret < 0)
			return ret;
		break;
	case TOGGLING_MODE_SRC:
		ret = fusb302_i2c_mask_write(chip, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_DFP);
		if (ret < 0)
			return ret;
		break;
	case TOGGLING_MODE_DRP:
		ret = fusb302_i2c_mask_write(chip, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_DRP);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}

	if (mode == TOGGLINE_MODE_OFF) {
		/* mask TOGDONE interrupt */
		ret = fusb302_i2c_set_bits(chip, FUSB_REG_MASKA,
					   FUSB_REG_MASKA_TOGDONE);
		if (ret < 0)
			return ret;
		chip->intr_togdone = false;
	} else {
		/* unmask TOGDONE interrupt */
		ret = fusb302_i2c_clear_bits(chip, FUSB_REG_MASKA,
					     FUSB_REG_MASKA_TOGDONE);
		if (ret < 0)
			return ret;
		chip->intr_togdone = true;
		/* start toggling */
		ret = fusb302_i2c_set_bits(chip, FUSB_REG_CONTROL2,
					   FUSB_REG_CONTROL2_TOGGLE);
		if (ret < 0)
			return ret;
		/* during toggling, consider cc as Open */
		chip->cc1 = TYPEC_CC_OPEN;
		chip->cc2 = TYPEC_CC_OPEN;
	}
	chip->toggling_mode = mode;

	return ret;
}

static const char * const typec_cc_status_name[] = {
	[TYPEC_CC_OPEN]		= "Open",
	[TYPEC_CC_RA]		= "Ra",
	[TYPEC_CC_RD]		= "Rd",
	[TYPEC_CC_RP_DEF]	= "Rp-def",
	[TYPEC_CC_RP_1_5]	= "Rp-1.5",
	[TYPEC_CC_RP_3_0]	= "Rp-3.0",
};

static const enum src_current_status cc_src_current[] = {
	[TYPEC_CC_OPEN]		= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RA]		= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RD]		= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RP_DEF]	= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RP_1_5]	= SRC_CURRENT_MEDIUM,
	[TYPEC_CC_RP_3_0]	= SRC_CURRENT_HIGH,
};

static int tcpm_set_cc(struct tcpc_dev *dev, enum typec_cc_status cc)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;
	bool pull_up, pull_down;
	u8 rd_mda;

	mutex_lock(&chip->lock);
	switch (cc) {
	case TYPEC_CC_OPEN:
		pull_up = false;
		pull_down = false;
		break;
	case TYPEC_CC_RD:
		pull_up = false;
		pull_down = true;
		break;
	case TYPEC_CC_RP_DEF:
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		pull_up = true;
		pull_down = false;
		break;
	default:
		fusb302_log("unsupported cc value %s\n",
			    typec_cc_status_name[cc]);
		ret = -EINVAL;
		goto done;
	}
	ret = fusb302_set_toggling(chip, TOGGLINE_MODE_OFF);
	if (ret < 0) {
		fusb302_log("cannot stop toggling, ret=%d\n", ret);
		goto done;
	}
	ret = fusb302_set_cc_pull(chip, pull_up, pull_down);
	if (ret < 0) {
		fusb302_log("cannot set cc pulling up %s, down %s, ret = %d\n",
			    pull_up ? "True" : "False",
			    pull_down ? "True" : "False",
			    ret);
		goto done;
	}
	/* reset the cc status */
	chip->cc1 = TYPEC_CC_OPEN;
	chip->cc2 = TYPEC_CC_OPEN;
	/* report OPEN status back to TCPM when not in PR_SWAP path*/
	if (!IsPRSwap)
		tcpm_cc_change(chip->tcpm_port);
	/* adjust current for SRC */
	if (pull_up) {
		ret = fusb302_set_src_current(chip, cc_src_current[cc]);
		if (ret < 0) {
			fusb302_log("cannot set src current %s, ret=%d\n",
				    typec_cc_status_name[cc], ret);
			goto done;
		}
	}
	/* enable/disable interrupts, BC_LVL for SNK and COMP_CHNG for SRC */
	if (pull_up) {
		rd_mda = rd_mda_value[cc_src_current[cc]];
		ret = fusb302_i2c_write(chip, FUSB_REG_MEASURE, rd_mda);
		if (ret < 0) {
			fusb302_log("cannot set SRC measure value, ret=%d\n",
				    ret);
			goto done;
		}

		/* WAR: Fairchild chip cannot send control packets when
		 * TOGDONE interrupt is enabled. Since, TOGDONE is not
		 * needed during PR_SWAP as CC pin is manupulated, do
		 * enable TOGDONE interrupt to unblock chip on sending
		 * the control messages such as PS_RDY.
		 */
		if (!IsPRSwap) {
			ret = fusb302_set_toggling(chip, TOGGLING_MODE_SRC);
			if (ret < 0) {
				fusb302_log("cannot set toggling to SRC mode,");
				fusb302_log("ret=%d\n", ret);
				goto done;
			}
		}

		ret = fusb302_i2c_mask_write(chip, FUSB_REG_MASK,
					     FUSB_REG_MASK_BC_LVL |
					     FUSB_REG_MASK_COMP_CHNG,
					     FUSB_REG_MASK_BC_LVL);
		if (ret < 0) {
			fusb302_log("cannot set SRC interrupt, ret=%d\n",
				    ret);
			goto done;
		}
		chip->intr_bc_lvl = false;
		chip->intr_comp_chng = true;
	}
	if (pull_down) {
		if (!IsPRSwap) {
			ret = fusb302_set_toggling(chip, TOGGLING_MODE_SNK);
			if (ret < 0) {
				fusb302_log("cannot set toggling to SNK mode,");
				fusb302_log("ret=%d\n", ret);
				goto done;
			}
		}

		ret = fusb302_i2c_mask_write(chip, FUSB_REG_MASK,
					     FUSB_REG_MASK_BC_LVL |
					     FUSB_REG_MASK_COMP_CHNG,
					     FUSB_REG_MASK_COMP_CHNG);
		if (ret < 0) {
			fusb302_log("cannot set SNK interrupt, ret=%d\n",
				    ret);
			goto done;
		}
		chip->intr_bc_lvl = true;
		chip->intr_comp_chng = false;
	}
	fusb302_log("cc := %s\n", typec_cc_status_name[cc]);
done:
	mutex_unlock(&chip->lock);

	return ret;
}

static int tcpm_get_cc(struct tcpc_dev *dev, enum typec_cc_status *cc1,
		       enum typec_cc_status *cc2)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);

	mutex_lock(&chip->lock);
	*cc1 = chip->cc1;
	*cc2 = chip->cc2;
	fusb302_log("cc1=%s, cc2=%s\n", typec_cc_status_name[*cc1],
		    typec_cc_status_name[*cc2]);
	mutex_unlock(&chip->lock);

	return 0;
}

static int tcpm_set_polarity(struct tcpc_dev *dev,
			     enum typec_cc_polarity polarity)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;
	struct pinctrl_state *set_state;

	mutex_lock(&chip->lock);

	fusb302_log("polarity := %d\n", polarity);

	if (!chip->gpio_pinctrl) {
		fusb302_log("gpio_pinctrl is not avalible\n");
		ret = -EFAULT;
		goto done;
	}

	set_state = pinctrl_lookup_state(chip->gpio_pinctrl,
					 polarity ? "usb3_switch_sel_1" :
						    "usb3_switch_sel_0");
	if (IS_ERR(set_state)) {
		ret = PTR_ERR(set_state);
		fusb302_log(
			"cannot get fusb302 gpio_pinctrl usb3_switch_sel_%d state, ret=%d\n",
			polarity,
			ret);
		goto done;
	}

	ret = pinctrl_select_state(chip->gpio_pinctrl, set_state);
	if (ret < 0)
		fusb302_log("cannot select state, ret=%d\n", ret);
done:
	mutex_unlock(&chip->lock);

	return ret;
}

static int tcpm_set_vconn(struct tcpc_dev *dev, bool on)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;
	struct pinctrl_state *set_state;
	u8 switches0_data = 0x00;
	u8 switches0_mask = FUSB_REG_SWITCHES0_VCONN_CC1 |
			    FUSB_REG_SWITCHES0_VCONN_CC2;

	mutex_lock(&chip->lock);
	if (chip->vconn_on == on) {
		fusb302_log("vconn is already %s\n", on ? "On" : "Off");
		goto done;
	}

	if (!chip->vconn) {
		chip->vconn = devm_regulator_get(chip->dev, "V_USB_boost");
		if (IS_ERR(chip->vconn)) {
			fusb302_log("still unable to get vconn regulator\n");
			ret = -ENODEV;
			goto done;
		}
	}

	if (on) {
		ret = regulator_enable(chip->vconn);
		if (ret) {
			fusb302_log("Unable to enable vconn regulator\n");
			goto done;
		}
		switches0_data = (chip->cc_polarity == TYPEC_POLARITY_CC1) ?
				 FUSB_REG_SWITCHES0_VCONN_CC2 :
				 FUSB_REG_SWITCHES0_VCONN_CC1;
	} else {
		ret = regulator_disable(chip->vconn);
		if (ret) {
			fusb302_log("Unable to disable vconn regulator\n");
			goto done;
		}
	}

	if (chip->gpio_pinctrl) {
		set_state = pinctrl_lookup_state(chip->gpio_pinctrl,
						 on ? "vconn_enable" :
						      "vconn_disable");
		if (IS_ERR(set_state)) {
			fusb302_log("cannot get pinctrl vconn_control state\n");
			ret = -ENODEV;
			goto done;
		}
		pinctrl_select_state(chip->gpio_pinctrl, set_state);
	}

	ret = fusb302_i2c_mask_write(chip, FUSB_REG_SWITCHES0,
				     switches0_mask, switches0_data);
	if (ret < 0)
		goto done;
	chip->vconn_on = on;
	fusb302_log("vconn := %s\n", on ? "On" : "Off");
done:
	mutex_unlock(&chip->lock);

	return ret;
}

static int tcpm_set_vbus(struct tcpc_dev *dev, bool on, bool charge)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;

	mutex_lock(&chip->lock);

	if (chip->vbus_on == on) {
		fusb302_log("vbus is already %s\n", on ? "On" : "Off");
	} else {
		if (chip->uc && chip->uc->pd_vbus_ctrl) {
			ret = chip->uc->pd_vbus_ctrl(on, true);
		} else {
			fusb302_log("no pd_vbus_ctrl\n");
			if (!chip->vbus) {
				chip->vbus = devm_regulator_get(chip->dev,
								"vbus");
				if (IS_ERR(chip->vbus)) {
					ret = PTR_ERR(chip->vbus);
					fusb302_log("cannot get vbus, ret=%d\n",
						    ret);
				}
			}

			if (chip->vbus) {
				if (on)
					ret = regulator_enable(chip->vbus);
				else
					ret = regulator_disable(chip->vbus);
			}
		}
		if (ret < 0) {
			fusb302_log("cannot %s vbus regulator, ret=%d\n",
				    on ? "enable" : "disable", ret);
			goto done;
		}
		chip->vbus_on = on;
		fusb302_log("vbus := %s\n", on ? "On" : "Off");
	}
	if (chip->charge_on == charge)
		fusb302_log("charge is already %s\n", charge ? "On" : "Off");
	else
		chip->charge_on = charge;

done:
	mutex_unlock(&chip->lock);

	return ret;
}

enum usb_typec_current {
	USB_TYPEC_CURRENT_NONE = 0,
	USB_TYPEC_CURRENT_DEFAULT,
	USB_TYPEC_CURRENT_1_5_A,
	USB_TYPEC_CURRENT_3_0_A,
};

static int tcpm_set_current_limit(struct tcpc_dev *dev, u32 max_ma, u32 mv)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;
	enum usb_typec_current sink_current, pre_sink_current;

	mutex_lock(&chip->lock);

	fusb302_log("current limit: %d ma, %d mv\n",
		    max_ma, mv);

	if ((mv == 0 || mv == 5000) &&
	    (max_ma == 0 || max_ma == 1500 || max_ma == 3000)) {
		if (max_ma == 0)
			sink_current = USB_TYPEC_CURRENT_DEFAULT;
		else if (max_ma == 1500)
			sink_current = USB_TYPEC_CURRENT_1_5_A;
		else
			sink_current = USB_TYPEC_CURRENT_3_0_A;
		if (chip->utc) {
			pre_sink_current = chip->utc->sink_current;
			chip->utc->sink_current = sink_current;
		}

		if (!chip->batt_psy) {
			chip->batt_psy = power_supply_get_by_name("battery");
			if (IS_ERR(chip->batt_psy)) {
				ret = PTR_ERR(chip->batt_psy);
				fusb302_log(
					"cannot get battery power supply, ret=%d\n",
					ret);
			}
		}

		if (chip->batt_psy && chip->utc &&
		    (sink_current != pre_sink_current)) {
			ret = chip->batt_psy->set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_TYPEC_SINK_CURRENT,
					(const union power_supply_propval *)
							&pre_sink_current);
			if (ret < 0) {
				fusb302_log(
					"cannot set battery sink current, ret=%d\n",
					ret);
			}
		}
	}

	mutex_unlock(&chip->lock);

	return 0;
}

static int fusb302_pd_tx_flush(struct fusb302_chip *chip)
{
	return fusb302_i2c_set_bits(chip, FUSB_REG_CONTROL0,
				    FUSB_REG_CONTROL0_TX_FLUSH);
}

static int fusb302_pd_rx_flush(struct fusb302_chip *chip)
{
	return fusb302_i2c_set_bits(chip, FUSB_REG_CONTROL1,
				    FUSB_REG_CONTROL1_RX_FLUSH);
}

static int fusb302_pd_set_auto_goodcrc(struct fusb302_chip *chip, bool on)
{
	if (on)
		return fusb302_i2c_set_bits(chip, FUSB_REG_SWITCHES1,
					    FUSB_REG_SWITCHES1_AUTO_GCRC);
	return fusb302_i2c_clear_bits(chip, FUSB_REG_SWITCHES1,
					    FUSB_REG_SWITCHES1_AUTO_GCRC);
}

static int fusb302_pd_set_interrupts(struct fusb302_chip *chip, bool on)
{
	int ret = 0;
	u8 mask_interrupts = FUSB_REG_MASK_COLLISION;
	u8 maska_interrupts = FUSB_REG_MASKA_RETRYFAIL |
			      FUSB_REG_MASKA_HARDSENT |
			      FUSB_REG_MASKA_TX_SUCCESS |
			      FUSB_REG_MASKA_HARDRESET;
	u8 maskb_interrupts = FUSB_REG_MASKB_GCRCSENT;

	ret = on ?
		fusb302_i2c_clear_bits(chip, FUSB_REG_MASK, mask_interrupts) :
		fusb302_i2c_set_bits(chip, FUSB_REG_MASK, mask_interrupts);
	if (ret < 0)
		return ret;
	ret = on ?
		fusb302_i2c_clear_bits(chip, FUSB_REG_MASKA, maska_interrupts) :
		fusb302_i2c_set_bits(chip, FUSB_REG_MASKA, maska_interrupts);
	if (ret < 0)
		return ret;
	ret = on ?
		fusb302_i2c_clear_bits(chip, FUSB_REG_MASKB, maskb_interrupts) :
		fusb302_i2c_set_bits(chip, FUSB_REG_MASKB, maskb_interrupts);
	return ret;
}

/*
 * requires chip lock: chip->lock;
 */
#define NOTIFY_SOURCE_WAR_DELAY_TIMESTAMP_MS	6000
static int fusb302_notify_uc_data_role_locked(struct fusb302_chip *chip,
					      enum typec_data_role value)
{
	u64 ts_msec = local_clock()/1000000;

	fusb302_log("notify_uc_data_role of %d\n", value);

	/*
	 * workaround: for some reason, calling notify_attached_source
	 * at an early boot stage (kernel time < ~3 secs) will leads to
	 * the usb to fail to reattach. As a workaround, fail the call
	 * until NOTIFY_SOURCE_WAR_DELAY_TIMESTAMP_MS after kernel start
	 * booting.
	 */
	if (ts_msec < NOTIFY_SOURCE_WAR_DELAY_TIMESTAMP_MS) {
		fusb302_log("WAR: do not notify_attached_source too soon\n");
		return -EAGAIN;
	}

	if (chip->uc != NULL && chip->uc->notify_attached_source != NULL) {
		chip->uc->notify_attached_source(chip->uc, value);
		return 0;
	}

	fusb302_log("notify uc data role error\n");
	return -ENODEV;
}

static int tcpm_set_pd_rx(struct tcpc_dev *dev, bool on)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;

	mutex_lock(&chip->lock);
	ret = fusb302_pd_rx_flush(chip);
	if (ret < 0) {
		fusb302_log("cannot flush pd rx buffer, ret=%d\n", ret);
		goto done;
	}
	ret = fusb302_pd_tx_flush(chip);
	if (ret < 0) {
		fusb302_log("cannot flush pd tx buffer, ret=%d\n", ret);
		goto done;
	}
	ret = fusb302_pd_set_auto_goodcrc(chip, on);
	if (ret < 0) {
		fusb302_log("cannot turn %s auto GCRC, ret=%d\n",
			    on ? "on" : "off", ret);
		goto done;
	}
	ret = fusb302_pd_set_interrupts(chip, on);
	if (ret < 0) {
		fusb302_log("cannot turn %s pd interrupts, ret=%d\n",
			    on ? "on" : "off", ret);
		goto done;
	}
	fusb302_log("pd := %s\n", on ? "on" : "off");
done:
	mutex_unlock(&chip->lock);

	return ret;
}

static const char * const typec_role_name[] = {
	[TYPEC_SINK]		= "Sink",
	[TYPEC_SOURCE]		= "Source",
};

static const char * const typec_data_role_name[] = {
	[TYPEC_DEVICE]		= "Device",
	[TYPEC_HOST]		= "Host",
};

static int tcpm_set_roles(struct tcpc_dev *dev, bool attached,
			  enum typec_role pwr, enum typec_data_role data)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;
	u8 switches1_mask = FUSB_REG_SWITCHES1_POWERROLE |
			    FUSB_REG_SWITCHES1_DATAROLE;
	u8 switches1_data = 0x00;

	if (!attached)
		data = TYPEC_DEVICE;

	mutex_lock(&chip->lock);

	ret = fusb302_notify_uc_data_role_locked(chip, data);
	if (ret < 0)
		goto done;

	if (pwr == TYPEC_SOURCE)
		switches1_data |= FUSB_REG_SWITCHES1_POWERROLE;
	if (data == TYPEC_HOST)
		switches1_data |= FUSB_REG_SWITCHES1_DATAROLE;

	PolicyIsSource = (pwr == TYPEC_SOURCE) ? true : false;
	PolicyIsDFP = (data == TYPEC_HOST) ? true : false;

	ret = fusb302_i2c_mask_write(chip, FUSB_REG_SWITCHES1,
				     switches1_mask, switches1_data);
	if (ret < 0) {
		fusb302_log("unable to set pd header %s, %s, ret=%d\n",
			    typec_role_name[pwr], typec_data_role_name[data],
			    ret);
		goto done;
	}
	fusb302_log("pd header := %s, %s\n", typec_role_name[pwr],
		    typec_data_role_name[data]);
done:
	mutex_unlock(&chip->lock);

	return ret;
}

static int tcpm_start_drp_toggling(struct tcpc_dev *dev,
				   enum typec_cc_status cc)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;

	mutex_lock(&chip->lock);
	ret = fusb302_set_src_current(chip, cc_src_current[cc]);
	if (ret < 0) {
		fusb302_log("unable to set src current %s, ret=%d\n",
			    typec_cc_status_name[cc], ret);
		goto done;
	}
	ret = fusb302_set_toggling(chip, TOGGLING_MODE_DRP);
	if (ret < 0) {
		fusb302_log("unable to start drp toggling, ret=%d\n", ret);
		goto done;
	}
	fusb302_log("start drp toggling\n");
done:
	mutex_unlock(&chip->lock);

	return ret;
}

static int fusb302_pd_send_message(struct fusb302_chip *chip,
				   const struct pd_message *msg)
{
	int ret = 0;
	u8 buf[40];
	u8 pos = 0;
	int len;

	/* SOP tokens */
	buf[pos++] = FUSB302_TKN_SYNC1;
	buf[pos++] = FUSB302_TKN_SYNC1;
	buf[pos++] = FUSB302_TKN_SYNC1;
	buf[pos++] = FUSB302_TKN_SYNC2;

	len = pd_header_cnt(msg->header) * 4;
	/* plug 2 for header */
	len += 2;
	if (len > 0x1F) {
		fusb302_log("PD message too long %d (incl. header)\n", len);
		return -EINVAL;
	}
	/* packsym tells the FUSB302 chip that the next X bytes are payload */
	buf[pos++] = FUSB302_TKN_PACKSYM | (len & 0x1F);
	buf[pos++] = msg->header & 0xFF;
	buf[pos++] = (msg->header >> 8) & 0xFF;

	len -= 2;
	memcpy(&buf[pos], msg->payload, len);
	pos += len;

	/* CRC */
	buf[pos++] = FUSB302_TKN_JAMCRC;
	/* EOP */
	buf[pos++] = FUSB302_TKN_EOP;
	/* turn tx off after sending message */
	buf[pos++] = FUSB302_TKN_TXOFF;
	/* start transmission */
	buf[pos++] = FUSB302_TKN_TXON;

	ret = fusb302_i2c_block_write(chip, FUSB_REG_FIFOS, pos, buf);
	if (ret < 0)
		return ret;
	fusb302_log("sending PD message header: %x\n", msg->header);
	fusb302_log("sending PD message len: %d\n", len);

	return ret;
}

static int fusb302_pd_send_hardreset(struct fusb302_chip *chip)
{
	return fusb302_i2c_set_bits(chip, FUSB_REG_CONTROL3,
				    FUSB_REG_CONTROL3_SEND_HARDRESET);
}

static const char * const transmit_type_name[] = {
	[TCPC_TX_SOP]			= "SOP",
	[TCPC_TX_SOP_PRIME]		= "SOP'",
	[TCPC_TX_SOP_PRIME_PRIME]	= "SOP''",
	[TCPC_TX_SOP_DEBUG_PRIME]	= "DEBUG'",
	[TCPC_TX_SOP_DEBUG_PRIME_PRIME]	= "DEBUG''",
	[TCPC_TX_HARD_RESET]		= "HARD_RESET",
	[TCPC_TX_CABLE_RESET]		= "CABLE_RESET",
	[TCPC_TX_BIST_MODE_2]		= "BIST_MODE_2",
};

static int tcpm_pd_transmit(struct tcpc_dev *dev, enum tcpm_transmit_type type,
			    const struct pd_message *msg)
{
	struct fusb302_chip *chip = container_of(dev, struct fusb302_chip,
						 tcpc_dev);
	int ret = 0;

	mutex_lock(&chip->lock);
	switch (type) {
	case TCPC_TX_SOP:
		/*
		 * add 10ms delay because of source cap send before partner
		 * enter wait cap state.
		 */
		if (pd_header_type_le(msg->header) == PD_DATA_SOURCE_CAP)
			msleep(10);
		ret = fusb302_pd_send_message(chip, msg);
		if (ret < 0)
			fusb302_log("cannot send PD message, ret=%d\n", ret);
		break;
	case TCPC_TX_HARD_RESET:
		ret = fusb302_pd_send_hardreset(chip);
		if (ret < 0)
			fusb302_log("cannot send hardreset, ret=%d\n", ret);
		break;
	default:
		fusb302_log("type %s not supported\n",
			    transmit_type_name[type]);
		ret = -EINVAL;
	}
	mutex_unlock(&chip->lock);

	return ret;
}

static enum typec_cc_status fusb302_bc_lvl_to_cc(u8 bc_lvl)
{
	if (bc_lvl == FUSB_REG_STATUS0_BC_LVL_1230_MAX)
		return TYPEC_CC_RP_3_0;
	if (bc_lvl == FUSB_REG_STATUS0_BC_LVL_600_1230)
		return TYPEC_CC_RP_1_5;
	if (bc_lvl == FUSB_REG_STATUS0_BC_LVL_200_600)
		return TYPEC_CC_RP_DEF;
	return TYPEC_CC_OPEN;
}

static void fusb302_bc_lvl_handler_work(struct work_struct *work)
{
	struct fusb302_chip *chip = container_of(work, struct fusb302_chip,
						 bc_lvl_handler.work);
	int ret = 0;
	u8 status0;
	u8 bc_lvl;
	enum typec_cc_status cc_status;

	mutex_lock(&chip->lock);
	if (!chip->intr_bc_lvl) {
		fusb302_log("BC_LVL interrupt is turned off, abort\n");
		goto done;
	}
	ret = fusb302_i2c_read(chip, FUSB_REG_STATUS0, &status0);
	if (ret < 0)
		goto done;
	fusb302_log("BC_LVL handler, status0=0x%02x\n", status0);
	if (status0 & FUSB_REG_STATUS0_ACTIVITY) {
		fusb302_log("CC activities detected, delay handling\n");
		mod_delayed_work(chip->wq, &chip->bc_lvl_handler,
				 msecs_to_jiffies(T_BC_LVL_DEBOUNCE_DELAY_MS));
		goto done;
	}
	bc_lvl = status0 & FUSB_REG_STATUS0_BC_LVL_MASK;
	cc_status = fusb302_bc_lvl_to_cc(bc_lvl);
	if (chip->cc_polarity == TYPEC_POLARITY_CC1) {
		if (chip->cc1 != cc_status) {
			fusb302_log("cc1: %s -> %s\n",
				    typec_cc_status_name[chip->cc1],
				    typec_cc_status_name[cc_status]);
			chip->cc1 = cc_status;
			tcpm_cc_change(chip->tcpm_port);
		}
	} else {
		if (chip->cc2 != cc_status) {
			fusb302_log("cc2: %s -> %s\n",
				    typec_cc_status_name[chip->cc2],
				    typec_cc_status_name[cc_status]);
			chip->cc2 = cc_status;
			tcpm_cc_change(chip->tcpm_port);
		}
	}

done:
	mutex_unlock(&chip->lock);
}

#define PDO_FIXED_FLAGS \
	(PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP | PDO_FIXED_USB_COMM)

static const u32 src_pdo[] = {
	PDO_FIXED(5000, 900, PDO_FIXED_FLAGS),
};

static const u32 snk_pdo[] = {
	PDO_FIXED(5000, 3000, PDO_FIXED_FLAGS),
	PDO_FIXED(9000, 2000, PDO_FIXED_FLAGS),
	PDO_BATT(4000, 10000, 18000),
};

static const struct tcpc_config fusb302_tcpc_config = {
	.src_pdo = src_pdo,
	.nr_src_pdo = ARRAY_SIZE(src_pdo),
	.snk_pdo = snk_pdo,
	.nr_snk_pdo = ARRAY_SIZE(snk_pdo),
	.max_snk_mv = 9000,
	.max_snk_ma = 3000,
	.max_snk_mw = 27000,
	.operating_snk_mw = 2500,
	.type = TYPEC_PORT_DRP,
	.default_role = TYPEC_SINK,
	.alt_modes = NULL,
};

static void init_tcpc_dev(struct tcpc_dev *fusb302_tcpc_dev)
{
	fusb302_tcpc_dev->config = &fusb302_tcpc_config;
	fusb302_tcpc_dev->init = tcpm_init;
	fusb302_tcpc_dev->get_vbus = tcpm_get_vbus;
	fusb302_tcpc_dev->set_cc = tcpm_set_cc;
	fusb302_tcpc_dev->get_cc = tcpm_get_cc;
	fusb302_tcpc_dev->set_polarity = tcpm_set_polarity;
	fusb302_tcpc_dev->set_vconn = tcpm_set_vconn;
	fusb302_tcpc_dev->set_vbus = tcpm_set_vbus;
	fusb302_tcpc_dev->set_current_limit = tcpm_set_current_limit;
	fusb302_tcpc_dev->set_pd_rx = tcpm_set_pd_rx;
	fusb302_tcpc_dev->set_roles = tcpm_set_roles;
	fusb302_tcpc_dev->start_drp_toggling = tcpm_start_drp_toggling;
	fusb302_tcpc_dev->pd_transmit = tcpm_pd_transmit;
	fusb302_tcpc_dev->mux = NULL;
}

#define VDD_3P3_VOL_MIN		3000000	/* uV */
#define VDD_3P3_VOL_MAX		3300000	/* uV */
#define VDD_OPTIMUM_MODE	40
#define SWITCH_VDD_1P8_VOL_MIN	1800000	/* uV */
#define SWITCH_VDD_1P8_VOL_MAX	1800000	/* uV */
#define SWITCH_VDD_OPTIMUM_MODE	40

static int init_regulators(struct fusb302_chip *chip)
{
	int ret = 0;

	chip->vdd = devm_regulator_get(chip->dev, "vdd");
	if (IS_ERR(chip->vdd)) {
		ret = PTR_ERR(chip->vdd);
		fusb302_log("cannot get vdd, ret=%d\n", ret);
		return ret;
	}
	ret = regulator_set_voltage(chip->vdd,
				    VDD_3P3_VOL_MIN, VDD_3P3_VOL_MAX);
	if (ret < 0) {
		fusb302_log("cannot set vdd voltage, ret=%d\n", ret);
		return ret;
	}
	ret = regulator_set_optimum_mode(chip->vdd, VDD_OPTIMUM_MODE);
	if (ret < 0) {
		fusb302_log("cannot set vdd optimum, ret=%d\n", ret);
		return ret;
	}
	ret = regulator_enable(chip->vdd);
	if (ret < 0) {
		fusb302_log("cannot enable vdd, ret=%d\n", ret);
		regulator_set_optimum_mode(chip->vdd, 0);
		return ret;
	}

	chip->switch_vdd = devm_regulator_get(chip->dev, "switch_vdd");
	if (IS_ERR(chip->switch_vdd)) {
		ret = PTR_ERR(chip->switch_vdd);
		fusb302_log("cannot get switch vdd, ret=%d\n", ret);
		goto disable_vdd;
	}
	ret = regulator_set_voltage(chip->switch_vdd, SWITCH_VDD_1P8_VOL_MIN,
				    SWITCH_VDD_1P8_VOL_MAX);
	if (ret < 0) {
		fusb302_log("cannot set switch vdd voltage, ret=%d\n", ret);
		goto disable_vdd;
	}
	ret = regulator_set_optimum_mode(chip->switch_vdd,
					 SWITCH_VDD_OPTIMUM_MODE);
	if (ret < 0) {
		fusb302_log("cannot set switch vdd optimum, ret=%d\n", ret);
		goto disable_vdd;
	}
	ret = regulator_enable(chip->switch_vdd);
	if (ret < 0) {
		fusb302_log("cannot enable switch vdd, ret=%d\n", ret);
		regulator_set_optimum_mode(chip->switch_vdd, 0);
		goto disable_vdd;
	}

	/* If vbus regulator is not ready, skip here and get it when used.*/
	chip->vbus = devm_regulator_get(chip->dev, "vbus");
	if (IS_ERR(chip->vbus)) {
		fusb302_log("cannot get vbus, ret=%d\n", ret);
		chip->vbus = NULL;
	}
	return ret;

disable_vdd:
	regulator_set_optimum_mode(chip->vdd, 0);
	regulator_disable(chip->vdd);
	return ret;
}

static const char * const cc_polarity_name[] = {
	[TYPEC_POLARITY_CC1]	= "Polarity_CC1",
	[TYPEC_POLARITY_CC2]	= "Polarity_CC2",
};

static int fusb302_set_cc_polarity(struct fusb302_chip *chip,
				   enum typec_cc_polarity cc_polarity)
{
	int ret = 0;
	u8 switches0_mask = FUSB_REG_SWITCHES0_CC1_PU_EN |
			    FUSB_REG_SWITCHES0_CC2_PU_EN |
			    FUSB_REG_SWITCHES0_VCONN_CC1 |
			    FUSB_REG_SWITCHES0_VCONN_CC2 |
			    FUSB_REG_SWITCHES0_MEAS_CC1 |
			    FUSB_REG_SWITCHES0_MEAS_CC2;
	u8 switches0_data = 0x00;
	u8 switches1_mask = FUSB_REG_SWITCHES1_TXCC1_EN |
			    FUSB_REG_SWITCHES1_TXCC2_EN;
	u8 switches1_data = 0x00;

	if (cc_polarity == TYPEC_POLARITY_CC1) {
		switches0_data = FUSB_REG_SWITCHES0_MEAS_CC1;
		if (chip->vconn_on)
			switches0_data |= FUSB_REG_SWITCHES0_VCONN_CC2;
		if (chip->pull_up)
			switches0_data |= FUSB_REG_SWITCHES0_CC1_PU_EN;
		switches1_data = FUSB_REG_SWITCHES1_TXCC1_EN;
	} else {
		switches0_data = FUSB_REG_SWITCHES0_MEAS_CC2;
		if (chip->vconn_on)
			switches0_data |= FUSB_REG_SWITCHES0_VCONN_CC1;
		if (chip->pull_up)
			switches0_data |= FUSB_REG_SWITCHES0_CC2_PU_EN;
		switches1_data = FUSB_REG_SWITCHES1_TXCC2_EN;
	}
	ret = fusb302_i2c_mask_write(chip, FUSB_REG_SWITCHES0,
				     switches0_mask, switches0_data);
	if (ret < 0)
		return ret;
	ret = fusb302_i2c_mask_write(chip, FUSB_REG_SWITCHES1,
				     switches1_mask, switches1_data);
	if (ret < 0)
		return ret;
	chip->cc_polarity = cc_polarity;

	return ret;
}

static int fusb302_handle_togdone_snk(struct fusb302_chip *chip,
				      u8 togdone_result)
{
	int ret = 0;
	u8 status0;
	u8 bc_lvl;
	enum typec_cc_polarity cc_polarity;
	enum typec_cc_status cc_status_active, cc1, cc2;

	/* set pull_up, pull_down */
	ret = fusb302_set_cc_pull(chip, false, true);
	if (ret < 0) {
		fusb302_log("cannot set cc to pull down, ret=%d\n", ret);
		return ret;
	}
	/* set polarity */
	cc_polarity = (togdone_result == FUSB_REG_STATUS1A_TOGSS_SNK1) ?
		      TYPEC_POLARITY_CC1 : TYPEC_POLARITY_CC2;
	ret = fusb302_set_cc_polarity(chip, cc_polarity);
	if (ret < 0) {
		fusb302_log("cannot set cc polarity %s, ret=%d\n",
			    cc_polarity_name[cc_polarity], ret);
		return ret;
	}
	/* fusb302_set_cc_polarity() has set the correct measure block */
	ret = fusb302_i2c_read(chip, FUSB_REG_STATUS0, &status0);
	if (ret < 0)
		return ret;
	bc_lvl = status0 & FUSB_REG_STATUS0_BC_LVL_MASK;
	cc_status_active = fusb302_bc_lvl_to_cc(bc_lvl);
	/* restart toggling if the cc status on the active line is OPEN */
	if (cc_status_active == TYPEC_CC_OPEN) {
		fusb302_log("restart toggling as CC_OPEN detected\n");
		ret = fusb302_set_toggling(chip, chip->toggling_mode);
		return ret;
	}
	/* update tcpm with the new cc value */
	cc1 = (cc_polarity == TYPEC_POLARITY_CC1) ?
	      cc_status_active : TYPEC_CC_OPEN;
	cc2 = (cc_polarity == TYPEC_POLARITY_CC2) ?
	      cc_status_active : TYPEC_CC_OPEN;
	if ((chip->cc1 != cc1) || (chip->cc2 != cc2)) {
		chip->cc1 = cc1;
		chip->cc2 = cc2;
		tcpm_cc_change(chip->tcpm_port);
	}
	/* turn off toggling */
	ret = fusb302_set_toggling(chip, TOGGLINE_MODE_OFF);
	if (ret < 0) {
		fusb302_log("cannot set toggling mode off, ret=%d\n", ret);
		return ret;
	}
	/* unmask bc_lvl interrupt */
	ret = fusb302_i2c_clear_bits(chip, FUSB_REG_MASK, FUSB_REG_MASK_BC_LVL);
	if (ret < 0) {
		fusb302_log("cannot unmask bc_lcl interrupt, ret=%d\n", ret);
		return ret;
	}
	chip->intr_bc_lvl = true;
	fusb302_log("detected cc1=%s, cc2=%s\n", typec_cc_status_name[cc1],
		    typec_cc_status_name[cc2]);

	return ret;
}

static int fusb302_handle_togdone_src(struct fusb302_chip *chip,
				      u8 togdone_result)
{
	/*
	- set polarity (measure cc, vconn, tx)
	- set pull_up, pull_down
	- set cc1, cc2, and update to tcpm_port
	- set I_COMP interrupt on
	*/
	int ret = 0;
	u8 status0;
	u8 ra_mda = ra_mda_value[chip->src_current_status];
	u8 rd_mda = rd_mda_value[chip->src_current_status];
	bool ra_comp, rd_comp;
	enum typec_cc_polarity cc_polarity;
	enum typec_cc_status cc_status_active, cc1, cc2;

	/* set pull_up, pull_down */
	ret = fusb302_set_cc_pull(chip, true, false);
	if (ret < 0) {
		fusb302_log("cannot set cc to pull up, ret=%d\n", ret);
		return ret;
	}
	/* set polarity */
	cc_polarity = (togdone_result == FUSB_REG_STATUS1A_TOGSS_SRC1) ?
		      TYPEC_POLARITY_CC1 : TYPEC_POLARITY_CC2;
	ret = fusb302_set_cc_polarity(chip, cc_polarity);
	if (ret < 0) {
		fusb302_log("cannot set cc polarity %s, ret=%d\n",
			    cc_polarity_name[cc_polarity], ret);
		return ret;
	}
	/* fusb302_set_cc_polarity() has set the correct measure block */
	ret = fusb302_i2c_write(chip, FUSB_REG_MEASURE, rd_mda);
	if (ret < 0)
		return ret;
	usleep_range(50, 100);
	ret = fusb302_i2c_read(chip, FUSB_REG_STATUS0, &status0);
	if (ret < 0)
		return ret;
	rd_comp = !!(status0 & FUSB_REG_STATUS0_COMP);
	if (!rd_comp) {
		ret = fusb302_i2c_write(chip, FUSB_REG_MEASURE, ra_mda);
		if (ret < 0)
			return ret;
		usleep_range(50, 100);
		ret = fusb302_i2c_read(chip, FUSB_REG_STATUS0, &status0);
		if (ret < 0)
			return ret;
		ra_comp = !!(status0 & FUSB_REG_STATUS0_COMP);
	}
	if (rd_comp)
		cc_status_active = TYPEC_CC_OPEN;
	else if (ra_comp)
		cc_status_active = TYPEC_CC_RD;
	else
		/* Ra is not supported, report as Open */
		cc_status_active = TYPEC_CC_OPEN;
	/* restart toggling if the cc status on the active line is OPEN */
	if (cc_status_active == TYPEC_CC_OPEN) {
		fusb302_log("restart toggling as CC_OPEN detected\n");
		ret = fusb302_set_toggling(chip, chip->toggling_mode);
		return ret;
	}
	/* update tcpm with the new cc value */
	cc1 = (cc_polarity == TYPEC_POLARITY_CC1) ?
	      cc_status_active : TYPEC_CC_OPEN;
	cc2 = (cc_polarity == TYPEC_POLARITY_CC2) ?
	      cc_status_active : TYPEC_CC_OPEN;
	if ((chip->cc1 != cc1) || (chip->cc2 != cc2)) {
		chip->cc1 = cc1;
		chip->cc2 = cc2;
		tcpm_cc_change(chip->tcpm_port);
	}
	/* turn off toggling */
	ret = fusb302_set_toggling(chip, TOGGLINE_MODE_OFF);
	if (ret < 0) {
		fusb302_log("cannot set toggling mode off, ret=%d\n", ret);
		return ret;
	}
	/* set MDAC to Rd threshold, and unmask I_COMP for unplug detection */
	ret = fusb302_i2c_write(chip, FUSB_REG_MEASURE, rd_mda);
	if (ret < 0)
		return ret;
	/* unmask comp_chng interrupt */
	ret = fusb302_i2c_clear_bits(chip, FUSB_REG_MASK,
				     FUSB_REG_MASK_COMP_CHNG);
	if (ret < 0) {
		fusb302_log("cannot unmask bc_lcl interrupt, ret=%d\n", ret);
		return ret;
	}
	chip->intr_comp_chng = true;
	fusb302_log("detected cc1=%s, cc2=%s\n", typec_cc_status_name[cc1],
		    typec_cc_status_name[cc2]);

	return ret;
}

static int fusb302_handle_togdone(struct fusb302_chip *chip)
{
	int ret = 0;
	u8 status1a;
	u8 togdone_result;

	ret = fusb302_i2c_read(chip, FUSB_REG_STATUS1A, &status1a);
	if (ret < 0)
		return ret;
	togdone_result = (status1a >> FUSB_REG_STATUS1A_TOGSS_POS) &
			 FUSB_REG_STATUS1A_TOGSS_MASK;
	switch (togdone_result) {
	case FUSB_REG_STATUS1A_TOGSS_SNK1:
	case FUSB_REG_STATUS1A_TOGSS_SNK2:
		return fusb302_handle_togdone_snk(chip, togdone_result);
	case FUSB_REG_STATUS1A_TOGSS_SRC1:
	case FUSB_REG_STATUS1A_TOGSS_SRC2:
		return fusb302_handle_togdone_src(chip, togdone_result);
	case FUSB_REG_STATUS1A_TOGSS_AA:
		/* doesn't support */
		fusb302_log("AudioAccessory not supported\n");
		fusb302_set_toggling(chip, chip->toggling_mode);
		break;
	default:
		fusb302_log("TOGDONE with an invalid state: %d\n",
			    togdone_result);
		fusb302_set_toggling(chip, chip->toggling_mode);
		break;
	}
	return ret;
}

static int fusb302_pd_reset(struct fusb302_chip *chip)
{
	return fusb302_i2c_set_bits(chip, FUSB_REG_RESET,
				    FUSB_REG_RESET_PD_RESET);
}

static int fusb302_pd_read_message(struct fusb302_chip *chip,
				   struct pd_message *msg)
{
	int ret = 0;
	u8 token;
	u8 crc[4];
	int len;

	/* first SOP token */
	ret = fusb302_i2c_read(chip, FUSB_REG_FIFOS, &token);
	if (ret < 0)
		return ret;
	ret = fusb302_i2c_block_read(chip, FUSB_REG_FIFOS, 2,
				     (u8 *)&msg->header);
	if (ret < 0)
		return ret;
	len = pd_header_cnt(msg->header) * 4;
	/* add 4 to length to include the CRC */
	if (len > PD_MAX_PAYLOAD * 4) {
		fusb302_log("PD message too long %d\n", len);
		return -EINVAL;
	}
	if (len > 0) {
		ret = fusb302_i2c_block_read(chip, FUSB_REG_FIFOS, len,
					     (u8 *)msg->payload);
		if (ret < 0)
			return ret;
	}
	/* another 4 bytes to read CRC out */
	ret = fusb302_i2c_block_read(chip, FUSB_REG_FIFOS, 4, crc);
	if (ret < 0)
		return ret;
	fusb302_log("PD message header: %x\n", msg->header);
	fusb302_log("PD message len: %d\n", len);

	return ret;
}

static irqreturn_t fusb302_irq_intn(int irq, void *dev_id)
{
	struct fusb302_chip *chip = dev_id;
	int ret = 0;
	u8 interrupt;
	u8 interrupta;
	u8 interruptb;
	u8 status0;
	u8 control2;
	u8 maska;
	u8 mask1;
	bool vbus_present;
	bool comp_result;
	bool intr_togdone;
	bool intr_bc_lvl;
	bool intr_comp_chng;
	struct pd_message pd_msg;

	pm_wakeup_event(chip->dev, PM_WAKE_DELAY_MS);

	mutex_lock(&chip->lock);
	/* grab a snapshot of intr flags */
	intr_togdone = chip->intr_togdone;
	intr_bc_lvl = chip->intr_bc_lvl;
	intr_comp_chng = chip->intr_comp_chng;

	ret = fusb302_i2c_read(chip, FUSB_REG_INTERRUPT, &interrupt);
	if (ret < 0)
		goto done;
	ret = fusb302_i2c_read(chip, FUSB_REG_INTERRUPTA, &interrupta);
	if (ret < 0)
		goto done;
	ret = fusb302_i2c_read(chip, FUSB_REG_INTERRUPTB, &interruptb);
	if (ret < 0)
		goto done;
	ret = fusb302_i2c_read(chip, FUSB_REG_STATUS0, &status0);
	if (ret < 0)
		goto done;
	ret = fusb302_i2c_read(chip, FUSB_REG_CONTROL2, &control2);
	if (ret < 0)
		goto done;
	ret = fusb302_i2c_read(chip, FUSB_REG_MASKA, &maska);
	if (ret < 0)
		goto done;
	ret = fusb302_i2c_read(chip, FUSB_REG_MASK, &mask1);
	if (ret < 0)
		goto done;

	fusb302_log("IRQ: 0x%02x, a: 0x%02x, b: 0x%02x, status0: 0x%02x",
		    interrupt, interrupta, interruptb, status0);
	fusb302_log("control2: 0x%02x maska: 0x%02x mask1: 0x%02x\n",
		    control2, maska, mask1);

	if (interrupt & FUSB_REG_INTERRUPT_VBUSOK) {
		vbus_present = !!(status0 & FUSB_REG_STATUS0_VBUSOK);
		fusb302_log("IRQ: VBUS_OK, vbus=%s\n",
			    vbus_present ? "On" : "Off");
		if (vbus_present != chip->vbus_present) {
			chip->vbus_present = vbus_present;
			tcpm_vbus_change(chip->tcpm_port);
		}
	}

	if ((interrupta & FUSB_REG_INTERRUPTA_TOGDONE) && intr_togdone) {
		fusb302_log("IRQ: TOGDONE\n");
		ret = fusb302_handle_togdone(chip);
		if (ret < 0) {
			fusb302_log("handle togdone error, ret=%d\n", ret);
			goto done;
		}
	}

	if ((interrupt & FUSB_REG_INTERRUPT_BC_LVL) && intr_bc_lvl) {
		fusb302_log("IRQ: BC_LVL, handler pending\n");
		/*
		 * as BC_LVL interrupt can be affected by PD activity,
		 * apply delay to for the handler to wait for the PD
		 * signaling to finish.
		 */
		mod_delayed_work(chip->wq, &chip->bc_lvl_handler,
				 msecs_to_jiffies(T_BC_LVL_DEBOUNCE_DELAY_MS));
	}

	if ((interrupt & FUSB_REG_INTERRUPT_COMP_CHNG) && intr_comp_chng) {
		comp_result = !!(status0 & FUSB_REG_STATUS0_COMP);
		fusb302_log("IRQ: COMP_CHNG, comp=%s\n",
			    comp_result ? "true" : "false");
		if (comp_result) {
			/* cc level > Rd_threashold, detach */
			if (chip->cc_polarity == TYPEC_POLARITY_CC1)
				chip->cc1 = TYPEC_CC_OPEN;
			else
				chip->cc2 = TYPEC_CC_OPEN;
			tcpm_cc_change(chip->tcpm_port);
		}
	}

	if (interrupt & FUSB_REG_INTERRUPT_COLLISION) {
		fusb302_log("IRQ: PD collision\n");
		tcpm_pd_transmit_complete(chip->tcpm_port, TCPC_TX_FAILED);
	}

	if (interrupta & FUSB_REG_INTERRUPTA_RETRYFAIL) {
		fusb302_log("IRQ: PD retry failed\n");
		tcpm_pd_transmit_complete(chip->tcpm_port, TCPC_TX_FAILED);
	}

	if (interrupta & FUSB_REG_INTERRUPTA_HARDSENT) {
		fusb302_log("IRQ: PD hardreset sent\n");
		ret = fusb302_pd_reset(chip);
		if (ret < 0) {
			fusb302_log("cannot PD reset, ret=%d\n", ret);
			goto done;
		}
		tcpm_pd_transmit_complete(chip->tcpm_port, TCPC_TX_SUCCESS);
	}

	if (interrupta & FUSB_REG_INTERRUPTA_TX_SUCCESS) {
		fusb302_log("IRQ: PD tx success\n");
		/* read out the received good CRC */
		ret = fusb302_pd_read_message(chip, &pd_msg);
		if (ret < 0) {
			fusb302_log("cannot read in GCRC, ret=%d\n", ret);
			goto done;
		}
		tcpm_pd_transmit_complete(chip->tcpm_port, TCPC_TX_SUCCESS);
	}

	if (interrupta & FUSB_REG_INTERRUPTA_HARDRESET) {
		fusb302_log("IRQ: PD received hardreset\n");
		ret = fusb302_pd_reset(chip);
		if (ret < 0) {
			fusb302_log("cannot PD reset, ret=%d\n", ret);
			goto done;
		}
		tcpm_pd_hard_reset(chip->tcpm_port);
	}

	if (interruptb & FUSB_REG_INTERRUPTB_GCRCSENT) {
		fusb302_log("IRQ: PD sent good CRC\n");
		ret = fusb302_pd_read_message(chip, &pd_msg);
		if (ret < 0) {
			fusb302_log("cannot read in PD message, ret=%d\n", ret);
			goto done;
		}
		tcpm_pd_receive(chip->tcpm_port, &pd_msg);
	}
done:
	mutex_unlock(&chip->lock);

	return IRQ_HANDLED;
}

static int init_gpio(struct fusb302_chip *chip)
{
	struct device_node *node;
	struct pinctrl_state *set_state;
	int ret = 0;

	node = chip->dev->of_node;
	chip->gpio_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR(chip->gpio_pinctrl)) {
		ret = PTR_ERR(chip->gpio_pinctrl);
		fusb302_log("cannot get pinctrl, ret=%d\n", ret);
		return ret;
	}
	set_state = pinctrl_lookup_state(chip->gpio_pinctrl, "default");
	if (IS_ERR(set_state)) {
		ret = PTR_ERR(chip->gpio_pinctrl);
		fusb302_log("cannot find the default pinctrl state, ret=%d\n",
			    ret);
		return ret;
	}
	ret = pinctrl_select_state(chip->gpio_pinctrl, set_state);
	if (ret < 0) {
		fusb302_log("cannot select state, ret=%d\n", ret);
		return ret;
	}
	chip->gpio_int_n = of_get_named_gpio(node, "fairchild,int_n", 0);
	if (!gpio_is_valid(chip->gpio_int_n)) {
		ret = chip->gpio_int_n;
		fusb302_log("cannot get named GPIO Int_N, ret=%d\n", ret);
		return ret;
	}
	ret = devm_gpio_request(chip->dev, chip->gpio_int_n, "fairchild,int_n");
	if (ret < 0) {
		fusb302_log("cannot request GPIO Int_N, ret=%d\n", ret);
		return ret;
	}
	ret = gpio_direction_input(chip->gpio_int_n);
	if (ret < 0) {
		fusb302_log("cannot set GPIO Int_N to input, ret=%d\n", ret);
		return ret;
	}
	ret = gpio_to_irq(chip->gpio_int_n);
	if (ret < 0) {
		fusb302_log("cannot request IRQ for GPIO Int_N, ret=%d\n", ret);
		return ret;
	}
	chip->gpio_int_n_irq = ret;
	return 0;
}

static int fusb302_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct fusb302_chip *chip;
	struct i2c_adapter *adapter;
	int ret = 0;

	if (!fusb302_log)
		fusb302_log = ipc_log_context_create(NUM_LOG_PAGES,
						     "fusb302", 0);
	adapter = to_i2c_adapter(client->dev.parent);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		fusb302_log("I2C/SMBus block functionality not supported!\n");
		return -ENODEV;
	}
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->i2c_client = client;
	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	mutex_init(&chip->lock);

	/* If batt_psy is not ready in probe, skip here and get it when used. */
	chip->batt_psy = power_supply_get_by_name("battery");
	if (IS_ERR(chip->batt_psy)) {
		ret = PTR_ERR(chip->batt_psy);
		fusb302_log("cannot get battery power supply, ret=%d\n", ret);
		chip->batt_psy = NULL;
	}

	chip->wq = create_singlethread_workqueue(dev_name(chip->dev));
	if (!chip->wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&chip->bc_lvl_handler, fusb302_bc_lvl_handler_work);
	init_tcpc_dev(&chip->tcpc_dev);

	ret = init_regulators(chip);
	if (ret < 0)
		return ret;
	ret = init_gpio(chip);
	if (ret < 0)
		goto disable_regulators;
	chip->tcpm_port = tcpm_register_port(&client->dev, &chip->tcpc_dev);
	if (IS_ERR(chip->tcpm_port)) {
		ret = PTR_ERR(chip->tcpm_port);
		fusb302_log("cannot register tcpm port, ret=%d\n", ret);
		goto disable_regulators;
	}
	ret = devm_request_threaded_irq(chip->dev, chip->gpio_int_n_irq,
					NULL, fusb302_irq_intn,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					"fsc_interrupt_int_n", chip);
	if (ret < 0) {
		fusb302_log("cannot request IRQ for GPIO Int_N, ret=%d\n", ret);
		goto disable_regulators;
	}
	__fusb302_chip = chip;

	ret = device_init_wakeup(chip->dev, true);

	if (unlikely(ret < 0)) {
		fusb302_log("wakeup init failed, ret=%d\n", ret);
		goto disable_regulators;
	}

	enable_irq_wake(chip->gpio_int_n_irq);
	return ret;

disable_regulators:
	regulator_set_optimum_mode(chip->vdd, 0);
	regulator_disable(chip->vdd);
	regulator_set_optimum_mode(chip->switch_vdd, 0);
	regulator_disable(chip->switch_vdd);

	return ret;
}

static int fusb302_remove(struct i2c_client *client)
{
	struct fusb302_chip *chip = i2c_get_clientdata(client);

	tcpm_unregister_port(chip->tcpm_port);
	regulator_disable(chip->vdd);
	regulator_disable(chip->switch_vdd);

	return 0;
}

static int fusb302_pm_suspend(struct device *dev)
{
	struct fusb302_chip *chip = dev->driver_data;

	if (atomic_read(&chip->i2c_busy))
		return -EBUSY;
	atomic_set(&chip->pm_suspend, 1);

	return 0;
}

static int fusb302_pm_resume(struct device *dev)
{
	struct fusb302_chip *chip = dev->driver_data;

	atomic_set(&chip->pm_suspend, 0);

	return 0;
}

static const struct of_device_id fusb302_dt_match[] = {
	{.compatible = "fairchild,fusb302"},
	{},
};

static const struct i2c_device_id fusb302_i2c_device_id[] = {
	{"typec_fusb302", 0},
	{},
};

static const struct dev_pm_ops fusb302_pm_ops = {
	.suspend = fusb302_pm_suspend,
	.resume = fusb302_pm_resume,
};

static struct i2c_driver fusb302_driver = {
	.driver = {
		   .name = "typec_fusb302",
		   .pm = &fusb302_pm_ops,
		   .of_match_table = of_match_ptr(fusb302_dt_match),
		   },
	.probe = fusb302_probe,
	.remove = fusb302_remove,
	.id_table = fusb302_i2c_device_id,
};
module_i2c_driver(fusb302_driver);

int usb_controller_register(struct device *parent, struct usb_controller *uc)
{
	struct fusb302_chip *chip = __fusb302_chip;

	if (chip == NULL)
		return -ENODEV;

	mutex_lock(&chip->lock);
	chip->uc = uc;
	mutex_unlock(&chip->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_controller_register);

int usb_typec_ctrl_register(struct device *parent, struct usb_typec_ctrl *utc)
{
	struct fusb302_chip *chip = __fusb302_chip;

	if (chip == NULL)
		return -ENODEV;

	mutex_lock(&chip->lock);
	chip->utc = utc;
	mutex_unlock(&chip->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_typec_ctrl_register);

MODULE_AUTHOR("Yueyao Zhu <yueyao@google.com>");
MODULE_DESCRIPTION("Fairchild FUSB302 Type-C Chip Driver");
MODULE_LICENSE("GPL");

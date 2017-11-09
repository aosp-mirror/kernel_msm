/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name		: fts.c
* Authors		: AMS(Analog Mems Sensor) Team
* Description	: FTS Capacitive touch screen controller (FingerTipS)
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
********************************************************************************
* REVISON HISTORY
* DATE		| DESCRIPTION
* 03/09/2012| First Release
* 08/11/2012| Code migration
* 09/04/2013| Support Blob Information
*******************************************************************************/

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
#include <linux/trustedui.h>
#endif

#ifdef CONFIG_OF
#ifndef USE_OPEN_CLOSE
#define USE_OPEN_CLOSE
#undef CONFIG_PM
#endif
#endif

#include <linux/input/mt.h>
#include "ftm4_ts.h"

#define FTS_REGISTER_PSY_MS  200

static struct i2c_driver fts_i2c_driver;

extern int msm_gpio_install_direct_irq(unsigned int gpio,
		unsigned int irq,
		unsigned int input_polarity);

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
extern int tui_force_close(uint32_t arg);
struct fts_ts_info *tui_tsp_info;
#endif

#ifdef USE_OPEN_CLOSE
static int fts_input_open(struct input_dev *dev);
static void fts_input_close(struct input_dev *dev);
#ifdef USE_OPEN_DWORK
static void fts_open_work(struct work_struct *work);
#endif
#endif

static int fts_stop_device(struct fts_ts_info *info);
static int fts_start_device(struct fts_ts_info *info);
static void fts_irq_enable(struct fts_ts_info *info, bool enable);
static void fts_reset_work(struct work_struct *work);
void fts_recovery_cx(struct fts_ts_info *info);
void fts_release_all_finger(struct fts_ts_info *info);
static int fts_suspend(struct i2c_client *client, pm_message_t mesg);
static int fts_resume(struct i2c_client *client);

#if defined(CONFIG_FB)
static int touch_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data);
#endif

static int fts_ts_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	return -EINVAL;
}

static enum power_supply_property fts_ts_props[] = {
	POWER_SUPPLY_PROP_TYPE,
};

static void fts_control_ta_detect_pin(struct fts_ts_info *info)
{
	int ta_pin = 0;

	if (!gpio_is_valid(info->board->ta_detect_pin))
		return;

	gpio_set_value(info->board->ta_detect_pin, info->charger_connected);

	ta_pin = gpio_get_value(info->board->ta_detect_pin);

	tsp_debug_info(&info->client->dev,
			"%s: ta_detect_pin status = %d.\n",
			__func__, ta_pin);
}

static void fts_external_power_changed(struct power_supply *psy)
{
	struct fts_ts_info *info = power_supply_get_drvdata(psy);
	union power_supply_propval prop = {0};

	if (!info->usb_psy)
		info->usb_psy = power_supply_get_by_name("usb");
	if (!info->usb_psy)
		return;
	if (info->usb_psy) {
		power_supply_get_property(info->usb_psy,
					  POWER_SUPPLY_PROP_ONLINE, &prop);
		if (info->charger_connected != prop.intval) {
			tsp_debug_info(&info->client->dev,
				"%s: charger_connected transition: %d => %d.\n",
				__func__, info->charger_connected, prop.intval);
			info->charger_connected = prop.intval;
			fts_control_ta_detect_pin(info);
		}
	}
}

static const struct power_supply_desc fts_ts_desc = {
	.name			= "touch",
	.type			= POWER_SUPPLY_TYPE_UNKNOWN,
	.properties		= fts_ts_props,
	.num_properties		= ARRAY_SIZE(fts_ts_props),
	.get_property		= fts_ts_get_property,
	.external_power_changed = fts_external_power_changed,
};

static void fts_psy_work(struct work_struct *work)
{
	struct fts_ts_info *info = container_of(work, struct fts_ts_info,
						psy_work.work);
	struct power_supply_config psy_cfg = {};

	psy_cfg.of_node = info->dev->of_node;
	psy_cfg.drv_data = info;
	info->ts_psy = devm_power_supply_register(info->dev, &fts_ts_desc,
						  &psy_cfg);
	if (!IS_ERR_OR_NULL(info->ts_psy)) {
		fts_external_power_changed(info->ts_psy);
	} else if (PTR_ERR(info->ts_psy) == -EPROBE_DEFER) {
		schedule_delayed_work(&info->psy_work,
				      msecs_to_jiffies(FTS_REGISTER_PSY_MS));
	} else {
		tsp_debug_err(info->dev,
			      "%s: Failed to register power supply\n",
			      __func__);
	}
}

int fts_write_reg(struct fts_ts_info *info,
		  unsigned char *reg, unsigned short num_com)
{
	struct i2c_msg xfer_msg[2];
	int ret = 0;

	if (info->touch_stopped) {
		tsp_debug_err(&info->client->dev,
			"%s: Sensor stopped\n", __func__);
		goto exit;
	}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_INPUT_SECURED & trustedui_get_current_mode()) {
		tsp_debug_err(&info->client->dev,
			"%s TSP no accessible from Linux,"
			"TUI is enabled!\n", __func__);
		return -EIO;
	}
#endif

	mutex_lock(&info->i2c_mutex);

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = num_com;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	ret = i2c_transfer(info->client->adapter, xfer_msg, 1);

	mutex_unlock(&info->i2c_mutex);
	return ret;

 exit:
	return -EIO;
}

int fts_read_reg(struct fts_ts_info *info, unsigned char *reg, int cnum,
		 unsigned char *buf, int num)
{
	struct i2c_msg xfer_msg[2];
	int ret = 0;

	if (info->touch_stopped) {
		tsp_debug_err(&info->client->dev,
			"%s: Sensor stopped\n", __func__);
		goto exit;
	}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_INPUT_SECURED & trustedui_get_current_mode()) {
		tsp_debug_err(&info->client->dev,
			"%s TSP no accessible from Linux,"
			" TUI is enabled!\n", __func__);
		return -EIO;
	}
#endif

	mutex_lock(&info->i2c_mutex);

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = cnum;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	xfer_msg[1].addr = info->client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	ret = i2c_transfer(info->client->adapter, xfer_msg, 2);

	mutex_unlock(&info->i2c_mutex);

	return ret;

 exit:
	return -EIO;
}

void fts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

void fts_command(struct fts_ts_info *info, unsigned char cmd)
{
	unsigned char regAdd = 0;
	int ret = 0;

	regAdd = cmd;
	ret = fts_write_reg(info, &regAdd, 1);
	tsp_debug_dbg(&info->client->dev,
		"FTS Command (%02X) , "
		"ret = %d\n", cmd, ret);
}

void fts_change_scan_rate(struct fts_ts_info *info, unsigned char cmd)
{
	unsigned char regAdd[2] = {0xC3, 0x00};
	int ret = 0;

	regAdd[1] = cmd;
	ret = fts_write_reg(info, &regAdd[0], 2);

	tsp_debug_dbg(&info->client->dev,
		"FTS %s Scan Rate (%02X %02X) , ret = %d\n",
		(cmd == FTS_CMD_FAST_SCAN) ? "90Hz" :
		(cmd == FTS_CMD_SLOW_SCAN) ? "60Hz" : "30Hz",
		regAdd[0], regAdd[1], ret);
}

int fts_systemreset(struct fts_ts_info *info)
{
	int ret = 0;
	unsigned char addr[4] = {0xB6, 0x00, 0x28, 0x80};
	unsigned char addr_wbcrc[4] = {0xB6, 0x00, 0x1E, 0x20};

	tsp_debug_info(&info->client->dev, "FTS Enable WBCRC\n");
	ret = fts_write_reg(info, &addr_wbcrc[0], 4);
	fts_delay(10);

	tsp_debug_dbg(&info->client->dev, "FTS SystemReset\n");
	ret = fts_write_reg(info, &addr[0], 4);
	fts_delay(10);

	return ret;
}

static void fts_interrupt_set(struct fts_ts_info *info, int enable)
{
	unsigned char regAdd[4] = {0xB6, 0x00, 0x2C, INT_ENABLE};

	if (enable== INT_ENABLE) {
		tsp_debug_dbg(&info->client->dev, "FTS INT Enable\n");
	} else {
		regAdd[3] = INT_DISABLE;
		tsp_debug_dbg(&info->client->dev, "FTS INT Disable\n");
	}

	fts_write_reg(info, &regAdd[0], 4);
}

void fts_get_afe_info(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char regAdd[3];
	unsigned char data[FTS_EVENT_SIZE];

	/* Reading the final AFE version */
	regAdd[0] = 0xd0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x52;

	rc = fts_read_reg(info, regAdd, 3, (unsigned char *)data, 3);
	if (rc < 0) {
		info->afe_ver = 0;
		tsp_debug_err(info->dev,
				"%s: Read Fail - Final AFE [Data : "
				"%2X] AFE Ver [Data : %2X] \n",
				__func__,
				data[1],
				data[2]);
	} else
		info->afe_ver = data[2];
}

static int fts_product_info_read(struct fts_ts_info *info)
{
	/* This function must be called with interrupts/irqs disabled */
	unsigned char data[FTS_EVENT_SIZE] = {0,};
	unsigned char prd_info[FTS_LOCKDOWNCODE_SIZE] ={0x0,};
	static u8 addr[2] = {READ_ONE_EVENT, 0};
	int retry = (FTS_RETRY_COUNT * 5);
	int total_length = 0;
	int offset = 0;
	int ret = 0;
	int i = 0;

	memset(&info->prd_info, 0, sizeof(struct fts_prd_info));

	info->fts_command(info, SENSEOFF);

	fts_command(info,LOCKDOWN_READ);

	while (retry--) {
		fts_delay(5);

		ret = fts_read_reg(info, &addr[0], 1, &data[0], FTS_EVENT_SIZE);
		if (ret < 0) {
			tsp_debug_err(info->dev, "ftm4_reg_read fail\n");
			goto error;
		}

		if (data[0] == EVENTID_LOCKDOWN_CODE) {
			total_length = data[1];
			offset = data[2];

			tsp_debug_dbg(info->dev, "Total length : %d |  offset : %d\n", total_length, offset);

			if (total_length == FTS_LOCKDOWNCODE_SIZE) {
				for (i = 0; i < 4; i++) {
					if (offset+i >= FTS_LOCKDOWNCODE_SIZE) {
						memcpy(&info->prd_info.product_id[0], &prd_info[0], 3);
						info->prd_info.chip_rev = (prd_info[3] >> 4) & 0xF;
						info->prd_info.fpc_rev = prd_info[3] & 0xF;
						info->prd_info.t_sensor_rev = prd_info[4];
						info->prd_info.site = prd_info[5];
						info->prd_info.inspector_no = prd_info[6];
						memcpy(&info->prd_info.date[0], &prd_info[7], 6);

						info->fts_command(info, SENSEON);
						return 0;
					}
					prd_info[offset+i] = data[i+3];
					tsp_debug_dbg(info->dev, "[fts_lockdown_read] code [0x%02X]\n", prd_info[offset+i]);
				}
			}
		} else if ((data[0] == EVENTID_ERROR) && (data[1] == EVENTID_ERROR_LOCKDOWN)) {
			switch (data[2] & 0x0F) {
			case 0x01:
				tsp_debug_err(info->dev, "[fts_lockdown_read] Error - no lockdown code");
				goto error;
			case 0x02:
				tsp_debug_err(info->dev, "[fts_lockdown_read] Error - Data Corrupted");
				goto error;
			case 0x03:
				tsp_debug_err(info->dev, "[fts_lockdown_read] Error - Command format invalid");
				goto error;
			}
		}
	}

	tsp_debug_err(info->dev, "[fts_lockdown_read] Error - Time over, retry =%d", retry);
error:
	info->fts_command(info, SENSEON);

	return -EINVAL;
}

int fts_get_version_info(struct fts_ts_info *info)
{
	/* This function must be called with interrupts/irqs disabled */
	int rc = 0;
	unsigned char addr[3] = {0xD0, 0x00, 0x56};
	unsigned char buff[7] = {0};
	char str[16] = {0};
	int str_ret = 0;

	rc = fts_read_reg(info, &addr[0], 3, &buff[0], 7);
	if (rc < 0) {
		tsp_debug_err(&info->client->dev, "FTS get version info fail!\n");
		goto error;
	}

	info->fw_version_of_ic = buff[1] + (buff[2] << 8);
	info->config_version_of_ic = buff[3] + (buff[4] << 8);
	info->fw_main_version_of_ic = buff[6] + (buff[5] << 8);
	info->ic_fw_ver.build = ((buff[5] >> 4) & 0x0F);
	info->ic_fw_ver.major = (buff[5] & 0x0F);
	info->ic_fw_ver.minor = buff[6];

	str_ret += snprintf(str + str_ret, sizeof(str) - str_ret,
			"v%d.%02d",
			info->ic_fw_ver.major,
			info->ic_fw_ver.minor);

	if (info->ic_fw_ver.build) {
		str_ret += snprintf(str + str_ret, sizeof(str) - str_ret,
				".%d",
				info->ic_fw_ver.build);
	}

	fts_get_afe_info(info);
	fts_product_info_read(info);
	get_pure_autotune_status(info);

	tsp_debug_info(&info->client->dev,
			"IC Firmware Version: 0x%04X [%s] "
			"IC Config Version: 0x%04X "
			"IC Main Version: 0x%04X "
			"AFE Version: 0x%02X\n",
			info->fw_version_of_ic, str,
			info->config_version_of_ic,
			info->fw_main_version_of_ic,
			info->afe_ver);
	tsp_debug_info(&info->client->dev,
			"product id: [%02x %02x %02x]\n",
			info->prd_info.product_id[0],
			info->prd_info.product_id[1],
			info->prd_info.product_id[2]);
	tsp_debug_info(&info->client->dev,
			"Chip revision: %d, fpc: %d, t_sensor: %d, site: %d, "
			"inspector_no: %d\n",
			info->prd_info.chip_rev, info->prd_info.fpc_rev,
			info->prd_info.t_sensor_rev, info->prd_info.site,
			info->prd_info.inspector_no);
	tsp_debug_info(&info->client->dev,
			"date : %02d.%02d.%02d %02d:%02d:%02d\n",
			info->prd_info.date[0], info->prd_info.date[1],
			info->prd_info.date[2], info->prd_info.date[3],
			info->prd_info.date[4], info->prd_info.date[5]);

error:
	return rc;
}

int fts_read_chip_id(struct fts_ts_info *info)
{
	unsigned char regAdd[3] = {0xB6, 0x00, 0x04};
	unsigned char val[7] = {0};
	int ret = 0;

	ret = fts_read_reg(info, regAdd, 3, (unsigned char *)val, 7);
	if (ret < 0) {
		tsp_debug_err(&info->client->dev, "%s failed. ret: %d\n",
			__func__, ret);
		return ret;
	}

	tsp_debug_dbg(&info->client->dev,
		"FTS %02X%02X%02X =  %02X %02X %02X %02X %02X %02X\n",
	       regAdd[0], regAdd[1], regAdd[2],
	       val[1], val[2], val[3], val[4],
	       val[5], val[6]);

	if ((val[1] == FTS_ID0) && (val[2] == FTS_ID1)) {
		if ((val[5] == 0x00) && (val[6] == 0x00)) {
			tsp_debug_err(&info->client->dev,
				"\n\r[fts_read_chip_id] Error - No FW : %02X %02X",
				val[5], val[6]);
			info->flash_corruption_info.fw_broken = true;
		}  else {
			tsp_debug_info(&info->client->dev,
				"FTS Chip ID : %02X %02X\n",
				val[1], val[2]);
			info->flash_corruption_info.fw_broken = false;
		}
	} else
		return -FTS_ERROR_INVALID_CHIP_ID;

	return ret;
}

int fts_wait_for_ready(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char addr;
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;
	int err_cnt = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	addr = READ_ONE_EVENT;

	while (fts_read_reg(info, &addr, 1,
				(unsigned char *)data, FTS_EVENT_SIZE)) {
		if (data[0] == EVENTID_CONTROLLER_READY) {
			info->flash_corruption_info.cfg_broken = false;
			info->flash_corruption_info.cx_broken = false;
			rc = 0;
			break;
		}

		if (data[0] == EVENTID_ERROR) {
			if (data[1] == EVENTID_ERROR_FLASH_CORRUPTION) {
				rc = -FTS_ERROR_EVENT_ID;

				tsp_debug_err(&info->client->dev,
					"%s: flash corruption:%02X,%02X,%02X\n",
						__func__, data[0],
						data[1], data[2]);

				switch (data[2]) {
				case EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_1:
					info->flash_corruption_info.cfg_broken = true;
					break;
				case EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_2:
					info->flash_corruption_info.cfg_broken = true;
					break;
				case EVENTID_ERROR_CX_FLASH_CORRUPTION:
					info->flash_corruption_info.cx_broken = true;
					break;
				default:
					break;
				}
			}

			if (err_cnt++ > 32) {
				rc = -FTS_ERROR_EVENT_ID;
				break;
			}
			continue;
		}

		if (retry++ > FTS_RETRY_COUNT) {
			rc = -FTS_ERROR_TIMEOUT;
			tsp_debug_err(&info->client->dev, "%s: Time Over\n",
					__func__);

			if (info->lowpower_mode) {
				schedule_delayed_work(&info->reset_work,
					msecs_to_jiffies(10));
			}
			break;
		}
		fts_delay(20);
	}

	tsp_debug_dbg(&info->client->dev,
		"%s: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
		__func__, data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7]);

	return rc;
}

int fts_get_channel_info(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char cmd[4] = { 0xB2, 0x00, 0x14, 0x02 };
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	fts_write_reg(info, &cmd[0], 4);
	cmd[0] = READ_ONE_EVENT;
	while (fts_read_reg
	       (info, &cmd[0], 1, (unsigned char *)data, FTS_EVENT_SIZE)) {
		if (data[0] == EVENTID_RESULT_READ_REGISTER) {
			if ((data[1] == cmd[1]) && (data[2] == cmd[2])) {
				info->SenseChannelLength = data[3];
				info->ForceChannelLength = data[4];
				rc = 0;
				break;
			}
		}
		if (retry++ > 30) {
			rc = -1;
			tsp_debug_err(&info->client->dev,
					"Time over - wait for channel info\n");
			break;
		}
		fts_delay(5);
	}
	return rc;
}

#ifdef FTS_SUPPORT_NOISE_PARAM
int fts_get_noise_param_address(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char regAdd[3];
	unsigned char rData[3];
	struct fts_noise_param *noise_param;
	int i;

	noise_param = (struct fts_noise_param *)&info->noise_param;

	regAdd[0] = 0xd0;
	regAdd[1] = 0x00;
	regAdd[2] = 32 * 2;

	if (info->digital_rev == FTS_DIGITAL_REV_1)
		rc = fts_read_reg(info, regAdd, 3,
				(unsigned char *)noise_param->pAddr, 2);
	else {
		rc = fts_read_reg(info, regAdd, 3, (unsigned char *)rData, 3);
		noise_param->pAddr[0] = rData[1] + (rData[2]<<8);
	}

	for (i = 1; i < MAX_NOISE_PARAM; i++)
		noise_param->pAddr[i] = noise_param->pAddr[0] + i * 2;

	for (i = 0; i < MAX_NOISE_PARAM; i++)
		tsp_debug_dbg(&info->client->dev,
				"Get Noise Param%d Address = 0x%4x\n",
				i, noise_param->pAddr[i]);

	return rc;
}

static int fts_get_noise_param(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char regAdd[3];
	unsigned char data[MAX_NOISE_PARAM * 2];
	struct fts_noise_param *noise_param;
	int i;
	unsigned char buf[3];

	noise_param = (struct fts_noise_param *)&info->noise_param;
	memset(data, 0x0, MAX_NOISE_PARAM * 2);

	for (i = 0; i < MAX_NOISE_PARAM; i++) {
		regAdd[0] = 0xb3;
		regAdd[1] = 0x00;
		regAdd[2] = 0x10;
		fts_write_reg(info, regAdd, 3);

		regAdd[0] = 0xb1;
		regAdd[1] = (noise_param->pAddr[i] >> 8) & 0xff;
		regAdd[2] = noise_param->pAddr[i] & 0xff;
		rc = fts_read_reg(info, regAdd, 3, &buf[0], 3);

		noise_param->pData[i] = buf[1]+(buf[2]<<8);
	}

	for (i = 0; i < MAX_NOISE_PARAM; i++) {
		tsp_debug_dbg(&info->client->dev,
				"Get Noise Param%d Address "
				"[ 0x%04x ] = 0x%04x\n", i,
				noise_param->pAddr[i],
				noise_param->pData[i]);
	}

	return rc;
}

static int fts_set_noise_param(struct fts_ts_info *info)
{
	int i;
	unsigned char regAdd[5];
	struct fts_noise_param *noise_param;

	noise_param = (struct fts_noise_param *)&info->noise_param;

	for (i = 0; i < MAX_NOISE_PARAM; i++) {
		regAdd[0] = 0xb3;
		regAdd[1] = 0x00;
		regAdd[2] = 0x10;
		fts_write_reg(info, regAdd, 3);

		regAdd[0] = 0xb1;
		regAdd[1] = (noise_param->pAddr[i] >> 8) & 0xff;
		regAdd[2] = noise_param->pAddr[i] & 0xff;
		regAdd[3] = noise_param->pData[i] & 0xff;
		regAdd[4] = (noise_param->pData[i] >> 8) & 0xff;
		fts_write_reg(info, regAdd, 5);
	}

	for (i = 0; i < MAX_NOISE_PARAM; i++) {
		tsp_debug_dbg(&info->client->dev,
			"Set Noise Param%d Address "
			"[ 0x%04x ] = 0x%04x\n",
			i,
			noise_param->pAddr[i],
			noise_param->pData[i]);
	}

	return 0;
}
#endif

int fts_cmd_completion_check(struct fts_ts_info *info, uint8_t event1, uint8_t event2, uint8_t event3)
{
	unsigned char val[8];
	unsigned char reg[2] = {0x85, 0};
	int retry = 100;
	int rc = 0;

	while (retry--) {
		fts_delay(10);
		info->fts_read_reg(info, &reg[0], 1, &val[0], FTS_EVENT_SIZE);
		if ((val[0] == event1) && (val[1] == event2) && (val[2] == event3)) {
			tsp_debug_info(&info->client->dev,
						"\n\r[fts_cmd_completion_check] OK [%02x][%02x][%02x]", val[0], val[1], val[2]);
			return rc;
		} else if (val[0] == 0x0F) {
			tsp_debug_err(&info->client->dev,
						"\n\r[fts_cmd_completion_check] Error - [%02x][%02x][%02x]", val[0], val[1], val[2]);
		}
	}

	rc = -1;
	if (retry <= 0)
		tsp_debug_err(&info->client->dev,
						"\n\r[fts_cmd_completion_check] Error - Time Over [%02x][%02x][%02x]", event1, event2, event3);
	return rc;
}

static int fts_init(struct fts_ts_info *info)
{
	unsigned char val[16];
	unsigned char regAdd[8];
	int rc = 0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_irq_enable(info, false);

	rc = fts_systemreset(info);
	if (rc < 0) {
		tsp_debug_err(&info->client->dev, "%s: Failed to system reset(rc = %d)\n",
				__func__, rc);
		return rc;
	}

	rc = fts_wait_for_ready(info);
	if (rc == -FTS_ERROR_EVENT_ID) {
		info->fw_version_of_ic = 0;
		info->config_version_of_ic = 0;
		info->fw_main_version_of_ic = 0;
	} else {
		fts_get_version_info(info);
	}

	rc = fts_read_chip_id(info);
	if (rc < 0) {
		tsp_debug_err(&info->client->dev, "%s: Failed to fts_read_chip_id\n",
					__func__);
		return rc;
	}
/*
	rc  = fts_fw_update(info);
	if (rc  < 0)
		tsp_debug_err(&info->client->dev, "%s: Failed to firmware update\n",
				__func__);
*/

#ifdef FEATURE_FTS_PRODUCTION_CODE
		info->digital_rev = FTS_DIGITAL_REV_2;
		rc = info->fts_get_channel_info(info);
		if (rc == 0) {
			tsp_debug_dbg(&info->client->dev,
						"FTS Sense(%02d) Force(%02d)\n",
						info->SenseChannelLength,
						info->ForceChannelLength);
		} else {
			tsp_debug_err(&info->client->dev,
						"FTS read failed rc = %d\n", rc);
			tsp_debug_err(&info->client->dev,
						"FTS Initialise Failed\n");
			return rc;
		}
		info->pFrame =
			kzalloc(info->SenseChannelLength * info->ForceChannelLength * 2,
				GFP_KERNEL);
		if (info->pFrame == NULL) {
			tsp_debug_err(&info->client->dev,
						"FTS pFrame kzalloc Failed\n");
			return -ENOMEM;
		}
		info->cx_data = kzalloc(info->SenseChannelLength *
						info->ForceChannelLength, GFP_KERNEL);
		if (!info->cx_data)
			tsp_debug_err(&info->client->dev,
					"%s: cx_data kzalloc Failed\n", __func__);
#endif
	fts_command(info, SENSEON);

#ifdef FTS_SUPPORT_NOISE_PARAM
	fts_get_noise_param_address(info);
#endif
	/* fts driver set functional feature */
	info->touch_count = 0;
	info->palm_pressed = false;
	info->flip_enable = false;
	info->mainscr_disable = false;

	info->deepsleep_mode = false;
	info->wirelesscharger_mode = false;
	info->lowpower_mode = false;
	info->lowpower_flag = 0x00;
	info->fts_power_state = FTS_POWER_STATE_ACTIVE;

	fts_command(info, FORCECALIBRATION);

	fts_irq_enable(info, true);
	fts_interrupt_set(info, INT_ENABLE);

	memset(val, 0x0, 4);
	regAdd[0] = READ_STATUS;
	fts_read_reg(info, regAdd, 1, (unsigned char *)val, 4);
	tsp_debug_dbg(&info->client->dev,
				"FTS ReadStatus(0x84) : "
				"%02X %02X %02X %02X\n",
				val[0], val[1], val[2], val[3]);

	tsp_debug_info(&info->client->dev,
				"FTS Initialized\n");

	return 0;
}

static void fts_debug_msg_event_handler(struct fts_ts_info *info,
				      unsigned char data[])
{
	tsp_debug_info(&info->client->dev,
	       "%s: %02X %02X %02X %02X "
	       "%02X %02X %02X %02X\n", __func__,
	       data[0], data[1], data[2], data[3],
	       data[4], data[5], data[6], data[7]);
}

static unsigned char fts_event_handler_type_b(struct fts_ts_info *info,
					      unsigned char data[],
					      unsigned char LeftEvent)
{
	unsigned char EventNum = 0;
	unsigned char NumTouches = 0;
	unsigned char TouchID = 0, EventID = 0, status = 0;
	unsigned char LastLeftEvent = 0;
	int x = 0, y = 0, z = 0;
	int bw = 0, bh = 0, palm = 0;
	int orient = 0;

#if defined(CONFIG_INPUT_BOOSTER)
	bool booster_restart = false;
#endif

	for (EventNum = 0; EventNum < LeftEvent; EventNum++) {
#ifdef DEBUG
		tsp_debug_dbg(&info->client->dev,
			"%d %2x %2x %2x %2x %2x %2x %2x %2x\n",
			EventNum,
			data[EventNum * FTS_EVENT_SIZE],
			data[EventNum * FTS_EVENT_SIZE+1],
			data[EventNum * FTS_EVENT_SIZE+2],
			data[EventNum * FTS_EVENT_SIZE+3],
			data[EventNum * FTS_EVENT_SIZE+4],
			data[EventNum * FTS_EVENT_SIZE+5],
			data[EventNum * FTS_EVENT_SIZE+6],
			data[EventNum * FTS_EVENT_SIZE+7]);
		tsp_debug_dbg(&info->client->dev,
			"fts_power_state (%d)\n",
			info->fts_power_state );
#endif

	if (info->fts_power_state == FTS_POWER_STATE_LOWPOWER)
		EventID = data[EventNum * FTS_EVENT_SIZE] & 0xFF;
	else
		EventID = data[EventNum * FTS_EVENT_SIZE] & 0x0F;

		if ((EventID >= 3) && (EventID <= 5)) {
			LastLeftEvent = 0;
			NumTouches = 1;
			TouchID = (data[EventNum * FTS_EVENT_SIZE] >> 4) & 0x0F;
		} else {
			LastLeftEvent =
			    data[7 + EventNum * FTS_EVENT_SIZE] & 0x0F;
			NumTouches =
			    (data[1 + EventNum * FTS_EVENT_SIZE] & 0xF0) >> 4;
			TouchID = data[1 + EventNum * FTS_EVENT_SIZE] & 0x0F;
			EventID = data[EventNum * FTS_EVENT_SIZE] & 0xFF;
			status = data[1 + EventNum * FTS_EVENT_SIZE] & 0xFF;
		}

		switch (EventID) {
		case EVENTID_NO_EVENT:
			break;

		case EVENTID_ERROR:
			if (data[1 + EventNum *
					FTS_EVENT_SIZE] == 0x08) {
				/* Get Auto tune fail event */
				if (data[2 + EventNum *
						FTS_EVENT_SIZE] == 0x00) {
					tsp_debug_err(&info->client->dev,
							"[FTS] Fail Mutual Auto tune\n");
				} else if (data[2 + EventNum *
							FTS_EVENT_SIZE] == 0x01) {
					tsp_debug_err(&info->client->dev,
							"[FTS] Fail Self Auto tune\n");
				}
			} else if (data[1 + EventNum *
							FTS_EVENT_SIZE] == 0x09)
				/*  Get detect SYNC fail event */
				tsp_debug_err(&info->client->dev,
						"[FTS] Fail detect SYNC\n");
			break;

		case EVENTID_HOVER_ENTER_POINTER:
		case EVENTID_HOVER_MOTION_POINTER:
			x = ((data[4 + EventNum * FTS_EVENT_SIZE] & 0xF0) >> 4)
			    | ((data[2 + EventNum * FTS_EVENT_SIZE]) << 4);
			y = ((data[4 + EventNum * FTS_EVENT_SIZE] & 0x0F) |
			     ((data[3 + EventNum * FTS_EVENT_SIZE]) << 4));

			z = data[5 + EventNum * FTS_EVENT_SIZE];

			input_mt_slot(info->input_dev, 0);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, 1);

			input_report_key(info->input_dev, BTN_TOUCH, 0);
			input_report_key(info->input_dev, BTN_TOOL_FINGER, 1);

			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev, ABS_MT_DISTANCE, 255 - z);
			break;

		case EVENTID_HOVER_LEAVE_POINTER:
			input_mt_slot(info->input_dev, 0);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, 0);
			break;

		case EVENTID_ENTER_POINTER:
			if (info->fts_power_state == FTS_POWER_STATE_LOWPOWER)
				break;

			info->touch_count++;
#if defined(CONFIG_INPUT_BOOSTER)
			booster_restart = true;
#endif
		case EVENTID_MOTION_POINTER:
			if (info->fts_power_state == FTS_POWER_STATE_LOWPOWER) {
				tsp_debug_info(&info->client->dev,
						"%s: low power mode\n", __func__);
				fts_release_all_finger(info);
				break;
			}

			if (info->touch_count == 0) {
				tsp_debug_info(&info->client->dev,
						"%s: count 0\n", __func__);
				fts_release_all_finger(info);
				break;
			}

			if ((EventID == EVENTID_MOTION_POINTER) &&
				(info->finger[TouchID].state ==
						EVENTID_LEAVE_POINTER)) {
				tsp_debug_info(&info->client->dev,
						"%s: state leave but point is moved.\n", __func__);
				break;
			}

			if (info->fts_power_state == FTS_POWER_STATE_LOWPOWER)
				break;

			x = ((data[1 + EventNum * FTS_EVENT_SIZE]
						& 0xFF) << 4) +
						((data[3 + EventNum * FTS_EVENT_SIZE]
								& 0xF0) >> 4);
			y = ((data[2 + EventNum * FTS_EVENT_SIZE]
						& 0xFF) << 4) +
				(data[3 + EventNum * FTS_EVENT_SIZE]
						& 0xF);

			z = data[4 + EventNum * FTS_EVENT_SIZE];

			bw = (data[6 + EventNum * FTS_EVENT_SIZE] << 2)
				| ((data[7 + EventNum * FTS_EVENT_SIZE] >> 6)
						& 0x03);

			bh = (data[7 + EventNum * FTS_EVENT_SIZE] & 0x3F)
				* bw / 63;

			orient = (s8)data[5 + EventNum * FTS_EVENT_SIZE];

			if (z == 255) {
				tsp_debug_info(&info->client->dev,
						"%s: Palm Detected\n", __func__);
				tsp_debug_event(&info->client->dev, "%s: "
						"[ID:%2d  X:%4d  Y:%4d  Z:%4d "
						" WM:%4d  Wm:%4d  Orient:%2d  "
						"tc:%2d]\n", __func__,
						TouchID, x, y, z,
						max(bw, bh), min(bw, bh),
						orient, info->touch_count);
				info->palm_pressed = true;
				fts_release_all_finger(info);
				return 0;
			}

			input_mt_slot(info->input_dev, TouchID);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER,
						   1 + (palm << 1));

			input_report_key(info->input_dev, BTN_TOUCH, 1);
			input_report_key(info->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(info->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev,
					ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev,
					ABS_MT_TOUCH_MAJOR, bw);
			input_report_abs(info->input_dev,
					ABS_MT_TOUCH_MINOR, bh);
			input_report_abs(info->input_dev,
					ABS_MT_PRESSURE, z);
			input_report_abs(info->input_dev,
					ABS_MT_ORIENTATION, orient);
			info->finger[TouchID].lx = x;
			info->finger[TouchID].ly = y;

			break;

		case EVENTID_LEAVE_POINTER:
			if (info->fts_power_state == FTS_POWER_STATE_LOWPOWER)
				break;

			if (info->palm_pressed) {
				tsp_debug_event(&info->client->dev,
						"%s: Palm Released\n",
						__func__);
				info->palm_pressed = false;
				return 0;
			}

			if (info->touch_count <= 0) {
				tsp_debug_info(&info->client->dev,
						"%s: count 0\n", __func__);
				fts_release_all_finger(info);
				break;
			}

			info->touch_count--;

			input_mt_slot(info->input_dev, TouchID);

			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, 0);

			if (info->touch_count == 0) {
				/* Clear BTN_TOUCH when All touch are released  */
				input_report_key(info->input_dev, BTN_TOUCH, 0);
				input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

			}
			break;
		case EVENTID_STATUS_EVENT:
			if (status == STATUS_EVENT_GLOVE_MODE) {

			} else if (status == STATUS_EVENT_RAW_DATA_READY) {
				unsigned char regAdd[4] = {0xB0, 0x01, 0x29, 0x01};

				fts_write_reg(info, &regAdd[0], 4);

				tsp_debug_info(&info->client->dev,
						"[FTS] Received the Raw Data Ready Event\n");
			} else if (status == STATUS_EVENT_FORCE_CAL_MUTUAL) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received Force Calibration Mutual only Event\n");
			} else if (status == STATUS_EVENT_FORCE_CAL_SELF) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received Force Calibration Self only Event\n");
			} else if (status == STATUS_EVENT_WATERMODE_ON) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received Water Mode On Event\n");
			} else if (status == STATUS_EVENT_WATERMODE_OFF) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received Water Mode Off Event\n");
			} else if (status == STATUS_EVENT_MUTUAL_CAL_FRAME_CHECK) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received Mutual Calib Frame Check Event\n");
			} else if (status == STATUS_EVENT_SELF_CAL_FRAME_CHECK) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received Self Calib Frame Check Event\n");
			} else if (status == FTS_EVENT_REBOOT_BY_ESD) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received ESD detected Event need to Reset\n");
				schedule_delayed_work(&info->reset_work,
					msecs_to_jiffies(10));
			} else if (status == FTS_EVENT_VR_MODE_ENABLED) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received VR Mode Enabled Event\n");
				info->vr_mode = 1;
			} else if (status == FTS_EVENT_VR_MODE_DISABLED) {
				tsp_debug_info(&info->client->dev,
						"[FTS] Received VR Mode Disabled Event\n");
				info->vr_mode = 0;
			} else {
				fts_debug_msg_event_handler(info,
						  &data[EventNum *
							FTS_EVENT_SIZE]);
			}
			break;

#ifdef FEATURE_FTS_PRODUCTION_CODE
		case EVENTID_RESULT_READ_REGISTER:
			procedure_cmd_event(info, &data[EventNum * FTS_EVENT_SIZE]);
			break;
#endif /* FEATURE_FTS_PRODUCTION_CODE */

		default:
			fts_debug_msg_event_handler(info,
						  &data[EventNum *
							FTS_EVENT_SIZE]);
			continue;
		}

		if (EventID == EVENTID_ENTER_POINTER)
			tsp_debug_event(&info->client->dev,
				"[P] tID:%d x:%d y:%d w:%d "
				"h:%d z:%d p:%d tc:%d tm:%d\n",
				TouchID, x, y, bw,
				bh, z, palm, info->touch_count, info->touch_mode);
		else if (EventID == EVENTID_HOVER_ENTER_POINTER)
			tsp_debug_event(&info->client->dev,
				"[HP] tID:%d x:%d y:%d z:%d\n",
				TouchID, x, y, z);

		else if (EventID == EVENTID_LEAVE_POINTER) {
			tsp_debug_event(&info->client->dev,
				"[R] tID:%d mc: %d tc:%d lx: %d ly: %d "
				"Ver[%02X%04X%01X%01X]\n",
				TouchID, info->finger[TouchID].mcount, info->touch_count,
				info->finger[TouchID].lx,
				info->finger[TouchID].ly,
				info->panel_revision,
				info->fw_main_version_of_ic,
				info->flip_enable,
				info->mainscr_disable);

			info->finger[TouchID].mcount = 0;
		} else if (EventID == EVENTID_HOVER_LEAVE_POINTER) {
			tsp_debug_event(&info->client->dev,
				"[HR] tID:%d Ver[%02X%04X%01X]\n",
				TouchID,
				info->panel_revision,
				info->fw_main_version_of_ic,
				info->flip_enable);
			info->finger[TouchID].mcount = 0;
		} else if (EventID == EVENTID_MOTION_POINTER)
			info->finger[TouchID].mcount++;

		if ((EventID == EVENTID_ENTER_POINTER) ||
			(EventID == EVENTID_MOTION_POINTER) ||
			(EventID == EVENTID_LEAVE_POINTER))
			info->finger[TouchID].state = EventID;

		input_sync(info->input_dev);
	}


#if defined(CONFIG_INPUT_BOOSTER)
	if ((EventID == EVENTID_ENTER_POINTER)
			|| (EventID == EVENTID_LEAVE_POINTER)) {
		if (booster_restart)
			input_booster_send_event(BOOSTER_DEVICE_TOUCH, BOOSTER_MODE_ON);
		if (!info->touch_count)
			input_booster_send_event(BOOSTER_DEVICE_TOUCH, BOOSTER_MODE_OFF);
	}
#endif

	return LastLeftEvent;
}

#ifdef FTS_SUPPORT_TA_MODE
static void fts_ta_cb(struct fts_callbacks *cb, int ta_status)
{
	struct fts_ts_info *info =
	    container_of(cb, struct fts_ts_info, callbacks);

	if (ta_status == 0x01 || ta_status == 0x03) {
		fts_command(info, FTS_CMD_CHARGER_PLUGGED);
		info->TA_Pluged = true;
		tsp_debug_dbg(&info->client->dev,
			 "%s: device_control : CHARGER CONNECTED, ta_status : %x\n",
			 __func__, ta_status);
	} else {
		fts_command(info, FTS_CMD_CHARGER_UNPLUGGED);
		info->TA_Pluged = false;
		tsp_debug_dbg(&info->client->dev,
			 "%s: device_control : CHARGER DISCONNECTED, ta_status : %x\n",
			 __func__, ta_status);
	}
}
#endif

/**
 * fts_interrupt_handler()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t fts_interrupt_handler(int irq, void *handle)
{
	struct fts_ts_info *info = handle;
	unsigned char regAdd[4] = {0xB6, 0x00, 0x23, READ_ALL_EVENT};
	unsigned short evtcount = 0;

	evtcount = 0;

	fts_read_reg(info, &regAdd[0], 3, (unsigned char *)&evtcount, 2);
	evtcount = evtcount >> 8;
	evtcount = evtcount / 2;

	if (evtcount > FTS_FIFO_MAX)
		evtcount = FTS_FIFO_MAX;

	if (evtcount > 0) {
		memset(info->data, 0x0, FTS_EVENT_SIZE * evtcount);
		fts_read_reg(info, &regAdd[3], 1, (unsigned char *)info->data,
				  FTS_EVENT_SIZE * evtcount);
		fts_event_handler_type_b(info, info->data, evtcount);
	}
	return IRQ_HANDLED;
}

static void fts_irq_enable(struct fts_ts_info *info,
		bool enable)
{
	spin_lock(&info->lock);

	if (enable) {
		if (atomic_cmpxchg(&info->irq_enabled, 0, 1) == 0) {
			tsp_debug_dbg(info->dev, "enable_irq\n");
			enable_irq(info->irq);

			if (device_may_wakeup(&info->client->dev))
				enable_irq_wake(info->irq);
		}
	} else {
		if (atomic_cmpxchg(&info->irq_enabled, 1, 0) == 1) {
			tsp_debug_dbg(info->dev, "disable_irq\n");
			if (device_may_wakeup(&info->client->dev))
				disable_irq_wake(info->irq);

			disable_irq_nosync(info->irq);
		}
	}

	spin_unlock(&info->lock);
}

#ifdef CONFIG_OF
#ifdef FTS_SUPPORT_TA_MODE
struct fts_callbacks *fts_charger_callbacks;
void tsp_charger_infom(bool en)
{
	pr_err("[TSP]%s: ta:%d\n",	__func__, en);

	if (fts_charger_callbacks && fts_charger_callbacks->inform_charger)
		fts_charger_callbacks->inform_charger(fts_charger_callbacks, en);
}
static void fts_tsp_register_callback(void *cb)
{
	fts_charger_callbacks = cb;
}
#endif
static int fts_power_ctrl(void *data, bool on)
{
	struct fts_ts_info *info = (struct fts_ts_info *)data;
	const struct fts_i2c_platform_data *pdata = info->board;
	struct device *dev = &info->client->dev;
	struct regulator *regulator_dvdd = NULL;
	struct regulator *regulator_avdd = NULL;
	static bool enabled;
	int retval = 0;

	if (enabled == on)
		return retval;

	/* touch power init */
	if (gpio_is_valid(pdata->vdd_gpio)) {
		gpio_request(pdata->vdd_gpio, "touch-vdd");
	} else {
		regulator_avdd = regulator_get(NULL, pdata->regulator_avdd);
		if (IS_ERR_OR_NULL(regulator_avdd)) {
			tsp_debug_err(dev, "%s: "
					"Failed to get %s regulator.\n",
					__func__, pdata->regulator_avdd);
			goto out;
		}
	}
	if (gpio_is_valid(pdata->vio_gpio)) {
		gpio_request(pdata->vio_gpio, "touch-vio");
	} else {
		regulator_dvdd = regulator_get(NULL, pdata->regulator_dvdd);
		if (IS_ERR_OR_NULL(regulator_dvdd)) {
			tsp_debug_err(dev, "%s: "
					"Failed to get %s regulator.\n",
					__func__, pdata->regulator_dvdd);
			goto out;
		}
	}

	tsp_debug_info(dev, "%s: %s\n", __func__, on ? "on" : "off");

	if (on) {
		if (gpio_is_valid(pdata->vdd_gpio)) {
			retval = gpio_direction_output(pdata->vdd_gpio, 1);
			if (retval) {
				tsp_debug_err(dev, "%s: "
						"Failed to enable vdd: %d\n",
						__func__, retval);
			}
		} else if (!IS_ERR_OR_NULL(regulator_avdd)) {
			retval = regulator_enable(regulator_avdd);
			if (retval) {
				tsp_debug_err(dev, "%s: "
						"Failed to enable avdd: %d\n",
						__func__, retval);
				goto out;
			}
		}

		if (gpio_is_valid(pdata->vio_gpio)) {
			retval = gpio_direction_output(pdata->vio_gpio, 1);
			if (retval) {
				tsp_debug_err(dev, "%s: "
						"Failed to enable vio: %d\n",
						__func__, retval);
			}
		} else if (!IS_ERR_OR_NULL(regulator_dvdd)) {
			retval = regulator_enable(regulator_dvdd);
			if (retval) {
				tsp_debug_err(dev, "%s: "
					"Failed to enable dvdd: %d\n",
					__func__, retval);
				goto out;
			}
		}

		retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_default);
		if (retval < 0)
			tsp_debug_err(dev, "%s: "
					"Failed to configure tsp_attn pin\n",
					__func__);

		fts_delay(5);
	} else {
		if (gpio_is_valid(pdata->vio_gpio)) {
			retval = gpio_direction_output(pdata->vio_gpio, 0);
			if (retval) {
				tsp_debug_err(dev, "%s: "
						"Failed to enable vio: %d\n",
						__func__, retval);
			}
		} else if (!IS_ERR_OR_NULL(regulator_dvdd)) {
			retval = regulator_disable(regulator_dvdd);
			if (retval) {
				tsp_debug_err(dev, "%s: "
						"Failed to enable dvdd: %d\n",
						__func__, retval);
				goto out;
			}
		}

		if (gpio_is_valid(pdata->vdd_gpio)) {
			retval = gpio_direction_output(pdata->vdd_gpio, 0);
			if (retval)
				tsp_debug_err(dev, "%s: "
						"Failed to enable vdd: %d\n",
						__func__, retval);
		} else if (!IS_ERR_OR_NULL(regulator_avdd)) {
			retval = regulator_disable(regulator_avdd);
			if (retval) {
				tsp_debug_err(dev, "%s: "
						"Failed to enable avdd: %d\n",
						__func__, retval);
				goto out;
			}
		}

		retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_sleep);
		if (retval < 0)
			tsp_debug_err(dev, "%s: Failed to "
					"configure tsp_attn pin\n", __func__);
	}

	enabled = on;

out:
	regulator_put(regulator_dvdd);
	regulator_put(regulator_avdd);

	return retval;
}

static int fts_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fts_i2c_platform_data *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;
	u32 coords[2], lines[2];
	int retval = 0;

	pdata->tspid = of_get_named_gpio(np, "stm,tspid_gpio", 0);
	if (gpio_is_valid(pdata->tspid))
		tsp_debug_dbg(dev,
				"TSP_ID : %d\n",
				gpio_get_value(pdata->tspid));
	else
		tsp_debug_err(dev,
				"Failed to get tspid gpio\n");

	pdata->tspid2 = of_get_named_gpio(np, "stm,tspid2_gpio", 0);
	if (gpio_is_valid(pdata->tspid2))
		tsp_debug_dbg(dev,
				"TSP_ID2 : %d\n",
				gpio_get_value(pdata->tspid2));
	else
		tsp_debug_err(dev,
				"Failed to get tspid2 gpio\n");

	pdata->gpio = of_get_named_gpio(np, "stm,irq_gpio", 0);
	if (gpio_is_valid(pdata->gpio)) {
		retval = gpio_request_one(pdata->gpio,
					GPIOF_DIR_IN, "stm,tsp_int");
		if (retval) {
			tsp_debug_err(dev,
						"Unable to request tsp_int [%d]\n",
						pdata->gpio);
			return -EINVAL;
		}
	} else {
		tsp_debug_err(dev,
				"Failed to get irq gpio\n");
		return -EINVAL;
	}

	tsp_debug_info(dev, "irq_gpio = %d\n", pdata->gpio);
	client->irq = of_irq_get_byname(np, "tp_direct_interrupt");
	tsp_debug_info(dev, "client->irq = %d\n", client->irq);

	if (of_property_read_u32(np, "stm,irq_type", &pdata->irq_type)) {
		tsp_debug_err(dev, "Failed to get irq_type property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "stm,grip_area", &pdata->grip_area))
		tsp_debug_err(dev, "Failed to get grip_area property\n");

	if (of_property_read_u32_array(np, "stm,max_coords", coords, 2)) {
		tsp_debug_err(dev, "Failed to get max_coords property\n");
		return -EINVAL;
	}
	pdata->max_x = coords[0];
	pdata->max_y = coords[1];

	if (of_property_read_u32_array(np, "stm,num_lines", lines, 2))
		tsp_debug_dbg(dev, "skipped to get num_lines property\n");
	else {
		pdata->SenseChannelLength = lines[0];
		pdata->ForceChannelLength = lines[1];
		tsp_debug_dbg(dev, "num_of[rx,tx]: [%d,%d]\n",
			pdata->SenseChannelLength, pdata->ForceChannelLength);
	}

	if (of_property_read_string(np, "stm,regulator_dvdd", &pdata->regulator_dvdd)) {
		tsp_debug_err(dev,
				"Failed to get regulator_dvdd name property\n");
	}
	if (of_property_read_string(np, "stm,regulator_avdd", &pdata->regulator_avdd)) {
		tsp_debug_err(dev,
				"Failed to get regulator_avdd name property\n");
	}

	pdata->vdd_gpio = of_get_named_gpio(np, "stm,vdd-gpio", 0);
	if (gpio_is_valid(pdata->vdd_gpio))
		tsp_debug_dbg(dev, "vdd_gpio : %d\n",
				gpio_get_value(pdata->vdd_gpio));
	else
		tsp_debug_err(dev, "Failed to get vdd_gpio gpio\n");

	pdata->vio_gpio = of_get_named_gpio(np, "stm,vio-gpio", 0);
	if (gpio_is_valid(pdata->vio_gpio))
		tsp_debug_dbg(dev, "vio_gpio :"
				"%d\n", gpio_get_value(pdata->vio_gpio));
	else
		tsp_debug_err(dev, "Failed to get vio_gpio gpio\n");

	pdata->power = fts_power_ctrl;

	pdata->reset_pin = of_get_named_gpio(np, "stm,reset-gpio", 0);
	if (gpio_is_valid(pdata->reset_pin)) {
		if (devm_gpio_request_one(&client->dev, pdata->reset_pin,
					GPIOF_OUT_INIT_LOW, "reset_pin")) {
			tsp_debug_err(dev, "Failed to request gpio reset_pin\n");
			pdata->reset_pin = -1;
		} else {
			tsp_debug_dbg(dev, "reset_pin : %d\n",
					gpio_get_value(pdata->reset_pin));
		}
	} else {
		tsp_debug_err(dev, "Failed to get reset_pin gpio\n");
	}

	pdata->ta_detect_pin = of_get_named_gpio(np, "stm,ta_detect_gpio", 0);
	if (gpio_is_valid(pdata->ta_detect_pin)) {
		if (devm_gpio_request_one(&client->dev, pdata->ta_detect_pin,
					GPIOF_OUT_INIT_LOW, "ta_detect_pin")) {
			tsp_debug_err(dev, "Failed to request gpio ta_detect_pin\n");
			pdata->ta_detect_pin = -1;
		} else {
			tsp_debug_dbg(dev, "ta_detect_pin : %d\n",
				      gpio_get_value(pdata->ta_detect_pin));
		}
	} else {
		tsp_debug_err(dev, "Failed to get ta_detect_pin gpio\n");
	}

	/* Optional parmeters(those values are not mandatory)
	 * do not return error value even if fail to get the value
	 */
	of_property_read_string(np, "stm,firmware_name", &pdata->firmware_name);

	if (of_property_read_string_index(np, "stm,project_name", 0, &pdata->project_name))
		tsp_debug_dbg(dev,
				"skipped to get project_name property\n");
	if (of_property_read_string_index(np, "stm,project_name", 1, &pdata->model_name))
		tsp_debug_dbg(dev,
				"skipped to get model_name property\n");

	pdata->max_width = 28;
	pdata->support_hover = true;
	pdata->support_mshover = true;
#ifdef FTS_SUPPORT_TA_MODE
	pdata->register_cb = fts_tsp_register_callback;
#endif

	return retval;
}
#endif

static int fts_setup_drv_data(struct i2c_client *client)
{
	int retval = 0;
	struct fts_i2c_platform_data *pdata;
	struct fts_ts_info *info;

	/* parse dt */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct fts_i2c_platform_data), GFP_KERNEL);

		if (!pdata) {
			tsp_debug_err(&client->dev, "Failed to allocate platform data\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
		retval = fts_parse_dt(client);
		if (retval) {
			tsp_debug_err(&client->dev, "Failed to parse dt\n");
			return retval;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		tsp_debug_err(&client->dev, "No platform data found\n");
			return -EINVAL;
	}
	if (!pdata->power) {
		tsp_debug_err(&client->dev, "No power contorl found\n");
			return -EINVAL;
	}

	pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pdata->pinctrl)) {
		tsp_debug_err(&client->dev, "could not get pinctrl\n");
		return PTR_ERR(pdata->pinctrl);
	}

	pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl, "on_state");
	if (IS_ERR(pdata->pins_default))
		tsp_debug_err(&client->dev, "could not get default pinstate\n");

	pdata->pins_sleep = pinctrl_lookup_state(pdata->pinctrl, "off_state");
	if (IS_ERR(pdata->pins_sleep))
		tsp_debug_err(&client->dev, "could not get sleep pinstate\n");

	info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		tsp_debug_err(&client->dev,
				"%s: Failed to alloc mem for info\n",
				__func__);
		return -ENOMEM;
	}

	info->client = client;
	info->board = pdata;
	info->irq = client->irq;
	info->irq_type = info->board->irq_type;
	atomic_set(&info->irq_enabled, 0);
	info->charger_connected = 0;
	info->touch_stopped = false;
	info->panel_revision = info->board->panel_revision;
	info->stop_device = fts_stop_device;
	info->start_device = fts_start_device;
	info->fts_command = fts_command;
	info->fts_read_reg = fts_read_reg;
	info->fts_write_reg = fts_write_reg;
	info->fts_systemreset = fts_systemreset;
	info->fts_get_version_info = fts_get_version_info;
	info->fts_wait_for_ready = fts_wait_for_ready;
#ifdef FEATURE_FTS_PRODUCTION_CODE
	info->fts_get_channel_info = fts_get_channel_info;
	info->fts_interrupt_set = fts_interrupt_set;
	info->fts_irq_enable = fts_irq_enable;
	info->fts_release_all_finger = fts_release_all_finger;
#endif

#ifdef FTS_SUPPORT_NOISE_PARAM
	info->fts_get_noise_param_address = fts_get_noise_param_address;
#endif

#ifdef USE_OPEN_DWORK
	INIT_DELAYED_WORK(&info->open_work, fts_open_work);
#endif
	info->delay_time = 300;
	INIT_DELAYED_WORK(&info->reset_work, fts_reset_work);

	if (info->board->support_hover)
		tsp_debug_info(&info->client->dev, "FTS Support Hover Event\n");
	else
		tsp_debug_info(&info->client->dev, "FTS Not support Hover Event\n");

	i2c_set_clientdata(client, info);

	if (pdata->get_ddi_type) {
		info->ddi_type = pdata->get_ddi_type();
		tsp_debug_info(&client->dev,
			"%s: DDI Type is %s[%d]\n",
			__func__, info->ddi_type ?
			"MAGNA" : "SDC", info->ddi_type);
	}

	info->switch_gpio = of_get_named_gpio(client->dev.of_node,
					      "stm,switch_gpio", 0);
	tsp_debug_info(&client->dev, "switch_gpio = %d\n", info->switch_gpio);

	if (!gpio_is_valid(info->switch_gpio)) {
		tsp_debug_err(&client->dev, "Failed to get switch gpio\n");
		return -EINVAL;
	}

	retval = gpio_request_one(info->switch_gpio,
				  GPIOF_OUT_INIT_LOW,
				  "stm,tsp_i2c_switch");
	if (retval) {
		tsp_debug_err(&client->dev,
			      "Unable to request tsp_i2c_switch [%d]\n",
			      info->switch_gpio);
		return -EINVAL;
	}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	tui_tsp_info = info;
#endif
	return retval;
}

static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	int retval = 0;
	struct fts_ts_info *info = NULL;
	static char fts_ts_phys[64] = { 0 };
	struct power_supply_config psy_cfg = {};
	int i = 0;

/*
	tsp_debug_info(&client->dev, "FTS Driver [12%s] %s %s\n",
	       FTS_TS_DRV_VERSION, __DATE__, __TIME__);
*/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		tsp_debug_err(&client->dev, "FTS err = EIO!\n");
		return -EIO;
	}

	/* Build up driver data */
	retval = fts_setup_drv_data(client);
	if (retval < 0) {
		tsp_debug_err(&client->dev,
				"%s: Failed to set up driver data\n",
				__func__);
		goto err_setup_drv_data;
	}

	info = (struct fts_ts_info *)i2c_get_clientdata(client);
	if (!info) {
		tsp_debug_err(&client->dev,
				"%s: Failed to get driver data\n", __func__);
		retval = -ENODEV;
		goto err_get_drv_data;
	}

	if (info->board->power) {
		info->board->power(info, true);
		fts_delay(15);

		if (gpio_is_valid(info->board->reset_pin))
			gpio_set_value(info->board->reset_pin, 1);
	}

	info->dev = &info->client->dev;
	info->input_dev = input_allocate_device();
	if (!info->input_dev) {
		tsp_debug_err(&info->client->dev, "FTS err = ENOMEM!\n");
		retval = -ENOMEM;
		goto err_input_allocate_device;
	}

	info->input_dev->dev.parent = &client->dev;
	info->input_dev->name = "touchscreen";
	snprintf(fts_ts_phys, sizeof(fts_ts_phys), "%s/input1",
		 info->input_dev->name);
	info->input_dev->phys = fts_ts_phys;
	info->input_dev->id.bustype = BUS_I2C;

#ifdef USE_OPEN_CLOSE
	info->input_dev->open = fts_input_open;
	info->input_dev->close = fts_input_close;
#endif

	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);
	set_bit(EV_ABS, info->input_dev->evbit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);
#endif
	set_bit(BTN_TOUCH, info->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, info->input_dev->keybit);

	input_mt_init_slots(info->input_dev, FINGER_MAX, INPUT_MT_DIRECT);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			     0, info->board->max_x, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			     0, info->board->max_y, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);

	mutex_init(&info->device_mutex);
	mutex_init(&info->i2c_mutex);
	spin_lock_init(&info->lock);

	info->enabled = false;
	mutex_lock(&info->device_mutex);
	retval = fts_init(info);
	info->reinit_done = true;
	mutex_unlock(&info->device_mutex);
	if (info->flash_corruption_info.fw_broken ||
	    info->flash_corruption_info.cfg_broken ||
	    info->flash_corruption_info.cx_broken) {
		tsp_debug_err(&info->client->dev,
			      "Attempt to recover corrupt/missing firmware.");
	} else if (retval < 0) {
		tsp_debug_err(&info->client->dev, "FTS fts_init fail!\n");
		goto err_fts_init;
	}

	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
				 0, 1024, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR,
				 0, 1024, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_DISTANCE,
				 0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_ORIENTATION,
				 -128, 127, 0, 0);

	input_set_drvdata(info->input_dev, info);
	i2c_set_clientdata(client, info);

	retval = input_register_device(info->input_dev);
	if (retval) {
		tsp_debug_err(&info->client->dev, "FTS input_register_device fail!\n");
		goto err_register_input;
	}

	for (i = 0; i < FINGER_MAX; i++) {
		info->finger[i].state = EVENTID_LEAVE_POINTER;
		info->finger[i].mcount = 0;
	}

	info->enabled = true;

	tsp_debug_info(&info->client->dev,
			"installing direct irq on GPIO %d\n",
			info->board->gpio);
	retval = msm_gpio_install_direct_irq(info->board->gpio, 0, 0);
	if (retval) {
		tsp_debug_info(&info->client->dev,
				"%s: Failed to install direct irq, ret = %d\n",
				__func__, retval);
		goto err_enable_irq;
	}

	retval = request_threaded_irq(info->irq, NULL,
			fts_interrupt_handler, info->board->irq_type,
			FTS_TS_DRV_NAME, info);
	if (retval < 0) {
		tsp_debug_err(&info->client->dev,
						"%s: Failed to enable attention interrupt\n",
						__func__);
		goto err_enable_irq;
	}
	atomic_set(&info->irq_enabled, 1);

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	trustedui_set_tsp_irq(info->irq);
	tsp_debug_info(&client->dev, "%s[%d] called!\n",
		__func__, info->irq);
#endif

#ifdef CONFIG_FB
	info->fb_notif.notifier_call = touch_fb_notifier_callback;
	retval = fb_register_client(&info->fb_notif);
	if (retval < 0) {
		tsp_debug_err(&client->dev, "%s: Failed to register fb client\n",
			__func__);
		goto err_fb_client;
	}
#endif


#ifdef FTS_SUPPORT_TA_MODE
	info->register_cb = info->board->register_cb;

	info->callbacks.inform_charger = fts_ta_cb;
	if (info->register_cb)
		info->register_cb(&info->callbacks);
#endif

	INIT_DELAYED_WORK(&info->psy_work, fts_psy_work);
	psy_cfg.of_node = info->dev->of_node;
	psy_cfg.drv_data = info;
	info->ts_psy = devm_power_supply_register(info->dev,
						  &fts_ts_desc, &psy_cfg);
	if (!IS_ERR_OR_NULL(info->ts_psy)) {
		fts_external_power_changed(info->ts_psy);
	} else if (PTR_ERR(info->ts_psy) == -EPROBE_DEFER) {
		schedule_delayed_work(&info->psy_work,
				      msecs_to_jiffies(FTS_REGISTER_PSY_MS));
	} else {
		tsp_debug_err(&client->dev,
			     "%s: Failed to register power supply\n", __func__);
		retval = PTR_ERR(info->ts_psy);
		goto err_power_supply;
	}

#ifdef FEATURE_FTS_PRODUCTION_CODE
	fts_production_init(info);
#endif /* FEATURE_FTS_PRODUCTION_CODE */
	device_init_wakeup(&client->dev, false);
	if (device_may_wakeup(&info->client->dev))
		enable_irq_wake(info->irq);
	info->lowpower_mode = true;

	return 0;

err_power_supply:
#ifdef CONFIG_FB
	fb_unregister_client(&info->fb_notif);

err_fb_client:
#endif
	fts_irq_enable(info, false);
	free_irq(info->irq, info);

err_enable_irq:
	input_unregister_device(info->input_dev);
	info->input_dev = NULL;

err_register_input:
	if (info->input_dev)
		input_free_device(info->input_dev);

err_fts_init:
	mutex_destroy(&info->device_mutex);
	mutex_destroy(&info->i2c_mutex);
err_input_allocate_device:
	info->board->power(info, false);
	kfree(info);
err_get_drv_data:
err_setup_drv_data:
	return retval;
}

static int fts_remove(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	tsp_debug_info(&info->client->dev, "FTS removed\n");

#if defined(CONFIG_FB)
	if (fb_unregister_client(&info->fb_notif))
		tsp_debug_err(&info->client->dev,
			"%s: Error occured while unregistering fb_notifier.\n", __func__);
#endif

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FLUSHBUFFER);

	fts_irq_enable(info, false);
	free_irq(info->irq, info);

	input_mt_destroy_slots(info->input_dev);

	input_unregister_device(info->input_dev);
	info->input_dev = NULL;

	info->board->power(info, false);

	kfree(info);

	return 0;
}

#ifdef USE_OPEN_CLOSE
#ifdef USE_OPEN_DWORK
static void fts_open_work(struct work_struct *work)
{
	int retval = 0;
	struct fts_ts_info *info = container_of(work, struct fts_ts_info,
						open_work.work);

	tsp_debug_info(&info->client->dev, "%s\n", __func__);

	retval = fts_start_device(info);
	if (retval < 0)
		tsp_debug_err(&info->client->dev,
			"%s: Failed to start device\n", __func__);
}
#endif
static int fts_input_open(struct input_dev *dev)
{
	struct fts_ts_info *info = input_get_drvdata(dev);
	unsigned char regAdd[4] = {0xB0, 0x01, 0x29, 0x41};
	int retval = 0;

	tsp_debug_info(&info->client->dev, "%s\n", __func__);

#ifdef USE_OPEN_DWORK
	schedule_delayed_work(&info->open_work,
			      msecs_to_jiffies(TOUCH_OPEN_DWORK_TIME));
#else
	retval = fts_start_device(info);
	if (retval < 0) {
		tsp_debug_err(&info->client->dev,
			"%s: Failed to start device\n", __func__);
		goto out;
	}
#endif

	tsp_debug_info(&info->client->dev,
			"FTS cmd after wakeup : h%d\n",
			info->retry_hover_enable_after_wakeup);

	if (info->retry_hover_enable_after_wakeup == 1) {
		fts_write_reg(info, &regAdd[0], 4);
		fts_command(info, FTS_CMD_HOVER_ON);
	}

out:
	return 0;
}

static void fts_input_close(struct input_dev *dev)
{
	struct fts_ts_info *info = input_get_drvdata(dev);

	tsp_debug_info(&info->client->dev, "%s\n", __func__);

#ifdef USE_OPEN_DWORK
	cancel_delayed_work(&info->open_work);
#endif

	fts_stop_device(info);

	info->retry_hover_enable_after_wakeup = 0;
}
#endif

static void fts_reinit(struct fts_ts_info *info)
{
	fts_interrupt_set(info, INT_DISABLE);
	fts_irq_enable(info, false);

	fts_systemreset(info);

	fts_wait_for_ready(info);

#ifdef FTS_SUPPORT_NOISE_PARAM
	fts_set_noise_param(info);
#endif

	fts_command(info, SENSEON);
	fts_delay(50);

#ifdef FTS_SUPPORT_TA_MODE
	if (info->TA_Pluged)
		fts_command(info, FTS_CMD_CHARGER_PLUGGED);
#endif

	info->touch_count = 0;
	info->palm_pressed = false;

	fts_command(info, FLUSHBUFFER);

	fts_irq_enable(info, true);
	fts_interrupt_set(info, INT_ENABLE);
}

void fts_release_all_finger(struct fts_ts_info *info)
{
	int i;

	for (i = 0; i < FINGER_MAX; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);

		if ((info->finger[i].state == EVENTID_ENTER_POINTER) ||
			(info->finger[i].state == EVENTID_MOTION_POINTER)) {
			info->touch_count--;
			if (info->touch_count < 0)
				info->touch_count = 0;

			tsp_debug_event(&info->client->dev,
				"[RA] tID:%d mc: %d tc:%d Ver[%02X%04X%01X%01X]\n",
				i, info->finger[i].mcount, info->touch_count,
				info->panel_revision, info->fw_main_version_of_ic,
				info->flip_enable, info->mainscr_disable);
		}

		info->finger[i].state = EVENTID_LEAVE_POINTER;
		info->finger[i].mcount = 0;
	}

	input_report_key(info->input_dev, BTN_TOUCH, 0);
	input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

#ifdef CONFIG_INPUT_BOOSTER
	input_booster_send_event(BOOSTER_DEVICE_TOUCH, BOOSTER_MODE_FORCE_OFF);
#endif

	input_sync(info->input_dev);
}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
void trustedui_mode_on(void)
{
	tsp_debug_info(&tui_tsp_info->client->dev,
				"%s, release all finger..", __func__);
	fts_release_all_finger(tui_tsp_info);
}
#endif

static void fts_reset_work(struct work_struct *work)
{
	struct fts_ts_info *info = container_of(work, struct fts_ts_info,
						reset_work.work);
	bool temp_lpm;

	temp_lpm = info->lowpower_mode;
	/* Reset-routine must go to power off state  */
	info->lowpower_mode = 0;

	tsp_debug_info(&info->client->dev, "%s, Call Power-Off to recover IC, lpm:%d\n", __func__, temp_lpm);
	fts_stop_device(info);

	fts_delay(100);	/* Delay to discharge the IC from ESD or On-state.*/
	if (fts_start_device(info) < 0)
		tsp_debug_err(&info->client->dev, "%s: Failed to start device\n", __func__);

	info->lowpower_mode = temp_lpm;
}

static int fts_stop_device(struct fts_ts_info *info)
{
	tsp_debug_dbg(&info->client->dev, "%s\n", __func__);

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
		tsp_debug_err(&info->client->dev,
			"%s TUI cancel event call!\n", __func__);
		fts_delay(100);
		tui_force_close(1);
		fts_delay(200);
		if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
			tsp_debug_err(&info->client->dev,
				"%s TUI flag force clear!\n", __func__);
			trustedui_clear_mask(TRUSTEDUI_MODE_VIDEO_SECURED|
						TRUSTEDUI_MODE_INPUT_SECURED);
			trustedui_set_mode(TRUSTEDUI_MODE_OFF);
		}
	}
#endif

	mutex_lock(&info->device_mutex);

	if (info->touch_stopped) {
		tsp_debug_err(&info->client->dev,
					"%s already power off\n", __func__);
		goto out;
	}

	if (info->lowpower_mode) {
		tsp_debug_info(&info->client->dev,
					"%s lowpower flag:%d\n",
					__func__, info->lowpower_flag);

		info->fts_power_state = FTS_POWER_STATE_LOWPOWER;

		fts_interrupt_set(info, INT_DISABLE);
		fts_irq_enable(info, false);
		synchronize_irq(info->irq);

		fts_command(info, FLUSHBUFFER);
		fts_command(info, SENSEOFF);
		fts_command(info, FLUSHBUFFER);

		fts_release_all_finger(info);
#ifdef FTS_SUPPORT_NOISE_PARAM
		fts_get_noise_param(info);
#endif

	} else {
		fts_interrupt_set(info, INT_DISABLE);
		fts_irq_enable(info, false);
		synchronize_irq(info->irq);

		fts_command(info, FLUSHBUFFER);
		fts_release_all_finger(info);
#ifdef FTS_SUPPORT_NOISE_PARAM
		fts_get_noise_param(info);
#endif
		info->touch_stopped = true;

		if (info->board->power)
			info->board->power(info, false);

		info->fts_power_state = FTS_POWER_STATE_POWERDOWN;
	}
 out:
	mutex_unlock(&info->device_mutex);
	return 0;
}

static int fts_start_device(struct fts_ts_info *info)
{
	tsp_debug_dbg(&info->client->dev, "%s %s\n",
			__func__,
			info->lowpower_mode ?
			"exit low power mode" : "");

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
		tsp_debug_err(&info->client->dev,
			"%s TUI cancel event call!\n", __func__);
		fts_delay(100);
		tui_force_close(1);
		fts_delay(200);
		if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
			tsp_debug_err(&info->client->dev,
				"%s TUI flag force clear!\n", __func__);
			trustedui_clear_mask(TRUSTEDUI_MODE_VIDEO_SECURED|
						TRUSTEDUI_MODE_INPUT_SECURED);
			trustedui_set_mode(TRUSTEDUI_MODE_OFF);
		}
	}
#endif

	mutex_lock(&info->device_mutex);

	if (!info->touch_stopped && !info->lowpower_mode) {
		tsp_debug_err(&info->client->dev,
				"%s already power on\n", __func__);
		goto out;
	}

	fts_release_all_finger(info);
	if (info->lowpower_mode) {
		/* low power mode command is sent after LCD OFF. */
		/* turn on touch power @ LCD ON */
		if (info->touch_stopped)
			goto tsp_power_on;

		info->reinit_done = false;
		fts_reinit(info);
		info->reinit_done = true;
	} else {
tsp_power_on:
		if (info->board->power)
			info->board->power(info, true);
		info->touch_stopped = false;

		info->reinit_done = false;
		fts_reinit(info);
		info->reinit_done = true;
	}

 out:
	mutex_unlock(&info->device_mutex);

	info->fts_power_state = FTS_POWER_STATE_ACTIVE;

	return 0;
}

static void fts_shutdown(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	tsp_debug_info(&info->client->dev, "FTS %s called!\n", __func__);

	if (info->lowpower_mode) {
		info->lowpower_mode = 0;
		tsp_debug_info(&info->client->dev, "FTS lowpower_mode off!\n");
	}

	fts_stop_device(info);
}

void fts_recovery_cx(struct fts_ts_info *info)
{
	unsigned char regAdd[4] = {0};
	unsigned char buf[8] = {0};
	unsigned char cnt = 100;
	int ret = 0;

	regAdd[0] = 0xB6;
	regAdd[1] = 0x00;
	regAdd[2] = 0x1E;
	regAdd[3] = 0x08;
	fts_write_reg(info, &regAdd[0], 4);		/* Loading FW to PRAM  without CRC Check */
	fts_delay(30);


	fts_command(info, CX_TUNNING);
	fts_delay(300);

	fts_command(info, FTS_CMD_SAVE_CX_TUNING);
	fts_delay(200);

	do {
		regAdd[0] = READ_ONE_EVENT;
		ret = fts_read_reg(info, regAdd, 1, &buf[0], FTS_EVENT_SIZE);

		fts_delay(10);

		if (cnt-- == 0)
				break;
	} while (buf[0] != 0x16 || buf[1] != 0x04);

	fts_command(info, SENSEON);
	fts_delay(50);

	fts_command(info, FLUSHBUFFER);
}

#ifdef CONFIG_PM
static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	tsp_debug_info(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users)
		fts_stop_device(info);

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	tsp_debug_info(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users)
		fts_start_device(info);

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}
#endif

static int fts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	tsp_debug_info(&info->client->dev, "%s power state : %d\n",
			__func__, info->fts_power_state);
	/* if suspend is called from non-active state, the i2c bus is not
	 * switched to AP, skipping suspend routine */
	if (info->fts_power_state != FTS_POWER_STATE_ACTIVE) {
		tsp_debug_info(&info->client->dev,
				"%s: calling suspend from non-active state, "
				"skipping\n", __func__);
		return 0;
	}

	fts_stop_device(info);

	gpio_set_value(info->switch_gpio, 1);
	tsp_debug_info(&info->client->dev,
			"%s: switch i2c to SLPI (set to %d)\n",
			__func__,
			gpio_get_value(info->switch_gpio));

	return 0;
}

static int fts_resume(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	tsp_debug_info(&info->client->dev, "%s power state : %d\n",
			__func__, info->fts_power_state);
	/* if resume is called from active state, the i2c bus is not
	 * switched to AP, skipping resume routine */
	if (info->fts_power_state == FTS_POWER_STATE_ACTIVE) {
		tsp_debug_info(&info->client->dev,
				"%s: calling resume from active state, "
				"skipping\n", __func__);
		return 0;
	}

	gpio_set_value(info->switch_gpio, 0);
	tsp_debug_info(&info->client->dev,
			"%s: switch i2c to AP (set to %d)\n",
			__func__,
			gpio_get_value(info->switch_gpio));

	fts_start_device(info);

	return 0;
}

#if defined(CONFIG_FB)
static int touch_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fts_ts_info *info =
		container_of(self, struct fts_ts_info, fb_notif);
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EARLY_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			fts_resume(info->client);
		else
			fts_suspend(info->client, PMSG_SUSPEND);
	}

	return 0;
}
#endif

static const struct i2c_device_id fts_device_id[] = {
	{FTS_TS_DRV_NAME, 0},
	{}
};

#ifdef CONFIG_PM
static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};
#endif

#ifdef CONFIG_OF
static struct of_device_id fts_match_table[] = {
	{ .compatible = "stm,ftm4_fts",},
	{ },
};
#else
#define fts_match_table NULL
#endif

static struct i2c_driver fts_i2c_driver = {
	.driver = {
		   .name = FTS_TS_DRV_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = fts_match_table,
#endif
#ifdef CONFIG_PM
		   .pm = &fts_dev_pm_ops,
#endif
		   },
	.probe = fts_probe,
	.remove = fts_remove,
	.shutdown = fts_shutdown,
#if (!defined(CONFIG_FB))
	.suspend = fts_suspend,
	.resume = fts_resume,
#endif
	.id_table = fts_device_id,
};

static int __init fts_driver_init(void)
{
	return i2c_add_driver(&fts_i2c_driver);
}

static void __exit fts_driver_exit(void)
{
	i2c_del_driver(&fts_i2c_driver);
}

MODULE_DESCRIPTION("STMicroelectronics MultiTouch IC Driver");
MODULE_AUTHOR("STMicroelectronics, Inc.");
MODULE_LICENSE("GPL v2");

module_init(fts_driver_init);
module_exit(fts_driver_exit);

/*
 * cyttsp5_loader.c
 * Cypress TrueTouch(TM) Standard Product V5 FW Loader Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2014 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */
#include "cyttsp5_regs.h"
#include "cyttsp5_core.h"
#include <linux/firmware.h>

#ifdef CONFIG_HUAWEI_KERNEL
#ifdef CONFIG_APP_INFO
#include <misc/app_info.h>
#endif
#endif /* CONFIG_HUAWEI_KERNEL */
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif/*CONFIG_HUAWEI_DSM*/

#define CYTTSP5_LOADER_NAME "cyttsp5_loader"
#define CY_FW_MANUAL_UPGRADE_FILE_NAME "cyttsp5_fw_manual_upgrade"

/* Enable UPGRADE_FW_AND_CONFIG_IN_PROBE definition
 * to perform FW and config upgrade during probe
 * instead of scheduling a work for it
 */
/* #define UPGRADE_FW_AND_CONFIG_IN_PROBE */

#define CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW 1
#define CYTTSP5_LOADER_FW_UPGRADE_RETRY_COUNT 3

#define CYTTSP5_FW_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE))

#define CYTTSP5_TTCONFIG_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE))

static const u8 cyttsp5_security_key[] = {
	0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A
};

/* Timeout values in ms. */
#define CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT		500
#define CY_LDR_SWITCH_TO_APP_MODE_TIMEOUT		300

#define CY_MAX_STATUS_SIZE				32

#define CY_DATA_MAX_ROW_SIZE			256
#define CY_DATA_ROW_SIZE				128

#define CY_ARRAY_ID_OFFSET				0
#define CY_ROW_NUM_OFFSET				1
#define CY_ROW_SIZE_OFFSET				3
#define CY_ROW_DATA_OFFSET				5

#define CY_POST_TT_CFG_CRC_MASK			0x2
#define FILENAME_LEN_MAX 				64
#define PANEL_NAME_LEN_MAX				16

#ifdef CONFIG_HUAWEI_DSM
extern struct tp_dsm_info g_tp_dsm_info;
extern struct dsm_client *tp_cyp_dclient;
extern ssize_t cyttsp5_dsm_record_basic_err_info(struct device *dev);
extern int cyttsp5_tp_report_dsm_err(struct device *dev, int type, int err_numb);
#endif/*CONFIG_HUAWEI_DSM*/

struct cyttsp5_loader_data {
	struct device *dev;
	struct cyttsp5_sysinfo *si;
	u8 status_buf[CY_MAX_STATUS_SIZE];
	struct completion int_running;
	struct completion calibration_complete;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	struct completion builtin_bin_fw_complete;
	int builtin_bin_fw_status;
	bool is_manual_upgrade_enabled;
#endif
	struct work_struct fw_and_config_upgrade;
	struct work_struct calibration_work;
	struct cyttsp5_loader_platform_data *loader_pdata;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	struct mutex config_lock;
	u8 *config_data;
	int config_size;
	bool config_loading;
#endif
};

struct cyttsp5_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

struct cyttsp5_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[CY_DATA_ROW_SIZE];
} __packed;

static struct cyttsp5_core_commands *cmd;

static inline struct cyttsp5_loader_data *cyttsp5_get_loader_data(
		struct device *dev)
{
	return cyttsp5_get_dynamic_data(dev, CY_MODULE_LOADER);
}

#if CYTTSP5_FW_UPGRADE || CYTTSP5_TTCONFIG_UPGRADE
/*
 * return code:
 * -1: Do not upgrade firmware
 *  0: Version info same, let caller decide
 *  1: Do a firmware upgrade
 */
static int cyttsp5_check_firmware_version(struct device *dev,
		u32 fw_ver_new, u32 fw_revctrl_new)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	u32 fw_ver_img;
	u32 fw_revctrl_img;

	fw_ver_img = ld->si->cydata.fw_ver_major << 8;
	fw_ver_img += ld->si->cydata.fw_ver_minor;

	tp_log_warning( "%s: img vers:0x%04X new vers:0x%04X\n", __func__,
			fw_ver_img, fw_ver_new);
	tp_log_warning( "%s: Firmware Configuration Version in iC:0x%04X\n",
			 __func__,ld->si->cydata.fw_ver_conf);

	if( FW_UPDATE_DIFF == cd->cpdata->fw_update_logic ) {
		tp_log_info("%s: update logic is update when image ver different with IC\n",__func__);
	} else {
		tp_log_info("%s: update logic is update to hige ver\n",__func__);
	}

	if (FW_UPDATE_DIFF == cd->cpdata->fw_update_logic) {
		if (fw_ver_new != fw_ver_img) {
			tp_log_info( "%s: Image ver is different, will upgrade\n", __func__);
			return 1;
		}
	} else {
		if (fw_ver_new > fw_ver_img) {
			tp_log_info( "%s: Image is newer, will upgrade\n", __func__);
			return 1;
		}
		if (fw_ver_new < fw_ver_img) {
			tp_log_info( "%s: Image is older, will NOT upgrade\n", __func__);
			return -1;
		}
	}

	fw_revctrl_img = ld->si->cydata.revctrl;

	tp_log_warning( "%s: img revctrl:0x%04X new revctrl:0x%04X\n",
			__func__, fw_revctrl_img, fw_revctrl_new);

	if (FW_UPDATE_DIFF == cd->cpdata->fw_update_logic) {
		if (fw_revctrl_new != fw_revctrl_img) {
			tp_log_info( "%s: Image ver is different, will upgrade\n", __func__);
			return 1;
		}
	} else {
		if (fw_revctrl_new > fw_revctrl_img) {
			tp_log_info( "%s: Image is newer, will upgrade\n", __func__);
			return 1;
		}
		if (fw_revctrl_new < fw_revctrl_img) {
			tp_log_info( "%s: Image is older, will NOT upgrade\n", __func__);
			return -1;
		}
	}
	tp_log_warning( "%s: Image has same version\n", __func__);

	return 0;
}
#ifdef CONFIG_HUAWEI_KERNEL

#else /* CONFIG_HUAWEI_KERNEL */
static u8 cyttsp5_get_panel_id(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return cd->panel_id;
}
#endif

static void cyttsp5_calibrate_idacs(struct work_struct *calibration_work)
{
	struct cyttsp5_loader_data *ld = container_of(calibration_work,
			struct cyttsp5_loader_data, calibration_work);
	struct device *dev = ld->dev;
	u8 mode;
	u8 status;
	int rc;

	rc = cmd->request_exclusive(dev, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err("%s:request_exclusive fail, rc:%d\n", __func__, rc);
		goto exit;
	}

	rc = cmd->nonhid_cmd->suspend_scanning(dev, 0);
	if (rc < 0) {
		tp_log_err("%s:suspend_scanning fail, rc:%d\n", __func__, rc);
		goto release;
	}

	for (mode = 0; mode < 3; mode++) {
		rc = cmd->nonhid_cmd->calibrate_idacs(dev, 0, mode, &status);
		if (rc < 0) {
			tp_log_err("%s:calibrate_idacs fail, rc:%d, mode:%d\n", __func__, rc, mode);
			goto release;
		}
	}

	rc = cmd->nonhid_cmd->resume_scanning(dev, 0);
	if (rc < 0) {
		tp_log_err("%s:resume_scanning fail, rc:%d\n", __func__, rc);
		goto release;
	}

	tp_log_info("%s: Calibration Done\n", __func__);

release:
	cmd->release_exclusive(dev);
exit:
	complete(&ld->calibration_complete);
}

static int cyttsp5_calibration_attention(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int rc = 0;

	schedule_work(&ld->calibration_work);

	cmd->unsubscribe_attention(dev, CY_ATTEN_STARTUP, CY_MODULE_LOADER,
		cyttsp5_calibration_attention, 0);

	return rc;
}


#endif /* CYTTSP5_FW_UPGRADE || CYTTSP5_TTCONFIG_UPGRADE */

#if CYTTSP5_FW_UPGRADE
static u8 *cyttsp5_get_row_(struct device *dev, u8 *row_buf,
		u8 *image_buf, int size)
{
	memcpy(row_buf, image_buf, size);
	return image_buf + size;
}

static int cyttsp5_ldr_enter_(struct device *dev, struct cyttsp5_dev_id *dev_id)
{
	int rc;
	u8 return_data[8];
	u8 mode;

	dev_id->silicon_id = 0;
	dev_id->rev_id = 0;
	dev_id->bl_ver = 0;

	cmd->request_reset(dev);

	rc = cmd->request_get_mode(dev, 0, &mode);
	if (rc < 0)
		return rc;

	if (mode == CY_MODE_UNKNOWN)
		return -EINVAL;

	if (mode == CY_MODE_OPERATIONAL) {
		rc = cmd->nonhid_cmd->start_bl(dev, 0);
		if (rc < 0)
			return rc;
	}

	rc = cmd->nonhid_cmd->get_bl_info(dev, 0, return_data);
	if (rc < 0)
		return rc;

	dev_id->silicon_id = get_unaligned_le32(&return_data[0]);
	dev_id->rev_id = return_data[4];
	dev_id->bl_ver = return_data[5] + (return_data[6] << 8)
		+ (return_data[7] << 16);

	return 0;
}

static int cyttsp5_ldr_init_(struct device *dev,
		struct cyttsp5_hex_image *row_image)
{
	return cmd->nonhid_cmd->initiate_bl(dev, 0, 8,
			(u8 *)cyttsp5_security_key, row_image->row_size,
			row_image->row_data);
}

static int cyttsp5_ldr_parse_row_(struct device *dev, u8 *row_buf,
	struct cyttsp5_hex_image *row_image)
{
	int rc = 0;

	row_image->array_id = row_buf[CY_ARRAY_ID_OFFSET];
	row_image->row_num = get_unaligned_be16(&row_buf[CY_ROW_NUM_OFFSET]);
	row_image->row_size = get_unaligned_be16(&row_buf[CY_ROW_SIZE_OFFSET]);

	if (row_image->row_size > ARRAY_SIZE(row_image->row_data)) {
		tp_log_err( "%s: row data buffer overflow\n", __func__);
		rc = -EOVERFLOW;
		goto cyttsp5_ldr_parse_row_exit;
	}

	memcpy(row_image->row_data, &row_buf[CY_ROW_DATA_OFFSET],
	       row_image->row_size);
cyttsp5_ldr_parse_row_exit:
	return rc;
}

static int cyttsp5_ldr_prog_row_(struct device *dev,
				 struct cyttsp5_hex_image *row_image)
{
	u16 length = row_image->row_size + 3;
	u8 data[3 + row_image->row_size];
	u8 offset = 0;

	data[offset++] = row_image->array_id;
	data[offset++] = LOW_BYTE(row_image->row_num);
	data[offset++] = HI_BYTE(row_image->row_num);
	memcpy(data + 3, row_image->row_data, row_image->row_size);
	return cmd->nonhid_cmd->prog_and_verify(dev, 0, length, data);
}

static int cyttsp5_ldr_verify_chksum_(struct device *dev)
{
	u8 result;
	int rc;

	rc = cmd->nonhid_cmd->verify_app_integrity(dev, 0, &result);
	if (rc)
		return rc;

	/* fail */
	if (result == 0)
		return -EINVAL;

	return 0;
}

static int cyttsp5_ldr_exit_(struct device *dev)
{
	return cmd->nonhid_cmd->launch_app(dev, 0);
}
/*Optimize log*/
static int cyttsp5_load_app_(struct device *dev, const u8 *fw, int fw_size)
{
	struct cyttsp5_dev_id *dev_id = NULL;
	struct cyttsp5_hex_image *row_image = NULL;
	u8 *row_buf = NULL;
	size_t image_rec_size = 0;
	size_t row_buf_size = CY_DATA_MAX_ROW_SIZE;
	int row_count = 0;
	u8 *p = NULL;
	u8 *last_row = NULL;
	int rc = 0;
	int rc_tmp = 0;

	tp_log_warning( "%s %d: enter to load app\n", __func__, __LINE__);
	image_rec_size = sizeof(struct cyttsp5_hex_image);
	if (fw_size % image_rec_size != 0) {
		tp_log_err( "%s: Firmware image is misaligned\n", __func__);
	#ifdef CONFIG_64BIT
		tp_log_err( "%s: fw_size = %d, image_rec_size = %ld\n",
						__func__, fw_size, image_rec_size);
	#else
		tp_log_err( "%s: fw_size = %d, image_rec_size = %d\n",
						__func__, fw_size, image_rec_size);
	#endif
		rc = -EINVAL;
		goto _cyttsp5_load_app_error;
	}

	tp_log_info( "%s: start load app\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "start load app");
#endif

	row_buf = kzalloc(row_buf_size, GFP_KERNEL);
	row_image = kzalloc(sizeof(struct cyttsp5_hex_image), GFP_KERNEL);
	dev_id = kzalloc(sizeof(struct cyttsp5_dev_id), GFP_KERNEL);
	if (!row_buf || !row_image || !dev_id) {
		rc = -ENOMEM;
		goto _cyttsp5_load_app_exit;
	}

	cmd->request_stop_wd(dev);

	tp_log_info( "%s: Send BL Loader Enter\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "Send BL Loader Enter");
#endif /* TTHE_TUNER_SUPPORT */
	rc = cyttsp5_ldr_enter_(dev, dev_id);
	if (rc) {
		tp_log_err( "%s: Error cannot start Loader (ret=%d)\n",
					__func__, rc);
		goto _cyttsp5_load_app_exit;
	}

	tp_log_vdebug( "%s: dev: silicon id=%08X rev=%02X bl=%08X\n",
				__func__, dev_id->silicon_id,
				dev_id->rev_id, dev_id->bl_ver);

	/* get last row */
	last_row = (u8 *)fw + fw_size - image_rec_size;
	cyttsp5_get_row_(dev, row_buf, last_row, image_rec_size);
	cyttsp5_ldr_parse_row_(dev, row_buf, row_image);

	/* initialise bootloader */
	rc = cyttsp5_ldr_init_(dev, row_image);
	if (rc) {
		tp_log_err( "%s: Error cannot init Loader (ret=%d)\n", __func__, rc);
		goto _cyttsp5_load_app_exit;
	}

	tp_log_info( "%s: Send BL Loader Blocks\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "Send BL Loader Blocks");
#endif
	p = (u8 *)fw;
	while (p < last_row) {

		/* Get row */
		memset(row_buf, 0, row_buf_size);
		p = cyttsp5_get_row_(dev, row_buf, p, image_rec_size);

		/* Parse row */
		rc = cyttsp5_ldr_parse_row_(dev, row_buf, row_image);
		tp_log_vdebug( "%s: array_id=%02X row_num=%04X(%d) row_size=%04X(%d)\n",
					__func__, row_image->array_id,
					row_image->row_num, row_image->row_num,
					row_image->row_size, row_image->row_size);
		if (rc) {
			tp_log_err( "%s: Parse Row Error (a=%d r=%d ret=%d\n",
						__func__, row_image->array_id,
						row_image->row_num, rc);
			goto _cyttsp5_load_app_exit;
		} else {
			tp_log_vdebug( "%s: Parse Row (a=%d r=%d ret=%d\n",
						__func__, row_image->array_id,
						row_image->row_num, rc);
		}

		/* program row */
		rc = cyttsp5_ldr_prog_row_(dev, row_image);
		if (rc) {
			tp_log_err( "%s: Program Row Error (array=%d row=%d ret=%d)\n",
						__func__, row_image->array_id,
						row_image->row_num, rc);
			goto _cyttsp5_load_app_exit;
		}

		tp_log_vdebug( "%s: array=%d row_cnt=%d row_num=%04X\n",
			__func__, row_image->array_id, row_count,
			row_image->row_num);
	}

	/* exit loader */
	tp_log_info( "%s: Send BL Loader Terminate\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "Send BL Loader Terminate");
#endif /* TTHE_TUNER_SUPPORT */

	rc = cyttsp5_ldr_exit_(dev);
	if (rc) {
		tp_log_err( "%s: Error on exit Loader (ret=%d)\n",	__func__, rc);

		/* verify app checksum */
		rc_tmp = cyttsp5_ldr_verify_chksum_(dev);
		if (rc_tmp) {
			tp_log_err( "%s: ldr_verify_chksum fail r=%d\n", __func__, rc_tmp);
		} else {
			tp_log_info( "%s: APP Checksum Verified\n", __func__);
		}
	}
	tp_log_warning("%s:load app successful.\n", __func__);

_cyttsp5_load_app_exit:
	kfree(row_buf);
	kfree(row_image);
	kfree(dev_id);
_cyttsp5_load_app_error:
	/* delete invalid log*/
	return rc;
}
static int cyttsp5_upgrade_firmware(struct device *dev, const u8 *fw_img,
		int fw_size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int retry = CYTTSP5_LOADER_FW_UPGRADE_RETRY_COUNT;
	bool wait_for_calibration_complete = false;
	int rc;

	tp_log_warning( "%s %d: cyttsp5 upgrade firmware begin\n", __func__, __LINE__);
	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_REQUEST_EXCLUSIVE_FAIL;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err( "%s %d: request_exclusive fail, rc = %d\n",
					__func__, __LINE__, rc);
		goto exit;
	}

	while (retry--) {
		rc = cyttsp5_load_app_(dev, fw_img, fw_size);
		if (rc < 0) {
			tp_log_err( "%s: Firmware update failed rc=%d, retry:%d\n",
						__func__, rc, retry);
		} else {
			break;
		}

		msleep(20);
	}

	if (rc < 0) {
		tp_log_err( "%s: Firmware update failed with error code %d\n",
					__func__, rc);
	} else if (ld->loader_pdata &&
			(ld->loader_pdata->flags
			 & CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
		reinit_completion(&ld->calibration_complete);
#else
		INIT_COMPLETION(ld->calibration_complete);
#endif
		/* set up call back for startup */
		tp_log_vdebug( "%s: Adding callback for calibration\n",	__func__);
		rc = cmd->subscribe_attention(dev, CY_ATTEN_STARTUP,
			CY_MODULE_LOADER, cyttsp5_calibration_attention, 0);
		if (rc) {
			tp_log_err( "%s: Failed adding callback for calibration\n", __func__);
			tp_log_err( "%s: No calibration will be performed\n", __func__);
			rc = 0;
		} else {
			wait_for_calibration_complete = true;
		}
	}

	cmd->release_exclusive(dev);

exit:
	if (!rc) {
		cmd->request_restart(dev, true);
	}

	/* some cypress ic should take 30S to do firmware update, but phone only use 20s to finish boot up,
	   and phone have gone to sleep when boot 30s, if we send calibration comman when TP ic is in sleep model,
	   calibration will fail and TP ic will going to die
	   to fix this issue, we move pm_runtime_put_sync(dev); to the place after calibration complete */
	if (wait_for_calibration_complete) {
		wait_for_completion(&ld->calibration_complete);
	}

	pm_runtime_put_sync(dev);
	return rc;
}

static int cyttsp5_loader_attention(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);

	complete(&ld->int_running);
	return 0;
}
#endif /* CYTTSP5_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
static int cyttsp5_check_firmware_version_platform(struct device *dev,
		struct cyttsp5_touch_firmware *fw)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!ld->si) {
		tp_log_info( "%s: No firmware infomation found, device FW may be corrupted\n",
			__func__);
		return CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->ver + 2);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->ver + 8);

	upgrade = cyttsp5_check_firmware_version(dev, fw_ver_new,
		fw_revctrl_new);

	if (upgrade > 0) {
		tp_log_info("%s:Will upgrade firmvare\n", __func__);
		return 1;
	}

	return 0;
}

static struct cyttsp5_touch_firmware *cyttsp5_get_platform_firmware(
		struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_firmware **fws;
	struct cyttsp5_touch_firmware *fw;
	u8 panel_id;

	panel_id = cyttsp5_get_panel_id(dev);
	if (panel_id == PANEL_ID_NOT_ENABLED) {
		tp_log_debug( "%s: Panel ID not enabled, using legacy firmware\n",
			__func__);
		return ld->loader_pdata->fw;
	}

	fws = ld->loader_pdata->fws;
	if (!fws) {
		tp_log_err( "%s: No firmwares provided\n", __func__);
		return NULL;
	}

	/* Find FW according to the Panel ID */
	while ((fw = *fws++)) {
		if (fw->panel_id == panel_id) {
			tp_log_debug( "%s: Found matching fw:%p with Panel ID: 0x%02X\n",
				__func__, fw, fw->panel_id);
			return fw;
		}
		tp_log_vdebug( "%s: Found mismatching fw:%p with Panel ID: 0x%02X\n",
			__func__, fw, fw->panel_id);
	}

	return NULL;
}

static int upgrade_firmware_from_platform(struct device *dev,
		bool forced)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_firmware *fw;
	int rc = -ENOSYS;
	int upgrade;

	tp_log_info("%s %d:upgrade firmware from platform start.\n",
				__func__, __LINE__);
	if (!ld->loader_pdata) {
		tp_log_err( "%s: No loader platform data\n", __func__);
		return rc;
	}

	fw = cyttsp5_get_platform_firmware(dev);
	if (!fw || !fw->img || !fw->size) {
		tp_log_err( "%s: No platform firmware\n", __func__);
		return rc;
	}

	if (!fw->ver || !fw->vsize) {
		tp_log_err( "%s: No platform firmware version\n",
			__func__);
		return rc;
	}

	if (forced)
		upgrade = forced;
	else
		upgrade = cyttsp5_check_firmware_version_platform(dev, fw);

	if (upgrade)
		return cyttsp5_upgrade_firmware(dev, fw->img, fw->size);

	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
static void _cyttsp5_firmware_cont(const struct firmware *fw, void *context)
{
	struct device *dev = context;
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u8 header_size = 0;

	if (!fw) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_FW_CONT_ERROR;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err( "%s %d: Missing parameter\n", __func__, __LINE__);
		goto cyttsp5_firmware_cont_exit;
	}

	if (!fw->data || !fw->size) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_FW_CONT_ERROR;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err( "%s: No firmware received\n", __func__);
		goto cyttsp5_firmware_cont_release_exit;
	}

	header_size = fw->data[0];
	if (header_size >= (fw->size + 1)) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_FW_CONT_ERROR;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err( "%s: Firmware format is invalid\n", __func__);
		goto cyttsp5_firmware_cont_release_exit;
	}

	cyttsp5_upgrade_firmware(dev, &(fw->data[header_size + 1]),
		fw->size - (header_size + 1));

cyttsp5_firmware_cont_release_exit:
	release_firmware(fw);

cyttsp5_firmware_cont_exit:
	ld->is_manual_upgrade_enabled = 0;
}

static int cyttsp5_check_firmware_version_builtin(struct device *dev,
		const struct firmware *fw)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!ld->si) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_GET_SYSINFO_FAIL;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_info( "%s: No firmware infomation found, device FW may be corrupted\n",
			__func__);
		return CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->data + 3);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->data + 9);

	upgrade = cyttsp5_check_firmware_version(dev, fw_ver_new, fw_revctrl_new);

	if (upgrade > 0) {
		tp_log_info("%s:Will upgrade firmware\n", __func__);
		return 1;
	}

	tp_log_info("%s:Will not upgrade firmware\n", __func__);
	return 0;
}

/*****************************************************************
Parameters    :  fw
                 context
Return        :  void
Description   :  if bin file have been upgraded,
					#ld->builtin_bin_fw_status will be set to 0
					if bin file haven't upgrade,
					#ld->builtin_bin_fw_status will be set to -EINVAL
*****************************************************************/
static void _cyttsp5_firmware_cont_builtin(const struct firmware *fw,
		void *context)
{
	struct device *dev = context;
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_platform_data *pdata = cyttsp5_get_platform_data(dev);
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int upgrade;

	if (!fw) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_FW_CONT_BUILTIN_ERROR;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_info( "%s: No builtin firmware\n", __func__);
		goto _cyttsp5_firmware_cont_builtin_exit;
	}

	if (!fw->data || !fw->size) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_FW_CONT_BUILTIN_ERROR;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err( "%s: Invalid builtin firmware\n", __func__);
		goto _cyttsp5_firmware_cont_builtin_exit;
	}
	tp_log_warning( "%s: Found firmware\n", __func__);
	if((ld->si) && (ld->si->cydata.fw_ver_conf < pdata->core_pdata->fw_upgrade_start_ver)){
		tp_log_warning( "%s: old FPC not need to upgrade FW,fw_ver_conf = %d\n",
					__func__, ld->si->cydata.fw_ver_conf);
		goto _cyttsp5_firmware_cont_builtin_exit;
	}

	if ( true == cd->config_crc_fail_flag ) {
		upgrade = 1;
		tp_log_info( "%s: IC config crc fail,will force upgrade bin\n", __func__);
	} else {
		upgrade = cyttsp5_check_firmware_version_builtin(dev, fw);
	}

	if (upgrade) {
		_cyttsp5_firmware_cont(fw, dev);
		ld->builtin_bin_fw_status = 0;
		complete(&ld->builtin_bin_fw_complete);
		return;
	}

_cyttsp5_firmware_cont_builtin_exit:
	release_firmware(fw);

	ld->builtin_bin_fw_status = -EINVAL;
	complete(&ld->builtin_bin_fw_complete);
}

static int upgrade_firmware_from_class(struct device *dev)
{
	int retval = 0;

	tp_log_warning( "%s: Enabling firmware class loader\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG,
			CY_FW_MANUAL_UPGRADE_FILE_NAME, dev, GFP_KERNEL, dev,
			_cyttsp5_firmware_cont);

	if (retval < 0) {
		tp_log_err( "%s: Fail request firmware class file load\n", __func__);
		return retval;
	} else {
		/* if retval is 0, it not mean reqest firmware success, more information
		   see the code of request_firmware_nowait */
		tp_log_info("%s %d:request firmware success.\n", __func__, __LINE__);
	}

	return 0;
}

/*
 * Generates binary FW filename as following:
 * - Panel ID not enabled: cyttsp5_fw.bin
 * - Panel ID enabled: cyttsp5_fw_pidXX.bin
 */
static char *generate_firmware_filename(struct device *dev)
{

#ifdef CONFIG_HUAWEI_KERNEL
	const char *product_name = NULL;
	struct cyttsp5_platform_data *pdata = cyttsp5_get_platform_data(dev);
	struct cyttsp5_core_platform_data *core_pdata = pdata->core_pdata;
#else /* CONFIG_HUAWEI_KERNEL */
	u8 panel_id;
#endif /* CONFIG_HUAWEI_KERNEL */
	char *filename = NULL;

	filename = kzalloc(FILENAME_LEN_MAX, GFP_KERNEL);
	if (!filename) {
		tp_log_err("%s %d:kzalloc fail\n", __func__, __LINE__);
		return NULL;
	}

#ifdef CONFIG_HUAWEI_KERNEL
	product_name = core_pdata->product_name;
	snprintf(filename, FILENAME_LEN_MAX, "%s_fw.bin", product_name);
#else /* CONFIG_HUAWEI_KERNEL */
	panel_id = cyttsp5_get_panel_id(dev);
	if (panel_id == PANEL_ID_NOT_ENABLED)
		snprintf(filename, FILENAME_LEN_MAX, "%s", CY_FW_FILE_NAME);
	else
		snprintf(filename, FILENAME_LEN_MAX, "%s_pid%02X%s",
			CY_FW_FILE_PREFIX, panel_id, CY_FW_FILE_SUFFIX);
#endif /* CONFIG_HUAWEI_KERNEL */
	tp_log_info( "%s: Filename: %s\n", __func__, filename);

	return filename;
}

/*****************************************************************
Parameters    :  dev
Return        :  0 or error number
Description   :  if the firmware have been upgraded, return 0
				 if some error happend or no need to upgrade, return error number
*****************************************************************/
static int upgrade_firmware_from_builtin(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
#ifdef CONFIG_HUAWEI_KERNEL
	const struct firmware *fw_entry = NULL;
#endif /*#ifdef CONFIG_HUAWEI_KERNEL*/
	char *filename = NULL;
	int retval = 0;

	tp_log_vdebug( "%s: Enabling firmware class loader built-in\n",	__func__);
	tp_log_info("%s %d:upgrade firmware from buildin start.\n", __func__, __LINE__);

	filename = generate_firmware_filename(dev);
	if (!filename) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_GENERATE_FW_NAME_FAIL;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err("%s %d:generate firmware filename fail\n", __func__, __LINE__);
		return -ENOMEM;
	}

#ifdef CONFIG_HUAWEI_KERNEL
	retval = request_firmware(&fw_entry, filename, dev);
	if (retval) {
		#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_UPDATE_status = FWU_REQUEST_FW_FAIL;
		#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err("%s %d: Fail request firmware class file load, ret = %d\n",
					__func__, __LINE__, retval);
		goto exit;
	}

	_cyttsp5_firmware_cont_builtin(fw_entry, dev);

#else /* CONFIG_HUAWEI_KERNEL */
	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			filename, dev, GFP_KERNEL, dev,
			_cyttsp5_firmware_cont_builtin);
	if (retval < 0) {
		tp_log_err( "%s: Fail request firmware class file load\n",
			__func__);
		goto exit;
	}
#endif /* CONFIG_HUAWEI_KERNEL */

	/* wait until FW binary upgrade finishes */
	wait_for_completion(&ld->builtin_bin_fw_complete);

	retval = ld->builtin_bin_fw_status;

exit:
	kfree(filename);

	return retval;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE */

#if CYTTSP5_TTCONFIG_UPGRADE
static int cyttsp5_write_config_row_(struct device *dev, u8 ebid,
		u16 row_number, u16 row_size, u8 *data)
{
	int rc;
	u16 actual_write_len;

	rc = cmd->nonhid_cmd->write_conf_block(dev, 0, row_number,
			row_size, ebid, data, (u8 *)cyttsp5_security_key,
			&actual_write_len);
	if (rc) {
		tp_log_err( "%s: Fail Put EBID=%d row=%d cmd fail r=%d\n",
			__func__, ebid, row_number, rc);
		return rc;
	}

	if (actual_write_len != row_size) {
		tp_log_err( "%s: Fail Put EBID=%d row=%d wrong write size=%d\n",
			__func__, ebid, row_number, actual_write_len);
		rc = -EINVAL;
	}

	return rc;
}
/*****************************************************************
Parameters    :  dev
                 ttconfig_data
                 ttconfig_size
Return        :  if success, return 0, else return error number
Description   :
*****************************************************************/
static int cyttsp5_upgrade_ttconfig(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	bool wait_for_calibration_complete = false;
	u8 ebid = CY_TCH_PARM_EBID;
	u16 row_size = CY_DATA_ROW_SIZE;
	u16 table_size;
	u16 row_count;
	u16 residue;
	u8 *row_buf;
	u8 verify_crc_status;
	u16 calculated_crc;
	u16 stored_crc;
	int rc = 0;
	int i;

	table_size = ttconfig_size;
	row_count = table_size / row_size;
	row_buf = (u8 *)ttconfig_data;

	tp_log_info( "%s: size:%d row_size=%d row_count=%d\n",
		__func__, table_size, row_size, row_count);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err("%s %d:request_exclusive fail, rc = %d\n", __func__, __LINE__, rc);
		goto exit;
	}

	rc = cmd->nonhid_cmd->suspend_scanning(dev, 0);
	if (rc < 0) {
		tp_log_err("%s %d:suspend_scanning fail, rc = %d\n", __func__, __LINE__, rc);
		goto release;
	}

	for (i = 0; i < row_count; i++) {
		tp_log_info( "%s: row=%d size=%d\n", __func__, i, row_size);
		rc = cyttsp5_write_config_row_(dev, ebid, i, row_size, row_buf);
		if (rc) {
			tp_log_err( "%s: Fail put row=%d r=%d\n", __func__, i, rc);
			break;
		}
		row_buf += row_size;
	}

	if (!rc) {
		residue = table_size % row_size;
		tp_log_info( "%s: row=%d size=%d\n", __func__, i, residue);
		rc = cyttsp5_write_config_row_(dev, ebid, i, residue, row_buf);
		row_count++;

		if (rc) {
			tp_log_err( "%s: Fail put row=%d r=%d\n", __func__, i, rc);
		}
	}

	if (!rc)
		tp_log_debug( "%s: TT_CFG updated: rows:%d bytes:%d\n",
			__func__, row_count, table_size);

	rc = cmd->nonhid_cmd->verify_config_block_crc(dev, 0, ebid,
			&verify_crc_status, &calculated_crc, &stored_crc);
	if (rc || verify_crc_status)
		tp_log_err( "%s: CRC Failed, ebid=%d, status=%d, scrc=%X ccrc=%X\n",
			__func__, ebid, verify_crc_status,
			calculated_crc, stored_crc);
	else
		tp_log_debug( "%s: CRC PASS, ebid=%d, status=%d, scrc=%X ccrc=%X\n",
			__func__, ebid, verify_crc_status,
			calculated_crc, stored_crc);

	rc = cmd->nonhid_cmd->resume_scanning(dev, 0);
	if (rc < 0) {
		tp_log_err( "%s: resume_scanning fail, rc = %d\n", __func__, rc);
		goto release;
	}

	if (ld->loader_pdata &&
			(ld->loader_pdata->flags
			 & CY_LOADER_FLAG_CALIBRATE_AFTER_TTCONFIG_UPGRADE)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
		reinit_completion(&ld->calibration_complete);
#else
		INIT_COMPLETION(ld->calibration_complete);
#endif
		/* set up call back for startup */
		tp_log_vdebug( "%s: Adding callback for calibration\n", __func__);
		rc = cmd->subscribe_attention(dev, CY_ATTEN_STARTUP,
			CY_MODULE_LOADER, cyttsp5_calibration_attention, 0);
		if (rc) {
			tp_log_err( "%s: Failed adding callback for calibration\n",
				__func__);
			tp_log_err( "%s: No calibration will be performed\n",
				__func__);
			rc = 0;
		} else
			wait_for_calibration_complete = true;
	}

release:
	cmd->release_exclusive(dev);

exit:
	if (!rc)
		cmd->request_restart(dev, true);

	pm_runtime_put_sync(dev);

	if (wait_for_calibration_complete)
		wait_for_completion(&ld->calibration_complete);

	return rc;
}
#endif /* CYTTSP5_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
static int cyttsp5_get_ttconfig_crc(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size, u16 *crc)
{
	u16 crc_loc;

	crc_loc = get_unaligned_le16(&ttconfig_data[2]);
	if (ttconfig_size < crc_loc + 2)
		return -EINVAL;

	*crc = get_unaligned_le16(&ttconfig_data[crc_loc]);

	return 0;
}
#ifndef CONFIG_HUAWEI_KERNEL
static int cyttsp5_get_ttconfig_version(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size, u16 *version)
{
	if (ttconfig_size < CY_TTCONFIG_VERSION_OFFSET
			+ CY_TTCONFIG_VERSION_SIZE)
		return -EINVAL;

	*version = get_unaligned_le16(
		&ttconfig_data[CY_TTCONFIG_VERSION_OFFSET]);

	return 0;
}
#endif

static int cyttsp5_check_ttconfig_version(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u16 cfg_crc_new;
	int rc;

	if (!ld->si) {
		tp_log_info( "%s %d: No firmware infomation found\n", __func__,__LINE__);
		return 0;
	}
#ifndef CONFIG_HUAWEI_KERNEL
	/* Check for config version */
	if (ld->loader_pdata->flags &
			CY_LOADER_FLAG_CHECK_TTCONFIG_VERSION) {
		u16 cfg_ver_new;

		rc = cyttsp5_get_ttconfig_version(dev, ttconfig_data,
				ttconfig_size, &cfg_ver_new);
		if (rc)
			return 0;

		tp_log_info( "%s: img_ver:0x%04X new_ver:0x%04X\n",
			__func__, ld->si->cydata.fw_ver_conf, cfg_ver_new);

		/* Check if config version is newer */
		if (cfg_ver_new > ld->si->cydata.fw_ver_conf) {
			tp_log_info( "%s: Config version newer, will upgrade\n", __func__);
			return 1;
		}

		tp_log_info( "%s: Config version is identical or older, will NOT upgrade\n",
			__func__);
	/* Check for config CRC */
	} else {
#endif
		rc = cyttsp5_get_ttconfig_crc(dev, ttconfig_data,
				ttconfig_size, &cfg_crc_new);
		if (rc) {
			tp_log_err("%s %d: get ttconfig crc err! rc = %d", __func__,__LINE__,rc);
			return 0;
		}

		tp_log_info("%s: img_crc:0x%04X new_crc:0x%04X\n",
			__func__, ld->si->ttconfig.crc, cfg_crc_new);

		if (cfg_crc_new != ld->si->ttconfig.crc) {
			tp_log_info("%s: Config CRC different, will upgrade\n",
				__func__);
			return 1;
		}

		tp_log_info("%s: Config CRC equal, will NOT upgrade\n",
			__func__);

#ifndef CONFIG_HUAWEI_KERNEL
	}
#endif

	return 0;
}
static int cyttsp5_check_ttconfig_version_platform(struct device *dev,
		struct cyttsp5_touch_config *ttconfig)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u32 fw_ver_config;
	u32 fw_revctrl_config;

    ld->si = cmd->request_sysinfo(dev);
	if (!ld->si) {
		tp_log_info( "%s: No firmware infomation found\n", __func__);
		return 0;
	}

	fw_ver_config = get_unaligned_be16(ttconfig->fw_ver + 2);
	/* 4 middle bytes are not used */
	fw_revctrl_config = get_unaligned_be32(ttconfig->fw_ver + 8);

	/* FW versions should match */
	if (cyttsp5_check_firmware_version(dev, fw_ver_config, fw_revctrl_config)) {
		tp_log_err( "%s: FW versions mismatch\n", __func__);
		return 0;
	}

	/* Check PowerOn Self Test, TT_CFG CRC bit */
	if ((ld->si->cydata.post_code & CY_POST_TT_CFG_CRC_MASK) == 0) {
		tp_log_debug( "%s: POST, TT_CFG failed (%X), will upgrade\n",
			__func__, ld->si->cydata.post_code);
		return 1;
	}

	return cyttsp5_check_ttconfig_version(dev, ttconfig->param_regs->data,
			ttconfig->param_regs->size);
}

static struct cyttsp5_touch_config *cyttsp5_get_platform_ttconfig(
		struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_config **ttconfigs;
	struct cyttsp5_touch_config *ttconfig;
	struct cyttsp5_platform_data *pdata = cyttsp5_get_platform_data(dev);
	struct cyttsp5_core_platform_data *core_pdata = pdata->core_pdata;
	u8 panel_id;
	const char *product_name = NULL;

	panel_id = cyttsp5_get_panel_id(dev);
	if (panel_id == PANEL_ID_NOT_ENABLED) {
		/* TODO: Make debug message */
		tp_log_info( "%s: Panel ID not enabled, using legacy ttconfig\n",
			__func__);
		return ld->loader_pdata->ttconfig;
	}

	product_name = core_pdata->product_name;

	ttconfigs = ld->loader_pdata->ttconfigs;
	if (!ttconfigs) {
		tp_log_err("%s %d:not ttconfigs data found\n", __func__, __LINE__);
		return NULL;
	}

	/* Find TT config according to the Panel ID */
	while ((ttconfig = *ttconfigs++)) {

		tp_log_info("%s %d:%d %d\n", __func__, __LINE__, ttconfig->panel_id, panel_id);
		tp_log_info("%s %d:%s %s\n", __func__, __LINE__, ttconfig->product_name, product_name);
		if (ttconfig->panel_id == panel_id
			&& strcmp(product_name, ttconfig->product_name) == 0) {
			/* TODO: Make debug message */
			tp_log_info( "%s: Found matching ttconfig:%p with Panel ID: 0x%02X\n",
				__func__, ttconfig, ttconfig->panel_id);
			return ttconfig;
		}
		tp_log_vdebug( "%s: Found mismatching ttconfig:%p with Panel ID: 0x%02X\n",
			__func__, ttconfig, ttconfig->panel_id);
	}

	return NULL;
}

/*****************************************************************
Parameters    :  dev
Return        :
Description   :  if have upgraded, return 0, else return error number
*****************************************************************/
static int upgrade_ttconfig_from_platform(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_config *ttconfig;
	struct touch_settings *param_regs;
	struct cyttsp5_touch_fw;
	int rc = -ENOSYS;
	int upgrade;

	tp_log_info("%s %d:upgrade ttconfig from platform start.\n",
				__func__, __LINE__);
	if (!ld->loader_pdata) {
		tp_log_info( "%s: No loader platform data\n", __func__);
		return rc;
	}

	ttconfig = cyttsp5_get_platform_ttconfig(dev);
	if (!ttconfig) {
		tp_log_info( "%s: No ttconfig data\n", __func__);
		return rc;
	}

	param_regs = ttconfig->param_regs;
	if (!param_regs) {
		tp_log_info( "%s: No touch parameters\n", __func__);
		return rc;
	}

	if (!param_regs->data || !param_regs->size) {
		tp_log_info( "%s: Invalid touch parameters\n", __func__);
		return rc;
	}

	if (!ttconfig->fw_ver || !ttconfig->fw_vsize) {
		tp_log_info( "%s: Invalid FW version for touch parameters\n",
			__func__);
		return rc;
	}

	upgrade = cyttsp5_check_ttconfig_version_platform(dev, ttconfig);
	if (upgrade) {
		tp_log_info("%s %d:upgrade ttconfig\n", __func__, __LINE__);
		rc = cyttsp5_upgrade_ttconfig(dev, param_regs->data, param_regs->size);
		tp_log_info("%s %d:upgrade ttconfig, rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}

	tp_log_info("%s, will not upgrade ttconfig\n", __func__);
	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
static ssize_t cyttsp5_config_data_write(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp5_loader_data *data = cyttsp5_get_loader_data(dev);
	u8 *p;

	tp_log_vdebug( "%s: offset:%lld count:%zu\n", __func__, offset, count);

	mutex_lock(&data->config_lock);

	if (!data->config_loading) {
		mutex_unlock(&data->config_lock);
		return -ENODEV;
	}

	p = krealloc(data->config_data, offset + count, GFP_KERNEL);
	if (!p) {
		kfree(data->config_data);
		data->config_data = NULL;
		mutex_unlock(&data->config_lock);
		return -ENOMEM;
	}
	data->config_data = p;

	memcpy(&data->config_data[offset], buf, count);
	data->config_size += count;

	mutex_unlock(&data->config_lock);

	return count;
}

static struct bin_attribute bin_attr_config_data = {
	.attr = {
		.name = "config_data",
		.mode = S_IWUSR,
	},
	.size = 0,
	.write = cyttsp5_config_data_write,
};

static ssize_t cyttsp5_config_loading_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	bool config_loading;

	mutex_lock(&ld->config_lock);
	config_loading = ld->config_loading;
	mutex_unlock(&ld->config_lock);

	return sprintf(buf, "%d\n", config_loading);
}

static int cyttsp5_verify_ttconfig_binary(struct device *dev,
		u8 *bin_config_data, int bin_config_size, u8 **start, int *len)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int header_size;
	u16 config_size;
	u32 fw_ver_config;
	u32 fw_revctrl_config;

	if (!ld->si) {
		tp_log_err( "%s: No firmware infomation found, device FW may be corrupted\n",
			__func__);
		return -ENODEV;
	}

	/*
	 * We need 11 bytes for FW version control info and at
	 * least 6 bytes in config (Length + Max Length + CRC)
	 */
	header_size = bin_config_data[0] + 1;
	if (header_size < 11 || header_size >= bin_config_size - 6) {
		tp_log_err( "%s: Invalid header size %d\n", __func__,
			header_size);
		return -EINVAL;
	}

	fw_ver_config = get_unaligned_be16(&bin_config_data[1]);
	/* 4 middle bytes are not used */
	fw_revctrl_config = get_unaligned_be32(&bin_config_data[7]);

	/* FW versions should match */
	if (cyttsp5_check_firmware_version(dev, fw_ver_config,
			fw_revctrl_config)) {
		tp_log_err( "%s: FW versions mismatch\n", __func__);
		return -EINVAL;
	}

	config_size = get_unaligned_le16(&bin_config_data[header_size]);
	/* Perform a simple size check (2 bytes for CRC) */
	if (config_size != bin_config_size - header_size - 2) {
		tp_log_err( "%s: Config size invalid\n", __func__);
		return -EINVAL;
	}

	*start = &bin_config_data[header_size];
	*len = bin_config_size - header_size;

	return 0;
}

/*
 * 1: Start loading TT Config
 * 0: End loading TT Config and perform upgrade
 *-1: Exit loading
 */
static ssize_t cyttsp5_config_loading_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	long value;
	u8 *start;
	int length;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc < 0 || value < -1 || value > 1) {
		tp_log_err( "%s: Invalid value\n", __func__);
		return size;
	}

	mutex_lock(&ld->config_lock);

	if (value == 1)
		ld->config_loading = true;
	else if (value == -1)
		ld->config_loading = false;
	else if (value == 0 && ld->config_loading) {
		ld->config_loading = false;
		if (ld->config_size == 0) {
			tp_log_err( "%s: No config data\n", __func__);
			goto exit_free;
		}

		rc = cyttsp5_verify_ttconfig_binary(dev,
				ld->config_data, ld->config_size,
				&start, &length);
		if (rc)
			goto exit_free;

		rc = cyttsp5_upgrade_ttconfig(dev, start, length);
	}

exit_free:
	kfree(ld->config_data);
	ld->config_data = NULL;
	ld->config_size = 0;

	mutex_unlock(&ld->config_lock);

	if (rc)
		return rc;

	return size;
}

static DEVICE_ATTR(config_loading, S_IRUSR | S_IWUSR,
	cyttsp5_config_loading_show, cyttsp5_config_loading_store);
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE */

static void cyttsp5_fw_and_config_upgrade(
		struct work_struct *fw_and_config_upgrade)
{
	struct cyttsp5_loader_data *ld = container_of(fw_and_config_upgrade,
			struct cyttsp5_loader_data, fw_and_config_upgrade);
	struct device *dev = ld->dev;
	int retVal = 0;

	ld->si = cmd->request_sysinfo(dev);
	if (!ld->si) {
		tp_log_err( "%s: Fail get sysinfo pointer from core\n",	__func__);
	}
#if !CYTTSP5_FW_UPGRADE
	tp_log_info( "%s: No FW upgrade method selected!\n", __func__);
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	if (!upgrade_firmware_from_platform(dev, false)) {
		tp_log_err("%s %d:PLATFORM_FW_UPGRADE fail\n", __func__, __LINE__);
	} else {
		tp_log_info("%s %d:update firmware from platform\n", __func__, __LINE__);
	}
#endif
#ifdef CONFIG_HUAWEI_DSM
	g_tp_dsm_info.constraints_UPDATE_status = TS_UPDATE_STATE_UNDEFINE;
#endif/*CONFIG_HUAWEI_DSM*/
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	retVal = upgrade_firmware_from_builtin(dev);
	if (!retVal) {
		tp_log_warning("%s %d:firmware have upgraded\n", __func__, __LINE__);
	} else {
		tp_log_warning("%s %d:firmware haven't upgraded, rc = %d\n", __func__, __LINE__, retVal);
	}
#endif
#ifdef CONFIG_HUAWEI_DSM
	if(TS_UPDATE_STATE_UNDEFINE != g_tp_dsm_info.constraints_UPDATE_status){
		cyttsp5_tp_report_dsm_err(dev, DSM_TP_FW_ERROR_NO, retVal);
	}
#endif/*CONFIG_HUAWEI_DSM*/

	/* when firmware in ic missing, ld->si will be NULL, this issue will lead to ttconfig upgrade fail,
	   to fix this issue, after upgrade bin, we should request system info again */
	if(!ld->si){
		ld->si = cmd->request_sysinfo(dev);
	}
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	retVal = upgrade_ttconfig_from_platform(dev);
	if (!retVal) {
		tp_log_warning("%s %d:ttconfig have upgreded\n", __func__, __LINE__);
	} else {
		tp_log_warning("%s %d:ttconfig haven't upgrade, rc = %d\n", __func__, __LINE__, retVal);
	}
	cyttsp5_set_app_info(dev);
#endif
}

#if CYTTSP5_FW_UPGRADE
static int cyttsp5_fw_upgrade_cb(struct device *dev)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	if (!upgrade_firmware_from_platform(dev, false))
		return 1;
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	if (!upgrade_firmware_from_builtin(dev))
		return 1;
#endif
	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
static ssize_t cyttsp5_forced_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int rc = upgrade_firmware_from_platform(dev, true);

	if (rc)
		return rc;
	return size;
}

static DEVICE_ATTR(forced_upgrade, S_IRUSR | S_IWUSR,
	NULL, cyttsp5_forced_upgrade_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
static ssize_t cyttsp5_manual_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int rc;

	tp_log_warning("%s %d:Enter manual_upgrade\n", __func__, __LINE__);
	if (ld->is_manual_upgrade_enabled) {
		tp_log_err("%s %d:System is doing fw upgrade, please try later\n",
					__func__, __LINE__);
		return -EBUSY;
	}

	ld->is_manual_upgrade_enabled = 1;

	rc = upgrade_firmware_from_class(ld->dev);

	if (rc < 0) {
		tp_log_err("%s %d:upgrade firmware from class fail\n", __func__, __LINE__);
		ld->is_manual_upgrade_enabled = 0;
	}
	return size;
}
/* set write-only permission for sysfs node*/
static DEVICE_ATTR(manual_upgrade, S_IWUSR,
	NULL, cyttsp5_manual_upgrade_store);
#endif
static int cyttsp5_loader_probe(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_loader_data *ld;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	int rc;

	if (!pdata || !pdata->loader_pdata) {
		tp_log_err( "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	ld = kzalloc(sizeof(*ld), GFP_KERNEL);
	if (!ld) {
		tp_log_err("%s %d:kzalloc fail\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

/* we have define CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE */
/* create forced upgrade file interface, this interface */
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	rc = device_create_file(dev, &dev_attr_forced_upgrade);
	if (rc) {
		tp_log_err( "%s: Error, could not create forced_upgrade\n",
				__func__);
		goto error_create_forced_upgrade;
	} else {
		tp_log_info("%s:create PLATFORM_FW_UPGRADE success\n", __func__);
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	rc = device_create_file(dev, &dev_attr_manual_upgrade);
	if (rc) {
		tp_log_err( "%s: Error, could not create manual_upgrade\n",
				__func__);
		goto error_create_manual_upgrade;
	} else {
		tp_log_info("%s:create BINARY_FW_UPGRADE success\n", __func__);
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	rc = device_create_file(dev, &dev_attr_config_loading);
	if (rc) {
		tp_log_err( "%s: Error, could not create config_loading\n",
				__func__);
		goto error_create_config_loading;
	} else {
		tp_log_info("%s:create MANUAL_TTCONFIG_UPGRADE success\n", __func__);
	}

	rc = device_create_bin_file(dev, &bin_attr_config_data);
	if (rc) {
		tp_log_err( "%s: Error, could not create config_data\n",
				__func__);
		goto error_create_config_data;
	}
#endif

	ld->loader_pdata = pdata->loader_pdata;
	ld->dev = dev;
	cd->cyttsp5_dynamic_data[CY_MODULE_LOADER] = ld;

#if CYTTSP5_FW_UPGRADE
	init_completion(&ld->int_running);
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	init_completion(&ld->builtin_bin_fw_complete);
#endif
	cmd->subscribe_attention(dev, CY_ATTEN_IRQ, CY_MODULE_LOADER,
		cyttsp5_loader_attention, CY_MODE_BOOTLOADER);

	cmd->subscribe_attention(dev, CY_ATTEN_LOADER, CY_MODULE_LOADER,
		cyttsp5_fw_upgrade_cb, CY_MODE_UNKNOWN);
#endif
#if CYTTSP5_FW_UPGRADE || CYTTSP5_TTCONFIG_UPGRADE
	init_completion(&ld->calibration_complete);
	INIT_WORK(&ld->calibration_work, cyttsp5_calibrate_idacs);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	mutex_init(&ld->config_lock);
#endif

#ifdef UPGRADE_FW_AND_CONFIG_IN_PROBE
	/* Call FW and config upgrade directly in probe */
	cyttsp5_fw_and_config_upgrade(&ld->fw_and_config_upgrade);
#else
	INIT_WORK(&ld->fw_and_config_upgrade, cyttsp5_fw_and_config_upgrade);
	schedule_work(&ld->fw_and_config_upgrade);
#endif
	tp_log_warning( "%s: Successful probe %s\n", __func__, dev_name(dev));
	return 0;

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
error_create_config_data:
	device_remove_file(dev, &dev_attr_config_loading);
error_create_config_loading:
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_manual_upgrade);
error_create_manual_upgrade:
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	device_remove_file(dev, &dev_attr_forced_upgrade);
error_create_forced_upgrade:
#endif
	cd->cyttsp5_dynamic_data[CY_MODULE_LOADER] = NULL;
	kfree(ld);
error_alloc_data_failed:
error_no_pdata:
	tp_log_err( "%s failed.\n", __func__);
	return rc;
}

static int cyttsp5_loader_release(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int rc = 0;

#if CYTTSP5_FW_UPGRADE
	cmd->unsubscribe_attention(dev, CY_ATTEN_IRQ, CY_MODULE_LOADER,
		cyttsp5_loader_attention, CY_MODE_BOOTLOADER);

	cmd->unsubscribe_attention(dev, CY_ATTEN_LOADER, CY_MODULE_LOADER,
		cyttsp5_fw_upgrade_cb, CY_MODE_UNKNOWN);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	device_remove_bin_file(dev, &bin_attr_config_data);
	device_remove_file(dev, &dev_attr_config_loading);
	kfree(ld->config_data);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_manual_upgrade);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	device_remove_file(dev, &dev_attr_forced_upgrade);
#endif
	cd->cyttsp5_dynamic_data[CY_MODULE_LOADER] = NULL;
	kfree(ld);
	return rc;
}

static char *core_ids[CY_MAX_NUM_CORE_DEVS] = {
	CY_DEFAULT_CORE_ID,
	NULL,
	NULL,
	NULL,
	NULL
};

static int num_core_ids = 1;

module_param_array(core_ids, charp, &num_core_ids, 0);
MODULE_PARM_DESC(core_ids,
	"Core id list of cyttsp5 core devices for loader module");

static int __init cyttsp5_loader_init(void)
{
	struct cyttsp5_core_data *cd;
	int rc = 0;
	int i, j;

	/* Check for invalid or duplicate core_ids */
	for (i = 0; i < num_core_ids; i++) {
		if (!strlen(core_ids[i])) {
			tp_log_err("%s: core_id %d is empty\n",	__func__, i + 1);
			return -EINVAL;
		}
		for (j = i+1; j < num_core_ids; j++) {
			if (!strcmp(core_ids[i], core_ids[j])) {
				tp_log_err("%s: core_ids %d and %d are same\n",
					__func__, i+1, j+1);
				return -EINVAL;
			}
		}
	}

	cmd = cyttsp5_get_commands();
	if (!cmd) {
		tp_log_err("%s %d:cyttsp5_get_commands fail\n", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < num_core_ids; i++) {
		cd = cyttsp5_get_core_data(core_ids[i]);
		if (!cd) {
			continue;
		}

		tp_log_info("%s: Registering loader module for core_id: %s\n",
			__func__, core_ids[i]);
		rc = cyttsp5_loader_probe(cd->dev);
		if (rc < 0) {
			tp_log_err("%s: Error, failed registering module\n", __func__);
			goto fail_unregister_devices;
		}
	}

	tp_log_info("%s: Cypress TTSP FW Loader Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);

	return 0;

fail_unregister_devices:
	for (i--; i >= 0; i--) {
		cd = cyttsp5_get_core_data(core_ids[i]);
		if (!cd) {
			continue;
		}

		cyttsp5_loader_release(cd->dev);
		tp_log_info("%s: Unregistering loader module for core_id: %s\n",
			__func__, core_ids[i]);
	}

	return rc;
}
module_init(cyttsp5_loader_init);

static void __exit cyttsp5_loader_exit(void)
{
	struct cyttsp5_core_data *cd;
	int i;

	for (i = 0; i < num_core_ids; i++) {
		cd = cyttsp5_get_core_data(core_ids[i]);
		if (!cd)
			continue;
		cyttsp5_loader_release(cd->dev);
		tp_log_info("%s: Unregistering loader module for core_id: %s\n",
			__func__, core_ids[i]);
	}
}
module_exit(cyttsp5_loader_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product FW Loader Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");

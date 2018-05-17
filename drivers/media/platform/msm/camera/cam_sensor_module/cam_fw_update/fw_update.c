/* OIS calibration interface for LC898123 F40
 *
 */

#include "fw_update.h"
#include "VCM_firmware.h"

#define OIS_CUR_FW_VERSION 0x08
#define OIS_COMPONENT_I2C_ADDR_WRITE 0x7C
#define VCM_COMPONENT_I2C_ADDR_WRITE 0xE4
#define VCM_EEPROM_I2C_ADDR_WRITE 0xE6
#define MK_SHARP   0x04170000
#define MK_LGIT    0x09170000
#define RETRY_MAX 3
#define LUT_MAX 3

static struct camera_io_master *g_io_master_info;
bool g_first = true;

void RamWrite32A( UINT_16 RamAddr, UINT_32 RamData )
{
	int rc = 0;
	struct camera_io_master *io_master_info = g_io_master_info;
	struct cam_sensor_i2c_reg_setting i2c_reg_settings;
	struct cam_sensor_i2c_reg_array i2c_reg_array;

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	i2c_reg_settings.size = 1;
	i2c_reg_settings.delay = 0;
	i2c_reg_array.reg_addr = RamAddr;
	i2c_reg_array.reg_data = RamData;
	i2c_reg_array.delay = 0;
	i2c_reg_settings.reg_setting = &i2c_reg_array;

	rc = camera_io_dev_write(io_master_info, &i2c_reg_settings);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "[OISFW] %s : write failed", __func__);
	}
}

void RamRead32A( UINT_16 RamAddr, UINT_32 *ReadData )
{
	int rc = 0;

	rc = camera_io_dev_read(g_io_master_info, RamAddr,
		ReadData, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_DWORD);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s read i2c failed", __func__);
}

int RamWrite8A( struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 RamData )
{
	int rc = 0;
	struct cam_sensor_i2c_reg_setting i2c_reg_settings;
	struct cam_sensor_i2c_reg_array i2c_reg_array;

	i2c_reg_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_settings.size = 1;
	i2c_reg_settings.delay = 0;
	i2c_reg_array.reg_addr = RamAddr;
	i2c_reg_array.reg_data = RamData;
	i2c_reg_array.delay = 0;
	i2c_reg_settings.reg_setting = &i2c_reg_array;

	rc = camera_io_dev_write(io_info, &i2c_reg_settings);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
			"%s: write 0x%x failed", __func__, RamAddr);
	}

	return rc;
}

int RamRead8A( struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 *ReadData )
{
	int rc = 0;

	rc = camera_io_dev_read(io_info, RamAddr,
		ReadData, CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR,
			"%s: read 0x%x failed", __func__, RamAddr);

	return rc;
}

void WitTim( UINT_16 UsWitTim )
{
	msleep(UsWitTim);
}

int CntWrt( UINT_8 *PcSetDat, UINT_16 UsDatNum)
{
	int rc = 0, cnt;
	uint16_t total_bytes = UsDatNum-1;
	uint8_t *ptr = NULL;
	struct camera_io_master *io_master_info = g_io_master_info;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * total_bytes,
		GFP_KERNEL);
	if (!i2c_reg_setting.reg_setting) {
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s Failed in allocating i2c_array", __func__);
		return -ENOMEM;
	}

	for (cnt = 0, ptr = &PcSetDat[1]; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =	PcSetDat[0];
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(io_master_info, &i2c_reg_setting, 1);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s i2c write sequence error:%d",
				__func__, rc);
	}
	kfree(i2c_reg_setting.reg_setting);
	return rc;
}

int CntRd3( UINT_32 addr, void *PcSetDat, UINT_16 UsDatNum )
{
	int rc = 0;

	rc = camera_io_dev_read_seq(g_io_master_info, addr, PcSetDat, CAMERA_SENSOR_I2C_TYPE_WORD, UsDatNum);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s read i2c failed", __func__);
	return rc;
}

void WPBCtrl( UINT_8 UcCtrl )
{
	//do nothing because lc898123F40 uses UnlockCodeSet() to handle WPB by itself
}

int checkHighLevelCommand(int cnt)
{
	int i;
	UINT_32 FWRead;


	for (i =0; i< cnt ; i++) {
		WitTim(10);
		RamRead32A(0xF100, &FWRead);//Check high level command ready.
		if (FWRead == 0x0 ) {
			CAM_INFO(CAM_SENSOR, "[OISFW]:checkHighLevelCommand finish.");
			return 0;
		} else
			CAM_INFO(CAM_SENSOR, "[OISFW]:Waiting...");
	}
	CAM_ERR(CAM_SENSOR, "[OISFW]:checkHighLevelCommand fail.");
	return -EINVAL;
}

int doFWupdate(UINT_16 CAL_ID, UINT_32 MODULE_MAKER)
{
	int rc = 0;

	if(MODULE_MAKER == MK_SHARP) {
		if (CAL_ID == 0x0) {
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s not to update FW because of unknown CAL_ID.", __func__);
			rc = -EINVAL;
		} else if (CAL_ID == 0x1) {
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s F40_FlashDownload(0, 4, 0 )", __func__);
			rc = F40_FlashDownload(0, 4, 0);
		} else {
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s F40_FlashDownload(0, 4, 1 )", __func__);
			rc = F40_FlashDownload(0, 4, 1);
		}
	} else if(MODULE_MAKER == MK_LGIT) {
		if (CAL_ID == 0x0) {
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s not to update FW because of unknown CAL_ID.", __func__);
			rc = -EINVAL;
		} else if (CAL_ID == 0x1) {
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s F40_FlashDownload(0, 9, 0 )", __func__);
			rc = F40_FlashDownload(0, 9, 0);
		} else {
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s F40_FlashDownload(0, 9, 1 )", __func__);
			rc = F40_FlashDownload(0, 9, 1);
		}
	} else {
		CAM_ERR(CAM_SENSOR, "[OISFW]:%s unknown module maker.", __func__);
		rc = -EINVAL;
	}

	if (rc == 0) {
		/*Wait for FW update finish.*/
		rc = checkHighLevelCommand(20);
	} else {
		CAM_ERR(CAM_SENSOR, "[OISFW]%s:OIS FW update failed rc = %d.",
			__func__, rc);
		rc = -EINVAL;
	}

	return rc;
}

bool checkOISFWversion(UINT_16 *cal_id, UINT_32 *module_maker)
{
	int rc;
	UINT_16 RamAddr;
	UINT_32 UlReadVal;
	UINT_16 FW_version;
	bool need_update = false;

	CAM_INFO(CAM_SENSOR, "[OISFW]:%s", __func__);

	RamAddr = 0x8000;
	RamRead32A(RamAddr, &UlReadVal);
	FW_version = UlReadVal & 0xFF;
	*module_maker = UlReadVal & 0xFFFF0000;
	CAM_INFO(CAM_SENSOR, "[OISFW]:%s module_version =  0x%02x",
			__func__, UlReadVal);

	RamAddr = 0x8004;
	RamRead32A(RamAddr, &UlReadVal);
	*cal_id = UlReadVal & 0xFF;
	CAM_INFO(CAM_SENSOR, "[OISFW]:%s CAL_ID = 0x%04x, MODULE_MAKER = 0x%x",
			__func__, *cal_id, *module_maker);

	if (FW_version >= OIS_CUR_FW_VERSION) {
		CAM_INFO(CAM_SENSOR, "[OISFW]%s: No need to update.", __func__);
		return false;
	} else {
		rc = checkHighLevelCommand(100);
		if (rc != 0) {
			CAM_ERR(CAM_SENSOR,
				"[OISFW]:%s checkHighLevelCommand failed: %d",
				__func__, rc);
			need_update = false;
			return -EINVAL;
		}

		CAM_INFO(CAM_SENSOR, "[OISFW]:%s checkHighLevelCommand = %d",
			__func__, rc);

		need_update = true;
	}

	return need_update;
}

int checkOISFWUpdate(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	int i;
	unsigned short cci_client_sid_backup;
	UINT_32 FWRead;
	UINT_16 FW_version;
	UINT_16 cal_id;
	UINT_32 module_maker;

	if (g_first != true) {
		CAM_INFO(CAM_SENSOR, "[OISFW]%s: No need.", __func__);
		return 0;
	}
	g_first = false;

	CAM_INFO(CAM_SENSOR, "[OISFW]:%s 1. sid = %d", __func__,
		s_ctrl->io_master_info.cci_client->sid);

	/* Bcakup the I2C slave address */
	cci_client_sid_backup = s_ctrl->io_master_info.cci_client->sid;

	/* Replace the I2C slave address with OIS component */
	s_ctrl->io_master_info.cci_client->sid =
		OIS_COMPONENT_I2C_ADDR_WRITE >> 1;

	g_io_master_info = &(s_ctrl->io_master_info);
	WitTim(100);

	if( checkOISFWversion(&cal_id, &module_maker) == true ) {
		rc = doFWupdate(cal_id, module_maker);

		for (i = 0; i < 2 ; i++) {
			RamRead32A(0x8000, &FWRead);
			FW_version = FWRead & 0xFF;
			CAM_INFO(CAM_SENSOR, "[OISFW]:%s 0x8000 =  0x%08x", __func__, FWRead);
			if (FW_version != OIS_CUR_FW_VERSION) {
				CAM_ERR(CAM_SENSOR,
					"[OISFW]:FW version check failed after update. retry.");
				rc = doFWupdate(cal_id, module_maker);
			} else {
				CAM_INFO(CAM_SENSOR,
					"[OISFW]: FW vserion verify pass.");
				break;
			}
		}
	}

	/* Restore the I2C slave address */
	s_ctrl->io_master_info.cci_client->sid =
		cci_client_sid_backup;
	CAM_INFO(CAM_SENSOR, "[OISFW]:%s 2. sid = %d", __func__,
		s_ctrl->io_master_info.cci_client->sid);

	return rc;
}

int checkRearVCMFWUpdate(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	int i = 0;
	UINT_32 retry = 0;
	UINT_32 size = 0;
	UINT_32 UlReadVal;
	UINT_32 VCM_rev = 0;
	UINT_32 regdata = 0;
	bool needupdate = false;
	unsigned short cci_client_sid_backup;
	struct cam_sensor_i2c_reg_array *fwtable = NULL;

	WitTim(300);

	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: original sid = %d", __func__,
		s_ctrl->io_master_info.cci_client->sid);

	cci_client_sid_backup = s_ctrl->io_master_info.cci_client->sid;

	/* 1. Check actuator revision with slave address 0x7C */
	s_ctrl->io_master_info.cci_client->sid =
		OIS_COMPONENT_I2C_ADDR_WRITE >> 1;

	g_io_master_info = &(s_ctrl->io_master_info);

	RamWrite32A(0xF01B, 0x1A02);
	RamRead32A(0xF01B, &UlReadVal);
	VCM_rev = (UlReadVal & 0xFF000000) >> 24;
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: 0x1A02 = 0x%08x, ACM rev = 0x%x",
		__func__, UlReadVal, VCM_rev);

	switch (VCM_rev) {
	case 0:
	case 1:
	case 2:
		CAM_INFO(CAM_SENSOR,
			"[VCMFW]%s: No need to update", __func__);
		break;
	case 3:
		fwtable = VCM_LC898219_EVT_2_1;
		size = sizeof(VCM_LC898219_EVT_2_1) /
			sizeof(struct cam_sensor_i2c_reg_array);
		break;
	default:
		CAM_INFO(CAM_SENSOR,
			"[VCMFW]%s: Unsupported rev", __func__);
		break;
	}

	/* 2. Whether to update or not */
	if (fwtable != NULL) {
		s_ctrl->io_master_info.cci_client->sid =
			VCM_EEPROM_I2C_ADDR_WRITE >> 1;
		for (i = 0; i < LUT_MAX; i++) {
			rc = RamRead8A(&(s_ctrl->io_master_info),
				fwtable[i].reg_addr, &regdata);
			if (rc < 0 || regdata != fwtable[i].reg_data) {
				CAM_INFO(CAM_SENSOR,
					"[VCMFW]%s: run FW update", __func__);
				needupdate = true;
				break;
			}
		}
	}
	if(needupdate == false) {
		CAM_INFO(CAM_SENSOR,
			"[VCMFW]%s: By pass fw update", __func__);
		s_ctrl->io_master_info.cci_client->sid =
			cci_client_sid_backup;
		return 0;
	}

	/* 3. Replace slave address with 0xE4, do AF firmware update */
	s_ctrl->io_master_info.cci_client->sid =
		VCM_COMPONENT_I2C_ADDR_WRITE >> 1;
	WitTim(8);

	for (retry = 0; retry < RETRY_MAX; retry++) {
		rc = RamRead8A(&(s_ctrl->io_master_info), 0xF0, &regdata);
		if (regdata == 0xA5) {
			WitTim(1);
			rc = camera_io_dev_poll(&(s_ctrl->io_master_info),
				0xE0, 0x00, 0, CAMERA_SENSOR_I2C_TYPE_BYTE,
				CAMERA_SENSOR_I2C_TYPE_BYTE, 100);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"[VCMFW]i2c poll 0xE0 failed");
				continue;
			}

			rc = DownloadRearVCMFW(&(s_ctrl->io_master_info),
				fwtable, size);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"[VCMFW]Download firmware failed");
				continue;
			}

			/* 4. Validate EEPROM data*/
			rc = ValidateRearVCMFW(&(s_ctrl->io_master_info),
				fwtable, size);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"[VCMFW]Validate firmware failed");
				continue;
			}
			CAM_INFO(CAM_SENSOR,
				"[VCMFW]%s: FW update success", __func__);
			break;
		} else {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]%s: NG module !", __func__);
			break;
		}
	}

	/* 5. Restore the I2C slave address */
	s_ctrl->io_master_info.cci_client->sid =
		cci_client_sid_backup;
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: restore sid = %d", __func__,
		s_ctrl->io_master_info.cci_client->sid);

	CAM_INFO(CAM_SENSOR, "[VCMFW]%s; rc = %d", __func__, rc);

	return rc;
}

int DownloadRearVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable, UINT_32 tbsize)
{
	int rc = 0;
	int i = 0;
	UINT_32 checkbusy_cnt = 0;
	UINT_32 regdata = 0;
	UINT_32 backup_data = 0;

	/* Backup standby setting */
	rc = RamRead8A(io_info, 0x81, &backup_data);
	if (rc < 0) {
		return rc;
	}
	regdata = backup_data & 0x7F;

	rc = RamWrite8A(io_info, 0x81, regdata);
	if (rc < 0) {
		goto restore_data;
	}
	WitTim(100);

	/* Enable EEPROM write mode */
	rc = RamWrite8A(io_info, 0x98, 0xE2);
	if (rc < 0) {
		goto restore_data;
	}
	rc = RamWrite8A(io_info, 0x99, 0xAE);
	if (rc < 0) {
		goto restore_data;
	}
	WitTim(1);

	/* EEPROM data writing */
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: flash firmware, size %d",
		__func__, tbsize);
	for (i = 0; i < tbsize; i++) {
		checkbusy_cnt = 0;
		io_info->cci_client->sid = VCM_EEPROM_I2C_ADDR_WRITE >> 1;
		rc = RamWrite8A(io_info,
			fwtable[i].reg_addr, fwtable[i].reg_data);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]%s: write data failed, rc = %d",
				__func__, rc);
			goto restore_data;
		}
		io_info->cci_client->sid = VCM_COMPONENT_I2C_ADDR_WRITE >> 1;
		do {
			WitTim(5);
			rc = RamRead8A(io_info, 0xE1, &regdata);
			checkbusy_cnt++;
		} while (((regdata & 0x01) != 0) && checkbusy_cnt < RETRY_MAX);

		if (checkbusy_cnt == RETRY_MAX) {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]%s: write data warning!", __func__);
		}
	}
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: write data complete.", __func__);

restore_data:
	/* Disable EEPROM write mode & restore standby setting */
	io_info->cci_client->sid = VCM_COMPONENT_I2C_ADDR_WRITE >> 1;
	rc = RamWrite8A(io_info, 0x98, 0x00);
	rc = RamWrite8A(io_info, 0x99, 0x00);
	rc = RamWrite8A(io_info, 0x81, backup_data);
	return rc;
}

int ValidateRearVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable, UINT_32 tbsize)
{
	int rc = 0;
	int i = 0;

	io_info->cci_client->sid = VCM_EEPROM_I2C_ADDR_WRITE >> 1;
	for (i = 0; i < tbsize; i++) {
		rc = camera_io_dev_poll(io_info,
			fwtable[i].reg_addr, fwtable[i].reg_data, 0,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"[VCMFW]i2c poll 0x%x failed",
				fwtable[i].reg_addr);
			return -EINVAL;
		}
	}
	CAM_INFO(CAM_SENSOR, "[VCMFW]%s: complete.", __func__);

	return rc;
}

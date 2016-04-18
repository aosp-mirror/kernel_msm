/*
* Copyright © 2016 FocalTech Systems Co., Ltd.  All Rights Reserved.
*
* This program is free software; you may redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_REG_FW_MAJ_VER	0xB1
#define FTS_REG_FW_MIN_VER	0xB2
#define FTS_REG_FW_SUB_MIN_VER	0xB3
#define FTS_FW_MIN_SIZE		8
#define FTS_FW_MAX_SIZE		(54 * 1024)
/* Firmware file is not supporting minor and sub minor so use 0 */
#define FTS_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FTS_FW_FILE_MIN_VER(x)	0
#define FTS_FW_FILE_SUB_MIN_VER(x) 0
#define FTS_FW_FILE_VENDOR_ID(x)	((x)->data[(x)->size - 1])
#define FTS_FW_FILE_MAJ_VER_FT6X36(x)	((x)->data[0x10a])
#define FTS_FW_FILE_VENDOR_ID_FT6X36(x)	((x)->data[0x108])
#define FTS_MAX_TRIES		5
#define FTS_RETRY_DLY		20
#define FTS_MAX_WR_BUF		10
#define FTS_MAX_RD_BUF		2
#define FTS_FW_PKT_META_LEN	6
#define FTS_FW_PKT_DLY_MS	20
#define FTS_FW_LAST_PKT		0x6ffa
#define FTS_EARSE_DLY_MS	100
#define FTS_55_AA_DLY_NS	5000
#define FTS_CAL_START		0x04
#define FTS_CAL_FIN		0x00
#define FTS_CAL_STORE		0x05
#define FTS_CAL_RETRY		100
#define FTS_REG_CAL		0x00
#define FTS_CAL_MASK		0x70
#define FTS_BLOADER_SIZE_OFF	12
#define FTS_BLOADER_NEW_SIZE	30
#define FTS_DATA_LEN_OFF_OLD_FW	8
#define FTS_DATA_LEN_OFF_NEW_FW	14
#define FTS_FINISHING_PKT_LEN_OLD_FW	6
#define FTS_FINISHING_PKT_LEN_NEW_FW	12
#define FTS_MAGIC_BLOADER_Z7	0x7bfa
#define FTS_MAGIC_BLOADER_LZ4	0x6ffa
#define FTS_MAGIC_BLOADER_GZF_30	0x7ff4
#define FTS_MAGIC_BLOADER_GZF	0x7bf4
#define FTS_REG_ECC		0xCC
#define FTS_RST_CMD_REG2	0xBC
#define FTS_READ_ID_REG		0x90
#define FTS_ERASE_APP_REG	0x61
#define FTS_ERASE_PARAMS_CMD	0x63
#define FTS_FW_WRITE_CMD	0xBF
#define FTS_REG_RESET_FW	0x07
#define FTS_RST_CMD_REG1	0xFC
#define FTS_FACTORYMODE_VALUE	0x40
#define FTS_WORKMODE_VALUE	0x00
#define FTS_APP_INFO_ADDR	0xd7f8

#define	BL_VERSION_LZ4        0
#define   BL_VERSION_Z7        1
#define   BL_VERSION_GZF        2
#define   FTS_REG_FW_VENDOR_ID 0xA8

#define FTS_PACKET_LENGTH	128
#define FTS_SETTING_BUF_LEN	128

#define FTS_UPGRADE_LOOP	30
#define FTS_MAX_POINTS_2	2
#define FTS_MAX_POINTS_5	5
#define FTS_MAX_POINTS_10	10
#define AUTO_CLB_NEED		1
#define AUTO_CLB_NONEED		0
#define FTS_UPGRADE_AA		0xAA
#define FTS_UPGRADE_55		0x55
#define HIDTOI2C_DISABLE	0
#define FTXXXX_INI_FILEPATH_CONFIG "/sdcard/"
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char CTPM_FW_PFW2_BLACK[] = {
	/* Old FW
	#include "FT_Upgrade_App.i"
	#include "FT6x36_V0xB3_20150917_app.i"
	#include "PFW2_3207_0x61_0x03_app.i"
	#include "FTS_FW/CEI_PFW2_3207_0x61_0x06_black_1_1mm_20151118_app.i"
	#include "FTS_FW/CEI_PFW2_3207_0x61_0x08_black_1_1mm_20160104_app.i"
	*/
	#include "FTS_FW/CEI_PFW2_3207_0x61_0x09_black_1_1_mm_20160127_app.i"
};

static unsigned char  CTPM_FW_PFW2_GOLD[] = {
	/* Old FW
	#include "FTS_FW/CEI_PFW2_3207_0x62_0x12_gold_1_1_mm_20151118_app.i"
	#include "FTS_FW/CEI_PFW2_3207_0x62_0x14_gold_1_1_mm_20160104_app.i"
	*/
	#include "FTS_FW/CEI_PFW2_3207_0x62_0x15_gold_1_1_mm_20160127_app.i"
};

static unsigned char  CTPM_FW_PFW2_ROSE_GOLD[] = {
	/* Old FW
	#include "FTS_FW/CEI_PFW2_3207_0x63_0x32_rose_gold_1_1mm_20151118_app.i"
	#include "FTS_FW/CEI_PFW2_3207_0x63_0x34_rose_gold_1_1mm_20160104_app.i"
	*/
	#include "FTS_FW/CEI_PFW2_3207_0x63_0x35_rose_gold_1_1_mm_20160127_app.i"
};

static unsigned char CTPM_FW_PFW2_SILVER[] = {
	/* Old FW
	#include "FTS_FW/CEI_PFW2_3207_0x64_0x42_silver_1_1_mm_20151118_app.i"
	#include "FTS_FW/CEI_PFW2_3207_0x64_0x44_silver_1_1_mm_20160104_app.i"
	*/
	#include "FTS_FW/CEI_PFW2_3207_0x64_0x45_silver_1_1_mm_20160127_app.i"
};

static unsigned char CTPM_FW_PFW2_SABLE[] = {
	#include "FTS_FW/CEI_PFW2_3207_0x68_0x20_sable_1_1_mm_20160225_app.i"
};

static unsigned char CTPM_FW_PFW2_NAVY[] = {
	#include "FTS_FW/CEI_PFW2_3207_0x69_0x80_navy_1_1_mm_20160225_app.i"
};

static unsigned char CTPM_FW_PFW3_BLACK[] = {
	/* Old FW
	#include "FTS_FW/CEI_PFW3_3207_0x67_0x72_black_1_1_mm_2_5D_R0_2_20151118_app.i"
	#include "FTS_FW/CEI_PFW3_3207_0x67_0x74_black_1_1_mm_2_5D_R0_2_20160104_app.i"
	*/
	#include "FTS_FW/CEI_PFW3_3207_0x67_0x75_black_1_1_mm_2_5D_R0_2_20160127_app.i"
};

static unsigned char CTPM_FW_PFW4_BLACK[] = {
	/* Old FW
	#include "FTS_FW/CEI_PFW4_3207_0x65_0x52_black_1_1_mm_2_5D_R0_25_20151118_app.i"
	#include "FTS_FW/CEI_PFW4_3207_0x65_0x54_black_1_1_mm_2_5D_R0_25_20160104_app.i"
	*/
	#include "FTS_FW/CEI_PFW4_3207_0x65_0x55_black_1_1_mm_2_5D_R0_25_20160127_app.i"
};

static unsigned char CTPM_FW_PFW5_BLACK[] = {
	/* Old FW
	#include "FTS_FW/CEI_PFW5_3207_0x66_0x62_black_1_8_mm_step_20151118_app.i"
	#include "FTS_FW/CEI_PFW5_3207_0x66_0x64_black_1_8_mm_step_20160104_app.i"
	*/
	#include "FTS_FW/CEI_PFW5_3207_0x66_0x65_black_1_8_mm_step_20160127_app.i"
};
/*static unsigned char aucFW_PRAM_BOOT[] = {
	#include "FT8606_Pramboot_V0.6_20150304.i"
};*/

struct fts_Upgrade_Info fts_updateinfo[] = {
	{0x55, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 10, 2000},
	{0x08, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 10, 0x79, 0x06, 100, 2000},
	{0x0a, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x07, 10, 1500},
	{0x06, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 100, 30, 0x79, 0x08, 10, 2000},
	{0x36, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x18, 10, 2000},
	{0x64, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x1c, 10, 2000},
	{0x55, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 10, 2000},
	{0x14, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x13, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x12, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x11, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},
	{0x54, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 2, 2, 0x54, 0x2c, 20, 2000},
	{0x58, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 2, 2, 0x58, 0x2c, 20, 2000},
	{0x59, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 30, 50, 0x79, 0x10, 1, 2000},
	{0x86, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 2, 2, 0x86, 0xA6, 20, 2000},
	/* FT3X07 */
	{0x0e, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x18, 10, 2000},
};
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct fts_Upgrade_Info fts_updateinfo_curr;

/*The newest firmware, if update must be changed here*/
u8 *CTPM_FW = NULL;
u16 fw_size = 0;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/

/*******************************************************************************
* Name: fts_update_fw_vendor_id
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
void fts_update_fw_vendor_id(struct fts_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FTS_REG_FW_VENDOR_ID;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	if (err < 0)
		dev_err(&client->dev, "fw vendor id read failed");
}

/*******************************************************************************
* Name: fts_update_fw_ver
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
void fts_update_fw_ver(struct fts_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FTS_REG_FW_VER;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");
	/*
	reg_addr = FTS_REG_FW_MIN_VER;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FTS_REG_FW_SUB_MIN_VER;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");
	*/

	dev_info(&client->dev, "Firmware version = 0x%x\n",
		data->fw_ver[0]);
}

/************************************************************************
* Name: fts_get_upgrade_array
* Brief: decide which ic
* Input: no
* Output: get ic info in fts_updateinfo_curr
* Return: no
***********************************************************************/
void fts_get_upgrade_array(void)
{
	u8 chip_id;
	int ret = 0;

	ret = fts_read_reg(fts_i2c_client, FTS_REG_ID, &chip_id);
	if (ret < 0)
		printk("[FTS] read chip id fail\n");

	printk("%s chip_id = %x\n", __func__, chip_id);

	memcpy(&fts_updateinfo_curr, &fts_updateinfo[15], sizeof(struct fts_Upgrade_Info));
}

/************************************************************************
* Name: fts_ctpm_auto_clb
* Brief:  auto calibration
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_clb(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	fts_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	fts_write_reg(client, 2, 0x4);
	msleep(300);
	if ((fts_updateinfo_curr.CHIP_ID == 0x11) ||
		(fts_updateinfo_curr.CHIP_ID == 0x12) ||
		(fts_updateinfo_curr.CHIP_ID == 0x13) ||
		(fts_updateinfo_curr.CHIP_ID == 0x14)) {
		for (i = 0 ; i < 100 ; i++) {
			fts_read_reg(client, 0x02, &uc_temp);
			if (0x02 == uc_temp || 0xFF == uc_temp) {
				break;
			}
			msleep(20);
		}
	} else {
		for (i = 0 ; i < 100 ; i++) {
			fts_read_reg(client, 0, &uc_temp);
			if (0x0 == ((uc_temp & 0x70) >> 4))
				break;

			msleep(20);
		}
	}
	fts_write_reg(client, 0, 0x40);
	msleep(200);
	fts_write_reg(client, 2, 0x5);
	msleep(300);
	fts_write_reg(client, 0, FTS_WORKMODE_VALUE);
	msleep(300);
	return 0;
}

void FW_upgrade_finish(void)
{
	fts_wq_data->loading_fw = false;

	if (fts_wq_data->suspending) {
		fts_ts_stop();
		fts_wq_data->suspending = false;
	}
}

/************************************************************************
* Name: fts_3x27_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_3x07_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;

	int err = 0;

	fts_wq_data->loading_fw = true;

	if (fts_wq_data->suspended) {
		FTS_DBG("[FTS]ready in suspend state, cannot FW upgrade\n");
		err = -1;
		goto FW_upgrade_fail;
	}

	if (pbt_buf[0] != 0x02) {
		FTS_DBG("[FTS] FW first byte is not 0x02. so it is invalid\n");
		err = -1;
		goto FW_upgrade_fail;
	}

	if (dw_lenth > 0x11f) {
		fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
		if (dw_lenth < fw_length) {
			FTS_DBG("[FTS] Fw length is invalid\n");
			err = -1;
			goto FW_upgrade_fail;
		}
	} else {
		FTS_DBG("[FTS] Fw length is invalid\n");
		err = -1;
		goto FW_upgrade_fail;
	}

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		auc_i2c_write_buf[0] = FTS_UPGRADE_AA;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(fts_updateinfo_curr.delay_readid);
		/*********Step 3:check READ-ID***********************/
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP) {
		err = -EIO;
		goto FW_upgrade_fail;
	}

	auc_i2c_write_buf[0] = FTS_READ_ID_REG;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	fts_i2c_write(client, auc_i2c_write_buf, 5);

	/*Step 4:erase app and panel paramenter area*/
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(fts_updateinfo_curr.delay_erase_flash);

	for (i = 0 ; i < 200 ; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (0xb0 == reg_val[0] && 0x02 == reg_val[1]) {
			FTS_DBG("[FTS] erase app finished\n");
			break;
		}
		msleep(50);
	}

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = fw_length;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

		for (i = 0 ; i < 30 ; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if (0xb0 == (reg_val[0] & 0xf0) &&
				(0x03 + (j % 0x0ffd)) ==
				(((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("[FTS] write a block data finished\n");
				break;
			}
			msleep(1);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);

		for (i = 0 ; i < 30 ; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if (0xb0 == (reg_val[0] & 0xf0) &&
				(0x03 + (j % 0x0ffd)) ==
				(((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("[FTS] write a block data finished\n");
				break;
			}
			msleep(1);
		}
	}


	/*********Step 6: read out checksum***********************/
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		err = -EIO;
		goto FW_upgrade_fail;
	}

	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	fts_update_fw_ver(fts_wq_data);

FW_upgrade_fail:

	FW_upgrade_finish();
	return err;
}

/*
*note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
/************************************************************************
* Name: fts_GetFirmwareSize
* Brief:  get file size
* Input: file name
* Output: no
* Return: file size
***********************************************************************/
static int fts_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, 128, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

/************************************************************************
* Name: fts_ReadFirmware
* Brief:  read firmware buf for .bin file.
* Input: file name, data buf
* Output: data buf
* Return: 0
***********************************************************************/
/*
note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int fts_ReadFirmware(char *firmware_name, unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, 128, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_with_app_file
* Brief:  upgrade with *.bin file
* Input: i2c info, file name
* Output: no
* Return: success =0
***********************************************************************/
int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret = 0;
	int fwsize = fts_GetFirmwareSize(firmware_name);
	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n", __func__);
		return -EIO;
	}
	if (fwsize < 8 || fwsize > 54 * 1024) {
		dev_err(&client->dev, "FW length error\n");
		return -EIO;
	}

	dev_err(&client->dev, "dean tag CHIP_ID : 0x%x\n", fts_updateinfo_curr.CHIP_ID);
	/*=========FW upgrade========================*/
	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_ATOMIC);
	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n", __func__);
		kfree(pbt_buf);
		return -EIO;
	}
	if ((fts_updateinfo_curr.CHIP_ID == 0x0e))
		i_ret = fts_3x07_ctpm_fw_upgrade(client, pbt_buf, fwsize);

	if (i_ret != 0)
		dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n", __func__);
	else if (fts_updateinfo_curr.AUTO_CLB == AUTO_CLB_NEED)
		fts_ctpm_auto_clb(client);

	kfree(pbt_buf);

	return i_ret;
}
/************************************************************************
* Name: fts_ctpm_get_i_file_ver
* Brief:  get .i file version
* Input: no
* Output: no
* Return: fw version
***********************************************************************/
int fts_ctpm_get_i_file_ver(void)
{
	u16 ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2) {
		if (fts_updateinfo_curr.CHIP_ID == 0x36  || fts_updateinfo_curr.CHIP_ID == 0x86  || fts_updateinfo_curr.CHIP_ID == 0x64)
			return CTPM_FW[0x10a];
		else if (fts_updateinfo_curr.CHIP_ID == 0x58)
			return CTPM_FW[0x1D0A];
		else
			return CTPM_FW[ui_sz - 2];
	}

	return 0x00;
}

int fts_ctpm_get_i_file_ver_for_cci(void)
{
	u16 ui_sz = fw_size;

	if (ui_sz > 2)
		return CTPM_FW[ui_sz - 2];
	else
		return 0x00;
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_with_i_file
* Brief:  upgrade with *.i file
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
	u8 *pbt_buf = NULL;
	int i_ret = 0;
	int fw_len = sizeof(CTPM_FW);

	if ((fts_updateinfo_curr.CHIP_ID == 0x0e)) {
		if (fw_len < 8 || fw_len > 32 * 1024) {
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		pbt_buf = CTPM_FW;
		i_ret = fts_3x07_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n", __func__);
	}

	return i_ret;
}

int fts_ctpm_fw_upgrade_with_i_file_for_cci_3207(struct i2c_client *client)
{
	int i_ret = -1;
	int fw_len = fw_size;

	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
	if (fts_updateinfo_curr.CHIP_ID == 0x0e) {
		if (fw_len < 8 || fw_len > 32 * 1024) {
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		i_ret = fts_3x07_ctpm_fw_upgrade(client, CTPM_FW, fw_size);
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n", __func__);
	} else {
		dev_err(&client->dev, "%s:CHIP_ID is wrong. 0x%x.\n", __func__, fts_updateinfo_curr.CHIP_ID);
	}

	return i_ret;
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade_for_cci
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_upgrade_for_cci(struct i2c_client *client, const u8 tp_id, bool force_upgrade)
{
	u8 uc_host_fm_ver = FTS_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int i_ret;


	switch (tp_id) {
	case TP_ID_PFW2_BLACK_ORIGIN:
	case TP_ID_PFW2_BLACK:
		CTPM_FW = CTPM_FW_PFW2_BLACK;
		fw_size = sizeof(CTPM_FW_PFW2_BLACK);
	break;
	case TP_ID_PFW2_GOLD:
		CTPM_FW = CTPM_FW_PFW2_GOLD;
		fw_size = sizeof(CTPM_FW_PFW2_GOLD);
	break;
	case TP_ID_PFW2_ROSE_GOLD:
		CTPM_FW = CTPM_FW_PFW2_ROSE_GOLD;
		fw_size = sizeof(CTPM_FW_PFW2_ROSE_GOLD);
	break;
	case TP_ID_PFW2_SILVER:
		CTPM_FW = CTPM_FW_PFW2_SILVER;
		fw_size = sizeof(CTPM_FW_PFW2_SILVER);
	break;
	case TP_ID_PFW2_SABLE:
		CTPM_FW = CTPM_FW_PFW2_SABLE;
		fw_size = sizeof(CTPM_FW_PFW2_SABLE);
	break;
	case TP_ID_PFW2_NAVY:
		CTPM_FW = CTPM_FW_PFW2_NAVY;
		fw_size = sizeof(CTPM_FW_PFW2_NAVY);
	break;
	case TP_ID_PFW3_BLACK:
		CTPM_FW = CTPM_FW_PFW3_BLACK;
		fw_size = sizeof(CTPM_FW_PFW3_BLACK);
	break;
	case TP_ID_PFW4_BLACK:
		CTPM_FW = CTPM_FW_PFW4_BLACK;
		fw_size = sizeof(CTPM_FW_PFW4_BLACK);
	break;
	case TP_ID_PFW5_BLACK:
		CTPM_FW = CTPM_FW_PFW5_BLACK;
		fw_size = sizeof(CTPM_FW_PFW5_BLACK);
	break;
	default:
		pr_err("[fts]tp is not correct\n");
		return -EIO;
	break;
	}

	fts_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver_for_cci();
	FTS_DBG("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n", uc_tp_fm_ver, uc_host_fm_ver);

	if (uc_tp_fm_ver < uc_host_fm_ver || force_upgrade) {
		msleep(100);
		i_ret = fts_ctpm_fw_upgrade_with_i_file_for_cci_3207(client);
		if (i_ret == 0) {
			msleep(300);
			FTS_DBG("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
		} else {
			pr_err("[FTS] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}
	} else {
		FTS_DBG("[FTS] version is newest, no need upgrade FW\n");
		return 1;
	}
	return 0;
}

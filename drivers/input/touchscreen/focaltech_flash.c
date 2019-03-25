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

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
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

#define	BL_VERSION_LZ4	0
#define	BL_VERSION_Z7	1
#define	BL_VERSION_GZF	2

#define	FTS_REG_FW_VENDOR_ID 0xA8

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

static unsigned char CTPM_FW_TP_ID_T[] = {
#include "FTS_FW/CEI_TULIP_FZW6_3267_0x98_V0x03_20180515_app.i"
};

static unsigned char CTPM_FW_TP_ID_S[] = {
#include "FTS_FW/CEI_Sakura_FZW7_3267_0x99_V0x08_20181108_app.i"
};

static unsigned char CTPM_FW_TP_ID_S_TRULY[] = {
#include "FTS_FW/MOBVOI_TIC3_FT3267_JINLONG_Ver0x_20180321_app.i"
};

static unsigned char CTPM_FW_TP_ID_S2[] = {
#include "FTS_FW/CEI_Sakura_FZW7_3267_0x9D_V0x01_20190221_app.i"
};

struct fts_Upgrade_Info fts_updateinfo[] = {
	{0x33, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x1c, 10, 2000},	/*,"FT3267"*/
	{0x55, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 10, 2000},	/*,"FT5x06"*/
	{0x08, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 10, 0x79, 0x06, 100, 2000},	/*,"FT5606"*/
	{0x0a, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x07, 10, 1500},	/*,"FT5x16"*/
	{0x06, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 100, 30, 0x79, 0x08, 10, 2000},	/*,"FT6x06"*/
	{0x36, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x18, 10, 2000},	/*,"FT6x36"*/
	{0x64, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x1c, 10, 2000},	/*,"FT6336GU"*/
	{0x55, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 10, 2000},	/*,"FT5x06i"*/
	{0x14, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},	/*,"FT5336"*/
	{0x13, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},	/*,"FT3316"*/
	{0x12, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},	/*,"FT5436i"*/
	{0x11, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},	/*,"FT5336i"*/
	{0x54, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 2, 2, 0x54, 0x2c, 20, 2000},	/*,"FT5x46"*/
	{0x58, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 2, 2, 0x58, 0x2c, 20, 2000},	/*"FT5822",*/
	{0x59, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 30, 50, 0x79, 0x10, 1, 2000},	/*"FT5x26",*/
	{0x86, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 2, 2, 0x86, 0xA6, 20, 2000},	/*"FT8606",*/
	{0x0e, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x18, 10, 2000},	/*,"FT3X07"*/
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
int fts_ft6336gu_upgrade(struct i2c_client *client, u8 *buf, u32 len);
int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_6336GU_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			       u32 dw_lenth);
int fts_6x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_5x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_5x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_5x46_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_5822_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_5x26_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_8606_writepram(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_8606_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int fts_3x07_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth);
int hidi2c_to_stdi2c(struct i2c_client *client);

/************************************************************************
* Name: hidi2c_to_stdi2c
* Brief:  HID to I2C
* Input: i2c info
* Output: no
* Return: fail =0
***********************************************************************/
int hidi2c_to_stdi2c(struct i2c_client *client)
{
	u8 auc_i2c_write_buf[5] = { 0 };
	int bRet = 0;
#if HIDTOI2C_DISABLE
	return 0;
#endif

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;
	bRet = fts_i2c_write(client, auc_i2c_write_buf, 3);
	msleep(10);
	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;
	fts_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

	if (0xeb == auc_i2c_write_buf[0] && 0xaa == auc_i2c_write_buf[1]
	    && 0x08 == auc_i2c_write_buf[2]) {
		pr_info("hidi2c_to_stdi2c successful.\n");
		bRet = 1;
	} else {
		pr_err("hidi2c_to_stdi2c error.\n");
		bRet = 0;
	}

	return bRet;
}

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

	dev_info(&client->dev, "Firmware version = 0x%x\n", data->fw_ver[0]);
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadVendorID
* Brief:  read vendor ID
* Input: i2c info, vendor ID
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadVendorID(struct i2c_client *client,
				     u8 *ucPVendorID)
{
	u8 reg_val[4] = { 0 };
	u32 i = 0;
	u8 auc_i2c_write_buf[10];
	int i_ret;

	*ucPVendorID = 0;
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0)
		FTS_DBG("HidI2c change to StdI2c fail !\n");

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);
		if (i_ret == 0)
			FTS_DBG("HidI2c change to StdI2c fail !\n");

		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_DBG("failed writing  0x55 and 0xaa !\n");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/*********Step 4: read vendor id from app param area***********************/
	msleep(10);
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0xd7;
	auc_i2c_write_buf[3] = 0x84;
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		fts_i2c_write(client, auc_i2c_write_buf, 4);
		msleep(5);
		reg_val[0] = reg_val[1] = 0x00;
		i_ret = fts_i2c_read(client, auc_i2c_write_buf, 0, reg_val, 2);
		if (0 != reg_val[0]) {
			*ucPVendorID = 0;
			FTS_DBG
			    ("In upgrade Vendor ID Mismatch, REG1 = 0x%x, REG2 = 0x%x, Definition:0x%x, i_ret=%d\n",
			     reg_val[0], reg_val[1], 0, i_ret);
		} else {
			*ucPVendorID = reg_val[0];
			FTS_DBG
			    ("In upgrade Vendor ID, REG1 = 0x%x, REG2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		}
	}
	msleep(50);
	/*********Step 5: reset the new FW***********************/
	FTS_DBG("Step 5: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0)
		FTS_DBG("HidI2c change to StdI2c fail !\n");

	msleep(10);
	return 0;
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadProjectCode
* Brief:  read project code
* Input: i2c info, project code
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadProjectCode(struct i2c_client *client,
					char *pProjectCode)
{
	u8 reg_val[4] = { 0 };
	u32 i = 0;
	u8 j = 0;
	u8 auc_i2c_write_buf[10];
	int i_ret;
	u32 temp;

	i_ret = hidi2c_to_stdi2c(client);

	if (i_ret == 0)
		FTS_DBG("HidI2c change to StdI2c fail !\n");

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);
		if (i_ret == 0)
			FTS_DBG("HidI2c change to StdI2c fail !\n");

		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_DBG("failed writing  0x55 and 0xaa !\n");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/*********Step 4: read vendor id from app param area***********************/
	msleep(10);
	/*read project code */
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	for (j = 0; j < 33; j++) {
		temp = 0xD7A0 + j;
		auc_i2c_write_buf[2] = (u8) (temp >> 8);
		auc_i2c_write_buf[3] = (u8) temp;
		fts_i2c_read(client, auc_i2c_write_buf, 4, pProjectCode + j, 1);
		if (*(pProjectCode + j) == '\0')
			break;
	}
	pr_info("project code = %s\n", pProjectCode);
	msleep(50);
	/*********Step 5: reset the new FW***********************/
	FTS_DBG("Step 5: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0)
		FTS_DBG("HidI2c change to StdI2c fail !\n");

	msleep(10);
	return 0;
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
	u32 i;
	int ret = 0;

	ret = fts_read_reg(fts_i2c_client, FTS_REG_ID, &chip_id);
	if (ret < 0)
		printk(KERN_ERR "[fts]read value fail\n");

	printk(KERN_ERR "[fts] %s chip_id = %x\n", __func__, chip_id);

	for (i = 0;
	     i < sizeof(fts_updateinfo) / sizeof(struct fts_Upgrade_Info);
	     i++) {
		if (chip_id == fts_updateinfo[i].CHIP_ID) {
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i],
			       sizeof(struct fts_Upgrade_Info));
			break;
		}
	}

	if (i >= sizeof(fts_updateinfo) / sizeof(struct fts_Upgrade_Info))
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[15],
		       sizeof(struct fts_Upgrade_Info));
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
	/* 5x36,5x36i */
	if ((fts_updateinfo_curr.CHIP_ID == 0x11) || (fts_updateinfo_curr.CHIP_ID == 0x12) || (fts_updateinfo_curr.CHIP_ID == 0x13) || (fts_updateinfo_curr.CHIP_ID == 0x14))	{
		for (i = 0; i < 100; i++) {
			fts_read_reg(client, 0x02, &uc_temp);
			if (0x02 == uc_temp || 0xFF == uc_temp)
				break;

			msleep(20);
		}
	} else {
		for (i = 0; i < 100; i++) {
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

/************************************************************************
* Name: fts_6x36_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[2] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;

	if (pbt_buf[0] != 0x02) {
		FTS_DBG("FW first byte is not 0x02. so it is invalid\n");
		return -EIO;
	}

	if (dw_lenth > 0x11f) {
		fw_length = ((u32) pbt_buf[0x100] << 8) + pbt_buf[0x101];
		if (dw_lenth < fw_length) {
			FTS_DBG("Fw length is invalid\n");
			return -EIO;
		}
	} else {
		FTS_DBG("Fw length is invalid\n");
		return -EIO;
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
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}

	if (i >= FTS_UPGRADE_LOOP) {
		dev_err(&client->dev,
			"[fts] i >= FTS_UPGRADE_LOOP, return EIO\n");
		return -EIO;
	}

	auc_i2c_write_buf[0] = FTS_READ_ID_REG;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	fts_i2c_write(client, auc_i2c_write_buf, 5);

	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(fts_updateinfo_curr.delay_erase_flash);

	for (i = 0; i < 200; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (0xb0 == reg_val[0] && 0x02 == reg_val[1]) {
			FTS_DBG("erase app finished\n");
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

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if (0xb0 == (reg_val[0] & 0xf0)
			    && (0x03 + (j % 0x0ffd)) ==
			    (((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("write a block data finished\n");
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
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if (0xb0 == (reg_val[0] & 0xf0)
			    && (0x03 + (j % 0x0ffd)) ==
			    (((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("write a block data finished\n");
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
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	return 0;
}

/************************************************************************
* Name: fts_i2c_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	u8 buf[2] = { 0 };

	buf[0] = regaddr;
	buf[1] = regvalue;
	return fts_i2c_write(client, buf, sizeof(buf));
}

/************************************************************************
* Name: fts_i2c_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(struct i2c_client *client)
{
	int ret = 0;
	int cnt = 0;
	u8 reg_value = 0;
	/* struct fts_ts_data *fts_data  =
		(struct fts_ts_data *)i2c_get_clientdata(client); */

	u8 chip_id = fts_updateinfo_curr.CHIP_ID;

	do {
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);

		if ((ret < 0) || (reg_value != chip_id))
			pr_info("[fts]TP Not Ready, ReadData = 0x%x\n",
			       reg_value);
		else if (reg_value == chip_id) {
			pr_info("[fts]TP Ready, Device ID = 0x%x\n",
			       reg_value);
			return 0;
		}

		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	return -EIO;
}

/************************************************************************
* Name: fts_fwupg_check_fw_valid
* Brief: check fw in tp is valid or not
* Input:
* Output:
* Return: return true if fw is valid, otherwise return false
***********************************************************************/
bool fts_fwupg_check_fw_valid(struct i2c_client *client)
{
	int ret = 0;

	ret = fts_wait_tp_to_valid(client);
	if (ret < 0) {
		printk(KERN_INFO "tp fw invaild");
		return false;
	}

	printk(KERN_INFO "tp fw vaild");
	return true;
}

int fts_ft6336gu_reset_to_boot(struct i2c_client *client)
{
	int ret = 0;

	printk(KERN_INFO "send 0xAA and 0x55 to FW, reset to boot environment");

	ret = fts_i2c_write_reg(client, FTS_REG_UPGRADE2, FTS_UPGRADE_AA);
	if (ret < 0) {
		printk(KERN_ERR "write FC=0xAA fail");
		return ret;
	}
	msleep(FTS_DELAY_FC_AA);

	ret = fts_i2c_write_reg(client, FTS_REG_UPGRADE2, FTS_UPGRADE_55);
	if (ret < 0) {
		printk(KERN_ERR "write FC=0x55 fail");
		return ret;
	}

	msleep(FTS_DELAY_UPGRADE_RESET);
	return 0;
}

/************************************************************************
* Name: fts_fwupg_get_boot_state
* Brief: read boot id(rom/pram/bootloader), confirm boot environment
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
int fts_fwupg_get_boot_state(struct i2c_client *client, enum FW_STATUS *fw_sts)
{
	int ret = 0;
	u8 cmd[4] = { 0 };
	u32 cmd_len = 0;
	u8 val[2] = { 0 };
	struct fts_ts_data *fts_data =
	    (struct fts_ts_data *)i2c_get_clientdata(client);

	/* struct ft_chip_t ids = fts_data->ic_info.ids;

	   struct fts_upgrade *upg = fwupgrade;

	   printk(KERN_INFO "**********read boot id**********");
	   if ((NULL == fw_sts) || (NULL == upg) || (NULL == upg->func)) {
	   printk(KERN_ERR "upgrade/func/fw_sts is null");
	   return -EINVAL;
	   }

	   if (upg->func->hid_supported)
	   fts_i2c_hid2std(client);
	 */

	cmd[0] = FTS_CMD_START1;
	cmd[1] = FTS_CMD_START2;
	ret = fts_i2c_write(client, cmd, 2);
	if (ret < 0) {
		printk(KERN_ERR "write 55 aa cmd fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	cmd[0] = FTS_CMD_READ_ID;
	cmd[1] = cmd[2] = cmd[3] = 0x00;
	if (fts_data->ic_info.is_incell)
		cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
	else
		cmd_len = FTS_CMD_READ_ID_LEN;
	ret = fts_i2c_read(client, cmd, cmd_len, val, 2);
	if (ret < 0) {
		printk(KERN_ERR "write 90 cmd fail");
		return ret;
	}

	printk(KERN_INFO "read boot id:0x%02x%02x", val[0], val[1]);
	/*
	   if ((val[0] == ids.rom_idh) && (val[1] == ids.rom_idl)) {
	   printk(KERN_INFO "tp run in romboot");
	   *fw_sts = FTS_RUN_IN_ROM;
	   } else if ((val[0] == ids.pb_idh) && (val[1] == ids.pb_idl)) {
	   printk(KERN_INFO "tp run in pramboot");
	   *fw_sts = FTS_RUN_IN_PRAM;
	   } else if ((val[0] == ids.bl_idh) && (val[1] == ids.bl_idl)) {
	   printk(KERN_INFO "tp run in bootloader");
	   *fw_sts = FTS_RUN_IN_BOOTLOADER;
	   }
	 */

	if ((val[0] == 0x64) && (val[1] == 0x26)) {
		printk(KERN_INFO "tp run in romboot");
		*fw_sts = FTS_RUN_IN_ROM;
	} else if ((val[0] == 0) && (val[1] == 0)) {
		printk(KERN_INFO "tp run in pramboot");
		*fw_sts = FTS_RUN_IN_PRAM;
	} else if ((val[0] == 0x79) && (val[1] == 0x1C)) {
		printk(KERN_INFO "tp run in bootloader");
		*fw_sts = FTS_RUN_IN_BOOTLOADER;
	}

	return 0;
}

/************************************************************************
* Name: fts_fwupg_check_state
* Brief: confirm tp run in romboot/pramboot/bootloader
* Input:
* Output:
* Return: return true if state is match, otherwise return false
***********************************************************************/
bool fts_fwupg_check_state(struct i2c_client *client, enum FW_STATUS rstate)
{
	int ret = 0;
	int i = 0;
	enum FW_STATUS cstate = FTS_RUN_IN_ERROR;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		ret = fts_fwupg_get_boot_state(client, &cstate);
		/* FTS_DEBUG("fw state=%d, retries=%d", cstate, i); */
		if (cstate == rstate)
			return true;
		msleep(FTS_DELAY_READ_ID);
	}

	return false;
}

/************************************************************************
* Name: fts_ft6336gu_check_flash_status
* Brief:
* Input: flash_status: correct value from tp
*        retries: read retry times
*        retries_delay: retry delay
* Output:
* Return: return true if flash status check pass, otherwise return false
***********************************************************************/
static bool fts_ft6336gu_check_flash_status(struct i2c_client *client,
					    u16 flash_status,
					    int retries, int retries_delay)
{
	int ret = 0;
	int i = 0;
	u8 cmd[4] = { 0 };
	u8 val[FTS_CMD_FLASH_STATUS_LEN] = { 0 };
	u16 read_status = 0;

	for (i = 0; i < retries; i++) {
		/* w 6a 00 00 00 r 2 */
		cmd[0] = FTS_CMD_FLASH_STATUS;
		ret =
		    fts_i2c_read(client, cmd, 4, val, FTS_CMD_FLASH_STATUS_LEN);
		read_status = (((u16) val[0]) << 8) + val[1];

		if (flash_status == read_status) {
			/* FTS_DEBUG("[UPGRADE]flash status ok"); */
			return true;
		}
		printk(KERN_DEBUG
		       "[fts] flash status fail,ok:%04x read:%04x, retries:%d\n",
		       flash_status, read_status, i);
		msleep(retries_delay);
	}

	return false;
}

/************************************************************************
* Name: fts_ft6336gu_erase
* Brief:
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_ft6336gu_erase(struct i2c_client *client)
{
	int ret = 0;
	u8 cmd = 0;
	bool flag = false;

	printk(KERN_INFO "**********erase app now**********\n");

	/*send to erase flash */
	cmd = FTS_CMD_ERASE_APP;
	ret = fts_i2c_write(client, &cmd, 1);
	if (ret < 0) {
		printk(KERN_ERR "[fts] erase cmd fail\n");
		return ret;
	}
	msleep(500);

	/* read status 0xF0AA: success */
	flag =
	    fts_ft6336gu_check_flash_status(client,
					    FTS_FLASH_STATUS_OK_FT6336GU, 50,
					    100);
	if (!flag) {
		printk(KERN_ERR "[fts] ecc flash status check fail\n");
		return -EIO;
	}

	return 0;
}

/************************************************************************
* Name: fts_ft6336gu_write_app
* Brief:
* Input:
* Output:
* Return: return ecc if success, otherwise return error code
***********************************************************************/
static int fts_ft6336gu_write_app(struct i2c_client *client, u32 start_addr,
				  u8 *buf, u32 len)
{
	int ret = 0;
	u32 i = 0;
	u32 j = 0;
	u32 packet_number = 0;
	u32 packet_len = 0;
	u32 addr = 0;
	u32 offset = 0;
	u32 remainder = 0;
	u8 packet_buf[FTS_FLASH_PACKET_LENGTH + FTS_CMD_WRITE_LEN] = { 0 };
	u8 ecc_in_host = 0;
	u8 cmd[4] = { 0 };
	u8 val[FTS_CMD_FLASH_STATUS_LEN] = { 0 };
	u16 read_status = 0;
	u16 wr_ok = 0;

	printk(KERN_INFO "**********write app to flash**********");

	if (NULL == buf) {
		printk(KERN_ERR "[fts] buf is null\n");
		return -EINVAL;
	}

	printk(KERN_INFO "[fts] tp fw len=%d\n", len);
	packet_number = len / FTS_FLASH_PACKET_LENGTH;
	remainder = len % FTS_FLASH_PACKET_LENGTH;
	if (remainder > 0)
		packet_number++;

	packet_len = FTS_FLASH_PACKET_LENGTH;
	printk(KERN_INFO "[fts] write fw,num:%d, remainder:%d\n",
	       packet_number, remainder);

	packet_buf[0] = FTS_CMD_WRITE;
	for (i = 0; i < packet_number; i++) {
		offset = i * FTS_FLASH_PACKET_LENGTH;
		addr = start_addr + offset;
		packet_buf[1] = BYTE_OFF_16(addr);
		packet_buf[2] = BYTE_OFF_8(addr);
		packet_buf[3] = BYTE_OFF_0(addr);

		/* last packet */
		if ((i == (packet_number - 1)) && remainder)
			packet_len = remainder;

		packet_buf[4] = BYTE_OFF_8(packet_len);
		packet_buf[5] = BYTE_OFF_0(packet_len);

		for (j = 0; j < packet_len; j++) {
			packet_buf[FTS_CMD_WRITE_LEN + j] = buf[offset + j];
			ecc_in_host ^= packet_buf[FTS_CMD_WRITE_LEN + j];
		}

		ret =
		    fts_i2c_write(client, packet_buf,
				  packet_len + FTS_CMD_WRITE_LEN);
		if (ret < 0) {
			printk(KERN_ERR "[fts][UPGRADE]app write fail\n");
			return ret;
		}
		mdelay(1);

		/* read status */
		wr_ok = FTS_FLASH_STATUS_OK_FT6336GU + i + 1;
		for (j = 0; j < FTS_RETRIES_WRITE; j++) {
			cmd[0] = FTS_CMD_FLASH_STATUS;
			cmd[1] = 0x00;
			cmd[2] = 0x00;
			cmd[3] = 0x00;
			ret =
			    fts_i2c_read(client, cmd, 4, val,
					 FTS_CMD_FLASH_STATUS_LEN);
			read_status = (((u16) val[0]) << 8) + val[1];

			if (wr_ok == read_status)
				break;

			mdelay(FTS_RETRIES_DELAY_WRITE);
		}
	}

	return (int)ecc_in_host;
}

/************************************************************************
* Name: fts_ft6336gu_ecc_cal
* Brief:
* Input:
* Output:
* Return: return ecc if success, otherwise return error code
***********************************************************************/
static int fts_ft6336gu_ecc_cal(struct i2c_client *client)
{
	int ret = 0;
	u8 reg_val = 0;
	u8 cmd = 0;

	printk(KERN_INFO "[fts] read out ecc\n");

	cmd = 0xcc;
	ret = fts_i2c_read(client, &cmd, 1, &reg_val, 1);
	if (ret < 0) {
		printk(KERN_ERR "[fts] read ft6336gu ecc fail\n");
		return ret;
	}

	return reg_val;
}

/************************************************************************
* Name: fts_fwupg_reset_in_boot
* Brief: RST CMD(07), reset to romboot(bootloader) in boot environment
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
int fts_fwupg_reset_in_boot(struct i2c_client *client)
{
	int ret = 0;
	u8 cmd = FTS_CMD_RESET;

	printk(KERN_INFO "[fts] reset in boot environment\n");
	ret = fts_i2c_write(client, &cmd, 1);
	if (ret < 0) {
		printk(KERN_ERR
		       "[fts] pram/rom/bootloader reset cmd write fail\n");
		return ret;
	}

	msleep(FTS_DELAY_UPGRADE_RESET);
	return 0;
}

/************************************************************************
* Name: fts_ft6336gu_upgrade
* Brief:
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
int fts_ft6336gu_upgrade(struct i2c_client *client, u8 *buf, u32 len)
{
	int ret = 0;
	bool state = false;
	u32 start_addr = 0;
	int ecc_in_host = 0;
	int ecc_in_tp = 0;
	u32 fw_length = 0;

	if (NULL == buf) {
		printk(KERN_ERR "[fts] fw buf is null\n");
		return -EINVAL;
	}

	if ((len < 0x120) || (len > (60 * 1024))) {
		printk(KERN_ERR "[fts] fw buffer len(%x) fail\n", len);
		return -EINVAL;
	}

	fw_length = ((u32) buf[0x100] << 8) + buf[0x101];
	printk(KERN_INFO "[fts] fw length is %d %d\n", fw_length, len);
	/* enter into upgrade environment */
	state = fts_fwupg_check_fw_valid(client);
	if (true == state) {
		ret = fts_ft6336gu_reset_to_boot(client);
		if (ret < 0) {
			printk(KERN_ERR "[fts] enter into bootloader fail\n");
			goto fw_reset;
		}
	}

	state = fts_fwupg_check_state(client, FTS_RUN_IN_BOOTLOADER);
	if (!state) {
		printk(KERN_ERR "[fts] fw not in bootloader, fail\n");
		goto fw_reset;
	}

	ret = fts_ft6336gu_erase(client);
	if (ret < 0) {
		printk(KERN_ERR "[fts] erase cmd write fail\n");
		goto fw_reset;
	}

	/* write app */
	/* start_addr = upgrade_func_ft6336gu.appoff; */
	start_addr = 0x0000;
	ecc_in_host =
	    fts_ft6336gu_write_app(client, start_addr, buf, fw_length);
	if (ecc_in_host < 0) {
		printk(KERN_ERR "[fts] lcd initial code write fail\n");
		goto fw_reset;
	}

	/* ecc */
	ecc_in_tp = fts_ft6336gu_ecc_cal(client);
	if (ecc_in_tp < 0) {
		printk(KERN_ERR "[fts] ecc read fail\n");
		goto fw_reset;
	}

	printk(KERN_INFO "[fts] ecc in tp:%x, host:%x\n", ecc_in_tp,
	       ecc_in_host);
	if (ecc_in_tp != ecc_in_host) {
		printk(KERN_ERR "[fts] ecc check fail\n");
		goto fw_reset;
	}

	printk(KERN_INFO "[fts] upgrade success, reset to normal boot\n");
	ret = fts_fwupg_reset_in_boot(client);
	if (ret < 0)
		printk(KERN_ERR "[fts] reset to normal boot fail\n");

	msleep(200);
	return 0;

fw_reset:
	printk(KERN_INFO "[fts] upgrade fail, reset to normal boot\n");
	ret = fts_fwupg_reset_in_boot(client);
	if (ret < 0)
		printk(KERN_ERR "[fts] reset to normal boot fail\n");

	return -EIO;
}

/************************************************************************
* Name: fts_6336GU_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6336GU_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			       u32 dw_lenth)
{
	u8 reg_val[2] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;

	if (pbt_buf[0] != 0x02) {
		FTS_DBG("FW first byte is not 0x02. so it is invalid\n");
		return -EIO;
	}

	if (dw_lenth > 0x11f) {
		fw_length = ((u32) pbt_buf[0x100] << 8) + pbt_buf[0x101];
		if (dw_lenth < fw_length) {
			FTS_DBG("Fw length is invalid\n");
			return -EIO;
		}
	} else {
		FTS_DBG("Fw length is invalid\n");
		return -EIO;
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
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	auc_i2c_write_buf[0] = FTS_READ_ID_REG;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	fts_i2c_write(client, auc_i2c_write_buf, 5);

	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(fts_updateinfo_curr.delay_erase_flash);

	for (i = 0; i < 200; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (0xb0 == reg_val[0] && 0x02 == reg_val[1]) {
			FTS_DBG("erase app finished\n");
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

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

			if (0xb0 == (reg_val[0] & 0xf0)
			    && (0x03 + (j % 0x0ffd)) ==
			    (((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("write a block data finished\n");
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
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if (0xb0 == (reg_val[0] & 0xf0)
			    && (0x03 + (j % 0x0ffd)) ==
			    (((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("write a block data finished\n");
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
		dev_err(&client->dev,
			"[fts]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0],
			bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	return 0;
}

/************************************************************************
* Name: fts_6x06_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[2] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);

		msleep(fts_updateinfo_curr.delay_55);

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		do {
			i++;
			i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && i < 5);

		/*********Step 3:check READ-ID***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG("Step 3: CTPM ID OK ,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
	}
	if (i > FTS_UPGRADE_LOOP)
		return -EIO;

	auc_i2c_write_buf[0] = 0xcd;

	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);

	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(fts_updateinfo_curr.delay_erase_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
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
		msleep(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}

	/*send the last six byte */
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		fts_i2c_write(client, packet_buf, 7);
		msleep(20);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	return 0;
}

/************************************************************************
* Name: fts_5x26_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x26_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[4] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret = 0;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);

		/*********Step 2:Enter upgrade mode and switch protocol*****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_DBG("failed writing  0x55 and 0xaa !\n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	/*erase app area */
	auc_i2c_write_buf[0] = 0x63;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	/*erase panel paramenter area */
	auc_i2c_write_buf[0] = 0x04;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	/*erase panel paramenter area */
	msleep(fts_updateinfo_curr.delay_erase_flash);
	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
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
		msleep(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}
	/*********Step 6: read out checksum***********************/
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	reg_val[0] = reg_val[1] = 0x00;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	printk(KERN_WARNING "Checksum FT5X26:%X %X\n", reg_val[0], bt_ecc);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);

	/********Step 8 Disable Write Flash*****/
	FTS_DBG("Step 8: Disable Write Flash\n");
	auc_i2c_write_buf[0] = 0x04;
	fts_i2c_write(client, auc_i2c_write_buf, 1);

	msleep(300);
	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = 0x00;
	fts_i2c_write(client, auc_i2c_write_buf, 2);

	return 0;
}

/************************************************************************
* Name: fts_5x36_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[2] = { 0 };
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u8 is_5336_fwsize_30 = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	int fw_filenth = sizeof(CTPM_FW);

	if (CTPM_FW[fw_filenth - 12] == 30)
		is_5336_fwsize_30 = 1;
	else
		is_5336_fwsize_30 = 0;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/*write 0x55 to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);

		/*********Step 3:check READ-ID***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			dev_dbg(&client->dev,
				"[fts] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID FAILD,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	auc_i2c_write_buf[0] = 0xcd;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	/*********20130705 mshl ********************/
	if (reg_val[0] <= 4)
		is_5336_new_bootloader = BL_VERSION_LZ4;
	else if (reg_val[0] == 7)
		is_5336_new_bootloader = BL_VERSION_Z7;
	else if (reg_val[0] >= 0x0f)
		is_5336_new_bootloader = BL_VERSION_GZF;

	/*********Step 4:erase app and panel paramenter area ********************/
	if (is_5336_fwsize_30) {
		auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(fts_updateinfo_curr.delay_erase_flash);

		auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(50);
	} else {
		auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(fts_updateinfo_curr.delay_erase_flash);
	}

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;

	if (is_5336_new_bootloader == BL_VERSION_LZ4
	    || is_5336_new_bootloader == BL_VERSION_Z7) {
		dw_lenth = dw_lenth - 8;
	} else if (is_5336_new_bootloader == BL_VERSION_GZF) {
		dw_lenth = dw_lenth - 14;
	}
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
		msleep(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}
	/*send the last six byte */
	if (is_5336_new_bootloader == BL_VERSION_LZ4
	    || is_5336_new_bootloader == BL_VERSION_Z7) {
		for (i = 0; i < 6; i++) {
			if (is_5336_new_bootloader == BL_VERSION_Z7)
				temp = 0x7bfa + i;
			else if (is_5336_new_bootloader == BL_VERSION_LZ4)
				temp = 0x6ffa + i;

			packet_buf[2] = (u8) (temp >> 8);
			packet_buf[3] = (u8) temp;
			temp = 1;
			packet_buf[4] = (u8) (temp >> 8);
			packet_buf[5] = (u8) temp;
			packet_buf[6] = pbt_buf[dw_lenth + i];
			bt_ecc ^= packet_buf[6];
			fts_i2c_write(client, packet_buf, 7);
			msleep(10);
		}
	} else if (is_5336_new_bootloader == BL_VERSION_GZF) {
		for (i = 0; i < 12; i++) {
			if (is_5336_new_bootloader == BL_VERSION_Z7)
				temp = 0x7ff4 + i;
			else if (is_5336_new_bootloader == BL_VERSION_LZ4)
				temp = 0x7bf4 + i;

			packet_buf[2] = (u8) (temp >> 8);
			packet_buf[3] = (u8) temp;
			temp = 1;
			packet_buf[4] = (u8) (temp >> 8);
			packet_buf[5] = (u8) temp;
			packet_buf[6] = pbt_buf[dw_lenth + i];
			bt_ecc ^= packet_buf[6];
			fts_i2c_write(client, packet_buf, 7);
			msleep(10);
		}
	}

	/*********Step 6: read out checksum***********************/
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}
	/*********Step 7: reset the new FW***********************/
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	return 0;
}

/************************************************************************
* Name: fts_5822_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5822_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[4] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	u8 bt_ecc_check;
	int i_ret;

	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0)
		FTS_DBG("HidI2c change to StdI2c fail !\n");

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);
		if (i_ret == 0)
			FTS_DBG("HidI2c change to StdI2c fail !\n");

		msleep(5);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_DBG("failed writing  0x55 and 0xaa !\n");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1350);

	for (i = 0; i < 15; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0 == reg_val[0] && 0xAA == reg_val[1])
			break;

		msleep(50);
	}
	printk(KERN_ERR "[fts][%s] erase app area reg_val[0] = %x reg_val[1] = %x\n",
	       __func__, reg_val[0], reg_val[1]);

	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_write(client, auc_i2c_write_buf, 4);
	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	bt_ecc_check = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
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
			bt_ecc_check ^= pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		printk(KERN_ERR "[fts][%s] bt_ecc = %x\n", __func__, bt_ecc);
		if (bt_ecc != bt_ecc_check)
			printk(KERN_ERR
				"[fts][%s] Host checksum error bt_ecc_check = %x\n",
			     __func__, bt_ecc_check);
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}
			printk(KERN_ERR "[fts][%s] reg_val[0] = %x reg_val[1] = %x\n",
			       __func__, reg_val[0], reg_val[1]);
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
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc_check ^=
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		printk(KERN_ERR "[fts][%s] bt_ecc = %x\n", __func__, bt_ecc);
		if (bt_ecc != bt_ecc_check)
			printk(KERN_ERR
				"[fts][%s] Host checksum error bt_ecc_check = %x\n",
			     __func__, bt_ecc_check);

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			printk(KERN_ERR "[fts][%s] reg_val[0] = %x reg_val[1] = %x\n",
			       __func__, reg_val[0], reg_val[1]);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
				break;

			printk(KERN_ERR "[fts][%s] reg_val[0] = %x reg_val[1] = %x\n",
			       __func__, reg_val[0], reg_val[1]);
			msleep(1);
		}
	}
	msleep(50);
	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);
	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8) (temp >> 16);
	auc_i2c_write_buf[2] = (u8) (temp >> 8);
	auc_i2c_write_buf[3] = (u8) (temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8) (temp >> 8);
	auc_i2c_write_buf[5] = (u8) (temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);
	for (i = 0; i < 100; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		dev_err(&client->dev,
			"[fts]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0],
			reg_val[1]);
		if (0xF0 == reg_val[0] && 0x55 == reg_val[1]) {
			dev_err(&client->dev,
				"[fts]--reg_val[0]=%02x reg_val[0]=%02x\n",
				reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}
	printk(KERN_WARNING "checksum %X %X\n", reg_val[0], bt_ecc);
	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0)
		FTS_DBG("HidI2c change to StdI2c fail !\n");

	return 0;
}

/************************************************************************
* Name: fts_5x06_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[2] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/*write 0x55 to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		do {
			i++;
			i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && i < 5);

		/*********Step 3:check READ-ID***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG("Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(fts_updateinfo_curr.delay_erase_flash);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	dw_lenth = dw_lenth - 8;
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
		msleep(FTS_PACKET_LENGTH / 6 + 1);
	}
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}
	/*send the last six byte */
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		fts_i2c_write(client, packet_buf, 7);
		msleep(20);
	}
	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}
	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);		/*make sure CTP startup normally */
	return 0;
}

/************************************************************************
* Name: fts_5x46_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x46_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[4] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0)
		FTS_DBG("hid change to i2c fail !\n");

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/* write 0x55 to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);

		if (i_ret == 0) {
			FTS_DBG("hid change to i2c fail !\n");
		}
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_DBG("failed writing  0x55 and 0xaa !\n");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1350);
	for (i = 0; i < 15; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0 == reg_val[0] && 0xAA == reg_val[1])
			break;

		msleep(50);
	}
	printk("[fts][%s] erase app area reg_val[0] = %x reg_val[1] = %x\n",
	       __func__, reg_val[0], reg_val[1]);
	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_write(client, auc_i2c_write_buf, 4);
	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
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
		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
				break;

			printk("[fts][%s] reg_val[0] = %x reg_val[1] = %x\n",
			       __func__, reg_val[0], reg_val[1]);
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
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			printk("[fts][%s] reg_val[0] = %x reg_val[1] = %x\n",
			       __func__, reg_val[0], reg_val[1]);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
				break;

			printk("[fts][%s] reg_val[0] = %x reg_val[1] = %x\n",
			       __func__, reg_val[0], reg_val[1]);
			msleep(1);

		}
	}

	msleep(50);

	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8) (temp >> 16);
	auc_i2c_write_buf[2] = (u8) (temp >> 8);
	auc_i2c_write_buf[3] = (u8) (temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8) (temp >> 8);
	auc_i2c_write_buf[5] = (u8) (temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);

	for (i = 0; i < 100; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		dev_err(&client->dev,
			"[fts]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0],
			reg_val[1]);
		if (0xF0 == reg_val[0] && 0x55 == reg_val[1]) {
			dev_err(&client->dev,
				"[fts]--reg_val[0]=%02x reg_val[0]=%02x\n",
				reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);

	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}
	printk(KERN_WARNING "checksum %X %X\n", reg_val[0], bt_ecc);
	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0) {
		FTS_DBG("HidI2c change to StdI2c fail !\n");
	}
	return 0;
}

/************************************************************************
*   Name: fts_8606_writepram
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_8606_writepram(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{

	u8 reg_val[4] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	FTS_DBG("8606 dw_lenth= %d", dw_lenth);
	if (dw_lenth > 0x10000 || dw_lenth == 0)
		return -EIO;

	for (i = 0; i < 20; i++) {
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 1);
		if (i_ret < 0) {
			FTS_DBG("failed writing  0x55 !\n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;

		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if ((reg_val[0] == 0x86
		     && reg_val[1] == 0x06) || (reg_val[0] == 0x86
						&& reg_val[1] == 0x06)) {
			msleep(50);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	/*********Step 4:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xae;
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
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
	}

	/*********Step 5: read out checksum***********************/
	/*send the opration head */
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}
	FTS_DBG("checksum %X %X\n", reg_val[0], bt_ecc);
	FTS_DBG("Read flash and compare\n");

	msleep(50);

	/*********Step 6: start app***********************/
	FTS_DBG("Step 6: start app\n");
	auc_i2c_write_buf[0] = 0x08;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(20);

	return 0;
}

/************************************************************************
*   Name: fts_8606_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_8606_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[4] = { 0 };
	u8 reg_val_id[4] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	unsigned char cmd[20];
	unsigned char Checksum = 0;

	auc_i2c_write_buf[0] = 0x05;
	reg_val_id[0] = 0x00;

	i_ret = fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val_id, 1);
	if (dw_lenth == 0)
		return -EIO;


	if (0x81 == (int)reg_val_id[0]) {
		if (dw_lenth > 1024 * 60)
			return -EIO;
	} else if (0x80 == (int)reg_val_id[0]) {
		if (dw_lenth > 1024 * 64)
			return -EIO;
	}

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_DBG("failed writing  0x55 and 0xaa !\n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;

		reg_val[0] = reg_val[1] = 0x00;

		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if ((reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		     && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
		    || (reg_val[0] == 0x86 && reg_val[1] == 0xA6)) {
			FTS_DBG
			    ("Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");

	{
		cmd[0] = 0x05;
		cmd[1] = reg_val_id[0];	/* 0x80; */
		cmd[2] = 0x00;	/* ??? */
		fts_i2c_write(client, cmd, 3);
	}

	{
		cmd[0] = 0x09;
		cmd[1] = 0x0B;
		fts_i2c_write(client, cmd, 2);
	}

	for (i = 0; i < dw_lenth; i++)
		Checksum ^= pbt_buf[i];

	msleep(50);

	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1350);

	for (i = 0; i < 15; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0 == reg_val[0] && 0xAA == reg_val[1])
			break;

		msleep(50);
	}

	bt_ecc = 0;
	FTS_DBG("Step 5:write firmware(FW) to ctpm flash\n");

	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;

	for (j = 0; j < packet_number; j++) {
		temp = 0x1000 + j * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
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

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
				break;

			msleep(1);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = 0x1000 + packet_number * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
				break;

			msleep(1);
		}
	}

	msleep(50);

	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	FTS_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);
	temp = 0x1000 + 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8) (temp >> 16);
	auc_i2c_write_buf[2] = (u8) (temp >> 8);
	auc_i2c_write_buf[3] = (u8) (temp);

	if (dw_lenth > LEN_FLASH_ECC_MAX) {
		temp = LEN_FLASH_ECC_MAX;
	} else {
		temp = dw_lenth;
		FTS_DBG("Step 6_1: read out checksum\n");
	}
	auc_i2c_write_buf[4] = (u8) (temp >> 8);
	auc_i2c_write_buf[5] = (u8) (temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);

	for (i = 0; i < 100; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0 == reg_val[0] && 0x55 == reg_val[1])
			break;

		msleep(1);
	}
	/*----------------------------------------------------------------------*/
	if (dw_lenth > LEN_FLASH_ECC_MAX) {
		temp = LEN_FLASH_ECC_MAX;	/*? 0x1000+LEN_FLASH_ECC_MAX */
		auc_i2c_write_buf[0] = 0x65;
		auc_i2c_write_buf[1] = (u8) (temp >> 16);
		auc_i2c_write_buf[2] = (u8) (temp >> 8);
		auc_i2c_write_buf[3] = (u8) (temp);
		temp = dw_lenth - LEN_FLASH_ECC_MAX;
		auc_i2c_write_buf[4] = (u8) (temp >> 8);
		auc_i2c_write_buf[5] = (u8) (temp);
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);

		msleep(dw_lenth / 256);

		for (i = 0; i < 100; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if (0xF0 == reg_val[0] && 0x55 == reg_val[1])
				break;

			msleep(1);

		}
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);

		return -EIO;
	}
	FTS_DBG("checksum %X %X\n", reg_val[0], bt_ecc);
	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);		/* make sure CTP startup normally */
	return 0;
}

/************************************************************************
* Name: fts_3x27_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_3x07_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			     u32 dw_lenth)
{
	u8 reg_val[2] = { 0 };
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;

	if (pbt_buf[0] != 0x02) {
		FTS_DBG("FW first byte is not 0x02. so it is invalid\n");
		return -EIO;
	}

	if (dw_lenth > 0x11f) {
		fw_length = ((u32) pbt_buf[0x100] << 8) + pbt_buf[0x101];
		if (dw_lenth < fw_length) {
			FTS_DBG("Fw length is invalid\n");
			return -EIO;
		}
	} else {
		FTS_DBG("Fw length is invalid\n");
		return -EIO;
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
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] =
		    auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
		    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) {
			FTS_DBG
			    ("Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
			     reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev,
				"[fts] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	auc_i2c_write_buf[0] = FTS_READ_ID_REG;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	fts_i2c_write(client, auc_i2c_write_buf, 5);

	/*Step 4:erase app and panel paramenter area */
	FTS_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(fts_updateinfo_curr.delay_erase_flash);

	for (i = 0; i < 200; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (0xb0 == reg_val[0] && 0x02 == reg_val[1]) {
			FTS_DBG("erase app finished\n");
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

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if (0xb0 == (reg_val[0] & 0xf0)
			    && (0x03 + (j % 0x0ffd)) ==
			    (((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("write a block data finished\n");
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
			packet_buf[6 + i] =
			    pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if (0xb0 == (reg_val[0] & 0xf0)
			    && (0x03 + (j % 0x0ffd)) ==
			    (((reg_val[0] & 0x0f) << 8) | reg_val[1])) {
				FTS_DBG("write a block data finished\n");
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
		dev_err(&client->dev, "[fts]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0], bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	FTS_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	return 0;
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
	sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

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
	sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

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
int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				      char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret = 0;
	int fwsize = fts_GetFirmwareSize(firmware_name);

	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",
			__func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 54 * 1024) {
		dev_err(&client->dev, "FW length error\n");
		return -EIO;
	}

	dev_err(&client->dev, "[fts] CHIP_ID : 0x%x, fw_size=%d\n",
		fts_updateinfo_curr.CHIP_ID, fwsize);
	/*=========FW upgrade========================*/
	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_ATOMIC);
	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",
			__func__);
		kfree(pbt_buf);
		return -EIO;
	}
	if ((fts_updateinfo_curr.CHIP_ID == 0x33))
		i_ret = fts_ft6336gu_upgrade(client, pbt_buf, fwsize);
	else if ((fts_updateinfo_curr.CHIP_ID == 0x55)
		 || (fts_updateinfo_curr.CHIP_ID == 0x08)
		 || (fts_updateinfo_curr.CHIP_ID == 0x0a)) {
		i_ret = fts_5x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x11)
		   || (fts_updateinfo_curr.CHIP_ID == 0x12)
		   || (fts_updateinfo_curr.CHIP_ID == 0x13)
		   || (fts_updateinfo_curr.CHIP_ID == 0x14)) {
		i_ret = fts_5x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x06)) {
		i_ret = fts_6x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x36)) {
		i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x64)) {
		i_ret = fts_ft6336gu_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x54)) {
		i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x58)) {
		i_ret = fts_5822_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x59)) {
		i_ret = fts_5x26_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x0e)) {
		i_ret = fts_3x07_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x86)) {
		dev_err(&client->dev,
			"%s:upgrade failed. err, chip is 3027 on EFWx\n",
			__func__);
		return -EIO;
		/*call the upgrade function */
		/* i_ret = fts_8606_writepram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT)); */

		if (i_ret != 0) {
			dev_err(&client->dev,
				"%s:fts_8606_writepram failed. err.\n",
				__func__);
			return -EIO;
		}

		i_ret = fts_8606_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}

	if (i_ret != 0)
		dev_err(&client->dev, "%s() - ERROR:[fts] upgrade failed..\n",
			__func__);
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
		if (fts_updateinfo_curr.CHIP_ID == 0x33 ||
			fts_updateinfo_curr.CHIP_ID == 0x64)
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
		return CTPM_FW[0xB4EC];
	else
		return 0x00;
}

/************************************************************************
* Name: fts_ctpm_update_project_setting
* Brief:  update project setting, only update these settings for COB project, or for some special case
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_update_project_setting(struct i2c_client *client)
{
	u8 uc_i2c_addr;
	u8 uc_io_voltage;
	u8 uc_panel_factory_id;
	u8 buf[FTS_SETTING_BUF_LEN];
	u8 reg_val[2] = { 0 };
	u8 auc_i2c_write_buf[10] = { 0 };
	u8 packet_buf[FTS_SETTING_BUF_LEN + 6];
	u32 i = 0;
	int i_ret;

	uc_i2c_addr = client->addr;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;

	/*Step 1:Reset  CTPM */
	if (fts_updateinfo_curr.CHIP_ID == 0x06
	    || fts_updateinfo_curr.CHIP_ID == 0x36) {
		fts_write_reg(client, 0xbc, 0xaa);
	} else {
		fts_write_reg(client, 0xfc, 0xaa);
	}
	msleep(50);

	/*write 0x55 to register 0xfc */
	if (fts_updateinfo_curr.CHIP_ID == 0x06
	    || fts_updateinfo_curr.CHIP_ID == 0x36) {
		fts_write_reg(client, 0xbc, 0x55);
	} else {
		fts_write_reg(client, 0xfc, 0x55);
	}
	msleep(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		msleep(5);
	} while (i_ret <= 0 && i < 5);

	/*********Step 3:check READ-ID***********************/
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
	    0x00;

	fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

	if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
	    && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
		dev_dbg(&client->dev,
			"[fts] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
			reg_val[0], reg_val[1]);
	else
		return -EIO;

	auc_i2c_write_buf[0] = 0xcd;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	dev_dbg(&client->dev, "bootloader version = 0x%x\n", reg_val[0]);

	/*--------- read current project setting  ---------- */
	/*set read start address */
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;

	fts_i2c_read(client, buf, 4, buf, FTS_SETTING_BUF_LEN);
	dev_dbg(&client->dev, "[fts] old setting: uc_i2c_addr = 0x%x,\
		uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n", buf[0], buf[2], buf[4]);

	 /*--------- Step 4:erase project setting --------------*/
	auc_i2c_write_buf[0] = 0x63;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*----------  Set new settings ---------------*/
	buf[0] = uc_i2c_addr;
	buf[1] = ~uc_i2c_addr;
	buf[2] = uc_io_voltage;
	buf[3] = ~uc_io_voltage;
	buf[4] = uc_panel_factory_id;
	buf[5] = ~uc_panel_factory_id;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	packet_buf[4] = 0;
	packet_buf[5] = FTS_SETTING_BUF_LEN;

	for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
		packet_buf[6 + i] = buf[i];

	fts_i2c_write(client, packet_buf, FTS_SETTING_BUF_LEN + 6);
	msleep(100);

	/********* reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);

	msleep(200);
	return 0;
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

	/*judge the fw that will be upgraded
	 * if illegal, then stop upgrade and return.
	 */
	if ((fts_updateinfo_curr.CHIP_ID == 0x11)
	    || (fts_updateinfo_curr.CHIP_ID == 0x12)
	    || (fts_updateinfo_curr.CHIP_ID == 0x13)
	    || (fts_updateinfo_curr.CHIP_ID == 0x14)
	    || (fts_updateinfo_curr.CHIP_ID == 0x55)
	    || (fts_updateinfo_curr.CHIP_ID == 0x06)
	    || (fts_updateinfo_curr.CHIP_ID == 0x0a)
	    || (fts_updateinfo_curr.CHIP_ID == 0x08)) {
		if (fw_len < 8 || fw_len > 32 * 1024) {
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}

		if ((CTPM_FW[fw_len - 8] ^ CTPM_FW[fw_len - 6]) == 0xFF
		    && (CTPM_FW[fw_len - 7] ^ CTPM_FW[fw_len - 5]) == 0xFF
		    && (CTPM_FW[fw_len - 3] ^ CTPM_FW[fw_len - 4]) == 0xFF) {
			/*FW upgrade */
			pbt_buf = CTPM_FW;
			/*call the upgrade function */
			if ((fts_updateinfo_curr.CHIP_ID == 0x55)
			    || (fts_updateinfo_curr.CHIP_ID == 0x08)
			    || (fts_updateinfo_curr.CHIP_ID == 0x0a)) {
				i_ret =
				    fts_5x06_ctpm_fw_upgrade(client, pbt_buf,
							     sizeof(CTPM_FW));
			} else if ((fts_updateinfo_curr.CHIP_ID == 0x11)
				   || (fts_updateinfo_curr.CHIP_ID == 0x12)
				   || (fts_updateinfo_curr.CHIP_ID == 0x13)
				   || (fts_updateinfo_curr.CHIP_ID == 0x14)) {
				i_ret =
				    fts_5x36_ctpm_fw_upgrade(client, pbt_buf,
							     sizeof(CTPM_FW));
			} else if ((fts_updateinfo_curr.CHIP_ID == 0x06)) {
				i_ret =
				    fts_6x06_ctpm_fw_upgrade(client, pbt_buf,
							     sizeof(CTPM_FW));
			}
			if (i_ret != 0)
				dev_err(&client->dev,
					"%s:upgrade failed. err.\n", __func__);
			else if (fts_updateinfo_curr.AUTO_CLB == AUTO_CLB_NEED) {
				fts_ctpm_auto_clb(client);
			}
		} else {
			dev_err(&client->dev, "%s:FW format error\n", __func__);
			return -EBADFD;
		}
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x36)) {
		if (fw_len < 8 || fw_len > 32 * 1024) {
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		pbt_buf = CTPM_FW;
		i_ret =
		    fts_6x36_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
				__func__);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x64)) {
		if (fw_len < 8 || fw_len > 48 * 1024) {
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		pbt_buf = CTPM_FW;
		i_ret =
		    fts_ft6336gu_upgrade(client, pbt_buf,
					       sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
				__func__);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x54)) {
		if (fw_len < 8 || fw_len > 54 * 1024) {
			pr_err("FW length error\n");
			return -EIO;
		}
		/*FW upgrade */
		pbt_buf = CTPM_FW;
		/*call the upgrade function */
		i_ret =
		    fts_5x46_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0) {
			dev_err(&client->dev, "[fts] upgrade failed. err=%d.\n",
				i_ret);
		} else {
#ifdef AUTO_CLB
			fts_ctpm_auto_clb(client);	/*start auto CLB */
#endif
		}
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x58)) {
		if (fw_len < 8 || fw_len > 54 * 1024) {
			pr_err("FW length error\n");
			return -EIO;
		}

		/*FW upgrade */
		pbt_buf = CTPM_FW;
		/*call the upgrade function */
		i_ret =
		    fts_5822_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0) {
			dev_err(&client->dev, "[fts] upgrade failed. err=%d.\n",
				i_ret);
		} else {
#ifdef AUTO_CLB
			fts_ctpm_auto_clb(client);	/*start auto CLB */
#endif
		}
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x59)) {
		if (fw_len < 8 || fw_len > 54 * 1024) {
			pr_err("FW length error\n");
			return -EIO;
		}

		/*FW upgrade */
		pbt_buf = CTPM_FW;
		/*call the upgrade function */
		i_ret =
		    fts_5x26_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0) {
			dev_err(&client->dev, "[fts] upgrade failed. err=%d.\n",
				i_ret);
		} else {
#ifdef AUTO_CLB
			fts_ctpm_auto_clb(client);	/*start auto CLB */
#endif
		}
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x0e)) {
		if (fw_len < 8 || fw_len > 32 * 1024) {
			dev_err(&client->dev, "%s:FW length error\n", __func__);
			return -EIO;
		}
		pbt_buf = CTPM_FW;
		i_ret =
		    fts_3x07_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
				__func__);
	} else if ((fts_updateinfo_curr.CHIP_ID == 0x86)) {
		/*FW upgrade */
		pbt_buf = CTPM_FW;

		dev_err(&client->dev,
			"%s:upgrade failed. err, chip is 3027 on EFWx\n",
			__func__);
		return -EIO;
		/*call the upgrade function */
		/* i_ret = fts_8606_writepram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT)); */

		if (i_ret != 0) {
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
				__func__);
			return -EIO;
		}

		i_ret =
		    fts_8606_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));

		if (i_ret != 0) {
			dev_err(&client->dev, "[fts] upgrade failed. err=%d.\n",
				i_ret);
		} else {
#ifdef AUTO_CLB
			fts_ctpm_auto_clb(client);	/*start auto CLB */
#endif
		}
	}
	return i_ret;
}

int fts_ctpm_fw_upgrade_with_i_file_for_cci_3207(struct i2c_client *client)
{
	int i_ret = -1;
	/* int fw_len = fw_size; */

	/*judge the fw that will be upgraded
	 * if illegal, then stop upgrade and return.
	 */
	if (fts_updateinfo_curr.CHIP_ID == 0x64) {
		i_ret = fts_ft6336gu_upgrade(client, CTPM_FW, fw_size);
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
				__func__);
	} else
		dev_err(&client->dev, "%s:CHIP_ID 0x%x is wrong.\n", __func__,
			fts_updateinfo_curr.CHIP_ID);

	return i_ret;
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	u8 uc_host_fm_ver = FTS_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int i_ret;

	fts_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	FTS_DBG("uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n", uc_tp_fm_ver,
		uc_host_fm_ver);

	if (uc_tp_fm_ver == FTS_REG_FW_VER || uc_tp_fm_ver < uc_host_fm_ver) {
		msleep(100);
		i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
		if (i_ret == 0) {
			msleep(300);
			uc_host_fm_ver = fts_ctpm_get_i_file_ver();
			FTS_DBG("upgrade to new version 0x%x\n",
				uc_host_fm_ver);
		} else {
			pr_err("[fts] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}
	} else
		FTS_DBG("version is newest, no need upgrade FW\n");

	return 0;
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade_for_cci
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_upgrade_for_cci(struct i2c_client *client, const u8 tp_id,
				  bool force_upgrade)
{
	u8 uc_host_fm_ver = FTS_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int i_ret;

	pr_info("[fts] %s TP ID = 0x%x\n", __func__, tp_id);
	switch (tp_id) {
	case TP_ID_T:
		CTPM_FW = CTPM_FW_TP_ID_T;
		fw_size = sizeof(CTPM_FW_TP_ID_T);
		break;
	case TP_ID_S:
		CTPM_FW = CTPM_FW_TP_ID_S;
		fw_size = sizeof(CTPM_FW_TP_ID_S);
		break;
	case TP_ID_S_TRULY:
		CTPM_FW = CTPM_FW_TP_ID_S_TRULY;
		fw_size = sizeof(CTPM_FW_TP_ID_S_TRULY);
		break;
	case TP_ID_S2:
		CTPM_FW = CTPM_FW_TP_ID_S2;
		fw_size = sizeof(CTPM_FW_TP_ID_S2);
		break;
	default:
		pr_err("[fts] TP ID 0x%x isn't correct\n", tp_id);
		return -EIO;
		break;
	}

	fts_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver_for_cci();
	FTS_DBG("uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n", uc_tp_fm_ver,
		uc_host_fm_ver);

	if (uc_tp_fm_ver != uc_host_fm_ver || force_upgrade) {
		msleep(100);
		i_ret = fts_ctpm_fw_upgrade_with_i_file_for_cci_3207(client);
		if (i_ret == 0) {
			msleep(300);
			FTS_DBG("upgrade to new version 0x%x\n",
				uc_host_fm_ver);
		} else {
			pr_err("[fts] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}
	} else {
		FTS_DBG("version is newest, no need upgrade FW\n");
		return 1;
	}
	return 0;
}

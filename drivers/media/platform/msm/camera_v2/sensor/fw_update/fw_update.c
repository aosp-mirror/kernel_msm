/* OIS calibration interface for LC898123 F40
 *
 */

#include "msm_sensor.h"
#include "../cci/msm_cci.h"
#include "fw_update.h"

#define OIS_COMPONENT_I2C_ADDR_WRITE 0x7C
static struct msm_camera_i2c_client *g_i2c_client = NULL;
bool g_first = true;


void RamWrite32A( UINT_16 RamAddr, UINT_32 RamData )
{
//Add 32 bit I2C writing function
	int rc = 0;
	uint8_t data[4] = {0,0,0,0};
	struct msm_camera_i2c_client *i2c_client = g_i2c_client;

	data[0] = (RamData >> 24) & 0xFF;
	data[1] = (RamData >> 16) & 0xFF;
	data[2] = (RamData >> 8)  & 0xFF;
	data[3] = (RamData) & 0xFF;

	rc = i2c_client->i2c_func_tbl->i2c_write_seq(
		i2c_client, RamAddr, &data[0], 4);
	if (rc < 0)
		pr_err("[OIS_Cali] %s : write failed\n", __func__);
}

void RamRead32A( UINT_16 RamAddr, UINT_32 * ReadData )
{
//Add 32 bit I2C writing function
	int rc = 0;
	uint8_t buf[4] = {0,0,0,0};
	struct msm_camera_i2c_client *i2c_client = g_i2c_client;

	rc = i2c_client->i2c_func_tbl->i2c_read_seq(
		i2c_client, RamAddr, &buf[0], 4);
	if (rc < 0)
		pr_err("[OIS_Cali] %s : read failed\n", __func__);
	else
		*ReadData = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

void WitTim( UINT_16	UsWitTim )
{
	mdelay(UsWitTim);
}

int CntWrt( UINT_8 * PcSetDat, UINT_16 UsDatNum)
{
	int rc = 0;
	int temp = 0;
	struct msm_camera_i2c_client *i2c_client = g_i2c_client;
	temp = i2c_client->addr_type;
	i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	rc = i2c_client->i2c_func_tbl->i2c_write_seq(i2c_client, PcSetDat[0], &PcSetDat[1], UsDatNum-1);
	i2c_client->addr_type = temp;
	if (rc < 0) {
		pr_err("[OIS_Cali] %s:i2c write sequence error:%d\n", __func__, rc);
		return rc;
	}
	return rc;
}

int CntRd3( UINT_32 addr, void * PcSetDat, UINT_16 UsDatNum )
{
	int rc = 0;
	struct msm_camera_i2c_client *i2c_client = g_i2c_client;
	rc = i2c_client->i2c_func_tbl->i2c_read_seq(i2c_client, addr, PcSetDat, UsDatNum);

	if (rc < 0) {
		pr_err("[OIS_Cali] %s:i2c write sequence error:%d\n", __func__, rc);
		return rc;
	}
	return rc;
}

void WPBCtrl( UINT_8 UcCtrl )
{
	//do nothing because lc898123F40 uses UnlockCodeSet() to handle WPB by itself
}

int checkHighLevelCommand(int cnt)
{
//Add 32 bit I2C writing function
	//int rc = 0;
	int i;
	UINT_32 FWRead;


	for (i =0; i< cnt ; i++)
	{
		WitTim(10);
		RamRead32A(0xF100, &FWRead);//Check high level command ready.
		if (FWRead == 0x0 )
		{
			pr_info("[OISFW]:checkHighLevelCommand finish.");
			return 0;
		}else
			pr_info("[OISFW]:Waiting...");
	}
	pr_err("[OISFW]:checkHighLevelCommand fail.");
	return -1;
}

#define LGIT 0x09
#define SHARPIT 0x04
#define NOUPDATE 0
#define CM1_SHARP 1
#define CM1_LG 2
#define CM2_SHARP 3
#define CM2_LG 4


int checkHWFWversion(void)
{
    int rc;
    UINT_16 RamAddr;
    UINT_32 RamData;
    UINT_32 UlReadVal;
    UINT_16 CM_version;
    UINT_16 FW_version;
    UINT_16 IT_version;
    UINT_16 EVT_version;

    pr_info("[OISFW]%s", __func__);

    RamAddr = 0x8000;
    RamRead32A(RamAddr, &UlReadVal);
    pr_info("[OISFW]:%s 0x8000 =  0x%08x", __func__, UlReadVal);
    FW_version = UlReadVal & 0xFF;
    IT_version = UlReadVal >> 24;
    pr_info("[OISFW]:%s FW_version =  0x%02x", __func__, FW_version);
    pr_info("[OISFW]:%s IT_version =  0x%02x", __func__, IT_version);

    if (FW_version >= 0x0F)
    {
        pr_info("[OISFW]%s: No need to update.", __func__);
        return 0;
    }

    rc = checkHighLevelCommand(100);
    if(rc != 0)
    {
        pr_err("[OISFW]:%s checkHighLevelCommand failed = %d \n", __func__, rc);
        return -1;
    }
    else
	pr_info("[OISFW]:%s checkHighLevelCommand = %d \n", __func__, rc);

    RamAddr = 0xF01B;
    RamData = 0x1A00;
    RamWrite32A(RamAddr, RamData);
    RamRead32A(RamAddr, &UlReadVal); //Read EEPROM
    /*
        SHARP    50 53 xx xx
        LG EVT1  00 00 4c 47
        LG EVT2  47 4c xx xx
    */
    pr_info("[OISFW]:%s 0x1A00 =  0x%08x", __func__, UlReadVal);
    EVT_version = UlReadVal >> 16;
    pr_info("[OISFW]:%s EVT_version =  0x%04x", __func__, EVT_version);

    RamAddr = 0xF01B;
    RamData = 0x1A02;
    RamWrite32A(RamAddr, RamData);
    RamRead32A(RamAddr, &UlReadVal);//Read EEPROM
    pr_info("[OISFW]:%s UlReadVal =  0x%08x", __func__, UlReadVal);

    if(EVT_version == 0x0)
        CM_version = UlReadVal & 0xFF; // LG EVT1
    else
        CM_version = UlReadVal >> 24; //LG EVT2 and SHARP

    pr_info("[OISFW]:%s CM_version =  0x%02x", __func__, CM_version);


    if(FW_version < 0x0F && IT_version == LGIT){
        if(CM_version < 2 )
            rc = CM1_LG;
        else if (CM_version == 2)
            rc = CM2_LG;
        else
            rc = NOUPDATE;
    }
    else if(FW_version < 0XF && IT_version == SHARPIT){
        if(CM_version < 2 )
            rc = CM1_SHARP;
        else if (CM_version == 2)
            rc = CM2_SHARP;
        else
            rc = NOUPDATE;
    }else{
        pr_info("[OISFW]%s: No need to update.", __func__);
        rc = NOUPDATE;
    }
    pr_info("[OISFW]%s:HW version : %d.", __func__, rc);

    return rc;
}

int checkFWUpdate(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    int i;
    unsigned short cci_client_sid_backup;
    UINT_32 FWRead;
    UINT_8 HWVer;

    if (g_first != true)
    {
        pr_info("[OISFW]%s: No need.\n", __func__);
        return 0;
    }
    g_first = false;

    pr_info("[OISFW]: %s 1. sid = %d \n", __func__, s_ctrl->sensor_i2c_client->cci_client->sid);
    /* Bcakup the I2C slave address */
    cci_client_sid_backup = s_ctrl->sensor_i2c_client->cci_client->sid;
    /* Replace the I2C slave address with OIS component */
    s_ctrl->sensor_i2c_client->cci_client->sid = OIS_COMPONENT_I2C_ADDR_WRITE >> 1;

    g_i2c_client = s_ctrl->sensor_i2c_client;
    WitTim(100);

    HWVer = checkHWFWversion();//Check current HW and FW version

    if (HWVer == CM1_LG)
    {
        pr_info("[OISFW]%s: F40_FlashDownload(0, 9, 6 )_LGCM1.", __func__);
        rc = F40_FlashDownload(0, 9, 6 );
    }
    else if (HWVer == CM1_SHARP)
    {
        pr_info("[OISFW]%s: F40_FlashDownload(0, 4, 6 )_SHARPCM1.", __func__);
		rc = F40_FlashDownload(0, 4, 6 );
    }
    else
        pr_info("[OISFW]%s: no need to update.", __func__);

    if(rc != 0)
    {
        pr_err("[OISFW]%s: OIS FW update failed rc = %d.", __func__, rc);
        /* Restore the I2C slave address */
        s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup;
        return rc;
    }
    //Wait for FW update finish.
    if(HWVer == CM1_LG || HWVer == CM1_SHARP)
    {
        for(i = 0; i < 20; i++)
        {
            WitTim(100);
            RamRead32A(0xF100, &FWRead);//Check high level command ready.
            pr_info("[OISFW]:%s FWRead =  0x%x", __func__, FWRead);
            if (FWRead == 0x0 )
            {
                pr_info("[OISFW]:Update finish.");
                break;
            }else
                pr_info("[OISFW]:Waiting...");
        }
        if (FWRead != 0x0)
        {
            rc = -1;
            pr_err("[OISFW]%s: OIS FW update failed rc = %d. FWRead =  0x%x", __func__, rc, FWRead);
        }
    }
    RamRead32A(0x8000, &FWRead);
    pr_info("[OISFW]:%s 0x8000 =  0x%08x", __func__, FWRead);
    /* Restore the I2C slave address */
    s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup;
    pr_info("[OISFW]: %s 2. sid = %d \n", __func__, s_ctrl->sensor_i2c_client->cci_client->sid);
    pr_info("[OISFW]%s rc = %d\n", __func__, rc);

    return rc;
}

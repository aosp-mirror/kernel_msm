/* Himax Android Driver Sample Code for HMX852xES chipset
*
* Copyright (C) 2014 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include "himax_852xES.h"

#define SUPPORT_FINGER_DATA_CHECKSUM    0x0F
#define TS_WAKE_LOCK_TIMEOUT            (2 * HZ)

static unsigned char IC_CHECKSUM = 0;
static unsigned char IC_TYPE = 0;

static int HX_TOUCH_INFO_POINT_CNT = 0;

static int HX_RX_NUM = 0;
static int HX_TX_NUM = 0;
static int HX_BT_NUM = 0;
static int HX_X_RES = 0;
static int HX_Y_RES = 0;
static int HX_MAX_PT = 0;
static bool HX_XY_REVERSE = false;
static bool HX_INT_IS_EDGE = false;

static unsigned int FW_VER_MAJ_FLASH_ADDR;
static unsigned int FW_VER_MAJ_FLASH_LENG;
static unsigned int FW_VER_MIN_FLASH_ADDR;
static unsigned int FW_VER_MIN_FLASH_LENG;

static unsigned int FW_CFG_VER_FLASH_ADDR;

static unsigned int CFG_VER_MAJ_FLASH_ADDR;
static unsigned int CFG_VER_MAJ_FLASH_LENG;
static unsigned int CFG_VER_MIN_FLASH_ADDR;
static unsigned int CFG_VER_MIN_FLASH_LENG;

static uint8_t AA_press = 0x00;
static uint8_t IC_STATUS_CHECK    = 0xAA;
static uint8_t EN_NoiseFilter = 0x00;
static uint8_t Last_EN_NoiseFilter = 0x00;
static uint8_t HX_DRIVER_PROBE_Fial = 0;
static int hx_point_num = 0;      //for himax_ts_work_func use
static int p_point_num = 0xFFFF;
static int point_flag[2] = {0};
static int haspoint = false;

static bool config_load = false;
static struct himax_config *config_selected = NULL;

static int iref_number = 11;
static bool iref_found = false;

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

#ifdef CONFIG_OF
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
static int himax_parse_config(struct himax_ts_data *ts, struct himax_config *pdata);
#endif
#endif

static int himax_hand_shaking(void)    //0:Running, 1:Stop, 2:I2C Fail
{
    int ret, result;
    uint8_t hw_reset_check[1];
    uint8_t hw_reset_check_2[1];
    uint8_t buf0[2];
    memset(hw_reset_check, 0x00, sizeof(hw_reset_check));
    memset(hw_reset_check_2, 0x00, sizeof(hw_reset_check_2));
    buf0[0] = 0xF2;
    if (IC_STATUS_CHECK == 0xAA) {
        buf0[1] = 0xAA;
        IC_STATUS_CHECK = 0x55;
    } else {
        buf0[1] = 0x55;
        IC_STATUS_CHECK = 0xAA;
    }
    ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
    if (ret < 0) {
        E("[Himax]:write 0xF2 failed line: %d \n", __LINE__);
        goto work_func_send_i2c_msg_fail;
    }
    msleep(50);
    buf0[0] = 0xF2;
    buf0[1] = 0x00;
    ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
    if (ret < 0) {
        E("[Himax]:write 0x92 failed line: %d \n", __LINE__);
        goto work_func_send_i2c_msg_fail;
    }
    msleep(2);
    ret = i2c_himax_read(private_ts->client, 0x90, hw_reset_check, 1, DEFAULT_RETRY_CNT);
    if (ret < 0) {
        E("[Himax]:i2c_himax_read 0x90 failed line: %d \n", __LINE__);
        goto work_func_send_i2c_msg_fail;
    }
    if ((IC_STATUS_CHECK != hw_reset_check[0])) {
        msleep(2);
        ret = i2c_himax_read(private_ts->client, 0x90, hw_reset_check_2, 1, DEFAULT_RETRY_CNT);
        if (ret < 0) {
            E("[Himax]:i2c_himax_read 0x90 failed line: %d \n", __LINE__);
            goto work_func_send_i2c_msg_fail;
        }
        if (hw_reset_check[0] == hw_reset_check_2[0]) {
            result = 1;
        } else {
            result = 0;
        }
    } else {
        result = 0;
    }
    return result;
work_func_send_i2c_msg_fail:
    return 2;
}

static int himax_ManualMode(int enter)
{
    uint8_t cmd[2];
    cmd[0] = enter;
    if ( i2c_himax_write(private_ts->client, HX_CMD_MANUALMODE , &cmd[0], 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    return 0;
}

static int himax_FlashMode(int enter)
{
    uint8_t cmd[2];
    cmd[0] = enter;
    if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    return 0;
}

static int himax_lock_flash(int enable)
{
    uint8_t cmd[5];
    if (i2c_himax_write(private_ts->client, 0xAA , &cmd[0], 0, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    /* lock sequence start */
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x06;
    if (i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 3, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    cmd[0] = 0x03;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if (i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    if (enable != 0) {
        cmd[0] = 0x63;
        cmd[1] = 0x02;
        cmd[2] = 0x70;
        cmd[3] = 0x03;
    } else {
        cmd[0] = 0x63;
        cmd[1] = 0x02;
        cmd[2] = 0x30;
        cmd[3] = 0x00;
    }
    if (i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER , &cmd[0], 4, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    if (i2c_himax_write_command(private_ts->client, HX_CMD_4A, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    msleep(50);
    if (i2c_himax_write(private_ts->client, 0xA9 , &cmd[0], 0, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    return 0;
    /* lock sequence stop */
}


static void himax_changeIref(int selected_iref)
{
    unsigned char temp_iref[16][2] = {
        {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00},
        {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00},
        {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00},
        {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}
    };
    uint8_t cmd[10];
    int i = 0;
    int j = 0;
    I("%s: start to check iref,iref number = %d\n", __func__, selected_iref);
    if (i2c_himax_write(private_ts->client, 0xAA , &cmd[0], 0, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return;
    }
    for (i = 0; i < 16; i++) {
        for (j = 0; j < 2; j++) {
            if (selected_iref == 1) {
                temp_iref[i][j] = E_IrefTable_1[i][j];
            } else if (selected_iref == 2) {
                temp_iref[i][j] = E_IrefTable_2[i][j];
            } else if (selected_iref == 3) {
                temp_iref[i][j] = E_IrefTable_3[i][j];
            } else if (selected_iref == 4) {
                temp_iref[i][j] = E_IrefTable_4[i][j];
            } else if (selected_iref == 5) {
                temp_iref[i][j] = E_IrefTable_5[i][j];
            } else if (selected_iref == 6) {
                temp_iref[i][j] = E_IrefTable_6[i][j];
            } else if (selected_iref == 7) {
                temp_iref[i][j] = E_IrefTable_7[i][j];
            }
        }
    }
    if (!iref_found) {
        //Read Iref
        //Register 0x43
        cmd[0] = 0x01;
        cmd[1] = 0x00;
        cmd[2] = 0x0A;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 3, 3) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return;
        }
        //Register 0x44
        cmd[0] = 0x00;
        cmd[1] = 0x00;
        cmd[2] = 0x00;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, 3) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return ;
        }
        //Register 0x46
        if ( i2c_himax_write(private_ts->client, 0x46 , &cmd[0], 0, 3) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return ;
        }
        //Register 0x59
        if ( i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return ;
        }
        //find iref group , default is iref 3
        for (i = 0; i < 16; i++) {
            if ((cmd[0] == temp_iref[i][0]) &&
                (cmd[1] == temp_iref[i][1])) {
                iref_number = i;
                iref_found = true;
                break;
            }
        }
        if (!iref_found ) {
            E("%s: Can't find iref number!\n", __func__);
            return ;
        } else {
            I("%s: iref_number=%d, cmd[0]=0x%x, cmd[1]=0x%x\n", __func__, iref_number, cmd[0], cmd[1]);
        }
    }
    msleep(5);
    //iref write
    //Register 0x43
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x06;
    if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 3, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    //Register 0x44
    cmd[0] = 0x00;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    //Register 0x45
    cmd[0] = temp_iref[iref_number][0];
    cmd[1] = temp_iref[iref_number][1];
    cmd[2] = 0x17;
    cmd[3] = 0x28;
    if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER , &cmd[0], 4, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    //Register 0x4A
    if ( i2c_himax_write(private_ts->client, HX_CMD_4A , &cmd[0], 0, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    //Read SFR to check the result
    //Register 0x43
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x0A;
    if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 3, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    //Register 0x44
    cmd[0] = 0x00;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    //Register 0x46
    if ( i2c_himax_write(private_ts->client, 0x46 , &cmd[0], 0, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    //Register 0x59
    if ( i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return ;
    }
    I( "%s:cmd[0]=%d,cmd[1]=%d,temp_iref_1=%d,temp_iref_2=%d\n", __func__, cmd[0], cmd[1], temp_iref[iref_number][0], temp_iref[iref_number][1]);
    if (cmd[0] != temp_iref[iref_number][0] || cmd[1] != temp_iref[iref_number][1]) {
        E("%s: IREF Read Back is not match.\n", __func__);
        E("%s: Iref [0]=%d,[1]=%d\n", __func__, cmd[0], cmd[1]);
    } else {
        I("%s: IREF Pass", __func__);
    }
    if (i2c_himax_write(private_ts->client, 0xA9 , &cmd[0], 0, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return;
    }
}

static uint8_t himax_calculateChecksum(bool change_iref, bool sense_off)
{
    int iref_flag = 0;
    uint8_t cmd[10];
    memset(cmd, 0x00, sizeof(cmd));
    if (sense_off) {
        if ( i2c_himax_write(private_ts->client, HX_CMD_TSSOFF , &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
            return 0;
        }
        msleep(30);
    }
    //Sleep out
    if ( i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT , &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    msleep(120);
    while (true) {
        if (change_iref) {
            if (iref_flag == 0) {
                himax_changeIref(2); //iref 2
            } else if (iref_flag == 1) {
                himax_changeIref(5); //iref 5
            } else if (iref_flag == 2) {
                himax_changeIref(1); //iref 1
            } else {
                goto CHECK_FAIL;
            }
            iref_flag ++;
        }
        cmd[0] = 0x00;
        cmd[1] = 0x04;
        cmd[2] = 0x0A;
        cmd[3] = 0x02;
        if (i2c_himax_write(private_ts->client, 0xED , &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        //Enable Flash
        cmd[0] = 0x01;
        cmd[1] = 0x00;
        cmd[2] = 0x02;
        if (i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        cmd[0] = 0x05;
        if (i2c_himax_write(private_ts->client, 0xD2 , &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        cmd[0] = 0x01;
        if (i2c_himax_write(private_ts->client, 0x53 , &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        msleep(200);
        if (i2c_himax_read(private_ts->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return -1;
        }
        I("%s 0xAD[0,1,2,3] = %d,%d,%d,%d \n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
        IC_checksum[0] = cmd[0];
        IC_checksum[1] = cmd[1];
        IC_checksum[2] = cmd[2];
        IC_checksum[3] = cmd[3];
        if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0 ) {
            himax_FlashMode(0);
            goto CHECK_PASS;
        } else {
            himax_FlashMode(0);
            goto CHECK_FAIL;
        }
CHECK_PASS:
        if (change_iref) {
            if (iref_flag < 3) {
                continue;
            } else {
                if (sense_off)
                    i2c_himax_write_command(private_ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
                return 1;
            }
        } else {
            if (sense_off)
                i2c_himax_write_command(private_ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
            return 1;
        }
CHECK_FAIL:
        if (sense_off)
            i2c_himax_write_command(private_ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
        return 0;
    }
    if (sense_off)
        i2c_himax_write_command(private_ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
    return 0;
}

int fts_ctpm_fw_upgrade_with_fs(unsigned char *fw, int len, bool change_iref)
{
    unsigned char *ImageBuffer = fw;
    int fullFileLength = len;
    int i;
    uint8_t cmd[5], last_byte, prePage;
    int FileLength;
    uint8_t checksumResult = 0;
    FileLength = fullFileLength;
    if ( i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT , &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    msleep(120);
    himax_lock_flash(0);
    cmd[0] = 0x05;
    cmd[1] = 0x00;
    cmd[2] = 0x02;
    if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    if ( i2c_himax_write(private_ts->client, 0x4F , &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    msleep(50);
    himax_ManualMode(1);
    himax_FlashMode(1);
    FileLength = (FileLength + 3) / 4;
    for (i = 0, prePage = 0; i < FileLength; i++) {
        last_byte = 0;
        cmd[0] = i & 0x1F;
        if (cmd[0] == 0x1F || i == FileLength - 1) {
            last_byte = 1;
        }
        cmd[1] = (i >> 5) & 0x1F;
        cmd[2] = (i >> 10) & 0x1F;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        if (prePage != cmd[1] || i == 0) {
            prePage = cmd[1];
            cmd[0] = 0x01;
            cmd[1] = 0x09;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E("%s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x0D;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E("%s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x09;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E("%s: i2c access fail!\n", __func__);
                return 0;
            }
        }
        memcpy(&cmd[0], &ImageBuffer[4 * i], 4);
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER , &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        cmd[0] = 0x01;
        cmd[1] = 0x0D;//cmd[2] = 0x02;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        cmd[0] = 0x01;
        cmd[1] = 0x09;//cmd[2] = 0x02;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
            E("%s: i2c access fail!\n", __func__);
            return 0;
        }
        if (last_byte == 1) {
            cmd[0] = 0x01;
            cmd[1] = 0x01;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E("%s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x05;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E("%s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x01;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E("%s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x00;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E("%s: i2c access fail!\n", __func__);
                return 0;
            }
            msleep(10);
            if (i == (FileLength - 1)) {
                himax_FlashMode(0);
                himax_ManualMode(0);
                checksumResult = himax_calculateChecksum(change_iref, false);
                //himax_ManualMode(0);
                himax_lock_flash(1);
                if (checksumResult) { //Success
                    return 1;
                } else { //Fail
                    E("%s: checksumResult fail!\n", __func__);
                    return 0;
                }
            }
        }
    }
    return 0;
}

static int himax_input_register(struct himax_ts_data *ts)
{
    int ret;
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        E("%s: Failed to allocate input device\n", __func__);
        return ret;
    }
    ts->input_dev->name = INPUT_DEV_NAME;
    set_bit(EV_SYN, ts->input_dev->evbit);
    set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);
#ifdef HX_SMART_WAKEUP
    set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
#ifdef HX_PALM_REPORT
    set_bit(KEY_SLEEP, ts->input_dev->keybit);
#endif
    set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    if (ts->protocol_type == PROTOCOL_TYPE_A) {
        //ts->input_dev->mtsize = ts->nFinger_support;
        input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
                     0, 3, 0, 0);
    } else {/* PROTOCOL_TYPE_B */
        set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
        input_mt_init_slots(ts->input_dev, ts->nFinger_support, 0);
    }
    I("input_set_abs_params: min_x %d, max_x %d, min_y %d, max_y %d\n",
      ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);
//    input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
//    input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);
    return input_register_device(ts->input_dev);
}

static void calcDataSize(uint8_t finger_num)
{
    struct himax_ts_data *ts_data = private_ts;
    ts_data->coord_data_size = 4 * finger_num;
    ts_data->area_data_size = ((finger_num / 4) + (finger_num % 4 ? 1 : 0)) * 4;
    ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;
    ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
                      ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
                     (((uint32_t)ts_data->x_channel * ts_data->y_channel +
                       ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size) ? 1 : 0;
    I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

static void calculate_point_number(void)
{
    HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4 ;
    if ( (HX_MAX_PT % 4) == 0)
        HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4 ;
    else
        HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) + 1) * 4 ;
}
#ifdef HX_LOADIN_CONFIG
static int himax_config_reg_write(struct i2c_client *client, uint8_t StartAddr, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    char cmd[12] = {0};
    cmd[0] = 0x8C;
    cmd[1] = 0x14;
    if ((i2c_himax_master_write(client, &cmd[0], 2, toRetry)) < 0)
        return -1;
    cmd[0] = 0x8B;
    cmd[1] = 0x00;
    cmd[2] = StartAddr;    //Start addr
    if ((i2c_himax_master_write(client, &cmd[0], 3, toRetry)) < 0)
        return -1;
    if ((i2c_himax_master_write(client, data, length, toRetry)) < 0)
        return -1;
    cmd[0] = 0x8C;
    cmd[1] = 0x00;
    if ((i2c_himax_master_write(client, &cmd[0], 2, toRetry)) < 0)
        return -1;
    return 0;
}
#endif
void himax_touch_information(void)
{
    char data[12] = {0};
    I("%s:IC_TYPE =%d\n", __func__, IC_TYPE);
    if (IC_TYPE == HX_85XX_ES_SERIES_PWON) {
        data[0] = 0x8C;
        data[1] = 0x14;
        i2c_himax_master_write(private_ts->client, &data[0], 2, DEFAULT_RETRY_CNT);
        msleep(10);
        data[0] = 0x8B;
        data[1] = 0x00;
        data[2] = 0x70;
        i2c_himax_master_write(private_ts->client, &data[0], 3, DEFAULT_RETRY_CNT);
        msleep(10);
        i2c_himax_read(private_ts->client, 0x5A, data, 12, DEFAULT_RETRY_CNT);
        HX_RX_NUM = data[0];                    // FE(70)
        HX_TX_NUM = data[1];                    // FE(71)
        //HX_MAX_PT = (data[2] & 0xF0) >> 4;      // FE(72)
        HX_MAX_PT = 2;
        if ((data[4] & 0x04) == 0x04) {         //FE(74)
            HX_XY_REVERSE = true;
            HX_Y_RES = data[6] * 256 + data[7]; //FE(76),FE(77)
            HX_X_RES = data[8] * 256 + data[9]; //FE(78),FE(79)
        } else {
            HX_XY_REVERSE = false;
            HX_X_RES = data[6] * 256 + data[7]; //FE(76),FE(77)
            HX_Y_RES = data[8] * 256 + data[9]; //FE(78),FE(79)
        }
        data[0] = 0x8C;
        data[1] = 0x00;
        i2c_himax_master_write(private_ts->client, &data[0], 2, DEFAULT_RETRY_CNT);
        msleep(10);
#ifdef HX_TP_PROC_2T2R
        data[0] = 0x8C;
        data[1] = 0x14;
        i2c_himax_master_write(private_ts->client, &data[0], 2, DEFAULT_RETRY_CNT);
        msleep(10);
        data[0] = 0x8B;
        data[1] = 0x00;
        data[2] = HX_2T2R_Addr;
        i2c_himax_master_write(private_ts->client, &data[0], 3, DEFAULT_RETRY_CNT);
        msleep(10);
        i2c_himax_read(private_ts->client, 0x5A, data, 10, DEFAULT_RETRY_CNT);
        HX_RX_NUM_2 = data[0];
        HX_TX_NUM_2 = data[1];
        I("%s:Touch Panel Type=%d \n", __func__, data[2]);
        if (data[2] == HX_2T2R_en_setting) //2T2R type panel
            Is_2T2R = true;
        else
            Is_2T2R = false;
        data[0] = 0x8C;
        data[1] = 0x00;
        i2c_himax_master_write(private_ts->client, &data[0], 2, DEFAULT_RETRY_CNT);
        msleep(10);
#endif
        data[0] = 0x8C;
        data[1] = 0x14;
        i2c_himax_master_write(private_ts->client, &data[0], 2, DEFAULT_RETRY_CNT);
        msleep(10);
        data[0] = 0x8B;
        data[1] = 0x00;
        data[2] = 0x02;
        i2c_himax_master_write(private_ts->client, &data[0], 3, DEFAULT_RETRY_CNT);
        msleep(10);
        i2c_himax_read(private_ts->client, 0x5A, data, 10, DEFAULT_RETRY_CNT);
        if ((data[1] & 0x01) == 1) { //FE(02)
            HX_INT_IS_EDGE = true;
        } else {
            HX_INT_IS_EDGE = false;
        }
        data[0] = 0x8C;
        data[1] = 0x00;
        i2c_himax_master_write(private_ts->client, &data[0], 2, DEFAULT_RETRY_CNT);
        msleep(10);
        I("%s:HX_RX_NUM =%d,HX_TX_NUM =%d,HX_MAX_PT=%d \n", __func__, HX_RX_NUM, HX_TX_NUM, HX_MAX_PT);
        if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, data, 1, 3) < 0) {
            E("%s: i2c access fail!\n", __func__);
        }
        private_ts->vendor_config_ver = data[0];
        I("config_ver=%x.\n", private_ts->vendor_config_ver);
    } else {
        HX_RX_NUM                = 0;
        HX_TX_NUM                = 0;
        HX_BT_NUM                = 0;
        HX_X_RES                = 0;
        HX_Y_RES                = 0;
        HX_MAX_PT                = 0;
        HX_XY_REVERSE            = false;
        HX_INT_IS_EDGE            = false;
    }
}
static uint8_t himax_read_Sensor_ID(struct i2c_client *client)
{
    uint8_t val_high[1], val_low[1], ID0 = 0, ID1 = 0;
    uint8_t sensor_id;
    char data[3];
    data[0] = 0x56;
    data[1] = 0x02;
    data[2] = 0x02;/*ID pin PULL High*/
    i2c_himax_master_write(client, &data[0], 3, DEFAULT_RETRY_CNT);
    msleep(1);
    //read id pin high
    i2c_himax_read(client, 0x57, val_high, 1, DEFAULT_RETRY_CNT);
    data[0] = 0x56;
    data[1] = 0x01;
    data[2] = 0x01;/*ID pin PULL Low*/
    i2c_himax_master_write(client, &data[0], 3, DEFAULT_RETRY_CNT);
    msleep(1);
    //read id pin low
    i2c_himax_read(client, 0x57, val_low, 1, DEFAULT_RETRY_CNT);
    if ((val_high[0] & 0x01) == 0)
        ID0 = 0x02; /*GND*/
    else if ((val_low[0] & 0x01) == 0)
        ID0 = 0x01; /*Floating*/
    else
        ID0 = 0x04; /*VCC*/
    if ((val_high[0] & 0x02) == 0)
        ID1 = 0x02; /*GND*/
    else if ((val_low[0] & 0x02) == 0)
        ID1 = 0x01; /*Floating*/
    else
        ID1 = 0x04; /*VCC*/
    if ((ID0 == 0x04) && (ID1 != 0x04)) {
        data[0] = 0x56;
        data[1] = 0x02;
        data[2] = 0x01;/*ID pin PULL High,Low*/
        i2c_himax_master_write(client, &data[0], 3, DEFAULT_RETRY_CNT);
        msleep(1);
    } else if ((ID0 != 0x04) && (ID1 == 0x04)) {
        data[0] = 0x56;
        data[1] = 0x01;
        data[2] = 0x02;/*ID pin PULL Low,High*/
        i2c_himax_master_write(client, &data[0], 3, DEFAULT_RETRY_CNT);
        msleep(1);
    } else if ((ID0 == 0x04) && (ID1 == 0x04)) {
        data[0] = 0x56;
        data[1] = 0x02;
        data[2] = 0x02;/*ID pin PULL High,High*/
        i2c_himax_master_write(client, &data[0], 3, DEFAULT_RETRY_CNT);
        msleep(1);
    }
    sensor_id = (ID1 << 4) | ID0;
    data[0] = 0xF3;
    data[1] = sensor_id;
    i2c_himax_master_write(client, &data[0], 2, DEFAULT_RETRY_CNT); /*Write to MCU*/
    msleep(1);
    return sensor_id;
}
static void himax_power_on_initCMD(struct i2c_client *client)
{
    I("%s:\n", __func__);
    //Sense on to update the information
    i2c_himax_write_command(client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
    msleep(30);
    i2c_himax_write_command(client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);
    msleep(50);
    i2c_himax_write_command(client, HX_CMD_TSSOFF, DEFAULT_RETRY_CNT);
    msleep(50);
    i2c_himax_write_command(client, HX_CMD_TSSLPIN, DEFAULT_RETRY_CNT);
    msleep(50);
    himax_touch_information();
    i2c_himax_write_command(client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
    msleep(30);
    i2c_himax_write_command(client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);
    msleep(50);
}

#ifdef HX_CHECK_CRC_AP
static bool Calculate_CRC_with_AP(unsigned char *FW_content)
{
    int i, j;
    int fw_data;
    int fw_data_2;
    int CRC = 0xFFFFFFFF;
    int PolyNomial = 0x82F63B78;
    int length = 0;

    length = 0x2000; //32k
    for (i = 0; i < length; i++) {
        fw_data = FW_content[i * 4 ];
        for (j = 1; j < 4; j++) {
            fw_data_2 = FW_content[i * 4 + j];
            fw_data += (fw_data_2) << (8 * j);
        }
        CRC = fw_data ^ CRC;
        for (j = 0; j < 32; j++) {
            if ((CRC % 2) != 0) {
                CRC = ((CRC >> 1) & 0x7FFFFFFF) ^ PolyNomial;
            } else {
                CRC = (((CRC >> 1) & 0x7FFFFFFF)& 0x7FFFFFFF);
            }
        }
    }
    I("CRC calculate from bin file is %x", CRC);

    if (CRC == 0)
        return true;
    else
        return false;
}
#endif

#ifdef HX_AUTO_UPDATE_FW
static int i_update_FW(bool manual)
{
    struct file *hx_filp = NULL;
    mm_segment_t oldfs;
    unsigned char *ImageBuffer;
    int fullFileLength;
    int upgrade_times = 0;
    int result;
    uint8_t FW_MAJ, FW_MIN, NEW_FW_MAJ, NEW_FW_MIN;
    uint8_t CFG_VER, NEW_CFG_VER;
    bool need_update_flag = false;
    bool asus_load_fail = false;

    if (private_ts->suspended) {
        goto no_update;
    }
    I("Start to update fw. \n");
    fw_update_processing= true;

    // load bin from ASUSFW
    hx_filp = filp_open(ASUS_FW_NAME, O_RDONLY, 0);
    if (IS_ERR(hx_filp)) {
        I("Open firmware file from ASUSFW failed \n");
        asus_load_fail = true;
    } else {
        oldfs = get_fs();
        set_fs(get_ds());
        result = hx_filp->f_op->read(hx_filp, upgrade_fw, sizeof(upgrade_fw), &hx_filp->f_pos);
        if (result < 0) {
            I("Read firmware file from ASUSFW failed \n");
            asus_load_fail = true;
        } else {
            set_fs(oldfs);
            filp_close(hx_filp, NULL);
            I("Update firmware from ASUSFW \n");
            ImageBuffer = (u8 *) upgrade_fw;
            fullFileLength = result;
        }
    }

    // load bin by using request_firmware
    if (asus_load_fail) {
        result = request_firmware(&private_ts->fw, HMX_FW_NAME, &private_ts->client->dev);
        if (private_ts->fw == NULL) {
            E("No firmware file, ignored firmware update \n");
            goto no_update;
        } else if (result) {
            E("Request_firmware failed, ret = %d \n", result);
            goto no_update;
        }
        private_ts->fw_data_start = (u8 *)private_ts->fw->data;
        private_ts->fw_size = private_ts->fw->size;
        ImageBuffer = private_ts->fw_data_start;
        fullFileLength = private_ts->fw->size;
    }

    FW_MAJ = private_ts->vendor_fw_ver_H;
    FW_MIN = private_ts->vendor_fw_ver_L;
    CFG_VER = private_ts->vendor_config_ver;
    NEW_FW_MAJ = ImageBuffer[FW_VER_MAJ_FLASH_ADDR];
    NEW_FW_MIN = ImageBuffer[FW_VER_MIN_FLASH_ADDR];
    NEW_CFG_VER = ImageBuffer[FW_CFG_VER_FLASH_ADDR];
    I("FW_VER=%2x,%2x. CFG_VER=%2x. \n", FW_MAJ, FW_MIN, CFG_VER);
    I("NEW FW_VER=%2x,%2x. NEW CFG_VER=%2x. \n", NEW_FW_MAJ, NEW_FW_MIN, NEW_CFG_VER);

#ifdef HX_CHECK_CRC_AP
    if (!Calculate_CRC_with_AP(ImageBuffer)) {
        I("New FW CRC fail, do not update.");
        goto no_update;
    }
#endif
    if (NEW_FW_MAJ != ASUS_FW_MAJ) {
        I("Non ASUS FW, do not update.");
        goto no_update;
    }

    if (!manual) {
        if ( (NEW_FW_MIN < FW_MIN) || ( (NEW_FW_MIN == FW_MIN) && (NEW_CFG_VER <= CFG_VER) )) {
            if ( himax_calculateChecksum(false, true) == 0 ) {
                I("IC Checksum fail, update to new FW. \n");
                need_update_flag = true;
            } else {
                I("No new FW version and IC Checksum pass, do not update. \n");
                goto no_update;
            }
        } else {
            I("Larger new FW version, update to new FW. \n");
            need_update_flag = true;
        }
    } else
        need_update_flag = true;

    if (need_update_flag) {
update_retry:
#ifdef HX_RST_PIN_FUNC
        himax_HW_reset(false, true);
#endif
        if (fts_ctpm_fw_upgrade_with_fs(ImageBuffer, fullFileLength, true) == 0) {
            if (upgrade_times < 3) {
                upgrade_times++;
                E("TP upgrade Error, Count: %d \n", upgrade_times);
                goto update_retry;
            } else {
                fw_update_result = false;
                fw_update_processing= false;
                return -1;//upgrade fail
            }
        } else {
            I("TP upgrade OK \n");
            private_ts->vendor_fw_ver_H = NEW_FW_MAJ;
            private_ts->vendor_fw_ver_L = NEW_FW_MIN;
            private_ts->vendor_config_ver = NEW_CFG_VER;
        }
#ifdef HX_RST_PIN_FUNC
        himax_HW_reset(true, true);
#endif
        fw_update_result = true;
        fw_update_processing= false;
        return 1;//upgrade success
    } else
        goto no_update;
no_update:
    fw_update_result = true;
    fw_update_processing= false;
    return 0;//NO upgrade
}
#endif

#ifdef HX_AUTO_UPDATE_CONFIG
int fts_ctpm_fw_upgrade_with_i_file_flash_cfg(struct himax_config *cfg)
{
    unsigned char *ImageBuffer = i_CTPM_FW; //NULL ;//
    int fullFileLength = FLASH_SIZE;
    int i, k;
    uint8_t cmd[5], last_byte, prePage;
    uint8_t tmp[5];
    int FileLength;
    int Polynomial = 0x82F63B78;
    int CRC32 = 0xFFFFFFFF;
    int BinDataWord = 0;
    int current_index = 0;
    uint8_t checksumResult = 0;
    I(" %s: flash CONFIG only START!\n", __func__);
#if 0
    //=========Dump FW image from Flash========
    setSysOperation(1);
    setFlashCommand(0x0F);
    setFlashDumpProgress(0);
    setFlashDumpComplete(0);
    setFlashDumpFail(0);
    queue_work(private_ts->flash_wq, &private_ts->flash_work);
    for (i = 0 ; i < 1024 ; i++) {
        if (getFlashDumpComplete()) {
            ImageBuffer = flash_buffer;
            fullFileLength = FLASH_SIZE;
            I(" %s: Load FW from flash OK! cycle=%d fullFileLength=%x\n", __func__, i, fullFileLength);
            break;
        }
        msleep(5);
    }
    if (i == 400) {
        E(" %s: Load FW from flash time out fail!\n", __func__);
        return 0;
    }
    //===================================
#endif
    current_index = CFB_START_ADDR + CFB_INFO_LENGTH;
    //Update the Config Part from config array
    for (i = 1 ; i < sizeof(cfg->c1) / sizeof(cfg->c1[0]) ; i++) ImageBuffer[current_index++] = cfg->c1[i];
    for (i = 1 ; i < sizeof(cfg->c2) / sizeof(cfg->c2[0]) ; i++) ImageBuffer[current_index++] = cfg->c2[i];
    for (i = 1 ; i < sizeof(cfg->c3) / sizeof(cfg->c3[0]) ; i++) ImageBuffer[current_index++] = cfg->c3[i];
    for (i = 1 ; i < sizeof(cfg->c4) / sizeof(cfg->c4[0]) ; i++) ImageBuffer[current_index++] = cfg->c4[i];
    for (i = 1 ; i < sizeof(cfg->c5) / sizeof(cfg->c5[0]) ; i++) ImageBuffer[current_index++] = cfg->c5[i];
    for (i = 1 ; i < sizeof(cfg->c6) / sizeof(cfg->c6[0]) ; i++) ImageBuffer[current_index++] = cfg->c6[i];
    for (i = 1 ; i < sizeof(cfg->c7) / sizeof(cfg->c7[0]) ; i++) ImageBuffer[current_index++] = cfg->c7[i];
    for (i = 1 ; i < sizeof(cfg->c8) / sizeof(cfg->c8[0]) ; i++) ImageBuffer[current_index++] = cfg->c8[i];
    for (i = 1 ; i < sizeof(cfg->c9) / sizeof(cfg->c9[0]) ; i++) ImageBuffer[current_index++] = cfg->c9[i];
    for (i = 1 ; i < sizeof(cfg->c10) / sizeof(cfg->c10[0]) ; i++) ImageBuffer[current_index++] = cfg->c10[i];
    for (i = 1 ; i < sizeof(cfg->c11) / sizeof(cfg->c11[0]) ; i++) ImageBuffer[current_index++] = cfg->c11[i];
    for (i = 1 ; i < sizeof(cfg->c12) / sizeof(cfg->c12[0]) ; i++) ImageBuffer[current_index++] = cfg->c12[i];
    for (i = 1 ; i < sizeof(cfg->c13) / sizeof(cfg->c13[0]) ; i++) ImageBuffer[current_index++] = cfg->c13[i];
    for (i = 1 ; i < sizeof(cfg->c14) / sizeof(cfg->c14[0]) ; i++) ImageBuffer[current_index++] = cfg->c14[i];
    for (i = 1 ; i < sizeof(cfg->c15) / sizeof(cfg->c15[0]) ; i++) ImageBuffer[current_index++] = cfg->c15[i];
    for (i = 1 ; i < sizeof(cfg->c16) / sizeof(cfg->c16[0]) ; i++) ImageBuffer[current_index++] = cfg->c16[i];
    for (i = 1 ; i < sizeof(cfg->c17) / sizeof(cfg->c17[0]) ; i++) ImageBuffer[current_index++] = cfg->c17[i];
    for (i = 1 ; i < sizeof(cfg->c18) / sizeof(cfg->c18[0]) ; i++) ImageBuffer[current_index++] = cfg->c18[i];
    for (i = 1 ; i < sizeof(cfg->c19) / sizeof(cfg->c19[0]) ; i++) ImageBuffer[current_index++] = cfg->c19[i];
    for (i = 1 ; i < sizeof(cfg->c20) / sizeof(cfg->c20[0]) ; i++) ImageBuffer[current_index++] = cfg->c20[i];
    for (i = 1 ; i < sizeof(cfg->c21) / sizeof(cfg->c21[0]) ; i++) ImageBuffer[current_index++] = cfg->c21[i];
    for (i = 1 ; i < sizeof(cfg->c22) / sizeof(cfg->c22[0]) ; i++) ImageBuffer[current_index++] = cfg->c22[i];
    for (i = 1 ; i < sizeof(cfg->c23) / sizeof(cfg->c23[0]) ; i++) ImageBuffer[current_index++] = cfg->c23[i];
    for (i = 1 ; i < sizeof(cfg->c24) / sizeof(cfg->c24[0]) ; i++) ImageBuffer[current_index++] = cfg->c24[i];
    for (i = 1 ; i < sizeof(cfg->c25) / sizeof(cfg->c25[0]) ; i++) ImageBuffer[current_index++] = cfg->c25[i];
    for (i = 1 ; i < sizeof(cfg->c26) / sizeof(cfg->c26[0]) ; i++) ImageBuffer[current_index++] = cfg->c26[i];
    for (i = 1 ; i < sizeof(cfg->c27) / sizeof(cfg->c27[0]) ; i++) ImageBuffer[current_index++] = cfg->c27[i];
    for (i = 1 ; i < sizeof(cfg->c28) / sizeof(cfg->c28[0]) ; i++) ImageBuffer[current_index++] = cfg->c28[i];
    for (i = 1 ; i < sizeof(cfg->c29) / sizeof(cfg->c29[0]) ; i++) ImageBuffer[current_index++] = cfg->c29[i];
    for (i = 1 ; i < sizeof(cfg->c30) / sizeof(cfg->c30[0]) ; i++) ImageBuffer[current_index++] = cfg->c30[i];
    for (i = 1 ; i < sizeof(cfg->c31) / sizeof(cfg->c31[0]) ; i++) ImageBuffer[current_index++] = cfg->c31[i];
    for (i = 1 ; i < sizeof(cfg->c32) / sizeof(cfg->c32[0]) ; i++) ImageBuffer[current_index++] = cfg->c32[i];
    for (i = 1 ; i < sizeof(cfg->c33) / sizeof(cfg->c33[0]) ; i++) ImageBuffer[current_index++] = cfg->c33[i];
    for (i = 1 ; i < sizeof(cfg->c34) / sizeof(cfg->c34[0]) ; i++) ImageBuffer[current_index++] = cfg->c34[i];
    for (i = 1 ; i < sizeof(cfg->c35) / sizeof(cfg->c35[0]) ; i++) ImageBuffer[current_index++] = cfg->c35[i];
    for (i = 1 ; i < sizeof(cfg->c36) / sizeof(cfg->c36[0]) ; i++) ImageBuffer[current_index++] = cfg->c36[i];
    for (i = 1 ; i < sizeof(cfg->c37) / sizeof(cfg->c37[0]) ; i++) ImageBuffer[current_index++] = cfg->c37[i];
    for (i = 1 ; i < sizeof(cfg->c38) / sizeof(cfg->c38[0]) ; i++) ImageBuffer[current_index++] = cfg->c38[i];
    for (i = 1 ; i < sizeof(cfg->c39) / sizeof(cfg->c39[0]) ; i++) ImageBuffer[current_index++] = cfg->c39[i];
    current_index = current_index + 50; //dummy data
    //I(" %s: current_index=%d\n", __func__,current_index);
    for (i = 1 ; i < sizeof(cfg->c40) / sizeof(cfg->c40[0]) ; i++) ImageBuffer[current_index++] = cfg->c40[i];
    for (i = 1 ; i < sizeof(cfg->c41) / sizeof(cfg->c41[0]) ; i++) ImageBuffer[current_index++] = cfg->c41[i];
    //cal_checksum start//
    FileLength = fullFileLength;
    FileLength = (FileLength + 3) / 4;
    for (i = 0; i < FileLength; i++) {
        memcpy(&cmd[0], &ImageBuffer[4 * i], 4);
        if (i < (FileLength - 1)) { /*cal_checksum*/
            for (k = 0; k < 4; k++) {
                BinDataWord |= cmd[k] << (k * 8);
            }
            CRC32 = BinDataWord ^ CRC32;
            for (k = 0; k < 32; k++) {
                if ((CRC32 % 2) != 0) {
                    CRC32 = ((CRC32 >> 1) & 0x7FFFFFFF) ^ Polynomial;
                } else {
                    CRC32 = ((CRC32 >> 1) & 0x7FFFFFFF);
                }
            }
            BinDataWord = 0;
        }
    }
    //cal_checksum end//
    FileLength = fullFileLength;
    if ( i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT , &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
        E(" %s: i2c access fail!\n", __func__);
        return 0;
    }
    msleep(120);
    himax_lock_flash(0);
    himax_ManualMode(1);
    himax_FlashMode(1);
    FileLength = (FileLength + 3) / 4;
    for (i = 0, prePage = 0; i < FileLength; i++) {
        last_byte = 0;
        if (i < 0x20)
            continue;
        else if ((i > 0xBF) && (i < (FileLength - 0x20)))
            continue;
        cmd[0] = i & 0x1F;
        if (cmd[0] == 0x1F || i == FileLength - 1) {
            last_byte = 1;
        }
        cmd[1] = (i >> 5) & 0x1F;
        cmd[2] = (i >> 10) & 0x1F;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
            E(" %s: i2c access fail!\n", __func__);
            return 0;
        }
        //I(" %s: CMD 44 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
        if (prePage != cmd[1] || i == 0x20) {
            prePage = cmd[1];
            //I(" %s: %d page erase\n",__func__,prePage);
            tmp[0] = 0x01;
            tmp[1] = 0x09;//tmp[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            tmp[0] = 0x05;
            tmp[1] = 0x2D;//tmp[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            msleep(30);
            tmp[0] = 0x01;
            tmp[1] = 0x09;//tmp[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            //I(" %s: CMD 44-47 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
            tmp[0] = 0x01;
            tmp[1] = 0x09;//tmp[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            tmp[0] = 0x01;
            tmp[1] = 0x0D;//tmp[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            tmp[0] = 0x01;
            tmp[1] = 0x09;//tmp[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_SET_ADDRESS , &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            //I(" %s: CMD 44-45 [0,1,2]=[%x,%x,%x]\n", __func__,cmd[0],cmd[1],cmd[2]);
            //I(" %s: Erase Page num=%x\n", __func__,prePage);
        }
        memcpy(&cmd[0], &ImageBuffer[4 * i], 4);
        if (i == (FileLength - 1)) {
            tmp[0] = (CRC32 & 0xFF);
            tmp[1] = ((CRC32 >> 8) & 0xFF);
            tmp[2] = ((CRC32 >> 16) & 0xFF);
            tmp[3] = ((CRC32 >> 24) & 0xFF);
            memcpy(&cmd[0], &tmp[0], 4);
            I("%s last_byte = 1, CRC32= %x, =[0,1,2,3] = %x,%x,%x,%x \n", __func__, CRC32, tmp[0], tmp[1], tmp[2], tmp[3]);
        }
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_WRITE_REGISTER , &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
            E(" %s: i2c access fail!\n", __func__);
            return 0;
        }
        //I(" %s: CMD 48 \n", __func__);
        cmd[0] = 0x01;
        cmd[1] = 0x0D;//cmd[2] = 0x02;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
            E(" %s: i2c access fail!\n", __func__);
            return 0;
        }
        cmd[0] = 0x01;
        cmd[1] = 0x09;//cmd[2] = 0x02;
        if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
            E(" %s: i2c access fail!\n", __func__);
            return 0;
        }
        if (last_byte == 1) {
            cmd[0] = 0x01;
            cmd[1] = 0x01;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x05;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x01;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            cmd[0] = 0x01;
            cmd[1] = 0x00;//cmd[2] = 0x02;
            if ( i2c_himax_write(private_ts->client, HX_CMD_FLASH_ENABLE , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
                E(" %s: i2c access fail!\n", __func__);
                return 0;
            }
            msleep(10);
            if (i == (FileLength - 1)) {
                himax_FlashMode(0);
                himax_ManualMode(0);
                checksumResult = himax_calculateChecksum(true, false);
                //himax_ManualMode(0);
                himax_lock_flash(1);
                I(" %s: flash CONFIG only END!\n", __func__);
                if (checksumResult) { //Success
                    return 1;
                } else { //Fail
                    E("%s: checksumResult fail!\n", __func__);
                    return 0;
                }
            }
        }
    }
    return 0;
}

static int i_update_FWCFG(struct himax_config *cfg)
{
    int upgrade_times = 0;
    I("%s: CHIP CONFG version=%x\n", __func__, private_ts->vendor_config_ver);
    I("%s: LOAD CONFG version=%x\n", __func__, cfg->c40[1]);
    if ( private_ts->vendor_config_ver != cfg->c40[1] ) {
update_retry:
#ifdef HX_RST_PIN_FUNC
        himax_HW_reset(false, true);
#endif
        if (fts_ctpm_fw_upgrade_with_i_file_flash_cfg(cfg) == 0) {
            upgrade_times++;
            E("%s: TP upgrade error times=%d\n", __func__, upgrade_times);
            if (upgrade_times < 3)
                goto update_retry;
        } else
            I("%s: TP upgrade OK\n", __func__);
#ifdef HX_RST_PIN_FUNC
        himax_HW_reset(false, true);
#endif
        return 1;
    } else
        return 0;
}

#endif

static int himax_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata)
{
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
    int rc = 0;
#endif
#ifndef CONFIG_OF
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
    int i = 0;
#endif
#endif
#ifdef HX_ESD_WORKAROUND
    char data[12] = {0};
#endif
    if (!client) {
        E("%s: Necessary parameters client are null!\n", __func__);
        return -1;
    }
    if (config_load == false) {
        config_selected = kzalloc(sizeof(*config_selected), GFP_KERNEL);
        if (config_selected == NULL) {
            E("%s: alloc config_selected fail!\n", __func__);
            return -1;
        }
    }
#ifndef CONFIG_OF
    pdata = client->dev.platform_data;
    if (!pdata) {
        E("%s: Necessary parameters pdata are null!\n", __func__);
        return -1;
    }
#endif
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
#ifdef CONFIG_OF
    //I("%s, config_selected, %X\n", __func__ ,(uint32_t)config_selected);
    if (config_load == false) {
        rc = himax_parse_config(private_ts, config_selected);
        if (rc < 0) {
            E(" DT:cfg table parser FAIL. ret=%d\n", rc);
            goto HimaxErr;
        } else if (rc == 0) {
            if ((private_ts->tw_x_max) && (private_ts->tw_y_max)) {
                pdata->abs_x_min = private_ts->tw_x_min;
                pdata->abs_x_max = private_ts->tw_x_max;
                pdata->abs_y_min = private_ts->tw_y_min;
                pdata->abs_y_max = private_ts->tw_y_max;
                I(" DT-%s:config-panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
                  pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
            }
            if ((private_ts->pl_x_max) && (private_ts->pl_y_max)) {
                pdata->screenWidth = private_ts->pl_x_max;
                pdata->screenHeight = private_ts->pl_y_max;
                I(" DT-%s:config-display-coords = (%d, %d)", __func__, pdata->screenWidth,
                  pdata->screenHeight);
            }
            config_load = true;
            I(" DT parser Done\n");
        }
    }
#else
    I("pdata->hx_config_size=%x.\n", (pdata->hx_config_size + 1));
    I("config_type_size=%x.\n", sizeof(struct himax_config));
    if (pdata->hx_config) {
        for (i = 0; i < pdata->hx_config_size / sizeof(struct himax_config); ++i) {
            I("(pdata->hx_config)[%x].fw_ver_main=%x.\n", i, (pdata->hx_config)[i].fw_ver_main);
            I("(pdata->hx_config)[%x].fw_ver_minor=%x.\n", i, (pdata->hx_config)[i].fw_ver_minor);
            I("(pdata->hx_config)[%x].sensor_id=%x.\n", i, (pdata->hx_config)[i].sensor_id);
            if ((private_ts->vendor_fw_ver_H << 8 | private_ts->vendor_fw_ver_L) <
                ((pdata->hx_config)[i].fw_ver_main << 8 | (pdata->hx_config)[i].fw_ver_minor)) {
                continue;
            } else {
                if ((private_ts->vendor_sensor_id == (pdata->hx_config)[i].sensor_id)) {
                    config_selected = &((pdata->hx_config)[i]);
                    I("hx_config selected, %X\n", (uint32_t)config_selected);
                    config_load = true;
                    break;
                } else if ((pdata->hx_config)[i].default_cfg) {
                    I("default_cfg detected.\n");
                    config_selected = &((pdata->hx_config)[i]);
                    I("hx_config selected, %X\n", (uint32_t)config_selected);
                    config_load = true;
                    break;
                }
            }
        }
    } else {
        E("[HimaxError] %s pdata->hx_config is not exist \n", __func__);
        goto HimaxErr;
    }
#endif
#endif
#ifdef HX_LOADIN_CONFIG
    if (config_selected) {
        //Read config id
        private_ts->vendor_config_ver = config_selected->c40[1];
        //Normal Register c1~c35
        i2c_himax_master_write(client, config_selected->c1, sizeof(config_selected->c1), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c2, sizeof(config_selected->c2), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c3, sizeof(config_selected->c3), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c4, sizeof(config_selected->c4), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c5, sizeof(config_selected->c5), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c6, sizeof(config_selected->c6), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c7, sizeof(config_selected->c7), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c8, sizeof(config_selected->c8), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c9, sizeof(config_selected->c9), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c10, sizeof(config_selected->c10), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c11, sizeof(config_selected->c11), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c12, sizeof(config_selected->c12), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c13, sizeof(config_selected->c13), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c14, sizeof(config_selected->c14), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c15, sizeof(config_selected->c15), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c16, sizeof(config_selected->c16), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c17, sizeof(config_selected->c17), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c18, sizeof(config_selected->c18), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c19, sizeof(config_selected->c19), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c20, sizeof(config_selected->c20), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c21, sizeof(config_selected->c21), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c22, sizeof(config_selected->c22), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c23, sizeof(config_selected->c23), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c24, sizeof(config_selected->c24), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c25, sizeof(config_selected->c25), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c26, sizeof(config_selected->c26), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c27, sizeof(config_selected->c27), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c28, sizeof(config_selected->c28), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c29, sizeof(config_selected->c29), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c30, sizeof(config_selected->c30), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c31, sizeof(config_selected->c31), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c32, sizeof(config_selected->c32), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c33, sizeof(config_selected->c33), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c34, sizeof(config_selected->c34), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c35, sizeof(config_selected->c35), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c36, sizeof(config_selected->c36), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c37, sizeof(config_selected->c37), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c38, sizeof(config_selected->c38), DEFAULT_RETRY_CNT);
        i2c_himax_master_write(client, config_selected->c39, sizeof(config_selected->c39), DEFAULT_RETRY_CNT);
        //Config Bank register
        himax_config_reg_write(client, 0x00, config_selected->c40, sizeof(config_selected->c40), DEFAULT_RETRY_CNT);
        himax_config_reg_write(client, 0x9E, config_selected->c41, sizeof(config_selected->c41), DEFAULT_RETRY_CNT);
        msleep(1);
    } else {
        E("[HimaxError] %s config_selected is null.\n", __func__);
        goto HimaxErr;
    }
#endif
#ifdef HX_ESD_WORKAROUND
    //Check R36 to check IC Status
    i2c_himax_read(client, 0x36, data, 2, 10);
    if (data[0] != 0x0F || data[1] != 0x53) {
        //IC is abnormal
        E("[HimaxError] %s R36 Fail : R36[0]=%d,R36[1]=%d,R36 Counter=%d \n", __func__, data[0], data[1], ESD_R36_FAIL);
        return -1;
    }
#endif
#ifdef HX_AUTO_UPDATE_CONFIG
    if (i_update_FWCFG(config_selected) == false)
        I("NOT Have new FWCFG=NOT UPDATE=\n");
    else
        I("Have new FWCFG=UPDATE=\n");
#endif
    himax_power_on_initCMD(client);
    I("%s: initialization complete\n", __func__);
    return 1;
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
HimaxErr:
    return -1;
#endif
}

#ifdef HX_RST_PIN_FUNC
void himax_HW_reset(uint8_t loadconfig, uint8_t int_off)
{
    struct himax_ts_data *ts = private_ts;
    int ret = 0;
    HW_RESET_ACTIVATE = 1;
    if (ts->rst_gpio) {
        if (!int_off) {
            if (ts->use_irq)
                himax_int_enable(private_ts->client->irq, 0, true);
            else {
                hrtimer_cancel(&ts->timer);
                ret = cancel_work_sync(&ts->work);
            }
        }
        I("%s: Now reset the Touch chip.\n", __func__);
        himax_rst_gpio_set(ts->rst_gpio, 0);
        msleep(20);
        himax_rst_gpio_set(ts->rst_gpio, 1);
        msleep(20);
        if (loadconfig)
            himax_loadSensorConfig(private_ts->client, private_ts->pdata);
        if (!int_off) {
            if (ts->use_irq)
                himax_int_enable(private_ts->client->irq, 1, true);
            else
                hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        }
    }
}
#endif

static u8 himax_read_FW_ver(bool hw_reset)
{
    uint8_t cmd[3];
    himax_int_enable(private_ts->client->irq, 0, true);
#ifdef HX_RST_PIN_FUNC
    if (hw_reset) {
        himax_HW_reset(false, true);
    }
#endif
    msleep(120);
    if (i2c_himax_read(private_ts->client, HX_VER_FW_MAJ, cmd, 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    private_ts->vendor_fw_ver_H = cmd[0];
    if (i2c_himax_read(private_ts->client, HX_VER_FW_MIN, cmd, 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    private_ts->vendor_fw_ver_L = cmd[0];
    I("FW_VER : %x,%x \n", private_ts->vendor_fw_ver_H, private_ts->vendor_fw_ver_L);
    if (i2c_himax_read(private_ts->client, HX_VER_FW_CFG, cmd, 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return 0;
    }
    private_ts->vendor_config_ver = cmd[0];
    I("CFG_VER : %x \n", private_ts->vendor_config_ver);
#ifdef HX_RST_PIN_FUNC
    if (hw_reset)
        himax_HW_reset(true, true);
#endif
    himax_int_enable(private_ts->client->irq, 1, true);
    return 0;
}

static bool himax_ic_package_check(struct himax_ts_data *ts)
{
    uint8_t cmd[3];
    memset(cmd, 0x00, sizeof(cmd));
    if (i2c_himax_read(ts->client, 0xD1, cmd, 3, DEFAULT_RETRY_CNT) < 0)
        return false ;
    if (cmd[0] == 0x05 && cmd[1] == 0x85 &&
        (cmd[2] == 0x25 || cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28)) {
        IC_TYPE = HX_85XX_ES_SERIES_PWON;
        IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
        //Himax: Set FW and CFG Flash Address
        FW_VER_MAJ_FLASH_ADDR = 133;    //0x0085
        FW_VER_MAJ_FLASH_LENG = 1;
        FW_VER_MIN_FLASH_ADDR = 134;    //0x0086
        FW_VER_MIN_FLASH_LENG = 1;
        CFG_VER_MAJ_FLASH_ADDR = 160;   //0x00A0
        CFG_VER_MAJ_FLASH_LENG = 12;
        CFG_VER_MIN_FLASH_ADDR = 172;   //0x00AC
        CFG_VER_MIN_FLASH_LENG = 12;
        FW_CFG_VER_FLASH_ADDR = 132;    //0x0084
#ifdef HX_AUTO_UPDATE_CONFIG
        CFB_START_ADDR = 0x80;
        CFB_LENGTH = 638;
        CFB_INFO_LENGTH = 68;
#endif
        I("Himax IC package 852x ES\n");
    } else {
        E("Himax IC package incorrect!!PKG[0]=%x,PKG[1]=%x,PKG[2]=%x\n", cmd[0], cmd[1], cmd[2]);
        return false ;
    }
    return true;
}

static void himax_read_TP_info(struct i2c_client *client)
{
    char data[12] = {0};
    //read fw version
    if (i2c_himax_read(client, HX_VER_FW_MAJ, data, 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return;
    }
    private_ts->vendor_fw_ver_H = data[0];
    if (i2c_himax_read(client, HX_VER_FW_MIN, data, 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return;
    }
    private_ts->vendor_fw_ver_L = data[0];
    //read config version
    if (i2c_himax_read(client, HX_VER_FW_CFG, data, 1, 3) < 0) {
        E("%s: i2c access fail!\n", __func__);
        return;
    }
    private_ts->vendor_config_ver = data[0];
    //read sensor ID
    private_ts->vendor_sensor_id = himax_read_Sensor_ID(client);
    I("sensor_id=%x.\n", private_ts->vendor_sensor_id);
    I("fw_ver=%x,%x.\n", private_ts->vendor_fw_ver_H, private_ts->vendor_fw_ver_L);
    I("config_ver=%x.\n", private_ts->vendor_config_ver);
}

#ifdef HX_ESD_WORKAROUND
void ESD_HW_REST(void)
{
    ESD_RESET_ACTIVATE = 1;
    ESD_COUNTER = 0;
    ESD_R36_FAIL = 0;
#ifdef HX_CHIP_STATUS_MONITOR
    HX_CHIP_POLLING_COUNT = 0;
#endif
    I("START_Himax TP: ESD - Reset\n");
    while (ESD_R36_FAIL <= 3 ) {
        himax_rst_gpio_set(private_ts->rst_gpio, 0);
        msleep(20);
        himax_rst_gpio_set(private_ts->rst_gpio, 1);
        msleep(20);
        if (himax_loadSensorConfig(private_ts->client, private_ts->pdata) < 0)
            ESD_R36_FAIL++;
        else
            break;
    }
    I("END_Himax TP: ESD - Reset\n");
}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
static void himax_chip_monitor_function(struct work_struct *work) //for ESD solution
{
    int ret = 0;
    I(" %s: POLLING_COUNT=%x, STATUS=%x \n", __func__, HX_CHIP_POLLING_COUNT, ret);
    if (HX_CHIP_POLLING_COUNT >= (HX_POLLING_TIMES - 1)) { //POLLING TIME
        HX_ON_HAND_SHAKING = 1;
        ret = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail
        HX_ON_HAND_SHAKING = 0;
        if (ret == 2) {
            I(" %s: I2C Fail \n", __func__);
            ESD_HW_REST();
        } else if (ret == 1) {
            I(" %s: MCU Stop \n", __func__);
            ESD_HW_REST();
        }
        HX_CHIP_POLLING_COUNT = 0; //clear polling counter
    } else
        HX_CHIP_POLLING_COUNT++;
    queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMER * HZ);
    return;
}
#endif

#ifdef HX_SMART_WAKEUP
static int himax_parse_wake_event(struct himax_ts_data *ts)
{
    uint8_t buf[5];
    if (i2c_himax_read(ts->client, 0x86, buf, 4, DEFAULT_RETRY_CNT))
        E("%s: can't read data from chip!\n", __func__);
    if ((buf[0] == 0x57) && (buf[1] == 0x61) && (buf[2] == 0x6B) && (buf[3] == 0x65)) {
        I("%s: WAKE UP system!\n", __func__);
        return 1;//Yes, wake up system
    } else {
        I("%s: NOT WKAE packet, SKIP!\n", __func__);
        I("buf[0]=%x, buf[1]=%x, buf[2]=%x, buf[3]=%x\n", buf[0], buf[1], buf[2], buf[3]);
        return 0;
    }
}
#endif

inline void himax_ts_work(struct himax_ts_data *ts)
{
    int ret = 0;
    uint8_t buf[128], finger_num, hw_reset_check[2];
    uint16_t finger_pressed;
    int32_t loop_i;
    unsigned char check_sum_cal = 0;
    int RawDataLen = 0;
    int raw_cnt_max, raw_cnt_rmd, hx_touch_info_size;
    int base, x, y, w;
    uint8_t coordInfoSize = ts->coord_data_size + ts->area_data_size + 4;
#ifdef HX_TP_PROC_DIAG
    uint8_t *mutual_data;
    uint8_t *self_data;
    uint8_t diag_cmd;
    int i, mul_num, self_num;
    int index = 0;
    int temp1, temp2;
    //coordinate dump start
    char coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 2];
    struct timeval t;
    struct tm broken;
    //coordinate dump end
#endif
#ifdef HX_CHIP_STATUS_MONITOR
    int j = 0;
#endif
    memset(buf, 0x00, sizeof(buf));
    memset(hw_reset_check, 0x00, sizeof(hw_reset_check));
#ifdef HX_CHIP_STATUS_MONITOR
    HX_CHIP_POLLING_COUNT = 0;
    if (HX_ON_HAND_SHAKING) { //chip on hand shaking,wait hand shaking
        for (j = 0; j < 100; j++) {
            if (HX_ON_HAND_SHAKING == 0) { //chip on hand shaking end
                I("%s:HX_ON_HAND_SHAKING OK check %d times\n", __func__, j);
                break;
            } else
                msleep(1);
        }
        if (j == 100) {
            E("%s:HX_ON_HAND_SHAKING timeout reject interrupt\n", __func__);
            return;
        }
    }
#endif
    raw_cnt_max = HX_MAX_PT / 4;
    raw_cnt_rmd = HX_MAX_PT % 4;
    if (raw_cnt_rmd != 0x00) { //more than 4 fingers
        RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
        hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 2) * 4;
    } else { //less than 4 fingers
        RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;
        hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 1) * 4;
    }
#ifdef HX_TP_PROC_DIAG
    diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
    if ((diag_cmd) || (ESD_RESET_ACTIVATE) || (HW_RESET_ACTIVATE))
#else
    if ((diag_cmd) || (HW_RESET_ACTIVATE))
#endif
    {
        ret = i2c_himax_read(ts->client, 0x86, buf, 128, DEFAULT_RETRY_CNT);
    } else {
        if (touch_monitor_stop_flag != 0) {
            ret = i2c_himax_read(ts->client, 0x86, buf, 128, DEFAULT_RETRY_CNT);
            touch_monitor_stop_flag-- ;
        } else {
            ret = i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size, DEFAULT_RETRY_CNT);
        }
    }
    if (ret < 0)
#else
    if (i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size, DEFAULT_RETRY_CNT))
#endif
    {
        E("%s: can't read data from chip!\n", __func__);
        goto err_workqueue_out;
    } else {
#ifdef HX_ESD_WORKAROUND
        for (i = 0; i < hx_touch_info_size; i++) {
            if (buf[i] == 0x00) { //case 2 ESD recovery flow-Disable
                check_sum_cal = 1;
            } else if (buf[i] == 0xED) { /*case 1 ESD recovery flow*/
                check_sum_cal = 2;
            } else {
                check_sum_cal = 0;
                i = hx_touch_info_size;
                break;
            }
        }
        //IC status is abnormal ,do hand shaking
#ifdef HX_TP_PROC_DIAG
        diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
        if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0 && diag_cmd == 0)  //ESD Check
#else
        if (check_sum_cal != 0 && diag_cmd == 0)
#endif
#else
#ifdef HX_ESD_WORKAROUND
        if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && HW_RESET_ACTIVATE == 0 )  //ESD Check
#else
        if (check_sum_cal != 0)
#endif
#endif
        {
            ret = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail
            if (ret == 2) {
                goto err_workqueue_out;
            }
            if ((ret == 1) && (check_sum_cal == 1)) {
                I("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
                ESD_HW_REST();
            } else if (check_sum_cal == 2) {
                I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
                ESD_HW_REST();
            }
            //himax_int_enable(ts->client->irq,1,true);
            return;
        } else if (ESD_RESET_ACTIVATE) {
            ESD_RESET_ACTIVATE = 0;/*drop 1st interrupts after chip reset*/
            I("[HIMAX TP MSG]:%s: Back from reset, ready to serve.\n", __func__);
            return;
        } else if (HW_RESET_ACTIVATE)
#else
        if (HW_RESET_ACTIVATE)
#endif
        {
            HW_RESET_ACTIVATE = 0;/*drop 1st interrupts after chip reset*/
            I("[HIMAX TP MSG]:%s: HW_RST Back from reset, ready to serve.\n", __func__);
            return;
        }
        for (loop_i = 0, check_sum_cal = 0; loop_i < hx_touch_info_size; loop_i++)
            check_sum_cal += buf[loop_i];
        if ((check_sum_cal != 0x00)) {
            I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
            return;
        }
    }
    if (ts->debug_log_level & BIT(0)) {
        I("%s: raw data:\n", __func__);
        for (loop_i = 0; loop_i < hx_touch_info_size; loop_i++) {
            I("0x%2.2X ", buf[loop_i]);
            if (loop_i % 8 == 7)
                I("\n");
        }
    }
    //touch monitor raw data fetch
#ifdef HX_TP_PROC_DIAG
    diag_cmd = getDiagCommand();
    if (diag_cmd >= 1 && diag_cmd <= 6) {
        //Check 128th byte CRC
        for (i = hx_touch_info_size, check_sum_cal = 0; i < 128; i++) {
            check_sum_cal += buf[i];
        }
        if (check_sum_cal % 0x100 != 0) {
            goto bypass_checksum_failed_packet;
        }
#ifdef HX_TP_PROC_2T2R
        if (Is_2T2R && diag_cmd == 4) {
            mutual_data = getMutualBuffer_2();
            self_data     = getSelfBuffer();
            // initiallize the block number of mutual and self
            mul_num = getXChannel_2() * getYChannel_2();
            self_num = getXChannel_2() + getYChannel_2();
        } else
#endif
        {
            mutual_data = getMutualBuffer();
            self_data     = getSelfBuffer();
            // initiallize the block number of mutual and self
            mul_num = getXChannel() * getYChannel();
            self_num = getXChannel() + getYChannel();
        }
        //Himax: Check Raw-Data Header
        if (buf[hx_touch_info_size] == buf[hx_touch_info_size + 1] && buf[hx_touch_info_size + 1] == buf[hx_touch_info_size + 2]
            && buf[hx_touch_info_size + 2] == buf[hx_touch_info_size + 3] && buf[hx_touch_info_size] > 0) {
            index = (buf[hx_touch_info_size] - 1) * RawDataLen;
            //I("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
            for (i = 0; i < RawDataLen; i++) {
                temp1 = index + i;
                if (temp1 < mul_num) { //mutual
                    mutual_data[index + i] = buf[i + hx_touch_info_size + 4];  //4: RawData Header
                } else {//self
                    temp1 = i + index;
                    temp2 = self_num + mul_num;
                    if (temp1 >= temp2) {
                        break;
                    }
                    self_data[i + index - mul_num] = buf[i + hx_touch_info_size + 4]; //4: RawData Header
                }
            }
        } else {
            I("[HIMAX TP MSG]%s: header format is wrong!\n", __func__);
        }
    } else if (diag_cmd == 7) {
        memcpy(&(diag_coor[0]), &buf[0], 128);
    }
    //coordinate dump start
    if (coordinate_dump_enable == 1) {
        for (i = 0; i < (15 + (HX_MAX_PT + 5) * 2 * 5); i++) {
            coordinate_char[i] = 0x20;
        }
        coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5] = 0xD;
        coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 1] = 0xA;
    }
    //coordinate dump end
bypass_checksum_failed_packet:
#endif
    EN_NoiseFilter = (buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 3);
    //I("EN_NoiseFilter=%d\n",EN_NoiseFilter);
    EN_NoiseFilter = EN_NoiseFilter & 0x01;
    //I("EN_NoiseFilter2=%d\n",EN_NoiseFilter);
    p_point_num = hx_point_num;
    if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
        hx_point_num = 0;
    else
        hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
    // Touch Point information
    if (hx_point_num != 0 ) {
        uint16_t old_finger = ts->pre_finger_mask;
        finger_num = buf[coordInfoSize - 4] & 0x0F;
        finger_pressed = buf[coordInfoSize - 2] << 8 | buf[coordInfoSize - 3];
        AA_press = 1;
        for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
            if (((finger_pressed >> loop_i) & 1) == 1) {
                base = loop_i * 4;
                x = buf[base] << 8 | buf[base + 1];
                y = (buf[base + 2] << 8 | buf[base + 3]);
                w = buf[(ts->nFinger_support * 4) + loop_i];
                finger_num--;
                if ((ts->debug_log_level & BIT(3)) > 0) {
                    if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1) {
                        if (ts->useScreenRes) {
                            I("status:%X, Screen:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
                              finger_pressed, loop_i + 1, x * ts->widthFactor >> SHIFTBITS,
                              y * ts->heightFactor >> SHIFTBITS, w, EN_NoiseFilter);
                        } else {
                            I("status:%X, Raw:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
                              finger_pressed, loop_i + 1, x, y, w, EN_NoiseFilter);
                        }
                    }
                }
                if (ts->protocol_type == PROTOCOL_TYPE_B) {
                    input_mt_slot(ts->input_dev, loop_i);
                }
                input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
                input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
                input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
                if (point_flag[loop_i] == 0) { //print out touch down coordinate
                    point_flag[loop_i] = 1;
                    I("P%d DOWN, x = %d, y = %d \n", loop_i+1, x, y);
                    haspoint = true;
                }
                if (ts->protocol_type == PROTOCOL_TYPE_A) {
                    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, loop_i);
                    input_mt_sync(ts->input_dev);
                } else {
                    ts->last_slot = loop_i;
                    input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
                }
                if (!ts->first_pressed) {
                    ts->first_pressed = 1;
                    I("S1@%d, %d\n", x, y);
                }
                ts->pre_finger_data[loop_i][0] = x;
                ts->pre_finger_data[loop_i][1] = y;
                if (ts->debug_log_level & BIT(1))
                    I("Finger %d=> X:%d, Y:%d W:%d, Z:%d, F:%d, N:%d\n",
                      loop_i + 1, x, y, w, w, loop_i + 1, EN_NoiseFilter);
            } else {
                if (ts->protocol_type == PROTOCOL_TYPE_B) {
                    input_mt_slot(ts->input_dev, loop_i);
                    input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
                    if (point_flag[loop_i] == 1) {
                        point_flag[loop_i] = 0;
                        I("P%d Up \n", loop_i+1);
                    }
                    if (ts->debug_log_level & BIT(1))
                        I("All Finger leave_Clear_last_event\n");
                }
                if (loop_i == 0 && ts->first_pressed == 1) {
                    ts->first_pressed = 2;
                    I("E1@%d, %d\n",
                      ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
                }
                if ((ts->debug_log_level & BIT(3)) > 0) {
                    if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1) {
                        if (ts->useScreenRes) {
                            I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n",
                              finger_pressed, loop_i + 1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
                              ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
                        } else {
                            I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",
                              finger_pressed, loop_i + 1, ts->pre_finger_data[loop_i][0],
                              ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
                        }
                    }
                }
            }
        }
        ts->pre_finger_mask = finger_pressed;
#ifdef HX_ESD_WORKAROUND
        ESD_COUNTER = 0;
#endif
        input_sync(ts->input_dev);
    } else if (hx_point_num == 0) {
#ifdef HX_PALM_REPORT
        loop_i = 0;
        base = loop_i * 4;
        x = buf[base] << 8 | buf[base + 1];
        y = (buf[base + 2] << 8 | buf[base + 3]);
        w = buf[(ts->nFinger_support * 4) + loop_i];
        if ((!atomic_read(&ts->suspend_mode)) && (x == 0xFA5A) && (y == 0xFA5A) && (w == 0x00)) {
            I(" %s HX_PALM_REPORT KEY power event press\n", __func__);
            input_report_key(ts->input_dev, KEY_SLEEP, 1);
            input_sync(ts->input_dev);
            msleep(100);
            I(" %s HX_PALM_REPORT KEY power event release\n", __func__);
            input_report_key(ts->input_dev, KEY_SLEEP, 0);
            input_sync(ts->input_dev);
            return;
        } else {
            if (!AA_press)
                I(" %s HX_PALM_REPORT_loopi=%d,base=%x,X=%x,Y=%x,W=%x \n", __func__, loop_i, base, x, y, w);
        }
#endif
        if (AA_press) { // leave event
            AA_press = 0;
            if (ts->protocol_type == PROTOCOL_TYPE_A)
                input_mt_sync(ts->input_dev);
            for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
                if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
                    if (ts->protocol_type == PROTOCOL_TYPE_B) {
                        input_mt_slot(ts->input_dev, loop_i);
                        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
                        if (haspoint) {
                            point_flag[loop_i] = 0;
                            I("P%d Up \n",loop_i+1);
                        }
                    }
                }
            }
            if (ts->pre_finger_mask > 0) {
                for (loop_i = 0; loop_i < ts->nFinger_support && (ts->debug_log_level & BIT(3)) > 0; loop_i++) {
                    if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
                        if (ts->useScreenRes) {
                            I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i + 1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
                              ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
                        } else {
                            I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i + 1, ts->pre_finger_data[loop_i][0], ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
                        }
                    }
                }
                ts->pre_finger_mask = 0;
            }
            if (ts->first_pressed == 1) {
                ts->first_pressed = 2;
                I("E1@%d, %d\n", ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
            }
            if (ts->debug_log_level & BIT(1))
                I("All Finger leave\n");
#ifdef HX_TP_PROC_DIAG
            //coordinate dump start
            if (coordinate_dump_enable == 1) {
                do_gettimeofday(&t);
                time_to_tm(t.tv_sec, 0, &broken);
                sprintf(&coordinate_char[0], "%2d:%2d:%2d:%lu,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec / 1000);
                sprintf(&coordinate_char[15], "Touch up!");
                coordinate_fn->f_op->write(coordinate_fn, &coordinate_char[0], 15 + (HX_MAX_PT + 5) * 2 * sizeof(char) * 5 + 2, &coordinate_fn->f_pos);
            }
            //coordinate dump end
#endif
        }
#ifdef HX_ESD_WORKAROUND
        ESD_COUNTER = 0;
#endif
        input_sync(ts->input_dev);
    }
    Last_EN_NoiseFilter = EN_NoiseFilter;
workqueue_out:
    return;
err_workqueue_out:
    I("%s: Now reset the Touch chip.\n", __func__);
#ifdef HX_RST_PIN_FUNC
    himax_HW_reset(true, false);
#endif
    goto workqueue_out;
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
    struct himax_ts_data *ts;
    ts = container_of(timer, struct himax_ts_data, timer);
    queue_work(ts->himax_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static irqreturn_t himax_ts_thread(int irq, void *ptr)
{
    struct himax_ts_data *ts = ptr;
    struct timespec timeStart, timeEnd, timeDelta;
    if (ts->debug_log_level & BIT(2)) {
        getnstimeofday(&timeStart);
        /*I(" Irq start time = %ld.%06ld s\n",
            timeStart.tv_sec, timeStart.tv_nsec/1000);*/
    }
#ifdef HX_SMART_WAKEUP
    if (atomic_read(&ts->suspend_mode) && (!FAKE_POWER_KEY_SEND) && (ts->SMWP_enable)) {
        wake_lock_timeout(&ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
        if (himax_parse_wake_event((struct himax_ts_data *)ptr)) {
            I("[%d] %s Touch DOWN x = %d, y = %d.\n", __LINE__, __func__, 1, 1);
            input_mt_slot(ts->input_dev,0);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 1);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 1);
            input_sync(ts->input_dev);
            input_mt_slot(ts->input_dev,0);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
            input_sync(ts->input_dev);
            FAKE_POWER_KEY_SEND = true;
            return IRQ_HANDLED;
        }
    }
#endif
    himax_ts_work((struct himax_ts_data *)ptr);
    if (ts->debug_log_level & BIT(2)) {
        getnstimeofday(&timeEnd);
        timeDelta.tv_nsec = (timeEnd.tv_sec * 1000000000 + timeEnd.tv_nsec)
                    - (timeStart.tv_sec * 1000000000 + timeStart.tv_nsec);
        /*I("Irq finish time = %ld.%06ld s\n",
            timeEnd.tv_sec, timeEnd.tv_nsec/1000);*/
        I("Touch latency = %ld us\n", timeDelta.tv_nsec / 1000);
    }
    return IRQ_HANDLED;
}

static void himax_ts_work_func(struct work_struct *work)
{
    struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
    himax_ts_work(ts);
}

int himax_ts_register_interrupt(struct i2c_client *client)
{
    struct himax_ts_data *ts = i2c_get_clientdata(client);
    int ret = 0;
    ts->irq_enabled = 0;
    ts->use_irq = 0;
    //Work functon
    if (client->irq) {/*INT mode*/
        ts->use_irq = 1;
        if (HX_INT_IS_EDGE) {
            I("%s edge triiger falling\n ", __func__);
            ret = request_threaded_irq(client->irq, NULL, himax_ts_thread, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
        } else {
            I("%s level trigger low\n ", __func__);
            ret = request_threaded_irq(client->irq, NULL, himax_ts_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ts);
        }
        if (ret == 0) {
            ts->irq_enabled = 1;
            irq_enable_count = 1;
            I("%s: irq register at qpio: %d\n", __func__, client->irq);
#ifdef HX_SMART_WAKEUP
            irq_set_irq_wake(client->irq, 1);
#endif
        } else {
            E("%s: request_irq failed\n", __func__);
        }
    } else {
        I("%s: client->irq is empty, use polling mode.\n", __func__);
    }
    if (!ts->use_irq) {/*if use polling mode need to disable HX_ESD_WORKAROUND function*/
        ts->himax_wq = create_singlethread_workqueue("himax_touch");
        INIT_WORK(&ts->work, himax_ts_work_func);
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = himax_ts_timer_func;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        I("%s: polling mode enabled\n", __func__);
    }
    return ret;
}

#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
    int ret = 0;
    struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
                        work_att.work);
    I(" %s in", __func__);
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts->fb_notif);
    if (ret)
        E(" Unable to register fb_notifier: %d\n", ret);
}
#endif

//=============================================================================================================
//
//    Segment : Himax SYS Debug Function
//
//=============================================================================================================
#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG

static ssize_t himax_debug_level_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct himax_ts_data *ts_data;
    ssize_t ret = 0;
    ts_data = private_ts;
    if (!HX_PROC_SEND_FLAG) {
        ret += sprintf(buf, "%d\n", ts_data->debug_log_level);
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_debug_level_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    struct himax_ts_data *ts;
    int i;
    int len = strlen(buf);
    if (len >= 12) {
        I("%s: no command exceeds 12 chars.\n", __func__);
        return -EFAULT;
    }
    ts = private_ts;
    ts->debug_log_level = 0;
    for (i = 0; i < len - 1; i++) {
        if ( buf[i] >= '0' && buf[i] <= '9' )
            ts->debug_log_level |= (buf[i] - '0');
        else if ( buf[i] >= 'A' && buf[i] <= 'F' )
            ts->debug_log_level |= (buf[i] - 'A' + 10);
        else if ( buf[i] >= 'a' && buf[i] <= 'f' )
            ts->debug_log_level |= (buf[i] - 'a' + 10);
        if (i != len - 2)
            ts->debug_log_level <<= 4;
    }
    if (ts->debug_log_level & BIT(3)) {
        if (ts->pdata->screenWidth > 0 && ts->pdata->screenHeight > 0 &&
            (ts->pdata->abs_x_max - ts->pdata->abs_x_min) > 0 &&
            (ts->pdata->abs_y_max - ts->pdata->abs_y_min) > 0) {
            ts->widthFactor = (ts->pdata->screenWidth << SHIFTBITS) / (ts->pdata->abs_x_max - ts->pdata->abs_x_min);
            ts->heightFactor = (ts->pdata->screenHeight << SHIFTBITS) / (ts->pdata->abs_y_max - ts->pdata->abs_y_min);
            if (ts->widthFactor > 0 && ts->heightFactor > 0)
                ts->useScreenRes = 1;
            else {
                ts->heightFactor = 0;
                ts->widthFactor = 0;
                ts->useScreenRes = 0;
            }
        } else
            I("Enable finger debug with raw position mode!\n");
    } else {
        ts->useScreenRes = 0;
        ts->widthFactor = 0;
        ts->heightFactor = 0;
    }
    return len;
}

static ssize_t himax_vendor_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    struct himax_ts_data *ts_data;
    ts_data = private_ts;
    if (!HX_PROC_SEND_FLAG) {
        ret += sprintf(buf, "%s_FW:%#x,%x_CFG:%#x_SensorId:%#x\n", HIMAX852xes_NAME,
                   ts_data->vendor_fw_ver_H, ts_data->vendor_fw_ver_L, ts_data->vendor_config_ver, ts_data->vendor_sensor_id);
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_attn_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    struct himax_ts_data *ts_data;
    ts_data = private_ts;
    if (!HX_PROC_SEND_FLAG) {
        sprintf(buf, "attn = %x\n", himax_int_gpio_read(ts_data->pdata->gpio_irq));
        ret = strlen(buf) + 1;
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_int_en_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct himax_ts_data *ts = private_ts;
    size_t ret = 0;
    if (!HX_PROC_SEND_FLAG) {
        ret += sprintf(buf + ret, "%d ", ts->irq_enabled);
        ret += sprintf(buf + ret, "\n");
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_int_en_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    struct himax_ts_data *ts = private_ts;
    int value, ret = 0;
    int len = strlen(buf);
    if (len >= 12) {
        I("%s: no command exceeds 12 chars.\n", __func__);
        return -EFAULT;
    }
    if (buf[0] == '0')
        value = false;
    else if (buf[0] == '1')
        value = true;
    else
        return -EINVAL;
    if (value) {
        if (HX_INT_IS_EDGE) {
            ret = request_threaded_irq(ts->client->irq, NULL, himax_ts_thread,
                           IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ts->client->name, ts);
        } else {
            ret = request_threaded_irq(ts->client->irq, NULL, himax_ts_thread,
                           IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts->client->name, ts);
        }
        if (ret == 0) {
            ts->irq_enabled = 1;
            irq_enable_count = 1;
        }
    } else {
        himax_int_enable(ts->client->irq, 0, true);
        free_irq(ts->client->irq, ts);
        ts->irq_enabled = 0;
    }
    return len;
}
static ssize_t himax_layout_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct himax_ts_data *ts = private_ts;
    size_t ret = 0;
    if (!HX_PROC_SEND_FLAG) {
        ret += sprintf(buf + ret, "%d ", ts->pdata->abs_x_min);
        ret += sprintf(buf + ret, "%d ", ts->pdata->abs_x_max);
        ret += sprintf(buf + ret, "%d ", ts->pdata->abs_y_min);
        ret += sprintf(buf + ret, "%d ", ts->pdata->abs_y_max);
        ret += sprintf(buf + ret, "\n");
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_layout_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    struct himax_ts_data *ts = private_ts;
    char buf_tmp[5];
    int i = 0, j = 0, k = 0, ret;
    int len = strlen(buf);
    unsigned long value;
    int layout[4] = {0};
    if (len >= 80) {
        I("%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }
    for (i = 0; i < 20; i++) {
        if (buf[i] == ',' || buf[i] == '\n') {
            memset(buf_tmp, 0x0, sizeof(buf_tmp));
            if (i - j <= 5)
                memcpy(buf_tmp, buf + j, i - j);
            else {
                I("buffer size is over 5 char\n");
                return len;
            }
            j = i + 1;
            if (k < 4) {
                ret = kstrtoul(buf_tmp, 10, &value);
                layout[k++] = value;
            }
        }
    }
    if (k == 4) {
        ts->pdata->abs_x_min = layout[0];
        ts->pdata->abs_x_max = layout[1];
        ts->pdata->abs_y_min = layout[2];
        ts->pdata->abs_y_max = layout[3];
        I("%d, %d, %d, %d\n",
          ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
        input_unregister_device(ts->input_dev);
        himax_input_register(ts);
    } else
        I("ERR@%d, %d, %d, %d\n",
          ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
    return len;
}
#ifdef HX_TP_PROC_RESET
static ssize_t himax_reset_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    int len = strlen(buf);
    if (len >= 12) {
        I("%s: no command exceeds 12 chars.\n", __func__);
        return -EFAULT;
    }
    if (buf[0] == '1')
        himax_HW_reset(true, false);
    //else if (buf_tmp[0] == '2')
    //    himax_HW_reset(true,true);
    //else if (buf_tmp[0] == '3')
    //    himax_HW_reset(false,true);
    //else if (buf_tmp[0] == '4')
    //    ESD_HW_REST();
    return len;
}
#endif

#ifdef HX_TP_PROC_DIAG
static uint8_t *getMutualBuffer(void)
{
    return diag_mutual;
}
static uint8_t *getSelfBuffer(void)
{
    return &diag_self[0];
}
static uint8_t getXChannel(void)
{
    return x_channel;
}
static uint8_t getYChannel(void)
{
    return y_channel;
}
static uint8_t getDiagCommand(void)
{
    return diag_command;
}
static void setXChannel(uint8_t x)
{
    x_channel = x;
}
static void setYChannel(uint8_t y)
{
    y_channel = y;
}
static void setMutualBuffer(void)
{
    diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

#ifdef HX_TP_PROC_2T2R
static uint8_t *getMutualBuffer_2(void)
{
    return diag_mutual_2;
}
static uint8_t getXChannel_2(void)
{
    return x_channel_2;
}
static uint8_t getYChannel_2(void)
{
    return y_channel_2;
}
static void setXChannel_2(uint8_t x)
{
    x_channel_2 = x;
}
static void setYChannel_2(uint8_t y)
{
    y_channel_2 = y;
}
static void setMutualBuffer_2(void)
{
    diag_mutual_2 = kzalloc(x_channel_2 * y_channel_2 * sizeof(uint8_t), GFP_KERNEL);
}
#endif
static void *himax_diag_seq_start(struct seq_file *s, loff_t *pos)
{
    if (*pos >= 1) return NULL;
    return (void *)((unsigned long) * pos + 1);
}

static void *himax_diag_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
    return NULL;
}
static void himax_diag_seq_stop(struct seq_file *s, void *v)
{
}
static int himax_diag_seq_read(struct seq_file *s, void *v)
{
    size_t count = 0;
    uint32_t loop_i;
    uint16_t mutual_num, self_num, width;
#ifdef HX_TP_PROC_2T2R
    if (Is_2T2R && diag_command == 4) {
        mutual_num    = x_channel_2 * y_channel_2;
        self_num    = x_channel_2 + y_channel_2; //don't add KEY_COUNT
        width        = x_channel_2;
        seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel_2, y_channel_2);
    } else
#endif
    {
        mutual_num    = x_channel * y_channel;
        self_num    = x_channel + y_channel; //don't add KEY_COUNT
        width        = x_channel;
        seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
    }
    // start to show out the raw data in adb shell
    if (diag_command >= 1 && diag_command <= 6) {
        if (diag_command <= 3) {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                seq_printf(s, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                    seq_printf(s, " %3d\n", diag_self[width + loop_i / width]);
            }
            seq_printf(s, "\n");
            for (loop_i = 0; loop_i < width; loop_i++) {
                seq_printf(s, "%4d", diag_self[loop_i]);
                if (((loop_i) % width) == (width - 1))
                    seq_printf(s, "\n");
            }
#ifdef HX_TP_PROC_2T2R
        } else if (Is_2T2R && diag_command == 4 ) {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                seq_printf(s, "%4d", diag_mutual_2[loop_i]);
                if ((loop_i % width) == (width - 1))
                    seq_printf(s, " %3d\n", diag_self[width + loop_i / width]);
            }
            seq_printf(s, "\n");
            for (loop_i = 0; loop_i < width; loop_i++) {
                seq_printf(s, "%4d", diag_self[loop_i]);
                if (((loop_i) % width) == (width - 1))
                    seq_printf(s, "\n");
            }
#endif
        } else if (diag_command > 4) {
            for (loop_i = 0; loop_i < self_num; loop_i++) {
                seq_printf(s, "%4d", diag_self[loop_i]);
                if (((loop_i - mutual_num) % width) == (width - 1))
                    seq_printf(s, "\n");
            }
        } else {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                seq_printf(s, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                    seq_printf(s, "\n");
            }
        }
        seq_printf(s, "ChannelEnd");
        seq_printf(s, "\n");
    } else if (diag_command == 7) {
        for (loop_i = 0; loop_i < 128 ; loop_i++) {
            if ((loop_i % 16) == 0)
                seq_printf(s, "LineStart:");
            seq_printf(s, "%4d", diag_coor[loop_i]);
            if ((loop_i % 16) == 15)
                seq_printf(s, "\n");
        }
    }
    return count;
}

static struct seq_operations himax_diag_seq_ops = {
    .start    = himax_diag_seq_start,
    .next    = himax_diag_seq_next,
    .stop    = himax_diag_seq_stop,
    .show    = himax_diag_seq_read,
};
static int himax_diag_proc_open(struct inode *inode, struct file *file)
{
    return seq_open(file, &himax_diag_seq_ops);
};
static ssize_t himax_diag_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
    const uint8_t command_ec_128_raw_flag         = 0x02;
    const uint8_t command_ec_24_normal_flag     = 0x00;
    uint8_t command_ec_128_raw_baseline_flag     = 0x01;
    uint8_t command_ec_128_raw_bank_flag         = 0x03;
    uint8_t command_F1h[2] = {0xF1, 0x00};
    char messages[80] = {0};
    if (len >= 80) {
        I("%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }
    if (copy_from_user(messages, buff, len)) {
        return -EFAULT;
    }
    diag_command = messages[0] - '0';
    I("[Himax]diag_command=0x%x\n", diag_command);
    if (diag_command == 0x01)    {//DC
        command_F1h[1] = command_ec_128_raw_baseline_flag;
        i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
    } else if (diag_command == 0x02) {//IIR
        command_F1h[1] = command_ec_128_raw_flag;
        i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
    } else if (diag_command == 0x03) {    //BANK
        command_F1h[1] = command_ec_128_raw_bank_flag;    //0x03
        i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
    } else if (diag_command == 0x04 ) { // 2T3R IIR
        command_F1h[1] = 0x04; //2T3R IIR
        i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
    } else if (diag_command == 0x00) {//Disable
        command_F1h[1] = command_ec_24_normal_flag;
        i2c_himax_write(private_ts->client, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
        touch_monitor_stop_flag = touch_monitor_stop_limit;
    }
    //coordinate dump start
    else if (diag_command == 0x08)    {
        coordinate_fn = filp_open(DIAG_COORDINATE_FILE, O_CREAT | O_WRONLY | O_APPEND | O_TRUNC, 0666);
        if (IS_ERR(coordinate_fn)) {
            E("%s: coordinate_dump_file_create error\n", __func__);
            coordinate_dump_enable = 0;
            filp_close(coordinate_fn, NULL);
        }
        coordinate_dump_enable = 1;
    } else if (diag_command == 0x09) {
        coordinate_dump_enable = 0;
        if (!IS_ERR(coordinate_fn)) {
            filp_close(coordinate_fn, NULL);
        }
    }
    //coordinate dump end
    else {
        E("[Himax]Diag command error!diag_command=0x%x\n", diag_command);
    }
    return len;
}

static struct file_operations himax_proc_diag_ops = {
    .owner = THIS_MODULE,
    .open = himax_diag_proc_open,
    .read = seq_read,
    .write = himax_diag_write,
};
#endif

#ifdef HX_TP_PROC_REGISTER
static ssize_t himax_register_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int base = 0;
    uint16_t loop_i, loop_j;
    uint8_t inData[128];
    uint8_t outData[5];
    memset(outData, 0x00, sizeof(outData));
    memset(inData, 0x00, sizeof(inData));
    I("Himax multi_register_command = %d \n", multi_register_command);
    if (!HX_PROC_SEND_FLAG) {
        if (multi_register_command == 1) {
            base = 0;
            for (loop_i = 0; loop_i < 6; loop_i++) {
                if (multi_register[loop_i] != 0x00) {
                    if (multi_cfg_bank[loop_i] == 1) {//config bank register
                        outData[0] = 0x14;
                        i2c_himax_write(private_ts->client, 0x8C , &outData[0], 1, DEFAULT_RETRY_CNT);
                        msleep(10);
                        outData[0] = 0x00;
                        outData[1] = multi_register[loop_i];
                        i2c_himax_write(private_ts->client, 0x8B , &outData[0], 2, DEFAULT_RETRY_CNT);
                        msleep(10);
                        i2c_himax_read(private_ts->client, 0x5A, inData, 128, DEFAULT_RETRY_CNT);
                        outData[0] = 0x00;
                        i2c_himax_write(private_ts->client, 0x8C , &outData[0], 1, DEFAULT_RETRY_CNT);
                        for (loop_j = 0; loop_j < 128; loop_j++)
                            multi_value[base++] = inData[loop_j];
                    } else {//normal register
                        i2c_himax_read(private_ts->client, multi_register[loop_i], inData, 128, DEFAULT_RETRY_CNT);
                        for (loop_j = 0; loop_j < 128; loop_j++)
                            multi_value[base++] = inData[loop_j];
                    }
                }
            }
            base = 0;
            for (loop_i = 0; loop_i < 6; loop_i++) {
                if (multi_register[loop_i] != 0x00) {
                    if (multi_cfg_bank[loop_i] == 1)
                        ret += sprintf(buf + ret, "Register: FE(%x)\n", multi_register[loop_i]);
                    else
                        ret += sprintf(buf + ret, "Register: %x\n", multi_register[loop_i]);
                    for (loop_j = 0; loop_j < 128; loop_j++) {
                        ret += sprintf(buf + ret, "0x%2.2X ", multi_value[base++]);
                        if ((loop_j % 16) == 15)
                            ret += sprintf(buf + ret, "\n");
                    }
                }
            }
            return ret;
        }
        if (config_bank_reg) {
            I("%s: register_command = FE(%x)\n", __func__, register_command);
            //Config bank register read flow.
            outData[0] = 0x14;
            i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
            msleep(10);
            outData[0] = 0x00;
            outData[1] = register_command;
            i2c_himax_write(private_ts->client, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);
            msleep(10);
            i2c_himax_read(private_ts->client, 0x5A, inData, 128, DEFAULT_RETRY_CNT);
            msleep(10);
            outData[0] = 0x00;
            i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
        } else {
            if (i2c_himax_read(private_ts->client, register_command, inData, 128, DEFAULT_RETRY_CNT) < 0)
                return ret;
        }
        if (config_bank_reg)
            ret += sprintf(buf, "command: FE(%x)\n", register_command);
        else
            ret += sprintf(buf, "command: %x\n", register_command);
        for (loop_i = 0; loop_i < 128; loop_i++) {
            ret += sprintf(buf + ret, "0x%2.2X ", inData[loop_i]);
            if ((loop_i % 16) == 15)
                ret += sprintf(buf + ret, "\n");
        }
        ret += sprintf(buf + ret, "\n");
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_register_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    char buf_tmp[6], length = 0;
    unsigned long result    = 0;
    uint8_t loop_i          = 0;
    uint16_t base           = 5;
    uint8_t write_da[128];
    uint8_t outData[5];
    int len = strlen(buf);
    if (len >= 80) {
        I("%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }
    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));
    memset(outData, 0x0, sizeof(outData));
    I("himax %s \n", buf);
    if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
        memset(multi_register, 0x00, sizeof(multi_register));
        memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
        memset(multi_value, 0x00, sizeof(multi_value));
        I("himax multi register enter\n");
        multi_register_command = 1;
        base     = 2;
        loop_i     = 0;
        while (true) {
            if (buf[base] == '\n')
                break;
            if (loop_i >= 6 )
                break;
            if (buf[base] == ':' && buf[base + 1] == 'x' && buf[base + 2] == 'F' &&
                buf[base + 3] == 'E' && buf[base + 4] != ':') {
                memcpy(buf_tmp, buf + base + 4, 2);
                if (!kstrtoul(buf_tmp, 16, &result)) {
                    multi_register[loop_i] = result;
                    multi_cfg_bank[loop_i++] = 1;
                }
                base += 6;
            } else {
                memcpy(buf_tmp, buf + base + 2, 2);
                if (!kstrtoul(buf_tmp, 16, &result)) {
                    multi_register[loop_i] = result;
                    multi_cfg_bank[loop_i++] = 0;
                }
                base += 4;
            }
        }
        I("========================== \n");
        for (loop_i = 0; loop_i < 6; loop_i++)
            I("%d,%d:", multi_register[loop_i], multi_cfg_bank[loop_i]);
        I("\n");
    } else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
        multi_register_command = 0;
        if (buf[2] == 'x') {
            if (buf[3] == 'F' && buf[4] == 'E') {//Config bank register
                config_bank_reg = true;
                memcpy(buf_tmp, buf + 5, 2);
                if (!kstrtoul(buf_tmp, 16, &result))
                    register_command = result;
                base = 7;
                I("CMD: FE(%x)\n", register_command);
            } else {
                config_bank_reg = false;
                memcpy(buf_tmp, buf + 3, 2);
                if (!kstrtoul(buf_tmp, 16, &result))
                    register_command = result;
                base = 5;
                I("CMD: %x\n", register_command);
            }
            for (loop_i = 0; loop_i < 128; loop_i++) {
                if (buf[base] == '\n') {
                    if (buf[0] == 'w') {
                        if (config_bank_reg) {
                            outData[0] = 0x14;
                            i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
                            msleep(10);
                            outData[0] = 0x00;
                            outData[1] = register_command;
                            i2c_himax_write(private_ts->client, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);
                            msleep(10);
                            i2c_himax_write(private_ts->client, 0x40, &write_da[0], length, DEFAULT_RETRY_CNT);
                            msleep(10);
                            outData[0] = 0x00;
                            i2c_himax_write(private_ts->client, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
                            I("CMD: FE(%x), %x, %d\n", register_command, write_da[0], length);
                        } else {
                            i2c_himax_write(private_ts->client, register_command, &write_da[0], length, DEFAULT_RETRY_CNT);
                            I("CMD: %x, %x, %d\n", register_command, write_da[0], length);
                        }
                    }
                    I("\n");
                    return len;
                }
                if (buf[base + 1] == 'x') {
                    buf_tmp[4] = '\n';
                    buf_tmp[5] = '\0';
                    memcpy(buf_tmp, buf + base + 2, 2);
                    if (!kstrtoul(buf_tmp, 16, &result)) {
                        write_da[loop_i] = result;
                    }
                    length++;
                }
                base += 4;
            }
        }
    }
    return len;
}
#endif

#ifdef HX_TP_PROC_DEBUG
static ssize_t himax_debug_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    size_t ret = 0;
    if (!HX_PROC_SEND_FLAG) {
        if (debug_level_cmd == 't') {
            if (fw_update_complete) {
                ret += sprintf(buf, "FW Update Complete ");
            } else {
                ret += sprintf(buf, "FW Update Fail ");
            }
        } else if (debug_level_cmd == 'h') {
            if (handshaking_result == 0) {
                ret += sprintf(buf, "Handshaking Result = %d (MCU Running)\n", handshaking_result);
            } else if (handshaking_result == 1) {
                ret += sprintf(buf, "Handshaking Result = %d (MCU Stop)\n", handshaking_result);
            } else if (handshaking_result == 2) {
                ret += sprintf(buf, "Handshaking Result = %d (I2C Error)\n", handshaking_result);
            } else {
                ret += sprintf(buf, "Handshaking Result = error \n");
            }
        } else if (debug_level_cmd == 'v') {
            ret += sprintf(buf + ret, "FW_VER = ");
            ret += sprintf(buf + ret, "0x%2.2X, %2.2X \n", private_ts->vendor_fw_ver_H, private_ts->vendor_fw_ver_L);
            ret += sprintf(buf + ret, "CONFIG_VER = ");
            ret += sprintf(buf + ret, "0x%2.2X \n", private_ts->vendor_config_ver);
            ret += sprintf(buf + ret, "\n");
        } else if (debug_level_cmd == 'd') {
            ret += sprintf(buf + ret, "Himax Touch IC Information :\n");
            if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
                ret += sprintf(buf + ret, "IC Type : D\n");
            } else if (IC_TYPE == HX_85XX_E_SERIES_PWON) {
                ret += sprintf(buf + ret, "IC Type : E\n");
            } else if (IC_TYPE == HX_85XX_ES_SERIES_PWON) {
                ret += sprintf(buf + ret, "IC Type : ES\n");
            } else {
                ret += sprintf(buf + ret, "IC Type error.\n");
            }
            if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW) {
                ret += sprintf(buf + ret, "IC Checksum : SW\n");
            } else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW) {
                ret += sprintf(buf + ret, "IC Checksum : HW\n");
            } else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC) {
                ret += sprintf(buf + ret, "IC Checksum : CRC\n");
            } else {
                ret += sprintf(buf + ret, "IC Checksum error.\n");
            }
            if (HX_INT_IS_EDGE) {
                ret += sprintf(buf + ret, "Interrupt : EDGE TIRGGER\n");
            } else {
                ret += sprintf(buf + ret, "Interrupt : LEVEL TRIGGER\n");
            }
            ret += sprintf(buf + ret, "RX Num : %d\n", HX_RX_NUM);
            ret += sprintf(buf + ret, "TX Num : %d\n", HX_TX_NUM);
            ret += sprintf(buf + ret, "BT Num : %d\n", HX_BT_NUM);
            ret += sprintf(buf + ret, "X Resolution : %d\n", HX_X_RES);
            ret += sprintf(buf + ret, "Y Resolution : %d\n", HX_Y_RES);
            ret += sprintf(buf + ret, "Max Point : %d\n", HX_MAX_PT);
#ifdef HX_TP_PROC_2T2R
            if (Is_2T2R) {
                ret += sprintf(buf + ret, "2T2R panel\n");
                ret += sprintf(buf + ret, "RX Num_2 : %d\n", HX_RX_NUM_2);
                ret += sprintf(buf + ret, "TX Num_2 : %d\n", HX_TX_NUM_2);
            }
#endif
        } else if (debug_level_cmd == 'i') {
            ret += sprintf(buf + ret, "Himax Touch Driver Version:\n");
            ret += sprintf(buf + ret, "%s \n", HIMAX_DRIVER_VER);
        }
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_debug_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    struct file *hx_filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    int len = strlen(buf);
    char fileName[128];
    if (len >= 80) {
        I("%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }
    if ( buf[0] == 'h') { //handshaking
        debug_level_cmd = buf[0];
        himax_int_enable(private_ts->client->irq, 0, true);
        handshaking_result = himax_hand_shaking(); //0:Running, 1:Stop, 2:I2C Fail
        himax_int_enable(private_ts->client->irq, 1, true);
        return len;
    } else if ( buf[0] == 'v') { //firmware version
        debug_level_cmd = buf[0];
        himax_read_FW_ver(true);
        return len;
    } else if ( buf[0] == 'd') { //test
        debug_level_cmd = buf[0];
        return len;
    } else if ( buf[0] == 'i') { //driver version
        debug_level_cmd = buf[0];
        return len;
    } else if (buf[0] == 't') {
        himax_int_enable(private_ts->client->irq, 0, true);
        wake_lock(&private_ts->ts_flash_wake_lock);
#ifdef HX_CHIP_STATUS_MONITOR
        HX_CHIP_POLLING_COUNT = 0;
        cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif
        debug_level_cmd         = buf[0];
        fw_update_complete        = false;
        memset(fileName, 0, 128);
        // parse the file name
        snprintf(fileName, len - 2, "%s", &buf[2]);
        I("%s: upgrade from file(%s) start!\n", __func__, fileName);
        // open file
        hx_filp = filp_open(fileName, O_RDONLY, 0);
        if (IS_ERR(hx_filp)) {
            E("%s: open firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return len;
        }
        oldfs = get_fs();
        set_fs(get_ds());
        // read the latest firmware binary file
        result = hx_filp->f_op->read(hx_filp, upgrade_fw, sizeof(upgrade_fw), &hx_filp->f_pos);
        if (result < 0) {
            E("%s: read firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return len;
        }
        set_fs(oldfs);
        filp_close(hx_filp, NULL);
        I("%s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
        if (result > 0) {
            // start to upgrade
#ifdef HX_RST_PIN_FUNC
            himax_HW_reset(false, true);
#endif
            if (fts_ctpm_fw_upgrade_with_fs(upgrade_fw, result, true) == 0) {
                E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
                fw_update_complete = false;
            } else {
                I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
                fw_update_complete = true;
            }
            goto firmware_upgrade_done;
            //return len;
        }
    }
firmware_upgrade_done:
#ifdef HX_RST_PIN_FUNC
    himax_HW_reset(true, true);
#endif
    wake_unlock(&private_ts->ts_flash_wake_lock);
    himax_int_enable(private_ts->client->irq, 1, true);
#ifdef HX_CHIP_STATUS_MONITOR
    HX_CHIP_POLLING_COUNT = 0;
    queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES * HZ);
#endif
    //todo himax_chip->tp_firmware_upgrade_proceed = 0;
    //todo himax_chip->suspend_state = 0;
    //todo enable_irq(himax_chip->irq);
    return len;
}
#endif

#ifdef HX_TP_PROC_FLASH_DUMP

static uint8_t getFlashCommand(void)
{
    return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
    return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
    return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
    return flash_dump_fail;
}

static uint8_t getSysOperation(void)
{
    return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
    return flash_read_step;
}

static uint8_t getFlashDumpSector(void)
{
    return flash_dump_sector;
}

static uint8_t getFlashDumpPage(void)
{
    return flash_dump_page;
}

static bool getFlashDumpGoing(void)
{
    return flash_dump_going;
}

static void setFlashBuffer(void)
{
    //int i=0;
    flash_buffer = kzalloc(FLASH_SIZE * sizeof(uint8_t), GFP_KERNEL);
    memset(flash_buffer, 0x00, FLASH_SIZE);
    //for(i=0; i<FLASH_SIZE; i++)
    //{
    //    flash_buffer[i] = 0x00;
    //}
}

static void setSysOperation(uint8_t operation)
{
    sys_operation = operation;
}

static void setFlashDumpProgress(uint8_t progress)
{
    flash_progress = progress;
    //I("setFlashDumpProgress : progress = %d ,flash_progress = %d \n",progress,flash_progress);
}

static void setFlashDumpComplete(uint8_t status)
{
    flash_dump_complete = status;
}

static void setFlashDumpFail(uint8_t fail)
{
    flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
    flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
    flash_read_step = step;
}

static void setFlashDumpSector(uint8_t sector)
{
    flash_dump_sector = sector;
}

static void setFlashDumpPage(uint8_t page)
{
    flash_dump_page = page;
}

static void setFlashDumpGoing(bool going)
{
    flash_dump_going = going;
}

static void himax_ts_flash_work_func(struct work_struct *work)
{
    struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);
    uint8_t page_tmp[128];
    uint8_t x59_tmp[4] = {0, 0, 0, 0};
    int i = 0, j = 0, k = 0, l = 0,/* j_limit = 0,*/ buffer_ptr = 0/*, flash_end_count = 0*/;
    uint8_t local_flash_command = 0;
    uint8_t sector = 0;
    uint8_t page = 0;
    /*uint8_t xAA_command[2] = {0xAA,0x00};*/
    uint8_t x81_command[2] = {HX_CMD_TSSLPOUT, 0x00};
    uint8_t x82_command[2] = {HX_CMD_TSSOFF, 0x00};
    uint8_t x35_command[2] = {HX_CMD_SETMICROOFF, 0x00};
    uint8_t x43_command[4] = {HX_CMD_FLASH_ENABLE, 0x00, 0x00, 0x00};
    uint8_t x44_command[4] = {HX_CMD_FLASH_SET_ADDRESS, 0x00, 0x00, 0x00};
    uint8_t x45_command[5] = {HX_CMD_FLASH_WRITE_REGISTER, 0x00, 0x00, 0x00, 0x00};
    uint8_t x46_command[2] = {0x46, 0x00};
    /*uint8_t x4A_command[2] = {HX_CMD_4A,0x00};*/
    uint8_t x4D_command[2] = {0x4D, 0x00};
    /*uint8_t x59_command[2] = {0x59,0x00};*/
    himax_int_enable(private_ts->client->irq, 0, true);
#ifdef HX_CHIP_STATUS_MONITOR
    HX_CHIP_POLLING_COUNT = 0;
    cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif
    setFlashDumpGoing(true);
    sector = getFlashDumpSector();
    page = getFlashDumpPage();
    local_flash_command = getFlashCommand();
#ifdef HX_RST_PIN_FUNC
    if (local_flash_command < 0x0F)
        himax_HW_reset(false, true);
#endif
    //if( i2c_himax_master_write(ts->client, xAA_command, 1, 3) < 0 )//sleep out
    //{
    //    E("%s i2c write AA fail.\n",__func__);
    //    goto Flash_Dump_i2c_transfer_error;
    //}
    //msleep(120);
    if ( i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0 ) { //sleep out
        E("%s i2c write 81 fail.\n", __func__);
        goto Flash_Dump_i2c_transfer_error;
    }
    msleep(120);
    if ( i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0 ) {
        E("%s i2c write 82 fail.\n", __func__);
        goto Flash_Dump_i2c_transfer_error;
    }
    msleep(100);
    I("%s: local_flash_command = %d enter.\n", __func__, local_flash_command);
    if ((local_flash_command == 1 || local_flash_command == 2) || (local_flash_command == 0x0F)) {
        x43_command[1] = 0x01;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0) {
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(100);
        for ( i = 0 ; i < 8 ; i++) {
            for (j = 0 ; j < 32 ; j++) {
                //I(" Step 2 i=%d , j=%d %s\n",i,j,__func__);
                //read page start
                for (k = 0; k < 128; k++) {
                    page_tmp[k] = 0x00;
                }
                for (k = 0; k < 32; k++) {
                    x44_command[1] = k;
                    x44_command[2] = j;
                    x44_command[3] = i;
                    if ( i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 ) {
                        E("%s i2c write 44 fail.\n", __func__);
                        goto Flash_Dump_i2c_transfer_error;
                    }
                    if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0) {
                        E("%s i2c write 46 fail.\n", __func__);
                        goto Flash_Dump_i2c_transfer_error;
                    }
                    //msleep(2);
                    if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0) {
                        E("%s i2c write 59 fail.\n", __func__);
                        goto Flash_Dump_i2c_transfer_error;
                    }
                    //msleep(2);
                    for (l = 0; l < 4; l++) {
                        page_tmp[k * 4 + l] = x59_tmp[l];
                    }
                    //msleep(10);
                }
                //read page end
                for (k = 0; k < 128; k++) {
                    flash_buffer[buffer_ptr++] = page_tmp[k];
                }
                setFlashDumpProgress(i * 32 + j);
            }
        }
    } else if (local_flash_command == 3) {
        x43_command[1] = 0x01;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(100);
        for (i = 0; i < 128; i++) {
            page_tmp[i] = 0x00;
        }
        for (i = 0; i < 32; i++) {
            x44_command[1] = i;
            x44_command[2] = page;
            x44_command[3] = sector;
            if ( i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 ) {
                E("%s i2c write 44 fail.\n", __func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0 ) {
                E("%s i2c write 46 fail.\n", __func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            //msleep(2);
            if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0 ) {
                E("%s i2c write 59 fail.\n", __func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            //msleep(2);
            for (j = 0; j < 4; j++) {
                page_tmp[i * 4 + j] = x59_tmp[j];
            }
            //msleep(10);
        }
        //read page end
        for (i = 0; i < 128; i++) {
            flash_buffer[buffer_ptr++] = page_tmp[i];
        }
    } else if (local_flash_command == 4) {
        //page write flow.
        //I("%s: local_flash_command = 4, enter.\n", __func__);
        // unlock flash
        himax_lock_flash(0);
        msleep(50);
        // page erase
        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        x43_command[3] = 0x02;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        x44_command[1] = 0x00;
        x44_command[2] = page;
        x44_command[3] = sector;
        if ( i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 44 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        if ( i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 4D fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(100);
        // enter manual mode
        x35_command[1] = 0x01;
        if ( i2c_himax_write(ts->client, x35_command[0], &x35_command[1], 1, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 35 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(100);
        // flash enable
        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        // set flash address
        x44_command[1] = 0x00;
        x44_command[2] = page;
        x44_command[3] = sector;
        if ( i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 44 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        // manual mode command : 47 to latch the flash address when page address change.
        x43_command[1] = 0x01;
        x43_command[2] = 0x09;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        x43_command[1] = 0x01;
        x43_command[2] = 0x0D;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        x43_command[1] = 0x01;
        x43_command[2] = 0x09;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        for (i = 0; i < 32; i++) {
            I("himax :i=%d \n", i);
            x44_command[1] = i;
            x44_command[2] = page;
            x44_command[3] = sector;
            if ( i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 ) {
                E("%s i2c write 44 fail.\n", __func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);
            x45_command[1] = flash_buffer[i * 4 + 0];
            x45_command[2] = flash_buffer[i * 4 + 1];
            x45_command[3] = flash_buffer[i * 4 + 2];
            x45_command[4] = flash_buffer[i * 4 + 3];
            if ( i2c_himax_write(ts->client, x45_command[0], &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 ) {
                E("%s i2c write 45 fail.\n", __func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);
            // manual mode command : 48 ,data will be written into flash buffer
            x43_command[1] = 0x01;
            x43_command[2] = 0x0D;
            if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
                E("%s i2c write 43 fail.\n", __func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);
            x43_command[1] = 0x01;
            x43_command[2] = 0x09;
            if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
                E("%s i2c write 43 fail.\n", __func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);
        }
        // manual mode command : 49 ,program data from flash buffer to this page
        x43_command[1] = 0x01;
        x43_command[2] = 0x01;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        x43_command[1] = 0x01;
        x43_command[2] = 0x05;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        x43_command[1] = 0x01;
        x43_command[2] = 0x01;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        // flash disable
        x43_command[1] = 0x00;
        if ( i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 43 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        // leave manual mode
        x35_command[1] = 0x01;
        if ( i2c_himax_write(ts->client, x35_command[0], &x35_command[1], 1, DEFAULT_RETRY_CNT) < 0 ) {
            E("%s i2c write 35 fail.\n", __func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);
        // lock flash
        himax_lock_flash(1);
        msleep(50);
        buffer_ptr = 128;
        I("Himax: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
    }
    I("Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
    if (local_flash_command == 0x01) {
        I(" buffer_ptr = %d \n", buffer_ptr);
        for (i = 0; i < buffer_ptr; i++) {
            I("%2.2X ", flash_buffer[i]);
            if ((i % 16) == 15) {
                I("\n");
            }
        }
        I("End~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    }
    i2c_himax_master_write(ts->client, x43_command, 1, 3);
    msleep(50);
    if (local_flash_command == 2) {
        struct file *fn;
        fn = filp_open(FLASH_DUMP_FILE, O_CREAT | O_WRONLY , 0);
        if (!IS_ERR(fn)) {
            fn->f_op->write(fn, flash_buffer, buffer_ptr * sizeof(uint8_t), &fn->f_pos);
            filp_close(fn, NULL);
        }
    }
#ifdef HX_RST_PIN_FUNC
    if (local_flash_command < 0x0F)
        himax_HW_reset(true, true);
#endif
    himax_int_enable(private_ts->client->irq, 1, true);
#ifdef HX_CHIP_STATUS_MONITOR
    HX_CHIP_POLLING_COUNT = 0;
    queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES * HZ);
#endif
    setFlashDumpGoing(false);
    setFlashDumpComplete(1);
    setSysOperation(0);
    return;
Flash_Dump_i2c_transfer_error:
#ifdef HX_RST_PIN_FUNC
    himax_HW_reset(true, true);
#endif
    himax_int_enable(private_ts->client->irq, 1, true);
#ifdef HX_CHIP_STATUS_MONITOR
    HX_CHIP_POLLING_COUNT = 0;
    queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, HX_POLLING_TIMES * HZ);
#endif
    setFlashDumpGoing(false);
    setFlashDumpComplete(0);
    setFlashDumpFail(1);
    setSysOperation(0);
    return;
}

static ssize_t himax_flash_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int loop_i;
    uint8_t local_flash_read_step = 0;
    uint8_t local_flash_complete = 0;
    uint8_t local_flash_progress = 0;
    uint8_t local_flash_command = 0;
    uint8_t local_flash_fail = 0;
    local_flash_complete = getFlashDumpComplete();
    local_flash_progress = getFlashDumpProgress();
    local_flash_command = getFlashCommand();
    local_flash_fail = getFlashDumpFail();
    I("flash_progress = %d \n", local_flash_progress);
    if (!HX_PROC_SEND_FLAG) {
        if (local_flash_fail) {
            ret += sprintf(buf + ret, "FlashStart:Fail \n");
            ret += sprintf(buf + ret, "FlashEnd");
            ret += sprintf(buf + ret, "\n");
            HX_PROC_SEND_FLAG = 1;
            return ret;
        }
        if (!local_flash_complete) {
            ret += sprintf(buf + ret, "FlashStart:Ongoing:0x%2.2x \n", flash_progress);
            ret += sprintf(buf + ret, "FlashEnd");
            ret += sprintf(buf + ret, "\n");
            HX_PROC_SEND_FLAG = 1;
            return ret;
        }
        if (local_flash_command == 1 && local_flash_complete) {
            ret += sprintf(buf + ret, "FlashStart:Complete \n");
            ret += sprintf(buf + ret, "FlashEnd");
            ret += sprintf(buf + ret, "\n");
            HX_PROC_SEND_FLAG = 1;
            return ret;
        }
        if (local_flash_command == 3 && local_flash_complete) {
            ret += sprintf(buf + ret, "FlashStart: \n");
            for (loop_i = 0; loop_i < 128; loop_i++) {
                ret += sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
                if ((loop_i % 16) == 15) {
                    ret += sprintf(buf + ret, "\n");
                }
            }
            ret += sprintf(buf + ret, "FlashEnd");
            ret += sprintf(buf + ret, "\n");
            HX_PROC_SEND_FLAG = 1;
            return ret;
        }
        //flash command == 0 , report the data
        local_flash_read_step = getFlashReadStep();
        ret += sprintf(buf + ret, "FlashStart:%2.2x \n", local_flash_read_step);
        for (loop_i = 0; loop_i < 1024; loop_i++) {
            ret += sprintf(buf + ret, "x%2.2X", flash_buffer[local_flash_read_step * 1024 + loop_i]);
            if ((loop_i % 16) == 15) {
                ret += sprintf(buf + ret, "\n");
            }
        }
        ret += sprintf(buf + ret, "FlashEnd");
        ret += sprintf(buf + ret, "\n");
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_flash_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    char buf_tmp[6];
    unsigned long result = 0;
    uint8_t loop_i = 0;
    int base = 0;
    int len = strlen(buf);
    if (len >= 80) {
        I("%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }
    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    I("%s: buf[0] = %s\n", __func__, buf);
    if (getSysOperation() == 1) {
        E("%s: SYS is busy , return!\n", __func__);
        return len;
    }
    if (buf[0] == '0') {
        setFlashCommand(0);
        if (buf[1] == ':' && buf[2] == 'x') {
            memcpy(buf_tmp, buf + 3, 2);
            I("%s: read_Step = %s\n", __func__, buf_tmp);
            if (!kstrtoul(buf_tmp, 16, &result)) {
                I("%s: read_Step = %lu \n", __func__, result);
                setFlashReadStep(result);
            }
        }
    } else if (buf[0] == '1') {
        setSysOperation(1);
        setFlashCommand(1);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);
        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    } else if (buf[0] == '2') {
        setSysOperation(1);
        setFlashCommand(2);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);
        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    } else if (buf[0] == '3') {
        setSysOperation(1);
        setFlashCommand(3);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);
        memcpy(buf_tmp, buf + 3, 2);
        if (!kstrtoul(buf_tmp, 16, &result)) {
            setFlashDumpSector(result);
        }
        memcpy(buf_tmp, buf + 7, 2);
        if (!kstrtoul(buf_tmp, 16, &result)) {
            setFlashDumpPage(result);
        }
        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    } else if (buf[0] == '4') {
        I("%s: command 4 enter.\n", __func__);
        setSysOperation(1);
        setFlashCommand(4);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);
        memcpy(buf_tmp, buf + 3, 2);
        if (!kstrtoul(buf_tmp, 16, &result)) {
            setFlashDumpSector(result);
        } else {
            E("%s: command 4 , sector error.\n", __func__);
            return len;
        }
        memcpy(buf_tmp, buf + 7, 2);
        if (!kstrtoul(buf_tmp, 16, &result)) {
            setFlashDumpPage(result);
        } else {
            E("%s: command 4 , page error.\n", __func__);
            return len;
        }
        base = 11;
        I("=========Himax flash page buffer start=========\n");
        for (loop_i = 0; loop_i < 128; loop_i++) {
            memcpy(buf_tmp, buf + base, 2);
            if (!kstrtoul(buf_tmp, 16, &result)) {
                flash_buffer[loop_i] = result;
                I("%d ", flash_buffer[loop_i]);
                if (loop_i % 16 == 15) {
                    I("\n");
                }
            }
            base += 3;
        }
        I("=========Himax flash page buffer end=========\n");
        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    }
    return len;
}
#endif

#ifdef HX_TP_PROC_SELF_TEST
static int himax_chip_self_test(void)
{
    uint8_t cmdbuf[11];
    uint8_t valuebuf[16];
    int i = 0, pf_value = 0x00;
    memset(cmdbuf, 0x00, sizeof(cmdbuf));
    memset(valuebuf, 0x00, sizeof(valuebuf));
    cmdbuf[0] = 0x06;
    i2c_himax_write(private_ts->client, 0xF1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(120);
    i2c_himax_write(private_ts->client, HX_CMD_TSSON, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
    msleep(120);
    i2c_himax_write(private_ts->client, HX_CMD_TSSLPOUT, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
    msleep(2000);
    i2c_himax_write(private_ts->client, HX_CMD_TSSOFF, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
    msleep(120);
    i2c_himax_write(private_ts->client, HX_CMD_TSSLPIN, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
    msleep(120);
    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xF1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(120);
    i2c_himax_read(private_ts->client, 0xB1, valuebuf, 8, DEFAULT_RETRY_CNT);
    msleep(10);
    for (i = 0; i < 8; i++) {
        I("[Himax]: After slf test 0xB1 buff_back[%d] = 0x%x\n", i, valuebuf[i]);
    }
    msleep(30);
    if (valuebuf[0] == 0xAA) {
        I("[Himax]: self-test pass\n");
        pf_value = 0x0;
    } else {
        E("[Himax]: self-test fail\n");
        pf_value = 0x1;
    }
    return pf_value;
}

static ssize_t himax_self_test_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int val = 0x00;
    int ret = 0;
    val = himax_chip_self_test();
    if (!HX_PROC_SEND_FLAG) {
        if (val == 0x00) {
            ret += sprintf(buf + ret, "Self_Test Pass\n");
        } else {
            ret += sprintf(buf + ret, "Self_Test Fail\n");
        }
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}
#endif

#ifdef HX_TP_PROC_HITOUCH
static ssize_t himax_hitouch_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int ret = 0;
    if (!HX_PROC_SEND_FLAG) {
        if (hitouch_command == 0) {
            ret += sprintf(buf + ret, "Himax Touch Driver Version:\n");
            ret += sprintf(buf + ret, "%s \n", HIMAX_DRIVER_VER);
        } else if (hitouch_command == 1) {
            ret += sprintf(buf + ret, "hitouch_is_connect = true\n");
        } else if (hitouch_command == 2) {
            ret += sprintf(buf + ret, "hitouch_is_connect = false\n");
        }
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

//-----------------------------------------------------------------------------------
//himax_hitouch_store
//command 0 : Get Driver Version
//command 1 : Hitouch Connect
//command 2 : Hitouch Disconnect
//-----------------------------------------------------------------------------------
static ssize_t himax_hitouch_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    int len = strlen(buf);
    if (len >= 80) {
        I("%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }
    if (buf[0] == '0') {
        hitouch_command = 0;
    } else if (buf[0] == '1') {
        hitouch_command = 1;
        hitouch_is_connect = true;
        I("hitouch_is_connect = true\n");
    } else if (buf[0] == '2') {
        hitouch_command = 2;
        hitouch_is_connect = false;
        I("hitouch_is_connect = false\n");
    }
    return len;
}
#endif

#ifdef HX_SMART_WAKEUP
static ssize_t himax_SMWP_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct himax_ts_data *ts = private_ts;
    size_t ret = 0;
    if (!HX_PROC_SEND_FLAG) {
        ret = snprintf(buf, PAGE_SIZE, "%d\n", ts->SMWP_enable);
        HX_PROC_SEND_FLAG = 1;
    } else
        HX_PROC_SEND_FLAG = 0;
    return ret;
}

static ssize_t himax_SMWP_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    int len = strlen(buf);
    struct himax_ts_data *ts = private_ts;
    if (len >= 80) {
        I("%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }
    if (buf[0] == '0')
        ts->SMWP_enable = 0;
    else if (buf[0] == '1')
        ts->SMWP_enable = 1;
    else
        return -EINVAL;
    I("%s: SMART_WAKEUP_enable = %d.\n", __func__, ts->SMWP_enable);
    return len;
}
#endif

#ifdef HX_TP_PROC_DIAG
static int himax_touch_proc_init(void)
{
    himax_touch_proc_dir = proc_mkdir( HIMAX_PROC_TOUCH_FOLDER, NULL);
    if (himax_touch_proc_dir == NULL) {
        E(" %s: himax_touch_proc_dir file create failed!\n", __func__);
        return -ENOMEM;
    }
    himax_proc_diag_file = proc_create(HIMAX_PROC_DIAG_FILE, (S_IWUSR | S_IRUGO),
                       himax_touch_proc_dir, &himax_proc_diag_ops);
    if (himax_proc_diag_file == NULL) {
        E(" %s: proc diag file create failed!\n", __func__);
        goto fail;
    }
    return 0 ;
fail:
    remove_proc_entry( HIMAX_PROC_TOUCH_FOLDER, NULL );
    return -ENOMEM;
}
#endif

#ifdef HX_TP_PROC_DIAG
static void himax_touch_proc_deinit(void)
{
    remove_proc_entry(HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir);
    remove_proc_entry( HIMAX_PROC_TOUCH_FOLDER, NULL );
}
#endif

#endif

#ifdef CONFIG_OF
#if defined(HX_LOADIN_CONFIG)||defined(HX_AUTO_UPDATE_CONFIG)
static int himax_parse_config(struct himax_ts_data *ts, struct himax_config *pdata)
{
    struct himax_config *cfg_table;
    struct device_node *node, *pp = NULL;
    struct property *prop;
    uint8_t cnt = 0, i = 0;
    u32 data = 0;
    uint32_t coords[4] = {0};
    int len = 0;
    char str[6] = {0};
    node = ts->client->dev.of_node;
    if (node == NULL) {
        E(" %s, can't find device_node", __func__);
        return -ENODEV;
    }
    while ((pp = of_get_next_child(node, pp)))
        cnt++;
    if (!cnt)
        return -ENODEV;
    cfg_table = kzalloc(cnt * (sizeof * cfg_table), GFP_KERNEL);
    if (!cfg_table)
        return -ENOMEM;
    pp = NULL;
    while ((pp = of_get_next_child(node, pp))) {
        if (of_property_read_u32(pp, "default_cfg", &data) == 0)
            cfg_table[i].default_cfg = data;
        if (of_property_read_u32(pp, "sensor_id", &data) == 0)
            cfg_table[i].sensor_id = (data);
        if (of_property_read_u32(pp, "fw_ver_main", &data) == 0)
            cfg_table[i].fw_ver_main = data;
        if (of_property_read_u32(pp, "fw_ver_minor", &data) == 0)
            cfg_table[i].fw_ver_minor = data;
        if (of_property_read_u32_array(pp, "himax,tw-coords", coords, 4) == 0) {
            cfg_table[i].tw_x_min = coords[0], cfg_table[i].tw_x_max = coords[1];    //x
            cfg_table[i].tw_y_min = coords[2], cfg_table[i].tw_y_max = coords[3];    //y
        }
        if (of_property_read_u32_array(pp, "himax,pl-coords", coords, 4) == 0) {
            cfg_table[i].pl_x_min = coords[0], cfg_table[i].pl_x_max = coords[1];    //x
            cfg_table[i].pl_y_min = coords[2], cfg_table[i].pl_y_max = coords[3];    //y
        }
        prop = of_find_property(pp, "c1", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c1");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c1, prop->value, len);
        prop = of_find_property(pp, "c2", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c2");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c2, prop->value, len);
        prop = of_find_property(pp, "c3", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c3");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c3, prop->value, len);
        prop = of_find_property(pp, "c4", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c4");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c4, prop->value, len);
        prop = of_find_property(pp, "c5", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c5");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c5, prop->value, len);
        prop = of_find_property(pp, "c6", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c6");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c6, prop->value, len);
        prop = of_find_property(pp, "c7", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c7");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c7, prop->value, len);
        prop = of_find_property(pp, "c8", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c8");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c8, prop->value, len);
        prop = of_find_property(pp, "c9", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c9");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c9, prop->value, len);
        prop = of_find_property(pp, "c10", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c10");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c10, prop->value, len);
        prop = of_find_property(pp, "c11", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c11");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c11, prop->value, len);
        prop = of_find_property(pp, "c12", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c12");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c12, prop->value, len);
        prop = of_find_property(pp, "c13", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c13");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c13, prop->value, len);
        prop = of_find_property(pp, "c14", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c14");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c14, prop->value, len);
        prop = of_find_property(pp, "c15", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c15");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c15, prop->value, len);
        prop = of_find_property(pp, "c16", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c16");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c16, prop->value, len);
        prop = of_find_property(pp, "c17", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c17");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c17, prop->value, len);
        prop = of_find_property(pp, "c18", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c18");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c18, prop->value, len);
        prop = of_find_property(pp, "c19", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c19");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c19, prop->value, len);
        prop = of_find_property(pp, "c20", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c20");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c20, prop->value, len);
        prop = of_find_property(pp, "c21", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c21");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c21, prop->value, len);
        prop = of_find_property(pp, "c22", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c22");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c22, prop->value, len);
        prop = of_find_property(pp, "c23", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c23");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c23, prop->value, len);
        prop = of_find_property(pp, "c24", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c24");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c24, prop->value, len);
        prop = of_find_property(pp, "c25", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c25");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c25, prop->value, len);
        prop = of_find_property(pp, "c26", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c26");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c26, prop->value, len);
        prop = of_find_property(pp, "c27", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c27");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c27, prop->value, len);
        prop = of_find_property(pp, "c28", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c28");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c28, prop->value, len);
        prop = of_find_property(pp, "c29", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c29");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c29, prop->value, len);
        prop = of_find_property(pp, "c30", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c30");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c30, prop->value, len);
        prop = of_find_property(pp, "c31", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c31");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c31, prop->value, len);
        prop = of_find_property(pp, "c32", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c32");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c32, prop->value, len);
        prop = of_find_property(pp, "c33", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c33");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c33, prop->value, len);
        prop = of_find_property(pp, "c34", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c34");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c34, prop->value, len);
        prop = of_find_property(pp, "c35", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c35");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c35, prop->value, len);
        prop = of_find_property(pp, "c36", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c36");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c36, prop->value, len);
        prop = of_find_property(pp, "c37", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c37");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c37, prop->value, len);
        prop = of_find_property(pp, "c38", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c38");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c38, prop->value, len);
        prop = of_find_property(pp, "c39", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c39");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c39, prop->value, len);
        prop = of_find_property(pp, "c40", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c40");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c40, prop->value, len);
#if 1
        I(" config version=[%02x]", cfg_table[i].c40[1]);
#endif
        prop = of_find_property(pp, "c41", &len);
        if ((!prop) || (!len)) {
            strcpy(str, "c41");
            goto of_find_property_error;
        }
        memcpy(cfg_table[i].c41, prop->value, len);
#if 1
        I(" DT#%d-def_cfg:%d,id:%05x, FW:%x.%x, len:%d,", i,
          cfg_table[i].default_cfg, cfg_table[i].sensor_id,
          cfg_table[i].fw_ver_main, cfg_table[i].fw_ver_minor, cfg_table[i].length);
        //I(" pl=[x_m%02d y_M%04d]\n", cfg_table[i].pl_x_min, cfg_table[i].pl_y_max);
#endif
        i++;
of_find_property_error:
        if (!prop) {
            D(" %s:Looking up %s property in node %s failed",
              __func__, str, pp->full_name);
            return -ENODEV;
        } else if (!len) {
            D(" %s:Invalid length of configuration data in %s\n",
              __func__, str);
            return -EINVAL;
        }
    }
    i = 0;    //confirm which config we should load
    while ((ts->vendor_fw_ver_H << 8 | ts->vendor_fw_ver_L) <
           (cfg_table[i].fw_ver_main << 8 | cfg_table[i].fw_ver_minor)) {
        i++;
    }
    if (cfg_table[i].default_cfg != 0)
        goto startloadconf;
    while (cfg_table[i].sensor_id > 0 && (cfg_table[i].sensor_id !=  ts->vendor_sensor_id)) {
        I(" id:%#x!=%#x, (i++)", cfg_table[i].sensor_id, ts->vendor_sensor_id);
        i++;
    }
startloadconf:
    if (i <= cnt) {
        I(" DT-%s cfg idx(%d) in cnt(%d)", __func__, i, cnt);
        pdata->fw_ver_main  = cfg_table[i].fw_ver_main;
        pdata->fw_ver_minor = cfg_table[i].fw_ver_minor;
        pdata->sensor_id          = cfg_table[i].sensor_id;
        memcpy(pdata->c1, cfg_table[i].c1, sizeof(pdata->c1));
        memcpy(pdata->c2, cfg_table[i].c2, sizeof(pdata->c2));
        memcpy(pdata->c3, cfg_table[i].c3, sizeof(pdata->c3));
        memcpy(pdata->c4, cfg_table[i].c4, sizeof(pdata->c4));
        memcpy(pdata->c5, cfg_table[i].c5, sizeof(pdata->c5));
        memcpy(pdata->c6, cfg_table[i].c6, sizeof(pdata->c6));
        memcpy(pdata->c7, cfg_table[i].c7, sizeof(pdata->c7));
        memcpy(pdata->c8, cfg_table[i].c8, sizeof(pdata->c8));
        memcpy(pdata->c9, cfg_table[i].c9, sizeof(pdata->c9));
        memcpy(pdata->c10, cfg_table[i].c10, sizeof(pdata->c10));
        memcpy(pdata->c11, cfg_table[i].c11, sizeof(pdata->c11));
        memcpy(pdata->c12, cfg_table[i].c12, sizeof(pdata->c12));
        memcpy(pdata->c13, cfg_table[i].c13, sizeof(pdata->c13));
        memcpy(pdata->c14, cfg_table[i].c14, sizeof(pdata->c14));
        memcpy(pdata->c15, cfg_table[i].c15, sizeof(pdata->c15));
        memcpy(pdata->c16, cfg_table[i].c16, sizeof(pdata->c16));
        memcpy(pdata->c17, cfg_table[i].c17, sizeof(pdata->c17));
        memcpy(pdata->c18, cfg_table[i].c18, sizeof(pdata->c18));
        memcpy(pdata->c19, cfg_table[i].c19, sizeof(pdata->c19));
        memcpy(pdata->c20, cfg_table[i].c20, sizeof(pdata->c20));
        memcpy(pdata->c21, cfg_table[i].c21, sizeof(pdata->c21));
        memcpy(pdata->c22, cfg_table[i].c22, sizeof(pdata->c22));
        memcpy(pdata->c23, cfg_table[i].c23, sizeof(pdata->c23));
        memcpy(pdata->c24, cfg_table[i].c24, sizeof(pdata->c24));
        memcpy(pdata->c25, cfg_table[i].c25, sizeof(pdata->c25));
        memcpy(pdata->c26, cfg_table[i].c26, sizeof(pdata->c26));
        memcpy(pdata->c27, cfg_table[i].c27, sizeof(pdata->c27));
        memcpy(pdata->c28, cfg_table[i].c28, sizeof(pdata->c28));
        memcpy(pdata->c29, cfg_table[i].c29, sizeof(pdata->c29));
        memcpy(pdata->c30, cfg_table[i].c30, sizeof(pdata->c30));
        memcpy(pdata->c31, cfg_table[i].c31, sizeof(pdata->c31));
        memcpy(pdata->c32, cfg_table[i].c32, sizeof(pdata->c32));
        memcpy(pdata->c33, cfg_table[i].c33, sizeof(pdata->c33));
        memcpy(pdata->c34, cfg_table[i].c34, sizeof(pdata->c34));
        memcpy(pdata->c35, cfg_table[i].c35, sizeof(pdata->c35));
        memcpy(pdata->c36, cfg_table[i].c36, sizeof(pdata->c36));
        memcpy(pdata->c37, cfg_table[i].c37, sizeof(pdata->c37));
        memcpy(pdata->c38, cfg_table[i].c38, sizeof(pdata->c38));
        memcpy(pdata->c39, cfg_table[i].c39, sizeof(pdata->c39));
        memcpy(pdata->c40, cfg_table[i].c40, sizeof(pdata->c40));
        memcpy(pdata->c41, cfg_table[i].c41, sizeof(pdata->c41));
        ts->tw_x_min = cfg_table[i].tw_x_min, ts->tw_x_max = cfg_table[i].tw_x_max;    //x
        ts->tw_y_min = cfg_table[i].tw_y_min, ts->tw_y_max = cfg_table[i].tw_y_max;    //y
        ts->pl_x_min = cfg_table[i].pl_x_min, ts->pl_x_max = cfg_table[i].pl_x_max;    //x
        ts->pl_y_min = cfg_table[i].pl_y_min, ts->pl_y_max = cfg_table[i].pl_y_max;    //y
        I(" DT#%d-def_cfg:%d,id:%05x, FW:%x.%x, len:%d,", i,
          cfg_table[i].default_cfg, cfg_table[i].sensor_id,
          cfg_table[i].fw_ver_main, cfg_table[i].fw_ver_minor, cfg_table[i].length);
        I(" DT-%s:tw-coords = %d, %d, %d, %d\n", __func__, ts->tw_x_min,
          ts->tw_x_max, ts->tw_y_min, ts->tw_y_max);
        I(" DT-%s:pl-coords = %d, %d, %d, %d\n", __func__, ts->pl_x_min,
          ts->pl_x_max, ts->pl_y_min, ts->pl_y_max);
        I(" config version=[%02x]", pdata->c40[1]);
    } else {
        E(" DT-%s cfg idx(%d) > cnt(%d)", __func__, i, cnt);
        return -EINVAL;
    }
    return 0;
}
#endif

static int himax_parse_dt(struct himax_ts_data *ts,
              struct himax_i2c_platform_data *pdata)
{
    int rc, coords_size = 0;
    uint32_t coords[4] = {0};
    struct property *prop;
    struct device_node *dt = ts->client->dev.of_node;
    u32 data = 0;
    prop = of_find_property(dt, "himax,panel-coords", NULL);
    if (prop) {
        coords_size = prop->length / sizeof(u32);
        if (coords_size != 4)
            D(" %s:Invalid panel coords size %d", __func__, coords_size);
    }
    if (of_property_read_u32_array(dt, "himax,panel-coords", coords, coords_size) == 0) {
        pdata->abs_x_min = coords[0], pdata->abs_x_max = coords[1];
        pdata->abs_y_min = coords[2], pdata->abs_y_max = coords[3];
        I(" DT-%s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
          pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
    }
    prop = of_find_property(dt, "himax,display-coords", NULL);
    if (prop) {
        coords_size = prop->length / sizeof(u32);
        if (coords_size != 4)
            D(" %s:Invalid display coords size %d", __func__, coords_size);
    }
    rc = of_property_read_u32_array(dt, "himax,display-coords", coords, coords_size);
    if (rc && (rc != -EINVAL)) {
        D(" %s:Fail to read display-coords %d\n", __func__, rc);
        return rc;
    }
    pdata->screenWidth  = coords[1];
    pdata->screenHeight = coords[3];
    I(" DT-%s:display-coords = (%d, %d)", __func__, pdata->screenWidth,
      pdata->screenHeight);
    pdata->gpio_irq = of_get_named_gpio(dt, "himax,irq-gpio", 0);
    if (!gpio_is_valid(pdata->gpio_irq)) {
        I(" DT:gpio_irq value is not valid\n");
    }
    pdata->gpio_reset = of_get_named_gpio(dt, "himax,rst-gpio", 0);
    if (!gpio_is_valid(pdata->gpio_reset)) {
        I(" DT:gpio_rst value is not valid\n");
    }
    pdata->gpio_3v3_en = of_get_named_gpio(dt, "himax,3v3-gpio", 0);
    if (!gpio_is_valid(pdata->gpio_3v3_en)) {
        I(" DT:gpio_3v3_en value is not valid\n");
    }
    I(" DT:gpio_irq=%d, gpio_rst=%d, gpio_3v3_en=%d", pdata->gpio_irq, pdata->gpio_reset, pdata->gpio_3v3_en);
    if (of_property_read_u32(dt, "report_type", &data) == 0) {
        pdata->protocol_type = data;
        I(" DT:protocol_type=%d", pdata->protocol_type);
    }
    return 0;
}
#endif

#ifdef HX_AUTO_UPDATE_FW
static ssize_t himax_fw_update_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int ret;
    if (fw_update_result)
        ret += sprintf(buf, "FW Update Success \n");
    else
        ret += sprintf(buf, "FW Update Fail \n");
    return ret;
}

static ssize_t himax_fw_update_write(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    int mode = 0;
    int ret;
    bool manualUpgrade = false;

    sscanf(buf, "%d", &mode);
    manualUpgrade = mode == FW_UPDATE_MANUAL_MODE;
    himax_int_enable(private_ts->client->irq, 0, true);
    ret = i_update_FW(manualUpgrade);
    himax_int_enable(private_ts->client->irq, 1, true);
    return size;
}

static ssize_t himax_fw_version_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int ret;
    himax_read_FW_ver(false);
    ret = sprintf(buf, "%x.%x.%x\n", private_ts->vendor_fw_ver_H,
        private_ts->vendor_fw_ver_L, private_ts->vendor_config_ver);
    return ret;
}

static ssize_t himax_checksum_read(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    int ret;
    ret = himax_calculateChecksum(false, true);
    return sprintf(buf, "%d%d%d%d\n", IC_checksum[0],IC_checksum[1],IC_checksum[2],IC_checksum[3]);
}
#endif

#ifdef ASUS_FACTORY_BUILD
static int g_bmmi_status = 0;

static ssize_t himax_show_status(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    return snprintf(buf, 20, "%d\n", g_bmmi_status);
}

static DEVICE_ATTR(mode, S_IRUGO, himax_show_status, NULL);
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
static DEVICE_ATTR(debug_level, S_IWUSR|S_IWGRP|S_IRUGO, himax_debug_level_read, himax_debug_level_write);
static DEVICE_ATTR(vendor, S_IRUGO, himax_vendor_read, NULL);
static DEVICE_ATTR(attn, S_IRUGO, himax_attn_read, NULL);
static DEVICE_ATTR(int_en, S_IWUSR|S_IWGRP|S_IRUGO, himax_int_en_read, himax_int_en_write);
static DEVICE_ATTR(layout, S_IWUSR|S_IWGRP|S_IRUGO, himax_layout_read, himax_layout_write);
#endif
#ifdef HX_TP_PROC_RESET
static DEVICE_ATTR(reset, S_IWUSR|S_IWGRP|S_IRUGO, NULL, himax_reset_write);
#endif
#ifdef HX_TP_PROC_REGISTER
static DEVICE_ATTR(register, S_IWUSR|S_IWGRP|S_IRUGO, himax_register_read, himax_register_write);
#endif
#ifdef HX_TP_PROC_DEBUG
static DEVICE_ATTR(debug, S_IWUSR|S_IWGRP|S_IRUGO, himax_debug_read, himax_debug_write);
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
static DEVICE_ATTR(flash_dump, S_IWUSR|S_IWGRP|S_IRUGO, himax_flash_read, himax_flash_write);
#endif
#ifdef HX_TP_PROC_SELF_TEST
static DEVICE_ATTR(self_test, S_IRUGO, himax_self_test_read, NULL);
#endif
#ifdef HX_TP_PROC_HITOUCH
static DEVICE_ATTR(hitouch, S_IWUSR|S_IWGRP|S_IRUGO, himax_hitouch_read, himax_hitouch_write);
#endif
#ifdef HX_SMART_WAKEUP
static DEVICE_ATTR(SMWP, S_IWUSR|S_IWGRP|S_IRUGO, himax_SMWP_read, himax_SMWP_write);
#endif
#ifdef HX_AUTO_UPDATE_FW
static DEVICE_ATTR(fw_update, S_IWUSR|S_IWGRP|S_IRUGO, himax_fw_update_read, himax_fw_update_write);
static DEVICE_ATTR(fw_version, S_IRUGO, himax_fw_version_read, NULL);
static DEVICE_ATTR(checksum, S_IRUGO, himax_checksum_read, NULL);
#endif

static struct attribute *himax_attrs[] = {
#ifdef ASUS_FACTORY_BUILD
    &dev_attr_mode.attr,
#endif
#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
    &dev_attr_debug_level.attr,
    &dev_attr_vendor.attr,
    &dev_attr_attn.attr,
    &dev_attr_int_en.attr,
    &dev_attr_layout.attr,
#endif
#ifdef HX_TP_PROC_RESET
    &dev_attr_reset.attr,
#endif
#ifdef HX_TP_PROC_REGISTER
    &dev_attr_register.attr,
#endif
#ifdef HX_TP_PROC_DEBUG
    &dev_attr_debug.attr,
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
    &dev_attr_flash_dump.attr,
#endif
#ifdef HX_TP_PROC_SELF_TEST
    &dev_attr_self_test.attr,
#endif
#ifdef HX_TP_PROC_HITOUCH
    &dev_attr_hitouch.attr,
#endif
#ifdef HX_SMART_WAKEUP
    &dev_attr_SMWP.attr,
#endif
#ifdef HX_AUTO_UPDATE_FW
    &dev_attr_fw_update.attr,
    &dev_attr_fw_version.attr,
    &dev_attr_checksum.attr,
#endif
    NULL
};

static const struct attribute_group himax_attribute_group = {
    .attrs = himax_attrs,
};

static int himax852xes_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = -ENOMEM;
    struct himax_ts_data *ts;
    struct himax_i2c_platform_data *pdata;
    printk("%s:\n", __func__);
    //Check I2C functionality
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        E("%s: i2c check functionality error\n", __func__);
        err = -ENODEV;
        goto err_check_functionality_failed;
    }
    ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
    if (ts == NULL) {
        E("%s: allocate himax_ts_data failed\n", __func__);
        err = -ENOMEM;
        goto err_alloc_data_failed;
    }
    i2c_set_clientdata(client, ts);
    ts->client = client;
    ts->dev = &client->dev;
#if 0
    /* Remove this since this will cause request_firmware fail.
       Thie line has no use to touch function. */
    dev_set_name(ts->dev, HIMAX852xes_NAME);
#endif
    pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
    if (pdata == NULL) { /*Allocate Platform data space*/
        err = -ENOMEM;
        goto err_dt_platform_data_fail;
    }
#ifdef CONFIG_OF
    if (client->dev.of_node) { /*DeviceTree Init Platform_data*/
        err = himax_parse_dt(ts, pdata);
        if (err < 0) {
            I(" pdata is NULL for DT\n");
            goto err_alloc_dt_pdata_failed;
        }
    }
#else
    pdata = client->dev.platform_data;
    if (pdata == NULL) {
        I(" pdata is NULL(dev.platform_data)\n");
        goto err_get_platform_data_fail;
    }
#endif
#ifdef HX_RST_PIN_FUNC
    ts->rst_gpio = pdata->gpio_reset;
#endif
    himax_gpio_power_config(ts->client, pdata);
#ifndef CONFIG_OF
    if (pdata->power) {
        err = pdata->power(1);
        if (err < 0) {
            E("%s: power on failed\n", __func__);
            goto err_power_failed;
        }
    }
#endif
    private_ts = ts;
    //Get Himax IC Type / FW information / Calculate the point number
    if (himax_ic_package_check(ts) == false) {
        E("Himax chip doesn NOT EXIST");
        goto err_ic_package_failed;
    }
    if (pdata->virtual_key)
        ts->button = pdata->virtual_key;
#ifdef  HX_TP_PROC_FLASH_DUMP
    ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
    if (!ts->flash_wq) {
        E("%s: create flash workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_create_flash_wq_failed;
    }
    INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);
    setSysOperation(0);
    setFlashBuffer();
#endif
    himax_read_TP_info(client);

    //Himax Power On and Load Config
    if (himax_loadSensorConfig(client, pdata) < 0) {
        E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
        goto err_detect_failed;
    }
    calculate_point_number();
#ifdef HX_TP_PROC_DIAG
    setXChannel(HX_RX_NUM); // X channel
    setYChannel(HX_TX_NUM); // Y channel
    setMutualBuffer();
    if (getMutualBuffer() == NULL) {
        E("%s: mutual buffer allocate fail failed\n", __func__);
        goto err_setchannel_failed;
    }
#ifdef HX_TP_PROC_2T2R
    if (Is_2T2R) {
        setXChannel_2(HX_RX_NUM_2); // X channel
        setYChannel_2(HX_TX_NUM_2); // Y channel
        setMutualBuffer_2();
        if (getMutualBuffer_2() == NULL) {
            E("%s: mutual buffer 2 allocate fail failed\n", __func__);
            goto err_setchannel_failed;
        }
    }
#endif
#endif
#ifdef CONFIG_OF
    ts->power = pdata->power;
#endif
    ts->pdata = pdata;
    ts->x_channel = HX_RX_NUM;
    ts->y_channel = HX_TX_NUM;
    ts->nFinger_support = HX_MAX_PT;
    //calculate the i2c data size
    calcDataSize(ts->nFinger_support);
    I("%s: calcDataSize complete\n", __func__);
    ts->pdata->abs_pressure_min        = 0;
    ts->pdata->abs_pressure_max        = 200;
    ts->pdata->abs_width_min           = 0;
    ts->pdata->abs_width_max           = 200;
    pdata->cable_config[0]             = 0xF0;
    pdata->cable_config[1]             = 0x00;
    ts->suspended                      = false;
    ts->protocol_type = pdata->protocol_type;
    I("%s: Use Protocol Type %c\n", __func__,
      ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');
    err = himax_input_register(ts);
    if (err) {
        E("%s: Unable to register %s input device\n",
          __func__, ts->input_dev->name);
        goto err_input_register_device_failed;
    }
#ifdef CONFIG_FB
    ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");
    if (!ts->himax_att_wq) {
        E(" allocate HMX_att_wq failed\n");
        err = -ENOMEM;
        goto err_get_intr_bit_failed;
    }
    INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
    queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));
#endif
#ifdef HX_CHIP_STATUS_MONITOR //for ESD solution
    ts->himax_chip_monitor_wq = create_singlethread_workqueue("himax_chip_monitor_wq");
    if (!ts->himax_chip_monitor_wq) {
        E(" %s: create workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_create_chip_monitor_wq_failed;
    }
    INIT_DELAYED_WORK(&ts->himax_chip_monitor, himax_chip_monitor_function);
    queue_delayed_work(ts->himax_chip_monitor_wq, &ts->himax_chip_monitor, HX_POLLING_TIMER * HZ);
#endif
#ifdef HX_SMART_WAKEUP
    ts->SMWP_enable = 1;
    wake_lock_init(&ts->ts_SMWP_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME);
#endif
    wake_lock_init(&ts->ts_flash_wake_lock, WAKE_LOCK_SUSPEND, HIMAX852xes_NAME);
#ifdef HX_TP_PROC_DIAG
    himax_touch_proc_init();
#endif
#ifdef HX_ESD_WORKAROUND
    ESD_RESET_ACTIVATE = 0;
#endif
    HW_RESET_ACTIVATE = 0;
#ifdef ASUS_FACTORY_BUILD
    g_bmmi_status = 1;
#endif
    err = sysfs_create_group(&client->dev.kobj, &himax_attribute_group);
    if (err)
        goto exit_sysfs;
    err = himax_ts_register_interrupt(ts->client);
    if (err)
        goto err_register_interrupt_failed;
    return 0;
err_register_interrupt_failed:
    if (ts->irq_enabled) {
        himax_int_enable(ts->client->irq, 0, true);
        free_irq(ts->client->irq, ts);
    }
    if (!ts->use_irq) {
        hrtimer_cancel(&ts->timer);
        destroy_workqueue(ts->himax_wq);
    }
#ifdef HX_TP_PROC_DIAG
    himax_touch_proc_deinit();
#endif
    wake_lock_destroy(&ts->ts_flash_wake_lock);
#ifdef HX_SMART_WAKEUP
    wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
    cancel_delayed_work_sync(&ts->himax_chip_monitor);
    destroy_workqueue(ts->himax_chip_monitor_wq);
err_create_chip_monitor_wq_failed:
#endif
exit_sysfs:
    sysfs_remove_group(&client->dev.kobj, &himax_attribute_group);
#ifdef CONFIG_FB
    cancel_delayed_work_sync(&ts->work_att);
    destroy_workqueue(ts->himax_att_wq);
    if (fb_unregister_client(&ts->fb_notif))
        I("Error occurred while unregistering fb_notifier.\n");
err_get_intr_bit_failed:
#endif
    input_free_device(ts->input_dev);
err_input_register_device_failed:
#ifdef HX_TP_PROC_DIAG
err_setchannel_failed:
#endif
err_detect_failed:
#ifdef  HX_TP_PROC_FLASH_DUMP
    destroy_workqueue(ts->flash_wq);
err_create_flash_wq_failed:
#endif
err_ic_package_failed:
#ifndef CONFIG_OF
err_power_failed:
#endif
#ifdef CONFIG_OF
err_alloc_dt_pdata_failed:
#else
err_get_platform_data_fail:
#endif
    kfree(pdata);
err_dt_platform_data_fail:
    kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
    HX_DRIVER_PROBE_Fial = 1;
    return err;
}

static int himax852xes_remove(struct i2c_client *client)
{
    struct himax_ts_data *ts = i2c_get_clientdata(client);
    if (ts->irq_enabled) {
        himax_int_enable(ts->client->irq, 0, true);
        free_irq(ts->client->irq, ts);
    }
    if (!ts->use_irq) {
        hrtimer_cancel(&ts->timer);
        destroy_workqueue(ts->himax_wq);
    }
#ifdef HX_TP_PROC_DIAG
    himax_touch_proc_deinit();
#endif
#ifdef HX_SMART_WAKEUP
    wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
    cancel_delayed_work_sync(&ts->himax_chip_monitor);
    destroy_workqueue(ts->himax_chip_monitor_wq);
#endif
#ifdef CONFIG_FB
    cancel_delayed_work_sync(&ts->work_att);
    destroy_workqueue(ts->himax_att_wq);
    if (fb_unregister_client(&ts->fb_notif))
        I("Error occurred while unregistering fb_notifier.\n");
#endif
    input_free_device(ts->input_dev);
#ifdef  HX_TP_PROC_FLASH_DUMP
    destroy_workqueue(ts->flash_wq);
#endif
    kfree(ts);
    return 0;
}

static int himax852xes_suspend(struct device *dev)
{
    int ret;
    int i = 0;
    uint8_t buf[2] = {0};
#ifdef HX_CHIP_STATUS_MONITOR
    int t = 0;
#endif
    struct himax_ts_data *ts;
    if (HX_DRIVER_PROBE_Fial) {
        I("%s: Driver probe fail. \n", __func__);
        return 0;
    }
    if (fw_update_processing) {
        I("fw updating, skip suspend\n");
        return 0;
    } else {
        I("%s: Enter suspended. \n", __func__);
        ts = dev_get_drvdata(dev);
        if (ts->suspended) {
            I("%s: Already suspended. Skipped. \n", __func__);
            return 0;
        } else {
            ts->suspended = true;
            I("%s: enter \n", __func__);
        }
#ifdef HX_TP_PROC_FLASH_DUMP
        if (getFlashDumpGoing()) {
            I("[himax] %s: Flash dump is going, reject suspend\n", __func__);
            return 0;
        }
#endif
#ifdef HX_TP_PROC_HITOUCH
        if (hitouch_is_connect) {
            I("[himax] %s: Hitouch connect, reject suspend\n", __func__);
            return 0;
        }
#endif
#ifdef HX_CHIP_STATUS_MONITOR
        if (HX_ON_HAND_SHAKING) { //chip on hand shaking,wait hand shaking
            for (t = 0; t < 100; t++) {
                if (HX_ON_HAND_SHAKING == 0) { //chip on hand shaking end
                    I("%s:HX_ON_HAND_SHAKING OK check %d times\n", __func__, t);
                    break;
                } else
                    msleep(1);
            }
            if (t == 100) {
                E("%s:HX_ON_HAND_SHAKING timeout reject suspend\n", __func__);
                return 0;
            }
        }
#endif
        if (haspoint) {
            for (i = 0; i < ts->nFinger_support; i++) {
                if (point_flag[i] == 1) {
                    point_flag[i] = 0;
                    input_mt_slot(ts->input_dev, i);
                    input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
                }
            }
            input_sync(ts->input_dev);
            haspoint = false;
        }
#ifdef HX_SMART_WAKEUP
        if (ts->SMWP_enable) {
            atomic_set(&ts->suspend_mode, 1);
            ts->pre_finger_mask = 0;
            FAKE_POWER_KEY_SEND = false;
            buf[0] = 0x8F;
            buf[1] = 0x20;
            ret = i2c_himax_master_write(ts->client, buf, 2, DEFAULT_RETRY_CNT);
            if (ret < 0) {
                E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
            }
            I("Enable GESTURE and DOUBLE CLICK, reject suspend \n");
            return 0;
        }
#endif
        himax_int_enable(ts->client->irq, 0, true);
#ifdef HX_CHIP_STATUS_MONITOR
        HX_CHIP_POLLING_COUNT = 0;
        cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif
        //Himax 852xes IC enter sleep mode
        buf[0] = HX_CMD_TSSOFF;
        ret = i2c_himax_master_write(ts->client, buf, 1, DEFAULT_RETRY_CNT);
        if (ret < 0) {
            E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
        }
        msleep(40);
        buf[0] = HX_CMD_TSSLPIN;
        ret = i2c_himax_master_write(ts->client, buf, 1, DEFAULT_RETRY_CNT);
        if (ret < 0) {
            E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
        }
        if (!ts->use_irq) {
            ret = cancel_work_sync(&ts->work);
            if (ret)
                himax_int_enable(ts->client->irq, 1, true);
        }
        //ts->first_pressed = 0;
        atomic_set(&ts->suspend_mode, 1);
        ts->pre_finger_mask = 0;
        if (ts->pdata->powerOff3V3 && ts->pdata->power)
            ts->pdata->power(0);
        return 0;
    }
}

static int himax852xes_resume(struct device *dev)
{
#ifdef HX_SMART_WAKEUP
    int ret;
    uint8_t buf[2] = {0};
#endif
#ifdef HX_CHIP_STATUS_MONITOR
    int t = 0;
#endif
    struct himax_ts_data *ts;
    if (HX_DRIVER_PROBE_Fial) {
        I("%s: Driver probe fail. \n", __func__);
        return 0;
    }
    if (fw_update_processing) {
        I("fw updating, skip resume\n");
        return 0;
    } else {
        I("%s: enter \n", __func__);
        ts = dev_get_drvdata(dev);
        if (ts->pdata->powerOff3V3 && ts->pdata->power)
            ts->pdata->power(1);
#ifdef HX_CHIP_STATUS_MONITOR
        if (HX_ON_HAND_SHAKING) { //chip on hand shaking,wait hand shaking
            for (t = 0; t < 100; t++) {
                if (HX_ON_HAND_SHAKING == 0) { //chip on hand shaking end
                    I("%s:HX_ON_HAND_SHAKING OK check %d times\n", __func__, t);
                    break;
                } else
                    msleep(1);
            }
            if (t == 100) {
                E("%s:HX_ON_HAND_SHAKING timeout reject resume\n", __func__);
                return 0;
            }
        }
#endif
#ifdef HX_SMART_WAKEUP
        if (ts->SMWP_enable) {
            //Sense Off
            i2c_himax_write_command(ts->client, HX_CMD_TSSOFF, DEFAULT_RETRY_CNT);
            msleep(40);
            //Sleep in
            i2c_himax_write_command(ts->client, HX_CMD_TSSLPIN, DEFAULT_RETRY_CNT);
            buf[0] = 0x8F;
            buf[1] = 0x00;
            ret = i2c_himax_master_write(ts->client, buf, 2, DEFAULT_RETRY_CNT);
            if (ret < 0) {
                E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
            }
            msleep(50);
        }
#endif
        //Sense On
        i2c_himax_write_command(ts->client, HX_CMD_TSSON, DEFAULT_RETRY_CNT);
        msleep(30);
        i2c_himax_write_command(ts->client, HX_CMD_TSSLPOUT, DEFAULT_RETRY_CNT);
        atomic_set(&ts->suspend_mode, 0);
#ifdef HX_SMART_WAKEUP
        if (!ts->SMWP_enable) {
            himax_int_enable(ts->client->irq, 1, true);
        }
#endif
#ifdef HX_CHIP_STATUS_MONITOR
        HX_CHIP_POLLING_COUNT = 0;
        queue_delayed_work(ts->himax_chip_monitor_wq, &ts->himax_chip_monitor, HX_POLLING_TIMER * HZ); //for ESD solution
#endif
        ts->suspended = false;
        return 0;
    }
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct himax_ts_data *ts =
        container_of(self, struct himax_ts_data, fb_notif);
    I(" %s\n", __func__);
    if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
        ts->client) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
            himax852xes_resume(&ts->client->dev);
            break;
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
            himax852xes_suspend(&ts->client->dev);
            break;
        }
    }
    return 0;
}
#endif

static const struct i2c_device_id himax852xes_ts_id[] = {
    {HIMAX852xes_NAME, 0 },
    {}
};

static const struct dev_pm_ops himax852xes_pm_ops = {
    .suspend = himax852xes_suspend,
    .resume  = himax852xes_resume,
};

#ifdef CONFIG_OF
static struct of_device_id himax_match_table[] = {
    {.compatible = "himax,852xes" },
    {},
};
#else
#define himax_match_table NULL
#endif

static struct i2c_driver himax852xes_driver = {
    .id_table = himax852xes_ts_id,
    .probe = himax852xes_probe,
    .remove = himax852xes_remove,
    .driver = {
        .name = HIMAX852xes_NAME,
        .owner = THIS_MODULE,
        .of_match_table = himax_match_table,
#ifdef CONFIG_PM
        .pm = &himax852xes_pm_ops,
#endif
    },
};

static void __init himax852xes_init_async(void *unused, async_cookie_t cookie)
{
    I("%s:Enter \n", __func__);
    i2c_add_driver(&himax852xes_driver);
}

static int __init himax852xes_init(void)
{
    I("Himax 852xES touch panel driver init\n");
    async_schedule(himax852xes_init_async, NULL);
    return 0;
}

static void __exit himax852xes_exit(void)
{
    i2c_del_driver(&himax852xes_driver);
}

module_init(himax852xes_init);
module_exit(himax852xes_exit);

MODULE_DESCRIPTION("Himax852xes driver");
MODULE_LICENSE("GPL");

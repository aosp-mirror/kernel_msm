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

#ifndef HIMAX852xES_H
#define HIMAX852xES_H

#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/wakelock.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#define HIMAX_DRIVER_VER "0.1.8.0"

#define HIMAX852xes_NAME "Himax852xes"
#define HIMAX852xes_FINGER_SUPPORT_NUM 10
#define HIMAX_I2C_ADDR                0x48
#define INPUT_DEV_NAME    "HX852XES"
#define FLASH_DUMP_FILE "/data/user/Flash_Dump.bin"
#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.csv"

#define D(x...) pr_notice("[HXTP] " x)
#define I(x...) pr_info("[HXTP] " x)
#define W(x...) pr_warning("[HXTP][WARNING] " x)
#define E(x...) pr_err("[HXTP][ERROR] " x)
#define DIF(x...) \
    if (debug_flag) \
    printk("[HXTP][DEBUG] " x) \
} while(0)

#ifdef CONFIG_TOUCHSCREEN_HIMAX_DEBUG
#define HX_TP_PROC_DIAG
#define HX_TP_PROC_RESET
#define HX_TP_PROC_REGISTER
#define HX_TP_PROC_DEBUG
#define HX_TP_PROC_FLASH_DUMP
#define HX_TP_PROC_SELF_TEST
#define HX_TP_PROC_HITOUCH
#define HX_TP_PROC_2T2R //if enable, Need to check "HX_2T2R_Addr"
                        //and "HX_2T2R_en_setting" with project FW eng.
#endif
//===========Himax Option function=============
//#define HX_RST_PIN_FUNC
//#define HX_LOADIN_CONFIG
//#define HX_AUTO_UPDATE_FW
//#define HX_AUTO_UPDATE_CONFIG        //if enable HX_AUTO_UPDATE_CONFIG, need to disable HX_LOADIN_CONFIG
//#define HX_SMART_WAKEUP
//#define HX_PALM_REPORT
//#define HX_CHECK_CRC_AP
#define HX_TIME_TELLING
//#define HX_ESD_WORKAROUND
//#define HX_CHIP_STATUS_MONITOR        //for ESD 2nd solution,default off

#define HX_85XX_A_SERIES_PWON        1
#define HX_85XX_B_SERIES_PWON        2
#define HX_85XX_C_SERIES_PWON        3
#define HX_85XX_D_SERIES_PWON        4
#define HX_85XX_E_SERIES_PWON        5
#define HX_85XX_ES_SERIES_PWON        6

#define HX_TP_BIN_CHECKSUM_SW        1
#define HX_TP_BIN_CHECKSUM_HW        2
#define HX_TP_BIN_CHECKSUM_CRC        3

#define HX_KEY_MAX_COUNT             4
#define DEFAULT_RETRY_CNT            10

#define HX_VKEY_0   KEY_BACK
#define HX_VKEY_1   KEY_HOME
#define HX_VKEY_2   KEY_RESERVED
#define HX_VKEY_3   KEY_RESERVED
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}

#define SHIFTBITS 5
#define FLASH_SIZE 32768

struct himax_virtual_key {
    int index;
    int keycode;
    int x_range_min;
    int x_range_max;
    int y_range_min;
    int y_range_max;
};

struct himax_config {
    uint8_t  default_cfg;
    uint8_t  sensor_id;
    uint8_t  fw_ver_main;
    uint8_t  fw_ver_minor;
    uint16_t length;
    uint32_t tw_x_min;
    uint32_t tw_x_max;
    uint32_t tw_y_min;
    uint32_t tw_y_max;
    uint32_t pl_x_min;
    uint32_t pl_x_max;
    uint32_t pl_y_min;
    uint32_t pl_y_max;
    uint8_t c1[11];
    uint8_t c2[11];
    uint8_t c3[11];
    uint8_t c4[11];
    uint8_t c5[11];
    uint8_t c6[11];
    uint8_t c7[11];
    uint8_t c8[11];
    uint8_t c9[11];
    uint8_t c10[11];
    uint8_t c11[11];
    uint8_t c12[11];
    uint8_t c13[11];
    uint8_t c14[11];
    uint8_t c15[11];
    uint8_t c16[11];
    uint8_t c17[11];
    uint8_t c18[17];
    uint8_t c19[15];
    uint8_t c20[5];
    uint8_t c21[11];
    uint8_t c22[4];
    uint8_t c23[3];
    uint8_t c24[3];
    uint8_t c25[4];
    uint8_t c26[2];
    uint8_t c27[2];
    uint8_t c28[2];
    uint8_t c29[2];
    uint8_t c30[2];
    uint8_t c31[2];
    uint8_t c32[2];
    uint8_t c33[2];
    uint8_t c34[2];
    uint8_t c35[3];
    uint8_t c36[5];
    uint8_t c37[5];
    uint8_t c38[9];
    uint8_t c39[14];
    uint8_t c40[159];
    uint8_t c41[99];
};

struct himax_ts_data {
    bool suspended;
    atomic_t suspend_mode;
    uint8_t x_channel;
    uint8_t y_channel;
    uint8_t useScreenRes;
    uint8_t diag_command;
    uint8_t vendor_fw_ver_H;
    uint8_t vendor_fw_ver_L;
    uint8_t vendor_config_ver;
    uint8_t vendor_sensor_id;

    uint8_t protocol_type;
    uint8_t first_pressed;
    uint8_t coord_data_size;
    uint8_t area_data_size;
    uint8_t raw_data_frame_size;
    uint8_t raw_data_nframes;
    uint8_t nFinger_support;
    uint8_t irq_enabled;
    uint8_t diag_self[50];

    uint16_t finger_pressed;
    uint16_t last_slot;
    uint16_t pre_finger_mask;

    uint32_t debug_log_level;
    uint32_t widthFactor;
    uint32_t heightFactor;
    uint32_t tw_x_min;
    uint32_t tw_x_max;
    uint32_t tw_y_min;
    uint32_t tw_y_max;
    uint32_t pl_x_min;
    uint32_t pl_x_max;
    uint32_t pl_y_min;
    uint32_t pl_y_max;

    int use_irq;
    int (*power)(int on);
    int pre_finger_data[10][2];

    struct device *dev;
    struct workqueue_struct *himax_wq;
    struct work_struct work;
    struct input_dev *input_dev;
    struct hrtimer timer;
    struct i2c_client *client;
    struct himax_i2c_platform_data *pdata;
    struct himax_virtual_key *button;
    struct wake_lock ts_flash_wake_lock;

#ifdef CONFIG_FB
    struct notifier_block fb_notif;
    struct workqueue_struct *himax_att_wq;
    struct delayed_work work_att;
#endif
#ifdef HX_CHIP_STATUS_MONITOR
    struct workqueue_struct *himax_chip_monitor_wq;
    struct delayed_work himax_chip_monitor;
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
    struct workqueue_struct             *flash_wq;
    struct work_struct                     flash_work;
#endif
#ifdef HX_RST_PIN_FUNC
    int rst_gpio;
#endif
#ifdef HX_SMART_WAKEUP
    uint8_t SMWP_enable;
    struct wake_lock ts_SMWP_wake_lock;
#endif
#ifdef HX_TIME_TELLING
    int sleepmode;
    struct workqueue_struct *himax_sleepmode_wq;
    struct work_struct sleepmode_work;
#endif
    const struct firmware *fw;
    u8 *fw_data_start;
    uint32_t fw_size;
};

void himax_timetelling_detection(int supplymode);

#endif
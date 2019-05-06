/* Raydium TouchScreen driver.
 *
 * Copyright (c) 2010  Raydium tech Ltd.
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
#ifndef __LINUX_RAYDIUM_H
#define __LINUX_RAYDIUM_H

#define RAYDIUM_NAME "raydium_ts"

#define COORDS_ARR_SIZE    4
#define I2C_VTG_MIN_UV    1800000
#define I2C_VTG_MAX_UV    1800000

#define RAYDIUM_VER     0x0010


#if defined(CONFIG_TOUCHSCREEN_RM_TS)
/* IC timing control arguments */
#define RAYDIUM_POWERON_DELAY_USEC    500
#define RAYDIUM_RESET_INTERVAL_MSEC   5
#define RAYDIUM_RESET_RESTORE_USEC    200
#define RAYDIUM_RESET_DELAY_MSEC      50

/* I2C bus slave address(ID) */
#define RAYDIUM_I2C_EID    (0x5A)
#define RAYDIUM_I2C_NID    (0x39)

/* I2C R/W configuration literal */
#define RAYDIUM_I2C_WRITE       I2C_SMBUS_WRITE
#define RAYDIUM_I2C_READ        I2C_SMBUS_READ
#define SYN_I2C_RETRY_TIMES     2
#define MAX_WRITE_PACKET_SIZE   64
#define MAX_READ_PACKET_SIZE    64

/* PDA address and bit definition*/
#define RAYDIUM_READ_FT_DATA_CMD        0x2000019C
#define RAYDIUM_GESTURE_STATE_CMD       0x200005F4
// 1byte, disable:0x00 ; enable:0x20
#define RAYDIUM_GESTURE_DISABLE         0x00
#define RAYDIUM_GESTURE_ENABLE          0x20
#define RAYDIUM_GESTURE_RESULT_CMD      0x200005F0
// 4bytes, [0]:ready ; [1]:type ; [2]:direction
#define RAYDIUM_CHECK_I2C_CMD           0x500009BC
#define RAYDIUM_PDA2_CTRL_CMD           0x50000628
#define RAYDIM_ENABLE_PDA2              0x04

/* PDA literal */
#define MASK_8BIT    0xFF
#define RAYDIUM_I2C_PDA_ADDRESS_LENGTH    4

#define PDA_MODE     1
#define PDA2_MODE    2

#define RAYDIUM_I2C_PDA_MODE_DISABLE      0x00
#define RAYDIUM_I2C_PDA_MODE_ENABLE       0x80
#define RAYDIUM_I2C_PDA_MODE_WORD_MODE    0x40
// Using byte mode due to data might be not word-aligment
#define RAYDIUM_I2C_PDA_2_MODE_DISABLE    0x20

#define RAYDIUM_PALM_MODE_DISABLE    0x00
#define RAYDIUM_PALM_MODE_ENABLE     0x01

#define RAYDIUM_TEST_FW	0x80
#define RAYDIUM_TEST_PARA	0x40
#define RAYDIUM_BOOTLOADER	0x20
#define RAYDIUM_FIRMWARE	0x10
#define RAYDIUM_PARA		0x08
#define RAYDIUM_COMP		0x04
#define RAYDIUM_BASELINE	0x02
#define RAYDIUM_INIT		0x01

#define FAIL          0
#define ERROR        -1
#define SUCCESS       1

#define DISABLE       0
#define ENABLE        1

/* PDA2 setting */
#define MAX_PAGE_AMOUNT    11	// Page 0 ~ Page A

/* PDA2 address and setting definition*/
#define RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR    0x00	// only in Page 0
#define RAYDIUM_PDA2_TCH_RPT_ADDR           0x01	// only in Page 0
#define RAYDIUM_PDA2_HOST_CMD_ADDR          0x02	// only in Page 0
#define RAYDIUM_PDA2_PALM_AREA_ADDR         0x03	// only in Page 0
#define RAYDIUM_PDA2_GESTURE_RPT_ADDR       0x04	// only in Page 0
#define RAYDIUM_PDA2_PALM_STATUS_ADDR       0x05	// only in Page 0
#define RAYDIUM_PDA2_FW_VERSION_ADDR        0x06	// only in Page 0
#define RAYDIUM_PDA2_PANEL_VERSION_ADDR     0x07	// only in Page 0
#define RAYDIUM_PDA2_DISPLAY_MODE_ADDR      0x08	// only in Page 0
#define RAYDIUM_PDA2_PDA_CFG_ADDR           0x09	// only in Page 0
#define RAYDIUM_PDA2_PAGE_ADDR              0x0A
// Page 0 ~ Page 9 will be directed to Page 0
#define RAYDIUM_PDA2_PAGE_0                 0x00
#define RAYDIUM_PDA2_ENABLE_PDA             0x0A
// temporary switch to PDA once
#define RAYDIUM_PDA2_2_PDA                  (MAX_PAGE_AMOUNT + 2)
// permanently switch to PDA mode

/* Raydium host cmd */
#define RAYDIUM_HOST_CMD_NO_OP              0x00
#define RAYDIUM_HOST_CMD_PWR_SLEEP          0x30
#define RAYDIUM_HOST_CMD_CALIBRATION        0x5C
#define RAYDIUM_HOST_CMD_TP_MODE            0x60
#define RAYDIUM_HOST_CMD_FT_MODE            0x61

/* PDA2 literal */
#define RAYDIUM_I2C_PDA2_PAGE_LENGTH        2

// entry byte + target page byte

#define NEW_PALM			1

#define ENABLE_ESD_CHECK		0
/* Touch report */
#define MAX_TOUCH_NUM                 2


#define MAX_REPORT_PACKAGE_SIZE        35
#define MAX_TCH_STATUS_PACKAGE_SIZE    4

#define MAX_GESTURERESULT_SIZE        4
#define PRESS_MAX                     0xFFFF
#define WIDTH_MAX                     0xFFFF
#define SHORT_HIGH_BYTE_SHIFT         8

/* FW update literal */
#define RAYDIUM_FW_BIN_PATH_LENGTH    256
#define RAYDIUM_FW_MAX_SIZE           0x6360	// FW + PARA
/* FT APK literal */
#define RAYDIUM_HOST_CMD_POS    0x00
#define RAYDIUM_FT_CMD_POS      0x01
#define RAYDIUM_FT_CMD_LENGTH   0x02

/* FT APK data type */
#define RAYDIUM_FT_UPDATE    0x01

/*Raydium system flag*/
#define RAYDIUM_INTERRUPT_FLAG     0x01
#define RAYDIUM_ENGINEER_MODE    0x02

/* define display mode */
#define ACTIVE_MODE     0x00
#define AMBIENT_MODE    0x01
#define SLEEP_MODE      0x02

/* Enable sysfs */
#define CONFIG_RM_SYSFS_DEBUG

/* Gesture switch */
#define GESTURE_EN

#define MSM_NEW_VER

#define PINCTRL_STATE_ACTIVE     "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND    "pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE    "pmx_ts_release"

#endif

#endif	/*__LINUX_RAYDIUM_H*/

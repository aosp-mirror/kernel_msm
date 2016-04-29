/* Copyright (c) 2016,  HUAWEI TECHNOLOGIES CO., LTD.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IDTP9220_H__
#define __IDTP9220_H__

#define pr_fmt(fmt) "IDTP9220 %s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/pinctrl/consumer.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

struct idtp9220_packet_t{        // write to structure at SRAM address 0x0400
  u16 status;           // Read/Write by both 9220 and 9220 host
  u16 startAddr;        // OTP image address of the current packet
  u16 codeLength;       // The size of the OTP image data in the current packet
  u16 dataChksum;       // Checksum of the current packet
  u8  dataBuf[128];     // OTP image data of the current packet
};

union idtp9220_interactive_data{
    bool   result;
    u8     strval[2];
    u16    shortval;
};

enum idtp9220_request_type_need_rx {
    SET_LDO_ENABLE,
    SET_VOUT_VOLTAGE,
    SET_COUT_CURRENT,
    SET_OPER_FREQ,
    GET_OPER_FREQ,
    GET_CHIP_INFO,
    IS_LDO_ENABLED,
    IS_RX_FW_BURNED,
    DO_BURN_RX_FW,
    DO_VERIFY_RX_FW,

    /* the follow need tx return data */
    GET_TX_HARDWARE_VERSION,
    GET_TX_TEMP,
    GET_TX_SOFTWARE_VERSION,
    GET_TX_VIN,
};

enum idtp9220_request_type_need_tx {
    GET_TX_HARDWARE_VERSION_COMMAND = 1,
    GET_TX_TEMP_COMMAND,
    SET_TX_OPER_FREQ,
    /*GET_TX_OPER_FREQ,*/
    GET_TX_SOFTWARE_VERSION_COMMAND = 5,
    GET_TX_VIN_COMMAND = 7,
};

struct idtp9220_receiver {
    struct i2c_client                           *client;
    struct device                               *dev;
    struct mutex                                read_write_lock;
    struct mutex                                service_request_lock;
    int                                         wireless_int_gpio;
    int                                         mask_wireless_int_gpio;

    /* power supply */
    struct power_supply                         *batt_psy;

    /* wait tx return data */
    struct semaphore                            tx_send_data_int;

    /* dynamatic adjust vout */
    struct delayed_work                         adjust_vout_work;
    struct timespec                             last_set_vout_time;
    struct timespec                             resume_time;

    /* process interupt event */
    struct work_struct                          process_intr_work;

    /* using default vout */
    bool                                        using_default_vout_flag;

    /* burn rx firmware */
    struct idtp9220_packet_t                    burn_packet;
};

/* Mask/Bit helpers */
#define _IDTP9220_MASK(BITS, POS) \
    ((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define IDTP9220_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
        _IDTP9220_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
                (RIGHT_BIT_POS))

/* Globle paremeters */
#define WAIT_TX_RETURN_DATA_TIMEOUT_ERR         1
#define TX_RETURN_DATA_IS_NOT_DEMAND_ERR        2
#define IDTP9220_I2C_READ_BYTES                 1
#define IDTP9220_DELAY_MS_MAX                   5000
#define IDTP9220_VOUT_CHECK_PERIOD_MS           180000
#define IDTP9220_VBAT_DEFAULT_MV                3600
#define IDTP9220_VOUT_DEFAULT_MV                4000
#define IDTP9220_VOUT_ADJUST_STEP_MV            500


/* Vout config parameters */
#define IDT9200_VOUT_STEP_MV                    100
#define IDT9200_VOUT_MIN_MV                     3500
#define IDT9200_VOUT_MAX_MV                     5000

/* Freq config parameters */
#define IDT9200_FREQ_MIN_KHZ                    350
#define IDT9200_FREQ_MAX_KHZ                    600

/* Cout config pareameters */
#define IDT9200_COUT_STEP_MA                    100
#define IDT9200_COUT_MIN_MA                     100
#define IDT9200_COUT_MAX_MA                     1300

/* Chip ID Register */
#define CHIP_ID_L_REG                           0x0000
#define CHIP_ID_H_REG                           0x0001
#define CHIP_ID_VALUE                           0x9220

/* Chip Revision and Font Register */
#define CHIP_REV_FONT_REG                       0x0002
#define CHIP_REV_MASK_SHIFT                     4

/* Customer ID Register */
#define CTM_ID_REG                              0x0003

/* OTP Firmware Major Revision Registers */
#define RX_FW_MAJOR_REV_L_REG                   0x0004
#define RX_FW_MAJOR_REV_H_REG                   0x0005

/*OTP Firmware Minor Revision Registers */
#define RX_FW_MINOR_REV_L_REG                   0x0006
#define RX_FW_MINOR_REV_H_REG                   0x0007

/* Status Register */
#define STATUS_L_REG                            0x0034
#define STATUS_VOUT_ON                          (1 << 7)
#define STATUS_VOUT_OFF                         (1 << 6)
#define STATUS_TX_DATA_RECV                     (1 << 4)

/* Interrupt Registers */
#define INTR_L_REG                              0x0036
#define TX_DATA_RECV                            (1 << 4)
#define INTR_H_REG                              0x0037

/* Vout ADC Value Registers, ADC_Vout_L (0x3C), ADC_Vout_H (0x3D) */
#define ADC_VOUT_L_REG                          0x003C
#define ADC_VOUT_H_REG                          0x003D
#define ADJUST_METE_MV                          35

/* Vout Set Register, Vout_Set */
#define VOUT_SET_REG                            0x003E
#define RX_LOUT_L_REG                           0x0044
#define RX_LOUT_H_REG                           0x0045

/* Operating Frequency in Registers*/
#define OP_FREQ_L_REG                           0x0048
#define OP_FREQ_H_REG                           0x0049

/* command register */
#define COMMAND_REG                             0x004E
#define TOGGLE_LDO_ON_OFF_MASK                  IDTP9220_MASK(1,1)
#define TOGGLE_LDO_ON_OFF_MASK_SHIFT            1
#define SEND_RX_DATA_MASK                       IDTP9220_MASK(0,0)
#define SEND_RX_DATA_MASK_SHIFT                 0
#define CLEAR_INTERRUPT_MASK                    IDTP9220_MASK(5,5)
#define CLEAR_INTERRUPT_MASK_SHIFT              5

/* RX Data Command Register */
#define RX_DATA_COMMAND_REG                     0x0051

/* RX Data Value Register */
#define RX_FREQ_DATA_L_REG                      0x0052
#define RX_FREQ_DATA_H_REG                      0x0053

/* Interrupt Clear Registers */
#define INT_CLEAR_L_REG                         0x0056
#define TX_DATA_RECV_CLEAR_MASK                 IDTP9220_MASK(4,4)
#define TX_DATA_RECV_CLEAR_MASK_SHIFT           4

/* TX Data Command Register */
#define TX_DATA_COMMAND_REG                     0x0059
enum{
    IS_TX_HARDWARE_VERSION_DATA = 1,
    IS_TX_TEMP_DATA,
    IS_SET_TX_FREQ_DATA,
    IS_GET_TX_FREQ_DATA,
    IS_TX_SOFTWARE_VERSIION_DATA,
    IS_TX_VIN = 7,
};

/* TX Data Value Register */
#define TX_DATA_01_REG                          0x005A
#define TX_DATA_02_REG                          0x005B

/* rx fw burn magic number */
#define RX_BURN_MAGIC_NUMBER_ADDR               0x3800
static const char burn_magic_number[] = {0xAA, 0xBB, 0xCC, 0xDD};

/* rx bootloader for download rx fw */
static const char rx_bootloader_data[] = {
  0x00, 0x04, 0x00, 0x20, 0x35, 0x01, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xFE, 0xE7, 0x00, 0x00, 0x80, 0x00, 0x00, 0xE0, 0x00, 0xBF, 0x40, 0x1E, 0xFC, 0xD2, 0x70, 0x47,
  0x00, 0xB5, 0x60, 0x4A, 0x60, 0x4B, 0x01, 0x70, 0x01, 0x20, 0xFF, 0xF7, 0xF3, 0xFF, 0x52, 0x1E,
  0x02, 0xD0, 0x18, 0x8B, 0x00, 0x06, 0xF7, 0xD4, 0x00, 0xBD, 0xF7, 0xB5, 0x05, 0x46, 0x5B, 0x48,
  0x81, 0xB0, 0x00, 0x21, 0x94, 0x46, 0x81, 0x81, 0x57, 0x48, 0x31, 0x21, 0x01, 0x80, 0x04, 0x21,
  0x81, 0x80, 0x06, 0x21, 0x01, 0x82, 0x28, 0x20, 0xFF, 0xF7, 0xDC, 0xFF, 0x00, 0x24, 0x0D, 0xE0,
  0x02, 0x99, 0x28, 0x5D, 0x09, 0x5D, 0x02, 0x46, 0x8A, 0x43, 0x01, 0xD0, 0x10, 0x20, 0x3F, 0xE0,
  0x81, 0x43, 0x02, 0xD0, 0x28, 0x19, 0xFF, 0xF7, 0xD3, 0xFF, 0x64, 0x1C, 0x64, 0x45, 0xEF, 0xD3,
  0x49, 0x48, 0x36, 0x21, 0x01, 0x82, 0x00, 0x24, 0x2F, 0xE0, 0x02, 0x98, 0x00, 0x27, 0x06, 0x5D,
  0x28, 0x19, 0x00, 0x90, 0x44, 0x4A, 0x08, 0x20, 0x90, 0x80, 0x02, 0x20, 0xFF, 0xF7, 0xBA, 0xFF,
  0x28, 0x5D, 0x33, 0x46, 0x83, 0x43, 0x16, 0xD0, 0x3F, 0x49, 0x04, 0x20, 0x88, 0x80, 0x02, 0x20,
  0xFF, 0xF7, 0xB0, 0xFF, 0x19, 0x46, 0x00, 0x98, 0xFF, 0xF7, 0xB2, 0xFF, 0x3A, 0x49, 0x0F, 0x20,
  0x88, 0x80, 0x02, 0x20, 0xFF, 0xF7, 0xA6, 0xFF, 0x28, 0x5D, 0xB0, 0x42, 0x03, 0xD0, 0x7F, 0x1C,
  0x0A, 0x2F, 0xDF, 0xD3, 0x01, 0xE0, 0x0A, 0x2F, 0x06, 0xD3, 0x35, 0x48, 0x29, 0x19, 0x41, 0x80,
  0x29, 0x5D, 0xC1, 0x80, 0x04, 0x20, 0x03, 0xE0, 0x64, 0x1C, 0x64, 0x45, 0xCD, 0xD3, 0x02, 0x20,
  0x2D, 0x49, 0x11, 0x22, 0x0A, 0x80, 0x04, 0x22, 0x8A, 0x80, 0x2C, 0x49, 0xFF, 0x22, 0x8A, 0x81,
  0x04, 0xB0, 0xF0, 0xBD, 0x2C, 0x49, 0x2B, 0x48, 0x08, 0x60, 0x2C, 0x48, 0x00, 0x23, 0x83, 0x81,
  0x2A, 0x49, 0x20, 0x39, 0x8B, 0x83, 0x03, 0x80, 0x24, 0x48, 0x5A, 0x21, 0x40, 0x38, 0x01, 0x80,
  0x81, 0x15, 0x81, 0x80, 0x0B, 0x21, 0x01, 0x81, 0x25, 0x49, 0x81, 0x81, 0x14, 0x20, 0xFF, 0xF7,
  0x71, 0xFF, 0x24, 0x4A, 0x01, 0x20, 0x10, 0x80, 0x02, 0x20, 0xFF, 0xF7, 0x6B, 0xFF, 0x8D, 0x20,
  0x10, 0x80, 0x93, 0x80, 0xFF, 0x20, 0x90, 0x82, 0x03, 0x20, 0x00, 0x02, 0x10, 0x82, 0xFC, 0x20,
  0x90, 0x83, 0x1C, 0x49, 0x95, 0x20, 0x20, 0x31, 0x08, 0x80, 0x15, 0x4D, 0x2B, 0x80, 0x28, 0x88,
  0x2C, 0x46, 0x01, 0x28, 0xFB, 0xD1, 0x61, 0x88, 0x80, 0x03, 0xA2, 0x88, 0x08, 0x18, 0x51, 0x18,
  0x8B, 0xB2, 0x00, 0x21, 0x04, 0xE0, 0x0E, 0x19, 0x36, 0x7A, 0xF3, 0x18, 0x9B, 0xB2, 0x49, 0x1C,
  0x8A, 0x42, 0xF8, 0xD8, 0xE1, 0x88, 0x99, 0x42, 0x01, 0xD0, 0x08, 0x20, 0x08, 0xE0, 0x00, 0x2A,
  0x05, 0xD0, 0x07, 0x49, 0x08, 0x31, 0xFF, 0xF7, 0x50, 0xFF, 0x20, 0x80, 0xDF, 0xE7, 0x02, 0x20,
  0x28, 0x80, 0xDC, 0xE7, 0x10, 0x27, 0x00, 0x00, 0x00, 0x5C, 0x00, 0x40, 0x40, 0x30, 0x00, 0x40,
  0x00, 0x04, 0x00, 0x20, 0xFF, 0x0F, 0x00, 0x00, 0x80, 0xE1, 0x00, 0xE0, 0x20, 0x6C, 0x00, 0x40,
  0x04, 0x1D, 0x00, 0x00, 0x00, 0x64, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* idtp9220 rx fw data */
#include "idtp9220-receiver-firmware.h"
#endif

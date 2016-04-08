/******************************************************************************
 * @file parameter.h
 *
 * parameter.h
 *
 * @version 0.0.1
 * @authors btok
 *
 *****************************************************************************//*
 * Copyright (2014), Cypress Semiconductor Corporation. All rights reserved.
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
 *****************************************************************************/

#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include <linux/fs.h>

#define MAX_STRING_LENGTH    128

enum parameter_id {
    TX_NUM,
    RX_NUM,
    BUTTON_NUM,
    SCANNING_MODE_BUTTON,
    TX_PERIOD_MUTUAL,
    TX_PERIOD_SELF,
    TX_PERIOD_BTN_MUTUAL,
    TX_PERIOD_BTN_SELF,
    MTX_ORDER,
    VDDA_MODE,
    SCALING_FACTOR_MUTUAL,
    SCALING_FACTOR_SELF,
    SCALING_FACTOR_BUTTON_MUTUAL,
    SCALING_FACTOR_BUTTON_SELF,
    BALANCING_TARGET_MUTUAL,
    BALANCING_TARGET_SELF,
    BALANCING_TARGET_BUTTON_MUTUAL,
    BALANCING_TARGET_BUTTON_SELF,
    GIDAC_MULT,
    RXDAC,
    REF_SCALE,
    ACT_LFT_EN,
    BL_H20_RJCT,
    TX_PUMP_VOLTAGE,
    SENSOR_ASSIGNMENT,
    GIDAC_LSB_CONFIG,
    INT_CAP_MUTUAL,
    INT_CAP_SELF,
    INT_CAP_BUTTON_MUTUAL,
    INT_CAP_BUTTON_SELF,
    TX_VOLTAGE_MUTUAL,
    PARAMETER_ID_MAX,
};

enum parameter_type {
    INTEGER,
    FLOAT,
    STRING,
    PARAMETER_TYPE_MAX,
};

union parameter_value {
    int32_t integer;
    int flt;
    char string[MAX_STRING_LENGTH];
};

/* SCANNING_MODE_BUTTON enumerated values */
#define SCANNING_MODE_BUTTON_SELF    "Self Capacitance"
#define SCANNING_MODE_BUTTON_MUTUAL    "Mutual Capacitance"
#define SCANNING_MODE_BUTTON_HYBRID    "Hybrid"

/* VDDA_MODE enumerated values */
#define VDDA_MODE_PUMP            "Pump Mode"
#define VDDA_MODE_BYPASS        "Bypass Mode"

/* ACT_LFT_EN enumerated values */
#define ACT_LFT_EN_DISABLED        "Disabled"
#define ACT_LFT_EN_ENABLED        "Enabled"
#define RX_IS_Y               "RX = Y; TX = X"
#define RX_IS_X               "RX = X; TX = Y"

/* BL_H20_RJCT enumerated values */
#define BL_H20_RJCT_DISABLED        "Disabled"
#define BL_H20_RJCT_ENABLED        "Enabled"

extern int parameter_init(struct file *file);

extern int parameter_get_info(enum parameter_id id, uint16_t *address,
        uint16_t *size, uint32_t *mask, enum parameter_type *type);

extern int parameter_get_enumerated_value(enum parameter_id id, int index,
        const union parameter_value **value);

extern void parameter_exit(void);

#endif /* _PARAMETER_H_ */

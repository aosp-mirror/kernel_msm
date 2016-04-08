/******************************************************************************
 * @file configuration.h
 *
 * configuration.h
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

#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include <linux/fs.h>


#define MAX_BUTTONS		4
#define MAX_SENSORS		1024
#define MAX_TX_SENSORS       128
#define MAX_RX_SENSORS       128

/* Multiply by 2 for double (min, max) values */
#define TABLE_BUTTON_MAX_SIZE	(MAX_BUTTONS * 2)
#define TABLE_SENSOR_MAX_SIZE	(MAX_SENSORS * 2)
#define TABLE_TX_MAX_SIZE (MAX_TX_SENSORS*2)
#define TABLE_RX_MAX_SIZE (MAX_RX_SENSORS*2)

struct configuration {
    int family_type;
    //int CM_EXCLUDING_COL_EDGE;
    int cm_excluding_col_edge;
    int cm_excluding_row_edge;
    //int CM_GRADIENT_CHECK_COL[];
    int cm_gradient_check_col[MAX_RX_SENSORS];
    int cm_gradient_check_row[MAX_RX_SENSORS];
    int cm_gradient_check_col_size;
    int cm_gradient_check_row_size;
    int cm_range_limit_row;
    int cm_range_limit_col;
    int cm_min_limit_cal;
    int cm_max_limit_cal;
    int cm_max_delta_sensor_percent;
    int cm_max_delta_button_percent;
    int min_sensor_rx;
    int max_sensor_rx;
    int min_sensor_tx;
    int max_sensor_tx;
    int min_button;
    int max_button;
    int max_delta_sensor;
    int cp_max_delta_sensor_rx_percent;
    int cp_max_delta_sensor_tx_percent;
    int cm_min_max_table_button[TABLE_BUTTON_MAX_SIZE];
    int cm_min_max_table_sensor[TABLE_SENSOR_MAX_SIZE];
    int cp_min_max_table_rx[TABLE_RX_MAX_SIZE];
    int cp_min_max_table_tx[TABLE_TX_MAX_SIZE];
    int cm_min_max_table_button_size;
    int cm_min_max_table_sensor_size;
    int cp_min_max_table_rx_size;
    int cp_min_max_table_tx_size;
    int cp_max_delta_button_percent;
};

int configuration_get(struct file *file, struct configuration *configuration);
#endif /* _CONFIGURATION_H_ */

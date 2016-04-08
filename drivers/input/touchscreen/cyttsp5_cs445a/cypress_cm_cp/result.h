/******************************************************************************
 * @file result.h
 *
 * result.h
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

#ifndef _RESULT_H_
#define _RESULT_H_
#include <linux/fs.h>
#include "cm_cp_test.h"
#include "configuration.h"
struct result {
    uint32_t tx_num;
    uint32_t rx_num;
    uint32_t button_num;
    uint32_t sensor_assignment;
    uint32_t config_ver;
    uint32_t revision_ctrl;
    uint32_t device_id_high;
    uint32_t device_id_low;
    bool cm_test_run;
    bool cp_test_run;
    /* Sensor Cm validation */
    bool cm_test_pass;
    bool cm_sensor_validation_pass;
    bool cm_sensor_row_delta_pass;
    bool cm_sensor_col_delta_pass;
    bool cm_sensor_calibration_pass;
    bool cm_sensor_delta_pass;
    bool cm_button_validation_pass;
    bool cm_button_delta_pass;
    int *cm_sensor_data;
    int32_t *cm_sensor_raw_data;
    int *cm_sensor_column_delta;
    int *cm_sensor_row_delta;
    int cm_sensor_calibration;
    int cm_sensor_average;
    int cm_sensor_delta;
    int *cm_button_data;
    int32_t *cm_button_raw_data;
    int cm_button_calibration;
    int cm_button_average;
    int cm_button_delta;
    struct gd_sensor*cm_gradient_col;
    struct gd_sensor*cm_gradient_row;
    /* Sensor Cp validation */
    bool cp_test_pass;
    bool cp_sensor_delta_pass;
    bool cp_sensor_rx_delta_pass;
    bool cp_sensor_tx_delta_pass;
    bool cp_sensor_average_pass;
    bool cp_button_delta_pass;
    bool cp_button_average_pass;
    bool cp_rx_validation_pass;
    bool cp_tx_validation_pass;
    int *cp_sensor_rx_data;
    int *cp_sensor_tx_data;
    int32_t *cp_sensor_rx_raw_data;
    int32_t *cp_sensor_tx_raw_data;
    int cp_sensor_rx_delta;
    int cp_sensor_tx_delta;
    int cp_sensor_rx_average;
    int cp_sensor_tx_average;
    int cp_sensor_rx_calibration;
    int cp_sensor_tx_calibration;
    int *cp_button_data;
    int32_t *cp_button_raw_data;
    int cp_button_delta;
    int cp_button_average;
    bool short_test_pass;
    bool test_summary;
};

extern int result_save(struct seq_file *file, struct configuration *configuration,
        struct result *result);
#endif /* _RESULT_H_ */

/******************************************************************************
 * @file result.c
 *
 * result.c
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

#include "result.h"
#include "version.h"
#include "debug.h"
#include "../cyttsp5_regs.h"
#include "../cyttsp5_core.h"

#include <linux/fs.h>


#define SENSOR_CM_VALIDATION        "Sensor Cm Validation"
#define SELFCAP_CALIBRATION_CHECK    "Self-cap Calibration Check"

#define PASS    "PASS"
#define FAIL    "FAIL"

#define PRINT_RESULT(index ,value, file) \
do {\
    if(value == 0){\
        seq_printf(file, "%dF-", index);\
    }else{\
        seq_printf(file, "%dP-", index);\
    }\
}while(0)

static int print_cm_info(struct seq_file *file, struct configuration *configuration,
        struct result *result)
{
    int ret = 0;
    int i, j;

    if ( !result || ! configuration) {
        ret = -EINVAL;
        tp_log_err("%s, param invalid\n", __func__);
        goto exit;
    }
    /*print cm_sensor_data*/
    if(result->cm_sensor_raw_data) {
        seq_printf(file, "cm_sensor_raw_data:\n");
        for(i = 0 ; i < result->tx_num; i++) {
            for(j = 0 ; j < result->rx_num; j++) {
                seq_printf(file, "%6d", result->cm_sensor_raw_data[i*result->rx_num + j]);
            }
            seq_printf(file, "\n");
        }
    }

    if(result->cm_sensor_data) {
        seq_printf(file, "cm_sensor_data:\n");
        for(i = 0 ; i < result->tx_num; i++) {
            for(j = 0 ; j < result->rx_num; j++) {
                seq_printf(file, "%6d", result->cm_sensor_data[i*result->rx_num + j]);
            }
            seq_printf(file, "\n");
        }
    }

    if(result->cm_gradient_col) {
        seq_printf(file, "cm_gradient_col:\n");
        for (i = 0; i < configuration->cm_gradient_check_col_size; i++)
           seq_printf(file, "%6d", result->cm_gradient_col[i].gradient_val);
        seq_printf(file, "\n");
    }

    if(result->cm_gradient_row) {
        seq_printf(file, "cm_gradient_row:\n");
        for (i = 0; i < configuration->cm_gradient_check_row_size; i++)
           seq_printf(file, "%6d", result->cm_gradient_row[i].gradient_val);
        seq_printf(file, "\n");
    }
exit:
    return ret;
}

static int print_cp_info(struct seq_file *file, struct configuration *configuration,
        struct result *result)
{
    int ret = 0;
    int i;

    if ( !result || ! configuration) {
        ret = -EINVAL;
        tp_log_err("%s, param invalid\n", __func__);
        goto exit;
    }
    /*print cp_sensor_data*/
    if(result->cp_sensor_rx_raw_data) {
        seq_printf(file, "cp_sensor_rx_raw_data:\n");
        for(i = 0 ; i < result->rx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_rx_raw_data[i]);
        seq_printf(file, "\n");
    }

    if(result->cp_sensor_rx_data) {
        seq_printf(file, "cp_sensor_rx_data:\n");
        for(i = 0 ; i < result->rx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_rx_data[i]);
        seq_printf(file, "\n");
    }

    if(result->cp_sensor_tx_raw_data) {
        seq_printf(file, "cp_sensor_tx_raw_data:\n");
        for(i = 0 ; i < result->tx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_tx_raw_data[i]);
        seq_printf(file, "\n");
    }

    if(result->cp_sensor_tx_data) {
        seq_printf(file, "cp_sensor_tx_data:\n");
        for(i = 0 ; i < result->tx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_tx_data[i]);
        seq_printf(file, "\n");
    }
exit:
    return ret;
}

int result_save(struct seq_file *file, struct configuration *configuration,
        struct result *result)
{
    int ret = 0;
    if(! configuration || !result) {
        tp_log_err("%s, param invalid\n",__func__);
        ret = -EINVAL;
        goto exit;
    }
    seq_printf(file, "result:");
    PRINT_RESULT(0, result->test_summary, file);
    if(result->cm_test_run) {
        PRINT_RESULT(1, result->cm_test_pass, file);
        PRINT_RESULT(2, result->cm_sensor_validation_pass, file);
        PRINT_RESULT(3, result->cm_sensor_col_delta_pass, file);
        PRINT_RESULT(4, result->cm_sensor_row_delta_pass, file );
    }
    if(result->cp_test_run) {
        PRINT_RESULT(5, result->cp_test_pass, file );
        PRINT_RESULT(6, result->cp_rx_validation_pass, file);
        PRINT_RESULT(7, result->cp_tx_validation_pass, file);
    }
    seq_printf(file, "\n");
    seq_printf(file ,"rx_num:%d, tx_num:%d\n", result->rx_num, result->tx_num);
    seq_printf(file ,"button_num:%d, firmware_version:0x%x\n", result->button_num, result->config_ver);
    if(result->cm_test_run) {
        print_cm_info(file, configuration, result);
    }
    if(result->cp_test_run) {
        print_cp_info(file, configuration, result);
    }
exit:
    return ret;
}

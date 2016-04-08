/******************************************************************************
 * @file cm_cp_test.h
 *
 * cm_cp_test.h
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

#ifndef _CM_CP_TEST_H_
#define _CM_CP_TEST_H_

#include <linux/fs.h>
#define HID_DESCRIPTOR_REGISTER				0x0001

/* Runtime Parameters */
#define FORCE_SINGLE_TX					0x1F

#define IDAC_AND_RX_ATTENUATOR_CALIBRATION_DATA_ID	0x00
#define IDAC_AND_RX_ATTENUATOR_CALIBRATION_DATA_LENGTH	10

#define RX_ATTENUATOR_MUTUAL_INDEX			0
#define IDAC_MUTUAL_INDEX				1
#define RX_ATTENUATOR_SELF_RX_INDEX			2
#define IDAC_SELF_RX_INDEX				3
#define RX_ATTENUATOR_SELF_TX_INDEX			4
#define IDAC_SELF_TX_INDEX				5
#define RX_ATTENUATOR_BUTTON_MUTUAL_INDEX		6
#define IDAC_BUTTON_MUTUAL_INDEX			7
#define RX_ATTENUATOR_BUTTON_SELF_INDEX			8
#define IDAC_BUTTON_SELF_INDEX				9

#define MUTUAL_CAP_RAW_DATA_ID				0x00
#define MUTUAL_LOCAL_PWC_DATA_ID				0x00

#define SELF_CAP_RAW_DATA_ID				0x03
#define SELF_LOCAL_PWC_DATA_ID				0x01

#define BUTTONS_DATA_ID					0x09

#define BUTTONS_NUM_ELEMENTS				4
#define BUTTONS_RAW_DATA_INDEX				0

#define CONFIG_BLOCK_ID					0x00

#define ROW_SIZE					128

#define READ_LENGTH 					100


struct gd_sensor {
    uint16_t cm_min;
    uint16_t cm_max;
    int cm_ave;
    uint16_t cm_min_exclude_edge;
    uint16_t cm_max_exclude_edge;
    int cm_ave_exclude_edge;
    int gradient_val;
};

extern int cm_cp_test_run(char *device_path, struct file *parameter_file,
        struct file *configuration_file, struct seq_file *result_file,
        int vdda, bool run_cm_test, bool run_cp_test,
        bool *cm_test_pass, bool *cp_test_pass);
#endif /* _CM_CP_TEST_H_ */

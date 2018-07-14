/*
 * Copyright 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _CAM_SENSOR_UTIL_FATP_H_
#define _CAM_SENSOR_UTIL_FATP_H_

#include <linux/types.h>
#include <linux/device.h>

#define ATTR_SENSOR_PROBE_STATUS  0
#define ATTR_OIS_PROBE_STATUS     1
#define ATTR_EEPROM_DATA          2

/**
 * This API return serial number of rear camera sensor module.
 */
uint64_t get_rear_cam_sensor_module_sn(void);

/**
 * This API return serial number of nfov camera sensor module.
 */
uint64_t get_nfov_cam_sensor_module_sn(void);

/**
 * This API return infinite up focused position of nfov camera sensor module.
 */
uint16_t get_rear_cam_inf_up_focused_position(void);

/**
 * This API return infinite horizontal focused position of nfov camera sensor
 * module.
 */
uint16_t get_rear_cam_inf_hori_focused_position(void);

/**
 * This API return infinite down focused position of nfov camera sensor module.
 */
uint16_t get_rear_cam_inf_down_focused_position(void);

/**
 * This API return 10cm up focused position of nfov camera sensor module.
 */
uint16_t get_rear_cam_10cm_up_focused_position(void);

/**
 * This API return 10cm horizontal focused position of nfov camera sensor
 * module.
 */
uint16_t get_rear_cam_10cm_hori_focused_position(void);

/**
 * This API return 10cm down focused position of nfov camera sensor module.
 */
uint16_t get_rear_cam_10cm_down_focused_position(void);

/**
 * @saddr : slave address
 * @data  : eeprom data
 *
 * This API reads the eeprom data, parse it and export it to user space.
 */
void export_eeprom_data(uint32_t saddr, uint8_t *data);

/**
 * @sensor_id : sensor id
 *
 * This API set the corresponding sensor probe status to be true.
 */
void set_cam_sensor_probe(uint16_t sensor_id);

/**
 * This API set the ois probe status to be true.
 */
void set_cam_ois_probe(void);

/**
 * @attr_type : attribute type to be created
 * @kobj      : kobject
 *
 * This API create specific attribute corresponding to kobj.
 */
void create_file(int attr_type, struct kobject *kobj);

/**
 * @attr_type : attribute type to be created
 * @kobj      : kobject
 *
 * This API remove specific attribute corresponding to kobj.
 */
void remove_file(int attr_type, struct kobject *kobj);

#endif
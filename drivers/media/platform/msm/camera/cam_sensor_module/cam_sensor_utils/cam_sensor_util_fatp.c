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
#include "cam_sensor_util_fatp.h"
#include "cam_debug_util.h"
#include <linux/sysfs.h>


#define REAR_CAM_SLAVE_ADDR 0x7c
#define NFOV_CAM_SLAVE_ADDR 0xA2

#define REAR_CAM_SENSOR_ID  0x363
#define NFOV_CAM_SENSOR_ID  0x355

// Data stored in rear camera(lc898123f40) is stored in little endian order.
#define POS_R_SN_1          13
#define POS_R_SN_2          12
#define POS_R_SN_3          19
#define POS_R_SN_4          18
#define POS_R_SN_5          17
#define POS_R_SN_6          16
#define POS_R_SN_7          23
#define POS_R_SN_8          22
#define POS_R_INF_UP_1      1883
#define POS_R_INF_UP_2      1882
#define POS_R_INF_HORI_1    1881
#define POS_R_INF_HORI_2    1880
#define POS_R_INF_DOWN_1    1887
#define POS_R_INF_DOWN_2    1886
#define POS_R_10CM_UP_1     1885
#define POS_R_10CM_UP_2     1884
#define POS_R_10CM_HORI_1   1891
#define POS_R_10CM_HORI_2   1890
#define POS_R_10CM_DOWN_1   1889
#define POS_R_10CM_DOWN_2   1888

#define POS_NFOV_SN_1       14
#define POS_NFOV_SN_2       15
#define POS_NFOV_SN_3       16
#define POS_NFOV_SN_4       17
#define POS_NFOV_SN_5       18
#define POS_NFOV_SN_6       19
#define POS_NFOV_SN_7       20
#define POS_NFOV_SN_8       21

#define UINT64_BYTE_1       7 // MSB
#define UINT64_BYTE_2       6
#define UINT64_BYTE_3       5
#define UINT64_BYTE_4       4
#define UINT64_BYTE_5       3
#define UINT64_BYTE_6       2
#define UINT64_BYTE_7       1
#define UINT64_BYTE_8       0 // LSB

#define UINT16_BYTE_1       1 // MSB
#define UINT16_BYTE_2       0 // LSB

#define READ_TO_UINT64(data, data_pos, byte_num) \
	(((uint64_t)*(data + data_pos)) << byte_num * 8)
#define READ_TO_UINT16(data, data_pos, byte_num) \
	(((uint16_t)*(data + data_pos)) << byte_num * 8)

static uint64_t rear_cam_serial_number;
static uint64_t nfov_cam_serial_number;
static uint16_t rear_cam_inf_up_focused_position;
static uint16_t rear_cam_inf_hori_focused_position;
static uint16_t rear_cam_inf_down_focused_position;
static uint16_t rear_cam_10cm_up_focused_position;
static uint16_t rear_cam_10cm_hori_focused_position;
static uint16_t rear_cam_10cm_down_focused_position;

static uint8_t sensor_probe_status;
static uint8_t ois_probe_status;

uint64_t get_rear_cam_sensor_module_sn(void)
{
	return rear_cam_serial_number;
}

uint64_t get_nfov_cam_sensor_module_sn(void)
{
	return nfov_cam_serial_number;
}

uint16_t get_rear_cam_inf_up_focused_position(void)
{
	return rear_cam_inf_up_focused_position;
}

uint16_t get_rear_cam_inf_hori_focused_position(void)
{
	return rear_cam_inf_hori_focused_position;
}

uint16_t get_rear_cam_inf_down_focused_position(void)
{
	return rear_cam_inf_down_focused_position;
}

uint16_t get_rear_cam_10cm_up_focused_position(void)
{
	return rear_cam_10cm_up_focused_position;
}

uint16_t get_rear_cam_10cm_hori_focused_position(void)
{
	return rear_cam_10cm_hori_focused_position;
}

uint16_t get_rear_cam_10cm_down_focused_position(void)
{
	return rear_cam_10cm_down_focused_position;
}

void export_eeprom_data(uint32_t saddr, uint8_t *data)
{
	if (saddr == REAR_CAM_SLAVE_ADDR) {
		rear_cam_serial_number =
			READ_TO_UINT64(data, POS_R_SN_1, UINT64_BYTE_1) +
			READ_TO_UINT64(data, POS_R_SN_2, UINT64_BYTE_2) +
			READ_TO_UINT64(data, POS_R_SN_3, UINT64_BYTE_3) +
			READ_TO_UINT64(data, POS_R_SN_4, UINT64_BYTE_4) +
			READ_TO_UINT64(data, POS_R_SN_5, UINT64_BYTE_5) +
			READ_TO_UINT64(data, POS_R_SN_6, UINT64_BYTE_6) +
			READ_TO_UINT64(data, POS_R_SN_7, UINT64_BYTE_7) +
			READ_TO_UINT64(data, POS_R_SN_8, UINT64_BYTE_8);

		rear_cam_inf_up_focused_position =
			READ_TO_UINT16(data, POS_R_INF_UP_1, UINT16_BYTE_1) +
			READ_TO_UINT16(data, POS_R_INF_UP_2, UINT16_BYTE_2);

		rear_cam_inf_hori_focused_position =
			READ_TO_UINT16(data, POS_R_INF_HORI_1, UINT16_BYTE_1) +
			READ_TO_UINT16(data, POS_R_INF_HORI_2, UINT16_BYTE_2);

		rear_cam_inf_down_focused_position =
			READ_TO_UINT16(data, POS_R_INF_DOWN_1, UINT16_BYTE_1) +
			READ_TO_UINT16(data, POS_R_INF_DOWN_2, UINT16_BYTE_2);

		rear_cam_10cm_up_focused_position =
			READ_TO_UINT16(data, POS_R_10CM_UP_1, UINT16_BYTE_1) +
			READ_TO_UINT16(data, POS_R_10CM_UP_2, UINT16_BYTE_2);

		rear_cam_10cm_hori_focused_position =
			READ_TO_UINT16(data, POS_R_10CM_HORI_1, UINT16_BYTE_1) +
			READ_TO_UINT16(data, POS_R_10CM_HORI_2, UINT16_BYTE_2);

		rear_cam_10cm_down_focused_position =
			READ_TO_UINT16(data, POS_R_10CM_DOWN_1, UINT16_BYTE_1) +
			READ_TO_UINT16(data, POS_R_10CM_DOWN_2, UINT16_BYTE_2);
	} else if (saddr == NFOV_CAM_SLAVE_ADDR) {
		nfov_cam_serial_number =
			READ_TO_UINT64(data, POS_NFOV_SN_1, UINT64_BYTE_1) +
			READ_TO_UINT64(data, POS_NFOV_SN_2, UINT64_BYTE_2) +
			READ_TO_UINT64(data, POS_NFOV_SN_3, UINT64_BYTE_3) +
			READ_TO_UINT64(data, POS_NFOV_SN_4, UINT64_BYTE_4) +
			READ_TO_UINT64(data, POS_NFOV_SN_5, UINT64_BYTE_5) +
			READ_TO_UINT64(data, POS_NFOV_SN_6, UINT64_BYTE_6) +
			READ_TO_UINT64(data, POS_NFOV_SN_7, UINT64_BYTE_7) +
			READ_TO_UINT64(data, POS_NFOV_SN_8, UINT64_BYTE_8);
	}
}

void set_cam_sensor_probe(uint16_t sensor_id)
{
	if (sensor_id == REAR_CAM_SENSOR_ID) {
		sensor_probe_status += 1;
	} else if (sensor_id == NFOV_CAM_SENSOR_ID) {
		sensor_probe_status += 1 << 1;
	}
}

void set_cam_ois_probe(void)
{
	ois_probe_status = 1;
}

static ssize_t rear_cam_inf_up_focus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n",
		get_rear_cam_inf_up_focused_position());
}
static DEVICE_ATTR(rear_inf_up_focusd_position, 0644,
	rear_cam_inf_up_focus_show, NULL);

static ssize_t rear_cam_inf_hori_focus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n",
		get_rear_cam_inf_hori_focused_position());
}
static DEVICE_ATTR(rear_inf_hori_focusd_position, 0644,
	rear_cam_inf_hori_focus_show, NULL);

static ssize_t rear_cam_inf_down_focus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n",
		get_rear_cam_inf_down_focused_position());
}
static DEVICE_ATTR(rear_inf_down_focusd_position, 0644,
	rear_cam_inf_down_focus_show, NULL);

static ssize_t rear_cam_10cm_up_focus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n",
		get_rear_cam_10cm_up_focused_position());
}
static DEVICE_ATTR(rear_10cm_up_focusd_position, 0644,
	rear_cam_10cm_up_focus_show, NULL);

static ssize_t rear_cam_10cm_hori_focus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n",
		get_rear_cam_10cm_hori_focused_position());
}
static DEVICE_ATTR(rear_10cm_hori_focusd_position, 0644,
	rear_cam_10cm_hori_focus_show, NULL);

static ssize_t rear_cam_10cm_down_focus_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n",
		get_rear_cam_10cm_down_focused_position());
}
static DEVICE_ATTR(rear_10cm_down_focusd_position, 0644,
	rear_cam_10cm_down_focus_show, NULL);

static ssize_t rear_cam_sn_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%llX\n",
		get_rear_cam_sensor_module_sn());
}
static DEVICE_ATTR(rear_sn, 0644, rear_cam_sn_show, NULL);

static ssize_t nfov_cam_sn_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%llX\n",
		get_nfov_cam_sensor_module_sn());
}
static DEVICE_ATTR(nfov_sn, 0644, nfov_cam_sn_show, NULL);

static struct attribute *attrs_eeprom[] = {
	&dev_attr_rear_sn.attr,
	&dev_attr_nfov_sn.attr,
	&dev_attr_rear_inf_up_focusd_position.attr,
	&dev_attr_rear_inf_hori_focusd_position.attr,
	&dev_attr_rear_inf_down_focusd_position.attr,
	&dev_attr_rear_10cm_up_focusd_position.attr,
	&dev_attr_rear_10cm_hori_focusd_position.attr,
	&dev_attr_rear_10cm_down_focusd_position.attr,
	NULL,
};

static struct attribute_group attr_group_eeprom = {
	.attrs = attrs_eeprom,
};

static ssize_t cam_sensor_probe_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n", sensor_probe_status);
}

static DEVICE_ATTR(cam_sensor_probe_status, 0644,
	cam_sensor_probe_status_show, NULL);

static ssize_t cam_ois_probe_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%X\n", ois_probe_status);
}

static DEVICE_ATTR(cam_ois_probe_status, 0644,
	cam_ois_probe_status_show, NULL);

void create_file(int attr_type, struct kobject *kobj)
{
	int ret = 0;
	switch (attr_type) {
	case ATTR_SENSOR_PROBE_STATUS:
		ret = sysfs_create_file(kobj,
			&dev_attr_cam_sensor_probe_status.attr);
		break;
	case ATTR_OIS_PROBE_STATUS:
		ret = sysfs_create_file(kobj,
			&dev_attr_cam_ois_probe_status.attr);
		break;
	case ATTR_EEPROM_DATA:
		ret = sysfs_create_group(kobj, &attr_group_eeprom);
		break;
	}
	if (ret) {
		CAM_ERR(CAM_UTIL, "create file %d, err: %d", attr_type, ret);
	}
}

void remove_file(int attr_type, struct kobject *kobj)
{
	switch (attr_type) {
	case ATTR_SENSOR_PROBE_STATUS:
		sysfs_remove_file(kobj,
			&dev_attr_cam_sensor_probe_status.attr);
		break;
	case ATTR_OIS_PROBE_STATUS:
		sysfs_remove_file(kobj,
			&dev_attr_cam_ois_probe_status.attr);
		break;
	case ATTR_EEPROM_DATA:
		sysfs_remove_group(kobj, &attr_group_eeprom);
		break;
    }
}
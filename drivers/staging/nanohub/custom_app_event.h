/*
 * Copyright (C) 2017 Mobvoi, Inc.
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

#ifndef _NANOHUB_CUSTOM_EVENT_H
#define _NANOHUB_CUSTOM_EVENT_H

#define EVT_NO_SENSOR_CONFIG_EVENT       0x00000300
	/*event to configure sensors*/
#define APP_TO_HOST_EVENTID              0x00000401

#define SENSOR_APP_EVT_STATUS_SUCCESS    0x00
#define SENSOR_APP_EVT_STATUS_ERROR      0x01 /*General failure*/
#define SENSOR_APP_EVT_STATUS_BUSY       0x02

#define SENSOR_APP_MSG_ID_CAL_RESULT     0x00
	/*Status of calibration, with resulting biases*/
#define SENSOR_APP_MSG_ID_TEST_RESULT    0x01 /*Status of self-test*/
#define SENSOR_APP_MSG_ID_CUSTOM_USE     0xFE /*Status of custom use*/


#define kAppIdVendorMobvoi  0x4D6F62766FULL /*"Mobvo"*/
#define kAppIdVendorPixart  0x5069784172ULL /*"PixAr"*/
#define kAppIdFuelGauge     4
#define kAppIdPahHeartRate  10
#define kAppIdCustomFlash   25

#define SENS_TYPE_HEARTRATE_PPG   16
#define SENS_TYPE_FUELGAUGE       68
#define SENS_TYPE_CUSTOM_FLASH    89


struct HostHubRawPacket {
	uint64_t appId;
	uint8_t dataLen; /*not incl this header, 128 bytes max*/
    /*raw data in unspecified format here*/
} __packed;

struct SensorAppEventHeader {
	uint8_t msgId;
	uint8_t sensorType;
	uint8_t status; /*0 for success, else application-specific error code*/
} __packed;

enum ConfigCmds {
	CONFIG_CMD_DISABLE      = 0,
	CONFIG_CMD_ENABLE       = 1,
	CONFIG_CMD_FLUSH        = 2,
	CONFIG_CMD_CFG_DATA     = 3,
	CONFIG_CMD_CALIBRATE    = 4,
};

struct ConfigCmd {
	uint32_t evtType;
	uint64_t latency;
	uint32_t rate;
	uint8_t sensorType;
	uint8_t cmd;
	uint16_t flags;
	/*uint8_t data[];*/
} __packed;

uint64_t MakeAppId(uint64_t vendorId, uint32_t appId);

int is_hr_log_data(struct nanohub_buf *buf, int len);
int is_custom_flash_data(struct nanohub_buf *buf, int len);

#endif


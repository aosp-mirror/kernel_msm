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
#include "main.h"
#include "comms.h"

#include "custom_app_event.h"


uint64_t MakeAppId(uint64_t vendorId, uint32_t appId)
{
	return (vendorId << 24) | (appId & 0x00FFFFFF);
}

int is_hr_log_data(struct nanohub_buf *buf, int len)
{

	uint64_t kAppIdPixartPahHr =
		MakeAppId(kAppIdVendorPixart, kAppIdPahHeartRate);

	struct HostHubRawPacket *p_HostHubRawPacket;
	struct SensorAppEventHeader *p_SensorAppEventHeader;

	uint32_t event_id;

	if (len < sizeof(uint32_t) +
		sizeof(struct HostHubRawPacket) +
		sizeof(struct SensorAppEventHeader))
		return -EINVAL;

	p_HostHubRawPacket =
		(struct HostHubRawPacket *)&(buf->buffer[sizeof(uint32_t)]);
	p_SensorAppEventHeader =
		(struct SensorAppEventHeader *)
		&(buf->buffer[sizeof(uint32_t)
		+ sizeof(struct HostHubRawPacket)]);

	event_id =
		le32_to_cpu((((uint32_t *)(buf)->buffer)[0]) & 0x7FFFFFFF);

	if (event_id != APP_TO_HOST_EVENTID)
		return -EINVAL;

	if (p_HostHubRawPacket->appId != kAppIdPixartPahHr) {
		return -EINVAL;
	}
	if (p_SensorAppEventHeader->msgId != SENSOR_APP_MSG_ID_CUSTOM_USE ||
		p_SensorAppEventHeader->sensorType != SENS_TYPE_HEARTRATE_PPG ||
		p_SensorAppEventHeader->status !=
			SENSOR_APP_EVT_STATUS_SUCCESS) {
		pr_err("nanohub: [HR] bad SensorAppEventHeader");
		pr_err("msgId: 0x%x, sensorType: %d, status: %d\n",
			p_SensorAppEventHeader->msgId,
			p_SensorAppEventHeader->sensorType,
			p_SensorAppEventHeader->status);
		return -EINVAL;
	}

	return 0;
}

int is_custom_flash_data(struct nanohub_buf *buf, int len)
{

	uint64_t kAppIdMobvoiCustomFlash =
		MakeAppId(kAppIdVendorMobvoi, kAppIdCustomFlash);

	struct HostHubRawPacket *p_HostHubRawPacket;
	struct SensorAppEventHeader *p_SensorAppEventHeader;

	uint32_t event_id;

	if (len < sizeof(uint32_t) +
		sizeof(struct HostHubRawPacket) +
		sizeof(struct SensorAppEventHeader))
		return -EINVAL;

	p_HostHubRawPacket =
		(struct HostHubRawPacket *)&(buf->buffer[sizeof(uint32_t)]);
	p_SensorAppEventHeader =
		(struct SensorAppEventHeader *)
		&(buf->buffer[sizeof(uint32_t)
		+ sizeof(struct HostHubRawPacket)]);

	event_id =
		le32_to_cpu((((uint32_t *)(buf)->buffer)[0]) & 0x7FFFFFFF);

	if (event_id != APP_TO_HOST_EVENTID)
		return -EINVAL;

	if (p_HostHubRawPacket->appId != kAppIdMobvoiCustomFlash) {
		return -EINVAL;
	}
	if (p_SensorAppEventHeader->msgId != SENSOR_APP_MSG_ID_CUSTOM_USE ||
		p_SensorAppEventHeader->sensorType != SENS_TYPE_CUSTOM_FLASH ||
		p_SensorAppEventHeader->status !=
			SENSOR_APP_EVT_STATUS_SUCCESS) {
		pr_err("nanohub: [CF] bad SensorAppEventHeader");
		pr_err("msgId: 0x%x, sensorType: %d, status: %d\n",
			p_SensorAppEventHeader->msgId,
			p_SensorAppEventHeader->sensorType,
			p_SensorAppEventHeader->status);
		return -EINVAL;
	}

	return 0;
}



/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <media/cam_req_mgr.h>
#include "cam_sensor_vsync.h"
#include "cam_debug_util.h"
#include <linux/qrtr.h>
#include <linux/net.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/string.h>
#include <net/sock.h>
#include <linux/soc/qcom/qmi.h>

static uint8_t pb_encode_tag(enum pb_wire_type wire_type,
				uint32_t field_number)
{
	return (uint8_t)((field_number << 3) | wire_type);
}

static uint32_t pb_encode_fixed32(uint8_t *stream, const uint32_t val)
{
	stream[0] = (uint8_t)(val & 0xFF);
	stream[1] = (uint8_t)((val >> 8) & 0xFF);
	stream[2] = (uint8_t)((val >> 16) & 0xFF);
	stream[3] = (uint8_t)((val >> 24) & 0xFF);
	return 4;
}

static uint32_t pb_encode_fixed64(uint8_t *stream, const uint64_t val)
{
	stream[0] = (uint8_t)(val & 0xFF);
	stream[1] = (uint8_t)((val >> 8) & 0xFF);
	stream[2] = (uint8_t)((val >> 16) & 0xFF);
	stream[3] = (uint8_t)((val >> 24) & 0xFF);
	stream[4] = (uint8_t)((val >> 32) & 0xFF);
	stream[5] = (uint8_t)((val >> 40) & 0xFF);
	stream[6] = (uint8_t)((val >> 48) & 0xFF);
	stream[7] = (uint8_t)((val >> 56) & 0xFF);
	return 8;
}

static uint32_t pb_encode_varint(uint8_t *stream, uint64_t value)
{
	uint32_t len = 0;

	do {
		stream[len++] = (uint8_t)((value & 0x7F) | 0x80);
		value >>= 7;
	} while (value > 0);
	stream[len - 1] &= 0x7F; /* Unset top bit on last byte */

	return len;
}

static uint32_t pb_encode_suid(uint8_t *data, struct sns_std_suid *suid)
{
	uint32_t len = 0;

	data[len++] = pb_encode_tag(PB_WT_64BIT, 1);
	len += pb_encode_fixed64(data + len, suid->suid_low);
	data[len++] = pb_encode_tag(PB_WT_64BIT, 2);
	len += pb_encode_fixed64(data + len, suid->suid_high);
	return len;
}

static uint32_t pb_encode_susp_config(uint8_t *data,
		struct sns_client_request_msg_suspend_config *susp_config)
{
	uint32_t len = 0;

	data[len++] = pb_encode_tag(PB_WT_VARINT, 1);
	len += pb_encode_varint(data + len, susp_config->client_proc_type);
	data[len++] = pb_encode_tag(PB_WT_VARINT, 2);
	len += pb_encode_varint(data + len, susp_config->delivery_type);
	return len;
}

static uint32_t pb_encode_sns_request(uint8_t *data,
				struct sns_std_request *sns_request)
{
	/* Skip optional batching field. */
	/* Start with encoding the payload, which is the 2nd field */
	uint32_t len = 0;

	data[len++] = pb_encode_tag(PB_WT_STRING, 2);
	len += pb_encode_varint(data + len, sns_request->data_len);
	memcpy(data + len, sns_request->data, sns_request->data_len);
	len += sns_request->data_len;
	data[len++] = pb_encode_tag(PB_WT_VARINT, 3);
	len += pb_encode_varint(data + len, sns_request->is_passive);
	return len;
}

static uint32_t pb_encode_vsync_packet(uint8_t *data,
				struct cam_sensor_vsync_packet *packet)
{
	uint32_t len = 0;
	data[len++] = pb_encode_tag(PB_WT_VARINT, 1);
	len += pb_encode_varint(data + len, packet->sensor_id);
	data[len++] = pb_encode_tag(PB_WT_VARINT, 2);
	len += pb_encode_varint(data + len, packet->counter);
	data[len++] = pb_encode_tag(PB_WT_64BIT, 3);
	len += pb_encode_fixed64(data + len, packet->timestamp);
	return len;
}

static char *hexdump(const char *in, size_t in_len, char *out, size_t out_len)
{
	static const char chr[] = "0123456789abcdef";
	size_t i, oi = 0;

	for (i = 0; i < in_len && oi < out_len - 3; i++) {
		char c = in[i];

		out[oi++] = chr[c / 16];
		out[oi++] = chr[c % 16];
		if ((oi+1) % 33 == 0)
			out[oi++] = ' ';
	}
	out[oi++] = '\0';
	return out;
}

void cam_vsync_make_sensor_request(struct cam_sensor_vsync_packet *packet,
					uint8_t *data, uint32_t *data_len)
{
	uint32_t len = 0;
	uint32_t temp;
	struct sns_client_request_msg sns_client_request;
	char buffer[256];

	memset(&sns_client_request, 0, sizeof(struct sns_client_request_msg));
	// Only VSync sensor IDs 0, 1 and 2 which are supported
	if (packet->sensor_id > 2)
		goto out;
	sns_client_request.suid.suid_low = 0x444ba5dd07802b6d;
	sns_client_request.suid.suid_high =
		0x00ef87ba3007249d | (packet->sensor_id << 56);
	sns_client_request.msg_id =
		SNS_CAMERA_VSYNC_MSGID_INTERRUPT_INFO;
	sns_client_request.susp_config.client_proc_type =
		SNS_STD_CLIENT_PROCESSOR_APSS;
	sns_client_request.susp_config.delivery_type =
		SNS_CLIENT_DELIVERY_WAKEUP;
	sns_client_request.request.data_len =
		pb_encode_vsync_packet(sns_client_request.request.data, packet);
	sns_client_request.request.has_passive = true;
	sns_client_request.request.is_passive = false;

	// Encode SUID. Field 1 - 1st field in struct
	data[len++] = pb_encode_tag(PB_WT_STRING, 1);
	temp = pb_encode_suid(data + len + 1, &sns_client_request.suid);
	data[len] = (uint8_t)temp;
	len += (temp + 1);

	// Encode message ID. 2nd field
	data[len++] = pb_encode_tag(PB_WT_32BIT, 2);
	len += pb_encode_fixed32(data + len, sns_client_request.msg_id);

	// Encode susp_config. 3rd field
	data[len++] = pb_encode_tag(PB_WT_STRING, 3);
	temp = pb_encode_susp_config(data + len + 1,
					&sns_client_request.susp_config);
	data[len] = (uint8_t)temp;
	len += (temp + 1);

	// Encode sns_request. 4th field
	data[len++] = pb_encode_tag(PB_WT_STRING, 4);
	temp = pb_encode_sns_request(data + len + 1,
				&sns_client_request.request);
	data[len] = (uint8_t)temp;
	len += (temp + 1);

out:
	*data_len = len;
	CAM_DBG(CAM_SENSOR, "VSync data(%d)%s", len,
		hexdump(data, len, buffer, 256));
}
EXPORT_SYMBOL(cam_vsync_make_sensor_request);

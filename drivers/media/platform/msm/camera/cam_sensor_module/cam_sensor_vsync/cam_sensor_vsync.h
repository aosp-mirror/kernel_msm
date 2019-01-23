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

#ifndef _CAM_SENSOR_VSYNC_H_
#define _CAM_SENSOR_VSYNC_H_


#include <linux/spinlock.h>
#include <uapi/media/cam_isp.h>
#include <uapi/media/cam_defs.h>

#include "cam_context.h"

#define CAM_VSYNC_REQ1_TLV_TYPE     0x1
#define CAM_VSYNC_RESP1_TLV_TYPE    0x2
#define CAM_VSYNC_OPT1_TLV_TYPE     0x10
#define CAM_VSYNC_OPT2_TLV_TYPE     0x11

#define CAM_VSYNC_DATA_SIZE_V01          1000
#define CAM_VSYNC_MAX_NAME_SIZE_V01      255

#define CAM_VSYNC_REQ_MSG_ID_V01         0x20
#define CAM_VSYNC_REQ_MAX_MSG_LEN_V01    520

/**   QMI Service ID for this Sensors Service  */
#define SNS_CLIENT_SVC_ID_V01               400
#define SNS_CLIENT_SVC_V01_IDL_MAJOR_VERS   0x01
#define CAM_VSYNC_INFLIGHT_WORKS            5

/* Enum definitions */

/* Wire types. Library user needs these only in encoder callbacks. */
enum pb_wire_type {
	PB_WT_VARINT = 0,
	PB_WT_64BIT  = 1,
	PB_WT_STRING = 2,
	PB_WT_32BIT  = 5
};

enum sns_std_client_processor {
	SNS_STD_CLIENT_PROCESSOR_APSS = 1,
};

enum sns_client_msgid {
	SNS_CLIENT_MSGID_SNS_CLIENT_DISABLE_REQ = 10,
	SNS_CAMERA_VSYNC_MSGID_INTERRUPT_INFO   = 1041
};

enum sns_client_delivery {
	SNS_CLIENT_DELIVERY_WAKEUP = 0,
	SNS_CLIENT_DELIVERY_NO_WAKEUP = 1
};

struct sns_std_suid {
	uint64_t suid_low;
	uint64_t suid_high;
};

struct sns_client_request_msg_suspend_config {
	enum sns_std_client_processor client_proc_type;
	enum sns_client_delivery delivery_type;
};

struct sns_std_request {
	bool has_batching;
	void *batching;
	uint32_t data_len;
	uint8_t data[CAM_VSYNC_DATA_SIZE_V01];
	bool has_passive;
	bool is_passive;
};

struct sns_client_request_msg {
	struct sns_std_suid suid;
	uint32_t msg_id;
	struct sns_client_request_msg_suspend_config susp_config;
	struct sns_std_request request;
};

struct cam_sensor_vsync_packet {
	uint64_t sensor_id;
	uint64_t counter;
	uint64_t timestamp;
};

void cam_vsync_make_sensor_request(struct cam_sensor_vsync_packet *packet,
					uint8_t *data, uint32_t *data_len);

int cam_notify_vsync_qmi(struct cam_req_mgr_message *msg);

#endif

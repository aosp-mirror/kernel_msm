/* arch/arm/mach-msm/qdsp5v2/adsp_private.h
 *
 * Copyright (C) 2010 Google, Inc.
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

#ifndef _MSM_ADSP_5V2_PRIVATE_H_
#define _MSM_ADSP_5V2_PRIVATE_H_

/* adsp rtos / hardware memory map */

#define QDSP_RAMC_OFFSET			0x00400000
#define ADSP_READ_CTRL_OFFSET			0x00400038
#define ADSP_WRITE_CTRL_OFFSET			0x00400034
#define ADSP_SEND_IRQ_OFFSET			0x00c00200


/* adsp rtos hardware / shared memory interface */

#define ADSP_WRITE_CTRL_MUTEX_M			0x80000000U
#define ADSP_WRITE_CTRL_MUTEX_NAVAIL_V		0x80000000U
#define ADSP_WRITE_CTRL_MUTEX_AVAIL_V		0x00000000U

#define ADSP_WRITE_CTRL_CMD_M			0x70000000U
#define ADSP_WRITE_CTRL_CMD_WRITE_REQ_V		0x00000000U
#define ADSP_WRITE_CTRL_CMD_WRITE_DONE_V	0x10000000U
#define ADSP_WRITE_CTRL_CMD_NO_CMD_V		0x70000000U

#define ADSP_WRITE_CTRL_STATUS_M		0x0E000000U
#define ADSP_WRITE_CTRL_NO_ERR_V		0x00000000U
#define ADSP_WRITE_CTRL_NO_FREE_BUF_V		0x02000000U

#define ADSP_WRITE_CTRL_DSP_ADDR_M		0x00FFFFFFU

#define ADSP_WRITE_CTRL_HTOD_CMD_ID_M		0x00FFFFFFU

/* Combination of MUTEX and CMD bits to check if the DSP is busy */
#define ADSP_WRITE_CTRL_READY_M			0xF0000000U
#define ADSP_WRITE_CTRL_READY_V			0x70000000U

/* RTOS to Host processor command mask values */
#define ADSP_READ_CTRL_FLAG_M			0x80000000U
#define ADSP_READ_CTRL_FLAG_UP_WAIT_V		0x00000000U
#define ADSP_READ_CTRL_FLAG_UP_CONT_V		0x80000000U

#define ADSP_READ_CTRL_CMD_M			0x60000000U
#define ADSP_READ_CTRL_READ_DONE_V		0x00000000U
#define ADSP_READ_CTRL_READ_REQ_V		0x20000000U
#define ADSP_READ_CTRL_NO_CMD_V			0x60000000U

/* Combination of FLAG and COMMAND bits to check if MSG ready */
#define ADSP_READ_CTRL_READY_M			0xE0000000U
#define ADSP_READ_CTRL_READY_V			0xA0000000U
#define ADSP_READ_CTRL_CONT_V			0xC0000000U
#define ADSP_READ_CTRL_DONE_V			0xE0000000U

#define ADSP_READ_CTRL_STATUS_M			0x18000000U
#define ADSP_READ_CTRL_NO_ERR_V			0x00000000U

#define ADSP_READ_CTRL_IN_PROG_M		0x04000000U
#define ADSP_READ_CTRL_NO_READ_IN_PROG_V	0x00000000U
#define ADSP_READ_CTRL_READ_IN_PROG_V		0x04000000U

#define ADSP_READ_CTRL_CMD_TYPE_M		0x03000000U
#define ADSP_READ_CTRL_CMD_TASK_TO_H_V		0x00000000U

#define ADSP_READ_CTRL_DSP_ADDR_M		0x00FFFFFFU

#define ADSP_READ_CTRL_MSG_ID_M			0x000000FFU
#define ADSP_READ_CTRL_TASK_ID_M		0x0000FF00U


/* modem adsp management DAL service interface */

#define ADSP_DAL_DEVICE		0x0200009A
#define ADSP_DAL_PORT		"SMD_DAL00"
#define ADSP_DAL_COMMAND	(DAL_OP_FIRST_DEVICE_API | 0x80000000)

struct adsp_dal_cmd {
	uint32_t cmd;
	uint32_t proc_id;
	uint32_t module;
	void *cookie;
};

#define ADSP_PROC_NONE			0
#define ADSP_PROC_MODEM			1
#define ADSP_PROC_APPS			2

#define ADSP_CMD_ENABLE			1
#define ADSP_CMD_DISABLE		2
#define ADSP_CMD_DISABLE_EVENT_RSP	6
#define ADSP_CMD_GET_INIT_INFO		11

#define ADSP_EVT_MOD_READY		0
#define ADSP_EVT_MOD_DISABLE		1
#define ADSP_EVT_INIT_INFO		6
#define ADSP_EVT_DISABLE_FAIL		7

#define ADSP_TASKS_MAX	64
#define ADSP_QUEUES_MAX 4

#define MODULE_NAME_MAX 32
#define QUEUE_NAME_MAX  32

#define ADSP_QUEUE_FLAG_16BIT 0
#define ADSP_QUEUE_FLAG_32BIT 1

struct adsp_queue_info {
	uint8_t  name[QUEUE_NAME_MAX];
	uint32_t offset; /* Queue Offset in DSP memory */
	uint16_t idx; /* Global queue identifier */
	uint16_t max_size; /* Max allowed size in bytes for a queue */
	uint16_t flag; /* queue is 32bit Vs 16 bits */
	uint16_t rvd1;
	uint32_t rvd2;
};

struct adsp_module_info 
{
	uint8_t  name[MODULE_NAME_MAX];
	uint32_t uuid;
	uint16_t task_id;
	uint16_t q_cnt;
	struct adsp_queue_info queue[ADSP_QUEUES_MAX];
	uint32_t rvd1;
	uint32_t rvd2;
};
	
struct adsp_evt_info {
	uint32_t module;
	uint32_t image;
	uint32_t apps_okts; /* wtf is an okts? */
};
	
struct adsp_dal_event {
	/* DAL common event header */
	uint32_t evt_handle;
	uint32_t evt_cookie;
	uint32_t evt_length;

	/* ADSP event header */
	uint32_t event;
	uint32_t version;
	uint32_t proc_id;

	/* payload */
	union {
		struct adsp_module_info module;
		struct adsp_evt_info info;
	} u;		
};

#endif

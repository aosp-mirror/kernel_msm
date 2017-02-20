/* touch_sic_abt_spi.h
 *
 * Copyright (C) 2015 LGE.
 *
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

#ifndef TOUCH_SIC_ABT_H
#define TOUCH_SIC_ABT_H

#include <linux/socket.h>
#include <linux/in.h>

#define TOUCH_ABT_ATTR(_name, _show, _store)		\
			struct touch_attribute touch_attr_##_name	\
			= __ATTR(_name, 0660, _show, _store)

/* for abt monitor app. header */
#define CMD_ABT_LOC_X_START_READ				(0x2A6)
#define CMD_ABT_LOC_X_END_READ					(0x2A7)
#define CMD_ABT_LOC_Y_START_READ				(0x2A8)
#define CMD_ABT_LOC_Y_END_READ					(0x2A9)

/* Register SPI Address */
#define ABT_CMD_SPI_ADDR					(0xC30)

/* Register SRAM Offset */
#define ABT_REG_RAW_COMPRESS_MODE				(0x27)
#define ABT_REG_TC_RAW_DATA_LOAD				(0x29)
#define ABT_STS_RAWDATA_LOAD					(0x22)	/*@ABT: 0x88*/

/* debug data report mode setting */
#define CMD_RAW_DATA_COMPRESS_WRITE		(ABT_CMD_SPI_ADDR + ABT_REG_RAW_COMPRESS_MODE)	/*@AP : 0xC57*/
#define CMD_RAW_DATA_REPORT_MODE_WRITE		(ABT_CMD_SPI_ADDR + ABT_REG_TC_RAW_DATA_LOAD)	/*@AP : 0xC59*/

#define REPORT_RNORG						(11)
#define REPORT_RAW						(12)
#define REPORT_BASELINE						(13)
#define REPORT_SEG1						(14)
#define REPORT_SEG2						(15)
#define REPORT_GLASS						(16)
#define REPORT_DEBUG_ONLY					(17)
#define REPORT_OFF						(20)
#define ACTIVE_SCREEN_CNT_X					(18)
#define ACTIVE_SCREEN_CNT_Y					(32)
#define HEAD_LOAD						(10)
#define DEF_RNDCPY_EVERY_Nth_FRAME				(4)

#define DEFAULT_PORT						(8095)
#define TS_TCP_PORT						(8097)
#define SEND_PORT						(8090)
#define OMK_BUF							(1000)

#define MODULE_NAME						"ABT_SOCKET"

enum E_DATA_TYPE {
	DATA_TYPE_RAW = 0,
	DATA_TYPE_BASELINE,
	DATA_TYPE_RN_ORG = 10,
	DATA_TYPE_SEG1 = 20,
	DATA_TYPE_SEG2,
	DATA_TYPE_MAX
};

enum DEBUG_TYPE {
	DEBUG_DATA = 0,
	DEBUG_MODE = 1,
	DEBUG_DATA_RW_MODE = 2,
	DEBUG_DATA_CAPTURE_MODE = 3,
	DEBUG_DATA_CMD_MODE = 4
};

enum RW_TYPE {
	READ_TYPE = 55,
	WRITE_TYPE = 66
};

enum DEBUG_REPORT_CMD {
	DEBUG_REPORT_POINT = 0x100,
	DEBUG_REPORT_OCD,
};

enum ABT_CONNECT_TOOL {
	NOTHING = 0,
	ABT_STUDIO = 1,
	TOUCH_SOLUTION = 2
};

/* UDP */
#pragma pack(push, 1)
struct send_data_t {
	u8 type;
	u8 mode;
	u8 flag;
	u8 touchCnt;
	u32 timestamp;
	u32 frame_num;
	u8 data[3348];
};

struct debug_report_header {
	u8 mode1;
	u8 mode2;
	u16 mode1_data_size;
	u16 mode2_data_size;
	u8 reserved[10];
};

/* TCP */
enum ECommCMD {
	TCP_REG_READ = 0x80,
	TCP_REG_WRITE,
	TCP_FRAME_START,
	TCP_REPORT_START,
	TCP_SYNC_START,
	TCP_SYNCDEBUG_START,
	TCP_CAPTURE_STOP,
	TCP_CONNECT_CMD,
	TCP_DISCONNECT_CMD
};

enum ECommRes {
	eCommRes_Success	 = 0,
	eCommRes_WriteFailed = 0x8001
};

typedef struct {
	u8		cmd;
	u16		addr;
	u16		size;
} TPacketHdr, *PPacketHdr;

struct s_comm_packet {
	TPacketHdr	hdr;
	union {
		u32		value;
		u8		frame[4000];
	} data;
};

struct sock_comm_t {
	struct device *dev;

	struct task_struct *thread;

	/*ABT Studio socket */
	struct socket *sock;
	struct sockaddr_in addr;

	struct socket *sock_send;
	struct sockaddr_in addr_send;

	/* Touch Solution socket */
	struct socket *ts_sock;
	struct sockaddr_in ts_addr;

	uint32_t (*sock_listener)(uint8_t *buf, uint32_t len);

	struct send_data_t data_send;

	struct s_comm_packet *recv_packet;
	struct s_comm_packet send_packet;

	int send_connected;
	char send_ip[20];

	u8 running;
};

struct T_ABTLog_FileHead {
	/* 4 byte => 4B */
	/* 2B : x축 해상도 */
	unsigned short resolution_x;
	/* 2B : y축 해상도 */
	unsigned short resolution_y;
	/* 4 byte => 8B */
	/* 1B : real node 개수 (x축) */
	unsigned char node_cnt_x;
	/* 1B : real node 개수 (y축) */
	unsigned char node_cnt_y;
	/* 1B : additional node 개수 */
	unsigned char additional_node_cnt;
	unsigned char dummy1;
	/* 4 byte => 12B */
	/* 2B : RN MIN value */
	unsigned short rn_min;
	/* 2B : RN MAX value */
	unsigned short rn_max;
	/* 4 byte => 16B */
	/* 1B : RAW Size (1B or 2B) */
	unsigned char raw_data_size;
	/* 1B : RN Size (1B or 2B) */
	unsigned char rn_data_size;
	/* 1B : frame buf 에 쌓인 data 의 type -> E_DATA_TYPE number */
	unsigned char frame_data_type;
	/* 1B : frame buf 에 쌓인 data 의 unit size (1B or 2B) */
	unsigned char frame_data_size;
	/* 4 byte => 20B */
	/* 4B : x_node start/end 의 location(screen resolution) */
	unsigned short loc_x[2];
	/* 4 byte => 24B */
	/* 4B : y_node start/end 의 location(screen resolution) */
	unsigned short loc_y[2];
	/* 104 byte => 128B */
	/* total 128B 중 나머지.. */
	unsigned char dummy[104];
} __packed;

#pragma pack(pop)

extern int sw49408_sic_abt_is_set_func(void);
extern void sw49408_sic_abt_probe(void);
extern void sw49408_sic_abt_remove(void);
extern void sw49408_sic_abt_init(struct device *dev);
extern int sw49408_sic_abt_is_debug_mode(void);
extern int sw49408_sic_abt_irq_handler(struct device *dev);
extern void sw49408_sic_abt_ocd_off(struct device *dev);
extern void sw49408_sic_abt_onchip_debug(struct device *dev, u8 *all_data);
extern void sw49408_sic_abt_report_mode(struct device *dev, u8 *all_data);
extern void sw49408_sic_abt_register_sysfs(struct kobject *kobj);
#endif


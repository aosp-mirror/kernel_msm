/* touch_sw49408_abt.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
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
#define TS_MODULE "[abt]"

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/signal.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/socket.h>
#include <linux/net.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw49408.h"
#include "touch_sw49408_abt.h"

#define MAX_REPORT_SLOT				16
#define P_CONTOUR_POINT_MAX			8
#define DEF_RNDCPY_EVERY_Nth_FRAME	(4)
#define PACKET_SIZE						128
#define DATA_I2CBASE_ADDR				(0xFD1)
#define SERIAL_DATA_OFFSET				(0x008B)
#define DBG_BUF_OFFSET					(0x461C)
#define DBG_BUF_GRID2_OFFSET			(0x500)
#define ABT_ABT_STUDIO_RN_AFL			(23)
#define FRONT_BACK_FRM_CNT_DATA_LEN		(8)
#define LINE_OFFSET_DATA_LEN			(36)


static int ocd_piece_size;
static u32 prev_rnd_piece_no = DEF_RNDCPY_EVERY_Nth_FRAME;

struct mutex abt_comm_lock;
struct mutex abt_socket_lock;
struct sock_comm_t abt_comm;
int abt_socket_mutex_flag;
int abt_socket_report_mode;

u16 frame_num = 0;
int abt_report_mode;
int abt_report_grid1_mode;
int abt_report_grid2_mode;
u8 abt_report_point;
u8 abt_report_ocd;
int abt_report_mode_onoff;
enum ABT_CONNECT_TOOL abt_conn_tool = NOTHING;

/*sysfs*/
u16 abt_ocd[2][ACTIVE_SCREEN_CNT_X * ACTIVE_SCREEN_CNT_Y];
u8 abt_ocd_read;
u8 abt_reportP[256];
char abt_head[128];
int abt_show_mode;
int abt_ocd_on;
u32 abt_compress_flag;
int abt_head_flag;

static int abt_ocd_off = 1;
int set_get_data_func = 0;

struct T_TouchInfo {
	u8 wakeUpType;
	u8 touchCnt:5;
	u8 buttonCnt:3;
	u16 palmBit;
} __packed;

struct T_TouchData {
	u8 toolType:4;
	u8 event:4;
	s8 track_id;
	u16 x;
	u16 y;
	union {
		u8 pressure;
		u8 contourPcnt;
	} byte;
	u8 angle;
	u16 width_major;
	u16 width_minor;
} __packed;


struct T_TCRegCopy2Report {
	u32 tc_reg_copy[5];
};

struct T_OnChipDebug {
	u32 rnd_addr;
	u32 rnd_piece_no;
};

struct T_ReportP {
	struct T_TouchInfo touchInfo;
	struct T_TouchData touchData[MAX_REPORT_SLOT];
	u16 contourP[P_CONTOUR_POINT_MAX];
	struct T_TCRegCopy2Report tc_reg;
	struct T_OnChipDebug ocd;
	u8 dummy[16];
};

int abt_force_set_report_mode(struct device *dev, u32 mode);
int ReadMemory(struct device *dev, u16 nMemAddr, u16 offset_addr,
	u32 nOffset, u16 nOffsetValueByteSize, u8 *pData, u32 nSize);

int sw49408_sic_abt_is_set_func(void)
{
	return set_get_data_func;
}

static void sic_set_get_data_func(u8 mode)
{
	if (mode == 1) {
		set_get_data_func = 1;
		TOUCH_I("(%s)change get_data \"sic_ts_get_data_debug_mode\"\n",
			__func__);
	} else {
		set_get_data_func = 0;
		TOUCH_I("(%s)change get_data \"sic_ts_get_data\"\n",
			__func__);
	}
}

static int32_t abt_ksocket_init_send_socket(struct sock_comm_t *abt_socket)
{
	int ret;
	struct socket *sock;
	struct sockaddr_in *addr = &abt_socket->addr_send;
	int *connected = &abt_socket->send_connected;
	char *ip = (char *)abt_socket->send_ip;

	ret = sock_create(AF_INET,
			  SOCK_DGRAM,
			  IPPROTO_UDP,
			  &(abt_socket->sock_send));
	sock = abt_socket->sock_send;

	if (ret >= 0) {
		memset(addr, 0, sizeof(struct sockaddr));
		addr->sin_family = AF_INET;
		addr->sin_addr.s_addr = in_aton(ip);
		addr->sin_port = htons(SEND_PORT);
	} else {
		TOUCH_I(MODULE_NAME": can not create socket %d\n",
			-ret);
		goto error;
	}

	ret = sock->ops->connect(sock,
				 (struct sockaddr *)addr,
				 sizeof(struct sockaddr),
				 !O_NONBLOCK);

	if (ret < 0) {
		TOUCH_I(MODULE_NAME": Could not connect to send socket," \
			"error = %d\n", -ret);
		goto error;
	} else {
		*connected = 1;
		TOUCH_I(MODULE_NAME ": connect send socket (%s,%d)(\n",
			ip, SEND_PORT);
	}

	return ret;
error:
	sock = NULL;
	*connected = 0;
	return ret;
}

static int abt_ksocket_receive(unsigned char *buf, int len)
{
	struct msghdr msg;
	struct iovec iov;
	struct socket *sock;
	struct sockaddr_in *addr;
	mm_segment_t oldfs;
	unsigned int flag = 0;
	int size = 0;

	if (abt_conn_tool == TOUCH_SOLUTION) {
		sock = abt_comm.ts_sock;
		addr = &abt_comm.ts_addr;
	} else if (abt_conn_tool == ABT_STUDIO) {
		sock = abt_comm.sock;
		addr = &abt_comm.addr;
	} else {
		return 0;
	}

	iov.iov_base = buf;
	iov.iov_len = len;

	msg.msg_flags = flag;
	msg.msg_name = addr;
	msg.msg_namelen  = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	/* [Bring-Up] Build error
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	*/
	msg.msg_control = NULL;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	size = sock_recvmsg(sock, &msg, len, msg.msg_flags);
	set_fs(oldfs);

	if (size > 0)
		abt_comm.sock_listener(buf, size);
	else
		TOUCH_I(": sock_recvmsg size invalid %d\n", size);

	return size;
}

static void abt_ksocket_start_for_pctool(struct device *dev)
{
	static int client_connected;
	int size, err;
	unsigned char *buf;
	struct socket *sock;

	/* kernel thread initialization */
	abt_comm.running = 1;
	abt_comm.dev = dev;

	err = sock_create(AF_INET,
			SOCK_STREAM,
			IPPROTO_TCP,
			&abt_comm.ts_sock);
	sock = abt_comm.ts_sock;

	if (err >= 0) {
		memset(&abt_comm.ts_addr, 0, sizeof(struct sockaddr));
		abt_comm.ts_addr.sin_family = AF_INET;
		abt_comm.ts_addr.sin_addr.s_addr = in_aton(abt_comm.send_ip);
		abt_comm.ts_addr.sin_port = htons(TS_TCP_PORT);
	} else {
		TOUCH_I(
			MODULE_NAME": can not create socket %d\n",
			-err);
		goto out;
	}

	err = sock->ops->connect(sock,
				(struct sockaddr *)&abt_comm.ts_addr,
				sizeof(struct sockaddr), !O_NONBLOCK);

	if (err < 0) {
		TOUCH_E(MODULE_NAME": Could not connect to tcp rw socket," \
			"error = %d\n", -err);
		goto out;
	} else {
		client_connected = 1;
		TOUCH_I(MODULE_NAME": TCP connected with TS (ip %s,port %d)(\n",
			abt_comm.send_ip, TS_TCP_PORT);
	}

	buf = kzalloc(sizeof(struct s_comm_packet), GFP_KERNEL);

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (abt_comm.running != 1)
			break;

		size = abt_ksocket_receive(&buf[0],
			sizeof(struct s_comm_packet));

		if (size < 1) {
			TOUCH_E(MODULE_NAME
				": RECEIVE sock_recvmsg invalid = %d\n", size);
			break;
		}
		if (kthread_should_stop()) {
			TOUCH_I(MODULE_NAME": kthread_should_stop\n");
			break;
		}
	}

out:
	if (buf != NULL)
		kfree(buf);
	__set_current_state(TASK_RUNNING);
	abt_comm.running = 0;
	abt_comm.thread = NULL;
	if (abt_comm.ts_sock != NULL) {
		sock_release(abt_comm.ts_sock);
		abt_comm.ts_sock = NULL;
	}
}

static void abt_ksocket_start_for_abtstudio(struct device *dev)
{
	int size, err;
	int bufsize = 10;
	unsigned char buf[bufsize+1];

	frame_num = 0;

	/* kernel thread initialization */
	abt_comm.running = 1;
	abt_comm.dev = dev;

	/* create a socket */
	err = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &abt_comm.sock);
	if (err < 0) {
		TOUCH_I(
			": could not create a datagram socket, error = %d\n",
			-ENXIO);
		goto out;
	}

	memset(&abt_comm.addr, 0, sizeof(struct sockaddr));
	abt_comm.addr.sin_family = AF_INET;
	abt_comm.addr.sin_addr.s_addr = htonl(INADDR_ANY);
	abt_comm.addr.sin_port = htons(DEFAULT_PORT);
	err = abt_comm.sock->ops->bind(abt_comm.sock,
					(struct sockaddr *)&abt_comm.addr,
					sizeof(struct sockaddr));
	if (err < 0) {
		TOUCH_I(
			MODULE_NAME
			": Could not bind to receive socket, error = %d\n",
			-err);
		goto out;
	}

	TOUCH_I(": listening on port %d\n", DEFAULT_PORT);

	/* init raw data send socket */
	abt_ksocket_init_send_socket(&abt_comm);

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		memset(&buf, 0, bufsize+1);

		size = 0;
		if (abt_comm.running == 1) {
			size = abt_ksocket_receive(buf, bufsize);
			TOUCH_I(": receive packet\n");
		} else {
			TOUCH_I(": running off\n");
			break;
		}

		if (kthread_should_stop())
			break;

		if (size < 0)
			TOUCH_I(
			": error getting datagram, sock_recvmsg error = %d\n",
			size);

		touch_msleep(50);
	}

out:
	__set_current_state(TASK_RUNNING);
	abt_comm.running = 0;
	abt_comm.thread = NULL;
}

static uint32_t abt_ksocket_send_exit(void)
{
	uint32_t ret = 0;
	struct msghdr msg;
	struct iovec iov;
	mm_segment_t oldfs;
	struct socket *sock;
	struct sockaddr_in addr;
	uint8_t buf = 1;

	ret = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP, &sock);
	if (ret < 0) {
		TOUCH_I(
			MODULE_NAME
			": could not create a datagram socket, error = %d\n",
			-ENXIO);
		goto error;
	}

	memset(&addr, 0, sizeof(struct sockaddr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = in_aton("127.0.0.1");
	addr.sin_port = htons(DEFAULT_PORT);

	ret = sock->ops->connect(
			sock,
			(struct sockaddr *)&addr,
			sizeof(struct sockaddr),
			!O_NONBLOCK);

	if (ret < 0) {
		TOUCH_I(
			MODULE_NAME
			": Could not connect to send socket, error = %d\n",
			-ret);
		goto error;
	} else {
		TOUCH_I(
			MODULE_NAME": connect send socket (%s,%d)\n",
			"127.0.0.1",
			DEFAULT_PORT);
	}

	iov.iov_base = &buf;
	iov.iov_len = 1;

	msg.msg_flags = 0;
	msg.msg_name = &addr;
	msg.msg_namelen  = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	/* [Bring-Up] Build error
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	*/
	msg.msg_control = NULL;

	oldfs = get_fs();

	set_fs(KERNEL_DS);
//	ret = sock_sendmsg(sock, &msg, 1);
	TOUCH_I(": exit send message return : %d\n", ret);
	set_fs(oldfs);
	sock_release(sock);

error:
	return ret;
}

static int abt_ksocket_send(struct socket *sock,
			struct sockaddr_in *addr,
			unsigned char *buf, int len)
{
	struct msghdr msg;
	struct iovec iov;
	mm_segment_t oldfs;
	int size = 0;

	if (sock == NULL)
		return 0;

	iov.iov_base = buf;
	iov.iov_len = len;

	msg.msg_flags = 0;
	msg.msg_name = addr;
	msg.msg_namelen  = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	/* [Bring-Up] Build error
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	*/
	msg.msg_control = NULL;

	oldfs = get_fs();

	set_fs(KERNEL_DS);
//	size = sock_sendmsg(sock, &msg, len);
	set_fs(oldfs);

	return size;
}

static int abt_set_report_mode(struct device *dev, u32 mode)
{
	int ret;

	TOUCH_I(
		"[ABT](%s)mode:%d\n", __func__, mode);
	if (abt_report_mode == mode) {
		TOUCH_I(
			"[ABT](%s) mode(%d) is already set\n",
			__func__, mode);
		return 0;
	}

	if (mode < 0) {
		TOUCH_I(
			"[ABT](%s) mode(%d) is invalid\n",
			__func__, mode);
		return -EINVAL;
	}

	ret = abt_force_set_report_mode(dev, mode);

	return ret;
}

static uint32_t abt_ksocket_rcv_from_pctool(uint8_t *buf, uint32_t len)
{
	int ret;
	abt_comm.recv_packet = (struct s_comm_packet *)buf;

	if (abt_comm.recv_packet->hdr.cmd == TCP_REG_READ) {
		ret = sw49408_reg_read(abt_comm.dev,
					abt_comm.recv_packet->hdr.addr,
					&abt_comm.send_packet.data.frame[0],
					abt_comm.recv_packet->data.value);
		if (ret < 0) {
			TOUCH_I(
				MODULE_NAME
				": TCP REG READ spi_read error : %d\n\n", ret);
		}

		abt_comm.send_packet.hdr.cmd = abt_comm.recv_packet->hdr.cmd;
		abt_comm.send_packet.hdr.addr = abt_comm.recv_packet->hdr.addr;
		abt_comm.send_packet.hdr.size =
					abt_comm.recv_packet->data.value;

		abt_ksocket_send(abt_comm.ts_sock,
				&abt_comm.ts_addr,
				(u8 *)&abt_comm.send_packet,
				sizeof(TPacketHdr) +
				abt_comm.recv_packet->data.value);
	} else if (abt_comm.recv_packet->hdr.cmd == TCP_REG_WRITE) {
		ret = sw49408_reg_write(abt_comm.dev,
					abt_comm.recv_packet->hdr.addr,
					&abt_comm.recv_packet->data.frame[0],
					abt_comm.recv_packet->hdr.size);
		if (ret < 0) {
			TOUCH_I(
				MODULE_NAME
				": TCP REG WRITE spi_read error : %d\n\n", ret);
			abt_comm.send_packet.data.value = eCommRes_WriteFailed;
		} else {
			abt_comm.send_packet.data.value = eCommRes_Success;
		}
		abt_comm.send_packet.hdr.cmd = abt_comm.recv_packet->hdr.cmd;
		abt_comm.send_packet.hdr.addr = abt_comm.recv_packet->hdr.addr;
		abt_comm.send_packet.hdr.size = sizeof(u32);
		abt_ksocket_send(abt_comm.ts_sock,
				&abt_comm.ts_addr,
				(u8 *)&abt_comm.send_packet,
				sizeof(TPacketHdr) + sizeof(u32));
	}

	return 0;
}

static uint32_t abt_ksocket_rcv_from_abtstudio(uint8_t *buf, uint32_t len)
{
	uint32_t cmd = (uint32_t)*((uint32_t *)buf);
	uint32_t val = (uint32_t)*((uint32_t *)(buf+4));
	TOUCH_I(": CMD=%d VAL=%d\n", cmd, val);
	mutex_lock(&abt_comm_lock);
	switch (cmd) {
	case CMD_RAW_DATA_REPORT_MODE_WRITE:
		TOUCH_I(
			MODULE_NAME": mode setting - %d\n",
			val);
		abt_set_report_mode(abt_comm.dev, val);
		break;

	case CMD_RAW_DATA_COMPRESS_WRITE:
		sw49408_write_value(abt_comm.dev,
			CMD_RAW_DATA_COMPRESS_WRITE, val);
		abt_compress_flag = val;
		break;

	case DEBUG_REPORT_POINT:
		abt_report_point = val;
		break;

	case DEBUG_REPORT_OCD:
		abt_report_ocd = val;
		break;

	default:
		TOUCH_I(": unknown command\n");
		break;
	}
	mutex_unlock(&abt_comm_lock);
	return 0;
}


static void abt_ksocket_exit(void)
{
	int err;

	if (abt_comm.thread == NULL) {
		if (abt_socket_mutex_flag == 1)
			mutex_destroy(&abt_socket_lock);
		abt_socket_mutex_flag = 0;
		abt_socket_report_mode = 0;
		abt_comm.running = 0;

		TOUCH_I(
			MODULE_NAME
			": no kernel thread to kill\n");

		return;
	}

	TOUCH_I(MODULE_NAME ": start killing thread\n");

	abt_comm.running = 2;

	if (abt_conn_tool == ABT_STUDIO) {
		if (abt_ksocket_send_exit() < 0) {
			TOUCH_E(MODULE_NAME": ABT_STUDIO abt_ksocket_send_exit error\n");
		} else {
			TOUCH_I(MODULE_NAME": ABT_STUDIO abt_ksocket_send_exit done\n");
		}
	} else {
		TOUCH_I(MODULE_NAME": Send disconnect command to Touch solution \n");

		/* send disconnect command to pc server */
		abt_comm.send_packet.hdr.cmd = TCP_DISCONNECT_CMD;
		abt_comm.send_packet.hdr.size = 4;
		abt_ksocket_send(abt_comm.ts_sock,
				&abt_comm.ts_addr,
				(u8 *)&abt_comm.send_packet,
				sizeof(TPacketHdr) +
				abt_comm.send_packet.hdr.size);

		/* abt_comm.ts_sock->ops->disconnect(abt_comm.ts_sock, 0); */
	}

	TOUCH_I(MODULE_NAME ": waiting for killing thread\n");
	err = kthread_stop(abt_comm.thread);
	if (err < 0) {
		TOUCH_I(MODULE_NAME ": unknown error %d while trying to" \
			"terminate kernel thread\n", -err);
	} else {
		while (abt_comm.running != 0) {
			TOUCH_I(MODULE_NAME ": waiting for killing thread." \
				"abt_comm.running.. %d\n", abt_comm.running);
			touch_msleep(10);
		}
		TOUCH_I(MODULE_NAME ": succesfully killed kernel thread!\n");
	}

	abt_comm.thread = NULL;

	if (abt_comm.sock != NULL) {
		sock_release(abt_comm.sock);
		abt_comm.sock = NULL;
	}

	if (abt_comm.sock_send != NULL) {
		sock_release(abt_comm.sock_send);
		abt_comm.sock_send = NULL;
	}

	if (abt_comm.ts_sock != NULL) {
		sock_release(abt_comm.ts_sock);
		abt_comm.ts_sock = NULL;
	}

	mutex_destroy(&abt_socket_lock);
	abt_socket_mutex_flag = 0;
	abt_socket_report_mode = 0;

	TOUCH_I(": module unloaded\n");
}

int32_t abt_ksocket_raw_data_send(uint8_t *buf, uint32_t len)
{
	int ret = 0;
	static uint32_t connect_error_count;

	if (abt_comm.send_connected == 0)
		abt_ksocket_init_send_socket(&abt_comm);

	if (abt_comm.send_connected == 1) {
		ret = abt_ksocket_send(abt_comm.sock_send,
				&abt_comm.addr_send,
				buf, len);
	} else {
		connect_error_count++;
		if (connect_error_count > 10) {
			TOUCH_I(
				": connection error - socket release\n");
			abt_force_set_report_mode(abt_comm.dev, 0);
			abt_ksocket_exit();
		}
	}

	return ret;
}

static int32_t abt_ksocket_init(struct device *dev,
			char *ip,
			uint32_t (*listener)(uint8_t *buf, uint32_t len))
{
	mutex_init(&abt_socket_lock);
	abt_socket_mutex_flag = 1;
	abt_socket_report_mode = 1;
	memcpy(abt_comm.send_ip, ip, 16);

	if (abt_conn_tool == ABT_STUDIO) {
		abt_comm.thread =
			kthread_run((void *)abt_ksocket_start_for_abtstudio,
						dev, MODULE_NAME);
	} else if (abt_conn_tool == TOUCH_SOLUTION) {
		abt_comm.thread =
			kthread_run((void *)abt_ksocket_start_for_pctool,
						dev, MODULE_NAME);
	}

	if (IS_ERR(abt_comm.thread)) {
		TOUCH_I(
			MODULE_NAME
			": unable to start kernel thread\n");
		abt_comm.thread = NULL;
		return -ENOMEM;
	}

	abt_comm.sock_listener = listener;

	return 0;
}

ssize_t show_abtApp(struct device *dev, char *buf)
{
	int i;
	ssize_t retVal = 0;

	if (abt_head_flag) {
		if (abt_show_mode == REPORT_SEG1)
			abt_head[14] = DATA_TYPE_SEG1;
	 else if (abt_show_mode == REPORT_SEG2)
			abt_head[14] = DATA_TYPE_SEG2;
	 else if (abt_show_mode == REPORT_RAW)
			abt_head[14] = DATA_TYPE_RAW;
	 else if (abt_show_mode == REPORT_BASELINE)
			abt_head[14] = DATA_TYPE_BASELINE;
	 else if (abt_show_mode == REPORT_RNORG)
			abt_head[14] = DATA_TYPE_RN_ORG;

		retVal = sizeof(abt_head);
		memcpy(&buf[0], (u8 *)&abt_head[0], retVal);
		abt_head_flag = 0;
		return retVal;
	}

	switch (abt_show_mode) {
	case REPORT_RNORG:
	case REPORT_RAW:
	case REPORT_BASELINE:
	case REPORT_SEG1:
	case REPORT_SEG2:
		i = ACTIVE_SCREEN_CNT_X * ACTIVE_SCREEN_CNT_Y*2;
		memcpy(&buf[0], (u8 *)&abt_ocd[abt_ocd_read^1][0], i);
		memcpy(&buf[i], (u8 *)&abt_reportP, sizeof(abt_reportP));
		i += sizeof(abt_reportP);
		retVal = i;
		break;
	case REPORT_DEBUG_ONLY:
		memcpy(&buf[0], (u8 *)&abt_reportP, sizeof(abt_reportP));
		i = sizeof(abt_reportP);
		retVal = i;
		break;
	default:
		abt_show_mode = 0;
		retVal = 0;
		break;
	}
	return retVal;
}

ssize_t store_abtApp(struct device *dev, const char *buf, size_t count)
{
	u32 mode;

	mode = count;

	if (mode == HEAD_LOAD) {
		abt_head_flag = 1;
		abt_ocd_on = 1;
		TOUCH_I("[ABT] abt_head load\n");
		return abt_head_flag;
	} else {
		abt_show_mode = mode;
		abt_head_flag = 0;
	}

	switch (abt_show_mode) {
	case REPORT_RNORG:
		TOUCH_I("[ABT] show mode : RNORG\n");
		break;
	case REPORT_RAW:
		TOUCH_I("[ABT] show mode : RAW\n");
		break;
	case REPORT_BASELINE:
		TOUCH_I("[ABT] show mode : BASELINE\n");
		break;
	case REPORT_SEG1:
		TOUCH_I("[ABT] show mode : SEG1\n");
		break;
	case REPORT_SEG2:
		TOUCH_I("[ABT] show mode : SEG2\n");
		break;
	case REPORT_DEBUG_ONLY:
		TOUCH_I("[ABT] show mode : DEBUG ONLY\n");
		break;
	case REPORT_OFF:
		TOUCH_I("[ABT] show mode : OFF\n");
		break;
	default:
		TOUCH_I(
			"[ABT] show mode unknown : %d\n", mode);
		break;
	}

	return abt_show_mode;
}

ssize_t show_abtTool(struct device *dev, char *buf)
{
	buf[0] = abt_report_mode_onoff;

	memcpy((u8 *)&buf[1], (u8 *)&(abt_comm.send_ip[0]), 16);
	TOUCH_I(
		MODULE_NAME
		":read raw report mode - mode:%d ip:%s\n",
		buf[0], (char *)&buf[1]);

	return 20;
}

ssize_t store_abtTool(struct device *dev,
				const char *buf,
				size_t count)
{
	int mode = buf[0];
	char *ip = (char *)&buf[1];
	bool setFlag = false;

	TOUCH_I(
		":set raw report mode - mode:%d IP:%s\n", mode, ip);

	if (mode > 47)
		mode -= 48;

	switch (mode) {
	case 1:
		abt_conn_tool = ABT_STUDIO;
		if (abt_comm.thread == NULL) {
			TOUCH_I(":  mode ABT STUDIO Start\n\n");
			abt_ksocket_init(dev,
					 (u8 *)ip,
					 abt_ksocket_rcv_from_abtstudio);
			sic_set_get_data_func(1);
			setFlag = true;
		} else {
			if (memcmp((u8 *)abt_comm.send_ip,
					(u8 *)ip, 16) != 0) {
				TOUCH_I(
					":IP change->ksocket exit n init\n\n");
				abt_ksocket_exit();
				abt_ksocket_init(dev, (u8 *)ip,
					 abt_ksocket_rcv_from_abtstudio);
				setFlag = true;
			}
		}
		break;
	case 2:
		abt_conn_tool = TOUCH_SOLUTION;

		if (abt_comm.thread == NULL) {
			TOUCH_I(
				": mode Touch Solution Start\n\n");
			abt_ksocket_init(dev, (u8 *)ip,
					 abt_ksocket_rcv_from_pctool);

			sic_set_get_data_func(0);
			abt_report_point = 0;
			abt_report_ocd = 0;

		} else {
			TOUCH_I(
				": abt_comm.thread Not NULL\n\n");
			if (memcmp((u8 *)abt_comm.send_ip,
					(u8 *)ip, 16) != 0) {
				abt_ksocket_exit();
				abt_ksocket_init(dev, (u8 *)ip,
						 abt_ksocket_rcv_from_pctool);
			} else {
				TOUCH_I(
					": same IP\n\n");
			}
		}
		break;
	default:
		abt_conn_tool = NOTHING;

		abt_ksocket_exit();
		sic_set_get_data_func(0);

		mutex_lock(&abt_comm_lock);
		abt_set_report_mode(dev, 0);
		mutex_unlock(&abt_comm_lock);
	}

	if (setFlag) {
		mutex_lock(&abt_comm_lock);
		abt_set_report_mode(dev, mode);
		mutex_unlock(&abt_comm_lock);
	}

	return mode;
}

int abt_force_set_report_mode(struct device *dev, u32 mode)
{
	int ret = 0;
	u32 rdata = 0;
	u32 wdata = mode;
	struct sw49408_data *d = to_sw49408_data(dev);

	/* send debug mode*/
	ret = sw49408_write_value(dev, d->reg_info.r_abt_cmd_spi_addr +
					ABT_REG_TC_RAW_DATA_LOAD, wdata);
	abt_report_mode = mode;

	if ((mode & 0x000000FF) != 0)
		abt_report_grid1_mode = (mode & 0x000000FF);

	if (((mode & 0x0000FF00) >> 8) != 0 )
		abt_report_grid2_mode = ((mode & 0x0000FF00) >> 8);

	/* receive debug report buffer*/
	if (mode >= 0) {
		ret = sw49408_read_value(dev, d->reg_info.r_abt_sts_spi_addr +
						ABT_STS_RAWDATA_LOAD, &rdata);
		TOUCH_I("(%d)rdata\n", rdata);
		if (ret < 0 || rdata <= 0) {
			TOUCH_I("(%s)debug report buffer pointer error\n",
				__func__);
			goto error;
		}

		TOUCH_I("(%s)debug report buffer pointer : 0x%x\n",
			__func__, rdata);
	}

	return 0;

error:
	wdata = 0;
	ret = sw49408_write_value(dev, d->reg_info.r_abt_cmd_spi_addr +
					 ABT_REG_TC_RAW_DATA_LOAD, wdata);
	abt_report_mode = 0;
	abt_report_grid1_mode = 0;
	abt_report_grid2_mode = 0;
	abt_report_mode_onoff = 0;
	return ret;
}

void sw49408_sic_abt_probe(void)
{
	mutex_init(&abt_comm_lock);
}

void sw49408_sic_abt_remove(void)
{
	mutex_destroy(&abt_comm_lock);
	if (abt_socket_mutex_flag == 1)
		mutex_destroy(&abt_socket_lock);
}

void sw49408_sic_abt_init(struct device *dev)
{
	u32 head_loc[4];
	struct T_ABTLog_FileHead tHeadBuffer;
	memset((u8 *)&tHeadBuffer, 0, sizeof(struct T_ABTLog_FileHead));

	if (abt_report_mode != 0)
		abt_force_set_report_mode(dev, abt_report_mode);

	sw49408_read_value(dev, CMD_ABT_LOC_X_START_READ, &head_loc[0]);
	sw49408_read_value(dev, CMD_ABT_LOC_X_END_READ, &head_loc[1]);
	sw49408_read_value(dev, CMD_ABT_LOC_Y_START_READ, &head_loc[2]);
	sw49408_read_value(dev, CMD_ABT_LOC_Y_END_READ, &head_loc[3]);

	tHeadBuffer.resolution_x = 1440;
	tHeadBuffer.resolution_y = 2720;
	tHeadBuffer.node_cnt_x = ACTIVE_SCREEN_CNT_X;
	tHeadBuffer.node_cnt_y = ACTIVE_SCREEN_CNT_Y;
	tHeadBuffer.additional_node_cnt = 0;
	tHeadBuffer.rn_min = 1000;
	tHeadBuffer.rn_max = 1300;
	tHeadBuffer.raw_data_size = sizeof(u16);
	tHeadBuffer.rn_data_size = sizeof(u16);
	tHeadBuffer.frame_data_type = DATA_TYPE_RN_ORG;
	tHeadBuffer.frame_data_size = sizeof(u16);
	tHeadBuffer.loc_x[0] = (u16) head_loc[0];
	tHeadBuffer.loc_x[1] = (u16) head_loc[1];
	tHeadBuffer.loc_y[0] = (u16) head_loc[2];
	tHeadBuffer.loc_y[1] = (u16) head_loc[3];

	memcpy((u8 *)&abt_head[0], (u8 *)&tHeadBuffer,
		sizeof(struct T_ABTLog_FileHead));
}

int sw49408_sic_abt_is_debug_mode(void)
{
	if (abt_show_mode >= REPORT_RNORG && abt_show_mode <= REPORT_DEBUG_ONLY)
		return 1;

	return 0;
}

int sw49408_sic_abt_irq_handler(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	u8 all_data[264];

	int report_mode = sw49408_sic_abt_is_set_func();
	if (sw49408_reg_read(dev, tc_ic_status,
			&all_data[0], sizeof(all_data)) < 0) {
		TOUCH_I("report data reg addr read fail\n");
		goto error;
	}

	memcpy(&d->info, all_data, sizeof(d->info));

	ret = sw49408_check_status(dev);
	if (ret < 0)
		goto error;

	if (report_mode) {
		if (d->info.wakeup_type == ABS_MODE)
			ret = sw49408_irq_abs_data(dev);
		else
			ret = sw49408_irq_lpwg(dev);

		sw49408_sic_abt_report_mode(dev, all_data);
	} else {
		if (d->info.wakeup_type == ABS_MODE) {
			if (sw49408_sic_abt_is_debug_mode()) {
				sw49408_sic_abt_onchip_debug(dev, &all_data[8]);
				ret = sw49408_irq_abs_data(dev);
			} else {
				sw49408_sic_abt_ocd_off(dev);
				ret = sw49408_irq_abs_data(dev);
			}
		} else {
			ret = sw49408_irq_lpwg(dev);
		}
	}

	if ((abt_conn_tool == NOTHING) && (d->info.debug.runtime_dbg_inttype > 0))
		sw49408_irq_runtime_engine_debug(dev);

error:
	return ret;
}

void sw49408_sic_abt_ocd_off(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 wdata = 0;

	if (abt_ocd_off) {
		sw49408_write_value(dev, d->reg_info.r_abt_cmd_spi_addr +
					ABT_REG_TC_RAW_DATA_LOAD, wdata);
		sw49408_read_value(dev, d->reg_info.r_abt_sts_spi_addr +
					ABT_STS_RAWDATA_LOAD, &wdata);
		TOUCH_I("[ABT] onchipdebug off: wdata=%d\n", wdata);
		abt_ocd_off = 0;
	}
}

void sw49408_sic_abt_onchip_debug(struct device *dev, u8 *all_data)
{
	u32			i, j;
	u32			wdata;
	static u32	u32_dbg_offset		= DBG_BUF_OFFSET/4;
	int			node;
	int			ret;
	u8			*_u8_data_ptr;
	struct T_ReportP local_reportP;
	struct sw49408_data *d = to_sw49408_data(dev);
	static u8	u8_ocd_pieces_cnt	= 0;

	abt_ocd_off = 1;

	memcpy(&local_reportP, all_data, sizeof(struct T_ReportP));

	/* Write onchipdebug on */
	if (abt_ocd_on) {
		wdata = abt_show_mode;
		TOUCH_I("[ABT] onchipdebug on(before write): wdata=%d\n", wdata);
		sw49408_write_value(dev, d->reg_info.r_abt_cmd_spi_addr +
					ABT_REG_TC_RAW_DATA_LOAD, wdata);
		sw49408_read_value(dev, d->reg_info.r_abt_sts_spi_addr +
					ABT_STS_RAWDATA_LOAD, &wdata);
		TOUCH_I("[ABT] onchipdebug on(after write): wdata=%d\n", wdata);
		abt_ocd_on = 0;
	} else if (abt_show_mode < REPORT_DEBUG_ONLY) {
		if (u8_ocd_pieces_cnt == 0 && u8_ocd_pieces_cnt
			!= prev_rnd_piece_no)
			abt_ocd_read ^= 1;

		ocd_piece_size = ACTIVE_SCREEN_CNT_X * ACTIVE_SCREEN_CNT_Y
			/ DEF_RNDCPY_EVERY_Nth_FRAME;

		if (ocd_piece_size % 2)
			ocd_piece_size -= 1;
		if (u8_ocd_pieces_cnt == 0)
			u32_dbg_offset	= DBG_BUF_OFFSET/4;
		node = u8_ocd_pieces_cnt * ocd_piece_size;

		if (u8_ocd_pieces_cnt != prev_rnd_piece_no) {
			if (u8_ocd_pieces_cnt !=
					DEF_RNDCPY_EVERY_Nth_FRAME - 1) {
				_u8_data_ptr =
					(u8 *)&abt_ocd[abt_ocd_read][node];
				for (j = 0; j < (ocd_piece_size*2)/MAX_RW_SIZE;
						j++) {
					ret = ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						u32_dbg_offset, sizeof(u32),
						_u8_data_ptr,
						sizeof(u8)*MAX_RW_SIZE);
					if (ret < 0)
						TOUCH_E("RNdata reg addr write fail [%d]\n",
							u8_ocd_pieces_cnt);
					_u8_data_ptr +=
						(sizeof(u8)*MAX_RW_SIZE);
					u32_dbg_offset +=
						(sizeof(u8)*MAX_RW_SIZE/4);
				}
				if ((ocd_piece_size*2) % MAX_RW_SIZE != 0) {
					ret = ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						u32_dbg_offset, sizeof(u32),
						_u8_data_ptr,
						sizeof(u8) *
						((ocd_piece_size*2) %
							MAX_RW_SIZE));
					if (ret < 0)
						TOUCH_E("RNdata reg addr write fail [%d]\n",
							u8_ocd_pieces_cnt);
					_u8_data_ptr += (sizeof(u8) *
						((ocd_piece_size * 2) %
							MAX_RW_SIZE));
					u32_dbg_offset += (sizeof(u8) *
						((ocd_piece_size * 2) %
							MAX_RW_SIZE)/4);
				}
			} else {
				i = ACTIVE_SCREEN_CNT_X * ACTIVE_SCREEN_CNT_Y;
				i -= ocd_piece_size *
					(DEF_RNDCPY_EVERY_Nth_FRAME - 1);
				_u8_data_ptr =
					(u8 *)&abt_ocd[abt_ocd_read][node];

				for (j = 0; j < (i*2)/MAX_RW_SIZE; j++) {
					ret = ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						u32_dbg_offset, sizeof(u32),
						_u8_data_ptr,
						sizeof(u8)*MAX_RW_SIZE);
					if (ret < 0)
						TOUCH_E("RNdata reg addr write fail [%d]\n",
							u8_ocd_pieces_cnt);
					_u8_data_ptr +=
						(sizeof(u8) * MAX_RW_SIZE);
					u32_dbg_offset +=
						(sizeof(u8) * MAX_RW_SIZE / 4);
				}
				if ((i * 2) % MAX_RW_SIZE != 0) {
					ret = ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						u32_dbg_offset, sizeof(u32),
						_u8_data_ptr,
						sizeof(u8)*((i*2)%MAX_RW_SIZE));
					if (ret < 0)
						TOUCH_E("RNdata reg addr write fail [%d]\n",
							u8_ocd_pieces_cnt);
					_u8_data_ptr += (sizeof(u8) *
						((i * 2) % MAX_RW_SIZE));
					u32_dbg_offset += (sizeof(u8) *
						((i * 2) % MAX_RW_SIZE) / 4);
				}
				memcpy(abt_reportP, all_data,
					sizeof(u8) * 34 * 4);
				memcpy(&abt_reportP[34 * 4], &all_data[34 * 4],
					sizeof(u8) * 112);
			}
		}
		prev_rnd_piece_no = u8_ocd_pieces_cnt;
		if (u8_ocd_pieces_cnt >= DEF_RNDCPY_EVERY_Nth_FRAME-1)
			u8_ocd_pieces_cnt = 0;
		else
			u8_ocd_pieces_cnt++;
	}
}

int ReadMemory(struct device *dev, u16 nMemAddr, u16 offset_addr, u32 nOffset,
	u16 nOffsetValueByteSize, u8 *pData, u32 nSize)
{
	u16 nRead;
	u32 nToRead = nSize;

	if (nToRead > MAX_RW_SIZE)
		nRead = MAX_RW_SIZE;
	else
		nRead = nToRead;

	while (nRead > 0) {
		sw49408_reg_write(dev, offset_addr, (u8 *)&nOffset,
			nOffsetValueByteSize);
		nRead = sw49408_reg_read(dev, nMemAddr, pData, nRead);
		if (nRead <= 0)
			return nRead;

		if (nToRead > nRead) {
			nToRead -= nRead;
			pData += nRead;
		} else {
			nToRead = 0;
		}
		nOffset += nRead/4;

		if (nToRead > MAX_RW_SIZE)
			nRead = MAX_RW_SIZE;
		else
			nRead = nToRead;
	}

	return nSize;
}

void sw49408_sic_abt_report_mode(struct device *dev, u8 *all_data)
{
	struct send_data_t *packet_ptr   = &abt_comm.data_send;
	struct debug_report_header *d_header =
		(struct debug_report_header *)(&abt_comm.data_send.data[0]);
	int  d_header_size = sizeof(struct debug_report_header);
	struct timeval t_stamp;
	struct sw49408_touch_info *t_info
		= (struct sw49408_touch_info *) all_data;
	u32  u32_dbg_grid1_mode_offset = DBG_BUF_OFFSET/4;
	u32  u32_dbg_grid2_mode_offset =
				(DBG_BUF_OFFSET + DBG_BUF_GRID2_OFFSET)/4;
	u8  *d_data_ptr = (u8 *)d_header + d_header_size;
	int  i;
	u16  grid1_data_size = 0;
	u16  grid2_data_size = 0;

	if (abt_report_grid1_mode || abt_report_grid2_mode) {
		//1. Header read
		ReadMemory(dev, DATA_I2CBASE_ADDR, SERIAL_DATA_OFFSET,
		u32_dbg_grid1_mode_offset,
		sizeof(u32), (u8 *)d_header,
		d_header_size);

		u32_dbg_grid1_mode_offset += (d_header_size/4);

		if ( (d_header->mode1 == abt_report_grid1_mode) &&
					(d_header->mode1_data_size != 0) ) {

			switch(d_header->mode1) {
			case ABT_ABT_STUDIO_RN_AFL :
				grid1_data_size = d_header->mode1_data_size -
						LINE_OFFSET_DATA_LEN -
						FRONT_BACK_FRM_CNT_DATA_LEN;
				break;
			default :
				grid1_data_size = d_header->mode1_data_size -
						FRONT_BACK_FRM_CNT_DATA_LEN;
				break;
			}

			if ( grid1_data_size % (ACTIVE_SCREEN_CNT_X *
						ACTIVE_SCREEN_CNT_Y) == 0 ) {
				for ( i = 0; i < d_header -> mode1_data_size /
							MAX_RW_SIZE; i++ ) {
					ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						u32_dbg_grid1_mode_offset,
						sizeof(u32), d_data_ptr,
						sizeof(u8)*MAX_RW_SIZE);

					d_data_ptr += (sizeof(u8)*MAX_RW_SIZE);
					u32_dbg_grid1_mode_offset +=
						(sizeof(u8)*MAX_RW_SIZE/4);
				}

				if ( d_header->mode1_data_size %
							 MAX_RW_SIZE != 0 ) {
					ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						u32_dbg_grid1_mode_offset,
						sizeof(u32), d_data_ptr,
						sizeof(u8) *
						(d_header -> mode1_data_size %
								MAX_RW_SIZE));
					d_data_ptr += (sizeof(u8) *
						(d_header->mode1_data_size %
								MAX_RW_SIZE));
					u32_dbg_grid1_mode_offset +=
						(sizeof(u8) *
						(d_header -> mode1_data_size %
								MAX_RW_SIZE)/4);
				}
			}
		} else {
			if ( d_header->mode1 != abt_report_grid1_mode ) {
				TOUCH_I("GRID1 debug data load error !![type : %d, size : %d] \n",
				d_header->mode1, d_header->mode1_data_size);
			}
		}

		if ( (d_header->mode2 == abt_report_grid2_mode) &&
					(d_header->mode2_data_size != 0) ) {

			switch(d_header->mode2)	{
			case ABT_ABT_STUDIO_RN_AFL :
				grid2_data_size = d_header->mode2_data_size -
						LINE_OFFSET_DATA_LEN -
						FRONT_BACK_FRM_CNT_DATA_LEN;
				break;
			default :
				grid2_data_size = d_header->mode2_data_size -
						FRONT_BACK_FRM_CNT_DATA_LEN;
				break;
			}

			if ( grid2_data_size %(ACTIVE_SCREEN_CNT_X *
						ACTIVE_SCREEN_CNT_Y) == 0 ) {
				for ( i = 0; i < d_header -> mode2_data_size /
							MAX_RW_SIZE; i++ ) {
					ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						 u32_dbg_grid2_mode_offset,
						sizeof(u32), d_data_ptr,
						sizeof(u8) * MAX_RW_SIZE);
					d_data_ptr += (sizeof(u8)*MAX_RW_SIZE);
					u32_dbg_grid2_mode_offset +=
						(sizeof(u8) * MAX_RW_SIZE / 4);
				}

				if ( d_header->mode2_data_size %
							 MAX_RW_SIZE != 0 ) {
					ReadMemory(dev, DATA_I2CBASE_ADDR,
						SERIAL_DATA_OFFSET,
						u32_dbg_grid2_mode_offset,
						sizeof(u32), d_data_ptr,
						sizeof(u8) *
						(d_header -> mode2_data_size %
								MAX_RW_SIZE));

					d_data_ptr += (sizeof(u8) *
						(d_header -> mode2_data_size %
								MAX_RW_SIZE));
					u32_dbg_grid2_mode_offset +=
						(sizeof(u8) *
						(d_header -> mode2_data_size %
								MAX_RW_SIZE)/4);
				}
			}
		} else {
			if ( d_header->mode2 != abt_report_grid2_mode ) {
				TOUCH_I("GRID2 debug data load error !![type : %d, size : %d] \n",
				d_header->mode2, d_header->mode2_data_size);
			}
		}
	} else {
		packet_ptr->touchCnt = 0;
	}

	/* ABS0 */
	if (t_info->wakeup_type == 0) {
		if (abt_report_ocd) {
			memcpy(d_data_ptr, &all_data[36 * 4], sizeof(u8) * 112);
			d_data_ptr += (sizeof(u8) * 112);
		}

		if (t_info->data[0].track_id != 15) {
			if (abt_report_point) {
				packet_ptr->touchCnt = t_info->touch_cnt;
				memcpy(d_data_ptr, &t_info->data[0],
				sizeof(struct sw49408_touch_data) *
				t_info->touch_cnt);
				d_data_ptr += sizeof(struct sw49408_touch_data)
							* t_info->touch_cnt;
			} else {
				packet_ptr->touchCnt = 0;
			}
		}
	}

	if ((u8 *)d_data_ptr - (u8 *)packet_ptr > 0) {
		do_gettimeofday(&t_stamp);
		frame_num++;
		packet_ptr->type = DEBUG_DATA;
		packet_ptr->mode = abt_report_grid1_mode;
		packet_ptr->frame_num = frame_num;
		packet_ptr->timestamp =
				t_stamp.tv_sec * 1000000 + t_stamp.tv_usec;

		packet_ptr->flag = 0;
		if (abt_report_point)
			packet_ptr->flag |= 0x1;
		if (abt_report_ocd)
			packet_ptr->flag |= (0x1)<<1;

		abt_ksocket_raw_data_send((u8 *)packet_ptr,
						(u8 *)d_data_ptr -
							(u8 *)packet_ptr);
	}
}

static TOUCH_ABT_ATTR(abt_monitor, show_abtApp, store_abtApp);
static TOUCH_ABT_ATTR(raw_report, show_abtTool, store_abtTool);

static struct attribute *sw49408_abt_attribute_list[] = {
	&touch_attr_abt_monitor.attr,
	&touch_attr_raw_report.attr,
	NULL,
};

static const struct attribute_group sw49408_abt_attribute_group = {
	.attrs = sw49408_abt_attribute_list,
};

void sw49408_sic_abt_register_sysfs(struct kobject *kobj)
{
	int ret = sysfs_create_group(kobj, &sw49408_abt_attribute_group);

	if (ret < 0)
		TOUCH_E("failed to create sysfs for abt\n");
}

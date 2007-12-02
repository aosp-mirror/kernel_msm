/** include/asm-arm/arch-msm/msm_rpcrouter.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM__ARCH_MSM_RPCROUTER_H
#define __ASM__ARCH_MSM_RPCROUTER_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/platform_device.h>

struct msm_rpc_endpoint;

struct rpcsvr_platform_device
{
	struct platform_device base;
	uint32_t prog;
	uint32_t vers;
};

#define RPC_DATA_IN	0
/*
 * Structures for sending / receiving direct RPC requests
 * XXX: Any cred/verif lengths > 0 not supported
 */

struct rpc_request_hdr
{
	uint32_t xid;
	uint32_t type;	/* 0 */
	uint32_t rpc_vers; /* 2 */
	uint32_t prog;
	uint32_t vers;
	uint32_t procedure;
	uint32_t cred_flavor;
	uint32_t cred_length;
	uint32_t verf_flavor;
	uint32_t verf_length;
};

typedef struct
{
	uint32_t low;
	uint32_t high;
} rpc_reply_progmismatch_data;

typedef struct
{
} rpc_denied_reply_hdr;

typedef struct
{
	uint32_t verf_flavor;
	uint32_t verf_length;
	uint32_t accept_stat;
#define RPC_ACCEPTSTAT_SUCCESS 0
#define RPC_ACCEPTSTAT_PROG_UNAVAIL 1
#define RPC_ACCEPTSTAT_PROG_MISMATCH 2
#define RPC_ACCEPTSTAT_PROC_UNAVAIL 3
#define RPC_ACCEPTSTAT_GARBAGE_ARGS 4
#define RPC_ACCEPTSTAT_SYSTEM_ERR 5
#define RPC_ACCEPTSTAT_PROG_LOCKED 6
	/*
	 * Following data is dependant on accept_stat
	 * If ACCEPTSTAT == PROG_MISMATCH then there is a
	 * 'rpc_reply_progmismatch_data' structure following the header.
	 * Otherwise the data is procedure specific
	 */
} rpc_accepted_reply_hdr;

struct rpc_reply_hdr
{
	uint32_t xid;
	uint32_t type;
	uint32_t reply_stat;
#define RPCMSG_REPLYSTAT_ACCEPTED 0
#define RPCMSG_REPLYSTAT_DENIED 1
	union {
		rpc_accepted_reply_hdr acc_hdr;
		rpc_denied_reply_hdr dny_hdr;
	} data;
};

/* flags for msm_rpc_connect() */
#define MSM_RPC_UNINTERRUPTIBLE 0x0001

/* use IS_ERR() to check for failure */
struct msm_rpc_endpoint *msm_rpc_open(void);
struct msm_rpc_endpoint *msm_rpc_connect(uint32_t prog, uint32_t vers, unsigned flags);

int msm_rpc_close(struct msm_rpc_endpoint *ept);
int msm_rpc_write(struct msm_rpc_endpoint *ept,
		  void *data, int len);
int msm_rpc_read(struct msm_rpc_endpoint *ept,
		 void **data, unsigned len, long timeout);
void msm_rpc_setup_req(struct rpc_request_hdr *hdr,
		       uint32_t prog, uint32_t vers, uint32_t proc);
int msm_rpc_register_server(struct msm_rpc_endpoint *ept,
			    uint32_t prog, uint32_t vers);
int msm_rpc_unregister_server(struct msm_rpc_endpoint *ept,
			      uint32_t prog, uint32_t vers);

/* simple blocking rpc call
 *
 * request is mandatory and must have a rpc_request_hdr
 * at the start.  The header will be filled out for you.
 *
 * reply provides a buffer for replies of reply_max_size
 */
int msm_rpc_call_reply(struct msm_rpc_endpoint *ept, uint32_t proc,
		       void *request, int request_size,
		       void *reply, int reply_max_size,
		       long timeout);
int msm_rpc_call(struct msm_rpc_endpoint *ept, uint32_t proc,
		 void *request, int request_size,
		 long timeout);

struct msm_rpc_server
{
	struct list_head list;
	uint32_t flags;

	uint32_t prog;
	uint32_t vers;

	int (*rpc_call)(struct msm_rpc_server *server,
			struct rpc_request_hdr *req, unsigned len);
};

int msm_rpc_create_server(struct msm_rpc_server *server);

#endif

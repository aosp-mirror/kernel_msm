/*
 * iaxxx-tunnel-priv.h -- iaxxx tunneling Service private data
 *
 * Copyright (c) 2016 Audience, inc.
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

#ifndef _IAXXX_TUNNEL_H
#define _IAXXX_TUNNEL_H

#include <linux/mfd/adnc/iaxxx-register-defs-out-tunnel-group.h>
#include <linux/mfd/adnc/iaxxx-tunnel-registers.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/cdev.h>
#include "iaxxx.h"

struct iaxxx_tunnel_buff_params {
	uint32_t buff_tail;
	uint32_t buff_head;
	uint32_t buff_addr;
	uint32_t buff_size;
};

enum iaxxx_tunnel_mode_type {
	TNL_MODE_SYNC = 0,
	TNL_MODE_ASYNC,
};

enum iaxxx_tunnel_encode_type {
	TNL_ENC_OPAQUE = 0,
	TNL_ENC_AFLOAT = 1,
	TNL_ENC_Q15 = 0xF,
};

enum iaxxx_tunnel_dir_type {
	TNL_HOST_DEVICE_RX = 0,
	TNL_DEVICE_HOST_TX,
};

enum iaxxx_out_tunnel_type {
	TNL0 = 0,
	TNL1,
	TNL2,
	TNL3,
	TNL4,
	TNL5,
	TNL6,
	TNL7,
	TNL8,
	TNL9,
	TNL10,
	TNL11,
	TNL12,
	TNL13,
	TNL14,
	TNL15,
	TNL16,
	TNL17,
	TNL18,
	TNL19,
	TNL20,
	TNLMAX
};

int iaxxx_tunnel_setup(struct iaxxx_priv *priv,
		uint32_t tunlEP, uint32_t tunlSrc,
		uint32_t tunlMode, uint32_t tunlEncode);
int iaxxx_tunnel_terminate(struct iaxxx_priv *priv, uint32_t tunlEP);
int iaxxx_get_tunnel_buff_params(struct iaxxx_priv *priv,
			struct iaxxx_tunnel_buff_params *buff_param);
int iaxxx_update_tunnel_buff_params(struct iaxxx_priv *priv,
				uint32_t buff_head, int mode);
int iaxxx_tunnel_read(struct iaxxx_priv *priv, void *readbuff, int count,
			int mode, int *bytes_remaining);
int iaxxx_tunnel_dev_init(struct iaxxx_priv *priv);
int iaxxx_tunnel_dev_destroy(struct iaxxx_priv *priv);
int iaxxx_flush_tunnel_fw_buff(struct iaxxx_priv *priv);
int iaxxx_set_tunnel_event_threshold(struct iaxxx_priv *priv,
					uint32_t tunnel_buff_thrshld);
int iaxxx_tunnel_signal_event(struct iaxxx_priv *priv);

#endif

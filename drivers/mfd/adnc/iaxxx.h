/*
 * iaxxx.h -- IAxxx MFD core internals
 *
 * Copyright 2016 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef __MFD_IAXXX_H__
#define __MFD_IAXXX_H__

#include <linux/device.h>

struct iaxxx_priv;
struct regmap;
struct regmap_config;
struct iaxxx_event;
struct iaxxx_evt_queue;
#define IAXXX_READ	0
#define IAXXX_WRITE	1
#define IAXXX_BUF_MAX_LEN	2048
#define IAXXX_CM4_CTRL_MGR_SRC_ID 0x2610
#define IAXXX_CRASH_EVENT_ID 2
#define IAXXX_SPI_SBL_SPEED	4800000
#define PLUGIN_INST_NONE	0xFF

#define EVENT_ID_KW_ID                0
/* Knowles VT algo specific events */
#define EVENT_ID_START_FRAME          1
#define EVENT_ID_END_FRAME            2
#define EVENT_ID_TRUE_CONFIRMATAION   3
#define EVENT_ID_FALSE_ACCEPTANCE     4
#define EVENT_ID_MAX_CONFIDENCE_LEVEL 5

/* Checksum Calculation */
#define CALC_FLETCHER16(DATA, SUM1, SUM2)	\
do {						\
	SUM1 += (DATA) & 0xffff;		\
	SUM2 += SUM1;				\
	SUM1 += ((DATA) >> 16) & 0xffff;	\
	SUM2 += SUM1;				\
	SUM1 = (SUM1 & 0xffff) + (SUM1 >> 16);	\
	SUM2 = (SUM2 & 0xffff) + (SUM2 >> 16);	\
} while (0)

/* System mode */
enum iaxxx_system_status_mode {
	SYSTEM_STATUS_MODE_RESET = 0,
	SYSTEM_STATUS_MODE_SBL,
	SYSTEM_STATUS_MODE_APPS,
};

enum {
	SYSRC_SUCCESS = 0,
	SYSRC_FAIL,
	SYSRC_ERR_PARM,
	SYSRC_ERR_MEM,
	SYSRC_ERR_BUSY,
	SYSRC_ERR_PEND,
	SYSRC_ERR_STATE,
	SYSRC_ERR_LIMIT,
	SYSRC_ERR_BOUNDARY,
	SYSRC_ERR_IO,
};

struct iaxxx_register_log {
	u32 addr;
	u32 val;
	bool op;
	struct timespec timestamp;
};

struct iaxxx_reg_dump_priv {
	struct iaxxx_register_log *log;
	uint32_t head;
	uint32_t tail;
	spinlock_t ring_lock;
};

struct firmware_file_header {
	uint32_t signature[2];
	uint32_t reserved;
	uint32_t abort_code;
	uint32_t number_of_sections;
	uint32_t entry_point;
};

/* Used for both binary data and checksum sections */
/* Each data section is followed by a checksum section */
struct firmware_section_header {
	uint32_t length;	/* Section length in 32-bit words */
	uint32_t start_address;
};

struct iaxxx_raw_bus_ops {
	int (*read)(struct iaxxx_priv *priv, void *buf, int len);
	int (*write)(struct iaxxx_priv *priv, const void *buf, int len);
};

int iaxxx_device_reset(struct iaxxx_priv *priv);
int iaxxx_device_init(struct iaxxx_priv *priv);
void iaxxx_device_exit(struct iaxxx_priv *priv);

/* Checks if the device firmware is ready to accept requests from the host */
int iaxxx_get_device_status(struct iaxxx_priv *priv, bool *status);

/* Boots the device into application mode */
int iaxxx_bootup(struct iaxxx_priv *priv);

/* Register map */
int iaxxx_regmap_init(struct iaxxx_priv *priv);
int iaxxx_sbl_regmap_init(struct iaxxx_priv *priv);
int iaxxx_application_regmap_init(struct iaxxx_priv *priv);

/* Bootloader */
int iaxxx_jump_to_request(struct iaxxx_priv *priv, uint32_t address);
int iaxxx_calibrate_oscillator_request(struct iaxxx_priv *priv, uint32_t delay);

/* Checksum */
int iaxxx_checksum_request(struct iaxxx_priv *priv, uint32_t address,
			uint32_t length, uint32_t *sum1, uint32_t *sum2);

/* Event manager */
int iaxxx_next_event_request(struct iaxxx_priv *priv, struct iaxxx_event *evt);
int iaxxx_event_handler(struct iaxxx_priv *priv, struct iaxxx_event *evt);

int iaxxx_subscribe_request(struct iaxxx_priv *priv, u16 event_id,
				u16 event_src, u16 event_dst, u32 opaque_data);

int iaxxx_unsubscribe_request(struct iaxxx_priv *priv,
				u16 event_id, u16 event_src, u16 event_dst);
int iaxxx_event_init(struct iaxxx_priv *priv);
void iaxxx_event_exit(struct iaxxx_priv *priv);
void register_transac_log(struct device *dev, uint32_t reg, uint32_t val,
								bool op);
int iaxxx_verify_fw_header(struct device *dev,
			struct firmware_file_header *header);
int iaxxx_download_section(struct iaxxx_priv *priv, const uint8_t *data,
				const struct firmware_section_header *section);
void iaxxx_copy_le32_to_cpu(void *dst, const void *src, size_t nbytes);

#endif /* __MFD_IAXXX_H__ */

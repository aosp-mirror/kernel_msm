/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

#if !defined(__ADF_TRACE_H)
#define __ADF_TRACE_H

/**
 *  DOC:  adf_trace
 *
 *  Atheros driver framework trace APIs
 *
 *  Trace, logging, and debugging definitions and APIs
 *
 */

 /* Include Files */
#include  <adf_nbuf.h>
#include "vos_types.h"

#ifdef FEATURE_DPTRACE_ENABLE
 /* DP Trace Implementation */
#define DPTRACE(p) p
#else
#define DPTRACE(p)  /*no-op*/
#endif

#define MAX_ADF_DP_TRACE_RECORDS       4000
#define ADF_DP_TRACE_RECORD_SIZE       16
#define INVALID_ADF_DP_TRACE_ADDR      0xffffffff
#define ADF_DP_TRACE_VERBOSITY_HIGH    3
#define ADF_DP_TRACE_VERBOSITY_MEDIUM  2
#define ADF_DP_TRACE_VERBOSITY_LOW     1
#define ADF_DP_TRACE_VERBOSITY_DEFAULT 0

/**
 * struct adf_mac_addr - mac address array
 * @bytes: MAC address bytes
 */
struct adf_mac_addr {
	uint8_t bytes[VOS_MAC_ADDR_SIZE];
};

/**
 * enum ADF_DP_TRACE_ID - Generic ID to identify various events in data path
 * @ADF_DP_TRACE_INVALID: Invalid ID
 * @ADF_DP_TRACE_DROP_PACKET_RECORD: Dropped packet stored with this id
 * @ADF_DP_TRACE_HDD_PACKET_PTR_RECORD: nbuf->data ptr of HDD
 * @ADF_DP_TRACE_HDD_PACKET_RECORD: nbuf->data stored with this id
 * @ADF_DP_TRACE_CE_PACKET_PTR_RECORD: nbuf->data ptr of CE
 * @ADF_DP_TRACE_CE_PACKET_RECORD: nbuf->data stored with this id
 * @ADF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD: nbuf->data ptr of txrx queue
 * @ADF_DP_TRACE_TXRX_PACKET_PTR_RECORD: nbuf->data ptr of txrx
 * @ADF_DP_TRACE_HTT_PACKET_PTR_RECORD: nbuf->data ptr of htt
 * @ADF_DP_TRACE_HTC_PACKET_PTR_RECORD: nbuf->data ptr of htc
 * @ADF_DP_TRACE_HIF_PACKET_PTR_RECORD: nbuf->data ptr of hif
 * @ADF_DP_TRACE_HDD_TX_TIMEOUT: hdd tx timeout event
 * @ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT: hdd tx softap timeout event
 *
 */

enum  ADF_DP_TRACE_ID {
	ADF_DP_TRACE_INVALID = 0,
	ADF_DP_TRACE_DROP_PACKET_RECORD,
	ADF_DP_TRACE_EAPOL_PACKET_RECORD,
	ADF_DP_TRACE_DHCP_PACKET_RECORD,
	ADF_DP_TRACE_ARP_PACKET_RECORD,
	ADF_DP_TRACE_DEFAULT_VERBOSITY,
	ADF_DP_TRACE_HDD_TX_TIMEOUT,
	ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT,
	ADF_DP_TRACE_HDD_PACKET_PTR_RECORD,
	ADF_DP_TRACE_CE_PACKET_PTR_RECORD,
	ADF_DP_TRACE_CE_FAST_PACKET_PTR_RECORD,
	ADF_DP_TRACE_FREE_PACKET_PTR_RECORD,
	ADF_DP_TRACE_LOW_VERBOSITY,
	ADF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD,
	ADF_DP_TRACE_TXRX_PACKET_PTR_RECORD,
	ADF_DP_TRACE_TXRX_FAST_PACKET_PTR_RECORD,
	ADF_DP_TRACE_HTT_PACKET_PTR_RECORD,
	ADF_DP_TRACE_HTC_PACKET_PTR_RECORD,
	ADF_DP_TRACE_HIF_PACKET_PTR_RECORD,
	ADF_DP_TRACE_MED_VERBOSITY,
	ADF_DP_TRACE_HDD_PACKET_RECORD,
	ADF_DP_TRACE_HIGH_VERBOSITY,
	ADF_DP_TRACE_MAX
};

enum adf_proto_dir {
	ADF_TX,
	ADF_RX
};

struct adf_dp_trace_ptr_buf {
	uint64_t cookie;
	uint16_t msdu_id;
	uint16_t status;
};

struct adf_dp_trace_proto_buf {
	struct adf_mac_addr sa;
	struct adf_mac_addr da;
	uint8_t vdev_id;
	uint8_t type;
	uint8_t subtype;
	uint8_t dir;
};


/**
 * struct adf_dp_trace_record_s - Describes a record in DP trace
 * @time: time when it got stored
 * @code: Describes the particular event
 * @data: buffer to store data
 * @size: Length of the valid data stored in this record
 * @pid : process id which stored the data in this record
 */
struct adf_dp_trace_record_s {
	uint64_t time;
	uint8_t code;
	uint8_t data[ADF_DP_TRACE_RECORD_SIZE];
	uint8_t size;
	uint32_t pid;
};

/**
 * struct adf_dp_trace_data - Parameters to configure/control DP trace
 * @head: Position of first record
 * @tail: Position of last record
 * @num:  Current index
 * @proto_bitmap: defines which protocol to be traced
 * @no_of_record: defines every nth packet to be traced
 * @verbosity : defines verbosity level
 * @enable: enable/disable DP trace
 * @count: current packet number
 */
struct s_adf_dp_trace_data {
	uint32_t head;
	uint32_t tail;
	uint32_t num;

	/* config for controlling the trace */
	uint8_t proto_bitmap;
	uint8_t no_of_record;
	uint8_t verbosity;
	bool enable;
	uint32_t count;
};
/* Function declarations and documenation */

#ifdef FEATURE_DPTRACE_ENABLE
void adf_dp_trace_init(void);
void adf_dp_trace_set_value(uint8_t proto_bitmap, uint8_t no_of_records,
			 uint8_t verbosity);
void adf_dp_trace_set_track(adf_nbuf_t nbuf);
void adf_dp_trace(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
			uint8_t *data, uint8_t size);
void adf_dp_trace_dump_all(uint32_t count);
typedef void (*tp_adf_dp_trace_cb)(struct adf_dp_trace_record_s* , uint16_t);
void adf_dp_display_record(struct adf_dp_trace_record_s *record,
							uint16_t index);
void adf_dp_trace_ptr(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
		uint8_t *data, uint8_t size, uint16_t msdu_id, uint16_t status);

void adf_dp_display_ptr_record(struct adf_dp_trace_record_s *pRecord,
				uint16_t recIndex);
uint8_t adf_dp_get_proto_bitmap(void);
void
adf_dp_trace_proto_pkt(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		uint8_t *sa, uint8_t *da, enum adf_proto_type type,
		enum adf_proto_subtype subtype, enum adf_proto_dir dir);
void adf_dp_display_proto_pkt(struct adf_dp_trace_record_s *record,
				uint16_t index);
void adf_dp_trace_log_pkt(uint8_t session_id, struct sk_buff *skb,
				uint8_t event_type);

#else
static inline void adf_dp_trace_init(void)
{
}

static inline void adf_dp_trace_set_value(uint8_t proto_bitmap,
		uint8_t no_of_records, uint8_t verbosity)
{
}

static inline void adf_dp_trace_set_track(adf_nbuf_t nbuf)
{
}

static inline void adf_dp_trace(adf_nbuf_t nbuf,
		enum ADF_DP_TRACE_ID code, uint8_t *data, uint8_t size)
{
}

static inline void adf_dp_trace_dump_all(uint32_t count)
{
}

static inline void adf_dp_display_record(struct adf_dp_trace_record_s *record,
							uint16_t index)
{
}

static inline void adf_dp_trace_ptr(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
		uint8_t *data, uint8_t size, uint16_t msdu_id, uint16_t status)
{
}

static inline void
adf_dp_display_ptr_record(struct adf_dp_trace_record_s *pRecord,
				uint16_t recIndex)
{
}

static inline uint8_t adf_dp_get_proto_bitmap(void)
{
    return 0;
}

static inline void
adf_dp_trace_proto_pkt(enum ADF_DP_TRACE_ID code, uint8_t vdev_id,
		uint8_t *sa, uint8_t *da, enum adf_proto_type type,
		enum adf_proto_subtype subtype, enum adf_proto_dir dir)
{
}

static inline void
adf_dp_display_proto_pkt(struct adf_dp_trace_record_s *record,
				uint16_t index)
{
}

static inline void adf_dp_trace_log_pkt(uint8_t session_id, struct sk_buff *skb,
				uint8_t event_type)
{
}

#endif

#endif  /* __ADF_TRACE_H */


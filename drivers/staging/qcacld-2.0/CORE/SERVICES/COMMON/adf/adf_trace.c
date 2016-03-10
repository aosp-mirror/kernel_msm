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

/**
 *  DOC:  adf_trace
 *
 *  ADF trace APIs
 *
 *  Trace, logging, and debugging definitions and APIs
 *
 */

 /* Include Files */
#include <adf_trace.h>
#include "adf_nbuf.h"
#include "adf_os_time.h"
#include "vos_trace.h"
#include "vos_packet.h"

/* Static and Global variables */
static spinlock_t l_dp_trace_lock;

static struct adf_dp_trace_record_s
			g_adf_dp_trace_tbl[MAX_ADF_DP_TRACE_RECORDS];

/*
 * all the options to configure/control DP trace are
 * defined in this structure
 */
static struct s_adf_dp_trace_data g_adf_dp_trace_data;
/*
 * all the call back functions for dumping DPTRACE messages from ring buffer
 * are stored in adf_dp_trace_cb_table, callbacks are initialized during init
 */
static tp_adf_dp_trace_cb adf_dp_trace_cb_table[ADF_DP_TRACE_MAX];

/**
 * adf_dp_trace_init() - enables the DP trace
 * Called during driver load and it enables DP trace
 *
 * Return: None
 */
void adf_dp_trace_init(void)
{
	uint8_t i;

	spin_lock_init(&l_dp_trace_lock);
	g_adf_dp_trace_data.head = INVALID_ADF_DP_TRACE_ADDR;
	g_adf_dp_trace_data.tail = INVALID_ADF_DP_TRACE_ADDR;
	g_adf_dp_trace_data.num = 0;
	g_adf_dp_trace_data.proto_bitmap = 0;
	g_adf_dp_trace_data.no_of_record = 0;
	g_adf_dp_trace_data.verbosity    = ADF_DP_TRACE_VERBOSITY_DEFAULT;
	g_adf_dp_trace_data.enable = true;

	for (i = 0; i < ADF_DP_TRACE_MAX; i++)
		adf_dp_trace_cb_table[i] = adf_dp_display_record;
}

/**
 * adf_dp_trace_set_value() - Configure the value to control DP trace
 * @proto_bitmap  : defines the protocol to be tracked
 * @no_of_records : defines the nth packet which is traced
 * @verbosity     : defines the verbosity level
 *
 * Return: None
 */
void adf_dp_trace_set_value(uint8_t proto_bitmap, uint8_t no_of_record,
			 uint8_t verbosity)
{
	spin_lock_bh(&l_dp_trace_lock);
	g_adf_dp_trace_data.proto_bitmap = proto_bitmap;
	g_adf_dp_trace_data.no_of_record = no_of_record;
	g_adf_dp_trace_data.verbosity    = verbosity;
	spin_unlock_bh(&l_dp_trace_lock);
}

/**
 * adf_dp_trace_enable_track() - enable the tracing for netbuf
 * @code : defines the event
 *
 * Return: true or false depends on whether tracing enabled
 */
static bool adf_dp_trace_enable_track(enum ADF_DP_TRACE_ID code)
{
	if (g_adf_dp_trace_data.verbosity == ADF_DP_TRACE_VERBOSITY_HIGH)
		return true;
	else if (g_adf_dp_trace_data.verbosity ==
		 ADF_DP_TRACE_VERBOSITY_MEDIUM &&
		(code <= ADF_DP_TRACE_HIF_PACKET_PTR_RECORD))
		return true;
	else if (g_adf_dp_trace_data.verbosity ==
		 ADF_DP_TRACE_VERBOSITY_LOW &&
		(code <= ADF_DP_TRACE_CE_PACKET_RECORD))
		return true;
	else if (g_adf_dp_trace_data.verbosity ==
		 ADF_DP_TRACE_VERBOSITY_DEFAULT &&
		(code == ADF_DP_TRACE_DROP_PACKET_RECORD))
		return true;
	return false;
}

/**
 * adf_dp_trace_set_track() - Marks whether the packet needs to be traced
 * @nbuf  : defines the netbuf
 *
 * Return: None
 */
void adf_dp_trace_set_track(adf_nbuf_t nbuf)
{
	spin_lock_bh(&l_dp_trace_lock);
	g_adf_dp_trace_data.count++;
	if (g_adf_dp_trace_data.proto_bitmap != 0) {
		if (vos_pkt_get_proto_type(nbuf,
		    g_adf_dp_trace_data.proto_bitmap, 0)) {
			ADF_NBUF_SET_DP_TRACE(nbuf, 1);
		}
	}
	if ((g_adf_dp_trace_data.no_of_record != 0) &&
	    (g_adf_dp_trace_data.count %
			g_adf_dp_trace_data.no_of_record == 0)) {
		ADF_NBUF_SET_DP_TRACE(nbuf, 1);
	}
	spin_unlock_bh(&l_dp_trace_lock);
}

/**
 * dump_hex_trace() - Display the data in buffer
 * @buf:     buffer which contains data to be displayed
 * @buf_len: defines the size of the data to be displayed
 *
 * Return: None
 */
static void dump_hex_trace(uint8_t *buf, uint8_t buf_len)
{
	uint8_t i;

	/* Dump the bytes in the last line */
	adf_os_print("DATA: ");
	for (i = 0; i < buf_len; i++)
		adf_os_print("%02x ", buf[i]);
	adf_os_print("\n");
}

/**
 * adf_dp_display_trace() - Displays a record in DP trace
 * @pRecord  : pointer to a record in DP trace
 * @recIndex : record index
 *
 * Return: None
 */
void adf_dp_display_record(struct adf_dp_trace_record_s *pRecord,
				uint16_t recIndex)
{
	adf_os_print("INDEX: %04d TIME: %012llu CODE: %02d\n", recIndex,
		     pRecord->time, pRecord->code);
	switch (pRecord->code) {
	case  ADF_DP_TRACE_HDD_TX_TIMEOUT:
		VOS_TRACE(VOS_MODULE_ID_ADF, VOS_TRACE_LEVEL_ERROR,
			  "HDD TX Timeout\n");
		break;
	case  ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT:
		VOS_TRACE(VOS_MODULE_ID_ADF, VOS_TRACE_LEVEL_ERROR,
			  "HDD SoftAP TX Timeout\n");
		break;
	case  ADF_DP_TRACE_VDEV_PAUSE:
		VOS_TRACE(VOS_MODULE_ID_ADF, VOS_TRACE_LEVEL_ERROR,
			  "VDEV Pause\n");
		break;
	case  ADF_DP_TRACE_VDEV_UNPAUSE:
		VOS_TRACE(VOS_MODULE_ID_ADF, VOS_TRACE_LEVEL_ERROR,
			  "VDEV UnPause\n");
		break;
	default:
		dump_hex_trace(pRecord->data, pRecord->size);
	}
}

/**
 * adf_dp_trace() - Stores the data in buffer
 * @nbuf  : defines the netbuf
 * @code : defines the event
 * @data : defines the data to be stored
 * @size : defines the size of the data record
 *
 * Return: None
 */
void adf_dp_trace(adf_nbuf_t nbuf, enum ADF_DP_TRACE_ID code,
			uint8_t *data, uint8_t size)
{
	struct adf_dp_trace_record_s *rec;

	/* Return when Dp trace is not enabled */
	if (!g_adf_dp_trace_data.enable)
		return;

	/* If nbuf is NULL, check for VDEV PAUSE, UNPAUSE, TIMEOUT */
	if (!nbuf) {
		switch (code) {
		case ADF_DP_TRACE_HDD_TX_TIMEOUT:
		case ADF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT:
		case ADF_DP_TRACE_VDEV_PAUSE:
		case ADF_DP_TRACE_VDEV_UNPAUSE:
			if (adf_dp_trace_enable_track(code))
				goto  register_record;
			else
				return;

		default:
			return;
		}
	}

	/* Return when the packet is not a data packet */
	if (NBUF_GET_PACKET_TRACK(nbuf) != NBUF_TX_PKT_DATA_TRACK)
		return;

	/* Return when nbuf is not marked for dp tracing or
	 * verbosity does not allow
	 */
	if ((adf_dp_trace_enable_track(code) == false) ||
	    !ADF_NBUF_GET_DP_TRACE(nbuf))
		return;

	/* Acquire the lock so that only one thread at a time can fill the ring
	 * buffer
	 */

register_record:

	spin_lock_bh(&l_dp_trace_lock);

	g_adf_dp_trace_data.num++;

	if (g_adf_dp_trace_data.num > MAX_ADF_DP_TRACE_RECORDS)
		g_adf_dp_trace_data.num = MAX_ADF_DP_TRACE_RECORDS;

	if (INVALID_ADF_DP_TRACE_ADDR == g_adf_dp_trace_data.head) {
		/* first record */
		g_adf_dp_trace_data.head = 0;
		g_adf_dp_trace_data.tail = 0;
	} else {
		/* queue is not empty */
		g_adf_dp_trace_data.tail++;

		if (MAX_ADF_DP_TRACE_RECORDS == g_adf_dp_trace_data.tail)
			g_adf_dp_trace_data.tail = 0;

		if (g_adf_dp_trace_data.head == g_adf_dp_trace_data.tail) {
			/* full */
			if (MAX_ADF_DP_TRACE_RECORDS ==
				++g_adf_dp_trace_data.head)
				g_adf_dp_trace_data.head = 0;
		}
	}

	rec = &g_adf_dp_trace_tbl[g_adf_dp_trace_data.tail];
	rec->code = code;
	rec->size = 0;
	if (data != NULL && size > 0) {
		if (size > ADF_DP_TRACE_RECORD_SIZE)
			size = ADF_DP_TRACE_RECORD_SIZE;

		rec->size = size;
		switch (code) {
		case ADF_DP_TRACE_HDD_PACKET_PTR_RECORD:
		case ADF_DP_TRACE_CE_PACKET_PTR_RECORD:
		case ADF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD:
		case ADF_DP_TRACE_TXRX_PACKET_PTR_RECORD:
		case ADF_DP_TRACE_HTT_PACKET_PTR_RECORD:
		case ADF_DP_TRACE_HTC_PACKET_PTR_RECORD:
		case ADF_DP_TRACE_HIF_PACKET_PTR_RECORD:
			adf_os_mem_copy(rec->data, (uint8_t *)(&data), size);
			break;

		case ADF_DP_TRACE_DROP_PACKET_RECORD:
		case ADF_DP_TRACE_HDD_PACKET_RECORD:
		case ADF_DP_TRACE_CE_PACKET_RECORD:
			adf_os_mem_copy(rec->data, data, size);
			break;
		default:
			break;
		}
	}
	rec->time = adf_os_gettimestamp();
	rec->pid = (in_interrupt() ? 0 : current->pid);
	spin_unlock_bh(&l_dp_trace_lock);
}

/**
 * adf_dp_trace_dump_all() - Dump data from ring buffer via call back functions
 *			  registered with ADF
 * @code : Reason code
 * @count : Number of lines to dump starting from tail to head
 *
 * Return : nothing
 */
void adf_dp_trace_dump_all(uint32_t count)
{
	struct adf_dp_trace_record_s pRecord;
	int32_t i, tail;

	if (!g_adf_dp_trace_data.enable) {
		VOS_TRACE(VOS_MODULE_ID_SYS,
			  VOS_TRACE_LEVEL_ERROR, "Tracing Disabled");
		return;
	}

	VOS_TRACE(VOS_MODULE_ID_SYS, VOS_TRACE_LEVEL_ERROR,
		  "Total Records: %d, Head: %d, Tail: %d",
		  g_adf_dp_trace_data.num, g_adf_dp_trace_data.head,
		  g_adf_dp_trace_data.tail);

	/* aquire the lock so that only one thread at a time can read
	 * the ring buffer
	 */
	spin_lock_bh(&l_dp_trace_lock);

	if (g_adf_dp_trace_data.head != INVALID_ADF_DP_TRACE_ADDR) {
		i = g_adf_dp_trace_data.head;
		tail = g_adf_dp_trace_data.tail;

		if (count) {
			if (count > g_adf_dp_trace_data.num)
				count = g_adf_dp_trace_data.num;
			if (tail >= (count - 1))
				i = tail - count + 1;
			else if (count != MAX_ADF_DP_TRACE_RECORDS)
				i = MAX_ADF_DP_TRACE_RECORDS - ((count - 1) -
							     tail);
		}

		pRecord = g_adf_dp_trace_tbl[i];
		spin_unlock_bh(&l_dp_trace_lock);
		for (;; ) {
			adf_dp_trace_cb_table[pRecord.
					   code] (&pRecord, (uint16_t)i);
			if (i == tail)
				break;
			i += 1;

			spin_lock_bh(&l_dp_trace_lock);
			if (MAX_ADF_DP_TRACE_RECORDS == i)
				i = 0;

			pRecord = g_adf_dp_trace_tbl[i];
			spin_unlock_bh(&l_dp_trace_lock);
		}
	} else {
		spin_unlock_bh(&l_dp_trace_lock);
	}
}

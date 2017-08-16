/*
 * Android/Easel coprocessor communication kernel-layer data structures
 * agreed upon by the drivers on both sides of the link (AP and Easel).
 *
 * Copyright 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _GOOGLE_EASEL_COMM_SHARED_H
#define _GOOGLE_EASEL_COMM_SHARED_H

#include <uapi/linux/google-easel-comm.h>
#include <linux/compiler.h>
#include <linux/types.h>

/*
 * Each side of the link allocates a command channel buffer in physically
 * contiguous memory (such that any portions can be addressed in a single iATU
 * region or single-block DMA transfer).  These are sometimes called
 * "ringbuffers", but in the current implementation flow control is performed
 * at each wraparound, effectively implementing a FIFO.
 *
 * Each driver uses PCIe remote writes to its peer's buffer to send commands
 * to the peer, and local memory access in the kernel virtual address space to
 * read the commands sent from the peer.
 *
 * Multi-byte integers in driver-layer data structures are sent across the link
 * in native format, which is assumed to be little-endian; endian conversion
 * would be needed for big endian systems.  It is assumed structure packing
 * rules are compatible on both sides of the link.
 */

/* command channel buffer size 100KB */
#define EASELCOMM_CMD_CHANNEL_SIZE      (100 * 1024)

/*
 * Command channel header at the top of each command channel buffer contains
 * the following info.
 *
 * Offset 0: Next producer sequence number (uint64_t 8 bytes Little Endian).
 * The producer (the remote peer) bumps this value each time a new command is
 * written to the buffer.  The consumer (the local driver) reads and processes
 * commands out of the buffer until its own "next sequence number" catches up
 * with this value.  The initial value is zero.  Sequence numbers cannot be
 * the CMD_BUFFER_WRAP_MARKER value defined below, which is reserved for
 * marking the buffer wraparound point.
 */
#define EASELCOMM_CMDCHAN_PRODUCER_SEQNBR       0

/* Alternatively can use this data structure for local access */
struct easelcomm_cmd_channel_header {
	uint64_t producer_seqnbr_next; /* next cmd seq# to be produced */
};

/* Size of command channel header in bytes */
#define EASELCOMM_CMDCHAN_HEADER_SIZE           8

/* easelcomm driver command codes */
enum {
	EASELCOMM_CMD_LINK_INIT = 0,      /* link partner init */
	EASELCOMM_CMD_LINK_SHUTDOWN,      /* link partner shutdown */
	EASELCOMM_CMD_ACK_SHUTDOWN,       /* link shutdown acknowledge */
	EASELCOMM_CMD_SEND_MSG,           /* send message */
	EASELCOMM_CMD_DMA_SG,             /* DMA scatter-gather list */
	EASELCOMM_CMD_DMA_XFER,           /* DMA transfer request */
	EASELCOMM_CMD_DMA_DONE,           /* inform DMA transfer done */
	EASELCOMM_CMD_FLUSH_SERVICE,      /* flush service messages */
	EASELCOMM_CMD_FLUSH_SERVICE_DONE, /* flush service done */
	EASELCOMM_CMD_CLOSE_SERVICE,      /* remote service handler closed */
};

/* Command channel producer wraparound marked by this invalid sequence # */
#define CMD_BUFFER_WRAP_MARKER  (0xFFFFFFFFULL)

/* Command header, precedes command-specific params (if any) */
struct easelcomm_cmd_header {
	uint64_t sequence_nbr;    /* seq# of command, or wrap marker */
	uint32_t service_id;      /* destination Easel service ID */
	uint32_t command_code;    /* easelcomm driver command code */
	uint32_t command_arg_len; /* # of bytes of command args that follow */
};

/* Kernel-layer Easel message descriptor plus appended message data */
struct easelcomm_kmsg {
	struct easelcomm_kmsg_desc desc;
	char message_data;             /* message data starts here */
};

/* SEND_MSG command argument is a struct easelcomm_msg */

/*
 * DMA direction, used in part to tell whether a mesasge id in a DMA_SG or
 * DMA_DONE argument refers to a client-allocated ID or a server-
 * allocated ID, and therefore also whether the associated message is in the
 * remote/incoming or local/outgoing message list maintained by the local
 * driver.
 */
enum easelcomm_dma_direction {
	EASELCOMM_DMA_DIR_TO_CLIENT,
	EASELCOMM_DMA_DIR_TO_SERVER,
};

/* DMA_SG command argument header */
struct easelcomm_dma_sg_header {
	easelcomm_msgid_t message_id; /* which message this SG list is for */
	uint32_t dma_dir;             /* which DMA direction */
	uint32_t scatterlist_size;    /* length of scatterlist that follows */
};
/* Above header is followed by a MNH driver scatter-gather list */

/* DMA_XFER command argument */
enum {
	EASELCOMM_DMA_XFER_SBLK,      /* single-block DMA request */
	EASELCOMM_DMA_XFER_MBLK,      /* multi-block DMA request */
	EASELCOMM_DMA_XFER_ABORT,     /* abort DMA request, error or discard */
};
struct easelcomm_dma_xfer_arg {
	easelcomm_msgid_t message_id; /* local or remote message ID */
	uint32_t dma_dir;             /* DMA direction tells local/remote */
	uint32_t xfer_type;           /* DMA transfer type */
	uint64_t server_addr;         /* MBLK LL addr or SBLK dest paddr */
};

/* DMA_DONE command argument */
struct easelcomm_dma_done_arg {
	easelcomm_msgid_t message_id; /* local or remote message ID */
	uint32_t dma_dir;             /* DMA direction tells local/remote */
	int32_t errcode;              /* DMA error code or zero if OK */
};

#endif /* _GOOGLE_EASEL_COMM_SHARED_H */

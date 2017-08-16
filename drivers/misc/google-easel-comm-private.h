/*
 * Android/Easel coprocessor communication hardware access to the PCIe/EP
 * driver on the local side of the link (AP or Easel), used by the common and
 * link-partner-specific code.
 *
 * Copyright 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _GOOGLE_EASEL_COMM_PRIVATE_H
#define _GOOGLE_EASEL_COMM_PRIVATE_H

#include "google-easel-comm-shared.h"

#include <linux/completion.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/*
 * Message types used to set the general type of processing for the message
 * and in which local data structures the message exists:
 * local: message originated locally with a local message ID, stored in the
 *    local message list for the associated Easel service.
 * remote non-reply: message received from remote with a remote message ID,
 *    stored in the remote message list, and also appears in the receive
 *    message queue until returned by a receiveMessage() call.
 * remote reply: message from remote that is processed as a reply to a local
 *    message, does not go into the receiveMessage() queue.
 */
enum easelcomm_msg_type {
	TYPE_LOCAL = 0,          /* local/outgoing */
	TYPE_REMOTE_NONREPLY,    /* remote/incoming general */
	TYPE_REMOTE_REPLY,       /* remote/incoming reply */
};

/*
 * Message metadata specific to DMA transfer handling.  Messages with no DMA
 * transfer have default values for these.
 */
struct easelcomm_dma_xfer_info {
	/* The local MNH driver scatter-gather list, or NULL if discarding */
	void *sg_local;
	/* Size in bytes of the local scatter-gather list, zero if discard */
	uint32_t sg_local_size;
	/* Local data private to the MNH layer associated with the SG list */
	void *sg_local_localdata;
	/*
	 * Easel/server keeps the remote scatter-gather list received from the
	 * client (in a DMA_SG command) here for processing.  The server is in
	 * charge of building the DMA Linked List for a multi-block transfer or
	 * deciding to use a single-block transfer based on the SG lists.
	 */
	void *sg_remote;
	/* Size in bytes of the remote SG list */
	uint32_t sg_remote_size;
	/* Informs server remote SG received from client */
	struct completion sg_remote_ready;
	/*
	 * Server determines DMA transfer type (SBLK/MBLK/ABORT) and sends to
	 * client to perform (or discard) the transfer.  Sent with DMA_XFER
	 * command.
	 */
	uint32_t xfer_type;
	/*
	 * The server's address applicable to the DMA transfer command:
	 * The Multi-Block Linked List physical address (in Easel memory) or
	 * the Single-Block server-side physical address of the transfer
	 * source or destination address).
	 */
	uint64_t server_addr;
	/*
	 * Informs client the DMA transfer request ready (DMA_XFER command
	 * received from server), ready to proceed (or discard).
	 */
	struct completion xfer_ready;
	/*
	 * Informs server DMA transfer is complete, DMA_DONE command received
	 * from client after transfer is complete or aborted.  Server can
	 * clean up message and DMA data structures and return to caller.
	 */
	struct completion xfer_done;
	/*
	 * DMA request is being aborted, either the remote sent an abort
	 * or locally a service flush or signal was received.
	 */
	bool aborting;
	/* DMA transfer error code (-errno) if non-zero */
	int32_t errcode;
};

/*
 * Message metadata, one for each local/outgoing and remote/incoming message
 * in progress on the local system.  Concurrency protection for all fields is
 * provided through the spinlock for the Easel service for which the message
 * is created.
 */
struct easelcomm_message_metadata {
	/*
	 * Message reference count, > 0 means kernel code is currently
	 * executing that holds a pointer to this message (metadata), or a
	 * data structure (other than the local/remote message lists)
	 * currently points to this message (namely a reply message pointed to
	 * by the replied-to message).  Zero reference count means the message
	 * can be safely removed from the local/remote list and freed (with the
	 * service lock held), which may happen outside the normal message
	 * lifecycle on a service flush.
	 */
	int reference_count;
	/*
	 * Set to true when the message is to be freed once the reference count
	 * goes to 0.
	 */
	bool free_message;
	/*
	 * True if message is marked to be flushed, lets DMA handling or
	 * reply waiter know to bail and let the message be flushed.
	 */
	bool flushing;
	/* what type of message: local, remote reply, remote non-reply */
	enum easelcomm_msg_type msg_type;
	/* local or remote message list linkage */
	struct list_head list;
	/*
	 * True if message is in the receiveMessage queue (a remote non-reply
	 * not yet picked up by a receiveMessage() call).
	 */
	bool queued;
	/* receiveMessage queue list linkage, if message is queued */
	struct list_head rcvq_list;
	/* signals reply message received, if message needs a reply */
	struct completion reply_received;
	/* pointer to reply message metadata once received, if need reply */
	struct easelcomm_message_metadata *reply_metadata;
	/* DMA info, if message includes a DMA transfer */
	struct easelcomm_dma_xfer_info dma_xfer;
	/* points to the Easel message desc and data */
	struct easelcomm_kmsg *msg;
};

/* forward declare for service/user state cross-references */
struct easelcomm_user_state;

/* Per-Easel-service data. */
struct easelcomm_service {
	/* service ID (same as easelcom_service array index) */
	unsigned int service_id;
	/* protects access to all mutable fields below and in messages */
	spinlock_t lock;
	/* currently registered user */
	struct easelcomm_user_state *user;
	/* true if service is being closed by local */
	bool shutdown_local;
	/* true if service was closed by remote */
	bool shutdown_remote;
	/* next local/outgoing message ID */
	easelcomm_msgid_t next_id;
	/* list of in-progress local/outgoing messages */
	struct list_head local_list;
	/* remote non-reply messages not yet returned by receiveMessage */
	struct list_head receivemsg_queue;
	/* signals new entry added to receivemsg_queue */
	struct completion receivemsg_queue_new;
	/* in-progress remote messages (reply and non-reply) */
	struct list_head remote_list;
	/* flush service complete on remote, ready to resume service */
	struct completion flush_done;
};

/*
 * Are we running as AP/client or Easel/server?  Most decisions are made
 * at runtime instead of compile time for code prettiness.
 */
#ifdef CONFIG_GOOGLE_EASEL_AP
#define easelcomm_is_client() (true)
#define easelcomm_is_server() (false)
#else
#define easelcomm_is_client() (false)
#define easelcomm_is_server() (true)
#endif

/* the easelcomm-{client,server} miscdevice for dev_*() messages */
extern struct miscdevice easelcomm_miscdev;

/* debug message util prefixes message ID by local or remote marker */
static inline char *easelcomm_msgid_prefix(
	struct easelcomm_message_metadata *msg_metadata)
{
	return msg_metadata->msg_type == TYPE_LOCAL ? "l" : "r";
}

/* DMA functions called by main */

/* Handle DMA_SG command */
extern void easelcomm_handle_cmd_dma_sg(
	struct easelcomm_service *service, char *command_args,
	int command_arg_len);
/* Handle DMA_XFER command */
extern void easelcomm_handle_cmd_dma_xfer(
	struct easelcomm_service *service, char *command_args,
	int command_arg_len);
/* Handle DMA_DONE command */
extern void easelcomm_handle_cmd_dma_done(
	struct easelcomm_service *service, char *command_args,
	int command_arg_len);
/* Handle RECVDMA ioctl */
extern int easelcomm_receive_dma(
	struct easelcomm_service *service,
	struct easelcomm_kbuf_desc *buf_desc);
/* Handle SENDDMA ioctl */
extern int easelcomm_send_dma(
	struct easelcomm_service *service,
	struct easelcomm_kbuf_desc *buf_desc);

/* Main functions called by DMA functions */

/* Drop a reference to a message, optionally mark for freeing */
extern void easelcomm_drop_reference(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata, bool mark_free);
/* Find a local/outgoing message by message ID */
extern struct easelcomm_message_metadata *easelcomm_find_local_message(
	struct easelcomm_service *service, easelcomm_msgid_t message_id);
/* Find a remote/incoming message by message ID */
extern struct easelcomm_message_metadata *easelcomm_find_remote_message(
	struct easelcomm_service *service, easelcomm_msgid_t message_id);
/* Start a new command to be sent to remote */
extern int easelcomm_start_cmd(
	struct easelcomm_service *service, int command_code,
	int command_arg_len);
/* Add command arguments to the in-progress command */
extern int easelcomm_append_cmd_args(
	struct easelcomm_service *service, void *cmd_args,
	size_t cmd_args_len);
/* Finish and send command to remote */
extern int easelcomm_send_cmd(struct easelcomm_service *service);
/* Start and send a new command with no arguments */
extern int easelcomm_send_cmd_noargs(
	struct easelcomm_service *service, int command_code);

/* Easel MNH coprocessor/PCIe hardware access functions */

/* register IRQ callbacks, etc. */
extern int easelcomm_hw_init(void);
/* AP retrieve Easel buffer address and setup EP iATU for both directions */
extern int easelcomm_hw_ap_setup_cmdchans(void);

/* send command channel data ready interrupt to remote */
extern int easelcomm_hw_send_data_interrupt(void);
/* send command channel wraparound interrupt to remote */
extern int easelcomm_hw_send_wrap_interrupt(void);

/* read remote memory */
extern int easelcomm_hw_remote_read(
	void *local_addr, size_t len, uint64_t remote_offset);
/* write remote memory */
extern int easelcomm_hw_remote_write(
	void *local_addr, size_t len, uint64_t remote_offset);

/* build an MNH DMA scatter-gather list */
extern void *easelcomm_hw_build_scatterlist(
	struct easelcomm_kbuf_desc *buf_desc,
	uint32_t *scatterlist_size,
	void **sglocaldata, enum easelcomm_dma_direction dma_dir);
/* get block count from SG list, for determing if multi- or single-block */
extern int easelcomm_hw_scatterlist_block_count(uint32_t scatterlist_size);
/* get start address from SG list for single-block transfer */
extern uint64_t easelcomm_hw_scatterlist_sblk_addr(void *sglist);
/* destroy (and unmap pages pinned by) a scatter-gather list */
extern void easelcomm_hw_destroy_scatterlist(void *sglocaldata);
/*
 * Easel builds multi-block DMA Linked List from local and remote SG lists.
 *
 * ll_data returns an opaque pointer passed to other calls.
 */
extern int easelcomm_hw_easel_build_ll(
	void *src_sg, void *dest_sg, void **ll_data);
/* Easel returns LL DMA address needed for multi-block transfer */
extern uint64_t easelcomm_hw_easel_ll_addr(void *ll_data);
/* Easel destroys MBLK DMA Linked List */
extern int easelcomm_hw_easel_destroy_ll(void *ll_data);
/* AP performs a single-block DMA transfer */
extern int easelcomm_hw_ap_dma_sblk_transfer(
	uint64_t ap_paddr, uint64_t easel_paddr, uint32_t xfer_len,
	bool to_easel);
/* AP performs a multi-block DMA transfer */
extern int easelcomm_hw_ap_dma_mblk_transfer(uint64_t ll_paddr, bool to_easel);

/* callbacks to main code from hardware-specific functions */

/* PCIe ready, local cmdchan buffer alloc'ed, init upper layer */
int easelcomm_init_pcie_ready(void *local_cmdchan_buffer);
/* Called when PCIe EP has been hotplug out */
int easelcomm_pcie_hotplug_out(void);
/* Handle command channel data received interrupt (MSG_SENT) */
extern void easelcomm_cmd_channel_data_handler(void);
/* Handle command channel wrapround interrupt (APPDEFINED_1) */
extern void easelcomm_cmd_channel_wrap_handler(void);
/* AP handles BOOTSTRAP_SET interrupt, Easel command channel is ready */
extern int easelcomm_client_remote_cmdchan_ready_handler(void);

#endif /* _GOOGLE_EASEL_COMM_PRIVATE_H */

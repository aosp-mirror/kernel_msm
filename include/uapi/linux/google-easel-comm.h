/*
 * Android/Easel coprocessor communication.
 *
 * Copyright 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _UAPI__GOOGLE_EASEL_COMM_H
#define _UAPI__GOOGLE_EASEL_COMM_H

#include <linux/compiler.h>
#include <linux/types.h>

/* Maximum message data size 12KB */
#define EASELCOMM_MAX_MESSAGE_SIZE      (12 * 1024)

/* Maximum service count */
#define EASELCOMM_SERVICE_COUNT 64

/* Easel message identifier.  Compatible with libeasel defines. */
typedef __u64 easelcomm_msgid_t;

struct easelcomm_wait {
	__s32 timeout_ms;           /* timeout in ms; -1 means indefinite */
};

/*
 * Userspace/kernel interface message descriptor.  libeasel converts
 * between its representation of messages and these descriptors, plus the
 * buffer descriptors below, when passing messages between the kernel and
 * userspace.
 */
struct easelcomm_kmsg_desc {
	/* 64-bit IDs go first for 32/64-bit struct packing conformity */
	easelcomm_msgid_t message_id;  /* message ID */
	easelcomm_msgid_t in_reply_to; /* msg ID replied to if non-zero */
	__u32 message_size;         /* size in bytes of the message data */
	__u32 dma_buf_size;         /* size of the DMA buffer transfer */
	__u32 need_reply;           /* non-zero if reply requested */
	__u32 replycode;            /* replycode if in_reply_to != 0 */
	struct easelcomm_wait wait;
};

enum easelcomm_dma_buffer_type {
	EASELCOMM_DMA_BUFFER_UNUSED = 0,
	EASELCOMM_DMA_BUFFER_USER,
	EASELCOMM_DMA_BUFFER_DMA_BUF
};

/*
 * Local buffer descriptor argument for ioctls that read and write message
 * or DMA buffers.  The message_id describes an in-progress local outgoing
 * message for which the message data is being written or the DMA source
 * buffer is being set, or a remote incoming message for which the message
 * data is being read or the DMA destination buffer is being set.
 */
struct easelcomm_kbuf_desc {
	easelcomm_msgid_t message_id;  /* ID of message for this transfer */
	void __user *buf;              /* local buffer source or dest */
	int dma_buf_fd;                /* fd of local dma_buf */
	int buf_type;                  /* use enum easelcomm_dma_buffer_type */
	__u32 buf_size;                /* size of the local buffer */
	struct easelcomm_wait wait;
};

/*
 * Kernel driver ioctls.  All ioctls return zero for success or -1 for
 * error with errno set: EINVAL for invalid parameters (dmesg may contain
 * further explanation), ENOMEM for out of memory, or other codes as described
 * below.
 */
#define EASELCOMM_IOC_MAGIC 0xEA

/*
 * Register the file descriptor for an easelcomm service.  The AP client will
 * flush any local client and remote server messages associated with the
 * service at this time, assuming any previous traffic is stale.  Registration
 * by the Easel server does not flush state and will commence processing of
 * any queued incoming messages previously sent to the service by a client.
 * Only one fd may be registered for a service on each of the AP and Easel
 * sides.
 */
#define EASELCOMM_IOC_REGISTER      _IOW(EASELCOMM_IOC_MAGIC, 0, int)

/*
 * Start sending an outgoing message.  Reads a message descriptor and
 * returns a modified descriptor with a local message ID assigned.  That
 * ID is then supplied in the buffer descriptors of WRITEBUF and SENDDMA
 * ioctls as appropriate to supply the message data and local DMA source
 * information.
 */
#define EASELCOMM_IOC_SENDMSG      _IOWR(EASELCOMM_IOC_MAGIC, 1, \
					struct easelcomm_kmsg_desc *)
/*
 * Read the message data for an incoming message received from the
 * remote.  The supplied remote message ID was returned by a previous
 * WAITMSG or WAITREPLY ioctl.  The data is read to the specified buffer
 * in the calling process.  If no DMA transfer is requested by the
 * associated message then successful completion of this ioctl frees the
 * local kernel copy of the message.
 */
#define EASELCOMM_IOC_READDATA      _IOW(EASELCOMM_IOC_MAGIC, 2, \
					struct easelcomm_kbuf_desc *)
/*
 * Write the message data for an outgoing message being sent to the
 * remote.  The supplied remote message ID was returned by a previous
 * SENDMSG ioctl.  The data is written from the specified buffer in the
 * calling process.  This ioctl sends the message to the remote upon
 * successful completion; even if the message data length is zero, this
 * ioctl must still follow a SENDMSG to actually send the message.  If no
 * DMA transfer is requested by the associated message then successful
 * completion of this ioctl frees the local kernel copy of the message.
 */
#define EASELCOMM_IOC_WRITEDATA     _IOW(EASELCOMM_IOC_MAGIC, 3, \
					struct easelcomm_kbuf_desc *)
/*
 * Initiate a DMA write for an outgoing message that includes a DMA
 * transfer.  The supplied local message ID was returned by a previous
 * SENDMSG ioctl.  The DMA data will be read from the specified buffer in the
 * calling process.  The ioctl returns once the DMA transfer is complete (or
 * skipped due to being discarded by the remote).  Successful completion of
 * this ioctl frees the local kernel copy of the message.
 */
#define EASELCOMM_IOC_SENDDMA       _IOW(EASELCOMM_IOC_MAGIC, 4, \
					struct easelcomm_kbuf_desc *)
/*
 * Specify the local destination for a DMA transfer requested by an incoming
 * message.  The supplied remote message ID was returned by a previous
 * WAITMSG or WAITREPLY ioctl.  The DMA data will be written to the specified
 * buffer in the calling process; or if a NULL buffer pointer is supplied then
 * the DMA transfer is discarded.  The ioctl returns once the DMA transfer is
 * complete (or discarded per the preceding).  Successful completion of this
 * ioctl frees the local kernel copy of the message.
 */
#define EASELCOMM_IOC_RECVDMA       _IOW(EASELCOMM_IOC_MAGIC, 5, \
					struct easelcomm_kbuf_desc *)
/*
 * Wait for and return a descriptor for a reply from the remote to a
 * local message that requests a reply.  The message ID in the supplied
 * descriptor was returned by a previous SENDMSG ioctl.  This ioctl waits
 * for the remote's reply message and returns a desciptor for that message.
 * A READDATA ioctl should then be issued to read the message data, followed
 * by a RECVDMA if a DMA transfer is requested by the reply.
 */
#define EASELCOMM_IOC_WAITREPLY    _IOWR(EASELCOMM_IOC_MAGIC, 6, \
					struct easelcomm_kmsg_desc *)
/*
 * Wait for and return a descriptor for an incoming message from the remote
 * (that is not a reply to a local message).  This ioctl waits for the next
 * non-reply message from the remote and returns a desciptor for that message.
 * A READDATA ioctl should then be issued to read the message data, followed
 * by a RECVDMA if a DMA transfer is requested by the message.
 * Returns error ESHUTDOWN if a SHUTDOWN ioctl is issued for the file
 * descriptor or the file descriptor has been closed.
 */
#define EASELCOMM_IOC_WAITMSG       _IOWR(EASELCOMM_IOC_MAGIC, 7, \
					struct easelcomm_kmsg_desc *)
/*
 * Shut down the local easelcomm connection for the given file descriptor.
 * Any other threads blocked on a WAITMSG using the same file descriptor will
 * return an ESHUTDOWN.  Any local in-progress outgoing or incoming messages
 * for the registered Easel service are discarded.
 */
#define EASELCOMM_IOC_SHUTDOWN       _IO(EASELCOMM_IOC_MAGIC, 8)

/*
 * Any local in-progress outgoing or incoming messages for the registered Easel
 * service are discarded, and the remote side is sent a command to flush its
 * local messages for the service.  Returns when the remote acknowledges the
 * flush complete.  No userspace handler is required to be registered on the
 * remote to perform the flush.
 */
#define EASELCOMM_IOC_FLUSH          _IO(EASELCOMM_IOC_MAGIC, 9)

/*
 * The last close() of an fd also flushes any local messages for the
 * registered service.
 */

#endif /* _UAPI__GOOGLE_EASEL_COMM_H */

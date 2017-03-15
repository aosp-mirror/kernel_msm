/*
 * Android/Easel coprocessor communication DMA routines.
 *
 * Copyright 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */

#include <uapi/linux/google-easel-comm.h>
#include "google-easel-comm-shared.h"
#include "google-easel-comm-private.h"

#include <linux/uaccess.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

/*
 * Server receives DMA scatter-gather list from client.  Store this and
 * wake up the local process handling the DMA request.
 */
void easelcomm_handle_cmd_dma_sg(
	struct easelcomm_service *service, char *command_args,
	int command_arg_len)
{
	struct easelcomm_dma_sg_header *sg_header;
	void *cmd_sg;
	struct easelcomm_message_metadata *msg_metadata;

	if (WARN_ON(command_arg_len < sizeof(struct easelcomm_dma_sg_header)))
		return;

	sg_header = (struct easelcomm_dma_sg_header *) command_args;
	command_args += sizeof(struct easelcomm_dma_sg_header);
	command_arg_len -= sizeof(struct easelcomm_dma_sg_header);

	if (WARN_ON(command_arg_len < sg_header->scatterlist_size))
		return;

	if (sg_header->dma_dir == EASELCOMM_DMA_DIR_TO_CLIENT) {
		msg_metadata =
			easelcomm_find_local_message(
				service, sg_header->message_id);
	} else {
		msg_metadata =
			easelcomm_find_remote_message(
				service, sg_header->message_id);
	}

	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device,
			"DMA_SG msg %u:%s%llu not found\n",
			service->service_id,
			sg_header->dma_dir == EASELCOMM_DMA_DIR_TO_CLIENT ?
			"l" : "r", sg_header->message_id);
		return;
	}

	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd DMA_SG msg %u:%s%llu size=%u\n",
		service->service_id,
		easelcomm_msgid_prefix(msg_metadata), sg_header->message_id,
		sg_header->scatterlist_size);

	if (WARN_ON(msg_metadata->dma_xfer.sg_remote))
		goto out;

	msg_metadata->dma_xfer.sg_remote_size = sg_header->scatterlist_size;

	if (sg_header->scatterlist_size) {
		cmd_sg = command_args;
		msg_metadata->dma_xfer.sg_remote =
			kmalloc(sg_header->scatterlist_size, GFP_KERNEL);
		if (WARN_ON(!msg_metadata->dma_xfer.sg_remote)) {
			msg_metadata->dma_xfer.sg_remote_size = 0;
			goto out;
		}

		memcpy(msg_metadata->dma_xfer.sg_remote, cmd_sg,
			sg_header->scatterlist_size);
	}

	/* Let the local waiter know the remote scatterlist ready. */
	complete(&msg_metadata->dma_xfer.sg_remote_ready);

out:
	easelcomm_drop_reference(service, msg_metadata, false);
}
EXPORT_SYMBOL(easelcomm_handle_cmd_dma_sg);

/*
 * Client receives DMA transfer instructions from server.  Server has chosen
 * single- vs. multi-block as necessary, and has provided either the single
 * block server-side source or destination address, or the multi-block
 * linked-list address.  Wake up the local process to perform the DMA
 * transfer.
 */
void easelcomm_handle_cmd_dma_xfer(
	struct easelcomm_service *service, char *command_args,
	int command_arg_len)
{
	struct easelcomm_dma_xfer_arg *dma_xfer;
	struct easelcomm_message_metadata *msg_metadata;

	if (WARN_ON(command_arg_len <
			sizeof(struct easelcomm_dma_xfer_arg)))
		return;

	dma_xfer = (struct easelcomm_dma_xfer_arg *) command_args;

	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd DMA_XFER msg %u:%s%llu type=%u saddr=%llx\n",
		service->service_id,
		dma_xfer->dma_dir == EASELCOMM_DMA_DIR_TO_SERVER ? "l" : "r",
		dma_xfer->message_id, dma_xfer->xfer_type,
		dma_xfer->server_addr);

	if (dma_xfer->dma_dir == EASELCOMM_DMA_DIR_TO_SERVER) {
		msg_metadata =
			easelcomm_find_local_message(
				service, dma_xfer->message_id);
	} else {
		msg_metadata =
			easelcomm_find_remote_message(
				service, dma_xfer->message_id);
	}

	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device,
			"DMA_XFER msg %u:%s%llu not found\n",
			service->service_id,
			dma_xfer->dma_dir == EASELCOMM_DMA_DIR_TO_SERVER ?
			"l" : "r", dma_xfer->message_id);
		return;
	}

	msg_metadata->dma_xfer.xfer_type = dma_xfer->xfer_type;
	msg_metadata->dma_xfer.server_addr = dma_xfer->server_addr;

	/* Let the local waiter know the DMA request is received */
	complete(&msg_metadata->dma_xfer.xfer_ready);
	easelcomm_drop_reference(service, msg_metadata, false);
}
EXPORT_SYMBOL(easelcomm_handle_cmd_dma_xfer);

/*
 * Server receives DMA done indication from client.
 * Safe to unmap buffers and return to waiting DMA originator or receiver.
 */
void easelcomm_handle_cmd_dma_done(
	struct easelcomm_service *service, char *command_args,
	int command_arg_len)
{
	struct easelcomm_dma_done_arg *dma_done_arg;
	struct easelcomm_message_metadata *msg_metadata;

	if (WARN_ON(command_arg_len < sizeof(struct easelcomm_dma_done_arg)))
		return;

	dma_done_arg = (struct easelcomm_dma_done_arg *) command_args;
	dev_dbg(easelcomm_miscdev.this_device,
		"recv cmd DMA_DONE msg %u:%s%llu err=%u\n",
		service->service_id,
		dma_done_arg->dma_dir == EASELCOMM_DMA_DIR_TO_CLIENT ?
		"l" : "r", dma_done_arg->message_id, dma_done_arg->errcode);

	if (dma_done_arg->dma_dir == EASELCOMM_DMA_DIR_TO_CLIENT) {
		msg_metadata =
			easelcomm_find_local_message(
				service, dma_done_arg->message_id);
	} else {
		msg_metadata =
			easelcomm_find_remote_message(
				service, dma_done_arg->message_id);
	}

	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device,
			"CMD_DMA_DONE msg %u:%s%llu not found\n",
			service->service_id,
			dma_done_arg->dma_dir == EASELCOMM_DMA_DIR_TO_CLIENT ?
			"l" : "r", dma_done_arg->message_id);
		return;
	}

	msg_metadata->dma_xfer.errcode = dma_done_arg->errcode;
	/*
	 * Wakeup local waiter waiting on remote SG (if this is an abort) or
	 * DMA completion.
	 */
	complete(&msg_metadata->dma_xfer.sg_remote_ready);
	complete(&msg_metadata->dma_xfer.xfer_done);
	easelcomm_drop_reference(service, msg_metadata, false);
}
EXPORT_SYMBOL(easelcomm_handle_cmd_dma_done);

static void *easelcomm_create_dma_scatterlist(
	struct easelcomm_kbuf_desc *buf_desc, uint32_t *scatterlist_size,
	void **sglocaldata, enum easelcomm_dma_direction dma_dir)
{
	return easelcomm_hw_build_scatterlist(buf_desc, scatterlist_size,
		sglocaldata, dma_dir);
}

/*
 * Client sends its local DMA dest scatter-gather list to the server, which
 * will use this info to choose a single- or multi-block transfer, and build
 * an MNH DMA Linked List structure based on the local and remote
 * scatter-gather lists, if needed.
 *
 * If the client is discarding the DMA transfer then a zero-length
 * scatter-gather list is sent.
 */
static int easelcomm_send_dma_scatterlist(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata,
	enum easelcomm_dma_direction dma_dir)
{
	struct easelcomm_dma_sg_header sg_header;
	int ret;

	sg_header.message_id = msg_metadata->msg->desc.message_id;
	sg_header.dma_dir = dma_dir;
	sg_header.scatterlist_size = msg_metadata->dma_xfer.sg_local_size;

	dev_dbg(easelcomm_miscdev.this_device,
		"send cmd DMA_SG msg %u:%s%llu size=%u\n",
		service->service_id, easelcomm_msgid_prefix(msg_metadata),
		msg_metadata->msg->desc.message_id,
		msg_metadata->dma_xfer.sg_local_size);
	ret = easelcomm_start_cmd(
		service, EASELCOMM_CMD_DMA_SG,
		sizeof(struct easelcomm_dma_sg_header) +
		msg_metadata->dma_xfer.sg_local_size);
	if (ret)
		return ret;

	ret = easelcomm_append_cmd_args(
		service, &sg_header, sizeof(struct easelcomm_dma_sg_header));
	if (ret)
		return ret;

	/* If not discarding locally append the scatterlist */
	if (msg_metadata->dma_xfer.sg_local_size) {
		ret = easelcomm_append_cmd_args(
			service, msg_metadata->dma_xfer.sg_local,
			msg_metadata->dma_xfer.sg_local_size);
		if (ret)
			return ret;
	}

	return easelcomm_send_cmd(service);
}

/* Server sends DMA_XFER command to client */
static int easelcomm_send_dma_xfer(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata,
	enum easelcomm_dma_direction dma_dir)
{
	struct easelcomm_dma_xfer_arg dma_xfer;
	int ret;

	dma_xfer.message_id = msg_metadata->msg->desc.message_id;
	dma_xfer.dma_dir = dma_dir;
	dma_xfer.xfer_type = msg_metadata->dma_xfer.xfer_type;
	dma_xfer.server_addr = msg_metadata->dma_xfer.server_addr;

	ret = easelcomm_start_cmd(
		service, EASELCOMM_CMD_DMA_XFER,
		sizeof(struct easelcomm_dma_xfer_arg));
	if (ret)
		return ret;

	ret = easelcomm_append_cmd_args(
		service, &dma_xfer, sizeof(struct easelcomm_dma_xfer_arg));
	if (ret)
		return ret;
	dev_dbg(easelcomm_miscdev.this_device, "send cmd DMA_XFER msg %u:%s%llu type=%u saddr=%llx\n",
		service->service_id,
		easelcomm_msgid_prefix(msg_metadata),
		msg_metadata->msg->desc.message_id,
		msg_metadata->dma_xfer.xfer_type,
		msg_metadata->dma_xfer.server_addr);
	return easelcomm_send_cmd(service);
}

/*
 * Server-side DMA handling.  When the client sends its scatterlist, inspect
 * the client and server lists to see if the DMA can be a single-block
 * transfer or whether it must be a multi-block transfer.  Construct the
 * multi-block linked list if needed.  Tell the client to proceed with a
 * single-block or multi-block transfer, wait for DMA done command from
 * client, and clean up.
 */
static int easelcomm_server_handle_dma_request(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata,
	enum easelcomm_dma_direction dma_dir)
{
	void *mblk_ll_data = NULL;
	bool discard = false;
	int ret;

	/* Wait for client to send its scatterlist, if not already */
	ret = wait_for_completion_interruptible(
		&msg_metadata->dma_xfer.sg_remote_ready);
	if (ret || msg_metadata->dma_xfer.aborting)
		goto abort;

	if (!msg_metadata->dma_xfer.sg_remote_size) {
		dev_dbg(easelcomm_miscdev.this_device,
			"client discards DMA for msg %u:%s%llu\n",
			service->service_id,
			easelcomm_msgid_prefix(msg_metadata),
			msg_metadata->msg->desc.message_id);
		return 0;
	}

	/* If discarding locally then send an abort to client. */
	if (msg_metadata->dma_xfer.sg_local_size == 0) {
		discard = true;
		goto abort;
	}

	/*
	 * Choose SBLK vs. MBLK DMA based on whether scatterlists need more
	 * than one block.
	 */
	if (easelcomm_hw_scatterlist_block_count(
			msg_metadata->dma_xfer.sg_local_size) == 1 &&
		easelcomm_hw_scatterlist_block_count(
			msg_metadata->dma_xfer.sg_remote_size) == 1) {
		/*
		 * Single-block DMA.  server_addr for the DMA_XFER command
		 * is the Easel-side starting physical address.
		 */
		msg_metadata->dma_xfer.xfer_type = EASELCOMM_DMA_XFER_SBLK;
		msg_metadata->dma_xfer.server_addr =
			easelcomm_hw_scatterlist_sblk_addr(
				msg_metadata->dma_xfer.sg_local);
	} else {
		/*
		 * Multiple-block DMA.	server_addr is the physical address
		 * of the Linked List we create here.
		 */
		msg_metadata->dma_xfer.xfer_type = EASELCOMM_DMA_XFER_MBLK;

		/* Build DMA Linked List for the transfer*/
		if (dma_dir == EASELCOMM_DMA_DIR_TO_SERVER)
			ret = easelcomm_hw_easel_build_ll(
				msg_metadata->dma_xfer.sg_remote,
				msg_metadata->dma_xfer.sg_local,
				&mblk_ll_data);
		else
			ret = easelcomm_hw_easel_build_ll(
				msg_metadata->dma_xfer.sg_local,
				msg_metadata->dma_xfer.sg_remote,
				&mblk_ll_data);

		/* If error, tell client to abort DMA transfer */
		if (ret < 0 || !mblk_ll_data)
			goto abort;
		msg_metadata->dma_xfer.server_addr =
			easelcomm_hw_easel_ll_addr(mblk_ll_data);
	}

	/* Send DMA_XFER to client */
	ret = easelcomm_send_dma_xfer(service, msg_metadata, dma_dir);
	if (ret)
		goto abort;

	/* Wait for client to send a DMA Done for this message */
	ret = wait_for_completion_interruptible(
		&msg_metadata->dma_xfer.xfer_done);
	if (ret || msg_metadata->dma_xfer.aborting)
		goto abort;

	ret = msg_metadata->dma_xfer.errcode;
	goto out;

abort:
	dev_dbg(easelcomm_miscdev.this_device,
		"aborting DMA for msg %u:%s%llu\n",
		service->service_id, easelcomm_msgid_prefix(msg_metadata),
		msg_metadata->msg->desc.message_id);
	msg_metadata->dma_xfer.aborting = true;
	/* Send a DMA abort to remote */
	msg_metadata->dma_xfer.xfer_type = EASELCOMM_DMA_XFER_ABORT;
	ret = easelcomm_send_dma_xfer(service, msg_metadata, dma_dir);
	if (ret)
		dev_err(easelcomm_miscdev.this_device,
			"send DMA abort for msg %u:%s%llu failed: %d\n",
			service->service_id,
			easelcomm_msgid_prefix(msg_metadata),
			msg_metadata->msg->desc.message_id, ret);
	ret = discard ? 0 : -EIO;

	/*
	 * TODO-LATER: Need to make sure DMA hardware no longer refers to
	 * LL before destroy.  Related: Ensure DMA hardware no longer
	 * accessing the locally-pinned memory pages prior to SG unmap.
	 */

out:
	/* Destroy LL if allocated for MBLK DMA */
	if (mblk_ll_data)
		easelcomm_hw_easel_destroy_ll(mblk_ll_data);
	return ret;
}

/* Client sends DMA Done command to server to let it know it can clean up. */
static int easelcomm_send_dma_done(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata,
	enum easelcomm_dma_direction dma_dir, int32_t errcode)
{
	struct easelcomm_dma_done_arg dma_done_arg;
	int ret = 0;

	/* Send DMA Done command to server */
	dma_done_arg.message_id =
		msg_metadata->msg->desc.message_id;
	dma_done_arg.dma_dir = dma_dir;
	dma_done_arg.errcode = errcode;
	ret = easelcomm_start_cmd(
		service, EASELCOMM_CMD_DMA_DONE,
		sizeof(struct easelcomm_dma_done_arg));
	if (ret)
		return ret;
	ret = easelcomm_append_cmd_args(
		service, &dma_done_arg, sizeof(struct easelcomm_dma_done_arg));
	if (ret)
		return ret;
	dev_dbg(easelcomm_miscdev.this_device,
		"send cmd DMA_DONE msg %u:%s%llu errcode=%d\n",
		service->service_id, easelcomm_msgid_prefix(msg_metadata),
		msg_metadata->msg->desc.message_id, errcode);
	ret = easelcomm_send_cmd(service);
	return ret;
}

/* Client performs single-block DMA transfer */
static int easelcomm_client_perform_dma_sblk(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata,
	enum easelcomm_dma_direction dir)
{
	uint64_t client_addr = easelcomm_hw_scatterlist_sblk_addr(
		msg_metadata->dma_xfer.sg_local);

	dev_dbg(easelcomm_miscdev.this_device, "sblk dma msg %u:%s%llu dir=%d size=%u local=%llx saddr=%llx\n",
		service->service_id, easelcomm_msgid_prefix(msg_metadata),
		msg_metadata->msg->desc.message_id, dir,
		msg_metadata->msg->desc.dma_buf_size, client_addr,
		msg_metadata->dma_xfer.server_addr);
	return easelcomm_hw_ap_dma_sblk_transfer(
		client_addr, msg_metadata->dma_xfer.server_addr,
		msg_metadata->msg->desc.dma_buf_size,
		dir == EASELCOMM_DMA_DIR_TO_SERVER);
}

/* Client performs multi-block DMA transfer */
static int easelcomm_client_perform_dma_mblk(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata,
	enum easelcomm_dma_direction dir)
{
	dev_dbg(easelcomm_miscdev.this_device, "mblk dma msg %u:%s%llu dir=%d size=%u ll=%llx\n",
		service->service_id, easelcomm_msgid_prefix(msg_metadata),
		msg_metadata->msg->desc.message_id, dir,
		msg_metadata->msg->desc.dma_buf_size,
		msg_metadata->dma_xfer.server_addr);
	return easelcomm_hw_ap_dma_mblk_transfer(
		msg_metadata->dma_xfer.server_addr,
		dir == EASELCOMM_DMA_DIR_TO_SERVER);
}

/*
 * Client handles DMA transfer.  Send local scatterlist to server, wait for
 * instructions back on how to proceed with single- or multi-block transfer,
 * perform the transfer, and send the DMA done command to server.
 */
static int easelcomm_client_handle_dma_request(
	struct easelcomm_service *service,
	struct easelcomm_message_metadata *msg_metadata,
	enum easelcomm_dma_direction dma_dir)
{
	int ret, ret2;

	/* Send scatterlist to server in a DMA_SG command */
	ret = easelcomm_send_dma_scatterlist(
		service, msg_metadata, dma_dir);
	if (ret)
		return ret;

	/* If sent a DMA discard then done */
	if (!msg_metadata->dma_xfer.sg_local_size)
		return 0;

	/* Wait for server to return the DMA request info */
	ret = wait_for_completion_interruptible(
		&msg_metadata->dma_xfer.xfer_ready);
	if (ret || msg_metadata->dma_xfer.aborting)
		goto abort;

	/*
	 * If server discards/aborts the DMA request then done.  If
	 * this is a client DMA send then silently discard.  If the
	 * client is receiving a DMA buffer then return an error on
	 * the read.
	 */
	if (msg_metadata->dma_xfer.xfer_type == EASELCOMM_DMA_XFER_ABORT)
		return dma_dir == EASELCOMM_DMA_DIR_TO_CLIENT ? -EIO : 0;

	/* Single-Block or Multi-Block DMA? */
	if (msg_metadata->dma_xfer.xfer_type == EASELCOMM_DMA_XFER_SBLK) {
		/* Perform the SBLK DMA transfer */
		ret = easelcomm_client_perform_dma_sblk(
			service, msg_metadata, dma_dir);
	} else {
		/* Perform the MBLK DMA transfer */
		ret = easelcomm_client_perform_dma_mblk(
			service, msg_metadata, dma_dir);
	}

	/* Tell server DMA transfer is done */
	ret2 = easelcomm_send_dma_done(service, msg_metadata, dma_dir, ret);
	return ret || ret2;

	/*
	 * We are aborting the transfer before it started (local flush or
	 * signal received)
	 */
abort:
	msg_metadata->dma_xfer.aborting = true;
	/* send abort to server */
	msg_metadata->dma_xfer.xfer_type = EASELCOMM_DMA_XFER_ABORT;
	easelcomm_send_dma_done(service, msg_metadata, dma_dir, -EIO);
	return -EIO;
}

/* RECVDMA ioctl processing. */
int easelcomm_receive_dma(
	struct easelcomm_service *service,
	struct easelcomm_kbuf_desc *buf_desc)
{
	struct easelcomm_message_metadata *msg_metadata;
	enum easelcomm_dma_direction dma_dir;
	int ret = 0;

	dev_dbg(easelcomm_miscdev.this_device,
			"RECVDMA msg %u:r%llu buf_type=%d dma_buf_fd=%d\n",
			service->service_id, buf_desc->message_id,
			buf_desc->buf_type, buf_desc->dma_buf_fd);

	msg_metadata =
		easelcomm_find_remote_message(service, buf_desc->message_id);
	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device, "RECVDMA msg r%llu not found\n",
			buf_desc->message_id);
		return -EINVAL;
	}

	if (buf_desc->buf_size &&
	    buf_desc->buf_size != msg_metadata->msg->desc.dma_buf_size) {
		dev_err(easelcomm_miscdev.this_device,
			"RECVDMA descriptor buffer size %u doesn't match msg r%llu size %u\n",
			buf_desc->buf_size, buf_desc->message_id,
			msg_metadata->msg->desc.dma_buf_size);
		ret = -EINVAL;
		goto out;
	}

	/* If the message doesn't even request a DMA transfer then skip it. */

	if (!msg_metadata->msg->desc.dma_buf_size)
		goto out;

	/* If it's a user buffer, check valid range and writable. */
	if (buf_desc->buf_type == EASELCOMM_DMA_BUFFER_USER) {
		if (!access_ok(VERIFY_WRITE, buf_desc->buf,
			       buf_desc->buf_size)) {
			ret = -EFAULT;
			goto out;
		}
	}
	dma_dir = easelcomm_is_client() ?
		EASELCOMM_DMA_DIR_TO_CLIENT : EASELCOMM_DMA_DIR_TO_SERVER;

	/*
	 * If the DMA transfer is not being discarded locally then generate
	 * the local DMA scatter-gather list.
	 */
	msg_metadata->dma_xfer.sg_local_size = 0;
	msg_metadata->dma_xfer.sg_local_localdata = NULL;

	if (buf_desc->buf_size) {
		msg_metadata->dma_xfer.sg_local =
			easelcomm_create_dma_scatterlist(
				buf_desc,
				&msg_metadata->dma_xfer.sg_local_size,
				&msg_metadata->dma_xfer.sg_local_localdata,
				dma_dir);
		if (!msg_metadata->dma_xfer.sg_local) {
			ret = -ENOMEM;
			msg_metadata->dma_xfer.sg_local_size = 0;
			goto out;
		}
	}

	if (easelcomm_is_client())
		ret = easelcomm_client_handle_dma_request(
			service, msg_metadata, dma_dir);
	else
		ret = easelcomm_server_handle_dma_request(
			service, msg_metadata, dma_dir);

	/* Free contents of struct mnh_sg_list */
	if (msg_metadata->dma_xfer.sg_local_localdata)
		easelcomm_hw_destroy_scatterlist(
			msg_metadata->dma_xfer.sg_local_localdata);

out:
	/* If no reply needed then done with the remote message. */
	easelcomm_drop_reference(service, msg_metadata,
				!msg_metadata->msg->desc.need_reply);
	return ret;
}
EXPORT_SYMBOL(easelcomm_receive_dma);

/* SENDDMA ioctl processing. */
int easelcomm_send_dma(
	struct easelcomm_service *service,
	struct easelcomm_kbuf_desc *buf_desc)
{
	struct easelcomm_message_metadata *msg_metadata;
	enum easelcomm_dma_direction dma_dir;
	int ret = 0;

	dev_dbg(easelcomm_miscdev.this_device,
			"SENDDMA msg %u:l%llu buf_type=%d dma_buf_fd=%d\n",
			service->service_id, buf_desc->message_id,
			buf_desc->buf_type, buf_desc->dma_buf_fd);

	msg_metadata =
		easelcomm_find_local_message(service, buf_desc->message_id);
	if (!msg_metadata) {
		dev_err(easelcomm_miscdev.this_device, "SENDDMA msg l%llu not found\n",
			buf_desc->message_id);
		return -EINVAL;
	}

	if (buf_desc->buf_size &&
		buf_desc->buf_size != msg_metadata->msg->desc.dma_buf_size) {
		dev_err(easelcomm_miscdev.this_device,
			"SENDDMA descriptor buffer size %u doesn't match message l%llu size %u\n",
			buf_desc->buf_size, buf_desc->message_id,
			msg_metadata->msg->desc.dma_buf_size);
		ret = -EINVAL;
		goto out;
	}

	/* If it's a user buffer, check valid range and readable. */
	if (buf_desc->buf_type == EASELCOMM_DMA_BUFFER_USER) {
		if (!access_ok(VERIFY_READ, buf_desc->buf,
			       buf_desc->buf_size)) {
			ret = -EFAULT;
			goto out;
		}
	}

	dma_dir = easelcomm_is_client() ?
		EASELCOMM_DMA_DIR_TO_SERVER : EASELCOMM_DMA_DIR_TO_CLIENT;

	/*
	 * If the DMA transfer is not being discarded locally then generate
	 * the local DMA scatter-gather list.
	 */
	msg_metadata->dma_xfer.sg_local_size = 0;
	msg_metadata->dma_xfer.sg_local_localdata = NULL;
	if (buf_desc->buf_size) {
		msg_metadata->dma_xfer.sg_local =
			easelcomm_create_dma_scatterlist(
				buf_desc,
				&msg_metadata->dma_xfer.sg_local_size,
				&msg_metadata->dma_xfer.sg_local_localdata,
				dma_dir);
		if (!msg_metadata->dma_xfer.sg_local) {
			ret = -ENOMEM;
			goto out;
		}
	}

	if (msg_metadata->msg->desc.dma_buf_size) {
		if (easelcomm_is_client())
			ret = easelcomm_client_handle_dma_request(
				service, msg_metadata, dma_dir);
		else
			ret = easelcomm_server_handle_dma_request(
				service, msg_metadata, dma_dir);
	}

	/* Free contents of struct mnh_sg_list */
	if (msg_metadata->dma_xfer.sg_local_localdata)
		easelcomm_hw_destroy_scatterlist(
			msg_metadata->dma_xfer.sg_local_localdata);

out:
	/* If no reply needed then done with local message. */
	easelcomm_drop_reference(service, msg_metadata,
		!msg_metadata->msg->desc.need_reply);
	return ret;
}
EXPORT_SYMBOL(easelcomm_send_dma);

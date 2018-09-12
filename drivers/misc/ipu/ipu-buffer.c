/*
 * Buffer management support for the Paintbox programmable IPU
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/ab-dram.h>
#include <linux/ipu-core.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <uapi/ipu.h>

#include "ipu-buffer.h"
#include "ipu-client.h"

/* The caller to this function must hold pb->lock */
static inline struct paintbox_buffer *ipu_buffer_get_buffer(
		struct paintbox_session *session, uint32_t buffer_id)
{
	return (struct paintbox_buffer *)idr_find(&session->buffer_idr,
			buffer_id);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_alloc(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer **buffer)
{
	struct paintbox_buffer *tmp;
	int ret;

	tmp = kzalloc(sizeof(struct paintbox_buffer), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = idr_alloc(&session->buffer_idr, tmp, 0, PAINTBOX_BUFFER_ID_MAX,
			GFP_KERNEL);
	if (ret < 0) {
		dev_err(pb->dev, "%s: error allocating buffer id, err = %d\n",
				__func__, ret);
		kfree(tmp);

		return ret;
	}

	tmp->buffer_id = ret;

	*buffer = tmp;

	return 0;
}

/* The caller to this function must hold pb->lock */
static void ipu_buffer_release_buffer(struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	idr_remove(&session->buffer_idr, buffer->buffer_id);
	kfree(buffer);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_map_dma_buf(struct paintbox_data *pb,
		struct paintbox_session *session, int dma_buf_fd,
		enum dma_data_direction dir, struct paintbox_buffer *buffer)
{
	int ret;

	buffer->dir = dir;

	buffer->dma_buf = dma_buf_get(dma_buf_fd);
	if (IS_ERR(buffer->dma_buf))
		return PTR_ERR(buffer->dma_buf);

	buffer->attach = dma_buf_attach(buffer->dma_buf, pb->dev);
	if (IS_ERR(buffer->attach)) {
		ret = PTR_ERR(buffer->attach);
		dev_err(pb->dev, "%s: failed to attach dma_buf, err %d\n",
				__func__, ret);
		goto err_put;
	}

	buffer->sg_table = dma_buf_map_attachment(buffer->attach, dir);
	if (IS_ERR(buffer->sg_table)) {
		ret = PTR_ERR(buffer->sg_table);
		dev_err(pb->dev, "%s: failed to map dma_buf, err %d\n",
				__func__, ret);
		goto err_detach;
	}

	return 0;

err_detach:
	dma_buf_detach(buffer->dma_buf, buffer->attach);
err_put:
	dma_buf_put(buffer->dma_buf);

	return ret;
}

/* The caller to this function must hold pb->lock */
static void ipu_buffer_unmap_dma_buf(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	dma_buf_unmap_attachment(buffer->attach, buffer->sg_table, buffer->dir);
	dma_buf_detach(buffer->dma_buf, buffer->attach);
	dma_buf_put(buffer->dma_buf);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_map_iommu(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	int ret = dma_map_sg_attrs(pb->dev, buffer->sg_table->sgl,
				buffer->sg_table->nents, buffer->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
	if (ret == 0) {
		dev_err(pb->dev,
				"%s: DMA map error, nents %d requested",
				__func__, buffer->sg_table->nents);
		return -EINVAL;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
static void ipu_buffer_unmap_iommu(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	dma_unmap_sg_attrs(pb->dev, buffer->sg_table->sgl,
			buffer->sg_table->nents, buffer->dir,
			DMA_ATTR_SKIP_CPU_SYNC);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_send_jqs_buffer_register(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	struct jqs_message_register_buffer req;

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_REGISTER_BUFFER);

	req.session_id = session->session_id;
	req.buffer_id = buffer->buffer_id;
	req.buffer_addr = sg_dma_address(buffer->sg_table->sgl);
	req.buffer_size = sg_dma_len(buffer->sg_table->sgl);

	dev_dbg(pb->dev, "%s: session_id %u buffer id %u addr %pad sz %u\n",
			__func__, session->session_id, req.buffer_id,
			&req.buffer_addr, req.buffer_size);

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_send_jqs_buffer_unregister(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	struct jqs_message_unregister_buffer req;

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_UNREGISTER_BUFFER);

	req.session_id = session->session_id;
	req.buffer_id = buffer->buffer_id;

	dev_dbg(pb->dev, "%s: session_id %u buffer id %u\n",
			__func__, session->session_id, req.buffer_id);

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

/* The caller to this function must hold pb->lock */
static int ipu_buffer_unmap_buffer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_buffer *buffer)
{
	/* Unmap the buffer from the IOMMU. */
	if (iommu_present(&ipu_bus_type))
		ipu_buffer_unmap_iommu(pb, session, buffer);

	ipu_buffer_unmap_dma_buf(pb, session, buffer);
	return 0;
}

int ipu_buffer_dma_buf_register_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct ipu_dma_buf_register_req __user *user_req;
	struct ipu_dma_buf_register_req req;
	struct paintbox_buffer *buffer;
	int ret;

	user_req = (struct ipu_dma_buf_register_req __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	if (req.dir >= DMA_NONE)
		return -EINVAL;

	mutex_lock(&pb->lock);

	/* Allocate a buffer id within the session. */
	ret = ipu_buffer_alloc(pb, session, &buffer);
	if (ret < 0)
		goto release_lock;

	/* Convert the DMA Buf fd to a scatter list. */
	ret = ipu_buffer_map_dma_buf(pb, session, req.dma_buf_fd,
			req.dir, buffer);
	if (ret < 0)
		goto release_buffer;

	/* If the IOMMU is enabled then map the buffer into the IOMMU IOVA
	 * space.
	 */
	if (iommu_present(&ipu_bus_type)) {
		ret = ipu_buffer_map_iommu(pb, session, buffer);
		if (ret < 0)
			goto unmap_buffer;
	} else {
		/* TODO(b/115418810):  Return an error if the scatter list is
		 * non-continuous.
		 */
	}

	/* Notify the JQS that the buffer has been mapped. */
	ret = ipu_buffer_send_jqs_buffer_register(pb, session, buffer);
	if (ret < 0)
		goto unmap_from_iommu;

	req.buffer_id = buffer->buffer_id;

	if (copy_to_user(user_req, &req, sizeof(req))) {
		ret = -EFAULT;
		goto unregister_from_jqs;
	}

	mutex_unlock(&pb->lock);

	return 0;

unregister_from_jqs:
	ipu_buffer_send_jqs_buffer_unregister(pb, session, buffer);
unmap_from_iommu:
	ipu_buffer_unmap_iommu(pb, session, buffer);
unmap_buffer:
	ipu_buffer_unmap_dma_buf(pb, session, buffer);
release_buffer:
	ipu_buffer_release_buffer(session, buffer);
release_lock:
	mutex_unlock(&pb->lock);

	return ret;
}

int ipu_buffer_dma_buf_unregister_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_buffer *buffer;
	int ret;

	mutex_lock(&pb->lock);

	buffer = ipu_buffer_get_buffer(session, (uint32_t)arg);
	if (!buffer) {
		ret = -EINVAL;
		goto err_exit;
	}

	ret = ipu_buffer_send_jqs_buffer_unregister(pb, session, buffer);
	if (ret == -EBUSY) {
		/* If the JQS returns that the buffer is busy then report back
		 * the busy error immediately.  The userspace will need to try
		 * again when the DMA transfer on that buffer has completed.
		 */
		goto err_exit;
	}

	/* TODO(b/114760293):  Other errors returned by
	 * ipu_buffer_send_jqs_buffer_unregister() would be catastrophic
	 * errors and we need to take the catastrophic error path once that is
	 * implemented.  In the event of a catastrophic error the driver should
	 * still unmap the buffer since the JQS will be reset.
	 */
	ipu_buffer_unmap_buffer(pb, session, buffer);

	ipu_buffer_release_buffer(session, buffer);

err_exit:
	mutex_unlock(&pb->lock);

	return ret;
}

int ipu_buffer_init_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	session->buffer_id_table = ab_dram_alloc_dma_buf_kernel(
			JQS_SESSION_MEMORY_SIZE);
	if (!session->buffer_id_table)
		return PTR_ERR(session->buffer_id_table);

	idr_init(&session->buffer_idr);

	return 0;
}

/* The caller to this function must hold pb->lock */
void ipu_buffer_release_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_buffer *buffer;
	int buffer_id;

	/* TODO(b/115419412):  We probably want to tell JQS to just invalidate
	 * the whole table rather than tell it to remove individual entries.
	 */

	idr_for_each_entry(&session->buffer_idr, buffer, buffer_id)
		(void)ipu_buffer_unmap_buffer(pb, session, buffer);

	ab_dram_free_dma_buf_kernel(session->buffer_id_table);

	idr_destroy(&session->buffer_idr);
}

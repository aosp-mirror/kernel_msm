/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2009-2010, Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * Original driver and v2 protocol changes from Code Aurora.
 * Heavily modified by Dima Zavin <dima@android.com>
 * Further cleanup by Brian Swetland <swetland@google.com>
 *
 */

#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/msm_q6venc.h>
#include <linux/wakelock.h>

//#include <asm/cacheflush.h>
#include "../dal.h"

#define DALDEVICEID_VENC_DEVICE		0x0200002D
#define DALDEVICEID_VENC_PORTNAME	"DSP_DAL_AQ_VID"

#define VENC_NAME			"q6venc"
#define VENC_MSG_MAX			128

#define VENC_INTERFACE_VERSION		0x00020000
#define MAJOR_MASK			0xFFFF0000
#define MINOR_MASK			0x0000FFFF
#define VENC_GET_MAJOR_VERSION(version)	((version & MAJOR_MASK)>>16)
#define VENC_GET_MINOR_VERSION(version)	(version & MINOR_MASK)

#define VERSION_CHECK 0

static DEFINE_MUTEX(idlecount_lock);
static int idlecount;
static struct wake_lock wakelock;
static struct wake_lock idlelock;

static void prevent_sleep(void)
{
        mutex_lock(&idlecount_lock);
        if (++idlecount == 1) {
                wake_lock(&idlelock);
                wake_lock(&wakelock);
	}
        mutex_unlock(&idlecount_lock);
}

static void allow_sleep(void)
{
        mutex_lock(&idlecount_lock);
        if (--idlecount == 0) {
                wake_unlock(&idlelock);
                wake_unlock(&wakelock);
	}
        mutex_unlock(&idlecount_lock);
}

enum {
	VENC_BUFFER_TYPE_INPUT,
	VENC_BUFFER_TYPE_OUTPUT,
	VENC_BUFFER_TYPE_QDSP6,
	VENC_BUFFER_TYPE_HDR
};
enum {
	VENC_DALRPC_GET_SYNTAX_HEADER = DAL_OP_FIRST_DEVICE_API,
	VENC_DALRPC_UPDATE_INTRA_REFRESH,
	VENC_DALRPC_UPDATE_FRAME_RATE,
	VENC_DALRPC_UPDATE_BITRATE,
	VENC_DALRPC_UPDATE_QP_RANGE,
	VENC_DALRPC_UPDATE_INTRA_PERIOD,
	VENC_DALRPC_REQUEST_IFRAME,
	VENC_DALRPC_START,
	VENC_DALRPC_STOP,
	VENC_DALRPC_SUSPEND,
	VENC_DALRPC_RESUME,
	VENC_DALRPC_FLUSH,
	VENC_DALRPC_QUEUE_INPUT,
	VENC_DALRPC_QUEUE_OUTPUT
};
struct venc_input_payload {
	u32 data;
};
struct venc_output_payload {
	u32 size;
	long long time_stamp;
	u32 flags;
	u32 data;
	u32 client_data_from_input;
};
union venc_payload {
	struct venc_input_payload input_payload;
	struct venc_output_payload output_payload;
};
struct venc_msg_type {
	u32 event;
	u32 status;
	union venc_payload payload;
};
struct venc_input_buf {
	struct venc_buf_type yuv_buf;
	u32 data_size;
	long long time_stamp;
	u32 flags;
	u32 dvs_offsetx;
	u32 dvs_offsety;
	u32 client_data;
	u32 op_client_data;
};
struct venc_output_buf {
	struct venc_buf_type bit_stream_buf;
	u32 client_data;
};
struct venc_buf {
	int fd;
	u32 offset;
	u32 size;
	u32 btype;
	unsigned long paddr;
	struct file *file;
};
struct venc_pmem_list {
	struct list_head list;
	struct venc_buf buf;
};
struct venc_qmsg {
	struct list_head list;
	struct venc_msg msg;
};
struct venc_dev {
	bool stop_called;
	enum venc_state_type state;

	struct list_head msg_pool;
	struct list_head msg_queue;
	spinlock_t msg_lock;

	struct list_head venc_pmem_list_head;
	spinlock_t venc_pmem_list_lock;
	struct dal_client *q6_handle;
	wait_queue_head_t venc_msg_evt;
	struct device *class_devp;
};

#define DEBUG_VENC 0
#if DEBUG_VENC
#define TRACE(fmt, x...)     \
	do { pr_debug("%s:%d " fmt, __func__, __LINE__, ##x); } while (0)
#else
#define TRACE(fmt, x...)         do { } while (0)
#endif

static struct cdev cdev;
static dev_t venc_dev_num;
static struct class *venc_class;
static struct venc_dev *venc_device_p;
static int venc_ref;

static inline int venc_check_version(u32 client, u32 server)
{
	int ret = -EINVAL;

	if ((VENC_GET_MAJOR_VERSION(client) == VENC_GET_MAJOR_VERSION(server))
	     && (VENC_GET_MINOR_VERSION(client) <=
		 VENC_GET_MINOR_VERSION(server)))
		ret = 0;

	return ret;
}

static struct venc_qmsg *__dequeue(spinlock_t *lock, struct list_head *queue)
{
	unsigned long flags;
	struct venc_qmsg *msg;
	spin_lock_irqsave(lock, flags);
	if (list_empty(queue)) {
		msg = NULL;
	} else {
		msg = list_first_entry(queue, struct venc_qmsg, list);
		list_del(&msg->list);
	}
	spin_unlock_irqrestore(lock, flags);
	return msg;
}

static inline struct venc_qmsg *venc_alloc_msg(struct venc_dev *dvenc)
{
	return __dequeue(&dvenc->msg_lock, &dvenc->msg_pool);
}
static inline struct venc_qmsg *venc_recv_msg(struct venc_dev *dvenc)
{
	return __dequeue(&dvenc->msg_lock, &dvenc->msg_queue);
}

static void venc_free_msg(struct venc_dev *dvenc, struct venc_qmsg *msg)
{
	unsigned long flags;
	spin_lock_irqsave(&dvenc->msg_lock, flags);
	list_add_tail(&msg->list, &dvenc->msg_pool);
	spin_unlock_irqrestore(&dvenc->msg_lock, flags);
}

static void venc_post(struct venc_dev *dvenc,
		      unsigned code, unsigned status,
		      union venc_msg_data *data)
{
	unsigned long flags;
	struct venc_qmsg *msg;
	msg = venc_alloc_msg(dvenc);
	if (msg == NULL) {
		pr_err("%s cannot alloc message\n", __func__);
		return;
	}
	msg->msg.msg_code = code;
	msg->msg.status_code = status;
	if (data) {
		msg->msg.msg_data_size = sizeof(union venc_msg_data);
		memcpy(&msg->msg.msg_data, data, sizeof(union venc_msg_data));
	} else {
		msg->msg.msg_data_size = 0;
	}

	spin_lock_irqsave(&dvenc->msg_lock, flags);
	list_add_tail(&msg->list, &dvenc->msg_queue);
	spin_unlock_irqrestore(&dvenc->msg_lock, flags);
	wake_up(&dvenc->venc_msg_evt);
}

static struct venc_pmem_list *venc_add_pmem_to_list(struct venc_dev *dvenc,
						      struct venc_pmem *mptr,
						      u32 btype)
{
	int ret = 0;
	unsigned long flags;
	unsigned long len;
	unsigned long vaddr;
	struct venc_pmem_list *plist = NULL;

	plist = kzalloc(sizeof(struct venc_pmem_list), GFP_KERNEL);
	if (!plist) {
		pr_err("%s: kzalloc failed\n", __func__);
		return NULL;
	}

	ret = get_pmem_file(mptr->fd, &(plist->buf.paddr),
		&vaddr, &len, &(plist->buf.file));

	/* xxx bounds checking insufficient here */
	if (ret) {
		pr_err("%s: get_pmem_file failed for fd=%d offset=%d\n",
			__func__, mptr->fd, mptr->offset);
		goto err_venc_add_pmem;
	} else if (mptr->offset >= len) {
		pr_err("%s: invalid offset (%d > %ld) for fd=%d\n",
		       __func__, mptr->offset, len, mptr->fd);
		ret = -EINVAL;
		goto err_venc_get_pmem;
	}

	plist->buf.fd = mptr->fd;
	plist->buf.paddr += mptr->offset;
	plist->buf.size = mptr->size;
	plist->buf.btype = btype;
	plist->buf.offset = mptr->offset;

	spin_lock_irqsave(&dvenc->venc_pmem_list_lock, flags);
	list_add(&plist->list, &dvenc->venc_pmem_list_head);
	spin_unlock_irqrestore(&dvenc->venc_pmem_list_lock, flags);
	return plist;

err_venc_get_pmem:
	put_pmem_file(plist->buf.file);
err_venc_add_pmem:
	kfree(plist);
	return NULL;
}

static struct venc_pmem_list *venc_get_pmem_from_list(
		struct venc_dev *dvenc, u32 pmem_fd,
		u32 offset, u32 btype)
{
	struct venc_pmem_list *plist;
	unsigned long flags;
	struct file *file;
	int found = 0;

	file = fget(pmem_fd);
	if (!file) {
		pr_err("%s: invalid encoder buffer fd(%d)\n", __func__,
			pmem_fd);
		return NULL;
	}
	spin_lock_irqsave(&dvenc->venc_pmem_list_lock, flags);
	list_for_each_entry(plist, &dvenc->venc_pmem_list_head, list) {
		if (plist->buf.btype == btype && plist->buf.file == file &&
			plist->buf.offset == offset) {
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&dvenc->venc_pmem_list_lock, flags);
	fput(file);
	if (found)
		return plist;

	else
		return NULL;
}

static int venc_set_buffer(struct venc_dev *dvenc, void *argp,
			     u32 btype)
{
	struct venc_pmem pmem;
	struct venc_pmem_list *plist;
	int ret = 0;

	ret = copy_from_user(&pmem, argp, sizeof(pmem));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	plist = venc_add_pmem_to_list(dvenc, &pmem, btype);
	if (plist == NULL) {
		pr_err("%s: buffer add_to_pmem_list failed\n",
			__func__);
		return -EPERM;
	}
	return ret;
}

static int venc_assign_q6_buffers(struct venc_dev *dvenc,
				    struct venc_buffers *pbufs,
				    struct venc_nonio_buf_config *pcfg)
{
	int ret = 0;
	struct venc_pmem_list *plist;

	plist = venc_add_pmem_to_list(dvenc, &(pbufs->recon_buf[0]),
				  VENC_BUFFER_TYPE_QDSP6);
	if (plist == NULL) {
		pr_err("%s: recon_buf0 failed to add_to_pmem_list\n",
			__func__);
		return -EPERM;
	}
	pcfg->recon_buf1.region = 0;
	pcfg->recon_buf1.phys = plist->buf.paddr;
	pcfg->recon_buf1.size = plist->buf.size;
	pcfg->recon_buf1.offset = 0;

	plist = venc_add_pmem_to_list(dvenc, &(pbufs->recon_buf[1]),
				  VENC_BUFFER_TYPE_QDSP6);
	if (plist == NULL) {
		pr_err("%s: recons_buf1 failed to add_to_pmem_list\n",
			__func__);
		return -EPERM;
	}
	pcfg->recon_buf2.region = 0;
	pcfg->recon_buf2.phys = plist->buf.paddr;
	pcfg->recon_buf2.size = plist->buf.size;
	pcfg->recon_buf2.offset = 0;

	plist = venc_add_pmem_to_list(dvenc, &(pbufs->wb_buf),
				  VENC_BUFFER_TYPE_QDSP6);
	if (plist == NULL) {
		pr_err("%s: wb_buf failed to add_to_pmem_list\n",
			__func__);
		return -EPERM;
	}
	pcfg->wb_buf.region = 0;
	pcfg->wb_buf.phys = plist->buf.paddr;
	pcfg->wb_buf.size = plist->buf.size;
	pcfg->wb_buf.offset = 0;

	plist = venc_add_pmem_to_list(dvenc, &(pbufs->cmd_buf),
				  VENC_BUFFER_TYPE_QDSP6);
	if (plist == NULL) {
		pr_err("%s: cmd_buf failed to add_to_pmem_list\n",
			__func__);
		return -EPERM;
	}
	pcfg->cmd_buf.region = 0;
	pcfg->cmd_buf.phys = plist->buf.paddr;
	pcfg->cmd_buf.size = plist->buf.size;
	pcfg->cmd_buf.offset = 0;

	plist = venc_add_pmem_to_list(dvenc, &(pbufs->vlc_buf),
				  VENC_BUFFER_TYPE_QDSP6);
	if (plist == NULL) {
		pr_err("%s: vlc_buf failed to add_to_pmem_list"
		" failed\n", __func__);
		return -EPERM;
	}
	pcfg->vlc_buf.region = 0;
	pcfg->vlc_buf.phys = plist->buf.paddr;
	pcfg->vlc_buf.size = plist->buf.size;
	pcfg->vlc_buf.offset = 0;

	return ret;
}

static int venc_start(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	struct venc_q6_config q6_config;
	struct venc_init_config vconfig;

	dvenc->state = VENC_STATE_START;
	ret = copy_from_user(&vconfig, argp, sizeof(struct venc_init_config));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	memcpy(&q6_config, &(vconfig.q6_config), sizeof(q6_config));
	ret = venc_assign_q6_buffers(dvenc, &(vconfig.q6_bufs),
		&(q6_config.buf_params));
	if (ret != 0) {
		pr_err("%s: assign_q6_buffers failed\n", __func__);
		return -EPERM;
	}

	q6_config.callback_event = dvenc->q6_handle;
	TRACE("%s: parameters: handle:%p, config:%p, callback:%p \n", __func__,
		dvenc->q6_handle, &q6_config, q6_config.callback_event);
	TRACE("%s: parameters:recon1:0x%x, recon2:0x%x,"
		" wb_buf:0x%x, cmd:0x%x, vlc:0x%x\n", __func__,
		q6_config.buf_params.recon_buf1.phys,
		q6_config.buf_params.recon_buf2.phys,
		q6_config.buf_params.wb_buf.phys,
		q6_config.buf_params.cmd_buf.phys,
		q6_config.buf_params.vlc_buf.phys);
	TRACE("%s: size of param:%d \n", __func__, sizeof(q6_config));
	ret = dal_call_f5(dvenc->q6_handle, VENC_DALRPC_START, &q6_config,
		sizeof(q6_config));
	if (ret != 0) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
		return ret;
	}
	return ret;
}

static int venc_encode_frame(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	struct venc_pmem buf;
	struct venc_input_buf q6_input;
	struct venc_pmem_list *plist;
	struct venc_buffer input;

	ret = copy_from_user(&input, argp, sizeof(struct venc_buffer));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	ret = copy_from_user(&buf,
			       ((struct venc_buffer *)argp)->ptr_buffer,
			       sizeof(struct venc_pmem));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	plist = venc_get_pmem_from_list(dvenc, buf.fd, buf.offset,
			VENC_BUFFER_TYPE_INPUT);
	if (NULL == plist) {
		plist = venc_add_pmem_to_list(dvenc, &buf,
			VENC_BUFFER_TYPE_INPUT);
		if (plist == NULL) {
			pr_err("%s: buffer add_to_pmem_list failed\n",
				__func__);
			return -EPERM;
		}
	}

	q6_input.flags = 0;
	if (input.flags & VENC_FLAG_EOS)
		q6_input.flags |= 0x00000001;
	q6_input.yuv_buf.region = 0;
	q6_input.yuv_buf.phys = plist->buf.paddr;
	q6_input.yuv_buf.size = plist->buf.size;
	q6_input.yuv_buf.offset = 0;
	q6_input.data_size = plist->buf.size;
	q6_input.client_data = (u32)input.client_data;
	q6_input.time_stamp = input.time_stamp;
	q6_input.dvs_offsetx = 0;
	q6_input.dvs_offsety = 0;

	TRACE("Pushing down input phys=0x%x fd= %d, client_data: 0x%x,"
		" time_stamp:%lld \n", q6_input.yuv_buf.phys, plist->buf.fd,
		input.client_data, input.time_stamp);
	ret = dal_call_f5(dvenc->q6_handle, VENC_DALRPC_QUEUE_INPUT,
		&q6_input, sizeof(q6_input));

	if (ret != 0)
		pr_err("%s: Q6 queue_input failed (%d)\n", __func__,
		(int)ret);
	return ret;
}

static int venc_fill_output(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	struct venc_pmem buf;
	struct venc_output_buf q6_output;
	struct venc_pmem_list *plist;
	struct venc_buffer output;

	ret = copy_from_user(&output, argp, sizeof(struct venc_buffer));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	ret = copy_from_user(&buf,
			       ((struct venc_buffer *)argp)->ptr_buffer,
			       sizeof(struct venc_pmem));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	plist =	venc_get_pmem_from_list(dvenc, buf.fd, buf.offset,
			VENC_BUFFER_TYPE_OUTPUT);
	if (NULL == plist) {
		plist = venc_add_pmem_to_list(dvenc, &buf,
				VENC_BUFFER_TYPE_OUTPUT);
		if (NULL == plist) {
			pr_err("%s: output buffer failed to add_to_pmem_list"
				"\n", __func__);
			return -EPERM;
		}
	}
	q6_output.bit_stream_buf.region = 0;
	q6_output.bit_stream_buf.phys = (u32)plist->buf.paddr;
	q6_output.bit_stream_buf.size = plist->buf.size;
	q6_output.bit_stream_buf.offset = 0;
	q6_output.client_data = (u32)output.client_data;
	ret =
	    dal_call_f5(dvenc->q6_handle, VENC_DALRPC_QUEUE_OUTPUT, &q6_output,
			sizeof(q6_output));
	if (ret != 0)
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
	return ret;
}

static int venc_stop(struct venc_dev *dvenc)
{
	int ret = 0;

	dvenc->stop_called = 1;
	ret = dal_call_f0(dvenc->q6_handle, VENC_DALRPC_STOP, 1);
	if (ret) {
		pr_err("%s: remote runction failed (%d)\n", __func__, ret);
		venc_post(dvenc, VENC_MSG_STOP, VENC_S_EFAIL, NULL);
	}
	return ret;
}

static int venc_pause(struct venc_dev *dvenc)
{
	int ret = 0;

	ret = dal_call_f0(dvenc->q6_handle, VENC_DALRPC_SUSPEND, 1);
	if (ret) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
		venc_post(dvenc, VENC_MSG_PAUSE, VENC_S_EFAIL, NULL);
	}
	return ret;
}

static int venc_resume(struct venc_dev *dvenc)
{
	int ret = 0;

	ret = dal_call_f0(dvenc->q6_handle, VENC_DALRPC_RESUME, 1);
	if (ret) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
		venc_post(dvenc, VENC_MSG_RESUME, VENC_S_EFAIL, NULL);
	}
	return ret;
}

static int venc_flush(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	int status = VENC_S_SUCCESS;
	union venc_msg_data data;

	if (copy_from_user(&data.flush_ret, argp, sizeof(struct venc_buffer_flush)))
		return -EFAULT;

	if (data.flush_ret.flush_mode == VENC_FLUSH_ALL) {
		ret = dal_call_f0(dvenc->q6_handle, VENC_DALRPC_FLUSH, 1);
		if (ret)
			status = VENC_S_EFAIL;
	} else
		status = VENC_S_ENOTSUPP;

	if (status == VENC_S_SUCCESS)
		return ret;

	venc_post(dvenc, VENC_MSG_FLUSH, status, &data);
	return -EIO;
}

static int venc_get_sequence_hdr(struct venc_dev *dvenc, void *argp)
{
	pr_err("%s not supported\n", __func__);
	return -EIO;
}

static int venc_set_qp_range(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	struct venc_qp_range qp;

	ret = copy_from_user(&qp, argp, sizeof(struct venc_qp_range));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	if (dvenc->state == VENC_STATE_START ||
		dvenc->state == VENC_STATE_PAUSE) {
		ret =
		    dal_call_f5(dvenc->q6_handle, VENC_DALRPC_UPDATE_QP_RANGE,
				&qp, sizeof(struct venc_qp_range));
		if (ret) {
			pr_err("%s: remote function failed (%d) \n", __func__,
				ret);
			return ret;
		}
	}
	return ret;
}

static int venc_set_intra_period(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	u32 pnum = 0;

	ret = copy_from_user(&pnum, argp, sizeof(int));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	if (dvenc->state == VENC_STATE_START ||
		dvenc->state == VENC_STATE_PAUSE) {
		ret = dal_call_f0(dvenc->q6_handle,
			VENC_DALRPC_UPDATE_INTRA_PERIOD, pnum);
		if (ret)
			pr_err("%s: remote function failed (%d)\n", __func__,
				ret);
	}
	return ret;
}

static int venc_set_intra_refresh(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	u32 mb_num = 0;

	ret = copy_from_user(&mb_num, argp, sizeof(int));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	if (dvenc->state == VENC_STATE_START ||
		dvenc->state == VENC_STATE_PAUSE) {
		ret = dal_call_f0(dvenc->q6_handle,
			VENC_DALRPC_UPDATE_INTRA_REFRESH, mb_num);
		if (ret)
			pr_err("%s: remote function failed (%d)\n", __func__,
				ret);
	}
	return ret;
}

static int venc_set_frame_rate(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	struct venc_frame_rate pdata;
	ret = copy_from_user(&pdata, argp, sizeof(struct venc_frame_rate));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	if (dvenc->state == VENC_STATE_START ||
		dvenc->state == VENC_STATE_PAUSE) {
		ret = dal_call_f5(dvenc->q6_handle,
				VENC_DALRPC_UPDATE_FRAME_RATE,
				(void *)&(pdata),
				sizeof(struct venc_frame_rate));
		if (ret)
			pr_err("%s: remote function failed (%d)\n", __func__,
				ret);
	}
	return ret;
}

static int venc_set_target_bitrate(struct venc_dev *dvenc, void *argp)
{
	int ret = 0;
	u32 pdata = 0;

	ret = copy_from_user(&pdata, argp, sizeof(int));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}
	if (dvenc->state == VENC_STATE_START ||
		dvenc->state == VENC_STATE_PAUSE) {
		ret = dal_call_f0(dvenc->q6_handle,
			VENC_DALRPC_UPDATE_BITRATE, pdata);
		if (ret)
			pr_err("%s: remote function failed (%d)\n", __func__,
				ret);
	}
	return ret;
}

static int venc_request_iframe(struct venc_dev *dvenc)
{
	int ret = 0;

	if (dvenc->state != VENC_STATE_START)
		return -EINVAL;

	ret = dal_call_f0(dvenc->q6_handle, VENC_DALRPC_REQUEST_IFRAME, 1);
	if (ret)
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
	return ret;
}

static int venc_stop_read_msg(struct venc_dev *dvenc)
{
	venc_post(dvenc, VENC_MSG_STOP_READING_MSG, 0, NULL);
	return 0;
}

static int venc_translate_error(enum venc_status_code q6_status)
{
	switch (q6_status) {
	case VENC_STATUS_SUCCESS:
		return VENC_S_SUCCESS;
	case VENC_STATUS_ERROR:
		return VENC_S_EFAIL;
	case VENC_STATUS_INVALID_STATE:
		return VENC_S_EINVALSTATE;
	case VENC_STATUS_FLUSHING:
		return VENC_S_EFLUSHED;
	case VENC_STATUS_INVALID_PARAM:
		return VENC_S_EBADPARAM;
	case VENC_STATUS_CMD_QUEUE_FULL:
		return VENC_S_ECMDQFULL;
	case VENC_STATUS_CRITICAL:
		return VENC_S_EFATAL;
	case VENC_STATUS_INSUFFICIENT_RESOURCES:
		return VENC_S_ENOHWRES;
	case VENC_STATUS_TIMEOUT:
		return VENC_S_ETIMEOUT;
	default:
		/* xxx probably shouldn't assume success */
		return 0;
	}
}

static void venc_q6_callback(void *_data, int len, void *cookie)
{
	int status = 0;
	struct venc_dev *dvenc = (struct venc_dev *)cookie;
	struct venc_msg_type *q6_msg = NULL;
	struct venc_input_payload *pload1;
	struct venc_output_payload *pload2;
	union venc_msg_data data;
	uint32_t *tmp = (uint32_t *) _data;

	if (dvenc == NULL) {
		pr_err("%s: empty driver parameter\n", __func__);
		return;
	}
	if (tmp[2] == sizeof(struct venc_msg_type)) {
		q6_msg = (struct venc_msg_type *)&tmp[3];
	} else {
		pr_err("%s: callback with empty message (%d, %d)\n",
			__func__, tmp[2], sizeof(struct venc_msg_type));
		return;
	}

	status = venc_translate_error(q6_msg->status);
	if (status != VENC_STATUS_SUCCESS)
		pr_err("%s: Q6 failed (%d)", __func__, (int)status);

	switch ((enum venc_event_type_enum)q6_msg->event) {
	case VENC_EVENT_START_STATUS:
		dvenc->state = VENC_STATE_START;
		venc_post(dvenc, VENC_MSG_START, status, NULL);
		break;
	case VENC_EVENT_STOP_STATUS:
		dvenc->state = VENC_STATE_STOP;
		venc_post(dvenc, VENC_MSG_STOP, status, NULL);
		break;
	case VENC_EVENT_SUSPEND_STATUS:
		dvenc->state = VENC_STATE_PAUSE;
		venc_post(dvenc, VENC_MSG_PAUSE, status, NULL);
		break;
	case VENC_EVENT_RESUME_STATUS:
		dvenc->state = VENC_STATE_START;
		venc_post(dvenc, VENC_MSG_RESUME, status, NULL);
		break;
	case VENC_EVENT_FLUSH_STATUS:
		data.flush_ret.flush_mode = VENC_FLUSH_INPUT;
		venc_post(dvenc, VENC_MSG_FLUSH, status, &data);
		data.flush_ret.flush_mode = VENC_FLUSH_OUTPUT;
		venc_post(dvenc, VENC_MSG_FLUSH, status, &data);		
		break;
	case VENC_EVENT_RELEASE_INPUT:
		pload1 = &((q6_msg->payload).input_payload);
		TRACE("Release_input: data: 0x%x \n", pload1->data);
		if (pload1 != NULL) {
			/* xxx should we zero? */
			data.buf.client_data = pload1->data;
			venc_post(dvenc, VENC_MSG_INPUT_BUFFER_DONE, status, &data);
		} else {
			pr_err("%s no payload on buffer done?\n", __func__);
		}
		break;
	case VENC_EVENT_DELIVER_OUTPUT:
		pload2 = &((q6_msg->payload).output_payload);
		data.buf.flags = 0;
		if (pload2->flags & VENC_FLAG_SYNC_FRAME)
			data.buf.flags |= VENC_FLAG_SYNC_FRAME;
		if (pload2->flags & VENC_FLAG_CODEC_CONFIG)
			data.buf.flags |= VENC_FLAG_CODEC_CONFIG;
		if (pload2->flags & VENC_FLAG_END_OF_FRAME)
			data.buf.flags |= VENC_FLAG_END_OF_FRAME;
		if (pload2->flags & VENC_FLAG_EOS)
			data.buf.flags |= VENC_FLAG_EOS;
		data.buf.len = pload2->size;
		data.buf.offset = 0;
		data.buf.time_stamp = pload2->time_stamp;
		data.buf.client_data = pload2->data;
		venc_post(dvenc, VENC_MSG_OUTPUT_BUFFER_DONE, status, &data);
		break;
	default:
		pr_err("%s: invalid response from Q6 (%d)\n", __func__,
			(int)q6_msg->event);
		break;
	}
}

static int venc_read_next_msg(struct venc_dev *dvenc, void __user *argp)
{
	int res;
	struct venc_qmsg *msg;

	res = wait_event_interruptible(dvenc->venc_msg_evt,
				       (msg = venc_recv_msg(dvenc)) != NULL);
	if (res < 0)
		return res;
	res = copy_to_user(argp, &msg->msg, sizeof(struct venc_msg));
	venc_free_msg(dvenc, msg);
	if (res)
		return -EFAULT;
	return 0;
}

static long q6venc_ioctl(struct file *file, u32 cmd,
			   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct venc_dev *dvenc = file->private_data;

	switch (cmd) {
	case VENC_IOCTL_SET_INPUT_BUFFER:
		return venc_set_buffer(dvenc, argp, VENC_BUFFER_TYPE_INPUT);
	case VENC_IOCTL_SET_OUTPUT_BUFFER:
		return venc_set_buffer(dvenc, argp, VENC_BUFFER_TYPE_OUTPUT);
	case VENC_IOCTL_GET_SEQUENCE_HDR:
		return venc_get_sequence_hdr(dvenc, argp);
	case VENC_IOCTL_SET_QP_RANGE:
		return venc_set_qp_range(dvenc, argp);
	case VENC_IOCTL_SET_INTRA_PERIOD:
		return venc_set_intra_period(dvenc, argp);
	case VENC_IOCTL_SET_INTRA_REFRESH:
		return venc_set_intra_refresh(dvenc, argp);
	case VENC_IOCTL_SET_FRAME_RATE:
		return venc_set_frame_rate(dvenc, argp);
	case VENC_IOCTL_SET_TARGET_BITRATE:
		return venc_set_target_bitrate(dvenc, argp);
	case VENC_IOCTL_CMD_REQUEST_IFRAME:
		if (dvenc->state == VENC_STATE_START)
			return venc_request_iframe(dvenc);
		else
			return 0;
	case VENC_IOCTL_CMD_START:
		return venc_start(dvenc, argp);
	case VENC_IOCTL_CMD_STOP:
		return venc_stop(dvenc);
	case VENC_IOCTL_CMD_PAUSE:
		return venc_pause(dvenc);
	case VENC_IOCTL_CMD_RESUME:
		return venc_resume(dvenc);
	case VENC_IOCTL_CMD_ENCODE_FRAME:
		return venc_encode_frame(dvenc, argp);
	case VENC_IOCTL_CMD_FILL_OUTPUT_BUFFER:
		return venc_fill_output(dvenc, argp);
	case VENC_IOCTL_CMD_FLUSH:
		return venc_flush(dvenc, argp);
	case VENC_IOCTL_CMD_READ_NEXT_MSG:
		return venc_read_next_msg(dvenc, argp);
	case VENC_IOCTL_CMD_STOP_READ_MSG:
		return venc_stop_read_msg(dvenc);
	default:
		pr_err("%s: invalid ioctl code (%d)\n", __func__, cmd);
		return -EINVAL;
	}
}

static int q6venc_open(struct inode *inode, struct file *file)
{
	int i;
	int ret = 0;
	struct venc_dev *dvenc;
	struct venc_qmsg *msg;
#if VERSION_CHECK
	struct dal_info version_info;
#endif

	dvenc = kzalloc(sizeof(struct venc_dev), GFP_KERNEL);
	if (!dvenc)
		return -ENOMEM;

	file->private_data = dvenc;
	INIT_LIST_HEAD(&dvenc->msg_pool);
	INIT_LIST_HEAD(&dvenc->msg_queue);
	INIT_LIST_HEAD(&dvenc->venc_pmem_list_head);
	init_waitqueue_head(&dvenc->venc_msg_evt);
	spin_lock_init(&dvenc->msg_lock);
	spin_lock_init(&dvenc->venc_pmem_list_lock);
	venc_ref++;

	for (i = 0; i < VENC_MSG_MAX; i++) {
		msg = kzalloc(sizeof(struct venc_qmsg), GFP_KERNEL);
		if (msg == NULL) {
			ret = -ENOMEM;
			goto fail_list_alloc;
		}
		venc_free_msg(dvenc, msg);
	}

	dvenc->q6_handle = dal_attach(DALDEVICEID_VENC_DEVICE,
				      DALDEVICEID_VENC_PORTNAME,
				      venc_q6_callback, (void *)dvenc);

	if (!(dvenc->q6_handle)) {
		pr_err("%s: daldevice_attach failed (%d)\n", __func__, ret);
		goto fail_list_alloc;
	}

#if VERSION_CHECK
	ret = dal_call_f9(dvenc->q6_handle, DAL_OP_INFO, &version_info,
		sizeof(struct dal_info));
	if (ret) {
		pr_err("%s: failed to get version\n", __func__);
		ret = -EINVAL;
		goto fail_open;
	}
	if (venc_check_version(VENC_INTERFACE_VERSION, version_info.version)) {
		pr_err("%s: driver version mismatch\n", __func__);
		ret = -EINVAL;
		goto fail_open;
	}
#endif
	ret = dal_call_f0(dvenc->q6_handle, DAL_OP_OPEN, 1);
	if (ret) {
		pr_err("%s: dal_call_open failed (%d)\n", __func__, ret);
		goto fail_open;
	}
	dvenc->state = VENC_STATE_STOP;
	prevent_sleep();
	return ret;

fail_open:
	dal_detach(dvenc->q6_handle);

fail_list_alloc:
	while ((msg = venc_alloc_msg(dvenc)))
		kfree(msg);

	kfree(dvenc);
	venc_ref--;
	return ret;
}

static int q6venc_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct venc_pmem_list *plist, *m;
	struct venc_dev *dvenc;
	struct venc_qmsg *msg;

	venc_ref--;

	dvenc = file->private_data;
	wake_up_all(&dvenc->venc_msg_evt);
	if (!dvenc->stop_called)
		dal_call_f0(dvenc->q6_handle, VENC_DALRPC_STOP, 1);
	dal_call_f0(dvenc->q6_handle, DAL_OP_CLOSE, 1);
	dal_detach(dvenc->q6_handle);


	/* free all messages in the pool */
	while ((msg = venc_alloc_msg(dvenc)))
		kfree(msg);

	/* free all messages sitting in the queue */
	while ((msg = venc_recv_msg(dvenc)))
		kfree(msg);

	list_for_each_entry_safe(plist, m, &dvenc->venc_pmem_list_head, list) {
		put_pmem_file(plist->buf.file);
		list_del(&plist->list);
		kfree(plist);
	}
	kfree(dvenc);
	allow_sleep();
	return ret;
}

const struct file_operations q6venc_fops = {
	.owner = THIS_MODULE,
	.open = q6venc_open,
	.release = q6venc_release,
	.unlocked_ioctl = q6venc_ioctl,
};

static int __init q6venc_init(void)
{
	int ret = 0;

	venc_device_p = kzalloc(sizeof(struct venc_dev), GFP_KERNEL);
	if (!venc_device_p) {
		pr_err("%s: unable to allocate memory for venc_device_p\n",
			__func__);
		return -ENOMEM;
	}
	wake_lock_init(&idlelock, WAKE_LOCK_IDLE, "venc_idle");
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "venc_suspend");

	ret = alloc_chrdev_region(&venc_dev_num, 0, 1, VENC_NAME);
	if (ret < 0) {
		pr_err("%s: alloc_chrdev_region failed (%d)\n", __func__,
			ret);
		return ret;
	}
	venc_class = class_create(THIS_MODULE, VENC_NAME);
	if (IS_ERR(venc_class)) {
		ret = PTR_ERR(venc_class);
		pr_err("%s: failed to create venc_class (%d)\n",
			__func__, ret);
		goto err_venc_class_create;
	}
	venc_device_p->class_devp =
	    device_create(venc_class, NULL, venc_dev_num, NULL,
			  VENC_NAME);
	if (IS_ERR(venc_device_p->class_devp)) {
		ret = PTR_ERR(venc_device_p->class_devp);
		pr_err("%s: failed to create class_device (%d)\n", __func__,
			ret);
		goto err_venc_class_device_create;
	}
	cdev_init(&cdev, &q6venc_fops);
	cdev.owner = THIS_MODULE;
	ret = cdev_add(&cdev, venc_dev_num, 1);
	if (ret < 0) {
		pr_err("%s: cdev_add failed (%d)\n", __func__, ret);
		goto err_venc_cdev_add;
	}
	init_waitqueue_head(&venc_device_p->venc_msg_evt);
	return ret;

err_venc_cdev_add:
	device_destroy(venc_class, venc_dev_num);
err_venc_class_device_create:
	class_destroy(venc_class);
err_venc_class_create:
	unregister_chrdev_region(venc_dev_num, 1);
	return ret;
}

static void __exit q6venc_exit(void)
{
	cdev_del(&(cdev));
	device_destroy(venc_class, venc_dev_num);
	class_destroy(venc_class);
	unregister_chrdev_region(venc_dev_num, 1);
}

MODULE_DESCRIPTION("Video encoder driver for QDSP6");
MODULE_VERSION("2.0");
module_init(q6venc_init);
module_exit(q6venc_exit);

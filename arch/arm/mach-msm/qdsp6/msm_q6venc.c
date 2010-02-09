/*
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2009, Google Inc.
 *
 * Original authors: Code Aurora Forum
 * Major cleanup: Dima Zavin <dima@android.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

//#define DEBUG 1

#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/msm_q6venc.h>

#include <asm/cacheflush.h>

#include "dal.h"

#define DALDEVICEID_VENC_DEVICE		0x0200002D
#define DALDEVICEID_VENC_PORTNAME	"DSP_DAL_AQ_VID"

enum {
	VENC_DALRPC_INITIALIZE = DAL_OP_FIRST_DEVICE_API,
	VENC_DALRPC_SET_CB_CHANNEL,
	VENC_DALRPC_ENCODE,
	VENC_DALRPC_INTRA_REFRESH,
	VENC_DALRPC_RC_CONFIG,
	VENC_DALRPC_ENCODE_CONFIG,
	VENC_DALRPC_STOP,
};

struct callback_event_data {
	u32				data_notify_event;
	u32				enc_cb_handle;
	u32				empty_input_buffer_event;
};

struct buf_info {
	unsigned long			paddr;
	unsigned long			vaddr;
	struct file			*file;
	struct venc_buf			venc_buf;
};

#define VENC_MAX_BUF_NUM		15
#define RLC_MAX_BUF_NUM			2
#define BITS_PER_PIXEL			12
#define PIXELS_PER_MACROBLOCK		16

#define VENC_CB_EVENT_ID		0xd0e4c0de

struct q6venc_dev {
	struct dal_client		*venc;
	struct callback_event_data	cb_ev_data;
	bool				stop_encode;
	struct buf_info			rlc_bufs[RLC_MAX_BUF_NUM];
	unsigned int			rlc_buf_index;
	unsigned int			rlc_buf_len;
	unsigned int                    enc_buf_size;
	struct buf_info			enc_bufs[VENC_MAX_BUF_NUM];
	unsigned int			num_enc_bufs;
	wait_queue_head_t		encode_wq;

	/* protects all state in q6venc_dev except for cb stuff below */
	struct mutex			lock;

	/* protects encode_done and done_frame inside the callback */
	spinlock_t			done_lock;
	struct frame_type		done_frame;
	bool				encode_done;
};

static int get_buf_info(struct buf_info *buf_info, struct venc_buf *venc_buf)
{
	unsigned long len;
	unsigned long vaddr;
	unsigned long paddr;
	struct file *file;
	int ret;

	ret = get_pmem_file(venc_buf->fd, &paddr, &vaddr, &len, &file);
	if (ret) {
		pr_err("%s: get_pmem_file failed for fd=%d offset=%ld\n",
		       __func__, venc_buf->fd, venc_buf->offset);
		return ret;
	} else if (venc_buf->offset >= len) {
		/* XXX: we really should check venc_buf->size too, but userspace
		 * sometimes leaves this uninitialized (in encode ioctl) */
		pr_err("%s: invalid offset/size (%ld + %ld > %ld) for fd=%d\n",
		       __func__, venc_buf->offset, venc_buf->size, len,
		       venc_buf->fd);
		put_pmem_file(file);
		return -EINVAL;
	}

	buf_info->file = file;
	buf_info->paddr = paddr + venc_buf->offset;
	buf_info->vaddr = vaddr;
	memcpy(&buf_info->venc_buf, venc_buf, sizeof(struct venc_buf));
	return 0;
}

static void put_buf_info(struct buf_info *buf_info)
{
	if (!buf_info || !buf_info->file)
		return;
	put_pmem_file(buf_info->file);
	buf_info->file = NULL;
}

static void q6venc_callback(void *context, void *data, uint32_t len)
{
	struct q6venc_dev *q6venc = context;
	struct q6_frame_type *q6frame = data;
	struct buf_info *rlc_buf;
	unsigned long flags;
	int i;

	pr_debug("%s \n", __func__);

	spin_lock_irqsave(&q6venc->done_lock, flags);
	q6venc->encode_done = true;
	for (i = 0; i < RLC_MAX_BUF_NUM; ++i) {
		rlc_buf = &q6venc->rlc_bufs[i];
		if (rlc_buf->paddr == q6frame->frame_addr)
			goto frame_found;
	}

	pr_err("%s: got incorrect phy address 0x%08x from q6 \n", __func__,
	       q6frame->frame_addr);
	q6venc->done_frame.q6_frame_type.frame_len = 0;
	wake_up_interruptible(&q6venc->encode_wq);
	goto done;

frame_found:
	memcpy(&q6venc->done_frame.frame_addr, &rlc_buf->venc_buf,
	       sizeof(struct venc_buf));
	memcpy(&q6venc->done_frame.q6_frame_type, q6frame,
	       sizeof(struct q6_frame_type));

	dmac_inv_range((const void *)q6venc->rlc_bufs[i].vaddr,
		       (const void *)(q6venc->rlc_bufs[i].vaddr +
				      q6venc->rlc_buf_len));

	wake_up_interruptible(&q6venc->encode_wq);

done:
	spin_unlock_irqrestore(&q6venc->done_lock, flags);
}

static void callback(void *data, int len, void *cookie)
{
	struct q6venc_dev *ve = (struct q6venc_dev *)cookie;
	uint32_t *tmp = (uint32_t *) data;

	if (tmp[0] == VENC_CB_EVENT_ID)
		q6venc_callback(ve, &tmp[3], tmp[2]);
	else
		pr_err("%s: Unknown callback received for %p\n", __func__, ve);
}

static int q6venc_open(struct inode *inode, struct file *file)
{
	struct q6venc_dev *q6venc;
	int err;

	q6venc = kzalloc(sizeof(struct q6venc_dev), GFP_KERNEL);
	if (!q6venc) {
		pr_err("%s: Unable to allocate memory for q6venc_dev\n",
		       __func__);
		return -ENOMEM;
	}

	file->private_data = q6venc;

	init_waitqueue_head(&q6venc->encode_wq);
	mutex_init(&q6venc->lock);
	spin_lock_init(&q6venc->done_lock);

	q6venc->venc = dal_attach(DALDEVICEID_VENC_DEVICE,
				  DALDEVICEID_VENC_PORTNAME,
				  callback, q6venc);
	if (!q6venc->venc) {
		pr_err("%s: dal_attach failed\n", __func__);
		err = -EIO;
		goto err_dal_attach;
	}

	q6venc->cb_ev_data.enc_cb_handle = VENC_CB_EVENT_ID;
	err = dal_call_f5(q6venc->venc, VENC_DALRPC_SET_CB_CHANNEL,
			  &q6venc->cb_ev_data, sizeof(q6venc->cb_ev_data));
	if (err) {
		pr_err("%s: set_cb_channgel failed\n",__func__);
		goto err_dal_call_set_cb;
	}

	pr_info("%s() handle=%p enc_cb=%08x\n", __func__, q6venc->venc,
		q6venc->cb_ev_data.enc_cb_handle);

	return 0;

err_dal_call_set_cb:
	dal_detach(q6venc->venc);
err_dal_attach:
	file->private_data = NULL;
	mutex_destroy(&q6venc->lock);
	kfree(q6venc);
	return err;
}

static int q6venc_release(struct inode *inode, struct file *file)
{
	struct q6venc_dev *q6venc;
	int id;

	q6venc = file->private_data;
	file->private_data = NULL;

	pr_info("q6venc_close() handle=%p\n", q6venc->venc);

	dal_detach(q6venc->venc);

	for (id = 0; id < q6venc->num_enc_bufs; id++)
		put_buf_info(&q6venc->enc_bufs[id]);
	put_buf_info(&q6venc->rlc_bufs[0]);
	put_buf_info(&q6venc->rlc_bufs[1]);

	mutex_destroy(&q6venc->lock);
	kfree(q6venc);
	return 0;
}

static int q6_config_encode(struct q6venc_dev *q6venc, uint32_t type,
			    struct init_config *init_config)
{
	struct q6_init_config *q6_init_config = &init_config->q6_init_config;
	int ret;
	int i;

	mutex_lock(&q6venc->lock);

	if (q6venc->num_enc_bufs != 0) {
		pr_err("%s: multiple sessions not supported\n", __func__);
		ret = -EBUSY;
		goto err_busy;
	}

	ret = get_buf_info(&q6venc->enc_bufs[0], &init_config->ref_frame_buf1);
	if (ret) {
		pr_err("%s: can't get ref_frame_buf1\n", __func__);
		goto err_get_ref_frame_buf1;
	}

	ret = get_buf_info(&q6venc->enc_bufs[1], &init_config->ref_frame_buf2);
	if (ret) {
		pr_err("%s: can't get ref_frame_buf2\n", __func__);
		goto err_get_ref_frame_buf2;
	}

	ret = get_buf_info(&q6venc->rlc_bufs[0], &init_config->rlc_buf1);
	if (ret) {
		pr_err("%s: can't get rlc_buf1\n", __func__);
		goto err_get_rlc_buf1;
	}

	ret = get_buf_info(&q6venc->rlc_bufs[1], &init_config->rlc_buf2);
	if (ret) {
		pr_err("%s: can't get rlc_buf2\n", __func__);
		goto err_get_rlc_buf2;
	}
	q6venc->rlc_buf_len = 2 * q6_init_config->rlc_buf_length;
	q6venc->num_enc_bufs = 2;

	q6venc->enc_buf_size =
		(q6_init_config->enc_frame_width_inmb * PIXELS_PER_MACROBLOCK) *
		(q6_init_config->enc_frame_height_inmb * PIXELS_PER_MACROBLOCK) *
		BITS_PER_PIXEL / 8;

	q6_init_config->ref_frame_buf1_phy = q6venc->enc_bufs[0].paddr;
	q6_init_config->ref_frame_buf2_phy = q6venc->enc_bufs[1].paddr;
	q6_init_config->rlc_buf1_phy = q6venc->rlc_bufs[0].paddr;
	q6_init_config->rlc_buf2_phy = q6venc->rlc_bufs[1].paddr;

	// The DSP may use the rlc_bufs during initialization,
	for (i=0; i<RLC_MAX_BUF_NUM; i++)
	{
		dmac_inv_range((const void *)q6venc->rlc_bufs[i].vaddr,
			(const void *)(q6venc->rlc_bufs[i].vaddr +
				q6venc->rlc_buf_len));
	}

	ret = dal_call_f5(q6venc->venc, type, q6_init_config,
			  sizeof(struct q6_init_config));
	if (ret) {
		pr_err("%s: rpc failed \n", __func__);
		goto err_dal_rpc_init;
	}
	mutex_unlock(&q6venc->lock);
	return 0;

err_dal_rpc_init:
	q6venc->num_enc_bufs = 0;
	put_pmem_file(q6venc->rlc_bufs[1].file);
err_get_rlc_buf2:
	put_pmem_file(q6venc->rlc_bufs[0].file);
err_get_rlc_buf1:
	put_pmem_file(q6venc->enc_bufs[1].file);
err_get_ref_frame_buf2:
	put_pmem_file(q6venc->enc_bufs[0].file);
err_get_ref_frame_buf1:
err_busy:
	mutex_unlock(&q6venc->lock);
	return ret;
}

static int q6_encode(struct q6venc_dev *q6venc, struct encode_param *enc_param)
{
	struct q6_encode_param *q6_param = &enc_param->q6_encode_param;
	struct file *file;
	struct buf_info *buf;
	int i;
	int ret;
	int rlc_buf_index;

	pr_debug("y_addr fd=%d offset=0x%08lx uv_offset=0x%08lx\n",
		 enc_param->y_addr.fd, enc_param->y_addr.offset,
		 enc_param->uv_offset);

	file = fget(enc_param->y_addr.fd);
	if (!file) {
		pr_err("%s: invalid encode buffer fd %d\n", __func__,
		       enc_param->y_addr.fd);
		return -EBADF;
	}

	mutex_lock(&q6venc->lock);

	for (i = 0; i < q6venc->num_enc_bufs; i++) {
		buf = &q6venc->enc_bufs[i];
		if (buf->file == file
		    && buf->venc_buf.offset == enc_param->y_addr.offset)
			break;
	}

	if (i == q6venc->num_enc_bufs) {
		if (q6venc->num_enc_bufs == VENC_MAX_BUF_NUM) {
			pr_err("%s: too many input buffers\n", __func__);
			ret = -ENOMEM;
			goto done;
		}

		buf = &q6venc->enc_bufs[q6venc->num_enc_bufs];
		ret = get_buf_info(buf, &enc_param->y_addr);
		if (ret) {
			pr_err("%s: can't get encode buffer\n", __func__);
			ret = -EINVAL;
			goto done;
		}

		if (!IS_ALIGNED(buf->paddr, PAGE_SIZE)) {
			pr_err("%s: input buffer not 4k aligned\n", __func__);
			put_buf_info(buf);
			ret = -EINVAL;
			goto done;
		}
		q6venc->num_enc_bufs++;
	}

	// We must invalidate the buffer that the DSP will write to
	// to ensure that a dirty cache line doesn't get flushed on
	// top of the data that the DSP is writing.
	// Unfortunately, we have to predict which rlc_buf index the
	// DSP is going to write to.  We assume it will write to buf
	// 0 the first time we call q6_encode, and alternate afterwards
	rlc_buf_index = q6venc->rlc_buf_index;
	dmac_inv_range((const void *)q6venc->rlc_bufs[rlc_buf_index].vaddr,
		       (const void *)(q6venc->rlc_bufs[rlc_buf_index].vaddr +
				      q6venc->rlc_buf_len));
	q6venc->rlc_buf_index = (q6venc->rlc_buf_index + 1) % RLC_MAX_BUF_NUM;

	q6_param->luma_addr = buf->paddr;
	q6_param->chroma_addr = q6_param->luma_addr + enc_param->uv_offset;
	pr_debug("luma_addr=0x%08x chroma_addr=0x%08x\n", q6_param->luma_addr,
		 q6_param->chroma_addr);

	// Ideally, each ioctl that passed in a data buffer would include the size
	// of the input buffer, so we can properly flush the cache on it.  Since
	// userspace does not fill in the size fields, we have to assume the size
	// based on the encoder configuration for now.
	flush_pmem_file(buf->file, enc_param->y_addr.offset,
		q6venc->enc_buf_size);

	ret = dal_call_f5(q6venc->venc, VENC_DALRPC_ENCODE, q6_param,
			  sizeof(struct q6_encode_param));
	if (ret) {
		pr_err("%s: encode rpc failed\n", __func__);
		goto done;
	}

	ret = 0;

done:
	mutex_unlock(&q6venc->lock);
	fput(file);
	return ret;
}

static int q6venc_ioctl(struct inode *inode, struct file *file,
			unsigned cmd, unsigned long arg)
{
	struct q6venc_dev *q6venc = file->private_data;
	struct init_config config;
	struct encode_param encode_param;
	struct intra_refresh intra_refresh;
	struct rc_config rc_config;
	struct frame_type frame_done;
	unsigned int id;
	unsigned long flags;
	int err = 0;

	if (!q6venc) {
		pr_err("%s: file has no private data\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s\n", __func__);

	switch (cmd) {
	case VENC_IOCTL_INITIALIZE:
		pr_debug("%s: VENC_IOCTL_INITIALIZE\n", __func__);
		if (copy_from_user(&config, (void __user *)arg, sizeof(config)))
			return -EFAULT;
		err = q6_config_encode(q6venc, VENC_DALRPC_INITIALIZE, &config);
		break;

	case VENC_IOCTL_ENCODE_CONFIG:
		pr_debug("%s: VENC_IOCTL_ENCODE_CONFIG\n", __func__);
		if (copy_from_user(&config, (void __user *)arg, sizeof(config)))
			return -EFAULT;

		err = q6_config_encode(q6venc, VENC_DALRPC_ENCODE_CONFIG,
				       &config);
		break;

	case VENC_IOCTL_ENCODE:
		pr_debug("%s: VENC_IOCTL_ENCODE\n", __func__);
		if (copy_from_user(&encode_param, (void __user *)arg,
				   sizeof(encode_param)))
			return -EFAULT;
		err = q6_encode(q6venc, &encode_param);
		break;

	case VENC_IOCTL_INTRA_REFRESH:
		pr_debug("%s: VENC_IOCTL_INTRA_REFRESH\n", __func__);
		if (copy_from_user(&intra_refresh, (void __user *)arg,
				   sizeof(intra_refresh)))
			return -EFAULT;

		mutex_lock(&q6venc->lock);
		err = dal_call_f5(q6venc->venc, VENC_DALRPC_INTRA_REFRESH,
				  &intra_refresh, sizeof(struct intra_refresh));
		mutex_unlock(&q6venc->lock);
		if (err)
			pr_err("%s: intra_refresh rpc failed\n", __func__);
		break;

	case VENC_IOCTL_RC_CONFIG:
		pr_debug("%s: VENC_IOCTL_RC_CONFIG\n", __func__);
		if (copy_from_user(&rc_config, (void __user *)arg,
				   sizeof(rc_config)))
			return -EFAULT;

		mutex_lock(&q6venc->lock);
		err = dal_call_f5(q6venc->venc, VENC_DALRPC_RC_CONFIG,
				  &rc_config, sizeof(rc_config));
		mutex_unlock(&q6venc->lock);
		if (err)
			pr_err("%s: dal_call_f5 failed\n", __func__);
		break;

	case VENC_IOCTL_STOP:
		pr_debug("%s: VENC_IOCTL_STOP\n", __func__);

		mutex_lock(&q6venc->lock);
		err = dal_call_f0(q6venc->venc, VENC_DALRPC_STOP, 1);
		if (err)
			pr_err("%s: dal_rpc STOP call failed\n", __func__);

		/* XXX: if the dal call fails we still want to continue to free
		 * the buffers. Is this correct? */
		for (id = 0; id < q6venc->num_enc_bufs; id++)
			put_buf_info(&q6venc->enc_bufs[id]);
		put_buf_info(&q6venc->rlc_bufs[0]);
		put_buf_info(&q6venc->rlc_bufs[1]);
		q6venc->num_enc_bufs = 0;
		q6venc->stop_encode = true;
		mutex_unlock(&q6venc->lock);
		break;

	case VENC_IOCTL_WAIT_FOR_ENCODE:
		pr_debug("%s: waiting for encode done event \n", __func__);
		err = wait_event_interruptible(q6venc->encode_wq,
				(q6venc->encode_done || q6venc->stop_encode));
		if (err < 0) {
			err = -ERESTARTSYS;
			break;
		}

		mutex_lock(&q6venc->lock);
		if (q6venc->stop_encode) {
			q6venc->stop_encode = false;
			mutex_unlock(&q6venc->lock);
			pr_debug("%s: Received Stop encode event \n", __func__);
			err = -EINTR;
			break;
		}

		spin_lock_irqsave(&q6venc->done_lock, flags);
		if (!q6venc->encode_done) {
			spin_unlock_irqrestore(&q6venc->done_lock, flags);
			pr_err("%s: encoding not stopped, and is not done.\n",
			       __func__);
			err = -EIO;
			break;
		}

		memcpy(&frame_done, &q6venc->done_frame,
		       sizeof(struct frame_type));
		q6venc->encode_done = false;
		spin_unlock_irqrestore(&q6venc->done_lock, flags);
		mutex_unlock(&q6venc->lock);

		if (frame_done.q6_frame_type.frame_len == 0) {
			pr_debug("%s: got incorrect address from q6\n",
				 __func__);
			err = -EIO;
			break;
		}

		pr_debug("%s: done encoding \n", __func__);
		if (copy_to_user((void __user *)arg, &frame_done,
				 sizeof(struct frame_type)))
			err = -EFAULT;
		break;

	case VENC_IOCTL_STOP_ENCODE:
		pr_debug("%s: Stop  encode event   \n", __func__);
		mutex_lock(&q6venc->lock);
		q6venc->stop_encode = true;
		wake_up_interruptible(&q6venc->encode_wq);
		mutex_unlock(&q6venc->lock);
		break;

	default:
		err = -ENOTTY;
		break;
	}

	return err;
}

static const struct file_operations q6venc_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= q6venc_open,
	.release	= q6venc_release,
	.ioctl		= q6venc_ioctl,
};

static struct miscdevice q6venc_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "q6venc",
	.fops	= &q6venc_dev_fops,
};

static int __init q6venc_init(void)
{
	int rc = 0;

	rc = misc_register(&q6venc_misc);
	if (rc)
		pr_err("%s: Unable to register q6venc misc device\n", __func__);
	return rc;
}

static void __exit q6venc_exit(void)
{
	misc_deregister(&q6venc_misc);
}

MODULE_DESCRIPTION("video encoder driver for QSD platform");
MODULE_VERSION("2.0");

module_init(q6venc_init);
module_exit(q6venc_exit);

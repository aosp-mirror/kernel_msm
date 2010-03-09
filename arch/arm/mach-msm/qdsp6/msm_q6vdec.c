/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

/*
#define DEBUG_TRACE_VDEC
#define DEBUG
*/

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>

#include <linux/android_pmem.h>
#include <linux/msm_q6vdec.h>

#include "../dal.h"

#define DALDEVICEID_VDEC_DEVICE		0x02000026
#define DALDEVICEID_VDEC_PORTNAME	"DSP_DAL_AQ_VID"

#define VDEC_INTERFACE_VERSION		0x00020000

#define MAJOR_MASK			0xFFFF0000
#define MINOR_MASK			0x0000FFFF

#define VDEC_GET_MAJOR_VERSION(version)	(((version)&MAJOR_MASK)>>16)

#define VDEC_GET_MINOR_VERSION(version)	((version)&MINOR_MASK)

#ifdef DEBUG_TRACE_VDEC
#define TRACE(fmt,x...)			\
	do { pr_debug("%s:%d " fmt, __func__, __LINE__, ##x); } while (0)
#else
#define TRACE(fmt,x...)		do { } while (0)
#endif


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
	VDEC_DALRPC_INITIALIZE = DAL_OP_FIRST_DEVICE_API,
	VDEC_DALRPC_SETBUFFERS,
	VDEC_DALRPC_FREEBUFFERS,
	VDEC_DALRPC_QUEUE,
	VDEC_DALRPC_SIGEOFSTREAM,
	VDEC_DALRPC_FLUSH,
	VDEC_DALRPC_REUSEFRAMEBUFFER,
	VDEC_DALRPC_GETDECATTRIBUTES,
};

enum {
	VDEC_ASYNCMSG_DECODE_DONE = 0xdec0de00,
	VDEC_ASYNCMSG_REUSE_FRAME,
};

struct vdec_init_cfg {
	u32			decode_done_evt;
	u32			reuse_frame_evt;
	struct vdec_config	cfg;
};

struct vdec_buffer_status {
	u32			data;
	u32			status;
};

#define VDEC_MSG_MAX		128

struct vdec_msg_list {
	struct list_head	list;
	struct vdec_msg		vdec_msg;
};

struct vdec_mem_info {
	u32			buf_type;
	u32			id;
	unsigned long		phys_addr;
	unsigned long		len;
	struct file		*file;
};

struct vdec_mem_list {
	struct list_head	list;
	struct vdec_mem_info	mem;
};

struct vdec_data {
	struct dal_client	*vdec_handle;
	struct list_head	vdec_msg_list_head;
	struct list_head	vdec_msg_list_free;
	wait_queue_head_t	vdec_msg_evt;
	spinlock_t		vdec_list_lock;
	struct list_head	vdec_mem_list_head;
	spinlock_t		vdec_mem_list_lock;
	int			mem_initialized;
	int			running;
	int			close_decode;
};

static struct class *driver_class;
static dev_t vdec_device_no;
static struct cdev vdec_cdev;
static int ref_cnt;
static DEFINE_MUTEX(vdec_ref_lock);

static inline int vdec_check_version(u32 client, u32 server)
{
	int ret = -EINVAL;
	if ((VDEC_GET_MAJOR_VERSION(client) == VDEC_GET_MAJOR_VERSION(server))
	    && (VDEC_GET_MINOR_VERSION(client) <=
		VDEC_GET_MINOR_VERSION(server)))
		ret = 0;
	return ret;
}

static int vdec_get_msg(struct vdec_data *vd, void *msg)
{
	struct vdec_msg_list *l;
	unsigned long flags;
	int ret = 0;

	if (!vd->running)
		return -EPERM;

	spin_lock_irqsave(&vd->vdec_list_lock, flags);
	list_for_each_entry_reverse(l, &vd->vdec_msg_list_head, list) {
		if (copy_to_user(msg, &l->vdec_msg, sizeof(struct vdec_msg)))
			pr_err("vdec_get_msg failed to copy_to_user!\n");
		if (l->vdec_msg.id == VDEC_MSG_REUSEINPUTBUFFER)
			TRACE("reuse_input_buffer %d\n", l->vdec_msg.buf_id);
		else if (l->vdec_msg.id == VDEC_MSG_FRAMEDONE)
			TRACE("frame_done (stat=%d)\n",
			      l->vdec_msg.vfr_info.status);
		else
			TRACE("unknown msg (msgid=%d)\n", l->vdec_msg.id);
		list_del(&l->list);
		list_add(&l->list, &vd->vdec_msg_list_free);
		ret = 1;
		break;
	}
	spin_unlock_irqrestore(&vd->vdec_list_lock, flags);

	if (vd->close_decode)
		ret = 1;

	return ret;
}

static void vdec_put_msg(struct vdec_data *vd, struct vdec_msg *msg)
{
	struct vdec_msg_list *l;
	unsigned long flags;
	int found = 0;

	spin_lock_irqsave(&vd->vdec_list_lock, flags);
	list_for_each_entry(l, &vd->vdec_msg_list_free, list) {
		memcpy(&l->vdec_msg, msg, sizeof(struct vdec_msg));
		list_del(&l->list);
		list_add(&l->list, &vd->vdec_msg_list_head);
		found = 1;
		break;
	}
	spin_unlock_irqrestore(&vd->vdec_list_lock, flags);

	if (found)
		wake_up(&vd->vdec_msg_evt);
	else
		pr_err("vdec_put_msg can't find free list!\n");
}

static struct vdec_mem_list *vdec_get_mem_from_list(struct vdec_data *vd,
						    u32 pmem_id, u32 buf_type)
{
	struct vdec_mem_list *l;
	unsigned long flags;
	int found = 0;

	spin_lock_irqsave(&vd->vdec_mem_list_lock, flags);
	list_for_each_entry(l, &vd->vdec_mem_list_head, list) {
		if (l->mem.buf_type == buf_type && l->mem.id == pmem_id) {
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&vd->vdec_mem_list_lock, flags);

	if (found)
		return l;
	else
		return NULL;

}

static int vdec_initialize(struct vdec_data *vd, void *argp)
{
	struct vdec_config_sps vdec_cfg_sps;
	struct vdec_init_cfg vi_cfg;
	struct vdec_buf_req vdec_buf_req;
	struct u8 *header;
	int ret = 0;

	ret = copy_from_user(&vdec_cfg_sps,
			     &((struct vdec_init *)argp)->sps_cfg,
			     sizeof(vdec_cfg_sps));

	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	vi_cfg.decode_done_evt = VDEC_ASYNCMSG_DECODE_DONE;
	vi_cfg.reuse_frame_evt = VDEC_ASYNCMSG_REUSE_FRAME;
	memcpy(&vi_cfg.cfg, &vdec_cfg_sps.cfg, sizeof(struct vdec_config));

	header = kmalloc(vdec_cfg_sps.seq.len, GFP_KERNEL);
	if (!header) {
		pr_err("%s: kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	ret = copy_from_user(header,
			     ((struct vdec_init *)argp)->sps_cfg.seq.header,
			     vdec_cfg_sps.seq.len);

	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		kfree(header);
		return ret;
	}

	TRACE("vi_cfg: handle=%p fourcc=0x%x w=%d h=%d order=%d notify_en=%d "
	      "vc1_rb=%d h264_sd=%d h264_nls=%d pp_flag=%d fruc_en=%d\n",
	      vd->vdec_handle, vi_cfg.cfg.fourcc, vi_cfg.cfg.width,
	      vi_cfg.cfg.height, vi_cfg.cfg.order, vi_cfg.cfg.notify_enable,
	      vi_cfg.cfg.vc1_rowbase, vi_cfg.cfg.h264_startcode_detect,
	      vi_cfg.cfg.h264_nal_len_size, vi_cfg.cfg.postproc_flag,
	      vi_cfg.cfg.fruc_enable);
	ret = dal_call_f13(vd->vdec_handle, VDEC_DALRPC_INITIALIZE,
			   &vi_cfg, sizeof(vi_cfg),
			   header, vdec_cfg_sps.seq.len,
			   &vdec_buf_req, sizeof(vdec_buf_req));

	kfree(header);

	if (ret)
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
	else
		ret = copy_to_user(((struct vdec_init *)argp)->buf_req,
				   &vdec_buf_req, sizeof(vdec_buf_req));

	vd->close_decode = 0;
	return ret;
}

static int vdec_setbuffers(struct vdec_data *vd, void *argp)
{
	struct vdec_buffer vmem;
	struct vdec_mem_list *l;
	unsigned long vstart;
	unsigned long flags;
	struct {
		uint32_t size;
		struct vdec_buf_info buf;
	} rpc;
	uint32_t res;

	int ret = 0;

	vd->mem_initialized = 0;

	ret = copy_from_user(&vmem, argp, sizeof(vmem));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	l = kzalloc(sizeof(struct vdec_mem_list), GFP_KERNEL);
	if (!l) {
		pr_err("%s: kzalloc failed!\n", __func__);
		return -ENOMEM;
	}

	l->mem.id = vmem.pmem_id;
	l->mem.buf_type = vmem.buf.buf_type;

	ret = get_pmem_file(l->mem.id, &l->mem.phys_addr, &vstart,
			    &l->mem.len, &l->mem.file);
	if (ret) {
		pr_err("%s: get_pmem_fd failed\n", __func__);
		goto err_get_pmem_file;
	}

	TRACE("pmem_id=%d (phys=0x%08lx len=0x%lx) buftype=%d num_buf=%d "
	      "islast=%d src_id=%d offset=0x%08x size=0x%x\n",
	      vmem.pmem_id, l->mem.phys_addr, l->mem.len,
	      vmem.buf.buf_type, vmem.buf.num_buf, vmem.buf.islast,
	      vmem.buf.region.src_id, vmem.buf.region.offset,
	      vmem.buf.region.size);

	/* input buffers */
	if ((vmem.buf.region.offset + vmem.buf.region.size) > l->mem.len) {
		pr_err("%s: invalid input buffer offset!\n", __func__);
		ret = -EINVAL;
		goto err_bad_offset;

	}
	vmem.buf.region.offset += l->mem.phys_addr;

	rpc.size = sizeof(vmem.buf);
	memcpy(&rpc.buf, &vmem.buf, sizeof(struct vdec_buf_info));


	ret = dal_call(vd->vdec_handle, VDEC_DALRPC_SETBUFFERS, 5,
		       &rpc, sizeof(rpc), &res, sizeof(res));

	if (ret < 4) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
		ret = -EIO;
		goto err_dal_call;
	}

	spin_lock_irqsave(&vd->vdec_mem_list_lock, flags);
	list_add(&l->list, &vd->vdec_mem_list_head);
	spin_unlock_irqrestore(&vd->vdec_mem_list_lock, flags);

	vd->mem_initialized = 1;
	return ret;

err_dal_call:
err_bad_offset:
	put_pmem_file(l->mem.file);
err_get_pmem_file:
	kfree(l);
	return ret;
}

static int vdec_queue(struct vdec_data *vd, void *argp)
{
	struct {
		uint32_t size;
		struct vdec_input_buf_info buf_info;
		uint32_t osize;
	} rpc;
	struct vdec_mem_list *l;
	struct {
		uint32_t result;
		uint32_t size;
		struct vdec_queue_status status;
	} rpc_res;

	u32 pmem_id;
	int ret = 0;

	if (!vd->mem_initialized) {
		pr_err("%s: memory is not being initialized!\n", __func__);
		return -EPERM;
	}

	ret = copy_from_user(&rpc.buf_info,
			     &((struct vdec_input_buf *)argp)->buffer,
			     sizeof(rpc.buf_info));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	ret = copy_from_user(&pmem_id,
			     &((struct vdec_input_buf *)argp)->pmem_id,
			     sizeof(u32));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	l = vdec_get_mem_from_list(vd, pmem_id, VDEC_BUFFER_TYPE_INPUT);

	if (NULL == l) {
		pr_err("%s: not able to find the buffer from list\n", __func__);
		return -EPERM;
	}

	if ((rpc.buf_info.size + rpc.buf_info.offset) >= l->mem.len) {
		pr_err("%s: invalid queue buffer offset!\n", __func__);
		return -EINVAL;
	}

	rpc.buf_info.offset += l->mem.phys_addr;
	rpc.size = sizeof(struct vdec_input_buf_info);
	rpc.osize = sizeof(struct vdec_queue_status);

	ret = dal_call(vd->vdec_handle, VDEC_DALRPC_QUEUE, 8,
		       &rpc, sizeof(rpc), &rpc_res, sizeof(rpc_res));
	if (ret < 4) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
		ret = -EIO;
	}
	return ret;
}

static int vdec_reuse_framebuffer(struct vdec_data *vd, void *argp)
{
	u32 buf_id;
	int ret = 0;

	ret = copy_from_user(&buf_id, argp, sizeof(buf_id));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	ret = dal_call_f0(vd->vdec_handle, VDEC_DALRPC_REUSEFRAMEBUFFER,
			  buf_id);
	if (ret)
		pr_err("%s: remote function failed (%d)\n", __func__, ret);

	return ret;
}

static int vdec_flush(struct vdec_data *vd, void *argp)
{
	u32 flush_type;
	int ret = 0;

	ret = copy_from_user(&flush_type, argp, sizeof(flush_type));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	TRACE("flush_type=%d\n", flush_type);
	ret = dal_call_f0(vd->vdec_handle, VDEC_DALRPC_FLUSH, flush_type);
	if (ret) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int vdec_close(struct vdec_data *vd, void *argp)
{
	struct vdec_mem_list *l;
	int ret = 0;

	pr_info("q6vdec_close()\n");
	vd->close_decode = 1;
	wake_up(&vd->vdec_msg_evt);
	ret = dal_call_f0(vd->vdec_handle, DAL_OP_CLOSE, 0);
	if (ret)
		pr_err("%s: failed to close daldevice (%d)\n", __func__, ret);

	if (vd->mem_initialized) {
		list_for_each_entry(l, &vd->vdec_mem_list_head, list)
		    put_pmem_file(l->mem.file);
	}

	return ret;
}
static int vdec_getdecattributes(struct vdec_data *vd, void *argp)
{
	struct {
		uint32_t status;
		uint32_t size;
		struct vdec_dec_attributes dec_attr;
	} rpc;
	uint32_t inp;
	int ret = 0;
	inp = sizeof(struct vdec_dec_attributes);

	ret = dal_call(vd->vdec_handle, VDEC_DALRPC_GETDECATTRIBUTES, 9,
		       &inp, sizeof(inp), &rpc, sizeof(rpc));
	if (ret < 4 || rpc.size != sizeof(struct vdec_dec_attributes)) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
		ret = -EIO;
	} else
		ret =
		    copy_to_user(((struct vdec_dec_attributes *)argp),
				 &rpc.dec_attr, sizeof(rpc.dec_attr));
	return ret;
}

static int vdec_freebuffers(struct vdec_data *vd, void *argp)
{
	struct vdec_buffer vmem;
	struct vdec_mem_list *l;
	struct {
		uint32_t size;
		struct vdec_buf_info buf;
	} rpc;
	uint32_t res;

	int ret = 0;

	if (!vd->mem_initialized) {
		pr_err("%s: memory is not being initialized!\n", __func__);
		return -EPERM;
	}

	ret = copy_from_user(&vmem, argp, sizeof(vmem));
	if (ret) {
		pr_err("%s: copy_from_user failed\n", __func__);
		return ret;
	}

	l = vdec_get_mem_from_list(vd, vmem.pmem_id, vmem.buf.buf_type);

	if (NULL == l) {
		pr_err("%s: not able to find the buffer from list\n", __func__);
		return -EPERM;
	}

	/* input buffers */
	if ((vmem.buf.region.offset + vmem.buf.region.size) > l->mem.len) {
		pr_err("%s: invalid input buffer offset!\n", __func__);
		return -EINVAL;

	}
	vmem.buf.region.offset += l->mem.phys_addr;

	rpc.size = sizeof(vmem.buf);
	memcpy(&rpc.buf, &vmem.buf, sizeof(struct vdec_buf_info));

	ret = dal_call(vd->vdec_handle, VDEC_DALRPC_FREEBUFFERS, 5,
		       &rpc, sizeof(rpc), &res, sizeof(res));
	if (ret < 4) {
		pr_err("%s: remote function failed (%d)\n", __func__, ret);
	}

	return ret;
}
static long vdec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vdec_data *vd = file->private_data;
	void __user *argp = (void __user *)arg;
	int ret = 0;

	if (!vd->running)
		return -EPERM;

	switch (cmd) {
	case VDEC_IOCTL_INITIALIZE:
		ret = vdec_initialize(vd, argp);
		break;

	case VDEC_IOCTL_SETBUFFERS:
		ret = vdec_setbuffers(vd, argp);
		break;

	case VDEC_IOCTL_QUEUE:
		TRACE("VDEC_IOCTL_QUEUE (pid=%d tid=%d)\n",
		      current->group_leader->pid, current->pid);
		ret = vdec_queue(vd, argp);
		break;

	case VDEC_IOCTL_REUSEFRAMEBUFFER:
		TRACE("VDEC_IOCTL_REUSEFRAMEBUFFER (pid=%d tid=%d)\n",
		      current->group_leader->pid, current->pid);
		ret = vdec_reuse_framebuffer(vd, argp);
		break;

	case VDEC_IOCTL_FLUSH:
		ret = vdec_flush(vd, argp);
		break;

	case VDEC_IOCTL_EOS:
		TRACE("VDEC_IOCTL_EOS (pid=%d tid=%d)\n",
		      current->group_leader->pid, current->pid);
		ret = dal_call_f0(vd->vdec_handle, VDEC_DALRPC_SIGEOFSTREAM, 0);
		if (ret)
			pr_err("%s: remote function failed (%d)\n",
			       __func__, ret);
		break;

	case VDEC_IOCTL_GETMSG:
		TRACE("VDEC_IOCTL_GETMSG (pid=%d tid=%d)\n",
		      current->group_leader->pid, current->pid);
		wait_event_interruptible(vd->vdec_msg_evt,
					 vdec_get_msg(vd, argp));

		if (vd->close_decode)
			ret = -EINTR;
		break;

	case VDEC_IOCTL_CLOSE:
		ret = vdec_close(vd, argp);
		break;

	case VDEC_IOCTL_GETDECATTRIBUTES:
		TRACE("VDEC_IOCTL_GETDECATTRIBUTES (pid=%d tid=%d)\n",
		      current->group_leader->pid, current->pid);
		ret = vdec_getdecattributes(vd, argp);

		if (ret)
			pr_err("%s: remote function failed (%d)\n",
			       __func__, ret);
		break;

	case VDEC_IOCTL_FREEBUFFERS:
		TRACE("VDEC_IOCTL_FREEBUFFERS (pid=%d tid=%d)\n",
		      current->group_leader->pid, current->pid);
		ret = vdec_freebuffers(vd, argp);

		if (ret)
			pr_err("%s: remote function failed (%d)\n",
			       __func__, ret);
		break;

	default:
		pr_err("%s: invalid ioctl!\n", __func__);
		ret = -EINVAL;
		break;
	}

	TRACE("ioctl done (pid=%d tid=%d)\n",
	      current->group_leader->pid, current->pid);

	return ret;
}

static void vdec_dcdone_handler(struct vdec_data *vd, void *frame,
				uint32_t frame_size)
{
	struct vdec_msg msg;
	struct vdec_mem_list *l;
	unsigned long flags;
	int found = 0;

	if (frame_size < sizeof(struct vdec_frame_info)) {
		pr_warning("%s: msg size mismatch %d != %d\n", __func__,
			   frame_size, sizeof(struct vdec_frame_info));
		return;
	}

	memcpy(&msg.vfr_info, (struct vdec_frame_info *)frame,
	       sizeof(struct vdec_frame_info));

	if (msg.vfr_info.status == VDEC_FRAME_DECODE_OK) {
		spin_lock_irqsave(&vd->vdec_mem_list_lock, flags);
		list_for_each_entry(l, &vd->vdec_mem_list_head, list) {
			if ((l->mem.buf_type == VDEC_BUFFER_TYPE_OUTPUT) &&
			    (msg.vfr_info.offset >= l->mem.phys_addr) &&
			    (msg.vfr_info.offset <
			     (l->mem.phys_addr + l->mem.len))) {
				found = 1;
				msg.vfr_info.offset -= l->mem.phys_addr;
				msg.vfr_info.data2 = l->mem.id;
				break;
			}
		}
		spin_unlock_irqrestore(&vd->vdec_mem_list_lock, flags);
	}

	if (found || (msg.vfr_info.status != VDEC_FRAME_DECODE_OK)) {
		msg.id = VDEC_MSG_FRAMEDONE;
		vdec_put_msg(vd, &msg);
	} else {
		pr_err("%s: invalid phys addr = 0x%x\n",
		       __func__, msg.vfr_info.offset);
	}

}

static void vdec_reuseibuf_handler(struct vdec_data *vd, void *bufstat,
				   uint32_t bufstat_size)
{
	struct vdec_buffer_status *vdec_bufstat;
	struct vdec_msg msg;

	/* TODO: how do we signal the client? If they are waiting on a
	 * message in an ioctl, they may block forever */
	if (bufstat_size != sizeof(struct vdec_buffer_status)) {
		pr_warning("%s: msg size mismatch %d != %d\n", __func__,
			   bufstat_size, sizeof(struct vdec_buffer_status));
		return;
	}
	vdec_bufstat = (struct vdec_buffer_status *)bufstat;
	msg.id = VDEC_MSG_REUSEINPUTBUFFER;
	msg.buf_id = vdec_bufstat->data;
	vdec_put_msg(vd, &msg);
}

static void callback(void *data, int len, void *cookie)
{
	struct vdec_data *vd = (struct vdec_data *)cookie;
	uint32_t *tmp = (uint32_t *) data;

	if (!vd->mem_initialized) {
		pr_err("%s:memory not initialize but callback called!\n",
		       __func__);
		return;
	}

	TRACE("vdec_async: tmp=0x%08x 0x%08x 0x%08x\n", tmp[0], tmp[1], tmp[2]);
	switch (tmp[0]) {
	case VDEC_ASYNCMSG_DECODE_DONE:
		vdec_dcdone_handler(vd, &tmp[3], tmp[2]);
		break;
	case VDEC_ASYNCMSG_REUSE_FRAME:
		vdec_reuseibuf_handler(vd, &tmp[3], tmp[2]);
		break;
	default:
		pr_err("%s: Unknown async message from DSP id=0x%08x sz=%u\n",
		       __func__, tmp[0], tmp[2]);
	}
}
static int vdec_open(struct inode *inode, struct file *file)
{
	int ret;
	int i;
	struct vdec_msg_list *l;
	struct vdec_data *vd;

	pr_info("q6vdec_open()\n");
	mutex_lock(&vdec_ref_lock);
	if (ref_cnt > 0) {
		pr_err("%s: Instance alredy running\n", __func__);
		mutex_unlock(&vdec_ref_lock);
		return -ENOMEM;
	}
	ref_cnt++;
	mutex_unlock(&vdec_ref_lock);
	vd = kmalloc(sizeof(struct vdec_data), GFP_KERNEL);
	if (!vd) {
		pr_err("%s: kmalloc failed\n", __func__);
		ret = -ENOMEM;
		goto vdec_open_err_handle_vd;
	}
	file->private_data = vd;

	vd->mem_initialized = 0;
	INIT_LIST_HEAD(&vd->vdec_msg_list_head);
	INIT_LIST_HEAD(&vd->vdec_msg_list_free);
	INIT_LIST_HEAD(&vd->vdec_mem_list_head);
	init_waitqueue_head(&vd->vdec_msg_evt);

	spin_lock_init(&vd->vdec_list_lock);
	spin_lock_init(&vd->vdec_mem_list_lock);
	for (i = 0; i < VDEC_MSG_MAX; i++) {
		l = kzalloc(sizeof(struct vdec_msg_list), GFP_KERNEL);
		if (!l) {
			pr_err("%s: kzalloc failed!\n", __func__);
			ret = -ENOMEM;
			goto vdec_open_err_handle_list;
		}
		list_add(&l->list, &vd->vdec_msg_list_free);
	}

	vd->vdec_handle = dal_attach(DALDEVICEID_VDEC_DEVICE,
				     DALDEVICEID_VDEC_PORTNAME,
				     callback, vd);

	if (!vd->vdec_handle) {
		pr_err("%s: failed to attach \n", __func__);
		ret = -EIO;
		goto vdec_open_err_handle_list;
	}

	vd->running = 1;
	prevent_sleep();
	return 0;

vdec_open_err_handle_list:
	{
		struct vdec_msg_list *l, *n;
		list_for_each_entry_safe(l, n, &vd->vdec_msg_list_free, list) {
			list_del(&l->list);
			kfree(l);
		}
	}
vdec_open_err_handle_vd:
	kfree(vd);
	return ret;
}

static int vdec_release(struct inode *inode, struct file *file)
{
	int ret;
	struct vdec_msg_list *l, *n;
	struct vdec_mem_list *m, *k;
	struct vdec_data *vd = file->private_data;

	vd->running = 0;
	wake_up_all(&vd->vdec_msg_evt);

	if (!vd->close_decode)
		vdec_close(vd, NULL);

	ret = dal_detach(vd->vdec_handle);
	if (ret)
		printk(KERN_INFO "%s: failed to detach (%d)\n", __func__, ret);

	list_for_each_entry_safe(l, n, &vd->vdec_msg_list_free, list) {
		list_del(&l->list);
		kfree(l);
	}

	list_for_each_entry_safe(l, n, &vd->vdec_msg_list_head, list) {
		list_del(&l->list);
		kfree(l);
	}

	list_for_each_entry_safe(m, k, &vd->vdec_mem_list_head, list) {
		list_del(&m->list);
		kfree(m);
	}
	mutex_lock(&vdec_ref_lock);
	BUG_ON(ref_cnt <= 0);
	ref_cnt--;
	mutex_unlock(&vdec_ref_lock);
	kfree(vd);
	allow_sleep();
	return 0;
}

static const struct file_operations vdec_fops = {
	.owner = THIS_MODULE,
	.open = vdec_open,
	.release = vdec_release,
	.unlocked_ioctl = vdec_ioctl,
};

static int __init vdec_init(void)
{
	struct device *class_dev;
	int rc = 0;

	wake_lock_init(&idlelock, WAKE_LOCK_IDLE, "vdec_idle");
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "vdec_suspend");

	rc = alloc_chrdev_region(&vdec_device_no, 0, 1, "vdec");
	if (rc < 0) {
		pr_err("%s: alloc_chrdev_region failed %d\n", __func__, rc);
		return rc;
	}

	driver_class = class_create(THIS_MODULE, "vdec");
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		pr_err("%s: class_create failed %d\n", __func__, rc);
		goto vdec_init_err_unregister_chrdev_region;
	}
	class_dev = device_create(driver_class, NULL,
				  vdec_device_no, NULL, "vdec");
	if (!class_dev) {
		pr_err("%s: class_device_create failed %d\n", __func__, rc);
		rc = -ENOMEM;
		goto vdec_init_err_class_destroy;
	}

	cdev_init(&vdec_cdev, &vdec_fops);
	vdec_cdev.owner = THIS_MODULE;
	rc = cdev_add(&vdec_cdev, MKDEV(MAJOR(vdec_device_no), 0), 1);

	if (rc < 0) {
		pr_err("%s: cdev_add failed %d\n", __func__, rc);
		goto vdec_init_err_class_device_destroy;
	}

	return 0;

vdec_init_err_class_device_destroy:
	device_destroy(driver_class, vdec_device_no);
vdec_init_err_class_destroy:
	class_destroy(driver_class);
vdec_init_err_unregister_chrdev_region:
	unregister_chrdev_region(vdec_device_no, 1);
	return rc;
}

static void __exit vdec_exit(void)
{
	device_destroy(driver_class, vdec_device_no);
	class_destroy(driver_class);
	unregister_chrdev_region(vdec_device_no, 1);
}

MODULE_DESCRIPTION("video decoder driver for QSD platform");
MODULE_VERSION("2.00");

module_init(vdec_init);
module_exit(vdec_exit);

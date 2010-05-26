/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/android_pmem.h>
#include <linux/clk.h>

#include "video_core_type.h"
#include "vcd_api.h"
#include "venc_internal.h"
#include "video_core_init.h"

#define VID_ENC_NAME   		"msm_vidc_enc"
#define VID_C_HCLK_RATE        170667000

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define INFO(x...) printk(KERN_INFO x)
#define ERR(x...) printk(KERN_ERR x)

static struct vid_enc_dev *vid_enc_device_p;
static dev_t vid_enc_dev_num;
static struct class *vid_enc_class;
static int vid_enc_ioctl(struct inode *inode, struct file *file,
	unsigned cmd, unsigned long arg);
static int stop_cmd;

static s32 vid_enc_get_empty_client_index(void)
{
	u32 i;
	u32 found = FALSE;

	for (i = 0; i < VID_ENC_MAX_ENCODER_CLIENTS; i++) {
		if (!vid_enc_device_p->venc_clients[i].vcd_handle) {
			found = TRUE;
			break;
		}
	}
	if (!found) {
		ERR("%s():ERROR No space for new client\n",
			__func__);
		return -1;
	} else {
		DBG("%s(): available client index = %u\n",
			__func__, i);
		return i;
	}
}


u32 vid_enc_get_status(u32 status)
{
	u32 venc_status;

	switch (status) {
	case VCD_S_SUCCESS:
		venc_status = VEN_S_SUCCESS;
		break;
	case VCD_ERR_FAIL:
		venc_status = VEN_S_EFAIL;
		break;
	case VCD_ERR_ALLOC_FAIL:
		venc_status = VEN_S_ENOSWRES;
		break;
	case VCD_ERR_ILLEGAL_OP:
		venc_status = VEN_S_EINVALCMD;
		break;
	case VCD_ERR_ILLEGAL_PARM:
		venc_status = VEN_S_EBADPARAM;
		break;
	case VCD_ERR_BAD_POINTER:
	case VCD_ERR_BAD_HANDLE:
		venc_status = VEN_S_EFATAL;
		break;
	case VCD_ERR_NOT_SUPPORTED:
		venc_status = VEN_S_ENOTSUPP;
		break;
	case VCD_ERR_BAD_STATE:
		venc_status = VEN_S_EINVALSTATE;
		break;
	case VCD_ERR_MAX_CLIENT:
		venc_status = VEN_S_ENOHWRES;
		break;
	default:
		venc_status = VEN_S_EFAIL;
		break;
	}
	return venc_status;
}

static void vid_enc_notify_client(struct video_client_ctx *client_ctx)
{
	if (client_ctx)
		complete(&client_ctx->event);
}

void vid_enc_vcd_open_done(struct video_client_ctx *client_ctx,
	struct vcd_handle_container_type *handle_container)
{
	DBG("vid_enc_vcd_open_done\n");

	if (client_ctx) {
		if (handle_container)
			client_ctx->vcd_handle = handle_container->handle;
		else
		ERR("%s(): ERROR. handle_container is NULL\n",
		__func__);
		vid_enc_notify_client(client_ctx);
	} else
		ERR("%s(): ERROR. client_ctx is NULL\n",
			__func__);
}

static void vid_enc_input_frame_done(struct video_client_ctx *client_ctx,
		u32 event, u32 status,
		struct vcd_frame_data_type *vcd_frame_data)
{
	struct vid_enc_msg *venc_msg;

	if (!client_ctx || !vcd_frame_data) {
		ERR("vid_enc_input_frame_done() NULL pointer \n");
		return;
	}

	venc_msg = kzalloc(sizeof(struct vid_enc_msg),
					    GFP_KERNEL);
	if (!venc_msg) {
		ERR("vid_enc_input_frame_done(): cannot allocate vid_enc_msg "
		" buffer\n");
		return;
	}

	venc_msg->venc_msg_info.statuscode = vid_enc_get_status(status);

	if (event == VCD_EVT_RESP_INPUT_DONE) {
		venc_msg->venc_msg_info.msgcode =
			VEN_MSG_INPUT_BUFFER_DONE;
		DBG("Send INPUT_DON message to client = %p\n",
			client_ctx);
	} else if (event == VCD_EVT_RESP_INPUT_FLUSHED) {
		venc_msg->venc_msg_info.msgcode = VEN_MSG_INPUT_BUFFER_DONE;
		DBG("Send INPUT_FLUSHED message to client = %p\n",
			client_ctx);
	} else {
		ERR("vid_enc_input_frame_done(): invalid event type\n");
		return;
	}

	venc_msg->venc_msg_info.buf.clientdata =
		(void *)vcd_frame_data->n_frm_clnt_data;
	venc_msg->venc_msg_info.msgdata_size =
		sizeof(struct vid_enc_msg);

	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&venc_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}

static void vid_enc_output_frame_done(struct video_client_ctx *client_ctx,
		u32 event, u32 status,
		struct vcd_frame_data_type *vcd_frame_data)
{
	struct vid_enc_msg *venc_msg;
	unsigned long kernel_vaddr, phy_addr, user_vaddr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;

	if (!client_ctx || !vcd_frame_data) {
		ERR("vid_enc_input_frame_done() NULL pointer \n");
		return;
	}

	venc_msg = kzalloc(sizeof(struct vid_enc_msg),
					   GFP_KERNEL);
	if (!venc_msg) {
		ERR("vid_enc_input_frame_done(): cannot allocate vid_enc_msg "
		" buffer\n");
		return;
	}

	venc_msg->venc_msg_info.statuscode = vid_enc_get_status(status);

	if (event == VCD_EVT_RESP_OUTPUT_DONE)
		venc_msg->venc_msg_info.msgcode =
		VEN_MSG_OUTPUT_BUFFER_DONE;

	else if (event == VCD_EVT_RESP_OUTPUT_FLUSHED)
		venc_msg->venc_msg_info.msgcode =
		VEN_MSG_OUTPUT_BUFFER_DONE;
	else {
		ERR("QVD: vid_enc_output_frame_done invalid cmd type \n");
		return;
	}

	kernel_vaddr =
		(unsigned long)vcd_frame_data->p_virtual;

	if (vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT,
		FALSE, &user_vaddr, &kernel_vaddr,
		&phy_addr, &pmem_fd, &file,
		&buffer_index)) {

		/* Buffer address in user space */
		venc_msg->venc_msg_info.buf.ptrbuffer =	(u8 *) user_vaddr;
		/* Buffer address in user space */
		venc_msg->venc_msg_info.buf.clientdata = (void *)
		vcd_frame_data->n_frm_clnt_data;
		/* Data length */
		venc_msg->venc_msg_info.buf.len =
			vcd_frame_data->n_data_len;
		venc_msg->venc_msg_info.buf.flags =
			vcd_frame_data->n_flags;
		/* Timestamp pass-through from input frame */
		venc_msg->venc_msg_info.buf.timestamp =
			vcd_frame_data->time_stamp;

		/* Decoded picture width and height */
		venc_msg->venc_msg_info.msgdata_size =
			sizeof(struct venc_buffer);
	} else {
		ERR("vid_enc_output_frame_done UVA can not be found\n");
		venc_msg->venc_msg_info.statuscode =
			VEN_S_EFATAL;
	}

	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&venc_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}

static void vid_enc_lean_event(struct video_client_ctx *client_ctx,
	u32 event, u32 status)
{
	struct vid_enc_msg *venc_msg;
	if (!client_ctx) {
		ERR("%s(): !client_ctx pointer \n",
			__func__);
		return;
	}

	venc_msg = kzalloc(sizeof(struct vid_enc_msg),
			GFP_KERNEL);
	if (!venc_msg) {
		ERR("%s(): cannot allocate vid_enc_msg buffer\n",
			__func__);
		return;
	}

	venc_msg->venc_msg_info.statuscode =
		vid_enc_get_status(status);

	switch (event) {
	case VCD_EVT_RESP_FLUSH_INPUT_DONE:
		INFO("\n msm_vidc_enc: Sending VCD_EVT_RESP_FLUSH_INPUT_DONE"
			 " to client");
		venc_msg->venc_msg_info.msgcode =
			VEN_MSG_FLUSH_INPUT_DONE;
		break;
	case VCD_EVT_RESP_FLUSH_OUTPUT_DONE:
		INFO("\n msm_vidc_enc: Sending VCD_EVT_RESP_FLUSH_OUTPUT_DONE"
			 " to client");
		venc_msg->venc_msg_info.msgcode =
			VEN_MSG_FLUSH_OUPUT_DONE;
		break;

	case VCD_EVT_RESP_START:
		INFO("\n msm_vidc_enc: Sending VCD_EVT_RESP_START"
			 " to client");
		venc_msg->venc_msg_info.msgcode =
			VEN_MSG_START;
		break;

	case VCD_EVT_RESP_STOP:
		INFO("\n msm_vidc_enc: Sending VCD_EVT_RESP_STOP"
			 " to client");
		venc_msg->venc_msg_info.msgcode =
			VEN_MSG_STOP;
		break;

	case VCD_EVT_RESP_PAUSE:
		INFO("\n msm_vidc_enc: Sending VCD_EVT_RESP_PAUSE"
			 " to client");
		venc_msg->venc_msg_info.msgcode =
			VEN_MSG_PAUSE;
		break;

	default:
		ERR("%s() : unknown event type \n",
			__func__);
		break;
	}

	venc_msg->venc_msg_info.msgdata_size = 0;

	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&venc_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}


void vid_enc_vcd_cb(u32 event, u32 status,
	void *info, u32 size, void *handle,
	void *const client_data)
{
	struct video_client_ctx *client_ctx =
		(struct video_client_ctx *)client_data;

	DBG("Entering %s()\n", __func__);

	if (!client_ctx) {
		ERR("%s(): client_ctx is NULL \n", __func__);
		return;
	}

	client_ctx->event_status = status;

	switch (event) {
	case VCD_EVT_RESP_OPEN:
		vid_enc_vcd_open_done(client_ctx,
		(struct vcd_handle_container_type *)info);
		break;

	case VCD_EVT_RESP_INPUT_DONE:
	case VCD_EVT_RESP_INPUT_FLUSHED:
		vid_enc_input_frame_done(client_ctx, event,
		status, (struct vcd_frame_data_type *)info);
		break;

	case VCD_EVT_RESP_OUTPUT_DONE:
	case VCD_EVT_RESP_OUTPUT_FLUSHED:
		vid_enc_output_frame_done(client_ctx, event, status,
		(struct vcd_frame_data_type *)info);
		break;

	case VCD_EVT_RESP_PAUSE:
	case VCD_EVT_RESP_START:
	case VCD_EVT_RESP_STOP:
	case VCD_EVT_RESP_FLUSH_INPUT_DONE:
	case VCD_EVT_RESP_FLUSH_OUTPUT_DONE:
	case VCD_EVT_IND_RECONFIG:
	case VCD_EVT_IND_HWERRFATAL:
	case VCD_EVT_IND_RESOURCES_LOST:
		vid_enc_lean_event(client_ctx, event, status);
		break;

	default:
		ERR("%s() :  Error - Invalid event type =%u\n",
		__func__, event);
		break;
	}
}

static u32 vid_enc_msg_pending(struct video_client_ctx *client_ctx)
{
	u32 islist_empty = 0;

	mutex_lock(&client_ctx->msg_queue_lock);
	islist_empty = list_empty(&client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);

	if (islist_empty) {
		DBG("%s(): vid_enc msg queue empty\n",
			__func__);
		if (client_ctx->stop_msg) {
			DBG("%s(): List empty and Stop Msg set\n",
				__func__);
			return client_ctx->stop_msg;
		}
	} else
		DBG("%s(): vid_enc msg queue Not empty\n",
			__func__);

	return !islist_empty;
}

static u32 vid_enc_get_next_msg(struct video_client_ctx *client_ctx,
		struct venc_msg *venc_msg_info)
{
	int rc;
	struct vid_enc_msg *vid_enc_msg = NULL;

	if (!client_ctx)
		return FALSE;

	rc = wait_event_interruptible(client_ctx->msg_wait,
		vid_enc_msg_pending(client_ctx));

	if (rc < 0 || client_ctx->stop_msg) {
		DBG("rc = %d, stop_msg = %u\n", rc, client_ctx->stop_msg);
		return FALSE;
	}

	mutex_lock(&client_ctx->msg_queue_lock);

	if (!list_empty(&client_ctx->msg_queue)) {
		DBG("%s(): After Wait \n", __func__);
		vid_enc_msg = list_first_entry(&client_ctx->msg_queue,
					struct vid_enc_msg, list);
		list_del(&vid_enc_msg->list);
		memcpy(venc_msg_info, &vid_enc_msg->venc_msg_info,
		sizeof(struct venc_msg));
		kfree(vid_enc_msg);
	}
	mutex_unlock(&client_ctx->msg_queue_lock);
	return TRUE;
}

static u32 vid_enc_close_client(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;

	int rc;

	INFO("\n msm_vidc_enc: Inside %s()", __func__);
	if (!client_ctx || (!client_ctx->vcd_handle)) {
		ERR("\n %s(): Invalid client_ctx", __func__);
		return FALSE;
	}

	mutex_lock(&vid_enc_device_p->lock);

	if (!stop_cmd) {
		vcd_status = vcd_stop(client_ctx->vcd_handle);
		DBG("Waiting for VCD_STOP: Before Timeout\n");
		if (!vcd_status) {
			rc = wait_for_completion_timeout(&client_ctx->event,
				5 * HZ);
			if (!rc) {
				ERR("%s:ERROR vcd_stop time out"
				"rc = %d\n", __func__, rc);
			}

			if (client_ctx->event_status) {
				ERR("%s:ERROR "
				"vcd_stop Not successs \n", __func__);
			}
		}
	}
	DBG("VCD_STOPPED: After Timeout, calling VCD_CLOSE\n");
	vcd_status = vcd_close(client_ctx->vcd_handle);

	if (vcd_status) {
		mutex_unlock(&vid_enc_device_p->lock);
		return FALSE;
	}

	memset((void *)client_ctx, 0,
		sizeof(struct video_client_ctx));

	vid_enc_device_p->num_clients--;
	stop_cmd = 0;
	mutex_unlock(&vid_enc_device_p->lock);
	return TRUE;
}


static int vid_enc_open(struct inode *inode, struct file *file)
{
	s32 client_index;
	struct video_client_ctx *client_ctx;
	u32 vcd_status = VCD_ERR_FAIL;

	INFO("\n msm_vidc_enc: Inside %s()", __func__);

	mutex_lock(&vid_enc_device_p->lock);

	stop_cmd = 0;
	if (vid_enc_device_p->num_clients == VID_ENC_MAX_ENCODER_CLIENTS) {
		ERR("ERROR : vid_enc_open() max number of clients"
		    "limit reached\n");
		mutex_unlock(&vid_enc_device_p->lock);
		return -ENODEV;
	}

#ifndef USE_RES_TRACKER
	DBG("Resource Tracker not in use");
	if (!vid_c_enable_clk(VID_C_HCLK_RATE)) {
		ERR("ERROR : vid_enc_open()	clock enabled failed\n");
		mutex_unlock(&vid_enc_device_p->lock);
		return -ENODEV;
	}
#endif

	DBG(" Virtual Address of ioremap is %p\n", vid_enc_device_p->virt_base);
	if (!vid_enc_device_p->num_clients) {
		if (!vid_c_load_firmware())
			return -ENODEV;
	}

	client_index = vid_enc_get_empty_client_index();

	if (client_index == -1) {
		ERR("%s() : No free clients client_index == -1\n",
			__func__);
		return -ENODEV;
	}

	client_ctx =
		&vid_enc_device_p->venc_clients[client_index];
	vid_enc_device_p->num_clients++;

	init_completion(&client_ctx->event);
	mutex_init(&client_ctx->msg_queue_lock);
	INIT_LIST_HEAD(&client_ctx->msg_queue);
	init_waitqueue_head(&client_ctx->msg_wait);
	vcd_status = vcd_open(vid_enc_device_p->device_handle, FALSE,
		vid_enc_vcd_cb, client_ctx);
	client_ctx->stop_msg = 0;

	wait_for_completion(&client_ctx->event);
	file->private_data = client_ctx;
	mutex_unlock(&vid_enc_device_p->lock);
	return 0;
}

static int vid_enc_release(struct inode *inode, struct file *file)
{
	struct video_client_ctx *client_ctx = file->private_data;
	INFO("\n msm_vidc_enc: Inside %s()", __func__);
	vid_enc_close_client(client_ctx);
	vid_c_release_firmware();
#ifndef USE_RES_TRACKER
	vid_c_disable_clk();
#endif
	INFO("\n msm_vidc_enc: Return from %s()", __func__);
	return 0;
}

static const struct file_operations vid_enc_fops = {
	.owner = THIS_MODULE,
	.open = vid_enc_open,
	.release = vid_enc_release,
	.ioctl = vid_enc_ioctl
};

void vid_enc_interrupt_deregister(void)
{
}

void vid_enc_interrupt_register(void *device_name)
{
}

void vid_enc_interrupt_clear(void)
{
}

void *vid_enc_map_dev_base_addr(void *device_name)
{
	return vid_enc_device_p->virt_base;
}

static int vid_enc_vcd_init(void)
{
	int rc;
	struct vcd_init_config_type vcd_init_config;
	u32 i;

	INFO("\n msm_vidc_enc: Inside %s()", __func__);
	vid_enc_device_p->num_clients = 0;

	for (i = 0; i < VID_ENC_MAX_ENCODER_CLIENTS; i++) {
		memset((void *)&vid_enc_device_p->venc_clients[i], 0,
		sizeof(vid_enc_device_p->venc_clients[i]));
	}

	mutex_init(&vid_enc_device_p->lock);
	vid_enc_device_p->virt_base = vid_c_get_ioaddr();

	if (!vid_enc_device_p->virt_base) {
		ERR("%s() : ioremap failed\n", __func__);
		return -ENOMEM;
	}

	vcd_init_config.p_device_name = "VID_C";
	vcd_init_config.pf_map_dev_base_addr =
		vid_enc_map_dev_base_addr;
	vcd_init_config.pf_interrupt_clr =
		vid_enc_interrupt_clear;
	vcd_init_config.pf_register_isr =
		vid_enc_interrupt_register;
	vcd_init_config.pf_deregister_isr =
		vid_enc_interrupt_deregister;

	rc = vcd_init(&vcd_init_config,
		&vid_enc_device_p->device_handle);

	if (rc) {
		ERR("%s() : vcd_init failed\n",
			__func__);
		return -ENODEV;
	}
	return 0;
}

static int __init vid_enc_init(void)
{
	int rc = 0;
	struct device *class_devp;

	INFO("\n msm_vidc_enc: Inside %s()", __func__);
	vid_enc_device_p = kzalloc(sizeof(struct vid_enc_dev),
					 GFP_KERNEL);
	if (!vid_enc_device_p) {
		ERR("%s Unable to allocate memory for vid_enc_dev\n",
			__func__);
		return -ENOMEM;
	}

	rc = alloc_chrdev_region(&vid_enc_dev_num, 0, 1, VID_ENC_NAME);
	if (rc < 0) {
		ERR("%s: alloc_chrdev_region Failed rc = %d\n",
			__func__, rc);
		goto error_vid_enc_alloc_chrdev_region;
	}

	vid_enc_class = class_create(THIS_MODULE, VID_ENC_NAME);
	if (IS_ERR(vid_enc_class)) {
		rc = PTR_ERR(vid_enc_class);
		ERR("%s: couldn't create vid_enc_class rc = %d\n",
			__func__, rc);
		goto error_vid_enc_class_create;
	}

	class_devp = device_create(vid_enc_class, NULL,
				vid_enc_dev_num, NULL, VID_ENC_NAME);

	if (IS_ERR(class_devp)) {
		rc = PTR_ERR(class_devp);
		ERR("%s: class device_create failed %d\n",
		__func__, rc);
		goto error_vid_enc_class_device_create;
	}

	vid_enc_device_p->device = class_devp;

	cdev_init(&vid_enc_device_p->cdev, &vid_enc_fops);
	vid_enc_device_p->cdev.owner = THIS_MODULE;
	rc = cdev_add(&(vid_enc_device_p->cdev), vid_enc_dev_num, 1);

	if (rc < 0) {
		ERR("%s: cdev_add failed %d\n",
		__func__, rc);
		goto error_vid_enc_cdev_add;
	}
	vid_enc_vcd_init();
	return 0;

error_vid_enc_cdev_add:
	device_destroy(vid_enc_class, vid_enc_dev_num);
error_vid_enc_class_device_create:
	class_destroy(vid_enc_class);
error_vid_enc_class_create:
	unregister_chrdev_region(vid_enc_dev_num, 1);
error_vid_enc_alloc_chrdev_region:
	kfree(vid_enc_device_p);

	return rc;
}

static void __exit vid_enc_exit(void)
{
	INFO("\n msm_vidc_enc: Inside %s()", __func__);
	cdev_del(&(vid_enc_device_p->cdev));
	device_destroy(vid_enc_class, vid_enc_dev_num);
	class_destroy(vid_enc_class);
	unregister_chrdev_region(vid_enc_dev_num, 1);
	kfree(vid_enc_device_p);
	INFO("\n msm_vidc_enc: Return from %s()", __func__);
}
static int vid_enc_ioctl(struct inode *inode, struct file *file,
		unsigned cmd, unsigned long arg)
{
	struct video_client_ctx *client_ctx = NULL;
	struct venc_ioctl_msg venc_msg;
	struct venc_basecfg base_config;
	struct venc_switch encoder_switch;
	struct venc_profile profile;
	struct ven_profilelevel profile_level;
	struct venc_sessionqp session_qp;
	struct venc_intraperiod intraperiod;
	struct venc_seqheader seq_header;
	struct venc_seqheader seq_header_user;
	struct venc_entropycfg entropy_cfg;
	struct venc_dbcfg dbcfg;
	struct venc_intrarefresh intrarefresh;
	struct venc_multiclicecfg multiclicecfg;
	struct venc_ratectrlcfg ratectrlcfg;
	struct venc_voptimingcfg voptimingcfg;
	struct venc_framerate framerate;
	struct venc_targetbitrate targetbitrate;
	struct venc_headerextension headerextension;
	struct venc_qprange qprange;
	struct venc_bufferflush bufferflush;
	struct venc_allocatorproperty allocatorproperty;
	struct venc_bufferpayload buffer_info;
	enum venc_buffer_dir buffer_dir;
	struct venc_buffer enc_buffer;
	struct venc_msg cb_msg;
	u32 result = TRUE;

	DBG("%s\n", __func__);

	client_ctx = (struct video_client_ctx *)file->private_data;
	if (!client_ctx) {
		ERR("!client_ctx. Cannot attach to device handle\n");
		return -ENODEV;
	}

	switch (cmd) {
	case VEN_IOCTL_CMD_READ_NEXT_MSG:
		if (copy_from_user(&venc_msg, (void __user *)arg,
						sizeof(venc_msg)))
			return -EFAULT;
		DBG("VEN_IOCTL_CMD_READ_NEXT_MSG\n");
		result = vid_enc_get_next_msg(client_ctx, &cb_msg);
		if (!result) {
			ERR("VEN_IOCTL_CMD_READ_NEXT_MSG failed\n");
			return -EIO;
		} else
			if (copy_to_user((void __user *) \
				venc_msg.outputparam, &cb_msg,
				sizeof(cb_msg)))
				return -EFAULT;
		break;

	case VEN_IOCTL_CMD_STOP_READ_MSG:
		DBG("VEN_IOCTL_CMD_STOP_READ_MSG\n");
		client_ctx->stop_msg = 1;
		wake_up(&client_ctx->msg_wait);
		break;

	case VEN_IOCTL_CMD_ENCODE_FRAME:
	case VEN_IOCTL_CMD_FILL_OUTPUT_BUFFER:
		if (copy_from_user(&venc_msg, (void __user *)arg,
						sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_CMD_ENCODE_FRAME"
			"/VEN_IOCTL_CMD_FILL_OUTPUT_BUFFER\n");

		if (copy_from_user(&enc_buffer,
		(void __user *)venc_msg.inputparam, sizeof(enc_buffer)))
			return -EFAULT;

		if (cmd == VEN_IOCTL_CMD_ENCODE_FRAME)
			result = vid_enc_encode_frame(client_ctx,
					&enc_buffer);
		else
			result = vid_enc_fill_output_buffer(client_ctx,
					&enc_buffer);

		if (!result) {
			DBG(" \n VEN_IOCTL_CMD_ENCODE_FRAME/"
				"VEN_IOCTL_CMD_FILL_OUTPUT_BUFFER failed");
			return -EIO;
		}
		break;

	case VEN_IOCTL_SET_INPUT_BUFFER:
	case VEN_IOCTL_SET_OUTPUT_BUFFER:

		if (copy_from_user(&venc_msg,
			(void __user *)arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INPUT_BUFFER/VEN_IOCTL_SET_OUTPUT_BUFFER\n");

		if (copy_from_user(&buffer_info,
			(void __user *)venc_msg.inputparam,
			sizeof(buffer_info)))
			return -EFAULT;

		buffer_dir = VEN_BUFFER_TYPE_INPUT;
		if (cmd == VEN_IOCTL_SET_OUTPUT_BUFFER)
			buffer_dir = VEN_BUFFER_TYPE_OUTPUT;

		result = vid_enc_set_buffer(client_ctx, &buffer_info,
				buffer_dir);
		if (!result) {
			DBG("\n VEN_IOCTL_SET_INPUT_BUFFER"
				"/VEN_IOCTL_SET_OUTPUT_BUFFER failed");
			return -EIO;
		}
		break;
	case VEN_IOCTL_SET_INPUT_BUFFER_REQ:
	case VEN_IOCTL_SET_OUTPUT_BUFFER_REQ:
		if (copy_from_user(&venc_msg,
			(void __user *)arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INPUT_BUFFER_REQ"
			"/VEN_IOCTL_SET_OUTPUT_BUFFER_REQ\n");

		if (copy_from_user(&allocatorproperty,
			(void __user *)venc_msg.inputparam,
			sizeof(allocatorproperty)))
			return -EFAULT;

	if (cmd == VEN_IOCTL_SET_OUTPUT_BUFFER_REQ)
			result = vid_enc_set_buffer_req(client_ctx,
					&allocatorproperty, FALSE);
		else
			result = vid_enc_set_buffer_req(client_ctx,
					&allocatorproperty, TRUE);
		if (!result) {
			DBG("setting VEN_IOCTL_SET_OUTPUT_BUFFER_REQ/"
			"VEN_IOCTL_SET_INPUT_BUFFER_REQ failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_INPUT_BUFFER_REQ:
	case VEN_IOCTL_GET_OUTPUT_BUFFER_REQ:
		if (copy_from_user(&venc_msg, (void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_INPUT_BUFFER_REQ/"
			"VEN_IOCTL_GET_OUTPUT_BUFFER_REQ \n");

		if (cmd == VEN_IOCTL_GET_OUTPUT_BUFFER_REQ)
			result = vid_enc_get_buffer_req(client_ctx,
					&allocatorproperty, FALSE);
		else
			result = vid_enc_get_buffer_req(client_ctx,
					&allocatorproperty, TRUE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&allocatorproperty,
				sizeof(allocatorproperty)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_CMD_FLUSH:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_CMD_FLUSH\n");

		if (copy_from_user(&bufferflush,
			(void __user *)venc_msg.inputparam,
			sizeof(bufferflush)))
			return -EFAULT;

		INFO("\n %s(): Calling vid_enc_flush with mode = %lu",
			 __func__, bufferflush.flush_mode);
		result = vid_enc_flush(client_ctx, &bufferflush);

		if (!result) {
			ERR("setting VEN_IOCTL_CMD_FLUSH failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_CMD_START:
		INFO("\n %s(): Executing VEN_IOCTL_CMD_START", __func__);
		result = vid_enc_start_stop(client_ctx, TRUE);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_START failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_CMD_STOP:
		INFO("\n %s(): Executing VEN_IOCTL_CMD_STOP", __func__);
		result = vid_enc_start_stop(client_ctx, FALSE);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_STOP failed\n");
			return -EIO;
		}
		stop_cmd = 1;
		break;

	case VEN_IOCTL_CMD_PAUSE:
		INFO("\n %s(): Executing VEN_IOCTL_CMD_PAUSE", __func__);
		result = vid_enc_pause_resume(client_ctx, TRUE);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_PAUSE failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_CMD_RESUME:
		INFO("\n %s(): Executing VEN_IOCTL_CMD_RESUME", __func__);
		result = vid_enc_pause_resume(client_ctx, FALSE);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_RESUME failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_SET_QP_RANGE:

		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_QP_RANGE\n");

		if (copy_from_user(&qprange,
			(void __user *)venc_msg.inputparam,
			sizeof(qprange)))
			return -EFAULT;

		result = vid_enc_set_get_qprange(client_ctx,
				&qprange, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_QP_RANGE failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_QP_RANGE:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_QP_RANGE\n");
		result = vid_enc_set_get_qprange(client_ctx,
				&qprange, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&qprange,
				sizeof(qprange)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_HEC:
		if (copy_from_user(&venc_msg, (void __user *)arg,
						sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_HEC\n");

		if (copy_from_user(&headerextension,
			(void __user *)venc_msg.inputparam,
			sizeof(headerextension)))
			return -EFAULT;

		result = vid_enc_set_get_headerextension(client_ctx,
				&headerextension, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_HEC failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_HEC:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_HEC\n");
		result = vid_enc_set_get_headerextension(client_ctx,
				&headerextension, FALSE);

		if (result) {
			if (copy_to_user(
			(void __user *)venc_msg.outputparam,
			&headerextension,
			sizeof(headerextension)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_TARGET_BITRATE:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_TARGET_BITRATE\n");

		if (copy_from_user(&targetbitrate,
			(void __user *)venc_msg.inputparam,
			sizeof(targetbitrate)))
			return -EFAULT;

		result = vid_enc_set_get_bitrate(client_ctx,
				&targetbitrate, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_TARGET_BITRATE failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_TARGET_BITRATE:
		if (copy_from_user(&venc_msg,
				(void __user *)arg,
				sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_TARGET_BITRATE\n");
		result = vid_enc_set_get_bitrate(client_ctx,
				&targetbitrate, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&targetbitrate,
				sizeof(targetbitrate)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_FRAME_RATE:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_FRAME_RATE\n");

		if (copy_from_user(&framerate,
			(void __user *)venc_msg.inputparam,
			sizeof(framerate)))
			return -EFAULT;

		result = vid_enc_set_get_framerate(client_ctx,
				&framerate, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_FRAME_RATE failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_FRAME_RATE:
		if (copy_from_user(&venc_msg,
				(void __user *)arg,
				sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_FRAME_RATE\n");
		result = vid_enc_set_get_framerate(client_ctx, &framerate,
				FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&framerate,
				sizeof(framerate)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_VOP_TIMING_CFG:

		if (copy_from_user(&venc_msg,
						(void __user *)arg,
						sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_VOP_TIMING_CFG\n");

		if (copy_from_user(
			&voptimingcfg, (void __user *)venc_msg.inputparam,
			sizeof(voptimingcfg)))
			return -EFAULT;

		result = vid_enc_set_get_voptimingcfg(client_ctx,
				&voptimingcfg, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_VOP_TIMING_CFG failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_VOP_TIMING_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_VOP_TIMING_CFG\n");
		result = vid_enc_set_get_voptimingcfg(client_ctx,
				&voptimingcfg, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&voptimingcfg,
				sizeof(voptimingcfg)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_RATE_CTRL_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_RATE_CTRL_CFG\n");

		if (copy_from_user(&ratectrlcfg,
			(void __user *)venc_msg.inputparam,
			sizeof(ratectrlcfg)))
			return -EFAULT;

		result = vid_enc_set_get_ratectrlcfg(client_ctx,
				&ratectrlcfg, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_RATE_CTRL_CFG failed\n");
			return -EIO;
		}

		break;

	case VEN_IOCTL_GET_RATE_CTRL_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_RATE_CTRL_CFG\n");
		result = vid_enc_set_get_ratectrlcfg(client_ctx,
				&ratectrlcfg, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&ratectrlcfg,
				sizeof(ratectrlcfg)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_MULTI_SLICE_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_MULTI_SLICE_CFG\n");

		if (copy_from_user(&multiclicecfg,
			(void __user *)venc_msg.inputparam,
			sizeof(multiclicecfg)))
			return -EFAULT;

		result = vid_enc_set_get_multiclicecfg(client_ctx,
				&multiclicecfg, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_MULTI_SLICE_CFG failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_MULTI_SLICE_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_MULTI_SLICE_CFG\n");
		result = vid_enc_set_get_multiclicecfg(client_ctx,
				&multiclicecfg, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&multiclicecfg,
				sizeof(multiclicecfg)))
				return -EFAULT;
		} else
				return -EIO;
		break;
	case VEN_IOCTL_SET_INTRA_REFRESH:

		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INTRA_REFRESH\n");

		if (copy_from_user(&intrarefresh,
			(void __user *)venc_msg.inputparam,
			sizeof(intrarefresh)))
			return -EFAULT;

		result = vid_enc_set_get_intrarefresh(client_ctx,
				&intrarefresh, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_INTRA_REFRESH failed\n");
			return -EIO;
		}

		break;

	case VEN_IOCTL_GET_INTRA_REFRESH:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_DEBLOCKING_CFG\n");
		result = vid_enc_set_get_intrarefresh(client_ctx,
				&intrarefresh, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&intrarefresh,
				sizeof(intrarefresh)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_DEBLOCKING_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_DEBLOCKING_CFG\n");

		if (copy_from_user(&dbcfg,
			(void __user *)venc_msg.inputparam,
			sizeof(dbcfg)))
			return -EFAULT;

		result = vid_enc_set_get_dbcfg(client_ctx,
				&dbcfg, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_DEBLOCKING_CFG failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_DEBLOCKING_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_DEBLOCKING_CFG\n");
		result = vid_enc_set_get_dbcfg(client_ctx,
				&dbcfg, FALSE);

		if (result) {
			if (copy_to_user(
			(void __user *)venc_msg.outputparam,
			&dbcfg,
			sizeof(dbcfg)))
				return -EFAULT;
		} else
				return -EIO;

		break;
	case VEN_IOCTL_SET_ENTROPY_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_ENTROPY_CFG\n");

		if (copy_from_user(&entropy_cfg,
			(void __user *)venc_msg.inputparam,
			sizeof(entropy_cfg)))
			return -EFAULT;

		result = vid_enc_set_get_entropy_cfg(client_ctx,
				&entropy_cfg, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_ENTROPY_CFG failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_ENTROPY_CFG:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_ENTROPY_CFG\n");

		result = vid_enc_set_get_entropy_cfg(client_ctx,
				&entropy_cfg, FALSE);

		if (result) {
			if (copy_to_user(
			(void __user *)venc_msg.outputparam,
			&entropy_cfg,
			sizeof(entropy_cfg)))
				return -EFAULT;
		} else
			return -EIO;

		break;

	case VEN_IOCTL_GET_SEQUENCE_HDR:

		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_SEQUENCE_HDR\n");

		if (copy_from_user(&seq_header,
			(void __user *)venc_msg.inputparam,
			sizeof(seq_header)))
			return -EFAULT;

		if (copy_from_user(&seq_header_user,
			(void __user *)venc_msg.inputparam,
			sizeof(seq_header_user)))
			return -EFAULT;

		seq_header.hdrbufptr = NULL;
		result = vid_enc_get_sequence_header(client_ctx,
				&seq_header);

		if (result) {
			if ((copy_to_user(
				(void __user *)seq_header_user.hdrbufptr,
				seq_header.hdrbufptr, seq_header.hdrlen)) ||
				(copy_to_user(
				(void __user *)&seq_header_user.hdrlen,
				&seq_header.hdrlen,
				sizeof(seq_header.hdrlen)))
				) {
				kfree(seq_header.hdrbufptr);
				seq_header.hdrbufptr = NULL;
				return -EFAULT;
			}
		} else {
				kfree(seq_header.hdrbufptr);
				seq_header.hdrbufptr = NULL;
				return -EIO;
		}

		kfree(seq_header.hdrbufptr);
		seq_header.hdrbufptr = NULL;

		break;

	case VEN_IOCTL_GET_CAPABILITY:
		return -EIO;
		break;
	case VEN_IOCTL_CMD_REQUEST_IFRAME:
		result = vid_enc_request_iframe(client_ctx);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_REQUEST_IFRAME failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_SET_INTRA_PERIOD:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INTRA_PERIOD\n");

		if (copy_from_user(&intraperiod,
			(void __user *)venc_msg.inputparam,
			sizeof(intraperiod)))
			return -EFAULT;

		result = vid_enc_set_get_intraperiod(client_ctx,
				&intraperiod, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_INTRA_PERIOD failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_INTRA_PERIOD:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_SESSION_QP\n");

		result = vid_enc_set_get_intraperiod(client_ctx,
				&intraperiod, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&intraperiod,
				sizeof(intraperiod)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_SESSION_QP:
		if (copy_from_user(
			&venc_msg, (void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_SESSION_QP\n");

		if (copy_from_user(&session_qp,
			(void __user *)venc_msg.inputparam,
			sizeof(session_qp)))
			return -EFAULT;

		result = vid_enc_set_get_session_qp(client_ctx,
				&session_qp, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_SESSION_QP failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_SESSION_QP:
		if (copy_from_user(
			&venc_msg, (void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_SESSION_QP\n");

		result = vid_enc_set_get_session_qp(client_ctx,
				&session_qp, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&session_qp,
				sizeof(session_qp)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_PROFILE_LEVEL:

		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_PROFILE_LEVEL\n");

		if (copy_from_user(&profile_level,
			(void __user *)venc_msg.inputparam,
			sizeof(profile_level)))
			return -EFAULT;

		result = vid_enc_set_get_profile_level(client_ctx,
				&profile_level, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_PROFILE_LEVEL failed\n");
			return -EIO;
		}

		break;
	case VEN_IOCTL_GET_PROFILE_LEVEL:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_CODEC_PROFILE\n");

		result = vid_enc_set_get_profile_level(client_ctx,
				&profile_level, FALSE);

		if (result) {
			if (copy_to_user(
			(void __user *)venc_msg.outputparam,
			&profile_level,
			sizeof(profile_level)))
				return -EFAULT;
		} else
				return -EIO;
		break;

	case VEN_IOCTL_SET_CODEC_PROFILE:
		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_CODEC_PROFILE\n");

		if (copy_from_user(&profile,
			(void __user *)venc_msg.inputparam,
			sizeof(profile)))
			return -EFAULT;

		result = vid_enc_set_get_profile(client_ctx,
				&profile, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_CODEC_PROFILE failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_CODEC_PROFILE:
		if (copy_from_user(
			&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_CODEC_PROFILE\n");

		result = vid_enc_set_get_profile(client_ctx,
				&profile, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&profile,
				sizeof(profile)))
				return -EFAULT;
		} else
				return -EIO;
		break;


	case VEN_IOCTL_SET_SHORT_HDR:
		if (copy_from_user(&venc_msg,
			(void __user *)arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("Getting VEN_IOCTL_SET_SHORT_HDR\n");

		if (copy_from_user(
			&encoder_switch,
			(void __user *)venc_msg.inputparam,
			sizeof(encoder_switch)))
			return -EFAULT;

		result = vid_enc_set_get_short_header(client_ctx,
				&encoder_switch, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_SHORT_HDR failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_SHORT_HDR:
		if (copy_from_user(&venc_msg,
			(void __user *)arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_LIVE_MODE\n");

		result = vid_enc_set_get_short_header(client_ctx,
				&encoder_switch, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&encoder_switch,
				sizeof(encoder_switch)))
				return -EFAULT;
		} else
				return -EIO;

		break;

	case VEN_IOCTL_SET_BASE_CFG:

		DBG("VEN_IOCTL_SET_BASE_CFG\n");

		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		if (copy_from_user(&base_config,
			(void __user *)venc_msg.inputparam,
			sizeof(base_config)))
			return -EFAULT;

		DBG("setting VEN_IOCTL_SET_BASE_CFG\n");

		result = vid_enc_set_get_base_cfg(client_ctx,
				&base_config, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_BASE_CFG failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_BASE_CFG:
		DBG("VEN_IOCTL_GET_BASE_CFG\n");

		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("Getting VEN_IOCTL_SET_BASE_CFG\n");

		result = vid_enc_set_get_base_cfg(client_ctx,
				&base_config, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&base_config,
				sizeof(base_config)))
				return -EFAULT;
		} else
			return -EIO;

		break;

	case VEN_IOCTL_SET_LIVE_MODE:

		if (copy_from_user(&venc_msg,
			(void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("Getting VEN_IOCTL_SET_LIVE_MODE\n");

		if (copy_from_user(&encoder_switch,
			(void __user *)venc_msg.inputparam,
			sizeof(encoder_switch)))
			return -EFAULT;

		result = vid_enc_set_get_live_mode(client_ctx,
				&encoder_switch, TRUE);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_LIVE_MODE failed\n");
			return -EIO;
		}
		break;

	case VEN_IOCTL_GET_LIVE_MODE:

		if (copy_from_user(
			&venc_msg, (void __user *)arg,
			sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_LIVE_MODE\n");

		result = vid_enc_set_get_live_mode(client_ctx,
				&encoder_switch, FALSE);

		if (result) {
			if (copy_to_user(
				(void __user *)venc_msg.outputparam,
				&encoder_switch,
				sizeof(encoder_switch)))
				return -EFAULT;
		} else
			return -EIO;

		break;
	case VEN_IOCTL_SET_AC_PREDICTION:
	case VEN_IOCTL_GET_AC_PREDICTION:
	case VEN_IOCTL_SET_RVLC:
	case VEN_IOCTL_GET_RVLC:
	case VEN_IOCTL_SET_ROTATION:
	case VEN_IOCTL_GET_ROTATION:
	case VEN_IOCTL_SET_DATA_PARTITION:
	case VEN_IOCTL_GET_DATA_PARTITION:
	default:
		ERR("%s(): Unsupported ioctl %d\n", __func__, cmd);
		return -ENOTTY;

		break;
	}
	return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Video encoder driver");
MODULE_VERSION("1.0");

module_init(vid_enc_init);
module_exit(vid_enc_exit);

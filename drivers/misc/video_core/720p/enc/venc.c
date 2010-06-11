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

#include "vcd_ddl_firmware.h"
#include "video_core_type.h"
#include "vcd_api.h"
#include "venc_internal.h"
#include "video_core_init.h"

#define VID_ENC_NAME "msm_vidc_enc"
#define VID_C_HCLK_RATE	170667000

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define INFO(x...) printk(KERN_INFO x)
#define ERR(x...) printk(KERN_ERR x)

static struct vid_enc_dev *vidc_enc_dev;
static dev_t vidc_enc_dev_num;
static struct class *vid_enc_class;
static int vid_enc_ioctl(struct inode *inode, struct file *file, unsigned cmd,
	unsigned long arg);
//TOOD stop_cmd wtf
static int stop_cmd;

static s32 vid_enc_get_empty_client_index(void)
{
	u32 i;
	u32 found = false;

	for (i = 0; i < VID_ENC_MAX_ENCODER_CLIENTS; i++) {
		if (!vidc_enc_dev->venc_clients[i].vcd_handle) {
			found = true;
			break;
		}
	}
	if (!found) {
		ERR("%s: ERROR No space for new client\n", __func__);
		return -1;
	}
	DBG("%s: available client index = %u\n", __func__, i);
	return i;
}

//TODO collapse this crap
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
	struct vcd_handle_container *handle_container)
{
	DBG("vid_enc_vcd_open_done\n");

	if (!client_ctx) {
		ERR("%s(): ERROR. client_ctx is NULL\n", __func__);
		return;
	}
	if (handle_container)
		client_ctx->vcd_handle = handle_container->handle;
	else
		ERR("%s: ERROR. handle_container is NULL\n", __func__);
	vid_enc_notify_client(client_ctx);
}

static void vid_enc_input_frame_done(struct video_client_ctx *client_ctx,
	u32 event, u32 status, struct vcd_frame_data *vcd_frame_data)
{
	struct vid_enc_msg *venc_msg;

	if (!client_ctx || !vcd_frame_data) {
		ERR("%s: NULL pointer\n", __func__);
		return;
	}

	venc_msg = kzalloc(sizeof(struct vid_enc_msg), GFP_KERNEL);
	if (!venc_msg) {
		ERR("%s: cannot allocate vid_enc_msg buffer\n", __func__);
		return;
	}

	venc_msg->venc_msg_info.statuscode = vid_enc_get_status(status);

	if (event == VCD_EVT_RESP_INPUT_DONE) {
		venc_msg->venc_msg_info.msgcode = VEN_MSG_INPUT_BUFFER_DONE;
		DBG("Send INPUT_DON message to client = %p\n", client_ctx);
	} else if (event == VCD_EVT_RESP_INPUT_FLUSHED) {
		venc_msg->venc_msg_info.msgcode = VEN_MSG_INPUT_BUFFER_DONE;
		DBG("Send INPUT_FLUSHED message to client = %p\n", client_ctx);
	} else {
		ERR("vid_enc_input_frame_done(): invalid event type\n");
		return;
	}

	venc_msg->venc_msg_info.buf.clientdata = vcd_frame_data->client_data;
	venc_msg->venc_msg_info.msgdata_size = sizeof(struct vid_enc_msg);

	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&venc_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}

static void vid_enc_output_frame_done(struct video_client_ctx *client_ctx,
	u32 event, u32 status, struct vcd_frame_data *frm_data)
{
	struct vid_enc_msg *venc_msg;
	void __user *user_addr;
	void *kern_addr;
	phys_addr_t phys_addr;
	int pmem_fd;
	struct file *file;
	s32 buf_index = -1;

	if (!client_ctx || !frm_data) {
		ERR("%s: NULL pointer\n", __func__);
		return;
	}

	venc_msg = kzalloc(sizeof(struct vid_enc_msg), GFP_KERNEL);
	if (!venc_msg) {
		ERR("%s: cannot allocate vid_enc_msg buffer\n", __func__);
		return;
	}

	venc_msg->venc_msg_info.statuscode = vid_enc_get_status(status);

	if (event == VCD_EVT_RESP_OUTPUT_DONE) {
		venc_msg->venc_msg_info.msgcode = VEN_MSG_OUTPUT_BUFFER_DONE;
	} else if (event == VCD_EVT_RESP_OUTPUT_FLUSHED) {
		venc_msg->venc_msg_info.msgcode = VEN_MSG_OUTPUT_BUFFER_DONE;
	} else {
		ERR("QVD: vid_enc_output_frame_done invalid cmd type\n");
		return;
	}

	kern_addr = frm_data->virt_addr;

	if (!vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT, false,
			&user_addr, &kern_addr, &phys_addr, &pmem_fd, &file,
			&buf_index)) {
		ERR("vid_enc_output_frame_done UVA can not be found\n");
		venc_msg->venc_msg_info.statuscode = VEN_S_EFATAL;
		goto out;
	}


	/* Buffer address in user space */
	venc_msg->venc_msg_info.buf.addr = user_addr;
	venc_msg->venc_msg_info.buf.clientdata = frm_data->client_data;
	/* Data length */
	venc_msg->venc_msg_info.buf.len = frm_data->data_len;
	venc_msg->venc_msg_info.buf.flags = frm_data->flags;
	/* time-stamp pass-through from input frame */
	venc_msg->venc_msg_info.buf.timestamp =	frm_data->time_stamp;
	/* Decoded picture width and height */
	venc_msg->venc_msg_info.msgdata_size = sizeof(struct venc_buffer);

out:
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
		ERR("%s(): !client_ctx pointer\n", __func__);
		return;
	}

	venc_msg = kzalloc(sizeof(struct vid_enc_msg), GFP_KERNEL);
	if (!venc_msg) {
		ERR("%s(): cannot allocate vid_enc_msg buffer\n", __func__);
		return;
	}

	venc_msg->venc_msg_info.statuscode = vid_enc_get_status(status);

	switch (event) {
	case VCD_EVT_RESP_FLUSH_INPUT_DONE:
		INFO("%s: Sending VCD_EVT_RESP_FLUSH_INPUT_DONE to client\n",
			__func__);
		venc_msg->venc_msg_info.msgcode = VEN_MSG_FLUSH_INPUT_DONE;
		break;
	case VCD_EVT_RESP_FLUSH_OUTPUT_DONE:
		INFO("%s: Sending VCD_EVT_RESP_FLUSH_OUTPUT_DONE to client\n",
			__func__);
		venc_msg->venc_msg_info.msgcode = VEN_MSG_FLUSH_OUPUT_DONE;
		break;
	case VCD_EVT_RESP_START:
		INFO("%s: Sending VCD_EVT_RESP_START to client\n", __func__);
		venc_msg->venc_msg_info.msgcode = VEN_MSG_START;
		break;
	case VCD_EVT_RESP_STOP:
		INFO("%s: Sending VCD_EVT_RESP_STOP to client\n", __func__);
		venc_msg->venc_msg_info.msgcode = VEN_MSG_STOP;
		break;
	case VCD_EVT_RESP_PAUSE:
		INFO("%s: Sending VCD_EVT_RESP_PAUSE to client\n", __func__);
		venc_msg->venc_msg_info.msgcode = VEN_MSG_PAUSE;
		break;
	default:
		ERR("%s: unknown event type\n", __func__);
		break;
	}

	venc_msg->venc_msg_info.msgdata_size = 0;

	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&venc_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}


void vid_enc_vcd_cb(u32 event, u32 status, void *info, u32 size, void *handle,
	void *const client_data)
{
	struct video_client_ctx *client_ctx = client_data;

	DBG("Entering %s\n", __func__);

	if (!client_ctx) {
		ERR("%s: client_ctx is NULL\n", __func__);
		return;
	}

	client_ctx->event_status = status;

	switch (event) {
	case VCD_EVT_RESP_OPEN:
		vid_enc_vcd_open_done(client_ctx, info);
		break;
	case VCD_EVT_RESP_INPUT_DONE:
	case VCD_EVT_RESP_INPUT_FLUSHED:
		vid_enc_input_frame_done(client_ctx, event, status, info);
		break;
	case VCD_EVT_RESP_OUTPUT_DONE:
	case VCD_EVT_RESP_OUTPUT_FLUSHED:
		vid_enc_output_frame_done(client_ctx, event, status, info);
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
		ERR("%s: Error invalid event type %u\n", __func__, event);
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
		DBG("%s: vid_enc msg queue empty\n", __func__);
		if (client_ctx->stop_msg) {
			DBG("%s: List empty and Stop Msg set\n", __func__);
			return client_ctx->stop_msg;
		}
	} else
		DBG("%s: vid_enc msg queue Not empty\n", __func__);

	return !islist_empty;
}

static u32 vid_enc_get_next_msg(struct video_client_ctx *client_ctx,
	struct venc_msg *venc_msg_info)
{
	int rc;
	struct vid_enc_msg *vid_enc_msg = NULL;

	if (!client_ctx)
		return false;

	rc = wait_event_interruptible(client_ctx->msg_wait,
		vid_enc_msg_pending(client_ctx));

	if (rc < 0 || client_ctx->stop_msg) {
		DBG("rc = %d, stop_msg = %u\n", rc, client_ctx->stop_msg);
		return false;
	}

	mutex_lock(&client_ctx->msg_queue_lock);

	if (!list_empty(&client_ctx->msg_queue)) {
		DBG("%s: After Wait\n", __func__);
		vid_enc_msg = list_first_entry(&client_ctx->msg_queue,
			struct vid_enc_msg, list);
		list_del(&vid_enc_msg->list);
		memcpy(venc_msg_info, &vid_enc_msg->venc_msg_info,
			sizeof(struct venc_msg));
		kfree(vid_enc_msg);
	}
	mutex_unlock(&client_ctx->msg_queue_lock);
	return true;
}

static u32 vid_enc_close_client(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;

	int rc;

	INFO("msm_vidc_enc: Inside %s\n", __func__);
	if (!client_ctx || !client_ctx->vcd_handle) {
		ERR("%s: Invalid client_ctx\n", __func__);
		return false;
	}

	mutex_lock(&vidc_enc_dev->lock);

	if (!stop_cmd) {
		vcd_status = vcd_stop(client_ctx->vcd_handle);
		DBG("Waiting for VCD_STOP: Before Timeout\n");
		if (!vcd_status) {
			rc = wait_for_completion_timeout(&client_ctx->event,
				5 * HZ);
			if (!rc) {
				ERR("%s: ERROR vcd_stop time out %d\n",
					__func__, rc);
			}

			if (client_ctx->event_status) {
				ERR("%s :ERROR vcd_stop Not success\n",
					__func__);
			}
		}
	}
	DBG("VCD_STOPPED: After Timeout, calling VCD_CLOSE\n");
	vcd_status = vcd_close(client_ctx->vcd_handle);

	if (vcd_status) {
		mutex_unlock(&vidc_enc_dev->lock);
		return false;
	}

	memset((void *)client_ctx, 0, sizeof(struct video_client_ctx));

	vidc_enc_dev->num_clients--;
	stop_cmd = 0;
	mutex_unlock(&vidc_enc_dev->lock);
	return true;
}


static int vid_enc_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	s32 client_index;
	struct video_client_ctx *client_ctx;
	u32 vcd_status = VCD_ERR_FAIL;

	INFO("msm_vidc_enc: Inside %s\n", __func__);

	mutex_lock(&vidc_enc_dev->lock);

	stop_cmd = 0;
	if (vidc_enc_dev->num_clients == VID_ENC_MAX_ENCODER_CLIENTS) {
		ERR("ERROR: vid_enc_open() max number of clients limit reached"
			"\n");
		rc = -ENODEV;
		goto out;
	}

#ifndef USE_RES_TRACKER
	DBG("Resource Tracker not in use");
	if (!vid_c_enable_clk(VID_C_HCLK_RATE)) {
		ERR("ERROR: vid_enc_open() clock enabled failed\n");
		rc = -ENODEV;
		goto out;
	}
#endif

	DBG("Virtual Address of ioremap is %p\n", vidc_enc_dev->virt_base);

	if (!vidc_enc_dev->num_clients) {
		rc = vcd_fw_prepare_all();
		if (rc)
			goto out;
	}

	client_index = vid_enc_get_empty_client_index();

	if (client_index == -1) {
		ERR("%s: No free clients client_index == -1\n", __func__);
		rc = -ENODEV;
		goto out;
	}

	client_ctx = &vidc_enc_dev->venc_clients[client_index];
	vidc_enc_dev->num_clients++;

	init_completion(&client_ctx->event);
	mutex_init(&client_ctx->msg_queue_lock);
	INIT_LIST_HEAD(&client_ctx->msg_queue);
	init_waitqueue_head(&client_ctx->msg_wait);
	vcd_status = vcd_open(vidc_enc_dev->device_handle, false,
		vid_enc_vcd_cb, client_ctx);
	client_ctx->stop_msg = 0;

	wait_for_completion(&client_ctx->event);
	file->private_data = client_ctx;

out:
	mutex_unlock(&vidc_enc_dev->lock);
	return rc;
}

static int vid_enc_release(struct inode *inode, struct file *file)
{
	struct video_client_ctx *client_ctx = file->private_data;
	INFO("msm_vidc_enc: Inside %s\n", __func__);
	vid_enc_close_client(client_ctx);
#ifndef USE_RES_TRACKER
	vid_c_disable_clk();
#endif
	INFO("msm_vidc_enc: Return from %s\n", __func__);
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
	return vidc_enc_dev->virt_base;
}

static int vid_enc_vcd_init(void)
{
	int rc;
	struct vcd_init_config vcd_init_config;
	u32 i;

	INFO("msm_vidc_enc: Inside %s\n", __func__);
	vidc_enc_dev->num_clients = 0;

	for (i = 0; i < VID_ENC_MAX_ENCODER_CLIENTS; i++)
		memset((void *)&vidc_enc_dev->venc_clients[i], 0,
			sizeof(vidc_enc_dev->venc_clients[i]));

	mutex_init(&vidc_enc_dev->lock);
	vidc_enc_dev->virt_base = vid_c_get_ioaddr();

	if (!vidc_enc_dev->virt_base) {
		ERR("%s: ioremap failed\n", __func__);
		return -ENOMEM;
	}

	vcd_init_config.device_name = "VID_C";
	vcd_init_config.pf_map_dev_base_addr = vid_enc_map_dev_base_addr;
	vcd_init_config.pf_interrupt_clr = vid_enc_interrupt_clear;
	vcd_init_config.pf_register_isr = vid_enc_interrupt_register;
	vcd_init_config.pf_deregister_isr = vid_enc_interrupt_deregister;

	rc = vcd_init(&vcd_init_config, &vidc_enc_dev->device_handle);

	if (rc) {
		ERR("%s: vcd_init failed\n", __func__);
		return -ENODEV;
	}
	return 0;
}

static int __init vid_enc_init(void)
{
	int rc = 0;
	struct device *class_devp;

	INFO("msm_vidc_enc: Inside %s\n", __func__);
	vidc_enc_dev = kzalloc(sizeof(struct vid_enc_dev), GFP_KERNEL);
	if (!vidc_enc_dev) {
		ERR("%s Unable to allocate memory for vid_enc_dev\n", __func__);
		return -ENOMEM;
	}

	rc = alloc_chrdev_region(&vidc_enc_dev_num, 0, 1, VID_ENC_NAME);
	if (rc < 0) {
		ERR("%s: alloc_chrdev_region Failed rc = %d\n", __func__, rc);
		goto error_vid_enc_alloc_chrdev_region;
	}

	vid_enc_class = class_create(THIS_MODULE, VID_ENC_NAME);
	if (IS_ERR(vid_enc_class)) {
		rc = PTR_ERR(vid_enc_class);
		ERR("%s: couldn't create vid_enc_class %d\n", __func__, rc);
		goto error_vid_enc_class_create;
	}

	class_devp = device_create(vid_enc_class, NULL, vidc_enc_dev_num, NULL,
		VID_ENC_NAME);
	if (IS_ERR(class_devp)) {
		rc = PTR_ERR(class_devp);
		ERR("%s: class device_create failed %d\n", __func__, rc);
		goto error_vid_enc_class_device_create;
	}

	vidc_enc_dev->device = class_devp;

	cdev_init(&vidc_enc_dev->cdev, &vid_enc_fops);
	vidc_enc_dev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&vidc_enc_dev->cdev, vidc_enc_dev_num, 1);

	if (rc < 0) {
		ERR("%s: cdev_add failed %d\n", __func__, rc);
		goto error_vid_enc_cdev_add;
	}
	vid_enc_vcd_init();
	return 0;

error_vid_enc_cdev_add:
	device_destroy(vid_enc_class, vidc_enc_dev_num);
error_vid_enc_class_device_create:
	class_destroy(vid_enc_class);
error_vid_enc_class_create:
	unregister_chrdev_region(vidc_enc_dev_num, 1);
error_vid_enc_alloc_chrdev_region:
	kfree(vidc_enc_dev);

	return rc;
}

static void __exit vid_enc_exit(void)
{
	INFO("msm_vidc_enc: Inside %s\n", __func__);
	cdev_del(&vidc_enc_dev->cdev);
	device_destroy(vid_enc_class, vidc_enc_dev_num);
	class_destroy(vid_enc_class);
	unregister_chrdev_region(vidc_enc_dev_num, 1);
	kfree(vidc_enc_dev);
	INFO("msm_vidc_enc: Return from %s\n", __func__);
}

static int vid_enc_ioctl(struct inode *inode, struct file *file,
	unsigned cmd, unsigned long arg)
{
	void __user *u_arg = (void __user *)arg;
	struct video_client_ctx *client_ctx;
	struct venc_ioctl_msg venc_msg;
	u32 result = true;

	DBG("%s\n", __func__);

	client_ctx = file->private_data;
	if (!client_ctx) {
		ERR("!client_ctx. Cannot attach to device handle\n");
		return -ENODEV;
	}

	switch (cmd) {
	case VEN_IOCTL_CMD_READ_NEXT_MSG:
	{
		struct venc_msg cb_msg;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;
		DBG("VEN_IOCTL_CMD_READ_NEXT_MSG\n");
		result = vid_enc_get_next_msg(client_ctx, &cb_msg);
		if (!result) {
			ERR("VEN_IOCTL_CMD_READ_NEXT_MSG failed\n");
			return -EIO;
		}
		if (copy_to_user(venc_msg.out, &cb_msg, sizeof(cb_msg)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_CMD_STOP_READ_MSG:
		DBG("VEN_IOCTL_CMD_STOP_READ_MSG\n");
		client_ctx->stop_msg = 1;
		wake_up(&client_ctx->msg_wait);
		break;
	case VEN_IOCTL_CMD_ENCODE_FRAME:
	case VEN_IOCTL_CMD_FILL_OUTPUT_BUFFER:
	{
		struct venc_buffer enc_buf;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_CMD_ENCODE_FRAME/"
			"VEN_IOCTL_CMD_FILL_OUTPUT_BUFFER\n");

		if (copy_from_user(&enc_buf, venc_msg.in, sizeof(enc_buf)))
			return -EFAULT;

		if (cmd == VEN_IOCTL_CMD_ENCODE_FRAME)
			result = vid_enc_encode_frame(client_ctx, &enc_buf);
		else
			result = vid_enc_fill_output_buffer(client_ctx,
				&enc_buf);

		if (!result) {
			DBG("VEN_IOCTL_CMD_ENCODE_FRAME/"
				"VEN_IOCTL_CMD_FILL_OUTPUT_BUFFER failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_SET_INPUT_BUFFER:
	case VEN_IOCTL_SET_OUTPUT_BUFFER:
	{
		struct venc_bufferpayload buf_info;
		enum venc_buffer_dir buf_dir;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INPUT_BUFFER/VEN_IOCTL_SET_OUTPUT_BUFFER\n");

		if (copy_from_user(&buf_info, venc_msg.in, sizeof(buf_info)))
			return -EFAULT;

		buf_dir = VEN_BUFFER_TYPE_INPUT;
		if (cmd == VEN_IOCTL_SET_OUTPUT_BUFFER)
			buf_dir = VEN_BUFFER_TYPE_OUTPUT;

		result = vid_enc_set_buffer(client_ctx, &buf_info, buf_dir);
		if (!result) {
			DBG("VEN_IOCTL_SET_INPUT_BUFFER"
				"/VEN_IOCTL_SET_OUTPUT_BUFFER failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_SET_INPUT_BUFFER_REQ:
	case VEN_IOCTL_SET_OUTPUT_BUFFER_REQ:
	{
		struct venc_allocatorproperty alloc;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INPUT_BUFFER_REQ"
			"/VEN_IOCTL_SET_OUTPUT_BUFFER_REQ\n");

		if (copy_from_user(&alloc, venc_msg.in, sizeof(alloc)))
			return -EFAULT;

		if (cmd == VEN_IOCTL_SET_OUTPUT_BUFFER_REQ)
			result = vid_enc_set_buffer_req(client_ctx, &alloc,
				false);
		else
			result = vid_enc_set_buffer_req(client_ctx, &alloc,
				true);
		if (!result) {
			DBG("setting VEN_IOCTL_SET_OUTPUT_BUFFER_REQ/"
				"VEN_IOCTL_SET_INPUT_BUFFER_REQ failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_INPUT_BUFFER_REQ:
	case VEN_IOCTL_GET_OUTPUT_BUFFER_REQ:
	{
		struct venc_allocatorproperty alloc;

		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_INPUT_BUFFER_REQ/"
			"VEN_IOCTL_GET_OUTPUT_BUFFER_REQ\n");

		if (cmd == VEN_IOCTL_GET_OUTPUT_BUFFER_REQ)
			result = vid_enc_get_buffer_req(client_ctx, &alloc,
				false);
		else
			result = vid_enc_get_buffer_req(client_ctx, &alloc,
				true);

		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &alloc, sizeof(alloc)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_CMD_FLUSH:
	{
		struct venc_bufferflush buf_flush;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_CMD_FLUSH\n");

		if (copy_from_user(&buf_flush, venc_msg.in, sizeof(buf_flush)))
			return -EFAULT;

		INFO("%s: Calling vid_enc_flush with mode = %lu\n", __func__,
			buf_flush.flush_mode);
		result = vid_enc_flush(client_ctx, &buf_flush);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_FLUSH failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_CMD_START:
		INFO("%s: Executing VEN_IOCTL_CMD_START\n", __func__);
		result = vid_enc_start(client_ctx);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_START failed\n");
			return -EIO;
		}
		break;
	case VEN_IOCTL_CMD_STOP:
		INFO("%s: Executing VEN_IOCTL_CMD_STOP", __func__);
		result = vid_enc_stop(client_ctx);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_STOP failed\n");
			return -EIO;
		}
		stop_cmd = 1;
		break;
	case VEN_IOCTL_CMD_PAUSE:
		INFO("%s: Executing VEN_IOCTL_CMD_PAUSE\n", __func__);
		result = vid_enc_pause(client_ctx);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_PAUSE failed\n");
			return -EIO;
		}
		break;
	case VEN_IOCTL_CMD_RESUME:
		INFO("%s: Executing VEN_IOCTL_CMD_RESUME\n", __func__);
		result = vid_enc_resume(client_ctx);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_RESUME failed\n");
			return -EIO;
		}
		break;
	case VEN_IOCTL_SET_QP_RANGE:
	{
		struct venc_qprange qprange;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_QP_RANGE\n");

		if (copy_from_user(&qprange, venc_msg.in, sizeof(qprange)))
			return -EFAULT;

		result = vid_enc_set_get_qprange(client_ctx, &qprange, true);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_QP_RANGE failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_QP_RANGE:
	{
		struct venc_qprange qprange;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_QP_RANGE\n");
		result = vid_enc_set_get_qprange(client_ctx, &qprange, false);

		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &qprange, sizeof(qprange)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_HEC:
	{
		struct venc_headerextension ext;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_HEC\n");

		if (copy_from_user(&ext, venc_msg.in, sizeof(ext)))
			return -EFAULT;

		result = vid_enc_set_get_headerextension(client_ctx, &ext,
			true);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_HEC failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_HEC:
	{
		struct venc_headerextension ext;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_HEC\n");
		result = vid_enc_set_get_headerextension(client_ctx, &ext,
			false);

		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &ext, sizeof(ext)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_TARGET_BITRATE:
	{
		struct venc_targetbitrate rate;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_TARGET_BITRATE\n");

		if (copy_from_user(&rate, venc_msg.in, sizeof(rate)))
			return -EFAULT;

		result = vid_enc_set_get_bitrate(client_ctx, &rate, true);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_TARGET_BITRATE failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_TARGET_BITRATE:
	{
		struct venc_targetbitrate rate;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_TARGET_BITRATE\n");
		result = vid_enc_set_get_bitrate(client_ctx, &rate, false);

		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &rate, sizeof(rate)))
				return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_FRAME_RATE:
	{
		struct venc_framerate frm_rate;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_FRAME_RATE\n");

		if (copy_from_user(&frm_rate, venc_msg.in, sizeof(frm_rate)))
			return -EFAULT;

		result = vid_enc_set_get_framerate(client_ctx, &frm_rate,
			true);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_FRAME_RATE failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_FRAME_RATE:
	{
		struct venc_framerate frm_rate;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_FRAME_RATE\n");
		result = vid_enc_set_get_framerate(client_ctx, &frm_rate,
			false);

		if (result) {
			if (copy_to_user(venc_msg.out,
					&frm_rate, sizeof(frm_rate)))
				return -EFAULT;
		} else
			return -EIO;
		break;
	}
	case VEN_IOCTL_SET_VOP_TIMING_CFG:
	{
		struct venc_voptimingcfg timing;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_VOP_TIMING_CFG\n");

		if (copy_from_user(&timing, venc_msg.in, sizeof(timing)))
			return -EFAULT;

		result = vid_enc_set_get_voptimingcfg(client_ctx, &timing,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_VOP_TIMING_CFG failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_VOP_TIMING_CFG:
	{
		struct venc_voptimingcfg timing;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_VOP_TIMING_CFG\n");
		result = vid_enc_set_get_voptimingcfg(client_ctx, &timing,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &timing, sizeof(timing)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_RATE_CTRL_CFG:
	{
		struct venc_ratectrlcfg rate_ctrl;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_RATE_CTRL_CFG\n");

		if (copy_from_user(&rate_ctrl, venc_msg.in, sizeof(rate_ctrl)))
			return -EFAULT;

		result = vid_enc_set_get_ratectrlcfg(client_ctx, &rate_ctrl,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_RATE_CTRL_CFG failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_RATE_CTRL_CFG:
	{
		struct venc_ratectrlcfg rate_ctrl;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_RATE_CTRL_CFG\n");
		result = vid_enc_set_get_ratectrlcfg(client_ctx, &rate_ctrl,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &rate_ctrl, sizeof(rate_ctrl)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_MULTI_SLICE_CFG:
	{
		struct venc_multiclicecfg slice;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_MULTI_SLICE_CFG\n");

		if (copy_from_user(&slice, venc_msg.in, sizeof(slice)))
			return -EFAULT;

		result = vid_enc_set_get_multiclicecfg(client_ctx, &slice,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_MULTI_SLICE_CFG failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_MULTI_SLICE_CFG:
	{
		struct venc_multiclicecfg slice;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_MULTI_SLICE_CFG\n");
		result = vid_enc_set_get_multiclicecfg(client_ctx, &slice,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &slice, sizeof(slice)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_INTRA_REFRESH:
	{
		struct venc_intrarefresh refresh;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INTRA_REFRESH\n");

		if (copy_from_user(&refresh, venc_msg.in, sizeof(refresh)))
			return -EFAULT;

		result = vid_enc_set_get_intrarefresh(client_ctx, &refresh,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_INTRA_REFRESH failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_INTRA_REFRESH:
	{
		struct venc_intrarefresh refresh;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_DEBLOCKING_CFG\n");
		result = vid_enc_set_get_intrarefresh(client_ctx, &refresh,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &refresh, sizeof(refresh)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_DEBLOCKING_CFG:
	{
		struct venc_dbcfg dbcfg;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_DEBLOCKING_CFG\n");

		if (copy_from_user(&dbcfg, venc_msg.in, sizeof(dbcfg)))
			return -EFAULT;
		result = vid_enc_set_get_dbcfg(client_ctx, &dbcfg, true);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_DEBLOCKING_CFG failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_DEBLOCKING_CFG:
	{
		struct venc_dbcfg dbcfg;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_DEBLOCKING_CFG\n");
		result = vid_enc_set_get_dbcfg(client_ctx, &dbcfg, false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &dbcfg, sizeof(dbcfg)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_ENTROPY_CFG:
	{
		struct venc_entropycfg entropy;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_ENTROPY_CFG\n");

		if (copy_from_user(&entropy, venc_msg.in, sizeof(entropy)))
			return -EFAULT;

		result = vid_enc_set_get_entropy_cfg(client_ctx, &entropy,
			true);

		if (!result) {
			ERR("setting VEN_IOCTL_SET_ENTROPY_CFG failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_ENTROPY_CFG:
	{
		struct venc_entropycfg entropy;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_ENTROPY_CFG\n");

		result = vid_enc_set_get_entropy_cfg(client_ctx, &entropy,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &entropy, sizeof(entropy)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_GET_SEQUENCE_HDR:
	{
		int rc = 0;
		struct venc_seqheader hdr;
		struct venc_seqheader hdr_user;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_SEQUENCE_HDR\n");

		if (copy_from_user(&hdr, venc_msg.in, sizeof(hdr)))
			return -EFAULT;
		if (copy_from_user(&hdr_user, venc_msg.in, sizeof(hdr_user)))
			return -EFAULT;

		hdr.buf = NULL;
		result = vid_enc_get_sequence_header(client_ctx, &hdr);
		if (!result)
			rc = -EIO;
		if (!rc || copy_to_user(hdr_user.buf, hdr.buf, hdr.hdr_len))
			rc = -EFAULT;
		if (!rc || copy_to_user(&hdr_user.hdr_len, &hdr.hdr_len,
				sizeof(hdr.hdr_len)))
			rc = -EFAULT;

		kfree(hdr.buf);
		hdr.buf = NULL;
		if (rc)
			return rc;
		break;
	}
	case VEN_IOCTL_GET_CAPABILITY:
		return -EIO;
	case VEN_IOCTL_CMD_REQUEST_IFRAME:
		result = vid_enc_request_iframe(client_ctx);
		if (!result) {
			ERR("setting VEN_IOCTL_CMD_REQUEST_IFRAME failed\n");
			return -EIO;
		}
		break;
	case VEN_IOCTL_SET_INTRA_PERIOD:
	{
		struct venc_intraperiod period;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_INTRA_PERIOD\n");

		if (copy_from_user(&period, venc_msg.in, sizeof(period)))
			return -EFAULT;

		result = vid_enc_set_get_intraperiod(client_ctx, &period,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_INTRA_PERIOD failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_INTRA_PERIOD:
	{
		struct venc_intraperiod period;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_SESSION_QP\n");

		result = vid_enc_set_get_intraperiod(client_ctx, &period,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &period, sizeof(period)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_SESSION_QP:
	{
		struct venc_sessionqp qp;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_SESSION_QP\n");

		if (copy_from_user(&qp,	venc_msg.in, sizeof(qp)))
			return -EFAULT;

		result = vid_enc_set_get_session_qp(client_ctx,	&qp, true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_SESSION_QP failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_SESSION_QP:
	{
		struct venc_sessionqp qp;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_SESSION_QP\n");

		result = vid_enc_set_get_session_qp(client_ctx, &qp, false);

		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &qp, sizeof(qp)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_PROFILE_LEVEL:
	{
		struct ven_profilelevel level;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_PROFILE_LEVEL\n");

		if (copy_from_user(&level, venc_msg.in, sizeof(level)))
			return -EFAULT;

		result = vid_enc_set_get_profile_level(client_ctx, &level,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_PROFILE_LEVEL failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_PROFILE_LEVEL:
	{
		struct ven_profilelevel level;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_CODEC_PROFILE\n");

		result = vid_enc_set_get_profile_level(client_ctx, &level,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &level, sizeof(level)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_CODEC_PROFILE:
	{
		struct venc_profile profile;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_SET_CODEC_PROFILE\n");

		if (copy_from_user(&profile, venc_msg.in, sizeof(profile)))
			return -EFAULT;

		result = vid_enc_set_get_profile(client_ctx, &profile, true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_CODEC_PROFILE failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_CODEC_PROFILE:
	{
		struct venc_profile profile;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_CODEC_PROFILE\n");

		result = vid_enc_set_get_profile(client_ctx, &profile, false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &profile, sizeof(profile)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_SHORT_HDR:
	{
		struct venc_switch enc_switch;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("Getting VEN_IOCTL_SET_SHORT_HDR\n");

		if (copy_from_user(&enc_switch, venc_msg.in,
				sizeof(enc_switch)))
			return -EFAULT;

		result = vid_enc_set_get_short_header(client_ctx, &enc_switch,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_SHORT_HDR failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_SHORT_HDR:
	{
		struct venc_switch enc_switch;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_LIVE_MODE\n");

		result = vid_enc_set_get_short_header(client_ctx, &enc_switch,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &enc_switch, sizeof(enc_switch)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_BASE_CFG:
	{
		struct venc_basecfg base;
		DBG("VEN_IOCTL_SET_BASE_CFG\n");

		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		if (copy_from_user(&base, venc_msg.in, sizeof(base)))
			return -EFAULT;

		DBG("setting VEN_IOCTL_SET_BASE_CFG\n");

		result = vid_enc_set_get_base_cfg(client_ctx, &base, true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_BASE_CFG failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_BASE_CFG:
	{
		struct venc_basecfg base;
		DBG("VEN_IOCTL_GET_BASE_CFG\n");

		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("Getting VEN_IOCTL_SET_BASE_CFG\n");

		result = vid_enc_set_get_base_cfg(client_ctx, &base, false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &base, sizeof(base)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_LIVE_MODE:
	{
		struct venc_switch enc_switch;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("Getting VEN_IOCTL_SET_LIVE_MODE\n");

		if (copy_from_user(&enc_switch, venc_msg.in, sizeof(enc_switch)))
			return -EFAULT;

		result = vid_enc_set_get_live_mode(client_ctx, &enc_switch,
			true);
		if (!result) {
			ERR("setting VEN_IOCTL_SET_LIVE_MODE failed\n");
			return -EIO;
		}
		break;
	}
	case VEN_IOCTL_GET_LIVE_MODE:
	{
		struct venc_switch enc_switch;
		if (copy_from_user(&venc_msg, u_arg, sizeof(venc_msg)))
			return -EFAULT;

		DBG("VEN_IOCTL_GET_LIVE_MODE\n");

		result = vid_enc_set_get_live_mode(client_ctx, &enc_switch,
			false);
		if (!result)
			return -EIO;
		if (copy_to_user(venc_msg.out, &enc_switch, sizeof(enc_switch)))
			return -EFAULT;
		break;
	}
	case VEN_IOCTL_SET_AC_PREDICTION:
	case VEN_IOCTL_GET_AC_PREDICTION:
	case VEN_IOCTL_SET_RVLC:
	case VEN_IOCTL_GET_RVLC:
	case VEN_IOCTL_SET_ROTATION:
	case VEN_IOCTL_GET_ROTATION:
	case VEN_IOCTL_SET_DATA_PARTITION:
	case VEN_IOCTL_GET_DATA_PARTITION:
	default:
		ERR("%s: Unsupported ioctl %d\n", __func__, cmd);
		return -ENOTTY;
	}
	return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Video encoder driver");
MODULE_VERSION("1.0");

module_init(vid_enc_init);
module_exit(vid_enc_exit);

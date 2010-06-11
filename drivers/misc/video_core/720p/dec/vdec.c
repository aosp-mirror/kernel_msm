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
#include <linux/timer.h>

#include "vcd_ddl_firmware.h"
#include "video_core_type.h"
#include "vcd_api.h"
#include "vdec_internal.h"
#include "video_core_init.h"


#define VID_C_HCLK_RATE 170667000

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define INFO(x...) printk(KERN_INFO x)
#define ERR(x...) printk(KERN_ERR x)

#define VID_DEC_NAME "msm_vidc_dec"

static struct vid_dec_dev *vidc_dec_dev;
static dev_t vidc_dec_dev_num;
static struct class *vidc_dec_class;

static s32 vid_dec_get_empty_client_index(void)
{
	u32 i, found = false;

	for (i = 0; i < VID_DEC_MAX_DECODER_CLIENTS; i++) {
		if (!vidc_dec_dev->vdec_clients[i].vcd_handle) {
			found = true;
			break;
		}
	}
	if (!found) {
		ERR("%s():ERROR No space for new client\n", __func__);
		return -1;
	}
	DBG("%s(): available client index = %u\n", __func__, i);
	return i;
}

u32 vid_dec_get_status(u32 status)
{
	u32 vdec_status;

	switch (status) {
	case VCD_ERR_BITSTREAM_ERR:
	case VCD_S_SUCCESS:
		vdec_status = VDEC_S_SUCCESS;
		break;
	case VCD_ERR_FAIL:
		vdec_status = VDEC_S_EFAIL;
		break;
	case VCD_ERR_ALLOC_FAIL:
		vdec_status = VDEC_S_ENOSWRES;
		break;
	case VCD_ERR_ILLEGAL_OP:
		vdec_status = VDEC_S_EINVALCMD;
		break;
	case VCD_ERR_ILLEGAL_PARM:
		vdec_status = VDEC_S_EBADPARAM;
		break;
	case VCD_ERR_BAD_POINTER:
	case VCD_ERR_BAD_HANDLE:
		vdec_status = VDEC_S_EFATAL;
		break;
	case VCD_ERR_NOT_SUPPORTED:
		vdec_status = VDEC_S_ENOTSUPP;
		break;
	case VCD_ERR_BAD_STATE:
		vdec_status = VDEC_S_EINVALSTATE;
		break;
	case VCD_ERR_BUSY:
		vdec_status = VDEC_S_BUSY;
		break;
	case VCD_ERR_MAX_CLIENT:
		vdec_status = VDEC_S_ENOHWRES;
		break;
	default:
		vdec_status = VDEC_S_EFAIL;
		break;
	}

	return vdec_status;
}

static void vid_dec_notify_client(struct video_client_ctx *client_ctx)
{
	if (client_ctx)
		complete(&client_ctx->event);
}

void vid_dec_vcd_open_done(struct video_client_ctx *client_ctx,
	struct vcd_handle_container *handle_container)
{
	DBG("vid_dec_vcd_open_done\n");

	if (!client_ctx) {
		ERR("%s(): ERROR. client_ctx is NULL\n", __func__);
		return;
	}

	if (handle_container)
		client_ctx->vcd_handle = handle_container->handle;
	else
		ERR("%s(): ERROR. handle_container is NULL\n", __func__);

	vid_dec_notify_client(client_ctx);
}

static void vid_dec_input_frame_done(struct video_client_ctx *client_ctx,
	u32 event, u32 status, struct vcd_frame_data *vcd_frame_data)
{
	struct vid_dec_msg *vdec_msg;

	if (!client_ctx || !vcd_frame_data) {
		ERR("vid_dec_input_frame_done() NULL pointer\n");
		return;
	}

	vdec_msg = kzalloc(sizeof(struct vid_dec_msg), GFP_KERNEL);
	if (!vdec_msg) {
		ERR("%s: cannot allocate vid_dec_msg buffer\n", __func__);
		return;
	}

	vdec_msg->vdec_msg_info.status_code = vid_dec_get_status(status);

	if (event == VCD_EVT_RESP_INPUT_DONE) {
		vdec_msg->vdec_msg_info.msgcode =
			VDEC_MSG_RESP_INPUT_BUFFER_DONE;
		DBG("Send INPUT_DON message to client = %p\n", client_ctx);

	} else if (event == VCD_EVT_RESP_INPUT_FLUSHED) {
		vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_RESP_INPUT_FLUSHED;
		DBG("Send INPUT_FLUSHED message to client = %p\n", client_ctx);
	} else {
		ERR("%s: invalid event type\n", __func__);
		return;
	}

	vdec_msg->vdec_msg_info.msgdata.input_frame_clientdata =
		vcd_frame_data->client_data;
	vdec_msg->vdec_msg_info.msgdatasize = sizeof(void *);

	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&vdec_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}

static void vid_dec_output_frame_done(struct video_client_ctx *client_ctx,
	u32 event, u32 status, struct vcd_frame_data *vcd_frame_data)
{
	struct vid_dec_msg *vdec_msg;

	void __user *user_addr;
	void *kern_addr;
	phys_addr_t phys_addr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;
	struct vdec_output_frameinfo *frm;

	if (!client_ctx || !vcd_frame_data) {
		ERR("%s: NULL pointer\n", __func__);
		return;
	}

	vdec_msg = kzalloc(sizeof(struct vid_dec_msg), GFP_KERNEL);
	if (!vdec_msg) {
		ERR("%s: cannot allocate vid_dec_msg buffer\n", __func__);
		return;
	}

	vdec_msg->vdec_msg_info.status_code = vid_dec_get_status(status);

	if (event == VCD_EVT_RESP_OUTPUT_DONE) {
		vdec_msg->vdec_msg_info.msgcode =
			VDEC_MSG_RESP_OUTPUT_BUFFER_DONE;
	} else if (event == VCD_EVT_RESP_OUTPUT_FLUSHED) {
		vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_RESP_OUTPUT_FLUSHED;
	} else {
		ERR("QVD: vid_dec_output_frame_done invalid cmd type\n");
		return;
	}

	kern_addr = vcd_frame_data->virt_addr;

	if (!vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT, false,
			&user_addr, &kern_addr, &phys_addr, &pmem_fd, &file,
			&buffer_index)) {
		ERR("vid_dec_output_frame_done UVA can not be found\n");
		vdec_msg->vdec_msg_info.status_code = VDEC_S_EFATAL;
		goto out;
	}

	frm = &vdec_msg->vdec_msg_info.msgdata.output_frame;
	/* Buffer address in user space */
	frm->user_addr = user_addr;
	frm->phys_addr = vcd_frame_data->phys_addr;
	/* Data length */
	frm->len = vcd_frame_data->data_len;
	frm->flags = vcd_frame_data->flags;
	/* timestamp pass-through from input frame */
	frm->time_stamp = vcd_frame_data->time_stamp;
	/* Output frame client data */
	frm->client_data = vcd_frame_data->client_data;
	/* Associated input frame client data */
	frm->input_frame_clientdata = (void *)vcd_frame_data->ip_frm_tag;
	/* Decoded picture width and height */
	frm->framesize.bottom = vcd_frame_data->dec_op_prop.disp_frm.bottom;
	frm->framesize.left = vcd_frame_data->dec_op_prop.disp_frm.left;
	frm->framesize.right = vcd_frame_data->dec_op_prop.disp_frm.right;
	frm->framesize.top = vcd_frame_data->dec_op_prop.disp_frm.top;
	vdec_msg->vdec_msg_info.msgdatasize = sizeof(*frm);

out:
	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&vdec_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}

static void vid_dec_lean_event(struct video_client_ctx *client_ctx,
	u32 event, u32 status)
{
	struct vid_dec_msg *vdec_msg;

	if (!client_ctx) {
		ERR("%s(): !client_ctx pointer\n", __func__);
		return;
	}

	vdec_msg = kzalloc(sizeof(struct vid_dec_msg), GFP_KERNEL);
	if (!vdec_msg) {
		ERR("%s(): cannot allocate vid_dec_msg buffer\n", __func__);
		return;
	}

	vdec_msg->vdec_msg_info.status_code = vid_dec_get_status(status);

	switch (event) {
	case VCD_EVT_IND_RECONFIG:
		INFO("msm_vidc_dec: Sending VDEC_MSG_EVT_CONFIG_CHANGED"
			 " to client\n");
		vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_EVT_CONFIG_CHANGED;
		break;
	case VCD_EVT_IND_RESOURCES_LOST:
		INFO("msm_vidc_dec: Sending VDEC_EVT_RESOURCES_LOST"
			 " to client\n");
		vdec_msg->vdec_msg_info.msgcode = VDEC_EVT_RESOURCES_LOST;
		break;
	case VCD_EVT_RESP_FLUSH_INPUT_DONE:
		INFO("msm_vidc_dec: Sending VDEC_MSG_RESP_FLUSH_INPUT_DONE"
			 " to client\n");
		vdec_msg->vdec_msg_info.msgcode =
			VDEC_MSG_RESP_FLUSH_INPUT_DONE;
		break;
	case VCD_EVT_RESP_FLUSH_OUTPUT_DONE:
		INFO("msm_vidc_dec: Sending VDEC_MSG_RESP_FLUSH_OUTPUT_DONE"
			 " to client\n");
		vdec_msg->vdec_msg_info.msgcode =
			VDEC_MSG_RESP_FLUSH_OUTPUT_DONE;
		break;
	case VCD_EVT_IND_HWERRFATAL:
		INFO("msm_vidc_dec: Sending VDEC_MSG_EVT_HW_ERROR to client\n");
		vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_EVT_HW_ERROR;
		break;
	case VCD_EVT_RESP_START:
		INFO("msm_vidc_dec: Sending VDEC_MSG_RESP_START_DONE"
			 " to client\n");
		vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_RESP_START_DONE;
		break;
	case VCD_EVT_RESP_STOP:
		INFO("msm_vidc_dec: Sending VDEC_MSG_RESP_STOP_DONE"
			 " to client\n");
		vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_RESP_STOP_DONE;
		break;
	case VCD_EVT_RESP_PAUSE:
		INFO("msm_vidc_dec: Sending VDEC_MSG_RESP_PAUSE_DONE"
			 " to client\n");
		vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_RESP_PAUSE_DONE;
		break;
	default:
		ERR("%s() : unknown event type\n", __func__);
		break;
	}

	vdec_msg->vdec_msg_info.msgdatasize = 0;
	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&vdec_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);
	wake_up(&client_ctx->msg_wait);
}


void vid_dec_vcd_cb(u32 event, u32 status, void *info, u32 size, void *handle,
	void *const client_data)
{
	struct video_client_ctx *client_ctx = (struct video_client_ctx *)
		client_data;

	DBG("Entering %s()\n", __func__);

	if (!client_ctx) {
		ERR("%s(): client_ctx is NULL\n", __func__);
		return;
	}

	client_ctx->event_status = status;

	switch (event) {
	case VCD_EVT_RESP_OPEN:
		vid_dec_vcd_open_done(client_ctx, info);
		break;
	case VCD_EVT_RESP_INPUT_DONE:
	case VCD_EVT_RESP_INPUT_FLUSHED:
		vid_dec_input_frame_done(client_ctx, event, status, info);
		break;
	case VCD_EVT_RESP_OUTPUT_DONE:
	case VCD_EVT_RESP_OUTPUT_FLUSHED:
		vid_dec_output_frame_done(client_ctx, event, status, info);
		break;
	case VCD_EVT_RESP_PAUSE:
	case VCD_EVT_RESP_STOP:
	case VCD_EVT_RESP_FLUSH_INPUT_DONE:
	case VCD_EVT_RESP_FLUSH_OUTPUT_DONE:
	case VCD_EVT_IND_RECONFIG:
	case VCD_EVT_IND_HWERRFATAL:
	case VCD_EVT_IND_RESOURCES_LOST:
		vid_dec_lean_event(client_ctx, event, status);
		break;
	case VCD_EVT_RESP_START:
		if (!client_ctx->seq_header_set)
			vid_dec_lean_event(client_ctx, event, status);
		else
			vid_dec_notify_client(client_ctx);
		break;
	default:
		ERR("%s():  Error - Invalid event type %u\n", __func__, event);
		break;
	}
}

static u32 vid_dec_set_codec(struct video_client_ctx *client_ctx,
	enum vdec_codec *vdec_codec_type)
{
	u32 result = true;
	struct vcd_property_hdr vcd_property_hdr;
	struct vcd_property_codec codec_type;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !vdec_codec_type)
		return false;

	vcd_property_hdr.id = VCD_I_CODEC;
	vcd_property_hdr.sz = sizeof(struct vcd_property_codec);

	switch (*vdec_codec_type) {
	case VDEC_CODECTYPE_MPEG4:
		codec_type.codec = VCD_CODEC_MPEG4;
		break;
	case VDEC_CODECTYPE_H264:
		codec_type.codec = VCD_CODEC_H264;
		break;
	case VDEC_CODECTYPE_DIVX_3:
		codec_type.codec = VCD_CODEC_DIVX_3;
		break;
	case VDEC_CODECTYPE_XVID:
		codec_type.codec = VCD_CODEC_XVID;
		break;
	case VDEC_CODECTYPE_H263:
		codec_type.codec = VCD_CODEC_H263;
		break;
	case VDEC_CODECTYPE_MPEG2:
		codec_type.codec = VCD_CODEC_MPEG2;
		break;
	case VDEC_CODECTYPE_VC1:
		codec_type.codec = VCD_CODEC_VC1;
		break;
	default:
		result = false;
		break;
	}

	if (result) {
		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&vcd_property_hdr, &codec_type);
		if (vcd_status)
			result = false;
	}
	return result;
}

static u32 vid_dec_set_output_format(struct video_client_ctx *client_ctx,
	enum vdec_output_format *output_format)
{
	u32 result = true;
	struct vcd_property_hdr prop_hdr;
	struct vcd_property_buffer_format buffer_format;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !output_format)
		return false;

	prop_hdr.id = VCD_I_BUFFER_FORMAT;;
	prop_hdr.sz = sizeof(struct vcd_property_buffer_format);

	switch (*output_format) {
	case VDEC_YUV_FORMAT_NV12:
		buffer_format.buffer_format = VCD_BUFFER_FORMAT_NV12;
		break;
	case VDEC_YUV_FORMAT_TILE_4x2:
		buffer_format.buffer_format = VCD_BUFFER_FORMAT_TILE_4x2;
		break;
	default:
		result = false;
		break;
	}

	if (!result)
		return false;

	vcd_status = vcd_set_property(client_ctx->vcd_handle, &prop_hdr,
		&buffer_format);

	//TODO fix false/true silliness
	if (vcd_status)
		return false;
	else
		return true;
}

static u32 vid_dec_set_frame_resolution(struct video_client_ctx *client_ctx,
	struct vdec_picsize *video_resoultion)
{
	struct vcd_property_hdr prop_hdr;
	struct vcd_property_frame_size res;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !video_resoultion)
		return false;

	prop_hdr.id = VCD_I_FRAME_SIZE;
	prop_hdr.sz = sizeof(struct vcd_property_frame_size);
	res.width = video_resoultion->frame_width;
	res.height = video_resoultion->frame_height;

	vcd_status = vcd_set_property(client_ctx->vcd_handle, &prop_hdr, &res);

	if (vcd_status)
		return false;
	else
		return true;
}

static u32 vid_dec_get_frame_resolution(struct video_client_ctx *client_ctx,
	struct vdec_picsize *video_res)
{
	struct vcd_property_hdr prop_hdr;
	struct vcd_property_frame_size frame_res;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !video_res)
		return false;

	prop_hdr.id = VCD_I_FRAME_SIZE;
	prop_hdr.sz = sizeof(struct vcd_property_frame_size);

	vcd_status = vcd_get_property(client_ctx->vcd_handle, &prop_hdr,
		&frame_res);

	video_res->frame_width = frame_res.width;
	video_res->frame_height = frame_res.height;
	video_res->scan_lines = frame_res.scan_lines;
	video_res->stride = frame_res.stride;

	if (vcd_status)
		return false;
	else
		return true;
}

static u32 vid_dec_get_buffer_req(struct video_client_ctx *client_ctx,
	struct vdec_allocatorproperty *vdec_buf_req)
{
	u32 vcd_status = VCD_ERR_FAIL;
	struct vcd_buffer_requirement vcd_buf_req;

	if (!client_ctx || !vdec_buf_req)
		return false;

	if (vdec_buf_req->buffer_type == VDEC_BUFFER_TYPE_INPUT) {
		vcd_status = vcd_get_buffer_requirements(client_ctx->vcd_handle,
			VCD_BUFFER_INPUT, &vcd_buf_req);
	} else {
		vcd_status = vcd_get_buffer_requirements(client_ctx->vcd_handle,
			VCD_BUFFER_OUTPUT, &vcd_buf_req);
	}

	if (vcd_status)
		return false;

	vdec_buf_req->mincount = vcd_buf_req.min_count;
	vdec_buf_req->maxcount = vcd_buf_req.max_count;
	vdec_buf_req->actualcount = vcd_buf_req.actual_count;
	vdec_buf_req->buffer_size = vcd_buf_req.size;
	vdec_buf_req->alignment = vcd_buf_req.align;
	vdec_buf_req->buf_poolid = vcd_buf_req.buf_pool_id;

	return true;
}

static u32 vid_dec_set_buffer(struct video_client_ctx *client_ctx,
	struct vdec_setbuffer_cmd *b_info)
{
	enum vcd_buffer_type buffer_type;
	enum buffer_dir dir_buffer = BUFFER_TYPE_INPUT;
	u32 vcd_status = VCD_ERR_FAIL;

	void __user *user_addr;
	void *kern_addr;
	phys_addr_t phys_addr;
	unsigned long len;
	int pmem_fd;
	struct file *file;
	struct buf_addr_table *addr_table;
	s32 buffer_index = -1;

	if (!client_ctx || !b_info)
		return false;

	user_addr = b_info->buffer.addr;

	if (b_info->buffer_type == VDEC_BUFFER_TYPE_OUTPUT)
		dir_buffer = BUFFER_TYPE_OUTPUT;

	/* if buffer already set, ignore */
	if (vid_c_lookup_addr_table(client_ctx, dir_buffer, true, &user_addr,
			&kern_addr, &phys_addr, &pmem_fd, &file,
			&buffer_index)) {
		DBG("%s: user_addr = %p is already set\n", __func__, user_addr);
		return true;
	}

	if (get_pmem_file(b_info->buffer.pmem_fd, (unsigned long *)&phys_addr,
			(unsigned long *)&kern_addr, &len, &file)) {
		ERR("%s: get_pmem_file failed\n", __func__);
		return false;
	}
	put_pmem_file(file);
	if (b_info->buffer_type == VDEC_BUFFER_TYPE_INPUT) {
		buffer_type = VCD_BUFFER_INPUT;
		client_ctx->num_of_input_buffers++;
		if (client_ctx->num_of_input_buffers > MAX_VIDEO_NUM_OF_BUFF) {
			ERR("%s(): num_of_input_buffers reached max value"
				" MAX_VIDEO_NUM_OF_BUFF\n", __func__);
			client_ctx->num_of_input_buffers--;
			return false;
		}
		buffer_index = client_ctx->num_of_input_buffers - 1;
		addr_table = &client_ctx->input_buf_addr_table[buffer_index];
		addr_table->user_addr = b_info->buffer.addr;
		addr_table->kern_addr = kern_addr;
		addr_table->phys_addr = phys_addr;
		addr_table->pmem_fd = b_info->buffer.pmem_fd;
		addr_table->file = file;
	} else {
		buffer_type = VCD_BUFFER_OUTPUT;
		client_ctx->num_of_output_buffers++;
		if (client_ctx->num_of_output_buffers >	MAX_VIDEO_NUM_OF_BUFF) {
			ERR("%s(): num_of_outut_buffers reached max value"
				" MAX_VIDEO_NUM_OF_BUFF\n", __func__);
			client_ctx->num_of_output_buffers--;
			return false;
		}
		buffer_index = client_ctx->num_of_output_buffers - 1;
		addr_table = &client_ctx->output_buf_addr_table[buffer_index];
		kern_addr = (u8 *)kern_addr + b_info->buffer.offset;
		phys_addr += b_info->buffer.offset;
		addr_table->user_addr =	b_info->buffer.addr;
		addr_table->kern_addr = kern_addr;
		addr_table->phys_addr = phys_addr;
		addr_table->pmem_fd = b_info->buffer.pmem_fd;
		addr_table->file = file;
	}

	vcd_status = vcd_set_buffer(client_ctx->vcd_handle, buffer_type,
		kern_addr, b_info->buffer.sz);

	if (!vcd_status)
		return true;
	else
		return false;
}

static u32 vid_dec_free_buffer(struct video_client_ctx *client_ctx,
	struct vdec_setbuffer_cmd *buffer_info)
{
	enum vcd_buffer_type buffer_type;
	enum buffer_dir dir_buffer = BUFFER_TYPE_INPUT;
	u32 vcd_status = VCD_ERR_FAIL;
	void __user *user_addr;
	void *kern_addr;
	phys_addr_t phys_addr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;

	if (!client_ctx || !buffer_info)
		return false;

	user_addr = buffer_info->buffer.addr;

	if (buffer_info->buffer_type == VDEC_BUFFER_TYPE_OUTPUT)
		dir_buffer = BUFFER_TYPE_OUTPUT;

	/*If buffer already set, ignore */
	if (!vid_c_lookup_addr_table(client_ctx, dir_buffer, true, &user_addr,
			&kern_addr, &phys_addr, &pmem_fd, &file,
			&buffer_index)) {
		DBG("%s: user_addr = %p is already set\n", __func__, user_addr);
		return true;
	}

	if (buffer_info->buffer_type == VDEC_BUFFER_TYPE_INPUT)
		buffer_type = VCD_BUFFER_INPUT;
	else
		buffer_type = VCD_BUFFER_OUTPUT;
	vcd_status = vcd_free_buffer(client_ctx->vcd_handle, buffer_type,
		kern_addr);

	if (!vcd_status)
		return true;
	else
		return false;
}

static u32 vid_dec_pause_resume(struct video_client_ctx *client_ctx, u32 pause)
{
	u32 vcd_status;

	if (!client_ctx) {
		ERR("%s: Invalid client_ctx\n", __func__);
		return false;
	}

	if (pause) {
		INFO("msm_vidc_dec: PAUSE command from client = %p\n",
			 client_ctx);
		vcd_status = vcd_pause(client_ctx->vcd_handle);
	} else {
		INFO("msm_vidc_dec: RESUME command from client = %p\n",
			 client_ctx);
		vcd_status = vcd_resume(client_ctx->vcd_handle);
	}

	if (vcd_status)
		return false;

	return true;

}
static u32 vid_dec_start(struct video_client_ctx *client_ctx)
{
	struct vid_dec_msg *vdec_msg = NULL;
	u32 vcd_status;

	INFO("msm_vidc_dec: Inside %s\n", __func__);
	if (!client_ctx) {
		ERR("\n Invalid client_ctx");
		return false;
	}

	if (!client_ctx->seq_header_set) {
		INFO("%s: Calling decode_start()\n", __func__);
		vcd_status = vcd_decode_start(client_ctx->vcd_handle, NULL);

		if (vcd_status) {
			ERR("%s: vcd_decode_start failed vcd_status = %u\n",
				__func__, vcd_status);
			return false;
		}
		return true;
	}

	INFO("%s(): Seq Hdr set: Send START_DONE to client\n", __func__);
	vdec_msg = kzalloc(sizeof(*vdec_msg), GFP_KERNEL);
	if (!vdec_msg) {
		ERR("%s: cannot allocate buffer\n", __func__);
		return false;
	}
	vdec_msg->vdec_msg_info.msgcode = VDEC_MSG_RESP_START_DONE;
	vdec_msg->vdec_msg_info.status_code = VDEC_S_SUCCESS;
	vdec_msg->vdec_msg_info.msgdatasize = 0;
	mutex_lock(&client_ctx->msg_queue_lock);
	list_add_tail(&vdec_msg->list, &client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);

	wake_up(&client_ctx->msg_wait);

	DBG("Send START_DONE message to client = %p\n",	client_ctx);

	return true;
}

static u32 vid_dec_stop(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;

	INFO("msm_vidc_dec: Inside %s\n", __func__);
	if (!client_ctx) {
		ERR("Invalid client_ctx\n");
		return false;
	}

	INFO("%s: Calling vcd_stop()\n", __func__);
	vcd_status = vcd_stop(client_ctx->vcd_handle);
	if (vcd_status) {
		ERR("%s: vcd_stop failed %u\n", __func__, vcd_status);
		return false;
	}
	DBG("Send STOP_DONE message to client = %p\n", client_ctx);
	return true;
}

static u32 vid_dec_decode_frame(struct video_client_ctx *client_ctx,
	struct vdec_input_frameinfo *frm_info)
{
	struct vcd_frame_data vcd_input_buffer;
	void *kern_addr;
	void __user *user_addr;
	phys_addr_t phys_addr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;
	u32 vcd_status = VCD_ERR_FAIL;

	if (!client_ctx || !frm_info)
		return false;

	user_addr = frm_info->user_addr;

	if (!vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_INPUT, true,
			&user_addr, &kern_addr, &phys_addr, &pmem_fd, &file,
			&buffer_index)) {
		ERR("%s: kern_addr not found\n", __func__);
		return false;
	}

	/* kernel_vaddr  is found. send the frame to VCD */
	memset((void *)&vcd_input_buffer, 0, sizeof(vcd_input_buffer));
	vcd_input_buffer.virt_addr = (u8 *)kern_addr + frm_info->pmem_offset;
	vcd_input_buffer.offset = frm_info->offset;
	vcd_input_buffer.client_data = frm_info->client_data;
	vcd_input_buffer.ip_frm_tag = (u32)frm_info->client_data;
	vcd_input_buffer.data_len = frm_info->data_len;
	vcd_input_buffer.time_stamp = frm_info->timestamp;
	/* Rely on VCD using the same flags as OMX */
	vcd_input_buffer.flags = frm_info->flags;

	vcd_status = vcd_decode_frame(client_ctx->vcd_handle,
		&vcd_input_buffer);

	if (vcd_status) {
		ERR("%s: vcd_decode_frame failed = %u\n", __func__, vcd_status);
		return false;
	}
	return true;
}

static u32 vid_dec_fill_output_buffer(struct video_client_ctx *client_ctx,
	struct vdec_fillbuffer_cmd *fill_buffer_cmd)
{
	void *kern_addr;
	void __user *user_addr;
	phys_addr_t phys_addr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;
	u32 vcd_status = VCD_ERR_FAIL;
	struct vcd_frame_data vcd_frame;

	if (!client_ctx || !fill_buffer_cmd)
		return false;

	user_addr = fill_buffer_cmd->buffer.addr;

	if (!vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT, true,
			&user_addr, &kern_addr, &phys_addr, &pmem_fd, &file,
			&buffer_index)) {
		ERR("%s: kern_addr not found\n", __func__);
		return false;
	}

	memset((void *)&vcd_frame, 0, sizeof(vcd_frame));
	vcd_frame.virt_addr = kern_addr;
	vcd_frame.client_data = fill_buffer_cmd->client_data;
	vcd_frame.alloc_len = fill_buffer_cmd->buffer.sz;

	vcd_status = vcd_fill_output_buffer(client_ctx->vcd_handle, &vcd_frame);
	if (vcd_status) {
		ERR("%s: vcd_fill_output_buffer failed = %u\n",	__func__,
			vcd_status);
		return false;
	}
	return true;
}

static u32 vid_dec_flush(struct video_client_ctx *client_ctx,
	enum vdec_bufferflush flush_dir)
{
	u32 vcd_status = VCD_ERR_FAIL;

	INFO("msm_vidc_dec: %s called with dir = %u\n", __func__, flush_dir);
	if (!client_ctx) {
		ERR("Invalid client_ctx\n");
		return false;
	}

	switch (flush_dir) {
	case VDEC_FLUSH_TYPE_INPUT:
		vcd_status = vcd_flush(client_ctx->vcd_handle, VCD_FLUSH_INPUT);
		break;
	case VDEC_FLUSH_TYPE_OUTPUT:
		vcd_status = vcd_flush(client_ctx->vcd_handle,
			VCD_FLUSH_OUTPUT);
		break;
	case VDEC_FLUSH_TYPE_ALL:
		vcd_status = vcd_flush(client_ctx->vcd_handle, VCD_FLUSH_ALL);
		break;
	default:
		ERR("%s: Invalid flush cmd. flush_dir = %u\n", __func__,
			flush_dir);
		return false;
	}

	if (vcd_status) {
		ERR("%s: vcd_flush failed. vcd_status = %u flush_dir = %u\n",
			__func__, vcd_status, flush_dir);
		return false;
	}
	return true;
}

static u32 vid_dec_msg_pending(struct video_client_ctx *client_ctx)
{
	u32 islist_empty = 0;
	mutex_lock(&client_ctx->msg_queue_lock);
	islist_empty = list_empty(&client_ctx->msg_queue);
	mutex_unlock(&client_ctx->msg_queue_lock);

	if (islist_empty) {
		DBG("%s: vid_dec msg queue empty\n", __func__);
		if (client_ctx->stop_msg) {
			DBG("%s: List empty and Stop Msg set\n", __func__);
			return client_ctx->stop_msg;
		}
	} else {
		DBG("%s: vid_dec msg queue Not empty\n", __func__);
	}

	return !islist_empty;
}

static u32 vid_dec_get_next_msg(struct video_client_ctx *client_ctx,
	struct vdec_msginfo *vdec_msg_info)
{
	int rc;
	struct vid_dec_msg *vid_dec_msg = NULL;

	if (!client_ctx)
		return false;

	rc = wait_event_interruptible(client_ctx->msg_wait,
		vid_dec_msg_pending(client_ctx));
	if (rc < 0 || client_ctx->stop_msg) {
		DBG("rc = %d, stop_msg = %u\n", rc, client_ctx->stop_msg);
		return false;
	}

	mutex_lock(&client_ctx->msg_queue_lock);
	if (!list_empty(&client_ctx->msg_queue)) {
		DBG("%s(): After Wait\n", __func__);
		vid_dec_msg = list_first_entry(&client_ctx->msg_queue,
			struct vid_dec_msg, list);
		list_del(&vid_dec_msg->list);
		memcpy(vdec_msg_info, &vid_dec_msg->vdec_msg_info,
		       sizeof(struct vdec_msginfo));
		kfree(vid_dec_msg);
	}
	mutex_unlock(&client_ctx->msg_queue_lock);
	return true;
}

static int vid_dec_ioctl(struct inode *inode, struct file *file,
	unsigned cmd, unsigned long arg)
{
	struct video_client_ctx *client_ctx = NULL;
	struct vdec_ioctl_msg vdec_msg;
	u32 vcd_status;
	void *kern_addr;
	phys_addr_t phys_addr;
	struct file *pmem_file;
	u32 result = true;
	void __user *u_arg = (void __user *)arg;

	DBG("%s\n", __func__);

	if (_IOC_TYPE(cmd) != VDEC_IOCTL_MAGIC)
		return -ENOTTY;

	client_ctx = (struct video_client_ctx *)file->private_data;
	if (!client_ctx) {
		ERR("!client_ctx. Cannot attach to device handle\n");
		return -ENODEV;
	}

	switch (cmd) {
	case VDEC_IOCTL_SET_CODEC:
	{
		enum vdec_codec codec_type;
		struct vcd_property_meta_data_enable metdata_disable;
		struct vcd_property_hdr header_type;
		DBG("VDEC_IOCTL_SET_CODEC\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&codec_type, vdec_msg.in,
				sizeof(codec_type)))
			return -EFAULT;
		DBG("setting code type = %u\n", codec_type);
		result = vid_dec_set_codec(client_ctx, &codec_type);
		if (!result)
			return -EIO;
		metdata_disable.meta_data_enable_flag = 0;
		header_type.sz = sizeof(metdata_disable);
		header_type.id = VCD_I_METADATA_ENABLE;

		vcd_status = vcd_set_property(client_ctx->vcd_handle,
			&header_type, (void *)&metdata_disable);

		if (vcd_status) {
			ERR("%s: vcd_set_property Failed for Meta Data Disable"
				"\n", __func__);
			return -ENODEV;
		}
		DBG("Disabled Meta Data\n");
		break;
	}
	case VDEC_IOCTL_SET_OUTPUT_FORMAT:
	{
		enum vdec_output_format out_format;
		DBG("VDEC_IOCTL_SET_OUTPUT_FORMAT\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&out_format, vdec_msg.in,
				sizeof(out_format)))
			return -EFAULT;

		result = vid_dec_set_output_format(client_ctx, &out_format);

		if (!result)
			return -EIO;
		break;
	}
	case VDEC_IOCTL_SET_PICRES:
	{
		struct vdec_picsize video_res;
		DBG("VDEC_IOCTL_SET_PICRES\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&video_res, vdec_msg.in, sizeof(video_res)))
			return -EFAULT;
		result = vid_dec_set_frame_resolution(client_ctx,
			&video_res);
		if (!result)
			return -EIO;
		break;
	}
	case VDEC_IOCTL_GET_PICRES:
	{
		struct vdec_picsize video_res;
		DBG("VDEC_IOCTL_GET_PICRES\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&video_res, vdec_msg.out, sizeof(video_res)))
			return -EFAULT;

		result = vid_dec_get_frame_resolution(client_ctx, &video_res);

		if (!result)
			return -EIO;

		if (copy_to_user(vdec_msg.out, &video_res,sizeof(video_res)))
			return -EFAULT;
		break;
	}
	case VDEC_IOCTL_SET_BUFFER_REQ:
	{
		//TODO unify these types
		struct vdec_allocatorproperty vdec_buf_req;
		struct vcd_buffer_requirement vcd_buf_req;

		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;

		if (copy_from_user(&vdec_buf_req, vdec_msg.in,
				sizeof(vdec_buf_req)))
			return -EFAULT;

		vcd_buf_req.actual_count = vdec_buf_req.actualcount;
		vcd_buf_req.align = vdec_buf_req.alignment;
		vcd_buf_req.max_count = vdec_buf_req.maxcount;
		vcd_buf_req.min_count = vdec_buf_req.mincount;
		vcd_buf_req.size = vdec_buf_req.buffer_size;

		switch (vdec_buf_req.buffer_type) {
		case VDEC_BUFFER_TYPE_INPUT:
			vcd_status = vcd_set_buffer_requirements(
				client_ctx->vcd_handle,	VCD_BUFFER_INPUT,
				&vcd_buf_req);
			break;
		case VDEC_BUFFER_TYPE_OUTPUT:
			vcd_status = vcd_set_buffer_requirements(
				client_ctx->vcd_handle,	VCD_BUFFER_OUTPUT,
				&vcd_buf_req);
			break;
		default:
			vcd_status = VCD_ERR_BAD_POINTER;
			break;
		}

		if (vcd_status)
			return -EFAULT;
		break;
	}
	case VDEC_IOCTL_GET_BUFFER_REQ:
	{
		struct vdec_allocatorproperty vdec_buf_req;
		DBG("VDEC_IOCTL_GET_BUFFER_REQ\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&vdec_buf_req, vdec_msg.out,
				sizeof(vdec_buf_req)))
			return -EFAULT;

		result = vid_dec_get_buffer_req(client_ctx, &vdec_buf_req);
		if (!result)
			return -EIO;

		if (copy_to_user(vdec_msg.out, &vdec_buf_req,
				sizeof(vdec_buf_req)))
			return -EFAULT;
		break;
	}
	case VDEC_IOCTL_SET_BUFFER:
	{
		struct vdec_setbuffer_cmd setbuffer;
		DBG("VDEC_IOCTL_SET_BUFFER\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&setbuffer, vdec_msg.in, sizeof(setbuffer)))
			return -EFAULT;
		result = vid_dec_set_buffer(client_ctx, &setbuffer);
		break;
	}
	case VDEC_IOCTL_FREE_BUFFER:
	{
		struct vdec_setbuffer_cmd setbuffer;
		DBG("VDEC_IOCTL_FREE_BUFFER\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&setbuffer, vdec_msg.in, sizeof(setbuffer)))
			return -EFAULT;

		result = vid_dec_free_buffer(client_ctx, &setbuffer);

		if (!result)
			return -EIO;
		break;
	}
	case VDEC_IOCTL_CMD_START:
		DBG("VDEC_IOCTL_CMD_START\n");
		result = vid_dec_start(client_ctx);

		if (!result)
			return -EIO;
		break;
	case VDEC_IOCTL_CMD_STOP:
		DBG("VDEC_IOCTL_CMD_STOP\n");
		result = vid_dec_stop(client_ctx);

		if (!result)
			return -EIO;
		break;
	case VDEC_IOCTL_CMD_PAUSE:
		DBG("VDEC_IOCTL_CMD_PAUSE\n");
		result = vid_dec_pause_resume(client_ctx, true);

		if (!result)
			return -EIO;
		break;
	case VDEC_IOCTL_CMD_RESUME:
		DBG("VDEC_IOCTL_CMD_RESUME\n");
		result = vid_dec_pause_resume(client_ctx, false);

		if (!result)
			return -EIO;
		break;
	case VDEC_IOCTL_DECODE_FRAME:
	{
		struct vdec_input_frameinfo frm_info;
		DBG("VDEC_IOCTL_DECODE_FRAME\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&frm_info, vdec_msg.in, sizeof(frm_info)))
			return -EFAULT;

		result = vid_dec_decode_frame(client_ctx, &frm_info);

		if (!result)
			return -EIO;
		break;
	}
	case VDEC_IOCTL_FILL_OUTPUT_BUFFER:
	{
		struct vdec_fillbuffer_cmd fill_cmd;

		DBG("VDEC_IOCTL_FILL_OUTPUT_BUFFER\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&fill_cmd, vdec_msg.in, sizeof(fill_cmd)))
			return -EFAULT;
		result = vid_dec_fill_output_buffer(client_ctx, &fill_cmd);
		if (!result)
			return -EIO;
		break;
	}
	case VDEC_IOCTL_CMD_FLUSH:
	{
		enum vdec_bufferflush flush_dir;
		DBG("VDEC_IOCTL_CMD_FLUSH\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_from_user(&flush_dir, vdec_msg.in, sizeof(flush_dir)))
			return -EFAULT;

		result = vid_dec_flush(client_ctx, flush_dir);

		if (!result)
			return -EIO;
		break;
	}
	case VDEC_IOCTL_GET_NEXT_MSG:
	{
		struct vdec_msginfo msg_info;
		DBG("VDEC_IOCTL_GET_NEXT_MSG\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		result = vid_dec_get_next_msg(client_ctx, &msg_info);

		if (!result)
			return -EIO;
		if (copy_to_user(vdec_msg.out, &msg_info, sizeof(msg_info)))
			return -EFAULT;
		break;
	}
	case VDEC_IOCTL_STOP_NEXT_MSG:
		DBG("VDEC_IOCTL_STOP_NEXT_MSG\n");
		client_ctx->stop_msg = 1;
		wake_up(&client_ctx->msg_wait);
		break;
	case VDEC_IOCTL_SET_SEQUENCE_HEADER:
	{
		struct vdec_seqheader vdec_seq_hdr;
		struct vcd_sequence_hdr vcd_seq_hdr;
		unsigned long sz;
		DBG("VDEC_IOCTL_SET_SEQUENCE_HEADER\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg))) {
			ERR("Copy from user vdec_msg failed\n");
			return -EFAULT;
		}
		if (copy_from_user(&vdec_seq_hdr, vdec_msg.in,
				sizeof(vdec_seq_hdr))) {
			ERR("Copy from user seq_header failed\n");
			return -EFAULT;
		}
		if (!vdec_seq_hdr.sz) {
			ERR("Seq len is zero\n");
			return -EFAULT;
		}

		if (get_pmem_file(vdec_seq_hdr.pmem_fd,
				(unsigned long *)&phys_addr,
				(unsigned long *)&kern_addr, &sz, &pmem_file)) {
			ERR("%s: get_pmem_file failed\n", __func__);
			return false;
		}
		put_pmem_file(pmem_file);

		vcd_seq_hdr.sz = vdec_seq_hdr.sz;
		kern_addr = (u8 *)kern_addr + vdec_seq_hdr.pmem_offset;
		vcd_seq_hdr.addr = kern_addr;
		if (!vcd_seq_hdr.addr) {
			ERR("Sequence Header pointer failed\n");
			return -EFAULT;
		}
		client_ctx->seq_header_set = true;
		if (vcd_decode_start(client_ctx->vcd_handle, &vcd_seq_hdr)) {
			ERR("Decode start Failed\n");
			client_ctx->seq_header_set = false;
			return -EFAULT;
		}
		DBG("Wait Client completion Sequence Header\n");
		wait_for_completion(&client_ctx->event);
		vcd_seq_hdr.addr = NULL;
		if (client_ctx->event_status) {
			ERR("Set Seq Header status is failed");
			return -EFAULT;
		}
		break;
	}
	case VDEC_IOCTL_GET_NUMBER_INSTANCES:
		DBG("VDEC_IOCTL_GET_NUMBER_INSTANCES\n");
		if (copy_from_user(&vdec_msg, u_arg, sizeof(vdec_msg)))
			return -EFAULT;
		if (copy_to_user(vdec_msg.out, &vidc_dec_dev->num_clients,
				sizeof(vidc_dec_dev->num_clients)))
			return -EFAULT;
		break;
	default:
		ERR("%s(): Unsupported ioctl\n", __func__);
		return -ENOTTY;
	}

	return 0;
}

static u32 vid_dec_close_client(struct video_client_ctx *client_ctx)
{
	u32 vcd_status;
	int rc;

	INFO("msm_vidc_dec: Inside %s\n", __func__);
	if (!client_ctx || !client_ctx->vcd_handle) {
		ERR("Invalid client_ctx\n");
		return false;
	}

	mutex_lock(&vidc_dec_dev->lock);
	vcd_status = vcd_stop(client_ctx->vcd_handle);

	if (!vcd_status) {
		rc = wait_for_completion_timeout(&client_ctx->event,
			(5 * HZ) / 10);
		if (!rc)
			DBG("%s:ERROR vcd_stop time out rc = %d\n", __func__,
				rc);

		if (client_ctx->event_status)
			ERR("%s:ERROR vcd_stop event_status failure\n",
				__func__);
	}
	vcd_status = vcd_close(client_ctx->vcd_handle);

	if (vcd_status) {
		mutex_unlock(&vidc_dec_dev->lock);
		return false;
	}
	memset((void *)client_ctx, 0, sizeof(*client_ctx));
	vidc_dec_dev->num_clients--;
	mutex_unlock(&vidc_dec_dev->lock);
	return true;
}

static int vid_dec_open(struct inode *inode, struct file *file)
{
	int rc;
	s32 client_index;
	struct video_client_ctx *client_ctx;
	u32 vcd_status = VCD_ERR_FAIL;

	INFO("msm_vidc_dec: Inside %s\n", __func__);
	mutex_lock(&vidc_dec_dev->lock);

	if (vidc_dec_dev->num_clients == VID_DEC_MAX_DECODER_CLIENTS) {
		ERR("%s: ERROR: max number of clients limit reached\n",
			__func__);
		mutex_unlock(&vidc_dec_dev->lock);
		return -ENODEV;
	}

#ifndef USE_RES_TRACKER
	DBG("Resource Tracker not in use");
	if (!vid_c_enable_clk(VID_C_HCLK_RATE)) {
		ERR("%s: ERROR: clock enabled failed\n", __func__);
		mutex_unlock(&vidc_dec_dev->lock);
		return -ENODEV;
	}
#endif

	DBG("Virtual Address of ioremap is %p\n", vidc_dec_dev->virt_base);

	if (!vidc_dec_dev->num_clients) {
		rc = vcd_fw_prepare_all();
		if (rc)
			return rc;
	}

	client_index = vid_dec_get_empty_client_index();
	if (client_index == -1) {
		ERR("%s: No free clients client_index == -1\n", __func__);
		return -ENODEV;
	}
	client_ctx = &vidc_dec_dev->vdec_clients[client_index];
	vidc_dec_dev->num_clients++;
	init_completion(&client_ctx->event);
	mutex_init(&client_ctx->msg_queue_lock);
	INIT_LIST_HEAD(&client_ctx->msg_queue);
	init_waitqueue_head(&client_ctx->msg_wait);
	client_ctx->stop_msg = 0;

	vcd_status = vcd_open(vidc_dec_dev->device_handle, true,
		vid_dec_vcd_cb, client_ctx);
	wait_for_completion(&client_ctx->event);
	client_ctx->seq_header_set = false;
	file->private_data = client_ctx;
	mutex_unlock(&vidc_dec_dev->lock);
	return 0;
}

static int vid_dec_release(struct inode *inode, struct file *file)
{
	struct video_client_ctx *client_ctx = file->private_data;

	INFO("msm_vidc_dec: Inside %s\n", __func__);
	vid_dec_close_client(client_ctx);
#ifndef USE_RES_TRACKER
	vid_c_disable_clk();
#endif
	INFO("msm_vidc_dec: Return from %s\n", __func__);
	return 0;
}

static const struct file_operations vid_dec_fops = {
	.owner = THIS_MODULE,
	.open = vid_dec_open,
	.release = vid_dec_release,
	.ioctl = vid_dec_ioctl,
};

void vid_dec_interrupt_deregister(void)
{
}

void vid_dec_interrupt_register(void *device_name)
{
}

void vid_dec_interrupt_clear(void)
{
}

void *vid_dec_map_dev_base_addr(void *device_name)
{
	return vidc_dec_dev->virt_base;
}

static int vid_dec_vcd_init(void)
{
	int rc;
	struct vcd_init_config vcd_init_config;
	u32 i;

	/* init_timer(&hw_timer); */
	INFO("msm_vidc_dec: Inside %s\n", __func__);
	vidc_dec_dev->num_clients = 0;

	for (i = 0; i < VID_DEC_MAX_DECODER_CLIENTS; i++) {
		memset((void *)&vidc_dec_dev->vdec_clients[i], 0,
			sizeof(vidc_dec_dev->vdec_clients[i]));
	}

	mutex_init(&vidc_dec_dev->lock);
	vidc_dec_dev->virt_base = vid_c_get_ioaddr();
	DBG("%s: base address for VIDC core %p\n", __func__,
		vidc_dec_dev->virt_base);

	if (!vidc_dec_dev->virt_base) {
		ERR("%s: ioremap failed\n", __func__);
		return -ENOMEM;
	}

	vcd_init_config.device_name = "VID_C";
	vcd_init_config.pf_map_dev_base_addr = vid_dec_map_dev_base_addr;
	vcd_init_config.pf_interrupt_clr = vid_dec_interrupt_clear;
	vcd_init_config.pf_register_isr = vid_dec_interrupt_register;
	vcd_init_config.pf_deregister_isr = vid_dec_interrupt_deregister;
	vcd_init_config.pf_timer_create = vid_c_timer_create;
	vcd_init_config.pf_timer_release = vid_c_timer_release;
	vcd_init_config.pf_timer_start = vid_c_timer_start;
	vcd_init_config.pf_timer_stop = vid_c_timer_stop;

	rc = vcd_init(&vcd_init_config, &vidc_dec_dev->device_handle);
	if (rc) {
		ERR("%s: vcd_init failed\n", __func__);
		return -ENODEV;
	}
	return 0;
}

static int __init vid_dec_init(void)
{
	int rc = 0;
	struct device *class_devp;

	INFO("msm_vidc_dec: Inside %s\n", __func__);
	vidc_dec_dev = kzalloc(sizeof(*vidc_dec_dev), GFP_KERNEL);
	if (!vidc_dec_dev) {
		ERR("%s Unable to allocate memory for vid_dec_dev\n", __func__);
		return -ENOMEM;
	}

	rc = alloc_chrdev_region(&vidc_dec_dev_num, 0, 1, VID_DEC_NAME);
	if (rc < 0) {
		ERR("%s: alloc_chrdev_region failed rc = %d\n",	__func__, rc);
		goto error_vid_dec_alloc_chrdev_region;
	}

	vidc_dec_class = class_create(THIS_MODULE, VID_DEC_NAME);
	if (IS_ERR(vidc_dec_class)) {
		rc = PTR_ERR(vidc_dec_class);
		ERR("%s: couldn't create vid_dec_class %d\n", __func__, rc);
		goto error_vid_dec_class_create;
	}

	class_devp = device_create(vidc_dec_class, NULL, vidc_dec_dev_num, NULL,
		VID_DEC_NAME);

	if (IS_ERR(class_devp)) {
		rc = PTR_ERR(class_devp);
		ERR("%s: class device_create failed %d\n", __func__, rc);
		goto error_vid_dec_class_device_create;
	}

	vidc_dec_dev->device = class_devp;

	cdev_init(&vidc_dec_dev->cdev, &vid_dec_fops);
	vidc_dec_dev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&vidc_dec_dev->cdev, vidc_dec_dev_num, 1);

	if (rc < 0) {
		ERR("%s: cdev_add failed %d\n", __func__, rc);
		goto error_vid_dec_cdev_add;
	}

	return vid_dec_vcd_init();

error_vid_dec_cdev_add:
	device_destroy(vidc_dec_class, vidc_dec_dev_num);
error_vid_dec_class_device_create:
	class_destroy(vidc_dec_class);
error_vid_dec_class_create:
	unregister_chrdev_region(vidc_dec_dev_num, 1);
error_vid_dec_alloc_chrdev_region:
	kfree(vidc_dec_dev);

	return rc;
}

static void __exit vid_dec_exit(void)
{
	INFO("msm_vidc_dec: Inside %s\n", __func__);
	cdev_del(&(vidc_dec_dev->cdev));
	device_destroy(vidc_dec_class, vidc_dec_dev_num);
	class_destroy(vidc_dec_class);
	unregister_chrdev_region(vidc_dec_dev_num, 1);
	kfree(vidc_dec_dev);
	INFO("msm_vidc_dec: Return from %s\n", __func__);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Video decoder driver");
MODULE_VERSION("1.0");

module_init(vid_dec_init);
module_exit(vid_dec_exit);

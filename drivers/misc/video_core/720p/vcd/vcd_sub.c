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

#include <asm/div64.h>

#include "video_core_type.h"
#include "vcd.h"
#include "vdec_internal.h"

static phys_addr_t vcd_pmem_get_physical(struct video_client_ctx *client_ctx,
	void *kern_addr)
{
	phys_addr_t phys_addr;
	void __user *user_addr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;

	if (vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_INPUT,
			false, &user_addr, &kern_addr, &phys_addr, &pmem_fd,
			&file, &buffer_index)) {
		return phys_addr;
	}
	if (vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT,
			false, &user_addr, &kern_addr, &phys_addr, &pmem_fd,
			&file, &buffer_index)) {
		return phys_addr;
	}
	VCD_MSG_ERROR("Couldn't get physical address");
	return 0;
}

void vcd_reset_device_channels(struct vcd_dev_ctxt *dev_ctxt)
{
	dev_ctxt->ddl_frame_ch_free = dev_ctxt->ddl_frame_ch_depth;
	dev_ctxt->ddl_cmd_ch_free = dev_ctxt->ddl_cmd_ch_depth;
	dev_ctxt->ddl_frame_ch_interim = 0;
	dev_ctxt->ddl_cmd_ch_interim = 0;
}

u32 vcd_get_command_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc **pp_transc)
{
	u32 result = false;

	*pp_transc = NULL;

	if (dev_ctxt->ddl_cmd_ch_free > 0) {
		if (dev_ctxt->ddl_cmd_concurrency) {
			--dev_ctxt->ddl_cmd_ch_free;
			result = true;
		} else if ((dev_ctxt->ddl_frame_ch_free +
				dev_ctxt->ddl_frame_ch_interim)	==
				dev_ctxt->ddl_frame_ch_depth) {
			--dev_ctxt->ddl_cmd_ch_free;
			result = true;
		}
	}

	if (result) {
		*pp_transc = vcd_get_free_trans_tbl_entry(dev_ctxt);

		if (!*pp_transc) {
			result = false;

			vcd_release_command_channel(dev_ctxt, *pp_transc);
		}

	}
	return result;
}

u32 vcd_get_command_channel_in_loop(struct vcd_dev_ctxt *dev_ctxt,
	 struct vcd_transc **pp_transc)
{
	u32 result = false;

	*pp_transc = NULL;

	if (dev_ctxt->ddl_cmd_ch_interim > 0) {
		if (dev_ctxt->ddl_cmd_concurrency) {
			--dev_ctxt->ddl_cmd_ch_interim;
			result = true;
		} else if ((dev_ctxt->ddl_frame_ch_free +
				dev_ctxt->ddl_frame_ch_interim)
				== dev_ctxt->ddl_frame_ch_depth) {
			--dev_ctxt->ddl_cmd_ch_interim;
			result = true;
		}
	} else {
		result = vcd_get_command_channel(dev_ctxt, pp_transc);
	}

	if (result && !*pp_transc) {
		*pp_transc = vcd_get_free_trans_tbl_entry(dev_ctxt);

		if (!*pp_transc) {
			result = false;

			++dev_ctxt->ddl_cmd_ch_interim;
		}
	}

	return result;
}

void vcd_mark_command_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc)
{
	++dev_ctxt->ddl_cmd_ch_interim;

	vcd_release_trans_tbl_entry(transc);
	if (dev_ctxt->ddl_cmd_ch_interim + dev_ctxt->ddl_cmd_ch_free >
			dev_ctxt->ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_command_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc)
{
	++dev_ctxt->ddl_cmd_ch_free;

	vcd_release_trans_tbl_entry(transc);
	if (dev_ctxt->ddl_cmd_ch_interim + dev_ctxt->ddl_cmd_ch_free >
			dev_ctxt->ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_multiple_command_channels(struct vcd_dev_ctxt *dev_ctxt,
	u32 channels)
{
	dev_ctxt->ddl_cmd_ch_free += channels;

	if (dev_ctxt->ddl_cmd_ch_interim + dev_ctxt->ddl_cmd_ch_free >
			dev_ctxt->ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_interim_command_channels(struct vcd_dev_ctxt *dev_ctxt)
{
	dev_ctxt->ddl_cmd_ch_free += dev_ctxt->ddl_cmd_ch_interim;
	dev_ctxt->ddl_cmd_ch_interim = 0;

	if (dev_ctxt->ddl_cmd_ch_interim + dev_ctxt->ddl_cmd_ch_free >
			dev_ctxt->ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

u32 vcd_get_frame_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc **pp_transc)
{
	u32 result = false;

	if (dev_ctxt->ddl_frame_ch_free > 0) {
		if (dev_ctxt->ddl_cmd_concurrency) {
			--dev_ctxt->ddl_frame_ch_free;
			result = true;
		} else if ((dev_ctxt->ddl_cmd_ch_free +
				dev_ctxt->ddl_cmd_ch_interim)
				== dev_ctxt->ddl_cmd_ch_depth) {
			--dev_ctxt->ddl_frame_ch_free;
			result = true;
		}
	}

	if (result) {
		*pp_transc = vcd_get_free_trans_tbl_entry(dev_ctxt);

		if (!*pp_transc) {
			result = false;

			vcd_release_frame_channel(dev_ctxt, *pp_transc);
		} else {
			(*pp_transc)->type = VCD_CMD_CODE_FRAME;
		}

	}

	return result;
}

u32 vcd_get_frame_channel_in_loop(struct vcd_dev_ctxt *dev_ctxt,
	 struct vcd_transc **pp_transc)
{
	u32 result = false;

	*pp_transc = NULL;

	if (dev_ctxt->ddl_frame_ch_interim > 0) {
		if (dev_ctxt->ddl_cmd_concurrency) {
			--dev_ctxt->ddl_frame_ch_interim;
			result = true;
		} else if ((dev_ctxt->ddl_cmd_ch_free +
				dev_ctxt->ddl_cmd_ch_interim) ==
				dev_ctxt->ddl_cmd_ch_depth) {
			--dev_ctxt->ddl_frame_ch_interim;
			result = true;
		}
	} else {
		result = vcd_get_frame_channel(dev_ctxt, pp_transc);
	}

	if (result && !*pp_transc) {
		*pp_transc = vcd_get_free_trans_tbl_entry(dev_ctxt);

		if (!*pp_transc) {
			result = false;
			VCD_MSG_FATAL("\n%s: All transactions are busy;"
				"Couldnt find free one\n", __func__);
			++dev_ctxt->ddl_frame_ch_interim;
		}

	}

	return result;
}

void vcd_mark_frame_channel(struct vcd_dev_ctxt *dev_ctxt)
{
	++dev_ctxt->ddl_frame_ch_interim;

	if (dev_ctxt->ddl_frame_ch_interim + dev_ctxt->ddl_frame_ch_free >
			dev_ctxt->ddl_cmd_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_frame_channel(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc)
{
	++dev_ctxt->ddl_frame_ch_free;

	vcd_release_trans_tbl_entry(transc);

	if (dev_ctxt->ddl_frame_ch_interim + dev_ctxt->ddl_frame_ch_free >
			dev_ctxt->ddl_cmd_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_multiple_frame_channels(struct vcd_dev_ctxt
	*dev_ctxt, u32 channels)
{
	dev_ctxt->ddl_frame_ch_free += channels;

	if (dev_ctxt->ddl_frame_ch_interim + dev_ctxt->ddl_frame_ch_free >
			dev_ctxt->ddl_frame_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_interim_frame_channels(struct vcd_dev_ctxt
	*dev_ctxt)
{
	dev_ctxt->ddl_frame_ch_free += dev_ctxt->ddl_frame_ch_interim;
	dev_ctxt->ddl_frame_ch_interim = 0;

	if (dev_ctxt->ddl_frame_ch_free > dev_ctxt->ddl_cmd_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

u32 vcd_core_is_busy(struct vcd_dev_ctxt *dev_ctxt)
{
	if (((dev_ctxt->ddl_cmd_ch_free + dev_ctxt->ddl_cmd_ch_interim) !=
			dev_ctxt->ddl_cmd_ch_depth) ||
			((dev_ctxt->ddl_frame_ch_free +
			dev_ctxt->ddl_frame_ch_interim) !=
			dev_ctxt->ddl_frame_ch_depth)) {
		return true;
	} else {
		return false;
	}
}

void vcd_device_timer_start(struct vcd_dev_ctxt *dev_ctxt)
{
	if (dev_ctxt->config.pf_timer_start)
		dev_ctxt->config.pf_timer_start(dev_ctxt->hw_timer_handle,
			dev_ctxt->hw_time_out);
}

void vcd_device_timer_stop(struct vcd_dev_ctxt *dev_ctxt)
{
	if (dev_ctxt->config.pf_timer_stop)
		dev_ctxt->config.pf_timer_stop(dev_ctxt->hw_timer_handle);
}


u32 vcd_common_allocate_set_buffer(struct vcd_clnt_ctxt *cctxt,
	 enum vcd_buffer_type buffer, size_t sz,
	 struct vcd_buffer_pool **pp_buf_pool)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_requirement buf_req;
	struct vcd_property_hdr prop_hdr;
	struct vcd_buffer_pool *buf_pool;

	if (buffer == VCD_BUFFER_INPUT) {
		prop_hdr.id = DDL_I_INPUT_BUF_REQ;
		buf_pool = &cctxt->in_buf_pool;
	} else if (buffer == VCD_BUFFER_OUTPUT) {
		prop_hdr.id = DDL_I_OUTPUT_BUF_REQ;
		buf_pool = &cctxt->out_buf_pool;
	} else {
		rc = VCD_ERR_ILLEGAL_PARM;
	}

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided");

	*pp_buf_pool = buf_pool;

	if (buf_pool->count > 0 && buf_pool->validated == buf_pool->count) {
		VCD_MSG_ERROR("Buffer pool is full");

		return VCD_ERR_FAIL;
	}

	if (!buf_pool->entries) {
		prop_hdr.sz = sizeof(buf_req);
		rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &buf_req);

		if (!VCD_FAILED(rc))
			rc = vcd_alloc_buffer_pool_entries(buf_pool, &buf_req);
		else
			VCD_MSG_ERROR("rc = 0x%x. Failed ddl_get_property", rc);
	}

	if (!VCD_FAILED(rc)) {
		if (buf_pool->buf_req.size > sz) {
			VCD_MSG_ERROR("required buffer size %u allocated size "
				"%u", buf_pool->buf_req.size, sz);
			rc = VCD_ERR_ILLEGAL_PARM;
		}
	}

	return rc;
}

u32 vcd_set_buffer_internal(struct vcd_clnt_ctxt *cctxt,
	struct vcd_buffer_pool *buf_pool, void *buf, size_t sz)
{
	struct vcd_buffer_entry *buf_entry;

	buf_entry = vcd_find_buffer_pool_entry(buf_pool, buf);
	if (buf_entry) {
		VCD_MSG_ERROR("This buffer address already exists");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!IS_ALIGNED((unsigned long)buf, buf_pool->buf_req.align)) {
		VCD_MSG_ERROR("Provided addr is not aligned");
		return VCD_ERR_BAD_POINTER;
	}

	buf_entry = vcd_get_free_buffer_pool_entry(buf_pool);
	if (!buf_entry) {
		VCD_MSG_ERROR("Can't allocate buffer pool is full");
		return VCD_ERR_FAIL;
	}

	printk("npelly adding %p to buf_pool %p\n", buf, buf_entry);
	buf_entry->virt_addr = buf;

	buf_entry->phys_addr = vcd_pmem_get_physical(cctxt->client_data, buf);

	if (!buf_entry->phys_addr) {
		VCD_MSG_ERROR("Couldn't get physical address");
		return VCD_ERR_BAD_POINTER;
	}

	if (!IS_ALIGNED((unsigned long)buf_entry->phys_addr,
			buf_pool->buf_req.align)) {
		VCD_MSG_ERROR("Physical addr is not aligned");
		return VCD_ERR_BAD_POINTER;
	}

	buf_entry->size = sz;
	buf_entry->frame.alloc_len = sz;
	buf_entry->allocated = false;

	buf_entry->frame.virt_addr = buf_entry->virt_addr;
	buf_entry->frame.phys_addr = buf_entry->phys_addr;

	buf_pool->validated++;

	return VCD_S_SUCCESS;

}

u32 vcd_allocate_buffer_internal(struct vcd_clnt_ctxt *cctxt,
	struct vcd_buffer_pool *buf_pool, size_t buf_size, void **virt_addr,
	phys_addr_t *phys_addr)
{
	struct vcd_buffer_entry *buf_entry;
	struct vcd_buffer_requirement *buf_req;
//	u32 addr;
//	int rc = 0;

	buf_entry = vcd_get_free_buffer_pool_entry(buf_pool);
	if (!buf_entry) {
		VCD_MSG_ERROR("Can't allocate buffer pool is full");
		return VCD_ERR_FAIL;
	}

	buf_req = &buf_pool->buf_req;

	//TODO strip align crap
//	buf_size += buf_req->align;

	buf_entry->buffer.virt_addr = dma_alloc_coherent(NULL, buf_size,
		&buf_entry->buffer.phys_addr, GFP_KERNEL);
	if (!buf_entry->buffer.virt_addr) {
		VCD_MSG_ERROR("Buffer allocation failed");
		return VCD_ERR_ALLOC_FAIL;
	}

	buf_entry->buffer.size = buf_size;
	buf_entry->allocated = true;

	buf_entry->frame.alloc_len = buf_entry->buffer.size;
	buf_entry->frame.virt_addr = buf_entry->buffer.virt_addr;
	buf_entry->frame.phys_addr = buf_entry->buffer.phys_addr;

	*virt_addr = buf_entry->buffer.virt_addr;
	*phys_addr = buf_entry->buffer.phys_addr;

	buf_pool->allocated++;
	buf_pool->validated++;

	return VCD_S_SUCCESS;
}

u32 vcd_free_one_buffer_internal(struct vcd_clnt_ctxt *cctxt,
	 enum vcd_buffer_type vcd_buffer_type, u8 *buffer)
{
	struct vcd_buffer_pool *buf_pool;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry *buf_entry;

	if (vcd_buffer_type == VCD_BUFFER_INPUT)
		buf_pool = &cctxt->in_buf_pool;
	else if (vcd_buffer_type == VCD_BUFFER_OUTPUT)
		buf_pool = &cctxt->out_buf_pool;
	else
		rc = VCD_ERR_ILLEGAL_PARM;

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided");

	buf_entry = vcd_find_buffer_pool_entry(buf_pool, buffer);
	if (!buf_entry) {
		VCD_MSG_ERROR("Buffer addr %p not found. Can't free buffer",
				  buffer);

		return VCD_ERR_ILLEGAL_PARM;
	}
	if (buf_entry->in_use) {
		VCD_MSG_ERROR("\n Buffer is in use and is not flushed");
		return VCD_ERR_ILLEGAL_OP;
	}

	VCD_MSG_LOW("Freeing buffer %p. Allocated %d", buf_entry->virt_addr,
		buf_entry->allocated);

	if (buf_entry->allocated) {
		dma_free_coherent(NULL, buf_entry->size, buf_entry->virt_addr,
			buf_entry->phys_addr);
		buf_entry->virt_addr = NULL;
		buf_pool->allocated--;

	}

	memset(buf_entry, 0, sizeof(struct vcd_buffer_entry));

	buf_pool->validated--;

	return VCD_S_SUCCESS;
}

u32 vcd_free_buffers_internal(struct vcd_clnt_ctxt *cctxt,
	 struct vcd_buffer_pool *buf_pool)
{
	u32 rc = VCD_S_SUCCESS;
	u32 i;

	VCD_MSG_LOW("vcd_free_buffers_internal:");

	if (!buf_pool->entries)
		return rc;

	for (i = 1; i <= buf_pool->count; i++) {
		struct vcd_buffer_entry *b = &buf_pool->entries[i];
		if (!b->valid || !b->allocated)
			continue;
		dma_free_coherent(NULL, b->size, b->virt_addr, b->phys_addr);
	}

	vcd_reset_buffer_pool_for_reuse(buf_pool);

	return rc;
}

u32 vcd_alloc_buffer_pool_entries(struct vcd_buffer_pool *buf_pool,
	struct vcd_buffer_requirement *buf_req)
{

	VCD_MSG_LOW("vcd_alloc_buffer_pool_entries:");

	buf_pool->buf_req = *buf_req;

	buf_pool->count = buf_req->actual_count;
	buf_pool->entries = kzalloc(sizeof(struct vcd_buffer_entry) *
		(buf_pool->count + 1), GFP_KERNEL);

	if (!buf_pool->entries) {
		VCD_MSG_ERROR("Buf_pool entries alloc failed");
		return VCD_ERR_ALLOC_FAIL;
	}

	buf_pool->queue = kzalloc(sizeof(struct vcd_buffer_entry *) *
		buf_pool->count, GFP_KERNEL);

	if (!buf_pool->queue) {
		VCD_MSG_ERROR("Buf_pool queue alloc failed");
		kfree(buf_pool->entries);
		return VCD_ERR_ALLOC_FAIL;
	}

	buf_pool->entries[0].valid = true;

	buf_pool->q_head = 0;
	buf_pool->q_tail = (u16) (buf_pool->count - 1);
	buf_pool->q_len = 0;

	buf_pool->validated = 0;
	buf_pool->allocated = 0;
	buf_pool->in_use = 0;

	return VCD_S_SUCCESS;
}

void vcd_free_buffer_pool_entries(struct vcd_buffer_pool *buf_pool)
{
	VCD_MSG_LOW("vcd_free_buffer_pool_entries:");

	kfree(buf_pool->entries);
	kfree(buf_pool->queue);

	memset(buf_pool, 0, sizeof(struct vcd_buffer_pool));
}

void vcd_flush_in_use_buffer_pool_entries(struct vcd_clnt_ctxt *cctxt,
	struct vcd_buffer_pool *buf_pool, u32 event)
{
	u32 i;
	VCD_MSG_LOW("vcd_flush_buffer_pool_entries: event=0x%x", event);

	if (!buf_pool->entries)
		return;

	for (i = 0; i <= buf_pool->count; i++) {
		if (buf_pool->entries[i].virt_addr &&
				buf_pool->entries[i].in_use) {
			cctxt->callback(event, VCD_S_SUCCESS,
				&buf_pool->entries[i].frame,
				sizeof(struct vcd_frame_data), cctxt,
				cctxt->client_data);
			buf_pool->entries[i].in_use = false;
			VCD_BUFFERPOOL_INUSE_DECREMENT(buf_pool->in_use);
		}
	}
}


void vcd_reset_buffer_pool_for_reuse(struct vcd_buffer_pool *buf_pool)
{
	VCD_MSG_LOW("vcd_reset_buffer_pool_for_reuse:");

	memset(&buf_pool->entries[1], 0, sizeof(struct vcd_buffer_entry) *
		buf_pool->count);
	memset(buf_pool->queue, 0, sizeof(struct vcd_buffer_entry *) *
		buf_pool->count);

	buf_pool->q_head = 0;
	buf_pool->q_tail = (u16) (buf_pool->count - 1);
	buf_pool->q_len = 0;

	buf_pool->validated = 0;
	buf_pool->allocated = 0;
	buf_pool->in_use = 0;

}

struct vcd_buffer_entry *vcd_get_free_buffer_pool_entry(struct vcd_buffer_pool
	*pool)
{
	int i;
	for (i = 1; i <= pool->count; i++) {
		if (!pool->entries[i].valid) {
			pool->entries[i].valid = true;
			return &pool->entries[i];
		}
	}
	return NULL;
}

struct vcd_buffer_entry *vcd_find_buffer_pool_entry(struct vcd_buffer_pool
	*pool, void *virt_addr)
{
	int i;
	for (i = 0; i <= pool->count; i++)
		if (pool->entries[i].virt_addr == virt_addr)
			return &pool->entries[i];
	return NULL;
}

u32 vcd_buffer_pool_entry_en_q(struct vcd_buffer_pool *pool,
	struct vcd_buffer_entry *entry)
{
	u16 i;
	u16 q_cntr;
	u32 found = false;

	if (pool->q_len == pool->count)
		return false;

	for (i = 0, q_cntr = pool->q_head; !found && i < pool->q_len;
			i++, q_cntr = (q_cntr + 1) % pool->count) {
		if (pool->queue[q_cntr] == entry)
			found = true;
	}

	if (found) {
		VCD_MSG_HIGH("this output buffer is already present in queue");
		VCD_MSG_HIGH("virt_addr %p phys_addr %x", entry->virt_addr,
			entry->phys_addr);
		return false;
	}

	pool->q_tail = (pool->q_tail + 1) % pool->count;
	pool->q_len++;
	pool->queue[pool->q_tail] = entry;

	return true;
}

struct vcd_buffer_entry *vcd_buffer_pool_entry_de_q(struct vcd_buffer_pool
	*pool)
{
	struct vcd_buffer_entry *entry;

	if (!pool || !pool->q_len)
		return NULL;

	entry = pool->queue[pool->q_head];
	pool->q_head = (pool->q_head + 1) % pool->count;
	pool->q_len--;

	return entry;
}

void vcd_flush_output_buffers(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_buffer_pool *buf_pool;
	struct vcd_buffer_entry *buf_entry;
	u32 count = 0;
	struct vcd_property_hdr prop_hdr;

	VCD_MSG_LOW("vcd_flush_output_buffers:");

	buf_pool = &cctxt->out_buf_pool;

	buf_entry = vcd_buffer_pool_entry_de_q(buf_pool);
	while (buf_entry) {
		if (!cctxt->decoding || buf_entry->in_use) {
			buf_entry->frame.data_len = 0;

			cctxt->callback(VCD_EVT_RESP_OUTPUT_FLUSHED,
				VCD_S_SUCCESS, &buf_entry->frame,
				sizeof(struct vcd_frame_data),
				cctxt, cctxt->client_data);

			buf_entry->in_use = false;

			count++;
		}

		buf_entry = vcd_buffer_pool_entry_de_q(buf_pool);
	}
	buf_pool->in_use = 0;

	if (cctxt->sched_clnt_valid && count > 0) {
		VCD_MSG_LOW("Updating scheduler O tkns = %u", count);

		sched_update_client_o_tkn(cctxt->dev_ctxt->sched_hdl,
			cctxt->sched_clnt_hdl, false,
			count * cctxt->sched_o_tkn_per_ip_frm);
	}

	if (cctxt->ddl_hdl_valid && cctxt->decoding) {
		prop_hdr.id = DDL_I_REQ_OUTPUT_FLUSH;
		prop_hdr.sz = sizeof(u32);
		count = 0x1;

		ddl_set_property(cctxt->ddl_handle, &prop_hdr, &count);
	}
}

u32 vcd_flush_buffers(struct vcd_clnt_ctxt *cctxt, u32 mode)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry *buf_entry;

	VCD_MSG_LOW("vcd_flush_buffers:");

	if (mode > VCD_FLUSH_ALL || !(mode & VCD_FLUSH_ALL)) {
		VCD_MSG_ERROR("Invalid flush mode %d", mode);

		return VCD_ERR_ILLEGAL_PARM;
	}

	VCD_MSG_MED("Flush mode %d requested", mode);

	if ((mode & VCD_FLUSH_INPUT) && cctxt->sched_clnt_valid) {
		rc = vcd_map_sched_status(sched_flush_client_buffer(
			dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
			(void **)&buf_entry));

		while (!VCD_FAILED(rc) && rc != VCD_S_SCHED_QEMPTY &&
				buf_entry) {
			if (buf_entry->virt_addr) {
				cctxt->callback(VCD_EVT_RESP_INPUT_FLUSHED,
					VCD_S_SUCCESS, &buf_entry->frame,
					sizeof(struct vcd_frame_data), cctxt,
					cctxt->client_data);
			}

			buf_entry->in_use = false;
			VCD_BUFFERPOOL_INUSE_DECREMENT(
				cctxt->in_buf_pool.in_use);
			buf_entry = NULL;
			rc = vcd_map_sched_status(sched_flush_client_buffer(
				dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
				(void **)&buf_entry));
		}

	}
	VCD_FAILED_RETURN(rc, "Failed: sched_flush_client_buffer");

	if (cctxt->status.frame_submitted > 0) {
		cctxt->status.flush_mode |= mode;
	} else {
		if (mode & VCD_FLUSH_OUTPUT) {
			vcd_flush_output_buffers(cctxt);
			vcd_release_all_clnt_frm_transc(cctxt);
		}

	}

	return VCD_S_SUCCESS;
}

void vcd_flush_buffers_in_err_fatal(struct vcd_clnt_ctxt *cctxt)
{
	VCD_MSG_LOW("\n vcd_flush_buffers_in_err_fatal:");
	vcd_flush_buffers(cctxt, VCD_FLUSH_ALL);
	vcd_flush_in_use_buffer_pool_entries(cctxt, &cctxt->in_buf_pool,
		VCD_EVT_RESP_INPUT_FLUSHED);
	vcd_flush_in_use_buffer_pool_entries(cctxt, &cctxt->out_buf_pool,
		VCD_EVT_RESP_OUTPUT_FLUSHED);
	cctxt->status.flush_mode = VCD_FLUSH_ALL;
	vcd_send_flush_done(cctxt, VCD_S_SUCCESS);
}

u32 vcd_init_client_context(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc;

	VCD_MSG_LOW("vcd_init_client_context:");

	rc = ddl_open(&cctxt->ddl_handle, cctxt->decoding);

	VCD_FAILED_RETURN(rc, "Failed: ddl_open");
	cctxt->ddl_hdl_valid = true;

	cctxt->clnt_state.state = VCD_CLIENT_STATE_OPEN;
	cctxt->clnt_state.state_table =	vcd_get_client_state_table(
		VCD_CLIENT_STATE_OPEN);

	cctxt->signature = VCD_SIGNATURE;
	cctxt->live = true;

	cctxt->cmd_q.pending_cmd = VCD_CMD_NONE;

	return rc;
}

void vcd_destroy_client_context(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_dev_ctxt *dev_ctxt;
	struct vcd_clnt_ctxt *client;
	u32 rc = VCD_S_SUCCESS;
	int idx;

	VCD_MSG_LOW("vcd_destroy_client_context:");

	dev_ctxt = cctxt->dev_ctxt;

	if (cctxt == dev_ctxt->cctxt_list_head) {
		VCD_MSG_MED("Clnt list head clnt being removed");

		dev_ctxt->cctxt_list_head = cctxt->next;
	} else {
		client = dev_ctxt->cctxt_list_head;
		while (client && cctxt != client->next)
			client = client->next;

		if (client)
			client->next = cctxt->next;

		if (!client) {
			rc = VCD_ERR_FAIL;

			VCD_MSG_ERROR("Client not found in client list");
		}
	}

	if (VCD_FAILED(rc))
		return;

	if (cctxt->sched_clnt_valid) {
		rc = VCD_S_SUCCESS;
		while (!VCD_FAILED(rc) && rc != VCD_S_SCHED_QEMPTY) {

			rc = vcd_map_sched_status(sched_flush_client_buffer(
				dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
				(void *)&idx));
			if (VCD_FAILED(rc))
				VCD_MSG_ERROR("\n Failed: "
					"sched_flush_client_buffer");
		}

		rc = vcd_map_sched_status(sched_remove_client(
			dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl));
		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("\n Failed: sched_remove_client");

		cctxt->sched_clnt_valid = false;
	}

	if (cctxt->seq_hdr.addr) {
		dma_free_coherent(NULL, cctxt->seq_hdr.sz, cctxt->seq_hdr.addr,
			cctxt->seq_hdr_phys_addr);
		cctxt->seq_hdr.addr = NULL;
	}

	vcd_free_buffers_internal(cctxt, &cctxt->in_buf_pool);
	vcd_free_buffers_internal(cctxt, &cctxt->out_buf_pool);
	vcd_free_buffer_pool_entries(&cctxt->in_buf_pool);
	vcd_free_buffer_pool_entries(&cctxt->out_buf_pool);
	vcd_release_all_clnt_transc(cctxt);

	if (cctxt->ddl_hdl_valid) {
		ddl_close(&cctxt->ddl_handle);
		cctxt->ddl_hdl_valid = false;
	}
	kfree(cctxt);
}

u32 vcd_check_for_client_context(struct vcd_dev_ctxt *dev_ctxt, s32 driver_id)
{
	struct vcd_clnt_ctxt *client;

	client = dev_ctxt->cctxt_list_head;
	while (client && client->driver_id != driver_id)
		client = client->next;

	if (!client)
		return false;
	else
		return true;
}

u32 vcd_validate_driver_handle(struct vcd_dev_ctxt *dev_ctxt, s32 driver_handle)
{
	driver_handle--;

	if (driver_handle < 0 || driver_handle >= VCD_DRIVER_INSTANCE_MAX ||
			!dev_ctxt->driver_ids[driver_handle]) {
		return false;
	} else {
		return true;
	}
}

u32 vcd_client_cmd_en_q(struct vcd_clnt_ctxt *cctxt,
	enum vcd_command_type command)
{
	u32 result;

	if (cctxt->cmd_q.pending_cmd == VCD_CMD_NONE) {
		cctxt->cmd_q.pending_cmd = command;
		result = true;
	} else {
		result = false;
	}

	return result;
}

void vcd_client_cmd_flush_and_en_q(struct vcd_clnt_ctxt *cctxt,
	enum vcd_command_type command)
{
	cctxt->cmd_q.pending_cmd = command;
}

u32 vcd_client_cmd_de_q(struct vcd_clnt_ctxt *cctxt,
	enum vcd_command_type *command)
{
	if (cctxt->cmd_q.pending_cmd == VCD_CMD_NONE)
		return false;

	*command = cctxt->cmd_q.pending_cmd;
	cctxt->cmd_q.pending_cmd = VCD_CMD_NONE;

	return true;
}

u32 vcd_get_next_queued_client_cmd(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt **cctxt, enum vcd_command_type *command)
{
	struct vcd_clnt_ctxt *client = dev_ctxt->cctxt_list_head;
	u32 result = false;

	while (client && !result) {
		*cctxt = client;
		result = vcd_client_cmd_de_q(client, command);
		client = client->next;
	}
	return result;
}

u32 vcd_map_sched_status(enum sched_status sched_status)
{
	u32 rc = VCD_S_SUCCESS;

	switch (sched_status) {

	case SCHED_S_OK:
		rc = VCD_S_SUCCESS;
		break;

	case SCHED_S_EOF:
		rc = VCD_S_SCHED_EOS;
		break;

	case SCHED_S_QEMPTY:
		rc = VCD_S_SCHED_QEMPTY;
		break;

	case SCHED_S_QFULL:
		rc = VCD_S_SCHED_QFULL;
		break;

	default:
		rc = VCD_ERR_FAIL;
		break;
	}

	return rc;
}

u32 vcd_submit_cmd_sess_start(struct vcd_transc *transc)
{
	u32 rc;
	struct vcd_phys_sequence_hdr seq_hdr;

	VCD_MSG_LOW("vcd_submit_cmd_sess_start:");

	if (transc->cctxt->decoding) {

		if (transc->cctxt->seq_hdr.addr) {
			seq_hdr.sz = transc->cctxt->seq_hdr.sz;
			seq_hdr.addr = transc->cctxt->seq_hdr_phys_addr;

			rc = ddl_decode_start(transc->cctxt->ddl_handle,
				&seq_hdr, (void *)transc);
		} else {
			rc = ddl_decode_start(transc->cctxt->ddl_handle, NULL,
				(void *)transc);
		}

	} else {
		rc = ddl_encode_start(transc->cctxt->ddl_handle,
			(void *)transc);
	}
	if (!VCD_FAILED(rc)) {
		transc->cctxt->status.cmd_submitted++;
		vcd_device_timer_start(transc->cctxt->dev_ctxt);
	} else
		VCD_MSG_ERROR("rc = 0x%x. Failed: ddl start", rc);

	return rc;
}

u32 vcd_submit_cmd_sess_end(struct vcd_transc *transc)
{
	u32 rc;

	VCD_MSG_LOW("vcd_submit_cmd_sess_end:");

	if (transc->cctxt->decoding) {
		rc = ddl_decode_end(transc->cctxt->ddl_handle,
			(void *)transc);
	} else {
		rc = ddl_encode_end(transc->cctxt->ddl_handle,
			(void *)transc);
	}
	if (!VCD_FAILED(rc)) {
		transc->cctxt->status.cmd_submitted++;
		vcd_device_timer_start(transc->cctxt->dev_ctxt);
	} else
		VCD_MSG_ERROR("rc = 0x%x. Failed: ddl end", rc);

	return rc;
}

void vcd_submit_cmd_client_close(struct vcd_clnt_ctxt *cctxt)
{
	ddl_close(&cctxt->ddl_handle);
	cctxt->ddl_hdl_valid = false;
	cctxt->status.cleaning_up = false;
	if (cctxt->status.close_pending) {
		vcd_destroy_client_context(cctxt);
		vcd_handle_for_last_clnt_close(cctxt->dev_ctxt, true);
	}
}

u32 vcd_submit_command_in_continue(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc)
{
	struct vcd_property_hdr prop_hdr;
	struct vcd_clnt_ctxt *client = NULL;
	enum vcd_command_type cmd = VCD_CMD_NONE;
	u32 rc = VCD_ERR_FAIL;
	u32 result = false;
	u32 flush = 0;
	u32 event = 0;

	VCD_MSG_LOW("\n vcd_submit_command_in_continue:");

	while (1) {
		result = vcd_get_next_queued_client_cmd(dev_ctxt, &client,
			&cmd);

		if (!result)
			break;

		transc->type = cmd;
		transc->cctxt = client;

		switch (cmd) {
		case VCD_CMD_CODEC_START:
			rc = vcd_submit_cmd_sess_start(transc);
			event = VCD_EVT_RESP_START;
			break;
		case VCD_CMD_CODEC_STOP:
			rc = vcd_submit_cmd_sess_end(transc);
			event = VCD_EVT_RESP_STOP;
			break;
		case VCD_CMD_OUTPUT_FLUSH:
			prop_hdr.id = DDL_I_REQ_OUTPUT_FLUSH;
			prop_hdr.sz = sizeof(u32);
			flush = 0x1;
			ddl_set_property(client->ddl_handle, &prop_hdr, &flush);
			vcd_release_command_channel(dev_ctxt, transc);
			rc = VCD_S_SUCCESS;
			break;
		case VCD_CMD_CLIENT_CLOSE:
			vcd_submit_cmd_client_close(client);
			vcd_release_command_channel(dev_ctxt, transc);
			rc = VCD_S_SUCCESS;
			break;
		default:
			VCD_MSG_ERROR("\n vcd_submit_command: Unknown"
				"command %d", (int)cmd);
			vcd_assert();
			break;
		}

		if (!VCD_FAILED(rc)) {
			break;
		} else	{
			VCD_MSG_ERROR("vcd_submit_command %d: failed 0x%x",
				cmd, rc);
			client->callback(event, rc, NULL, 0, client,
				client->client_data);
		}
	}
	return result;
}

u32 vcd_schedule_frame(struct vcd_dev_ctxt *dev_ctxt, struct vcd_clnt_ctxt
	**pp_cctxt, struct vcd_buffer_entry **pp_ip_buf_entry)
{
	u32 rc = VCD_S_SUCCESS;
	VCD_MSG_LOW("vcd_schedule_frame:");

	if (!dev_ctxt->cctxt_list_head) {
		VCD_MSG_HIGH("Client list empty");
		return false;
	}

	rc = vcd_map_sched_status(sched_de_queue_frame(dev_ctxt->sched_hdl,
		(void **) pp_ip_buf_entry, (void **) pp_cctxt));
	if (VCD_FAILED(rc)) {
		VCD_MSG_FATAL("vcd_submit_frame: sched_de_queue_frame"
			"failed 0x%x", rc);
	  return false;
	}

	if (rc == VCD_S_SCHED_QEMPTY) {
		VCD_MSG_HIGH("No frame available. Sched queues are empty");
		return false;
	}

	if (!*pp_cctxt || !*pp_ip_buf_entry) {
		VCD_MSG_FATAL("Sched returned invalid values. ctxt=%p,"
			"ipbuf=%p", *pp_cctxt, *pp_ip_buf_entry);
		return false;
	}

	if (rc == VCD_S_SCHED_EOS)
		(*pp_ip_buf_entry)->frame.flags |= VCD_FRAME_FLAG_EOS;

	return true;
}

void vcd_try_submit_frame(struct vcd_dev_ctxt *dev_ctxt)
{
	struct vcd_transc *transc;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_clnt_ctxt *cctxt = NULL;
	struct vcd_buffer_entry *ip_buf_entry = NULL;
	u32 result = false;

	VCD_MSG_LOW("vcd_try_submit_frame:");

	if (!vcd_get_frame_channel(dev_ctxt, &transc))
		return;

	if (!vcd_schedule_frame(dev_ctxt, &cctxt, &ip_buf_entry)) {
		vcd_release_frame_channel(dev_ctxt, transc);
		return;
	}

	rc = vcd_power_event(dev_ctxt, cctxt, VCD_EVT_PWR_CLNT_CMD_BEGIN);

	if (!VCD_FAILED(rc)) {
		transc->cctxt = cctxt;
		transc->ip_buf_entry = ip_buf_entry;

		result = vcd_submit_frame(dev_ctxt, transc);
	} else {
		VCD_MSG_ERROR("Failed: VCD_EVT_PWR_CLNT_CMD_BEGIN");

		vcd_requeue_input_frame(dev_ctxt, cctxt, ip_buf_entry);

		vcd_map_sched_status(sched_update_client_o_tkn(
			dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
			true, cctxt->sched_o_tkn_per_ip_frm));
	}

	if (!result) {
		vcd_release_frame_channel(dev_ctxt, transc);
		vcd_power_event(dev_ctxt, cctxt, VCD_EVT_PWR_CLNT_CMD_FAIL);
	}
}

u32 vcd_submit_frame(struct vcd_dev_ctxt *dev_ctxt, struct vcd_transc *transc)
{
	struct vcd_clnt_ctxt *cctxt = NULL;
	struct vcd_frame_data *ip_frm_entry;
	struct vcd_buffer_entry *op_buf_entry = NULL;
	u32 rc = VCD_S_SUCCESS;
	u32 evcode = 0;
	struct ddl_frame_data_tag ddl_ip_frm;
	struct ddl_frame_data_tag ddl_op_frm;

	VCD_MSG_LOW("vcd_submit_frame:");
	cctxt = transc->cctxt;
	ip_frm_entry = &transc->ip_buf_entry->frame;

	transc->op_buf_entry = op_buf_entry;
	transc->ip_frm_tag = ip_frm_entry->ip_frm_tag;
	transc->time_stamp = ip_frm_entry->time_stamp;
	ip_frm_entry->ip_frm_tag = (u32) transc;
	memset(&ddl_ip_frm, 0, sizeof(ddl_ip_frm));
	memset(&ddl_op_frm, 0, sizeof(ddl_op_frm));
	if (cctxt->decoding) {
		evcode = CLIENT_STATE_EVENT_NUMBER(pf_decode_frame);
		ddl_ip_frm.vcd_frm = *ip_frm_entry;
		rc = ddl_decode_frame(cctxt->ddl_handle, &ddl_ip_frm,
			(void *) transc);
	} else {
		op_buf_entry = vcd_buffer_pool_entry_de_q(&cctxt->out_buf_pool);
		if (!op_buf_entry) {
			VCD_MSG_ERROR("Sched provided frame when no"
				"op buffer was present");
			rc = VCD_ERR_FAIL;
		} else {
			op_buf_entry->in_use = true;
			cctxt->out_buf_pool.in_use++;
			ddl_ip_frm.vcd_frm = *ip_frm_entry;
			ddl_ip_frm.frm_delta = vcd_calculate_frame_delta(cctxt,
				ip_frm_entry);

			ddl_op_frm.vcd_frm = op_buf_entry->frame;

			evcode = CLIENT_STATE_EVENT_NUMBER(pf_encode_frame);

			rc = ddl_encode_frame(cctxt->ddl_handle, &ddl_ip_frm,
				&ddl_op_frm, (void *) transc);
		}
	}
	ip_frm_entry->ip_frm_tag = transc->ip_frm_tag;
	if (!VCD_FAILED(rc)) {
		vcd_device_timer_start(dev_ctxt);
		cctxt->status.frame_submitted++;
		if (ip_frm_entry->flags & VCD_FRAME_FLAG_EOS)
			vcd_do_client_state_transition(cctxt,
				VCD_CLIENT_STATE_EOS, evcode);
	} else {
		VCD_MSG_ERROR("Frame submission failed. rc = 0x%x", rc);
		vcd_handle_submit_frame_failed(dev_ctxt, transc);
	}
	return true;
}

u32 vcd_try_submit_frame_in_continue(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc)
{
	struct vcd_clnt_ctxt *cctxt = NULL;
	struct vcd_buffer_entry *ip_buf_entry = NULL;

	VCD_MSG_LOW("vcd_try_submit_frame_in_continue:");

	if (!vcd_schedule_frame(dev_ctxt, &cctxt, &ip_buf_entry))
		return false;

	transc->cctxt = cctxt;
	transc->ip_buf_entry = ip_buf_entry;

	return vcd_submit_frame(dev_ctxt, transc);
}

u32 vcd_process_cmd_sess_start(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_transc *transc;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_process_cmd_sess_start:");
	if (vcd_get_command_channel(cctxt->dev_ctxt, &transc)) {
		rc = vcd_power_event(cctxt->dev_ctxt, cctxt,
			VCD_EVT_PWR_CLNT_CMD_BEGIN);

		if (!VCD_FAILED(rc)) {
			transc->type = VCD_CMD_CODEC_START;
			transc->cctxt = cctxt;
			rc = vcd_submit_cmd_sess_start(transc);
		} else {
			VCD_MSG_ERROR("Failed: VCD_EVT_PWR_CLNT_CMD_BEGIN");
		}

		if (VCD_FAILED(rc)) {
			vcd_release_command_channel(cctxt->dev_ctxt,
							transc);
		}
	} else {
		u32 result;

		result = vcd_client_cmd_en_q(cctxt, VCD_CMD_CODEC_START);
		if (!result) {
			rc = VCD_ERR_BUSY;
			VCD_MSG_ERROR("%s(): vcd_client_cmd_en_q() "
				"failed\n", __func__);
			vcd_assert();
		}
	}

	if (VCD_FAILED(rc)) {
		vcd_power_event(cctxt->dev_ctxt, cctxt,
			VCD_EVT_PWR_CLNT_CMD_FAIL);
	}

	return rc;
}

void vcd_send_frame_done_in_eos(struct vcd_clnt_ctxt *cctxt,
	 struct vcd_frame_data *input_frame, u32 valid_opbuf)
{
	VCD_MSG_LOW("vcd_send_frame_done_in_eos:");

	if (!input_frame->virt_addr && !valid_opbuf) {
		VCD_MSG_MED("Sending NULL output with EOS");

		cctxt->out_buf_pool.entries[0].frame.flags = VCD_FRAME_FLAG_EOS;
		cctxt->out_buf_pool.entries[0].frame.data_len = 0;
		cctxt->out_buf_pool.entries[0].frame.time_stamp =
			input_frame->time_stamp;
		cctxt->out_buf_pool.entries[0].frame.ip_frm_tag =
			input_frame->ip_frm_tag;

		cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS,
			&cctxt->out_buf_pool.entries[0].frame,
			sizeof(struct vcd_frame_data), cctxt,
			cctxt->client_data);

		memset(&cctxt->out_buf_pool.entries[0].frame, 0,
			sizeof(struct vcd_frame_data));
	} else if (!input_frame->data_len) {
		if (cctxt->decoding)
			vcd_send_frame_done_in_eos_for_dec(cctxt, input_frame);
		else
			vcd_send_frame_done_in_eos_for_enc(cctxt, input_frame);
	}
}

void vcd_send_frame_done_in_eos_for_dec(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame)
{
	struct vcd_buffer_entry *buf_entry;
	struct vcd_property_hdr prop_hdr;
	u32 rc;
	struct ddl_frame_data_tag ddl_frm;

	prop_hdr.id = DDL_I_DPB_RETRIEVE;
	prop_hdr.sz = sizeof(struct ddl_frame_data_tag);
	memset(&ddl_frm, 0, sizeof(ddl_frm));
	rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &ddl_frm);

	if (VCD_FAILED(rc) || !ddl_frm.vcd_frm.virt_addr) {
		cctxt->status.eos_trig_ip_frm = *input_frame;
		cctxt->status.eos_wait_for_op_buf = true;

		return;
	}

	buf_entry = vcd_find_buffer_pool_entry(&cctxt->out_buf_pool,
		ddl_frm.vcd_frm.virt_addr);
	if (!buf_entry) {
		VCD_MSG_ERROR("Unrecognized buffer address provided %p",
				  ddl_frm.vcd_frm.virt_addr);
		vcd_assert();
	} else {
		vcd_map_sched_status(sched_update_client_o_tkn(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,\
			false, cctxt->sched_o_tkn_per_ip_frm));

		VCD_MSG_MED("Sending non-NULL output with EOS");

		buf_entry->frame.data_len = 0;
		buf_entry->frame.offset = 0;
		buf_entry->frame.flags |= VCD_FRAME_FLAG_EOS;
		buf_entry->frame.ip_frm_tag = input_frame->ip_frm_tag;
		buf_entry->frame.time_stamp = input_frame->time_stamp;

		cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS,
			&buf_entry->frame, sizeof(struct vcd_frame_data),
			cctxt, cctxt->client_data);

		buf_entry->in_use = false;
		VCD_BUFFERPOOL_INUSE_DECREMENT(cctxt->out_buf_pool.in_use);
	}
}

void vcd_send_frame_done_in_eos_for_enc(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame)
{
	struct vcd_buffer_entry *op_buf_entry;

	if (!cctxt->out_buf_pool.q_len) {
		cctxt->status.eos_trig_ip_frm = *input_frame;

		cctxt->status.eos_wait_for_op_buf = true;

		return;
	}

	op_buf_entry = vcd_buffer_pool_entry_de_q(&cctxt->out_buf_pool);
	if (!op_buf_entry) {
		VCD_MSG_ERROR("%s(): vcd_buffer_pool_entry_de_q() "
			"failed\n", __func__);
		vcd_assert();
	} else {
		vcd_map_sched_status(sched_update_client_o_tkn(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
			false, cctxt->sched_o_tkn_per_ip_frm));

		VCD_MSG_MED("Sending non-NULL output with EOS");

		op_buf_entry->frame.data_len = 0;
		op_buf_entry->frame.flags |= VCD_FRAME_FLAG_EOS;
		op_buf_entry->frame.ip_frm_tag = input_frame->ip_frm_tag;
		op_buf_entry->frame.time_stamp = input_frame->time_stamp;

		cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS,
			&op_buf_entry->frame, sizeof(struct vcd_frame_data),
			cctxt, cctxt->client_data);
	}
}

u32 vcd_handle_recvd_eos(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame, u32 *pb_eos_handled)
{
	union sched_value_type sched_val;
	u32 rc;

	VCD_MSG_LOW("vcd_handle_recvd_eos:");

	*pb_eos_handled = false;

	if (input_frame->virt_addr && input_frame->data_len)
		return VCD_S_SUCCESS;

	input_frame->data_len = 0;

	rc = vcd_map_sched_status(sched_get_client_param(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_CURRQLEN, &sched_val));

	VCD_FAILED_RETURN(rc, "Failed: sched_get_client_param");

	if (sched_val.un_value > 0) {
		rc = vcd_map_sched_status(sched_mark_client_eof(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl));

		if (!VCD_FAILED(rc)) {
			*pb_eos_handled = true;
		} else {
			VCD_MSG_ERROR("rc = 0x%x. Failed: "
				"sched_mark_client_eof", rc);
		}

	} else if (cctxt->decoding && !input_frame->virt_addr) {
		rc = vcd_map_sched_status(sched_update_client_o_tkn(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl, true,
			cctxt->sched_o_tkn_per_ip_frm));
	} else if (!cctxt->decoding) {

		vcd_send_frame_done_in_eos(cctxt, input_frame, false);

		if (cctxt->status.eos_wait_for_op_buf) {
			vcd_do_client_state_transition(cctxt,
				VCD_CLIENT_STATE_EOS,
				CLIENT_STATE_EVENT_NUMBER(pf_encode_frame));
		}

		*pb_eos_handled = true;

	}

	if (*pb_eos_handled && input_frame->virt_addr &&
			!input_frame->data_len) {
		cctxt->callback(VCD_EVT_RESP_INPUT_DONE, VCD_S_SUCCESS,
			input_frame, sizeof(struct vcd_frame_data), cctxt,
			cctxt->client_data);
	}
	return rc;
}

u32 vcd_handle_first_decode_frame(struct vcd_clnt_ctxt *cctxt)
{
	struct ddl_property_dec_pic_buffers dpb;
	struct vcd_property_hdr prop_hdr;
	u32 rc;
	u16 i;
	u16 q_cntr;
	struct ddl_frame_data_tag *frm_entry;
	struct ddl_frame_data_tag ddl_frm;
	struct vcd_buffer_pool *out_buf_pool;

	VCD_MSG_LOW("vcd_handle_first_decode_frame:");

	if (!cctxt->in_buf_pool.entries || !cctxt->out_buf_pool.entries ||
			cctxt->in_buf_pool.validated !=
			cctxt->in_buf_pool.count ||
			cctxt->out_buf_pool.validated !=
			cctxt->out_buf_pool.count) {
		VCD_MSG_ERROR("Buffer pool is not completely setup yet");

		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_add_client_to_sched(cctxt);

	VCD_FAILED_RETURN(rc, "Failed: vcd_add_client_to_sched");

	prop_hdr.id = DDL_I_DPB;
	prop_hdr.sz = sizeof(dpb);

	out_buf_pool = &cctxt->out_buf_pool;

	frm_entry = kmalloc(sizeof(struct ddl_frame_data_tag) *
		out_buf_pool->count, GFP_KERNEL);
	if (!frm_entry) {
		VCD_MSG_ERROR("Memory allocation failure");
		return VCD_ERR_ALLOC_FAIL;
	}

	for (i = 1; i <= out_buf_pool->count; i++)
		frm_entry[i - 1].vcd_frm = out_buf_pool->entries[i].frame;

	dpb.dec_pic_buffers = frm_entry;
	dpb.no_of_dec_pic_buf = out_buf_pool->count;
	rc = ddl_set_property(cctxt->ddl_handle, &prop_hdr, &dpb);

	kfree(frm_entry);

	VCD_FAILED_RETURN(rc, "Failed: DDL set DDL_I_DPB");

	if (out_buf_pool->q_len > 0) {
		prop_hdr.id = DDL_I_DPB_RELEASE;
		prop_hdr.sz = sizeof(struct ddl_frame_data_tag);

		for (i = 0, q_cntr = out_buf_pool->q_head; !VCD_FAILED(rc) &&
				i < out_buf_pool->q_len; i++,
				q_cntr = (q_cntr + 1) % out_buf_pool->count) {

			ddl_frm.vcd_frm = out_buf_pool->queue[q_cntr]->frame;

			rc = ddl_set_property(cctxt->ddl_handle, &prop_hdr,
				&ddl_frm);

			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR
					("Error returning output buffer to HW");

				out_buf_pool->queue[q_cntr]->in_use = false;
			} else {
				out_buf_pool->queue[q_cntr]->in_use = true;
				out_buf_pool->in_use++;
			}
		}

		if (VCD_FAILED(rc))
			return rc;
		rc = vcd_map_sched_status(sched_update_client_o_tkn(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl, true,
			cctxt->sched_o_tkn_per_ip_frm * out_buf_pool->q_len));
	}
	return rc;
}

u32 vcd_setup_with_ddl_capabilities(struct vcd_dev_ctxt *dev_ctxt)
{
	struct vcd_property_hdr prop_hdr;
	struct ddl_property_capability capability;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_setup_with_ddl_capabilities:");

	if (dev_ctxt->ddl_cmd_ch_depth)
		goto out;

	prop_hdr.id = DDL_I_CAPABILITY;
	prop_hdr.sz = sizeof(capability);

	/*
	 * Since this is underlying core's property we don't need a
	 * ddl client handle.
	 */
	rc = ddl_get_property(NULL, &prop_hdr, &capability);

	if (VCD_FAILED(rc))
		goto out;

	/*
	** Allocate the transaction table.
	*/
	dev_ctxt->trans_tbl_size = VCD_MAX_CLIENT_TRANSACTIONS *
		capability.max_num_client + capability.general_command_depth;

	dev_ctxt->trans_tbl = kzalloc(sizeof(struct vcd_transc) *
		dev_ctxt->trans_tbl_size, GFP_KERNEL);
	if (!dev_ctxt->trans_tbl) {
		VCD_MSG_ERROR("Transaction table alloc failed");
		rc = VCD_ERR_ALLOC_FAIL;
		goto out;
	}

	/*
	** Set the command/frame depth
	*/
	dev_ctxt->ddl_cmd_concurrency =	!capability.exclusive;
	dev_ctxt->ddl_frame_ch_depth = capability.frame_command_depth;
	dev_ctxt->ddl_cmd_ch_depth = capability.general_command_depth;

	vcd_reset_device_channels(dev_ctxt);

	dev_ctxt->hw_time_out = capability.ddl_time_out_in_ms;

out:
	return rc;
}

struct vcd_transc *vcd_get_free_trans_tbl_entry(struct vcd_dev_ctxt *dev_ctxt)
{
	u8 i;

	if (!dev_ctxt->trans_tbl)
		return NULL;

	i = 0;
	while (i < dev_ctxt->trans_tbl_size && dev_ctxt->trans_tbl[i].in_use)
		i++;

	if (i == dev_ctxt->trans_tbl_size) {
		return NULL;
	} else {
		memset(&dev_ctxt->trans_tbl[i], 0, sizeof(struct vcd_transc));

		dev_ctxt->trans_tbl[i].in_use = true;

		return &dev_ctxt->trans_tbl[i];
	}
}

void vcd_release_trans_tbl_entry(struct vcd_transc *trans_entry)
{
	if (trans_entry)
		trans_entry->in_use = false;
}

u32 vcd_add_client_to_sched(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_property_hdr prop_hdr;
	struct sched_client_init_param sched_input_init;
	u32 rc, seqhdr_present = 0;;

	if (cctxt->sched_clnt_valid) {
		VCD_MSG_HIGH("Schedulder client is already added ");
		return VCD_S_SUCCESS;
	}

	prop_hdr.id = DDL_I_FRAME_PROC_UNITS;
	prop_hdr.sz = sizeof(cctxt->frm_p_units);
	rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr,
		&cctxt->frm_p_units);
	VCD_FAILED_RETURN(rc, "Failed: Get DDL_I_FRAME_PROC_UNITS");

	if (cctxt->decoding) {
		cctxt->frm_rate.fps_numerator = VCD_DEC_INITIAL_FRAME_RATE;
		cctxt->frm_rate.fps_denominator = 1;

		sched_input_init.o_tkn_per_ip_frm =
			VCD_SCHEDULER_DEC_DFLT_OTKN_PERFRM;
		cctxt->sched_o_tkn_per_ip_frm =
			VCD_SCHEDULER_DEC_DFLT_OTKN_PERFRM;

		sched_input_init.o_tkn_max = cctxt->sched_o_tkn_per_ip_frm *
			cctxt->out_buf_pool.count+1;
	} else {
		sched_input_init.o_tkn_per_ip_frm =
			VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM;
		cctxt->sched_o_tkn_per_ip_frm =
			VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM;
		prop_hdr.id = DDL_I_SEQHDR_PRESENT;
		prop_hdr.sz = sizeof(seqhdr_present);
		rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr,
			&seqhdr_present);
		if (!VCD_FAILED(rc)) {
			if (seqhdr_present == 0x1) {
				VCD_MSG_MED("Sequence hdr present");
				sched_input_init.o_tkn_per_ip_frm++;
			}
			sched_input_init.o_tkn_max = cctxt->out_buf_pool.count;
			prop_hdr.id = VCD_I_FRAME_RATE;
			prop_hdr.sz = sizeof(cctxt->frm_rate);
			rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr,
				&cctxt->frm_rate);
		}
	}

	VCD_FAILED_RETURN(rc, "Failed: DDL get VCD_I_FRAME_RATE");

	if (cctxt->live)
		sched_input_init.client_ctgy = SCHED_CLNT_RT_NOBUFF;
	else
		sched_input_init.client_ctgy = SCHED_CLNT_NONRT;

	sched_input_init.max_queue_len = max(cctxt->in_buf_pool.count,
		VCD_MAX_SCHEDULER_QUEUE_SIZE(cctxt->frm_rate.fps_numerator,
		cctxt->frm_rate.fps_denominator));
	cctxt->reqd_perf_lvl = cctxt->frm_p_units *
		cctxt->frm_rate.fps_numerator /	cctxt->frm_rate.fps_denominator;

	sched_input_init.frm_rate.numer = cctxt->frm_rate.fps_numerator;
	sched_input_init.frm_rate.denom = cctxt->frm_rate.fps_denominator;
	sched_input_init.tkn_per_frm = cctxt->frm_p_units;
	sched_input_init.alloc_p_tkn_rate = cctxt->reqd_perf_lvl;

	sched_input_init.o_tkn_init = 0;

	sched_input_init.client_data = cctxt;

	rc = vcd_map_sched_status(sched_add_client(cctxt->dev_ctxt->sched_hdl,
		&sched_input_init, &cctxt->sched_clnt_hdl));

	if (!VCD_FAILED(rc))
		cctxt->sched_clnt_valid = true;

	return rc;
}

u32 vcd_handle_input_done(struct vcd_clnt_ctxt *cctxt, void *payload, u32 event,
	u32 status)
{
	struct vcd_transc *transc;
	struct ddl_frame_data_tag *frame = (struct ddl_frame_data_tag *)payload;
	u32 rc;

	if (!cctxt->status.frame_submitted && !cctxt->status.frame_delayed) {
		VCD_MSG_ERROR("Input done was not expected");
		vcd_assert();

		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_validate_io_done_pyld(payload, status);
	VCD_FAILED_RETURN(rc, "Bad input done payload");

	transc = (struct vcd_transc *)frame->vcd_frm.ip_frm_tag;

	if (transc->ip_buf_entry->frame.virt_addr != frame->vcd_frm.virt_addr ||
			!transc->ip_buf_entry->in_use) {
		VCD_MSG_ERROR("Bad frm transaction state");
		vcd_assert();
	}

	frame->vcd_frm.ip_frm_tag = transc->ip_frm_tag;

	cctxt->callback(event, status, &frame->vcd_frm,
		sizeof(struct vcd_frame_data), cctxt, cctxt->client_data);

	transc->frame_type = frame->vcd_frm.frame_type;

	transc->ip_buf_entry->in_use = false;
	VCD_BUFFERPOOL_INUSE_DECREMENT(cctxt->in_buf_pool.in_use);
	transc->ip_buf_entry = NULL;
	transc->input_done = true;

	if (transc->input_done && transc->frame_done)
		transc->in_use = false;

	if (VCD_FAILED(status)) {
		VCD_MSG_ERROR("INPUT_DONE returned err = 0x%x", status);
		vcd_handle_input_done_failed(cctxt, transc);
	}

	if (cctxt->status.frame_submitted > 0)
		cctxt->status.frame_submitted--;
	else
		cctxt->status.frame_delayed--;

	if (!VCD_FAILED(status) && cctxt->decoding) {
		if (frame->vcd_frm.interlaced)
			vcd_handle_input_done_for_interlacing(cctxt);
		if (frame->frm_trans_end)
			vcd_handle_input_done_with_trans_end(cctxt);
	}

	return VCD_S_SUCCESS;
}

void vcd_handle_input_done_in_eos(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 status)
{
	struct vcd_transc *transc;
	struct ddl_frame_data_tag *frame = (struct ddl_frame_data_tag *)payload;

	if (VCD_FAILED(vcd_validate_io_done_pyld(payload, status)))
		return;

	transc = (struct vcd_transc *)frame->vcd_frm.ip_frm_tag;

	vcd_handle_input_done(cctxt, payload, VCD_EVT_RESP_INPUT_DONE, status);

	if ((frame->vcd_frm.flags & VCD_FRAME_FLAG_EOS)) {
		VCD_MSG_HIGH("Got input done for EOS initiator");
		transc->input_done = false;
		transc->in_use = true;
	}
}

u32 vcd_validate_io_done_pyld(void *payload, u32 status)
{
	struct ddl_frame_data_tag *frame = (struct ddl_frame_data_tag *)payload;

	if (!frame) {
		VCD_MSG_ERROR("Bad payload from DDL");
		vcd_assert();

		return VCD_ERR_BAD_POINTER;
	}

	if (!frame->vcd_frm.ip_frm_tag || frame->vcd_frm.ip_frm_tag ==
			VCD_FRAMETAG_INVALID) {
		VCD_MSG_ERROR("bad input frame tag");
		vcd_assert();
		return VCD_ERR_BAD_POINTER;
	}

	if (!frame->vcd_frm.virt_addr && status != VCD_ERR_INTRLCD_FIELD_DROP)
		return VCD_ERR_BAD_POINTER;

	return VCD_S_SUCCESS;
}

void vcd_handle_input_done_failed(struct vcd_clnt_ctxt *cctxt,
		struct vcd_transc *transc)
{
	if (cctxt->decoding) {
		vcd_map_sched_status(sched_update_client_o_tkn(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl, true,
			cctxt->sched_o_tkn_per_ip_frm));

		transc->in_use = false;
	}
}

void vcd_handle_input_done_for_interlacing(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc;

	cctxt->status.int_field_cnt++;

	if (cctxt->status.int_field_cnt == 1) {
		rc = vcd_map_sched_status(sched_update_client_o_tkn(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl, true,
			cctxt->sched_o_tkn_per_ip_frm));

		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("sched_update_client_o_tkn failed");
	} else if (cctxt->status.int_field_cnt == VCD_DEC_NUM_INTERLACED_FIELDS)
		cctxt->status.int_field_cnt = 0;
}

void vcd_handle_input_done_with_trans_end(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc;
	union sched_value_type sched_val;
	if (!cctxt->decoding)
		return;

	if (cctxt->out_buf_pool.in_use < cctxt->out_buf_pool.buf_req.min_count)
		return;

	rc = vcd_map_sched_status(sched_get_client_param(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_OTKNCURRENT, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("sched_get_client_param:OTKNCURRENT failed");
		return;
	}

	if (!sched_val.un_value) {
		VCD_MSG_MED("All output buffers with core are pending display");

		rc = vcd_map_sched_status(sched_update_client_o_tkn(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl, true,
			cctxt->sched_o_tkn_per_ip_frm));

		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("sched_update_client_o_tkn failed");
	}
}

u32 vcd_handle_output_required(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 status)
{
	struct vcd_transc *transc;
	struct ddl_frame_data_tag *frame = (struct ddl_frame_data_tag *)payload;
	u32 rc;

	if (!cctxt->status.frame_submitted && !cctxt->status.frame_delayed) {
		VCD_MSG_ERROR("\n Input done was not expected");
		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_validate_io_done_pyld(payload, status);
	VCD_FAILED_RETURN(rc, "\n Bad input done payload");

	transc = (struct vcd_transc *)frame->vcd_frm.ip_frm_tag;

	if (transc->ip_buf_entry->frame.virt_addr != frame->vcd_frm.virt_addr ||
			!transc->ip_buf_entry->in_use) {
		VCD_MSG_ERROR("\n Bad frm transaction state");
		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_map_sched_status(sched_re_queue_frame(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		(void *) transc->ip_buf_entry));

	VCD_FAILED_RETURN(rc, "Failed: sched_queue_frame");

	if (transc->ip_buf_entry->frame.flags &	VCD_FRAME_FLAG_EOS) {
		rc = vcd_map_sched_status(sched_mark_client_eof(
			cctxt->dev_ctxt->sched_hdl,
			cctxt->sched_clnt_hdl));
	}

	VCD_FAILED_RETURN(rc, "Failed: sched_mark_client_eof");

	transc->ip_buf_entry = NULL;
	transc->in_use = false;
	frame->frm_trans_end = true;

	if (VCD_FAILED(status))
		VCD_MSG_ERROR("\n OUTPUT_REQ returned err = 0x%x", status);

	if (cctxt->status.frame_submitted > 0)
		cctxt->status.frame_submitted--;
	else
		cctxt->status.frame_delayed--;

	if (!VCD_FAILED(status) && cctxt->decoding &&
			frame->vcd_frm.interlaced) {
		if (cctxt->status.int_field_cnt > 0)
			VCD_MSG_ERROR("\n Not expected: OUTPUT_REQ"
				"for 2nd interlace field");
	}

	return VCD_S_SUCCESS;
}

u32 vcd_handle_output_required_in_flushing(struct vcd_clnt_ctxt *cctxt,
	void *payload)
{
	u32 rc;
	struct vcd_transc *transc;

	rc = vcd_validate_io_done_pyld(payload, VCD_S_SUCCESS);
	VCD_FAILED_RETURN(rc, "Bad input done payload");

	transc = (struct vcd_transc *) (((struct ddl_frame_data_tag *)payload)->
		 vcd_frm.ip_frm_tag);

	((struct ddl_frame_data_tag *)payload)->vcd_frm.interlaced = false;

	rc = vcd_handle_input_done(cctxt, payload, VCD_EVT_RESP_INPUT_FLUSHED,
		VCD_S_SUCCESS);

	transc->in_use = false;
	((struct ddl_frame_data_tag *)payload)->frm_trans_end = true;

	return rc;
}

u32 vcd_handle_frame_done(struct vcd_clnt_ctxt *cctxt, void *payload, u32 event,
	u32 status)
{
	struct vcd_buffer_entry *op_buf_entry;
	struct ddl_frame_data_tag *op_frm = (struct ddl_frame_data_tag *)
		payload;
	struct vcd_transc *transc;
	u32 rc;

	rc = vcd_validate_io_done_pyld(payload, status);
	VCD_FAILED_RETURN(rc, "Bad payload recvd");

	transc = (struct vcd_transc *)op_frm->vcd_frm.ip_frm_tag;

	if (op_frm->vcd_frm.virt_addr) {

		if (!transc->op_buf_entry) {
			op_buf_entry = vcd_find_buffer_pool_entry(
				&cctxt->out_buf_pool, op_frm->vcd_frm.virt_addr);
		} else {
			op_buf_entry = transc->op_buf_entry;
		}

		if (!op_buf_entry) {
			VCD_MSG_ERROR("Invalid output buffer returned"
				"from DDL");
			vcd_assert();
			rc = VCD_ERR_BAD_POINTER;
		} else if (!op_buf_entry->in_use) {
			VCD_MSG_ERROR("Bad output buffer %p recv from DDL",
				op_buf_entry->frame.virt_addr);
			vcd_assert();
			rc = VCD_ERR_BAD_POINTER;
		} else {
			op_buf_entry->in_use = false;
			VCD_BUFFERPOOL_INUSE_DECREMENT(
				cctxt->out_buf_pool.in_use);
			VCD_MSG_LOW("outBufPool.InUse = %d",
				cctxt->out_buf_pool.in_use);
		}
	}
	VCD_FAILED_RETURN(rc, "Bad output buffer pointer");
	op_frm->vcd_frm.time_stamp = transc->time_stamp;
	op_frm->vcd_frm.ip_frm_tag = transc->ip_frm_tag;
	op_frm->vcd_frm.frame_type = transc->frame_type;

	transc->frame_done = true;

	if (transc->input_done && transc->frame_done)
		transc->in_use = false;

	if (status == VCD_ERR_INTRLCD_FIELD_DROP ||
			(op_frm->intrlcd_ip_frm_tag != VCD_FRAMETAG_INVALID &&
			op_frm->intrlcd_ip_frm_tag)) {
		vcd_handle_frame_done_for_interlacing(cctxt, transc, op_frm,
			status);
	}

	if (status != VCD_ERR_INTRLCD_FIELD_DROP) {
		cctxt->callback(event, status, &op_frm->vcd_frm,
			sizeof(struct vcd_frame_data), cctxt,
			cctxt->client_data);
	}
	return VCD_S_SUCCESS;
}

void vcd_handle_frame_done_in_eos(struct vcd_clnt_ctxt *cctxt, void *payload,
	u32 status)
{
	struct ddl_frame_data_tag *frame = (struct ddl_frame_data_tag *)payload;

	VCD_MSG_LOW("vcd_handle_frame_done_in_eos:");

	if (VCD_FAILED(vcd_validate_io_done_pyld(payload, status)))
		return;

	if (cctxt->status.eos_prev_valid) {
		vcd_handle_frame_done(cctxt,
			(void *)&cctxt->status.eos_prev_op_frm,
			VCD_EVT_RESP_OUTPUT_DONE, status);
	}

	cctxt->status.eos_prev_op_frm = *frame;
	cctxt->status.eos_prev_valid = true;
}

void vcd_handle_frame_done_for_interlacing(struct vcd_clnt_ctxt *cctxt,
	 struct vcd_transc *transc_ip1, struct ddl_frame_data_tag *op_frm,
	 u32 status)
{
	struct vcd_transc *transc_ip2 =	(struct vcd_transc *)
		op_frm->intrlcd_ip_frm_tag;

	if (status == VCD_ERR_INTRLCD_FIELD_DROP) {
		cctxt->status.int_field_cnt = 0;
		return;
	}

	op_frm->intrlcd_ip_frm_tag = transc_ip2->ip_frm_tag;

	transc_ip2->frame_done = true;

	if (transc_ip2->input_done && transc_ip2->frame_done)
		transc_ip2->in_use = false;

	if (!transc_ip1->frame_type || !transc_ip2->frame_type) {
		VCD_MSG_ERROR("DDL didn't provided frame type");
		return;
	}
}

u32 vcd_handle_first_frame_done(struct vcd_clnt_ctxt *cctxt, void *payload)
{
	if (!cctxt->decoding)
		return vcd_handle_first_encode_frame_done(cctxt, payload);

	return VCD_S_SUCCESS;
}

u32 vcd_handle_first_encode_frame_done(struct vcd_clnt_ctxt *cctxt,
	void *payload)
{
	struct vcd_buffer_entry *buf_entry;
	struct vcd_frame_data *frm_entry;
	u32 rc, seqhdr_present;
	struct vcd_property_hdr prop_hdr;
	struct vcd_sequence_hdr seq_hdr;
	struct vcd_property_codec codec;
	union sched_value_type sched_val;
	struct vcd_transc *transc;
	struct ddl_frame_data_tag *payload_frm = (struct ddl_frame_data_tag *)
		payload;
	VCD_MSG_LOW("vcd_handle_first_encode_frame_done:");

	rc = vcd_validate_io_done_pyld(payload, VCD_S_SUCCESS);
	VCD_FAILED_RETURN(rc, "Validate frame done payload failed");

	transc = (struct vcd_transc *)payload_frm->vcd_frm.ip_frm_tag;

	prop_hdr.id = DDL_I_SEQHDR_PRESENT;
	prop_hdr.sz = sizeof(seqhdr_present);
	rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &seqhdr_present);
	VCD_FAILED_RETURN(rc, "Failed: DDL_I_SEQHDR_PRESENT");
	if (!seqhdr_present)
		return VCD_S_SUCCESS;

	buf_entry = vcd_buffer_pool_entry_de_q(&cctxt->out_buf_pool);

	if (!buf_entry) {
		VCD_MSG_ERROR("Sched provided frame when 2nd op buffer "
			"was unavailable");

		rc = VCD_ERR_FAIL;
		vcd_assert();
		return rc;
	}

	frm_entry = &buf_entry->frame;
	prop_hdr.id = VCD_I_CODEC;
	prop_hdr.sz = sizeof(struct vcd_property_codec);

	rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &codec);
	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: ddl_get_property:VCD_I_CODEC",
			rc);
		goto out;
	}

	if (codec.codec != VCD_CODEC_H263) {
		prop_hdr.id = VCD_I_SEQ_HEADER;
		prop_hdr.sz = sizeof(struct vcd_sequence_hdr);

		seq_hdr.addr = frm_entry->virt_addr;
		seq_hdr.sz = buf_entry->size;

		rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &seq_hdr);
		if (VCD_FAILED(rc)) {
			VCD_MSG_ERROR("rc = 0x%x. Failed: "
				 "ddl_get_property:VCD_I_SEQ_HEADER", rc);
			goto out;
		}
	} else {
		VCD_MSG_LOW("Codec Type is H.263\n");
	}

	sched_val.un_value = VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM;

	rc = vcd_map_sched_status(sched_set_client_param(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_OTKNPERIPFRM, &sched_val));
	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x.Failed: sched_set_client_param", rc);
		goto out;
	}

	frm_entry->data_len = seq_hdr.sz;
	frm_entry->time_stamp = transc->time_stamp;
	frm_entry->ip_frm_tag =	transc->ip_frm_tag;
	frm_entry->flags |= VCD_FRAME_FLAG_CODECCONFIG;

	cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS, frm_entry,
		sizeof(struct vcd_frame_data), cctxt, cctxt->client_data);

out:
	if (VCD_FAILED(rc))
		vcd_buffer_pool_entry_en_q(&cctxt->out_buf_pool, buf_entry);

	return rc;
}

void vcd_handle_eos_trans_end(struct vcd_clnt_ctxt *cctxt)
{
	if (cctxt->status.eos_prev_valid) {
		vcd_handle_frame_done(cctxt,
			(void *)&cctxt->status.eos_prev_op_frm,
			VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS);

		cctxt->status.eos_prev_valid = false;
	}

	if (cctxt->status.flush_mode)
		vcd_process_pending_flush_in_eos(cctxt);

	if (cctxt->status.stop_pending)
		vcd_process_pending_stop_in_eos(cctxt);
	else {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_RUN,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}
}

void vcd_handle_eos_done(struct vcd_clnt_ctxt *cctxt, struct vcd_transc *transc,
	u32 status)
{
	struct vcd_frame_data  vcd_frm;
	VCD_MSG_LOW("vcd_handle_eos_done:");

	if (VCD_FAILED(status))
		VCD_MSG_ERROR("EOS DONE returned error = 0x%x", status);

	if (cctxt->status.eos_prev_valid) {
		cctxt->status.eos_prev_op_frm.vcd_frm.flags |=
			VCD_FRAME_FLAG_EOS;

		vcd_handle_frame_done(cctxt,
			(void *)&cctxt->status.eos_prev_op_frm,
			VCD_EVT_RESP_OUTPUT_DONE, VCD_S_SUCCESS);

		cctxt->status.eos_prev_valid = false;
	} else {
		if (transc->ip_buf_entry) {
			transc->ip_buf_entry->frame.ip_frm_tag =
				transc->ip_frm_tag;

			vcd_send_frame_done_in_eos(cctxt,
				&transc->ip_buf_entry->frame, false);
		} else {
			memset(&vcd_frm, 0, sizeof(struct vcd_frame_data));
			vcd_frm.ip_frm_tag = transc->ip_frm_tag;
			vcd_frm.time_stamp = transc->time_stamp;
			vcd_frm.flags = VCD_FRAME_FLAG_EOS;
			vcd_send_frame_done_in_eos(cctxt, &vcd_frm, true);
		}
	}
	if (transc->ip_buf_entry) {
		if (transc->ip_buf_entry->frame.virt_addr) {
			transc->ip_buf_entry->frame.ip_frm_tag =
				transc->ip_frm_tag;

			cctxt->callback(VCD_EVT_RESP_INPUT_DONE,
				  VCD_S_SUCCESS, &transc->ip_buf_entry->frame,
				  sizeof(struct vcd_frame_data), cctxt,
				  cctxt->client_data);
		}
		transc->ip_buf_entry->in_use = false;
		VCD_BUFFERPOOL_INUSE_DECREMENT(cctxt->in_buf_pool.in_use);
		transc->ip_buf_entry = NULL;
		cctxt->status.frame_submitted--;
	}

	transc->in_use = false;
	vcd_mark_frame_channel(cctxt->dev_ctxt);
	if (cctxt->status.flush_mode)
		vcd_process_pending_flush_in_eos(cctxt);

	if (cctxt->status.stop_pending) {
		vcd_process_pending_stop_in_eos(cctxt);
	} else if (!cctxt->status.eos_wait_for_op_buf) {
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_RUN,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}
}

void vcd_handle_start_done(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc, u32 status)
{
	cctxt->status.cmd_submitted--;
	vcd_mark_command_channel(cctxt->dev_ctxt, transc);

	if (!VCD_FAILED(status)) {
		cctxt->callback(VCD_EVT_RESP_START, status, NULL, 0, cctxt,
			cctxt->client_data);

		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_RUN,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	} else {
		VCD_MSG_ERROR("ddl callback returned failure.status = 0x%x",
			status);
		vcd_handle_err_in_starting(cctxt, status);
	}
}

void vcd_handle_stop_done(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc, u32 status)
{
	u32 rc = VCD_S_SUCCESS;
	u32 seq_hdrpresent = 0;
	union sched_value_type sched_val;
	struct vcd_property_hdr  prop_hdr;
	VCD_MSG_LOW("vcd_handle_stop_done:");
	cctxt->status.cmd_submitted--;
	vcd_mark_command_channel(cctxt->dev_ctxt, transc);

	if (VCD_FAILED(status)) {
		VCD_MSG_FATAL("STOP_DONE returned error = 0x%x", status);
		status = VCD_ERR_HW_FATAL;
		vcd_handle_device_err_fatal(cctxt->dev_ctxt, cctxt);
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_INVALID,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
		goto out;
	}

	if (!cctxt->decoding) {
		prop_hdr.id = DDL_I_SEQHDR_PRESENT;
		prop_hdr.sz = sizeof(seq_hdrpresent);
		rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr,
			&seq_hdrpresent);
		if (VCD_FAILED(rc)) {
			VCD_MSG_ERROR("Failed: DDL Get DDL_I_SEQHDR_PRESENT %d",
				rc);
			goto open_out;
		}
		if (seq_hdrpresent == 0x1) {
			sched_val.un_value = VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM
				+ 1;

			rc = vcd_map_sched_status(sched_set_client_param(
				cctxt->dev_ctxt->sched_hdl,
				cctxt->sched_clnt_hdl,
				SCHED_I_CLNT_OTKNPERIPFRM, &sched_val));
			if (VCD_FAILED(rc))
				VCD_MSG_ERROR("Failed: sched_set_client_param "
					"%d", rc);
		}
	}
open_out:
	vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_OPEN,
		CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));

out:
	cctxt->callback(VCD_EVT_RESP_STOP, status, NULL, 0, cctxt,
					  cctxt->client_data);

	memset(&cctxt->status, 0, sizeof(struct vcd_clnt_status));
}

void vcd_handle_stop_done_in_starting(struct vcd_clnt_ctxt *cctxt,
	struct vcd_transc *transc, u32 status)
{
	VCD_MSG_LOW("vcd_handle_stop_done_in_starting:");
	cctxt->status.cmd_submitted--;
	vcd_mark_command_channel(cctxt->dev_ctxt, transc);
	if (!VCD_FAILED(status)) {
		cctxt->callback(VCD_EVT_RESP_START, cctxt->status.last_err,
			NULL, 0, cctxt, cctxt->client_data);
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_OPEN,
			   CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	} else {
		VCD_MSG_FATAL("VCD Cleanup: STOP_DONE returned error "
			"= 0x%x", status);
		vcd_handle_err_fatal(cctxt, VCD_EVT_RESP_START,
			VCD_ERR_HW_FATAL);
	}
}

void vcd_handle_stop_done_in_invalid(struct vcd_clnt_ctxt *cctxt, u32 status)
{
	u32 rc;
	VCD_MSG_LOW("vcd_handle_stop_done_in_invalid:");
	if (!VCD_FAILED(status)) {
		vcd_client_cmd_flush_and_en_q(cctxt, VCD_CMD_CLIENT_CLOSE);
		if (cctxt->status.frame_submitted) {
			vcd_release_multiple_frame_channels(cctxt->dev_ctxt,
			cctxt->status.frame_submitted);

			cctxt->status.frame_submitted = 0;
			cctxt->status.frame_delayed = 0;
		}
		if (cctxt->status.cmd_submitted) {
			vcd_release_multiple_command_channels(cctxt->dev_ctxt,
				cctxt->status.cmd_submitted);
			cctxt->status.cmd_submitted = 0;
		}
	} else {
		VCD_MSG_FATAL("VCD Cleanup: STOP_DONE returned error "
			"= 0x%x", status);
		vcd_handle_device_err_fatal(cctxt->dev_ctxt, cctxt);
		cctxt->status.cleaning_up = false;
	}
	vcd_flush_buffers_in_err_fatal(cctxt);
	VCD_MSG_HIGH("VCD cleanup: All buffers are returned");
	if (cctxt->status.stop_pending) {
		cctxt->callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS, NULL, 0,
			cctxt, cctxt->client_data);
		cctxt->status.stop_pending = false;
	}
	rc = vcd_power_event(cctxt->dev_ctxt, cctxt, VCD_EVT_PWR_CLNT_ERRFATAL);
	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("VCD_EVT_PWR_CLNT_ERRFATAL failed");
	if (!cctxt->status.cleaning_up &&
		cctxt->status.close_pending) {
		vcd_destroy_client_context(cctxt);
		vcd_handle_for_last_clnt_close(cctxt->dev_ctxt, false);
	}
}

u32 vcd_handle_input_frame(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *input_frame)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	struct vcd_buffer_entry *buf_entry;
	struct vcd_frame_data *frm_entry;
	u32 rc = VCD_S_SUCCESS;
	u32 eos_handled = false;

	VCD_MSG_LOW("vcd_handle_input_frame:");

	VCD_MSG_LOW("input buffer: addr=(0x%p), size=(%d), len=(%d)",
		input_frame->virt_addr, input_frame->alloc_len,
		input_frame->data_len);

	if ((!input_frame->virt_addr || !input_frame->data_len) &&
			!(input_frame->flags & VCD_FRAME_FLAG_EOS)) {
		VCD_MSG_ERROR("Bad frame ptr/len/EOS combination");

		return VCD_ERR_ILLEGAL_PARM;
	}

	if (!cctxt->status.b1st_frame_recvd) {
		if (cctxt->decoding)
			rc = vcd_handle_first_decode_frame(cctxt);

		if (!VCD_FAILED(rc)) {
			cctxt->status.first_ts = input_frame->time_stamp;
			cctxt->status.prev_ts = cctxt->status.first_ts;

			cctxt->status.b1st_frame_recvd = true;

			vcd_power_event(cctxt->dev_ctxt, cctxt,
				VCD_EVT_PWR_CLNT_FIRST_FRAME);
		}
	}
	VCD_FAILED_RETURN(rc, "Failed: Frist frame handling");

	buf_entry = vcd_find_buffer_pool_entry(&cctxt->in_buf_pool,
		input_frame->virt_addr);
	if (!buf_entry) {
		VCD_MSG_ERROR("Bad buffer addr: %p", input_frame->virt_addr);
		return VCD_ERR_FAIL;
	}

	if (buf_entry->in_use) {
		VCD_MSG_ERROR("An inuse input frame is being re-queued to "
			"scheduler");
		return VCD_ERR_FAIL;
	}

	if (input_frame->alloc_len > buf_entry->size) {
		VCD_MSG_ERROR("Bad buffer Alloc_len %d, Actual size=%d",
			input_frame->alloc_len, buf_entry->size);

		return VCD_ERR_ILLEGAL_PARM;
	}

	frm_entry = &buf_entry->frame;

	*frm_entry = *input_frame;
	frm_entry->phys_addr = buf_entry->phys_addr;

	if (input_frame->flags & VCD_FRAME_FLAG_EOS)
		rc = vcd_handle_recvd_eos(cctxt, input_frame, &eos_handled);

	if (VCD_FAILED(rc) || eos_handled) {
		VCD_MSG_HIGH("rc = 0x%x, eos_handled = %d", rc, eos_handled);

		return rc;
	}

	rc = vcd_map_sched_status(sched_queue_frame(dev_ctxt->sched_hdl,
		cctxt->sched_clnt_hdl, (void *)buf_entry));

	VCD_FAILED_RETURN(rc, "Failed: sched_queue_frame");

	buf_entry->in_use = true;
	cctxt->in_buf_pool.in_use++;
	if (input_frame->flags & VCD_FRAME_FLAG_EOS) {
		rc = vcd_map_sched_status(sched_mark_client_eof(
			cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl));
	}

	VCD_FAILED_RETURN(rc, "Failed: sched_mark_client_eof");

	vcd_try_submit_frame(dev_ctxt);
	return rc;
}

void vcd_release_all_clnt_frm_transc(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u8 i;

	VCD_MSG_LOW("vcd_release_all_clnt_frm_transc:");

	for (i = 0; i < dev_ctxt->trans_tbl_size; i++) {
		if (dev_ctxt->trans_tbl[i].in_use &&
				cctxt == dev_ctxt->trans_tbl[i].cctxt &&
				dev_ctxt->trans_tbl[i].type ==
				VCD_CMD_CODE_FRAME) {
			vcd_release_trans_tbl_entry(&dev_ctxt->trans_tbl[i]);
		}
	}
}

void vcd_release_all_clnt_def_frm_transc(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u8 i;

	VCD_MSG_LOW("vcd_release_all_clnt_def_frm_transc:");

	for (i = 0; i < dev_ctxt->trans_tbl_size; i++) {
		if (dev_ctxt->trans_tbl[i].in_use &&
				cctxt == dev_ctxt->trans_tbl[i].cctxt &&
				dev_ctxt->trans_tbl[i].type == VCD_CMD_NONE) {
			vcd_release_trans_tbl_entry(&dev_ctxt->trans_tbl[i]);
		}
	}
}

void vcd_release_all_clnt_transc(struct vcd_clnt_ctxt *cctxt)
{
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u8 i;

	VCD_MSG_LOW("vcd_release_all_clnt_def_frm_transc:");

	for (i = 0; i < dev_ctxt->trans_tbl_size; i++) {
		if (dev_ctxt->trans_tbl[i].in_use &&
				cctxt == dev_ctxt->trans_tbl[i].cctxt) {
			vcd_release_trans_tbl_entry(&dev_ctxt->trans_tbl[i]);
		}
	}
}

void vcd_send_flush_done(struct vcd_clnt_ctxt *cctxt, u32 status)
{
	VCD_MSG_LOW("vcd_send_flush_done:");

	if (cctxt->status.flush_mode & VCD_FLUSH_INPUT) {
		cctxt->callback(VCD_EVT_RESP_FLUSH_INPUT_DONE, status, NULL, 0,
			cctxt, cctxt->client_data);
		cctxt->status.flush_mode &= ~VCD_FLUSH_INPUT;
	}

	if (cctxt->status.flush_mode & VCD_FLUSH_OUTPUT) {
		cctxt->callback(VCD_EVT_RESP_FLUSH_OUTPUT_DONE,	status, NULL, 0,
			cctxt, cctxt->client_data);
		cctxt->status.flush_mode &= ~VCD_FLUSH_OUTPUT;
	}
}

u32 vcd_store_seq_hdr(struct vcd_clnt_ctxt *cctxt,
	struct vcd_sequence_hdr *seq_hdr)
{
//	u32 rc;
//	struct vcd_property_hdr prop_hdr;
//	u32 align;
//	u32 addr;
//	int ret = 0;

	if (!seq_hdr->sz || !seq_hdr->addr) {
		VCD_MSG_ERROR("Bad seq hdr");
		return VCD_ERR_BAD_POINTER;
	}

	if (cctxt->seq_hdr.addr) {
		VCD_MSG_HIGH("Old seq hdr detected");

		dma_free_coherent(NULL, cctxt->seq_hdr.sz +
			VCD_SEQ_HDR_PADDING_BYTES, cctxt->seq_hdr.addr,
			cctxt->seq_hdr_phys_addr);
		cctxt->seq_hdr.addr = NULL;
	}

	cctxt->seq_hdr.sz = seq_hdr->sz;

	//TODO strip out all this alignment crap?
#if 0
	prop_hdr.id = DDL_I_SEQHDR_ALIGN_BYTES;
	prop_hdr.size = sizeof(u32);

	rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &align);

	VCD_FAILED_RETURN(rc,
		"Failed: ddl_get_property DDL_I_SEQHDR_ALIGN_BYTES");

	VCD_MSG_MED("Seq hdr alignment bytes = %d", align);
#endif

	cctxt->seq_hdr.addr = dma_alloc_coherent(NULL,
		cctxt->seq_hdr.sz + VCD_SEQ_HDR_PADDING_BYTES,
		&cctxt->seq_hdr_phys_addr, GFP_KERNEL);
	if (!cctxt->seq_hdr.addr) {
		VCD_MSG_ERROR("Seq hdr allocation failed");
		return VCD_ERR_ALLOC_FAIL;
	}

	memset(cctxt->seq_hdr.addr, 0,
		cctxt->seq_hdr.sz + VCD_SEQ_HDR_PADDING_BYTES);
	memcpy(cctxt->seq_hdr.addr, seq_hdr->addr, seq_hdr->sz);

	return VCD_S_SUCCESS;
}

u32 vcd_set_frame_rate(struct vcd_clnt_ctxt *cctxt,
	struct vcd_property_frame_rate *fps)
{
	union sched_value_type sched_val;
	u32 rc;

	sched_val.frm_rate.numer = fps->fps_numerator;
	sched_val.frm_rate.denom = fps->fps_denominator;
	cctxt->frm_rate = *fps;

	rc = vcd_map_sched_status(sched_set_client_param(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_FRAMERATE, &sched_val));
	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_FRAMERATE",
			rc);
	}

	rc = vcd_update_clnt_perf_lvl(cctxt, &cctxt->frm_rate,
		cctxt->frm_p_units);
	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_update_clnt_perf_lvl",
				  rc);
	}

	sched_val.un_value = cctxt->reqd_perf_lvl;

	rc = vcd_map_sched_status(sched_set_client_param(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_PTKNRATE, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_PTKNRATE",
				  rc);
	}

	return VCD_S_SUCCESS;
}

u32 vcd_set_frame_size(struct vcd_clnt_ctxt *cctxt,
	struct vcd_property_frame_size *frm_size)
{
	struct vcd_property_hdr prop_hdr;
	union sched_value_type sched_val;
	u32 rc;
	u32 frm_p_units;
	frm_size = NULL;

	prop_hdr.id = DDL_I_FRAME_PROC_UNITS;
	prop_hdr.sz = sizeof(frm_p_units);
	rc = ddl_get_property(cctxt->ddl_handle, &prop_hdr, &frm_p_units);

	VCD_FAILED_RETURN(rc, "Failed: Get DDL_I_FRAME_PROC_UNITS");

	cctxt->frm_p_units = sched_val.un_value = frm_p_units;

	rc = vcd_map_sched_status(sched_set_client_param(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_PTKNPERFRM, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_PTKNPERFRM",
				  rc);
	}

	rc = vcd_update_clnt_perf_lvl(cctxt, &cctxt->frm_rate, frm_p_units);

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_update_clnt_perf_lvl",
				  rc);
	}

	sched_val.un_value = cctxt->reqd_perf_lvl;

	rc = vcd_map_sched_status(sched_set_client_param(
		cctxt->dev_ctxt->sched_hdl, cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_PTKNRATE, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_PTKNRATE",
				  rc);
	}

	return VCD_S_SUCCESS;
}

void vcd_process_pending_flush_in_eos(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_HIGH("Buffer flush is pending");

	rc = vcd_flush_buffers(cctxt, cctxt->status.flush_mode);

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_flush_buffers", rc);

	cctxt->status.eos_wait_for_op_buf = false;

	vcd_send_flush_done(cctxt, VCD_S_SUCCESS);
}

void vcd_process_pending_stop_in_eos(struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	rc = vcd_flush_buffers(cctxt, VCD_FLUSH_ALL);

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_flush_buffers", rc);

	VCD_MSG_HIGH("All buffers are returned. Enqueuing stop cmd");

	vcd_client_cmd_flush_and_en_q(cctxt, VCD_CMD_CODEC_STOP);
	cctxt->status.stop_pending = false;

	vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_STOPPING,
		CLIENT_STATE_EVENT_NUMBER(pf_stop));
}

u32 vcd_calculate_frame_delta(struct vcd_clnt_ctxt *cctxt,
	struct vcd_frame_data *frame)
{
	u32 frm_delta;
	u64 temp, temp1;

	temp = frame->time_stamp - cctxt->status.prev_ts;

	VCD_MSG_LOW("Curr_ts=%lld  Prev_ts=%lld Diff=%llu", frame->time_stamp,
		cctxt->status.prev_ts, temp);

	temp = temp * cctxt->time_resoln;
	temp = (temp + (VCD_TIMESTAMP_RESOLUTION >> 1));
	temp1 = do_div(temp, VCD_TIMESTAMP_RESOLUTION);
	frm_delta = temp;
	VCD_MSG_LOW("temp1=%lld  temp=%lld", temp1, temp);
	cctxt->status.time_elapsed += frm_delta;

	temp = ((u64)cctxt->status.time_elapsed * VCD_TIMESTAMP_RESOLUTION);
	temp = (temp + (cctxt->time_resoln >> 1));
	temp1 = do_div(temp, cctxt->time_resoln);

	cctxt->status.prev_ts = cctxt->status.first_ts + temp;

	VCD_MSG_LOW("Time_elapsed=%u, Drift=%llu, new Prev_ts=%lld",
		cctxt->status.time_elapsed, temp1, cctxt->status.prev_ts);

	return frm_delta;
}

struct vcd_buffer_entry *vcd_check_fill_output_buffer(struct vcd_clnt_ctxt
	*cctxt, struct vcd_frame_data *buffer)
{
	struct vcd_buffer_pool *buf_pool = &cctxt->out_buf_pool;
	struct vcd_buffer_entry *buf_entry;

	if (!buf_pool->entries) {
		VCD_MSG_ERROR("Buffers not set or allocated yet");
		return NULL;
	}

	if (!buffer->virt_addr) {
		VCD_MSG_ERROR("NULL buffer address provided");
		return NULL;
	}

	buf_entry = vcd_find_buffer_pool_entry(buf_pool, buffer->virt_addr);
	if (!buf_entry) {
		VCD_MSG_ERROR("Unrecognized buffer address provided %p",
			buffer->virt_addr);
		return NULL;
	}

	if (buf_entry->in_use) {
		VCD_MSG_ERROR("An inuse output frame is being provided for "
			"reuse");
		return NULL;
	}

	if (buffer->alloc_len < buf_pool->buf_req.size ||
			buffer->alloc_len > buf_entry->size) {
		VCD_MSG_ERROR("Bad buffer Alloc_len = %d, Actual size = %d, "
			" Min size = %u", buffer->alloc_len, buf_entry->size,
			 buf_pool->buf_req.size);
		return NULL;
	}

	return buf_entry;
}

void vcd_handle_ind_hw_err_fatal(struct vcd_clnt_ctxt *cctxt, u32 event,
	u32 status)
{
	if (cctxt->status.frame_submitted) {
		cctxt->status.frame_submitted--;
		vcd_mark_frame_channel(cctxt->dev_ctxt);
	}
	vcd_handle_err_fatal(cctxt, event, status);
}

void vcd_handle_err_fatal(struct vcd_clnt_ctxt *cctxt, u32 event, u32 status)
{
	u32 rc;
	VCD_MSG_LOW("vcd_handle_err_fatal: event=%x, err=%x", event, status);
	if (!VCD_FAILED_FATAL(status))
		return;

	if (VCD_FAILED_DEVICE_FATAL(status)) {
		vcd_clnt_handle_device_err_fatal(cctxt, event);
		vcd_handle_device_err_fatal(cctxt->dev_ctxt, cctxt);
	} else if (VCD_FAILED_CLIENT_FATAL(status)) {
		cctxt->status.last_evt = event;

		if (cctxt->sched_clnt_valid) {
			rc = vcd_map_sched_status(sched_suspend_resume_client(
				cctxt->dev_ctxt->sched_hdl,
				cctxt->sched_clnt_hdl, false));
			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR("Failed: sched_suspend_resume_"
					"client rc=0x%x", rc);
			}
		}
		cctxt->callback(event, VCD_ERR_HW_FATAL, NULL, 0, cctxt,
						   cctxt->client_data);
		cctxt->status.cleaning_up = true;
		vcd_client_cmd_flush_and_en_q(cctxt, VCD_CMD_CODEC_STOP);
		vcd_do_client_state_transition(cctxt, VCD_CLIENT_STATE_INVALID,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}
}

void vcd_handle_err_in_starting(struct vcd_clnt_ctxt *cctxt, u32 status)
{
	VCD_MSG_LOW("\n vcd_handle_err_in_starting:");
	if (VCD_FAILED_FATAL(status)) {
		vcd_handle_err_fatal(cctxt, VCD_EVT_RESP_START, status);
	} else {
		cctxt->status.last_err = status;
		VCD_MSG_HIGH("\n VCD cleanup: Enqueuing stop cmd");
		vcd_client_cmd_flush_and_en_q(cctxt, VCD_CMD_CODEC_STOP);
	}
}

void vcd_handle_trans_pending(struct vcd_clnt_ctxt *cctxt)
{
	if (!cctxt->status.frame_submitted) {
		VCD_MSG_ERROR("Transaction pending response was not expected");
		vcd_assert();
		return;
	}
	cctxt->status.frame_submitted--;
	cctxt->status.frame_delayed++;
	vcd_mark_frame_channel(cctxt->dev_ctxt);
}

u32 vcd_requeue_input_frame(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt *cctxt, struct vcd_buffer_entry *buf_entry)
{
	u32 rc;
	rc = vcd_map_sched_status(sched_re_queue_frame(dev_ctxt->sched_hdl,
		cctxt->sched_clnt_hdl, (void *) buf_entry));

	VCD_FAILED_RETURN(rc, "Failed: Sched_ReQueueFrame");

	if (buf_entry->frame.flags & VCD_FRAME_FLAG_EOS) {
		rc = vcd_map_sched_status(sched_mark_client_eof(dev_ctxt->
			sched_hdl, cctxt->sched_clnt_hdl));
	}

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("rc = 0x%x: Failed: Sched_MarkClientEOF", rc);

	return rc;
}

void vcd_handle_submit_frame_failed(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_transc *transc)
{
	struct vcd_clnt_ctxt *cctxt = transc->cctxt;
	u32 rc;

	vcd_mark_frame_channel(dev_ctxt);
	transc->in_use = false;

	vcd_handle_err_fatal(cctxt, VCD_EVT_IND_HWERRFATAL,
		VCD_ERR_CLIENT_FATAL);

	if (vcd_get_command_channel(dev_ctxt, &transc)) {
		transc->type = VCD_CMD_CODEC_STOP;
		transc->cctxt = cctxt;
		rc = vcd_submit_cmd_sess_end(transc);
		if (VCD_FAILED(rc)) {
			vcd_release_command_channel(dev_ctxt, transc);
			VCD_MSG_ERROR("rc = 0x%x. Failed: VCD_SubmitCmdSessEnd",
				rc);
		}
	}
}

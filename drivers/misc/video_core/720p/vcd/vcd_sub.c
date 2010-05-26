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

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

u8 *vcd_pmem_get_physical(struct video_client_ctx *client_ctx,
			  unsigned long kernel_vaddr)
{
	unsigned long phy_addr, user_vaddr;
	int pmem_fd;
	struct file *file;
	s32 buffer_index = -1;

	if (vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_INPUT,
					  FALSE, &user_vaddr, &kernel_vaddr,
					  &phy_addr, &pmem_fd, &file,
					  &buffer_index)) {

		return (u8 *) phy_addr;
	} else if (vid_c_lookup_addr_table(client_ctx, BUFFER_TYPE_OUTPUT,
		FALSE, &user_vaddr, &kernel_vaddr, &phy_addr, &pmem_fd, &file,
		&buffer_index)) {
		return (u8 *) phy_addr;
	} else {
		VCD_MSG_ERROR("Couldn't get physical address");

		return NULL;
	}

}

void vcd_reset_device_channels(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	p_dev_ctxt->n_ddl_frame_ch_free = p_dev_ctxt->n_ddl_frame_ch_depth;
	p_dev_ctxt->n_ddl_cmd_ch_free   = p_dev_ctxt->n_ddl_cmd_ch_depth;
	p_dev_ctxt->n_ddl_frame_ch_interim = 0;
	p_dev_ctxt->n_ddl_cmd_ch_interim = 0;
}

u32 vcd_get_command_channel(
	struct vcd_dev_ctxt_type *p_dev_ctxt,
	 struct vcd_transc_type **pp_transc)
{
	u32 b_result = FALSE;

	*pp_transc = NULL;

	if (p_dev_ctxt->n_ddl_cmd_ch_free > 0) {
		if (p_dev_ctxt->b_ddl_cmd_concurrency) {
			--p_dev_ctxt->n_ddl_cmd_ch_free;
			b_result = TRUE;
		} else if ((p_dev_ctxt->n_ddl_frame_ch_free +
			 p_dev_ctxt->n_ddl_frame_ch_interim)
			== p_dev_ctxt->n_ddl_frame_ch_depth) {
				--p_dev_ctxt->n_ddl_cmd_ch_free;
				b_result = TRUE;
		}
	}

	if (b_result) {
		*pp_transc = vcd_get_free_trans_tbl_entry(p_dev_ctxt);

		if (!*pp_transc) {
			b_result = FALSE;

			vcd_release_command_channel(p_dev_ctxt, *pp_transc);
		}

	}
	return b_result;
}

u32 vcd_get_command_channel_in_loop(
	struct vcd_dev_ctxt_type *p_dev_ctxt,
	 struct vcd_transc_type **pp_transc)
{
	u32 b_result = FALSE;

	*pp_transc = NULL;

	if (p_dev_ctxt->n_ddl_cmd_ch_interim > 0) {
		if (p_dev_ctxt->b_ddl_cmd_concurrency) {
			--p_dev_ctxt->n_ddl_cmd_ch_interim;
			b_result = TRUE;
		} else if ((p_dev_ctxt->n_ddl_frame_ch_free +
				p_dev_ctxt->n_ddl_frame_ch_interim)
				== p_dev_ctxt->n_ddl_frame_ch_depth) {
				--p_dev_ctxt->n_ddl_cmd_ch_interim;
				b_result = TRUE;
		}
	} else {
		b_result = vcd_get_command_channel(p_dev_ctxt, pp_transc);
	}

	if (b_result && !*pp_transc) {
		*pp_transc = vcd_get_free_trans_tbl_entry(p_dev_ctxt);

		if (!*pp_transc) {
			b_result = FALSE;

			++p_dev_ctxt->n_ddl_cmd_ch_interim;
		}

	}

	return b_result;
}

void vcd_mark_command_channel(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_transc_type *p_transc)
{
	++p_dev_ctxt->n_ddl_cmd_ch_interim;

	vcd_release_trans_tbl_entry(p_transc);
	if (p_dev_ctxt->n_ddl_cmd_ch_interim +
		p_dev_ctxt->n_ddl_cmd_ch_free >
		p_dev_ctxt->n_ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_command_channel(
	struct vcd_dev_ctxt_type *p_dev_ctxt, struct vcd_transc_type *p_transc)
{
	++p_dev_ctxt->n_ddl_cmd_ch_free;

	vcd_release_trans_tbl_entry(p_transc);
	if (p_dev_ctxt->n_ddl_cmd_ch_interim + p_dev_ctxt->n_ddl_cmd_ch_free >
		p_dev_ctxt->n_ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_multiple_command_channels(struct vcd_dev_ctxt_type
	*p_dev_ctxt, u32 n_channels)
{
	p_dev_ctxt->n_ddl_cmd_ch_free += n_channels;

	if (p_dev_ctxt->n_ddl_cmd_ch_interim +
		p_dev_ctxt->n_ddl_cmd_ch_free >
		p_dev_ctxt->n_ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_interim_command_channels(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	p_dev_ctxt->n_ddl_cmd_ch_free += p_dev_ctxt->n_ddl_cmd_ch_interim;
	p_dev_ctxt->n_ddl_cmd_ch_interim = 0;

	if (p_dev_ctxt->n_ddl_cmd_ch_interim + p_dev_ctxt->n_ddl_cmd_ch_free >
		p_dev_ctxt->n_ddl_cmd_ch_depth) {
		VCD_MSG_ERROR("\n Command channel access counters messed up");
		vcd_assert();
	}
}

u32 vcd_get_frame_channel(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_transc_type **pp_transc)
{
	u32 b_result = FALSE;

	if (p_dev_ctxt->n_ddl_frame_ch_free > 0) {
		if (p_dev_ctxt->b_ddl_cmd_concurrency) {
			--p_dev_ctxt->n_ddl_frame_ch_free;
			b_result = TRUE;
		} else if ((p_dev_ctxt->n_ddl_cmd_ch_free +
			 p_dev_ctxt->n_ddl_cmd_ch_interim)
			== p_dev_ctxt->n_ddl_cmd_ch_depth) {
			--p_dev_ctxt->n_ddl_frame_ch_free;
			b_result = TRUE;
		}
	}

	if (b_result) {
		*pp_transc = vcd_get_free_trans_tbl_entry(p_dev_ctxt);

		if (!*pp_transc) {
			b_result = FALSE;

			vcd_release_frame_channel(p_dev_ctxt, *pp_transc);
		} else {
			(*pp_transc)->e_type = VCD_CMD_CODE_FRAME;
		}

	}

	return b_result;
}

u32 vcd_get_frame_channel_in_loop(
	struct vcd_dev_ctxt_type *p_dev_ctxt,
	 struct vcd_transc_type **pp_transc)
{
	u32 b_result = FALSE;

	*pp_transc = NULL;

	if (p_dev_ctxt->n_ddl_frame_ch_interim > 0) {
		if (p_dev_ctxt->b_ddl_cmd_concurrency) {
			--p_dev_ctxt->n_ddl_frame_ch_interim;
			b_result = TRUE;
		} else if ((p_dev_ctxt->n_ddl_cmd_ch_free +
			 p_dev_ctxt->n_ddl_cmd_ch_interim)
			== p_dev_ctxt->n_ddl_cmd_ch_depth) {
			--p_dev_ctxt->n_ddl_frame_ch_interim;
			b_result = TRUE;
		}
	} else {
		b_result = vcd_get_frame_channel(p_dev_ctxt, pp_transc);
	}

	if (b_result && !*pp_transc) {
		*pp_transc = vcd_get_free_trans_tbl_entry(p_dev_ctxt);

		if (!*pp_transc) {
			b_result = FALSE;
			VCD_MSG_FATAL("\n%s: All transactions are busy;"
				"Couldnt find free one\n", __func__);
			++p_dev_ctxt->n_ddl_frame_ch_interim;
		}

	}

	return b_result;
}

void vcd_mark_frame_channel(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	++p_dev_ctxt->n_ddl_frame_ch_interim;

	if (p_dev_ctxt->n_ddl_frame_ch_interim +
		p_dev_ctxt->n_ddl_frame_ch_free >
		p_dev_ctxt->n_ddl_cmd_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_frame_channel(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_transc_type *p_transc)
{
	++p_dev_ctxt->n_ddl_frame_ch_free;

	vcd_release_trans_tbl_entry(p_transc);

	if (p_dev_ctxt->n_ddl_frame_ch_interim +
		p_dev_ctxt->n_ddl_frame_ch_free >
		p_dev_ctxt->n_ddl_cmd_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_multiple_frame_channels(struct vcd_dev_ctxt_type
	*p_dev_ctxt, u32 n_channels)
{
	p_dev_ctxt->n_ddl_frame_ch_free += n_channels;

	if (p_dev_ctxt->n_ddl_frame_ch_interim +
		p_dev_ctxt->n_ddl_frame_ch_free >
		p_dev_ctxt->n_ddl_frame_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

void vcd_release_interim_frame_channels(struct vcd_dev_ctxt_type
	*p_dev_ctxt)
{
	p_dev_ctxt->n_ddl_frame_ch_free +=
		p_dev_ctxt->n_ddl_frame_ch_interim;
	p_dev_ctxt->n_ddl_frame_ch_interim = 0;

	if (p_dev_ctxt->n_ddl_frame_ch_free >
		p_dev_ctxt->n_ddl_cmd_ch_depth) {
		VCD_MSG_FATAL("Frame channel access counters messed up");
		vcd_assert();
	}
}

u32 vcd_core_is_busy(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	if (((p_dev_ctxt->n_ddl_cmd_ch_free +
		  p_dev_ctxt->n_ddl_cmd_ch_interim) !=
		 p_dev_ctxt->n_ddl_cmd_ch_depth)
		||
		((p_dev_ctxt->n_ddl_frame_ch_free +
		  p_dev_ctxt->n_ddl_frame_ch_interim) !=
		 p_dev_ctxt->n_ddl_frame_ch_depth)
	  ) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void vcd_device_timer_start(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	if (p_dev_ctxt->config.pf_timer_start)
		p_dev_ctxt->config.pf_timer_start(p_dev_ctxt->p_hw_timer_handle,
			p_dev_ctxt->n_hw_time_out);
}

void vcd_device_timer_stop(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	if (p_dev_ctxt->config.pf_timer_stop)
		p_dev_ctxt->config.pf_timer_stop(p_dev_ctxt->p_hw_timer_handle);
}


u32 vcd_common_allocate_set_buffer(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 enum vcd_buffer_type e_buffer,
	 u32 n_buf_size, struct vcd_buffer_pool_type **pp_buf_pool)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_requirement_type Buf_req;
	struct vcd_property_hdr_type Prop_hdr;
	struct vcd_buffer_pool_type *p_buf_pool;

	if (e_buffer == VCD_BUFFER_INPUT) {
		Prop_hdr.prop_id = DDL_I_INPUT_BUF_REQ;
		p_buf_pool = &p_cctxt->in_buf_pool;
	} else if (e_buffer == VCD_BUFFER_OUTPUT) {
		Prop_hdr.prop_id = DDL_I_OUTPUT_BUF_REQ;
		p_buf_pool = &p_cctxt->out_buf_pool;
	} else {
		rc = VCD_ERR_ILLEGAL_PARM;
	}

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided");

	*pp_buf_pool = p_buf_pool;

	if (p_buf_pool->n_count > 0 &&
		p_buf_pool->n_validated == p_buf_pool->n_count) {
		VCD_MSG_ERROR("Buffer pool is full");

		return VCD_ERR_FAIL;
	}

	if (!p_buf_pool->a_entries) {
		Prop_hdr.n_size = sizeof(Buf_req);
		rc = ddl_get_property(p_cctxt->ddl_handle, &Prop_hdr, &Buf_req);

		if (!VCD_FAILED(rc)) {
			rc = vcd_alloc_buffer_pool_entries(p_buf_pool,
							   &Buf_req);

		} else {
			VCD_MSG_ERROR("rc = 0x%x. Failed: ddl_get_property",
					  rc);
		}

	}

	if (!VCD_FAILED(rc)) {
		if (p_buf_pool->buf_req.n_size > n_buf_size) {
			VCD_MSG_ERROR("\n required buffer size %u "
				"allocated size %u", p_buf_pool->buf_req.
				n_size, n_buf_size);

			rc = VCD_ERR_ILLEGAL_PARM;
		}
	}

	return rc;
}

u32 vcd_set_buffer_internal(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_buffer_pool_type *p_buf_pool, u8 *p_buffer, u32 n_buf_size)
{
	struct vcd_buffer_entry_type *p_buf_entry;

	p_buf_entry = vcd_find_buffer_pool_entry(p_buf_pool, p_buffer);
	if (p_buf_entry) {
		VCD_MSG_ERROR("This buffer address already exists");

		return VCD_ERR_ILLEGAL_OP;
	}

	if (((u32) p_buffer % p_buf_pool->buf_req.n_align)) {
		VCD_MSG_ERROR("Provided addr is not aligned");

		return VCD_ERR_BAD_POINTER;
	}

	p_buf_entry = vcd_get_free_buffer_pool_entry(p_buf_pool);
	if (!p_buf_entry) {
		VCD_MSG_ERROR("Can't allocate buffer pool is full");

		return VCD_ERR_FAIL;
	}

	p_buf_entry->p_virtual = p_buffer;

	p_buf_entry->p_physical =
		(u8 *) vcd_pmem_get_physical(p_cctxt->p_client_data,
					 (unsigned long)p_buffer);

	if (!p_buf_entry->p_physical) {
		VCD_MSG_ERROR("Couldn't get physical address");

		return VCD_ERR_BAD_POINTER;
	}

	if (((u32)p_buf_entry->p_physical %
		p_buf_pool->buf_req.n_align)) {
		VCD_MSG_ERROR("Physical addr is not aligned");
		return VCD_ERR_BAD_POINTER ;
	}

	p_buf_entry->n_size = n_buf_size;
	p_buf_entry->frame.n_alloc_len = n_buf_size;
	p_buf_entry->b_allocated = FALSE;

	p_buf_entry->frame.p_virtual = p_buf_entry->p_virtual;
	p_buf_entry->frame.p_physical = p_buf_entry->p_physical;

	p_buf_pool->n_validated++;

	return VCD_S_SUCCESS;

}

u32 vcd_allocate_buffer_internal(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_buffer_pool_type *p_buf_pool,
	 u32 n_buf_size, u8 **pp_vir_buf_addr, u8 **pp_phy_buf_addr)
{
	struct vcd_buffer_entry_type *p_buf_entry;
	struct vcd_buffer_requirement_type *p_buf_req;
	u32 n_addr;
	int rc = 0;

	p_buf_entry = vcd_get_free_buffer_pool_entry(p_buf_pool);
	if (!p_buf_entry) {
		VCD_MSG_ERROR("Can't allocate buffer pool is full");

		return VCD_ERR_FAIL;
	}

	p_buf_req = &p_buf_pool->buf_req;

	n_buf_size += p_buf_req->n_align;

	rc = vcd_pmem_alloc(n_buf_size, &p_buf_entry->p_alloc,
				&p_buf_entry->p_physical);

	if (rc < 0) {
		VCD_MSG_ERROR("Buffer allocation failed");

		return VCD_ERR_ALLOC_FAIL;
	}

	p_buf_entry->n_size = n_buf_size;
	p_buf_entry->frame.n_alloc_len = n_buf_size;

	if (!p_buf_entry->p_physical) {
		VCD_MSG_ERROR("Couldn't get physical address");

		return VCD_ERR_BAD_POINTER;
	}

	p_buf_entry->b_allocated = TRUE;

	if (p_buf_req->n_align > 0) {

		n_addr = (u32) p_buf_entry->p_physical;
		n_addr += p_buf_req->n_align;
		n_addr -= (n_addr % p_buf_req->n_align);
		p_buf_entry->p_virtual = p_buf_entry->p_alloc;
		p_buf_entry->p_virtual += (u32) (n_addr - (u32)
			p_buf_entry->p_physical);
		p_buf_entry->p_physical = (u8 *) n_addr;
	} else {
		VCD_MSG_LOW("No buffer alignment required");

		p_buf_entry->p_virtual = p_buf_entry->p_alloc;

	}

	p_buf_entry->frame.p_virtual = p_buf_entry->p_virtual;
	p_buf_entry->frame.p_physical = p_buf_entry->p_physical;

	*pp_vir_buf_addr = p_buf_entry->p_virtual;
	*pp_phy_buf_addr = p_buf_entry->p_physical;

	p_buf_pool->n_allocated++;
	p_buf_pool->n_validated++;

	return VCD_S_SUCCESS;
}

u32 vcd_free_one_buffer_internal(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 enum vcd_buffer_type e_buffer, u8 *p_buffer)
{
	struct vcd_buffer_pool_type *p_buf_pool;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry_type *p_buf_entry;

	if (e_buffer == VCD_BUFFER_INPUT)
		p_buf_pool = &p_cctxt->in_buf_pool;
	else if (e_buffer == VCD_BUFFER_OUTPUT)
		p_buf_pool = &p_cctxt->out_buf_pool;
	else
		rc = VCD_ERR_ILLEGAL_PARM;

	VCD_FAILED_RETURN(rc, "Invalid buffer type provided");

	p_buf_entry = vcd_find_buffer_pool_entry(p_buf_pool, p_buffer);
	if (!p_buf_entry) {
		VCD_MSG_ERROR("Buffer addr %p not found. Can't free buffer",
				  p_buffer);

		return VCD_ERR_ILLEGAL_PARM;
	}
	if (p_buf_entry->b_in_use) {
		VCD_MSG_ERROR("\n Buffer is in use and is not flushed");
		return VCD_ERR_ILLEGAL_OP;
	}

	VCD_MSG_LOW("Freeing buffer %p. Allocated %d",
			p_buf_entry->p_virtual, p_buf_entry->b_allocated);

	if (p_buf_entry->b_allocated) {
		vcd_pmem_free(p_buf_entry->p_alloc, p_buf_entry->p_physical);

		p_buf_pool->n_allocated--;

	}

	memset(p_buf_entry, 0, sizeof(struct vcd_buffer_entry_type));

	p_buf_pool->n_validated--;

	return VCD_S_SUCCESS;
}

u32 vcd_free_buffers_internal(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_buffer_pool_type *p_buf_pool)
{
	u32 rc = VCD_S_SUCCESS;
	u32 i;

	VCD_MSG_LOW("vcd_free_buffers_internal:");

	if (p_buf_pool->a_entries) {
		for (i = 1; i <= p_buf_pool->n_count; i++) {
			if (p_buf_pool->a_entries[i].b_valid &&
				p_buf_pool->a_entries[i].b_allocated) {
				vcd_pmem_free(p_buf_pool->a_entries[i].p_alloc,
						  p_buf_pool->a_entries[i].
						  p_physical);
			}
		}

	}

	vcd_reset_buffer_pool_for_reuse(p_buf_pool);

	return rc;
}

u32 vcd_alloc_buffer_pool_entries(
	struct vcd_buffer_pool_type *p_buf_pool,
	 struct vcd_buffer_requirement_type *p_buf_req)
{

	VCD_MSG_LOW("vcd_alloc_buffer_pool_entries:");

	p_buf_pool->buf_req = *p_buf_req;

	p_buf_pool->n_count = p_buf_req->n_actual_count;
	p_buf_pool->a_entries = (struct vcd_buffer_entry_type *)
		vcd_malloc(sizeof(struct vcd_buffer_entry_type) *
			   (p_buf_pool->n_count + 1));

	if (!p_buf_pool->a_entries) {
		VCD_MSG_ERROR("Buf_pool entries alloc failed");

		return VCD_ERR_ALLOC_FAIL;
	}

	p_buf_pool->a_queue = (struct vcd_buffer_entry_type **)
		vcd_malloc(sizeof(struct vcd_buffer_entry_type *) *
			   p_buf_pool->n_count);

	if (!p_buf_pool->a_queue) {
		VCD_MSG_ERROR("Buf_pool queue alloc failed");

		vcd_free(p_buf_pool->a_entries);

		return VCD_ERR_ALLOC_FAIL;
	}

	memset(p_buf_pool->a_entries, 0,
		sizeof(struct vcd_buffer_entry_type) *
		(p_buf_pool->n_count + 1));

	memset(p_buf_pool->a_queue,
		0, sizeof(struct vcd_buffer_entry_type *) *
		p_buf_pool->n_count);

	p_buf_pool->a_entries[0].b_valid = TRUE;

	p_buf_pool->n_q_head = 0;
	p_buf_pool->n_q_tail = (u16) (p_buf_pool->n_count - 1);
	p_buf_pool->n_q_len = 0;

	p_buf_pool->n_validated = 0;
	p_buf_pool->n_allocated = 0;
	p_buf_pool->n_in_use = 0;

	return VCD_S_SUCCESS;
}

void vcd_free_buffer_pool_entries(struct vcd_buffer_pool_type *p_buf_pool)
{
	VCD_MSG_LOW("vcd_free_buffer_pool_entries:");

	if (p_buf_pool->a_entries) {
		vcd_free(p_buf_pool->a_entries);
		vcd_free(p_buf_pool->a_queue);
	}

	memset(p_buf_pool, 0, sizeof(struct vcd_buffer_pool_type));
}

void vcd_flush_in_use_buffer_pool_entries(struct vcd_clnt_ctxt_type_t *p_cctxt,
	struct vcd_buffer_pool_type *p_buf_pool, u32 event)
{
	u32 i;
	VCD_MSG_LOW("vcd_flush_buffer_pool_entries: event=0x%x", event);

	if (p_buf_pool->a_entries) {
		for (i = 0; i <= p_buf_pool->n_count; i++) {
			if (p_buf_pool->a_entries[i].p_virtual &&
				p_buf_pool->a_entries[i].b_in_use) {
				p_cctxt->callback(event, VCD_S_SUCCESS,
					&p_buf_pool->a_entries[i].frame,
					sizeof(struct vcd_frame_data_type),
					p_cctxt, p_cctxt->p_client_data);
				p_buf_pool->a_entries[i].b_in_use = FALSE;
				VCD_BUFFERPOOL_INUSE_DECREMENT(
					p_buf_pool->n_in_use);
		 }
		}
	}
}


void vcd_reset_buffer_pool_for_reuse(struct vcd_buffer_pool_type *p_buf_pool)
{
	VCD_MSG_LOW("vcd_reset_buffer_pool_for_reuse:");

	memset(&p_buf_pool->a_entries[1], 0,
		sizeof(struct vcd_buffer_entry_type) *
		p_buf_pool->n_count);
	memset(p_buf_pool->a_queue,
		0, sizeof(struct vcd_buffer_entry_type *) *
		p_buf_pool->n_count);

	p_buf_pool->n_q_head = 0;
	p_buf_pool->n_q_tail = (u16) (p_buf_pool->n_count - 1);
	p_buf_pool->n_q_len = 0;

	p_buf_pool->n_validated = 0;
	p_buf_pool->n_allocated = 0;
	p_buf_pool->n_in_use = 0;

}

struct vcd_buffer_entry_type *vcd_get_free_buffer_pool_entry
	(struct vcd_buffer_pool_type *p_pool) {
	u32 i;

	i = 1;
	while (i <= p_pool->n_count && p_pool->a_entries[i].b_valid)
		i++;


	if (i <= p_pool->n_count) {
		p_pool->a_entries[i].b_valid = TRUE;

		return &p_pool->a_entries[i];
	} else {
		return NULL;
	}
}

struct vcd_buffer_entry_type *vcd_find_buffer_pool_entry
	(struct vcd_buffer_pool_type *p_pool, u8 *p_v_addr)
{
	u32 i;
	u32 b_found = FALSE;

	for (i = 0; i <= p_pool->n_count && !b_found; i++) {
		if (p_pool->a_entries[i].p_virtual == p_v_addr)
			b_found = TRUE;

	}

	if (b_found)
		return &p_pool->a_entries[i - 1];
	else
		return NULL;

}

u32 vcd_buffer_pool_entry_en_q(
	struct vcd_buffer_pool_type *p_pool,
	 struct vcd_buffer_entry_type *p_entry)
{
	u16 i;
	u16 n_q_cntr;
	u32 b_found = FALSE;

	if (p_pool->n_q_len == p_pool->n_count)
		return FALSE;

	for (i = 0, n_q_cntr = p_pool->n_q_head;
		 !b_found && i < p_pool->n_q_len;
		 i++, n_q_cntr = (n_q_cntr + 1) % p_pool->n_count) {
		if (p_pool->a_queue[n_q_cntr] == p_entry)
			b_found = TRUE;
	}

	if (b_found) {
		VCD_MSG_HIGH("\n this output buffer is already present"
			" in queue");
		VCD_MSG_HIGH("\n Vir Addr %p Phys Addr %p",
			p_entry->p_virtual, p_entry->p_physical);
		return FALSE;
	}

	p_pool->n_q_tail = (p_pool->n_q_tail + 1) % p_pool->n_count;
	p_pool->n_q_len++;
	p_pool->a_queue[p_pool->n_q_tail] = p_entry;

	return TRUE;
}

struct vcd_buffer_entry_type *vcd_buffer_pool_entry_de_q
	(struct vcd_buffer_pool_type *p_pool) {
	struct vcd_buffer_entry_type *p_entry;

	if (!p_pool || !p_pool->n_q_len)
		return NULL;

	p_entry = p_pool->a_queue[p_pool->n_q_head];
	p_pool->n_q_head = (p_pool->n_q_head + 1) % p_pool->n_count;
	p_pool->n_q_len--;

	return p_entry;
}

void vcd_flush_output_buffers(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_buffer_pool_type *p_buf_pool;
	struct vcd_buffer_entry_type *p_buf_entry;
	u32 n_count = 0;
	struct vcd_property_hdr_type prop_hdr;

	VCD_MSG_LOW("vcd_flush_output_buffers:");

	p_buf_pool = &p_cctxt->out_buf_pool;

	p_buf_entry = vcd_buffer_pool_entry_de_q(p_buf_pool);
	while (p_buf_entry) {
		if (!p_cctxt->b_decoding || p_buf_entry->b_in_use) {
			p_buf_entry->frame.n_data_len = 0;

			p_cctxt->callback(VCD_EVT_RESP_OUTPUT_FLUSHED,
					  VCD_S_SUCCESS,
					  &p_buf_entry->frame,
					  sizeof(struct vcd_frame_data_type),
					  p_cctxt, p_cctxt->p_client_data);

			p_buf_entry->b_in_use = FALSE;

			n_count++;
		}

		p_buf_entry = vcd_buffer_pool_entry_de_q(p_buf_pool);
	}
	p_buf_pool->n_in_use = 0;

	if (p_cctxt->b_sched_clnt_valid && n_count > 0) {
		VCD_MSG_LOW("Updating scheduler O tkns = %u", n_count);

		(void)sched_update_client_o_tkn(p_cctxt->p_dev_ctxt->sched_hdl,
						p_cctxt->sched_clnt_hdl,
						FALSE,
						n_count *
						p_cctxt->
						n_sched_o_tkn_per_ip_frm);
	}

	if (p_cctxt->b_ddl_hdl_valid && p_cctxt->b_decoding) {
		prop_hdr.prop_id = DDL_I_REQ_OUTPUT_FLUSH;
		prop_hdr.n_size = sizeof(u32);
		n_count = 0x1;

		(void)ddl_set_property(p_cctxt->ddl_handle, &prop_hdr,
					   &n_count);
	}
}

u32 vcd_flush_buffers(struct vcd_clnt_ctxt_type_t *p_cctxt, u32 n_mode)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry_type *p_buf_entry;

	VCD_MSG_LOW("vcd_flush_buffers:");

	if (n_mode > VCD_FLUSH_ALL || !(n_mode & VCD_FLUSH_ALL)) {
		VCD_MSG_ERROR("Invalid flush mode %d", n_mode);

		return VCD_ERR_ILLEGAL_PARM;
	}

	VCD_MSG_MED("Flush mode %d requested", n_mode);

	if ((n_mode & VCD_FLUSH_INPUT) && p_cctxt->b_sched_clnt_valid) {
		rc = vcd_map_sched_status(sched_flush_client_buffer
					  (p_dev_ctxt->sched_hdl,
					   p_cctxt->sched_clnt_hdl,
					   (void **)&p_buf_entry));

		while (!VCD_FAILED(rc) &&
			   rc != VCD_S_SCHED_QEMPTY && p_buf_entry) {
			if (p_buf_entry->p_virtual) {
				p_cctxt->callback(VCD_EVT_RESP_INPUT_FLUSHED,
						  VCD_S_SUCCESS,
						  &p_buf_entry->frame,
						  sizeof(struct
							 vcd_frame_data_type),
						  p_cctxt,
						  p_cctxt->p_client_data);

			}

			p_buf_entry->b_in_use = FALSE;
			VCD_BUFFERPOOL_INUSE_DECREMENT(
				p_cctxt->in_buf_pool.n_in_use);
			p_buf_entry = NULL;
			rc = vcd_map_sched_status(sched_flush_client_buffer
						  (p_dev_ctxt->sched_hdl,
						   p_cctxt->sched_clnt_hdl,
						   (void **)&p_buf_entry));
		}

	}
	VCD_FAILED_RETURN(rc, "Failed: sched_flush_client_buffer");

	if (p_cctxt->status.n_frame_submitted > 0) {

		p_cctxt->status.n_flush_mode |= n_mode;

	} else {

		if (n_mode & VCD_FLUSH_OUTPUT) {
			vcd_flush_output_buffers(p_cctxt);
			vcd_release_all_clnt_frm_transc(p_cctxt);
		}

	}

	return VCD_S_SUCCESS;
}

void vcd_flush_buffers_in_err_fatal(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	VCD_MSG_LOW("\n vcd_flush_buffers_in_err_fatal:");
	(void) vcd_flush_buffers(p_cctxt, VCD_FLUSH_ALL);
	vcd_flush_in_use_buffer_pool_entries(p_cctxt,
		&p_cctxt->in_buf_pool, VCD_EVT_RESP_INPUT_FLUSHED);
	vcd_flush_in_use_buffer_pool_entries(p_cctxt,
		&p_cctxt->out_buf_pool,	VCD_EVT_RESP_OUTPUT_FLUSHED);
	p_cctxt->status.n_flush_mode = VCD_FLUSH_ALL;
	vcd_send_flush_done(p_cctxt, VCD_S_SUCCESS);
}

u32 vcd_init_client_context(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc;

	VCD_MSG_LOW("vcd_init_client_context:");

	rc = ddl_open(&p_cctxt->ddl_handle, p_cctxt->b_decoding);

	VCD_FAILED_RETURN(rc, "Failed: ddl_open");
	p_cctxt->b_ddl_hdl_valid = TRUE;

	p_cctxt->clnt_state.e_state = VCD_CLIENT_STATE_OPEN;
	p_cctxt->clnt_state.p_state_table =
		vcd_get_client_state_table(VCD_CLIENT_STATE_OPEN);

	p_cctxt->n_signature = VCD_SIGNATURE;
	p_cctxt->b_live = TRUE;

	p_cctxt->cmd_q.e_pending_cmd = VCD_CMD_NONE;

	return rc;
}

void vcd_destroy_client_context(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt;
	struct vcd_clnt_ctxt_type_t *p_client;
	u32 rc = VCD_S_SUCCESS;
	int n_idx;

	VCD_MSG_LOW("vcd_destroy_client_context:");

	p_dev_ctxt = p_cctxt->p_dev_ctxt;

	if (p_cctxt == p_dev_ctxt->p_cctxt_list_head) {
		VCD_MSG_MED("Clnt list head clnt being removed");

		p_dev_ctxt->p_cctxt_list_head = p_cctxt->p_next;
	} else {
		p_client = p_dev_ctxt->p_cctxt_list_head;
		while (p_client && p_cctxt != p_client->p_next)
			p_client = p_client->p_next;

		if (p_client)
			p_client->p_next = p_cctxt->p_next;

		if (!p_client) {
			rc = VCD_ERR_FAIL;

			VCD_MSG_ERROR("Client not found in client list");
		}
	}

	if (VCD_FAILED(rc))
		return;

	if (p_cctxt->b_sched_clnt_valid) {
		rc = VCD_S_SUCCESS;
		while (!VCD_FAILED(rc) && rc != VCD_S_SCHED_QEMPTY) {

			rc = vcd_map_sched_status(sched_flush_client_buffer
						  (p_dev_ctxt->sched_hdl,
						   p_cctxt->sched_clnt_hdl,
						   (void *)&n_idx));
			if (VCD_FAILED(rc))
				VCD_MSG_ERROR("\n Failed: "
					"sched_flush_client_buffer");
		}

		rc = vcd_map_sched_status(sched_remove_client
					  (p_dev_ctxt->sched_hdl,
					   p_cctxt->sched_clnt_hdl));
		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("\n Failed: sched_remove_client");

		p_cctxt->b_sched_clnt_valid = FALSE;
	}

	if (p_cctxt->seq_hdr.p_sequence_header) {
		vcd_pmem_free(p_cctxt->seq_hdr.p_sequence_header,
				  p_cctxt->p_seq_hdr_phy_addr);
		p_cctxt->seq_hdr.p_sequence_header = NULL;
	}

	vcd_free_buffers_internal(p_cctxt, &p_cctxt->in_buf_pool);
	vcd_free_buffers_internal(p_cctxt, &p_cctxt->out_buf_pool);
	vcd_free_buffer_pool_entries(&p_cctxt->in_buf_pool);
	vcd_free_buffer_pool_entries(&p_cctxt->out_buf_pool);
	vcd_release_all_clnt_transc(p_cctxt);

	if (p_cctxt->b_ddl_hdl_valid) {
		(void)ddl_close(&p_cctxt->ddl_handle);
		p_cctxt->b_ddl_hdl_valid = FALSE;
	}
	vcd_free(p_cctxt);
}

u32 vcd_check_for_client_context(
	struct vcd_dev_ctxt_type *p_dev_ctxt, s32 driver_id)
{
	struct vcd_clnt_ctxt_type_t *p_client;

	p_client = p_dev_ctxt->p_cctxt_list_head;
	while (p_client && p_client->driver_id != driver_id)
		p_client = p_client->p_next;

	if (!p_client)
		return FALSE;
	else
		return TRUE;
}

u32 vcd_validate_driver_handle(
	struct vcd_dev_ctxt_type *p_dev_ctxt, s32 driver_handle)
{
	driver_handle--;

	if (driver_handle < 0 ||
		driver_handle >= VCD_DRIVER_INSTANCE_MAX ||
		!p_dev_ctxt->b_driver_ids[driver_handle]) {
		return FALSE;
	} else {
		return TRUE;
	}
}

u32 vcd_client_cmd_en_q(
	struct vcd_clnt_ctxt_type_t *p_cctxt, enum vcd_command_type command)
{
	u32 b_result;

	if (p_cctxt->cmd_q.e_pending_cmd == VCD_CMD_NONE) {
		p_cctxt->cmd_q.e_pending_cmd = command;
		b_result = TRUE;
	} else {
		b_result = FALSE;
	}

	return b_result;
}

void vcd_client_cmd_flush_and_en_q(
	struct vcd_clnt_ctxt_type_t *p_cctxt, enum vcd_command_type command)
{
	p_cctxt->cmd_q.e_pending_cmd = command;
}

u32 vcd_client_cmd_de_q(struct vcd_clnt_ctxt_type_t *p_cctxt,
	enum vcd_command_type *p_command)
{
	if (p_cctxt->cmd_q.e_pending_cmd == VCD_CMD_NONE)
		return FALSE;

	*p_command = p_cctxt->cmd_q.e_pending_cmd;
	p_cctxt->cmd_q.e_pending_cmd = VCD_CMD_NONE;

	return TRUE;
}

u32 vcd_get_next_queued_client_cmd(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_clnt_ctxt_type_t **p_cctxt, enum vcd_command_type *p_command)
{
	struct vcd_clnt_ctxt_type_t *p_client = p_dev_ctxt->p_cctxt_list_head;
	u32 b_result = FALSE;

	while (p_client && !b_result) {
		*p_cctxt = p_client;
		b_result = vcd_client_cmd_de_q(p_client, p_command);
		p_client = p_client->p_next;
	}
	return b_result;
}

u32 vcd_map_sched_status(enum sched_status_type sched_status)
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

u32 vcd_submit_cmd_sess_start(struct vcd_transc_type *p_transc)
{
	u32 rc;
	struct vcd_sequence_hdr_type Seq_hdr;

	VCD_MSG_LOW("vcd_submit_cmd_sess_start:");

	if (p_transc->p_cctxt->b_decoding) {

		if (p_transc->p_cctxt->seq_hdr.p_sequence_header) {
			Seq_hdr.n_sequence_header_len =
				p_transc->p_cctxt->seq_hdr.
				n_sequence_header_len;
			Seq_hdr.p_sequence_header =
				p_transc->p_cctxt->p_seq_hdr_phy_addr;

			rc = ddl_decode_start(p_transc->p_cctxt->ddl_handle,
						  &Seq_hdr, (void *)p_transc);
		} else {
			rc = ddl_decode_start(p_transc->p_cctxt->ddl_handle,
						  NULL, (void *)p_transc);
		}

	} else {
		rc = ddl_encode_start(p_transc->p_cctxt->ddl_handle,
					  (void *)p_transc);
	}
	if (!VCD_FAILED(rc)) {
		p_transc->p_cctxt->status.n_cmd_submitted++;
		vcd_device_timer_start(p_transc->p_cctxt->p_dev_ctxt);
	} else
		VCD_MSG_ERROR("rc = 0x%x. Failed: ddl start", rc);

	return rc;
}

u32 vcd_submit_cmd_sess_end(struct vcd_transc_type *p_transc)
{
	u32 rc;

	VCD_MSG_LOW("vcd_submit_cmd_sess_end:");

	if (p_transc->p_cctxt->b_decoding) {
		rc = ddl_decode_end(p_transc->p_cctxt->ddl_handle,
					(void *)p_transc);
	} else {
		rc = ddl_encode_end(p_transc->p_cctxt->ddl_handle,
					(void *)p_transc);
	}
	if (!VCD_FAILED(rc)) {
		p_transc->p_cctxt->status.n_cmd_submitted++;
		vcd_device_timer_start(p_transc->p_cctxt->p_dev_ctxt);
	} else
		VCD_MSG_ERROR("rc = 0x%x. Failed: ddl end", rc);

	return rc;
}

void vcd_submit_cmd_client_close(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	(void) ddl_close(&p_cctxt->ddl_handle);
	p_cctxt->b_ddl_hdl_valid = FALSE;
	p_cctxt->status.b_cleaning_up = FALSE;
	if (p_cctxt->status.b_close_pending) {
		vcd_destroy_client_context(p_cctxt);
		vcd_handle_for_last_clnt_close(p_cctxt->p_dev_ctxt, TRUE);
	}
}

u32 vcd_submit_command_in_continue(struct vcd_dev_ctxt_type
	*p_dev_ctxt, struct vcd_transc_type *p_transc)
{
	struct vcd_property_hdr_type   prop_hdr;
	struct vcd_clnt_ctxt_type_t *p_client = NULL;
	enum vcd_command_type cmd = VCD_CMD_NONE;
	u32 rc = VCD_ERR_FAIL;
	u32 b_result = FALSE, n_flush = 0, event = 0;
	u32 b_break = FALSE;

	VCD_MSG_LOW("\n vcd_submit_command_in_continue:");

	while (!b_break) {
		b_result = vcd_get_next_queued_client_cmd(p_dev_ctxt,
			&p_client, &cmd);

		if (!b_result)
			b_break = TRUE;
		else {
			p_transc->e_type = cmd;
			p_transc->p_cctxt = p_client;

		 switch (cmd) {
		 case VCD_CMD_CODEC_START:
			{
				rc = vcd_submit_cmd_sess_start(p_transc);
				event = VCD_EVT_RESP_START;
				break;
			}
		 case VCD_CMD_CODEC_STOP:
			{
				rc = vcd_submit_cmd_sess_end(p_transc);
				event = VCD_EVT_RESP_STOP;
				break;
			}
		 case VCD_CMD_OUTPUT_FLUSH:
			{
				prop_hdr.prop_id = DDL_I_REQ_OUTPUT_FLUSH;
				prop_hdr.n_size = sizeof(u32);
				n_flush = 0x1;
				(void) ddl_set_property(p_client->ddl_handle,
						 &prop_hdr, &n_flush);
				vcd_release_command_channel(p_dev_ctxt,
					p_transc);
				rc = VCD_S_SUCCESS;
				break;
			}
		 case VCD_CMD_CLIENT_CLOSE:
			{
				vcd_submit_cmd_client_close(p_client);
				vcd_release_command_channel(p_dev_ctxt,
					p_transc);
				rc = VCD_S_SUCCESS;
				break;
			}
		 default:
			{
				VCD_MSG_ERROR("\n vcd_submit_command: Unknown"
					"command %d", (int)cmd);
				vcd_assert();
				break;
			}
		 }

		 if (!VCD_FAILED(rc)) {
			b_break = TRUE;
		 } else	{
			VCD_MSG_ERROR("vcd_submit_command %d: failed 0x%x",
				cmd, rc);
			p_client->callback(event, rc, NULL, 0, p_client,
				p_client->p_client_data);
		 }
	  }
	}
	return b_result;
}

u32 vcd_schedule_frame(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_clnt_ctxt_type_t **pp_cctxt, struct vcd_buffer_entry_type
	**pp_ip_buf_entry)
{
	u32 rc = VCD_S_SUCCESS;
	VCD_MSG_LOW("vcd_schedule_frame:");

	if (!p_dev_ctxt->p_cctxt_list_head) {
		VCD_MSG_HIGH("Client list empty");
		return FALSE;
	}

	rc = vcd_map_sched_status(sched_de_queue_frame(p_dev_ctxt->sched_hdl,
		(void **) pp_ip_buf_entry, (void **) pp_cctxt));
	if (VCD_FAILED(rc)) {
		VCD_MSG_FATAL("vcd_submit_frame: sched_de_queue_frame"
			"failed 0x%x", rc);
	  return FALSE;
	}

	if (rc == VCD_S_SCHED_QEMPTY) {
		VCD_MSG_HIGH("No frame available. Sched queues are empty");
		return FALSE;
	}

	if (!*pp_cctxt || !*pp_ip_buf_entry) {
		VCD_MSG_FATAL("Sched returned invalid values. ctxt=%p,"
			"ipbuf=%p",	*pp_cctxt, *pp_ip_buf_entry);
		return FALSE;
	}

	if (rc == VCD_S_SCHED_EOS)
		(*pp_ip_buf_entry)->frame.n_flags |= VCD_FRAME_FLAG_EOS;

	return TRUE;
}

void vcd_try_submit_frame(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	struct vcd_transc_type *p_transc;
	u32 rc = VCD_S_SUCCESS;
	struct vcd_clnt_ctxt_type_t *p_cctxt = NULL;
	struct vcd_buffer_entry_type *p_ip_buf_entry = NULL;
	u32 b_result = FALSE;

	VCD_MSG_LOW("vcd_try_submit_frame:");

	if (!vcd_get_frame_channel(p_dev_ctxt, &p_transc))
		return;

	if (!vcd_schedule_frame(p_dev_ctxt, &p_cctxt, &p_ip_buf_entry)) {
		vcd_release_frame_channel(p_dev_ctxt, p_transc);
		return;
	}

	rc = vcd_power_event(p_dev_ctxt, p_cctxt, VCD_EVT_PWR_CLNT_CMD_BEGIN);

	if (!VCD_FAILED(rc)) {
		p_transc->p_cctxt = p_cctxt;
		p_transc->p_ip_buf_entry = p_ip_buf_entry;

		b_result = vcd_submit_frame(p_dev_ctxt, p_transc);
	} else {
		VCD_MSG_ERROR("Failed: VCD_EVT_PWR_CLNT_CMD_BEGIN");

		(void) vcd_requeue_input_frame(p_dev_ctxt, p_cctxt,
				p_ip_buf_entry);

		(void) vcd_map_sched_status(sched_update_client_o_tkn(
			p_dev_ctxt->sched_hdl, p_cctxt->sched_clnt_hdl,
			TRUE, p_cctxt->n_sched_o_tkn_per_ip_frm));
	}

	if (!b_result) {
		vcd_release_frame_channel(p_dev_ctxt, p_transc);
		(void) vcd_power_event(p_dev_ctxt, p_cctxt,
				VCD_EVT_PWR_CLNT_CMD_FAIL);
	}
}

u32 vcd_submit_frame(struct vcd_dev_ctxt_type *p_dev_ctxt,
					 struct vcd_transc_type *p_transc)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt = NULL;
	struct vcd_frame_data_type *p_ip_frm_entry;
	struct vcd_buffer_entry_type *p_op_buf_entry = NULL;
	u32 rc = VCD_S_SUCCESS;
	u32 evcode = 0;
	struct ddl_frame_data_type_tag ddl_ip_frm;
	struct ddl_frame_data_type_tag ddl_op_frm;

	VCD_MSG_LOW("vcd_submit_frame:");
	p_cctxt = p_transc->p_cctxt;
	p_ip_frm_entry = &p_transc->p_ip_buf_entry->frame;

	p_transc->p_op_buf_entry = p_op_buf_entry;
	p_transc->n_ip_frm_tag = p_ip_frm_entry->n_ip_frm_tag;
	p_transc->time_stamp = p_ip_frm_entry->time_stamp;
	p_ip_frm_entry->n_ip_frm_tag = (u32) p_transc;
	memset(&ddl_ip_frm, 0, sizeof(ddl_ip_frm));
	memset(&ddl_op_frm, 0, sizeof(ddl_op_frm));
	if (p_cctxt->b_decoding) {
		evcode = CLIENT_STATE_EVENT_NUMBER(pf_decode_frame);
		ddl_ip_frm.vcd_frm = *p_ip_frm_entry;
		rc = ddl_decode_frame(p_cctxt->ddl_handle, &ddl_ip_frm,
							   (void *) p_transc);
	} else {
		p_op_buf_entry = vcd_buffer_pool_entry_de_q(
			&p_cctxt->out_buf_pool);
		if (!p_op_buf_entry) {
			VCD_MSG_ERROR("Sched provided frame when no"
				"op buffer was present");
			rc = VCD_ERR_FAIL;
		} else {
			p_op_buf_entry->b_in_use = TRUE;
			p_cctxt->out_buf_pool.n_in_use++;
			ddl_ip_frm.vcd_frm = *p_ip_frm_entry;
			ddl_ip_frm.n_frm_delta =
				vcd_calculate_frame_delta(p_cctxt,
					p_ip_frm_entry);

			ddl_op_frm.vcd_frm = p_op_buf_entry->frame;

			evcode = CLIENT_STATE_EVENT_NUMBER(pf_encode_frame);

			rc = ddl_encode_frame(p_cctxt->ddl_handle,
				&ddl_ip_frm, &ddl_op_frm, (void *) p_transc);
		}
	}
	p_ip_frm_entry->n_ip_frm_tag = p_transc->n_ip_frm_tag;
	if (!VCD_FAILED(rc)) {
		vcd_device_timer_start(p_dev_ctxt);
		p_cctxt->status.n_frame_submitted++;
		if (p_ip_frm_entry->n_flags & VCD_FRAME_FLAG_EOS)
			vcd_do_client_state_transition(p_cctxt,
				VCD_CLIENT_STATE_EOS, evcode);
	} else {
		VCD_MSG_ERROR("Frame submission failed. rc = 0x%x", rc);
		vcd_handle_submit_frame_failed(p_dev_ctxt, p_transc);
	}
	return TRUE;
}

u32 vcd_try_submit_frame_in_continue(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_transc_type *p_transc)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt = NULL;
	struct vcd_buffer_entry_type *p_ip_buf_entry = NULL;

	VCD_MSG_LOW("vcd_try_submit_frame_in_continue:");

	if (!vcd_schedule_frame(p_dev_ctxt, &p_cctxt, &p_ip_buf_entry))
		return FALSE;

	p_transc->p_cctxt = p_cctxt;
	p_transc->p_ip_buf_entry = p_ip_buf_entry;

	return vcd_submit_frame(p_dev_ctxt, p_transc);
}

u32 vcd_process_cmd_sess_start(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_transc_type *p_transc;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_process_cmd_sess_start:");
	if (vcd_get_command_channel(p_cctxt->p_dev_ctxt, &p_transc)) {
		rc = vcd_power_event(p_cctxt->p_dev_ctxt,
					 p_cctxt, VCD_EVT_PWR_CLNT_CMD_BEGIN);

		if (!VCD_FAILED(rc)) {
			p_transc->e_type = VCD_CMD_CODEC_START;
			p_transc->p_cctxt = p_cctxt;
			rc = vcd_submit_cmd_sess_start(p_transc);
		} else {
			VCD_MSG_ERROR("Failed: VCD_EVT_PWR_CLNT_CMD_BEGIN");
		}

		if (VCD_FAILED(rc)) {
			vcd_release_command_channel(p_cctxt->p_dev_ctxt,
							p_transc);
		}
	} else {
		u32 b_result;

		b_result = vcd_client_cmd_en_q(p_cctxt, VCD_CMD_CODEC_START);
		if (!b_result) {
			rc = VCD_ERR_BUSY;
			VCD_MSG_ERROR("%s(): vcd_client_cmd_en_q() "
				"failed\n", __func__);
			vcd_assert();
		}
	}

	if (VCD_FAILED(rc)) {
		(void)vcd_power_event(p_cctxt->p_dev_ctxt,
					  p_cctxt, VCD_EVT_PWR_CLNT_CMD_FAIL);
	}

	return rc;
}

void vcd_send_frame_done_in_eos(struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_frame_data_type *p_input_frame, u32 valid_opbuf)
{
	VCD_MSG_LOW("vcd_send_frame_done_in_eos:");

	if (!p_input_frame->p_virtual && !valid_opbuf) {
		VCD_MSG_MED("Sending NULL output with EOS");

		p_cctxt->out_buf_pool.a_entries[0].frame.n_flags =
			VCD_FRAME_FLAG_EOS;
		p_cctxt->out_buf_pool.a_entries[0].frame.n_data_len = 0;
		p_cctxt->out_buf_pool.a_entries[0].frame.time_stamp =
			p_input_frame->time_stamp;
		p_cctxt->out_buf_pool.a_entries[0].frame.n_ip_frm_tag =
			p_input_frame->n_ip_frm_tag;

		p_cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE,
				  VCD_S_SUCCESS,
				  &p_cctxt->out_buf_pool.a_entries[0].frame,
				  sizeof(struct vcd_frame_data_type),
				  p_cctxt, p_cctxt->p_client_data);

		memset(&p_cctxt->out_buf_pool.a_entries[0].frame,
			   0, sizeof(struct vcd_frame_data_type));
	} else if (!p_input_frame->n_data_len) {
		if (p_cctxt->b_decoding) {
			vcd_send_frame_done_in_eos_for_dec(p_cctxt,
							   p_input_frame);
		} else {
			vcd_send_frame_done_in_eos_for_enc(p_cctxt,
							   p_input_frame);
		}

	}
}

void vcd_send_frame_done_in_eos_for_dec(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_frame_data_type *p_input_frame)
{
	struct vcd_buffer_entry_type *p_buf_entry;
	struct vcd_property_hdr_type prop_hdr;
	u32 rc;
	struct ddl_frame_data_type_tag ddl_frm;

	prop_hdr.prop_id = DDL_I_DPB_RETRIEVE;
	prop_hdr.n_size = sizeof(struct ddl_frame_data_type_tag);
	memset(&ddl_frm, 0, sizeof(ddl_frm));
	rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr, &ddl_frm);

	if (VCD_FAILED(rc) || !ddl_frm.vcd_frm.p_virtual) {
		p_cctxt->status.eos_trig_ip_frm = *p_input_frame;

		p_cctxt->status.b_eos_wait_for_op_buf = TRUE;

		return;
	}

	p_buf_entry = vcd_find_buffer_pool_entry(&p_cctxt->out_buf_pool,
		ddl_frm.vcd_frm.p_virtual);
	if (!p_buf_entry) {
		VCD_MSG_ERROR("Unrecognized buffer address provided = %p",
				  ddl_frm.vcd_frm.p_virtual);

		vcd_assert();
	} else {
		(void)
			vcd_map_sched_status(sched_update_client_o_tkn
					 (p_cctxt->p_dev_ctxt->sched_hdl,
					  p_cctxt->sched_clnt_hdl, FALSE,
					  p_cctxt->n_sched_o_tkn_per_ip_frm));

		VCD_MSG_MED("Sending non-NULL output with EOS");

		p_buf_entry->frame.n_data_len = 0;
		p_buf_entry->frame.n_offset = 0;
		p_buf_entry->frame.n_flags |= VCD_FRAME_FLAG_EOS;
		p_buf_entry->frame.n_ip_frm_tag = p_input_frame->n_ip_frm_tag;
		p_buf_entry->frame.time_stamp = p_input_frame->time_stamp;

		p_cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE,
				  VCD_S_SUCCESS,
				  &p_buf_entry->frame,
				  sizeof(struct vcd_frame_data_type),
				  p_cctxt, p_cctxt->p_client_data);

		p_buf_entry->b_in_use = FALSE;
		VCD_BUFFERPOOL_INUSE_DECREMENT(p_cctxt->out_buf_pool.n_in_use);
	}
}

void vcd_send_frame_done_in_eos_for_enc(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_frame_data_type *p_input_frame)
{
	struct vcd_buffer_entry_type *p_op_buf_entry;

	if (!p_cctxt->out_buf_pool.n_q_len) {
		p_cctxt->status.eos_trig_ip_frm = *p_input_frame;

		p_cctxt->status.b_eos_wait_for_op_buf = TRUE;

		return;
	}

	p_op_buf_entry = vcd_buffer_pool_entry_de_q(&p_cctxt->out_buf_pool);
	if (!p_op_buf_entry) {
		VCD_MSG_ERROR("%s(): vcd_buffer_pool_entry_de_q() "
			"failed\n", __func__);
		vcd_assert();
	} else {
		(void)
			vcd_map_sched_status(sched_update_client_o_tkn
					 (p_cctxt->p_dev_ctxt->sched_hdl,
					  p_cctxt->sched_clnt_hdl, FALSE,
					  p_cctxt->n_sched_o_tkn_per_ip_frm));

		VCD_MSG_MED("Sending non-NULL output with EOS");

		p_op_buf_entry->frame.n_data_len = 0;
		p_op_buf_entry->frame.n_flags |= VCD_FRAME_FLAG_EOS;
		p_op_buf_entry->frame.n_ip_frm_tag =
			p_input_frame->n_ip_frm_tag;
		p_op_buf_entry->frame.time_stamp = p_input_frame->time_stamp;

		p_cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE,
				  VCD_S_SUCCESS,
				  &p_op_buf_entry->frame,
				  sizeof(struct vcd_frame_data_type),
				  p_cctxt, p_cctxt->p_client_data);
	}
}

u32 vcd_handle_recvd_eos(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_frame_data_type *p_input_frame, u32 *pb_eos_handled)
{
	union sched_value_type sched_val;
	u32 rc;

	VCD_MSG_LOW("vcd_handle_recvd_eos:");

	*pb_eos_handled = FALSE;

	if (p_input_frame->p_virtual &&
			p_input_frame->n_data_len)
		return VCD_S_SUCCESS;

	p_input_frame->n_data_len = 0;

	rc = vcd_map_sched_status(sched_get_client_param
				  (p_cctxt->p_dev_ctxt->sched_hdl,
				   p_cctxt->sched_clnt_hdl,
				   SCHED_I_CLNT_CURRQLEN, &sched_val));

	VCD_FAILED_RETURN(rc, "Failed: sched_get_client_param");

	if (sched_val.un_value > 0) {
		rc = vcd_map_sched_status(sched_mark_client_eof
					  (p_cctxt->p_dev_ctxt->sched_hdl,
					   p_cctxt->sched_clnt_hdl));

		if (!VCD_FAILED(rc)) {
			*pb_eos_handled = TRUE;
		} else {
			VCD_MSG_ERROR("rc = 0x%x. Failed: "
				"sched_mark_client_eof", rc);
		}

	} else if (p_cctxt->b_decoding && !p_input_frame->p_virtual) {
		rc = vcd_map_sched_status(sched_update_client_o_tkn
					  (p_cctxt->p_dev_ctxt->sched_hdl,
					   p_cctxt->sched_clnt_hdl, TRUE,
					   p_cctxt->n_sched_o_tkn_per_ip_frm));
	} else if (!p_cctxt->b_decoding) {

		vcd_send_frame_done_in_eos(p_cctxt, p_input_frame, FALSE);

		if (p_cctxt->status.b_eos_wait_for_op_buf) {
			vcd_do_client_state_transition(p_cctxt,
				VCD_CLIENT_STATE_EOS,
				CLIENT_STATE_EVENT_NUMBER
				(pf_encode_frame));
		}

		*pb_eos_handled = TRUE;

	}

	if (*pb_eos_handled &&
		p_input_frame->p_virtual &&
		!p_input_frame->n_data_len) {
		p_cctxt->callback(VCD_EVT_RESP_INPUT_DONE,
				  VCD_S_SUCCESS,
				  p_input_frame,
				  sizeof(struct vcd_frame_data_type),
				  p_cctxt, p_cctxt->p_client_data);
	}
	return rc;
}

u32 vcd_handle_first_decode_frame(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct ddl_property_dec_pic_buffers_type dpb;
	struct vcd_property_hdr_type prop_hdr;
	u32 rc;
	u16 i;
	u16 n_q_cntr;
	struct ddl_frame_data_type_tag *p_frm_entry;
	struct ddl_frame_data_type_tag ddl_frm;
	struct vcd_buffer_pool_type *p_out_buf_pool;

	VCD_MSG_LOW("vcd_handle_first_decode_frame:");

	if (!p_cctxt->in_buf_pool.a_entries ||
		!p_cctxt->out_buf_pool.a_entries ||
		p_cctxt->in_buf_pool.n_validated !=
		p_cctxt->in_buf_pool.n_count ||
		p_cctxt->out_buf_pool.n_validated !=
		p_cctxt->out_buf_pool.n_count) {
		VCD_MSG_ERROR("Buffer pool is not completely setup yet");

		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_add_client_to_sched(p_cctxt);

	VCD_FAILED_RETURN(rc, "Failed: vcd_add_client_to_sched");

	prop_hdr.prop_id = DDL_I_DPB;
	prop_hdr.n_size = sizeof(dpb);

	p_out_buf_pool = &p_cctxt->out_buf_pool;

	p_frm_entry =
		(struct ddl_frame_data_type_tag *)
		vcd_malloc(sizeof(struct ddl_frame_data_type_tag) *
						   p_out_buf_pool->n_count);
	if (!p_frm_entry) {
		VCD_MSG_ERROR("Memory allocation failure");

		return VCD_ERR_ALLOC_FAIL;
	}

	for (i = 1; i <= p_out_buf_pool->n_count; i++)
		p_frm_entry[i - 1].vcd_frm = p_out_buf_pool->a_entries[i].frame;


	dpb.a_dec_pic_buffers = p_frm_entry;
	dpb.n_no_of_dec_pic_buf = p_out_buf_pool->n_count;
	rc = ddl_set_property(p_cctxt->ddl_handle, &prop_hdr, &dpb);

	vcd_free(p_frm_entry);

	VCD_FAILED_RETURN(rc, "Failed: DDL set DDL_I_DPB");

	if (p_out_buf_pool->n_q_len > 0) {
		prop_hdr.prop_id = DDL_I_DPB_RELEASE;
		prop_hdr.n_size = sizeof(struct ddl_frame_data_type_tag);

		for (i = 0, n_q_cntr = p_out_buf_pool->n_q_head;
			 !VCD_FAILED(rc) && i < p_out_buf_pool->n_q_len;
			 i++, n_q_cntr = (n_q_cntr + 1) %
			 p_out_buf_pool->n_count) {

			ddl_frm.vcd_frm =
				p_out_buf_pool->a_queue[n_q_cntr]->frame;

			rc = ddl_set_property(p_cctxt->ddl_handle, &prop_hdr,
						  &ddl_frm);

			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR
					("Error returning output buffer to HW");

				p_out_buf_pool->a_queue[n_q_cntr]->b_in_use =
					FALSE;
			} else {
				p_out_buf_pool->a_queue[n_q_cntr]->b_in_use =
					TRUE;
				p_out_buf_pool->n_in_use++;
			}
		}

		if (!VCD_FAILED(rc)) {
			rc = vcd_map_sched_status(sched_update_client_o_tkn
						  (p_cctxt->p_dev_ctxt->
						   sched_hdl,
						   p_cctxt->sched_clnt_hdl,
						   TRUE,
						   p_cctxt->
						   n_sched_o_tkn_per_ip_frm *
						   p_out_buf_pool->n_q_len));
		}
	}
	return rc;
}

u32 vcd_setup_with_ddl_capabilities(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	struct vcd_property_hdr_type Prop_hdr;
	struct ddl_property_capability_type capability;
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_LOW("vcd_setup_with_ddl_capabilities:");

	if (!p_dev_ctxt->n_ddl_cmd_ch_depth) {
		Prop_hdr.prop_id = DDL_I_CAPABILITY;
		Prop_hdr.n_size = sizeof(capability);

		/*
		** Since this is underlying core's property we don't need a
		** ddl client handle.
		*/
		rc = ddl_get_property(NULL, &Prop_hdr, &capability);

		if (!VCD_FAILED(rc)) {
			/*
			** Allocate the transaction table.
			*/
			p_dev_ctxt->n_trans_tbl_size =
				(VCD_MAX_CLIENT_TRANSACTIONS *
				capability.n_max_num_client) +
				capability.n_general_command_depth;

			p_dev_ctxt->a_trans_tbl = (struct vcd_transc_type *)
				vcd_malloc(sizeof(struct vcd_transc_type) *
				p_dev_ctxt->n_trans_tbl_size);

			if (!p_dev_ctxt->a_trans_tbl) {
				VCD_MSG_ERROR("Transaction table alloc failed");

				rc = VCD_ERR_ALLOC_FAIL;
			} else	{
				memset(p_dev_ctxt->a_trans_tbl, 0,
					sizeof(struct vcd_transc_type) *
					p_dev_ctxt->n_trans_tbl_size);

				/*
				** Set the command/frame depth
				*/
				p_dev_ctxt->b_ddl_cmd_concurrency =
					!capability.b_exclusive;
				p_dev_ctxt->n_ddl_frame_ch_depth =
					capability.n_frame_command_depth;
				p_dev_ctxt->n_ddl_cmd_ch_depth =
					capability.n_general_command_depth;

				vcd_reset_device_channels(p_dev_ctxt);

				p_dev_ctxt->n_hw_time_out =
					capability.n_ddl_time_out_in_ms;

			}
		}
	}
	return rc;
}

struct vcd_transc_type *vcd_get_free_trans_tbl_entry
	(struct vcd_dev_ctxt_type *p_dev_ctxt) {
	u8 i;

	if (!p_dev_ctxt->a_trans_tbl)
		return NULL;

	i = 0;
	while (i < p_dev_ctxt->n_trans_tbl_size &&
		   p_dev_ctxt->a_trans_tbl[i].b_in_use)
		i++;

	if (i == p_dev_ctxt->n_trans_tbl_size) {
		return NULL;
	} else {
		memset(&p_dev_ctxt->a_trans_tbl[i], 0,
			   sizeof(struct vcd_transc_type));

		p_dev_ctxt->a_trans_tbl[i].b_in_use = TRUE;

		return &p_dev_ctxt->a_trans_tbl[i];
	}
}

void vcd_release_trans_tbl_entry(struct vcd_transc_type *p_trans_entry)
{
	if (p_trans_entry)
		p_trans_entry->b_in_use = FALSE;
}

u32 vcd_add_client_to_sched(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_property_hdr_type prop_hdr;
	struct sched_client_init_param_type sched_input_init;
	u32 rc, seqhdr_present = 0;;

	if (p_cctxt->b_sched_clnt_valid) {
		VCD_MSG_HIGH("Schedulder client is already added ");
		return VCD_S_SUCCESS;
	}

	prop_hdr.prop_id = DDL_I_FRAME_PROC_UNITS;
	prop_hdr.n_size = sizeof(p_cctxt->n_frm_p_units);
	rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr,
				  &p_cctxt->n_frm_p_units);
	VCD_FAILED_RETURN(rc, "Failed: Get DDL_I_FRAME_PROC_UNITS");

	if (p_cctxt->b_decoding) {
		p_cctxt->frm_rate.n_fps_numerator = VCD_DEC_INITIAL_FRAME_RATE;
		p_cctxt->frm_rate.n_fps_denominator = 1;

		sched_input_init.n_o_tkn_per_ip_frm =
			p_cctxt->n_sched_o_tkn_per_ip_frm =
			VCD_SCHEDULER_DEC_DFLT_OTKN_PERFRM;
		sched_input_init.n_o_tkn_max =
			p_cctxt->n_sched_o_tkn_per_ip_frm *
			p_cctxt->out_buf_pool.n_count+1;
	} else {
		sched_input_init.n_o_tkn_per_ip_frm =
		p_cctxt->n_sched_o_tkn_per_ip_frm =
			VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM;
		prop_hdr.prop_id = DDL_I_SEQHDR_PRESENT;
		prop_hdr.n_size = sizeof(seqhdr_present);
		rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr,
							   &seqhdr_present);
		if (!VCD_FAILED(rc)) {
			if (seqhdr_present == 0x1) {
				VCD_MSG_MED("Sequence hdr present");
				sched_input_init.n_o_tkn_per_ip_frm++;
			}
			sched_input_init.n_o_tkn_max =
				p_cctxt->out_buf_pool.n_count;
			prop_hdr.prop_id = VCD_I_FRAME_RATE;
			prop_hdr.n_size = sizeof(p_cctxt->frm_rate);
			rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr,
						  &p_cctxt->frm_rate);
		}
	}

	VCD_FAILED_RETURN(rc, "Failed: DDL get VCD_I_FRAME_RATE");

	if (p_cctxt->b_live)
		sched_input_init.client_ctgy = SCHED_CLNT_RT_NOBUFF;
	else
		sched_input_init.client_ctgy = SCHED_CLNT_NONRT;

	sched_input_init.n_max_queue_len = MAX(p_cctxt->in_buf_pool.n_count,
						   VCD_MAX_SCHEDULER_QUEUE_SIZE
						   (p_cctxt->frm_rate.
						n_fps_numerator,
						p_cctxt->frm_rate.
						n_fps_denominator));
	p_cctxt->n_reqd_perf_lvl =
		p_cctxt->n_frm_p_units * p_cctxt->frm_rate.n_fps_numerator /
		p_cctxt->frm_rate.n_fps_denominator;

	sched_input_init.frm_rate.n_numer = p_cctxt->frm_rate.n_fps_numerator;
	sched_input_init.frm_rate.n_denom = p_cctxt->frm_rate.n_fps_denominator;
	sched_input_init.n_p_tkn_per_frm = p_cctxt->n_frm_p_units;
	sched_input_init.n_alloc_p_tkn_rate = p_cctxt->n_reqd_perf_lvl;

	sched_input_init.n_o_tkn_init = 0;

	sched_input_init.p_client_data = p_cctxt;

	rc = vcd_map_sched_status(sched_add_client
				  (p_cctxt->p_dev_ctxt->sched_hdl,
				   &sched_input_init,
				   &p_cctxt->sched_clnt_hdl));

	if (!VCD_FAILED(rc))
		p_cctxt->b_sched_clnt_valid = TRUE;

	return rc;
}

u32 vcd_handle_input_done(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 void *p_payload, u32 event, u32 status)
{
	struct vcd_transc_type *p_transc;
	struct ddl_frame_data_type_tag *p_frame =
		(struct ddl_frame_data_type_tag *) p_payload;
	u32 rc;

	if (!p_cctxt->status.n_frame_submitted &&
		!p_cctxt->status.n_frame_delayed) {
		VCD_MSG_ERROR("Input done was not expected");
		vcd_assert();

		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_validate_io_done_pyld(p_payload, status);
	VCD_FAILED_RETURN(rc, "Bad input done payload");

	p_transc = (struct vcd_transc_type *)p_frame->vcd_frm.n_ip_frm_tag;

	if ((p_transc->p_ip_buf_entry->frame.p_virtual !=
		 p_frame->vcd_frm.p_virtual)
		|| !p_transc->p_ip_buf_entry->b_in_use) {
		VCD_MSG_ERROR("Bad frm transaction state");
		vcd_assert();
	}

	p_frame->vcd_frm.n_ip_frm_tag = p_transc->n_ip_frm_tag;

	p_cctxt->callback(event,
			  status,
			  &p_frame->vcd_frm,
			  sizeof(struct vcd_frame_data_type),
			  p_cctxt, p_cctxt->p_client_data);

	p_transc->e_frame_type = p_frame->vcd_frm.e_frame_type;

	p_transc->p_ip_buf_entry->b_in_use = FALSE;
	VCD_BUFFERPOOL_INUSE_DECREMENT(p_cctxt->in_buf_pool.n_in_use);
	p_transc->p_ip_buf_entry = NULL;
	p_transc->b_input_done = TRUE;

	if (p_transc->b_input_done && p_transc->b_frame_done)
		p_transc->b_in_use = FALSE;

	if (VCD_FAILED(status)) {
		VCD_MSG_ERROR("INPUT_DONE returned err = 0x%x", status);
		vcd_handle_input_done_failed(p_cctxt, p_transc);
	}

	if (p_cctxt->status.n_frame_submitted > 0)
		p_cctxt->status.n_frame_submitted--;
	else
		p_cctxt->status.n_frame_delayed--;

	if (!VCD_FAILED(status) &&
		p_cctxt->b_decoding) {
		if (p_frame->vcd_frm.b_interlaced)
			vcd_handle_input_done_for_interlacing(p_cctxt);
		if (p_frame->b_frm_trans_end)
			vcd_handle_input_done_with_trans_end(p_cctxt);
	}

	return VCD_S_SUCCESS;
}

void vcd_handle_input_done_in_eos(
	struct vcd_clnt_ctxt_type_t *p_cctxt, void *p_payload, u32 status)
{
	struct vcd_transc_type *p_transc;
	struct ddl_frame_data_type_tag *p_frame =
		(struct ddl_frame_data_type_tag *) p_payload;

	if (VCD_FAILED(vcd_validate_io_done_pyld(p_payload, status)))
		return;

	p_transc = (struct vcd_transc_type *)p_frame->vcd_frm.n_ip_frm_tag;

	(void)vcd_handle_input_done(p_cctxt,
		p_payload, VCD_EVT_RESP_INPUT_DONE, status);

	if ((p_frame->vcd_frm.n_flags & VCD_FRAME_FLAG_EOS)) {
		VCD_MSG_HIGH("Got input done for EOS initiator");
		p_transc->b_input_done = FALSE;
		p_transc->b_in_use = TRUE;
	}
}

u32 vcd_validate_io_done_pyld(void *p_payload, u32 status)
{
	struct ddl_frame_data_type_tag *p_frame =
		(struct ddl_frame_data_type_tag *) p_payload;

	if (!p_frame) {
		VCD_MSG_ERROR("Bad payload from DDL");
		vcd_assert();

		return VCD_ERR_BAD_POINTER;
	}

	if (!p_frame->vcd_frm.n_ip_frm_tag ||
		p_frame->vcd_frm.n_ip_frm_tag == VCD_FRAMETAG_INVALID) {
		VCD_MSG_ERROR("bad input frame tag");
		vcd_assert();
		return VCD_ERR_BAD_POINTER;
	}

	if (!p_frame->vcd_frm.p_virtual &&
		status != VCD_ERR_INTRLCD_FIELD_DROP)
		return VCD_ERR_BAD_POINTER;

	return VCD_S_SUCCESS;
}

void vcd_handle_input_done_failed(
	struct vcd_clnt_ctxt_type_t *p_cctxt, struct vcd_transc_type *p_transc)
{
	if (p_cctxt->b_decoding) {
		(void)vcd_map_sched_status(sched_update_client_o_tkn
				 (p_cctxt->p_dev_ctxt->sched_hdl,
				  p_cctxt->sched_clnt_hdl, TRUE,
				  p_cctxt->n_sched_o_tkn_per_ip_frm));

		p_transc->b_in_use = FALSE;
	}
}

void vcd_handle_input_done_for_interlacing(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc;

	p_cctxt->status.n_int_field_cnt++;

	if (p_cctxt->status.n_int_field_cnt == 1) {
		rc = vcd_map_sched_status(sched_update_client_o_tkn
					  (p_cctxt->p_dev_ctxt->sched_hdl,
					   p_cctxt->sched_clnt_hdl, TRUE,
					   p_cctxt->n_sched_o_tkn_per_ip_frm));

		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("sched_update_client_o_tkn failed");
	} else if (p_cctxt->status.n_int_field_cnt ==
		VCD_DEC_NUM_INTERLACED_FIELDS)
		p_cctxt->status.n_int_field_cnt = 0;
}

void vcd_handle_input_done_with_trans_end(
	struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc;
	union sched_value_type sched_val;
	if (!p_cctxt->b_decoding)
		return;

	if (p_cctxt->out_buf_pool.n_in_use <
		p_cctxt->out_buf_pool.buf_req.n_min_count)
		return;

	rc = vcd_map_sched_status(sched_get_client_param(
		p_cctxt->p_dev_ctxt->sched_hdl, p_cctxt->sched_clnt_hdl,
		SCHED_I_CLNT_OTKNCURRENT, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("sched_get_client_param:OTKNCURRENT failed");
		return;
	}

	if (!sched_val.un_value) {
		VCD_MSG_MED("All output buffers with core are pending display");

		rc = vcd_map_sched_status(sched_update_client_o_tkn(
			p_cctxt->p_dev_ctxt->sched_hdl,
			p_cctxt->sched_clnt_hdl,
			TRUE,
			p_cctxt->n_sched_o_tkn_per_ip_frm));

		if (VCD_FAILED(rc))
			VCD_MSG_ERROR("sched_update_client_o_tkn failed");
	}
}

u32 vcd_handle_output_required(struct vcd_clnt_ctxt_type_t
	*p_cctxt, void *p_payload, u32 status)
{
	struct vcd_transc_type *p_transc;
	struct ddl_frame_data_type_tag *p_frame =
		(struct ddl_frame_data_type_tag *)p_payload;
	u32 rc;

	if (!p_cctxt->status.n_frame_submitted &&
		!p_cctxt->status.n_frame_delayed) {
		VCD_MSG_ERROR("\n Input done was not expected");
		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_validate_io_done_pyld(p_payload, status);
	VCD_FAILED_RETURN(rc, "\n Bad input done payload");

	p_transc = (struct vcd_transc_type *)p_frame->
		vcd_frm.n_ip_frm_tag;

	if ((p_transc->p_ip_buf_entry->frame.p_virtual !=
		 p_frame->vcd_frm.p_virtual) ||
		!p_transc->p_ip_buf_entry->b_in_use) {
		VCD_MSG_ERROR("\n Bad frm transaction state");
		return VCD_ERR_BAD_STATE;
	}

	rc = vcd_map_sched_status(sched_re_queue_frame(
			p_cctxt->p_dev_ctxt->sched_hdl,
			p_cctxt->sched_clnt_hdl,
			(void *) p_transc->p_ip_buf_entry));

	VCD_FAILED_RETURN(rc, "Failed: sched_queue_frame");

	if (p_transc->p_ip_buf_entry->frame.n_flags &
		VCD_FRAME_FLAG_EOS) {
		rc = vcd_map_sched_status(sched_mark_client_eof(
			p_cctxt->p_dev_ctxt->sched_hdl,
			p_cctxt->sched_clnt_hdl));
	}

	VCD_FAILED_RETURN(rc, "Failed: sched_mark_client_eof");

	p_transc->p_ip_buf_entry = NULL;
	p_transc->b_in_use = FALSE;
	p_frame->b_frm_trans_end = TRUE;

	if (VCD_FAILED(status))
		VCD_MSG_ERROR("\n OUTPUT_REQ returned err = 0x%x",
			status);

	if (p_cctxt->status.n_frame_submitted > 0)
		p_cctxt->status.n_frame_submitted--;
	else
		p_cctxt->status.n_frame_delayed--;

	if (!VCD_FAILED(status) &&
		p_cctxt->b_decoding &&
		p_frame->vcd_frm.b_interlaced) {
		if (p_cctxt->status.n_int_field_cnt > 0)
			VCD_MSG_ERROR("\n Not expected: OUTPUT_REQ"
				"for 2nd interlace field");
	}

	return VCD_S_SUCCESS;
}


u32 vcd_handle_output_required_in_flushing(
struct vcd_clnt_ctxt_type_t *p_cctxt, void *p_payload)
{
	u32 rc;
	struct vcd_transc_type *p_transc;

	rc = vcd_validate_io_done_pyld(p_payload, VCD_S_SUCCESS);
	VCD_FAILED_RETURN(rc, "Bad input done payload");

	p_transc = (struct vcd_transc_type *)
		(((struct ddl_frame_data_type_tag *)p_payload)->
		 vcd_frm.n_ip_frm_tag);

	((struct ddl_frame_data_type_tag *)p_payload)->
		vcd_frm.b_interlaced = FALSE;

	rc = vcd_handle_input_done(p_cctxt, p_payload,
			VCD_EVT_RESP_INPUT_FLUSHED, VCD_S_SUCCESS);

	p_transc->b_in_use = FALSE;
	((struct ddl_frame_data_type_tag *)p_payload)->b_frm_trans_end = TRUE;

	return rc;
}

u32 vcd_handle_frame_done(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 void *p_payload, u32 event, u32 status)
{
	struct vcd_buffer_entry_type *p_op_buf_entry;
	struct ddl_frame_data_type_tag *p_op_frm =
		(struct ddl_frame_data_type_tag *) p_payload;
	struct vcd_transc_type *p_transc;
	u32 rc;

	rc = vcd_validate_io_done_pyld(p_payload, status);
	VCD_FAILED_RETURN(rc, "Bad payload recvd");

	p_transc = (struct vcd_transc_type *)p_op_frm->vcd_frm.n_ip_frm_tag;

	if (p_op_frm->vcd_frm.p_virtual) {

		if (!p_transc->p_op_buf_entry) {
			p_op_buf_entry =
				vcd_find_buffer_pool_entry(
					&p_cctxt->out_buf_pool,
					p_op_frm->vcd_frm.
					p_virtual);
		} else {
			p_op_buf_entry = p_transc->p_op_buf_entry;
		}

		if (!p_op_buf_entry) {
			VCD_MSG_ERROR("Invalid output buffer returned"
				"from DDL");
			vcd_assert();
			rc = VCD_ERR_BAD_POINTER;
		} else if (!p_op_buf_entry->b_in_use) {
			VCD_MSG_ERROR("Bad output buffer 0x%p recvd from DDL",
					  p_op_buf_entry->frame.p_virtual);
			vcd_assert();
			rc = VCD_ERR_BAD_POINTER;
		} else {
			p_op_buf_entry->b_in_use = FALSE;
			VCD_BUFFERPOOL_INUSE_DECREMENT(
				p_cctxt->out_buf_pool.n_in_use);
			VCD_MSG_LOW("outBufPool.InUse = %d",
						p_cctxt->out_buf_pool.n_in_use);
		}
	}
	VCD_FAILED_RETURN(rc, "Bad output buffer pointer");
	p_op_frm->vcd_frm.time_stamp = p_transc->time_stamp;
	p_op_frm->vcd_frm.n_ip_frm_tag = p_transc->n_ip_frm_tag;
	p_op_frm->vcd_frm.e_frame_type = p_transc->e_frame_type;

	p_transc->b_frame_done = TRUE;

	if (p_transc->b_input_done && p_transc->b_frame_done)
		p_transc->b_in_use = FALSE;

	if (status == VCD_ERR_INTRLCD_FIELD_DROP ||
		(p_op_frm->n_intrlcd_ip_frm_tag != VCD_FRAMETAG_INVALID &&
		 p_op_frm->n_intrlcd_ip_frm_tag)) {
		vcd_handle_frame_done_for_interlacing(p_cctxt, p_transc,
							  p_op_frm, status);
	}

	if (status != VCD_ERR_INTRLCD_FIELD_DROP) {
		p_cctxt->callback(event,
				  status,
				  &p_op_frm->vcd_frm,
				  sizeof(struct vcd_frame_data_type),
				  p_cctxt, p_cctxt->p_client_data);
	}
	return VCD_S_SUCCESS;
}

void vcd_handle_frame_done_in_eos(
	struct vcd_clnt_ctxt_type_t *p_cctxt, void *p_payload, u32 status)
{
	struct ddl_frame_data_type_tag *p_frame =
		(struct ddl_frame_data_type_tag *) p_payload;

	VCD_MSG_LOW("vcd_handle_frame_done_in_eos:");

	if (VCD_FAILED(vcd_validate_io_done_pyld(p_payload, status)))
		return;

	if (p_cctxt->status.b_eos_prev_valid) {
		(void)vcd_handle_frame_done(p_cctxt,
			(void *)&p_cctxt->status.
			eos_prev_op_frm,
			VCD_EVT_RESP_OUTPUT_DONE,
			status);
	}

	p_cctxt->status.eos_prev_op_frm = *p_frame;
	p_cctxt->status.b_eos_prev_valid = TRUE;
}

void vcd_handle_frame_done_for_interlacing(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_transc_type *p_transc_ip1,
	 struct ddl_frame_data_type_tag *p_op_frm, u32 status)
{
	struct vcd_transc_type *p_transc_ip2 =
		(struct vcd_transc_type *)p_op_frm->n_intrlcd_ip_frm_tag;

	if (status == VCD_ERR_INTRLCD_FIELD_DROP) {
		p_cctxt->status.n_int_field_cnt = 0;
		return;
	}

	p_op_frm->n_intrlcd_ip_frm_tag = p_transc_ip2->n_ip_frm_tag;

	p_transc_ip2->b_frame_done = TRUE;

	if (p_transc_ip2->b_input_done && p_transc_ip2->b_frame_done)
		p_transc_ip2->b_in_use = FALSE;

	if (!p_transc_ip1->e_frame_type ||
			!p_transc_ip2->e_frame_type) {
		VCD_MSG_ERROR("DDL didn't provided frame type");

		return;
	}
}

u32 vcd_handle_first_frame_done(
	struct vcd_clnt_ctxt_type_t *p_cctxt, void *p_payload)
{
	if (!p_cctxt->b_decoding)
		return vcd_handle_first_encode_frame_done(p_cctxt, p_payload);

	return VCD_S_SUCCESS;
}

u32 vcd_handle_first_encode_frame_done(
	struct vcd_clnt_ctxt_type_t *p_cctxt, void *p_payload)
{
	struct vcd_buffer_entry_type *p_buf_entry;
	struct vcd_frame_data_type *p_frm_entry;
	u32 rc, seqhdr_present;
	struct vcd_property_hdr_type prop_hdr;
	struct vcd_sequence_hdr_type seq_hdr;
	struct vcd_property_codec_type codec;
	union sched_value_type sched_val;
	struct vcd_transc_type *p_transc;
	struct ddl_frame_data_type_tag *p_payload_frm =
		(struct ddl_frame_data_type_tag *) p_payload;
	VCD_MSG_LOW("vcd_handle_first_encode_frame_done:");

	rc = vcd_validate_io_done_pyld(p_payload, VCD_S_SUCCESS);
	VCD_FAILED_RETURN(rc, "Validate frame done payload failed");

	p_transc = (struct vcd_transc_type *)p_payload_frm->
		vcd_frm.n_ip_frm_tag;

	prop_hdr.prop_id = DDL_I_SEQHDR_PRESENT;
	prop_hdr.n_size = sizeof(seqhdr_present);
	rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr,
		&seqhdr_present);
	VCD_FAILED_RETURN(rc, "Failed: DDL_I_SEQHDR_PRESENT");
	if (!seqhdr_present)
		return VCD_S_SUCCESS;

	p_buf_entry = vcd_buffer_pool_entry_de_q(&p_cctxt->out_buf_pool);

	if (!p_buf_entry) {
		VCD_MSG_ERROR("Sched provided frame when 2nd op buffer "
			"was unavailable");

		rc = VCD_ERR_FAIL;
		vcd_assert();
	} else {
		p_frm_entry = &p_buf_entry->frame;
		prop_hdr.prop_id = VCD_I_CODEC;
		prop_hdr.n_size = sizeof(struct vcd_property_codec_type);

		rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr, &codec);

		if (!VCD_FAILED(rc)) {
			if (codec.e_codec != VCD_CODEC_H263) {
				prop_hdr.prop_id = VCD_I_SEQ_HEADER;
				prop_hdr.n_size = sizeof(struct
					vcd_sequence_hdr_type);

				seq_hdr.p_sequence_header =
					p_frm_entry->p_virtual;
				seq_hdr.n_sequence_header_len =
					p_buf_entry->n_size;

				rc = ddl_get_property(p_cctxt->ddl_handle,
					&prop_hdr, &seq_hdr);
			} else {
				VCD_MSG_LOW("Codec Type is H.263\n");
			}

			if (!VCD_FAILED(rc)) {
				sched_val.un_value =
					VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM;

				rc = vcd_map_sched_status(sched_set_client_param
					(p_cctxt->p_dev_ctxt->
					sched_hdl,
					p_cctxt->sched_clnt_hdl,
					SCHED_I_CLNT_OTKNPERIPFRM,
					&sched_val));

				if (!VCD_FAILED(rc)) {
					p_frm_entry->n_data_len =
						seq_hdr.n_sequence_header_len;
					p_frm_entry->time_stamp =
						p_transc->time_stamp;
					p_frm_entry->n_ip_frm_tag =
						p_transc->n_ip_frm_tag;
					p_frm_entry->n_flags |=
						VCD_FRAME_FLAG_CODECCONFIG;

				p_cctxt->callback(VCD_EVT_RESP_OUTPUT_DONE,
					VCD_S_SUCCESS, p_frm_entry,
					sizeof(struct vcd_frame_data_type),
					p_cctxt,
					p_cctxt->p_client_data);

				} else {
					VCD_MSG_ERROR("rc = 0x%x.Failed: "
						"sched_set_client_param", rc);
				}

			} else {
				VCD_MSG_ERROR
					("rc = 0x%x. Failed: "
					 "ddl_get_property:VCD_I_SEQ_HEADER",
					 rc);
			}
		} else {
			VCD_MSG_ERROR
				("rc = 0x%x. Failed: "
				 "ddl_get_property:VCD_I_CODEC",
				 rc);
		}

		if (VCD_FAILED(rc)) {
			(void)vcd_buffer_pool_entry_en_q(&p_cctxt->out_buf_pool,
							 p_buf_entry);
		}
	}

	return rc;
}

void vcd_handle_eos_trans_end(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	if (p_cctxt->status.b_eos_prev_valid) {
		(void) vcd_handle_frame_done(p_cctxt,
			(void *)&p_cctxt->status.eos_prev_op_frm,
			VCD_EVT_RESP_OUTPUT_DONE,
			VCD_S_SUCCESS);

		p_cctxt->status.b_eos_prev_valid = FALSE;
	}

	if (p_cctxt->status.n_flush_mode)
		vcd_process_pending_flush_in_eos(p_cctxt);

	if (p_cctxt->status.b_stop_pending)
		vcd_process_pending_stop_in_eos(p_cctxt);
	else {
		vcd_do_client_state_transition(p_cctxt,
			VCD_CLIENT_STATE_RUN,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}
}

void vcd_handle_eos_done(struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_transc_type *p_transc, u32 status)
{
	struct vcd_frame_data_type  vcd_frm;
	VCD_MSG_LOW("vcd_handle_eos_done:");

	if (VCD_FAILED(status))
		VCD_MSG_ERROR("EOS DONE returned error = 0x%x", status);

	if (p_cctxt->status.b_eos_prev_valid) {
		p_cctxt->status.eos_prev_op_frm.vcd_frm.n_flags |=
			VCD_FRAME_FLAG_EOS;

		(void)vcd_handle_frame_done(p_cctxt,
						(void *)&p_cctxt->status.
						eos_prev_op_frm,
						VCD_EVT_RESP_OUTPUT_DONE,
						VCD_S_SUCCESS);

		p_cctxt->status.b_eos_prev_valid = FALSE;
	} else {
		if (p_transc->p_ip_buf_entry) {
			p_transc->p_ip_buf_entry->frame.n_ip_frm_tag =
				p_transc->n_ip_frm_tag;

			vcd_send_frame_done_in_eos(p_cctxt,
				&p_transc->p_ip_buf_entry->frame, FALSE);
		} else {
			memset(&vcd_frm, 0, sizeof(struct vcd_frame_data_type));
			vcd_frm.n_ip_frm_tag = p_transc->n_ip_frm_tag;
			vcd_frm.time_stamp = p_transc->time_stamp;
			vcd_frm.n_flags = VCD_FRAME_FLAG_EOS;
			vcd_send_frame_done_in_eos(p_cctxt, &vcd_frm, TRUE);
		}
	}
	if (p_transc->p_ip_buf_entry) {
		if (p_transc->p_ip_buf_entry->frame.p_virtual) {
			p_transc->p_ip_buf_entry->frame.n_ip_frm_tag =
				p_transc->n_ip_frm_tag;

			p_cctxt->callback(VCD_EVT_RESP_INPUT_DONE,
					  VCD_S_SUCCESS,
					  &p_transc->p_ip_buf_entry->frame,
					  sizeof(struct vcd_frame_data_type),
					  p_cctxt, p_cctxt->p_client_data);
		}
		p_transc->p_ip_buf_entry->b_in_use = FALSE;
		VCD_BUFFERPOOL_INUSE_DECREMENT(p_cctxt->in_buf_pool.n_in_use);
		p_transc->p_ip_buf_entry = NULL;
		p_cctxt->status.n_frame_submitted--;
	}

	p_transc->b_in_use = FALSE;
	vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);
	if (p_cctxt->status.n_flush_mode)
		vcd_process_pending_flush_in_eos(p_cctxt);

	if (p_cctxt->status.b_stop_pending) {
		vcd_process_pending_stop_in_eos(p_cctxt);
	} else if (!p_cctxt->status.b_eos_wait_for_op_buf) {
		vcd_do_client_state_transition(p_cctxt,
						   VCD_CLIENT_STATE_RUN,
						   CLIENT_STATE_EVENT_NUMBER
						   (pf_clnt_cb));
	}
}

void vcd_handle_start_done(struct vcd_clnt_ctxt_type_t *p_cctxt,
	struct vcd_transc_type *p_transc, u32 status)
{
	p_cctxt->status.n_cmd_submitted--;
	vcd_mark_command_channel(p_cctxt->p_dev_ctxt, p_transc);

	if (!VCD_FAILED(status)) {
		p_cctxt->callback(VCD_EVT_RESP_START, status, NULL,
			0, p_cctxt,	p_cctxt->p_client_data);

		vcd_do_client_state_transition(p_cctxt,
			VCD_CLIENT_STATE_RUN,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	} else {
		VCD_MSG_ERROR("ddl callback returned failure."
			"status = 0x%x", status);
		vcd_handle_err_in_starting(p_cctxt, status);
	}
}

void vcd_handle_stop_done(struct vcd_clnt_ctxt_type_t *p_cctxt,
	struct vcd_transc_type *p_transc, u32 status)
{
	u32 rc = VCD_S_SUCCESS, seq_hdrpresent = 0;
	union sched_value_type sched_val;
	struct vcd_property_hdr_type  prop_hdr;
	VCD_MSG_LOW("vcd_handle_stop_done:");
	p_cctxt->status.n_cmd_submitted--;
	vcd_mark_command_channel(p_cctxt->p_dev_ctxt, p_transc);

	if (!VCD_FAILED(status)) {
		if (!p_cctxt->b_decoding) {
			prop_hdr.prop_id = DDL_I_SEQHDR_PRESENT;
			prop_hdr.n_size = sizeof(seq_hdrpresent);
			rc = ddl_get_property(p_cctxt->ddl_handle,
				&prop_hdr, &seq_hdrpresent);

		if (!VCD_FAILED(rc)) {
			if (seq_hdrpresent == 0x1) {
				sched_val.un_value =
					VCD_SCHEDULER_ENC_DFLT_OTKN_PERFRM
					+ 1;

				rc = vcd_map_sched_status(
						sched_set_client_param(
						p_cctxt->p_dev_ctxt->sched_hdl,
						p_cctxt->sched_clnt_hdl,
						SCHED_I_CLNT_OTKNPERIPFRM,
						&sched_val)
					 );

				if (VCD_FAILED(rc))
					VCD_MSG_ERROR("Failed: "
						"sched_set_client_param"
						" %d", rc);
			}

		} else {
			VCD_MSG_ERROR("Failed: DDL Get DDL_I_SEQHDR_"
				"PRESENT %d", rc);
		}

		}
		vcd_do_client_state_transition(p_cctxt,
			VCD_CLIENT_STATE_OPEN,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	} else {
		VCD_MSG_FATAL("STOP_DONE returned error = 0x%x", status);
		status = VCD_ERR_HW_FATAL;
		vcd_handle_device_err_fatal(p_cctxt->p_dev_ctxt, p_cctxt);
		vcd_do_client_state_transition(p_cctxt,
			VCD_CLIENT_STATE_INVALID,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}

	p_cctxt->callback(VCD_EVT_RESP_STOP, status, NULL, 0, p_cctxt,
					  p_cctxt->p_client_data);

	memset(&p_cctxt->status, 0, sizeof(struct vcd_clnt_status_type));
}

void vcd_handle_stop_done_in_starting(struct vcd_clnt_ctxt_type_t
	*p_cctxt, struct vcd_transc_type *p_transc, u32 status)
{
	VCD_MSG_LOW("vcd_handle_stop_done_in_starting:");
	p_cctxt->status.n_cmd_submitted--;
	vcd_mark_command_channel(p_cctxt->p_dev_ctxt, p_transc);
	if (!VCD_FAILED(status)) {
		p_cctxt->callback(VCD_EVT_RESP_START, p_cctxt->status.
			e_last_err, NULL, 0, p_cctxt, p_cctxt->p_client_data);
		vcd_do_client_state_transition(p_cctxt, VCD_CLIENT_STATE_OPEN,
			   CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	} else {
		VCD_MSG_FATAL("VCD Cleanup: STOP_DONE returned error "
			"= 0x%x", status);
		vcd_handle_err_fatal(p_cctxt, VCD_EVT_RESP_START,
			VCD_ERR_HW_FATAL);
	}
}

void vcd_handle_stop_done_in_invalid(struct vcd_clnt_ctxt_type_t
	*p_cctxt, u32 status)
{
	u32 rc;
	VCD_MSG_LOW("vcd_handle_stop_done_in_invalid:");
	if (!VCD_FAILED(status)) {
		vcd_client_cmd_flush_and_en_q(p_cctxt, VCD_CMD_CLIENT_CLOSE);
		if (p_cctxt->status.n_frame_submitted) {
			vcd_release_multiple_frame_channels(p_cctxt->p_dev_ctxt,
			p_cctxt->status.n_frame_submitted);

			p_cctxt->status.n_frame_submitted = 0;
			p_cctxt->status.n_frame_delayed = 0;
		}
		if (p_cctxt->status.n_cmd_submitted) {
			vcd_release_multiple_command_channels(
				p_cctxt->p_dev_ctxt,
				p_cctxt->status.n_cmd_submitted);
			p_cctxt->status.n_cmd_submitted = 0;
		}
	} else {
		VCD_MSG_FATAL("VCD Cleanup: STOP_DONE returned error "
			"= 0x%x", status);
		vcd_handle_device_err_fatal(p_cctxt->p_dev_ctxt, p_cctxt);
		p_cctxt->status.b_cleaning_up = FALSE;
	}
	vcd_flush_buffers_in_err_fatal(p_cctxt);
	VCD_MSG_HIGH("VCD cleanup: All buffers are returned");
	if (p_cctxt->status.b_stop_pending) {
		p_cctxt->callback(VCD_EVT_RESP_STOP, VCD_S_SUCCESS, NULL, 0,
			p_cctxt, p_cctxt->p_client_data);
		p_cctxt->status.b_stop_pending = FALSE;
	}
	rc = vcd_power_event(p_cctxt->p_dev_ctxt, p_cctxt,
						  VCD_EVT_PWR_CLNT_ERRFATAL);
	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("VCD_EVT_PWR_CLNT_ERRFATAL failed");
	if (!p_cctxt->status.b_cleaning_up &&
		p_cctxt->status.b_close_pending) {
		vcd_destroy_client_context(p_cctxt);
		vcd_handle_for_last_clnt_close(p_cctxt->p_dev_ctxt, FALSE);
	}
}

u32 vcd_handle_input_frame(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_frame_data_type *p_input_frame)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	struct vcd_buffer_entry_type *p_buf_entry;
	struct vcd_frame_data_type *p_frm_entry;
	u32 rc = VCD_S_SUCCESS;
	u32 b_eos_handled = FALSE;

	VCD_MSG_LOW("vcd_handle_input_frame:");

	VCD_MSG_LOW("input buffer: addr=(0x%p), size=(%d), len=(%d)",
			p_input_frame->p_virtual, p_input_frame->n_alloc_len,
			p_input_frame->n_data_len);

	if ((!p_input_frame->p_virtual || !p_input_frame->n_data_len)
		&& !(p_input_frame->n_flags & VCD_FRAME_FLAG_EOS)) {
		VCD_MSG_ERROR("Bad frame ptr/len/EOS combination");

		return VCD_ERR_ILLEGAL_PARM;
	}

	if (!p_cctxt->status.b1st_frame_recvd) {
		if (p_cctxt->b_decoding)
			rc = vcd_handle_first_decode_frame(p_cctxt);

		if (!VCD_FAILED(rc)) {
			p_cctxt->status.first_ts = p_input_frame->time_stamp;
			p_cctxt->status.prev_ts = p_cctxt->status.first_ts;

			p_cctxt->status.b1st_frame_recvd = TRUE;

			(void)vcd_power_event(p_cctxt->p_dev_ctxt,
						  p_cctxt,
						  VCD_EVT_PWR_CLNT_FIRST_FRAME);
		}
	}
	VCD_FAILED_RETURN(rc, "Failed: Frist frame handling");

	p_buf_entry = vcd_find_buffer_pool_entry(&p_cctxt->in_buf_pool,
						 p_input_frame->p_virtual);
	if (!p_buf_entry) {
		VCD_MSG_ERROR("Bad buffer addr: %p", p_input_frame->p_virtual);
		return VCD_ERR_FAIL;
	}

	if (p_buf_entry->b_in_use) {
		VCD_MSG_ERROR("An inuse input frame is being"
			"re-queued to scheduler");
		return VCD_ERR_FAIL;
	}

	if (p_input_frame->n_alloc_len > p_buf_entry->n_size) {
		VCD_MSG_ERROR("Bad buffer Alloc_len %d, Actual size=%d",
			p_input_frame->n_alloc_len, p_buf_entry->n_size);

		return VCD_ERR_ILLEGAL_PARM;
	}

	p_frm_entry = &p_buf_entry->frame;

	*p_frm_entry = *p_input_frame;
	p_frm_entry->p_physical = p_buf_entry->p_physical;

	if (p_input_frame->n_flags & VCD_FRAME_FLAG_EOS) {
		rc = vcd_handle_recvd_eos(p_cctxt, p_input_frame,
					  &b_eos_handled);
	}

	if (VCD_FAILED(rc) || b_eos_handled) {
		VCD_MSG_HIGH("rc = 0x%x, b_eos_handled = %d", rc,
				 b_eos_handled);

		return rc;
	}

	rc = vcd_map_sched_status(sched_queue_frame(p_dev_ctxt->sched_hdl,
							p_cctxt->sched_clnt_hdl,
							(void *)p_buf_entry));

	VCD_FAILED_RETURN(rc, "Failed: sched_queue_frame");

	p_buf_entry->b_in_use = TRUE;
	p_cctxt->in_buf_pool.n_in_use++;
	if (p_input_frame->n_flags & VCD_FRAME_FLAG_EOS) {
		rc = vcd_map_sched_status(sched_mark_client_eof
					  (p_cctxt->p_dev_ctxt->sched_hdl,
					   p_cctxt->sched_clnt_hdl));
	}

	VCD_FAILED_RETURN(rc, "Failed: sched_mark_client_eof");

	vcd_try_submit_frame(p_dev_ctxt);
	return rc;
}

void vcd_release_all_clnt_frm_transc(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u8 i;

	VCD_MSG_LOW("vcd_release_all_clnt_frm_transc:");

	for (i = 0; i < p_dev_ctxt->n_trans_tbl_size; i++) {
		if (p_dev_ctxt->a_trans_tbl[i].b_in_use &&
			p_cctxt == p_dev_ctxt->a_trans_tbl[i].p_cctxt &&
			p_dev_ctxt->a_trans_tbl[i].
			e_type == VCD_CMD_CODE_FRAME) {
			vcd_release_trans_tbl_entry(&p_dev_ctxt->
							a_trans_tbl[i]);
		}
	}
}

void vcd_release_all_clnt_def_frm_transc(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
		struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
		u8 i;

		VCD_MSG_LOW("vcd_release_all_clnt_def_frm_transc:");

		for (i = 0; i < p_dev_ctxt->n_trans_tbl_size; i++) {
			if (p_dev_ctxt->a_trans_tbl[i].b_in_use &&
				p_cctxt == p_dev_ctxt->a_trans_tbl[i].p_cctxt
				&& p_dev_ctxt->a_trans_tbl[i].e_type ==
				VCD_CMD_NONE) {
					vcd_release_trans_tbl_entry(
						&p_dev_ctxt->a_trans_tbl[i]);
			}
		}
}

void vcd_release_all_clnt_transc(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
		struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
		u8 i;

		VCD_MSG_LOW("vcd_release_all_clnt_def_frm_transc:");

		for (i = 0; i < p_dev_ctxt->n_trans_tbl_size; i++) {
			if (p_dev_ctxt->a_trans_tbl[i].b_in_use &&
				p_cctxt == p_dev_ctxt->a_trans_tbl[i].p_cctxt) {
					vcd_release_trans_tbl_entry(
						&p_dev_ctxt->a_trans_tbl[i]);
			}
		}
}

void vcd_send_flush_done(struct vcd_clnt_ctxt_type_t *p_cctxt, u32 status)
{
	VCD_MSG_LOW("vcd_send_flush_done:");

	if (p_cctxt->status.n_flush_mode & VCD_FLUSH_INPUT) {
		p_cctxt->callback(VCD_EVT_RESP_FLUSH_INPUT_DONE,
			status, NULL, 0, p_cctxt, p_cctxt->p_client_data);
		p_cctxt->status.n_flush_mode &= ~VCD_FLUSH_INPUT;
	}

	if (p_cctxt->status.n_flush_mode & VCD_FLUSH_OUTPUT) {
		p_cctxt->callback(VCD_EVT_RESP_FLUSH_OUTPUT_DONE,
			status, NULL, 0, p_cctxt, p_cctxt->p_client_data);
		p_cctxt->status.n_flush_mode &= ~VCD_FLUSH_OUTPUT;
	}
}

u32 vcd_store_seq_hdr(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_sequence_hdr_type *p_seq_hdr)
{
	u32 rc;
	struct vcd_property_hdr_type prop_hdr;
	u32 n_align;
	u8 *p_virtual_aligned;
	u32 n_addr;
	int ret = 0;

	if (!p_seq_hdr->n_sequence_header_len
		|| !p_seq_hdr->p_sequence_header) {
		VCD_MSG_ERROR("Bad seq hdr");

		return VCD_ERR_BAD_POINTER;
	}

	if (p_cctxt->seq_hdr.p_sequence_header) {
		VCD_MSG_HIGH("Old seq hdr detected");

		vcd_pmem_free(p_cctxt->seq_hdr.p_sequence_header,
				  p_cctxt->p_seq_hdr_phy_addr);
		p_cctxt->seq_hdr.p_sequence_header = NULL;
	}

	p_cctxt->seq_hdr.n_sequence_header_len =
		p_seq_hdr->n_sequence_header_len;

	prop_hdr.prop_id = DDL_I_SEQHDR_ALIGN_BYTES;
	prop_hdr.n_size = sizeof(u32);

	rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr, &n_align);

	VCD_FAILED_RETURN(rc,
			  "Failed: ddl_get_property DDL_I_SEQHDR_ALIGN_BYTES");

	VCD_MSG_MED("Seq hdr alignment bytes = %d", n_align);

	ret = vcd_pmem_alloc(p_cctxt->seq_hdr.n_sequence_header_len + n_align +
				 VCD_SEQ_HDR_PADDING_BYTES,
				 &(p_cctxt->seq_hdr.p_sequence_header),
				 &(p_cctxt->p_seq_hdr_phy_addr));

	if (ret < 0) {
		VCD_MSG_ERROR("Seq hdr allocation failed");

		return VCD_ERR_ALLOC_FAIL;
	}

	if (!p_cctxt->p_seq_hdr_phy_addr) {
		VCD_MSG_ERROR("Couldn't get physical address");

		return VCD_ERR_BAD_POINTER;
	}

	if (n_align > 0) {
		n_addr = (u32) p_cctxt->p_seq_hdr_phy_addr;
		n_addr += n_align;
		n_addr -= (n_addr % n_align);
		p_virtual_aligned = p_cctxt->seq_hdr.p_sequence_header;
		p_virtual_aligned += (u32) (n_addr -
			(u32) p_cctxt->p_seq_hdr_phy_addr);
		p_cctxt->p_seq_hdr_phy_addr = (u8 *) n_addr;
	} else {
		p_virtual_aligned = p_cctxt->seq_hdr.p_sequence_header;
	}

	memcpy(p_virtual_aligned, p_seq_hdr->p_sequence_header,
		p_seq_hdr->n_sequence_header_len);

	return VCD_S_SUCCESS;
}

u32 vcd_set_frame_rate(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_property_frame_rate_type *p_fps)
{
	union sched_value_type sched_val;
	u32 rc;

	sched_val.frm_rate.n_numer = p_fps->n_fps_numerator;
	sched_val.frm_rate.n_denom = p_fps->n_fps_denominator;
	p_cctxt->frm_rate = *p_fps;

	rc = vcd_map_sched_status(sched_set_client_param
				  (p_cctxt->p_dev_ctxt->sched_hdl,
				   p_cctxt->sched_clnt_hdl,
				   SCHED_I_CLNT_FRAMERATE, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_FRAMERATE",
				  rc);
	}

	rc = vcd_update_clnt_perf_lvl(p_cctxt, &p_cctxt->frm_rate,
					  p_cctxt->n_frm_p_units);

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_update_clnt_perf_lvl",
				  rc);
	}

	sched_val.un_value = p_cctxt->n_reqd_perf_lvl;

	rc = vcd_map_sched_status(sched_set_client_param
				  (p_cctxt->p_dev_ctxt->sched_hdl,
				   p_cctxt->sched_clnt_hdl,
				   SCHED_I_CLNT_PTKNRATE, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_PTKNRATE",
				  rc);
	}

	return VCD_S_SUCCESS;
}

u32 vcd_set_frame_size(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_property_frame_size_type *p_frm_size)
{
	struct vcd_property_hdr_type prop_hdr;
	union sched_value_type sched_val;
	u32 rc;
	u32 n_frm_p_units;
	p_frm_size = NULL;

	prop_hdr.prop_id = DDL_I_FRAME_PROC_UNITS;
	prop_hdr.n_size = sizeof(n_frm_p_units);
	rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr, &n_frm_p_units);

	VCD_FAILED_RETURN(rc, "Failed: Get DDL_I_FRAME_PROC_UNITS");

	p_cctxt->n_frm_p_units = sched_val.un_value = n_frm_p_units;

	rc = vcd_map_sched_status(sched_set_client_param
				  (p_cctxt->p_dev_ctxt->sched_hdl,
				   p_cctxt->sched_clnt_hdl,
				   SCHED_I_CLNT_PTKNPERFRM, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_PTKNPERFRM",
				  rc);
	}

	rc = vcd_update_clnt_perf_lvl(p_cctxt, &p_cctxt->frm_rate,
					  n_frm_p_units);

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_update_clnt_perf_lvl",
				  rc);
	}

	sched_val.un_value = p_cctxt->n_reqd_perf_lvl;

	rc = vcd_map_sched_status(sched_set_client_param
				  (p_cctxt->p_dev_ctxt->sched_hdl,
				   p_cctxt->sched_clnt_hdl,
				   SCHED_I_CLNT_PTKNRATE, &sched_val));

	if (VCD_FAILED(rc)) {
		VCD_MSG_ERROR("rc = 0x%x. Failed: Set SCHED_I_CLNT_PTKNRATE",
				  rc);
	}

	return VCD_S_SUCCESS;
}

void vcd_process_pending_flush_in_eos(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_HIGH("Buffer flush is pending");

	rc = vcd_flush_buffers(p_cctxt, p_cctxt->status.n_flush_mode);

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_flush_buffers", rc);

	p_cctxt->status.b_eos_wait_for_op_buf = FALSE;

	vcd_send_flush_done(p_cctxt, VCD_S_SUCCESS);
}

void vcd_process_pending_stop_in_eos(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	rc = vcd_flush_buffers(p_cctxt, VCD_FLUSH_ALL);

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("rc = 0x%x. Failed: vcd_flush_buffers", rc);

	VCD_MSG_HIGH("All buffers are returned. Enqueuing stop cmd");

	vcd_client_cmd_flush_and_en_q(p_cctxt, VCD_CMD_CODEC_STOP);
	p_cctxt->status.b_stop_pending = FALSE;

	vcd_do_client_state_transition(p_cctxt,
					   VCD_CLIENT_STATE_STOPPING,
					   CLIENT_STATE_EVENT_NUMBER(pf_stop));
}

u32 vcd_calculate_frame_delta(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_frame_data_type *p_frame)
{
	u32 n_frm_delta;
	u64 n_temp, temp1;

	n_temp = p_frame->time_stamp - p_cctxt->status.prev_ts;

	VCD_MSG_LOW("Curr_ts=%lld  Prev_ts=%lld Diff=%llu",
			p_frame->time_stamp, p_cctxt->status.prev_ts, n_temp);

	n_temp = n_temp * p_cctxt->n_time_resoln;
	n_temp = (n_temp + (VCD_TIMESTAMP_RESOLUTION >> 1));
	temp1 = do_div(n_temp, VCD_TIMESTAMP_RESOLUTION);
	n_frm_delta = n_temp;
	VCD_MSG_LOW("temp1=%lld  n_temp=%lld", temp1, n_temp);
	p_cctxt->status.n_time_elapsed += n_frm_delta;

	n_temp = ((u64)p_cctxt->status.n_time_elapsed \
			  * VCD_TIMESTAMP_RESOLUTION);
	n_temp = (n_temp + (p_cctxt->n_time_resoln >> 1));
	temp1 = do_div(n_temp, p_cctxt->n_time_resoln);

	p_cctxt->status.prev_ts = p_cctxt->status.first_ts + n_temp;

	VCD_MSG_LOW("Time_elapsed=%u, Drift=%llu, new Prev_ts=%lld",
			p_cctxt->status.n_time_elapsed, temp1,
			p_cctxt->status.prev_ts);

	return n_frm_delta;
}

struct vcd_buffer_entry_type *vcd_check_fill_output_buffer
	(struct vcd_clnt_ctxt_type_t *p_cctxt,
	 struct vcd_frame_data_type *p_buffer) {
	struct vcd_buffer_pool_type *p_buf_pool = &p_cctxt->out_buf_pool;
	struct vcd_buffer_entry_type *p_buf_entry;

	if (!p_buf_pool->a_entries) {
		VCD_MSG_ERROR("Buffers not set or allocated yet");

		return NULL;
	}

	if (!p_buffer->p_virtual) {
		VCD_MSG_ERROR("NULL buffer address provided");
		return NULL;
	}

	p_buf_entry =
		vcd_find_buffer_pool_entry(p_buf_pool, p_buffer->p_virtual);
	if (!p_buf_entry) {
		VCD_MSG_ERROR("Unrecognized buffer address provided = %p",
				  p_buffer->p_virtual);
		return NULL;
	}

	if (p_buf_entry->b_in_use) {
		VCD_MSG_ERROR
			("An inuse output frame is being provided for reuse");
		return NULL;
	}

	if (p_buffer->n_alloc_len < p_buf_pool->buf_req.n_size ||
		p_buffer->n_alloc_len > p_buf_entry->n_size) {
		VCD_MSG_ERROR
			("Bad buffer Alloc_len = %d, Actual size = %d, "
			 " Min size = %u",
			 p_buffer->n_alloc_len, p_buf_entry->n_size,
			 p_buf_pool->buf_req.n_size);
		return NULL;
	}

	return p_buf_entry;
}

void vcd_handle_ind_hw_err_fatal(struct vcd_clnt_ctxt_type_t *p_cctxt,
	u32 event, u32 status)
{
	if (p_cctxt->status.n_frame_submitted) {
		p_cctxt->status.n_frame_submitted--;
		vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);
	}
	vcd_handle_err_fatal(p_cctxt, event, status);
}

void vcd_handle_err_fatal(struct vcd_clnt_ctxt_type_t *p_cctxt, u32 event,
						  u32 status)
{
	u32 rc;
	VCD_MSG_LOW("vcd_handle_err_fatal: event=%x, err=%x", event, status);
	if (!VCD_FAILED_FATAL(status))
		return;

	if (VCD_FAILED_DEVICE_FATAL(status)) {
		vcd_clnt_handle_device_err_fatal(p_cctxt, event);
		vcd_handle_device_err_fatal(p_cctxt->p_dev_ctxt, p_cctxt);
	} else if (VCD_FAILED_CLIENT_FATAL(status)) {
		p_cctxt->status.e_last_evt = event;

		if (p_cctxt->b_sched_clnt_valid) {
			rc = vcd_map_sched_status(sched_suspend_resume_client(
				p_cctxt->p_dev_ctxt->sched_hdl,
				p_cctxt->sched_clnt_hdl, FALSE));
			if (VCD_FAILED(rc)) {
				VCD_MSG_ERROR("Failed: sched_suspend_resume_"
					"client rc=0x%x", rc);
			}
		}
		p_cctxt->callback(event, VCD_ERR_HW_FATAL, NULL, 0, p_cctxt,
						   p_cctxt->p_client_data);
		p_cctxt->status.b_cleaning_up = TRUE;
		vcd_client_cmd_flush_and_en_q(p_cctxt, VCD_CMD_CODEC_STOP);
		vcd_do_client_state_transition(p_cctxt,
			VCD_CLIENT_STATE_INVALID,
			CLIENT_STATE_EVENT_NUMBER(pf_clnt_cb));
	}
}

void vcd_handle_err_in_starting(struct vcd_clnt_ctxt_type_t *p_cctxt,
								u32 status)
{
	VCD_MSG_LOW("\n vcd_handle_err_in_starting:");
	if (VCD_FAILED_FATAL(status)) {
		vcd_handle_err_fatal(p_cctxt, VCD_EVT_RESP_START, status);
	} else {
		p_cctxt->status.e_last_err = status;
		VCD_MSG_HIGH("\n VCD cleanup: Enqueuing stop cmd");
		vcd_client_cmd_flush_and_en_q(p_cctxt, VCD_CMD_CODEC_STOP);
	}
}

void vcd_handle_trans_pending(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	if (!p_cctxt->status.n_frame_submitted) {
		VCD_MSG_ERROR("Transaction pending response was not expected");
		vcd_assert();
		return;
	}
	p_cctxt->status.n_frame_submitted--;
	p_cctxt->status.n_frame_delayed++;
	vcd_mark_frame_channel(p_cctxt->p_dev_ctxt);
}

u32 vcd_requeue_input_frame(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_clnt_ctxt_type_t *p_cctxt,
	struct vcd_buffer_entry_type *p_buf_entry)
{
	u32 rc;
	rc = vcd_map_sched_status(sched_re_queue_frame(p_dev_ctxt->sched_hdl,
		p_cctxt->sched_clnt_hdl, (void *) p_buf_entry));

	VCD_FAILED_RETURN(rc, "Failed: Sched_ReQueueFrame");

	if (p_buf_entry->frame.n_flags & VCD_FRAME_FLAG_EOS) {
		rc = vcd_map_sched_status(sched_mark_client_eof(p_dev_ctxt->
			sched_hdl, p_cctxt->sched_clnt_hdl));
	}

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("rc = 0x%x: Failed: Sched_MarkClientEOF", rc);

	return rc;
}

void vcd_handle_submit_frame_failed(struct vcd_dev_ctxt_type
	*p_dev_ctxt, struct vcd_transc_type *p_transc)
{
	struct vcd_clnt_ctxt_type_t *p_cctxt = p_transc->p_cctxt;
	u32 rc;

	vcd_mark_frame_channel(p_dev_ctxt);
	p_transc->b_in_use = FALSE;

	vcd_handle_err_fatal(p_cctxt, VCD_EVT_IND_HWERRFATAL,
		VCD_ERR_CLIENT_FATAL);

	if (vcd_get_command_channel(p_dev_ctxt, &p_transc)) {
		p_transc->e_type = VCD_CMD_CODEC_STOP;
		p_transc->p_cctxt = p_cctxt;
		rc = vcd_submit_cmd_sess_end(p_transc);
		if (VCD_FAILED(rc))	{
			vcd_release_command_channel(p_dev_ctxt, p_transc);
			VCD_MSG_ERROR("rc = 0x%x. Failed: VCD_SubmitCmdSessEnd",
				rc);
		}
	}
}

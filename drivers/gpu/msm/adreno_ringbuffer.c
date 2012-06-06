/* Copyright (c) 2002,2007-2012, Code Aurora Forum. All rights reserved.
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
 */
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/log2.h>

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cffdump.h"

#include "adreno.h"
#include "adreno_pm4types.h"
#include "adreno_ringbuffer.h"
#include "adreno_debugfs.h"
#include "adreno_postmortem.h"

#include "a2xx_reg.h"

#define GSL_RB_NOP_SIZEDWORDS				2
/* protected mode error checking below register address 0x800
*  note: if CP_INTERRUPT packet is used then checking needs
*  to change to below register address 0x7C8
*/
#define GSL_RB_PROTECTED_MODE_CONTROL		0x200001F2

/* Firmware file names
 * Legacy names must remain but replacing macro names to
 * match current kgsl model.
 * a200 is yamato
 * a220 is leia
 */
#define A200_PFP_FW "yamato_pfp.fw"
#define A200_PM4_FW "yamato_pm4.fw"
#define A220_PFP_470_FW "leia_pfp_470.fw"
#define A220_PM4_470_FW "leia_pm4_470.fw"
#define A225_PFP_FW "a225_pfp.fw"
#define A225_PM4_FW "a225_pm4.fw"

static void adreno_ringbuffer_submit(struct adreno_ringbuffer *rb)
{
	BUG_ON(rb->wptr == 0);

	/* Let the pwrscale policy know that new commands have
	 been submitted. */
	kgsl_pwrscale_busy(rb->device);

	/*synchronize memory before informing the hardware of the
	 *new commands.
	 */
	mb();

	adreno_regwrite(rb->device, REG_CP_RB_WPTR, rb->wptr);
}

static void
adreno_ringbuffer_waitspace(struct adreno_ringbuffer *rb, unsigned int numcmds,
			  int wptr_ahead)
{
	int nopcount;
	unsigned int freecmds;
	unsigned int *cmds;
	uint cmds_gpu;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	unsigned long wait_timeout = msecs_to_jiffies(adreno_dev->wait_timeout);
	unsigned long wait_time;

	/* if wptr ahead, fill the remaining with NOPs */
	if (wptr_ahead) {
		/* -1 for header */
		nopcount = rb->sizedwords - rb->wptr - 1;

		cmds = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		cmds_gpu = rb->buffer_desc.gpuaddr + sizeof(uint)*rb->wptr;

		GSL_RB_WRITE(cmds, cmds_gpu, cp_nop_packet(nopcount));

		/* Make sure that rptr is not 0 before submitting
		 * commands at the end of ringbuffer. We do not
		 * want the rptr and wptr to become equal when
		 * the ringbuffer is not empty */
		do {
			GSL_RB_GET_READPTR(rb, &rb->rptr);
		} while (!rb->rptr);

		rb->wptr++;

		adreno_ringbuffer_submit(rb);

		rb->wptr = 0;
	}

	wait_time = jiffies + wait_timeout;
	/* wait for space in ringbuffer */
	while (1) {
		GSL_RB_GET_READPTR(rb, &rb->rptr);

		freecmds = rb->rptr - rb->wptr;

		if (freecmds == 0 || freecmds > numcmds)
			break;

		if (time_after(jiffies, wait_time)) {
			KGSL_DRV_ERR(rb->device,
			"Timed out while waiting for freespace in ringbuffer "
			"rptr: 0x%x, wptr: 0x%x\n", rb->rptr, rb->wptr);
			if (!adreno_dump_and_recover(rb->device))
				wait_time = jiffies + wait_timeout;
			else
				/* GPU is hung and we cannot recover */
				BUG();
		}
	}
}

static unsigned int find_faulting_ib1_size(struct adreno_ringbuffer *rb,
				unsigned int rptr, unsigned int ib1)
{
	unsigned int value;
	unsigned int temp_rptr = rptr * sizeof(unsigned int);
	unsigned int rb_size = rb->buffer_desc.size;

	do {
		temp_rptr = adreno_ringbuffer_dec_wrapped(temp_rptr, rb_size);
		kgsl_sharedmem_readl(&rb->buffer_desc, &value, temp_rptr);

		if (ib1 == value) {
			temp_rptr = adreno_ringbuffer_dec_wrapped(temp_rptr,
								rb_size);
			kgsl_sharedmem_readl(&rb->buffer_desc, &value,
						temp_rptr);
			if (adreno_cmd_is_ib(value)) {
				temp_rptr += 2 * sizeof(unsigned int);
				kgsl_sharedmem_readl(&rb->buffer_desc, &value,
						temp_rptr);
				return value;
			} else {
				temp_rptr += sizeof(unsigned int);
			}
		}
	} while (temp_rptr != rptr);

	return 0;
}

static unsigned int *adreno_ringbuffer_allocspace(struct adreno_ringbuffer *rb,
					     unsigned int numcmds)
{
	unsigned int	*ptr = NULL;

	BUG_ON(numcmds >= rb->sizedwords);

	GSL_RB_GET_READPTR(rb, &rb->rptr);
	/* check for available space */
	if (rb->wptr >= rb->rptr) {
		/* wptr ahead or equal to rptr */
		/* reserve dwords for nop packet */
		if ((rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			adreno_ringbuffer_waitspace(rb, numcmds, 1);
	} else {
		/* wptr behind rptr */
		if ((rb->wptr + numcmds) >= rb->rptr)
			adreno_ringbuffer_waitspace(rb, numcmds, 0);
		/* check for remaining space */
		/* reserve dwords for nop packet */
		if ((rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			adreno_ringbuffer_waitspace(rb, numcmds, 1);
	}

	ptr = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
	rb->wptr += numcmds;

	return ptr;
}

static int _load_firmware(struct kgsl_device *device, const char *fwfile,
			  void **data, int *len)
{
	const struct firmware *fw = NULL;
	int ret;

	ret = request_firmware(&fw, fwfile, device->dev);

	if (ret) {
		KGSL_DRV_ERR(device, "request_firmware(%s) failed: %d\n",
			     fwfile, ret);
		return ret;
	}

	*data = kmalloc(fw->size, GFP_KERNEL);

	if (*data) {
		memcpy(*data, fw->data, fw->size);
		*len = fw->size;
	} else
		KGSL_MEM_ERR(device, "kmalloc(%d) failed\n", fw->size);

	release_firmware(fw);
	return (*data != NULL) ? 0 : -ENOMEM;
}

static int adreno_ringbuffer_load_pm4_ucode(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i, ret = 0;

	if (adreno_dev->pm4_fw == NULL) {
		int len;
		void *ptr;

		ret = _load_firmware(device, adreno_dev->pm4_fwfile,
			&ptr, &len);

		if (ret)
			goto err;

		/* PM4 size is 3 dword aligned plus 1 dword of version */
		if (len % ((sizeof(uint32_t) * 3)) != sizeof(uint32_t)) {
			KGSL_DRV_ERR(device, "Bad firmware size: %d\n", len);
			ret = -EINVAL;
			kfree(ptr);
			goto err;
		}

		adreno_dev->pm4_fw_size = len / sizeof(uint32_t);
		adreno_dev->pm4_fw = ptr;
	}

	KGSL_DRV_INFO(device, "loading pm4 ucode version: %d\n",
		adreno_dev->pm4_fw[0]);

	adreno_regwrite(device, REG_CP_DEBUG, 0x02000000);
	adreno_regwrite(device, REG_CP_ME_RAM_WADDR, 0);
	for (i = 1; i < adreno_dev->pm4_fw_size; i++)
		adreno_regwrite(device, REG_CP_ME_RAM_DATA,
				     adreno_dev->pm4_fw[i]);
err:
	return ret;
}

static int adreno_ringbuffer_load_pfp_ucode(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i, ret = 0;

	if (adreno_dev->pfp_fw == NULL) {
		int len;
		void *ptr;

		ret = _load_firmware(device, adreno_dev->pfp_fwfile,
			&ptr, &len);
		if (ret)
			goto err;

		/* PFP size shold be dword aligned */
		if (len % sizeof(uint32_t) != 0) {
			KGSL_DRV_ERR(device, "Bad firmware size: %d\n", len);
			ret = -EINVAL;
			kfree(ptr);
			goto err;
		}

		adreno_dev->pfp_fw_size = len / sizeof(uint32_t);
		adreno_dev->pfp_fw = ptr;
	}

	KGSL_DRV_INFO(device, "loading pfp ucode version: %d\n",
		adreno_dev->pfp_fw[0]);

	adreno_regwrite(device, REG_CP_PFP_UCODE_ADDR, 0);
	for (i = 1; i < adreno_dev->pfp_fw_size; i++)
		adreno_regwrite(device, REG_CP_PFP_UCODE_DATA,
				     adreno_dev->pfp_fw[i]);
err:
	return ret;
}

int adreno_ringbuffer_start(struct adreno_ringbuffer *rb, unsigned int init_ram)
{
	int status;
	/*cp_rb_cntl_u cp_rb_cntl; */
	union reg_cp_rb_cntl cp_rb_cntl;
	unsigned int *cmds, rb_cntl;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	uint cmds_gpu;

	if (rb->flags & KGSL_FLAGS_STARTED)
		return 0;

	if (init_ram) {
		rb->timestamp = 0;
		GSL_RB_INIT_TIMESTAMP(rb);
	}

	kgsl_sharedmem_set(&rb->memptrs_desc, 0, 0,
			   sizeof(struct kgsl_rbmemptrs));

	kgsl_sharedmem_set(&rb->buffer_desc, 0, 0xAA,
			   (rb->sizedwords << 2));

	adreno_regwrite(device, REG_CP_RB_WPTR_BASE,
			     (rb->memptrs_desc.gpuaddr
			      + GSL_RB_MEMPTRS_WPTRPOLL_OFFSET));

	/* setup WPTR delay */
	adreno_regwrite(device, REG_CP_RB_WPTR_DELAY, 0 /*0x70000010 */);

	/*setup REG_CP_RB_CNTL */
	adreno_regread(device, REG_CP_RB_CNTL, &rb_cntl);
	cp_rb_cntl.val = rb_cntl;

	/*
	 * The size of the ringbuffer in the hardware is the log2
	 * representation of the size in quadwords (sizedwords / 2)
	 */
	cp_rb_cntl.f.rb_bufsz = ilog2(rb->sizedwords >> 1);

	/*
	 * Specify the quadwords to read before updating mem RPTR.
	 * Like above, pass the log2 representation of the blocksize
	 * in quadwords.
	*/
	cp_rb_cntl.f.rb_blksz = ilog2(KGSL_RB_BLKSIZE >> 3);

	cp_rb_cntl.f.rb_poll_en = GSL_RB_CNTL_POLL_EN; /* WPTR polling */
	/* mem RPTR writebacks */
	cp_rb_cntl.f.rb_no_update =  GSL_RB_CNTL_NO_UPDATE;

	adreno_regwrite(device, REG_CP_RB_CNTL, cp_rb_cntl.val);

	adreno_regwrite(device, REG_CP_RB_BASE, rb->buffer_desc.gpuaddr);

	adreno_regwrite(device, REG_CP_RB_RPTR_ADDR,
			     rb->memptrs_desc.gpuaddr +
			     GSL_RB_MEMPTRS_RPTR_OFFSET);

	/* explicitly clear all cp interrupts */
	adreno_regwrite(device, REG_CP_INT_ACK, 0xFFFFFFFF);

	/* setup scratch/timestamp */
	adreno_regwrite(device, REG_SCRATCH_ADDR,
			     device->memstore.gpuaddr +
			     KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp));

	adreno_regwrite(device, REG_SCRATCH_UMSK,
			     GSL_RB_MEMPTRS_SCRATCH_MASK);

	/* update the eoptimestamp field with the last retired timestamp */
	kgsl_sharedmem_writel(&device->memstore,
			     KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp),
			     rb->timestamp);

	/* load the CP ucode */

	status = adreno_ringbuffer_load_pm4_ucode(device);
	if (status != 0)
		return status;

	/* load the prefetch parser ucode */
	status = adreno_ringbuffer_load_pfp_ucode(device);
	if (status != 0)
		return status;

	adreno_regwrite(device, REG_CP_QUEUE_THRESHOLDS, 0x000C0804);

	rb->rptr = 0;
	rb->wptr = 0;

	/* clear ME_HALT to start micro engine */
	adreno_regwrite(device, REG_CP_ME_CNTL, 0);

	/* ME_INIT */
	cmds = adreno_ringbuffer_allocspace(rb, 19);
	cmds_gpu = rb->buffer_desc.gpuaddr + sizeof(uint)*(rb->wptr-19);

	GSL_RB_WRITE(cmds, cmds_gpu, CP_HDR_ME_INIT);
	/* All fields present (bits 9:0) */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x000003ff);
	/* Disable/Enable Real-Time Stream processing (present but ignored) */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);
	/* Enable (2D <-> 3D) implicit synchronization (present but ignored) */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);

	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_RB_SURFACE_INFO));
	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_PA_SC_WINDOW_OFFSET));
	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_VGT_MAX_VTX_INDX));
	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_SQ_PROGRAM_CNTL));
	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_RB_DEPTHCONTROL));
	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_PA_SU_POINT_SIZE));
	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_PA_SC_LINE_CNTL));
	GSL_RB_WRITE(cmds, cmds_gpu,
		SUBBLOCK_OFFSET(REG_PA_SU_POLY_OFFSET_FRONT_SCALE));

	/* Instruction memory size: */
	GSL_RB_WRITE(cmds, cmds_gpu,
		     (adreno_encode_istore_size(adreno_dev)
		      | adreno_dev->pix_shader_start));
	/* Maximum Contexts */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000001);
	/* Write Confirm Interval and The CP will wait the
	* wait_interval * 16 clocks between polling  */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);

	/* NQ and External Memory Swap */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);
	/* Protected mode error checking */
	GSL_RB_WRITE(cmds, cmds_gpu, GSL_RB_PROTECTED_MODE_CONTROL);
	/* Disable header dumping and Header dump address */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);
	/* Header dump size */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);

	adreno_ringbuffer_submit(rb);

	/* idle device to validate ME INIT */
	status = adreno_idle(device, KGSL_TIMEOUT_DEFAULT);

	if (status == 0)
		rb->flags |= KGSL_FLAGS_STARTED;

	return status;
}

void adreno_ringbuffer_stop(struct adreno_ringbuffer *rb)
{
	if (rb->flags & KGSL_FLAGS_STARTED) {
		/* ME_HALT */
		adreno_regwrite(rb->device, REG_CP_ME_CNTL, 0x10000000);
		rb->flags &= ~KGSL_FLAGS_STARTED;
	}
}

int adreno_ringbuffer_init(struct kgsl_device *device)
{
	int status;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;

	rb->device = device;
	/*
	 * It is silly to convert this to words and then back to bytes
	 * immediately below, but most of the rest of the code deals
	 * in words, so we might as well only do the math once
	 */
	rb->sizedwords = KGSL_RB_SIZE >> 2;

	/* allocate memory for ringbuffer */
	status = kgsl_allocate_contiguous(&rb->buffer_desc,
		(rb->sizedwords << 2));

	if (status != 0) {
		adreno_ringbuffer_close(rb);
		return status;
	}

	/* allocate memory for polling and timestamps */
	/* This really can be at 4 byte alignment boundry but for using MMU
	 * we need to make it at page boundary */
	status = kgsl_allocate_contiguous(&rb->memptrs_desc,
		sizeof(struct kgsl_rbmemptrs));

	if (status != 0) {
		adreno_ringbuffer_close(rb);
		return status;
	}

	/* overlay structure on memptrs memory */
	rb->memptrs = (struct kgsl_rbmemptrs *) rb->memptrs_desc.hostptr;

	return 0;
}

void adreno_ringbuffer_close(struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);

	kgsl_sharedmem_free(&rb->buffer_desc);
	kgsl_sharedmem_free(&rb->memptrs_desc);

	kfree(adreno_dev->pfp_fw);
	kfree(adreno_dev->pm4_fw);

	adreno_dev->pfp_fw = NULL;
	adreno_dev->pm4_fw = NULL;

	memset(rb, 0, sizeof(struct adreno_ringbuffer));
}

static uint32_t
adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
				struct adreno_context *context,
				unsigned int flags, unsigned int *cmds,
				int sizedwords)
{
	unsigned int *ringcmds;
	unsigned int timestamp;
	unsigned int total_sizedwords = sizedwords + 6;
	unsigned int i;
	unsigned int rcmd_gpu;

	/* reserve space to temporarily turn off protected mode
	*  error checking if needed
	*/
	total_sizedwords += flags & KGSL_CMD_FLAGS_PMODE ? 4 : 0;
	total_sizedwords += !(flags & KGSL_CMD_FLAGS_NO_TS_CMP) ? 7 : 0;
	/* 2 dwords to store the start of command sequence */
	total_sizedwords += 2;

	ringcmds = adreno_ringbuffer_allocspace(rb, total_sizedwords);
	/* GPU may hang during space allocation, if thats the case the current
	 * context may have hung the GPU */
	if (context->flags & CTXT_FLAGS_GPU_HANG) {
		KGSL_CTXT_WARN(rb->device,
		"Context %p caused a gpu hang. Will not accept commands for context %d\n",
		context, context->id);
		return rb->timestamp;
	}

	rcmd_gpu = rb->buffer_desc.gpuaddr
		+ sizeof(uint)*(rb->wptr-total_sizedwords);

	GSL_RB_WRITE(ringcmds, rcmd_gpu, cp_nop_packet(1));
	GSL_RB_WRITE(ringcmds, rcmd_gpu, KGSL_CMD_IDENTIFIER);

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		/* disable protected mode error checking */
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			cp_type3_packet(CP_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, 0);
	}

	for (i = 0; i < sizedwords; i++) {
		GSL_RB_WRITE(ringcmds, rcmd_gpu, *cmds);
		cmds++;
	}

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		/* re-enable protected mode error checking */
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			cp_type3_packet(CP_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, 1);
	}

	rb->timestamp++;
	timestamp = rb->timestamp;

	/* start-of-pipeline and end-of-pipeline timestamps */
	GSL_RB_WRITE(ringcmds, rcmd_gpu, cp_type0_packet(REG_CP_TIMESTAMP, 1));
	GSL_RB_WRITE(ringcmds, rcmd_gpu, rb->timestamp);
	GSL_RB_WRITE(ringcmds, rcmd_gpu, cp_type3_packet(CP_EVENT_WRITE, 3));
	GSL_RB_WRITE(ringcmds, rcmd_gpu, CACHE_FLUSH_TS);
	GSL_RB_WRITE(ringcmds, rcmd_gpu,
		     (rb->device->memstore.gpuaddr +
		      KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp)));
	GSL_RB_WRITE(ringcmds, rcmd_gpu, rb->timestamp);

	if (!(flags & KGSL_CMD_FLAGS_NO_TS_CMP)) {
		/* Conditional execution based on memory values */
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			cp_type3_packet(CP_COND_EXEC, 4));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, (rb->device->memstore.gpuaddr +
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable)) >> 2);
		GSL_RB_WRITE(ringcmds, rcmd_gpu, (rb->device->memstore.gpuaddr +
			KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts)) >> 2);
		GSL_RB_WRITE(ringcmds, rcmd_gpu, rb->timestamp);
		/* # of conditional command DWORDs */
		GSL_RB_WRITE(ringcmds, rcmd_gpu, 2);
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			cp_type3_packet(CP_INTERRUPT, 1));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, CP_INT_CNTL__RB_INT_MASK);
	}

	adreno_ringbuffer_submit(rb);

	/* return timestamp of issued coREG_ands */
	return timestamp;
}

void
adreno_ringbuffer_issuecmds(struct kgsl_device *device,
						struct adreno_context *drawctxt,
						unsigned int flags,
						unsigned int *cmds,
						int sizedwords)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;

	if (device->state & KGSL_STATE_HUNG)
		return;

	adreno_ringbuffer_addcmds(rb, drawctxt, flags, cmds, sizedwords);
}

static bool _parse_ibs(struct kgsl_device_private *dev_priv, uint gpuaddr,
			   int sizedwords);

static bool
_handle_type3(struct kgsl_device_private *dev_priv, uint *hostaddr)
{
	unsigned int opcode = cp_type3_opcode(*hostaddr);
	switch (opcode) {
	case CP_INDIRECT_BUFFER_PFD:
	case CP_INDIRECT_BUFFER_PFE:
	case CP_COND_INDIRECT_BUFFER_PFE:
	case CP_COND_INDIRECT_BUFFER_PFD:
		return _parse_ibs(dev_priv, hostaddr[1], hostaddr[2]);
	case CP_NOP:
	case CP_WAIT_FOR_IDLE:
	case CP_WAIT_REG_MEM:
	case CP_WAIT_REG_EQ:
	case CP_WAT_REG_GTE:
	case CP_WAIT_UNTIL_READ:
	case CP_WAIT_IB_PFD_COMPLETE:
	case CP_REG_RMW:
	case CP_REG_TO_MEM:
	case CP_MEM_WRITE:
	case CP_MEM_WRITE_CNTR:
	case CP_COND_EXEC:
	case CP_COND_WRITE:
	case CP_EVENT_WRITE:
	case CP_EVENT_WRITE_SHD:
	case CP_EVENT_WRITE_CFL:
	case CP_EVENT_WRITE_ZPD:
	case CP_DRAW_INDX:
	case CP_DRAW_INDX_2:
	case CP_DRAW_INDX_BIN:
	case CP_DRAW_INDX_2_BIN:
	case CP_VIZ_QUERY:
	case CP_SET_STATE:
	case CP_SET_CONSTANT:
	case CP_IM_LOAD:
	case CP_IM_LOAD_IMMEDIATE:
	case CP_LOAD_CONSTANT_CONTEXT:
	case CP_INVALIDATE_STATE:
	case CP_SET_SHADER_BASES:
	case CP_SET_BIN_MASK:
	case CP_SET_BIN_SELECT:
	case CP_SET_BIN_BASE_OFFSET:
	case CP_SET_BIN_DATA:
	case CP_CONTEXT_UPDATE:
	case CP_INTERRUPT:
	case CP_IM_STORE:
		break;
	/* these shouldn't come from userspace */
	case CP_ME_INIT:
	case CP_SET_PROTECTED_MODE:
	default:
		KGSL_CMD_ERR(dev_priv->device, "bad CP opcode %0x\n", opcode);
		return false;
		break;
	}

	return true;
}

static bool
_handle_type0(struct kgsl_device_private *dev_priv, uint *hostaddr)
{
	unsigned int reg = type0_pkt_offset(*hostaddr);
	unsigned int cnt = type0_pkt_size(*hostaddr);
	if (reg < 0x0192 || (reg + cnt) >= 0x8000) {
		KGSL_CMD_ERR(dev_priv->device, "bad type0 reg: 0x%0x cnt: %d\n",
			     reg, cnt);
		return false;
	}
	return true;
}

/*
 * Traverse IBs and dump them to test vector. Detect swap by inspecting
 * register writes, keeping note of the current state, and dump
 * framebuffer config to test vector
 */
static bool _parse_ibs(struct kgsl_device_private *dev_priv,
			   uint gpuaddr, int sizedwords)
{
	static uint level; /* recursion level */
	bool ret = false;
	uint *hostaddr, *hoststart;
	int dwords_left = sizedwords; /* dwords left in the current command
					 buffer */
	struct kgsl_mem_entry *entry;

	spin_lock(&dev_priv->process_priv->mem_lock);
	entry = kgsl_sharedmem_find_region(dev_priv->process_priv,
					   gpuaddr, sizedwords * sizeof(uint));
	spin_unlock(&dev_priv->process_priv->mem_lock);
	if (entry == NULL) {
		KGSL_CMD_ERR(dev_priv->device,
			     "no mapping for gpuaddr: 0x%08x\n", gpuaddr);
		return false;
	}

	hostaddr = (uint *)kgsl_gpuaddr_to_vaddr(&entry->memdesc, gpuaddr);
	if (hostaddr == NULL) {
		KGSL_CMD_ERR(dev_priv->device,
			     "no mapping for gpuaddr: 0x%08x\n", gpuaddr);
		return false;
	}

	hoststart = hostaddr;

	level++;

	KGSL_CMD_INFO(dev_priv->device, "ib: gpuaddr:0x%08x, wc:%d, hptr:%p\n",
		gpuaddr, sizedwords, hostaddr);

	mb();
	while (dwords_left > 0) {
		bool cur_ret = true;
		int count = 0; /* dword count including packet header */

		switch (*hostaddr >> 30) {
		case 0x0: /* type-0 */
			count = (*hostaddr >> 16)+2;
			cur_ret = _handle_type0(dev_priv, hostaddr);
			break;
		case 0x1: /* type-1 */
			count = 2;
			break;
		case 0x3: /* type-3 */
			count = ((*hostaddr >> 16) & 0x3fff) + 2;
			cur_ret = _handle_type3(dev_priv, hostaddr);
			break;
		default:
			KGSL_CMD_ERR(dev_priv->device, "unexpected type: "
				"type:%d, word:0x%08x @ 0x%p, gpu:0x%08x\n",
				*hostaddr >> 30, *hostaddr, hostaddr,
				gpuaddr+4*(sizedwords-dwords_left));
			cur_ret = false;
			count = dwords_left;
			break;
		}

		if (!cur_ret) {
			KGSL_CMD_ERR(dev_priv->device,
				"bad sub-type: #:%d/%d, v:0x%08x"
				" @ 0x%p[gb:0x%08x], level:%d\n",
				sizedwords-dwords_left, sizedwords, *hostaddr,
				hostaddr, gpuaddr+4*(sizedwords-dwords_left),
				level);

			if (ADRENO_DEVICE(dev_priv->device)->ib_check_level
				>= 2)
				print_hex_dump(KERN_ERR,
					level == 1 ? "IB1:" : "IB2:",
					DUMP_PREFIX_OFFSET, 32, 4, hoststart,
					sizedwords*4, 0);
			goto done;
		}

		/* jump to next packet */
		dwords_left -= count;
		hostaddr += count;
		if (dwords_left < 0) {
			KGSL_CMD_ERR(dev_priv->device,
				"bad count: c:%d, #:%d/%d, "
				"v:0x%08x @ 0x%p[gb:0x%08x], level:%d\n",
				count, sizedwords-(dwords_left+count),
				sizedwords, *(hostaddr-count), hostaddr-count,
				gpuaddr+4*(sizedwords-(dwords_left+count)),
				level);
			if (ADRENO_DEVICE(dev_priv->device)->ib_check_level
				>= 2)
				print_hex_dump(KERN_ERR,
					level == 1 ? "IB1:" : "IB2:",
					DUMP_PREFIX_OFFSET, 32, 4, hoststart,
					sizedwords*4, 0);
			goto done;
		}
	}

	ret = true;
done:
	if (!ret)
		KGSL_DRV_ERR(dev_priv->device,
			"parsing failed: gpuaddr:0x%08x, "
			"host:0x%p, wc:%d\n", gpuaddr, hoststart, sizedwords);

	level--;

	return ret;
}

int
adreno_ringbuffer_issueibcmds(struct kgsl_device_private *dev_priv,
				struct kgsl_context *context,
				struct kgsl_ibdesc *ibdesc,
				unsigned int numibs,
				uint32_t *timestamp,
				unsigned int flags)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int *link;
	unsigned int *cmds;
	unsigned int i;
	struct adreno_context *drawctxt;
	unsigned int start_index = 0;

	if (device->state & KGSL_STATE_HUNG)
		return -EBUSY;
	if (!(adreno_dev->ringbuffer.flags & KGSL_FLAGS_STARTED) ||
	      context == NULL || ibdesc == 0 || numibs == 0)
		return -EINVAL;

	drawctxt = context->devctxt;

	if (drawctxt->flags & CTXT_FLAGS_GPU_HANG) {
		KGSL_CTXT_WARN(device, "Context %p caused a gpu hang.."
			" will not accept commands for this context\n",
			drawctxt);
		return -EDEADLK;
	}
	link = kzalloc(sizeof(unsigned int) * numibs * 3, GFP_KERNEL);
	cmds = link;
	if (!link) {
		KGSL_MEM_ERR(device, "Failed to allocate memory for for command"
			" submission, size %x\n", numibs * 3);
		return -ENOMEM;
	}

	/*When preamble is enabled, the preamble buffer with state restoration
	commands are stored in the first node of the IB chain. We can skip that
	if a context switch hasn't occured */

	if (drawctxt->flags & CTXT_FLAGS_PREAMBLE &&
		adreno_dev->drawctxt_active == drawctxt)
		start_index = 1;

	for (i = start_index; i < numibs; i++) {
		if (unlikely(adreno_dev->ib_check_level >= 1 &&
		    !_parse_ibs(dev_priv, ibdesc[i].gpuaddr,
				ibdesc[i].sizedwords))) {
			kfree(link);
			return -EINVAL;
		}
		*cmds++ = CP_HDR_INDIRECT_BUFFER_PFD;
		*cmds++ = ibdesc[i].gpuaddr;
		*cmds++ = ibdesc[i].sizedwords;
	}

	kgsl_setstate(device, context->id,
		      kgsl_mmu_pt_get_flags(device->mmu.hwpagetable,
			device->id));

	adreno_drawctxt_switch(adreno_dev, drawctxt, flags);

	*timestamp = adreno_ringbuffer_addcmds(&adreno_dev->ringbuffer,
					drawctxt, 0,
					&link[0], (cmds - link));

	KGSL_CMD_INFO(device, "ctxt %d g %08x numibs %d ts %d\n",
		context->id, (unsigned int)ibdesc, numibs, *timestamp);

	kfree(link);

#ifdef CONFIG_MSM_KGSL_CFF_DUMP
	/*
	 * insert wait for idle after every IB1
	 * this is conservative but works reliably and is ok
	 * even for performance simulations
	 */
	adreno_idle(device, KGSL_TIMEOUT_DEFAULT);
#endif

	return 0;
}

static int _find_start_of_cmd_seq(struct adreno_ringbuffer *rb,
					unsigned int *ptr,
					bool inc)
{
	int status = -EINVAL;
	unsigned int val1;
	unsigned int size = rb->buffer_desc.size;
	unsigned int start_ptr = *ptr;

	while ((start_ptr / sizeof(unsigned int)) != rb->wptr) {
		if (inc)
			start_ptr = adreno_ringbuffer_inc_wrapped(start_ptr,
									size);
		else
			start_ptr = adreno_ringbuffer_dec_wrapped(start_ptr,
									size);
		kgsl_sharedmem_readl(&rb->buffer_desc, &val1, start_ptr);
		if (KGSL_CMD_IDENTIFIER == val1) {
			if ((start_ptr / sizeof(unsigned int)) != rb->wptr)
				start_ptr = adreno_ringbuffer_dec_wrapped(
							start_ptr, size);
				*ptr = start_ptr;
				status = 0;
				break;
		}
	}
	return status;
}

static int _find_cmd_seq_after_eop_ts(struct adreno_ringbuffer *rb,
					unsigned int *rb_rptr,
					unsigned int global_eop,
					bool inc)
{
	int status = -EINVAL;
	unsigned int temp_rb_rptr = *rb_rptr;
	unsigned int size = rb->buffer_desc.size;
	unsigned int val[3];
	int i = 0;
	bool check = false;

	if (inc && temp_rb_rptr / sizeof(unsigned int) != rb->wptr)
		return status;

	do {
		/* when decrementing we need to decrement first and
		 * then read make sure we cover all the data */
		if (!inc)
			temp_rb_rptr = adreno_ringbuffer_dec_wrapped(
					temp_rb_rptr, size);
		kgsl_sharedmem_readl(&rb->buffer_desc, &val[i],
					temp_rb_rptr);

		if (check && ((inc && val[i] == global_eop) ||
			(!inc && (val[i] ==
			cp_type3_packet(CP_MEM_WRITE, 2) ||
			val[i] == CACHE_FLUSH_TS)))) {
			/* decrement i, i.e i = (i - 1 + 3) % 3 if
			 * we are going forward, else increment i */
			i = (i + 2) % 3;
			if (val[i] == rb->device->memstore.gpuaddr +
				KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp)) {
				int j = ((i + 2) % 3);
				if ((inc && (val[j] == CACHE_FLUSH_TS ||
						val[j] == cp_type3_packet(
							CP_MEM_WRITE, 2))) ||
					(!inc && val[j] == global_eop)) {
						/* Found the global eop */
						status = 0;
						break;
				}
			}
			/* if no match found then increment i again
			 * since we decremented before matching */
			i = (i + 1) % 3;
		}
		if (inc)
			temp_rb_rptr = adreno_ringbuffer_inc_wrapped(
						temp_rb_rptr, size);

		i = (i + 1) % 3;
		if (2 == i)
			check = true;
	} while (temp_rb_rptr / sizeof(unsigned int) != rb->wptr);
	/* temp_rb_rptr points to the global eop, move forward till
	 * the next command */
	if (!status) {
		status = _find_start_of_cmd_seq(rb, &temp_rb_rptr, true);
		if (!status) {
			*rb_rptr = temp_rb_rptr;
			KGSL_DRV_ERR(rb->device,
			"Offset of cmd sequence after eop timestamp: 0x%x\n",
			temp_rb_rptr / sizeof(unsigned int));
		}
	}
	return status;
}

static int _find_hanging_ib_sequence(struct adreno_ringbuffer *rb,
				unsigned int *rb_rptr,
				unsigned int ib1)
{
	int status = -EINVAL;
	unsigned int temp_rb_rptr = *rb_rptr;
	unsigned int size = rb->buffer_desc.size;
	unsigned int val[2];
	int i = 0;
	bool check = false;
	bool ctx_switch = false;

	while (temp_rb_rptr / sizeof(unsigned int) != rb->wptr) {
		kgsl_sharedmem_readl(&rb->buffer_desc, &val[i], temp_rb_rptr);

		if (check && val[i] == ib1) {
			/* decrement i, i.e i = (i - 1 + 2) % 2 */
			i = (i + 1) % 2;
			if (adreno_cmd_is_ib(val[i])) {
				/* go till start of command sequence */
				status = _find_start_of_cmd_seq(rb,
						&temp_rb_rptr, false);
				KGSL_DRV_ERR(rb->device,
				"Found the hanging IB at offset 0x%x\n",
				temp_rb_rptr / sizeof(unsigned int));
				break;
			}
			/* if no match the increment i since we decremented
			 * before checking */
			i = (i + 1) % 2;
		}
		/* Make sure you do not encounter a context switch twice, we can
		 * encounter it once for the bad context as the start of search
		 * can point to the context switch */
		if (val[i] == KGSL_CONTEXT_TO_MEM_IDENTIFIER) {
			if (ctx_switch) {
				KGSL_DRV_ERR(rb->device,
				"Context switch encountered before bad "
				"IB found\n");
				break;
			}
			ctx_switch = true;
		}
		i = (i + 1) % 2;
		if (1 == i)
			check = true;
		temp_rb_rptr = adreno_ringbuffer_inc_wrapped(temp_rb_rptr,
								size);
	}
	if  (!status)
		*rb_rptr = temp_rb_rptr;
	return status;
}

static void _turn_preamble_on_for_ib_seq(struct adreno_ringbuffer *rb,
				unsigned int rb_rptr)
{
	unsigned int temp_rb_rptr = rb_rptr;
	unsigned int size = rb->buffer_desc.size;
	unsigned int val[2];
	int i = 0;
	bool check = false;
	bool cmd_start = false;

	/* Go till the start of the ib sequence and turn on preamble */
	while (temp_rb_rptr / sizeof(unsigned int) != rb->wptr) {
		kgsl_sharedmem_readl(&rb->buffer_desc, &val[i], temp_rb_rptr);
		if (check && KGSL_START_OF_IB_IDENTIFIER == val[i]) {
			/* decrement i */
			i = (i + 1) % 2;
			if (val[i] == cp_nop_packet(4)) {
				temp_rb_rptr = adreno_ringbuffer_dec_wrapped(
						temp_rb_rptr, size);
				kgsl_sharedmem_writel(&rb->buffer_desc,
					temp_rb_rptr, cp_nop_packet(1));
			}
			KGSL_DRV_ERR(rb->device,
			"Turned preamble on at offset 0x%x\n",
			temp_rb_rptr / 4);
			break;
		}
		/* If you reach beginning of next command sequence then exit
		 * First command encountered is the current one so don't break
		 * on that. */
		if (KGSL_CMD_IDENTIFIER == val[i]) {
			if (cmd_start)
				break;
			cmd_start = true;
		}

		i = (i + 1) % 2;
		if (1 == i)
			check = true;
		temp_rb_rptr = adreno_ringbuffer_inc_wrapped(temp_rb_rptr,
								size);
	}
}

static void _copy_valid_rb_content(struct adreno_ringbuffer *rb,
		unsigned int rb_rptr, unsigned int *temp_rb_buffer,
		int *rb_size, unsigned int *bad_rb_buffer,
		int *bad_rb_size,
		int *last_valid_ctx_id)
{
	unsigned int good_rb_idx = 0, cmd_start_idx = 0;
	unsigned int val1 = 0;
	struct kgsl_context *k_ctxt;
	struct adreno_context *a_ctxt;
	unsigned int bad_rb_idx = 0;
	int copy_rb_contents = 0;
	unsigned int temp_rb_rptr;
	unsigned int size = rb->buffer_desc.size;
	unsigned int good_cmd_start_idx = 0;

	/* Walk the rb from the context switch. Omit any commands
	 * for an invalid context. */
	while ((rb_rptr / sizeof(unsigned int)) != rb->wptr) {
		kgsl_sharedmem_readl(&rb->buffer_desc, &val1, rb_rptr);

		if (KGSL_CMD_IDENTIFIER == val1) {
			/* Start is the NOP dword that comes before
			 * KGSL_CMD_IDENTIFIER */
			cmd_start_idx = bad_rb_idx - 1;
			if (copy_rb_contents)
				good_cmd_start_idx = good_rb_idx - 1;
		}

		/* check for context switch indicator */
		if (val1 == KGSL_CONTEXT_TO_MEM_IDENTIFIER) {
			unsigned int temp_idx, val2;
			/* increment by 3 to get to the context_id */
			temp_rb_rptr = rb_rptr + (3 * sizeof(unsigned int)) %
					size;
			kgsl_sharedmem_readl(&rb->buffer_desc, &val2,
						temp_rb_rptr);

			/* if context switches to a context that did not cause
			 * hang then start saving the rb contents as those
			 * commands can be executed */
			k_ctxt = idr_find(&rb->device->context_idr, val2);
			if (k_ctxt) {
				a_ctxt = k_ctxt->devctxt;

			/* If we are changing to a good context and were not
			 * copying commands then copy over commands to the good
			 * context */
			if (!copy_rb_contents && ((k_ctxt &&
				!(a_ctxt->flags & CTXT_FLAGS_GPU_HANG)) ||
				!k_ctxt)) {
				for (temp_idx = cmd_start_idx;
					temp_idx < bad_rb_idx;
					temp_idx++)
					temp_rb_buffer[good_rb_idx++] =
						bad_rb_buffer[temp_idx];
				*last_valid_ctx_id = val2;
				copy_rb_contents = 1;
			} else if (copy_rb_contents && k_ctxt &&
				(a_ctxt->flags & CTXT_FLAGS_GPU_HANG)) {
				/* If we are changing to bad context then remove
				 * the dwords we copied for this sequence from
				 * the good buffer */
				good_rb_idx = good_cmd_start_idx;
				copy_rb_contents = 0;
			}
			}
		}

		if (copy_rb_contents)
			temp_rb_buffer[good_rb_idx++] = val1;
		/* Copy both good and bad commands for replay to the bad
		 * buffer */
		bad_rb_buffer[bad_rb_idx++] = val1;

		rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr, size);
	}
	*rb_size = good_rb_idx;
	*bad_rb_size = bad_rb_idx;
}

int adreno_ringbuffer_extract(struct adreno_ringbuffer *rb,
				struct adreno_recovery_data *rec_data)
{
	int status;
	struct kgsl_device *device = rb->device;
	unsigned int rb_rptr = rb->wptr * sizeof(unsigned int);
	struct kgsl_context *context;
	struct adreno_context *adreno_context;

	context = idr_find(&device->context_idr, rec_data->context_id);
	status = _find_cmd_seq_after_eop_ts(rb, &rb_rptr, rec_data->global_eop,
						false);
	if (status)
		goto done;

	if (context) {
		adreno_context = context->devctxt;

		if (adreno_context->flags & CTXT_FLAGS_PREAMBLE) {
			if (rec_data->ib1) {
				status = _find_hanging_ib_sequence(rb, &rb_rptr,
								rec_data->ib1);
				if (status)
					goto copy_rb_contents;
			}
			_turn_preamble_on_for_ib_seq(rb, rb_rptr);
		}
	}

copy_rb_contents:
	_copy_valid_rb_content(rb, rb_rptr, rec_data->rb_buffer,
				&rec_data->rb_size,
				rec_data->bad_rb_buffer,
				&rec_data->bad_rb_size,
				&rec_data->last_valid_ctx_id);
	/* If we failed to get the hanging IB sequence then we cannot execute
	 * commands from the bad context */
	if (status) {
		rec_data->bad_rb_size = 0;
		status = 0;
	}
done:
	return status;
}

void
adreno_ringbuffer_restore(struct adreno_ringbuffer *rb, unsigned int *rb_buff,
			int num_rb_contents)
{
	int i;
	unsigned int *ringcmds;
	unsigned int rcmd_gpu;

	if (!num_rb_contents)
		return;

	if (num_rb_contents > (rb->buffer_desc.size - rb->wptr)) {
		adreno_regwrite(rb->device, REG_CP_RB_RPTR, 0);
		rb->rptr = 0;
		BUG_ON(num_rb_contents > rb->buffer_desc.size);
	}
	ringcmds = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
	rcmd_gpu = rb->buffer_desc.gpuaddr + sizeof(unsigned int) * rb->wptr;
	for (i = 0; i < num_rb_contents; i++)
		GSL_RB_WRITE(ringcmds, rcmd_gpu, rb_buff[i]);
	rb->wptr += num_rb_contents;
	adreno_ringbuffer_submit(rb);
}

void adreno_print_fault_ib_work(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
				print_fault_ib);
	mutex_lock(&device->mutex);
	adreno_print_fault_ib(device);
	mutex_unlock(&device->mutex);
}

void adreno_print_fault_ib(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct ib_list ib_list;
	int i;

	unsigned int ib_sz;

	if (!device->page_fault_ptbase ||
		!is_adreno_ib_dump_on_pagef_enabled(device))
		goto done;

	ib_sz = find_faulting_ib1_size(&adreno_dev->ringbuffer,
					device->page_fault_rptr,
					device->page_fault_ib1);
	if (!ib_sz) {
		KGSL_DRV_ERR(device, "Could not find size of fault ib 0x%x\n",
				device->page_fault_ib1);
		goto done;
	}
	ib_list.count = 0;

	KGSL_DRV_ERR(device, "Page faulting IB1 0x%x, size 0x%x\n",
			device->page_fault_ib1, ib_sz);
	dump_ib1(device, device->page_fault_ptbase, 0,
		device->page_fault_ib1, ib_sz, &ib_list, true);

	/* print ib2's in faulting ib1 */
	for (i = 0; i < ib_list.count; i++) {
		KGSL_DRV_ERR(device, "IB2 0x%x, size 0x%x\n",
			ib_list.bases[i], ib_list.sizes[i]);
		dump_ib(device, "IB2:", device->page_fault_ptbase,
			ib_list.offsets[i], ib_list.bases[i], ib_list.sizes[i],
			true);
	}
	KGSL_DRV_ERR(device, "Finished printing fault IB\n");
done:
	device->page_fault_ptbase = 0;
}

/*
 * STP support for the Paintbox programmable IPU
 *
 * Copyright (C) 2015 Google, Inc.
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

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-bus.h"
#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"
#include "paintbox-sram.h"
#include "paintbox-stp.h"
#include "paintbox-stp-debug.h"
#include "paintbox-stp-pc-histogram.h"
#include "paintbox-stp-sram.h"

/* To save 1000s of cycles in the perf thread's inner loop, we keep a cache of
 * each stp's enabled bit in the paintbox_stp structure.  Ensure that this bit
 * is kept up-to-date by passing every STP_CTRL write through this function
 *
 * The caller to this function must hold pb->stp.lock
 */
static inline void stp_ctrl_write(struct paintbox_data *pb,
		struct paintbox_stp *stp, uint32_t ctrl)
{
	paintbox_writel(pb->dev, ctrl, IPU_CSR_STP_OFFSET + STP_CTRL);
	stp->enabled = (ctrl & STP_CTRL_ENA_MASK) != 0;
}

/* The caller to this function must hold pb->lock */
int validate_stp(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id)
{
	unsigned int stp_index = stp_id_to_index(stp_id);

	if (stp_index >= pb->stp.num_stps) {
		dev_err(pb->dev, "%s: invalid stp_id %d\n", __func__, stp_id);
		return -EINVAL;
	}

	if (pb->stp.stps[stp_index].session != session) {
		dev_err(pb->dev, "%s: access error stp_id %d\n", __func__,
				stp_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_stp *get_stp(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int stp_id, int *err)
{
	int ret = validate_stp(pb, session, stp_id);

	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->stp.stps[stp_id_to_index(stp_id)];
}

/* The LBP access control masks are not implemented on the V0 IPU. */
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
/* This function is used to set the LBP access control mask for an STP on a
 * newly allocated LBP.
 * The caller to this function must hold pb->lock
 */
void enable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp)
{
	struct paintbox_stp *stp, *stp_next;
	uint32_t mask;

	/* Grant access to the LBP to all STPs in the session. */
	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		unsigned long irq_flags;

		spin_lock_irqsave(&pb->stp.lock, irq_flags);

		paintbox_stp_select(pb, stp->stp_id);

		mask = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_MASK);
		mask |= 1 << (lbp->pool_id + STP_MASK_LBP_MASK_SHIFT);
		paintbox_writel(pb->dev, mask, IPU_CSR_STP_OFFSET + STP_MASK);

		spin_unlock_irqrestore(&pb->stp.lock, irq_flags);
	}
}

/* This function is used to clear the LBP access control mask for an STP on a
 * released LBP.
 * The caller to this function must hold pb->lock
 */
void disable_stp_access_to_lbp(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_lbp *lbp)
{
	struct paintbox_stp *stp, *stp_next;
	uint32_t mask;

	/* Disable access to the LBP for all STPs in the session. */
	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		unsigned long irq_flags;

		spin_lock_irqsave(&pb->stp.lock, irq_flags);

		paintbox_stp_select(pb, stp->stp_id);

		mask = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_MASK);
		mask &= ~(1 << (lbp->pool_id + STP_MASK_LBP_MASK_SHIFT));
		paintbox_writel(pb->dev, mask, IPU_CSR_STP_OFFSET + STP_MASK);

		spin_unlock_irqrestore(&pb->stp.lock, irq_flags);
	}
}

static void set_lbp_access_mask(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp)
{
	struct paintbox_lbp *lbp, *lbp_next;
	unsigned long irq_flags;
	uint32_t mask;

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);

	/* Grant access to all LBPs associated with this session */
	mask = 0;

	list_for_each_entry_safe(lbp, lbp_next, &session->lbp_list,
			session_entry)
		mask |= 1 << (lbp->pool_id + STP_MASK_LBP_MASK_SHIFT);

	paintbox_writel(pb->dev, mask, IPU_CSR_STP_OFFSET + STP_MASK);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);
}
#endif

/* The caller to this function must hold pb->lock */
int allocate_stp(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned int stp_index)
{
	struct paintbox_stp *stp;

	stp = &pb->stp.stps[stp_index];
	if (stp->session) {
		dev_err(pb->dev, "%s: access error stp_id %d\n", __func__,
				stp_index_to_id(stp_index));
		return -EACCES;
	}

	stp->session = session;
	list_add_tail(&stp->session_entry, &session->stp_list);

	session->stp_count++;

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	paintbox_pm_stp_enable(pb, stp);
#endif

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	set_lbp_access_mask(pb, session, stp);
#endif
	pb->stp.available_stp_mask &= ~(1ULL << stp_index);
	dev_dbg(pb->dev, "stp%u allocated\n", stp_index_to_id(stp_index));

	return 0;
}

int allocate_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;
	unsigned int stp_id = (unsigned int)arg;
	unsigned int stp_index = stp_id_to_index(stp_id);

	if (stp_index >= pb->stp.num_stps) {
		dev_err(pb->dev, "%s: invalid stp_id %d\n", __func__, stp_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);

	ret = allocate_stp(pb, session, stp_index);
	if (ret < 0) {
		dev_err(pb->dev, "%s: allocate stp_id %d error %d\n",
				__func__, stp_index_to_id(stp_index), ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	mutex_unlock(&pb->lock);

	return ret;
}

/* The caller to this function must hold pb->lock */
void release_stp(struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_stp *stp)
{
	unsigned long irq_flags;
	int stp_index;
	uint64_t stp_mask = 1;

	/* If this STP has pc sampling enabled, we need to disable it */
	stp_index = stp_id_to_index(stp->stp_id);
	stp_mask <<= stp_index;
	if (pb->perf_stp_sample_mask & stp_mask) {
		uint64_t next_mask = pb->perf_stp_sample_mask & ~stp_mask;

		stp_pc_histogram_enable(pb, next_mask);
	}

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);

	stp_ctrl_write(pb, stp, STP_CTRL_RESET_MASK);
	stp_ctrl_write(pb, stp, 0);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
	paintbox_pm_stp_disable(pb, stp);
#endif

	/* Remove the processor from the session. */
	list_del(&stp->session_entry);

	if (WARN_ON(--session->stp_count < 0))
		session->stp_count = 0;

	stp->session = NULL;
	pb->stp.available_stp_mask |= 1ULL << stp_index;
}

int release_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	int ret = 0;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret == 0) {
		release_stp(pb, session, stp);
		signal_completion_on_first_alloc_waiter(pb);
	}
	mutex_unlock(&pb->lock);

	return ret;
}

int stp_ctrl_set(struct paintbox_data *pb,
		 struct paintbox_session *session, unsigned int stp_id,
		 uint64_t set_mask)
{
	struct paintbox_stp *stp;
	unsigned long irq_flags;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);

	ctrl = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_CTRL);
	ctrl |= set_mask;
	stp_ctrl_write(pb, stp, ctrl);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int start_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	unsigned long irq_flags;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "stp%u start\n", stp_id);

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);

	ctrl = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_CTRL);
	ctrl |= STP_CTRL_ENA_MASK;

	stp_ctrl_write(pb, stp, ctrl);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int stop_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	unsigned long irq_flags;
	uint32_t ctrl;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "stp%u stop\n", stp->stp_id);

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);

	ctrl = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_CTRL);
	ctrl &= ~STP_CTRL_ENA_MASK;
	stp_ctrl_write(pb, stp, ctrl);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	mutex_unlock(&pb->lock);

	return ret;
}

int resume_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;

	dev_dbg(pb->dev, "stp%u resume\n", stp_id);
	/* Resume bit is self-clearing */
	return stp_ctrl_set(pb, session, stp_id, STP_CTRL_RESUME_MASK);
}

/* The caller to this function must hold pb->lock */
static int paintbox_stp_reset(struct paintbox_data *pb,
		struct paintbox_stp *stp)
{
	unsigned long irq_flags;
	uint64_t ctrl;

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);

	/* Stop and reset the processor */
	ctrl = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_CTRL);
	ctrl &= ~STP_CTRL_ENA_MASK;
	stp_ctrl_write(pb, stp, STP_CTRL_RESET_MASK);
	stp_ctrl_write(pb, stp, ctrl);

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	stp_pc_histogram_clear(stp, pb->stp.inst_mem_size_in_instructions);

	return 0;
}

/* The caller to this function must hold pb->lock */
int paintbox_stp_reset_all(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_stp *stp, *stp_next;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select_all(pb);
	paintbox_writel(pb->dev, STP_CTRL_RESET_MASK, IPU_CSR_STP_OFFSET +
			STP_CTRL);
	paintbox_writel(pb->dev, 0, IPU_CSR_STP_OFFSET + STP_CTRL);

	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		stp->enabled = false;
		stp_pc_histogram_clear(stp,
				pb->stp.inst_mem_size_in_instructions);
	}

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	return 0;
}

int reset_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	int ret;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "stp%u reset\n", stp_id);

	ret = paintbox_stp_reset(pb, stp);

	mutex_unlock(&pb->lock);

	return ret;
}

int paintbox_reset_all_stp_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_stp *stp, *stp_next;

	mutex_lock(&pb->lock);

	/* Fast path of broadcasting: broadcasting reset to all STPs when:
	 *	1) there is only 1 session exist; or
	 *	2) one of the sessions has all STPs
	 */
	if (pb->session_count == 1 || session->stp_count == pb->stp.num_stps) {
		int ret;

		ret = paintbox_stp_reset_all(pb, session);
		if (ret < 0) {
			mutex_unlock(&pb->lock);
			return ret;
		}
	} else {
		list_for_each_entry_safe(stp, stp_next, &session->stp_list,
				session_entry) {
			int ret = paintbox_stp_reset(pb, stp);

			if (ret < 0) {
				mutex_unlock(&pb->lock);
				return ret;
			}
		}
	}

	dev_dbg(pb->dev, "stp reset all\n");

	mutex_unlock(&pb->lock);

	return 0;
}

/* TODO:  This should be removed b/36069658 */
int setup_stp_ioctl(struct paintbox_data *pb, struct paintbox_session *session,
		unsigned long arg)
{
	struct stp_config __user *user_config;
	struct stp_config config;
	struct paintbox_sram_config sram_config;
	struct paintbox_stp *stp;
	size_t max_len_bytes;
	int ret;

	user_config = (struct stp_config __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, config.processor_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	max_len_bytes = pb->stp.inst_mem_size_in_instructions *
			STP_INST_SRAM_INSTRUCTION_WIDTH_BYTES;

	if (config.len > max_len_bytes) {
		dev_err(pb->dev,
				"%s: stp%u: program too large, %lu > %lu bytes\n",
				__func__, config.processor_id, config.len,
				max_len_bytes);
		mutex_unlock(&pb->lock);
		return -ERANGE;
	}

	dev_dbg(pb->dev, "stp%u setup program len %zu bytes\n", stp->stp_id,
			config.len);

	/* TODO:  The assembler generates the pISA in the byte order
	 * expected by the DV.  In order for the pISA to be used by the hardware
	 * the instruction buffer needs to be byte swapped.  This will
	 * eventually be fixed in the assembler.  b/30316979
	 */
	ret = create_scalar_sram_config(&sram_config, stp->stp_id,
			SRAM_TARGET_STP_INSTRUCTION_RAM, true /* swap data */,
			false /* pad to align */);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = paintbox_stp_reset(pb, stp);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = sram_write_user_buffer(pb, &sram_config, 0, config.buf,
			config.len);
	if (ret < 0)
		dev_err(pb->dev, "%s: stp%u: setup stp failed, ret = %d\n",
				__func__, config.processor_id, ret);

	mutex_unlock(&pb->lock);

	return ret;
}

int get_program_state_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct stp_program_state __user *user_program_state;
	struct stp_program_state program_state;
	struct paintbox_stp *stp;
	unsigned long irq_flags;
	uint64_t stat;
	int ret;

	user_program_state = (struct stp_program_state __user *)arg;
	if (copy_from_user(&program_state, user_program_state,
			sizeof(program_state)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, program_state.stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	paintbox_stp_select(pb, stp->stp_id);
	stat = paintbox_stp_stat_read(pb);

	program_state.enabled = stp->enabled;

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	program_state.stalled = (stat & STP_STAT_STALLED_MASK) != 0;
	program_state.program_counter =
			(stat & STP_STAT_PC_MASK) >> STP_STAT_PC_SHIFT;

	dev_dbg(pb->dev, "stp%u enabled %u stalled %u pc 0x%08x\n", stp->stp_id,
			program_state.enabled, program_state.stalled,
			program_state.program_counter);

	mutex_unlock(&pb->lock);

	if (copy_to_user(user_program_state, &program_state,
			sizeof(program_state)))
		return -EFAULT;

	return 0;
}

int paintbox_get_all_processor_states(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_all_stp_state __user *user_state;
	struct paintbox_all_stp_state state;
	struct paintbox_stp *stp, *stp_next;
	unsigned long irq_flags;
	uint64_t stat;

	user_state = (struct paintbox_all_stp_state __user *)arg;

	state.enabled = 0;
	state.stalled = 0;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->stp.lock, irq_flags);

	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry) {
		uint64_t stp_mask = 1ULL << stp->stp_id;

		paintbox_stp_select(pb, stp->stp_id);
		stat = paintbox_stp_stat_read(pb);

		if (stp->enabled)
			state.enabled |= stp_mask;

		if (stat & STP_STAT_STALLED_MASK)
			state.stalled |= stp_mask;
	}

	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

	dev_dbg(pb->dev, "stp get all states\n");

	mutex_unlock(&pb->lock);

	if (copy_to_user(user_state, &state, sizeof(state)))
		return -EFAULT;

	return 0;
}

static void paintbox_stp_imr_write(struct paintbox_data *pb,
		struct paintbox_stp *stp, uint32_t val)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->stp.lock, irq_flags);
	paintbox_stp_select(pb, stp->stp_id);
	paintbox_writel(pb->dev, val, IPU_CSR_STP_OFFSET + STP_IMR);
	spin_unlock_irqrestore(&pb->stp.lock, irq_flags);
}

static void paintbox_stp_enable_interrupts(struct paintbox_data *pb,
		struct paintbox_stp *stp)
{
	paintbox_stp_imr_write(pb, stp, STP_IMR_INT_MASK | STP_IMR_ERR_MASK);
}

static void paintbox_stp_disable_interrupts(struct paintbox_data *pb,
		struct paintbox_stp *stp)
{
	paintbox_stp_imr_write(pb, stp, 0);
}

int bind_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct stp_interrupt_config __user *user_req;
	struct stp_interrupt_config req;
	struct paintbox_stp *stp;
	int ret;

	user_req = (struct stp_interrupt_config __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, req.stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "stp%u bind irq%u\n", req.stp_id, req.interrupt_id);

	ret = bind_stp_interrupt(pb, session, stp, req.interrupt_id);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	paintbox_stp_enable_interrupts(pb, stp);
	paintbox_enable_stp_error_interrupt(pb, stp->stp_id);
	paintbox_enable_stp_interrupt(pb, stp->stp_id);

	mutex_unlock(&pb->lock);

	return ret;
}

int unbind_stp_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int stp_id = (unsigned int)arg;
	struct paintbox_stp *stp;
	int ret = 0;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	paintbox_stp_disable_interrupts(pb, stp);
	paintbox_disable_stp_error_interrupt(pb, stp->stp_id);
	paintbox_disable_stp_interrupt(pb, stp_id);

	ret = unbind_stp_interrupt(pb, session, stp);
	if (ret < 0) {
		dev_err(pb->dev, "%s: stp%u: unable to unbind interrupt, %d\n",
				__func__, stp_id, ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(pb->dev, "stp%u unbind irq\n", stp_id);

	mutex_unlock(&pb->lock);

	return 0;
}

/* Callers to this function must set the select register appropriately */
static void paintbox_stp_handle_interrupt(struct paintbox_data *pb,
		struct paintbox_stp *stp, ktime_t timestamp, uint32_t mask)
{
	uint32_t status;
	uint16_t lbp_id;
	uint16_t int_code;

	status = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_ISR);

	if (status & mask) {
		if (status & STP_ISR_INT_MASK)
			int_code = paintbox_readl(pb->dev,
					IPU_CSR_STP_OFFSET + STP_IRQ_LOG);

		if (status & STP_ISR_ERR_MASK) {
			lbp_id = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET +
					STP_ERR_LOG);
			dev_err(pb->dev,
					"%s: stp%d: error accessing lbp%hu\n",
					__func__, stp->stp_id, lbp_id);
		}

		paintbox_writel(pb->dev, status, IPU_CSR_STP_OFFSET +
				STP_ISR);
		stp->interrupt_count++;

		if (status & STP_ISR_INT_MASK)
			paintbox_irq_waiter_signal(pb, stp->irq, timestamp,
					int_code, 0 /* error */);

		/* TODO: signal error to irq waiter.  b/62351992 */
	}

	status = paintbox_readl(pb->dev, IPU_CSR_STP_OFFSET + STP_ISR_OVF);
	if (status & mask) {
		stp->interrupt_count++;
		paintbox_writel(pb->dev, status, IPU_CSR_STP_OFFSET +
				STP_ISR_OVF);
	}
}

static void paintbox_stp_handle_sw_interrupt(struct paintbox_data *pb,
		struct paintbox_stp *stp, ktime_t timestamp)
{
	paintbox_stp_handle_interrupt(pb, stp, timestamp, STP_ISR_INT_MASK);
}

irqreturn_t paintbox_stp_interrupt(struct paintbox_data *pb, uint64_t stp_mask,
		ktime_t timestamp)
{
	unsigned int stp_index;

	for (stp_index = 0; stp_index < pb->stp.num_stps && stp_mask;
			stp_index++, stp_mask >>= 1) {
		struct paintbox_stp *stp;

		if (!(stp_mask & 0x01))
			continue;

		stp = &pb->stp.stps[stp_index];

		spin_lock(&pb->stp.lock);

		paintbox_stp_select(pb, stp->stp_id);
		paintbox_stp_handle_sw_interrupt(pb, stp, timestamp);

		spin_unlock(&pb->stp.lock);
	}

	return IRQ_HANDLED;
}

static void paintbox_stp_handle_error_interrupt(struct paintbox_data *pb,
		struct paintbox_stp *stp, ktime_t timestamp)
{
	paintbox_stp_handle_interrupt(pb, stp, timestamp, STP_ISR_ERR_MASK);
}

irqreturn_t paintbox_stp_error_interrupt(struct paintbox_data *pb,
		uint64_t stp_mask, ktime_t timestamp)
{
	unsigned int stp_index;

	for (stp_index = 0; stp_index < pb->stp.num_stps && stp_mask;
			stp_index++, stp_mask >>= 1) {
		struct paintbox_stp *stp;

		if (!(stp_mask & 0x01))
			continue;

		stp = &pb->stp.stps[stp_index];

		spin_lock(&pb->stp.lock);

		paintbox_stp_select(pb, stp->stp_id);
		paintbox_stp_handle_error_interrupt(pb, stp, timestamp);

		spin_unlock(&pb->stp.lock);
	}

	return IRQ_HANDLED;
}

/* The caller to this function must hold pb->lock */
void paintbox_stp_release(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_stp *stp, *stp_next;

	list_for_each_entry_safe(stp, stp_next, &session->stp_list,
			session_entry)
		release_stp(pb, session, stp);
}

/* All sessions must be released before remove can be called. */
void paintbox_stp_remove(struct paintbox_data *pb)
{
	unsigned int stp_index;

	for (stp_index = 0; stp_index < pb->stp.num_stps; stp_index++) {
		struct paintbox_stp *stp = &pb->stp.stps[stp_index];

		paintbox_stp_debug_remove(pb, stp);

		kfree(stp->stalled);
	}

	kfree(pb->stp.stps);
}

/* Resets shadows and restores power state. */
void paintbox_stp_post_ipu_reset(struct paintbox_data *pb)
{
	unsigned int i;
	struct paintbox_stp *stp;

	pb->stp.selected_stp_id = STP_SEL_DEF & STP_SEL_STP_SEL_M;

	for (i = 0; i < pb->stp.num_stps; i++) {
		stp = &pb->stp.stps[i];
		stp->enabled = false;

#ifndef CONFIG_PAINTBOX_FPGA_SUPPORT
		if (stp->pm_enabled)
			paintbox_pm_stp_enable(pb, stp);
#endif

		/* TODO(showarth): restore lbp access */
	}
}

int paintbox_stp_init(struct paintbox_data *pb)
{
	unsigned int stp_index;

	spin_lock_init(&pb->stp.lock);

	pb->stp.stps = kcalloc(pb->stp.num_stps, sizeof(struct paintbox_stp),
			GFP_KERNEL);
	if (!pb->stp.stps)
		return -ENOMEM;

	pb->stp.selected_stp_id = STP_SEL_DEF & STP_SEL_STP_SEL_M;

	for (stp_index = 0; stp_index < pb->stp.num_stps; stp_index++) {
		struct paintbox_stp *stp = &pb->stp.stps[stp_index];

		stp->stp_id = stp_index_to_id(stp_index);

		paintbox_stp_debug_init(pb, stp);

		dev_dbg(pb->dev, "stp%u: base 0x%08x len %lu\n", stp->stp_id,
				IPU_CSR_STP_OFFSET, STP_BLOCK_LEN);
	}

	return 0;
}

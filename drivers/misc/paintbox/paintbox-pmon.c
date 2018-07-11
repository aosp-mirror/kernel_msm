/*
 * PMON support for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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
#include <linux/fs.h>

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-lbp.h"
#include "paintbox-pmon-regs.h"
#include "paintbox-stp.h"

/* returns true if |value| is in the range [lo, hi] */
static inline bool in_range(int64_t value, int64_t lo, int64_t hi)
{
	return lo <= value && value <= hi;
}

/* returns the number of counters for block type |block| */
static int pmon_num_counters_in_block(enum pmon_block_type block)
{
	/* TODO query from hardware */
	if (block == PMON_BLOCK_DMA)
		return DMA_PMON_COUNTERS;
	return DEFAULT_PMON_COUNTERS;
}

/* returns the index of the first core in |block| */
static int pmon_core_id_min(enum pmon_block_type block)
{
	return (block == PMON_BLOCK_STP) ? stp_index_to_id(0) : 0;
}

/* returns the index of the last core in |block| */
static int pmon_core_id_max(struct paintbox_data *pb,
		enum pmon_block_type block)
{
	switch (block) {
	case PMON_BLOCK_LBP:
		return pb->lbp.num_lbps - 1;
	case PMON_BLOCK_STP:
		return stp_index_to_id(pb->stp.num_stps - 1);
	default:
		return 0;
	}
}

/* this debug wrapper traces all pmon register writes */
static inline void pmon_writeq(struct paintbox_data *pb, uint64_t b,
		uint32_t addr)
{
	dev_dbg(pb->dev, "writeq 0x%08x=%16llx", addr, b);

	paintbox_writeq(pb->dev, b, addr);
}

/* this debug wrapper traces all pmon register reads */
static inline uint64_t pmon_readq(struct paintbox_data *pb, uint32_t addr)
{
	uint64_t b = paintbox_readq(pb->dev, addr);

	dev_dbg(pb->dev, "readq %16llx=0x%08x", b, addr);
	return b;
}

/* Returns true if the |session| owns the specified pmon counters.
 *
 * caller must hold pb->lock
 */
static inline bool pmon_check_session(struct paintbox_data *pb,
		enum pmon_block_type block, int core_id,
		struct paintbox_session *session)
{
	int stp_index;

	switch (block) {
	case PMON_BLOCK_BIF:
		return pb->bif.pmon_session == session;
	case PMON_BLOCK_MMU:
		return pb->mmu.pmon_session == session;
	case PMON_BLOCK_DMA:
		return pb->dma.pmon_session == session;
	case PMON_BLOCK_LBP:
		return pb->lbp.lbps[core_id].session == session;
	case PMON_BLOCK_STP:
		stp_index = stp_id_to_index(core_id);
		return pb->stp.stps[stp_index].session == session;
	}
	dev_err(pb->dev, "%s: invalid block (%i)", __func__, block);

	return false;
}

/* select & lock as necessary, based on block */
static int pmon_block_lock(struct paintbox_data *pb, enum pmon_block_type block,
		int core_id, struct paintbox_session *session,
		unsigned long *irq_flags)
{
	dev_dbg(pb->dev, "%s: lock block=%i core_id=%i\n", __func__, block,
			core_id);

	mutex_lock(&pb->lock);

	if (!pmon_check_session(pb, block, core_id, session)) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev,
				"%s: this pmon resource %i[%i] does not belong to this session",
				__func__, block, core_id);
		return -EACCES;
	}

	switch (block) {
	case PMON_BLOCK_STP:
		spin_lock_irqsave(&pb->stp.lock, *irq_flags);
		paintbox_stp_select(pb, core_id);
		break;
	case PMON_BLOCK_LBP:
		paintbox_lbp_select(pb, core_id);
		break;
	default:
		/* holding pb->lock is enough */
		break;
	}
	return 0;
}

/* unlock as necessary, based on block */
static inline void pmon_block_unlock(struct paintbox_data *pb,
		enum pmon_block_type block, unsigned long irq_flags)
{
	switch (block) {
	case PMON_BLOCK_STP:
		spin_unlock_irqrestore(&pb->stp.lock, irq_flags);
		break;
	default:
		break;
	}
	mutex_unlock(&pb->lock);
	dev_dbg(pb->dev, "%s: unlock block=%i\n", __func__, block);
}

/* returns the address of the enable register for |block| */
uint32_t pmon_reg_addr_cfg(struct paintbox_data *pb,
		enum pmon_block_type block)
{
	switch (block) {
	case PMON_BLOCK_BIF:
		return IPU_CSR_AXI_OFFSET + BIF_PMON_CFG;
	case PMON_BLOCK_MMU:
		return IPU_CSR_AXI_OFFSET + MMU_PMON_CFG;
	case PMON_BLOCK_DMA:
		return IPU_CSR_DMA_TOP_OFFSET + DMA_PMON_CFG;
	case PMON_BLOCK_LBP:
		return IPU_CSR_LBP_OFFSET + LBP_PMON_CFG;
	case PMON_BLOCK_STP:
		return IPU_CSR_STP_OFFSET + STP_PMON_CFG;
	}
	dev_err(pb->dev, "%s: pmon_block_type %i is out of bounds", __func__,
			block);
	return 0;
}

/* returns the address of the counter |counter_id| in |block| */
uint32_t pmon_reg_addr_counter(struct paintbox_data *pb,
		enum pmon_block_type block, int counter_id)
{
	uint32_t cfg;

	cfg = pmon_reg_addr_cfg(pb, block);
	if (!cfg)
		return 0;

	if (!in_range(counter_id, 0, pmon_num_counters_in_block(block) - 1)) {
		dev_err(pb->dev, "%s: counter_id %i is out of bounds", __func__,
				counter_id);
		return 0;
	}

	return cfg + PMON_CNT_0 + counter_id * PMON_CNT_STRIDE;
}

/* Returns true if all pmon_op fields are legal.  Otherwise prints error
 * and return false.
 */
static bool pmon_op_in_range(struct paintbox_data *pb, const struct pmon_op *op)
{
	if (!in_range(op->sel, 0, PMON_CNT_CFG_SEL_M)) {
		dev_err(pb->dev, "%s: pmon_op.sel %i is out of bounds",
				__func__, op->mask);
		return false;
	}
	if (!in_range(op->mask, 0, PMON_CNT_CFG_MATCH_M)) {
		dev_err(pb->dev, "%s: pmon_op.mask %i is out of bounds",
				__func__, op->mask);
		return false;
	}
	if (!in_range(op->match, 0, PMON_CNT_CFG_MASK_M)) {
		dev_err(pb->dev, "%s: pmon_op.match %i is out of bounds",
				__func__, op->match);
		return false;
	}

	return true;
}

/* Returns a pointer to the bif/mmu/dma pmon_session pointer
 *
 * Returns 0 for invalid/other blocks
 *
 * The pmon_session pointer points to the session that has allocated the pmon
 * counters for |block| or NULL if the |block| is not allocated.
 *
 * no locks are necessary to call this function
 */
struct paintbox_session **pmon_session_handle(struct paintbox_data *pb,
		enum pmon_block_type block)
{
	switch (block) {
	case PMON_BLOCK_BIF:
		return &pb->bif.pmon_session;
	case PMON_BLOCK_MMU:
		return &pb->mmu.pmon_session;
	case PMON_BLOCK_DMA:
		return &pb->dma.pmon_session;
	case PMON_BLOCK_LBP:
	case PMON_BLOCK_STP:
		dev_err(pb->dev,
				"%s: the pmon counters for this block (%i) are automatically allocated with the core",
				__func__, block);
		return NULL;
	}
	dev_err(pb->dev, "%s: invalid block (%i)", __func__, block);
	return NULL;
}

int pmon_allocate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	enum pmon_block_type block = (enum pmon_block_type)arg;
	struct paintbox_session **session_handle;

	dev_dbg(pb->dev, "%s: block=%i\n", __func__, block);

	mutex_lock(&pb->lock);

	session_handle = pmon_session_handle(pb, block);
	if (session_handle == NULL) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: can't allocate block %i", __func__,
				block);
		return -EINVAL;
	}

	if (*session_handle) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev,
				"%s: this block (%i) is already allocated",
				__func__, block);
		return -EACCES;
	}

	*session_handle = session;
	mutex_unlock(&pb->lock);

	return 0;
}

int pmon_release_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	enum pmon_block_type block = (enum pmon_block_type)arg;
	struct paintbox_session **session_handle;

	dev_dbg(pb->dev, "%s: block=%i\n", __func__, block);

	mutex_lock(&pb->lock);

	session_handle = pmon_session_handle(pb, block);
	if (session_handle == NULL) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev, "%s: can't allocate block %i\n", __func__,
				block);
		return -EINVAL;
	}

	if (*session_handle != session) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev,
				"%s: this block (%i) is not allocated to this session\n",
				__func__, block);
		return -EACCES;
	}

	*session_handle = NULL;
	mutex_unlock(&pb->lock);

	return 0;
}

int pmon_config_write_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct pmon_config __user *user_config =
			(struct pmon_config __user *)arg;
	uint64_t cfg = 0;
	uint32_t counter_addr;
	unsigned long irq_flags;
	int core_id_min;
	int core_id_max;
	int ret;

	struct pmon_config config;

	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	dev_dbg(pb->dev, "%s: block=%i core_id=%i counter_id=%i\n", __func__,
			config.block, config.core_id, config.counter_id);

	/* first, validate every config value */
	counter_addr = pmon_reg_addr_counter(pb, config.block,
			config.counter_id);
	if (!counter_addr)
		return -EINVAL;

	core_id_min = pmon_core_id_min(config.block);
	core_id_max = pmon_core_id_max(pb, config.block);
	if (!in_range(config.core_id, core_id_min, core_id_max)) {
		dev_err(pb->dev, "%s: core_id %i is out of bounds", __func__,
				config.core_id);
		return -EINVAL;
	}

	if (!in_range(config.mode, PMON_MODE_DISABLED,
			PMON_MODE_COUNT_INCREMENT)) {
		dev_err(pb->dev, "%s: mode %i is out of bounds", __func__,
				config.mode);
		return -EINVAL;
	}

	if (!in_range(config.threshold, 0, PMON_CNT_CFG_THRESHOLD_M)) {
		dev_err(pb->dev, "%s: threshold %i is out of bounds", __func__,
				config.threshold);
		return -EINVAL;
	}

	if (!pmon_op_in_range(pb, &config.inc)) {
		dev_err(pb->dev, "%s: inc is out of bounds", __func__);
		return -EINVAL;
	}

	if (!pmon_op_in_range(pb, &config.dec)) {
		dev_err(pb->dev, "%s: dec is out of bounds", __func__);
		return -EINVAL;
	}

	cfg |= ((uint64_t)config.mode) << PMON_CNT_CFG_MODE_SHIFT;
	cfg |= ((uint64_t)config.dec.inv) << PMON_CNT_CFG_DEC_INV_SHIFT;
	cfg |= ((uint64_t)config.dec.sel) << PMON_CNT_CFG_DEC_SEL_SHIFT;
	cfg |= ((uint64_t)config.dec.mask) << PMON_CNT_CFG_DEC_MASK_SHIFT;
	cfg |= ((uint64_t)config.dec.match) << PMON_CNT_CFG_DEC_MATCH_SHIFT;
	cfg |= ((uint64_t)config.inc.inv) << PMON_CNT_CFG_INC_INV_SHIFT;
	cfg |= ((uint64_t)config.inc.sel) << PMON_CNT_CFG_INC_SEL_SHIFT;
	cfg |= ((uint64_t)config.inc.mask) << PMON_CNT_CFG_INC_MASK_SHIFT;
	cfg |= ((uint64_t)config.inc.match) << PMON_CNT_CFG_INC_MATCH_SHIFT;
	cfg |= ((uint64_t)config.threshold) << PMON_CNT_CFG_THRESHOLD_SHIFT;

	ret = pmon_block_lock(pb, config.block, config.core_id, session,
			&irq_flags);
	if (ret < 0)
		return ret;

	pmon_writeq(pb, cfg, counter_addr + PMON_CNT_CFG);

	pmon_block_unlock(pb, config.block, irq_flags);

	return 0;
}

int pmon_data_read_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct pmon_data __user *user_data =
			(struct pmon_data __user *)arg;

	uint32_t counter_addr;

	unsigned long irq_flags;
	struct pmon_data data;
	uint64_t cnt;
	uint64_t sts_acc;
	uint64_t sts;
	int ret;

	if (copy_from_user(&data, user_data, sizeof(data)))
		return -EFAULT;

	dev_dbg(pb->dev, "%s: block=%i core_id=%i counter_id=%i\n", __func__,
			data.block, data.core_id, data.counter_id);

	counter_addr = pmon_reg_addr_counter(pb, data.block, data.counter_id);
	if (!counter_addr)
		return -EINVAL;

	ret = pmon_block_lock(pb, data.block, data.core_id, session,
			&irq_flags);
	if (ret < 0)
		return ret;

	cnt = pmon_readq(pb, counter_addr + PMON_CNT);
	sts_acc = pmon_readq(pb, counter_addr + PMON_CNT_STS_ACC);
	sts = pmon_readq(pb, counter_addr + PMON_CNT_STS);

	pmon_block_unlock(pb, data.block, irq_flags);

	data.count = cnt;
	data.accumulator = sts_acc;
	data.accumulator_overflow = (sts >> PMON_CNT_STS_ACC_OF_SHIFT) & 1;
	data.accumulator_underflow = (sts >> PMON_CNT_STS_ACC_UF_SHIFT) & 1;
	data.count_overflow = (sts >> PMON_CNT_STS_CNT_OF_SHIFT) & 1;

	dev_dbg(pb->dev, "%s: count=%llu acc=%i%s%s%s", __func__, data.count,
			data.accumulator,
			(data.count_overflow ? " (count_overflow)" : ""),
			(data.accumulator_overflow ? " (acc_overflow)" : ""),
			(data.accumulator_underflow ? " (acc_underflow)" : ""));

	if (copy_to_user(user_data, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

int pmon_data_write_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct pmon_data __user *user_data =
			(struct pmon_data __user *)arg;

	uint32_t counter_addr;

	unsigned long irq_flags;
	struct pmon_data data;
	uint64_t sts = 0;
	int ret;

	if (copy_from_user(&data, user_data, sizeof(data)))
		return -EFAULT;

	dev_dbg(pb->dev,
			"%s: block=%i core_id=%i counter_id=%i count=%llu, count_of=%i acc=%i, acc_of=%i acc_uf=%i\n",
			__func__, data.block, data.core_id, data.counter_id,
			data.count, data.count_overflow, data.accumulator,
			data.accumulator_overflow, data.accumulator_underflow);

	counter_addr = pmon_reg_addr_counter(pb, data.block,
			data.counter_id);
	if (!counter_addr)
		return -EINVAL;

	if (!in_range(data.count, 0, PMON_CNT_CNT_M)) {
		dev_err(pb->dev, "%s: count %llu is out of bounds", __func__,
				data.count);
		return -EINVAL;
	}

	if (!in_range(data.accumulator, 0, PMON_CNT_STS_ACC_M)) {
		dev_err(pb->dev, "%s: accumulator %i is out of bounds",
				__func__, data.accumulator);
		return -EINVAL;
	}

	sts |= ((uint64_t)data.accumulator_overflow)
			<< PMON_CNT_STS_ACC_OF_SHIFT;
	sts |= ((uint64_t)data.accumulator_underflow)
			<< PMON_CNT_STS_ACC_UF_SHIFT;
	sts |= ((uint64_t)data.count_overflow)
			<< PMON_CNT_STS_CNT_OF_SHIFT;

	ret = pmon_block_lock(pb, data.block, data.core_id, session,
			&irq_flags);
	if (ret < 0)
		return ret;

	pmon_writeq(pb, data.count, counter_addr + PMON_CNT);
	pmon_writeq(pb, data.accumulator, counter_addr + PMON_CNT_STS_ACC);
	pmon_writeq(pb, sts, counter_addr + PMON_CNT_STS);

	pmon_block_unlock(pb, data.block, irq_flags);

	return 0;
}

int pmon_enable_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct pmon_enable __user *user_enable =
			(struct pmon_enable __user *)arg;

	uint32_t enable_addr;

	unsigned long irq_flags;
	struct pmon_enable enable;
	uint64_t enable_reg = 0;
	int dma_channel_max = 0;
	int rptr_id_max = 0;
	int ret;

	if (copy_from_user(&enable, user_enable, sizeof(enable)))
		return -EFAULT;

	dev_dbg(pb->dev,
			"%s: block=%i core_id=%i enable=%i channel_id_0=%i channel_id_1=%i",
			__func__, enable.block, enable.core_id, enable.enable,
			enable.channel_id_0, enable.channel_id_1);

	enable_addr = pmon_reg_addr_cfg(pb, enable.block);
	if (!enable_addr)
		return -EINVAL;

	if (enable.block == PMON_BLOCK_DMA)
		dma_channel_max = pb->dma.num_channels - 1;

	if (!in_range(enable.channel_id_0, 0, dma_channel_max)) {
		dev_err(pb->dev, "%s: channel_id_0 %i is out of bounds",
				__func__, enable.channel_id_0);
		return -EINVAL;
	}

	if (!in_range(enable.channel_id_1, 0, dma_channel_max)) {
		dev_err(pb->dev, "%s: channel_id_1 %i is out of bounds",
				__func__, enable.channel_id_1);
		return -EINVAL;
	}

	if (enable.block == PMON_BLOCK_LBP)
		rptr_id_max = LBP_CTRL_PMON_RD_SEL_M;

	if (!in_range(enable.rptr_id, 0, rptr_id_max)) {
		dev_err(pb->dev, "%s: rptr_id %i is out of bounds",
				__func__, enable.rptr_id);
		return -EINVAL;
	}

	enable_reg |= ((uint64_t)enable.enable) << PMON_CFG_ENABLE_SHIFT;
	if (enable.enable) {
		enable_reg |= ((uint64_t)enable.channel_id_0)
				<< DMA_PMON_CFG_CNT_0_CHAN_SHIFT;
		enable_reg |= ((uint64_t)enable.channel_id_1)
				<< DMA_PMON_CFG_CNT_1_CHAN_SHIFT;
	}

	ret = pmon_block_lock(pb, enable.block, enable.core_id, session,
			&irq_flags);
	if (ret < 0)
		return ret;

	if (enable.block == PMON_BLOCK_LBP && enable.enable) {
		/* set the rptr_id */
		paintbox_lbp_set_pmon_rptr_id(pb, enable.rptr_id);
	}
	pmon_writeq(pb, enable_reg, enable_addr);

	pmon_block_unlock(pb, enable.block, irq_flags);

	return 0;
}

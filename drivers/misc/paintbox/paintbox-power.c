/*
 * Power management support for the Paintbox programmable IPU
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

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

#define IO_POWER_RAMP_TIME 10 /* us */

/* Delay to prevent in-rush current */
#define CORE_POWER_RAMP_TIME 10 /* us */

/* Delay for rams to wake up */
#define RAM_POWER_RAIL_RAMP_TIME 1 /* us */

/* Delay for system to stabilize before sending real traffic */
#define CORE_SYSTEM_STABLIZE_TIME 100 /* us */

#ifdef CONFIG_PAINTBOX_DEBUG
static uint64_t paintbox_pm_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	return paintbox_readq(pb->dev, IPU_CSR_AON_OFFSET +
			reg_entry->reg_offset);
}

static void paintbox_pm_reg_entry_write
		(struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	paintbox_writeq(pb->dev, val, IPU_CSR_AON_OFFSET +
			reg_entry->reg_offset);
}

static const char *io_pm_reg_names[IO_AON_NUM_REGS] = {
	REG_NAME_ENTRY(IPU_VERSION),
	REG_NAME_ENTRY(IPU_CHECKSUM),
	REG_NAME_ENTRY(IPU_CAP),
	REG_NAME_ENTRY(CLK_GATE_CONTROL_STP_IDLE_GATE_DIS),
	REG_NAME_ENTRY(CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS),
	REG_NAME_ENTRY(CLK_GATE_CONTROL),
	REG_NAME_ENTRY(IDLE_CLK_COUNT),
	REG_NAME_ENTRY(IPU_CORE_PAIRS_EN),
	REG_NAME_ENTRY(CORE_POWER_ON_N),
	REG_NAME_ENTRY(CORE_ISO_ON),
	REG_NAME_ENTRY(CORE_RAM_ON_N),
	REG_NAME_ENTRY(IPU_DMA_CHAN_EN),
	REG_NAME_ENTRY(IO_POWER_ON_N),
	REG_NAME_ENTRY(IO_ISO_ON),
	REG_NAME_ENTRY(IO_RAM_ON_N),
	REG_NAME_ENTRY(SOFT_RESET),
	REG_NAME_ENTRY(IPU_IO_SWITCHED_CLK_EN),
	REG_NAME_ENTRY(JQS_BOOT_ADDR),
	REG_NAME_ENTRY(JQS_CONTROL),
	REG_NAME_ENTRY(JQS_WATCHDOG_CMP_INIT),
	REG_NAME_ENTRY(JQS_CACHE_ENABLE),
	REG_NAME_ENTRY(JQS_CACHE_END_ADDR_MSB),
	REG_NAME_ENTRY(JQS_I_CACHE_CTRL),
	REG_NAME_ENTRY(JQS_D_CACHE_CTRL),
	REG_NAME_ENTRY(IPU_STATUS),
	REG_NAME_ENTRY(AON_SPARE)
};

static inline int paintbox_pm_dump_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = io_pm_reg_names[REG_INDEX(reg_offset)];

	return dump_ipu_register(pb, IPU_CSR_AON_OFFSET, reg_offset, reg_name,
			buf, written, len);
}

int paintbox_pm_dump_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;

	ret = paintbox_pm_dump_reg(pb, IPU_VERSION, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IPU_CHECKSUM, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IPU_CAP, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, CLK_GATE_CONTROL_STP_IDLE_GATE_DIS, buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS, buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, CLK_GATE_CONTROL, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IDLE_CLK_COUNT, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IPU_CORE_PAIRS_EN, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IPU_DMA_CHAN_EN, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, CORE_POWER_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, CORE_ISO_ON, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, CORE_RAM_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IO_POWER_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IO_ISO_ON, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IO_RAM_ON_N, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, SOFT_RESET, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IPU_IO_SWITCHED_CLK_EN, buf, &written,
			len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, JQS_BOOT_ADDR, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, JQS_CONTROL, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, JQS_WATCHDOG_CMP_INIT, buf, &written,
			len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, JQS_CACHE_ENABLE, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, JQS_CACHE_END_ADDR_MSB, buf, &written,
			len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, JQS_I_CACHE_CTRL, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, JQS_D_CACHE_CTRL, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = paintbox_pm_dump_reg(pb, IPU_STATUS, buf, &written, len);
	if (ret < 0)
		goto err_exit;

	return written;

err_exit:
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}
#endif

/* The caller to this function must hold pb->dma.dma_lock */
void paintbox_pm_enable_dma_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (channel->pm_enabled)
		return;

	channel->pm_enabled = true;

	pb->io.regs.dma_chan_en |= 1 << channel->channel_id;
	paintbox_writel(pb->dev, pb->io.regs.dma_chan_en,
			IPU_CSR_AON_OFFSET + IPU_DMA_CHAN_EN);
}

/* The caller to this function must hold pb->dma.dma_lock */
void paintbox_pm_disable_dma_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (!channel->pm_enabled)
		return;

	channel->pm_enabled = false;

	pb->io.regs.dma_chan_en &= ~(1 << channel->channel_id);
	paintbox_writel(pb->dev, pb->io.regs.dma_chan_en,
			IPU_CSR_AON_OFFSET + IPU_DMA_CHAN_EN);
}

void paintbox_enable_mmu_bif_idle_clock_gating(struct paintbox_data *pb)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->power.power_lock, irq_flags);

	if (WARN_ON(--pb->power.bif_mmu_clock_idle_disable_ref_count < 0))
		pb->power.bif_mmu_clock_idle_disable_ref_count = 0;

	if (pb->power.bif_mmu_clock_idle_disable_ref_count == 0) {
		uint32_t val;

		/* TODO(ahampson):  Determine if controlling the SSP idle is
		 * necessary.
		 */
		val = paintbox_readl(pb->dev, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
		val &= ~(CLK_GATE_CONTROL_MMU_IDLE_GATE_DIS_MASK |
				CLK_GATE_CONTROL_BIF_IDLE_GATE_DIS_MASK |
				CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK);
		paintbox_writel(pb->dev, val, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
	}

	spin_unlock_irqrestore(&pb->power.power_lock, irq_flags);
}

void paintbox_disable_mmu_bif_idle_clock_gating(struct paintbox_data *pb)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->power.power_lock, irq_flags);

	if (pb->power.bif_mmu_clock_idle_disable_ref_count == 0) {
		uint32_t val;

		/* TODO(ahampson):  Determine if controlling the SSP idle is
		 * necessary.
		 */
		val = paintbox_readl(pb->dev, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
		val |= CLK_GATE_CONTROL_MMU_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_BIF_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK;
		paintbox_writel(pb->dev, val, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
	}

	pb->power.bif_mmu_clock_idle_disable_ref_count++;

	spin_unlock_irqrestore(&pb->power.power_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
static void paintbox_core_power_enable(struct paintbox_data *pb,
		unsigned int requested_cores)
{
	uint32_t max_core_mask, active_core_mask;
	unsigned int active_cores;
	uint32_t reg;

	/* If the active core count is already at our requested core count then
	 * there is nothing to do.
	 */
	if (requested_cores <= pb->power.active_core_count)
		return;

	/* The maximum number of cores is equal to the number of STPs. */
	max_core_mask = (1 << pb->stp.num_stps) - 1;

	active_core_mask = (1 << requested_cores) - 1;

	/* Disable STP idle clock gating */
	paintbox_writel(pb->dev, active_core_mask, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);

	/* Disable LBP idle clock gating */
	paintbox_writel(pb->dev, (active_core_mask << 1) | 0x1,
			IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);

	paintbox_disable_mmu_bif_idle_clock_gating(pb);

	/* IPU cores need to be enabled in sequence in pairs */
	for (active_cores = pb->power.active_core_count;
			active_cores < requested_cores; active_cores += 2) {
		uint32_t new_core_mask_n = (max_core_mask <<
				(active_cores + 1)) & max_core_mask;
		uint32_t new_core_pairs = (active_cores + 1) / 2;

		/* Power on the odd core first.  The first write clear the PRE
		 * bits for the cores to be powered.  The second write clears
		 * the MAIN bits for the cores
		 */
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N + 4);
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		udelay(CORE_POWER_RAMP_TIME);

		new_core_mask_n = (max_core_mask << (active_cores + 2)) &
				max_core_mask;

		/* Power on the even core next.  The first write clear the PRE
		 * bits for the cores to be powered.  The second write clears
		 * the MAIN bits for the cores
		 */
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N + 4);
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		udelay(CORE_POWER_RAMP_TIME);

		/* We need to run the clock to the core pair that's being
		 * powered on briefly so that all the synchronizers clock
		 * through their data and all the Xs (or random values in the
		 * real HW) clear. Then we need to turn the clock back off so
		 * that we can meet timing on the RAM SD pin -- the setup & hold
		 * on the RAM's SD pin is significantly longer that 1 clock
		 * cycle.
		 */
		paintbox_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn clocks off on all active cores */
		paintbox_writel(pb->dev, 0, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn on RAMs for the core */
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_RAM_ON_N);
		udelay(RAM_POWER_RAIL_RAMP_TIME);

		/* Restore clocks to all active core pairs. */
		paintbox_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Disable core isolation for the requested core */
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_ISO_ON);
	}

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	paintbox_enable_mmu_bif_idle_clock_gating(pb);

	/* Enable STP idle clock gating */
	reg = paintbox_readl(pb->dev, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);
	reg &= ~active_core_mask;
	paintbox_writel(pb->dev, reg, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);

	/* Enable LBP idle clock gating */
	reg = paintbox_readl(pb->dev, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);
	reg &= ~((active_core_mask << 1) | 0x1);
	paintbox_writel(pb->dev, reg, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);

	pb->power.active_core_count = requested_cores;
}

/* The caller to this function must hold pb->lock */
static void paintbox_core_power_disable(struct paintbox_data *pb,
		unsigned int requested_cores)
{
	uint32_t max_core_mask;

	/* If the active core count is already at our requested core count then
	 * there is nothing to do.
	 */
	if (requested_cores >= pb->power.active_core_count)
		return;

	/* The maximum number of cores is equal to the number of STPs. */
	max_core_mask = (1 << pb->stp.num_stps) - 1;

	paintbox_disable_mmu_bif_idle_clock_gating(pb);

	do {
		int new_active_cores = pb->power.active_core_count - 2;
		uint32_t new_core_mask_n = (max_core_mask << new_active_cores) &
				max_core_mask;
		uint32_t new_core_pairs = (new_active_cores + 1) / 2;

		/* Enable core isolation for the disabled cores */
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_ISO_ON);

		/* Turn off clocks to all cores during the RAM power transition.
		 */
		paintbox_writel(pb->dev, 0, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn off RAMs for the disabled core pairs */
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_RAM_ON_N);

		/* Need to briefly turn on the clocks to the cores being turned
		 * off to propagate the RAM SD pin change into the RAM, then
		 * need to turn the clocks off again, since the cores are being
		 * turned off.
		 */
		paintbox_writel(pb->dev, new_core_pairs + 1,
				IPU_CSR_AON_OFFSET + IPU_CORE_PAIRS_EN);

		/* Turn off clocks to the disabled core pairs. */
		paintbox_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn off the core pair */
		paintbox_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		pb->power.active_core_count = new_active_cores;
	} while (pb->power.active_core_count > requested_cores);

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	paintbox_enable_mmu_bif_idle_clock_gating(pb);
}

/* The caller to this function must hold pb->lock */
void paintbox_pm_stp_enable(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	stp->pm_enabled = true;

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 */
	paintbox_core_power_enable(pb, (stp->stp_id + 1) & ~1);
}

/* The caller to this function must hold pb->lock */
void paintbox_pm_lbp_enable(struct paintbox_data *pb, struct paintbox_lbp *lbp)
{
	lbp->pm_enabled = true;

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 *
	 * LBP0 and LBP15 are part of the IO block and are controlled
	 * separately.
	 */
	if (lbp->pool_id > 0 && lbp->pool_id < 15)
		paintbox_core_power_enable(pb, (lbp->pool_id + 1) & ~1);
}

/* The caller to this function must hold pb->lock */
static void paintbox_core_power_down_walk(struct paintbox_data *pb)
{
	unsigned int requested_cores;

	/* Walk backwards starting from the active core count until we find an
	 * enabled STP or LBP.  This is the new requested core count.
	 */
	for (requested_cores = pb->power.active_core_count;
			requested_cores > 0; requested_cores--) {
		struct paintbox_stp *stp =
				&pb->stp.stps[stp_id_to_index(requested_cores)];
		struct paintbox_lbp *lbp = &pb->lbp.lbps[requested_cores];

		if (stp->pm_enabled || lbp->pm_enabled)
			break;
	}

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 */
	paintbox_core_power_disable(pb, (requested_cores + 1) & ~1);
}

/* The caller to this function must hold pb->lock */
void paintbox_pm_stp_disable(struct paintbox_data *pb, struct paintbox_stp *stp)
{
	stp->pm_enabled = false;

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->power.stay_on)
		return;
#endif

	paintbox_core_power_down_walk(pb);
}

/* The caller to this function must hold pb->lock */
void paintbox_pm_lbp_disable(struct paintbox_data *pb, struct paintbox_lbp *lbp)
{
	lbp->pm_enabled = false;

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->power.stay_on)
		return;
#endif

	/* LBP0 and LBP15 are part of the IO block and are controlled
	 * separately.
	 */
	if (lbp->pool_id > 0 && lbp->pool_id < 15)
		paintbox_core_power_down_walk(pb);
}

#ifdef CONFIG_PAINTBOX_DEBUG
static int paintbox_power_stay_on_show(struct seq_file *s, void *p)
{
	struct paintbox_data *pb = s->private;

	seq_printf(s, "%d\n", pb->power.stay_on);
	return 0;
}

static int paintbox_power_stay_on_open(struct inode *inode, struct file *file)
{
	return single_open(file, paintbox_power_stay_on_show, inode->i_private);
}

static ssize_t paintbox_power_stay_on_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_data *pb = s->private;
	int ret, val;

	ret = kstrtoint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		mutex_lock(&pb->lock);

		if (pb->power.stay_on && val == 0)
			paintbox_core_power_down_walk(pb);

		pb->power.stay_on = !!val;

		mutex_unlock(&pb->lock);

		return count;
	}

	dev_err(pb->dev, "%s: invalid value, err = %d", __func__, ret);
	return ret < 0 ? ret : count;
}

static const struct file_operations stay_on_fops = {
	.open = paintbox_power_stay_on_open,
	.write = paintbox_power_stay_on_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};
#endif

int paintbox_pm_init(struct paintbox_data *pb)
{
#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_debug_create_entry(pb, &pb->power.debug,
			pb->debug_root, "power", -1,
			paintbox_pm_dump_registers, NULL, &pb->power);

	paintbox_debug_create_reg_entries(pb, &pb->power.debug, io_pm_reg_names,
			IO_AON_NUM_REGS, paintbox_pm_reg_entry_write,
			paintbox_pm_reg_entry_read);

	pb->power.stay_on_dentry = debugfs_create_file("stay_on", 0640,
			pb->power.debug.debug_dir, pb, &stay_on_fops);
	if (IS_ERR(pb->power.stay_on_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(pb->power.stay_on_dentry));
		return PTR_ERR(pb->power.stay_on_dentry);
	}
#endif

	spin_lock_init(&pb->power.power_lock);

	/* TODO:  Add support for debugfs entry that allows a user to
	 * force a certain number of active cores. b/62352592
	 */

	dev_dbg(pb->dev, "io_aon: base 0x%08x len %lu\n", IPU_CSR_AON_OFFSET,
			IO_AON_BLOCK_LEN);

	return 0;
}

void paintbox_pm_remove(struct paintbox_data *pb)
{
#ifdef CONFIG_PAINTBOX_DEBUG
	debugfs_remove(pb->power.stay_on_dentry);
	paintbox_debug_free_reg_entries(&pb->power.debug);
	paintbox_debug_free_entry(&pb->power.debug);
#endif
}

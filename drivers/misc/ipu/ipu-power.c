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
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-power.h"
#include "ipu-regs.h"
#include "ipu-stp.h"

#define IO_POWER_RAMP_TIME 10 /* us */

/* Delay to prevent in-rush current */
#define CORE_POWER_RAMP_TIME 10 /* us */

/* Delay for rams to wake up */
#define RAM_POWER_RAIL_RAMP_TIME 1 /* us */

/* Delay for system to stabilize before sending real traffic */
#define CORE_SYSTEM_STABLIZE_TIME 100 /* us */

void ipu_power_enable_mmu_bif_idle_clock_gating(struct paintbox_data *pb)
{
	if (WARN_ON(--pb->power.bif_mmu_clock_idle_disable_ref_count < 0))
		pb->power.bif_mmu_clock_idle_disable_ref_count = 0;

	if (pb->power.bif_mmu_clock_idle_disable_ref_count == 0) {
		uint32_t val;

		/* TODO(b/115407902):  Determine if controlling the SSP idle is
		 * necessary.
		 */
		val = ipu_readl(pb->dev, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
		val &= ~(CLK_GATE_CONTROL_MMU_IDLE_GATE_DIS_MASK |
				CLK_GATE_CONTROL_BIF_IDLE_GATE_DIS_MASK |
				CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK);
		ipu_writel(pb->dev, val, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
	}
}

void ipu_power_disable_mmu_bif_idle_clock_gating(struct paintbox_data *pb)
{
	if (pb->power.bif_mmu_clock_idle_disable_ref_count == 0) {
		uint32_t val;

		/* TODO(b/115407902):  Determine if controlling the SSP idle is
		 * necessary.
		 */
		val = ipu_readl(pb->dev, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
		val |= CLK_GATE_CONTROL_MMU_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_BIF_IDLE_GATE_DIS_MASK |
			CLK_GATE_CONTROL_SSP_IDLE_GATE_DIS_MASK;
		ipu_writel(pb->dev, val, IPU_CSR_AON_OFFSET +
				CLK_GATE_CONTROL);
	}

	pb->power.bif_mmu_clock_idle_disable_ref_count++;

}

/* The caller to this function must hold pb->lock */
void ipu_power_enable_cores(struct paintbox_data *pb,
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
	ipu_writel(pb->dev, active_core_mask, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);

	/* Disable LBP idle clock gating */
	ipu_writel(pb->dev, (active_core_mask << 1) | 0x1,
			IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);

	ipu_power_disable_mmu_bif_idle_clock_gating(pb);

	/* IPU cores need to be enabled in sequence in pairs */
	for (active_cores = pb->power.active_core_count;
			active_cores < requested_cores; active_cores += 2) {
		uint32_t new_core_mask_n = (max_core_mask <<
				(active_cores + 1)) & max_core_mask;
		uint32_t new_core_pairs = ((active_cores + 1) / 2) + 1;

		/* Power on the odd core first.  The first write clear the PRE
		 * bits for the cores to be powered.  The second write clears
		 * the MAIN bits for the cores
		 */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N + 4);
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		udelay(CORE_POWER_RAMP_TIME);

		new_core_mask_n = (max_core_mask << (active_cores + 2)) &
				max_core_mask;

		/* Power on the even core next.  The first write clear the PRE
		 * bits for the cores to be powered.  The second write clears
		 * the MAIN bits for the cores
		 */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N + 4);
		ipu_writel(pb->dev, new_core_mask_n,
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
		ipu_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn clocks off on all active cores */
		ipu_writel(pb->dev, 0, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn on RAMs for the core */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_RAM_ON_N);
		udelay(RAM_POWER_RAIL_RAMP_TIME);

		/* Restore clocks to all active core pairs. */
		ipu_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Disable core isolation for the requested core */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_ISO_ON);
	}

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	ipu_power_enable_mmu_bif_idle_clock_gating(pb);

	/* Enable STP idle clock gating */
	reg = ipu_readl(pb->dev, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);
	reg &= ~active_core_mask;
	ipu_writel(pb->dev, reg, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_STP_IDLE_GATE_DIS);

	/* Enable LBP idle clock gating */
	reg = ipu_readl(pb->dev, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);
	reg &= ~((active_core_mask << 1) | 0x1);
	ipu_writel(pb->dev, reg, IPU_CSR_AON_OFFSET +
			CLK_GATE_CONTROL_LBP_IDLE_GATE_DIS);

	pb->power.active_core_count = requested_cores;
}

/* The caller to this function must hold pb->lock */
void ipu_power_disable_cores(struct paintbox_data *pb,
		unsigned int requested_cores)
{
	uint32_t max_core_mask;

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	if (requested_cores < pb->power.min_active_core_count)
		requested_cores = pb->power.min_active_core_count;
#endif

	/* If the active core count is already at our requested core count then
	 * there is nothing to do.
	 */
	if (requested_cores >= pb->power.active_core_count)
		return;

	/* The maximum number of cores is equal to the number of STPs. */
	max_core_mask = (1 << pb->stp.num_stps) - 1;

	ipu_power_disable_mmu_bif_idle_clock_gating(pb);

	do {
		int new_active_cores = pb->power.active_core_count - 2;
		uint32_t new_core_mask_n = (max_core_mask << new_active_cores) &
				max_core_mask;
		uint32_t new_core_pairs = (new_active_cores + 1) / 2;

		/* Enable core isolation for the disabled cores */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_ISO_ON);

		/* Turn off clocks to all cores during the RAM power transition.
		 */
		ipu_writel(pb->dev, 0, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn off RAMs for the disabled core pairs */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_RAM_ON_N);

		/* Need to briefly turn on the clocks to the cores being turned
		 * off to propagate the RAM SD pin change into the RAM, then
		 * need to turn the clocks off again, since the cores are being
		 * turned off.
		 */
		ipu_writel(pb->dev, new_core_pairs + 1,
				IPU_CSR_AON_OFFSET + IPU_CORE_PAIRS_EN);

		/* Turn off clocks to the disabled core pairs. */
		ipu_writel(pb->dev, new_core_pairs, IPU_CSR_AON_OFFSET +
				IPU_CORE_PAIRS_EN);

		/* Turn off the core pair */
		ipu_writel(pb->dev, new_core_mask_n,
				IPU_CSR_AON_OFFSET + CORE_POWER_ON_N);

		pb->power.active_core_count = new_active_cores;
	} while (pb->power.active_core_count > requested_cores);

	udelay(CORE_SYSTEM_STABLIZE_TIME);

	ipu_power_enable_mmu_bif_idle_clock_gating(pb);
}

/* The caller to this function must hold pb->lock */
void ipu_power_core_power_walk_down(struct paintbox_data *pb)
{
	unsigned int requested_cores;

	/* Walk backwards starting from the active core count until we find an
	 * enabled STP or LBP.  This is the new requested core count.
	 */
	for (requested_cores = pb->power.active_core_count;
			requested_cores > 0; requested_cores--) {
		struct paintbox_stp *stp = &pb->stp.stps[ipu_stp_id_to_index(
				requested_cores)];
		struct paintbox_lbp *lbp = &pb->lbp.lbps[requested_cores];

		if (stp->pm_enabled || lbp->pm_enabled)
			break;
	}

	/* IPU cores are power controlled in pairs so round up the requested
	 * cores if it is an odd numbered core.
	 */
	ipu_power_disable_cores(pb, (requested_cores + 1) & ~1);
}

void ipu_power_init(struct paintbox_data *pb)
{
	pb->power.regs.dma_chan_en = IPU_DMA_CHAN_EN_DEF;
}

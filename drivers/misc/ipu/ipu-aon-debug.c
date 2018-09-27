/*
 * AON debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2018 Google, Inc.
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
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#include "ipu-aon-debug.h"
#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-power.h"
#include "ipu-regs.h"

/* The caller to this function must hold pb->lock */
static uint64_t ipu_aon_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	return ipu_readq(pb->dev, IPU_CSR_AON_OFFSET + reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static void ipu_aon_reg_entry_write(struct paintbox_debug_reg_entry *reg_entry,
		uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	ipu_writeq(pb->dev, val, IPU_CSR_AON_OFFSET + reg_entry->reg_offset);
}

static const char *ipu_aon_reg_names[IO_AON_NUM_REGS] = {
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

static inline int ipu_aon_dump_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = ipu_aon_reg_names[REG_INDEX(reg_offset)];

	return ipu_debug_dump_register(pb, IPU_CSR_AON_OFFSET, reg_offset,
			reg_name, buf, written, len);
}

/* The caller to this function must hold pb->lock */
int ipu_aon_dump_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;
	int ret, written = 0;
	unsigned int i;

	for (i = 0; i < IO_AON_NUM_REGS; i++) {
		if (ipu_aon_reg_names[i] == NULL)
			continue;

		ret = ipu_aon_dump_reg(pb, i * IPU_REG_WIDTH, buf, &written,
				len);
		if (ret < 0) {
			dev_err(pb->dev, "%s: register dump error, err = %d",
					__func__, ret);
			return ret;
		}
	}

	return written;
}

static int ipu_aon_debug_min_core_enable_set(void *data, u64 val)
{
	struct paintbox_data *pb = data;

	mutex_lock(&pb->lock);

	/* The maximum number of cores is equal to the number of STPs. */
	if (val > pb->stp.num_stps)
		val = pb->stp.num_stps;

	pb->power.min_active_core_count = val;

	if (val > pb->power.active_core_count)
		ipu_power_enable_cores(pb, val);
	else if (val < pb->power.active_core_count)
		ipu_power_disable_cores(pb, val);

	mutex_unlock(&pb->lock);

	return 0;
}

static int ipu_aon_debug_min_core_enable_get(void *data, u64 *val)
{
	struct paintbox_data *pb = data;

	mutex_lock(&pb->lock);

	*val = pb->power.min_active_core_count;

	mutex_unlock(&pb->lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ion_aon_debug_min_core_enable_fops,
		ipu_aon_debug_min_core_enable_get,
		ipu_aon_debug_min_core_enable_set, "%llu\n");

void ipu_aon_debug_init(struct paintbox_data *pb)
{
	ipu_debug_create_entry(pb, &pb->aon_debug, pb->debug_root, "aon", -1,
			ipu_aon_dump_registers, NULL, pb);

	ipu_debug_create_reg_entries(pb, &pb->aon_debug, ipu_aon_reg_names,
			IO_AON_NUM_REGS, ipu_aon_reg_entry_write,
			ipu_aon_reg_entry_read);

	pb->power.min_core_enable_dentry =
			debugfs_create_file("min_core_enable", 0640,
			pb->aon_debug.debug_dir, pb,
			&ion_aon_debug_min_core_enable_fops);
	if (IS_ERR(pb->power.min_core_enable_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(pb->power.min_core_enable_dentry));
	}
}

void ipu_aon_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove(pb->power.min_core_enable_dentry);
	ipu_debug_free_reg_entries(&pb->aon_debug);
	ipu_debug_free_entry(&pb->aon_debug);
}

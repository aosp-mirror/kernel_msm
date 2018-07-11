/*
 * JQS management support for the Paintbox programmable IPU
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

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/types.h>

#include "paintbox-bus.h"
#include "paintbox-bus-common.h"
#include "paintbox-bus-jqs.h"
#include "paintbox-regs.h"

#define JQS_FIRMWARE_NAME "paintbox-jqs.fw"

/*
 * The firmware assumes space immediately after the code to put globals and the
 * execution stack.
 *
 * TODO(b/78233088) 64k is overkill by a lot, but until we enforce some limits
 * in the linker script at third_party/riscv_tools/firmware/ref/link.common.ld
 * this should suffice.
 */
#define JQS_FIRMWARE_WORKING_SPACE 0x10000

/* Delay for I/O block to wake up */
#define IO_POWER_RAMP_TIME 10 /* us */

/* Delay to prevent in-rush current */
#define CORE_POWER_RAMP_TIME 10 /* us */

/* Delay for rams to wake up */
#define RAM_POWER_RAIL_RAMP_TIME 1 /* us */

/* Delay for system to stabilize before sending real traffic */
#define CORE_SYSTEM_STABLIZE_TIME 100 /* us */

static void paintbox_bus_jqs_start_firmware(struct paintbox_bus *bus,
		dma_addr_t boot_addr)
{
	/* The Airbrush IPU needs to be put in reset before turning on the
	 * I/O block.
	 */
	paintbox_bus_writel(bus, SOFT_RESET_IPU_MASK, IPU_CSR_AON_OFFSET +
			SOFT_RESET);

	paintbox_bus_writel(bus, JQS_CACHE_ENABLE_I_CACHE_MASK |
			JQS_CACHE_ENABLE_D_CACHE_MASK,
			IPU_CSR_AON_OFFSET + JQS_CACHE_ENABLE);

	paintbox_bus_writel(bus, (uint32_t)boot_addr, IPU_CSR_AON_OFFSET +
			JQS_BOOT_ADDR);

	/* Pre-power the I/O block and then enable power */
	paintbox_bus_writeq(bus, IO_POWER_ON_N_MAIN_MASK, IPU_CSR_AON_OFFSET +
			IO_POWER_ON_N);
	paintbox_bus_writeq(bus, 0, IPU_CSR_AON_OFFSET + IO_POWER_ON_N);

	udelay(IO_POWER_RAMP_TIME);

	/* We need to run the clock to the I/O block while it is being powered
	 * on briefly so that all the synchronizers clock through their data and
	 * all the Xs (or random values in the real HW) clear. Then we need to
	 * turn the clock back off so that we can meet timing on the RAM SD pin
	 * -- the setup & hold on the RAM's SD pin is significantly longer than
	 * 1 clock cycle.
	 */
	paintbox_bus_writel(bus, IPU_IO_SWITCHED_CLK_EN_VAL_MASK,
			IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);
	paintbox_bus_writel(bus, 0, IPU_CSR_AON_OFFSET +
			IPU_IO_SWITCHED_CLK_EN);

	/* Power on RAMs for I/O block */
	paintbox_bus_writel(bus, 0, IPU_CSR_AON_OFFSET + IO_RAM_ON_N);
	udelay(RAM_POWER_RAIL_RAMP_TIME);

	/* Turn on clocks to I/O block */
	paintbox_bus_writel(bus, IPU_IO_SWITCHED_CLK_EN_VAL_MASK,
			IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);

	/* Turn off isolation for I/O block */
	paintbox_bus_writel(bus, 0, IPU_CSR_AON_OFFSET + IO_ISO_ON);

	/* Take the IPU out of reset. */
	paintbox_bus_writel(bus, 0, IPU_CSR_AON_OFFSET + SOFT_RESET);

	/* Enable the JQS */
	paintbox_bus_writel(bus, JQS_CONTROL_CORE_FETCH_EN_MASK,
			IPU_CSR_AON_OFFSET + JQS_CONTROL);

	/* Notify paintbox devices that the firmware is up */
	paintbox_bus_notify_firmware_up(bus);
}

static void paintbox_bus_jqs_start_rom_firmware(struct paintbox_bus *bus)
{
	dev_dbg(bus->parent_dev, "enabling ROM firmware\n");
	paintbox_bus_jqs_start_firmware(bus, JQS_BOOT_ADDR_DEF);
	bus->fw_status = JQS_FW_STATUS_ROM_RUNNING;
}

static void paintbox_bus_release_resources(struct paintbox_bus *bus)
{
	bus->ops->free(bus->parent_dev, &bus->fw_shared_buffer);
	if (bus->fw) {
		release_firmware(bus->fw);
		bus->fw = NULL;
	}
}

int paintbox_bus_jqs_enable_firmware(struct paintbox_bus *bus)
{
	/* Firmware status will be set to INIT at boot or if Airbursh has been
	 * turned off while running the ROM based firmware.  RAM based firmware
	 * only needs to be requested once at boot.
	 */
	if (bus->fw_status == JQS_FW_STATUS_INIT) {
		dev_dbg(bus->parent_dev, "requesting firmware %s\n",
				JQS_FIRMWARE_NAME);
		if (request_firmware_direct(&bus->fw, JQS_FIRMWARE_NAME,
				bus->parent_dev) < 0) {
			goto reset_to_rom;
		}

		bus->fw_status = JQS_FW_STATUS_RAM_REQUESTED;
	}

	/* If the firmware is in the requested state then stage it to DRAM.
	 * Firmware status will return this state whenever Airbrush transitions
	 * to the OFF state.
	 */
	if (bus->fw_status == JQS_FW_STATUS_RAM_REQUESTED) {
		bus->ops->alloc(bus->parent_dev, bus->fw->size +
				JQS_FIRMWARE_WORKING_SPACE,
				&bus->fw_shared_buffer);
		if (!bus->fw_shared_buffer.host_vaddr)
			goto reset_to_rom;

		memcpy(bus->fw_shared_buffer.host_vaddr, bus->fw->data,
			bus->fw->size);
		bus->ops->sync(bus->parent_dev, &bus->fw_shared_buffer, 0,
			bus->fw->size, DMA_TO_DEVICE);

		dev_dbg(bus->parent_dev, "firmware staged at 0x%pad\n",
				&bus->fw_shared_buffer.jqs_paddr);

		bus->fw_status = JQS_FW_STATUS_RAM_STAGED;
	}

	/* If the firmware has been staged then enable the firmware.  Firmware
	 * status will return to this state for all suspend and sleep states
	 * with the exception of OFF.
	 */
	if (bus->fw_status == JQS_FW_STATUS_RAM_STAGED) {
		dev_dbg(bus->parent_dev, "starting firmware at 0x%pad\n",
				&bus->fw_shared_buffer.jqs_paddr);

		paintbox_bus_jqs_start_firmware(bus,
			bus->fw_shared_buffer.jqs_paddr);
		bus->fw_status = JQS_FW_STATUS_RAM_RUNNING;
	}

	return 0;

reset_to_rom:
	paintbox_bus_release_resources(bus);
	paintbox_bus_jqs_start_rom_firmware(bus);
	return 0;
}

void paintbox_bus_jqs_disable_firmware(struct paintbox_bus *bus)
{
	/* Notify paintbox devices that the firmware is down */
	paintbox_bus_notify_firmware_down(bus);

	switch (bus->fw_status) {
	case JQS_FW_STATUS_RAM_RUNNING:
		dev_dbg(bus->parent_dev, "disabling RAM based firmware\n");
		bus->fw_status = JQS_FW_STATUS_RAM_STAGED;
		break;
	case JQS_FW_STATUS_ROM_RUNNING:
		dev_dbg(bus->parent_dev, "disabling ROM based firmware\n");
		bus->fw_status = JQS_FW_STATUS_INIT;
		break;
	default:
		return;
	};

	paintbox_bus_writel(bus, 0, IPU_CSR_AON_OFFSET + JQS_CONTROL);

	/* Turn on isolation for I/O block */
	paintbox_bus_writel(bus, IO_ISO_ON_VAL_MASK, IPU_CSR_AON_OFFSET +
			IO_ISO_ON);

	/* Turn off clocks to I/O block */
	paintbox_bus_writel(bus, 0, IPU_CSR_AON_OFFSET +
			IPU_IO_SWITCHED_CLK_EN);

	/* Power off RAMs for I/O block */
	paintbox_bus_writel(bus, IO_RAM_ON_N_VAL_MASK, IPU_CSR_AON_OFFSET +
			IO_RAM_ON_N);

	/* Need to briefly turn on the clocks to the I/O block to propagate the
	 * RAM SD pin change into the RAM, then need to turn the clocks off
	 * again, since the I/O block is being turned off.
	 */
	paintbox_bus_writel(bus, IPU_IO_SWITCHED_CLK_EN_VAL_MASK,
			IPU_CSR_AON_OFFSET + IPU_IO_SWITCHED_CLK_EN);
	paintbox_bus_writel(bus, 0, IPU_CSR_AON_OFFSET +
			IPU_IO_SWITCHED_CLK_EN);

	/* Power off I/O block */
	paintbox_bus_writeq(bus, IO_POWER_ON_N_PRE_MASK |
			IO_POWER_ON_N_MAIN_MASK, IPU_CSR_AON_OFFSET +
			IO_POWER_ON_N);
}

void paintbox_bus_jqs_release(struct paintbox_bus *bus)
{
	paintbox_bus_jqs_disable_firmware(bus);

	paintbox_bus_release_resources(bus);
}

#ifdef CONFIG_PAINTBOX_DEBUG
static inline bool paintbox_bus_is_jqs_running(struct paintbox_bus *bus)
{
	return bus->fw_status == JQS_FW_STATUS_RAM_RUNNING ||
			bus->fw_status == JQS_FW_STATUS_ROM_RUNNING;
}

static int paintbox_bus_jqs_debug_enable_show(struct seq_file *s, void *p)
{
	struct paintbox_bus *bus = s->private;

	seq_printf(s, "%d\n", paintbox_bus_is_jqs_running(bus));
	return 0;
}

static int paintbox_bus_jqs_debug_enable_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, paintbox_bus_jqs_debug_enable_show,
			inode->i_private);
}

static ssize_t paintbox_bus_jqs_debug_enable_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_bus *bus = s->private;
	int ret, val;

	ret = kstrtoint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		if (paintbox_bus_is_jqs_running(bus) && val == 0)
			paintbox_bus_jqs_disable_firmware(bus);
		else if (!paintbox_bus_is_jqs_running(bus) && val == 1)
			paintbox_bus_jqs_enable_firmware(bus);

		return count;
	}

	dev_err(bus->parent_dev, "%s: invalid value, err = %d", __func__, ret);

	return ret < 0 ? ret : count;
}

static const struct file_operations jqs_enable_fops = {
	.open = paintbox_bus_jqs_debug_enable_open,
	.write = paintbox_bus_jqs_debug_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

void paintbox_bus_jqs_debug_init(struct paintbox_bus *bus)
{
	bus->fw_enable_dentry = debugfs_create_file("fw_enable", 0640,
			bus->debug_root, bus, &jqs_enable_fops);
	if (IS_ERR(bus->fw_enable_dentry)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->fw_enable_dentry));
	}
}

void paintbox_bus_jqs_debug_remove(struct paintbox_bus *bus)
{
	debugfs_remove(bus->fw_enable_dentry);
}
#endif

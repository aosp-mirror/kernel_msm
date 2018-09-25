// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Oscar chip.
 *
 * Copyright (C) 2017 Google, Inc.
 */

#include <asm/current.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/refcount.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "linux/mfd/abc-pcie.h"
#include "oscar.h"

#include "gasket_core.h"
#include "gasket_interrupt.h"
#include "gasket_page_table.h"
#include "gasket_sysfs.h"

/* for abc ioctls, move later */
#include <linux/dma-buf.h>
#include <linux/ab-dram.h>
#include <uapi/abc-pcie-dma.h>
#include "../../mfd/abc-pcie-dma.h"
#define OSCAR_DMA_CHAN 1

#define DRIVER_NAME "abc-pcie-tpu"
#define DRIVER_VERSION "0.3"

#define OSCAR_BAR_SIZE 0x100000

/* Number of bytes allocated for coherent memory. */
#define OSCAR_CH_MEM_BYTES (PAGE_SIZE * MAX_NUM_COHERENT_PAGES)

/* Access PCI memory via BAR 0 for the Gasket framework */
#define OSCAR_BAR_INDEX 0

/* The number of user-mappable memory ranges in Oscar BAR. */
#define NUM_BAR_RANGES 3

#define OSCAR_BAR_OFFSET 0
#define OSCAR_CM_OFFSET 0x1000000

/* The number of nodes in an Oscar chip. */
#define NUM_NODES 1

/* TPU logical interrupts handled by this driver */
#define OSCAR_SCALAR_CORE_0_INT	0 /* signalled via an MSI IRQ */
#define OSCAR_INSTR_QUEUE_INT	1 /* signalled via an MSI IRQ */
#define OSCAR_LOWPRIO_INT	2 /* AKA wireinterrupt_2, notified by parent */

#define OSCAR_N_IRQS		2 /* logical interrupts 0 and 1 are IRQs */
#define OSCAR_N_INTS		3 /* 3 logical interrupts including mux'ed */

/*
 * The total number of entries in the page table. Should match the value read
 * from the register OSCAR_BAR_REG_HIB_PAGE_TABLE_SIZE.
 */
#define OSCAR_PAGE_TABLE_TOTAL_ENTRIES 2048

#define OSCAR_EXTENDED_SHIFT 63 /* Extended address bit position. */

#define OSCAR_RESET_RETRY 120	/* check reset 120 times */
#define OSCAR_RESET_DELAY 100	/* wait 100 ms between checks */
				/* total 12 sec wait maximum */

/* enum sysfs_attribute_type: enumeration of the supported sysfs entries. */
enum sysfs_attribute_type {
	ATTR_KERNEL_HIB_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES,
};

/*
 * Register offsets into BAR memory.
 * Only values necessary for driver implementation are defined.
 */
enum oscar_bar_regs {
	OSCAR_BAR_REG_HIB_PAGE_TABLE_SIZE = 0x6000,
	OSCAR_BAR_REG_KERNEL_HIB_EXTENDED_TABLE = 0x6008,
	OSCAR_BAR_REG_KERNEL_HIB_TRANSLATION_ENABLE = 0x6010,
	OSCAR_BAR_REG_KERNEL_HIB_DMA_PAUSE = 0x46050,
	OSCAR_BAR_REG_KERNEL_HIB_DMA_PAUSE_MASK = 0x6058,
	OSCAR_BAR_REG_HIB_PAGE_TABLE_INIT = 0x6078,
	OSCAR_BAR_REG_USER_HIB_DMA_PAUSE = 0x86D8,
	OSCAR_BAR_REG_USER_HIB_DMA_PAUSED = 0x86E0,
	OSCAR_BAR_REG_HIB_PAGE_TABLE = 0x10000,

	/* Top Level Registers. */
	OSCAR_BAR_REG_AON_RESET = 0x20000,
	OSCAR_BAR_REG_AON_CLOCK_ENABLE = 0x20008,
	OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE = 0x00020010,
	OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL = 0x00020018,
	OSCAR_BAR_REG_AON_MEM_SHUTDOWN = 0x00020020,
	OSCAR_BAR_REG_AON_MEM_POWERDOWN = 0x00020028,
	OSCAR_BAR_REG_AON_CLAMP_ENABLE = 0x00020038,
	OSCAR_BAR_REG_AON_FORCE_QUIESCE = 0x00020040,
	OSCAR_BAR_REG_AON_AXITIEOFFS = 0x00020048,
	OSCAR_BAR_REG_AON_IDLE = 0x00020050,
};

/* For now map the entire BAR into user space. (This helps debugging when
 * running test vectors from user land)
 * In production driver we want to exclude the kernel HIB.
 */

/* Configuration for page table. */
static struct gasket_page_table_config oscar_page_table_configs[NUM_NODES] = {
	{
		.id = 0,
		.mode = GASKET_PAGE_TABLE_MODE_NORMAL,
		.total_entries = OSCAR_PAGE_TABLE_TOTAL_ENTRIES,
		.base_reg = OSCAR_BAR_REG_HIB_PAGE_TABLE,
		.extended_reg = OSCAR_BAR_REG_KERNEL_HIB_EXTENDED_TABLE,
		.extended_bit = OSCAR_EXTENDED_SHIFT,
	},
};

/* The regions in the BAR0 space that can be mapped into user space. */
static const struct gasket_mappable_region
oscar_mappable_regions[NUM_BAR_RANGES] = {
	{ 0x0000, 0x1000 },
	{ 0x4000, 0x1000 },
	{ 0x8000, 0x1000 },
};

static const struct gasket_mappable_region cm_mappable_regions[1] = {
	{ 0x00000, OSCAR_CH_MEM_BYTES }
};

/* Gasket interrupt data, not really used since we manage our own IRQs */
static struct gasket_interrupt_desc oscar_interrupts[OSCAR_N_INTS];

/* Airbrush-specific DRAM buffers plus PCIe DMA engine data transfers */
struct abc_buffer {
	int dma_buf_fd;
	pid_t pid;
	refcount_t refcount;
	uint32_t map_flags;
	uint32_t page_table_index;
	uint64_t device_address;
	struct list_head abc_buffers_list; /* protected by abc_buffers_lock */
	struct oscar_dev *oscar_dev;
	struct dma_buf *dma_buf;
	size_t len;
	struct mutex mapping_lock; /* protects sg_table and below */
	struct sg_table *sg_table;
	struct dma_buf_attachment *dma_buf_attachment;
	dma_addr_t abdram_dma_addr;
};


/* One of these per oscar device instance */
struct oscar_dev {
	struct gasket_dev *gasket_dev;
	bool parent_ioremap;
	int irqs[OSCAR_N_IRQS]; /* maps MSI IRQs to TPU logical ints */
	struct notifier_block lowprio_irq_nb;
	struct atomic_notifier_head *lowprio_irq_nh;
	/* List of all active ABC DRAM buffers, protected by abc_buffers_lock */
	struct mutex abc_buffers_lock;
	struct list_head abc_buffers;
};


/* Act as if only GCB is instantiated. */
static int bypass_top_level;
module_param(bypass_top_level, int, 0644);

static int oscar_device_open_cb(struct gasket_dev *gasket_dev)
{
	return gasket_reset_nolock(gasket_dev);
}

static int oscar_get_status(struct gasket_dev *gasket_dev)
{
	/* Always returns ALIVE for now */
	return GASKET_STATUS_ALIVE;
}

/* Caller must hold abc_buffers_lock */
static struct abc_buffer *abc_get_buffer_locked(struct oscar_dev *oscar_dev,
						int fd)
{
	struct abc_buffer *abc_buffer;

	list_for_each_entry(abc_buffer, &oscar_dev->abc_buffers,
			    abc_buffers_list) {
		if (abc_buffer->dma_buf_fd == fd &&
		    abc_buffer->pid == current->pid) {
			refcount_inc(&abc_buffer->refcount);
			return abc_buffer;

		}
	}
	return NULL;
}

static struct abc_buffer *abc_get_buffer(struct oscar_dev *oscar_dev, int fd)
{
	struct abc_buffer *buffer;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	buffer = abc_get_buffer_locked(oscar_dev, fd);
	mutex_unlock(&oscar_dev->abc_buffers_lock);
	return buffer;
}

/* caller must hold abc_buffer->mapping_lock */
static void abc_unmap_buffer_locked(struct abc_buffer *abc_buffer)
{
	struct gasket_dev *gasket_dev = abc_buffer->oscar_dev->gasket_dev;
	int pgtbl_index;

	if (!abc_buffer->sg_table)
		return;
	pgtbl_index = abc_buffer->page_table_index;
	gasket_page_table_unmap(gasket_dev->page_table[pgtbl_index],
				abc_buffer->device_address,
				abc_buffer->len / PAGE_SIZE);

	dma_buf_unmap_attachment(abc_buffer->dma_buf_attachment,
				 abc_buffer->sg_table, DMA_TO_DEVICE);

	abc_buffer->sg_table = NULL;
	abc_buffer->abdram_dma_addr = 0;
	abc_buffer->len = 0;
	abc_buffer->map_flags = 0;
	abc_buffer->device_address = 0;
	abc_buffer->page_table_index = 0;
}

/* abc_buffer not valid on return */
static void oscar_abc_release_buffer(struct abc_buffer *abc_buffer)
{
	mutex_lock(&abc_buffer->mapping_lock);
	if (abc_buffer->sg_table)
		abc_unmap_buffer_locked(abc_buffer);
	dma_buf_detach(abc_buffer->dma_buf, abc_buffer->dma_buf_attachment);
	abc_buffer->dma_buf_attachment = NULL;
	mutex_unlock(&abc_buffer->mapping_lock);
	dma_buf_put(abc_buffer->dma_buf); /* release our ref on dma_buf */
	kfree(abc_buffer);
}

/* abc_buffer not valid on return */
static void abc_put_buffer_locked(struct abc_buffer *abc_buffer)
{
	if (refcount_dec_and_test(&abc_buffer->refcount))
		oscar_abc_release_buffer(abc_buffer);
}

/* abc_buffer not valid on return */
static void abc_put_buffer(struct abc_buffer *abc_buffer)
{
	struct oscar_dev *oscar_dev = abc_buffer->oscar_dev;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	abc_put_buffer_locked(abc_buffer);
	mutex_unlock(&oscar_dev->abc_buffers_lock);
}

static int oscar_abc_alloc_buffer(struct oscar_dev *oscar_dev,
				  struct oscar_abdram_alloc_ioctl __user *argp)
{
	struct oscar_abdram_alloc_ioctl ibuf;
	struct dma_buf *abc_dma_buf;
	struct dma_buf *temp_dma_buf;
	struct dma_buf_attachment *dma_buf_attachment;
	struct abc_buffer *abc_buffer;
	struct abc_buffer *temp_abc_buf;
	int fd;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	abc_dma_buf = ab_dram_alloc_dma_buf_kernel((size_t)ibuf.size);
	if (IS_ERR(abc_dma_buf))
		return PTR_ERR(abc_dma_buf);

	dma_buf_attachment =
		dma_buf_attach(abc_dma_buf, oscar_dev->gasket_dev->dev);
	if (IS_ERR(dma_buf_attachment)) {
		ret = PTR_ERR(dma_buf_attachment);
		goto free_buf;
	}

	fd = dma_buf_fd(abc_dma_buf, O_CLOEXEC);
	if (fd < 0) {
		ret = fd;
		goto detach_buf;
	}

	/* grab our own ref to the dma_buf for so long as we use it */
	temp_dma_buf = dma_buf_get(fd);
	if (IS_ERR(temp_dma_buf)) {
		ret = PTR_ERR(temp_dma_buf);
		goto detach_buf;
	}

	abc_buffer = kmalloc(sizeof(struct abc_buffer), GFP_KERNEL);
	if (!abc_buffer) {
		ret = -ENOMEM;
		goto detach_buf;
	}
	abc_buffer->dma_buf_fd = fd;
	abc_buffer->pid = current->pid;
	abc_buffer->dma_buf = abc_dma_buf;
	abc_buffer->dma_buf_attachment = dma_buf_attachment;
	abc_buffer->oscar_dev = oscar_dev;
	INIT_LIST_HEAD(&abc_buffer->abc_buffers_list);
	refcount_set(&abc_buffer->refcount, 1);
	mutex_init(&abc_buffer->mapping_lock);
	abc_buffer->sg_table = NULL;
	abc_buffer->abdram_dma_addr = 0;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	temp_abc_buf = abc_get_buffer_locked(oscar_dev, fd);
	if (WARN_ON(temp_abc_buf)) {
		abc_put_buffer(temp_abc_buf);
		kfree(abc_buffer);
		ret = -EBUSY;
		goto detach_buf;
	}

	list_add_tail(&oscar_dev->abc_buffers, &abc_buffer->abc_buffers_list);
	mutex_unlock(&oscar_dev->abc_buffers_lock);
	return fd;

detach_buf:
	dma_buf_detach(abc_dma_buf, dma_buf_attachment);
free_buf:
	ab_dram_free_dma_buf_kernel(abc_dma_buf);
	return ret;
}

static int oscar_abc_sync_buffer(struct oscar_dev *oscar_dev,
				 struct oscar_abdram_sync_ioctl __user *argp)
{
	struct oscar_abdram_sync_ioctl ibuf;
	struct dma_buf *abc_dma_buf;
	struct abc_pcie_dma_desc abc_dma_desc;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	abc_dma_buf = dma_buf_get(ibuf.fd);
	if (IS_ERR(abc_dma_buf))
		return PTR_ERR(abc_dma_buf);

	abc_dma_desc.local_buf_type = DMA_BUFFER_USER;
	abc_dma_desc.local_buf = (void *)ibuf.host_address;
	abc_dma_desc.local_buf_size = ibuf.len;
	abc_dma_desc.remote_buf_type = DMA_BUFFER_DMA_BUF;
	abc_dma_desc.remote_dma_buf_fd = ibuf.fd;
	abc_dma_desc.dir = ibuf.cmd == OSCAR_SYNC_FROM_BUFFER ?
		DMA_FROM_DEVICE : DMA_TO_DEVICE;
	abc_dma_desc.chan = OSCAR_DMA_CHAN;
	ret = abc_pcie_issue_dma_xfer(&abc_dma_desc);

	dma_buf_put(abc_dma_buf);
	return ret;
}

static int oscar_abc_map_buffer(struct oscar_dev *oscar_dev,
				struct oscar_abdram_map_ioctl __user *argp)
{
	struct gasket_dev *gasket_dev = oscar_dev->gasket_dev;
	struct oscar_abdram_map_ioctl ibuf;
	struct abc_buffer *abc_buffer;
	int fd;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.page_table_index >= gasket_dev->num_page_tables)
		return -EFAULT;

	fd = ibuf.fd;
	abc_buffer = abc_get_buffer(oscar_dev, fd);
	if (!abc_buffer)
		return -EINVAL;

	mutex_lock(&abc_buffer->mapping_lock);
	if (abc_buffer->sg_table) {
		ret = -EBUSY;
		goto put_buffer;
	}

	/* TODO: merge dma dir flags support */
	abc_buffer->sg_table =
		dma_buf_map_attachment(abc_buffer->dma_buf_attachment,
				       DMA_TO_DEVICE);
	if (IS_ERR(abc_buffer->sg_table)) {
		ret = PTR_ERR(abc_buffer->sg_table);
		goto put_buffer;
	}

	abc_buffer->abdram_dma_addr =
		sg_dma_address(abc_buffer->sg_table->sgl);
	abc_buffer->len = sg_dma_len(abc_buffer->sg_table->sgl);

	if (gasket_page_table_are_addrs_bad(
			    gasket_dev->page_table[ibuf.page_table_index],
			    abc_buffer->abdram_dma_addr, ibuf.device_address,
			    abc_buffer->len)) {
		ret = -EINVAL;
		goto unmap_sg;
	}

	abc_buffer->page_table_index = ibuf.page_table_index;
	abc_buffer->device_address = ibuf.device_address;
	ret = gasket_page_table_map(
		    gasket_dev->page_table[ibuf.page_table_index],
		    abc_buffer->abdram_dma_addr, abc_buffer->device_address,
		    abc_buffer->len / PAGE_SIZE,  false);

unmap_sg:
	if (ret) {
		dma_buf_unmap_attachment(abc_buffer->dma_buf_attachment,
					 abc_buffer->sg_table, DMA_TO_DEVICE);
		abc_buffer->sg_table = NULL;
	}
put_buffer:
	abc_put_buffer(abc_buffer);
	mutex_unlock(&abc_buffer->mapping_lock);
	return ret;
}

static int oscar_abc_unmap_buffer(struct oscar_dev *oscar_dev, int dma_buf_fd)
{
	struct abc_buffer *abc_buffer;

	abc_buffer = abc_get_buffer(oscar_dev, dma_buf_fd);
	if (!abc_buffer)
		return -EINVAL;

	mutex_lock(&abc_buffer->mapping_lock);
	abc_unmap_buffer_locked(abc_buffer);
	mutex_unlock(&abc_buffer->mapping_lock);
	abc_put_buffer(abc_buffer);
	return 0;
}

/*
 * Caller must hold abc_buffers_lock.  abc_buffer not valid on return,
 * and abc_buffers list entry is deleted, use safe iterators.
 */
static void oscar_abc_mark_buffer_for_release(struct abc_buffer *abc_buffer)
{
	list_del(&abc_buffer->abc_buffers_list);
	abc_put_buffer_locked(abc_buffer);
}

static int oscar_abc_dealloc_buffer(struct oscar_dev *oscar_dev, int dma_buf_fd)
{
	struct abc_buffer *abc_buffer;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	abc_buffer = abc_get_buffer_locked(oscar_dev, dma_buf_fd);
	if (!abc_buffer) {
		mutex_unlock(&oscar_dev->abc_buffers_lock);
		return -EINVAL;
	}
	abc_put_buffer_locked(abc_buffer);
	oscar_abc_mark_buffer_for_release(abc_buffer);
	mutex_unlock(&oscar_dev->abc_buffers_lock);
	return 0;
}

/* Enters GCB reset state. */
static int oscar_enter_reset(struct gasket_dev *gasket_dev)
{
	struct oscar_dev *oscar_dev =
		platform_get_drvdata(gasket_dev->platform_dev);
	struct abc_buffer *buffer;
	struct abc_buffer *buftemp;

	mutex_lock(&oscar_dev->abc_buffers_lock);
	list_for_each_entry_safe(buffer, buftemp, &oscar_dev->abc_buffers,
				 abc_buffers_list) {
		oscar_abc_mark_buffer_for_release(buffer);
	}
	mutex_unlock(&oscar_dev->abc_buffers_lock);

	if (bypass_top_level)
		return 0;

	/* 1. Check whether we are already in reset to guard HIB access. */
	if (gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
			       OSCAR_BAR_REG_AON_RESET) == 0) {
		/* 1a. Enable DMA Pause. */
		gasket_dev_write_64(gasket_dev, 1, OSCAR_BAR_INDEX,
				    OSCAR_BAR_REG_USER_HIB_DMA_PAUSE);
		/* 1b. Wait for DMA Pause to complete. */
		if (gasket_wait_with_reschedule(gasket_dev, OSCAR_BAR_INDEX,
						OSCAR_BAR_REG_USER_HIB_DMA_PAUSED,
						1, 1, OSCAR_RESET_RETRY,
						OSCAR_RESET_DELAY)) {
			dev_err(gasket_dev->dev,
				"DMA pause failed after timeout (%d ms)\n",
				OSCAR_RESET_RETRY * OSCAR_RESET_DELAY);
			return -ETIMEDOUT;
		}
	}

	/* 2. Enable Quiesce. */
	gasket_dev_write_64(gasket_dev, 1, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_FORCE_QUIESCE);

	/* 3. Enable Reset. */
	gasket_dev_write_64(gasket_dev, 1, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_RESET);

	/*
	 *  4. Disable Clock Enable.
	 *  - clock_enable = 0.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 2, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 5. Enable Clamp. */
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLAMP_ENABLE);

	/* 6. Enable Memory shutdown. */
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_MEM_SHUTDOWN);
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_MEM_POWERDOWN);

	/* 7. Enable Logic shutdown. */
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL);
	gasket_dev_write_64(gasket_dev, 0x1ffff, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE);
	return 0;
}

/* Called on final close via device_close_cb. */
static int oscar_device_cleanup(struct gasket_dev *gasket_dev)
{
	return oscar_enter_reset(gasket_dev);
}

/* Quits GCB reset state. */
static int oscar_quit_reset(struct gasket_dev *gasket_dev)
{
	uint64_t reg;

	if (bypass_top_level)
		return 0;

	/* 1. Enable Logic shutdown. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_PRE);
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_LOGIC_SHUTDOWN_ALL);

	/*
	 * 2. Enable Clock Enable, and set idle_override to force the clock on.
	 * - clock_enable = 1.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 3, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/*
	 * 3. Disable Clock Enable.
	 *  - clock_enable = 0.
	 *  - cb_idle_override = 1.
	 */
	gasket_dev_write_64(gasket_dev, 2, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 4. Disable Memory shutdown. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_MEM_SHUTDOWN);
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_MEM_POWERDOWN);

	/*
	 * 5. Enable Clock Enable, with dynamic activity based clock gating.
	 *  - clock_enable = 1.
	 *  - cb_idle_override = 0.
	 */
	gasket_dev_write_64(gasket_dev, 3, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLOCK_ENABLE);

	/* 6. Disable Clamp. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_CLAMP_ENABLE);

	/* 7. Disable Reset. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_RESET);

	/* 8. Disable Quiesce. */
	gasket_dev_write_64(gasket_dev, 0, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_FORCE_QUIESCE);

	/*
	 * Set tpu_aon register axiTieOffsReg fields arcache and awcache
	 * bit 1, allowing DRAM controller to combine our 16B transactions into
	 * 32B transactions to get full bandwidth.
	 * TODO: May not be necessary on later Airbrush cards.
	 */
	reg = gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
				 OSCAR_BAR_REG_AON_AXITIEOFFS);
	reg |= 0x2 << 18 | 0x2 << 2;
	gasket_dev_write_64(gasket_dev, reg, OSCAR_BAR_INDEX,
			    OSCAR_BAR_REG_AON_AXITIEOFFS);
	return 0;
}

static int oscar_reset(struct gasket_dev *gasket_dev)
{
	int ret = 0;

	if (bypass_top_level)
		return 0;

	ret = oscar_enter_reset(gasket_dev);
	if (ret < 0)
		return ret;
	return oscar_quit_reset(gasket_dev);
}

/* Gate or un-gate Oscar clock. */
static long oscar_clock_gating(struct gasket_dev *gasket_dev,
			       struct oscar_gate_clock_ioctl __user *argp)
{
	struct oscar_gate_clock_ioctl ibuf;

	if (bypass_top_level)
		return 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	dev_dbg(gasket_dev->dev, "%s %llu\n", __func__, ibuf.enable);
	/* TODO: implement or remove this ioctl */
	return 0;
}

static uint oscar_ioctl_check_permissions(struct file *filp, uint cmd)
{
	return !!(filp->f_mode & FMODE_WRITE);
}

static long oscar_ioctl(struct file *filp, uint cmd, void __user *argp)
{
	struct gasket_dev *gasket_dev = filp->private_data;
	struct oscar_dev *oscar_dev =
		platform_get_drvdata(gasket_dev->platform_dev);
	struct oscar_abdram_alloc_ioctl __user *abdram_alloc;
	struct oscar_abdram_sync_ioctl __user *abdram_sync;
	struct oscar_abdram_map_ioctl __user *abdram_map;

	if (!oscar_ioctl_check_permissions(filp, cmd))
		return -EPERM;

	switch (cmd) {
	case OSCAR_IOCTL_GATE_CLOCK:
		return oscar_clock_gating(gasket_dev, argp);
	case OSCAR_IOCTL_ABC_ALLOC_BUFFER:
		abdram_alloc = (struct oscar_abdram_alloc_ioctl *)argp;
		return oscar_abc_alloc_buffer(oscar_dev, abdram_alloc);
	case OSCAR_IOCTL_ABC_SYNC_BUFFER:
		abdram_sync = (struct oscar_abdram_sync_ioctl *)argp;
		return oscar_abc_sync_buffer(oscar_dev, abdram_sync);
	case OSCAR_IOCTL_ABC_MAP_BUFFER:
		abdram_map = (struct oscar_abdram_map_ioctl *)argp;
		return oscar_abc_map_buffer(oscar_dev, abdram_map);
	case OSCAR_IOCTL_ABC_UNMAP_BUFFER:
		return oscar_abc_unmap_buffer(oscar_dev, (int)argp);
	case OSCAR_IOCTL_ABC_DEALLOC_BUFFER:
		return oscar_abc_dealloc_buffer(oscar_dev, (int)argp);
	default:
		return -ENOTTY; /* unknown command */
	}
}

static ssize_t sysfs_show(struct device *device, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	struct gasket_dev *gasket_dev;
	struct gasket_sysfs_attribute *gasket_attr;
	enum sysfs_attribute_type type;

	gasket_dev = gasket_sysfs_get_device_data(device);
	if (!gasket_dev) {
		dev_err(device, "No Gasket device sysfs mapping found\n");
		return -ENODEV;
	}

	gasket_attr = gasket_sysfs_get_attr(device, attr);
	if (!gasket_attr) {
		dev_err(device, "No Gasket device sysfs attr data found\n");
		gasket_sysfs_put_device_data(device, gasket_dev);
		return -ENODEV;
	}

	type = (enum sysfs_attribute_type)gasket_attr->data.attr_type;
	switch (type) {
	case ATTR_KERNEL_HIB_PAGE_TABLE_SIZE:
		ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				gasket_page_table_num_entries(
					gasket_dev->page_table[0]));
		break;
	case ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE:
		ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				gasket_page_table_num_entries(
					gasket_dev->page_table[0]));
		break;
	case ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES:
		ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				gasket_page_table_num_active_pages(
					gasket_dev->page_table[0]));
		break;
	default:
		dev_dbg(gasket_dev->dev, "Unknown attribute: %s\n",
			attr->attr.name);
		ret = 0;
		break;
	}

	gasket_sysfs_put_attr(device, gasket_attr);
	gasket_sysfs_put_device_data(device, gasket_dev);
	return ret;
}

static struct gasket_sysfs_attribute oscar_sysfs_attrs[] = {
	GASKET_SYSFS_RO(node_0_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_PAGE_TABLE_SIZE),
	GASKET_SYSFS_RO(node_0_simple_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE),
	GASKET_SYSFS_RO(node_0_num_mapped_pages, sysfs_show,
		ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES),
	GASKET_END_OF_ATTR_ARRAY
};

static struct gasket_driver_desc oscar_gasket_desc = {
	.name = DRIVER_NAME,
	.driver_version = DRIVER_VERSION,
	.major = 120,
	.minor = 0,
	.module = THIS_MODULE,

	.num_page_tables = NUM_NODES,
	.page_table_bar_index = OSCAR_BAR_INDEX,
	.page_table_configs = oscar_page_table_configs,
	.page_table_extended_bit = OSCAR_EXTENDED_SHIFT,

	.bar_descriptions = {
		{
			OSCAR_BAR_SIZE,
			(VM_WRITE | VM_READ),
			OSCAR_BAR_OFFSET,
			NUM_BAR_RANGES,
			oscar_mappable_regions,
			PCI_BAR
		},
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
		GASKET_UNUSED_BAR,
	},
	.coherent_buffer_description = {
		OSCAR_CH_MEM_BYTES,
		(VM_WRITE | VM_READ),
		OSCAR_CM_OFFSET,
	},
	.interrupt_type = DEVICE_MANAGED,
	.num_interrupts = OSCAR_N_INTS,
	.interrupts = oscar_interrupts,

	.device_open_cb = oscar_device_open_cb,
	.device_close_cb = oscar_device_cleanup,
	.ioctl_handler_cb = oscar_ioctl,
	.device_status_cb = oscar_get_status,
	.hardware_revision_cb = NULL,
	.device_reset_cb = oscar_reset,
};

static irqreturn_t oscar_interrupt_handler(int irq, void *arg)
{
	struct oscar_dev *oscar_dev = (struct oscar_dev *)arg;
	struct gasket_interrupt_data *interrupt_data =
		oscar_dev->gasket_dev->interrupt_data;
	int i;

	/* Map this IRQ to a TPU logical interrupt, send to gasket */
	for (i = 0; i < OSCAR_N_IRQS; i++) {
		if (irq == oscar_dev->irqs[i]) {
			gasket_handle_interrupt(interrupt_data, i);
			return IRQ_HANDLED;
		}
	}

	return IRQ_NONE;
}

static int oscar_lowprio_irq_notify(struct notifier_block *nb,
				    unsigned long irq, void *data)
{
	struct oscar_dev *oscar_dev =
	    container_of(nb, struct oscar_dev, lowprio_irq_nb);
	struct gasket_interrupt_data *interrupt_data =
		oscar_dev->gasket_dev->interrupt_data;
	u32 intnc_val = (u32)data;

	if (irq == ABC_MSI_AON_INTNC &&
	    (intnc_val & 1 << INTNC_TPU_WIREINTERRUPT2))
		gasket_handle_interrupt(interrupt_data, OSCAR_LOWPRIO_INT);
	return NOTIFY_OK;
}

static void oscar_interrupt_cleanup(struct oscar_dev *oscar_dev)
{
	int ret;

	ret = atomic_notifier_chain_unregister(oscar_dev->lowprio_irq_nh,
					       &oscar_dev->lowprio_irq_nb);
	if (ret)
		dev_warn(oscar_dev->gasket_dev->dev,
			 "Unregister lowprio irq notifier failed: %d\n", ret);
}

static int oscar_setup_device(struct platform_device *pdev,
			      struct oscar_dev *oscar_dev,
			      struct gasket_dev *gasket_dev)
{
	struct resource *r;
	int irq;
	ulong page_table_ready;
	int retries = 0;
	struct device *dev = &pdev->dev;
	const struct dma_map_ops *parent_dma_ops =
		get_dma_ops(pdev->dev.parent);
	phys_addr_t mem_phys;
	resource_size_t mem_size;
	void *mem_virt = NULL;
	u64 u64prop;
	int ret;

	if (parent_dma_ops)
		set_dma_ops(dev, parent_dma_ops);
	else
		dev_warn(dev, "No dma_ops to inherit from parent mfd device\n");

	/*
	 * The memory resource passes the physical address range of our
	 * memory region in the associated BAR of the parent mfd device.
	 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(dev, "cannot get mem resource\n");
		return -ENODEV;
	}
	mem_phys = r->start;
	mem_size = resource_size(r);

	/*
	 * If the parent device has already mapped our region (to handle
	 * multi-function device-specific init actions) then it passes the
	 * virtual address of the I/O remapping to us in property
	 * "tpu-mem-mapping".
	 */

	ret = device_property_read_u64(dev, "tpu-mem-mapping", &u64prop);
	if (!ret)
		mem_virt = (void *)u64prop;

	if (mem_virt) {
		oscar_dev->parent_ioremap = true;
	} else {
		dev_info(dev, "no tpu-mem-mapping from parent, remapping\n");
		mem_virt = ioremap_nocache(mem_phys, mem_size);
		if (!mem_virt) {
			dev_err(dev, "failed to map our memory region\n");
			return -ENODEV;
		}
	}

	gasket_dev->bar_data[OSCAR_BAR_INDEX].phys_base = mem_phys;
	gasket_dev->bar_data[OSCAR_BAR_INDEX].length_bytes = mem_size;
	gasket_dev->bar_data[OSCAR_BAR_INDEX].virt_base = mem_virt;

	irq = platform_get_irq_byname(pdev, "tpu-scalar-core-0-irq");
	if (irq < 0) {
		dev_err(dev, "cannot get scalar core 0 irq\n");
		return -ENODEV;
	}
	oscar_dev->irqs[OSCAR_SCALAR_CORE_0_INT] = irq;
	ret = devm_request_irq(dev, irq, oscar_interrupt_handler, IRQF_ONESHOT,
			       dev_name(dev), oscar_dev);
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", irq);
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "tpu-instr-queue-irq");
	if (irq < 0) {
		dev_err(dev, "cannot get instruction queue irq\n");
		return -ENODEV;
	}
	oscar_dev->irqs[OSCAR_INSTR_QUEUE_INT] = irq;
	ret = devm_request_irq(dev, irq, oscar_interrupt_handler, IRQF_ONESHOT,
			       dev_name(dev), oscar_dev);
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", irq);
		return -ENODEV;
	}

	ret = device_property_read_u64(dev, "intnc-notifier-chain", &u64prop);
	if (!ret)
		oscar_dev->lowprio_irq_nh =
		    (struct atomic_notifier_head *)u64prop;
	if (oscar_dev->lowprio_irq_nh) {
		oscar_dev->lowprio_irq_nb.notifier_call =
		    oscar_lowprio_irq_notify;
		ret = atomic_notifier_chain_register(oscar_dev->lowprio_irq_nh,
						    &oscar_dev->lowprio_irq_nb);
		if (ret)
			dev_warn(dev,
				 "Cannot register notifier for lowprio irq\n");
	} else {
		dev_warn(dev,
			 "no intnc non-critical irq notifier supplied\n");
	}

	oscar_reset(gasket_dev);

	while (retries < OSCAR_RESET_RETRY) {
		page_table_ready =
		    gasket_dev_read_64(gasket_dev, OSCAR_BAR_INDEX,
				       OSCAR_BAR_REG_HIB_PAGE_TABLE_INIT);
		if (page_table_ready)
			break;
		schedule_timeout(msecs_to_jiffies(OSCAR_RESET_DELAY));
		retries++;
	}

	if (retries == OSCAR_RESET_RETRY) {
		dev_err(gasket_dev->dev, "Page table init timed out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int oscar_probe(struct platform_device *pdev)
{
	int ret;
	struct oscar_dev *oscar_dev;
	struct gasket_dev *gasket_dev;
	struct device *dev = &pdev->dev;

	ret = gasket_platform_add_device(pdev, &gasket_dev);
	if (ret) {
		dev_err(dev, "error adding gasket device\n");
		return ret;
	}

	oscar_dev = kmalloc(sizeof(struct oscar_dev), GFP_KERNEL);
	if (!oscar_dev) {
		ret = -ENOMEM;
		goto remove_device;
	}

	platform_set_drvdata(pdev, oscar_dev);
	oscar_dev->gasket_dev = gasket_dev;
	INIT_LIST_HEAD(&oscar_dev->abc_buffers);
	mutex_init(&oscar_dev->abc_buffers_lock);

	ret = oscar_setup_device(pdev, oscar_dev, gasket_dev);
	if (ret) {
		dev_err(dev, "Setup device failed\n");
		goto remove_device;
	}

	ret = gasket_sysfs_create_entries(gasket_dev->dev_info.device,
					  oscar_sysfs_attrs);
	if (ret)
		dev_err(dev, "error creating device sysfs entries\n");

	ret = gasket_enable_device(gasket_dev);
	if (ret) {
		dev_err(dev, "error enabling gasket device\n");
		goto remove_device;
	}

	/* Place device in low power mode until opened */
	oscar_enter_reset(gasket_dev);
	return 0;

remove_device:
	gasket_platform_remove_device(pdev);
	kfree(oscar_dev);
	return ret;
}

static int oscar_remove(struct platform_device *pdev)
{
	struct oscar_dev *oscar_dev = platform_get_drvdata(pdev);
	struct gasket_dev *gasket_dev = oscar_dev->gasket_dev;

	gasket_disable_device(gasket_dev);
	oscar_interrupt_cleanup(oscar_dev);
	if (!oscar_dev->parent_ioremap)
		iounmap(gasket_dev->bar_data[OSCAR_BAR_INDEX].virt_base);
	gasket_platform_remove_device(pdev);
	kfree(oscar_dev);
	return 0;
}

static struct platform_driver oscar_platform_driver = {
		.probe = oscar_probe,
		.remove = oscar_remove,
		.driver = {
			.name = DRIVER_NAME,
		}
};

static int __init oscar_init(void)
{
	int ret;

	ret = platform_driver_register(&oscar_platform_driver);
	if (!ret)
		ret = gasket_register_device(&oscar_gasket_desc);
	if (ret)
		platform_driver_unregister(&oscar_platform_driver);
	return ret;
}

static void __exit oscar_exit(void)
{
	gasket_unregister_device(&oscar_gasket_desc);
	platform_driver_unregister(&oscar_platform_driver);
}

MODULE_DESCRIPTION("Google Oscar driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("John Joseph <jnjoseph@google.com>");
module_init(oscar_init);
module_exit(oscar_exit);

// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Oscar chip.
 *
 * Copyright (C) 2017 Google, Inc.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "../../mfd/abc-pcie-private.h"
#include "linux/mfd/abc-pcie.h"
#include "oscar.h"

#include "gasket_core.h"
#include "gasket_interrupt.h"
#include "gasket_page_table.h"
#include "gasket_sysfs.h"

#define DRIVER_NAME "abc-pcie-tpu"
#define DRIVER_VERSION "0.2"

#define OSCAR_BAR_SIZE 0x100000

/* Number of bytes allocated for coherent memory. */
#define OSCAR_CH_MEM_BYTES (PAGE_SIZE * MAX_NUM_COHERENT_PAGES)

/*
 * Access PCI memory via BAR 0 for the Gasket framework; the actual BAR mapping
 * has been setup by the parent device.
 */
#define OSCAR_BAR_INDEX 0

/* The number of user-mappable memory ranges in Oscar BAR. */
#define NUM_BAR_RANGES 3

#define OSCAR_BAR_OFFSET 0
#define OSCAR_CM_OFFSET 0x1000000

/* The number of nodes in an Oscar chip. */
#define NUM_NODES 1

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

/* TODO: pass these as resources from mfd driver */
static struct gasket_interrupt_desc oscar_interrupts[] = {
	{
		ABC_MSI_4_TPU_IRQ0, 0, 0,
	},
	{
		ABC_MSI_5_TPU_IRQ1, 0, 0,
	},
	{
		INTNC_TPU_WIREINTERRUPT2, 0, 0,
	},
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

/* Enters GCB reset state. */
static int oscar_enter_reset(struct gasket_dev *gasket_dev)
{
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

	if (!oscar_ioctl_check_permissions(filp, cmd))
		return -EPERM;

	switch (cmd) {
	case OSCAR_IOCTL_GATE_CLOCK:
		return oscar_clock_gating(gasket_dev, argp);
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

	type = (enum sysfs_attribute_type)gasket_sysfs_get_attr(device, attr);
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
	.num_interrupts = ARRAY_SIZE(oscar_interrupts),
	.interrupts = oscar_interrupts,

	.device_open_cb = oscar_device_open_cb,
	.device_close_cb = oscar_device_cleanup,
	.ioctl_handler_cb = oscar_ioctl,
	.device_status_cb = oscar_get_status,
	.hardware_revision_cb = NULL,
	.device_reset_cb = oscar_reset,
};

static int oscar_interrupt_callback(uint32_t irq, void *payload)
{
	struct gasket_dev *gasket_dev = (struct gasket_dev *)payload;
	struct gasket_interrupt_data *interrupt_data =
		gasket_dev->interrupt_data;
	int i;

	for (i = 0; i < oscar_gasket_desc.num_interrupts; i++) {
		if (irq == oscar_interrupts[i].index) {
			gasket_handle_interrupt(interrupt_data, i);
			return IRQ_HANDLED;
		}
	}

	return IRQ_NONE;
}

static int oscar_register_interrupt_callbacks(struct gasket_dev *gasket_dev)
{
	int ret, i;

	for (i = 0; i < oscar_gasket_desc.num_interrupts; i++) {
		ret = abc_reg_irq_callback2(&oscar_interrupt_callback,
					    oscar_interrupts[i].index,
					    gasket_dev);
		if (ret) {
			dev_err(gasket_dev->dev,
				"Cannot register IRQ callback for index %d: %d\n",
				i, ret);
			return ret;
		}
	}
	return 0;
}

static void oscar_interrupt_cleanup(struct gasket_dev *gasket_dev)
{
	int ret, i;

	for (i = 0; i < oscar_gasket_desc.num_interrupts; i++) {
		ret = abc_reg_irq_callback2(NULL, oscar_interrupts[i].index,
					    NULL);
		if (ret)
			dev_warn(gasket_dev->dev,
				"Unregister IRQ callback for index %d: %d\n",
				i, ret);
	}
}

static int oscar_setup_device(struct gasket_dev *gasket_dev)
{
	ulong page_table_ready;
	int retries = 0;
	int ret;

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

	ret = oscar_register_interrupt_callbacks(gasket_dev);
	return ret;
}

static int oscar_probe(struct platform_device *pdev)
{
	struct resource *r;

	int ret;
	struct gasket_dev *gasket_dev;
	struct device *dev = &pdev->dev;
	/* TODO: remove abc_dev after fixing memory resources */
	struct abc_device *abc_dev = pdev->dev.platform_data;
	const struct dma_map_ops *parent_dma_ops =
		get_dma_ops(pdev->dev.parent);

	if (parent_dma_ops)
		set_dma_ops(dev, parent_dma_ops);
	else
		dev_warn(dev, "No dma_ops to inherit from parent mfd device\n");

	ret = gasket_platform_add_device(pdev, &gasket_dev);
	if (ret) {
		dev_err(dev, "error adding gasket device\n");
		return ret;
	}

	platform_set_drvdata(pdev, gasket_dev);

	/*
	 * Get the IO memory resource for this device, this corresponds
	 * to the BAR that has been mapped by the parent mfd device.
	 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(dev, "cannot get mem resource\n");
		ret = -ENODEV;
		goto remove_device;
	}

	/* Map BARs */
	gasket_dev->bar_data[OSCAR_BAR_INDEX].phys_base = r->start;
	gasket_dev->bar_data[OSCAR_BAR_INDEX].length_bytes = r->end - r->start;

#if 0 /* TODO: fix addresses passed, restore this code */
	if (!devm_request_mem_region(dev,
				     gasket_dev->bar_data[OSCAR_BAR_INDEX].phys_base,
				     gasket_dev->bar_data[OSCAR_BAR_INDEX].length_bytes,
				     gasket_dev->dev_info.name)) {
		dev_err(dev, "cannot request BAR %d memory region\n",
			OSCAR_BAR_INDEX);
		ret = -EINVAL;
		goto remove_device;
	}
#endif /* TODO */

	/* TODO: pass this in mem resource */
	gasket_dev->bar_data[OSCAR_BAR_INDEX].virt_base = abc_dev->tpu_config;
	if (!gasket_dev->bar_data[OSCAR_BAR_INDEX].virt_base) {
		dev_err(dev, "BAR %d memory region not setup\n",
			OSCAR_BAR_INDEX);
		ret = -ENODEV;
		goto remove_device;
	}

	ret = oscar_setup_device(gasket_dev);
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
	return ret;
}

static int oscar_remove(struct platform_device *pdev)
{
	struct gasket_dev *gasket_dev = platform_get_drvdata(pdev);

	gasket_disable_device(gasket_dev);
	oscar_interrupt_cleanup(gasket_dev);
	iounmap(gasket_dev->bar_data[OSCAR_BAR_INDEX].virt_base);
#if 0 /* TODO: restore when proper addresses passed from mfd driver */
	release_mem_region(gasket_dev->bar_data[OSCAR_BAR_INDEX].phys_base,
			   gasket_dev->bar_data[OSCAR_BAR_INDEX].length_bytes);
#endif /* TODO */
	gasket_platform_remove_device(pdev);
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

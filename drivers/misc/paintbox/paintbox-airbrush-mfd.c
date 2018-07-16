/*
 * PCI bus driver for the Paintbox programmable IPU
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
#include <linux/kernel.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>

/* TODO(ahampson):  Lassen needs to move MSI enum to a public header. */
#include "../../mfd/abc-pcie-private.h"

#include "paintbox-bus-impl.h"
#include "paintbox-regs.h"

struct paintbox_airbrush_mfd_data {
	struct paintbox_bus *bus;
	struct device *dev;

	/* Depending on the IOMMU configuration the board the IPU may need
	 * to use MFD parent's device for mapping DMA buffers.  Otherwise, the
	 * dma device can be set to our own device and use SWIOTLB to map
	 * buffers.
	 */
	struct device *dma_dev;

	/* TODO(ahampson, rogerwolff) This should be replaced with
	 * interactions with the Airbrush DRAM manager.  For now, this
	 * this is just a bump allocator that does not expect any memory
	 * to be freed.
	 */
	dma_addr_t next_jqs_allocation;

	struct paintbox_pdata pdata;
	struct paintbox_bus_ops ops;
};

/* Paintbox IO virtual address space bounds
 * TODO:  These are place holder values.  I need to figure out the
 * correct value for these.  This comes out to 512MB right now.
 */
#define PAINTBOX_IOVA_START		0x20000000
#define PAINTBOX_IOVA_SIZE		0x40000000

/* TODO(ahampson):  There is no Airbrush DRAM manager yet so we just write the
 * firmware image to the base of Airbrush DRAM.  b/74122875
 */
#define AIRBRUSH_DRAM_START_PADDR 0x20000000

/* TOOD:  The error base is specific to the platform and should
 * be passed in through the platform data.
 */
#define PAINTBOX_ERROR_BASE		0x8000000000

/* TODO:  Figure out if there is a way to get this information from
 * the system.
 */
#define PAINTBOX_INPUT_ADDR_SIZE	43 /* bits */

/* TODO:  Determine appropriate values for airbrush. */
#define PAINTBOX_OUTPUT_ADDR_SIZE	32 /* bits */

/* Android APs usually use 4K pages.  This may change in future versions. */
#define PAINTBOX_PAGE_SIZE_BITMAP	SZ_4K

static void paintbox_airbrush_mfd_writel(struct device *dev, uint32_t val,
		unsigned int offset)
{
	int ret;

	ret = ipu_config_write(offset, sizeof(uint32_t), val);
	if (ret < 0)
		dev_err(dev, "write error\n");
}

static void paintbox_airbrush_mfd_writeq(struct device *dev, uint64_t val,
		unsigned int offset)
{
	uint32_t low, high;
	int ret;

	/* TODO(ahampson):  This should be moved into the MFD since there are
	 * likely other devices that have 64 bit registers on it.
	 */
	low = (uint32_t)(val & 0xFFFFFFFF);
	high = (uint32_t)(val >> 32);

	ret = ipu_config_write(offset, sizeof(uint32_t), low);
	if (ret < 0) {
		dev_err(dev, "write error\n");
		return;
	}

	ret = ipu_config_write(offset + sizeof(uint32_t), sizeof(uint32_t),
			high);
	if (ret < 0)
		dev_err(dev, "write error\n");
}

static uint32_t paintbox_airbrush_mfd_readl(struct device *dev,
		unsigned int offset)
{
	uint32_t val;
	int ret;

	ret = ipu_config_read(offset, sizeof(uint32_t), &val);
	if (ret < 0) {
		dev_err(dev, "read error\n");
		return 0;
	}

	return val;
}

static uint64_t paintbox_airbrush_mfd_readq(struct device *dev,
		unsigned int offset)
{
	uint32_t low, high;
	int ret;

	/* TODO(ahampson):  This should be moved into the MFD since there are
	 * likely other devices that have 64 bit registers on it.
	 */
	ret = ipu_config_read(offset, sizeof(uint32_t), &low);
	if (ret < 0) {
		dev_err(dev, "read error\n");
		return 0;
	}

	ret = ipu_config_read(offset + sizeof(uint32_t), sizeof(uint32_t),
			&high);
	if (ret < 0) {
		dev_err(dev, "read error\n");
		return 0;
	}

	return (((uint64_t)high) << 32) | low;
}

static int paintbox_airbrush_mfd_alloc(struct device *dev, size_t size,
		struct paintbox_shared_buffer *shared_buffer)
{
	struct paintbox_airbrush_mfd_data *dev_data = dev_get_drvdata(dev);

	shared_buffer->host_vaddr = dma_alloc_coherent(dev_data->dma_dev, size,
			&shared_buffer->host_dma_addr, GFP_KERNEL);

	if (!shared_buffer->host_vaddr)
		return -ENOMEM;

	shared_buffer->size = size;
	shared_buffer->jqs_paddr = AIRBRUSH_DRAM_START_PADDR +
			dev_data->next_jqs_allocation;

	dev_data->next_jqs_allocation += size;

	return 0;
}

void paintbox_airbrush_mfd_free(struct device *dev,
		struct paintbox_shared_buffer *shared_buffer)
{
	struct paintbox_airbrush_mfd_data *dev_data = dev_get_drvdata(dev);

	dma_free_coherent(dev_data->dma_dev, shared_buffer->size,
			shared_buffer->host_vaddr,
			shared_buffer->host_dma_addr);
}

void paintbox_airbrush_mfd_sync(struct device *dev,
		struct paintbox_shared_buffer *shared_buffer, uint32_t offset,
		size_t size, enum dma_data_direction direction)
{
	struct dma_element_t desc;
	dma_addr_t dma_addr = offset + shared_buffer->host_dma_addr;
	dma_addr_t jqs_addr = offset + shared_buffer->jqs_paddr;

	desc.len = size;

	if (direction == DMA_TO_DEVICE) {
		desc.src_addr = (uint32_t)(dma_addr & 0xFFFFFFFF);
		desc.src_u_addr = (uint32_t)(dma_addr >> 32);

		desc.dst_addr = (uint32_t)(jqs_addr & 0xFFFFFFFF);
		desc.dst_u_addr = (uint32_t)(jqs_addr >> 32);
	} else {
		desc.dst_addr = (uint32_t)(dma_addr & 0xFFFFFFFF);
		desc.dst_u_addr = (uint32_t)(dma_addr >> 32);

		desc.src_addr = (uint32_t)(jqs_addr & 0xFFFFFFFF);
		desc.src_u_addr = (uint32_t)(jqs_addr >> 32);
	}

	desc.chan = 0;

	(void)dma_sblk_start(0, direction, &desc);
}

static struct device *paintbox_airbrush_mfd_get_dma_device(struct device *dev)
{
	struct paintbox_airbrush_mfd_data *dev_data = dev_get_drvdata(dev);

	return dev_data->dma_dev;
}

static irqreturn_t paintbox_airbrush_mfd_interrupt(int irq, void *arg)
{
	struct paintbox_airbrush_mfd_data *dev_data =
		(struct paintbox_airbrush_mfd_data *)arg;
	uint32_t status, val;
	int ret;

	ret = ipu_config_read(IPU_CSR_APB_OFFSET + IPU_ISR, sizeof(uint32_t),
			&status);
	if (ret < 0)
		return ret;

	if (status == 0)
		return IRQ_NONE;

	/* Read the JQS to SYS doorbell register to acknowledge the JQS
	 * interrupt.
	 */
	ret = ipu_config_read(IPU_CSR_JQS_OFFSET + JQS_SYS_DBL,
			sizeof(uint32_t), &val);
	if (ret < 0)
		dev_err(dev_data->dev, "read error\n");

	paintbox_bus_dispatch_irq(dev_data->bus, status);

	/* Send an acknowledgment interrupt to the JQS.
	 * TODO(ahampson):  This may need to be changed in the future to
	 * contain a mask of acknowledged interrupts.  In the interim, writing
	 * any value will do.
	 */
	ret = ipu_config_write(IPU_CSR_JQS_OFFSET + SYS_JQS_DBL,
			sizeof(uint32_t), 1);
	if (ret < 0)
		dev_err(dev_data->dev, "write error\n");

	return ret;
}

static void paintbox_set_platform_data(struct platform_device *pdev,
		struct paintbox_pdata *pdata)
{
	pdata->page_size_bitmap = PAINTBOX_PAGE_SIZE_BITMAP;
	pdata->input_address_size = PAINTBOX_INPUT_ADDR_SIZE;
	pdata->output_address_size = PAINTBOX_OUTPUT_ADDR_SIZE;
	pdata->dma_base = PAINTBOX_IOVA_START;
	pdata->dma_size = PAINTBOX_IOVA_SIZE;

	/* TODO(ahampson):  Figure out how to get the hardware ID from the MFD
	 * if it is possible.  Otherwise we will need a lookup table and use the
	 * the IPU_VERSION register to reconcile it.
	 */
	pdata->hardware_id = 11;
}

static void paintbox_set_bus_ops(struct paintbox_bus_ops *ops)
{
	ops->write32 = &paintbox_airbrush_mfd_writel;
	ops->write64 = &paintbox_airbrush_mfd_writeq;
	ops->read32 = &paintbox_airbrush_mfd_readl;
	ops->read64 = &paintbox_airbrush_mfd_readq;
	ops->alloc = &paintbox_airbrush_mfd_alloc;
	ops->free = &paintbox_airbrush_mfd_free;
	ops->sync = &paintbox_airbrush_mfd_sync;
	ops->get_dma_device = &paintbox_airbrush_mfd_get_dma_device;
}

static int paintbox_airbrush_mfd_probe(struct platform_device *pdev)
{
	struct paintbox_airbrush_mfd_data *dev_data;
	int ret, irq_index;

	dev_data = devm_kzalloc(&pdev->dev, sizeof(*dev_data), GFP_KERNEL);
	if (dev_data == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev_data);

	dev_data->dev = &pdev->dev;

	paintbox_set_platform_data(pdev, &dev_data->pdata);
	paintbox_set_bus_ops(&dev_data->ops);

	for (irq_index = 0; irq_index < platform_irq_count(pdev); irq_index++) {
		int irq = platform_get_irq(pdev, irq_index);

		if (irq < 0) {
			dev_err(&pdev->dev, "platform_get_irq failed\n");
			return -ENODEV;
		}

		ret = devm_request_irq(&pdev->dev, irq,
				paintbox_airbrush_mfd_interrupt, IRQF_ONESHOT,
				dev_name(&pdev->dev), dev_data);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request irq\n");
			return ret;
		}
	}

	if (iommu_present(pdev->dev.parent->bus)) {
		dev_data->dma_dev = pdev->dev.parent;
	} else {
		arch_setup_dma_ops(&pdev->dev, 0, 0, NULL,
				false /* coherent */);
		dev_data->dma_dev = &pdev->dev;
	}

	ret = paintbox_bus_initialize(&pdev->dev, &dev_data->ops,
			&dev_data->pdata, &dev_data->bus);
	if (ret < 0)
		return ret;

#ifdef CONFIG_PAINTBOX_IOMMU
	ret = paintbox_bus_device_register(dev_data->bus, "paintbox-iommu",
			PAINTBOX_DEVICE_TYPE_IOMMU);
	if (ret < 0) {
		dev_err(&pdev->dev, "IOMMU device register failed, ret %d\n",
				ret);
		goto err_deinitialize_bus;
	}
#endif

	ret = paintbox_bus_device_register(dev_data->bus, "paintbox-ipu",
			PAINTBOX_DEVICE_TYPE_IPU);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"IPU device register failed, ret %d\n", ret);
		goto err_deinitialize_bus;
	}

	return 0;

err_deinitialize_bus:
	paintbox_bus_deinitialize(dev_data->bus);

	return ret;
}

static int paintbox_airbrush_mfd_remove(struct platform_device *pdev)
{
	struct paintbox_airbrush_mfd_data *dev_data =
			platform_get_drvdata(pdev);

	paintbox_bus_deinitialize(dev_data->bus);
	dev_data->bus = NULL;

	return 0;
}

static struct platform_driver paintbox_airbrush_mfd_driver = {
	.driver = {
		.name	= DRV_NAME_ABC_PCIE_IPU,
	},
	.probe		= paintbox_airbrush_mfd_probe,
	.remove		= paintbox_airbrush_mfd_remove,
};

module_platform_driver(paintbox_airbrush_mfd_driver);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Paintbox Airbrush MFD Driver");

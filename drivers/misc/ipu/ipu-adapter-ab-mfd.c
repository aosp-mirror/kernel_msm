/*
 * Airbrush MFD adapter driver for the Paintbox programmable IPU
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
#include <linux/ab-dram.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "ipu-adapter.h"
#include "ipu-core-jqs-msg-transport.h"
#include "ipu-regs.h"

struct ipu_adapter_ab_mfd_data {
	struct paintbox_bus *bus;
	struct device *dev;

	/* Depending on the IOMMU configuration the board the IPU may need
	 * to use MFD parent's device for mapping DMA buffers.  Otherwise, the
	 * dma device can be set to our own device and use SWIOTLB to map
	 * buffers.
	 */
	struct device *dma_dev;
	struct paintbox_pdata pdata;
	struct paintbox_bus_ops ops;
	struct completion dma_completion;
	struct mutex sync_lock;
};

/* Paintbox IO virtual address space bounds
 * TODO(b/115432213):  These are place holder values.  I need to figure out the
 * correct value for these.  This comes out to 512MB right now.
 */
#define PAINTBOX_IOVA_START		0x20000000
#define PAINTBOX_IOVA_SIZE		0x40000000

/* TODO(b/115433779):  Figure out if there is a way to get this information from
 * the system.
 */
#define PAINTBOX_INPUT_ADDR_SIZE	43 /* bits */

/* TODO(b/115433779):  Determine appropriate values for airbrush. */
#define PAINTBOX_OUTPUT_ADDR_SIZE	32 /* bits */

/* Android APs usually use 4K pages.  This may change in future versions. */
#define PAINTBOX_PAGE_SIZE_BITMAP	SZ_4K

static int ipu_adapter_ab_mfd_dma_callback(uint8_t chan,
		enum dma_data_direction dir, enum abc_dma_trans_status status);

static void ipu_adapter_ab_mfd_writel(struct device *dev, uint32_t val,
		unsigned int offset)
{
	int ret;

	ret = ipu_config_write(offset, sizeof(uint32_t), val);

	/* TODO(b/114760293):  An error writing an IPU register should be
	 * considered a catastrophic failure.  The IPU stack should cancel any
	 * running jobs and reset the IPU.
	 */
	if (ret < 0)
		dev_err(dev, "%s: write error offset %u val 0x%08x, ret %d\n",
				__func__, offset, val, ret);

}

static void ipu_adapter_ab_mfd_writeq(struct device *dev, uint64_t val,
		unsigned int offset)
{
	ipu_adapter_ab_mfd_writel(dev, (uint32_t)(val & 0xFFFFFFFF), offset);
	ipu_adapter_ab_mfd_writel(dev, (uint32_t)(val >> 32), offset +
			sizeof(uint32_t));
}

static uint32_t ipu_adapter_ab_mfd_readl(struct device *dev,
		unsigned int offset)
{
	uint32_t val;
	int ret;

	ret = ipu_config_read(offset, sizeof(uint32_t), &val);
	if (ret < 0) {
		/* TODO(b/114760293):  An error writing an IPU register should
		 * be considered a catastrophic failure.  The IPU stack should
		 * cancel any running jobs and reset the IPU.
		 */
		dev_err(dev, "%s: read error offset %u val 0x%08x, ret %d\n",
				__func__, offset, val, ret);
		return 0;
	}

	return val;
}

static uint64_t ipu_adapter_ab_mfd_readq(struct device *dev,
		unsigned int offset)
{
	uint32_t low, high;

	low = ipu_adapter_ab_mfd_readl(dev, offset);
	high = ipu_adapter_ab_mfd_readl(dev, offset + sizeof(uint32_t));

	return (((uint64_t)high) << 32) | low;
}

static int ipu_adapter_ab_mfd_alloc(struct device *dev, size_t size,
		struct paintbox_shared_buffer *shared_buffer)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);
	struct dma_buf *ab_dram_dmabuf;

	shared_buffer->host_vaddr = dma_alloc_coherent(dev_data->dma_dev, size,
			&shared_buffer->host_dma_addr, GFP_KERNEL);

	if (!shared_buffer->host_vaddr)
		return -ENOMEM;

	ab_dram_dmabuf = ab_dram_alloc_dma_buf_kernel(size);
	if (IS_ERR(ab_dram_dmabuf))
		return PTR_ERR(ab_dram_dmabuf);

	shared_buffer->ab_dram_dma_buf = ab_dram_dmabuf;

	shared_buffer->size = size;
	shared_buffer->jqs_paddr = ab_dram_get_dma_buf_paddr(ab_dram_dmabuf);

	return 0;
}

void ipu_adapter_ab_mfd_free(struct device *dev,
		struct paintbox_shared_buffer *shared_buffer)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);

	if (shared_buffer->ab_dram_dma_buf)
		ab_dram_free_dma_buf_kernel(shared_buffer->ab_dram_dma_buf);

	if (shared_buffer->host_vaddr)
		dma_free_coherent(dev_data->dma_dev, shared_buffer->size,
				shared_buffer->host_vaddr,
				shared_buffer->host_dma_addr);
}

static struct device *ipu_adapter_ab_mfd_get_dma_device(struct device *dev)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);

	return dev_data->dma_dev;
}

/* TODO(b/115431811):  This function should be split into two helper
 * functions, a PIO version and a DMA version.  The decision on whether or not
 * use the PIO version or DMA version will depend on the size of the transfer.
 */
void ipu_adapter_ab_mfd_sync(struct device *dev,
		struct paintbox_shared_buffer *shared_buffer, uint32_t offset,
		size_t size, enum dma_data_direction direction)
{
	struct ipu_adapter_ab_mfd_data *dev_data = dev_get_drvdata(dev);
	struct dma_element_t desc;
	dma_addr_t dma_addr = offset + shared_buffer->host_dma_addr;
	dma_addr_t jqs_addr = offset + shared_buffer->jqs_paddr;

	desc.len = size;

	mutex_lock(&dev_data->sync_lock);

	/* TODO(b/115431813):  The Endpoint DMA interface needs to be
	 * cleaned up so that it presents a synchronous interface to kernel
	 * clients or there needs to be mechanism to reserve a DMA channel to
	 * the IPU.
	 */
	(void)abc_reg_dma_irq_callback(&ipu_adapter_ab_mfd_dma_callback, 0);

	if (direction == DMA_TO_DEVICE) {
		dev_dbg(dev, "%s: va %p da %pad -> airbrush pa %pad sz %zu\n",
				__func__, shared_buffer->host_vaddr,
				&shared_buffer->host_dma_addr,
				&shared_buffer->jqs_paddr, size);

		desc.src_addr = (uint32_t)(dma_addr & 0xFFFFFFFF);
		desc.src_u_addr = (uint32_t)(dma_addr >> 32);

		desc.dst_addr = (uint32_t)(jqs_addr & 0xFFFFFFFF);
		desc.dst_u_addr = (uint32_t)(jqs_addr >> 32);
	} else {
		dev_dbg(dev, "%s: airbrush pa %pad -> va %p da %pad sz %zu\n",
				__func__, &shared_buffer->jqs_paddr,
				shared_buffer->host_vaddr,
				&shared_buffer->host_dma_addr, size);

		desc.dst_addr = (uint32_t)(dma_addr & 0xFFFFFFFF);
		desc.dst_u_addr = (uint32_t)(dma_addr >> 32);

		desc.src_addr = (uint32_t)(jqs_addr & 0xFFFFFFFF);
		desc.src_u_addr = (uint32_t)(jqs_addr >> 32);
	}

	desc.chan = 0;

	reinit_completion(&dev_data->dma_completion);

	/* The DMA code in the ABC MFD driver currently does not fail. */
	(void)dma_sblk_start(0, direction, &desc);

	wait_for_completion(&dev_data->dma_completion);

	/* TODO(b/115431813):  Right now we are only holding the callback
	 * for the duration of the sync.  This should be revisted once the DMA
	 * driver interface is cleaned up.
	 */
	(void)abc_reg_dma_irq_callback(NULL, 0);

	mutex_unlock(&dev_data->sync_lock);
}

/* TODO(b/115431813):  This can be eliminated once the DMA interface is
 * cleaned up
 */
static struct ipu_adapter_ab_mfd_data *g_dev_data;

static int ipu_adapter_ab_mfd_dma_callback(uint8_t chan,
		enum dma_data_direction dir, enum abc_dma_trans_status status)
{
	struct ipu_adapter_ab_mfd_data *dev_data = g_dev_data;

	complete(&dev_data->dma_completion);

	return 0;
}

static irqreturn_t ipu_adapter_ab_mfd_interrupt(int irq, void *arg)
{
	struct ipu_adapter_ab_mfd_data *dev_data =
		(struct ipu_adapter_ab_mfd_data *)arg;

	return ipu_core_jqs_msg_transport_interrupt(dev_data->bus);
}

static void ipu_adapter_ab_mfd_set_platform_data(struct platform_device *pdev,
		struct paintbox_pdata *pdata)
{
	pdata->page_size_bitmap = PAINTBOX_PAGE_SIZE_BITMAP;
	pdata->input_address_size = PAINTBOX_INPUT_ADDR_SIZE;
	pdata->output_address_size = PAINTBOX_OUTPUT_ADDR_SIZE;
	pdata->dma_base = PAINTBOX_IOVA_START;
	pdata->dma_size = PAINTBOX_IOVA_SIZE;

	/* TODO(b/115434021):  Figure out how to get the hardware ID from the
	 * MFD if it is possible.  Otherwise we will need a lookup table and
	 * use the the IPU_VERSION register to reconcile it.
	 */
	pdata->hardware_id = 11;
}

static void ipu_adapter_ab_mfd_set_bus_ops(struct paintbox_bus_ops *ops)
{
	ops->write32 = &ipu_adapter_ab_mfd_writel;
	ops->write64 = &ipu_adapter_ab_mfd_writeq;
	ops->read32 = &ipu_adapter_ab_mfd_readl;
	ops->read64 = &ipu_adapter_ab_mfd_readq;
	ops->alloc = &ipu_adapter_ab_mfd_alloc;
	ops->free = &ipu_adapter_ab_mfd_free;
	ops->sync = &ipu_adapter_ab_mfd_sync;
	ops->get_dma_device = &ipu_adapter_ab_mfd_get_dma_device;
}

static int ipu_adapter_ab_mfd_probe(struct platform_device *pdev)
{
	struct ipu_adapter_ab_mfd_data *dev_data;
	int ret, irq_index;

	dev_data = devm_kzalloc(&pdev->dev, sizeof(*dev_data), GFP_KERNEL);
	if (dev_data == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev_data);

	dev_data->dev = &pdev->dev;

	ipu_adapter_ab_mfd_set_platform_data(pdev, &dev_data->pdata);
	ipu_adapter_ab_mfd_set_bus_ops(&dev_data->ops);

	mutex_init(&dev_data->sync_lock);

	/* TODO(ahampson, pztang):  This can be removed once the DMA interface
	 * clean up is done. b/115431813
	 */
	g_dev_data = dev_data;

	init_completion(&dev_data->dma_completion);

	for (irq_index = 0; irq_index < platform_irq_count(pdev); irq_index++) {
		int irq = platform_get_irq(pdev, irq_index);

		if (irq < 0) {
			dev_err(&pdev->dev,
					"%s: platform_get_irq failed, err %d\n",
					__func__, irq);
			return -ENODEV;
		}

		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
				ipu_adapter_ab_mfd_interrupt, IRQF_ONESHOT,
				dev_name(&pdev->dev), dev_data);
		if (ret < 0) {
			dev_err(&pdev->dev,
					"%s: failed to request irq, err %d\n",
					__func__, ret);
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

	ret = ipu_bus_initialize(&pdev->dev, &dev_data->ops,
			&dev_data->pdata, &dev_data->bus);
	if (ret < 0)
		return ret;

#if IS_ENABLED(CONFIG_PAINTBOX_AIRBRUSH_IOMMU)
	ret = ipu_bus_device_register(dev_data->bus, "paintbox-iommu",
			PAINTBOX_DEVICE_TYPE_IOMMU);
	if (ret < 0) {
		dev_err(&pdev->dev, "IOMMU device register failed, ret %d\n",
				ret);
		goto err_deinitialize_bus;
	}
#endif

	ret = ipu_bus_device_register(dev_data->bus, "paintbox-ipu",
			PAINTBOX_DEVICE_TYPE_IPU);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"IPU device register failed, ret %d\n", ret);
		goto err_deinitialize_bus;
	}

	return 0;

err_deinitialize_bus:
	ipu_bus_deinitialize(dev_data->bus);

	return ret;
}

static int ipu_adapter_ab_mfd_remove(struct platform_device *pdev)
{
	struct ipu_adapter_ab_mfd_data *dev_data =
			platform_get_drvdata(pdev);

	ipu_bus_deinitialize(dev_data->bus);
	dev_data->bus = NULL;

	return 0;
}

static struct platform_driver ipu_adapter_ab_mfd_driver = {
	.driver = {
		.name	= DRV_NAME_ABC_PCIE_IPU,
	},
	.probe		= ipu_adapter_ab_mfd_probe,
	.remove		= ipu_adapter_ab_mfd_remove,
};

module_platform_driver(ipu_adapter_ab_mfd_driver);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPU Airbrush MFD Adapter Driver");

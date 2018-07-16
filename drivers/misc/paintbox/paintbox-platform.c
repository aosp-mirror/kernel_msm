/*
 * Platform bus driver for the Paintbox programmable IPU
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

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "paintbox-bus-impl.h"
#include "paintbox-regs.h"

struct paintbox_platform_bus_data {
	struct paintbox_bus *bus;
	struct device *dev;
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	void __iomem *airbrush_dram_base;
#endif
	void __iomem *reg_base;
	unsigned int irq;
	/* TODO(ahampson, rogerwolff) This should be replaced with
	 * interactions with the Airbrush DRAM manager.  For now, this
	 * this is just a bump allocator that does not expect any memory
	 * to be freed.
	 */
	uint32_t next_allocation_offset;

	struct paintbox_pdata pdata;
	struct paintbox_bus_ops ops;
};

/* Paintbox IO virtual address space bounds
 * TODO:  These are place holder values.  I need to figure out the
 * correct value for these.  This comes out to 512MB right now.
 */
#define PAINTBOX_IOVA_START		0x20000000
#define PAINTBOX_IOVA_SIZE		0x40000000

/* TOOD:  The error base is specific to the platform and should
 * be passed in through the platform data.
 */
#define PAINTBOX_ERROR_BASE		0x8000000000

/* TODO:  Figure out if there is a way to get this information from
 * the system.
 */
#define PAINTBOX_INPUT_ADDR_SIZE	43 /* bits */

/* TODO:  This will need to be configurable.  The output address size
 * on Easel will be 32 bits but on a normal system it will be 40 bits.
 */
#define PAINTBOX_OUTPUT_ADDR_SIZE	32 /* bits */

/* Easel will use 4K pages.  This may change in future versions. */
#define PAINTBOX_PAGE_SIZE_BITMAP	SZ_4K

static void paintbox_platform_bus_writel(struct device *dev, uint32_t val,
		unsigned int offset)
{
	struct paintbox_platform_bus_data *dev_data = dev_get_drvdata(dev);

	__iowmb();
	writel_relaxed(val, dev_data->reg_base + offset);
}

static void paintbox_platform_bus_writeq(struct device *dev, uint64_t val,
		unsigned int offset)
{
	struct paintbox_platform_bus_data *dev_data = dev_get_drvdata(dev);

	__iowmb();
	writeq_relaxed(val, dev_data->reg_base + offset);
}

static uint32_t paintbox_platform_bus_readl(struct device *dev,
		unsigned int offset)
{
	struct paintbox_platform_bus_data *dev_data = dev_get_drvdata(dev);
	uint32_t val;

	val = readl_relaxed(dev_data->reg_base + offset);

	__iormb();

	return val;
}

static uint64_t paintbox_platform_bus_readq(struct device *dev,
		unsigned int offset)
{
	struct paintbox_platform_bus_data *dev_data = dev_get_drvdata(dev);
	uint64_t value;

	val = readq_relaxed(dev_data->reg_base + offset);
	__iormb();

	return val;
}

/* TODO(b/74122875):  There is no Airbrush DRAM manager yet so we just write the
 * firmware image to the base of Airbrush DRAM.
 */
#define AIRBRUSH_DRAM_START_PADDR 0x20000000

int paintbox_platform_bus_alloc(struct device *dev, size_t size,
		struct paintbox_shared_buffer *shared_buffer)
{
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	struct paintbox_platform_bus_data *dev_data = dev_get_drvdata(dev);

	shared_buffer->host_vaddr = dev_data->airbrush_dram_base +
		dev_data->next_allocation_offset;
	shared_buffer->size = size;
	shared_buffer->host_dma_addr = 0;
	shared_buffer->jqs_paddr = AIRBRUSH_DRAM_START_PADDR +
		dev_data->next_allocation_offset;

	dev_data->next_allocation_offset += size;

	return 0;
#else
	/* TODO(ahampson):  Implement for AXI variant. */
	return -ENOMEM;
#endif
}

void paintbox_platform_bus_free(struct device *dev,
		struct paintbox_shared_buffer *alloc)
{
	/* nop. You can't free with this allocator. */
}

void paintbox_platform_bus_sync(struct device *dev,
		struct paintbox_shared_buffer *alloc, uint32_t offset,
		size_t size, enum dma_data_direction direction)
{
	/* Since we are mapping the shared memory region as IO memory it is
	 * configured as write combine so it should write through the cache
	 * to memory. Therefore, we do not need to do anything explicit to
	 * make changes visible.
	 */
}

static struct device *paintbox_platform_bus_get_dma_device(struct device *dev)
{
	/* The IOMMU is not enabled on the QEMU kernel so the DMA device to use
	 * is the paintbox platform bus device.
	 */
	return dev;
}

static irqreturn_t paintbox_platform_bus_interrupt(int irq, void *arg)
{
	struct paintbox_platform_bus_data *dev_data =
			(struct paintbox_platform_bus_data *)arg;
	irqreturn_t ret;
	uint32_t status;

	status = readl_relaxed(dev_data->reg_base + IPU_CSR_APB_OFFSET +
			IPU_ISR);
	__iormb();

	if (status == 0)
		return IRQ_NONE;

	/* Read the JQS to SYS doorbell register to acknowledge the JQS
	 * interrupt.
	 */
	readl_relaxed(dev_data->reg_base + IPU_CSR_JQS_OFFSET + JQS_SYS_DBL);
	__iormb();

	ret = paintbox_bus_dispatch_irq(dev_data->bus, status);

	/* Send an acknowledgment interrupt to the JQS.
	 * TODO(ahampson):  This may need to be changed in the future to
	 * contain a mask of acknowledged interrupts.  In the interim, writing
	 * any value will do.
	 */
	__iowmb();
	writel_relaxed(1, dev_data->reg_base + IPU_CSR_JQS_OFFSET +
			SYS_JQS_DBL);

	return ret;
}

static int paintbox_get_hardware_id(struct device *dev, uint32_t *hardware_id)
{
	uint64_t data;
	int ret;

	ret = of_property_read_u64(dev->of_node, "hardware-id", &data);
	if (ret < 0) {
		dev_err(dev, "%s: hardware-id not set in device tree, err %d\n",
				__func__, ret);
		return ret;
	}

	*hardware_id = (uint32_t)data;

	return 0;
}

static int paintbox_set_platform_data(struct platform_device *pdev,
		struct paintbox_pdata *pdata)
{
	int ret;

	pdata->page_size_bitmap = PAINTBOX_PAGE_SIZE_BITMAP;
	pdata->input_address_size = PAINTBOX_INPUT_ADDR_SIZE;
	pdata->output_address_size = PAINTBOX_OUTPUT_ADDR_SIZE;
	pdata->dma_base = PAINTBOX_IOVA_START;
	pdata->dma_size = PAINTBOX_IOVA_SIZE;

	ret = paintbox_get_hardware_id(&pdev->dev, &pdata->hardware_id);
	if (ret < 0)
		return ret;

	return 0;
}

static void paintbox_set_bus_ops(struct paintbox_bus_ops *ops)
{
	ops->write32 = &paintbox_platform_bus_writel;
	ops->write64 = &paintbox_platform_bus_writeq;
	ops->read32 = &paintbox_platform_bus_readl;
	ops->read64 = &paintbox_platform_bus_readq;
	ops->alloc = &paintbox_platform_bus_alloc;
	ops->free = &paintbox_platform_bus_free;
	ops->sync = &paintbox_platform_bus_sync;
	ops->get_dma_device = &paintbox_platform_bus_get_dma_device;
}

static int paintbox_platform_bus_probe(struct platform_device *pdev)
{
	struct paintbox_platform_bus_data *dev_data;
	struct resource *r;
	int ret;

	dev_data = devm_kzalloc(&pdev->dev, sizeof(*dev_data), GFP_KERNEL);
	if (dev_data == NULL)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "platform_get_resource failed\n");
		return -ENODEV;
	}

	dev_data->reg_base = devm_ioremap(&pdev->dev, r->start,
			resource_size(r));
	if (dev_data->reg_base == NULL) {
		dev_err(&pdev->dev, "unable to remap MMIO\n");
		return -ENOMEM;
	}

#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	/* TODO(ahampson):  Under emulation, Airbrush DRAM is implemented as a
	 * shared memory file used by both the QEMU and Simulator processes.
	 * Initially, the shared memory will be exposed to the kernel as a
	 * I/O memory resource for the Paintbox driver.  This will be
	 * replaced at a later date with something more general as the Airbrush
	 * DRAM manager is brought up.
	 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	/* TODO(ahampson):  Do not error out if the Airbrush DRAM resource is
	 * not present.  This is a temporary measure while the Airbrush DRAM
	 * emulation is brought up.  Once everything is in place this can
	 * become mandatory.
	 */
	if (r) {
		dev_data->airbrush_dram_base = devm_ioremap(&pdev->dev,
				r->start, resource_size(r));
		if (dev_data->airbrush_dram_base)
			dev_info(&pdev->dev,
					"Airbrush DRAM %pa - %pa mapped to 0x%p\n",
					&r->start, &r->end,
					dev_data->airbrush_dram_base);
	}
#endif

	dev_data->irq = platform_get_irq(pdev, 0);
	if (dev_data->irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq failed\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, dev_data->irq,
			paintbox_platform_bus_interrupt, IRQF_SHARED,
			dev_name(&pdev->dev), dev_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return ret;
	}

	platform_set_drvdata(pdev, dev_data);

	dev_data->dev = &pdev->dev;

	ret = paintbox_set_platform_data(pdev, &dev_data->pdata);
	if (ret < 0)
		return ret;

	paintbox_set_bus_ops(&dev_data->ops);

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
		dev_err(&pdev->dev, "IPU device register failed, ret %d\n",
				ret);
		goto err_deinitialize_bus;
	}

	return 0;

err_deinitialize_bus:
	paintbox_bus_deinitialize(dev_data->bus);
	dev_data->bus = NULL;

	return ret;
}

static int paintbox_platform_bus_remove(struct platform_device *pdev)
{
	struct paintbox_platform_bus_data *dev_data =
			platform_get_drvdata(pdev);

	paintbox_bus_deinitialize(dev_data->bus);
	dev_data->bus = NULL;

	return 0;
}

static const struct of_device_id paintbox_of_match[] = {
	{ .compatible = "google,paintbox", },
	{},
};
MODULE_DEVICE_TABLE(of, paintbox_of_match);

static struct platform_driver paintbox_driver = {
	.probe		= paintbox_platform_bus_probe,
	.remove		= paintbox_platform_bus_remove,
	.driver = {
		.name = "paintbox",
		.of_match_table = paintbox_of_match,
	}
};
module_platform_driver(paintbox_driver);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Paintbox Platform Bus Driver");

/*
 * Virtual bus support for the Paintbox programmable IPU
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
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "paintbox-bus.h"
#include "paintbox-bus-common.h"
#include "paintbox-bus-impl.h"
#include "paintbox-bus-jqs.h"

struct bus_type paintbox_bus_type;


void paintbox_writel(struct device *dev, uint32_t val, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	paintbox_bus_writel(pb_dev->bus, val, offset);
}

void paintbox_writeq(struct device *dev, uint64_t val, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	paintbox_bus_writeq(pb_dev->bus, val, offset);
}

uint32_t paintbox_readl(struct device *dev, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return paintbox_bus_readl(pb_dev->bus, offset);
}

uint64_t paintbox_readq(struct device *dev, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return paintbox_bus_readq(pb_dev->bus, offset);
}

struct device *paintbox_bus_get_iommu_device(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return &bus->devices[PAINTBOX_DEVICE_TYPE_IOMMU]->dev;
}

struct device *paintbox_get_dma_device(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return bus->ops->get_dma_device(bus->parent_dev);
}

struct dentry *paintbox_bus_get_debug_root(struct device *dev)
{
#ifdef CONFIG_PAINTBOX_DEBUG
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return bus->debug_root;
#else
	return NULL;
#endif
}

struct iommu_group *paintbox_get_device_group(struct device *dev)
{
	struct paintbox_device *pb_dev;
	struct iommu_group *group;

	if (dev->bus != &paintbox_bus_type)
		return ERR_PTR(-EINVAL);

	group = iommu_group_get(dev);
	if (group)
		return group;

	pb_dev = to_paintbox_device(dev);

	if (pb_dev->bus->group)
		return pb_dev->bus->group;

	group = iommu_group_alloc();
	if (IS_ERR(group))
		return NULL;

	iommu_group_set_name(group, "paintbox");

	pb_dev->bus->group = group;

	return group;
}

void paintbox_request_irq(struct device *dev, pb_irq_handler_t handler,
		uint32_t irq_mask)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	pb_dev->handler = handler;
	pb_dev->irq_mask = irq_mask;
}

void paintbox_release_irq(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	pb_dev->handler = NULL;
	pb_dev->irq_mask = 0;
}

void paintbox_set_device_ops(struct device *dev,
		const struct paintbox_device_ops *dev_ops)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	pb_dev->dev_ops = dev_ops;
}

irqreturn_t paintbox_bus_dispatch_irq(struct paintbox_bus *bus, uint32_t status)
{
	irqreturn_t ret = IRQ_NONE;
	unsigned int i;

	for (i = 0; i < PAINTBOX_DEVICE_TYPE_COUNT; i++) {
		struct paintbox_device *pb_dev = bus->devices[i];

		if (!(pb_dev && pb_dev->handler))
			continue;

		if (status & pb_dev->irq_mask)
			ret |= pb_dev->handler(&pb_dev->dev, status);
	}

	return ret;
}

static void paintbox_bus_device_release(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	kfree(pb_dev);
}

int paintbox_bus_device_register(struct paintbox_bus *bus, const char *name,
		enum paintbox_device_type type)
{
	struct paintbox_device *pb_dev;
	struct paintbox_pdata *pdata = bus->pdata;
	int ret;

	WARN_ON(type >= PAINTBOX_DEVICE_TYPE_COUNT);
	WARN_ON(bus->devices[type]);

	pb_dev = kzalloc(sizeof(*pb_dev), GFP_KERNEL);
	if (pb_dev == NULL)
		return -ENOMEM;

	pb_dev->dev.bus = &paintbox_bus_type;
	pb_dev->dev.platform_data = pdata;

	/* TODO:  Look for a better way to do this.  Normally it is
	 * done in OF but since we are manually constructing the IOMMU device we
	 * need to do it here.
	 */
	pb_dev->dev.coherent_dma_mask = DMA_BIT_MASK(pdata->input_address_size);
	pb_dev->dev.dma_mask = &pb_dev->dev.coherent_dma_mask;
	pb_dev->dev.release = &paintbox_bus_device_release;
	pb_dev->bus = bus;

#ifdef CONFIG_PAINTBOX_DEBUG
	/* TODO(ahampson): Currently we use the bus's debug root for the IPU
	 * and IOMMU devices.  In the future we might want to have separate
	 * debug roots for each device.
	 */
	pb_dev->debug_root = bus->debug_root;
#endif

	dev_set_name(&pb_dev->dev, name);

	ret = device_register(&pb_dev->dev);
	if (ret < 0) {
		dev_err(bus->parent_dev,
				"failed to register device %s, ret %d\n",
				name, ret);
		put_device(&pb_dev->dev);
		kfree(pb_dev);
		return ret;
	}

	bus->devices[type] = pb_dev;

	/* Clear the dma ops for the IOMMU device and setup the dma_ops for the
	 * IOMMU.  The arm64 dma map code will set up the swiotlb dma map for
	 * dma device.
	 */
	if (type == PAINTBOX_DEVICE_TYPE_IPU) {
		arch_setup_dma_ops(&pb_dev->dev, pdata->dma_base,
			pdata->dma_size,
			(struct iommu_ops *)paintbox_bus_type.iommu_ops,
			false /* coherent */);
	} else {
		arch_teardown_dma_ops(&pb_dev->dev);
		arch_setup_dma_ops(&pb_dev->dev, 0, 0, NULL,
				false /* coherent */);
	}

	return 0;
}

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_bus_debug_init(struct paintbox_bus *bus)
{
	bus->debug_root = debugfs_create_dir("paintbox", NULL);
	if (IS_ERR(bus->debug_root)) {
		dev_err(bus->parent_dev, "%s: err = %ld", __func__,
				PTR_ERR(bus->debug_root));
		return PTR_ERR(bus->debug_root);
	}

	return 0;
}
#endif

void paintbox_bus_notify_firmware_up(struct paintbox_bus *bus)
{
	unsigned int i;

	for (i = 0; i < PAINTBOX_DEVICE_TYPE_COUNT; i++) {
		struct paintbox_device *pb_dev = bus->devices[i];

		if (!pb_dev || !pb_dev->dev_ops ||
				!pb_dev->dev_ops->firmware_up)
			continue;

		pb_dev->dev_ops->firmware_up(&pb_dev->dev);
	}
}

void paintbox_bus_notify_firmware_down(struct paintbox_bus *bus)
{
	unsigned int i;

	for (i = 0; i < PAINTBOX_DEVICE_TYPE_COUNT; i++) {
		struct paintbox_device *pb_dev = bus->devices[i];

		if (!pb_dev || !pb_dev->dev_ops ||
					!pb_dev->dev_ops->firmware_down)
			continue;

		pb_dev->dev_ops->firmware_down(&pb_dev->dev);
	}
}

int paintbox_bus_initialize(struct device *parent_dev,
		struct paintbox_bus_ops *ops, struct paintbox_pdata *pdata,
		struct paintbox_bus **pb_bus)
{
	struct paintbox_bus *bus;
	int ret;

	bus = kzalloc(sizeof(struct paintbox_bus), GFP_KERNEL);
	if (bus == NULL)
		return -ENOMEM;

	bus->parent_dev = parent_dev;
	bus->ops = ops;
	bus->pdata = pdata;

#ifdef CONFIG_PAINTBOX_DEBUG
	ret = paintbox_bus_debug_init(bus);
	if (ret < 0)
		return ret;

	paintbox_bus_jqs_debug_init(bus);
#endif

	ret = paintbox_bus_jqs_enable_firmware(bus);
	if (ret < 0) {
		paintbox_bus_deinitialize(bus);
		return ret;
	}

	*pb_bus = bus;

	return 0;
}

void paintbox_bus_deinitialize(struct paintbox_bus *bus)
{
	unsigned int i;

	for (i = 0; i < PAINTBOX_DEVICE_TYPE_COUNT; i++) {
		struct paintbox_device *pb_dev = bus->devices[i];

		if (!pb_dev)
			continue;

		bus->devices[i] = NULL;

		device_unregister(&pb_dev->dev);
	}

	paintbox_bus_jqs_release(bus);

#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_bus_jqs_debug_remove(bus);
	debugfs_remove(bus->debug_root);
#endif

	kfree(bus);
}

/* The Linux IOMMU is designed around an IOMMU providing translation services to
 * all devices on a particular bus.  The Paintbox IOMMU is integrated into the
 * Paintbox IPU.  To make the Paintbox IOMMU fit within the Linux IOMMU
 * framework we will create a virtual bus between the core paintbox drver and
 * the IOMMU driver.
 */
static int paintbox_bus_match(struct device *dev, struct device_driver *drv)
{
	/* Just do a simple match based on the device and driver names */
	if (strcmp(dev_name(dev), drv->name) == 0)
		return 1;

	return 0;
}

struct bus_type paintbox_bus_type = {
	.name	= "paintbox",
	.match	= paintbox_bus_match,
};
EXPORT_SYMBOL(paintbox_bus_type);

static int __init paintbox_bus_init(void)
{
	return bus_register(&paintbox_bus_type);
}

postcore_initcall(paintbox_bus_init);

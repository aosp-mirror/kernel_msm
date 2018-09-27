/*
 * IPU core support for the Paintbox programmable IPU
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
#include <linux/ipu-core.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "ipu-adapter.h"
#include "ipu-core-internal.h"
#include "ipu-core-jqs.h"
#include "ipu-core-jqs-debug.h"
#include "ipu-core-jqs-msg-transport.h"

struct bus_type ipu_bus_type;

int ipu_alloc_queue(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return ipu_core_jqs_msg_transport_alloc_queue(pb_dev->bus);
}

void ipu_free_queue(struct device *dev, uint32_t q_id)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	ipu_core_jqs_msg_transport_free_queue(pb_dev->bus, q_id);
}

int ipu_user_write(struct device *dev, uint32_t q_id,
		const void __user *buf, size_t size)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return ipu_core_jqs_msg_transport_user_write(pb_dev->bus, q_id, buf,
			size);
}

ssize_t ipu_user_read(struct device *dev, uint32_t q_id, void __user *buf,
		size_t size)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return ipu_core_jqs_msg_transport_user_read(pb_dev->bus, q_id, buf,
			size);
}

int ipu_kernel_write(struct device *dev, const struct jqs_message *msg)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return ipu_core_jqs_msg_transport_kernel_write(pb_dev->bus, msg);

}

int ipu_kernel_write_sync(struct device *dev,
		const struct jqs_message *msg, struct jqs_message *rsp,
		size_t rsp_size)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return ipu_core_jqs_msg_transport_kernel_write_sync(pb_dev->bus, msg,
			rsp, rsp_size);
}

void ipu_writel(struct device *dev, uint32_t val, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	ipu_core_writel(pb_dev->bus, val, offset);
}

void ipu_writeq(struct device *dev, uint64_t val, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	ipu_core_writeq(pb_dev->bus, val, offset);
}

uint32_t ipu_readl(struct device *dev, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return ipu_core_readl(pb_dev->bus, offset);
}

uint64_t ipu_readq(struct device *dev, unsigned int offset)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	return ipu_core_readq(pb_dev->bus, offset);
}

struct device *ipu_get_iommu_device(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return &bus->devices[PAINTBOX_DEVICE_TYPE_IOMMU]->dev;
}

struct device *ipu_get_dma_device(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return bus->ops->get_dma_device(bus->parent_dev);
}

int ipu_alloc_memory(struct device *dev, size_t size,
		struct paintbox_shared_buffer *shared_buffer)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return ipu_core_memory_alloc(bus, size, shared_buffer);
}

void ipu_free_memory(struct device *dev,
			struct paintbox_shared_buffer *shared_buffer)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	ipu_core_memory_free(bus, shared_buffer);
}

struct dentry *ipu_get_debug_root(struct device *dev)
{
#if IS_ENABLED(CONFIG_IPU_DEBUG)
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return bus->debug_root;
#else
	return NULL;
#endif
}

struct iommu_group *ipu_get_device_group(struct device *dev)
{
	struct paintbox_device *pb_dev;
	struct iommu_group *group;

	if (dev->bus != &ipu_bus_type)
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

void ipu_set_device_ops(struct device *dev,
		const struct paintbox_device_ops *dev_ops)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	pb_dev->dev_ops = dev_ops;
}

bool ipu_is_jqs_ready(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);
	struct paintbox_bus *bus = pb_dev->bus;

	return ipu_core_jqs_is_ready(bus);
}

static void ipu_bus_device_release(struct device *dev)
{
	struct paintbox_device *pb_dev = to_paintbox_device(dev);

	kfree(pb_dev);
}

int ipu_bus_device_register(struct paintbox_bus *bus, const char *name,
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

	pb_dev->dev.bus = &ipu_bus_type;
	pb_dev->dev.platform_data = pdata;
	pb_dev->dev.parent = bus->parent_dev;

	/* TODO:  Look for a better way to do this.  Normally it is
	 * done in OF but since we are manually constructing the IOMMU device we
	 * need to do it here.
	 */
	pb_dev->dev.coherent_dma_mask = DMA_BIT_MASK(pdata->input_address_size);
	pb_dev->dev.dma_mask = &pb_dev->dev.coherent_dma_mask;
	pb_dev->dev.release = &ipu_bus_device_release;
	pb_dev->bus = bus;

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	/* TODO(b/115426798): Currently we use the bus's debug root for the IPU
	 * and IOMMU devices.  In the future we might want to have separate
	 * debug roots for each device.
	 */
	pb_dev->debug_root = bus->debug_root;
#endif

	device_initialize(&pb_dev->dev);

	dev_set_name(&pb_dev->dev, name);

	/* Add the new device to the IPU JQS power domain */
	ret = pm_genpd_add_device(&bus->gpd, &pb_dev->dev);
	if (ret < 0) {
		dev_err(bus->parent_dev,
				"failed to add device %s to %s power domain, ret %d\n",
				name, bus->gpd.name, ret);
		goto put_device;
	}

	ret = device_add(&pb_dev->dev);
	if (ret < 0) {
		dev_err(bus->parent_dev,
				"failed to register device %s, ret %d\n",
				name, ret);
		goto remove_from_power_domain;
	}

	bus->devices[type] = pb_dev;

	/* Clear the dma ops for the IOMMU device and setup the dma_ops for the
	 * IOMMU.  The arm64 dma map code will set up the swiotlb dma map for
	 * dma device.
	 */
	if (type == PAINTBOX_DEVICE_TYPE_IPU) {
		arch_setup_dma_ops(&pb_dev->dev, pdata->dma_base,
			pdata->dma_size,
			(struct iommu_ops *)ipu_bus_type.iommu_ops,
			false /* coherent */);
	} else {
		arch_teardown_dma_ops(&pb_dev->dev);
		arch_setup_dma_ops(&pb_dev->dev, 0, 0, NULL,
				false /* coherent */);
	}

	return 0;

remove_from_power_domain:
	pm_genpd_remove_device(&bus->gpd, &pb_dev->dev);
put_device:
	put_device(&pb_dev->dev);
	kfree(pb_dev);
	return ret;
}

#if IS_ENABLED(CONFIG_IPU_DEBUG)
int ipu_core_debug_init(struct paintbox_bus *bus)
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

void ipu_core_notify_firmware_up(struct paintbox_bus *bus)
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

void ipu_core_notify_firmware_down(struct paintbox_bus *bus)
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

int ipu_bus_initialize(struct device *parent_dev,
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

	spin_lock_init(&bus->irq_lock);

#if IS_ENABLED(CONFIG_IPU_DEBUG)
	ret = ipu_core_debug_init(bus);
	if (ret < 0)
		return ret;
#endif
	ipu_core_jqs_debug_init(bus);

	ret = ipu_core_jqs_init(bus);
	if (ret < 0) {
		ipu_bus_deinitialize(bus);
		return ret;
	}

	*pb_bus = bus;

	return 0;
}

void ipu_bus_deinitialize(struct paintbox_bus *bus)
{
	unsigned int i;

	for (i = 0; i < PAINTBOX_DEVICE_TYPE_COUNT; i++) {
		struct paintbox_device *pb_dev = bus->devices[i];

		if (!pb_dev)
			continue;

		bus->devices[i] = NULL;

		pm_genpd_remove_device(&bus->gpd, &pb_dev->dev);

		device_unregister(&pb_dev->dev);
	}

	ipu_core_jqs_remove(bus);

	ipu_core_jqs_debug_remove(bus);
#if IS_ENABLED(CONFIG_IPU_DEBUG)
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
static int ipu_bus_match(struct device *dev, struct device_driver *drv)
{
	/* Just do a simple match based on the device and driver names */
	if (strcmp(dev_name(dev), drv->name) == 0)
		return 1;
	return 0;
}

struct bus_type ipu_bus_type = {
	.name	= "paintbox",
	.match	= ipu_bus_match,
};
EXPORT_SYMBOL(ipu_bus_type);

static int __init ipu_bus_init(void)
{
	return bus_register(&ipu_bus_type);
}

postcore_initcall(ipu_bus_init);

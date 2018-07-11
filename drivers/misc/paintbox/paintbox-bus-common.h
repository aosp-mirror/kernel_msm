/*
 * Common bus support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_BUS_COMMON_H__
#define __PAINTBOX_BUS_COMMON_H__

#include <linux/iommu.h>
#include <linux/types.h>

#include "paintbox-bus.h"
#include "paintbox-bus-impl.h"

#define JQS_CACHE_LINE_SIZE 64

struct paintbox_device;

struct paintbox_bus {
	struct paintbox_device *devices[PAINTBOX_DEVICE_TYPE_COUNT];
	struct paintbox_bus_ops *ops;
	struct paintbox_pdata *pdata;
	struct device *parent_dev;
	struct iommu_group *group;
#ifdef CONFIG_PAINTBOX_DEBUG
	struct dentry *debug_root;
#endif
	enum paintbox_jqs_status fw_status;

	const struct firmware *fw;
	struct paintbox_shared_buffer fw_shared_buffer;

#ifdef CONFIG_PAINTBOX_DEBUG
	struct dentry *fw_enable_dentry;
#endif
};

struct paintbox_device {
	struct device dev;
	struct paintbox_bus *bus;
	enum paintbox_device_type type;
	pb_irq_handler_t handler;
	const struct paintbox_device_ops *dev_ops;
	uint32_t irq_mask;
#ifdef CONFIG_PAINTBOX_DEBUG
	struct dentry *debug_root;
#endif
};

static inline struct paintbox_device *to_paintbox_device(struct device *dev)
{
	return container_of(dev, struct paintbox_device, dev);
}

static inline void paintbox_bus_writel(struct paintbox_bus *bus, uint32_t val,
		unsigned int offset)
{
	bus->ops->write32(bus->parent_dev, val, offset);
}

static inline void paintbox_bus_writeq(struct paintbox_bus *bus, uint64_t val,
		unsigned int offset)
{
	bus->ops->write64(bus->parent_dev, val, offset);
}

static inline uint32_t paintbox_bus_readl(struct paintbox_bus *bus,
		unsigned int offset)
{
	return bus->ops->read32(bus->parent_dev, offset);
}

static inline uint64_t paintbox_bus_readq(struct paintbox_bus *bus,
		unsigned int offset)
{
	return bus->ops->read64(bus->parent_dev, offset);
}

static inline int paintbox_bus_alloc(struct paintbox_bus *bus, size_t size,
		struct paintbox_shared_buffer *shared_buffer)
{
	memset(shared_buffer, 0, sizeof(*shared_buffer));
	/*
	 * JQS caches data JQS_CACHE_LINE_SIZE bytes at a time, and writes all
	 * bytes back to memory if any byte in the line is modified.
	 *
	 * We need to round to the nearest cache line size to avoid a potential
	 * memory consistency problem where paintbox_bus_alloc is called twice,
	 * memory is written from the AP on one of the allocations, memory is
	 * written from JQS on the other allocation, and the allocations share a
	 * cache line.
	 *
	 * Note: This logic should live in the Airbrush DRAM manager. But since
	 * that component doesn't exist yet, it'll live here for now.
	 */
	size = ALIGN(size, JQS_CACHE_LINE_SIZE);
	return bus->ops->alloc(bus->parent_dev, size, shared_buffer);
}

static inline void paintbox_bus_free(struct paintbox_bus *bus,
		struct paintbox_shared_buffer *shared_buffer) {
	if (shared_buffer->host_vaddr) {
		bus->ops->free(bus->parent_dev, shared_buffer);
		memset(shared_buffer, 0, sizeof(*shared_buffer));
	}
}

static inline void paintbox_bus_sync(struct paintbox_bus *bus,
		struct paintbox_shared_buffer *alloc, uint32_t offset,
		size_t size, enum dma_data_direction direction) {
	bus->ops->sync(bus->parent_dev, alloc, offset, size, direction);
}

void paintbox_bus_notify_firmware_up(struct paintbox_bus *bus);
void paintbox_bus_notify_firmware_down(struct paintbox_bus *bus);

#endif /* __PAINTBOX_BUS_COMMON_H__ */

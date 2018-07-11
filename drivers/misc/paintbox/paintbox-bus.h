/*
 * Paintbox Bus Header
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

#ifndef __PAINTBOX_BUS_H__
#define __PAINTBOX_BUS_H__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/dma-direction.h>


struct paintbox_pdata {
	unsigned long page_size_bitmap;
	unsigned int input_address_size;
	unsigned int output_address_size;
	uint64_t dma_base;
	uint64_t dma_size;
	uint32_t hardware_id;
};

typedef irqreturn_t (*pb_irq_handler_t)(struct device *, uint32_t);

/* Upcall interface between the common paintbox bus layer and the paintbox
 * devices.
 */
struct paintbox_device_ops {
	void (*firmware_up)(struct device *dev);
	void (*firmware_down)(struct device *dev);
};

extern struct bus_type paintbox_bus_type;

void paintbox_writel(struct device *dev, uint32_t val, unsigned int offset);
void paintbox_writeq(struct device *dev, uint64_t val, unsigned int offset);
uint32_t paintbox_readl(struct device *dev, unsigned int offset);
uint64_t paintbox_readq(struct device *dev, unsigned int offset);

/* If the handler is already set then the new handler will overwrite the old
 * handler entry.
 */
void paintbox_request_irq(struct device *dev, pb_irq_handler_t handler,
		uint32_t irq_mask);
void paintbox_release_irq(struct device *dev);

void paintbox_set_device_ops(struct device *dev,
		const struct paintbox_device_ops *dev_ops);

struct iommu_group *paintbox_get_device_group(struct device *dev);
struct device *paintbox_bus_get_iommu_device(struct device *dev);
struct dentry *paintbox_bus_get_debug_root(struct device *dev);

#endif /* __PAINTBOX_BUS_H__ */

/*
 * IPU Adapter Header
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

#ifndef __IPU_ADAPTER_H__
#define __IPU_ADAPTER_H__

#include <linux/ipu-core.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/types.h>

struct paintbox_bus;

enum paintbox_device_type {
	PAINTBOX_DEVICE_TYPE_IPU,
	PAINTBOX_DEVICE_TYPE_IOMMU,
	PAINTBOX_DEVICE_TYPE_COUNT
};

enum paintbox_jqs_status {
	JQS_FW_STATUS_INIT = 0,
	JQS_FW_STATUS_REQUESTED,
	JQS_FW_STATUS_STAGED,
	JQS_FW_STATUS_RUNNING
};

/* Interface between the common paintbox bus layer and bus implementation layer.
 */
struct paintbox_bus_ops {
	void (*write32)(struct device *dev, uint32_t value,
			unsigned int offset);
	void (*write64)(struct device *dev, uint64_t value,
			unsigned int offset);
	uint32_t (*read32)(struct device *dev, unsigned int offset);
	uint64_t (*read64)(struct device *dev, unsigned int offset);
	int (*alloc)(struct device *dev, size_t size,
			struct paintbox_shared_buffer *shared_buffer);
	void (*free)(struct device *dev,
			struct paintbox_shared_buffer *shared_buffer);
	void (*sync)(struct device *dev,
			struct paintbox_shared_buffer *shared_buffer,
			uint32_t offset, size_t size,
			enum dma_data_direction direction);
	int (*map_to_bar)(struct device *dev,
			struct paintbox_shared_buffer *shared_buffer);
	int (*unmap_from_bar)(struct device *dev,
			struct paintbox_shared_buffer *shared_buffer);
	struct device *(*get_dma_device)(struct device *dev);
};

/* The following group of functions can be called in an atomic context */
void ipu_bus_notify_watchdog(struct paintbox_bus *bus);
void ipu_bus_notify_link_up(struct paintbox_bus *bus);
void ipu_bus_notify_link_down(struct paintbox_bus *bus);

void ipu_bus_notify_clock_enable(struct paintbox_bus *bus,
		uint64_t clock_rate_hz);
void ipu_bus_notify_clock_disable(struct paintbox_bus *bus);

int ipu_bus_device_register(struct paintbox_bus *bus, const char *name,
		enum paintbox_device_type type);
enum paintbox_jqs_status ipu_bus_get_fw_status(struct paintbox_bus *bus);

int ipu_bus_initialize(struct device *parent_dev,
		struct paintbox_bus_ops *ops, struct paintbox_pdata *pdata,
		struct paintbox_bus **bus);
void ipu_bus_deinitialize(struct paintbox_bus *bus);

#endif /* __IPU_ADAPTER_H__ */

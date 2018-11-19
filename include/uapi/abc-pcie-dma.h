/*
 * Android coprocessor DMA library
 *
 * Copyright 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _UAPI__ABC_PCIE_DMA_H
#define _UAPI__ABC_PCIE_DMA_H

#include <linux/compiler.h>
#include <linux/dma-direction.h>
#include <linux/ioctl.h>
#include <linux/types.h>

#define ABC_PCIE_DMA_IOC_MAGIC 'U'

enum dma_buf_type {
	DMA_BUFFER_USER = 0,
	DMA_BUFFER_DMA_BUF
};

struct abc_pcie_dma_desc_legacy {
	enum dma_buf_type local_buf_type; /* local buffer type (DMA/user) */
	union {
		void __user *local_buf; /* local buffer address */
		int local_dma_buf_fd; /* local DMA buffer file descriptor */
	};
	uint32_t local_buf_size; /* local buffer size */

	/* TODO(alexperez):  Remove support for specifying an arbitrary AB
	 * physical address once bringup is done. b/113105230
	 *
	 * local buffer type (DMA/AB physical)
	 */
	enum dma_buf_type remote_buf_type;
	union {
		uint64_t remote_buf; /* remote buffer address */
		int remote_dma_buf_fd; /* remote DMA buffer file descriptor */
	};
	enum dma_data_direction dir; /* direction of the DMA transfer */
	uint8_t chan; /* dma channel to be used */
};

struct abc_pcie_dma_desc {
	enum dma_buf_type local_buf_type; /* local buffer type (DMA/user) */
	union {
		void __user *local_buf; /* local buffer address */
		int local_dma_buf_fd; /* local DMA buffer file descriptor */
	};
	uint64_t local_dma_buf_off; /* offset within dma buf to xfer from/to */

	enum dma_buf_type remote_buf_type;
	union {
		uint64_t remote_buf; /* remote buffer virtual address */
		int remote_dma_buf_fd; /* remote DMA buffer file descriptor */
	};
	uint64_t remote_dma_buf_off; /* offset within dma buf to xfer from/to */

	uint64_t size; /* number of bytes to transfer */
	enum dma_data_direction dir; /* direction of the DMA transfer */
};

#define ABC_PCIE_DMA_IOC_POST_DMA_XFER_LEGACY				\
	_IOW(ABC_PCIE_DMA_IOC_MAGIC, 1, struct abc_pcie_dma_desc_legacy *)

#define ABC_PCIE_DMA_IOC_POST_DMA_XFER_SYNC				\
	_IOW(ABC_PCIE_DMA_IOC_MAGIC, 2, struct abc_pcie_dma_desc)

#endif /* _UAPI__ABC_PCIE_DMA_H */

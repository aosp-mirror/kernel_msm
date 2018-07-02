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

#include <linux/dma-direction.h>
#include <linux/ioctl.h>

#define ABC_PCIE_DMA_IOC_MAGIC 'U'

enum dma_buf_type {
	DMA_BUFFER_USER = 0,
	DMA_BUFFER_DMA_BUF
};

struct abc_pcie_dma_desc {
	void __user *local_buf;      /* local buffer address */
	__u32 local_buf_size;        /* local buffer size */
	int local_dma_buf_fd;        /* local DMA buffer file descriptor */
	enum dma_buf_type buf_type;  /* local buffer type (DMA/user) */
	/* remote buf will be replaced by an _fd when ABC mem.alloc is avail. */
	__u64 remote_buf;            /* physical address on ABC */
	enum dma_data_direction dir; /* direction of the DMA transfer */
	uint8_t chan;                /* dma channel to be used */
};

#define ABC_PCIE_DMA_IOC_POST_DMA_XFER					\
	_IOW(ABC_PCIE_DMA_IOC_MAGIC, 1, struct abc_pcie_dma_desc *)

#endif /* _UAPI__ABC_PCIE_DMA_H */

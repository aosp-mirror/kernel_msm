/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Oscar kernel-userspace interface definitions.
 *
 * Copyright (C) 2018 Google, Inc.
 */
#ifndef __OSCAR_H__
#define __OSCAR_H__

#include <linux/ioctl.h>
#include <linux/types.h>

struct oscar_gate_clock_ioctl {
	/* Enter or leave clock gated state. */
	uint64_t enable;

	/* If set, enter clock gating state, regardless of custom block's
	 * internal idle state
	 */
	uint64_t force_idle;
};

struct oscar_abdram_alloc_ioctl {
	uint32_t flags;		/* alloc flags, reserved, should be zero */
	uint64_t size;		/* size of buffer to allocate in bytes */
};

#define OSCAR_SYNC_TO_BUFFER	0x1
#define OSCAR_SYNC_FROM_BUFFER	0x2

struct oscar_abdram_sync_ioctl {
	uint32_t fd;		/* AB-DRAM buffer dma_buf fd */
	uint32_t cmd;		/* one of OSCAR_SYNC_* commands above */
	uint64_t host_address;	/* host buffer address */
	uint64_t len;		/* transfer length in bytes */
};

/* gasket fields are same as gasket.h ioctls */
struct oscar_abdram_map_ioctl {
	uint32_t fd;			/* AB-DRAM buffer dma_buf fd */
	uint32_t flags;			/* gasket mapping flags */
	uint32_t page_table_index;	/* gasket page table idnex */
	uint64_t device_address;	/* TPU address at which to map */
};

/* Base number for all Oscar-common IOCTLs */
#define OSCAR_IOCTL_BASE 0x7F

/* Enable/Disable clock gating. */
#define OSCAR_IOCTL_GATE_CLOCK                                                 \
	_IOW(OSCAR_IOCTL_BASE, 0, struct oscar_gate_clock_ioctl)

/*
 * Allocate AB-DRAM buffer.
 * Returns dma_buf fd for the buffer, or negative for error.
 */
#define OSCAR_IOCTL_ABC_ALLOC_BUFFER                                           \
	_IOW(OSCAR_IOCTL_BASE, 1, struct oscar_abdram_alloc_ioctl)

/*
 * Sync AP and AB-DRAM buffers via PCIe EP DMA engine.
 */
#define OSCAR_IOCTL_ABC_SYNC_BUFFER                                            \
	_IOW(OSCAR_IOCTL_BASE, 2, struct oscar_abdram_sync_ioctl)

/* Map AB-DRAM buffer to TPU. */
#define OSCAR_IOCTL_ABC_MAP_BUFFER                                             \
	_IOW(OSCAR_IOCTL_BASE, 3, struct oscar_abdram_map_ioctl)

/*
 * Unmap previously mapped AB-DRAM buffer from TPU.
 * Parameter is AB-DRAM dma_buf fd returned by ALLOC_BUFFER.
 */
#define OSCAR_IOCTL_ABC_UNMAP_BUFFER                                           \
	_IOW(OSCAR_IOCTL_BASE, 4, uint32_t)

/*
 * Deallocate AB-DRAM buffer.  If mapped to TPU, unmap first.
 * Parameter is AB-DRAM dma_buf fd returned by ALLOC_BUFFER, which must be
 * closed by caller after this call returns.
 */
#define OSCAR_IOCTL_ABC_DEALLOC_BUFFER                                         \
	_IOW(OSCAR_IOCTL_BASE, 5, uint32_t)

#endif /* __OSCAR_H__ */

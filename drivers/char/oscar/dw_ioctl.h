/* DS kernel-userspace interface definition(s).
 *
 * Copyright (C) 2017 Google, Inc.
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
#ifndef __DW_IOCTL_H__
#define __DW_IOCTL_H__

#ifdef __KERNEL__
#include "linux_ds_ioctl.h"
#endif
#include <linux/ioctl.h>
#ifndef __KERNEL__
#include <stdint.h>
#endif

/* Structural definitions/macros. */
/* The number of PCI BARs. */
#define DW_NUM_BARS 3

/* constants */
#define DW_PAGE_SHIFT 12
#define DW_PAGE_SIZE (1 << DW_PAGE_SHIFT)

#define DW_EXTENDED_SHIFT 63 /* Extended address bit position. */

/* Addresses are 2^3=8 bytes each. */
/* page in second level page table */
/* holds DW_PAGE_SIZE/8 addresses  */
#define DW_ADDR_SHIFT 3
#define DW_LEVEL_SHIFT (DW_PAGE_SHIFT - DW_ADDR_SHIFT)
#define DW_LEVEL_SIZE (1 << DW_LEVEL_SHIFT)

#define DW_PAGE_TABLE_MAX 65536
#define DW_SIMPLE_PAGE_MAX DW_PAGE_TABLE_MAX
#define DW_EXTENDED_PAGE_MAX (DW_PAGE_TABLE_MAX << DW_LEVEL_SHIFT)

#define DW_RESET_RETRY 120 /* check reset 120 times */
#define DW_RESET_DELAY 100 /* wait 100 ms between checks */
			   /* total 12 sec wait maximum */

#define DW_CHIP_INIT_DONE 2
#define DW_RESET_ACCEPTED 0

enum dw_reset_types {
	DW_HARD_RESET = 1,
	DW_SOFT_RESET = 2,
	DW_CHIP_REINIT_RESET = 3
};

/* Interrupt defines */
/* DS device interrupts enums must be dense (i.e., no empty slots). */
enum dw_interrupt {
	DW_INTERRUPT_INSTR_QUEUE = 0,
	DW_INTERRUPT_INPUT_ACTV_QUEUE = 1,
	DW_INTERRUPT_PARAM_QUEUE = 2,
	DW_INTERRUPT_OUTPUT_ACTV_QUEUE = 3,
	DW_INTERRUPT_SC_HOST_0 = 4,
	DW_INTERRUPT_SC_HOST_1 = 5,
	DW_INTERRUPT_SC_HOST_2 = 6,
	DW_INTERRUPT_SC_HOST_3 = 7,
	DW_INTERRUPT_TOP_LEVEL_0 = 8,
	DW_INTERRUPT_TOP_LEVEL_1 = 9,
	DW_INTERRUPT_TOP_LEVEL_2 = 10,
	DW_INTERRUPT_TOP_LEVEL_3 = 11,
	DW_INTERRUPT_FATAL_ERR = 12,
	DW_INTERRUPT_COUNT = 13,
};

/* Interrupt defines for wire interrupts (multiple dw_interrupts may be
 * multiplexed under a single wire interrupt). Exactly which wire maps
 * to which interrupt is chip specific.
 */
enum dw_wire_interrupt {
	DW_INTERRUPT_WIRE_0 = 0,
	DW_INTERRUPT_WIRE_1 = 1,
	DW_INTERRUPT_WIRE_2 = 2,
};

/*
 * Clock Gating ioctl.
 */
struct dw_gate_clock_ioctl {
	/* enable : enter or leave clock gated state. */
	uint64_t enable;

	/* force_idle: if set, enter clock gating state, regardless of custom
	 * block's internal idle state
	 */
	uint64_t force_idle;
};

/* Base number for all DW-common IOCTLs */
#define DW_IOCTL_BASE 0x7F

/*
 * DS_IOCTL_CLOCK_GATING: Enable/Disable clock gating.
 */
#define DW_IOCTL_GATE_CLOCK _IOW(DW_IOCTL_BASE, 0, struct dw_gate_clock_ioctl)

#endif /* __DW_IOCTL_H__ */

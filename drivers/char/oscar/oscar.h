/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Oscar kernel-userspace interface definitions.
 *
 * Copyright (C) 2018 Google, Inc.
 */
#ifndef __OSCAR_H__
#define __OSCAR_H__

#include <linux/ioctl.h>

struct oscar_gate_clock_ioctl {
	/* Enter or leave clock gated state. */
	uint64_t enable;

	/* If set, enter clock gating state, regardless of custom block's
	 * internal idle state
	 */
	uint64_t force_idle;
};

/* Base number for all Oscar-common IOCTLs */
#define OSCAR_IOCTL_BASE 0x7F

/* Enable/Disable clock gating. */
#define OSCAR_IOCTL_GATE_CLOCK                                                 \
	_IOW(OSCAR_IOCTL_BASE, 0, struct oscar_gate_clock_ioctl)

#endif /* __OSCAR_H__ */

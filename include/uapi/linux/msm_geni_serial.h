/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __UAPI_LINUX_MSM_GENI_SERIAL_H
#define __UAPI_LINUX_MSM_GENI_SERIAL_H

/* IOCTLS used by BT clients to control UART power state */

#define MSM_GENI_SERIAL_TIOCFAULT	0x54EC  /* Uart fault */
#define MSM_GENI_SERIAL_TIOCPMGET	_IOR('t', 55, int)	/* PM get */ /* PPPIOCGCHAN */
#define MSM_GENI_SERIAL_TIOCPMPUT	_IOR('t', 86, int)	/* PM put */ /* PPPIOCGUNIT */
#define MSM_GENI_SERIAL_TIOCPMACT	0x54EF	/* PM is active */

#endif /* __UAPI_LINUX_MSM_GENI_SERIAL_H */

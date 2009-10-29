/*
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
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

#ifndef __ASM_ARCH_MSM_SERIAL_HS_H
#define __ASM_ARCH_MSM_SERIAL_HS_H

#include <linux/serial_core.h>

/* API to request the uart clock off or on for low power management
 * Clients should call request_clock_off() when no uart data is expected,
 * and must call request_clock_on() before any further uart data can be
 * received. */
extern void msm_hs_request_clock_off(struct uart_port *uport);
extern void msm_hs_request_clock_on(struct uart_port *uport);
/* uport->lock must be held when calling _locked() */
extern void msm_hs_request_clock_off_locked(struct uart_port *uport);
extern void msm_hs_request_clock_on_locked(struct uart_port *uport);

/* Optional platform device data for msm_serial_hs driver.
 * Used to configure low power rx wakeup */
struct msm_serial_hs_platform_data {
	int rx_wakeup_irq;  /* wakeup irq */
	/* bool: inject char into rx tty on wakeup */
	unsigned char inject_rx_on_wakeup;
	char rx_to_inject;

	void (*exit_lpm_cb)(struct uart_port *);
};

#endif

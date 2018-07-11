/*
 * Interrupt support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
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

#ifndef __PAINTBOX_IRQ_H__
#define __PAINTBOX_IRQ_H__

#include <linux/kernel.h>
#include <linux/types.h>

#include "paintbox-common.h"

int allocate_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int release_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int wait_for_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int paintbox_flush_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int paintbox_flush_all_interrupts_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int paintbox_irq_init(struct paintbox_data *pb);
void paintbox_irq_remove(struct paintbox_data *pb);
void paintbox_irq_release_all_waiters(struct paintbox_data *pb);

/* The caller to these functions must hold pb->lock */
int validate_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id);
struct paintbox_irq *get_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id,
		int *err);
int allocate_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id);
int release_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_irq *irq);

int bind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		unsigned int interrupt_id);
int unbind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel);
int bind_stp_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_stp *stp, unsigned int interrupt_id);
int unbind_stp_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp);
void paintbox_irq_wait_for_release_complete(struct paintbox_data *pb,
		struct paintbox_session *session);

/* The following function must be called with interrupts disabled. */
void paintbox_irq_waiter_signal(struct paintbox_data *pb,
		struct paintbox_irq *irq, ktime_t timestamp, uint16_t data,
		int error);

#endif /* __PAINTBOX_IRQ_H__ */

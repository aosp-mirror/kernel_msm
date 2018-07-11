/*
 * Perf Monitor kernel driver support for the Paintbox Image Processing Unit
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

#ifndef __PAINTBOX_PMON_INTERNAL_H__
#define __PAINTBOX_PMON_INTERNAL_H__

#include "paintbox-common.h"

/* |arg| - enum pmon_block_type: block to allocate (BIF/MMU/DMA)
 *
 * When you allocate a LBP or STP, you automatically gain access to the pmon
 * counters for that resource.  The pmon counters for BIF/MMU/DMA are a global
 * resource and must be allocated by a session before they can be used.
 */
int pmon_allocate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* |arg| - enum pmon_block_type: block to release (BIF/MMU/DMA)
 *
 * When you allocate a LBP or STP, you automatically gain access to the pmon
 * counters for that resource.  The pmon counters for BIF/MMU/DMA are a global
 * resource and must be allocated by a session before they can be used.
 */
int pmon_release_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* |arg| - struct pmon_config
 *
 * Configure the pmon counter specified by block/core/counter
 */
int pmon_config_write_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* |arg| - struct pmon_data
 *
 * Read the pmon counter specified by block/core/counter
 */
int pmon_data_read_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* |arg| - struct pmon_data
 *
 * Write the pmon counter specified by block/core/counter
 */
int pmon_data_write_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* |arg| - struct pmon_enable
 *
 * Enable/disable the pmon counters on the specified block.  Also sets block
 * level settings like LBP's rptr_id and DMA's channel_id_N.
 */
int pmon_enable_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

#endif /* __PAINTBOX_PMON_INTERNAL_H__ */

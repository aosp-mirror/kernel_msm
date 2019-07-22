/*
 * BCM15602 consumer interface
 *
 * Copyright (C) 2019 Google, Inc.
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

#ifndef _BCM15602_CONSUMER_H
#define _BCM15602_CONSUMER_H

/* driver data structure */
struct bcm15602_chip;

bool bcm15602_is_vbat_above_threshold(struct bcm15602_chip *ddata,
				      int threshold_us);

#endif /* _BCM15602_CONSUMER_H */

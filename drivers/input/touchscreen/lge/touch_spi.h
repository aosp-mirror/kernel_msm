/*
 * touch_spi.h
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef TOUCH_SPI_H
#define TOUCH_SPI_H

#include <touch_hwif.h>

extern int touch_spi_read(struct spi_device *, struct touch_bus_msg *msg);
extern int touch_spi_write(struct spi_device *, struct touch_bus_msg *msg);
extern int touch_spi_xfer(struct spi_device *, struct touch_xfer_msg *xfer);
extern int touch_spi_device_init(struct touch_hwif *hwif, void *driver);
extern void touch_spi_device_exit(struct touch_hwif *hwif);

#endif

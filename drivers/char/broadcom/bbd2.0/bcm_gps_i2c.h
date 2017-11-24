/******************************************************************************
* Copyright (C) 2013 Broadcom Corporation
*
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation version 2.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
******************************************************************************/

#ifndef _BCM_GPS_I2C_H_
#define _BCM_GPS_I2C_H_

struct bcm_gps_platform_data {
	unsigned int gpio_i2c; /* HOST_REQ : to indicate that ASIC has data to send. */
};

/* example : arch/board_specific_file.c

#define GPS_I2C_GPIO (25)

static struct bcm_gps_platform_data gps_i2c_data = {
	.gpio_i2c = GPS_I2C_GPIO,
};


static struct i2c_board_info gps_i2c[] = {
	{
	I2C_BOARD_INFO("gpsi2c", 0x00),
	.platform_data = & gps_i2c_data,
	},
};

...
{
	i2c_register_board_info(1, gps_i2c, ARRAY_SIZE(gps_i2c));
}


*/

#endif /* _BCM_GPS_I2C_H_ */

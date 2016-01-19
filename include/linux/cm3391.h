/* include/linux/cm3391.h
 *
 * Copyright (C) 2015 Vishay Capella Microsystems. Inc.
 * Author: Frank Hsieh <frank.hsieh@vishay.com>
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

#ifndef __LINUX_CM3391_H
#define __LINUX_CM3391_H


#define CM3391_I2C_NAME "cm3391"

/*cm3391*/
#define	CM3391_SLAVE_addr	  0x48

/*for MAIN command*/
#define CM3391_SD		          (0x0001)
#define CM3391_SD_MASK		    (0xFFF7)

#define CM3391_CMD_Addr          0x00

//#define CM3391_RD_SEL_MASK		0xFC
#define CM3391_RD_SEL_R   		   0x08
#define CM3391_RD_SEL_G		       0x09
#define CM3391_RD_SEL_B   		   0x0A
#define CM3391_RD_SEL_X   		   0x0B
#define CM3391_RD_SEL_Y		       0x06
#define CM3391_RD_SEL_Z   		   0x07

/*for integration time setting command*/
#define CM3391_IT_50MS   		     0x0000
#define CM3391_IT_100MS   		   0x0010
#define CM3391_IT_200MS   		   0x0020
#define CM3391_IT_400MS   		   0x0030
#define CM3391_IT_800MS   		   0x0040

/*for high sensitive setting command*/
#define CM3391_HD_ENABLE         0x0008
#define CM3391_HD_DISABLE        0x0000

#define LS_PWR_ON					(1 << 0)
#define PS_PWR_ON					(1 << 1)

struct cm3391_platform_data {
	int (*power)(int, uint8_t); /* power to the chip */
	uint16_t RGB_slave_address;
};

#endif

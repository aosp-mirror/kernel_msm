/*
 * Utility definitions for TUSB1044 USB3.0 Redriver
 *
 * Copyright (C) 2016 HTC Corporation.
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

/* General Registers */
#define REG_REVISION		0x08
#define REG_CONFIG_CTRL		0x0A
#define REG_CHANNEL_SWAP_SEL	0x0B
#define REG_VOD_DCGAIN		0x0C
#define REG_DP_AUX		0x13

/* register CONFIG_CTRL value */
#define DISABLE_TX_RX	0x00
#define USB3_ON_CC1	0x01
#define	USB3_ON_CC2	0x05
#define DP_ON_CC1	0x0A
#define DP_ON_CC2	0x0E

/* register DP_AUX value */
#define ENABLE_AUX_SNOOP	0x00
#define DISABLE_AUX_SNOOP	0x80

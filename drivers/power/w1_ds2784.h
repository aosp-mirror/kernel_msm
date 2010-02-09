/* drivers/w1/slaves/w1_ds2784.h
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Justin Lin <Justin_Lin@htc.com>
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

#ifndef __w1_ds2784_h__
#define __w1_ds2784_h__


/* Known commands to the DS2784 chip */
#define W1_DS2784_SWAP			0xAA
#define W1_DS2784_READ_DATA		0x69
#define W1_DS2784_WRITE_DATA		0x6C
#define W1_DS2784_COPY_DATA		0x48
#define W1_DS2784_RECALL_DATA		0xB8
#define W1_DS2784_LOCK			0x6A

/* Number of valid register addresses */
#define DS2784_DATA_SIZE		0x80

#define DS2784_EEPROM_BLOCK0		0x20
#define DS2784_ACTIVE_FULL		0x20
#define DS2784_EEPROM_BLOCK1		0x30
#define DS2784_RATED_CAPACITY		0x32
#define DS2784_CURRENT_OFFSET_BIAS	0x33
#define DS2784_ACTIVE_EMPTY		0x3b

/**
 * The DS2482 registers - there are 3 registers that are addressed by a read
 * pointer. The read pointer is set by the last command executed.
 *
 * To read the data, issue a register read for any address
 */
#define DS2482_CMD_RESET		0xF0	/* No param */
#define DS2482_CMD_SET_READ_PTR		0xE1	/* Param: DS2482_PTR_CODE_xxx */
#define DS2482_CMD_CHANNEL_SELECT	0xC3
#define DS2482_CMD_WRITE_CONFIG		0xD2	/* Param: Config byte */
#define DS2482_CMD_1WIRE_RESET		0xB4	/* Param: None */
#define DS2482_CMD_1WIRE_SINGLE_BIT	0x87	/* Param: Bit byte (bit7) */
#define DS2482_CMD_1WIRE_WRITE_BYTE	0xA5	/* Param: Data byte */
#define DS2482_CMD_1WIRE_READ_BYTE	0x96	/* Param: None */
/* Note to read the byte, Set the ReadPtr to Data then read (any addr) */
#define DS2482_CMD_1WIRE_TRIPLET	0x78	/* Param: Dir byte (bit7) */

/* Values for DS2482_CMD_SET_READ_PTR */
#define DS2482_PTR_CODE_STATUS		0xF0
#define DS2482_PTR_CODE_DATA		0xE1
#define DS2482_PTR_CODE_CHANNEL		0xD2	/* DS2482-800 only */
#define DS2482_PTR_CODE_CONFIG		0xC3

/*
DS2784 1-wire slave memory map definitions
*/
#define DS2784_REG_PORT			0x00
#define DS2784_REG_STS			0x01
#define DS2784_REG_RAAC_MSB		0x02
#define DS2784_REG_RAAC_LSB		0x03
#define DS2784_REG_RSAC_MSB		0x04
#define DS2784_REG_RSAC_LSB		0x05
#define DS2784_REG_RARC			0x06
#define DS2784_REG_RSRC			0x07
#define DS2784_REG_AVG_CURR_MSB		0x08
#define DS2784_REG_AVG_CURR_LSB		0x09
#define DS2784_REG_TEMP_MSB		0x0A
#define DS2784_REG_TEMP_LSB		0x0B
#define DS2784_REG_VOLT_MSB		0x0C
#define DS2784_REG_VOLT_LSB		0x0D
#define DS2784_REG_CURR_MSB		0x0E
#define DS2784_REG_CURR_LSB		0x0F
#define DS2784_REG_ACCUMULATE_CURR_MSB	0x10
#define DS2784_REG_ACCUMULATE_CURR_LSB	0x11
#define DS2784_REG_ACCUMULATE_CURR_LSB1	0x12
#define DS2784_REG_ACCUMULATE_CURR_LSB2	0x13
#define DS2784_REG_AGE_SCALAR		0x14
#define DS2784_REG_SPECIALL_FEATURE	0x15
#define DS2784_REG_FULL_MSB		0x16
#define DS2784_REG_FULL_LSB		0x17
#define DS2784_REG_ACTIVE_EMPTY_MSB	0x18
#define DS2784_REG_ACTIVE_EMPTY_LSB	0x19
#define DS2784_REG_STBY_EMPTY_MSB	0x1A
#define DS2784_REG_STBY_EMPTY_LSB	0x1B
#define DS2784_REG_EEPROM		0x1F
#define DS2784_REG_MFG_GAIN_RSGAIN_MSB	0xB0
#define DS2784_REG_MFG_GAIN_RSGAIN_LSB	0xB1

#define DS2784_REG_CTRL			0x60
#define DS2784_REG_ACCUMULATE_BIAS	0x61
#define DS2784_REG_AGE_CAPA_MSB		0x62
#define DS2784_REG_AGE_CAPA_LSB		0x63
#define DS2784_REG_CHARGE_VOLT		0x64
#define DS2784_REG_MIN_CHARGE_CURR	0x65
#define DS2784_REG_ACTIVE_EMPTY_VOLT	0x66
#define DS2784_REG_ACTIVE_EMPTY_CURR	0x67
#define DS2784_REG_ACTIVE_EMPTY_40	0x68
#define DS2784_REG_RSNSP		0x69
#define DS2784_REG_FULL_40_MSB		0x6A
#define DS2784_REG_FULL_40_LSB		0x6B
#define DS2784_REG_FULL_SEG_4_SLOPE	0x6C
#define DS2784_REG_FULL_SEG_3_SLOPE	0x6D
#define DS2784_REG_FULL_SEG_2_SLOPE	0x6E
#define DS2784_REG_FULL_SEG_1_SLOPE	0x6F
#define DS2784_REG_AE_SEG_4_SLOPE	0x70
#define DS2784_REG_AE_SEG_3_SLOPE	0x71
#define DS2784_REG_AE_SEG_2_SLOPE	0x72
#define DS2784_REG_AE_SEG_1_SLOPE	0x73
#define DS2784_REG_SE_SEG_4_SLOPE	0x74
#define DS2784_REG_SE_SEG_3_SLOPE	0x75
#define DS2784_REG_SE_SEG_2_SLOPE	0x76
#define DS2784_REG_SE_SEG_1_SLOPE	0x77
#define DS2784_REG_RSGAIN_MSB		0x78
#define DS2784_REG_RSGAIN_LSB		0x79
#define DS2784_REG_RSTC			0x7A
#define DS2784_REG_CURR_OFFSET_BIAS	0x7B
#define DS2784_REG_TBP34		0x7C
#define DS2784_REG_TBP23		0x7D
#define DS2784_REG_TBP12		0x7E
#define DS2784_REG_PROTECTOR_THRESHOLD	0x7F

#define DS2784_REG_USER_EEPROM_20	0x20

#endif /* !__w1_ds2784__ */

/*
 * Copyright (C) 2007-2008 HTC Corporation.
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

#ifndef CAMERA_MT9T013_H
#define CAMERA_MT9T013_H
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/sizes.h>

/*************************************************************
*	IOCTL define
*************************************************************/

#define MT9T013_I2C_IOCTL_MAGIC 'm'

#define MT9T013_I2C_IOCTL_W \
	        _IOW(MT9T013_I2C_IOCTL_MAGIC, 0, unsigned)

#define MT9T013_I2C_IOCTL_R \
	        _IOR(MT9T013_I2C_IOCTL_MAGIC, 1, unsigned)

#define MT9T013_I2C_IOCTL_AF_W \
	        _IOW(MT9T013_I2C_IOCTL_MAGIC, 2, unsigned)

#define MT9T013_I2C_IOCTL_CAMIF_PAD_REG_RESET \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 3)

#define MT9T013_I2C_IOCTL_CAMIF_PAD_REG_RESET_2 \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 4)

#define CAMERA_CONFIGURE_GPIOS \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 7)

#define CAMERA_UNCONFIGURE_GPIOS \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 8)
	
#define CAMERA_LENS_POWER_ON \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 9)
	
#define CAMERA_LENS_POWER_OFF \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 10)

#define MT9T013_I2C_IOCTL_CAMIF_APPS_RESET \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 11)

/* Replacement ioctls() for the clkrgm_sec RPCs. */

#define CAMIO_VFE_MDC_CLK 1 /* enable, disable */
#define CAMIO_MDC_CLK     2 /* enable, disable */
#define CAMIO_VFE_CLK     3 /* clk_select, freq_prog */

#define MT9T013_I2C_IOCTL_CLK_ENABLE \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 12, unsigned)

#define MT9T013_I2C_IOCTL_CLK_DISABLE \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 13, unsigned)

#define MT9T013_I2C_IOCTL_CLK_SELECT \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 14, unsigned)

#define MT9T013_I2C_IOCTL_CLK_FREQ_PROG \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 15, unsigned)

#define CAMSENSOR_REG_INIT		0<<0
#define CAMSENSOR_REG_UPDATE_PERIODIC 	1<<0
#define CAMSENSOR_TYPE_PREVIEW          0<<1
#define CAMSENSOR_TYPE_SNAPSHOT		1<<1

#define MT9T013_I2C_IOCTL_SENSOR_SETTING \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 16, uint32_t)

struct mt9t013_reg_struct
{
	uint16_t vt_pix_clk_div;            /* 0x0300 */
	uint16_t vt_sys_clk_div;            /* 0x0302 */
	uint16_t pre_pll_clk_div;           /* 0x0304 */
	uint16_t pll_multiplier;            /* 0x0306 */
	uint16_t op_pix_clk_div;            /* 0x0308 */
	uint16_t op_sys_clk_div;            /* 0x030A */
	uint16_t scale_m;                   /* 0x0404 */
	uint16_t row_speed;                 /* 0x3016 */
	uint16_t x_addr_start;              /* 0x3004 */
	uint16_t x_addr_end;                /* 0x3008 */
	uint16_t y_addr_start;              /* 0x3002 */
	uint16_t y_addr_end;                /* 0x3006 */
	uint16_t read_mode;                 /* 0x3040 */
	uint16_t x_output_size ;            /* 0x034C */
	uint16_t y_output_size;             /* 0x034E */
	uint16_t line_length_pck;           /* 0x300C */
	uint16_t frame_length_lines;        /* 0x300A */
	uint16_t coarse_integration_time;   /* 0x3012 */
	uint16_t fine_integration_time;     /* 0x3014 */
}; 

struct mt9t013_reg_pat {
	struct mt9t013_reg_struct reg[2];
};

#define MT9T013_I2C_IOCTL_GET_REGISTERS \
	_IOR(MT9T013_I2C_IOCTL_MAGIC, 17, struct mt9t013_reg_pat *)

struct mt9t013_exposure_gain {
	uint16_t gain;
	uint16_t line;
};

#define MT9T013_I2C_IOCTL_EXPOSURE_GAIN \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 18, struct exposure_gain *)

#define MT9T013_I2C_IOCTL_MOVE_FOCUS \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 19, uint32_t)

#define MT9T013_I2C_IOCTL_SET_DEFAULT_FOCUS \
	_IOW(MT9T013_I2C_IOCTL_MAGIC, 20, uint32_t)

#define MT9T013_I2C_IOCTL_POWER_DOWN \
	_IO(MT9T013_I2C_IOCTL_MAGIC, 21)

struct mt9t013_init {
	int preview; /* in: 1 for preview, 0 for capture */
	uint16_t chipid; /* out: chip id */
};

#define MT9T013_I2C_IOCTL_INIT \
	_IOWR(MT9T013_I2C_IOCTL_MAGIC, 22, struct mt9t013_init *)

#endif


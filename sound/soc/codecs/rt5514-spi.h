/*
 * rt5514-spi.h  --  RT5514 driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RT5514_SPI_H__
#define __RT5514_SPI_H__

/**
 * RT5514_SPI_BUF_LEN is the buffer size of SPI master controller.
*/
#define RT5514_SPI_BUF_LEN		240

#define RT5514_BUFFER_VOICE_BASE	0x18002fb4
#define RT5514_BUFFER_VOICE_LIMIT	0x18002fb8
#define RT5514_BUFFER_VOICE_WP		0x18002fbc

#define RT5514_BUFFER_MUSIC_BASE	0x1800103c
#define RT5514_BUFFER_MUSIC_LIMIT	0x18001040
#define RT5514_BUFFER_MUSIC_WP		0x18001044

#define RT5514_BUFFER_ADC_BASE		0x18002fc0
#define RT5514_BUFFER_ADC_LIMIT		0x18002fc4
#define RT5514_BUFFER_ADC_WP		0x18002fc8

#define RT5514_HOTWORD_FLAG		0x18001034
#define RT5514_MUSDET_FLAG		0x18001038
#define RT5514_DSP_WOV_TYPE		0x18002fac
#define RT5514_DSP_FUNC			0x18002fb0

/* SPI Command */
enum {
	RT5514_SPI_CMD_16_READ = 0,
	RT5514_SPI_CMD_16_WRITE,
	RT5514_SPI_CMD_32_READ,
	RT5514_SPI_CMD_32_WRITE,
	RT5514_SPI_CMD_BURST_READ,
	RT5514_SPI_CMD_BURST_WRITE,
};

enum {
	RT5514_DSP_NO_STREAM,
	RT5514_DSP_STREAM_HOTWORD,
	RT5514_DSP_STREAM_MUSDET,
	RT5514_DSP_STREAM_ADC,
};

#define RT5514_DBG_BUF_ADDR 0x4ff60000
#define RT5514_DBG_BUF_SIZE 0x100
#define RT5514_DBG_BUF_CNT  0x1f // (DBG_BUF_SIZE-2*4)/8

typedef struct _dbgBuf_Unit{
	unsigned int id : 8;
	unsigned int ts : 24;
	unsigned int val;
} DBGBUF_UNIT;

typedef struct _dbgBuf_Mem {
	DBGBUF_UNIT unit[RT5514_DBG_BUF_CNT];
	unsigned int reserve;
	unsigned int idx;
} RT5514_DBGBUF_MEM;

int rt5514_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len);
int rt5514_spi_burst_write(u32 addr, const u8 *txbuf, size_t len);

#endif /* __RT5514_SPI_H__ */

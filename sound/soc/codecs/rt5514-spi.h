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
 * the value should be mulitple of 8
*/
#define RT5514_SPI_BUF_LEN		240
#define RT5514_SPI_RETRY_CNT		100

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

#define SPI_SWITCH_MASK_WORK_0 (1 << 0)
#define SPI_SWITCH_MASK_WORK_1 (1 << 1)
#define SPI_SWITCH_MASK_WORK_2 (1 << 2)
#define SPI_SWITCH_MASK_COPY (1 << 3)
#define SPI_SWITCH_MASK_LOAD (1 << 4)
#define SPI_SWITCH_MASK_CMD (1 << 5)
#define SPI_SWITCH_MASK_WATCHDOG (1 << 6)
#define SPI_SWITCH_MASK_NO_IRQ (1 << 7)
#define SPI_SWITCH_MASK_NO_CHRE (1 << 8)
#define SPI_SWITCH_MASK_RESET (1 << 9)

#if IS_ENABLED(CONFIG_SND_SOC_RT5514_QMI)
#define SPI_SWITCH_MASK_CHRE_QMI (1 << 10)
#endif

#define SPI_SWITCH_MASK_WORK_3 (1 << 11)
#define ZEOR_LATENCY_BUFFER_MS 200

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
	RT5514_DSP_STREAM_ZLATENCY,
};

enum {
	RT5514_ID_HOTWORD = 0,
	RT5514_ID_MUSDET,
	RT5514_ID_ADC,
	RT5514_ID_ZLATENCY,
	RT5514_ID_MAX,
};

enum rt5514_work_mode {
	RT5514_MODE_IRQ = 0,
	RT5514_MODE_ADC,
	RT5514_MODE_ZLATENCY,
};

enum {
	RT5514_DSP_HOTWORD = 0,
	RT5514_DSP_MUSDET,
	RT5514_DSP_CHRE,
};

#define RT5514_DBG_BUF_SIZE 0x100
#define RT5514_DBG_BUF_CNT  0x1f // (DBG_BUF_SIZE-2*4)/8

struct _dbgBuf_Unit {
	unsigned int id : 8;
	unsigned int ts : 24;
	unsigned int val;
};

struct _dbgBuf_Mem {
	struct _dbgBuf_Unit unit[RT5514_DBG_BUF_CNT];
	unsigned int reserve;
	unsigned int idx;
};

int rt5514_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len);
int rt5514_spi_burst_write(u32 addr, const u8 *txbuf, size_t len);
void rt5514_spi_request_switch(u32 mask, bool is_require);

extern void (*rt5514_watchdog_handler_cb)(void);
extern bool (*rt5514_buffer_status_cb)(void);
extern int (*rt5514_zlatency_cb)(void);
extern struct regmap *rt5514_g_i2c_regmap;
#define RT5514_SPI_SWITCH_GPIO	5
int rt5514_set_gpio(int gpio, bool output);

#endif /* __RT5514_SPI_H__ */

/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "video_core_type.h"
#include "vcd_ddl_utils.h"
#include "vcd_ddl_metadata.h"

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define ERR(x...) printk(KERN_ERR x)

#ifdef CORE_TIMING_INFO
static unsigned int g_ddl_dec_t1, g_ddl_enc_t1;
static unsigned int g_ddl_dec_ttotal, g_ddl_enc_ttotal;
static unsigned int g_ddl_dec_count, g_ddl_enc_count;
#endif

size_t npelly_size[] = {
	0x100000,
	0x080000,
	0x51c00,
	DDL_CONTEXT_MEMORY,
	DDL_DB_LINE_BUF_SIZE,
	DDL_MPEG4_DATA_PARTITION_BUF_SIZE,
	DDL_METADATA_TOTAL_INPUTBUFSIZE,
	DDL_DBG_CORE_DUMP_SIZE,
	0x040000,
	DDL_ENC_SEQHEADER_SIZE,
};

struct ddl_dma_buffer npelly_b[30];

u32 npelly_init(void) {
	int i;
	printk("\nnpelly npelly max_key = %d\n", npelly_max_key);
	for (i=0; i<npelly_max_key; i++) {
		struct ddl_dma_buffer *b = &npelly_b[i];
		b->size = npelly_size[i];
		b->virt_addr = dma_alloc_coherent(NULL, b->size,
			&b->phys_addr, GFP_KERNEL);
		if (!b->virt_addr) {
			printk("\nnpelly %s: Could not allocate %d for %d\n",
				__FUNCTION__, b->size, i);
			return -1;
		}
		printk("\nnpelly ALLOC %d for %d\n", b->size, i);
		memset(b->virt_addr, 0, b->size);
	}
	return 0;
}

void *ddl_dma_alloc(struct ddl_dma_buffer *b, size_t sz, enum npelly_key key)
{
	printk("\nnpelly RETRIEVE %d for %d\n", sz, key);

	if (sz > npelly_b[key].size) {
		printk("\nnpelly OH SHIT, %d > %d for %d\n", sz, npelly_b[key].size, key);
		BUG_ON(true);
	}
	*b = npelly_b[key];
	b->size = sz;
	memset(b->virt_addr, 0, sz);

	return b->virt_addr;
}

void ddl_dma_free(struct ddl_dma_buffer *b)
{
	printk("\nnpelly RELEASE %d\n", b->size);

	b->virt_addr = NULL;
	b->size = 0;
}

#ifdef CORE_TIMING_INFO
void ddl_get_core_start_time(u8 codec_type)
{
	u32 *ddl_t1 = NULL;
	if (!codec_type)
		ddl_t1 = &g_ddl_dec_t1;
	else if (codec_type == 1)
		ddl_t1 = &g_ddl_enc_t1;

	if (!*ddl_t1) {
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		*ddl_t1 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
	}
}

void ddl_calc_core_time(u8 codec_type)
{
	u32 *ddl_t1 = NULL, *ddl_ttotal = NULL,
		*ddl_count = NULL;
	if (!codec_type) {
		DBG("\n720p Core Decode ");
		ddl_t1 = &g_ddl_dec_t1;
		ddl_ttotal = &g_ddl_dec_ttotal;
		ddl_count = &g_ddl_dec_count;
	} else if (codec_type == 1) {
		DBG("\n720p Core Encode ");
		ddl_t1 = &g_ddl_enc_t1;
		ddl_ttotal = &g_ddl_enc_ttotal;
		ddl_count = &g_ddl_enc_count;
	}

	if (*ddl_t1) {
		int ddl_t2;
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		ddl_t2 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
		*ddl_ttotal += (ddl_t2 - *ddl_t1);
		*ddl_count = *ddl_count + 1;
		DBG("time %u, average time %u, count %u",
			ddl_t2 - *ddl_t1, (*ddl_ttotal)/(*ddl_count),
			*ddl_count);
		*ddl_t1 = 0;
	}
}

void ddl_reset_time_variables(u8 codec_type)
{
	if (!codec_type) {
		DBG("\n Reset Decoder time variables");
		g_ddl_dec_t1 = 0;
		g_ddl_dec_ttotal = 0;
		g_ddl_dec_count = 0;
	} else if (codec_type == 1) {
		DBG("\n Reset Encoder time variables ");
		g_ddl_enc_t1 = 0;
		g_ddl_enc_ttotal = 0;
		g_ddl_enc_count = 0;
	}
}
#endif

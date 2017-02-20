/* production_test.h
 *
 * Copyright (C) 2015 LGE.
 *
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

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw49408.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* production test */
#define PT_FRAME_1				1
#define PT_FRAME_2				2
#define PT_FRAME_3				3
#define PT_FRAME_4				4
#define tc_tsp_test_ctl				(0x4)

/* pt test status offset */
#define pt_sts_off				(42)
#define pt_result_off				(43)

#define abt_rawdata_load_sts			(0x22)
#define abt_rawdata_load_ctl			(0x29)

struct sw49408_test_off {
	u16 offset0;
	u16 offset1;
} __packed;

struct sw49408_test_off_info {
	struct sw49408_test_off m1_m2_raw;
	struct sw49408_test_off frame0_1;
	struct sw49408_test_off frame2_short;
	struct sw49408_test_off os_result;
} __packed;

/* tune code */
#define tc_tune_code_size			260
#define tc_total_ch_size			32
#define TSP_TUNE_CODE_L_GOFT_OFFSET		0
#define TSP_TUNE_CODE_L_M1_OFT_OFFSET		2
#define TSP_TUNE_CODE_L_G1_OFT_OFFSET		(TSP_TUNE_CODE_L_M1_OFT_OFFSET \
							+ tc_total_ch_size)
#define TSP_TUNE_CODE_L_G2_OFT_OFFSET		(TSP_TUNE_CODE_L_G1_OFT_OFFSET \
							+ tc_total_ch_size)
#define TSP_TUNE_CODE_L_G3_OFT_OFFSET		(TSP_TUNE_CODE_L_G2_OFT_OFFSET \
							+ tc_total_ch_size)
#define TSP_TUNE_CODE_R_GOFT_OFFSET		(TSP_TUNE_CODE_L_G3_OFT_OFFSET \
							+ tc_total_ch_size)
#define TSP_TUNE_CODE_R_M1_OFT_OFFSET		(TSP_TUNE_CODE_R_GOFT_OFFSET + 2)
#define TSP_TUNE_CODE_R_G1_OFT_OFFSET		(TSP_TUNE_CODE_R_M1_OFT_OFFSET \
							+ tc_total_ch_size)
#define TSP_TUNE_CODE_R_G2_OFT_OFFSET		(TSP_TUNE_CODE_R_G1_OFT_OFFSET \
							+ tc_total_ch_size)
#define TSP_TUNE_CODE_R_G3_OFT_OFFSET		(TSP_TUNE_CODE_R_G2_OFT_OFFSET \
							+ tc_total_ch_size)

#define PATH_SIZE		64
#define BURST_SIZE		512
#define RAWDATA_SIZE		2
#define ROW_SIZE		32//34
#define COL_SIZE		18
#define M1_COL_SIZE		2
#define LOG_BUF_SIZE		256
#define OCD_SIZE		104
#define DBG_BUF2_OFFSET		1280
#define BUF_SIZE 		(PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define TIME_STR_LEN		(64)

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

/* LCD:OFF	*/
#define NO_TEST			(0x0)
#define U0_PT_TEST		(0x0)
#define U0_OPEN_NODE_TEST	(0x1)
#define U0_SHORT_NODE_TEST	(0x2)
#define U0_M1_RAWDATA_TEST	(0x3)
#define U0_M1_NOISE_TEST	(0x4)
#define U0_M2_RAWDATA_TEST	(0x5)
#define U0_M2_NOISE_TEST	(0x6)
#define U0_M2_DELTA_JITTER  (0xA)
#define M2_DIFF_TEST		(0x10)
#define M1_DIFF_TEST		(0x11)
/* LCD:Partial  */
#define U2_PT_TEST		(0x200)
#define U2_M2_RAWDATA_TEST	(0x205)
/* LCD:ON	*/
#define U3_PT_TEST		(0x300)
#define U3_OPEN_NODE_TEST	(0x301)
#define U3_SHORT_NODE_TEST	(0x302)
#define U3_M1_RAWDATA_TEST	(0x303)
#define U3_M1_NOISE_TEST	(0x304)
#define U3_M2_RAWDATA_TEST	(0x305)
#define U3_M2_NOISE_TEST	(0x306)
#define U3_M2_DELTA_JITTER	(0x30A)

#define PT_FRAME_SIZE		((ROW_SIZE*COL_SIZE) << 1)

#define PT_FRAME_1_OFFSET	(d->reg_info.r_sys_buf_sram_oft)
#define PT_FRAME_2_OFFSET	((d->reg_info.r_sys_buf_sram_oft) + (((PT_FRAME_SIZE) * 1) / 4))
#define PT_FRAME_3_OFFSET	((d->reg_info.r_sys_buf_sram_oft) + (((PT_FRAME_SIZE) * 2) / 4))
#define PT_FRAME_4_OFFSET	((d->reg_info.r_sys_buf_sram_oft) + (((PT_FRAME_SIZE) * 3) / 4))

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

extern void touch_msleep(unsigned int msecs);
int sw49408_prd_register_sysfs(struct device *dev);
void sw49408_te_test_logging(struct device *dev, char *buf);

#endif



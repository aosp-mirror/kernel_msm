/*
 * PMON Register definitions for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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

#ifndef __PAINTBOX_PMON_REGS_H__
#define __PAINTBOX_PMON_REGS_H__

#include "paintbox-regs.h"

/* paintbox-regs.h includes absolute register addresses for all pmon registers
 * This header file converts register addresses back into relative values
 * which allows pmon code to be reused across blocks/counters.
 */

#define PMON_CFG_ENABLE_SHIFT    STP_PMON_CFG_ENABLE_SHIFT

/* number of bytes between block config and counter 0 */
#define PMON_CNT_0 (STP_PMON_CNT_0_CFG - STP_PMON_CFG)

/* number of bytes between successive counters */
#define PMON_CNT_STRIDE (STP_PMON_CNT_1_CFG - STP_PMON_CNT_0_CFG)

/* number of bytes from counter base to register */
#define PMON_CNT_CFG     (STP_PMON_CNT_0_CFG     - STP_PMON_CNT_0_CFG)
#define PMON_CNT         (STP_PMON_CNT_0         - STP_PMON_CNT_0_CFG)
#define PMON_CNT_STS_ACC (STP_PMON_CNT_0_STS_ACC - STP_PMON_CNT_0_CFG)
#define PMON_CNT_STS     (STP_PMON_CNT_0_STS     - STP_PMON_CNT_0_CFG)

#define PMON_CNT_CFG_THRESHOLD_M STP_PMON_CNT_0_CFG_THRESHOLD_M
#define PMON_CNT_CFG_SEL_M       STP_PMON_CNT_0_CFG_INC_SEL_M
#define PMON_CNT_CFG_MATCH_M     STP_PMON_CNT_0_CFG_INC_MATCH_M
#define PMON_CNT_CFG_MASK_M      STP_PMON_CNT_0_CFG_INC_MASK_M

#define PMON_CNT_CFG_DEC_INV_SHIFT STP_PMON_CNT_0_CFG_DEC_INV_SHIFT
#define PMON_CNT_CFG_DEC_SEL_SHIFT STP_PMON_CNT_0_CFG_DEC_SEL_SHIFT
#define PMON_CNT_CFG_DEC_MASK_SHIFT STP_PMON_CNT_0_CFG_DEC_MASK_SHIFT
#define PMON_CNT_CFG_DEC_MATCH_SHIFT STP_PMON_CNT_0_CFG_DEC_MATCH_SHIFT
#define PMON_CNT_CFG_INC_INV_SHIFT STP_PMON_CNT_0_CFG_INC_INV_SHIFT
#define PMON_CNT_CFG_INC_SEL_SHIFT STP_PMON_CNT_0_CFG_INC_SEL_SHIFT
#define PMON_CNT_CFG_INC_MASK_SHIFT STP_PMON_CNT_0_CFG_INC_MASK_SHIFT
#define PMON_CNT_CFG_INC_MATCH_SHIFT STP_PMON_CNT_0_CFG_INC_MATCH_SHIFT
#define PMON_CNT_CFG_THRESHOLD_SHIFT STP_PMON_CNT_0_CFG_THRESHOLD_SHIFT
#define PMON_CNT_CFG_MODE_SHIFT STP_PMON_CNT_0_CFG_MODE_SHIFT

#define PMON_CNT_STS_ACC_OF_SHIFT STP_PMON_CNT_0_STS_ACC_OF_SHIFT
#define PMON_CNT_STS_ACC_UF_SHIFT STP_PMON_CNT_0_STS_ACC_UF_SHIFT
#define PMON_CNT_STS_CNT_OF_SHIFT STP_PMON_CNT_0_STS_CNT_OF_SHIFT
#define PMON_CNT_STS_ACC_M 0xffULL
#define PMON_CNT_CNT_M STP_PMON_CNT_0_CNT_M

/* number of pmon counters in DMA block */
#define DMA_PMON_COUNTERS 4

/* default number of pmon counters in a block */
#define DEFAULT_PMON_COUNTERS 2

#endif /* __PAINTBOX_PMON_REGS_H__ */

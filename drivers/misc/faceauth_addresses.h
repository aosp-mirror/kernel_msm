/*
 * FaceAuth firmware address definition
 *
 * Copyright (C) 2019 Google, Inc.
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

#ifndef __FACEAUTH_ADDRESSES_H__
#define __FACEAUTH_ADDRESSES_H__

/* input image sizes */
#define INPUT_IMAGE_WIDTH 480
#define INPUT_IMAGE_HEIGHT 640
#define INPUT_IMAGE_SIZE (INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT)

/*
 * Registers accessible through BAR0
 * task input/output addresses
 */
#ifdef SYSREG_AON_IPU_REG0
#define RESULT_FLAG_ADDR SYSREG_AON_IPU_REG0
#define ANGLE_RESULT_FLAG_ADDR SYSREG_AON_IPU_REG1
#define INPUT_FLAG_ADDR SYSREG_AON_IPU_REG2
#else
#define SYSREG_AON 0x30000
#define SYSREG_REG_GP_INT0 (SYSREG_AON + 0x37C)
#define SYSREG_AON_IPU_REG29 (SYSREG_AON + 0x438)
#define RESULT_FLAG_ADDR (SYSREG_AON + 0x3C4)
#define ANGLE_RESULT_FLAG_ADDR (SYSREG_AON + 0x3C8)
#define INPUT_FLAG_ADDR (SYSREG_AON + 0x3CC)
#endif

/* AB DRAM Addresses */
/* FW Binary 0x20000000, ~32MB for FW */
#define DYNAMIC_VERBOSITY_RAM_ADDR 0x21fffff0
#define DISABLE_FEATURES_ADDR 0x21fffff8

/* input image addresses */
/* 0x22000000 -> 0x2212C000input images */
#define DOT_LEFT_IMAGE_ADDR 0x22000000
#define DOT_RIGHT_IMAGE_ADDR (DOT_LEFT_IMAGE_ADDR + INPUT_IMAGE_SIZE)
#define FLOOD_IMAGE_ADDR (DOT_RIGHT_IMAGE_ADDR + INPUT_IMAGE_SIZE)
#define RIGHT_FLOOD_IMAGE_ADDR (FLOOD_IMAGE_ADDR + INPUT_IMAGE_SIZE)

/* 0x2212C000 -> 0x2212C400 Calibration
 * TODO(mahdih): figure out calibration exact size
 */
#define CALIBRATION_DATA_ADDR (RIGHT_FLOOD_IMAGE_ADDR + INPUT_IMAGE_SIZE)
#define INPUT_ADDR_END (CALIBRATION_DATA_ADDR + 0x400)

/* 0x2212C400 -> 0x2214C400 Embedding database
 */
#define FACE_EMBEDDING_DATABASE_ADDR INPUT_ADDR_END
#define FACE_EMBEDDING_DATABASE_SIZE (256 * 512)


/* 0x2214C400 -> 0x2214C500 Cache Flush indexes
 */
#define CACHE_FLUSH_INDEXES_ADDR                                               \
	(FACE_EMBEDDING_DATABASE_ADDR + FACE_EMBEDDING_DATABASE_SIZE)
#define CACHE_FLUSH_END_ADDR (FACE_EMBEDDING_DATABASE_ADDR + 0x100)

/* 0x2214C500 -> 0x2224C500 Output Allocator */
#define OUTPUT_ALLOCATOR_BASE_ADDR CACHE_FLUSH_END_ADDR
#define OUTPUT_ALLOCATOR_SIZE (1024 * 1024)
#define OUTPUT_ALLOCATOR_ADDR_END                                              \
	(OUTPUT_ALLOCATOR_BASE_ADDR + OUTPUT_ALLOCATOR_SIZE)

/* 0x2224C500 -> 0x2234C500 Logs */
#define PRINTF_LOG_ADDR OUTPUT_ALLOCATOR_ADDR_END
#define PRINTF_LOG_SIZE 0x00100000
#define PRINTF_LOG_ADDR_END ((PRINTF_LOG_ADDR) + (PRINTF_LOG_SIZE))

/* 0x2234C500 -> 0x2234D500 Internal State */
#define INTERNAL_STATE_ADDR PRINTF_LOG_ADDR_END
#define INTERNAL_STATE_SIZE 0x000001000

#define END_PUBLIC_MEMORY_ADDR                                                 \
	(INTERNAL_STATE_ADDR + INTERNAL_STATE_SIZE)

/* TODO(jisshin): clean up space used for Citadel, please move it after internal
 * State
 */
#define WRITE_TO_CITADEL_ADDR 0x23800000
#define READ_FROM_CITADEL_ADDR 0x23801000
#endif /* __FACEAUTH_ADDRESSES_H__ */

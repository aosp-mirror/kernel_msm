/* arch/arm/mach-msm/htc_mnemosyne.h
 * Copyright (C) 2013 HTC Corporation.
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
#ifndef __MACH_MNEMOSYNE_H
#define __MACH_MNEMOSYNE_H

/* XXX: assembly do not support sizeof bulletin command,
 * set element size to calculate offset in assembly.
 */
#if defined(CONFIG_64BIT)
#define MNEMOSYNE_ELEMENT_TYPE			uint64_t	/* DO NOT FORGOT TO CHANGE ELEMENT SIZE. */
#define MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	3		/* for asm use to get shift bit for size. */
#else
/* default 32bit */
#define MNEMOSYNE_ELEMENT_TYPE			uint32_t	/* DO NOT FORGOT TO CHANGE ELEMENT SIZE. */
#define MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	2		/* for asm use to get shift bit for size. */
#endif
#define MNEMOSYNE_ELEMENT_SIZE			(1<<MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT)		/* in bytes */

#ifdef __ASSEMBLY__
#include <linux/linkage.h>
#include <linux/threads.h>

/* cannot include linux/kernel.h here, define it. */
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define DECLARE_MNEMOSYNE_START()
#define DECLARE_MNEMOSYNE(name)			ASM_ENUM	mnemosyne_##name
#define DECLARE_MNEMOSYNE_ARRAY(name, number)	ASM_ENUM_ARRAY	mnemosyne_##name, number

/* Emulate enum in assembly */
.SET LAST_ENUM_VALUE, 0

.MACRO ASM_ENUM_ARRAY name, number
/* Export the offset of an element, we can address by getting base and adding this offset */
.EQUIV \name, LAST_ENUM_VALUE * MNEMOSYNE_ELEMENT_SIZE
.SET LAST_ENUM_VALUE, LAST_ENUM_VALUE + \number
.ENDM

.MACRO ASM_ENUM name
ASM_ENUM_ARRAY \name, 1
.ENDM

#define DECLARE_MNEMOSYNE_END()

/* For physical memory space, we need to do virt_to_phys manually,
   MACRO cannot work. */

/* Implementattion of translating MPIDR_EL1 to index
 *
 * Pseudo C-code:
 *
 * void MPIDR2INDEX(u64 &dst, u64 &tmp) {
 *	 u64 cpu_idx, cluster_idx;
 *	 cpu_idx = mpidr & 0xff;
 *	 cluster_idx = (mpidr & 0xff00) >> 8;
 *	 dst = cluster_idx * 4 + cpu_idx;
 * }
 * Note: input and output registers must be disjoint register sets.
 */
	.macro MPIDR2INDEX dst, tmp
	mrs	\tmp, mpidr_el1

	and	\dst, \tmp, #0xff		/* cpu index */

	and	\tmp, \tmp, #0xff00		/* cluster index */
	add	\dst, \dst, \tmp, lsr #6	/* lsr 8 bits and lsl 2 bits. */
	.endm

	.macro VIRT2PHYS dst, virt, anchor, tmp
	ldr	\dst, =\virt
	adr	\tmp, \anchor
	add	\dst, \dst, \tmp
	ldr	\tmp, =\anchor
	sub	\dst, \dst, \tmp
	.endm

#else
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#define DECLARE_MNEMOSYNE_START()		struct mnemosyne_data {
#define DECLARE_MNEMOSYNE(name)			MNEMOSYNE_ELEMENT_TYPE name;
#define DECLARE_MNEMOSYNE_ARRAY(name, number)	MNEMOSYNE_ELEMENT_TYPE name[number];
#define DECLARE_MNEMOSYNE_END()			};
#endif /* __ASSEMBLY__ */

#include <htc_mnemosyne/htc_mnemosyne_footprint.inc>

/* For c/c++ codes, export API to set and get footprints. */
#ifndef __ASSEMBLY__
#define MNEMOSYNE_SET(f, v)		do {								\
						if (mnemosyne_get_base()) {mnemosyne_get_base()->f = (MNEMOSYNE_ELEMENT_TYPE )(v);}	\
					} while(0);

#define MNEMOSYNE_SET_I(f, i, v)	do {									\
						if (mnemosyne_get_base()) {mnemosyne_get_base()->f[i] = (MNEMOSYNE_ELEMENT_TYPE)(v);}	\
					} while(0);

#define MNEMOSYNE_GET(f)		((mnemosyne_get_base())?mnemosyne_get_base()->f:0)

#define MNEMOSYNE_GET_ADDR(f)		((mnemosyne_get_base())?&mnemosyne_get_base()->f:NULL)

#define MNEMOSYNE_GET_I(f, i)		((mnemosyne_get_base())?mnemosyne_get_base()->f[i]:0)

#define MNEMOSYNE_GET_ADDR_I(f, i)	((mnemosyne_get_base())?&mnemosyne_get_base()->f[i]:NULL)

struct mnemosyne_platform_data {
	u64 phys;
	u64 size;
};

struct mnemosyne_data *mnemosyne_get_base(void);
int mnemosyne_is_ready(void);
int mnemosyne_early_init(void);
#endif /* __ASSEMBLY__ */
#endif

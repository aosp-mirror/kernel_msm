/*
 * Generic parity calculation
 *
 * Copyright (C), 2008-2019, OPPO Mobile Comm Corp., Ltd.
 * Author: Zeng Zhaoxiu <zengzhaoxiu@oppo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

#ifndef _ASM_GENERIC_BITOPS_ARCH_PARITY_H_
#define _ASM_GENERIC_BITOPS_ARCH_PARITY_H_

#include <asm/types.h>

/*
 * Refrence to 'https://graphics.stanford.edu/~seander/bithacks.html#ParityParallel'.
 */

static inline unsigned int __arch_parity4(unsigned int w)
{
    w &= 0xf;
    return ((PARITY_MAGIC) >> w) & 1;
}

static inline unsigned int __arch_parity8(unsigned int w)
{
    w ^= w >> 4;
    return __arch_parity4(w);
}

static inline unsigned int __arch_parity16(unsigned int w)
{
    w ^= w >> 8;
    return __arch_parity8(w);
}

static inline unsigned int __arch_parity32(unsigned int w)
{
    w ^= w >> 16;
    return __arch_parity16(w);
}

static inline unsigned int __arch_parity64(__u64 w)
{
    return __arch_parity32((unsigned int)(w >> 32) ^ (unsigned int)w);
}

#endif /* _ASM_GENERIC_BITOPS_ARCH_PARITY_H_ */

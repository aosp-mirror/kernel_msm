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

#ifndef _ASM_GENERIC_BITOPS_CONST_PARITY_H_
#define _ASM_GENERIC_BITOPS_CONST_PARITY_H_

/*
 * Compile time versions of __arch_parityN()
 */
#define __const_parity4(w) (((PARITY_MAGIC) >> ((w) & 0xf)) & 1)
#define __const_parity8(w) (__const_parity4((w) ^ ((w) >> 4)))
#define __const_parity16(w) (__const_parity8((w) ^ ((w) >> 8)))
#define __const_parity32(w) (__const_parity16((w) ^ ((w) >> 16)))
#define __const_parity64(w) (__const_parity32((w) ^ ((w) >> 32)))

/*
 * Generic interface.
 */
#define parity4(w) (__builtin_constant_p(w) ? __const_parity4(w) : __arch_parity4(w))
#define parity8(w) (__builtin_constant_p(w) ? __const_parity8(w) : __arch_parity8(w))
#define parity16(w) (__builtin_constant_p(w) ? __const_parity16(w) : __arch_parity16(w))
#define parity32(w) (__builtin_constant_p(w) ? __const_parity32(w) : __arch_parity32(w))
#define parity64(w) (__builtin_constant_p(w) ? __const_parity64(w) : __arch_parity64(w))

/*
 * Interface for known constant arguments
 */
#define PARITY4(w) (BUILD_BUG_ON_ZERO(!__builtin_constant_p(w)) + __const_parity4(w))
#define PARITY8(w) (BUILD_BUG_ON_ZERO(!__builtin_constant_p(w)) + __const_parity8(w))
#define PARITY16(w) (BUILD_BUG_ON_ZERO(!__builtin_constant_p(w)) + __const_parity16(w))
#define PARITY32(w) (BUILD_BUG_ON_ZERO(!__builtin_constant_p(w)) + __const_parity32(w))
#define PARITY64(w) (BUILD_BUG_ON_ZERO(!__builtin_constant_p(w)) + __const_parity64(w))

/*
 * Type invariant interface to the compile time constant parity functions.
 */
#define PARITY(w) PARITY64((u64)(w))

#endif /* _ASM_GENERIC_BITOPS_CONST_PARITY_H_ */

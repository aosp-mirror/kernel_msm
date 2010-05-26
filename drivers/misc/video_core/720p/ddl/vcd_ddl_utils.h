/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _VCD_DDL_UTILS_H_
#define _VCD_DDL_UTILS_H_

#include "vcd_ddl_core.h"
#include "vcd_ddl.h"

#define DDL_INLINE

#define DDL_ALIGN_SIZE(n_size, n_guard_bytes, n_align_mask) \
  (((u32)(n_size) + n_guard_bytes) & n_align_mask)

#define DDL_MALLOC(x)  kmalloc(x, GFP_KERNEL)
#define DDL_FREE(x)   { if ((x)) kfree((x)); (x) = NULL; }

void ddl_pmem_alloc(struct ddl_buf_addr_type *, u32, u32);

void ddl_pmem_free(struct ddl_buf_addr_type);

void ddl_get_core_start_time(u8 codec_type);

void ddl_calc_core_time(u8 codec_type);

void ddl_reset_time_variables(u8 codec_type);

#define DDL_ASSERT(x)
#define DDL_MEMSET(src, value, len) memset((src), (value), (len))
#define DDL_MEMCPY(dest, src, len)  memcpy((dest), (src), (len))

#define DDL_ADDR_IS_ALIGNED(addr, align_bytes) \
(!((u32)(addr) & ((align_bytes) - 1)))

#endif

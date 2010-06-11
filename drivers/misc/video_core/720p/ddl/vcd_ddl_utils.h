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

//TODO get rid of this hack
enum npelly_key {
	npelly_dec_dpb = 0,
	npelly_dec_ref,
	npelly_dec_h264,
	npelly_context,
	npelly_dbl,
	npelly_mpeg4,
	npelly_meta,
	npelly_debug,
	npelly_enc_dpb,
	npelly_enc_seq,
	npelly_max_key,
};

void *ddl_dma_alloc(struct ddl_dma_buffer *, size_t, enum npelly_key key);
void ddl_dma_free(struct ddl_dma_buffer *);

#ifdef CORE_TIMING_INFO
void ddl_get_core_start_time(u8 codec_type);

void ddl_calc_core_time(u8 codec_type);

void ddl_reset_time_variables(u8 codec_type);
#endif

#endif

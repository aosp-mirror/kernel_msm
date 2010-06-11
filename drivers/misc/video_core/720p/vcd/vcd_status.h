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

#ifndef _VCD_ERR_STATUS_H_
#define _VCD_ERR_STATUS_H_

#define VCD_EVT_RESP_BASE              0x1000
#define VCD_EVT_RESP_OPEN              (VCD_EVT_RESP_BASE + 0x1)
#define VCD_EVT_RESP_START             (VCD_EVT_RESP_BASE + 0x2)
#define VCD_EVT_RESP_STOP              (VCD_EVT_RESP_BASE + 0x3)
#define VCD_EVT_RESP_PAUSE             (VCD_EVT_RESP_BASE + 0x4)
#define VCD_EVT_RESP_FLUSH_INPUT_DONE  (VCD_EVT_RESP_BASE + 0x5)
#define VCD_EVT_RESP_FLUSH_OUTPUT_DONE (VCD_EVT_RESP_BASE + 0x6)
#define VCD_EVT_RESP_INPUT_FLUSHED     (VCD_EVT_RESP_BASE + 0x7)
#define VCD_EVT_RESP_OUTPUT_FLUSHED    (VCD_EVT_RESP_BASE + 0x8)
#define VCD_EVT_RESP_INPUT_DONE        (VCD_EVT_RESP_BASE + 0x9)
#define VCD_EVT_RESP_OUTPUT_DONE       (VCD_EVT_RESP_BASE + 0xa)

#define VCD_EVT_IND_BASE               0x2000
#define VCD_EVT_IND_RECONFIG           (VCD_EVT_IND_BASE + 0x1)
#define VCD_EVT_IND_HWERRFATAL         (VCD_EVT_IND_BASE + 0x2)
#define VCD_EVT_IND_RESOURCES_LOST     (VCD_EVT_IND_BASE + 0x3)

#define VCD_S_SUCCESS           0x0

#define VCD_S_ERR_BASE              0x80000000
#define VCD_ERR_FAIL                (VCD_S_ERR_BASE + 0x1)
#define VCD_ERR_ALLOC_FAIL          (VCD_S_ERR_BASE + 0x2)
#define VCD_ERR_ILLEGAL_OP          (VCD_S_ERR_BASE + 0x3)
#define VCD_ERR_ILLEGAL_PARM        (VCD_S_ERR_BASE + 0x4)
#define VCD_ERR_BAD_POINTER         (VCD_S_ERR_BASE + 0x5)
#define VCD_ERR_BAD_HANDLE          (VCD_S_ERR_BASE + 0x6)
#define VCD_ERR_NOT_SUPPORTED       (VCD_S_ERR_BASE + 0x7)
#define VCD_ERR_BAD_STATE           (VCD_S_ERR_BASE + 0x8)
#define VCD_ERR_BUSY                (VCD_S_ERR_BASE + 0x9)
#define VCD_ERR_MAX_CLIENT          (VCD_S_ERR_BASE + 0xa)
#define VCD_ERR_IFRAME_EXPECTED     (VCD_S_ERR_BASE + 0xb)
#define VCD_ERR_INTRLCD_FIELD_DROP  (VCD_S_ERR_BASE + 0xc)
#define VCD_ERR_HW_FATAL            (VCD_S_ERR_BASE + 0xd)
#define VCD_ERR_BITSTREAM_ERR       (VCD_S_ERR_BASE + 0xe)
#define VCD_FAILED(rc)   ((rc > VCD_S_ERR_BASE) ? true : false)

#endif

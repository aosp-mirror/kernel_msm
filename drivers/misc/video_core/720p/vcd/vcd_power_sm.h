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
#ifndef _VCD_POWERSM_H_
#define _VCD_POWERSM_H_

#define VCD_EVT_PWR_BASE                0x5000
#define VCD_EVT_PWR_DEV_INIT_BEGIN      (VCD_EVT_PWR_BASE + 0x1)
#define VCD_EVT_PWR_DEV_INIT_END        (VCD_EVT_PWR_BASE + 0x2)
#define VCD_EVT_PWR_DEV_INIT_FAIL       (VCD_EVT_PWR_BASE + 0x3)
#define VCD_EVT_PWR_DEV_TERM_BEGIN      (VCD_EVT_PWR_BASE + 0x4)
#define VCD_EVT_PWR_DEV_TERM_END        (VCD_EVT_PWR_BASE + 0x5)
#define VCD_EVT_PWR_DEV_TERM_FAIL       (VCD_EVT_PWR_BASE + 0x6)
#define VCD_EVT_PWR_DEV_SLEEP_BEGIN     (VCD_EVT_PWR_BASE + 0x7)
#define VCD_EVT_PWR_DEV_SLEEP_END       (VCD_EVT_PWR_BASE + 0x8)
#define VCD_EVT_PWR_DEV_SET_PERFLVL     (VCD_EVT_PWR_BASE + 0x9)
#define VCD_EVT_PWR_DEV_HWTIMEOUT       (VCD_EVT_PWR_BASE + 0xa)
#define VCD_EVT_PWR_CLNT_CMD_BEGIN      (VCD_EVT_PWR_BASE + 0xb)
#define VCD_EVT_PWR_CLNT_CMD_END        (VCD_EVT_PWR_BASE + 0xc)
#define VCD_EVT_PWR_CLNT_CMD_FAIL       (VCD_EVT_PWR_BASE + 0xd)
#define VCD_EVT_PWR_CLNT_PAUSE          (VCD_EVT_PWR_BASE + 0xe)
#define VCD_EVT_PWR_CLNT_RESUME         (VCD_EVT_PWR_BASE + 0xf)
#define VCD_EVT_PWR_CLNT_FIRST_FRAME    (VCD_EVT_PWR_BASE + 0x10)
#define VCD_EVT_PWR_CLNT_LAST_FRAME     (VCD_EVT_PWR_BASE + 0x11)
#define VCD_EVT_PWR_CLNT_ERRFATAL       (VCD_EVT_PWR_BASE + 0x12)

enum vcd_pwr_clk_state_type {
	VCD_PWRCLK_STATE_OFF = 0,
	VCD_PWRCLK_STATE_ON_NOTCLOCKED,
	VCD_PWRCLK_STATE_ON_CLOCKED,
	VCD_PWRCLK_STATE_ON_CLOCKGATED
};

#endif

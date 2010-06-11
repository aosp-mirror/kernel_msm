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
#ifndef _SCHEDULER_UTILS_H_
#define _SCHEDULER_UTILS_H_

#include "vid_frame_scheduler_api.h"

//TODO lots of low hanging fruit in here
#define SCHED_INLINE

#if DEBUG

#define SCHED_MSG_LOW(xx_fmt, ...)	printk(KERN_INFO "\n " \
					xx_fmt, ## __VA_ARGS__)
#define SCHED_MSG_MED(xx_fmt, ...)	printk(KERN_INFO "\n" \
					xx_fmt, ## __VA_ARGS__)
#define SCHED_MSG_HIGH(xx_fmt, ...)	printk(KERN_WARNING "\n" \
					xx_fmt, ## __VA_ARGS__)

#else

#define SCHED_MSG_LOW(xx_fmt...)
#define SCHED_MSG_MED(xx_fmt...)
#define SCHED_MSG_HIGH(xx_fmt...)

#endif

#define SCHED_MSG_ERR(xx_fmt, ...)	printk(KERN_ERR "\n err: " \
					xx_fmt, ## __VA_ARGS__)
#define SCHED_MSG_FATAL(xx_fmt, ...)	printk(KERN_ERR "\n<FATAL> " \
					xx_fmt, ## __VA_ARGS__)

SCHED_INLINE void SCHED_ASSERT(int val);

SCHED_INLINE int SCHED_MIN(int x, int y);

SCHED_INLINE enum sched_status SCHED_CRITSEC_CREATE(u32 **cs);

SCHED_INLINE enum sched_status SCHED_CRITSEC_RELEASE(u32 *cs);

SCHED_INLINE enum sched_status SCHED_CRITSEC_ENTER(u32 *cs);

SCHED_INLINE enum sched_status SCHED_CRITSEC_LEAVE(u32 *cs);

SCHED_INLINE void *SCHED_MALLOC(int size);

SCHED_INLINE void SCHED_FREE(void *ptr);

SCHED_INLINE void *SCHED_MEMSET(void *ptr, int val, int size);

SCHED_INLINE enum sched_status SCHED_GET_CURRENT_TIME(u32 *pn_time);

#endif

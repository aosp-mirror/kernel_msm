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
#ifndef _VCD_UTIL_H_
#define _VCD_UTIL_H_

#include "vcd_api.h"

#if DEBUG

#define VCD_MSG_LOW(xx_fmt...)		printk(KERN_INFO "\n\t* " xx_fmt)
#define VCD_MSG_MED(xx_fmt...)		printk(KERN_INFO "\n  * " xx_fmt)
#define VCD_MSG_HIGH(xx_fmt...)		printk(KERN_WARNING "\n" xx_fmt)

#else

#define VCD_MSG_LOW(xx_fmt...)
#define VCD_MSG_MED(xx_fmt...)
#define VCD_MSG_HIGH(xx_fmt...)

#endif

#define VCD_MSG_ERROR(xx_fmt...)	printk(KERN_ERR "\n err: " xx_fmt)
#define VCD_MSG_FATAL(xx_fmt...)	printk(KERN_ERR "\n<FATAL> " xx_fmt)

#define VCD_FAILED_RETURN(rc, xx_fmt...)		\
	do {						\
		if (VCD_FAILED(rc)) {			\
			printk(KERN_ERR  xx_fmt);	\
			return rc;			\
		}					\
	} while	(0)

#define VCD_FAILED_DEVICE_FATAL(rc) \
	(rc == VCD_ERR_HW_FATAL ? TRUE : FALSE)
#define VCD_FAILED_CLIENT_FATAL(rc) \
	(rc == VCD_ERR_CLIENT_FATAL ? TRUE : FALSE)

#define VCD_FAILED_FATAL(rc)  \
	((VCD_FAILED_DEVICE_FATAL(rc) || VCD_FAILED_CLIENT_FATAL(rc)) \
	? TRUE : FALSE)


#define vcd_assert()                     VCD_MSG_FATAL("ASSERT")
#define vcd_malloc(n_bytes)              kmalloc(n_bytes, GFP_KERNEL)
#define vcd_free(p_mem)                  kfree(p_mem)

#ifdef NO_IN_KERNEL_PMEM
	#define VCD_PMEM_malloc(n_bytes)         kmalloc(n_bytes, GFP_KERNEL)
	#define VCD_PMEM_free(p_mem)             kfree(p_mem)
	#define VCD_PMEM_get_physical(p_mem)     virt_to_phys(p_mem)
#else
	int vcd_pmem_alloc(u32 size, u8 **kernel_vaddr, u8 **phy_addr);
	int vcd_pmem_free(u8 *kernel_vaddr, u8 *phy_addr);
#endif

u32 vcd_critical_section_create(u32 **p_cs);
u32 vcd_critical_section_release(u32 *cs);
u32 vcd_critical_section_enter(u32 *cs);
u32 vcd_critical_section_leave(u32 *cs);

#endif

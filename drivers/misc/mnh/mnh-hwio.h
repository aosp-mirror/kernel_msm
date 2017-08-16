/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of Intel nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __MNH_HWIO_H_
#define __MNH_HWIO_H_

#include "mnh-hwio-bases.h"
#include "mnh-pcie.h"

static inline uint32_t mnh_reg_read(uint32_t addr)
{
	uint32_t data = 0;

	mnh_config_read(addr, 4, &data);

	return data;
}

static inline void mnh_reg_write(uint32_t addr, uint32_t value)
{
	mnh_config_write(addr, 4, value);
}

#define HW_IN(bAddr, mod, reg) \
	mnh_reg_read(HWIO_##mod##_##reg##_ADDR(bAddr, 0))
#define HW_INx(bAddr, mod, reg, inst) \
	mnh_reg_read(HWIO_##mod##_##reg##_ADDR(bAddr, inst))
#define HW_PRTx(bAddr, mod, reg, inst) \
	printk(#mod" "#reg" "#inst" %p\n", \
		HWIO_##mod##_##reg##_ADDR(bAddr, inst))

#define HW_INf(bAddr, mod, reg, fld) \
	((mnh_reg_read(HWIO_##mod##_##reg##_ADDR(bAddr, 0)) & \
	  HWIO_##mod##_##reg##_##fld##_FLDMASK) >> \
	 HWIO_##mod##_##reg##_##fld##_FLDSHFT)
#define HW_INxf(bAddr, mod, reg, inst, fld) \
	((mnh_reg_read(HWIO_##mod##_##reg##_ADDR(bAddr, inst)) & \
	  HWIO_##mod##_##reg##_##fld##_FLDMASK) >> \
	 HWIO_##mod##_##reg##_##fld##_FLDSHFT)

#define HW_OUT(bAddr, mod, reg, val) \
	mnh_reg_write(HWIO_##mod##_##reg##_ADDR(bAddr, 0), val)
#define HW_PRT(bAddr, mod, reg) \
	printk(#mod" "#reg" %p\n", \
	       HWIO_##mod##_##reg##_ADDR(bAddr, 0))

#define HW_OUTx(bAddr, mod, reg, inst, val) \
	mnh_reg_write(HWIO_##mod##_##reg##_ADDR(bAddr, inst), val)

#define HW_OUTf(bAddr, mod, reg, fld, val) \
	mnh_reg_write( \
		       HWIO_##mod##_##reg##_ADDR(bAddr, 0), \
		       (mnh_reg_read(HWIO_##mod##_##reg##_ADDR(bAddr, 0)) & \
			~HWIO_##mod##_##reg##_##fld##_FLDMASK) | \
		       ((val << HWIO_##mod##_##reg##_##fld##_FLDSHFT) & \
			HWIO_##mod##_##reg##_##fld##_FLDMASK))

#define HW_OUTxf(bAddr, mod, reg, inst, fld, val) \
	mnh_reg_write( \
		       HWIO_##mod##_##reg##_ADDR(bAddr, inst), \
		       (mnh_reg_read(HWIO_##mod##_##reg##_ADDR(bAddr, inst)) & \
			~HWIO_##mod##_##reg##_##fld##_FLDMASK) | \
		       ((val << HWIO_##mod##_##reg##_##fld##_FLDSHFT) & \
			HWIO_##mod##_##reg##_##fld##_FLDMASK))

#endif /* __MNH_HWIO_H_ */

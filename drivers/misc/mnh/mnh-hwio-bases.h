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

#ifndef __MNH_HWIO_BASES_H_
#define __MNH_HWIO_BASES_H_

#define MNH_BAD_ADDR (0xFFFFFFFF)

/* silicon addresses */
#define HWIO_ROM_BASE_ADDR			(0x00000000)
#define HWIO_SRAM_BASE_ADDR			(0x00100000)
#define HWIO_PCIE_EP_BASE_ADDR			(0x00200000)
#define HWIO_SCU_BASE_ADDR			(0x04003000)
#define HWIO_CPU_BASE_ADDR			(0x04004000)

#define HWIO_DDR_CTL_BASE_ADDR                  (0x04008000)
#define HWIO_DDR_PI_BASE_ADDR                   (0x04009000)
#define HWIO_DDR_PHY_BASE_ADDR                  (0x0400A000)

#define HWIO_WDT_BASE_ADDR			(0x04040000)
#define HWIO_TIMER_BASE_ADDR			(0x04041000)
#define HWIO_SPIS_BASE_ADDR			(0x04042000)
#define HWIO_SPIM_BASE_ADDR			(0x04043000)
#define HWIO_GPIO_BASE_ADDR			(0x04044000)
#define HWIO_I2C_BASE_ADDR(blki)		(0x04045000 + (blki * 0x1000))
#define HWIO_UART_BASE_ADDR(blki)		(0x04049000 + (blki * 0x1000))
#define HWIO_PMON_BASE_ADDR			(0x0404C000)
#define HWIO_AXI_BASE_ADDR			(0x0404D000)

#define HWIO_PCIE_PHY_BASE_ADDR			(0x04080000)
#define HWIO_PCIE_SS_BASE_ADDR			(0x040C0000)
#define HWIO_MIPI_TX_BASE_ADDR(blki)		(0x04010000 + (blki * 0x1000))
#define HWIO_MIPI_RX_BASE_ADDR(blki)		(0x04012000 + (blki * 0x1000))
#define HWIO_MIPI_TOP_BASE_ADDR			(0x04015000)


#define HWIO_PCIE_EP_TYPE0_HDR_BASE_ADDR  (HWIO_PCIE_EP_BASE_ADDR)
#define HWIO_PCIE_EP_SPCIE_CAP_BASE_ADDR  (HWIO_PCIE_EP_BASE_ADDR + 0x148)
#define HWIO_PCIE_EP_L1SUB_CAP_BASE_ADDR  (HWIO_PCIE_EP_BASE_ADDR + 0x170)
#define HWIO_PCIE_EP_PM_CAP_BASE_ADDR     (HWIO_PCIE_EP_BASE_ADDR + 0x40)
#define HWIO_PCIE_EP_PCIE_CAP_BASE_ADDR   (HWIO_PCIE_EP_BASE_ADDR + 0x70)
#define HWIO_PCIE_EP_AER_CAP_BASE_ADDR    (HWIO_PCIE_EP_BASE_ADDR + 0x100)
#define HWIO_PCIE_EP_MSI_CAP_BASE_ADDR    (HWIO_PCIE_EP_BASE_ADDR + 0x50)
#define HWIO_PCIE_EP_LTR_CAP_BASE_ADDR    (HWIO_PCIE_EP_BASE_ADDR + 0x168)
#define HWIO_PCIE_EP_PORT_LOGIC_BASE_ADDR (HWIO_PCIE_EP_BASE_ADDR + 0x700)

#endif /* __MNH_HWIO_BASES_H_ */

/*
 * dap_swd.h - Linux kernel modules for DAP-SWD
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

#ifndef __LINUX_DAP_SWD_H
#define __LINUX_DAP_SWD_H

#include <linux/ioctl.h>

/*
 * Default NVIC base addresses
 */
#define NVIC_Addr    0xe000e000

// NVIC: Interrupt Controller Type Register
#define NVIC_ICT       (NVIC_Addr + 0x0004)
#define INTLINESNUM    0x0000001F  // Interrupt Line Numbers

// NVIC: CPUID Base Register
#define NVIC_CPUID     (NVIC_Addr + 0x0D00)
#define CPUID_PARTNO   0x0000FFF0  // Part Number Mask
#define CPUID_REVISION 0x0000000F  // Revision Mask
#define CPUID_VARIANT  0x00F00000  // Variant Mask

// NVIC: Vector Table Offset Register
#define NVIC_VTOR      (NVIC_Addr + 0x0D08)

// NVIC: Application Interrupt/Reset Control Register
#define NVIC_AIRCR     (NVIC_Addr + 0x0D0C)
#define VECTRESET      0x00000001  // Reset Cortex-M (except Debug)
#define VECTCLRACTIVE  0x00000002  // Clear Active Vector Bit
#define SYSRESETREQ    0x00000004  // Reset System (except Debug)
#define VECTKEY        0x05FA0000  // Write Key

// NVIC: Debug Fault Status Register
#define NVIC_DFSR      (NVIC_Addr + 0x0D30)
#define HALTED         0x00000001  // Halt Flag
#define BKPT           0x00000002  // BKPT Flag
#define DWTTRAP        0x00000004  // DWT Match
#define VCATCH         0x00000008  // Vector Catch Flag
#define EXTERNAL       0x00000010  // External Debug Request

#define ARM_REG_R0          0
#define ARM_REG_R1          1
#define ARM_REG_R2          2
#define ARM_REG_R3          3
#define ARM_REG_R4          4
#define ARM_REG_R5          5
#define ARM_REG_R6          6
#define ARM_REG_R7          7
#define ARM_REG_R8          8
#define ARM_REG_R9          9
#define ARM_REG_R10         10
#define ARM_REG_R11         11
#define ARM_REG_R12         12
#define ARM_REG_R13         13
#define ARM_REG_R14         14
#define ARM_REG_R15         15
#define ARM_REG_xPSR        16      /* flags, execution number, and state information */
#define ARMV7M_REG_MSP      17      /* main SP */
#define ARMV7M_REG_PSP      18      /* process SP */
#define ARMV7M_REG_SPECIAL  20      /* PRIMASK, BASEPRI, FAULTMASK, and CONTROL */
#define ARM_REG_FPSCR       33      /* floating-point status */
#define ARM_REG_S0          64
#define ARM_REG_S1          65
#define ARM_REG_S2          66
#define ARM_REG_S3          67
#define ARM_REG_S4          68
#define ARM_REG_S5          69
#define ARM_REG_S6          70
#define ARM_REG_S7          71
#define ARM_REG_S8          72
#define ARM_REG_S9          73
#define ARM_REG_S10         74
#define ARM_REG_S11         75
#define ARM_REG_S12         76
#define ARM_REG_S13         77
#define ARM_REG_S14         78
#define ARM_REG_S15         79
#define ARM_REG_S16         80
#define ARM_REG_S17         81
#define ARM_REG_S18         82
#define ARM_REG_S19         83
#define ARM_REG_S20         84
#define ARM_REG_S21         85
#define ARM_REG_S22         86
#define ARM_REG_S23         87
#define ARM_REG_S24         88
#define ARM_REG_S25         89
#define ARM_REG_S26         90
#define ARM_REG_S27         91
#define ARM_REG_S28         92
#define ARM_REG_S29         93
#define ARM_REG_S30         94
#define ARM_REG_S31         95

#define ARM_REG_SP     ARM_REG_R13  /* stack pointer */
#define ARM_REG_LR     ARM_REG_R14  /* link register */
#define ARM_REG_PC     ARM_REG_R15  /* program counter */

/*
 * Cortex-M3 packages these four registers as bitfields
 * in one Debug Core register.  So say r0 and r2 docs;
 * it was removed from r1 docs, but still works.
 */
#define ARMV7M_REG_PRIMASK      ARMV7M_REG_SPECIAL
#define ARMV7M_PRIMASK_SHIFT    0
#define ARMV7M_PRIMASK_MASK     (1 << ARMV7M_PRIMASK_SHIFT)

#define ARMV7M_REG_BASEPRI      ARMV7M_REG_SPECIAL
#define ARMV7M_BASEPRI_SHIFT    8
#define ARMV7M_BASEPRI_MASK     (0xFF << ARMV7M_BASEPRI_SHIFT)

#define ARMV7M_REG_FAULTMASK    ARMV7M_REG_SPECIAL
#define ARMV7M_FAULTMASK_SHIFT  16
#define ARMV7M_FAULTMASK_MASK   (1 << ARMV7M_FAULTMASK_SHIFT)

#define ARMV7M_REG_CONTROL      ARMV7M_REG_SPECIAL
#define ARMV7M_CONTROL_SHIFT    24
#define ARMV7M_CONTROL_MASK     (0x3 << ARMV7M_CONTROL_SHIFT)

#define ARM_REG_D0          ARM_REG_S0
#define ARM_REG_D1          ARM_REG_S2
#define ARM_REG_D2          ARM_REG_S4
#define ARM_REG_D3          ARM_REG_S6
#define ARM_REG_D4          ARM_REG_S8
#define ARM_REG_D5          ARM_REG_S10
#define ARM_REG_D6          ARM_REG_S12
#define ARM_REG_D7          ARM_REG_S14
#define ARM_REG_D8          ARM_REG_S16
#define ARM_REG_D9          ARM_REG_S18
#define ARM_REG_D10         ARM_REG_S20
#define ARM_REG_D11         ARM_REG_S22
#define ARM_REG_D12         ARM_REG_S24
#define ARM_REG_D13         ARM_REG_S26
#define ARM_REG_D14         ARM_REG_S28
#define ARM_REG_D15         ARM_REG_S30

/* Note: The register can only be accessed in the halt state */
#define SWD_IOCTL_CONNECT       _IO('D', 0)
#define SWD_IOCTL_DISCONNECT    _IO('D', 1)
#define SWD_IOCTL_RESET_HALT    _IO('D', 2)
#define SWD_IOCTL_RUN           _IOW('D', 3, unsigned int)
#define SWD_IOCTL_HALT          _IO('D', 4)
#define SWD_IOCTL_RESET_RUN     _IO('D', 5)
#define SWD_IOCTL_WR_REG        _IOW('D', 6, unsigned int [2])
#define SWD_IOCTL_RD_REG        _IOWR('D', 7, unsigned int [2])
#define SWD_IOCTL_WR_MEM        _IOW('D', 8, unsigned int [2])
#define SWD_IOCTL_RD_MEM        _IOWR('D', 9, unsigned int [2])


struct dap_swd_platform_data {
    int gpio_clk;
    int gpio_data;
};

struct msm_gpio_reg {
	volatile void __iomem *io_reg;
	volatile void __iomem *ctl_reg;
};

#endif

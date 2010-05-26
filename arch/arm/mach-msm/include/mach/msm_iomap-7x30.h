/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * The MSM peripherals are spread all over across 768MB of physical
 * space, which makes just having a simple IO_ADDRESS macro to slide
 * them into the right virtual location rough.  Instead, we will
 * provide a master phys->virt mapping for peripherals here.
 *
 */

#ifndef __ASM_ARCH_MSM_IOMAP_7X30_H
#define __ASM_ARCH_MSM_IOMAP_7X30_H

/* Physical base address and size of peripherals.
 * Ordered by the virtual base addresses they will be mapped at.
 *
 * MSM_VIC_BASE must be an value that can be loaded via a "mov"
 * instruction, otherwise entry-macro.S will not compile.
 *
 * If you add or remove entries here, you'll want to edit the
 * msm_io_desc array in arch/arm/mach-msm/io.c to reflect your
 * changes.
 *
 */

#define MSM_VIC_BASE          IOMEM(0xF8000000)
#define MSM_VIC_PHYS          0xC0080000
#define MSM_VIC_SIZE          SZ_4K

#define MSM_CSR_BASE          IOMEM(0xF8001000)
#define MSM_CSR_PHYS          0xC0100000
#define MSM_CSR_SIZE          SZ_4K

#define MSM_TMR_PHYS          MSM_CSR_PHYS
#define MSM_TMR_BASE          MSM_CSR_BASE
#define MSM_TMR_SIZE          SZ_4K

#define MSM_GPT_BASE 	      (MSM_TMR_BASE + 0x4)
#define MSM_DGT_BASE 	      (MSM_TMR_BASE + 0x24)

#define MSM_DMOV_BASE         IOMEM(0xF8002000)
#define MSM_DMOV_PHYS         0xAC400000
#define MSM_DMOV_SIZE         SZ_4K

#define MSM_GPIO1_BASE        IOMEM(0xF8003000)
#define MSM_GPIO1_PHYS        0xAC001000
#define MSM_GPIO1_SIZE        SZ_4K

#define MSM_GPIO2_BASE        IOMEM(0xF8004000)
#define MSM_GPIO2_PHYS        0xAC101000
#define MSM_GPIO2_SIZE        SZ_4K

#define MSM_CLK_CTL_BASE      IOMEM(0xF8005000)
#define MSM_CLK_CTL_PHYS      0xAB800000
#define MSM_CLK_CTL_SIZE      SZ_4K

#define MSM_CLK_CTL_SH2_BASE  IOMEM(0xF8006000)
#define MSM_CLK_CTL_SH2_PHYS  0xABA01000
#define MSM_CLK_CTL_SH2_SIZE  SZ_4K

#define MSM_ACC_BASE          IOMEM(0xF8007000)
#define MSM_ACC_PHYS          0xC0101000
#define MSM_ACC_SIZE          SZ_4K

#define MSM_SAW_BASE          IOMEM(0xF8008000)
#define MSM_SAW_PHYS          0xC0102000
#define MSM_SAW_SIZE          SZ_4K

#define MSM_GCC_BASE	      IOMEM(0xF8009000)
#define MSM_GCC_PHYS	      0xC0182000
#define MSM_GCC_SIZE	      SZ_4K

#define MSM_TCSR_BASE	      IOMEM(0xF800A000)
#define MSM_TCSR_PHYS	      0xAB600000
#define MSM_TCSR_SIZE	      SZ_4K

#define MSM_SHARED_RAM_BASE   IOMEM(0xF8100000)
#define MSM_SHARED_RAM_PHYS   0x00100000
#define MSM_SHARED_RAM_SIZE   SZ_1M

#define MSM_UART1_PHYS        0xACA00000
#define MSM_UART1_SIZE        SZ_4K

#define MSM_UART2_PHYS        0xACB00000
#define MSM_UART2_SIZE        SZ_4K

#define MSM_UART3_PHYS        0xACC00000
#define MSM_UART3_SIZE        SZ_4K

#ifdef CONFIG_MSM_DEBUG_UART
#define MSM_DEBUG_UART_BASE   0xF9000000
#if CONFIG_MSM_DEBUG_UART == 1
#define MSM_DEBUG_UART_PHYS   MSM_UART1_PHYS
#elif CONFIG_MSM_DEBUG_UART == 2
#define MSM_DEBUG_UART_PHYS   MSM_UART2_PHYS
#elif CONFIG_MSM_DEBUG_UART == 3
#define MSM_DEBUG_UART_PHYS   MSM_UART3_PHYS
#endif
#define MSM_DEBUG_UART_SIZE   SZ_4K
#endif

#define MSM_MDC_BASE	      IOMEM(0xF8200000)
#define MSM_MDC_PHYS	      0xAA500000
#define MSM_MDC_SIZE	      SZ_1M

#define MSM_AD5_BASE          IOMEM(0xF8300000)
#define MSM_AD5_PHYS          0xA7000000
#define MSM_AD5_SIZE          (SZ_1M*13)

#define MSM_VFE_PHYS          0xA0F00000
#define MSM_VFE_SIZE          SZ_1M

#define MSM_I2C_SIZE          SZ_4K
#define MSM_I2C_PHYS          0xACD00000

#define MSM_I2C_2_PHYS	      0xACF00000
#define MSM_I2C_2_SIZE	      SZ_4K

#define MSM_QUP_PHYS          0xA8301000
#define MSM_QUP_SIZE          SZ_4K

#define MSM_GSBI_QUP_I2C_PHYS 0xA8300000
#define MSM_GSBI_QUP_I2C_SIZE 4

#define MSM_HSUSB_PHYS        0xA3600000
#define MSM_HSUSB_SIZE        SZ_1K

#define MSM_PMDH_PHYS         0xAD600000
#define MSM_PMDH_SIZE         SZ_4K

#define MSM_EMDH_PHYS         0xAD700000
#define MSM_EMDH_SIZE         SZ_4K

#define MSM_MDP_PHYS          0xA3F00000
#define MSM_MDP_SIZE          0x000F0000

#define MSM_UART1DM_PHYS      0xA0200000
#define MSM_UART2DM_PHYS      0xA0300000

#define MSM_TSSC_PHYS         0xAA300000
#define MSM_TSSC_SIZE         SZ_4K

#define MSM_SDC1_PHYS         0xA0400000
#define MSM_SDC1_SIZE         SZ_4K

#define MSM_SDC2_PHYS         0xA0500000
#define MSM_SDC2_SIZE         SZ_4K

#define MSM_SDC3_PHYS         0xA3000000
#define MSM_SDC3_SIZE         SZ_4K

#define MSM_SDC4_PHYS         0xA3100000
#define MSM_SDC4_SIZE         SZ_4K

#define MSM_NAND_PHYS         0xA0200000
#define MSM_NAND_SIZE         SZ_4K

#define MSM_PMIC_SSBI_PHYS    0xAD900000
#define MSM_PMIC_SSBI_SIZE    SZ_4K

#define MSM_GPU_REG_PHYS      0xA3500000
#define MSM_GPU_REG_SIZE      SZ_128K

#define MSM_SPI_PHYS          0xA8000000
#define MSM_SPI_SIZE          SZ_4K

#endif

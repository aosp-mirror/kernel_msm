/* drivers/mtd/devices/msm_nand.h
 *
 * Copyright (C) 2007 Google, Inc.
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
 */

#ifndef __DRIVERS_MTD_DEVICES_MSM_NAND_H
#define __DRIVERS_MTD_DEVICES_MSM_NAND_H

#include <mach/msm_iomap.h>

#define MSM_NAND_REG(off) (MSM_NAND_BASE + (off))

#define MSM_NAND_FLASH_CMD            MSM_NAND_REG(0x0000)
#define MSM_NAND_ADDR0                MSM_NAND_REG(0x0004)
#define MSM_NAND_ADDR1                MSM_NAND_REG(0x0008)
#define MSM_NAND_FLASH_CHIP_SELECT    MSM_NAND_REG(0x000C)
#define MSM_NAND_EXEC_CMD             MSM_NAND_REG(0x0010)
#define MSM_NAND_FLASH_STATUS         MSM_NAND_REG(0x0014)
#define MSM_NAND_BUFFER_STATUS        MSM_NAND_REG(0x0018)
#define MSM_NAND_DEV0_CFG0            MSM_NAND_REG(0x0020)
#define MSM_NAND_DEV0_CFG1            MSM_NAND_REG(0x0024)
#define MSM_NAND_DEV1_CFG0            MSM_NAND_REG(0x0030)
#define MSM_NAND_DEV1_CFG1            MSM_NAND_REG(0x0034)
#define MSM_NAND_READ_ID              MSM_NAND_REG(0x0040)
#define MSM_NAND_READ_STATUS          MSM_NAND_REG(0x0044)
#define MSM_NAND_CONFIG_DATA          MSM_NAND_REG(0x0050)
#define MSM_NAND_CONFIG               MSM_NAND_REG(0x0054)
#define MSM_NAND_CONFIG_MODE          MSM_NAND_REG(0x0058)
#define MSM_NAND_CONFIG_STATUS        MSM_NAND_REG(0x0060)
#define MSM_NAND_MACRO1_REG           MSM_NAND_REG(0x0064)
#define MSM_NAND_XFR_STEP1            MSM_NAND_REG(0x0070)
#define MSM_NAND_XFR_STEP2            MSM_NAND_REG(0x0074)
#define MSM_NAND_XFR_STEP3            MSM_NAND_REG(0x0078)
#define MSM_NAND_XFR_STEP4            MSM_NAND_REG(0x007C)
#define MSM_NAND_XFR_STEP5            MSM_NAND_REG(0x0080)
#define MSM_NAND_XFR_STEP6            MSM_NAND_REG(0x0084)
#define MSM_NAND_XFR_STEP7            MSM_NAND_REG(0x0088)
#define MSM_NAND_DEV_CMD0             MSM_NAND_REG(0x00A0)
#define MSM_NAND_DEV_CMD1             MSM_NAND_REG(0x00A4)
#define MSM_NAND_DEV_CMD2             MSM_NAND_REG(0x00A8)
#define MSM_NAND_DEV_CMD_VLD          MSM_NAND_REG(0x00AC)
#define MSM_NAND_EBI2_MISR_SIG_REG    MSM_NAND_REG(0x00B0)
#define MSM_NAND_EBI2_ECC_BUF_CFG     MSM_NAND_REG(0x00F0)
#define MSM_NAND_FLASH_BUFFER         MSM_NAND_REG(0x0100)

/* device commands */

#define MSM_NAND_CMD_SOFT_RESET         0x01
#define MSM_NAND_CMD_PAGE_READ          0x32
#define MSM_NAND_CMD_PAGE_READ_ECC      0x33
#define MSM_NAND_CMD_PAGE_READ_ALL      0x34
#define MSM_NAND_CMD_SEQ_PAGE_READ      0x15
#define MSM_NAND_CMD_PRG_PAGE           0x36
#define MSM_NAND_CMD_PRG_PAGE_ECC       0x37
#define MSM_NAND_CMD_PRG_PAGE_ALL       0x39
#define MSM_NAND_CMD_BLOCK_ERASE        0x3A
#define MSM_NAND_CMD_FETCH_ID           0x0B
#define MSM_NAND_CMD_STATUS             0x0C
#define MSM_NAND_CMD_RESET              0x0D

#endif

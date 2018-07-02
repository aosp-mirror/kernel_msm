/*
 * Airbrush PCIe function driver
 *
 * Copyright (C) 2018 Samsung Electronics Co. Ltd.
 *              http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ABC_PCIE_PRIVATE_H
#define __ABC_PCIE_PRIVATE_H

#include <linux/mfd/abc-pcie.h>
#include <linux/cdev.h>

struct abc_pcie_devdata {
	struct abc_device   *abc_dev;
	int		irq;
	struct cdev c_dev;
	void __iomem	*bar[6];
	uint32_t        msi;
	struct mutex	mutex;
};

/* Interrupt(MSI) from ABC to AP */
enum abc_msi_msg_t {
	ABC_MSI_0_TMU_AON,
	ABC_MSI_1_NOC_TIMEOUT,
	ABC_MSI_2_IPU_IRQ0,
	ABC_MSI_3_IPU_IRQ1,
	ABC_MSI_4_TPU_IRQ0,
	ABC_MSI_5_TPU_IRQ1,
	ABC_MSI_6_PPC_MIF,
	ABC_MSI_7_TRAINING_DONE,
	ABC_MSI_8_SPI_INTR,
	ABC_MSI_9_WDT0,
	ABC_MSI_10_PMU,
	ABC_MSI_11_RADM_CPL_TIMEOUT,
	ABC_MSI_12_RADM_QOVERFLOW,
	ABC_MSI_13_TRGT_CPL_TIMEOUT,
	ABC_MSI_14_FLUSH_DONE,
	ABC_MSI_RD_DMA_0,
	ABC_MSI_RD_DMA_1,
	ABC_MSI_RD_DMA_2,
	ABC_MSI_RD_DMA_3,
	ABC_MSI_RD_DMA_4,
	ABC_MSI_RD_DMA_5,
	ABC_MSI_RD_DMA_6,
	ABC_MSI_RD_DMA_7,
	ABC_MSI_WR_DMA_0,
	ABC_MSI_WR_DMA_1,
	ABC_MSI_WR_DMA_2,
	ABC_MSI_WR_DMA_3,
	ABC_MSI_WR_DMA_4,
	ABC_MSI_WR_DMA_5,
	ABC_MSI_WR_DMA_6,
	ABC_MSI_WR_DMA_7,
	ABC_MSI_AON_INTNC,
	INTNC_IPU_HPM_APBIF,
	INTNC_IPU_ERR,
	INTNC_TIED,
	INTNC_TPU_WIREINTERRUPT2,
	INTNC_TMU_AON,
	INTNC_WDT1_WDTINT,
	INTNC_AON_UART,
	INTNC_OTP_AON,
	INTNC_PPMU_IPU,
	INTNC_PPMU_TPU,
	INTNC_PPMU_FSYS_M,
	INTNC_PPMU_FSYS_S,
};

#endif

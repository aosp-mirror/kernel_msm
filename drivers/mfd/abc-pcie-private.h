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

#endif

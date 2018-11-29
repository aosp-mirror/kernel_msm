/*
 * Airbrush PCIe Notifier Interface
 * Note: this is not to be confused with intnc_notifier.
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ABC_PCIE_NOTIFIER_H
#define _ABC_PCIE_NOTIFIER_H

#include <linux/bitops.h>
#include <linux/notifier.h>

/**
 * Notifier types for Airbrush PCIe blocking events
 *
 * PCIE_LINK_POST_ENABLE - called when PCIe link is enabled; also
 *     called after PCIe is unmapped from EL2 and back to EL1 control.
 *
 * PCIE_LINK_PRE_DISABLE - called when PCIe link will disable; also
 *     called before PCIe will be unmapped from EL1 and entering EL2.
 */
#define ABC_PCIE_LINK_POST_ENABLE	BIT(0)
#define ABC_PCIE_LINK_PRE_DISABLE	BIT(1)

int abc_register_pcie_link_blocking_event(struct notifier_block *nb);
int abc_unregister_pcie_link_blocking_event(struct notifier_block *nb);

#endif /* _ABC_PCIE_NOTIFIER_H */

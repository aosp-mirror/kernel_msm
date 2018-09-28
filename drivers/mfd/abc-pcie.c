/*
 * Airbrush PCIe function driver
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 *
 * Author: Sayanta Pattanayak <sayanta.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#if IS_ENABLED(CONFIG_ARM64_DMA_USE_IOMMU)
#include <asm/dma-iommu.h>
#endif
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>

#include <linux/mfd/abc-pcie.h>
#include <linux/mfd/abc-pcie-sfr.h>

#include "abc-pcie-private.h"

#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))
static struct abc_device *abc_dev;

void __iomem *get_tpu_virt(void)
{
	return abc_dev->tpu_config;
}

bool abc_pcie_enumerated(void)
{
	if (!abc_dev || !abc_dev->pcie_config)
		return false;
	return true;
}

int abc_pcie_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	struct inb_region   ir;
	u32 region_offset;
#endif

	if (!abc_dev || !abc_dev->pcie_config || !abc_dev->fsys_config)
		return -EFAULT;

	if (((TPU_START) <= offset) && (offset < (IPU_START))) {
		offset -= TPU_START;
		return tpu_config_read(offset, len, data);
	} else if (((IPU_START) <= offset) && (offset < (MIF_START))) {
		offset -= IPU_START;
		return ipu_config_read(offset, len, data);
#ifndef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	} else {
		base_offset = abc_dev->base_config + offset;
#else
	} else if (((FSYS_START) <= offset) && (offset < (DBI_START))) {
		base_offset = abc_dev->fsys_config +
				(offset - FSYS_START);
	} else if (((DBI_START) <= offset) && (offset < (FSYS_NIC_GPV))) {
		base_offset = abc_dev->pcie_config +
				(offset - DBI_START);
	} else if (((AON_AXI2APB) <= offset) && (offset < (AON_NIC_GPV))) {
		base_offset = abc_dev->aon_config +
				(offset - AON_AXI2APB);
	} else {
		region_offset = offset & ~(ABC_MISC_SFR_REGION_MASK);
		ir.target_pcie_address = ABC_SFR_BASE + region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 3;
		ir.memmode = 0;
		ir.bar = 3;
		set_inbound_iatu(ir);
		region_offset = offset & (ABC_MISC_SFR_REGION_MASK);
		base_offset = abc_dev->sfr_misc_config + region_offset;
#endif
	}
	*data = readl_relaxed(base_offset);
	__iormb();
	return 0;
}

int abc_pcie_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	struct inb_region   ir;
	u32 region_offset;
#endif

	if (!abc_dev || !abc_dev->pcie_config || !abc_dev->fsys_config)
		return -EFAULT;

	if (((TPU_START) <= offset) && (offset < (IPU_START))) {
		offset -= TPU_START;
		return tpu_config_write(offset, len, data);
	} else if (((IPU_START) <= offset) && (offset < (MIF_START))) {
		offset -= IPU_START;
		return ipu_config_write(offset, len, data);
#ifndef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	} else {
		base_offset = abc_dev->base_config + offset;
#else
	} else if (((FSYS_START) <= offset) && (offset < (DBI_START))) {
		base_offset = abc_dev->fsys_config +
				(offset - FSYS_START);
	} else if (((DBI_START) <= offset) && (offset < (FSYS_NIC_GPV))) {
		base_offset = abc_dev->pcie_config +
				(offset - DBI_START);
	} else if (((AON_AXI2APB) <= offset) && (offset < (AON_NIC_GPV))) {
		base_offset = abc_dev->aon_config +
				(offset - AON_AXI2APB);
	} else {
		region_offset = offset & ~(ABC_MISC_SFR_REGION_MASK);
		ir.target_pcie_address = ABC_SFR_BASE + region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 3;
		ir.memmode = 0;
		ir.bar = 3;
		set_inbound_iatu(ir);
		region_offset = offset & (ABC_MISC_SFR_REGION_MASK);
		base_offset = abc_dev->sfr_misc_config + region_offset;
#endif
	}
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

int aon_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->aon_config)
		return -EFAULT;

	base_offset = abc_dev->aon_config + offset;
	*data = readl_relaxed(base_offset);
	__iormb();
	return 0;
}

int aon_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev || !abc_dev->aon_config)
		return -EFAULT;

	base_offset = abc_dev->aon_config + offset;
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

int ipu_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev->ipu_config)
		return -EFAULT;

	base_offset = abc_dev->ipu_config + offset;
	*data = readl_relaxed(base_offset);
	__iormb();
	return 0;
}

int ipu_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev->ipu_config)
		return -EFAULT;

	base_offset = abc_dev->ipu_config + offset;
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

int tpu_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev->tpu_config)
		return -EFAULT;

	base_offset = abc_dev->tpu_config + offset;
	*data = readl_relaxed(base_offset);
	__iormb();
	return 0;
}

int tpu_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev->tpu_config)
		return -EFAULT;

	base_offset = abc_dev->tpu_config + offset;
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

/* memory_config_read can be used to read from
 * SRAM and DRAM of ABC.
 */
int memory_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;
	struct inb_region   ir;
	u32 region_offset;

	if (!abc_dev || !abc_dev->memory_config)
		return -EFAULT;

	region_offset = offset & ~(ABC_MEMORY_REGION_MASK);

	/* Check whether mapping is required or not */
	if (abc_dev->memory_map != region_offset) {
		ir.target_pcie_address = region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 4;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
		ir.bar = 3;
#else
		ir.bar = 2;
#endif
		ir.memmode = 0;
		set_inbound_iatu(ir);
		abc_dev->memory_map = region_offset;
	}

	region_offset = offset & ABC_MEMORY_REGION_MASK;
	base_offset = abc_dev->memory_config + region_offset;
	*data = readl_relaxed(base_offset);
	__iormb();

	return 0;
}

/* memory_config_write can be used to write to
 * SRAM and DRAM of ABC.
 */
int memory_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;
	struct inb_region   ir;
	u32 region_offset;

	if (!abc_dev || !abc_dev->memory_config)
		return -EFAULT;


	region_offset = offset & ~(ABC_MEMORY_REGION_MASK);

	/* Check whether mapping is required or not */
	if (abc_dev->memory_map != region_offset) {
		ir.target_pcie_address = region_offset;
		ir.u_target_pcie_address = 0x0;
		ir.mode =  MEM_MATCH;
		ir.region = 4;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
		ir.bar = 3;
#else
		ir.bar = 2;
#endif
		ir.memmode = 0;
		set_inbound_iatu(ir);
		abc_dev->memory_map = region_offset;
	}

	region_offset = offset & ABC_MEMORY_REGION_MASK;
	base_offset = abc_dev->memory_config + region_offset;
	__iowmb();
	writel_relaxed(data, base_offset);
	return 0;
}

#if IS_ENABLED(CONFIG_AIRBRUSH_SM)
u32 abc_pcie_get_linkspeed(void)
{
	u32 link_status;

	abc_pcie_config_read(ABC_PCIE_DBI_BASE + LINK_CONTROL_LINK_STATUS_REG,
				0x0, &link_status);
	return (link_status >> 16) & LINK_SPEED;
}

u32 abc_pcie_get_linkstate(void)
{
	u32 link_state;
	u32 l1_substate;

	abc_pcie_config_read(ABC_PCIE_DBI_BASE + LINK_CONTROL_LINK_STATUS_REG,
			 0x0, &link_state);
	abc_pcie_config_read(ABC_PCIE_DBI_BASE + L1SUB_CONTROL1_REG,
			 0x0, &l1_substate);
	if (link_state & ASPM_L1_ENABLE) {
		if (l1_substate & ASPM_L1_1_ENABLE)
			return ASPM_L11;
		if (l1_substate & ASPM_L1_2_ENABLE)
			return ASPM_L12;
	}
	if (link_state & ASPM_L0s_ENABLE)
		return ASPM_L0s;

	/* If ASPM is disabled only L0 */
	return NOASPM;
}

void abc_pcie_set_linkspeed(u32 target_linkspeed)
{
	u32 link_status2;
	u32 val;
	u32 current_linkspeed;

	if (target_linkspeed == 0)
		return;

	current_linkspeed = abc_pcie_get_linkspeed();
	if (target_linkspeed == current_linkspeed)
		return;

	/* Changing the target link speed in link_control2_link_status2 */
	abc_pcie_config_read(ABC_PCIE_DBI_BASE + LINK_CONTROL2_LINK_STATUS2_REG,
			 0x0, &link_status2);
	link_status2 &= ~(TARGET_LINK_SPEED);
	link_status2 |= target_linkspeed;
	abc_pcie_config_write(
		ABC_PCIE_DBI_BASE + LINK_CONTROL2_LINK_STATUS2_REG,
		0x0, link_status2);

	/* Asserting the directed speed change */
	abc_pcie_config_read(ABC_PCIE_DBI_BASE + PORT_LOGIC_GEN2_CTRL_OFF,
			 0x0, &val);
	val |= DIRECTED_SPEED_CHANGE;
	abc_pcie_config_write(ABC_PCIE_DBI_BASE + PORT_LOGIC_GEN2_CTRL_OFF,
			  0x0, val);
}

void abc_pcie_set_linkstate(u32 target_linkstate)
{
	struct abc_pcie_pm_ctrl smctrl;
	u32 current_linkstate;

	smctrl.l0s_en = 0;
	smctrl.l1_en = 0;
	smctrl.aspm_L11 = 0;
	smctrl.aspm_L12 = 0;

	current_linkstate = abc_pcie_get_linkstate();
	if (target_linkstate == current_linkstate)
		return;

	if (target_linkstate == NOASPM) {
		abc_set_pcie_pm_ctrl(&smctrl);
		return;
	}
	/* Invalid Link State */
	if (target_linkstate ==  PM_L2)
		return;

	if (target_linkstate == ASPM_L0s) {
		smctrl.l0s_en = 1;
	} else {
		if (target_linkstate == ASPM_L11) {
			smctrl.l1_en = 1;
			smctrl.aspm_L11 = 1;
		} else if (target_linkstate == ASPM_L12) {
			smctrl.l1_en = 1;
			smctrl.aspm_L12 = 1;
		}
	}
	abc_set_pcie_pm_ctrl(&smctrl);
}

u32 string_to_integer(char *string)
{
	if (!strcmp(string, "L0s"))
		return ASPM_L0s;
	else if (!strcmp(string, "L0"))
		return NOASPM;
	else if (!strcmp(string, "L1.1"))
		return ASPM_L11;
	else if (!strcmp(string, "L1.2"))
		return ASPM_L12;
	else if (!strcmp(string, "L3") || !strcmp(string, "L2"))
		return PM_L2;
	else
		return NOASPM;
}

int abc_pcie_state_manager(const struct block_property *property, void *data)
{
	u32 target_linkstate;
	u32 current_linkstate;
	u32 target_linkspeed;
	u32 current_linkspeed;

	if (!property)
		return -EINVAL;

	target_linkstate = string_to_integer(property->substate_name);
	target_linkspeed = property->data_rate;

	/* change to the requested speed and state */
	abc_pcie_set_linkspeed(target_linkspeed);
	abc_pcie_set_linkstate(target_linkstate);

	current_linkspeed = abc_pcie_get_linkspeed();
	current_linkstate = abc_pcie_get_linkstate();

	if (current_linkspeed == target_linkspeed &&
	    current_linkstate == target_linkstate) {
		return -EINVAL;
	}
	return 0;
}
#endif

int abc_set_aspm_state(bool state)
{
	u32 aspm_sts;
	u32 aspm_l1_sub_states;

	if (state) {
		/* Enabling ASPM L0s, L1 support [1:0] = 0x3 */
		aspm_sts = readl_relaxed(abc_dev->pcie_config +
				 LINK_CONTROL_LINK_STATUS_REG);
		__iormb();
		aspm_sts &= CLEAR_ASPM;
		aspm_sts |= ENABLE_ASPM;
		__iowmb();
		writel_relaxed(aspm_sts, abc_dev->pcie_config +
		       LINK_CONTROL_LINK_STATUS_REG);

		/* Enabling ASPM L1 substates L1.1 [3:3] = 0x1, [2:2] = 0x1 */
		aspm_l1_sub_states = readl_relaxed(abc_dev->pcie_config +
					   L1SUB_CONTROL1_REG);
		__iormb();
		aspm_l1_sub_states &= CLEAR_L1_SUBSTATES;
		aspm_l1_sub_states |= ENABLE_L1_SUBSTATES;
		__iowmb();
		writel_relaxed(aspm_l1_sub_states, abc_dev->pcie_config +
		       L1SUB_CONTROL1_REG);
	} else {
		/* Disabling ASPM L0s, L1 support [1:0] = 0x0 */
		aspm_sts = readl_relaxed(abc_dev->pcie_config +
				 LINK_CONTROL_LINK_STATUS_REG);
		__iormb();
		aspm_sts &= CLEAR_ASPM;
		__iowmb();
		writel_relaxed(aspm_sts, abc_dev->pcie_config +
		       LINK_CONTROL_LINK_STATUS_REG);

		/* Disabling ASPM L1 substates L1.1 [3:3] = 0x0, [2:2] = 0x0 */
		aspm_l1_sub_states = readl_relaxed(abc_dev->pcie_config +
					   L1SUB_CONTROL1_REG);
		__iormb();
		aspm_l1_sub_states &= CLEAR_L1_SUBSTATES;
		__iowmb();
		writel_relaxed(aspm_l1_sub_states, abc_dev->pcie_config +
		       L1SUB_CONTROL1_REG);
	}

	return 0;
}

int abc_set_pcie_pm_ctrl(struct abc_pcie_pm_ctrl *pmctrl)
{
	u32 aspm_l11_l12;
	u32 l1_l0s_enable;
	u32 val;

	if (pmctrl == NULL)
		return -EINVAL;

	aspm_l11_l12 = readl_relaxed(abc_dev->pcie_config
				+ L1SUB_CONTROL1_REG);
	__iormb();
	aspm_l11_l12 &= ~(0xC);
	aspm_l11_l12 |= (pmctrl->aspm_L11 << 3);
	aspm_l11_l12 |= (pmctrl->aspm_L12 << 2);

	l1_l0s_enable = readl_relaxed(abc_dev->pcie_config
				+ LINK_CONTROL_LINK_STATUS_REG);
	__iormb();
	l1_l0s_enable &= ~(0x3);
	l1_l0s_enable |= (pmctrl->l0s_en << 0);
	l1_l0s_enable |= (pmctrl->l1_en << 1);

	__iowmb();

	/* Enabling ASPM L11 & L12, PCI_PM L11 & L12 */
	writel_relaxed(aspm_l11_l12,
		abc_dev->pcie_config + L1SUB_CONTROL1_REG);

	/* Enabling L1 or L0s or both */
	writel_relaxed(l1_l0s_enable, abc_dev->pcie_config
				+ LINK_CONTROL_LINK_STATUS_REG);
	/* LTR Enable */
	if (pmctrl->aspm_L12) {
		val = readl_relaxed(
			abc_dev->pcie_config + PCIE_CAP_DEV_CTRL_STS2_REG);
		__iormb();
		val |= LTR_ENABLE;
		__iowmb();
		writel_relaxed(val,
			abc_dev->pcie_config + PCIE_CAP_DEV_CTRL_STS2_REG);
	} else {
		/* Clearing LTR Enable bit */
		val = readl_relaxed(
			abc_dev->pcie_config + PCIE_CAP_DEV_CTRL_STS2_REG);
		__iormb();
		val &= ~(LTR_ENABLE);
		__iowmb();
		writel_relaxed(val,
			abc_dev->pcie_config + PCIE_CAP_DEV_CTRL_STS2_REG);
	}

	/* Clock Request Enable*/
	writel_relaxed(0x1, abc_dev->fsys_config + CLOCK_REQ_EN);
	return 0;
}

int set_inbound_iatu(struct inb_region ir)
{
	unsigned long flags;
	u32 val;
	u32 set_val;
	u32 config = ir.memmode;
	int bar = ir.bar;
	u32 iatu_offset = (ir.region * IATU_REGION_OFFSET);
	u32 size;

	if (bar > BAR_4) {
		pr_err("Exceeding BAR number\n");
		return -EIO;
	}

	size = abc_dev->bar_base[bar].end - abc_dev->bar_base[bar].start;

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for iATU access mode */
	val = readl_relaxed(
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	__iormb();

	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_IATU;

	__iowmb();
	writel_relaxed(set_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	/* Lower Base address */
	writel_relaxed(abc_dev->bar_base[bar].start,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_INBOUND));

	/* Upper Base address */
	writel_relaxed(0x0,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_INBOUND));

	/* Limit */
	writel_relaxed(abc_dev->bar_base[bar].start + size,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_INBOUND));

	/* Lower Target address */
	writel_relaxed(ir.target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_INBOUND));

	/* Upper Target address */
	writel_relaxed(ir.u_target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_INBOUND));

	/* Configuring Region control */
	writel_relaxed(config,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_INBOUND));

	/* Enable region */
	writel_relaxed(0x80000000,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_INBOUND));

	writel_relaxed(val,
			abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

int set_outbound_iatu(struct outb_region outreg)
{
	unsigned long flags;
	u32 val;
	u32 set_val;

	u32 iatu_offset = (outreg.region * IATU_REGION_OFFSET);

	u32 config = outreg.memmode;

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for iATU access mode */
	val = readl_relaxed(
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	__iormb();
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_IATU;

	__iowmb();
	writel_relaxed(set_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	/* Writing the lower source address */
	writel_relaxed(outreg.base_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_OUTBOUND));

	/* Writing the upper source address */
	writel_relaxed(outreg.u_base_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND));

	/* Wrting the limit register */
	writel_relaxed(outreg.limit_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_OUTBOUND));

	/* Writing the lower target address*/
	writel_relaxed(outreg.target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND));

	/* Writing the upper target address*/
	writel_relaxed(outreg.u_target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND));

	/* Configuring Region control */
	writel_relaxed(config,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_OUTBOUND));

	/* Enable region */
	writel_relaxed(0x80000000,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_OUTBOUND));

	writel_relaxed(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

#if IS_ENABLED(CONFIG_ARM64_DMA_USE_IOMMU)
static void setup_smmu(struct pci_dev *pdev)
{
	struct dma_iommu_mapping *mapping;
	int atomic_ctx = 1;
	int bypass_enable = 1;
	int ret;

/* Following taken from msm_11ad.c */
#define SMMU_BASE	0x10000000 /* Device address range base */
#define SMMU_SIZE	0x40000000 /* Device address range size */

	mapping = arm_iommu_create_mapping(&platform_bus_type,
					SMMU_BASE, SMMU_SIZE);

	if (IS_ERR_OR_NULL(mapping)) {
		ret = PTR_ERR(mapping) ?: -ENODEV;
		dev_err(&pdev->dev,
			"Failed to create IOMMU mapping (%d)\n", ret);
		return;
	}

	ret = iommu_domain_set_attr(
		mapping->domain, DOMAIN_ATTR_ATOMIC, &atomic_ctx);
	if (ret) {
		dev_err(&pdev->dev,
			"Set atomic attribute to SMMU failed (%d)\n", ret);
	}

	ret = iommu_domain_set_attr(mapping->domain,
				   DOMAIN_ATTR_S1_BYPASS,
				   &bypass_enable);
	if (ret) {
		dev_err(&pdev->dev,
			"Set bypass attribute to SMMU failed (%d)\n", ret);
	}

	ret = arm_iommu_attach_device(&pdev->dev, mapping);
	if (ret) {
		dev_err(&pdev->dev,
			"arm_iommu_attach_device failed (%d)\n", ret);
		return;
	}
	/* mahdih: investigate why this was not needed in binder */
	dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
	dev_info(&pdev->dev, "attached to IOMMU\n");
}
#endif

dma_addr_t abc_dma_map_single(void *ptr,  size_t size,
		enum dma_data_direction dir)
{
	return dma_map_single(&abc_dev->pdev->dev, ptr, size, dir);
}
dma_addr_t abc_dma_map_page(struct page *page, size_t offset, size_t size,
		enum dma_data_direction dir)
{
	return dma_map_page(&abc_dev->pdev->dev, page, offset,  size, dir);
}

void abc_dma_unmap_page(dma_addr_t addr, size_t size,
		enum dma_data_direction dir)
{
	dma_unmap_page(&abc_dev->pdev->dev, addr, size, dir);
}
void abc_dma_unmap_single(dma_addr_t addr, size_t size,
		enum dma_data_direction dir)
{
	dma_unmap_single(&abc_dev->pdev->dev, addr, size, dir);
}
void *abc_alloc_coherent(size_t size, dma_addr_t *dma_addr)
{
	return dma_alloc_coherent(&abc_dev->pdev->dev, size, dma_addr,
			GFP_KERNEL |  GFP_DMA | __GFP_ZERO
			);
}

void abc_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr)
{
	dma_free_coherent(&abc_dev->pdev->dev, size, cpu_addr, dma_addr);
}

int dma_mblk_start(uint8_t chan, enum dma_data_direction dir,
			    phys_addr_t start_addr)
{
	u32 dma_offset;
	u32 val;
	u32 set_val;
	u32 list_addr_l = LOWER((uint64_t)start_addr);
	u32 list_addr_u = UPPER((uint64_t)start_addr);
	unsigned long flags;

	if (!(dir == DMA_FROM_DEVICE || dir == DMA_TO_DEVICE))
		return -EINVAL;

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	val = readl_relaxed(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	__iormb();
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_DMA;
	__iowmb();
	writel_relaxed(set_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	if (dir == DMA_FROM_DEVICE) {
		pr_debug("DMA MBLK WRITE[EP2AP]: CH%d\n", chan);
		dma_offset = chan * DMA_WRITE_OFFSET;

		__iowmb();
		writel_relaxed(DMA_ENABLE,
			abc_dev->pcie_config + DMA_WRITE_ENGINE);
		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
			DMA_WRITE_INTERRUPT_MASK);
		writel_relaxed(0x04000308, abc_dev->pcie_config + dma_offset +
			DMA_WRITE_CHANNEL_CONTROL_1);
		writel_relaxed(list_addr_l, abc_dev->pcie_config + dma_offset +
			DMA_LLP_LOW_OFF_WRCH);
		writel_relaxed(list_addr_u, abc_dev->pcie_config + dma_offset +
			DMA_LLP_HIGH_OFF_WRCH);
		writel_relaxed(chan, abc_dev->pcie_config + DMA_WRITE_DOORBELL);
	} else {
		pr_debug("DMA MBLK READ[AP2EP]: CH%d\n", chan);
		dma_offset = chan * DMA_READ_OFFSET;

		__iowmb();
		writel_relaxed(DMA_ENABLE,
			abc_dev->pcie_config + DMA_READ_ENGINE);
		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
			DMA_READ_INTERRUPT_MASK);
		writel_relaxed(0x04000308, abc_dev->pcie_config + dma_offset +
			DMA_READ_CHANNEL_CONTROL_1);
		writel_relaxed(list_addr_l, abc_dev->pcie_config + dma_offset +
			DMA_LLP_LOW_OFF_RDCH);
		writel_relaxed(list_addr_u, abc_dev->pcie_config + dma_offset +
			DMA_LLP_HIGH_OFF_RDCH);
		writel_relaxed(chan,
			abc_dev->pcie_config + DMA_READ_DOORBELL);
	}

	__iowmb();
	writel_relaxed(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

int dma_sblk_start(uint8_t chan, enum dma_data_direction dir,
					struct dma_element_t *blk)
{
	u32 dma_offset;
	u32 val;
	u32 set_val;
	unsigned long flags;

	if (!(dir == DMA_FROM_DEVICE || dir == DMA_TO_DEVICE))
		return -EINVAL;

	spin_lock_irqsave(&abc_dev->fsys_reg_lock, flags);

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	val = readl_relaxed(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	__iormb();
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_DMA;
	__iowmb();
	writel_relaxed(set_val,
			abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	if (dir ==  DMA_TO_DEVICE) {
		dma_offset = chan * DMA_READ_OFFSET;

		__iowmb();

		writel_relaxed(DMA_ENABLE, abc_dev->pcie_config +
				DMA_READ_ENGINE);
		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
				DMA_READ_INTERRUPT_MASK);
		writel_relaxed(0x04000008, abc_dev->pcie_config + dma_offset +
				DMA_READ_CHANNEL_CONTROL_1);
		writel_relaxed(blk->len, abc_dev->pcie_config + dma_offset +
				DMA_READ_TRANSFER_SIZE);
		writel_relaxed(blk->src_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_SAR_LOW);
		writel_relaxed(blk->src_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_SAR_HIGH);
		writel_relaxed(blk->dst_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_DAR_LOW);
		writel_relaxed(blk->dst_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_READ_DAR_HIGH);
		writel_relaxed(chan, abc_dev->pcie_config + DMA_READ_DOORBELL);
	} else {
		dma_offset = chan * DMA_WRITE_OFFSET;

		__iowmb();

		writel_relaxed(DMA_ENABLE, abc_dev->pcie_config +
				DMA_WRITE_ENGINE);
		writel_relaxed(DMA_MASK, abc_dev->pcie_config +
				DMA_WRITE_INTERRUPT_MASK);
		writel_relaxed(0x04000008, abc_dev->pcie_config + dma_offset +
				DMA_WRITE_CHANNEL_CONTROL_1);
		writel_relaxed(blk->len, abc_dev->pcie_config + dma_offset +
				DMA_WRITE_TRANSFER_SIZE);
		writel_relaxed(blk->src_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_SAR_LOW);
		writel_relaxed(blk->src_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_SAR_HIGH);
		writel_relaxed(blk->dst_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_DAR_LOW);
		writel_relaxed(blk->dst_u_addr, abc_dev->pcie_config +
				dma_offset + DMA_WRITE_DAR_HIGH);
		writel_relaxed(chan, abc_dev->pcie_config + DMA_WRITE_DOORBELL);
	}

	__iowmb();
	writel_relaxed(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock_irqrestore(&abc_dev->fsys_reg_lock, flags);

	return 0;
}

int abc_reg_notifier_callback(struct notifier_block *nb)
{
	int ret = atomic_notifier_chain_register(&abc_dev->intnc_notifier, nb);

	if (ret) {
		pr_err("Could not register notifier\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(abc_reg_notifier_callback);

int abc_reg_irq_callback(irq_cb_t sys_cb, int irq_no)
{
	unsigned long flags;

	if (irq_no >= ABC_MSI_0_TMU_AON && irq_no <= ABC_MSI_14_FLUSH_DONE) {
		spin_lock_irqsave(&abc_dev->lock, flags);
		abc_dev->sys_cb[irq_no] = sys_cb;
		spin_unlock_irqrestore(&abc_dev->lock, flags);
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(abc_reg_irq_callback);

int abc_reg_dma_irq_callback(irq_dma_cb_t dma_cb, int dma_chan)
{
	unsigned long flags;

	spin_lock_irqsave(&abc_dev->dma_callback_lock, flags);

	if (dma_chan >= 0 && dma_chan <= 7)
		abc_dev->dma_cb[dma_chan] = dma_cb;

	spin_unlock_irqrestore(&abc_dev->dma_callback_lock, flags);

	return 0;
}

static irqreturn_t abc_pcie_dma_irq_handler(int irq, void *ptr)
{
	u32 override_val;
	u32 override_set_val;
	u32 dma_chan;
	u32 dma_rd_stat;
	u32 dma_wr_stat;
	int pos;

	spin_lock(&abc_dev->fsys_reg_lock);

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	override_val = readl_relaxed(abc_dev->fsys_config +
					SYSREG_FSYS_DBI_OVERRIDE);
	__iormb();
	override_set_val = override_val & ~(DBI_OVERRIDE_MASK);
	override_set_val = DBI_OVERRIDE_DMA;
	__iowmb();
	writel_relaxed(override_set_val, abc_dev->fsys_config +
					SYSREG_FSYS_DBI_OVERRIDE);

	/* DMA Read Callback Implementation */
	dma_rd_stat = readl_relaxed(
		abc_dev->pcie_config + DMA_READ_INT_STATUS_OFF);
	__iormb();
	pos = 0;
	pr_debug("---dma_rd_stat = 0x%x--\n", dma_rd_stat);
	while ((pos = find_next_bit((unsigned long *) &dma_rd_stat, 32,
					pos)) != 32) {
		__iowmb();
		writel_relaxed((0x1 << pos),
				abc_dev->pcie_config + DMA_READ_INT_CLEAR_OFF);

		spin_lock(&abc_dev->dma_callback_lock);

		if (pos >= 0 && pos <= 7) {
			dma_chan = pos;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan,
						DMA_TO_DEVICE, DMA_DONE);
		} else if (pos >= 16 && pos <= 23) {
			dma_chan = pos - 16;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan,
						DMA_TO_DEVICE, DMA_ABORT);
		}

		spin_unlock(&abc_dev->dma_callback_lock);

		pos++;
	}

	/* DMA Write Callback Implementation */
	dma_wr_stat = readl_relaxed(
		abc_dev->pcie_config + DMA_WRITE_INT_STATUS_OFF);
	__iormb();
	pos = 0;
	pr_debug("---dma_wr_stat = 0x%x--\n", dma_wr_stat);
	while ((pos = find_next_bit((unsigned long *) &dma_wr_stat, 32,
					pos)) != 32) {
		__iowmb();
		writel_relaxed((0x1 << pos),
				abc_dev->pcie_config + DMA_WRITE_INT_CLEAR_OFF);

		spin_lock(&abc_dev->dma_callback_lock);

		if (pos >= 0 && pos <= 7) {
			dma_chan = pos;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan,
						DMA_FROM_DEVICE, DMA_DONE);
		} else if (pos >= 16 && pos <= 23) {
			dma_chan = pos - 16;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan,
						DMA_FROM_DEVICE, DMA_ABORT);
		}

		spin_unlock(&abc_dev->dma_callback_lock);

		pos++;
	}

	__iowmb();
	writel_relaxed(override_val,
		abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	spin_unlock(&abc_dev->fsys_reg_lock);

	return IRQ_HANDLED;
}

static irqreturn_t abc_pcie_irq_handler(int irq, void *ptr)
{
	struct pci_dev *pdev = ptr;
	u32 intnc_val;
	u32 msi_cap_val;

	irq = irq - pdev->irq;
	pr_info_ratelimited("MSI Irq number is : %d\n", irq);
	spin_lock(&abc_dev->lock);

	if (irq >= ABC_MSI_0_TMU_AON && irq <= ABC_MSI_14_FLUSH_DONE) {
		if (abc_dev->sys_cb[irq] != NULL) {
			/* Callback respective handler */
			abc_dev->sys_cb[irq](irq);
		}
	} else if (irq == ABC_MSI_AON_INTNC) {
		/* Mask 31st MSI during Interrupt handling period */
		msi_cap_val = readl_relaxed(
				abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);
		__iormb();
		__iowmb();
		writel_relaxed((msi_cap_val | MSI_CAP_MASK_31),
			abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);

		/* Check the sysreg status register and do the callback */
		intnc_val = readl_relaxed(
			abc_dev->fsys_config + SYSREG_FSYS_INTERRUPT);
		__iormb();
		atomic_notifier_call_chain(&abc_dev->intnc_notifier,
					   irq, (void *)(u64)intnc_val);
		__iowmb();
		writel_relaxed(msi_cap_val,
			abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);
	}
	spin_unlock(&abc_dev->lock);
	return IRQ_HANDLED;
}

/* TODO(basso):  This is temporary code to enable power and clocks to the IPU
 * and TPU.  This should be replaced with proper regulator and clock control
 * interfaces.
 */
#define CLK_CON_DIV_PLL_AON_CLK 0x00B1180C
#define CLK_CON_DIV_DIV4_PLLCLK_TPU 0x00041800
#define CLK_CON_DIV_DIV4_PLLCLK_IPU 0x00241800
#define PLL_CON0_PLL_IPU 0x00240120
#define CLK_CON_MUX_MOUT_AONCLK_PLLCLK1 0x00241000
#define REG_PCIe_INIT 0x00B30388
#define PMU_CONTROL 0x00BA0004
#define PMU_CONTROL_PHY_RET_OFF (1 << 8)
#define PMU_CONTROL_BLK_TPU_UP_REQ (1 << 1)
#define PMU_CONTROL_BLK_IPU_UP_REQ (1 << 0)
#define TIMEOUT_POWERSWITCH_HANDSHAKE_IPU 0x00BA3530
#define TIMEOUT_POWERSWITCH_HANDSHAKE_TPU 0x00BA3534

static int abc_pcie_ipu_tpu_enable(void)
{
	int err;

	err = abc_pcie_config_write(TIMEOUT_POWERSWITCH_HANDSHAKE_IPU, 4,
			0x00000001);
	if (WARN_ON(err < 0))
		return err;

	err = abc_pcie_config_write(TIMEOUT_POWERSWITCH_HANDSHAKE_TPU, 4,
			0x00000001);
	if (WARN_ON(err < 0))
		return err;

	err = abc_pcie_config_write(REG_PCIe_INIT, 4, 0x00000001);
	if (WARN_ON(err < 0))
		return err;

	err = abc_pcie_config_write(PMU_CONTROL, 4, PMU_CONTROL_PHY_RET_OFF |
			PMU_CONTROL_BLK_TPU_UP_REQ |
			PMU_CONTROL_BLK_IPU_UP_REQ);
	if (WARN_ON(err < 0))
		return err;

	/* TODO(basso):  Determine the proper timeout here. */
	usleep_range(1000, 2000);

	/* Reduce AON clk to 233MHz to safely make IPU/TPU APB clk changes */
	err = abc_pcie_config_write(CLK_CON_DIV_PLL_AON_CLK, 4, 0x00000003);
	if (WARN_ON(err < 0))
		return err;

	err = abc_pcie_config_write(CLK_CON_DIV_DIV4_PLLCLK_IPU, 4, 0x00000003);
	if (WARN_ON(err < 0))
		return err;

	err = abc_pcie_config_write(CLK_CON_DIV_DIV4_PLLCLK_TPU, 4, 0x00000003);
	if (WARN_ON(err < 0))
		return err;

	/* Configure IPU PLL for 549 MHz */
	err = abc_pcie_config_write(PLL_CON0_PLL_IPU, 4, 0x81CA0203);
	if (WARN_ON(err < 0))
		return err;

	/* TODO(basso):  Determine the proper timeout here. */
	usleep_range(1000, 2000);

	err = abc_pcie_config_write(PLL_CON0_PLL_IPU, 4, 0x81CA0213);
	if (WARN_ON(err < 0))
		return err;

	err = abc_pcie_config_write(CLK_CON_MUX_MOUT_AONCLK_PLLCLK1, 4, 0x1);
	if (WARN_ON(err < 0))
		return err;

	/* Restore AON clk to 933MHz */
	err = abc_pcie_config_write(CLK_CON_DIV_PLL_AON_CLK, 4, 0);
	if (WARN_ON(err < 0))
		return err;

	/* TODO(basso):  Determine the proper timeout here. */
	usleep_range(1000, 2000);

	return 0;
}

uint32_t abc_pcie_irq_init(struct pci_dev *pdev)
{
	int err, vector, i;
	struct abc_pcie_devdata *abc = dev_get_drvdata(&pdev->dev);

	abc->msi = pci_msi_vec_count(pdev);

	vector = pci_alloc_irq_vectors(pdev, 1, abc->msi, PCI_IRQ_MSI);
	if (vector < 0) {
		dev_err(&pdev->dev, "failed to get MSI interrupts\n");
		return vector;
	}

	/* MSI IRQs Request for system IP's */
	for (i = ABC_MSI_0_TMU_AON; i < ABC_MSI_RD_DMA_0; i++) {
		/*
		 * ABC_MSI_2_IPU_IRQ0 and ABC_MSI_3_IPU_IRQ1 are registered by
		 * the paintbox IPU driver.  Similar for the 2 high prio
		 * TPU interrupts.
		 */
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1 ||
		    i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1)
			continue;
		err = request_irq(pdev->irq + i,
				abc_pcie_irq_handler, 0,
				DRV_NAME_ABC_PCIE, pdev);
		if (err) {
			dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
					pdev->irq + i, err);
			goto free_irq;
		}
		dev_dbg(&pdev->dev, "request irq:%d\n", pdev->irq+i);
	}

	/* MSI IRQs request */
	for (i = ABC_MSI_RD_DMA_0; i <= ABC_MSI_WR_DMA_7; i++) {
		err = request_irq(pdev->irq + i,
				abc_pcie_dma_irq_handler, 0,
				DRV_NAME_ABC_PCIE, pdev);

		if (err) {
			dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
					pdev->irq + i, err);
			goto free_irq;
		}
		dev_dbg(&pdev->dev, "request irq:%d\n", pdev->irq+i);
	}

	err = request_irq(pdev->irq + ABC_MSI_AON_INTNC,
			abc_pcie_irq_handler, 0,
			DRV_NAME_ABC_PCIE, pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
				pdev->irq + ABC_MSI_AON_INTNC, err);
		goto free_irq;
	}
	dev_dbg(&pdev->dev, "request irq:%d\n", pdev->irq+ABC_MSI_AON_INTNC);
	return 0;

free_irq:
	for (--i; i >= ABC_MSI_0_TMU_AON; i--) {
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1 ||
		    i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1)
			continue;
		free_irq(pdev->irq + i, pdev);
	}

	pci_free_irq_vectors(pdev);
	return err;

}

static void abc_pcie_irq_free(struct pci_dev *pdev)
{
	unsigned int i;

	for (i = ABC_MSI_0_TMU_AON; i <= ABC_MSI_AON_INTNC; i++) {
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1 ||
		    i == ABC_MSI_4_TPU_IRQ0 || i == ABC_MSI_5_TPU_IRQ1)
			continue;
		free_irq(pdev->irq + i, pdev);
	}

	pci_free_irq_vectors(pdev);
}

static const struct resource ipu_resources[] = {
	{
		.name = DRV_NAME_ABC_PCIE_IPU,
		.start = ABC_MSI_2_IPU_IRQ0,
		.end = ABC_MSI_2_IPU_IRQ0,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = DRV_NAME_ABC_PCIE_IPU,
		.start = ABC_MSI_3_IPU_IRQ1,
		.end = ABC_MSI_3_IPU_IRQ1,
		.flags = IORESOURCE_IRQ,
	}
};

#define TPU_MEM_MAPPING		0 /* index of tpu-mem-mapping property */
#define TPU_INTNC_NOTIFIER	1 /* index of intnc-notifier-chain tpu prop */
static struct property_entry tpu_properties[] = {
	/* filled in with VA of ioremap of tpu-mem */
	PROPERTY_ENTRY_U64("tpu-mem-mapping", 0),
	/* filled in with notifier chain for mux'ed low-priority interrupts */
	PROPERTY_ENTRY_U64("intnc-notifier-chain", 0),
	{ }
};

static struct resource tpu_resources[] = {
	{
		.name = "tpu-mem",
		.start = 0,
		.end = (2 * 1024 * 1024) - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "tpu-scalar-core-0-irq",
		.start = ABC_MSI_4_TPU_IRQ0,
		.end = ABC_MSI_4_TPU_IRQ0,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "tpu-instr-queue-irq",
		.start = ABC_MSI_5_TPU_IRQ1,
		.end = ABC_MSI_5_TPU_IRQ1,
		.flags = IORESOURCE_IRQ,
	},
};
static const struct resource fsys_resources[] = {
};

static const struct resource pcie_dma_resources[] = {
	{
		.name = DRV_NAME_ABC_PCIE_DMA,
		.start = 0, /* tbd */
		.end = (2 * 1024 * 1024) - 1, /* tbd */
		.flags = IORESOURCE_MEM, /* tbd */
	}
};

#define DEV(_name, _r) \
	{ .name = _name, .num_resources = ARRAY_SIZE(_r), .resources = _r, }
#define DEVPROP(_name, _r, _p)						   \
	{ .name = _name, .num_resources = ARRAY_SIZE(_r), .resources = _r,   \
	      .properties = _p }

static struct mfd_cell abc_pcie_bar0[] = {
	DEVPROP(DRV_NAME_ABC_PCIE_TPU, tpu_resources, tpu_properties),
	DEV(DRV_NAME_ABC_PCIE_IPU, ipu_resources),
#ifndef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	DEV(DRV_NAME_ABC_PCIE_BLK_FSYS, fsys_resources),
	DEV(DRV_NAME_ABC_PCIE_DMA, pcie_dma_resources),
#endif
};

#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
static struct mfd_cell abc_pcie_bar2[] = {
	DEV(DRV_NAME_ABC_PCIE_BLK_FSYS, fsys_resources),
	DEV(DRV_NAME_ABC_PCIE_DMA, pcie_dma_resources)
};
#endif

static const struct pci_device_id abc_pcie_ids[] = {
	/* ABC Vendor and DeviceID */
	{ PCI_DEVICE(0x1ae0, 0xabcd) },
	{ 0, }
};

static int abc_pcie_init_child_devices(struct pci_dev *pdev)
{
	int err;

	/* fill in tpu-mem-mapping with VA of our mapping for tpu-mem */
	tpu_properties[TPU_MEM_MAPPING].value.u64_data =
	    (u64)abc_dev->tpu_config;
	/* fill in address of notifier block for the INTNC mux'ed IRQ */
	tpu_properties[TPU_INTNC_NOTIFIER].value.u64_data =
	    (u64)&abc_dev->intnc_notifier;

	err = mfd_add_devices(&pdev->dev, PLATFORM_DEVID_NONE, abc_pcie_bar0,
			ARRAY_SIZE(abc_pcie_bar0), &pdev->resource[0],
			pdev->irq, NULL);
	if (err < 0) {
		dev_err(&pdev->dev, "mfd_add_devices[0] failed: %d\n", err);
		return err;
	}

#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	err = mfd_add_devices(&pdev->dev, PLATFORM_DEVID_NONE, abc_pcie_bar2,
			ARRAY_SIZE(abc_pcie_bar2), &pdev->resource[2], 0, NULL);
	if (err < 0) {
		mfd_remove_devices(&pdev->dev);
		dev_err(&pdev->dev, "mfd_add_devices[2] failed: %d\n", err);
		return err;
	}
#endif

	return 0;
}

static int abc_pcie_probe(struct pci_dev *pdev,
				   const struct pci_device_id *ent)
{
	int i;
	int err;
	int bar;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct abc_pcie_devdata *abc;
	struct resource *res;
	struct pci_bus_region region;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	abc = kzalloc(sizeof(struct abc_pcie_devdata), GFP_KERNEL);
	if (abc == NULL)
		return -ENOMEM;

	abc_dev = kzalloc(sizeof(struct abc_device), GFP_KERNEL);
	if (abc_dev == NULL) {
		err = -ENOMEM;
		goto err1;
	}

	abc_dev->pdev = pdev;
	abc_dev->dev  = dev;
	abc->abc_dev = abc_dev;

	/* Assigning abc_pcie_devdata as driver data to abc_pcie driver */
	dev_set_drvdata(&pdev->dev, abc);

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Cannot enable PCI device\n");
		goto err2;
	}

	err = pci_request_regions(pdev, DRV_NAME_ABC_PCIE);
	if (err) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err3;
	}

	pci_set_master(pdev);

	spin_lock_init(&abc_dev->lock);
	spin_lock_init(&abc_dev->fsys_reg_lock);
	spin_lock_init(&abc_dev->dma_callback_lock);

	bar = 0;
	/* Restricting till BAR4 mapping */
	while (bar < 5) {
		base = pci_ioremap_bar(pdev, bar);
		if (!base) {
			dev_warn(dev, "failed to read BAR%d\n", bar);
			if (bar >= 5)
				goto exit_loop;
			WARN_ON(bar == BAR_0);
		}
		abc->bar[bar] = base;
		res = &pdev->resource[bar];

#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
		if (bar == 0) {
			/* In current setup BAR0 Mapped for IPU & TPU */
			abc_dev->tpu_config = base;
			/* IPU is located at 2MB from TPU block */
			abc_dev->ipu_config = base + (2 * 1024 * 1024);
		}
		if (bar == 2) {
			/* In current setup BAR2 Mapped for FSYS & DBI */
			abc_dev->fsys_config = base;
			/* PCIe-DBI base address is at 1MB from FSYS base */
			abc_dev->pcie_config = base + (1024 * 1024);
			for (i = 0; i < ARRAY_SIZE(abc_pcie_bar2); i++) {
				/* Assigning abc_device as platform data to
				 * abc_pcie mfd cell drivers
				 */
				abc_pcie_bar2[i].platform_data = abc_dev;
				abc_pcie_bar2[i].pdata_size = sizeof(*abc_dev);
			}
		}
		if (bar == 4) {
			/* In current setup BAR4 Mapped for AON */
			abc_dev->aon_config = base;
		}
		if (bar == 3)
			abc_dev->memory_config = base;
#else
		if (bar == 0) {
			/* Mapping complete SFR region into single BAR */
			abc_dev->base_config = base;
			abc_dev->tpu_config = base;
			abc_dev->ipu_config = base + IPU_START;
			abc_dev->fsys_config = base + FSYS_START;
			abc_dev->pcie_config = base + DBI_START;
			abc_dev->aon_config = base + AON_AXI2APB;
			for (i = 0; i < ARRAY_SIZE(abc_pcie_bar0); i++) {
				/* Assigning abc_device as platform data to
				 * abc_pcie mfd cell drivers
				 */
				abc_pcie_bar0[i].platform_data = abc_dev;
				abc_pcie_bar0[i].pdata_size = sizeof(*abc_dev);
			}
		}
		if (bar == 2) {
			abc_dev->bar2_base = base;
			abc_dev->memory_config = base;
		}
		if (bar == 4)
			abc_dev->bar4_base = base;
#endif
		pcibios_resource_to_bus(pdev->bus, &region, res);
		abc_dev->bar_base[bar].start = region.start;
		abc_dev->bar_base[bar].end   = region.end;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
		bar++;
#else
		bar += 2;
#endif
	}

exit_loop:
	/* IRQ handling */
#if CONFIG_PCI_MSI
	err = abc_pcie_irq_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "abc_irq_init failed\n");
		goto err4;
	}
#endif
#if IS_ENABLED(CONFIG_AIRBRUSH_SM)
	/* Registering the callback to the ASM */
	ab_sm_register_blk_callback((block_name_t)BLK_FSYS,
			abc_pcie_state_manager, (void *)pdev);
#endif
	pci_set_drvdata(pdev, abc);

#if IS_ENABLED(CONFIG_ARM64_DMA_USE_IOMMU)
	setup_smmu(pdev);
#endif

	err = abc_pcie_ipu_tpu_enable();
	if (err < 0)
		goto err5;

	ATOMIC_INIT_NOTIFIER_HEAD(&abc_dev->intnc_notifier);
	err = abc_pcie_init_child_devices(pdev);
	if (err < 0)
		goto err5;

	return 0;
err5:
	abc_pcie_irq_free(pdev);
err4:
	pci_release_regions(pdev);
err3:
	pci_disable_device(pdev);
err2:
	kfree(abc_dev);
err1:
	kfree(abc);

	return err;
}

static void abc_pcie_remove(struct pci_dev *pdev)
{
	enum pci_barno bar;
	struct abc_pcie_devdata *abc = pci_get_drvdata(pdev);

	mfd_remove_devices(&pdev->dev);

	for (bar = BAR_0; bar <= BAR_4; bar += 2) {
		if (abc->bar[bar])
			iounmap(abc->bar[bar]);
	}

	abc_pcie_irq_free(pdev);
	pci_set_drvdata(pdev, NULL);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	kfree(abc_dev);
	kfree(abc);
}

static struct pci_driver abc_pcie_driver = {
	.name = DRV_NAME_ABC_PCIE,
	.id_table = abc_pcie_ids,
	.probe = abc_pcie_probe,
	.remove = abc_pcie_remove,
	.shutdown = abc_pcie_remove,
};

static int __init abc_pcie_init(void)
{
	return pci_register_driver(&abc_pcie_driver);
}

static void __exit abc_pcie_exit(void)
{
	pci_unregister_driver(&abc_pcie_driver);
}
module_exit(abc_pcie_exit);
module_init(abc_pcie_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sayanta Pattanayak <sayanta.p@samsung.com>");
MODULE_DESCRIPTION("Airbrush PCI-e function Driver");

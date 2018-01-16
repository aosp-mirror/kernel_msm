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
int pcie_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	struct inb_region   ir;
	u32 region_offset;
#endif

	if (!abc_dev->pcie_config || !abc_dev->fsys_config)
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
	*data = readl(base_offset);
	return 0;
}

int pcie_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;
#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	struct inb_region   ir;
	u32 region_offset;
#endif

	if (!abc_dev->pcie_config || !abc_dev->fsys_config)
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
	writel(data, base_offset);
	return 0;
}

int aon_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	base_offset = abc_dev->aon_config + offset;
	*data = readl(base_offset);
	return 0;
}

int aon_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	base_offset = abc_dev->aon_config + offset;
	writel(data, base_offset);
	return 0;
}

int ipu_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev->ipu_config)
		return -EFAULT;

	base_offset = abc_dev->ipu_config + offset;
	*data = readl(base_offset);
	return 0;
}

int ipu_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev->ipu_config)
		return -EFAULT;

	base_offset = abc_dev->ipu_config + offset;
	writel(data, base_offset);
	return 0;
}

int tpu_config_read(u32 offset, u32 len, u32 *data)
{
	void __iomem *base_offset;

	if (!abc_dev->tpu_config)
		return -EFAULT;

	base_offset = abc_dev->tpu_config + offset;
	*data = readl(base_offset);
	return 0;
}

int tpu_config_write(u32 offset, u32 len, u32 data)
{
	void __iomem *base_offset;

	if (!abc_dev->tpu_config)
		return -EFAULT;

	base_offset = abc_dev->tpu_config + offset;
	writel(data, base_offset);
	return 0;
}

int abc_set_pcie_pm_ctrl(struct abc_pcie_pm_ctrl *pmctrl)
{
	u32 aspm_l11_l12;
	u32 l1_l0s_enable;
	u32 pme_en;

	if (pmctrl == NULL)
		return -EINVAL;

	aspm_l11_l12 = readl(abc_dev->pcie_config
				+ L1SUB_CONTROL1_REG);
	aspm_l11_l12 &= ~(0xC);
	aspm_l11_l12 |= (pmctrl->aspm_L11 << 2);
	aspm_l11_l12 |= (pmctrl->aspm_L12 << 3);
	/*todo. Need to validate the following configuration further */
	aspm_l11_l12 |= 0xA00;

	l1_l0s_enable = readl(abc_dev->pcie_config
				+ LINK_CONTROL_LINK_STATUS_REG);
	l1_l0s_enable &= ~(0x3);
	l1_l0s_enable |= (pmctrl->l0s_en << 0);
	l1_l0s_enable |= (pmctrl->l1_en << 1);
	/*todo. Need to validate the following configuration further */
	l1_l0s_enable |= 0x70120000;

	pme_en = readl(abc_dev->pcie_config + PME_EN);
	pme_en &= ~(0x100);
	pme_en |= (pmctrl->pme_en << 8);

	/* Enabling ASPM L11 & L12, PCI_PM L11 & L12 */
	writel(aspm_l11_l12, abc_dev->pcie_config + L1SUB_CONTROL1_REG);

	/* Enabling L1 or L0s or both */
	writel(l1_l0s_enable, abc_dev->pcie_config
				+ LINK_CONTROL_LINK_STATUS_REG);

	/* Clock Request Enable*/
	writel(0x1, abc_dev->fsys_config + CLOCK_REQ_EN);

	/* PME enable */
	writel(pme_en, abc_dev->pcie_config + PME_EN);

	pr_info("Inside ASPM\n");
	return 0;
}

int set_inbound_iatu(struct inb_region inb)
{
	u32 val;
	u32 set_val;
	u32 config = inb.memmode;
	int bar = inb.bar;
	u32 iatu_offset = (inb.region * IATU_REGION_OFFSET);
	u32 size;

	if (bar > BAR_4) {
		pr_err("Exceeding BAR number\n");
		return -1;
	}
	size = abc_dev->bar_base[bar].end - abc_dev->bar_base[bar].start;

	/* Set SYSREG_FSYS DBI_OVERRIDE for iATU access mode */
	val = readl(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_IATU;
	writel(set_val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	/* Lower Base address */
	writel(abc_dev->bar_base[bar].start,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_INBOUND));

	/* Upper Base address */
	writel(0x0,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_INBOUND));

	/* Limit */
	writel(abc_dev->bar_base[bar].start + size,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_INBOUND));

	/* Lower Target address */
	writel(inb.target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_INBOUND));

	/* Upper Target address */
	writel(inb.u_target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_INBOUND));

	/* Configuring Region control */
	writel(config,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_INBOUND));

	/* Enable region */
	writel(0x80000000,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_INBOUND));

	writel(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	return 0;
}

int set_outbound_iatu(struct outb_region outb)
{
	u32 val;
	u32 set_val;

	u32 iatu_offset = (outb.region * IATU_REGION_OFFSET);

	u32 config = outb.memmode;

	/* Set SYSREG_FSYS DBI_OVERRIDE for iATU access mode */
	val = readl(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_IATU;
	writel(set_val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	/* Writing the lower source address */
	writel(outb.base_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_BASE_ADDR_OFF_OUTBOUND));

	/* Writing the upper source address */
	writel(outb.u_base_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND));

	/* Wrting the limit register */
	writel(outb.limit_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LIMIT_ADDR_OFF_OUTBOUND));

	/* Writing the lower target address*/
	writel(outb.target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND));

	/* Writing the upper target address*/
	writel(outb.u_target_pcie_address,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND));

	/* Configuring Region control */
	writel(config,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_1_OFF_OUTBOUND));

	/* Enable region */
	writel(0x80000000,
	       (abc_dev->pcie_config + iatu_offset +
		PF0_ATU_CAP_IATU_REGION_CTRL_2_OFF_OUTBOUND));

	writel(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
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

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	val = readl(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_DMA;
	writel(set_val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	if (dir == DMA_FROM_DEVICE) {
		pr_info("DMA MBLK WRITE[EP2AP]: CH%d\n", chan);
		dma_offset = chan * DMA_WRITE_OFFSET;
		writel(DMA_ENABLE, abc_dev->pcie_config + DMA_WRITE_ENGINE);
		writel(DMA_MASK, abc_dev->pcie_config +
						DMA_WRITE_INTERRUPT_MASK);
		writel(0x04000308, abc_dev->pcie_config + dma_offset +
						DMA_WRITE_CHANNEL_CONTROL_1);
		writel(list_addr_l, abc_dev->pcie_config + dma_offset +
						DMA_LLP_LOW_OFF_WRCH);
		writel(list_addr_u, abc_dev->pcie_config + dma_offset +
						DMA_LLP_HIGH_OFF_WRCH);
		writel(chan, abc_dev->pcie_config + DMA_WRITE_DOORBELL);
	} else if (dir == DMA_TO_DEVICE) {
		pr_info("DMA MBLK READ[AP2EP]: CH%d\n", chan);
		dma_offset = chan * DMA_READ_OFFSET;
		writel(DMA_ENABLE, abc_dev->pcie_config + DMA_READ_ENGINE);
		writel(DMA_MASK, abc_dev->pcie_config +
						DMA_READ_INTERRUPT_MASK);
		writel(0x04000308, abc_dev->pcie_config + dma_offset +
						DMA_READ_CHANNEL_CONTROL_1);
		writel(list_addr_l, abc_dev->pcie_config + dma_offset +
						DMA_LLP_LOW_OFF_RDCH);
		writel(list_addr_u, abc_dev->pcie_config + dma_offset +
						DMA_LLP_HIGH_OFF_RDCH);
		writel(chan, abc_dev->pcie_config + DMA_READ_DOORBELL);
	} else
		return -EINVAL;

	writel(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	return 0;
}

int dma_sblk_start(uint8_t chan, enum dma_data_direction dir,
					struct dma_element_t *blk)
{
	u32 dma_offset;
	u32 val;
	u32 set_val;

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	val = readl(abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	set_val = val & ~(DBI_OVERRIDE_MASK);
	set_val |= DBI_OVERRIDE_DMA;
	writel(set_val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);

	if (dir ==  DMA_TO_DEVICE) {
		dma_offset = chan * DMA_READ_OFFSET;
		writel(DMA_ENABLE, abc_dev->pcie_config + DMA_READ_ENGINE);
		writel(DMA_MASK,
		       abc_dev->pcie_config + DMA_READ_INTERRUPT_MASK);
		writel(0x04000008, abc_dev->pcie_config + dma_offset +  DMA_READ_CHANNEL_CONTROL_1);
		writel(blk->len, abc_dev->pcie_config + dma_offset + DMA_READ_TRANSFER_SIZE);
		writel(blk->src_addr, abc_dev->pcie_config + dma_offset + DMA_READ_SAR_LOW);
		writel(blk->src_u_addr, abc_dev->pcie_config + dma_offset + DMA_READ_SAR_HIGH);
		writel(blk->dst_addr, abc_dev->pcie_config + dma_offset + DMA_READ_DAR_LOW);
		writel(blk->dst_u_addr, abc_dev->pcie_config + dma_offset + DMA_READ_DAR_HIGH);
		writel(chan, abc_dev->pcie_config + DMA_READ_DOORBELL);
	} else {
		dma_offset = chan * DMA_WRITE_OFFSET;
		writel(DMA_ENABLE, abc_dev->pcie_config + DMA_WRITE_ENGINE);
		writel(DMA_MASK, abc_dev->pcie_config + DMA_WRITE_INTERRUPT_MASK);
		writel(0x04000008, abc_dev->pcie_config + dma_offset +  DMA_WRITE_CHANNEL_CONTROL_1);
		writel(blk->len, abc_dev->pcie_config + dma_offset + DMA_WRITE_TRANSFER_SIZE);
		writel(blk->src_addr, abc_dev->pcie_config + dma_offset + DMA_WRITE_SAR_LOW);
		writel(blk->src_u_addr, abc_dev->pcie_config + dma_offset + DMA_WRITE_SAR_HIGH);
		writel(blk->dst_addr, abc_dev->pcie_config + dma_offset + DMA_WRITE_DAR_LOW);
		writel(blk->dst_u_addr, abc_dev->pcie_config + dma_offset + DMA_WRITE_DAR_HIGH);
		writel(chan, abc_dev->pcie_config + DMA_WRITE_DOORBELL);
	}

	writel(val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	return 0;
}

int abc_reg_irq_callback(irq_cb_t sys_cb, int irq_no)
{
	unsigned long flags;

	if ((irq_no >= ABC_MSI_0_TMU_AON && irq_no <= ABC_MSI_14_FLUSH_DONE)
			|| (irq_no == ABC_MSI_AON_INTNC)) {
		spin_lock_irqsave(&abc_dev->lock, flags);
		abc_dev->sys_cb[irq_no] = sys_cb;
		spin_unlock_irqrestore(&abc_dev->lock, flags);
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(abc_reg_irq_callback);

int abc_reg_irq_callback2(irq_cb_t2 sys_cb, int irq_no, void *payload)
{
	unsigned long flags;

	if ((irq_no >= ABC_MSI_0_TMU_AON && irq_no <= ABC_MSI_14_FLUSH_DONE) ||
		(irq_no >= ABC_MSI_AON_INTNC && irq_no <= INTNC_PPMU_FSYS_S)) {
		spin_lock_irqsave(&abc_dev->lock, flags);
		abc_dev->sys_cb2[irq_no] = sys_cb;
		abc_dev->handler_payload[irq_no] = payload;
		spin_unlock_irqrestore(&abc_dev->lock, flags);
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(abc_reg_irq_callback2);

int abc_reg_dma_irq_callback(irq_dma_cb_t dma_cb, int dma_chan)
{
	if (dma_chan >= 0 && dma_chan <= 7)
		abc_dev->dma_cb[dma_chan] = dma_cb;
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

	/* Set SYSREG_FSYS DBI_OVERRIDE for DMA access mode */
	override_val = readl(abc_dev->fsys_config +
					SYSREG_FSYS_DBI_OVERRIDE);
	override_set_val = override_val & ~(DBI_OVERRIDE_MASK);
	override_set_val = DBI_OVERRIDE_DMA;
	writel(override_set_val, abc_dev->fsys_config +
					SYSREG_FSYS_DBI_OVERRIDE);

	/* DMA Read Callback Implementation */
	dma_rd_stat = readl(abc_dev->pcie_config + DMA_READ_INT_STATUS_OFF);
	pos = 0;
	pr_info("---dma_rd_stat = 0x%x--\n", dma_rd_stat);
	while ((pos = find_next_bit((unsigned long *) &dma_rd_stat, 32,
					pos)) != 32) {
		writel((0x1 << pos),
				abc_dev->pcie_config + DMA_READ_INT_CLEAR_OFF);

		if (pos >= 0 && pos <= 7) {
			dma_chan = pos;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan, DMA_TO_DEVICE,
						DMA_DONE);
		} else if (pos >= 16 && pos <= 23) {
			dma_chan = pos - 16;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan, DMA_TO_DEVICE,
						DMA_ABORT);
		}
		pos++;
	}

	/* DMA Write Callback Implementation */
	dma_wr_stat = readl(abc_dev->pcie_config + DMA_WRITE_INT_STATUS_OFF);
	pos = 0;
	pr_info("---dma_wr_stat = 0x%x--\n", dma_wr_stat);
	while ((pos = find_next_bit((unsigned long *) &dma_wr_stat, 32,
					pos)) != 32) {
		writel((0x1 << pos),
				abc_dev->pcie_config + DMA_WRITE_INT_CLEAR_OFF);

		if (pos >= 0 && pos <= 7) {
			dma_chan = pos;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan, DMA_FROM_DEVICE,
						DMA_DONE);
		} else if (pos >= 16 && pos <= 23) {
			dma_chan = pos - 16;
			if (abc_dev->dma_cb[dma_chan] != NULL)
				abc_dev->dma_cb[dma_chan](dma_chan, DMA_FROM_DEVICE,
						DMA_ABORT);
		}
		pos++;
	}

	writel(override_val, abc_dev->fsys_config + SYSREG_FSYS_DBI_OVERRIDE);
	return IRQ_HANDLED;
}

static irqreturn_t abc_pcie_irq_handler(int irq, void *ptr)
{
	struct pci_dev *pdev = ptr;
	u32 msi_status_31;
	int pos, cb_pos;
	u32 msi_cap_val;

	irq = irq - pdev->irq;
	pr_info_ratelimited("MSI Irq number is : %d\n", irq);
	spin_lock(&abc_dev->lock);

	if (irq >= ABC_MSI_0_TMU_AON && irq <= ABC_MSI_14_FLUSH_DONE) {
		if (abc_dev->sys_cb[irq] != NULL) {
			/* Callback respective handler */
			abc_dev->sys_cb[irq](irq);
		} else if (abc_dev->sys_cb2[irq] != NULL &&
				abc_dev->handler_payload[irq] != NULL) {
			abc_dev->sys_cb2[irq](irq,
					abc_dev->handler_payload[irq]);
		}
	} else if (irq == ABC_MSI_AON_INTNC) {
		/* Mask 31st MSI during Interrupt handling period */
		msi_cap_val = readl(abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);
		writel((msi_cap_val | MSI_CAP_MASK_31),
				abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);

		/* Check the sysreg status register and do the callback */
		msi_status_31 = readl(abc_dev->fsys_config +
				SYSREG_FSYS_INTERRUPT);
		cb_pos = MAX_ABC_MSI;
		pos = 0;
		while ((pos = find_next_bit((unsigned long *) &msi_status_31,
						32, pos)) != 32) {
			cb_pos += pos;
			if (abc_dev->sys_cb2[cb_pos] &&
				abc_dev->handler_payload[cb_pos])
				abc_dev->sys_cb2[cb_pos](cb_pos,
					abc_dev->handler_payload[cb_pos]);
			else
				pr_err("Unregistered interrupt %d on \
					MSI 31st line has been triggered \n", pos);
			pos++;
	}
		writel(msi_cap_val, abc_dev->pcie_config + MSI_CAP_OFF_10H_REG);

	}
	spin_unlock(&abc_dev->lock);
	return IRQ_HANDLED;
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
		/* ABC_MSI_2_IPU_IRQ0 and ABC_MSI_3_IPU_IRQ1 are registered by
		 * the paintbox IPU driver
		 */
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1)
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
		/* ABC_MSI_3_IPU_IRQ1 is freed by paintbox driver*/
		if (i == ABC_MSI_3_IPU_IRQ1)
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
		if (i == ABC_MSI_2_IPU_IRQ0 || i == ABC_MSI_3_IPU_IRQ1)
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
static const struct resource tpu_resources[] = {
	{
		.name = DRV_NAME_ABC_PCIE_TPU,
		.start = 0,			/* TODO: determine */
		.end = (2 * 1024 * 1024) - 1,	/* TODO: determine */
		.flags = IORESOURCE_MEM,
	}
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

static struct mfd_cell abc_pcie_bar0[] = {
	DEV(DRV_NAME_ABC_PCIE_TPU, tpu_resources),
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

	err = mfd_add_devices(&pdev->dev, -1, abc_pcie_bar0,
			ARRAY_SIZE(abc_pcie_bar0), &pdev->resource[0],
			pdev->irq, NULL);
	if (err < 0) {
		dev_err(&pdev->dev, "mfd_add_devices[0] failed: %d\n", err);
		return err;
	}

#ifdef CONFIG_MULTIPLE_BAR_MAP_FOR_ABC_SFR
	err = mfd_add_devices(&pdev->dev, -1, abc_pcie_bar2,
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
	volatile int bar;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct abc_pcie_devdata *abc;
	struct resource *res;
	struct pci_bus_region region;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	abc = kzalloc(sizeof(struct abc_pcie_devdata), GFP_KERNEL);
	if (abc == NULL) {
		dev_err(&pdev->dev, "no memory for device data\n");
		return -ENOMEM;
	}

	abc_dev = kzalloc(sizeof(struct abc_device), GFP_KERNEL);
	if (abc_dev == NULL) {
		dev_err(&pdev->dev, "no memory for abc device data\n");
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
			abc_dev->sfr_misc_config = base;
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
		if (bar == 2)
			abc_dev->bar2_base = base;
		if (bar == 4)
			abc_dev->bar4_base = base;
#endif
		pcibios_resource_to_bus(pdev->bus, &region, res);
		abc_dev->bar_base[bar].start = region.start;
		abc_dev->bar_base[bar].end   = region.end;
		bar+=2;
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

	pci_set_drvdata(pdev, abc);

#if IS_ENABLED(CONFIG_ARM64_DMA_USE_IOMMU)
	setup_smmu(pdev);
#endif

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

	for (bar = BAR_0; bar <= BAR_4; bar+=2) {
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

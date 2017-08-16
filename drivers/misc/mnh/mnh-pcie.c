/*
 *
 * MNH PCIe/DMA HOST Driver
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
/* #define DEBUG */
#include <asm/dma-iommu.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/rwsem.h>
#include <linux/scatterlist.h>
#include <linux/pagemap.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/dma-buf.h>

#include "mnh-pcie.h"
#include "hw-mnh-regs.h"

#define COMBINE_SG 1

#define DEV_MINOR			0
#define DEVICENO			1
#define MODULE_NAME			"mnh_pci"
#define PCI_DEVICE_ID_MNH		0x3140

#define BAR_0 0
#define BAR_2 2
#define BAR_4 4
#define BAR_MAX_NUM 6

#define SUCCESS 0
#define INIT_DONE 0x1

#define SET_BIT(val, bitIndex)	  (val |= (1 << bitIndex))
#define CLEAR_BIT(val, bitIndex)  (val &= ~(1 << bitIndex))
#define BIT_IS_SET(val, bitIndex) (val & (1 << bitIndex))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))


#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))

#define IS_NULL(ptr) ((ptr == NULL)?1:0)

/* mnh_pci_tbl - PCI Device ID Table */
static const struct pci_device_id mnh_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_MNH)},
	{0, } /* terminate the list */
};
MODULE_DEVICE_TABLE(pci, mnh_pci_tbl);

struct mnh_device {
	struct pci_dev	*pdev;
	void __iomem	*pcie_config;
	void __iomem	*config;
	void __iomem	*ddr;
	irq_cb_t	msg_cb;
	irq_cb_t	vm_cb;
	irq_dma_cb_t	dma_cb;
	uint32_t	ring_buf_addr;
	uint32_t	msi_num;
	int		irq;
	uint8_t		bar2_iatu_region; /* 0  - whole BAR2
					1 - ROM/SRAM
					2 - Peripheral Config */
	uint32_t	bar_size[BAR_MAX_NUM];
	bool powered;
};

static struct mnh_device *mnh_dev;

static void mnh_check_pci_resources(struct pci_dev *dev, int bar);
static uint32_t mnh_check_iatu_bar2(uint32_t offset);

void *mnh_alloc_coherent(size_t size, dma_addr_t *dma_addr)
{
	return dma_alloc_coherent(
		&mnh_dev->pdev->dev, size, dma_addr, GFP_KERNEL);
}
EXPORT_SYMBOL(mnh_alloc_coherent);

void mnh_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr)
{
	dma_free_coherent(
		&mnh_dev->pdev->dev, size, cpu_addr, dma_addr);
}
EXPORT_SYMBOL(mnh_free_coherent);

/**
 * Map host memory for access by MNH PCIe host
 * @param[in] cpu_addr cpu virtual address of the memory region
 * @param[in] size size of the memory region in bytes
 * @param[in] direction DMA direction DMA_TO_DEVICE, etc.
 * @return DMA address returned by dma_map_single(), or zero for error
 */
dma_addr_t mnh_map_mem(
	void *cpu_addr, size_t size, enum dma_data_direction direction)
{
	dma_addr_t dma_addr =
		dma_map_single(&mnh_dev->pdev->dev, cpu_addr, size, direction);
	if (dma_mapping_error(&mnh_dev->pdev->dev, dma_addr))
		return 0;
	return dma_addr;
}
EXPORT_SYMBOL(mnh_map_mem);

/**
 * Unmap host memory from MNH PCIe host access
 * @param[in] dma_addr DMA address of the memory returned by mnh_map_mem()
 * @param[in] size size of the memory region in bytes
 * @param[in] direction DMA direction DMA_TO_DEVICE, etc.
 */
void mnh_unmap_mem(
	dma_addr_t dma_addr, size_t size, enum dma_data_direction direction)
{
	dma_unmap_single(&mnh_dev->pdev->dev, dma_addr, size, direction);
}
EXPORT_SYMBOL(mnh_unmap_mem);

/**
 * API to read data from PCIE configuration space
 * @param[in] offset  offset into PCIE configuration space(BAR0)
 * @param[in] len     buffer size : supported size is 4
 * @param[in] data    data to be read into. Client must allocate
 *                    the buffer for reading and pass the pointer
 * @return 0 if success or -EINVAL or -EFATAL on failure
 * an error happens during the process.(mnh_dma_chan_status_t)
 */
int mnh_pcie_config_read(uint32_t offset,  uint32_t len, uint32_t *data)
{

	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->pdev->current_state != PCI_D0)
		return -EIO;

	if (len != sizeof(uint32_t))
		return -EINVAL; /* only 32bit access is supported */

	if (offset > mnh_dev->bar_size[BAR_0] - sizeof(uint32_t))
		return -EINVAL; /* address invalid */

	if (WARN_ON(!mnh_dev->powered)) {
		dev_err(&mnh_dev->pdev->dev,
			"%s: cannot do pcie transfer while powering down\n",
			__func__);
		return -EIO;
	}

	pci_read_config_dword(mnh_dev->pdev, offset, data);

	dev_dbg(&mnh_dev->pdev->dev, "Read PCIE Config [0x%08x]-0x%x\n",
		offset, *data);

	if (*data != 0xffffffff)
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL(mnh_pcie_config_read);

/**
 * API to write data to PCIE configuration space
 * @param[in] offset offset into PCIE configuration space(BAR0)
 * @param[in] len	 buffer size : supported size is 4
 * @param[in] data	 data to be written
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_pcie_config_write(uint32_t offset, uint32_t len, uint32_t data)
{
	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->pdev->current_state != PCI_D0)
		return -EIO;

	if (len != sizeof(uint32_t))
		return -EINVAL; /* only 32bit access is supported */

	if (offset > mnh_dev->bar_size[BAR_0] - sizeof(uint32_t))
		return -EINVAL; /* address invalid */

	dev_dbg(&mnh_dev->pdev->dev, "Write PCIE Config[0x%08x]-0x%x",
		offset, data);

	if (WARN_ON(!mnh_dev->powered)) {
		dev_err(&mnh_dev->pdev->dev,
			"%s: cannot do pcie transfer while powering down\n",
			__func__);
		return -EIO;
	}

	pci_write_config_dword(mnh_dev->pdev, offset, data);

	return 0;

}
EXPORT_SYMBOL(mnh_pcie_config_write);

/**
 * API to read data from MNH configuration space
 * @param[in] offset  offset into MNH Address space(BAR2)
 * @param[in] len     buffer size : supported size is 1,2,4
 * @param[in] data    data to be read into. Client must allocate
 *                    the buffer for reading and pass the pointer
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_config_read(uint32_t offset,  uint32_t len, uint32_t *data)
{
	uint32_t new_offset;

	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->pdev->current_state != PCI_D0)
		return -EIO;

	if (offset > HW_MNH_PCIE_BAR_2_ADDR_END - len) {
		dev_err(&mnh_dev->pdev->dev, "Addr Invalid: %x", offset);
		return -EINVAL; /* address invalid */
	}

	new_offset = mnh_check_iatu_bar2(offset);

	if (WARN_ON(!mnh_dev->powered)) {
		dev_err(&mnh_dev->pdev->dev,
			"%s: cannot do pcie transfer while powering down\n",
			__func__);
		return -EIO;
	}

	if (len == sizeof(uint32_t))
		*data = ioread32(mnh_dev->config + new_offset);
	else if (len == sizeof(uint16_t))
		*data = ioread16(mnh_dev->config + new_offset);
	else if (len == sizeof(uint8_t))
		*data = ioread8(mnh_dev->config + new_offset);
	else {
		dev_err(&mnh_dev->pdev->dev, "%s: invalid len %d\n",
			__func__, len);
		return -EINVAL;
	}

	dev_dbg(&mnh_dev->pdev->dev, "Read Config[0x%08x] - 0x%x",
		new_offset, *data);

	if (*data != 0xffffffff)
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL(mnh_config_read);

/**
 * API to write data to MNH configuration space
 * @param[in] offset  offset into MNH Address space(BAR2)
 * @param[in] len	buffer size : supported size could be 4
 * @param[in] data	 pointer to the data to be wrriten.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_config_write(uint32_t offset, uint32_t len, uint32_t data)
{
	uint32_t new_offset;

	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->pdev->current_state != PCI_D0)
		return -EIO;

	if (offset > HW_MNH_PCIE_BAR_2_ADDR_END - len)
		return -EINVAL; /* address invalid */

	new_offset = mnh_check_iatu_bar2(offset);

	dev_dbg(&mnh_dev->pdev->dev, "Write Config[0x%08x] - 0x%x",
		new_offset, data);

	if (WARN_ON(!mnh_dev->powered)) {
		dev_err(&mnh_dev->pdev->dev,
			"%s: cannot do pcie transfer while powering down\n",
			__func__);
		return -EIO;
	}

	if (len == sizeof(uint32_t))
		iowrite32(data, mnh_dev->config + new_offset);
	else if (len == sizeof(uint16_t))
		iowrite16(data, mnh_dev->config + new_offset);
	else if (len == sizeof(uint8_t))
		iowrite8(data, mnh_dev->config + new_offset);
	else {
		dev_err(&mnh_dev->pdev->dev, "%s: invalid len %d\n",
			__func__, len);
		return -EINVAL;
	}

	return 0;

}
EXPORT_SYMBOL(mnh_config_write);

/**
 * API to read data from MNH DDR space
 * @param[in] offset  offset into MNH DRAM space(BAR4)
 * @param[in] len     buffer size
 * @param[in] data    data to be read into. Client must allocate
 *                    the buffer for reading and pass the pointer
 * @return 0 if success or -EINVAL or -EFATAL on failure
 * @note Usually DMA will be used to read/write from DDR space in MNH
 *       This maybe only used for debugging purpose
 */
int mnh_ddr_read(uint32_t offset,  uint32_t len, void *data)
{
	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->pdev->current_state != PCI_D0)
		return -EIO;

	if (len > mnh_dev->bar_size[BAR_4])
		return -EINVAL;

	if (offset > mnh_dev->bar_size[BAR_4] - len)
		return -EINVAL; /* address invalid */

	dev_dbg(&mnh_dev->pdev->dev, "Read DDR[0x%08x], len-%d, data-0x%0x",
		offset, len, *(uint32_t *)data);

	if (WARN_ON(!mnh_dev->powered)) {
		dev_err(&mnh_dev->pdev->dev,
			"%s: cannot do pcie transfer while powering down\n",
			__func__);
		return -EIO;
	}

	memcpy(data, mnh_dev->ddr + offset, len);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_read);

/**
 * API to write data to MNH DDR space
 * @param[in] offset  offset into MNH DRAM space(BAR4)
 * @param[in] len	buffer size
 * @param[in] data	 pointer to the data to be wrriten.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 * @note Usually DMA will be used to read/write from DDR space in MNH
 *		 This maybe only used for debugging purpose
 */
int mnh_ddr_write(uint32_t offset, uint32_t len, void *data)
{
	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->pdev->current_state != PCI_D0)
		return -EIO;

	if (len > mnh_dev->bar_size[BAR_4])
			return -EINVAL;

	if (offset > mnh_dev->bar_size[BAR_4] - len)
		return -EINVAL; /* address invalid */

	if (!data) {
		dev_err(&mnh_dev->pdev->dev, "No Data\n");
		return -EINVAL; /*data pointer is NULL */
	}

	dev_dbg(&mnh_dev->pdev->dev, "Write DDR[0x%08x], len-%d, data-0x%x",
			offset, len, *(uint32_t *)data);

	if (WARN_ON(!mnh_dev->powered)) {
		dev_err(&mnh_dev->pdev->dev,
			"%s: cannot do pcie transfer while powering down\n",
			__func__);
		return -EIO;
	}

	memcpy(mnh_dev->ddr + offset, data, len);

	return 0;
}
EXPORT_SYMBOL(mnh_ddr_write);

/**
 * API to generate IRQ from AP to MNH
 * @param[in] irq  IRQ ID to be sent (mnh_irq_msg_t)
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_send_irq(enum mnh_irq_msg_t irq)
{
	uint32_t mask = 0, data = 0;

	SET_BIT(mask, irq);

	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->pdev->current_state != PCI_D0)
		return -EIO;

	dev_dbg(&mnh_dev->pdev->dev, "Send IRQ to EP:%d", irq);

	/* Update PCIE_SW_INTR_EN to enable the spcecific bit*/
	mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_SW_INTR_EN,
		sizeof(uint32_t), &data);
	data |= mask;
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_SW_INTR_EN,
		sizeof(uint32_t), data);
	dev_dbg(&mnh_dev->pdev->dev, "Write sw_intr_en: 0x%x", data);

	/* trigger the interrupt write */
	mask |= HW_MNH_PCIE_SW_INTR_TRIGG_MASK_RW;
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_SW_INTR_TRIGG,
		sizeof(uint32_t), mask);
	dev_dbg(&mnh_dev->pdev->dev, "write sw_intr_trigg: 0x%x", mask);

	return 0;
}
EXPORT_SYMBOL(mnh_send_irq);


/** API to register IRQ callback to receive msg delivery from MNH
 * @param[in] msg_cb  interrupt handler for MSI received from MNH, pass NULL
 *					  to de-register
 * @param[in] vm_cb   interrupt handler for Vendor Message received from MNH,
 *					  pass NULL to de-register
 * @param[in] dma_cb  interrupt handler for DMA message(DMA_DONE/ABORT) received
 *					  from MNH, pass NULL to de-register
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_reg_irq_callback(irq_cb_t msg_cb, irq_cb_t vm_cb, irq_dma_cb_t dma_cb)
{
	if (!mnh_dev)
		return -ENODEV;

	mnh_dev->msg_cb = msg_cb;
	mnh_dev->vm_cb = vm_cb;
	mnh_dev->dma_cb = dma_cb;

	return 0;
}
EXPORT_SYMBOL(mnh_reg_irq_callback);

/**
 * API to send Vendor specific message from AP to MNH
 * @param[in] msg  Vendor message to be sent include msg_id
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_send_vendor_msg(struct mnh_pcie_vm msg)
{
	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	dev_dbg(&mnh_dev->pdev->dev, "Send Vendor Msg: NOT SUPPORTED\n");
	return 0;
}
EXPORT_SYMBOL(mnh_send_vendor_msg);


/**
 * API to get the base ring buffer address of MNH
 * @param[out] rb_base ring buffer address TODO: Need to change 64bit
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_get_rb_base(uint64_t *rb_base)
{
	uint32_t data;

	if (mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET + HW_MNH_PCIE_GP_1,
			sizeof(uint32_t), &data) != SUCCESS) {
		dev_err(&mnh_dev->pdev->dev, "Not able to read RB base\n");
		return -EINVAL;
	}
	*rb_base = data;

	dev_dbg(&mnh_dev->pdev->dev, "RB base: %llx", *rb_base);

	return 0;
}
EXPORT_SYMBOL(mnh_get_rb_base);

/**
 * API to set outbound region
 * @param outb[in] - iATU outbound region information
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_set_outbound_iatu(struct mnh_outb_region *outb)
{
	uint32_t data, upper, lower;
	int size = sizeof(uint32_t);

	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	dev_dbg(&mnh_dev->pdev->dev, "Set outbound IATU\n");

	if ((outb->region > 0xF) ||
		(outb->base_address > HW_MNH_PCIE_OUTBOUND_BASE) ||
		(outb->limit_address > HW_MNH_PCIE_OUTBOUND_END))
		return -EINVAL; /* address out of range */

	data = IATU_OUTBOUND | outb->region;
	mnh_pcie_config_write(IATU_VIEWPORT, size, data);
	upper = UPPER(outb->base_address);
	lower = LOWER(outb->base_address);
	mnh_pcie_config_write(IATU_LWR_BASE_ADDR, size, lower);
	mnh_pcie_config_write(IATU_UPPER_BASE_ADDR, size, upper);
	data = outb->limit_address;
	mnh_pcie_config_write(IATU_LIMIT_ADDR, size, data);
	upper = UPPER(outb->target_pcie_address);
	lower = LOWER(outb->target_pcie_address);
	mnh_pcie_config_write(IATU_LWR_TARGET_ADDR, size, lower);
	mnh_pcie_config_write(IATU_UPPER_TARGET_ADDR, size, upper);
	mnh_pcie_config_write(IATU_REGION_CTRL_1, size, IATU_MEM);
	mnh_pcie_config_write(IATU_REGION_CTRL_2, size, IATU_ENABLE);
	udelay(1);
	mnh_pcie_config_read(IATU_REGION_CTRL_2, size, &data);
	if (data != IATU_ENABLE)
		dev_err(&mnh_dev->pdev->dev, "Set outbound IATU Fail\n");

	return 0;
}
EXPORT_SYMBOL(mnh_set_outbound_iatu);

int mnh_set_inbound_iatu(struct mnh_inb_window *inb)
{
	uint32_t data, upper, lower;
	int size = sizeof(uint32_t);

	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	dev_dbg(&mnh_dev->pdev->dev, "Set inbound IATU\n");

	if (inb->mode == BAR_MATCH) {
		if ((inb->region > 0xF) || (inb->bar > 5))
		    return -EINVAL;
		data = IATU_INBOUND | inb->region;
		mnh_pcie_config_write(IATU_VIEWPORT, size, data);
		upper = UPPER(inb->target_mth_address);
		lower = LOWER(inb->target_mth_address);
		mnh_pcie_config_write(IATU_LWR_TARGET_ADDR, size, lower);
		mnh_pcie_config_write(IATU_UPPER_TARGET_ADDR, size, upper);
		data = IATU_MEM;
		mnh_pcie_config_write(IATU_REGION_CTRL_1, size, data);
		data = IATU_BAR_MODE | (inb->bar << 8);
		mnh_pcie_config_write(IATU_REGION_CTRL_2, size, data);
		udelay(1);
		mnh_pcie_config_read(IATU_REGION_CTRL_2, size, &data);
	} else {
		if ((inb->region > 0xF) ||
			(inb->target_mth_address >
			HW_MNH_PCIE_OUTBOUND_BASE) ||
			((inb->target_mth_address
			+ inb->limit_pcie_address)
			> HW_MNH_PCIE_OUTBOUND_BASE) ||
			(inb->memmode > 0xf))
				return -EINVAL; /* address out of range */
		data = IATU_INBOUND | inb->region;
		mnh_pcie_config_write(IATU_VIEWPORT, size, data);
		upper = UPPER(inb->base_pcie_address);
		lower = LOWER(inb->base_pcie_address);
		mnh_pcie_config_write(IATU_LWR_BASE_ADDR, size, lower);
		mnh_pcie_config_write(IATU_UPPER_BASE_ADDR, size, upper);
		data = inb->limit_pcie_address;
		mnh_pcie_config_write(IATU_LIMIT_ADDR, size, data);
		upper = UPPER(inb->target_mth_address);
		lower = LOWER(inb->target_mth_address);
		mnh_pcie_config_write(IATU_LWR_TARGET_ADDR, size, lower);
		mnh_pcie_config_write(IATU_UPPER_TARGET_ADDR, size, upper);
		mnh_pcie_config_write(IATU_REGION_CTRL_1, size, inb->memmode);
		mnh_pcie_config_write(IATU_REGION_CTRL_2, size, IATU_ENABLE);
		udelay(1);
		mnh_pcie_config_read(IATU_REGION_CTRL_2, size, &data);
		if (data != IATU_ENABLE)
			dev_err(&mnh_dev->pdev->dev, "Set inbound IATU Fail\n");
	}
	return 0;
}
EXPORT_SYMBOL(mnh_set_inbound_iatu);

/**
 * API to abort DMA transfer on specific channel
 * @param[in] chan	   The channel number for DMA transfer abort
 * @param[in] dir	   Direction of DMA channel, READ or WRITE
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_abort(uint8_t chan, enum mnh_dma_chan_dir_t dir)
{
	uint32_t data;
	uint8_t len = sizeof(data);
	struct mnh_dma_state_info_t state;

	dev_dbg(&mnh_dev->pdev->dev, "DMA ABORT: DIR:%d CH:%d\n", dir, chan);

	mnh_dma_get_status(chan, dir, &state);
	if(state.status != DMA_CHAN_RUNNING) {
		dev_dbg(&mnh_dev->pdev->dev, "CH not running! - %d\n",
		state.status);
		return -EINVAL;
	}

	/* update doorbell register */
	if (dir == DMA_EP2AP) { /* write */
		data = DMA_WRITE_DOORBELL_OFF_MASK_WR_STOP;
		data |= DMA_WRITE_DOORBELL_OFF_MASK_CH_NUM & chan;
		mnh_pcie_config_write(DMA_WRITE_DOORBELL_OFF, len, data);
	} else if (dir == DMA_AP2EP) { /* read */
		data = DMA_READ_DOORBELL_OFF_MASK_RD_STOP;
		data |= DMA_READ_DOORBELL_OFF_MASK_CH_NUM & chan;
		mnh_pcie_config_write(DMA_READ_DOORBELL_OFF, len, data);
	}

	return 0;
}
EXPORT_SYMBOL(mnh_dma_abort);


/**
 * API to resume DMA transfer on specific channel
 * @param[in] chan The channel number for DMA transfer resume
 * @param[in] dir  Direction of DMA channel, READ or WRITE
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_resume(uint8_t chan, enum mnh_dma_chan_dir_t dir)
{
	struct mnh_dma_state_info_t state;

	/* Get the chan state */
	mnh_dma_get_status(chan, dir, &state);

	if (state.status != DMA_CHAN_STOPPED) {
		dev_err(&mnh_dev->pdev->dev,
			"CH[#: %d, dir: %d]  is not STOPPED, can't resume\n",
			chan, dir);
		return -EINVAL;
	}

	/*TODO: LL list check */
	/*TODO: Get the current src and dst pos and do single block update */
	dev_dbg(&mnh_dev->pdev->dev, "DMA RESUME/ABORT NOT SUPPORTED YET\n");

	return -EINVAL;
}
EXPORT_SYMBOL(mnh_dma_resume);

/**
 * API to read/write single block on specific channel.
 * @param[in] chan  The channel number for DMA transfer to be performed
 * @param[in] dir   The channel direction (read/write)
 * @param[in] blk   One dma element info which include SAR(Source Address),
 *			        DAR(Destination Address) and transfer size.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_sblk_start(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		struct mnh_dma_element_t *blk)
{
	uint32_t data = 0;
	uint8_t size = sizeof(data);

	if (dir == DMA_EP2AP) { /* write */
		dev_dbg(&mnh_dev->pdev->dev, "DMA WRITE[EP2AP]: CH%d\n", chan);
		data = DMA_WRITE_ENGINE_EN_OFF_MASK_ENABLE;
		mnh_pcie_config_write(DMA_WRITE_ENGINE_EN_OFF, size, data);
		mnh_pcie_config_write(DMA_WRITE_INT_MASK_OFF, size, 0);
	} else if (dir == DMA_AP2EP) { /* read */
		dev_dbg(&mnh_dev->pdev->dev, "DMA READ[AP2EP]: CH%d\n", chan);
		data = DMA_READ_ENGINE_EN_OFF_MASK_ENABLE;
		mnh_pcie_config_write(DMA_READ_ENGINE_EN_OFF, size, data);
		mnh_pcie_config_write(DMA_READ_INT_MASK_OFF, size, 0);
	} else
		return -EINVAL;

	data = DMA_VIEWPORT_SEL_OFF_MASK_CHAN_NUM & chan;
	data |= dir ? DMA_VIEWPORT_SEL_OFF_MASK_CHAN_DIR:0;
	mnh_pcie_config_write(DMA_VIEWPORT_SEL_OFF, size, data);
	data = DMA_CH_CONTROL1_MASK_LIE;
	data |= DMA_CH_CONTROL1_MASK_RIE;
	data |= DMA_CH_CONTROL1_OFF_RESERVED;
	mnh_pcie_config_write(DMA_CH_CONTROL1_OFF, size, data);
	data = blk->len;
	mnh_pcie_config_write(DMA_TRANSFER_SIZE_OFF, size, data);
	data = LOWER(blk->src_addr);
	mnh_pcie_config_write(DMA_SAR_LOW_OFF, size, data);
	data = UPPER(blk->src_addr);
	mnh_pcie_config_write(DMA_SAR_HIGH_OFF, size, data);
	data = LOWER(blk->dst_addr);
	mnh_pcie_config_write(DMA_DAR_LOW_OFF, size, data);
	data = UPPER(blk->dst_addr);
	mnh_pcie_config_write(DMA_DAR_HIGH_OFF, size, data);

	if (dir == DMA_EP2AP) { /* write */
		data = DMA_WRITE_DOORBELL_OFF_MASK_CH_NUM & chan;
		mnh_pcie_config_write(DMA_WRITE_DOORBELL_OFF, size, data);
	} else if (dir == DMA_AP2EP) { /* read */
		data = DMA_READ_DOORBELL_OFF_MASK_CH_NUM & chan;
		mnh_pcie_config_write(DMA_READ_DOORBELL_OFF, size, data);
	}

	return 0;
}
EXPORT_SYMBOL(mnh_dma_sblk_start);


static enum dma_data_direction mnh_to_dma_dir(enum mnh_dma_chan_dir_t mnh_dir)
{
	return (mnh_dir == DMA_AP2EP) ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
}

/**
 * Convert Linux scatterlist to array of entries used by PCIe EP DMA engine
 * @param[in] sc_list   Scatter gather list for DMA buffer
 * @param[in] count  Number of entries in scatterlist
 * @param[out] sg  Array generated dma addresses and length.
 * @param[in] maxsg  Allocated max array number of the sg
 * @return a count of sg entries used on success
 *         -EINVAL if exceeding maxsg
 */
static int scatterlist_to_mnh_sg(struct scatterlist *sc_list, int count,
	struct mnh_sg_entry *sg, size_t maxsg)
{
	struct scatterlist *in_sg;
	int i, u;

	i = 0;	/* iterator of *sc_list */
	u = 0;	/* iterator of *sg */

	for_each_sg(sc_list, in_sg, count, i) {
		if (u < maxsg) {
			sg[u].paddr = sg_dma_address(in_sg);
			sg[u].size = sg_dma_len(in_sg);

		dev_dbg(&mnh_dev->pdev->dev,
			"sg[%d] : Address %pa , length %zu\n",
			u, &sg[u].paddr, sg[u].size);
#ifdef COMBINE_SG
			if ((u > 0) && (sg[u-1].paddr + sg[u-1].size ==
				sg[u].paddr)) {
				sg[u-1].size = sg[u-1].size
					+ sg[u].size;
				sg[u].size = 0;
			} else {
				u++;
			}
#else
			u++;
#endif
		} else {
			dev_err(&mnh_dev->pdev->dev, "maxsg exceeded\n");
			return -EINVAL;
		}
	}
	sg[u].paddr = 0x0;

	dev_dbg(&mnh_dev->pdev->dev, "SGL with %d/%d entries\n", u, i);

	return u;
}

/**
 * Import dma_buf (from ION buffer)
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries
 * @return 0        on SUCCESS
 *         negative on failure
 */
static int mnh_sg_import_dma_buf(int fd, struct mnh_sg_list *sgl)
{
	int ret;

	sgl->dma_buf = dma_buf_get(fd);
	if (IS_ERR(sgl->dma_buf)) {
		ret = PTR_ERR(sgl->dma_buf);
		dev_err(&mnh_dev->pdev->dev,
				"%s: failed to get dma_buf, err %d\n",
				__func__, ret);
		return ret;
	}

	sgl->attach = dma_buf_attach(sgl->dma_buf, &mnh_dev->pdev->dev);
	if (IS_ERR(sgl->attach)) {
		ret = PTR_ERR(sgl->attach);
		dev_err(&mnh_dev->pdev->dev,
				"%s: failed to attach dma_buf, err %d\n",
				__func__, ret);
		goto err_put;
	}

	sgl->sg_table = dma_buf_map_attachment(sgl->attach,
						mnh_to_dma_dir(sgl->dir));
	if (IS_ERR(sgl->sg_table)) {
		ret = PTR_ERR(sgl->sg_table);
		dev_err(&mnh_dev->pdev->dev,
				"%s: failed to map dma_buf, err %d\n",
				__func__, ret);
		goto err_detach;
	}

	return 0;

err_detach:
	dma_buf_detach(sgl->dma_buf, sgl->attach);
err_put:
	dma_buf_put(sgl->dma_buf);
	return ret;
}

/**
 * API to build a scatter-gather list for multi-block DMA transfer for a
 * dma_buf
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[out] sg  Array of maxsg pointers to struct mnh_sg_entry, allocated
 *			and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries.
 * @return 0        on SUCCESS
 *         negative on failure
 */
int mnh_sg_retrieve_from_dma_buf(int fd, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl)
{
	int ret;
	size_t maxsg;

	/* Retrieve sg_table from dma_buf framework */
	ret = mnh_sg_import_dma_buf(fd, sgl);
	if (ret)
		return ret;

	/* Use sg_table->sgl as our sc_list */
	sgl->sc_list = sgl->sg_table->sgl;
	sgl->n_num = sgl->sg_table->nents;

	/*
	 * The driver assumes either ION userspace code (e.g. Camera HAL)
	 * or dma_buf provider has handled cache correctly.
	 */
	/* dma_sync_sg_for_device(&mnh_dev->pdev->dev, sgl->sc_list,
				sgl->n_num, mnh_to_dma_dir(sgl->dir)); */

	/*
	 * Allocate enough for one entry per sc_list entry, plus end of list.
	 */
	maxsg = sgl->n_num + 1;
	*sg = vmalloc(maxsg * sizeof(struct mnh_sg_entry));
	if (!(*sg)) {
		mnh_sg_release_from_dma_buf(sgl);
		return -ENOMEM;
	}

	dev_dbg(&mnh_dev->pdev->dev,
		"Enter %s: n_num:%d maxsg:%zu\n", __func__, sgl->n_num, maxsg);

	/* Convert sc_list to a Synopsys compatible linked-list */
	sgl->length = scatterlist_to_mnh_sg(sgl->sc_list, sgl->n_num,
								*sg, maxsg);
	if (IS_ERR(&sgl->length)) {
		vfree((*sg));
		*sg = NULL;
		ret = PTR_ERR(&sgl->length);
		mnh_sg_release_from_dma_buf(sgl);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mnh_sg_retrieve_from_dma_buf);

/**
 * API to release a scatter-gather list for a dma_buf
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		mnh_sg_retrieve_from_dma_buf
 * @return 0 for SUCCESS
 */
int mnh_sg_release_from_dma_buf(struct mnh_sg_list *sgl)
{
	/* dma_sync_sg_for_cpu(&mnh_dev->pdev->dev, sgl->sc_list,
				sgl->n_num, mnh_to_dma_dir(sgl->dir)); */
	dma_buf_unmap_attachment(sgl->attach, sgl->sg_table,
						mnh_to_dma_dir(sgl->dir));
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	sgl->length = 0;
	dma_buf_detach(sgl->dma_buf, sgl->attach);
	dma_buf_put(sgl->dma_buf);
	return 0;
}
EXPORT_SYMBOL(mnh_sg_release_from_dma_buf);

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * buffer
 * @param[in] dmadest  Starting virtual addr of the DMA destination
 * @param[in] size Totalsize of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct mnh_sg_entry, allocated
 *			and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			page list, scatter gather list and num of its entries.
 * @return The number of sg[] entries filled out by the routine, negative if
 *		   overflow or sg[] not allocated.
 */
int mnh_sg_build(void *dmadest, size_t size, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl)
{
	int i, u, fp_offset, count;
	int n_num, p_num;
	int first_page, last_page;
	size_t maxsg;

	/*
	 * Allocate enough for one entry per page, perhaps needing 1 more due
	 * to crossing a page boundary, plus end of list.
	 */
	maxsg = (size / PAGE_SIZE) + 3;
	*sg = vmalloc(maxsg * sizeof(struct mnh_sg_entry));
	if (!(*sg))
		return -ENOMEM;

	/* page num calculation */
	first_page = ((unsigned long) dmadest & PAGE_MASK) >> PAGE_SHIFT;
	last_page = (((unsigned long) dmadest + size - 1) & PAGE_MASK) >> PAGE_SHIFT;
	fp_offset = (unsigned long) dmadest & ~PAGE_MASK;
	p_num = last_page - first_page + 1;

	dev_dbg(&mnh_dev->pdev->dev,
		"Enter mnh_sg_build:p_num:%d, maxsg:%zu\n",
		p_num, maxsg);

	sgl->mypage = kcalloc(p_num, sizeof(struct page *), GFP_KERNEL);
	if (!sgl->mypage) {
		kfree((*sg));
		*sg = NULL;
		sgl->n_num = 0;
		sgl->length = 0;
		return -EINVAL;
	}
	sgl->sc_list = kcalloc(p_num, sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list) {
		kfree((*sg));
		*sg = NULL;
		kfree(sgl->mypage);
		sgl->mypage = NULL;
		sgl->n_num = 0;
		sgl->length = 0;
		return -EINVAL;
	}

	down_read(&current->mm->mmap_sem);
	n_num = get_user_pages((unsigned long) dmadest, p_num,
			       FOLL_WRITE | FOLL_FORCE, sgl->mypage, NULL);
	up_read(&current->mm->mmap_sem);
	if (n_num < 0) {
		dev_err(&mnh_dev->pdev->dev, "fail to get user_pages\n");
		goto free_mem;
	}
	if (n_num < maxsg) {
		sg_init_table(sgl->sc_list, n_num);
		if (n_num == 1) {
			sg_set_page(sgl->sc_list, *(sgl->mypage),
				size, fp_offset);
		} else {
			sg_set_page(sgl->sc_list, *(sgl->mypage),
				PAGE_SIZE - fp_offset, fp_offset);
			for (i = 1; i < n_num-1; i++) {
				sg_set_page(sgl->sc_list + i, *(sgl->mypage + i),
					PAGE_SIZE, 0);
			}
			sg_set_page(sgl->sc_list + n_num-1, *(sgl->mypage + n_num-1),
				size - (PAGE_SIZE - fp_offset)
				- ((n_num-2)*PAGE_SIZE), 0);
		}

		count = dma_map_sg(&mnh_dev->pdev->dev, sgl->sc_list,
				n_num, mnh_to_dma_dir(sgl->dir));

		u = scatterlist_to_mnh_sg(sgl->sc_list, count, *sg, maxsg);
		if (u < 0)
			goto unmap_sg;
	} else {
		dev_err(&mnh_dev->pdev->dev, "maxsg exceeded\n");
		goto release_page;
	}
	sgl->n_num = n_num;
	sgl->length = u;

	return 0;

unmap_sg:
	dma_unmap_sg(&mnh_dev->pdev->dev,
		sgl->sc_list, n_num, mnh_to_dma_dir(sgl->dir));
release_page:
	for (i = 0; i < sgl->n_num; i++)
		put_page(*(sgl->mypage + i));
free_mem:
	kfree((*sg));
	*sg = NULL;
	kfree(sgl->mypage);
	sgl->mypage = NULL;
	kfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	sgl->length = 0;

	return -EINVAL;
}
EXPORT_SYMBOL(mnh_sg_build);

/**
 * API to release scatter gather list for a user buffer
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		mnh_sg_build
 * @return 0 for SUCCESS
 */
int mnh_sg_destroy(struct mnh_sg_list *sgl)
{
	int i;
	struct page *page;

	dma_unmap_sg(&mnh_dev->pdev->dev, sgl->sc_list,
			sgl->n_num, mnh_to_dma_dir(sgl->dir));
	for (i = 0; i < sgl->n_num; i++) {
		page = *(sgl->mypage + i);
		/* Mark page as dirty before releasing the pages. */
		if (!PageReserved(page))
			SetPageDirty(page);
		put_page(page);
	}
	kfree(sgl->mypage);
	sgl->mypage = NULL;
	kfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;

	return 0;
}
EXPORT_SYMBOL(mnh_sg_destroy);

/**
 * API to read/write multi blocks on specific channel.
 * Multi block read/write are based on Linked List(Transfer List) built by MNH.
 * @param[in] chan  The channel number for DMA transfer to be performed
 * @param[in] dir   The channel direction (read/write)
 * @param[in] start_addr  Physical start_addr(in MNH) where transfer list is
 *						  stored.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_mblk_start(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		phys_addr_t *start_addr)
{
	uint32_t data = 0;
	uint8_t size = sizeof(data);
	uint64_t addr = *start_addr;

	if (dir == DMA_EP2AP) { /* write */
		dev_dbg(&mnh_dev->pdev->dev, "DMA WRITE M[EP2AP]:CH%d\n", chan);
		data = DMA_WRITE_ENGINE_EN_OFF_MASK_ENABLE;
		mnh_pcie_config_write(DMA_WRITE_ENGINE_EN_OFF, size, data);
		mnh_pcie_config_write(DMA_WRITE_INT_MASK_OFF, size, 0);
	} else if (dir == DMA_AP2EP) { /* read */
		dev_dbg(&mnh_dev->pdev->dev, "DMA READ M[AP2EP]:CH%d\n", chan);
		data = DMA_READ_ENGINE_EN_OFF_MASK_ENABLE;
		mnh_pcie_config_write(DMA_READ_ENGINE_EN_OFF, size, data);
		mnh_pcie_config_write(DMA_READ_INT_MASK_OFF, size, 0);
	}

	data = DMA_VIEWPORT_SEL_OFF_MASK_CHAN_NUM & chan;
	data |= dir ? DMA_VIEWPORT_SEL_OFF_MASK_CHAN_DIR:0;
	mnh_pcie_config_write(DMA_VIEWPORT_SEL_OFF, size, data);
	data = DMA_CH_CONTROL1_MASK_LLE;
	data |= DMA_CH_CONTROL1_MASK_CCS;
	data |= DMA_CH_CONTROL1_OFF_RESERVED;
	mnh_pcie_config_write(DMA_CH_CONTROL1_OFF, size, data);
	data = LOWER(addr);
	mnh_pcie_config_write(DMA_LLP_LOW_OFF, size, data);
	data = UPPER(addr);
	mnh_pcie_config_write(DMA_LLP_HIGH_OFF, size, data);

	mnh_pcie_config_write(DMA_READ_LINKED_LIST_ERR_EN_OFF, size, 0xF00F);
	mnh_pcie_config_write(DMA_WRITE_LINKED_LIST_ERR_EN_OFF, size, 0xF00F);

	if (dir == DMA_EP2AP) { /* write */
		data = DMA_WRITE_DOORBELL_OFF_MASK_CH_NUM & chan;
		mnh_pcie_config_write(DMA_WRITE_DOORBELL_OFF, size, data);
	} else if (dir == DMA_AP2EP) { /* read */
		data = DMA_READ_DOORBELL_OFF_MASK_CH_NUM & chan;
		mnh_pcie_config_write(DMA_READ_DOORBELL_OFF, size, data);
	}

	return 0;
}
EXPORT_SYMBOL(mnh_dma_mblk_start);

/**
 * API to get the current status of chan
 * Multi block DMA write is to transfer data from MNH to AP. Multi block write
 * are based on Linked List descriptor. Caller specify the source and
 * destination address of each element through linked list(blk_llist).
 * @param[in] chan The write channel on which the transfer is performed.
 * @param[in] dir The direction of channel specified on chan.
 * @param[out] info chan information includes channel status, transferred
 *				    size, error status, and etc.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */

int mnh_dma_get_status(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		struct mnh_dma_state_info_t *info)
{
	uint32_t data;
	uint32_t data_hi;
	uint32_t err = (0x1 << chan);

	/* Select CH */
	data = dir ? DMA_VIEWPORT_SEL_OFF_MASK_CHAN_DIR:0;
	data |= DMA_VIEWPORT_SEL_OFF_MASK_CHAN_NUM & chan;
	mnh_pcie_config_write(DMA_VIEWPORT_SEL_OFF, sizeof(data), data);

	/* Get Status */
	mnh_pcie_config_read(DMA_CH_CONTROL1_OFF, sizeof(data), &data);
	info->status = (data & DMA_CH_CONTROL1_MASK_CS) >> 5;

	mnh_pcie_config_read(DMA_TRANSFER_SIZE_OFF, sizeof(data), &data);
	info->xferred = data;

	info->err = 0;
	if (dir == DMA_EP2AP) { /* write */
		mnh_pcie_config_read(DMA_WRITE_ERR_STATUS_OFF, sizeof(data), &data);
		if (err & (data & DMA_WRITE_ERR_STATUS_OFF_MASK_APP_READ_ERR))
			info->err = DMA_ERR_WR;
		else if (err & (data & DMA_WRITE_ERR_STATUS_OFF_MASK_LL_FETCH_ERR))
			info->err = DMA_ERR_FETCH_LL;
	} else if (dir == DMA_AP2EP) { /*read */
		mnh_pcie_config_read(DMA_READ_ERR_STATUS_OFF, sizeof(data),
			&data);
		mnh_pcie_config_read(DMA_READ_ERR_STATUS_HIGH_OFF,
			sizeof(data), &data_hi);
		if (err & (data & DMA_READ_ERR_STATUS_OFF_MASK_APP_READ_ERR))
			info->err = DMA_ERR_RD;
		else if (err & (data & DMA_READ_ERR_STATUS_OFF_MASK_LL_FETCH_ERR))
			info->err = DMA_ERR_FETCH_LL;
		else if (err & (data & DMA_READ_ERR_STATUS_HIGH_OFF_UNSUPPORTED))
			info->err = DMA_ERR_UNSUPPORTED_RQST;
		else if (err & (data & DMA_READ_ERR_STATUS_HIGH_OFF_CPL_ABORT))
			info->err = DMA_ERR_COMPLETER_ABORT;
		else if (err & (data & DMA_READ_ERR_STATUS_HIGH_OFF_CPL_TIMEOUT))
			info->err = DMA_ERR_CPL_TIME_OUT;
		else if (err & (data & DMA_READ_ERR_STATUS_HIGH_OFF_MASK_DATA_POISON))
			info->err = DMA_ERR_DATA_POISONING;
	}

	dev_dbg(&mnh_dev->pdev->dev,
		"DMA Status[ch:%d, dir:%d] - , status:%d, xferred:%lld, err:%d\n",
		chan, dir, info->status, info->xferred, info->err);

	return 0;
}
EXPORT_SYMBOL(mnh_dma_get_status);


/************************************************
	 LOCAL APIS
*************************************************/


/**
 * mnh_check_pci_resources - MNH PCIE interrupt handler
 * @irq: Linux interrupt number
 * @ptr: Pointer to interrupt-specific data
 *
 * Handles MNH interrupts signaled using MSI
 */

static void mnh_check_pci_resources(struct pci_dev *dev, int bar)
{
	resource_size_t start, len, end;
	unsigned long flags;

	dev_dbg(&dev->dev, "PCI RESOURCES CHECK:%d\n", bar);

	start = pci_resource_start(dev, bar);
	len  = pci_resource_len(dev, bar);
	end = pci_resource_end(dev, bar);
	flags = pci_resource_flags(dev, bar);

	mnh_dev->bar_size[bar] = len;

	if (bar == BAR_2) {
		if (len == 0x8000000) /* 128M */
			mnh_dev->bar2_iatu_region = 0;
		else if (len == 0x800000 || len == 0x400000) /* 8M or 4M */
			mnh_dev->bar2_iatu_region = 1;
		else {
			dev_err(&dev->dev, "Not supported BAR2 size!!\n");
			mnh_dev->bar2_iatu_region = 0;
		}
		dev_dbg(&dev->dev, "bar2_iatu_region :%d",
			mnh_dev->bar2_iatu_region);
	}

	dev_dbg(&dev->dev,
		"PCI:BAR[%d]:start[%llx],end[%llx],len[%llx],flag[%lx]",
		bar, start, end, len, flags);

	if ((flags & IORESOURCE_MEM) || (flags & IORESOURCE_MEM_64))
		dev_dbg(&dev->dev, " flags : %s\n", "MEMORY");
	else if (flags & IORESOURCE_IO)
		dev_dbg(&dev->dev, " flags: %s\n", "IO");
	else
		dev_dbg(&dev->dev, " flags: %lu\n", flags);
}

/**
 * mnh_check_iatu_bar2 - IATU Bar2 reprogramming
 * @offset: register address
 *
 * Based on the register offset, reprogram IATU BAR2.
 * Due to limitation of BAR2 size, we need to remap the IATU either for
 * SRAM or Cluster register region.
 */
static uint32_t mnh_check_iatu_bar2(uint32_t offset)
{
	uint8_t new_region;
	uint64_t start_addr;
	uint32_t new_offset;
	struct mnh_inb_window iatu;

	if (!mnh_dev || !mnh_dev->pdev)
		return -ENODEV;

	if (mnh_dev->bar2_iatu_region == 0) /* No need to reprogram the IATU */
		return offset;

	if (offset < HW_MNH_PCIE_BAR2_R2_ADDR_START) {
		new_region = 1;
		start_addr = HW_MNH_PCIE_BAR2_R1_ADDR_START;
		new_offset = offset;
	} else {
		new_region = 2;
		start_addr = HW_MNH_PCIE_BAR2_R2_ADDR_START;
		new_offset = offset - HW_MNH_PCIE_BAR2_R2_ADDR_START;
	}

	if (mnh_dev->bar2_iatu_region != new_region) {
		dev_dbg(&mnh_dev->pdev->dev, "IATU BAR2 reprogram - region#%d",
			new_region);

		iatu.mode = BAR_MATCH;
		iatu.bar = 2;
		iatu.region = 1;
		iatu.target_mth_address = start_addr;
		iatu.limit_pcie_address = 0xfff;
		iatu.base_pcie_address = 0x3fffffff00000000;
		mnh_set_inbound_iatu(&iatu);
		mnh_dev->bar2_iatu_region = new_region;
	}

	return new_offset;
}

/**
 * mnh_dma_send_cb - DMA Callback notify
 * @chan: DMA channel being used
 * @dir:  DMA direction
 * @status: DMA transfer status(result)
 *
 * Upon DMA transfer finished, send out the result back to Client and EP
 */
static int mnh_dma_send_cb(uint8_t chan, enum mnh_dma_chan_dir_t dir,
	enum mnh_dma_trans_status_t status)
{

	uint32_t ep_irq = 0;
	uint32_t status_shift = 0, dir_shift = 0;

	mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
			HW_MNH_PCIE_GP_2, sizeof(uint32_t),
			&ep_irq);

	dir_shift = (dir == DMA_AP2EP) ? 0 : 16;
	status_shift = (status == DMA_DONE) ? 0 : 8;
	ep_irq |= 1 << chan << dir_shift << status_shift;

	/* Send IRQ to EP */
	mnh_config_write(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
				HW_MNH_PCIE_GP_2, sizeof(uint32_t),
				ep_irq);
	mnh_send_irq(IRQ_DMA_STATUS);

	/* Send CB to Client */
	if (!IS_NULL(mnh_dev->dma_cb))
			mnh_dev->dma_cb(chan, dir, status);

	return 0;
}

/**
 * API to initialize DMA
 * perform DMA initialization.
 * This includes reset MNH DMA controller logic and check enabled channel
 * and set the MSI address and data.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
static int mnh_dma_init(void)
{
	uint32_t low_addr, high_addr, data, msi_data;
	uint8_t msi_en = 0;
	uint32_t read_imwr = 0, write_imwr = 0;
	int size = sizeof(uint32_t);

	/* Read MSI addr */
	mnh_pcie_config_read(MSI_CAP_ID_NEXT_CTRL_REG, size, &data);
	msi_en = (data & MULTIPLE_MSG_ENABLE_MASK) >> 20;
	mnh_pcie_config_read(MSI_CAP_OFF_04H_REG, size, &low_addr);
	mnh_pcie_config_read(MSI_CAP_OFF_08H_REG, size, &high_addr);
	mnh_pcie_config_read(MSI_CAP_OFF_0CH_REG, size, &msi_data);
	mnh_pcie_config_read(MSI_CAP_OFF_10H_REG, size, &data);

	/* Update DMA MSI addr information */
	mnh_pcie_config_write(DMA_WRITE_DONE_IMWR_LOW_OFF, size, low_addr);
	mnh_pcie_config_write(DMA_WRITE_DONE_IMWR_HIGH_OFF, size, high_addr);
	mnh_pcie_config_write(DMA_WRITE_ABORT_IMWR_LOW_OFF, size, low_addr);
	mnh_pcie_config_write(DMA_WRITE_ABORT_IMWR_HIGH_OFF, size, high_addr);

	mnh_pcie_config_write(DMA_READ_DONE_IMWR_LOW_OFF, size, low_addr);
	mnh_pcie_config_write(DMA_READ_DONE_IMWR_HIGH_OFF, size, high_addr);
	mnh_pcie_config_write(DMA_READ_ABORT_IMWR_LOW_OFF, size, low_addr);
	mnh_pcie_config_write(DMA_READ_ABORT_IMWR_HIGH_OFF, size, high_addr);

	/* Set DMA MSI */
	msi_data = msi_data & (0xFFFFFFF << msi_en);
	dev_dbg(&mnh_dev->pdev->dev, " msi_data: 0x%x, msi_en:%d",
		msi_data, msi_en);
	if (mnh_dev->msi_num > 1) {
		dev_dbg(&mnh_dev->pdev->dev, "%d MSI\n", mnh_dev->msi_num);
		read_imwr = (msi_data | MSI_DMA_READ) |
			(msi_data | MSI_DMA_READ) << 16;
		write_imwr = (msi_data | MSI_DMA_WRITE) |
			(msi_data | MSI_DMA_WRITE) << 16;
	} else if (mnh_dev->msi_num == 1) {
		dev_dbg(&mnh_dev->pdev->dev, "1 MSI\n");
		read_imwr = msi_data | msi_data << 16;
		write_imwr = read_imwr;
	}

	mnh_pcie_config_write(DMA_READ_CH01_IMWR_DATA_OFF, size, read_imwr);
	mnh_pcie_config_write(DMA_READ_CH23_IMWR_DATA_OFF, size, read_imwr);
	mnh_pcie_config_write(DMA_WRITE_CH01_IMWR_DATA_OFF, size, write_imwr);
	mnh_pcie_config_write(DMA_WRITE_CH23_IMWR_DATA_OFF, size, write_imwr);

	return 0;
}

#ifdef CONFIG_MNH_PCIE_MULTIPLE_MSI

/**
 * mnh_pci_dma_irq_handler - MNH PCIE DMA interrupt handler
 * @irq: Linux interrupt number
 * @ptr: Pointer to interrupt-specific data
 */
static irqreturn_t mnh_pci_dma_irq_handler(int irq, void *ptr)
{

	struct mnh_device *dev = (struct mnh_device *)ptr;
	uint32_t data, done, abort;
	uint8_t chan, i = 0;
	uint8_t len = sizeof(data);
	enum mnh_dma_chan_dir_t dir;
	enum mnh_dma_trans_status_t status = -1;

	dev_dbg(&dev->pdev->dev, "DMA IRQ Handler: %d\n", irq);

	if (irq < dev->pdev->irq + MSI_DMA_READ ||
		irq > dev->pdev->irq + MSI_DMA_WRITE) {
		dev_err(&dev->pdev->dev, "Invalid DMA IRQ : %d\n", irq);
		return IRQ_HANDLED;
	}

	if (irq == dev->pdev->irq + MSI_DMA_READ) {
		dir = DMA_AP2EP;
		mnh_pcie_config_read(DMA_READ_INT_STATUS_OFF, len, &data);
		done = data & DMA_READ_INT_STATUS_OFF_DONE_STATUS;
		abort = (data & DMA_READ_INT_STATUS_OFF_ABORT_STATUS) >> 16;
		/* Clear interrupt */
		mnh_pcie_config_write(DMA_READ_INT_CLEAR_OFF, len, data);

	} else if (irq == dev->pdev->irq + MSI_DMA_WRITE) {
		dir = DMA_EP2AP;
		mnh_pcie_config_read(DMA_WRITE_INT_STATUS_OFF, len, &data);
		done = data & DMA_WRITE_INT_STATUS_OFF_DONE_STATUS;
		abort = (data & DMA_WRITE_INT_STATUS_OFF_ABORT_STATUS) >> 16;

		/* Clear interrupt */
		mnh_pcie_config_write(DMA_WRITE_INT_CLEAR_OFF, len, data);
	} else {
		dev_err(&dev->pdev->dev,  "Invalid DMA IRQ : %d\n", irq);
		return IRQ_HANDLED;
	}

	for (i = 0; i < DMA_MAX_CHAN; i++) {
		chan = 1 << i;
		if (done & chan) {
			status = DMA_DONE;
			dev_dbg(&dev->pdev->dev, "DMA DONE:ch[%d], dir[%d]",
				i, dir);
			mnh_dma_send_cb(i, dir, status);
		}
		if (abort & chan) {
			status = DMA_ABORT;
			dev_dbg(&dev->pdev->dev, "DMA ABORT:ch[%d], dir[%d]",
				i, dir);
			mnh_dma_send_cb(i, dir, status);
		}
	}
	return IRQ_HANDLED;
}

/**
 * mnh_pci_irq_handler - MNH PCIE interrupt handler
 * @irq: Linux interrupt number
 * @ptr: Pointer to interrupt-specific data
 *
 * Handles MNH interrupts signaled using MSI
 */
static irqreturn_t mnh_pci_irq_handler(int irq, void *ptr)
{
	struct mnh_device *dev = (struct mnh_device *)ptr;
	uint32_t	  local_irq;

	dev_dbg(&dev->pdev->dev, "irq_handler: %d\n", irq);

	if ((irq < dev->pdev->irq + MSI_GENERIC_START) ||
		(irq > dev->pdev->irq + MSI_GENERIC_END)) {
		dev_err(&dev->pdev->dev, "Invalid IRQ : %d\n", irq);
		return IRQ_HANDLED;
	}

	if (irq == dev->pdev->irq + MSI_BOOTSTRAP_SET) {
		/* Read ring_buf_addr */
		mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
			HW_MNH_PCIE_GP_1, sizeof(uint32_t),
			&mnh_dev->ring_buf_addr);
		dev_dbg(&dev->pdev->dev, "MNH ring buffer addr: 0x%x\n",
				mnh_dev->ring_buf_addr);
	}


	/* If MSG Callback is set, then call the callback to notify the MSI
	has been delivered */
	if (!IS_NULL(dev->msg_cb)) {
		local_irq = irq - dev->pdev->irq;
		dev->msg_cb(local_irq);
		dev_dbg(&dev->pdev->dev, "MSI sent out to client:0x%x\n",
			local_irq);
	}

	return IRQ_HANDLED;
}
#endif
/**
 * mnh_pci_irq_single_handler - MNH PCIE interrupt handler for single MSI
 * @irq: Linux interrupt number
 * @ptr: Pointer to interrupt-specific data
 *
 * Handles MNH/DMA interrrupts when single MSI is being used
 */
static irqreturn_t mnh_pci_irq_single_handler(int irq, void *ptr)
{
	struct mnh_device *dev = (struct mnh_device *)ptr;
	uint32_t msi_vector;
	uint32_t data1, data2, done, abort;
	uint8_t chan, i = 0;
	uint8_t len = sizeof(uint32_t);
	enum mnh_dma_chan_dir_t dir;
	enum mnh_dma_trans_status_t status = -1;

	dev_dbg(&dev->pdev->dev, "irq_single_handler: %d\n", irq);

	/* Check DMA interrupt */
	mnh_pcie_config_read(DMA_READ_INT_STATUS_OFF, len, &data1);
	mnh_pcie_config_read(DMA_WRITE_INT_STATUS_OFF, len, &data2);
	if (data1) {
		dev_dbg(&dev->pdev->dev, "DMA Read IRQ found -0x%x\n", data1);
		dir = DMA_AP2EP;
		done = data1 & DMA_READ_INT_STATUS_OFF_DONE_STATUS;
		abort = (data1 & DMA_READ_INT_STATUS_OFF_ABORT_STATUS) >> 16;
		/* Clear interrupt */
		mnh_pcie_config_write(DMA_READ_INT_CLEAR_OFF, len, data1);
	} else if (data2) {
		dev_dbg(&dev->pdev->dev, "DMA Write IRQ found -0x%x\n", data2);
		dir = DMA_EP2AP;
		done = data2 & DMA_WRITE_INT_STATUS_OFF_DONE_STATUS;
		abort = (data2 & DMA_WRITE_INT_STATUS_OFF_ABORT_STATUS) >> 16;
		/* Clear interrupt */
		mnh_pcie_config_write(DMA_WRITE_INT_CLEAR_OFF, len, data2);
	}

	if (data1 || data2) {
		for (i = 0; i < DMA_MAX_CHAN; i++) {
			chan = 1 << i;
			if (done & chan) {
				status = DMA_DONE;
				dev_dbg(&dev->pdev->dev, "DMA DONE:ch[%d], dir[%d]",
					i, dir);
				mnh_dma_send_cb(i, dir, status);
			}
			if (abort & chan) {
				status = DMA_ABORT;
				dev_dbg(&dev->pdev->dev, "DMA ABORT:ch[%d], dir[%d]",
					i, dir);
				mnh_dma_send_cb(i, dir, status);
			}
		}
		return IRQ_HANDLED;
	}


	mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
			HW_MNH_PCIE_GP_3, sizeof(uint32_t), &msi_vector);
	dev_dbg(&dev->pdev->dev, "msi_vector: 0x%x\n", msi_vector);

	/* Check MSI generic interrupt */
	if ((msi_vector >= MSI_GENERIC_START) ||
		(msi_vector <=  MSI_GENERIC_END)) {
		if (msi_vector ==  MSI_BOOTSTRAP_SET) {
			/* Read ring_buf_addr */
			mnh_config_read(HW_MNH_PCIE_CLUSTER_ADDR_OFFSET +
				HW_MNH_PCIE_GP_1, sizeof(uint32_t),
				&mnh_dev->ring_buf_addr);
			dev_dbg(&dev->pdev->dev, "MNH ring buffer addr: 0x%x\n",
					mnh_dev->ring_buf_addr);
		}
		/* If MSG Callback is set, then call the callback to notify
		the MSI has been delivered */
		if (!IS_NULL(dev->msg_cb)) {
			dev->msg_cb(msi_vector);
			dev_dbg(&dev->pdev->dev, "MSI sent out to client:0x%x\n",
				msi_vector);
		}
	} else if (msi_vector != 0)
		dev_err(&dev->pdev->dev, "Invalid IRQ : %d\n", msi_vector);

	return IRQ_HANDLED;

}

/**
 * mnh_irq_init - Setting up MNH PCIE MSI and register irq handler
 * @pdev: PCIE dev
 * @return returns msi number initialized successfully
 */
static int mnh_irq_init(struct pci_dev *pdev)
{

#ifdef CONFIG_MNH_PCIE_MULTIPLE_MSI
	int err, vector, i;
	uint32_t msinum = MSI_DMA_WRITE + 1;

	vector = pci_enable_msi_range(pdev, 1, msinum);
	dev_err(&pdev->dev, "vector :%d , msi_num:%d, irq:%d\n",
		vector, msinum, pdev->irq);
	if (vector < msinum) {
		dev_err(&pdev->dev, "failed to enable MSI range :%d, err:%d\n",
			msinum, vector);
		if (vector >= 1) {
			/* Fall back to 1 MSI */
			err = request_threaded_irq(pdev->irq, NULL,
				mnh_pci_irq_single_handler,
				IRQF_SHARED | IRQF_ONESHOT,
				MODULE_NAME, mnh_dev);
			if (err) {
				dev_err(&pdev->dev,
					"fail to req msi %d: err:%d\n",
					pdev->irq, err);
				goto error;
			}
			return 1;
		}
		goto error;
	}


	/* MSI IRQs request */
	for (i = MSI_GENERIC_START; i <= MSI_GENERIC_END; i++) {
		err = request_threaded_irq(pdev->irq + i,
			NULL, mnh_pci_irq_handler,
			IRQF_SHARED | IRQF_ONESHOT, MODULE_NAME, mnh_dev);
		if (err) {
			dev_err(&pdev->dev, "failed to req MSI:%d, err:%d\n",
				pdev->irq + i, err);
			goto free_irq;
		}
		dev_err(&pdev->dev, "request irq:%d\n", pdev->irq+i);
	}

	/* DMA READ MSI IRQ request */
	err = request_threaded_irq(pdev->irq + MSI_DMA_READ,
			NULL, mnh_pci_dma_irq_handler,
			IRQF_SHARED | IRQF_ONESHOT, MODULE_NAME, mnh_dev);
	dev_err(&pdev->dev, "request irq:%d\n", pdev->irq+MSI_DMA_READ);
	if (err) {
		dev_err(&pdev->dev, "request DMA read irq failure. irq = %d\n",
			   pdev->irq + MSI_DMA_READ);
		goto free_irq;
	}

	/* DMA WRITE MSI IRQ request */
	err = request_threaded_irq(pdev->irq + MSI_DMA_WRITE,
		NULL, mnh_pci_dma_irq_handler,
		IRQF_SHARED | IRQF_ONESHOT, MODULE_NAME, mnh_dev);
	dev_err(&pdev->dev, "request irq:%d\n", pdev->irq+MSI_DMA_WRITE);
	if (err) {
		dev_err(&pdev->dev, "request DMA write irq failure. irq = %d\n",
			   pdev->irq + MSI_DMA_WRITE);
		goto free_dma_irq;
	}

	return msinum;

free_dma_irq:
	free_irq(pdev->irq + MSI_DMA_READ, mnh_dev);
free_irq:

	for (--i; i >= MSI_GENERIC_START; i--)
		free_irq(pdev->irq + i, mnh_dev);

#else
	int err;

	err = pci_enable_msi(pdev);
	if (err) {
		dev_err(&pdev->dev, "fail to allocate msi\n");
		return 0;
	}

	err = request_threaded_irq(pdev->irq, NULL,
		mnh_pci_irq_single_handler,
		IRQF_SHARED | IRQF_ONESHOT,
		MODULE_NAME, mnh_dev);

	if (err) {
		dev_err(&pdev->dev, "fail to req msi %d: err:%d\n",
			pdev->irq, err);
		goto error;
	}

	return 1;

#endif

error:
	dev_err(&pdev->dev, "Error setting up irq\n");
	pci_disable_msi(pdev);
	return 0;
}

static int mnh_irq_deinit(struct pci_dev *pdev)
{
	int i;

	dev_dbg(&mnh_dev->pdev->dev, "IRQ deinitializing..\n");

	if (mnh_dev->msi_num == 1) {
		dev_dbg(&mnh_dev->pdev->dev, "free-irq : %d", pdev->irq);
		free_irq(pdev->irq, mnh_dev);
		pci_disable_msi(pdev);
	} else if (mnh_dev->msi_num > 1) {
		for (i = MSI_GENERIC_START; i <= MSI_GENERIC_END; i++) {
			dev_dbg(&mnh_dev->pdev->dev, "free-irq:%d",
				pdev->irq + i);
			free_irq(pdev->irq + i, mnh_dev);
		}
		free_irq(pdev->irq + MSI_DMA_READ, mnh_dev);
		free_irq(pdev->irq + MSI_DMA_WRITE, mnh_dev);
		pci_disable_msi(pdev);
	}

	return 0;

}

static void setup_smmu(struct pci_dev *pdev)
{
	struct dma_iommu_mapping *mapping;
	int atomic_ctx = 1;
	int bypass_enable = 1;
	int ret;

/* Following stolen from msm_11ad.c */
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

/*
 * mnh_pci_fixup - Fixup routine before probe
 *
 * @pdev: PCI device structure
 */
static void mnh_pci_fixup(struct pci_dev *pdev)
{
	dev_dbg(&pdev->dev, "MNH PCI Entering mnh_pci_fixup\n");
	if (pdev->class == PCI_CLASS_NOT_DEFINED) {
		dev_info(&pdev->dev, "setting pcie class\n");
		pdev->class = PCI_CLASS_MEMORY_OTHER;
	}
}

int mnh_pci_init_resume(void)
{
	int err;
	int i = 0;
	uint32_t  magic;
	struct mnh_inb_window iatu;
	struct pci_dev *pdev = mnh_dev->pdev;

	/* enable pci dev */
	err = pci_enable_device(pdev);
	if (err)
		dev_err(&pdev->dev, "failed to enable pci device.\n");

	/* set PCI host mastering  */
	pci_set_master(pdev);

	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(64)) ||
		dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64))) {

		err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (err)
			err = dma_set_coherent_mask(&pdev->dev,
							DMA_BIT_MASK(32));
	}
	if (err)
		dev_err(&pdev->dev, "No usable DMA configuration, aborting\n");

	for (i = 0; i < BAR_MAX_NUM; i++)
		mnh_check_pci_resources(pdev, i);

	mnh_dma_init();

	/* Programing the inbound IATU */
	iatu.mode = BAR_MATCH;
	iatu.bar = 2;
	iatu.region = 1;
	iatu.target_mth_address = HW_MNH_PCIE_BAR_2_ADDR_START;
	iatu.limit_pcie_address = 0xfff;
	iatu.base_pcie_address = 0x3fffffff00000000;
	mnh_set_inbound_iatu(&iatu);

	iatu.mode = BAR_MATCH;
	iatu.bar = 4;
	iatu.region = 2;
	iatu.target_mth_address = HW_MNH_PCIE_BAR_4_ADDR_START;
	iatu.limit_pcie_address = 0xfff;
	iatu.base_pcie_address = 0x3ffffff00000000;
	mnh_set_inbound_iatu(&iatu);

	err = mnh_pcie_config_read(0, sizeof(uint32_t), &magic);
	if (err) {
		dev_err(&pdev->dev, "failed to read magic register: 0x%08x (%d)\n",
			magic, err);
	}

	dev_dbg(&pdev->dev, "MNH PCIe reinitialization successful.\n");

	return 0;
}

/**
 * mnh_pci_probe - Device Initialization Routine
 *
 * @pdev: PCI device structure
 * @ent: entry in kcs_pci_tbl
 *
 * Return: 0 on success, <0 on failure.
 */
static int mnh_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int err;
	int i = 0;
	uint32_t  magic;
	struct mnh_inb_window iatu;

	dev_dbg(&pdev->dev, "MNH PCI Device found [%04x:%04x] (rev %x)\n",
		 (int)pdev->vendor, (int)pdev->device, (int)pdev->revision);

	mnh_dev = kzalloc(sizeof(struct mnh_device), GFP_KERNEL);
	if (mnh_dev == NULL) {
			err = -ENOMEM;
		goto end;
	}

	mnh_dev->pdev = pdev;

	/* probe is called after pcie has enumerated, so set powered flag */
	mnh_dev->powered = true;

	/* enable pci dev */
	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to enable pci device.\n");
		goto free_device;
	}

	/* set PCI host mastering  */
	pci_set_master(pdev);

	/* pci request regions */
	err = pci_request_regions(pdev, MODULE_NAME);
	dev_dbg(&pdev->dev, "return value of request_region:%x", err);

	if (err) {
		dev_err(&pdev->dev, "failed to get pci regions.\n");
		goto disable_device;
	}


	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(64)) ||
		dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64))) {

		err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (err)
			err = dma_set_coherent_mask(&pdev->dev,
							DMA_BIT_MASK(32));
	}
	if (err) {
		dev_err(&pdev->dev, "No usable DMA configuration, aborting\n");
		goto release_regions;
	}

	for (i = 0; i < BAR_MAX_NUM; i++)
		mnh_check_pci_resources(pdev, i);

	/* mapping PCIE configuration memory */
	mnh_dev->pcie_config = pci_ioremap_bar(pdev, BAR_0);

	if (!mnh_dev->pcie_config) {
		dev_err(&pdev->dev, "mapping BAR0/1 device memory failure.\n");
		err = -ENOMEM;
		goto release_regions;
	}

	/* mapping Peripharal configuration memory */
	mnh_dev->config = pci_ioremap_bar(pdev, BAR_2);
		if (!mnh_dev->config) {
			dev_err(&pdev->dev, "mapping BAR2/3 memory failure.\n");
			err = -ENOMEM;
			goto unmap_bar;
	}

	/* mapping Peripharal configuration memory */
	mnh_dev->ddr = pci_ioremap_bar(pdev, BAR_4);
		if (!mnh_dev->ddr) {
			dev_err(&pdev->dev, "mapping BAR4/5 memory failure.\n");
			err = -ENOMEM;
			goto unmap_bar;
	}



/* IRQ handling */
#if CONFIG_PCI_MSI
	mnh_dev->msi_num = mnh_irq_init(pdev);
#endif

	pci_set_drvdata(pdev, (void *)mnh_dev);

	mnh_dma_init();

	/* Programing the inbound IATU */
	iatu.mode = BAR_MATCH;
	iatu.bar = 2;
	iatu.region = 1;
	iatu.target_mth_address = HW_MNH_PCIE_BAR_2_ADDR_START;
	iatu.limit_pcie_address = 0xfff;
	iatu.base_pcie_address = 0x3fffffff00000000;
	mnh_set_inbound_iatu(&iatu);

	iatu.mode = BAR_MATCH;
	iatu.bar = 4;
	iatu.region = 2;
	iatu.target_mth_address = HW_MNH_PCIE_BAR_4_ADDR_START;
	iatu.limit_pcie_address = 0xfff;
	iatu.base_pcie_address = 0x3ffffff00000000;
	mnh_set_inbound_iatu(&iatu);

	setup_smmu(pdev);

	err = mnh_pcie_config_read(0, sizeof(uint32_t), &magic);
	if (err) {
		dev_err(&pdev->dev, "failed to read magic register: 0x%08x (%d)\n",
			magic, err);
	}

	dev_dbg(&pdev->dev, "MNH PCIe initialization successful.\n");

	return 0;

unmap_bar:
	if (mnh_dev->pcie_config)
		pci_iounmap(pdev, mnh_dev->pcie_config);
	if (mnh_dev->config)
		pci_iounmap(pdev, mnh_dev->config);
	if (mnh_dev->ddr)
		pci_iounmap(pdev, mnh_dev->ddr);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
free_device:
        pci_set_drvdata(pdev, NULL);
	kfree(mnh_dev);
end:
	dev_err(&pdev->dev, "initialization failed.\n");
	return err;
}

/**
 * mnh_pci_remove - Device Removal Routine
 *
 * @pdev: PCI device structure
 *
 * mnh_pci_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.
 */
static void mnh_pci_remove(struct pci_dev *pdev)
{
	struct mnh_device *dev;

	dev = pci_get_drvdata(pdev);
	if (!dev) {
		dev_err(&pdev->dev, "get Drvdata fail\n");
		return;
	}

	mnh_irq_deinit(pdev);

	if (dev->pcie_config) {
		dev_err(&pdev->dev, "ioummap pcie_config\n");
		pci_iounmap(pdev, dev->pcie_config);
	}
	if (dev->config) {
		dev_err(&pdev->dev, "ioummap config\n");
		pci_iounmap(pdev, dev->config);
	}
	if (dev->ddr) {
		dev_err(&pdev->dev, "ioummap ddr\n");
		pci_iounmap(pdev, dev->ddr);
	}

	pci_set_drvdata(pdev, NULL);
	kfree(dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	dev_dbg(&pdev->dev, "MNH PCIe driver is removed\n");
}

int mnh_pci_suspend(void)
{
	struct pci_dev *pdev = mnh_dev->pdev;
	struct device *dev = &mnh_dev->pdev->dev;

	dev_dbg(dev, "%s: enter\n", __func__);

	/* disable IRQs */
#ifdef CONFIG_MNH_PCIE_MULTIPLE_MSI
	for (i = MSI_GENERIC_START; i <= MSI_GENERIC_END; i++)
		disable_irq(pdev->irq + i);
	disable_irq(pdev->irq + MSI_DMA_READ);
	disable_irq(pdev->irq + MSI_DMA_WRITE);
#else
	disable_irq(pdev->irq);
#endif

	mnh_dev->powered = false;

	dev_dbg(dev, "%s: exit\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_pci_suspend);

int mnh_pci_resume(void)
{
	struct pci_dev *pdev = mnh_dev->pdev;
	struct device *dev = &mnh_dev->pdev->dev;
	int ret = 0;

	dev_dbg(dev, "%s: enter\n", __func__);

	mnh_dev->powered = true;

	ret = mnh_pci_init_resume();
	if (ret) {
		dev_err(dev, "%s: mnh_pci_init_resume failed (%d)\n",
			__func__, ret);
		return ret;
	}

	/* enable IRQs */
#ifdef CONFIG_MNH_PCIE_MULTIPLE_MSI
	for (i = MSI_GENERIC_START; i <= MSI_GENERIC_END; i++)
		enable_irq(pdev->irq + i);
	enable_irq(pdev->irq + MSI_DMA_READ);
	enable_irq(pdev->irq + MSI_DMA_WRITE);
#else
	enable_irq(pdev->irq);
#endif

	dev_dbg(dev, "%s: exit\n", __func__);

	return ret;
}
EXPORT_SYMBOL_GPL(mnh_pci_resume);

#if 0 /* TODO: implement error handlers below */
static const struct pci_error_handlers mnh_pci_err_handler = {
	.error_detected = mnh_pci_error_detected,
	.slot_reset = mnh_pci_slot_reset,
	.resume = mnh_pci_resume
};
#endif

/*
 *	PCI driver structure
 */
static struct pci_driver mnh_pci_driver = {
	.name = MODULE_NAME,
	.id_table = mnh_pci_tbl,
	.probe = mnh_pci_probe,
	.remove = mnh_pci_remove,
	.shutdown = mnh_pci_remove,
};

module_pci_driver(mnh_pci_driver);

DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,
			 PCI_DEVICE_ID_MNH,
			 mnh_pci_fixup);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("MNH PCI HOST DRIVER");
MODULE_LICENSE("GPL v2");

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

#ifndef __ABC_PCIE_H
#define __ABC_PCIE_H

#include <linux/cdev.h>
#include <linux/dma-direction.h>
#include <linux/pci.h>

#define DRV_NAME_ABC_PCIE	"abc-pcie"
#define DRV_NAME_ABC_PCIE_BLK_FSYS "abc-pcie-fsys"
#define DRV_NAME_ABC_PCIE_CMU	"abc-pcie-cmu"
#define DRV_NAME_ABC_PCIE_DMA   "abc-pcie-dma"
#define DRV_NAME_ABC_PCIE_IPU	"abc-pcie-ipu"
#define DRV_NAME_ABC_PCIE_TPU	"abc-pcie-tpu"
#define DRV_NAME_ABC_PCIE_PMU	"abc-pcie-pmu"
#define DRV_NAME_ABC_PCIE_SYSREG "abc-pcie-sysreg"
#define DRV_NAME_ABC_PCIE_SPI	"abc-pcie-spi"
#define DRV_NAME_ABC_PCIE_UART	"abc-pcie-uart"
#define DRV_NAME_ABC_PCIE_DMA   "abc-pcie-dma"

/*todo..for now keeping max. minor count 1
 *can be increased further on need basis.
 */

#define MAX_MINOR_COUNT  1
#define FSYS_MINOR_NUMBER 2

#define MAX_DMA_INT	16
#define MAX_ABC_MSI	32
/* 32 MSI interrupts (0-31) + 12 Mux interrupts on 31st MSI */
#define MAX_INT		44

/* todo..add platform specific data */
struct abc_pcie_devdata;

enum pci_barno {
	BAR_0,
	BAR_1,
	BAR_2,
	BAR_3,
	BAR_4,
	BAR_5,
};

enum abc_address_map {
	TPU_START = 0x0,
	IPU_START = 0x200000,
	MIF_START = 0x500000,
	FSYS_START = 0x700000,
	DBI_START = 0x800000,
	FSYS_NIC_GPV = 0x900000,
	FSYS_RSVD = 0xA00000,
	AON_AXI2APB = 0xB00000,
	AON_NIC_GPV = 0xC00000,
	AON_CM0_DEBUG = 0xD00000,
	CORE_GPV = 0xE00000,
	SFR_RSVD = 0xF00000,
	SFR_MAX  = 0xFFFFFF,
};

#define ABC_SFR_BASE	0x10000000
#define ABC_MISC_SFR_REGION_MASK	0xFFFF

enum abc_dma_trans_status {
	DMA_DONE = 0, /* DONE: DMA DONE interrupt */
	DMA_ABORT,     /* ABORT: DMA ABORT interrupt */
};

typedef int (*irq_cb_t)(uint32_t irq);
typedef int (*irq_cb_t2)(uint32_t irq, void *payload);
typedef int (*irq_dma_cb_t)(uint8_t chan, enum dma_data_direction dir,
				enum abc_dma_trans_status status);

struct abc_device {
	int gpio;
	struct device	*dev;
	struct pci_dev	*pdev;
	struct cdev c_dev;
	void __iomem	*pcie_config;
	void __iomem	*ipu_config;
	void __iomem	*tpu_config;
	void __iomem	*fsys_config;
	void __iomem	*aon_config;
	void __iomem	*sfr_misc_config;
	void __iomem	*base_config;
	void __iomem	*bar2_base;
	void __iomem	*bar4_base;
	struct pci_bus_region bar_base[BAR_4];
	unsigned char *wr_buf;
	dma_addr_t wr_buf_addr;
	unsigned char *rd_buf;
	dma_addr_t rd_buf_addr;
	irq_dma_cb_t	dma_cb[MAX_DMA_INT];
	irq_cb_t	sys_cb[MAX_INT];
	irq_cb_t2	sys_cb2[MAX_INT];
	void		*handler_payload[MAX_INT];
	spinlock_t      lock;
};

enum {
	ABC_DMA_WR = 1,
	ABC_DMA_RD,
};

enum inb_mode {BAR_MATCH, MEM_MATCH};

struct inb_region {
	enum inb_mode mode;
	uint8_t region;
	uint8_t memmode;
	uint32_t bar;
	uint32_t base_address;
	uint32_t u_base_address;
	uint32_t limit_address;
	uint32_t target_pcie_address;
	uint32_t u_target_pcie_address;
};
int set_inbound_iatu(struct inb_region inb);

struct outb_region {
	uint32_t region;
	uint8_t memmode;
	uint32_t base_address;
	uint32_t u_base_address;
	uint32_t limit_address;
	uint32_t target_pcie_address;
	uint32_t u_target_pcie_address;
};
int set_outbound_iatu(struct outb_region outb);

struct config_write {
	u32 offset;
	u32 len;
	u32 data;
};

struct config_read {
	u32 offset;
	u32 len;
	u32 *data;
};

struct dma_element_t {
	uint32_t len;
	uint32_t src_addr;
	uint32_t src_u_addr;
	uint32_t dst_addr;
	uint32_t dst_u_addr;
	uint8_t	 chan;
};

struct abc_dma_desc {
	uint32_t buf_addr;
	uint32_t buf_u_addr;
	uint32_t len;
	uint8_t	 chan;
	void  *local_buf;      /* local buffer address */
};

struct abc_pcie_pm_ctrl {
	int pme_en;
	int aspm_L11;
	int aspm_L12;
	int l0s_en;
	int l1_en;
};

int dma_sblk_start(uint8_t chan, enum dma_data_direction dir,
		   struct dma_element_t *blk);
int dma_mblk_start(uint8_t chan, enum dma_data_direction dir,
			    phys_addr_t start_addr);

#define ABC_PCIE_CONFIG_READ        _IOW('P', 0x1, struct config_read)
#define ABC_PCIE_CONFIG_WRITE       _IOW('P', 0x2, struct config_write)
#define ABC_PCIE_SET_IB_IATU        _IOW('P', 0x3, struct inb_region)
#define ABC_PCIE_SET_OB_IATU        _IOW('P', 0x4, struct outb_region)
#define ABC_PCIE_ALLOC_BUF          _IOW('P', 0x5, unsigned long)
#define ABC_PCIE_SET_RD_DMA         _IOW('P', 0x6, struct abc_dma_desc)
#define ABC_PCIE_SET_WR_DMA         _IOW('P', 0x7, struct abc_dma_desc)


int pcie_config_read(u32 offset, u32 len, u32 *data);
int pcie_config_write(u32 offset, u32 len, u32 data);
int aon_config_read(u32 offset, u32 len, u32 *data);
int aon_config_write(u32 offset, u32 len, u32 data);
int ipu_config_read(u32 offset, u32 len, u32 *data);
int ipu_config_write(u32 offset, u32 len, u32 data);
int tpu_config_read(u32 offset, u32 len, u32 *data);
int tpu_config_write(u32 offset, u32 len, u32 data);
int ddr_config_read(u32 offset, u32 len, u32 *data);
int ddr_config_write(u32 offset, u32 len, u32 data);
int abc_reg_dma_irq_callback(irq_dma_cb_t dma_cb, int dma_chan);
int abc_reg_irq_callback(irq_cb_t sys_cb, int irq_no);
int abc_reg_irq_callback2(irq_cb_t2 sys_cb, int irq_no, void *payload);
void *abc_alloc_coherent(size_t size, dma_addr_t *dma_addr);
void abc_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr);
dma_addr_t abc_dma_map_page(struct page *page, size_t offset, size_t size,
		enum dma_data_direction dir);
dma_addr_t abc_dma_map_single(void *ptr,  size_t size,
		enum dma_data_direction dir);
void abc_dma_unmap_page(dma_addr_t addr,  size_t size,
		enum dma_data_direction dir);
void abc_dma_unmap_single(dma_addr_t addr,  size_t size,
		enum dma_data_direction dir);
int abc_set_pcie_pm_ctrl(struct abc_pcie_pm_ctrl *pmctrl);
int abc_set_pcie_link_l1(bool enabled);
#endif

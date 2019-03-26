/*
 * Android coprocessor DMA library
 *
 * Copyright 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ABC_PCIE_DMA_H
#define __ABC_PCIE_DMA_H

#include <linux/mfd/abc-pcie.h>

/**
 * Multi-block transfer configuration
 */
/* linked list headers */
#define LL_DATA_ELEMENT         0x1
#define LL_IRQ_DATA_ELEMENT     0x9
#define LL_LINK_ELEMENT         0x5
#define LL_LAST_LINK_ELEMENT    0x7

struct abc_pcie_dma_uapi {
	struct miscdevice mdev;

};

struct abc_pcie_dma {
	struct platform_device *pdev;
	struct device *dma_dev;
	struct abc_pcie_dma_uapi uapi;
};

/**
 * Structure of scatter gather list entry
 */
struct abc_pcie_sg_entry {
	phys_addr_t paddr; /* Physical address */
	size_t size;       /* size of entry */
};

struct abc_pcie_sg_list {
	struct page **mypage;
	struct scatterlist *sc_list;
	int n_num;
	int length;
	enum dma_data_direction dir;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sg_table;
};

/**
 * Structure of a single linked list entry
 */
struct abc_pcie_dma_ll_element {
	uint32_t header;
	uint32_t size;
	uint32_t sar_low;
	uint32_t sar_high;
	uint32_t dar_low;
	uint32_t dar_high;
};


/*******************************************************************************
 *
 *	APIs
 *
 ******************************************************************************/

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * local buffer
 * @param[in]  dmadest  Starting virtual addr of the DMA destination
 * @param[in]  size Totalsize of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *             allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *             page list, scatter gather list and num of its entries.
 * @return     The number of sg[] entries filled out by the routine, negative if
 *             overflow or sg[] not allocated.
 */
int abc_pcie_user_local_buf_sg_build(void *dmadest, size_t size,
				     struct abc_pcie_sg_entry **sg,
				     struct abc_pcie_sg_list *sgl);

/**
 * API to release scatter gather list for a user buffer
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *            abc_pcie_user_local_buf_sg_build
 * @return 0  for SUCCESS
 */
int abc_pcie_user_local_buf_sg_destroy(struct abc_pcie_sg_entry **sg,
				       struct abc_pcie_sg_list *sgl);

/**
 * API to build a scatter-gather list for multi-block DMA transfer for a
 * dma_buf
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[in] off  Offset within dma_buf to begin building scatter-gather list
 *                 from.
 * @param[in] size Size, in bytes, of transfer.
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *                 allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries.
 * @return 0        on SUCCESS
 *         negative on failure
 */
int abc_pcie_sg_retrieve_from_dma_buf(int fd, uint64_t off, uint64_t size,
				      struct abc_pcie_sg_entry **sg,
				      struct abc_pcie_sg_list *sgl);

/**
 * API to release a scatter-gather list for a dma_buf
 * @param[in] sgl Pointer to the scatter gather list that was built during
 *                abc_pcie_sg_retrieve_from_dma_buf.
 * @param[in] sg  Pointer to array of struct abc_pcie_sg_entry that was
 *                allocated by abc_pcie_sg_retrieve_from_dma_buf. May be NULL.
 * @return 0 for SUCCESS
 */
int abc_pcie_sg_release_from_dma_buf(struct abc_pcie_sg_list *sgl,
					struct abc_pcie_sg_entry *sg);

/**
 * Top-level API for DMA transfers for kernel space memory
 * TODO(b/114422444): AB DMA driver Clean-up & Improvements
 * Generate scatterlist, decide on transfer mode (sblk/mblk)
 * @dma_desc[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_issue_dma_xfer_vmalloc(struct abc_pcie_dma_desc *dma_desc);

/**
 * Issue a DMA transfer
 * @param[in] dma_desc descriptor structure for the DMA transaction
 * @return 0 on success
 */
int abc_pcie_issue_dma_xfer(struct abc_pcie_dma_desc *dma_desc);

/**
 * Prepare for multi-block transfer:
 * Generates linked list, uploads the list and initiates transfer
 * @param[in] src_sg scatterlist of the source buffer
 * @param[in] dst_sg scatterlist of the destination buffer
 * @param[in] dir direction of the DMA transfer
 * @return 0 on success
 */
int abc_pcie_setup_mblk_xfer(struct abc_pcie_sg_entry *src_sg,
			     struct abc_pcie_sg_entry *dst_sg,
			     struct abc_pcie_dma_desc *dma_desc);


int init_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi);
void remove_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi);

#endif /* __ABC_PCIE_DMA_H */

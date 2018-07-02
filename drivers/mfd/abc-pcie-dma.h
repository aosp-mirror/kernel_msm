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

#define DMA_MAX_CHAN 4

/**
 * Multi-block transfer configuration
 */
/* maximum number of elements per list */
#define DMA_LL_LENGTH 256
/* maximum supported number of lists */
#define MAX_LL_ELEMENT 64
/* linked list headers */
#define LL_DATA_ELEMENT         0x1
#define LL_IRQ_DATA_ELEMENT     0x19
#define LL_LINK_ELEMENT         0x5
#define LL_LAST_LINK_ELEMENT    0x7

/* SRAM Address where Linked List is located. */
#define HW_ABC_PCIE_LL_BASE    	0x26000

/**
 * DMA Channel status type
 */
enum abc_pcie_dma_chan_status {
	DMA_CHAN_UNKNOWN = 0, /* Unknown */
	DMA_CHAN_RUNNING,     /* Channel is running */
	DMA_CHAN_HALTED,      /* Channel is halted */
	DMA_CHAN_STOPPED,     /* Channel is stopped */
	DMA_CHAN_QUEUING      /* Queuing. Not a real HW status */
};

/**
 * Structure used for DMA Channel status
 */
struct abc_pcie_dma_state_info {
	uint8_t status;   /* dma chan status abc_pcie_dma_chan_status */
	uint32_t err;     /* hw error status if there is any */
	uint64_t xferred; /* size of data have been transferred on ch */
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

/**
 * Structure of a complete linked list for mblk transfers
 */
struct abc_pcie_dma_ll {
	/* total number of lists in the linked list */
	uint32_t size;
	/* array of pointers to each list in the linked list */
	struct abc_pcie_dma_ll_element *ll_element[MAX_LL_ELEMENT];
	/* array of physical addresses of each list's first element in the LL */
	dma_addr_t dma[MAX_LL_ELEMENT];
};


/*******************************************************************************
 *
 *	APIs
 *
 ******************************************************************************/

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * buffer
 * @param[in]  dmadest  Starting virtual addr of the DMA destination
 * @param[in]  size Totalsize of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *             allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *             page list, scatter gather list and num of its entries.
 * @return     The number of sg[] entries filled out by the routine, negative if
 *             overflow or sg[] not allocated.
 */
int abc_pcie_user_buf_sg_build(void *dmadest, size_t size,
			       struct abc_pcie_sg_entry **sg,
			       struct abc_pcie_sg_list *sgl);

/**
 * API to release scatter gather list for a user buffer
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *            abc_pcie_user_buf_sg_build
 * @return 0  for SUCCESS
 */
int abc_pcie_user_buf_sg_destroy(struct abc_pcie_sg_list *sgl);

/**
 * API to verify that a scatter gather list is valid
 * @param[in] *sg  pointer to the scatter gather list that was built during
 *		abc_pcie_user_buf_sg_build
 * @param[in] size  size of the list in bytes
 * @param[in] sgl  pointer to the scatter gather local data, if any
 * @return 0 for SUCCESS
 */
int abc_pcie_sg_verify(struct abc_pcie_sg_entry *sg, size_t size,
		       struct abc_pcie_sg_list *sgl);

/**
 * API to build a scatter-gather list for multi-block DMA transfer for a
 * dma_buf
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *                 allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries.
 * @return 0        on SUCCESS
 *         negative on failure
 */
int abc_pcie_sg_retrieve_from_dma_buf(int fd, struct abc_pcie_sg_entry **sg,
				      struct abc_pcie_sg_list *sgl);

/**
 * API to release a scatter-gather list for a dma_buf
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		abc_pcie_sg_retrieve_from_dma_buf
 * @return 0 for SUCCESS
 */
int abc_pcie_sg_release_from_dma_buf(struct abc_pcie_sg_list *sgl);

/**
 * API to get the current status of a specified channel
 * @param[in] chan The channel number
 * @param[in] dir The direction of channel.
 * @param[out] info chan information includes channel status, transferred
 *				 size, error status, and etc.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int abc_pcie_dma_get_status(uint8_t chan, enum dma_data_direction dir,
			    struct abc_pcie_dma_state_info *info);

/**
 * WIP: API to abort DMA transfer on specific channel
 * @param[in] chan	The channel number for DMA transfer abort
 * @param[in] dir	Direction of DMA channel, READ or WRITE
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int abc_pcie_dma_abort(uint8_t chan, enum dma_data_direction dir);

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
/**
 * Create the linked list for the provided source and destination
 * scatterlists. The linked list lives in contiguous memory.
 * @param[in] src_sg Scatterlist of the source buffer
 * @param[in] dst_sg Scatterlist of the destination buffer
 * @param[out] ll Linked list (allocated in this function)
 * @return 0 on success
 */
int abc_pcie_ll_build(struct abc_pcie_sg_entry *src_sg,
		      struct abc_pcie_sg_entry *dst_sg,
		      struct abc_pcie_dma_ll *ll);

/**
 * Free memory allocation for linked list
 * @param[in] ll linked list object
 * @return 0
 */
int abc_pcie_ll_destroy(struct abc_pcie_dma_ll *ll);


#endif /* __ABC_PCIE_DMA_H */

/*
 * Android Airbrush coprocessor DMA library
 *
 * Copyright 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ab-dram.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/errno.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/mfd/abc-pcie.h>
#include <uapi/abc-pcie-dma.h>
#include "abc-pcie-dma.h"
#include "abc-pcie-private.h"

#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))
/* TODO(isca): b/120289070 */
#define DMA_CHAN	1

/* TODO(alexperez): Remove this, when we get async mods. */
static struct abc_pcie_dma abc_dma;

static DEFINE_MUTEX(dma_mutex);
static DECLARE_COMPLETION(dma_done);
static enum abc_dma_trans_status dma_status;

/* TODO: extend to include multiple concurrent dma channels */
static int dma_callback(uint8_t chan, enum dma_data_direction dir,
			enum abc_dma_trans_status status)
{
	dev_dbg(&abc_dma.pdev->dev,
		"%s: DMA callback DIR(%d), status(%d)\n",
		__func__, (int)dir, (int)chan);
	dma_status = status;
	complete(&dma_done);
	return 0;
}

struct abc_pcie_dma_mblk_desc {
	size_t size;
	dma_addr_t dma_paddr;
	struct dma_buf *ab_dram_dma_buf;
	struct bar_mapping mapping;
};

static int abc_pcie_dma_alloc_ab_dram(size_t size,
		struct abc_pcie_dma_mblk_desc *mblk_xfer_desc)
{
	struct dma_buf *ab_dram_dmabuf;

	ab_dram_dmabuf = ab_dram_alloc_dma_buf_kernel(size);
	if (IS_ERR(ab_dram_dmabuf))
		return PTR_ERR(ab_dram_dmabuf);

	memset(mblk_xfer_desc, 0, sizeof(*mblk_xfer_desc));
	mblk_xfer_desc->ab_dram_dma_buf = ab_dram_dmabuf;

	mblk_xfer_desc->size = size;
	mblk_xfer_desc->dma_paddr = ab_dram_get_dma_buf_paddr(ab_dram_dmabuf);

	return 0;
}

/**
 * Convert Linux scatterlist to array of entries used by PCIe EP DMA engine
 * @param[in]  sc_list   Scatter gather list for DMA buffer
 * @param[in]  count     Number of entries in scatterlist
 * @param[in]  off       Offset within scatterlist to begin building entries.
 * @param[in]  size      Size, in bytes, of transfer.
 * @param[out] sg        Array generated dma addresses and length.
 * @param[in]  maxsg     Allocated max array number of the sg
 * @return a count of sg entries used on success
 *         -EINVAL if exceeding maxsg
 */
static int scatterlist_to_abc_sg(struct scatterlist *sc_list, int count,
				 uint64_t off, uint64_t size,
				 struct abc_pcie_sg_entry *sg, size_t maxsg)
{
	struct scatterlist *in_sg;
	int i, u;
	uint64_t sc_size = 0;
	uint64_t off_rem = off;		/* remaining offset */
	uint64_t size_rem = size;	/* remaining size */

	i = 0;	/* iterator of *sc_list */
	u = 0;	/* iterator of *sg */

	for (i = 0, in_sg = sc_list; i < count; i++, in_sg = sg_next(in_sg)) {
		if (off_rem < sg_dma_len(in_sg))
			break;

		off_rem -= sg_dma_len(in_sg);
		sc_size += sg_dma_len(in_sg);
	}

	for (; size_rem && (i < count); i++, in_sg = sg_next(in_sg)) {
		/* Last entry is reserved for the NULL terminator */
		if (u >= (maxsg - 1)) {
			dev_err(&abc_dma.pdev->dev,
				"maxsg exceeded\n");
			return -EINVAL;
		}
		sg[u].paddr = sg_dma_address(in_sg) + off_rem;
		sg[u].size = min(sg_dma_len(in_sg) - off_rem, size_rem);
		dev_dbg(&abc_dma.pdev->dev,
			"sg[%d] : Address %pa , length %zu\n",
			u, &sg[u].paddr, sg[u].size);
		off_rem = 0;
		size_rem -= sg[u].size;
		sc_size += sg_dma_len(in_sg);
		if ((u > 0) && (sg[u-1].paddr + sg[u-1].size ==
			sg[u].paddr)) {
			sg[u-1].size = sg[u-1].size
				+ sg[u].size;
			sg[u].size = 0;
		} else {
			u++;
		}
	}

	if (size_rem) {
		dev_err(&abc_dma.pdev->dev,
			"%s: transfer oob: off %lu, size %lu, sc list size %lu\n",
			__func__, off, size, sc_size);
		return -EINVAL;
	}

	/*
	 * Zero out the list terminator entry.
	 * abc dma engine looks at sg.paddr=0 for end of chain
	 */
	memset(&sg[u], 0, sizeof(sg[0]));
	u++; /* Count of entries includes list terminator entry */

	dev_dbg(&abc_dma.pdev->dev,
		"SGL with %d/%d entries\n", u, i);

	return u;
}

/**
 * Import dma_buf (from ION buffer)
 * @param[in]  fd       Handle of dma_buf passed from user
 * @param[out] sgl      pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries
 * @return 0            on SUCCESS, negative on failure
 */
static int abc_pcie_sg_import_dma_buf(int fd, struct abc_pcie_sg_list *sgl)
{
	int ret;

	sgl->dma_buf = dma_buf_get(fd);
	if (IS_ERR(sgl->dma_buf)) {
		ret = PTR_ERR(sgl->dma_buf);
		dev_err(&abc_dma.pdev->dev,
				"%s: failed to get dma_buf, err %d\n",
				__func__, ret);
		return ret;
	}

	sgl->attach = dma_buf_attach(sgl->dma_buf,
				     abc_dma.dma_dev);
	if (IS_ERR(sgl->attach)) {
		ret = PTR_ERR(sgl->attach);
		dev_err(&abc_dma.pdev->dev,
				"%s: failed to attach dma_buf, err %d\n",
				__func__, ret);
		goto err_put;
	}

	sgl->sg_table = dma_buf_map_attachment(sgl->attach, sgl->dir);
	if (IS_ERR(sgl->sg_table)) {
		ret = PTR_ERR(sgl->sg_table);
		dev_err(&abc_dma.pdev->dev,
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
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *                 allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries.
 * @return 0        on SUCCESS
 *         negative on failure
 */
int abc_pcie_sg_retrieve_from_dma_buf(int fd, uint64_t off, uint64_t size,
				      struct abc_pcie_sg_entry **sg,
				      struct abc_pcie_sg_list *sgl)
{
	int ret;
	size_t maxsg;

	/* Retrieve sg_table from dma_buf framework */
	ret = abc_pcie_sg_import_dma_buf(fd, sgl);
	if (ret)
		return ret;

	/* Use sg_table->sgl as our sc_list */
	sgl->sc_list = sgl->sg_table->sgl;
	sgl->n_num = sgl->sg_table->nents;

	/*
	 * Allocate enough for one entry per sc_list entry, plus end of list.
	 */
	maxsg = sgl->n_num + 1;
	*sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!(*sg)) {
		abc_pcie_sg_release_from_dma_buf(sgl, NULL);
		return -ENOMEM;
	}

	dev_dbg(&abc_dma.pdev->dev,
		"Enter %s: n_num:%d maxsg:%zu\n", __func__, sgl->n_num, maxsg);

	/* Convert sc_list to a Synopsys compatible linked-list */
	sgl->length = scatterlist_to_abc_sg(sgl->sc_list, sgl->n_num, off, size,
					    *sg, maxsg);
	if (sgl->length < 0) {
		ret = sgl->length;
		abc_pcie_sg_release_from_dma_buf(sgl, *sg);
		*sg = NULL;
		return ret;
	}

	return 0;
}

int abc_pcie_sg_release_from_dma_buf(struct abc_pcie_sg_list *sgl,
					struct abc_pcie_sg_entry *sg)
{
	dma_buf_unmap_attachment(sgl->attach, sgl->sg_table, sgl->dir);
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	sgl->length = 0;
	dma_buf_detach(sgl->dma_buf, sgl->attach);
	dma_buf_put(sgl->dma_buf);
	if (sg)
		vfree(sg);

	return 0;
}

/**
 * Local function to retrieve pages from memory allocated by vmalloc
 * It increments page ref count by 1
 * TODO(b/114422444): AB DMA driver Clean-up & Improvements
 * @param[in] start Starting address of vmalloc memory
 * @param[in] nr_pages Number of pages
 * @param[in] write Not used
 * @param[out] pages Pointers to pages that retrived from vmalloc_to_page call
 * @return Number of pages retrived
 */
static int get_vmalloc_pages(void *addr, int nr_pages, int write,
		      struct page **pages)
{
	int i;

	for (i = 0; i < nr_pages; i++) {
		*(pages + i) = vmalloc_to_page(addr);
		get_page(*(pages + i));
		addr += PAGE_SIZE;
	}
	return nr_pages;
}

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a buffer
 * allocated from vmalloc
 * TODO(b/114422444): AB DMA driver Clean-up & Improvements
 * @param[in] dmadest  Starting virtual addr of the DMA destination
 * @param[in] size Total size of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *             allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *             page list, scatter gather list and num of its entries.
 * @return The number of sg[] entries filled out by the routine, negative if
 *             overflow or sg[] not allocated.
 */
static int abc_pcie_vmalloc_buf_sg_build(void *dmadest, size_t size,
				     struct abc_pcie_sg_entry **sg,
				     struct abc_pcie_sg_list *sgl)
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
	*sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!(*sg))
		return -ENOMEM;

	/* page num calculation */
	first_page = ((dma_addr_t) dmadest & PAGE_MASK) >> PAGE_SHIFT;
	last_page = (((dma_addr_t) dmadest + size - 1) & PAGE_MASK)
		>> PAGE_SHIFT;
	fp_offset = (dma_addr_t) dmadest & ~PAGE_MASK;
	p_num = last_page - first_page + 1;

	dev_dbg(&abc_dma.pdev->dev,
		"%s:p_num:%d, maxsg:%zu\n",
		__func__, p_num, maxsg);

	sgl->mypage = kcalloc(p_num, sizeof(struct page *), GFP_KERNEL);
	if (!sgl->mypage)
		goto free_sg;
	sgl->sc_list = kcalloc(p_num, sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list)
		goto free_sg;

	n_num = get_vmalloc_pages(dmadest, p_num, 0, sgl->mypage);

	if (n_num < p_num) {
		dev_err(&abc_dma.pdev->dev,
			"%s: fail to get user_pages\n", __func__);
		goto release_page;
	}
	if (n_num < maxsg) {
		sg_init_table(sgl->sc_list, n_num);
		if (n_num == 1) {
			sg_set_page(sgl->sc_list, *(sgl->mypage),
				size, fp_offset);
		} else {
			sg_set_page(sgl->sc_list, *(sgl->mypage),
				PAGE_SIZE - fp_offset, fp_offset);
			for (i = 1; i < n_num - 1; i++) {
				sg_set_page(sgl->sc_list + i,
					    *(sgl->mypage + i), PAGE_SIZE, 0);
			}
			sg_set_page(sgl->sc_list + n_num - 1,
				    *(sgl->mypage + n_num - 1),
				    size - (PAGE_SIZE - fp_offset)
				    - ((n_num - 2) * PAGE_SIZE), 0);
		}
		count = dma_map_sg(abc_dma.dma_dev,
				   sgl->sc_list, n_num, sgl->dir);
		if (count <= 0) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to map scatterlist region (%d) n_num(%d)\n",
			       __func__, count, n_num);
			goto release_page;
		}

		u = scatterlist_to_abc_sg(sgl->sc_list, count, /*off=*/0, size,
					  *sg, maxsg);
		if (u < 0)
			goto unmap_sg;
	} else {
		dev_err(&abc_dma.pdev->dev,
			"%s: maxsg exceeded\n", __func__);
		goto release_page;
	}
	sgl->n_num = n_num;
	sgl->length = u;

	return 0;

unmap_sg:
	dma_unmap_sg(abc_dma.dma_dev, sgl->sc_list, n_num, sgl->dir);
release_page:
	for (i = 0; i < n_num; i++)
		put_page(*(sgl->mypage + i));
free_sg:
	vfree((*sg));
	*sg = NULL;
	kfree(sgl->mypage);
	sgl->mypage = NULL;
	kfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	sgl->length = 0;

	return -EINVAL;
}

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * local buffer
 * @param[in] dmadest  Starting virtual addr of the DMA destination
 * @param[in] size Total size of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *             allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			page list, scatter gather list and num of its entries.
 * @return The number of sg[] entries filled out by the routine, negative if
 *		   overflow or sg[] not allocated.
 */
int abc_pcie_user_local_buf_sg_build(void *dmadest, size_t size,
				     struct abc_pcie_sg_entry **sg,
				     struct abc_pcie_sg_list *sgl)
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
	*sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!(*sg))
		return -ENOMEM;

	/* page num calculation */
	first_page = ((unsigned long) dmadest & PAGE_MASK) >> PAGE_SHIFT;
	last_page = (((unsigned long) dmadest + size - 1) & PAGE_MASK)
		>> PAGE_SHIFT;
	fp_offset = (unsigned long) dmadest & ~PAGE_MASK;
	p_num = last_page - first_page + 1;

	dev_dbg(&abc_dma.pdev->dev,
		"%s:p_num:%d, maxsg:%zu\n",
		__func__, p_num, maxsg);

	sgl->mypage = kcalloc(p_num, sizeof(struct page *), GFP_KERNEL);
	if (!sgl->mypage) {
		vfree((*sg));
		*sg = NULL;
		sgl->n_num = 0;
		sgl->length = 0;
		return -EINVAL;
	}
	sgl->sc_list = kcalloc(p_num, sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list) {
		vfree((*sg));
		*sg = NULL;
		kfree(sgl->mypage);
		sgl->mypage = NULL;
		sgl->n_num = 0;
		sgl->length = 0;
		return -EINVAL;
	}

	down_read(&current->mm->mmap_sem);

	n_num = get_user_pages((unsigned long)dmadest, p_num,
			       FOLL_WRITE | FOLL_FORCE, sgl->mypage, NULL);

	up_read(&current->mm->mmap_sem);
	if (n_num < p_num) {
		dev_err(&abc_dma.pdev->dev,
			"%s: fail to get user_pages\n", __func__);
		goto release_page;
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
				sg_set_page(sgl->sc_list + i,
					    *(sgl->mypage + i), PAGE_SIZE, 0);
			}
			sg_set_page(sgl->sc_list + n_num-1,
				    *(sgl->mypage + n_num-1),
				    size - (PAGE_SIZE - fp_offset)
				    - ((n_num-2)*PAGE_SIZE), 0);
		}
		count = dma_map_sg(abc_dma.dma_dev,
				   sgl->sc_list, n_num, sgl->dir);
		if (count <= 0) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to map scatterlist region (%d) n_num(%d)\n",
			       __func__, count, n_num);
			goto release_page;
		}

		u = scatterlist_to_abc_sg(sgl->sc_list, count, /*off=*/0, size,
					  *sg, maxsg);
		if (u < 0)
			goto unmap_sg;
	} else {
		dev_err(&abc_dma.pdev->dev,
			"%s: maxsg exceeded\n", __func__);
		goto release_page;
	}
	sgl->n_num = n_num;
	sgl->length = u;

	return 0;

unmap_sg:
	dma_unmap_sg(abc_dma.dma_dev, sgl->sc_list, n_num, sgl->dir);
release_page:
	for (i = 0; i < n_num; i++)
		put_page(*(sgl->mypage + i));

	vfree((*sg));
	*sg = NULL;
	kfree(sgl->mypage);
	sgl->mypage = NULL;
	kfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	sgl->length = 0;

	return -EINVAL;
}

/**
 * API to release scatter gather list for a user local buffer
 * @param[in] **sg pointer to pointer to sg entry allocated during
 *		abc_pcie_user_local_buf_sg_build
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		abc_pcie_user_local_buf_sg_build
 * @return 0 for SUCCESS
 */
int abc_pcie_user_local_buf_sg_destroy(struct abc_pcie_sg_entry **sg,
				       struct abc_pcie_sg_list *sgl)
{
	int i;
	struct page *page;

	dma_unmap_sg(abc_dma.dma_dev, sgl->sc_list,
			sgl->n_num, sgl->dir);
	for (i = 0; i < sgl->n_num; i++) {
		page = *(sgl->mypage + i);
		/* Mark page as dirty before releasing the pages. */
		if (!PageReserved(page))
			SetPageDirty(page);
		put_page(page);
	}
	vfree((*sg));
	*sg = NULL;
	kfree(sgl->mypage);
	sgl->mypage = NULL;
	kfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;

	return 0;
}

int abc_pcie_ll_count_dma_element(struct abc_pcie_sg_entry *src_sg,
			    struct abc_pcie_sg_entry *dst_sg)
{
	int entries = 0;
	int dst_size_left = dst_sg->size;
	int src_size_left = src_sg->size;

	while ((src_sg->paddr != 0x0) && (dst_sg->paddr != 0x0)) {
		if (dst_size_left == src_size_left) {
			src_sg++;
			dst_sg++;
			src_size_left = src_sg->size;
			dst_size_left = dst_sg->size;
		} else if (src_size_left > dst_size_left) {
			src_size_left -= dst_size_left;
			dst_sg++;
			dst_size_left = dst_sg->size;
		} else {
			dst_size_left -= src_size_left;
			src_sg++;
			src_size_left = src_sg->size;

		}
		entries++;
	}
	return entries;
}

static void abc_pcie_dma_update_ll_element(struct abc_pcie_sg_entry *sg_src,
	struct abc_pcie_sg_entry *sg_dst, int size,
	struct abc_pcie_dma_ll_element *ll_element)
{
	ll_element->size = size;
	ll_element->sar_low = LOWER(sg_src->paddr);
	ll_element->sar_high = UPPER(sg_src->paddr);
	ll_element->dar_low = LOWER(sg_dst->paddr);
	ll_element->dar_high = UPPER(sg_dst->paddr);
}

/**
 * API to generate a linked list for DMA transfers from
 * source and destination scatterlists.
 * @src_sg[in] Scatterlist of the source buffer
 * @dst_sg[in] Scatterlist of the destination buffer
 * @mblk_xfer_desc[in] pre allocated bar buffer
 * @ll_num_entries [in] - the number of entries in the link list
 */
static int abc_pcie_dma_ll_setup(struct abc_pcie_sg_entry *src_sg,
			    struct abc_pcie_sg_entry *dst_sg,
			    struct abc_pcie_dma_mblk_desc *mblk_xfer_desc,
			    int ll_num_entries)
{
	struct abc_pcie_dma_ll_element ll_element;
	struct abc_pcie_sg_entry sg_dst, sg_src;
	int i, s;
	void *base_addr;

	base_addr = mblk_xfer_desc->mapping.bar_vaddr;
	ll_element.header = LL_DATA_ELEMENT;
	i = 0; /* source buffer scatterlist index */
	s = 0; /* destination buffer scatterlist index */
	sg_dst = dst_sg[0];
	sg_src = src_sg[0];
	/* last element has a unique header */
	for (; ll_num_entries > 1 &&
			(sg_src.paddr != 0x0) &&
			(sg_dst.paddr != 0x0); --ll_num_entries) {
		if (sg_src.size == sg_dst.size) {
			abc_pcie_dma_update_ll_element(&sg_src,
				&sg_dst, sg_src.size, &ll_element);
			i++;
			s++;
			sg_src = src_sg[i];
			sg_dst = dst_sg[s];
		} else if (sg_src.size > sg_dst.size) {
			abc_pcie_dma_update_ll_element(&sg_src,
				&sg_dst, sg_dst.size, &ll_element);
			sg_src.paddr = sg_src.paddr + sg_dst.size;
			sg_src.size = sg_src.size - sg_dst.size;
			s++;
			sg_dst = dst_sg[s];
		} else {
			abc_pcie_dma_update_ll_element(&sg_src,
				&sg_dst, sg_src.size, &ll_element);
			sg_dst.paddr = sg_dst.paddr + sg_src.size;
			sg_dst.size = sg_dst.size - sg_src.size;
			i++;
			sg_src = src_sg[i];
		}

		if (ll_num_entries == 2)
			ll_element.header = LL_IRQ_DATA_ELEMENT;

		memcpy_toio((void *)base_addr, &ll_element, sizeof(ll_element));
		base_addr += sizeof(ll_element);
	}

	// we didn't reach the end of the linked list - error in the count
	if ((sg_src.paddr != 0x0 && sg_dst.paddr != 0) || ll_num_entries > 1) {
		dev_err(&abc_dma.pdev->dev,
				"%s: did not reach the end of the Scatterlists\n",
				__func__);
		return -EINVAL;
	}
	/* last element */
	ll_element.header = LL_LAST_LINK_ELEMENT;
	ll_element.sar_low = LOWER(mblk_xfer_desc->dma_paddr);
	ll_element.sar_high = UPPER(mblk_xfer_desc->dma_paddr);
	memcpy_toio((void *)base_addr, &ll_element, sizeof(ll_element));
	return 0;
}

/* Build abc_pcie_sg_entry structure for the remote buffer
 * Since buffers on ABC are contiguous the generated scatterlist
 * only has two entries, one with the starting address and one with
 * the terminator entry
 * TODO(alexperez):  Remove support for specifying an arbitrary
 * physical address once bringup is done.
 */
int abc_pcie_user_remote_buf_sg_build(__u64 remote_buf, size_t size,
				      struct abc_pcie_sg_entry **sg,
				      struct abc_pcie_sg_list *sgl)
{
	size_t maxsg;
	struct abc_pcie_sg_entry *local_sg;
	/*
	 * Allocate enough for one entry per sc_list entry, plus end of list.
	 */
	maxsg = 1 + 1;
	local_sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!sg) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to allocate remote buffer\n", __func__);
		return -ENOMEM;
	}

	local_sg[0].paddr = (phys_addr_t)remote_buf;
	local_sg[0].size = size;

	/* List terminator */
	local_sg[1].paddr = 0;
	local_sg[1].size = 0;

	*sg = local_sg;

	sgl->length = 2;
	return 0;
}

/**
 * API to release scatter gather list for a user remote buffer
 * @param[in] **sg pointer to pointer to sg entry allocated during
 *		abc_pcie_user_remote_buf_sg_build
 * @return 0 for SUCCESS
 */
int abc_pcie_user_remote_buf_sg_destroy(struct abc_pcie_sg_entry **sg)
{
	vfree((*sg));
	*sg = NULL;

	return 0;
}

/**
 * Top-level API for DMA transfers for kernel space memory
 * TODO(b/114422444): AB DMA driver Clean-up & Improvements
 * Generate scatterlist, decide on transfer mode (sblk/mblk)
 * @dma_desc[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_issue_dma_xfer_vmalloc(struct abc_pcie_dma_desc *dma_desc)
{
	struct dma_element_t dma_blk;
	struct abc_pcie_sg_list *local_sg_list;
	struct abc_pcie_sg_list *remote_sg_list;
	struct abc_pcie_sg_entry *local_sg_entries;
	struct abc_pcie_sg_entry *remote_sg_entries;
	uint32_t local_sg_size;
	uint32_t remote_sg_size;
	uint8_t dma_chan;
	int err = 0;

	if (dma_desc->size == 0) {
		dev_err(&abc_dma.pdev->dev, "%s: Invalid transfer size: 0\n",
				__func__);
		return -EINVAL;
	}

	dev_dbg(&abc_dma.pdev->dev,
		"%s: local_buf_type=%d, local_buf=%pK, size=%u\n",
		__func__, dma_desc->local_buf_type, dma_desc->local_buf,
		dma_desc->size);

	/* consider validateing local buffer */

	/* Create scatterlist of local buffer */
	local_sg_list = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!local_sg_list)
		return -ENOMEM;

	local_sg_list->dir = dma_desc->dir;
	local_sg_entries = NULL;

	err = abc_pcie_vmalloc_buf_sg_build(dma_desc->local_buf,
						  dma_desc->size,
						  &local_sg_entries,
						  local_sg_list);
	if (err < 0)
		goto release_local_sg_list;

	local_sg_size = local_sg_list->length;

	/* Create scatterlist of remote buffer */
	remote_sg_list = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!remote_sg_list) {
		err = -ENOMEM;
		goto release_local_buf;
	}

	remote_sg_list->dir = dma_desc->dir;
	remote_sg_entries = NULL;

	/* TODO: size will not work for DMA buffers! */
	switch (dma_desc->remote_buf_type) {
	case DMA_BUFFER_USER:
		err = abc_pcie_user_remote_buf_sg_build(
				dma_desc->remote_buf,
				dma_desc->size,
				&remote_sg_entries, remote_sg_list);
		break;
	case DMA_BUFFER_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(
				dma_desc->remote_dma_buf_fd,
				dma_desc->remote_dma_buf_off,
				dma_desc->size,
				&remote_sg_entries,
				remote_sg_list);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
				"%s: Unknown remote DMA buffer type %d\n",
				__func__, dma_desc->remote_buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0)
		goto release_remote_sg_list;

	/* check the size of the scatterlist */
	remote_sg_size = remote_sg_list->length;
	dev_dbg(&abc_dma.pdev->dev,
		"%s: Scatterlists with (%d/%d) local/remote entries\n",
		__func__, local_sg_size, remote_sg_size);

	/* Based on the scatterlists determine if sblk or mblk transfer */
	/* Single block transfer */
	if (remote_sg_size <= 2 && local_sg_size <= 2) {
		dev_dbg(&abc_dma.pdev->dev,
			"%s: SBLK transfer: ->local (%pK/%zu) ->remote (%pa/%zu)\n",
			__func__, &local_sg_entries[0].paddr,
			local_sg_entries[0].size, &remote_sg_entries[0].paddr,
			remote_sg_entries[0].size);

		dma_chan = DMA_CHAN;

		/* Register DMA callback */
		err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);
		if (err < 0)
			goto release_remote_sg_list;

		dma_blk.src_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			LOWER((phys_addr_t)local_sg_entries[0].paddr) :
			LOWER((phys_addr_t)remote_sg_entries[0].paddr);
		dma_blk.src_u_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			UPPER((phys_addr_t)local_sg_entries[0].paddr) :
			UPPER((phys_addr_t)remote_sg_entries[0].paddr);
		dma_blk.dst_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			LOWER((phys_addr_t)remote_sg_entries[0].paddr) :
			LOWER((phys_addr_t)local_sg_entries[0].paddr);
		dma_blk.dst_u_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			UPPER((phys_addr_t)remote_sg_entries[0].paddr) :
			UPPER((phys_addr_t)local_sg_entries[0].paddr);
		dma_blk.len = local_sg_entries[0].size; /* TODO: fix this */

		dev_dbg(&abc_dma.pdev->dev,
			"DMA - SRC(L:0x%pK U:0x%pK) to DST(L:0x%pK U:0x%pK), size(%d)\n",
			dma_blk.src_addr, dma_blk.src_u_addr,
			dma_blk.dst_addr, dma_blk.dst_u_addr,
			dma_blk.len);
		mutex_lock(&dma_mutex);
		reinit_completion(&dma_done);
		dma_sblk_start(dma_chan, dma_desc->dir, &dma_blk);
		err = wait_for_completion_interruptible(&dma_done);
		if (err) {
			dev_dbg(&abc_dma.pdev->dev,
				"%s: DMA aborted (%d)\n",
				__func__, err);
		}
		/* unregister callback */
		abc_reg_dma_irq_callback(NULL, dma_chan);
		mutex_unlock(&dma_mutex);

	/* Multi block transfer */
	} else {
		dev_dbg(&abc_dma.pdev->dev,
			"%s: Using multi block transfer (dir=%s)\n",
			__func__,
			(dma_desc->dir == DMA_TO_DEVICE) ? "AP2EP" : "EP2AP");
		if (dma_desc->dir == DMA_TO_DEVICE)
			err = abc_pcie_setup_mblk_xfer(local_sg_entries,
						       remote_sg_entries,
						       dma_desc);
		else
			err = abc_pcie_setup_mblk_xfer(remote_sg_entries,
						       local_sg_entries,
						       dma_desc);
	}

	switch (dma_desc->remote_buf_type) {
	case DMA_BUFFER_USER:
		abc_pcie_user_remote_buf_sg_destroy(&remote_sg_entries);
		break;
	case DMA_BUFFER_DMA_BUF:
		abc_pcie_sg_release_from_dma_buf(remote_sg_list,
							remote_sg_entries);
		break;
	default:
		break;
	}

release_remote_sg_list:
	kfree(remote_sg_list);

release_local_buf:
	abc_pcie_user_local_buf_sg_destroy(&local_sg_entries, local_sg_list);

release_local_sg_list:
	kfree(local_sg_list);

	return err;
}

/**
 * Top-level API for DMA transfers
 * Generate scatterlist, decide on transfer mode (sblk/mblk)
 * @dma_desc[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_issue_dma_xfer(struct abc_pcie_dma_desc *dma_desc)
{
	struct dma_element_t dma_blk;
	struct abc_pcie_sg_list *local_sg_list;
	struct abc_pcie_sg_list *remote_sg_list;
	struct abc_pcie_sg_entry *local_sg_entries;
	struct abc_pcie_sg_entry *remote_sg_entries;
	uint32_t local_sg_size;
	uint32_t remote_sg_size;
	uint8_t dma_chan;
	int err = 0;

	if (dma_desc->size == 0) {
		dev_err(&abc_dma.pdev->dev, "%s: Invalid transfer size: 0\n",
				__func__);
		return -EINVAL;
	}

	dev_dbg(&abc_dma.pdev->dev,
		"%s: local_buf_type=%d, local_buf=%pK, size=%u\n",
		__func__, dma_desc->local_buf_type, dma_desc->local_buf,
		dma_desc->size);

	/* consider validateing local buffer */

	/* Create scatterlist of local buffer */
	local_sg_list = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!local_sg_list)
		return -ENOMEM;

	local_sg_list->dir = dma_desc->dir;
	local_sg_entries = NULL;

	switch (dma_desc->local_buf_type) {
	case DMA_BUFFER_USER:
		err = abc_pcie_user_local_buf_sg_build(dma_desc->local_buf,
						       dma_desc->size,
						       &local_sg_entries,
						       local_sg_list);
		break;
	case DMA_BUFFER_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(
			dma_desc->local_dma_buf_fd,
			dma_desc->local_dma_buf_off,
			dma_desc->size,
			&local_sg_entries,
			local_sg_list);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Unknown local DMA buffer type %d\n",
		       __func__, dma_desc->local_buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0)
		goto release_local_sg_list;

	local_sg_size = local_sg_list->length;

	/* Create scatterlist of remote buffer */
	remote_sg_list = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!remote_sg_list) {
		err = -ENOMEM;
		goto release_local_buf;
	}

	remote_sg_list->dir = dma_desc->dir;
	remote_sg_entries = NULL;

	/* TODO: size will not work for DMA buffers! */
	switch (dma_desc->remote_buf_type) {
	case DMA_BUFFER_USER:
		err = abc_pcie_user_remote_buf_sg_build(
				dma_desc->remote_buf,
				dma_desc->size,
				&remote_sg_entries, remote_sg_list);
		break;
	case DMA_BUFFER_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(
				dma_desc->remote_dma_buf_fd,
				dma_desc->remote_dma_buf_off,
				dma_desc->size,
				&remote_sg_entries,
				remote_sg_list);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
				"%s: Unknown remote DMA buffer type %d\n",
				__func__, dma_desc->remote_buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0)
		goto release_remote_sg_list;

	/* check the size of the scatterlist */
	remote_sg_size = remote_sg_list->length;
	dev_dbg(&abc_dma.pdev->dev,
		"%s: Scatterlists with (%d/%d) local/remote entries\n",
		__func__, local_sg_size, remote_sg_size);

	/* Based on the scatterlists determine if sblk or mblk transfer */
	/* Single block transfer */
	if (remote_sg_size <= 2 && local_sg_size <= 2) {
		dev_dbg(&abc_dma.pdev->dev,
			"%s: SBLK transfer: ->local (%pa/%zu) ->remote (%pa/%zu)\n",
			__func__, &local_sg_entries[0].paddr,
			local_sg_entries[0].size, &remote_sg_entries[0].paddr,
			remote_sg_entries[0].size);

		dma_chan = DMA_CHAN;

		/* Register DMA callback */
		err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);
		if (err < 0)
			goto release_remote_sg_list;

		dma_blk.src_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			LOWER((uint64_t)local_sg_entries[0].paddr) :
			LOWER((uint64_t)remote_sg_entries[0].paddr);
		dma_blk.src_u_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			UPPER((uint64_t)local_sg_entries[0].paddr) :
			UPPER((uint64_t)remote_sg_entries[0].paddr);
		dma_blk.dst_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			LOWER((uint64_t)remote_sg_entries[0].paddr) :
			LOWER((uint64_t)local_sg_entries[0].paddr);
		dma_blk.dst_u_addr = (dma_desc->dir == DMA_TO_DEVICE) ?
			UPPER((uint64_t)remote_sg_entries[0].paddr) :
			UPPER((uint64_t)local_sg_entries[0].paddr);
		dma_blk.len = local_sg_entries[0].size; /* TODO: fix this */

		dev_dbg(&abc_dma.pdev->dev,
			"DMA - SRC(L:0x%x U:0x%x) to DST(L:0x%x U:0x%x), size(%d)\n",
			dma_blk.src_addr, dma_blk.src_u_addr,
			dma_blk.dst_addr, dma_blk.dst_u_addr,
			dma_blk.len);
		mutex_lock(&dma_mutex);
		reinit_completion(&dma_done);
		dma_sblk_start(dma_chan, dma_desc->dir, &dma_blk);
		err = wait_for_completion_interruptible(&dma_done);
		if (err) {
			dev_dbg(&abc_dma.pdev->dev,
				"%s: DMA aborted (%d)\n",
				__func__, err);
		}
		/* unregister callback */
		abc_reg_dma_irq_callback(NULL, dma_chan);
		mutex_unlock(&dma_mutex);

	/* Multi block transfer */
	} else {
		dev_dbg(&abc_dma.pdev->dev,
			"%s: Using multi block transfer (dir=%s)\n",
			__func__,
			(dma_desc->dir == DMA_TO_DEVICE) ? "AP2EP" : "EP2AP");
		if (dma_desc->dir == DMA_TO_DEVICE)
			err = abc_pcie_setup_mblk_xfer(local_sg_entries,
						       remote_sg_entries,
						       dma_desc);
		else
			err = abc_pcie_setup_mblk_xfer(remote_sg_entries,
						       local_sg_entries,
						       dma_desc);
	}

	switch (dma_desc->remote_buf_type) {
	case DMA_BUFFER_USER:
		abc_pcie_user_remote_buf_sg_destroy(&remote_sg_entries);
		break;
	case DMA_BUFFER_DMA_BUF:
		abc_pcie_sg_release_from_dma_buf(remote_sg_list,
							remote_sg_entries);
		break;
	default:
		break;
	}

release_remote_sg_list:
	kfree(remote_sg_list);

release_local_buf:
	switch (dma_desc->local_buf_type) {
	case DMA_BUFFER_USER:
		abc_pcie_user_local_buf_sg_destroy(&local_sg_entries,
							 local_sg_list);
		break;
	case DMA_BUFFER_DMA_BUF:
		abc_pcie_sg_release_from_dma_buf(local_sg_list,
							local_sg_entries);
		break;
	default:
		break;
	}

release_local_sg_list:
	kfree(local_sg_list);

	return err;
}
EXPORT_SYMBOL(abc_pcie_issue_dma_xfer);

/**
 * Build linked list for multi-block transfer locally and transfer
 * to endpoint via single block DMA transfer.
 * @src_sg[in] Scatterlist of the source buffer
 * @dst_sg[in] Scatterlist of the destination buffer
 * @dma_desc[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_setup_mblk_xfer(struct abc_pcie_sg_entry *src_sg,
			     struct abc_pcie_sg_entry *dst_sg,
			     struct abc_pcie_dma_desc *dma_desc)
/*			    enum dma_data_direction dir)*/
{
	int dma_chan;
	size_t ll_size;
	int ll_num_entries;
	struct abc_pcie_dma_mblk_desc mblk_xfer_desc;
	int err = 0;
	enum dma_data_direction dir = dma_desc->dir;

	dma_chan = DMA_CHAN;

	/* add one since we have an end of list marker in the end */
	ll_num_entries = abc_pcie_ll_count_dma_element(src_sg, dst_sg) + 1;
	if (ll_num_entries == 1) {
		dev_err(&abc_dma.pdev->dev,
			"%s: Error with received scatter lists\n",
			__func__);
		return -ENOMEM;
	}
	ll_size = ll_num_entries * sizeof(struct abc_pcie_dma_ll_element);

	err = abc_pcie_dma_alloc_ab_dram(ll_size, &mblk_xfer_desc);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to alloc ch%d BAR size: %d error: %d\n",
			__func__, dma_chan, ll_size, err);
		return err;
	}

	mutex_lock(&dma_mutex);

	err = abc_pcie_map_bar_region(abc_dma.dma_dev,
			&abc_dma.pdev->dev /* owner */, BAR_2,
			mblk_xfer_desc.size, mblk_xfer_desc.dma_paddr,
			&mblk_xfer_desc.mapping);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to map to BAR ch%d error: %d\n",
			__func__, dma_chan, err);
		goto unlock_unmap_buf;
	}

	err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to register ch%d dma irq callback: %d\n",
			__func__, dma_chan, err);
		goto unmap_bar_buf;
	}

	err = abc_pcie_dma_ll_setup(src_sg,
			    dst_sg, &mblk_xfer_desc, ll_num_entries);
	if (err)
		goto unregister_dma_callback;
	reinit_completion(&dma_done);

	err = dma_mblk_start(dma_chan, dir, mblk_xfer_desc.dma_paddr);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: start ch%d mblk dma xfer failed: %d\b",
			__func__, dma_chan, err);
		goto unregister_dma_callback;
	}

	err = wait_for_completion_interruptible(&dma_done);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: Wait for ch%d LL upload failed: %d\b",
			__func__, dma_chan, err);
		goto unregister_dma_callback;
	}

	dev_dbg(&abc_dma.pdev->dev,
		"%s: ch%d mblk transfer complete\n",
		__func__, dma_chan);
	err = 0;

unregister_dma_callback:
	abc_reg_dma_irq_callback(NULL, dma_chan);
unmap_bar_buf:
	abc_pcie_unmap_bar_region(abc_dma.dma_dev,
			&abc_dma.pdev->dev /* owner */,
			&mblk_xfer_desc.mapping);
unlock_unmap_buf:
	mutex_unlock(&dma_mutex);
	ab_dram_free_dma_buf_kernel(mblk_xfer_desc.ab_dram_dma_buf);

	return err;
}

int abc_pcie_dma_drv_probe(struct platform_device *pdev)
{
	int err = 0;

	abc_dma.pdev = pdev;

	/* Depending on the IOMMU configuration the board the IPU may need
	 * to use MFD parent's device for mapping DMA buffers.  Otherwise, the
	 * dma device can be set to our own device and use SWIOTLB to map
	 * buffers.
	 */
	if (iommu_present(pdev->dev.parent->bus)) {
		abc_dma.dma_dev = pdev->dev.parent;
	} else {
		/* MFD doesn't seem to configure DMA ops for client devices */
		arch_setup_dma_ops(&pdev->dev, 0, 0, NULL,
				false /* coherent */);
		abc_dma.dma_dev = &pdev->dev;
	}
	abc_dma.uapi.mdev.parent = &pdev->dev;
	err = init_abc_pcie_dma_uapi(&abc_dma.uapi);

	return err;
}

int abc_pcie_dma_drv_remove(struct platform_device *pdev)
{
	remove_abc_pcie_dma_uapi(&abc_dma.uapi);
	return 0;
}

static const struct platform_device_id abc_pcie_dma_ids[] = {
	{
		.name = DRV_NAME_ABC_PCIE_DMA,
	}, {
		/* check abc-pcie-fsys */
	}
};
MODULE_DEVICE_TABLE(platform, abc_pcie_dma_ids);

static struct platform_driver abc_pcie_dma_driver = {
	.probe    = abc_pcie_dma_drv_probe,
	.remove   = abc_pcie_dma_drv_remove,
	.id_table = abc_pcie_dma_ids,
	.driver   = {
		.name = DRV_NAME_ABC_PCIE_DMA,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
};
module_platform_driver(abc_pcie_dma_driver);

MODULE_AUTHOR("Google Inc.");
MODULE_DESCRIPTION("ABC PCIe endpoint DMA host side driver");
MODULE_LICENSE("GPL");

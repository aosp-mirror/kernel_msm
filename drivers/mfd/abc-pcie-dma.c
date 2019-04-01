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
#include <linux/mfd/abc-pcie-dma.h>
#include <uapi/linux/abc-pcie-dma.h>
#include "abc-pcie-private.h"

#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))
/* TODO(isca): b/120289070 */
#define DMA_CHAN	1

/* TODO(alexperez): Remove this, when we get async mods. */
static struct abc_pcie_dma abc_dma;

static DEFINE_MUTEX(dma_mutex);
static DEFINE_SPINLOCK(dma_spinlock);

/* pending_[\w\_]+_q: List of all pending transactions (all sessions).
 * First element in the pending_q is always the currently executing
 * transfer.
 */
static struct list_head pending_to_dev_q;
static struct list_head pending_from_dev_q;

/* Global session used for "sessionless" kernel calls */
static struct abc_pcie_dma_session global_session;

/* Global cache for waiter objects */
static struct kmem_cache *waiter_cache;

static void abc_pcie_exec_dma_xfer(struct abc_dma_xfer *xfer);
int abc_pcie_sg_release_from_dma_buf(struct abc_pcie_sg_list *sgl,
					struct abc_pcie_sg_entry *sg);

/* TODO: extend to include multiple concurrent dma channels */
static int dma_callback(uint8_t chan, enum dma_data_direction dir,
			enum abc_dma_trans_status status)
{
	struct abc_dma_xfer *xfer;
	struct abc_dma_wait_info *wait_info;
	struct list_head *pending_q;

	dev_dbg(&abc_dma.pdev->dev,
		"%s: DMA callback DIR(%d), status(%d)\n",
		__func__, (int)dir, (int)chan);

	spin_lock(&dma_spinlock);
	pending_q = (dir == DMA_TO_DEVICE) ? &pending_to_dev_q :
						&pending_from_dev_q;
	/* Update state on finished transfer, dequeue from pending list */
	xfer = list_first_entry_or_null(pending_q, struct abc_dma_xfer,
					list_pending);
	if (WARN_ON(!xfer)) {
		dev_err(&abc_dma.pdev->dev, "%s: Got interrupt but no xfer!\n",
			__func__);
		spin_unlock(&dma_spinlock);
		return 0;
	}

	wait_info = xfer->wait_info;
	wait_info->error = (status == DMA_ABORT) ? -EIO : 0;

	complete_all(&wait_info->done);

	xfer->pending = false;
	list_del(&xfer->list_pending);

	/* Look for the next transfer to execute */
	xfer = list_first_entry_or_null(pending_q, struct abc_dma_xfer,
					list_pending);

	/* Note that the assumption here is abc_pcie_exec_dma_xfer never returns
	 * error. The validation checks done by pcie_exec_dma_xfer and its
	 * child functions are already done by the create function. If this
	 * assumption is no longer true then we must iterate thru the pending
	 * queue until empty or we have a good transaction to execute or risk
	 * not clearing the transactions in the pending queue.
	 */
	if (xfer)
		abc_pcie_exec_dma_xfer(xfer);
	spin_unlock(&dma_spinlock);
	return 0;
}

/**
 * abc_find_xfer:
 * Find a xfer based on id in a particular session.
 * IMPORTANT: Any user to this function must take a session lock.
 * TODO(alexperez): Consider using idr for faster lookups and given properties
 *                  of session list.
 */
struct abc_dma_xfer *abc_pcie_dma_find_xfer(
			struct abc_pcie_dma_session *session, uint64_t id)
{
	bool found = false;
	struct abc_dma_xfer *xfer;

	list_for_each_entry(xfer, &session->transfers, list_transfers) {
		if (xfer->id == id) {
			found = true;
			break;
		}
	}

	xfer = found ? xfer : NULL;

	return xfer;
}

static int abc_pcie_dma_alloc_ab_dram(size_t size,
		struct abc_pcie_dma_mblk_desc *mblk_desc)
{
	struct dma_buf *ab_dram_dmabuf;

	ab_dram_dmabuf = ab_dram_alloc_dma_buf_kernel(size);
	if (IS_ERR(ab_dram_dmabuf))
		return PTR_ERR(ab_dram_dmabuf);

	memset(mblk_desc, 0, sizeof(*mblk_desc));
	mblk_desc->ab_dram_dma_buf = ab_dram_dmabuf;

	mblk_desc->size = size;
	mblk_desc->dma_paddr = ab_dram_get_dma_buf_paddr(ab_dram_dmabuf);

	return 0;
}

/**
 * API to build scatterlist for CMA buffers.
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
int abc_pcie_local_cma_build(enum dma_data_direction dir,
				size_t size, struct abc_buf_desc *buf_desc)
{
	size_t maxsg;
	struct abc_pcie_sg_entry *local_sg;
	phys_addr_t local_buf = (phys_addr_t)buf_desc->local_addr;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;
	buf_desc->sge = NULL;

	/*
	 * Allocate enough for one entry per sc_list entry, plus end of list.
	 */
	maxsg = 1 + 1;
	local_sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!local_sg) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to allocate local buffer\n", __func__);
		return -ENOMEM;
	}

	local_sg[0].paddr = local_buf;
	local_sg[0].size = size;

	/* List terminator */
	local_sg[1].paddr = 0;
	local_sg[1].size = 0;

	buf_desc->sge = local_sg;

	sgl->length = 2;
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
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
int abc_pcie_sg_retrieve_from_dma_buf(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	int ret;
	size_t maxsg;
	struct abc_pcie_sg_entry *sg;
	int fd = buf_desc->fd;
	uint64_t off = buf_desc->offset;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;
	buf_desc->sge = NULL;

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
	sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!sg) {
		abc_pcie_sg_release_from_dma_buf(sgl, NULL);
		return -ENOMEM;
	}
	buf_desc->sge = sg;

	dev_dbg(&abc_dma.pdev->dev,
		"Enter %s: n_num:%d maxsg:%zu\n", __func__, sgl->n_num, maxsg);

	/* Convert sc_list to a Synopsys compatible linked-list */
	sgl->length = scatterlist_to_abc_sg(sgl->sc_list, sgl->n_num, off, size,
					    sg, maxsg);
	if (sgl->length < 0) {
		ret = sgl->length;
		abc_pcie_sg_release_from_dma_buf(sgl, sg);
		buf_desc->sge = NULL;
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
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
static int abc_pcie_vmalloc_buf_sg_build(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	int i, u, fp_offset, count;
	int n_num, p_num;
	int first_page, last_page;
	size_t maxsg;
	struct abc_pcie_sg_entry *sg;
	void *dmadest = buf_desc->local_addr;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;
	buf_desc->sge = NULL;

	/*
	 * Allocate enough for one entry per page, perhaps needing 1 more due
	 * to crossing a page boundary, plus end of list.
	 */
	maxsg = (size / PAGE_SIZE) + 3;
	sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!sg)
		return -ENOMEM;
	buf_desc->sge = sg;

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
		count = dma_map_sg_attrs(abc_dma.dma_dev,
				   sgl->sc_list, n_num, sgl->dir,
				   DMA_ATTR_SKIP_CPU_SYNC);
		if (count <= 0) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to map scatterlist region (%d) n_num(%d)\n",
			       __func__, count, n_num);
			goto release_page;
		}

		u = scatterlist_to_abc_sg(sgl->sc_list, count, /*off=*/0, size,
					  sg, maxsg);
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
	dma_unmap_sg_attrs(abc_dma.dma_dev, sgl->sc_list, n_num, sgl->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
release_page:
	for (i = 0; i < n_num; i++)
		put_page(*(sgl->mypage + i));
free_sg:
	vfree(sg);
	buf_desc->sge = NULL;
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
 * @param[in] dir Direction of transfer
 * @param[in] size Total size of the transfer in bytes
 * @param[out] lbuf
 * @return POSIX error or zero if successful.
 */
int abc_pcie_user_local_buf_sg_build(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	int i, u, fp_offset, count;
	int n_num, p_num;
	int first_page, last_page;
	size_t maxsg;
	struct abc_pcie_sg_entry *sg;
	void *dmadest = buf_desc->local_addr;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;
	buf_desc->sge = NULL;

	/*
	 * Allocate enough for one entry per page, perhaps needing 1 more due
	 * to crossing a page boundary, plus end of list.
	 */
	maxsg = (size / PAGE_SIZE) + 3;
	sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!sg)
		return -ENOMEM;
	buf_desc->sge = sg;

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
		vfree(sg);
		buf_desc->sge = NULL;
		sgl->n_num = 0;
		sgl->length = 0;
		return -EINVAL;
	}
	sgl->sc_list = kcalloc(p_num, sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list) {
		vfree(sg);
		buf_desc->sge = NULL;
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
		count = dma_map_sg_attrs(abc_dma.dma_dev,
				   sgl->sc_list, n_num, sgl->dir,
				   DMA_ATTR_SKIP_CPU_SYNC);
		if (count <= 0) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to map scatterlist region (%d) n_num(%d)\n",
			       __func__, count, n_num);
			goto release_page;
		}

		u = scatterlist_to_abc_sg(sgl->sc_list, count, /*off=*/0, size,
					  sg, maxsg);
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
	dma_unmap_sg_attrs(abc_dma.dma_dev, sgl->sc_list, n_num, sgl->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
release_page:
	for (i = 0; i < n_num; i++)
		put_page(*(sgl->mypage + i));

	vfree(sg);
	buf_desc->sge = NULL;
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

	dma_unmap_sg_attrs(abc_dma.dma_dev, sgl->sc_list, sgl->n_num, sgl->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
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
 * @mblk_desc[in] pre allocated bar buffer
 * @ll_num_entries [in] - the number of entries in the link list
 */
static int abc_pcie_dma_ll_setup(struct abc_pcie_sg_entry *src_sg,
			    struct abc_pcie_sg_entry *dst_sg,
			    struct abc_pcie_dma_mblk_desc *mblk_desc,
			    int ll_num_entries)
{
	struct abc_pcie_dma_ll_element ll_element;
	struct abc_pcie_sg_entry sg_dst, sg_src;
	int i, s;
	void *base_addr;

	base_addr = mblk_desc->mapping.bar_vaddr;
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
	ll_element.sar_low = LOWER(mblk_desc->dma_paddr);
	ll_element.sar_high = UPPER(mblk_desc->dma_paddr);
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
int abc_pcie_user_remote_buf_sg_build(enum dma_data_direction dir,
					size_t size,
					struct abc_buf_desc *buf_desc)
{
	size_t maxsg;
	struct abc_pcie_sg_entry *local_sg;
	__u64 remote_buf = buf_desc->remote_addr;
	struct abc_pcie_sg_list *sgl = buf_desc->sgl;

	sgl->dir = dir;
	buf_desc->sge = NULL;

	/*
	 * Allocate enough for one entry per sc_list entry, plus end of list.
	 */
	maxsg = 1 + 1;
	local_sg = vmalloc(maxsg * sizeof(struct abc_pcie_sg_entry));
	if (!local_sg) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to allocate remote buffer\n", __func__);
		return -ENOMEM;
	}

	local_sg[0].paddr = (phys_addr_t)remote_buf;
	local_sg[0].size = size;

	/* List terminator */
	local_sg[1].paddr = 0;
	local_sg[1].size = 0;

	buf_desc->sge = local_sg;

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

int abc_pcie_clean_dma_local_buffers(struct abc_dma_xfer *xfer)
{
	struct abc_pcie_sg_list *sgl;
	struct abc_pcie_sg_entry *sge;
	struct abc_pcie_dma_mblk_desc *mblk_desc = xfer->mblk_desc;
	struct dma_element_t *sblk_desc = xfer->sblk_desc;

	if (xfer->transfer_method == DMA_TRANSFER_USING_MBLOCK &&
		mblk_desc) {
		/**
		 * Note that this code assumes all the elements associated
		 * with the creation of mblk_desc are valid.
		 */
		ab_dram_free_dma_buf_kernel(mblk_desc->ab_dram_dma_buf);
		kfree(mblk_desc);
		xfer->mblk_desc = NULL;
	} else if(xfer->transfer_method == DMA_TRANSFER_USING_SBLOCK &&
		sblk_desc) {
		kfree(sblk_desc);
		xfer->sblk_desc = NULL;
	}


	sgl = xfer->local_buf.sgl;
	sge = xfer->local_buf.sge;

	switch ((xfer->local_buf).buf_type) {
	case DMA_BUFFER_KIND_USER:
	case DMA_BUFFER_KIND_VMALLOC:
		abc_pcie_user_local_buf_sg_destroy(&sge, sgl);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		abc_pcie_sg_release_from_dma_buf(sgl, sge);
		break;
	case DMA_BUFFER_KIND_CMA:
		vfree(sge);
		break;
	default:
		break;
	}

	kfree(sgl);

	xfer->local_buf.sgl = NULL;
	xfer->local_buf.sge = NULL;

	return 0;
}

int abc_pcie_clean_dma_remote_buffers(struct abc_dma_xfer *xfer)
{
	struct abc_pcie_sg_list *sgl;
	struct abc_pcie_sg_entry *sge;


	sgl = xfer->remote_buf.sgl;
	sge = xfer->remote_buf.sge;

	switch ((xfer->remote_buf).buf_type) {
	case DMA_BUFFER_KIND_USER:
		abc_pcie_user_remote_buf_sg_destroy(&sge);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		abc_pcie_sg_release_from_dma_buf(sgl, sge);
		break;
	default:
		break;
	}

	kfree(sgl);

	xfer->remote_buf.sgl = NULL;
	xfer->remote_buf.sge = NULL;

	return 0;
}

/**
 * Internal DMA API for cleaning a transfer and removing it from the internal,
 * data structures. De-allocates abc_dma_xfer passed to it.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
void abc_pcie_clean_dma_xfer_locked(struct abc_dma_xfer *xfer)
{
	unsigned long flags;
	struct abc_dma_xfer *head_xfer;
	struct abc_dma_wait_info *wait_info;
	struct list_head *pending_q;
	struct abc_pcie_sg_list *sgl;
	bool wait_for_transfer = false;

	pending_q = (xfer->dir == DMA_TO_DEVICE) ? &pending_to_dev_q :
						&pending_from_dev_q;

	wait_info = xfer->wait_info;

	/* Clear from pending list if it is in there */
	spin_lock_irqsave(&dma_spinlock, flags);

	head_xfer = list_first_entry_or_null(pending_q, struct abc_dma_xfer,
					list_pending);
	if (head_xfer == xfer)
		wait_for_transfer = true;
	else if (xfer->pending)
		list_del(&xfer->list_pending);

	spin_unlock_irqrestore(&dma_spinlock, flags);

	if (wait_for_transfer) {
		/* Wait for transfer to be done. Note that the interrupt
		 * handler will dequeue from pending list as well as
		 * re-start the new transaction.
		 */
		dev_dbg(&abc_dma.pdev->dev,
			"%s: Waiting for transfer to finish\n", __func__);
		wait_for_completion(&wait_info->done);
	}

	if (wait_info != NULL) {
		wait_info->xfer = NULL;
		/* Handle the case where there are no waiters */
		if(!wait_info->active_waiters) {
			kmem_cache_free((xfer->session)->waiter_cache,
					wait_info);
			xfer->wait_info = NULL;
		} else {
			complete_all(&wait_info->done);
		}
	}

	list_del(&xfer->list_transfers);

	if (!xfer->synced &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_DMA_BUF &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_CMA) {
		sgl = (xfer->local_buf).sgl;
		dma_sync_sg_for_cpu(abc_dma.dma_dev, sgl->sc_list, sgl->n_num,
					sgl->dir);
	}

	/* Release all buffer allocations */
	abc_pcie_clean_dma_local_buffers(xfer);
	abc_pcie_clean_dma_remote_buffers(xfer);
	dev_dbg(&abc_dma.pdev->dev, "%s: destroying xfer: id:%0llu\n",
		__func__, xfer->id);
	kfree(xfer);
}

/**
 * Kernel API for cleaning a transfer and removing it from the internal,
 * data structures. De-allocates abc_dma_xfer passed to it.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
void abc_pcie_clean_dma_xfer(struct abc_dma_xfer *xfer)
{
	struct abc_pcie_dma_session *session = xfer->session;
	mutex_lock(&session->lock);
	abc_pcie_clean_dma_xfer_locked(xfer);
	mutex_unlock(&session->lock);
}
EXPORT_SYMBOL(abc_pcie_clean_dma_xfer);

int abc_pcie_dma_get_kernel_wait(struct abc_dma_xfer *xfer,
					struct abc_pcie_dma_session **session,
					struct abc_dma_wait_info **wait_info)
{
	*wait_info = NULL;
	mutex_lock(&(xfer->session)->lock);
	if (xfer->wait_info == NULL) {
		mutex_unlock(&(xfer->session)->lock);
		dev_err(&abc_dma.pdev->dev, "%s: Wait info struct is NULL!\n",
			__func__);
		return -EINVAL;
	}
	(xfer->wait_info)->active_waiters++;
	*session = xfer->session;
	*wait_info = xfer->wait_info;
	mutex_unlock(&(xfer->session)->lock);
	return 0;
}

int abc_pcie_dma_get_user_wait(uint64_t id,
				struct abc_pcie_dma_session *session,
				struct abc_dma_wait_info **wait_info)
{
	int err = 0;
	struct abc_dma_xfer *xfer;
	*wait_info = NULL;

	mutex_lock(&session->lock);
	xfer = abc_pcie_dma_find_xfer(session, id);
	if (!xfer) {
		err = -EINVAL;
		dev_err(&abc_dma.pdev->dev, "%s: Could not find xfer:%0d\n",
			__func__, id);
		goto release_lock;
	}
	if (xfer->wait_info == NULL) {
		err = -EINVAL;
		dev_err(&abc_dma.pdev->dev, "%s: Wait info struct is NULL!\n",
			__func__);
		goto release_lock;
	}
	(xfer->wait_info)->active_waiters++;

	*wait_info = xfer->wait_info;
release_lock:
	mutex_unlock(&session->lock);
	return err;
}

int abc_pcie_dma_do_wait(struct abc_pcie_dma_session *session,
			struct abc_dma_wait_info *wait_info, int timeout,
			int *error,
			uint32_t *start_id)
{
	int err = 0;
	long remaining = 0;
	struct abc_dma_xfer *xfer;
	struct abc_pcie_sg_list *sgl;

	remaining = wait_for_completion_interruptible_timeout(&wait_info->done,
					usecs_to_jiffies(timeout));

	mutex_lock(&session->lock);
	xfer = wait_info->xfer;
	*start_id = wait_info->start_id;
	dev_dbg(&abc_dma.pdev->dev,
		"%s: completed xfer: id:%0llu start_id:%0u timeout:%0d [%0d]\n",
		__func__, wait_info->xfer_id, wait_info->start_id, timeout,
		remaining);
	/* Note that DMA_BUF does its own sync, only needs to be done for
	 * malloc and vmalloc buffers
	 */
	if (xfer != NULL && !xfer->synced &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_DMA_BUF &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_CMA) {
		sgl = (xfer->local_buf).sgl;
		dma_sync_sg_for_cpu(abc_dma.dma_dev, sgl->sc_list, sgl->n_num,
					sgl->dir);
		xfer->synced = true;
	}
	wait_info->active_waiters--;
	*error = wait_info->error;
	if (!wait_info->active_waiters && xfer == NULL)
		kmem_cache_free(session->waiter_cache, wait_info);
	mutex_unlock(&session->lock);

	if (remaining == 0) {
		err = -ETIMEDOUT;
		dev_dbg(&abc_dma.pdev->dev, "%s: DMA Timed Out (%d)\n",
			__func__, err);

	} else if(remaining < 0) {
		err = -ERESTARTSYS;
		dev_dbg(&abc_dma.pdev->dev, "%s: DMA Restart (%d)\n",
			__func__, err);
	}

	return err;
}

/**
 * Kernel API for waiting for a transfer to be done.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan.
 * @timeout[in] Timeout usec, 0=zero wait, <0 = Infinity
 * @error[out] If any error encountered in the transaction itself it will be
 *            indicated here.
 * @start_id[out] start_id incremented with every re-start
 */
int abc_pcie_wait_dma_xfer(struct abc_dma_xfer *xfer, int timeout, int *error,
	uint32_t *start_id)
{
	int err = 0;
	struct abc_pcie_dma_session *session;
	struct abc_dma_wait_info *wait_info;
	*error = 0;

	err = abc_pcie_dma_get_kernel_wait(xfer, &session, &wait_info);
	if (err)
		return err;

	err = abc_pcie_dma_do_wait(session, wait_info, timeout, error,
				start_id);

	return err;
}
EXPORT_SYMBOL(abc_pcie_wait_dma_xfer);

static int abc_pcie_setup_sblk_xfer(struct abc_dma_xfer *xfer)
{
	int err = 0;
	struct dma_element_t *dma_blk;
	struct abc_pcie_sg_entry *local_sg_entries = xfer->local_buf.sge;
	struct abc_pcie_sg_entry *remote_sg_entries = xfer->remote_buf.sge;

	xfer->transfer_method = DMA_TRANSFER_USING_SBLOCK;

	dev_dbg(&abc_dma.pdev->dev,
		"%s: SBLK transfer: local (%pa/%zu) remote (%pa/%zu)\n",
		__func__, &local_sg_entries[0].paddr,
		local_sg_entries[0].size, &remote_sg_entries[0].paddr,
		remote_sg_entries[0].size);

	dma_blk = kzalloc(sizeof(struct dma_element_t), GFP_KERNEL);
	if (!dma_blk) {
		err = -ENOMEM;
		return err;
	}

	dma_blk->src_addr = (xfer->dir == DMA_TO_DEVICE) ?
		LOWER((uint64_t)local_sg_entries[0].paddr) :
		LOWER((uint64_t)remote_sg_entries[0].paddr);
	dma_blk->src_u_addr = (xfer->dir == DMA_TO_DEVICE) ?
		UPPER((uint64_t)local_sg_entries[0].paddr) :
		UPPER((uint64_t)remote_sg_entries[0].paddr);
	dma_blk->dst_addr = (xfer->dir == DMA_TO_DEVICE) ?
		LOWER((uint64_t)remote_sg_entries[0].paddr) :
		LOWER((uint64_t)local_sg_entries[0].paddr);
	dma_blk->dst_u_addr = (xfer->dir == DMA_TO_DEVICE) ?
		UPPER((uint64_t)remote_sg_entries[0].paddr) :
		UPPER((uint64_t)local_sg_entries[0].paddr);
	dma_blk->len = local_sg_entries[0].size; /* TODO: fix this */

	xfer->sblk_desc = dma_blk;
	return 0;
}

/**
 * Build linked list for multi-block transfer locally and transfer
 * to endpoint via single block DMA transfer.
 * @xfer[in] xfer to be operated on.
 */
static int abc_pcie_setup_mblk_xfer(struct abc_dma_xfer *xfer)
{
	int dma_chan;
	size_t ll_size;
	int ll_num_entries;
	struct abc_pcie_dma_mblk_desc *mblk_desc;
	int err = 0;
	struct abc_pcie_sg_entry *src_sg; /* Scatterlist of the src buffer */
	struct abc_pcie_sg_entry *dst_sg; /* Scatterlist of the dst buffer */

	xfer->transfer_method = DMA_TRANSFER_USING_MBLOCK;

	dev_dbg(&abc_dma.pdev->dev,
		"%s: MBLK transfer (dir=%s)\n",
		__func__,
		(xfer->dir == DMA_TO_DEVICE) ? "AP2EP" : "EP2AP");
	src_sg = (xfer->dir == DMA_TO_DEVICE) ? xfer->local_buf.sge :
						xfer->remote_buf.sge;
	dst_sg = (xfer->dir == DMA_TO_DEVICE) ? xfer->remote_buf.sge :
						xfer->local_buf.sge;

	mblk_desc = kzalloc(sizeof(struct abc_pcie_dma_mblk_desc), GFP_KERNEL);
	if (!mblk_desc)
		return -ENOMEM;

	xfer->mblk_desc = mblk_desc;

	/* add one since we have an end of list marker in the end */
	ll_num_entries =
		abc_pcie_ll_count_dma_element(src_sg, dst_sg) + 1;
	if (ll_num_entries == 1) {
		dev_err(&abc_dma.pdev->dev,
			"%s: Error with received scatter lists\n",
			__func__);
		err = -ENOMEM;
		goto release_mblk;
	}
	ll_size = ll_num_entries * sizeof(struct abc_pcie_dma_ll_element);

	err = abc_pcie_dma_alloc_ab_dram(ll_size, mblk_desc);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to alloc ch%d BAR size: %d error: %d\n",
			__func__, dma_chan, ll_size, err);
		err = -ENOMEM;
		goto release_mblk;
	}

	/**
	 * Send link list to AB. Note that this does not touch the
	 * DMA engine as the DMA engine is only protected by
	 * dma_spinlock.
	 */
	mutex_lock(&dma_mutex);

	/* TODO(alexperez): Use a reserved iATU */
	err = abc_pcie_map_bar_region(abc_dma.dma_dev,
			&abc_dma.pdev->dev /* owner */, BAR_2,
			mblk_desc->size, mblk_desc->dma_paddr,
			&mblk_desc->mapping);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to map to BAR ch%d error: %d\n",
			__func__, dma_chan, err);

			goto unlock_unmap_buf;
	}

	err = abc_pcie_dma_ll_setup(src_sg, dst_sg, mblk_desc,
					ll_num_entries);

	abc_pcie_unmap_bar_region(abc_dma.dma_dev,
			&abc_dma.pdev->dev /* owner */,
			&mblk_desc->mapping);

	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: ll_setup failed ch%d error: %d\n",
			__func__, dma_chan, err);
		goto unlock_unmap_buf;
	}

	mutex_unlock(&dma_mutex);

	return 0;

unlock_unmap_buf:
	ab_dram_free_dma_buf_kernel(mblk_desc->ab_dram_dma_buf);
	mutex_unlock(&dma_mutex);
release_mblk:
	kfree(mblk_desc);
	xfer->mblk_desc = NULL;

	return err;
}

/**
 * dma_create_xfer_from_kernel_desc:
 * Allocates transfer and copies data from descriptor to transfer.
 */
static int dma_alloc_xfer_from_kernel_desc(
	struct abc_pcie_kernel_dma_desc *desc, struct abc_dma_xfer **new_xfer)
{
	int err;
	struct abc_dma_xfer *xfer;

	if (!(desc->dir == DMA_FROM_DEVICE || desc->dir == DMA_TO_DEVICE)) {
		dev_err(&abc_dma.pdev->dev, "%s: Invalid dir specification\n",
			__func__);
		return -EINVAL;
	}

	xfer = kzalloc(sizeof(struct abc_dma_xfer), GFP_KERNEL);
	if (!xfer)
		return -ENOMEM;

	xfer->dir = desc->dir;
	xfer->size = desc->size;

	xfer->local_buf.buf_type = desc->local_buf_kind;
	switch (desc->local_buf_kind) {
	case DMA_BUFFER_KIND_USER:
	case DMA_BUFFER_KIND_VMALLOC:
		xfer->local_buf.local_addr = desc->local_buf;
		xfer->local_buf.offset = 0;
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		xfer->local_buf.fd = desc->local_dma_buf_fd;
		xfer->local_buf.offset = desc->local_dma_buf_off;
		break;
	case DMA_BUFFER_KIND_CMA:
		xfer->local_buf.local_addr = desc->local_buf;
		xfer->local_buf.offset = 0;
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Invalid descriptor type\n", __func__);
		err = -EINVAL;
		goto free_xfer;
	}

	xfer->remote_buf.buf_type = desc->remote_buf_kind;
	switch (desc->remote_buf_kind) {
	case DMA_BUFFER_KIND_USER:
		xfer->remote_buf.remote_addr = desc->remote_buf;
		xfer->remote_buf.offset = 0;
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		xfer->remote_buf.fd = desc->remote_dma_buf_fd;
		xfer->remote_buf.offset = desc->remote_dma_buf_off;
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Invalid descriptor type\n", __func__);
		err = -EINVAL;
		goto free_xfer;
	}
	*new_xfer = xfer;
	return 0;

free_xfer:
	kfree(xfer);
	return err;
}

/**
 * Create and prepare a transfer.
 * @session[in] session to which this transfer belongs to.
 * @desc[in] Describes the transfer.
 * @new_xfer[out] Transfer structure that is re-used for the rest of the calls.
 * @return 0 on success
 */
int abc_pcie_create_dma_xfer(struct abc_pcie_dma_session *session,
				struct abc_pcie_kernel_dma_desc *desc,
				struct abc_dma_xfer **new_xfer)
{
	struct abc_dma_xfer *xfer;
	uint32_t local_sg_size;
	uint32_t remote_sg_size;
	struct abc_buf_desc *lbuf;
	struct abc_buf_desc *rbuf;
	int err = 0;

	if (!session)
		return -EINVAL;

	err = dma_alloc_xfer_from_kernel_desc(desc, &xfer);
	if (err)
		return err;

	dev_dbg(&abc_dma.pdev->dev, "%s: xfer_id:%0llu", __func__,
		session->next_xfer_id);

	*new_xfer = xfer;
	xfer->session = session;
	lbuf = &xfer->local_buf;
	rbuf = &xfer->remote_buf;

	if (xfer->size == 0) {
		dev_err(&abc_dma.pdev->dev, "%s: Invalid transfer size: 0\n",
				__func__);
		return -EINVAL;
	}

	dev_dbg(&abc_dma.pdev->dev,
		"%s: local_buf_type=%d, local_buf=%pK, size=%u\n",
		__func__, lbuf->buf_type, lbuf->local_addr, xfer->size);

	/* Create scatterlist of local buffer */
	lbuf->sgl = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!lbuf->sgl)
		return -ENOMEM;

	switch (lbuf->buf_type) {
	case DMA_BUFFER_KIND_USER:
		err = abc_pcie_user_local_buf_sg_build(xfer->dir, xfer->size,
							lbuf);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(xfer->dir, xfer->size,
							lbuf);
		break;
	case DMA_BUFFER_KIND_VMALLOC:
		err = abc_pcie_vmalloc_buf_sg_build(xfer->dir, xfer->size,
							lbuf);
		break;
	case DMA_BUFFER_KIND_CMA:
		err = abc_pcie_local_cma_build(xfer->dir, xfer->size, lbuf);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Unknown local DMA buffer type %d\n",
		       __func__, lbuf->buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0) {
		kfree(lbuf->sgl);
		goto clean_transfer;
	}

	/* Create scatterlist of remote buffer */
	rbuf->sgl = kzalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!rbuf->sgl) {
		err = -ENOMEM;
		abc_pcie_clean_dma_local_buffers(xfer);
		goto clean_transfer;
	}

	switch (rbuf->buf_type) {
	case DMA_BUFFER_KIND_USER:
		err = abc_pcie_user_remote_buf_sg_build(xfer->dir,
							xfer->size,
							rbuf);
		break;
	case DMA_BUFFER_KIND_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(xfer->dir,
							xfer->size,
							rbuf);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
				"%s: Unknown remote DMA buffer type %d\n",
				__func__, rbuf->buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0) {
		kfree(rbuf->sgl);
		abc_pcie_clean_dma_local_buffers(xfer);
		goto clean_transfer;
	}

	/* check the size of the scatterlist */
	local_sg_size = (lbuf->sgl)->length;
	remote_sg_size = (rbuf->sgl)->length;
	dev_dbg(&abc_dma.pdev->dev,
		"%s: Scatterlists with (%d/%d) local/remote entries\n",
		__func__, local_sg_size, remote_sg_size);

	/* Based on the scatterlists determine if sblk or mblk transfer */
	/* Single block transfer */
	if (remote_sg_size <= 2 && local_sg_size <= 2)
		err = abc_pcie_setup_sblk_xfer(xfer);

	else /* Multi block transfer */
		err = abc_pcie_setup_mblk_xfer(xfer);

	if (err)
		goto clean_buffers;

	/* Update transfer state and add to the list */
	mutex_lock(&session->lock);
	list_add_tail(&xfer->list_transfers, &session->transfers);
	xfer->pending = false;
	xfer->id = session->next_xfer_id;
	session->next_xfer_id++;
	mutex_unlock(&session->lock);
	dev_dbg(&abc_dma.pdev->dev, "%s: created xfer: id:%0llu\n",
		__func__, xfer->id);
	return 0;

	/* In case of error clear & free xfer */
clean_buffers:
	abc_pcie_clean_dma_local_buffers(xfer);
	abc_pcie_clean_dma_remote_buffers(xfer);
clean_transfer:
	kfree(xfer);
	*new_xfer = NULL;
	return err;
}
EXPORT_SYMBOL(abc_pcie_create_dma_xfer);

/**
 * Create and prepare a sessionless transfer.
 * @desc[in] Describes the transfer.
 * @new_xfer[out] Transfer structure that is re-used for the rest of the calls.
 * @return 0 on success
 */
int abc_pcie_create_sessionless_dma_xfer(struct abc_pcie_kernel_dma_desc *desc,
					struct abc_dma_xfer **new_xfer)
{
	return abc_pcie_create_dma_xfer(&global_session, desc, new_xfer);
}

/**
 * Top-level API for asynchronous DMA transfers
 * Starts the hardware DMA transfer.
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
static void abc_pcie_exec_dma_xfer(struct abc_dma_xfer *xfer)
{
	int err = 0;
	int dma_chan = DMA_CHAN;
	struct abc_dma_wait_info *wait_info;

	if (xfer->transfer_method == DMA_TRANSFER_USING_SBLOCK)
		err = dma_sblk_start(dma_chan, xfer->dir, xfer->sblk_desc);

	else /* Multi block transfer */
		err = dma_mblk_start(dma_chan, xfer->dir,
					xfer->mblk_desc->dma_paddr);

	if (!err)
		return;

	/* We have error clean, dequeue from pending, clean the transfer */
	wait_info = xfer->wait_info;
	wait_info->error = err;
	list_del(&xfer->list_pending);
	xfer->pending = false;
}

/**
 * Internal function for starting DMA transfers
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_start_dma_xfer_locked(struct abc_dma_xfer *xfer,
					uint32_t *start_id)
{
	int err = 0;
	unsigned long flags;
	struct abc_dma_wait_info *wait_info, *last_wait_info;
	bool schedule_run = false;
	struct list_head *pending_q;
	struct abc_pcie_sg_list *sgl = (xfer->local_buf).sgl;

	pending_q = (xfer->dir == DMA_TO_DEVICE) ? &pending_to_dev_q :
						&pending_from_dev_q;

	/**
	 * Guard against an application trying to re-start a transaction
	 * when it has not finished or is in an error state.
	 */
	spin_lock_irqsave(&dma_spinlock, flags);
	if (xfer->pending) {
		dev_err(&abc_dma.pdev->dev,
			"%s: Error: Cannot re-start pending DMA [ID:%0d]\n",
			__func__, xfer->id);
		err = -EBUSY; /* Ask user to try again */
	}
	spin_unlock_irqrestore(&dma_spinlock, flags);
	if (err)
		return err;

	if ((xfer->local_buf).buf_type != DMA_BUFFER_KIND_DMA_BUF &&
		(xfer->local_buf).buf_type != DMA_BUFFER_KIND_CMA)
		dma_sync_sg_for_device(abc_dma.dma_dev, sgl->sc_list,
					sgl->n_num, sgl->dir);

	/* Configure wait structures */
	wait_info = xfer->wait_info;
	last_wait_info = NULL;

	/* Reuse wait_info if there are no references to it. */
	if (wait_info != NULL && !wait_info->active_waiters) {
		reinit_completion(&wait_info->done);
	} else {
		if (wait_info)
			last_wait_info = wait_info;
		wait_info = (struct abc_dma_wait_info *)kmem_cache_alloc(
				(xfer->session)->waiter_cache, GFP_KERNEL);
		if (!wait_info)
			return -ENOMEM;
		if (last_wait_info)
			last_wait_info->xfer = NULL;
		init_completion(&wait_info->done);
	}
	wait_info->error = 0;
	wait_info->active_waiters = 0;
	wait_info->start_id = (xfer->wait_info == NULL) ? 0 :
				(xfer->wait_info)->start_id + 1;
	wait_info->xfer_id = xfer->id;
	wait_info->xfer = xfer;
	xfer->wait_info = wait_info;
	xfer->synced = false;

	dev_dbg(&abc_dma.pdev->dev, "%s: starting xfer: id:%0llu start_id:%0u\n",
			__func__, wait_info->xfer_id, wait_info->start_id);

	*start_id = wait_info->start_id;

	spin_lock_irqsave(&dma_spinlock, flags);
	xfer->pending = true;
	if (list_empty(pending_q))
		schedule_run = true;
	list_add_tail(&xfer->list_pending, pending_q);
	if (schedule_run)
		abc_pcie_exec_dma_xfer(xfer);
	spin_unlock_irqrestore(&dma_spinlock, flags);

	return err;
}

/**
 * Kernel client API for starting DMA transfers
 * @xfer[in] Data structure describing the DMA transfer including
 *               local and remote buffer descriptors & dma chan
 */
int abc_pcie_start_dma_xfer(struct abc_dma_xfer *xfer, uint32_t *start_id)
{
	int err;
	mutex_lock(&(xfer->session)->lock);
	err = abc_pcie_start_dma_xfer_locked(xfer, start_id);
	mutex_unlock(&(xfer->session)->lock);
	return err;
}
EXPORT_SYMBOL(abc_pcie_start_dma_xfer);

/**
 * Issue a Synchronous DMA transfer
 * @session[in] Currently open session.
 * @desc[in] Describes the transfer.
 * @return 0 on success
 */
int abc_pcie_issue_dma_xfer_sync(struct abc_pcie_dma_session *session,
				struct abc_pcie_kernel_dma_desc *desc)
{
	int err = 0;
	struct abc_dma_xfer *xfer;
	int xfer_error = 0;
	uint32_t start_id;

	err = abc_pcie_create_dma_xfer(session, desc, &xfer);
	if (err)
		return err;

	err = abc_pcie_start_dma_xfer(xfer, &start_id);
	if (err)
		goto  clean_transfer;

	err = abc_pcie_wait_dma_xfer(xfer, -1, &xfer_error, &start_id);

clean_transfer:
	abc_pcie_clean_dma_xfer(xfer);

	return err;
}
EXPORT_SYMBOL(abc_pcie_issue_dma_xfer_sync);

/**
 * Issue a Sessionless Synchronous DMA transfer
 * @desc[in] Describes the transfer.
 * @return 0 on success
 */
int abc_pcie_issue_sessionless_dma_xfer_sync(
					struct abc_pcie_kernel_dma_desc *desc)
{
	return abc_pcie_issue_dma_xfer_sync(&global_session, desc);
}

/**
 * Initializes a session.
 * @session[out] New session.
 * @return 0 on success
 */
int abc_pcie_dma_open_session(struct abc_pcie_dma_session *session)
{
	INIT_LIST_HEAD(&session->transfers);
	mutex_init(&session->lock);
	session->waiter_cache = waiter_cache;
	return 0;
}

/**
 * Cleans up a session.
 * @session[in] Existing session.
 * @return 0 on success
 */
void abc_pcie_dma_close_session(struct abc_pcie_dma_session *session)
{
	struct abc_dma_xfer *xfer, *nxt_xfer;
	list_for_each_entry_safe(xfer, nxt_xfer, &session->transfers,
		list_transfers) {
		abc_pcie_clean_dma_xfer(xfer);
	}
}

/**
 * Closes and destroys a session.
 * @session[in] Existing session.
 * @return 0 on success
 */
int abc_pcie_dma_drv_probe(struct platform_device *pdev)
{
	int err = 0;
	int dma_chan = 0;
	int i;

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
	if (err)
		return err;

	INIT_LIST_HEAD(&pending_to_dev_q);
	INIT_LIST_HEAD(&pending_from_dev_q);

	for (dma_chan=0; dma_chan < ABC_DMA_MAX_CHAN; dma_chan++) {
		err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);
		if (err) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to register ch%d dma irq callback: %d\n",
				__func__, dma_chan, err);
			break;
		}
	}

	/* Initialize Global Session */
	err = abc_pcie_dma_open_session(&global_session);
	if (err)
		goto restore_callback;

	waiter_cache = kmem_cache_create("wait_info",
					sizeof(struct abc_dma_wait_info),
					ARCH_SLAB_MINALIGN, 0, NULL);

	if (!waiter_cache) {
		err = -ENOMEM;
		goto restore_callback;
	}

	global_session.waiter_cache = waiter_cache;

restore_callback:
	/* Restore state of callback array if there is an error registering */
	if (err) {
		for (i=dma_chan; i >= 0; i--)
			abc_reg_dma_irq_callback(NULL, i);
	}

	return err;
}

int abc_pcie_dma_drv_remove(struct platform_device *pdev)
{
	remove_abc_pcie_dma_uapi(&abc_dma.uapi);
	abc_pcie_dma_close_session(&global_session);
	kmem_cache_destroy(waiter_cache);

	/* TODO(alexperez): remove elements! */
	list_del(&pending_to_dev_q);
	list_del(&pending_from_dev_q);
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

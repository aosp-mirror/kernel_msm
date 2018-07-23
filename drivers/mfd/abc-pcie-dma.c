/*
 * Android Airbrush coprocessor DMA library
 *
 * Copyright 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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

#define BENCHMARK_ENABLE    /* timing and throughput measurements */

#define UPPER(address) ((unsigned int)((address & 0xFFFFFFFF00000000) >> 32))
#define LOWER(address) ((unsigned int)(address & 0x00000000FFFFFFFF))

static DEFINE_MUTEX(dma_mutex);
static DECLARE_COMPLETION(dma_done);
static enum abc_dma_trans_status dma_status;

#ifdef BENCHMARK_ENABLE
static uint64_t ll_dma_start_time_zero;
static uint64_t ll_dma_start_time[DMA_MAX_CHAN];
static uint64_t ll_dma_size[DMA_MAX_CHAN];
#endif

static int abc_pcie_dma_open(struct inode *inode, struct file *filp);
static int abc_pcie_dma_release(struct inode *inode, struct file *filp);
static long abc_pcie_dma_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg);
static long abc_pcie_dma_compat_ioctl(struct file *file, unsigned int cmd,
				      unsigned long arg);

struct abc_pcie_dma {
	struct platform_device *pdev;
	struct device *dma_dev;
};

struct abc_pcie_dma abc_dma;

static const struct file_operations abc_dma_fops = {
	.owner = THIS_MODULE,
	.open = abc_pcie_dma_open,
	.release = abc_pcie_dma_release,
	.unlocked_ioctl = abc_pcie_dma_ioctl,
	.compat_ioctl = abc_pcie_dma_compat_ioctl,
	.llseek = no_llseek,
};

struct miscdevice abc_pcie_dma_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DRV_NAME_ABC_PCIE_DMA,
	.fops = &abc_dma_fops,
};

/* TODO: extend to include multiple concurrent dma channels */
static int dma_callback(uint8_t chan, enum dma_data_direction dir,
			enum abc_dma_trans_status status)
{
#ifdef BENCHMARK_ENABLE
	uint64_t callback_time_ns;
	uint64_t dma_time_ns, dma_time_zero_ns;
	uint64_t rate, rate_zero;

	callback_time_ns = ktime_to_ns(ktime_get());
	dma_time_ns = callback_time_ns - ll_dma_start_time[chan];
	dma_time_zero_ns = callback_time_ns - ll_dma_start_time_zero;
	rate = (ll_dma_size[chan] * NSEC_PER_SEC) / dma_time_ns
		/ 1024 / 1024;
	rate_zero = (ll_dma_size[chan] * NSEC_PER_SEC)
		/ dma_time_zero_ns / 1024 / 1024;
	dev_info(&abc_dma.pdev->dev,
		 "%s DMA_PERF: ch:%d, dir:%s, time: %llu, size: %llu, rate(MB/s) %llu\n",
		 __func__, chan, (dir == DMA_TO_DEVICE)?"RD(AP2EP)":"WR(EP2AP)",
		 dma_time_ns, ll_dma_size[chan], rate);
	dev_info(&abc_dma.pdev->dev,
		 "%s DMA_PERF_zero: ch:%d, dir:%s, time: %llu, size: %llu, rate(MB/s) %llu\n",
		 __func__, chan, (dir == DMA_TO_DEVICE)?"RD(AP2EP)":"WR(EP2AP)",
		 dma_time_zero_ns, ll_dma_size[chan], rate_zero);
#else
	dev_dbg(&abc_dma.pdev->dev,
		"%s: DMA callback DIR(%d), status(%d)\n",
		__func__, (int)dir, (int)chan);
#endif
	dma_status = status;
	complete(&dma_done);
	return 0;
}

/**
 * Convert Linux scatterlist to array of entries used by PCIe EP DMA engine
 * @param[in]  sc_list   Scatter gather list for DMA buffer
 * @param[in]  count     Number of entries in scatterlist
 * @param[out] sg        Array generated dma addresses and length.
 * @param[in]  maxsg     Allocated max array number of the sg
 * @return a count of sg entries used on success
 *         -EINVAL if exceeding maxsg
 */
static int scatterlist_to_abc_sg(struct scatterlist *sc_list, int count,
				 struct abc_pcie_sg_entry *sg, size_t maxsg)
{
	struct scatterlist *in_sg;
	int i, u;

	i = 0;	/* iterator of *sc_list */
	u = 0;	/* iterator of *sg */
	for_each_sg(sc_list, in_sg, count, i) {
		/* Last entry is reserved for the NULL terminator */
		if (u >= (maxsg - 1)) {
			dev_err(&abc_dma.pdev->dev,
				"maxsg exceeded\n");
			return -EINVAL;
		}
		sg[u].paddr = sg_dma_address(in_sg);
		sg[u].size = sg_dma_len(in_sg);
		dev_dbg(&abc_dma.pdev->dev,
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
int abc_pcie_sg_retrieve_from_dma_buf(int fd, struct abc_pcie_sg_entry **sg,
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
		abc_pcie_sg_release_from_dma_buf(sgl);
		return -ENOMEM;
	}

	dev_dbg(&abc_dma.pdev->dev,
		"Enter %s: n_num:%d maxsg:%zu\n", __func__, sgl->n_num, maxsg);

	/* Convert sc_list to a Synopsys compatible linked-list */
	sgl->length = scatterlist_to_abc_sg(sgl->sc_list, sgl->n_num,
					    *sg, maxsg);
	if (IS_ERR(&sgl->length)) {
		vfree((*sg));
		*sg = NULL;
		ret = PTR_ERR(&sgl->length);
		abc_pcie_sg_release_from_dma_buf(sgl);
		return ret;
	}

	return 0;
}

/**
 * API to release a scatter-gather list for a dma_buf
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		abc_sg_retrieve_from_dma_buf
 * @return 0 for SUCCESS
 */
int abc_pcie_sg_release_from_dma_buf(struct abc_pcie_sg_list *sgl)
{
	dma_buf_unmap_attachment(sgl->attach, sgl->sg_table, sgl->dir);
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	sgl->length = 0;
	dma_buf_detach(sgl->dma_buf, sgl->attach);
	dma_buf_put(sgl->dma_buf);
	return 0;
}

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * buffer
 * @param[in] dmadest  Starting virtual addr of the DMA destination
 * @param[in] size Totalsize of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct abc_pcie_sg_entry,
 *             allocated and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			page list, scatter gather list and num of its entries.
 * @return The number of sg[] entries filled out by the routine, negative if
 *		   overflow or sg[] not allocated.
 */
int abc_pcie_user_buf_sg_build(void *dmadest, size_t size,
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

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 5, 7)
	n_num = get_user_pages_fast((unsigned long)dmadest, p_num,
				    0, sgl->mypage);
#else
	n_num = get_user_pages((unsigned long)dmadest, p_num,
			       FOLL_WRITE | FOLL_FORCE, sgl->mypage, NULL);
#endif

	up_read(&current->mm->mmap_sem);
	if (n_num < 0) {
		dev_err(&abc_dma.pdev->dev,
			"%s: fail to get user_pages\n", __func__);
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
			goto free_mem;
		}

		u = scatterlist_to_abc_sg(sgl->sc_list, count, *sg, maxsg);
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
	dma_unmap_sg(abc_dma.dma_dev,
		sgl->sc_list, n_num, sgl->dir);
release_page:
	for (i = 0; i < sgl->n_num; i++)
		put_page(*(sgl->mypage + i));
free_mem:
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
 * API to release scatter gather list for a user buffer
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		abc_pcie_user_buf_sg_build
 * @return 0 for SUCCESS
 */
int abc_pcie_user_buf_sg_destroy(struct abc_pcie_sg_list *sgl)
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
	kfree(sgl->mypage);
	sgl->mypage = NULL;
	kfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;

	return 0;
}

int abc_pcie_sg_verify(struct abc_pcie_sg_entry *sg, size_t size,
		       struct abc_pcie_sg_list *sgl)
{
	int i;
	size_t len = size / sizeof(struct abc_pcie_sg_entry);

	/* At least one entry plus null terminator required */
	if (len < 2) {
		dev_err(&abc_dma.pdev->dev,
			"Invalid SG list length %zu\n", len);
		return -EINVAL;
	}
	if (sg == NULL) {
		dev_err(&abc_dma.pdev->dev, "No SG list\n");
		return -EINVAL;
	}
	for (i = 0; i < len - 1; i++) {
		if (sg[i].paddr == 0) {
			dev_err(&abc_dma.pdev->dev,
				"Early list end\n");
			return -EINVAL;
		}
		if (sg[i].size == 0) {
			dev_err(&abc_dma.pdev->dev,
				"Invalid entry size\n");
			return -EINVAL;
		}
	}
	/* Verify terminator */
	if (sg[i].paddr != 0) {
		dev_err(&abc_dma.pdev->dev,
			"Missing list terminator\n");
		return -EINVAL;
	}
	return 0;
}

/**
 * Free memory allocation for linked list
 */
int abc_pcie_ll_destroy(struct abc_pcie_dma_ll *ll)
{
	int i = 0;

	while (i <= ll->size) {
		abc_free_coherent(
			DMA_LL_LENGTH * sizeof(struct abc_pcie_dma_ll_element),
			ll->ll_element[i], ll->dma[i]);
		i++;
	}
	return 0;
}

/**
 * API to generate a linked list for DMA transfers from
 * source and destination scatterlists.
 * @src_sg[in] Scatterlist of the source buffer
 * @dst_sg[in] Scatterlist of the destination buffer
 * @ll[out]    returned linked list structure
 */
int abc_pcie_ll_build(struct abc_pcie_sg_entry *src_sg,
			    struct abc_pcie_sg_entry *dst_sg,
			    struct abc_pcie_dma_ll *ll)
{
	struct abc_pcie_dma_ll_element *ll_element, *tmp_element;
	struct abc_pcie_sg_entry sg_dst, sg_src;
	dma_addr_t dma;
	int i, s, u;

	dev_dbg(&abc_dma.pdev->dev,
			"dst_sg[%d] : Address %pa , length %zu\n",
			0, &dst_sg[0].paddr, dst_sg[0].size);
	dev_dbg(&abc_dma.pdev->dev,
			"src_sg[%d] : Address %pa , length %zu\n",
			0, &src_sg[0].paddr, src_sg[0].size);

	/* Memory allocation for linked list */
	ll_element = abc_alloc_coherent(
		DMA_LL_LENGTH * sizeof(struct abc_pcie_dma_ll_element), &dma);
	ll->size = 0;
	ll->ll_element[0] = ll_element;
	ll->dma[0] = dma;
	if (!ll_element) {
		dev_err(&abc_dma.pdev->dev, "LL alloc failed\n");
		return -EINVAL;
	}
	i = 0; /* source buffer scatterlist index */
	s = 0; /* destination buffer scatterlist index */
	u = 0; /* linked list index */
	sg_src = src_sg[i];
	sg_dst = dst_sg[s];
	if ((sg_src.paddr == 0x0) || (sg_dst.paddr == 0x0)) {
		dev_err(&abc_dma.pdev->dev,
			"Input lists invalid\n");
		return -EINVAL;
	}
	while ((sg_src.paddr != 0x0) && (sg_dst.paddr != 0x0)) {
		if (sg_src.size == sg_dst.size) {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_src.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			i++;
			s++;
			sg_src = src_sg[i];
			sg_dst = dst_sg[s];
		} else if (sg_src.size > sg_dst.size) {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_dst.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			sg_src.paddr = sg_src.paddr + sg_dst.size;
			sg_src.size = sg_src.size - sg_dst.size;
			s++;
			sg_dst = dst_sg[s];
		} else {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_src.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			sg_dst.paddr = sg_dst.paddr + sg_src.size;
			sg_dst.size = sg_dst.size - sg_src.size;
			i++;
			sg_src = src_sg[i];
		}
		u++;
		if (u == DMA_LL_LENGTH - 1) {
			dev_dbg(&abc_dma.pdev->dev,
				"%s: reached DMA_LL_LENGTH u=%d\n",
				__func__, u);
			ll_element[u].header = LL_LINK_ELEMENT;
			if ((sg_src.paddr == 0x0) || (sg_dst.paddr == 0x0)) {
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) ll->dma[0]);
				ll_element[u].sar_high =
					UPPER((uint64_t) ll->dma[0]);
				return 0;
			}
			if (ll->size >= (MAX_LL_ELEMENT-1)) {
				dev_err(&abc_dma.pdev->dev,
					"Out of dma elements\n");
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) ll->dma[0]);
				ll_element[u].sar_high =
					UPPER((uint64_t) ll->dma[0]);
				abc_pcie_ll_destroy(ll);
				return -EINVAL;
			}
			tmp_element = abc_alloc_coherent(
				DMA_LL_LENGTH
				* sizeof(struct abc_pcie_dma_ll_element),
				&dma);
			if (!tmp_element) {
				dev_err(&abc_dma.pdev->dev,
					"Element allcation failed\n");
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) ll->dma[0]);
				ll_element[u].sar_high =
					UPPER((uint64_t) ll->dma[0]);
				abc_pcie_ll_destroy(ll);
				return -EINVAL;
			}
			ll_element[u].sar_low =
				LOWER((uint64_t) dma);
			ll_element[u].sar_high =
				UPPER((uint64_t) dma);
			ll_element = tmp_element;
			u = 0;
			ll->size++;
			ll->ll_element[ll->size] = ll_element;
			ll->dma[ll->size] = dma;
		}
	}
	dev_dbg(&abc_dma.pdev->dev,
		"%s: Created %d linked lists, last list size %d\n",
		__func__, ll->size+1, u);
	ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
	ll_element[u].header = LL_LAST_LINK_ELEMENT;
	ll_element[u].sar_low = LOWER((uint64_t) ll->dma[0]);
	ll_element[u].sar_high = UPPER((uint64_t) ll->dma[0]);
	return 0;
}

/* Build abc_pcie_sg_entry structure for the remote buffer */
/* Since buffers on ABC are contiguous the generated scatterlist */
/* only has two entries, one with the starting address and one with */
/* the terminator entry */
int abc_pcie_remote_sg(__u64 remote_buf, size_t size,
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

#ifdef BENCHMARK_ENABLE
	ll_dma_start_time_zero = ktime_to_ns(ktime_get());
#endif
	dev_dbg(&abc_dma.pdev->dev,
		"%s: buf_type=%d, local_buf=%p, local_buf_size=%u\n",
		__func__, dma_desc->buf_type, dma_desc->local_buf,
		dma_desc->local_buf_size);

	/* consider validateing local buffer */

	/* Create scatterlist of local buffer */
	local_sg_list = kmalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!local_sg_list)
		return -ENOMEM;

	local_sg_list->dma_buf = NULL;
	local_sg_list->attach = NULL;
	local_sg_list->sg_table = NULL;
	local_sg_list->dir = dma_desc->dir;

	switch (dma_desc->buf_type) {
	case DMA_BUFFER_USER:
		err = abc_pcie_user_buf_sg_build(dma_desc->local_buf,
						 dma_desc->local_buf_size,
						 &local_sg_entries,
						 local_sg_list);
		break;
	case DMA_BUFFER_DMA_BUF:
		err = abc_pcie_sg_retrieve_from_dma_buf(
			dma_desc->local_dma_buf_fd,
			&local_sg_entries,
			local_sg_list);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Unknown DMA buffer type %d\n",
		       __func__, dma_desc->buf_type);
		err = -EINVAL;
		break;
	}
	if (err < 0) {
		kfree(local_sg_list);
		return err;
	}
	local_sg_size = local_sg_list->length;

	/* Create scatterlist of remote buffer */
	remote_sg_list = kmalloc(sizeof(struct abc_pcie_sg_list), GFP_KERNEL);
	if (!remote_sg_list)
		return -ENOMEM;

	remote_sg_list->dma_buf = NULL;
	remote_sg_list->attach = NULL;
	remote_sg_list->sg_table = NULL;
	remote_sg_list->dir = dma_desc->dir;
	remote_sg_entries = NULL;

	/* TODO: local_buf_size will not work for DMA buffers! */
	err = abc_pcie_remote_sg(dma_desc->remote_buf,
				 dma_desc->local_buf_size,
				 &remote_sg_entries, remote_sg_list);
	if (err < 0) {
		kfree(remote_sg_list);
		return err;
	}
	/* check the size of the scatterlist */
	remote_sg_size = remote_sg_list->length;
	dev_dbg(&abc_dma.pdev->dev,
		"%s: Scatterlists with (%d/%d) local/remote entries\n",
		__func__, local_sg_size, remote_sg_size);

	/* Based on the scatterlists determine if sblk or mblk transfer */
	/* Single block transfer */
	if (remote_sg_size <= 2 && local_sg_size <= 2) {
		dev_dbg(&abc_dma.pdev->dev,
			"%s: SBLK transfer: ->local (%pa/%zu) \
			->remote (%pa/%zu)\n", __func__,
			&local_sg_entries[0].paddr, local_sg_entries[0].size,
			&remote_sg_entries[0].paddr, remote_sg_entries[0].size);

		dma_chan = dma_desc->chan;

		/* Register DMA callback */
		err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);

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
#ifdef BENCHMARK_ENABLE
		ll_dma_size[dma_chan] = dma_blk.len;
		ll_dma_start_time[dma_chan] = ktime_to_ns(ktime_get());
#endif
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
		err = abc_reg_dma_irq_callback(NULL, dma_chan);
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

	/* Releasing buffers */
	switch (dma_desc->buf_type) {
	case DMA_BUFFER_USER:
		err = abc_pcie_user_buf_sg_destroy(local_sg_list);
		break;
	case DMA_BUFFER_DMA_BUF:
		err = abc_pcie_sg_release_from_dma_buf(local_sg_list);
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: Unknown DMA buffer type %d\n",
		       __func__, dma_desc->buf_type);
		err = -EINVAL;
		break;
	}

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
	struct abc_pcie_dma_ll *abc_pcie_ll = NULL;
#ifdef ABC_PCIE_UPLOAD_LL
	struct dma_element_t dma_blk; /* for backward compatibility */
#endif
	uint64_t ll_paddr;
	int dma_chan;
	int err = 0;
	enum dma_data_direction dir = dma_desc->dir;

	/* Allocate contiguous memory for linked list */
	abc_pcie_ll = kmalloc(sizeof(struct abc_pcie_dma_ll), GFP_KERNEL);
	if (!abc_pcie_ll) {
		dev_err(&abc_dma.pdev->dev,
			"%s: failed to kmalloc linked list buffer\n",
			__func__);
		return -ENOMEM;
	}

	/* generate linked list */
	err = abc_pcie_ll_build(src_sg, dst_sg, abc_pcie_ll);
	if (err) {
		kfree(abc_pcie_ll);
		return err;
	}

#ifdef ABC_PCIE_UPLOAD_LL
	/*
	 * WIP, this is currently just uploading one of the lists, I will have
	 * to upload all of them, iterate over
	 * abc_pcie_ll->dma[0..abc_pcie_ll->size]
	 */
	ll_paddr = HW_ABC_PCIE_LL_BASE;
	/* upload linked list */
	dma_blk.src_addr   = LOWER((uint64_t)abc_pcie_ll->dma[0]);
	dma_blk.src_u_addr = UPPER((uint64_t)abc_pcie_ll->dma[0]);
	dma_blk.dst_addr   = LOWER(ll_paddr); /* ABC memory */
	dma_blk.dst_u_addr = UPPER(ll_paddr); /* ABC memory */
	dma_blk.len = sizeof(struct abc_pcie_dma_ll);

	dma_chan = dma_desc->chan;
	dev_dbg(&abc_dma.pdev->dev,
		"%s: Uploading linked list via sblk transfer on DMA ch%d\n",
		__func__, dma_chan);

	mutex_lock(&dma_mutex);
	err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);
	reinit_completion(&dma_done);
	err = dma_sblk_start(dma_chan, DMA_TO_DEVICE, &dma_blk);
	/* add error check */
	/* consider adding timeout: wait_for_completion_interruptible_timeout*/
	err = wait_for_completion_interruptible(&dma_done);
	err = abc_reg_dma_irq_callback(NULL, dma_chan);
	mutex_unlock(&dma_mutex);
#else
	dev_dbg(&abc_dma.pdev->dev,
		"%s: Using linked list from local (AP) memory\n",
		__func__);
	ll_paddr = (uint64_t)abc_pcie_ll->dma[0];
#endif

	dma_chan = dma_desc->chan;
	/* Register DMA callback */
	err = abc_reg_dma_irq_callback(&dma_callback, dma_chan);

	dev_dbg(&abc_dma.pdev->dev,
		"%s: Starting mblk transfer on DMA ch%d\n",
		__func__, dma_chan);
#ifdef BENCHMARK_ENABLE
	ll_dma_size[dma_chan] = dma_desc->local_buf_size;
	ll_dma_start_time[dma_chan] = ktime_to_ns(ktime_get());
#endif
	/* start multi-block transfer */
	mutex_lock(&dma_mutex);
	reinit_completion(&dma_done);
	err = dma_mblk_start(dma_chan, dir, ll_paddr);
	/* wait for DMA completion, register callback before use */
	err = wait_for_completion_interruptible(&dma_done);
	/* unregister callback */
	err = abc_reg_dma_irq_callback(NULL, dma_chan);
	mutex_unlock(&dma_mutex);
	dev_dbg(&abc_dma.pdev->dev,
		"%s: mblk transfer finished on DMA ch%d\n",
		__func__, dma_chan);

	abc_pcie_ll_destroy(abc_pcie_ll);
	return 0;
}

static int abc_pcie_dma_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int abc_pcie_dma_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* IOCTL interface */
static long abc_pcie_dma_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct abc_pcie_dma_desc dma_desc;
	int err = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				  _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg,
				  _IOC_SIZE(cmd));
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"%s: access error!\n", __func__);
		return -EFAULT;
	}

	if (_IOC_TYPE(cmd) != ABC_PCIE_DMA_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER:
		dev_dbg(&abc_dma.pdev->dev,
			"%s: Received IOCTL for DMA request\n", __func__);
		err = copy_from_user(&dma_desc, (void __user *)arg,
				     sizeof(dma_desc));
		if (err) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to copy from userspace (%d)\n",
				__func__, err);
			return err;
		}
		err = abc_pcie_issue_dma_xfer(&dma_desc);
		if (err) {
			dev_err(&abc_dma.pdev->dev,
				"%s: failed to perform DMA (%d)\n",
				__func__, err);
			return err;
		}
		break;
	default:
		dev_err(&abc_dma.pdev->dev,
			"%s: unknown ioctl %c, dir=%d, #%d (0x%08x)\n",
			__func__, _IOC_TYPE(cmd), _IOC_DIR(cmd), _IOC_NR(cmd),
			cmd);
		break;
	}

	return err;
}

static long abc_pcie_dma_compat_ioctl(struct file *file, unsigned int cmd,
				      unsigned long arg)
{
	int ret = 0;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(ABC_PCIE_DMA_IOC_POST_DMA_XFER):
		cmd &= ~(_IOC_SIZEMASK << _IOC_SIZESHIFT);
		cmd |= sizeof(void *) << _IOC_SIZESHIFT;
		ret = abc_pcie_dma_ioctl(file, cmd,
					 (unsigned long)compat_ptr(arg));
		break;
	default:
		ret = abc_pcie_dma_ioctl(file, cmd, arg);
		break;
	}

	return ret;
}

static int abc_pcie_dma_drv_probe(struct platform_device *pdev)
{
	int err;

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

	err = misc_register(&abc_pcie_dma_miscdev);
	if (err) {
		dev_err(&abc_dma.pdev->dev,
			"misc_register failed\n");
		return err;
	}

	return 0;
}

static int abc_pcie_dma_drv_remove(struct platform_device *pdev)
{
	misc_deregister(&abc_pcie_dma_miscdev);
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
	},
};
module_platform_driver(abc_pcie_dma_driver);

MODULE_AUTHOR("Google Inc.");
MODULE_DESCRIPTION("ABC PCIe endpoint DMA host side driver");
MODULE_LICENSE("GPL");

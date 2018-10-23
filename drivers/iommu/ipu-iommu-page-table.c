/*
 * IOMMU page table for the Paintbox programmable IPU library
 *
 * Copyright 2019 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/mfd/abc-pcie.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/types.h>

#include "io-pgtable.h"
#include "ipu-iommu-page-table.h"

typedef u64 arm_lpae_iopte;

/* Struct accessors */
#define io_pgtable_to_data(x)						\
	container_of((x), struct arm_lpae_io_pgtable, iop)

#define io_pgtable_ops_to_data(x)					\
	io_pgtable_to_data(io_pgtable_ops_to_pgtable(x))

struct arm_lpae_io_pgtable {
	struct io_pgtable	iop;

	int			levels;
	unsigned int		pgd_bits;
	size_t			pgd_size;
	unsigned long		pg_shift;
	unsigned long		bits_per_level;

	void			*pgd;
	void			*pgd_ttbr1;
};


struct io_pgtable_ops *ipu_iommu_page_table_alloc_ops(
	struct io_pgtable_cfg *cfg,
	void *cookie)
{
	struct io_pgtable_ops *ops;
	arm_lpae_iopte *ptep;
	struct arm_lpae_io_pgtable *data;

	/* create arm l1 page table */
	ops = alloc_io_pgtable_ops(ARM_64_LPAE_S2, cfg,
			cookie);

	data = io_pgtable_ops_to_data(ops);
	ptep = data->pgd;

	/* dma map l1 page table */
	cfg->arm_lpae_s2_cfg.vttbr = abc_dma_map_single(ptep,  data->pgd_size,
		DMA_TO_DEVICE);
	if (cfg->arm_lpae_s2_cfg.vttbr == 0) {
		dev_err(cfg->iommu_dev, "%s pgtable_ops - dma mapping failed\n",
			__func__);
		goto free_ops_data;
	}

	dev_dbg(cfg->iommu_dev, "%s iommu page table was created!", __func__);
	return ops;

free_ops_data:
	kfree(data);

	return NULL;
}

dma_addr_t ipu_iommu_pg_table_get_dma_address(struct io_pgtable_ops *ops)
{
	struct arm_lpae_io_pgtable *data = io_pgtable_ops_to_data(ops);
	struct io_pgtable_cfg *cfg = &data->iop.cfg;

	return cfg->arm_lpae_s2_cfg.vttbr;
}

void ipu_iommu_free_pgtable(struct io_pgtable_ops *ops)
{
	struct arm_lpae_io_pgtable *data;
	struct io_pgtable_cfg *cfg;

	data = io_pgtable_ops_to_data(ops);
	cfg = &data->iop.cfg;
	/* dma unmap */
	abc_dma_unmap_single(cfg->arm_lpae_s2_cfg.vttbr, data->pgd_size,
			DMA_TO_DEVICE);

	free_io_pgtable_ops(ops);
}

void ipu_iommu_pgtable_update_device(struct io_pgtable_ops *ops,
		struct device *dev, void *cookie)
{
	struct arm_lpae_io_pgtable *data;
	struct io_pgtable_cfg *cfg;

	data = io_pgtable_ops_to_data(ops);
	cfg = &data->iop.cfg;
	cfg->iommu_dev = dev;
	data->iop.cookie = cookie;
}

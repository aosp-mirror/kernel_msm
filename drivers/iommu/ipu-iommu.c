/*
 * IOMMU Driver for the Paintbox programmable IPU
 *
 * Copyright (C) 2019 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/dma-iommu.h>
#include <linux/err.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "io-pgtable.h"
#include "ipu-iommu-page-table.h"
#include "../drivers/misc/ipu/ipu-regs-v2-generated.h"

struct ipu_iommu_data {
	struct device *dev;
	struct jqs_message_ipu_reg_access *jqs_msg;
	struct jqs_message_ipu_reg_values *jqs_rsp;
	uint64_t page_table_base_address;
	bool iommu_up;
};

struct ipu_domain {
	struct iommu_domain		domain;
	struct ipu_iommu_data	*iommu_data;
	struct mutex			pgtbl_mutex;
	struct mutex			init_mutex;
	struct io_pgtable_ops	*pgtbl_ops;
};

struct io_pgtable_ops	*ipu_iommu_pgtbl_ops;

#define MMU_FLUSH_DELAY 10 /* us */
#define MMU_FLUSH_MAX_ATTEMPTS 60

#define MMU_SYNC_DELAY 10 /* us */
#define MMU_SYNC_MAX_ATTEMPTS 3

#define MMU_FLUSH_ADDRESS_RSHIFT 12

static inline struct ipu_domain *to_ipu_domain(
		struct iommu_domain *dom)
{
	return container_of(dom, struct ipu_domain, domain);
}

static void ipu_iommu_reset_jqs_reg_msg(struct ipu_iommu_data *iommu_data)
{
	INIT_JQS_MSG((*(iommu_data->jqs_msg)), JQS_MESSAGE_TYPE_IPU_REG_ACCESS);
	/* since we always send only one register */
	iommu_data->jqs_msg->num_regs = 1;
	iommu_data->jqs_msg->header.size =
		offsetof(struct jqs_message_ipu_reg_access, regs) +
		sizeof(iommu_data->jqs_msg->regs[0]) *
		iommu_data->jqs_msg->num_regs;
}

static struct jqs_message_ipu_reg_values*
ipu_iommu_send_jqs_multi_reg_msg(struct ipu_iommu_data *iommu,
	struct jqs_message_ipu_reg_access *jqs_msg)
{
	uint64_t err;

	err = ipu_kernel_write_sync(iommu->dev,
		(const struct jqs_message *)jqs_msg,
		(struct jqs_message *)iommu->jqs_rsp,
		sizeof(*(iommu->jqs_rsp)));
	if (err < 0)
		return (struct jqs_message_ipu_reg_values *)err;
	return iommu->jqs_rsp;
}

static int ipu_iommu_send_jqs_iommu_activation_msg(
	struct ipu_iommu_data *iommu,
	bool activate,
	dma_addr_t page_table_addr)
{
	uint64_t err;
	struct jqs_message_iommu_activate req;
	struct jqs_message_ack rsp;

	if (iommu == NULL)
		return -EINVAL;

	if (iommu->iommu_up == false)
		return 0;

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_IOMMU_ACTIVATE);

	req.activate = activate;
	req.page_table_addr = page_table_addr;

	err = ipu_kernel_write_sync(iommu->dev,
		(const struct jqs_message *)&req,
		(struct jqs_message *)&rsp,
		sizeof(rsp));
	if (err < 0)
		return err;
	return rsp.error;
}

static inline void jsq_reg_msg_append(
	struct jqs_message_ipu_reg_access *jqs_msg,
	unsigned int address, uint64_t value, bool is_read)
{
	jqs_msg->regs[jqs_msg->num_regs].val.address =
		IPU_CSR_AXI_OFFSET + address;
	jqs_msg->regs[jqs_msg->num_regs].val.value = value;
	jqs_msg->regs[jqs_msg->num_regs].read = is_read;
	++jqs_msg->num_regs;
}


static struct jqs_message_ipu_reg_values *
ipu_iommu_send_jqs_reg_msg(struct ipu_iommu_data *iommu,
	unsigned int address, unsigned int value, bool is_read)
{
	struct jqs_message_ipu_reg_values *rsp;

	iommu->jqs_msg->num_regs = 0;
	jsq_reg_msg_append(iommu->jqs_msg, address, value, is_read);
	rsp = ipu_iommu_send_jqs_multi_reg_msg(iommu, iommu->jqs_msg);
	if (IS_ERR(rsp))
		return rsp;

	if (rsp->header.type != JQS_MESSAGE_TYPE_IPU_REG_VALUES ||
		rsp->num_regs != (is_read ? 1 : 0)) {
		dev_err(iommu->dev, "%s Error in response\n", __func__);
		return (struct jqs_message_ipu_reg_values *)(-ENOMSG);
	}
	return rsp;
}

static void ipu_iommu_shutdown_mmu(struct iommu_domain *domain,
		struct device *dev)
{
	struct ipu_domain *pb_domain = to_ipu_domain(domain);
	struct ipu_iommu_data *iommu_data = pb_domain->iommu_data;

	mutex_lock(&pb_domain->init_mutex);

	if (iommu_data && iommu_data->iommu_up) {
		ipu_iommu_send_jqs_iommu_activation_msg(iommu_data,
			false /* activate */, 0 /* page table addr */);
		iommu_data->iommu_up = false;
	}
	dev->archdata.iommu = NULL;
	pb_domain->iommu_data = NULL;
	mutex_unlock(&pb_domain->init_mutex);
	dev_dbg(dev, "%s iommu was shutdown", __func__);
}

static void ipu_iommu_firmware_suspended(struct device *dev)
{
	struct device *iommu_dev = ipu_get_iommu_device(dev);
	struct ipu_iommu_data *iommu_data = dev_get_drvdata(iommu_dev);

	dev_dbg(iommu_dev, "JQS firmware is suspended\n");
	if (iommu_data)
		iommu_data->iommu_up = false;
}

static void ipu_iommu_firmware_down(struct device *dev)
{
	dev_dbg(dev, "%s JQS firmware is going down\n", __func__);
}

static void ipu_iommu_firmware_up(struct device *dev)
{
	struct device *iommu_dev = ipu_get_iommu_device(dev);
	struct ipu_iommu_data *iommu;
	int err;

	if (!iommu_dev) {
		dev_err(dev, "%s iommu device was not found\n", __func__);
		return;
	}

	iommu = dev_get_drvdata(iommu_dev);
	iommu->iommu_up = true;
	err = ipu_iommu_send_jqs_iommu_activation_msg(
		iommu, true /*activate*/,
		iommu->page_table_base_address);
	if (err) {
		dev_err(dev,
			"%s error sending iommu activation message\n",
			__func__);
		iommu->iommu_up = false;
		return;
	}

	dev_dbg(dev, "%s iommu was loaded\n", __func__);
}

static void ipu_iommu_dram_down(struct device *dev)
{
	struct iommu_domain *domain =
		iommu_get_domain_for_dev(dev);
	struct ipu_domain *pb_domain;
	struct ipu_iommu_data *iommu_data;
	struct io_pgtable_ops *ops;

	if (domain == NULL) {
		dev_err(dev, "%s domain not initialized", __func__);
		return;
	}
	pb_domain = to_ipu_domain(domain);
	iommu_data = pb_domain->iommu_data;
	ops = pb_domain->pgtbl_ops;

	if (iommu_data)
		iommu_data->iommu_up = false;
	ipu_iommu_shutdown_mmu(domain, dev);
	if (ops) {
		mutex_lock(&pb_domain->pgtbl_mutex);
		ipu_iommu_pgtable_mem_down(ops);
		mutex_unlock(&pb_domain->pgtbl_mutex);
	}
}

static void ipu_iommu_tlb_sync(void *priv)
{
	struct ipu_iommu_data *iommu = (struct ipu_iommu_data *)priv;
	struct jqs_message_ipu_reg_values *rsp;
	int attempts = 0;

	dev_dbg(iommu->dev, "%s\n", __func__);

	rsp = ipu_iommu_send_jqs_reg_msg(iommu,
		MMU_SYNC, MMU_SYNC_SYNC_MASK, false /*is_read*/);
	if (IS_ERR(rsp)) {
		dev_err(iommu->dev, "%s Error (%d) writing to mmu sync register\n",
				__func__, (int)rsp);
		return;
	}

	while (1) {
		rsp = ipu_iommu_send_jqs_reg_msg(iommu,
			MMU_SYNC, 0, true /*is_read*/);
		if (IS_ERR(rsp)) {
			dev_err(iommu->dev,
				"%s Error (%d) reading status register\n",
				__func__, (int)rsp);
			return;
		}
		if (!(rsp->regs[0].value & MMU_SYNC_SYNC_MASK))
			break;

		if (++attempts >= MMU_SYNC_MAX_ATTEMPTS) {
			dev_err(iommu->dev,
					"%s: timeout waiting for sync to clear\n",
					__func__);
			/* TODO:  A proper recovery path for a
			 * flush FIFO timeout should be developed for
			 * this case.  b/35470877
			 */
			return;
		}
		udelay(MMU_SYNC_DELAY);
	}
}

static void ipu_iommu_tlb_flush_all(void *priv)
{
	struct ipu_iommu_data *iommu = (struct ipu_iommu_data *)priv;

	dev_dbg(iommu->dev, "%s\n", __func__);

	/* function was left empty intentionally
	 * logic was moved to JQS
	 *
	 * the IOMMU JQS logic:
	 * the JQS takes care of the IOMMU cache flushing
	 * 2 types of flush are supported
	 * - per address flushes - done for smaller buffers
	 * - full flush - done when a larger memory area should be freed
	 * when performing a full flush no DMA transfers can occur.
	 * since the kernel IOMMU can not move forward with allocating
	 * new buffers before we flush the address range for old ones the
	 * following logic is implemented when a full flush is needed:
	 * - if no transfers are active flush and continue, otherwise:
	 * - all new transfers are stalled
	 * - everytime a transaction finishes,
	 *		check if all transfers are finished
	 *		and if so:
	 * -  - flush
	 * -  - after flushing try to start transaction on all channels
	 * -  - enable new transaction
	 */
}

static void ipu_iommu_tlb_add_flush(unsigned long iova, size_t size,
		size_t granule, bool leaf, void *priv)
{
	struct ipu_iommu_data *iommu = (struct ipu_iommu_data *)priv;
	unsigned long offset;
	int attempts = 0;
	struct jqs_message_ipu_reg_values *rsp;

	dev_dbg(iommu->dev, "%s:iova 0x%016lx sz %zu leaf %d\n", __func__, iova,
			size, leaf);

	for (offset = 0; offset < size; offset += PAGE_SIZE) {
		while (1) {
			rsp = ipu_iommu_send_jqs_reg_msg(iommu,
				MMU_FLUSH_FIFO_STATUS, 0, true /*is_read*/);
			if (IS_ERR(rsp)) {
				dev_err(iommu->dev,
					"%s Error (%d) reading status register\n",
					__func__, (int)rsp);
				return;
			}
			if (!(rsp->regs[0].value &
				MMU_FLUSH_FIFO_STATUS_FULL_MASK))
				break;
			if (++attempts >= MMU_FLUSH_MAX_ATTEMPTS) {
				dev_err(iommu->dev,
						"%s: timeout waiting for flush FIFO to clear\n",
						__func__);
				/* TODO:  A proper recovery path for a
				 * flush FIFO timeout should be developed for
				 * this case.  b/35470877
				 */
				return;
			}

			udelay(MMU_FLUSH_DELAY);
		}
		rsp = ipu_iommu_send_jqs_reg_msg(iommu, MMU_FLUSH_ADDRESS,
			(iova + offset) >> MMU_FLUSH_ADDRESS_RSHIFT,
			false /*is_read*/);
		if (IS_ERR(rsp)) {
			dev_err(iommu->dev,
				"%s Error (%d) writing to flush address register\n",
				__func__, (int)rsp);
			return;
		}
	}
	ipu_iommu_tlb_sync(priv);
}

static struct iommu_gather_ops ipu_iommu_gather_ops = {
	.tlb_flush_all	= ipu_iommu_tlb_flush_all,
	.tlb_add_flush	= ipu_iommu_tlb_add_flush,
	.tlb_sync	= ipu_iommu_tlb_sync
};

/* iommu_ops functions */
static struct iommu_domain *ipu_iommu_domain_alloc(unsigned int type)
{
	struct ipu_domain *pb_domain;
	int ret;

	if (type != IOMMU_DOMAIN_DMA)
		return NULL;

	pb_domain = kzalloc(sizeof(*pb_domain), GFP_KERNEL);
	if (!pb_domain)
		return NULL;

	pb_domain->domain.type = type;
	pb_domain->domain.ops = ipu_bus_type.iommu_ops;
	pb_domain->domain.pgsize_bitmap =
			ipu_bus_type.iommu_ops->pgsize_bitmap;

	ret = iommu_get_dma_cookie(&pb_domain->domain);
	if (ret < 0) {
		kfree(pb_domain);
		return ERR_PTR(ret);
	}

	mutex_init(&pb_domain->init_mutex);
	mutex_init(&pb_domain->pgtbl_mutex);

	return &pb_domain->domain;
}

static void ipu_iommu_domain_free(struct iommu_domain *domain)
{
	struct ipu_domain *pb_domain = to_ipu_domain(domain);

	kfree(pb_domain);
}

static int ipu_iommu_map(struct iommu_domain *domain, unsigned long iova,
		phys_addr_t paddr, size_t size, int prot)
{
	struct ipu_domain *pb_domain = to_ipu_domain(domain);
	struct io_pgtable_ops *ops = pb_domain->pgtbl_ops;
	int ret;

	if (!ops)
		return -ENODEV;

	mutex_lock(&pb_domain->pgtbl_mutex);
	ret = ops->map(ops, iova, paddr, size, prot);
	mutex_unlock(&pb_domain->pgtbl_mutex);

	return ret;
}

static int ipu_iommu_create_page_table(struct iommu_domain *domain,
		struct device *dev)
{
	struct ipu_domain *pb_domain = to_ipu_domain(domain);
	struct ipu_iommu_data *iommu_data;
	struct paintbox_pdata *pdata;
	struct io_pgtable_cfg pgtbl_cfg;
	struct device *iommu_dev;
	bool map_start = false;

	iommu_dev = ipu_get_iommu_device(dev);
	if (!iommu_dev)
		return -ENODEV;

	mutex_lock(&pb_domain->init_mutex);
	if (dev->archdata.iommu) {
		mutex_unlock(&pb_domain->init_mutex);
		dev_dbg(dev, "%s dev (%s) already attached to an IOMMU\n",
			__func__, dev_name(dev));
		return 0;
	}

	iommu_data = dev_get_drvdata(iommu_dev);

	if (ipu_iommu_pgtbl_ops) {
		pb_domain->pgtbl_ops = ipu_iommu_pgtbl_ops;
		ipu_iommu_pgtable_update_device(
				pb_domain->pgtbl_ops, iommu_dev, iommu_data);
		dev_dbg(dev, "%s loading existing page table\n", __func__);
	} else {
		pdata = iommu_dev->platform_data;
		pgtbl_cfg = (struct io_pgtable_cfg) {
			.pgsize_bitmap	= pdata->page_size_bitmap,
			.ias		= pdata->input_address_size,
			.oas		= pdata->output_address_size,
			.tlb		= &ipu_iommu_gather_ops,
			.iommu_dev	= iommu_dev,
			.quirks = IO_PGTABLE_QUIRK_ARM_NS,
		};

		pb_domain->pgtbl_ops = ipu_iommu_page_table_alloc_ops(
			&pgtbl_cfg,	iommu_data);
		if (pb_domain->pgtbl_ops == NULL) {
			mutex_unlock(&pb_domain->init_mutex);
			return -ENOMEM;
		}
		map_start = true;
		ipu_iommu_pgtbl_ops = pb_domain->pgtbl_ops;
		dev_dbg(dev, "%s created new page table\n", __func__);
	}

	pb_domain->iommu_data = iommu_data;
	dev->archdata.iommu = pb_domain;
	iommu_data->page_table_base_address = ipu_iommu_pg_table_get_dma_address
		(pb_domain->pgtbl_ops);

	mutex_lock(&pb_domain->pgtbl_mutex);
	ipu_iommu_pgtable_mem_up(pb_domain->pgtbl_ops);
	mutex_unlock(&pb_domain->pgtbl_mutex);
	mutex_unlock(&pb_domain->init_mutex);
	/* identity map first Gb - due to iommu hw bug
	 * TODO(b/123649740)
	 */
	if (map_start && ipu_iommu_map(domain, 0 /*iova*/,
		0 /*paddr*/, SZ_1G, 0xd7)) {
		dev_err(dev, "%s failed to identity map first Gb\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

static void ipu_iommu_dram_up(struct device *dev)
{
	struct iommu_domain *domain =
		iommu_get_domain_for_dev(dev);

	if (domain == NULL) {
		dev_err(dev, "%s domain no initialized", __func__);
		return;
	}
	ipu_iommu_create_page_table(
		domain, dev);
}

static int ipu_iommu_attach_dev(struct iommu_domain *domain,
		struct device *dev)
{
	dev_dbg(dev, "%s: %s attached to iommu\n", __func__,
			dev_name(dev));

	return 0;
}

static void ipu_iommu_detach_dev(struct iommu_domain *domain,
		struct device *dev)
{
	dev_dbg(dev, "%s: %s detached from iommu\n", __func__,
			dev_name(dev));
}

static phys_addr_t ipu_iommu_iova_to_phys(struct iommu_domain *domain,
		dma_addr_t iova)
{
	struct ipu_domain *pb_domain = to_ipu_domain(domain);
	struct io_pgtable_ops *ops = pb_domain->pgtbl_ops;
	phys_addr_t ret;

	if (!ops)
		return -ENODEV;

	mutex_lock(&pb_domain->pgtbl_mutex);
	ret = ops->iova_to_phys(ops, iova);
	mutex_unlock(&pb_domain->pgtbl_mutex);

	dev_dbg(pb_domain->iommu_data->dev,
			"iova to phys iova %pad -> pa %pa\n", &iova, &ret);

	return ret;
}

static size_t ipu_iommu_unmap(struct iommu_domain *domain,
		unsigned long iova, size_t size)
{
	struct ipu_domain *pb_domain = to_ipu_domain(domain);
	struct io_pgtable_ops *ops = pb_domain->pgtbl_ops;
	size_t ret;

	if (!ops)
		return -ENODEV;

	mutex_lock(&pb_domain->pgtbl_mutex);
	ret = ops->unmap(ops, iova, size);
	mutex_unlock(&pb_domain->pgtbl_mutex);

	return ret;
}

static int ipu_iommu_add_device(struct device *dev)
{
	struct iommu_group *group;
	struct device *iommu_dev;

	/* Don't add the IOMMU to the IOMMU group. */
	iommu_dev = ipu_get_iommu_device(dev);
	if (iommu_dev == dev)
		return 0;

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group)) {
		dev_dbg(dev,
			"%s unable to find group for device %s\n",
			__func__, dev_name(dev));
		return PTR_ERR(group);
	}
	iommu_group_put(group);

	return 0;
}

static void ipu_iommu_remove_device(struct device *dev)
{
	iommu_group_remove_device(dev);
}

size_t ipu_iommu_map_sg(struct iommu_domain *domain, unsigned long iova,
			 struct scatterlist *sg, unsigned int nents, int prot)
{
	struct ipu_domain *pb_domain = to_ipu_domain(domain);
	struct io_pgtable_ops *ops = pb_domain->pgtbl_ops;
	int ret;
	size_t sz;

	if (!ops)
		return -ENODEV;

	mutex_lock(&pb_domain->pgtbl_mutex);
	ret = ops->map_sg(ops, iova, sg, nents, prot, &sz);
	mutex_unlock(&pb_domain->pgtbl_mutex);

	return ret;
}


static struct iommu_group *ipu_iommu_device_group(struct device *dev)
{
	struct iommu_group *group;

	group = ipu_get_device_group(dev);
	if (IS_ERR(group))
		return group;

	return group;
}

static struct iommu_ops ipu_iommu_ops = {
	.domain_alloc		= ipu_iommu_domain_alloc,
	.domain_free		= ipu_iommu_domain_free,
	.attach_dev		= ipu_iommu_attach_dev,
	.detach_dev		= ipu_iommu_detach_dev,
	.map			= ipu_iommu_map,
	.unmap			= ipu_iommu_unmap,
	.map_sg			= ipu_iommu_map_sg,
	.iova_to_phys		= ipu_iommu_iova_to_phys,
	.add_device		= ipu_iommu_add_device,
	.remove_device		= ipu_iommu_remove_device,
	.device_group		= ipu_iommu_device_group,
};


static const struct paintbox_device_ops ipu_iommu_dev_ops = {
	.firmware_up = ipu_iommu_firmware_up,
	.firmware_down = ipu_iommu_firmware_down,
	.firmware_suspended = ipu_iommu_firmware_suspended,
	.dram_up = ipu_iommu_dram_up,
	.dram_down = ipu_iommu_dram_down,
};

static int ipu_iommu_probe(struct device *dev)
{
	struct ipu_iommu_data *iommu_data;
	struct paintbox_pdata *pdata;
	int ret = 0;

	iommu_data = devm_kzalloc(dev, sizeof(*iommu_data), GFP_KERNEL);
	if (!iommu_data)
		return -ENOMEM;

	iommu_data->jqs_msg = devm_kzalloc(dev,
		sizeof(struct jqs_message_ipu_reg_access), GFP_KERNEL);
	if (iommu_data->jqs_msg == NULL) {
		ret = -ENOMEM;
		goto free_iommu_data;
	}

	iommu_data->jqs_rsp = devm_kzalloc(dev,
		sizeof(struct jqs_message_ipu_reg_values), GFP_KERNEL);
	if (iommu_data->jqs_rsp == NULL) {
		ret = -ENOMEM;
		goto free_jqs_msg;
	}

	dev_dbg(dev, "%s\n", __func__);

	ipu_iommu_reset_jqs_reg_msg(iommu_data);
	dev_set_drvdata(dev, iommu_data);
	iommu_data->dev = dev;
	pdata = dev->platform_data;
	ipu_iommu_ops.pgsize_bitmap = pdata->page_size_bitmap;
	if (!iommu_present(&ipu_bus_type))
		ret = bus_set_iommu(&ipu_bus_type, &ipu_iommu_ops);
	if (ret < 0) {
		dev_err(dev, "failed to register iommu with bus\n");
		dev_set_drvdata(dev, NULL);
		goto free_jqs_res;
	}

	ipu_set_device_ops(dev, &ipu_iommu_dev_ops);

	iommu_data->iommu_up = false;
	return 0;

free_jqs_res:
	kfree(iommu_data->jqs_rsp);

free_jqs_msg:
	kfree(iommu_data->jqs_msg);

free_iommu_data:
	kfree(iommu_data);

	return ret;
}

static int ipu_iommu_remove(struct device *dev)
{
	struct ipu_iommu_data *iommu_data = dev_get_drvdata(dev);

	ipu_iommu_send_jqs_iommu_activation_msg(iommu_data,
		false /* activate */, 0 /* page table addr */);
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static struct device_driver ipu_iommu_driver = {
	.name	= "ipu-iommu",
	.bus	= &ipu_bus_type,
	.probe	= ipu_iommu_probe,
	.remove	= ipu_iommu_remove,
};

int ipu_iommu_init(void)
{
	return driver_register(&ipu_iommu_driver);
}

static void __exit ipu_iommu_exit(void)
{
	driver_unregister(&ipu_iommu_driver);
}

subsys_initcall(ipu_iommu_init);
module_exit(ipu_iommu_exit);

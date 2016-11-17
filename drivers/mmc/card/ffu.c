/*
 * *  ffu.c
 *
 *  Copyright 2007-2008 Pierre Ossman
 *
 *  Modified by SanDisk Corp., Copyright (c) 2013 SanDisk Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program includes bug.h, card.h, host.h, mmc.h, scatterlist.h,
 * slab.h, ffu.h & swap.h header files
 * The original, unmodified version of this program - the mmc_test.c
 * file - is obtained under the GPL v2.0 license that is available via
 * http://www.gnu.org/licenses/,
 * or http://www.opensource.org/licenses/gpl-2.0.php
*/

#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include "ffu.h"
extern int mmc_get_ext_csd(struct mmc_card *card, u8 **new_ext_csd);
/**
 * struct mmc_ffu_pages - pages allocated by 'alloc_pages()'.
 *  <at> page: first page in the allocation
 *  <at> order: order of the number of pages allocated
 */
struct mmc_ffu_pages {
	struct page *page;
	unsigned int order;
};

/**
 * struct mmc_ffu_mem - allocated memory.
 *  <at> arr: array of allocations
 *  <at> cnt: number of allocations
 */
struct mmc_ffu_mem {
	struct mmc_ffu_pages *arr;
	unsigned int cnt;
};

struct mmc_ffu_area {
	unsigned long max_sz;
	unsigned int max_tfr;
	unsigned int max_segs;
	unsigned int max_seg_sz;
	unsigned int blocks;
	unsigned int sg_len;
	struct mmc_ffu_mem *mem;
	struct scatterlist *sg;
};

static void mmc_ffu_prepare_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned int sg_len,
	u32 arg, unsigned int blocks, unsigned int blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ? MMC_WRITE_BLOCK :
			MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = arg;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	if (blocks == 1) {
		mrq->stop = NULL;
	} else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Checks that a normal transfer didn't have any errors
 */
static int mmc_ffu_check_result(struct mmc_request *mrq)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data);

	if (mrq->cmd->error != 0) {
		return -EINVAL;
    }

	if (mrq->data->error != 0) {
		return -EINVAL;
    }

	if (mrq->stop != NULL && mrq->stop->error != 0) {
		return -1;
    }

	if (mrq->data->bytes_xfered != (mrq->data->blocks * mrq->data->blksz)) {
		return -EINVAL;
    }

	return 0;
}

static int mmc_ffu_busy(struct mmc_command *cmd)
{
	return !(cmd->resp[0] & R1_READY_FOR_DATA) ||
		(R1_CURRENT_STATE(cmd->resp[0]) == R1_STATE_PRG);
}

static int mmc_ffu_wait_busy(struct mmc_card *card)
{
	int ret, busy = 0;
	struct mmc_command cmd = {0};

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	do {
		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret) {
			break;
        }

		if (!busy && mmc_ffu_busy(&cmd)) {
			busy = 1;
			if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY) {
				pr_warn("%s: Warning: Host did not "
					"wait for busy state to end.\n",
					mmc_hostname(card->host));
			}
		}

	} while (mmc_ffu_busy(&cmd));

	return ret;
}

/*
 * transfer with certain parameters
 */
static int mmc_ffu_simple_transfer(struct mmc_card *card,
	struct scatterlist *sg, unsigned int sg_len, u32 arg,
	unsigned int blocks, unsigned int blksz, int write)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_command stop = {0};
	struct mmc_data data = {0};

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;
	mmc_ffu_prepare_mrq(card, &mrq, sg, sg_len, arg, blocks, blksz,
		write);
	mmc_wait_for_req(card->host, &mrq);

	mmc_ffu_wait_busy(card);

	return mmc_ffu_check_result(&mrq);
}

/*
 * Map memory into a scatterlist.
 */
static int mmc_ffu_map_sg(struct mmc_ffu_mem *mem, unsigned long size,
	struct scatterlist *sglist, unsigned int max_segs,
	unsigned int max_seg_sz, unsigned int *sg_len,
	int min_sg_len)
{
	struct scatterlist *sg = NULL;
	unsigned int i;
	unsigned long sz = size;

	sg_init_table(sglist, max_segs);
	if (min_sg_len > max_segs) {
		min_sg_len = max_segs;
    }

	*sg_len = 0;
	do {
		for (i = 0; i < mem->cnt; i++) {
			unsigned long len = PAGE_SIZE << mem->arr[i].order;

			if (min_sg_len && (size / min_sg_len < len)) {
				len = ALIGN(size / min_sg_len, CARD_BLOCK_SIZE);
            }

			if (len > sz) {
				len = sz;
            }

			if (len > max_seg_sz) {
				len = max_seg_sz;
            }

			if (sg) {
				sg = sg_next(sg);
			} else {
				sg = sglist;
            }

			if (!sg) {
				return -EINVAL;
            }

			sg_set_page(sg, mem->arr[i].page, len, 0);
			sz -= len;
			*sg_len += 1;
			if (!sz) {
				break;
            }
		}
	} while (sz);

	if (sg) {
		sg_mark_end(sg);
    }

	return 0;
}

static void mmc_ffu_free_mem(struct mmc_ffu_mem *mem)
{
	if (!mem) {
		return;
	}

	while (mem->cnt--) {
		__free_pages(mem->arr[mem->cnt].page, mem->arr[mem->cnt].order);
	}

	if (mem->arr) {
		kfree(mem->arr);
	}
}

/*
 * Cleanup struct mmc_ffu_area.
 */
static int mmc_ffu_area_cleanup(struct mmc_ffu_area *area)
{
	kfree(area->sg);
	mmc_ffu_free_mem(area->mem);

	return 0;
}

/*
 * Allocate a lot of memory, preferably max_sz but at least min_sz. In case
 * there isn't much memory do not exceed 1/16th total low mem pages. Also do
 * not exceed a maximum number of segments and try not to make segments much
 * bigger than maximum segment size.
 */
static struct mmc_ffu_mem *mmc_ffu_alloc_mem(unsigned long min_sz,
	unsigned long max_sz, unsigned int max_segs, unsigned int max_seg_sz)
{
	unsigned long max_page_cnt = DIV_ROUND_UP(max_sz, PAGE_SIZE);
	unsigned long min_page_cnt = DIV_ROUND_UP(min_sz, PAGE_SIZE);
	unsigned long max_seg_page_cnt = DIV_ROUND_UP(max_seg_sz, PAGE_SIZE);
	unsigned long page_cnt = 0;
	unsigned long limit = nr_free_buffer_pages() >> 4;
	struct mmc_ffu_mem *mem;

	if (max_page_cnt > limit) {
		max_page_cnt = limit;
	}

	if (min_page_cnt > max_page_cnt) {
		min_page_cnt = max_page_cnt;
	}

	if (max_seg_page_cnt > max_page_cnt) {
		max_seg_page_cnt = max_page_cnt;
	}

	if (max_segs > max_page_cnt) {
		max_segs = max_page_cnt;
	}

	mem = kzalloc(sizeof(struct mmc_ffu_mem), GFP_KERNEL);
	if (!mem) {
		pr_err("FFU: %s: mem->arr is null\n", __func__);
		return NULL;
	}

	mem->arr = kzalloc(sizeof(struct mmc_ffu_pages) * max_segs, GFP_KERNEL);
	if (!mem->arr) {
		pr_err("FFU: %s: mem->arr is null\n", __func__);
		goto out_free;
	}

	while (max_page_cnt) {
		struct page *page;
		unsigned int order;
		gfp_t flags = GFP_KERNEL | GFP_DMA | __GFP_NOWARN |
			__GFP_NORETRY;

		order = get_order(max_seg_page_cnt << PAGE_SHIFT);
		while (1) {
			page = alloc_pages(flags, order);
			if (page || !order) {
				break;
            }

			order -= 1;
		}
		if (!page) {
			if (page_cnt < min_page_cnt) {
				goto out_free;
            }

			break;
		}
		mem->arr[mem->cnt].page = page;
		mem->arr[mem->cnt].order = order;
		mem->cnt += 1;
		if (max_page_cnt <= (1UL << order)) {
			break;
        }

		max_page_cnt -= 1UL << order;
		page_cnt += 1UL << order;
		if (mem->cnt >= max_segs) {
			if (page_cnt < min_page_cnt) {
				goto out_free;
            }

			break;
		}
	}

	return mem;

out_free:
	mmc_ffu_free_mem(mem);
	return NULL;
}

/*
 * Initialize an area for data transfers.
 * Copy the data to the allocated pages.
 */
static int mmc_ffu_area_init(struct mmc_ffu_area *area, struct mmc_card *card,
	u8 *data, unsigned int size)
{
	int ret, i, length;

	area->max_segs = card->host->max_segs;
	area->max_seg_sz = card->host->max_seg_size & ~(CARD_BLOCK_SIZE - 1);
	area->max_tfr = size;

	if (area->max_tfr >> 9 > card->host->max_blk_count) {
		area->max_tfr = card->host->max_blk_count << 9;
    }

	if (area->max_tfr > card->host->max_req_size) {
		area->max_tfr = card->host->max_req_size;
    }

	if (area->max_tfr / area->max_seg_sz > area->max_segs) {
		area->max_tfr = area->max_segs * area->max_seg_sz;
    }

	/*
	 * Try to allocate enough memory for a max. sized transfer. Less is OK
	 * because the same memory can be mapped into the scatterlist more than
	 * once. Also, take into account the limits imposed on scatterlist
	 * segments by the host driver.
	 */
	area->mem = mmc_ffu_alloc_mem(1, area->max_tfr, area->max_segs,
			area->max_seg_sz);
	if (!area->mem) {
		pr_err("FFU: %s: area->mem is null\n", __func__);
		return -ENOMEM;
	}

	/* copy data to page */
	length = 0;
	for (i = 0; i < area->mem->cnt; i++) {
		 memcpy(page_address(area->mem->arr[i].page), data + length,
			min(size - length, area->max_seg_sz));
		length += area->max_seg_sz;
	}

	area->sg = kmalloc(sizeof(struct scatterlist) * area->max_segs,
		GFP_KERNEL);
	if (!area->sg) {
		ret = -ENOMEM;
		goto out_free;
	}

	ret = mmc_ffu_map_sg(area->mem, size, area->sg,
			area->max_segs, area->max_seg_sz, &area->sg_len, 1);

	if (ret != 0) {
        pr_err("FFU: %s: ret = %d\n", __func__, ret);

		goto out_free;
    }

	return 0;

out_free:
	mmc_ffu_area_cleanup(area);
	return ret;
}

static int mmc_ffu_write(struct mmc_card *card, u8 *src, u32 arg,
	unsigned int size)
{
	int rc;
	struct mmc_ffu_area mem;

	mem.sg = NULL;
	mem.mem = NULL;

	if (!src) {
		pr_err("FFU: %s: data buffer is NULL\n",
			mmc_hostname(card->host));
		return -EINVAL;
	}
	rc = mmc_ffu_area_init(&mem, card, src, size);
	if (rc != 0) {
		pr_err("FFU: %s: rc = %d\n", __func__, rc);
		goto exit;
	}
	rc = mmc_ffu_simple_transfer(card, mem.sg, mem.sg_len, arg,
		size / CARD_BLOCK_SIZE, CARD_BLOCK_SIZE, 1);

exit:
	mmc_ffu_area_cleanup(&mem);
	return rc;
}
/* Flush all scheduled work from the MMC work queue.
 * and initialize the MMC device */
static int mmc_ffu_restart(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	int err = 0;

	(void)mmc_cache_ctrl(host,0);
	err = mmc_power_save_host(host);
	if (err) {
		pr_warn("%s: going to sleep failed (%d)!!!\n",
			__func__ , err);
		goto exit;
	}

	err = mmc_power_restore_host(host);

exit:

	return err;
}

/* Host set the EXT_CSD */
static int mmc_host_set_ffu(struct mmc_card *card, u32 ffu_enable)
{
	int err;

	/* check if card is eMMC 5.0 or higher */
	if (card->ext_csd.rev < 7) {
        pr_err("FFU: %s: card->ext_csd.rev = %d\n", __func__, card->ext_csd.rev);
		return -EINVAL;
    }

	if (FFU_ENABLED(ffu_enable)) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_FW_CONFIG, MMC_FFU_ENABLE,
			card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("%s: switch to FFU failed with error %d\n",
				mmc_hostname(card->host), err);
			return err;
		}
	}

	return 0;
}

#define CID_MANFID_SAMSUNG    0x15
#define CID_MANFID_TOSHIBA    0x11

int mmc_ffu_download(struct mmc_card *card, struct mmc_command *cmd,
	u8 *data, unsigned int buf_bytes)
{
	u8 *ext_csd;
	int err;
	int ret;

	if (card == NULL) {
		err = -EINVAL;
		pr_err("FFU: %s: argument error (card == NULL)\n", __func__);
		goto exit;
	}

	if (cmd == NULL) {
		err = -EINVAL;
		pr_err("FFU: %s: argument error (cmd == NULL)\n", __func__);
		goto exit;
	}

	/* Read the EXT_CSD */
	err = mmc_get_ext_csd(card, &ext_csd);
	if (err) {
		pr_err("FFU: %s: error %d sending ext_csd\n",
			mmc_hostname(card->host), err);
		goto exit;
	}

	/* Check if FFU is supported by card */
	if (!FFU_SUPPORTED_MODE(ext_csd[EXT_CSD_SUPPORTED_MODE])) {
		err = -EINVAL;
		pr_err("FFU: %s: error %d FFU is not supported\n",
			mmc_hostname(card->host), err);
		goto free_ext_csd;
	}

	err = mmc_host_set_ffu(card, ext_csd[EXT_CSD_FW_CONFIG]);
	if (err) {
		pr_err("FFU: %s: error %d FFU is not supported\n",
			mmc_hostname(card->host), err);
		err = -EINVAL;
		goto free_ext_csd;
	}

	/* set device to FFU mode */
	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_MODE_CONFIG,
		MMC_FFU_MODE_SET, card->ext_csd.generic_cmd6_time);
	if (err) {
		pr_err("FFU: %s: error %d FFU is not supported\n",
			mmc_hostname(card->host), err);
		goto exit_normal;
	}

	/* set CMD ARG */
	cmd->arg = ext_csd[EXT_CSD_FFU_ARG] |
		ext_csd[EXT_CSD_FFU_ARG + 1] << 8 |
		ext_csd[EXT_CSD_FFU_ARG + 2] << 16 |
		ext_csd[EXT_CSD_FFU_ARG + 3] << 24;

	pr_err("FFU: %s: card->cid.manfid=%d\n", __func__, card->cid.manfid);

	/* If arg is zero, should be set to a special value for samsung eMMC
	 */
	if (card->cid.manfid == CID_MANFID_SAMSUNG && cmd->arg == 0x0)
	{
		cmd->arg = 0xc7810000;
	}

	err = mmc_ffu_write(card, data, cmd->arg, buf_bytes);

exit_normal:
	/* host switch back to work in normal MMC Read/Write commands */
	ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
		EXT_CSD_MODE_CONFIG, MMC_FFU_MODE_NORMAL,
		card->ext_csd.generic_cmd6_time);
	if (ret)
		err = ret;
free_ext_csd:
	kfree(ext_csd);
exit:
	return err;
}
EXPORT_SYMBOL(mmc_ffu_download);

int mmc_ffu_install(struct mmc_card *card)
{
	u8 *ext_csd;
	int err;
	u32 ffu_data_len;
	u32 timeout;

	if (card == NULL) {
		err = -EINVAL;
		pr_err("FFU: %s: argument error (card == NULL)\n", __func__);
		goto exit;
	}

	err = mmc_get_ext_csd(card, &ext_csd);
	if (err) {
		pr_err("FFU: %s: error %d sending ext_csd\n",
			mmc_hostname(card->host), err);
		goto exit;
	}
	pr_err("FFU_STATUS before0: %d !!!\n",ext_csd[EXT_CSD_FFU_STATUS]);
	/* Check if FFU is supported */
	if (!FFU_SUPPORTED_MODE(ext_csd[EXT_CSD_SUPPORTED_MODE]) ||
		FFU_CONFIG(ext_csd[EXT_CSD_FW_CONFIG])) {
		err = -EINVAL;
		pr_err("FFU: %s: error %d FFU is not supported\n",
			mmc_hostname(card->host), err);
		goto free_ext_csd;
	}

	/* check mode operation */
	if (!FFU_FEATURES(ext_csd[EXT_CSD_FFU_FEATURES])) {

		/*work around for toshiba eMMC*/
		if(card->cid.manfid == CID_MANFID_TOSHIBA)
		{
			kfree(ext_csd);
			pr_err("FFU: %s: toshiba emmc need to check FFU_STATUS before restart eMMC.\n", __func__);
			/* read ext_csd */
			err = mmc_get_ext_csd(card, &ext_csd);
			if (err) {
				pr_err("FFU: %s: error %d sending ext_csd before restart eMMC\n",
					mmc_hostname(card->host), err);
				goto exit;
			}
			/* return status */
			err = ext_csd[EXT_CSD_FFU_STATUS];
			if (err) {
				pr_err("FFU: %s: error %d FFU install before restart eMMC:\n",
					mmc_hostname(card->host), err);
				err = -EINVAL;
				goto free_ext_csd;
			}
		}

		/* restart the eMMC */
		err = mmc_ffu_restart(card);
		if (err) {
			pr_err("FFU: %s: error %d FFU install:\n",
				mmc_hostname(card->host), err);
		}
	} else {
			ffu_data_len = ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG] |
						   ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG + 1] << 8 |
						   ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG + 2] << 16 |
						   ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG + 3] << 24;

			if (!ffu_data_len) {
				err = -EPERM;
				goto free_ext_csd;;
			}
		err = mmc_get_ext_csd(card, &ext_csd);
		if (err) {
			pr_err("FFU: %s: error %d sending ext_csd\n",
				mmc_hostname(card->host), err);
			goto exit;
		}

			/* set device to FFU mode */
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_MODE_CONFIG, 0x1,
				card->ext_csd.generic_cmd6_time);
			if (err) {
				pr_err("FFU: %s: error %d FFU is not supported\n",
					mmc_hostname(card->host), err);
				goto free_ext_csd;
			}

			timeout = ext_csd[EXT_CSD_OPERATION_CODE_TIMEOUT];
			if (timeout == 0 || timeout > 0x17) {
				timeout = 0x17;
				pr_warn("FFU: %s: operation code timeout is out "
						"of range. Using maximum timeout.\n",
					mmc_hostname(card->host));
			}

			/* timeout is at millisecond resolution */
			timeout = (100 * (1 << timeout) / 1000) + 1;
			/* set ext_csd to install mode */
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_MODE_OPERATION_CODES,
				MMC_FFU_INSTALL_SET, timeout);

			if (err) {
				pr_err("FFU: %s: error %d setting install mode\n",
					mmc_hostname(card->host), err);
				goto free_ext_csd;
			}

		}
		kfree(ext_csd);
		/* read ext_csd */
		err = mmc_get_ext_csd(card, &ext_csd);
		if (err) {
			pr_err("FFU: %s: error %d sending ext_csd\n",
				mmc_hostname(card->host), err);
			goto exit;
		}
		/* return status */
		err = ext_csd[EXT_CSD_FFU_STATUS];
		pr_err("FFU_STATUS after: %d !!!\n",ext_csd[EXT_CSD_FFU_STATUS]);
		if (err) {
			pr_err("FFU: %s: error %d FFU install:\n",
				mmc_hostname(card->host), err);
			err = -EINVAL;
			goto free_ext_csd;
		}
free_ext_csd:
	kfree(ext_csd);
exit:
	return err;
}
EXPORT_SYMBOL(mmc_ffu_install);

#ifdef CONFIG_MMC_FFU_SAMSUNG45

#define EXT_CSD_MODE_CONFIG_SAMSUNG45	133
#define EXT_CSD_FFU_STATUS_SAMSUNG45	253

int mmc_ffu_execute(struct mmc_card *card, struct mmc_command *cmd,
	u8 *data, int buf_bytes)
{
	u8 ext_csd[CARD_BLOCK_SIZE];
	int err;
	int ret = 0;

    if (card == NULL) {
        err = -EINVAL;
        pr_err("FFU: %s: argument error (card == NULL)\n", __func__);
        goto exit;
    }

    if (cmd == NULL) {
        err = -EINVAL;
        pr_err("FFU: %s: argument error (cmd == NULL)\n", __func__);
        goto exit;
    }

	/* Read the EXT_CSD */
	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		pr_err("FFU: %s: error %d sending ext_csd\n",
			mmc_hostname(card->host), err);
		goto exit;
	}

	/*
	 * Check Manufacturer ID and revision,
	 * This is only for Samsung eMMC 4.5
	 */
	if (card->cid.manfid != CID_MANFID_SAMSUNG ||
	    card->ext_csd.rev != 6) {
		err = -EINVAL;
		pr_err("FFU: %s: error %d FFU is not supported\n",
			mmc_hostname(card->host), err);
		goto exit;
	}

	/* Check if FFU is supported by card */
	if (!FFU_SUPPORTED_MODE(ext_csd[EXT_CSD_SUPPORTED_MODE])) {
		err = -EINVAL;
		pr_err("FFU: %s: error %d FFU is not supported\n",
			mmc_hostname(card->host), err);
		goto exit;
	}

	if (FFU_ENABLED(ext_csd[EXT_CSD_FW_CONFIG])) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_FW_CONFIG, MMC_FFU_ENABLE,
			card->ext_csd.generic_cmd6_time);
		if (err) {
			pr_err("%s: switch to FFU failed with error %d\n",
				mmc_hostname(card->host), err);
			return err;
		}
	}

	/* set device to FFU mode */
	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
		EXT_CSD_MODE_CONFIG_SAMSUNG45, MMC_FFU_MODE_SET,
		card->ext_csd.generic_cmd6_time);
	if (err) {
		pr_err("FFU: %s: error %d FFU is not supported\n",
			mmc_hostname(card->host), err);
		goto exit;
	}
	cmd->arg = 0xc7810000;

	err = mmc_ffu_write(card, data, cmd->arg, buf_bytes);

	/* host switch back to work in normal MMC Read/Write commands */
	ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
		EXT_CSD_MODE_CONFIG_SAMSUNG45, MMC_FFU_MODE_NORMAL,
		card->ext_csd.generic_cmd6_time);
	if (ret)
		err = ret;

	/* read ext_csd */
	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		pr_err("FFU: %s: error %d sending ext_csd\n",
		mmc_hostname(card->host), err);
		goto exit;
	}

	/* return status */
	err = ext_csd[EXT_CSD_FFU_STATUS_SAMSUNG45];
	if (!err) {
		pr_err("FFU: %s: error %d FFU execute:\n",
			mmc_hostname(card->host), err);
		err = -EINVAL;
		goto exit;
	}
	err = mmc_ffu_restart(card);
	if (err) {
		pr_err("FFU: %s: error %d FFU execute:\n",
			mmc_hostname(card->host), err);
	}
exit:
	return err;
}

EXPORT_SYMBOL(mmc_ffu_execute);

#endif

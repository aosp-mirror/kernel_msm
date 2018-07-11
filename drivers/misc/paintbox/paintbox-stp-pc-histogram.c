/*
 * STP PC Histogram support for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

struct perf_sample {
	bool enabled;
	uint64_t stat;
};

/* The caller to this function must hold pb->lock */
static void paintbox_perf_record_sample(struct paintbox_data *pb,
		uint64_t timestamp, int stp_id,
		const struct perf_sample *sample)
{
	int stp_index = stp_id_to_index(stp_id);
	struct paintbox_stp *stp = &pb->stp.stps[stp_index];

	if (!sample->enabled) {
		stp->disabled++;
		return;
	}

	if (sample->stat & STP_STAT_STALLED_MASK) {
		uint64_t pc;

		pc = ((sample->stat & STP_STAT_PC_MASK) >> STP_STAT_PC_SHIFT);

		stp->stalled[pc]++;
	} else {
		stp->running++;
	}
}

static int paintbox_perf_thread_main(void *data)
{
	struct perf_sample *samples;
	size_t samples_size;
	struct paintbox_data *pb = (struct paintbox_data *)data;

	samples_size = pb->stp.num_stps * sizeof(struct perf_sample);
	samples = kzalloc(samples_size, GFP_KERNEL);
	if (WARN(!samples, "Paintbox perf_thread can't allocate memory"))
		return -ENOMEM;

	dev_dbg(pb->dev, "Paintbox perf_thread running\n");

	while (!kthread_should_stop()) {
		uint64_t timestamp;
		uint64_t samples_gathered_mask;
		uint64_t temp_samples_mask;
		unsigned long irq_flags;
		int stp_index;

		mutex_lock(&pb->lock);

		/* gather the samples as quickly as possible in a critical
		 * section
		 */
		samples_gathered_mask = pb->perf_stp_sample_mask;

		temp_samples_mask = samples_gathered_mask;

		spin_lock_irqsave(&pb->stp.lock, irq_flags);
		for (stp_index = 0; temp_samples_mask;
				 stp_index++, temp_samples_mask >>= 1) {
			int stp_id;

			if (!(temp_samples_mask & 1))
				continue;

			/* since the enabled state is controlled completely in
			 * the driver, we can just read our local cache instead
			 * of reading back from the chip
			 */
			samples[stp_index].enabled =
					pb->stp.stps[stp_index].enabled;

			stp_id = stp_index_to_id(stp_index);
			paintbox_stp_select(pb, stp_id);

			samples[stp_index].stat = paintbox_stp_stat_read(pb);
		}
		spin_unlock_irqrestore(&pb->stp.lock, irq_flags);

		/* process the samples after the spin lock has been released. */
		timestamp = get_cycles();

		for (stp_index = 0; samples_gathered_mask;
				 stp_index++, samples_gathered_mask >>= 1) {
			if (!(samples_gathered_mask & 1))
				continue;
			paintbox_perf_record_sample(pb, timestamp,
					stp_index_to_id(stp_index),
					&samples[stp_index]);
		}

		mutex_unlock(&pb->lock);
	}

	kfree(samples);
	return 0;
}

/* The caller to this function must hold pb->lock */
void stp_pc_histogram_clear(struct paintbox_stp *stp,
		size_t inst_mem_size_in_instructions)
{
	/* If histogram recording has not been enabled then there is nothing to
	 * do here.
	 */
	if (!stp->stalled)
		return;

	stp->disabled = 0;
	stp->running = 0;

	memset(stp->stalled, 0, inst_mem_size_in_instructions *
			sizeof(stp->stalled[0]));
}

int stp_pc_histogram_clear_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long stp_mask)
{
	int stp_index;
	int ret = 0;
	struct paintbox_stp *stp;

	mutex_lock(&pb->lock);

	for (stp_index = 0; stp_mask; stp_index++, stp_mask >>= 1) {
		int stp_id;

		if ((stp_mask & 1) == 0)
			continue;

		stp_id = stp_index_to_id(stp_index);

		stp = get_stp(pb, session, stp_id, &ret);
		if (ret < 0)
			break;

		stp_pc_histogram_clear(stp,
				pb->stp.inst_mem_size_in_instructions);
	}

	mutex_unlock(&pb->lock);

	return ret;
}

/* Modifies pb->perf_stp_sample_mask and starts/stops the sampling thread as
 * needed
 *
 * The caller to this function must hold pb->lock
 */
int stp_pc_histogram_enable(struct paintbox_data *pb, uint64_t stp_mask)
{
	/* save the previous sample mask so we can act on changes */
	uint64_t prev_mask = pb->perf_stp_sample_mask;
	/* only modify status for stps this session owns */
	pb->perf_stp_sample_mask = stp_mask;

	dev_dbg(pb->dev, "%s: stp_pc_sample_mask %llx->%llx\n", __func__,
			prev_mask, pb->perf_stp_sample_mask);

	if (stp_mask != 0) {
		uint64_t mask = stp_mask;
		unsigned int stp_index;

		for (stp_index = 0; stp_index < pb->stp.num_stps && mask;
				stp_index++, mask >>= 1) {
			struct paintbox_stp *stp;
			size_t len;

			/* If this stp is not part of the measurement then skip
			 * it.
			 */
			if (!(mask & 0x01))
				continue;

			stp = &pb->stp.stps[stp_index];

			/* If this stp already has a stalled buffer then
			 * continue.
			 */
			if (stp->stalled)
				continue;

			len = pb->stp.inst_mem_size_in_instructions *
					sizeof(stp->stalled[0]);
			stp->stalled = kzalloc(len, GFP_KERNEL);
			if (!stp->stalled) {
				pr_err("Failed to allocate stalled array %zub",
						len);
				return -ENOMEM;
			}
		}
	}

	if (prev_mask == 0 && pb->perf_stp_sample_mask != 0) {
		/* start the sampling thread */
		struct task_struct *perf_thread;

		perf_thread = kthread_run(paintbox_perf_thread_main, pb,
				"paintbox_perf");
		if (IS_ERR(perf_thread)) {
			pr_err("Failed to create paintbox_perf thread");
			pb->perf_stp_sample_mask = 0;
			return PTR_ERR(perf_thread);
		}
		pb->perf_thread = perf_thread;
	} else if (prev_mask != 0 && pb->perf_stp_sample_mask == 0) {
		/* stop the sampling thread outside of the lock */
		kthread_stop(pb->perf_thread);
		pb->perf_thread = NULL;
	}
	return 0;
}

int stp_pc_histogram_enable_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long stp_mask)
{
	int ret = 0;
	int stp_index;

	/* This is a mask of stps this session owns */
	uint64_t session_stp_mask = 0;

	uint64_t new_mask;

	mutex_lock(&pb->lock);

	/* This session can only enable/disable sampling for stps it owns. Build
	 * a mask of all stps owned by this session
	 */
	for (stp_index = 0; stp_index < pb->stp.num_stps; stp_index++) {
		if (pb->stp.stps[stp_index].session == session)
			session_stp_mask |= 1 << stp_index;
	}

	if (stp_mask & ~session_stp_mask) {
		mutex_unlock(&pb->lock);
		dev_err(pb->dev,
				"%s: can't enable pc sampling for stps not allocated to session %llu\n",
				__func__, (stp_mask & ~session_stp_mask));
		return -EACCES;
	}

	new_mask = (stp_mask & session_stp_mask) |
			(pb->perf_stp_sample_mask & ~session_stp_mask);

	ret = stp_pc_histogram_enable(pb, new_mask);

	mutex_unlock(&pb->lock);

	return ret;
}

int stp_pc_histogram_read_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct stp_pc_histogram __user *user_histogram =
			(struct stp_pc_histogram __user *)arg;
	uint32_t __user *user_stalled;
	struct stp_pc_histogram histogram;
	struct paintbox_stp *stp;
	size_t stalled_size;
	int ret = 0;

	/* get stp_id from user space */
	if (copy_from_user(&histogram, user_histogram, sizeof(histogram)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	stp = get_stp(pb, session, histogram.stp_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* If histogram recording has not been previously been enabled then
	 * there is no data.
	 */
	if (!stp->stalled) {
		mutex_unlock(&pb->lock);
		return -ENODATA;
	}

	histogram.disabled = stp->disabled;
	histogram.running = stp->running;

	if (copy_to_user(user_histogram, &histogram, sizeof(histogram))) {
		mutex_unlock(&pb->lock);
		return -EFAULT;
	}

	/* We return an array of stp->inst_mem_size_in_instructions uint32_t
	 * values immediately following the struct stp_pc_histogram
	 */

	stalled_size = sizeof(stp->stalled[0]) *
			pb->stp.inst_mem_size_in_instructions;

	user_stalled = (uint32_t __user *)(user_histogram + 1);
	if (copy_to_user(user_stalled, stp->stalled, stalled_size)) {
		mutex_unlock(&pb->lock);
		return -EFAULT;
	}

	mutex_unlock(&pb->lock);

	return 0;
}


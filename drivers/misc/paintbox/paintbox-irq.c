/*
 * Interrupt support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
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

#include <linux/completion.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <uapi/paintbox.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-irq.h"
#include "paintbox-dma.h"
#include "paintbox-stp.h"

#define SESSION_WAIT_MAX_SLEEP_US 100

struct paintbox_irq_group_waiter {
	struct list_head session_entry;
	struct paintbox_irq_group_wait *wait;
	struct completion completion;
	bool priority_irq_triggered;
	int error;
};

/* The caller to this function must hold pb->irq_lock */
static void paintbox_irq_enq_pending_event(struct paintbox_data *pb,
		struct paintbox_irq *irq, struct paintbox_irq_event *event)
{
	if (irq->event_read_index == (irq->event_write_index + 1) %
			IRQ_MAX_PER_WAIT_PERIOD) {
		dev_warn_ratelimited(pb->dev, "irq event dropped\n");
		return;
	}

	memcpy(&irq->events[irq->event_write_index], event,
			sizeof(struct paintbox_irq_event));
	irq->event_write_index = (irq->event_write_index + 1) %
			IRQ_MAX_PER_WAIT_PERIOD;
}

/* The caller to this function must hold pb->irq_lock */
static bool paintbox_irq_deq_pending_event(struct paintbox_irq *irq,
		struct paintbox_irq_event *event)
{
	if (irq->event_read_index == irq->event_write_index)
		return false;

	memcpy(event, &irq->events[irq->event_read_index],
			sizeof(struct paintbox_irq_event));
	irq->event_read_index = (irq->event_read_index + 1) %
			IRQ_MAX_PER_WAIT_PERIOD;

	return true;
}

/* The caller to this function must hold pb->irq_lock */
static inline void paintbox_irq_flush_events(struct paintbox_data *pb,
		struct paintbox_irq *irq)
{
	irq->event_write_index = 0;
	irq->event_read_index = 0;
}

/* The caller to this function must hold pb->irq_lock */
static bool paintbox_irq_group_add_event(
		struct paintbox_irq_group_waiter *waiter,
		struct paintbox_irq *irq, struct paintbox_irq_event *event)
{
	struct paintbox_irq_group_wait *wait = waiter->wait;
	unsigned int index;

	for (index = 0; index < wait->base.irq_count; index++) {
		struct paintbox_irq_wait_entry *entry = &wait->irqs[index];

		if (entry->interrupt_id != irq->interrupt_id)
			continue;

		/* If this interrupt has already been reported then return
		 * false so that it will be added to the irq's pending queue.
		 */
		if (entry->triggered)
			return false;

		entry->triggered = true;
		memcpy(&entry->event, event, sizeof(struct paintbox_irq_event));

		if (event->error) {
			/* The first error in the irq group will be reported
			 * through errno.  Any additional errors will only be
			 * reported in the entry array.
			 */
			if (waiter->error == 0)
				waiter->error = event->error;
		}

		if (entry->flags & PB_IRQ_PRIORITY)
			waiter->priority_irq_triggered = true;

		if (entry->flags & PB_IRQ_REQUIRED)
			wait->base.required_irqs_triggered++;

		return true;
	}

	return false;
}

/* The caller to this function must hold pb->irq_lock */
static bool paintbox_irq_group_verify_complete(
		struct paintbox_irq_group_waiter *waiter)
{
	/* An error in the interrupt group will cause the wait to end
	 * immediately.
	 */
	if (waiter->error)
		return true;

	/* If a priority interrupt in the interrupt group has triggered then
	 * end the wait immediately.
	 */
	if (waiter->priority_irq_triggered)
		return true;

	/* If all the interrupts in the group are optional then return now,
	 * there is no need to check to see if all the required interrupts have
	 * triggered.
	 */
	if (waiter->wait->base.required_irq_count == 0)
		return false;

	if (waiter->wait->base.required_irqs_triggered ==
			waiter->wait->base.required_irq_count)
		return true;

	return false;
}

static int paintbox_irq_group_wait_dup_from_user(struct paintbox_data *pb,
		struct paintbox_irq_group_wait **out,
		struct paintbox_irq_group_wait __user *user_wait)
{
	struct paintbox_irq_group_wait_base base;
	struct paintbox_irq_group_wait *wait;
	size_t len;

	if (copy_from_user(&base, user_wait,
			sizeof(struct paintbox_irq_group_wait_base)))
		return -EFAULT;

	if (base.irq_count > pb->io.num_interrupts)
		return -ERANGE;

	len = sizeof(struct paintbox_irq_group_wait_base) + base.irq_count *
			sizeof(struct paintbox_irq_wait_entry);

	wait = kzalloc(len, GFP_KERNEL);
	if (!wait)
		return -ENOMEM;

	if (copy_from_user(wait, user_wait, len)) {
		kfree(wait);
		return -EFAULT;
	}

	*out = wait;

	return 0;
}

static int paintbox_irq_group_wait_copy_to_user(struct paintbox_data *pb,
		struct paintbox_irq_group_wait __user *user_wait,
		struct paintbox_irq_group_wait *wait)
{
	size_t len;
	int ret = 0;

	len = sizeof(struct paintbox_irq_group_wait_base) +
			wait->base.irq_count *
			sizeof(struct paintbox_irq_wait_entry);

	if (copy_to_user(user_wait, wait, len))
		ret = -EFAULT;

	kfree(wait);

	return ret;
}

/* The caller to this function must hold pb->irq_lock */
static void release_all_waiters(struct paintbox_data *pb,
		struct paintbox_irq *irq)
{
	/* walk list of waiting threads, wake any that are waiting on this
	 * interrupt
	 */
	struct paintbox_irq_group_waiter *waiter, *waiter_next;

	if (!irq || !irq->session)
		return;

	/* waiting threads have priority on getting interrupts (longest waiting
	 * first)
	 */
	list_for_each_entry_safe(waiter, waiter_next, &irq->session->wait_list,
			session_entry) {
		/* Check to see if an error has been already reported for this
		 * group.  If so then the waiting thread should have already
		 * been notified.  If it has not then set the error to -EPIPE.
		 *
		 * Note: The error is only set in the waiter context structure,
		 * it is not set in each individual irq entry.  This function is
		 * called when releasing resources so the userspace is not
		 * going to be interested in interrupt data.
		 */
		if (waiter->error)
			continue;

		waiter->error = -EPIPE;
		complete(&waiter->completion);
	}

	paintbox_irq_flush_events(pb, irq);
}

/* The caller to this function must hold pb->lock */
int validate_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id)
{

	if (interrupt_id >= pb->io.num_interrupts) {
		dev_err(pb->dev, "%s: invalid interrupt_id %d\n", __func__,
				interrupt_id);
		return -EINVAL;
	}

	if (pb->irqs[interrupt_id].session != session) {
		dev_err(pb->dev, "%s: access error: interrupt_id %d\n",
				__func__, interrupt_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_irq *get_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id,
		int *err)
{
	int ret;

	ret = validate_interrupt(pb, session, interrupt_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->irqs[interrupt_id];
}

/* The caller to this function must hold pb->lock */
int allocate_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int interrupt_id)
{
	struct paintbox_irq *irq;

	irq = &pb->irqs[interrupt_id];
	if (irq->session) {
		dev_err(pb->dev, "%s: access error: interrupt_id %d\n",
				__func__, interrupt_id);
		return -EACCES;
	}

	irq->session = session;
	list_add_tail(&irq->session_entry, &session->irq_list);
	pb->io.available_interrupt_mask &= ~(1ULL << interrupt_id);

	return 0;
}

int allocate_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;
	unsigned int interrupt_id = (unsigned int)arg;

	if (interrupt_id >= pb->io.num_interrupts) {
		dev_err(pb->dev,
				"%s: invalid interrupt_id %d, %d >= %d\n",
				__func__, interrupt_id, interrupt_id,
				pb->io.num_interrupts);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	ret = allocate_interrupt(pb, session, interrupt_id);
	if (ret < 0)
		dev_err(pb->dev,
				"%s: allocate interrupt_id %d error %d\n",
				__func__, interrupt_id, ret);
	mutex_unlock(&pb->lock);

	return ret;
}

int paintbox_flush_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int interrupt_id = (unsigned int)arg;
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret;

	mutex_lock(&pb->lock);

	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	paintbox_irq_flush_events(pb, irq);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int paintbox_flush_all_interrupts_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_irq *irq, *irq_next;
	unsigned long irq_flags;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	list_for_each_entry_safe(irq, irq_next, &session->irq_list,
			session_entry)
		paintbox_irq_flush_events(pb, irq);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
int bind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		unsigned int interrupt_id)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret;

	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	if (channel->irq) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		dev_err(pb->dev,
				"%s: channel%u already bound to int%u\n",
				__func__, channel->channel_id,
				channel->irq->interrupt_id);
		return -EBUSY;
	}

	if (irq->source != IRQ_SRC_NONE) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		dev_err(pb->dev,
				"%s: dma channel%u unable to bind int%u\n",
				__func__, channel->channel_id, interrupt_id);
		return -EEXIST;
	}

	irq->dma_channel = channel;
	irq->source = IRQ_SRC_DMA_CHANNEL;
	channel->irq = irq;

	paintbox_irq_flush_events(pb, irq);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

/* The caller to this function must hold pb->lock */
int unbind_dma_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	irq = channel->irq;
	if (!irq) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return -ENOENT;
	}

	release_all_waiters(pb, irq);

	channel->irq = NULL;
	irq->dma_channel = NULL;
	irq->source = IRQ_SRC_NONE;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

/* The caller to this function must hold pb->lock */
int bind_stp_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp,
		unsigned int interrupt_id)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;
	int ret;

	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	if (stp->irq) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		dev_err(pb->dev, "%s: stp%u already bound to int%u\n", __func__,
				stp->stp_id, stp->irq->interrupt_id);
		return -EBUSY;
	}

	if (irq->source != IRQ_SRC_NONE) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		dev_err(pb->dev, "%s: stp%u unable to bind int%u\n", __func__,
				stp->stp_id, interrupt_id);
		return -EEXIST;
	}

	irq->stp = stp;
	irq->source = IRQ_SRC_STP;
	stp->irq = irq;

	paintbox_irq_flush_events(pb, irq);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

/* The caller to this function must hold pb->lock */
int unbind_stp_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_stp *stp)
{
	struct paintbox_irq *irq;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	irq = stp->irq;
	if (!irq) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return -ENOENT;
	}

	release_all_waiters(pb, irq);

	stp->irq = NULL;
	irq->stp = NULL;
	irq->source = IRQ_SRC_NONE;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);

	return 0;
}

int wait_for_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_irq_group_wait __user *user_wait;
	struct paintbox_irq_group_wait *wait = NULL;
	struct paintbox_irq_group_waiter waiter;
	unsigned long irq_flags;
	unsigned long start = jiffies;
	int copy_ret, ret;
	unsigned int irq_index;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;

	start_time = pb->stats.ioctl_time_enabled ? ktime_get_boottime() :
			ktime_set(0, 0);
#endif

	user_wait = (struct paintbox_irq_group_wait __user *)arg;

	ret = paintbox_irq_group_wait_dup_from_user(pb, &wait, user_wait);
	if (ret < 0)
		return ret;

	memset(&waiter, 0, sizeof(waiter));
	waiter.wait = wait;

	dev_dbg(pb->dev, "waiting for %u irqs timeout %lld",
			wait->base.irq_count, wait->base.timeout_ns);

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	/* It shouldn't be possible to enter this ioctl while a release is
	 * occuring but check and warn if does happen.
	 */
	if (WARN_ON(session->releasing)) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		mutex_unlock(&pb->lock);
		kfree(wait);
		return -EPIPE;
	}

	/* Check each requested interrupt in the group and make sure that the
	 * session has access to it and check whether the interrupt has
	 * already occurred.
	 */
	for (irq_index = 0; irq_index < wait->base.irq_count; irq_index++) {
		struct paintbox_irq_wait_entry *entry = &wait->irqs[irq_index];
		struct paintbox_irq_event event;
		struct paintbox_irq *irq;
		int err;

		irq = get_interrupt(pb, session, entry->interrupt_id, &err);
		if (err < 0) {
			/* This interrupt is not allocated to the session,
			 * mark it as an error and return.
			 */
			entry->event.error = err;

			if (waiter.error == 0)
				waiter.error = err;

			/* TODO, I am not sure we necessarily want to break on
			 * the first error.  It might be better to collect all
			 * the IRQ state and then return.
			 */
			break;
		}

		if (paintbox_irq_deq_pending_event(irq, &event)) {
			bool handled = paintbox_irq_group_add_event(&waiter,
					irq, &event);
			WARN_ON(!handled);
		}
	}

	/* If the completion criteria for the irq group has been met then clean
	 * up and exit.
	 */
	if (paintbox_irq_group_verify_complete(&waiter)) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		mutex_unlock(&pb->lock);

		ret = waiter.error;
		goto cleanup_and_exit;
	}

	/* add this thread to the end of the list of threads waiting for
	 * interrupts
	 */
	init_completion(&waiter.completion);
	list_add_tail(&waiter.session_entry, &session->wait_list);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
	mutex_unlock(&pb->lock);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb,  PB_STATS_WAIT_PRE,
				start_time, ktime_get_boottime(), 0);
#endif

	if (wait->base.timeout_ns == LONG_MAX)
		ret = wait_for_completion_interruptible(&waiter.completion);
	else {
		long time_remaining;

		/* wait_for_completion_interruptible_timeout returns
		 * -ERESTARTSYS if it receives a signal.  Otherwise it returns
		 * the number of jiffies until the timeout.  So 0 means it timed
		 * out
		 */
		time_remaining = wait_for_completion_interruptible_timeout(
				&waiter.completion,
				nsecs_to_jiffies64(wait->base.timeout_ns));
		if (time_remaining == 0) {
			wait->base.timeout_ns = 0;
			ret = -ETIMEDOUT;
		} else if (time_remaining < 0) {
			unsigned long elapsed = jiffies - start;
			int64_t elapsed_ns = jiffies_to_nsecs(elapsed);

			if (elapsed_ns >= wait->base.timeout_ns)
				wait->base.timeout_ns = 0;
			else
				wait->base.timeout_ns -= elapsed_ns;
			ret = time_remaining; /* -ERESTARTSYS */
		} else {
			/* time_remaining > 0
			 * completion was signaled before timeout
			 */
			wait->base.timeout_ns =
					jiffies_to_nsecs(time_remaining);
			ret = 0;
		}
	}

#ifdef CONFIG_PAINTBOX_DEBUG
	start_time = ktime_get_boottime();
#endif

	mutex_lock(&pb->lock);
	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	/* remove this thread from the waiting thread list */
	list_del(&waiter.session_entry);

	/* If we are releasing the session or shutting down then notify the
	 * release/shutdown code that the release is complete.
	 */
	if (session->releasing)
		complete(&session->release_completion);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
	mutex_unlock(&pb->lock);

	/* Verify that the completion criteria for the irq group has been met.
	 */
	if (paintbox_irq_group_verify_complete(&waiter)) {
		/* Interrupt errors take precedence over timeout errors. */
		ret = waiter.error ? waiter.error : ret;
	}

cleanup_and_exit:
	copy_ret = paintbox_irq_group_wait_copy_to_user(pb, user_wait, wait);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb,  PB_STATS_WAIT_POST,
				start_time, ktime_get_boottime(), 0);
#endif

	/* Copy errors take precedence over any other errors. */
	return copy_ret ? copy_ret : ret;
}

/* Must be called with interrupts disabled */
void paintbox_irq_waiter_signal(struct paintbox_data *pb,
		struct paintbox_irq *irq, ktime_t timestamp, uint16_t data,
		int error)
{
	struct paintbox_irq_group_waiter *waiter, *waiter_next;
	struct paintbox_irq_event event;

	event.timestamp_ns = ktime_to_ns(timestamp);
	event.data = data;
	event.error = error;

	spin_lock(&pb->irq_lock);

	/* why not check IRQ pointer against null and return before locking?
	 * Probably not a problem on Easel but on a future SMP system it may be
	 * possible for one CPU to be processing an interrupt while another CPU
	 * is in the process of releasing the IRQ object.
	 */
	if (!irq) {
		spin_unlock(&pb->irq_lock);
		return;
	}

	/* Waiting threads have priority on getting interrupts (longest waiting
	 * first)
	 */
	list_for_each_entry_safe(waiter, waiter_next, &irq->session->wait_list,
			session_entry) {
		if (paintbox_irq_group_add_event(waiter, irq, &event)) {
			/* Check to see if the completion criteria for the irq
			 * group has been met, if so then signal the waiting
			 * thread.
			 */
			if (paintbox_irq_group_verify_complete(waiter))
				complete(&waiter->completion);

			/* The interrupt has been claimed, return. */
			spin_unlock(&pb->irq_lock);
			return;
		}
	}

	/* If the interrupt is not claimed then store the interrupt in the
	 * pending queue.
	 */
	paintbox_irq_enq_pending_event(pb, irq, &event);

	spin_unlock(&pb->irq_lock);
}

/* The caller to this function must hold pb->lock */
int release_interrupt(struct paintbox_data *pb,
		struct paintbox_session *session, struct paintbox_irq *irq)
{
	switch (irq->source) {
	case IRQ_SRC_NONE:
		break;
	case IRQ_SRC_DMA_CHANNEL:
		unbind_dma_interrupt(pb, session, irq->dma_channel);
		break;
	case IRQ_SRC_STP:
		unbind_stp_interrupt(pb, session, irq->stp);
		break;
	default:
		dev_err(pb->dev, "%s: invalid interrupt source\n", __func__);
		return -EINVAL;
	};

	list_del(&irq->session_entry);
	irq->session = NULL;
	irq->source = IRQ_SRC_NONE;
	pb->io.available_interrupt_mask |= 1ULL << irq->interrupt_id;

	return 0;
}

int release_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int interrupt_id = (unsigned int)arg;
	struct paintbox_irq *irq;
	int ret;

	mutex_lock(&pb->lock);
	irq = get_interrupt(pb, session, interrupt_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = release_interrupt(pb, session, irq);
	signal_completion_on_first_alloc_waiter(pb);

	mutex_unlock(&pb->lock);

	return ret;
}

int paintbox_irq_init(struct paintbox_data *pb)
{
	unsigned int interrupt_id;

	spin_lock_init(&pb->irq_lock);

	pb->irqs = kcalloc(pb->io.num_interrupts, sizeof(struct paintbox_irq),
			GFP_KERNEL);
	if (!pb->irqs)
		return -ENOMEM;

	for (interrupt_id = 0; interrupt_id < pb->io.num_interrupts;
			interrupt_id++) {
		/* Store interrupt id with object as a convenience to avoid
		 * doing a lookup later on.
		 */
		pb->irqs[interrupt_id].interrupt_id = interrupt_id;
		pb->irqs[interrupt_id].source = IRQ_SRC_NONE;
	}

	return 0;
}

void paintbox_irq_release_all_waiters(struct paintbox_data *pb)
{
	unsigned long irq_flags;
	unsigned int interrupt_id;

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	for (interrupt_id = 0; interrupt_id < pb->io.num_interrupts;
			interrupt_id++)
		release_all_waiters(pb, &pb->irqs[interrupt_id]);

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
void paintbox_irq_wait_for_release_complete(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	unsigned long timeout, irq_flags;

	timeout = usecs_to_jiffies(SESSION_WAIT_MAX_SLEEP_US);

	spin_lock_irqsave(&pb->irq_lock, irq_flags);

	if (list_empty(&session->wait_list)) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return;
	}

	/* It shouldn't be possible to call release() twice so there should not
	 * be an in progress release.
	 */
	if (WARN_ON(session->releasing)) {
		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		return;
	}

	session->releasing = true;

	reinit_completion(&session->release_completion);

	while (!list_empty(&session->wait_list)) {
		unsigned int ret;

		spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
		mutex_unlock(&pb->lock);

		ret = wait_for_completion_interruptible_timeout(
				&session->release_completion, timeout);

		mutex_lock(&pb->lock);

		if (ret == 0) {
			dev_err(pb->dev, "%s: timeout waiting for release\n",
					__func__);
			return;
		}

		spin_lock_irqsave(&pb->irq_lock, irq_flags);
	}

	session->releasing = false;

	spin_unlock_irqrestore(&pb->irq_lock, irq_flags);
}

void paintbox_irq_remove(struct paintbox_data *pb)
{
	unsigned int interrupt_id;

	paintbox_irq_release_all_waiters(pb);

	mutex_lock(&pb->lock);

	for (interrupt_id = 0; interrupt_id < pb->io.num_interrupts;
			interrupt_id++) {
		struct paintbox_irq *irq = &pb->irqs[interrupt_id];

		if (irq->session)
			paintbox_irq_wait_for_release_complete(pb,
					irq->session);
	}

	mutex_unlock(&pb->lock);

	kfree(pb->irqs);
}

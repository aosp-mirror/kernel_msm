/*
 *  service.c - part of servicefs, a small filesystem for service namespaces.
 *
 *  Copyright (C) 2015 Corey Tabaka <eieio@google.com>
 *  Copyright (C) 2015 Google, Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/file.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/poison.h>
#include <linux/poll.h>
#include <linux/namei.h>
#include <linux/atomic.h>
#include <uapi/linux/servicefs.h>

#include "servicefs_private.h"

/*
 * ServiceFS uses a careful locking strategy to maximize concurrency.
 * These are the key points of the strategy:
 *    1. DO NOT introduce any new global locks to the system.
 *    2. Use short non-preemptible critical sections, where possible, to avoid
 *       priority inversion issues and reschedules in performance critical
 *       regions.
 *    3. Use read/write locks to improve concurrency of RO operations.
 *    4. Use sleep as an operation not as a consequence of concurrency.
 *
 * The rwlock s_message_lock protects most members of struct service, except for
 * the channel list and channel id sweeper. This lock orders access to the
 * message queue and service context pointer.
 *
 * The rwlock s_channel_lock protects the channel list, channel id sweeper, and
 * orders writes to per-channel context pointers. Reads from per-channel context
 * pointers are not synchronized with respect to writes and should be
 * synchronized in userspace in multithreaded services. This is deemed
 * acceptable since changing context pointers requires userspace
 * synchronization between setting the pointer and receiving messages,
 * regardless of whether the kernel synchronizes reads and updates internally.
 *
 * The mutex m_mutex protects most per-message members of struct message. This
 * lock serializes access to the message payload iovecs and pending fd list.
 */

#define THIS_SERVICE_FD (-1)

#define SERVICEFS_TRACE 0

#if SERVICEFS_TRACE
#define TRACE(fmt, args...) pr_err(fmt, ##args)
#else
#define TRACE(fmt, args...) pr_debug(fmt, ##args)
#endif

/**
 * servicefs_atomic_update_mask - update a mask in an atomic variable
 * @v: pointer to atomic variable to update
 * @clr: bits to clear in the atomic variable
 * @set: bits to set in the atomic variable
 *
 * Atomically updates the bits in an atomic variable by first clearing
 * the bits given by |clr| and then setting the bits given by |set|.
 *
 * Returns the previous value of the mask.
 */
static inline int servicefs_atomic_update_mask(atomic_t *v, int clr, int set)
{
	int c, old;

	c = atomic_read(v);
	while ((old = atomic_cmpxchg(v, c, (c & ~clr) | set)) != c)
		c = old;

	return old;
}

#define servicefs_atomic_mask_set(v, set) \
	servicefs_atomic_update_mask((v), 0, (set))

#define servicefs_atomic_mask_clear(v, clr) \
	servicefs_atomic_update_mask((v), (clr), 0)

/*
 * Usage of struct service.s_flags:
 * bit 0:    service canceled bit (1=canceled)
 * bit 31-1: threads in queue semaphore
 *
 * The thread count is used during shutdown to wake all the threads blocked
 * in the queue semaphore. Combinding these into a single atomic variable
 * prevents races between shutdown and entering the queue semaphore.
 */
static inline bool servicefs_queue_enter(struct service *svc)
{
	int c, old;
	const int canceled = SERVICE_FLAGS_CANCELED;
	const int inc = SERVICE_FLAGS_COUNTER;
	atomic_t *v = &svc->s_flags;

	c = atomic_read(v);
	while (!(c & canceled) && (old = atomic_cmpxchg(v, c, c + inc)) != c)
		c = old;

	return (c & canceled) == 0;
}

static inline void servicefs_queue_exit(struct service *svc)
{
	atomic_sub(SERVICE_FLAGS_COUNTER, &svc->s_flags);
}

static inline int servicefs_queue_count(struct service *svc)
{
	return atomic_read(&svc->s_flags) >> SERVICE_FLAGS_COUNTER_OFFSET;
}

static inline bool is_close_message(struct message *m)
{
	return m->m_task == NULL && m->m_op == SERVICEFS_OP_UNIX_CLOSE;
}

/**
 * servicefs_service_cache - cache for service structs.
 */
static struct kmem_cache *servicefs_service_cache;

/**
 * servicefs_channel_cache - cache for channel structs.
 */
static struct kmem_cache *servicefs_channel_cache;

/**
 * servicefs_message_cache - cache for message structs.
 */
static struct kmem_cache *servicefs_message_cache;

/**
 * servicefs_impulse_cache - cache for impulse structs.
 */
static struct kmem_cache *servicefs_impulse_cache;

/**
 * servicefs_pending_fd_cache - cache for pending_fd structs.
 */
static struct kmem_cache *servicefs_pending_fd_cache;

int servicefs_cache_init(void)
{
	unsigned long flags = 0;
#ifdef CONFIG_SERVICEFS_USE_POISON
	flags |= SLAB_POISON;
#endif

	servicefs_service_cache = KMEM_CACHE(service, flags);
	if (!servicefs_service_cache)
		goto error;

	servicefs_channel_cache = KMEM_CACHE(channel, flags);
	if (!servicefs_channel_cache)
		goto error;

	servicefs_message_cache = KMEM_CACHE(message, flags);
	if (!servicefs_message_cache)
		goto error;

	servicefs_impulse_cache = KMEM_CACHE(impulse, flags);
	if (!servicefs_impulse_cache)
		goto error;

	servicefs_pending_fd_cache = KMEM_CACHE(pending_fd, flags);
	if (!servicefs_pending_fd_cache)
		goto error;

	return 0;

error:
	if (servicefs_service_cache) {
		kmem_cache_destroy(servicefs_service_cache);
		servicefs_service_cache = NULL;
	}
	if (servicefs_channel_cache) {
		kmem_cache_destroy(servicefs_channel_cache);
		servicefs_channel_cache = NULL;
	}
	if (servicefs_message_cache) {
		kmem_cache_destroy(servicefs_message_cache);
		servicefs_message_cache = NULL;
	}
	if (servicefs_impulse_cache) {
		kmem_cache_destroy(servicefs_impulse_cache);
		servicefs_impulse_cache = NULL;
	}
	if (servicefs_pending_fd_cache) {
		kmem_cache_destroy(servicefs_pending_fd_cache);
		servicefs_pending_fd_cache = NULL;
	}

	return -ENOMEM;
}

/*
 * Forward declarations.
 */
static struct message *message_new(struct service *svc, struct channel *c,
		int op, const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt, const __s32 *fds, size_t fdcnt,
		struct task_struct *task);
static void __message_get(struct message *msg);
static void __message_put(struct message *msg);
static void message_put(struct service *svc, struct message *msg);
static void __message_cancel(struct message *msg);
static void message_put_complete(struct message *msg, int retcode);
static void channel_put(struct channel *c);
static bool channel_cancel(struct channel *c);

static void channel_free(struct channel *c)
{
	kmem_cache_free(servicefs_channel_cache, c);
}

static void message_free(struct message *m)
{
	TRACE("free: msg=%p m_op=%d cid=%d\n", m, m->m_op, m->m_channel->c_id);

	if (m->m_task)
		put_task_struct(m->m_task);
	channel_put(m->m_channel);
	kmem_cache_free(servicefs_message_cache, m);
}

static void impulse_free(struct impulse *i)
{
	TRACE("free: i=%p cid=%d\n", i, i->i_channel->c_id);

	channel_put(i->i_channel);
	kmem_cache_free(servicefs_impulse_cache, i);
}

static void pending_fd_free(struct pending_fd *pfd)
{
	kmem_cache_free(servicefs_pending_fd_cache, pfd);
}

/**
 * service_free - free an unused service struct
 * @svc: pointer to the service struct to free
 *
 * Performs sanity checks to make sure the service is properly torn down before
 * freeing its memory.
 */
void service_free(struct service *svc)
{
	struct message *msg;
	int mid;

	TRACE("freeing service: svc=%p\n", svc);

	/* make sure everything is already properly cleaned up */
	BUG_ON(!list_empty(&svc->s_impulses));
	BUG_ON(!(atomic_read(&svc->s_flags) & SERVICE_FLAGS_CANCELED));
	BUG_ON(waitqueue_active(&svc->s_wqselect));

	/* Clean up received messages. There should be no waiters blocked by now. */
	idr_for_each_entry(&svc->s_message_idr, msg, mid) {
		TRACE("cleaning up message: msg=%p m_id=%d m_op=%d\n",
				msg, msg->m_id, msg->m_op);
		__message_get(msg);
		message_put_complete(msg, -ESHUTDOWN);
	}

	BUG_ON(!list_empty(&svc->s_messages));

	/* free allocators */
	idr_destroy(&svc->s_channel_idr);
	idr_destroy(&svc->s_message_idr);

	kmem_cache_free(servicefs_service_cache, svc);
}

/**
 * service_new - allocate and initialize a new service
 *
 * Returns a pointer to the new service.
 */
struct service *service_new(void)
{
	struct service *svc = kmem_cache_alloc(servicefs_service_cache, GFP_KERNEL);
	if (!svc) {
		pr_err("failed to allocate service struct\n");
		goto out;
	}

	TRACE("creating new service: svc=%p\n", svc);

	svc->s_count = (atomic_t) ATOMIC_INIT(0);
	svc->s_flags = (atomic_t) ATOMIC_INIT(SERVICE_FLAGS_DEFAULT);

	rwlock_init(&svc->s_message_lock);
	sema_init(&svc->s_queue_messages, 0);
	rwlock_init(&svc->s_channel_lock);

	idr_init(&svc->s_message_idr);
	idr_init(&svc->s_channel_idr);

	// Disable RCU free.
	svc->s_message_idr.no_rcu_free = true;
	svc->s_channel_idr.no_rcu_free = true;

	svc->s_message_start = 0;
	svc->s_channel_start = 0;

	INIT_LIST_HEAD(&svc->s_impulses);
	INIT_LIST_HEAD(&svc->s_messages);

	init_waitqueue_head(&svc->s_wqselect);

	svc->s_context = 0;
	svc->s_filp = NULL;

out:
	return svc;
}

int service_cancel(struct service *svc)
{
	int ret;
	struct channel *c;
	struct impulse *i, *in;
	struct message *m;
	int cid, mid;
	int previous_mask;
	int count;

	previous_mask = servicefs_atomic_mask_set(&svc->s_flags,
			SERVICE_FLAGS_CANCELED);

	if ((previous_mask & SERVICE_FLAGS_CANCELED) == 0) {
		TRACE("canceling service=%p\n", svc);

		TRACE("removing dentry service=%p\n", svc);
		servicefs_remove_dentry(svc->s_filp->f_path.dentry);

		/* Wake up all the queue waiters. */
		count = servicefs_queue_count(svc);
		while (count--)
			up(&svc->s_queue_messages);

		wake_up_poll(&svc->s_wqselect, POLLHUP | POLLFREE);

		write_lock(&svc->s_message_lock);
		/* Free pending impulses. */
		list_for_each_entry_safe(i, in, &svc->s_impulses, i_impulses_node) {
			list_del_init(&i->i_impulses_node);
			impulse_free(i);
		}
		/* Cancel received messages. Final refs are released by service_free. */
		idr_for_each_entry(&svc->s_message_idr, m, mid) {
			__message_cancel(m);
		}
		write_unlock(&svc->s_message_lock);

		write_lock(&svc->s_channel_lock);
		idr_for_each_entry(&svc->s_channel_idr, c, cid) {
			channel_cancel(c);
		}
		write_unlock(&svc->s_channel_lock);

		ret = 0;
	} else {
		TRACE("already canceled service=%p\n", svc);
		ret = -ESHUTDOWN;
	}

	return ret;
}

/**
 * alloc_channel_id - allocate an unused channel id for a channel
 * @c: a pointer to the channel to associate the id with.
 *
 * This function allocates an id for a channel on its service's channel id
 * allocator. The given channel's c_id member is filled in with the id, and
 * the channel pointer is associated with the id, for future lookup.
 *
 * The c_service member of the channel struct must be set to the associated
 * service pointer prior to calling this function.
 */
static int alloc_channel_id(struct channel *c)
{
	int ret;
	struct service *svc = c->c_service;
	BUG_ON(svc == NULL);

	idr_preload(GFP_KERNEL);
	write_lock(&svc->s_channel_lock);

	ret = idr_alloc(&svc->s_channel_idr, c, svc->s_channel_start, -1, GFP_NOWAIT);
	if (ret < 0) {
		write_unlock(&svc->s_channel_lock);
		idr_preload_end();
		return ret;
	} else {
		svc->s_channel_start = (svc->s_channel_start + 1) & 0x7fffffff;
	}

	c->c_id = ret;
	write_unlock(&svc->s_channel_lock);
	idr_preload_end();
	return 0;
}

/**
 * free_channel_id - free the channel id for a channel
 * @c: a pointer to the channel to free the id for.
 */
static void free_channel_id(struct channel *c)
{
	struct service *svc = c->c_service;
	BUG_ON(svc == NULL);

	write_lock(&svc->s_channel_lock);
	idr_remove(&svc->s_channel_idr, c->c_id);
	write_unlock(&svc->s_channel_lock);
}

/**
 * __alloc_message_id - allocates an unused message for a message
 * @svc: pointer to the service to allocate the message id on.
 * @m: pointer to the message to associate the id with.
 *
 * This function allocates an id for a message on its service's message id
 * allocator. The given message's m_id member is filled in with the id and
 * the message pointer is associated with the id for future lookup.
 *
 * The message's m_channel member must be set before calling this function.
 *
 * This function must be called with the service's s_message_lock held.
 */
static int __alloc_message_id(struct message *m)
{
	int ret;
	struct service *svc = m->m_channel->c_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_message_lock);

	ret = idr_alloc(&svc->s_message_idr, m, svc->s_message_start, -1, GFP_NOWAIT);
	if (ret < 0)
		return ret;
	else
		svc->s_message_start = (svc->s_message_start + 1) & 0x7fffffff;

	m->m_id = ret;
	return 0;
}

/**
 * __free_message_id - free the message id for a message
 * @m: a pointer to the message to free the id for.
 *
 * This function must be called with the service's s_message_lock held.
 */
static void __free_message_id(struct message *m)
{
	struct service *svc = m->m_channel->c_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_message_lock);

	if (m->m_id != MESSAGE_NO_ID) {
		idr_remove(&svc->s_message_idr, m->m_id);
		m->m_id = MESSAGE_NO_ID;
	}
}

/** __enqueue_item - enqueue an item on the given queue.
 * The caller must hold s_message_lock.
 */
static inline void __enqueue_item(struct service *svc,
		struct list_head *list, struct list_head *node)
{
	pr_debug("svc=%p\n", svc);

	list_add_tail(node, list);
	up(&svc->s_queue_messages);
	wake_up_poll(&svc->s_wqselect, POLLIN | POLLRDNORM);
}

static inline int enqueue_impulse(struct service *svc,
		struct channel *c, struct impulse *impulse)
{
	write_lock(&svc->s_message_lock);

	if (is_channel_canceled(c) || is_service_canceled(svc)) {
		TRACE("cid=%d channel_canceled=%d service_canceled=%d\n",
				c->c_id, is_channel_canceled(c), is_service_canceled(svc));
		write_unlock(&svc->s_message_lock);
		impulse_free(impulse);
		return -ESHUTDOWN;
	}

	__enqueue_item(svc, &svc->s_impulses, &impulse->i_impulses_node);

	write_unlock(&svc->s_message_lock);
	return 0;
}

static inline int enqueue_message(struct service *svc,
		struct channel *c, struct message *msg)
{
	int ret;

	idr_preload(GFP_KERNEL);
	write_lock(&svc->s_message_lock);

	if ((is_channel_canceled(c) && !is_close_message(msg))
			|| is_service_canceled(svc)) {
		TRACE("cid=%d channel_canceled=%d service_canceled=%d\n",
				c->c_id, is_channel_canceled(c), is_service_canceled(svc));
		ret = -ESHUTDOWN;
		goto error;
	}

	ret = __alloc_message_id(msg);
	if (ret) {
		pr_err("Failed to alloc message id: %d\n", ret);
		goto error;
	}

	__enqueue_item(svc, &svc->s_messages, &msg->m_messages_node);

error:
	if (ret)
		__message_put(msg);
	TRACE("enqueue: svc=%p m_id=%d m_op=%d msg->m_ref.count=%d\n",
			svc, msg->m_id, msg->m_op, atomic_read(&msg->m_ref.refcount));
	write_unlock(&svc->s_message_lock);
	idr_preload_end();
	return ret;
}

/**
 * channel_new - allocates a new channel to a service
 * @svc: a pointer to the service to allocate the channel for.
 *
 * This function allocates a new channel struct and associates it with the
 * given service. It handles allocating an id for the channel and adding
 * it to the list of open channels.
 */
struct channel *channel_new(struct service *svc)
{
	int ret;
	struct channel *c = kmem_cache_alloc(servicefs_channel_cache, GFP_KERNEL);
	if (!c)
		return NULL;

	kref_init(&c->c_ref);
	c->c_service = svc;
	c->c_context = 0;
	c->c_flags = (atomic_t) ATOMIC_INIT(0);
	c->c_events = (atomic_t) ATOMIC_INIT(0);
	init_waitqueue_head(&c->c_waitqueue);

	ret = alloc_channel_id(c);
	if (ret) {
		channel_free(c);
		return NULL;
	} else {
		TRACE("added channel id=%d to service=%p\n", c->c_id, svc);
		return c;
	}
}

static void channel_release_kref(struct kref *ref)
{
	struct channel *c = container_of(ref, struct channel, c_ref);

	pr_debug("c=%p c_id=%d\n", c, c->c_id);

	free_channel_id(c);
	BUG_ON(waitqueue_active(&c->c_waitqueue));
	channel_free(c);
}

static void channel_put(struct channel *c)
{
	kref_put(&c->c_ref, channel_release_kref);
}

static bool channel_get(struct channel *c)
{
	return !!kref_get_unless_zero(&c->c_ref);
}

/**
 * channel_cancel - cancel a channel and wake up clients
 * @c: pointer to the channel to cancel
 *
 * Puts the channel into canceled state and wakes up any
 * clients that are blocked waiting for channel events.
 * POLLHUP is guaranteed by the kernel to always be in the
 * client event set, and is used to signal that the file
 * descriptor is in a canceled state.
 *
 * Returns true if this call canceled the channel, false if the
 * channel was already canceled.
 */
static bool channel_cancel(struct channel *c)
{
	int previous_mask;
	struct service *svc = c->c_service;
	BUG_ON(!svc);

	previous_mask = servicefs_atomic_mask_set(&c->c_flags,
			CHANNEL_FLAGS_CANCELED);

	if ((previous_mask & CHANNEL_FLAGS_CANCELED) == 0) {
		servicefs_atomic_mask_set(&c->c_events, POLLHUP | POLLFREE);
		wake_up_poll(&c->c_waitqueue, POLLHUP | POLLFREE);
		return true;
	} else {
		return false;
	}
}

/*
 * channel_remove - remove a channel from a service
 * @c: pointer to the channel to remove from its service
 *
 * This should only be called by client_release() when the
 * file object associated with this channel is being
 * released.
 */
void channel_remove(struct channel *c)
{
	int ret;
	struct service *svc = c->c_service;
	BUG_ON(!svc);

	TRACE("removing channel id=%d from service=%p\n", c->c_id, c->c_service);

	if (channel_cancel(c)) {
		/*
		 * Generate a special close message to signal to the service that the
		 * channel has closed. This message is only sent if the channel is not
		 * already canceled by the service explicitly closing it or the service
		 * shutting down.
		 */
		struct message *msg = message_new(svc, c, SERVICEFS_OP_UNIX_CLOSE,
				NULL, 0, NULL, 0, NULL, 0, NULL);
		if (!msg) {
			pr_err("Failed to allocate close message!\n");
			goto done;
		}

		ret = enqueue_message(svc, c, msg);
		if (ret)
			TRACE("Failed to enqueue close message: %d\n", ret);
	}

done:
	/* Drop the channel ref. The close message maintains its own ref. */
	channel_put(c);
}

/**
 * __message_cancel - cancel a pending message
 * @msg: pointer to the message to cancel
 *
 * Must be called with the service's s_message_lock held.
 */
static void __message_cancel(struct message *msg)
{
	TRACE("cancel: msg=%p\n", msg);
	msg->m_status = -ESHUTDOWN;
	servicefs_atomic_mask_set(&msg->m_flags,
			MESSAGE_FLAGS_COMPLETED | MESSAGE_FLAGS_CANCELED);
	wake_up(&msg->m_waitqueue);
}

static struct channel *__lookup_channel(struct service *svc, int cid)
{
	return idr_find(&svc->s_channel_idr, cid);
}

static struct message *message_new(struct service *svc, struct channel *c,
		int op, const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt, const __s32 *fds, size_t fdcnt,
		struct task_struct *task)
{
	kuid_t kuid;
	kgid_t kgid;
	struct message *msg = kmem_cache_alloc(
			servicefs_message_cache, GFP_KERNEL);
	if (!msg)
		return NULL;

	INIT_LIST_HEAD(&msg->m_messages_node);
	INIT_LIST_HEAD(&msg->m_pending_fds);
	kref_init(&msg->m_ref);
	mutex_init(&msg->m_mutex);

	msg->m_id = MESSAGE_NO_ID;
	msg->m_op = op;
	msg->m_flags = (atomic_t) ATOMIC_INIT(MESSAGE_FLAGS_PENDING_RECEIVE);

	init_waitqueue_head(&msg->m_waitqueue);

	/*
	 * Get a ref to the channel. This ref is dropped when the message is freed.
	 */
	channel_get(c);
	msg->m_channel = c;
	msg->m_priority = task ? task_nice(task) : -1;
	msg->m_status = 0;

	iov_buffer_uinit(&msg->m_sbuf, svec, scnt);
	iov_buffer_uinit(&msg->m_rbuf, rvec, rcnt);

	msg->m_fds = fds;
	msg->m_fdcnt = fdcnt;

	/*
	 * Get a ref to the task and stash the task struct for later use. The task
	 * struct ref is dropped when the message is freed.
	 */
	if (task)
		get_task_struct(task);
	msg->m_task = task;

	/* Credentials always come from the current task. */
	current_euid_egid(&kuid, &kgid);
	msg->m_euid = __kuid_val(kuid);
	msg->m_egid = __kgid_val(kgid);
	msg->m_pid = pid_vnr(task_tgid(current));
	msg->m_tid = pid_vnr(task_pid(current));

	return msg;
}

static struct message *__lookup_message(struct service *svc, int mid)
{
	return idr_find(&svc->s_message_idr, mid);
}

/**
 * lookup_and_lock_message - looks up a message
 * @svc: pointer to the service in which to look up the message
 * @mid: message id of the message to lookup
 * @check_interrupted: whether to check the interrupted status of the message
 */
static struct message *lookup_get_message(struct service *svc, int mid,
		bool check_interrupted)
{
	struct message *msg;

	read_lock(&svc->s_message_lock);

	msg = __lookup_message(svc, mid);
	if (!msg) {
		read_unlock(&svc->s_message_lock);
		return ERR_PTR(-ENOENT);
	}

	if (check_interrupted && is_message_interrupted(msg)) {
		read_unlock(&svc->s_message_lock);
		return ERR_PTR(-EINTR);
	}

	__message_get(msg);
	read_unlock(&svc->s_message_lock);

	return msg;
}

static struct message *lookup_and_lock_message(struct service *svc, int mid,
		bool check_interrupted)
{
	struct message *msg = lookup_get_message(svc, mid, check_interrupted);
	if (IS_ERR(msg))
		return msg;

	mutex_lock(&msg->m_mutex);
	return msg;
}

static void unlock_and_put_message(struct service *svc, struct message *msg)
{
	mutex_unlock(&msg->m_mutex);
	message_put(svc, msg);
}

static struct message *__dequeue_message(struct service *svc)
{
	struct message *msg;
	lockdep_assert_held(&svc->s_message_lock);

	msg = list_first_entry_or_null(&svc->s_messages, struct message, m_messages_node);
	if (!msg)
		return NULL;

	list_del_init(&msg->m_messages_node);
	servicefs_atomic_mask_set(&msg->m_flags, MESSAGE_FLAGS_PENDING_REPLY);

	/* If the message is a close message inherit the ref. */
	if (!is_close_message(msg))
		__message_get(msg);

	TRACE("dequeue: svc=%p m_id=%d m_op=%d msg->m_ref.count=%d\n",
			svc, msg->m_id, msg->m_op, atomic_read(&msg->m_ref.refcount));
	return msg;
}

static struct impulse *__dequeue_impulse(struct service *svc)
{
	struct impulse *impulse;
	lockdep_assert_held(&svc->s_message_lock);

	impulse = list_first_entry_or_null(&svc->s_impulses, struct impulse, i_impulses_node);
	if (impulse)
		list_del_init(&impulse->i_impulses_node);
	return impulse;
}

/**
 * message_release_kref - clean up a message after the last kref is dropped
 * @ref: pointer to the messages m_ref member
 *
 * Must be called with the service's s_message_lock held for writing.
 */
static void message_release_kref(struct kref *ref)
{
	struct message *msg = container_of(ref, struct message, m_ref);

	TRACE("release: msg=%p m_id=%d m_op=%d m_flags=0x%x m_status=%zd\n", msg,
			msg->m_id, msg->m_op, atomic_read(&msg->m_flags), msg->m_status);

	/* remove the message in case it's queued */
	list_del_init(&msg->m_messages_node);
	__free_message_id(msg);
	message_free(msg);
}

/**
 * __message_put - drop a reference to a message
 * @msg: pointer to the message to drop the reference to
 *
 * Must be called with the service's s_message_lock held for writing.
 */
static void __message_put(struct message *msg)
{
	kref_put(&msg->m_ref, message_release_kref);
}

static void message_put(struct service *svc, struct message *msg)
{
	write_lock(&svc->s_message_lock);
	__message_put(msg);
	write_unlock(&svc->s_message_lock);
}

/**
 * __message_get - get a reference to a message
 * @msg: pointer to the message to get a reference to
 *
 * Must be called with the service's s_message_lock held for reading/writing.
 */
static void __message_get(struct message *msg)
{
	BUG_ON(!kref_get_unless_zero(&msg->m_ref));
}

/**
 * message_put_complete - complete a message and wake up the sender
 * @msg: pointer to the message to complete
 * @retcode: the return code to return to the sender
 *
 * The caller must hold a ref to the message for the duration of the call,
 * which is dropped by this call.
 */
static void message_put_complete(struct message *msg, int retcode)
{
	bool drop_ref;
	struct pending_fd *pfd, *pfdn;
	struct service *svc = msg->m_channel->c_service;
	BUG_ON(!svc);

	write_lock(&svc->s_message_lock);

	/* Pending reply and close messages have an extra ref to drop. */
	drop_ref = is_message_pending_reply(msg) || is_close_message(msg);

	servicefs_atomic_mask_set(&msg->m_flags, MESSAGE_FLAGS_COMPLETED);
	msg->m_status = retcode;

	list_del_init(&msg->m_messages_node);
	__free_message_id(msg);

	write_unlock(&svc->s_message_lock);

	mutex_lock(&msg->m_mutex);

	/* Install pending fds or roll back fd allocation depending on retcode. */
	list_for_each_entry_safe(pfd, pfdn,
			&msg->m_pending_fds, p_pending_fds_node) {
		if (retcode >= 0) {
			servicefs_fd_install(msg->m_task, pfd->p_fd, pfd->p_filp);
		} else {
			servicefs_put_unused_fd(msg->m_task, pfd->p_fd);
			fput(pfd->p_filp);
		}

		list_del(&pfd->p_pending_fds_node);
		pending_fd_free(pfd);
	}

	TRACE("complete: svc=%p m_id=%d m_op=%d msg->m_ref.count=%d\n",
			svc, msg->m_id, msg->m_op, atomic_read(&msg->m_ref.refcount));

	mutex_unlock(&msg->m_mutex);

	write_lock(&svc->s_message_lock);
	wake_up(&msg->m_waitqueue);

	/* Drop the extra ref held by the service. */
	if (drop_ref)
		__message_put(msg);

	/* Drop the ref held by the caller. */
	__message_put(msg);
	write_unlock(&svc->s_message_lock);
}

/**
 * servicefs_check_channel - check if an fd in a message is a channel
 * @svc: pointer to the service owning the message
 * @svcfd: fd of the service to check the fd against
 * @mgsid: id of the message that the fd is in
 * @index: index of the fd to check in the message's fd array
 * @cid: optional user pointer to store the channel id
 * @ctx: optional user pointer to store the channel context pointer
 *
 * Checks whether the fd at the given index within the message's fd
 * array is a channel to the service described by svcfd. If the fd is
 * a channel to the service, optionally return the channel's id and
 * context pointer before returning success.
 *
 * svcfd may be -1, in which case the channel is checked against svc.
 */
int servicefs_check_channel(struct service *svc, int svcfd, int msgid,
		int index, __s32 __user *cid, __u64 __user *ctx)
{
	struct service *check_svc;
	struct message *msg;
	struct channel *c;
	struct file *filp = NULL, *svc_filp = NULL;
	int ret = 0;

	/* determine which service to check the channel against */
	if (svcfd == THIS_SERVICE_FD) {
		check_svc = svc;
	} else {
		svc_filp = fget(svcfd);
		if (!svc_filp) {
			ret = -EBADF;
			goto error;
		}

		check_svc = servicefs_get_service_from_file(svc_filp);
		if (!check_svc) {
			ret = -EINVAL;
			goto error;
		}
	}

	/* validate arguments and retrieve the message struct */
	if (cid && !access_ok(VERIFY_WRITE, cid, sizeof(*cid))) {
		ret = -EFAULT;
		goto error;
	}

	if (ctx && !access_ok(VERIFY_WRITE, ctx, sizeof(*ctx))) {
		ret = -EFAULT;
		goto error;
	}

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg)) {
		ret = PTR_ERR(msg);
		goto error;
	}

	if (is_close_message(msg) || index < 0 || index >= msg->m_fdcnt) {
		unlock_and_put_message(svc, msg);
		ret = -EINVAL;
		goto error;
	}

	/* get a reference to the file object for the fd */
	filp = servicefs_fget(msg->m_task, msg->m_fds[index]);
	if (!filp) {
		unlock_and_put_message(svc, msg);
		ret = -EBADF;
		goto error;
	}

	/* allow other threads the access the message now */
	unlock_and_put_message(svc, msg);

	/* get the channel for this file object if it's a channel */
	c = servicefs_get_channel_from_file(filp);
	if (!c || c->c_service != check_svc) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* return channel id and context if requested */
	if (cid && __put_user(c->c_id, cid)) {
		ret = -EFAULT;
		goto error;
	}

	if (ctx && __put_user(c->c_context, ctx)) {
		ret = -EFAULT;
		goto error;
	}

error:
	if (svc_filp)
		fput(svc_filp);
	if (filp)
		fput(filp);
	return ret;
}

int servicefs_push_channel(struct service *svc, int svcfd, int msgid,
		int flags, __s32 __user *cid, __u64 ctx)
{
	struct pending_fd *pfd;
	struct message *msg;
	struct channel *c;
	struct file *file, *svc_file, *put_file;
	int fd;

	/* filter user flags and force RDONLY */
	flags &= O_NONBLOCK | O_CLOEXEC;
	flags |= O_RDONLY;

	/*
	 * The new channel is added either to this service or the one
	 * specified by svcfd.
	 */
	if (svcfd == THIS_SERVICE_FD) {
		svc_file = svc->s_filp;
		put_file = NULL;
	} else {
		svc_file = fget(svcfd);
		if (!svc_file)
			return -EBADF;
		if (!servicefs_get_service_from_file(svc_file)) {
			fput(svc_file);
			return -EINVAL;
		}
		put_file = svc_file;
	}

	if (cid && !access_ok(VERIFY_WRITE, cid, sizeof(*cid)))
		return -EFAULT;

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg)) {
		fd = PTR_ERR(msg);
		goto error_unlocked;
	}

	if (is_close_message(msg)) {
		fd = -EINVAL;
		goto error_locked;
	}

	/* msg is valid and msg->m_mutex is locked at this point */
	fd = servicefs_get_unused_fd_flags(msg->m_task, flags);
	if (unlikely(fd < 0))
		goto error_locked;

	file = servicefs_create_channel(svc_file, flags);
	if (IS_ERR(file)) {
		servicefs_put_unused_fd(msg->m_task, fd);
		fd = PTR_ERR(file);
		goto error_locked;
	}

	c = file->private_data;
	BUG_ON(!c);

	c->c_context = ctx;

	pfd = kmem_cache_alloc(servicefs_pending_fd_cache, GFP_KERNEL);
	if (!pfd) {
		fput(file);
		servicefs_put_unused_fd(msg->m_task, fd);
		fd = -ENOMEM;
		goto error_locked;
	}

	/* write the new channel id to the user, if cid is not NULL */
	if (cid && __put_user(c->c_id, cid)) {
		fput(file);
		pending_fd_free(pfd);
		servicefs_put_unused_fd(msg->m_task, fd);
		fd = -EFAULT;
		goto error_locked;
	}

	/* no more failures can occur, complete the channel setup */
	servicefs_complete_channel_setup(file);

	/* Add the pending fd to the message. */
	pfd->p_fd = fd;
	pfd->p_filp = file;
	list_add_tail(&pfd->p_pending_fds_node, &msg->m_pending_fds);

error_locked:
	unlock_and_put_message(svc, msg);
error_unlocked:
	if (put_file)
		fput(put_file);
	return fd;
}

int servicefs_close_channel(struct service *svc, int cid)
{
	struct channel *c;
	int ret = 0;

	TRACE("close: cid=%d\n", cid);

	write_lock(&svc->s_channel_lock);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	channel_cancel(c);

done:
	write_unlock(&svc->s_channel_lock);
	return ret;
}

int servicefs_set_service_context(struct service *svc, __u64 ctx)
{
	write_lock(&svc->s_message_lock);
	svc->s_context = ctx;
	write_unlock(&svc->s_message_lock);

	return 0;
}

int servicefs_set_channel_context(struct service *svc, int cid, __u64 ctx)
{
	struct channel *c;
	int ret = 0;

	TRACE("set_ctx: cid=%d ctx=%p\n", cid, (void *) ctx);

	write_lock(&svc->s_channel_lock);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	c->c_context = ctx;

done:
	write_unlock(&svc->s_channel_lock);
	return ret;
}

static int wait_for_message(struct service *svc, long timeout)
{
	int ret = 0;

	if (!servicefs_queue_enter(svc))
		return -ESHUTDOWN;

	if (timeout) {
		if (down_interruptible(&svc->s_queue_messages)) {
			TRACE("Interrupted\n");
			ret = -ERESTARTSYS;
			goto done;
		}
	} else if (down_trylock(&svc->s_queue_messages)) {
		TRACE("Timeout\n");
		ret = -ETIMEDOUT;
		goto done;
	}

done:
	servicefs_queue_exit(svc);
	return ret;
}

int servicefs_msg_recv(struct service *svc,
		struct servicefs_msg_info_struct __user *msg_info, long timeout)
{
	struct message *msg;
	struct impulse *impulse;
	__u64 s_context;
	__u64 c_context;
	int c_id;
	struct servicefs_msg_info_struct info_out;
	int ret;

	TRACE("waiting for message: svc=%p timeout=%ld\n", svc, timeout);

	while (1) {
		ret = wait_for_message(svc, timeout);
		if (ret)
			return ret;

		write_lock(&svc->s_message_lock);

		if (is_service_canceled(svc)) {
			TRACE("Canceled\n");
			write_unlock(&svc->s_message_lock);
			return -ESHUTDOWN;
		}

		/* Impulses take priority over synchronous messages. */
		if (!list_empty(&svc->s_impulses)) {
			goto impulse_path;
		}

		if (!list_empty(&svc->s_messages))
			goto message_path;

		write_unlock(&svc->s_message_lock);
	}

message_path:
	TRACE("message_path\n");
	msg = __dequeue_message(svc);
	BUG_ON(!msg);

	s_context = svc->s_context;
	c_context = msg->m_channel->c_context;
	c_id = msg->m_channel->c_id;

	write_unlock(&svc->s_message_lock);
	mutex_lock(&msg->m_mutex);

	TRACE("m_id=%d m_op=%d m_sbuf.i_len=%zu m_rbuf.i_len=%zu msg_info=%p\n",
			msg->m_id, msg->m_op, msg->m_sbuf.i_len, msg->m_rbuf.i_len, msg_info);

	info_out.pid = msg->m_pid;
	info_out.tid = msg->m_tid;
	info_out.cid = c_id;
	info_out.mid = msg->m_id;
	info_out.euid = msg->m_euid;
	info_out.egid = msg->m_egid;
	info_out.service_private = s_context;
	info_out.channel_private = c_context;
	info_out.op = msg->m_op;
	info_out.flags = 0;
	info_out.send_len = msg->m_sbuf.i_len;
	info_out.recv_len = msg->m_rbuf.i_len;
	info_out.fd_count = msg->m_fdcnt;

	mutex_unlock(&msg->m_mutex);

	/* clear the impulse payload to prevent leaking stack contents */
	memset(info_out.impulse, 0, sizeof(info_out.impulse));

	if (copy_to_user(msg_info, &info_out, sizeof(info_out))) {
		pr_debug("Fault transferring msg info: pid=%d\n",
				pid_vnr(task_tgid(current)));
		message_put(svc, msg);
		return -EFAULT;
	}

	return 0;

impulse_path:
	TRACE("impulse_path\n");
	impulse = __dequeue_impulse(svc);
	BUG_ON(!impulse);

	s_context = svc->s_context;
	c_context = impulse->i_channel->c_context;
	c_id = impulse->i_channel->c_id;

	write_unlock(&svc->s_message_lock);

	TRACE("i_op=%d i_len=%u\n", impulse->i_op, impulse->i_len);

	info_out.pid = impulse->i_pid;
	info_out.tid = impulse->i_tid;
	info_out.cid = c_id;
	info_out.mid = MESSAGE_NO_ID; // impulses have no message id.
	info_out.euid = impulse->i_euid;
	info_out.egid = impulse->i_egid;
	info_out.service_private = s_context;
	info_out.channel_private = c_context;
	info_out.op = impulse->i_op;
	info_out.flags = 0;
	info_out.send_len = impulse->i_len;
	info_out.recv_len = 0;
	info_out.fd_count = 0;

	/* copy the impulse payload and clear the remaining bytes */
	memcpy(info_out.impulse, impulse->i_data, impulse->i_len);
	memset(((uint8_t *) info_out.impulse) + impulse->i_len, 0,
			sizeof(info_out.impulse) - impulse->i_len);

	impulse_free(impulse);

	if (copy_to_user(msg_info, &info_out, sizeof(info_out))) {
		pr_debug("Fault transferring msg info: pid=%d\n",
				pid_vnr(task_tgid(current)));
		return -EFAULT;
	}

	return 0;
}

ssize_t servicefs_msg_readv(struct service *svc, int msgid,
		const struct iovec *vec, size_t cnt)
{
	struct message *msg;
	ssize_t ret;
	struct iov_buffer buf;
	bool remote_fault;

	iov_buffer_uinit(&buf, vec, cnt);

	TRACE("readv: svc=%p msgid=%d length=%zu\n", svc, msgid, buf.i_len);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	ret = vm_transfer_from_remote(&msg->m_sbuf, &buf, msg->m_task,
			&remote_fault);

	unlock_and_put_message(svc, msg);

	/*
	 * Use EACCESS to communicate that the remote process faulted during
	 * the payload transfer.
	 */
	if (ret == -EFAULT && remote_fault)
		ret = -EACCES;

	return ret;
}

ssize_t servicefs_msg_writev(struct service *svc, int msgid,
		const struct iovec *vec, size_t cnt)
{
	struct message *msg;
	ssize_t ret;
	struct iov_buffer buf;
	bool remote_fault;

	iov_buffer_uinit(&buf, vec, cnt);

	TRACE("writev: svc=%p msgid=%d length=%zu\n", svc, msgid, buf.i_len);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	ret = vm_transfer_to_remote(&msg->m_rbuf, &buf, msg->m_task,
			&remote_fault);

	unlock_and_put_message(svc, msg);

	/*
	 * use EACCES to communicate that the remote process faulted during
	 * the payload transfer.
	 */
	if (ret == -EFAULT && remote_fault)
		ret = -EACCES;

	return ret;
}

int servicefs_msg_seek(struct service *svc, int msgid, long offset,
		int whence)
{
	return -EINVAL;
}

ssize_t servicefs_msg_busv(struct service *svc, int dst_msgid, long dst_off,
		int src_msgid, long src_off, size_t len)
{
	return -EINVAL;
}

int servicefs_msg_reply(struct service *svc, int msgid, ssize_t retcode)
{
	struct message *msg;

	TRACE("reply: svc=%p msgid=%d retcode=%zd\n", svc, msgid, retcode);

	/*
	 * Make sure the return code is in the valid range [-MAX_ERRNO, MAX_INT].
	 * TODO: block special return values, such as ERESTARTSYS.
	 */
	if (retcode < -MAX_ERRNO)
		return -EINVAL;

	msg = lookup_get_message(svc, msgid, false);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	message_put_complete(msg, retcode);
	return 0;
}

int servicefs_msg_reply_fd(struct service *svc, int msgid, unsigned int pushfd)
{
	struct message *msg;
	int newfd;
	struct file *filp;

	TRACE("reply_fd: svc=%p msgid=%d pushfd=%u\n", svc, msgid, pushfd);

	msg = lookup_get_message(svc, msgid, false);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	if (is_message_interrupted(msg) || is_close_message(msg)) {
		message_put_complete(msg, -EINTR);
		return 0;
	}

	mutex_lock(&msg->m_mutex);

	filp = fget(pushfd);
	if (!filp) {
		unlock_and_put_message(svc, msg);
		return -EINVAL;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(msg->m_task, 0);
	if (newfd < 0) {
		fput(filp);
		unlock_and_put_message(svc, msg);
		return newfd;
	}

	servicefs_fd_install(msg->m_task, newfd, filp);

	mutex_unlock(&msg->m_mutex);

	message_put_complete(msg, newfd);
	return 0;
}

int servicefs_msg_push_fd(struct service *svc, int msgid, unsigned int pushfd)
{
	struct message *msg;
	struct pending_fd *pfd;
	int ret = 0;
	int newfd;
	struct file *filp;

	TRACE("push_fd: svc=%p msgid=%d pushfd=%u\n", svc, msgid, pushfd);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	if (is_close_message(msg)) {
		ret = -EINVAL;
		goto error;
	}

	filp = fget(pushfd);
	if (!filp) {
		ret = -EINVAL;
		goto error;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(msg->m_task, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error;
	}

	pfd = kmem_cache_alloc(servicefs_pending_fd_cache, GFP_KERNEL);
	if (!pfd) {
		fput(filp);
		servicefs_put_unused_fd(msg->m_task, newfd);
		ret = -ENOMEM;
		goto error;
	}

	/* Add the pending fd to the message. */
	pfd->p_fd = newfd;
	pfd->p_filp = filp;
	list_add_tail(&pfd->p_pending_fds_node, &msg->m_pending_fds);

	ret = newfd;

error:
	unlock_and_put_message(svc, msg);
	return ret;
}

int servicefs_msg_get_fd(struct service *svc, int msgid, unsigned int index)
{
	struct message *msg;
	int ret = 0;
	int newfd;
	int getfd;
	struct file *filp;

	TRACE("get_fd: svc=%p msgid=%d index=%u\n", svc, msgid, index);

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg))
		return PTR_ERR(msg);

	if (is_close_message(msg) || index >= msg->m_fdcnt) {
		ret = -EINVAL;
		goto error;
	}
	getfd = msg->m_fds[index];

	filp = servicefs_fget(msg->m_task, getfd);
	if (!filp) {
		ret = -EBADF;
		goto error;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(current, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error;
	}

	servicefs_fd_install(current, newfd, filp);
	ret = newfd;

error:
	unlock_and_put_message(svc, msg);
	return ret;
}

int servicefs_mod_channel_events(struct service *svc, int cid,
		int clr, int set)
{
	int ret = 0;
	int previous;
	struct channel *c;

	read_lock(&svc->s_channel_lock);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	previous = servicefs_atomic_update_mask(&c->c_events, clr, set);
	wake_up_poll(&c->c_waitqueue, (long) ((previous & ~clr) | set));

done:
	read_unlock(&svc->s_channel_lock);
	return ret;

}

ssize_t servicefs_msg_sendv(struct channel *c, int op,
		const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt,
		const __s32 *fds, size_t fdcnt, long task_state)
{
	struct message *msg;
	struct service *svc;
	int ret;

	svc = c->c_service;
	BUG_ON(!svc);

	msg = message_new(svc, c, op, svec, scnt, rvec, rcnt, fds, fdcnt, current);
	if (!msg)
		return -ENOMEM;

	TRACE("sendv: svc=%p msg=%p op=%d scnt=%zu slen=%zu rcnt=%zu rlen=%zu fdcnt=%zu\n",
			svc, msg, op, msg->m_sbuf.i_cnt, msg->m_sbuf.i_len,
			msg->m_rbuf.i_cnt, msg->m_rbuf.i_len, msg->m_fdcnt);

	ret = enqueue_message(svc, c, msg);
	if (ret)
		return ret;

	/* wait for message reply/cancel or a signal if interruptible */
	if (task_state == TASK_INTERRUPTIBLE)
		wait_event_interruptible(msg->m_waitqueue, is_message_completed(msg));
	else
		wait_event(msg->m_waitqueue, is_message_completed(msg));

	mutex_lock(&msg->m_mutex);

	if (is_message_completed(msg)) {
		ret = msg->m_status;
	} else {
		servicefs_atomic_mask_set(&msg->m_flags, MESSAGE_FLAGS_INTERRUPTED);
		ret = -EINTR;
	}

	mutex_unlock(&msg->m_mutex);

	/* release the message now that we're done with it */
	message_put(svc, msg);
	TRACE("sendv: ret=%d\n", ret);
	return ret;
}

int servicefs_msg_send_impulse(struct channel *c, int op,
		void __user *buf, size_t len)
{
	struct impulse *impulse;
	struct service *svc;
	kuid_t kuid;
	kgid_t kgid;

	svc = c->c_service;
	BUG_ON(!svc);

	if (len > sizeof(impulse->i_data))
		return -EINVAL;

	impulse = kmem_cache_alloc(servicefs_impulse_cache, GFP_KERNEL);
	if (!impulse)
		return -ENOMEM;

	INIT_LIST_HEAD(&impulse->i_impulses_node);
	impulse->i_op = op;
	impulse->i_len = len;

	/*
	 * The impulse holds a ref to the channel. The channel file object also
	 * holds a ref to the channel, which is guaranteed to be maintained for
	 * the duration of this function because the ioctl holds a ref to the
	 * channel file object. The channel can be canceled however, which is
	 * handled below.
	 */
	channel_get(c);
	impulse->i_channel = c;

	current_euid_egid(&kuid, &kgid);
	impulse->i_euid = __kuid_val(kuid);
	impulse->i_egid = __kgid_val(kgid);

	impulse->i_pid = pid_vnr(task_tgid(current));
	impulse->i_tid = pid_vnr(task_pid(current));

	if (len > 0 && copy_from_user(&impulse->i_data, buf, len)) {
		pr_debug("Fault transferring impulse payload: pid=%d\n",
			pid_vnr(task_tgid(current)));
		impulse_free(impulse);
		return -EFAULT;
	}

	return enqueue_impulse(svc, c, impulse);
}

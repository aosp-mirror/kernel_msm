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
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/poison.h>
#include <linux/poll.h>
#include <linux/namei.h>

#include "servicefs_private.h"
#include "servicefs_ioctl.h"
#include "servicefs_compat_ioctl.h"

#define THIS_SERVICE_FD (-1)

/**
 * message_cache - cache for message structs.
 */
static struct kmem_cache *message_cache;

/**
 * impulse_cache - cache for impulse structs.
 */
static struct kmem_cache *impulse_cache;

int servicefs_cache_init(void)
{
	unsigned long flags = SLAB_HWCACHE_ALIGN;
#ifdef CONFIG_SERVICEFS_USE_POISON
	flags |= SLAB_POISON;
#endif

	message_cache = KMEM_CACHE(message, flags);
	if (!message_cache)
		goto error;

	impulse_cache = KMEM_CACHE(impulse, flags);
	if (!impulse_cache)
		goto error;

	return 0;

error:
	if (message_cache) {
		kmem_cache_destroy(message_cache);
		message_cache = NULL;
	}
	if (impulse_cache) {
		kmem_cache_destroy(impulse_cache);
		impulse_cache = NULL;
	}

	return -ENOMEM;
}

void __message_get(struct message *msg);
void __detach_message(struct message *msg);

void service_poison(struct service *svc)
{
#ifdef CONFIG_SERVICEFS_USE_POISON
	unsigned char *end = ((unsigned char *) svc) + sizeof(*svc) - 1;
	memset(svc, POISON_FREE, sizeof(*svc) - 1);
	*end = POISON_END;
#endif
}

void channel_poison(struct channel *c)
{
#ifdef CONFIG_SERVICEFS_USE_POISON
	unsigned char *end = ((unsigned char *) c) + sizeof(*c) - 1;
	memset(c, POISON_FREE, sizeof(*c) - 1);
	*end = POISON_END;
#endif
}

void channel_free(struct channel *c)
{
	channel_poison(c);
	kfree(c);
}

void message_free(struct message *m)
{
	kmem_cache_free(message_cache, m);
}

void impulse_free(struct impulse *i)
{
	kmem_cache_free(impulse_cache, i);
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
	pr_debug("freeing service: svc=%p\n", svc);

	/* make sure everything is already properly cleaned up */
	BUG_ON(!list_empty(&svc->s_channels));
	BUG_ON(!list_empty(&svc->s_impulses));
	BUG_ON(!list_empty(&svc->s_messages));
	BUG_ON(!list_empty(&svc->s_active));
	BUG_ON(waitqueue_active(&svc->s_wqreceivers));
	BUG_ON(waitqueue_active(&svc->s_wqselect));

	/* free allocators */
	idr_destroy(&svc->s_channel_idr);
	idr_destroy(&svc->s_message_idr);

	service_poison(svc);
	kfree(svc);
}

/**
 * service_new - allocate and initialize a new service
 *
 * Returns a pointer to the new service.
 */
struct service *service_new(void)
{
	struct service *svc = kzalloc(sizeof(struct service), GFP_KERNEL);
	if (!svc) {
		pr_err("failed to allocate service struct\n");
		goto out;
	}

	pr_debug("creating new service: svc=%p\n", svc);

	mutex_init(&svc->s_mutex);

	idr_init(&svc->s_channel_idr);
	idr_init(&svc->s_message_idr);

	INIT_LIST_HEAD(&svc->s_channels);
	INIT_LIST_HEAD(&svc->s_impulses);
	INIT_LIST_HEAD(&svc->s_messages);
	INIT_LIST_HEAD(&svc->s_active);

	init_waitqueue_head(&svc->s_wqreceivers);
	init_waitqueue_head(&svc->s_wqselect);

	svc->s_flags = SERVICE_FLAGS_DEFAULT;

out:
	return svc;
}

int service_cancel(struct service *svc)
{
	int ret;
	struct channel *c, *cn;
	struct impulse *i, *in;
	struct message *m, *mn;

	mutex_lock(&svc->s_mutex);

	if (!__is_service_canceled(svc)) {
		pr_debug("canceling service=%p\n", svc);
		svc->s_flags |= SERVICE_FLAGS_CANCELED;

		pr_debug("removing dentry service=%p\n", svc);
		servicefs_remove_dentry(svc->s_filp->f_path.dentry);

		list_for_each_entry_safe(i, in, &svc->s_impulses, i_impulses_node) {
			__impulse_cancel(i);
		}

		list_for_each_entry_safe(m, mn, &svc->s_messages, m_messages_node) {
			__message_cancel(m);
		}

		list_for_each_entry_safe(m, mn, &svc->s_active, m_messages_node) {
			__message_cancel(m);
		}

		list_for_each_entry_safe(c, cn, &svc->s_channels, c_channels_node) {
			__channel_cancel(c);
		}

		wake_up_all(&svc->s_wqreceivers);
		wake_up_poll(&svc->s_wqselect, POLLHUP | POLLFREE);

		ret = 0;
	} else {
		pr_debug("already canceled service=%p\n", svc);
		ret = -ESHUTDOWN;
	}

	mutex_unlock(&svc->s_mutex);

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
 *
 * This function must be called with the service's s_mutex held.
 */
static int alloc_channel_id(struct channel *c)
{
	int ret;
	struct service *svc = c->c_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	ret = idr_alloc(&svc->s_channel_idr, c, svc->s_channel_start, -1, GFP_KERNEL);
	if (ret < 0)
		return ret;
	else
		svc->s_channel_start = (svc->s_channel_start + 1) & 0x7fffffff;

	c->c_id = ret;
	return 0;
}

/**
 * free_channel_id - free the channel id for a channel
 * @c: a pointer to the channel to free the id for.
 *
 * This function must be called with service's s_mutex held.
 */
static void free_channel_id(struct channel *c)
{
	struct service *svc = c->c_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	idr_remove(&svc->s_channel_idr, c->c_id);
}

/**
 * alloc_message_id - allocates an unused message for a message
 * @svc: pointer to the service to allocate the message id on.
 * @m: pointer to the message to associate the id with.
 *
 * This function allocates an id for a message on its service's message id
 * allocator. The given message's m_id member is filled in with the id and
 * the message pointer is associated with the id for future lookup.
 *
 * The message's m_service member must be set before calling this function.
 *
 * This function must be called with the service's s_mutex held.
 */
static int alloc_message_id(struct message *m)
{
	int ret;
	struct service *svc = m->m_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	ret = idr_alloc(&svc->s_message_idr, m, svc->s_message_start, -1, GFP_KERNEL);
	if (ret < 0)
		return ret;
	else
		svc->s_message_start = (svc->s_message_start + 1) & 0x7fffffff;

	m->m_id = ret;
	return 0;
}

/**
 * free_message_id - free the message id for a message
 * @m: a pointer to the message to free the id for.
 *
 * This function must be called with the service's s_mutex held.
 */
static void free_message_id(struct message *m)
{
	struct service *svc = m->m_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	if (m->m_id != MESSAGE_NO_ID) {
		idr_remove(&svc->s_message_idr, m->m_id);
		m->m_id = MESSAGE_NO_ID;
	}
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
	struct channel *c = kmalloc(sizeof(struct channel), GFP_KERNEL);
	if (!c)
		goto alloc_fail;

	c->c_service = svc;
	c->c_context = NULL;
	c->c_flags = 0;

	c->c_events = 0;
	init_waitqueue_head(&c->c_waitqueue);

	mutex_lock(&svc->s_mutex);

	ret = alloc_channel_id(c);
	if (ret)
		goto id_fail;

	list_add_tail(&c->c_channels_node, &svc->s_channels);

	mutex_unlock(&svc->s_mutex);

	pr_debug("added channel id=%d to service=%p\n", c->c_id, svc);

alloc_fail:
	return c;

id_fail:
	mutex_unlock(&svc->s_mutex);
	channel_free(c);
	return NULL;
}

/**
 * __channel_cancel - cancel a channel and wake up clients
 * @c: pointer to the channel to cancel
 *
 * Puts the channel into canceled state and wakes up any
 * clients that are blocked waiting for channel events.
 * POLLHUP is guaranteed by the kernel to always be in the
 * client event set, and is used to signal that the file
 * descriptor is in a canceled state.
 *
 * Must be called with the service's s_mutex held.
 */
void __channel_cancel(struct channel *c)
{
	struct service *svc = c->c_service;
	BUG_ON(!svc);

	lockdep_assert_held(&svc->s_mutex);

	list_del_init(&c->c_channels_node);
	free_channel_id(c);

	c->c_flags |= CHANNEL_FLAGS_CANCELED;
	c->c_events |= POLLHUP | POLLFREE;

	wake_up_poll(&c->c_waitqueue, POLLHUP | POLLFREE);
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
	struct impulse *i, *in;
	struct message *m, *mn;
	struct service *svc = c->c_service;
	BUG_ON(svc == NULL);

	pr_debug("removing channel id=%d from service=%p\n", c->c_id, svc);

	mutex_lock(&svc->s_mutex);

	if (!__is_channel_canceled(c))
		__channel_cancel(c);

	/* cancel all the pending impulses on this channel */
	list_for_each_entry_safe(i, in, &svc->s_impulses, i_impulses_node) {
		if (i->i_channel == c)
			__impulse_cancel(i);
	}

	/* cancel all the pending messages on this channel */
	list_for_each_entry_safe(m, mn, &svc->s_messages, m_messages_node) {
		mutex_lock(&m->m_mutex);

		if (m->m_channel == c)
			__message_cancel(m);

		mutex_unlock(&m->m_mutex);
	}

	/* detach all the active messages from this channel */
	list_for_each_entry_safe(m, mn, &svc->s_active, m_messages_node) {
		mutex_lock(&m->m_mutex);

		if (m->m_channel == c)
			__detach_message(m);

		mutex_unlock(&m->m_mutex);
	}

	mutex_unlock(&svc->s_mutex);

	channel_free(c);
}

/**
 * __message_cancel - cancel a pending message
 * @msg: pointer to the message to cancel
 *
 * Completes a message with a return code of -ESHUTDOWN.
 *
 * Must be called with the service's s_mutex held.
 */
void __message_cancel(struct message *msg)
{
	pr_debug("msg=%p\n", msg);
	__message_complete(msg, -ESHUTDOWN);
}

/**
 * __detach_message - detaches a message from its channel
 * @msg: pointer to the message to detach
 *
 * Must be called with the message's m_mutex held.
 */
void __detach_message(struct message *msg)
{
	pr_debug("msg=%p\n", msg);
	msg->m_channel = NULL;
}

void __impulse_cancel(struct impulse *impulse)
{
	list_del_init(&impulse->i_impulses_node);
	impulse_free(impulse);
}

struct channel *__lookup_channel(struct service *svc, int cid)
{
	return idr_find(&svc->s_channel_idr, cid);
}

struct message *__lookup_message(struct service *svc, int mid)
{
	return idr_find(&svc->s_message_idr, mid);
}

/**
 * lookup_and_lock_message - looks up a message, acquiring its m_mutex
 * @svc: pointer to the service in which to look up the message
 * @mid: message id of the message to lookup
 * @check_interrupted: whether to check the interrupted status of the message
 *  and bail out, leaving the message unlocked
 * Returns a message on success, with its m_mutex acquired. On error
 * an error code is returned as an error pointer.
 *
 * This function properly interlocks the release of svc->s_mutex with
 * the acquisition of the message's m_mutex.
 */
struct message *lookup_and_lock_message(struct service *svc, int mid,
		bool check_interrupted)
{
	struct message *msg;

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, mid);
	if (!msg) {
		mutex_unlock(&svc->s_mutex);
		return ERR_PTR(-ENOENT);
	}

	mutex_lock(&msg->m_mutex);

	if (check_interrupted && __is_message_interrupted(msg)) {
		mutex_unlock(&msg->m_mutex);
		mutex_unlock(&svc->s_mutex);
		return ERR_PTR(-EINTR);
	}

	// unlock s_mutex, leaving m_mutex locked
	mutex_unlock(&svc->s_mutex);
	return msg;
}

struct message *__peek_message(struct service *svc)
{
	lockdep_assert_held(&svc->s_mutex);
	return list_first_entry_or_null(&svc->s_messages, struct message, m_messages_node);
}

int __activate_message(struct message *msg)
{
	int ret;
	struct service *svc = msg->m_service;
	BUG_ON(svc == NULL);

	lockdep_assert_held(&svc->s_mutex);

	ret = alloc_message_id(msg);
	if (ret)
		goto error;

	list_del_init(&msg->m_messages_node);
	list_add_tail(&msg->m_messages_node, &svc->s_active);

	__message_get(msg);

error:
	return ret;
}

struct impulse *__dequeue_impulse(struct service *svc)
{
	struct impulse *impulse;
	lockdep_assert_held(&svc->s_mutex);
	impulse = list_first_entry_or_null(&svc->s_impulses, struct impulse, i_impulses_node);
	if (impulse)
		list_del_init(&impulse->i_impulses_node);
	return impulse;
}

/**
 * message_release_kref - clean up a message after the last kref is dropped
 * @ref: pointer to the messages m_ref member
 *
 * Must be called with the service's s_mutex held.
 */
static void message_release_kref(struct kref *ref)
{
	struct message *msg = container_of(ref, struct message, m_ref);

	pr_debug("msg=%p m_id=%d m_completed=%d m_interrupted=%d "
			"m_status=%zu\n", msg, msg->m_id, msg->m_completed,
			msg->m_interrupted, msg->m_status);

	/* remove the message in case it's queued */
	list_del_init(&msg->m_messages_node);
	free_message_id(msg);

	put_task_struct(msg->m_task);
	message_free(msg);
}

/**
 * __message_put - drop a reference to a message
 * @msg: pointer to the message to drop the reference to
 *
 * Must be called with the service's s_mutex held.
 */
void __message_put(struct message *msg)
{
	kref_put(&msg->m_ref, message_release_kref);
}

void message_put(struct service *svc, struct message *msg)
{
	mutex_lock(&svc->s_mutex);
	__message_put(msg);
	mutex_unlock(&svc->s_mutex);
}


/**
 * __message_get - get a reference to a message
 * @msg: pointer to the message to get a reference to
 *
 * Must be called with the service's s_mutex held.
 */
void __message_get(struct message *msg)
{
	kref_get(&msg->m_ref);
}

/**
 * __message_complete - complete a message and wake up the sender
 * @msg: pointer to the message to complete
 * @retcode: the return code to return to the sender
 *
 * Must be called with the service's s_mutex held.
 */
void __message_complete(struct message *msg, int retcode)
{
	/* capture active state before we free the message id */
	bool active = __is_message_active(msg);

	list_del_init(&msg->m_messages_node);
	free_message_id(msg);

	mutex_lock(&msg->m_mutex);

	msg->m_status = retcode;
	msg->m_completed = true;

	mutex_unlock(&msg->m_mutex);

	wake_up(&msg->m_waitqueue);

	if (active)
		__message_put(msg);
}

/*
 * Utilities to handle differences between 64bit and 32bit userspace data sizes.
 * When CONFIG_COMPAT is not defined, simplified versions that ignore 32bit are
 * used and BUG is triggered if requests for 32bit are made.
 */
#ifdef CONFIG_COMPAT
static long copy_msg_info_to_user(
		struct servicefs_msg_info_struct __user *dest,
		struct servicefs_msg_info_struct *src, bool is_compat)
{
	if (is_compat) {
		struct servicefs_compat_msg_info_struct compat_info;

		compat_info.pid = src->pid;
		compat_info.tid = src->tid;
		compat_info.cid = src->cid;
		compat_info.mid = src->mid;
		compat_info.euid= src->euid;
		compat_info.egid = src->egid;
		compat_info.service_private = ptr_to_compat(src->service_private);
		compat_info.channel_private = ptr_to_compat(src->channel_private);
		compat_info.op = src->op;
		compat_info.flags = src->flags;
		compat_info.send_len = src->send_len;
		compat_info.recv_len = src->recv_len;
		compat_info.fd_count = src->fd_count;
		memcpy(compat_info.impulse, src->impulse, sizeof(compat_info.impulse));

		return copy_to_user(dest, &compat_info, sizeof(compat_info));
	} else {
		return copy_to_user(dest, src, sizeof(*src));
	}
}

static inline bool check_access_cid(int __user *cid, bool is_compat) {
	size_t size = is_compat ? sizeof(compat_int_t) : sizeof(*cid);
	return access_ok(VERIFY_WRITE, cid, size);
}

static inline bool check_access_ctx(void __user **ctx, bool is_compat) {
	size_t size = is_compat ? sizeof(compat_uptr_t) : sizeof(*ctx);
	return access_ok(VERIFY_WRITE, ctx, size);
}

static inline long __put_user_cid(int cid, int __user *cid_ptr,
		bool is_compat) {
	if (is_compat) {
		compat_int_t compat_cid = cid;
		return __copy_to_user(cid_ptr, &compat_cid, sizeof(compat_cid));
	} else {
		return __put_user(cid, cid_ptr);
	}
}

static inline long __put_user_ctx(void *ctx, void __user **ctx_ptr,
		bool is_compat) {
	if (is_compat) {
		compat_uptr_t compat_ctx = ptr_to_compat(ctx);
		return __copy_to_user(ctx_ptr, &compat_ctx, sizeof(compat_ctx));
	} else {
		return __put_user(ctx, ctx_ptr);
	}
}
#else
static long copy_msg_info_to_user(
		struct servicefs_msg_info_struct __user *dest,
		struct servicefs_msg_info_struct *src, bool is_compat)
{
	BUG_ON(is_compat);
	return copy_to_user(dest, src, sizeof(*src));
}

static inline bool check_access_cid(int __user *cid, bool is_compat) {
	BUG_ON(is_compat);
	return access_ok(VERIFY_WRITE, cid, sizeof(*cid));
}

static inline bool check_access_ctx(void __user **ctx, bool is_compat) {
	BUG_ON(is_compat);
	return access_ok(VERIFY_WRITE, ctx, sizeof(*ctx));
}

static inline long __put_user_cid(int cid, int __user *cid_ptr,
		bool is_compat) {
	BUG_ON(is_compat);
	return __put_user(cid, cid_ptr);
}

static inline long __put_user_ctx(void *ctx, void __user **ctx_ptr,
		bool is_compat) {
	BUG_ON(is_compat);
	return __put_user(ctx, ctx_ptr);
}
#endif

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
		int index, int __user *cid, void __user **ctx, bool is_compat)
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
	if (cid && !check_access_cid(cid, is_compat)) {
		ret = -EFAULT;
		goto error;
	}

	if (ctx && !check_access_ctx(ctx, is_compat)) {
		ret = -EFAULT;
		goto error;
	}

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg)) {
		ret = PTR_ERR(msg);
		goto error;
	}

	if (index < 0 || index >= msg->m_fdcnt) {
		mutex_unlock(&msg->m_mutex);
		ret = -EINVAL;
		goto error;
	}

	/* get a reference to the file object for the fd */
	filp = servicefs_fget(msg->m_task, msg->m_fds[index]);
	if (!filp) {
		mutex_unlock(&msg->m_mutex);
		ret = -EBADF;
		goto error;
	}

	/* allow other threads the access the message now */
	mutex_unlock(&msg->m_mutex);

	/* get the channel for this file object if it's a channel */
	c = servicefs_get_channel_from_file(filp);
	if (!c || c->c_service != check_svc) {
		ret = -EOPNOTSUPP;
		goto error;
	}

	/* return channel id and context if requested */
	if (cid && __put_user_cid(c->c_id, cid, is_compat)) {
		ret = -EFAULT;
		goto error;
	}

	if (ctx && __put_user_ctx(c->c_context, ctx, is_compat)) {
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
		int flags, int __user *cid, void __user *ctx, bool is_compat)
{
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
		if (!servicefs_get_service_from_file(svc_file))
			return -EINVAL;
		put_file = svc_file;
	}

	if (cid && !check_access_cid(cid, is_compat))
		return -EFAULT;

	msg = lookup_and_lock_message(svc, msgid, true);
	if (IS_ERR(msg)) {
		fd = PTR_ERR(msg);
		goto error_unlocked;
	}

	/* msg is valid and msg->m_mutex is locked at this point */
	fd = servicefs_get_unused_fd_flags(msg->m_task, flags);
	if (unlikely(fd < 0))
		goto error_locked;

	file = servicefs_create_channel(svc_file, flags);
	if (IS_ERR(file)) {
		put_unused_fd(fd);
		fd = PTR_ERR(file);
		goto error_locked;
	}

	c = file->private_data;
	BUG_ON(!c);

	c->c_context = ctx;

	/* write the new channel id to the user, if cid is not NULL */
	if (cid && __put_user_cid(c->c_id, cid, is_compat)) {
		put_unused_fd(fd);
		fd = -EFAULT;
		goto error_locked;
	}

	/* no more failures can occur, complete the channel setup */
	servicefs_complete_channel_setup(file);
	servicefs_fd_install(msg->m_task, fd, file);

error_locked:
	mutex_unlock(&msg->m_mutex);
error_unlocked:
	if (put_file)
		fput(put_file);
	return fd;
}

int servicefs_close_channel(struct service *svc, int cid)
{
	struct channel *c;
	int ret = 0;

	pr_debug("cid=%d\n", cid);

	mutex_lock(&svc->s_mutex);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	__channel_cancel(c);

done:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

int servicefs_set_service_context(struct service *svc, void __user *ctx)
{
	mutex_lock(&svc->s_mutex);
	svc->s_context = ctx;
	mutex_unlock(&svc->s_mutex);

	return 0;
}

int servicefs_set_channel_context(struct service *svc, int cid,
		void __user *ctx)
{
	struct channel *c;
	int ret = 0;

	pr_debug("cid=%d ctx=%p\n", cid, ctx);

	mutex_lock(&svc->s_mutex);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	c->c_context = ctx;

done:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

static inline bool is_message_pending(struct service *svc)
{
	bool pending;

	mutex_lock(&svc->s_mutex);
	pending = __is_service_canceled(svc)
			|| !list_empty(&svc->s_impulses)
			|| !list_empty(&svc->s_messages);
	mutex_unlock(&svc->s_mutex);

	return pending;
}

int servicefs_msg_recv(struct service *svc,
		struct servicefs_msg_info_struct __user *msg_info, long timeout,
		bool is_compat)
{
	struct message *msg;
	struct impulse *impulse;
	void __user * s_context;
	void __user * c_context;
	int c_id;
	struct servicefs_msg_info_struct info_out;
	int ret;

	mutex_lock(&svc->s_mutex);

	do {
		if (__is_service_canceled(svc)) {
			pr_debug("canceled_path\n");
			mutex_unlock(&svc->s_mutex);
			return -ESHUTDOWN;
		}

		/* impulses take priority over messages */
		if (!list_empty(&svc->s_impulses))
			goto impulse_path;

		if (!list_empty(&svc->s_messages))
			goto message_path;

		if (signal_pending(current)) {
			pr_debug("interrupted_path\n");
			mutex_unlock(&svc->s_mutex);
			return -ERESTARTSYS;
		}

		if (timeout <= 0) {
			pr_debug("timeout_path\n");
			mutex_unlock(&svc->s_mutex);
			return -ETIMEDOUT;
		}

		mutex_unlock(&svc->s_mutex);

		pr_debug("waiting for message: svc=%p timeout=%ld\n", svc, timeout);

		timeout = wait_event_interruptible_timeout(svc->s_wqreceivers,
				is_message_pending(svc), timeout);

		mutex_lock(&svc->s_mutex);
	} while (1);

message_path:
	pr_debug("message_path\n");
	msg = __peek_message(svc);
	BUG_ON(!msg);

	ret = __activate_message(msg);
	if (ret) {
		pr_debug("error: %d\n", ret);
		mutex_unlock(&svc->s_mutex);
		return ret;
	}

	/* grab the service/channel context and channel id before releasing s_mutex */
	s_context = svc->s_context;
	c_context = msg->m_channel ? msg->m_channel->c_context : NULL;
	c_id = msg->m_channel ? msg->m_channel->c_id : -1;

	mutex_lock(&msg->m_mutex);
	mutex_unlock(&svc->s_mutex);

	pr_debug("m_id=%d m_op=%d m_sbuf.i_len=%zu m_rbuf.i_len=%zu msg_info=%p\n",
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

	/* clear the impulse payload to prevent leaking stack contents */
	memset(info_out.impulse, 0, sizeof(info_out.impulse));

	if (copy_msg_info_to_user(msg_info, &info_out, is_compat)) {
		pr_debug("Fault transferring msg info: pid=%d\n",
				pid_vnr(task_tgid(current)));
		mutex_unlock(&msg->m_mutex);
		message_put(svc, msg);
		return -EFAULT;
	}

	mutex_unlock(&msg->m_mutex);
	return 0;

impulse_path:
	pr_debug("impulse_path\n");
	impulse = __dequeue_impulse(svc);
	BUG_ON(!impulse);

	/* grab the service/channel context and channel id before releasing s_mutex */
	s_context = svc->s_context;
	c_context = impulse->i_channel ? impulse->i_channel->c_context : NULL;
	c_id = impulse->i_channel ? impulse->i_channel->c_id : -1;

	mutex_unlock(&svc->s_mutex);

	pr_debug("i_op=%d i_len=%zu\n", impulse->i_op, impulse->i_len);

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

	if (copy_msg_info_to_user(msg_info, &info_out, is_compat)) {
		pr_debug("Fault transferring msg info: pid=%d\n",
				pid_vnr(task_tgid(current)));
		return -EFAULT;
	}

	return 0;
}

ssize_t servicefs_msg_readv(struct service *svc, int msgid, const iov *vec,
		size_t cnt)
{
	struct message *msg;
	ssize_t ret;
	struct iov_buffer buf;
	bool remote_fault;

	iov_buffer_uinit(&buf, vec, cnt);

	pr_debug("svc=%p msgid=%d length=%zu\n", svc, msgid, buf.i_len);

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error;
	}

	mutex_lock(&msg->m_mutex); // nesting: first s_mutex then m_mutex

	if (__is_message_interrupted(msg)) {
		mutex_unlock(&msg->m_mutex);
		ret = -EINTR;
		goto error;
	}

	mutex_unlock(&svc->s_mutex); // allow other threads into the service

	ret = vm_transfer_from_remote(&msg->m_sbuf, &buf, msg->m_task,
			&remote_fault);

	mutex_unlock(&msg->m_mutex);

	/*
	 * use EACCESS to communicate that the remote process faulted during
	 * the payload transfer.
	 */
	if (ret == -EFAULT && remote_fault)
		ret = -EACCES;

	return ret;

error:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

ssize_t servicefs_msg_writev(struct service *svc, int msgid, const iov *vec,
		size_t cnt)
{
	struct message *msg;
	ssize_t ret;
	struct iov_buffer buf;
	bool remote_fault;

	iov_buffer_uinit(&buf, vec, cnt);

	pr_debug("svc=%p msgid=%d length=%zu\n", svc, msgid, buf.i_len);

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error;
	}

	mutex_lock(&msg->m_mutex); // nesting: first s_mutex then m_mutex

	if (__is_message_interrupted(msg)) {
		mutex_unlock(&msg->m_mutex);
		ret = -EINTR;
		goto error;
	}

	mutex_unlock(&svc->s_mutex); // allow other threads into the service

	ret = vm_transfer_to_remote(&msg->m_rbuf, &buf, msg->m_task,
			&remote_fault);

	mutex_unlock(&msg->m_mutex);

	/*
	 * use EACCES to communicate that the remote process faulted during
	 * the payload transfer.
	 */
	if (ret == -EFAULT && remote_fault)
		ret = -EACCES;

	return ret;

error:
	mutex_unlock(&svc->s_mutex);
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
	int ret = 0;

	pr_debug("svc=%p msgid=%d retcode=%zd\n", svc, msgid, retcode);

	/*
	 * make sure the return code is in the valid range [-MAX_ERRNO, MAX_INT].
	 * TODO: block special return values, such as ERESTARTSYS.
	 */
	if (retcode < -MAX_ERRNO)
		return -EINVAL;

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error;
	}

	__message_complete(msg, retcode);

error:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

int servicefs_msg_reply_fd(struct service *svc, int msgid, unsigned int pushfd)
{
	struct message *msg;
	int ret = 0;
	int newfd;
	struct file *filp;

	pr_debug("svc=%p msgid=%d pushfd=%u\n", svc, msgid, pushfd);

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error_svc_unlock;
	}

	mutex_lock(&msg->m_mutex);

	if (__is_message_interrupted(msg)) {
		mutex_unlock(&msg->m_mutex);
		__message_put(msg);
		ret = -EINTR;
		goto error_svc_unlock;
	}

	filp = fget(pushfd);
	if (!filp) {
		ret = -EINVAL;
		goto error_msg_unlock;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(msg->m_task, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error_msg_unlock;
	}

	servicefs_fd_install(msg->m_task, newfd, filp);

	mutex_unlock(&msg->m_mutex);

	__message_complete(msg, newfd);

	mutex_unlock(&svc->s_mutex);
	return 0;

error_msg_unlock:
	mutex_unlock(&msg->m_mutex);
error_svc_unlock:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

int servicefs_msg_push_fd(struct service *svc, int msgid, unsigned int pushfd)
{
	struct message *msg;
	int ret = 0;
	int newfd;
	struct file *filp;

	pr_debug("svc=%p msgid=%d pushfd=%u\n", svc, msgid, pushfd);

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error_svc_unlock;
	}

	mutex_lock(&msg->m_mutex);

	if (__is_message_interrupted(msg)) {
		ret = -EINTR;
		goto error_msg_unlock;
	}

	filp = fget(pushfd);
	if (!filp) {
		ret = -EINVAL;
		goto error_msg_unlock;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(msg->m_task, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error_msg_unlock;
	}

	servicefs_fd_install(msg->m_task, newfd, filp);
	ret = newfd;

error_msg_unlock:
	mutex_unlock(&msg->m_mutex);
error_svc_unlock:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

int servicefs_msg_get_fd(struct service *svc, int msgid, unsigned int index)
{
	struct message *msg;
	int ret = 0;
	int newfd;
	int getfd;
	struct file *filp;

	pr_debug("svc=%p msgid=%d index=%u\n", svc, msgid, index);

	mutex_lock(&svc->s_mutex);

	msg = __lookup_message(svc, msgid);
	if (!msg) {
		ret = -ENOENT;
		goto error_svc_unlock;
	}

	mutex_lock(&msg->m_mutex);

	if (__is_message_interrupted(msg)) {
		ret = -EINTR;
		goto error_msg_unlock;
	}

	if (index >= msg->m_fdcnt) {
		ret = -EINVAL;
		goto error_msg_unlock;
	}
	getfd = msg->m_fds[index];

	filp = servicefs_fget(msg->m_task, getfd);
	if (!filp) {
		ret = -EBADF;
		goto error_msg_unlock;
	}

	// TODO: add selinux check for FD__USE and other relevant checks
	// look in security/selinux/hooks.c

	newfd = servicefs_get_unused_fd_flags(current, 0);
	if (newfd < 0) {
		fput(filp);
		ret = newfd;
		goto error_msg_unlock;
	}

	servicefs_fd_install(current, newfd, filp);
	ret = newfd;

error_msg_unlock:
	mutex_unlock(&msg->m_mutex);
error_svc_unlock:
	mutex_unlock(&svc->s_mutex);
	return ret;
}

int servicefs_mod_channel_events(struct service *svc, int cid,
		int clr, int set)
{
	int ret = 0;
	struct channel *c;

	mutex_lock(&svc->s_mutex);

	c = __lookup_channel(svc, cid);
	if (!c) {
		ret = -ENOENT;
		goto done;
	}

	c->c_events &= ~clr; // clear first
	c->c_events |= set; // then set, so set bits always take effect

	wake_up_poll(&c->c_waitqueue, c->c_events);

done:
	mutex_unlock(&svc->s_mutex);
	return ret;

}

ssize_t servicefs_msg_sendv(struct channel *c, int op, const iov *svec,
		size_t scnt, const iov *rvec, size_t rcnt,
		const int *fds, size_t fdcnt, long task_state)
{
	struct message *msg;
	struct service *svc;
	kuid_t kuid;
	kgid_t kgid;
	int ret;

	svc = c->c_service;
	BUG_ON(!svc);

	msg = kmem_cache_alloc(message_cache, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	INIT_LIST_HEAD(&msg->m_messages_node);
	kref_init(&msg->m_ref); // implicit reference
	mutex_init(&msg->m_mutex);

	msg->m_id = MESSAGE_NO_ID;
	msg->m_op = op;
	msg->m_channel = c;
	msg->m_service = svc;
	msg->m_completed = false;
	msg->m_interrupted = false;

	init_waitqueue_head(&msg->m_waitqueue);

	msg->m_priority = task_nice(current);
	msg->m_status = 0;

	iov_buffer_uinit(&msg->m_sbuf, svec, scnt);
	iov_buffer_uinit(&msg->m_rbuf, rvec, rcnt);

	msg->m_fds = fds;
	msg->m_fdcnt = fdcnt;

	/*
	 * Get a ref to the current task and stash the task struct for later use.
	 * The task struct ref is dropped by the message kref release function
	 * when the last ref to the message is dropped.
	 */
	get_task_struct(current);
	msg->m_task = current;

	/* Grab the euid/egid of the sending process. */
	current_euid_egid(&kuid, &kgid);
	msg->m_euid = __kuid_val(kuid);
	msg->m_egid = __kgid_val(kgid);

	/*
	 * Record the thread and process ids now in case the process or thread
	 * dies before the message description is received. This behavior is
	 * required to properly identify channels closed during process
	 * termination.
	 */
	msg->m_pid = pid_vnr(task_tgid(msg->m_task));
	msg->m_tid = pid_vnr(task_pid(msg->m_task));

	pr_debug("msg=%p op=%d scnt=%zu slen=%zu rcnt=%zu rlen=%zu fdcnt=%zu\n",
			msg, op, msg->m_sbuf.i_cnt, msg->m_sbuf.i_len,
			msg->m_rbuf.i_cnt, msg->m_rbuf.i_len, msg->m_fdcnt);

	mutex_lock(&svc->s_mutex);

	/*
	 * Messages can't be sent after a channel has shutdown, but channels
	 * remain until the last reference to their open file description is
	 * closed. Make sure we don't queue any messages after the channel
	 * is shutdown.
	 */
	if (__is_channel_canceled(c) || __is_service_canceled(svc)) {
		pr_debug("cid=%d channel_canceled=%d service_canceled=%d\n",
				c->c_id, __is_channel_canceled(c), __is_service_canceled(svc));
		__message_put(msg);
		mutex_unlock(&svc->s_mutex);
		return -ESHUTDOWN;
	}

	list_add_tail(&msg->m_messages_node, &svc->s_messages);

	pr_debug("svc=%p s_wqreceivers=%s s_wqselect=%s\n", svc,
		waitqueue_active(&svc->s_wqreceivers) ? "active" : "empty",
		waitqueue_active(&svc->s_wqselect) ? "active" : "empty");

	if (waitqueue_active(&svc->s_wqreceivers))
		wake_up(&svc->s_wqreceivers);
	else
		wake_up_poll(&svc->s_wqselect, POLLIN | POLLRDNORM);

	mutex_unlock(&svc->s_mutex);

	/* wait for message reply/cancel or a signal if interruptible */
	if (task_state == TASK_INTERRUPTIBLE)
		wait_event_interruptible(msg->m_waitqueue, is_message_completed(msg));
	else
		wait_event(msg->m_waitqueue, is_message_completed(msg));

	mutex_lock(&msg->m_mutex);

	if (!msg->m_completed) {
		msg->m_interrupted = true;
		msg->m_status = -EINTR;
	}

	ret = msg->m_status;

	mutex_unlock(&msg->m_mutex);

	/* release the message now that we're done with it */
	message_put(svc, msg);

	pr_debug("ret=%d\n", ret);
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

	impulse = kmem_cache_alloc(impulse_cache, GFP_KERNEL);
	if (!impulse)
		return -ENOMEM;

	INIT_LIST_HEAD(&impulse->i_impulses_node);
	impulse->i_op = op;
	impulse->i_channel = c;
	impulse->i_service = svc;
	impulse->i_len = len;

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

	mutex_lock(&svc->s_mutex);

	/*
	 * Messages can't be sent after a channel has shutdown, but channels
	 * remain until the last reference to their open file description is
	 * closed. Make sure we don't queue any messages after the channel
	 * is shutdown.
	 */
	if (__is_channel_canceled(c) || __is_service_canceled(svc)) {
		pr_debug("cid=%d channel_canceled=%d service_canceled=%d\n",
				c->c_id, __is_channel_canceled(c), __is_service_canceled(svc));
		impulse_free(impulse);
		mutex_unlock(&svc->s_mutex);
		return -ESHUTDOWN;
	}

	list_add_tail(&impulse->i_impulses_node, &svc->s_impulses);

	pr_debug("svc=%p s_wqreceivers=%s s_wqselect=%s\n", svc,
		waitqueue_active(&svc->s_wqreceivers) ? "active" : "empty",
		waitqueue_active(&svc->s_wqselect) ? "active" : "empty");

	if (waitqueue_active(&svc->s_wqreceivers))
		wake_up(&svc->s_wqreceivers);
	else
		wake_up_poll(&svc->s_wqselect, POLLIN | POLLRDNORM);

	mutex_unlock(&svc->s_mutex);

	return 0;
}

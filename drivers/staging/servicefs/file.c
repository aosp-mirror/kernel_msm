/*
 *  file.c - part of servicefs, a small filesystem for service namespaces.
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
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/pagemap.h>
#include <linux/namei.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/compat.h>
#include <uapi/linux/servicefs.h>

#include "servicefs_private.h"

static int initial_open(struct inode *inode, struct file *filp);
static int initial_release(struct inode *inode, struct file *filp);

static long service_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg);
static unsigned int service_poll(struct file *filp, poll_table *wait);
static int service_release(struct inode *inode, struct file *filp);

static long channel_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg);
static unsigned int channel_poll(struct file *filp, poll_table *wait);
static int channel_release(struct inode *inode, struct file *filp);
static ssize_t channel_read(struct file *filp, char __user *buf,
		size_t len, loff_t *ppos);
static ssize_t channel_write(struct file *filp, const char __user *buf,
		size_t len, loff_t *ppos);

const struct file_operations initial_file_operations = {
	.owner          = THIS_MODULE,
	.open           = initial_open,
	.release        = initial_release,
};

static const struct file_operations service_file_operations = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = service_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = service_ioctl,
#endif
	.release        = service_release,
	.poll           = service_poll,
};

static const struct file_operations channel_file_operations = {
	.owner          = THIS_MODULE,
	.llseek         = NULL,
	.read           = channel_read,
	.write          = channel_write,
	.poll           = channel_poll,
	.unlocked_ioctl = channel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = channel_ioctl,
#endif
	.mmap           = NULL,
	.flush          = NULL,
	.release        = channel_release,
};

/**
 * initial_open - initial open file op for service inodes
 * @inode: pointer to the inode associated with the service.
 * @filp: pointer to the new file object being opened on this inode.
 *
 * Handles file open operations on service inodes. This first open
 * of a service inode (that one that creates it) becomes the
 * service side of the endpoint; this file object gets assigned
 * service_file_operations and respond to service APIs. Subsequent
 * opens become clients (channels) of the service; these file
 * objects are assigned channel_file_operations and respond to the
 * client APIs.
 */
static int initial_open(struct inode *inode, struct file *filp)
{
	int count;
	struct service *svc = inode->i_private;
	if (!svc)
		return -EINVAL;

	count = atomic_add_return(1, &svc->s_count);
	pr_debug("count=%d\n", count);

	if (count == 1) {
		filp->private_data = svc;
		filp->f_op = &service_file_operations;
		svc->s_filp = filp; // DO NOT take a ref to the file struct.
	} else {
		struct channel *c = channel_new(svc);
		if (!c)
			return -EINVAL; // TODO: propagate error code from channel_new()

		filp->private_data = c;
		filp->f_op = &channel_file_operations;

		/*
		 * TODO: Remove this last uninterruptible sleep by addressing the two
		 * conditions where either the open message has been received or it
		 * hasn't.
		 */
		return servicefs_msg_sendv_uninterruptible(c, SERVICEFS_OP_UNIX_OPEN,
				NULL, 0, NULL, 0, NULL, 0);
	}

	return 0;
}

/**
 * create_channel_open - open called by the service-side create channel API
 * @inode: pointer to the inode associate with the service.
 * @filp: pointer to the new file object being opened on this inode.
 *
 * Handles file open operatons on service inodes whenever a service is the
 * initiator of the channel creation. The main difference between this
 * function and initial_open is that it always creates a new channel and a
 * creation message is never sent, which could deadlock single-threaded
 * services.
 */
static int create_channel_open(struct inode *inode, struct file *filp)
{
	int count;
	struct channel *c;
	struct service *svc = inode->i_private;
	if (!svc)
		return -EINVAL;

	count = atomic_add_return(1, &svc->s_count);
	pr_debug("count=%d\n", count);
	BUG_ON(count < 1);

	c = channel_new(svc);
	if (!c)
		return -EINVAL; // TODO: propagate error code from channel_new()

	filp->private_data = c;
	filp->f_op = &initial_file_operations;

	return 0;
}

void servicefs_complete_channel_setup(struct file *filp)
{
	BUG_ON(filp->f_op != &initial_file_operations);
	filp->f_op = &channel_file_operations;
}

/**
 * initial_release - initial release op for service inodes
 * @inode: pointer to the inode associated with the service.
 * @filp: pointer to the file object being released on this inode.
 *
 * This is unused. During open the file ops of file objects for
 * this inode are reassigned to either &service_file_operations
 * or &channel_file_operations. These specify specialized release
 * functions services and channels, respectively.
 */
static int initial_release(struct inode *inode, struct file *filp)
{
	int count;
	struct service *svc = inode->i_private;
	if (!svc)
		return -EINVAL;

	count = atomic_add_return(-1, &svc->s_count);
	pr_debug("count=%d\n", count);

	pr_warn("File was cleaned up before being completely set up!!!\n");

	return 0;
}

/**
 * service_ioctl - dispatch service ioctl APIs
 * @filp: pointer to the file struct associated with the service.
 * @cmd: the requested ioctl number
 * @arg: the ioctl argument; this is interpreted as a pointer
 *       to a user buffer in most commands.
 *
 * Handles reading arguments from userspace, validation of iovecs
 * when relevant, and dispatch to service API handlers. Fast
 * iovecs and allocation/deallocation of slow iovecs is also
 * handled.
 */
static long service_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int ret;
	struct service *svc = filp->private_data;
	void __user *ubuf = (void __user *) arg;
	struct iovec iovstack[UIO_FASTIOV];
	struct iovec *vec = iovstack;

	BUG_ON(svc == NULL);

	switch (cmd) {
		case SERVICEFS_SET_SERVICE_CONTEXT:
			ret = servicefs_set_service_context(svc, arg);
			break;

		case SERVICEFS_SET_CHANNEL_CONTEXT: {
			struct servicefs_set_channel_context_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_set_channel_context(svc, params.cid, params.ctx);
		}
		break;

		case SERVICEFS_MSG_RECV: {
			long timeout = filp->f_flags & O_NONBLOCK ? 0 : MAX_SCHEDULE_TIMEOUT;
			ret = servicefs_msg_recv(svc, ubuf, timeout);
		}
		break;

		case SERVICEFS_MSG_READV: {
			struct servicefs_msg_rwvec_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			if (is_compat_task()) {
				ret = compat_rw_copy_check_uvector(WRITE,
						(const struct compat_iovec __user *) params.vec,
						params.len, ARRAY_SIZE(iovstack), iovstack, &vec);
			} else {
				ret = rw_copy_check_uvector(WRITE,
						(const struct iovec __user *) params.vec, params.len,
						ARRAY_SIZE(iovstack), iovstack, &vec);
			}

			if (ret < 0)
				goto error;

			ret = servicefs_msg_readv(svc, params.msgid, vec, params.len);
		}
		break;

		case SERVICEFS_MSG_WRITEV: {
			struct servicefs_msg_rwvec_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			if (is_compat_task()) {
				ret = compat_rw_copy_check_uvector(READ,
						(const struct compat_iovec __user *) params.vec,
						params.len, ARRAY_SIZE(iovstack), iovstack, &vec);
			} else {
				ret = rw_copy_check_uvector(READ,
						(const struct iovec __user *) params.vec, params.len,
						ARRAY_SIZE(iovstack), iovstack, &vec);
			}

			if (ret < 0)
				goto error;

			ret = servicefs_msg_writev(svc, params.msgid, vec, params.len);
		}
		break;

		case SERVICEFS_MSG_SEEK: {
			struct servicefs_msg_seek_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_msg_seek(svc, params.msgid, params.offset,
					params.whence);
		}
		break;

		case SERVICEFS_MSG_BUSV: {
			struct servicefs_msg_busv_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_msg_busv(svc, params.dst_msgid, params.dst_offset,
					params.src_msgid, params.src_offset, params.len);
		}
		break;

		case SERVICEFS_MSG_REPLY: {
			struct servicefs_msg_reply_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_msg_reply(svc, params.msgid, params.retcode);
		}
		break;

		case SERVICEFS_MSG_REPLY_FD: {
			struct servicefs_msg_reply_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_msg_reply_fd(svc, params.msgid, params.retcode);
		}
		break;

		case SERVICEFS_MOD_CHANNEL_EVENTS: {
			struct servicefs_mod_channel_events_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_mod_channel_events(svc, params.cid,
					params.clr, params.set);
		}
		break;

		case SERVICEFS_MSG_PUSH_FD: {
			struct servicefs_msg_push_fd_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_msg_push_fd(svc, params.msgid, params.pushfd);
		}
		break;

		case SERVICEFS_MSG_GET_FD: {
			struct servicefs_msg_get_fd_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_msg_get_fd(svc, params.msgid, params.index);
		}
		break;

		case SERVICEFS_PUSH_CHANNEL: {
			struct servicefs_push_channel_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_push_channel(svc, params.svcfd, params.msgid,
					params.flags, (__s32 __user *) params.cid, params.ctx);
		}
		break;

		case SERVICEFS_CLOSE_CHANNEL:
			ret = servicefs_close_channel(svc, arg);
			break;

		case SERVICEFS_CHECK_CHANNEL: {
			struct servicefs_check_channel_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			ret = servicefs_check_channel(svc, params.svcfd, params.msgid,
					params.index, (__s32 __user *) params.cid,
					(__u64 __user *) params.ctx);
		}
		break;

		case SERVICEFS_CANCEL_SERVICE:
			ret = service_cancel(svc);
			break;

		default:
			pr_debug("cmd=%08x nr=%u size=%u\n", cmd, _IOC_NR(cmd),
					_IOC_SIZE(cmd));
			ret = -ENOTTY;
			break;
	}

error:
	if (vec != iovstack)
		kfree(vec);

	return ret;
}

struct file *servicefs_create_channel(struct file *svc_file, int flags)
{
	struct file *file;
	int ret;

	file = alloc_file(&svc_file->f_path, FMODE_READ,
			&initial_file_operations);
	if (IS_ERR(file))
		return file;

	file->f_flags = O_RDONLY | (flags & (O_NONBLOCK | O_CLOEXEC));

	ret = create_channel_open(svc_file->f_path.dentry->d_inode, file);
	if (ret < 0) {
		put_filp(file);
		return ERR_PTR(ret);
	}

	/* account for the new file's references to the dentry and mount */
	path_get(&file->f_path);

	return file;
}

/**
 * servicefs_get_channel_from_file - get the channel struct from a file
 * @filp: pointer to the file that may or may not be a channel.
 *
 * Returns a pointer to the channel struct if the file represents a servicefs
 * channel, NULL otherwise.
 */
struct channel *servicefs_get_channel_from_file(struct file *filp)
{
	if (filp->f_op != &channel_file_operations)
		return NULL;
	else
		return filp->private_data;
}

/**
 * servicefs_get_service_from_file - get the service struct from a file
 * @filp: pointer to the file that may or may not be a service.
 *
 * Returns a pointer to the service struct if the file represents a servicefs
 * service, NULL otherwise.
 */
struct service *servicefs_get_service_from_file(struct file *filp)
{
	if (filp->f_op != &service_file_operations)
		return NULL;
	else
		return filp->private_data;
}

/**
 * service_poll - handle the poll file op for services
 * @filp: pointer to the file object associated with the service.
 * @wait: pointer to the poll table to add entries to.
 *
 * Adds the service's s_wqselect to the poll table and checks for
 * presence of messages.
 */
static unsigned int service_poll(struct file *filp, poll_table *wait)
{
	int mask = 0;
	struct service *svc = filp->private_data;
	BUG_ON(svc == NULL);

	poll_wait(filp, &svc->s_wqselect, wait);

	read_lock(&svc->s_message_lock);

	if (!list_empty(&svc->s_impulses) || !list_empty(&svc->s_messages))
		mask = POLLIN | POLLRDNORM;

	read_unlock(&svc->s_message_lock);

	pr_debug("svc=%p mask=%x\n", svc, mask);
	return mask;
}

/**
 * service_release - release file op for services
 * @inode: pointer to the inode associated with the service.
 * @filp: pointer to the file object being released on this inode.
 *
 * Removes the dentry for this service, removing the service from
 * the namespace. The service is canceled, which starts cleanup
 * of the connected channels, receivers, and messages.
 *
 * The service structure itself is not released until the inode
 * is destroyed to avoid races between this and the client_release
 * function.
 */
static int service_release(struct inode *inode, struct file *filp)
{
	int count;
	struct service *svc = inode->i_private;
	if (!svc)
		return -EINVAL;

	count = atomic_add_return(-1, &svc->s_count);
	pr_debug("count=%d\n", count);
	service_cancel(svc);

	return 0;
}

/**
 * channel_ioctl - dispatch channel (client) ioctl APIs
 * @filp: pointer to the file struct associated with the service.
 * @cmd: the requested ioctl number
 * @arg: the ioctl argument; this is interpreted as a pointer
 *       to a user buffer in most commands.
 *
 * Handles reading arguments from userspace, validation of iovecs
 * when relevant, and dispatch to channel API handlers. Fast
 * iovecs and allocation/deallocation of slow iovecs is also
 * handled.
 */
static long channel_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int ret;
	struct channel *c = filp->private_data;
	void __user *ubuf = (void __user *) arg;
	struct iovec siovstack[UIO_FASTIOV];
	struct iovec riovstack[UIO_FASTIOV];
	struct iovec *svec = siovstack;
	struct iovec *rvec = riovstack;
	__s32 fdstack[UIO_FASTIOV];
	__s32 *fds = fdstack;

	BUG_ON(!c);

	switch (cmd) {
		case SERVICEFS_MSG_SENDV: {
			struct servicefs_msg_sendv_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			/*
			 * prevent spoofing open/close ops and enforce that NULL arrays
			 * must have 0 element counts.
			 */
			if ((params.op == SERVICEFS_OP_UNIX_OPEN)
				    || (params.op == SERVICEFS_OP_UNIX_CLOSE)
				    || (!params.svec && params.scnt)
				    || (!params.rvec && params.rcnt)
				    || (!params.fds && params.fdcnt)) {
				ret = -EINVAL;
				goto error;
			}

			if (params.svec) {
				if (is_compat_task()) {
					ret = compat_rw_copy_check_uvector(READ,
							(const struct compat_iovec __user *) params.svec,
							params.scnt, ARRAY_SIZE(siovstack), siovstack,
							&svec);
				} else {
					ret = rw_copy_check_uvector(READ,
							(const struct iovec __user *) params.svec,
							params.scnt, ARRAY_SIZE(siovstack), siovstack,
							&svec);
				}

				if (ret < 0)
					goto error;
			} else {
				svec = NULL;
			}

			if (params.rvec) {
				if (is_compat_task()) {
					ret = compat_rw_copy_check_uvector(WRITE,
							(const struct compat_iovec __user *) params.rvec,
							params.rcnt, ARRAY_SIZE(riovstack), riovstack,
							&rvec);
				} else {
					ret = rw_copy_check_uvector(WRITE,
							(const struct iovec __user *) params.rvec,
							params.rcnt, ARRAY_SIZE(riovstack), riovstack,
							&rvec);
				}

				if (ret < 0)
					goto error;
			} else {
				rvec = NULL;
			}

			if (params.fds) {
				if (params.fdcnt > UIO_MAXIOV) {
					ret = -EINVAL;
					goto error;
				}

				if (params.fdcnt > UIO_FASTIOV) {
					fds = kmalloc(sizeof(__s32) * params.fdcnt, GFP_KERNEL);
					if (fds == NULL) {
						ret = -ENOMEM;
						goto error;
					}
				}

				if (copy_from_user(fds, (const __user __s32 *) params.fds,
							sizeof(__s32) * params.fdcnt)) {
					ret = -EFAULT;
					goto error;
				}
			} else {
				fds = NULL;
			}

			ret = servicefs_msg_sendv_interruptible(c, params.op,
					svec, params.scnt, rvec, params.rcnt,
					fds, params.fdcnt);
		}
		break;

		case SERVICEFS_MSG_SEND_IMPULSE: {
			struct servicefs_msg_send_impulse_struct params;

			if (copy_from_user(&params, ubuf, sizeof(params))) {
				ret = -EFAULT;
				goto error;
			}

			/*
			 * prevent spoofing open/close ops and enforce that NULL arrays
			 * must have 0 element counts.
			 */
			if ((params.op == SERVICEFS_OP_UNIX_OPEN)
				    || (params.op == SERVICEFS_OP_UNIX_CLOSE)
				    || (!params.buf && params.len)) {
				ret = -EINVAL;
				goto error;
			}

			ret = servicefs_msg_send_impulse(c, params.op,
					(void __user *) params.buf, params.len);
		}
		break;

		default:
			ret = -ENOTTY;
			break;
	}

error:
	if (svec != siovstack)
		kfree(svec);
	if (rvec != riovstack)
		kfree(rvec);
	if (fds != fdstack)
		kfree(fds);

	return ret;
}

/**
 * channel_poll - handle the poll file op for channels
 * @filp: pointer to the file object associated with the channel.
 * @wait: pointer to the poll table to add entries to.
 *
 * Adds the channel's c_waitqueue to the poll table and checks for
 * any events set on the channel.
 */
static unsigned int channel_poll(struct file *filp, poll_table *wait)
{
	int mask;
	struct service *svc;
	struct channel *c = filp->private_data;
	BUG_ON(!c);

	svc = c->c_service;
	BUG_ON(!svc);

	poll_wait(filp, &c->c_waitqueue, wait);
	mask = atomic_read(&c->c_events);

	pr_debug("mask=%08x\n", mask);
	return mask;
}

static int channel_release(struct inode *inode, struct file *filp)
{
	int count;
	struct channel *c = filp->private_data;
	struct service *svc = inode->i_private;
	if (!svc)
		return -EINVAL;

	BUG_ON(!c);

	count = atomic_add_return(-1, &svc->s_count);
	pr_debug("count=%d\n", count);

	channel_remove(c);

	return 0;
}

static ssize_t channel_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	ssize_t ret;
	const struct iovec rvec[1] = {
		{ .iov_base = buf, .iov_len = len },
	};
	struct channel *c = filp->private_data;
	BUG_ON(!c);

	pr_debug("cid=%d buf=%p len=%zu\n", c->c_id, buf, len);

	ret = servicefs_msg_sendv_interruptible(c, SERVICEFS_OP_UNIX_READ,
			NULL, 0, rvec, 1, NULL, 0);
	return ret;
}

static ssize_t channel_write(struct file *filp, const char __user *buf, size_t len, loff_t *ppos)
{
	ssize_t ret;
	const struct iovec svec[1] = {
		{ .iov_base = (char __user *) buf, .iov_len = len },
	};
	struct channel *c = filp->private_data;
	BUG_ON(!c);

	pr_debug("cid=%d buf=%p len=%zu\n", c->c_id, buf, len);

	ret = servicefs_msg_sendv_interruptible(c, SERVICEFS_OP_UNIX_WRITE,
			svec, 1, NULL, 0, NULL, 0);
	return ret;
}

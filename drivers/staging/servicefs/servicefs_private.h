#ifndef _SERVICEFS_PRIVATE_H
#define _SERVICEFS_PRIVATE_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/rwlock.h>
#include <linux/idr.h>
#include <linux/uio.h>
#include <linux/wait.h>
#include <linux/kref.h>
#include <linux/fs.h>

#include "iov_buffer.h"

struct service {
	atomic_t             s_count;

	rwlock_t             s_message_lock;   // protects message lists and id allocator
	struct semaphore     s_queue_messages;

	struct idr           s_message_idr;    // message id allocator
	int                  s_message_start;
	struct list_head     s_impulses;       // pending async messages
	struct list_head     s_messages;       // pending sync messages (blocked client threads)

	rwlock_t             s_channel_lock;   // protects channel id allocator
	struct idr           s_channel_idr;    // channel id allocator
	int                  s_channel_start;

	wait_queue_head_t    s_wqselect;       // wait queue for poll/select

#define SERVICE_FLAGS_DEFAULT              (0)
#define SERVICE_FLAGS_CANCELED             (1<<0)
#define SERVICE_FLAGS_COUNTER_OFFSET       (1)
#define SERVICE_FLAGS_COUNTER              (1<<SERVICE_FLAGS_COUNTER_OFFSET)
	// bit 0 is canceled bit; bits 31-1 are the thread counter
	atomic_t             s_flags;

	// TODO(eieio): figure out what to do about forking/execing
	__u64                s_context;        // userspace context pointer
	struct file *        s_filp;           // does not hold a ref to the file
};

struct channel {
	struct kref          c_ref;
	struct service *     c_service;

	int                  c_id;

	atomic_t             c_events;         // events for poll/select
	wait_queue_head_t    c_waitqueue;      // wait queue for poll/select

#define CHANNEL_FLAGS_CANCELED             (1<<0)
#define CHANNEL_FLAGS_THREAD_POOL          (1<<1)
	atomic_t             c_flags;

	// TODO(eieio): figure out what to do about forking/execing
	__u64                c_context;
};

struct message {
	struct list_head     m_messages_node;  // hangs on s_messages

#define MESSAGE_NO_ID                     (-1)
	int                  m_id;

	struct kref          m_ref;            // protected by service.s_mutex

	int                  m_priority;       // boost priority
	struct task_struct * m_task;           // blocked client task
	pid_t                m_pid;            // sender tgid
	pid_t                m_tid;            // sender pid
	uid_t                m_euid;           // sender euid
	gid_t                m_egid;           // sender egid
	wait_queue_head_t    m_waitqueue;      // wait queue for sender

	struct channel *     m_channel;

	int                  m_op;

#define MESSAGE_FLAGS_PENDING_RECEIVE      (0)
#define MESSAGE_FLAGS_PENDING_REPLY        (1<<0)
#define MESSAGE_FLAGS_COMPLETED            (1<<1)
#define MESSAGE_FLAGS_INTERRUPTED          (1<<2)
#define MESSAGE_FLAGS_CANCELED             (1<<3)
	atomic_t             m_flags;

	/* state below may be modified by multiple service threads */
	struct mutex         m_mutex;          // sync service access

	struct iov_buffer    m_sbuf;           // send buffer vecs
	struct iov_buffer    m_rbuf;           // receive buffer vecs

	const __s32 *        m_fds;
	size_t               m_fdcnt;

	ssize_t              m_status;         // return code

	struct list_head     m_pending_fds;
};

struct pending_fd {
	struct list_head    p_pending_fds_node;// hangs on m_pending_fds
	int                 p_fd;
	struct file *       p_filp;            // holds a ref
};

struct impulse {
	struct list_head     i_impulses_node;  // hangs on s_impulses

	pid_t                i_pid;            // sender tgid
	pid_t                i_tid;            // sender pid
	uid_t                i_euid;           // sender euid
	gid_t                i_egid;           // sender egid

	struct channel *     i_channel;

	int                  i_op;
	long                 i_data[4];
	uint8_t              i_len;
};

/*
 * Initialize caches.
 */
int servicefs_cache_init(void);

/*
 * Creation, status, and removal of services.
 */
struct service *service_new(void);
int service_cancel(struct service *svc);
void service_free(struct service *svc);

static inline bool is_service_canceled(struct service *svc)
{
	return !!(atomic_read(&svc->s_flags) & SERVICE_FLAGS_CANCELED);
}

/*
 * Creation, status, and removal of channels.
 */
struct channel *channel_new(struct service *svc);
void channel_remove(struct channel *c);

static inline bool is_channel_canceled(struct channel *c)
{
	return !!(atomic_read(&c->c_flags) & CHANNEL_FLAGS_CANCELED);
}

/*
 * Status of messages.
 */
static inline bool is_message_completed(struct message *m)
{
	return !!(atomic_read(&m->m_flags) & MESSAGE_FLAGS_COMPLETED);
}

static inline bool is_message_interrupted(struct message *m)
{
	return !!(atomic_read(&m->m_flags) & MESSAGE_FLAGS_INTERRUPTED);
}

static inline bool is_message_canceled(struct message *m)
{
	return !!(atomic_read(&m->m_flags) & MESSAGE_FLAGS_CANCELED);
}

static inline bool is_message_pending_reply(struct message *m)
{
	return !!(atomic_read(&m->m_flags) & MESSAGE_FLAGS_PENDING_REPLY);
}

/*
 * Removal of a service's dentry.
 */
void servicefs_remove_dentry(struct dentry *dentry);

/*
 * Utility to create a new channel and its associated file.
 */
struct file *servicefs_create_channel(struct file *svc_file, int flags);
void servicefs_complete_channel_setup(struct file *filp);

/*
 * Get a channel struct from a file struct, if it's actually a channel.
 */
struct channel *servicefs_get_channel_from_file(struct file *filp);

/*
 * Get a service struct from a file struct, if it's actually a service.
 */
struct service *servicefs_get_service_from_file(struct file *filp);

/*
 * Service API handlers. Called by service_ioctl().
 */
struct servicefs_msg_info_struct;

int servicefs_push_channel(struct service *svc, int svcfd, int msgid,
		int flags, __s32 __user *cid, __u64 ctx);
int servicefs_close_channel(struct service *svc, int cid);
int servicefs_check_channel(struct service *svc, int svcfd, int msgid,
		int index, __s32 __user *cid, __u64 __user *ctx);

int servicefs_set_service_context(struct service *svc, __u64 ctx);
int servicefs_set_channel_context(struct service *svc, int cid, __u64 ctx);
int servicefs_msg_recv(struct service *svc,
		struct servicefs_msg_info_struct __user *msg_info, long timeout);
ssize_t servicefs_msg_readv(struct service *svc, int msgid,
		const struct iovec *vec, size_t cnt);
ssize_t servicefs_msg_writev(struct service *svc, int msgid,
		const struct iovec *vec, size_t cnt);
int servicefs_msg_seek(struct service *svc, int msgid, long offset, int whence);
ssize_t servicefs_msg_busv(struct service *svc, int dst_msgid, long dst_off,
		int src_msgid, long src_off, size_t len);
int servicefs_msg_reply(struct service *svc, int msgid, ssize_t retcode);
int servicefs_msg_reply_fd(struct service *svc, int msgid, unsigned int pushfd);
int servicefs_mod_channel_events(struct service *svc, int cid,
		int clr, int set);
int servicefs_msg_push_fd(struct service *svc, int msgid, unsigned int pushfd);
int servicefs_msg_get_fd(struct service *svc, int msgid, unsigned int index);

/*
 * Client API handlers. Called by client_ioctl() and various client file ops.
 */
ssize_t servicefs_msg_sendv(struct channel *c, int op,
		const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt,
		const __s32 *fds, size_t fdcnt, long task_state);
int servicefs_msg_send_impulse(struct channel *c, int op,
		void __user *buf, size_t len);

static inline ssize_t servicefs_msg_sendv_interruptible(
		struct channel *c, int op,
		const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt,
		const __s32 *fds, size_t fdcnt)
{
	return servicefs_msg_sendv(c, op, svec, scnt, rvec, rcnt, fds, fdcnt,
			TASK_INTERRUPTIBLE);
}

static inline ssize_t servicefs_msg_sendv_uninterruptible(
		struct channel *c, int op,
		const struct iovec *svec, size_t scnt,
		const struct iovec *rvec, size_t rcnt,
		const __s32 *fds, size_t fdcnt)
{
	return servicefs_msg_sendv(c, op, svec, scnt, rvec, rcnt, fds, fdcnt,
			TASK_UNINTERRUPTIBLE);
}

/*
 * Data transfer utilities for moving data between address spaces.
 */
ssize_t vm_transfer_to_remote(struct iov_buffer *remote,
		struct iov_buffer *local, struct task_struct *task, bool *remote_fault);
ssize_t vm_transfer_from_remote(struct iov_buffer *remote,
		struct iov_buffer *local, struct task_struct *task, bool *remote_fault);

/*
 * File descriptor and file object utilities. These are analogs of
 * common kernel functions, modified to act on the specified task
 * instead of implicitly on current.
 */
int servicefs_get_unused_fd_flags(struct task_struct *task, int flags);
void servicefs_fd_install(struct task_struct *task, unsigned int fd,
		struct file *file);
void servicefs_put_unused_fd(struct task_struct *task, unsigned int fd);
struct file *servicefs_fget(struct task_struct *task, unsigned int fd);

#endif


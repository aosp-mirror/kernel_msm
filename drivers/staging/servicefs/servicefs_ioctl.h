#ifndef _SERVICEFS_IOCTL_H
#define _SERVICEFS_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/uio.h>
#include <linux/file.h>

typedef struct iovec iov;

struct servicefs_set_channel_context_struct {
	int           cid;
	void *        ctx;
};

struct servicefs_msg_sendv_struct {
	int           op;
	const iov *   svec;
	size_t        scnt;
	const iov *   rvec;
	size_t        rcnt;
	const int *   fds;
	size_t        fdcnt;
};

struct servicefs_msg_info_struct {
	int           pid;
	int           tid;
	int           cid;
	int           mid;
	int           euid;
	int           egid;
	void *        service_private;
	void *        channel_private;
	int           op;
	int           flags;
	size_t        send_len;
	size_t        recv_len;
	size_t        fd_count;
	long          impulse[4];
};

struct servicefs_msg_rwvec_struct {
	int           msgid;
	const iov *   vec;
	size_t        len;
};

struct servicefs_msg_seek_struct {
	int           msgid;
	long          offset;
	int           whence;
};

struct servicefs_msg_busv_struct {
	int           dst_msgid;
	long          dst_offset;
	int           src_msgid;
	long          src_offset;
	size_t        len;
};

struct servicefs_msg_reply_struct {
	int           msgid;
	int           retcode;
};

struct servicefs_msg_send_impulse_struct {
	int           op;
	void *        buf;
	size_t        len;
};

struct servicefs_mod_channel_events_struct {
	int            cid;
	int            clr;
	int            set;
};

struct servicefs_msg_push_fd_struct {
	int            msgid;
	unsigned int   pushfd;
};

struct servicefs_msg_get_fd_struct {
	int           msgid;
	unsigned int  index;
};

struct servicefs_push_channel_struct {
	int           svcfd;
	int           msgid;
	int           flags;
	int *         cid;
	void *        ctx;
};

struct servicefs_check_channel_struct {
	int           svcfd;
	int           msgid;
	int           index;
	int *         cid;
	void *        ctx;
};

#define SERVICEFS_SET_SERVICE_CONTEXT       _IOW ('x',  0, void *)
#define SERVICEFS_SET_CHANNEL_CONTEXT       _IOW ('x',  1, struct servicefs_set_channel_context_struct)
#define SERVICEFS_MSG_SENDV                 _IOWR('x',  2, struct servicefs_msg_sendv_struct)
#define SERVICEFS_MSG_RECV                  _IOR ('x',  3, struct servicefs_msg_info_struct)
#define SERVICEFS_MSG_READV                 _IOR ('x',  4, struct servicefs_msg_rwvec_struct)
#define SERVICEFS_MSG_WRITEV                _IOW ('x',  5, struct servicefs_msg_rwvec_struct)
#define SERVICEFS_MSG_SEEK                  _IOW ('x',  6, struct servicefs_msg_seek_struct)
#define SERVICEFS_MSG_BUSV                  _IOWR('x',  7, struct servicefs_msg_busv_struct)
#define SERVICEFS_MSG_REPLY                 _IOW ('x',  8, struct servicefs_msg_reply_struct)
// Removed ioctl 9.
#define SERVICEFS_MSG_REPLY_FD              _IOW ('x', 10, struct servicefs_msg_reply_struct)
#define SERVICEFS_MOD_CHANNEL_EVENTS        _IOW ('x', 11, struct servicefs_mod_channel_events_struct)
#define SERVICEFS_MSG_PUSH_FD               _IOW ('x', 12, struct servicefs_msg_push_fd_struct)
#define SERVICEFS_MSG_GET_FD                _IOW ('x', 13, struct servicefs_msg_get_fd_struct)
#define SERVICEFS_PUSH_CHANNEL              _IOWR('x', 14, struct servicefs_push_channel_struct)
#define SERVICEFS_CLOSE_CHANNEL             _IOW ('x', 15, int)
#define SERVICEFS_CHECK_CHANNEL             _IOWR('x', 16, struct servicefs_check_channel_struct)

#define SERVICEFS_IOCTL_MAX_NR              (16)

#define SERVICEFS_OP_UNIX_OPEN              (-1)
#define SERVICEFS_OP_UNIX_CLOSE             (-2)
#define SERVICEFS_OP_UNIX_READ              (-3)
#define SERVICEFS_OP_UNIX_WRITE             (-4)
#define SERVICEFS_OP_UNIX_SEEK              (-5)
#endif


#ifndef _SERVICEFS_COMPAT_IOCTL_H_
#define _SERVICEFS_COMPAT_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/compat.h>

struct servicefs_compat_set_channel_context_struct {
	compat_int_t  cid;
	compat_uptr_t ctx;
};

struct servicefs_compat_msg_sendv_struct {
	compat_int_t  op;
	compat_uptr_t svec;
	compat_size_t scnt;
	compat_uptr_t rvec;
	compat_size_t rcnt;
	compat_uptr_t fds;
	compat_size_t fdcnt;
};

struct servicefs_compat_msg_info_struct {
	compat_int_t  pid;
	compat_int_t  tid;
	compat_int_t  cid;
	compat_int_t  mid;
	compat_int_t  euid;
	compat_int_t  egid;
	compat_uptr_t service_private;
	compat_uptr_t channel_private;
	compat_int_t  op;
	compat_int_t  flags;
	compat_size_t send_len;
	compat_size_t recv_len;
	compat_size_t fd_count;
	u64           impulse[4];
};

struct servicefs_compat_msg_rwvec_struct {
	compat_int_t  msgid;
	compat_uptr_t vec;
	compat_size_t len;
};

struct servicefs_compat_msg_seek_struct {
	compat_int_t  msgid;
	compat_long_t offset;
	compat_int_t  whence;
};

struct servicefs_compat_msg_busv_struct {
	compat_int_t  dst_msgid;
	compat_long_t dst_offset;
	compat_int_t  src_msgid;
	compat_long_t src_offset;
	compat_size_t len;
};

struct servicefs_compat_msg_reply_struct {
	compat_int_t msgid;
	compat_int_t retcode;
};

struct servicefs_compat_msg_send_impulse_struct {
	compat_int_t  op;
	compat_uptr_t buf;
	compat_size_t len;
};

struct servicefs_compat_mod_channel_events_struct {
	compat_int_t cid;
	compat_int_t clr;
	compat_int_t set;
};

struct servicefs_compat_msg_push_fd_struct {
	compat_int_t  msgid;
	compat_uint_t pushfd;
};

struct servicefs_compat_msg_get_fd_struct {
	compat_int_t  msgid;
	compat_uint_t index;
};

struct servicefs_compat_push_channel_struct {
	compat_int_t  svcfd;
	compat_int_t  msgid;
	compat_int_t  flags;
	compat_uptr_t cid;
	compat_uptr_t ctx;
};

struct servicefs_compat_check_channel_struct {
	compat_int_t  svcfd;
	compat_int_t  msgid;
	compat_int_t  index;
	compat_uptr_t cid;
	compat_uptr_t ctx;
};

#define SERVICEFS_COMPAT_SET_SERVICE_CONTEXT _IOW ('x',  0, compat_uptr_t)
#define SERVICEFS_COMPAT_SET_CHANNEL_CONTEXT _IOW ('x',  1, struct servicefs_compat_set_channel_context_struct)
#define SERVICEFS_COMPAT_MSG_SENDV           _IOWR('x',  2, struct servicefs_compat_msg_sendv_struct)
#define SERVICEFS_COMPAT_MSG_RECV            _IOR ('x',  3, struct servicefs_compat_msg_info_struct)
#define SERVICEFS_COMPAT_MSG_READV           _IOR ('x',  4, struct servicefs_compat_msg_rwvec_struct)
#define SERVICEFS_COMPAT_MSG_WRITEV          _IOW ('x',  5, struct servicefs_compat_msg_rwvec_struct)
#define SERVICEFS_COMPAT_MSG_SEEK            _IOW ('x',  6, struct servicefs_compat_msg_seek_struct)
#define SERVICEFS_COMPAT_MSG_BUSV            _IOWR('x',  7, struct servicefs_compat_msg_busv_struct)
#define SERVICEFS_COMPAT_MSG_REPLY           _IOW ('x',  8, struct servicefs_compat_msg_reply_struct)
#define SERVICEFS_COMPAT_MSG_SEND_IMPULSE    _IOW ('x',  9, struct servicefs_compat_msg_send_impulse_struct)
#define SERVICEFS_COMPAT_MSG_REPLY_FD        _IOW ('x', 10, struct servicefs_compat_msg_reply_struct)
#define SERVICEFS_COMPAT_MOD_CHANNEL_EVENTS  _IOW ('x', 11, struct servicefs_compat_mod_channel_events_struct)
#define SERVICEFS_COMPAT_MSG_PUSH_FD         _IOW ('x', 12, struct servicefs_compat_msg_push_fd_struct)
#define SERVICEFS_COMPAT_MSG_GET_FD          _IOW ('x', 13, struct servicefs_compat_msg_get_fd_struct)
#define SERVICEFS_COMPAT_PUSH_CHANNEL        _IOWR('x', 14, struct servicefs_compat_push_channel_struct)
#define SERVICEFS_COMPAT_CLOSE_CHANNEL       _IOW ('x', 15, compat_int_t)
#define SERVICEFS_COMPAT_CHECK_CHANNEL       _IOWR('x', 16, struct servicefs_compat_check_channel_struct)
#define SERVICEFS_COMPAT_CANCEL_SERVICE      _IO  ('x', 17)

#endif // _SERVICEFS_COMPAT_IOCTL_H_

#ifndef _SERVICEFS_IOCTL_H
#define _SERVICEFS_IOCTL_H

#include <linux/types.h>

struct servicefs_set_channel_context_struct {
	__s32         cid; // int
	__aligned_u64 ctx; // void *
};

struct servicefs_msg_sendv_struct {
	__s32         op;    // int
	__aligned_u64 svec;  // const struct iovec *
	__aligned_u64 scnt;  // size_t
	__aligned_u64 rvec;  // const struct iovec *
	__aligned_u64 rcnt;  // size_t
	__aligned_u64 fds;   // int *
	__aligned_u64 fdcnt; // size_t
};

struct servicefs_msg_info_struct {
	__s32         pid;             // int
	__s32         tid;             // int
	__s32         cid;             // int
	__s32         mid;             // int
	__s32         euid;            // int
	__s32         egid;            // int
	__s32         op;              // int32_t
	__u32         flags;           // uint32_t
	__aligned_u64 service_private; // void *
	__aligned_u64 channel_private; // void *
	__aligned_u64 send_len;        // size_t
	__aligned_u64 recv_len;        // size_t
	__aligned_u64 fd_count;        // size_t
	__aligned_u64 impulse[4];      // uint64_t
};

struct servicefs_msg_rwvec_struct {
	__s32         msgid; // int
	__aligned_u64 vec;   // const struct iovec *
	__aligned_u64 len;   // size_t
};

struct servicefs_msg_seek_struct {
	__s32         msgid;  // int
	__s32         whence; // int
	__aligned_u64 offset; // size_t
};

struct servicefs_msg_busv_struct {
	__s32         dst_msgid;  // int
	__s32         src_msgid;  // int
	__aligned_u64 dst_offset; // size_t
	__aligned_u64 src_offset; // size_t
	__aligned_u64 len;        // size_t
};

struct servicefs_msg_reply_struct {
	__s32         msgid;   // int
	__s32         retcode; // int
};

struct servicefs_msg_send_impulse_struct {
	__s32         op;  // int
	__aligned_u64 buf; // const void *
	__aligned_u64 len; // size_t
};

struct servicefs_mod_channel_events_struct {
	__s32         cid;     // int
	__u32         clr;     // uint32_t
	__u32         set;     // uint32_t
	__u32         padding; // none
};

struct servicefs_msg_push_fd_struct {
	__s32         msgid;  // int
	__u32         pushfd; // unsigned int
};

struct servicefs_msg_get_fd_struct {
	__s32         msgid; // int
	__u32         index; // unsigned int
};

struct servicefs_push_channel_struct {
	__s32         svcfd; // int
	__s32         msgid; // int
	__s32         flags; // int
	__aligned_u64 cid;   // int *
	__aligned_u64 ctx;   // void *
};

struct servicefs_check_channel_struct {
	__s32         svcfd; // int
	__s32         msgid; // int
	__s32         index; // int
	__aligned_u64 cid;   // int *
	__aligned_u64 ctx;   // void *
};

#define SERVICEFS_SET_SERVICE_CONTEXT       _IOW ('x',  0, __u64)
#define SERVICEFS_SET_CHANNEL_CONTEXT       _IOW ('x',  1, struct servicefs_set_channel_context_struct)
#define SERVICEFS_MSG_SENDV                 _IOWR('x',  2, struct servicefs_msg_sendv_struct)
#define SERVICEFS_MSG_RECV                  _IOR ('x',  3, struct servicefs_msg_info_struct)
#define SERVICEFS_MSG_READV                 _IOR ('x',  4, struct servicefs_msg_rwvec_struct)
#define SERVICEFS_MSG_WRITEV                _IOW ('x',  5, struct servicefs_msg_rwvec_struct)
#define SERVICEFS_MSG_SEEK                  _IOW ('x',  6, struct servicefs_msg_seek_struct)
#define SERVICEFS_MSG_BUSV                  _IOWR('x',  7, struct servicefs_msg_busv_struct)
#define SERVICEFS_MSG_REPLY                 _IOW ('x',  8, struct servicefs_msg_reply_struct)
#define SERVICEFS_MSG_SEND_IMPULSE          _IOW ('x',  9, struct servicefs_msg_send_impulse_struct)
#define SERVICEFS_MSG_REPLY_FD              _IOW ('x', 10, struct servicefs_msg_reply_struct)
#define SERVICEFS_MOD_CHANNEL_EVENTS        _IOW ('x', 11, struct servicefs_mod_channel_events_struct)
#define SERVICEFS_MSG_PUSH_FD               _IOW ('x', 12, struct servicefs_msg_push_fd_struct)
#define SERVICEFS_MSG_GET_FD                _IOW ('x', 13, struct servicefs_msg_get_fd_struct)
#define SERVICEFS_PUSH_CHANNEL              _IOWR('x', 14, struct servicefs_push_channel_struct)
#define SERVICEFS_CLOSE_CHANNEL             _IOW ('x', 15, __s32)
#define SERVICEFS_CHECK_CHANNEL             _IOWR('x', 16, struct servicefs_check_channel_struct)
#define SERVICEFS_CANCEL_SERVICE            _IO  ('x', 17)

#define SERVICEFS_IOCTL_MAX_NR              (17)

#define SERVICEFS_OP_UNIX_OPEN              (-1)
#define SERVICEFS_OP_UNIX_CLOSE             (-2)
#define SERVICEFS_OP_UNIX_READ              (-3)
#define SERVICEFS_OP_UNIX_WRITE             (-4)
#define SERVICEFS_OP_UNIX_SEEK              (-5)

#endif


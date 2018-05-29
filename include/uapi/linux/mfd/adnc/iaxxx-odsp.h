/****************************************************************************
 ****************************************************************************
 ***
 ***   This header was automatically generated from a Linux kernel header
 ***   of the same name, to make information necessary for userspace to
 ***   call into the kernel available to libc.  It contains only constants,
 ***   structures, and macros generated from the original header, and thus,
 ***   contains no copyrightable information.
 ***
 ***   To edit the content of this header, modify the corresponding
 ***   source file (e.g. under external/kernel-headers/original/) then
 ***   run bionic/libc/kernel/tools/update_all.py
 ***
 ***   Any manual change here will be lost the next time this script will
 ***   be run. You've been warned!
 ***
 ****************************************************************************
 ****************************************************************************/
#ifndef __IAXXX_ODSP_H__
#define __IAXXX_ODSP_H__
struct iaxxx_plugin_info {
  uint32_t plg_idx;
  uint32_t pkg_id;
  uint32_t block_id;
  uint32_t inst_id;
  uint32_t priority;
};
struct iaxxx_plugin_param {
  uint32_t inst_id;
  uint32_t param_id;
  uint32_t param_val;
  uint8_t block_id;
};
struct iaxxx_plugin_param_blk {
  uint32_t inst_id;
  uint32_t param_size;
  uint64_t param_blk;
  uint8_t block_id;
  uint32_t id;
};
struct iaxxx_plugin_create_cfg {
  char file_name[256];
  uint32_t inst_id;
  uint32_t cfg_size;
  uint64_t cfg_val;
  uint8_t block_id;
};
struct iaxxx_set_event {
  uint8_t inst_id;
  uint32_t event_enable_mask;
  uint32_t block_id;
};
struct iaxxx_evt_info {
  uint16_t src_id;
  uint16_t event_id;
  uint16_t dst_id;
  uint32_t dst_opaque;
};
struct iaxxx_get_event {
  uint16_t event_id;
  uint32_t data;
};
struct iaxxx_pkg_mgmt_info {
  char pkg_name[256];
  uint32_t pkg_id;
  uint32_t proc_id;
};
#define IAXXX_IOCTL_MAGIC 'I'
#define ODSP_PLG_CREATE _IO(IAXXX_IOCTL_MAGIC, 0x11)
#define ODSP_PLG_RESET _IO(IAXXX_IOCTL_MAGIC, 0x12)
#define ODSP_PLG_ENABLE _IO(IAXXX_IOCTL_MAGIC, 0x13)
#define ODSP_PLG_DISABLE _IO(IAXXX_IOCTL_MAGIC, 0x14)
#define ODSP_PLG_DESTROY _IO(IAXXX_IOCTL_MAGIC, 0x15)
#define ODSP_PLG_SET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x16)
#define ODSP_PLG_GET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x17)
#define ODSP_PLG_SET_PARAM_BLK _IO(IAXXX_IOCTL_MAGIC, 0x18)
#define ODSP_PLG_SET_CREATE_CFG _IO(IAXXX_IOCTL_MAGIC, 0x19)
#define ODSP_PLG_SET_EVENT _IO(IAXXX_IOCTL_MAGIC, 0x1A)
#define ODSP_EVENT_SUBSCRIBE _IO(IAXXX_IOCTL_MAGIC, 0x1B)
#define ODSP_GET_EVENT _IO(IAXXX_IOCTL_MAGIC, 0x1C)
#define ODSP_EVENT_UNSUBSCRIBE _IO(IAXXX_IOCTL_MAGIC, 0x1D)
#define ODSP_LOAD_PACKAGE _IO(IAXXX_IOCTL_MAGIC, 0x1E)
#define ODSP_UNLOAD_PACKAGE _IO(IAXXX_IOCTL_MAGIC, 0x1F)
#define ODSP_UNLOAD_KW_MODEL _IO(IAXXX_IOCTL_MAGIC, 0x20)
#define ODSP_START_RECOGNITION _IO(IAXXX_IOCTL_MAGIC, 0x21)
#define ODSP_STOP_RECOGNITION _IO(IAXXX_IOCTL_MAGIC, 0x22)
#define ODSP_GET_KW_RECOGNIZE_BITMAP _IO(IAXXX_IOCTL_MAGIC, 0x23)
#endif

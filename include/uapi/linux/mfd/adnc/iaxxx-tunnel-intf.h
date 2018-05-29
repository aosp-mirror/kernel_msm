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
#ifndef _IAXXX_TUNNEL_INTF_H
#define _IAXXX_TUNNEL_INTF_H
struct tunlMsg {
  uint32_t tunlEP;
  uint32_t tunlSrc;
  uint32_t tunlMode;
  uint32_t tunlEncode;
} __packed;
struct iaxxx_tunnel_header {
  uint8_t magic[4];
  uint16_t tunnel_id;
  uint8_t crc[6];
  uint64_t ts;
  uint32_t seq_no;
  uint16_t size;
  uint8_t encoding;
  uint8_t sample_rate;
  char buf[0];
} __packed;
struct iaxxx_tnl_evt_info {
  uint16_t src_id;
  uint16_t event_id;
  uint16_t dst_id;
  uint32_t dst_opaque;
  uint32_t evt_threshold;
} __packed;
enum {
  TNL_VQ_CONFIDENCE = 0,
  TNL_CS_OUT1,
  TNL_CS_OUT2,
  TNL_DOA,
  TNL_CVQ,
  TNL_MIC1,
  TNL_MIC2,
  TNL_MIC3,
  TNL_MIC4,
  TNL_AEC_REF1,
  TNL_AEC_REF2,
  TNL_AEC_MIXER,
  TNL_MIC1_Q15,
  TNL_MIC2_Q15,
  TNL_MIC3_Q15,
  TNL_MIC4_Q15,
  TNL_AEC_MIXER_Q15,
  TNL_MBC,
  TNL_PEQ,
  TNL_VP_PARAM,
  TNL_MAX,
};
#define TUNNEL_SETUP _IOWR('K', 0x011, struct tunlMsg)
#define TUNNEL_TERMINATE _IOWR('K', 0x012, struct tunlMsg)
#define TUNNEL_SUBSCRIBE_META _IO('K', 0x013)
#define TUNNEL_SUBSCRIBE_ALL _IO('K', 0x014)
#define TUNNEL_SUBSCRIBE_CVQ _IO('K', 0x015)
#define TUNNEL_UNSUBSCRIBE_META _IO('K', 0x016)
#define TUNNEL_UNSUBSCRIBE_ALL _IO('K', 0x017)
#define TUNNEL_UNSUBSCRIBE_CVQ _IO('K', 0x018)
#define TUNNEL_SUBSCRIBE_META_DOA _IO('K', 0x019)
#define TUNNEL_SUBSCRIBE_META_VQ _IO('K', 0x01a)
#define TUNNEL_UNSUBSCRIBE_META_DOA _IO('K', 0x01b)
#define TUNNEL_UNSUBSCRIBE_META_VQ _IO('K', 0x01c)
#define TUNNEL_EVENT_SUBSCRIBE _IOWR('K', 0x01d, struct iaxxx_tnl_evt_info)
#define TUNNEL_EVENT_UNSUBSCRIBE _IOWR('K', 0x01e, struct iaxxx_tnl_evt_info)
#endif

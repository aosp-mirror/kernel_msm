/*
 * Copyright (c) 2012-2013, 2018 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/*===========================================================================
  \file wlan_nlink_common.h
  
  Exports and types for the Netlink Service interface. This header file contains
  message types and definitions that is shared between the user space service
  (e.g. BTC service) and WLAN kernel module.


===========================================================================*/

#ifndef WLAN_NLINK_COMMON_H__
#define WLAN_NLINK_COMMON_H__

#include <linux/netlink.h>
#include <linux/if.h>
/*---------------------------------------------------------------------------
 * External Functions
 *-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 *-------------------------------------------------------------------------*/
#define WLAN_NL_MAX_PAYLOAD   256     /* maximum size for netlink message*/
#define WLAN_NLINK_PROTO_FAMILY  NETLINK_USERSOCK
#define WLAN_NLINK_MCAST_GRP_ID  0x01 

/*---------------------------------------------------------------------------
 * Type Declarations
 *-------------------------------------------------------------------------*/

/* 
 * The following enum defines the target service within WLAN driver for which the
 * message is intended for. Each service along with its counterpart 
 * in the user space, define a set of messages they recognize.
 * Each of this message will have an header of type tAniMsgHdr defined below.
 * Each Netlink message to/from a kernel module will contain only one
 * message which is preceded by a tAniMsgHdr. The maximun size (in bytes) of
 * a netlink message is assumed to be MAX_PAYLOAD bytes.
 *
 *         +------------+-------+----------+----------+
 *         |Netlink hdr | Align |tAniMsgHdr| msg body |
 *         +------------+-------+----------|----------+
 */

// Message Types 
#define WLAN_BTC_QUERY_STATE_REQ    0x01  // BTC  --> WLAN
#define WLAN_BTC_BT_EVENT_IND       0x02  // BTC  --> WLAN
#define WLAN_BTC_QUERY_STATE_RSP    0x03  // WLAN -->  BTC
#define WLAN_MODULE_UP_IND          0x04  // WLAN -->  BTC
#define WLAN_MODULE_DOWN_IND        0x05  // WLAN -->  BTC
#define WLAN_STA_ASSOC_DONE_IND     0x06  // WLAN -->  BTC
#define WLAN_STA_DISASSOC_DONE_IND  0x07  // WLAN -->  BTC

// Special Message Type used by AMP, intercepted by send_btc_nlink_msg() and
// replaced by WLAN_STA_ASSOC_DONE_IND or WLAN_STA_DISASSOC_DONE_IND
#define WLAN_AMP_ASSOC_DONE_IND     0x10

// Special Message Type used by SoftAP, intercepted by send_btc_nlink_msg() and
// replaced by WLAN_STA_ASSOC_DONE_IND
#define WLAN_BTC_SOFTAP_BSS_START      0x11
#define WLAN_MSG_RPS_ENABLE_IND        0x10A
#define WLAN_SVC_IFACE_NUM_QUEUES      6

#define WLAN_SVC_SAP_RESTART_IND 0x108
#define WLAN_SVC_WLAN_TP_IND     0x109
// Event data for WLAN_BTC_QUERY_STATE_RSP & WLAN_STA_ASSOC_DONE_IND
typedef struct
{
   unsigned char channel;  // 0 implies STA not associated to AP
} tWlanAssocData;

#define ANI_NL_MSG_BASE     0x10    /* Some arbitrary base */

typedef enum eAniNlModuleTypes {
   ANI_NL_MSG_PUMAC = ANI_NL_MSG_BASE + 0x01,// PTT Socket App
   ANI_NL_MSG_PTT   = ANI_NL_MSG_BASE + 0x07,// Quarky GUI
   WLAN_NL_MSG_BTC,
   WLAN_NL_MSG_OEM,
   WLAN_NL_MSG_SVC  = ANI_NL_MSG_BASE + 0x0A,
   ANI_NL_MSG_LOG   = ANI_NL_MSG_BASE + 0x0C,
   ANI_NL_MSG_MAX  
} tAniNlModTypes, tWlanNlModTypes;

#define WLAN_NL_MSG_BASE ANI_NL_MSG_BASE
#define WLAN_NL_MSG_MAX  ANI_NL_MSG_MAX

struct wlan_rps_data {
   char ifname[IFNAMSIZ];
   uint16_t num_queues;
   uint16_t cpu_map[WLAN_SVC_IFACE_NUM_QUEUES];
};

//All Netlink messages must contain this header
typedef struct sAniHdr {
   unsigned short type;
   unsigned short length;
} tAniHdr, tAniMsgHdr;

/**
 * enum wlan_tp_level - indicates wlan throughput level
 * @WLAN_SVC_TP_NONE:    used for initialization
 * @WLAN_SVC_TP_LOW:     used to identify low throughput level
 * @WLAN_SVC_TP_MEDIUM:  used to identify medium throughput level
 * @WLAN_SVC_TP_HIGH:    used to identify high throughput level
 *
 * The different throughput levels are determined on the basis of # of tx and
 * rx packets and other threshold values. For example, if the # of total
 * packets sent or received by the driver is greater than 500 in the last 100ms
 * , the driver has a high throughput requirement. The driver may tweak certain
 * system parameters based on the throughput level.
 */
enum wlan_tp_level {
   WLAN_SVC_TP_NONE,
   WLAN_SVC_TP_LOW,
   WLAN_SVC_TP_MEDIUM,
   WLAN_SVC_TP_HIGH,
};

/* Indication to enable TCP delayed ack in TPUT indication */
#define TCP_DEL_ACK_IND    (1 << 0)

/**
 * struct wlan_rx_tp_data - msg to TCP delayed ack and advance window scaling
 * @level:            Throughput level.
 * @rx_tp_flags:      Bit map of flags, for which this indcation will take
 *                    effect, bit map for TCP_ADV_WIN_SCL and TCP_DEL_ACK_IND.
 */
struct wlan_rx_tp_data {
   enum wlan_tp_level level;
   uint16_t rx_tp_flags;
};

#endif //WLAN_NLINK_COMMON_H__

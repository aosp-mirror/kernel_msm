/*
 * Copyright (c) 2013-2016 The Linux Foundation. All rights reserved.
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


#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <adf_os_types.h>
#include <adf_nbuf.h>
#include <adf_os_io.h>
#include <adf_os_lock.h>
#include <net/ieee80211_radiotap.h>
#include "adf_trace.h"
#include "vos_trace.h"

#ifdef CONFIG_WCNSS_MEM_PRE_ALLOC
#include <net/cnss_prealloc.h>
#endif

/* Packet Counter */
static uint32_t nbuf_tx_mgmt[NBUF_TX_PKT_STATE_MAX];
static uint32_t nbuf_tx_data[NBUF_TX_PKT_STATE_MAX];

/**
 * adf_nbuf_tx_desc_count_display() - Displays the packet counter
 *
 * Return: none
 */
void adf_nbuf_tx_desc_count_display(void)
{
	adf_os_print("Current Snapshot of the Driver:\n");
	adf_os_print("Data Packets:\n");
	adf_os_print("HDD %d TXRX_Q %d TXRX %d HTT %d",
		     nbuf_tx_data[NBUF_TX_PKT_HDD] -
		     (nbuf_tx_data[NBUF_TX_PKT_TXRX] +
		     nbuf_tx_data[NBUF_TX_PKT_TXRX_ENQUEUE] -
		     nbuf_tx_data[NBUF_TX_PKT_TXRX_DEQUEUE]),
		     nbuf_tx_data[NBUF_TX_PKT_TXRX_ENQUEUE] -
		     nbuf_tx_data[NBUF_TX_PKT_TXRX_DEQUEUE],
		     (nbuf_tx_data[NBUF_TX_PKT_TXRX] -
		     nbuf_tx_data[NBUF_TX_PKT_HTT]),
		     (nbuf_tx_data[NBUF_TX_PKT_HTT]  -
		     nbuf_tx_data[NBUF_TX_PKT_HTC]));
	adf_os_print(" HTC %d  HIF %d CE %d TX_COMP %d\n",
		     (nbuf_tx_data[NBUF_TX_PKT_HTC]  -
		     nbuf_tx_data[NBUF_TX_PKT_HIF]),
		     (nbuf_tx_data[NBUF_TX_PKT_HIF]  -
		     nbuf_tx_data[NBUF_TX_PKT_CE]),
		     (nbuf_tx_data[NBUF_TX_PKT_CE]   -
		     nbuf_tx_data[NBUF_TX_PKT_FREE]),
		     nbuf_tx_data[NBUF_TX_PKT_FREE]);
	adf_os_print("Mgmt Packets:\n");
	adf_os_print("TXRX %d HTT %d HTC %d HIF %d CE %d TX_COMP %d\n",
		     (nbuf_tx_mgmt[NBUF_TX_PKT_TXRX] -
		     nbuf_tx_mgmt[NBUF_TX_PKT_HTT]),
		     (nbuf_tx_mgmt[NBUF_TX_PKT_HTT]  -
		     nbuf_tx_mgmt[NBUF_TX_PKT_HTC]),
		     (nbuf_tx_mgmt[NBUF_TX_PKT_HTC]  -
		     nbuf_tx_mgmt[NBUF_TX_PKT_HIF]),
		     (nbuf_tx_mgmt[NBUF_TX_PKT_HIF]  -
		     nbuf_tx_mgmt[NBUF_TX_PKT_CE]),
		     (nbuf_tx_mgmt[NBUF_TX_PKT_CE]   -
		     nbuf_tx_mgmt[NBUF_TX_PKT_FREE]),
		     nbuf_tx_mgmt[NBUF_TX_PKT_FREE]);
}

/**
 * adf_nbuf_tx_desc_count_update() - Updates the layer packet counter
 * @packet_type   : packet type either mgmt/data
 * @current_state : layer at which the packet currently present
 *
 * Return: none
 */
static inline void adf_nbuf_tx_desc_count_update(uint8_t packet_type,
							uint8_t current_state)
{
	switch (packet_type) {
	case NBUF_TX_PKT_MGMT_TRACK:
		nbuf_tx_mgmt[current_state]++;
		break;
	case NBUF_TX_PKT_DATA_TRACK:
		nbuf_tx_data[current_state]++;
		break;
	default:
		break;
	}
}

/**
 * adf_nbuf_tx_desc_count_clear() - Clears packet counter for both data, mgmt
 *
 * Return: none
 */
void adf_nbuf_tx_desc_count_clear(void)
{
	memset(nbuf_tx_mgmt, 0, sizeof(nbuf_tx_mgmt));
	memset(nbuf_tx_data, 0, sizeof(nbuf_tx_data));
}

/**
 * adf_nbuf_set_state() - Updates the packet state
 * @nbuf:            network buffer
 * @current_state :  layer at which the packet currently is
 *
 * This function updates the packet state to the layer at which the packet
 * currently is
 *
 * Return: none
 */
void adf_nbuf_set_state(adf_nbuf_t nbuf, uint8_t current_state)
{
	/*
	 * Only Mgmt, Data Packets are tracked. WMI messages
	 * such as scan commands are not tracked
	 */
	uint8_t packet_type;

	packet_type = NBUF_GET_PACKET_TRACK(nbuf);

	if ((packet_type != NBUF_TX_PKT_DATA_TRACK) &&
	    (packet_type != NBUF_TX_PKT_MGMT_TRACK)) {
		return;
	}
	NBUF_SET_PACKET_STATE(nbuf, current_state);
	adf_nbuf_tx_desc_count_update(packet_type,
				      current_state);
}

adf_nbuf_trace_update_t  trace_update_cb = NULL;

#if defined(CONFIG_WCNSS_MEM_PRE_ALLOC) && defined(FEATURE_SKB_PRE_ALLOC)
struct sk_buff *__adf_nbuf_pre_alloc(adf_os_device_t osdev, size_t size)
{
	struct sk_buff *skb = NULL;

	if (size >= WCNSS_PRE_SKB_ALLOC_GET_THRESHOLD)
		skb = wcnss_skb_prealloc_get(size);

	return skb;
}

int __adf_nbuf_pre_alloc_free(struct sk_buff *skb)
{
	return wcnss_skb_prealloc_put(skb);
}
#else
struct sk_buff *__adf_nbuf_pre_alloc(adf_os_device_t osdev, size_t size)
{
	return NULL;
}

int __adf_nbuf_pre_alloc_free(struct sk_buff *skb)
{
	return 0;
}
#endif

/*
 * @brief This allocates an nbuf aligns if needed and reserves
 *        some space in the front, since the reserve is done
 *        after alignment the reserve value if being unaligned
 *        will result in an unaligned address.
 *
 * @param hdl
 * @param size
 * @param reserve
 * @param align
 *
 * @return nbuf or NULL if no memory
 */
struct sk_buff *
__adf_nbuf_alloc(adf_os_device_t osdev, size_t size, int reserve, int align, int prio)
{
    struct sk_buff *skb;
    unsigned long offset;

    if(align)
        size += (align - 1);

    skb = dev_alloc_skb(size);

    if (skb)
       goto skb_cb;

    skb = __adf_nbuf_pre_alloc(osdev, size);

    if (!skb) {
        printk("ERROR:NBUF alloc failed\n");
        return NULL;
    }

skb_cb:
    memset(skb->cb, 0x0, sizeof(skb->cb));

    /*
     * The default is for netbuf fragments to be interpreted
     * as wordstreams rather than bytestreams.
     * Set the CVG_NBUF_MAX_EXTRA_FRAGS+1 wordstream_flags bits,
     * to provide this default.
     */
    NBUF_EXTRA_FRAG_WORDSTREAM_FLAGS(skb) =
        (1 << (CVG_NBUF_MAX_EXTRA_FRAGS + 1)) - 1;

    /**
     * XXX:how about we reserve first then align
     */

    /**
     * Align & make sure that the tail & data are adjusted properly
     */
    if(align){
        offset = ((unsigned long) skb->data) % align;
        if(offset)
            skb_reserve(skb, align - offset);
    }

    /**
     * NOTE:alloc doesn't take responsibility if reserve unaligns the data
     * pointer
     */
    skb_reserve(skb, reserve);

    return skb;
}

#ifdef QCA_ARP_SPOOFING_WAR
/*
 * __adf_rx_nbuf_alloc() Rx buffer allocation function *
 * @hdl:
 * @size:
 * @reserve:
 * @align:
 *
 * Use existing buffer allocation API and overwrite
 * priv_data field of skb->cb for registering callback
 * as it is not used for Rx case.
 *
 * Return: nbuf or NULL if no memory
 */
struct sk_buff *
__adf_rx_nbuf_alloc(adf_os_device_t osdev, size_t size, int reserve, int align, int prio)
{
    struct sk_buff *skb;

    skb = __adf_nbuf_alloc(osdev, size, reserve,align, prio);
    if (skb) {
        NBUF_CB_PTR(skb) = osdev->filter_cb;
    }
    return skb;
}
#endif
/*
 * @brief free the nbuf its interrupt safe
 * @param skb
 */
void
__adf_nbuf_free(struct sk_buff *skb)
{
#ifdef QCA_MDM_DEVICE
#if defined(IPA_OFFLOAD) && (!defined(IPA_UC_OFFLOAD) ||\
   (defined(IPA_UC_OFFLOAD) && defined(IPA_UC_STA_OFFLOAD)))
    if( (NBUF_OWNER_ID(skb) == IPA_NBUF_OWNER_ID) && NBUF_CALLBACK_FN(skb) )
        NBUF_CALLBACK_FN_EXEC(skb);
    else
#endif
#endif /* QCA_MDM_DEVICE */
    {
       if (__adf_nbuf_pre_alloc_free(skb))
           return;
       dev_kfree_skb_any(skb);
    }
}


/*
 * @brief Reference the nbuf so it can get held until the last free.
 * @param skb
 */

void
__adf_nbuf_ref(struct sk_buff *skb)
{
    skb_get(skb);
}

/**
 *  @brief Check whether the buffer is shared
 *  @param skb: buffer to check
 *
 *  Returns true if more than one person has a reference to this
 *  buffer.
 */
int
__adf_nbuf_shared(struct sk_buff *skb)
{
    return skb_shared(skb);
}
/**
 * @brief create a nbuf map
 * @param osdev
 * @param dmap
 *
 * @return a_status_t
 */
a_status_t
__adf_nbuf_dmamap_create(adf_os_device_t osdev, __adf_os_dma_map_t *dmap)
{
    a_status_t error = A_STATUS_OK;
    /**
     * XXX: driver can tell its SG capablity, it must be handled.
     * XXX: Bounce buffers if they are there
     */
    (*dmap) = kzalloc(sizeof(struct __adf_os_dma_map), GFP_KERNEL);
    if(!(*dmap))
        error = A_STATUS_ENOMEM;

    return error;
}

/**
 * @brief free the nbuf map
 *
 * @param osdev
 * @param dmap
 */
void
__adf_nbuf_dmamap_destroy(adf_os_device_t osdev, __adf_os_dma_map_t dmap)
{
    kfree(dmap);
}

/**
 * @brief get the dma map of the nbuf
 *
 * @param osdev
 * @param bmap
 * @param skb
 * @param dir
 *
 * @return a_status_t
 */
a_status_t
__adf_nbuf_map(
    adf_os_device_t osdev,
    struct sk_buff *skb,
    adf_os_dma_dir_t dir)
{
#ifdef ADF_OS_DEBUG
    struct skb_shared_info  *sh = skb_shinfo(skb);
#endif
    adf_os_assert(
        (dir == ADF_OS_DMA_TO_DEVICE) || (dir == ADF_OS_DMA_FROM_DEVICE));

    /*
     * Assume there's only a single fragment.
     * To support multiple fragments, it would be necessary to change
     * adf_nbuf_t to be a separate object that stores meta-info
     * (including the bus address for each fragment) and a pointer
     * to the underlying sk_buff.
     */
    adf_os_assert(sh->nr_frags == 0);

    return __adf_nbuf_map_single(osdev, skb, dir);

    return A_STATUS_OK;
}

/**
 * @brief adf_nbuf_unmap() - to unmap a previously mapped buf
 */
void
__adf_nbuf_unmap(
    adf_os_device_t osdev,
    struct sk_buff *skb,
    adf_os_dma_dir_t dir)
{
    adf_os_assert(
        (dir == ADF_OS_DMA_TO_DEVICE) || (dir == ADF_OS_DMA_FROM_DEVICE));

    adf_os_assert(((dir == ADF_OS_DMA_TO_DEVICE) || (dir == ADF_OS_DMA_FROM_DEVICE)));
    /*
     * Assume there's a single fragment.
     * If this is not true, the assertion in __adf_nbuf_map will catch it.
     */
    __adf_nbuf_unmap_single(osdev, skb, dir);
}

a_status_t
__adf_nbuf_map_single(
    adf_os_device_t osdev, adf_nbuf_t buf, adf_os_dma_dir_t dir)
{
    u_int32_t paddr_lo;

/* tempory hack for simulation */
#ifdef A_SIMOS_DEVHOST
    NBUF_MAPPED_PADDR_LO(buf) = paddr_lo = (u_int32_t) buf->data;
    return A_STATUS_OK;
#else
    /* assume that the OS only provides a single fragment */
    NBUF_MAPPED_PADDR_LO(buf) = paddr_lo =
        dma_map_single(osdev->dev, buf->data,
                       skb_end_pointer(buf) - buf->data, dir);
    return dma_mapping_error(osdev->dev, paddr_lo) ?
        A_STATUS_FAILED : A_STATUS_OK;
#endif	/* #ifdef A_SIMOS_DEVHOST */
}

void
__adf_nbuf_unmap_single(
    adf_os_device_t osdev, adf_nbuf_t buf, adf_os_dma_dir_t dir)
{
#if !defined(A_SIMOS_DEVHOST)
    dma_unmap_single(osdev->dev, NBUF_MAPPED_PADDR_LO(buf),
                     skb_end_pointer(buf) - buf->data, dir);
#endif	/* #if !defined(A_SIMOS_DEVHOST) */
}

/**
 * @brief return the dma map info
 *
 * @param[in]  bmap
 * @param[out] sg (map_info ptr)
 */
void
__adf_nbuf_dmamap_info(__adf_os_dma_map_t bmap, adf_os_dmamap_info_t *sg)
{
    adf_os_assert(bmap->mapped);
    adf_os_assert(bmap->nsegs <= ADF_OS_MAX_SCATTER);

    memcpy(sg->dma_segs, bmap->seg, bmap->nsegs *
           sizeof(struct __adf_os_segment));
    sg->nsegs = bmap->nsegs;
}
/**
 * @brief return the frag data & len, where frag no. is
 *        specified by the index
 *
 * @param[in] buf
 * @param[out] sg (scatter/gather list of all the frags)
 *
 */
void
__adf_nbuf_frag_info(struct sk_buff *skb, adf_os_sglist_t  *sg)
{
#if defined(ADF_OS_DEBUG) || defined(__ADF_SUPPORT_FRAG_MEM)
    struct skb_shared_info  *sh = skb_shinfo(skb);
#endif
    adf_os_assert(skb != NULL);
    sg->sg_segs[0].vaddr = skb->data;
    sg->sg_segs[0].len   = skb->len;
    sg->nsegs            = 1;

#ifndef __ADF_SUPPORT_FRAG_MEM
    adf_os_assert(sh->nr_frags == 0);
#else
    for(int i = 1; i <= sh->nr_frags; i++){
        skb_frag_t    *f        = &sh->frags[i - 1];
        sg->sg_segs[i].vaddr    = (uint8_t *)(page_address(f->page) +
                                  f->page_offset);
        sg->sg_segs[i].len      = f->size;

        adf_os_assert(i < ADF_OS_MAX_SGLIST);
    }
    sg->nsegs += i;
#endif
}

a_status_t
__adf_nbuf_set_rx_cksum(struct sk_buff *skb, adf_nbuf_rx_cksum_t *cksum)
{
    switch (cksum->l4_result) {
    case ADF_NBUF_RX_CKSUM_NONE:
        skb->ip_summed = CHECKSUM_NONE;
        break;
    case ADF_NBUF_RX_CKSUM_TCP_UDP_UNNECESSARY:
        skb->ip_summed = CHECKSUM_UNNECESSARY;
        break;
    case ADF_NBUF_RX_CKSUM_TCP_UDP_HW:
        skb->ip_summed = CHECKSUM_PARTIAL;
        skb->csum      = cksum->val;
        break;
    default:
        printk("ADF_NET:Unknown checksum type\n");
        adf_os_assert(0);
	return A_STATUS_ENOTSUPP;
    }
    return A_STATUS_OK;
}

adf_nbuf_tx_cksum_t
__adf_nbuf_get_tx_cksum(struct sk_buff *skb)
{
    switch (skb->ip_summed) {
    case CHECKSUM_NONE:
        return ADF_NBUF_TX_CKSUM_NONE;
    case CHECKSUM_PARTIAL:
        /* XXX ADF and Linux checksum don't map with 1-to-1. This is not 100%
         * correct. */
        return ADF_NBUF_TX_CKSUM_TCP_UDP;
    case CHECKSUM_COMPLETE:
        return ADF_NBUF_TX_CKSUM_TCP_UDP_IP;
    default:
        return ADF_NBUF_TX_CKSUM_NONE;
    }
}

a_status_t
__adf_nbuf_get_vlan_info(adf_net_handle_t hdl, struct sk_buff *skb,
                         adf_net_vlanhdr_t *vlan)
{
     return A_STATUS_OK;
}

a_uint8_t
__adf_nbuf_get_tid(struct sk_buff *skb)
{
    return skb->priority;
}

void
__adf_nbuf_set_tid(struct sk_buff *skb, a_uint8_t tid)
{
        skb->priority = tid;
}

a_uint8_t
__adf_nbuf_get_exemption_type(struct sk_buff *skb)
{
    return ADF_NBUF_EXEMPT_NO_EXEMPTION;
}

void
__adf_nbuf_dmamap_set_cb(__adf_os_dma_map_t dmap, void *cb, void *arg)
{
    return;
}

void
__adf_nbuf_reg_trace_cb(adf_nbuf_trace_update_t cb_func_ptr)
{
   trace_update_cb = cb_func_ptr;
   return;
}

/**
 * __adf_nbuf_data_get_dhcp_subtype() - get the subtype
 *              of DHCP packet.
 * @data: Pointer to DHCP packet data buffer
 *
 * This func. returns the subtype of DHCP packet.
 *
 * Return: subtype of the DHCP packet.
 */
enum adf_proto_subtype
__adf_nbuf_data_get_dhcp_subtype(uint8_t *data)
{
	enum adf_proto_subtype subtype = ADF_PROTO_INVALID;

	if ((data[DHCP_OPTION53_OFFSET] == DHCP_OPTION53) &&
		(data[DHCP_OPTION53_LENGTH_OFFSET] ==
					DHCP_OPTION53_LENGTH)) {

		switch (data[DHCP_OPTION53_STATUS_OFFSET]) {
		case DHCPDISCOVER:
			subtype = ADF_PROTO_DHCP_DISCOVER;
			break;
		case DHCPREQUEST:
			subtype = ADF_PROTO_DHCP_REQUEST;
			break;
		case DHCPOFFER:
			subtype = ADF_PROTO_DHCP_OFFER;
			break;
		case DHCPACK:
			subtype = ADF_PROTO_DHCP_ACK;
			break;
		case DHCPNAK:
			subtype = ADF_PROTO_DHCP_NACK;
			break;
		case DHCPRELEASE:
			subtype = ADF_PROTO_DHCP_RELEASE;
			break;
		case DHCPINFORM:
			subtype = ADF_PROTO_DHCP_INFORM;
			break;
		case DHCPDECLINE:
			subtype = ADF_PROTO_DHCP_DECLINE;
			break;
		default:
			break;
		}
	}

	return subtype;
}

/**
 * __adf_nbuf_data_get_eapol_subtype() - get the subtype
 *            of EAPOL packet.
 * @data: Pointer to EAPOL packet data buffer
 *
 * This func. returns the subtype of EAPOL packet.
 *
 * Return: subtype of the EAPOL packet.
 */
enum adf_proto_subtype
__adf_nbuf_data_get_eapol_subtype(uint8_t *data)
{
	uint16_t eapol_key_info;
	enum adf_proto_subtype subtype = ADF_PROTO_INVALID;
	uint16_t mask;

	eapol_key_info = (uint16_t)(*(uint16_t *)
			(data + EAPOL_KEY_INFO_OFFSET));

	mask = eapol_key_info & EAPOL_MASK;
	switch (mask) {
	case EAPOL_M1_BIT_MASK:
		subtype = ADF_PROTO_EAPOL_M1;
		break;
	case EAPOL_M2_BIT_MASK:
		subtype = ADF_PROTO_EAPOL_M2;
		break;
	case EAPOL_M3_BIT_MASK:
		subtype = ADF_PROTO_EAPOL_M3;
		break;
	case EAPOL_M4_BIT_MASK:
		subtype = ADF_PROTO_EAPOL_M4;
		break;
	default:
		break;
	}

	return subtype;
}

/**
 * __adf_nbuf_data_get_arp_subtype() - get the subtype
 *            of ARP packet.
 * @data: Pointer to ARP packet data buffer
 *
 * This func. returns the subtype of ARP packet.
 *
 * Return: subtype of the ARP packet.
 */
enum adf_proto_subtype
__adf_nbuf_data_get_arp_subtype(uint8_t *data)
{
	uint16_t subtype;
	enum adf_proto_subtype proto_subtype = ADF_PROTO_INVALID;

	subtype = (uint16_t)(*(uint16_t *)
			(data + ARP_SUB_TYPE_OFFSET));

	switch (adf_os_cpu_to_be16(subtype)) {
	case ARP_REQUEST:
		proto_subtype = ADF_PROTO_ARP_REQ;
		break;
	case ARP_RESPONSE:
		proto_subtype = ADF_PROTO_ARP_RES;
		break;
	default:
		break;
	}

	return proto_subtype;
}

/**
 * __adf_nbuf_data_get_icmp_subtype() - get the subtype
 *            of IPV4 ICMP packet.
 * @data: Pointer to IPV4 ICMP packet data buffer
 *
 * This func. returns the subtype of ICMP packet.
 *
 * Return: subtype of the ICMP packet.
 */
enum adf_proto_subtype
__adf_nbuf_data_get_icmp_subtype(uint8_t *data)
{
	uint8_t subtype;
	enum adf_proto_subtype proto_subtype = ADF_PROTO_INVALID;

	subtype = (uint8_t)(*(uint8_t *)
			(data + ICMP_SUBTYPE_OFFSET));

	switch (subtype) {
	case ICMP_REQUEST:
		proto_subtype = ADF_PROTO_ICMP_REQ;
		break;
	case ICMP_RESPONSE:
		proto_subtype = ADF_PROTO_ICMP_RES;
		break;
	default:
		break;
	}

	return proto_subtype;
}

/**
 * __adf_nbuf_data_get_icmpv6_subtype() - get the subtype
 *            of IPV6 ICMPV6 packet.
 * @data: Pointer to IPV6 ICMPV6 packet data buffer
 *
 * This func. returns the subtype of ICMPV6 packet.
 *
 * Return: subtype of the ICMPV6 packet.
 */
enum adf_proto_subtype
__adf_nbuf_data_get_icmpv6_subtype(uint8_t *data)
{
	uint8_t subtype;
	enum adf_proto_subtype proto_subtype = ADF_PROTO_INVALID;

	subtype = (uint8_t)(*(uint8_t *)
			(data + ICMPV6_SUBTYPE_OFFSET));

	switch (subtype) {
	case ICMPV6_REQUEST:
		proto_subtype = ADF_PROTO_ICMPV6_REQ;
		break;
	case ICMPV6_RESPONSE:
		proto_subtype = ADF_PROTO_ICMPV6_RES;
		break;
	case ICMPV6_NS:
		proto_subtype = ADF_PROTO_ICMPV6_NS;
		break;
	case ICMPV6_NA:
		proto_subtype = ADF_PROTO_ICMPV6_NA;
		break;
	default:
		break;
	}

	return proto_subtype;
}

/**
 * __adf_nbuf_data_get_ipv4_proto() - get the proto type
 *            of IPV4 packet.
 * @data: Pointer to IPV4 packet data buffer
 *
 * This func. returns the proto type of IPV4 packet.
 *
 * Return: proto type of IPV4 packet.
 */
uint8_t
__adf_nbuf_data_get_ipv4_proto(uint8_t *data)
{
	uint8_t proto_type;

	proto_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV4_PROTO_TYPE_OFFSET));
	return proto_type;
}

/**
 * __adf_nbuf_data_get_ipv6_proto() - get the proto type
 *            of IPV6 packet.
 * @data: Pointer to IPV6 packet data buffer
 *
 * This func. returns the proto type of IPV6 packet.
 *
 * Return: proto type of IPV6 packet.
 */
uint8_t
__adf_nbuf_data_get_ipv6_proto(uint8_t *data)
{
	uint8_t proto_type;

	proto_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV6_PROTO_TYPE_OFFSET));
	return proto_type;
}

/**
 * __adf_nbuf_data_is_dhcp_pkt() - check if it is DHCP packet.
 * @data: Pointer to DHCP packet data buffer
 *
 * This func. checks whether it is a DHCP packet or not.
 *
 * Return: A_STATUS_OK if it is a DHCP packet
 *         A_STATUS_FAILED if not
 */
a_status_t
__adf_nbuf_data_is_dhcp_pkt(uint8_t *data)
{
   a_uint16_t    SPort;
   a_uint16_t    DPort;

    SPort = (a_uint16_t)(*(a_uint16_t *)(data + ADF_NBUF_TRAC_IPV4_OFFSET +
                                     ADF_NBUF_TRAC_IPV4_HEADER_SIZE));
    DPort = (a_uint16_t)(*(a_uint16_t *)(data + ADF_NBUF_TRAC_IPV4_OFFSET +
                                     ADF_NBUF_TRAC_IPV4_HEADER_SIZE + sizeof(a_uint16_t)));

    if (((ADF_NBUF_TRAC_DHCP_SRV_PORT == adf_os_cpu_to_be16(SPort)) &&
       (ADF_NBUF_TRAC_DHCP_CLI_PORT == adf_os_cpu_to_be16(DPort))) ||
       ((ADF_NBUF_TRAC_DHCP_CLI_PORT == adf_os_cpu_to_be16(SPort)) &&
       (ADF_NBUF_TRAC_DHCP_SRV_PORT == adf_os_cpu_to_be16(DPort))))
    {
        return A_STATUS_OK;
    }
    else
    {
        return A_STATUS_FAILED;
    }
}

/**
 * __adf_nbuf_data_is_eapol_pkt() - check if it is EAPOL packet.
 * @data: Pointer to EAPOL packet data buffer
 *
 * This func. checks whether it is a EAPOL packet or not.
 *
 * Return: A_STATUS_OK if it is a EAPOL packet
 *         A_STATUS_FAILED if not
 */
a_status_t
__adf_nbuf_data_is_eapol_pkt(uint8_t *data)
{
    a_uint16_t    ether_type;

    ether_type = (a_uint16_t)(*(a_uint16_t *)(data +
			ADF_NBUF_TRAC_ETH_TYPE_OFFSET));
    if (ADF_NBUF_TRAC_EAPOL_ETH_TYPE == adf_os_cpu_to_be16(ether_type))
    {
        return A_STATUS_OK;
    }
    else
    {
        return A_STATUS_FAILED;
    }
}

/**
 * __adf_nbuf_data_is_ipv4_arp_pkt() - check if it is ARP packet.
 * @data: Pointer to ARP packet data buffer
 *
 * This func. checks whether it is a ARP packet or not.
 *
 * Return: TRUE if it is a ARP packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv4_arp_pkt(uint8_t *data)
{
	uint16_t ether_type;

	ether_type = (uint16_t)(*(uint16_t *)(data +
				ADF_NBUF_TRAC_ETH_TYPE_OFFSET));

	if (ether_type == adf_os_cpu_to_be16(ADF_NBUF_TRAC_ARP_ETH_TYPE))
		return true;
	else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv4_pkt() - check if it is IPV4 packet.
 * @data: Pointer to IPV4 packet data buffer
 *
 * This func. checks whether it is a IPV4 packet or not.
 *
 * Return: TRUE if it is a IPV4 packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv4_pkt(uint8_t *data)
{
	uint16_t ether_type;

	ether_type = (uint16_t)(*(uint16_t *)(data +
				ADF_NBUF_TRAC_ETH_TYPE_OFFSET));

	if (ether_type == adf_os_cpu_to_be16(ADF_NBUF_TRAC_IPV4_ETH_TYPE))
		return true;
	else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv4_mcast_pkt() - check if it is IPV4 multicast packet.
 * @data: Pointer to IPV4 packet data buffer
 *
 * This func. checks whether it is a IPV4 muticast packet or not.
 *
 * Return: TRUE if it is a IPV4 multicast packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv4_mcast_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv4_pkt(data)) {
		uint8_t *dst_addr =
			(uint8_t *)(data + ADF_NBUF_TRAC_IPV4_DEST_ADDR_OFFSET);

		/*
		 * Check first byte of the IP address and if it
		 * from 224 to 239, then it can represent multicast IP.
		 */
		if (dst_addr[0] >= 224 && dst_addr[0]  <= 239)
			return true;
		else
			return false;
	} else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv6_mcast_pkt() - check if it is IPv6 multicast packet.
 * @data: Pointer to IPv6 packet data buffer
 *
 * This func. checks whether it is a IPv6 multicast packet or not.
 *
 * Return: TRUE if it is a IPV6 multicast packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv6_mcast_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv6_pkt(data)) {
		uint16_t *dst_addr;

		dst_addr = (uint16_t *)(data + ADF_NBUF_TRAC_IPV6_DEST_ADDR_OFFSET);

		/*
		 * Check first byte of the IP address and if it
		 * 0xFF00 then it is a IPV6 mcast packet.
		 */
		if (*dst_addr == adf_os_cpu_to_be16(ADF_NBUF_TRAC_IPV6_DEST_ADDR))
			return true;
		else
			return false;
	} else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv6_pkt() - check if it is IPV6 packet.
 * @data: Pointer to IPV6 packet data buffer
 *
 * This func. checks whether it is a IPV6 packet or not.
 *
 * Return: TRUE if it is a IPV6 packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv6_pkt(uint8_t *data)
{
	uint16_t ether_type;

	ether_type = (uint16_t)(*(uint16_t *)(data +
				ADF_NBUF_TRAC_ETH_TYPE_OFFSET));

	if (ether_type == adf_os_cpu_to_be16(ADF_NBUF_TRAC_IPV6_ETH_TYPE))
		return true;
	else
		return false;
}

/**
 * __adf_nbuf_data_is_icmp_pkt() - check if it is IPV4 ICMP packet.
 * @data: Pointer to IPV4 ICMP packet data buffer
 *
 * This func. checks whether it is a ICMP packet or not.
 *
 * Return: TRUE if it is a ICMP packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_icmp_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv4_pkt(data)) {
		uint8_t pkt_type;

		pkt_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV4_PROTO_TYPE_OFFSET));

		if (pkt_type == ADF_NBUF_TRAC_ICMP_TYPE)
			return true;
		else
			return false;
	} else
		return false;
}

/**
 * __adf_nbuf_data_is_icmpv6_pkt() - check if it is IPV6 ICMPV6 packet.
 * @data: Pointer to IPV6 ICMPV6 packet data buffer
 *
 * This func. checks whether it is a ICMPV6 packet or not.
 *
 * Return: TRUE if it is a ICMPV6 packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_icmpv6_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv6_pkt(data)) {
		uint8_t pkt_type;

		pkt_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV6_PROTO_TYPE_OFFSET));

		if (pkt_type == ADF_NBUF_TRAC_ICMPV6_TYPE)
			return true;
		else
			return false;
	} else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv4_udp_pkt() - check if it is IPV4 UDP packet.
 * @data: Pointer to IPV4 UDP packet data buffer
 *
 * This func. checks whether it is a IPV4 UDP packet or not.
 *
 * Return: TRUE if it is a IPV4 UDP packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv4_udp_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv4_pkt(data)) {
		uint8_t pkt_type;

		pkt_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV4_PROTO_TYPE_OFFSET));

		if (pkt_type == ADF_NBUF_TRAC_UDP_TYPE)
			return true;
		else
			return false;
	} else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv4_tcp_pkt() - check if it is IPV4 TCP packet.
 * @data: Pointer to IPV4 TCP packet data buffer
 *
 * This func. checks whether it is a IPV4 TCP packet or not.
 *
 * Return: TRUE if it is a IPV4 TCP packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv4_tcp_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv4_pkt(data)) {
		uint8_t pkt_type;

		pkt_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV4_PROTO_TYPE_OFFSET));

		if (pkt_type == ADF_NBUF_TRAC_TCP_TYPE)
			return true;
		else
			return false;
	} else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv6_udp_pkt() - check if it is IPV6 UDP packet.
 * @data: Pointer to IPV6 UDP packet data buffer
 *
 * This func. checks whether it is a IPV6 UDP packet or not.
 *
 * Return: TRUE if it is a IPV6 UDP packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv6_udp_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv6_pkt(data)) {
		uint8_t pkt_type;

		pkt_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV6_PROTO_TYPE_OFFSET));

		if (pkt_type == ADF_NBUF_TRAC_UDP_TYPE)
			return true;
		else
			return false;
	} else
		return false;
}

/**
 * __adf_nbuf_data_is_ipv6_tcp_pkt() - check if it is IPV6 TCP packet.
 * @data: Pointer to IPV6 TCP packet data buffer
 *
 * This func. checks whether it is a IPV6 TCP packet or not.
 *
 * Return: TRUE if it is a IPV6 TCP packet
 *         FALSE if not
 */
bool __adf_nbuf_data_is_ipv6_tcp_pkt(uint8_t *data)
{
	if (__adf_nbuf_data_is_ipv6_pkt(data)) {
		uint8_t pkt_type;

		pkt_type = (uint8_t)(*(uint8_t *)(data +
				ADF_NBUF_TRAC_IPV6_PROTO_TYPE_OFFSET));

		if (pkt_type == ADF_NBUF_TRAC_TCP_TYPE)
			return true;
		else
			return false;
	} else
		return false;
}

#ifdef QCA_PKT_PROTO_TRACE
void
__adf_nbuf_trace_update(struct sk_buff *buf, char *event_string)
{
   char string_buf[NBUF_PKT_TRAC_MAX_STRING];

   if ((!trace_update_cb) || (!event_string)) {
      return;
   }

   if (!adf_nbuf_trace_get_proto_type(buf)) {
      return;
   }

   /* Buffer over flow */
   if (NBUF_PKT_TRAC_MAX_STRING <=
       (adf_os_str_len(event_string) + NBUF_PKT_TRAC_PROTO_STRING)) {
      return;
   }

   adf_os_mem_zero(string_buf,
                   NBUF_PKT_TRAC_MAX_STRING);
   adf_os_mem_copy(string_buf,
                   event_string, adf_os_str_len(event_string));
   switch (adf_nbuf_trace_get_proto_type(buf)) {
   case NBUF_PKT_TRAC_TYPE_EAPOL:
      adf_os_mem_copy(string_buf + adf_os_str_len(event_string),
                      "EPL", NBUF_PKT_TRAC_PROTO_STRING);
      break;
   case NBUF_PKT_TRAC_TYPE_DHCP:
      adf_os_mem_copy(string_buf + adf_os_str_len(event_string),
                      "DHC", NBUF_PKT_TRAC_PROTO_STRING);
      break;
   case NBUF_PKT_TRAC_TYPE_MGMT_ACTION:
      adf_os_mem_copy(string_buf + adf_os_str_len(event_string),
                      "MACT", NBUF_PKT_TRAC_PROTO_STRING);
      break;
   case NBUF_PKT_TRAC_TYPE_ARP:
      adf_os_mem_copy(string_buf + adf_os_str_len(event_string),
                      "ARP", NBUF_PKT_TRAC_PROTO_STRING);
      break;
   case NBUF_PKT_TRAC_TYPE_NS:
      adf_os_mem_copy(string_buf + adf_os_str_len(event_string),
                      "NS", NBUF_PKT_TRAC_PROTO_STRING);
      break;
   case NBUF_PKT_TRAC_TYPE_NA:
      adf_os_mem_copy(string_buf + adf_os_str_len(event_string),
                      "NA", NBUF_PKT_TRAC_PROTO_STRING);
      break;
   default:
      break;
   }

   trace_update_cb(string_buf);
   return;
}
#endif /* QCA_PKT_PROTO_TRACE */

/**
 * adf_nbuf_update_radiotap() - Update radiotap header from rx_status
 *
 * @rx_status: Pointer to rx_status.
 * @nbuf:      nbuf pointe to which radiotap has to be updated
 * @headroom_sz: Available headroom size.
 *
 * Return: length of rtap_len updated.
 */
int adf_nbuf_update_radiotap(struct mon_rx_status *rx_status, adf_nbuf_t nbuf,
			     u_int32_t headroom_sz)
{
	uint8_t rtap_buf[sizeof(struct ieee80211_radiotap_header) + 100] = {0};
	struct ieee80211_radiotap_header *rthdr =
		(struct ieee80211_radiotap_header *)rtap_buf;
	uint32_t rtap_hdr_len = sizeof(struct ieee80211_radiotap_header);
	uint32_t rtap_len = rtap_hdr_len;

	/* IEEE80211_RADIOTAP_TSFT              __le64       microseconds*/
	rthdr->it_present = cpu_to_le32(1 << IEEE80211_RADIOTAP_TSFT);
	put_unaligned_le64(rx_status->tsft,
			   (void *)&rtap_buf[rtap_len]);
	rtap_len += 8;

	/* IEEE80211_RADIOTAP_FLAGS u8*/
	rthdr->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_FLAGS);
	rtap_buf[rtap_len] = rx_status->flags;
	rtap_len += 1;

	/* IEEE80211_RADIOTAP_RATE  u8           500kb/s*/
	rthdr->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_RATE);
	rtap_buf[rtap_len] = rx_status->rate;
	rtap_len += 1;
	rthdr->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_CHANNEL);
	/* IEEE80211_RADIOTAP_CHANNEL, Channel frequency in Mhz */
	put_unaligned_le16(rx_status->chan, (void *)&rtap_buf[rtap_len]);
	rtap_len += 2;
	/* Channel flags. */

	put_unaligned_le16(rx_status->chan_flags, (void *)&rtap_buf[rtap_len]);
	rtap_len += 2;

	/* IEEE80211_RADIOTAP_DBM_ANTSIGNAL */
	rthdr->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL);
#define NORMALIZED_TO_NOISE_FLOOR (-96)
	/*
	 * rssi_comb is int dB, need to convert it to dBm.
	 * normalize value to noise floor of -96 dBm
	 */
	rtap_buf[rtap_len] = rx_status->ant_signal_db +
		NORMALIZED_TO_NOISE_FLOOR;
	rtap_len += 1;
	rthdr->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_ANTENNA);
	rtap_buf[rtap_len] = rx_status->nr_ant;
	rtap_len += 1;

	rthdr->it_len = cpu_to_le16(rtap_len);

	adf_nbuf_pull_head(nbuf, headroom_sz  - rtap_len);
	adf_os_mem_copy(adf_nbuf_data(nbuf), rthdr, rtap_hdr_len);
	adf_os_mem_copy(adf_nbuf_data(nbuf) + rtap_hdr_len, rtap_buf +
			rtap_hdr_len, rtap_len - rtap_hdr_len);
	return rtap_len;
}

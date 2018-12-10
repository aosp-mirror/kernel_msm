/*
 * iaxxx-tunnel-dev.c -- iaxxx Tunneling Service device node
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#define pr_fmt(fmt) "iaxxx : %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mfd/adnc/iaxxx-tunnel-intf.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/circ_buf.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/mfd/adnc/iaxxx-core.h>

#include "iaxxx.h"
#include "iaxxx-cdev.h"
#include "iaxxx-dbgfs.h"
#include "iaxxx-tunnel-priv.h"

#define PBUFF_SIZE			(1024*1024)	/* Producer Buffer */
#define UBUFF_SIZE			(256*1024)	/* Client Fifo */

#define TUNNEL_MAGIC1 0x45
#define TUNNEL_MAGIC2 0x4D
#define TUNNEL_MAGIC3 0x4F
#define TUNNEL_MAGIC4 0x52

#define TNL_SYNC_MODE 0
#define TNL_ASYNC_MODE 1
#define MAX_PACKET_SIZE 1920

#ifdef CONFIG_MFD_IAXXX_TUNNEL_POLL
#define IAXXX_TUNNEL_THRESHOLD 0
#else
#define IAXXX_TUNNEL_THRESHOLD 0x1400
#endif

#define CHECK_SEQUENCE

#define PRODUCER_PRIO 15	/* Less than mdss_dsi_event */
#define PRODUCER_WAIT_TIME_US 1000 /* 1ms interval */
#define PRODUCER_MAX_WAIT_TIME_US 100000 /* 100ms interval */
#define TNL_SRC_Q 0x00010000	/* For defining new tunnel id with Q format */

#define IAXXX_TUNNEL_TERMINATE_TIMEOUT	1000

enum {
	IAXXX_TFLG_FW_CRASH,
	IAXXX_TFLG_RUN_THREAD,
	IAXXX_TFLG_ADJ_CLIENT,
	IAXXX_TFLG_ADJ_THREAD,
	IAXXX_TFLG_THRESHOLD,
	IAXXX_TFLG_DRAIN_BIT,
	IAXXX_TFLG_ALLOW_TUNNELS,
	IAXXX_TFLG_DISABLE_ALL
};

#define IAXXX_DEBUG_LAUNCH_DELAY 60000 /* 60 seconds */

struct iaxxx_tunnel_ep {
	struct tunlMsg tnl_ep;
	struct list_head src_head_list;
};

static phys_addr_t iaxxx_prod_buf;
static size_t iaxxx_prod_buf_size;

static unsigned long tunnel_flags;

#define is_valid_tid(__id__) ((__id__) < TNLMAX)
#define is_valid_size(__size__) ((__size__) > 0 && \
			(__size__) <= MAX_PACKET_SIZE && \
			!((__size__) & 3))

#define circ_cnt(__c__) \
	(CIRC_CNT((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_space(__c__) \
	(CIRC_SPACE((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_cnt_to_end(__c__) \
	(CIRC_CNT_TO_END((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_space_to_end(__c__) \
	(CIRC_SPACE_TO_END((__c__)->head, (__c__)->tail, (__c__)->size))

/*
 * Init head / tail
 */
static inline void circ_init(struct iaxxx_circ_buf *circ)
{
	circ->head = circ->tail = 0;
}

/*
 * Update tail
 */
static inline void circ_peek_finish(struct iaxxx_circ_buf *circ, int size)
{
	smp_wmb(); /* Make sure data is copied before updating indexes */
	circ->tail = (circ->tail + (size >> 2)) & (circ->size - 1);
}

/*
 * Update head pointer
 */
static inline void circ_fill_finish(struct iaxxx_circ_buf *circ, int size)
{
	smp_wmb(); /* Make sure data is copied before updating indexes */
	circ->head = (circ->head + (size >> 2)) & (circ->size - 1);
}

/*
 * Return maximum contiguous free buffer
 */
static inline int circ_get_free_buffer(struct iaxxx_circ_buf *circ, void **buf)
{
	int size = circ_space_to_end(circ) << 2;

	if (size > 0)
		*buf = &circ->buf[circ->head << 2];

	return size;
}

/*
 * Fill buf with the data in circular queue but no tail moving
 */
static inline int circ_peek_data(struct iaxxx_circ_buf *circ,
						void *buf, int size)
{
	u8 *p = buf;

	int len = min(size, circ_cnt(circ) << 2);
	int first_len =
		min(len, circ_cnt_to_end(circ) << 2);

	smp_rmb(); /* Make sure fetching indexes before data copy */
	memcpy(p, circ->buf + (circ->tail << 2), first_len);
	if (first_len < len)
		memcpy(p + first_len, circ->buf, len - first_len);

	return len;
}

/*
 * Push data to circular queue
 */
static inline void circ_in(struct iaxxx_circ_buf *circ,
					const void *buf, int size)
{
	void *user_buf = NULL;
	int len;

	len = circ_get_free_buffer(circ, &user_buf);
	if (len < size) {
		memcpy(user_buf, buf, len);
		memcpy(circ->buf, buf + len, size - len);
	} else {
		memcpy(user_buf, buf, size);
	}

	circ_fill_finish(circ, size);
}

/*
 * Copy data from circular buffer to user buffer
 */
static int circ_copy_to_user(struct iaxxx_circ_buf *circ,
		char __user *buf, int size)
{
	int len;
	int ret;

	len = min(size, circ_cnt_to_end(circ) << 2);
	ret = copy_to_user(buf, circ->buf + (circ->tail << 2), len);
	if (ret)
		return -EFAULT;

	if (size > len) {
		ret = copy_to_user(buf + len, circ->buf, size - len);
		if (ret)
			return -EFAULT;
	}

	circ_peek_finish(circ, size);

	return 0;
}

/*
 * If user buffer is greater then circular queue data size,
 * then copy all data to user buffer.
 * If user buffer is less than circular queue data size,
 * then check each packet and copy only whole packet can
 * fit into user buffer so that there's no split packet
 * stored in user buffer.
 */
static int circ_to_user(struct iaxxx_circ_buf *circ, char __user *buf,
		size_t count, unsigned int *copied)
{
	struct iaxxx_tunnel_header header;
	int hdr_size = sizeof(struct iaxxx_tunnel_header);
	int size;
	unsigned int sum = 0;
	int cnt = min_t(int, circ_cnt(circ) << 2, count);

	if (count > cnt) {
		/*
		 * If user buffer is big enough, copy all
		 * since the circular buffer has already
		 * packet aligned
		 */
		if (circ_copy_to_user(circ, buf, cnt))
			return -EFAULT;

		*copied = cnt;
		return 0;
	}

	/* User buffer is small, so check the packet one by one */
	while (1) {
		size = circ_peek_data(circ, &header, hdr_size);
		if (size < hdr_size)
			break;

		size += header.size;
		if ((circ_cnt(circ) << 2) < size)
			break;

		if (count < (sum + size))
			break;

		if (circ_copy_to_user(circ, buf + sum, size))
			break;

		sum += size;
	}

	if (sum == 0)
		return -EFAULT;

	*copied = sum;
	return 0;
}

/*
 * Parse packet header and return tunnel ID and data len
 */
static inline int parse_header(struct iaxxx_tunnel_header *header, u16 *tid)
{
	if (header->magic[0] != TUNNEL_MAGIC1 ||
		header->magic[1] != TUNNEL_MAGIC2 ||
		header->magic[2] != TUNNEL_MAGIC3 ||
		header->magic[3] != TUNNEL_MAGIC4 ||
		!is_valid_tid(header->tunnel_id) ||
		((header->encoding != TNL_ENC_OPAQUE) &&
		!is_valid_size(header->size))) {
		pr_debug("Fault packet: magic: %x %x %x %x, tid = %d, size = %d",
			header->magic[0], header->magic[1],
			header->magic[2], header->magic[3],
			header->tunnel_id, header->size);
		return -EINVAL;
	}

	*tid = header->tunnel_id;
	return header->size;
}

static unsigned long adjust_tunnels_new(struct iaxxx_tunnel_data *t_intf_priv)
{
	struct iaxxx_tunnel_client *client;
	unsigned long new = 0;

	spin_lock(&t_intf_priv->lock);
	list_for_each_entry_rcu(client, &t_intf_priv->list, node)
		new |= client->tid_flag;
	spin_unlock(&t_intf_priv->lock);

	return new;
}

/*
 * Request to subscribe the tunnel events with a threshold
 */
static int tunnel_event_subscribe(struct iaxxx_tunnel_data *t_intf_priv,
					uint32_t src_id, uint32_t evt_id,
					uint32_t dst_id, uint32_t destOpaque,
					uint32_t event_thrshld)
{
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct device *dev = (struct device *)priv->dev;
	int ret = 0;

	pr_debug("%s(): tunnel_evt_cnt: %d\n", __func__,
			atomic_read(&t_intf_priv->tunnel_evt_cnt));

	if (atomic_inc_return(&t_intf_priv->tunnel_evt_cnt) == 1) {
		pr_debug("%s(): flushing the left overs\n", __func__);
		ret = iaxxx_set_tunnel_event_threshold(priv,
					IAXXX_TUNNEL_THRESHOLD);
		if (ret) {
			pr_err("%s failed to set tnl evt threshold %d\n",
						__func__, ret);
			return ret;
		}
		ret = iaxxx_core_evt_subscribe(dev, src_id, evt_id,
						dst_id, destOpaque);
		if (ret) {
			pr_err("%s failed evt subscribe %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

/*
 * Request to unsubscribe the tunnel event
 */
static int tunnel_event_unsubscribe(struct iaxxx_tunnel_data *t_intf_priv,
					uint32_t src_id, uint32_t evt_id,
					uint32_t dst_id, uint32_t destOpaque,
					uint32_t event_thrshld)
{
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct device *dev = (struct device *)priv->dev;
	int ret = 0;

	pr_debug("%s(): tunnel_evt_cnt: %d\n", __func__,
			atomic_read(&t_intf_priv->tunnel_evt_cnt));

	if (atomic_dec_return(&t_intf_priv->tunnel_evt_cnt) == 0) {
		ret = iaxxx_set_tunnel_event_threshold(priv,
						IAXXX_TUNNEL_THRESHOLD);
		if (ret) {
			pr_err("%s failed to set tnl evt threshold %d\n",
						__func__, ret);
			return ret;
		}
		ret = iaxxx_core_evt_unsubscribe(dev, src_id, evt_id,
						dst_id);
		if (ret) {
			pr_err("%s failed evt subscribe %d\n", __func__, ret);
			return ret;
		}
	}

	return ret;
}

/*
 * Adjust tunnel settings by comparing new & old flags
 */
static void adjust_tunnels(struct iaxxx_tunnel_data *t_intf_priv)
{
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct iaxxx_tunnel_ep *tnl_src_node = NULL;
	struct list_head *position, *tmp;
	int tnl_count = 0;
	unsigned long *flags = &t_intf_priv->flags;
	unsigned long changes, old, new = 0;
	int id, pos;

	if (!test_and_clear_bit(IAXXX_TFLG_ADJ_THREAD, flags))
		return;

	old = tunnel_flags;

	if (!test_and_clear_bit(IAXXX_TFLG_DISABLE_ALL, flags))
		new = adjust_tunnels_new(t_intf_priv);

	if (new == old)
		return;

	changes = (old ^ new) & ((1 << TNLMAX) - 1);

	while (changes) {
		id = ffs(changes) - 1;
		pos = (1 << id);

		if (new & pos) {
			list_for_each_safe(position, tmp,
						&t_intf_priv->src_list) {
				tnl_src_node = list_entry(position,
					struct iaxxx_tunnel_ep, src_head_list);

				if (tnl_src_node->tnl_ep.tunlEP == id) {
					pr_notice(
					"setup tnl%d, src%x, mode%s, enc%s\n",
					id,
					tnl_src_node->tnl_ep.tunlSrc & 0xffff,
					(tnl_src_node->tnl_ep.tunlMode
					== TNL_SYNC_MODE) ? "SYNC" : "ASYNC",
					(tnl_src_node->tnl_ep.tunlEncode
					== TNL_ENC_AFLOAT) ? "AFLOAT" :
					(tnl_src_node->tnl_ep.tunlEncode
					== TNL_ENC_Q15) ? "Q15" : "Other");

					if (iaxxx_tunnel_setup_hw(priv,
					tnl_src_node->tnl_ep.tunlEP,
					tnl_src_node->tnl_ep.tunlSrc & 0xffff,
					tnl_src_node->tnl_ep.tunlMode,
					tnl_src_node->tnl_ep.tunlEncode)) {
						pr_err("%s: iaxxx_tunnel_setup_hw failed\n",
							__func__);
						return;
					}
					/* Init sequence no */
					t_intf_priv->tunnel_seq_no[id] = 0;
					t_intf_priv->tunnels_active_count++;
				}
			}

			if (!t_intf_priv->event_registered) {
				if (!tunnel_event_subscribe(t_intf_priv,
					IAXXX_SYSID_TUNNEL_EVENT, 0,
					IAXXX_SYSID_HOST, 0,
					IAXXX_TUNNEL_THRESHOLD)) {
					t_intf_priv->event_registered =  true;
				} else {
					pr_err("tnl events subscribe failed");
					return;
				}
			}
		} else {
			pr_notice("terminate tunnel %d\n", id);
			if (iaxxx_tunnel_terminate_hw(priv, id))
				pr_err("iaxxx_tunnel_terminate failed\n");

			if (t_intf_priv->tunnels_active_count > 0)
				t_intf_priv->tunnels_active_count--;
			else
				pr_err("decrementing tunnel active count exceeded\n");

			list_for_each_safe(position,
					tmp, &t_intf_priv->src_list) {
				tnl_src_node =
				list_entry(position,
				struct iaxxx_tunnel_ep, src_head_list);
				/* map and remove the src node from list */
				if (tnl_src_node->tnl_ep.tunlEP == id) {
					list_del(position);
					kfree(tnl_src_node);
					atomic_set(
					&t_intf_priv->src_enable_id[id], 0);
				}
			}
			while (tnl_count < TNLMAX) {
				if (atomic_read(
				&t_intf_priv->src_enable_id[tnl_count]) == 1)
					break;
				tnl_count++;
			}
			if (tnl_count == TNLMAX &&
				t_intf_priv->event_registered) {
				if (!tunnel_event_unsubscribe(t_intf_priv,
					IAXXX_SYSID_TUNNEL_EVENT, 0,
					IAXXX_SYSID_HOST, 0, 0)) {
					t_intf_priv->event_registered =  false;
				} else {
					pr_err("subscribe tunnel events failed");
					return;
				}
			}
		}
		changes &= ~pos;

		old ^= pos;
	}
	tunnel_flags = old;
}

static void adjust_tunnels_do(struct iaxxx_tunnel_data *tpriv,
				struct iaxxx_tunnel_client *client)
{
	if (client && test_and_clear_bit(IAXXX_TFLG_ADJ_CLIENT, &tpriv->flags))
		set_bit(IAXXX_TFLG_ADJ_THREAD, &tpriv->flags);

	if (test_bit(IAXXX_TFLG_ADJ_THREAD, &tpriv->flags))
		wake_up(&tpriv->producer_wq);
}

/*
 * Get tunnel data from codec and fill * into circular buffer
 */
static int producer_thread(void *arg)
{
	struct iaxxx_tunnel_data *t_intf_priv = arg;
	struct iaxxx_priv *priv = t_intf_priv->priv;
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int size;
	int bytes, bytes_remaining;
	void *buf;
	int wait_time_us = PRODUCER_WAIT_TIME_US;

	while (1) {
		/* Get a free contiguous buffer */
		if (kthread_should_stop()) {
			set_bit(IAXXX_TFLG_DISABLE_ALL, &t_intf_priv->flags);
			set_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags);
			pr_debug("thread should stop\n");
			break;
		}

		if (kthread_should_park()) {
			pr_debug("parking producer thread\n");
			adjust_tunnels(t_intf_priv);
			kthread_parkme();
			continue;
		}

		if (!test_bit(IAXXX_TFLG_ALLOW_TUNNELS, &t_intf_priv->flags)) {
			dev_info(t_intf_priv->dev,
				"Waiting until tunnels is allowed...\n");
			wait_event(t_intf_priv->producer_wq,
				kthread_should_stop() ||
				kthread_should_park() ||
				test_bit(IAXXX_TFLG_ALLOW_TUNNELS,
					&t_intf_priv->flags));
			dev_info(t_intf_priv->dev, "Tunnels allowed!\n");

			/* Check for reasons stop, park or other */
			if (!test_bit(IAXXX_TFLG_ALLOW_TUNNELS,
				&t_intf_priv->flags))
				continue;

			set_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags);
		}

		if (test_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags))
			adjust_tunnels(t_intf_priv);

		size = circ_get_free_buffer(circ, &buf);

		WARN_ONCE(size < 4, "No more buffer");

		if (size >= 4 && t_intf_priv->tunnels_active_count > 0) {
			/* Get the data max size words.
			 * Sync mode reading is enabled if at least
			 * one tunnel is configured in sync mode
			 */
			bytes = iaxxx_tunnel_read_hw(priv,
					buf, size >> 2,
					t_intf_priv->flags_sync_mode,
					&bytes_remaining) << 2;

			pr_debug("%s: buf = %p sz = %d, bytes = %d, cnt = %d\n",
					__func__, buf, size,
					bytes, circ_cnt(circ) << 2);

			if (bytes > 0) {
				circ_fill_finish(circ, bytes);

				/*
				 * Wakeup consumer only when client
				 * is registered.
				 * If only debug client (client_no==0)
				 * just keep matching tail with head
				 */
				if (t_intf_priv->client_no > 0)
					wake_up(&t_intf_priv->consumer_wq);
				else
					circ->tail = circ->head;

				/* Restore timeout value */
				wait_time_us = PRODUCER_WAIT_TIME_US;

				/*
				 * Interrupt is not triggerred if the
				 * remaining data is bigger than threshold,
				 * so keep consuming data
				 */
				if ((bytes_remaining << 2) >=
						IAXXX_TUNNEL_THRESHOLD)
					continue;

				if (atomic_xchg
					(&t_intf_priv->event_occurred, 0) > 1)
					continue;

				pr_debug("%s: tnl producer wait for event\n",
							__func__);

				wait_event(t_intf_priv->producer_wq,
					atomic_read(
						&t_intf_priv->event_occurred)
					|| kthread_should_stop() ||
					test_bit(IAXXX_TFLG_ADJ_THREAD,
						&t_intf_priv->flags));
				continue;
			}
		}

		if (tunnel_flags) {
			usleep_range(wait_time_us, wait_time_us + 5);
			/*
			 * If failed, keep increasing wait time to
			 * 100us until MAX
			 */
			wait_time_us += 100;
			wait_time_us = min(wait_time_us,
					PRODUCER_MAX_WAIT_TIME_US);
			continue;
		}
		pr_info("%s: producer thread wait for start\n", __func__);
		wait_event(t_intf_priv->producer_wq,
			kthread_should_stop() ||
			kthread_should_park() ||
			test_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags));
	}

	adjust_tunnels(t_intf_priv);
	return 0;
}

/*
 * Attach a client to tunnel stream
 */
static int tunneling_attach_client(struct iaxxx_tunnel_data *tunnel_data,
			struct iaxxx_tunnel_client *client)
{
	client->user_circ.buf = kmalloc(UBUFF_SIZE, GFP_KERNEL);
	if (!client->user_circ.buf)
		return -ENOMEM;

	client->user_circ.size = UBUFF_SIZE >> 2;
	circ_init(&client->user_circ);

	client->tunnel_data = tunnel_data;

	spin_lock(&tunnel_data->lock);
	list_add_tail_rcu(&client->node, &tunnel_data->list);
	tunnel_data->client_no++;
	spin_unlock(&tunnel_data->lock);

	return 0;
}

/*
 * Detach a client from tunnel stream
 */
static int tunneling_detach_client(struct iaxxx_tunnel_data *tunnel_data,
			struct iaxxx_tunnel_client *client)
{
	spin_lock(&tunnel_data->lock);
	list_del_rcu(&client->node);
	tunnel_data->client_no--;
	spin_unlock(&tunnel_data->lock);
	synchronize_rcu();

	kfree(client->user_circ.buf);

	return 0;
}

/*
 * Return tunned id for given src end
 */
static int tunnel_find_id(struct iaxxx_tunnel_data *t_intf_priv,
			int src, uint32_t mode, uint32_t encode, int set)
{
	struct iaxxx_tunnel_ep *tnl_src_node;
	struct list_head *pos, *tmp;
	int id = 0;
	int unused_id = 0;

	if (!list_empty_careful(&t_intf_priv->src_list)) {
		list_for_each_safe(pos, tmp, &t_intf_priv->src_list) {
			tnl_src_node = list_entry(pos,
				struct iaxxx_tunnel_ep, src_head_list);
			if (tnl_src_node->tnl_ep.tunlSrc == src &&
				tnl_src_node->tnl_ep.tunlEncode == encode &&
				tnl_src_node->tnl_ep.tunlMode == mode) {
				pr_debug("%s: id exist: %d\n",
					__func__, tnl_src_node->tnl_ep.tunlEP);
				return tnl_src_node->tnl_ep.tunlEP;
			}
			/* update id if the current end point id is
			 * greter than previous id.
			 */
			if (tnl_src_node->tnl_ep.tunlEP > id)
				id = tnl_src_node->tnl_ep.tunlEP;
		}
		/* Add one to id.
		 * This is will be the next max best id to assign
		 */
		id += 1;
	} else
		return 0;

	if (!set)
		return -ENOENT;

	pr_debug("%s: new id : %d\n", __func__, id);
	/* see if any unused id can be found between the next best id */
	while (unused_id < id) {
		if (atomic_read(&t_intf_priv->src_enable_id[unused_id]) == 0)
			return unused_id;
		unused_id++;
	}

	/* If no unused id is found then check
	 * if next best ID is greater then or equal to max
	 * tunnel id then it should return error
	 * else return the next best id as tunnel id to assign.
	 */
	if (id >= TNLMAX) {
		pr_err("%s: id : %d\n", __func__, id);
		return -EINVAL;
	}

	return id;
}

/*
 * Copy buffer to a client's buffer
 */
static void tunnel_copy_to_client(struct iaxxx_tunnel_client *client,
				u16 tid, int count)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int size;

	if (!is_valid_tid(tid) || !test_bit(tid, &client->tid_flag))
		return;

	if ((circ_space(&client->user_circ) << 2) < count)
		/* drop the current packet */
		return;

	size = min(count, circ_cnt_to_end(circ) << 2);
	circ_in(&client->user_circ, circ->buf + (circ->tail << 2), size);
	if (size < count)
		circ_in(&client->user_circ, circ->buf, count - size);

	wake_up(&client->wq);
}

#ifdef CHECK_SEQUENCE
static void check_packet(struct iaxxx_tunnel_data *t_intf_priv,
			u16 tid, uint32_t seq_no)
{
	if (t_intf_priv->tunnel_seq_no[tid] == 0) {
		t_intf_priv->tunnel_seq_no[tid] = seq_no;
		return;
	}

	if ((seq_no - t_intf_priv->tunnel_seq_no[tid]) != 1) {
		pr_debug("Sequence number error id = %d old = %x, new = %x\n",
			tid,
			t_intf_priv->tunnel_seq_no[tid],
			seq_no);
		t_intf_priv->tunnel_seq_err[tid] +=
			seq_no - t_intf_priv->tunnel_seq_no[tid];
	}

	t_intf_priv->tunnel_seq_no[tid] = seq_no;
	t_intf_priv->tunnel_packet_no[tid]++;
	t_intf_priv->tunnel_total_packet_no++;
}
#endif

/*
 * Consume circular buffer and feed data to each client kfifo
 */
static int consumer_thread(void *arg)
{
	struct iaxxx_tunnel_data *t_intf_priv = arg;
	struct iaxxx_tunnel_client *client;
	u16 tid;
	int rc;
	int size;
	int sz_used;
	struct iaxxx_tunnel_header hdr;
	int hdr_size = sizeof(hdr);
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;

	while (1) {
		if (kthread_should_stop())
			break;

		if (kthread_should_park()) {
			kthread_parkme();
			continue;
		}

		size = 0;
		sz_used = circ_cnt(circ) << 2;
		if (sz_used >= hdr_size) {
			circ_peek_data(circ, &hdr, hdr_size);

			size = parse_header(&hdr, &tid);
			if (size < 0) {
				/* Increase magic error */
				t_intf_priv->tunnel_magic_errcnt++;

				/* Packet header error */
				circ_peek_finish(circ, 4);
				continue;
			}
			if (size > 0)
				size += hdr_size;
		}

		if (size > 0 && sz_used >= size) {
			rcu_read_lock();
			/* Fill to each client's fifo */
			list_for_each_entry_rcu(client,
						&t_intf_priv->list, node)
				tunnel_copy_to_client(client, tid, size);
			rcu_read_unlock();
			circ_peek_finish(circ, size);

#ifdef CHECK_SEQUENCE
			if (t_intf_priv->client_no)
				check_packet(t_intf_priv, tid, hdr.seq_no);
#endif

			continue;
		}

		if (sz_used == 0) {
			wait_event(t_intf_priv->consumer_wq,
				(circ_cnt(circ) > 0) ||
				kthread_should_stop() ||
				kthread_should_park());
			continue;
		}

		/* We got some data but not enough */
		rc = wait_event_timeout(t_intf_priv->consumer_wq,
			((circ_cnt(circ) << 2) > sz_used) ||
			kthread_should_park() ||
			kthread_should_stop(),
			HZ);

		sz_used = circ_cnt(circ) << 2;
		if (!rc && sz_used < size) {
			pr_err("%s: Consume invalid packet in circ queue\n",
				__func__);
			circ_peek_finish(circ, sz_used);
		}
	}

	return 0;
}

/*
 * Request to setup a tunnel to producer
 */
int iaxxx_tunnel_setup(struct iaxxx_tunnel_client *client, uint32_t src,
				uint32_t mode, uint32_t encode)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_tunnel_ep *tnl_src_node;
	int id;
	int rc = 0;

	pr_debug("%s(): src 0x%x\n", __func__, src);
	id = tunnel_find_id(t_intf_priv, src, mode, encode, true);
	if (id < 0)
		return -EINVAL;

	pr_debug("%s id found %d already there :%d\n",
		__func__, id, atomic_read(&t_intf_priv->src_enable_id[id]));
	if (atomic_read(&t_intf_priv->src_enable_id[id]) == 0) {
		/* Allocate tunnel endpoint list for the tunneling */
		tnl_src_node =
		kzalloc(sizeof(struct iaxxx_tunnel_ep), GFP_KERNEL);
		if (!tnl_src_node)
			return -ENOMEM;
		tnl_src_node->tnl_ep.tunlEP = id;
		tnl_src_node->tnl_ep.tunlSrc = src;
		tnl_src_node->tnl_ep.tunlMode = mode;
		tnl_src_node->tnl_ep.tunlEncode = encode;
		/* Add tunnel endpoint to tunnel src list */
		list_add_tail(&tnl_src_node->src_head_list,
				&t_intf_priv->src_list);
		atomic_set(&t_intf_priv->src_enable_id[id], 1);
	}

	if (!test_and_set_bit(id, &client->tid_flag))
		set_bit(IAXXX_TFLG_ADJ_CLIENT, &t_intf_priv->flags);

	if (atomic_inc_return(&t_intf_priv->tunnel_ref_cnt[id]) == 1) {
		pr_err("%s id found %d ref count :%d\n",
			__func__, id,
			atomic_read(&t_intf_priv->tunnel_ref_cnt[id]));
		/* Set flag if this tunnel id is in sync mode */
		if (mode == TNL_SYNC_MODE)
			set_bit(id, &t_intf_priv->flags_sync_mode);
	}

	adjust_tunnels_do(t_intf_priv, client);
	pr_debug("%s: tid: %x src: %x client flag: %lx\n",
		__func__, id, src, client->tid_flag);

	return rc;
}

/*
 * Terminate a tunnel for a client (Producer will terminate the tunnel)
 */
int iaxxx_tunnel_term(struct iaxxx_tunnel_client *client, uint32_t src,
				uint32_t mode, uint32_t encode)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	int id;
	int rc = 0;

	id = tunnel_find_id(t_intf_priv, src, mode, encode, false);
	if (id < 0)
		return -EINVAL;

	if (!test_and_clear_bit(id, &client->tid_flag) ||
		!atomic_read(&t_intf_priv->tunnel_ref_cnt[id]))
		return -EINVAL;

	set_bit(IAXXX_TFLG_ADJ_CLIENT, &t_intf_priv->flags);

	if (atomic_dec_return(&t_intf_priv->tunnel_ref_cnt[id]) == 0) {
		if (mode == TNL_SYNC_MODE)
			clear_bit(id, &t_intf_priv->flags_sync_mode);
	}

	pr_debug("%s: tid: %x src: %x client flag: %lx global flag: %lx\n",
		__func__, id, src, client->tid_flag, t_intf_priv->flags);

	adjust_tunnels_do(t_intf_priv, client);
	return rc;
}

int iaxxx_tunnel_read(struct file *filp, char __user *buf,
			size_t count)
{
	struct iaxxx_tunnel_client *client = filp->private_data;
	struct iaxxx_circ_buf *user_circ = &client->user_circ;
	unsigned int copied;
	int ret;

	if (!circ_cnt(user_circ)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (!(wait_event_interruptible_timeout(client->wq,
						circ_cnt(user_circ), HZ))) {
			pr_err("Timeout in iaxxx_tunnel_read\n");
			return 0;
		}
	}
	ret = circ_to_user(user_circ, buf, count, &copied);

	return ret ? ret : copied;
}

static ssize_t tunnel_read(struct file *filp, char __user *buf,
			size_t count, loff_t *f_pos)
{
	return iaxxx_tunnel_read(filp, buf, count);
}
int iaxxx_tunnel_open_common(struct inode *inode, struct file *filp, int id)
{
	struct iaxxx_tunnel_data *t_intf_priv = NULL;
	struct iaxxx_tunnel_client *client;
	int rc;

	pr_debug("tunneling device open called");

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode");
		return -ENODEV;
	}

	if (id == TUNNEL_0)
		t_intf_priv = container_of((inode)->i_cdev,
				struct iaxxx_tunnel_data, tunnel_cdev.cdev);
	else if (id == TUNNEL_1)
		t_intf_priv = container_of((inode)->i_cdev,
					struct iaxxx_tunnel_data,
					tunnel_sensor_cdev.cdev);

	if (t_intf_priv == NULL) {
		pr_err("Unable to fetch tunnel private data");
		return -ENODEV;
	}

	if (!pm_runtime_enabled(t_intf_priv->dev))
		return -EIO;

	client = kzalloc(sizeof(struct iaxxx_tunnel_client),
				GFP_KERNEL | __GFP_NOWARN);
	if (!client)
		return -ENOMEM;

	rc = tunneling_attach_client(t_intf_priv, client);
	if (rc) {
		kfree(client);
		return rc;
	}

	init_waitqueue_head(&client->wq);

	filp->private_data = client;

	/* taking the sync for the device if open is going to return
	 * successful. this means that a tunneling node is opened and the sync
	 * would be held till the close is called. This would also take
	 * care of the case of multiple clients as well
	 */
	pm_runtime_get_sync(t_intf_priv->dev);
	set_bit(IAXXX_TFLG_ALLOW_TUNNELS, &t_intf_priv->flags);

	return 0;
}

static int tunnel_open(struct inode *inode, struct file *filp)
{
	/* need to be protected to ensure  concurrency*/
	return iaxxx_tunnel_open_common(inode, filp, TUNNEL_0);
}

static long tunnel_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct iaxxx_tunnel_client * const client =
			(struct iaxxx_tunnel_client *)filp->private_data;
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct tunlMsg msg;
	int ret = 0;

	if (!priv) {
		pr_err("Unable to fetch tunnel private data\n");
		return -EINVAL;
	}

	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "Chip state NULL\n");
		return -EINVAL;
	}

	if (!pm_runtime_enabled(t_intf_priv->dev)) {
		dev_err(priv->dev, "FW state not valid %s()\n", __func__);
		return -EIO;
	}

	if (arg != 0 && _IOC_DIR(cmd) == (_IOC_READ | _IOC_WRITE)
		&& _IOC_SIZE(cmd) == sizeof(struct tunlMsg)) {
		if (copy_from_user(&msg, (void __user *)arg,
					sizeof(struct tunlMsg))) {
			pr_err("parameter copy from user failed\n");
			return -EFAULT;
		}

		pr_debug("cmd: %x, TunlSrc: %x, tunlMode: %x, tunlEncode: %x",
			cmd, (uint32_t)msg.tunlSrc, (uint32_t)msg.tunlMode,
			(uint32_t)msg.tunlEncode);
	}

	switch (cmd) {
	case TUNNEL_SETUP:
		ret = iaxxx_tunnel_setup(client, msg.tunlSrc, msg.tunlMode,
					msg.tunlEncode);
		if (ret) {
			pr_err("Unable to setup tunnel");
			return	-EINVAL;
		}
		break;
	case TUNNEL_TERMINATE:
		ret = iaxxx_tunnel_term(client, msg.tunlSrc, msg.tunlMode,
					msg.tunlEncode);
		if (ret) {
			pr_err("Unable to terminate tunnel");
			ret = -EINVAL;
		}
		break;
	default:
		pr_err("Invalid ioctl command received %x", cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long tunnel_compat_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	return tunnel_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

int iaxxx_tunnel_release_common(struct inode *inode, struct file *filp, int id)
{
	struct iaxxx_tunnel_data *t_intf_priv = NULL;
	struct iaxxx_tunnel_client *client;

	pr_debug("tunneling device release called");

	if (inode == NULL) {
		pr_err("invalid inode pointer");
		goto error;
	}

	if ((inode)->i_cdev == NULL) {
		pr_err("invalid cdev pointer in inode");
		goto error;
	}

	if (id == TUNNEL_0)
		t_intf_priv = container_of((inode)->i_cdev,
				struct iaxxx_tunnel_data, tunnel_cdev.cdev);
	else if (id == TUNNEL_1)
		t_intf_priv = container_of((inode)->i_cdev,
					struct iaxxx_tunnel_data,
					tunnel_sensor_cdev.cdev);

	if (t_intf_priv == NULL) {
		pr_err("unable to fetch tunnel private data");
		goto error;
	}

	client = filp->private_data;
	if (client) {
		tunneling_detach_client(t_intf_priv, client);
		set_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags);
		adjust_tunnels_do(t_intf_priv, NULL);
	}
	/* ignore threadfn return value */
	pr_debug("stopping stream kthread");
	pm_runtime_put_sync(t_intf_priv->dev);
	kfree(client);
	filp->private_data = NULL;

error:
	return 0;
}

static int tunnel_release(struct inode *inode, struct file *filp)
{
	return iaxxx_tunnel_release_common(inode, filp, TUNNEL_0);
}

static unsigned int tunnel_poll(struct file *filep,
	struct poll_table_struct *wait)
{
	struct iaxxx_tunnel_client *const client = filep->private_data;
	unsigned int mask = 0;

	poll_wait(filep, &client->wq, wait);
	if (circ_cnt(&client->user_circ))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations tunneling_fops = {
	.owner = THIS_MODULE,
	.open = tunnel_open,
	.read = tunnel_read,
	.unlocked_ioctl	= tunnel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= tunnel_compat_ioctl,
#endif
	.release = tunnel_release,
	.poll = tunnel_poll,
};

/*
 * Show tunnel status, enable, num of clients etc.
 */
static ssize_t iaxxx_tunnel_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_tunnel_ep *tnl_src_node;
	struct list_head *pos, *tmp;
	int index;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	scnprintf(buf, PAGE_SIZE,
	"Tunnel ID\tSRC\tEnable\tMode\tEncode\tNum of Clients\tNo Packets\tError Packets\n");
	list_for_each_safe(pos, tmp, &t_intf_priv->src_list) {
		int enable = true;

		index = strnlen(buf, PAGE_SIZE);
		tnl_src_node = list_entry(pos,
				struct iaxxx_tunnel_ep, src_head_list);
		scnprintf(&buf[index], PAGE_SIZE - index,
		"    %02d    \t%04x\t%d\t%s\t%s\t\t%d\t%lu\t\t%d\n",
		tnl_src_node->tnl_ep.tunlEP,
		tnl_src_node->tnl_ep.tunlSrc & 0xffff,
		enable,
		(tnl_src_node->tnl_ep.tunlMode == TNL_SYNC_MODE) ?
		"SYNC" : "ASYNC",
		((tnl_src_node->tnl_ep.tunlEncode == TNL_ENC_AFLOAT) ?
		"AFLOAT" : (tnl_src_node->tnl_ep.tunlEncode == TNL_ENC_Q15) ?
		"Q15" : "Other"),
		atomic_read(
		&t_intf_priv->tunnel_ref_cnt[tnl_src_node->tnl_ep.tunlEP]),
		t_intf_priv->tunnel_packet_no[tnl_src_node->tnl_ep.tunlEP],
		t_intf_priv->tunnel_seq_err[tnl_src_node->tnl_ep.tunlEP]);
	}

	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
		"\nTotal Packets: %lu\n",
		t_intf_priv->tunnel_total_packet_no);

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t iaxxx_tunnel_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	if (tunnel_flags)
		return -EPERM;

	/* Clear packet history */
	memset(t_intf_priv->tunnel_packet_no, 0,
		sizeof(t_intf_priv->tunnel_packet_no));
	memset(t_intf_priv->tunnel_seq_err, 0,
		sizeof(t_intf_priv->tunnel_seq_err));
	t_intf_priv->tunnel_total_packet_no = 0;

	return count;
}

static DEVICE_ATTR(tunnel_status, 0600,
		iaxxx_tunnel_status_show, iaxxx_tunnel_status_store);

/*
 * Get the information on producer circular queue
 */
static ssize_t iaxxx_tunnel_circ_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_circ_buf *circ;
	int index;
	int cnt;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	circ = &t_intf_priv->stream_circ;

	cnt = circ_cnt_to_end(circ) << 2;

	scnprintf(buf, PAGE_SIZE,
		"Circular: head %d, tail %d, cnt %d, space %d\n",
		circ->head, circ->tail, cnt, circ_space(circ) << 2);

	index = strnlen(buf, PAGE_SIZE);

	hex_dump_to_buffer(circ->buf + (circ->tail << 2), cnt, 16, 1,
			&buf[index], PAGE_SIZE - index, false);
	return index;
}
static DEVICE_ATTR(tunnel_circ, 0400, iaxxx_tunnel_circ_show, NULL);

/*
 * Show packet header magic error
 */
static ssize_t iaxxx_tunnel_header_errcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE,
		"Packet Header Error Count = %ld\n",
		t_intf_priv->tunnel_magic_errcnt);
}
static DEVICE_ATTR(tunnel_header_errcnt, 0400,
			iaxxx_tunnel_header_errcnt_show, NULL);

/*
 * Show sequence number error
 */
static ssize_t iaxxx_tunnel_seqno_errcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_tunnel_ep *tnl_src_node;
	struct list_head *pos, *tmp;
	int index;
	uint64_t sum = 0;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	scnprintf(buf, PAGE_SIZE,
		"Tunnel ID\tTunnel Src ID\tSeqno Error Count\n");
	list_for_each_safe(pos, tmp, &t_intf_priv->src_list) {
		index = strnlen(buf, PAGE_SIZE);
		tnl_src_node = list_entry(pos,
			struct iaxxx_tunnel_ep, src_head_list);
		scnprintf(&buf[index], PAGE_SIZE - index,
		"    %02d    \t%04x\t\t\t%d\n",
		tnl_src_node->tnl_ep.tunlEP,
		tnl_src_node->tnl_ep.tunlSrc,
		t_intf_priv->tunnel_seq_err[tnl_src_node->tnl_ep.tunlEP]);
		sum +=
		t_intf_priv->tunnel_seq_err[tnl_src_node->tnl_ep.tunlEP];
	}

	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
			"Total error: %llu\n", sum);

	return strnlen(buf, PAGE_SIZE);
}
static DEVICE_ATTR(tunnel_seqno_errcnt, 0400,
			iaxxx_tunnel_seqno_errcnt_show, NULL);

/*
 * sysfs attr info
 */
static struct attribute *iaxxx_attrs[] = {
	&dev_attr_tunnel_status.attr,
	&dev_attr_tunnel_circ.attr,
	&dev_attr_tunnel_header_errcnt.attr,
	&dev_attr_tunnel_seqno_errcnt.attr,
	NULL,
};

/*
 * sysfs attr group info
 */
static const struct attribute_group iaxxx_attr_group = {
	.attrs = iaxxx_attrs,
};

/*
 * Init early stage before firmware download
 */
int iaxxx_tunnel_dev_init_early(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *t_intf_priv = NULL;

	t_intf_priv = devm_kzalloc(priv->dev, sizeof(*t_intf_priv), GFP_KERNEL);
	if (!t_intf_priv)
		return -ENOMEM;

	priv->tunnel_data = t_intf_priv;
	t_intf_priv->priv = priv;

	return 0;
}

static void iaxxx_tunnel_stop(struct iaxxx_tunnel_data *t_intf_priv)
{
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int i;

	if (!test_and_clear_bit(IAXXX_TFLG_RUN_THREAD, &t_intf_priv->flags))
		return;

	kthread_park(t_intf_priv->producer_thread);
	kthread_park(t_intf_priv->consumer_thread);

	circ_init(circ);
	atomic_set(&t_intf_priv->event_occurred, 0);
	atomic_set(&t_intf_priv->tunnel_evt_cnt, 0);
	for (i = 0; i < TNLMAX; i++) {
		t_intf_priv->tunnel_seq_err[i] = 0;
		t_intf_priv->tunnel_seq_no[i] = 0;
		t_intf_priv->tunnel_packet_no[i] = 0;
	}
	t_intf_priv->tunnel_total_packet_no = 0;
	t_intf_priv->tunnel_magic_errcnt = 0;
}

static int iaxxx_tunnel_start(struct iaxxx_tunnel_data *priv)
{
	int err = 0;

	if (test_and_set_bit(IAXXX_TFLG_RUN_THREAD, &priv->flags))
		return -EEXIST;

	clear_bit(IAXXX_TFLG_DISABLE_ALL, &priv->flags);
	set_bit(IAXXX_TFLG_ADJ_THREAD, &priv->flags);

	kthread_unpark(priv->producer_thread);
	kthread_unpark(priv->consumer_thread);

	return err;
}

static int iaxxx_notifier_cb(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct iaxxx_tunnel_data *priv = container_of(nb,
			struct iaxxx_tunnel_data, crash_notifier);
	int ret = 0;

	switch (val) {
	case IAXXX_EV_RECOVERY:
		if (!test_and_set_bit(IAXXX_TFLG_ALLOW_TUNNELS, &priv->flags))
			wake_up_process(priv->producer_thread);
		break;

	case IAXXX_EV_CRASH:
		set_bit(IAXXX_TFLG_FW_CRASH, &priv->flags);
		break;

	case IAXXX_EV_ROUTE_ACTIVE:
		if (!test_and_set_bit(IAXXX_TFLG_ALLOW_TUNNELS, &priv->flags))
			wake_up_process(priv->producer_thread);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int iaxxx_tunnel_suspend_rt(struct device *dev)
{
	struct iaxxx_priv *priv = dev_get_drvdata(dev);
	struct iaxxx_tunnel_data *tpriv = priv->tunnel_data;

	if (!test_bit(IAXXX_TFLG_FW_CRASH, &tpriv->flags)) {
		set_bit(IAXXX_TFLG_DISABLE_ALL, &tpriv->flags);
		set_bit(IAXXX_TFLG_ADJ_THREAD, &tpriv->flags);
	}

	iaxxx_tunnel_stop(tpriv);

	return 0;
}

static int iaxxx_tunnel_resume_rt(struct device *dev)
{
	struct iaxxx_priv *priv = dev_get_drvdata(dev);
	struct iaxxx_tunnel_data *tpriv = priv->tunnel_data;

	if (test_bit(IAXXX_TFLG_FW_CRASH, &tpriv->flags)) {
		clear_bit(IAXXX_TFLG_ALLOW_TUNNELS, &tpriv->flags);
		clear_bit(IAXXX_TFLG_THRESHOLD, &tpriv->flags);
	}

	iaxxx_tunnel_start(tpriv);

	clear_bit(IAXXX_TFLG_FW_CRASH, &tpriv->flags);

	return 0;
}

static int iaxxx_tunnel_dev_suspend(struct device *dev)
{
	return iaxxx_tunnel_suspend_rt(dev);
}

static int iaxxx_tunnel_dev_resume(struct device *dev)
{
	return iaxxx_tunnel_resume_rt(dev);
}


/*
 * Init remaining stuffs
 */
static int iaxxx_tunnel_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_tunnel_data *t_intf_priv = NULL;
	struct iaxxx_priv *priv = NULL;
	int err;

	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("No device data found\n");
		return -EINVAL;
	}

	pr_debug("%s: initializing tunneling", __func__);

	dev_set_drvdata(dev, priv);
	t_intf_priv = priv->tunnel_data;
	t_intf_priv->event_registered = false;
	t_intf_priv->dev = dev;

	if (iaxxx_prod_buf) {
		/* If reserved memory exists, use it */
		t_intf_priv->stream_circ.buf = phys_to_virt(iaxxx_prod_buf);
		t_intf_priv->stream_circ.size =
			iaxxx_prod_buf_size >> 2;
		pr_debug("%s: carvout %p, %zd\n",
			__func__, t_intf_priv->stream_circ.buf,
			iaxxx_prod_buf_size);
	} else {
		/* If no reserved, allocate default memory */
		t_intf_priv->stream_circ.buf = devm_kmalloc(priv->dev,
						PBUFF_SIZE, GFP_KERNEL);
		t_intf_priv->stream_circ.size = PBUFF_SIZE >> 2;
		iaxxx_prod_buf_size = PBUFF_SIZE;
	}

	if (!t_intf_priv->stream_circ.buf) {
		err = -ENOMEM;
		goto error_circ_buf;
	}

	/* Initialize client structure */
	INIT_LIST_HEAD(&t_intf_priv->list);
	spin_lock_init(&t_intf_priv->lock);

#ifdef CONFIG_MFD_IAXXX_SENSOR_TUNNEL
	iaxxx_sensor_tunnel_init(priv);
#endif
	err = iaxxx_cdev_create(&t_intf_priv->tunnel_cdev, dev,
		&tunneling_fops, t_intf_priv, IAXXX_CDEV_TUNNEL);
	if (err) {
		pr_err("error in creating the char device");
		err = -EIO;
		goto error_cdev;
	}


	INIT_LIST_HEAD(&t_intf_priv->src_list);

	/* Create producer thread */
	init_waitqueue_head(&t_intf_priv->producer_wq);
	t_intf_priv->producer_thread = kthread_run(producer_thread,
			t_intf_priv, "iaxxx tunnel producer thread");
	if (IS_ERR(t_intf_priv->producer_thread)) {
		pr_err("Cannot create producer thread\n");
		err = PTR_ERR(t_intf_priv->producer_thread);
		goto error_producer_thread;
	}

	/* Create consumer thread */
	init_waitqueue_head(&t_intf_priv->consumer_wq);
	t_intf_priv->consumer_thread = kthread_run(consumer_thread,
			t_intf_priv, "iaxxx tunnel consumer thread");
	if (IS_ERR(t_intf_priv->consumer_thread)) {
		pr_err("Cannot create consumer thread\n");
		err = PTR_ERR(t_intf_priv->consumer_thread);
		goto error_consumer_thread;
	}

	if (sysfs_create_group(&priv->dev->kobj, &iaxxx_attr_group))
		pr_err("Cannot create tunnel sysfs\n");

	pr_debug("streaming cdev initialized.\n");

	t_intf_priv->crash_notifier.notifier_call = iaxxx_notifier_cb;
	err = iaxxx_fw_notifier_register(priv->dev,
					&t_intf_priv->crash_notifier);
	if (err) {
		dev_err(dev, "%s: failed to register for fw notifier\n",
				__func__);
		goto error_consumer_thread;
	}

	pm_runtime_enable(dev);
	return 0;

error_consumer_thread:
	kthread_stop(t_intf_priv->producer_thread);
error_producer_thread:
error_cdev:
error_circ_buf:
	if (err && t_intf_priv)
		kfree(t_intf_priv);

	return err;
}

static int iaxxx_tunnel_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv = NULL;
	struct iaxxx_tunnel_data *t_intf_priv = NULL;

	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	t_intf_priv = priv->tunnel_data;

	iaxxx_cdev_destroy(&t_intf_priv->tunnel_cdev);
#ifdef CONFIG_MFD_IAXXX_SENSOR_TUNNEL
	iaxxx_sensor_tunnel_exit(priv);
#endif
	sysfs_remove_group(&priv->dev->kobj, &iaxxx_attr_group);
	kthread_stop(t_intf_priv->producer_thread);
	kthread_stop(t_intf_priv->consumer_thread);
	kfree(t_intf_priv);
	return 0;
}

int iaxxx_tunnel_signal_event(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *t_intf_priv = priv->tunnel_data;

	atomic_inc(&t_intf_priv->event_occurred);
	wake_up(&t_intf_priv->producer_wq);
	return 0;
}
EXPORT_SYMBOL(iaxxx_tunnel_signal_event);

static int __init iaxxx_reserve_audio_buffer(char *p)
{
	char *old_p = p;
	unsigned long start, size;

	if (!p)
		return 0;

	size = memparse(p, &p);

	/* Check if value and power of 2 */
	if (p == old_p || !is_power_of_2(size))
		return 0;

	if (*p != '$')
		return 0;

	start = memparse(p + 1, &p);

	if (!iaxxx_prod_buf &&
		!memblock_reserve(start, size)) {
		iaxxx_prod_buf = start;
		iaxxx_prod_buf_size = size;
	}

	return 0;
}

static const struct dev_pm_ops iaxxx_tunnel_pm_ops = {
	SET_RUNTIME_PM_OPS(iaxxx_tunnel_suspend_rt,
			iaxxx_tunnel_resume_rt,
			NULL)
	SET_SYSTEM_SLEEP_PM_OPS(iaxxx_tunnel_dev_suspend,
				iaxxx_tunnel_dev_resume)

};

static const struct of_device_id iaxxx_tunnel_dt_match[] = {
	{.compatible = "knowles,iaxxx-tunnel-celldrv"},
	{}
};

static struct platform_driver iaxxx_tunnel_driver = {
	.probe  = iaxxx_tunnel_dev_probe,
	.remove = iaxxx_tunnel_dev_remove,
	.driver = {
		.name = "iaxxx-tunnel-celldrv",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_tunnel_dt_match,
		.pm = &iaxxx_tunnel_pm_ops,
	},
};

static int __init iaxxx_tunnel_init(void)
{
	int ret;

	ret = platform_driver_register(&iaxxx_tunnel_driver);
	if (ret)
		pr_err("Failed to register tunnel platform driver");
	return ret;
}

static void __exit iaxxx_tunnel_exit(void)
{
	platform_driver_unregister(&iaxxx_tunnel_driver);
}

early_param("audio_buffer", iaxxx_reserve_audio_buffer);

module_init(iaxxx_tunnel_init);
module_exit(iaxxx_tunnel_exit);

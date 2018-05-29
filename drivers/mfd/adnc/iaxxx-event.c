/*
 * iaxxx-event.c -- IAxxx events
 *
 * Copyright 2017 Knowles Corporation
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

#define DEBUG
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include "iaxxx.h"


/*****************************************************************************
 * iaxxx_core_evt_subscribe()
 * @brief Subscribe to an event
 *
 * @src_id     -   System Id of event source
 * @event_id   -   Event Id
 * @dst_id     -   System Id of event destination
 * @ds_opaque  -   Information sought by destination task when even occurs.
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_evt_subscribe(struct device *dev, uint16_t src_id,
			uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque)
{
	int ret = -EINVAL;
	int status;
	uint32_t sys_id;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s()\n", __func__);

	if (src_id == IAXXX_SYSID_INVALID || dst_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "Invalid System Ids %s()\n", __func__);
		return ret;
	}

	/*
	 * Update all event subscription registers
	 * Event ID, IDS of source and destination, destination opaque
	 */
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_ID_ADDR, event_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		return ret;
	}
	sys_id = ((dst_id << 16) | src_id);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR, sys_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		return ret;
	}
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_ADDR,
			dst_opaque);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		return ret;
	}
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
		(1 << IAXXX_EVT_MGMT_EVT_SUB_REQ_POS),
		IAXXX_EVT_MGMT_EVT_SUB_REQ_MASK);
	if (ret) {
		dev_err(dev, "Update bit failed %s()\n", __func__);
		return ret;
	}
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL(iaxxx_core_evt_subscribe);

/*****************************************************************************
 * iaxxx_core_evt_unsubscribe()
 * @brief UnSubscribe to an event
 *
 * @src_id     -   System Id of event source
 * @event_id   -   Event Id
 * @dst_id     -   System Id of event destination
 * @ds_opaque  -   Information sought by destination task when even occurs.
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_evt_unsubscribe(struct device *dev, uint16_t src_id,
			uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque)
{
	int ret = -EINVAL;
	int status;
	uint32_t sys_id;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s()\n", __func__);

	if (src_id == IAXXX_SYSID_INVALID || dst_id == IAXXX_SYSID_INVALID) {
		dev_err(dev, "Invalid System Ids %s()\n", __func__);
		return ret;
	}
	/*
	 * Update all event subscription registers
	 * Event ID, Subsystem IDS of source and destination, destination
	 *  opaque
	 */
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_ID_ADDR, event_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		return ret;
	}
	sys_id = ((dst_id << 16) | src_id);
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR, sys_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		return ret;
	}
	ret = regmap_write(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_ADDR,
			dst_opaque);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		return ret;
	}
	ret = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
		(1 << IAXXX_EVT_MGMT_EVT_UNSUB_REQ_POS),
		IAXXX_EVT_MGMT_EVT_UNSUB_REQ_MASK);
	if (ret) {
		dev_err(dev, "Update bit failed %s()\n", __func__);
		return ret;
	}
	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL(iaxxx_core_evt_unsubscribe);


/*****************************************************************************
 * iaxxx_core_retrieve_event()
 * @brief Retrieve an event notification
 *
 * @event_id	-	Event Id
 * @data	-	Event data
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_retrieve_event(struct device *dev, uint16_t *event_id,
		uint32_t *data)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int r_index = priv->event_queue->r_index;

	if (!priv)
		return ret;

	dev_dbg(dev, "%s()\n", __func__);

	mutex_lock(&priv->event_queue_lock);
	r_index++;
	/* Check if there are no events */
	if (r_index == (priv->event_queue->w_index + 1)) {
		pr_err("%s Buffer underflow\n", __func__);
		mutex_unlock(&priv->event_queue_lock);
		return ret;
	}
	if (r_index == IAXXX_MAX_EVENTS)
		r_index = 0;

	priv->event_queue->r_index = r_index;
	*event_id = priv->event_queue->event_info[r_index].event_id;
	*data = priv->event_queue->event_info[r_index].data;
	pr_debug("%s() event Id %d, data %d read index %d\n", __func__,
			*event_id, *data, r_index);
	mutex_unlock(&priv->event_queue_lock);
	return 0;
}
EXPORT_SYMBOL(iaxxx_core_retrieve_event);

/**
 * iaxxx_get_event_work - Work function to read events from the event queue
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * This work function is scheduled by the ISR when any data is found in the
 * event queue.
 *
 * This function reads the available events from the queue and passes them
 * along to the event manager.
 */
static void iaxxx_get_event_work(struct work_struct *work)
{
	int rc;
	uint32_t count;
	struct iaxxx_event event;
	struct iaxxx_priv *priv = container_of(work, struct iaxxx_priv,
							event_work_struct);
	struct device *dev = priv->dev;

	mutex_lock(&priv->event_work_lock);

	if (priv->cm4_crashed) {
		dev_dbg(priv->dev, "CM4 crash event handler called:%d\n",
							priv->cm4_crashed);
		if (priv->iaxxx_state->fw_state == FW_APP_MODE) {
			priv->iaxxx_state->fw_state = FW_CRASH;
			if (priv->crash_handler)
				rc = priv->crash_handler(priv);
			goto out;
		}
	}
	/* Read the count of available events */
	rc = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
			&count);
	if (rc) {
		dev_err(dev, "Failed to read EVENT_COUNT, rc = %d\n", rc);
		goto out;
	}

	while (count) {
		rc = iaxxx_next_event_request(priv, &event);
		if (rc) {
			dev_err(dev, "Failed to read event, rc = %d\n", rc);
			goto out;
		}
		rc = iaxxx_event_handler(priv, &event);
		if (rc) {
			dev_err(dev, "Event 0x%.04X:0x%.04X not delivered\n",
					event.event_src, event.event_id);
			goto out;
		}
		/* Read the count of available events */
		rc = regmap_read(priv->regmap,
				IAXXX_EVT_MGMT_EVT_COUNT_ADDR,
				&count);
		if (rc) {
			dev_err(dev, "Failed to read EVENT_COUNT, rc = %d\n",
					rc);
			goto out;
		}
	}

out:
	mutex_unlock(&priv->event_work_lock);
}

/**
 * iaxxx_event_init - Initialize Event Queue
 *
 * @priv : iaxxx private data
 */
int iaxxx_event_init(struct iaxxx_priv *priv)
{
	int rc;

	priv->event_queue = kmalloc(sizeof(struct iaxxx_evt_queue), GFP_KERNEL);
	if (!priv->event_queue)
		return -ENOMEM;
	priv->event_queue->r_index = -1;
	priv->event_queue->w_index = -1;
	priv->event_workq =
		alloc_workqueue("iaxxx-evnt-wq", WQ_MEM_RECLAIM, 0);
	if (!priv->event_workq) {
		pr_err("%s: failed to register event workq\n",
				__func__);
		rc = -ENOMEM;
		kfree(priv->event_queue);
		return rc;
	}
	/* Set the work queue function as iaxxx_get_event_work() */
	INIT_WORK(&priv->event_work_struct, iaxxx_get_event_work);
	return 0;
}

/**
 * iaxxx_event_exit - Free Event Queue
 *
 * @priv : iaxxx private data
 */
void iaxxx_event_exit(struct iaxxx_priv *priv)
{
	kfree(priv->event_queue);
	destroy_workqueue(priv->event_workq);
	priv->event_workq = NULL;
}

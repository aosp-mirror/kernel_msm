/* drivers/input/touchscreen/touch_bus_negotiator.c
 *
 * Touch Bus Negotiator for Google Pixel devices.
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include "touch_bus_negotiator.h"

static void tbn_on_event(struct work_struct *work)
{
	int err = 0;
	struct tbn_context *tbn =
		container_of(work, struct tbn_context, on_event);

	do {
		dev_dbg(tbn->dev, "received message from server\n");
	} while ((err = qmi_recv_msg(tbn->qmi_handle)) == 0);

	if (err != -ENOMSG)
		dev_err(tbn->dev, "qmi_recv_msg() failed with %d\n", err);
}

static void tbn_process_event(
		struct qmi_handle *handle,
		enum qmi_event_type event,
		void *context)
{
	struct tbn_context *tbn = (struct tbn_context *)context;

	switch (event) {
	case QMI_RECV_MSG:
		dev_dbg(tbn->dev, "received event from QMI server\n");
		queue_work(tbn->event_processor, &tbn->on_event);
		break;
	default:
		dev_err(tbn->dev, "unhandled QMI event: %d\n", (int)event);
	}
}

static int tbn_on_server_event(
		struct notifier_block *nb,
		unsigned long event,
		void *data)
{
	struct tbn_context *tbn =
		container_of(nb, struct tbn_context, server_event_notifier);

	switch (event) {
	case QMI_SERVER_ARRIVE:
		schedule_work(&tbn->on_server_arrive);
		break;
	case QMI_SERVER_EXIT:
		schedule_work(&tbn->on_server_exit);
		break;
	default:
		dev_err(tbn->dev, "unhandled server event: %d\n", (int)event);
	}

	return 0;
}

int tbn_request_bus(struct tbn_context *tbn)
{
	int err = 0;
	struct msg_desc req_desc = { 0 }, rsp_desc = { 0 };
	struct tbn_kernel_request_bus_v01 req;
	struct tbn_ssc_release_bus_v01 rsp;

	dev_info(tbn->dev, "kernel requesting bus access from SLPI\n");

	if (!tbn || !tbn->qmi_handle)
		return -EINVAL;

	req_desc.max_msg_len = TBN_KERNEL_REQUEST_BUS_V01_MAX_MSG_LEN;
	req_desc.msg_id      = QMI_TBN_KERNEL_REQUEST_BUS_V01;
	req_desc.ei_array    = tbn_kernel_request_bus_v01_ei;

	rsp_desc.max_msg_len = TBN_SSC_RELEASE_BUS_V01_MAX_MSG_LEN;
	rsp_desc.msg_id      = QMI_TBN_SSC_RELEASE_BUS_V01;
	rsp_desc.ei_array    = tbn_ssc_release_bus_v01_ei;

	mutex_lock(&tbn->service_lock);
	err = qmi_send_req_wait(
			tbn->qmi_handle,
			&req_desc, &req, sizeof(req),
			&rsp_desc, &rsp, sizeof(rsp),
			TBN_REQUEST_BUS_TIMEOUT_MS);
	mutex_unlock(&tbn->service_lock);
	if (err) {
		dev_err(tbn->dev, "qmi_send_req_wait() failed with: %d\n", err);
		return err;
	}

	return 0;
}

int tbn_release_bus(struct tbn_context *tbn)
{
	int err = 0;
	struct msg_desc req_desc = { 0 }, rsp_desc = { 0 };
	struct tbn_kernel_release_bus_v01 req;
	struct tbn_ssc_acquire_bus_v01 rsp;

	dev_info(tbn->dev, "kernel releasing bus access from SLPI\n");

	if (!tbn || !tbn->qmi_handle)
		return -EINVAL;

	req_desc.max_msg_len = TBN_KERNEL_RELEASE_BUS_V01_MAX_MSG_LEN;
	req_desc.msg_id      = QMI_TBN_KERNEL_RELEASE_BUS_V01;
	req_desc.ei_array    = tbn_kernel_release_bus_v01_ei;

	rsp_desc.max_msg_len = TBN_SSC_ACQUIRE_BUS_V01_MAX_MSG_LEN;
	rsp_desc.msg_id      = QMI_TBN_SSC_ACQUIRE_BUS_V01;
	rsp_desc.ei_array    = tbn_ssc_acquire_bus_v01_ei;

	mutex_lock(&tbn->service_lock);
	err = qmi_send_req_wait(
			tbn->qmi_handle,
			&req_desc, &req, sizeof(req),
			&rsp_desc, &rsp, sizeof(rsp),
			TBN_RELEASE_BUS_TIMEOUT_MS);
	mutex_unlock(&tbn->service_lock);
	if (err) {
		dev_err(tbn->dev, "qmi_send_req_wait() failed with: %d\n", err);
		return err;
	}

	return 0;
}

static void tbn_connect_to_remote_server(struct tbn_context *tbn)
{
	int err = 0;

	dev_dbg(tbn->dev, "connecting to remote tbn server on SLPI\n");

	tbn->qmi_handle = qmi_handle_create(tbn_process_event, tbn);
	if (!tbn->qmi_handle) {
		dev_err(tbn->dev, "failed to create qmi handle\n");
		return;
	}

	err = qmi_connect_to_service(
			tbn->qmi_handle,
			TBN_SERVICE_ID_V01,
			TBN_SERVICE_VERS_V01,
			0 /* service instance */);
	if (err) {
		dev_err(tbn->dev, "failed to connect to qmi service, "
				"err = %d\n", err);
		qmi_handle_destroy(tbn->qmi_handle);
		tbn->qmi_handle = NULL;
	}

	dev_dbg(tbn->dev, "connected to remote tbn server on SLPI\n");
}

static void tbn_on_server_arrive(struct work_struct *work)
{
	struct tbn_context *tbn =
		container_of(work, struct tbn_context, on_server_arrive);

	dev_info(tbn->dev, "remote tbn server online, connecting\n");

	mutex_lock(&tbn->service_lock);
	tbn_connect_to_remote_server(tbn);
	mutex_unlock(&tbn->service_lock);
}

static void tbn_on_server_exit(struct work_struct *work)
{
	struct tbn_context *tbn =
		container_of(work, struct tbn_context, on_server_exit);

	dev_info(tbn->dev, "remote tbn server offline, disconnecting\n");

	mutex_lock(&tbn->service_lock);
	qmi_handle_destroy(tbn->qmi_handle);
	tbn->qmi_handle = NULL;
	mutex_unlock(&tbn->service_lock);
}

struct tbn_context *tbn_init(struct device *dev)
{
	int err = 0;
	struct tbn_context *tbn = NULL;

	tbn = kzalloc(sizeof(struct tbn_context), GFP_KERNEL);
	if (!tbn) {
		dev_err(dev, "failed to allocate tbn context\n");
		goto fail_allocate_tbn_context;
	}

	tbn->dev = dev;
	tbn->server_event_notifier.notifier_call = tbn_on_server_event;
	mutex_init(&tbn->service_lock);
	INIT_WORK(&tbn->on_server_arrive, tbn_on_server_arrive);
	INIT_WORK(&tbn->on_server_exit, tbn_on_server_exit);
	INIT_WORK(&tbn->on_event, tbn_on_event);

	tbn->event_processor =
		create_singlethread_workqueue("tbn_event_processor");
	if (!tbn->event_processor) {
		dev_err(tbn->dev, "failed to create event processing thread\n");
		goto fail_create_workqueue;
	}

	err = qmi_svc_event_notifier_register(
			TBN_SERVICE_ID_V01,
			TBN_SERVICE_VERS_V01,
			0 /* service instance */,
			&tbn->server_event_notifier);
	if (err) {
		dev_err(tbn->dev, "failed to register server event notifier\n");
		goto fail_register_server_event_notifier;
	}

	dev_info(tbn->dev, "bus negotiator initialized: %p\n", tbn);

	return tbn;

fail_register_server_event_notifier:
	destroy_workqueue(tbn->event_processor);
fail_create_workqueue:
	kfree(tbn);
fail_allocate_tbn_context:
	return NULL;
}

void tbn_cleanup(struct tbn_context *tbn)
{
	dev_info(tbn->dev, "destructing bus negotiator: %p\n", tbn);

	if (!tbn)
		return;

	qmi_svc_event_notifier_unregister(
			TBN_SERVICE_ID_V01,
			TBN_SERVICE_VERS_V01,
			0 /* service instance */,
			&tbn->server_event_notifier);

	mutex_lock(&tbn->service_lock);

	if (tbn->qmi_handle)
		qmi_handle_destroy(tbn->qmi_handle);

	mutex_unlock(&tbn->service_lock);

	if (tbn->event_processor)
		destroy_workqueue(tbn->event_processor);

	kfree(tbn);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QMI based Touch Bus Negotiator");

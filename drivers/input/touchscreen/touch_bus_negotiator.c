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
#include <linux/net.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include "touch_bus_negotiator.h"

static int send_request_wait(struct qmi_handle *handle,
			     int req_id, size_t req_len,
			     struct qmi_elem_info *req_ei, void *req,
			     struct qmi_elem_info *rsp_ei, void *rsp,
			     unsigned long timeout_ms)
{
	struct qmi_txn txn;
	int err;

	err = qmi_txn_init(handle, &txn, rsp_ei, rsp);
	if (err < 0)
		return err;

	err = qmi_send_request(handle, NULL, &txn,
			       req_id, req_len, req_ei, req);
	if (err < 0) {
		qmi_txn_cancel(&txn);
		return err;
	}

	err = qmi_txn_wait(&txn, timeout_ms);

	return err;
}

int tbn_request_bus(struct tbn_context *tbn)
{
	int err = 0;
	struct tbn_kernel_request_bus_v01 req;
	struct tbn_ssc_release_bus_v01 rsp;

	dev_info(tbn->dev, "kernel requesting bus access from SLPI\n");

	if (!tbn || !tbn->connected)
		return -EINVAL;

	mutex_lock(&tbn->service_lock);
	err = send_request_wait(&tbn->qmi_handle,
				QMI_TBN_KERNEL_REQUEST_BUS_V01,
				TBN_KERNEL_REQUEST_BUS_V01_MAX_MSG_LEN,
				tbn_kernel_request_bus_v01_ei, &req,
				tbn_ssc_release_bus_v01_ei, &rsp,
				TBN_REQUEST_BUS_TIMEOUT_MS);
	mutex_unlock(&tbn->service_lock);
	if (err < 0) {
		dev_err(tbn->dev, "send request failed with: %d\n", err);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tbn_request_bus);

int tbn_release_bus(struct tbn_context *tbn)
{
	int err = 0;
	struct tbn_kernel_release_bus_v01 req;
	struct tbn_ssc_acquire_bus_v01 rsp;

	dev_info(tbn->dev, "kernel releasing bus access from SLPI\n");

	if (!tbn || !tbn->connected)
		return -EINVAL;

	mutex_lock(&tbn->service_lock);
	err = send_request_wait(&tbn->qmi_handle,
				QMI_TBN_KERNEL_RELEASE_BUS_V01,
				TBN_KERNEL_RELEASE_BUS_V01_MAX_MSG_LEN,
				tbn_kernel_release_bus_v01_ei, &req,
				tbn_ssc_acquire_bus_v01_ei, &rsp,
				TBN_RELEASE_BUS_TIMEOUT_MS);
	mutex_unlock(&tbn->service_lock);
	if (err < 0) {
		dev_err(tbn->dev, "send request failed with: %d\n", err);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tbn_release_bus);

static int qmi_new_server(struct qmi_handle *qmi,
			  struct qmi_service *service)
{
	struct tbn_context *tbn =
		container_of(qmi, struct tbn_context, qmi_handle);
	struct sockaddr_qrtr sq = { AF_QIPCRTR, service->node, service->port };
	int err = 0;

	dev_info(tbn->dev, "remote tbn server online, connecting\n");

	mutex_lock(&tbn->service_lock);
	err = kernel_connect(qmi->sock, (struct sockaddr *)&sq, sizeof(sq), 0);
	if (err < 0)
		dev_err(tbn->dev, "failed to connect with %d\n", err);
	else
		tbn->connected = true;
	mutex_unlock(&tbn->service_lock);


	return err;
}

static void qmi_del_server(struct qmi_handle *qmi,
			   struct qmi_service *service)
{
	struct tbn_context *tbn =
		container_of(qmi, struct tbn_context, qmi_handle);

	dev_info(tbn->dev, "remote tbn server offline, disconnecting\n");

	mutex_lock(&tbn->service_lock);
	tbn->connected = false;
	mutex_unlock(&tbn->service_lock);
}

static struct qmi_ops qmi_ops = {
	.new_server = qmi_new_server,
	.del_server = qmi_del_server,
};

struct tbn_context *tbn_init(struct device *dev)
{
	int err = 0;
	struct tbn_context *tbn = NULL;

	tbn = kzalloc(sizeof(struct tbn_context), GFP_KERNEL);
	if (!tbn)
		goto fail_allocate_tbn_context;

	tbn->dev = dev;
	mutex_init(&tbn->service_lock);

	err = qmi_handle_init(&tbn->qmi_handle, 0, &qmi_ops, NULL);
	if (err < 0) {
		dev_err(tbn->dev, "failed to init qmi handle with %d\n", err);
		goto fail_register_server_event_notifier;
	}

	err = qmi_add_lookup(&tbn->qmi_handle,
			     TBN_SERVICE_ID_V01,
			     TBN_SERVICE_VERS_V01,
			     0);
	if (err < 0) {
		dev_err(tbn->dev, "failed to add server lookup with %d\n", err);
		goto fail_register_server_event_notifier;
	}

	dev_info(tbn->dev, "bus negotiator initialized: %p\n", tbn);

	return tbn;

fail_register_server_event_notifier:
	qmi_handle_release(&tbn->qmi_handle);
	kfree(tbn);
fail_allocate_tbn_context:
	return NULL;
}
EXPORT_SYMBOL_GPL(tbn_init);

void tbn_cleanup(struct tbn_context *tbn)
{
	dev_info(tbn->dev, "destructing bus negotiator: %p\n", tbn);

	if (!tbn)
		return;

	mutex_lock(&tbn->service_lock);

	qmi_handle_release(&tbn->qmi_handle);

	mutex_unlock(&tbn->service_lock);

	kfree(tbn);
}
EXPORT_SYMBOL_GPL(tbn_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QMI based Touch Bus Negotiator");

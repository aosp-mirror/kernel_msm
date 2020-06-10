// SPDX-License-Identifier: GPL-2.0
/* sound/soc/codecs/rt5514-qmi.c
 *
 * RT5514 qmi driver for Google Pixel devices.
 *
 * Copyright (C) 2020 Google, Inc.
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
#include "rt5514-qmi.h"

#define RT5514_QMI_REGISTER_NOTIFICATION_TIMEOUT_MS 500

static int rt5514_qmi_send_request_wait(struct qmi_handle *handle,
					int req_id, size_t req_len,
					struct qmi_elem_info *req_ei, void *req,
					struct qmi_elem_info *rsp_ei, void *rsp,
					unsigned long timeout_ms)
{
	struct qmi_txn txn;
	int err;

	err = qmi_txn_init(handle, &txn, rsp_ei, rsp);
	if (err < 0) {
		pr_err("%s: qmi_txn_init failed: %d\n", __func__, err);
		return err;
	}

	err = qmi_send_request(handle, NULL, &txn,
				req_id, req_len, req_ei, req);
	if (err < 0) {
		qmi_txn_cancel(&txn);
		pr_err("%s: qmi_send_request failed: %d\n", __func__, err);
		return err;
	}

	err = qmi_txn_wait(&txn, msecs_to_jiffies(timeout_ms));
	if (err < 0) {
		pr_err("%s: qmi_txn_wait failed: %d\n", __func__, err);
	}

	return err;
}

static void rt5514_qmi_svc_arrive(struct work_struct *work)
{
	int err = 0;
	struct rtk_spi_register_notification_req_v01 req;
	struct rtk_spi_register_notification_resp_v01 rsp;
	struct rt5514_qmi_instance *instance = container_of(work,
						struct rt5514_qmi_instance,
						svc_arrive_work);
	mutex_lock(&instance->service_lock);
	err = rt5514_qmi_send_request_wait(&instance->qmi_handle,
			RTK_SPI_REGISTER_NOTIFICATION_REQ_V01,
			RTK_SPI_REGISTER_NOTIFICATION_REQ_V01_MAX_MSG_LEN,
			rtk_spi_register_notification_req_v01_ei, &req,
			rtk_spi_register_notification_resp_v01_ei, &rsp,
			RT5514_QMI_REGISTER_NOTIFICATION_TIMEOUT_MS);
	mutex_unlock(&instance->service_lock);
	if (err < 0) {
		dev_err(instance->dev, "send request failed with: %d\n", err);
	} else if (rsp.resp.result != QMI_RESULT_SUCCESS_V01) {
		dev_err(instance->dev, "qmi result failed: %d\n",
					rsp.resp.result);
	} else {
		dev_info(instance->dev, "Send request success\n");
	}
}

void rt5514_qmi_ind_cb(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
				struct qmi_txn *txn, const void *decoded)
{
	const struct rtk_spi_error_ind_v01 *ind_msg = decoded;
	struct rt5514_qmi_instance *instance = container_of(qmi,
						struct rt5514_qmi_instance,
						qmi_handle);

	dev_info(instance->dev, "%s: %d\n", __func__, ind_msg->error_code);
	if (!txn) {
		dev_err(instance->dev, "Invalid transaction\n");
		return;
	}

	if (instance->func_cb) {
		instance->func_cb(ind_msg->error_code);
	}
}

static struct qmi_msg_handler handlers[] = {
	{
		.type = QMI_INDICATION,
		.msg_id = RTK_SPI_ERROR_IND_V01,
		.ei = rtk_spi_error_ind_v01_ei,
		.decoded_size = sizeof(struct rtk_spi_error_ind_v01),
		.fn = rt5514_qmi_ind_cb
	},
	{}
};

static int qmi_new_server(struct qmi_handle *qmi,
			  struct qmi_service *service)
{
	struct rt5514_qmi_instance *instance =
		container_of(qmi, struct rt5514_qmi_instance, qmi_handle);
	struct sockaddr_qrtr sq = { AF_QIPCRTR, service->node, service->port };
	int err = 0;

	dev_info(instance->dev, "remote rtk_spi server online, connecting\n");

	mutex_lock(&instance->service_lock);
	err = kernel_connect(qmi->sock, (struct sockaddr *)&sq, sizeof(sq), 0);
	if (err < 0)
		dev_err(instance->dev, "failed to connect with %d\n", err);
	else
		instance->connected = true;
	mutex_unlock(&instance->service_lock);

	queue_work(instance->qmi_wq, &instance->svc_arrive_work);

	return err;
}

static void qmi_del_server(struct qmi_handle *qmi,
			   struct qmi_service *service)
{
	struct rt5514_qmi_instance *instance =
		container_of(qmi, struct rt5514_qmi_instance, qmi_handle);

	dev_info(instance->dev,
		 "remote rtk_spi server offline, disconnecting\n");

	mutex_lock(&instance->service_lock);
	instance->connected = false;
	mutex_unlock(&instance->service_lock);
}

static struct qmi_ops qmi_ops = {
	.new_server = qmi_new_server,
	.del_server = qmi_del_server,
};

struct rt5514_qmi_instance *rt5514_qmi_init(struct device *dev,
					    rt5514_qmi_callback cb)
{
	int err = 0;
	struct rt5514_qmi_instance *instance = NULL;

	instance = kzalloc(sizeof(struct rt5514_qmi_instance), GFP_KERNEL);
	if (!instance)
		goto fail_allocate_instance;

	instance->dev = dev;
	mutex_init(&instance->service_lock);

	err = qmi_handle_init(&instance->qmi_handle,
			      RTK_SPI_ERROR_IND_V01_MAX_MSG_LEN,
			      &qmi_ops, handlers);
	if (err < 0) {
		dev_err(instance->dev,
			"failed to init qmi handle with %d\n", err);
		goto fail_register_server_event_notifier;
	}

	err = qmi_add_lookup(&instance->qmi_handle,
			     RTK_SPI_SERVICE_ID_V01,
			     RTK_SPI_SERVICE_VERS_V01,
			     0);
	if (err < 0) {
		dev_err(instance->dev,
			"failed to add server lookup with %d\n", err);
		goto fail_register_server_event_notifier;
	}

	instance->qmi_wq = alloc_workqueue("rt5514-qmi-wq", WQ_UNBOUND |
					 WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!instance->qmi_wq) {
		dev_err(instance->dev, "failed to alloc qmi_wq\n");
		goto fail_allocate_qmi_wq;
	}

	INIT_WORK(&instance->svc_arrive_work, rt5514_qmi_svc_arrive);

	instance->func_cb = cb;
	dev_info(instance->dev, "rt5514_qmi initialized\n");

	return instance;

fail_allocate_qmi_wq:
fail_register_server_event_notifier:
	qmi_handle_release(&instance->qmi_handle);
	kfree(instance);
fail_allocate_instance:
	return NULL;
}
EXPORT_SYMBOL_GPL(rt5514_qmi_init);

void rt5514_qmi_cleanup(struct rt5514_qmi_instance *instance)
{
	dev_info(instance->dev, "destructing rt5514-qmi\n");

	if (!instance)
		return;

	instance->func_cb = NULL;
	qmi_handle_release(&instance->qmi_handle);

	kfree(instance);
}
EXPORT_SYMBOL_GPL(rt5514_qmi_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RT5514 QMI driver");

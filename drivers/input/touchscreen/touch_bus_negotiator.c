// SPDX-License-Identifier: GPL-2.0
/* drivers/input/touchscreen/touch_bus_negotiator.c
 *
 * Touch Bus Negotiator for Google Pixel devices.
 *
 * Copyright (C) 2019 Google, Inc.
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
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/input/touch_bus_negotiator.h>

static irqreturn_t tbn_lpi2ap_irq_thread(int irq, void *ptr)
{
	struct tbn_context *tbn = (struct tbn_context *)ptr;

	if (completion_done(&tbn->bus_released) &&
		completion_done(&tbn->bus_requested)) {
		return IRQ_HANDLED;
	}

	/* For bus release, there two possibilities:
	 * 1. lpi2ap gpio value already changed to LPI
	 * 2. tbn_release_bus() with TBN_RELEASE_BUS_TIMEOUT_MS timeout
	 *    for complete_all(&tbn->bus_released);
	 */
	while (!completion_done(&tbn->bus_released)) {
		if (gpio_get_value(tbn->lpi2ap_gpio) == TBN_BUS_OWNER_LPI)
			complete_all(&tbn->bus_released);
		else {
			/* wait 10 ms for gpio stablized */
			msleep(10);
		}
	}

	/* For bus request, there two possibilities:
	 * 1. lpi2ap gpio value already changed to AP
	 * 2. tbn_request_bus() with TBN_REQUEST_BUS_TIMEOUT_MS timeout
	 *    for complete_all(&tbn->bus_requested);
	 */
	while (!completion_done(&tbn->bus_requested)) {
		if (gpio_get_value(tbn->lpi2ap_gpio) == TBN_BUS_OWNER_AP)
			complete_all(&tbn->bus_requested);
		else {
			/* wait 10 ms for gpio stablized */
			msleep(10);
		}
	}

	return IRQ_HANDLED;
}

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

	err = qmi_txn_wait(&txn, msecs_to_jiffies(timeout_ms));

	return err;
}

static void tbn_request_work(struct work_struct *work)
{
	int err = 0;
	struct tbn_kernel_request_bus_v01 req;
	struct tbn_ssc_release_bus_v01 rsp;
	struct tbn_context *tbn;

	tbn = container_of(work, struct tbn_context, request_work);

	if (!tbn || !tbn->connected)
		return;

	dev_info(tbn->dev, "kernel requesting bus access from SLPI\n");

	mutex_lock(&tbn->service_lock);
	err = send_request_wait(&tbn->qmi_handle,
				QMI_TBN_KERNEL_REQUEST_BUS_V01,
				TBN_KERNEL_REQUEST_BUS_V01_MAX_MSG_LEN,
				tbn_kernel_request_bus_v01_ei, &req,
				tbn_ssc_release_bus_v01_ei, &rsp,
				TBN_REQUEST_BUS_TIMEOUT_MS);
	mutex_unlock(&tbn->service_lock);
	complete_all(&tbn->bus_requested);
	if (err < 0)
		dev_err(tbn->dev, "send request failed with: %d\n", err);
	else
		dev_dbg(tbn->dev, "kernel requesting bus access from SLPI ... SUCCESS!\n");
}

int tbn_request_bus(struct tbn_context *tbn)
{
	if (!tbn || !tbn->connected)
		return 0;

	if (mutex_is_locked(&tbn->service_lock))
		dev_err(tbn->dev, "wait for a response on qmi transaction!\n");
	else {
		reinit_completion(&tbn->bus_requested);
		if (tbn->mode == TBN_MODE_QMI) {
			queue_work(tbn->qmi_wq, &tbn->request_work);
			if (wait_for_completion_timeout(&tbn->bus_requested,
			    msecs_to_jiffies(TBN_REQUEST_BUS_TIMEOUT_MS)) == 0)
				dev_err(tbn->dev,
					"%s: timeout!\n", __func__);
		} else {
			irq_set_irq_type(tbn->lpi2ap_irq, IRQ_TYPE_LEVEL_LOW);
			enable_irq(tbn->lpi2ap_irq);
			gpio_direction_output(tbn->ap2lpi_gpio,
						TBN_BUS_OWNER_AP);
			if (wait_for_completion_timeout(&tbn->bus_requested,
				msecs_to_jiffies(TBN_REQUEST_BUS_TIMEOUT_MS))
				== 0) {
				dev_err(tbn->dev,
					"%s: timeout!\n", __func__);
				complete_all(&tbn->bus_requested);
			} else
				dev_dbg(tbn->dev,
					"kernel requesting bus access from SLPI ... SUCCESS!\n");
			disable_irq_nosync(tbn->lpi2ap_irq);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tbn_request_bus);

static void tbn_release_work(struct work_struct *work)
{
	int err = 0;
	struct tbn_kernel_release_bus_v01 req;
	struct tbn_ssc_acquire_bus_v01 rsp;
	struct tbn_context *tbn;

	tbn = container_of(work, struct tbn_context, release_work);

	if (!tbn || !tbn->connected)
		return;

	dev_info(tbn->dev, "kernel releasing bus access from SLPI\n");

	mutex_lock(&tbn->service_lock);
	err = send_request_wait(&tbn->qmi_handle,
				QMI_TBN_KERNEL_RELEASE_BUS_V01,
				TBN_KERNEL_RELEASE_BUS_V01_MAX_MSG_LEN,
				tbn_kernel_release_bus_v01_ei, &req,
				tbn_ssc_acquire_bus_v01_ei, &rsp,
				TBN_RELEASE_BUS_TIMEOUT_MS);
	mutex_unlock(&tbn->service_lock);
	complete_all(&tbn->bus_released);
	if (err < 0)
		dev_err(tbn->dev, "send request failed with: %d\n", err);
	else
		dev_dbg(tbn->dev, "kernel releasing bus access from SLPI ... SUCCESS!\n");
}

int tbn_release_bus(struct tbn_context *tbn)
{
	if (!tbn || !tbn->connected)
		return 0;

	if (mutex_is_locked(&tbn->service_lock))
		dev_err(tbn->dev, "wait for a response on qmi transaction!\n");
	else {
		reinit_completion(&tbn->bus_released);
		if (tbn->mode == TBN_MODE_QMI) {
			queue_work(tbn->qmi_wq, &tbn->release_work);
			if (wait_for_completion_timeout(&tbn->bus_released,
			    msecs_to_jiffies(TBN_RELEASE_BUS_TIMEOUT_MS)) == 0)
				dev_err(tbn->dev,
					"%s: timeout!\n", __func__);
		} else {
			irq_set_irq_type(tbn->lpi2ap_irq, IRQ_TYPE_LEVEL_HIGH);
			enable_irq(tbn->lpi2ap_irq);
			gpio_direction_output(tbn->ap2lpi_gpio,
						TBN_BUS_OWNER_LPI);
			if (wait_for_completion_timeout(&tbn->bus_released,
				msecs_to_jiffies(TBN_RELEASE_BUS_TIMEOUT_MS))
				== 0) {
				dev_err(tbn->dev,
					"%s: timeout!\n", __func__);
				complete_all(&tbn->bus_released);
			} else
				dev_dbg(tbn->dev,
					"kernel releasing bus access from SLPI ... SUCCESS!\n");
			disable_irq_nosync(tbn->lpi2ap_irq);
		}
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
	struct device_node *np = dev->of_node;

	tbn = kzalloc(sizeof(struct tbn_context), GFP_KERNEL);
	if (!tbn)
		goto fail_allocate_tbn_context;

	tbn->dev = dev;
	mutex_init(&tbn->service_lock);

	if (of_property_read_bool(np, "tbn,ap2lpi_gpio") &&
		of_property_read_bool(np, "tbn,lpi2ap_gpio")) {
		tbn->mode = TBN_MODE_GPIO;
		tbn->ap2lpi_gpio = of_get_named_gpio(np, "tbn,ap2lpi_gpio", 0);
		tbn->lpi2ap_gpio = of_get_named_gpio(np, "tbn,lpi2ap_gpio", 0);

		if (gpio_is_valid(tbn->ap2lpi_gpio)) {
			err = devm_gpio_request_one(tbn->dev, tbn->ap2lpi_gpio,
				GPIOF_OUT_INIT_LOW, "tbn,ap2lpi_gpio");
			if (err) {
				dev_err(tbn->dev, "%s: Unable to request ap2lpi_gpio %d, err %d!\n",
					__func__, tbn->ap2lpi_gpio, err);
				goto fail_gpio;
			}
		}

		if (gpio_is_valid(tbn->lpi2ap_gpio)) {
			err = devm_gpio_request_one(tbn->dev, tbn->lpi2ap_gpio,
				GPIOF_DIR_IN, "tbn,lpi2ap_gpio");
			if (err) {
				dev_err(tbn->dev, "%s: Unable to request lpi2ap_gpio %d, err %d!\n",
					__func__, tbn->lpi2ap_gpio, err);
				goto fail_gpio;
			}
			tbn->lpi2ap_irq = gpio_to_irq(tbn->lpi2ap_gpio);
			err = devm_request_threaded_irq(tbn->dev,
				tbn->lpi2ap_irq, NULL,
				tbn_lpi2ap_irq_thread,
				IRQF_TRIGGER_HIGH |
				IRQF_ONESHOT, "tbn", tbn);
			if (err) {
				dev_err(tbn->dev,
					"%s: Unable to request_threaded_irq, err %d!\n",
					__func__, err);
				goto fail_gpio;
			}
			disable_irq_nosync(tbn->lpi2ap_irq);
		} else {
			dev_err(tbn->dev, "%s: invalid lpi2ap_gpio %d!\n",
				__func__, tbn->lpi2ap_gpio);
			goto fail_gpio;
		}

		tbn->connected = true;
	} else {
		tbn->mode = TBN_MODE_QMI;

		err = qmi_handle_init(&tbn->qmi_handle, 0, &qmi_ops, NULL);
		if (err < 0) {
			dev_err(tbn->dev,
				"failed to init qmi handle with %d\n", err);
			goto fail_register_server_event_notifier;
		}

		err = qmi_add_lookup(&tbn->qmi_handle,
				     TBN_SERVICE_ID_V01,
				     TBN_SERVICE_VERS_V01,
				     0);
		if (err < 0) {
			dev_err(tbn->dev,
				"failed to add server lookup with %d\n", err);
			goto fail_register_server_event_notifier;
		}

		tbn->qmi_wq = alloc_workqueue("tbn-qmi-wq",
				WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
		if (!tbn->qmi_wq) {
			dev_err(tbn->dev, "failed to alloc qmi_wq\n");
			goto fail_allocate_qmi_wq;
		}

		INIT_WORK(&tbn->request_work, tbn_request_work);
		INIT_WORK(&tbn->release_work, tbn_release_work);
	}

	init_completion(&tbn->bus_requested);
	init_completion(&tbn->bus_released);
	complete_all(&tbn->bus_requested);
	complete_all(&tbn->bus_released);

	dev_info(tbn->dev,
		"%s: gpios(lpi2ap: %d ap2lpi: %d), mode %d\n",
		__func__, tbn->lpi2ap_gpio, tbn->ap2lpi_gpio, tbn->mode);

	dev_info(tbn->dev, "bus negotiator initialized: %p\n", tbn);

	return tbn;

fail_allocate_qmi_wq:
fail_register_server_event_notifier:
	qmi_handle_release(&tbn->qmi_handle);
fail_gpio:
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

	if (tbn->mode == TBN_MODE_QMI) {
		destroy_workqueue(tbn->qmi_wq);
		qmi_handle_release(&tbn->qmi_handle);
	}

	kfree(tbn);
}
EXPORT_SYMBOL_GPL(tbn_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QMI based Touch Bus Negotiator");

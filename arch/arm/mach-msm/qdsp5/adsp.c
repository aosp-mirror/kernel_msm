/* arch/arm/mach-msm/qdsp5/adsp.c
 *
 * Register/Interrupt access for userspace aDSP library.
 *
 * Copyright (c) 2008 QUALCOMM Incorporated
 * Copyright (C) 2008 Google, Inc.
 * Author: Iliyan Malchev <ibm@android.com>
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

/* TODO:
 * - move shareable rpc code outside of adsp.c
 * - general solution for virt->phys patchup
 * - queue IDs should be relative to modules
 * - disallow access to non-associated queues
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

static struct wake_lock adsp_wake_lock;
static inline void prevent_suspend(void)
{
	wake_lock(&adsp_wake_lock);
}
static inline void allow_suspend(void)
{
	wake_unlock(&adsp_wake_lock);
}

#include <linux/io.h>
#include <mach/msm_iomap.h>
#include "adsp.h"

#define INT_ADSP INT_ADSP_A9_A11

static struct adsp_info adsp_info;
static struct msm_rpc_endpoint *rpc_cb_server_client;
static struct msm_adsp_module *adsp_modules;
static int adsp_open_count;
static DEFINE_MUTEX(adsp_open_lock);

/* protect interactions with the ADSP command/message queue */
static spinlock_t adsp_cmd_lock;

static uint32_t current_image = -1;

void adsp_set_image(struct adsp_info *info, uint32_t image)
{
	current_image = image;
}

uint32_t adsp_get_module(struct adsp_info *info, uint32_t task)
{
	return info->task_to_module[current_image][task];
}

uint32_t adsp_get_queue_offset(struct adsp_info *info, uint32_t queue_id)
{
	BUG_ON(current_image == -1UL);
	return info->queue_offset[current_image][queue_id];
}

static int rpc_adsp_rtos_app_to_modem(uint32_t cmd, uint32_t module,
				      struct msm_adsp_module *adsp_module)
{
	int rc;
	struct rpc_adsp_rtos_app_to_modem_args_t rpc_req;
	struct rpc_reply_hdr *rpc_rsp;

	msm_rpc_setup_req(&rpc_req.hdr,
			  RPC_ADSP_RTOS_ATOM_PROG,
			  RPC_ADSP_RTOS_ATOM_VERS,
			  RPC_ADSP_RTOS_APP_TO_MODEM_PROC);

	rpc_req.gotit = cpu_to_be32(1);
	rpc_req.cmd = cpu_to_be32(cmd);
	rpc_req.proc_id = cpu_to_be32(RPC_ADSP_RTOS_PROC_APPS);
	rpc_req.module = cpu_to_be32(module);
	rc = msm_rpc_write(adsp_module->rpc_client, &rpc_req, sizeof(rpc_req));
	if (rc < 0) {
		pr_err("adsp: could not send RPC request: %d\n", rc);
		return rc;
	}

	rc = msm_rpc_read(adsp_module->rpc_client,
			  (void **)&rpc_rsp, -1, (5*HZ));
	if (rc < 0) {
		pr_err("adsp: error receiving RPC reply: %d (%d)\n",
		       rc, -ERESTARTSYS);
		return rc;
	}

	if (be32_to_cpu(rpc_rsp->reply_stat) != RPCMSG_REPLYSTAT_ACCEPTED) {
		pr_err("adsp: RPC call was denied!\n");
		kfree(rpc_rsp);
		return -EPERM;
	}

	if (be32_to_cpu(rpc_rsp->data.acc_hdr.accept_stat) !=
	    RPC_ACCEPTSTAT_SUCCESS) {
		pr_err("adsp error: RPC call was not successful (%d)\n",
		       be32_to_cpu(rpc_rsp->data.acc_hdr.accept_stat));
		kfree(rpc_rsp);
		return -EINVAL;
	}

	kfree(rpc_rsp);
	return 0;
}

struct msm_adsp_module *find_adsp_module_by_id(
	struct adsp_info *info, uint32_t id)
{
	if (id > info->max_module_id)
		return NULL;
	else
		return info->id_to_module[id];
}

static struct msm_adsp_module *find_adsp_module_by_name(
	struct adsp_info *info, const char *name)
{
	unsigned n;
	for (n = 0; n < info->module_count; n++)
		if (!strcmp(name, adsp_modules[n].name))
			return adsp_modules + n;
	return NULL;
}

static int adsp_rpc_init(struct msm_adsp_module *adsp_module)
{
	adsp_module->rpc_client = msm_rpc_connect(
		RPC_ADSP_RTOS_ATOM_PROG,
		RPC_ADSP_RTOS_ATOM_VERS,
		MSM_RPC_UNINTERRUPTIBLE);

	if (IS_ERR(adsp_module->rpc_client)) {
		int rc = PTR_ERR(adsp_module->rpc_client);
		adsp_module->rpc_client = 0;
		pr_err("adsp: could not open rpc client: %d\n", rc);
		return rc;
	}

	return 0;
}

int msm_adsp_get(const char *name, struct msm_adsp_module **out,
		 struct msm_adsp_ops *ops, void *driver_data)
{
	struct msm_adsp_module *module;
	int rc = 0;

	module = find_adsp_module_by_name(&adsp_info, name);
	if (!module)
		return -ENODEV;

	mutex_lock(&module->lock);
	pr_info("adsp: opening module %s\n", module->name);
	if (module->open_count++ == 0 && module->clk)
		clk_enable(module->clk);

	mutex_lock(&adsp_open_lock);
	if (adsp_open_count++ == 0) {
		enable_irq(INT_ADSP);
		prevent_suspend();
	}
	mutex_unlock(&adsp_open_lock);

	if (module->ops) {
		rc = -EBUSY;
		goto done;
	}

	rc = adsp_rpc_init(module);
	if (rc)
		goto done;

	module->ops = ops;
	module->driver_data = driver_data;
	*out = module;
	rc = rpc_adsp_rtos_app_to_modem(RPC_ADSP_RTOS_CMD_REGISTER_APP,
					module->id, module);
	if (rc) {
		module->ops = NULL;
		module->driver_data = NULL;
		*out = NULL;
		pr_err("adsp: REGISTER_APP failed\n");
		goto done;
	}

	pr_info("adsp: module %s has been registered\n", module->name);

done:
	mutex_lock(&adsp_open_lock);
	if (rc && --adsp_open_count == 0) {
		disable_irq(INT_ADSP);
		allow_suspend();
	}
	if (rc && --module->open_count == 0 && module->clk)
		clk_disable(module->clk);
	mutex_unlock(&adsp_open_lock);
	mutex_unlock(&module->lock);
	return rc;
}

static int msm_adsp_disable_locked(struct msm_adsp_module *module);

void msm_adsp_put(struct msm_adsp_module *module)
{
	unsigned long flags;

	mutex_lock(&module->lock);
	if (--module->open_count == 0 && module->clk)
		clk_disable(module->clk);
	if (module->ops) {
		pr_info("adsp: closing module %s\n", module->name);

		/* lock to ensure a dsp event cannot be delivered
		 * during or after removal of the ops and driver_data
		 */
		spin_lock_irqsave(&adsp_cmd_lock, flags);
		module->ops = NULL;
		module->driver_data = NULL;
		spin_unlock_irqrestore(&adsp_cmd_lock, flags);

		if (module->state != ADSP_STATE_DISABLED) {
			pr_info("adsp: disabling module %s\n", module->name);
			msm_adsp_disable_locked(module);
		}

		msm_rpc_close(module->rpc_client);
		module->rpc_client = 0;
		if (--adsp_open_count == 0) {
			disable_irq(INT_ADSP);
			allow_suspend();
			pr_info("adsp: disable interrupt\n");
		}
	} else {
		pr_info("adsp: module %s is already closed\n", module->name);
	}
	mutex_unlock(&module->lock);
}

/* this should be common code with rpc_servers.c */
static int rpc_send_accepted_void_reply(struct msm_rpc_endpoint *client,
					uint32_t xid, uint32_t accept_status)
{
	int rc = 0;
	uint8_t reply_buf[sizeof(struct rpc_reply_hdr)];
	struct rpc_reply_hdr *reply = (struct rpc_reply_hdr *)reply_buf;

	reply->xid = cpu_to_be32(xid);
	reply->type = cpu_to_be32(1); /* reply */
	reply->reply_stat = cpu_to_be32(RPCMSG_REPLYSTAT_ACCEPTED);

	reply->data.acc_hdr.accept_stat = cpu_to_be32(accept_status);
	reply->data.acc_hdr.verf_flavor = 0;
	reply->data.acc_hdr.verf_length = 0;

	rc = msm_rpc_write(rpc_cb_server_client, reply_buf, sizeof(reply_buf));
	if (rc < 0)
		pr_err("adsp: could not write RPC response: %d\n", rc);
	return rc;
}

int msm_adsp_write(struct msm_adsp_module *module, unsigned dsp_queue_addr,
		   void *cmd_buf, size_t cmd_size)
{
	uint32_t ctrl_word;
	uint32_t dsp_q_addr;
	uint32_t dsp_addr;
	uint32_t cmd_id = 0;
	int cnt = 0;
	int ret_status = 0;
	unsigned long flags;
	struct adsp_info *info = module->info;

	spin_lock_irqsave(&adsp_cmd_lock, flags);

	if (module->state != ADSP_STATE_ENABLED) {
		spin_unlock_irqrestore(&adsp_cmd_lock, flags);
		pr_err("adsp: module %s not enabled before write\n",
		       module->name);
		return -ENODEV;
	}
	dsp_q_addr = adsp_get_queue_offset(info, dsp_queue_addr);
	dsp_q_addr &= ADSP_RTOS_WRITE_CTRL_WORD_DSP_ADDR_M;

	/* Poll until the ADSP is ready to accept a command.
	 * Wait for 100us, return error if it's not responding.
	 * If this returns an error, we need to disable ALL modules and
	 * then retry.
	 */
	while (((ctrl_word = readl(info->write_ctrl)) &
		ADSP_RTOS_WRITE_CTRL_WORD_READY_M) !=
		ADSP_RTOS_WRITE_CTRL_WORD_READY_V) {
		if (cnt > 10) {
			pr_err("adsp: timeout waiting for DSP write ready\n");
			ret_status = -EIO;
			goto fail;
		}
		pr_warning("adsp: waiting for DSP write ready\n");
		udelay(10);
		cnt++;
	}

	/* Set the mutex bits */
	ctrl_word &= ~(ADSP_RTOS_WRITE_CTRL_WORD_MUTEX_M);
	ctrl_word |=  ADSP_RTOS_WRITE_CTRL_WORD_MUTEX_NAVAIL_V;

	/* Clear the command bits */
	ctrl_word &= ~(ADSP_RTOS_WRITE_CTRL_WORD_CMD_M);

	/* Set the queue address bits */
	ctrl_word &= ~(ADSP_RTOS_WRITE_CTRL_WORD_DSP_ADDR_M);
	ctrl_word |= dsp_q_addr;

	writel(ctrl_word, info->write_ctrl);

	/* Generate an interrupt to the DSP.  This notifies the DSP that
	 * we are about to send a command on this particular queue.  The
	 * DSP will in response change its state.
	 */
	writel(1, info->send_irq);

	/* Poll until the adsp responds to the interrupt; this does not
	 * generate an interrupt from the adsp.  This should happen within
	 * 5ms.
	 */
	cnt = 0;
	while ((readl(info->write_ctrl) &
		ADSP_RTOS_WRITE_CTRL_WORD_MUTEX_M) ==
		ADSP_RTOS_WRITE_CTRL_WORD_MUTEX_NAVAIL_V) {
		if (cnt > 5) {
			pr_err("adsp: timeout waiting for adsp ack\n");
			ret_status = -EIO;
			goto fail;
		}
		mdelay(1);
		cnt++;
	}

	/* Read the ctrl word */
	ctrl_word = readl(info->write_ctrl);

	if ((ctrl_word & ADSP_RTOS_WRITE_CTRL_WORD_STATUS_M) !=
	    ADSP_RTOS_WRITE_CTRL_WORD_NO_ERR_V) {
		ret_status = -EIO;
		pr_err("adsp: failed to write queue %x, retry\n", dsp_q_addr);
		goto fail;
	}

	/* Ctrl word status bits are 00, no error in the ctrl word */

	/* Get the DSP buffer address */
	dsp_addr = (ctrl_word & ADSP_RTOS_WRITE_CTRL_WORD_DSP_ADDR_M) +
		   (uint32_t)MSM_AD5_BASE;

	if (dsp_addr < (uint32_t)(MSM_AD5_BASE + QDSP_RAMC_OFFSET)) {
		uint16_t *buf_ptr = (uint16_t *) cmd_buf;
		uint16_t *dsp_addr16 = (uint16_t *)dsp_addr;
		cmd_size /= sizeof(uint16_t);

		/* Save the command ID */
		cmd_id = (uint32_t) buf_ptr[0];

		/* Copy the command to DSP memory */
		cmd_size++;
		while (--cmd_size)
			*dsp_addr16++ = *buf_ptr++;
	} else {
		uint32_t *buf_ptr = (uint32_t *) cmd_buf;
		uint32_t *dsp_addr32 = (uint32_t *)dsp_addr;
		cmd_size /= sizeof(uint32_t);

		/* Save the command ID */
		cmd_id = buf_ptr[0];

		cmd_size++;
		while (--cmd_size)
			*dsp_addr32++ = *buf_ptr++;
	}

	/* Set the mutex bits */
	ctrl_word &= ~(ADSP_RTOS_WRITE_CTRL_WORD_MUTEX_M);
	ctrl_word |=  ADSP_RTOS_WRITE_CTRL_WORD_MUTEX_NAVAIL_V;

	/* Set the command bits to write done */
	ctrl_word &= ~(ADSP_RTOS_WRITE_CTRL_WORD_CMD_M);
	ctrl_word |= ADSP_RTOS_WRITE_CTRL_WORD_CMD_WRITE_DONE_V;

	/* Set the queue address bits */
	ctrl_word &= ~(ADSP_RTOS_WRITE_CTRL_WORD_DSP_ADDR_M);
	ctrl_word |= dsp_q_addr;

	writel(ctrl_word, info->write_ctrl);

	/* Generate an interrupt to the DSP.  It does not respond with
	 * an interrupt, and we do not need to wait for it to
	 * acknowledge, because it will hold the mutex lock until it's
	 * ready to receive more commands again.
	 */
	writel(1, info->send_irq);

	module->num_commands++;

fail:
	spin_unlock_irqrestore(&adsp_cmd_lock, flags);
	return ret_status;
}

#ifdef CONFIG_MSM_ADSP_REPORT_EVENTS
static void *modem_event_addr;
static void read_modem_event(void *buf, size_t len)
{
	uint32_t *dptr = buf;
	struct rpc_adsp_rtos_modem_to_app_args_t *sptr =
		modem_event_addr;
	dptr[0] = be32_to_cpu(sptr->event);
	dptr[1] = be32_to_cpu(sptr->module);
	dptr[2] = be32_to_cpu(sptr->image);
}
#endif

static void handle_adsp_rtos_mtoa_app(struct rpc_request_hdr *req)
{
	struct rpc_adsp_rtos_modem_to_app_args_t *args =
		(struct rpc_adsp_rtos_modem_to_app_args_t *)req;
	uint32_t event = be32_to_cpu(args->event);
	uint32_t proc_id = be32_to_cpu(args->proc_id);
	uint32_t module_id = be32_to_cpu(args->module);
	uint32_t image = be32_to_cpu(args->image);
	struct msm_adsp_module *module;

	pr_info("adsp: rpc event=%d, proc_id=%d, module=%d, image=%d\n",
		event, proc_id, module_id, image);

	module = find_adsp_module_by_id(&adsp_info, module_id);
	if (!module) {
		pr_err("adsp: module %d is not supported!\n", module_id);
		rpc_send_accepted_void_reply(rpc_cb_server_client, req->xid,
				RPC_ACCEPTSTAT_GARBAGE_ARGS);
		return;
	}

	mutex_lock(&module->lock);
	switch (event) {
	case RPC_ADSP_RTOS_MOD_READY:
		pr_info("adsp: module %s: READY\n", module->name);
		module->state = ADSP_STATE_ENABLED;
		wake_up(&module->state_wait);
		adsp_set_image(module->info, image);
		break;
	case RPC_ADSP_RTOS_MOD_DISABLE:
		pr_info("adsp: module %s: DISABLED\n", module->name);
		module->state = ADSP_STATE_DISABLED;
		wake_up(&module->state_wait);
		break;
	case RPC_ADSP_RTOS_SERVICE_RESET:
		pr_info("adsp: module %s: SERVICE_RESET\n", module->name);
		module->state = ADSP_STATE_DISABLED;
		wake_up(&module->state_wait);
		break;
	case RPC_ADSP_RTOS_CMD_SUCCESS:
		pr_info("adsp: module %s: CMD_SUCCESS\n", module->name);
		break;
	case RPC_ADSP_RTOS_CMD_FAIL:
		pr_info("adsp: module %s: CMD_FAIL\n", module->name);
		break;
	default:
		pr_info("adsp: unknown event %d\n", event);
		rpc_send_accepted_void_reply(rpc_cb_server_client, req->xid,
					     RPC_ACCEPTSTAT_GARBAGE_ARGS);
		goto done;
	}
	rpc_send_accepted_void_reply(rpc_cb_server_client, req->xid,
				     RPC_ACCEPTSTAT_SUCCESS);
done:
	mutex_unlock(&module->lock);
#ifdef CONFIG_MSM_ADSP_REPORT_EVENTS
	modem_event_addr = (uint32_t *)req;
	module->ops->event(module->driver_data,
				EVENT_MSG_ID,
				EVENT_LEN,
				read_modem_event);
#endif
}

static int handle_adsp_rtos_mtoa(struct rpc_request_hdr *req)
{
	switch (req->procedure) {
	case RPC_ADSP_RTOS_MTOA_NULL_PROC:
		rpc_send_accepted_void_reply(rpc_cb_server_client,
					     req->xid,
					     RPC_ACCEPTSTAT_SUCCESS);
		break;
	case RPC_ADSP_RTOS_MODEM_TO_APP_PROC:
		handle_adsp_rtos_mtoa_app(req);
		break;
	default:
		pr_err("adsp: unknowned proc %d\n", req->procedure);
		rpc_send_accepted_void_reply(
			rpc_cb_server_client, req->xid,
			RPC_ACCEPTSTAT_PROC_UNAVAIL);
		break;
	}
	return 0;
}

/* this should be common code with rpc_servers.c */
static int adsp_rpc_thread(void *data)
{
	void *buffer;
	struct rpc_request_hdr *req;
	int rc;

	do {
		rc = msm_rpc_read(rpc_cb_server_client, &buffer, -1, -1);
		if (rc < 0) {
			pr_err("adsp: could not read rpc: %d\n", rc);
			break;
		}
		req = (struct rpc_request_hdr *)buffer;

		req->type = be32_to_cpu(req->type);
		req->xid = be32_to_cpu(req->xid);
		req->rpc_vers = be32_to_cpu(req->rpc_vers);
		req->prog = be32_to_cpu(req->prog);
		req->vers = be32_to_cpu(req->vers);
		req->procedure = be32_to_cpu(req->procedure);

		if (req->type != 0)
			goto bad_rpc;
		if (req->rpc_vers != 2)
			goto bad_rpc;
		if (req->prog != RPC_ADSP_RTOS_MTOA_PROG)
			goto bad_rpc;
		if (req->vers != RPC_ADSP_RTOS_MTOA_VERS)
			goto bad_rpc;

		handle_adsp_rtos_mtoa(req);
		kfree(buffer);
		continue;

bad_rpc:
		pr_err("adsp: bogus rpc from modem\n");
		kfree(buffer);
	} while (1);
	do_exit(0);
}

static size_t read_event_size;
static void *read_event_addr;

static void read_event_16(void *buf, size_t len)
{
	uint16_t *dst = buf;
	uint16_t *src = read_event_addr;
	len /= 2;
	if (len > read_event_size)
		len = read_event_size;
	while (len--)
		*dst++ = *src++;
}

static void read_event_32(void *buf, size_t len)
{
	uint32_t *dst = buf;
	uint32_t *src = read_event_addr;
	len /= 2;
	if (len > read_event_size)
		len = read_event_size;
	while (len--)
		*dst++ = *src++;
}

static int adsp_rtos_read_ctrl_word_cmd_tast_to_h_v(
	struct adsp_info *info, void *dsp_addr)
{
	struct msm_adsp_module *module;
	unsigned rtos_task_id;
	unsigned msg_id;
	unsigned msg_length;
	void (*func)(void *, size_t);

	if (dsp_addr >= (void *)(MSM_AD5_BASE + QDSP_RAMC_OFFSET)) {
		uint32_t *dsp_addr32 = dsp_addr;
		uint32_t tmp = *dsp_addr32++;
		rtos_task_id = (tmp & ADSP_RTOS_READ_CTRL_WORD_TASK_ID_M) >> 8;
		msg_id = (tmp & ADSP_RTOS_READ_CTRL_WORD_MSG_ID_M);
		read_event_size = tmp >> 16;
		read_event_addr = dsp_addr32;
		msg_length = read_event_size * sizeof(uint32_t);
		func = read_event_32;
	} else {
		uint16_t *dsp_addr16 = dsp_addr;
		uint16_t tmp = *dsp_addr16++;
		rtos_task_id = (tmp & ADSP_RTOS_READ_CTRL_WORD_TASK_ID_M) >> 8;
		msg_id = tmp & ADSP_RTOS_READ_CTRL_WORD_MSG_ID_M;
		read_event_size = *dsp_addr16++;
		read_event_addr = dsp_addr16;
		msg_length = read_event_size * sizeof(uint16_t);
		func = read_event_16;
	}

	if (rtos_task_id > info->max_task_id) {
		pr_err("adsp: bogus task id %d\n", rtos_task_id);
		return 0;
	}
	module = find_adsp_module_by_id(info,
					adsp_get_module(info, rtos_task_id));

	if (!module) {
		pr_err("adsp: no module for task id %d\n", rtos_task_id);
		return 0;
	}

	module->num_events++;

	if (!module->ops) {
		pr_err("adsp: module %s is not open\n", module->name);
		return 0;
	}

	module->ops->event(module->driver_data, msg_id, msg_length, func);
	return 0;
}

static int adsp_get_event(struct adsp_info *info)
{
	uint32_t ctrl_word;
	uint32_t ready;
	void *dsp_addr;
	uint32_t cmd_type;
	int cnt;
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&adsp_cmd_lock, flags);

	/* Whenever the DSP has a message, it updates this control word
	 * and generates an interrupt.  When we receive the interrupt, we
	 * read this register to find out what ADSP task the command is
	 * comming from.
	 *
	 * The ADSP should *always* be ready on the first call, but the
	 * irq handler calls us in a loop (to handle back-to-back command
	 * processing), so we give the DSP some time to return to the
	 * ready state.  The DSP will not issue another IRQ for events
	 * pending between the first IRQ and the event queue being drained,
	 * unfortunately.
	 */

	for (cnt = 0; cnt < 10; cnt++) {
		ctrl_word = readl(info->read_ctrl);

		if ((ctrl_word & ADSP_RTOS_READ_CTRL_WORD_FLAG_M) ==
		    ADSP_RTOS_READ_CTRL_WORD_FLAG_UP_CONT_V)
			goto ready;

		udelay(10);
	}
	pr_warning("adsp: not ready after 100uS\n");
	rc = -EBUSY;
	goto done;

ready:
	/* Here we check to see if there are pending messages. If there are
	 * none, we siply return -EAGAIN to indicate that there are no more
	 * messages pending.
	 */
	ready = ctrl_word & ADSP_RTOS_READ_CTRL_WORD_READY_M;
	if ((ready != ADSP_RTOS_READ_CTRL_WORD_READY_V) &&
	    (ready != ADSP_RTOS_READ_CTRL_WORD_CONT_V)) {
		rc = -EAGAIN;
		goto done;
	}

	/* DSP says that there are messages waiting for the host to read */

	/* Get the Command Type */
	cmd_type = ctrl_word & ADSP_RTOS_READ_CTRL_WORD_CMD_TYPE_M;

	/* Get the DSP buffer address */
	dsp_addr = (void *)((ctrl_word &
			     ADSP_RTOS_READ_CTRL_WORD_DSP_ADDR_M) +
			    (uint32_t)MSM_AD5_BASE);

	/* We can only handle Task-to-Host messages */
	if (cmd_type != ADSP_RTOS_READ_CTRL_WORD_CMD_TASK_TO_H_V) {
		pr_err("adsp: unknown dsp cmd_type %d\n", cmd_type);
		rc = -EIO;
		goto done;
	}

	adsp_rtos_read_ctrl_word_cmd_tast_to_h_v(info, dsp_addr);

	ctrl_word = readl(info->read_ctrl);
	ctrl_word &= ~ADSP_RTOS_READ_CTRL_WORD_READY_M;

	/* Write ctrl word to the DSP */
	writel(ctrl_word, info->read_ctrl);

	/* Generate an interrupt to the DSP */
	writel(1, info->send_irq);

done:
	spin_unlock_irqrestore(&adsp_cmd_lock, flags);
	return rc;
}

static irqreturn_t adsp_irq_handler(int irq, void *data)
{
	struct adsp_info *info = &adsp_info;
	int cnt = 0;
	for (cnt = 0; cnt < 10; cnt++)
		if (adsp_get_event(info) < 0)
			break;
	if (cnt > info->event_backlog_max)
		info->event_backlog_max = cnt;
	info->events_received += cnt;
	if (cnt == 10)
		pr_err("adsp: too many (%d) events for single irq!\n", cnt);
	return IRQ_HANDLED;
}

int msm_adsp_enable(struct msm_adsp_module *module)
{
	int rc = 0;

	pr_info("msm_adsp_enable() '%s'\n", module->name);

	mutex_lock(&module->lock);
	switch (module->state) {
	case ADSP_STATE_DISABLED:
		rc = rpc_adsp_rtos_app_to_modem(RPC_ADSP_RTOS_CMD_ENABLE,
						module->id, module);
		if (rc)
			break;
		module->state = ADSP_STATE_ENABLING;
		mutex_unlock(&module->lock);
		rc = wait_event_timeout(module->state_wait,
					module->state != ADSP_STATE_ENABLING,
					1 * HZ);
		mutex_lock(&module->lock);
		if (module->state == ADSP_STATE_ENABLED) {
			rc = 0;
		} else {
			pr_err("adsp: module '%s' enable timed out\n",
			       module->name);
			rc = -ETIMEDOUT;
		}
		break;
	case ADSP_STATE_ENABLING:
		pr_warning("adsp: module '%s' enable in progress\n",
			   module->name);
		break;
	case ADSP_STATE_ENABLED:
		pr_warning("adsp: module '%s' already enabled\n",
			   module->name);
		break;
	case ADSP_STATE_DISABLING:
		pr_err("adsp: module '%s' disable in progress\n",
		       module->name);
		rc = -EBUSY;
		break;
	}
	mutex_unlock(&module->lock);
	return rc;
}

static int msm_adsp_disable_locked(struct msm_adsp_module *module)
{
	int rc = 0;

	switch (module->state) {
	case ADSP_STATE_DISABLED:
		pr_warning("adsp: module '%s' already disabled\n",
			   module->name);
		break;
	case ADSP_STATE_ENABLING:
	case ADSP_STATE_ENABLED:
		rc = rpc_adsp_rtos_app_to_modem(RPC_ADSP_RTOS_CMD_DISABLE,
						module->id, module);
		module->state = ADSP_STATE_DISABLED;
	}
	return rc;
}

int msm_adsp_disable(struct msm_adsp_module *module)
{
	int rc;
	pr_info("msm_adsp_disable() '%s'\n", module->name);
	mutex_lock(&module->lock);
	rc = msm_adsp_disable_locked(module);
	mutex_unlock(&module->lock);
	return rc;
}

static int msm_adsp_probe(struct platform_device *pdev)
{
	unsigned count;
	int rc, i;

	wake_lock_init(&adsp_wake_lock, WAKE_LOCK_SUSPEND, "adsp");

	rc = adsp_init_info(&adsp_info);
	if (rc)
		return rc;
	adsp_info.send_irq += MSM_AD5_BASE;
	adsp_info.read_ctrl += MSM_AD5_BASE;
	adsp_info.write_ctrl += MSM_AD5_BASE;
	count = adsp_info.module_count;

	adsp_modules = kzalloc(
		(sizeof(struct msm_adsp_module) + sizeof(void *)) *
		count, GFP_KERNEL);
	if (!adsp_modules)
		return -ENOMEM;

	adsp_info.id_to_module = (void *) (adsp_modules + count);

	spin_lock_init(&adsp_cmd_lock);

	rc = request_irq(INT_ADSP, adsp_irq_handler, IRQF_TRIGGER_RISING,
			 "adsp", 0);
	if (rc < 0)
		goto fail_request_irq;
	disable_irq(INT_ADSP);

	rpc_cb_server_client = msm_rpc_open();
	if (IS_ERR(rpc_cb_server_client)) {
		rpc_cb_server_client = NULL;
		rc = PTR_ERR(rpc_cb_server_client);
		pr_err("adsp: could not create rpc server (%d)\n", rc);
		goto fail_rpc_open;
	}

	rc = msm_rpc_register_server(rpc_cb_server_client,
				     RPC_ADSP_RTOS_MTOA_PROG,
				     RPC_ADSP_RTOS_MTOA_VERS);
	if (rc) {
		pr_err("adsp: could not register callback server (%d)\n", rc);
		goto fail_rpc_register;
	}

	/* start the kernel thread to process the callbacks */
	kthread_run(adsp_rpc_thread, NULL, "kadspd");

	for (i = 0; i < count; i++) {
		struct msm_adsp_module *mod = adsp_modules + i;
		mutex_init(&mod->lock);
		init_waitqueue_head(&mod->state_wait);
		mod->info = &adsp_info;
		mod->name = adsp_info.module[i].name;
		mod->id = adsp_info.module[i].id;
		if (adsp_info.module[i].clk_name)
			mod->clk = clk_get(NULL, adsp_info.module[i].clk_name);
		else
			mod->clk = NULL;
		if (mod->clk && adsp_info.module[i].clk_rate)
			clk_set_rate(mod->clk, adsp_info.module[i].clk_rate);
		mod->verify_cmd = adsp_info.module[i].verify_cmd;
		mod->patch_event = adsp_info.module[i].patch_event;
		INIT_HLIST_HEAD(&mod->pmem_regions);
		mod->pdev.name = adsp_info.module[i].pdev_name;
		mod->pdev.id = -1;
		adsp_info.id_to_module[mod->id] = mod;
		platform_device_register(&mod->pdev);
	}

	msm_adsp_publish_cdevs(adsp_modules, count);

	return 0;

fail_rpc_register:
	msm_rpc_close(rpc_cb_server_client);
	rpc_cb_server_client = NULL;
fail_rpc_open:
	enable_irq(INT_ADSP);
	free_irq(INT_ADSP, 0);
fail_request_irq:
	kfree(adsp_modules);
	return rc;
}

static struct platform_driver msm_adsp_driver = {
	.probe = msm_adsp_probe,
	.driver = {
#if CONFIG_MSM_AMSS_VERSION == 6210
		.name = "rs3000000a:20f17fd3",
#elif (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
		.name = "rs3000000a:71d1094b",
#else
#error "Unknown AMSS version"
#endif
		.owner = THIS_MODULE,
	},
};

static int __init adsp_init(void)
{
	return platform_driver_register(&msm_adsp_driver);
}

device_initcall(adsp_init);

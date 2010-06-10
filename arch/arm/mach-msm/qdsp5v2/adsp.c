/* arch/arm/mach-msm/qdsp5v2/adsp.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <mach/msm_iomap.h>

#include "../dal.h"

#include "adsp.h"
#include "adsp_private.h"

struct msm_adsp_queue {
	const char *name;
	uint32_t offset;
	uint32_t max_size;
	uint32_t flags;
};
	
struct msm_adsp_module {
	msm_adsp_callback func;
	void *cookie;
	
	wait_queue_head_t wait;
	struct msm_adsp *adsp;
	uint32_t id;

	unsigned active;

	const char *name;
	struct msm_adsp_module *next;
	struct msm_adsp_queue queue[ADSP_QUEUES_MAX];
};

struct msm_adsp {
	/* DSP "registers" */
	void *read_ctrl;
	void *write_ctrl;
	void *send_irq;
	void *base;

	/* DAL client handle for DSP control service */
	struct dal_client *client;

	spinlock_t callback_lock;
	spinlock_t write_lock;
	spinlock_t event_lock;

	wait_queue_head_t callback_wq;

	/* list of all existing dsp modules */
	struct msm_adsp_module *all_modules;

	/* map from dsp rtos task IDs to modules */
	struct msm_adsp_module *task_to_module[ADSP_TASKS_MAX];

	/* used during initialization */
	struct adsp_module_info tmpmodule;

};

static struct msm_adsp the_adsp;

static struct msm_adsp_module *id_to_module(struct msm_adsp *adsp, unsigned id)
{
	struct msm_adsp_module *module;

	for (module = adsp->all_modules; module; module = module->next)
		if (module->id == id)
			return module;
	return NULL;
}

int msm_adsp_get(const char *name, struct msm_adsp_module **module,
		 msm_adsp_callback func, void *cookie)
{
	struct msm_adsp *adsp = &the_adsp;
	unsigned long flags;
	int ret = -ENODEV;
	struct msm_adsp_module *m;

	for (m = adsp->all_modules; m; m = m->next) {
		if (!strcmp(m->name, name)) {
			spin_lock_irqsave(&m->adsp->callback_lock, flags);
			if (m->func == 0) {
				m->func = func;
				m->cookie = cookie;
				*module = m;
				ret = 0;
			} else {
				ret = -EBUSY;
			}
			spin_unlock_irqrestore(&m->adsp->callback_lock, flags);
			break;
		}
	}
	return ret;
}

void msm_adsp_put(struct msm_adsp_module *m)
{
	unsigned long flags;

	spin_lock_irqsave(&m->adsp->callback_lock, flags);
	m->func = 0;
	m->cookie = 0;
	spin_unlock_irqrestore(&m->adsp->callback_lock, flags);
}


int msm_adsp_lookup_queue(struct msm_adsp_module *module, const char *name)
{
	int n;
	for (n = 0; n < ADSP_QUEUES_MAX; n++) {
		if (!module->queue[n].name)
			break;
		if (!strcmp(name, module->queue[n].name))
			return n;
	}
	return -ENODEV;
}

static int msm_adsp_command(struct msm_adsp_module *module, unsigned cmd_id)
{
	struct adsp_dal_cmd cmd;
	int ret;

	cmd.cmd = cmd_id;
	cmd.proc_id = ADSP_PROC_APPS;
	cmd.module = module->id;
	cmd.cookie = 0;

	ret = dal_call_f5(module->adsp->client, ADSP_DAL_COMMAND,
			  &cmd, sizeof(cmd));
	if (ret)
		return -EIO;

	return 0;
}

int msm_adsp_enable(struct msm_adsp_module *module)
{
	int ret;
	/* XXX interlock? */

	ret = msm_adsp_command(module, ADSP_CMD_ENABLE);
	if (ret < 0) {
		pr_err("msm_adsp_enable: error enabling %s %d\n",
		       module->name, ret);
		return -EIO;
	}
	ret = wait_event_timeout(module->adsp->callback_wq,
				 module->active, 5 * HZ);
	if (!ret) {
		pr_err("msm_adsp_enable: timeout enabling %s\n",
		       module->name);
		return -ETIMEDOUT;
	}

	printk("msm_adsp_enable: %s enabled.\n", module->name);
	return 0;
}

int msm_adsp_disable(struct msm_adsp_module *module)
{
	/* XXX interlock? */
	return msm_adsp_command(module, ADSP_CMD_DISABLE);
}

int msm_adsp_write(struct msm_adsp_module *module, unsigned queue_idx,
		   void *cmd_buf, size_t cmd_size)
{
	struct msm_adsp *adsp;
	uint32_t val;
	uint32_t dsp_q_addr;
	uint32_t dsp_addr;
	uint32_t cmd_id = 0;
	int cnt = 0;
	int ret = 0;
	unsigned long flags;

	if (!module || !cmd_size || (queue_idx >= ADSP_QUEUES_MAX))
		return -EINVAL;

	if (module->queue[queue_idx].name == NULL)
		return -EINVAL;

	adsp = module->adsp;

	spin_lock_irqsave(&adsp->write_lock, flags);

#if 0
	if (module->state != ADSP_STATE_ENABLED) {
		ret = -ENODEV;
		goto done;
	}
#endif

	dsp_q_addr = module->queue[queue_idx].offset;
	dsp_q_addr &= ADSP_WRITE_CTRL_DSP_ADDR_M;

	/* Poll until the ADSP is ready to accept a command.
	 * Wait for 100us, return error if it's not responding.
	 * If this returns an error, we need to disable ALL modules and
	 * then retry.
	 */
	while (((val = readl(adsp->write_ctrl)) &
		ADSP_WRITE_CTRL_READY_M) !=
		ADSP_WRITE_CTRL_READY_V) {
		if (cnt > 50) {
			pr_err("timeout waiting for DSP write ready\n");
			ret = -EIO;
			goto done;
		}
		udelay(2);
		cnt++;
	}

	/* Set the mutex bits */
	val &= ~(ADSP_WRITE_CTRL_MUTEX_M);
	val |=  ADSP_WRITE_CTRL_MUTEX_NAVAIL_V;

	/* Clear the command bits */
	val &= ~(ADSP_WRITE_CTRL_CMD_M);

	/* Set the queue address bits */
	val &= ~(ADSP_WRITE_CTRL_DSP_ADDR_M);
	val |= dsp_q_addr;

	writel(val, adsp->write_ctrl);

	/* Generate an interrupt to the DSP.  This notifies the DSP that
	 * we are about to send a command on this particular queue.  The
	 * DSP will in response change its state.
	 */
	writel(1, adsp->send_irq);

	/* Poll until the adsp responds to the interrupt; this does not
	 * generate an interrupt from the adsp.  This should happen within
	 * 5ms.
	 */
	cnt = 0;
	while ((readl(adsp->write_ctrl) &
		ADSP_WRITE_CTRL_MUTEX_M) ==
		ADSP_WRITE_CTRL_MUTEX_NAVAIL_V) {
		if (cnt > 2500) {
			pr_err("timeout waiting for adsp ack\n");
			ret = -EIO;
			goto done;
		}
		udelay(2);
		cnt++;
	}

	/* Read the ctrl word */
	val = readl(adsp->write_ctrl);

	if ((val & ADSP_WRITE_CTRL_STATUS_M) !=
	    ADSP_WRITE_CTRL_NO_ERR_V) {
		ret = -EIO;
		pr_err("failed to write queue %x, retry\n", dsp_q_addr);
		goto done;
	}

	/* No error */
	/* Get the DSP buffer address */
	dsp_addr = (val & ADSP_WRITE_CTRL_DSP_ADDR_M) +
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
	val &= ~(ADSP_WRITE_CTRL_MUTEX_M);
	val |=  ADSP_WRITE_CTRL_MUTEX_NAVAIL_V;
	
	/* Set the command bits to write done */
	val &= ~(ADSP_WRITE_CTRL_CMD_M);
	val |= ADSP_WRITE_CTRL_CMD_WRITE_DONE_V;
	
	/* Set the queue address bits */
	val &= ~(ADSP_WRITE_CTRL_DSP_ADDR_M);
	val |= dsp_q_addr;
	
	writel(val, adsp->write_ctrl);
	
	/* Generate an interrupt to the DSP.  It does not respond with
	 * an interrupt, and we do not need to wait for it to
	 * acknowledge, because it will hold the mutex lock until it's
	 * ready to receive more commands again.
	 */
	writel(1, adsp->send_irq);
	
//	module->num_commands++;

done:
	spin_unlock_irqrestore(&adsp->write_lock, flags);
	return ret;
}

static int adsp_read_task_to_host(struct msm_adsp *adsp, void *dsp_addr)
{
	struct msm_adsp_module *module;
	unsigned task_id;
	unsigned msg_id;
	unsigned msg_length;
	unsigned n;
	unsigned tmp;
	union {
		u32 data32[16];
		u16 data16[32];
	} u;

	if (dsp_addr >= (void *)(MSM_AD5_BASE + QDSP_RAMC_OFFSET)) {
		uint32_t *dsp_addr32 = dsp_addr;
		tmp = *dsp_addr32++;
		task_id = (tmp & ADSP_READ_CTRL_TASK_ID_M) >> 8;
		msg_id = (tmp & ADSP_READ_CTRL_MSG_ID_M);
		tmp >>= 16;
		if (tmp > 16) {
			pr_err("adsp: message too large (%d x 32)\n", tmp);
			tmp = 16;
		}
		msg_length = tmp * sizeof(uint32_t);
		for (n = 0; n < tmp; n++)
			u.data32[n] = *dsp_addr32++;
	} else {
		uint16_t *dsp_addr16 = dsp_addr;
		tmp = *dsp_addr16++;
		task_id = (tmp & ADSP_READ_CTRL_TASK_ID_M) >> 8;
		msg_id = tmp & ADSP_READ_CTRL_MSG_ID_M;
		tmp = *dsp_addr16++;
		if (tmp > 32) {
			pr_err("adsp: message too large (%d x 16)\n", tmp);
			tmp = 32;
		}
		msg_length = tmp * sizeof(uint16_t);
		for (n = 0; n < tmp; n++)
			u.data16[n] = *dsp_addr16++;
	}

#if 0
	pr_info("ADSP EVENT TASK %d MSG %d SIZE %d\n",
		task_id, msg_id, msg_length);
#endif
	if (task_id > ADSP_TASKS_MAX) {
		pr_err("adsp: bogus task id %d\n", task_id);
		return 0;
	}
	module = adsp->task_to_module[task_id];

	if (!module) {
		pr_err("adsp: no module for task id %d\n", task_id);
		return 0;
	}

	if (!module->func) {
		pr_err("module %s is not open\n", module->name);
		return 0;
	}

	module->func(msg_id, u.data32, msg_length, module->cookie);
	return 0;
}

static int adsp_get_event(struct msm_adsp *adsp)
{
	uint32_t val;
	uint32_t ready;
	void *dsp_addr;
	uint32_t cmd_type;
	int cnt;
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&adsp->event_lock, flags);

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

	for (cnt = 0; cnt < 50; cnt++) {
		val = readl(adsp->read_ctrl);

		if ((val & ADSP_READ_CTRL_FLAG_M) ==
		    ADSP_READ_CTRL_FLAG_UP_CONT_V)
			goto ready;

		udelay(2);
	}
	pr_err("adsp_get_event: not ready after 100uS\n");
	rc = -EBUSY;
	goto done;

ready:
	/* Here we check to see if there are pending messages. If there are
	 * none, we siply return -EAGAIN to indicate that there are no more
	 * messages pending.
	 */
	ready = val & ADSP_READ_CTRL_READY_M;
	if ((ready != ADSP_READ_CTRL_READY_V) &&
	    (ready != ADSP_READ_CTRL_CONT_V)) {
		rc = -EAGAIN;
		goto done;
	}

	/* DSP says that there are messages waiting for the host to read */

	/* Get the Command Type */
	cmd_type = val & ADSP_READ_CTRL_CMD_TYPE_M;

	/* Get the DSP buffer address */
	dsp_addr = (void *)((val &
			     ADSP_READ_CTRL_DSP_ADDR_M) +
			    (uint32_t)MSM_AD5_BASE);

	/* We can only handle Task-to-Host messages */
	if (cmd_type != ADSP_READ_CTRL_CMD_TASK_TO_H_V) {
		rc = -EIO;
		goto done;
	}

	adsp_read_task_to_host(adsp, dsp_addr);

	val = readl(adsp->read_ctrl);
	val &= ~ADSP_READ_CTRL_READY_M;

	/* Write ctrl word to the DSP */
	writel(val, adsp->read_ctrl);

	/* Generate an interrupt to the DSP */
	writel(1, adsp->send_irq);

done:
	spin_unlock_irqrestore(&adsp->event_lock, flags);
	return rc;
}

static irqreturn_t adsp_irq_handler(int irq, void *data)
{
	struct msm_adsp *adsp = &the_adsp;
	int count = 0;
	for (count = 0; count < 15; count++)
		if (adsp_get_event(adsp) < 0)
			break;
#if 0
	if (count > adsp->event_backlog_max)
		adsp->event_backlog_max = count;
	adsp->events_received += count;
#endif
	if (count == 15)
		pr_err("too many (%d) events for single irq!\n", count);
	return IRQ_HANDLED;
}

static void adsp_dal_callback(void *data, int len, void *cookie)
{
	struct msm_adsp *adsp = cookie;
	struct adsp_dal_event *e = data;
	struct msm_adsp_module *m;
#if 0
	pr_info("adsp: h %08x c %08x l %08x\n",
		e->evt_handle, e->evt_cookie, e->evt_length);
	pr_info("    : e %08x v %08x p %08x\n",
		e->event, e->version, e->proc_id);
	pr_info("    : m %08x i %08x a %08x\n",
		e->u.info.module, e->u.info.image, e->u.info.apps_okts);
#endif

	switch (e->event) {
	case ADSP_EVT_INIT_INFO:
		memcpy(&adsp->tmpmodule, &e->u.module,
		       sizeof(adsp->tmpmodule));
		break;
	case ADSP_EVT_MOD_READY:
		m = id_to_module(adsp, e->u.info.module);
		if (m) {
			pr_info("adsp: %s READY\n", m->name);
			m->active = 1;
		}
		break;
	case ADSP_EVT_MOD_DISABLE:
		/* does not actually happen in adsp5v2 */
		m = id_to_module(adsp, e->u.info.module);
		if (m)
			pr_info("adsp: %s DISABLED\n", m->name);
		break;
	case ADSP_EVT_DISABLE_FAIL:
		m = id_to_module(adsp, e->u.info.module);
		if (m)
			pr_info("adsp: %s DISABLE FAILED\n", m->name);
		break;
	default:
		pr_err("adsp_dal_callback: unknown event %d\n", e->event);
	}
	wake_up(&adsp->callback_wq);
}

static void adsp_add_module(struct msm_adsp *adsp, struct adsp_module_info *mi)
{
	struct msm_adsp_module *module;
	int n;

	if (mi->task_id >= ADSP_TASKS_MAX) {
		pr_err("adsp: module '%s' task id %d is invalid\n",
		       mi->name, mi->task_id);
		return;
	}
	if (mi->q_cnt > ADSP_QUEUES_MAX) {
		pr_err("adsp: module '%s' q_cnt %d is invalid\n",
		       mi->name, mi->q_cnt);
		return;
	}

	module = kzalloc(sizeof(*module), GFP_KERNEL);
	if (!module)
		return;

	module->name = kstrdup(mi->name, GFP_KERNEL);
	if (!module->name)
		goto fail_module_name;

	for (n = 0; n < mi->q_cnt; n++) {
		struct msm_adsp_queue *queue = module->queue + n;
		queue->name = kstrdup(mi->queue[n].name, GFP_KERNEL);
		if (!queue->name)
			goto fail_queue_name;
		queue->offset = mi->queue[n].offset;
		queue->max_size = mi->queue[n].max_size;
		queue->flags = mi->queue[n].flag;
	}

	init_waitqueue_head(&module->wait);
	module->id = mi->uuid;
	module->adsp = adsp;

	module->next = adsp->all_modules;
	adsp->all_modules = module;

	adsp->task_to_module[mi->task_id] = module;
#if 0
	pr_info("adsp: module '%s' id 0x%x task %d\n",
		module->name, module->id, mi->task_id);
	for (n = 0; (n < ADSP_TASKS_MAX) && module->queue[n].name; n++)
		pr_info("       queue '%s' off 0x%x size %d flags %x",
			module->queue[n].name, module->queue[n].offset,
			module->queue[n].max_size, module->queue[n].flags);
#endif
	return;

fail_queue_name:
	for (n = 0; n < mi->q_cnt; n++)
		if (module->queue[n].name)
			kfree(module->queue[n].name);
fail_module_name:
	kfree(module);
}

static int adsp_probe(struct platform_device *pdev) {
	struct msm_adsp *adsp = &the_adsp;
	struct adsp_dal_cmd cmd;
	int ret, n;

	pr_info("*** adsp_probe() ***\n");

	adsp->base = MSM_AD5_BASE;
	adsp->read_ctrl = adsp->base + ADSP_READ_CTRL_OFFSET;
	adsp->write_ctrl = adsp->base + ADSP_WRITE_CTRL_OFFSET;
	adsp->send_irq = adsp->base + ADSP_SEND_IRQ_OFFSET;

	adsp->client = dal_attach(ADSP_DAL_DEVICE, ADSP_DAL_PORT,
				  adsp_dal_callback, adsp);
	if (!adsp->client) {
		pr_err("adsp_probe: cannot attach to dal device\n");
		return -ENODEV;
	}

	cmd.cmd = ADSP_CMD_GET_INIT_INFO;
	cmd.proc_id = ADSP_PROC_APPS;
	cmd.module = 0;
	cmd.cookie = 0;

	for (n = 0; n < 64; n++) {
		adsp->tmpmodule.uuid = 0xffffffff;
		ret = dal_call_f5(adsp->client, ADSP_DAL_COMMAND,
				  &cmd, sizeof(cmd));
		if (ret) {
			pr_err("adsp_probe() get info dal call failed\n");
			break;
		}
		ret = wait_event_timeout(adsp->callback_wq, 
					 (adsp->tmpmodule.uuid != 0xffffffff),
					 5*HZ);
		if (ret == 0) {
			pr_err("adsp_probe() timed out getting module info\n");
			break;
		}
		if (adsp->tmpmodule.uuid == 0x7fffffff)
			break;
		if (adsp->tmpmodule.task_id == 0xffff)
			continue;
//		adsp_print_module(&adsp->tmpmodule);
		adsp_add_module(adsp, &adsp->tmpmodule);
	}

	ret = request_irq(INT_AD5A_MPROC_APPS_0, adsp_irq_handler,
			  IRQF_TRIGGER_RISING, "adsp", 0);
	if (ret < 0)
		return ret;

	pr_info("*** adsp_probe() done ***\n");
	return 0;
}

static struct platform_driver adsp_driver = {
	.probe	= adsp_probe,
	.driver	= {
		.name	= "SMD_DAL00",
		.owner	= THIS_MODULE,
	},
};

extern int msm_codec_init(void);

static int __init adsp_init(void)
{
	struct msm_adsp *adsp = &the_adsp;

	pr_info("*** adsp_init() ***\n");

	init_waitqueue_head(&adsp->callback_wq);
	spin_lock_init(&adsp->callback_lock);
	spin_lock_init(&adsp->write_lock);
	spin_lock_init(&adsp->event_lock);

	msm_codec_init();

	return platform_driver_register(&adsp_driver);
}

module_init(adsp_init);

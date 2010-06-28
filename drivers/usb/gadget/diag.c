/*
 * Diag Function Device - Route DIAG frames between SMD and USB
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/mutex.h>

#include <mach/msm_smd.h>

#include <linux/usb/android_composite.h>

#define NO_HDLC 1
#define ROUTE_TO_USERSPACE 1

#if 1
#define TRACE(tag,data,len,decode) do {} while(0)
#else
static void TRACE(const char *tag, const void *_data, int len, int decode)
{
	const unsigned char *data = _data;
	int escape = 0;

	printk(KERN_INFO "%s", tag);
	if (decode) {
		while (len-- > 0) {
			unsigned x = *data++;
			if (x == 0x7e) {
				printk(" $$");
				escape = 0;
				continue;
			}
			if (x == 0x7d) {
				escape = 1;
				continue;
			}
			if (escape) {
				escape = 0;
				printk(" %02x", x ^ 0x20);
			} else {
				printk(" %02x", x);
			}
		}
	} else {
		while (len-- > 0) {
			printk(" %02x", *data++);
		}
		printk(" $$");
	}
	printk("\n");
}
#endif

#define HDLC_MAX 4096

#define TX_REQ_BUF_SZ 8192
#define RX_REQ_BUF_SZ 8192

/* number of tx/rx requests to allocate */
#define TX_REQ_NUM 4
#define RX_REQ_NUM 4

struct diag_context
{
	struct usb_function function;
	struct usb_composite_dev *cdev;
	struct usb_ep *out;
	struct usb_ep *in;
	struct list_head tx_req_idle;
	struct list_head rx_req_idle;
	spinlock_t req_lock;
#if ROUTE_TO_USERSPACE
	struct mutex user_lock;
#define ID_TABLE_SZ 10 /* keep this small */
	struct list_head rx_req_user;
	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	char *user_read_buf;
	uint32_t user_read_len;
	char *user_readp;
	bool opened;

	/* list of registered command ids to be routed to userspace */
	unsigned char id_table[ID_TABLE_SZ];
#endif
	smd_channel_t *ch;
	int in_busy;
#ifdef CONFIG_ARCH_QSD8X50
	smd_channel_t *ch_dsp;
	int in_busy_dsp;
#endif
	int online;

	/* assembly buffer for USB->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;

	u64 tx_count; /* to smd */
	u64 rx_count; /* from smd */

	int function_enable;
};

static struct usb_interface_descriptor diag_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xFF,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor diag_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor diag_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor diag_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor diag_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_diag_descs[] = {
	(struct usb_descriptor_header *) &diag_interface_desc,
	(struct usb_descriptor_header *) &diag_fullspeed_in_desc,
	(struct usb_descriptor_header *) &diag_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_diag_descs[] = {
	(struct usb_descriptor_header *) &diag_interface_desc,
	(struct usb_descriptor_header *) &diag_highspeed_in_desc,
	(struct usb_descriptor_header *) &diag_highspeed_out_desc,
	NULL,
};

static struct diag_context _context;

static inline struct diag_context *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct diag_context, function);
}

static void smd_try_to_send(struct diag_context *ctxt);
#ifdef CONFIG_ARCH_QSD8X50
static void dsp_try_to_send(struct diag_context *ctxt);
#endif

static void diag_queue_out(struct diag_context *ctxt);

/* add a request to the tail of a list */
static void req_put(struct diag_context *ctxt, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->req_lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct diag_context *ctxt,
		struct list_head *head)
{
	struct usb_request *req = 0;
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	if (!list_empty(head)) {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return req;
}

static void reqs_free(struct diag_context *ctxt, struct usb_ep *ep,
			struct list_head *head)
{
	struct usb_request *req;
	while ((req = req_get(ctxt, head))) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

#if ROUTE_TO_USERSPACE
#define USB_DIAG_IOC_MAGIC 0xFF
#define USB_DIAG_FUNC_IOC_REGISTER_SET _IOW(USB_DIAG_IOC_MAGIC, 3, char *)

static long diag_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	unsigned long flags;
	unsigned char temp_id_table[ID_TABLE_SZ];

	if (cmd != USB_DIAG_FUNC_IOC_REGISTER_SET) {
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		return -EINVAL;
	}

	if (copy_from_user(temp_id_table, (unsigned char *)argp, ID_TABLE_SZ))
		return -EFAULT;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	memcpy(ctxt->id_table, temp_id_table, ID_TABLE_SZ);
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return 0;
}

static ssize_t diag_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req = 0;
	int ret = 0;

	mutex_lock(&ctxt->user_lock);

	if (ctxt->user_read_len && ctxt->user_readp) {
		if (count > ctxt->user_read_len)
			count = ctxt->user_read_len;
		if (copy_to_user(buf, ctxt->user_readp, count))
			ret = -EFAULT;
		else {
			ctxt->user_readp += count;
			ctxt->user_read_len -= count;
			ret = count;
		}
		goto end;
	}

	mutex_unlock(&ctxt->user_lock);
	ret = wait_event_interruptible(ctxt->read_wq,
		(req = req_get(ctxt, &ctxt->rx_req_user)) || !ctxt->online);
	mutex_lock(&ctxt->user_lock);
	if (ret < 0) {
		pr_err("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}
	if (!ctxt->online) {
		pr_err("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}
	if (req) {
		if (req->actual == 0) {
			pr_info("%s: no data\n", __func__);
			goto end;
		}
		if (count > req->actual)
			count = req->actual;
		if (copy_to_user(buf, req->buf, count)) {
			ret = -EFAULT;
			goto end;
		}
		req->actual -= count;
		if (req->actual) {
			memcpy(ctxt->user_read_buf, req->buf + count, req->actual);
			ctxt->user_read_len = req->actual;
			ctxt->user_readp = ctxt->user_read_buf;
		}
		ret = count;
	}

end:
	if (req)
		req_put(ctxt, &ctxt->rx_req_idle, req);

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static ssize_t diag_write(struct file *fp, const char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req = 0;
	int ret = 0;

	ret = wait_event_interruptible(ctxt->write_wq,
		((req = req_get(ctxt, &ctxt->tx_req_idle)) || !ctxt->online));

	mutex_lock(&ctxt->user_lock);
	if (ret < 0) {
		pr_err("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}

	if (!ctxt->online) {
		pr_err("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}

	if (count > TX_REQ_BUF_SZ)
		count = TX_REQ_BUF_SZ;

	if (req) {
		if (copy_from_user(req->buf, buf, count)) {
			ret = -EFAULT;
			goto end;
		}

		req->length = count;
		ret = usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
		if (ret < 0) {
			pr_err("%s: usb_ep_queue error %d\n", __func__, ret);
			goto end;
		}

		ret = req->length;
	}

end:
	if (req)
		req_put(ctxt, &ctxt->tx_req_idle, req);

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static int diag_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	int rc = 0;

	mutex_lock(&ctxt->user_lock);
	if (ctxt->opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (!ctxt->user_read_buf) {
		ctxt->user_read_buf = kmalloc(RX_REQ_BUF_SZ, GFP_KERNEL);
		if (!ctxt->user_read_buf) {
			rc = -ENOMEM;
			goto done;
		}
	}
	ctxt->opened = true;

done:
	mutex_unlock(&ctxt->user_lock);
	return rc;
}

static int diag_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;

	mutex_lock(&ctxt->user_lock);
	ctxt->opened = false;
	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (ctxt->user_read_buf) {
		kfree(ctxt->user_read_buf);
		ctxt->user_read_buf = 0;
	}
	mutex_unlock(&ctxt->user_lock);

	return 0;
}

static struct file_operations diag_fops = {
	.owner =   THIS_MODULE,
	.read =    diag_read,
	.write =   diag_write,
	.open =    diag_open,
	.release = diag_release,
	.unlocked_ioctl = diag_ioctl,
};

static struct miscdevice diag_device_fops = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag",
	.fops = &diag_fops,
};
#endif

static void diag_in_complete(struct usb_ep *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;
#if ROUTE_TO_USERSPACE
	char c;
#endif

	ctxt->in_busy = 0;
	req_put(ctxt, &ctxt->tx_req_idle, req);

#if ROUTE_TO_USERSPACE
	c = *((char *)req->buf + req->actual - 1);
	if (c == 0x7e)
		wake_up(&ctxt->write_wq);
#endif

	smd_try_to_send(ctxt);
}

#ifdef CONFIG_ARCH_QSD8X50
static void diag_dsp_in_complete(struct usb_ep *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;

	ctxt->in_busy_dsp = 0;
	req_put(ctxt, &ctxt->tx_req_idle, req);
	dsp_try_to_send(ctxt);
	wake_up(&ctxt->write_wq);
}
#endif

static void diag_process_hdlc(struct diag_context *ctxt, void *_data, unsigned len)
{
	unsigned char *data = _data;
	unsigned count = ctxt->hdlc_count;
	unsigned escape = ctxt->hdlc_escape;
	unsigned char *hdlc = ctxt->hdlc_buf;

	while (len-- > 0) {
		unsigned char x = *data++;
		if (x == 0x7E) { 
			if (count > 2) {
				/* we're just ignoring the crc here */
				TRACE("PC>", hdlc, count - 2, 0);
				if (ctxt->ch)
					smd_write(ctxt->ch, hdlc, count - 2);
			}
			count = 0;
			escape = 0;
		} else if (x == 0x7D) {
			escape = 1;
		} else {
			if (escape) {
				x = x ^ 0x20;
				escape = 0;
			}
			hdlc[count++] = x;

			/* discard frame if we overflow */
			if (count == HDLC_MAX)
				count = 0;
		}
	}

	ctxt->hdlc_count = count;
	ctxt->hdlc_escape = escape;
}

#if ROUTE_TO_USERSPACE
static int if_route_to_userspace(struct diag_context *ctxt, unsigned int cmd_id)
{
	unsigned long flags;
	int i;

	if (!ctxt->opened || cmd_id == 0)
		return 0;

	/* command ids 0xfb..0xff are not used by msm diag; we steal these ids
	 * for communication between userspace tool and host test tool.
	 */
	if (cmd_id >= 0xfb && cmd_id <= 0xff)
		return 1;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	for (i = 0; i < ARRAY_SIZE(ctxt->id_table); i++)
		if (ctxt->id_table[i] == cmd_id) {
			/* if the command id equals to any of registered ids,
			 * route to userspace to handle.
			 */
			spin_unlock_irqrestore(&ctxt->req_lock, flags);
			return 1;
		}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return 0;
}
#endif

static void diag_out_complete(struct usb_ep *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;

	if (req->status == 0) {
#if ROUTE_TO_USERSPACE
		unsigned int cmd_id = *((unsigned char *)req->buf);
		if (if_route_to_userspace(ctxt, cmd_id)) {
			req_put(ctxt, &ctxt->rx_req_user, req);
			wake_up(&ctxt->read_wq);
			diag_queue_out(ctxt);
			return;
		}
#endif

#if NO_HDLC
		TRACE("PC>", req->buf, req->actual, 0);
		if (ctxt->ch)
			smd_write(ctxt->ch, req->buf, req->actual);
#else
		diag_process_hdlc(ctxt, req->buf, req->actual);
#endif
		ctxt->tx_count += req->actual;
	}

	req_put(ctxt, &ctxt->rx_req_idle, req);
	diag_queue_out(ctxt);
}

static void diag_queue_out(struct diag_context *ctxt)
{
	struct usb_request *req;
	int rc;

	req = req_get(ctxt, &ctxt->rx_req_idle);
	if (!req) {
		pr_err("%s: rx req queue - out of buffer\n", __func__);
		return;
	}

	req->complete = diag_out_complete;
	req->context = ctxt;
	req->length = RX_REQ_BUF_SZ;

	rc = usb_ep_queue(ctxt->out, req, GFP_ATOMIC);
	if (rc < 0) {
		pr_err("%s: usb_ep_queue failed: %d\n", __func__, rc);
		req_put(ctxt, &ctxt->rx_req_idle, req);
	}
}

static void smd_try_to_send(struct diag_context *ctxt)
{
again:
	if (ctxt->ch && (!ctxt->in_busy)) {
		int r = smd_read_avail(ctxt->ch);

		if (r > TX_REQ_BUF_SZ) {
			return;
		}
		if (r > 0) {
			struct usb_request *req;
			req = req_get(ctxt, &ctxt->tx_req_idle);
			if (!req) {
				pr_err("%s: tx req queue is out of buffers\n",
					__func__);
				return;
			}
			smd_read(ctxt->ch, req->buf, r);
			ctxt->rx_count += r;

			if (!ctxt->online) {
//				printk("$$$ discard %d\n", r);
				req_put(ctxt, &ctxt->tx_req_idle, req);
				goto again;
			}
			req->complete = diag_in_complete;
			req->context = ctxt;
			req->length = r;

			TRACE("A9>", req->buf, r, 1);
			ctxt->in_busy = 1;
			r = usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
			if (r < 0) {
				pr_err("%s: usb_ep_queue failed: %d\n",
					__func__, r);
				req_put(ctxt, &ctxt->tx_req_idle, req);
				ctxt->in_busy = 0;
			}
		}
	}
}

static void smd_diag_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	smd_try_to_send(ctxt);
}

#ifdef CONFIG_ARCH_QSD8X50
static void dsp_try_to_send(struct diag_context *ctxt)
{
again:
	if (ctxt->ch_dsp && (!ctxt->in_busy_dsp)) {
		int r = smd_read_avail(ctxt->ch_dsp);

		if (r > TX_REQ_BUF_SZ) {
			return;
		}
		if (r > 0) {
			struct usb_request *req;
			req = req_get(ctxt, &ctxt->tx_req_idle);
			if (!req) {
				pr_err("%s: tx req queue is out of buffers\n",
					__func__);
				return;
			}
			smd_read(ctxt->ch_dsp, req->buf, r);

			if (!ctxt->online) {
//				printk("$$$ discard %d\n", r);
				req_put(ctxt, &ctxt->tx_req_idle, req);
				goto again;
			}
			req->complete = diag_dsp_in_complete;
			req->context = ctxt;
			req->length = r;

			TRACE("Q6>", req->buf, r, 1);
			ctxt->in_busy_dsp = 1;
			r = usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
			if (r < 0) {
				pr_err("%s: usb_ep_queue failed: %d\n",
					__func__, r);
				req_put(ctxt, &ctxt->tx_req_idle, req);
				ctxt->in_busy_dsp = 0;
			}
		}
	}
}

static void dsp_diag_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	dsp_try_to_send(ctxt);
}
#endif

static int __init create_bulk_endpoints(struct diag_context *ctxt,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = ctxt->cdev;
	struct usb_ep *ep;
	struct usb_request *req;
	int n;

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	ctxt->in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		return -ENODEV;
	}
	ctxt->out = ep;

	ctxt->tx_count = ctxt->rx_count = 0;

	for (n = 0; n < RX_REQ_NUM; n++) {
		req = usb_ep_alloc_request(ctxt->out, GFP_KERNEL);
		if (!req) {
			DBG(cdev, "%s: usb_ep_alloc_request out of memory\n",
				__func__);
			goto rx_fail;
		}
		req->buf = kmalloc(RX_REQ_BUF_SZ, GFP_KERNEL);
		if (!req->buf) {
			DBG(cdev, "%s: kmalloc out of memory\n", __func__);
			goto rx_fail;
		}
		req->context = ctxt;
		req->complete = diag_out_complete;
		req_put(ctxt, &ctxt->rx_req_idle, req);
	}

	for (n = 0; n < TX_REQ_NUM; n++) {
		req = usb_ep_alloc_request(ctxt->in, GFP_KERNEL);
		if (!req) {
			DBG(cdev, "%s: usb_ep_alloc_request out of memory\n",
				__func__);
			goto tx_fail;
		}
		req->buf = kmalloc(TX_REQ_BUF_SZ, GFP_KERNEL);
		if (!req->buf) {
			DBG(cdev, "%s: kmalloc out of memory\n", __func__);
			goto tx_fail;
		}
		req->context = ctxt;
		req->complete = diag_in_complete;
		req_put(ctxt, &ctxt->tx_req_idle, req);
	}

	return 0;

tx_fail:
	reqs_free(ctxt, ctxt->in, &ctxt->tx_req_idle);
rx_fail:
	reqs_free(ctxt, ctxt->out, &ctxt->rx_req_idle);
	return -ENOMEM;
}

static int
diag_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct diag_context	*ctxt = func_to_dev(f);
	int			id;
	int			ret;

	ctxt->cdev = cdev;

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	diag_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = create_bulk_endpoints(ctxt, &diag_fullspeed_in_desc,
			&diag_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		diag_highspeed_in_desc.bEndpointAddress =
			diag_fullspeed_in_desc.bEndpointAddress;
		diag_highspeed_out_desc.bEndpointAddress =
			diag_fullspeed_out_desc.bEndpointAddress;
	}

#if ROUTE_TO_USERSPACE
	misc_register(&diag_device_fops);
#endif

	return 0;
}

static void
diag_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct diag_context	*ctxt = func_to_dev(f);
	reqs_free(ctxt, ctxt->out, &ctxt->rx_req_idle);
	reqs_free(ctxt, ctxt->in, &ctxt->tx_req_idle);

#if ROUTE_TO_USERSPACE
	misc_deregister(&diag_device_fops);
#endif

	ctxt->tx_count = ctxt->rx_count = 0;
}

static int diag_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct diag_context	*ctxt = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
#if ROUTE_TO_USERSPACE
	struct usb_request *req;
#endif
	int ret;

	ret = usb_ep_enable(ctxt->in,
			ep_choose(cdev->gadget,
				&diag_highspeed_in_desc,
				&diag_fullspeed_in_desc));
	if (ret)
		return ret;
	ret = usb_ep_enable(ctxt->out,
			ep_choose(cdev->gadget,
				&diag_highspeed_out_desc,
				&diag_fullspeed_out_desc));
	if (ret) {
		usb_ep_disable(ctxt->in);
		return ret;
	}
	ctxt->online = 1;

#if ROUTE_TO_USERSPACE
	/* recycle unhandled rx reqs to user if any */
	while ((req = req_get(ctxt, &ctxt->rx_req_user)))
		req_put(ctxt, &ctxt->rx_req_idle, req);
#endif

	diag_queue_out(ctxt);
	smd_try_to_send(ctxt);
#ifdef CONFIG_ARCH_QSD8X50
	dsp_try_to_send(ctxt);
#endif

#if ROUTE_TO_USERSPACE
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->write_wq);
#endif

	return 0;
}

static void diag_function_disable(struct usb_function *f)
{
	struct diag_context	*ctxt = func_to_dev(f);

	ctxt->online = 0;
#if ROUTE_TO_USERSPACE
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->write_wq);
#endif
	usb_ep_disable(ctxt->in);
	usb_ep_disable(ctxt->out);
}

static int diag_set_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	if (_context.cdev)
		android_enable_function(&_context.function, enabled);
	_context.function_enable = !!enabled;
	return 0;
}

static int diag_get_tx_rx_count(char *buffer, struct kernel_param *kp)
{
	struct diag_context *ctxt = &_context;

	return sprintf(buffer, "tx: %llu bytes, rx: %llu bytes",
	ctxt->tx_count, ctxt->rx_count);
}
module_param_call(tx_rx_count, NULL, diag_get_tx_rx_count, NULL, 0444);

static int diag_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !_context.function.disabled;
	return 1;
}
module_param_call(enabled, diag_set_enabled, diag_get_enabled, NULL, 0664);


int diag_bind_config(struct usb_configuration *c)
{
	struct diag_context *ctxt = &_context;
	int ret;

	printk(KERN_INFO "diag_bind_config\n");

	ret = smd_open("SMD_DIAG", &ctxt->ch, ctxt, smd_diag_notify);
	if (ret)
		return ret;

#ifdef CONFIG_ARCH_QSD8X50
	ret = smd_open("DSP_DIAG", &ctxt->ch_dsp, ctxt, dsp_diag_notify);
	if (ret) {
		pr_err("%s: smd_open failed (DSP_DIAG)\n", __func__);
		return ret;
	}
#endif

	ctxt->cdev = c->cdev;
	ctxt->function.name = "diag";
	ctxt->function.descriptors = fs_diag_descs;
	ctxt->function.hs_descriptors = hs_diag_descs;
	ctxt->function.bind = diag_function_bind;
	ctxt->function.unbind = diag_function_unbind;
	ctxt->function.set_alt = diag_function_set_alt;
	ctxt->function.disable = diag_function_disable;

	ctxt->function.disabled = !_context.function_enable;

	return usb_add_function(c, &ctxt->function);
}

static struct android_usb_function diag_function = {
	.name = "diag",
	.bind_config = diag_bind_config,
};

static int __init init(void)
{
	struct diag_context *ctxt = &_context;

	printk(KERN_INFO "diag init\n");
	spin_lock_init(&ctxt->req_lock);
	INIT_LIST_HEAD(&ctxt->rx_req_idle);
	INIT_LIST_HEAD(&ctxt->tx_req_idle);
#if ROUTE_TO_USERSPACE
	mutex_init(&ctxt->user_lock);
	INIT_LIST_HEAD(&ctxt->rx_req_user);
	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);
#endif

	android_register_function(&diag_function);
	return 0;
}
module_init(init);

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

#include <mach/msm_smd.h>

#include <linux/usb/android_composite.h>

#define NO_HDLC 1

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

struct diag_context
{
	struct usb_function function;
	struct usb_composite_dev *cdev;
	struct usb_ep *out;
	struct usb_ep *in;
	struct usb_request *req_out;
	struct usb_request *req_in;
	smd_channel_t *ch;
	int in_busy;
	int online;

	/* assembly buffer for USB->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;
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

static void diag_queue_out(struct diag_context *ctxt);

static void diag_in_complete(struct usb_ep *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;
	ctxt->in_busy = 0;
	smd_try_to_send(ctxt);
}

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

static void diag_out_complete(struct usb_ep *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;

	if (req->status == 0) {
#if NO_HDLC
		TRACE("PC>", req->buf, req->actual, 0);
		if (ctxt->ch)
			smd_write(ctxt->ch, req->buf, req->actual);
#else
		diag_process_hdlc(ctxt, req->buf, req->actual);
#endif
	}
	diag_queue_out(ctxt);
}

static void diag_queue_out(struct diag_context *ctxt)
{
	struct usb_request *req = ctxt->req_out;

	req->complete = diag_out_complete;
	req->context = ctxt;
	req->length = 8192;

	usb_ep_queue(ctxt->out, req, GFP_ATOMIC);
}

static void smd_try_to_send(struct diag_context *ctxt)
{
again:
	if (ctxt->ch && (!ctxt->in_busy)) {
		int r = smd_read_avail(ctxt->ch);

		if (r > 8192) {
			return;
		}
		if (r > 0) {
			struct usb_request *req = ctxt->req_in;
			smd_read(ctxt->ch, req->buf, r);

			if (!ctxt->online) {
//				printk("$$$ discard %d\n", r);
				goto again;
			}
			req->complete = diag_in_complete;
			req->context = ctxt;
			req->length = r;

			TRACE("A9>", req->buf, r, 1);
			ctxt->in_busy = 1;
			usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
		}
	}
}

static void smd_diag_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	smd_try_to_send(ctxt);
}

static int __init create_bulk_endpoints(struct diag_context *ctxt,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = ctxt->cdev;
	struct usb_ep *ep;

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

	ctxt->req_out = usb_ep_alloc_request(ctxt->out, GFP_KERNEL);
	ctxt->req_in = usb_ep_alloc_request(ctxt->in, GFP_KERNEL);
	ctxt->req_out->buf = kmalloc(8192, GFP_KERNEL);
	ctxt->req_in->buf = kmalloc(8192, GFP_KERNEL);
	ctxt->req_out->complete = diag_out_complete;
	ctxt->req_in->complete = diag_in_complete;

	return 0;
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

	return 0;
}

static void
diag_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct diag_context	*ctxt = func_to_dev(f);

	kfree(ctxt->req_out->buf);
	kfree(ctxt->req_in->buf);
	usb_ep_free_request(ctxt->out, ctxt->req_out);
	usb_ep_free_request(ctxt->in, ctxt->req_in);
}

static int diag_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct diag_context	*ctxt = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
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

	diag_queue_out(ctxt);
	smd_try_to_send(ctxt);

	return 0;
}

static void diag_function_disable(struct usb_function *f)
{
	struct diag_context	*ctxt = func_to_dev(f);

	ctxt->online = 0;
	usb_ep_disable(ctxt->in);
	usb_ep_disable(ctxt->out);
}

static int diag_set_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	if (_context.cdev)
		android_enable_function(&_context.function, enabled);
	return 0;
}

static int diag_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !_context.function.hidden;
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

	ctxt->cdev = c->cdev;
	ctxt->function.name = "diag";
	ctxt->function.descriptors = fs_diag_descs;
	ctxt->function.hs_descriptors = hs_diag_descs;
	ctxt->function.bind = diag_function_bind;
	ctxt->function.unbind = diag_function_unbind;
	ctxt->function.set_alt = diag_function_set_alt;
	ctxt->function.disable = diag_function_disable;

	/* start disabled */
	ctxt->function.hidden = 1;

	return usb_add_function(c, &ctxt->function);
}

static struct android_usb_function diag_function = {
	.name = "diag",
	.bind_config = diag_bind_config,
};

static int __init init(void)
{
	printk(KERN_INFO "diag init\n");
	android_register_function(&diag_function);
	return 0;
}
module_init(init);

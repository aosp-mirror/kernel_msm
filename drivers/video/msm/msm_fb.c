/* drivers/video/msm/msm_fb.c
 *
 * Core MSM framebuffer driver.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/msm_mdp.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <mach/msm_fb.h>
#include <mach/board.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>

#define MSMFB_DEBUG 1
#ifdef CONFIG_FB_MSM_LOGO
#define INIT_IMAGE_FILE "/logo.rle"
extern int load_565rle_image(char *filename);
#endif

#define PRINT_FPS 0
#define PRINT_BLIT_TIME 0

#define SLEEPING 0x4
#define UPDATING 0x3
#define FULL_UPDATE_DONE 0x2
#define WAKING 0x1
#define AWAKE 0x0

#define NONE 0
#define SUSPEND_RESUME 0x1
#define FPS 0x2
#define BLIT_TIME 0x4
#define SHOW_UPDATES 0x8

#define DLOG(mask, fmt, args...) \
do { \
	if (msmfb_debug_mask & mask) \
		printk(KERN_INFO "msmfb: "fmt, ##args); \
} while (0)

#define BITS_PER_PIXEL(info) (info->fb->var.bits_per_pixel)
#define BYTES_PER_PIXEL(info) (info->fb->var.bits_per_pixel >> 3)

static int msmfb_debug_mask;
module_param_named(msmfb_debug_mask, msmfb_debug_mask, int,
		   S_IRUGO | S_IWUSR | S_IWGRP);

struct mdp_device *mdp;

struct msmfb_info {
	struct fb_info *fb;
	struct msm_panel_data *panel;
	int xres;
	int yres;
	unsigned output_format;
	unsigned yoffset;
	unsigned frame_requested;
	unsigned frame_done;
	int sleeping;
	unsigned update_frame;
	struct {
		int left;
		int top;
		int eright; /* exclusive */
		int ebottom; /* exclusive */
	} update_info;
	char *black;

	struct early_suspend earlier_suspend;
	struct early_suspend early_suspend;
	struct wake_lock idle_lock;
	spinlock_t update_lock;
	struct mutex panel_init_lock;
	wait_queue_head_t frame_wq;
	struct workqueue_struct *resume_workqueue;
	struct work_struct resume_work;
	struct msmfb_callback dma_callback;
	struct msmfb_callback vsync_callback;
	struct hrtimer fake_vsync;
	ktime_t vsync_request_time;
};

static int msmfb_open(struct fb_info *info, int user)
{
	return 0;
}

static int msmfb_release(struct fb_info *info, int user)
{
	return 0;
}

/* Called from dma interrupt handler, must not sleep */
static void msmfb_handle_dma_interrupt(struct msmfb_callback *callback)
{
	unsigned long irq_flags;
	struct msmfb_info *msmfb  = container_of(callback, struct msmfb_info,
					       dma_callback);
#if PRINT_FPS
	int64_t dt;
	ktime_t now;
	static int64_t frame_count;
	static ktime_t last_sec;
#endif

	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	msmfb->frame_done = msmfb->frame_requested;
	if (msmfb->sleeping == UPDATING &&
	    msmfb->frame_done == msmfb->update_frame) {
		DLOG(SUSPEND_RESUME, "full update completed\n");
		queue_work(msmfb->resume_workqueue, &msmfb->resume_work);
	}
#if PRINT_FPS
	now = ktime_get();
	dt = ktime_to_ns(ktime_sub(now, last_sec));
	frame_count++;
	if (dt > NSEC_PER_SEC) {
		int64_t fps = frame_count * NSEC_PER_SEC * 100;
		frame_count = 0;
		last_sec = ktime_get();
		do_div(fps, dt);
		DLOG(FPS, "fps * 100: %llu\n", fps);
	}
#endif
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	wake_up(&msmfb->frame_wq);
}

static int msmfb_start_dma(struct msmfb_info *msmfb)
{
	uint32_t x, y, w, h;
	unsigned addr;
	unsigned long irq_flags;
	uint32_t yoffset;
	s64 time_since_request;
	struct msm_panel_data *panel = msmfb->panel;

	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	time_since_request = ktime_to_ns(ktime_sub(ktime_get(),
			     msmfb->vsync_request_time));
	if (time_since_request > 20 * NSEC_PER_MSEC) {
		uint32_t us;
		us = do_div(time_since_request, NSEC_PER_MSEC) / NSEC_PER_USEC;
		printk(KERN_WARNING "msmfb_start_dma %lld.%03u ms after vsync "
			"request\n", time_since_request, us);
	}
	if (msmfb->frame_done == msmfb->frame_requested) {
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		return -1;
	}
	if (msmfb->sleeping == SLEEPING) {
		DLOG(SUSPEND_RESUME, "tried to start dma while asleep\n");
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		return -1;
	}
	x = msmfb->update_info.left;
	y = msmfb->update_info.top;
	w = msmfb->update_info.eright - x;
	h = msmfb->update_info.ebottom - y;
	yoffset = msmfb->yoffset;
	msmfb->update_info.left = msmfb->xres + 1;
	msmfb->update_info.top = msmfb->yres + 1;
	msmfb->update_info.eright = 0;
	msmfb->update_info.ebottom = 0;
	if (unlikely(w > msmfb->xres || h > msmfb->yres ||
		     w == 0 || h == 0)) {
		printk(KERN_INFO "invalid update: %d %d %d "
				"%d\n", x, y, w, h);
		msmfb->frame_done = msmfb->frame_requested;
		goto error;
	}
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);

	addr = ((msmfb->xres * (yoffset + y) + x) * BYTES_PER_PIXEL(msmfb));
	mdp->dma(mdp, addr + msmfb->fb->fix.smem_start,
		 msmfb->xres * BYTES_PER_PIXEL(msmfb), w, h, x, y,
		 &msmfb->dma_callback,
		 panel->interface_type);
	return 0;
error:
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	/* some clients need to clear their vsync interrupt */
	if (panel->clear_vsync)
		panel->clear_vsync(panel);
	wake_up(&msmfb->frame_wq);
	return 0;
}

/* Called from esync interrupt handler, must not sleep */
static void msmfb_handle_vsync_interrupt(struct msmfb_callback *callback)
{
	struct msmfb_info *msmfb = container_of(callback, struct msmfb_info,
					       vsync_callback);
	wake_unlock(&msmfb->idle_lock);
	msmfb_start_dma(msmfb);
}

static enum hrtimer_restart msmfb_fake_vsync(struct hrtimer *timer)
{
	struct msmfb_info *msmfb  = container_of(timer, struct msmfb_info,
					       fake_vsync);
	msmfb_start_dma(msmfb);
	return HRTIMER_NORESTART;
}

static void msmfb_pan_update(struct fb_info *info, uint32_t left, uint32_t top,
			     uint32_t eright, uint32_t ebottom,
			     uint32_t yoffset, int pan_display)
{
	struct msmfb_info *msmfb = info->par;
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;
	int sleeping;
	int retry = 1;
#if PRINT_FPS
	ktime_t t1, t2;
	static uint64_t pans;
	static uint64_t dt;
	t1 = ktime_get();
#endif

	DLOG(SHOW_UPDATES, "update %d %d %d %d %d %d\n",
		left, top, eright, ebottom, yoffset, pan_display);
restart:
	spin_lock_irqsave(&msmfb->update_lock, irq_flags);

	/* if we are sleeping, on a pan_display wait 10ms (to throttle back
	 * drawing otherwise return */
	if (msmfb->sleeping == SLEEPING) {
		DLOG(SUSPEND_RESUME, "drawing while asleep\n");
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		if (pan_display)
			wait_event_interruptible_timeout(msmfb->frame_wq,
				msmfb->sleeping != SLEEPING, HZ/10);
		return;
	}

	sleeping = msmfb->sleeping;
	/* on a full update, if the last frame has not completed, wait for it */
	if ((pan_display && msmfb->frame_requested != msmfb->frame_done) ||
			    sleeping == UPDATING) {
		int ret;
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		ret = wait_event_interruptible_timeout(msmfb->frame_wq,
			msmfb->frame_done == msmfb->frame_requested &&
			msmfb->sleeping != UPDATING, 5 * HZ);
		if (ret <= 0 && (msmfb->frame_requested != msmfb->frame_done ||
				 msmfb->sleeping == UPDATING)) {
			if (retry && panel->request_vsync &&
			    (sleeping == AWAKE)) {
				wake_lock_timeout(&msmfb->idle_lock, HZ/4);
				panel->request_vsync(panel,
					&msmfb->vsync_callback);
				retry = 0;
				printk(KERN_WARNING "msmfb_pan_display timeout "
					"rerequest vsync\n");
			} else {
				printk(KERN_WARNING "msmfb_pan_display timeout "
					"waiting for frame start, %d %d\n",
					msmfb->frame_requested,
					msmfb->frame_done);
				return;
			}
		}
		goto restart;
	}

#if PRINT_FPS
	t2 = ktime_get();
	if (pan_display) {
		uint64_t temp = ktime_to_ns(ktime_sub(t2, t1));
		do_div(temp, 1000);
		dt += temp;
		pans++;
		if (pans > 1000) {
			do_div(dt, pans);
			DLOG(FPS, "ave_wait_time: %lld\n", dt);
			dt = 0;
			pans = 0;
		}
	}
#endif

	msmfb->frame_requested++;
	/* if necessary, update the y offset, if this is the
	 * first full update on resume, set the sleeping state */
	if (pan_display) {
		msmfb->yoffset = yoffset;
		if (left == 0 && top == 0 && eright == info->var.xres &&
		    ebottom == info->var.yres) {
			if (sleeping == WAKING) {
				msmfb->update_frame = msmfb->frame_requested;
				DLOG(SUSPEND_RESUME, "full update starting\n");
				msmfb->sleeping = UPDATING;
			}
		}
	}

	/* set the update request */
	if (left < msmfb->update_info.left)
		msmfb->update_info.left = left;
	if (top < msmfb->update_info.top)
		msmfb->update_info.top = top;
	if (eright > msmfb->update_info.eright)
		msmfb->update_info.eright = eright;
	if (ebottom > msmfb->update_info.ebottom)
		msmfb->update_info.ebottom = ebottom;
	DLOG(SHOW_UPDATES, "update queued %d %d %d %d %d\n",
		msmfb->update_info.left, msmfb->update_info.top,
		msmfb->update_info.eright, msmfb->update_info.ebottom,
		msmfb->yoffset);
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);

	/* if the panel is all the way on wait for vsync, otherwise sleep
	 * for 16 ms (long enough for the dma to panel) and then begin dma */
	msmfb->vsync_request_time = ktime_get();
	if (panel->request_vsync && (sleeping == AWAKE)) {
		wake_lock_timeout(&msmfb->idle_lock, HZ/4);
		panel->request_vsync(panel, &msmfb->vsync_callback);
	} else {
		if (!hrtimer_active(&msmfb->fake_vsync)) {
			hrtimer_start(&msmfb->fake_vsync,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
		}
	}
}

static void msmfb_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom)
{
	msmfb_pan_update(info, left, top, eright, ebottom, 0, 0);
}

static void power_on_panel(struct work_struct *work)
{
	struct msmfb_info *msmfb =
		container_of(work, struct msmfb_info, resume_work);
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;

	mutex_lock(&msmfb->panel_init_lock);
	DLOG(SUSPEND_RESUME, "turning on panel\n");
	if (msmfb->sleeping == UPDATING) {
		wake_lock_timeout(&msmfb->idle_lock, HZ);
		if (panel->unblank(panel)) {
			printk(KERN_INFO "msmfb: panel unblank failed,"
			       "not starting drawing\n");
			goto error;
		}
		wake_unlock(&msmfb->idle_lock);
		spin_lock_irqsave(&msmfb->update_lock, irq_flags);
		msmfb->sleeping = AWAKE;
		wake_up(&msmfb->frame_wq);
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	}
error:
	mutex_unlock(&msmfb->panel_init_lock);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/* turn off the panel */
static void msmfb_earlier_suspend(struct early_suspend *h)
{
	struct msmfb_info *msmfb = container_of(h, struct msmfb_info,
						earlier_suspend);
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;

	mutex_lock(&msmfb->panel_init_lock);
	msmfb->sleeping = SLEEPING;
	wake_up(&msmfb->frame_wq);
	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	wait_event_timeout(msmfb->frame_wq,
			   msmfb->frame_requested == msmfb->frame_done, HZ/10);

	mdp->dma(mdp, virt_to_phys(msmfb->black), 0,
		 msmfb->fb->var.xres, msmfb->fb->var.yres, 0, 0,
		 NULL, panel->interface_type);
	mdp->dma_wait(mdp, panel->interface_type);

	/* turn off the panel */
	panel->blank(panel);
}

static void msmfb_suspend(struct early_suspend *h)
{
	struct msmfb_info *msmfb = container_of(h, struct msmfb_info,
						early_suspend);
	struct msm_panel_data *panel = msmfb->panel;
	/* suspend the panel */
	panel->suspend(panel);
	mutex_unlock(&msmfb->panel_init_lock);
}

static void msmfb_resume(struct early_suspend *h)
{
	struct msmfb_info *msmfb = container_of(h, struct msmfb_info,
						early_suspend);
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;

	if (panel->resume(panel)) {
		printk(KERN_INFO "msmfb: panel resume failed, not resuming "
		       "fb\n");
		return;
	}
	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	msmfb->frame_requested = msmfb->frame_done = msmfb->update_frame = 0;
	msmfb->sleeping = WAKING;
	DLOG(SUSPEND_RESUME, "ready, waiting for full update\n");
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
}
#endif

static int msmfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 size;

	if ((var->xres != info->var.xres) ||
	    (var->yres != info->var.yres) ||
	    (var->xoffset != info->var.xoffset) ||
	    (mdp->check_output_format(mdp, var->bits_per_pixel)) ||
	    (var->grayscale != info->var.grayscale))
		 return -EINVAL;

	size = var->xres_virtual * var->yres_virtual *
		(var->bits_per_pixel >> 3);
	if (size > info->fix.smem_len)
		return -EINVAL;
	return 0;
}

static int msmfb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;

	/* we only support RGB ordering for now */
	if (var->bits_per_pixel == 32 || var->bits_per_pixel == 24) {
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
	} else if (var->bits_per_pixel == 16) {
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
	} else
		return -1;
	mdp->set_output_format(mdp, var->bits_per_pixel);
	fix->line_length = var->xres * var->bits_per_pixel / 8;

	return 0;
}

int msmfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct msmfb_info *msmfb = info->par;
	struct msm_panel_data *panel = msmfb->panel;

	/* "UPDT" */
	if ((panel->caps & MSMFB_CAP_PARTIAL_UPDATES) &&
	    (var->reserved[0] == 0x54445055)) {
#if 0
		printk(KERN_INFO "pan frame %d-%d, rect %d %d %d %d\n",
		       msmfb->frame_requested, msmfb->frame_done,
		       var->reserved[1] & 0xffff,
		       var->reserved[1] >> 16, var->reserved[2] & 0xffff,
		       var->reserved[2] >> 16);
#endif
		msmfb_pan_update(info, var->reserved[1] & 0xffff,
				 var->reserved[1] >> 16,
				 var->reserved[2] & 0xffff,
				 var->reserved[2] >> 16, var->yoffset, 1);
	} else {
		msmfb_pan_update(info, 0, 0, info->var.xres, info->var.yres,
				 var->yoffset, 1);
	}
	return 0;
}

static void msmfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	cfb_fillrect(p, rect);
	msmfb_update(p, rect->dx, rect->dy, rect->dx + rect->width,
		     rect->dy + rect->height);
}

static void msmfb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	cfb_copyarea(p, area);
	msmfb_update(p, area->dx, area->dy, area->dx + area->width,
		     area->dy + area->height);
}

static void msmfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	msmfb_update(p, image->dx, image->dy, image->dx + image->width,
		     image->dy + image->height);
}


static int msmfb_blit(struct fb_info *info,
		      void __user *p)
{
	struct mdp_blit_req req;
	struct mdp_blit_req_list req_list;
	int i;
	int ret;

	if (copy_from_user(&req_list, p, sizeof(req_list)))
		return -EFAULT;

	for (i = 0; i < req_list.count; i++) {
		struct mdp_blit_req_list *list =
			(struct mdp_blit_req_list *)p;
		if (copy_from_user(&req, &list->req[i], sizeof(req)))
			return -EFAULT;
		ret = mdp->blit(mdp, info, &req);
		if (ret)
			return ret;
	}
	return 0;
}


DEFINE_MUTEX(mdp_ppp_lock);

static int msmfb_ioctl(struct fb_info *p, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret;
#if PRINT_BLIT_TIME
	ktime_t t1, t2;
#endif

	switch (cmd) {
	case MSMFB_GRP_DISP:
		mdp->set_grp_disp(mdp, arg);
		break;
	case MSMFB_BLIT:
#if PRINT_BLIT_TIME
		t1 = ktime_get();
#endif
		ret = msmfb_blit(p, argp);
#if PRINT_BLIT_TIME
		t2 = ktime_get();
		DLOG(BLIT_TIME, "total %lld\n",
		       ktime_to_ns(t2) - ktime_to_ns(t1));
#endif
		if (ret)
			return ret;
		break;
	default:
			printk(KERN_INFO "msmfb unknown ioctl: %d\n", cmd);
			return -EINVAL;
	}
	return 0;
}

static struct fb_ops msmfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = msmfb_open,
	.fb_release = msmfb_release,
	.fb_check_var = msmfb_check_var,
	.fb_set_par = msmfb_set_par,
	.fb_pan_display = msmfb_pan_display,
	.fb_fillrect = msmfb_fillrect,
	.fb_copyarea = msmfb_copyarea,
	.fb_imageblit = msmfb_imageblit,
	.fb_ioctl = msmfb_ioctl,
};

static unsigned PP[16];


#if MSMFB_DEBUG
static ssize_t debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}


static ssize_t debug_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	const int debug_bufmax = 4096;
	static char buffer[4096];
	int n = 0;
	struct msmfb_info *msmfb = (struct msmfb_info *)file->private_data;
	unsigned long irq_flags;

	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	n = scnprintf(buffer, debug_bufmax, "yoffset %d\n", msmfb->yoffset);
	n += scnprintf(buffer + n, debug_bufmax, "frame_requested %d\n",
		       msmfb->frame_requested);
	n += scnprintf(buffer + n, debug_bufmax, "frame_done %d\n",
		       msmfb->frame_done);
	n += scnprintf(buffer + n, debug_bufmax, "sleeping %d\n",
		       msmfb->sleeping);
	n += scnprintf(buffer + n, debug_bufmax, "update_frame %d\n",
		       msmfb->update_frame);
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	n++;
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations debug_fops = {
	.read = debug_read,
	.open = debug_open,
};
#endif

static void setup_fb_info(struct msmfb_info *msmfb)
{
	struct fb_info *fb_info = msmfb->fb;
	int r;

	/* finish setting up the fb_info struct */
	strncpy(fb_info->fix.id, "msmfb", 16);
	fb_info->fix.ypanstep = 1;

	fb_info->fbops = &msmfb_ops;
	fb_info->flags = FBINFO_DEFAULT;

	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR;
	fb_info->fix.line_length = msmfb->xres * 2;
	fb_info->var.xres = msmfb->xres;
	fb_info->var.yres = msmfb->yres;
	fb_info->var.width = msmfb->panel->fb_data->width;
	fb_info->var.height = msmfb->panel->fb_data->height;
	fb_info->var.xres_virtual = msmfb->xres;
	fb_info->var.yres_virtual = msmfb->yres * 2;
	fb_info->var.bits_per_pixel = 16;
	fb_info->var.accel_flags = 0;

	fb_info->var.yoffset = 0;

	if (msmfb->panel->caps & MSMFB_CAP_PARTIAL_UPDATES) {
		/* set the param in the fixed screen, so userspace can't
		 * change it. This will be used to check for the
		 * capability. */
		fb_info->fix.reserved[0] = 0x5444;
		fb_info->fix.reserved[1] = 0x5055;

		/* This preloads the value so that if userspace doesn't
		 * change it, it will be a full update */
		fb_info->var.reserved[0] = 0x54445055;
		fb_info->var.reserved[1] = 0;
		fb_info->var.reserved[2] = (uint16_t)msmfb->xres |
					   ((uint32_t)msmfb->yres << 16);
	}

	fb_info->var.red.offset = 11;
	fb_info->var.red.length = 5;
	fb_info->var.red.msb_right = 0;
	fb_info->var.green.offset = 5;
	fb_info->var.green.length = 6;
	fb_info->var.green.msb_right = 0;
	fb_info->var.blue.offset = 0;
	fb_info->var.blue.length = 5;
	fb_info->var.blue.msb_right = 0;

	mdp->set_output_format(mdp, fb_info->var.bits_per_pixel);

	r = fb_alloc_cmap(&fb_info->cmap, 16, 0);
	fb_info->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;
}

static int setup_fbmem(struct msmfb_info *msmfb, struct platform_device *pdev)
{
	struct fb_info *fb = msmfb->fb;
	struct resource *resource;
	unsigned long size = msmfb->xres * msmfb->yres *
		BYTES_PER_PIXEL(msmfb) * 2;
	unsigned long resource_size;
	unsigned char *fbram;

	/* board file might have attached a resource describing an fb */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource)
		return -EINVAL;
	resource_size = resource->end - resource->start + 1;

	/* check the resource is large enough to fit the fb */
	if (resource_size < size) {
		printk(KERN_ERR "msmfb: allocated resource is too small for "
				"fb\n");
		return -ENOMEM;
	}
	fb->fix.smem_start = resource->start;
	fb->fix.smem_len = resource_size;
	fbram = ioremap(resource->start, resource_size);
	if (fbram == 0) {
		printk(KERN_ERR "msmfb: cannot allocate fbram!\n");
		return -ENOMEM;
	}

	fb->screen_base = fbram;
	return 0;
}

static int msmfb_probe(struct platform_device *pdev)
{
	struct fb_info *fb;
	struct msmfb_info *msmfb;
	struct msm_panel_data *panel = pdev->dev.platform_data;
	int ret;

	if (!panel) {
		pr_err("msmfb_probe: no platform data\n");
		return -EINVAL;
	}
	if (!panel->fb_data) {
		pr_err("msmfb_probe: no fb_data\n");
		return -EINVAL;
	}

	fb = framebuffer_alloc(sizeof(struct msmfb_info), &pdev->dev);
	if (!fb)
		return -ENOMEM;
	msmfb = fb->par;
	msmfb->fb = fb;
	msmfb->panel = panel;
	msmfb->xres = panel->fb_data->xres;
	msmfb->yres = panel->fb_data->yres;

	ret = setup_fbmem(msmfb, pdev);
	if (ret)
		goto error_setup_fbmem;

	setup_fb_info(msmfb);

	spin_lock_init(&msmfb->update_lock);
	mutex_init(&msmfb->panel_init_lock);
	init_waitqueue_head(&msmfb->frame_wq);
	msmfb->resume_workqueue = create_workqueue("panel_on");
	if (msmfb->resume_workqueue == NULL) {
		printk(KERN_ERR "failed to create panel_on workqueue\n");
		ret = -ENOMEM;
		goto error_create_workqueue;
	}
	INIT_WORK(&msmfb->resume_work, power_on_panel);
	msmfb->black = kzalloc(msmfb->fb->var.bits_per_pixel*msmfb->xres,
			       GFP_KERNEL);

	wake_lock_init(&msmfb->idle_lock, WAKE_LOCK_IDLE, "msmfb_idle_lock");

#ifdef CONFIG_HAS_EARLYSUSPEND
	msmfb->early_suspend.suspend = msmfb_suspend;
	msmfb->early_suspend.resume = msmfb_resume;
	msmfb->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&msmfb->early_suspend);

	msmfb->earlier_suspend.suspend = msmfb_earlier_suspend;
	msmfb->earlier_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&msmfb->earlier_suspend);
#endif

#if MSMFB_DEBUG
	debugfs_create_file("msm_fb", S_IFREG | S_IRUGO, NULL,
			    (void *)fb->par, &debug_fops);
#endif

	printk(KERN_INFO "msmfb_probe() installing %d x %d panel\n",
	       msmfb->xres, msmfb->yres);

	msmfb->dma_callback.func = msmfb_handle_dma_interrupt;
	msmfb->vsync_callback.func = msmfb_handle_vsync_interrupt;
	hrtimer_init(&msmfb->fake_vsync, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);


	msmfb->fake_vsync.function = msmfb_fake_vsync;

	ret = register_framebuffer(fb);
	if (ret)
		goto error_register_framebuffer;

	msmfb->sleeping = WAKING;

#ifdef CONFIG_FB_MSM_LOGO
	if (!load_565rle_image(INIT_IMAGE_FILE)) {
		/* Flip buffer */
		msmfb->update_info.left = 0;
		msmfb->update_info.top = 0;
		msmfb->update_info.eright = info->var.xres;
		msmfb->update_info.ebottom = info->var.yres;
		msmfb_pan_update(info, 0, 0, fb->var.xres,
				 fb->var.yres, 0, 1);
	}
#endif
	return 0;

error_register_framebuffer:
	wake_lock_destroy(&msmfb->idle_lock);
	destroy_workqueue(msmfb->resume_workqueue);
error_create_workqueue:
	iounmap(fb->screen_base);
error_setup_fbmem:
	framebuffer_release(msmfb->fb);
	return ret;
}

static struct platform_driver msm_panel_driver = {
	/* need to write remove */
	.probe = msmfb_probe,
	.driver = {.name = "msm_panel"},
};


static int msmfb_add_mdp_device(struct device *dev,
				struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (mdp)
		return 0;
	mdp = container_of(dev, struct mdp_device, dev);
	return platform_driver_register(&msm_panel_driver);
}

static void msmfb_remove_mdp_device(struct device *dev,
				struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (dev != &mdp->dev)
		return;
	platform_driver_unregister(&msm_panel_driver);
	mdp = NULL;
}

static struct class_interface msm_fb_interface = {
	.add_dev = &msmfb_add_mdp_device,
	.remove_dev = &msmfb_remove_mdp_device,
};

static int __init msmfb_init(void)
{
	return register_mdp_client(&msm_fb_interface);
}

module_init(msmfb_init);

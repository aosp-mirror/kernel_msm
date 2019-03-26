/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <uapi/linux/sched/types.h>
#include <video/mipi_display.h>

#include "dsi_display.h"
#include "dsi_panel.h"
#include "sde_trace.h"
#include "sde_connector.h"


#define TE_TIMEOUT_MS	50

#define for_each_display_mode(i, mode, panel) \
	for (i = 0, mode = dsi_panel_to_display(panel)->modes; \
		i < panel->num_timing_nodes; i++, mode++)


struct panel_switch_funcs {
	struct panel_switch_data *(*create)(struct dsi_panel *panel);
	void (*destroy)(struct panel_switch_data *pdata);
	void (*perform_switch)(struct panel_switch_data *pdata,
			       const struct dsi_display_mode *mode);
};

struct panel_switch_data {
	struct dsi_panel *panel;
	struct dentry *debug_root;

	struct kthread_work switch_work;
	struct kthread_worker worker;
	struct task_struct *thread;

	const struct dsi_display_mode *display_mode;
	const struct dsi_display_mode *idle_mode;
	wait_queue_head_t switch_wq;
	bool switch_pending;
	int switch_te_listen_count;

	atomic_t te_counter;
	struct dsi_display_te_listener te_listener;
	struct completion te_completion;

	const struct panel_switch_funcs *funcs;
};

static inline
struct dsi_display *dsi_panel_to_display(const struct dsi_panel *panel)
{
	return dev_get_drvdata(panel->parent);
}

static inline bool is_display_mode_same(const struct dsi_display_mode *m1,
					const struct dsi_display_mode *m2)
{
	return m1 && m2 && m1->timing.refresh_rate == m2->timing.refresh_rate;
}

static inline void sde_atrace_mode_fps(const struct panel_switch_data *pdata,
				       const struct dsi_display_mode *mode)
{
	sde_atrace('C', pdata->thread, "FPS", mode->timing.refresh_rate);
}

static void panel_handle_te(struct dsi_display_te_listener *tl)
{
	struct panel_switch_data *pdata;

	if (unlikely(!tl))
		return;

	pdata = container_of(tl, struct panel_switch_data, te_listener);

	complete(&pdata->te_completion);

	if (likely(pdata->thread)) {
		/* 1-bit counter that shows up in panel thread timeline */
		sde_atrace('C', pdata->thread, "TE_VSYNC",
			   atomic_inc_return(&pdata->te_counter) & 1);
	}
}

static void panel_switch_cmd_set_transfer(struct panel_switch_data *pdata,
					  const struct dsi_display_mode *mode)
{
	struct dsi_panel *panel = pdata->panel;
	struct dsi_panel_cmd_set *cmd;
	int rc;

	cmd = &mode->priv_info->cmd_sets[DSI_CMD_SET_TIMING_SWITCH];

	rc = dsi_panel_cmd_set_transfer(panel, cmd);
	if (rc)
		pr_warn("failed to send TIMING switch cmd, rc=%d\n", rc);
}

static void panel_switch_to_mode(struct panel_switch_data *pdata,
				 const struct dsi_display_mode *mode)
{
	SDE_ATRACE_BEGIN(__func__);
	if (pdata->funcs && pdata->funcs->perform_switch)
		pdata->funcs->perform_switch(pdata, mode);

	if (pdata->switch_pending) {
		pdata->switch_pending = false;
		wake_up_all(&pdata->switch_wq);
	}

	SDE_ATRACE_END(__func__);
}

static void panel_switch_worker(struct kthread_work *work)
{
	struct panel_switch_data *pdata;
	struct dsi_panel *panel;
	struct dsi_display *display;
	const struct dsi_display_mode *mode;
	const unsigned long timeout = msecs_to_jiffies(TE_TIMEOUT_MS);
	int rc;
	u32 te_listen_cnt;

	if (unlikely(!work))
		return;

	pdata = container_of(work, struct panel_switch_data, switch_work);
	panel = pdata->panel;
	if (unlikely(!panel))
		return;

	display = dsi_panel_to_display(panel);
	if (unlikely(!display))
		return;

	mutex_lock(&panel->panel_lock);
	mode = pdata->display_mode;
	if (unlikely(!mode)) {
		mutex_unlock(&panel->panel_lock);
		return;
	}

	SDE_ATRACE_BEGIN(__func__);

	pr_debug("switching mode to %dhz\n", mode->timing.refresh_rate);

	te_listen_cnt = pdata->switch_te_listen_count;
	if (te_listen_cnt) {
		reinit_completion(&pdata->te_completion);
		dsi_display_add_te_listener(display, &pdata->te_listener);
	}

	/* switch is shadowed by vsync so this can be done ahead of TE */
	panel_switch_to_mode(pdata, mode);
	mutex_unlock(&panel->panel_lock);

	if (te_listen_cnt) {
		rc = wait_for_completion_timeout(&pdata->te_completion,
						 timeout);
		if (!rc)
			pr_warn("Timed out waiting for TE while switching!\n");
		else
			pr_debug("TE received after %dus\n",
				jiffies_to_usecs(timeout - rc));
	}

	sde_atrace_mode_fps(pdata, mode);
	SDE_ATRACE_END(__func__);

	/*
	 * this is meant only for debugging purposes, keep TE enabled for a few
	 * extra frames to see how they align after switch
	 */
	if (te_listen_cnt) {
		te_listen_cnt--;
		pr_debug("waiting for %d extra te\n", te_listen_cnt);
		while (rc && te_listen_cnt) {
			rc = wait_for_completion_timeout(&pdata->te_completion,
							timeout);
			te_listen_cnt--;
		}
		dsi_display_remove_te_listener(display, &pdata->te_listener);
	}
}

static bool dsi_mode_matches_cmdline(const struct dsi_display_mode *dm,
				     const struct drm_cmdline_mode *cm)
{

	if (!cm->refresh_specified && !cm->specified)
		return false;
	if (cm->refresh_specified && cm->refresh != dm->timing.refresh_rate)
		return false;
	if (cm->specified && (cm->xres != dm->timing.h_active ||
			      cm->yres != dm->timing.v_active))
		return false;
	return true;
}

static const struct dsi_display_mode *
display_mode_from_cmdline(const struct dsi_panel *panel,
			  const char *modestr)
{
	const struct dsi_display *display;
	const struct dsi_display_mode *mode;
	struct drm_cmdline_mode cm = {0};
	int i;

	display = dsi_panel_to_display(panel);
	if (!display)
		return ERR_PTR(-ENODEV);

	if (!drm_mode_parse_command_line_for_connector(modestr,
						       display->drm_conn,
						       &cm))
		return ERR_PTR(-EINVAL);

	for_each_display_mode(i, mode, panel) {
		if (dsi_mode_matches_cmdline(mode, &cm))
			return mode;
	}

	return NULL;
}

static const struct dsi_display_mode *
display_mode_from_user(const struct dsi_panel *panel,
		       const char __user *user_buf, size_t user_len)
{
	char modestr[40];
	size_t len = min(user_len, sizeof(modestr) - 1);
	int rc;

	rc = copy_from_user(modestr, user_buf, len);
	if (rc)
		return ERR_PTR(-EFAULT);

	modestr[len] = '\0';

	return display_mode_from_cmdline(panel, strim(modestr));
}

static void panel_queue_switch(struct panel_switch_data *pdata,
			       const struct dsi_display_mode *new_mode)
{
	if (unlikely(!pdata || !pdata->panel || !new_mode))
		return;

	kthread_flush_work(&pdata->switch_work);

	mutex_lock(&pdata->panel->panel_lock);
	pdata->display_mode = new_mode;
	pdata->switch_pending = true;
	mutex_unlock(&pdata->panel->panel_lock);

	kthread_queue_work(&pdata->worker, &pdata->switch_work);
}

static int panel_switch(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata = panel->private_data;

	if (!panel->cur_mode)
		return -EINVAL;

	SDE_ATRACE_BEGIN(__func__);
	panel_queue_switch(pdata, panel->cur_mode);
	SDE_ATRACE_END(__func__);

	return 0;
}

static int panel_pre_kickoff(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata = panel->private_data;
	const unsigned long timeout = msecs_to_jiffies(TE_TIMEOUT_MS);

	if (!wait_event_timeout(pdata->switch_wq,
				!pdata->switch_pending, timeout))
		pr_warn("Timed out waiting for panel switch\n");

	return 0;
}

static int panel_pre_disable(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata = panel->private_data;

	kthread_flush_worker(&pdata->worker);

	return 0;
}

static int panel_idle(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata = panel->private_data;
	const struct dsi_display_mode *idle_mode;

	if (unlikely(!pdata))
		return -EINVAL;

	kthread_flush_work(&pdata->switch_work);

	mutex_lock(&panel->panel_lock);
	idle_mode = pdata->idle_mode;
	if (idle_mode && !is_display_mode_same(idle_mode, panel->cur_mode)) {
		/*
		 * clocks are going to be turned off right after this call, so
		 * switch needs to happen synchronously
		 */
		panel_switch_to_mode(pdata, idle_mode);
		pdata->display_mode = idle_mode;

		sde_atrace_mode_fps(pdata, idle_mode);
	}
	mutex_unlock(&panel->panel_lock);

	SDE_ATRACE_INT("display_idle", 1);

	return 0;
}

static int panel_wakeup(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata = panel->private_data;
	const struct dsi_display_mode *mode = NULL;

	if (unlikely(!pdata))
		return -EINVAL;

	mutex_lock(&panel->panel_lock);
	if (!is_display_mode_same(pdata->display_mode, panel->cur_mode))
		mode = panel->cur_mode;
	mutex_unlock(&panel->panel_lock);

	if (mode)
		panel_queue_switch(pdata, mode);

	SDE_ATRACE_INT("display_idle", 0);

	return 0;
}

static ssize_t debugfs_panel_switch_mode_write(struct file *file,
					       const char __user *user_buf,
					       size_t user_len,
					       loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct panel_switch_data *pdata = seq->private;
	const struct dsi_display_mode *mode;

	if (!pdata->panel || !dsi_panel_initialized(pdata->panel))
		return -ENOENT;

	mode = display_mode_from_user(pdata->panel, user_buf, user_len);
	if (IS_ERR(mode))
		return PTR_ERR(mode);
	else if (!mode)
		return -ENOENT;

	panel_queue_switch(pdata, mode);

	return user_len;
}

static int debugfs_panel_switch_mode_read(struct seq_file *seq, void *data)
{
	struct panel_switch_data *pdata = seq->private;
	const struct dsi_display_mode *mode;

	if (unlikely(!pdata->panel))
		return -ENOENT;

	mutex_lock(&pdata->panel->panel_lock);
	mode = pdata->display_mode;
	mutex_unlock(&pdata->panel->panel_lock);

	if (mode)
		seq_printf(seq, "%dx%d@%d\n", mode->timing.h_active,
			   mode->timing.v_active, mode->timing.refresh_rate);
	else
		seq_puts(seq, "unknown");

	return 0;
}

static int debugfs_panel_switch_mode_open(struct inode *inode, struct file *f)
{
	return single_open(f, debugfs_panel_switch_mode_read, inode->i_private);
}

static const struct file_operations panel_switch_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_panel_switch_mode_open,
	.write = debugfs_panel_switch_mode_write,
	.read = seq_read,
	.release = single_release,
};

static ssize_t sysfs_idle_mode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	const struct dsi_display *display;
	struct panel_switch_data *pdata;
	const struct dsi_display_mode *mode = NULL;
	ssize_t rc;

	display = dev_get_drvdata(dev);
	if (unlikely(!display || !display->panel ||
		     !display->panel->private_data))
		return -EINVAL;

	pdata = display->panel->private_data;

	mutex_lock(&pdata->panel->panel_lock);
	mode = pdata->idle_mode;
	mutex_unlock(&pdata->panel->panel_lock);

	if (mode)
		rc = snprintf(buf, PAGE_SIZE, "%dx%d@%d\n",
			      mode->timing.h_active, mode->timing.v_active,
			      mode->timing.refresh_rate);
	else
		rc = snprintf(buf, PAGE_SIZE, "none\n");

	return rc;
}
static ssize_t sysfs_idle_mode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	const struct dsi_display *display;
	struct panel_switch_data *pdata;
	const struct dsi_display_mode *mode = NULL;

	display = dev_get_drvdata(dev);
	if (unlikely(!display || !display->panel ||
		     !display->panel->private_data))
		return -EINVAL;

	pdata = display->panel->private_data;
	if (count > 1) {
		char *modestr = kstrndup(buf, count, GFP_KERNEL);

		/* remove any trailing lf at end of sysfs input */
		mode = display_mode_from_cmdline(display->panel,
						 strim(modestr));
		kfree(modestr);

		if (IS_ERR(mode))
			return PTR_ERR(mode);
	}
	mutex_lock(&display->panel->panel_lock);
	pdata->idle_mode = mode;
	mutex_unlock(&display->panel->panel_lock);

	return count;
}

static DEVICE_ATTR(idle_mode, 0644,
		   sysfs_idle_mode_show,
		   sysfs_idle_mode_store);

static struct attribute *panel_switch_sysfs_attrs[] = {
	&dev_attr_idle_mode.attr,
	NULL,
};

static struct attribute_group panel_switch_sysfs_attrs_group = {
	.attrs = panel_switch_sysfs_attrs,
};

static const struct dsi_panel_funcs panel_funcs = {
	.mode_switch = panel_switch,
	.pre_disable = panel_pre_disable,
	.pre_kickoff = panel_pre_kickoff,
	.idle        = panel_idle,
	.wakeup      = panel_wakeup,
};

static int panel_switch_data_init(struct dsi_panel *panel,
				  struct panel_switch_data *pdata)
{
	struct sched_param param = {
		.sched_priority = 16,
	};
	const struct dsi_display *display;

	display = dsi_panel_to_display(panel);
	if (unlikely(!display))
		return -ENOENT;

	kthread_init_work(&pdata->switch_work, panel_switch_worker);
	kthread_init_worker(&pdata->worker);
	pdata->thread = kthread_run(kthread_worker_fn, &pdata->worker, "panel");
	if (IS_ERR_OR_NULL(pdata->thread))
		return -EFAULT;

	pdata->panel = panel;
	pdata->te_listener.handler = panel_handle_te;
	pdata->display_mode = panel->cur_mode;

	sched_setscheduler(pdata->thread, SCHED_FIFO, &param);
	init_completion(&pdata->te_completion);
	init_waitqueue_head(&pdata->switch_wq);
	atomic_set(&pdata->te_counter, 0);

	panel->private_data = pdata;
	panel->funcs = &panel_funcs;

	pdata->debug_root = debugfs_create_dir("switch", display->root);
	debugfs_create_file("mode", 0600, pdata->debug_root, pdata,
			    &panel_switch_fops);
	debugfs_create_u32("te_listen_count", 0600, pdata->debug_root,
			    &pdata->switch_te_listen_count);
	debugfs_create_atomic_t("te_counter", 0600, pdata->debug_root,
				&pdata->te_counter);

	sysfs_create_group(&panel->parent->kobj,
			   &panel_switch_sysfs_attrs_group);

	return 0;
}

static void panel_switch_data_deinit(struct panel_switch_data *pdata)
{
	kthread_flush_worker(&pdata->worker);
	kthread_stop(pdata->thread);
	sysfs_remove_group(&pdata->panel->parent->kobj,
			   &panel_switch_sysfs_attrs_group);
}

static void panel_switch_data_destroy(struct panel_switch_data *pdata)
{
	panel_switch_data_deinit(pdata);
	if (pdata->panel && pdata->panel->parent)
		devm_kfree(pdata->panel->parent, pdata);
}

static struct panel_switch_data *panel_switch_create(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata;
	int rc;

	pdata = devm_kzalloc(panel->parent, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	rc = panel_switch_data_init(panel, pdata);
	if (rc)
		return ERR_PTR(rc);

	return pdata;
}

const struct panel_switch_funcs panel_switch_default_funcs = {
	.create = panel_switch_create,
	.destroy = panel_switch_data_destroy,
	.perform_switch = panel_switch_cmd_set_transfer,
};

int dsi_panel_switch_init(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata = NULL;
	const struct panel_switch_funcs *funcs = &panel_switch_default_funcs;

	if (funcs->create)
		pdata = funcs->create(panel);

	if (IS_ERR_OR_NULL(pdata))
		return -ENOENT;

	pdata->funcs = funcs;

	return 0;
}

/*
 * This should be called without panel_lock as flush/wait on worker can
 * deadlock while holding it
 */
void dsi_panel_switch_destroy(struct dsi_panel *panel)
{
	struct panel_switch_data *pdata = panel->private_data;

	if (pdata && pdata->funcs && pdata->funcs->destroy)
		pdata->funcs->destroy(pdata);
}

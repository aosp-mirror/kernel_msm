/* Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
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

#define CREATE_TRACE_POINTS
#include "msm_vidc_debug.h"
#include "vidc_hfi_api.h"

#define MAX_DBG_BUF_SIZE 4096
int msm_vidc_debug = VIDC_ERR | VIDC_WARN;
int msm_vidc_debug_out = VIDC_OUT_PRINTK;
int msm_fw_debug = 0x18;
int msm_fw_debug_mode = 0x1;
int msm_fw_low_power_mode = 0x1;
int msm_vidc_hw_rsp_timeout = 1000;
u32 msm_fw_coverage = 0x0;
int msm_vidc_vpe_csc_601_to_709 = 0x0;
int msm_vidc_dec_dcvs_mode = 0x1;
int msm_vidc_enc_dcvs_mode = 0x1;
int msm_vidc_sys_idle_indicator = 0x0;
u32 msm_vidc_firmware_unload_delay = 15000;
int msm_vidc_thermal_mitigation_disabled = 0x0;

static struct mutex debugfs_lock;

#define DYNAMIC_BUF_OWNER(__binfo) ({ \
	atomic_read(&__binfo->ref_count) == 2 ? "video driver" : "firmware";\
})

static int vidc_debug_core_show(struct seq_file *s, void *unused)
{
	struct msm_vidc_core *core = s->private;
	struct hfi_device *hdev;
	struct hal_fw_info fw_info;
	int i = 0, rc = 0;

	if (!core || !core->device) {
		dprintk(VIDC_ERR, "Invalid params, core: %pK\n", core);
		return 0;
	}

	mutex_lock(&debugfs_lock);
	hdev = core->device;

	seq_puts(s, "===============================\n");
	seq_printf(s,  "CORE %d: %pK\n", core->id, core);
	seq_puts(s,  "===============================\n");
	seq_printf(s,  "Core state: %d\n", core->state);
	rc = call_hfi_op(hdev, get_fw_info, hdev->hfi_device_data, &fw_info);
	if (rc) {
		dprintk(VIDC_WARN, "Failed to read FW info\n");
		goto err_fw_info;
	}

	seq_printf(s,  "FW version : %s\n", fw_info.version);
	seq_printf(s,  "base addr: %pa\n", &fw_info.base_addr);
	seq_printf(s,  "register_base: %pa\n", &fw_info.register_base);
	seq_printf(s,  "register_size: %u\n", fw_info.register_size);
	seq_printf(s,  "irq: %u\n", fw_info.irq);

err_fw_info:
	for (i = SYS_MSG_START; i < SYS_MSG_END; i++) {
		seq_printf(s, "completions[%d]: %s\n", i,
			completion_done(&core->completions[SYS_MSG_INDEX(i)]) ?
			"pending" : "done");
	}
	mutex_unlock(&debugfs_lock);
	return 0;
}

static int vidc_debug_core_open(struct inode *inode, struct file *file)
{
	return single_open(file, vidc_debug_core_show, inode->i_private);
}

static const struct file_operations core_info_fops = {
	.open = vidc_debug_core_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int trigger_ssr_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t trigger_ssr_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos) {
	u32 ssr_trigger_val;
	int rc;
	struct msm_vidc_core *core = filp->private_data;
	rc = sscanf(buf, "%d", &ssr_trigger_val);
	if (rc < 0) {
		dprintk(VIDC_WARN, "returning error err %d\n", rc);
		rc = -EINVAL;
	} else {
		msm_vidc_trigger_ssr(core, ssr_trigger_val);
		rc = count;
	}
	return rc;
}

static const struct file_operations ssr_fops = {
	.open = trigger_ssr_open,
	.write = trigger_ssr_write,
};

struct dentry *msm_vidc_debugfs_init_drv(void)
{
	struct dentry *dir = NULL;

	mutex_init(&debugfs_lock);
	dir = debugfs_create_dir("msm_vidc", NULL);
	if (IS_ERR_OR_NULL(dir)) {
		dir = NULL;
		goto failed_create_dir;
	}

	if (!debugfs_create_u32("debug_level", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_debug)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("fw_level", S_IRUGO | S_IWUSR,
			dir, &msm_fw_debug)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("fw_debug_mode", S_IRUGO | S_IWUSR,
			dir, &msm_fw_debug_mode)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("fw_coverage", S_IRUGO | S_IWUSR,
			dir, &msm_fw_coverage)) {
		dprintk(VIDC_WARN, "debugfs_create_file fw_coverage: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("dcvs_dec_mode", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_dec_dcvs_mode)) {
		dprintk(VIDC_WARN, "debugfs_create_file dcvs_dec_mode: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("dcvs_enc_mode", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_enc_dcvs_mode)) {
		dprintk(VIDC_WARN, "debugfs_create_file dcvs_enc_mode: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("fw_low_power_mode", S_IRUGO | S_IWUSR,
			dir, &msm_fw_low_power_mode)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("debug_output", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_debug_out)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("hw_rsp_timeout", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_hw_rsp_timeout)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_bool("enable_vpe_csc_601_709", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_vpe_csc_601_to_709)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_bool("sys_idle_indicator", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_sys_idle_indicator)) {
		dprintk(VIDC_ERR,
			"debugfs_create_file: sys_idle_indicator fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("firmware_unload_delay", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_firmware_unload_delay)) {
		dprintk(VIDC_ERR,
			"debugfs_create_file: firmware_unload_delay fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_u32("disable_thermal_mitigation", S_IRUGO | S_IWUSR,
			dir, &msm_vidc_thermal_mitigation_disabled)) {
		dprintk(VIDC_ERR,
			"debugfs_create_file: disable_thermal_mitigation fail\n");
		goto failed_create_dir;
	}
	return dir;

failed_create_dir:
	if (dir)
		debugfs_remove_recursive(vidc_driver->debugfs_root);

	return NULL;
}

void msm_vidc_debugfs_deinit_drv(void)
{
	mutex_destroy(&debugfs_lock);
}

struct dentry *msm_vidc_debugfs_init_core(struct msm_vidc_core *core,
		struct dentry *parent)
{
	struct dentry *dir = NULL;
	char debugfs_name[MAX_DEBUGFS_NAME];
	if (!core) {
		dprintk(VIDC_ERR, "Invalid params, core: %pK\n", core);
		goto failed_create_dir;
	}

	snprintf(debugfs_name, MAX_DEBUGFS_NAME, "core%d", core->id);
	dir = debugfs_create_dir(debugfs_name, parent);
	if (!dir) {
		dprintk(VIDC_ERR, "Failed to create debugfs for msm_vidc\n");
		goto failed_create_dir;
	}

	if (!debugfs_create_file("info", S_IRUGO, dir, core, &core_info_fops)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_file("trigger_ssr", S_IWUSR,
			dir, core, &ssr_fops)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
failed_create_dir:
	return dir;
}

static struct msm_vidc_inst *get_inst(struct msm_vidc_inst *inst)
{
	struct msm_vidc_core *core = NULL;
	struct msm_vidc_inst *temp, *ret = NULL;

	if (!inst)
		return NULL;

	mutex_lock(&vidc_driver->lock);
	list_for_each_entry(core, &vidc_driver->cores, list) {
		mutex_lock(&core->lock);
		list_for_each_entry(temp, &core->instances, list)
			if (temp == inst) {
				ret = inst;
				break;
			}
		mutex_unlock(&core->lock);
		if (ret != NULL)
			break;
	}
	mutex_unlock(&vidc_driver->lock);
	return ret;
}

static int publish_unreleased_reference(struct msm_vidc_inst *inst,
		struct seq_file *s)
{
	struct buffer_info *temp = NULL;

	if (!inst) {
		dprintk(VIDC_ERR, "%s: invalid param\n", __func__);
		return -EINVAL;
	}

	if (inst->buffer_mode_set[CAPTURE_PORT] == HAL_BUFFER_MODE_DYNAMIC) {
		seq_puts(s, "Pending buffer references\n");

		mutex_lock(&inst->registeredbufs.lock);
		list_for_each_entry(temp, &inst->registeredbufs.list, list) {
			if (temp->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
			!temp->inactive && atomic_read(&temp->ref_count)) {
				seq_printf(s,
					"\tpending buffer: %pad fd[0] = %d ref_count = %d held by: %s\n",
					&temp->device_addr[0],
					temp->fd[0],
					atomic_read(&temp->ref_count),
					DYNAMIC_BUF_OWNER(temp));
			}
		}
		mutex_unlock(&inst->registeredbufs.lock);
	}

	return 0;
}

static int vidc_debug_inst_show(struct seq_file *s, void *unused)
{
	struct msm_vidc_inst *inst = get_inst(s->private);
	int i, j;

	if (!inst) {
		dprintk(VIDC_ERR, "Instance deleted\n");
		return -EINVAL;
	}

	mutex_lock(&debugfs_lock);
	seq_puts(s, "==============================\n");
	seq_printf(s, "INSTANCE: %pK (%s)\n", inst,
		inst->session_type == MSM_VIDC_ENCODER ? "Encoder" : "Decoder");
	seq_puts(s, "==============================\n");
	seq_printf(s, "core: %pK\n", inst->core);
	seq_printf(s, "height: %d\n", inst->prop.height[CAPTURE_PORT]);
	seq_printf(s, "width: %d\n", inst->prop.width[CAPTURE_PORT]);
	seq_printf(s, "fps: %d\n", inst->prop.fps);
	seq_printf(s, "state: %d\n", inst->state);
	seq_printf(s, "secure: %d\n", !!(inst->flags & VIDC_SECURE));
	seq_puts(s, "-----------Formats-------------\n");
	for (i = 0; i < MAX_PORT_NUM; i++) {
		seq_printf(s,  "capability: %s\n",
			i == OUTPUT_PORT ? "Output" : "Capture");
		seq_printf(s, "name : %s\n", inst->fmts[i]->name);
		seq_printf(s, "planes : %d\n", inst->fmts[i]->num_planes);
		seq_printf(s, "type: %s\n", inst->fmts[i]->type == OUTPUT_PORT ?
			"Output" : "Capture");

		switch (inst->buffer_mode_set[i]) {
		case HAL_BUFFER_MODE_STATIC:
			seq_printf(s, "buffer mode : %s\n", "static");
			break;
		case HAL_BUFFER_MODE_RING:
			seq_printf(s, "buffer mode : %s\n", "ring");
			break;
		case HAL_BUFFER_MODE_DYNAMIC:
			seq_printf(s, "buffer mode : %s\n", "dynamic");
			break;
		default:
			seq_puts(s, "buffer mode : unsupported\n");
		}

		 seq_printf(s, "count: %u\n",
				inst->bufq[i].vb2_bufq.num_buffers);

		for (j = 0; j < inst->fmts[i]->num_planes; j++)
			seq_printf(s, "size for plane %d: %u\n", j,
			inst->bufq[i].vb2_bufq.plane_sizes[j]);

		if (i < MAX_PORT_NUM - 1)
			seq_puts(s, "\n");
	}
	seq_puts(s,  "-------------------------------\n");
	for (i = SESSION_MSG_START; i < SESSION_MSG_END; i++) {
		seq_printf(s,  "completions[%d]: %s\n", i,
		completion_done(&inst->completions[SESSION_MSG_INDEX(i)]) ?
		"pending" : "done");
	}

	seq_printf(s, "ETB Count: %d\n", inst->count.etb);
	seq_printf(s, "EBD Count: %d\n", inst->count.ebd);
	seq_printf(s, "FTB Count: %d\n", inst->count.ftb);
	seq_printf(s,  "FBD Count: %d\n", inst->count.fbd);

	publish_unreleased_reference(inst, s);
	mutex_unlock(&debugfs_lock);
	return 0;
}

static int vidc_debug_inst_open(struct inode *inode, struct file *file)
{
	return single_open(file, vidc_debug_inst_show, inode->i_private);
}

static const struct file_operations inst_info_fops = {
	.open = vidc_debug_inst_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

struct dentry *msm_vidc_debugfs_init_inst(struct msm_vidc_inst *inst,
		struct dentry *parent)
{
	struct dentry *dir = NULL, *info = NULL;
	char debugfs_name[MAX_DEBUGFS_NAME];

	if (!inst) {
		dprintk(VIDC_ERR, "Invalid params, inst: %p\n", inst);
		goto exit;
	}
	snprintf(debugfs_name, MAX_DEBUGFS_NAME, "inst_%pK", inst);

	if (!dir) {
		dprintk(VIDC_ERR, "Failed to create debugfs for msm_vidc\n");
		goto exit;
	}

	info = debugfs_create_file("info", 0444, dir,
			inst, &inst_info_fops);
	if (!info) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_file;
	}

	dir->d_inode->i_private = info->d_inode->i_private;
	inst->debug.pdata[FRAME_PROCESSING].sampling = true;
	return dir;

failed_create_file:
	debugfs_remove_recursive(dir);
	dir = NULL;
exit:
	return dir;
}

void msm_vidc_debugfs_update(struct msm_vidc_inst *inst,
	enum msm_vidc_debugfs_event e)
{
	struct msm_vidc_debug *d = &inst->debug;
	char a[64] = "Frame processing";
	switch (e) {
	case MSM_VIDC_DEBUGFS_EVENT_ETB:
		mutex_lock(&inst->lock);
		inst->count.etb++;
		mutex_unlock(&inst->lock);
		if (inst->count.ebd && inst->count.ftb > inst->count.fbd) {
			d->pdata[FRAME_PROCESSING].name[0] = '\0';
			tic(inst, FRAME_PROCESSING, a);
		}
	break;
	case MSM_VIDC_DEBUGFS_EVENT_EBD:
		mutex_lock(&inst->lock);
		inst->count.ebd++;
		mutex_unlock(&inst->lock);
		if (inst->count.ebd && inst->count.ebd == inst->count.etb) {
			toc(inst, FRAME_PROCESSING);
			dprintk(VIDC_PROF, "EBD: FW needs input buffers\n");
		}
		if (inst->count.ftb == inst->count.fbd)
			dprintk(VIDC_PROF, "EBD: FW needs output buffers\n");
	break;
	case MSM_VIDC_DEBUGFS_EVENT_FTB: {
		inst->count.ftb++;
		if (inst->count.ebd && inst->count.etb > inst->count.ebd) {
			d->pdata[FRAME_PROCESSING].name[0] = '\0';
			tic(inst, FRAME_PROCESSING, a);
		}
	}
	break;
	case MSM_VIDC_DEBUGFS_EVENT_FBD:
		inst->debug.samples++;
		if (inst->count.ebd && inst->count.fbd == inst->count.ftb) {
			toc(inst, FRAME_PROCESSING);
			dprintk(VIDC_PROF, "FBD: FW needs output buffers\n");
		}
		if (inst->count.etb == inst->count.ebd)
			dprintk(VIDC_PROF, "FBD: FW needs input buffers\n");
		break;
	default:
		dprintk(VIDC_ERR, "Invalid state in debugfs: %d\n", e);
		break;
	}
}


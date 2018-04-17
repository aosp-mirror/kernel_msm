/* Copyright (c) 2013-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/of_device.h>
#include <linux/qcom_iommu.h>
#include <linux/sched_clock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>

#include "msm_isp.h"
#include "msm_isp_util.h"
#include "msm_isp_axi_util.h"
#include "msm_isp_stats_util.h"
#include "msm_sd.h"
#include "msm_isp40.h"
#include "msm_isp32.h"

static const struct of_device_id msm_vfe_dt_match[] = {
	{
		.compatible = "qcom,vfe40",
		.data = &vfe40_hw_info,
	},
	{
		.compatible = "qcom,vfe32",
		.data = &vfe32_hw_info,
	},
	{}
};

MODULE_DEVICE_TABLE(of, msm_vfe_dt_match);

static const struct platform_device_id msm_vfe_dev_id[] = {
	{"msm_vfe32", (kernel_ulong_t) &vfe32_hw_info},
	{}
};
#define MAX_OVERFLOW_COUNTERS  29
#define OVERFLOW_LENGTH 1024
#define OVERFLOW_BUFFER_LENGTH 64
static char stat_line[OVERFLOW_LENGTH];

struct msm_isp_statistics stats;
struct msm_isp_ub_info ub_info;
static char *stats_str[MAX_OVERFLOW_COUNTERS] = {
	"imgmaster0_overflow_cnt",
	"imgmaster1_overflow_cnt",
	"imgmaster2_overflow_cnt",
	"imgmaster3_overflow_cnt",
	"imgmaster4_overflow_cnt",
	"imgmaster5_overflow_cnt",
	"imgmaster6_overflow_cnt",
	"be_overflow_cnt",
	"bg_overflow_cnt",
	"bf_overflow_cnt",
	"awb_overflow_cnt",
	"rs_overflow_cnt",
	"cs_overflow_cnt",
	"ihist_overflow_cnt",
	"skinbhist_overflow_cnt",
	"bfscale_overflow_cnt",
	"ISP_VFE0_client_info.active",
	"ISP_VFE0_client_info.ab",
	"ISP_VFE0_client_info.ib",
	"ISP_VFE1_client_info.active",
	"ISP_VFE1_client_info.ab",
	"ISP_VFE1_client_info.ib",
	"ISP_CPP_client_info.active",
	"ISP_CPP_client_info.ab",
	"ISP_CPP_client_info.ib",
	"ISP_last_overflow.ab",
	"ISP_last_overflow.ib",
	"ISP_VFE_CLK_RATE",
	"ISP_CPP_CLK_RATE",
};

#define MAX_DEPTH_BW_REQ_HISTORY 25
#define MAX_BW_HISTORY_BUFF_LEN  6144
#define MAX_BW_HISTORY_LINE_BUFF_LEN 512

#define MAX_UB_INFO_BUFF_LEN  1024
#define MAX_UB_INFO_LINE_BUFF_LEN 256

static struct msm_isp_bw_req_info
		msm_isp_bw_request_history[MAX_DEPTH_BW_REQ_HISTORY];
static int msm_isp_bw_request_history_idx;
static char bw_request_history_buff[MAX_BW_HISTORY_BUFF_LEN];
static char ub_info_buffer[MAX_UB_INFO_BUFF_LEN];
static spinlock_t req_history_lock;
static int vfe_debugfs_statistics_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t vfe_debugfs_statistics_read(struct file *t_file, char *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	uint64_t *ptr;
	char buffer[OVERFLOW_BUFFER_LENGTH] = {0};
	struct vfe_device *vfe_dev = (struct vfe_device *)
		t_file->private_data;
	struct msm_isp_statistics *stats = vfe_dev->stats;

	memset(stat_line, 0, sizeof(stat_line));
	msm_isp_util_get_bandwidth_stats(vfe_dev, stats);
	ptr = (uint64_t *)(stats);
	for (i = 0; i < MAX_OVERFLOW_COUNTERS; i++) {
		strlcat(stat_line, stats_str[i], sizeof(stat_line));
		strlcat(stat_line, "     ", sizeof(stat_line));
		snprintf(buffer, sizeof(buffer), "%llu", ptr[i]);
		strlcat(stat_line, buffer, sizeof(stat_line));
		strlcat(stat_line, "\r\n", sizeof(stat_line));
	}
	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, stat_line, strlen(stat_line));
}

static ssize_t vfe_debugfs_statistics_write(struct file *t_file,
	const char *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	struct vfe_device *vfe_dev = (struct vfe_device *)
		t_file->private_data;
	struct msm_isp_statistics *stats = vfe_dev->stats;
	memset(stats, 0, sizeof(struct msm_isp_statistics));

	return sizeof(struct msm_isp_statistics);
}

static int bw_history_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t bw_history_read(struct file *t_file, char *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	char *out_buffer = bw_request_history_buff;
	char line_buffer[MAX_BW_HISTORY_LINE_BUFF_LEN] = {0};
	struct msm_isp_bw_req_info *isp_req_hist =
		(struct msm_isp_bw_req_info *) t_file->private_data;

	memset(out_buffer, 0, MAX_BW_HISTORY_BUFF_LEN);

	snprintf(line_buffer, sizeof(line_buffer),
		"Bus bandwidth request history in chronological order:\n");
	strlcat(out_buffer, line_buffer, sizeof(bw_request_history_buff));

	snprintf(line_buffer, sizeof(line_buffer),
		"MSM_ISP_MIN_AB = %u, MSM_ISP_MIN_IB = %u\n\n",
		MSM_ISP_MIN_AB, MSM_ISP_MIN_IB);
	strlcat(out_buffer, line_buffer, sizeof(bw_request_history_buff));

	for (i = 0; i < MAX_DEPTH_BW_REQ_HISTORY; i++) {
		snprintf(line_buffer, sizeof(line_buffer),
		 "idx = %d, client = %u, timestamp = %llu, ab = %llu, ib = %llu\n"
		 "ISP0.active = %x, ISP0.ab = %llu, ISP0.ib = %llu\n"
		 "ISP1.active = %x, ISP1.ab = %llu, ISP1.ib = %llu\n"
		 "CPP.active = %x, CPP.ab = %llu, CPP.ib = %llu\n\n",
		 i, isp_req_hist[i].client, isp_req_hist[i].timestamp,
		 isp_req_hist[i].total_ab, isp_req_hist[i].total_ib,
		 isp_req_hist[i].client_info[0].active,
		 isp_req_hist[i].client_info[0].ab,
		 isp_req_hist[i].client_info[0].ib,
		 isp_req_hist[i].client_info[1].active,
		 isp_req_hist[i].client_info[1].ab,
		 isp_req_hist[i].client_info[1].ib,
		 isp_req_hist[i].client_info[2].active,
		 isp_req_hist[i].client_info[2].ab,
		 isp_req_hist[i].client_info[2].ib);
		strlcat(out_buffer, line_buffer,
		sizeof(bw_request_history_buff));
	}
	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, strlen(out_buffer));
}

static ssize_t bw_history_write(struct file *t_file,
	const char *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	struct msm_isp_bw_req_info *isp_req_hist =
		(struct msm_isp_bw_req_info *) t_file->private_data;

	memset(isp_req_hist, 0, sizeof(msm_isp_bw_request_history));
	msm_isp_bw_request_history_idx = 0;
	return sizeof(msm_isp_bw_request_history);
}

static int ub_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t ub_info_read(struct file *t_file, char *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	char *out_buffer = ub_info_buffer;
	char line_buffer[MAX_UB_INFO_LINE_BUFF_LEN] = {0};
	struct vfe_device *vfe_dev =
		(struct vfe_device *) t_file->private_data;
	struct msm_isp_ub_info *ub_info = vfe_dev->ub_info;

	memset(out_buffer, 0, MAX_UB_INFO_LINE_BUFF_LEN);
	snprintf(line_buffer, sizeof(line_buffer),
		"wm_ub_policy_type = %d\n"
		"num_wm = %d\n"
		"wm_ub = %d\n",
		ub_info->policy, ub_info->num_wm, ub_info->wm_ub);
	strlcat(out_buffer, line_buffer,
	    sizeof(ub_info_buffer));
	for (i = 0; i < ub_info->num_wm; i++) {
		snprintf(line_buffer, sizeof(line_buffer),
			"data[%d] = 0x%x, addr[%d] = 0x%llx\n",
			i, ub_info->data[i], i, ub_info->addr[i]);
		strlcat(out_buffer, line_buffer,
			sizeof(ub_info_buffer));
	}

	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, strlen(out_buffer));
}

static ssize_t ub_info_write(struct file *t_file,
	const char *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	struct vfe_device *vfe_dev =
		(struct vfe_device *) t_file->private_data;
	struct msm_isp_ub_info *ub_info = vfe_dev->ub_info;

	memset(ub_info, 0, sizeof(struct msm_isp_ub_info));

	return sizeof(struct msm_isp_ub_info);
}

static const struct file_operations vfe_debugfs_error = {
	.open = vfe_debugfs_statistics_open,
	.read = vfe_debugfs_statistics_read,
	.write = vfe_debugfs_statistics_write,
};

static const struct file_operations bw_history_ops = {
	.open = bw_history_open,
	.read = bw_history_read,
	.write = bw_history_write,
};

static const struct file_operations ub_info_ops = {
	.open = ub_info_open,
	.read = ub_info_read,
	.write = ub_info_write,
};

void msm_isp_update_req_history(uint32_t client, uint64_t ab,
				 uint64_t ib,
				 struct msm_isp_bandwidth_info *client_info,
				 unsigned long long ts)
{
	int i;

	spin_lock(&req_history_lock);
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].client =
		client;
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].timestamp =
		ts;
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].total_ab =
		ab;
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].total_ib =
		ib;

	for (i = 0; i < MAX_ISP_CLIENT; i++) {
		msm_isp_bw_request_history[msm_isp_bw_request_history_idx].
			client_info[i].active = client_info[i].active;
		msm_isp_bw_request_history[msm_isp_bw_request_history_idx].
			client_info[i].ab = client_info[i].ab;
		msm_isp_bw_request_history[msm_isp_bw_request_history_idx].
			client_info[i].ib = client_info[i].ib;
	}

	msm_isp_bw_request_history_idx = (msm_isp_bw_request_history_idx + 1)
			 % MAX_DEPTH_BW_REQ_HISTORY;
	spin_unlock(&req_history_lock);
}

static int vfe_probe(struct platform_device *pdev)
{
	struct vfe_device *vfe_dev;
	/*struct msm_cam_subdev_info sd_info;*/
	const struct of_device_id *match_dev;
	int rc = 0;
	pr_err("%s: CAMERA \n", __func__);
	vfe_dev = kzalloc(sizeof(struct vfe_device), GFP_KERNEL);
	if (!vfe_dev) {
		pr_err("%s: no enough memory\n", __func__);
		rc = -ENOMEM;
		goto end;
	}
	vfe_dev->stats = kzalloc(sizeof(struct msm_isp_statistics), GFP_KERNEL);
	if (!vfe_dev->stats) {
		pr_err("%s: no enough memory\n", __func__);
		rc = -ENOMEM;
		goto probe_fail1;
	}

	vfe_dev->ub_info = kzalloc(sizeof(struct msm_isp_ub_info), GFP_KERNEL);
	if (!vfe_dev->ub_info) {
		pr_err("%s: no enough memory\n", __func__);
		rc = -ENOMEM;
		goto probe_fail2;
	}
	if (pdev->dev.of_node) {
		pr_err("%s: CAMERA 1 \n", __func__);
		of_property_read_u32((&pdev->dev)->of_node,
			"cell-index", &pdev->id);
		match_dev = of_match_device(msm_vfe_dt_match, &pdev->dev);
		//if (!match_dev) {
		if (1) {
			pr_err("%s: No vfe hardware info\n", __func__);
			rc = -EINVAL;
			goto probe_fail3;
		}
		vfe_dev->hw_info =
			(struct msm_vfe_hardware_info *) match_dev->data;
	} else {
		pr_err("%s: CAMERA 2 \n", __func__);
		vfe_dev->hw_info = (struct msm_vfe_hardware_info *)
			platform_get_device_id(pdev)->driver_data;
	}

	if (!vfe_dev->hw_info) {
		pr_err("%s: No vfe hardware info\n", __func__);
		rc = -EINVAL;
		goto probe_fail3;
	}

probe_fail3:
	pr_err("%s: CAMERA 3 Falling to fail3\n", __func__);
	kfree(vfe_dev->ub_info);
probe_fail2:
	kfree(vfe_dev->stats);
probe_fail1:
	kfree(vfe_dev);
end:
	return rc;
}

static struct platform_driver vfe_driver = {
	.probe = vfe_probe,
	.driver = {
		.name = "msm_vfe",
		.owner = THIS_MODULE,
		.of_match_table = msm_vfe_dt_match,
	},
	.id_table = msm_vfe_dev_id,
};

static int __init msm_vfe_init_module(void)
{
	return platform_driver_register(&vfe_driver);
}

static void __exit msm_vfe_exit_module(void)
{
	platform_driver_unregister(&vfe_driver);
}

module_init(msm_vfe_init_module);
module_exit(msm_vfe_exit_module);
MODULE_DESCRIPTION("MSM VFE driver");
MODULE_LICENSE("GPL v2");

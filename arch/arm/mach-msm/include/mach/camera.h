/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __ASM__ARCH_CAMERA_H
#define __ASM__ARCH_CAMERA_H

#include <linux/list.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include "linux/types.h"

#include <mach/board.h>
#include <media/msm_camera.h>

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define CDBG(fmt, args...) printk(KERN_INFO "msm_camera: " fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define MSM_CAMERA_MSG 0
#define MSM_CAMERA_EVT 1
#define NUM_WB_EXP_NEUTRAL_REGION_LINES 4
#define NUM_WB_EXP_STAT_OUTPUT_BUFFERS  3
#define NUM_AUTOFOCUS_MULTI_WINDOW_GRIDS 16
#define NUM_STAT_OUTPUT_BUFFERS      3
#define NUM_AF_STAT_OUTPUT_BUFFERS      3

enum msm_queue {
	MSM_CAM_Q_CTRL,     /* control command or control command status */
	MSM_CAM_Q_VFE_EVT,  /* adsp event */
	MSM_CAM_Q_VFE_MSG,  /* adsp message */
	MSM_CAM_Q_V4L2_REQ, /* v4l2 request */
};

enum vfe_resp_msg {
	VFE_EVENT,
	VFE_MSG_GENERAL,
	VFE_MSG_SNAPSHOT,
	VFE_MSG_OUTPUT1,
	VFE_MSG_OUTPUT2,
	VFE_MSG_STATS_AF,
	VFE_MSG_STATS_WE, /* AEC + AWB */
	VFE_MSG_OUTPUT_P,   /* preview (continuous mode ) */
	VFE_MSG_OUTPUT_T,   /* thumbnail (snapshot mode )*/
	VFE_MSG_OUTPUT_S,   /* main image (snapshot mode )*/
	VFE_MSG_OUTPUT_V,   /* video   (continuous mode ) */
	VFE_MSG_STATS_AEC,
	VFE_MSG_STATS_AWB,
	VFE_MSG_STATS_RS,
	VFE_MSG_STATS_CS,
	VFE_MSG_STATS_IHIST,
	VFE_MSG_STATS_SKIN,
};

struct msm_vfe_phy_info {
	uint32_t sbuf_phy;
	uint32_t y_phy;
	uint32_t cbcr_phy;
	uint8_t  output_id; /* OUTPUT_MODE_P/T/S/V */
};

struct msm_vfe_resp {
	enum vfe_resp_msg type;
	struct msm_vfe_evt_msg evt_msg;
	struct msm_vfe_phy_info phy;
	void    *extdata;
	int32_t extlen;
};

struct msm_vfe_callback {
	void (*vfe_resp)(struct msm_vfe_resp *,
		enum msm_queue, void *syncdata,
		gfp_t gfp);
	void* (*vfe_alloc)(int, void *syncdata, gfp_t gfp);
	void (*vfe_free)(void *ptr);
	int (*flash_ctrl)(void *syncdata, int level);
};

struct msm_camvfe_fn {
	int (*vfe_init)(struct msm_vfe_callback *, struct platform_device *);
	int (*vfe_enable)(struct camera_enable_cmd *);
	int (*vfe_config)(struct msm_vfe_cfg_cmd *, void *);
	int (*vfe_disable)(struct camera_enable_cmd *,
		struct platform_device *dev);
	void (*vfe_release)(struct platform_device *);
};

struct msm_sensor_ctrl {
	int (*s_init)(const struct msm_camera_sensor_info *);
	int (*s_release)(void);
	int (*s_config)(void __user *);
};

/* this structure is used in kernel */
struct msm_queue_cmd {
	struct list_head list_config;
	struct list_head list_control;
	struct list_head list_frame;
	struct list_head list_pict;
	enum msm_queue type;
	void *command;
	int on_heap;
};

struct msm_device_queue {
	struct list_head list;
	spinlock_t lock;
	wait_queue_head_t wait;
	int max;
	int len;
	const char *name;
};

struct msm_sync {
	/* These two queues are accessed from a process context only.  They contain
	 * pmem descriptors for the preview frames and the stats coming from the
	 * camera sensor.
	 */
	struct hlist_head pmem_frames;
	struct hlist_head pmem_stats;

	/* The message queue is used by the control thread to send commands
	 * to the config thread, and also by the DSP to send messages to the
	 * config thread.  Thus it is the only queue that is accessed from
	 * both interrupt and process context.
	 */
	struct msm_device_queue event_q;

	/* This queue contains preview frames. It is accessed by the DSP (in
	 * in interrupt context, and by the frame thread.
	 */
	struct msm_device_queue frame_q;
	int unblock_poll_frame;

	/* This queue contains snapshot frames.  It is accessed by the DSP (in
	 * interrupt context, and by the control thread.
	 */
	struct msm_device_queue pict_q;

	struct msm_camera_sensor_info *sdata;
	struct msm_camvfe_fn vfefn;
	struct msm_sensor_ctrl sctrl;
	struct wake_lock wake_lock;
	struct platform_device *pdev;
	uint8_t opencnt;
	void *cropinfo;
	int  croplen;

	uint32_t pp_mask;
	struct msm_queue_cmd *pp_prev;
	struct msm_queue_cmd *pp_snap;

	/* When this flag is set, we send preview-frame notifications to config
	 * as well as to the frame queue.  By default, the flag is cleared.
	 */
	uint32_t report_preview_to_config;

	const char *apps_id;

	struct mutex lock;
	struct list_head list;
	int get_pic_abort;
};

#define MSM_APPS_ID_V4L2 "msm_v4l2"
#define MSM_APPS_ID_PROP "msm_qct"

struct msm_device {
	struct msm_sync *sync; /* most-frequently accessed */
	struct device *device;
	struct cdev cdev;
	/* opened is meaningful only for the config and frame nodes,
	 * which may be opened only once.
	 */
	atomic_t opened;
};

struct msm_control_device {
	struct msm_device *pmsm;

	/* Used for MSM_CAM_IOCTL_CTRL_CMD_DONE responses */
	uint8_t ctrl_data[50];
	struct msm_ctrl_cmd ctrl;
	struct msm_queue_cmd qcmd;

	/* This queue used by the config thread to send responses back to the
	 * control thread.  It is accessed only from a process context.
	 */
	struct msm_device_queue ctrl_q;
};

struct register_address_value_pair {
	uint16_t register_address;
	uint16_t register_value;
};

struct msm_pmem_region {
	struct hlist_node list;
	unsigned long paddr;
	unsigned long kvaddr;
	unsigned long len;
	struct file *file;
	struct msm_pmem_info info;
};

struct axidata {
	uint32_t bufnum1;
	uint32_t bufnum2;
	struct msm_pmem_region *region;
	uint32_t bufnum3;
};

#ifdef CONFIG_MSM_CAMERA_V4L2
/* Below functions are added for V4L2 kernel APIs */
struct msm_v4l2_driver {
	struct msm_sync *sync;
	int (*open)(struct msm_sync *, const char *apps_id);
	int (*release)(struct msm_sync *);
	int (*ctrl)(struct msm_sync *, struct msm_ctrl_cmd *);
	int (*reg_pmem)(struct msm_sync *, struct msm_pmem_info *);
	int (*get_frame) (struct msm_sync *, struct msm_frame *);
	int (*put_frame) (struct msm_sync *, struct msm_frame *);
	int (*get_pict) (struct msm_sync *, struct msm_ctrl_cmd *);
	unsigned int (*drv_poll) (struct msm_sync *, struct file *,
				struct poll_table_struct *);
};

int msm_v4l2_register(struct msm_v4l2_driver *);
int msm_v4l2_unregister(struct msm_v4l2_driver *);
#endif

void msm_camvfe_init(void);
int msm_camvfe_check(void *);
void msm_camvfe_fn_init(struct msm_camvfe_fn *, void *);
int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
					struct msm_sensor_ctrl *));

enum msm_camio_clk_type {
	CAMIO_VFE_MDC_CLK,
	CAMIO_MDC_CLK,
	CAMIO_VFE_CLK,
	CAMIO_VFE_AXI_CLK,

	CAMIO_VFE_CAMIF_CLK,
	CAMIO_VFE_PBDG_CLK,
	CAMIO_CAM_MCLK_CLK,
	CAMIO_CAMIF_PAD_PBDG_CLK,
	CAMIO_CSI_CLK,
	CAMIO_CSI_VFE_CLK,
	CAMIO_CSI_PCLK,
	CAMIO_MAX_CLK
};

enum msm_camio_clk_src_type {
	MSM_CAMIO_CLK_SRC_INTERNAL,
	MSM_CAMIO_CLK_SRC_EXTERNAL,
	MSM_CAMIO_CLK_SRC_MAX
};

enum msm_s_test_mode {
	S_TEST_OFF,
	S_TEST_1,
	S_TEST_2,
	S_TEST_3
};

enum msm_s_resolution {
	S_QTR_SIZE,
	S_FULL_SIZE,
	S_INVALID_SIZE
};

enum msm_s_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	S_REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	S_UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	S_UPDATE_ALL,
	/* Not valid update */
	S_UPDATE_INVALID
};

enum msm_s_setting {
	S_RES_PREVIEW,
	S_RES_CAPTURE
};

int msm_camio_enable(struct platform_device *dev);

int  msm_camio_clk_enable(enum msm_camio_clk_type clk);
int  msm_camio_clk_disable(enum msm_camio_clk_type clk);
int  msm_camio_clk_config(uint32_t freq);
void msm_camio_clk_rate_set(int rate);
void msm_camio_clk_axi_rate_set(int rate);
void msm_disable_io_gpio_clk(struct platform_device *);

void msm_camio_camif_pad_reg_reset(void);
void msm_camio_camif_pad_reg_reset_2(void);

void msm_camio_vfe_blk_reset(void);

void msm_camio_clk_sel(enum msm_camio_clk_src_type);
void msm_camio_disable(struct platform_device *);
int msm_camio_probe_on(struct platform_device *);
int msm_camio_probe_off(struct platform_device *);
#endif

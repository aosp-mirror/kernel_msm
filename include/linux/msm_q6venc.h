/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _MSM_VENC_H_
#define _MSM_VENC_H_

#include <linux/types.h>

struct venc_buf {
	unsigned int src_id;
	int fd;
	unsigned long offset;
	unsigned long size;
};

struct q6_init_config {
	unsigned short		venc_standard;
	unsigned short		partial_run_length_flag;
	unsigned short		h263_annex_ispt;
	unsigned short		h263_annex_jspt;
	unsigned short		h263_annex_tspt;
	unsigned short		rc_flag;
	unsigned short		one_mv_flag;
	unsigned short		acdc_pred_enable;
	unsigned short		rounding_bit_ctrl;
	unsigned short		rotation_flag;
	unsigned short		max_mvx;
	unsigned short		max_mvy;
	unsigned short		enc_frame_height_inmb;
	unsigned short		enc_frame_width_inmb;
	unsigned short		dvs_frame_height;
	unsigned short		dvs_frame_width;

	/* unused by userspace, filled in by kernel */
	unsigned int		ref_frame_buf1_phy;
	unsigned int		ref_frame_buf2_phy;
	unsigned int		rlc_buf1_phy;
	unsigned int		rlc_buf2_phy;
	unsigned int		rlc_buf_length;
};

struct init_config {
	struct venc_buf		ref_frame_buf1;
	struct venc_buf		ref_frame_buf2;
	struct venc_buf		rlc_buf1;
	struct venc_buf		rlc_buf2;
	struct q6_init_config	q6_init_config;
};

struct q6_encode_param {
	unsigned int		luma_addr;
	unsigned int		chroma_addr;
	unsigned int		x_offset;
	unsigned int		y_offset;
	unsigned int		frame_rho_budget;
	unsigned int		frame_type;
	unsigned int		qp;
};

struct encode_param {
	struct venc_buf		y_addr;
	unsigned long		uv_offset;
	struct q6_encode_param	q6_encode_param;
};

struct intra_refresh {
	unsigned int		intra_refresh_enable;
	unsigned int		intra_mb_num;
};

struct rc_config {
	unsigned short		max_frame_qp_up_delta;
	unsigned short		max_frame_qp_down_delta;
	unsigned short		min_frame_qp;
	unsigned short		max_frame_qp;
};

struct q6_frame_type {
	unsigned int		frame_type;
	unsigned int		frame_len;
	unsigned int		frame_addr;
	unsigned int		map_table;
};

struct frame_type {
	struct venc_buf		frame_addr;
	struct q6_frame_type	q6_frame_type;
};

#define VENC_IOCTL_MAGIC 'V'

#define VENC_IOCTL_INITIALIZE     _IOW(VENC_IOCTL_MAGIC, 1, struct init_config)
#define VENC_IOCTL_ENCODE	  _IOW(VENC_IOCTL_MAGIC, 2, struct encode_param)
#define VENC_IOCTL_INTRA_REFRESH _IOW(VENC_IOCTL_MAGIC, 3, struct intra_refresh)
#define VENC_IOCTL_RC_CONFIG      _IOW(VENC_IOCTL_MAGIC, 4, struct rc_config)
#define VENC_IOCTL_ENCODE_CONFIG  _IOW(VENC_IOCTL_MAGIC, 5, struct init_config)
#define VENC_IOCTL_STOP              _IO(VENC_IOCTL_MAGIC, 6)
#define VENC_IOCTL_WAIT_FOR_ENCODE  _IOR(VENC_IOCTL_MAGIC, 7, struct frame_type)
#define VENC_IOCTL_STOP_ENCODE       _IO(VENC_IOCTL_MAGIC, 8)

#endif /* _MSM_VENC_H_ */

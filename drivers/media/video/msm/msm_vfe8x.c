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

#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <mach/irqs.h>
#include <linux/clk.h>
#include "msm_vfe8x_proc.h"

#define ON  1
#define OFF 0

static void *vfe_syncdata;
static struct clk *ebi1_clk;
static const char *const clk_name = "ebi1_clk";

static int vfe_enable(struct camera_enable_cmd *enable)
{
	return 0;
}

static int vfe_disable(struct camera_enable_cmd *enable,
		       struct platform_device *dev)
{
	vfe_stop();
	msm_camio_disable(dev);
	return 0;
}

static int vfe_init(struct msm_vfe_callback *presp, struct platform_device *dev)
{
	int rc = 0;

	ebi1_clk = clk_get(NULL, clk_name);
	if (!ebi1_clk) {
		pr_err("%s: could not get %s\n", __func__, clk_name);
		return -EIO;
	}

	rc = clk_set_rate(ebi1_clk, 128000000);
	if (rc < 0) {
		pr_err("%s: clk_set_rate(%s) failed: %d\n", __func__,
			clk_name, rc);
		return rc;
	}

	rc = vfe_cmd_init(presp, dev, vfe_syncdata);
	if (rc < 0)
		return rc;

	/* Bring up all the required GPIOs and Clocks */
	return msm_camio_enable(dev);
}

static void vfe_release(struct platform_device *dev)
{
	struct msm_sensor_ctrl *sctrl =
		&((struct msm_sync *)vfe_syncdata)->sctrl;

	if (ebi1_clk) {
		clk_set_rate(ebi1_clk, 0);
		clk_put(ebi1_clk);
		ebi1_clk = 0;
	}

	if (sctrl)
		sctrl->s_release();

	msm_camio_disable(dev);
	vfe_cmd_release(dev);
	vfe_syncdata = NULL;
}

static void vfe_config_axi(int mode,
			   struct axidata *ad,
			   struct vfe_cmd_axi_output_config *ao)
{
	struct msm_pmem_region *regptr;
	int i, j;
	uint32_t *p1, *p2;

	if (mode == OUTPUT_1 || mode == OUTPUT_1_AND_2) {
		regptr = ad->region;
		for (i = 0; i < ad->bufnum1; i++) {

			p1 = &(ao->output1.outputY.outFragments[i][0]);
			p2 = &(ao->output1.outputCbcr.outFragments[i][0]);

			for (j = 0; j < ao->output1.fragmentCount; j++) {

				*p1 = regptr->paddr + regptr->info.y_off;
				p1++;

				*p2 = regptr->paddr + regptr->info.cbcr_off;
				p2++;
			}
			regptr++;
		}
	}
	/* if OUTPUT1 or Both */
	if (mode == OUTPUT_2 || mode == OUTPUT_1_AND_2) {

		regptr = &(ad->region[ad->bufnum1]);
		CDBG("bufnum2 = %d\n", ad->bufnum2);

		for (i = 0; i < ad->bufnum2; i++) {

			p1 = &(ao->output2.outputY.outFragments[i][0]);
			p2 = &(ao->output2.outputCbcr.outFragments[i][0]);

			CDBG("config_axi: O2, phy = 0x%lx, y_off = %d, "\
			     "cbcr_off = %d\n", regptr->paddr,
			     regptr->info.y_off, regptr->info.cbcr_off);

			for (j = 0; j < ao->output2.fragmentCount; j++) {

				*p1 = regptr->paddr + regptr->info.y_off;
				CDBG("vfe_config_axi: p1 = 0x%x\n", *p1);
				p1++;

				*p2 = regptr->paddr + regptr->info.cbcr_off;
				CDBG("vfe_config_axi: p2 = 0x%x\n", *p2);
				p2++;
			}
			regptr++;
		}
	}
}

#define ERR_COPY_FROM_USER() \
	pr_err("%s(%d): copy from user\n", __func__, __LINE__)

#define CHECKED_COPY_FROM_USER(in) {					\
	if (cmd->length != sizeof(*(in))) {				\
		pr_err("msm_camera: %s:%d cmd %d: user data size %d "	\
			"!= kernel data size %d\n",			\
			__func__, __LINE__,				\
			cmd->id, cmd->length, sizeof(*(in)));		\
		rc = -EIO;						\
		break;							\
	}								\
	if (copy_from_user((in), (void __user *)cmd->value,		\
			sizeof(*(in)))) {				\
		ERR_COPY_FROM_USER();					\
		rc = -EFAULT;						\
		break;							\
	}								\
}

static int vfe_proc_general(struct msm_vfe_command_8k *cmd)
{
	int rc = 0;

	CDBG("%s: cmdID = %d\n", __func__, cmd->id);

	switch (cmd->id) {
	case VFE_CMD_ID_RESET:
		msm_camio_vfe_blk_reset();
		msm_camio_camif_pad_reg_reset_2();
		vfe_reset();
		break;

	case VFE_CMD_ID_START:{
			struct vfe_cmd_start start;
			CHECKED_COPY_FROM_USER(&start);

			/* msm_camio_camif_pad_reg_reset_2(); */
			msm_camio_camif_pad_reg_reset();
			vfe_start(&start);
		}
		break;

	case VFE_CMD_ID_CAMIF_CONFIG:{
			struct vfe_cmd_camif_config camif;
			CHECKED_COPY_FROM_USER(&camif);

			vfe_camif_config(&camif);
		}
		break;

	case VFE_CMD_ID_BLACK_LEVEL_CONFIG:{
			struct vfe_cmd_black_level_config bl;
			CHECKED_COPY_FROM_USER(&bl);

			vfe_black_level_config(&bl);
		}
		break;

	case VFE_CMD_ID_ROLL_OFF_CONFIG:{
			/* rolloff is too big to be on the stack */
			struct vfe_cmd_roll_off_config *rolloff =
			    kmalloc(sizeof(struct vfe_cmd_roll_off_config),
				    GFP_KERNEL);
			if (!rolloff) {
				pr_err("%s: out of memory\n", __func__);
				rc = -ENOMEM;
				break;
			}
			/* Wrap CHECKED_COPY_FROM_USER() in a do-while(0) loop
			 * to make sure we free rolloff when copy_from_user()
			 * fails.
			 */
			do {
				CHECKED_COPY_FROM_USER(rolloff);
				vfe_roll_off_config(rolloff);
			} while (0);
			kfree(rolloff);
		}
		break;

	case VFE_CMD_ID_DEMUX_CHANNEL_GAIN_CONFIG:{
			struct vfe_cmd_demux_channel_gain_config demuxc;
			CHECKED_COPY_FROM_USER(&demuxc);

			/* demux is always enabled.  */
			vfe_demux_channel_gain_config(&demuxc);
		}
		break;

	case VFE_CMD_ID_DEMOSAIC_CONFIG:{
			struct vfe_cmd_demosaic_config demosaic;
			CHECKED_COPY_FROM_USER(&demosaic);

			vfe_demosaic_config(&demosaic);
		}
		break;

	case VFE_CMD_ID_FOV_CROP_CONFIG:
	case VFE_CMD_ID_FOV_CROP_UPDATE:{
			struct vfe_cmd_fov_crop_config fov;
			CHECKED_COPY_FROM_USER(&fov);

			vfe_fov_crop_config(&fov);
		}
		break;

	case VFE_CMD_ID_MAIN_SCALER_CONFIG:
	case VFE_CMD_ID_MAIN_SCALER_UPDATE:{
			struct vfe_cmd_main_scaler_config mainds;
			CHECKED_COPY_FROM_USER(&mainds);

			vfe_main_scaler_config(&mainds);
		}
		break;

	case VFE_CMD_ID_WHITE_BALANCE_CONFIG:
	case VFE_CMD_ID_WHITE_BALANCE_UPDATE:{
			struct vfe_cmd_white_balance_config wb;
			CHECKED_COPY_FROM_USER(&wb);

			vfe_white_balance_config(&wb);
		}
		break;

	case VFE_CMD_ID_COLOR_CORRECTION_CONFIG:
	case VFE_CMD_ID_COLOR_CORRECTION_UPDATE:{
			struct vfe_cmd_color_correction_config cc;
			CHECKED_COPY_FROM_USER(&cc);

			vfe_color_correction_config(&cc);
		}
		break;

	case VFE_CMD_ID_LA_CONFIG:{
			struct vfe_cmd_la_config la;
			CHECKED_COPY_FROM_USER(&la);

			vfe_la_config(&la);
		}
		break;

	case VFE_CMD_ID_RGB_GAMMA_CONFIG:{
			struct vfe_cmd_rgb_gamma_config rgb;
			CHECKED_COPY_FROM_USER(&rgb);

			rc = vfe_rgb_gamma_config(&rgb);
		}
		break;

	case VFE_CMD_ID_CHROMA_ENHAN_CONFIG:
	case VFE_CMD_ID_CHROMA_ENHAN_UPDATE:{
			struct vfe_cmd_chroma_enhan_config chrom;
			CHECKED_COPY_FROM_USER(&chrom);

			vfe_chroma_enhan_config(&chrom);
		}
		break;

	case VFE_CMD_ID_CHROMA_SUPPRESSION_CONFIG:
	case VFE_CMD_ID_CHROMA_SUPPRESSION_UPDATE:{
			struct vfe_cmd_chroma_suppression_config chromsup;
			CHECKED_COPY_FROM_USER(&chromsup);

			vfe_chroma_sup_config(&chromsup);
		}
		break;

	case VFE_CMD_ID_ASF_CONFIG:{
			struct vfe_cmd_asf_config asf;
			CHECKED_COPY_FROM_USER(&asf);

			vfe_asf_config(&asf);
		}
		break;

	case VFE_CMD_ID_SCALER2Y_CONFIG:
	case VFE_CMD_ID_SCALER2Y_UPDATE:{
			struct vfe_cmd_scaler2_config ds2y;
			CHECKED_COPY_FROM_USER(&ds2y);

			vfe_scaler2y_config(&ds2y);
		}
		break;

	case VFE_CMD_ID_SCALER2CbCr_CONFIG:
	case VFE_CMD_ID_SCALER2CbCr_UPDATE:{
			struct vfe_cmd_scaler2_config ds2cbcr;
			CHECKED_COPY_FROM_USER(&ds2cbcr);

			vfe_scaler2cbcr_config(&ds2cbcr);
		}
		break;

	case VFE_CMD_ID_CHROMA_SUBSAMPLE_CONFIG:{
			struct vfe_cmd_chroma_subsample_config sub;
			CHECKED_COPY_FROM_USER(&sub);

			vfe_chroma_subsample_config(&sub);
		}
		break;

	case VFE_CMD_ID_FRAME_SKIP_CONFIG:{
			struct vfe_cmd_frame_skip_config fskip;
			CHECKED_COPY_FROM_USER(&fskip);

			vfe_frame_skip_config(&fskip);
		}
		break;

	case VFE_CMD_ID_OUTPUT_CLAMP_CONFIG:{
			struct vfe_cmd_output_clamp_config clamp;
			CHECKED_COPY_FROM_USER(&clamp);

			vfe_output_clamp_config(&clamp);
		}
		break;

		/* module update commands */
	case VFE_CMD_ID_BLACK_LEVEL_UPDATE:{
			struct vfe_cmd_black_level_config blk;
			CHECKED_COPY_FROM_USER(&blk);

			vfe_black_level_update(&blk);
		}
		break;

	case VFE_CMD_ID_DEMUX_CHANNEL_GAIN_UPDATE:{
			struct vfe_cmd_demux_channel_gain_config dmu;
			CHECKED_COPY_FROM_USER(&dmu);

			vfe_demux_channel_gain_update(&dmu);
		}
		break;

	case VFE_CMD_ID_DEMOSAIC_BPC_UPDATE:{
			struct vfe_cmd_demosaic_bpc_update demo_bpc;
			CHECKED_COPY_FROM_USER(&demo_bpc);

			vfe_demosaic_bpc_update(&demo_bpc);
		}
		break;

	case VFE_CMD_ID_DEMOSAIC_ABF_UPDATE:{
			struct vfe_cmd_demosaic_abf_update demo_abf;
			CHECKED_COPY_FROM_USER(&demo_abf);

			vfe_demosaic_abf_update(&demo_abf);
		}
		break;

	case VFE_CMD_ID_LA_UPDATE:{
			struct vfe_cmd_la_config la;
			CHECKED_COPY_FROM_USER(&la);

			vfe_la_update(&la);
		}
		break;

	case VFE_CMD_ID_RGB_GAMMA_UPDATE:{
			struct vfe_cmd_rgb_gamma_config rgb;
			CHECKED_COPY_FROM_USER(&rgb);

			rc = vfe_rgb_gamma_update(&rgb);
		}
		break;

	case VFE_CMD_ID_ASF_UPDATE:{
			struct vfe_cmd_asf_update asf;
			CHECKED_COPY_FROM_USER(&asf);

			vfe_asf_update(&asf);
		}
		break;

	case VFE_CMD_ID_FRAME_SKIP_UPDATE:{
			struct vfe_cmd_frame_skip_update fskip;
			CHECKED_COPY_FROM_USER(&fskip);

			vfe_frame_skip_update(&fskip);
		}
		break;

	case VFE_CMD_ID_CAMIF_FRAME_UPDATE:{
			struct vfe_cmds_camif_frame fup;
			CHECKED_COPY_FROM_USER(&fup);

			vfe_camif_frame_update(&fup);
		}
		break;

		/* stats update commands */
	case VFE_CMD_ID_STATS_AUTOFOCUS_UPDATE:{
			struct vfe_cmd_stats_af_update afup;
			CHECKED_COPY_FROM_USER(&afup);

			vfe_stats_update_af(&afup);
		}
		break;

	case VFE_CMD_ID_STATS_WB_EXP_UPDATE:{
			struct vfe_cmd_stats_wb_exp_update wbexp;
			CHECKED_COPY_FROM_USER(&wbexp);

			vfe_stats_update_wb_exp(&wbexp);
		}
		break;

		/* control of start, stop, update, etc... */
	case VFE_CMD_ID_STOP:
		vfe_stop();
		break;

	case VFE_CMD_ID_GET_HW_VERSION:
		break;

		/* stats */
	case VFE_CMD_ID_STATS_SETTING:{
			struct vfe_cmd_stats_setting stats;
			CHECKED_COPY_FROM_USER(&stats);

			vfe_stats_setting(&stats);
		}
		break;

	case VFE_CMD_ID_STATS_AUTOFOCUS_START:{
			struct vfe_cmd_stats_af_start af;
			CHECKED_COPY_FROM_USER(&af);

			vfe_stats_start_af(&af);
		}
		break;

	case VFE_CMD_ID_STATS_AUTOFOCUS_STOP:
		vfe_stats_af_stop();
		break;

	case VFE_CMD_ID_STATS_WB_EXP_START:{
			struct vfe_cmd_stats_wb_exp_start awexp;
			CHECKED_COPY_FROM_USER(&awexp);

			vfe_stats_start_wb_exp(&awexp);
		}
		break;

	case VFE_CMD_ID_STATS_WB_EXP_STOP:
		vfe_stats_wb_exp_stop();
		break;

	case VFE_CMD_ID_ASYNC_TIMER_SETTING:
		break;

	case VFE_CMD_ID_UPDATE:
		vfe_update();
		break;

		/* test gen */
	case VFE_CMD_ID_TEST_GEN_START:
		break;

	case VFE_CMD_ID_EPOCH1_CONFIG:{
			struct vfe_cmds_camif_epoch epoch1;
			CHECKED_COPY_FROM_USER(&epoch1);
			vfe_epoch1_config(&epoch1);
		}
		break;

/*
  acknowledge from upper layer
	these are not in general command.

	case VFE_CMD_ID_OUTPUT1_ACK:
		break;
	case VFE_CMD_ID_OUTPUT2_ACK:
		break;
	case VFE_CMD_ID_EPOCH1_ACK:
		break;
	case VFE_CMD_ID_EPOCH2_ACK:
		break;
	case VFE_CMD_ID_STATS_AUTOFOCUS_ACK:
		break;
	case VFE_CMD_ID_STATS_WB_EXP_ACK:
		break;
*/

	default:
		pr_err("%s: invalid cmd id %d\n", __func__, cmd->id);
		rc = -EINVAL;
		break;
	}			/* switch */

	return rc;
}

static int vfe_config(struct msm_vfe_cfg_cmd *cmd, void *data)
{
	struct msm_pmem_region *regptr;
	struct msm_vfe_command_8k vfecmd;
	struct vfe_cmd_axi_output_config axio;
	struct axidata *axid = data;

	int rc = 0;

	if (cmd->cmd_type != CMD_FRAME_BUF_RELEASE &&
	    cmd->cmd_type != CMD_STATS_BUF_RELEASE &&
	    cmd->cmd_type != CMD_STATS_AF_BUF_RELEASE) {
		if (copy_from_user(&vfecmd,
				   (void __user *)(cmd->value), sizeof(vfecmd))) {
			ERR_COPY_FROM_USER();
			return -EFAULT;
		}
	}

	CDBG("%s: cmdType = %d\n", __func__, cmd->cmd_type);

	switch (cmd->cmd_type) {
	case CMD_GENERAL:
		rc = vfe_proc_general(&vfecmd);
		break;

	case CMD_STATS_ENABLE:
	case CMD_STATS_AXI_CFG: {
			int i;
			struct vfe_cmd_stats_setting scfg;

			BUG_ON(!axid);

			if (vfecmd.length != sizeof(scfg)) {
				pr_err
				    ("msm_camera: %s: cmd %d: user-space "\
				     "data size %d != kernel data size %d\n",
				     __func__,
				     cmd->cmd_type, vfecmd.length,
				     sizeof(scfg));
				return -EIO;
			}

			if (copy_from_user(&scfg,
					   (void __user *)(vfecmd.value),
					   sizeof(scfg))) {
				ERR_COPY_FROM_USER();
				return -EFAULT;
			}

			regptr = axid->region;
			if (axid->bufnum1 > 0) {
				for (i = 0; i < axid->bufnum1; i++) {
					scfg.awbBuffer[i] =
					    (uint32_t) (regptr->paddr);
					regptr++;
				}
			}

			if (axid->bufnum2 > 0) {
				for (i = 0; i < axid->bufnum2; i++) {
					scfg.afBuffer[i] =
					    (uint32_t) (regptr->paddr);
					regptr++;
				}
			}

			vfe_stats_setting(&scfg);
		}
		break;

	case CMD_STATS_AF_AXI_CFG:
		break;

	case CMD_FRAME_BUF_RELEASE: {
			/* preview buffer release */
			struct msm_frame *b;
			unsigned long p;
			struct vfe_cmd_output_ack fack;

			BUG_ON(!data);

			b = (struct msm_frame *)(cmd->value);
			p = *(unsigned long *)data;

			fack.ybufaddr[0] = (uint32_t) (p + b->y_off);

			fack.chromabufaddr[0] = (uint32_t) (p + b->cbcr_off);

			if (b->path == MSM_FRAME_PREV_1)
				vfe_output1_ack(&fack);

			if (b->path == MSM_FRAME_ENC ||
			    b->path == MSM_FRAME_PREV_2)
				vfe_output2_ack(&fack);


		}
		break;

	case CMD_SNAP_BUF_RELEASE:
		break;

	case CMD_STATS_BUF_RELEASE: {
			struct vfe_cmd_stats_wb_exp_ack sack;

			BUG_ON(!data);

			sack.nextWbExpOutputBufferAddr = *(uint32_t *) data;
			vfe_stats_wb_exp_ack(&sack);
		}
		break;

	case CMD_STATS_AF_BUF_RELEASE: {
			struct vfe_cmd_stats_af_ack ack;

			BUG_ON(!data);

			ack.nextAFOutputBufferAddr = *(uint32_t *) data;
			vfe_stats_af_ack(&ack);
		}
		break;

	case CMD_AXI_CFG_OUT1: {

			BUG_ON(!axid);

			if (copy_from_user(&axio, (void __user *)(vfecmd.value),
					   sizeof(axio))) {
				ERR_COPY_FROM_USER();
				return -EFAULT;
			}

			vfe_config_axi(OUTPUT_1, axid, &axio);
			vfe_axi_output_config(&axio);
		}
		break;

	case CMD_AXI_CFG_OUT2:
	case CMD_RAW_PICT_AXI_CFG: {

			BUG_ON(!axid);

			if (copy_from_user(&axio, (void __user *)(vfecmd.value),
					   sizeof(axio))) {
				ERR_COPY_FROM_USER();
				return -EFAULT;
			}

			vfe_config_axi(OUTPUT_2, axid, &axio);

			axio.outputDataSize = 0;
			vfe_axi_output_config(&axio);
		}
		break;

	case CMD_AXI_CFG_O1_AND_O2:
	case CMD_AXI_CFG_SNAP_O1_AND_O2: {

			BUG_ON(!axid);

			if (copy_from_user(&axio, (void __user *)(vfecmd.value),
					   sizeof(axio))) {
				ERR_COPY_FROM_USER();
				return -EFAULT;
			}

			vfe_config_axi(OUTPUT_1_AND_2, axid, &axio);
			vfe_axi_output_config(&axio);
		}
		break;

	default:
		break;
	}			/* switch */

	return rc;
}

void msm_camvfe_fn_init(struct msm_camvfe_fn *fptr, void *data)
{
	fptr->vfe_init = vfe_init;
	fptr->vfe_enable = vfe_enable;
	fptr->vfe_config = vfe_config;
	fptr->vfe_disable = vfe_disable;
	fptr->vfe_release = vfe_release;
	vfe_syncdata = data;
}

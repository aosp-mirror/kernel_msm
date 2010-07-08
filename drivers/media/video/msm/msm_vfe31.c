/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
#include "msm_vfe31.h"
#include <mach/camera.h>
#include <linux/io.h>

#define CHECKED_COPY_FROM_USER(in) {					\
	if (copy_from_user((in), (void __user *)cmd->value,		\
			cmd->length)) {					\
		rc = -EFAULT;						\
		break;							\
	}								\
}

#define vfe31_get_ch_ping_addr(chn) \
	(vfe_io_r(VFE_AXI_OFFSET + 0x18 * (chn)))
#define vfe31_get_ch_pong_addr(chn) \
	(vfe_io_r(VFE_AXI_OFFSET + 0x18 * (chn) + 4))
#define vfe31_get_ch_addr(ping_pong, chn) \
	(((ping_pong) & (1 << (chn))) == 0 ? \
	vfe31_get_ch_pong_addr(chn) : vfe31_get_ch_ping_addr(chn))

#define vfe31_put_ch_ping_addr(chn, addr) \
	(vfe_io_w((addr), VFE_AXI_OFFSET + 0x18 * (chn)))
#define vfe31_put_ch_pong_addr(chn, addr) \
	(vfe_io_w((addr), VFE_AXI_OFFSET + 0x18 * (chn) + 4))
#define vfe31_put_ch_addr(ping_pong, chn, addr) \
	(((ping_pong) & (1 << (chn))) == 0 ?   \
	vfe31_put_ch_pong_addr((chn), (addr)) : \
	vfe31_put_ch_ping_addr((chn), (addr)))

static struct vfe31_ctrl_type *vfe31_ctrl;
static void *vfe_syncdata;

struct vfe31_isr_queue_cmd {
	struct list_head list;
	uint32_t                           vfeInterruptStatus0;
	uint32_t                           vfeInterruptStatus1;
	struct vfe_frame_asf_info          vfeAsfFrameInfo;
	struct vfe_frame_bpc_info          vfeBpcFrameInfo;
	struct vfe_msg_camif_status        vfeCamifStatusLocal;
};

static struct vfe31_cmd_type vfe31_cmd[] = {
/* 0*/	{V31_DUMMY_0},
		{V31_SET_CLK},
		{V31_RESET},
		{V31_START},
		{V31_TEST_GEN_START},
/* 5*/	{V31_OPERATION_CFG, V31_OPERATION_CFG_LEN},
		{V31_AXI_OUT_CFG, V31_AXI_OUT_LEN, V31_AXI_OUT_OFF, 0xFF},
		{V31_CAMIF_CFG, V31_CAMIF_LEN, V31_CAMIF_OFF, 0xFF},
		{V31_AXI_INPUT_CFG},
		{V31_BLACK_LEVEL_CFG, V31_BLACK_LEVEL_LEN, V31_BLACK_LEVEL_OFF,
		0xFF},
/*10*/  {V31_ROLL_OFF_CFG, V31_ROLL_OFF_CFG_LEN, V31_ROLL_OFF_CFG_OFF,
		0xFF},
		{V31_DEMUX_CFG, V31_DEMUX_LEN, V31_DEMUX_OFF, 0xFF},
		{V31_DEMOSAIC_0_CFG, V31_DEMOSAIC_0_LEN, V31_DEMOSAIC_0_OFF,
		0xFF},
		{V31_DEMOSAIC_1_CFG, V31_DEMOSAIC_1_LEN, V31_DEMOSAIC_1_OFF,
		0xFF},
		{V31_DEMOSAIC_2_CFG, V31_DEMOSAIC_2_LEN, V31_DEMOSAIC_2_OFF,
		0xFF},
/*15*/	{V31_FOV_CFG, V31_FOV_LEN, V31_FOV_OFF, 0xFF},
		{V31_MAIN_SCALER_CFG, V31_MAIN_SCALER_LEN, V31_MAIN_SCALER_OFF,
		0xFF},
		{V31_WB_CFG, V31_WB_LEN, V31_WB_OFF, 0xFF},
		{V31_COLOR_COR_CFG, V31_COLOR_COR_LEN, V31_COLOR_COR_OFF, 0xFF},
		{V31_RGB_G_CFG, V31_RGB_G_LEN, V31_RGB_G_OFF, 0xFF},
/*20*/	{V31_LA_CFG, V31_LA_LEN, V31_LA_OFF, 0xFF },
		{V31_CHROMA_EN_CFG, V31_CHROMA_EN_LEN, V31_CHROMA_EN_OFF, 0xFF},
		{V31_CHROMA_SUP_CFG, V31_CHROMA_SUP_LEN, V31_CHROMA_SUP_OFF,
		0xFF},
		{V31_MCE_CFG, V31_MCE_LEN, V31_MCE_OFF, 0xFF},
		{V31_SK_ENHAN_CFG},
/*25*/	{V31_ASF_CFG, V31_ASF_LEN, V31_ASF_OFF, 0xFF},
		{V31_S2Y_CFG, V31_S2Y_LEN, V31_S2Y_OFF, 0xFF},
		{V31_S2CbCr_CFG, V31_S2CbCr_LEN, V31_S2CbCr_OFF, 0xFF},
		{V31_CHROMA_SUBS_CFG, V31_CHROMA_SUBS_LEN, V31_CHROMA_SUBS_OFF,
			0xFF},
		{V31_OUT_CLAMP_CFG, V31_OUT_CLAMP_LEN, V31_OUT_CLAMP_OFF, 0xFF},
/*30*/	{V31_FRAME_SKIP_CFG, V31_FRAME_SKIP_LEN, V31_FRAME_SKIP_OFF, 0xFF},
		{V31_DUMMY_1},
		{V31_DUMMY_2},
		{V31_DUMMY_3},
		{V31_UPDATE},
/*35*/	{V31_BL_LVL_UPDATE, V31_BLACK_LEVEL_LEN, V31_BLACK_LEVEL_OFF,
		0xFF},
		{V31_DEMUX_UPDATE, V31_DEMUX_LEN, V31_DEMUX_OFF, 0xFF},
		{V31_DEMOSAIC_1_UPDATE, V31_DEMOSAIC_1_LEN, V31_DEMOSAIC_1_OFF,
		0xFF},
		{V31_DEMOSAIC_2_UPDATE, V31_DEMOSAIC_2_LEN, V31_DEMOSAIC_2_OFF,
		0xFF},
		{V31_FOV_UPDATE, V31_FOV_LEN, V31_FOV_OFF, 0xFF},
/*40*/	{V31_MAIN_SCALER_UPDATE, V31_MAIN_SCALER_LEN, V31_MAIN_SCALER_OFF,
		0xFF},
		{V31_WB_UPDATE, V31_WB_LEN, V31_WB_OFF, 0xFF},
		{V31_COLOR_COR_UPDATE, V31_COLOR_COR_LEN, V31_COLOR_COR_OFF,
		0xFF},
		{V31_RGB_G_UPDATE, V31_RGB_G_LEN, V31_CHROMA_EN_OFF, 0xFF},
		{V31_LA_UPDATE, V31_LA_LEN, V31_LA_OFF, 0xFF },
/*45*/	{V31_CHROMA_EN_UPDATE, V31_CHROMA_EN_LEN, V31_CHROMA_EN_OFF, 0xFF},
		{V31_CHROMA_SUP_UPDATE, V31_CHROMA_SUP_LEN, V31_CHROMA_SUP_OFF,
		0xFF},
		{V31_MCE_UPDATE, V31_MCE_LEN, V31_MCE_OFF, 0xFF},
		{V31_SK_ENHAN_UPDATE},
		{V31_S2CbCr_UPDATE, V31_S2CbCr_LEN, V31_S2CbCr_OFF, 0xFF},
/*50*/	{V31_S2Y_UPDATE, V31_S2Y_LEN, V31_S2Y_OFF, 0xFF},
		{V31_ASF_UPDATE, V31_ASF_UPDATE_LEN, V31_ASF_OFF, 0xFF},
		{V31_FRAME_SKIP_UPDATE},
		{V31_CAMIF_FRAME_UPDATE},
		{V31_STATS_AF_UPDATE, V31_STATS_AF_LEN, V31_STATS_AF_OFF},
/*55*/	{V31_STATS_AE_UPDATE, V31_STATS_AE_LEN, V31_STATS_AE_OFF},
		{V31_STATS_AWB_UPDATE, V31_STATS_AWB_LEN, V31_STATS_AWB_OFF},
		{V31_STATS_RS_UPDATE, V31_STATS_RS_LEN, V31_STATS_RS_OFF},
		{V31_STATS_CS_UPDATE, V31_STATS_CS_LEN, V31_STATS_CS_OFF},
		{V31_STATS_SKIN_UPDATE},
/*60*/	{V31_STATS_IHIST_UPDATE, V31_STATS_IHIST_LEN, V31_STATS_IHIST_OFF},
		{V31_DUMMY_4},
		{V31_EPOCH1_ACK},
		{V31_EPOCH2_ACK},
		{V31_START_RECORDING},
/*65*/	{V31_STOP_RECORDING},
		{V31_DUMMY_5},
		{V31_DUMMY_6},
		{V31_CAPTURE, V31_CAPTURE_LEN, 0xFF},
		{V31_DUMMY_7},
/*70*/	{V31_STOP},
		{V31_GET_HW_VERSION},
		{V31_GET_FRAME_SKIP_COUNTS},
		{V31_OUTPUT1_BUFFER_ENQ},
		{V31_OUTPUT2_BUFFER_ENQ},
/*75*/	{V31_OUTPUT3_BUFFER_ENQ},
		{V31_JPEG_OUT_BUF_ENQ},
		{V31_RAW_OUT_BUF_ENQ},
		{V31_RAW_IN_BUF_ENQ},
		{V31_STATS_AF_ENQ},
/*80*/	{V31_STATS_AE_ENQ},
		{V31_STATS_AWB_ENQ},
		{V31_STATS_RS_ENQ},
		{V31_STATS_CS_ENQ},
		{V31_STATS_SKIN_ENQ},
/*85*/	{V31_STATS_IHIST_ENQ},
		{V31_DUMMY_8},
		{V31_JPEG_ENC_CFG},
		{V31_DUMMY_9},
		{V31_STATS_AF_START, V31_STATS_AF_LEN, V31_STATS_AF_OFF},
/*90*/	{V31_STATS_AF_STOP},
		{V31_STATS_AE_START, V31_STATS_AE_LEN, V31_STATS_AE_OFF},
		{V31_STATS_AE_STOP},
		{V31_STATS_AWB_START, V31_STATS_AWB_LEN, V31_STATS_AWB_OFF},
		{V31_STATS_AWB_STOP},
/*95*/	{V31_STATS_RS_START, V31_STATS_RS_LEN, V31_STATS_RS_OFF},
		{V31_STATS_RS_STOP},
		{V31_STATS_CS_START, V31_STATS_CS_LEN, V31_STATS_CS_OFF},
		{V31_STATS_CS_STOP},
		{V31_STATS_SKIN_START, V31_STATS_IHIST_LEN,
			V31_STATS_IHIST_OFF},
/*100*/	{V31_STATS_SKIN_STOP},
		{V31_STATS_IHIST_START},
		{V31_STATS_IHIST_STOP},
		{V31_DUMMY_10},
		{V31_SYNC_TIMER_SETTING},
/*105*/	{V31_ASYNC_TIMER_SETTING},
};

static void vfe_io_w(u32 data, unsigned long offset)
{
	writel(data, vfe31_ctrl->vfebase + offset);
	wmb();
}

static u32 vfe_io_r(unsigned long offset)
{
	uint32_t data = readl(vfe31_ctrl->vfebase + offset);
	rmb();
	return data;
}

static void msm_io_memcpy(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	int i;
	u32 *d = (u32 *) dest_addr;
	u32 *s = (u32 *) src_addr;
	/* memcpy_toio does not work. Use writel for now */
	for (i = 0; i < len / 4; i++) {
		writel(*s++, d++);
		wmb();
	}
}

static void vfe_io_w_axi_out_chan(u32 data, int8_t chan)
{
	uint32_t temp;
	vfe_io_w(data, V31_AXI_OUT_OFF + 20 + 24 * chan);
	temp = vfe_io_r(V31_AXI_OUT_OFF + 20 + 24 * chan);
}

static void vfe_msg_get_phy_addr(struct msm_vfe_phy_info *pinfo,
	struct vfe_message *data)
{
	pinfo->y_phy = data->_u.msgOut.yBuffer;
	pinfo->cbcr_phy = data->_u.msgOut.cbcrBuffer;
}

static void vfe31_proc_ops(enum VFE31_MESSAGE_ID id, void *msg, size_t len)
{
	struct msm_vfe_resp *rp;
	struct vfe_message *vfe_msg_data = NULL;

	rp = vfe31_ctrl->resp->vfe_alloc(sizeof(struct msm_vfe_resp),
		vfe31_ctrl->syncdata, GFP_ATOMIC);
	if (!rp) {
		pr_err("rp: cannot allocate buffer\n");
		return;
	}
	CDBG("vfe31_proc_ops, msgId = %d\n", id);
	rp->evt_msg.type   = MSM_CAMERA_MSG;
	rp->evt_msg.msg_id = id;
	rp->evt_msg.len    = len;
	rp->evt_msg.data   = msg;
	vfe_msg_data = (struct vfe_message *)rp->evt_msg.data;

	switch (rp->evt_msg.msg_id) {
	case MSG_ID_SNAPSHOT_DONE:
		rp->type = VFE_MSG_SNAPSHOT;
		break;

	case MSG_ID_OUTPUT_P:
		rp->type = VFE_MSG_OUTPUT_P;
		rp->phy.output_id = OUTPUT_TYPE_P;
		vfe_msg_get_phy_addr(&rp->phy, vfe_msg_data);
		break;

	case MSG_ID_OUTPUT_T:
		rp->type = VFE_MSG_OUTPUT_T;
		rp->phy.output_id = OUTPUT_TYPE_T;
		vfe_msg_get_phy_addr(&rp->phy, vfe_msg_data);
		break;

	case MSG_ID_OUTPUT_S:
		rp->type = VFE_MSG_OUTPUT_S;
		rp->phy.output_id = OUTPUT_TYPE_S;
		vfe_msg_get_phy_addr(&rp->phy, vfe_msg_data);
		break;

	case MSG_ID_OUTPUT_V:
		rp->type = VFE_MSG_OUTPUT_V;
		rp->phy.output_id = OUTPUT_TYPE_V;
		vfe_msg_get_phy_addr(&rp->phy, vfe_msg_data);
		break;

	case MSG_ID_STATS_AF:
		rp->type = VFE_MSG_STATS_AF;
		rp->phy.sbuf_phy = vfe_msg_data->_u.msgStats.buffer;
		break;

	case MSG_ID_STATS_AWB:
		rp->type = VFE_MSG_STATS_AWB;
		rp->phy.sbuf_phy = vfe_msg_data->_u.msgStats.buffer;
		break;

	case MSG_ID_STATS_AEC:
		rp->type = VFE_MSG_STATS_AEC;
		rp->phy.sbuf_phy = vfe_msg_data->_u.msgStats.buffer;
		break;

	default:
		rp->type = VFE_MSG_GENERAL;
		break;
	}
	vfe31_ctrl->resp->vfe_resp(rp, MSM_CAM_Q_VFE_MSG, vfe31_ctrl->syncdata,
		GFP_ATOMIC);
}

static void vfe_send_outmsg(uint8_t msgid, uint32_t pyaddr, uint32_t pcbcraddr)
{
	struct vfe_message msg;
	msg._d = msgid;
	msg._u.msgOut.yBuffer     = pyaddr;
	msg._u.msgOut.cbcrBuffer  = pcbcraddr;
	vfe31_proc_ops(msgid, &msg, sizeof(struct vfe_message));
}

static int vfe31_enable(struct camera_enable_cmd *enable)
{
	return 0;
}

void vfe_stop(void)
{
	uint8_t axiBusyFlag = true;
	unsigned long flags;

	/* for reset hw modules, and send msg when reset_irq comes. */
	spin_lock_irqsave(&vfe31_ctrl->stop_flag_lock, flags);
	vfe31_ctrl->stop_ack_pending = TRUE;
	spin_unlock_irqrestore(&vfe31_ctrl->stop_flag_lock, flags);

	/* disable all interrupts.  */
	vfe_io_w(VFE_DISABLE_ALL_IRQS, VFE_IRQ_MASK_0);
	vfe_io_w(VFE_DISABLE_ALL_IRQS, VFE_IRQ_MASK_1);

	/* clear all pending interrupts*/
	vfe_io_w(VFE_CLEAR_ALL_IRQS, VFE_IRQ_CLEAR_0);
	vfe_io_w(VFE_CLEAR_ALL_IRQS, VFE_IRQ_CLEAR_1);
	vfe_io_w(1, VFE_IRQ_CMD);

	/* in either continuous or snapshot mode, stop command can be issued
	 * at any time. stop camif immediately. */
	vfe_io_w(CAMIF_COMMAND_STOP_IMMEDIATELY, VFE_CAMIF_COMMAND);

	/* axi halt command. */
	vfe_io_w(AXI_HALT, VFE_AXI_CMD);

	while (axiBusyFlag) {
		if (vfe_io_r(VFE_AXI_STATUS) & 0x1) {
			axiBusyFlag = false;
		}
	}
	vfe_io_w(AXI_HALT_CLEAR, VFE_AXI_CMD);

	/* after axi halt, then ok to apply global reset. */
	/* enable reset_ack and async timer interrupt only while
	stopping the pipeline.*/
	vfe_io_w(0xf0000000, VFE_IRQ_MASK_0);
	vfe_io_w(VFE_IMASK_WHILE_STOPPING_1, VFE_IRQ_MASK_1);
	vfe_io_w(VFE_RESET_UPON_STOP_CMD, VFE_GLOBAL_RESET);
}

static int vfe31_disable(struct camera_enable_cmd *enable,
	struct platform_device *dev)
{
	vfe_stop();
	msm_camio_disable(dev);
	return 0;
}

static void vfe31_release(struct platform_device *pdev)
{
	struct msm_sensor_ctrl *sctrl =
		&((struct msm_sync *)vfe_syncdata)->sctrl;
	struct resource *vfemem, *vfeio;

	if (sctrl)
		sctrl->s_release();

	vfemem = vfe31_ctrl->vfemem;
	vfeio  = vfe31_ctrl->vfeio;

	kfree(vfe31_ctrl->extdata);
	free_irq(vfe31_ctrl->vfeirq, 0);
	iounmap(vfe31_ctrl->vfebase);
	kfree(vfe31_ctrl);
	vfe31_ctrl = NULL;
	release_mem_region(vfemem->start, (vfemem->end - vfemem->start) + 1);
	msm_camio_disable(pdev);
	vfe_syncdata = NULL;
}

static int vfe31_config_axi(int mode, struct axidata *ad, uint32_t *ao)
{
	int i;
	uint32_t *p, *p1, *p2;
	struct vfe31_output_ch *outp1 = NULL, *outp2 = NULL;
	struct msm_pmem_region *regp1 = NULL, *regp2 = NULL;

	p = ao + 2;

	CDBG("vfe31_config_axi: mode = %d, bufnum1 = %d, bufnum2 = %d\n",
		mode, ad->bufnum1, ad->bufnum2);

	switch (mode) {
	case OUTPUT_2:
		if (ad->bufnum2 != 3)
			return -EINVAL;
		*p = 0x200;    /* preview with wm0 & wm1 */

		vfe31_ctrl->outpath.out0.ch0 = 0; /* luma   */
		vfe31_ctrl->outpath.out0.ch1 = 1; /* chroma */
		regp1 = &(ad->region[ad->bufnum1]);
		outp1 = &(vfe31_ctrl->outpath.out0);
		vfe31_ctrl->outpath.output_mode |= VFE31_OUTPUT_MODE_PT;

		for (i = 0; i < 2; i++) {
			p1 = ao + 6 + i;    /* wm0 for y  */
			*p1 = (regp1->paddr + regp1->info.y_off);

			p1 = ao + 12 + i;  /* wm1 for cbcr */
			*p1 = (regp1->paddr + regp1->info.cbcr_off);
			regp1++;
		}
		outp1->free_buf.available = 1;
		outp1->free_buf.paddr = regp1->paddr;
		outp1->free_buf.y_off = regp1->info.y_off;
		outp1->free_buf.cbcr_off = regp1->info.cbcr_off;

		CDBG("vfe31_config_axi: free_buf paddr = 0x%x, y_off = %d,"
			" cbcr_off = %d\n",
			outp1->free_buf.paddr, outp1->free_buf.y_off,
			outp1->free_buf.cbcr_off);
		break;

	case OUTPUT_1_AND_2:
		/* use wm0& 4 for thumbnail, wm1&5 for main image.*/
		if ((ad->bufnum1 < 1) || (ad->bufnum2 < 1))
			return -EINVAL;
		/* at least one frame for snapshot.  */
		*p++ = 0x1;    /* xbar cfg0 */
		*p = 0x203;    /* xbar cfg1 */
		vfe31_ctrl->outpath.out0.ch0 = 0; /* thumbnail luma   */
		vfe31_ctrl->outpath.out0.ch1 = 4; /* thumbnail chroma */
		vfe31_ctrl->outpath.out1.ch0 = 1; /* main image luma   */
		vfe31_ctrl->outpath.out1.ch1 = 5; /* main image chroma */
		vfe31_ctrl->outpath.output_mode |=
			VFE31_OUTPUT_MODE_S;  /* main image.*/
		vfe31_ctrl->outpath.output_mode |=
			VFE31_OUTPUT_MODE_PT;  /* thumbnail. */

		regp1 = &(ad->region[0]); /* this is thumbnail buffer. */
		/* this is main image buffer. */
		regp2 = &(ad->region[ad->bufnum1]);
		outp1 = &(vfe31_ctrl->outpath.out0);
		outp2 = &(vfe31_ctrl->outpath.out1); /* snapshot */

		p1 = ao + 6;   /* wm0 ping */
		*p1++ = (regp1->paddr + regp1->info.y_off);
		/* this is to duplicate ping address to pong.*/
		*p1 = (regp1->paddr + regp1->info.y_off);
		p1 = ao + 30;  /* wm4 ping */
		*p1++ = (regp1->paddr + regp1->info.cbcr_off);
		/* this is to duplicate ping address to pong.*/
		*p1 = (regp1->paddr + regp1->info.cbcr_off);
		p1 = ao + 12;   /* wm1 ping */
		*p1++ = (regp2->paddr + regp2->info.y_off);
		/* pong = ping,*/
		*p1 = (regp2->paddr + regp2->info.y_off);
		p1 = ao + 36;  /* wm5 */
		*p1++ = (regp2->paddr + regp2->info.cbcr_off);
		*p1 = (regp2->paddr + regp2->info.cbcr_off);
		break;

	case OUTPUT_1_AND_3:
		/* use wm0& 4 for preview, wm1&5 for video.*/
		if ((ad->bufnum1 < 2) || (ad->bufnum2 < 2))
			return -EINVAL;
		*p++ = 0x1;    /* xbar cfg0 */
		*p = 0x1a03;    /* xbar cfg1 */
		vfe31_ctrl->outpath.out0.ch0 = 0; /* preview luma   */
		vfe31_ctrl->outpath.out0.ch1 = 4; /* preview chroma */
		vfe31_ctrl->outpath.out2.ch0 = 1; /* video luma     */
		vfe31_ctrl->outpath.out2.ch1 = 5; /* video chroma   */
		vfe31_ctrl->outpath.output_mode |=
			VFE31_OUTPUT_MODE_V;  /* video*/
		vfe31_ctrl->outpath.output_mode |=
			VFE31_OUTPUT_MODE_PT;  /* preview */

		regp1 = &(ad->region[0]); /* this is preview buffer. */
		regp2 = &(ad->region[ad->bufnum1]);/* this is video buffer. */
		outp1 = &(vfe31_ctrl->outpath.out0); /* preview */
		outp2 = &(vfe31_ctrl->outpath.out2); /* video */


		for (i = 0; i < 2; i++) {
			p1 = ao + 6 + i;    /* wm0 for y  */
			*p1 = (regp1->paddr + regp1->info.y_off);

			p1 = ao + 30 + i;  /* wm1 for cbcr */
			*p1 = (regp1->paddr + regp1->info.cbcr_off);
			regp1++;
		}

		for (i = 0; i < 2; i++) {
			p2 = ao + 12 + i;    /* wm0 for y  */
			*p2 = (regp2->paddr + regp2->info.y_off);

			p2 = ao + 36 + i;  /* wm1 for cbcr */
			*p2 = (regp2->paddr + regp2->info.cbcr_off);
			regp2++;
		}
		outp1->free_buf.available = 1;
		outp1->free_buf.paddr = regp1->paddr;
		outp1->free_buf.y_off = regp1->info.y_off;
		outp1->free_buf.cbcr_off = regp1->info.cbcr_off;

		outp2->free_buf.available = 1;
		outp2->free_buf.paddr = regp2->paddr;
		outp2->free_buf.y_off = regp2->info.y_off;
		outp2->free_buf.cbcr_off = regp2->info.cbcr_off;
		CDBG("vfe31_config_axi: preview free_buf"
			"paddr = 0x%x, y_off = %d, cbcr_off = %d\n",
			outp1->free_buf.paddr, outp1->free_buf.y_off,
			outp1->free_buf.cbcr_off);
		CDBG("vfe31_config_axi: video free_buf"
			"paddr = 0x%x,y_off = %d, cbcr_off = %d\n",
			outp2->free_buf.paddr, outp2->free_buf.y_off,
			outp2->free_buf.cbcr_off);
		break;

	case CAMIF_TO_AXI_VIA_OUTPUT_2:  /* use wm0 only */
		if (ad->bufnum2 < 1)
			return -EINVAL;
		CDBG("config axi for raw snapshot.\n");
		*p = 0x60;    /* raw snapshot with wm0 */
		vfe31_ctrl->outpath.out1.ch0 = 0; /* raw */
		regp1 = &(ad->region[ad->bufnum1]);
		vfe31_ctrl->outpath.output_mode |= VFE31_OUTPUT_MODE_S;
		p1 = ao + 6;    /* wm0 for y  */
		*p1 = (regp1->paddr + regp1->info.y_off);
		break;

	default:
		break;
	}
	msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[V31_AXI_OUT_CFG].offset,
		ao, vfe31_cmd[V31_AXI_OUT_CFG].length);
	return 0;
}

static void vfe31_reset_internal_variables(void)
{
	unsigned long flags;

	vfe31_ctrl->vfeImaskCompositePacked = 0;
	/* state control variables */
	vfe31_ctrl->start_ack_pending = FALSE;

	spin_lock_irqsave(&vfe31_ctrl->stop_flag_lock, flags);
	vfe31_ctrl->stop_ack_pending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->stop_flag_lock, flags);

	vfe31_ctrl->reset_ack_pending = FALSE;

	spin_lock_irqsave(&vfe31_ctrl->update_ack_lock, flags);
	vfe31_ctrl->update_ack_pending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->update_ack_lock, flags);

	vfe31_ctrl->req_stop_video_rec = FALSE;
	vfe31_ctrl->req_start_video_rec = FALSE;

	spin_lock_irqsave(&vfe31_ctrl->state_lock, flags);
	vfe31_ctrl->vstate = VFE_STATE_IDLE;
	spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);

	/* 0 for continuous mode, 1 for snapshot mode */
	vfe31_ctrl->operation_mode = 0;
	vfe31_ctrl->outpath.output_mode = 0;
	vfe31_ctrl->vfe_capture_count = 0;

	/* this is unsigned 32 bit integer. */
	vfe31_ctrl->vfeFrameId = 0;
	vfe31_ctrl->output1Pattern = 0xffffffff;
	vfe31_ctrl->output1Period  = 31;
	vfe31_ctrl->output2Pattern = 0xffffffff;
	vfe31_ctrl->output2Period  = 31;
	vfe31_ctrl->vfeFrameSkipCount   = 0;
	vfe31_ctrl->vfeFrameSkipPeriod  = 31;

	/* Stats control variables. */
	memset(&vfe31_ctrl->afStatsControl, 0,
		sizeof(struct vfe_stats_control));
	memset(&vfe31_ctrl->awbStatsControl, 0,
		sizeof(struct vfe_stats_control));
	memset(&vfe31_ctrl->aecStatsControl, 0,
		sizeof(struct vfe_stats_control));
	memset(&vfe31_ctrl->ihistStatsControl, 0,
		sizeof(struct vfe_stats_control));
	memset(&vfe31_ctrl->rsStatsControl, 0,
		sizeof(struct vfe_stats_control));
	memset(&vfe31_ctrl->csStatsControl, 0,
		sizeof(struct vfe_stats_control));
}

static void vfe31_reset(void)
{
	uint32_t vfe_version;

	vfe31_reset_internal_variables();
	vfe_version = vfe_io_r(VFE_VERSION);
	CDBG("vfe_version = 0x%x\n", vfe_version);
	/* disable all interrupts.  vfeImaskLocal is also reset to 0
	* to begin with. */
	vfe_io_w(VFE_DISABLE_ALL_IRQS, VFE_IRQ_MASK_0);
	vfe_io_w(VFE_DISABLE_ALL_IRQS, VFE_IRQ_MASK_1);

	/* clear all pending interrupts*/
	vfe_io_w(VFE_CLEAR_ALL_IRQS, VFE_IRQ_CLEAR_0);
	vfe_io_w(VFE_CLEAR_ALL_IRQS, VFE_IRQ_CLEAR_1);
	vfe_io_w(1, VFE_IRQ_CMD);

	/* enable reset_ack interrupt.  */
	vfe_io_w(VFE_IMASK_WHILE_STOPPING_1, VFE_IRQ_MASK_1);

	/* Write to VFE_GLOBAL_RESET_CMD to reset the vfe hardware. Once reset
	 * is done, hardware interrupt will be generated.  VFE ist processes
	 * the interrupt to complete the function call.  Note that the reset
	 * function is synchronous. */
	vfe_io_w(VFE_RESET_UPON_RESET_CMD, VFE_GLOBAL_RESET);
}

static int vfe31_operation_config(uint32_t *cmd)
{
	uint32_t *p = cmd;

	vfe31_ctrl->operation_mode = *p;
	vfe31_ctrl->stats_comp = *(++p);

	vfe_io_w(*(++p), VFE_CFG_OFF);
	vfe_io_w(*(++p), VFE_MODULE_CFG);
	vfe_io_w(*(++p), VFE_REALIGN_BUF);
	vfe_io_w(*(++p), VFE_CHROMA_UP);
	vfe_io_w(*(++p), VFE_STATS_CFG);
	return 0;
}

static uint32_t vfe_stats_awb_buf_init(struct vfe_cmd_stats_buf *in)
{
	uint32_t *ptr = in->statsBuf;
	uint32_t addr;

	addr = ptr[0];
	vfe_io_w(addr, VFE_BUS_STATS_AWB_WR_PING_ADDR);
	addr = ptr[1];
	vfe_io_w(addr, VFE_BUS_STATS_AWB_WR_PONG_ADDR);
	vfe31_ctrl->awbStatsControl.nextFrameAddrBuf = in->statsBuf[2];
	return 0;
}

static uint32_t vfe_stats_aec_buf_init(struct vfe_cmd_stats_buf *in)
{
	uint32_t *ptr = in->statsBuf;
	uint32_t addr;

	addr = ptr[0];
	vfe_io_w(addr, VFE_BUS_STATS_AEC_WR_PING_ADDR);
	addr = ptr[1];
	vfe_io_w(addr, VFE_BUS_STATS_AEC_WR_PONG_ADDR);
	vfe31_ctrl->aecStatsControl.nextFrameAddrBuf = in->statsBuf[2];
	return 0;
}

static uint32_t vfe_stats_af_buf_init(struct vfe_cmd_stats_buf *in)
{
	uint32_t *ptr = in->statsBuf;
	uint32_t addr;

	addr = ptr[0];
	vfe_io_w(addr, VFE_BUS_STATS_AF_WR_PING_ADDR);
	addr = ptr[1];
	vfe_io_w(addr, VFE_BUS_STATS_AF_WR_PONG_ADDR);
	vfe31_ctrl->afStatsControl.nextFrameAddrBuf = in->statsBuf[2];
	return 0;
}

static uint32_t vfe_stats_ihist_buf_init(struct vfe_cmd_stats_buf *in)
{
	uint32_t *ptr = in->statsBuf;
	uint32_t addr;

	addr = ptr[0];
	vfe_io_w(addr, VFE_BUS_STATS_HIST_WR_PING_ADDR);
	addr = ptr[1];
	vfe_io_w(addr, VFE_BUS_STATS_HIST_WR_PONG_ADDR);
	vfe31_ctrl->ihistStatsControl.nextFrameAddrBuf = in->statsBuf[2];
	return 0;
}

static uint32_t vfe_stats_rs_buf_init(struct vfe_cmd_stats_buf *in)
{
	uint32_t *ptr = in->statsBuf;
	uint32_t addr;

	addr = ptr[0];
	vfe_io_w(addr, VFE_BUS_STATS_RS_WR_PING_ADDR);
	addr = ptr[1];
	vfe_io_w(addr, VFE_BUS_STATS_RS_WR_PONG_ADDR);
	vfe31_ctrl->rsStatsControl.nextFrameAddrBuf = in->statsBuf[2];
	return 0;
}

static uint32_t vfe_stats_cs_buf_init(struct vfe_cmd_stats_buf *in)
{
	uint32_t *ptr = in->statsBuf;
	uint32_t addr;

	addr = ptr[0];
	vfe_io_w(addr, VFE_BUS_STATS_CS_WR_PING_ADDR);
	addr = ptr[1];
	vfe_io_w(addr, VFE_BUS_STATS_CS_WR_PONG_ADDR);
	vfe31_ctrl->csStatsControl.nextFrameAddrBuf = in->statsBuf[2];
	return 0;
}

static void vfe31_start_common(void)
{
	unsigned long flags;

	vfe31_ctrl->start_ack_pending = TRUE;
	CDBG("VFE opertaion mode = 0x%x.\n", vfe31_ctrl->operation_mode);
	CDBG("VFE output path out mode = 0x%x.\n",
		vfe31_ctrl->outpath.output_mode);
	vfe_io_w(0x00EFE021, VFE_IRQ_MASK_0);
	vfe_io_w(VFE_IMASK_WHILE_STOPPING_1, VFE_IRQ_MASK_1);

	vfe_io_w(1, VFE_REG_UPDATE_CMD);
	vfe_io_w(1, VFE_CAMIF_COMMAND);

	spin_lock_irqsave(&vfe31_ctrl->state_lock, flags);
	vfe31_ctrl->vstate = VFE_STATE_ACTIVE;
	spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);
}

static int vfe31_start_recording(void)
{
	vfe31_ctrl->req_start_video_rec = TRUE;
	return 0;
}

static int vfe31_stop_recording(void)
{
	vfe31_ctrl->req_stop_video_rec = TRUE;
	return 0;
}

static int vfe31_capture(uint32_t num_frames_capture)
{
	uint32_t irq_comp_mask = 0;

	/* capture command is valid for both idle and active state. */
	vfe31_ctrl->outpath.out1.capture_cnt = num_frames_capture;
	if (vfe31_ctrl->operation_mode == 1) {
		vfe31_ctrl->outpath.out0.capture_cnt = num_frames_capture;
	}
	vfe31_ctrl->vfe_capture_count = num_frames_capture;
	irq_comp_mask = vfe_io_r(VFE_IRQ_COMP_MASK);

	if (vfe31_ctrl->operation_mode == 1) {
		if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_PT) {
			irq_comp_mask |= (0x1 << vfe31_ctrl->outpath.out0.ch0 |
					0x1 << vfe31_ctrl->outpath.out0.ch1);
		}
		if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
			irq_comp_mask |=
				(0x1 << (vfe31_ctrl->outpath.out1.ch0 + 8) |
				0x1 << (vfe31_ctrl->outpath.out1.ch1 + 8));
		}
		if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_PT) {
			vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out0.ch0);
			vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out0.ch1);
		}
		if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
			vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out1.ch0);
			vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out1.ch1);
		}
	} else {  /* this is raw snapshot mode. */
		CDBG("config the comp imask for raw snapshot mode.\n");
		if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
			irq_comp_mask |=
				(0x1 << (vfe31_ctrl->outpath.out1.ch0 + 8));
			vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out1.ch0);
		}
	}
	vfe_io_w(irq_comp_mask, VFE_IRQ_COMP_MASK);
	vfe_io_r(VFE_IRQ_COMP_MASK);
	vfe31_start_common();
	vfe_io_r(VFE_IRQ_COMP_MASK);
	/* for debug */
	vfe_io_w(1, 0x18C);
	vfe_io_w(1, 0x188);
	return 0;
}

static int vfe31_start(void)
{
	uint32_t irq_comp_mask = 0;

	/* start command now is only good for continuous mode. */
	if (vfe31_ctrl->operation_mode & 1)
		return 0;
	irq_comp_mask = vfe_io_r(VFE_IRQ_COMP_MASK);

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_PT) {
		irq_comp_mask |= (0x1 << vfe31_ctrl->outpath.out0.ch0 |
			0x1 << vfe31_ctrl->outpath.out0.ch1);
	}

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_V) {
		irq_comp_mask |= (0x1 << (vfe31_ctrl->outpath.out2.ch0 + 16) |
			0x1 << (vfe31_ctrl->outpath.out2.ch1 + 16));
	}

	vfe_io_w(irq_comp_mask, VFE_IRQ_COMP_MASK);

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_PT) {
		vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out0.ch0);
		vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out0.ch1);
	}
	vfe31_start_common();
	return 0;
}

static void vfe31_update(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->update_ack_lock, flags);
	vfe31_ctrl->update_ack_pending = TRUE;
	spin_unlock_irqrestore(&vfe31_ctrl->update_ack_lock, flags);
	vfe_io_w(1, VFE_REG_UPDATE_CMD);
}

void vfe31_program_dmi_cfg(enum VFE31_DMI_RAM_SEL bankSel)
{
	/* set bit 8 for auto increment. */
	uint32_t value = VFE_DMI_CFG_DEFAULT;
	value += (uint32_t)bankSel;

	vfe_io_w(value, VFE_DMI_CFG);
	/* by default, always starts with offset 0. */
	vfe_io_w(0, VFE_DMI_ADDR);
}

void vfe31_write_gamma_cfg(enum VFE31_DMI_RAM_SEL channel_sel,
						const uint32_t *tbl)
{
	int i;
	uint32_t value, value1, value2;

	vfe31_program_dmi_cfg(channel_sel);
	/* for loop for extracting init table. */
	for (i = 0 ; i < (VFE31_GAMMA_NUM_ENTRIES/2) ; i++) {
		value = *tbl++;
		value1 = value & 0x0000FFFF;
		value2 = (value & 0xFFFF0000)>>16;
		vfe_io_w(value1, VFE_DMI_DATA_LO);
		vfe_io_w(value2, VFE_DMI_DATA_LO);
	}
	vfe31_program_dmi_cfg(NO_MEM_SELECTED);
}

static int vfe31_proc_general(struct msm_vfe31_cmd *cmd)
{
	int i, rc = 0;
	uint32_t old_val = 0, new_val = 0;
	uint32_t *cmdp = NULL;
	uint32_t *cmdp_local = NULL;

	CDBG("vfe31_proc_general: cmdID = %d, length = %d\n",
		cmd->id, cmd->length);
	switch (cmd->id) {
	case V31_RESET:
		vfe31_reset();
		break;
	case V31_START:
		rc = vfe31_start();
		break;
	case V31_UPDATE:
		vfe31_update();
		break;
	case V31_CAPTURE:
		rc = vfe31_capture(1);
		break;
	case V31_START_RECORDING:
		rc = vfe31_start_recording();
		break;
	case V31_STOP_RECORDING:
		rc = vfe31_stop_recording();
		break;
	case V31_OPERATION_CFG:
		if (cmd->length != V31_OPERATION_CFG_LEN) {
			rc = -EINVAL;
			goto proc_general_done;
		}
		cmdp = kmalloc(V31_OPERATION_CFG_LEN, GFP_KERNEL);
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				V31_OPERATION_CFG_LEN)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		rc = vfe31_operation_config(cmdp);
		break;

	case V31_STATS_AE_START:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val |= AE_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);
		break;

	case V31_STATS_AF_START:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val |= AF_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);
		break;

	case V31_STATS_AWB_START:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val |= AWB_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);
		break;

	case V31_STATS_IHIST_START:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val |= IHIST_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);
		break;


	case V31_STATS_RS_START:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val |= RS_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);
		break;

	case V31_STATS_CS_START:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val |= CS_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);
		break;

	case V31_MCE_UPDATE:
	case V31_MCE_CFG:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		/* Incrementing with 4 so as to point to the 2nd Register as
		the 2nd register has the mce_enable bit */
		old_val = vfe_io_r(V31_CHROMA_EN_OFF + 4);
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		new_val = *cmdp_local;
		old_val &= MCE_EN_MASK;
		new_val = new_val | old_val;
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_CHROMA_EN_OFF + 4,
			      &new_val, 4);
		cmdp_local += 1;

		old_val = vfe_io_r(V31_CHROMA_EN_OFF + 8);
		new_val = *cmdp_local;
		old_val &= MCE_Q_K_MASK;
		new_val = new_val | old_val;
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_CHROMA_EN_OFF + 8,
			      &new_val, 4);
		cmdp_local += 1;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp_local, vfe31_cmd[cmd->id].length);
		break;

	case V31_DEMOSAIC_2_UPDATE: /* 38 BPC update   */
	case V31_DEMOSAIC_2_CFG:  /* 14 BPC config   */
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		new_val = *cmdp_local;

		old_val = vfe_io_r(V31_DEMOSAIC_0_OFF);
		old_val &= BPC_MASK;

		new_val = new_val | old_val;
		*cmdp_local = new_val;
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_DEMOSAIC_0_OFF,
			      cmdp_local, 4);
		cmdp_local += 1;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp_local, vfe31_cmd[cmd->id].length);
		break;

	case V31_DEMOSAIC_1_UPDATE:/* 37 ABF update  */
	case V31_DEMOSAIC_1_CFG: /* 13 ABF config  */
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		new_val = *cmdp_local;

		old_val = vfe_io_r(V31_DEMOSAIC_0_OFF);
		old_val &= ABF_MASK;
		new_val = new_val | old_val;
		*cmdp_local = new_val;

		msm_io_memcpy(vfe31_ctrl->vfebase + V31_DEMOSAIC_0_OFF,
			      cmdp_local, 4);

		cmdp_local += 1;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp_local, vfe31_cmd[cmd->id].length);
		break;

	case V31_ROLL_OFF_CFG:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp_local, 16);
		cmdp_local += 4;
		vfe31_program_dmi_cfg(ROLLOFF_RAM);
		/* for loop for extrcting init table. */
		for (i = 0 ; i < (VFE31_ROLL_OFF_INIT_TABLE_SIZE * 2) ; i++) {
			vfe_io_w(*cmdp_local, VFE_DMI_DATA_LO);
			cmdp_local++;
		}
		CDBG("done writing init table\n");
		/* by default, always starts with offset 0. */
		vfe_io_w(LENS_ROLL_OFF_DELTA_TABLE_OFFSET, VFE_DMI_ADDR);
		/* for loop for extracting delta table. */
		for (i = 0 ; i < (VFE31_ROLL_OFF_DELTA_TABLE_SIZE * 2) ; i++) {
			vfe_io_w(*cmdp_local, VFE_DMI_DATA_LO);
			cmdp_local++;
		}
		vfe31_program_dmi_cfg(NO_MEM_SELECTED);
		break;

	case V31_LA_CFG:
	case V31_LA_UPDATE:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);

		/* If the value is 0x00 then write in to LUT bank0
		if the value is 0x01 then write in to LUT bank1 */
		if (*cmdp == 0x0)
			vfe31_program_dmi_cfg(LUMA_ADAPT_LUT_RAM_BANK0);
		else
			vfe31_program_dmi_cfg(LUMA_ADAPT_LUT_RAM_BANK1);
		cmdp += 1;
		/* for loop for extracting init table. */
		for (i = 0 ; i < VFE31_LA_TABLE_LENGTH ; i++) {
			vfe_io_w(*cmdp, VFE_DMI_DATA_LO);
			cmdp++;
		}
		vfe31_program_dmi_cfg(NO_MEM_SELECTED);
		cmdp -= 1;
		break;

	case V31_RGB_G_CFG:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_RGB_G_OFF, cmdp, 4);
		cmdp += 1;
		vfe31_write_gamma_cfg(RGBLUT_RAM_CH0_BANK0 , cmdp);
		vfe31_write_gamma_cfg(RGBLUT_RAM_CH1_BANK0 , cmdp);
		vfe31_write_gamma_cfg(RGBLUT_RAM_CH2_BANK0 , cmdp);
		cmdp -= 1;
		break;

	case V31_RGB_G_UPDATE:
		cmdp = kmalloc(cmd->length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp, (void __user *)cmd->value,
				cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}

		msm_io_memcpy(vfe31_ctrl->vfebase + V31_RGB_G_OFF, cmdp, 4);
		old_val = *cmdp;
		cmdp += 1;

		if (old_val) {
			vfe31_write_gamma_cfg(RGBLUT_RAM_CH0_BANK1, cmdp);
			vfe31_write_gamma_cfg(RGBLUT_RAM_CH1_BANK1, cmdp);
			vfe31_write_gamma_cfg(RGBLUT_RAM_CH2_BANK1, cmdp);
		} else {
			vfe31_write_gamma_cfg(RGBLUT_RAM_CH0_BANK0, cmdp);
			vfe31_write_gamma_cfg(RGBLUT_RAM_CH1_BANK0, cmdp);
			vfe31_write_gamma_cfg(RGBLUT_RAM_CH2_BANK0, cmdp);
		}
		cmdp -= 1;
		break;

	case V31_STATS_AWB_STOP:
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val &= ~AWB_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		break;

	case V31_STATS_AE_STOP:
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val &= ~AE_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		break;

	case V31_STATS_AF_STOP:
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val &= ~AF_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		break;

	case V31_STATS_IHIST_STOP:
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val &= ~IHIST_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		break;

	case V31_STATS_RS_STOP:
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val &= ~RS_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		break;

	case V31_STATS_CS_STOP:
		old_val = vfe_io_r(VFE_MODULE_CFG);
		old_val &= ~CS_ENABLE_MASK;
		vfe_io_w(old_val, VFE_MODULE_CFG);
		break;

	case V31_STOP:
		vfe_stop();
		break;

	default:
		if (cmd->length != vfe31_cmd[cmd->id].length)
			return -EINVAL;

		cmdp = kmalloc(vfe31_cmd[cmd->id].length, GFP_KERNEL);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}

		CHECKED_COPY_FROM_USER(cmdp);
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, vfe31_cmd[cmd->id].length);
		break;
	}

proc_general_done:
	kfree(cmdp);

	return rc;
}

static void vfe31_stats_af_ack(struct vfe_cmd_stats_ack *pAck)
{
	unsigned long flags;
	spin_lock_irqsave(&vfe31_ctrl->af_ack_lock, flags);
	vfe31_ctrl->afStatsControl.nextFrameAddrBuf = pAck->nextStatsBuf;
	vfe31_ctrl->afStatsControl.ackPending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->af_ack_lock, flags);
}

static void vfe31_stats_awb_ack(struct vfe_cmd_stats_ack *pAck)
{
	unsigned long flags;
	spin_lock_irqsave(&vfe31_ctrl->awb_ack_lock, flags);
	vfe31_ctrl->awbStatsControl.nextFrameAddrBuf = pAck->nextStatsBuf;
	vfe31_ctrl->awbStatsControl.ackPending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->awb_ack_lock, flags);
}

static void vfe31_stats_aec_ack(struct vfe_cmd_stats_ack *pAck)
{
	unsigned long flags;
	spin_lock_irqsave(&vfe31_ctrl->aec_ack_lock, flags);
	vfe31_ctrl->aecStatsControl.nextFrameAddrBuf = pAck->nextStatsBuf;
	vfe31_ctrl->aecStatsControl.ackPending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->aec_ack_lock, flags);
}

static void vfe31_stats_ihist_ack(struct vfe_cmd_stats_ack *pAck)
{
	unsigned long flags;
	spin_lock_irqsave(&vfe31_ctrl->ihist_ack_lock, flags);
	vfe31_ctrl->ihistStatsControl.nextFrameAddrBuf = pAck->nextStatsBuf;
	vfe31_ctrl->ihistStatsControl.ackPending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->ihist_ack_lock, flags);
}

static void vfe31_stats_rs_ack(struct vfe_cmd_stats_ack *pAck)
{
	unsigned long flags;
	spin_lock_irqsave(&vfe31_ctrl->rs_ack_lock, flags);
	vfe31_ctrl->rsStatsControl.nextFrameAddrBuf = pAck->nextStatsBuf;
	vfe31_ctrl->rsStatsControl.ackPending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->rs_ack_lock, flags);
}

static void vfe31_stats_cs_ack(struct vfe_cmd_stats_ack *pAck)
{
	unsigned long flags;
	spin_lock_irqsave(&vfe31_ctrl->cs_ack_lock, flags);
	vfe31_ctrl->csStatsControl.nextFrameAddrBuf = pAck->nextStatsBuf;
	vfe31_ctrl->csStatsControl.ackPending = FALSE;
	spin_unlock_irqrestore(&vfe31_ctrl->cs_ack_lock, flags);
}

static int vfe31_config(struct msm_vfe_cfg_cmd *cmd, void *data)
{
	struct msm_vfe31_cmd vfecmd;
	long rc = 0;
	uint32_t i = 0;
	struct vfe_cmd_stats_buf *scfg = NULL;
	struct msm_pmem_region *regptr = NULL;
	struct vfe_cmd_stats_ack *sack = NULL;

	if (cmd->cmd_type != CMD_FRAME_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_AEC_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_AWB_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_IHIST_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_RS_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_CS_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_AF_BUF_RELEASE) {
		if (copy_from_user(&vfecmd, (void __user *)cmd->value,
				sizeof(vfecmd))) {
			pr_err("%s %d: copy_from_user failed\n", __func__,
				__LINE__);
			return -EFAULT;
		}
	} else if (cmd->cmd_type != CMD_FRAME_BUF_RELEASE) {
		/* This must be stats release. */
		if (!data)
			return -EFAULT;
		sack = kmalloc(sizeof(struct vfe_cmd_stats_ack), GFP_KERNEL);
		if (!sack)
			return -ENOMEM;
		sack->nextStatsBuf = *(uint32_t *)data;
	}

	CDBG("%s: cmdType = %d\n", __func__, cmd->cmd_type);

	switch (cmd->cmd_type) {
	case CMD_GENERAL:
		rc = vfe31_proc_general(&vfecmd);
		break;
	case CMD_FRAME_BUF_RELEASE: {
		struct msm_frame *b;
		unsigned long p;
		struct vfe31_free_buf *fbuf = NULL;
		if (!data)
			return -EFAULT;

		b = (struct msm_frame *)(cmd->value);
		p = *(unsigned long *)data;

		CDBG("CMD_FRAME_BUF_RELEASE b->path = %d\n", b->path);

		if (b->path & OUTPUT_TYPE_P) {
			CDBG("CMD_FRAME_BUF_RELEASE got free buffer\n");
			fbuf = &vfe31_ctrl->outpath.out0.free_buf;
		} else if (b->path & OUTPUT_TYPE_S) {
			fbuf = &vfe31_ctrl->outpath.out1.free_buf;
		} else if (b->path & OUTPUT_TYPE_V) {
			fbuf = &vfe31_ctrl->outpath.out2.free_buf;
		} else
			return -EFAULT;

		fbuf->paddr = p;
		fbuf->y_off = b->y_off;
		fbuf->cbcr_off = b->cbcr_off;
		fbuf->available = 1;
		}
		break;

	case CMD_SNAP_BUF_RELEASE:
		break;
	case CMD_STATS_AEC_BUF_RELEASE:
		vfe31_stats_aec_ack(sack);
		break;
	case CMD_STATS_AF_BUF_RELEASE:
		vfe31_stats_af_ack(sack);
		break;
	case CMD_STATS_AWB_BUF_RELEASE:
		vfe31_stats_awb_ack(sack);
		break;
	case CMD_STATS_IHIST_BUF_RELEASE:
		vfe31_stats_ihist_ack(sack);
		break;
	case CMD_STATS_RS_BUF_RELEASE:
		vfe31_stats_rs_ack(sack);
		break;
	case CMD_STATS_CS_BUF_RELEASE:
		vfe31_stats_cs_ack(sack);
		break;

	case CMD_AXI_CFG_PREVIEW: {
		struct axidata *axid;
		uint32_t *axio = NULL;
		axid = data;
		if (!axid)
			return -EFAULT;
		axio = kmalloc(vfe31_cmd[V31_AXI_OUT_CFG].length, GFP_KERNEL);
		if (!axio)
			return -ENOMEM;

		if (copy_from_user(axio, (void __user *)vfecmd.value,
				vfe31_cmd[V31_AXI_OUT_CFG].length)) {
			kfree(axio);
			return -EFAULT;
		}
		vfe31_config_axi(OUTPUT_2, axid, axio);
		kfree(axio);
	}
		break;

	case CMD_RAW_PICT_AXI_CFG: {
		struct axidata *axid;
		uint32_t *axio = NULL;
		axid = data;
		if (!axid)
			return -EFAULT;
		axio = kmalloc(vfe31_cmd[V31_AXI_OUT_CFG].length, GFP_KERNEL);
		if (!axio)
			return -ENOMEM;

		if (copy_from_user(axio, (void __user *)vfecmd.value,
				vfe31_cmd[V31_AXI_OUT_CFG].length)) {
			kfree(axio);
			return -EFAULT;
		}
		vfe31_config_axi(CAMIF_TO_AXI_VIA_OUTPUT_2, axid, axio);
		kfree(axio);
	}
		break;

	case CMD_AXI_CFG_SNAP: {
		struct axidata *axid;
		uint32_t *axio = NULL;
		axid = data;
		if (!axid)
			return -EFAULT;
		axio = kmalloc(vfe31_cmd[V31_AXI_OUT_CFG].length, GFP_KERNEL);
		if (!axio)
			return -ENOMEM;

		if (copy_from_user(axio, (void __user *)vfecmd.value,
				vfe31_cmd[V31_AXI_OUT_CFG].length)) {
			kfree(axio);
			return -EFAULT;
		}
		vfe31_config_axi(OUTPUT_1_AND_2, axid, axio);
		kfree(axio);
	}
		break;

	case CMD_AXI_CFG_VIDEO: {
		struct axidata *axid;
		uint32_t *axio = NULL;
		axid = data;
		if (!axid)
			return -EFAULT;
		axio = kmalloc(vfe31_cmd[V31_AXI_OUT_CFG].length, GFP_KERNEL);
		if (!axio)
			return -ENOMEM;

		if (copy_from_user(axio, (void __user *)vfecmd.value,
				vfe31_cmd[V31_AXI_OUT_CFG].length)) {
			kfree(axio);
			return -EFAULT;
		}
		vfe31_config_axi(OUTPUT_1_AND_3, axid, axio);
		kfree(axio);
	}
		break;

	case CMD_STATS_AF_ENABLE:
	case CMD_STATS_AWB_ENABLE:
	case CMD_STATS_IHIST_ENABLE:
	case CMD_STATS_RS_ENABLE:
	case CMD_STATS_CS_ENABLE:
	case CMD_STATS_AEC_ENABLE: {
		struct axidata *axid;
		axid = (struct axidata *)data;
		if (!axid)
			return -EFAULT;

		scfg = kmalloc(sizeof(struct vfe_cmd_stats_buf), GFP_KERNEL);
		if (!scfg)
			return -ENOMEM;
		regptr = axid->region;
		if (axid->bufnum1 > 0) {
			for (i = 0; i < axid->bufnum1; i++) {
				scfg->statsBuf[i] = (uint32_t)(regptr->paddr);
				regptr++;
			}
		}
		/* individual */
		switch (cmd->cmd_type) {
		case CMD_STATS_AEC_ENABLE:
			rc = vfe_stats_aec_buf_init(scfg);
			break;
		case CMD_STATS_AF_ENABLE:
			rc = vfe_stats_af_buf_init(scfg);
			break;
		case CMD_STATS_AWB_ENABLE:
			rc = vfe_stats_awb_buf_init(scfg);
			break;
		case CMD_STATS_IHIST_ENABLE:
			rc = vfe_stats_ihist_buf_init(scfg);
			break;
		case CMD_STATS_RS_ENABLE:
			rc = vfe_stats_rs_buf_init(scfg);
			break;
		case CMD_STATS_CS_ENABLE:
			rc = vfe_stats_cs_buf_init(scfg);
			break;
		}
	}

	default:
		rc = -EINVAL;
		break;
	}
	kfree(scfg);
	kfree(sack);
	CDBG("%s done: rc = %d\n", __func__, (int) rc);
	return rc;
}

static inline void vfe31_read_irq_status(struct vfe31_irq_status *out)
{
	memset(out, 0, sizeof(struct vfe31_irq_status));
	out->vfeIrqStatus0 = vfe_io_r(VFE_IRQ_STATUS_0);
	out->vfeIrqStatus1 = vfe_io_r(VFE_IRQ_STATUS_1);
	out->camifStatus = vfe_io_r(VFE_CAMIF_STATUS);
	CDBG("camifStatus  = 0x%x\n", out->camifStatus);
}

static void vfe31_send_msg_no_payload(enum VFE31_MESSAGE_ID id)
{
	struct vfe_message msg;

	CDBG("vfe31_send_msg_no_payload\n");
	msg._d = id;
	vfe31_proc_ops(id, &msg, 0);
}

static void vfe31_process_reg_update_irq(void)
{
	unsigned long flags;
	uint32_t temp;

	if (vfe31_ctrl->req_start_video_rec) {
		if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_V) {
			vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out2.ch0);
			vfe_io_w_axi_out_chan(1, vfe31_ctrl->outpath.out2.ch1);
		}
		vfe31_ctrl->req_start_video_rec =  FALSE;
		CDBG("start video triggered .\n");
	} else if (vfe31_ctrl->req_stop_video_rec) {
		if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_V) {
			vfe_io_w_axi_out_chan(0, vfe31_ctrl->outpath.out2.ch0);
			vfe_io_w_axi_out_chan(0, vfe31_ctrl->outpath.out2.ch1);
		}
		vfe31_ctrl->req_stop_video_rec =  FALSE;
		CDBG("stop video triggered .\n");
	}
	if (vfe31_ctrl->start_ack_pending == TRUE) {
		vfe31_send_msg_no_payload(MSG_ID_START_ACK);
		vfe31_ctrl->start_ack_pending = FALSE;
	} else {
		spin_lock_irqsave(&vfe31_ctrl->update_ack_lock, flags);
		if (vfe31_ctrl->update_ack_pending == TRUE) {
			spin_unlock_irqrestore(
				&vfe31_ctrl->update_ack_lock, flags);
			vfe31_send_msg_no_payload(MSG_ID_UPDATE_ACK);
			spin_lock_irqsave(&vfe31_ctrl->update_ack_lock, flags);
			vfe31_ctrl->update_ack_pending = FALSE;
			spin_unlock_irqrestore(
				&vfe31_ctrl->update_ack_lock, flags);
		} else {
			spin_unlock_irqrestore(
				&vfe31_ctrl->update_ack_lock, flags);
		}
	}
	if (vfe31_ctrl->operation_mode & 1) {  /* in snapshot mode */
		/* later we need to add check for live snapshot mode. */
		vfe31_ctrl->vfe_capture_count--;
		/* if last frame to be captured: */
		if (vfe31_ctrl->vfe_capture_count == 0) {
			/* stop the bus output:  write master enable = 0*/
			if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_PT) {
				vfe_io_w_axi_out_chan(0,
					vfe31_ctrl->outpath.out0.ch0);
				vfe_io_w_axi_out_chan(0,
					vfe31_ctrl->outpath.out0.ch1);
			}
			if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
				vfe_io_w_axi_out_chan(0,
					vfe31_ctrl->outpath.out1.ch0);
				vfe_io_w_axi_out_chan(0,
					vfe31_ctrl->outpath.out1.ch1);
			}
			vfe_io_w(CAMIF_COMMAND_STOP_AT_FRAME_BOUNDARY,
				VFE_CAMIF_COMMAND);

			temp = vfe_io_r(VFE_CAMIF_COMMAND);
			/* then do reg_update. */
			vfe_io_w(1, VFE_REG_UPDATE_CMD);
		}
	} /* if snapshot mode. */
}

static void vfe31_set_default_reg_values(void)
{
	vfe_io_w(0x800080, VFE_DEMUX_GAIN_0);
	vfe_io_w(0x800080, VFE_DEMUX_GAIN_1);
	vfe_io_w(0xFFFFF, VFE_CGC_OVERRIDE);

	/* default frame drop period and pattern */
	vfe_io_w(0x1f, VFE_FRAMEDROP_ENC_Y_CFG);
	vfe_io_w(0x1f, VFE_FRAMEDROP_ENC_CBCR_CFG);
	vfe_io_w(0xFFFFFFFF, VFE_FRAMEDROP_ENC_Y_PATTERN);
	vfe_io_w(0xFFFFFFFF, VFE_FRAMEDROP_ENC_CBCR_PATTERN);
	vfe_io_w(0x1f, VFE_FRAMEDROP_VIEW_Y);
	vfe_io_w(0x1f, VFE_FRAMEDROP_VIEW_CBCR);
	vfe_io_w(0xFFFFFFFF, VFE_FRAMEDROP_VIEW_Y_PATTERN);
	vfe_io_w(0xFFFFFFFF, VFE_FRAMEDROP_VIEW_CBCR_PATTERN);
	vfe_io_w(0, VFE_CLAMP_MIN);
	vfe_io_w(0xFFFFFF, VFE_CLAMP_MAX);

	/* stats UB config */
	vfe_io_w(0x3900007, VFE_BUS_STATS_AEC_UB_CFG);
	vfe_io_w(0x3980007, VFE_BUS_STATS_AF_UB_CFG);
	vfe_io_w(0x3A0000F, VFE_BUS_STATS_AWB_UB_CFG);
	vfe_io_w(0x3B00007, VFE_BUS_STATS_RS_UB_CFG);
	vfe_io_w(0x3B8001F, VFE_BUS_STATS_CS_UB_CFG);
	vfe_io_w(0x3D8001F, VFE_BUS_STATS_HIST_UB_CFG);
	vfe_io_w(0x3F80007, VFE_BUS_STATS_SKIN_UB_CFG);
}

static void vfe31_process_reset_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->state_lock, flags);
	vfe31_ctrl->vstate = VFE_STATE_IDLE;
	spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);

	spin_lock_irqsave(&vfe31_ctrl->stop_flag_lock, flags);
	if (vfe31_ctrl->stop_ack_pending) {
		vfe31_ctrl->stop_ack_pending = FALSE;
		spin_unlock_irqrestore(&vfe31_ctrl->stop_flag_lock, flags);
		vfe31_send_msg_no_payload(MSG_ID_STOP_ACK);
	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->stop_flag_lock, flags);
		/* this is from reset command. */
		vfe31_set_default_reg_values();

		/* reload all write masters. (frame & line)*/
		vfe_io_w(0x7FFF, VFE_BUS_CMD);
		vfe31_send_msg_no_payload(MSG_ID_RESET_ACK);
	}
}

static void vfe31_process_camif_sof_irq(void)
{
	uint32_t temp;

	if (vfe31_ctrl->operation_mode == 3) {  /* in raw snapshot mode */
		if (vfe31_ctrl->start_ack_pending) {
			vfe31_send_msg_no_payload(MSG_ID_START_ACK);
			vfe31_ctrl->start_ack_pending = FALSE;
		}
		vfe31_ctrl->vfe_capture_count--;
		/* if last frame to be captured: */
		if (vfe31_ctrl->vfe_capture_count == 0) {
			vfe_io_w(CAMIF_COMMAND_STOP_AT_FRAME_BOUNDARY, VFE_CAMIF_COMMAND);
			temp = vfe_io_r(VFE_CAMIF_COMMAND);
		}
	} /* if raw snapshot mode. */

	vfe31_ctrl->vfeFrameId++;
	CDBG("camif_sof_irq, frameId = %d\n", vfe31_ctrl->vfeFrameId);
}

static void vfe31_process_output_path_irq_0(void)
{
	uint32_t ping_pong;
	uint32_t pyaddr, pcbcraddr;
	uint8_t out_bool = 0;

	/* we render frames in the following conditions:
	1. Continuous mode and the free buffer is avaialable.
	2. In snapshot shot mode, free buffer is not always available.
	when pending snapshot count is <=1,  then no need to use
	free buffer.
	*/
	out_bool =
		((vfe31_ctrl->operation_mode & 1) &&
		(vfe31_ctrl->vfe_capture_count <= 1)) ||
		(vfe31_ctrl->outpath.out0.free_buf.available);
	if (out_bool) {
		ping_pong = vfe_io_r(VFE_BUS_PING_PONG_STATUS);

		/* Y channel */
		pyaddr = vfe31_get_ch_addr(ping_pong,
			vfe31_ctrl->outpath.out0.ch0);
		/* Chroma channel */
		pcbcraddr = vfe31_get_ch_addr(ping_pong,
			vfe31_ctrl->outpath.out0.ch1);

		CDBG("output path 0, pyaddr = 0x%x, pcbcraddr = 0x%x\n",
			pyaddr, pcbcraddr);
		if (vfe31_ctrl->outpath.out0.free_buf.available) {
			/* Y channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out0.ch0,
				vfe31_ctrl->outpath.out0.free_buf.paddr +
				vfe31_ctrl->outpath.out0.free_buf.y_off);
			/* Chroma channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out0.ch1,
				vfe31_ctrl->outpath.out0.free_buf.paddr +
				vfe31_ctrl->outpath.out0.free_buf.cbcr_off);
			vfe31_ctrl->outpath.out0.free_buf.available = 0;
		}
		if (vfe31_ctrl->operation_mode & 1) {
			/* will add message for multi-shot. */
			vfe31_ctrl->outpath.out0.capture_cnt--;
		} else {
			/* always send message for continous mode. */
			/* if continuous mode, this is for display. (preview) */
			vfe_send_outmsg(MSG_ID_OUTPUT_P, pyaddr, pcbcraddr);
		}
	} else {
		vfe31_ctrl->outpath.out0.frame_drop_cnt++;
		CDBG("path_irq_0 - no free buffer!\n");
	}
}

static void vfe31_process_output_path_irq_1(void)
{
	uint32_t ping_pong;
	uint32_t pyaddr, pcbcraddr;
	/* this must be snapshot main image output. */
	uint8_t out_bool;

	/* we render frames in the following conditions:
	1. Continuous mode and the free buffer is avaialable.
	2. In snapshot shot mode, free buffer is not always available.
	-- when pending snapshot count is <=1,  then no need to use
	free buffer.
	*/
	out_bool =
		((vfe31_ctrl->operation_mode & 1) &&
		 (vfe31_ctrl->vfe_capture_count <= 1)) ||
		(vfe31_ctrl->outpath.out1.free_buf.available);
	if (out_bool) {
		ping_pong = vfe_io_r(VFE_BUS_PING_PONG_STATUS);

		/* Y channel */
		pyaddr = vfe31_get_ch_addr(ping_pong,
			vfe31_ctrl->outpath.out1.ch0);
		/* Chroma channel */
		pcbcraddr = vfe31_get_ch_addr(ping_pong,
			vfe31_ctrl->outpath.out1.ch1);

		CDBG("snapshot main, pyaddr = 0x%x, pcbcraddr = 0x%x\n",
			pyaddr, pcbcraddr);
		if (vfe31_ctrl->outpath.out1.free_buf.available) {
			/* Y channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out1.ch0,
				vfe31_ctrl->outpath.out1.free_buf.paddr +
				vfe31_ctrl->outpath.out1.free_buf.y_off);
			/* Chroma channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out1.ch1,
				vfe31_ctrl->outpath.out1.free_buf.paddr +
				vfe31_ctrl->outpath.out1.free_buf.cbcr_off);
			vfe31_ctrl->outpath.out1.free_buf.available = 0;
		}

		vfe31_ctrl->outpath.out1.capture_cnt--;
	} else {
		vfe31_ctrl->outpath.out1.frame_drop_cnt++;
		CDBG("path_irq_1 - no free buffer!\n");
	}
}

static void vfe31_process_output_path_irq_2(void)
{
	uint32_t ping_pong;
	uint32_t pyaddr, pcbcraddr;
	uint8_t out_bool;

	/* we render frames in the following conditions:
	1. Continuous mode and the free buffer is avaialable.
	2. In snapshot shot mode, free buffer is not always available.
	-- when pending snapshot count is <=1,  then no need to use
	free buffer.
	*/
	out_bool =
		((vfe31_ctrl->operation_mode & 1) &&
		(vfe31_ctrl->vfe_capture_count <= 1)) ||
		(vfe31_ctrl->outpath.out2.free_buf.available);
	if (out_bool) {
		ping_pong = vfe_io_r(VFE_BUS_PING_PONG_STATUS);

		/* Y channel */
		pyaddr = vfe31_get_ch_addr(ping_pong,
			vfe31_ctrl->outpath.out2.ch0);
		/* Chroma channel */
		pcbcraddr = vfe31_get_ch_addr(ping_pong,
			vfe31_ctrl->outpath.out2.ch1);

		CDBG("video output, pyaddr = 0x%x, pcbcraddr = 0x%x\n",
			pyaddr, pcbcraddr);

		if (vfe31_ctrl->outpath.out2.free_buf.available) {
			/* Y channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out2.ch0,
				vfe31_ctrl->outpath.out2.free_buf.paddr +
				vfe31_ctrl->outpath.out2.free_buf.y_off);
			/* Chroma channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out2.ch1,
				vfe31_ctrl->outpath.out2.free_buf.paddr +
				vfe31_ctrl->outpath.out2.free_buf.cbcr_off);
			vfe31_ctrl->outpath.out2.free_buf.available = 0;
		}
		vfe_send_outmsg(MSG_ID_OUTPUT_V, pyaddr, pcbcraddr);
	} else {
		vfe31_ctrl->outpath.out2.frame_drop_cnt++;
		CDBG("path_irq_2 - no free buffer!\n");
	}
}

static void vfe31_process_stats_comb_irq(uint32_t *irqstatus)
{
	return;
}

static uint32_t vfe31_process_stats_irq_common(uint32_t statsNum,
						uint32_t newAddr) {
	uint32_t pingpongStatus;
	uint32_t returnAddr;
	uint32_t pingpongAddrOffset;

	/* must be 0=ping, 1=pong */
	pingpongStatus =
		(vfe_io_r(VFE_BUS_PING_PONG_STATUS)
		& ((uint32_t)(1<<(statsNum + 7)))) >> (statsNum + 7);
	/* stats bits starts at 7 */
	CDBG("statsNum %d, pingpongStatus %d\n", statsNum, pingpongStatus);
	pingpongAddrOffset =
		((uint32_t)(VFE_BUS_STATS_PING_PONG_BASE)) +
				(3*statsNum)*4 + (1-pingpongStatus)*4;
	returnAddr = vfe_io_r(pingpongAddrOffset);
	vfe_io_w(newAddr, pingpongAddrOffset);
	return returnAddr;
}

static void vfe_send_stats_msg(uint32_t bufAddress, uint32_t statsNum)
{
	struct vfe_message msg;

	/* fill message with right content. */
	msg._u.msgStats.frameCounter = vfe31_ctrl->vfeFrameId;
	msg._u.msgStats.buffer = bufAddress;

	switch (statsNum) {
	case statsAeNum:
		msg._d = MSG_ID_STATS_AEC;
		break;
	case statsAfNum:
		msg._d = MSG_ID_STATS_AF;
		break;
	case statsAwbNum:
		msg._d = MSG_ID_STATS_AWB;
		break;
	case statsIhistNum:
		msg._d = MSG_ID_STATS_IHIST;
		break;
	case statsRsNum:
		msg._d = MSG_ID_STATS_RS;
		break;
	case statsCsNum:
		msg._d = MSG_ID_STATS_CS;
		break;
	default:
		goto stats_done;
	}

	vfe31_proc_ops(msg._d, &msg, sizeof(struct vfe_message));
stats_done:
	return;
}

static void vfe31_process_stats_ae_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->aec_ack_lock, flags);
	if (!vfe31_ctrl->aecStatsControl.ackPending) {
		vfe31_ctrl->aecStatsControl.ackPending = TRUE;
		spin_unlock_irqrestore(&vfe31_ctrl->aec_ack_lock, flags);
		vfe31_ctrl->aecStatsControl.bufToRender =
			vfe31_process_stats_irq_common(statsAeNum,
			vfe31_ctrl->aecStatsControl.nextFrameAddrBuf);

		vfe_send_stats_msg(vfe31_ctrl->aecStatsControl.bufToRender,
						statsAeNum);
	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->aec_ack_lock, flags);
		vfe31_ctrl->aecStatsControl.droppedStatsFrameCount++;
	}

}

static void vfe31_process_stats_af_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->af_ack_lock, flags);
	if (!vfe31_ctrl->afStatsControl.ackPending) {
		vfe31_ctrl->afStatsControl.ackPending = TRUE;
		spin_unlock_irqrestore(&vfe31_ctrl->af_ack_lock, flags);
		vfe31_ctrl->afStatsControl.bufToRender =
			vfe31_process_stats_irq_common(statsAfNum,
			vfe31_ctrl->afStatsControl.nextFrameAddrBuf);

		vfe_send_stats_msg(vfe31_ctrl->afStatsControl.bufToRender,
						statsAfNum);
	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->af_ack_lock, flags);
		vfe31_ctrl->afStatsControl.droppedStatsFrameCount++;
	}

}

static void vfe31_process_stats_awb_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->awb_ack_lock, flags);
	if (!vfe31_ctrl->awbStatsControl.ackPending) {
		vfe31_ctrl->awbStatsControl.ackPending = TRUE;
		spin_unlock_irqrestore(&vfe31_ctrl->awb_ack_lock, flags);
		vfe31_ctrl->awbStatsControl.bufToRender =
			vfe31_process_stats_irq_common(statsAwbNum,
			vfe31_ctrl->awbStatsControl.nextFrameAddrBuf);

		vfe_send_stats_msg(vfe31_ctrl->awbStatsControl.bufToRender,
						statsAwbNum);
	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->awb_ack_lock, flags);
		vfe31_ctrl->awbStatsControl.droppedStatsFrameCount++;
	}

}

static void vfe31_process_stats_ihist_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->ihist_ack_lock, flags);
	if (!vfe31_ctrl->ihistStatsControl.ackPending) {
		vfe31_ctrl->ihistStatsControl.ackPending = TRUE;
		spin_unlock_irqrestore(&vfe31_ctrl->ihist_ack_lock, flags);
		vfe31_ctrl->ihistStatsControl.bufToRender =
			vfe31_process_stats_irq_common(statsIhistNum,
			vfe31_ctrl->ihistStatsControl.nextFrameAddrBuf);

		vfe_send_stats_msg(vfe31_ctrl->ihistStatsControl.bufToRender,
						statsIhistNum);
	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->ihist_ack_lock, flags);
		vfe31_ctrl->ihistStatsControl.droppedStatsFrameCount++;
	}
}


static void vfe31_process_stats_rs_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->rs_ack_lock, flags);
	if (!vfe31_ctrl->rsStatsControl.ackPending) {
		vfe31_ctrl->rsStatsControl.ackPending = TRUE;
		spin_unlock_irqrestore(&vfe31_ctrl->rs_ack_lock, flags);
		vfe31_ctrl->rsStatsControl.bufToRender =
			vfe31_process_stats_irq_common(statsRsNum,
			vfe31_ctrl->rsStatsControl.nextFrameAddrBuf);

		vfe_send_stats_msg(vfe31_ctrl->rsStatsControl.bufToRender,
						statsRsNum);
	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->rs_ack_lock, flags);
		vfe31_ctrl->rsStatsControl.droppedStatsFrameCount++;
	}
}

static void vfe31_process_stats_cs_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe31_ctrl->cs_ack_lock, flags);
	if (!vfe31_ctrl->csStatsControl.ackPending) {
		vfe31_ctrl->csStatsControl.ackPending = TRUE;
		spin_unlock_irqrestore(&vfe31_ctrl->cs_ack_lock, flags);
		vfe31_ctrl->csStatsControl.bufToRender =
			vfe31_process_stats_irq_common(statsCsNum,
			vfe31_ctrl->csStatsControl.nextFrameAddrBuf);

		vfe_send_stats_msg(vfe31_ctrl->csStatsControl.bufToRender,
						statsCsNum);
	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->cs_ack_lock, flags);
		vfe31_ctrl->csStatsControl.droppedStatsFrameCount++;
	}
}

static void vfe31_do_tasklet(unsigned long data)
{
	unsigned long flags;
	struct vfe31_isr_queue_cmd *qcmd = NULL;

	CDBG("=== vfe31_do_tasklet start ===\n");

	spin_lock_irqsave(&vfe31_ctrl->tasklet_lock, flags);
	qcmd = list_first_entry(&vfe31_ctrl->tasklet_q,
		struct vfe31_isr_queue_cmd, list);

	if (!qcmd) {
		spin_unlock_irqrestore(&vfe31_ctrl->tasklet_lock, flags);
		return;
	}

	list_del(&qcmd->list);
	spin_unlock_irqrestore(&vfe31_ctrl->tasklet_lock, flags);

	/* interrupt to be processed,  *qcmd has the payload.  */
	if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_REG_UPDATE_MASK) {
		CDBG("irq regUpdateIrq\n");
		vfe31_process_reg_update_irq();
	}

	if (qcmd->vfeInterruptStatus1 & VFE_IMASK_WHILE_STOPPING_1) {
		CDBG("irq resetAckIrq\n");
		vfe31_process_reset_irq();
	}

	spin_lock_irqsave(&vfe31_ctrl->state_lock, flags);
	if (vfe31_ctrl->vstate == VFE_STATE_ACTIVE) {
		/* irqs below are only valid when in active state. */
		spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);
		/* next, check output path related interrupts. */
		if (qcmd->vfeInterruptStatus0 &
			VFE_IRQ_STATUS0_IMAGE_COMPOSIT_DONE0_MASK) {
			vfe31_process_output_path_irq_0();
		}
		if (qcmd->vfeInterruptStatus0 &
			VFE_IRQ_STATUS0_IMAGE_COMPOSIT_DONE1_MASK) {
			vfe31_process_output_path_irq_1();
		}
		if (qcmd->vfeInterruptStatus0 &
			VFE_IRQ_STATUS0_IMAGE_COMPOSIT_DONE2_MASK) {
			vfe31_process_output_path_irq_2();
		}
		/* in snapshot mode if done then send snapshot done message */
		if (vfe31_ctrl->operation_mode & 1) {
			if ((vfe31_ctrl->outpath.out0.capture_cnt == 0) &&
				(vfe31_ctrl->outpath.out1.capture_cnt == 0)) {
				vfe31_send_msg_no_payload(MSG_ID_SNAPSHOT_DONE);
				vfe_io_w(CAMIF_COMMAND_STOP_IMMEDIATELY, VFE_CAMIF_COMMAND);
			}
		}
		/* then process stats irq. */
		if (vfe31_ctrl->stats_comp) {
			/* process stats comb interrupt. */
			if (qcmd->vfeInterruptStatus0 &
				VFE_IRQ_STATUS0_STATS_COMPOSIT_MASK) {
				CDBG("Stats composite irq occured.\n");
				vfe31_process_stats_comb_irq(
					&qcmd->vfeInterruptStatus0);
			}
		} else {
			/* process individual stats interrupt. */
			if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_STATS_AEC) {
				CDBG("Stats AEC irq occured.\n");
				vfe31_process_stats_ae_irq();
			}
			if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_STATS_AWB) {
				CDBG("Stats AWB irq occured.\n");
				vfe31_process_stats_awb_irq();
			}
			if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_STATS_AF) {
				CDBG("Stats AF irq occured.\n");
				vfe31_process_stats_af_irq();
			}
			if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_STATS_IHIST) {
				CDBG("Stats IHIST irq occured.\n");
				vfe31_process_stats_ihist_irq();
			}
			if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_STATS_RS) {
				CDBG("Stats RS irq occured.\n");
				vfe31_process_stats_rs_irq();
			}
			if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_STATS_CS) {
				CDBG("Stats CS irq occured.\n");
				vfe31_process_stats_cs_irq();
			}
		}
	} else {
		/* do we really need spin lock for state? */
		spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);
	}
	if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_CAMIF_SOF_MASK) {
		vfe31_process_camif_sof_irq();
	}
	kfree(qcmd);
	CDBG("=== vfe31_do_tasklet end ===\n");
}

DECLARE_TASKLET(vfe31_tasklet, vfe31_do_tasklet, 0);

static irqreturn_t vfe31_parse_irq(int irq_num, void *data)
{
	unsigned long flags;
	struct vfe31_irq_status irq;
	struct vfe31_isr_queue_cmd *qcmd;

	CDBG("vfe_parse_irq\n");
	vfe31_read_irq_status(&irq);

	if ((irq.vfeIrqStatus0 == 0) && (irq.vfeIrqStatus1 == 0)) {
		CDBG("vfe_parse_irq: vfeIrqStatus0 & 1 are both 0!\n");
		return IRQ_HANDLED;
	}

	qcmd = kzalloc(sizeof(struct vfe31_isr_queue_cmd), GFP_ATOMIC);
	if (!qcmd) {
		pr_err("vfe_parse_irq: qcmd malloc failed!\n");
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&vfe31_ctrl->stop_flag_lock, flags);
	if (vfe31_ctrl->stop_ack_pending) {
		irq.vfeIrqStatus0 &= VFE_IMASK_WHILE_STOPPING_0;
		irq.vfeIrqStatus1 &= VFE_IMASK_WHILE_STOPPING_1;
	}
	spin_unlock_irqrestore(&vfe31_ctrl->stop_flag_lock, flags);

	CDBG("vfe_parse_irq: Irq_status0 = 0x%x, Irq_status1 = 0x%x.\n",
		irq.vfeIrqStatus0, irq.vfeIrqStatus1);

	qcmd->vfeInterruptStatus0 = irq.vfeIrqStatus0;
	qcmd->vfeInterruptStatus1 = irq.vfeIrqStatus1;

	spin_lock_irqsave(&vfe31_ctrl->tasklet_lock, flags);
	list_add_tail(&qcmd->list, &vfe31_ctrl->tasklet_q);
	spin_unlock_irqrestore(&vfe31_ctrl->tasklet_lock, flags);
	tasklet_schedule(&vfe31_tasklet);

	/* clear the pending interrupt of the same kind.*/
	vfe_io_w(irq.vfeIrqStatus0, VFE_IRQ_CLEAR_0);
	vfe_io_w(irq.vfeIrqStatus1, VFE_IRQ_CLEAR_1);

	vfe_io_w(1, VFE_IRQ_CMD);

	return IRQ_HANDLED;
}

static int vfe31_resource_init(struct msm_vfe_callback *presp,
	struct platform_device *pdev, void *sdata)
{
	struct resource	*vfemem, *vfeirq, *vfeio;
	int rc;
	struct msm_camera_sensor_info *s_info;
	s_info = pdev->dev.platform_data;

	pdev->resource = s_info->resource;
	pdev->num_resources = s_info->num_resources;

	vfemem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!vfemem) {
		pr_err("%s: no mem resource?\n", __func__);
		return -ENODEV;
	}

	vfeirq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!vfeirq) {
		pr_err("%s: no irq resource?\n", __func__);
		return -ENODEV;
	}

	vfeio = request_mem_region(vfemem->start,
		resource_size(vfemem), pdev->name);
	if (!vfeio) {
		pr_err("%s: VFE region already claimed\n", __func__);
		return -EBUSY;
	}

	vfe31_ctrl = kzalloc(sizeof(struct vfe31_ctrl_type), GFP_KERNEL);
	if (!vfe31_ctrl) {
		rc = -ENOMEM;
		goto cmd_init_failed1;
	}

	vfe31_ctrl->vfeirq = vfeirq->start;

	vfe31_ctrl->vfebase =
		ioremap(vfemem->start, (vfemem->end - vfemem->start) + 1);
	if (!vfe31_ctrl->vfebase) {
		rc = -ENOMEM;
		pr_err("%s: vfe ioremap failed\n", __func__);
		goto cmd_init_failed2;
	}

	rc = request_irq(vfe31_ctrl->vfeirq, vfe31_parse_irq,
		IRQF_TRIGGER_RISING, "vfe", 0);
	if (rc < 0)
		goto cmd_init_failed2;

	if (presp && presp->vfe_resp)
		vfe31_ctrl->resp = presp;
	else {
		rc = -EINVAL;
		goto cmd_init_failed3;
	}

	vfe31_ctrl->extdata = kmalloc(sizeof(struct vfe31_frame_extra),
				      GFP_KERNEL);
	if (!vfe31_ctrl->extdata) {
		rc = -ENOMEM;
		goto cmd_init_failed3;
	}

	vfe31_ctrl->extlen = sizeof(struct vfe31_frame_extra);

	spin_lock_init(&vfe31_ctrl->stop_flag_lock);
	spin_lock_init(&vfe31_ctrl->state_lock);
	spin_lock_init(&vfe31_ctrl->update_ack_lock);
	spin_lock_init(&vfe31_ctrl->tasklet_lock);

	spin_lock_init(&vfe31_ctrl->aec_ack_lock);
	spin_lock_init(&vfe31_ctrl->awb_ack_lock);
	spin_lock_init(&vfe31_ctrl->af_ack_lock);
	spin_lock_init(&vfe31_ctrl->ihist_ack_lock);
	spin_lock_init(&vfe31_ctrl->rs_ack_lock);
	spin_lock_init(&vfe31_ctrl->cs_ack_lock);
	INIT_LIST_HEAD(&vfe31_ctrl->tasklet_q);

	vfe31_ctrl->syncdata = sdata;
	vfe31_ctrl->vfemem = vfemem;
	vfe31_ctrl->vfeio = vfeio;
	return 0;

cmd_init_failed3:
	free_irq(vfe31_ctrl->vfeirq, 0);
	iounmap(vfe31_ctrl->vfebase);
cmd_init_failed2:
	kfree(vfe31_ctrl);
cmd_init_failed1:
	release_mem_region(vfemem->start, (vfemem->end - vfemem->start) + 1);
	return rc;
}

static int vfe31_init(struct msm_vfe_callback *presp,
	struct platform_device *dev)
{
	int rc = 0;
	rc = vfe31_resource_init(presp, dev, vfe_syncdata);
	if (rc < 0)
		return rc;

	/* Bring up all the required GPIOs and Clocks */
	rc = msm_camio_enable(dev);
	return rc;
}

void msm_camvfe_fn_init(struct msm_camvfe_fn *fptr, void *data)
{
	fptr->vfe_init    = vfe31_init;
	fptr->vfe_enable  = vfe31_enable;
	fptr->vfe_config  = vfe31_config;
	fptr->vfe_disable = vfe31_disable;
	fptr->vfe_release = vfe31_release;
	vfe_syncdata = data;
}

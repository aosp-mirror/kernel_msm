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

#include "video_core_type.h"
#include "vcd_res_tracker.h"
#include "video_core_init.h"

#include <linux/pm_qos_params.h>
#ifdef AXI_CLK_SCALING
#include <mach/msm_reqs.h>
#endif

#define MSM_AXI_QOS_NAME "msm_vidc_reg"

#define QVGA_PERF_LEVEL (300 * 30)
#define VGA_PERF_LEVEL (1200 * 30)
#define WVGA_PERF_LEVEL (1500 * 30)

static unsigned int mfc_clk_freq_table[3] = {
	61440000, 122880000, 170667000
};

#ifndef CONFIG_MSM_NPA_SYSTEM_BUS
static unsigned int axi_clk_freq_table_enc[2] = {
	122880, 192000
};
static unsigned int axi_clk_freq_table_dec[2] = {
	122880, 192000
};
#else
static unsigned int axi_clk_freq_table_enc[2] = {
	MSM_AXI_FLOW_VIDEO_RECORDING_720P,
	MSM_AXI_FLOW_VIDEO_RECORDING_720P
};
static unsigned int axi_clk_freq_table_dec[2] = {
	MSM_AXI_FLOW_VIDEO_PLAYBACK_720P,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_720P
};
#endif
static u32 res_trk_convert_freq_to_perf_lvl(u64 n_freq)
{
	u64 n_perf_lvl;
	u64 n_temp;

	VCDRES_MSG_MED("\n %s():: n_freq = %u\n", __func__, (u32)n_freq);

	if (!n_freq)
		return 0;

	n_temp = n_freq * 1000;
	do_div(n_temp, VCD_RESTRK_HZ_PER_1000_PERFLVL);
	n_perf_lvl = (u32)n_temp;
	VCDRES_MSG_MED("\n %s(): n_perf_lvl = %u\n", __func__,
		(u32)n_perf_lvl);

	return (u32)n_perf_lvl;
}

static u32 res_trk_convert_perf_lvl_to_freq(u64 n_perf_lvl)
{
	u64 n_freq, n_temp;

	VCDRES_MSG_MED("\n %s():: n_perf_lvl = %u\n", __func__,
		(u32)n_perf_lvl);
	n_temp = (n_perf_lvl * VCD_RESTRK_HZ_PER_1000_PERFLVL) + 999;
	do_div(n_temp, 1000);
	n_freq = (u32)n_temp;
	VCDRES_MSG_MED("\n %s(): n_freq = %u\n", __func__, (u32)n_freq);

	return (u32)n_freq;
}

u32 res_trk_power_up(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_enable");
	VCDRES_MSG_LOW("clk_regime_sel_rail_control");
#ifdef AXI_CLK_SCALING
{
	int rc;
	VCDRES_MSG_MED("\n res_trk_power_up():: "
		"Calling AXI add requirement\n");
	rc = pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		MSM_AXI_QOS_NAME, PM_QOS_DEFAULT_VALUE);
	if (rc < 0)	{
		VCDRES_MSG_ERROR("Request AXI bus QOS fails. rc = %d\n",
			rc);
		return FALSE;
	}
}
#endif

#ifdef USE_RES_TRACKER
	VCDRES_MSG_MED("\n res_trk_power_up():: Calling "
		"vid_c_enable_pwr_rail()\n");
	return vid_c_enable_pwr_rail();
#endif
	return TRUE;
}

u32 res_trk_power_down(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_disable");
#ifdef AXI_CLK_SCALING
	VCDRES_MSG_MED("\n res_trk_power_down()::"
		"Calling AXI remove requirement\n");
	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		MSM_AXI_QOS_NAME);
#endif

#ifdef USE_RES_TRACKER
	VCDRES_MSG_MED("\n res_trk_power_down():: Calling "
		"vid_c_disable_pwr_rail()\n");
	return vid_c_disable_pwr_rail();
#endif
	return TRUE;
}

u32 res_trk_enable_clocks(void)
{
	VCDRES_MSG_LOW("clk_regime_msm_enable");
#ifdef USE_RES_TRACKER
	VCDRES_MSG_MED("\n res_trk_enable_clocks():: Calling "
		"vid_c_enable_clk()\n");
	return vid_c_enable_clk();
#endif
	return TRUE;
}

u32 res_trk_disable_clocks(void)
{
	VCDRES_MSG_LOW("clk_regime_msm_disable");

#ifdef USE_RES_TRACKER
	VCDRES_MSG_MED("\n res_trk_disable_clocks():: Calling "
		"vid_c_disable_clk()\n");
	return vid_c_disable_clk();
#endif
	return TRUE;
}

u32 res_trk_get_max_perf_level(u32 *pn_max_perf_lvl)
{
	if (!pn_max_perf_lvl) {
		VCDRES_MSG_ERROR("%s(): pn_max_perf_lvl is NULL\n",
			__func__);
		return FALSE;
	}

	*pn_max_perf_lvl = VCD_RESTRK_MAX_PERF_LEVEL;
	return TRUE;
}

u32 res_trk_set_perf_level(u32 n_req_perf_lvl, u32 *pn_set_perf_lvl,
	struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 axi_freq = 0, mfc_freq = 0, calc_mfc_freq = 0;
	int rc = -1;

	if (!pn_set_perf_lvl) {
		VCDRES_MSG_ERROR("%s(): pn_perf_lvl is NULL\n",
			__func__);
		return FALSE;
	}

	VCDRES_MSG_LOW("%s(), n_req_perf_lvl = %d", __func__, n_req_perf_lvl);
	if (p_cctxt) {
		calc_mfc_freq = res_trk_convert_perf_lvl_to_freq(
			(u64)n_req_perf_lvl);

		if (calc_mfc_freq < VCD_RESTRK_MIN_FREQ_POINT)
			calc_mfc_freq = VCD_RESTRK_MIN_FREQ_POINT;
		else if (calc_mfc_freq > VCD_RESTRK_MAX_FREQ_POINT)
			calc_mfc_freq = VCD_RESTRK_MAX_FREQ_POINT;

		if (!p_cctxt->b_decoding) {
			if (n_req_perf_lvl >= VGA_PERF_LEVEL) {
				mfc_freq = mfc_clk_freq_table[2];
				axi_freq = axi_clk_freq_table_enc[1];
			} else {
				mfc_freq = mfc_clk_freq_table[0];
				axi_freq = axi_clk_freq_table_enc[0];
			}
			VCDRES_MSG_HIGH("\n ENCODER: axi_freq = %u"
				", mfc_freq = %u, calc_mfc_freq = %u,"
				" n_req_perf_lvl = %u", axi_freq,
				mfc_freq, calc_mfc_freq,
				n_req_perf_lvl);
		} else {
			if (n_req_perf_lvl <= QVGA_PERF_LEVEL) {
				mfc_freq = mfc_clk_freq_table[0];
				axi_freq = axi_clk_freq_table_dec[0];
			} else {
				axi_freq = axi_clk_freq_table_dec[0];
				if (n_req_perf_lvl <= VGA_PERF_LEVEL)
					mfc_freq = mfc_clk_freq_table[0];
				else if (n_req_perf_lvl <= WVGA_PERF_LEVEL)
					mfc_freq = mfc_clk_freq_table[1];
				else {
					mfc_freq = mfc_clk_freq_table[2];
					axi_freq = axi_clk_freq_table_dec[1];
				}
			}
			VCDRES_MSG_HIGH("\n DECODER: axi_freq = %u"
				", mfc_freq = %u, calc_mfc_freq = %u,"
				" n_req_perf_lvl = %u", axi_freq,
				mfc_freq, calc_mfc_freq,
				n_req_perf_lvl);
		}
	} else {
		VCDRES_MSG_HIGH("%s() WARNING:: p_cctxt is NULL", __func__);
		return TRUE;
	}

#ifdef AXI_CLK_SCALING
    if (n_req_perf_lvl != VCD_RESTRK_MIN_PERF_LEVEL) {
		VCDRES_MSG_HIGH("\n %s(): Setting AXI freq to %u",
			__func__, axi_freq);
		rc = pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ,
			MSM_AXI_QOS_NAME, axi_freq);

		if (rc < 0)	{
			VCDRES_MSG_ERROR("\n Update AXI bus QOS fails,"
				"rc = %d\n", rc);
			return FALSE;
		}
	}
#endif

#ifdef USE_RES_TRACKER
    if (n_req_perf_lvl != VCD_RESTRK_MIN_PERF_LEVEL) {
		VCDRES_MSG_HIGH("\n %s(): Setting MFC freq to %u",
			__func__, mfc_freq);
		if (!vid_c_sel_clk_rate(mfc_freq)) {
			VCDRES_MSG_ERROR("%s(): vid_c_sel_clk_rate FAILED\n",
				__func__);
			*pn_set_perf_lvl = 0;
			return FALSE;
		}
	}
#endif

	*pn_set_perf_lvl =
	    res_trk_convert_freq_to_perf_lvl((u64) mfc_freq);
	return TRUE;
}

u32 res_trk_get_curr_perf_level(u32 *pn_perf_lvl)
{
	unsigned long n_freq;

	if (!pn_perf_lvl) {
		VCDRES_MSG_ERROR("%s(): pn_perf_lvl is NULL\n",
			__func__);
		return FALSE;
	}
	VCDRES_MSG_LOW("clk_regime_msm_get_clk_freq_hz");
	if (!vid_c_get_clk_rate(&n_freq)) {
		VCDRES_MSG_ERROR("%s(): vid_c_get_clk_rate FAILED\n",
			__func__);
		*pn_perf_lvl = 0;
		return FALSE;
	}

	*pn_perf_lvl = res_trk_convert_freq_to_perf_lvl((u64) n_freq);
	VCDRES_MSG_MED("%s(): n_freq = %lu, *pn_perf_lvl = %u", n_freq,
		*pn_perf_lvl);
	return TRUE;
}

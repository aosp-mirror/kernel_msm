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
static u32 res_trk_convert_freq_to_perf_lvl(u64 freq)
{
	u64 perf_lvl;
	u64 temp;

	pr_debug("\n %s():: freq = %u\n", __func__, (u32)freq);

	if (!freq)
		return 0;

	temp = freq * 1000;
	do_div(temp, VCD_RESTRK_HZ_PER_1000_PERFLVL);
	perf_lvl = (u32)temp;
	pr_debug("\n %s(): perf_lvl = %u\n", __func__, (u32)perf_lvl);

	return (u32)perf_lvl;
}

static u32 res_trk_convert_perf_lvl_to_freq(u64 perf_lvl)
{
	u64 freq, temp;

	pr_debug("\n %s():: perf_lvl = %u\n", __func__,
		(u32)perf_lvl);
	temp = (perf_lvl * VCD_RESTRK_HZ_PER_1000_PERFLVL) + 999;
	do_div(temp, 1000);
	freq = (u32)temp;
	pr_debug("\n %s(): freq = %u\n", __func__, (u32)freq);

	return (u32)freq;
}

u32 res_trk_power_up(void)
{
	pr_debug("clk_regime_rail_enable\n");
	pr_debug("clk_regime_sel_rail_control\n");
#ifdef AXI_CLK_SCALING
{
	int rc;
	pr_debug("\n res_trk_power_up():: "
		"Calling AXI add requirement\n");
	rc = pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		MSM_AXI_QOS_NAME, PM_QOS_DEFAULT_VALUE);
	if (rc < 0)	{
		pr_err("Request AXI bus QOS fails. rc = %d\n", rc);
		return false;
	}
}
#endif

#ifdef USE_RES_TRACKER
	pr_debug("\n res_trk_power_up():: Calling "
		"vid_c_enable_pwr_rail()\n");
	return vid_c_enable_pwr_rail();
#endif
	return true;
}

u32 res_trk_power_down(void)
{
	pr_debug("clk_regime_rail_disable\n");
#ifdef AXI_CLK_SCALING
	pr_debug("\n res_trk_power_down()::"
		"Calling AXI remove requirement\n");
	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		MSM_AXI_QOS_NAME);
#endif

#ifdef USE_RES_TRACKER
	pr_debug("\n res_trk_power_down():: Calling "
		"vid_c_disable_pwr_rail()\n");
	return vid_c_disable_pwr_rail();
#endif
	return true;
}

u32 res_trk_enable_clocks(void)
{
	pr_debug("clk_regime_msm_enable\n");
#ifdef USE_RES_TRACKER
	pr_debug("\n res_trk_enable_clocks():: Calling "
		"vid_c_enable_clk()\n");
	return vid_c_enable_clk();
#endif
	return true;
}

u32 res_trk_disable_clocks(void)
{
	pr_debug("clk_regime_msm_disable\n");

#ifdef USE_RES_TRACKER
	pr_debug("\n res_trk_disable_clocks():: Calling "
		"vid_c_disable_clk()\n");
	return vid_c_disable_clk();
#endif
	return true;
}

u32 res_trk_get_max_perf_level(u32 *pn_max_perf_lvl)
{
	if (!pn_max_perf_lvl) {
		pr_err("%s(): pn_max_perf_lvl is NULL\n", __func__);
		return false;
	}

	*pn_max_perf_lvl = VCD_RESTRK_MAX_PERF_LEVEL;
	return true;
}

u32 res_trk_set_perf_level(u32 req_perf_lvl, u32 *pn_set_perf_lvl,
	struct vcd_clnt_ctxt *cctxt)
{
	u32 axi_freq = 0, mfc_freq = 0, calc_mfc_freq = 0;

	if (!pn_set_perf_lvl) {
		pr_err("%s(): pn_perf_lvl is NULL\n", __func__);
		return false;
	}

	pr_debug("%s(), req_perf_lvl = %d\n", __func__, req_perf_lvl);
	if (cctxt) {
		calc_mfc_freq = res_trk_convert_perf_lvl_to_freq(
			(u64)req_perf_lvl);

		if (calc_mfc_freq < VCD_RESTRK_MIN_FREQ_POINT)
			calc_mfc_freq = VCD_RESTRK_MIN_FREQ_POINT;
		else if (calc_mfc_freq > VCD_RESTRK_MAX_FREQ_POINT)
			calc_mfc_freq = VCD_RESTRK_MAX_FREQ_POINT;

		if (!cctxt->decoding) {
			if (req_perf_lvl >= VGA_PERF_LEVEL) {
				mfc_freq = mfc_clk_freq_table[2];
				axi_freq = axi_clk_freq_table_enc[1];
			} else {
				mfc_freq = mfc_clk_freq_table[0];
				axi_freq = axi_clk_freq_table_enc[0];
			}
			pr_debug("\n ENCODER: axi_freq = %u"
				", mfc_freq = %u, calc_mfc_freq = %u,"
				" req_perf_lvl = %u", axi_freq,
				mfc_freq, calc_mfc_freq,
				req_perf_lvl);
		} else {
			if (req_perf_lvl <= QVGA_PERF_LEVEL) {
				mfc_freq = mfc_clk_freq_table[0];
				axi_freq = axi_clk_freq_table_dec[0];
			} else {
				axi_freq = axi_clk_freq_table_dec[0];
				if (req_perf_lvl <= VGA_PERF_LEVEL)
					mfc_freq = mfc_clk_freq_table[0];
				else if (req_perf_lvl <= WVGA_PERF_LEVEL)
					mfc_freq = mfc_clk_freq_table[1];
				else {
					mfc_freq = mfc_clk_freq_table[2];
					axi_freq = axi_clk_freq_table_dec[1];
				}
			}
			pr_debug("\n DECODER: axi_freq = %u"
				", mfc_freq = %u, calc_mfc_freq = %u,"
				" req_perf_lvl = %u", axi_freq,
				mfc_freq, calc_mfc_freq,
				req_perf_lvl);
		}
	} else {
		pr_debug("%s() WARNING:: cctxt is NULL\n", __func__);
		return true;
	}

#ifdef AXI_CLK_SCALING
	if (req_perf_lvl != VCD_RESTRK_MIN_PERF_LEVEL) {
		int rc = -1;
		pr_debug("\n %s(): Setting AXI freq to %u",
			__func__, axi_freq);
		rc = pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ,
			MSM_AXI_QOS_NAME, axi_freq);

		if (rc < 0) {
			pr_err("\n Update AXI bus QOS fails,rc = %d\n", rc);
			return false;
		}
	}
#endif

#ifdef USE_RES_TRACKER
	if (req_perf_lvl != VCD_RESTRK_MIN_PERF_LEVEL) {
		pr_debug("\n %s(): Setting MFC freq to %u",
			__func__, mfc_freq);
		if (!vid_c_sel_clk_rate(mfc_freq)) {
			pr_err("%s(): vid_c_sel_clk_rate FAILED\n", __func__);
			*pn_set_perf_lvl = 0;
			return false;
		}
	}
#endif

	*pn_set_perf_lvl =
	    res_trk_convert_freq_to_perf_lvl((u64) mfc_freq);
	return true;
}

u32 res_trk_get_curr_perf_level(u32 *pn_perf_lvl)
{
	unsigned long freq;

	if (!pn_perf_lvl) {
		pr_err("%s(): pn_perf_lvl is NULL\n", __func__);
		return false;
	}
	pr_debug("clk_regime_msm_get_clk_freq_hz\n");
	if (!vid_c_get_clk_rate(&freq)) {
		pr_err("%s(): vid_c_get_clk_rate FAILED\n", __func__);
		*pn_perf_lvl = 0;
		return false;
	}

	*pn_perf_lvl = res_trk_convert_freq_to_perf_lvl((u64) freq);
	pr_debug("%s(): freq = %lu, *pn_perf_lvl = %u\n", __func__,
		freq, *pn_perf_lvl);
	return true;
}

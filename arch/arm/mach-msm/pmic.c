/* arch/arm/mach-msm/qdsp6/pmic.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

#include "pmic.h"

#include <mach/msm_rpcrouter.h>

#define LIB_NULL_PROC 0
#define LIB_RPC_GLUE_CODE_INFO_REMOTE_PROC 1
#define LP_MODE_CONTROL_PROC 2
#define VREG_SET_LEVEL_PROC 3
#define VREG_PULL_DOWN_SWITCH_PROC 4
#define SECURE_MPP_CONFIG_DIGITAL_OUTPUT_PROC 5
#define SECURE_MPP_CONFIG_I_SINK_PROC 6
#define RTC_START_PROC 7
#define RTC_STOP_PROC 8
#define RTC_GET_TIME_PROC 9
#define RTC_ENABLE_ALARM_PROC 10
#define RTC_DISABLE_ALARM_PROC 11
#define RTC_GET_ALARM_TIME_PROC 12
#define RTC_GET_ALARM_STATUS_PROC 13
#define RTC_SET_TIME_ADJUST_PROC 14
#define RTC_GET_TIME_ADJUST_PROC 15
#define SET_LED_INTENSITY_PROC 16
#define FLASH_LED_SET_CURRENT_PROC 17
#define FLASH_LED_SET_MODE_PROC 18
#define FLASH_LED_SET_POLARITY_PROC 19
#define SPEAKER_CMD_PROC 20
#define SET_SPEAKER_GAIN_PROC 21
#define VIB_MOT_SET_VOLT_PROC 22
#define VIB_MOT_SET_MODE_PROC 23
#define VIB_MOT_SET_POLARITY_PROC 24
#define VID_EN_PROC 25
#define VID_IS_EN_PROC 26
#define VID_LOAD_DETECT_EN_PROC 27
#define MIC_EN_PROC 28
#define MIC_IS_EN_PROC 29
#define MIC_SET_VOLT_PROC 30
#define MIC_GET_VOLT_PROC 31
#define SPKR_EN_RIGHT_CHAN_PROC 32
#define SPKR_IS_RIGHT_CHAN_EN_PROC 33
#define SPKR_EN_LEFT_CHAN_PROC 34
#define SPKR_IS_LEFT_CHAN_EN_PROC 35
#define SET_SPKR_CONFIGURATION_PROC 36
#define GET_SPKR_CONFIGURATION_PROC 37
#define SPKR_GET_GAIN_PROC 38
#define SPKR_IS_EN_PROC 39
#define SPKR_EN_MUTE_PROC 40
#define SPKR_IS_MUTE_EN_PROC 41
#define SPKR_SET_DELAY_PROC 42
#define SPKR_GET_DELAY_PROC 43
#define SECURE_MPP_CONFIG_DIGITAL_INPUT_PROC 44
#define SET_SPEAKER_DELAY_PROC 45
#define SPEAKER_1K6_ZIN_ENABLE_PROC 46
#define SPKR_SET_MUX_HPF_CORNER_FREQ_PROC 47
#define SPKR_GET_MUX_HPF_CORNER_FREQ_PROC 48
#define SPKR_IS_RIGHT_LEFT_CHAN_ADDED_PROC 49
#define SPKR_EN_STEREO_PROC 50
#define SPKR_IS_STEREO_EN_PROC 51
#define SPKR_SELECT_USB_WITH_HPF_20HZ_PROC 52
#define SPKR_IS_USB_WITH_HPF_20HZ_PROC 53
#define SPKR_BYPASS_MUX_PROC 54
#define SPKR_IS_MUX_BYPASSED_PROC 55
#define SPKR_EN_HPF_PROC 56
#define SPKR_IS_HPF_EN_PROC 57
#define SPKR_EN_SINK_CURR_FROM_REF_VOLT_CIR_PROC 58
#define SPKR_IS_SINK_CURR_FROM_REF_VOLT_CIR_EN_PROC 59
#define SPKR_ADD_RIGHT_LEFT_CHAN_PROC 60
#define SPKR_SET_GAIN_PROC 61
#define SPKR_EN_PROC 62


/* rpc related */
#define PMIC_RPC_TIMEOUT (5*HZ)

#define PMIC_RPC_PROG	0x30000061
#define PMIC_RPC_VER	0x00010001

/* error bit flags defined by modem side */
#define PM_ERR_FLAG__PAR1_OUT_OF_RANGE		(0x0001)
#define PM_ERR_FLAG__PAR2_OUT_OF_RANGE		(0x0002)
#define PM_ERR_FLAG__PAR3_OUT_OF_RANGE		(0x0004)
#define PM_ERR_FLAG__PAR4_OUT_OF_RANGE		(0x0008)
#define PM_ERR_FLAG__PAR5_OUT_OF_RANGE		(0x0010)

#define PM_ERR_FLAG__ALL_PARMS_OUT_OF_RANGE   	(0x001F)

#define PM_ERR_FLAG__SBI_OPT_ERR		(0x0080)
#define PM_ERR_FLAG__FEATURE_NOT_SUPPORTED	(0x0100)

#define	PMIC_BUFF_SIZE		256

static DEFINE_MUTEX(pmic_mutex);
static struct msm_rpc_endpoint *pmic_ept;


static int modem_to_linux_err(uint err)
{
	if (err == 0)
		return 0;

	if (err & PM_ERR_FLAG__ALL_PARMS_OUT_OF_RANGE)
		return -EINVAL;

	if (err & PM_ERR_FLAG__SBI_OPT_ERR)
		return -EIO;

	if (err & PM_ERR_FLAG__FEATURE_NOT_SUPPORTED)
		return -ENOSYS;

	return -EPERM;
}


/*
 * 1) network byte order
 * 2) RPC request header(40 bytes) and RPC reply header (24 bytes)
 * 3) each transaction consists of a request and reply
 * 3) PROC (comamnd) layer has its own sub-protocol defined
 * 4) sub-protocol can be grouped to follwoing 7 cases:
 *  	a) set one argument, no get
 * 	b) set two argument, no get
 * 	c) set three argument, no get
 * 	d) set a struct, no get
 * 	e) set a argument followed by a struct, no get
 * 	f) set a argument, get a argument
 * 	g) no set, get either a argument or a struct
 */

/* Returns number of reply bytes (minus reply header size) or
 * negative value on error.
 */
static int pmic_rpc(int proc, void *msg, int msglen, void *rep, int replen)
{
	int r;
	mutex_lock(&pmic_mutex);

	if (!pmic_ept) {
		pmic_ept = msm_rpc_connect(PMIC_RPC_PROG, PMIC_RPC_VER, 0);
		if (!pmic_ept) {
			pr_err("pmic: cannot connect to rpc server\n");
			r = -ENODEV;
			goto done;
		}
	}
	r = msm_rpc_call_reply(pmic_ept, proc, msg, msglen, 
			       rep, replen, PMIC_RPC_TIMEOUT);
	if (r >= 0) {
		if (r < sizeof(struct rpc_reply_hdr)) {
			r = -EIO;
			goto done;
		}
		r -= sizeof(struct rpc_reply_hdr);
	}
done:
	mutex_unlock(&pmic_mutex);
	return r;
}

struct pmic_reply {
	struct rpc_reply_hdr hdr;
	uint32_t status;
	uint32_t data;
};
	
/**
 * pmic_rpc_set_only() - set arguments and no get
 * @data0:	first argumrnt
 * @data1:	second argument
 * @data2:	third argument
 * @data3:	fourth argument
 * @num:	number of argument
 * @proc:	command/request id
 *
 * This function covers case a, b, and c
 */
static int pmic_rpc_set_only(uint data0, uint data1, uint data2, uint data3,
			     int num, int proc)
{
	struct {
		struct rpc_request_hdr hdr;
		uint32_t data[4];
	} msg;
	struct pmic_reply rep;
	int r;

	if (num > 4)
		return -EINVAL;

	msg.data[0] = cpu_to_be32(data0);
	msg.data[1] = cpu_to_be32(data1);
	msg.data[2] = cpu_to_be32(data2);
	msg.data[3] = cpu_to_be32(data3);

	r = pmic_rpc(proc, &msg,
		     sizeof(struct rpc_request_hdr) + num * sizeof(uint32_t),
		     &rep, sizeof(rep));
	if (r < 0)
		return r;
	if (r < sizeof(uint32_t))
		return -EIO;

	return modem_to_linux_err(be32_to_cpu(rep.status));
}

/**
 * pmic_rpc_set_struct() - set the whole struct
 * @xflag:	indicates an extra argument
 * @xdata:	the extra argument
 * @*data:	starting address of struct
 * @size:	size of struct
 * @proc:	command/request id
 *
 * This fucntion covers case d and e
 */
static int pmic_rpc_set_struct(int xflag, uint xdata, uint *data, uint size,
			       int proc)
{
	struct {
		struct rpc_request_hdr hdr;
		uint32_t data[32+2];
	} msg;
	struct pmic_reply rep;
	int n = 0;

	size = (size + 3) & (~3);
	if (size > (32 * sizeof(uint32_t)))
		return -EINVAL;

	if (xflag)
		msg.data[n++] = cpu_to_be32(xdata);

	msg.data[n++] = cpu_to_be32(1);
	while (size > 0) {
		size -= 4;
		msg.data[n++] = cpu_to_be32(*data++);
	}
		
	n = pmic_rpc(proc, &msg,
		     sizeof(struct rpc_request_hdr) + n * sizeof(uint32_t),
		     &rep, sizeof(rep));
	if (n < 0)
		return n;
	if (n < sizeof(uint32_t))
		return -EIO;

	return modem_to_linux_err(be32_to_cpu(rep.status));
}

int pmic_lp_mode_control(enum switch_cmd cmd, enum vreg_lp_id id)
{
	return pmic_rpc_set_only(cmd, id, 0, 0, 2, LP_MODE_CONTROL_PROC);
}
EXPORT_SYMBOL(pmic_lp_mode_control);

int pmic_secure_mpp_control_digital_output(enum mpp_which which,
	enum mpp_dlogic_level level,
	enum mpp_dlogic_out_ctrl out)
{
	return pmic_rpc_set_only(which, level, out, 0, 3,
				SECURE_MPP_CONFIG_DIGITAL_OUTPUT_PROC);
}
EXPORT_SYMBOL(pmic_secure_mpp_control_digital_output);

int pmic_secure_mpp_config_i_sink(enum mpp_which which,
				enum mpp_i_sink_level level,
				enum mpp_i_sink_switch onoff)
{
	return pmic_rpc_set_only(which, level, onoff, 0, 3,
				SECURE_MPP_CONFIG_I_SINK_PROC);
}
EXPORT_SYMBOL(pmic_secure_mpp_config_i_sink);

int pmic_secure_mpp_config_digital_input(enum mpp_which which,
	enum mpp_dlogic_level level,
	enum mpp_dlogic_in_dbus dbus)
{
	return pmic_rpc_set_only(which, level, dbus, 0, 3,
				SECURE_MPP_CONFIG_DIGITAL_INPUT_PROC);
}
EXPORT_SYMBOL(pmic_secure_mpp_config_digital_input);

int pmic_rtc_start(struct rtc_time *time)
{
	return pmic_rpc_set_struct(0, 0, (uint *)time, sizeof(*time),
				RTC_START_PROC);
}
EXPORT_SYMBOL(pmic_rtc_start);

int pmic_rtc_stop(void)
{
	return pmic_rpc_set_only(0, 0, 0, 0, 0, RTC_STOP_PROC);
}
EXPORT_SYMBOL(pmic_rtc_stop);

int pmic_rtc_enable_alarm(enum rtc_alarm alarm,
	struct rtc_time *time)
{
	return pmic_rpc_set_struct(1, alarm, (uint *)time, sizeof(*time),
				RTC_ENABLE_ALARM_PROC);
}
EXPORT_SYMBOL(pmic_rtc_enable_alarm);

int pmic_rtc_disable_alarm(enum rtc_alarm alarm)
{
	return pmic_rpc_set_only(alarm, 0, 0, 0, 1, RTC_DISABLE_ALARM_PROC);
}
EXPORT_SYMBOL(pmic_rtc_disable_alarm);

int pmic_rtc_set_time_adjust(uint adjust)
{
	return pmic_rpc_set_only(adjust, 0, 0, 0, 1,
				RTC_SET_TIME_ADJUST_PROC);
}
EXPORT_SYMBOL(pmic_rtc_set_time_adjust);

/*
 * generic speaker
 */
int pmic_speaker_cmd(const enum spkr_cmd cmd)
{
	return pmic_rpc_set_only(cmd, 0, 0, 0, 1, SPEAKER_CMD_PROC);
}
EXPORT_SYMBOL(pmic_speaker_cmd);

int pmic_set_spkr_configuration(struct spkr_config_mode	*cfg)
{
	return pmic_rpc_set_struct(0, 0, (uint *)cfg, sizeof(*cfg),
				SET_SPKR_CONFIGURATION_PROC);
}
EXPORT_SYMBOL(pmic_set_spkr_configuration);

int pmic_spkr_en_right_chan(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, SPKR_EN_RIGHT_CHAN_PROC);
}
EXPORT_SYMBOL(pmic_spkr_en_right_chan);

int pmic_spkr_en_left_chan(uint	enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, SPKR_EN_LEFT_CHAN_PROC);
}
EXPORT_SYMBOL(pmic_spkr_en_left_chan);

int pmic_set_speaker_gain(enum spkr_gain gain)
{
	return pmic_rpc_set_only(gain, 0, 0, 0, 1, SET_SPEAKER_GAIN_PROC);
}
EXPORT_SYMBOL(pmic_set_speaker_gain);

int pmic_set_speaker_delay(enum spkr_dly delay)
{
	return pmic_rpc_set_only(delay, 0, 0, 0, 1, SET_SPEAKER_DELAY_PROC);
}
EXPORT_SYMBOL(pmic_set_speaker_delay);

int pmic_speaker_1k6_zin_enable(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1,
				SPEAKER_1K6_ZIN_ENABLE_PROC);
}
EXPORT_SYMBOL(pmic_speaker_1k6_zin_enable);

int pmic_spkr_set_mux_hpf_corner_freq(enum spkr_hpf_corner_freq	freq)
{
	return pmic_rpc_set_only(freq, 0, 0, 0, 1,
				SPKR_SET_MUX_HPF_CORNER_FREQ_PROC);
}
EXPORT_SYMBOL(pmic_spkr_set_mux_hpf_corner_freq);

int pmic_spkr_select_usb_with_hpf_20hz(uint	enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1,
				SPKR_SELECT_USB_WITH_HPF_20HZ_PROC);
}
EXPORT_SYMBOL(pmic_spkr_select_usb_with_hpf_20hz);

int pmic_spkr_bypass_mux(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, SPKR_BYPASS_MUX_PROC);
}
EXPORT_SYMBOL(pmic_spkr_bypass_mux);

int pmic_spkr_en_hpf(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, SPKR_EN_HPF_PROC);
}
EXPORT_SYMBOL(pmic_spkr_en_hpf);

int pmic_spkr_en_sink_curr_from_ref_volt_cir(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1,
				SPKR_EN_SINK_CURR_FROM_REF_VOLT_CIR_PROC);
}
EXPORT_SYMBOL(pmic_spkr_en_sink_curr_from_ref_volt_cir);

/*
 * 	speaker indexed by left_right
 */
int pmic_spkr_en(enum spkr_left_right left_right, uint enable)
{
	return pmic_rpc_set_only(left_right, enable, 0, 0, 2, SPKR_EN_PROC);
}
EXPORT_SYMBOL(pmic_spkr_en);

int pmic_spkr_set_gain(enum spkr_left_right left_right, enum spkr_gain gain)
{
	return pmic_rpc_set_only(left_right, gain, 0, 0, 2, SPKR_SET_GAIN_PROC);
}
EXPORT_SYMBOL(pmic_spkr_set_gain);

int pmic_spkr_set_delay(enum spkr_left_right left_right, enum spkr_dly delay)
{
	return pmic_rpc_set_only(left_right, delay, 0, 0, 2,
				SPKR_SET_DELAY_PROC);
}
EXPORT_SYMBOL(pmic_spkr_set_delay);

int pmic_spkr_en_mute(enum spkr_left_right left_right, uint enabled)
{
	return pmic_rpc_set_only(left_right, enabled, 0, 0, 2,
				SPKR_EN_MUTE_PROC);
}
EXPORT_SYMBOL(pmic_spkr_en_mute);

/*
 * 	mic
 */
int pmic_mic_en(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, MIC_EN_PROC);
}
EXPORT_SYMBOL(pmic_mic_en);

int pmic_mic_set_volt(enum mic_volt vol)
{
	return pmic_rpc_set_only(vol, 0, 0, 0, 1, MIC_SET_VOLT_PROC);
}
EXPORT_SYMBOL(pmic_mic_set_volt);

int pmic_vib_mot_set_volt(uint vol)
{
	return pmic_rpc_set_only(vol, 0, 0, 0, 1, VIB_MOT_SET_VOLT_PROC);
}
EXPORT_SYMBOL(pmic_vib_mot_set_volt);

int pmic_vib_mot_set_mode(enum pm_vib_mot_mode mode)
{
	return pmic_rpc_set_only(mode, 0, 0, 0, 1, VIB_MOT_SET_MODE_PROC);
}
EXPORT_SYMBOL(pmic_vib_mot_set_mode);

int pmic_vib_mot_set_polarity(enum pm_vib_mot_pol pol)
{
	return pmic_rpc_set_only(pol, 0, 0, 0, 1, VIB_MOT_SET_POLARITY_PROC);
}
EXPORT_SYMBOL(pmic_vib_mot_set_polarity);

int pmic_vid_en(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, VID_EN_PROC);
}
EXPORT_SYMBOL(pmic_vid_en);

int pmic_vid_load_detect_en(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, VID_LOAD_DETECT_EN_PROC);
}
EXPORT_SYMBOL(pmic_vid_load_detect_en);

int pmic_set_led_intensity(enum ledtype type, int level)
{
	return pmic_rpc_set_only(type, level, 0, 0, 2, SET_LED_INTENSITY_PROC);
}
EXPORT_SYMBOL(pmic_set_led_intensity);

int pmic_flash_led_set_current(const uint16_t milliamps)
{
	return pmic_rpc_set_only(milliamps, 0, 0, 0, 1,
				FLASH_LED_SET_CURRENT_PROC);
}
EXPORT_SYMBOL(pmic_flash_led_set_current);

int pmic_flash_led_set_mode(enum flash_led_mode mode)
{
	return pmic_rpc_set_only((int)mode, 0, 0, 0, 1,
				FLASH_LED_SET_MODE_PROC);
}
EXPORT_SYMBOL(pmic_flash_led_set_mode);

int pmic_flash_led_set_polarity(enum flash_led_pol pol)
{
	return pmic_rpc_set_only((int)pol, 0, 0, 0, 1,
				FLASH_LED_SET_POLARITY_PROC);
}
EXPORT_SYMBOL(pmic_flash_led_set_polarity);

int pmic_spkr_add_right_left_chan(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1,
				SPKR_ADD_RIGHT_LEFT_CHAN_PROC);
}
EXPORT_SYMBOL(pmic_spkr_add_right_left_chan);

int pmic_spkr_en_stereo(uint enable)
{
	return pmic_rpc_set_only(enable, 0, 0, 0, 1, SPKR_EN_STEREO_PROC);
}
EXPORT_SYMBOL(pmic_spkr_en_stereo);


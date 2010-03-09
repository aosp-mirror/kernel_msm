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

#ifndef _MACH_MSM_QDSP6_ADIE_
#define _MACH_MSM_QDSP6_ADIE_

#include "../dal.h"

#define ADIE_DAL_DEVICE		0x02000029
#define ADIE_DAL_PORT		"SMD_DAL_AM_AUD"

enum {
	ADIE_OP_GET_NUM_PATHS = DAL_OP_FIRST_DEVICE_API,
	ADIE_OP_GET_ALL_PATH_IDS,
	ADIE_OP_SET_PATH,
	ADIE_OP_GET_NUM_PATH_FREQUENCY_PLANS,
	ADIE_OP_GET_PATH_FREQUENCY_PLANS,
	ADIE_OP_SET_PATH_FREQUENCY_PLAN,
	ADIE_OP_PROCEED_TO_STAGE,
	ADIE_OP_MUTE_PATH
};

/* Path IDs for normal operation. */
#define ADIE_PATH_HANDSET_TX			0x010740f6
#define ADIE_PATH_HANDSET_RX			0x010740f7
#define ADIE_PATH_HEADSET_MONO_TX		0x010740f8
#define ADIE_PATH_HEADSET_STEREO_TX		0x010740f9
#define ADIE_PATH_HEADSET_MONO_RX		0x010740fa
#define ADIE_PATH_HEADSET_STEREO_RX		0x010740fb
#define ADIE_PATH_SPEAKER_TX			0x010740fc
#define ADIE_PATH_SPEAKER_RX			0x010740fd
#define ADIE_PATH_SPEAKER_STEREO_RX		0x01074101

/* Path IDs used for TTY */
#define ADIE_PATH_TTY_HEADSET_TX		0x010740fe
#define ADIE_PATH_TTY_HEADSET_RX		0x010740ff

/* Path IDs used by Factory Test Mode. */
#define ADIE_PATH_FTM_MIC1_TX			0x01074108
#define ADIE_PATH_FTM_MIC2_TX			0x01074107
#define ADIE_PATH_FTM_HPH_L_RX			0x01074106
#define ADIE_PATH_FTM_HPH_R_RX			0x01074104
#define ADIE_PATH_FTM_EAR_RX			0x01074103
#define ADIE_PATH_FTM_SPKR_RX			0x01074102

/* Path IDs for Loopback */
/* Path IDs used for Line in -> AuxPGA -> Line Out Stereo Mode*/
#define ADIE_PATH_AUXPGA_LINEOUT_STEREO_LB	0x01074100
/* Line in -> AuxPGA -> LineOut Mono */
#define ADIE_PATH_AUXPGA_LINEOUT_MONO_LB	0x01073d82
/* Line in -> AuxPGA -> Stereo Headphone */
#define ADIE_PATH_AUXPGA_HDPH_STEREO_LB		0x01074109
/* Line in -> AuxPGA -> Mono Headphone */
#define ADIE_PATH_AUXPGA_HDPH_MONO_LB		0x01073d85
/* Line in -> AuxPGA -> Earpiece */
#define ADIE_PATH_AUXPGA_EAP_LB			0x01073d81
/* Line in -> AuxPGA -> AuxOut */
#define ADIE_PATH_AUXPGA_AUXOUT_LB		0x01073d86

/* Concurrency Profiles */
#define ADIE_PATH_SPKR_STEREO_HDPH_MONO_RX	0x01073d83
#define ADIE_PATH_SPKR_MONO_HDPH_MONO_RX	0x01073d84
#define ADIE_PATH_SPKR_MONO_HDPH_STEREO_RX	0x01073d88
#define ADIE_PATH_SPKR_STEREO_HDPH_STEREO_RX	0x01073d89

/* stages */
#define ADIE_STAGE_PATH_OFF			0x0050
#define ADIE_STAGE_DIGITAL_READY		0x0100
#define ADIE_STAGE_DIGITAL_ANALOG_READY		0x1000
#define ADIE_STAGE_ANALOG_OFF			0x0750
#define ADIE_STAGE_DIGITAL_OFF			0x0600

/* path types */
#define ADIE_PATH_RX		0
#define ADIE_PATH_TX		1
#define ADIE_PATH_LOOPBACK	2

/* mute states */
#define ADIE_MUTE_OFF		0
#define ADIE_MUTE_ON		1


#endif

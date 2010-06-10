/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __ADSP_MODULE_AFE_H
#define __ADSP_MODULE_AFE_H

#define AFE_DEVICE_MI2S_CODEC_RX 1     /* internal codec rx path  */
#define AFE_DEVICE_MI2S_CODEC_TX 2     /* internal codec tx path  */
#define AFE_DEVICE_AUX_CODEC_RX  3     /* external codec rx path  */
#define AFE_DEVICE_AUX_CODEC_TX  4     /* external codec tx path  */
#define AFE_DEVICE_MI2S_HDMI_RX  5     /* HDMI/FM block rx path   */
#define AFE_DEVICE_MI2S_HDMI_TX  6     /* HDMI/FM block tx path   */
#define AFE_DEVICE_ID_MAX        7

#define AFE_VOLUME_UNITY 0x4000 /* Q14 format */

#define AFE_CMD_CODEC_CONFIG_CMD     0x1
#define AFE_CMD_CODEC_CONFIG_LEN sizeof(struct afe_cmd_codec_config)

struct afe_cmd_codec_config{
	uint16_t cmd_id;
	uint16_t device_id;
	uint16_t activity;
	uint16_t sample_rate;
	uint16_t channel_mode;
	uint16_t volume;
	uint16_t reserved;
} __attribute__ ((packed));

#define AFE_CMD_AUX_CODEC_CONFIG_CMD 	0x3
#define AFE_CMD_AUX_CODEC_CONFIG_LEN sizeof(struct afe_cmd_aux_codec_config)

struct afe_cmd_aux_codec_config{
	uint16_t cmd_id;
	uint16_t dma_path_ctl;
	uint16_t pcm_ctl;
	uint16_t eight_khz_int_mode;
	uint16_t aux_codec_intf_ctl;
	uint16_t data_format_padding_info;
} __attribute__ ((packed));

#define AFE_MSG_CODEC_CONFIG_ACK		0x0001
#define AFE_MSG_CODEC_CONFIG_ACK_LEN	\
	sizeof(struct afe_msg_codec_config_ack)

#define AFE_MSG_CODEC_CONFIG_ENABLED 0x1
#define AFE_MSG_CODEC_CONFIG_DISABLED 0xFFFF

struct afe_msg_codec_config_ack {
	uint16_t device_id;
	uint16_t device_activity;
} __attribute__((packed));


#endif

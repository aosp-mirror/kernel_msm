/*
 * Copyright (c) 1992-2009, Code Aurora Forum. All rights reserved.
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

#ifndef __ADSP_MODULE_AUDPLAY
#define __ADSP_MODULE_AUDPLAY

#define AUDPLAY_CMD_BITSTREAM_DATA_AVAIL		0x0000
#define AUDPLAY_CMD_BITSTREAM_DATA_AVAIL_LEN	\
	sizeof(struct audplay_cmd_bitstream_data_avail)

/* Type specification of dec_data_avail message sent to AUDPLAYTASK
*/
struct audplay_cmd_bitstream_data_avail{
	/*command ID*/
	unsigned int cmd_id;

	/* Decoder ID for which message is being sent */
	unsigned int decoder_id;

	/* Start address of data in ARM global memory */
	unsigned int buf_ptr;

	/* Number of 16-bit words of bit-stream data contiguously
	* available at the above-mentioned address
	*/
	unsigned int buf_size;

	/* Partition number used by audPlayTask to communicate with DSP's RTOS
	* kernel
	*/
	unsigned int partition_number;

} __attribute__((packed));

#define AUDPLAY_CMD_CHANNEL_INFO 0x0001
#define AUDPLAY_CMD_CHANNEL_INFO_LEN \
  sizeof(struct audplay_cmd_channel_info)

struct audplay_cmd_channel_select {
  unsigned int cmd_id;
  unsigned int stream_id;
  unsigned int channel_select;
} __attribute__((packed));

struct audplay_cmd_threshold_update {
  unsigned int cmd_id;
  unsigned int threshold_update;
  unsigned int threshold_value;
} __attribute__((packed));

union audplay_cmd_channel_info {
  struct audplay_cmd_channel_select ch_select;
  struct audplay_cmd_threshold_update thr_update;
};

#define AUDPLAY_CMD_HPCM_BUF_CFG 0x0003
#define AUDPLAY_CMD_HPCM_BUF_CFG_LEN \
  sizeof(struct audplay_cmd_hpcm_buf_cfg)

struct audplay_cmd_hpcm_buf_cfg {
	unsigned int cmd_id;
	unsigned int hostpcm_config;
	unsigned int feedback_frequency;
	unsigned int byte_swap;
	unsigned int max_buffers;
	unsigned int partition_number;
} __attribute__((packed));

#define AUDPLAY_CMD_BUFFER_REFRESH 0x0004
#define AUDPLAY_CMD_BUFFER_REFRESH_LEN \
  sizeof(struct audplay_cmd_buffer_update)

struct audplay_cmd_buffer_refresh {
	unsigned int cmd_id;
	unsigned int num_buffers;
	unsigned int buf_read_count;
	unsigned int buf0_address;
	unsigned int buf0_length;
	unsigned int buf1_address;
	unsigned int buf1_length;
} __attribute__((packed));

#define AUDPLAY_CMD_BITSTREAM_DATA_AVAIL_NT2            0x0005
#define AUDPLAY_CMD_BITSTREAM_DATA_AVAIL_NT2_LEN    \
	sizeof(struct audplay_cmd_bitstream_data_avail_nt2)

/* Type specification of dec_data_avail message sent to AUDPLAYTASK
 * for NT2 */
struct audplay_cmd_bitstream_data_avail_nt2 {
	/*command ID*/
	unsigned int cmd_id;

	/* Decoder ID for which message is being sent */
	unsigned int decoder_id;

	/* Start address of data in ARM global memory */
	unsigned int buf_ptr;

	/* Number of 16-bit words of bit-stream data contiguously
	*  available at the above-mentioned address
	*/
	unsigned int buf_size;

	/* Partition number used by audPlayTask to communicate with DSP's RTOS
	* kernel
	*/
	unsigned int partition_number;

	/* bitstream write pointer */
	unsigned int dspBitstreamWritePtr;

} __attribute__((packed));

#define AUDPLAY_CMD_OUTPORT_FLUSH 0x0006

struct audplay_cmd_outport_flush {
	unsigned int cmd_id;
} __attribute__((packed));


/* messages from dsp to apps */

#define AUDPLAY_MSG_DEC_NEEDS_DATA		0x0001
#define AUDPLAY_MSG_DEC_NEEDS_DATA_MSG_LEN	\
	sizeof(audplay_msg_dec_needs_data)

struct audplay_msg_dec_needs_data {
	/* reserved*/
	unsigned int dec_id;

	/*The read pointer offset of external memory till which bitstream
	has been dmed in*/
	unsigned int adecDataReadPtrOffset;

	/*The buffer size of external memory. */
	unsigned int adecDataBufSize;

	unsigned int 	bitstream_free_len;
	unsigned int	bitstream_write_ptr;
	unsigned int	bitstream_buf_start;
	unsigned int	bitstream_buf_len;
} __attribute__((packed));

#define AUDPLAY_UP_STREAM_INFO 0x0003
#define AUDPLAY_UP_STREAM_INFO_LEN \
	sizeof(struct audplay_msg_stream_info)

struct audplay_msg_stream_info {
	unsigned int decoder_id;
	unsigned int channel_info;
	unsigned int sample_freq;
	unsigned int bitstream_info;
	unsigned int bit_rate;
} __attribute__((packed));

#define AUDPLAY_MSG_BUFFER_UPDATE 0x0004
#define AUDPLAY_MSG_BUFFER_UPDATE_LEN \
	sizeof(struct audplay_msg_buffer_update)

struct audplay_msg_buffer_update {
	unsigned int buffer_write_count;
	unsigned int num_of_buffer;
	unsigned int buf0_address;
	unsigned int buf0_length;
	unsigned int buf1_address;
	unsigned int buf1_length;
} __attribute__((packed));

#define AUDPLAY_UP_OUTPORT_FLUSH_ACK 0x0005

#define ADSP_MESSAGE_ID 0xFFFF

#endif

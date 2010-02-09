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

#define ACDB_DAL_DEVICE		0x02000069
#define ACDB_DAL_PORT		"SMD_DAL_AM_AUD"

#define ACDB_OP_IOCTL		DAL_OP_FIRST_DEVICE_API

/* ioctls */
#define ACDB_GET_DEVICE		0x0108bb92
#define ACDB_SET_DEVICE		0x0108bb93
#define ACDB_GET_STREAM		0x0108bb95
#define ACDB_SET_STREAM		0x0108bb96
#define ACDB_GET_DEVICE_TABLE	0x0108bb97
#define ACDB_GET_STREAM_TABLE	0x0108bb98

#define ACDB_RES_SUCCESS	0
#define ACDB_RES_FAILURE	-1
#define ACDB_RES_BADPARM	-2
#define ACDB_RES_BADSTATE	-3

struct acdb_cmd_device {
	uint32_t size;

	uint32_t command_id;
	uint32_t device_id;
	uint32_t network_id;
	uint32_t sample_rate_id;
	uint32_t interface_id;
	uint32_t algorithm_block_id;

	/* physical page aligned buffer */
	uint32_t total_bytes;
	uint32_t unmapped_buf;
} __attribute__((packed));

struct acdb_cmd_device_table {
	uint32_t size;

	uint32_t command_id;
	uint32_t device_id;
	uint32_t network_id;
	uint32_t sample_rate_id;

	/* physical page aligned buffer */
	uint32_t total_bytes;
	uint32_t unmapped_buf;

	uint32_t res_size;
} __attribute__((packed));

struct acdb_result {
	uint32_t dal_status;
	uint32_t size;

	uint32_t unmapped_buf;
	uint32_t used_bytes;
	uint32_t result;
} __attribute__((packed));

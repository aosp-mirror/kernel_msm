// SPDX-License-Identifier: GPL-2.0
/* sound/soc/codecs/rtk_spi_v01.c
 *
 * QMI message interface definition for RTK_SPI driver.
 *
 * Copyright (C) 2020 Google, Inc.
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

#include "rtk_spi_v01.h"

struct qmi_elem_info rtk_spi_register_notification_req_v01_ei[] = {
	{
		.data_type = QMI_EOTI,
		.array_type = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

struct qmi_elem_info rtk_spi_register_notification_resp_v01_ei[] = {
	{
		.data_type = QMI_STRUCT,
		.elem_len = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.array_type = NO_ARRAY,
		.tlv_type = 0x02,
		.offset = offsetof(
			  struct rtk_spi_register_notification_resp_v01, resp),
		.ei_array = qmi_response_type_v01_ei,
	},
	{
		.data_type = QMI_EOTI,
		.array_type = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

struct qmi_elem_info rtk_spi_error_ind_v01_ei[] = {
	{
		.data_type = QMI_UNSIGNED_1_BYTE,
		.elem_len = 1,
		.elem_size = sizeof(uint8_t),
		.array_type = NO_ARRAY,
		.tlv_type = 0x01,
		.offset = offsetof(struct rtk_spi_error_ind_v01, error_code),
	},
	{
		.data_type = QMI_EOTI,
		.array_type = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

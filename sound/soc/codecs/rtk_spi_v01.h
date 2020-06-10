// SPDX-License-Identifier: GPL-2.0
/* sound/soc/codecs/rtk_spi_v01.h
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

#ifndef RTK_SPI_V01_H
#define RTK_SPI_V01_H

#include <linux/soc/qcom/qmi.h>

#define RTK_SPI_SERVICE_ID_V01 366
#define RTK_SPI_SERVICE_VERS_V01 1

#define RTK_SPI_REGISTER_NOTIFICATION_REQ_V01  0x0000
#define RTK_SPI_REGISTER_NOTIFICATION_RESP_V01 0x0000

#define RTK_SPI_ERROR_IND_V01  0X0001


struct rtk_spi_register_notification_req_v01 {
	char placeholder;
};
#define RTK_SPI_REGISTER_NOTIFICATION_REQ_V01_MAX_MSG_LEN 0
extern struct qmi_elem_info rtk_spi_register_notification_req_v01_ei[];

struct rtk_spi_register_notification_resp_v01 {
	struct qmi_response_type_v01 resp;
};
#define RTK_SPI_REGISTER_NOTIFICATION_RESP_V01_MAX_MSG_LEN 0
extern struct qmi_elem_info rtk_spi_register_notification_resp_v01_ei[];

struct rtk_spi_error_ind_v01 {
	uint8_t error_code;
};

#define RTK_SPI_ERROR_IND_V01_MAX_MSG_LEN 1
extern struct qmi_elem_info rtk_spi_error_ind_v01_ei[];

#endif // RTK_SPI_V01_H

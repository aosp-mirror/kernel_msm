/* drivers/serial/msm_serial_hs.h
 *
 * MSM High speed uart driver
 *
 * Copyright (c) 2007-2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#ifndef __MSM_UART_HS_H__
#define __MSM_UART_HS_H__

enum msm_uart_hs_pm_e {
	MSM_UART_HS_POWER_DOWN,
	MSM_UART_HS_POWER_UP
};
unsigned int msm_uart_hs_tx_empty(int line);
void msm_uart_hs_safe_pm(int line, enum msm_uart_hs_pm_e mode);

#endif

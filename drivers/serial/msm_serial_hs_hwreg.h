/* drivers/serial/msm_serial_hs_hwreg.h
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

#ifndef MSM_SERIAL_HS_HWREG_H
#define MSM_SERIAL_HS_HWREG_H

#define UARTDM_MR1_ADDR 0x0
#define UARTDM_MR2_ADDR 0x4

/* write only register */
#define UARTDM_CSR_ADDR 0x8

/* write only register */
#define UARTDM_TF_ADDR 0x70
#define UARTDM_TF2_ADDR 0x74
#define UARTDM_TF3_ADDR 0x78
#define UARTDM_TF4_ADDR 0x7C

/* write only register */
#define UARTDM_CR_ADDR 0x10
/* write only register */
#define UARTDM_IMR_ADDR 0x14

#define UARTDM_IPR_ADDR 0x18
#define UARTDM_TFWR_ADDR 0x1c
#define UARTDM_RFWR_ADDR 0x20
#define UARTDM_HCR_ADDR 0x24
#define UARTDM_DMRX_ADDR 0x34
#define UARTDM_IRDA_ADDR 0x38
#define UARTDM_DMEN_ADDR 0x3c

/* UART_DM_NO_CHARS_FOR_TX */
#define UARTDM_NCF_TX_ADDR 0x40

#define UARTDM_BADR_ADDR 0x44

#define UARTDM_SIM_CFG_ADDR 0x80

/* Read Only register */
#define UARTDM_SR_ADDR 0x8

/* Read Only register */
#define UARTDM_RF_ADDR  0x70
#define UARTDM_RF2_ADDR 0x74
#define UARTDM_RF3_ADDR 0x78
#define UARTDM_RF4_ADDR 0x7C

/* Read Only register */
#define UARTDM_MISR_ADDR 0x10

/* Read Only register */
#define UARTDM_ISR_ADDR 0x14
#define UARTDM_RX_TOTAL_SNAP_ADDR 0x38

#define UARTDM_RXFS_ADDR 0x50

/* Register field Mask Mapping */
#define UARTDM_SR_PAR_FRAME_BMSK	BIT(5)
#define UARTDM_SR_OVERRUN_BMSK		BIT(4)
#define UARTDM_SR_TXEMT_BMSK		BIT(3)
#define UARTDM_SR_TXRDY_BMSK		BIT(2)
#define UARTDM_SR_RXRDY_BMSK		BIT(0)

#define UARTDM_CR_TX_DISABLE_BMSK	BIT(3)
#define UARTDM_CR_RX_DISABLE_BMSK	BIT(1)
#define UARTDM_CR_TX_EN_BMSK		BIT(2)
#define UARTDM_CR_RX_EN_BMSK		BIT(0)

/* UARTDM_CR channel_comman bit value (register field is bits 8:4) */
#define RESET_RX		0x10
#define RESET_TX		0x20
#define RESET_ERROR_STATUS	0x30
#define RESET_BREAK_INT		0x40
#define START_BREAK		0x50
#define STOP_BREAK		0x60
#define RESET_CTS		0x70
#define RESET_STALE_INT		0x80
#define RFR_LOW			0xD0
#define RFR_HIGH		0xE0
#define CR_PROTECTION_EN	0x100
#define STALE_EVENT_ENABLE	0x500
#define STALE_EVENT_DISABLE	0x600
#define FORCE_STALE_EVENT	0x400
#define CLEAR_TX_READY		0x300
#define RESET_TX_ERROR		0x800
#define RESET_TX_DONE		0x810

#define UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK 0xffffff00
#define UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK 0x3f
#define UARTDM_MR1_CTS_CTL_BMSK 0x40
#define UARTDM_MR1_RX_RDY_CTL_BMSK 0x80

#define UARTDM_MR2_ERROR_MODE_BMSK 0x40
#define UARTDM_MR2_BITS_PER_CHAR_BMSK 0x30

/* bits per character configuration */
#define FIVE_BPC  (0 << 4)
#define SIX_BPC   (1 << 4)
#define SEVEN_BPC (2 << 4)
#define EIGHT_BPC (3 << 4)

#define UARTDM_MR2_STOP_BIT_LEN_BMSK 0xc
#define STOP_BIT_ONE (1 << 2)
#define STOP_BIT_TWO (3 << 2)

#define UARTDM_MR2_PARITY_MODE_BMSK 0x3

/* Parity configuration */
#define NO_PARITY 0x0
#define EVEN_PARITY 0x1
#define ODD_PARITY 0x2
#define SPACE_PARITY 0x3

#define UARTDM_IPR_STALE_TIMEOUT_MSB_BMSK 0xffffff80
#define UARTDM_IPR_STALE_LSB_BMSK 0x1f

/* These can be used for both ISR and IMR register */
#define UARTDM_ISR_TX_READY_BMSK	BIT(7)
#define UARTDM_ISR_CURRENT_CTS_BMSK	BIT(6)
#define UARTDM_ISR_DELTA_CTS_BMSK	BIT(5)
#define UARTDM_ISR_RXLEV_BMSK		BIT(4)
#define UARTDM_ISR_RXSTALE_BMSK		BIT(3)
#define UARTDM_ISR_RXBREAK_BMSK		BIT(2)
#define UARTDM_ISR_RXHUNT_BMSK		BIT(1)
#define UARTDM_ISR_TXLEV_BMSK		BIT(0)

/* Field definitions for UART_DM_DMEN*/
#define UARTDM_TX_DM_EN_BMSK 0x1
#define UARTDM_RX_DM_EN_BMSK 0x2

#endif /* MSM_SERIAL_HS_HWREG_H */

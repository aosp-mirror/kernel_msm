/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#ifndef _BSL_HOST_H
#define _BSL_HOST_H

#include <linux/types.h>

/* Defines for BSL Commands */
#define MSP430_I2C_MASSERASE    0x15
#define MSP430_I2C_READDATA     0x18
#define MSP430_I2C_SEGERASE     0x12
#define MSP430_I2C_BSLPASSWORD  0x11
#define MSP430_I2C_UNLOCKINFO   0x13
#define MSP430_I2C_SENDDATA     0x10
#define MSP430_I2C_SETPC        0x17
#define MSP430_I2C_CHECKCRC     0x16
#define MSP430_I2C_HEADER       0x80
#define MSP430_I2C_RESET        0x1C
#define MSP430_I2C_SUCCESS      0x01
#define MSP430_I2C_MESSAGE_RESP 0x3B
#define MSP430_I2C_CMD_RESP     0x3A
#define MSP430_I2C_ATTEMPTS     5
#define BUFFER_SIZE             1024
#define MSP430_MAX_CHUNK_SIZE   250

/* Largest Address on the bus */
#define MAX_I2C_ADDR            0xFE

/* Defines for response codes */
#define MSP430_STATUS_OPERATION_OK               0x00
#define MSP430_STATUS_PASSWORD_ERROR             0x05
#define MSP430_STATUS_TXTFILE_ERROR             0x101
#define MSP430_STATUS_RESET_ERROR               0x102
#define MSP430_STATUS_I2C_NOT_FOUND             0x103
#define MSP430_STATUS_I2C_TRANSACTION_ERROR     0x104
#define MSP430_STATUS_INVALID_RESP_HEADER       0x105
#define MSP430_STATUS_INVALID_RESP_LENGTH       0x106
#define MSP430_STATUS_INVALID_RESP_CRC          0x107
#define MSP430_STATUS_INVOKE_FAIL               0x108
#define MSP430_STATUS_VERIFICATION_FAIL         0x109

#define CRC16_POLY                  0x1021
#define uintn_t                     UINTN

#define MSP430_I2C_SEGMENT_SIZE     512
#define MSP430_RESET_VECTOR_ADDR    0xFFFE

/* Host Packet Structures */
typedef struct sI2CPayload
{
	uint8_t ui8Command;
	uint8_t ui8Addr_L;
	uint8_t ui8Addr_M;
	uint8_t ui8Addr_H;
	uint8_t* ui8pData;
} tI2CPayload;

typedef struct sI2CBSLPacket
{
	uint8_t ui8Header;
	uint16_t ui16Length;
	tI2CPayload tPayload;
	uint16_t ui16Checksum;
} tI2CBSLPacket;

/* Public Function ProtoTypes */
extern uint32_t MSP430BSL_sendData(uint8_t* data, uint32_t addr, uint32_t size);
extern uint32_t MSP430BSL_readData(uint8_t* inBuffer, uint32_t addr,
					uint32_t size);
extern uint32_t MSP430BSL_unlockDevice(uint8_t* password);
extern uint32_t MSP430BSL_massErase(void);
extern uint16_t MSP430BSL_checkCRC(uint32_t addr, uint32_t length,
					uint16_t* crcCalculate);
extern uint32_t MSP430BSL_invokeBSL(uint8_t* data, uint32_t length);
extern uint32_t MSP430BSL_setProgramCounter(uint16_t addr);

extern uint16_t calculateCRC16(uint8_t* data, uint32_t length);

#endif /* BSLHOST */

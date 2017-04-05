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

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/syscalls.h>


#include "msp430_i2cbsl.h"

#define malloc(a) kmalloc(a, GFP_KERNEL)
#define free(a) kfree(a)

/* Statics */
static uint32_t checkBSLResponse(void);
static bool MSP430BSL_sendI2CPacket(tI2CBSLPacket tPacket,
				uint32_t recBytes);
static bool MSP430BSL_I2CWrite(uint8_t* buffer, uint32_t numberOfBytes);
static bool MSP430BSL_I2CWriteRead (uint8_t* writeBuffer,
	uint32_t numOfWriteBytes, uint8_t* readBuffer, uint32_t numOfReadBytes);
static uint8_t xferBuffer[BUFFER_SIZE];
static uint8_t recBuffer[BUFFER_SIZE];
static uint32_t checkBSLReadResponse(uint32_t length);

/* Global I2C Handle that is opened by the main application */
extern int i2c_file;
extern uint16_t msp430slaveAddress;

uint32_t MSP430BSL_sendData(uint8_t* data, uint32_t addr, uint32_t size)
{
	tI2CBSLPacket packet;

	/* MSP430 allows for a max of a 20-bit address */
	packet.tPayload.ui8Addr_L = (uint8_t)(addr & 0x000FF);
	packet.tPayload.ui8Addr_M = (uint8_t)((addr & 0x0FF00) >> 8);
	packet.tPayload.ui8Addr_H = (uint8_t)((addr & 0xF0000) >> 16);

	packet.ui8Header = MSP430_I2C_HEADER;
	packet.ui16Length = (uint16_t)(size + 4);
	packet.tPayload.ui8Command = MSP430_I2C_SENDDATA;
	packet.tPayload.ui8pData = malloc(sizeof(uint8_t)*size);
	memcpy(packet.tPayload.ui8pData, data, size);

	/* Did the transaction finish correctly? */
	if (!MSP430BSL_sendI2CPacket(packet, 1)) {
		free(packet.tPayload.ui8pData);
		return MSP430_STATUS_I2C_TRANSACTION_ERROR;
	}

	free(packet.tPayload.ui8pData);
	return checkBSLResponse();
}

uint32_t MSP430BSL_readData(uint8_t* inBuffer, uint32_t addr, uint32_t size)
{
	tI2CBSLPacket packet;
	uint32_t res;

	/* Setting up the Read Packet */
	packet.ui8Header = MSP430_I2C_HEADER;
	packet.tPayload.ui8Command = MSP430_I2C_READDATA;
	packet.tPayload.ui8pData = malloc(sizeof(uint8_t)*2);
	packet.tPayload.ui8Addr_L = (uint8_t)(addr & 0x000FF);
	packet.tPayload.ui8Addr_M = (uint8_t)((addr & 0x0FF00) >> 8);
	packet.tPayload.ui8Addr_H = (uint8_t)((addr & 0xF0000) >> 16);
	packet.tPayload.ui8pData[0] = (uint8_t)(size & 0x000FF);
	packet.tPayload.ui8pData[1] = (uint8_t)((size & 0x0FF00) >> 8);
	packet.ui16Length = 6;

	if (!MSP430BSL_sendI2CPacket(packet, size)) {
		free(packet.tPayload.ui8pData);
		return MSP430_STATUS_I2C_TRANSACTION_ERROR;
	}

	free(packet.tPayload.ui8pData);
	res = checkBSLReadResponse(size);

	if (res != MSP430_STATUS_OPERATION_OK) {
		return res;
	} else {
		memcpy(inBuffer, recBuffer+5,size);
		return res;
	}
}

uint32_t MSP430BSL_unlockDevice(uint8_t* password)
{
	tI2CBSLPacket packet;

	/* (Signals to skip Address ) */
	packet.tPayload.ui8Addr_L = 0;
	packet.tPayload.ui8Addr_M = 0;
	packet.tPayload.ui8Addr_H = 0xFF;

	packet.ui8Header = MSP430_I2C_HEADER;
	packet.ui16Length = 33;
	packet.tPayload.ui8Command = MSP430_I2C_BSLPASSWORD;
	packet.tPayload.ui8pData = malloc(sizeof(uint8_t)*32);
	memcpy(packet.tPayload.ui8pData, password, 32);

	/* Did the transaction finish correctly? */
	if (!MSP430BSL_sendI2CPacket(packet, 1)) {
		free(packet.tPayload.ui8pData);
		return MSP430_STATUS_I2C_TRANSACTION_ERROR;
	}

	free(packet.tPayload.ui8pData);
	return checkBSLResponse();
}

uint32_t MSP430BSL_massErase(void)
{
	uint16_t crc16;

	xferBuffer[0] = MSP430_I2C_HEADER;
	xferBuffer[1] = 1;
	xferBuffer[2] = 0;
	xferBuffer[3] = MSP430_I2C_MASSERASE;

	/* Calculating/Setting the CRC */
	crc16 = calculateCRC16(xferBuffer + 3, 1);
	xferBuffer[4] = crc16 & 0xFF;
	xferBuffer[5] = ((crc16 >> 8) & 0xFF);

	if (!MSP430BSL_I2CWrite(xferBuffer,6))
		return MSP430_STATUS_INVOKE_FAIL;
	return MSP430_STATUS_OPERATION_OK;
}

uint32_t MSP430BSL_setProgramCounter(uint16_t addr)
{
	uint16_t crc16;

	xferBuffer[0] = MSP430_I2C_HEADER;
	xferBuffer[1] = 4;
	xferBuffer[2] = 0;
	xferBuffer[3] = MSP430_I2C_SETPC;

	/* Setting the address */
	xferBuffer[4] = (uint8_t)(addr & 0x000FF);
	xferBuffer[5] = (uint8_t)((addr & 0x0FF00) >> 8);
	xferBuffer[6] = 0;

	/* Calculating/Setting the CRC */
	crc16 = calculateCRC16(xferBuffer + 3, 4);
	xferBuffer[7] = crc16 & 0xFF;
	xferBuffer[8] = ((crc16 >> 8) & 0xFF);

	if (!MSP430BSL_I2CWrite(xferBuffer,9))
		return MSP430_STATUS_INVOKE_FAIL;
	return MSP430_STATUS_OPERATION_OK;
}

uint16_t MSP430BSL_checkCRC(uint32_t addr, uint32_t length,
				uint16_t* crcCalculate)
{
	tI2CBSLPacket packet;

	packet.ui8Header = MSP430_I2C_HEADER;
	packet.tPayload.ui8Command = MSP430_I2C_CHECKCRC;
	packet.ui16Length = 6;
	packet.tPayload.ui8pData = malloc(sizeof(uint8_t)*2);
	packet.tPayload.ui8Addr_L = (uint8_t)(addr & 0x000FF);
	packet.tPayload.ui8Addr_M = (uint8_t)((addr & 0x0FF00) >> 8);
	packet.tPayload.ui8Addr_H = (uint8_t)((addr & 0xF0000) >> 16);
	packet.tPayload.ui8pData[0] = (uint8_t)(length & 0x000FF);
	packet.tPayload.ui8pData[1] = (uint8_t)((length & 0x0FF00) >> 8);

	/* Did the transaction finish correctly? */
	if (!MSP430BSL_sendI2CPacket(packet, 3)) {
		return MSP430_STATUS_I2C_TRANSACTION_ERROR;
	}

	free(packet.tPayload.ui8pData);
	checkBSLReadResponse(2);

	*crcCalculate = recBuffer[6];
	*crcCalculate = ((*crcCalculate) << 8) | recBuffer[5];

	return MSP430_STATUS_OPERATION_OK;
}

static uint32_t checkBSLReadResponse(uint32_t length)
{
	uint16_t recLength, recCrc;

	/* Is the header correct? */
	if (recBuffer[0] != 0) {
		return MSP430_STATUS_INVALID_RESP_HEADER;
	}

	/* Is the header correct? */
	if (recBuffer[1] != MSP430_I2C_HEADER) {
		return MSP430_STATUS_INVALID_RESP_HEADER;
	}

	/* Do we have the right length? */
	recLength = recBuffer[3];
	recLength = (recLength << 8) | recBuffer[2];

	if (recLength != (length + 1)) {
		return MSP430_STATUS_INVALID_RESP_LENGTH;
	}

	/* Does the CRC match? */
	recCrc = recBuffer[recLength + 5];
	recCrc = (recCrc << 8) | recBuffer[recLength + 4];

	if (recCrc != calculateCRC16(recBuffer + 4, recLength)) {
		return MSP430_STATUS_INVALID_RESP_CRC;
	}

	/* Is the response actually good? */
	if (recBuffer[4] != MSP430_I2C_CMD_RESP) {
		return recBuffer[4];
	}

	return MSP430_STATUS_OPERATION_OK;
}

static uint32_t checkBSLResponse()
{
	uint16_t recLength, recCrc;

	/* Is the header correct? */
	if (recBuffer[0] != 0) {
		return MSP430_STATUS_INVALID_RESP_HEADER;
	}

	/* Is the header correct? */
	if (recBuffer[1] != MSP430_I2C_HEADER) {
		return MSP430_STATUS_INVALID_RESP_HEADER;
	}

	/* Do we have the right length? */
	recLength = recBuffer[3];
	recLength = (recLength << 8) | recBuffer[2];

	if (recLength != 2) {
		return MSP430_STATUS_INVALID_RESP_LENGTH;
	}

	/* Does the CRC match? */
	recCrc = recBuffer[recLength + 5];
	recCrc = (recCrc << 8) | recBuffer[recLength + 4];

	if (recCrc != calculateCRC16(recBuffer + 4, recLength)) {
		return MSP430_STATUS_INVALID_RESP_CRC;
	}

	/* Is the response actually good? */
	if (recBuffer[4] != MSP430_I2C_MESSAGE_RESP ||
		recBuffer[5] != MSP430_STATUS_OPERATION_OK) {
		return recBuffer[5];
		pr_info("INFO: Is the response actually good?\n");
		pr_info("recBuffer[4] = %d, recBuffer[5] = %d\n",
				recBuffer[4], recBuffer[5]);
	}

	return MSP430_STATUS_OPERATION_OK;
}

uint32_t MSP430BSL_invokeBSL(uint8_t* data, uint32_t length)
{
	if(!MSP430BSL_I2CWrite(data,length))
		return MSP430_STATUS_INVOKE_FAIL;
	return MSP430_STATUS_OPERATION_OK;
}

static bool MSP430BSL_sendI2CPacket(tI2CBSLPacket tPacket,
				uint32_t recBytes)
{
	uint16_t crc16, offset;

	offset = 0;

	/* Constructing the buffer */
	xferBuffer[0] = tPacket.ui8Header;
	xferBuffer[1] = tPacket.ui16Length & 0xFF;
	xferBuffer[2] = (tPacket.ui16Length >> 8) & 0xFF;
	xferBuffer[3] = tPacket.tPayload.ui8Command;

	/* Checking to see if there is a valid address */
	if (tPacket.tPayload.ui8Addr_H != 0xFF) {
		xferBuffer[4] = tPacket.tPayload.ui8Addr_L;
		xferBuffer[5] = tPacket.tPayload.ui8Addr_M;
		xferBuffer[6] = tPacket.tPayload.ui8Addr_H;
		offset = 3;
	}

	if ((tPacket.ui16Length - offset - 1) > 0)
		memcpy(xferBuffer + 4 + offset, tPacket.tPayload.ui8pData,
				tPacket.ui16Length - offset - 1);

	/* Calculating/Setting the CRC */
	crc16 = calculateCRC16(xferBuffer + 3, tPacket.ui16Length);
	xferBuffer[tPacket.ui16Length + 3] = crc16 & 0xFF;
	xferBuffer[tPacket.ui16Length + 4] = ((crc16 >> 8) & 0xFF);

	if (!MSP430BSL_I2CWriteRead(xferBuffer, tPacket.ui16Length + 5,
				recBuffer, recBytes + 7))
		return false;

	return true;
}

static bool MSP430BSL_I2CWriteRead (uint8_t* writeBuffer,
	uint32_t numOfWriteBytes, uint8_t* readBuffer, uint32_t numOfReadBytes)
{
	struct i2c_msg messages[2];
	struct i2c_rdwr_ioctl_data packets;

	/* Setting up the register write */
	messages[0].addr = msp430slaveAddress;
	messages[0].flags = 0;
	messages[0].len = numOfWriteBytes;
	messages[0].buf = writeBuffer;

	/* Setting up the read */
	messages[1].addr = msp430slaveAddress;
	messages[1].flags = I2C_M_RD;
	messages[1].len = numOfReadBytes;
	messages[1].buf = readBuffer;

	/* Sending the request */
	packets.msgs = messages;
	packets.nmsgs = 2;

	if (sys_ioctl(i2c_file, I2C_RDWR, (long)&packets) < 0) {
		pr_err("MSP430BSL_I2CWriteRead ERROR!\n");
		return false;
	}

	return true;
}

static bool MSP430BSL_I2CWrite(uint8_t* buffer, uint32_t numberOfBytes)
{
	struct i2c_msg messages[1];
	struct i2c_rdwr_ioctl_data packets;

	/* Setting up the register write */
	messages[0].addr = msp430slaveAddress;
	messages[0].flags = 0;
	messages[0].len = numberOfBytes;
	messages[0].buf = buffer;

	/* Sending the request */
	packets.msgs = messages;
	packets.nmsgs = 1;

	if (sys_ioctl(i2c_file, I2C_RDWR, (long)&packets) < 0) {
		pr_err("MSP430BSL_I2CWrite ERROR!\n");
		return false;
	}

	return true;

}

uint16_t calculateCRC16(uint8_t* data, uint32_t length)
{
	uint32_t i;
	uint16_t j;
	uint16_t msg;
	uint16_t crc = 0xFFFF;
	uint8_t* pmsg = data;

	uint32_t msg_size = length;

	for (i = 0 ; i < msg_size ; i ++) {
		msg = (pmsg[i] << 8);
		for (j = 0 ; j < 8 ; j++) {
			if ((msg ^ crc) >> 15)
				crc = (crc << 1) ^ CRC16_POLY;
			else
				crc <<= 1;
			msg <<= 1;
		}
	}

	return(crc);
}


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

#include "msp430_i2cbsl.h"
#include "msp430_firmware_parser.h"
#include "msp430_firmware.h"

#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm-generic/ioctl.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/firmware.h>

#define MSP430_I2C_BUS_PATH "/dev/i2c-6"
uint8_t msp430slaveAddress = 0x48;

int i2c_file;

uint8_t bslPassword[32] =
{
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

uint8_t invokeString[8] =
	{0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE};

/* Firmware File Externals */
extern const unsigned long eprom_sections;
extern const unsigned long eprom_address[];
extern const unsigned long eprom_length_of_sections[];

int msp430_firmware_update_start(struct device *dev)
{
	tMSPMemorySegment *firmwareImage;
	tMSPMemorySegment *tTXTFile;
	uint32_t ii, uu, res;
	uint8_t resetVector[2];
	uint32_t resetVectorValue;
	int fw_version_skip = 3;
	const struct firmware *fw_entry = NULL;

	pr_info("\n----- Simplified MSP430 I2C Linux BSL ----- \n");

	res = request_firmware(&fw_entry, "msp430.fw", dev);
	if (res != 0) {
		pr_err("ERROR: Firmware image not available\n");
		return res;
	}

	/* Assuming failure */
	res = 1;

	/* Opening the I2C interface */
	if ((i2c_file = sys_open(MSP430_I2C_BUS_PATH, O_RDWR, 0)) < 0) {
		pr_err("ERROR: Could not sys_open I2C Driver\n");
		return res;
	}

	/* Parsing the current firmware file */
	pr_info("INFO: Parsing firmware image from C-Array.\n");
	firmwareImage = MSP430BSL_parseSRecordArray(eprom_sections,
						eprom_address,
						eprom_length_of_sections,
						fw_entry->data);

	if (firmwareImage == NULL) {
		pr_err("ERROR: Could not read firmware image!\n");
		return res;
	}

	/* Invoking the BSL */
	pr_info("INFO: Invoking the BSL.\n");
	if (MSP430BSL_invokeBSL(invokeString, 8) != MSP430_STATUS_OPERATION_OK ) {
		pr_err("ERROR: Could not invoke BSL!\n");
		goto BSLCleanUp;
	}

	/* Sleeping after the invoke */
	msleep(100);

	/* Issuing a mass reset  */
	pr_info("INFO: Issuing a mass reset\n");
	if (MSP430BSL_massErase() != MSP430_STATUS_OPERATION_OK ) {
		pr_err("ERROR: Could not issue mass erase!\n");
		goto BSLCleanUp;
	}

	/* Sleeping after the mass reset */
	msleep(100);

	/* Unlocking the device */
	pr_info("INFO: Unlocking the device \n");
	if (MSP430BSL_unlockDevice(bslPassword) != MSP430_STATUS_OPERATION_OK ) {
		pr_err("ERROR: Could not unlock device!\n");
		goto BSLCleanUp;
	}

	/* Programming all memory segments */
	for (uu = 0; uu < 5; uu++) {
		pr_info("INFO: Programming attempt number %d\n", uu);

		tTXTFile = firmwareImage;
		while (tTXTFile != NULL) {
			pr_info("INFO: Programming @0x%x with %d bytes of data... ",
				tTXTFile->ui32MemoryStartAddr,
				tTXTFile->ui32MemoryLength);

			/* Programming the memory segment */
			ii = 0;
			while (ii < tTXTFile->ui32MemoryLength) {
				if ((tTXTFile->ui32MemoryLength - ii) > 128) {
					res = MSP430BSL_sendData(tTXTFile->ui8Buffer + ii,
							tTXTFile->ui32MemoryStartAddr + ii, 128);
					ii += 128;
				} else {
					res = MSP430BSL_sendData(tTXTFile->ui8Buffer + ii,
							tTXTFile->ui32MemoryStartAddr + ii,
							(tTXTFile->ui32MemoryLength - ii));
					ii = tTXTFile->ui32MemoryLength;
				}

				if (fw_version_skip > 0) {
					res = MSP430_STATUS_OPERATION_OK;
					fw_version_skip--;
				}

				if (res != MSP430_STATUS_OPERATION_OK) {
					pr_err("ERROR: Programming address 0x%x (Code 0x%x).\n",
						tTXTFile->ui32MemoryStartAddr + ii, res);
					break;
				}
			}

			if (res != MSP430_STATUS_OPERATION_OK)
				break;

			pr_info("done!\n");

			tTXTFile = tTXTFile->pNextSegment;
		}

		if (res == MSP430_STATUS_OPERATION_OK) {
			pr_info("INFO: Programmed all memory locations successfully.\n");
			break;
		}
	}

	/* Resetting the device */
	pr_info("INFO: Resetting the device.\n");
	res = MSP430BSL_readData(resetVector, MSP430_RESET_VECTOR_ADDR, 2);

	if (res != MSP430_STATUS_OPERATION_OK) {
		pr_err("ERROR: Could not read reset vector address!\n");
		goto BSLCleanUp;
	}

	resetVectorValue = (resetVector[1] << 8) | resetVector[0];
	pr_info("INFO: Reset vector read as 0x%x\n", resetVectorValue);
	res = MSP430BSL_setProgramCounter(resetVectorValue);

	if (res != MSP430_STATUS_OPERATION_OK) {
		pr_err("ERROR: Could not set program counter!\n");
		goto BSLCleanUp;
	}

	pr_info("INFO: Firmware updated without issue.\n");

BSLCleanUp:
	MSP430BSL_cleanUpPointer(firmwareImage);
	return res;
}

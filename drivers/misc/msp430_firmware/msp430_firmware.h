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

#ifndef __MSP430_FIRMWARE_H
#define __MSP430_FIRMWARE_H

const unsigned long eprom_address[] =
{
	0x00001880, 0x00001940, 0x00001948, 0x00004400, 0x0000EB8E, 0x0000FFD2,
	0x0000FFE8,
};
const unsigned long eprom_length_of_sections[] =
{
	0x00000002, 0x00000002, 0x00000002, 0x0000A78D, 0x000000C6, 0x00000014,
	0x00000018,
};

const unsigned long eprom_sections    = 0x00000007;
const unsigned long eprom_termination = 0x00000000;
const unsigned long eprom_start       = 0x00001880;
const unsigned long eprom_finish      = 0x00010000;
const unsigned long eprom_length      = 0x0000E780;

#define EPROM_TERMINATION 0x00000000
#define EPROM_START       0x00001880
#define EPROM_FINISH      0x00010000
#define EPROM_LENGTH      0x0000E780
#define EPROM_SECTIONS    0x00000007

#endif

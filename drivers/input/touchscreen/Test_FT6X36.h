/* 
* Copyright © 2016 FocalTech Systems Co., Ltd.  All Rights Reserved.
* 
* This program is free software; you may redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by 
* the Free Software Foundation; version 2 of the License. 
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS 
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN 
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
* SOFTWARE. 
*/
#ifndef _TEST_FT6X36_H
#define _TEST_FT6X36_H

#include "test_lib.h"

boolean FT6X36_StartTest(void);
boolean FT3207_StartTest(bool* selftest_result);
int FT6X36_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
int FT3207_get_test_data(char **pTestData);

unsigned char FT6X36_TestItem_EnterFactoryMode(void);
unsigned char FT6X36_TestItem_RawDataTest(bool * bTestResult);
//unsigned char FT6X36_TestItem_ChannelsTest(bool * bTestResult);
unsigned char FT6X36_TestItem_CbTest(bool * bTestResult);
unsigned char FT6X36_TestItem_DeltaCbTest(unsigned char * bTestResult);
unsigned char FT6X36_TestItem_ChannelsDeviationTest(unsigned char * bTestResult);
unsigned char FT6X36_TestItem_TwoSidesDeviationTest(unsigned char * bTestResult);

boolean GetWaterproofMode(int iTestType, unsigned char ucChannelValue);

#define DATA_SIZE 1024*1
#define PROCESS_LOG_SIZE 1024*2
#define ALLMSG_SIZE 1024*4

#endif

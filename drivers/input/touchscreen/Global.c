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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/printk.h>

#include <linux/delay.h>

#include "ini.h"
#include "Global.h"

#include "test_lib.h"

#define DEVIDE_MODE_ADDR	0x00

struct StruScreenSeting g_ScreenSetParam;
struct stTestItem g_stTestItem[1][MAX_TEST_ITEM];

int g_TestItemNum = 0;
char g_strIcName[20] = {0};

int GetPrivateProfileString(char *section, char *ItemName, char *defaultvalue, char *returnValue, char *IniFile)
{
	char value[512] = {0};
	int len = 0;

	if (NULL == returnValue) {
		pr_info("[FTS] returnValue==NULL in function %s.", __func__);
		return 0;
	}

	if (ini_get_key(IniFile, section, ItemName, value) < 0)	{
		if (NULL != defaultvalue)
			memcpy(value, defaultvalue, strlen(defaultvalue));

		snprintf(returnValue, 128, "%s", value);
		return 0;
	} else {
		len = snprintf(returnValue, 128, "%s", value);
	}

	return len;
}

void focal_msleep(int ms)
{
	msleep(ms);
}
void SysDelay(int ms)
{
	msleep(ms);
}
int focal_abs(int value)
{
	if (value < 0)
		value = 0 - value;

	return value;
}

unsigned char get_ic_code(char *strIcName)
{
	if (strncmp(strIcName, "FT3X07", 6) == 0)
		return IC_FT3X07;

	return 0xff;
}

void get_ic_name(unsigned char ucIcCode, char *strIcName)
{
	if (NULL == strIcName)
		return;

	snprintf(strIcName, 128, "%s", "NA");

	if (ucIcCode == IC_FT3X07)
		snprintf(strIcName, 128, "%s",  "FT3X07");
}
void OnInit_InterfaceCfg(char *strIniFile)
{
	char str[128];

	GetPrivateProfileString("Interface", "IC_Type", "FT5X36", str, strIniFile);
	g_ScreenSetParam.iSelectedIC = get_ic_code(str);

	GetPrivateProfileString("Interface", "Normalize_Type", 0, str, strIniFile);
	g_ScreenSetParam.isNormalize = atoi(str);
}

int ReadReg(unsigned char RegAddr, unsigned char *RegData)
{
	int iRet;

	if (NULL == fts_i2c_read_test) {
		pr_info("[focal] %s fts_i2c_read == NULL\n", __func__);
		return ERROR_CODE_INVALID_COMMAND;
	}

	iRet = fts_i2c_read_test(&RegAddr, 1, RegData, 1);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

int WriteReg(unsigned char RegAddr, unsigned char RegData)
{
	int iRet;
	unsigned char cmd[2] = {0};

	if (NULL == fts_i2c_write_test) {
		pr_info("[focal] %s fts_i2c_write == NULL\n", __func__);
		return ERROR_CODE_INVALID_COMMAND;
		}

	cmd[0] = RegAddr;
	cmd[1] = RegData;
	iRet = fts_i2c_write_test(cmd, 2);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

unsigned char Comm_Base_IIC_IO(unsigned char *pWriteBuffer, int iBytesToWrite, unsigned char *pReadBuffer, int iBytesToRead)
{
	int iRet;

	if (NULL == fts_i2c_read_test) {
		pr_info("[FTS] %s fts_i2c_read == NULL\n", __func__);
		return ERROR_CODE_INVALID_COMMAND;
		}

	iRet = fts_i2c_read_test(pWriteBuffer, iBytesToWrite, pReadBuffer, iBytesToRead);

	if (iRet >= 0)
		return ERROR_CODE_OK;
	else
		return ERROR_CODE_COMM_ERROR;
}

unsigned char EnterWork(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if (ReCode == ERROR_CODE_OK) {
		if (((RunState >> 4) & 0x07) == 0x00) {
			ReCode = ERROR_CODE_OK;
		} else {
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0);
			if (ReCode == ERROR_CODE_OK) {
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if (ReCode == ERROR_CODE_OK) {
					if (((RunState >> 4) & 0x07) == 0x00)
						ReCode = ERROR_CODE_OK;
					else
						ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	return ReCode;
}

unsigned char EnterFactory(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if (ReCode == ERROR_CODE_OK) {
		if (((RunState>>4)&0x07) == 0x04) {
			ReCode = ERROR_CODE_OK;
		} else {
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0x40);
			if (ReCode == ERROR_CODE_OK) {
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if (ReCode == ERROR_CODE_OK) {
					if (((RunState >> 4) & 0x07) == 0x04)
						ReCode = ERROR_CODE_OK;
					else
						ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	return ReCode;
}


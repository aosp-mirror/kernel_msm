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
#include <linux/bug.h>
#include <linux/delay.h>

#include "ini.h"
#include "Global.h"
#include "test_lib.h"

#define DEVIDE_MODE_ADDR	0x00

struct StruScreenSeting g_ScreenSetParam; //«Ì¹õ³]¸m°Ñ¼Æ
struct stTestItem g_stTestItem[1][MAX_TEST_ITEM];
//struct structSCapConf g_stSCapConf;
int g_TestItemNum = 0;
char g_strIcName[STR_NAME] = {0};

int GetPrivateProfileString(char *section, char *ItemName, char *defaultvalue, char *returnValue, char *IniFile){
	char value[512] = {0};
	int len = 0;

	if(NULL == returnValue)
	{
		printk("[fts] returnValue == NULL in function %s. \n", __func__);
		return 0;
	}

	if(ini_get_key(IniFile, section, ItemName, value) < 0)
	{
		if(NULL != defaultvalue) memcpy(value, defaultvalue, strlen(defaultvalue));
		sprintf(returnValue, "%s", value);
		return 0;
	} else {
		len = sprintf(returnValue, "%s", value);
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
	if(value < 0)
		value = 0 - value;

	return value;
}
/////////////////////////////////////////////////////////////////////////////
////Àò¨úIC¹ïÀ³½X
/////////////////////////////////////////////////////////////////////////////
unsigned char get_ic_code(char *strIcName)
{
	if (strncmp(strIcName, "FT5X36", strlen("FT5X36")) == 0)
		return IC_FT5X36;
	if (strncmp(strIcName, "FT5X36i", strlen("FT5X36i")) == 0)
		return IC_FT5X36i;
	if (strncmp(strIcName, "FT3X16", strlen("FT3X16")) == 0)
		return IC_FT3X16;
	if (strncmp(strIcName, "FT3X26", strlen("FT3X26")) == 0)
		return IC_FT3X26;
	if (strncmp(strIcName, "FT3267", strlen("FT3267")) == 0)
		return IC_FT3267;

	if (strncmp(strIcName, "FT5X46", strlen("FT5X46")) == 0)
		return IC_FT5X46;
	if (strncmp(strIcName, "FT5X46i", strlen("FT5X46i")) == 0)
		return IC_FT5X46i;
	if (strncmp(strIcName, "FT5526", strlen("FT5526")) == 0)
		return IC_FT5526;
	if (strncmp(strIcName, "FT3X17", strlen("FT3X17")) == 0)
		return IC_FT3X17;
	if (strncmp(strIcName, "FT5436", strlen("FT5436")) == 0)
		return IC_FT5436;
	if (strncmp(strIcName, "FT3X27", strlen("FT3X27")) == 0)
		return IC_FT3X27;
	if (strncmp(strIcName, "FT5526i", strlen("FT5526i")) == 0)
		return IC_FT5526I;
	if (strncmp(strIcName, "FT5416", strlen("FT5416")) == 0)
		return IC_FT5416;
	if (strncmp(strIcName, "FT5426", strlen("FT5426")) == 0)
		return IC_FT5426;
	if (strncmp(strIcName, "FT5435", strlen("FT5435")) == 0)
		return IC_FT5435;

	if (strncmp(strIcName, "FT6X06", strlen("FT6X06")) == 0)
		return IC_FT6X06;
	if (strncmp(strIcName, "FT3X06", strlen("FT3X06")) == 0)
		return IC_FT3X06;

	if (strncmp(strIcName, "FT6X36", strlen("FT6X36")) == 0)
		return IC_FT6X36;
	if (strncmp(strIcName, "FT3X07", strlen("FT3X07")) == 0)
		return IC_FT3X07;
	if (strncmp(strIcName, "FT6416", strlen("FT6416")) == 0)
		return IC_FT6416;
	if (strncmp(strIcName, "FT6336G/U", strlen("FT6336G/U")) == 0)
		return IC_FT6426;

	if (strncmp(strIcName, "FT5X16", strlen("FT5X16")) == 0)
		return IC_FT5X16;
	if (strncmp(strIcName, "FT5X12", strlen("FT5X12")) == 0)
		return IC_FT5X12;

	if (strncmp(strIcName, "FT5506", strlen("FT5506")) == 0)
		return IC_FT5506;
	if (strncmp(strIcName, "FT5606", strlen("FT5606")) == 0)
		return IC_FT5606;
	if (strncmp(strIcName, "FT5816", strlen("FT5816")) == 0)
		return IC_FT5816;

	if (strncmp(strIcName, "FT5822", strlen("FT5822")) == 0)
		return IC_FT5822;
	if (strncmp(strIcName, "FT5626", strlen("FT5626")) == 0)
		return IC_FT5626;
	if (strncmp(strIcName, "FT5726", strlen("FT5726")) == 0)
		return IC_FT5726;
	if (strncmp(strIcName, "FT5826B", strlen("FT5826B")) == 0)
		return IC_FT5826B;
	if (strncmp(strIcName, "FT5826S", strlen("FT5826S")) == 0)
		return IC_FT5826S;

	if (strncmp(strIcName, "FT5306", strlen("FT5306")) == 0)
		return IC_FT5306;
	if (strncmp(strIcName, "FT5406", strlen("FT5406")) == 0)
		return IC_FT5406;

	if (strncmp(strIcName, "FT8606", strlen("FT8606")) == 0)
		return IC_FT8606;

	return 0xff;
}
/////////////////////////////////////////////////////////////////////////////
////Àò¨úIC¦W
/////////////////////////////////////////////////////////////////////////////
void get_ic_name(unsigned char ucIcCode, char *strIcName)
{
	if (NULL == strIcName)
		return;

	/*if can't find IC , set 'NA' */
	snprintf(strIcName, STR_NAME, "%s", "NA");

	if (ucIcCode == IC_FT5X36)
		snprintf(strIcName, STR_NAME, "%s", "FT5X36");
	if (ucIcCode == IC_FT5X36i)
		snprintf(strIcName, STR_NAME, "%s", "FT5X36i");
	if (ucIcCode == IC_FT3X16)
		snprintf(strIcName, STR_NAME, "%s", "FT3X16");
	if (ucIcCode == IC_FT3X26)
		snprintf(strIcName, STR_NAME, "%s", "FT3X26");

	if (ucIcCode == IC_FT5X46)
		snprintf(strIcName, STR_NAME, "%s", "FT5X22");
	if (ucIcCode == IC_FT5X46)
		snprintf(strIcName, STR_NAME, "%s", "FT5X46");
	if (ucIcCode == IC_FT5X46i)
		snprintf(strIcName, STR_NAME, "%s", "FT5X46i");
	if (ucIcCode == IC_FT5526)
		snprintf(strIcName, STR_NAME, "%s", "FT5526");
	if (ucIcCode == IC_FT3X17)
		snprintf(strIcName, STR_NAME, "%s", "FT3X17");
	if (ucIcCode == IC_FT5436)
		snprintf(strIcName, STR_NAME, "%s", "FT5436");
	if (ucIcCode == IC_FT3X27)
		snprintf(strIcName, STR_NAME, "%s", "FT3X27");
	if (ucIcCode == IC_FT5526I)
		snprintf(strIcName, STR_NAME, "%s", "FT5526i");
	if (ucIcCode == IC_FT5416)
		snprintf(strIcName, STR_NAME, "%s", "FT5416");
	if (ucIcCode == IC_FT5426)
		snprintf(strIcName, STR_NAME, "%s", "FT5426");
	if (ucIcCode == IC_FT5435)
		snprintf(strIcName, STR_NAME, "%s", "FT5435");

	if (ucIcCode == IC_FT6X06)
		snprintf(strIcName, STR_NAME, "%s", "FT6X06");
	if (ucIcCode == IC_FT3X06)
		snprintf(strIcName, STR_NAME, "%s", "FT3X06");

	if (ucIcCode == IC_FT6X36)
		snprintf(strIcName, STR_NAME, "%s", "FT6X36");
	if (ucIcCode == IC_FT3X07)
		snprintf(strIcName, STR_NAME, "%s", "FT3X07");
	if (ucIcCode == IC_FT6416)
		snprintf(strIcName, STR_NAME, "%s", "FT6416");
	if (ucIcCode == IC_FT6426)
		snprintf(strIcName, STR_NAME, "%s", "FT6336G/U");

	if (ucIcCode == IC_FT5X16)
		snprintf(strIcName, STR_NAME, "%s", "FT5X16");
	if (ucIcCode == IC_FT5X12)
		snprintf(strIcName, STR_NAME, "%s", "FT5X12");

	if (ucIcCode == IC_FT5506)
		snprintf(strIcName, STR_NAME, "%s", "FT5506");
	if (ucIcCode == IC_FT5606)
		snprintf(strIcName, STR_NAME, "%s", "FT5606");
	if (ucIcCode == IC_FT5816)
		snprintf(strIcName, STR_NAME, "%s", "FT5816");

	if (ucIcCode == IC_FT5822)
		snprintf(strIcName, STR_NAME, "%s", "FT5822");
	if (ucIcCode == IC_FT5626)
		snprintf(strIcName, STR_NAME, "%s", "FT5626");
	if (ucIcCode == IC_FT5726)
		snprintf(strIcName, STR_NAME, "%s", "FT5726");
	if (ucIcCode == IC_FT5826B)
		snprintf(strIcName, STR_NAME, "%s", "FT5826B");
	if (ucIcCode == IC_FT5826S)
		snprintf(strIcName, STR_NAME, "%s", "FT5826S");

	if (ucIcCode == IC_FT5306)
		snprintf(strIcName, STR_NAME, "%s", "FT5306");
	if (ucIcCode == IC_FT5406)
		snprintf(strIcName, STR_NAME, "%s", "FT5406");

	if (ucIcCode == IC_FT8606)
		snprintf(strIcName, STR_NAME, "%s", "FT8606");
	/* if(ucIcCode == IC_FT8716)
		snprintf(strIcName, STR_NAME, "%s",  "FT8716");*/

	if (ucIcCode == IC_FT3267)
		snprintf(strIcName, STR_NAME, "%s", "FT3267");

	return;
}
void OnInit_InterfaceCfg(char * strIniFile)
{
	char str[128];

	///////////////////////////IC_Type
	GetPrivateProfileString("Interface","IC_Type","FT5X36",str,strIniFile);
	g_ScreenSetParam.iSelectedIC = get_ic_code(str);

	/////////////////////////Normalize Type
	GetPrivateProfileString("Interface","Normalize_Type",0,str,strIniFile);
	g_ScreenSetParam.isNormalize = atoi(str);

}
/************************************************************************
* Name: ReadReg(Same function name as FT_MultipleTest)
* Brief:  Read Register
* Input: RegAddr
* Output: RegData
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
int ReadReg(unsigned char RegAddr, unsigned char *RegData)
{
	int iRet;

	if(NULL == fts_i2c_read_test)
		{
		printk("[focal] %s fts_i2c_read == NULL \n", __func__);
		return (ERROR_CODE_INVALID_COMMAND);
		}

	iRet = fts_i2c_read_test(&RegAddr, 1, RegData, 1);

	if(iRet >= 0)
		return (ERROR_CODE_OK);
	else
		return (ERROR_CODE_COMM_ERROR);
}
/************************************************************************
* Name: WriteReg(Same function name as FT_MultipleTest)
* Brief:  Write Register
* Input: RegAddr, RegData
* Output: null
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
int WriteReg(unsigned char RegAddr, unsigned char RegData)
{
	int iRet;
	unsigned char cmd[2] = {0};

	if(NULL == fts_i2c_write_test)
		{
		printk("[focal] %s fts_i2c_write == NULL \n", __func__);
		return (ERROR_CODE_INVALID_COMMAND);
		}

	cmd[0] = RegAddr;
	cmd[1] = RegData;
	iRet = fts_i2c_write_test(cmd, 2);

	if(iRet >= 0)
		return (ERROR_CODE_OK);
	else
		return (ERROR_CODE_COMM_ERROR);
}
/************************************************************************
* Name: Comm_Base_IIC_IO(Same function name as FT_MultipleTest)
* Brief:  Write/Read Data by IIC
* Input: pWriteBuffer, iBytesToWrite, iBytesToRead
* Output: pReadBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char Comm_Base_IIC_IO(unsigned char *pWriteBuffer, int  iBytesToWrite, unsigned char *pReadBuffer, int iBytesToRead)
{
	int iRet;

	if(NULL == fts_i2c_read_test)
		{
		printk("[fts] %s fts_i2c_read == NULL \n", __func__);
		return (ERROR_CODE_INVALID_COMMAND);
		}

	iRet = fts_i2c_read_test(pWriteBuffer, iBytesToWrite, pReadBuffer, iBytesToRead);

	if(iRet >= 0)
		return (ERROR_CODE_OK);
	else
		return (ERROR_CODE_COMM_ERROR);
}
/************************************************************************
* Name: EnterWork(Same function name as FT_MultipleTest)
* Brief:  Enter Work Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char EnterWork(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if(ReCode == ERROR_CODE_OK)
	{
		if(((RunState>>4)&0x07) == 0x00)	//work
		{
			ReCode = ERROR_CODE_OK;
		}
		else
		{
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0);
			if(ReCode == ERROR_CODE_OK)
			{
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if(ReCode == ERROR_CODE_OK)
				{
					if(((RunState>>4)&0x07) == 0x00)	ReCode = ERROR_CODE_OK;
					else	ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	return ReCode;

}
/************************************************************************
* Name: EnterFactory
* Brief:  enter Fcatory Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0 is OK, else fail.
***********************************************************************/
unsigned char EnterFactory(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if(ReCode == ERROR_CODE_OK)
	{
		if(((RunState>>4)&0x07) == 0x04)	//factory
		{
			ReCode = ERROR_CODE_OK;
		}
		else
		{
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0x40);
			if(ReCode == ERROR_CODE_OK)
			{
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if(ReCode == ERROR_CODE_OK)
				{
					if(((RunState>>4)&0x07) == 0x04)	ReCode = ERROR_CODE_OK;
					else	ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	return ReCode;
}


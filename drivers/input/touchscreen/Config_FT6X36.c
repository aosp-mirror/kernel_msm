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
#include "Config_FT6X36.h"
#include "ini.h"
#include "Global.h"


struct stCfg_FT6X36_TestItem g_stCfg_FT6X36_TestItem;
struct stCfg_FT6X36_BasicThreshold g_stCfg_FT6X36_BasicThreshold;


void OnInit_FT6X36_TestItem(char *strIniFile)
{
	char str[512] = {0};

	GetPrivateProfileString("TestItem", "RAWDATA_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.RAWDATA_TEST = atoi(str);

	GetPrivateProfileString("TestItem", "CB_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.CB_TEST = atoi(str);

	GetPrivateProfileString("TestItem", "DELTA_CB_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.DELTA_CB_TEST = atoi(str);
}

void OnInit_FT6X36_BasicThreshold(char *strIniFile)
{
	char str[512] = {0};

	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Min", "13000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.RawDataTest_Min = atoi(str);
	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Max", "17000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.RawDataTest_Max = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "CbTest_Min", "3", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.CbTest_Min = atoi(str);
	GetPrivateProfileString("Basic_Threshold", "CbTest_Max", "900", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.CbTest_Max = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Base", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Base = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Differ_Max", "50", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Differ_Max = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Include_Key_Test", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Include_Key_Test = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Key_Differ_Max", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Key_Differ_Max = atoi(str);


	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S1", "15", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S1 = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S2", "15", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S2 = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S3", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S3 = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S4", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S4 = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S5", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S5 = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S6", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S6 = atoi(str);
}

void SetTestItem_FT6X36(void)
{
	g_TestItemNum = 0;

	/* Enter Factory Mode */
	g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT6X36_ENTER_FACTORY_MODE;
	g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
	g_stTestItem[0][g_TestItemNum].TestResult = RESULT_NULL;
	g_TestItemNum++;

	/* RawData Test */
	if (g_stCfg_FT6X36_TestItem.RAWDATA_TEST == 1) {
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT6X36_RAWDATA_TEST;
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult = RESULT_NULL;
		g_TestItemNum++;
	}

	/* CB Deviation Test */
	if (g_stCfg_FT6X36_TestItem.CB_DEVIATION_TEST == 1) {
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT6X36_CB_DEVIATION_TEST;
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult = RESULT_NULL;
		g_TestItemNum++;
	}

	/* CB_TEST */
	if (g_stCfg_FT6X36_TestItem.CB_TEST == 1) {
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT6X36_CB_TEST;
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult = RESULT_NULL;
		g_TestItemNum++;
	}

	/* DELTA_CB_TEST */
	if (g_stCfg_FT6X36_TestItem.DELTA_CB_TEST == 1) {
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT6X36_DELTA_CB_TEST;
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult = RESULT_NULL;
		g_TestItemNum++;
	}
}


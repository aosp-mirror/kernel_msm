/************************************************************************
* Copyright (C) 2012-2017, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Config_FT6X36.c
*
* Author: Software Development Team, AE
*
* Created: 2015-10-08
*
* Abstract: Set Config for FT6X36/FT3X07/FT6416/FT6426
*
************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>

#include "../include/focaltech_test_ini.h"
#include "../include/focaltech_test_supported_ic.h"
#include "../focaltech_test_config.h"

#if (FTS_CHIP_TEST_TYPE ==FT6X36_TEST)


struct stCfg_FT6X36_TestItem g_stCfg_FT6X36_TestItem;
struct stCfg_FT6X36_BasicThreshold g_stCfg_FT6X36_BasicThreshold;


void OnInit_FT6X36_TestItem(char *strIniFile)
{
	char str[512] = {0};

	FTS_TEST_FUNC_ENTER();
	/* FW Version*/
	GetPrivateProfileString("TestItem", "FW_VERSION_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.FW_VERSION_TEST = fts_atoi(str);

	/* Factory ID*/
	GetPrivateProfileString("TestItem", "FACTORY_ID_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.FACTORY_ID_TEST = fts_atoi(str);

	/* Project Code Test*/
	GetPrivateProfileString("TestItem", "PROJECT_CODE_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.PROJECT_CODE_TEST = fts_atoi(str);

	/* IC Version*/
	GetPrivateProfileString("TestItem", "IC_VERSION_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.IC_VERSION_TEST = fts_atoi(str);

	/* CHANNEL_NUM_TEST*/
	GetPrivateProfileString("TestItem", "CHANNEL_NUM_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.CHANNEL_NUM_TEST = fts_atoi(str);

	/* CHANNEL_SHORT_TEST*/
	GetPrivateProfileString("TestItem", "CHANNEL_SHORT_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.CHANNEL_SHORT_TEST = fts_atoi(str);

	/* INT_PIN_TEST*/
	GetPrivateProfileString("TestItem", "INT_PIN_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.INT_PIN_TEST = fts_atoi(str);

	/* RESET_PIN_TEST*/
	GetPrivateProfileString("TestItem", "RESET_PIN_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.RESET_PIN_TEST = fts_atoi(str);

	/* NOISE_TEST*/
	GetPrivateProfileString("TestItem", "NOISE_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.NOISE_TEST = fts_atoi(str);

	/* CB_TEST*/
	GetPrivateProfileString("TestItem", "CB_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.CB_TEST = fts_atoi(str);

	/* DELTA_CB_TEST*/
	GetPrivateProfileString("TestItem", "DELTA_CB_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.DELTA_CB_TEST = fts_atoi(str);

	/* RawData Test*/
	GetPrivateProfileString("TestItem", "RAWDATA_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.RAWDATA_TEST = fts_atoi(str);

	/* CHANNELS_DEVIATION_TEST*/
	GetPrivateProfileString("TestItem", "CHANNELS_DEVIATION_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.CHANNELS_DEVIATION_TEST = fts_atoi(str);

	/* TWO_SIDES_DEVIATION_TEST*/
	GetPrivateProfileString("TestItem", "TWO_SIDES_DEVIATION_TEST", "1", str, strIniFile);
	g_stCfg_FT6X36_TestItem.TWO_SIDES_DEVIATION_TEST = fts_atoi(str);


	/* FPC_SHORT_TEST*/
	GetPrivateProfileString("TestItem", "FPC_SHORT_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.FPC_SHORT_TEST = fts_atoi(str);

	/* FPC_OPEN_TEST*/
	GetPrivateProfileString("TestItem", "FPC_OPEN_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.FPC_OPEN_TEST = fts_atoi(str);

	/* SREF_OPEN_TEST*/
	GetPrivateProfileString("TestItem", "SREF_OPEN_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.SREF_OPEN_TEST = fts_atoi(str);

	/* TE_TEST*/
	GetPrivateProfileString("TestItem", "TE_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.TE_TEST = fts_atoi(str);

	/* CB_DEVIATION_TEST*/
	GetPrivateProfileString("TestItem", "CB_DEVIATION_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.CB_DEVIATION_TEST = fts_atoi(str);

	/* DIFFER_TEST*/
	GetPrivateProfileString("TestItem", "DIFFER_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.DIFFER_TEST = fts_atoi(str);

	/* WEAK_SHORT_TEST*/
	GetPrivateProfileString("TestItem", "WEAK_SHORT_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.WEAK_SHORT_TEST = fts_atoi(str);

	/* DIFFER_TEST2*/
	GetPrivateProfileString("TestItem", "DIFFER_TEST2", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.DIFFER_TEST2 = fts_atoi(str);

	/* K1_DIFFER_TEST*/
	GetPrivateProfileString("TestItem", "K1_DIFFER_TEST", "0", str, strIniFile);
	g_stCfg_FT6X36_TestItem.K1_DIFFER_TEST = fts_atoi(str);
	FTS_TEST_FUNC_EXIT();
}

void OnInit_FT6X36_BasicThreshold(char *strIniFile)
{
	char str[512] = {0};

	FTS_TEST_FUNC_ENTER();
	/* FW Version*/

	GetPrivateProfileString("Basic_Threshold", "FW_VER_VALUE", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FW_VER_VALUE = fts_atoi(str);

	/* Factory ID*/
	GetPrivateProfileString("Basic_Threshold", "Factory_ID_Number", "255", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.Factory_ID_Number = fts_atoi(str);

	/* Project Code Test*/
	GetPrivateProfileString("Basic_Threshold", "Project_Code", " ", str, strIniFile);
	/*g_stCfg_FT6X36_BasicThreshold.Project_Code.Format("%s", str);*/
	sprintf(g_stCfg_FT6X36_BasicThreshold.Project_Code, "%s", str);

	/* IC Version*/
	GetPrivateProfileString("Basic_Threshold", "IC_Version", "3", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.IC_Version = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Min", "13000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.RawDataTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Max", "17000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.RawDataTest_Max = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold", "ChannelNumTest_ChannelNum", "22", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelNumTest_ChannelNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "ChannelNumTest_KeyNum", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelNumTest_KeyNum = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelShortTest_K1", "255", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelShortTest_K1 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "ChannelShortTest_K2", "255", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelShortTest_K2 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "ChannelShortTest_CB", "255", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelShortTest_CB = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ResetPinTest_RegAddr", "136", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ResetPinTest_RegAddr = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "IntPinTest_RegAddr", "175", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.IntPinTest_RegAddr = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "NoiseTest_Max", "20", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.NoiseTest_Max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "NoiseTest_Frames", "32", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.NoiseTest_Frames = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "NoiseTest_Time", "1", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.NoiseTest_Time = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "NoiseTest_SampeMode", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.NoiseTest_SampeMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "NoiseTest_NoiseMode", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.NoiseTest_NoiseMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "NoiseTest_ShowTip", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.NoiseTest_ShowTip = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "CbTest_Min", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.CbTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "CbTest_Max", "250", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.CbTest_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Base", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Base = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Differ_Max", "50", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Differ_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Include_Key_Test", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Include_Key_Test = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Key_Differ_Max", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Key_Differ_Max = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S1", "15", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S1 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S2", "15", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S2 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S3", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S3 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S4", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S4 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S5", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S5 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Deviation_S6", "12", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S6 = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Set_Critical", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Set_Critical = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Critical_S1", "20", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S1 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Critical_S2", "20", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S2 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Critical_S3", "20", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S3 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Critical_S4", "20", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S4 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Critical_S5", "20", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S5 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DeltaCbTest_Critical_S6", "20", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S6 = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Deviation_S1", "8", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Deviation_S1 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Deviation_S2", "8", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Deviation_S2 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Deviation_S3", "8", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Deviation_S3 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Deviation_S4", "8", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Deviation_S4 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Deviation_S5", "8", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Deviation_S5 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Deviation_S6", "8", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Deviation_S6 = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Set_Critical", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Set_Critical = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Critical_S1", "13", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Critical_S1 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Critical_S2", "13", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Critical_S2 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Critical_S3", "13", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Critical_S3 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Critical_S4", "13", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Critical_S4 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Critical_S5", "13", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Critical_S5 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "ChannelsDeviationTest_Critical_S6", "13", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.ChannelsDeviationTest_Critical_S6 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Deviation_S1", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Deviation_S1 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Deviation_S2", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Deviation_S2 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Deviation_S3", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Deviation_S3 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Deviation_S4", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Deviation_S4 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Deviation_S5", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Deviation_S5 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Deviation_S6", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Deviation_S6 = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Set_Critical", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Set_Critical = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Critical_S1", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Critical_S1 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Critical_S2", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Critical_S2 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Critical_S3", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Critical_S3 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Critical_S4", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Critical_S4 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Critical_S5", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Critical_S5 = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "TwoSidesDeviationTest_Critical_S6", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.TwoSidesDeviationTest_Critical_S6 = fts_atoi(str);

	/* fpc short*/
	GetPrivateProfileString("Basic_Threshold", "FPCShortTest_Min_CB", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCShort_CB_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "FPCShortTest_Max_CB", "1015", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCShort_CB_Max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "FPCShortTest_Min_RawData", "5000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCShort_RawData_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "FPCShortTest_Max_RawData", "50000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCShort_RawData_Max = fts_atoi(str);

	/*fpc open*/
	GetPrivateProfileString("Basic_Threshold", "FPCOpenTest_Min_CB", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCOpen_CB_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "FPCOpenTest_Max_CB", "1015", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCOpen_CB_Max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "FPCOpenTest_Min_RawData", "5000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCOpen_RawData_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "FPCOpenTest_Max_RawData", "50000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.FPCOpen_RawData_Max = fts_atoi(str);

	/*SREFOpen_Test_Hole*/
	GetPrivateProfileString("Basic_Threshold", "SREFOpen_Test_Hole", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.SREFOpen_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "SREFOpen_Test_Hole_Base1", "50", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.SREFOpen_Hole_Base1 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "SREFOpen_Test_Hole_Base2", "50", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.SREFOpen_Hole_Base2 = fts_atoi(str);

	/*CB_DEVIATION_TEST*/
	GetPrivateProfileString("Basic_Threshold", "CBDeviationTest_Hole", "50", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.CBDeviationTest_Hole = fts_atoi(str);

	/*DIFFER_TEST*/
	GetPrivateProfileString("Basic_Threshold", "DifferTest_Ave_Hole", "500", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.Differ_Ave_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "DifferTest_Max_Hole", "500", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.Differ_Max_Hole = fts_atoi(str);


	/*WEAK_SHORT_TEST*/
	GetPrivateProfileString("Basic_Threshold", "Weak_Short_Hole", "500", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.WeakShortThreshold = fts_atoi(str);

	/*DIFFER_TEST2*/
	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_H_Min", "20000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DifferTest2_Data_H_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_H_Max", "24000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DifferTest2_Data_H_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_M_Min", "7100", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DifferTest2_Data_M_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_M_Max", "7300", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DifferTest2_Data_M_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_L_Min", "14000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DifferTest2_Data_L_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_L_Max", "16000", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.DifferTest2_Data_L_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_H", "1", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.bDifferTest2_Data_H = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_M", "1", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.bDifferTest2_Data_M = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "DifferTest2_Data_L", "1", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.bDifferTest2_Data_L = fts_atoi(str);

	/*K1 differ test*/
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_StartK1", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_StartK1 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_EndK1", "25", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_EndK1 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_MinHole_STC2", "0", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_MinHold2 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_MaxHole_STC2", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_MaxHold2 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_MinHole_STC4", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_MinHold4 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_MaxHole_STC4", "10", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_MaxHold4 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_Deviation2", "1", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_Deviation2 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "K1DifferTest_Deviation4", "5", str, strIniFile);
	g_stCfg_FT6X36_BasicThreshold.K1DifferTest_Deviation4 = fts_atoi(str);
	FTS_TEST_FUNC_EXIT();
}

void SetTestItem_FT6X36(void)
{

	g_TestItemNum = 0;

	FTS_TEST_FUNC_ENTER();
	/*FACTORY_ID_TEST*/
	if (g_stCfg_FT6X36_TestItem.FACTORY_ID_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_FACTORY_ID_TEST);
	}

	/*Project Code Test*/
	if (g_stCfg_FT6X36_TestItem.PROJECT_CODE_TEST == 1)
		fts_SetTestItemCodeName(Code_FT6X36_PROJECT_CODE_TEST);

	/*FW Version Test*/
	if (g_stCfg_FT6X36_TestItem.FW_VERSION_TEST == 1)
		fts_SetTestItemCodeName(Code_FT6X36_FW_VERSION_TEST);

	/*IC Version Test*/
	if (g_stCfg_FT6X36_TestItem.IC_VERSION_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_IC_VERSION_TEST);
	}

	/*Enter Factory Mode*/

	fts_SetTestItemCodeName(Code_FT6X36_ENTER_FACTORY_MODE);

	/*CHANNEL_NUM_TEST*/
	if (g_stCfg_FT6X36_TestItem.CHANNEL_NUM_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_CHANNEL_NUM_TEST);
	}


	/*Differ Test*/
	if (g_stCfg_FT6X36_TestItem.DIFFER_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_DIFFER_TEST);
	}

	/*Differ Test2*/
	if (g_stCfg_FT6X36_TestItem.DIFFER_TEST2 == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_DIFFER_TEST2);
	}

	/*CB Deviation Test*/
	if (g_stCfg_FT6X36_TestItem.CB_DEVIATION_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_CB_DEVIATION_TEST);
	}

	/*FPC_SHORT_TEST*/
	if (g_stCfg_FT6X36_TestItem.FPC_SHORT_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_FPC_SHORT_TEST);
	}

	/*FPC_OPEN_TEST*/
	if (g_stCfg_FT6X36_TestItem.FPC_OPEN_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_FPC_OPEN_TEST);
	}

	/*CB_TEST*/
	if (g_stCfg_FT6X36_TestItem.CB_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_CB_TEST);
	}

	/*DELTA_CB_TEST*/
	if (g_stCfg_FT6X36_TestItem.DELTA_CB_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_DELTA_CB_TEST);
	}

	/*RawData Test*/
	if (g_stCfg_FT6X36_TestItem.RAWDATA_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_RAWDATA_TEST);
	}

	/*CHANNELS_DEVIATION_TEST*/
	if (g_stCfg_FT6X36_TestItem.CHANNELS_DEVIATION_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_CHANNELS_DEVIATION_TEST);
	}

	/*TWO_SIDES_DEVIATION_TEST*/
	if (g_stCfg_FT6X36_TestItem.TWO_SIDES_DEVIATION_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_TWO_SIDES_DEVIATION_TEST);
	}

	/*CHANNEL_SHORT_TEST*/
	if (g_stCfg_FT6X36_TestItem.CHANNEL_SHORT_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_CHANNEL_SHORT_TEST);
	}

	/*SREF_OPEN_TEST*/
	if (g_stCfg_FT6X36_TestItem.SREF_OPEN_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_SREF_OPEN_TEST);
	}

	/*NOISE_TEST*/
	if (g_stCfg_FT6X36_TestItem.NOISE_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_NOISE_TEST);
	}

	/*Weak Short Test*/
	if (g_stCfg_FT6X36_TestItem.WEAK_SHORT_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_WEAK_SHORT_TEST);
	}

	/*K1 Differ Test*/
	if (g_stCfg_FT6X36_TestItem.K1_DIFFER_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_K1_DIFFER_TEST);
	}

	/*RESET_PIN_TEST*/
	if (g_stCfg_FT6X36_TestItem.RESET_PIN_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_RESET_PIN_TEST);
	}
	/*INT_PIN_TEST*/
	if (g_stCfg_FT6X36_TestItem.INT_PIN_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_INT_PIN_TEST);
	}

	/*TE Test*/
	if (g_stCfg_FT6X36_TestItem.TE_TEST == 1) {

		fts_SetTestItemCodeName(Code_FT6X36_TE_TEST);
	}

	FTS_TEST_FUNC_EXIT();
}

#endif


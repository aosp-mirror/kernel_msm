/************************************************************************
* Copyright (C) 2012-2017, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Test_FT6X36.c
*
* Author: Software Development Team, AE
*
* Created: 2015-10-08
*
* Abstract: test item for FT6X36/FT3X07/FT6416/FT6426
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "../include/focaltech_test_detail_threshold.h"
#include "../include/focaltech_test_supported_ic.h"
#include "../include/focaltech_test_main.h"
#include "../focaltech_test_config.h"

#if (FTS_CHIP_TEST_TYPE ==FT6X36_TEST)

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

#define DEVIDE_MODE_ADDR    0x00
#define REG_LINE_NUM    0x01
#define REG_TX_NUM  0x02
#define REG_RX_NUM  0x03
#define REG_PATTERN_5422        0x53
#define REG_MAPPING_SWITCH      0x54
#define REG_TX_NOMAPPING_NUM        0x55
#define REG_RX_NOMAPPING_NUM      0x56
#define REG_NORMALIZE_TYPE      0x16
#define REG_ScCbBuf0    0x4E
#define REG_ScWorkMode  0x44
#define REG_ScCbAddrR   0x45
#define REG_RawBuf0 0x36
#define REG_WATER_CHANNEL_SELECT 0x09


#define C6208_SCAN_ADDR 0x08

#define C6X36_CHANNEL_NUM   0x0A        /*  1 Byte read and write (RW)  TP_Channel_Num  The maximum number of channels in VA area is 63*/
#define C6X36_KEY_NUM       0x0B        /*  1 Byte read and write(RW)           TP_Key_Num      The maximum number of virtual keys independent channel is 63*/
#define C6X36_CB_ADDR_W 0x32            /*    1 Byte    read and write(RW)  CB_addr     Normal mode CB-- address    */
#define C6X36_CB_ADDR_R 0x33            /*  1 Byte  read and write(RW)  CB_addr     Normal mode CB-- address*/
#define C6X36_CB_BUF  0x39           /*Byte only read (RO)  CB_buf      Ò»The channel is 2 bytes, and the length is 2*N                 */
#define C6X36_RAWDATA_ADDR  0x34    /* 1 Byte    read and write(RW)  RawData_addr        Rawdata--address                            */
#define C6X36_RAWDATA_BUF   0x35

#define C6206_FACTORY_TEST_MODE         0xAE    /*0: normal factory model, 1: production test factory mode 1 (using a single  +0 level of water scanning), 2: production test factory mode 2 (using a single ended + non waterproof scanning)*/
#define C6206_FACTORY_TEST_STATUS       0xAD

#define MAX_SCAP_CHANNEL_NUM        144/*Single Chip 72; Double Chips 144*/
#define MAX_SCAP_KEY_NUM            8

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
enum WaterproofType {
	WT_NeedProofOnTest,
	WT_NeedProofOffTest,
	WT_NeedTxOnVal,
	WT_NeedRxOnVal,
	WT_NeedTxOffVal,
	WT_NeedRxOffVal,
};
/*****************************************************************************
* Static variables
*****************************************************************************/

static int m_RawData[MAX_SCAP_CHANNEL_NUM] = {0};
static unsigned char m_ucTempData[MAX_SCAP_CHANNEL_NUM * 2] = {0};

static int m_CbData[MAX_SCAP_CHANNEL_NUM] = {0};
static int m_TempCbData[MAX_SCAP_CHANNEL_NUM] = {0};
static int m_DeltaCbData[MAX_SCAP_CHANNEL_NUM] = {0};
static int m_DeltaCb_DifferData[MAX_SCAP_CHANNEL_NUM] = {0};


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct stCfg_FT6X36_BasicThreshold g_stCfg_FT6X36_BasicThreshold;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
/*Communication function*/
static int StartScan(void);
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer);
static unsigned char GetPanelChannels(unsigned char *pPanelRows);
static unsigned char GetPanelKeys(unsigned char *pPanelCols);
static unsigned char GetRawData(void);
static unsigned char GetChannelNum(void);
static void Save_Test_Data(int iData[MAX_SCAP_CHANNEL_NUM], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount);
static void ShowRawData(void);

boolean FT6X36_StartTest(void);
int FT6X36_get_test_data(char *pTestData);/*pTestData, External application for memory, buff size >= 1024*80*/

unsigned char FT6X36_TestItem_EnterFactoryMode(void);
unsigned char FT6X36_TestItem_RawDataTest(bool *bTestResult);
unsigned char FT6X36_TestItem_CbTest(bool *bTestResult);
unsigned char FT6X36_TestItem_DeltaCbTest(unsigned char *bTestResult);



/************************************************************************
* Name: FT6X36_StartTest
* Brief:  Test entry. Determine which test item to test
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/
boolean FT6X36_StartTest()
{
	bool bTestResult = true;
	bool bTempResult = 1;
	unsigned char ReCode = 0;
	unsigned char ucDevice = 0;
	int iItemCount = 0;

	/*1. Init part*/
	if (InitTest() < 0) {
		FTS_TEST_ERROR("[focal] Failed to init test.");
		return false;
	}

	/*-2. test item*/
	if (0 == g_TestItemNum)
		bTestResult = false;

	for (iItemCount = 0; iItemCount < g_TestItemNum; iItemCount++) {
		m_ucTestItemCode = g_stTestItem[ucDevice][iItemCount].ItemCode;

		/*FT6X36_ENTER_FACTORY_MODE*/
		if (Code_FT6X36_ENTER_FACTORY_MODE == g_stTestItem[ucDevice][iItemCount].ItemCode
		   ) {
			ReCode = FT6X36_TestItem_EnterFactoryMode();
			if (ERROR_CODE_OK != ReCode || (!bTempResult)) {
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
				break;/*if this item FAIL, no longer test.*/
			} else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		/*FT6X36_RAWDATA_TEST*/
		if (Code_FT6X36_RAWDATA_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
		   ) {
			ReCode = FT6X36_TestItem_RawDataTest(&bTempResult);
			if (ERROR_CODE_OK != ReCode || (!bTempResult)) {
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			} else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}


		/*FT6X36_CB_TEST*/
		if (Code_FT6X36_CB_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
		   ) {
			ReCode = FT6X36_TestItem_CbTest(&bTempResult);
			if (ERROR_CODE_OK != ReCode || (!bTempResult)) {
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			} else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		/*Code_FT6X36_DELTA_CB_TEST*/
		if (Code_FT6X36_DELTA_CB_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
		   ) {
			ReCode = FT6X36_TestItem_DeltaCbTest(&bTempResult);
			if (ERROR_CODE_OK != ReCode || (!bTempResult)) {
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			} else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

	}

	/*--------------3. End Part*/
	FinishTest();

	/*--------------4. return result*/
	return bTestResult;
}

/************************************************************************
* Name: FT6X36_TestItem_EnterFactoryMode
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT6X36_TestItem_EnterFactoryMode(void)
{
	unsigned char ReCode = ERROR_CODE_INVALID_PARAM;
	int iRedo = 5;  /*If failed, repeat 5 times.*/
	int i ;

	SysDelay(150);
	for (i = 1; i <= iRedo; i++) {
		ReCode = EnterFactory();
		if (ERROR_CODE_OK != ReCode) {
			FTS_TEST_ERROR("Failed to Enter factory mode...");
			if (i < iRedo) {
				SysDelay(50);
				continue;
			}
		} else {
			break;
		}

	}
	SysDelay(300);

	if (ReCode != ERROR_CODE_OK) {
		return ReCode;
	}

	/*After the success of the factory model, read the number of channels*/
	ReCode = GetChannelNum();

	return ReCode;
}


/************************************************************************
* Name: FT6X36_TestItem_RawDataTest
* Brief:  TestItem: RawDataTest. Check if SCAP RawData is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT6X36_TestItem_RawDataTest(bool *bTestResult)
{
	int i = 0;
	/*bool bFlag = true;*/
	unsigned char ReCode = ERROR_CODE_OK;
	bool btmpresult = true;
	int RawDataMin = 0, RawDataMax = 0;

	int iNgNum = 0;
	int iMax = 0, iMin = 0, iAvg = 0;

	RawDataMin = g_stCfg_FT6X36_BasicThreshold.RawDataTest_Min;
	RawDataMax = g_stCfg_FT6X36_BasicThreshold.RawDataTest_Max;

	FTS_TEST_INFO("====Test Item: -------- RawData Test \r");

	for (i = 0; i < 3; i++) {
		ReCode = WriteReg(C6206_FACTORY_TEST_MODE, Proof_Normal);
		if (ERROR_CODE_OK == ReCode)
			ReCode = StartScan();
		if (ERROR_CODE_OK == ReCode)
			break;
	}

	ReCode = GetRawData();

	if (ReCode == ERROR_CODE_OK) { /*Read the RawData and calculate the Differ value*/
		FTS_TEST_DBG("==== Test Data:  ");
		ShowRawData();
	} else {
		FTS_TEST_ERROR("RawData Test is Error. Failed to get Raw Data!!");
		btmpresult = false;/*Can not get RawData, is also considered  NG*/
		goto TEST_END;
	}

	/*-------------Judge over the range of rawData*/
	iNgNum = 0;
	iMax = m_RawData[0];
	iMin = m_RawData[0];
	iAvg = 0;

	for (i = 0; i < g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum; i++) {
		RawDataMin = g_stCfg_SCap_DetailThreshold.RawDataTest_Min[i];/*Fetch detail threshold*/
		RawDataMax = g_stCfg_SCap_DetailThreshold.RawDataTest_Max[i];/*Fetch detail threshold*/
		if (m_RawData[i] < RawDataMin || m_RawData[i] > RawDataMax) {
			btmpresult = false;

			if (iNgNum == 0)
				FTS_TEST_DBG("NG Data: \r");

			if (i < g_ScreenSetParam.iChannelsNum)
				FTS_TEST_DBG("Ch_%02d: %d Set_Range=(%d, %d) ,	", i + 1, m_RawData[i], RawDataMin, RawDataMax);
			else
				FTS_TEST_DBG("Key_%d: %d Set_Range=(%d, %d) ,	", i + 1 - g_ScreenSetParam.iChannelsNum, m_RawData[i], RawDataMin, RawDataMax);
			if (iNgNum % 6 == 0) {
				FTS_TEST_DBG("\r\n");
			}

			iNgNum++;
		}

		/*Just calculate the maximum minimum average value*/
		iAvg += m_RawData[i];
		if (iMax < m_RawData[i])iMax = m_RawData[i];
		if (iMin > m_RawData[i])iMin = m_RawData[i];

	}

	iAvg /= g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum;
	FTS_TEST_DBG("Max Raw Value: %d, Min Raw Value: %d, Deviation Value: %d, Average Value: %d", iMax, iMin, iMax - iMin, iAvg);

	/*>>Collect test data and save CSV file.*/

	Save_Test_Data(m_RawData, 0, 1, g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum, 1);
	TestResultLen += sprintf(TestResult + TestResultLen, " RawData Test is %s. \n\n", (btmpresult ? "OK" : "NG"));

TEST_END:
	if (btmpresult) {
		* bTestResult = true;
		FTS_TEST_INFO("RawData Test is OK!\r");
	} else {
		* bTestResult = false;
		FTS_TEST_INFO("RawData Test is NG!\r");
	}
	TestResultLen += sprintf(TestResult + TestResultLen, " RawData Test is %s. \n\n", (btmpresult ? "OK" : "NG"));
	return ReCode;
}

/************************************************************************
* Name: FT6X36_TestItem_CbTest
* Brief:  TestItem: CB Test. Check if SCAP CB is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT6X36_TestItem_CbTest(bool *bTestResult)
{
	int readlen = 0;

	bool btmpresult = true;
	int iCbMin = 0, iCbMax = 0;
	unsigned char chOldMode = 0;

	/*unsigned char WaterProofResult=0;*/
	unsigned char ReCode = ERROR_CODE_OK;
	BYTE pReadData[300] = {0};
	unsigned char I2C_wBuffer[1];

	int iNgNum = 0;
	int iMax = 0, iMin = 0, iAvg = 0;

	int i = 0;

	memset(m_CbData, 0, sizeof(m_CbData));
	readlen = g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum;

	FTS_TEST_INFO("==========Test Item: --------  CB Test");

	iCbMin = g_stCfg_FT6X36_BasicThreshold.CbTest_Min;
	iCbMax = g_stCfg_FT6X36_BasicThreshold.CbTest_Max;

	ReCode = ReadReg(C6206_FACTORY_TEST_MODE, &chOldMode);

	for (i = 0; i < 3; i++) {
		ReCode = WriteReg(C6206_FACTORY_TEST_MODE, Proof_NoWaterProof);
		if (ERROR_CODE_OK == ReCode)
			ReCode = StartScan();
		if (ERROR_CODE_OK == ReCode)break;
	}

	if ((ERROR_CODE_OK != ReCode)/* || (1 != WaterProofResult)*/) {
		btmpresult = false;
		FTS_TEST_ERROR("=====  CB test Failed!");
	} else {
		FTS_TEST_DBG("nGet Proof_NoWaterProof CB Data...");


		/*Waterproof CB*/
		I2C_wBuffer[0] = 0x39;
		ReCode = WriteReg(0x33, 0);
		ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, pReadData, readlen * 2);

		for (i = 0; i < readlen; i++) {
			m_TempCbData[i] = (unsigned short)(pReadData[i * 2] << 8 | pReadData[i * 2 + 1]);

			/*SysDelay(5);*/

			if (i == 0) { /*Half*/
				FTS_TEST_DBG("== CB Data: ");
				FTS_TEST_DBG("Left Channel:	");
			} else if (i * 2 == g_ScreenSetParam.iChannelsNum)
				FTS_TEST_DBG("Right Channel:	");
			else if (i ==  g_ScreenSetParam.iChannelsNum)
				FTS_TEST_DBG("\r\nKey:		");

		}
	}

	/*Single waterproof*/
	FTS_TEST_DBG("Proof_Level0 CB Test...\r");
	for (i = 0; i < 3; i++) {
		ReCode = WriteReg(C6206_FACTORY_TEST_MODE, Proof_Level0);
		if (ERROR_CODE_OK == ReCode)
			ReCode = StartScan();
		if (ERROR_CODE_OK == ReCode)break;
	}
	if ((ERROR_CODE_OK != ReCode)/* || (1 != WaterProofResult)*/) {
		btmpresult = false;
		FTS_TEST_ERROR("=== CB test Failed!");
	} else {
		FTS_TEST_DBG("Get Proof_Level0 CB Data...");

		/*BYTE pReadData[300] = {0};*/
		/*unsigned char I2C_wBuffer[1];*/
		/*Waterproof CB*/
		I2C_wBuffer[0] = 0x39;
		ReCode = WriteReg(0x33, 0);
		ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, pReadData, readlen * 2);

		for (i = 0; i < readlen; i++) {
			m_CbData[i] = (unsigned short)(pReadData[i * 2] << 8 | pReadData[i * 2 + 1]);
		}

		ReCode = WriteReg(C6206_FACTORY_TEST_MODE, chOldMode);

		/*----------------------------------------------------------Judge whether or not to exceed the threshold*/
		iNgNum = 0;
		iMax = m_TempCbData[0];
		iMin = m_TempCbData[0];
		iAvg = 0;

		for (i = 0; i < g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum; i++) {

			iCbMin = g_stCfg_SCap_DetailThreshold.CbTest_Min[i];/*Fetch detail threshold*/
			iCbMax = g_stCfg_SCap_DetailThreshold.CbTest_Max[i];/*Fetch detail threshold*/

			if (m_TempCbData[i] < iCbMin || m_TempCbData[i] > iCbMax) {
				if (iNgNum == 0) {
					FTS_TEST_DBG("NG Data: \r");
				}
				btmpresult = false;
				if (i < g_ScreenSetParam.iChannelsNum)
					FTS_TEST_DBG("Ch_%02d: %d Set_Range=(%d, %d) ,	", i + 1, m_TempCbData[i], iCbMin, iCbMax);
				else
					FTS_TEST_DBG("Key_%d: %d Set_Range=(%d, %d),	", i + 1 - g_ScreenSetParam.iChannelsNum, m_TempCbData[i], iCbMin, iCbMax);
				if (iNgNum % 6 == 0) {
					FTS_TEST_DBG("\r");
				}

				iNgNum++;
			}

			/* calculate the maximum minimum average value*/
			iAvg += m_TempCbData[i];
			if (iMax < m_TempCbData[i])iMax = m_TempCbData[i];
			if (iMin > m_TempCbData[i])iMin = m_TempCbData[i];

		}

		iAvg /= g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum;
		FTS_TEST_DBG(" Max CB Value: %d, Min CB Value: %d, Deviation Value: %d, Average Value: %d", iMax, iMin, iMax - iMin, iAvg);

		if (btmpresult) {
			FTS_TEST_INFO("CB Test is OK!\r");
			* bTestResult = 1;
		} else {
			FTS_TEST_INFO("CB Test is NG!\r");
			* bTestResult = 0;
		}
	}
	/*>Collect test data and save CSV file.*/

	Save_Test_Data(m_TempCbData, 0, 1, g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum, 1);

	TestResultLen += sprintf(TestResult + TestResultLen, " CB Test is %s. \n\n", (btmpresult ? "OK" : "NG"));

	return ReCode;

}

/************************************************************************
* Name: FT6X36_TestItem_DeltaCbTest
* Brief:  TestItem: Delta CB Test. Check if SCAP Delta CB is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT6X36_TestItem_DeltaCbTest(unsigned char *bTestResult)
{
	bool btmpresult = true;
	int readlen = g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum;
	int i = 0;

	/*The maximum Delta_Ci and minimum Delta_Ci difference is less than the preset value*/
	int Delta_Ci_Differ = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S1;
	int Delta_Ci_Differ_S2 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S2;
	int Delta_Ci_Differ_S3 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S3;
	int Delta_Ci_Differ_S4 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S4;
	int Delta_Ci_Differ_S5 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S5;
	int Delta_Ci_Differ_S6 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S6;

	int Delta_Min = 0, Delta_Max = 0;

	int Critical_Delta_S1 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S1;
	int Critical_Delta_S2 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S2;
	int Critical_Delta_S3 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S3;
	int Critical_Delta_S4 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S4;
	int Critical_Delta_S5 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S5;
	int Critical_Delta_S6 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S6;

	bool bUseCriticalValue = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Set_Critical;

	bool bCriticalResult = false;


	/*new test*/
	int Sort1Min, Sort2Min, Sort3Min, Sort4Min, Sort5Min, Sort6Min;
	int Sort1Max, Sort2Max, Sort3Max, Sort4Max, Sort5Max, Sort6Max;
	int Sort1Min_ChNum, Sort2Min_ChNum, Sort3Min_ChNum, Sort4Min_ChNum, Sort5Min_ChNum, Sort6Min_ChNum;
	int Sort1Max_ChNum, Sort2Max_ChNum, Sort3Max_ChNum, Sort4Max_ChNum, Sort5Max_ChNum, Sort6Max_ChNum;
	bool bUseSort1 = false;
	bool bUseSort2 = false;
	bool bUseSort3 = false;
	bool bUseSort4 = false;
	bool bUseSort5 = false;
	bool bUseSort6 = false;

	int Num = 0;

	int Key_Delta_Max = 0;
	int SetKeyMax = 0;
	int set_Delta_Cb_Max = 0;

	FTS_TEST_INFO("Test Item: Delta CB Test ");

	for (i = 0; i < readlen; i++) {
		m_DeltaCbData[i] = m_TempCbData[i] - m_CbData[i];
		if (i == 0) { /*Half*/
			FTS_TEST_DBG("======= Delta CB Data: ");
			FTS_TEST_DBG("Left Channel:	");
		} else if (i * 2 == g_ScreenSetParam.iChannelsNum) {
			FTS_TEST_DBG("Right Channel:	");
		} else if (i ==  g_ScreenSetParam.iChannelsNum) {
			FTS_TEST_DBG("Key:		");
		}
		FTS_TEST_DBG("%3d	", m_DeltaCbData[i]);

	}

	/*Delta CB Differ*/
	for (i = 0; i < readlen; i++) {
		m_DeltaCb_DifferData[i] = m_DeltaCbData[i] - g_stCfg_SCap_DetailThreshold.DeltaCbTest_Base[i];

		if (i == 0) { /*Half*/
			FTS_TEST_DBG("Differ Data of Delta CB: ");
			FTS_TEST_DBG("Left Channel:	");
		} else if (i * 2 == g_ScreenSetParam.iChannelsNum) {
			FTS_TEST_DBG("Right Channel:	");
		} else if (i ==  g_ScreenSetParam.iChannelsNum) {
			FTS_TEST_DBG("Key:		");
		}
		FTS_TEST_DBG("%3d	", m_DeltaCb_DifferData[i]);
	}

	/*The maximum Delta_Ci and minimum Delta_Ci difference is less than the preset value*/
	Delta_Ci_Differ = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S1;
	Delta_Ci_Differ_S2 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S2;
	Delta_Ci_Differ_S3 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S3;
	Delta_Ci_Differ_S4 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S4;
	Delta_Ci_Differ_S5 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S5;
	Delta_Ci_Differ_S6 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Deviation_S6;

	Delta_Min = 0;
	Delta_Max = 0;

	Critical_Delta_S1 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S1;
	Critical_Delta_S2 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S2;
	Critical_Delta_S3 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S3;
	Critical_Delta_S4 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S4;
	Critical_Delta_S5 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S5;
	Critical_Delta_S6 = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Critical_S6;

	bUseCriticalValue = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Set_Critical;

	bCriticalResult = false;


	/*new test*/
	bUseSort1 = false;
	bUseSort2 = false;
	bUseSort3 = false;
	bUseSort4 = false;
	bUseSort5 = false;
	bUseSort6 = false;

	Num = 0;

	Sort1Min_ChNum = Sort2Min_ChNum = Sort3Min_ChNum = Sort4Min_ChNum = Sort5Min_ChNum = Sort6Min_ChNum = 0;
	Sort1Max_ChNum = Sort2Max_ChNum = Sort3Max_ChNum = Sort4Max_ChNum = Sort5Max_ChNum = Sort6Max_ChNum = 0;

	Sort1Min = Sort2Min = Sort3Min = Sort4Min = Sort5Min  = Sort6Min = 1000;
	Sort1Max = Sort2Max = Sort3Max = Sort4Max = Sort5Max = Sort6Max = -1000;

	for (i = 0; i < g_ScreenSetParam.iChannelsNum/*readlen*/; i++) {
		if (g_stCfg_SCap_DetailThreshold.DeltaCxTest_Sort[i] == 1) {
			bUseSort1 = true;
			if (m_DeltaCb_DifferData[i] < Sort1Min) {
				Sort1Min = m_DeltaCb_DifferData[i];
				Sort1Min_ChNum = i;
			}
			if (m_DeltaCb_DifferData[i] > Sort1Max) {
				Sort1Max = m_DeltaCb_DifferData[i];
				Sort1Max_ChNum = i;
			}
		}
		if (g_stCfg_SCap_DetailThreshold.DeltaCxTest_Sort[i] == 2) {
			bUseSort2 = true;
			if (m_DeltaCb_DifferData[i] < Sort2Min) {
				Sort2Min = m_DeltaCb_DifferData[i];
				Sort2Min_ChNum = i;
			}
			if (m_DeltaCb_DifferData[i] > Sort2Max) {
				Sort2Max = m_DeltaCb_DifferData[i];
				Sort2Max_ChNum = i;
			}
		}
		if (g_stCfg_SCap_DetailThreshold.DeltaCxTest_Sort[i] == 3) {
			bUseSort3 = true;
			if (m_DeltaCb_DifferData[i] < Sort3Min) {
				Sort3Min = m_DeltaCb_DifferData[i];
				Sort3Min_ChNum = i;
			}
			if (m_DeltaCb_DifferData[i] > Sort3Max) {
				Sort3Max = m_DeltaCb_DifferData[i];
				Sort3Max_ChNum = i;
			}
		}
		if (g_stCfg_SCap_DetailThreshold.DeltaCxTest_Sort[i] == 4) {
			bUseSort4 = true;
			if (m_DeltaCb_DifferData[i] < Sort4Min) {
				Sort4Min = m_DeltaCb_DifferData[i];
				Sort4Min_ChNum = i;
			}
			if (m_DeltaCb_DifferData[i] > Sort4Max) {
				Sort4Max = m_DeltaCb_DifferData[i];
				Sort4Max_ChNum = i;
			}
		}
		if (g_stCfg_SCap_DetailThreshold.DeltaCxTest_Sort[i] == 5) {
			bUseSort5 = true;
			if (m_DeltaCb_DifferData[i] < Sort5Min) {
				Sort5Min = m_DeltaCb_DifferData[i];
				Sort5Min_ChNum = i;
			}
			if (m_DeltaCb_DifferData[i] > Sort5Max) {
				Sort5Max = m_DeltaCb_DifferData[i];
				Sort5Max_ChNum = i;
			}
		}
		if (g_stCfg_SCap_DetailThreshold.DeltaCxTest_Sort[i] == 6) {
			bUseSort6 = true;
			if (m_DeltaCb_DifferData[i] < Sort6Min) {
				Sort6Min = m_DeltaCb_DifferData[i];
				Sort6Min_ChNum = i;
			}
			if (m_DeltaCb_DifferData[i] > Sort6Max) {
				Sort6Max = m_DeltaCb_DifferData[i];
				Sort6Max_ChNum = i;
			}
		}


	}
	if (bUseSort1) {
		if (Delta_Ci_Differ <= Sort1Max - Sort1Min) {
			if (bUseCriticalValue) {
				if (Sort1Max - Sort1Min >= Critical_Delta_S1) {
					btmpresult = false;
				} else {
					if (focal_abs(Sort1Max) > focal_abs(Sort1Min))
						Num = Sort1Max_ChNum;
					else
						Num = Sort1Min_ChNum;
					/*SetCriticalMsg(Num);*/

					bCriticalResult = true;
				}
				FTS_TEST_DBG(" Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Critical Deviation of Sort1: %d",
							 Sort1Max, Sort1Min, Sort1Max - Sort1Min, Critical_Delta_S1);
			} else {
				btmpresult = false;
				FTS_TEST_ERROR("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort1: %d",
							   Sort1Max, Sort1Min, Sort1Max - Sort1Min, Delta_Ci_Differ);
			}
		} else {
			FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort1: %d",
						 Sort1Max, Sort1Min, Sort1Max - Sort1Min, Delta_Ci_Differ);
		}

		FTS_TEST_DBG("Max Deviation,Sort1: %d, ", Sort1Max - Sort1Min);
	}
	if (bUseSort2) {

		if (Delta_Ci_Differ_S2 <= Sort2Max - Sort2Min) {
			if (bUseCriticalValue) {
				if (Sort2Max - Sort2Min >= Critical_Delta_S2) {
					btmpresult = false;
				} else {
					if (focal_abs(Sort2Max) > focal_abs(Sort2Min))
						Num = Sort2Max_ChNum;
					else
						Num = Sort2Min_ChNum;
					/*SetCriticalMsg(Num);*/
					bCriticalResult = true;
				}
				FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Critical Deviation of Sort2: %d",
							 Sort2Max, Sort2Min, Sort2Max - Sort2Min, Critical_Delta_S2);
			} else {
				btmpresult = false;
				FTS_TEST_ERROR("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort2: %d",
							   Sort2Max, Sort2Min, Sort2Max - Sort2Min, Delta_Ci_Differ_S2);
			}
		} else {
			FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort2: %d",
						 Sort2Max, Sort2Min, Sort2Max - Sort2Min, Delta_Ci_Differ_S2);
		}

		FTS_TEST_DBG("Sort2: %d, ", Sort2Max - Sort2Min);
	}
	if (bUseSort3) {

		if (Delta_Ci_Differ_S3 <= Sort3Max - Sort3Min) {
			if (bUseCriticalValue) {
				if (Sort3Max - Sort3Min >= Critical_Delta_S3) {
					btmpresult = false;
				} else {
					if (focal_abs(Sort3Max) > focal_abs(Sort3Min))
						Num = Sort3Max_ChNum;
					else
						Num = Sort3Min_ChNum;
					/*SetCriticalMsg(Num);*/
					bCriticalResult = true;
				}
				FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Critical Deviation of Sort3: %d",
							 Sort3Max, Sort3Min, Sort3Max - Sort3Min, Critical_Delta_S3);
			} else {
				btmpresult = false;
				FTS_TEST_ERROR("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort3: %d",
							   Sort3Max, Sort3Min, Sort3Max - Sort3Min, Delta_Ci_Differ_S3);

			}
		} else {
			FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort3: %d",
						 Sort3Max, Sort3Min, Sort3Max - Sort3Min, Delta_Ci_Differ_S3);

		}
		FTS_TEST_DBG("Sort3: %d, ", Sort3Max - Sort3Min);
	}
	if (bUseSort4) {
		if (Delta_Ci_Differ_S4 <= Sort4Max - Sort4Min) {
			if (bUseCriticalValue) {
				if (Sort4Max - Sort4Min >= Critical_Delta_S4) {
					btmpresult = false;
				} else {
					if (focal_abs(Sort4Max) > focal_abs(Sort4Min))
						Num = Sort4Max_ChNum;
					else
						Num = Sort4Min_ChNum;
					/*SetCriticalMsg(Num);*/
					bCriticalResult = true;
				}
				FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Critical Deviation of Sort4: %d",
							 Sort4Max, Sort4Min, Sort4Max - Sort4Min, Critical_Delta_S4);

			} else {
				btmpresult = false;
				FTS_TEST_ERROR("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort4: %d",
							   Sort4Max, Sort4Min, Sort4Max - Sort4Min, Delta_Ci_Differ_S4);

			}
		} else {
			FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort4: %d",
						 Sort4Max, Sort4Min, Sort4Max - Sort4Min, Delta_Ci_Differ_S4);

		}
		FTS_TEST_DBG("Sort4: %d, ", Sort4Max - Sort4Min);
	}
	if (bUseSort5) {
		if (Delta_Ci_Differ_S5 <= Sort5Max - Sort5Min) {
			if (bUseCriticalValue) {
				if (Sort5Max - Sort5Min >= Critical_Delta_S5) {
					btmpresult = false;
				} else {
					if (focal_abs(Sort5Max) > focal_abs(Sort5Min))
						Num = Sort5Max_ChNum;
					else
						Num = Sort5Min_ChNum;
					/*SetCriticalMsg(Num);*/
					bCriticalResult = true;
				}
				FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Critical Deviation of Sort5: %d",
							 Sort5Max, Sort5Min, Sort5Max - Sort5Min, Critical_Delta_S5);

			} else {
				btmpresult = false;
				FTS_TEST_ERROR("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort5: %d",
							   Sort5Max, Sort5Min, Sort5Max - Sort5Min, Delta_Ci_Differ_S5);

			}
		} else {
			FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort5: %d",
						 Sort5Max, Sort5Min, Sort5Max - Sort5Min, Delta_Ci_Differ_S5);

		}
		FTS_TEST_DBG("Sort5: %d, ", Sort5Max - Sort5Min);
	}
	if (bUseSort6) {
		if (Delta_Ci_Differ_S6 <= Sort6Max - Sort6Min) {
			if (bUseCriticalValue) {
				if (Sort6Max - Sort6Min >= Critical_Delta_S6) {
					btmpresult = false;
				} else {
					if (focal_abs(Sort6Max) > focal_abs(Sort6Min))
						Num = Sort6Max_ChNum;
					else
						Num = Sort6Min_ChNum;
					/*SetCriticalMsg(Num);*/
					bCriticalResult = true;
				}
				FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Critical Deviation of Sort6: %d",
							 Sort6Max, Sort6Min, Sort6Max - Sort6Min, Critical_Delta_S6);
			} else {
				btmpresult = false;
				FTS_TEST_ERROR("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort6: %d",
							   Sort6Max, Sort6Min, Sort6Max - Sort6Min, Delta_Ci_Differ_S6);

			}
		} else {
			FTS_TEST_DBG("Condition1: Max of Delta CB_Differ: %d, Min of Delta CB_Differ: %d, Get Deviation: %d, Set Deviation of Sort6: %d",
						 Sort6Max, Sort6Min, Sort6Max - Sort6Min, Delta_Ci_Differ_S6);

		}
		FTS_TEST_DBG("Sort6: %d, ", Sort6Max - Sort6Min);
	}

	/*Max Delta_Ci cannot exceed default values*/

	Delta_Min = Delta_Max = focal_abs(m_DeltaCb_DifferData[0]);
	for (i = 1; i < g_ScreenSetParam.iChannelsNum/*readlen*/; i++) {
		if (focal_abs(m_DeltaCb_DifferData[i]) < Delta_Min) {
			Delta_Min = focal_abs(m_DeltaCb_DifferData[i]);
		}
		if (focal_abs(m_DeltaCb_DifferData[i]) > Delta_Max) {
			Delta_Max = focal_abs(m_DeltaCb_DifferData[i]);
		}
	}

	set_Delta_Cb_Max = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Differ_Max;
	if (set_Delta_Cb_Max < focal_abs(Delta_Max)) {
		btmpresult = false;
	}
	FTS_TEST_DBG("Condition2: Get Max Differ Data of Delta_CB(abs): %d, Set Max Differ Data of Delta_CB(abs): %d", Delta_Max, set_Delta_Cb_Max);


	if (g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Include_Key_Test) {

		SetKeyMax = g_stCfg_FT6X36_BasicThreshold.DeltaCbTest_Key_Differ_Max;

		Key_Delta_Max = focal_abs(m_DeltaCb_DifferData[g_ScreenSetParam.iChannelsNum]);
		for (i = g_ScreenSetParam.iChannelsNum; i < g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum; i++) {
			if (focal_abs(m_DeltaCb_DifferData[i]) > Key_Delta_Max) {
				Key_Delta_Max = focal_abs(m_DeltaCb_DifferData[i]);
			}
		}
		if (SetKeyMax <= Key_Delta_Max) {
			btmpresult = false;
		}
		FTS_TEST_DBG("Condition3: Include Key Test, Get Max Key Data: %d, Set Max Key Data: %d", Key_Delta_Max, SetKeyMax);
	}

	FTS_TEST_DBG("Max Differ Data of Delta_CB(abs): %d ", Delta_Max);

	if (bCriticalResult && btmpresult) {
		FTS_TEST_DBG("Delta CB Test has Critical Result(TBD)!");
	}
	/*Delta Ci End*/
	if (btmpresult) {
		FTS_TEST_INFO("Delta CB Test is OK!\r");

		if (bCriticalResult)
			* bTestResult = 2;
		else
			* bTestResult = 1;
	} else {
		FTS_TEST_INFO("Delta CB Test is NG!\r");
		* bTestResult = 0;
	}
	TestResultLen += sprintf(TestResult + TestResultLen, " Delta CB Test is %s. \n\n", (btmpresult ? "OK" : "NG"));
	/*Collect test data and save CSV file.*/

	Save_Test_Data(m_DeltaCbData, 0, 1, g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum, 1);

	/*Collect test data and save CSV file.*/

	Save_Test_Data(m_DeltaCb_DifferData, 0, 1, g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum, 2);

	/*GetDeltaCiDataMsg();*/
	return 0;
}

/************************************************************************
* Name: FT6X36_get_test_data
* Brief:  get data of test result
* Input: none
* Output: pTestData, the returned buff
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int FT6X36_get_test_data(char *pTestData)
{
	if (NULL == pTestData) {
		FTS_TEST_ERROR("pTestData == NULL ");
		return -1;
	}
	memcpy(pTestData, g_pStoreAllData, (g_lenStoreMsgArea + g_lenStoreDataArea));
	return (g_lenStoreMsgArea + g_lenStoreDataArea);
}



/************************************************************************
* Name: GetPanelChannels(Same function name as FT_MultipleTest GetChannelNum)
* Brief:  Get row of TP
* Input: none
* Output: pPanelChannels
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelChannels(unsigned char *pPanelChannels)
{
	return ReadReg(C6X36_CHANNEL_NUM, pPanelChannels);
}

/************************************************************************
* Name: GetPanelKeys(Same function name as FT_MultipleTest GetKeyNum)
* Brief:  get column of TP
* Input: none
* Output: pPanelKeys
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelKeys(unsigned char *pPanelKeys)
{
	return ReadReg(C6X36_KEY_NUM, pPanelKeys);
}
/************************************************************************
* Name: StartScan(Same function name as FT_MultipleTest)
* Brief:  Scan TP, do it before read Raw Data
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int StartScan(void)
{
	unsigned char RegVal = 0x01;
	unsigned int times = 0;
	const unsigned int MaxTimes = 500/*20*/;     /* The longest wait 160ms*/
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(C6208_SCAN_ADDR, &RegVal);
	if (ReCode == ERROR_CODE_OK) {
		RegVal = 0x01;      /*Top bit position 1, start scan*/
		ReCode = WriteReg(C6208_SCAN_ADDR, RegVal);
		if (ReCode == ERROR_CODE_OK) {
			while (times++ < MaxTimes) {    /*Wait for the scan to complete*/
				SysDelay(8);    /*8ms*/
				ReCode = ReadReg(C6208_SCAN_ADDR, &RegVal);
				if (ReCode == ERROR_CODE_OK) {
					if (RegVal == 0) {
						/*ReCode == WriteReg(0x01, 0x00);*/
						break;
					}
				} else {
					break;
				}
			}
			if (times < MaxTimes)    ReCode = ERROR_CODE_OK;
			else ReCode = ERROR_CODE_COMM_ERROR;
		}
	}

	return ReCode;

}
/************************************************************************
* Name: ReadRawData(Same function name as FT_MultipleTest)
* Brief:  read Raw Data
* Input: Freq(No longer used, reserved), LineNum, ByteNum
* Output: pRevBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3];
	unsigned short BytesNumInTestMode1 = 0;

	int i = 0;

	BytesNumInTestMode1 = ByteNum;

	/*Read raw data in test mode1*/
	I2C_wBuffer[0] = C6X36_RAWDATA_ADDR;/*Rawdata start addr register;*/
	I2C_wBuffer[1] = 0;/*start index*/
	ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 2, NULL, 0);/*   set rawdata start addr*/

	if ((ReCode == ERROR_CODE_OK)) {
		if (ReCode == ERROR_CODE_OK) {
			I2C_wBuffer[0] = C6X36_RAWDATA_BUF; /*rawdata buffer addr register;*/

			ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, m_ucTempData, BytesNumInTestMode1);

		}
	}


	if (ReCode == ERROR_CODE_OK) {
		for (i = 0; i < (ByteNum >> 1); i++) {
			pRevBuffer[i] = (m_ucTempData[i << 1] << 8) + m_ucTempData[(i << 1) + 1];
		}
	}

	return ReCode;

}


/************************************************************************
* Name: Save_Test_Data
* Brief:  Storage format of test data
* Input: int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount
* Output: none
* Return: none
***********************************************************************/
static void Save_Test_Data(int iData[MAX_SCAP_CHANNEL_NUM], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount)
{
	int iLen = 0;
	int i = 0, j = 0;

	/*Save  Msg (ItemCode is enough, ItemName is not necessary, so set it to "NA".)*/
	iLen = sprintf(g_pTmpBuff, "NA, %d, %d, %d, %d, %d, ", \
				   m_ucTestItemCode, Row, Col, m_iStartLine, ItemCount);
	memcpy(g_pMsgAreaLine2 + g_lenMsgAreaLine2, g_pTmpBuff, iLen);
	g_lenMsgAreaLine2 += iLen;

	m_iStartLine += Row;
	m_iTestDataCount++;

	/*Save Data*/
	for (i = 0 + iArrayIndex; i < Row + iArrayIndex; i++) {
		for (j = 0; j < Col; j++) {
			if (j == (Col - 1)) /*The Last Data of the Row, add "\n"*/
				iLen = sprintf(g_pTmpBuff, "%d, \n",  iData[j]);
			else
				iLen = sprintf(g_pTmpBuff, "%d, ", iData[j]);

			memcpy(g_pStoreDataArea + g_lenStoreDataArea, g_pTmpBuff, iLen);
			g_lenStoreDataArea += iLen;
		}
	}

}

/************************************************************************
* Name: GetChannelNum
* Brief:  Get Channel Num(Tx and Rx)
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetChannelNum(void)
{
	unsigned char ReCode;
	unsigned char rBuffer[1]; /*new unsigned char;*/

	/*m_strCurrentTestMsg = "Get Tx Num...";*/
	ReCode = GetPanelChannels(rBuffer);
	if (ReCode == ERROR_CODE_OK) {
		g_ScreenSetParam.iChannelsNum = rBuffer[0];
	} else {
		FTS_TEST_ERROR("Failed to get channel number");
		g_ScreenSetParam.iChannelsNum = 0;
	}

	/*m_strCurrentTestMsg = "Get Rx Num...";*/

	ReCode = GetPanelKeys(rBuffer);
	if (ReCode == ERROR_CODE_OK) {
		g_ScreenSetParam.iKeyNum = rBuffer[0];
	} else {
		g_ScreenSetParam.iKeyNum = 0;
		FTS_TEST_ERROR("Failed to get Rx number");
	}

	return ReCode;

}

/************************************************************************
* Name: GetRawData
* Brief:  get panel rawdata by read rawdata function
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char GetRawData(void)
{
	int readlen = 0;
	unsigned char ReCode = ERROR_CODE_OK;

	/*Enter Factory Mode*/
	ReCode = EnterFactory();
	if (ERROR_CODE_OK != ReCode) {
		FTS_TEST_ERROR("Failed to Enter Factory Mode...");
		return ReCode;
	}

	/*--------------------------------------------Check Num of Channel*/
	if (0 == (g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum)) {
		ReCode = GetChannelNum();
		if (ERROR_CODE_OK != ReCode) {
			FTS_TEST_ERROR("Error Channel Num...");
			return ERROR_CODE_INVALID_PARAM;
		}
	}

	/*Start Scanning*/
	FTS_TEST_DBG("Start Scan ...");
	ReCode = StartScan();
	if (ERROR_CODE_OK != ReCode) {
		FTS_TEST_ERROR("Failed to Scan ...");
		return ReCode;
	}

	memset(m_RawData, 0, sizeof(m_RawData));

	/*Read RawData*/
	readlen = g_ScreenSetParam.iChannelsNum + g_ScreenSetParam.iKeyNum;
	if (readlen <= 0 || readlen >= MAX_SCAP_CHANNEL_NUM) return ERROR_CODE_INVALID_PARAM;


	ReCode = ReadRawData(3, 0, readlen * 2, m_RawData);
	if (ReCode != ERROR_CODE_OK) {
		FTS_TEST_ERROR("Failed to Read RawData...");
		return ReCode;
	}

	return ReCode;

}

/************************************************************************
* Name: ShowRawData
* Brief:  Show RawData
* Input: none
* Output: none
* Return: none.
***********************************************************************/
static void ShowRawData(void)
{
	int iChannelsNum = 0, iKeyNum = 0;

	/*--Show RawData*/
	FTS_TEST_DBG("\nChannels:  ");
	for (iChannelsNum = 0; iChannelsNum < g_ScreenSetParam.iChannelsNum; iChannelsNum++) {
		FTS_TEST_DBG("%5d    ", m_RawData[iChannelsNum]);
	}

	FTS_TEST_DBG("\nKeys:  ");
	for (iKeyNum = 0; iKeyNum < g_ScreenSetParam.iKeyNum; iKeyNum++) {
		FTS_TEST_DBG("%5d    ", m_RawData[g_ScreenSetParam.iChannelsNum + iKeyNum]);
	}

}

#endif

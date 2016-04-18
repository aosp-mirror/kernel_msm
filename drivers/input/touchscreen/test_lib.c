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
#include <linux/time.h>
#include <linux/slab.h>

#include "test_lib.h"
#include "Global.h"

#include "Config_FT6X36.h"
#include "Test_FT6X36.h"

#include "ini.h"

#define FTS_DRIVER_LIB_INFO  "Test_Lib_Version  V1.3.0 2015-10-10"

FTS_I2C_READ_FUNCTION fts_i2c_read_test;
FTS_I2C_WRITE_FUNCTION fts_i2c_write_test;

char *g_testparamstring = NULL;

int init_i2c_read_func(FTS_I2C_READ_FUNCTION fpI2C_Read)
{
	fts_i2c_read_test = fpI2C_Read;
	return 0;
}

int init_i2c_write_func(FTS_I2C_WRITE_FUNCTION fpI2C_Write)
{
	fts_i2c_write_test = fpI2C_Write;
	return 0;
}

/************************************************************************
* Name: set_param_data
* Brief:  load Config. Set IC series, init test items,
* init basic threshold, int detailThreshold, and set order of test items
* Input: TestParamData, from ini file.
* Output: none
* Return: 0. No sense, just according to the old format.
***********************************************************************/
int set_param_data(char *TestParamData)
{
	pr_info("[fts]Enter set_param_data\n");
	g_testparamstring = TestParamData;	/* get param of ini file */
	ini_get_key_data(g_testparamstring);	/* get param to struct */

	/* Set g_ScreenSetParam.iSelectedIC */
	OnInit_InterfaceCfg(g_testparamstring);

	/* Get IC Name */
	get_ic_name(g_ScreenSetParam.iSelectedIC, g_strIcName);

	if (IC_FT6X36 >> 4 == g_ScreenSetParam.iSelectedIC >> 4) {
		OnInit_FT6X36_TestItem(g_testparamstring);
		OnInit_FT6X36_BasicThreshold(g_testparamstring);
		OnInit_SCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT6X36();
	} else {
		pr_info("[fts]%s The IC type error in this testing\n", __func__);
		return -EIO;
	}

	return 0;
}

/************************************************************************
* Name: free_test_param_data
* Brief:  release printer memory
* Input: none
* Output: none
* Return: none.
***********************************************************************/
void free_test_param_data(void)
{
	if (!g_testparamstring)
		kfree(g_testparamstring);

	g_testparamstring = NULL;
}

/************************************************************************
* Name: show_lib_ver
* Brief:  get lib version
* Input: none
* Output: pLibVer
* Return: the length of lib version.
***********************************************************************/
int show_lib_ver(char *pLibVer)
{
	int num_read_chars = 0;

	num_read_chars = snprintf(pLibVer, 128, "%s\n", FTS_DRIVER_LIB_INFO);

	return num_read_chars;
}



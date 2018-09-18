/*
  *
  **************************************************************************
  **                        STMicroelectronics				 **
  **************************************************************************
  **                        marco.cali@st.com				  **
  **************************************************************************
  *                                                                        *
  *			FTS API for MP test				*
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsTest.c
  * \brief Contains all the functions related to the Mass Production Test
  */

#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsFrame.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"
#include "../fts.h"


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/firmware.h>


#ifdef LIMITS_H_FILE
#include "../fts_limits.h"
#endif


TestToDo tests;	/* /< global variable that specify the tests to perform during
		  * the Mass Production Test */
static LimitFile limit_file;	/* /< variable which contains the limit file
				 * during test */

/**
  * Initialize the testToDo variable with the default tests to perform during
  * the Mass Production Test
  * @return OK
  */
int initTestToDo(void)
{
	/*** Initialize Limit File ***/
	limit_file.size = 0;
	limit_file.data = NULL;
	strlcpy(limit_file.name, " ", MAX_LIMIT_FILE_NAME);


	tests.MutualRawAdjITO = 1;

	tests.MutualRaw = 1;
	tests.MutualRawGap = 0;
	tests.MutualRawAdj = 0;

	tests.MutualRawLP = 1;
	tests.MutualRawGapLP = 0;
	tests.MutualRawAdjLP = 0;

	tests.MutualCx1 = 0;
	tests.MutualCx2 = 0;
	tests.MutualCx2Adj = 0;
	tests.MutualCxTotal = 0;
	tests.MutualCxTotalAdj = 0;

#ifdef PHONE_KEY
	tests.MutualKeyRaw = 1;
#else
	tests.MutualKeyRaw = 0;
#endif
	tests.MutualKeyCx1 = 0;
	tests.MutualKeyCx2 = 0;
#ifdef PHONE_KEY
	tests.MutualKeyCxTotal = 0;
#else
	tests.MutualKeyCxTotal = 0;
#endif

	tests.SelfForceRaw = 1;
	tests.SelfForceRawGap = 0;

	tests.SelfForceRawLP = 1;
	tests.SelfForceRawGapLP = 0;

	tests.SelfForceIx1 = 0;
	tests.SelfForceIx2 = 0;
	tests.SelfForceIx2Adj = 0;
	tests.SelfForceIxTotal = 0;
	tests.SelfForceIxTotalAdj = 0;
	tests.SelfForceCx1 = 0;
	tests.SelfForceCx2 = 0;
	tests.SelfForceCx2Adj = 0;
	tests.SelfForceCxTotal = 0;
	tests.SelfForceCxTotalAdj = 0;

	tests.SelfSenseRaw = 1;
	tests.SelfSenseRawGap = 0;

	tests.SelfSenseRawLP = 1;
	tests.SelfSenseRawGapLP = 0;

	tests.SelfSenseIx1 = 0;
	tests.SelfSenseIx2 = 0;
	tests.SelfSenseIx2Adj = 0;
	tests.SelfSenseIxTotal = 0;
	tests.SelfSenseIxTotalAdj = 0;
	tests.SelfSenseCx1 = 0;
	tests.SelfSenseCx2 = 0;
	tests.SelfSenseCx2Adj = 0;
	tests.SelfSenseCxTotal = 0;
	tests.SelfSenseCxTotalAdj = 0;

	return OK;
}

/**
  * Compute the Horizontal adjacent matrix doing the abs of the difference
  * between the column i with the i-1 one. \n
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHoriz(i8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		pr_err("computeAdjHoriz: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjHoriz: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Horizontal adjacent matrix of short values doing the abs of
  * the difference between the column i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  *  array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which
  * will contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizTotal(short *data, int row, int column, u16 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		pr_err("computeAdjHorizTotal: ERROR %08X\n",
			ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjHorizTotal: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix doing the abs of the difference between
  * the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVert(i8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		pr_err("computeAdjVert: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjVert: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix of short values doing the abs of
  * the difference between the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertTotal(short *data, int row, int column, u16 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		pr_err("computeAdjVertTotal: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjVertTotal: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}

/**
  * Compute the Horizontal adjacent matrix doing the abs of the difference
  * between the column i with the i-1 one. \n
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  *  contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizFromU(u8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		pr_err("computeAdjHoriz: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjHoriz: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Horizontal adjacent matrix of u16 values doing the abs of
  * the difference between the column i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizTotalFromU(u16 *data, int row, int column, u16 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		pr_err("computeAdjHorizTotal: ERROR %08X\n",
			ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjHorizTotal: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++)
		for (j = 1; j < column; j++)
			*(*result + (i * (column - 1) + (j - 1))) =
				abs(data[i * column + j] - data[i * column +
								(j - 1)]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix doing the abs of the difference between
  * the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertFromU(u8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		pr_err("computeAdjVert: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjVert: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}

/**
  * Compute the Vertical adjacent matrix of u16 values doing the abs of
  * the difference between the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertTotalFromU(u16 *data, int row, int column, u16 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		pr_err("computeAdjVertTotal: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *)kmalloc(size * sizeof(u16), GFP_KERNEL);
	if (*result == NULL) {
		pr_err("computeAdjVertTotal: ERROR %08X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++)
		for (j = 0; j < column; j++)
			*(*result + ((i - 1) * column + j)) =
				abs(data[i * column + j] - data[(i - 1) *
								column + j]);

	return OK;
}



/**
  * Check that each value of a matrix of short doesn't exceed a min and a Max
  * value
  * (these values are included in the interval). \n
  * The matrix is stored as 1 dimension array one row after the other. \n
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min minimum value allowed
  * @param max Maximum value allowed
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMinMax(short *data, int row, int column, int min, int max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min || data[i * column + j] >
			    max) {
				pr_err("checkLimitsMinMax: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					i, j, data[i * column + j], min, max);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that the difference between the max and min of a matrix of short
  * is less or equal to a threshold.\n
  * The matrix is stored as 1 dimension array one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param threshold threshold value allowed
  * @return OK if the difference is <= to threshold otherwise
  * ERROR_TEST_CHECK_FAIL
  */
int checkLimitsGap(short *data, int row, int column, int threshold)
{
	int i, j;
	int min_node;
	int max_node;

	if (row == 0 || column == 0) {
		pr_err("checkLimitsGap: invalid number of rows = %d or columns = %d  ERROR %08X\n",
			row, column, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	min_node = data[0];
	max_node = data[0];

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min_node)
				min_node = data[i * column + j];
			else if (data[i * column + j] > max_node)
				max_node = data[i * column + j];
		}
	}

	if (max_node - min_node > threshold) {
		pr_err("checkLimitsGap: GAP = %d exceed limit  %d\n",
			max_node - min_node, threshold);
		return ERROR_TEST_CHECK_FAIL;
	} else
		return OK;
}

/**
  * Check that each value of a matrix of i8 doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval). \n
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMap(i8 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				pr_err("checkLimitsMap: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					i, j, data[i * column + j],
					min[i * column + j],
					max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of short doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval).
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapTotal(short *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				pr_err("checkLimitsMapTotal: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					i, j, data[i * column + j],
					min[i * column + j],
					max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u8 doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval). \n
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapFromU(u8 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				pr_err("checkLimitsMap: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					i, j, data[i * column + j],
					min[i * column + j],
					max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u16 doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval).
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapTotalFromU(u16 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] ||
			    data[i * column + j] > max[i * column + j]) {
				pr_err("checkLimitsMapTotal: Node[%d,%d] = %d exceed limit [%d, %d]\n",
					i, j, data[i * column + j],
					min[i * column + j],
					max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u8 doesn't exceed a specific Max value
  * set for each node (max value is included in the interval).
  * The matrixes of data and max values are stored as 1 dimension arrays one row
  * after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapAdj(u8 *data, int row, int column, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] > max[i * column + j]) {
				pr_err("checkLimitsMapAdj: Node[%d,%d] = %d exceed limit > %d\n",
					i, j,
					data[i * column + j],
					max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u16 doesn't exceed a specific Max value
  * set for each node (max value is included in the interval).
  * The matrixes of data and max values are stored as 1 dimension arrays one row
  * after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapAdjTotal(u16 *data, int row, int column, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] > max[i * column + j]) {
				pr_err("checkLimitsMapAdjTotal: Node[%d,%d] = %d exceed limit > %d\n",
					i, j,
					data[i * column + j],
					max[i * column + j]);
				count++;
			}
		}
	}

	return count;	/* if count is 0 = OK, test completed successfully */
}

/**
  * Perform an ITO test setting all the possible options
  * (see @link ito_opt ITO Options @endlink) and checking MS Raw ADJ if enabled
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param todo pointer to a TestToDo variable which select the test to do
  * @param frame pointer to a MutualSenseframe variable which will store the ITO
  *  MS RAW data. If NULL, no data will be returned while if MutualRawAdjITO
  *  item in todo is disabled the variable will be untouched
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ito(char *path_limits, TestToDo *todo,
			MutualSenseFrame *frame)
{
	int res = OK;
	u8 sett[2] = { 0x00, 0x00 };
	MutualSenseFrame msRawFrame;
	MutualSenseFrame *ptr_frame = NULL;
	int *thresholds = NULL;
	u16 *adj = NULL;
	int trows, tcolumns;

	msRawFrame.node_data = NULL;

	pr_info("ITO Production test is starting...\n");

	res = fts_system_reset();
	if (res < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_PROD_TEST_ITO);
		return res | ERROR_PROD_TEST_ITO;
	}

	sett[0] = SPECIAL_TUNING_IOFF;
	pr_info("Trimming Ioff...\n");
	res = writeSysCmd(SYS_CMD_SPECIAL_TUNING, sett, 2);
	if (res < OK) {
		pr_err("production_test_ito: Trimm Ioff ERROR %08X\n",
			(res | ERROR_PROD_TEST_ITO));
		return res | ERROR_PROD_TEST_ITO;
	}

	sett[0] = 0xFF;
	sett[1] = 0xFF;
	pr_info("ITO Check command sent...\n");
	res = writeSysCmd(SYS_CMD_ITO, sett, 2);
	if (res < OK) {
		pr_err("production_test_ito: ERROR %08X\n",
			 (res | ERROR_PROD_TEST_ITO));
		return res | ERROR_PROD_TEST_ITO;
	}

	pr_info("ITO Command = OK!\n");

	pr_info("MS RAW ITO ADJ TEST:\n");
	if (todo->MutualRawAdjITO == 1) {
		pr_info("Collecting MS Raw data...\n");

		if (frame != NULL) {
			pr_info("%s: Copying MS Raw data to caller!\n",
				__func__);
			ptr_frame  = frame;
		} else
			ptr_frame = &msRawFrame;

		res |= getMSFrame3(MS_RAW, ptr_frame);
		if (res < OK) {
			pr_err("%s: getMSFrame failed... ERROR %08X\n",
				__func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}

		print_frame_short("MS Raw ITO frame =",
				  array1dTo2d_short(
					  (*ptr_frame).node_data,
					  (*ptr_frame).node_data_size,
					  (*ptr_frame).header.sense_node),
				  (*ptr_frame).header.force_node,
				  (*ptr_frame).header.sense_node);

		pr_info("MS RAW ITO ADJ HORIZONTAL TEST:\n");
		res = computeAdjHorizTotal((*ptr_frame).node_data,
					   (*ptr_frame).header.force_node,
					   (*ptr_frame).header.sense_node,
					   &adj);
		if (res < OK) {
			pr_err("%s: computeAdjHoriz failed... ERROR %08X\n",
				__func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}

		res = parseProductionTestLimits(path_limits, &limit_file,
						MS_RAW_ITO_ADJH, &thresholds,
						&trows, &tcolumns);
		if (res < OK || (trows != (*ptr_frame).header.force_node ||
				 tcolumns != (*ptr_frame).header.sense_node -
				 1)) {
			pr_err("%s: parseProductionTestLimits MS_RAW_ITO_ADJH failed... ERROR %08X\n",
				__func__, ERROR_PROD_TEST_DATA);
			goto ERROR;
		}


		res = checkLimitsMapAdjTotal(adj,
					     (*ptr_frame).header.force_node,
					     (*ptr_frame).header.sense_node - 1,
					     thresholds);
		if (res != OK) {
			pr_err("production_test_data: checkLimitsAdj MS RAW ITO ADJH failed... ERROR COUNT = %d\n",
				res);
			pr_err("MS RAW ITO ADJ HORIZONTAL TEST:.................FAIL\n\n");
			res = ERROR_PROD_TEST_ITO;
			goto ERROR;
		} else
			pr_info("MS RAW ITO ADJ HORIZONTAL TEST:.................OK\n");

		kfree(thresholds);
		thresholds = NULL;

		kfree(adj);
		adj = NULL;

		pr_info("MS RAW ITO ADJ VERTICAL TEST:\n");
		res = computeAdjVertTotal((*ptr_frame).node_data,
					  (*ptr_frame).header.force_node,
					  (*ptr_frame).header.sense_node,
					  &adj);
		if (res < OK) {
			pr_err("%s: computeAdjVert failed... ERROR %08X\n",
				__func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}

		res = parseProductionTestLimits(path_limits, &limit_file,
						MS_RAW_ITO_ADJV, &thresholds,
						&trows, &tcolumns);
		if (res < OK || (trows != (*ptr_frame).header.force_node - 1 ||
				 tcolumns != (*ptr_frame).header.sense_node)) {
			pr_err("%s: parseProductionTestLimits MS_RAW_ITO_ADJV failed... ERROR %08X\n",
				__func__, ERROR_PROD_TEST_ITO);
			goto ERROR;
		}


		res = checkLimitsMapAdjTotal(adj, (*ptr_frame).header.force_node
					- 1, (*ptr_frame).header.sense_node,
					     thresholds);
		if (res != OK) {
			pr_err("%s: checkLimitsAdj MS RAW ITO ADJV failed... ERROR COUNT = %d\n",
				__func__, res);
			pr_err("MS RAW ITO ADJ VERTICAL TEST:.................FAIL\n\n");
			res = ERROR_PROD_TEST_ITO;
			goto ERROR;
		} else
			pr_info("MS RAW ITO ADJ VERTICAL TEST:.................OK\n");

		kfree(thresholds);
		thresholds = NULL;

		kfree(adj);
		adj = NULL;
	} else
		pr_info("MS RAW ITO ADJ TEST:.................SKIPPED\n");

ERROR:
	if (thresholds != NULL)
		kfree(thresholds);
	if (adj != NULL)
		kfree(adj);
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	freeLimitsFile(&limit_file);
	res |= fts_system_reset();
	if (res < OK) {
		pr_err("production_test_ito: ERROR %08X\n",
			ERROR_PROD_TEST_ITO);
		res = (res | ERROR_PROD_TEST_ITO);
	}
	return res;
}

/**
  * Perform the Initialization of the IC
  * @param type type of initialization to do
  * (see @link sys_special_opt Initialization Options (Full or Panel) @endlink)
  * @return OK if success or an error code which specify the type of error
  */
int production_test_initialization(u8 type)
{
	int res;

	pr_info("INITIALIZATION Production test is starting...\n");
	if (type != SPECIAL_PANEL_INIT && type != SPECIAL_FULL_PANEL_INIT) {
		pr_err("production_test_initialization: Type incompatible! Type = %02X ERROR %08X\n",
			type, ERROR_OP_NOT_ALLOW |
			ERROR_PROD_TEST_INITIALIZATION);
		return ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_INITIALIZATION;
	}

	res = fts_system_reset();
	if (res < 0) {
		pr_err("production_test_initialization: ERROR %08X\n",
			ERROR_PROD_TEST_INITIALIZATION);
		return res | ERROR_PROD_TEST_INITIALIZATION;
	}

	pr_info("INITIALIZATION command sent... %02X\n", type);
	res = writeSysCmd(SYS_CMD_SPECIAL, &type, 1);
	if (res < OK) {
		pr_err("production_test_initialization: ERROR %08X\n",
			(res | ERROR_PROD_TEST_INITIALIZATION));
		return res | ERROR_PROD_TEST_INITIALIZATION;
	}


	pr_info("Refresh Sys Info...\n");
	res |= readSysInfo(1);	/* need to update the chipInfo in order
				  * to refresh several versions */

	if (res < 0) {
		pr_err("production_test_initialization: read sys info ERROR %08X\n",
			ERROR_PROD_TEST_INITIALIZATION);
		res = (res | ERROR_PROD_TEST_INITIALIZATION);
	}

	return res;
}


//  @param signature value of the MP flag to save if the Mass Production Test succeed
/**
  * Perform a FULL (ITO + INIT + DATA CHECK) Mass Production Test of the IC
  * @param pathThresholds name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param saveInit if >0 (possible values: NO_INIT, SPECIAL_PANEL_INIT or
  * SPECIAL_FULL_PANEL_INIT),
  * the Initialization of the IC is executed otherwise it is skipped
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_main(char *pathThresholds, int stop_on_fail, int saveInit,
			 TestToDo *todo)
{
	int res, ret;

	pr_info("MAIN Production test is starting...\n");

	pr_info("ITO TEST:\n");
	res = production_test_ito(pathThresholds, todo, NULL);
	if (res < 0) {
		pr_err("Error during ITO TEST! ERROR %08X\n", res);
		goto END;/* in case of ITO TEST failure is no sense keep going
			 * */
	} else
		pr_info("ITO TEST OK!\n");

	pr_info("INITIALIZATION TEST :\n");
	if (saveInit != NO_INIT) {
		res = production_test_initialization((u8)saveInit);
		if (res < 0) {
			pr_err("Error during  INITIALIZATION TEST! ERROR %08X\n",
				res);
			if (stop_on_fail)
				goto END;
		} else
			pr_info("INITIALIZATION TEST OK!\n");
	} else
		pr_info("INITIALIZATION TEST :................. SKIPPED\n");

	if (saveInit == 1) {
		pr_info("Cleaning up...\n");
		ret = fts_system_reset();
		if (ret < 0) {
			pr_err("production_test_main: system reset ERROR %08X\n",
				ret);
			res |= ret;
			if (stop_on_fail)
				goto END;
		}
	}

	pr_info("PRODUCTION DATA TEST:\n");
	ret = production_test_data(pathThresholds, stop_on_fail, todo);
	if (ret < 0)
		pr_err("Error during PRODUCTION DATA TEST! ERROR %08X\n", ret);
	else
		pr_info("PRODUCTION DATA TEST OK!\n");

	res |= ret;
	/* the OR is important because if the data test is OK but
	 * the init test fail, the main production test result should = FAIL */

END:
	if (res < 0) {
		pr_err("MAIN Production test finished.................FAILED\n");
		return res;
	} else {
		pr_info("MAIN Production test finished.................OK\n");
		return OK;
	}
}

/**
  * Perform all the test selected in a TestTodo variable related to MS raw data
  * (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_raw(char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret, count_fail = 0;
	MutualSenseFrame msRawFrame;


	int *thresholds = NULL;
	int trows, tcolumns;

	u16 *adj = NULL;

	msRawFrame.node_data = NULL;

	/************** Mutual Sense Test *************/
	pr_info("MS RAW DATA TEST is starting...\n");
	if (todo->MutualRaw == 1 || todo->MutualRawGap == 1 ||
	    todo->MutualRawAdj == 1) {
		ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
		mdelay(WAIT_FOR_FRESH_FRAMES);
		ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
		mdelay(WAIT_AFTER_SENSEOFF);
		ret |= getMSFrame3(MS_RAW, &msRawFrame);
		if (ret < OK) {
			pr_err("production_test_data: getMSFrame failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			return ret | ERROR_PROD_TEST_DATA;
		}

		print_frame_short("MS Raw frame =",
				  array1dTo2d_short(
					  msRawFrame.node_data,
					  msRawFrame.node_data_size,
					  msRawFrame.header.sense_node),
				  msRawFrame.header.force_node,
				  msRawFrame.header.sense_node);

		pr_info("MS RAW MIN MAX TEST:\n");
		if (todo->MutualRaw == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_RAW_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 2)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_MIN_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMinMax(msRawFrame.node_data,
						msRawFrame.header.force_node,
						msRawFrame.header.sense_node,
						thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMinMax MS RAW failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS RAW MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW MIN MAX TEST:.................OK\n");
			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("MS RAW MIN MAX TEST:.................SKIPPED\n");

		pr_info("MS RAW GAP TEST:\n");
		if (todo->MutualRawGap == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file, MS_RAW_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_GAP failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap(msRawFrame.node_data,
					     msRawFrame.header.force_node,
					     msRawFrame.header.sense_node,
					     thresholds[0]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsGap MS RAW failed... ERROR = %08X\n",
					ret);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW GAP TEST:.................OK\n\n");
			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("MS RAW GAP TEST:.................SKIPPED\n");

		pr_info("MS RAW ADJ TEST:\n");
		if (todo->MutualRawAdj == 1) {
			pr_info("MS RAW ADJ HORIZONTAL TEST:\n");
			ret = computeAdjHorizTotal(msRawFrame.node_data,
						   msRawFrame.header.force_node,
						   msRawFrame.header.sense_node,
						   &adj);
			if (ret < OK) {
				pr_err("production_test_data: computeAdjHoriz failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_RAW_ADJH,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows !=
					 msRawFrame.header.force_node ||
					 tcolumns !=
					 msRawFrame.header.sense_node - 1)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_ADJH failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMapAdjTotal(adj,
						     msRawFrame.header.
						     force_node,
						     msRawFrame.header.
						     sense_node - 1,
						     thresholds);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsAdj MS RAW ADJH failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS RAW ADJ HORIZONTAL TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW ADJ HORIZONTAL TEST:.................OK\n");

			kfree(thresholds);
			thresholds = NULL;

			kfree(adj);
			adj = NULL;

			pr_info("MS RAW ADJ VERTICAL TEST:\n");
			ret = computeAdjVertTotal(msRawFrame.node_data,
						  msRawFrame.header.force_node,
						  msRawFrame.header.sense_node,
						  &adj);
			if (ret < OK) {
				pr_err("production_test_data: computeAdjVert failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_RAW_ADJV,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != msRawFrame.header.force_node -
					 1 || tcolumns !=
					 msRawFrame.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_ADJV failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMapAdjTotal(adj,
						     msRawFrame.header.
						     force_node - 1,
						     msRawFrame.header.
						     sense_node, thresholds);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsAdj MS RAW ADJV failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS RAW ADJ VERTICAL TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW ADJ VERTICAL TEST:.................OK\n");
			kfree(thresholds);
			thresholds = NULL;

			kfree(adj);
			adj = NULL;
		} else
			pr_info("MS RAW ADJ TEST:.................SKIPPED\n");
	} else
		pr_info("MS RAW FRAME TEST:.................SKIPPED\n");

	pr_info("MS KEY RAW TEST:\n");
	if (todo->MutualKeyRaw == 1) {
		ret = production_test_ms_key_raw(path_limits);
		if (ret < 0) {
			pr_err("production_test_data: production_test_ms_key_raw failed... ERROR = %08X\n",
				ret);
			count_fail += 1;
			if (count_fail == 1) {
				pr_err("MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",
					count_fail);
				goto ERROR_LIMITS;
			}
		}
	} else
		pr_info("MS KEY RAW TEST:.................SKIPPED\n");

	ret = production_test_ms_raw_lp(path_limits, stop_on_fail, todo);
	if (ret < 0) {
		pr_err("production_test_data: production_test_ms_raw_lp failed... ERROR = %08X\n",
			ret);
		count_fail += 1;
		if (count_fail == 1) {
			pr_err("MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",
				count_fail);
			goto ERROR_LIMITS;
		}
	}

ERROR:

	if (count_fail == 0) {
		if (msRawFrame.node_data != NULL) {
			kfree(msRawFrame.node_data);
			msRawFrame.node_data = NULL;
		}
		pr_info("MS RAW DATA TEST finished!.................OK\n");
		return OK;
	} else {
		if (msRawFrame.node_data != NULL)
			kfree(msRawFrame.node_data);
		if (thresholds != NULL)
			kfree(thresholds);
		if (adj != NULL)
			kfree(adj);
		pr_err("MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",
			count_fail);
		return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
	}


ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}


/**
  * Perform all the test selected in a TestTodo variable related to MS low power
  * raw data
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_raw_lp(char *path_limits, int stop_on_fail,
			      TestToDo *todo)
{
	int ret, count_fail = 0;
	MutualSenseFrame msRawFrame;


	int *thresholds = NULL;
	int trows, tcolumns;

	u16 *adj = NULL;

	msRawFrame.node_data = NULL;

	/************** Mutual Sense Test **************/
	pr_info("MS RAW LP DATA TEST:\n");
	if (todo->MutualRawLP == 1 || todo->MutualRawGapLP == 1 ||
	    todo->MutualRawAdjLP == 1) {
		ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_LP_ACTIVE);
		mdelay(WAIT_FOR_FRESH_FRAMES);
		ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
		mdelay(WAIT_AFTER_SENSEOFF);
		ret |= getMSFrame3(MS_RAW, &msRawFrame);
		if (ret < 0) {
			pr_err("production_test_data: getMSFrame failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			return ret | ERROR_PROD_TEST_DATA;
		}

		print_frame_short("MS Raw LP frame =",
				  array1dTo2d_short(
					  msRawFrame.node_data,
					  msRawFrame.node_data_size,
					  msRawFrame.header.sense_node),
				  msRawFrame.header.force_node,
				  msRawFrame.header.sense_node);

		pr_info("MS RAW LP MIN MAX TEST:\n");
		if (todo->MutualRawLP == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_RAW_LP_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_LP_MIN_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMinMax(msRawFrame.node_data,
						msRawFrame.header.force_node,
						msRawFrame.header.sense_node,
						thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMinMax MS RAW LP failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS RAW LP MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW LP MIN MAX TEST:.................OK\n");
			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("MS RAW LP MIN MAX TEST:.................SKIPPED\n");

		pr_info("MS RAW LP GAP TEST:\n");
		if (todo->MutualRawGapLP == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_RAW_LP_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_LP_GAP failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap(msRawFrame.node_data,
					     msRawFrame.header.force_node,
					     msRawFrame.header.sense_node,
					     thresholds[0]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsGap MS RAW LP failed... ERROR = %08X\n",
					ret);
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW LP GAP TEST:.................OK\n\n");
			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("MS RAW LP GAP TEST:.................SKIPPED\n");

		pr_info("MS RAW LP ADJ TEST:\n");
		if (todo->MutualRawAdjLP == 1) {
			pr_info("MS RAW LP ADJ HORIZONTAL TEST:\n");
			ret = computeAdjHorizTotal(msRawFrame.node_data,
						   msRawFrame.header.force_node,
						   msRawFrame.header.sense_node,
						   &adj);
			if (ret < 0) {
				pr_err("production_test_data: computeAdjHoriz failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_RAW_LP_ADJH,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != msRawFrame.header.force_node ||
					tcolumns !=
					msRawFrame.header.sense_node - 1)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_LP_ADJH failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMapAdjTotal(adj,
						     msRawFrame.header.
						     force_node,
						     msRawFrame.header.
						     sense_node - 1,
						     thresholds);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsAdj MS RAW LP ADJH failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS RAW LP ADJ HORIZONTAL TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW LP ADJ HORIZONTAL TEST:.................OK\n");

			kfree(thresholds);
			thresholds = NULL;

			kfree(adj);
			adj = NULL;

			pr_info("MS RAW LP ADJ VERTICAL TEST:\n");
			ret = computeAdjVertTotal(msRawFrame.node_data,
						  msRawFrame.header.force_node,
						  msRawFrame.header.sense_node,
						  &adj);
			if (ret < 0) {
				pr_err("production_test_data: computeAdjVert failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_RAW_LP_ADJV,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != msRawFrame.header.force_node -
					1 || tcolumns !=
					msRawFrame.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits MS_RAW_ADJV failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}


			ret = checkLimitsMapAdjTotal(adj,
						     msRawFrame.header.
						     force_node - 1,
						     msRawFrame.header.
						     sense_node, thresholds);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsAdj MS RAW ADJV failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS RAW LP ADJ VERTICAL TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;
			} else
				pr_info("MS RAW LP ADJ VERTICAL TEST:.................OK\n");
			kfree(thresholds);
			thresholds = NULL;

			kfree(adj);
			adj = NULL;
		} else
			pr_info("MS RAW LP ADJ TEST:.................SKIPPED\n");
	} else
		pr_info("MS RAW LP FRAME TEST:.................SKIPPED\n");

ERROR:
	if (count_fail == 0) {
		if (msRawFrame.node_data != NULL) {
			kfree(msRawFrame.node_data);
			msRawFrame.node_data = NULL;
		}
		pr_info("MS RAW DATA TEST finished!.................OK\n");
		return OK;
	} else {
		if (msRawFrame.node_data != NULL)
			kfree(msRawFrame.node_data);
		if (thresholds != NULL)
			kfree(thresholds);
		if (adj != NULL)
			kfree(adj);
		pr_err("MS RAW LP DATA TEST:.................FAIL fails_count = %d\n\n",
			count_fail);
		return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
	}


ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}

/**
  * Perform MS raw test for keys data
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_key_raw(char *path_limits)
{
	int ret;
	MutualSenseFrame msRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;

	/************** Mutual Sense Test **************/
	pr_info("MS KEY RAW DATA TEST is starting...\n");
	ret = setScanMode(SCAN_MODE_ACTIVE, 0xFF);
	mdelay(WAIT_FOR_FRESH_FRAMES);
	ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
	mdelay(WAIT_AFTER_SENSEOFF);
	ret |= getMSFrame3(MS_KEY_RAW, &msRawFrame);
	if (ret < 0) {
		pr_err("production_test_data: getMSKeyFrame failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = parseProductionTestLimits(path_limits, &limit_file,
					MS_KEY_RAW_MIN_MAX, &thresholds, &trows,
					&tcolumns);
	if (ret < 0 || (trows != 1 || tcolumns != 2)) {
		pr_err("production_test_data: parseProductionTestLimits MS_KEY_RAW_MIN_MAX failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
	}

	ret = checkLimitsMinMax(msRawFrame.node_data,
				msRawFrame.header.force_node,
				msRawFrame.header.sense_node,
				thresholds[0], thresholds[1]);
	if (ret != OK) {
		pr_err("production_test_data: checkLimitsMinMax MS KEY RAW failed... ERROR COUNT = %d\n",
			ret);
		goto ERROR;
	} else
		pr_info("MS KEY RAW TEST:.................OK\n\n");

	kfree(thresholds);
	thresholds = NULL;

	kfree(msRawFrame.node_data);
	msRawFrame.node_data = NULL;
	return OK;

ERROR:
	print_frame_short("MS Key Raw frame =", array1dTo2d_short(
				  msRawFrame.node_data,
				  msRawFrame.node_data_size,
				  msRawFrame.header.sense_node),
			  msRawFrame.header.force_node,
			  msRawFrame.header.sense_node);
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	pr_err("MS KEY RAW TEST:.................FAIL\n\n");
	return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;

ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to MS Init
  * data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_cx(char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	int trows, tcolumns;

	MutualSenseData msCompData;
	TotMutualSenseData totCompData;

	u8 *adjhor = NULL;

	u8 *adjvert = NULL;

	u16 container;
	/* u16 *total_cx = NULL; */
	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;


	/* MS CX TEST */
	pr_info("MS CX Testes are starting...\n");

	ret = readMutualSenseCompensationData(LOAD_CX_MS_TOUCH, &msCompData);
	/* read MS compensation data */
	if (ret < 0) {
		pr_err("production_test_data: readMutualSenseCompensationData failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = readTotMutualSenseCompensationData(LOAD_PANEL_CX_TOT_MS_TOUCH,
						 &totCompData);
	/* read  TOT MS compensation data */
	if (ret < 0) {
		pr_err("production_test_data: readTotMutualSenseCompensationData failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return ret | ERROR_PROD_TEST_DATA;
	}

	pr_info("MS CX1 TEST:\n");
	if (todo->MutualCx1 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_CX1_MIN_MAX, &thresholds,
						&trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			pr_err("production_test_data: parseProductionTestLimits MS_CX1_MIN_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (u16)msCompData.cx1;
		ret = checkLimitsMinMax(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMinMax MS CX1 failed... ERROR COUNT = %d\n",
				ret);
			pr_err("MS CX1 TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("MS CX1 TEST:.................OK\n\n");
	} else
		pr_info("MS CX1 TEST:.................SKIPPED\n\n");

	kfree(thresholds);
	thresholds = NULL;

	pr_info("MS CX2 MIN MAX TEST:\n");
	if (todo->MutualCx2 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_CX2_MAP_MIN, &thresholds_min,
						&trows, &tcolumns);
						/* load min thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits MS_CX2_MAP_MIN failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_CX2_MAP_MAX, &thresholds_max,
						&trows, &tcolumns);
						/* load max thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits MS_CX2_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     thresholds_min, thresholds_max);
					 /* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap MS CX2 MIN MAX failed... ERROR COUNT = %d\n",
				ret);
			pr_err("MS CX2 MIN MAX TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("MS CX2 MIN MAX TEST:.................OK\n\n");

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		pr_info("MS CX2 MIN MAX TEST:.................SKIPPED\n\n");

	pr_info("MS CX2 ADJ TEST:\n");
	if (todo->MutualCx2Adj == 1) {
		/* MS CX2 ADJ HORIZ */
		pr_info("MS CX2 ADJ HORIZ TEST:\n");

		ret = computeAdjHoriz(msCompData.node_data,
				      msCompData.header.force_node,
				      msCompData.header.sense_node,
				      &adjhor);
		if (ret < 0) {
			pr_err("production_test_data: computeAdjHoriz failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		pr_info("MS CX2 ADJ HORIZ computed!\n");

		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_CX2_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node ||
				tcolumns != msCompData.header.sense_node - 1)) {
			pr_err("production_test_data: parseProductionTestLimits MS_CX2_ADJH_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj(adjhor, msCompData.header.force_node,
					msCompData.header.sense_node - 1,
					thresholds_max);
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMapAdj CX2 ADJH failed... ERROR COUNT = %d\n",
				ret);
			pr_err("MS CX2 ADJ HORIZ TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("MS CX2 ADJ HORIZ TEST:.................OK\n\n");

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;

		/* MS CX2 ADJ VERT */
		pr_info("MS CX2 ADJ VERT TEST:\n");

		ret = computeAdjVert(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     &adjvert);
		if (ret < 0) {
			pr_err("production_test_data: computeAdjVert failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		pr_info("MS CX2 ADJ VERT computed!\n");

		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_CX2_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node - 1 ||
				tcolumns != msCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits MS_CX2_ADJV_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj(adjvert, msCompData.header.force_node -
					1, msCompData.header.sense_node - 1,
					thresholds_max);
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMapAdj CX2 ADJV failed... ERROR COUNT = %d\n",
				ret);
			pr_err("MS CX2 ADJ HORIZ TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("MS CX2 ADJ VERT TEST:.................OK\n\n");

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		pr_info("MS CX2 ADJ TEST:.................SKIPPED\n\n");

	/* START OF TOTAL CHECK */
	pr_info("MS TOTAL CX TEST:\n");

	if (todo->MutualCxTotal == 1 || todo->MutualCxTotalAdj == 1) {
		pr_info("MS TOTAL CX MIN MAX TEST:\n");
		if (todo->MutualCxTotal == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_TOTAL_CX_MAP_MIN,
							&thresholds_min,
							&trows, &tcolumns);
			/* load min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits MS_TOTAL_CX_MAP_MIN failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							MS_TOTAL_CX_MAP_MAX,
							&thresholds_max,
							&trows, &tcolumns);
			/* load max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits MS_TOTAL_CX_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal(totCompData.node_data,
						  totCompData.header.force_node,
						  totCompData.header.sense_node,
						  thresholds_min,
						  thresholds_max);
			/* check the limits */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMap  MS TOTAL CX TEST failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS TOTAL CX MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("MS TOTAL CX MIN MAX TEST:.................OK\n\n");

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			pr_info("MS TOTAL CX MIN MAX TEST:.................SKIPPED\n\n");


		pr_info("MS TOTAL CX ADJ TEST:\n");
		if (todo->MutualCxTotalAdj == 1) {
			/* MS TOTAL CX ADJ HORIZ */
			pr_info("MS TOTAL CX ADJ HORIZ TEST:\n");

			ret = computeAdjHorizTotal(totCompData.node_data,
					   totCompData.header.force_node,
					   totCompData.header.sense_node,
					   &total_adjhor);
			if (ret < 0) {
				pr_err("production_test_data: computeAdjHoriz failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			pr_info("MS TOTAL CX ADJ HORIZ computed!\n");

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						MS_TOTAL_CX_ADJH_MAP_MAX,
						&thresholds_max,
						&trows, &tcolumns);
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns !=
					totCompData.header.sense_node - 1)) {
				pr_err("production_test_data: parseProductionTestLimits MS_TOTAL_CX_ADJH_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal(total_adjhor,
						     totCompData.header.
						     force_node,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMapAdj MS TOTAL CX ADJH failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS TOTAL CX ADJ HORIZ TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("MS TOTAL CX ADJ HORIZ TEST:.................OK\n\n");

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;

			/* MS TOTAL CX ADJ VERT */
			pr_info("MS TOTAL CX ADJ VERT TEST:\n");

			ret = computeAdjVertTotal(totCompData.node_data,
						  totCompData.header.force_node,
						  totCompData.header.sense_node,
						  &total_adjvert);
			if (ret < 0) {
				pr_err("production_test_data: computeAdjVert failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			pr_info("MS TOTAL CX ADJ VERT computed!\n");

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						MS_TOTAL_CX_ADJV_MAP_MAX,
						&thresholds_max,
						&trows, &tcolumns);
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns !=
					totCompData.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits MS_TOTAL_CX_ADJV_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal(total_adjvert,
						     totCompData.header.
						     force_node - 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMapAdj MS TOTAL CX ADJV failed... ERROR COUNT = %d\n",
					ret);
				pr_err("MS TOTAL CX ADJ HORIZ TEST:.................FAIL\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("MS TOTAL CX ADJ VERT TEST:.................OK\n");

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			pr_info("MS TOTAL CX ADJ TEST:.................SKIPPED\n");

		kfree(totCompData.node_data);
		totCompData.node_data = NULL;
	} else
		pr_info("MS TOTAL CX TEST:.................SKIPPED\n");



	if ((todo->MutualKeyCx1 | todo->MutualKeyCx2 |
	     todo->MutualKeyCxTotal) == 1) {
		ret = production_test_ms_key_cx(path_limits, stop_on_fail,
						todo);
		if (ret < 0) {
			count_fail += 1;
			pr_err("production_test_data: production_test_ms_key_cx failed... ERROR = %08X\n",
				ret);
			pr_err("MS CX testes finished!.................FAILED  fails_count = %d\n\n",
				count_fail);
			return ret;
		}
	} else
		pr_info("MS KEY CX TEST:.................SKIPPED\n");

ERROR:

	if (count_fail == 0) {
		pr_info("MS CX testes finished!.................OK\n");
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return OK;
	} else {
		print_frame_i8("MS Init Data (Cx2) =", array1dTo2d_i8(
				       msCompData.node_data,
				       msCompData.node_data_size,
				       msCompData.header.sense_node),
			       msCompData.header.force_node,
			       msCompData.header.sense_node);
		print_frame_short(" TOT MS Init Data (Cx) =", array1dTo2d_short(
					  totCompData.node_data,
					  totCompData.node_data_size,
					  totCompData.header.sense_node),
				  totCompData.header.force_node,
				  totCompData.header.sense_node);
		pr_err("MS CX testes finished!.................FAILED  fails_count = %d\n\n",
			count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (adjhor != NULL)
			kfree(adjhor);
		if (adjvert != NULL)
			kfree(adjvert);
		if (totCompData.node_data != NULL)
			kfree(totCompData.node_data);
		if (total_adjhor != NULL)
			kfree(total_adjhor);
		if (total_adjvert != NULL)
			kfree(total_adjvert);
		if (msCompData.node_data != NULL)
			kfree(msCompData.node_data);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (totCompData.node_data != NULL)
		kfree(totCompData.node_data);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to MS Init
  * data of the keys
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_key_cx(char *path_limits, int stop_on_fail,
			      TestToDo *todo)
{
	int ret;
	int count_fail = 0;
	int num_keys = 0;

	int *thresholds = NULL;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	int trows, tcolumns;

	MutualSenseData msCompData;
	TotMutualSenseData totCompData;


	short container;


	/* MS CX TEST */
	pr_info("MS KEY CX Testes are starting...\n");

	ret = readMutualSenseCompensationData(LOAD_CX_MS_KEY, &msCompData);
	/* read MS compensation data */
	if (ret < 0) {
		pr_err("production_test_data: readMutualSenseCompensationData failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	if (msCompData.header.force_node > msCompData.header.sense_node)
		/* the meaningful data are only in the first row,
		  * the other rows are only a copy of the first one */
		num_keys = msCompData.header.force_node;
	else
		num_keys = msCompData.header.sense_node;

	pr_info("MS KEY CX1 TEST:\n");
	if (todo->MutualKeyCx1 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_KEY_CX1_MIN_MAX, &thresholds,
						&trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			pr_err("production_test_data: parseProductionTestLimits MS_KEY_CX1_MIN_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)msCompData.cx1;
		ret = checkLimitsMinMax(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMinMax MS CX1 failed... ERROR COUNT = %d\n",
				ret);
			pr_err("MS KEY CX1 TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("MS KEY CX1 TEST:.................OK\n\n");
	} else
		pr_info("MS KEY CX1 TEST:.................SKIPPED\n\n");

	kfree(thresholds);
	thresholds = NULL;

	pr_info("MS KEY CX2 TEST:\n");
	if (todo->MutualKeyCx2 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_KEY_CX2_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load min thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node  ||
				tcolumns != msCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits MS_KEY_CX2_MAP_MIN failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_KEY_CX2_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load max thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node  ||
				tcolumns != msCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits MS_KEY_CX2_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap(msCompData.node_data,
				     msCompData.header.force_node,
				     msCompData.header.sense_node,
				     thresholds_min, thresholds_max);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap MS KEY CX2 failed... ERROR COUNT = %d\n",
				ret);
			pr_err("MS KEY CX2 TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("MS KEY CX2 TEST:.................OK\n\n");

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		pr_info("MS CX2 TEST:.................SKIPPED\n\n");

	/* START OF TOTAL CHECK */
	pr_info("MS KEY TOTAL CX TEST:\n");

	if (todo->MutualKeyCxTotal == 1) {
		ret = readTotMutualSenseCompensationData(
			LOAD_PANEL_CX_TOT_MS_KEY, &totCompData);
		if (ret < 0) {
			pr_err("production_test_data: computeTotalCx failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_KEY_TOTAL_CX_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load min thresholds */
		if (ret < 0 || (trows != totCompData.header.force_node ||
				tcolumns != totCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits MS_KEY_TOTAL_CX_MAP_MIN failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						MS_KEY_TOTAL_CX_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load max thresholds */
		if (ret < 0 || (trows != totCompData.header.force_node  ||
				tcolumns != totCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits MS_KEY_TOTAL_CX_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapTotal(totCompData.node_data,
					  totCompData.header.force_node,
					  totCompData.header.sense_node,
					  thresholds_min, thresholds_max);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap  MS TOTAL KEY CX TEST failed... ERROR COUNT = %d\n",
				ret);
			pr_err("MS KEY TOTAL CX TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("MS KEY TOTAL CX TEST:.................OK\n\n");

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;

		kfree(totCompData.node_data);
		totCompData.node_data = NULL;
	} else
		pr_info("MS KEY TOTAL CX TEST:.................SKIPPED\n");


ERROR:
	if (count_fail == 0) {
		pr_info("MS KEY CX testes finished!.................OK\n");
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return OK;
	} else {
		print_frame_i8("MS Key Init Data (Cx2) =", array1dTo2d_i8(
				       msCompData.node_data,
				       msCompData.node_data_size,
				       msCompData.header.sense_node),
			       msCompData.header.force_node,
			       msCompData.header.sense_node);
		pr_err("MS Key CX testes finished!.................FAILED  fails_count = %d\n\n",
			count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (msCompData.node_data != NULL)
			kfree(msCompData.node_data);
		if (totCompData.node_data != NULL)
			kfree(totCompData.node_data);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	if (totCompData.node_data != NULL)
		kfree(totCompData.node_data);
	return ret;
}

/**
  * Perform all the test selected in a TestTodo variable related to SS raw data
  *(touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_raw(char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret;
	int count_fail = 0;
	int rows, columns;

	SelfSenseFrame ssRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;

	/* SS TEST */
	pr_info("SS RAW Testes are starting...\n");

	/************** Self Sense Test **************/

	pr_info("Getting SS Frame...\n");
	ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
	mdelay(WAIT_FOR_FRESH_FRAMES);
	ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
	mdelay(WAIT_AFTER_SENSEOFF);
	ret |= getSSFrame3(SS_RAW, &ssRawFrame);
	if (ret < 0) {
		pr_err("production_test_data: getSSFrame failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	print_frame_short("SS Raw force frame =",
			  array1dTo2d_short(
				  ssRawFrame.force_data,
				  ssRawFrame.header.force_node,
				  1),
			  ssRawFrame.header.force_node, 1);
	print_frame_short("SS Raw sense frame =",
			  array1dTo2d_short(
				  ssRawFrame.sense_data,
				  ssRawFrame.header.sense_node,
				  ssRawFrame.header.sense_node),
			  1, ssRawFrame.header.sense_node);

	/* SS RAW (PROXIMITY) FORCE TEST */
	pr_info("SS RAW FORCE TEST:\n");



	if (todo->SelfForceRaw == 1 || todo->SelfForceRawGap == 1) {
		columns = 1;	/* there are no data for the sense channels
				  * because is a force frame */
		rows = ssRawFrame.header.force_node;

		pr_info("SS RAW FORCE MIN MAX TEST:\n");
		if (todo->SelfForceRaw == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_FORCE_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_FORCE_MIN_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax(ssRawFrame.force_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMinMax SS RAW FORCE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS RAW (PROXIMITY) FORCE MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW FORCE MIN MAX TEST:.................OK\n\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW FORCE MIN MAX TEST:.................SKIPPED\n\n");

		pr_info("SS RAW FORCE GAP TEST:\n");
		if (todo->SelfForceRawGap == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_FORCE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_FORCE_GAP failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap(ssRawFrame.force_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsGap SS RAW FORCE GAP failed... ERROR = %08X\n",
					ret);
				pr_err("SS RAW FORCE GAP TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW FORCE GAP TEST:.................OK\n\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW FORCE GAP TEST:.................SKIPPED\n\n");

		kfree(ssRawFrame.force_data);
		ssRawFrame.force_data = NULL;
	} else
		pr_info("SS RAW FORCE TEST:.................SKIPPED\n\n");

	/* SS RAW (PROXIMITY) SENSE TEST */
	pr_info("SS RAW SENSE TEST:\n");

	if (todo->SelfSenseRaw == 1 || todo->SelfSenseRawGap == 1) {
		columns = ssRawFrame.header.sense_node;
		rows = 1;/* there are no data for the force channels
			  *  because is a sense frame */

		pr_info("SS RAW SENSE MIN MAX TEST:\n");
		if (todo->SelfSenseRaw == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_SENSE_MIN_MAX,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_SENSE_MIN_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax(ssRawFrame.sense_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMinMax SS RAW SENSE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS RAW SENSE MIN MAX TEST:.................FAIL\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW SENSE MIN MAX TEST:.................OK\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW SENSE MIN MAX TEST:.................SKIPPED\n");

		pr_info("SS RAW SENSE GAP TEST:\n");
		if (todo->SelfSenseRawGap == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_SENSE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_SENSE_GAP failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap(ssRawFrame.sense_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsGap SS RAW SENSE GAP failed... ERROR = %08X\n",
					ret);
				pr_err("SS RAW SENSE GAP TEST:.................FAIL\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW SENSE GAP TEST:.................OK\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW SENSE GAP TEST:.................SKIPPED\n");

		kfree(ssRawFrame.sense_data);
		ssRawFrame.sense_data = NULL;
	}

	ret = production_test_ss_raw_lp(path_limits, stop_on_fail, todo);
	if (ret < OK) {
		pr_err("production_test_data: production_test_ss_raw_lp failed... ERROR = %08X\n",
			ret);
		count_fail += 1;
	}

	if (count_fail == 0) {
		pr_info("SS RAW testes finished!.................OK\n\n");
		return OK;
	} else {
		pr_err("SS RAW testes finished!.................FAILED  fails_count = %d\n\n",
			count_fail);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (ssRawFrame.force_data != NULL)
		kfree(ssRawFrame.force_data);
	if (ssRawFrame.sense_data != NULL)
		kfree(ssRawFrame.sense_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}


/**
  * Perform all the test selected in a TestTodo variable related to SS raw data
  * low power
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_raw_lp(char *path_limits, int stop_on_fail,
			      TestToDo *todo)
{
	int ret;
	int count_fail = 0;
	int rows, columns;

	SelfSenseFrame ssRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;

	/* SS TEST */
	pr_info("SS RAW LP Testes are starting...\n");

	/************** Self Sense Test **************/

	pr_info("Getting SS LP Frame...\n");
	ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_LP_DETECT);
	mdelay(WAIT_FOR_FRESH_FRAMES);
	ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
	mdelay(WAIT_AFTER_SENSEOFF);
	ret |= getSSFrame3(SS_RAW, &ssRawFrame);
	if (ret < 0) {
		pr_err("production_test_data: getSSFrame failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	print_frame_short("SS Raw LP force frame =",
			  array1dTo2d_short(
				  ssRawFrame.force_data,
				  ssRawFrame.header.force_node, 1),
			  ssRawFrame.header.force_node, 1);
	print_frame_short("SS Raw LP sense frame =",
			  array1dTo2d_short(
				  ssRawFrame.sense_data,
				  ssRawFrame.header.sense_node,
				  ssRawFrame.header.sense_node),
			  1, ssRawFrame.header.sense_node);

	/* SS RAW (PROXIMITY) FORCE TEST */
	pr_info("SS RAW LP FORCE TEST:\n");

	if (todo->SelfForceRawLP == 1 || todo->SelfForceRawGapLP == 1) {
		columns = 1;	/* there are no data for the sense channels
				  *  because is a force frame */
		rows = ssRawFrame.header.force_node;

		pr_info("SS RAW LP FORCE MIN MAX TEST:\n");
		if (todo->SelfForceRawLP == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_LP_FORCE_MIN_MAX,
							&thresholds,
							&trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_FORCE_MIN_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax(ssRawFrame.force_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMinMax SS RAW FORCE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS RAW LP FORCE MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW LP FORCE MIN MAX TEST:.................OK\n\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW LP FORCE MIN MAX TEST:.................SKIPPED\n\n");

		pr_info("SS RAW LP FORCE GAP TEST:\n");
		if (todo->SelfForceRawGapLP == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_LP_FORCE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_FORCE_GAP failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap(ssRawFrame.force_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsGap SS RAW FORCE GAP failed... ERROR = %08X\n",
					ret);
				pr_err("SS RAW LP FORCE GAP TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW LP FORCE GAP TEST:.................OK\n\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW LP FORCE GAP TEST:.................SKIPPED\n\n");

		kfree(ssRawFrame.force_data);
		ssRawFrame.force_data = NULL;
	} else
		pr_info("SS RAW LP FORCE TEST:.................SKIPPED\n\n");

	/* SS RAW (PROXIMITY) SENSE TEST */
	pr_info("SS RAW LP SENSE TEST:\n");

	if (todo->SelfSenseRawLP == 1 || todo->SelfSenseRawGapLP == 1) {
		columns = ssRawFrame.header.sense_node;
		rows = 1;/* there are no data for the force channels
			  * because is a sense frame */

		pr_info("SS RAW LP SENSE MIN MAX TEST:\n");
		if (todo->SelfSenseRawLP == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_LP_SENSE_MIN_MAX,
							&thresholds,
							&trows, &tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 2)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_SENSE_MIN_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMinMax(ssRawFrame.sense_data, rows,
						columns, thresholds[0],
						thresholds[1]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMinMax SS RAW SENSE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS RAW LP SENSE MIN MAX TEST:.................FAIL\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW SENSE MIN MAX TEST:.................OK\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW LP SENSE MIN MAX TEST:.................SKIPPED\n");

		pr_info("SS RAW LP SENSE GAP TEST:\n");
		if (todo->SelfSenseRawGapLP == 1) {
			ret = parseProductionTestLimits(path_limits,
							&limit_file,
							SS_RAW_LP_SENSE_GAP,
							&thresholds, &trows,
							&tcolumns);
			if (ret < OK || (trows != 1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_RAW_SENSE_GAP failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsGap(ssRawFrame.sense_data, rows,
					     columns, thresholds[0]);
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsGap SS RAW SENSE GAP failed... ERROR = %08X\n",
					ret);
				pr_err("SS RAW LP SENSE GAP TEST:.................FAIL\n");
				count_fail += 1;
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA |
					      ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else
				pr_info("SS RAW LP SENSE GAP TEST:.................OK\n");

			kfree(thresholds);
			thresholds = NULL;
		} else
			pr_info("SS RAW LP SENSE GAP TEST:.................SKIPPED\n");

		kfree(ssRawFrame.sense_data);
		ssRawFrame.sense_data = NULL;
	}

	if (count_fail == 0) {
		pr_info("SS RAW LP testes finished!.................OK\n\n");
		return OK;
	} else {
		pr_err("SS RAW LP testes finished!.................FAILED  fails_count = %d\n\n",
			count_fail);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (ssRawFrame.force_data != NULL)
		kfree(ssRawFrame.force_data);
	if (ssRawFrame.sense_data != NULL)
		kfree(ssRawFrame.sense_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to SS Init
  * data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_ix_cx(char *path_limits, int stop_on_fail,
			     TestToDo *todo)
{
	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;

	SelfSenseData ssCompData;
	TotSelfSenseData totCompData;

	u8 *adjhor = NULL;
	u8 *adjvert = NULL;

	short container;

	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;

	pr_info("SS IX CX testes are starting...\n");
	ret = readSelfSenseCompensationData(LOAD_CX_SS_TOUCH, &ssCompData);
	/* read the SS compensation data */
	if (ret < 0) {
		pr_err("production_test_data: readSelfSenseCompensationData failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		return ret | ERROR_PROD_TEST_DATA;
	}

	ret = readTotSelfSenseCompensationData(LOAD_PANEL_CX_TOT_SS_TOUCH,
					       &totCompData);
	/* read the TOT SS compensation data */
	if (ret < 0) {
		pr_err("production_test_data: readTotSelfSenseCompensationData failed... ERROR %08X\n",
			ERROR_PROD_TEST_DATA);
		kfree(ssCompData.ix2_fm);
		kfree(ssCompData.ix2_sn);
		kfree(ssCompData.cx2_fm);
		kfree(ssCompData.cx2_sn);
		return ret | ERROR_PROD_TEST_DATA;
	}

	/************* SS FORCE IX **************/
	/* SS IX1 FORCE TEST */
	pr_info("SS IX1 FORCE TEST:\n");
	if (todo->SelfForceIx1 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX1_FORCE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX1_FORCE_MIN_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		container = (short)ssCompData.f_ix1;
		ret = checkLimitsMinMax(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMinMax SS IX1 FORCE TEST failed... ERROR COUNT = %d\n",
				ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS IX1 FORCE TEST:.................OK\n\n");
	} else
		pr_info("SS IX1 FORCE TEST:.................SKIPPED\n\n");

	kfree(thresholds);
	thresholds = NULL;
	/* SS IX2 FORCE TEST */
	pr_info("SS IX2 FORCE MIN MAX TEST:\n");
	if (todo->SelfForceIx2 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX2_FORCE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX2_FORCE_MAP_MIN failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX2_FORCE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX2_FORCE_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapFromU(ssCompData.ix2_fm,
					  ssCompData.header.force_node, 1,
					  thresholds_min,
					  thresholds_max);	/* check the
								 * values with
								 * thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap SS IX2 FORCE failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS IX2 FORCE MIN MAX TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS IX2 FORCE MIN MAX TEST:.................OK\n\n");

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		pr_info("SS IX2 FORCE MIN MAX TEST:.................SKIPPED\n\n");

	pr_info("SS IX2 FORCE ADJ TEST:\n");
	if (todo->SelfForceIx2Adj == 1) {
		/* SS IX2 FORCE ADJV TEST */
		pr_info("SS IX2 FORCE ADJVERT TEST:\n");
		ret = computeAdjVertFromU(ssCompData.ix2_fm,
					  ssCompData.header.force_node, 1,
					  &adjvert);
		if (ret < 0) {
			pr_err("production_test_data: computeAdjVert SS IX2 FORCE ADJV failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		pr_info("SS IX2 FORCE ADJV computed!\n");

		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX2_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);	/* load the max
								 * thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 ||
				tcolumns != 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX2_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj(adjvert, ssCompData.header.force_node -
					1, 1, thresholds_max);	/* check the
								 * values with
								 * thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap SS IX2 FORCE failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS IX2 FORCE ADJV TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS IX2 FORCE ADJV TEST:.................OK\n\n");

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		pr_info("SS IX2 FORCE ADJ TEST:.................SKIPPED\n\n");

	/* SS TOTAL FORCE IX */
	pr_info("SS TOTAL IX FORCE TEST:\n");
	if (todo->SelfForceIxTotal == 1 || todo->SelfForceIxTotalAdj == 1) {
		pr_info("SS TOTAL IX FORCE MIN MAX TEST:\n");
		if (todo->SelfForceIxTotal == 1) {
			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_IX_FORCE_MAP_MIN,
						&thresholds_min,
						&trows, &tcolumns);
						/* load the min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_IX_FORCE_MAP_MIN failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_IX_FORCE_MAP_MAX,
						&thresholds_max,
						&trows, &tcolumns);
						/* load the max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_IX_FORCE_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotalFromU(totCompData.ix_fm,
						       totCompData.header.
						       force_node, 1,
						       thresholds_min,
						       thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMap  SS TOTAL IX FORCE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL IX FORCE MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL IX FORCE MIN MAX TEST:.................OK\n\n");

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			pr_info("SS TOTAL IX FORCE MIN MAX TEST:.................SKIPPED\n");

		pr_info("SS TOTAL IX FORCE ADJ TEST:\n");
		if (todo->SelfForceIxTotalAdj == 1) {
			/* SS TOTAL IX FORCE ADJV TEST */
			pr_info("SS TOTAL IX FORCE ADJVERT TEST:\n");
			ret = computeAdjVertTotalFromU(totCompData.ix_fm,
						       totCompData.header.
						       force_node, 1,
						       &total_adjvert);
			if (ret < 0) {
				pr_err("production_test_data: computeAdjVert SS TOTAL IX FORCE ADJV failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			pr_info("SS TOTAL IX FORCE ADJV computed!\n");

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_IX_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_IX_FORCE_ADJV_MAP_MAX... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal(total_adjvert,
						     totCompData.header.
						     force_node - 1, 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMap SS TOTAL IX FORCE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL IX FORCE ADJV TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL IX FORCE ADJV TEST:.................OK\n\n");

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			pr_info("SS TOTAL IX FORCE ADJ TEST:.................SKIPPED\n");
	} else
		pr_info("SS TOTAL IX FORCE TEST:.................SKIPPED\n\n");


	/************** SS SENSE IX **************/
	/* SS IX1 SENSE TEST */
	pr_info("SS IX1 SENSE TEST:\n");
	if (todo->SelfSenseIx1 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX1_SENSE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX1_SENSE_MIN_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.s_ix1;
		ret = checkLimitsMinMax(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMinMax SS IX1 SENSE TEST failed... ERROR COUNT = %d\n",
				ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS IX1 SENSE TEST:.................OK\n\n");
	} else
		pr_info("SS IX1 SENSE TEST:.................SKIPPED\n\n");

	kfree(thresholds);
	thresholds = NULL;
	/* SS IX2 SENSE TEST */
	pr_info("SS IX2 SENSE MIN MAX TEST:\n");
	if (todo->SelfSenseIx2 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX2_SENSE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX2_SENSE_MAP_MIN failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX2_SENSE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapFromU(ssCompData.ix2_sn, 1,
					  ssCompData.header.sense_node,
					  thresholds_min, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap SS IX2 SENSE failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS IX2 SENSE MIN MAX TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS IX2 SENSE MIN MAX TEST:.................OK\n\n");

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		pr_info("SS IX2 SENSE MIN MAX TEST:.................SKIPPED\n\n");

	pr_info("SS IX2 SENSE ADJ TEST:\n");
	if (todo->SelfSenseIx2Adj == 1) {
		/* SS IX2 SENSE ADJH TEST */
		pr_info("SS IX2 SENSE ADJHORIZ TEST:\n");
		ret = computeAdjHorizFromU(ssCompData.ix2_sn, 1,
					   ssCompData.header.sense_node,
					   &adjhor);
		if (ret < 0) {
			pr_err("production_test_data: computeAdjHoriz SS IX2 SENSE ADJH failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		pr_info("SS IX2 SENSE ADJ HORIZ computed!\n");


		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_IX2_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node - 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX2_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj(adjhor, 1,
					ssCompData.header.sense_node - 1,
					thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMapAdj SS IX2 SENSE ADJH failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS IX2 SENSE ADJH TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS IX2 SENSE ADJH TEST:.................OK\n\n");

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;
	} else
		pr_info("SS IX2 SENSE ADJ TEST:.................SKIPPED\n");

	/* SS TOTAL IX SENSE */
	pr_info("SS TOTAL IX SENSE TEST:\n");
	if (todo->SelfSenseIxTotal == 1 || todo->SelfSenseIxTotalAdj == 1) {
		pr_info("SS TOTAL IX SENSE MIN MAX TEST:\n");
		if (todo->SelfSenseIxTotal == 1) {
			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_IX_SENSE_MAP_MIN,
						&thresholds_min,
						&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_IX_SENSE_MAP_MIN failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_IX_SENSE_MAP_MAX,
						&thresholds_max,
						&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_IX_SENSE_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotalFromU(totCompData.ix_sn, 1,
						       totCompData.header.
						       sense_node,
						       thresholds_min,
						       thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMap SS TOTAL IX SENSE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL IX SENSE MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL IX SENSE MIN MAX TEST:.................OK\n\n");

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			pr_info("SS TOTAL IX SENSE MIN MAX TEST:.................SKIPPED\n");


		pr_info("SS TOTAL IX SENSE ADJ TEST:\n");
		if (todo->SelfSenseIxTotalAdj == 1) {
			/* SS TOTAL IX SENSE ADJH TEST */
			pr_info("SS TOTAL IX SENSE ADJHORIZ TEST:\n");
			ret = computeAdjHorizTotalFromU(totCompData.ix_sn, 1,
							totCompData.header.
							sense_node,
							&total_adjhor);
			if (ret < 0) {
				pr_err("production_test_data: computeAdjHoriz SS TOTAL IX SENSE ADJH failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			pr_info("SS TOTAL IX SENSE ADJ HORIZ computed!\n");

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_IX_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node - 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_IX_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal(total_adjhor, 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMapAdj SS TOTAL IX SENSE ADJH failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL IX SENSE ADJH TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL IX SENSE ADJH TEST:.................OK\n\n");

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;
		} else
			pr_info("SS TOTAL IX SENSE ADJ TEST:.................SKIPPED\n");
	} else
		pr_info("SS TOTAL IX SENSE TEST:.................SKIPPED\n");

	/************* SS SENSE CX **************/
	/* SS CX1 FORCE TEST */
	pr_info("SS CX1 FORCE TEST:\n");
	if (todo->SelfForceCx1 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX1_FORCE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			pr_err("production_test_data: parseProductionTestLimits SS_CX1_FORCE_MIN_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.f_cx1;
		ret = checkLimitsMinMax(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMinMax SS CX1 FORCE TEST failed... ERROR COUNT = %d\n",
				ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS CX1 FORCE TEST:.................OK\n\n");
		kfree(thresholds);
		thresholds = NULL;
	} else
		pr_info("SS CX1 FORCE TEST:.................SKIPPED\n\n");

	/* SS CX2 FORCE TEST */
	pr_info("SS CX2 FORCE MIN MAX TEST:\n");
	if (todo->SelfForceCx2 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX2_FORCE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_CX2_FORCE_MAP_MIN failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX2_FORCE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node ||
				tcolumns != 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_CX2_FORCE_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap(ssCompData.cx2_fm,
				     ssCompData.header.force_node, 1,
				     thresholds_min,
				     thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap SS CX2 FORCE failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS CX2 FORCE MIN MAX TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS CX2 FORCE MIN MAX TEST:.................OK\n\n");

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		pr_info("SS CX2 FORCE MIN MAX TEST:.................SKIPPED\n");

	pr_info("SS CX2 FORCE ADJ TEST:\n");
	if (todo->SelfForceCx2Adj == 1) {
		/* SS CX2 FORCE ADJV TEST */
		pr_info("SS CX2 FORCE ADJVERT TEST:\n");
		ret = computeAdjVert(ssCompData.cx2_fm,
				     ssCompData.header.force_node, 1, &adjvert);
		/* compute the ADJV for CX2  FORCE */
		if (ret < 0) {
			pr_err("production_test_data: computeAdjVert SS CX2 FORCE ADJV failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		pr_info("SS CX2 FORCE ADJV computed!\n");

		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX2_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 ||
				tcolumns != 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_CX2_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj(adjvert, ssCompData.header.force_node -
					1, 1, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap SS IX2 FORCE failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS CX2 FORCE ADJV TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS CX2 FORCE ADJV TEST:.................OK\n\n");

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjvert);
		adjvert = NULL;
	} else
		pr_info("SS CX2 FORCE ADJ TEST:.................SKIPPED\n\n");

	/* SS TOTAL CX FORCE */
	pr_info("SS TOTAL CX FORCE TEST:\n");
	if (todo->SelfForceCxTotal == 1 || todo->SelfForceCxTotalAdj == 1) {
		pr_info("SS TOTAL CX FORCE MIN MAX TEST:\n");
		if (todo->SelfForceCxTotal == 1) {
			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_CX_FORCE_MAP_MIN,
						&thresholds_min,
						&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_CX_FORCE_MAP_MIN failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_CX_FORCE_MAP_MAX,
						&thresholds_max,
						&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows !=
					totCompData.header.force_node ||
					tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_CX_FORCE_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal(totCompData.cx_fm,
						  totCompData.header.force_node,
						  1, thresholds_min,
						  thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMap SS TOTAL FORCE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL FORCE MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL FORCE MIN MAX TEST:.................OK\n\n");

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			pr_info("SS TOTAL CX FORCE MIN MAX TEST:.................SKIPPED\n");

		/* SS TOTAL CX FORCE ADJV TEST */
		pr_info("SS TOTAL CX FORCE ADJ TEST:\n");
		if (todo->SelfForceCxTotalAdj == 1) {
			pr_info("SS TOTAL CX FORCE ADJVERT TEST:\n");
			ret = computeAdjVertTotal(totCompData.cx_fm,
						  totCompData.header.force_node,
						  1, &total_adjvert);
			/* compute the ADJV for CX2  FORCE */
			if (ret < 0) {
				pr_err("production_test_data: computeAdjVert SS TOTAL CX FORCE ADJV failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			pr_info("SS TOTAL CX FORCE ADJV computed!\n");

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_CX_FORCE_ADJV_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != totCompData.header.force_node -
					1 || tcolumns != 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_CX_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal(total_adjvert,
						     totCompData.header.
						     force_node - 1, 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMap SS TOTAL CX FORCE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL CX FORCE ADJV TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL CX FORCE ADJV TEST:.................OK\n\n");

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjvert);
			total_adjvert = NULL;
		} else
			pr_info("SS TOTAL CX FORCE ADJ TEST:.................SKIPPED\n");
	} else
		pr_info("SS TOTAL CX FORCE TEST:.................SKIPPED\n\n");



	/************* SS SENSE CX *************/
	/* SS CX1 SENSE TEST */
	pr_info("SS CX1 SENSE TEST:\n");
	if (todo->SelfSenseCx1 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX1_SENSE_MIN_MAX,
						&thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			pr_err("production_test_data: parseProductionTestLimits SS_CX1_SENSE_MIN_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		container = (short)ssCompData.s_cx1;
		ret = checkLimitsMinMax(&container, 1, 1, thresholds[0],
					thresholds[1]);
		/* check the limits */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMinMax SS CX1 SENSE TEST failed... ERROR COUNT = %d\n",
				ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS CX1 SENSE TEST:.................OK\n\n");

		kfree(thresholds);
		thresholds = NULL;
	} else
		pr_info("SS CX1 SENSE TEST:.................SKIPPED\n\n");


	/* SS CX2 SENSE TEST */
	pr_info("SS CX2 SENSE MIN MAX TEST:\n");
	if (todo->SelfSenseCx2 == 1) {
		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX2_SENSE_MAP_MIN,
						&thresholds_min, &trows,
						&tcolumns);
		/* load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits SS_CX2_SENSE_MAP_MIN failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX2_SENSE_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node)) {
			pr_err("production_test_data: parseProductionTestLimits SS_CX2_SENSE_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMap(ssCompData.cx2_sn, 1,
				     ssCompData.header.sense_node,
				     thresholds_min, thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMap SS CX2 SENSE failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS CX2 SENSE MIN MAX TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS CX2 SENSE MIN MAX TEST:.................OK\n\n");

		kfree(thresholds_min);
		thresholds_min = NULL;
		kfree(thresholds_max);
		thresholds_max = NULL;
	} else
		pr_info("SS CX2 SENSE MIN MAX TEST:.................SKIPPED\n");

	pr_info("SS CX2 SENSE ADJ TEST:\n");
	if (todo->SelfSenseCx2Adj == 1) {
		/* SS CX2 SENSE ADJH TEST */
		pr_info("SS CX2 SENSE ADJHORIZ TEST:\n");
		ret = computeAdjHoriz(ssCompData.cx2_sn, 1,
				      ssCompData.header.sense_node, &adjhor);
		if (ret < 0) {
			pr_err("production_test_data: computeAdjHoriz SS CX2 SENSE ADJH failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}
		pr_info("SS CX2 SENSE ADJH computed!\n");


		ret = parseProductionTestLimits(path_limits, &limit_file,
						SS_CX2_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
		/* load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns !=
				ssCompData.header.sense_node - 1)) {
			pr_err("production_test_data: parseProductionTestLimits SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n",
				ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = checkLimitsMapAdj(adjhor, 1,
					ssCompData.header.sense_node - 1,
					thresholds_max);
		/* check the values with thresholds */
		if (ret != OK) {
			pr_err("production_test_data: checkLimitsMapAdj SS CX2 SENSE ADJH failed... ERROR COUNT = %d\n",
				ret);
			pr_err("SS CX2 SENSE ADJH TEST:.................FAIL\n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			pr_info("SS CX2 SENSE ADJH TEST:.................OK\n");

		kfree(thresholds_max);
		thresholds_max = NULL;
		kfree(adjhor);
		adjhor = NULL;
	} else
		pr_info("SS CX2 SENSE ADJ TEST:.................SKIPPED\n\n");

	/* SS TOTAL CX SENSE */
	pr_info("SS TOTAL CX SENSE TEST:\n");
	if (todo->SelfSenseCxTotal == 1 || todo->SelfSenseCxTotalAdj == 1) {
		pr_info("SS TOTAL CX SENSE MIN MAX TEST:\n");
		if (todo->SelfSenseCxTotal == 1) {
			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_CX_SENSE_MAP_MIN,
						&thresholds_min,
						&trows, &tcolumns);
			/* load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_CX_SENSE_MAP_MIN failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_CX_SENSE_MAP_MAX,
						&thresholds_max,
						&trows, &tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_CX_SENSE_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapTotal(totCompData.cx_sn, 1,
						  totCompData.header.sense_node,
						  thresholds_min,
						  thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMap SS TOTAL CX SENSE failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL CX SENSE MIN MAX TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL CX SENSE MIN MAX TEST:.................OK\n\n");

			kfree(thresholds_min);
			thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			pr_info("SS TOTAL CX SENSE MIN MAX TEST:.................SKIPPED\n");


		/* SS TOTAL IX SENSE ADJH TEST */
		pr_info("SS TOTAL CX SENSE ADJ TEST:\n");
		if (todo->SelfSenseCxTotalAdj == 1) {
			pr_info("SS TOTAL CX SENSE ADJHORIZ TEST:\n");
			ret = computeAdjHorizTotal(totCompData.cx_sn, 1,
					   totCompData.header.sense_node,
					   &total_adjhor);
			if (ret < 0) {
				pr_err("production_test_data: computeAdjHoriz SS TOTAL CX SENSE ADJH failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			pr_info("SS TOTAL CX SENSE ADJ HORIZ computed!\n");


			ret = parseProductionTestLimits(path_limits,
						&limit_file,
						SS_TOTAL_CX_SENSE_ADJH_MAP_MAX,
						&thresholds_max, &trows,
						&tcolumns);
			/* load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns !=
					totCompData.header.sense_node - 1)) {
				pr_err("production_test_data: parseProductionTestLimits SS_TOTAL_CX_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n",
					ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = checkLimitsMapAdjTotal(total_adjhor, 1,
						     totCompData.header.
						     sense_node - 1,
						     thresholds_max);
			/* check the values with thresholds */
			if (ret != OK) {
				pr_err("production_test_data: checkLimitsMapAdj SS TOTAL CX SENSE ADJH failed... ERROR COUNT = %d\n",
					ret);
				pr_err("SS TOTAL CX SENSE ADJH TEST:.................FAIL\n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				pr_info("SS TOTAL CX SENSE ADJH TEST:.................OK\n\n");

			kfree(thresholds_max);
			thresholds_max = NULL;
			kfree(total_adjhor);
			total_adjhor = NULL;
		} else
			pr_info("SS TOTAL CX SENSE ADJ TEST:.................SKIPPED\n");
	} else
		pr_info("SS TOTAL CX SENSE TEST:.................SKIPPED\n");



ERROR:
	if (count_fail == 0) {
		kfree(ssCompData.ix2_fm);
		ssCompData.ix2_fm = NULL;
		kfree(ssCompData.ix2_sn);
		ssCompData.ix2_sn = NULL;
		kfree(ssCompData.cx2_fm);
		ssCompData.cx2_fm = NULL;
		kfree(ssCompData.cx2_sn);
		ssCompData.cx2_sn = NULL;
		kfree(totCompData.ix_fm);
		totCompData.ix_fm = NULL;
		kfree(totCompData.ix_sn);
		totCompData.ix_sn = NULL;
		kfree(totCompData.cx_fm);
		totCompData.cx_fm = NULL;
		kfree(totCompData.cx_sn);
		totCompData.cx_sn = NULL;
		pr_info("SS IX CX testes finished!.................OK\n\n");
		return OK;
	} else {
	/* print all kind of data in just one row for readability reason */
		print_frame_u8("SS Init Data Ix2_fm = ", array1dTo2d_u8(
				       ssCompData.ix2_fm,
				       ssCompData.header.force_node, 1),
			       ssCompData.header.force_node, 1);
		print_frame_i8("SS Init Data Cx2_fm = ", array1dTo2d_i8(
				       ssCompData.cx2_fm,
				       ssCompData.header.force_node, 1),
			       ssCompData.header.force_node, 1);
		print_frame_u8("SS Init Data Ix2_sn = ", array1dTo2d_u8(
				       ssCompData.ix2_sn,
				       ssCompData.header.sense_node,
				       ssCompData.header.sense_node), 1,
			       ssCompData.header.sense_node);
		print_frame_i8("SS Init Data Cx2_sn = ", array1dTo2d_i8(
				       ssCompData.cx2_sn,
				       ssCompData.header.sense_node,
				       ssCompData.header.sense_node), 1,
			       ssCompData.header.sense_node);
		print_frame_u16("TOT SS Init Data Ix_fm = ", array1dTo2d_u16(
					totCompData.ix_fm,
					totCompData.header.force_node, 1),
				totCompData.header.force_node, 1);
		print_frame_short("TOT SS Init Data Cx_fm = ",
				  array1dTo2d_short(totCompData.cx_fm,
						    totCompData.header.
						    force_node, 1),
				  totCompData.header.force_node, 1);
		print_frame_u16("TOT SS Init Data Ix_sn = ", array1dTo2d_u16(
					totCompData.ix_sn,
					totCompData.header.sense_node,
					totCompData.header.sense_node), 1,
				totCompData.header.sense_node);
		print_frame_short("TOT SS Init Data Cx_sn = ",
				  array1dTo2d_short(totCompData.cx_sn,
						    totCompData.header.
						    sense_node,
						    totCompData.header.
						    sense_node),
				  1, totCompData.header.sense_node);
		pr_err("SS IX CX testes finished!.................FAILED  fails_count = %d\n\n",
			count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (adjhor != NULL)
			kfree(adjhor);
		if (adjvert != NULL)
			kfree(adjvert);
		if (total_adjhor != NULL)
			kfree(total_adjhor);
		if (total_adjvert != NULL)
			kfree(total_adjvert);
		if (ssCompData.ix2_fm != NULL)
			kfree(ssCompData.ix2_fm);
		if (ssCompData.ix2_sn != NULL)
			kfree(ssCompData.ix2_sn);
		if (ssCompData.cx2_fm != NULL)
			kfree(ssCompData.cx2_fm);
		if (ssCompData.cx2_sn != NULL)
			kfree(ssCompData.cx2_sn);
		if (totCompData.ix_fm != NULL)
			kfree(totCompData.ix_fm);
		if (totCompData.ix_sn != NULL)
			kfree(totCompData.ix_sn);
		if (totCompData.cx_fm != NULL)
			kfree(totCompData.cx_fm);
		if (totCompData.cx_sn != NULL)
			kfree(totCompData.cx_sn);
		return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (ssCompData.ix2_fm != NULL)
		kfree(ssCompData.ix2_fm);
	if (ssCompData.ix2_sn != NULL)
		kfree(ssCompData.ix2_sn);
	if (ssCompData.cx2_fm != NULL)
		kfree(ssCompData.cx2_fm);
	if (ssCompData.cx2_sn != NULL)
		kfree(ssCompData.cx2_sn);
	if (totCompData.ix_fm != NULL)
		kfree(totCompData.ix_fm);
	if (totCompData.ix_sn != NULL)
		kfree(totCompData.ix_sn);
	if (totCompData.cx_fm != NULL)
		kfree(totCompData.cx_fm);
	if (totCompData.cx_sn != NULL)
		kfree(totCompData.cx_sn);
	return ret;
}

/**
  * Perform a complete Data Test check of the IC
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_data(char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int res = OK, ret;

	if (todo == NULL) {
		pr_err("production_test_data: No TestToDo specified!! ERROR = %08X\n",
			(ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA));
		return ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA;
	}


	pr_info("DATA Production test is starting...\n");


	ret = production_test_ms_raw(path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		pr_err("production_test_data: production_test_ms_raw failed... ERROR = %08X\n",
			ret);
		if (stop_on_fail == 1)
			goto END;
	}



	ret = production_test_ms_cx(path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		pr_err("production_test_data: production_test_ms_cx failed... ERROR = %08X\n",
			ret);
		if (stop_on_fail == 1)
			goto END;
	}


	ret = production_test_ss_raw(path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		pr_err("production_test_data: production_test_ss_raw failed... ERROR = %08X\n",
			ret);
		if (stop_on_fail == 1)
			goto END;
	}

	ret = production_test_ss_ix_cx(path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		pr_err("production_test_data: production_test_ss_ix_cx failed... ERROR = %08X\n",
			ret);
		if (stop_on_fail == 1)
			goto END;
	}

END:
	freeLimitsFile(&limit_file);	/* /< release the limit file loaded
					 * during the test */
	if (res < OK)
		pr_err("DATA Production test failed!\n");
	else
		pr_info("DATA Production test finished!\n");
	return res;
}


/*************** TP Sensitivity calibration API ********************/

/**
  * Perform the Pre Calibration MS Test when the stimpad is down
  * @param[out] frame pointer to the frame which will contain
  * the average frame resulting from the test
  * @param target reference value for the frame, each node should be
  * around +-percentage% this value
  * @param percentage percentage of the target value which define
  * the valid interval for the frame, if <0 the test will be skipped
  * @return OK if success or an error code which specify the type of error
  */
int tp_sensitivity_test_pre_cal_ms(MutualSenseFrame *finalFrame, short target,
				   int percentage)
{
	int ret = OK;
	int count = 0, i = 0, j = 0;
	short min, max;
	MutualSenseFrame frame;

	finalFrame->node_data = NULL;


	pr_info("%s: Start TP sensitivity MS Pre Cal...\n", __func__);
	pr_info("%s: IMPORTANT!!! Stimpad should be on the display of the device!\n",
		__func__);
	ret = getMSFrame3(MS_STRENGTH, &frame);
	if (ret < OK) {
		pr_err("%s: can not read MS Frame... ERROR %08X\n",
			__func__, ret);
		goto ERROR;
	}

	finalFrame->header = frame.header;
	finalFrame->node_data_size = frame.node_data_size;

	finalFrame->node_data = (short *)kzalloc(frame.node_data_size *
						 sizeof(short), GFP_KERNEL);
	if (finalFrame->node_data == NULL) {
		pr_err("%s: can not allocate node_data ERROR %08X\n",
			__func__, ERROR_ALLOC | ERROR_GET_FRAME);
		ret = ERROR_ALLOC | ERROR_GET_FRAME;
		goto ERROR;
	}

	/* collecting frames */
	do {
		for (i = 0; i < finalFrame->node_data_size; i++) {
			finalFrame->node_data[i] += (frame.node_data[i] * 10) /
						    SENS_TEST_NUM_FRAMES;
		}

		if (frame.node_data != NULL) {
			kfree(frame.node_data);
			frame.node_data = NULL;
		}

		count++;

		/* exclude one more reading at the end*/
		if (count < SENS_TEST_NUM_FRAMES)
			ret = getMSFrame3(MS_STRENGTH, &frame);
	} while ((count < SENS_TEST_NUM_FRAMES) && (ret >= OK));

	if (ret < OK) {
		pr_err("%s: Error while capturing the frame %d! ERROR %08X\n",
			__func__, count, ret);
		goto ERROR;
	}

	ret = OK;
	/* check against +-percentage% target */
	pr_info("%s: Computing average frame...\n", __func__);

	min = target - (target * percentage / 100);
	max = target + (target * percentage / 100);

	for (i = 0; i < finalFrame->header.force_node; i++) {
		for (j = 0; j < finalFrame->header.sense_node; j++) {
			finalFrame->node_data[i *
					      finalFrame->header.sense_node +
					      j] /= 10;
		/*if percentage is <0 skip this test, just collect data */
			if ((percentage > 0) &&
			    ((finalFrame->node_data[i * finalFrame->header.
						    sense_node
						    + j] >
			      max) ||
			     (finalFrame->node_data[i *
						    finalFrame->header.
						    sense_node
						    + j] <
			      min))) {
				pr_err("%s: MS Force Node[%d, %d] = %d exceed limit [%d, %d]\n",
					__func__, i, j,
					finalFrame->node_data[i *
							       finalFrame
							       ->header.
							       sense_node + j],
					 min, max);
				ret = ERROR_TEST_CHECK_FAIL;
			}
		}
	}


	/* print average frame in the log */
	print_frame_short("MS FS Mean =",
			  array1dTo2d_short(
				  finalFrame->node_data,
				  finalFrame->node_data_size,
				  finalFrame->header.sense_node),
			  finalFrame->header.force_node,
			  finalFrame->header.sense_node);

	if (ret != OK)
		pr_err("%s: TP sensitivity MS Pre Cal test FAILED... ERROR %08X\n",
			__func__, ret);
	else
		pr_info("%s: TP sensitivity MS Pre Cal FINISHED!\n",
			__func__);

	return ret;


ERROR:
	if (frame.node_data != NULL) {
		kfree(frame.node_data);
		frame.node_data = NULL;
	}


	if (finalFrame->node_data != NULL) {
		kfree(finalFrame->node_data);
		finalFrame->node_data = NULL;
	}

	return ret;
}



/**
  * Perform the Pre Calibration SS Test when the stimpad is down
  * @param[out] frame pointer to the frame which will contain the average frame
  * resulting from the test
  * @param target reference value for the frame, each node should be around
  * +-percentage% this value
  * @param percentage percentage of the target value which define the valid
  * interval for the frame
  * @return OK if success or an error code which specify the type of error
  */
int tp_sensitivity_test_pre_cal_ss(SelfSenseFrame *finalFrame, short target,
				int percentage)
{
	int ret = OK;
	int count = 0, i = 0;
	short min, max;
	SelfSenseFrame frame;
	int *temp_force = NULL;
	int *temp_sense = NULL;

	finalFrame->force_data = NULL;
	finalFrame->sense_data = NULL;

	pr_info("%s: Start TP sensitivity SS Pre Cal...\n", __func__);
	pr_info("%s: IMPORTANT!!! Stimpad should be on the display of the device!\n",
		__func__);
	ret = getSSFrame3(SS_STRENGTH, &frame);
	if (ret < OK) {
		pr_err("%s: can not read SS Frame... ERROR %08X\n",
			__func__, ret);
		goto ERROR;
	}

	finalFrame->header = frame.header;

	finalFrame->force_data = (short *)kzalloc(frame.header.force_node *
						  sizeof(short), GFP_KERNEL);
	temp_force = (int *)kzalloc(frame.header.force_node *
						  sizeof(int), GFP_KERNEL);
	finalFrame->sense_data = (short *)kzalloc(frame.header.sense_node *
						  sizeof(short), GFP_KERNEL);
	temp_sense = (int *)kzalloc(frame.header.sense_node *
						  sizeof(int), GFP_KERNEL);
	if (finalFrame->force_data == NULL ||
	    temp_force == NULL ||
	    finalFrame->sense_data == NULL ||
	    temp_sense == NULL) {

		pr_err("%s: can not allocate memory ERROR %08X\n",
			__func__, ERROR_ALLOC | ERROR_GET_FRAME);
		ret = ERROR_ALLOC | ERROR_GET_FRAME;
		goto ERROR;
	}

	/* collecting frames */
	do {
		for (i = 0; i < finalFrame->header.force_node; i++)
			temp_force[i] += frame.force_data[i];

		for (i = 0; i < finalFrame->header.sense_node; i++)
			temp_sense[i] += frame.sense_data[i];

		count++;

		if (frame.force_data != NULL) {
			kfree(frame.force_data);
			frame.force_data = NULL;
		}
		if (frame.sense_data != NULL) {
			kfree(frame.sense_data);
			frame.sense_data = NULL;
		}

		/* exclude one more reading at the end*/
		if (count < SENS_TEST_NUM_FRAMES)
			ret = getSSFrame3(SS_STRENGTH, &frame);
	} while ((count < SENS_TEST_NUM_FRAMES) && (ret >= OK));

	if (ret < OK) {
		pr_err("%s: Error while capturing the frame %d! ERROR %08X\n",
			__func__, count, ret);
		goto ERROR;
	}

	ret = OK;

	/* compute the average and check against +-percentage% target */
	min = target - (target * percentage / 100);
	max = target + (target * percentage / 100);

	for (i = 0; i < finalFrame->header.force_node; i++) {
		finalFrame->force_data[i] = temp_force[i] /
						SENS_TEST_NUM_FRAMES;
		if ((percentage > 0) && ((finalFrame->force_data[i] > max) ||
					 (finalFrame->force_data[i] < min))) {
			pr_err("%s: SS Force Node[%d] = %d exceed limit [%d, %d]\n",
				__func__, i, finalFrame->force_data[i],
				min, max);
			ret = ERROR_TEST_CHECK_FAIL;
		}
	}

	for (i = 0; i < finalFrame->header.sense_node; i++) {
		finalFrame->sense_data[i] = temp_sense[i] /
						SENS_TEST_NUM_FRAMES;
		if ((finalFrame->sense_data[i] > max) ||
		    (finalFrame->sense_data[i] < min)) {
			pr_err("%s: SS Sense Node[%d] = %d exceed limit [%d, %d]\n",
				__func__, i, finalFrame->sense_data[i],
				min, max);
			ret = ERROR_TEST_CHECK_FAIL;
		}
	}

	/* print average frame in the log */
	print_frame_short("SS FS force Mean =",
			  array1dTo2d_short(
				  finalFrame->force_data,
				  finalFrame->header.force_node,
				  1),
			  finalFrame->header.force_node, 1);
	print_frame_short("SS FS sense Mean =",
			  array1dTo2d_short(
				  finalFrame->sense_data,
				  finalFrame->header.sense_node,
				  finalFrame->header.sense_node),
			  1, finalFrame->header.sense_node);


	kfree(temp_force);
	temp_force = NULL;

	kfree(temp_sense);
	temp_sense = NULL;

	if (ret < OK)
		pr_err("%s: TP sensitivity SS Pre Cal test FAILED... ERROR %08X\n",
			__func__, ret);
	else {
		pr_info("%s: TP sensitivity SS Pre Cal FINISHED!\n",
			__func__);
		ret = OK;
	}

	return ret;


ERROR:

	kfree(temp_force);
	temp_force = NULL;

	kfree(temp_sense);
	temp_sense = NULL;

	kfree(frame.force_data);
	frame.force_data = NULL;

	kfree(frame.sense_data);
	frame.sense_data = NULL;

	kfree(finalFrame->force_data);
	finalFrame->force_data = NULL;

	kfree(finalFrame->sense_data);
	finalFrame->sense_data = NULL;

	return ret;
}

/**
  * Compute Digital gains for calibration
  * @param frame pointer to the frame used as reference to compute the gains
  * @param target reference target value for computing the gains
  * @param saveGain if 1, will save the gain table into the chip otherwise will
  * not save it
  * @return OK if success or an error code which specify the type of error
  */
int tp_sensitivity_compute_gains(MutualSenseFrame *frame, short target,
				int saveGain)
{
	int ret = OK;
	int i = 0;
	u8 gains[frame->node_data_size];

	if ((frame->node_data == NULL) || (frame->node_data_size == 0)) {
		pr_err("%s: Invalid frame data passed as argument! ERROR %08X\n",
			__func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	memset(gains, 0, frame->node_data_size);

	pr_info("%s: Start to compute Digital Gains...\n", __func__);
	for (i = 0; i < frame->node_data_size; i++)
		gains[i] = ((target * 100) / frame->node_data[i]) > 255 ?
			   (u8)(255) : (u8)(((target * 100) /
					     frame->node_data[i]));
	/* clamp the max value to 255 because gain is only one byte */


	/* print average frame in the log */
	print_frame_u8("MS Digital Gain =",
		       array1dTo2d_u8(
			       gains,
			       frame->node_data_size,
			       frame->header.sense_node),
		       frame->header.force_node,
		       frame->header.sense_node);


	/* if(saveGain==1){ */
	/* write gains into the IC */
	ret = writeHostDataMemory(LOAD_SENS_CAL_COEFF, gains,
				  frame->header.force_node,
				  frame->header.sense_node, 0, 0, saveGain);
	if (ret != OK)
		pr_err("%s: impossible to write digital gains! ERROR %08X\n",
			__func__, ret);
	/* } */

	if (ret < OK)
		pr_err("%s: compute Digital Gains FAILED! ERROR %08X\n",
			__func__, ret);
	else {
		pr_info("%s: compute Digital Gains FINISHED!\n", __func__);
		ret = OK;
	}

	return ret;
}

/**
  * Perform the Post Calibration MS Test when the stimpad is down
  * @param[out] finalFrame pointer to the frame which will contain
  * the average frame resulting from the test
  * @param[out] deltas pointer to the frame which will contain
  * the FS Uniform frame (worst_neighborhood/mean)
  * @param target reference value for the frame, each node should be
  * around +-percentage% this value
  * @param percentage percentage of the target value which define
  * the valid interval for the frame, if <0 the test will be skipped
  * @param[out] mean_normal pointer to the variable which will contain the mean
  * of the normal area
  * @param[out] mean_edge pointer to the variable which will contain the mean of
  * the edge area
  * @return OK if success or an error code which specify the type of error
  */
int tp_sensitivity_test_post_cal_ms(MutualSenseFrame *finalFrame,
				    MutualSenseFrame *deltas, short target,
				    int percentage, int *mean_normal,
				    int *mean_edge)
{
	short currentNode;
	int final_force_num;
	int final_sense_num;
	short *final_node;
	int delta_sense_num;
	short *delta_node;
	short *delta;
	short adjNode;
	int ret = OK;
	int i = 0, j = 0, min, max;


	if ((finalFrame == NULL) || (deltas == NULL) || (mean_normal == NULL) ||
	    (mean_edge == NULL)) {
		pr_err("%s: Invalid arguments Passed! ERROR %08X\n",
			__func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*mean_normal = 0;
	*mean_edge = 0;

	finalFrame->node_data = NULL;
	deltas->node_data = NULL;

	pr_info("%s: Start TP sensitivity MS Post Cal...\n", __func__);
	pr_info("%s: IMPORTANT!!! Stimpad should be on the display of the device!\n",
		__func__);

	/* collect frames skipping the tests + print on the log */
	ret = tp_sensitivity_test_pre_cal_ms(finalFrame, target, -1);
	if (ret < OK) {
		pr_err("%s: can not collect MS Frame... ERROR %08X\n",
			__func__, ret);
		goto ERROR;
	}


	deltas->header = finalFrame->header;
	deltas->node_data_size = finalFrame->node_data_size;

	deltas->node_data = (short *)kzalloc(deltas->node_data_size *
					     sizeof(short), GFP_KERNEL);
	if (deltas->node_data == NULL) {
		pr_err("%s: can not allocate deltas node_data ERROR %08X\n",
			__func__, ERROR_ALLOC | ERROR_GET_FRAME);
		ret = ERROR_ALLOC | ERROR_GET_FRAME;
		goto ERROR;
	}

	/* compute the average of the whole panel and check against
	  * +-percentage% target */
	pr_info("%s: Computing average of whole panel and delta for each node...\n",
		__func__);

	final_force_num = finalFrame->header.force_node;
	final_sense_num = finalFrame->header.sense_node;
	final_node = finalFrame->node_data;
	delta_sense_num = deltas->header.sense_node;
	delta_node = deltas->node_data;


	for (i = 0; i < final_force_num; i++) {
		for (j = 0; j < final_sense_num; j++) {
			currentNode = finalFrame->node_data[i *
							    finalFrame->header.
							    sense_node + j];
			delta = &delta_node[i * delta_sense_num + j];

			if ((i == 0) ||
			    (i == (final_force_num - 1)) ||
			    (j == 0) ||
			    (j == (final_sense_num - 1))) {
				/* edge nodes */
				*mean_edge += currentNode;
				if ((i == 0) ||
				    (i == final_force_num - 1)) {
					/* need to check adj node up or down for
					  *  nodes in the corners */
					if ((i == 0) &&
					    ((j == 0) ||
					     (j == final_sense_num - 1))) {
						adjNode = currentNode -
							  final_node[(i + 1) *
							   final_sense_num + j];
						if (abs(adjNode) > *delta)
							*delta = abs(adjNode);
					}

					if ((i == (final_force_num - 1)) &&
					    ((j == 0) ||
					     (j == final_sense_num - 1))) {
						adjNode = currentNode -
							  final_node[(i - 1) *
							   final_sense_num + j];
						if (abs(adjNode) > *delta)
							*delta = abs(adjNode);
					}

					/* scan the row */
					if ((j - 1) >= 0) {
						adjNode = currentNode -
							  final_node[i *
							    final_sense_num +
							    (j - 1)];
						if (abs(adjNode) > *delta)
							*delta = abs(adjNode);
					}

					if ((j + 1) < final_sense_num) {
						adjNode = currentNode -
							  final_node[i *
							    final_sense_num +
							    (j + 1)];
						if (abs(adjNode) > *delta)
						    *delta = abs(adjNode);
					}
				}

				if ((j == 0) ||
				    (j == final_sense_num - 1)) {
					/* scan the column */
					if ((i - 1) >= 0) {
						adjNode = currentNode -
							  final_node[(i - 1) *
							   final_sense_num + j];
						if (abs(adjNode) > *delta)
							*delta = abs(adjNode);
					}

					if ((i + 1) < final_force_num) {
						adjNode = currentNode -
							  final_node[(i + 1) *
							   final_sense_num + j];
						if (abs(adjNode) > *delta)
							*delta = abs(adjNode);
					}
				}
			} else {
				/*normal nodes */
				*mean_normal += currentNode;

				/* picking up the worst difference between
				  * one pixel and its neighbors */
				if ((i - 1) >= 1) {
					adjNode = currentNode -
						  final_node[(i - 1) *
							final_sense_num + j];
					if (abs(adjNode) > *delta)
						*delta = abs(adjNode);
				}

				if ((i + 1) < (final_force_num - 1)) {
					adjNode = currentNode -
						  final_node[(i + 1) *
							final_sense_num + j];
					if (abs(adjNode) > *delta)
						*delta = abs(adjNode);
				}
				if ((j - 1) >= 1) {
					adjNode = currentNode -
						  final_node[i *
						    final_sense_num + (j - 1)];
					if (abs(adjNode) > *delta)
						*delta = abs(adjNode);
				}

				if ((j + 1) < (final_sense_num - 1)) {
					adjNode = currentNode -
						  final_node[i *
						    final_sense_num + (j + 1)];
					if (abs(adjNode) > *delta)
						*delta = abs(adjNode);
				}
			}
		}
	}

	*mean_normal /= (finalFrame->header.force_node - 2) *
			(finalFrame->header.sense_node - 2);
	*mean_edge /= (finalFrame->header.force_node * 2) +
		      (finalFrame->header.sense_node - 2) * 2;

	pr_info("%s: Normal Frame average = %d\n", __func__, *mean_normal);
	pr_info("%s: Edge Frame average = %d\n", __func__, *mean_edge);
	/* compute the average and check against +-% target */
	min = target - (target * percentage / 100);
	max = target + (target * percentage / 100);

	if ((percentage > 0) && ((*mean_normal < min) || (*mean_normal >
							  max))) {
		pr_err("%s: Normal Frame average = %d exceed limit [%d, %d]\n",
			__func__, *mean_normal, min, max);
		ret = ERROR_TEST_CHECK_FAIL;
	}

	if ((percentage > 0) && ((*mean_edge < min) || (*mean_edge > max))) {
		pr_err("%s: Edge Frame average = %d exceed limit [%d, %d]\n",
			__func__, *mean_edge, min, max);
		ret = ERROR_TEST_CHECK_FAIL;
	}

	for (i = 0; i < deltas->header.force_node; i++) {
		for (j = 0; j < deltas->header.sense_node; j++) {
			if ((i == 0) || (i == deltas->header.force_node) ||
			    (j == 0) || (j == deltas->header.sense_node))
				deltas->node_data[i *
						  deltas->header.sense_node +
						  j] =
					deltas->node_data[i *
							  deltas->header.
							  sense_node + j] *
					100 /
					(*mean_edge);
			else
				deltas->node_data[i *
						  deltas->header.sense_node +
						  j] =
					deltas->node_data[i *
							  deltas->header.
							  sense_node + j] *
					100 /
					(*mean_normal);

			if ((percentage > 0) && (deltas->node_data[i *
								   deltas->
								   header.
								   sense_node +
								   j] >
						 percentage)) {
				pr_err("%s: Delta Node[%d, %d] = %d exceed limit [%d]\n",
					__func__, i, j,
					deltas->node_data[i *
							   deltas
							   ->header.sense_node +
							   j], percentage);
				ret = ERROR_TEST_CHECK_FAIL;
			}
		}
	}


	/* print average frame in the log */
	print_frame_short("FS Uniform (%) =",
			  array1dTo2d_short(
				  deltas->node_data,
				  deltas->node_data_size,
				  deltas->header.sense_node),
			  deltas->header.force_node,
			  deltas->header.sense_node);


	if (ret < OK)
		pr_err("%s: TP sensitivity MS Post Cal test FAILED... ERROR %08X\n",
			__func__, ret);
	else {
		pr_info("%s: TP sensitivity MS Post Cal FINISHED!\n",
			__func__);
		ret = OK;
	}

	return ret;


ERROR:
	if (deltas->node_data != NULL) {
		kfree(deltas->node_data);
		deltas->node_data = NULL;
	}


	if (finalFrame->node_data != NULL) {
		kfree(finalFrame->node_data);
		finalFrame->node_data = NULL;
	}

	return ret;
}


/**
  * Compute Digital gains for calibration
  * @param enter if =1 turn on TP Sensitivity mode, otherwise will turn it off
  * @param saveGain if 1, will save the gain table into the chip otherwise will
  * not save it
  * @return OK if success or an error code which specify the type of error
  */
int tp_sensitivity_mode(u8 enter, int saveGain)
{
	int res, ret = OK;
	u8 cmd[4] = { 0xC0, 0x00, 0x00, 0x00 };
	u8 sett = SPECIAL_WRITE_HOST_MEM_TO_FLASH;

	pr_info("%s: Start TP Sensitivity Mode... enter = %02X\n",
		 __func__, enter);
	if (enter == 1) {
		/* enter TP Sensitivity mode*/
		ret = fts_disableInterrupt();
		pr_info("%s: Entering TP Sensitivity Mode disabling algos...\n",
			__func__);
		cmd[3] = 0x01;
		res = fts_writeFwCmd(cmd, 4);
		if (res < OK)
			pr_err("%s: Error while turning on TP Sens Mode! ERROR %08X\n",
				__func__, res);
	} else {
		/* exit TP Sensitivity mode*/
		pr_info("%s: Exiting TP Sensitivity Mode enabling algos...\n",
			__func__);
		res = fts_writeFwCmd(cmd, 4);
		if (res < OK)
			pr_err("%s: Error while turning off TP Sens Mode! ERROR %08X\n",
				__func__, res);

		if (saveGain == 1) {
			pr_info("%s: Trigger writing gains into the flash...\n",
				__func__);
			ret = writeSysCmd(SYS_CMD_SPECIAL, &sett, 1);
			if (ret < OK)
				pr_err("%s: error while writing gains into the flash! ERROR %08X\n",
					__func__, res);
		}

		res |= senseOn();
		res |= fts_enableInterrupt();
	}

	res |= ret;

	if (res < OK)
		pr_err("%s: TP Sensitivity Mode... ERROR %08X!\n",
			__func__, res);
	else
		pr_info("%s: TP Sensitivity Mode FINISHED!\n", __func__);

	return res;
}


/**
  * Compute Digital gains for calibration
  * @param scan select the scan mode which should be enabled
  * @param enableGains =1 apply gains when computing the strength otherwise
  * the gains will be ignored
  * @return OK if success or an error code which specify the type of error
  */
int tp_sensitivity_set_scan_mode(u8 scan, int enableGains)
{
	int res, ret = OK;
	u8 cmd[4] = { 0xC0, 0x00, 0x01, 0x00 };


	pr_info("%s: Set TP Sensitivity Scan Mode... scan = %02X, enableGains = %d\n",
		__func__, scan, enableGains);


	if (enableGains == 1) {
		/* Consider Sensitivity Gains when computing Strength */
		cmd[3] = 0x01;
		ret = fts_writeFwCmd(cmd, 4);
		if (ret < OK)
			pr_err("%s: Error while enabling Gains in TP Sens Mode! ERROR %08X\n",
				__func__, ret);
	} else {
		/* Exclude Sensitivity Gains when computing Strength */
		ret = fts_writeFwCmd(cmd, 4);
		if (ret < OK)
			pr_err("%s: Error while disabling Gain in TP Sens Mode! ERROR %08X\n",
				__func__, ret);
	}

	res = setScanMode(SCAN_MODE_LOCKED, scan);
	if (res < OK)
		pr_err("Error while setting the scan frequency... ERROR %08X\n",
			res);

	res |= ret;

	if (res < OK)
		pr_err("%s: Set TP Sensitivity Scan Mode... ERROR %08X!\n",
			__func__, res);
	else
		pr_info("%s: Set TP Sensitivity Scan FINISHED!\n", __func__);

	return res;
}




/**
  * Compute the standard deviation for each node form a series of frames
  * @param numFrames number of frames to collect to compute the standard
  * deviation
  * @param[out] std pointer to the frame which will contain the standard
  * deviation for each node
  * @return OK if success or an error code which specify the type of error
  */
int tp_sensitivity_test_std_ms(int numFrames, MutualSenseFrame *std)
{
	int ret = OK;
	int i = 0, count = 0;
	MutualSenseFrame frame;
	int *mean = NULL;/* store the mean value for each node */
	unsigned long *stdTemp = NULL;


	if (std == NULL) {
		pr_err("%s: Invalid arguments Passed! ERROR %08X\n",
			__func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	std->node_data = NULL;

	pr_info("%s: Start TP sensitivity STD... collecting %d frames!\n",
		__func__, numFrames);

	/* collect frames skipping the tests + print on the log */
	ret = getMSFrame3(MS_STRENGTH, &frame);
	if (ret < OK) {
		pr_err("%s: can not read MS Frame... ERROR %08X\n",
			__func__, ret);
		goto ERROR;
	}

	std->header = frame.header;
	std->node_data_size = frame.node_data_size;

	std->node_data = (short *)kzalloc(std->node_data_size * sizeof(short),
					  GFP_KERNEL);
	mean = (int *)kzalloc(std->node_data_size * sizeof(int), GFP_KERNEL);
	stdTemp = (unsigned long *)kzalloc(std->node_data_size *
					   sizeof(unsigned long),
					   GFP_KERNEL);
	if (std->node_data == NULL ||
	    mean == NULL ||
	    stdTemp == NULL) {
		pr_err("%s: can not allocate memory ERROR %08X\n",
			__func__, ERROR_ALLOC | ERROR_GET_FRAME);
		ret = ERROR_ALLOC | ERROR_GET_FRAME;
		goto ERROR;
	}

	/* collecting frames */
	do {
		for (i = 0; i < frame.node_data_size; i++) {
			mean[i] += frame.node_data[i];
			stdTemp[i] += frame.node_data[i] * frame.node_data[i];
		}
		count++;

		if (frame.node_data != NULL) {
			kfree(frame.node_data);
			frame.node_data = NULL;
		}

		/* exclude one more reading at the end*/
		if (count < numFrames)
			ret = getMSFrame3(MS_STRENGTH, &frame);
	} while ((count < numFrames) && (ret >= OK));

	if (ret < OK) {
		pr_err("%s: error while collecting the frames! ERROR%08X\n",
			__func__, ret);
		goto ERROR;
	}

	/* compute the average for each node */
	pr_info("%s: Computing std for each node...\n", __func__);

	for (i = 0; i < std->node_data_size; i++) {
		mean[i] /= numFrames;
		stdTemp[i] = stdTemp[i] / numFrames - (mean[i] * mean[i]);
		std->node_data[i] = (short)int_sqrt(stdTemp[i]);
	}

	kfree(stdTemp);
	stdTemp = NULL;
	kfree(mean);
	mean = NULL;

	/* print average frame in the log */
	print_frame_short("STD =",
			  array1dTo2d_short(
				  std->node_data,
				  std->node_data_size,
				  std->header.sense_node),
			  std->header.force_node,
			  std->header.sense_node);

	if (ret < OK)
		pr_err("%s: TP sensitivity STD test FAILED... ERROR %08X\n",
			__func__, ret);
	else {
		pr_info("%s: TP sensitivity STD FINISHED!\n",
			__func__);
		ret = OK;
	}

	return ret;

ERROR:

	kfree(frame.node_data);
	frame.node_data = NULL;

	kfree(std->node_data);
	std->node_data = NULL;

	kfree(stdTemp);
	stdTemp = NULL;

	kfree(mean);
	mean = NULL;

	return ret;
}


/**
  * Retrieve the actual Test Limit data from the system (bin file or header
  * file)
  * @param path name of Production Test Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param file pointer to the LimitFile struct which will contains the limits
  * data
  * @return OK if success or an error code which specify the type of error
  */
int getLimitsFile(char *path, LimitFile *file)
{
	const struct firmware *fw = NULL;
	struct device *dev = NULL;
	int fd = -1;

	pr_info("Get Limits File starting... %s\n", path);

	if (file->data != NULL) {
		/* to avoid memory leak on consecutive call of
		 * the function with the same pointer */
		pr_err("Pointer to Limits Data already contains something... freeing its content!\n");
		kfree(file->data);
		file->data = NULL;
		file->size = 0;
	}

	strlcpy(file->name, path, MAX_LIMIT_FILE_NAME);
	if (strncmp(path, "NULL", 4) == 0) {
#ifdef LIMITS_H_FILE
		pr_info("Loading Limits File from .h!\n");
		file->size = LIMITS_SIZE_NAME;
		file->data = (char *)kmalloc((file->size) * sizeof(char),
					     GFP_KERNEL);
		if (file->data != NULL) {
			memcpy(file->data, (char *)(LIMITS_ARRAY_NAME),
			       file->size);
			return OK;
		} else {
			pr_err("Error while allocating data... ERROR %08X\n",
				path, ERROR_ALLOC);
			return ERROR_ALLOC;
		}
#else
		pr_err("limit file path NULL... ERROR %08X\n",
			 ERROR_FILE_NOT_FOUND);
		return ERROR_FILE_NOT_FOUND;
#endif
	} else {
		dev = getDev();
		if (dev != NULL) {
			pr_info("Loading Limits File from .csv!\n");
			fd = request_firmware(&fw, path, dev);
			if (fd == 0) {
				pr_info("Start to copy %s...\n", path);
				file->size = fw->size;
				file->data = (char *)kmalloc((file->size) *
							     sizeof(char),
							     GFP_KERNEL);
				if (file->data != NULL) {
					memcpy(file->data, (char *)fw->data,
					       file->size);
					pr_info("Limit file Size = %d\n",
						file->size);
					release_firmware(fw);
					return OK;
				} else {
					pr_err("Error while allocating data... ERROR %08X\n",
						ERROR_ALLOC);
					release_firmware(fw);
					return ERROR_ALLOC;
				}
			} else {
				pr_err("Request the file %s failed... ERROR %08X\n",
					path, ERROR_FILE_NOT_FOUND);
				return ERROR_FILE_NOT_FOUND;
			}
		} else {
			pr_err("Error while getting the device ERROR %08X\n",
				ERROR_FILE_READ);
			return ERROR_FILE_READ;
		}
	}
}

/**
  * Reset and release the memory which store a Production Limit File previously
  * loaded
  * @param file pointer to the LimitFile struct to free
  * @return OK if success or an error code which specify the type of error
  */

int freeLimitsFile(LimitFile *file)
{
	pr_info("Freeing Limit File ...\n");
	if (file != NULL) {
		if (file->data != NULL) {
			kfree(file->data);
			file->data = NULL;
		} else
			pr_err("Limit File was already freed!\n");
		file->size = 0;
		strlcpy(file->name, " ", MAX_LIMIT_FILE_NAME);
		return OK;
	} else {
		pr_err("Passed a NULL argument! ERROR %08X\n",
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}

/**
  * Reset and release the memory which store the current Limit File
  * previously loaded
  * @return OK if success or an error code which specify the type of error
  */
int freeCurrentLimitsFile(void)
{
	return freeLimitsFile(&limit_file);
}

/**
  * Parse the raw data read from a Production test limit file in order
  * to find the specified information
  * If no limits file data are passed, the function loads and stores the limit
  * file from the system
  * @param path name of Production Test Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param file pointer to LimitFile struct that should be parsed or
  * NULL if the limit file in the system should be loaded and then parsed
  * @param label string which identify a particular set of data in the file that
  * want to be loaded
  * @param data pointer to the pointer which will contains the specified limits
  * data
  * as 1 dimension matrix with data arranged row after row
  * @param row pointer to a int variable which will contain the number of row of
  * data
  * @param column pointer to a int variable which will contain the number of
  * column of data
  * @return OK if success or an error code which specify the type of error
  */
int parseProductionTestLimits(char *path, LimitFile *file, char *label,
			      int **data, int *row, int *column)
{
	int find = 0;
	char *token = NULL;
	int i = 0;
	int j = 0;
	int z = 0;

	char *line2 = NULL;
	char line[800];
	char *buf = NULL;
	int n, size, pointer = 0, ret = OK;
	char *data_file = NULL;

	if (file == NULL || strcmp(path, file->name) != 0 || file->size == 0) {
		pr_info("No limit File data passed... try to get them from the system!\n");
		ret = getLimitsFile(LIMITS_FILE, &limit_file);
		if (ret < OK) {
			pr_err("parseProductionTestLimits: ERROR %08X\n",
				ERROR_FILE_NOT_FOUND);
			return ERROR_FILE_NOT_FOUND;
		}
		size = limit_file.size;
		data_file = limit_file.data;
	} else {
		pr_info("Limit File data passed as arguments!\n");
		size = file->size;
		data_file = file->data;
	}



	pr_info("The size of the limits file is %d bytes...\n", size);



	while (find == 0) {
		/* start to look for the wanted label */
		if (readLine(&data_file[pointer], line, size - pointer, &n) <
		    0) {
			find = -1;
			break;
		}
		pointer += n;
		if (line[0] == '*') {
		/* each header row start with * ex. *label,n_row,n_colum */
			line2 = kstrdup(line, GFP_KERNEL);
			if (line2 == NULL) {
				pr_err("parseProductionTestLimits: kstrdup ERROR %08X\n",
					ERROR_ALLOC);
				ret = ERROR_ALLOC;
				goto END;
			}
			buf = line2;
			line2 += 1;
			token = strsep(&line2, ",");
			if (strcmp(token, label) == 0) {
				/* if the row is the wanted one, r
				 * retrieve rows and columns info */
				find = 1;
				token = strsep(&line2, ",");
				if (token != NULL) {
					sscanf(token, "%d", row);
					pr_info("Row = %d\n", *row);
				} else {
					pr_err("parseProductionTestLimits 1: ERROR %08X\n",
						ERROR_FILE_PARSE);
					ret = ERROR_FILE_PARSE;
					goto END;
				}
				token = strsep(&line2, ",");
				if (token != NULL) {
					sscanf(token, "%d", column);
					pr_info("Column = %d\n", *column);
				} else {
					pr_err("parseProductionTestLimits 2: ERROR %08X\n",
						ERROR_FILE_PARSE);
					ret = ERROR_FILE_PARSE;
					goto END;
				}

				kfree(buf);
				buf = NULL;
				*data = (int *)kmalloc(((*row) * (*column)) *
						       sizeof(int), GFP_KERNEL);
			/* allocate the memory for containing the data */
				j = 0;
				if (*data == NULL) {
					pr_err("parseProductionTestLimits: ERROR %08X\n",
						ERROR_ALLOC);
					ret = ERROR_ALLOC;
					goto END;
				}


				/* start to read the data */
				for (i = 0; i < *row; i++) {
					if (readLine(&data_file[pointer], line,
						     size - pointer, &n) < 0) {
						pr_err("parseProductionTestLimits : ERROR %08X\n",
							ERROR_FILE_READ);
						ret = ERROR_FILE_READ;
						goto END;
					}
					pointer += n;
					line2 = kstrdup(line, GFP_KERNEL);
					if (line2 == NULL) {
						pr_err("parseProductionTestLimits: kstrdup ERROR %08X\n",
							ERROR_ALLOC);
						ret = ERROR_ALLOC;
						goto END;
					}
					buf = line2;
					token = strsep(&line2, ",");
					for (z = 0; (z < *column) && (token !=
								      NULL);
					     z++) {
						sscanf(token, "%d", ((*data) +
								     j));
						j++;
						token = strsep(&line2, ",");
					}
					kfree(buf);
					buf = NULL;
				}
				if (j == ((*row) * (*column))) {
					/* check that all the data are read */
					pr_info("READ DONE!\n");
					ret = OK;
					goto END;
				}
				pr_err("parseProductionTestLimits 3: ERROR %08X\n",
					ERROR_FILE_PARSE);
				ret = ERROR_FILE_PARSE;
				goto END;
			}
			kfree(buf);
			buf = NULL;
		}
	}
	pr_err("parseProductionTestLimits: ERROR %08X\n",
		 ERROR_LABEL_NOT_FOUND);
	ret = ERROR_LABEL_NOT_FOUND;
END:
	if (buf != NULL)
		kfree(buf);
	return ret;
}


/**
  * Read one line of a text file passed as array of byte and terminate it with
  * a termination character '\0'
  * @param data text file as array of bytes
  * @param line pointer to an array of char that will contain the line read
  * @param size size of data
  * @param n pointer to a int variable which will contain the number of
  * characters of the line
  * @return OK if success or an error code which specify the type of error
  */
int readLine(char *data, char *line, int size, int *n)
{
	int i = 0;

	if (size < 1)
		return ERROR_OP_NOT_ALLOW;

	while (data[i] != '\n' && i < size) {
		line[i] = data[i];
		i++;
	}
	*n = i + 1;
	line[i] = '\0';

	return OK;
}

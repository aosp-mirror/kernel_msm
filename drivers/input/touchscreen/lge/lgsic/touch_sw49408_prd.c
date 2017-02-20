/* production_test.c
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define TS_MODULE "[prd]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
//#include <soc/qcom/lge/board_lge.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw49408.h"
#include "touch_sw49408_prd.h"

static char line[50000];
static char W_Buf[BUF_SIZE];
//static char tmpbuf[BUF_SIZE];
static u16 tmpbuf[ROW_SIZE*COL_SIZE];
static u16 M2_Rawdata_buf[ROW_SIZE*COL_SIZE];
static u16 M1_Rawdata_buf[ROW_SIZE*M1_COL_SIZE];
//static u16 M1_Rawdata_buf[ROW_SIZE*COL_SIZE];
static u16 LowerImage[ROW_SIZE][COL_SIZE];
static u16 UpperImage[ROW_SIZE][COL_SIZE];

static void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n",
				__func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n",
			__func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n",
						__func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n",
							__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d",
							fname,
							(i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
						goto error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
	return;
}
static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[TIME_STR_LEN] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&my_date);
			snprintf(time_string, TIME_STR_LEN,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static int write_test_mode(struct device *dev, u32 type)
{
	u32 testmode = 0;
	int retry = 20;
	u32 rdata = 0x01;
	int waiting_time = 400;
	struct sw49408_data *d = to_sw49408_data(dev);

	testmode = type;
	if (type == U3_OPEN_NODE_TEST)
		waiting_time = 10;
	else if (type == U3_SHORT_NODE_TEST)
		waiting_time = 1000;
	else if (type == U0_M1_NOISE_TEST || type == U0_M2_NOISE_TEST ||
		type == U3_M1_NOISE_TEST || type == U3_M2_NOISE_TEST ||
		type == M1_DIFF_TEST || type == M2_DIFF_TEST)
		waiting_time = 500;

	/* TestType Set */
	sw49408_reg_write(dev, d->reg_info.r_tc_cmd_spi_addr + tc_tsp_test_ctl,
			(u8 *)&testmode,
			sizeof(testmode));
	TOUCH_I("write testmode = 0x%x\n", testmode);
	touch_msleep(waiting_time);

	/* Check Test Result - wait until 0 is written */
	do {
		touch_msleep(100);
		sw49408_reg_read(dev, d->reg_info.r_tc_sts_spi_addr + pt_sts_off,
				(u8 *)&rdata,
				sizeof(rdata));
		TOUCH_I("rdata = 0x%x\n", rdata);
	} while ((rdata != 0xAA) && retry--);

	if (rdata != 0xAA) {
		TOUCH_I("ProductionTest Type [%x] Time out\n", type);
		goto error;
	}
	return 1;
error:
	TOUCH_E("[%s] fail\n", __func__);
	return 0;
}

static int prd_os_xline_result_read(struct device *dev,
	u16 (*buf)[COL_SIZE], int type)
{
	int i = 0;
	int j = 0;
	u16 buffer[ROW_SIZE*COL_SIZE];
	int cur = 0;
	int ret = 0;
	u8 w_val = 0x0;
	int row_size;
	int col_size;
    struct sw49408_data *d = to_sw49408_data(dev);

	sw49408_write_value(dev, spr_data_offset,
		PT_FRAME_4_OFFSET);

	sw49408_reg_read(dev, data_access_addr,
		(u16 *)buffer, sizeof(u16)*ROW_SIZE*COL_SIZE);

	switch (type) {
	case U3_OPEN_NODE_TEST:
		w_val = 0x1;
		row_size = ROW_SIZE;
		col_size = COL_SIZE;
		break;
	case U3_SHORT_NODE_TEST:
		w_val = 0x2;
		// for row - column swap.
		row_size = COL_SIZE;
		col_size = ROW_SIZE;
		break;
	}

	if (ret == 0) {
		for (i = 0; i < row_size; i++) {
			for (j = 0; j < col_size; j++) {
				if (buffer[cur] != 0) {
					if(type == U3_OPEN_NODE_TEST)
						buf[i][j] = (buf[i][j] | w_val);
					else
						buf[j][i] = (buf[j][i] | w_val);
				}
				cur++;
			}
		}
	}

	return ret;
}

/*
 * Frame1 : M1 Raw Data, M2 Raw Data, Open1 Data, Short Data
 * Frame2 : Open2, P2P-1 Noise, Delta Jitter Max
 * Frame3 : Open3, Delta Jitter Min
 * Fraem4 : O/S Result, P2P-2 Noise, AVG Jitter
 */

static int read_pt_frame(struct device *dev, struct sw49408_data *d, u32 type, u8 frame_type)
{
	u32 frame_size = 0;
	int is_flip_x_y_data = 0;
	int x_size = 0;
	int y_size = 0;
	int cur = 0;
	int x, y;
	u16* buf_ptr;

	TOUCH_I("\n%s : Frame_%d\n", __func__, frame_type);

	/* Frame Size definition */
	/* Don't change order of case. it will have an effect on case logic. */
	switch(type){
	case U3_OPEN_NODE_TEST:
	case U3_M2_NOISE_TEST:
	case U0_M2_RAWDATA_TEST:
	case U0_M2_NOISE_TEST:
		if(frame_type == 4){
			x_size = ROW_SIZE;
			y_size = COL_SIZE;
			buf_ptr = M2_Rawdata_buf;
			memset((u16*)buf_ptr, 0x0, ROW_SIZE*COL_SIZE*sizeof(u16));
			break;
		}
	case U3_M2_DELTA_JITTER:
		if(frame_type == 4){
			x_size = M1_COL_SIZE;
			y_size = ROW_SIZE;
			is_flip_x_y_data = 1;
			buf_ptr = M2_Rawdata_buf;
			memset((u16*)buf_ptr, 0x0, ROW_SIZE*M1_COL_SIZE*sizeof(u16));
			break;
		}
	case U3_M2_RAWDATA_TEST:
	case U2_M2_RAWDATA_TEST:
		x_size = COL_SIZE;
		y_size = ROW_SIZE;
		buf_ptr = M2_Rawdata_buf;
		memset((u16*)buf_ptr, 0x0, ROW_SIZE*COL_SIZE*sizeof(u16));
		break;
	case U3_M1_RAWDATA_TEST:
	case U3_M1_NOISE_TEST:
		x_size = M1_COL_SIZE;
		y_size = ROW_SIZE;
		is_flip_x_y_data = 1;
		buf_ptr = M1_Rawdata_buf;
		memset((u16*)buf_ptr, 0x0, ROW_SIZE*M1_COL_SIZE*sizeof(u16));
		break;
	case U0_M1_RAWDATA_TEST:
		is_flip_x_y_data = 1;
	case U0_M1_NOISE_TEST:
		x_size = M1_COL_SIZE;
		y_size = ROW_SIZE;
		buf_ptr = M1_Rawdata_buf;
		memset((u16*)buf_ptr, 0x0, ROW_SIZE*M1_COL_SIZE*sizeof(u16));
		break;
	default:
		TOUCH_I("Not defined Test Type!\n");
		return 1;
	}


	frame_size = x_size * y_size * sizeof(u16);

	switch(frame_type){
		case PT_FRAME_1:
			sw49408_write_value(dev, spr_data_offset,PT_FRAME_1_OFFSET);
			break;
		case PT_FRAME_2:
			sw49408_write_value(dev, spr_data_offset,PT_FRAME_2_OFFSET);
			break;
		case PT_FRAME_3:
			sw49408_write_value(dev, spr_data_offset,PT_FRAME_3_OFFSET);
			break;
		case PT_FRAME_4:
			sw49408_write_value(dev, spr_data_offset,PT_FRAME_4_OFFSET);
			break;
		default :
			TOUCH_I("Not defined Frame_Type! \n");
			return 1;
	}

	if(is_flip_x_y_data){
		sw49408_reg_read(dev, data_access_addr, (u32*)tmpbuf, frame_size * sizeof(u16));
		memset(buf_ptr, 0x0, frame_size);
		for(x = 0; x < x_size; x++) {
			for(y = 0; y < y_size; y++) {
				buf_ptr[y * x_size + x] = tmpbuf[cur];
				cur++;
			}
		}
	} else
		sw49408_reg_read(dev, data_access_addr, (u32*)buf_ptr, frame_size * sizeof(u16));


	return frame_size;
}


static int prd_open_short_test(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int type = 0;
	int ret = 0;
	int write_test_mode_result = 0;
	u32 open_result = 0;
	u32 short_result = 0;
	u32 openshort_all_result = 0;
	u16 buf[ROW_SIZE][COL_SIZE];
	int i = 0;
	int j = 0;

	/* Test Type Write */
	write_file(dev, "[OPEN_SHORT_ALL_TEST]\n", TIME_INFO_SKIP);

	memset(&buf, 0x0, sizeof(buf));

	/* 1. open_test */
	type = U3_OPEN_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	sw49408_reg_read(dev, d->reg_info.r_tc_sts_spi_addr + pt_result_off,
			(u8 *)&open_result, sizeof(open_result));
	TOUCH_I("open_result = %d\n", open_result);

	if (open_result) {
		ret = prd_os_xline_result_read(dev, buf, type);
		openshort_all_result |= 0x1;
	}

	/* 2. short_test */
	type = U3_SHORT_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	sw49408_reg_read(dev, d->reg_info.r_tc_sts_spi_addr + pt_result_off,
		(u8 *)&short_result, sizeof(short_result));
	TOUCH_I("short_result = %d\n", short_result);

	if (short_result) {
		ret = prd_os_xline_result_read(dev, buf, type);
		openshort_all_result |= 0x2;
	}

	/* fail case */
	if (openshort_all_result != 0) {
		ret = snprintf(W_Buf, BUF_SIZE, "\n   : ");
		for (i = 0; i < COL_SIZE; i++)
			ret += snprintf(W_Buf + ret,
					BUF_SIZE - ret,
					" [%2d] ", i);

		for (i = 0; i < ROW_SIZE; i++) {
			ret += snprintf(W_Buf + ret,
					BUF_SIZE - ret,
					"\n[%2d] ", i);
			for (j = 0; j < COL_SIZE; j++) {
				ret += snprintf(W_Buf + ret,
					BUF_SIZE - ret, "%5s ",
				((buf[i][j] & 0x3) == 0x3) ?  "O,S" :
				((buf[i][j] & 0x1) == 0x1) ?  "O" :
				((buf[i][j] & 0x2) == 0x2) ?  "S" : "-");
			}
		}
		ret += snprintf(W_Buf + ret, BUF_SIZE - ret, "\n");
	} else
		ret = snprintf(W_Buf + ret, BUF_SIZE - ret,
				"OPEN_SHORT_ALL_TEST : Pass\n");

	write_file(dev, W_Buf, TIME_INFO_SKIP);

	return openshort_all_result;
}

static int prd_print_rawdata(struct device *dev, char *buf, u32 type,
				int pt_num, int* ret)
{
	int i = 0, j = 0;
	int min = 9999;
	int max = 0;
	u16 *rawdata_buf = NULL;
	int col_size = 0;

	/* print a frame data */
	*ret = snprintf(buf + *ret, PAGE_SIZE, "\n   : ");
	if (pt_num >= 0 && pt_num <= 5)
		*ret = snprintf(buf + *ret, PAGE_SIZE, "PT_FRAME_%d   : ",
								 pt_num);
	*ret = snprintf(buf + *ret, PAGE_SIZE, "\n   : ");

	if (type == U0_M1_RAWDATA_TEST || type == U0_M1_NOISE_TEST ||
		type == M1_DIFF_TEST) {
		col_size = M1_COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
	} else if (type == U3_M1_RAWDATA_TEST || type == U3_M1_NOISE_TEST) {
		col_size = COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
	} else if (type == U0_M2_RAWDATA_TEST || type == U0_M2_NOISE_TEST ||
		type == U3_M2_RAWDATA_TEST || type == U3_M2_NOISE_TEST ||
		type == M2_DIFF_TEST || type == U3_OPEN_NODE_TEST ||
		type == U3_M2_DELTA_JITTER) {
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
	} else if (type == U3_SHORT_NODE_TEST) {
		col_size = 4;
		rawdata_buf = M2_Rawdata_buf;
	}

	for (i = 0; i < col_size; i++)
		*ret += snprintf(buf + *ret, PAGE_SIZE - *ret, " [%2d] ", i);

	for (i = 0; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0, };
		int log_ret = 0;

		*ret += snprintf(buf + *ret, PAGE_SIZE - *ret,  "\n[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret,  "[%2d]  ", i);

		if (type == U0_M1_RAWDATA_TEST || type == U0_M1_NOISE_TEST ||
			type == U3_M1_RAWDATA_TEST || type == U3_M1_NOISE_TEST) {
			// Change print range
			for (j = 0; j < M1_COL_SIZE; j++) {
				*ret += snprintf(buf + *ret, PAGE_SIZE - *ret,
					"%5d ", rawdata_buf[i*col_size+j]);

				log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret,	"%5d ",
						rawdata_buf[i*col_size+j]);

				if (rawdata_buf[i*col_size+j] != 0 &&
						rawdata_buf[i*col_size+j] < min)
					min = rawdata_buf[i*col_size+j];

				if (rawdata_buf[i*col_size+j] > max)
					max = rawdata_buf[i*col_size+j];
			}
		} else {
			for (j = 0; j < col_size; j++) {
				*ret += snprintf(buf + *ret, PAGE_SIZE - *ret,
					"%5d ", rawdata_buf[i*col_size+j]);

				log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret,	"%5d ",
						rawdata_buf[i*col_size+j]);

				if (rawdata_buf[i*col_size+j] != 0 &&
						rawdata_buf[i*col_size+j] < min)
					min = rawdata_buf[i*col_size+j];

				if (rawdata_buf[i*col_size+j] > max)
					max = rawdata_buf[i*col_size+j];
			}
		}
		TOUCH_I("%s\n", log_buf);
	}

	*ret += snprintf(buf + *ret, PAGE_SIZE - *ret, "\n");

	*ret += snprintf(buf + *ret, PAGE_SIZE - *ret,
			"\nRawdata min : %d , max : %d\n", min, max);

	return *ret;
}

static void prd_read_rawdata(struct device *dev, u32 type)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	if (type == U0_M1_RAWDATA_TEST)	{
		TOUCH_I("\nFRAME_1\n");
		read_pt_frame(dev, d, type, PT_FRAME_1);
		ret = prd_print_rawdata(dev, W_Buf, type, 1, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U0_M1_NOISE_TEST) {
		TOUCH_I("\nFRAME_2\n");
		read_pt_frame(dev, d, type, PT_FRAME_2);
		ret = prd_print_rawdata(dev, W_Buf, type, 2, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
		TOUCH_I("\nFRAME_4\n");
		read_pt_frame(dev, d, type, PT_FRAME_4);
		ret = prd_print_rawdata(dev, W_Buf, type, 4, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U0_M2_RAWDATA_TEST) {
		TOUCH_I("\nFRAME_1\n");
		read_pt_frame(dev, d, type, PT_FRAME_1);
		ret = prd_print_rawdata(dev, W_Buf, type, 1, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U0_M2_NOISE_TEST) {
		TOUCH_I("\nFRAME_2\n");
		read_pt_frame(dev, d, type, PT_FRAME_2);
		ret = prd_print_rawdata(dev, W_Buf, type, 2, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
		TOUCH_I("\nFRAME_4\n");
		read_pt_frame(dev, d, type, PT_FRAME_4);
		ret = prd_print_rawdata(dev, W_Buf, type, 4, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U2_M2_RAWDATA_TEST) {
		TOUCH_I("\nFRAME_1\n");
		read_pt_frame(dev, d, type, PT_FRAME_1);
		ret = prd_print_rawdata(dev, W_Buf, type, 1, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U3_M1_RAWDATA_TEST) {
		TOUCH_I("\nFRAME_1\n");
		read_pt_frame(dev, d, type, PT_FRAME_1);
		ret = prd_print_rawdata(dev, W_Buf, type, 1, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U3_M1_NOISE_TEST) {
		TOUCH_I("\nFRAME_2\n");
		read_pt_frame(dev, d, type, PT_FRAME_2);
		ret = prd_print_rawdata(dev, W_Buf, type, 2, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
		TOUCH_I("\nFRAME_4\n");
		read_pt_frame(dev, d, type, PT_FRAME_4);
		ret = prd_print_rawdata(dev, W_Buf, type, 4, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U3_M2_RAWDATA_TEST) {
		TOUCH_I("\nFRAME_1\n");
		read_pt_frame(dev, d, type, PT_FRAME_1);
		ret = prd_print_rawdata(dev, W_Buf, type, 1, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U3_M2_NOISE_TEST) {
		TOUCH_I("\nFRAME_2\n");
		read_pt_frame(dev, d, type, PT_FRAME_2);
		ret = prd_print_rawdata(dev, W_Buf, type, 2, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
		TOUCH_I("\nFRAME_4\n");
		read_pt_frame(dev, d, type, PT_FRAME_4);
		ret = prd_print_rawdata(dev, W_Buf, type, 4, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	} else if (type == U3_M2_DELTA_JITTER) {
		TOUCH_I("\nFRAME_2\n");
		read_pt_frame(dev, d, type, PT_FRAME_2);
		ret = prd_print_rawdata(dev, W_Buf, type, 2, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
		TOUCH_I("\nFRAME_3\n");
		read_pt_frame(dev, d, type, PT_FRAME_3);
		ret = prd_print_rawdata(dev, W_Buf, type, 3, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
		TOUCH_I("\nFRAME_4\n");
		read_pt_frame(dev, d, type, PT_FRAME_4);
		ret = prd_print_rawdata(dev, W_Buf, type, 4, &ret);
		write_file(dev, W_Buf, TIME_INFO_SKIP);
	}
}

static int sdcard_spec_file_read(struct device *dev)
{
	int ret = 0;
	int fd = 0;
	char *path[2] = { "/mnt/sdcard/lucy_limit.txt",
		"/mnt/sdcard/lucy_limit_mfts.txt"
	};

	int path_idx = 0;

	mm_segment_t old_fs = get_fs();

	if (touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;
	set_fs(KERNEL_DS);
	fd = sys_open(path[path_idx], O_RDONLY, 0);
	if (fd >= 0) {
		sys_read(fd, line, sizeof(line));
		sys_close(fd);
		TOUCH_I("%s file existing\n", path[path_idx]);
		ret = 1;
	}
	set_fs(old_fs);

	return ret;
}

static int spec_file_read(struct device *dev)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fwlimit = NULL;
	const char *path[2] = { ts->panel_spec,
		ts->panel_spec_mfts
	};
	int path_idx = 0;

	if (touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		TOUCH_I("panel_spec_file name is null\n");
		ret = -1;
		goto error;
	}

	if (request_firmware(&fwlimit, path[path_idx], dev) < 0) {
		TOUCH_I("request ihex is failed in normal mode\n");
		ret = -1;
		goto error;
	}

	if (fwlimit->data == NULL) {
		ret = -1;
		TOUCH_I("fwlimit->data is NULL\n");
		goto error;
	}

	strlcpy(line, fwlimit->data, sizeof(line));

error:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

static int sic_get_limit(struct device *dev, char *breakpoint,
						 u16 (*buf)[COL_SIZE])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int boot_mode = 0;
	int file_exist = 0;
	int tx_num = 0;
	int rx_num = 0;


	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode > MINIOS_MFTS_CURVED
			|| boot_mode < NORMAL_BOOT) {
		ret = -1;
		goto error;
	}

	file_exist = sdcard_spec_file_read(dev);
	if (!file_exist) {
		TOUCH_I("There's no spec file. read it from F/W\n");
		ret = spec_file_read(dev);
		if (ret == -1)
			goto error;
	}

	if (line == NULL) {
		ret =  -1;
		goto error;
	}

	found = strnstr(line, breakpoint, sizeof(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_I(
			"failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -1;
		goto error;
	}

	memset(buf, 0, ROW_SIZE * COL_SIZE * 2);

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') &&
					(line[q - p] <= '9'); p++) {
				buf[tx_num][rx_num] += ((line[q - p] - '0') *
								cipher);
				cipher *= 10;
			}
			r++;
			if (r % (int)COL_SIZE == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;
		if (r == (int)ROW_SIZE * (int)COL_SIZE) {
			TOUCH_I("panel_spec_file scanning is success\n");
			break;
		}
	}

	if (ret == 0) {
		ret = -1;
		goto error;

	} else {
		TOUCH_I("panel_spec_file scanning is success\n");
		return ret;
	}

error:
	return ret;
}

static int prd_print_max_noise(struct device *dev, char *buf, u32 type,
								int ret_val)
{
	int i = 0, j = 0;
	int ret = ret_val;
	int max = 0;
	u16 *rawdata_buf = NULL;
	int col_size = 0;

	switch (type) {
	case U0_M1_RAWDATA_TEST:
	case M1_DIFF_TEST:
	case U0_M1_NOISE_TEST:
	case U3_M1_NOISE_TEST:
		col_size = M1_COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
		break;
	case U0_M2_RAWDATA_TEST:
	case U3_M2_RAWDATA_TEST:
	case U0_M2_NOISE_TEST:
	case U3_M2_NOISE_TEST:
	case M2_DIFF_TEST:
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
		break;
	default:
		return -EINVAL;
	}

	max = rawdata_buf[0];

	for (i = 0; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0, };
		int log_ret = 0;

		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,  "[%2d]  ", i);
		for (j = 0; j < col_size; j++) {
			log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret,
					"%5d ", rawdata_buf[i*col_size+j]);
			if (rawdata_buf[i*col_size+j] > max) {
				max = rawdata_buf[i*col_size+j];
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
						"noise_max[%d][%d] = %d\n", j,
						i, rawdata_buf[i*col_size+j]);
			}
		}
		TOUCH_I("%s\n", log_buf);
	}

	return ret;
}

/* Rawdata compare result
	Pass : reurn 0
	Fail : return 1
*/
static int prd_compare_rawdata(struct device *dev, u32 type)
{
	/* spec reading */
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	u16 *rawdata_buf = NULL;
	int col_size = 0;
	int i, j;
	int ret = 0;
	int result = 0;

	switch (type) {
	case U0_M1_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U0_M1_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U0_M1_Upper");
		col_size = M1_COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
		break;
	case U0_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U0_M2_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U0_M2_Upper");
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
		break;
	case U3_M1_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U3_M1_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U3_M1_Upper");
		col_size = COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
		break;
	case U3_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U3_M2_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U3_M2_Upper");
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
		break;
	}

	sic_get_limit(dev, lower_str, LowerImage);
	sic_get_limit(dev, upper_str, UpperImage);

	for (i = 0; i < ROW_SIZE; i++) {
		for (j = 0; j < col_size; j++) {
			if ((rawdata_buf[i*col_size+j] < LowerImage[i][j]) ||
					(rawdata_buf[i*col_size+j] >
					UpperImage[i][j])) {
				if ((type != U0_M1_RAWDATA_TEST) &&
						(i <= 1 && j <= 4)) {
					if (rawdata_buf[i*col_size+j] != 0) {
						result = 1;
						ret += snprintf(W_Buf + ret,
							BUF_SIZE - ret,
							"F [%d][%d] = %d\n",
							 i, j,
							rawdata_buf[
								i * col_size +
									j]);
					}
				} else {
					result = 1;
					ret += snprintf(W_Buf + ret, BUF_SIZE -
							ret,
							"F [%d][%d] = %d\n",
							i, j,
							rawdata_buf[
								i * col_size +
									j]);
				}
			}
		}
	}

	return result;
}

int tune_goft_print(t_goft_tune tune_val, char* log_buf, int mode_select)
{
	int ret = 0;

	if (mode_select == 1) {
		if (tune_val.b.r_goft_tune_m1_sign == 0) {
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"-%d  ", tune_val.b.r_goft_tune_m1);
		} else {
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"%d  ", tune_val.b.r_goft_tune_m1);
		}
	} else if (mode_select == 2) {
		if (tune_val.b.r_goft_tune_m2_sign == 0) {
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"-%d  ", tune_val.b.r_goft_tune_m2);
		} else {
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"%d  ", tune_val.b.r_goft_tune_m2);
		}
	}

	ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
	return ret;
}

int tune_loft_print(int num_ch, u16* tune_val, char* log_buf)
{
	int i;
	int ret = 0;
	int temp = 0;

	ret = snprintf(log_buf, tc_tune_code_size, "Odd : ");
	for (i = 0; i < num_ch; i++) {
		if (((tune_val[i] >> 5) & 1) == 0) {
			temp = tune_val[i] & 0x1F;
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"-%d  ", temp);
		} else if (((tune_val[i] >> 5) & 1) == 1) {
			temp = tune_val[i] & 0x1F;
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"%d  ", temp);
		} else
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"ISB  ");
	}

	ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");

	ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "Even : ");
	for (i = 0; i < num_ch; i++) {
		if (((tune_val[i] >> 13) & 1) == 0) {
			temp = (tune_val[i] >> 8) & 0x1F;
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"-%d  ", temp);
		} else if (((tune_val[i] >> 13) & 1) == 1) {
			temp = (tune_val[i] >> 8) & 0x1F;
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"%d  ", temp);

		} else
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"ISB  ");
	}

	return ret;
}
static void tune_display(struct device *dev, struct sw49408_tune_data *t,
								int type)
{
	char log_buf[tc_tune_code_size] = {0,};
	int num_ch = MAX_CHANNEL / 2;
	int ret = 0;

	switch (type) {
	case U0_M1_RAWDATA_TEST:
		ret = snprintf(log_buf, tc_tune_code_size,
				"GOFT left tune_code_read : ");

		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_goft_print(t->r_goft_tune_u0_m1m2_left, log_buf, 1);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"GOFT right tune_code_read : ");

		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_goft_print(t->r_goft_tune_u0_m1m2_right,
								log_buf, 1);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
//LOFT
		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT left tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m1_left, log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT right tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m1_right, log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		break;

	case U0_M2_RAWDATA_TEST:

		ret = snprintf(log_buf, tc_tune_code_size,
				"GOFT left tune_code_read : ");

		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_goft_print(t->r_goft_tune_u0_m1m2_left, log_buf, 2);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"GOFT right tune_code_read : ");

		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_goft_print(t->r_goft_tune_u0_m1m2_right,
								log_buf, 2);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G1 left tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m2_g1_left,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G1 right tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m2_g1_right,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G2 left tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m2_g2_left,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G2 right tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m2_g2_right,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G3 left tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m2_g3_left,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G3 right tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u0_m2_g3_right,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		break;

	case U3_M2_RAWDATA_TEST:

		ret = snprintf(log_buf, tc_tune_code_size,
				"GOFT left tune_code_read : ");

		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_goft_print(t->r_goft_tune_u3_m1m2_left, log_buf, 2);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"GOFT right tune_code_read : ");

		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_goft_print(t->r_goft_tune_u3_m1m2_right,
								log_buf, 2);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;
		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G1 left tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u3_m2_g1_left,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G1 right tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u3_m2_g1_right,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G2 left tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u3_m2_g2_left,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G2 right tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u3_m2_g2_right,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G3 left tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u3_m2_g3_left,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		ret = 0;

		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT G3 right tune_code_read : ");
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		ret = 0;
		ret += tune_loft_print(num_ch, t->r_loft_tune_u3_m2_g3_right,
								log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);

		break;
	}
}

static void read_tune_code(struct device *dev, u32 type)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_tune_data t;

	sw49408_reg_read(dev, d->reg_info.r_tune_code_spi_addr, &t,
				sizeof(struct sw49408_tune_data));
	write_file(dev, "\n[Read Tune Code]\n", TIME_INFO_SKIP);

	tune_display(dev, &t, type);
	write_file(dev, "\n", TIME_INFO_SKIP);
}

static int prd_rawdata_test(struct device *dev, u32 type)
{
	char test_type[32] = {0, };
	int result = 0;
	int write_test_mode_result = 0;

	if (type ==  U0_M1_RAWDATA_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U0_M1_RAWDATA_TEST]");
	else if (type == U0_M2_RAWDATA_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U0_M2_RAWDATA_TEST]");
	else if (type == U0_M1_NOISE_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U0_M1_NOISE_TEST]");
	else if (type == U0_M2_NOISE_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U0_M2_NOISE_TEST]");
	else if (type == U3_M1_RAWDATA_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U3_M1_RAWDATA_TEST]");
	else if (type == U3_M2_RAWDATA_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U3_M2_RAWDATA_TEST]");
	else if (type == U3_M1_NOISE_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U3_M1_NOISE_TEST]");
	else if (type == U3_M2_NOISE_TEST)
		snprintf(test_type, sizeof(test_type),
				"[U3_M2_NOISE_TEST]");
	else if (type == U3_M2_DELTA_JITTER)
			snprintf(test_type, sizeof(test_type),
				"[U3_M2_DELTA_JITTER]");
	else {
		TOUCH_I("Test Type not defined\n");
		return 1;
	}

	/* Test Type Write */
	write_file(dev, test_type, TIME_INFO_SKIP);

	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("production test couldn't be done\n");
		return 1;
	}

	prd_read_rawdata(dev, type);

	memset(W_Buf, 0, BUF_SIZE);
	/* rawdata compare result(pass : 0 fail : 1) */
	result = prd_compare_rawdata(dev, type);
	write_file(dev, W_Buf, TIME_INFO_SKIP);

	/* To Do - tune code result check */
	/*result = */
	read_tune_code(dev, type);

	return result;
}

static void ic_run_info_print(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int ret = 0;
	u32 rdata[2] = {0};

	sw49408_reg_read(dev, d->reg_info.r_pt_info_spi_addr + pt_info_date,
						(u8 *)&rdata, sizeof(rdata));

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"\n===== Production Info =====\n");
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"lot : %d\n", d->ic_info.lot );
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"date : 0x%X 0x%X\n",
			rdata[0], rdata[1]);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"date : %04d.%02d.%02d %02d:%02d:%02d Site%d\n\n",
		rdata[0] & 0xFFFF, (rdata[0] >> 16 & 0xFF),
		(rdata[0] >> 24 & 0xFF), rdata[1] & 0xFF,
		(rdata[1] >> 8 & 0xFF),
		(rdata[1] >> 16 & 0xFF),
		(rdata[1] >> 24 & 0xFF));

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static void firmware_version_log(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int boot_mode = 0;

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode >= MINIOS_MFTS_FOLDER)
		ret = sw49408_ic_info(dev);

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"======== Firmware Info ========\n");
	if (d->ic_info.version.build) {
		ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d.%d\n",
			d->ic_info.version.major, d->ic_info.version.minor,
			d->ic_info.version.build);
	} else {
		ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d\n",
			d->ic_info.version.major, d->ic_info.version.minor);
	}

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"fpc : %d, lcm : %d, lot : %d\n",
			d->ic_info.fpc, d->ic_info.lcm, d->ic_info.lot);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"product id : %s\n", d->ic_info.product_id);

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static int ic_exception_check(struct device *dev, char *buf)
{
#if 0
	struct sw49408_data *d = to_sw49408_data(dev);
	int boot_mode = 0;
	int ret = 0;

	boot_mode = touch_boot_mode_check(dev);
	/* MINIOS mode, MFTS mode check */
	if ((lge_get_boot_mode() > LGE_BOOT_MODE_NORMAL) || boot_mode > 0) {
		if (d->fw.revision < 2 ||
				d->fw.revision > 3 ||
				d->fw.version[0] != 1) {
			TOUCH_I("ic_revision : %d, fw_version : v%d.%02d\n",
					d->fw.revision,
					d->fw.version[0], d->fw.version[1]);

			ret = snprintf(buf, PAGE_SIZE,
					"========RESULT=======\n");

			if (d->fw.version[0] != 1) {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"version[v%d.%02d] : Fail\n",
					d->fw.version[0], d->fw.version[1]);
			} else {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"version[v%d.%02d] : Pass\n",
					d->fw.version[0], d->fw.version[1]);
			}

			if (d->fw.revision < 2 || d->fw.revision > 3) {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"revision[%d] : Fail\n",
					d->fw.revision);
			} else {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"revision[%d] : Pass\n",
					d->fw.revision);
			}
			write_file(dev, buf, TIME_INFO_SKIP);

			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"Raw data : Fail\n");
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"Channel Status : Fail\n");
			write_file(dev, "", TIME_INFO_WRITE);
		}
	}
	return ret;
#endif
	return 0;
}

static int check_noise_test(struct device *dev, char* buf, u32 type,
								int* ret_val)
{
	int write_test_mode_result = 0;
	int offset = *ret_val;

	if (type == U0_M1_NOISE_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"[U0_M1_NOISE_TEST]\n");
	if (type == U3_M1_NOISE_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"[U3_M1_NOISE_TEST]\n");
	else if (type == M1_DIFF_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"[M1_DIFF_TEST]\n");
	else if (type == U0_M2_NOISE_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"[U0_M2_NOISE_TEST]\n");
	else if (type == U3_M2_NOISE_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"[U3_M2_NOISE_TEST]\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"[M2_DIFF_TEST]\n");

	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("production test couldn't be done\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"production test couln't be done\n");
		return 1;
	}

	prd_read_rawdata(dev, type);
	offset = prd_print_max_noise(dev, buf, type, offset);
	if (offset < 0) {
		TOUCH_E("production test couldn't be done\n");
		offset = snprintf(buf, PAGE_SIZE,
				"production test couln't be done\n");
		return 1;
	}

	*ret_val = offset;

	return 0;
}

void sw49408_te_test_logging(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	write_file(dev, buf, TIME_INFO_SKIP);
	write_file(dev, d->te_test_log, TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	firmware_version_log(dev);
	ic_run_info_print(dev);
}

static ssize_t show_sd(struct device *dev, char *buf)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int openshort_ret = 0;
	int rawdata_ret = 0;
	int pt_command;
	int ret = 0;
	char te_log[64] = {0};
	u32 tc_status_val = 0;

	/* file create , time log */
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test Start\n");

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"LCD mode is not U3. Test Result : Fail\n");
		TOUCH_I("LCD mode is not U3. Test Result : Fail\n");
		write_file(dev, buf, TIME_INFO_SKIP);
		write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
		return ret;
	}

	mutex_lock(&ts->lock);
	if (sw49408_reg_read(dev, tc_status,
			(u32 *)&tc_status_val, sizeof(tc_status_val)) < 0) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"tc status addr read error : Fail\n");
		TOUCH_E("tc status addr read error : Fail\n");
		write_file(dev, buf, TIME_INFO_SKIP);
		write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
		mutex_unlock(&ts->lock);
		return ret;
	}
	TOUCH_I("tc status read : %x\n", tc_status_val);

	/* ic rev check - MINIOS mode, MFTS mode check */
	ret = ic_exception_check(dev, buf);
	if (ret > 0) {
		mutex_unlock(&ts->lock);
		return ret;
	}

	firmware_version_log(dev);
	ic_run_info_print(dev);

	if (sw49408_te_info(dev, te_log))
		write_file(dev, te_log, TIME_INFO_SKIP);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	sw49408_tc_driving(dev, LCD_MODE_STOP);

	pt_command = U3_PT_TEST;
	sw49408_reg_write(dev, d->reg_info.r_tc_cmd_spi_addr + tc_tsp_test_ctl, &pt_command, sizeof(int));

	/*
		OPEN_SHORT_ALL_TEST
		open - pass : 0, fail : 1
		short - pass : 0, fail : 2
	*/
	TOUCH_I("\nU3 OPEN_SHORT RAWDATA TEST\n");
	openshort_ret = prd_open_short_test(dev);

	/*
		U3_M2_RAWDATA_TEST
		rawdata - pass : 0, fail : 1
		rawdata tunecode - pass : 0, fail : 2
	*/
	TOUCH_I("\nU3 M2 RAWDATA TEST\n");
	rawdata_ret = prd_rawdata_test(dev, U3_M2_RAWDATA_TEST);

	/*
		DDIC Test - pass : 0, fail : 1
	*/

	ret = snprintf(buf, PAGE_SIZE,
			"\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");
	if (rawdata_ret == 0 && (!(tc_status_val >> 28) & 0x1)) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Pass\n");
		TOUCH_I("Raw Data : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Fail (%d/%d)\n",
				(rawdata_ret) ? 0 : 1,
				((tc_status_val >> 28) & 0x1) ? 0 : 1);
		TOUCH_I("Raw Data : Fail (%d/%d)\n",
				(rawdata_ret) ? 0 : 1,
				((tc_status_val >> 28) & 0x1) ? 0 : 1);
	}
	if (openshort_ret == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Pass\n");
		TOUCH_I("Channel Status : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Channel Status : Fail (%d/%d)\n",
			((openshort_ret & 0x1) == 0x1) ? 0 : 1,
			((openshort_ret & 0x2) == 0x2) ? 0 : 1);
		TOUCH_I("Channel Status : Fail (%d/%d)\n",
			((openshort_ret & 0x1) == 0x1) ? 0 : 1,
			((openshort_ret & 0x2) == 0x2) ? 0 : 1);
	}

	TOUCH_I("=====================\n");

	write_file(dev, buf, TIME_INFO_SKIP);

	ts->driver->power(dev, POWER_OFF);
	ts->driver->power(dev, POWER_ON);
	touch_msleep(90);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	log_file_size_check(dev);
	TOUCH_I("Show_sd Test End\n");
	return ret;
}

static ssize_t get_data(struct device *dev, int16_t *buf, u32 wdata)
{
	u32 data_offset = 0;
    u32 rdata = 1;
    int retry = 1000;
	int __frame_size = ROW_SIZE*COL_SIZE*RAWDATA_SIZE;

	struct sw49408_data *d = to_sw49408_data(dev);

	/* write 1 : GETRAWDATA
	   write 2 : GETDELTA */
	TOUCH_I("======== get data ========\n");

	sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				abt_rawdata_load_ctl, (u8 *)&wdata,
				sizeof(wdata));
	TOUCH_I("wdata = %d\n", wdata);

	/* wait until 0 is written */
	do {
		TOUCH_I("retry = %d\n", retry);
		if (retry != 1000)
			touch_msleep(5);
		sw49408_reg_read(dev, d->reg_info.r_abt_sts_spi_addr +
            abt_rawdata_load_sts,
				(u8 *)&rdata, sizeof(rdata));

	} while ((rdata != 0) && retry--);
	/* check whether 0 is written or not */

	if (rdata != 0) {
		TOUCH_E("== get data time out! ==\n");
		goto error;
	}

	/*read data*/
	if (__frame_size % 4)
		__frame_size = (((__frame_size >> 2) + 1) << 2);
	data_offset = d->reg_info.r_dbg_buf_sram_oft;

	sw49408_reg_write(dev, spr_data_offset, (u8 *)&data_offset,
								sizeof(u32));

	sw49408_reg_read(dev, data_access_addr, (u8 *)buf, __frame_size);

	return 0;

error:
	return 1;
}

static ssize_t get_ocd(struct device *dev, int8_t *buf, u32 wdata)
{
	u32 data_offset = 0;
	u32 rdata = 1;
	int retry = 1000;
	int __data_size = OCD_SIZE;

	struct sw49408_data *d = to_sw49408_data(dev);

	/* write 3 : GETOCD */
	TOUCH_I("======== get ocd ========\n");

	sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				abt_rawdata_load_ctl, (u8 *)&wdata,
				sizeof(wdata));
	TOUCH_I("wdata = %d\n", wdata);

	/* wait until 0 is written */
	do {
		TOUCH_I("retry = %d\n", retry);
		if (retry != 1000)
			touch_msleep(5);
		sw49408_reg_read(dev, d->reg_info.r_abt_sts_spi_addr
						+ abt_rawdata_load_sts,
						(u8 *)&rdata, sizeof(rdata));

	} while ((rdata != 0) && retry--);
	/* check whether 0 is written or not */

	if (rdata != 0) {
		TOUCH_E("== get data time out! ==\n");
		goto error;
	}

	/*read data*/
	if (__data_size % 4)
		__data_size = (((__data_size >> 2) + 1) << 2);

	data_offset = d->reg_info.r_dbg_buf_sram_oft;

	sw49408_reg_write(dev, spr_data_offset, (u8 *)&data_offset,
								sizeof(u32));

	sw49408_reg_read(dev, data_access_addr, (u8 *)buf, __data_size);

	return 0;

error:
	return 1;
}

static ssize_t show_ocd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret2 = 0;
	uint8_t *ocd = NULL;
	int i = 0;
	int j = 0;
	int start_point = 0;
	int cnt_point = 0;
	int index[11] = {0, 6, 12, 18, 30, 42, 52, 60, 68, 86, 101};
	int order[11] = {6, 6, 6, 12, 12, 10, 8, 8, 18, 15, 3};

	ocd = kzalloc(sizeof(uint8_t) * (OCD_SIZE), GFP_KERNEL);

	if (ocd == NULL) {
		TOUCH_E("ocd mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== OnChipDebugdata ========\n");

	mutex_lock(&ts->lock);
	ret2 = get_ocd(dev, ocd, 3);
	mutex_unlock(&ts->lock);
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < 11; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", index[i]);
		log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "[%2d]  ", index[i]);

		start_point = index[i];
		cnt_point = order[i];

		for (j = 0 ; j < cnt_point; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%5d ", ocd[start_point + j]);
			log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "%5d ", ocd[start_point + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (ocd!= NULL)
	{
		kfree(ocd);
	}

	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret2 = 0;
	int16_t *delta = NULL;
	int i = 0;
	int j = 0;

	delta = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (delta == NULL) {
		TOUCH_E("delta mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== Deltadata ========\n");

	mutex_lock(&ts->lock);
	ret2 = get_data(dev, delta, 2);  /* 2 == deltadata */
	mutex_unlock(&ts->lock);
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret, "[%2d]  ", i);

		for (j = 0 ; j < COL_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", delta[i * COL_SIZE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ", delta[i * COL_SIZE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (delta != NULL)
		kfree(delta);

	return ret;
}

static ssize_t show_fdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	int ret2 = 0;
	u32 type = U3_M2_RAWDATA_TEST;

	/* LCD off */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD Off. Test Result : Fail\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret2 = write_test_mode(dev, type);
	if (ret2 == 0) {
		TOUCH_E("write_test_mode fail\n");
		ts->driver->power(dev, POWER_OFF);
		ts->driver->power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		mutex_unlock(&ts->lock);
		return ret;
	}

	prd_read_rawdata(dev, type);
	ret = prd_print_rawdata(dev, buf, type, 6, &ret);

	ts->driver->power(dev, POWER_OFF);
	ts->driver->power(dev, POWER_ON);
	touch_msleep(90);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int i = 0;
	int j = 0;

	rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (rawdata == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== rawdata ========\n");

	mutex_lock(&ts->lock);
	ret2 = get_data(dev, rawdata, 1);  /* 2 == deltadata */
	mutex_unlock(&ts->lock);
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < ROW_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"[%2d]  ", i);

		for (j = 0 ; j < COL_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%5d ",
				rawdata[i * COL_SIZE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ",
				rawdata[i * COL_SIZE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	return ret;
}

static ssize_t get_pt_data(struct device *dev, u8 *buf, u32 wdata)
{
	u32 data_offset = 0;
	u32 rdata = 1;
	int retry = 1000;
	int __frame_size = ROW_SIZE*COL_SIZE*RAWDATA_SIZE;

	struct sw49408_data *d = to_sw49408_data(dev);

	TOUCH_I("======== get PT data ========\n");

	sw49408_reg_write(dev, 0xC27, (u8 *)&wdata, sizeof(wdata));

	TOUCH_I("write 0xC27 wdata = %d\n", wdata);

	/* wait until 0 is written */
	do {
		TOUCH_I("retry = %d\n", retry);
		if (retry != 1000)
			touch_msleep(5);
		sw49408_reg_read(dev, d->reg_info.r_abt_sts_spi_addr +
            abt_rawdata_load_sts,
				(u8 *)&rdata, sizeof(rdata));

	} while ((rdata != 0) && retry--);
	/* check whether 0 is written or not */

	if (rdata != 0) {
		TOUCH_E("== get data time out! ==\n");
		goto error;
	}

	/*read data*/
	if (__frame_size % 4)
		__frame_size = (((__frame_size >> 2) + 1) << 2);
	data_offset = d->reg_info.r_sys_buf_sram_oft;

	sw49408_reg_write(dev, spr_data_offset, (u8 *)&data_offset,
								sizeof(u32));

	sw49408_reg_read(dev, data_access_addr, (u8 *)buf, __frame_size);

	return 0;

error:
	return 1;
}

static ssize_t show_pt_data(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret2 = 0;
	u8 *ptdata = NULL;
	int i = 0;
	int j = 0;

	ptdata = kzalloc(sizeof(u8) * ROW_SIZE*COL_SIZE*RAWDATA_SIZE, GFP_KERNEL);

	if (ptdata == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== PT data ========\n");

	mutex_lock(&ts->lock);
	ret2 = get_pt_data(dev, ptdata, 1);
	mutex_unlock(&ts->lock);
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < 40 ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"[%2d]  ", i);

		for (j = 0 ; j < 28 ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%2X ",
				ptdata[i * 28 + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%2X ",
				ptdata[i * 28 + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (ptdata != NULL)
		kfree(ptdata);

	return ret;
}

static ssize_t show_noise(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	int result = 0;

	TOUCH_I("Noise Test Start\n");
	ret = snprintf(buf, PAGE_SIZE, "======== Noise Test ========\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	sw49408_tc_driving(dev, LCD_MODE_STOP);

	if (d->lcd_mode == LCD_MODE_U2) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD Mode is U2, Try in U3 or U0.\n");
		goto done;
	} else if (d->lcd_mode == LCD_MODE_U0) {
		/* M1 Noise Test */
		result = check_noise_test(dev, buf, U0_M1_NOISE_TEST, &ret);
		if (result > 0)
			goto done;

		/* M1 Diff Test */
		result = check_noise_test(dev, buf, M1_DIFF_TEST, &ret);
		if (result > 0)
			goto done;
	}

	/* M2 Noise Test */
	result = check_noise_test(dev, buf, U3_M2_NOISE_TEST, &ret);
	if (result > 0)
		goto done;

	/* M2 Diff Test */
	result = check_noise_test(dev, buf, M2_DIFF_TEST, &ret);
	if (result > 0)
		goto done;

done:
	ts->driver->power(dev, POWER_OFF);
	ts->driver->power(dev, POWER_ON);
	touch_msleep(90);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"============================\n");
	TOUCH_I("Noise Test End\n");

	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int m1_rawdata_ret = 0;
	int m2_rawdata_ret = 0;
	int pt_command;
	int ret = 0;

	/* file create , time log */
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test Start\n");

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U0) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"LCD mode is not U0. Test Result : Fail\n");
		write_file(dev, buf, TIME_INFO_SKIP);
		write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
		TOUCH_I("LCD mode is not U0. Test Result : Fail\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	firmware_version_log(dev);
	ic_run_info_print(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	sw49408_tc_driving(dev, LCD_MODE_STOP);

	pt_command = U0_PT_TEST;
	sw49408_reg_write(dev, d->reg_info.r_tc_cmd_spi_addr + tc_tsp_test_ctl, &pt_command, sizeof(int));

	TOUCH_I("\nU0_M1_RAWDATA_TEST\n");
	m1_rawdata_ret = prd_rawdata_test(dev, U0_M1_RAWDATA_TEST);

	TOUCH_I("\nU0_M2_RAWDATA_TEST\n");
	m2_rawdata_ret = prd_rawdata_test(dev, U0_M2_RAWDATA_TEST);

	ret = snprintf(buf, PAGE_SIZE, "========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	if (!m1_rawdata_ret && !m2_rawdata_ret) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : %s\n", "Pass");
		TOUCH_I("LPWG RawData : %s\n", "Pass");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : %s (%d/%d)\n", "Fail",
			m1_rawdata_ret ? 0 : 1, m2_rawdata_ret ? 0 : 1);
		TOUCH_I("LPWG RawData : %s (%d/%d)\n", "Fail",
			m1_rawdata_ret ? 0 : 1, m2_rawdata_ret ? 0 : 1);
	}
	TOUCH_I("=====================\n");

	write_file(dev, buf, TIME_INFO_SKIP);

	ts->driver->power(dev, POWER_OFF);
	ts->driver->power(dev, POWER_ON);
	touch_msleep(90);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	log_file_size_check(dev);
	TOUCH_I("Show_lpwg_sd Test End\n");

	return ret;
}
static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(fdata, show_fdata, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(noise_test, show_noise, NULL);
static TOUCH_ATTR(ocd, show_ocd, NULL);
static TOUCH_ATTR(pt_data, show_pt_data, NULL);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_fdata.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_noise_test.attr,
	&touch_attr_ocd.attr,
	&touch_attr_pt_data.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int sw49408_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}

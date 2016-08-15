/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
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

 /************************************************************************
*
* File Name: focaltech_esd_protection.c
*
* Author:	  Software Department, FocalTech
*
* Created: 2016-03-18
*   
* Modify:
*
* Abstract: 
* 启动ESD保护，在一定的间隔时间检查一下是否需要重启TP，
* 但在有其他情况下使用IIC通信期间，不能启动ESD保护，
* 避免因同步问题造成其他错误发生。
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_ESD_PROTECTION_INFO  "File Version of  focaltech_esd_protection.c:  V1.1.0 2016-03-24"
#define ESD_PROTECTION_WAIT_TIME 		2000//ms

#define A3_REG_VALUE					0x87
#define RESET_91_REGVALUE_SAMECOUNT 	5

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/



/*******************************************************************************
* Static variables
*******************************************************************************/
static struct timeval g_last_comm_time;//the Communication time of i2c RW 	
static struct task_struct *thread_esd_protection = NULL;
extern struct i2c_client *fts_i2c_client;
extern struct fts_ts_data *fts_wq_data;

static DECLARE_WAIT_QUEUE_HEAD(esd_protection_waiter);

static int g_start_esd_protection = 0;//
static int g_esd_protection_use_i2c = 0;//
static int g_esd_protection_checking = 0;//
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_esd_protection_timeout(void *unused);
static int fts_esd_protection_check(void);
static void fts_esd_check_func(void);

int  fts_esd_protection_init(void);
int  fts_esd_protection_exit(void);
int  fts_esd_protection_notice(void);
int  fts_esd_protection_suspend(void);
int  fts_esd_protection_resume(void);

/*******************************************************************************
* functions body
*******************************************************************************/
int fts_esd_protection_init(void)
{
	int err = 0;

	printk("[focal] %s \n", FOCALTECH_ESD_PROTECTION_INFO);	//show version
	
	g_start_esd_protection = 1;
	g_esd_protection_use_i2c = 0;
	g_esd_protection_checking = 0;
	
	do_gettimeofday(&g_last_comm_time);
	
	thread_esd_protection = kthread_run(fts_esd_protection_timeout, 0, "focal_esd_protection");
	if (IS_ERR(thread_esd_protection))
	{
		err = PTR_ERR(thread_esd_protection);
		printk("failed to create kernel thread: %d\n", err);
	}
	return 0;	
}
int fts_esd_protection_exit(void)
{
	kthread_stop(thread_esd_protection);	
	msleep(ESD_PROTECTION_WAIT_TIME);
	return 0;
}
static int fts_esd_protection_timeout(void *unused)
{
	unsigned int iDeltaTime=0;
	unsigned long uljiffies=0;
	struct timeval tv;

	struct sched_param param = { .sched_priority = 5 };
	sched_setscheduler(current, SCHED_RR, &param); 
	uljiffies=msecs_to_jiffies(ESD_PROTECTION_WAIT_TIME+20);
		
	do
	{
		/*设置等待条件*/
		wait_event_interruptible(esd_protection_waiter, g_start_esd_protection);
		/*设置等待超时*/
		wait_event_interruptible_timeout(esd_protection_waiter, 0, uljiffies);

		/*是否可以开始启动ESD保护*/
		if(0 == g_start_esd_protection)
			continue;
		/*记录当前时间*/
		do_gettimeofday(&tv);
		iDeltaTime = (tv.tv_sec - g_last_comm_time.tv_sec)*MSEC_PER_SEC + (tv.tv_usec - g_last_comm_time.tv_usec)/1000;

		/*当前时间与最后一次其他情况下使用IIC通信的时间，相比较，超过一定时间则执行ESD保护检查*/
		if(ESD_PROTECTION_WAIT_TIME < iDeltaTime)
		{
			pr_debug("enter fts_esd_protection_check(): iDeltaTime(ms) %d .\n", iDeltaTime);
			fts_esd_protection_check();								
		}
	}while(!kthread_should_stop());

	return 0;
}

int  fts_esd_protection_suspend(void)
{
	g_start_esd_protection = 0;
	return 0;
}
int  fts_esd_protection_resume(void)
{
	g_start_esd_protection = 1;
	wake_up_interruptible(&esd_protection_waiter);
	return 0;
}

/* 
*   
*记录其他IIC通信开始的时间
*   
*/
int fts_esd_protection_notice(void)
{
	int i = 0;

	/*如果是ESD保护在使用IIC通信，则退出*/
	if(1 == g_esd_protection_use_i2c)
		return -3;
	
	/*检查当前是否有ESD保护在使用IIC通信*/
	for(i = 0; i < 10; i++)
	{
		if(0 == g_esd_protection_checking)
			break;
		msleep(2);
	}
	if(i == 10)
	{
		printk("Failed to read/write i2c,  the i2c communication has been accounted for ESD PROTECTION.\n");
		return -1;
	}
	
	/*只要有其他地方使用了IIC通信，ESD保护则可以开始*/
	//if(0 == g_start_esd_protection)
		//g_start_esd_protection = 1;
	
	/*记录其他IIC通信开始的时间*/
	do_gettimeofday(&g_last_comm_time);
	
		
	return 0;
}

static int fts_esd_protection_check(void)
{
	g_esd_protection_checking = 1;	
	g_esd_protection_use_i2c = 1;
	//调用ESD保护检查函数
	fts_esd_check_func();
	g_esd_protection_use_i2c = 0;	
	g_esd_protection_checking = 0;
	return 0;
}
/************************************************************************
 * Name: force_reset_guitar
 * Brief: reset
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
 static void force_reset_guitar(void)
 {
	struct fts_ts_data *data = fts_wq_data;
 	if(!data){
		printk("%s NULL pointer error!\n",__func__);
		return;
 	}
	/* Reset CTP */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		pr_debug("%s esd reset tp!\n",__func__);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(20);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		msleep(400);
	}
 }

 /************************************************************************
 * Name: fts_esd_check_func
 * Brief: esd check function
 * Input: struct work_struct
 * Output: no
 * Return: 0
 ***********************************************************************/
 void fts_esd_check_func(void)
 {
	int i;
	int ret = -1;
	u8 data;
	static u8 g_first_read_91 = 0x01;	
	static u8 g_old_91_Reg_Value = 0x00;
	static u8 g_91value_same_count = 0;

	if(!fts_i2c_client){
		printk("%s NULL pointer error!\n",__func__);
		return;
 	}

	 for (i = 0; i < 3; i++) 
	 {
		 ret = fts_read_reg(fts_i2c_client, 0xA3,&data);
		 if (ret<0) 
		 {
			 printk("[Focal][Touch] read value fail\n");
		 }
		 if (ret==2 && A3_REG_VALUE==data) 
		 {
			 break;
		 }
	 }
 
	 if (i >= 3) 
	 {
		 force_reset_guitar();
		 g_91value_same_count = 0;
		 pr_debug("focal--tpd reset. i >= 3  ret = %d  A3_Reg_Value = 0x%02x\n ", ret, data);
		 return;
	 }
 	
 	// test 0x91 register.
	 ret = fts_read_reg(fts_i2c_client, 0x91,&data);
	 if (ret<0) 
	 {
		 printk("[Focal][Touch] read value of 0x91 register fail  !!!\n");
	 }
	 pr_debug("focal---------91 register value = 0x%02x	 old value = 0x%02x\n",  data, g_old_91_Reg_Value);
	 if(0x01 == g_first_read_91) 
	 {
		 g_old_91_Reg_Value = data;
		 g_first_read_91 = 0x00;
	 } 
	 else 
	 {
		 if(g_old_91_Reg_Value == data)
		 {
			 g_91value_same_count++;
			 printk("focal 91 value ==============, g_91value_same_count=%d\n", g_91value_same_count);
			 if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) 
			 {
				 force_reset_guitar();
				 printk("focal--tpd reset. g_91value_same_count = 5\n");
				 g_91value_same_count = 0;
			 }

			 g_old_91_Reg_Value = data;
		 } 
		 else 
		 {
			 g_old_91_Reg_Value = data;
			 g_91value_same_count = 0;
		 }
	 }
 }

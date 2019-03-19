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
/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include "test_lib.h"
#include "Test_FT6X36.h"
#include <linux/proc_fs.h>

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE		0
#define PROC_READ_REGISTER	1
#define PROC_WRITE_REGISTER	2
#define PROC_AUTOCLB		4
#define PROC_UPGRADE_INFO	5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA		7
#define PROC_SET_TEST_FLAG	8
#define PROC_NAME		"ftxxxx-debug"

#define WRITE_BUF_SIZE		512
#define READ_BUF_SIZE		512

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
#if GTP_ESD_PROTECT
int apk_debug_flag = 0;
#endif
/*******************************************************************************
* Static function prototypes
*******************************************************************************/



/************************************************************************
*   Name: FT_SC_Enter_Factory_Mode
*  Brief: touch ic enter factory mode
* Input: no
* Output: no
* Return: pass or fail
***********************************************************************/
static int FT_SC_Enter_Factory_Mode(void)
{
	u8 reg_addr, val_write, val_read = 0xff;
	u8 try_cnt = 0;

	reg_addr = DEV_MODE;
	val_write = FACTORY_MODE;

	/* Try to enter factory mode */
	while ((val_read != FACTORY_MODE) && (try_cnt < 10))
	{
		fts_write_reg(fts_i2c_client, reg_addr, val_write);
		msleep(10);
		fts_read_reg(fts_i2c_client, reg_addr, &val_read);
		FTS_DBG("[FTS][FACTORY][%d] >>>>> val_read: 0x%x\n", try_cnt, val_read);
		msleep(10);
		try_cnt++;
	}

	if((val_read == FACTORY_MODE) && (try_cnt < 10) )
	{
		FTS_DBG("[FTS][FACTORY] >>>>> Enter Factory Mode\n");
	}
	else
	{
		FTS_DBG("[FTS][FACTORY] >>>>> Enter Factory Mode Fail\n");
		return ERR_ENTER_FACTORY;
	}

	return 0;
}
/************************************************************************
*   Name: FT_SC_Enter_Normal_Mode
*  Brief: touch ic enter normal mode
* Input: no
* Output: no
* Return: pass or fail
***********************************************************************/
static int FT_SC_Enter_Normal_Mode(void)
{
	u8 reg_addr, val_write, val_read = 0xff;
	u8 try_cnt = 0;

	reg_addr = DEV_MODE;
	val_write = NORMAL_MODE;

	/* Try to enter normal mode */
	while (val_read != NORMAL_MODE && (try_cnt < 10))
	{
		fts_write_reg(fts_i2c_client, reg_addr, val_write);
		msleep(10);
		fts_read_reg(fts_i2c_client, reg_addr, &val_read);
		msleep(10);
		try_cnt++;
	}

	if ((val_read == NORMAL_MODE) && (try_cnt < 10) )
	{
		FTS_DBG("[FTS][FACTORY] >>>>> Enter Normal Mode\n");
	}
	else
	{
		FTS_DBG("[FTS][FACTORY] >>>>> Enter Normal Mode Fail\n");
		return ERR_ENTER_NORMAL;
	}

	return 0;
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	char upgrade_file_path[128];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
		sprintf(upgrade_file_path, "%s", writebuf + 1);
		upgrade_file_path[buflen-1] = '\0';
		FTS_DBG("%s\n", upgrade_file_path);
		disable_irq(fts_i2c_client->irq);
		#if GTP_ESD_PROTECT
			apk_debug_flag = 1;
		#endif
		
		ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
		#if GTP_ESD_PROTECT
			apk_debug_flag = 0;
		#endif
		enable_irq(fts_i2c_client->irq);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
			return ret;
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	
	return count;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		dev_err(&fts_i2c_client->dev, "%s:copy to user error\n", __func__);
		return -EFAULT;
	}

	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
	.owner = THIS_MODULE,
	.read = fts_debug_read,
	.write = fts_debug_write,
};
#else
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	char upgrade_file_path[128];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
	
		memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
		sprintf(upgrade_file_path, "%s", writebuf + 1);
		upgrade_file_path[buflen-1] = '\0';
		FTS_DBG("%s\n", upgrade_file_path);
		disable_irq(fts_i2c_client->irq);
		#if GTP_ESD_PROTECT
			apk_debug_flag = 1;
		#endif
		ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
		#if GTP_ESD_PROTECT
			apk_debug_flag = 0;
		#endif
		enable_irq(fts_i2c_client->irq);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
			return ret;
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{	
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry)
	{
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		
		return -ENOMEM;
	} 
	else
	{
		dev_info(&client->dev, "Create proc entry success!\n");
		
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{
	
	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(NULL, fts_proc_entry);
		#endif
}

/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	int err = -1;
	u16 reg_addr = FTS_REG_FW_VER;

	err = fts_read_reg(fts_i2c_client, reg_addr, &fwver);
	if (err < 0) {
		num_read_chars += sprintf(buf + num_read_chars, "get tp fw version fail!, i2c err :%d\n", err);
	} else {
		if (fwver == 255)
			num_read_chars += sprintf(buf + num_read_chars, "get tp fw version is wrong!\n");
		else
			num_read_chars += sprintf(buf + num_read_chars, "tp fw version : 0x%x\n", fwver);
	}
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tpdriver_version_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpdriver_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	
	mutex_lock(&fts_input_dev->mutex);
	
	num_read_chars = snprintf(buf, 128,"%s \n", FTS_DRIVER_INFO);
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpdriver_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpdriver_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2)
	{
		if (num_read_chars != 4)
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = kstrtoul(valbuf, 16, &wmreg);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}
	if (2 == num_read_chars)
	{
		/*read register*/
		regaddr = wmreg;
		printk("[focal][test](0x%02x)\n", regaddr);
		if (fts_read_reg(client, regaddr, &regvalue) < 0)
			printk("[Focal] %s : Could not read the register(0x%02x)\n", __func__, regaddr);
		else
			printk("[Focal] %s : the register(0x%02x) is 0x%02x\n", __func__, regaddr, regvalue);
	} 
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue)<0)
			dev_err(dev, "[Focal] %s : Could not write the register(0x%02x)\n", __func__, regaddr);
		else
			dev_dbg(dev, "[Focal] %s : Write 0x%02x into register(0x%02x) successful\n", __func__, regvalue, regaddr);
	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
	#if GTP_ESD_PROTECT
		apk_debug_flag = 1;
	#endif
	
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __func__, uc_host_fm_ver);
	}
	else
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __func__, i_ret);
	}
	
	#if GTP_ESD_PROTECT
		apk_debug_flag = 0;
	#endif
	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
	#if GTP_ESD_PROTECT
		apk_debug_flag = 1;
	#endif
	fts_ctpm_fw_upgrade_with_app_file(client, fwname);
	#if GTP_ESD_PROTECT
		apk_debug_flag = 0;
	#endif
	enable_irq(client->irq);
	
	mutex_unlock(&fts_input_dev->mutex);
	return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}


/************************************************************************
* Name: fts_tptestchnum_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tptestchnum_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0, tmp_chars = 0;
	char tmp_buf[128];
	u8 reg_addr, val_chnl_num = 0, val_key_num = 0;

	mutex_lock(&fts_input_dev->mutex);
	memset(buf,0,sizeof(char));

	if (FT_SC_Enter_Factory_Mode() == ERR_ENTER_FACTORY)
	{
		FTS_DBG("[FTS][FACTORY] >>>>> Fail at Test Channel and Key Number enter factory mode\n");
		tmp_chars = snprintf(tmp_buf, 128,"Fail at Test Channel and Key Number enter factory mode\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;
	}
	else
	{
		reg_addr = TP_CHNL_NUM;
		fts_read_reg(fts_i2c_client, reg_addr, &val_chnl_num);

		reg_addr = TP_KEY_NUM;
		fts_read_reg(fts_i2c_client, reg_addr, &val_key_num);

		if ((val_chnl_num != MAX_CHANNEL_NUMBER) || (val_key_num != MAX_KEY_NUMBER) )
		{
			FTS_DBG("[FTS][FACTORY] >>>>>Failed Test Channel and Key Number ch_num: %d, key_num: %d\n",val_chnl_num,val_key_num);
			tmp_chars = snprintf(tmp_buf, 128, "Failed Test Channel and Key Number ch_num: %d, key_num: %d\n",val_chnl_num,val_key_num);
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
		}
		else
		{
			FTS_DBG("[FTS][FACTORY] >>>>>Pass Test_Channel_and_Key_Number ch_num: %d, key_num: %d\n",val_chnl_num,val_key_num);
			tmp_chars = snprintf(tmp_buf, 128, "Pass Test_Channel_and_Key_Number ch_num: %d, key_num: %d\n",val_chnl_num,val_key_num);
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
		}
	}

	if (FT_SC_Enter_Normal_Mode() == ERR_ENTER_NORMAL)
	{
		FTS_DBG("[FTS][FACTORY] >>>>>Fail at Test Channel and Key Number return normal mode\n");
		tmp_chars = snprintf(tmp_buf, 128,"Fail at Test Channel and Key Number return normal mode\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;
	}
	mutex_unlock(&fts_input_dev->mutex);

	fts_reset_chip();

	return num_read_chars;
}
/************************************************************************
* Name: fts_tptestchnum_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tptestchnum_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tptestrawdata_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tptestrawdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0, tmp_chars = 0;
	int try_cnt = 0;
	int i = 0;
	char tmp_buf[128];
	u8 reg_val = 0, val_write = 0, val_read = 0xff;
	u8 rawdata_packet_buf[MAX_CHANNEL_NUMBER * 2];
	u16 rawdata_buf[MAX_CHANNEL_NUMBER];
	//u16 Rmax = 0, Rmin = 0xffff;

	mutex_lock(&fts_input_dev->mutex);
	memset(buf,0,sizeof(char));

	if (FT_SC_Enter_Factory_Mode() == ERR_ENTER_FACTORY)
	{
		FTS_DBG("[FTS][FACTORY] >>>>> Fail at Test RawData enter factory mode\n");
		tmp_chars = snprintf(tmp_buf, 128,"Fail at Test RawData enter factory mode\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;
	}
	else
	{
		do
		{	//TODO need to debug for write 0x01 on register 0x08 by i2c
			reg_val = TP_A_CMD_REG;
			val_write = 0x01;
			fts_write_reg(fts_i2c_client, reg_val, val_write);
			msleep(50);
			//after, need to get 0x00 on register 0x08 ready for getting rawdata
			fts_read_reg(fts_i2c_client, reg_val, &val_read);
			try_cnt++;
		}
		while ( (val_read != 0) && (try_cnt < 10) );

		if (val_read != 0)
		{
			FTS_DBG("[FTS][FACTORY] >>>>>Fail at Test RawData ready\n");
			tmp_chars = snprintf(tmp_buf, 128,"Fail at Test RawData ready\n");
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
			goto ERROR_RETURN;
		}

		reg_val = RAW_DATA_ADDR_R;
		val_write = 0;
		//tell touch ic, read rawdata from addr 0x00 --> write 0x00 on register 0x34
		fts_write_reg(fts_i2c_client, reg_val, val_write);
		msleep(500);

		reg_val = RAW_DATA_BUF;
		//start to read from register 0x35. the data has CHANNEL_NUMBER * 2 bytes.
		fts_i2c_read(fts_i2c_client, &reg_val, 1, rawdata_packet_buf, MAX_CHANNEL_NUMBER * 2);

		FTS_DBG("-------------- List All Raw Data --------------\n");
		tmp_chars = snprintf(tmp_buf, 128,"-------------- List All Raw Data --------------\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;
		/* List Raw Data of Each Channel */
		for(i = 0; i < MAX_CHANNEL_NUMBER * 2; i += 2)
		{
			rawdata_buf[i/2] = (((u16)rawdata_packet_buf[i]) << 8) | ((u16)rawdata_packet_buf[i+1]);

			FTS_DBG("[FTS] Raw Data[%d]: %d\n",i/2,rawdata_buf[i/2]);
			tmp_chars = snprintf(tmp_buf, 128,"Raw Data[%d]: %d\n",i/2,rawdata_buf[i/2]);
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
		}

	}

	FTS_DBG("-------------- End Raw Data --------------\n");
	tmp_chars = snprintf(tmp_buf, 128,"-------------- End Raw Data --------------\n");
	strcat(buf,tmp_buf);
	num_read_chars += tmp_chars;

ERROR_RETURN:

	if (FT_SC_Enter_Normal_Mode() == ERR_ENTER_NORMAL)
	{
		FTS_DBG("[FTS][FACTORY] >>>>>Fail at Test RawData return normal mode\n");
		tmp_chars = snprintf(tmp_buf, 128,"Fail at Test RawData return normal mode\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;
	}
	mutex_unlock(&fts_input_dev->mutex);

	fts_reset_chip();

	return num_read_chars;
}

/************************************************************************
* Name: fts_tptestrawdata_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tptestrawdata_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tptestcbdata_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tptestcbdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0, tmp_chars = 0;
	int try_cnt = 0;
	int i = 0;
	char tmp_buf[128];
	u8 reg_val = 0, val_write = 0, val_read = 0xff;
	u8 cbdata_packet_buf[MAX_CHANNEL_NUMBER * 2];
	u16 cb_data[MAX_CHANNEL_NUMBER];
	u16 cbdata_buf[MAX_CHANNEL_NUMBER];

	u8 cbDeltadata_packet_buf[MAX_CHANNEL_NUMBER * 2];
	u16 cbDeltadata_buf[MAX_CHANNEL_NUMBER];
	//u16 CBmax = 0, CBmin = 0xffff;
	//u16 CBDeltamax = 0, CBDeltamin = 0xffff;

	mutex_lock(&fts_input_dev->mutex);
	memset(buf,0,sizeof(char));

	if (FT_SC_Enter_Factory_Mode() == ERR_ENTER_FACTORY)
	{
		FTS_DBG("[FTS][FACTORY] >>>>>Fail at Test CBDATA enter factory mode\n");
		tmp_chars = snprintf(tmp_buf, 128,"Fail at Test CBDATA enter factory mode\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;
	}
	else
	{
		//F_TESTMODE_2 for getting CB data
		reg_val = FACTORY_TEST_MODE_ADDR;
		val_write = F_TESTMODE_2;
		//tell touch ic, change FACTORY_TEST_MODE 2
		fts_write_reg(fts_i2c_client, reg_val, val_write);
		msleep(100);

		do
		{	//TODO need to debug for write 0x01 on register 0x08 by i2c
			reg_val = TP_A_CMD_REG;
			val_write = 0x01;
			fts_write_reg(fts_i2c_client, reg_val, val_write);
			msleep(50);
			//after, need to get 0x00 on register 0x08 ready for getting cbdata
			fts_read_reg(fts_i2c_client, reg_val, &val_read);
			try_cnt++;
		}
		while ( (val_read != 0) && (try_cnt < 10) );

		if (val_read != 0)
		{
			FTS_DBG("[FTS][FACTORY] >>>>>Fail at Test CBData ready\n");
			tmp_chars = snprintf(tmp_buf, 128,"Fail at Test CBData ready\n");
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
			goto ERROR_RETURN;
		}

		//TODO: checking, take CB data is same way with rawdata or not?
		reg_val = CB_DATA_ADDR_R;
		val_write = 0;
		//tell touch ic, read CB data from addr 0x00 --> write 0x00 on register 0x33
		fts_read_reg(fts_i2c_client, reg_val, &val_read);
		msleep(500);

		reg_val = CB_DATA_BUF;
		//start read from register 0x39. the data has CHANNEL_NUMBER * 2 bytes.
		fts_i2c_read(fts_i2c_client, &reg_val, 1, cbdata_packet_buf, MAX_CHANNEL_NUMBER * 2);

		FTS_DBG("-------------- List CB Data --------------\n");
		tmp_chars = snprintf(tmp_buf, 128,"-------------- List CB Data --------------\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;

		for (i = 0; i < MAX_CHANNEL_NUMBER * 2; i += 2)
		{
			cb_data[i/2] = cbdata_buf[i/2] = (((u16)cbdata_packet_buf[i]) << 8) | ((u16)cbdata_packet_buf[i+1]);
			FTS_DBG("[FTS] CB Data[%d]: %d\n",i/2,cb_data[i/2]);
			tmp_chars = snprintf(tmp_buf, 128,"CB Data[%d]: %d\n",i/2,cb_data[i/2]);
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
		}

		FTS_DBG("-------------- End CB Data --------------\n");
		tmp_chars = snprintf(tmp_buf, 128,"-------------- End CB Data --------------\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;

		//F_TESTMODE_1 for getting CB Delta data
		reg_val = FACTORY_TEST_MODE_ADDR;
		val_write = F_TESTMODE_1;
		//tell touch ic, change FACTORY_TEST_MODE 1
		fts_write_reg(fts_i2c_client, reg_val, val_write);
		msleep(100);
		do
		{	//TODO need to debug for write 0x01 on register 0x08 by i2c
			reg_val = TP_A_CMD_REG;
			val_write = 0x01;
			fts_write_reg(fts_i2c_client, reg_val, val_write);
			msleep(50);
			//after, need to get 0x00 on register 0x08 ready for getting cb delta data
			fts_read_reg(fts_i2c_client, reg_val, &val_read);
			try_cnt++;
		}
		while ( (val_read != 0) && (try_cnt < 10) );
		if (val_read != 0)
		{
			FTS_DBG("[FTS][FACTORY] >>>>>Fail at Test CB Delta Data ready\n");
			tmp_chars = snprintf(tmp_buf, 128,"Fail at Test CB Delta Data ready\n");
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
			goto ERROR_RETURN;
		}

		//TODO: checking, take CB Delta data is same way with rawdata or not?
		reg_val = CB_DATA_ADDR_R;
		val_write = 0;
		//tell touch ic, read CB Delta data from addr 0x00 --> write 0x00 on register 0x33
		fts_write_reg(fts_i2c_client, reg_val, val_write);
		msleep(500);

		reg_val = CB_DATA_BUF;
		//start read from register 0x39. the data has CHANNEL_NUMBER * 2 bytes.
		fts_i2c_read(fts_i2c_client, &reg_val, 1, cbDeltadata_packet_buf, MAX_CHANNEL_NUMBER * 2);

		FTS_DBG("-------------- List CB Delta Data --------------\n");
		tmp_chars = snprintf(tmp_buf, 128,"-------------- List CB Delta Data --------------\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;

		for (i = 0; i < MAX_CHANNEL_NUMBER * 2; i += 2)
		{
			cbDeltadata_buf[i/2] = abs(cbdata_buf[i/2] - ( (((u16)cbDeltadata_packet_buf[i]) << 8) | ((u16)cbDeltadata_packet_buf[i+1])));
			//cbDeltadata_buf[i/2] = cbdata_buf[i/2] - ( (((u16)cbDeltadata_packet_buf[i]) << 8) | ((u16)cbDeltadata_packet_buf[i+1]));

			FTS_DBG("[FTS] CB Delta Data[%d]: %d\n",i/2,cbDeltadata_buf[i/2]);
			tmp_chars = snprintf(tmp_buf, 128,"CB Delta Data[%d]: %d\n",i/2,cbDeltadata_buf[i/2]);
			strcat(buf,tmp_buf);
			num_read_chars += tmp_chars;
		}
		FTS_DBG("-------------- End CB Delta Data --------------\n");
		tmp_chars = snprintf(tmp_buf, 128,"-------------- End CB Delta Data --------------\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;

	}
ERROR_RETURN:

	if (FT_SC_Enter_Normal_Mode() == ERR_ENTER_NORMAL)
	{
		FTS_DBG("[FTS][FACTORY] >>>>>Fail at Test CBDATA return normal mode\n");
		tmp_chars = snprintf(tmp_buf, 128,"Fail at Test CBDATA return normal mode\n");
		strcat(buf,tmp_buf);
		num_read_chars += tmp_chars;
	}
	mutex_unlock(&fts_input_dev->mutex);
	fts_reset_chip();

	return num_read_chars;
}
/************************************************************************
* Name: fts_tptestcbdata_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tptestcbdata_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
int FTS_I2c_Read(unsigned char *wBuf, int wLen, unsigned char *rBuf, int rLen)
{
	if (NULL == fts_i2c_client) {
		return -1;
	}

	return fts_i2c_read(fts_i2c_client, wBuf, wLen, rBuf, rLen);
}
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

int FTS_I2c_Write(unsigned char *wBuf, int wLen)
{
	if (NULL == fts_i2c_client) {
		return -1;
	}

	return fts_i2c_write(fts_i2c_client, wBuf, wLen);
}

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

#define FT3207_CFG_FILEPATH "/system/res/TP/"
#define FT3207_RESULT_FILEPATH "/sdcard/"
static int ft3207_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT3207_CFG_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[FTS][TOUCH_ERR] %s : error occured while opening file %s. \n", __func__, filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft3207_ReadInIData(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT3207_CFG_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[FTS][TOUCH_ERR] %s : error occured while opening file %s. \n", __func__, filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
static int ft3207_get_testparam_from_ini(char *config_name)
{
	char *config_data = NULL;
	int file_size;

	file_size = ft3207_GetInISize(config_name);

	pr_err("[FTS][Touch] %s : inisize = %d \n ", __func__, file_size);
	if (file_size <= 0) {
		pr_err("[FTS][TOUCH_ERR] %s : ERROR : Get firmware size failed \n", __func__);
		return -EIO;
	}

	config_data = kmalloc(file_size + 1, GFP_ATOMIC);

	if (ft3207_ReadInIData(config_name, config_data)) {
		pr_err("[FTS][TOUCH_ERR] %s() - ERROR: request_firmware failed \n", __func__);
		kfree(config_data);
		return -EIO;
	}
	else {
		pr_info("[FTS][Touch] %s : ft3207_ReadInIData successful \n", __func__);
	}
	//TODO checking: GC change here code.
	if(set_param_data(config_data)<0)
	{
		printk("[FTS]%s The IC type error in this testing\n", __func__);
		return -1;
	}
	return 0;
}

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

static mm_segment_t oldfs;

static struct file *fts_selftest_file_open(void)
{

	struct file* filp = NULL;
	char filepath[128];
	int err = 0;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT3207_RESULT_FILEPATH, "selftest.csv");

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filepath, O_WRONLY|O_CREAT|O_APPEND , 0644);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}

	return filp;
}
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

int fts_selftest_file_write(struct file* file, unsigned char *data, int len)
{

	int ret;
	ret = file->f_op->write(file, data, len, &file->f_pos);

	return ret;
}
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

void fts_selftest_file_close(struct file* file)
{

	set_fs(oldfs);
	filp_close(file, NULL);
}
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
int selft_test_result = 0;

static ssize_t fts_selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	char config_file[128], result_buf[128];
	char *w_buf;
	struct file *w_file;
	int w_len=0, i = 0;

	bool item_result[SELFTEST_ITEM_NUM]={0};
	int count = 0;
	selft_test_result = 1;

	disable_irq(client->irq);
	memset(config_file, 0, sizeof(config_file));
	sprintf(config_file, TP_INI_FILE);

	printk("[FTS]%s : loading config file : %s \n", __func__, config_file);

	init_i2c_read_func(FTS_I2c_Read);
	init_i2c_write_func(FTS_I2c_Write);

	if (ft3207_get_testparam_from_ini(config_file) < 0) {
		printk("[FTS][TOUCH_ERR] %s : get testparam from ini failure \n", __func__);
	}
	else {
		printk("[FTS][Touch] %s : tp test Start... \n", __func__);

		if (FT3207_StartTest(item_result)) {
			printk("[FTS][Touch] %s : tp test pass \n", __func__);
			selft_test_result = 0;
		}
		else {
			printk("[FTS][Touch] %s : tp test failure \n", __func__);
			selft_test_result = 1;
		}

		for (i = 0; i < 3; i++) {
			if (fts_write_reg(client, 0x00, 0x00) >= 0)
				break;
			else
				msleep(200);
		}

		w_file = fts_selftest_file_open();
		if (!w_file) {
			printk("[FTS][Touch] %s : Open log file fail !\n", __func__);
		} else {
			w_len =  FT3207_get_test_data(&w_buf);
			fts_selftest_file_write(w_file, w_buf, w_len);


			w_len = sprintf(result_buf, "Selftest %s\n=====END=====\n",selft_test_result ? "FAIL" : "PASS");
			fts_selftest_file_write(w_file, result_buf, w_len);
			fts_selftest_file_close(w_file);
		}

		free_test_param_data();
	}

	enable_irq(client->irq);

	//TODO GC add reset chip here
	fts_reset_chip();

	if (selft_test_result) {
		printk("[FTS] %s: Selftest FAIL\n", __func__);
	}
	else {
		printk("[FTS] %s : Selftest PASS\n", __func__);
	}

	count += sprintf(buf + count, "[FTS] : Selftest %s;\n", selft_test_result ? "FAIL" : "PASS");


	for (i = 0 ;i < SELFTEST_ITEM_NUM; i++) {
		if (item_result[i] == false) {
			switch(i) {
				case ENTER_FACTORY_MODE_TEST:
					count += sprintf(buf + count, "TP EnterFTM Fail;\n");
					printk("[fts]%s : TP EnterFTM Fail \n", __func__);
					return count;
				break;
				case RAWDATA_TEST:
					count += sprintf(buf + count, "RawData Fail;\n");
					printk("[fts]%s : TP RawData Fail \n", __func__);
				break;
				case CB_TEST:
					count += sprintf(buf + count, "CB Fail;\n");
					printk("[fts]%s : TP CB Fail \n", __func__);
				break;
				case DIFF_CB_TEST:
					count += sprintf(buf + count, "DiffCB Fail;\n");
					printk("[fts]%s : TP DiffCB Fail \n", __func__);
				break;
				default:
				break;
			}
		}
	}

	return count;
}

//dean add for FW check start
static ssize_t fts_fw_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	if(fts_wq_data->fw_ver[0] == 0) {
		count += sprintf(buf + count, "FAIL\n");
		count += sprintf(buf + count, "TP ID is wrong\n"); 
	}
	else {
		count += sprintf(buf + count, "Pass, (fw_ver : 0x%x, fw_id : 0x%x, tp_id : 0x%x)\n",
		fts_wq_data->fw_ver[0], fts_wq_data->fw_vendor_id, fts_wq_data->tp_vendor_id);
	}
	return count;
}

#define FTS_ALL_PACKET_LENGTH	256
static int getTPid(void)
{
	struct i2c_client *client = fts_i2c_client;
	unsigned char cmd[6] = {0};
	u8 tmpBuf[FTS_ALL_PACKET_LENGTH] = {0};
	int err = 0;

	//first, let touch ic enter read mode
	cmd[0] = 0x55;
	cmd[1] = 0xAA;
	err = fts_i2c_write(client, cmd, 2);
	if (err < 0) {
		dev_err(&client->dev, "TP vendor id read failed 1 \n");
		goto tp_id_err;
	}
	msleep(2);
	fts_reset_chip();
	err = fts_i2c_write(client, cmd, 2);
	if (err < 0) {
		dev_err(&client->dev, "TP vendor id read failed 2 \n");
		goto tp_id_err;
	}
	msleep(2);

	//start to read touch F/W's TP vendor ID
	cmd[0] = 0x02;
	cmd[1] = 0xFD;
	cmd[2] = (u8)((7 * FTS_ALL_PACKET_LENGTH)>>8);
	cmd[3] = (u8)(7 * FTS_ALL_PACKET_LENGTH);
	cmd[4] = (u8)((FTS_ALL_PACKET_LENGTH-1)>>8);
	cmd[5] = (u8)(FTS_ALL_PACKET_LENGTH-1);
	err = fts_i2c_read(client, cmd, 6, tmpBuf, FTS_ALL_PACKET_LENGTH);
	if (err < 0) {
		dev_err(&client->dev, "TP vendor id read failed 3 \n");
		goto tp_id_err;
	}

	msleep(2);

	//lock flash cptm
	cmd[0] = 0x0A;
	cmd[1] = 0xF5;
	err = fts_i2c_write(client, cmd, 2);
	if (err < 0) {
		dev_err(&client->dev, "TP vendor id read failed 4 \n");
		goto tp_id_err;
	}

	msleep(100);

	fts_reset_chip();
	msleep(300);

	return tmpBuf[0xB4];

tp_id_err:

	fts_reset_chip();
	msleep(300);
	return -1;
}

int check_TP_ID(char *buf, ssize_t buf_len, u8 tp_id) {

	switch(tp_id) {
		case TP_ID_0:
			buf_len += sprintf(buf + buf_len, "TP is 0x82\n");
 			break;
		case TP_ID_1:
			buf_len += sprintf(buf + buf_len, "TP is 0x83\n");
			break;
		case TP_ID_2:
			buf_len += sprintf(buf + buf_len, "TP is 0x84\n");
			break;
		case TP_ID_3:
			buf_len += sprintf(buf + buf_len, "TP is 0x85\n");
			break;
		default:
			buf_len += sprintf(buf + buf_len, "TP isn't correct\n");
			break;
	}
	return buf_len;

}

//dean add get TP infomation include fw ver, tp id from chip by i2c
static ssize_t fts_getTPinfo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0, tp_id = 0;
	u8 err = -1;
	u8 reg_addr = FTS_REG_FW_VER;

	mutex_lock(&fts_input_dev->mutex);

	err = fts_i2c_read(fts_i2c_client, &reg_addr, 1, &fwver, 1);

	if (fwver == 255 || err < 0)
		num_read_chars += sprintf(buf + num_read_chars,"get tp fw version fail!\n");
	else
	{
		num_read_chars += sprintf(buf + num_read_chars, "FW Ver : 0x%02X\n", fwver);
	}

	tp_id = getTPid();
	num_read_chars += sprintf(buf + num_read_chars, "TP ID : 0x%02X\n", tp_id);

	num_read_chars = check_TP_ID(buf, num_read_chars, tp_id);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

//dean add get TP infomation include fw ver, tp id from chip by i2c
static ssize_t fts_TPupgradeAPP_for_i_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	u8 err = -1;

	mutex_lock(&fts_input_dev->mutex);

	disable_irq(client->irq);

	err = fts_ctpm_auto_upgrade_for_cci(fts_i2c_client, fts_wq_data->tp_vendor_id, true);

	if(err == 0) {
		fts_update_fw_ver(fts_wq_data);
		fts_update_fw_vendor_id(fts_wq_data);

		num_read_chars += sprintf(buf + num_read_chars, "TP Upgrade is OK, FW ver : 0x%x, TP ID : 0x%x\n",
		fts_wq_data->fw_ver[0], fts_wq_data->tp_vendor_id);
	} else {
		num_read_chars += sprintf(buf + num_read_chars, "TP Upgrade is Fail\n");
	}

	num_read_chars = check_TP_ID(buf, num_read_chars,  fts_wq_data->tp_vendor_id);

	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static ssize_t show_tp_debug_tag(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifndef FTS_GESTRUE_EN
	return sprintf(buf, "TP down event log : %s\n", print_point?"ON":"OFF");
#else
	return sprintf(buf, "TP down event log : %s, gesture log: %s\n", print_point?"ON":"OFF",print_gesture?"ON":"OFF");
#endif
}

static ssize_t store_tp_debug_tag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_DEBUG "[fts]%s, buf : %d\n", __func__, buf[0]);

	switch(buf[0]) {
		case DISABLE_TAG:
			printk(KERN_DEBUG "[fts]%s, TP debug log disable\n", __func__);
			print_point = false;
#ifdef FTS_GESTRUE_EN
			print_gesture = false;
#endif
		break;
		case ENABLE_POINT_TAG:
			printk(KERN_DEBUG "[fts]%s, TP down event log enable\n", __func__);
			print_point = true;
		break;
#ifdef FTS_GESTRUE_EN
		case ENABLE_GESTURE_TAG:
			printk(KERN_DEBUG "[fts]%s, TP gesture log enable\n", __func__);
			print_gesture = true;
		break;
#endif
		default :
			printk(KERN_DEBUG "[fts]%s, invalid format\n", __func__);
		break;
	}

	return count;
}

#ifdef FTS_GESTRUE_EN
static ssize_t show_tp_enable_big_area_event_tag(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "[fts]%s, Enable TP big area event into ambient mode\n", __func__);
	en_big_area_func = true;
	return sprintf(buf, "TP big area function : %s\n", en_big_area_func?"ON":"OFF");
}

static ssize_t show_tp_disable_big_area_event_tag(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "[fts]%s, Disable TP big area event into ambient mode\n", __func__);
	en_big_area_func = false;
	return sprintf(buf, "TP big area function : %s\n", en_big_area_func?"ON":"OFF");
}

#endif

static ssize_t tp_enable_irq_func(struct device *dev, struct device_attribute *attr, char *buf)
{
	enable_irq(fts_wq_data->client->irq);
	return snprintf(buf, 64, "TP irq enable\n");
}

static ssize_t tp_disable_irq_func(struct device *dev, struct device_attribute *attr, char *buf)
{
	disable_irq(fts_wq_data->client->irq);
	return snprintf(buf, 64, "TP irq disable\n");
}

static ssize_t show_tp_debug_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct irq_desc *desc = irq_to_desc(fts_wq_data->client->irq);

	return snprintf(buf, 256, "fts_wq_queue_result: %u, disable_depth: %u,\n"
		"irq_handler_recovery_count: %u, suspend_resume_recovery_count: %u\n"
		"plam_recovery_count: %u\n", fts_wq_queue_result, desc->depth,
		irq_handler_recovery_count, suspend_resume_recovery_count,
		plam_recovery_count);
}

static ssize_t fts_tp_pwr_disabled_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0 ;
	unsigned int disabling_tp;

	err = kstrtouint(buf, 10, &disabling_tp);

	if (disabling_tp) {
		printk(KERN_DEBUG "[fts]%s, Disable TP \n", __func__);

		err = fts_ts_disable(dev);
		ts_pwr_disabled = true;
	} else {
		if (ts_pwr_disabled) {
			printk(KERN_DEBUG "[fts]%s, Enable TP Now \n", __func__);
			fts_ts_start(dev);
			ts_pwr_disabled = false;
		} else {
			printk(KERN_INFO "[fts]%s, TP power already on \n", __func__);
			return count;
		}
	}
	return count;
}

static ssize_t fts_tp_pwr_disabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "[fts]%s, Show TP Disable state\n", __func__);
	return EPERM;
}

static DEVICE_ATTR(ftstppwrdisable, S_IRUGO|S_IWUSR|S_IWGRP, fts_tp_pwr_disabled_show, fts_tp_pwr_disabled_store);

/****************************************/
/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

static DEVICE_ATTR(ftstpdriverver, S_IRUGO|S_IWUSR, fts_tpdriver_version_show, fts_tpdriver_version_store);
/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, fts_getprojectcode_show, fts_getprojectcode_store);

static DEVICE_ATTR(ftstptestchnum, S_IRUGO|S_IWUSR, fts_tptestchnum_show, fts_tptestchnum_store);
static DEVICE_ATTR(ftstptestrawdata, S_IRUGO|S_IWUSR, fts_tptestrawdata_show, fts_tptestrawdata_store);
static DEVICE_ATTR(ftstptestcbdata, S_IRUGO|S_IWUSR, fts_tptestcbdata_show, fts_tptestcbdata_store);

static DEVICE_ATTR(ftstpselftest, S_IRUGO, fts_selftest_show, NULL);

//dean add for FW check
static DEVICE_ATTR(fw_check, S_IRUGO, fts_fw_check_show, NULL);

static DEVICE_ATTR(fts_getTPinfo, S_IRUGO, fts_getTPinfo_show, NULL);
static DEVICE_ATTR(fts_TPupgradeAPP_for_i, S_IRUGO, fts_TPupgradeAPP_for_i_show, NULL);
static DEVICE_ATTR(en_msg_report_data, S_IRUGO | S_IWUSR, show_tp_debug_tag, store_tp_debug_tag);

#ifdef FTS_GESTRUE_EN
static DEVICE_ATTR(enable_big_area_event, S_IRUGO, show_tp_enable_big_area_event_tag, NULL);
static DEVICE_ATTR(disable_big_area_event, S_IRUGO, show_tp_disable_big_area_event_tag, NULL);

#endif

static DEVICE_ATTR(enable_tp_irq, S_IRUGO, tp_enable_irq_func, NULL);
static DEVICE_ATTR(disable_tp_irq, S_IRUGO, tp_disable_irq_func, NULL);
static DEVICE_ATTR(tp_debug_info, S_IRUGO, show_tp_debug_info, NULL);

/*add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstppwrdisable.attr,
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftstpdriverver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
	&dev_attr_ftstptestchnum.attr,
	&dev_attr_ftstptestrawdata.attr,
	&dev_attr_ftstptestcbdata.attr,
	&dev_attr_ftstpselftest.attr,
	&dev_attr_fw_check.attr,
	&dev_attr_fts_getTPinfo.attr,
	&dev_attr_fts_TPupgradeAPP_for_i.attr,
	&dev_attr_en_msg_report_data.attr,
#ifdef FTS_GESTRUE_EN
	&dev_attr_enable_big_area_event.attr,
	&dev_attr_disable_big_area_event.attr,
#endif
	&dev_attr_enable_tp_irq.attr,
	&dev_attr_disable_tp_irq.attr,
	&dev_attr_tp_debug_info.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
	int err;
	
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} 
	else
	{
		pr_info("fts:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}

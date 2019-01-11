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
#ifndef INI_H
#define INI_H

#define MAX_KEY_NUM		300
#define MAX_KEY_NAME_LEN	50
#define MAX_KEY_VALUE_LEN	360

#define MAX_CFG_BUF     480
#define SUCCESS		0
/* return value */
#define CFG_OK			SUCCESS
#define CFG_SECTION_NOT_FOUND	-1 
#define CFG_KEY_NOT_FOUND	-2 
#define CFG_ERR			-10 

#define CFG_ERR_OPEN_FILE	-10 
#define CFG_ERR_CREATE_FILE	-11 
#define CFG_ERR_READ_FILE	-12 
#define CFG_ERR_WRITE_FILE	-13 
#define CFG_ERR_FILE_FORMAT	-14 
#define CFG_ERR_TOO_MANY_KEY_NUM	-15
#define CFG_ERR_OUT_OF_LEN	-16

#define CFG_ERR_EXCEED_BUF_SIZE	-22 

#define COPYF_OK 		SUCCESS 
#define COPYF_ERR_OPEN_FILE	-10 
#define COPYF_ERR_CREATE_FILE	-11 
#define COPYF_ERR_READ_FILE	-12 
#define COPYF_ERR_WRITE_FILE	-13 

#define FOCAL_DBG
#ifdef FOCAL_DBG
#define FTS_DBG(fmt, args...) printk("[FTS]" fmt, ## args)
#else
#define FTS_DBG(fmt, args...) do{}while(0)
#endif


struct ini_key_location {
	int ini_section_line_no;
	int ini_key_line_no;
	int ini_key_lines;
};

typedef struct _ST_INI_FILE_DATA
{
	char pSectionName[MAX_KEY_NAME_LEN];
	char pKeyName[MAX_KEY_NAME_LEN];
	char pKeyValue[MAX_KEY_VALUE_LEN];
	int iSectionNameLen;
	int iKeyNameLen;
	int iKeyValueLen;
}ST_INI_FILE_DATA;
//extern ST_INI_FILE_DATA g_st_ini_file_data[MAX_KEY_NUM];
//extern int g_used_key_num;

int ini_get_key(char *filedata, char * section, char * key, char * value);
int ini_get_sections(char *filedata, unsigned char * sections[], int max);

int  ini_split_section(char *section, char **name, char **index);
//int  ini_join_section(char **section, char *name, char *index);

int atoi(char *nptr);
char * ini_str_trim_r(char * buf);
char * ini_str_trim_l(char * buf);

int init_key_data(void);
int ini_get_key_data(char *filedata);
#endif

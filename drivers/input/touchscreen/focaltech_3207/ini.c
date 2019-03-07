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
#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/unistd.h>

#include "ini.h"
//#include "scap_test_lib.h"

char CFG_SSL = '[';
char CFG_SSR = ']';
char CFG_NIS = ':';
char CFG_NTS = '#';
char CFG_EQS = '=';

ST_INI_FILE_DATA g_st_ini_file_data[MAX_KEY_NUM];
int g_used_key_num = 0;

char * ini_str_trim_r(char * buf);
char * ini_str_trim_l(char * buf);
static int ini_file_get_line(char *filedata, char *buffer, int maxlen); 
//static int ini_split_key_value(char *buf, char **key, char **val); 
static long atol(char *nptr);


/*************************************************************
Function: Àò±okeyªº­È
Input: char * filedata¡@¤å¥ó¡Fchar * section¡@¶µ­È¡Fchar * key¡@Áä­È
Output: char * value¡@keyªº­È
Return: 0	SUCCESS
	-1
	-2
	-10
	-12
	-14
	-22
Note: 
*************************************************************/
int ini_get_key(char *filedata, char * section, char * key, char * value)
{
	int i = 0;
	int ret = -2;
	for(i = 0; i < g_used_key_num; i++)
	{
		if(strncmp(section, g_st_ini_file_data[i].pSectionName,
			 g_st_ini_file_data[i].iSectionNameLen) != 0)
			 continue;
		//printk("Section Name:%s, Len:%d\n\n", g_st_ini_file_data[i].pSectionName, g_st_ini_file_data[i].iSectionNameLen); 
		if(strncmp(key, g_st_ini_file_data[i].pKeyName,
			 g_st_ini_file_data[i].iKeyNameLen) == 0)
		{
			memcpy(value, g_st_ini_file_data[i].pKeyValue, g_st_ini_file_data[i].iKeyValueLen);
			ret = 0;
			break;
		}
	}
	return ret;

#if 0
	char buf1[MAX_CFG_BUF + 1], buf2[MAX_CFG_BUF + 1];
	char *key_ptr, *val_ptr;
	int  n, ret;
	int dataoff = 0;
	
	*value='\0';

	//FTS_DBG("Enter ini_get_key() \n");

	//bGetParamString
	//FTS_DBG("g_testparam.CheckIncludeKeyTest = %d \n", g_testparam.CheckIncludeKeyTest);
	while(1) { /* ·j§ä¶µsection */
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);
		dataoff += n;
		if(n < -1)
			goto r_cfg_end;
		ret = CFG_SECTION_NOT_FOUND;
		if(n < 0)
			goto r_cfg_end; /* ¤å¥ó§À¡A¥¼µo²{ */ 

		if(n > MAX_CFG_BUF)
			goto r_cfg_end;

		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if(n == 0 || buf1[0] == CFG_NTS)
			continue;       /* ªÅ¦æ ©Î µùÄÀ¦æ */ 

		ret = CFG_ERR_FILE_FORMAT;
		if(n > 2 && ((buf1[0] == CFG_SSL && buf1[n-1] != CFG_SSR)))
			goto r_cfg_end;
		if(buf1[0] == CFG_SSL) {
			buf1[n-1] = 0x00;
			if(strcmp(buf1+1, section) == 0)
			{
				break; /* §ä¨ì¶µsection */
			}
		} 
	} 
	//FTS_DBG("section = %s \n",section);

	while(1){ /* ·j§äkey */ 
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);
		dataoff += n;
		if(n < -1) 
			goto r_cfg_end;
		ret = CFG_KEY_NOT_FOUND;
		if(n < 0)
			goto r_cfg_end;/* ¤å¥ó§À¡A¥¼µo²{key */ 
		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if(n == 0 || buf1[0] == CFG_NTS) 
			continue;       /* ªÅ¦æ ©Î µùÄÀ¦æ */ 
		ret = CFG_KEY_NOT_FOUND; 
		if(buf1[0] == CFG_SSL)
		{
			goto r_cfg_end;
		}
		if(buf1[n-1] == '+') { 
			buf1[n-1] = 0x00; 
			while(1) {
				ret = CFG_ERR_READ_FILE; 
				n = ini_file_get_line(filedata+dataoff, buf2, MAX_CFG_BUF);
				dataoff += n;
				if(n < -1) 
					goto r_cfg_end; 
				if(n < 0) 
					break;

				n = strlen(ini_str_trim_r(buf2)); 
				ret = CFG_ERR_EXCEED_BUF_SIZE; 
				if(n > 0 && buf2[n-1] == '+'){
					buf2[n-1] = 0x00; 
					if( (strlen(buf1) + strlen(buf2)) > MAX_CFG_BUF) 
						goto r_cfg_end; 
					strcat(buf1, buf2); 
					continue; 
				} 
				if(strlen(buf1) + strlen(buf2) > MAX_CFG_BUF) 
					goto r_cfg_end; 
				strcat(buf1, buf2); 
				//printk("value = %s\n", buf1);
				break; 
			} 
		} 
		ret = CFG_ERR_FILE_FORMAT; 
		if(ini_split_key_value(buf1, &key_ptr, &val_ptr) != 1) 
			goto r_cfg_end; 

		ini_str_trim_l(ini_str_trim_r(key_ptr)); 
		if(strcmp(key_ptr, key) != 0) 
			continue;                                  /* ©Mkey­È¤£¤Ç°t */ 
		strcpy(value, val_ptr); 
		break; 
	} 
	ret = CFG_OK; 
r_cfg_end: 

	return ret; 
#endif
	
} 
/*************************************************************
Function: Àò±o©Ò¦³section
Input:  char *filename¡@¤å¥ó,int max ³Ì¤j¥iªð¦^ªºsectionªº­Ó¼Æ
Output: char *sections[]¡@¦s©ñsection¦W¦r
Return: ªð¦^section­Ó¼Æ¡C­Y¥X¿ù¡Aªð¦^­t¼Æ¡C
		-10			¤å¥ó¥´¶}¥X¿ù
		-12			¤å¥óÅª¨ú¿ù»~
		-14			¤å¥ó®æ¦¡¿ù»~
Note: 
*************************************************************/
int ini_get_sections(char *filedata, unsigned char * sections[], int max)
{
	//FILE *fp; 
	char buf1[MAX_CFG_BUF + 1]; 
	int n, n_sections = 0, ret; 
	int dataoff = 0;
	
//	if((fp = fopen(filename, "rb")) == NULL) 
//		return CFG_ERR_OPEN_FILE; 
	
	while(1) {
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);
		dataoff += n;
		if(n < -1) 
			goto cfg_scts_end; 
		if(n < 0)
			break;
		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if(n == 0 || buf1[0] == CFG_NTS) 
			continue;
		ret = CFG_ERR_FILE_FORMAT;
		if(n > 2 && ((buf1[0] == CFG_SSL && buf1[n-1] != CFG_SSR)))
			goto cfg_scts_end;
		if(buf1[0] == CFG_SSL) {
			if (max!=0){
				buf1[n-1] = 0x00;
				strcpy((char *)sections[n_sections], buf1+1);
				if (n_sections>=max)
					break;
			}
			n_sections++;
		} 

	} 
	ret = n_sections;
cfg_scts_end: 
	return ret;
} 


/*************************************************************
Function: ¥h°£¦r²Å¦ê¥kÃäªºªÅ¦r²Å
Input:  char * buf ¦r²Å¦ê«ü°w
Output: 
Return: ¦r²Å¦ê«ü°w
Note: 
*************************************************************/
char * ini_str_trim_r(char * buf)
{
	int len,i;
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);
//	tmp = (char *)malloc(len);
	
	memset(tmp,0x00,len);
	for(i = 0;i < len;i++) {
		if (buf[i] !=' ')
			break;
	}
	if (i < len) {
		strncpy(tmp,(buf+i),(len-i));
	}
	strncpy(buf,tmp,len);
//	free(tmp);
	return buf;
}

/*************************************************************
Function: ¥h°£¦r²Å¦ê¥ªÃäªºªÅ¦r²Å
Input:  char * buf ¦r²Å¦ê«ü°w
Output: 
Return: ¦r²Å¦ê«ü°w
Note: 
*************************************************************/
char * ini_str_trim_l(char * buf)
{
	int len,i;	
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);
	memset(tmp,0x00,len);

	for(i = 0;i < len;i++) {
		if (buf[len-i-1] !=' ')
			break;
	}
	if (i < len) {
		strncpy(tmp,buf,len-i);
	}
	strncpy(buf,tmp,len);
	//free(tmp);
	return buf;
}
/*************************************************************
Function: ±q¤å¥ó¤¤Åª¨ú¤@¦æ
Input:  FILE *fp ¤å¥ó¥y¬`¡Fint maxlen ½w½Ä°Ï³Ì¤jªø«×
Output: char *buffer ¤@¦æ¦r²Å¦ê
Return: >0	¹ê»ÚÅªªºªø«×
	-1	¤å¥óµ²§ô
	-2	Åª¤å¥ó¥X¿ù
Note: 
*************************************************************/
static int ini_file_get_line(char *filedata, char *buffer, int maxlen)
{
	int i=0;
	int j=0;
	int iRetNum=-1;
	char ch1='\0'; 

	for(i=0, j=0; i<maxlen; j++) { 
		ch1 = filedata[j];
		iRetNum = j+1;
		if(ch1 == '\n' || ch1 == '\r') //line end
		{
			ch1 = filedata[j+1];
			if(ch1 == '\n' || ch1 == '\r') 
			{
				iRetNum++;
			}

			break; // ´«¦æ
		}else if(ch1 == 0x00) 
		{
			iRetNum = -1;
			break; //file end
		}
		else
		{
			buffer[i++] = ch1;    
		}
	} 
	buffer[i] = '\0'; 

	return iRetNum;
} 
/*************************************************************
Function: ¤ÀÂ÷key©Mvalue
			key=val
			jack   =   liaoyuewang 
			|      |   | 
			k1     k2  i 
Input:  char *buf
Output: char **key, char **val
Return: 1 --- ok 
	0 --- blank line 
	-1 --- no key, "= val" 
	-2 --- only key, no '=' 
Note: 
*************************************************************/
/*static int  ini_split_key_value(char *buf, char **key, char **val)
{
	int  i, k1, k2, n; 

	//FTS_DBG("section = %s \n",section);
	//FTS_DBG("Enter ini_split_key_value() \n");
	
	if((n = strlen((char *)buf)) < 1)
		return 0; 
	for(i = 0; i < n; i++) 
		if(buf[i] != ' ' && buf[i] != '\t')
			break; 

	if(i >= n)
		return 0;

	if(buf[i] == '=')
		return -1;
	
	k1 = i;
	for(i++; i < n; i++) 
		if(buf[i] == '=') 
			break;

	if(i >= n)
		return -2;
	k2 = i;
	
	for(i++; i < n; i++)
		if(buf[i] != ' ' && buf[i] != '\t') 
			break; 

	buf[k2] = '\0'; 

	*key = buf + k1; 
	*val = buf + i; 
	return 1; 
}*/ 

int my_atoi(const char *str)
{
	int result = 0;
	int signal = 1; 
	if((*str>='0'&&*str<='9')||*str=='-'||*str=='+') {
		if(*str=='-'||*str=='+') { 
			if(*str=='-')
				signal = -1; /*¿é¤J­t¼Æ*/
			str++;
		}
	}
	else 
		return 0;

	while(*str>='0' && *str<='9')
	   result = result*10 + (*str++ - '0' );
	
	return signal*result;
}

int isspace(int x)  
{  
	if(x==' '||x=='\t'||x=='\n'||x=='\f'||x=='\b'||x=='\r')  
		return 1;  
	else   
		return 0;  
}  
  
int isdigit(int x)  
{  
	if(x<='9' && x>='0')           
		return 1;   
	else   
		return 0;  
  
} 

static long atol(char *nptr)
{
	int c; /* current char */
	long total; /* current total */
	int sign; /* if ''-'', then negative, otherwise positive */
	/* skip whitespace */
	while ( isspace((int)(unsigned char)*nptr) )
		++nptr;
	c = (int)(unsigned char)*nptr++;
	sign = c; /* save sign indication */
	if (c == '-' || c == '+')
		c = (int)(unsigned char)*nptr++; /* skip sign */
	total = 0;
	while (isdigit(c)) {
		total = 10 * total + (c - '0'); /* accumulate digit */
		c = (int)(unsigned char)*nptr++; /* get next char */
	}
	if (sign == '-')
		return -total;
	else
		return total; /* return result, negated if necessary */
}
/***
*int atoi(char *nptr) - Convert string to long
*
*Purpose:
* Converts ASCII string pointed to by nptr to binary.
* Overflow is not detected. Because of this, we can just use
* atol().
*
*Entry:
* nptr = ptr to string to convert
*
*Exit:
* return int value of the string
*
*Exceptions:
* None - overflow is not detected.
*
*******************************************************************************/
int atoi(char *nptr)
{
	return (int)atol(nptr);
}

int init_key_data(void)
{
	int i = 0;
	
	g_used_key_num = 0;
		
	for(i = 0; i < MAX_KEY_NUM; i++)
	{
		memset(g_st_ini_file_data[i].pSectionName, 0, MAX_KEY_NAME_LEN);
		memset(g_st_ini_file_data[i].pKeyName, 0, MAX_KEY_NAME_LEN);
		memset(g_st_ini_file_data[i].pKeyValue, 0, MAX_KEY_VALUE_LEN);	
		g_st_ini_file_data[i].iSectionNameLen = 0;
		g_st_ini_file_data[i].iKeyNameLen = 0;
		g_st_ini_file_data[i].iKeyValueLen = 0;
	}
	return 1;
}

/*************************************************************
Function:Åª¨ú©Ò¦³ªº°Ñ¼Æ¤Î¨ä­È¨ìµ²ºcÅé¸Ì
Return: ªð¦^key­Ó¼Æ¡C­Y¥X¿ù¡Aªð¦^­t¼Æ¡C
		-10			¤å¥ó¥´¶}¥X¿ù
		-12			¤å¥óÅª¨ú¿ù»~
		-14			¤å¥ó®æ¦¡¿ù»~
Note: 
*************************************************************/
int ini_get_key_data(char *filedata)
{
	//FILE *fp; 
	char buf1[MAX_CFG_BUF + 1]={0}; 
	int n=0;
	int ret=0;  //n_sections = 0, 
	int dataoff = 0;
	int iEqualSign = 0;
	int i = 0;
	char tmpSectionName[MAX_CFG_BUF + 1]={0}; 
	
//	if((fp = fopen(filename, "rb")) == NULL) 
//		return CFG_ERR_OPEN_FILE; 

	init_key_data();/*init*/

	g_used_key_num = 0;
	while(1) {/*·j§ä¶µsection */
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);
		
		if(n < -1) 
			goto cfg_scts_end; 
		if(n < 0)
			break;/* ¤å¥ó§À */ 
		
		dataoff += n;
		
		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if(n == 0 || buf1[0] == CFG_NTS) 
			continue;       /* ªÅ¦æ ©Î µùÄÀ¦æ */ 
		ret = CFG_ERR_FILE_FORMAT;
		//get section name
		if(n > 2 && ((buf1[0] == CFG_SSL && buf1[n-1] != CFG_SSR)))
		{
			printk("Bad Section:%s\n\n", buf1);
			goto cfg_scts_end;//bad section
		}
	
		
		if(buf1[0] == CFG_SSL) {	
			g_st_ini_file_data[g_used_key_num].iSectionNameLen = n-2;	
			if(MAX_KEY_NAME_LEN < g_st_ini_file_data[g_used_key_num].iSectionNameLen)
			{
				ret = CFG_ERR_OUT_OF_LEN;
				printk("MAX_KEY_NAME_LEN: CFG_ERR_OUT_OF_LEN\n\n");
				goto cfg_scts_end;			
			}
		
			buf1[n-1] = 0x00;
			strcpy((char *)tmpSectionName, buf1+1);
			//printk("Section Name:%s, Len:%d\n\n", tmpSectionName, n-2);

			continue;
		} 
		//get section name end

		strcpy( g_st_ini_file_data[g_used_key_num].pSectionName, tmpSectionName);
		g_st_ini_file_data[g_used_key_num].iSectionNameLen = strlen(tmpSectionName);	
		
		iEqualSign = 0;
		for(i=0; i < n; i++)
		{
			if(buf1[i] == CFG_EQS )
			{
				iEqualSign = i;
				break;
			}
		}
		if(0 == iEqualSign)
			continue;
		g_st_ini_file_data[g_used_key_num].iKeyNameLen = iEqualSign;	
		if(MAX_KEY_NAME_LEN < g_st_ini_file_data[g_used_key_num].iKeyNameLen)
			{
				ret = CFG_ERR_OUT_OF_LEN;
				printk("MAX_KEY_NAME_LEN: CFG_ERR_OUT_OF_LEN\n\n");
				goto cfg_scts_end;	
			}
		memcpy(g_st_ini_file_data[g_used_key_num].pKeyName,
		buf1, g_st_ini_file_data[g_used_key_num].iKeyNameLen);

		/*µ¥¸¹«á¤@¬q½áµ¹Áä­È*/
		g_st_ini_file_data[g_used_key_num].iKeyValueLen = n-iEqualSign-1;
		if(MAX_KEY_VALUE_LEN < g_st_ini_file_data[g_used_key_num].iKeyValueLen)
			{
				ret = CFG_ERR_OUT_OF_LEN;
				printk("MAX_KEY_VALUE_LEN: CFG_ERR_OUT_OF_LEN\n\n");
				goto cfg_scts_end;
			}
		memcpy(g_st_ini_file_data[g_used_key_num].pKeyValue,
		buf1+ iEqualSign+1, g_st_ini_file_data[g_used_key_num].iKeyValueLen);
		
		ret = g_used_key_num;
		
		g_used_key_num++;
		if(MAX_KEY_NUM < g_used_key_num)
			{
				ret = CFG_ERR_TOO_MANY_KEY_NUM;
				printk("MAX_KEY_NUM: CFG_ERR_TOO_MANY_KEY_NUM\n\n");
				goto cfg_scts_end;
			}
	} 
cfg_scts_end: 
	return ret;
} 


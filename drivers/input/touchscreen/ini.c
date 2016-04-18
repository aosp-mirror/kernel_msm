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

char CFG_SSL = '[';
char CFG_SSR = ']';
char CFG_NIS = ':';
char CFG_NTS = '#';
char CFG_EQS = '=';

ST_INI_FILE_DATA g_st_ini_file_data[MAX_KEY_NUM];
int g_used_key_num = 0;

char *ini_str_trim_r(char *buf);
char *ini_str_trim_l(char *buf);
static int ini_file_get_line(char *filedata, char *buffer, int maxlen);
static long atol(char *nptr);

int ini_get_key(char *filedata, char *section, char *key, char *value)
{
	int i = 0;
	int ret = -2;
	for (i = 0; i < g_used_key_num; i++) {
			if (strncmp(section, g_st_ini_file_data[i].pSectionName, g_st_ini_file_data[i].iSectionNameLen) != 0)
				 continue;

			if (strncmp(key, g_st_ini_file_data[i].pKeyName,
				 g_st_ini_file_data[i].iKeyNameLen) == 0) {
				memcpy(value, g_st_ini_file_data[i].pKeyValue, g_st_ini_file_data[i].iKeyValueLen);
				ret = 0;
				break;
			}
	}
	return ret;
}

char *ini_str_trim_r(char *buf)
{
	int len, i;
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);

	memset(tmp, 0x00, len);
	for (i = 0; i < len; i++) {
		if (buf[i] != ' ')
			break;
	}
	if (i < len)
		strlcpy(tmp, (buf+i), 512);

	strlcpy(buf, tmp, 512);

	return buf;
}

char *ini_str_trim_l(char *buf)
{
	int len, i;
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);
	memset(tmp, 0x00, len);

	for (i = 0; i < len; i++) {
		if (buf[len-i-1] != ' ')
			break;
	}
	if (i < len)
		strlcpy(tmp, buf, 512);

	strlcpy(buf, tmp, 512);

	return buf;
}

static int ini_file_get_line(char *filedata, char *buffer, int maxlen)
{
	int i = 0;
	int j = 0;
	int iRetNum = -1;
	char ch1 = '\0';

	for (i = 0, j = 0; i < maxlen; j++) {
		ch1 = filedata[j];
		iRetNum = j+1;
		if (ch1 == '\n' || ch1 == '\r') {
			ch1 = filedata[j+1];
			if (ch1 == '\n' || ch1 == '\r')
				iRetNum++;

			break;
		} else if (ch1 == 0x00) {
			iRetNum = -1;
			break;
		} else {
			buffer[i++] = ch1;
		}
	}
	buffer[i] = '\0';

	return iRetNum;
}

int isspace(int x)
{
	if (x == ' ' || x == '\t' || x == '\n' || x == '\f' || x == '\b' || x == '\r')
		return 1;
	else
		return 0;
}

int isdigit(int x)
{
	if (x <= '9' && x >= '0')
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
	while (isspace((int)(unsigned char)*nptr))
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

int atoi(char *nptr)
{
	return (int)atol(nptr);
}

int init_key_data(void)
{
	int i = 0;

	g_used_key_num = 0;

	for (i = 0; i < MAX_KEY_NUM; i++) {
		memset(g_st_ini_file_data[i].pSectionName, 0, MAX_KEY_NAME_LEN);
		memset(g_st_ini_file_data[i].pKeyName, 0, MAX_KEY_NAME_LEN);
		memset(g_st_ini_file_data[i].pKeyValue, 0, MAX_KEY_VALUE_LEN);
		g_st_ini_file_data[i].iSectionNameLen = 0;
		g_st_ini_file_data[i].iKeyNameLen = 0;
		g_st_ini_file_data[i].iKeyValueLen = 0;
	}

	return 1;
}

int ini_get_key_data(char *filedata)
{
	char buf1[MAX_CFG_BUF + 1] = {0};
	int n = 0;
	int ret = 0;
	int dataoff = 0;
	int iEqualSign = 0;
	int i = 0;
	char tmpSectionName[MAX_CFG_BUF + 1] = {0};

	init_key_data();

	g_used_key_num = 0;
	while (1) {
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);

		if (n < -1)
			goto cfg_scts_end;
		if (n < 0)
			break;

		dataoff += n;

		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if (n == 0 || buf1[0] == CFG_NTS)
			continue;
		ret = CFG_ERR_FILE_FORMAT;

		if (n > 2 && ((buf1[0] == CFG_SSL && buf1[n-1] != CFG_SSR))) {
			pr_debug("[FTS] Bad Section:%s\n\n", buf1);
			goto cfg_scts_end;
		}

		if (buf1[0] == CFG_SSL) {
			g_st_ini_file_data[g_used_key_num].iSectionNameLen = n-2;
			if (MAX_KEY_NAME_LEN < g_st_ini_file_data[g_used_key_num].iSectionNameLen) {
				ret = CFG_ERR_OUT_OF_LEN;
				pr_debug("[FTS] MAX_KEY_NAME_LEN: CFG_ERR_OUT_OF_LEN\n\n");
				goto cfg_scts_end;
			}

			buf1[n-1] = 0x00;
			strlcpy((char *)tmpSectionName, buf1+1, MAX_CFG_BUF);

			continue;
		}

		strlcpy(g_st_ini_file_data[g_used_key_num].pSectionName, tmpSectionName, MAX_CFG_BUF);
		g_st_ini_file_data[g_used_key_num].iSectionNameLen = strlen(tmpSectionName);

		iEqualSign = 0;
		for (i = 0; i < n; i++) {
			if (buf1[i] == CFG_EQS) {
					iEqualSign = i;
					break;
			}
		}
		if (0 == iEqualSign)
			continue;

		g_st_ini_file_data[g_used_key_num].iKeyNameLen = iEqualSign;
		if (MAX_KEY_NAME_LEN < g_st_ini_file_data[g_used_key_num].iKeyNameLen) {
				ret = CFG_ERR_OUT_OF_LEN;
				pr_debug("[FTS] MAX_KEY_NAME_LEN: CFG_ERR_OUT_OF_LEN\n\n");
				goto cfg_scts_end;
		}
		memcpy(g_st_ini_file_data[g_used_key_num].pKeyName,
			buf1, g_st_ini_file_data[g_used_key_num].iKeyNameLen);

		g_st_ini_file_data[g_used_key_num].iKeyValueLen = n-iEqualSign-1;
		if (MAX_KEY_VALUE_LEN < g_st_ini_file_data[g_used_key_num].iKeyValueLen) {
				ret = CFG_ERR_OUT_OF_LEN;
				pr_debug("[FTS] MAX_KEY_VALUE_LEN: CFG_ERR_OUT_OF_LEN\n\n");
				goto cfg_scts_end;
		}
		memcpy(g_st_ini_file_data[g_used_key_num].pKeyValue,
			buf1 + iEqualSign + 1, g_st_ini_file_data[g_used_key_num].iKeyValueLen);

		ret = g_used_key_num;

		g_used_key_num++;
		if (MAX_KEY_NUM < g_used_key_num) {
				ret = CFG_ERR_TOO_MANY_KEY_NUM;
				pr_debug("[FTS] MAX_KEY_NUM: CFG_ERR_TOO_MANY_KEY_NUM\n\n");
				goto cfg_scts_end;
		}
	}

cfg_scts_end:

	return ret;
}

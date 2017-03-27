/* touch_sw49408.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
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
#define TS_MODULE "[sw49408]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw49408.h"
#include "touch_sw49408_abt.h"
#include "touch_sw49408_prd.h"

static const char *debug_type[] = {
	"Disable Type",
	"Buffer Type",
	"Always Report Type"
};
#define TCI_FAIL_NUM 17
static const char const *tci_debug_str[TCI_FAIL_NUM] = {
	"NONE",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP_LONG",
	"MULTI_FINGER",
	"DELAY_TIME",/* It means Over Tap */
	"TIMEOUT_INTER_TAP_SHORT",
	"PALM_STATE",
	"DEBUG8",
	"EDGE_SCRUBBED",
	"LONG_TIME_TAP",
	"ABNORMAL_TOUCH",
	"DEBUG12",
	"DEBUG13",
	"DEBUG14",
	"DEBUG15",
	"DEBUG16"
};
#define SWIPE_FAIL_NUM 17
static const char const *swipe_debug_str[SWIPE_FAIL_NUM] = {
	"ERROR",
	"1FINGER_FAST_RELEASE",
	"MULTI_FINGER",
	"FAST_SWIPE",
	"SLOW_SWIPE",
	"OUT_OF_ACTIVE_AREA",
	"RATIO_FAIL",
	"OUT_OF_START_AREA",
	"DEBUG8",
	"ABNORMAL_TOUCH",
	"DEBUG10",
	"PALM",
	"WRONG_DIRECTION",
	"DEBUG13",
	"DEBUG14",
	"DEBUG15",
	"DEBUG16"
};

int sw49408_xfer_msg(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	int addr_check = 0;
	int buf_cnt = 0;
	int ret = 0;
	int i = 0;

	for (i = 0; i < xfer->msg_count; i++) {
        buf_cnt = 0;
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;
		if (rx->addr >= command_start_addr && rx->addr
			<= command_end_addr)
			addr_check = 1;
		else
			addr_check = 0;

		if (rx->size) {
			if (addr_check == 1)
				tx->data[0] = ((rx->size > 4) ? 0x30 : 0x10);
			else
				tx->data[0] = ((rx->size > 4) ? 0x20 : 0x00);

			tx->data[buf_cnt++] |= ((rx->addr >> 8) & 0x0f);
			tx->data[buf_cnt++] = (rx->addr & 0xff);
			tx->data[buf_cnt++] = 0;
			tx->data[buf_cnt++] = 0;
			tx->data[buf_cnt++] = 0;
			tx->data[buf_cnt++] = 0;

			if (addr_check == 1) {
				for (; buf_cnt < 18;)
					tx->data[buf_cnt++] = 0;
			}

			tx->size = buf_cnt;
			rx->size += R_HEADER_SIZE;
		} else {
			if (tx->size > (MAX_XFER_BUF_SIZE - W_HEADER_SIZE)) {
				TOUCH_E("buffer overflow\n");
				ret = -EOVERFLOW;
				goto error;

			}

			tx->data[0] = 0x40;
			tx->data[0] |= ((tx->addr >> 8) & 0x0f);
			tx->data[1] = (tx->addr  & 0xff);
			memcpy(&tx->data[W_HEADER_SIZE], tx->buf, tx->size);
			tx->size += W_HEADER_SIZE;
		}
	}

	ret = touch_bus_xfer(dev, xfer);
	if (ret) {
		TOUCH_E("touch bus error : %d\n", ret);
		goto error;
	}

	for (i = 0; i < xfer->msg_count; i++) {
		rx = &xfer->data[i].rx;

		if (rx->size) {
			memcpy(rx->buf, rx->data + R_HEADER_SIZE,
				(rx->size - R_HEADER_SIZE));
		}
	}

error:
	for (i = 0; i < xfer->msg_count; i++) {
		rx = &xfer->data[i].rx;
		if (rx->size)
			rx->size = 0;
	}

	mutex_unlock(&d->spi_lock);

	return ret;
}

void sw49408_xfer_msg_ready(struct device *dev, u8 msg_cnt)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);

	mutex_lock(&d->spi_lock);

	ts->xfer->msg_count = msg_cnt;
}

int sw49408_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;
	int buf_cnt = 0;

	mutex_lock(&d->spi_lock);

	ts->tx_buf[0] = ((size > 4) ? 0x20 : 0x00);
	ts->tx_buf[buf_cnt++] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[buf_cnt++] = (addr & 0xff);
	ts->tx_buf[buf_cnt++] = 0;
	ts->tx_buf[buf_cnt++] = 0;
	ts->tx_buf[buf_cnt++] = 0;
	ts->tx_buf[buf_cnt++] = 0;

	msg.tx_buf = ts->tx_buf;
	msg.rx_buf = ts->rx_buf;
	msg.tx_size = R_HEADER_SIZE;
	msg.rx_size = R_HEADER_SIZE + size;

	ret = touch_bus_read(dev, &msg);

	if (ret) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->spi_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[R_HEADER_SIZE], size);
	mutex_unlock(&d->spi_lock);
	return 0;
}

int sw49408_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->spi_lock);
	ts->tx_buf[0] = 0x40;
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr  & 0xff);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_HEADER_SIZE + size;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	memcpy(&ts->tx_buf[W_HEADER_SIZE], data, size);

	ret = touch_bus_write(dev, &msg);
	mutex_unlock(&d->spi_lock);

	if (ret) {
		TOUCH_E("touch bus error : %d\n", ret);
		return ret;
	}

	return 0;
}

/*
static int sw49408_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			TOUCH_I("FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			TOUCH_I("FB_BLANK\n");
	}

	return 0;
}
*/
static int sw49408_reset_ctrl(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int data;

	TOUCH_TRACE();

	switch (ctrl) {
	default :
        case SW_RESET:
		TOUCH_I("%s : SW Reset\n", __func__);
	        data = 0;
	        sw49408_reg_write(dev, SPI_RST_CTL, &data, sizeof(int));
	        data = 1;
	        sw49408_reg_write(dev, SPI_RST_CTL, &data, sizeof(int));
		touch_msleep(ts->caps.sw_reset_delay);
		break;

	case HW_RESET:
		TOUCH_I("%s : HW Reset\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(1);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(ts->caps.hw_reset_delay);
		break;
	    break;
	}

	return 0;
}

static int sw49408_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		atomic_set(&d->init, IC_INIT_NEED);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_power_vio(dev, 0);
		touch_power_vdd(dev, 0);
		touch_msleep(1);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		touch_power_vdd(dev, 1);
		touch_power_vio(dev, 1);
		touch_gpio_direction_output(ts->reset_pin, 1);
		break;

	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	}

	return 0;
}

static void sw49408_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

static void sw49408_get_swipe_info(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	d->swipe.info[SWIPE_R].distance = 5;
	d->swipe.info[SWIPE_R].ratio_thres = 100;
	d->swipe.info[SWIPE_R].ratio_distance = 2;
	d->swipe.info[SWIPE_R].ratio_period = 5;
	d->swipe.info[SWIPE_R].min_time = 0;
	d->swipe.info[SWIPE_R].max_time = 150;
	d->swipe.info[SWIPE_R].area.x1 = 80;//401;
	d->swipe.info[SWIPE_R].area.y1 = 0;
	d->swipe.info[SWIPE_R].area.x2 = 1359;
	d->swipe.info[SWIPE_R].area.y2 = 2780;
	d->swipe.info[SWIPE_R].start_area.x1 = 80;//401;
	d->swipe.info[SWIPE_R].start_area.y1 = 0;
	d->swipe.info[SWIPE_R].start_area.x2 = 1359;
	d->swipe.info[SWIPE_R].start_area.y2 = 2780;
	d->swipe.info[SWIPE_R].wrong_direction_thres = 0;

	d->swipe.info[SWIPE_D].distance = 5;
	d->swipe.info[SWIPE_D].ratio_thres = 100;
	d->swipe.info[SWIPE_D].ratio_distance = 2;
	d->swipe.info[SWIPE_D].ratio_period = 5;
	d->swipe.info[SWIPE_D].min_time = 0;
	d->swipe.info[SWIPE_D].max_time = 150;
	d->swipe.info[SWIPE_D].area.x1 = 80;//401;
	d->swipe.info[SWIPE_D].area.y1 = 0;
	d->swipe.info[SWIPE_D].area.x2 = 1359;
	d->swipe.info[SWIPE_D].area.y2 = 2780;
	d->swipe.info[SWIPE_D].start_area.x1 = 80;//401;
	d->swipe.info[SWIPE_D].start_area.y1 = 0;
	d->swipe.info[SWIPE_D].start_area.x2 = 1359;
	d->swipe.info[SWIPE_D].start_area.y2 = 2780;
	d->swipe.info[SWIPE_D].wrong_direction_thres = 0;

	d->swipe.info[SWIPE_L].distance = 5;
	d->swipe.info[SWIPE_L].ratio_thres = 100;
	d->swipe.info[SWIPE_L].ratio_distance = 2;
	d->swipe.info[SWIPE_L].ratio_period = 5;
	d->swipe.info[SWIPE_L].min_time = 0;
	d->swipe.info[SWIPE_L].max_time = 150;
	d->swipe.info[SWIPE_L].area.x1 = 80;//401;
	d->swipe.info[SWIPE_L].area.y1 = 0;
	d->swipe.info[SWIPE_L].area.x2 = 1359;
	d->swipe.info[SWIPE_L].area.y2 = 2780;
	d->swipe.info[SWIPE_L].start_area.x1 = 80;//401;
	d->swipe.info[SWIPE_L].start_area.y1 = 0;
	d->swipe.info[SWIPE_L].start_area.x2 = 1359;
	d->swipe.info[SWIPE_L].start_area.y2 = 2780;
	d->swipe.info[SWIPE_L].wrong_direction_thres = 0;

	d->swipe.info[SWIPE_U].distance = 15;
	d->swipe.info[SWIPE_U].ratio_thres = 150;
	d->swipe.info[SWIPE_U].ratio_distance = 2;
	d->swipe.info[SWIPE_U].ratio_period = 0;
	d->swipe.info[SWIPE_U].min_time = 0;
	d->swipe.info[SWIPE_U].max_time = 150;
	d->swipe.info[SWIPE_U].area.x1 = 80;//401;
	d->swipe.info[SWIPE_U].area.y1 = 0;
	d->swipe.info[SWIPE_U].area.x2 = 1359;
	d->swipe.info[SWIPE_U].area.y2 = 2780;
	d->swipe.info[SWIPE_U].start_area.x1 = 80;//401;
	d->swipe.info[SWIPE_U].start_area.y1 = 2400;
	d->swipe.info[SWIPE_U].start_area.x2 = 1359;
	d->swipe.info[SWIPE_U].start_area.y2 = 2780;
	d->swipe.info[SWIPE_U].wrong_direction_thres = 5;

	d->swipe.mode = 0;
		/*SWIPE_RIGHT_BIT | SWIPE_DOWN_BIT |
			  SWIPE_LEFT_BIT | SWIPE_UP_BIT; */
}

int sw49408_ic_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	u32 version = 0;
	u32 bootmode = 0;
	u32 product[2] = {0};
	char ver_str[32] = {0};

	sw49408_xfer_msg_ready(dev, 7);

	ts->xfer->data[0].rx.addr = d->reg_info.r_chip_info_spi_addr
					+ tc_version;
	ts->xfer->data[0].rx.buf = (u8 *)&version;
	ts->xfer->data[0].rx.size = sizeof(version);

	ts->xfer->data[1].rx.addr = d->reg_info.r_chip_info_spi_addr
					+ tc_product_id1;
	ts->xfer->data[1].rx.buf = (u8 *)&product[0];
	ts->xfer->data[1].rx.size = sizeof(product);

	ts->xfer->data[2].rx.addr = spr_boot_st;
	ts->xfer->data[2].rx.buf = (u8 *)&bootmode;
	ts->xfer->data[2].rx.size = sizeof(bootmode);

	ts->xfer->data[3].rx.addr = d->reg_info.r_pt_info_spi_addr +
					pt_info_fpc_type;
	ts->xfer->data[3].rx.buf = (u8 *)&d->ic_info.fpc;
	ts->xfer->data[3].rx.size = sizeof(d->ic_info.fpc);

	ts->xfer->data[4].rx.addr = d->reg_info.r_pt_info_spi_addr +
					pt_info_lcm_type;
	ts->xfer->data[4].rx.buf = (u8 *)&d->ic_info.lcm;
	ts->xfer->data[4].rx.size = sizeof(d->ic_info.lcm);

	ts->xfer->data[5].rx.addr = d->reg_info.r_pt_info_spi_addr +
					pt_info_lot_num;
	ts->xfer->data[5].rx.buf = (u8 *)&d->ic_info.lot;
	ts->xfer->data[5].rx.size = sizeof(d->ic_info.lot);

	ts->xfer->data[6].rx.addr = chip_rev_addr;
	ts->xfer->data[6].rx.buf = (u8 *)&d->ic_info.chip_revision;
	ts->xfer->data[6].rx.size = sizeof(d->ic_info.chip_revision);

	sw49408_xfer_msg(dev, ts->xfer);

	d->ic_info.version.build = ((version >> 12) & 0xF);
	d->ic_info.version.major = ((version >> 8) & 0xF);
	d->ic_info.version.minor = version & 0xFF;
	d->ic_info.chip_revision = d->ic_info.chip_revision & 0xF;
	memcpy(&d->ic_info.product_id[0], &product[0], sizeof(product));

	if (d->ic_info.version.build) {
		snprintf(ver_str, 32, "v%d.%02d.%d",
			d->ic_info.version.major, d->ic_info.version.minor,
			d->ic_info.version.build);
	} else {
		snprintf(ver_str, 32, "v%d.%02d",
			d->ic_info.version.major, d->ic_info.version.minor);
	}

	TOUCH_I("==Print PT info Data==\n");
	TOUCH_I("version : %s, chip : %d, protocol : %d\n",
		ver_str, (version >> 16) & 0xFF, (version >> 24) & 0xFF);
	TOUCH_I("chip_rev : %d, fpc : %d, lcm : %d, lot : %d\n",
		d->ic_info.chip_revision, d->ic_info.fpc, d->ic_info.lcm, d->ic_info.lot);
	TOUCH_I("product id : %s\n", d->ic_info.product_id);
	TOUCH_I("flash boot : %s, %s, crc : %s\n",
		(bootmode >> 1 & 0x1) ? "BUSY" : "idle",
		(bootmode >> 2 & 0x1) ? "done" : "BOOTING",
		(bootmode >> 3 & 0x1) ? "ERROR" : "ok");

	if ((((version >> 16) & 0xFF) != 9) ||
			(((version >> 24) & 0xFF) != 4)) {
		TOUCH_I("FW is in abnormal state because of ESD or something.\n");
		ret = -EAGAIN;
	}

	return ret;
}

int sw49408_te_info(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 count = 0;
	u32 ms = 0;
	u32 hz = 0;
	int ret = 0;
	char te_log[64] ={0};

	if (buf == NULL)
		buf = te_log;

	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, 63, "not support on u%d\n",
				d->lcd_mode);
		return ret;
	}

	sw49408_reg_read(dev, d->reg_info.r_tc_sts_spi_addr +
			tc_rtc_te_interval_cnt, (u8 *)&count, sizeof(u32));

	if (count > 100 && count < 10000) {
		ms = (count * 100 * 1000) / 32764;
		hz = (32764 * 100) / count;

		ret = snprintf(buf + ret, 63,
			"%s : %d, %d.%02d ms, %d.%02d hz\n", __func__, count,
			ms / 100, ms % 100, hz / 100, hz % 100);

		TOUCH_I("%s", buf);
	}

	return ret;
}

static void sw49408_clear_q_sensitivity(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	d->q_sensitivity = 0;
	sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			QCOVER_SENSITIVITY, &d->q_sensitivity, sizeof(u32));

	TOUCH_I("%s : %s(%d)\n", __func__, "NORMAL", d->q_sensitivity);
}

static int sw49408_get_tci_data(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	u8 i = 0;
	u32 rdata[MAX_LPWG_CODE];

	if (!count)
		return 0;

	ts->lpwg.code_num = count;

	memcpy(&rdata, d->info.data, sizeof(u32) * count);

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = rdata[i] & 0xffff;
		ts->lpwg.code[i].y = (rdata[i] >> 16) & 0xffff;

		if ((ts->lpwg.mode >= LPWG_PASSWORD) &&
				(ts->role.hide_coordinate))
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int sw49408_get_swipe_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 rdata[3];
	int count = 1;

	/* swipe_info */
	/* start (X, Y), end (X, Y), time = 2bytes * 5 = 10 bytes */
	memcpy(&rdata, d->info.data, sizeof(u32) * 3);

	TOUCH_I("Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
			rdata[0] & 0xffff, rdata[0] >> 16,
			rdata[1] & 0xffff, rdata[1] >> 16,
			rdata[2] & 0xffff);

	ts->lpwg.code_num = count;
	ts->lpwg.code[0].x = rdata[1] & 0xffff;
	ts->lpwg.code[0].y = rdata[1]  >> 16;

	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static void set_debug_reason(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 wdata[2] = {0, };
	u32 start_addr = 0x0;
	u32 swipe_data = 0x0;
	int i = 0;
	int ret = 0;

	if (type == SWIPE) {
		swipe_data = d->swipe_debug_type;
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_FAIL_DEBUG_W, &swipe_data, sizeof(swipe_data));
		if (ret < 0)
			TOUCH_I("set swipe fail reason : %d\n", ret);
	} else {
		wdata[0] = (u32)type;
		wdata[0] |= (d->tci_debug_type == 1) ? 0x01 << 2 : 0x01 << 3;
		wdata[1] = TCI_DEBUG_ALL;
		TOUCH_I("TCI%d-type:%d\n", type + 1, wdata[0]);

		sw49408_xfer_msg_ready(dev, 2);
		start_addr = d->reg_info.r_abt_cmd_spi_addr + TCI_FAIL_DEBUG_W;
		for (i = 0; i < 2; i++) {
			ts->xfer->data[i].tx.addr = start_addr + i;
			ts->xfer->data[i].tx.buf = (u8 *)&wdata[i];
			ts->xfer->data[i].tx.size = sizeof(u32);
		}

		sw49408_xfer_msg(dev, ts->xfer);
	}

	return;
}

static int sw49408_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data[7];
	int i = 0;
	u32 start_addr = 0x0;

	if (d->tci_debug_type != 0)
		set_debug_reason(dev, TCI_1);

	lpwg_data[0] = ts->tci.mode;
	lpwg_data[1] = info1->tap_count | (info2->tap_count << 16);
	lpwg_data[2] = info1->min_intertap | (info2->min_intertap << 16);
	lpwg_data[3] = info1->max_intertap | (info2->max_intertap << 16);
	lpwg_data[4] = info1->touch_slop | (info2->touch_slop << 16);
	lpwg_data[5] = info1->tap_distance | (info2->tap_distance << 16);
	lpwg_data[6] = info1->intr_delay | (info2->intr_delay << 16);

	sw49408_xfer_msg_ready(dev, 7);
	start_addr = d->reg_info.r_abt_cmd_spi_addr + TCI_ENABLE_W;
	for (i = 0; i < 7; i++) {
		ts->xfer->data[i].tx.addr = start_addr + i;
		ts->xfer->data[i].tx.buf = (u8 *)&lpwg_data[i];
		ts->xfer->data[i].tx.size = sizeof(u32);
	}

	return sw49408_xfer_msg(dev, ts->xfer);
}

static int sw49408_tci_password(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	if (d->tci_debug_type != 0)
		set_debug_reason(dev, TCI_2);

	return sw49408_tci_knock(dev);
}

static int sw49408_tci_active_area(struct device *dev,
		u32 x1, u32 y1, u32 x2, u32 y2)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	u32 start_addr = d->reg_info.r_abt_cmd_spi_addr + ACT_AREA_X1_W;
	u32 areas[4] = {x1 | (x1 << 16), y1 | (y1 << 16),
			x2 | (x2 << 16), y2 | (y2 << 16)};
	int i = 0;

	sw49408_xfer_msg_ready(dev, 4);
	for (i = 0; i < 4; i++) {
		ts->xfer->data[i].tx.addr = start_addr + i;
		ts->xfer->data[i].tx.buf = (u8 *)&areas[i];
		ts->xfer->data[i].tx.size = sizeof(u32);
	}

	ret = sw49408_xfer_msg(dev, ts->xfer);

	return ret;
}

static void sw49408_tci_area_set(struct device *dev, int cover_status)
{
	if (cover_status == QUICKCOVER_CLOSE) {
		sw49408_tci_active_area(dev, 0, 0, 0, 0);
		TOUCH_I("LPWG Active Area - QUICKCOVER_CLOSE\n");
	} else {
		sw49408_tci_active_area(dev, 80, 0, 1359, 2780);
		TOUCH_I("LPWG Active Area - NORMAL\n");
	}
}

static int sw49408_tci_control(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data;
	int ret = 0;

	switch (type) {
	case ENABLE_CTRL:
		lpwg_data = ts->tci.mode;
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			TCI_ENABLE_W, &lpwg_data, sizeof(lpwg_data));
		break;

	case TAP_COUNT_CTRL:
		lpwg_data = info1->tap_count | (info2->tap_count << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			TAP_COUNT_W,&lpwg_data, sizeof(lpwg_data));
		break;

	case MIN_INTERTAP_CTRL:
		lpwg_data = info1->min_intertap | (info2->min_intertap << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			MIN_INTERTAP_W, &lpwg_data, sizeof(lpwg_data));
		break;

	case MAX_INTERTAP_CTRL:
		lpwg_data = info1->max_intertap | (info2->max_intertap << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			MAX_INTERTAP_W, &lpwg_data, sizeof(lpwg_data));
		break;

	case TOUCH_SLOP_CTRL:
		lpwg_data = info1->touch_slop | (info2->touch_slop << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			TOUCH_SLOP_W, &lpwg_data, sizeof(lpwg_data));
		break;

	case TAP_DISTANCE_CTRL:
		lpwg_data = info1->tap_distance | (info2->tap_distance << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			TAP_DISTANCE_W, &lpwg_data, sizeof(lpwg_data));
		break;

	case INTERRUPT_DELAY_CTRL:
		lpwg_data = info1->intr_delay | (info2->intr_delay << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			INT_DELAY_W, &lpwg_data, sizeof(lpwg_data));
		break;

	case ACTIVE_AREA_CTRL:
		ret = sw49408_tci_active_area(dev,
				ts->tci.area.x1,
				ts->tci.area.y1,
				ts->tci.area.x2,
				ts->tci.area.y2);
		break;

	default:
		break;
	}

	return ret;
}

static int sw49408_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	int ret = 0;

	switch (mode) {
	case LPWG_NONE:
		ts->tci.mode = 0;
		ret = sw49408_tci_control(dev, ENABLE_CTRL);
		break;

	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		ret = sw49408_tci_knock(dev);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x01 | (0x01 << 16);
		info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
		info1->tap_distance = 7;

		ret = sw49408_tci_password(dev);
		break;

	case LPWG_PASSWORD_ONLY:
		ts->tci.mode = 0x01 << 16;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		ret = sw49408_tci_password(dev);
		break;

	default:
		TOUCH_I("Unknown lpwg control case\n");
		break;
	}

	TOUCH_I("sw49408_lpwg_control mode = %d\n", mode);

	return ret;
}

static int sw49408_swipe_active_area(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct swipe_info *right = &d->swipe.info[SWIPE_R];
	struct swipe_info *down = &d->swipe.info[SWIPE_D];
	struct swipe_info *left = &d->swipe.info[SWIPE_L];
	struct swipe_info *up = &d->swipe.info[SWIPE_U];
	u32 active_area[4] = {0x0, };
	int ret = 0;

	active_area[0] = (right->area.x1) | (left->area.x1 << 16);
	active_area[1] = (right->area.y1) | (left->area.y1 << 16);
	active_area[2] = (right->area.x2) | (left->area.x2 << 16);
	active_area[3] = (right->area.y2) | (left->area.y2 << 16);

	ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
		SWIPE_HORIZONTAL_ACT_AREA_X1_W, active_area, sizeof(active_area));

	active_area[0] = (down->area.x1) | (up->area.x1 << 16);
	active_area[1] = (down->area.y1) | (up->area.y1 << 16);
	active_area[2] = (down->area.x2) | (up->area.x2 << 16);
	active_area[3] = (down->area.y2) | (up->area.y2 << 16);

	ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
		SWIPE_VERTICAL_ACT_AREA_X1_W, active_area, sizeof(active_area));

	return ret;
}

static int sw49408_swipe_control(struct device *dev, int type)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	struct swipe_info *right = &d->swipe.info[SWIPE_R];
	struct swipe_info *down = &d->swipe.info[SWIPE_D];
	struct swipe_info *left = &d->swipe.info[SWIPE_L];
	struct swipe_info *up = &d->swipe.info[SWIPE_U];
	u32 swipe_data = 0;
	int ret = 0;

	switch (type) {
	case SWIPE_ENABLE_CTRL:
		swipe_data = d->swipe.mode;
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_ENABLE_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_DISABLE_CTRL:
		swipe_data = 0;
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_ENABLE_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_DIST_CTRL:
		swipe_data = (right->distance) | (down->distance << 8) |
				(left->distance << 16) | (up->distance << 24);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_DIST_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_THR_CTRL:
		swipe_data = (right->ratio_thres) | (down->ratio_thres << 8) |
				(left->ratio_thres << 16) | (up->ratio_thres << 24);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_RATIO_THR_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_DIST_CTRL:
		swipe_data = (right->ratio_distance) | (down->ratio_distance << 8) |
				(left->ratio_distance << 16) | (up->ratio_distance << 24);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				SWIPE_RATIO_DIST_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_PERIOD_CTRL:
		swipe_data = (right->ratio_period) | (down->ratio_period << 8) |
			(left->ratio_period << 16) | (up->ratio_period << 24);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				SWIPE_RATIO_PERIOD_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_TIME_MIN_CTRL:
		swipe_data = (right->min_time) | (left->min_time << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_HORIZONTAL_TIME_MIN_W, &swipe_data, sizeof(swipe_data));
		swipe_data = (down->min_time) | (up->min_time << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_VERTICAL_TIME_MIN_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_TIME_MAX_CTRL:
		swipe_data = (right->max_time) | (left->max_time << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_HORIZONTAL_TIME_MAX_W, &swipe_data, sizeof(swipe_data));
		swipe_data = (down->max_time) | (up->max_time << 16);
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			SWIPE_VERTICAL_TIME_MAX_W, &swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_AREA_CTRL:
		ret = sw49408_swipe_active_area(dev);
		break;
	default:
		break;
	}

	return ret;
}

static int sw49408_swipe_mode(struct device *dev, u8 lcd_mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct swipe_info *right = &d->swipe.info[SWIPE_R];
	struct swipe_info *down = &d->swipe.info[SWIPE_D];
	struct swipe_info *left = &d->swipe.info[SWIPE_L];
	struct swipe_info *up = &d->swipe.info[SWIPE_U];
	u32 swipe_data[12] = {0x0, };
	int ret = 0;
	int i = 0;
	u32 start_addr = 0x0;

	if (!d->swipe.mode)
		return ret;

	if (lcd_mode == LCD_MODE_U3) {
		ret = sw49408_swipe_control(dev, SWIPE_DISABLE_CTRL);
		TOUCH_I("swipe disable\n");
	} else {
		swipe_data[0] = d->swipe.mode;
		swipe_data[1] = (right->distance) | (down->distance << 8) |
					(left->distance << 16) | (up->distance << 24);
		swipe_data[2] = (right->ratio_thres) | (down->ratio_thres << 8) |
				(left->ratio_thres << 16) | (up->ratio_thres << 24);
		swipe_data[3] = (right->ratio_distance) | (down->ratio_distance << 8) |
				(left->ratio_distance << 16) | (up->ratio_distance << 24);
		swipe_data[4] = (right->ratio_period) | (down->ratio_period << 8) |
			(left->ratio_period << 16) | (up->ratio_period << 24);
		swipe_data[5] = (right->min_time) | (left->min_time << 16);
		swipe_data[6] = (right->max_time) | (left->max_time << 16);
		swipe_data[7] = (right->area.x1) | (left->area.x1 << 16);
		swipe_data[8] = (right->area.y1) | (left->area.y1 << 16);
		swipe_data[9] = (right->area.x2) | (left->area.x2 << 16);
		swipe_data[10] = (right->area.y2) | (left->area.y2 << 16);
		swipe_data[11] = (right->wrong_direction_thres) | (down->wrong_direction_thres << 8) |
			(left->wrong_direction_thres << 16) | (up->wrong_direction_thres << 24);

		sw49408_xfer_msg_ready(dev, 12);
		start_addr = d->reg_info.r_abt_cmd_spi_addr + SWIPE_ENABLE_W;
		for (i = 0; i < 12; i++) {
			ts->xfer->data[i].tx.addr = start_addr + i;
			ts->xfer->data[i].tx.buf = (u8 *)&swipe_data[i];
			ts->xfer->data[i].tx.size = sizeof(u32);
		}
		ret = sw49408_xfer_msg(dev, ts->xfer);

		swipe_data[0] = (down->min_time) | (up->min_time << 16);
		swipe_data[1] = (down->max_time) | (up->max_time << 16);
		swipe_data[2] = (down->area.x1) | (up->area.x1 << 16);
		swipe_data[3] = (down->area.y1) | (up->area.y1 << 16);
		swipe_data[4] = (down->area.x2) | (up->area.x2 << 16);
		swipe_data[5] = (down->area.y2) | (up->area.y2 << 16);

		sw49408_xfer_msg_ready(dev, 6);
		start_addr = d->reg_info.r_abt_cmd_spi_addr + SWIPE_VERTICAL_TIME_MIN_W;
		for (i = 0; i < 6; i++) {
			ts->xfer->data[i].tx.addr = start_addr + i;
			ts->xfer->data[i].tx.buf = (u8 *)&swipe_data[i];
			ts->xfer->data[i].tx.size = sizeof(u32);
		}
		ret = sw49408_xfer_msg(dev, ts->xfer);

		swipe_data[0] = (right->start_area.x1) | (left->start_area.x1 << 16);
		swipe_data[1] = (right->start_area.y1) | (left->start_area.y1 << 16);
		swipe_data[2] = (right->start_area.x2) | (left->start_area.x2 << 16);
		swipe_data[3] = (right->start_area.y2) | (left->start_area.y2 << 16);
		swipe_data[4] = (down->start_area.x1) | (up->start_area.x1 << 16);
		swipe_data[5] = (down->start_area.y1) | (up->start_area.y1 << 16);
		swipe_data[6] = (down->start_area.x2) | (up->start_area.x2 << 16);
		swipe_data[7] = (down->start_area.y2) | (up->start_area.y2 << 16);

		sw49408_xfer_msg_ready(dev, 8);
		start_addr = d->reg_info.r_abt_cmd_spi_addr + SWIPE_HORIZONTAL_START_AREA_X1_W;
		for (i = 0; i < 8; i++) {
			ts->xfer->data[i].tx.addr = start_addr + i;
			ts->xfer->data[i].tx.buf = (u8 *)&swipe_data[i];
			ts->xfer->data[i].tx.size = sizeof(u32);
		}
		ret = sw49408_xfer_msg(dev, ts->xfer);

		if (d->swipe_debug_type)
			set_debug_reason(dev, SWIPE);

		TOUCH_I("swipe enable\n");
	}

	return ret;
}

static int sw49408_clock(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);

	if (onoff) {
		sw49408_reg_write(dev, SPI_OSC_CTL, &onoff, sizeof(onoff));
		sw49408_reg_write(dev, SPI_CLK_CTL, &onoff, sizeof(onoff));
		atomic_set(&ts->state.sleep, IC_NORMAL);
	} else {
		if (d->lcd_mode == LCD_MODE_U0) {
			sw49408_reg_write(dev, SPI_CLK_CTL, &onoff,
								sizeof(onoff));
			sw49408_reg_write(dev, SPI_OSC_CTL, &onoff,
								sizeof(onoff));
			atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
		}
	}

	TOUCH_I("sw49408_clock -> %s\n",
		onoff ? "ON" : d->lcd_mode == LCD_MODE_U0 ? "OFF" : "SKIP");

	return 0;
}

int sw49408_tc_driving(struct device *dev, int mode)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 ctrl = 0;
	u32 rst_cnt = 0;
	u8 rdata;

	d->driving_mode = mode;
	switch (mode) {
	case LCD_MODE_U0:
		ctrl = 0x01;
		break;

	case LCD_MODE_U2_UNBLANK:
		ctrl = 0x101;
		break;

	case LCD_MODE_U2:
		ctrl = 0x101;
		break;

	case LCD_MODE_U3:
		ctrl = 0x185;
		break;

	case LCD_MODE_U3_PARTIAL:
		ctrl = 0x385;
		break;

	case LCD_MODE_U3_QUICKCOVER:
		ctrl = 0x585;
		break;

	case LCD_MODE_STOP:
		ctrl = 0x02;
		break;
	}

	/* swipe set */
	sw49408_swipe_mode(dev, mode);

	touch_msleep(20);
	TOUCH_I("sw49408_tc_driving = %d, %x\n", mode, ctrl);
	sw49408_reg_read(dev, spr_subdisp_st, (u8 *)&rdata, sizeof(u32));
	TOUCH_I("DDI Display Mode = %d\n", rdata);
	sw49408_reg_write(dev, d->reg_info.r_tc_cmd_spi_addr + tc_driving_ctl,
				&ctrl, sizeof(ctrl));
	sw49408_reg_read(dev, rst_cnt_addr, &rst_cnt, sizeof(u32));
	TOUCH_I("Reset Cnt : %d\n", rst_cnt);

	return 0;
}

static void sw49408_deep_sleep(struct device *dev)
{
	sw49408_tc_driving(dev, LCD_MODE_STOP);
	sw49408_clock(dev, 0);
}

static void sw49408_debug_tci(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u8 debug_reason_buf[TCI_MAX_NUM][TCI_DEBUG_MAX_NUM];
	u32 rdata[9] = {0, };
	u8 count[2] = {0, };
	u8 count_max = 0;
	u32 i, j = 0;
	u8 buf = 0;

	if (!d->tci_debug_type)
		return;

	sw49408_reg_read(dev, d->reg_info.r_abt_sts_spi_addr + TCI_DEBUG_R,
				&rdata, sizeof(rdata));

	count[TCI_1] = (rdata[0] & 0xFFFF);
	count[TCI_2] = ((rdata[0] >> 16) & 0xFFFF);
	count_max = (count[TCI_1] > count[TCI_2]) ? count[TCI_1] : count[TCI_2];

	if (count_max == 0)
		return;

	if (count_max > TCI_DEBUG_MAX_NUM) {
		count_max = TCI_DEBUG_MAX_NUM;
		if (count[TCI_1] > TCI_DEBUG_MAX_NUM)
			count[TCI_1] = TCI_DEBUG_MAX_NUM;
		if (count[TCI_2] > TCI_DEBUG_MAX_NUM)
			count[TCI_2] = TCI_DEBUG_MAX_NUM;
	}

	for (i = 0; i < ((count_max-1)/4)+1; i++) {
		memcpy(&debug_reason_buf[TCI_1][i*4], &rdata[i+1], sizeof(u32));
		memcpy(&debug_reason_buf[TCI_2][i*4], &rdata[i+5], sizeof(u32));
	}

	TOUCH_I("TCI count_max = %d\n", count_max);
	for (i = 0; i < TCI_MAX_NUM; i++) {
		TOUCH_I("TCI count[%d] = %d\n", i, count[i]);
		for (j = 0; j < count[i]; j++) {
			buf = debug_reason_buf[i][j];
			TOUCH_I("TCI_%d - DBG[%d/%d]: %s\n",
				i + 1, j + 1, count[i],
				(buf > 0 && buf < TCI_FAIL_NUM) ?
					tci_debug_str[buf] :
					tci_debug_str[0]);
		}
	}
}

static void sw49408_debug_swipe(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u8 debug_reason_buf[SWIPE_MAX_NUM][SWIPE_DEBUG_MAX_NUM];
	u32 rdata[9] = {0 , };
	u8 count[4] = {0, };
	u8 count_max_horizontal = 0;
	u8 count_max_vertical = 0;
	u8 count_max = 0;
	u32 i, j = 0;
	u8 buf = 0;

	if (!d->swipe_debug_type)
		return;

	sw49408_reg_read(dev, d->reg_info.r_abt_sts_spi_addr + SWIPE_DEBUG_R,
				&rdata, sizeof(rdata));

	count[SWIPE_R] = (rdata[0] & 0xF);
	count[SWIPE_D] = ((rdata[0] >> 8) & 0xF);
	count[SWIPE_L] = ((rdata[0] >> 16) & 0xF);
	count[SWIPE_U] = ((rdata[0] >> 24) & 0xF);
	count_max_horizontal = (count[SWIPE_R] > count[SWIPE_L]) ?
			count[SWIPE_R] : count[SWIPE_L];
	count_max_vertical = (count[SWIPE_U] > count[SWIPE_D]) ?
			count[SWIPE_U] : count[SWIPE_D];
	count_max = (count_max_horizontal > count_max_vertical) ?
			count_max_horizontal : count_max_vertical;

	if (count_max == 0)
		return;

	if (count_max > SWIPE_DEBUG_MAX_NUM) {
		count_max = SWIPE_DEBUG_MAX_NUM;
		if (count[SWIPE_R] > SWIPE_DEBUG_MAX_NUM)
			count[SWIPE_R] = SWIPE_DEBUG_MAX_NUM;
		if (count[SWIPE_D] > SWIPE_DEBUG_MAX_NUM)
			count[SWIPE_D] = SWIPE_DEBUG_MAX_NUM;
		if (count[SWIPE_L] > SWIPE_DEBUG_MAX_NUM)
			count[SWIPE_L] = SWIPE_DEBUG_MAX_NUM;
		if (count[SWIPE_U] > SWIPE_DEBUG_MAX_NUM)
			count[SWIPE_U] = SWIPE_DEBUG_MAX_NUM;
	}

	for (i = 0; i < ((count_max-1)/4)+1; i++) {
		memcpy(&debug_reason_buf[SWIPE_R][i*4], &rdata[i+1],
			sizeof(u32));
		memcpy(&debug_reason_buf[SWIPE_D][i*4], &rdata[i+5],
			sizeof(u32));
		memcpy(&debug_reason_buf[SWIPE_L][i*4], &rdata[i+3],
			sizeof(u32));
		memcpy(&debug_reason_buf[SWIPE_U][i*4], &rdata[i+7],
			sizeof(u32));
	}

	for (i = 0; i < SWIPE_MAX_NUM; i++) {
		for (j = 0; j < count[i]; j++) {
			buf = debug_reason_buf[i][j];
			TOUCH_I("SWIPE_%s - DBG[%d/%d]: %s\n",
				i == SWIPE_R ? "Right" :
				i == SWIPE_D ? "Down" :
				i == SWIPE_L ? "Left" : "Up",
				j + 1, count[i],
				(buf > 0 && buf < SWIPE_FAIL_NUM) ?
					swipe_debug_str[buf] :
					swipe_debug_str[0]);
		}
	}
}


static int sw49408_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			sw49408_lpwg_control(dev, LPWG_DOUBLE_TAP);
			sw49408_tc_driving(dev, d->lcd_mode);
			return 0;
		}
		if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg_mode\n");
			sw49408_debug_tci(dev);
			sw49408_debug_swipe(dev);
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("suspend ts->lpwg.sensor == PROX_NEAR\n");
			sw49408_deep_sleep(dev);
		} else if (ts->lpwg.qcover == HALL_NEAR) {
			/* knock on/code disable */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				sw49408_clock(dev, 1);

			sw49408_tc_driving(dev, LCD_MODE_STOP);
		} else {
			/* knock on disable ,knock on, knock code */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				sw49408_clock(dev, 1);

			sw49408_tci_area_set(dev, QUICKCOVER_OPEN);
			sw49408_lpwg_control(dev, ts->lpwg.mode);
			if (ts->lpwg.mode == LPWG_NONE && !d->swipe.mode) {
				/* knock on code disable, swipe disable */
				sw49408_deep_sleep(dev);
			} else {
				sw49408_tc_driving(dev, d->lcd_mode);
			}
		}
		return 0;
	}

	/* resume */
	touch_report_all_event(ts);
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("resume ts->lpwg.screen on\n");
		sw49408_lpwg_control(dev, LPWG_NONE);
		if (ts->lpwg.qcover == HALL_NEAR)
			sw49408_tc_driving(dev, LCD_MODE_U3_QUICKCOVER);
		else
			sw49408_tc_driving(dev, d->lcd_mode);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		TOUCH_I("resume ts->lpwg.sensor == PROX_NEAR\n");
		sw49408_deep_sleep(dev);
	} else {
		/* partial */
		TOUCH_I("resume Partial\n");
		if (ts->lpwg.qcover == HALL_NEAR)
			sw49408_tci_area_set(dev, QUICKCOVER_CLOSE);
		else
			sw49408_tci_area_set(dev, QUICKCOVER_OPEN);
		sw49408_lpwg_control(dev, ts->lpwg.mode);
		sw49408_tc_driving(dev, LCD_MODE_U3_PARTIAL);
	}

	return 0;
}

static int sw49408_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		break;

	case LPWG_UPDATE_ALL:
		if ( (ts->lpwg.screen == 1 && value[1] == 0 &&
			ts->lpwg.sensor == PROX_FAR) ||
			(ts->lpwg.qcover == 1 && value[3] == 0) )
			sw49408_clear_q_sensitivity(dev);

		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];

		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg.mode,
			ts->lpwg.screen ? "ON" : "OFF",
			ts->lpwg.sensor ? "FAR" : "NEAR",
			ts->lpwg.qcover ? "CLOSE" : "OPEN");

		sw49408_lpwg_mode(dev);

		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

static void sw49408_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
	int wireless_state = atomic_read(&ts->state.wireless);

	TOUCH_TRACE();

	d->charger = 0;
	/* wire */
	if (charger_state == CONNECT_INVALID)
		d->charger = CONNECT_NONE;
	else if ((charger_state == CONNECT_DCP)
			|| (charger_state == CONNECT_PROPRIETARY))
		d->charger = CONNECT_TA;
	else if (charger_state == CONNECT_HUB)
		d->charger = CONNECT_OTG;
	else
		d->charger = CONNECT_USB;

	/* code for TA simulator */
	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_4) {
		TOUCH_I("TA Simulator mode, Set CONNECT_TA\n");
		d->charger = CONNECT_TA;
	}

	/* wireless */
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try SPI\n");
		return;
	}

	sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr + SPR_CHARGER_STS,
				&d->charger, sizeof(u32));
}

static void sw49408_lcd_mode(struct device *dev, u32 mode)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	d->prev_lcd_mode = d->lcd_mode;
	d->lcd_mode = mode;
	TOUCH_I("lcd_mode: %d (prev: %d)\n", d->lcd_mode, d->prev_lcd_mode);
}

static int sw49408_check_mode(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	if (d->lcd_mode != LCD_MODE_U3) {
		if (d->lcd_mode == LCD_MODE_U2) {
			if (d->prev_lcd_mode == LCD_MODE_U2_UNBLANK) {
				TOUCH_I("U2 UNBLANK -> U2\n");
				ret = 1;
			} else {
				TOUCH_I("U2 mode change\n");
			}
		} else if (d->lcd_mode == LCD_MODE_U2_UNBLANK) {
			switch (d->prev_lcd_mode) {
			case LCD_MODE_U2:
				TOUCH_I("U2 -> U2 UNBLANK\n");
				ret = 1;
				break;
			case LCD_MODE_U0:
				TOUCH_I("U0 -> U2 UNBLANK mode change\n");
				break;
			default:
				TOUCH_I("%s - Not Defined Mode\n", __func__);
				break;
			}
		} else if (d->lcd_mode == LCD_MODE_U0) {
			TOUCH_I("U0 mode change\n");
		} else {
			TOUCH_I("%s - Not defined mode\n", __func__);
		}
	}

	return ret;
}

static void sw49408_lcd_event_read_reg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u32 rdata[5] = {0};

	sw49408_xfer_msg_ready(dev, 5);

	ts->xfer->data[0].rx.addr = tc_ic_status;
	ts->xfer->data[0].rx.buf = (u8 *)&rdata[0];
	ts->xfer->data[0].rx.size = sizeof(rdata[0]);

	ts->xfer->data[1].rx.addr = tc_status;
	ts->xfer->data[1].rx.buf = (u8 *)&rdata[1];
	ts->xfer->data[1].rx.size = sizeof(rdata[1]);

	ts->xfer->data[2].rx.addr = spr_subdisp_st;
	ts->xfer->data[2].rx.buf = (u8 *)&rdata[2];
	ts->xfer->data[2].rx.size = sizeof(rdata[2]);

	ts->xfer->data[3].rx.addr = tc_version;
	ts->xfer->data[3].rx.buf = (u8 *)&rdata[3];
	ts->xfer->data[3].rx.size = sizeof(rdata[3]);

	ts->xfer->data[4].rx.addr = 0x0;
	ts->xfer->data[4].rx.buf = (u8 *)&rdata[4];
	ts->xfer->data[4].rx.size = sizeof(rdata[4]);

	sw49408_xfer_msg(dev, ts->xfer);

	TOUCH_I(
		"reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x reg[%x] = 0x%x\n",
		tc_ic_status, rdata[0], tc_status, rdata[1],
		spr_subdisp_st, rdata[2], tc_version, rdata[3],
		0x0, rdata[4]);
	TOUCH_I("v%d.%02d\n", (rdata[3] >> 8) & 0xF, rdata[3] & 0xFF);
}

static int sw49408_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	sw49408_connect(dev);
	return 0;
}

static int sw49408_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	sw49408_connect(dev);
	return 0;
}

static int sw49408_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}

static int sw49408_debug_tool(struct device *dev, u32 value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (value == DEBUG_TOOL_ENABLE) {
		ts->driver->irq_handler = sw49408_sic_abt_irq_handler;
	} else {
		ts->driver->irq_handler = sw49408_irq_handler;
	}

	return 0;
}
static int sw49408_debug_option(struct device *dev, u32 *data)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 chg_mask = data[0];
	u32 enable = data[1];

	switch (chg_mask) {
	case DEBUG_OPTION_0:
		TOUCH_I("Debug Option 0 %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_1:
		break;
	case DEBUG_OPTION_2:
		TOUCH_I("Runtime Debug %s\n", enable ? "Enable" : "Disable");
		sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr + RUNTIME_DEBUG,
									&enable, sizeof(u32));
		break;
	case DEBUG_OPTION_3:
		break;
	case DEBUG_OPTION_4:
		TOUCH_I("TA Simulator mode %s\n",
			enable ? "Enable" : "Disable");
		sw49408_connect(dev);
		break;
	default:
		TOUCH_E("Not supported debug option\n");
		break;
	}

	return 0;
}

static void sw49408_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct sw49408_data *d =
			container_of(to_delayed_work(fb_notify_work),
				struct sw49408_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U3)
		ret = FB_RESUME;
	else
		ret = FB_SUSPEND;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

static void sw49408_te_test_work_func(struct work_struct *te_test_work)
{
	struct sw49408_data *d =
			container_of(to_delayed_work(te_test_work),
				struct sw49408_data, te_test_work);
	struct touch_core_data *ts = to_touch_core(d->dev);
	u32 count = 0;
	u32 ms = 0;
	u32 hz = 0;
	u32 hz_min = 0xFFFFFFFF;
	int ret = 0;
	int i = 0;

	memset(d->te_test_log, 0x0, sizeof(d->te_test_log));
	d->te_ret = 0;
	d->te_write_log = DO_WRITE_LOG;
	TOUCH_I("DDIC Test Start\n");

	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(d->te_test_log + ret, 63, "not support on u%d\n",
			d->lcd_mode);
		d->te_ret = 1;
		return ;
	}

	mutex_lock(&ts->lock);
	for (i = 0; i < 100; i++) {
		sw49408_reg_read(d->dev, d->reg_info.r_tc_sts_spi_addr +
			tc_rtc_te_interval_cnt, (u8 *)&count, sizeof(u32));
		if (count == 0) {
			ret = snprintf(d->te_test_log + ret, 63,
				"[%d] : 0, 0 ms, 0 hz\n", i + 1);
			d->te_ret = 1;
			hz_min = 0;
			TOUCH_I("%s\n", d->te_test_log);
			break;
		}

		ms = (count * 100 * 1000) / 32764;
		hz = (32764 * 100) / count;

		if (hz < hz_min)
			hz_min = hz;

		if ((hz / 100 < 57) || (hz / 100 > 63)) {
			ret = snprintf(d->te_test_log + ret, 63,
				"[%d] : %d, %d.%02d ms, %d.%02d hz\n", i + 1, count,
				ms / 100, ms % 100, hz / 100, hz % 100);
			d->te_ret = 1;
			TOUCH_I("[%d] %s\n", i + 1, d->te_test_log);
			break;
		}
		touch_msleep(15);
	}
	mutex_unlock(&ts->lock);

	TOUCH_I("DDIC Test END : [%s] %d.%02d hz\n", d->te_ret ? "Fail" : "Pass",
		hz_min / 100, hz_min % 100);
}
static int sw49408_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
	case NOTIFY_TOUCH_RESET:
		TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
		atomic_set(&d->init, IC_INIT_NEED);
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		sw49408_lcd_mode(dev, *(u32 *)data);
		ret = sw49408_check_mode(dev);
		if (ret == 0)
			queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		else
			ret = 0;
		break;
	case LCD_EVENT_READ_REG:
		TOUCH_I("LCD_EVENT_READ_REG\n");
		sw49408_lcd_event_read_reg(dev);
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = sw49408_usb_status(dev, *(u32 *)data);
		break;
	case NOTIFY_WIRELEES:
		TOUCH_I("NOTIFY_WIRELEES!\n");
		ret = sw49408_wireless_status(dev, *(u32 *)data);
		break;
	case NOTIFY_EARJACK:
		TOUCH_I("NOTIFY_EARJACK!\n");
		ret = sw49408_earjack_status(dev, *(u32 *)data);
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			REG_IME_STATE, (u32*)data, sizeof(u32));
		break;
	case NOTIFY_DEBUG_TOOL:
		ret = sw49408_debug_tool(dev, *(u32 *)data);
		TOUCH_I("NOTIFY_DEBUG_TOOL!\n");
		break;
	case NOTIFY_CALL_STATE:
		TOUCH_I("NOTIFY_CALL_STATE!\n");
		ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			REG_CALL_STATE, (u32 *)data, sizeof(u32));
		break;
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		ret = sw49408_debug_option(dev, (u32 *)data);
		break;
	case NOTIFY_ONHAND_STATE:
		TOUCH_I("NOTIFY_ONHAND_STATE!\n");
		break;
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static void sw49408_init_works(struct sw49408_data *d)
{
	d->wq_log = create_singlethread_workqueue("touch_wq_log");

	if (!d->wq_log)
		TOUCH_E("failed to create workqueue log\n");

	INIT_DELAYED_WORK(&d->te_test_work, sw49408_te_test_work_func);
	INIT_DELAYED_WORK(&d->fb_notify_work, sw49408_fb_notify_work_func);
}

static void sw49408_init_locks(struct sw49408_data *d)
{
	mutex_init(&d->spi_lock);
}

static int sw49408_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate synaptics data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_gpio_init(ts->maker_id_pin, "touch_make_id");
	touch_gpio_direction_input(ts->maker_id_pin);

	touch_power_init(dev);
	touch_bus_init(dev, MAX_BUF_SIZE);

	sw49408_init_works(d);
	sw49408_init_locks(d);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		touch_gpio_init(ts->reset_pin, "touch_reset");
		touch_gpio_direction_output(ts->reset_pin, 1);
		/* Deep Sleep */
		sw49408_deep_sleep(dev);
		return 0;
	}

	sw49408_get_tci_info(dev);
	sw49408_get_swipe_info(dev);
	pm_qos_add_request(&d->pm_qos_req, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	d->lcd_mode = LCD_MODE_U3;
	d->tci_debug_type = 1;
	d->swipe_debug_type = 0;
	d->tc_status_rst_cnt = 0;
	d->tc_status_fwup_cnt = 0;
	sw49408_sic_abt_probe();

	return 0;
}

static int sw49408_remove(struct device *dev)
{
	struct sw49408_data* d = to_sw49408_data(dev);

	TOUCH_TRACE();
	pm_qos_remove_request(&d->pm_qos_req);
	sw49408_sic_abt_remove();
	return 0;
}

static int sw49408_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);
	struct sw49408_version *device = &d->ic_info.version;
	struct sw49408_version bin = {0};
	struct sw49408_version *binary = &bin;
	char pid[12] = {0};
	int update = 0;

	if ((bin_ver_offset > FLASH_FW_SIZE) ||
			(bin_pid_offset > FLASH_FW_SIZE)) {
		TOUCH_I("%s : invalid offset\n", __func__);
		return -1;
	}

	bin.build = (fw->data[bin_ver_offset] >> 4 ) & 0xF;
	bin.major = fw->data[bin_ver_offset] & 0xF;
	bin.minor = fw->data[bin_ver_offset + 1];

	memcpy(pid, &fw->data[bin_pid_offset], 8);

	if (ts->force_fwup) {
		update = 1;
	} else if (binary->major != device->major) {
		update = 1;
	} else {
		if (binary->minor != device->minor)
			update = 1;
		else if (binary->build > device->build)
			update = 1;
	}

	TOUCH_I("%s : binary[%d.%02d.%d] device[%d.%02d.%d]" \
		" -> update: %d, force: %d\n", __func__,
		binary->major, binary->minor, binary->build,
		device->major, device->minor, device->build,
		update, ts->force_fwup);

	return update;
}

static int sw49408_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;

	do {
		touch_msleep(delay);
		sw49408_read_value(dev, addr, &data);

		if ((data & mask) == expect) {
			if (value)
				*value = data;
			TOUCH_I(
				"%d, addr[%04x] data[%08x], mask[%08x], expect[%08x]\n",
				retry, addr, data, mask, expect);
			return 0;
		}
	} while (--retry);

	if (value)
		*value = data;

	TOUCH_I("%s addr[%04x], expect[%x], mask[%x], data[%x]\n",
		__func__, addr, expect, mask, data);

	return -EPERM;
}

int common_header_verify(t_cfg_info_def *header)
{
	t_cfg_info_def *head = (t_cfg_info_def *)header;
	t_cfg_c_header_def *common_head =
		(t_cfg_c_header_def *)(header + sizeof(t_cfg_info_def));

	if (head->cfg_magic_code != CFG_MAGIC_CODE) {
		TOUCH_I("Invalid CFG_MAGIC_CODE. %8.8X\n",
			head->cfg_magic_code);
		return -1;
	}

	if (head->cfg_chip_id != CFG_CHIP_ID) {
		TOUCH_I("Invalid Chip ID. (49408 != %d)\n",
			head->cfg_chip_id);
		return -2;
	}

	if (head->cfg_struct_version <= 0) {
		TOUCH_I("Invalid cfg_struct_version. %8.8X\n",
			head->cfg_struct_version);
		return -3;
	}

	if (head->cfg_specific_cnt <= 0) {
		TOUCH_I("No Specific Data. %8.8X\n",
			head->cfg_specific_cnt);
		return -4;
	}

	if (head->cfg_size.b.common_cfg_size > CFG_C_MAX_SIZE) {
		TOUCH_I("Over CFG COMMON MAX Size (%d). %8.8X\n",
			CFG_C_MAX_SIZE, head->cfg_size.b.common_cfg_size);
		return -5;
	}

	if (head->cfg_size.b.specific_cfg_size > CFG_S_MAX_SIZE) {
		TOUCH_I("Over CFG SPECIFIC MAX Size (%d). %8.8X\n",
			CFG_S_MAX_SIZE, head->cfg_size.b.specific_cfg_size);
		return -6;
	}

	TOUCH_I("==================== COMMON ====================\n");
	TOUCH_I("magic code         : 0x%8.8X\n", head->cfg_magic_code);
	TOUCH_I("chip id            : %d\n", head->cfg_chip_id);
	TOUCH_I("struct_ver         : %d\n", head->cfg_struct_version);
	TOUCH_I("specific_cnt       : %d\n", head->cfg_specific_cnt);
	TOUCH_I("cfg_c size         : %d\n", head->cfg_size.b.common_cfg_size);
	TOUCH_I("cfg_s size         : %d\n",
					head->cfg_size.b.specific_cfg_size);
	TOUCH_I("date               : 0x%8.8X\n", head->cfg_global_date);
	TOUCH_I("time               : 0x%8.8X\n", head->cfg_global_time);
	TOUCH_I("common_ver         : %d\n", common_head->cfg_common_ver);

	return 1;
}

int specific_header_verify(unsigned char *header, int i)
{
	t_cfg_s_header_def *head = (t_cfg_s_header_def *)header;
	char tmp[8] = {0, };

	if (head->cfg_specific_info1.b.chip_rev <= 0
		&& head->cfg_specific_info1.b.chip_rev > 10) {
		TOUCH_I("Invalid Chip revision id %8.8X\n",
			head->cfg_specific_info1.b.chip_rev);
		return -2;
	}

	memset(tmp, 0, 8);
	memcpy((void*)tmp, (void *)&head->cfg_model_name, 4);

	TOUCH_I("==================== SPECIFIC #%d =====================\n",
						i +1);
	TOUCH_I("chip_rev           : %d\n",
					head->cfg_specific_info1.b.chip_rev);
	TOUCH_I("fpcb_id            : %d\n",
					head->cfg_specific_info1.b.fpcb_id);
	TOUCH_I("lcm_id             : %d\n",
					head->cfg_specific_info1.b.lcm_id);
	TOUCH_I("model_id           : %d\n",
					head->cfg_specific_info1.b.model_id);
	TOUCH_I("model_name         : %s\n", tmp);
	TOUCH_I("lot_id             : %d\n",
					head->cfg_specific_info2.b.lot_id);
	TOUCH_I("ver                : %d\n",
					head->cfg_specific_version);

	return 1;
}

static int sw49408_img_binary_verify(unsigned char *imgBuf)
{
	unsigned char *specific_ptr;
	unsigned char *cfg_buf_base = &imgBuf[FLASH_FW_SIZE];
	int i;
	t_cfg_info_def *head = (t_cfg_info_def *)cfg_buf_base;

	u32 *fw_crc = (u32 *)&imgBuf[FLASH_FW_SIZE -4];
	u32 *fw_size = (u32 *)&imgBuf[FLASH_FW_SIZE -8];

	if (*fw_crc == 0x0
		|| *fw_crc == 0xFFFFFFFF
		|| *fw_size > FLASH_FW_SIZE) {
		TOUCH_I("Firmware Size Invalid READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC Invalid READ : 0x%X\n", *fw_crc);
		return E_FW_CODE_SIZE_ERR;
	} else {
		TOUCH_I("Firmware Size READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC READ : 0x%X\n", *fw_crc);
	}

	if (common_header_verify(head) < 0) {
		TOUCH_I("No Common CFG! Firmware Code Only\n");
		return E_FW_CODE_ONLY_VALID;
	}

	specific_ptr = cfg_buf_base + head->cfg_size.b.common_cfg_size;
	for (i = 0; i < head->cfg_specific_cnt; i++) {
		if (specific_header_verify(specific_ptr, i) < 0) {
			TOUCH_I("specific CFG invalid!\n");
			return -2;
		}
		specific_ptr += head->cfg_size.b.specific_cfg_size;
	}

	return E_FW_CODE_AND_CFG_VALID;
}

void sw49408_default_reg_map(struct device* dev){
	struct sw49408_data *d = to_sw49408_data(dev);
	d->reg_info.r_info_ptr_spi_addr     = 0x043;
	d->reg_info.r_tc_cmd_spi_addr       = 0xC00;
	d->reg_info.r_watch_cmd_spi_addr    = 0xC10;
	d->reg_info.r_test_cmd_spi_addr     = 0xC20;
	d->reg_info.r_abt_cmd_spi_addr      = 0xC30;
	d->reg_info.r_cfg_c_sram_oft        = 0x960;
	d->reg_info.r_cfg_s_sram_oft        = 0x9E0;
	d->reg_info.r_sys_buf_sram_oft      = 0xAE0;
	d->reg_info.r_abt_buf_sram_oft      = 0xD67;
	d->reg_info.r_dbg_buf_sram_oft      = 0x1187;
	d->reg_info.r_ic_status_spi_addr    = 0x200;
	d->reg_info.r_tc_status_spi_addr    = 0x201;
	d->reg_info.r_abt_report_spi_addr   = 0x202;
	d->reg_info.r_reserv_spi_addr       = 0x242;
	d->reg_info.r_chip_info_spi_addr    = 0x25c;
	d->reg_info.r_reg_info_spi_addr     = 0x264;
	d->reg_info.r_pt_info_spi_addr      = 0x270;
	d->reg_info.r_tc_sts_spi_addr       = 0x27F;
	d->reg_info.r_abt_sts_spi_addr      = 0x2BF;
	d->reg_info.r_tune_code_spi_addr    = 0x30F;
	d->reg_info.r_aod_spi_addr          = 0x3E3;
	TOUCH_I("================================================\n");
	TOUCH_I("sw49408 default register map loaded!!\n");
	TOUCH_I("================================================\n");
}

int sw49408_chip_info_load(struct device* dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 info_ptr_val = 0;
	u32 status_val[2] = {0,};
	u16 chip_info_addr = 0;
	u16 reg_info_addr = 0;

	if (sw49408_reg_read(dev, tc_ic_status,
				(u32 *)&status_val[0], sizeof(status_val)) < 0) {
		TOUCH_E("status addr read error\n");
		goto error;
	}
	TOUCH_I("ic status read : %8.8X, tc status read : %8.8X\n",
						status_val[0], status_val[1]);

	if (sw49408_reg_read(dev, info_ptr_addr,
			(u8 *)&info_ptr_val, sizeof(u32)) < 0) {
		TOUCH_E("Info ptr addr read error\n");
		goto error;
	}
	TOUCH_I("info ptr addr read : %8.8X\n", info_ptr_val);

	if (info_ptr_val == 0 || ((info_ptr_val >> 24) != 0)) {

		TOUCH_E("info_ptr_addr invalid!\n");
		goto error;
	}

	chip_info_addr = (info_ptr_val & 0xFFF);
	reg_info_addr  = ((info_ptr_val >> 12) & 0xFFF);

	TOUCH_I("========== Info ADDR ==========\n");
	TOUCH_I("chip_info_addr        : %4.4X\n", chip_info_addr);
	TOUCH_I("reg_info_addr         : %4.4X\n", reg_info_addr);
	TOUCH_I("========== Info ADDR ==========\n");

	if (!((status_val[1] >> 27) & 0x1))
		TOUCH_E("Model ID is not loaded : %x\n", status_val[1]);
	else
		TOUCH_I("Model ID is loaded\n");

	if ((status_val[1] >> 28) & 0x1)
		TOUCH_E("Production Test info checksum error : %x\n",
								 status_val[1]);
	else
		TOUCH_I("Production Test info checksum is ok\n");

	//chip info read
	if (sw49408_reg_read(dev, chip_info_addr, (u8 *)&d->chip_info,
				sizeof(struct sw49408_chip_info)) < 0) {
		TOUCH_E("Chip info Read Error\n");
		goto error;
	}

	if (sw49408_reg_read(dev, reg_info_addr, (u8 *)&d->reg_info,
				sizeof(struct sw49408_reg_info)) < 0) {
		TOUCH_E("Reg info Read Error\n");
		goto error;
	}

	return 0;

error:
	// ic default register map loading
	sw49408_default_reg_map(dev);
	return -1;
}

static int sw49408_fw_upgrade(struct device *dev,
			     const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	u8 *fwdata = (u8 *) fw->data;
	u32 data;
	u32 cfg_c_size;
	u32 cfg_s_size;
	t_cfg_info_def *head;
	int ret;
	int img_check_result;

	TOUCH_I("%s - START\n", __func__);

	if (fw->size > FLASH_SIZE) {
		TOUCH_I("Image Size invalid : %d. The Size must be less then 128KB. The Process of Firmware Download could not process!\n",
			(unsigned int)fw->size);
		return -EPERM;
	} else {
		TOUCH_I("Image Size : 0x%8.8X(%d)\n",
			(unsigned int)fw->size, (unsigned int)fw->size);
	}

	// Binary Check
	img_check_result = sw49408_img_binary_verify((unsigned char*)fwdata);

	switch (img_check_result) {
		case E_FW_CODE_AND_CFG_VALID:
			break;
		case E_FW_CODE_CFG_ERR:
		case E_FW_CODE_SIZE_ERR:
		case E_FW_CODE_ONLY_VALID:
		default:
			return -EPERM;
	}

	/* CM3 hold */
	sw49408_write_value(dev, spr_rst_ctl, 2);

	/* sram write enable */
	sw49408_write_value(dev, spr_sram_ctl, 3);

	/* code sram base address write */
	sw49408_write_value(dev, spr_code_offset, 0);

	/* first 60KB firmware image download to code sram */
	sw49408_reg_write(dev, code_access_addr, &fwdata[0], MAX_RW_SIZE);

	/* code sram base address write */
	sw49408_write_value(dev, spr_code_offset, MAX_RW_SIZE / 4);

	/* last 12KB firmware image download to code sram */
	sw49408_reg_write(dev, code_access_addr, &fwdata[MAX_RW_SIZE],
		FLASH_FW_SIZE - MAX_RW_SIZE);

	/* Init Boot Code */
	sw49408_write_value(dev, fw_boot_code_addr, FW_BOOT_LOADER_INIT);
	/* Boot Start */
	sw49408_write_value(dev, spr_boot_ctl, 1);
	/* CM3 Release*/
	sw49408_write_value(dev, spr_rst_ctl, 0);

	/* firmware boot done check */
	ret = sw49408_condition_wait(dev, fw_boot_code_addr, NULL,
				    FW_BOOT_LOADER_CODE, 0xFFFFFFFF, 10, 10);

	if (ret < 0) {
		TOUCH_E("failed : \'boot check\'\n");
		return -EPERM;
	} else {
		TOUCH_I("success : boot check\n");
	}


	if (sw49408_chip_info_load(dev) < 0) {
		TOUCH_E("IC_Register map load fail!\n");
		return -EPERM;
	}

	if (d->reg_info.r_cfg_c_sram_oft == 0 ||
			d->chip_info.r_conf_dn_index == 0 ||
			d->chip_info.r_conf_dn_index > 20) {
		TOUCH_E("CFG_S_INDEX(%d) or CFG_DN_OFFSET(%x) invalid\n",
			d->chip_info.r_conf_dn_index,
			d->reg_info.r_cfg_c_sram_oft);
		return -EPERM;
	}

	/* Firmware Download Start */
	sw49408_write_value(dev, d->reg_info.r_tc_cmd_spi_addr +
				tc_flash_dn_ctl,
				(FLASH_KEY_CODE_CMD << 16) | 1);
	touch_msleep(ts->caps.hw_reset_delay);

	/* download check */
	ret = sw49408_condition_wait(dev, d->reg_info.r_tc_sts_spi_addr, &data,
				FLASH_CODE_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);

	if (ret < 0) {
		TOUCH_E("failed : \'code check\'\n");
		return -EPERM;
	}

	if (img_check_result == E_FW_CODE_AND_CFG_VALID) {
		head = (t_cfg_info_def *)&fwdata[FLASH_FW_SIZE];

		cfg_c_size = head->cfg_size.b.common_cfg_size;
		cfg_s_size = head->cfg_size.b.specific_cfg_size;

		if (d->chip_info.r_conf_dn_index == 0 ||
				((d->chip_info.r_conf_dn_index * cfg_s_size) >
				 (fw->size - FLASH_FW_SIZE - cfg_c_size))) {
			TOUCH_I("Invalid Specific CFG Index => 0x%8.8X\n",
				d->chip_info.r_conf_dn_index);
			return -EPERM;
		}
		/* cfg_c sram base address write */
		sw49408_write_value(dev, spr_data_offset,
					d->reg_info.r_cfg_c_sram_oft);

		/* Conf data download to conf sram */
		// CFG Common Download to CFG Download buffer (SRAM)
		sw49408_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE],
					cfg_c_size);

		/* cfg_s sram base address write */
		sw49408_write_value(dev, spr_data_offset,
					d->reg_info.r_cfg_s_sram_oft);
		// CFG Specific Download to CFG Download buffer (SRAM)
		sw49408_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE +
					cfg_c_size +
					(d->chip_info.r_conf_dn_index - 1)*
					cfg_s_size], cfg_s_size);
		//sw49408_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE], FLASH_CONF_SIZE);

		/* Conf Download Start */
		sw49408_write_value(dev, d->reg_info.r_tc_cmd_spi_addr +
					 tc_flash_dn_ctl,
					(FLASH_KEY_CONF_CMD << 16) | 2);

		/* Conf check */
		ret = sw49408_condition_wait(dev, d->reg_info.r_tc_sts_spi_addr,
						&data, FLASH_CONF_DNCHK_VALUE,
						0xFFFFFFFF, 10, 200);
		if (ret < 0) {
			TOUCH_E("failed : \'cfg check\'\n");
			return -EPERM;
		} else {
			TOUCH_I("success : cfg_check\n");
		}
	}
	TOUCH_I("===== Firmware download Okay =====\n");

	return 0;
}

static int sw49408_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;
	int i = 0;

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n",
			&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		if (!strcmp(d->ic_info.product_id, "L1L57P2")) {
			memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
			TOUCH_I("get fwpath from def_fwpath : rev:%d\n",
			d->ic_info.revision);
		} else {
			memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
			TOUCH_I("wrong product id[%s] : fw_path set for default\n",
							d->ic_info.product_id);
	        }
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);

		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (sw49408_fw_compare(dev, fw)) {
		ret = -EINVAL;
		touch_msleep(200);
		for (i = 0; i < 2 && ret; i++)
			ret = sw49408_fw_upgrade(dev, fw);
	} else {
		release_firmware(fw);
		return -EPERM;
	}

	release_firmware(fw);
	return 0;
}

static int sw49408_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();
	// temp LCD mode set
	d->lcd_mode = LCD_MODE_U0;

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		sw49408_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
	}

	if (atomic_read(&d->init) == IC_INIT_DONE)
		sw49408_lpwg_mode(dev);
	else /* need init */
		ret = 1;

	return ret;
}

static int sw49408_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int mfts_mode = 0;

	TOUCH_TRACE();
	// temp LCD mode set
	d->lcd_mode = LCD_MODE_U3;

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		sw49408_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		sw49408_ic_info(dev);
		if (sw49408_upgrade(dev) == 0) {
			sw49408_power(dev, POWER_OFF);
			sw49408_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
	}
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		sw49408_deep_sleep(dev);
		return -EPERM;
	}

	return 0;
}

static int sw49408_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	u32 data = 1;
	int ret = 0;

	TOUCH_TRACE();
/*	TOUCH_I("temp %d msleep for Video mode display transition\n", ts->caps.hw_reset_delay);
	touch_msleep(ts->caps.hw_reset_delay);

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("fb_notif change\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = sw49408_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}
*/
	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);
	TOUCH_I("%s: runtime debug : %s\n", __func__,
			atomic_read(&ts->state.debug_option_mask) &
			DEBUG_OPTION_2 ? "Enable" : "Disable");

	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE)
		sw49408_sic_abt_init(dev);

	if (sw49408_chip_info_load(dev) < 0) {
		TOUCH_E("IC_Register map load fail!\n");
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		sw49408_power(dev, POWER_OFF);
		sw49408_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
	}

	ret = sw49408_ic_info(dev);
	if (ret < 0) {
		touch_interrupt_control(dev, INTERRUPT_DISABLE);
		sw49408_power(dev, POWER_OFF);
		sw49408_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
	}

	ret = sw49408_reg_write(dev, d->reg_info.r_tc_cmd_spi_addr +
				tc_device_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_device_ctrl\', ret:%d\n", ret);

	ret = sw49408_reg_write(dev, d->reg_info.r_tc_cmd_spi_addr +
				tc_interrupt_ctl, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'tc_interrupt_ctrl\', ret:%d\n", ret);

	ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				SPR_CHARGER_STS, &d->charger, sizeof(u32));
	if (ret)
		TOUCH_E("failed to write \'spr_charger_sts\', ret:%d\n", ret);

	data = atomic_read(&ts->state.ime);
	ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				REG_IME_STATE, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'reg_ime_state\', ret:%d\n", ret);

	data = atomic_read(&ts->state.incoming_call);
	ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				REG_CALL_STATE, &data, sizeof(u32));
	if (ret)
		TOUCH_E("failed to write \'incoming_call\', ret:%d\n", ret);

	ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
			QCOVER_SENSITIVITY, &d->q_sensitivity, sizeof(u32));
	if (ret)
		TOUCH_E("failed to write \'QCOVER_SENSITIVITY\', ret:%d\n",
			ret);

	data = PROTOCOL_DISABLE;
	ret = sw49408_reg_write(dev, SERIAL_I2C_EN, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'serial_i2c_enable\', ret:%d\n", ret);

	data = ABNORMAL_IC_DETECTION;
	ret = sw49408_reg_write(dev, SPI_TATTN_OPT, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'spi_attn_report\', ret:%d\n", ret);

	data = (atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_2) ?
		RUNTIME_DEBUG_ENABLE : RUNTIME_DEBUG_DISABLE;
	ret = sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
						RUNTIME_DEBUG, &data, sizeof(u32));
	if (ret)
		TOUCH_E("failed to write \'runtime_debug\', ret:%d\n", ret);

	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);

	sw49408_lpwg_mode(dev);
	if (ret)
		TOUCH_E("failed to lpwg_control, ret:%d\n", ret);

	sw49408_te_info(dev, NULL);

	return 0;
}

/* (1 << 5)|(1 << 6)|(1 << 7)|(1 << 9)|(1 << 10)|(1 << 31) */
#define INT_RESET_CLR_BIT	0x800006E0
/* (1 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_LOGGING_CLR_BIT	0x50A000
/* (1 << 5) |(1 << 6) |(1 << 7)|(0 << 9)|(0 << 10)|(0 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_NORMAL_MASK		0x5080E0
#define IC_DEBUG_SIZE		16	/* byte */

int sw49408_check_status(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	u32 status = d->info.device_status;
	u32 ic_status = d->info.ic_status;
	u32 debugging_mask = 0x0;
	u8 debugging_length = 0x0;
	u32 debugging_type = 0x0;
	u32 status_mask = 0x0;
	u32 spi_err = 0;
	u32 spi_fault = 0;
	int checking_log_flag = 0;
	const int checking_log_size = 1024;
	char checking_log[1024] = {0};
	int length = 0;

	status_mask = status ^ INT_NORMAL_MASK;
	if (status_mask & INT_RESET_CLR_BIT) {
		TOUCH_I("%s : Need Reset, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ERESTART;
	} else if (status_mask & INT_LOGGING_CLR_BIT) {
		TOUCH_I("%s : Need Logging, status = %x, ic_status = %x\n",
			__func__, status, ic_status);
		ret = -ERANGE;
	}

	if (ret != 0) {
		if (!(status & (1 << 5))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[5]Device_ctl not Set");
		}
		if (!(status & (1 << 6))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[6]Code CRC Invalid err : %d %s",
                                d->tc_status_rst_cnt,
				d->tc_status_rst_cnt >= 3 ? "Skip Reset" : "Reset");
			if (d->tc_status_rst_cnt >= 3)
				ret = -ERANGE;
			else {
				d->tc_status_rst_cnt++;
				ret = -ERESTART;
			}
		}
		if (!(status & (1 << 7))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[7]CFG CRC Invalid err : %d %s",
                                d->tc_status_fwup_cnt,
				d->tc_status_fwup_cnt >= 3 ? "Skip Upgrade" : "Upgrade");
			if (d->tc_status_fwup_cnt >= 3)
				ret = -ERANGE;
			else {
				d->tc_status_fwup_cnt++;
				ret = -EUPGRADE;
			}
		}
		if (status & (1 << 9)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[9]Abnormal status Detected : %d %s",
				d->tc_status_rst_cnt,
				d->tc_status_rst_cnt >= 3 ? "Skip Reset" : "Reset");
			if (d->tc_status_rst_cnt >= 3)
				ret = -ERANGE;
			else {
				d->tc_status_rst_cnt++;
				ret = -ERESTART;
			}
		}
		if (status & (1 << 10)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[10]System Error Detected : %d %s",
				d->tc_status_rst_cnt,
				d->tc_status_rst_cnt >= 3 ? "Skip Reset" : "Reset");
			if (d->tc_status_rst_cnt >= 3)
				ret = -ERANGE;
			else {
				d->tc_status_rst_cnt++;
				ret = -ERESTART;
			}
		}
		if (status & (1 << 13)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[13]Display mode Mismatch");
		}
		if (!(status & (1 << 15))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[15]Interrupt_Pin Invalid");
		}
		if (!(status & (1 << 20))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[20]Touch interrupt status Invalid");
		}
		if (!(status & (1 << 22))) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[22]TC driving Invalid");
		}
		if (status & (1 << 31)) {
			checking_log_flag = 1;
			length += snprintf(checking_log + length,
				checking_log_size - length,
				"[31]ESD(Stripe) error detected");
//			lge_panel_recovery_mode();
			ret = -ERANGE;
		}

		if (checking_log_flag) {
			TOUCH_E("%s, status = %x, ic_status = %x\n",
					checking_log, status, ic_status);
		}

		sw49408_te_info(dev, NULL);
	}

	if ((ic_status & 1) || (ic_status & (1 << 3))) {
		sw49408_reg_read(dev, SPI_ERROR_ST, (u8 *)&spi_err, sizeof(u32));
		sw49408_reg_read(dev, SPI_FAULT_TYPE, (u8 *)&spi_fault, sizeof(u32));

		TOUCH_I("%s : Watchdog Exception - status : %x, ic_status : %x, spi_err : %x, spi_fault : %x\n",
			__func__, status, ic_status, spi_err, spi_fault);

		sw49408_te_info(dev, NULL);

		ret = -ERESTART;
	}

	debugging_mask = ((status >> 16) & 0xF);
	if (debugging_mask == 0x2) {
		if ((ret != -ERESTART) && (ret != -EUPGRADE)) {
			TOUCH_I("TC_Driving OK\n");
			d->tc_status_rst_cnt = 0;
			d->tc_status_fwup_cnt = 0;
			ret = -ERANGE;
		} else {
			return ret;
		}
		// debugging_mask 0x3 : error report
		// debugging_mask 0x4 : debugging report
	} else if (debugging_mask == 0x3 || debugging_mask == 0x4) {
		debugging_length = ((d->info.debug.ic_debug_info >> 24) & 0xFF);
		debugging_type = (d->info.debug.ic_debug_info & 0x00FFFFFF);

		TOUCH_E(
				"%s, INT_TYPE:%x,Length:%d,Type:%x,Log:%x %x %x\n",
				__func__, debugging_mask,
				debugging_length, debugging_type,
				d->info.debug.ic_debug[0], d->info.debug.ic_debug[1],
				d->info.debug.ic_debug[2]);
	}

	return ret;
}

static ssize_t sw49408_get_rn_data(struct device *dev, int16_t *buf, int frame_size)
{
	u32 data_offset = 0;

	struct sw49408_data *d = to_sw49408_data(dev);

	data_offset = d->reg_info.r_dbg_buf_sram_oft + (DBG_BUF2_OFFSET/4);
	TOUCH_I("data_offset = %d \n", data_offset);

	sw49408_reg_write(dev, spr_data_offset, (u8 *)&data_offset, sizeof(u32));

	sw49408_reg_read(dev, data_access_addr, (u8 *)buf, frame_size);

	return 0;
}

int sw49408_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	struct sw49408_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;

	/* check if palm detected */
	if (data[0].track_id >= WATER_ID) {
		if (data[0].event == TOUCHSTS_DOWN) {
			ts->is_cancel = 1;
			TOUCH_I("%s Detected\n", data[0].track_id == WATER_ID ?
					"Water" : "Palm");
		} else if (data[0].event == TOUCHSTS_UP) {
			ts->is_cancel = 0;
			TOUCH_I("%s Released\n", data[0].track_id == WATER_ID ?
					"Water" : "Palm");
		}
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < touch_count; i++) {
		if (data[i].track_id >= MAX_FINGER)
			continue;

		if (data[i].event == TOUCHSTS_DOWN
			|| data[i].event == TOUCHSTS_MOVE) {
			ts->new_mask |= (1 << data[i].track_id);
			tdata = ts->tdata + data[i].track_id;

			tdata->id = data[i].track_id;
			tdata->type = data[i].tool_type;
			tdata->x = data[i].x;
			tdata->y = data[i].y;
			tdata->pressure = data[i].pressure;
			tdata->width_major = data[i].width_major;
			tdata->width_minor = data[i].width_minor;

			if (data[i].width_major == data[i].width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = (s8)(data[i].angle);

			finger_index++;

			TOUCH_D(ABS,
				"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
					tdata->id,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);

		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

int sw49408_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);

	/* check if touch cnt is valid */
	if (d->info.touch_cnt == 0 || d->info.touch_cnt > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, d->info.touch_cnt);
		return -ERANGE;
	}

	return sw49408_irq_abs_data(dev);
}

int sw49408_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	if (d->info.wakeup_type == KNOCK_1) {
		if (ts->lpwg.mode != LPWG_NONE) {
			sw49408_get_tci_data(dev,
				ts->tci.info[TCI_1].tap_count);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		}
	} else if (d->info.wakeup_type == KNOCK_2) {
		if (ts->lpwg.mode >= LPWG_PASSWORD) {
			sw49408_get_tci_data(dev,
				ts->tci.info[TCI_2].tap_count);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
	} else if (d->info.wakeup_type == SWIPE_LEFT) {
		TOUCH_I("SWIPE_LEFT\n");
		sw49408_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
	} else if (d->info.wakeup_type == SWIPE_RIGHT) {
		TOUCH_I("SWIPE_RIGHT\n");
		sw49408_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
	} else if (d->info.wakeup_type == SWIPE_DOWN) {
		TOUCH_I("SWIPE_DOWN\n");
		sw49408_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_DOWN;
	} else if (d->info.wakeup_type == SWIPE_UP) {
		TOUCH_I("SWIPE_UP\n");
		sw49408_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_UP;
	} else if (d->info.wakeup_type == KNOCK_OVERTAP) {
		TOUCH_I("LPWG wakeup_type is Overtap\n");
		sw49408_get_tci_data(dev,
				ts->tci.info[TCI_2].tap_count + 1);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if (d->info.wakeup_type == CUSTOM_DEBUG) {
		TOUCH_I("LPWG wakeup_type is CUSTOM_DEBUG\n");
		sw49408_debug_tci(dev);
		sw49408_debug_swipe(dev);
	} else {
		TOUCH_I("LPWG wakeup_type is not support type![%d]\n",
			d->info.wakeup_type);
	}

	return ret;
}

void sw49408_irq_runtime_engine_debug(struct device *dev)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	u8 ocd_debug[OCD_SIZE];
	int a = 0;
	int b = 0;
	int start_point = 0;
	int cnt_point = 0;
	int index[11] = {0, 6, 12, 18, 30, 42, 54, 60, 68, 86, 101};
	int order[11] = {6, 6, 6, 12, 12, 10, 8, 8, 18, 15, 3};
	int16_t *rndata = NULL;
	int ret = 0;
	int x = 0;
	int y = 0;

	/* runtime_dbg_inttype = 0 : no action
	   runtime_dbg_inttype = 1 : ocd
	   runtime_dbg_inttype = 2 : RN
	   runtime_dbg_inttype = 3 : ocd & RN */
	if((d->info.debug.runtime_dbg_inttype == 1) || (d->info.debug.runtime_dbg_inttype == 3))
	{
		TOUCH_I("ABT Engine Debug : runtime_dbg_inttype = %d, runtime_dbg_case = %04X \n",
				d->info.debug.runtime_dbg_inttype, d->info.debug.runtime_dbg_case);
		memcpy(&ocd_debug, &d->info.debug, sizeof(d->info.debug));

		for (a = 0 ; a < 11; a++)
		{
			char debug_buf[LOG_BUF_SIZE] = {0,};
			int debug_ret = 0;

			debug_ret += snprintf(debug_buf + debug_ret, LOG_BUF_SIZE - debug_ret, "[%2d]  ", index[a]);

			start_point = index[a];
			cnt_point = order[a];

			for (b = 0 ; b < cnt_point; b++)
			{
				debug_ret += snprintf(debug_buf + debug_ret, LOG_BUF_SIZE - debug_ret, "%5d ", ocd_debug[start_point + b]);
			}
			TOUCH_I("%s \n", debug_buf);
		}
	}

	if((d->info.debug.runtime_dbg_inttype == 2) || (d->info.debug.runtime_dbg_inttype == 3))
	{
		int rnsize = sizeof(int16_t) * (ROW_SIZE * COL_SIZE);

		TOUCH_I("ABT Engine Debug : runtime_dbg_inttype = %d, runtime_dbg_case = %04X \n",
				d->info.debug.runtime_dbg_inttype, d->info.debug.runtime_dbg_case);

		if (rnsize % 4)
			rnsize = (((rnsize >> 2) + 1) << 2);

		rndata = kzalloc(rnsize, GFP_KERNEL);

		if (rndata == NULL) {
			TOUCH_E("mem_error\n");
			goto error;
		}

		ret = sw49408_get_rn_data(dev, rndata, rnsize);

		if (ret == 1) {
			TOUCH_E("rn_data_get error \n");
			goto error;
		}

		for (x = 0 ; x < ROW_SIZE ; x++)
		{
			char debug_buf[LOG_BUF_SIZE] = {0,};
			int debug_ret = 0;

			debug_ret += snprintf(debug_buf + debug_ret, LOG_BUF_SIZE - debug_ret, "[%2d]  ", x);

			for (y = 0 ; y < COL_SIZE ; y++)
			{
				debug_ret += snprintf(debug_buf + debug_ret, LOG_BUF_SIZE - debug_ret, "%5d ", rndata[x * COL_SIZE + y]);
			}
			TOUCH_I("%s\n", debug_buf);
		}
	}

error:
	if (rndata != NULL)
		kfree(rndata);
}

int sw49408_irq_handler(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	pm_qos_update_request(&d->pm_qos_req, 10);
	sw49408_reg_read(dev, tc_ic_status, &d->info,
				sizeof(d->info));
	ret = sw49408_check_status(dev);
	pm_qos_update_request(&d->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	if (ret < 0)
		goto error;
	if (d->info.wakeup_type == ABS_MODE) {
		ret = sw49408_irq_abs(dev);
	} else {
		ret = sw49408_irq_lpwg(dev);
	}

	if (atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_2)
		if (d->info.debug.runtime_dbg_inttype > 0)
			sw49408_irq_runtime_engine_debug(dev);

error:
	return ret;
}

static ssize_t store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	char command[6] = {0};
	u32 reg = 0;
	int value = 0;
	u32 data = 1;
	u16 reg_addr;

	if (sscanf(buf, "%5s %x %x", command, &reg, &value) <= 0)
		return count;

	mutex_lock(&ts->lock);
	reg_addr = reg;
	if (!strcmp(command, "write")) {
		data = value;
		if (sw49408_reg_write(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else if (!strcmp(command, "read")) {
		if (sw49408_reg_read(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else {
		TOUCH_D(BASE_INFO, "Usage\n");
		TOUCH_D(BASE_INFO, "Write reg value\n");
		TOUCH_D(BASE_INFO, "Read reg\n");
	}
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t show_tci_debug(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	u32 rdata = -1;

	mutex_lock(&ts->lock);
	if (sw49408_reg_read(dev, d->reg_info.r_abt_sts_spi_addr +
				TCI_FAIL_DEBUG_R,
				(u8 *)&rdata, sizeof(rdata)) < 0) {
		TOUCH_I("Fail to Read TCI Debug Reason type\n");
		mutex_unlock(&ts->lock);
		return ret;
	}
	mutex_unlock(&ts->lock);

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Read TCI Debug Reason type[IC] = %s\n",
			debug_type[(rdata & 0x8) ? 2 :
					(rdata & 0x4 ? 1 : 0)]);
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Read TCI Debug Reason type[Driver] = %s\n",
			debug_type[d->tci_debug_type]);
	TOUCH_I("Read TCI Debug Reason type = %s\n",
			debug_type[d->tci_debug_type]);

	return ret;
}

static ssize_t store_tci_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 2 || value < 0) {
		TOUCH_I("SET TCI debug reason wrong, 0, 1, 2 only\n");
		return count;
	}

	d->tci_debug_type = (u8)value;
	TOUCH_I("SET TCI Debug reason type = %s\n", debug_type[value]);

	return count;
}

static ssize_t show_swipe_debug(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;
	u32 rdata = -1;

	mutex_lock(&ts->lock);
	if (sw49408_reg_read(dev, d->reg_info.r_abt_sts_spi_addr +
				SWIPE_FAIL_DEBUG_R,
				(u8 *)&rdata, sizeof(rdata)) < 0) {
		TOUCH_I("Fail to Read SWIPE Debug reason type\n");
		mutex_unlock(&ts->lock);
		return ret;
	}
	mutex_unlock(&ts->lock);

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Read SWIPE Debug reason type[IC] = %s\n",
			debug_type[(rdata & 0x2) ? 2 :
					(rdata & 0x1 ? 1 : 0)]);
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Read SWIPE Debug reason type[Driver] = %s\n",
			debug_type[d->swipe_debug_type]);
	TOUCH_I("Read SWIPE Debug reason type = %s\n",
			debug_type[d->swipe_debug_type]);

	return ret;
}

static ssize_t store_swipe_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 2 || value < 0) {
		TOUCH_I("SET SWIPE debug reason wrong, 0, 1, 2 only\n");
		return count;
	}

	d->swipe_debug_type = (u32)value;
	TOUCH_I("Write SWIPE Debug reason type = %s\n", debug_type[value]);

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf,
								 size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	mutex_lock(&ts->lock);
	touch_interrupt_control(dev, INTERRUPT_DISABLE);
	sw49408_reset_ctrl(dev, value);

	sw49408_init(dev);
	touch_interrupt_control(dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t store_q_sensitivity(struct device *dev, const char *buf,
								size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	mutex_lock(&ts->lock);
	d->q_sensitivity = value;
	sw49408_reg_write(dev, d->reg_info.r_abt_cmd_spi_addr +
				QCOVER_SENSITIVITY,
				&d->q_sensitivity, sizeof(u32));
	mutex_unlock(&ts->lock);

	TOUCH_I("%s : %s(%d)\n", __func__,
			value ? "SENSITIVE" : "NORMAL", value);

	return count;
}

static ssize_t show_te(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	mutex_lock(&ts->lock);
	ret = sw49408_te_info(dev, buf);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_te_test(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);

	queue_delayed_work(d->wq_log, &d->te_test_work, 0);

	return 0;
}

static ssize_t show_te_result(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	TOUCH_I("DDIC Test result : %s\n", d->te_ret ? "Fail" : "Pass");
	ret = snprintf(buf + ret, PAGE_SIZE, "DDIC Test result : %s\n",
			d->te_ret ? "Fail" : "Pass");

#ifdef CONFIG_LGE_TOUCH_LGSIC_SW49408_PRD
	if (d->te_write_log == DO_WRITE_LOG) {
		struct touch_core_data *ts = to_touch_core(dev);

		mutex_lock(&ts->lock);
		sw49408_te_test_logging(d->dev, buf);
		mutex_unlock(&ts->lock);
		d->te_write_log = LOG_WRITE_DONE;
	}
#endif

	return ret;
}

static ssize_t show_lcd_block_result(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49408_data *d = to_sw49408_data(dev);

	int ret = 0;
	u32 status = 0;
	u32 lcd_block = 0;

	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, 63, "not support on u%d\n",
				d->lcd_mode);
		return ret;
	}

	TOUCH_I("lcd block check Start\n");

	mutex_lock(&ts->lock);
	if (sw49408_reg_read(dev, tc_status,
				(u32 *)&status, sizeof(status)) < 0) {
		ret = snprintf(buf + ret, PAGE_SIZE,
				"lcd block ocurred addr read error\n");
		TOUCH_E("lcd block ocurred addr read error\n");
		mutex_unlock(&ts->lock);
		return ret;
	}
	mutex_unlock(&ts->lock);
	if(status & (1 << 31))
		lcd_block = 1;

	ret = snprintf(buf + ret, PAGE_SIZE, "%x\n", lcd_block);
	TOUCH_I("tc status : %x, lcd block status : %x\n",
			status, lcd_block);
	TOUCH_I("lcd block check End\n");

	return ret;
}

static ssize_t show_swipe_enable(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int ret = 0;

	ret = snprintf(buf + ret, PAGE_SIZE, "SWIPE mode = 0x%X\n",
			d->swipe.mode);

	TOUCH_I("Swipe mode = %s\n",
			d->swipe.mode & SWIPE_UP_BIT ? "SWIPE_UP" : "Disable");

	return ret;
}

static ssize_t store_swipe_enable(struct device *dev,
						const char *buf, size_t count)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 1 || value < 0) {
		TOUCH_I("Set Swipe mode wrong, 0, 1 only\n");
		return count;
	}

	d->swipe.mode = value ? SWIPE_UP_BIT | SWIPE_UP_RELEASE_ENABLE : 0;

	TOUCH_I("%s : %s\n", __func__, value ? "SWIPE_UP" : "Disable");

	return count;
}

static TOUCH_ATTR(te_test, show_te_test, NULL);
static TOUCH_ATTR(te_result, show_te_result, NULL);
static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(tci_debug, show_tci_debug, store_tci_debug);
static TOUCH_ATTR(swipe_debug, show_swipe_debug, store_swipe_debug);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(q_sensitivity, NULL, store_q_sensitivity);
static TOUCH_ATTR(te, show_te, NULL);
static TOUCH_ATTR(lcd_block_result, show_lcd_block_result, NULL);
static TOUCH_ATTR(swipe_enable, show_swipe_enable, store_swipe_enable);

static struct attribute *sw49408_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_tci_debug.attr,
	&touch_attr_swipe_debug.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_q_sensitivity.attr,
	&touch_attr_te.attr,
	&touch_attr_te_test.attr,
	&touch_attr_te_result.attr,
	&touch_attr_lcd_block_result.attr,
	&touch_attr_swipe_enable.attr,
	NULL,
};

static const struct attribute_group sw49408_attribute_group = {
	.attrs = sw49408_attribute_list,
};

static int sw49408_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &sw49408_attribute_group);
	if (ret < 0)
		TOUCH_E("sw49408 sysfs register failed\n");

	//sw49408_prd_register_sysfs(dev);
	sw49408_sic_abt_register_sysfs(&ts->kobj);

	return 0;
}

static int sw49408_get_cmd_version(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int offset = 0;
	int ret = 0;
	u32 rdata[2] = {0};

	ret = sw49408_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	if (d->ic_info.version.build) {
		offset = snprintf(buf + offset, PAGE_SIZE - offset,
			"version : v%d.%02d.%d\n", d->ic_info.version.major,
			d->ic_info.version.minor, d->ic_info.version.build);
	} else {
		offset = snprintf(buf + offset, PAGE_SIZE - offset,
			"version : v%d.%02d\n", d->ic_info.version.major,
			d->ic_info.version.minor);
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
		"chip_rev : %d, fpc : %d, lcm : %d, lot : %d\n",
			d->ic_info.chip_revision, d->ic_info.fpc, d->ic_info.lcm, d->ic_info.lot);

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
		"product id : [%s]\n\n", d->ic_info.product_id);

	sw49408_reg_read(dev, d->reg_info.r_pt_info_spi_addr + pt_info_date,
						(u8 *)&rdata, sizeof(rdata));
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"date : %04d.%02d.%02d " \
				"%02d:%02d:%02d Site%d\n",
		rdata[0] & 0xFFFF, (rdata[0] >> 16 & 0xFF),
		(rdata[0] >> 24 & 0xFF), rdata[1] & 0xFF,
		(rdata[1] >> 8 & 0xFF), (rdata[1] >> 16 & 0xFF),
		(rdata[1] >> 24 & 0xFF));

	return offset;
}

static int sw49408_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct sw49408_data *d = to_sw49408_data(dev);
	int offset = 0;
	int ret = 0;

	ret = sw49408_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	if (d->ic_info.version.build) {
		offset = snprintf(buf, PAGE_SIZE, "v%d.%02d.%d\n",
			d->ic_info.version.major, d->ic_info.version.minor,
			d->ic_info.version.build);
	} else {
		offset = snprintf(buf, PAGE_SIZE, "v%d.%02d\n",
			d->ic_info.version.major, d->ic_info.version.minor);
	}

	return offset;
}

static int sw49408_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int sw49408_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = sw49408_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = sw49408_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static struct touch_driver touch_driver = {
	.probe = sw49408_probe,
	.remove = sw49408_remove,
	.suspend = sw49408_suspend,
	.resume = sw49408_resume,
	.init = sw49408_init,
	.irq_handler = sw49408_irq_handler,
	.power = sw49408_power,
	.upgrade = sw49408_upgrade,
	.lpwg = sw49408_lpwg,
	.notify = sw49408_notify,
	.register_sysfs = sw49408_register_sysfs,
	.set = sw49408_set,
	.get = sw49408_get,
};

#define MATCH_NAME			"lge,sw49408"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_SPI,
	.name = "sw49408",
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
	.bits_per_word = 8,
	.spi_mode = SPI_MODE_0,
	.max_freq = (25 * 1000000),
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();

	if (touch_get_device_type() != TYPE_SW49408 ) {
		TOUCH_I("%s, sw49408 returned\n", __func__);
		return 0;
	}

	TOUCH_I("%s, sw49408 start\n", __func__);

	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("hoyeon.jang@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");

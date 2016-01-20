/* CwMcuSensor.c - driver file for HTC SensorHUB
 *
 * Copyright (C) 2014 HTC Ltd.
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

#ifndef __CWMCUSENSOR_BUS_H__
#define __CWMCUSENSOR_BUS_H__

#include <linux/kernel.h>

struct cwmcu_bus_client {
	struct i2c_client *i2c_client;
	struct spi_device *spi_client;
};

int mcu_do_tx(const struct cwmcu_bus_client *client, u8 *buf, int len);
int mcu_do_rx(const struct cwmcu_bus_client *client, u8 *buf, int len);
int CWMCU_do_write(struct cwmcu_bus_client *mcu_client,
			  u8 reg_addr, u8 data);
int CWMCU_do_write_block(struct cwmcu_bus_client *mcu_client,
			  u8 reg_addr, u8 len, u8 *data);
int CWMCU_do_read(struct cwmcu_bus_client *mcu_client,
			 u8 reg_addr, u8 len, u8 *data);
int mcu_spi_tx_cmd(const struct cwmcu_bus_client *mcu_client, u8 cmd, u8 *data, int len);
int mcu_spi_rx_cmd(const struct cwmcu_bus_client *mcu_client, u8 cmd, u8 *data, int len);






#endif /* __CWMCUSENSOR_BUS_H__ */

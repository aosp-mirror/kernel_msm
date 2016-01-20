/* CwMcuSensor_i2c.c - driver file for HTC SensorHUB
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

#include <linux/CwMcuSensor_bus.h>
#include <linux/i2c.h>


int mcu_do_tx(const struct cwmcu_bus_client *client, u8 *buf, int len)
{
	int ret = 0;
	ret = i2c_master_send(client->i2c_client, (char *)buf, (int)len);
	return ret;
}

int mcu_do_rx(const struct cwmcu_bus_client *client, u8 *buf, int len)
{
	int ret = 0;
	ret = i2c_master_recv(client->i2c_client, (char *)buf, (int)len);
	return ret;
}


int CWMCU_do_write(struct cwmcu_bus_client *mcu_client,
			  u8 reg_addr, u8 data)
{
	int rc = 0;
	rc = i2c_smbus_write_byte_data(mcu_client->i2c_client,
						  reg_addr, data);
	return rc;
}

int CWMCU_do_write_block(struct cwmcu_bus_client *mcu_client,
			  u8 reg_addr, u8 len, u8 *data)
{
	int rc = 0;
	rc = i2c_smbus_write_i2c_block_data(mcu_client->i2c_client,
						  reg_addr, len, data);
	return rc;
}


int CWMCU_do_read(struct cwmcu_bus_client *mcu_client,
			 u8 reg_addr, u8 len, u8 *data)
{
	int rc = 0;
	rc = i2c_smbus_read_i2c_block_data(mcu_client->i2c_client, reg_addr,
						   len, data);
	return rc;
}

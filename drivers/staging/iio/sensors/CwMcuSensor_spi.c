/* CwMcuSensor_spi.c - driver file for HTC SensorHUB
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
#include <linux/spi/spi.h>
#include <linux/string.h>

#define MCU_SPI_MAX_WRITE_LENGTH	32
#define MCU_SPI_MAX_READ_LENGTH		32
#define MCU_SPI_DUMMY_LENGTH		48


#define MCU_SPI_REG_RW_PROTOCOL_LEN 3
#define MCU_SPI_REG_READ_ACK_CRC_LEN 3
#define MCU_SPI_RW_PACKAGE_LEN 		4

#define MCU_SPI_READ_PROTOCOL       0x55
#define MCU_SPI_WRITE_PROTOCOL      0xAA
#define MCU_SPI_RW_ACK1             0x5A
#define MCU_SPI_RW_ACK2             0xA5

static int spi_check_crc(u8 *data, u8 len, u8 crc, u8 check){
	int i;
	u8 crc_data = 0xFF;

	for (i = 0; i < len ; i++) {
		crc_data ^= data[i];
	}

	if(!check)
		return crc_data;
	if(crc_data == crc)
		return 0;
	else {
		return -1;
	}
}

static int spi_xfer(const struct cwmcu_bus_client *client, unsigned char *txbuf,unsigned char *rxbuf, int len)
{
	int ret;
    struct spi_transfer transfer_1;

	transfer_1.bits_per_word = 8;
	transfer_1.speed_hz = 1000000;
	transfer_1.tx_nbits = SPI_NBITS_SINGLE;
	transfer_1.rx_nbits = SPI_NBITS_SINGLE;
	transfer_1.tx_buf =(txbuf==NULL)?NULL: txbuf;
	transfer_1.rx_buf =(rxbuf==NULL)?NULL: rxbuf;
	transfer_1.len = len;
	transfer_1.delay_usecs = 10;

	ret = spi_sync_transfer(client->spi_client, &transfer_1, 1);
	return ret;
}

static void spi_transfer_init(struct spi_transfer *transfer, int size)
{
	int i = 0;

	for(i = 0; i < size; i++) {
		transfer[i].bits_per_word = 8;
		transfer[i].speed_hz = 1000000;
		transfer[i].tx_nbits = SPI_NBITS_SINGLE;
		transfer[i].rx_nbits = SPI_NBITS_SINGLE;
		transfer[i].tx_buf = NULL;
		transfer[i].rx_buf = NULL;
		transfer[i].delay_usecs = 0;
	}
}

int mcu_do_tx(const struct cwmcu_bus_client *client, u8 *buf, int len)
{
	int ret = 0;
	unsigned char* tx_buf=NULL, *rx_buf=NULL;

	tx_buf = buf;
	rx_buf = NULL;
	ret = spi_xfer(client, tx_buf,rx_buf, len);
	if(ret < 0)
	return ret;
	return len;
}

int mcu_do_rx(const struct cwmcu_bus_client *client, u8 *buf, int len)
{
	int ret = 0;
	unsigned char *tx_buf=NULL, *rx_buf=NULL;

	tx_buf = NULL;
	rx_buf = buf;
	ret = spi_xfer(client, tx_buf, rx_buf, len);
    if(ret < 0)
	return ret;
	return len;
}

int CWMCU_do_write(struct cwmcu_bus_client *mcu_client,
			  u8 reg_addr, u8 data)
{
	return CWMCU_do_write_block(mcu_client, reg_addr, 1, &data);
}

int CWMCU_do_write_block(struct cwmcu_bus_client *mcu_client,
			  u8 reg_addr, u8 len, u8 *data)
{
	struct spi_transfer transfer[1];
	int ret = 0, i = 0, transfer_len = 0;
	u8 spi_protocol_data[MCU_SPI_MAX_WRITE_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 spi_dummy_data[MCU_SPI_MAX_WRITE_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 crc = 0;

	if (len > MCU_SPI_MAX_WRITE_LENGTH)
        return -EINVAL;

	spi_transfer_init(transfer, ARRAY_SIZE(transfer));
	transfer_len = 0;

	spi_protocol_data[0] = MCU_SPI_WRITE_PROTOCOL;
	spi_protocol_data[1] = reg_addr;
	spi_protocol_data[2] = len;
	spi_protocol_data[3] = spi_check_crc(spi_protocol_data, MCU_SPI_REG_RW_PROTOCOL_LEN, crc, 0);

	transfer[transfer_len].tx_buf = spi_protocol_data; // mcu spi  protocol
	transfer[transfer_len].len = len + MCU_SPI_DUMMY_LENGTH;
	transfer[transfer_len].rx_buf = spi_dummy_data; // mcu spi  protocol
	transfer_len++;

	crc = spi_check_crc(data, len, crc, 0);

	memcpy(spi_protocol_data + MCU_SPI_RW_PACKAGE_LEN, data, len);
	spi_protocol_data[MCU_SPI_RW_PACKAGE_LEN + len] = crc;

	ret = spi_sync_transfer(mcu_client->spi_client, transfer, transfer_len);
	if (ret < 0)
	{
		return ret;
	}

	for (i = 0; i < len + MCU_SPI_DUMMY_LENGTH - 1 ; i++) {
		if(spi_dummy_data[i] == MCU_SPI_RW_ACK1 && spi_dummy_data[i+1] == MCU_SPI_RW_ACK2) {
			return len;
		}
	}

	ret = -EIO;
	return ret;
}


int CWMCU_do_read(struct cwmcu_bus_client *mcu_client,
			 u8 reg_addr, u8 len, u8 *data)
{
	struct spi_transfer transfer[1];
	int ret = 0, transfer_len = 0, data_index = 0, i = 0, data_start_index = 0;
	u8 mcu_ack[MCU_SPI_RW_PACKAGE_LEN] = {0};
	u8 spi_protocol_data[MCU_SPI_MAX_READ_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 spi_dummy_data[MCU_SPI_MAX_READ_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 crc = 0;

	if (len > MCU_SPI_MAX_READ_LENGTH)
		return -EINVAL;

	spi_transfer_init(transfer, ARRAY_SIZE(transfer));
	transfer_len = 0;

	spi_protocol_data[data_index++] = MCU_SPI_READ_PROTOCOL;
	spi_protocol_data[data_index++] = reg_addr;
	spi_protocol_data[data_index++] = len;
	spi_protocol_data[data_index++] = spi_check_crc(spi_protocol_data, MCU_SPI_REG_RW_PROTOCOL_LEN, crc, 0);

	transfer[transfer_len].tx_buf = spi_protocol_data;

	transfer[transfer_len].rx_buf = spi_dummy_data;
	transfer[transfer_len].len = len + MCU_SPI_DUMMY_LENGTH;
	transfer_len++;

	ret = spi_sync_transfer(mcu_client->spi_client, transfer, transfer_len);
	if (ret < 0)
	{
		return ret;
	}

	for(i = data_index;i < len + MCU_SPI_DUMMY_LENGTH - 1;i++) {
		if(spi_dummy_data[i] == MCU_SPI_RW_ACK1 && spi_dummy_data[i+1] == MCU_SPI_RW_ACK2) {
			mcu_ack[0] = spi_dummy_data[i];
			mcu_ack[1] = spi_dummy_data[i+1];
			data_start_index = i + 2;
			memcpy(data, spi_dummy_data + data_start_index, len);
			break;
		}
	}

	if(mcu_ack[0] != MCU_SPI_RW_ACK1 || mcu_ack[1] != MCU_SPI_RW_ACK2) {
		ret = -EIO;
		return ret;
	}

	crc = spi_dummy_data[data_start_index + len];

	if(spi_check_crc(data, len, crc, 1) < 0) {
		return -EAGAIN;
	}

	return len;
}


#define MCU_DLOAD_SPI_MAX_READ_LENGTH	128
#define MCU_DLOAD_SPI_MAX_WRITE_LENGTH	32

#define MCU_DLOAD_SPI_READ_PROTOCOL  0x33
#define MCU_DLOAD_SPI_WRITE_PROTOCOL 0xCC
#define MCU_DLOAD_SPI_RW_ACK1        0x3C
#define MCU_DLOAD_SPI_RW_ACK2        0xC3

int mcu_spi_tx_cmd(const struct cwmcu_bus_client *mcu_client, u8 cmd, u8 *data, int len)
{
	struct spi_transfer transfer[1];
	int ret = 0, i = 0, transfer_len = 0;
	u8 spi_protocol_data[MCU_DLOAD_SPI_MAX_WRITE_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 spi_dummy_data[MCU_DLOAD_SPI_MAX_WRITE_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 crc = 0;

	if (len > MCU_DLOAD_SPI_MAX_WRITE_LENGTH)
        return -EINVAL;

	spi_transfer_init(transfer, ARRAY_SIZE(transfer));
	transfer_len = 0;

	spi_protocol_data[0] = MCU_DLOAD_SPI_WRITE_PROTOCOL;
	spi_protocol_data[1] = cmd;
	spi_protocol_data[2] = len;
	spi_protocol_data[3] = spi_check_crc(spi_protocol_data, MCU_SPI_REG_RW_PROTOCOL_LEN, crc, 0);

	transfer[transfer_len].tx_buf = spi_protocol_data; // mcu spi  protocol
	transfer[transfer_len].len = len + MCU_SPI_DUMMY_LENGTH;
	transfer[transfer_len].rx_buf = spi_dummy_data; // mcu spi  protocol
	transfer_len++;

	crc = spi_check_crc(data, len, crc, 0);

	memcpy(spi_protocol_data + MCU_SPI_RW_PACKAGE_LEN, data, len);
	spi_protocol_data[MCU_SPI_RW_PACKAGE_LEN + len] = crc;

	ret = spi_sync_transfer(mcu_client->spi_client, transfer, transfer_len);
	if (ret < 0)
	{
		return ret;
	}

	for (i = 0; i < len + MCU_SPI_DUMMY_LENGTH - 1 ; i++) {
		if(spi_dummy_data[i] == MCU_DLOAD_SPI_RW_ACK1 && spi_dummy_data[i+1] == MCU_DLOAD_SPI_RW_ACK2) {
			return len;
		}
	}

	ret = -EIO;
	return ret;
}

int mcu_spi_rx_cmd(const struct cwmcu_bus_client *mcu_client, u8 cmd, u8 *data, int len)
{
	struct spi_transfer transfer[1];
	int ret = 0, transfer_len = 0, data_index = 0, i = 0, data_start_index = 0;
	u8 mcu_ack[MCU_SPI_RW_PACKAGE_LEN] = {0};
	u8 spi_protocol_data[MCU_DLOAD_SPI_MAX_READ_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 spi_dummy_data[MCU_DLOAD_SPI_MAX_READ_LENGTH + MCU_SPI_DUMMY_LENGTH] = {0};
	u8 crc = 0;

	if (len > MCU_DLOAD_SPI_MAX_READ_LENGTH)
		return -EINVAL;

	spi_transfer_init(transfer, ARRAY_SIZE(transfer));
	transfer_len = 0;

	spi_protocol_data[data_index++] = MCU_DLOAD_SPI_READ_PROTOCOL;
	spi_protocol_data[data_index++] = cmd;
	spi_protocol_data[data_index++] = len;
	spi_protocol_data[data_index++] = spi_check_crc(spi_protocol_data, MCU_SPI_REG_RW_PROTOCOL_LEN, crc, 0);

	transfer[transfer_len].tx_buf = spi_protocol_data;

	transfer[transfer_len].rx_buf = spi_dummy_data;
	transfer[transfer_len].len = len + MCU_SPI_DUMMY_LENGTH;
	transfer_len++;

	ret = spi_sync_transfer(mcu_client->spi_client, transfer, transfer_len);
	if (ret < 0)
	{
		return ret;
	}

	for(i = data_index;i < len + MCU_SPI_DUMMY_LENGTH - 1;i++) {
		if(spi_dummy_data[i] == MCU_DLOAD_SPI_RW_ACK1 && spi_dummy_data[i+1] == MCU_DLOAD_SPI_RW_ACK2) {
			mcu_ack[0] = spi_dummy_data[i];
			mcu_ack[1] = spi_dummy_data[i+1];
			data_start_index = i + 2;
			memcpy(data, spi_dummy_data + data_start_index, len);
			break;
		}
	}

	if(mcu_ack[0] != MCU_DLOAD_SPI_RW_ACK1 || mcu_ack[1] != MCU_DLOAD_SPI_RW_ACK2) {
		ret = -EIO;
		return ret;
	}

	crc = spi_dummy_data[data_start_index + len];

	if(spi_check_crc(data, len, crc, 1) < 0) {
		return -EAGAIN;
	}

	return len;
}

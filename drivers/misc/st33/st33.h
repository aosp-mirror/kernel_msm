/*
 * Copyright (C) 2017 ST Microelectronics S.A.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
 
#define ST33_MAGIC	0xEA
#define ST33_NAME "st33"
#define ST33_SPI_FREQ_MIN 100000 /* 100KHz */
#define ST33_SPI_FREQ_MAX 10000000 /* 3MHz */


struct st33_platform_data {
    unsigned int irq_gpio;
    unsigned int pow_gpio;
};

#define ST33_GET_IRQ_LEVEL			_IOR(ST33_MAGIC, 0x01, unsigned int)
#define ST33_POW_ON					_IOR(ST33_MAGIC, 0x02, unsigned int)
#define ST33_POW_OFF				_IOR(ST33_MAGIC, 0x03, unsigned int)
#define ST33_RECOVERY				_IOR(ST33_MAGIC, 0x04, unsigned int)
#define ST33_GET_SPI_SPEED_HZ		_IOR(ST33_MAGIC, 0x10, unsigned int)
#define ST33_SET_SPI_SPEED_HZ		_IOR(ST33_MAGIC, 0x11, unsigned int)


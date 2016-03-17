/*
 * fusb302 usb phy driver for type-c and PD
 *
 * Copyright (C) 2015, 2016 Fairchild Semiconductor Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Seee the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "fusb30X.h"
#include "platform.h"

FSC_BOOL DeviceWrite(FSC_U8 regAddr, FSC_U8 length, FSC_U8* data)
{
    FSC_BOOL error;
    error = platform_i2c_write(FUSB300SlaveAddr, FUSB300AddrLength, length, length, FUSB300IncSize, regAddr, data);
    if (error == FALSE)
        return TRUE;
    return FALSE;
}

FSC_BOOL DeviceRead(FSC_U8 regAddr, FSC_U8 length, FSC_U8* data)
{
    FSC_BOOL error;
    error = platform_i2c_read(FUSB300SlaveAddr, FUSB300AddrLength, length, length, FUSB300IncSize, regAddr, data);
    if (error == FALSE)
        return TRUE;
    return FALSE;
}

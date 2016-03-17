/*
* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include "tfa_dsp_fw.h"
#include "tfa_service.h"
#include "tfa_internal.h"

#include "tfa98xx_tfafieldnames.h"

#ifdef TFA98XX_FULL
/** clockless way to determine if this is the tfa9887
 *  by testing if the PVP bit is writable
 */
int tfa9887B_is87(Tfa98xx_handle_t handle)
{
	unsigned short save_value, check_value;

	tfa98xx_read_register16(handle, 0x08, &save_value);
	if ((save_value&0x0400) == 0) /* if clear it's 87 */
		return 1;
	/* try to clear pvp bit */
	tfa98xx_write_register16(handle, 0x08, (save_value & ~0x0400));
	tfa98xx_read_register16(handle, 0x08, &check_value);
	/* restore */
	tfa98xx_write_register16(handle, 0x08, save_value);
	/* could we write the bit */
	return (check_value != save_value) ? 1 : 0; /* if changed it's the 87 */
}
#endif

static enum Tfa98xx_Error tfa9887B_specific(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int result;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	/* all i2C registers are already set to default */

	result = TFA_SET_BF(handle, AMPE, 1);
	if (result < 0)
		return -result;

	/* some other registers must be set for optimal amplifier behaviour */
	tfa98xx_write_register16(handle, 0x05, 0x13AB);
	tfa98xx_write_register16(handle, 0x06, 0x001F);
	/* peak voltage protection is always on, but may be written */
	tfa98xx_write_register16(handle, 0x08, 0x3C4E);
	/*TFA98XX_SYSCTRL_DCA=0*/
	tfa98xx_write_register16(handle, 0x09, 0x024D);
	tfa98xx_write_register16(handle, 0x41, 0x0308);
	error = tfa98xx_write_register16(handle, 0x49, 0x0E82);

	return error;
}

/*
 * register device specifics functions
 */
void tfa9887B_ops(struct tfa_device_ops *ops)
{
	ops->tfa_init = tfa9887B_specific;
}

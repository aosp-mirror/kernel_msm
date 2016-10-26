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

static enum Tfa98xx_Error tfa9891_specific(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	/* ----- generated code start ----- */
	/* -----  version 18.0 ----- */
	tfa98xx_write_register16(handle, 0x09, 0x025d); /*POR=0x024d*/
	tfa98xx_write_register16(handle, 0x10, 0x0018); /*POR=0x0024*/
	tfa98xx_write_register16(handle, 0x22, 0x0003); /*POR=0x0023*/
	tfa98xx_write_register16(handle, 0x25, 0x0001); /*POR=0x0000*/
	tfa98xx_write_register16(handle, 0x46, 0x0000); /*POR=0x4000*/
	tfa98xx_write_register16(handle, 0x55, 0x3ffb); /*POR=0x7fff*/
	/* ----- generated code end   ----- */

	return error;
}

/*
 * register device specifics functions
 */
void tfa9891_ops(struct tfa_device_ops *ops)
{
	ops->tfa_init = tfa9891_specific;

}

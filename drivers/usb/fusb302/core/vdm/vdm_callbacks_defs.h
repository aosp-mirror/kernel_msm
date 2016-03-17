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
#ifdef FSC_HAVE_VDM

#ifndef __FSC_VDM_CALLBACKS_DEFS_H__
#define __FSC_VDM_CALLBACKS_DEFS_H__
/*
 * This file defines types for callbacks that the VDM block will use.
 * The intention is for the user to define functions that return data 
 * that VDM messages require, ex whether or not to enter a mode.
 */

#include "vdm_types.h"
#include "../PD_Types.h" 

typedef Identity (*RequestIdentityInfo)(void);

typedef SvidInfo (*RequestSvidInfo)(void);

typedef ModesInfo (*RequestModesInfo)(FSC_U16);

typedef FSC_BOOL (*ModeEntryRequest)(FSC_U16 svid, FSC_U32 mode_index);

typedef FSC_BOOL (*ModeExitRequest)(FSC_U16 svid, FSC_U32 mode_index);

typedef FSC_BOOL (*EnterModeResult)(FSC_BOOL success, FSC_U16 svid, FSC_U32 mode_index);

typedef void (*ExitModeResult)(FSC_BOOL success, FSC_U16 svid, FSC_U32 mode_index);

typedef void (*InformIdentity)(FSC_BOOL success, SopType sop, Identity id);

typedef void (*InformSvids)(FSC_BOOL success, SopType sop, SvidInfo svid_info);

typedef void (*InformModes)(FSC_BOOL success, SopType sop, ModesInfo modes_info);

typedef void (*InformAttention)(FSC_U16 svid, FSC_U8 mode_index);

#endif // __FSC_VDM_CALLBACKS_DEFS_H__

#endif // FSC_HAVE_VDM


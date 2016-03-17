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

/*
 * Using this file to emulate the callbacks that a real DPM would set up for the VDM block.
 * Setting default values etc in here too.
 */
#ifdef FSC_HAVE_VDM

#ifndef __DPM_EMULATION_H__
#define __DPM_EMULATION_H__
 
#include "../platform.h"
#include "vdm_types.h"
#include "../PD_Types.h"

Identity 	vdmRequestIdentityInfo		(void);
SvidInfo 	vdmRequestSvidInfo			(void);
ModesInfo 	vdmRequestModesInfo         (FSC_U16 svid);
FSC_BOOL	vdmModeEntryRequest         (FSC_U16 svid, FSC_U32 mode_index);
FSC_BOOL	vdmModeExitRequest			(FSC_U16 svid, FSC_U32 mode_index);
FSC_BOOL 	vdmEnterModeResult			(FSC_BOOL success, FSC_U16 svid, FSC_U32 mode_index);
void		vdmExitModeResult			(FSC_BOOL success, FSC_U16 svid, FSC_U32 mode_index);
void		vdmInformIdentity			(FSC_BOOL success, SopType sop, Identity id);
void 		vdmInformSvids				(FSC_BOOL success, SopType sop, SvidInfo svid_info);
void		vdmInformModes				(FSC_BOOL success, SopType sop, ModesInfo modes_info);
void 		vdmInformAttention			(FSC_U16 svid, FSC_U8 mode_index);

void        vdmInitDpm                  (void);

#endif // __DPM_EMULATION_H__

#endif // FSC_HAVE_VDM

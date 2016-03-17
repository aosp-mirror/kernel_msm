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

#ifndef __FSC_VDM_DEFS_H__
#define __FSC_VDM_DEFS_H__

#include "vdm_types.h"

// definition/configuration object - these are all the things that the system needs to configure.
typedef struct {
	FSC_BOOL 						data_capable_as_usb_host	: 1;
	FSC_BOOL 						data_capable_as_usb_device	: 1;
	ProductType						product_type				: 3;
	FSC_BOOL						modal_operation_supported	: 1;
	FSC_U16							usb_vendor_id				: 16;
	FSC_U32							test_id						: 20; // for Cert Stat VDO, "allocated by USB-IF during certification"
	FSC_U16							usb_product_id				: 16;
	FSC_U16							bcd_device					: 16;
	FSC_U8							cable_hw_version			: 4;
	FSC_U8							cable_fw_version			: 4;
	CableToType						cable_to_type				: 2;
	CableToPr						cable_to_pr					: 1;
	CableLatency					cable_latency				: 4;
	CableTermType					cable_term					: 2;
	SsDirectionality				sstx1_dir_supp				: 1;
	SsDirectionality				sstx2_dir_supp				: 1;
	SsDirectionality				ssrx1_dir_supp				: 1;
	SsDirectionality				ssrx2_dir_supp				: 1;
	VbusCurrentHandlingCapability	vbus_current_handling_cap	: 2;
	VbusThruCable					vbus_thru_cable				: 1;
	Sop2Presence					sop2_presence				: 1;

	VConnFullPower					vconn_full_power			: 3;
	VConnRequirement				vconn_requirement			: 1;
	VBusRequirement					vbus_requirement			: 1;
	
	UsbSsSupport					usb_ss_supp					: 3;
	AmaUsbSsSupport					ama_usb_ss_supp				: 3;
	
	FSC_U32                      	num_svids;
	FSC_U16							svids[MAX_NUM_SVIDS];
	FSC_U32                      	num_modes_for_svid[MAX_NUM_SVIDS];
	
	// TODO: A lot of potential wasted memory here...
	FSC_U32							modes[MAX_NUM_SVIDS][MAX_MODES_PER_SVID];
	
} VendorDefinition;

#endif // header guard

#endif // FSC_HAVE_VDM

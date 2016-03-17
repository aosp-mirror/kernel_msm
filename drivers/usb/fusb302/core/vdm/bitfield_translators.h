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

#ifndef __VDM_BITFIELD_TRANSLATORS_H__
#define __VDM_BITFIELD_TRANSLATORS_H__

#include "../platform.h" 
				
/*
 * Functions that convert bits into internal header representations...
 */
	UnstructuredVdmHeader 	getUnstructuredVdmHeader(FSC_U32 in);	// converts 32 bits into an unstructured vdm header struct
	StructuredVdmHeader 	getStructuredVdmHeader(FSC_U32 in);		// converts 32 bits into a structured vdm header struct
	IdHeader 				getIdHeader(FSC_U32 in);					// converts 32 bits into an ID Header struct
	VdmType 				getVdmTypeOf(FSC_U32 in);				// returns structured/unstructured vdm type
	
/*
 * Functions that convert internal header representations into bits...
 */
	FSC_U32 	getBitsForUnstructuredVdmHeader(UnstructuredVdmHeader in);	// converts unstructured vdm header struct into 32 bits
	FSC_U32 	getBitsForStructuredVdmHeader(StructuredVdmHeader in);		// converts structured vdm header struct into 32 bits 
	FSC_U32 	getBitsForIdHeader(IdHeader in);							// converts ID Header struct into 32 bits 

/*
 * Functions that convert bits into internal VDO representations...
 */
	CertStatVdo 			getCertStatVdo(FSC_U32 in);
	ProductVdo 				getProductVdo(FSC_U32 in);
	CableVdo 				getCableVdo(FSC_U32 in);
	AmaVdo 					getAmaVdo(FSC_U32 in);

/*
 * Functions that convert internal VDO representations into bits...
 */	
 	FSC_U32 getBitsForProductVdo(ProductVdo in);	// converts Product VDO struct into 32 bits
	FSC_U32 getBitsForCertStatVdo(CertStatVdo in);	// converts Cert Stat VDO struct into 32 bits
	FSC_U32	getBitsForCableVdo(CableVdo in);		// converts Cable VDO struct into 32 bits
	FSC_U32	getBitsForAmaVdo(AmaVdo in);			// converts AMA VDO struct into 32 bits
	
#endif // header guard

#endif // FSC_HAVE_VDM

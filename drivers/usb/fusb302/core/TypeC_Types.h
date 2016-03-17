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
#ifndef __FSC_TYPEC_TYPES_H__
#define __FSC_TYPEC_TYPES_H__

typedef enum {
    USBTypeC_Sink = 0,
    USBTypeC_Source,
    USBTypeC_DRP,
    USBTypeC_UNDEFINED = 99
} USBTypeCPort;

typedef enum {
    Disabled = 0,
    ErrorRecovery,
    Unattached,
    AttachWaitSink,
    AttachedSink,
    AttachWaitSource,
    AttachedSource,
    TrySource,
    TryWaitSink,
    TrySink,
    TryWaitSource,
    AudioAccessory,
    DebugAccessory,
    AttachWaitAccessory,
    PoweredAccessory,
    UnsupportedAccessory,
    DelayUnattached,
    UnattachedSource
} ConnectionState;

typedef enum {
    CCTypeOpen = 0,
    CCTypeRa,
    CCTypeRdUSB,
    CCTypeRd1p5,
    CCTypeRd3p0,
    CCTypeUndefined
} CCTermType;

typedef enum {
    TypeCPin_None = 0,
    TypeCPin_GND1,
    TypeCPin_TXp1,
    TypeCPin_TXn1,
    TypeCPin_VBUS1,
    TypeCPin_CC1,
    TypeCPin_Dp1,
    TypeCPin_Dn1,
    TypeCPin_SBU1,
    TypeCPin_VBUS2,
    TypeCPin_RXn2,
    TypeCPin_RXp2,
    TypeCPin_GND2,
    TypeCPin_GND3,
    TypeCPin_TXp2,
    TypeCPin_TXn2,
    TypeCPin_VBUS3,
    TypeCPin_CC2,
    TypeCPin_Dp2,
    TypeCPin_Dn2,
    TypeCPin_SBU2,
    TypeCPin_VBUS4,
    TypeCPin_RXn1,
    TypeCPin_RXp1,
    TypeCPin_GND4
} TypeCPins_t;

typedef enum {
    utccNone = 0,
    utccDefault,
    utcc1p5A,
    utcc3p0A
} USBTypeCCurrent;

#endif // __FSC_TYPEC_TYPES_H__

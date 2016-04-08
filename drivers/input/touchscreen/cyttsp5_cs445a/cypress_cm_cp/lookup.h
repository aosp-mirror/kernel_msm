/******************************************************************************
 * @file lookup.h
 *
 * lookup.h
 *
 * @version 0.0.1
 * @authors btok
 *
 *****************************************************************************//*
 * Copyright (2014), Cypress Semiconductor Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *****************************************************************************/

#ifndef _LOOKUP_H_
#define _LOOKUP_H_

/* RX Attenuator Lookup Table */
struct rx_attenuator_lookup {
    int index;
    int value;
};

#define RX_ATTENUATOR_LOOKUP_TABLE    \
    {1, 24},             \
    {2, 12},             \
    {3, 8},             \
    {4, 6},             \
    {5, 5},             \
    {6, 4},             \
    {8, 3},             \
    {11, 2},             \
    {12, 1}

/* MTX Sum Lookup Table */
#define MTX_ORDER_MIN    1
#define MTX_ORDER_MAX    24

#define IS_MTX_ORDER_VALID(mtx_order)    \
    ((mtx_order) >= MTX_ORDER_MIN && (mtx_order) <= MTX_ORDER_MAX)

#define GET_MTX_SUM_OFFSET(mtx_order)    \
    ((mtx_order) - MTX_ORDER_MIN)

#define MTX_SUM_LOOKUP_TABLE            \
    1, 2, 2, 2, 3, 2, 1, 2, 3, 2, 3, 2,    \
    3, 4, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4

/* SelfCap Signal Swing Lookup Table */
#define REF_SCALE_MIN    0
#define REF_SCALE_MAX    7
#define RXDAC_MIN    0
#define RXDAC_MAX    15

#define VREF_LOW_INDEX    0
#define VREF_MID_INDEX    1
#define VREF_HIGH_INDEX    2

#define IS_REF_SCALE_VALID(ref_scale)    \
    ((ref_scale) >= REF_SCALE_MIN && (ref_scale) <= REF_SCALE_MAX)
#define IS_RXDAC_VALID(rxdac)    \
    ((rxdac) >= RXDAC_MIN && (rxdac) <= RXDAC_MAX)

#define GET_SELFCAP_SIGNAL_SWING_OFFSET(ref_scale, rxdac)    \
    (((ref_scale) * (RXDAC_MAX - RXDAC_MIN + 1) + (rxdac)) * 3)

#define GET_VREF_LOW_OFFSET(ref_scale, rxdac)    \
    (GET_SELFCAP_SIGNAL_SWING_OFFSET(ref_scale, rxdac) + VREF_LOW_INDEX)
#define GET_VREF_MID_OFFSET(ref_scale, rxdac)    \
    (GET_SELFCAP_SIGNAL_SWING_OFFSET(ref_scale, rxdac) + VREF_MID_INDEX)
#define GET_VREF_HIGH_OFFSET(ref_scale, rxdac)    \
    (GET_SELFCAP_SIGNAL_SWING_OFFSET(ref_scale, rxdac) + VREF_HIGH_INDEX)

#define SELFCAP_SIGNAL_SWING_LOOKUP_TABLE    \
    1, 1, 1, /* REF_SCALE = 0, RXDAC = 0 */    \
    1, 1, 1, /* REF_SCALE = 0, RXDAC = 1 */    \
    0, 1, 1, /* REF_SCALE = 0, RXDAC = 2 */    \
    0, 1, 1, /* REF_SCALE = 0, RXDAC = 3 */    \
    0, 1, 1, /* REF_SCALE = 0, RXDAC = 4 */    \
    0, 1, 1, /* REF_SCALE = 0, RXDAC = 5 */    \
    0, 1, 1, /* REF_SCALE = 0, RXDAC = 6 */    \
    0, 1, 1, /* REF_SCALE = 0, RXDAC = 7 */    \
    0, 1, 1, /* REF_SCALE = 0, RXDAC = 8 */    \
    0, 1, 2, /* REF_SCALE = 0, RXDAC = 9 */    \
    0, 1, 2, /* REF_SCALE = 0, RXDAC = 10 */    \
    0, 1, 2, /* REF_SCALE = 0, RXDAC = 11 */    \
    0, 1, 2, /* REF_SCALE = 0, RXDAC = 12 */    \
    0, 1, 2, /* REF_SCALE = 0, RXDAC = 13 */    \
    0, 1, 2, /* REF_SCALE = 0, RXDAC = 14 */    \
    0, 1, 2, /* REF_SCALE = 0, RXDAC = 15 */    \
    1, 1, 1, /* REF_SCALE = 1, RXDAC = 0 */    \
    1, 1, 1, /* REF_SCALE = 1, RXDAC = 1 */    \
    1, 1, 1, /* REF_SCALE = 1, RXDAC = 2 */    \
    1, 1, 1, /* REF_SCALE = 1, RXDAC = 3 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 4 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 5 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 6 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 7 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 8 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 9 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 10 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 11 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 12 */    \
    1, 1, 2, /* REF_SCALE = 1, RXDAC = 13 */    \
    1, 1, 3, /* REF_SCALE = 1, RXDAC = 14 */    \
    0, 1, 3, /* REF_SCALE = 1, RXDAC = 15 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 0 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 1 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 2 */    \
    1, 1, 2, /* REF_SCALE = 2, RXDAC = 3 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 4 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 5 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 6 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 7 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 8 */    \
    1, 2, 2, /* REF_SCALE = 2, RXDAC = 9 */    \
    1, 2, 3, /* REF_SCALE = 2, RXDAC = 10 */    \
    1, 2, 3, /* REF_SCALE = 2, RXDAC = 11 */    \
    1, 2, 3, /* REF_SCALE = 2, RXDAC = 12 */    \
    0, 2, 3, /* REF_SCALE = 2, RXDAC = 13 */    \
    0, 2, 3, /* REF_SCALE = 2, RXDAC = 14 */    \
    0, 2, 3, /* REF_SCALE = 2, RXDAC = 15 */    \
    1, 2, 2, /* REF_SCALE = 3, RXDAC = 0 */    \
    1, 2, 2, /* REF_SCALE = 3, RXDAC = 1 */    \
    1, 2, 2, /* REF_SCALE = 3, RXDAC = 2 */    \
    1, 2, 2, /* REF_SCALE = 3, RXDAC = 3 */    \
    1, 2, 2, /* REF_SCALE = 3, RXDAC = 4 */    \
    1, 2, 2, /* REF_SCALE = 3, RXDAC = 5 */    \
    1, 2, 2, /* REF_SCALE = 3, RXDAC = 6 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 7 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 8 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 9 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 10 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 11 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 12 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 13 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 14 */    \
    1, 2, 3, /* REF_SCALE = 3, RXDAC = 15 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 0 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 1 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 2 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 3 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 4 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 5 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 6 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 7 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 8 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 9 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 10 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 11 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 12 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 13 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 14 */    \
    1, 2, 3, /* REF_SCALE = 4, RXDAC = 15 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 0 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 1 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 2 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 3 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 4 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 5 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 6 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 7 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 8 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 9 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 10 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 11 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 12 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 13 */    \
    1, 2, 3, /* REF_SCALE = 5, RXDAC = 14 */    \
    1, 2, 4, /* REF_SCALE = 5, RXDAC = 15 */    \
    1, 2, 3, /* REF_SCALE = 6, RXDAC = 0 */    \
    2, 2, 3, /* REF_SCALE = 6, RXDAC = 1 */    \
    2, 2, 3, /* REF_SCALE = 6, RXDAC = 2 */    \
    2, 2, 3, /* REF_SCALE = 6, RXDAC = 3 */    \
    2, 2, 3, /* REF_SCALE = 6, RXDAC = 4 */    \
    2, 2, 3, /* REF_SCALE = 6, RXDAC = 5 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 6 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 7 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 8 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 9 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 10 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 11 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 12 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 13 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 14 */    \
    1, 2, 4, /* REF_SCALE = 6, RXDAC = 15 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 0 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 1 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 2 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 3 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 4 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 5 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 6 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 7 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 8 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 9 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 10 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 11 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 12 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 13 */    \
    1, 2, 4, /* REF_SCALE = 7, RXDAC = 14 */    \
    1, 3, 5, /* REF_SCALE = 7, RXDAC = 15 */
#endif /* _LOOKUP_H_ */

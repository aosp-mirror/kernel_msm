/******************************************************************************
 * @file debug.h
 *
 * debug.h
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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef DEBUG
#include <stdio.h>
#define DEBUG_PRINTF(...)	\
    do { fprintf(stderr,  __VA_ARGS__); } while(0)
#else
#define DEBUG_PRINTF(...)
#endif

#ifdef VDEBUG
#include <stdio.h>
#define VDEBUG_PRINTF(...)	\
    do { fprintf(stderr,  __VA_ARGS__); } while(0)
#else
#define VDEBUG_PRINTF(...)
#endif
#endif /* _DEBUG_H_ */

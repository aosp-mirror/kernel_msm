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
 * Creates a circular queue to track state transitions and times
 * Log does not overwrite if full - entries are dropped
 */
#ifdef FSC_DEBUG

#ifndef FSC_LOG_H
#define	FSC_LOG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "platform.h"
    
#define LOG_SIZE 64

typedef struct{
    FSC_U16 state;
    FSC_U16 time_ms;
    FSC_U16 time_s;
} StateLogEntry;

typedef struct{
    StateLogEntry logQueue[ LOG_SIZE ];
    FSC_U8 Start;
    FSC_U8 End;
    FSC_U8 Count;
} StateLog;

void InitializeStateLog(StateLog *log);
FSC_BOOL WriteStateLog(StateLog *log, FSC_U16 state, FSC_U16 time_ms, FSC_U16 time_s);
FSC_BOOL ReadStateLog(StateLog *log, FSC_U16 * state, FSC_U16 * time_ms, FSC_U16 * time_s);
FSC_BOOL IsStateLogFull(StateLog *log);
FSC_BOOL IsStateLogEmpty(StateLog *log);
void DeleteStateLog(StateLog *log);

#ifdef	__cplusplus
}
#endif // __cplusplus

#endif	/* FSC_LOG_H */

#endif // FSC_DEBUG

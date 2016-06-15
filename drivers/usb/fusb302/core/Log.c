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
#ifdef FSC_DEBUG

#include "Log.h"
#include <linux/printk.h>

void InitializeStateLog(StateLog *log)
{
    log->Count = 0;
    log->End = 0;
    log->Start = 0;
}

extern StateLog TypeCStateLog;
extern StateLog PDStateLog;
FSC_BOOL WriteStateLog(StateLog *log, FSC_U16 state, FSC_U16 time_ms, FSC_U16 time_s)
{
    if (log == &TypeCStateLog)
        pr_debug("FUSB TYPEC state log, state: 0x%x\n", state);
	else if (log == &PDStateLog)
		pr_debug("FUSB PDPolicy state log, state: 0x%x\n", state);
	else
		pr_debug("not match TYPEC & PDPolicy\n");

    if(!IsStateLogFull(log))
    {
        FSC_U8 index = log->End;
        log->logQueue[index].state = state;
        log->logQueue[index].time_ms = time_ms;
        log->logQueue[index].time_s = time_s;

        log->End += 1;
        if(log->End == LOG_SIZE)
        {
            log->End = 0;
        }

        log->Count += 1;

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

FSC_BOOL ReadStateLog(StateLog *log, FSC_U16 * state, FSC_U16 * time_ms, FSC_U16 * time_s) // Read first log and delete entry
{
    if(!IsStateLogEmpty(log))
    {
        FSC_U8 index = log->Start;
        *state = log->logQueue[index].state;
        *time_ms = log->logQueue[index].time_ms;
        *time_s = log->logQueue[index].time_s;

        log->Start += 1;
        if(log->Start == LOG_SIZE)
        {
            log->Start = 0;
        }

        log->Count -= 1;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

FSC_BOOL IsStateLogFull(StateLog *log)
{
    return (log->Count == LOG_SIZE) ? TRUE : FALSE;
}

FSC_BOOL IsStateLogEmpty(StateLog *log)
{
    return (!log->Count) ? TRUE : FALSE;
}

void DeleteStateLog(StateLog *log)
{
}

#endif // FSC_DEBUG


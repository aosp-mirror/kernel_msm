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

#ifndef FUSB30X_TYPES_H
#define FUSB30X_TYPES_H

#include <linux/i2c.h>                              // i2c_client, spinlock_t
#include <linux/hrtimer.h>                          // hrtimer
#include <linux/wakelock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/usb/usb_controller.h>
#include <linux/usb/class-dual-role.h>
#include <linux/usb/usb_typec.h>
#include "FSCTypes.h"                               // FUSB30x custom types

#ifdef FSC_DEBUG
#define FSC_HOSTCOMM_BUFFER_SIZE    64              // Length of the hostcomm buffer
#endif // FSC_DEBUG

struct fusb30x_chip                                 // Contains data required by this driver
{
    struct mutex lock;                              // Synchronization lock
    struct mutex statemachine_lock;

#ifdef FSC_DEBUG
    FSC_U8 dbgTimerTicks;                           // Count of timer ticks
    FSC_U8 dbgTimerRollovers;                       // Timer tick counter rollover counter
    FSC_U8 dbgSMTicks;                              // Count of state machine ticks
    FSC_U8 dbgSMRollovers;                          // State machine tick counter rollover counter
    FSC_S32 dbg_gpio_StateMachine;                  // Gpio that toggles every time the state machine is triggered
    FSC_BOOL dbg_gpio_StateMachine_value;           // Value of sm toggle state machine
    char HostCommBuf[FSC_HOSTCOMM_BUFFER_SIZE];     // Buffer used to communicate with HostComm
#endif // FSC_DEBUG

    /* Internal config data */
    FSC_S32 InitDelayMS;                            // Number of milliseconds to wait before initializing the fusb30x
    FSC_S32 numRetriesI2C;                          // Number of times to retry I2C reads/writes

    /* I2C */
    struct i2c_client* client;                      // I2C client provided by kernel
    FSC_BOOL use_i2c_blocks;                        // True if I2C_FUNC_SMBUS_I2C_BLOCK is supported

    struct pinctrl *fusb302_pinctrl;

    struct regulator *boost_5v;

    /* GPIO */
    FSC_S32 gpio_VBus5V;                            // VBus 5V GPIO pin
    FSC_BOOL gpio_VBus5V_value;                     // true if active, false otherwise
    FSC_S32 gpio_VBusOther;                         // VBus other GPIO pin (eg. VBus 12V) (NOTE: Optional feature - if set to <0 during GPIO init, then feature is disabled)
    FSC_BOOL gpio_VBusOther_value;                  // true if active, false otherwise
    FSC_S32 gpio_IntN;                              // INT_N GPIO pin

#ifdef FSC_INTERRUPT_TRIGGERED
    FSC_S32 gpio_IntN_irq;                          // IRQ assigned to INT_N GPIO pin
#endif  // FSC_INTERRUPT_TRIGGERED
    
    /* Threads */
    struct delayed_work init_worker;                // Kicks off our runtime worker
    struct work_struct worker;                      // Main state machine actions

    /* Timers */
    struct hrtimer timer_state_machine;             // High-resolution timer for the state machine

    atomic_t pm_suspended;
    struct wake_lock fusb_wlock;

    struct usb_controller *uc;
	struct usb_typec_ctrl *utc;

    struct dual_role_phy_instance *fusb_instance;
    enum dual_role_prop_mode pmode;
    enum dual_role_prop_power_role prole;
    enum dual_role_prop_data_role drole;
    enum dual_role_prop_vconn_supply vconn;
};

extern struct fusb30x_chip* g_chip;

struct fusb30x_chip* fusb30x_GetChip(void);         // Getter for the global chip structure
void fusb30x_SetChip(struct fusb30x_chip* newChip); // Setter for the global chip structure

#endif /* FUSB30X_TYPES_H */

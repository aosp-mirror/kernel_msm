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
#include <linux/printk.h>                                                       // pr_err, printk, etc
#include <linux/delay.h>
#include "../core/PD_Types.h"
#include "fusb30x_global.h"                                                     // Chip structure
#include "platform_helpers.h"                                                   // Implementation details
#include "../core/platform.h"
#include "../core/TypeC_Types.h"

/*******************************************************************************
* Function:        platform_set/get_vbus_lvl_enable
* Input:           VBUS_LVL - requested voltage
*                  Boolean - enable this voltage level
*                  Boolean - turn off other supported voltages
* Return:          Boolean - on or off
* Description:     Provide access to the VBUS control pins.
******************************************************************************/
void platform_set_vbus_lvl_enable(VBUS_LVL level, FSC_BOOL blnEnable, FSC_BOOL blnDisableOthers)
{
    FSC_U32 i;

    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Enable/Disable the 5V Source
        fusb_GPIO_Set_VBus5v(blnEnable == TRUE ? true : false);
        break;
    case VBUS_LVL_12V:
        // Enable/Disable the 12V Source
        fusb_GPIO_Set_VBusOther(blnEnable == TRUE ? true : false);
        break;
    default:
        // Otherwise, do nothing.
        break;
    }

    // Turn off other levels, if requested
    if (blnDisableOthers || ((level == VBUS_LVL_ALL) && (blnEnable == FALSE)))
    {
        i = 0;

        do {
            // Skip the current level
            if( i == level ) continue;

            // Turn off the other level(s)
            platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_COUNT);
    }

    return;
}

FSC_BOOL platform_get_vbus_lvl_enable(VBUS_LVL level)
{
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Return the state of the 5V VBUS Source.
        return fusb_GPIO_Get_VBus5v() ? TRUE : FALSE;

    case VBUS_LVL_12V:
        // Return the state of the 12V VBUS Source.
        return fusb_GPIO_Get_VBusOther() ? TRUE : FALSE;

    default:
        // Otherwise, return FALSE.
        return FALSE;
    }
}

extern FSC_BOOL VCONN_enabled;
/*******************************************************************************
 * Function:        platform_set_vconn_enable
 * Input:           blnEnable - enable or disable VCONN
 * Return:          Boolean - State of VCONN GPIO
 * Description:     Provide access to the VCONN control pin(s).
 ******************************************************************************/
FSC_BOOL platform_set_vconn_enable(FSC_BOOL blnEnable)
{
    struct fusb30x_chip *chip = fusb30x_GetChip();
    FSC_BOOL ret;

    ret = fusb_Power_Vconn(blnEnable);

    if (ret && VCONN_enabled != blnEnable) {
        chip->vconn = blnEnable ? DUAL_ROLE_PROP_VCONN_SUPPLY_YES : DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
        if (chip->fusb_instance)
            dual_role_instance_changed(chip->fusb_instance);
    }

    return ret;
}

/*******************************************************************************
* Function:        platform_set_vbus_discharge
* Input:           Boolean
* Return:          None
* Description:     Enable/Disable Vbus Discharge Path
******************************************************************************/
void platform_set_vbus_discharge(FSC_BOOL blnEnable)
{
    // TODO - Implement if required for platform
}

/*******************************************************************************
* Function:        platform_get_device_irq_state
* Input:           None
* Return:          Boolean.  TRUE = Interrupt Active
* Description:     Get the state of the INT_N pin.  INT_N is active low.  This
*                  function handles that by returning TRUE if the pin is
*                  pulled low indicating an active interrupt signal.
******************************************************************************/
FSC_BOOL platform_get_device_irq_state(void)
{
    return fusb_GPIO_Get_IntN() ? TRUE : FALSE;
}

/*******************************************************************************
* Function:        platform_i2c_write
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to transmit
*                  PacketSize - Maximum size of each transmitted packet
*                  IncSize - Number of bytes to send before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer of char data to transmit
* Return:          Error state
* Description:     Write a char buffer to the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_write(FSC_U8 SlaveAddress,
                        FSC_U8 RegAddrLength,
                        FSC_U8 DataLength,
                        FSC_U8 PacketSize,
                        FSC_U8 IncSize,
                        FSC_U32 RegisterAddress,
                        FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    if (Data == NULL)
    {
        pr_err("%s - Error: Write data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (fusb_I2C_WriteData((FSC_U8)RegisterAddress, DataLength, Data))
    {
        ret = TRUE;
    }
    else  // I2C Write failure
    {
        ret = FALSE;       // Write data block to the device
    }
    return ret;
}

/*******************************************************************************
* Function:        platform_i2c_read
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to attempt to read
*                  PacketSize - Maximum size of each received packet
*                  IncSize - Number of bytes to recv before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer for received char data
* Return:          Error state.
* Description:     Read char data from the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_read(FSC_U8 SlaveAddress,
                       FSC_U8 RegAddrLength,
                       FSC_U8 DataLength,
                       FSC_U8 PacketSize,
                       FSC_U8 IncSize,
                       FSC_U32 RegisterAddress,
                       FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    FSC_S32 i = 0;
    FSC_U8 temp = 0;
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        pr_err("FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        return FALSE;
    }

    if (Data == NULL)
    {
        pr_err("%s - Error: Read data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (DataLength > 1 && chip->use_i2c_blocks)    // Do block reads if able and necessary
    {
        if (!fusb_I2C_ReadBlockData(RegisterAddress, DataLength, Data))
        {
            ret = FALSE;
        }
        else
        {
            ret = TRUE;
        }
    }
    else
    {
        for (i = 0; i < DataLength; i++)
        {
            if (fusb_I2C_ReadData((FSC_U8)RegisterAddress + i, &temp))
            {
                Data[i] = temp;
                ret = TRUE;
            }
            else
            {
                ret = FALSE;
                break;
            }
        }
    }

    return ret;
}

/*****************************************************************************
* Function:        platform_enable_timer
* Input:           enable - TRUE to enable platform timer, FALSE to disable
* Return:          None
* Description:     Enables or disables platform timer
******************************************************************************/
void platform_enable_timer(FSC_BOOL enable)
{
    if (enable == TRUE)
    {
        fusb_StartTimers();
    }
    else
    {
        fusb_StopTimers();
    }
}

/*****************************************************************************
* Function:        platform_delay_10us
* Input:           delayCount - Number of 10us delays to wait
* Return:          None
* Description:     Perform a software delay in intervals of 10us.
******************************************************************************/
void platform_delay_10us(FSC_U32 delayCount)
{
    fusb_Delay10us(delayCount);
}

extern FSC_BOOL IsPRSwap;
extern CCTermType CC1TermCCDebounce;
extern CCTermType CC2TermCCDebounce;
/*******************************************************************************
* Function:        platform_notify_cc_orientation
* Input:           orientation - Orientation of CC (NONE, CC1, CC2)
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current CC orientation. Called in SetStateAttached... and
*                  SetStateUnattached functions.
******************************************************************************/
void platform_notify_cc_orientation(CC_ORIENTATION orientation)
{
	struct pinctrl_state *set_state;
	struct fusb30x_chip* chip = fusb30x_GetChip();

	// Optional: Notify platform of CC orientation
	pr_info("FUSB %s: orientation=[%d], CC1=[%d], CC2=[%d]\n", __func__, orientation, CC1TermCCDebounce, CC2TermCCDebounce);
	if (orientation == CC1) {
		if (chip->fusb302_pinctrl) {
			set_state = pinctrl_lookup_state(chip->fusb302_pinctrl, "usb3_switch_sel_0");
			if (IS_ERR(set_state)) {
				pr_err("cannot get fusb302 pinctrl usb3_switch_sel_0 state");
				return;
			}

			pinctrl_select_state(chip->fusb302_pinctrl, set_state);
		}
	} else if (orientation == CC2) {
		if (chip->fusb302_pinctrl) {
			set_state = pinctrl_lookup_state(chip->fusb302_pinctrl, "usb3_switch_sel_1");
			if (IS_ERR(set_state)) {
				pr_err("cannot get fusb302 pinctrl usb3_switch_sel_1 state");
				return;
			}

			pinctrl_select_state(chip->fusb302_pinctrl, set_state);
		}
	} else if (orientation == NONE) {
		if (chip && chip->uc && chip->uc->pd_vbus_ctrl) {
			chip->uc->pd_vbus_ctrl(-1, FALSE);
			if (!IsPRSwap)
				platform_notify_attached_source(0);
		}
	}
}

extern ConnectionState         ConnState;          // Variable indicating the current connection state
extern PolicyState_t           PolicyState;
/*******************************************************************************
* Function:        platform_notify_pd_contract
* Input:           contract - TRUE: Contract, FALSE: No Contract
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current PD contract status. Called in PDPolicy.
*******************************************************************************/
void platform_notify_pd_contract(FSC_BOOL contract)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();

    // Optional: Notify platform of PD contract
	pr_info("FUSB %s: Contract=[%d], typec_state(0x%x), pd_state(0x%x)\n", __func__, contract, ConnState, PolicyState);
	if (contract) {
		if (ConnState == AttachedSink) {
			if (chip && chip->uc && chip->uc->pd_vbus_ctrl) {
				chip->uc->pd_vbus_ctrl(0, FALSE);
			}
		} else if (ConnState == AttachedSource) {
			if (chip && chip->uc && chip->uc->pd_vbus_ctrl) {
				chip->uc->pd_vbus_ctrl(1, FALSE);
			}
		}
	}
}

/*******************************************************************************
* Function:        platform_notify_unsupported_accessory
* Input:           None
* Return:          None
* Description:     A callback used by the core to report entry to the
*                  Unsupported Accessory state. The platform may implement
*                  USB Billboard.
*******************************************************************************/
void platform_notify_unsupported_accessory(void)
{
    // Optional: Implement USB Billboard
    printk(KERN_INFO "FUSB %s: invoked\n", __func__);
}

extern FSC_BOOL PolicyIsDFP;
void platform_notify_attached_source(int value)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	int notify_retry_count = 0;

	pr_info("FUSB %s: value(%d), typec_state(0x%x), pd_state(0x%x)\n", __func__, value, ConnState, PolicyState);

	do {
		if (chip != NULL && chip->uc != NULL && chip->uc->notify_attached_source != NULL) {
			chip->uc->notify_attached_source(chip->uc, value);
			break;
		} else {
			printk(KERN_INFO "FUSB %s: structure null, retry_count = %d\n", __func__, notify_retry_count);
			notify_retry_count++;
			msleep(5000);
		}
	} while (notify_retry_count <= 3);

	PolicyIsDFP = value ? TRUE : FALSE;
	chip->pmode = value ? DUAL_ROLE_PROP_MODE_DFP : DUAL_ROLE_PROP_MODE_UFP;
	chip->drole = value ? DUAL_ROLE_PROP_DR_HOST : DUAL_ROLE_PROP_DR_DEVICE;
	if (chip->fusb_instance)
		dual_role_instance_changed(chip->fusb_instance);
}

u8 platform_select_source_capability(u8 obj_cnt, doDataObject_t pd_data[7], int *device_max_ma)
{
    return fusb_battery_select_source_capability(obj_cnt, pd_data, device_max_ma);
}

int usb_controller_register(struct device *parent, struct usb_controller *uc)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL)
		return -ENODEV;

	chip->uc = uc;
	return 0;
}
EXPORT_SYMBOL_GPL(usb_controller_register);

int usb_typec_ctrl_register(struct device *parent, struct usb_typec_ctrl *utc)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL)
		return -ENODEV;

	chip->utc = utc;
	return 0;
}
EXPORT_SYMBOL_GPL(usb_typec_ctrl_register);

/* Copyright (c) 2016,  HUAWEI TECHNOLOGIES CO., LTD.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "idtp9220-receiver.h"

static int __idtp9220_read(struct idtp9220_receiver *chip, u16 reg,
                u8 *val)
{
    struct i2c_client *client = chip->client;
    struct i2c_msg msg[2];
    int ret = 0;
    u16 reg_be = cpu_to_be16(reg);

    memset(msg, 0, sizeof(msg));

    if (!client->adapter)
    {
        return -ENODEV;
    }

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = sizeof(reg_be);
    msg[0].buf = (u8 *)&reg_be;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = IDTP9220_I2C_READ_BYTES;
    msg[1].buf = val;

    ret = i2c_transfer(client->adapter, msg, 2);
    if(2 == ret)
    {
        ret = 0;
    }
    else
    {
        ret = (ret < 0) ? ret : -EIO;
    }

    return ret;
}

static int __idtp9220_write(struct idtp9220_receiver *chip, u16 reg, u8 val)
{
    struct i2c_client *client = chip->client;
    struct i2c_msg msg[1];
    u8 data[3];
    int ret;

    memset(msg, 0, sizeof(msg));

    data[0] = reg >> 8;
    data[1] = reg & 0xff;
    data[2] = val;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = ARRAY_SIZE(data);
    msg[0].buf = data;

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if(1 == ret)
    {
        ret = 0;
    }
    else
    {
        ret = (ret < 0) ? ret : -EIO;
    }

    return ret;
}

static int idtp9220_read(struct idtp9220_receiver *chip, u16 reg,
                u8 *val)
{
    int rc;

    mutex_lock(&chip->read_write_lock);
    rc = __idtp9220_read(chip, reg, val);
    mutex_unlock(&chip->read_write_lock);

    return rc;
}

static int idtp9220_write(struct idtp9220_receiver *chip, u16 reg,
                        u8 val)
{
    int rc;

    mutex_lock(&chip->read_write_lock);
    rc = __idtp9220_write(chip, reg, val);
    mutex_unlock(&chip->read_write_lock);

    return rc;
}

static int idtp9220_masked_write(struct idtp9220_receiver *chip, u16 reg,
                        u8 mask, u8 val)
{
    int rc;
    u8 temp;

    mutex_lock(&chip->read_write_lock);
    rc = __idtp9220_read(chip, reg, &temp);
    if (rc < 0)
    {
        goto out;
    }

    temp &= ~mask;
    temp |= val & mask;
    rc = __idtp9220_write(chip, reg, temp);

out:
    mutex_unlock(&chip->read_write_lock);
    return rc;
}

static int idtp9220_Xfer9220i2c(struct idtp9220_receiver *chip, u16 reg, char *bBuf, int bOffs, int bSize)
{

    int i, rc;

    for (i = 0; i < bSize; i++)
    {
        rc = idtp9220_write(chip, reg + i, bBuf[bOffs + i]);
        if(rc < 0)
        {
            return rc;
        }
    }

    return 0;
}

static int idtp9220_set_ldout_enable(struct idtp9220_receiver *chip, bool enable)
{
    int rc, i;

    i = enable << TOGGLE_LDO_ON_OFF_MASK_SHIFT;
    rc = idtp9220_masked_write(chip, COMMAND_REG, TOGGLE_LDO_ON_OFF_MASK, i);
    if (rc < 0)
    {
        pr_err("Couldn't set reg_command enable = %d rc = %d\n", enable, rc);
        return rc;
    }

    return 0;
}

static int idtp9220_set_vout_voltage(struct idtp9220_receiver *chip, u16 voltage)
{
    int rc,i;

    if(chip->using_default_vout_flag)
    {
        voltage = IDT9200_VOUT_MAX_MV;
    }

    if ((voltage < IDT9200_VOUT_MIN_MV) ||
        (voltage > IDT9200_VOUT_MAX_MV))
    {
        pr_err( "bad vout voltage %d asked to set\n",
                            voltage);
        return -EINVAL;
    }

    i = (voltage - IDT9200_VOUT_MIN_MV) / IDT9200_VOUT_STEP_MV;

    rc = idtp9220_write(chip, VOUT_SET_REG, i);
    if (rc < 0)
    {
        pr_err("cannot set vout to %dmv rc = %d\n", voltage, rc);
    }

    return rc;
}

static int idtp9220_get_vout_voltage(struct idtp9220_receiver *chip, union idtp9220_interactive_data *vout)
{
    int rc;

    rc = idtp9220_read(chip, ADC_VOUT_L_REG, &vout->strval[0]);
    if(rc < 0)
    {
        return rc;
    }

    rc = idtp9220_read(chip, ADC_VOUT_H_REG, &vout->strval[1]);
    if(rc < 0)
    {
        return rc;
    }

    vout->strval[1] = vout->strval[1] & 0xf;

    vout->shortval = vout->shortval * 6 * 21 * 1000 / 40950 + ADJUST_METE_MV; //vout = val/4095*6*2.1

    pr_info("vout_l:%02x vout_h:%02x\n", vout->strval[0], vout->strval[1]);

    return 0;
}

void idtp9220_ap_mask_rxint_enable(bool enable)
{
    if(global_idtp9220_receiver && global_idtp9220_receiver->mask_wireless_int_gpio)
    {
        pr_err("set sleep gpio to %d\n", enable);
        gpio_set_value(global_idtp9220_receiver->mask_wireless_int_gpio, enable);
    }
}

static int idtp9220_clear_tx_data_receiv_intr(struct idtp9220_receiver *chip)
{
    int rc, i;

    /* clear the corresponding Interrupt Registers bit*/
    i = 1 << TX_DATA_RECV_CLEAR_MASK_SHIFT;
    rc = idtp9220_masked_write(chip, INT_CLEAR_L_REG, TX_DATA_RECV_CLEAR_MASK, i);
    if (rc < 0)
    {
        pr_err("set clear tx_data_recceive interrupt bit fail, rc = %d\n", rc);
        return rc;
    }

    /* run clear interrupt command */
    i = 1 << CLEAR_INTERRUPT_MASK_SHIFT;
    rc = idtp9220_masked_write(chip, COMMAND_REG, CLEAR_INTERRUPT_MASK, i);
    if (rc < 0)
    {
        pr_err("run clear tx_data_recceive interrupt bit fail, rc =%d\n", rc);
    }

    return rc;
}

static int idtp9220_rx_communicate_with_tx(struct idtp9220_receiver *chip,
                       enum idtp9220_request_type_need_tx command_type,
                       union idtp9220_interactive_data *val)
{
    int rc;
    u8 reg;

    /* set data commond */
    rc = idtp9220_write(chip, RX_DATA_COMMAND_REG, command_type);
    if (rc < 0)
    {
        return rc;
    }

    /* M0 to sends aata command and value to TX*/
    rc = idtp9220_masked_write(chip, COMMAND_REG, SEND_RX_DATA_MASK,
            1 << SEND_RX_DATA_MASK_SHIFT);
    if (rc < 0)
    {
        return rc;
    }

    /* wait Tx return result  */
    if (down_timeout(&chip->tx_send_data_int, msecs_to_jiffies(IDTP9220_DELAY_MS_MAX)))
    {
        pr_err("wait tx version data intr timeout\n");
        rc = WAIT_TX_RETURN_DATA_TIMEOUT_ERR;
        goto clerar_interrupt;
    }

    /* get Tx return result */
    rc = idtp9220_read(chip, TX_DATA_COMMAND_REG, &reg);
    if(rc < 0)
    {
        goto clerar_interrupt;
    }

    /* process return data */
    if(reg != command_type)
    {
        pr_err("return tx data type is error, reg = %d, data_type = %d\n", reg, command_type);
        rc = TX_RETURN_DATA_IS_NOT_DEMAND_ERR;
        goto clerar_interrupt;
    }

    if(IS_TX_HARDWARE_VERSION_DATA == reg)
    {
        /* read hardware version*/
        rc = idtp9220_read(chip, TX_DATA_01_REG, &val->strval[0]);
        if(rc < 0)
        {
            goto clerar_interrupt;
        }

        val->strval[1] = 0;

    }
    else if(IS_TX_SOFTWARE_VERSIION_DATA == reg)
    {
        /* read tx software version0*/
        rc = idtp9220_read(chip, TX_DATA_01_REG, &(val[0].strval[0]));
        if(rc < 0)
        {
              goto clerar_interrupt;
        }

        /* read tx software version1*/
        rc = idtp9220_read(chip, TX_DATA_02_REG, &(val[0].strval[1]));
        if(rc < 0)
        {
             goto clerar_interrupt;
        }

        /* read tx software version2*/
        rc = idtp9220_read(chip, TX_DATA_03_REG, &(val[1].strval[0]));
        if(rc < 0)
        {
             goto clerar_interrupt;
        }

        /* read tx software version3*/
        rc = idtp9220_read(chip, TX_DATA_04_REG, &(val[1].strval[1]));
        if(rc < 0)
        {
             goto clerar_interrupt;
        }
    }
    else if((IS_TX_TEMP_DATA == reg) || (IS_TX_VIN == reg))
    {
        /* read tx NTC1 temp or freq_l*/
        rc = idtp9220_read(chip, TX_DATA_01_REG, &val->strval[0]);
        if(rc < 0)
        {
            goto clerar_interrupt;
        }

        /* read NTC2 temp or freq_h*/
        rc = idtp9220_read(chip, TX_DATA_02_REG, &val->strval[1]);
        if(rc < 0)
        {
            goto clerar_interrupt;
        }
    }
    else
    {
    }

clerar_interrupt:
    /* clear interrupt, do not care return value */
    idtp9220_clear_tx_data_receiv_intr(chip);

    return rc;
}

static int idtp9220_get_tx_hard_version(struct idtp9220_receiver *chip, union idtp9220_interactive_data *value)
{
    int rc;

    pm_stay_awake(chip->dev);
    rc = idtp9220_rx_communicate_with_tx(chip, GET_TX_HARDWARE_VERSION_COMMAND, value);
    pm_relax(chip->dev);

    return rc;
}

static int idtp9220_get_tx_soft_version(struct idtp9220_receiver *chip, union idtp9220_interactive_data *value)
{
    int rc;

    pm_stay_awake(chip->dev);
    rc = idtp9220_rx_communicate_with_tx(chip, GET_TX_SOFTWARE_VERSION_COMMAND, value);
    pm_relax(chip->dev);

    return rc;
}

static int idtp9220_get_tx_vin(struct idtp9220_receiver *chip, union idtp9220_interactive_data *value)
{
    int rc;

    pm_stay_awake(chip->dev);
    rc = idtp9220_rx_communicate_with_tx(chip, GET_TX_VIN_COMMAND, value);
    pm_relax(chip->dev);

    return rc;
}


static int idtp9220_get_tx_temp(struct idtp9220_receiver *chip, union idtp9220_interactive_data *temp)
{
    int rc;

    pm_stay_awake(chip->dev);
    rc = idtp9220_rx_communicate_with_tx(chip, GET_TX_TEMP_COMMAND, temp);
    pm_relax(chip->dev);

    return rc;
}

static int idtp9220_get_freq(struct idtp9220_receiver *chip, union idtp9220_interactive_data *freq)
{
    int rc;

    /* read freq_l */
    rc = idtp9220_read(chip, OP_FREQ_L_REG, &freq->strval[0]);
    if(rc < 0)
    {
        return rc;
    }

    /* read freq_h */
    rc = idtp9220_read(chip, OP_FREQ_H_REG, &freq->strval[1]);
    if(rc < 0)
    {
        return rc;
    }

    return 0;
}

static int idtp9220_set_freq(struct idtp9220_receiver *chip, union idtp9220_interactive_data *freq)
{
    int rc;

    if ((freq->shortval < IDT9200_FREQ_MIN_KHZ) ||
        (freq->shortval > IDT9200_FREQ_MAX_KHZ))
    {
        pr_err( "bad freq %d asked to set\n", freq->shortval);
        return -EINVAL;
    }

    /* set  freq command */
    rc = idtp9220_write(chip, RX_DATA_COMMAND_REG, SET_TX_OPER_FREQ);
    if (rc < 0)
    {
        pr_err("cannot set freq command\n");
        return rc;
    }

    /* set freq value */
    rc = idtp9220_write(chip, RX_FREQ_DATA_L_REG, freq->strval[0]);
    if (rc < 0)
    {
        pr_err("cannot set freq_l to %dKHZ rc = %d\n", freq->strval[0], rc);
        return rc;
    }

    rc = idtp9220_write(chip, RX_FREQ_DATA_H_REG, freq->strval[1]);
    if (rc < 0)
    {
        pr_err("cannot set freq_l to %dKHZ rc = %d\n", freq->strval[1], rc);
        return rc;
    }

    /* M0 to sends data command and value to TX*/
    rc = idtp9220_masked_write(chip, COMMAND_REG, SEND_RX_DATA_MASK,
               1 << SEND_RX_DATA_MASK_SHIFT);
    if (rc < 0)
    {
        pr_err("cannot sends data command and value to TX, rc = %d\n", rc);
        return rc;
    }

    return 0;
}

static int idtp9220_set_cout_current(struct idtp9220_receiver *chip, u16 current_ma)
{
    int rc,i;

    if ((current_ma < IDT9200_COUT_MIN_MA) ||
        (current_ma > IDT9200_COUT_MAX_MA))
    {
        pr_err( "bad cout current %d asked to set\n", current_ma);
        return -EINVAL;
    }

    i = (current_ma - IDT9200_COUT_MIN_MA) / IDT9200_COUT_STEP_MA;

    rc = idtp9220_write(chip, VOUT_SET_REG, i);
    if (rc < 0)
    {
        pr_err("cannot set cout to %dma rc = %d\n", current_ma, rc);
    }

    return rc;
}

static int idtp9220_get_cout_current(struct idtp9220_receiver *chip, union idtp9220_interactive_data *cout)
{
    int rc;

    rc = idtp9220_read(chip, RX_LOUT_L_REG, &cout->strval[0]);
    if(rc < 0)
    {
        return rc;
    }

    rc = idtp9220_read(chip, RX_LOUT_H_REG, &cout->strval[1]);
    if(rc < 0)
    {
        return rc;
    }

    return 0;
}


static bool idtp9220_detect_tx_data_receive(struct idtp9220_receiver *chip)
{
    int rc;
    u8 reg_status;
    rc = idtp9220_read(chip, STATUS_L_REG, &reg_status);
    if (rc < 0)
    {
         pr_err("Unable to read system status reg rc = %d\n", rc);
         return false;
    }

    pr_err("reg status:%02x\n", reg_status);

    return reg_status & STATUS_TX_DATA_RECV;
}

static int idtp9220_is_ldoout_enable(struct idtp9220_receiver *chip, bool *ldoout_enable)
{
    int rc;
    u8 reg;

    rc = idtp9220_read(chip, STATUS_L_REG, &reg);
    if(rc < 0)
    {
        return rc;
    }

    if(reg & STATUS_VOUT_ON)
    {
        *ldoout_enable = true;
    }
    else
    {
        *ldoout_enable = false;
    }

    return 0;

}

static int idtp9220_verify_fw_prepare(struct idtp9220_receiver *chip)
{
    int rc;

    /* disable PWM */
    rc = idtp9220_write(chip, 0x3c00, 0x80);
    if (rc < 0)
    {
        return rc;
    }

    /* core key */
    rc = idtp9220_write(chip, 0x3000, 0x5a);
    if (rc < 0)
    {
        return rc;
    }

    /* hold M0 */
    rc = idtp9220_write(chip, 0x3040, 0x11);
    if (rc < 0)
    {
        return rc;
    }

    /* OTP_VRR (VRR=3.0V) */
    rc = idtp9220_write(chip, 0x5c04, 0x05);
    if (rc < 0)
    {
        return rc;
    }

    /* OTP_CTRL (VRR_EN=1, EN=1) */
    rc = idtp9220_write(chip, 0x5c00, 0x11);
    if (rc < 0)
    {
        return rc;
    }

    return 0;
}

static int idtp9220_verify_otp_fw(struct idtp9220_receiver *chip, bool *result)
{
    unsigned char data;
    int rc, i, reg, size;

    rc = idtp9220_verify_fw_prepare(chip);
    if(rc < 0)
    {
        pr_err("idtp9220_verify_fw_prepare failed\n");
        return rc;
    }

    reg = 0x8000;
    size = sizeof(idt_firmware);
    for (i = 0; i < size; i++)
    {
        data = 0;
        rc = idtp9220_read(chip, reg + i, &data);
        if(rc <0)
        {
            return rc;
        }

        if (data != idt_firmware[i])
        {
            pr_err("fw check err data[%d]:%02x != boot[%d]:%02x.\n",
                i, data, i, idt_firmware[i]);
            return 0;
        }
    }

    *result = true;

    return 0;
}

static int idtp9220_burn_fw_end(struct idtp9220_receiver *chip)
{
    /* write key */
    idtp9220_write(chip, 0x3000, 0x5a);
    /* remove code remapping */
    idtp9220_write(chip, 0x3048, 0x00);

    /* reset M0 */
    idtp9220_write(chip, 0x3040, 0x80);
    mdelay(100);

    return 0;
}

static int idtp9220_is_fw_burned(struct idtp9220_receiver *chip, bool *is_burned)
{
    int rc, len, i, reg;
    u8 data;

    /* read otp value prepare */
    rc = idtp9220_verify_fw_prepare(chip);
    if(rc < 0)
    {
        pr_err("idtp9220_verify_fw_prepare failed\n");
        /* reset M0 */
        idtp9220_burn_fw_end(chip);
        return rc;
    }

    /* verify rx burn magic number */
    reg = 0x8000 + RX_BURN_MAGIC_NUMBER_ADDR;
    len = sizeof(burn_magic_number);
    for (i = 0; i < len; i++)
    {
        data = 0;
        rc = idtp9220_read(chip, reg + i, &data);
        if(rc <0)
        {
            pr_err("cannot read magic[%d]\n", i);
            /* reset M0 */
            idtp9220_burn_fw_end(chip);
            return rc;
        }

        if (data != burn_magic_number[i])
        {
            pr_err("burn magic number check err data[%d]:%02x != magic[%d]:%02x.\n", i, data, i, burn_magic_number[i]);
            break;
        }
    }

    if(i == len)
    {
        *is_burned = true;
    }
    else
    {
        *is_burned = false;
    }

    /* reset M0 */
    idtp9220_burn_fw_end(chip);

    return 0;
}

static int idtp9220_burn_fw_prepare(struct idtp9220_receiver *chip)
{
    int rc, len, i;
    u8 data;

    rc = idtp9220_read(chip, 0x5870, &data);
    if(rc < 0)
    {
        return rc;
    }
    pr_info("0x5870 %s:%d :%02x\n", __func__, __LINE__, data);

    rc = idtp9220_read(chip, 0x5874, &data);
    if(rc < 0)
    {
        return rc;
    }
    pr_info("0x5874 %s:%d :%02x\n", __func__, __LINE__, data);

    /* write key */
    rc = idtp9220_write(chip, 0x3000, 0x5a);
    if(rc < 0)
    {
        return rc;
    }

    /*halt M0 execution*/
    rc = idtp9220_write(chip, 0x3040, 0x10);
    if(rc < 0)
    {
        return rc;
    }

    /* download bootloader to sram */
    len = sizeof(rx_bootloader_data);
    for (i = 0; i < len; i++)
    {
        rc = idtp9220_write(chip, 0x1c00+i, rx_bootloader_data[i]);
        if (rc < 0)
        {
            pr_err("can not download bootloader to sram failed, rc=%d\n", rc);
            return rc;
        }
    }

    /* map RAM to OTP */
    rc = idtp9220_write(chip, 0x3048, 0x80);
    if(rc <0)
    {
        return rc;
    }

    /* reset chip and run the bootloader. Because of M0 reset, i2c NAK may not be get.
      * so ignore NAK
      */
    idtp9220_write(chip, 0x3040, 0x80);
    mdelay(100);

    /* verify the download bootloader data */
    rc = idtp9220_read(chip, 0x3048, &data);
    if(rc < 0)
    {
        return rc;
    }
    pr_info("0x3048 %s:%d :%02x\n", __func__, __LINE__, data);

    /* verify bootloader data */
    len = sizeof(rx_bootloader_data);
    for (i = 0; i < len; i++)
    {
        rc = idtp9220_read(chip, 0x1c00+i, &data);
        if(rc <0)
        {
            return rc;
        }

        if (data != rx_bootloader_data[i])
        {
            pr_err("MAXUEYUE bootloader check err data[%d]:%02x != boot[%d]:%02x.\n", i, data, i, rx_bootloader_data[i]);
            return 1;
        }
    }

    return 0;
}

static int idtp9220_burn_fw_process(struct idtp9220_receiver *chip, char *srcData,
    int srcOffs, int size, int otp_address)
{
    int rc, i,j;
    u16 StartAddr, CheckSum, CodeLength;

    for (i = 0; i < size; i += 128)        // program pages of 128 bytes
    {
        /* build a packet */
        StartAddr = (u16)otp_address + i;
        CheckSum = StartAddr;
        CodeLength = 128;

        memset(&chip->burn_packet, 0, sizeof(chip->burn_packet));

        /* (1) copy the 128 bytes of the OTP image data to the packet data buffer */
        if(size < 128)
        {
            memcpy(&chip->burn_packet.dataBuf, srcData + i + srcOffs, size);
        }
        else
        {
            memcpy(&chip->burn_packet.dataBuf, srcData + i + srcOffs, 128);
        }
        /* (2) calculate the packet checksum of the 128-byte data, StartAddr, and CodeLength */
        for (j = 127; j >= 0; j--)        // find the 1st non zero value byte from the end of the sBuf[] buffer
        {
            if (chip->burn_packet.dataBuf[j] != 0)
            {
                  break;
            }
            else
            {
                  CodeLength--;
            }
        }

        if (0 == CodeLength)
        {
            continue;             // skip programming if nothing to program
        }

        for (; j >= 0; j--)
        {
            CheckSum += chip->burn_packet.dataBuf[j];    // add the nonzero values
        }

        CheckSum += CodeLength;         // finish calculation of the check sum

        /*(3) fill up StartAddr, CodeLength, CheckSum of the current packet */
        memcpy(&chip->burn_packet.startAddr, &StartAddr, 2);
        memcpy(&chip->burn_packet.codeLength, &CodeLength, 2);
        memcpy(&chip->burn_packet.dataChksum, &CheckSum, 2);

        /*send the current packet to 9220 SRAM via I2C */
          /* read status is guaranteed to be != 1 at this point*/
        if (idtp9220_Xfer9220i2c(chip, 0x400, (char*)&chip->burn_packet, 0, CodeLength + 8))
        {
            pr_err("ERROR: on writing to OTP buffer");
            return 0;
        }

        /*write 1 to the Status in the SRAM. This informs the 9220 to start programming the new packet
          *from SRAM to OTP memory
          */
        chip->burn_packet.status = 1;
        if (idtp9220_Xfer9220i2c(chip, 0x400, (char*)&chip->burn_packet, 0, 1))
        {
            return 0;
        }

        /* wait for 9220 bootloader to complete programming the current packet image data from SRAM to the OTP.
         *  The boot loader will update the Status in the SRAM as follows:
         *  Status:
         * "0" - reset value (from AP)
         * "1" - buffer validated / busy (from AP)
         * "2" - finish "OK" (from the boot loader)
         * "4" - programming error (from the boot loader)
         * "8" - wrong check sum (from the boot loader)
         * "16"- programming not possible (try to write "0" to bit location already programmed to "1")
         *  (from the boot loader)
         * DateTime startT = DateTime.Now
         */
        do
        {
            mdelay(100);
            rc = idtp9220_read(chip, 0x400, (u8*)&chip->burn_packet.status);
            if(rc < 0)
            {
                return rc;
            }

            pr_info("on readign OTP buffer status sBuf:%02x i:%d\n", chip->burn_packet.status, i);
        } while (1 == chip->burn_packet.status); //check if OTP programming finishes "OK"

        if (chip->burn_packet.status != 2)        // not OK
        {
            pr_err("ERROR: buffer write to OTP returned status:%d\n" , chip->burn_packet.status);
            return 0;
        }
    }

    return 1;
}

static int idtp9220_burn_fw(struct idtp9220_receiver *chip, bool *result)
{
    int rc, len;
    bool is_burned = false;

    pr_err("process burn fw event\n");

    /*  Step1 : check whether fw has been burned */
    rc = idtp9220_is_fw_burned(chip, &is_burned);
    if(rc)
    {
        pr_err("check fw burned status failed\n");
        return rc;
    }

    if(is_burned)
    {
        pr_err("fw has been burned before and return\n");
        return 0;
    }

    pr_err("step1 success\n");

    /*  step2: Transfer 9220 boot loader code "OTPBootloader" to 9220 SRAM
     * - Setup 9220 registers before transferring the boot loader code
     * - Transfer the boot loader code to 9220 SRAM
     * - Reset 9220 => 9220 M0 runs the boot loader
     */
    rc = idtp9220_burn_fw_prepare(chip);
    if(rc < 0)
    {
        return rc;
    }
    pr_err("step2 success\n");

    /* Step3: program OTP image data to 9220 OTP memory */
    len = sizeof(idt_firmware);
    rc = idtp9220_burn_fw_process(chip, idt_firmware, 0, len, 0);
    if(rc != 1)
    {
        pr_err("can not download firmware to otp\n");
        return rc;
    }
    pr_err("step3 success\n");

    /* Step4: write burn magic number */
    len = sizeof(burn_magic_number);
    rc = idtp9220_burn_fw_process(chip, (char*)burn_magic_number, 0, len, RX_BURN_MAGIC_NUMBER_ADDR);
    if(rc != 1)
    {
        pr_err("can not write magic number to otp\n");
        return rc;
    }
    pr_err("step4 success\n");

    /* Step5: restore system (Need to reset or power cycle 9220 to run the OTP code) */
    rc = idtp9220_burn_fw_end(chip);
    if(rc < 0)
    {
        return rc;
    }
    pr_err("step5 success\n");

    *result = true;

    return 0;
}

static int idtp9220_do_device_action(struct idtp9220_receiver *chip,
                       enum idtp9220_request_type_need_rx request_type,
                       union idtp9220_interactive_data *val)
{
    int rc;

    mutex_lock(&chip->service_request_lock);

    switch (request_type)
    {
        case SET_LDO_ENABLE:
            rc = idtp9220_set_ldout_enable(chip, val->result);
            break;
        case SET_VOUT_VOLTAGE:
            rc = idtp9220_set_vout_voltage(chip, val->shortval);
            break;
        case GET_VOUT_VOLTAGE:
            rc = idtp9220_get_vout_voltage(chip, val);
            break;
        case SET_COUT_CURRENT:
            rc = idtp9220_set_cout_current(chip, val->shortval);
            break;
        case GET_COUT_CURRENT:
            rc = idtp9220_get_cout_current(chip, val);
            break;
        case SET_OPER_FREQ:
            rc = idtp9220_set_freq(chip, val);
            break;
        case GET_OPER_FREQ:
            rc = idtp9220_get_freq(chip, val);
            break;
        case GET_CHIP_INFO:
            break;
        case IS_LDO_ENABLED:
            rc = idtp9220_is_ldoout_enable(chip, &val->result);
            break;
        case IS_RX_FW_BURNED:
            rc = idtp9220_is_fw_burned(chip, &val->result);
            break;
        case DO_BURN_RX_FW:
            chip->burn_result = IN_PROCESSING;
            rc = idtp9220_burn_fw(chip, &val->result);
            break;
        case DO_VERIFY_RX_FW:
            chip->verify_result = IN_PROCESSING;
            rc = idtp9220_verify_otp_fw(chip, &val->result);
            break;
        case GET_TX_HARDWARE_VERSION:
            rc = idtp9220_get_tx_hard_version(chip, val);
            break;
        case GET_TX_TEMP:
            rc = idtp9220_get_tx_temp(chip, val);
            break;
        case GET_TX_SOFTWARE_VERSION:
            rc = idtp9220_get_tx_soft_version(chip, val);
            break;
        case GET_TX_VIN:
            rc = idtp9220_get_tx_vin(chip, val);
            break;
    }

    mutex_unlock(&chip->service_request_lock);

    return rc;
}

static void idtp9220_process_burn_fw_work(struct work_struct *work)
{
    int rc;
    struct idtp9220_receiver *chip = container_of(work,
                struct idtp9220_receiver, process_burn_fw_work);
    union idtp9220_interactive_data value = {0};

    pm_stay_awake(chip->dev);
    rc = idtp9220_do_device_action(chip, DO_BURN_RX_FW, &value);
    if(rc)
    {
        pr_err("can not do burn rx fw\n");
        pm_relax(chip->dev);
        return;
    }

    if(!value.result)
    {
        pr_err("burn fw fail\n");
        chip->burn_result = PROCESSING_FAIL;
    }
    else
    {
        pr_err("burn fw success\n");
        chip->burn_result = PROCESSING_OK;
    }
    pm_relax(chip->dev);
}

static void idtp9220_process_verify_fw_work(struct work_struct *work)
{
    int rc;
    struct idtp9220_receiver *chip = container_of(work,
                struct idtp9220_receiver, process_verify_fw_work);
    union idtp9220_interactive_data value = {0};

    pm_stay_awake(chip->dev);
    rc = idtp9220_do_device_action(chip, DO_VERIFY_RX_FW, &value);
    if(rc)
    {
        pr_err("can not do verify rx fw\n");
        pm_relax(chip->dev);
        return;
    }

    if(!value.result)
    {
        pr_err("burn fw fail\n");
        chip->verify_result = PROCESSING_FAIL;
    }
    else
    {
        pr_err("burn fw success\n");
        chip->verify_result = PROCESSING_OK;
    }
    pm_relax(chip->dev);
}

static ssize_t idtp9220_version_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
    int rc;
    union idtp9220_interactive_data chip_id, rx_fw_major_rev, rx_fw_minor_rev;
    u8 chip_rev, vset, status, cust_id;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    rc = idtp9220_read(chip, STATUS_L_REG, &status);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, VOUT_SET_REG, &vset);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, CHIP_ID_L_REG, &chip_id.strval[0]);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, CHIP_ID_H_REG, &chip_id.strval[1]);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, CHIP_REV_FONT_REG, &chip_rev);
    if(rc < 0)
    {
        goto read_version_err;
    }

    chip_rev = chip_rev >> CHIP_REV_MASK_SHIFT;

    rc = idtp9220_read(chip, CTM_ID_REG, &cust_id);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, RX_FW_MAJOR_REV_L_REG, &rx_fw_major_rev.strval[0]);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, RX_FW_MAJOR_REV_H_REG, &rx_fw_major_rev.strval[1]);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, RX_FW_MINOR_REV_L_REG, &rx_fw_minor_rev.strval[0]);
    if(rc < 0)
    {
        goto read_version_err;
    }

    rc = idtp9220_read(chip, RX_FW_MINOR_REV_H_REG, &rx_fw_minor_rev.strval[1]);
    if(rc < 0)
    {
        goto read_version_err;
    }

    return sprintf(buf, "chip_id:%04x\nchip_rev:%02x\ncust_id:%02x status:%02x vset:%02x\nrx_fw_major_rev:%04x\nrx_fw_minor_rev:%04x\n",
                 chip_id.shortval, chip_rev, cust_id, status, vset, rx_fw_major_rev.shortval,rx_fw_minor_rev.shortval);

read_version_err:
    return sprintf(buf, "Can not access idtp9220\n");
}

static ssize_t idtp9220_detect_tx_data_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
    u8 receive;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    receive = idtp9220_detect_tx_data_receive(chip);

    if (receive)
    {
        return sprintf(buf, "detect valid tx data.\n");
    }
    else
    {
        return sprintf(buf, "detect no tx data.\n");
    }
}

static ssize_t idtp9220_ldout_enable_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data result = {0};

    rc = idtp9220_do_device_action(chip, IS_LDO_ENABLED, &result);

    if(rc)
    {
        return sprintf(buf, "Can not access idtp9220\n");
    }

    if(result.result)
    {
        return sprintf(buf, "LDO Vout is ON.\n");
    }
    else
    {
        return sprintf(buf, "LDO Vout is OFF.\n");
    }
}

static ssize_t idtp9220_ldout_enable_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data result = {0};

    result.result = strncmp(buf, "1", 1) ? false : true;

    rc = idtp9220_do_device_action(chip, SET_LDO_ENABLE, &result);
    if(rc)
    {
        pr_err("can not change ldoout enable\n");
    }

    return count;
}

static ssize_t idtp9220_vout_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *di = i2c_get_clientdata(client);

    union idtp9220_interactive_data vout;

    rc = idtp9220_do_device_action(di, GET_VOUT_VOLTAGE, &vout);
    if(rc)
    {
        return sprintf(buf, "can not get vout\n");
    }

    return sprintf(buf, "Vout ADC Value: %dMV\n", vout.shortval);
}

static ssize_t idtp9220_vout_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data voltage = {0};

    voltage.shortval = (u16)simple_strtoul(buf, NULL, 10);

    rc = idtp9220_do_device_action(chip, SET_VOUT_VOLTAGE, &voltage);
    if(rc)
    {
        pr_err("can not change vout voltage\n");
    }

    return count;
}

static ssize_t idtp9220_cout_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *di = i2c_get_clientdata(client);
    union idtp9220_interactive_data cout;

    rc = idtp9220_do_device_action(di, GET_COUT_CURRENT, &cout);
    if(rc)
    {
        return sprintf(buf, "can not get cout\n");
    }

    pr_info("cout_l:%02x cout_h:%02x\n", cout.strval[0], cout.strval[1]);

    return sprintf(buf, "Output Current: %dMA\n", cout.shortval);
}

static ssize_t idtp9220_cout_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data value = {0};

    value.shortval = (u16)simple_strtoul(buf, NULL, 10);

    rc = idtp9220_do_device_action(chip, SET_COUT_CURRENT, &value);
    if(rc)
    {
        pr_err("can not change COUT current\n");
    }

    return count;
}

static ssize_t idtp9220_burn_fw_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", chip->burn_result);
}

static ssize_t idtp9220_burn_fw_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    bool enable = strncmp(buf, "1", 1) ? false : true;

    if(true == enable)
    {
        schedule_work(&chip->process_burn_fw_work);
    }

    return count;
}

static ssize_t idtp9220_verify_otp_fw_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", chip->verify_result);
}

static ssize_t idtp9220_verify_otp_fw_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    bool enable = strncmp(buf, "1", 1) ? false : true;

    if(true == enable)
    {
        schedule_work(&chip->process_verify_fw_work);
    }

    return count;
}

static ssize_t idtp9220_is_burned_fw_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data value = {0};

    rc = idtp9220_do_device_action(chip, IS_RX_FW_BURNED, &value);
    if(rc)
    {
        pr_err("can not do burn rx fw\n");
    }

    if(!value.result)
    {
        return sprintf(buf, "false\n");
    }

    return sprintf(buf, "true\n");
}

static ssize_t idtp9220_ap_mask_rxint_gpio_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{

    int enable;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    enable = (int)simple_strtoul(buf, NULL, 10);
    pr_err("set mask rxint GPIO status to %d\n", enable);
    gpio_set_value(chip->mask_wireless_int_gpio, enable);
    return count;
}

static ssize_t idtp9220_freq_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data freq = {0};
    u16 real_freq = 0;

    rc = idtp9220_do_device_action(chip, GET_OPER_FREQ, &freq);
    if(rc)
    {
        return sprintf(buf, "can not get freq\n");
    }

    pr_err("freq_l =%d, freq_h =%d\n", freq.strval[0], freq.strval[1]);
    if(freq.shortval)
    {
           real_freq = (64 * 6000)/freq.shortval;
    }

    return sprintf(buf, "freq Value: %dKHz\n", real_freq);
}

static ssize_t idtp9220_freq_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data freq = {0};

    freq.shortval = (u16)simple_strtoul(buf, NULL, 10);

    rc = idtp9220_do_device_action(chip, SET_OPER_FREQ, &freq);
    if(rc)
    {
        pr_err("can not change freq\n");
    }

    return count;
}

static ssize_t idtp9220_tx_temp_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data temp;

    rc = idtp9220_do_device_action(chip, GET_TX_TEMP, &temp);
    if(rc)
    {
        return sprintf(buf, "can not get temp\n");
    }

    return sprintf(buf, "temp1 = %d, temp2 = %d\n", (int)temp.strval[0], (int)temp.strval[1]);
 }

static ssize_t idtp9220_tx_hard_version_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data hard_version;

    rc = idtp9220_do_device_action(chip, GET_TX_HARDWARE_VERSION, &hard_version);
    if(rc)
    {
        return sprintf(buf, "can not get hard version\n");
    }

    return sprintf(buf, "tx hard version = %d\n", hard_version.shortval);
}

static ssize_t idtp9220_tx_soft_version_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data soft_version[2];

    rc = idtp9220_do_device_action(chip, GET_TX_SOFTWARE_VERSION, soft_version);
    if(rc)
    {
        return sprintf(buf, "can not get tx software version\n");
    }

    return sprintf(buf, "tx soft version = %d.%d.%d.%d\n", soft_version[1].strval[1],
             soft_version[1].strval[0], soft_version[0].strval[1], soft_version[0].strval[0]);
}

static ssize_t idtp9220_tx_vin_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data tx_vin;

    rc = idtp9220_do_device_action(chip, GET_TX_VIN, &tx_vin);
    if(rc)
    {
       return sprintf(buf, "can not get tx vin\n");
    }

    return sprintf(buf, "tx vin = %dMV\n", tx_vin.shortval);
}

static ssize_t idtp9220_intr_status_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    union idtp9220_interactive_data intr_reg;

    idtp9220_read(chip, INTR_L_REG, &intr_reg.strval[0]);

    idtp9220_read(chip, INTR_H_REG, &intr_reg.strval[1]);

    if(gpio_is_valid(chip->wireless_int_gpio))
    {
        return sprintf(buf, "INTR_L_REG = %d, INTR_H_REG = %d, wireless_int_gpio status = %d\n",
                    intr_reg.strval[0], intr_reg.strval[1], gpio_get_value(chip->wireless_int_gpio));
    }
    else
    {
        return sprintf(buf, "INTR_L_REG = %d, INTR_H_REG = %d\n", intr_reg.strval[0], intr_reg.strval[1]);
    }
}

static ssize_t idtp9220_clear_int_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int rc;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    /*clear interrupt */
    rc = idtp9220_clear_tx_data_receiv_intr(chip);
    if (rc < 0)
    {
        return sprintf(buf, "clear intr fail\n");
    }

    return sprintf(buf, "clear intr success\n");
}

static ssize_t idtp9220_using_default_vout_flag_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    return sprintf(buf, "using_default_vout_flag = %d\n", chip->using_default_vout_flag);
}

static ssize_t idtp9220_using_default_vout_flag_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);
    int flag;

    flag = simple_strtoul(buf, NULL, 10);

    if(flag)
    {
        chip->using_default_vout_flag = true;
    }
    else
    {
        chip->using_default_vout_flag = false;
    }

    idtp9220_set_vout_voltage(chip, IDT9200_VOUT_MAX_MV);

    return count;
}

static DEVICE_ATTR(version, S_IRUGO, idtp9220_version_show, NULL);
static DEVICE_ATTR(detect_tx_data, S_IRUGO, idtp9220_detect_tx_data_show, NULL);
static DEVICE_ATTR(ldout_enable, S_IWUSR | S_IRUGO, idtp9220_ldout_enable_show, idtp9220_ldout_enable_store);
static DEVICE_ATTR(vout, S_IWUSR | S_IRUGO, idtp9220_vout_show, idtp9220_vout_store);
static DEVICE_ATTR(cout, S_IWUSR | S_IRUGO, idtp9220_cout_show, idtp9220_cout_store);
static DEVICE_ATTR(verify_otp_fw, S_IWUSR | S_IRUGO, idtp9220_verify_otp_fw_show, idtp9220_verify_otp_fw_store);
static DEVICE_ATTR(is_burned_fw, S_IRUSR, idtp9220_is_burned_fw_show, NULL);
static DEVICE_ATTR(burn_fw, S_IWUSR | S_IRUGO, idtp9220_burn_fw_show, idtp9220_burn_fw_store);
static DEVICE_ATTR(freq, S_IWUSR | S_IRUGO, idtp9220_freq_show, idtp9220_freq_store);
static DEVICE_ATTR(tx_temp, S_IRUGO, idtp9220_tx_temp_show, NULL);
static DEVICE_ATTR(tx_hard_version, S_IRUGO, idtp9220_tx_hard_version_show, NULL);
static DEVICE_ATTR(tx_soft_version, S_IRUGO, idtp9220_tx_soft_version_show, NULL);
static DEVICE_ATTR(tx_vin, S_IRUGO, idtp9220_tx_vin_show, NULL);
static DEVICE_ATTR(intr_status, S_IRUGO, idtp9220_intr_status_show, NULL);
static DEVICE_ATTR(clear_int, S_IRUGO, idtp9220_clear_int_show, NULL);
static DEVICE_ATTR(using_default_vout_flag, S_IWUSR | S_IRUGO, idtp9220_using_default_vout_flag_show, idtp9220_using_default_vout_flag_store);
static DEVICE_ATTR(ap_mask_rxint_gpio, S_IWUSR, NULL, idtp9220_ap_mask_rxint_gpio_store);

static struct attribute *idtp9220_sysfs_attrs[] = {
    &dev_attr_version.attr,
    &dev_attr_detect_tx_data.attr,
    &dev_attr_ldout_enable.attr,
    &dev_attr_vout.attr,
    &dev_attr_cout.attr,
    &dev_attr_verify_otp_fw.attr,
    &dev_attr_is_burned_fw.attr,
    &dev_attr_burn_fw.attr,
    &dev_attr_freq.attr,
    &dev_attr_tx_temp.attr,
    &dev_attr_tx_hard_version.attr,
    &dev_attr_tx_soft_version.attr,
    &dev_attr_tx_vin.attr,
    &dev_attr_intr_status.attr,
    &dev_attr_clear_int.attr,
    &dev_attr_using_default_vout_flag.attr,
    &dev_attr_ap_mask_rxint_gpio.attr,
    NULL,
};

static const struct attribute_group idtp9220_sysfs_group_attrs = {
  .attrs = idtp9220_sysfs_attrs,
};

static void idtp9220_monitor_work(struct work_struct *work)
{
    int vbat;
    union power_supply_propval ret = {0,};
    union idtp9220_interactive_data ldoout_enable = {0};
    union idtp9220_interactive_data vout = {0};
    union idtp9220_interactive_data freq = {0};
    union power_supply_propval new_current_limit = {0,};

    struct delayed_work *dwork = to_delayed_work(work);
    struct idtp9220_receiver *chip = container_of(dwork,
                    struct idtp9220_receiver, monitor_work);

    if (!chip->batt_psy)
    {
        chip->batt_psy = power_supply_get_by_name("battery");
    }

    if(chip->batt_psy)
    {
        /* idtp9220 monitor mechanism:
         * condition is that rx ldo  is on and can get vbat.
         * rx vout vary with vbat.
         * usb input current limit vary with vbat.
         */
        idtp9220_do_device_action(chip, IS_LDO_ENABLED, &ldoout_enable);

        if(ldoout_enable.result)
        {
            /* set freq */
            freq.shortval = IDTP9220_TX_FREQ_KHZ;
            idtp9220_do_device_action(chip, SET_OPER_FREQ, &freq);

            /* check whether battery is present */
            chip->batt_psy->get_property(chip->batt_psy,
                  POWER_SUPPLY_PROP_PRESENT, &ret);
            if(ret.intval)
            {
                /* get battery voltage */
                chip->batt_psy->get_property(chip->batt_psy,
                      POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
                vbat = ret.intval / 1000;

                if(vbat < IDTP9220_VBAT_DEFAULT_MV)
                {
                    vout.shortval = IDTP9220_VOUT_DEFAULT_MV;
                }
                else
                {
                    vout.shortval = vbat + IDTP9220_VOUT_ADJUST_STEP_MV;
                }

                /* danymic change rx vout */
                pr_info("vbat = %d, vout = %d\n",vbat, vout.shortval);
                get_monotonic_boottime(&chip->last_set_vout_time);
                pr_info("chip->last_setvout_time =%ld\n",
                             chip->last_set_vout_time.tv_sec);
                idtp9220_do_device_action(chip, SET_VOUT_VOLTAGE, &vout);

                /* danymic change usb input current */
                if(vbat >= IDTP9220_VBAT_HIGH_MV && !chip->vbat_high)
                {
                    pr_err("adjust to low usb inpuct current limit\n");
                    chip->vbat_high = true;
                    new_current_limit.intval = IDTP9220_VBAT_HIGH_CUR_LIMIT_MA;
                    chip->batt_psy->set_property(chip->batt_psy,
                        POWER_SUPPLY_PROP_USB_INPUT_CURRENT, &new_current_limit);
                }
                else if(vbat < IDTP9220_VBAT_HIGH_MV - IDTP9220_VBAT_HIGH_DELTA_MV && chip->vbat_high)
                {
                    pr_err("adjust to low cur limit\n");
                    chip->vbat_high = false;
                    new_current_limit.intval = IDTP9220_VBAT_LOW_CUR_LIMIT_MA;
                    chip->batt_psy->set_property(chip->batt_psy,
                        POWER_SUPPLY_PROP_USB_INPUT_CURRENT, &new_current_limit);
                }
            }
        }
    }

    schedule_delayed_work(&chip->monitor_work,
            msecs_to_jiffies(IDTP9220_MONITOR_PERIOD_MS));
}

static void idtp9220_process_intr_work(struct work_struct *work)
{
    u8 reg = 0;
    struct idtp9220_receiver *chip = container_of(work,
                struct idtp9220_receiver, process_intr_work);

    pr_err("process wireless intr work\n");
    idtp9220_read(chip, INTR_L_REG, &reg);

    if(reg && TX_DATA_RECV)
    {
        pr_err("receive tx data \n");
        up(&chip->tx_send_data_int);
    }
    else
    {
        idtp9220_clear_tx_data_receiv_intr(chip);
    }
}

static irqreturn_t idtp9220_receiver_stat_handler(int irq, void *dev_id)
{
    struct idtp9220_receiver *chip = dev_id;

    pr_err("Ap get an wireless interupt\n");
    schedule_work(&chip->process_intr_work);

    return IRQ_HANDLED;
}

static int idtp9220_initialize_gpio_state(struct idtp9220_receiver *chip)
{
    int ret;
    struct pinctrl *pctrl;
    struct pinctrl_state *pctrl_state;

    /* set the pinctrl state */
    pctrl = devm_pinctrl_get(chip->dev);
    if (IS_ERR(pctrl))
    {
        pr_err("pinctrl get failed\n");
        return PTR_ERR(pctrl);
    }

    pctrl_state = pinctrl_lookup_state(pctrl, "default");
    if (IS_ERR(pctrl_state))
    {
        pr_err("pinctrl get state failed\n");
        ret = PTR_ERR(pctrl_state);
        goto err;
    }

    ret = pinctrl_select_state(pctrl, pctrl_state);
    if (ret)
    {
        pr_err("pinctrl enable state failed\n");
        goto err;
    }

    return 0;

err:
    devm_pinctrl_put(pctrl);
    return ret;
}

static int idtp9220_initialize_gpio_direction_value(struct idtp9220_receiver *chip)
{
    int rc;

    /* initialize ap mask rx int gpio  */
    chip->mask_wireless_int_gpio = of_get_named_gpio(chip->dev->of_node, "mask-wireless-int-gpio", 0);
    if (!gpio_is_valid(chip->mask_wireless_int_gpio))
    {
        pr_err("Looking up mask-wireless-int-gpio property in node %s failed %d\n",
            chip->dev->of_node->full_name,
            chip->mask_wireless_int_gpio);
        return -EINVAL;
    }

    rc = gpio_request(chip->mask_wireless_int_gpio, "mask-wireless-int-gpio");
    if (rc)
    {
        gpio_free(chip->mask_wireless_int_gpio);
        rc = gpio_request(chip->mask_wireless_int_gpio, "mask-wireless-int-gpio");
        if(rc)
        {
            pr_err("can't request gpio port %d\n", chip->mask_wireless_int_gpio);
            return -EINVAL;
        }
    }
    gpio_direction_output(chip->mask_wireless_int_gpio, 0);

    /* initialize rx int ap gpio */
    chip->wireless_int_gpio= of_get_named_gpio(chip->dev->of_node, "wireless-int-gpio", 0);
    if (!gpio_is_valid(chip->wireless_int_gpio))
    {
        pr_err("Looking up mask-rxint-gpio property in node %s failed %d\n",
            chip->dev->of_node->full_name,
            chip->wireless_int_gpio);
        return -EINVAL;
    }

    rc = gpio_request(chip->wireless_int_gpio, "wireless-int-gpio");
    if (rc)
    {
        gpio_free(chip->wireless_int_gpio);
        rc = gpio_request(chip->wireless_int_gpio, "wireless-int-gpio");
        if(rc)
        {
            pr_err("can't request gpio port %d\n", chip->wireless_int_gpio);
            return -EINVAL;
        }
    }

    gpio_direction_input(chip->wireless_int_gpio);

    return 0;
}

static int idtp9220_receiver_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    int rc;
    struct idtp9220_receiver *chip;

    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
    {
        pr_err("Couldn't allocate memory\n");
        return -ENOMEM;
    }

    chip->client = client;
    chip->dev = &client->dev;
    i2c_set_clientdata(client, chip);

    mutex_init(&chip->read_write_lock);
    mutex_init(&chip->service_request_lock);
    device_init_wakeup(chip->dev, true);

    idtp9220_initialize_gpio_state(chip);
    idtp9220_initialize_gpio_direction_value(chip);

    /* STAT irq configuration */
    if (client->irq)
    {
        rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
                idtp9220_receiver_stat_handler,
                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                "idtp9220_receiver_stat_irq", chip);
        if (rc < 0)
        {
            pr_err("request_irq for irq=%d  failed rc = %d\n", client->irq, rc);
        }
        enable_irq_wake(client->irq);
    }

    sema_init(&chip->tx_send_data_int, 0);

    chip->batt_psy = power_supply_get_by_name("battery");

    if(sysfs_create_group(&client->dev.kobj, &idtp9220_sysfs_group_attrs))
    {
        pr_err("create sysfs attrs failed!\n");
        return -EIO;
    }

    memset(&chip->last_set_vout_time, 0, sizeof(struct timespec));
    INIT_DELAYED_WORK(&chip->monitor_work, idtp9220_monitor_work);
    INIT_WORK(&chip->process_intr_work, idtp9220_process_intr_work);
    INIT_WORK(&chip->process_burn_fw_work, idtp9220_process_burn_fw_work);
    INIT_WORK(&chip->process_verify_fw_work, idtp9220_process_verify_fw_work);
    schedule_delayed_work(&chip->monitor_work,
        msecs_to_jiffies(IDTP9220_MONITOR_PERIOD_MS));

    global_idtp9220_receiver = chip;

    return 0;
}

static int idtp9220_receiver_remove(struct i2c_client *client)
{
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    cancel_delayed_work_sync(&chip->monitor_work);

    return 0;
}

static int idtp9220_resume(struct device *dev)
{
    bool ldout_enable = false;
    struct i2c_client *client = to_i2c_client(dev);
    struct idtp9220_receiver *chip = i2c_get_clientdata(client);

    idtp9220_is_ldoout_enable(chip, &ldout_enable);
    if(ldout_enable)
    {
        get_monotonic_boottime(&chip->resume_time);
        pr_err("resume_time = %ld, last_setvout_time =%ld\n",
            chip->resume_time.tv_sec, chip->last_set_vout_time.tv_sec);

        if( (chip->resume_time.tv_sec - chip->last_set_vout_time.tv_sec) >
             IDTP9220_MONITOR_PERIOD_MS / 1000)
        {
            /*make sure no work waits in the queue, if the work is already queued
             *(not on the timer) the cancel will fail. That is not a problem
             *because we just want the work started.
             */
            cancel_delayed_work_sync(&chip->monitor_work);
            schedule_delayed_work(&chip->monitor_work, 0);
        }
    }

    return 0;
}

static int idtp9220_suspend(struct device *dev)
{
    return 0;
}

static const struct dev_pm_ops idtp9220_pm_ops = {
    .resume        = idtp9220_resume,
    .suspend       = idtp9220_suspend,
};

static const struct i2c_device_id idtp9220_receiver_id[] = {
    {"idtp9220-receiver", 0},
    {},
};

static struct of_device_id idtp9220_match_table[] = {
    {
        .compatible    = "idt,idtp9220-receiver",
    },
    { },
};

MODULE_DEVICE_TABLE(i2c, idtp9220_receiver_id);

static struct i2c_driver idtp9220_receiver_driver = {
    .driver        = {
        .name        = "idtp9220-receiver",
        .owner        = THIS_MODULE,
        .of_match_table    = idtp9220_match_table,
        .pm        = &idtp9220_pm_ops,
    },
    .probe        = idtp9220_receiver_probe,
    .remove        = idtp9220_receiver_remove,
    .id_table    = idtp9220_receiver_id,
};

module_i2c_driver(idtp9220_receiver_driver);

MODULE_DESCRIPTION("idtp9220 Receiver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:idtp9220-receiver");

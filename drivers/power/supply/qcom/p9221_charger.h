/*
 * P9221 Wireless Charger Driver
 *
 * Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#ifndef __P9220_CHARGER_H__
#define __P9220_CHARGER_H__

#define P9221_CHIP_ID_L_REG		0x00
#define P9221_CHIP_ID_H_REG		0x01
#define P9221_CHIP_REVISION_REG		0x02
#define P9221_CUSTOMER_ID_REG		0x03
#define P9221_OTP_FW_MAJOR_REV_L_REG	0x04
#define P9221_OTP_FW_MINOR_REV_L_REG	0x06
#define P9221_OTP_FW_DATE_REG		0x08
#define P9221_OTP_FW_TIME_REG		0x14
#define P9221_SRAM_FW_MAJOR_REV_L_REG	0x1C
#define P9221_SRAM_FW_MINOR_REV_L_REG	0x1E
#define P9221_SRAM_FW_DATE_REG		0x20
#define P9221_SRAM_FW_TIME_REG		0x2C
#define P9221_STATUS_L_REG		0x34
#define P9221_INT_L_REG			0x36
#define P9221_INT_L_MASK		0xF7
#define P9221_INT_L_STAT_VOUT		BIT(7)
#define P9221_INT_L_STAT_VRECT		BIT(6)
#define P9221_INT_L_STAT_ACMISSING	BIT(5)
#define P9221_INT_L_STAT_TX_DATA_RX	BIT(4)
#define P9221_INT_L_STAT_OV_TEMP	BIT(2)
#define P9221_INT_L_STAT_OV_VOLT	BIT(1)
#define P9221_INT_L_STAT_OV_CURRENT	BIT(0)
#define P9221_INT_L_STAT_OV_MASK	(P9221_INT_L_STAT_OV_TEMP | \
					 P9221_INT_L_STAT_OV_VOLT | \
					 P9221_INT_L_STAT_OV_CURRENT)
#define P9221_INT_ENABLE_L_REG		0x38
#define P9221_CHG_STATUS_REG		0x3A
#define P9221_EPT_REG			0x3B
#define P9221_ADC_VOUT_L_REG		0x3C
#define P9221_ADC_VOUT_MASK		0x0FFF
#define P9221_VOUT_SET_REG		0x3E
#define P9221_MAX_VOUT_SET_MV_DEFAULT	9000 /* mV */
#define P9221_ADC_VRECT_L_REG		0x40
#define P9221_ADC_VRECT_MASK		0x0FFF
#define P9221_OVSET_REG			0x42
#define P9221_OVSET_MASK		0xF0
#define P9221_OVSET_SHIFT		4
#define P9221_RX_IOUT_L_REG		0x44
#define P9221_ADC_DIE_TEMP_L_REG	0x46
#define P9221_ADC_DIE_TEMP_MASK		0x0FFF
#define P9221_OP_FREQ_L_REG		0x48
#define P9221_ILIM_SET_REG		0x4A
#define P9221_ADC_ALIGN_X_REG		0x4B
#define P9221_ADC_ALIGN_Y_REG		0x4C
#define P9221_OP_MODE_REG		0x4D
#define P9221_COM_REG			0x4E
#define P9221_COM_SWITCH_TO_RAM_MASK	BIT(6)
#define P9221_COM_CLEAR_INT_MASK	BIT(5)
#define P9221_COM_SEND_CHG_STAT_MASK	BIT(4)
#define P9221_COM_SEND_EOP_MASK		BIT(3)
#define P9221_COM_SEND_RX_DATA_MASK	BIT(0)
#define P9221_INT_CLEAR_L_REG		0x56
#define P9221_INT_CLEAR_H_REG		0x57
#define P9221_RXID_REG			0x5C
#define P9221_RXID_LEN			6
#define P9221_MPREQ_REG			0x5C
#define P9221_MPREQ_LEN			6
#define P9221_OV_CLAMP_REG		0x62
#define P9221_OV_CLAMP_MASK		0x07
#define P9221_FOD_REG			0x68
#define P9221_NUM_FOD			16
#define P9221_RX_RAWIOUT_L_REG		0x7A
#define P9221_RX_RAWIOUT_MASK		0x0FFF
#define P9221_PMA_AD_L_REG		0x7C
#define P9221_RX_PINGFREQ_L_REG		0xFC
#define P9221_RX_PINGFREQ_MASK		0x0FFF
#define P9221_LAST_REG			0xFF

#define P9221_EOP_UNKNOWN		0x00
#define P9221_EOP_EOC			0x01
#define P9221_EOP_INTERNAL_FAULT	0x02
#define P9221_EOP_OVER_TEMP		0x03
#define P9221_EOP_OVER_VOLT		0x04
#define P9221_EOP_OVER_CURRENT		0x05
#define P9221_EOP_BATT_FAIL		0x06
#define P9221_EOP_RECONFIG		0x07
#define P9221_EOP_NO_RESPONSE		0x08
#define P9221_EOP_NEGOTIATION_FAIL	0x0A
#define P9221_EOP_RESTART_POWER		0x0B

struct p9221_charger_platform_data {
	int irq_gpio;
	int irq_int;
	int max_vout_mv;
	u8 fod[P9221_NUM_FOD];
	bool fod_set;
};

struct p9221_charger_data {
	struct i2c_client			*client;
	struct p9221_charger_platform_data	*pdata;
	struct power_supply			*wc_psy;
	struct power_supply			*dc_psy;
	struct notifier_block			nb;
	struct mutex				io_lock;
	struct device				*dev;
	struct delayed_work			notifier_work;
	int					online;
	u16					addr;
	u8					count;
};

struct p9221_prop_reg_map_entry {
	enum power_supply_property	prop;
	u16				reg;
	bool				get;
	bool				set;
};


#define P9221_SHOW_COOKED(name, reg, format)				\
	static ssize_t p9221_show_##name(struct device *dev,		\
					struct device_attribute *attr,	\
					char *buf)			\
	{								\
		struct i2c_client *client = to_i2c_client(dev);		\
		struct p9221_charger_data *charger =			\
					i2c_get_clientdata(client);	\
		u32 val;						\
		int ret;						\
									\
		ret = p9221_reg_read_cooked(charger, reg, &val);	\
		if (ret)						\
			return ret;					\
		return snprintf(buf, PAGE_SIZE, format, val);		\
	}

#define P9221_SHOW(name, reg, width, mask, format)			\
	static ssize_t p9221_show_##name(struct device *dev,		\
					struct device_attribute *attr,	\
					char *buf)			\
	{								\
		struct i2c_client *client = to_i2c_client(dev);		\
		struct p9221_charger_data *charger =			\
					i2c_get_clientdata(client);	\
		u##width val;						\
		int ret;						\
									\
		ret = p9221_reg_read_##width(charger, reg, &val);	\
		if (ret)						\
			return ret;					\
		val &= mask;						\
		return snprintf(buf, PAGE_SIZE, format, val);		\
	}

#define P9221_STORE_COOKED(name, reg)					\
	static ssize_t p9221_store_##name(struct device *dev,		\
					  struct device_attribute *attr,\
					  const char *buf, size_t count)\
	{								\
		struct i2c_client *client = to_i2c_client(dev);		\
		struct p9221_charger_data *charger =			\
					i2c_get_clientdata(client);	\
		u32 val;						\
		int ret;						\
									\
		ret = kstrtou32(buf, 0, &val);				\
		if (ret < 0)						\
			return ret;					\
									\
		ret = p9221_reg_write_cooked(charger, reg, val);	\
		if (ret)						\
			return ret;					\
		return count;						\
	}

#define P9221_STORE(name, reg, width, mask)				\
	static ssize_t p9221_store_##name(struct device *dev,		\
					  struct device_attribute *attr,\
					  const char *buf, size_t count)\
	{								\
		struct i2c_client *client = to_i2c_client(dev);		\
		struct p9221_charger_data *charger =			\
					i2c_get_clientdata(client);	\
		u##width val;						\
		int ret;						\
									\
		ret = kstrtou##width(buf, 0, &val);			\
		if (ret < 0)						\
			return ret;					\
									\
		val &= mask;						\
		ret = p9221_reg_write_##width(charger, reg, val);	\
		if (ret)						\
			return ret;					\
		return count;						\
	}

#define P9221_SHOW_STORE(name, reg, width, mask, format) \
	P9221_SHOW(name, reg, width, mask, format)	 \
	P9221_STORE(name, reg, width, mask)

#define P9221_SHOW_STORE_COOKED(name, reg, format) \
	P9221_SHOW_COOKED(name, reg, format)	 \
	P9221_STORE_COOKED(name, reg)

#endif /* __P9220_CHARGER_H__ */

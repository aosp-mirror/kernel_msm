/* touch_hwif.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define TS_MODULE "[hwif]"

#include <linux/gpio.h>
/*#include <linux/regulator/machine.h>*/
#include <linux/regulator/consumer.h>
#include <linux/irq.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_i2c.h>
#include <touch_spi.h>
#include <touch_hwif.h>
#include "../../../../kernel/irq/internals.h"

/* -- gpio -- */
int touch_gpio_init(int pin, const char *name)
{
	int ret = 0;
	TOUCH_I("%s - pin:%d, name:%s\n", __func__, pin, name);

	if (gpio_is_valid(pin))
		ret = gpio_request(pin, name);

	return ret;
}
EXPORT_SYMBOL(touch_gpio_init);

void touch_gpio_direction_input(int pin)
{
	TOUCH_I("%s - pin:%d\n", __func__, pin);

	if (gpio_is_valid(pin))
		gpio_direction_input(pin);
}
EXPORT_SYMBOL(touch_gpio_direction_input);

void touch_gpio_direction_output(int pin, int value)
{
	TOUCH_I("%s - pin:%d, value:%d\n", __func__, pin, value);

	if (gpio_is_valid(pin))
		gpio_direction_output(pin, value);
}
EXPORT_SYMBOL(touch_gpio_direction_output);

/* -- power -- */
int touch_power_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (gpio_is_valid(ts->vdd_pin)) {
		gpio_request(ts->vdd_pin, "touch-vdd");
	} else {
		ts->vdd = regulator_get(dev, "vdd");
		if (IS_ERR(ts->vdd))
			TOUCH_I("regulator \"vdd\" not exist\n");
	}

	if (gpio_is_valid(ts->vio_pin)) {
		gpio_request(ts->vio_pin, "touch-vio");
	} else {
		ts->vio = regulator_get(dev, "vio");
		if (IS_ERR(ts->vio))
			TOUCH_I("regulator \"vio\" not exist\n");
	}

	return 0;
}
EXPORT_SYMBOL(touch_power_init);

void touch_power_vdd(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (gpio_is_valid(ts->vdd_pin)) {
		ret = gpio_direction_output(ts->vdd_pin, value);
	} else if (!IS_ERR_OR_NULL(ts->vdd)) {
		if (value)
			ret = regulator_enable((struct regulator *)ts->vdd);
		else
			ret = regulator_disable((struct regulator *)ts->vdd);
	}

	if (ret)
		TOUCH_E("ret = %d\n", ret);
}
EXPORT_SYMBOL(touch_power_vdd);

void touch_power_vio(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (gpio_is_valid(ts->vio_pin)) {
		ret = gpio_direction_output(ts->vio_pin, value);
	} else if (!IS_ERR_OR_NULL(ts->vio)) {
		if (value)
			ret = regulator_enable((struct regulator *)ts->vio);
		else
			ret = regulator_disable((struct regulator *)ts->vio);
	}

	if (ret)
		TOUCH_E("ret = %d\n", ret);
}
EXPORT_SYMBOL(touch_power_vio);

int touch_bus_init(struct device *dev, int buf_size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (buf_size) {
		ts->tx_buf = devm_kzalloc(dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!ts->tx_buf)
			TOUCH_E("fail to allocate tx_buf\n");

		ts->rx_buf = devm_kzalloc(dev, buf_size, GFP_KERNEL | GFP_DMA);
		if (!ts->rx_buf)
			TOUCH_E("fail to allocate rx_buf\n");
	}

	ts->xfer = devm_kzalloc(dev, sizeof(struct touch_xfer_msg), GFP_KERNEL);
	if (!ts->xfer)
		TOUCH_E("fail to allocate xfer\n");

	ts->pinctrl.ctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(ts->pinctrl.ctrl)) {
		if (PTR_ERR(ts->pinctrl.ctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		TOUCH_I("Target does not use pinctrl\n");
		ts->pinctrl.ctrl = NULL;
	} else {
		ts->pinctrl.active = pinctrl_lookup_state(ts->pinctrl.ctrl,
				"touch_pin_active");

		if (IS_ERR_OR_NULL(ts->pinctrl.active))
			TOUCH_E("cannot get pinctrl.active\n");

		ts->pinctrl.suspend = pinctrl_lookup_state(ts->pinctrl.ctrl,
				"touch_pin_sleep");

		if (IS_ERR_OR_NULL(ts->pinctrl.suspend))
			TOUCH_E("cannot get pinctrl.suspend\n");

		if (!IS_ERR_OR_NULL(ts->pinctrl.active)) {
			ret = pinctrl_select_state(ts->pinctrl.ctrl,
					ts->pinctrl.active);
			if (ret)
				TOUCH_I("cannot set pinctrl.active\n");
			else
				TOUCH_I("pinctrl set active\n");
		}
	}

	if (ts->bus_type == HWIF_SPI) {
		ret = spi_setup(to_spi_device(dev));

		if (ret < 0) {
			TOUCH_E("Failed to perform SPI setup\n");
			return -ENODEV;
		}
	}

	return ret;
}
EXPORT_SYMBOL(touch_bus_init);

int touch_bus_read(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_read(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI)
		ret = touch_spi_read(to_spi_device(dev), msg);

	return ret;
}
EXPORT_SYMBOL(touch_bus_read);

int touch_bus_write(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_write(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI)
		ret = touch_spi_write(to_spi_device(dev), msg);

	return ret;
}
EXPORT_SYMBOL(touch_bus_write);

int touch_bus_xfer(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C) {
		TOUCH_E("Not implemented\n");
		ret = -1;
	} else if (ts->bus_type == HWIF_SPI) {
		ret = touch_spi_xfer(to_spi_device(dev), xfer);
	}

	return ret;
}
EXPORT_SYMBOL(touch_bus_xfer);

void touch_enable_irq_wake(unsigned int irq)
{
	enable_irq_wake(irq);
}

void touch_disable_irq_wake(unsigned int irq)
{
	disable_irq_wake(irq);
}

#define istate core_internal_state__do_not_mess_with_it
void touch_enable_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if (desc) {
		if (desc->istate & IRQS_PENDING)
			TOUCH_D(BASE_INFO, "Remove pending irq(%d)\n", irq);
		desc->istate &= ~(IRQS_PENDING);
	}
	enable_irq(irq);
}

void touch_disable_irq(unsigned int irq)
{
	disable_irq_nosync(irq);
}

int touch_request_irq(unsigned int irq, irq_handler_t handler,
		     irq_handler_t thread_fn,
		     unsigned long flags, const char *name, void *dev)
{
	return request_threaded_irq(irq, handler, thread_fn, flags, name, dev);
}

void touch_resend_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	if (desc) {
		if (desc->istate & IRQS_PENDING)
			TOUCH_D(BASE_INFO, "irq(%d) pending\n", irq);
	}
}

void touch_set_irq_pending(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	TOUCH_D(BASE_INFO, "%s : irq=%d\n", __func__, irq);

	if (desc) {
		raw_spin_lock_irqsave(&desc->lock, flags);
		desc->istate |= IRQS_PENDING;
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}

int touch_boot_mode(void)
{
	int ret = 0;
/*	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO)
		ret = TOUCH_CHARGER_MODE;
*/
	return ret;
}
EXPORT_SYMBOL(touch_boot_mode);

int touch_boot_mode_check(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	//ret = lge_get_factory_boot();

	if (ret != NORMAL_BOOT) {
		switch (atomic_read(&ts->state.mfts)) {
			case MFTS_NONE :
				ret = MINIOS_AAT;
				break;
			case MFTS_FOLDER :
				ret = MINIOS_MFTS_FOLDER;
				break;
			case MFTS_FLAT :
				ret = MINIOS_MFTS_FLAT;
				break;
			case MFTS_CURVED :
				ret = MINIOS_MFTS_CURVED;
				break;
			default :
				ret = MINIOS_AAT;
				break;
		}
	}
	else
		ret = NORMAL_BOOT;

	return ret;
}
EXPORT_SYMBOL(touch_boot_mode_check);

enum touch_device_type touch_get_device_type(void)
{
	enum touch_device_type ret = TYPE_SW49408;

	TOUCH_I("%s = [%d]\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(touch_get_device_type);

int touch_bus_device_init(struct touch_hwif *hwif, void *driver)
{
	if (hwif->bus_type == HWIF_I2C)
		return touch_i2c_device_init(hwif, driver);
	else if (hwif->bus_type == HWIF_SPI)
		return touch_spi_device_init(hwif, driver);

	TOUCH_E("Unknown touch interface : %d\n", hwif->bus_type);

	return -ENODEV;
}
EXPORT_SYMBOL(touch_bus_device_init);

void touch_bus_device_exit(struct touch_hwif *hwif)
{
	if (hwif->bus_type == HWIF_I2C)
		touch_i2c_device_exit(hwif);
	else if (hwif->bus_type == HWIF_SPI)
		touch_spi_device_exit(hwif);
}
EXPORT_SYMBOL(touch_bus_device_exit);

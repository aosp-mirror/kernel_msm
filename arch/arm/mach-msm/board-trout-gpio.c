/* arch/arm/mach-msm/board-trout-gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/sysdev.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/htc_pwrsink.h>

#include "board-trout.h"
#include "gpio_chip.h"

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_trout."

static uint cpld_usb_h2w_sw;
module_param_named(usb_h2w_sw, cpld_usb_h2w_sw, uint, 0);

static uint8_t trout_cpld_shadow[4] = {
#if defined(CONFIG_MSM_DEBUG_UART1)
	/* H2W pins <-> UART1 */
        [0] = 0x40, // for serial debug, low current
#else
	/* H2W pins <-> UART3, Bluetooth <-> UART1 */
        [0] = 0x80, // for serial debug, low current
#endif
        [1] = 0x04, // I2C_PULL
        [3] = 0x04, // mmdi 32k en
};
static uint8_t trout_int_mask[2] = {
        [0] = 0xff, /* mask all interrupts */
        [1] = 0xff,
};
static uint8_t trout_sleep_int_mask[] = {
        [0] = 0xff,
        [1] = 0xff,
};
static int trout_suspended;

static int trout_gpio_read(struct gpio_chip *chip, unsigned n)
{
	uint8_t b;
	int reg;
	if (n >= TROUT_GPIO_VIRTUAL_BASE)
		n += TROUT_GPIO_VIRTUAL_TO_REAL_OFFSET;
	b = 1U << (n & 7);
	reg = (n & 0x78) >> 2; // assumes base is 128
	return !!(readb(TROUT_CPLD_BASE + reg) & b);
}

static void update_pwrsink(unsigned gpio, unsigned on)
{
	switch(gpio) {
	case TROUT_GPIO_UI_LED_EN:
		htc_pwrsink_set(PWRSINK_LED_BUTTON, on ? 100 : 0);
		break;
	case TROUT_GPIO_QTKEY_LED_EN:
		htc_pwrsink_set(PWRSINK_LED_KEYBOARD, on ? 100 : 0);
		break;
	}
}

static uint8_t trout_gpio_write_shadow(unsigned n, unsigned on)
{
	uint8_t b = 1U << (n & 7);
	int reg = (n & 0x78) >> 2; // assumes base is 128

	if(on)
		return trout_cpld_shadow[reg >> 1] |= b;
	else
		return trout_cpld_shadow[reg >> 1] &= ~b;
}

static int trout_gpio_write(struct gpio_chip *chip, unsigned n, unsigned on)
{
	int reg = (n & 0x78) >> 2; // assumes base is 128
	unsigned long flags;
	uint8_t reg_val;

	if ((reg >> 1) >= ARRAY_SIZE(trout_cpld_shadow)) {
		printk(KERN_ERR "trout_gpio_write called on input %d\n", n);
		return -ENOTSUPP;
	}

	local_irq_save(flags);
	update_pwrsink(n, on);
	reg_val = trout_gpio_write_shadow(n, on);
	writeb(reg_val, TROUT_CPLD_BASE + reg);
	local_irq_restore(flags);
	return 0;
}

static int trout_gpio_configure(struct gpio_chip *chip, unsigned int gpio, unsigned long flags)
{
	if(flags & (GPIOF_OUTPUT_LOW | GPIOF_OUTPUT_HIGH))
		trout_gpio_write(chip, gpio, flags & GPIOF_OUTPUT_HIGH);
	return 0;
}

static int trout_gpio_get_irq_num(struct gpio_chip *chip, unsigned int gpio, unsigned int *irqp, unsigned long *irqnumflagsp)
{
	if ((gpio < TROUT_GPIO_BANK0_FIRST_INT_SOURCE ||
	     gpio > TROUT_GPIO_BANK0_LAST_INT_SOURCE) &&
	    (gpio < TROUT_GPIO_BANK1_FIRST_INT_SOURCE ||
	     gpio > TROUT_GPIO_BANK1_LAST_INT_SOURCE))
		return -ENOENT;
	*irqp = TROUT_GPIO_TO_INT(gpio);
	if(irqnumflagsp)
		*irqnumflagsp = 0;
	return 0;
}

static void trout_gpio_irq_ack(unsigned int irq)
{
	int bank = TROUT_INT_TO_BANK(irq);
	uint8_t mask = TROUT_INT_TO_MASK(irq);
	int reg = TROUT_BANK_TO_STAT_REG(bank);
	/*printk(KERN_INFO "trout_gpio_irq_ack irq %d\n", irq);*/
	writeb(mask, TROUT_CPLD_BASE + reg);
}

static void trout_gpio_irq_mask(unsigned int irq)
{
	unsigned long flags;
	uint8_t reg_val;
	int bank = TROUT_INT_TO_BANK(irq);
	uint8_t mask = TROUT_INT_TO_MASK(irq);
	int reg = TROUT_BANK_TO_MASK_REG(bank);

	local_irq_save(flags);
	reg_val = trout_int_mask[bank] |= mask;
	/*printk(KERN_INFO "trout_gpio_irq_mask irq %d => %d:%02x\n",
	       irq, bank, reg_val);*/
	if (!trout_suspended)
		writeb(reg_val, TROUT_CPLD_BASE + reg);
	local_irq_restore(flags);
}

static void trout_gpio_irq_unmask(unsigned int irq)
{
	unsigned long flags;
	uint8_t reg_val;
	int bank = TROUT_INT_TO_BANK(irq);
	uint8_t mask = TROUT_INT_TO_MASK(irq);
	int reg = TROUT_BANK_TO_MASK_REG(bank);

	local_irq_save(flags);
	reg_val = trout_int_mask[bank] &= ~mask;
	/*printk(KERN_INFO "trout_gpio_irq_unmask irq %d => %d:%02x\n",
	       irq, bank, reg_val);*/
	if (!trout_suspended)
		writeb(reg_val, TROUT_CPLD_BASE + reg);
	local_irq_restore(flags);
}

int trout_gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	unsigned long flags;
	int bank = TROUT_INT_TO_BANK(irq);
	uint8_t mask = TROUT_INT_TO_MASK(irq);

	local_irq_save(flags);
	if(on)
		trout_sleep_int_mask[bank] &= ~mask;
	else
		trout_sleep_int_mask[bank] |= mask;
	local_irq_restore(flags);
	return 0;
}

static void trout_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int j, m;
	unsigned v;
	int bank;
	int stat_reg;
	int int_base = TROUT_INT_START;
	uint8_t int_mask;

	for (bank = 0; bank < 2; bank++) {
		stat_reg = TROUT_BANK_TO_STAT_REG(bank);
		v = readb(TROUT_CPLD_BASE + stat_reg);
		int_mask = trout_int_mask[bank];
		if (v & int_mask) {
			writeb(v & int_mask, TROUT_CPLD_BASE + stat_reg);
			printk(KERN_ERR "trout_gpio_irq_handler: got masked "
			       "interrupt: %d:%02x\n", bank, v & int_mask);
		}
		v &= ~int_mask;
		while (v) {
			m = v & -v;
			j = fls(m) - 1;
			/*printk(KERN_INFO "msm_gpio_irq_handler %d:%02x %02x b"
			       "it %d irq %d\n", bank, v, m, j, int_base + j);*/
			v &= ~m;
			generic_handle_irq(int_base + j);
		}
		int_base += TROUT_INT_BANK0_COUNT;
	}
	desc->chip->ack(irq);
}

static int trout_sysdev_suspend(struct sys_device *dev, pm_message_t state)
{
	trout_suspended = 1;
	writeb(trout_sleep_int_mask[0],
	       TROUT_CPLD_BASE + TROUT_GPIO_INT_MASK0_REG);
	writeb(trout_sleep_int_mask[1],
	       TROUT_CPLD_BASE + TROUT_GPIO_INT_MASK1_REG);
	writeb(trout_sleep_int_mask[0],
	       TROUT_CPLD_BASE + TROUT_GPIO_INT_STAT0_REG);
	writeb(trout_sleep_int_mask[1],
	       TROUT_CPLD_BASE + TROUT_GPIO_INT_STAT1_REG);
	return 0;
}

int trout_sysdev_resume(struct sys_device *dev)
{
	writeb(trout_int_mask[0], TROUT_CPLD_BASE + TROUT_GPIO_INT_MASK0_REG);
	writeb(trout_int_mask[1], TROUT_CPLD_BASE + TROUT_GPIO_INT_MASK1_REG);
	trout_suspended = 0;
	return 0;
}

static struct irq_chip trout_gpio_irq_chip = {
	.name      = "troutgpio",
	.ack       = trout_gpio_irq_ack,
	.mask      = trout_gpio_irq_mask,
	.unmask    = trout_gpio_irq_unmask,
	.set_wake  = trout_gpio_irq_set_wake,
	//.set_type  = trout_gpio_irq_set_type,
};

static struct gpio_chip trout_gpio_chip = {
	.start = TROUT_GPIO_START,
	.end = TROUT_GPIO_END,
	.configure = trout_gpio_configure,
	.get_irq_num = trout_gpio_get_irq_num,
	.read = trout_gpio_read,
	.write = trout_gpio_write,
//	.read_detect_status = trout_gpio_read_detect_status,
//	.clear_detect_status = trout_gpio_clear_detect_status
};

struct sysdev_class trout_sysdev_class = {
	.name = "troutgpio_irq",
	.suspend = trout_sysdev_suspend,
	.resume = trout_sysdev_resume,
};

static struct sys_device trout_irq_device = {
	.cls    = &trout_sysdev_class,
};

static int __init trout_init_gpio(void)
{
	int i;

	if (!machine_is_trout())
		return 0;

	/* adjust GPIOs based on bootloader request */
	pr_info("trout_init_gpio: cpld_usb_hw2_sw = %d\n", cpld_usb_h2w_sw);
	trout_gpio_write_shadow(TROUT_GPIO_USB_H2W_SW, cpld_usb_h2w_sw);

	for(i = 0; i < ARRAY_SIZE(trout_cpld_shadow); i++)
		writeb(trout_cpld_shadow[i], TROUT_CPLD_BASE + i * 2);

	for(i = TROUT_INT_START; i <= TROUT_INT_END; i++) {
		set_irq_chip(i, &trout_gpio_irq_chip);
		set_irq_handler(i, handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	register_gpio_chip(&trout_gpio_chip);

	set_irq_type(MSM_GPIO_TO_INT(17), IRQF_TRIGGER_HIGH);
	set_irq_chained_handler(MSM_GPIO_TO_INT(17), trout_gpio_irq_handler);
	set_irq_wake(MSM_GPIO_TO_INT(17), 1);

	if(sysdev_class_register(&trout_sysdev_class) == 0)
		sysdev_register(&trout_irq_device);

	return 0;
}

postcore_initcall(trout_init_gpio);

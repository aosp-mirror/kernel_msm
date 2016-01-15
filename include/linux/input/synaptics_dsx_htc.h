/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SYNAPTICS_DSX_H_
#define _SYNAPTICS_DSX_H_

#define PLATFORM_DRIVER_NAME "synaptics_dsx"
#define I2C_DRIVER_NAME "synaptics_dsx_i2c"
#define SPI_DRIVER_NAME "synaptics_dsx_spi"

/*
 * struct synaptics_dsx_cap_button_map - 0D button map
 * @nbuttons: number of 0D buttons
 * @map: pointer to array of button types
 */
struct synaptics_dsx_cap_button_map {
	unsigned char nbuttons;
	unsigned int *map;
};

#define SYN_CFG_BLK_UNIT	(16)
#define SYN_CONFIG_SIZE 	(128 * SYN_CFG_BLK_UNIT)

struct synaptics_rmi4_config {
	uint32_t sensor_id;
	uint32_t eng_id;
	uint32_t pr_number;
	uint8_t cover_setting_size;
	uint8_t cover_setting[10];
	uint8_t uncover_setting[10];
	uint8_t glove_mode_setting[10];
	uint16_t length;
	uint8_t  config[SYN_CONFIG_SIZE];
	const char *disp_panel;
};

/*
 * struct synaptics_dsx_board_data - DSX board data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @swap_axes: swap axes flag
 * @irq_gpio: attention interrupt GPIO
 * @irq_on_state: attention interrupt active state
 * @power_gpio: power switch GPIO
 * @power_on_state: power switch active state
 * @reset_gpio: reset GPIO
 * @reset_on_state: reset active state
 * @irq_flags: IRQ flags
 * @device_descriptor_addr: HID device descriptor address
 * @panel_x: x-axis resolution of display panel
 * @panel_y: y-axis resolution of display panel
 * @power_delay_ms: delay time to wait after powering up device
 * @reset_delay_ms: delay time to wait after resetting device
 * @reset_active_ms: reset active time
 * @byte_delay_us: delay time between two bytes of SPI data
 * @block_delay_us: delay time between two SPI transfers
 * @pwr_reg_name: pointer to name of regulator for power control
 * @bus_reg_name: pointer to name of regulator for bus pullup control
 * @cap_button_map: pointer to 0D button map
 */
struct synaptics_dsx_board_data {
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	int irq_gpio;
	int irq_on_state;
	int power_gpio;
	int power_gpio_1v8;
	int power_on_state;
	int reset_gpio;
	int reset_on_state;
	int switch_gpio;
	unsigned long irq_flags;
	unsigned short device_descriptor_addr;
	unsigned int panel_x;
	unsigned int panel_y;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
	unsigned int byte_delay_us;
	unsigned int block_delay_us;
	const char *pwr_reg_name;
	const char *bus_reg_name;
	struct synaptics_dsx_cap_button_map *cap_button_map;
	uint32_t eng_id_mask;
	uint32_t eng_id;
	uint16_t tw_pin_mask;
	uint32_t display_width;
	uint32_t display_height;
	uint8_t update_feature;
	uint8_t support_glove;
	uint8_t support_cover;
	uint32_t hall_block_touch_time;
	int config_num;
	struct synaptics_rmi4_config *config_table;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;
};

#endif

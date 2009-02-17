#ifndef ELAN_I2C_H
#define ELAN_I2C_H

#define ELAN_8232_I2C_NAME "elan-touch"

struct elan_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int (*power)(int on);
};

#endif


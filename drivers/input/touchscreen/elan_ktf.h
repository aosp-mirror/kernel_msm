#ifndef _LINUX_ELAN_KTF_H
#define _LINUX_ELAN_KTF_H

/* Quanta BU10SW, Stanley Tsao, 2015.12.04, Change for XU1 400x400 panel { */
#if 0
#define ELAN_X_MAX      1280
#define ELAN_Y_MAX      2112
#else
#define ELAN_X_MAX      1024
#define ELAN_Y_MAX      1024
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.12.04, Change for XU1 400x400 panel { */


#define L2500_ADDR			0x7bd0
#define EKTF2100_ADDR		0x7bd0
#define EKTF2200_ADDR		0x7bd0
#define EKTF3100_ADDR		0x7c16
#define FW_ADDR					L2500_ADDR

/* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function { */
#define ELAN_KTF_VTG_MIN_UV		2600000
#define ELAN_KTF_VTG_MAX_UV		3300000
#define ELAN_KTF_I2C_VTG_MIN_UV	1800000
#define ELAN_KTF_I2C_VTG_MAX_UV	1800000
/* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function } */


#define ELAN_KTF_NAME "elan_ktf"

struct elan_ktf_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.01, Add for device tree property { */
    u32 intr_gpio_flags;
    u32 rst_gpio_flags;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.01, Add for device tree property } */    
        int mode_check_gpio;
	int (*power)(int on);
};

#endif /* _LINUX_ELAN_KTF_H */

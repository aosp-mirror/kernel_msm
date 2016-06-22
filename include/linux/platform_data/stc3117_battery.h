/*
 *  Copyright (C) 2011 STMicroelectronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

int null_fn(void)
{
	return 0;                // for discharging status
}

int Temperature_fn(void)
{
	return (25);
}

#ifndef __STC3117_BATTERY_H_
#define __STC3117_BATTERY_H_

#define STC3117_DEV_NAME			"stc3117"

struct stc311x_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_enable)(void);
	int (*power_supply_register)(struct device *parent, struct power_supply *psy);
	void (*power_supply_unregister)(struct power_supply *psy);
	int (*ExternalTemperature) (void); /*External temperature fonction, return degrC*/

	int Vmode;       /* 1=Voltage mode, 0=mixed mode */
	int Alm_SOC;     /* SOC alm level %*/
	int Alm_Vbat;    /* Vbat alm level mV*/
	int CC_cnf;      /* nominal CC_cnf */
	int VM_cnf;      /* nominal VM cnf */
	int Rint;		 /*nominal Rint*/
	int Cnom;        /* nominal capacity in mAh */
	int Rsense;      /* sense resistor mOhms*/
	int RelaxCurrent; /* current for relaxation in mA (< C/20) */
	int Adaptive;     /* 1=Adaptive mode enabled, 0=Adaptive mode disabled */
	int CapDerating[7];   /* capacity derating in 0.1%, for temp = 60, 40, 25, 10,  0, -10, -20 degrC */
	int OCVValue[16];    /* OCV curve adjustment */
	int SOCValue[16];

	int ForceExternalTemperature; /* 1=External temperature, 0=STC3115 temperature */
};


static struct stc311x_platform_data default_stc3117_platform_data = {

	.battery_online = NULL,
	.charger_online = NULL, 		/* used in stc311x_get_status() */
	.charger_enable = NULL, 		/* used in stc311x_get_status() */
	.power_supply_register = NULL,
	.power_supply_unregister = NULL,

	//Gas gauge specific, Default:
	.Alm_SOC = 10,      /* SOC alm level %*/
	.Alm_Vbat = 3600,   /* Vbat alm level mV*/
	.CC_cnf = 66,      /* nominal CC_cnf, coming from battery characterisation*/
	.VM_cnf = 184,      /* nominal VM cnf , coming from battery characterisation*/
	.Rint = 501,		/* nominal internal impedance*/
	.Cnom = 340,       /* nominal capacity in mAh, coming from battery characterisation*/
	.Rsense = 10,       /* sense resistor mOhms*/
	.RelaxCurrent = 17, /* current for relaxation in mA (< C/20) */
	.Adaptive = 1,     /* 1=Adaptive mode enabled, 0=Adaptive mode disabled */

	/* Elentec Co Ltd Battery pack - 80 means 8% */
	.CapDerating[6] = 37,   /* capacity derating in 0.1%, for temp = -20°C */
	.CapDerating[5] = 37,   /* capacity derating in 0.1%, for temp = -10°C */
	.CapDerating[4] = 37,    /* capacity derating in 0.1%, for temp = 0°C */
	.CapDerating[3] = 10,  /* capacity derating in 0.1%, for temp = 10°C */
	.CapDerating[2] = 0,  /* capacity derating in 0.1%, for temp = 25°C */
	.CapDerating[1] = 0,  /* capacity derating in 0.1%, for temp = 40°C */
	.CapDerating[0] = 0,  /* capacity derating in 0.1%, for temp = 60°C */

	.OCVValue[0] = 3400,             /* OCV curve adjustment */
	.OCVValue[1] = 3605,             /* OCV curve adjustment */
	.OCVValue[2] = 3672,             /* OCV curve adjustment */
	.OCVValue[3] = 3687,             /* OCV curve adjustment */
	.OCVValue[4] = 3704,             /* OCV curve adjustment */
	.OCVValue[5] = 3734,             /* OCV curve adjustment */
	.OCVValue[6] = 3755,             /* OCV curve adjustment */
	.OCVValue[7] = 3779,             /* OCV curve adjustment */
	.OCVValue[8] = 3818,             /* OCV curve adjustment */
	.OCVValue[9] = 3866,             /* OCV curve adjustment */
	.OCVValue[10] = 3947,            /* OCV curve adjustment */
	.OCVValue[11] = 3999,            /* OCV curve adjustment */
	.OCVValue[12] = 4053,            /* OCV curve adjustment */
	.OCVValue[13] = 4160,            /* OCV curve adjustment */
	.OCVValue[14] = 4277,            /* OCV curve adjustment */
	.OCVValue[15] = 4388,            /* OCV curve adjustment */

	.SOCValue[0]  = 0x0,
	.SOCValue[1]  = 0x6,
	.SOCValue[2]  = 0x0C,
	.SOCValue[3]  = 0x14,
	.SOCValue[4]  = 0x1E,
	.SOCValue[5]  = 0x28,
	.SOCValue[6]  = 0x32,
	.SOCValue[7]  = 0x3C,
	.SOCValue[8]  = 0x50,
	.SOCValue[9]  = 0x64,
	.SOCValue[10] = 0x78,
	.SOCValue[11] = 0x82,
	.SOCValue[12] = 0x8C,
	.SOCValue[13] = 0xA0,
	.SOCValue[14] = 0xB4,
	.SOCValue[15] = 0xC8,

	/*if the application temperature data is preferred than the STC3117 temperature*/
	.ExternalTemperature = Temperature_fn, /*External temperature fonction, return °C*/
	.ForceExternalTemperature = 0, /* 1=External temperature, 0=STC3117 temperature */
};

#endif

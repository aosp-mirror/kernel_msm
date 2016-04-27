/*
 *  Copyright (C) 2011 STMicroelectronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __STC3117_BATTERY_H_
#define __STC3117_BATTERY_H_

#define OCVTAB_SIZE                      16      /* OCVTAB size of STC311x */
#define SOCTAB_SIZE                      16      /* SOCTAB size of STC311x */

struct stc311x_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_enable)(void);
	int (*power_supply_register)(struct device *parent,
				     struct power_supply *psy);
	void (*power_supply_unregister)(struct power_supply *psy);

	/* 1=Voltage mode, 0=mixed mode */
	int Vmode;
	/* SOC alm level %*/
	int Alm_SOC;
	/* Vbat alm level mV*/
	int Alm_Vbat;
	/* nominal CC_cnf */
	int CC_cnf;
	/* nominal VM cnf */
	int VM_cnf;
	/*nominal Rint*/
	int Rint;
	/* nominal capacity in mAh */
	int Cnom;
	/* sense resistor mOhms*/
	int Rsense;
	/* current for relaxation in mA (< C/20) */
	int RelaxCurrent;
	/* 1=Adaptive mode enabled, 0=Adaptive mode disabled */
	int Adaptive;
	/* capacity derating in 0.1%, for temp = 60, 40, 25, 10,   0, -10
	 * °C,-20°C */
	int CapDerating[7];
	/* OCV curve adjustment */
	int OCVValue[OCVTAB_SIZE];
	/* SOC curve adjustment */
	int SOCValue[SOCTAB_SIZE];
	/*External temperature fonction, return °C*/
	int (*ExternalTemperature)(void);
	/* 1=External temperature, 0=STC3115 temperature */
	int ForceExternalTemperature;
};

/*Function declaration*/
extern int STC31xx_SetPowerSavingMode(void);
extern int STC31xx_StopPowerSavingMode(void);
extern int STC31xx_AlarmSet(void);
extern int STC31xx_AlarmStop(void);
extern int STC31xx_AlarmGet(void);
extern int STC31xx_AlarmClear(void);
extern int STC31xx_AlarmSetVoltageThreshold(int VoltThresh);
extern int STC31xx_AlarmSetSOCThreshold(int SOCThresh);
extern int STC31xx_RelaxTmrSet(int CurrentThreshold);
extern int stc311x_updata(void);

#endif

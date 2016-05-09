#if CONFIG_BATTERY_STC3117
int null_fn(void)
{
	return 0;                // for discharging status
}

int Temperature_fn(void)
{
	return (25);
}

static struct stc311x_platform_data stc3117_data = {
		.battery_online = NULL,
		.charger_online = null_fn, 		// used in stc311x_get_status()
		.charger_enable = null_fn,		// used in stc311x_get_status()
		.power_supply_register = NULL,
		.power_supply_unregister = NULL,

		.Vmode= 0,       /*REG_MODE, BIT_VMODE 1=Voltage mode, 0=mixed mode */
		.Alm_SOC = 10,      /* SOC alm level %*/
		.Alm_Vbat = 3600,   /* Vbat alm level mV*/
		.CC_cnf = 59,      /* nominal CC_cnf, coming from battery characterisation*/
		.VM_cnf = 271,      /* nominal VM cnf , coming from battery characterisation*/
		.Rint = 330,			/* nominal internal impedance*/
		.Cnom = 300,       /* nominal capacity in mAh, coming from battery characterisation*/
		.Rsense = 10,       /* sense resistor mOhms*/
		.RelaxCurrent = 150, /* current for relaxation in mA (< C/20) */
		.Adaptive = 1,     /* 1=Adaptive mode enabled, 0=Adaptive mode disabled */

		/* Elentec Co Ltd Battery pack - 80 means 8% */
	.CapDerating[6] = -5,   /* capacity derating in 0.1%, for temp = -20°C */
	.CapDerating[5] = -5,   /* capacity derating in 0.1%, for temp = -10°C */
	.CapDerating[4] = -5,    /* capacity derating in 0.1%, for temp = 0°C */
	.CapDerating[3] = -5,  /* capacity derating in 0.1%, for temp = 10°C */
	.CapDerating[2] = -5,  /* capacity derating in 0.1%, for temp = 25°C */
	.CapDerating[1] = 0,  /* capacity derating in 0.1%, for temp = 40°C */
	.CapDerating[0] = 0,  /* capacity derating in 0.1%, for temp = 60°C */

		//OCV curve example for a 4.35V li-ion battery
	.OCVValue[0] = 3300,             /* OCV curve adjustment */
	.OCVValue[1] = 3559,             /* OCV curve adjustment */
	.OCVValue[2] = 3671,             /* OCV curve adjustment */
	.OCVValue[3] = 3684,             /* OCV curve adjustment */
	.OCVValue[4] = 3702,             /* OCV curve adjustment */
	.OCVValue[5] = 3730,             /* OCV curve adjustment */
	.OCVValue[6] = 3747,             /* OCV curve adjustment */
	.OCVValue[7] = 3765,             /* OCV curve adjustment */
	.OCVValue[8] = 3795,             /* OCV curve adjustment */
	.OCVValue[9] = 3832,             /* OCV curve adjustment */
	.OCVValue[10] = 3899,            /* OCV curve adjustment */
	.OCVValue[11] = 3941,            /* OCV curve adjustment */
	.OCVValue[12] = 3967,            /* OCV curve adjustment */
	.OCVValue[13] = 4087,            /* OCV curve adjustment */
	.OCVValue[14] = 4181,            /* OCV curve adjustment */
	.OCVValue[15] = 4297,            /* OCV curve adjustment */

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


static struct i2c_board_info __initdata beagle_i2c2_boardinfo[] = {

	#if CONFIG_BATTERY_STC3117
	{
		I2C_BOARD_INFO("stc3117", 0x70),
		.platform_data = &stc3117_data,
	},
	#endif

};

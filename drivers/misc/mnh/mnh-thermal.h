#ifndef __MNH_THERMAL_H__
#define __MNH_THERMAL_H__

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>

#define MNH_NUM_PVT_SENSORS 4

/* Thermal characterization data */
#define API_TRIM_CODE		0xF
#define MNH_PRECISION		0x1

/* Thermal efuse data */
#define NUM_BITS_EFUSE	10
#define API_BITS_SLOPE	5
#define API_BITS_OFFSET	(NUM_BITS_EFUSE-API_BITS_SLOPE)

/* Temperature calculation algorithm parameters */
#define API_POLY_N4 (-16743)  /* -1.6743e-11 : 15bits excluding sign */
#define API_POLY_N3 (+81542)  /* +8.1542e-08 : 17bits excluding sign */
#define API_POLY_N2 (-18201)  /* -1.8201e-04 : 15bits excluding sign */
#define API_POLY_N1 (+31020)  /* +3.1020e-01 : 15bits excluding sign */
#define API_POLY_N0 (-48380)  /* -4.8380e+01 : 16bits excluding sign */

#define N3_E15_MULTIPLIER 1000		/* 1e3*/
#define N2_E15_MULTIPLIER 10000000	/* 1e7 */
#define N1_E15_MULTIPLIER 10000000000	/* 1e10 */
#define N0_E15_MULTIPLIER 1000000000000	/* 1e12 */

#define API_RES_SLOPE  78125	/* 0.0078125 */
#define API_RES_OFFSET 5	/* 0.5 */

#define RES_SLOPE_DIVIDER 10000000 /* 1e7 */
#define RES_OFFSET_E15_MULTIPLIER 100000000000000 /* 1e14 */

#define API_BITS_SLOPE_MASK    0x1F
#define API_BITS_OFFSET_MASK   0x1F

#define N12_DIVIDER  1000000000000	/* 1e12 */
#define N12_ROUNDING_NUM 500000000000	/* 5e11 */

/* PVT Data conversion wait time calculation
 * PVT_FREQ: 1.15MHZ
 * PVT_WAIT_CYCLE: 376 + marginal cycles
 * PVT_WAIT_TIME: ((1/PVT_FREQ)*PVT_WAIT_CYCLE*PRECISION)
 * Average samples: (1<<(PRECISION*2))
 * PVT data conversion time:
 *   temp/vol mode: 376 PVT_SENSOR_CLK cycles
 *   process mode: 4 PVT_SENSOR_CLK cycles
 */
#define PVT_FREQ_KHZ		1150
#define PVT_FREQ_MS_UNIT	1000000
#define PVT_FREQ_US_UNIT	1000
#define PVT_CYCLE	(PVT_FREQ_MS_UNIT/PVT_FREQ_KHZ)
#define PVT_WAIT_CYCLE	390
#define PVT_AVG_SAMPLES(p)	(1<<(p*2))
#define PVT_WAIT_MS(p)	(((PVT_CYCLE*PVT_WAIT_CYCLE*PVT_AVG_SAMPLES(p)) \
			/PVT_FREQ_MS_UNIT)+1)
#define PVT_WAIT_US(p)	((PVT_CYCLE*PVT_WAIT_CYCLE*PVT_AVG_SAMPLES(p)) \
			/PVT_FREQ_US_UNIT)

struct mnh_thermal_device {
	struct platform_device *pdev;
	struct device *dev;
	struct reset_control *reset;
	uint32_t regs;
	uint32_t wait_time_us;
	uint32_t mnh_trim_init_done;
	struct mnh_thermal_sensor *sensors[MNH_NUM_PVT_SENSORS];
};

struct mnh_thermal_sensor {
	struct mnh_thermal_device *mnh_dev;
	struct thermal_zone_device *tzd;
	uint32_t id;
	uint32_t slope;
	uint32_t offset;
	uint32_t alarm_temp;
};

/* PVT Sensor operation mode */
enum mnh_thermal_op_mode {
	THERMAL_OPMODE_TEMP, /* Temperature evaluation */
	THERMAL_OPMODE_VOL,  /* Voltage evaluation */
	THERMAL_OPMODE_LVT,  /* Process LVT evaluation */
	THERMAL_OPMODE_HVT,  /* Process HVT evaluation */
	THERMAL_OPMODE_SVT,  /* Process SVT evaluation */
	THERMAL_OPMODE_MAX = THERMAL_OPMODE_SVT
};

struct mnh_thermal_op_param {
	int vsample;
	int psample;
};

struct mnh_thermal_efuse_param {
	uint8_t row1;
	uint8_t row2;
	uint8_t bh;
	uint8_t bl;
};

#endif /* __MNH_THERMAL_H__ */

/*
 * Definitions for akm8976 compass chip.
 */
#ifndef AKM8976_H
#define AKM8976_H

#include <linux/ioctl.h>

/* Compass device dependent definition */
#define AKECS_MODE_MEASURE	0x00	/* Starts measurement. Please use AKECS_MODE_MEASURE_SNG */
					/* or AKECS_MODE_MEASURE_SEQ instead of this. */
#define AKECS_MODE_PFFD		0x01	/* Start pedometer and free fall detect. */
#define AKECS_MODE_E2P_READ	0x02	/* E2P access mode (read). */
#define AKECS_MODE_POWERDOWN	0x03	/* Power down mode */

#define AKECS_MODE_MEASURE_SNG	0x10	/* Starts single measurement */
#define AKECS_MODE_MEASURE_SEQ	0x11	/* Starts sequential measurement */

/* Default register settings */
#define CSPEC_AINT		0x01	/* Amplification for acceleration sensor */
#define CSPEC_SNG_NUM		0x01	/* Single measurement mode */
#define CSPEC_SEQ_NUM		0x02	/* Sequential measurement mode */
#define CSPEC_SFRQ_32		0x00	/* Measurement frequency: 32Hz */
#define CSPEC_SFRQ_64		0x01	/* Measurement frequency: 64Hz */
#define CSPEC_MCS		0x07	/* Clock frequency */
#define CSPEC_MKS		0x01	/* Clock type: CMOS level */
#define CSPEC_INTEN		0x01	/* Interruption pin enable: Enable */

#define RBUFF_SIZE		31	/* Rx buffer size */
#define MAX_CALI_SIZE	0x1000U	/* calibration buffer size */

/* AK8976A register address */
#define AKECS_REG_ST			0xC0
#define AKECS_REG_TMPS			0xC1
#define AKECS_REG_MS1			0xE0
#define AKECS_REG_MS2			0xE1
#define AKECS_REG_MS3			0xE2

#define AKMIO				0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_INIT                  _IO(AKMIO, 0x01)
#define ECS_IOCTL_WRITE                 _IOW(AKMIO, 0x02, char[5])
#define ECS_IOCTL_READ                  _IOWR(AKMIO, 0x03, char[5])
#define ECS_IOCTL_RESET      	          _IO(AKMIO, 0x04)
#define ECS_IOCTL_INT_STATUS            _IO(AKMIO, 0x05)
#define ECS_IOCTL_FFD_STATUS            _IO(AKMIO, 0x06)
#define ECS_IOCTL_SET_MODE              _IOW(AKMIO, 0x07, short)
#define ECS_IOCTL_GETDATA               _IOR(AKMIO, 0x08, char[RBUFF_SIZE+1])
#define ECS_IOCTL_GET_NUMFRQ            _IOR(AKMIO, 0x09, char[2])
#define ECS_IOCTL_SET_PERST             _IO(AKMIO, 0x0A)
#define ECS_IOCTL_SET_G0RST             _IO(AKMIO, 0x0B)
#define ECS_IOCTL_SET_YPR               _IOW(AKMIO, 0x0C, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS       _IOR(AKMIO, 0x0D, int)
#define ECS_IOCTL_GET_CLOSE_STATUS      _IOR(AKMIO, 0x0E, int)
#define ECS_IOCTL_GET_CALI_DATA         _IOR(AKMIO, 0x0F, char[MAX_CALI_SIZE])
#define ECS_IOCTL_GET_DELAY             _IOR(AKMIO, 0x30, short)

/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_MODE		_IOW(AKMIO, 0x10, short)
#define ECS_IOCTL_APP_SET_MFLAG		_IOW(AKMIO, 0x11, short)
#define ECS_IOCTL_APP_GET_MFLAG		_IOW(AKMIO, 0x12, short)
#define ECS_IOCTL_APP_SET_AFLAG		_IOW(AKMIO, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG		_IOR(AKMIO, 0x14, short)
#define ECS_IOCTL_APP_SET_TFLAG		_IOR(AKMIO, 0x15, short)
#define ECS_IOCTL_APP_GET_TFLAG		_IOR(AKMIO, 0x16, short)
#define ECS_IOCTL_APP_RESET_PEDOMETER   _IO(AKMIO, 0x17)
#define ECS_IOCTL_APP_SET_DELAY		_IOW(AKMIO, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY		ECS_IOCTL_GET_DELAY
#define ECS_IOCTL_APP_SET_MVFLAG	_IOW(AKMIO, 0x19, short)	/* Set raw magnetic vector flag */
#define ECS_IOCTL_APP_GET_MVFLAG	_IOR(AKMIO, 0x1A, short)	/* Get raw magnetic vector flag */

/* IOCTLs for pedometer */
#define ECS_IOCTL_SET_STEP_CNT          _IOW(AKMIO, 0x20, short)

/* Default GPIO setting */
#define ECS_RST		146	/*MISC4, bit2 */
#define ECS_CLK_ON	155	/*MISC5, bit3 */
#define ECS_INTR	161	/*INT2, bit1 */

struct akm8976_platform_data {
	int reset;
	int clk_on;
	int intr;
};

extern char *get_akm_cal_ram(void);

#endif


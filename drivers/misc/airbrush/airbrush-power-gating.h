#ifndef _AIRBRUSH_POWER_GATING_H_
#define _AIRBRUSH_POWER_GATING_H_

#include<linux/airbrush-sm-ctrl.h>

int ab_tpu_tiles_on(void);
int ab_tpu_tiles_off(void);
int ab_ipu_cores_on(void);
int ab_ipu_cores_off(void);

#endif //_AIRBRUSH_POWER_GATING_H_

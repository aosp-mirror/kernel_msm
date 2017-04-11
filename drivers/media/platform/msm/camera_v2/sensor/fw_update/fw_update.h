/* OIS calibration interface for LC898123 F40
 *
 */
#include	"PhoneUpdate.h"
#include	"../msm_sensor.h"
#include	"../actuator/msm_actuator.h"

void RamWrite32A(UINT_16 RamAddr, UINT_32 RamData);
void RamRead32A(UINT_16 RamAddr, UINT_32 * ReadData);
int CntWrt(UINT_8 * PcSetDat, UINT_16 CntWrt);
int CntRd3(UINT_32 addr, void *	PcSetDat, UINT_16	UsDatNum);
void WitTim(UINT_16) ;
void WPBCtrl(UINT_8 UcCtrl);
int checkHWFWversion(void);
int checkFWUpdate(struct msm_sensor_ctrl_t *s_ctrl);
int checkVCMFWUpdate(struct msm_sensor_ctrl_t *s_ctrl);

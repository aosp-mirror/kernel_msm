/* OIS calibration interface for LC898123 F40
 *
 */
#include "PhoneUpdate.h"
#include "../cam_sensor/cam_sensor_dev.h"

void RamWrite32A(UINT_16 RamAddr, UINT_32 RamData);
void RamRead32A(UINT_16 RamAddr, UINT_32 *ReadData);
int RamWrite8A(struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 RamData);
int RamRead8A(struct camera_io_master *io_info,
	UINT_32 RamAddr, UINT_32 *ReadData);
int CntWrt(UINT_8 *PcSetDat, UINT_16 CntWrt);
int CntRd3(UINT_32 addr, void *PcSetDat, UINT_16 UsDatNum);
void WitTim(UINT_16) ;
void WPBCtrl(UINT_8 UcCtrl);
bool checkOISFWversion(UINT_16 *cal_id, UINT_32 *module_maker);
int checkOISFWUpdate(struct cam_sensor_ctrl_t *s_ctrl);
int checkRearVCMFWUpdate(struct cam_sensor_ctrl_t *s_ctrl);
int DownloadRearVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable,
	UINT_32 tbsize);
int ValidateRearVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable,
	UINT_32 tbsize);
int checkFrontVCMFWUpdate(struct cam_sensor_ctrl_t *s_ctrl);
int DownloadFrontVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable,
	UINT_32 tbsize);
int ValidateFrontVCMFW(struct camera_io_master *io_info,
	struct cam_sensor_i2c_reg_array *fwtable,
	UINT_32 tbsize);

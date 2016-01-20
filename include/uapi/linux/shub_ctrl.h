#ifndef __SHUB_CTRL_H__
#define __SHUB_CTRL_H__

#define SHUB_DLOAD_SUPPORT 0
#define SHUB_LOGGING_SUPPORT 0
#define SHUB_EVENT_SUPPORT 0
#define SHUB_FIRMWARE_UPDATE_SUPPORT 1

#ifdef SHUB_DLOAD_SUPPORT
#define SHUB_DLOAD_DEVICE_NAME                 "shub_dload"
#define SHUB_DLOAD_IOCTL_CODE                  (0xAA)
#define SHUB_DLOAD_IOCTL_WAIT_FOR_NOTIFY       _IOR(SHUB_DLOAD_IOCTL_CODE, 1, uint32_t)
#define SHUB_DLOAD_IOCTL_GET_RAMDUMP_SIZE      _IOR(SHUB_DLOAD_IOCTL_CODE, 2, uint32_t)
#define SHUB_DLOAD_IOCTL_RAMDUMP_START         _IO(SHUB_DLOAD_IOCTL_CODE, 3)
#define SHUB_DLOAD_IOCTL_RAMDUMP_DONE          _IO(SHUB_DLOAD_IOCTL_CODE, 4)
#define SHUB_DLOAD_IOCTL_ENABLE_DLOAD          _IOW(SHUB_DLOAD_IOCTL_CODE, 5, uint8_t)
#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_LOGGING_SUPPORT
#define SHUB_LOG_DEVICE_NAME                   "shub_log"
#define SHUB_LOG_IOCTL_CODE                    (0xBB)
#define SHUB_LOG_IOCTL_WAIT_FOR_NOTIFY         _IOR(SHUB_LOG_IOCTL_CODE, 1, uint32_t)
#define SHUB_LOG_IOCTL_GET_LOG_SIZE            _IOR(SHUB_LOG_IOCTL_CODE, 2, uint32_t)
#define SHUB_LOG_IOCTL_GET_LOG_START           _IO(SHUB_LOG_IOCTL_CODE, 3)
#define SHUB_LOG_IOCTL_GET_LOG_DONE            _IO(SHUB_LOG_IOCTL_CODE, 4)
#define SHUB_LOG_IOCTL_SET_LOGMASK             _IOW(SHUB_LOG_IOCTL_CODE, 5, uint32_t)
#define SHUB_LOG_IOCTL_GET_LOGMASK             _IOR(SHUB_LOG_IOCTL_CODE, 6, uint32_t)
#define SHUB_LOG_IOCTL_SET_LOGLEVEL            _IOW(SHUB_LOG_IOCTL_CODE, 7, uint32_t)
#define SHUB_LOG_IOCTL_GET_LOGLEVEL            _IOR(SHUB_LOG_IOCTL_CODE, 8, uint32_t)
#endif //SHUB_LOGGING_SUPPORT

#ifdef SHUB_EVENT_SUPPORT
#define SHUB_EVENT_DEVICE_NAME                   "shub_event"
#define SHUB_EVENT_IOCTL_CODE                    (0xDD)
#define SHUB_EVENT_IOCTL_WAIT_FOR_NOTIFY         _IOR(SHUB_EVENT_IOCTL_CODE, 1, uint32_t)
#define SHUB_EVENT_IOCTL_GET_EVENT_SIZE          _IOR(SHUB_EVENT_IOCTL_CODE, 2, uint32_t)
#define SHUB_EVENT_IOCTL_GET_EVENT_START         _IO(SHUB_EVENT_IOCTL_CODE, 3)
#define SHUB_EVENT_IOCTL_GET_EVENT_DONE          _IO(SHUB_EVENT_IOCTL_CODE, 4)
#endif //SHUB_EVENT_SUPPORT


#ifdef SHUB_FIRMWARE_UPDATE_SUPPORT
typedef struct {
    uint8_t arch;
    uint8_t sense;
    uint8_t cw_lib;
    uint8_t water;
    uint8_t active_engine;
    uint8_t project_mapping;
} mcu_fw_version_t;

typedef struct {
    uint8_t jenkins_num_hi;
    uint8_t jenkins_num_lo;
    uint8_t build_time_hh;
    uint8_t build_time_mm;
    uint8_t cw_branch;
    uint8_t cw_mcu_type;
} mcu_fw_info_t;

#define SHUB_FIRMWARE_UPDATE_DEVICE_NAME        "shub_fw_fla"
#define SHUB_FW_IOCTL_CODE                      (0xCC)
#define SHUB_FW_IOCTL_PRE_FLASH                 _IO(SHUB_FW_IOCTL_CODE, 1)
#define SHUB_FW_IOCTL_POST_FLASH                _IO(SHUB_FW_IOCTL_CODE, 2)
#define SHUB_FW_IOCTL_GET_FW_VERSION            _IOR(SHUB_FW_IOCTL_CODE, 3, mcu_fw_version_t)
#define SHUB_FW_IOCTL_GET_FW_CHECKSUM           _IOR(SHUB_FW_IOCTL_CODE, 4, uint32_t)
#define SHUB_FW_IOCTL_GET_FW_INFO               _IOR(SHUB_FW_IOCTL_CODE, 5, mcu_fw_info_t)
#define SHUB_FW_IOCTL_START_FW_CHECKSUM         _IOW(SHUB_FW_IOCTL_CODE, 6, uint32_t)
#endif //SHUB_FIRMWARE_UPDATE_SUPPORT

#endif /* __SHUB_CTRL_H__ */


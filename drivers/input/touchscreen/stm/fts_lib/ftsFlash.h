/*

**************************************************************************
**                        STMicroelectronics							**
**************************************************************************
**                        marco.cali@st.com								**
**************************************************************************
*                                                                        *
*			FTS API for Flashing the IC							 *
*                                                                        *
**************************************************************************
**************************************************************************

*/

/*!
* \file ftsFlash.h
* \brief Contains all the definitions and structs to handle the FW update process
*/

#ifndef FTS_FLASH_H
#define FTS_FLASH_H

#include "ftsSoftware.h"

//Flash possible status
#define FLASH_READY						0										///< value to indicate that the flash is ready
#define FLASH_BUSY						1										///< value to indicate that the flash is busy
#define FLASH_UNKNOWN					-1										///< value to indicate an unknown status of the flash

#define FLASH_STATUS_BYTES				1										///< number of bytes to check for read the flash status



//Flash timing parameters
#define FLASH_RETRY_COUNT				200										///< number of attemps to read the flash status
#define FLASH_WAIT_BEFORE_RETRY         50										///< time to wait in ms between one status reading and another


#ifdef FW_H_FILE
#define PATH_FILE_FW	"NULL"
#else
#ifdef FTM3_CHIP
#define PATH_FILE_FW	"st_fts.bin"	/* FW bin file name */
#else
#define PATH_FILE_FW	"ftm5_fw.ftb"	/* new FW bin file name */
#endif
#endif

#ifndef FTM3_CHIP
#ifdef FTI
#define FLASH_CHUNK			64*1024												///< Max number of bytes that the DMA can burn on the flash in one shot in FTI
#define DMA_CHUNK			32													///< Max number of bytes that can be written in I2C to the DMA
#else
#define FLASH_CHUNK			64*1024												///< Max number of bytes that the DMA can burn on the flash in one shot
#define DMA_CHUNK			2*1024												///< Max number of bytes that can be written in I2C to the DMA
#endif
#endif

/**
 * Define which kind of erase age by page should be performed
 */
typedef enum{
	ERASE_ALL =0,																///< erase all the pages
	SKIP_PANEL_INIT =1,															///< skip erase Panel Init Pages
	SKIP_PANEL_CX_INIT = 2														///< skip erase Panel Init and CX Pages
}ErasePage;

/** @addtogroup fw_file
 * @{
 */

/**
* Struct which contains information and data of the FW that should be burnt into the IC
*/
typedef struct {
	u8* data;																	///< pointer to an array of bytes which represent the FW data
	u16 fw_ver;																	///< FW version of the FW file
	u16 config_id;																///< Config ID of the FW file
	u16 cx_ver;																	///< Cx version of the FW file
	u8 externalRelease[EXTERNAL_RELEASE_INFO_SIZE];								///< External Release Info of the FW file
	int data_size;																///< dimension of data (the actual data to be burnt)
#ifndef FTM3_CHIP
	u32 sec0_size;																///< dimension of section 0 (FW) in .ftb file
	u32 sec1_size;																///< dimension of section 1 (Config) in .ftb file
	u32 sec2_size;																///< dimension of section 2 (Cx) in .ftb file
	u32 sec3_size;																///< dimension of section 3 (TBD) in .ftb file
#endif
} Firmware;

/** @}*/

/** @addtogroup flash_command
 * @{
 */
#ifdef FTM3_CHIP
int flash_status(void);
int flash_status_ready(void);
int wait_for_flash_ready(void);
int fillMemory(u32 address, u8 * data, int size);
#else
int wait_for_flash_ready(u8 type);
#ifndef FTI
int fts_warm_boot(void);
#else
int hold_m3(void);
#ifdef SONIA
int flash_disable_remapping(void);
#endif
#endif
int flash_erase_unlock(void);
int flash_full_erase(void);
#ifndef FTI
int flash_erase_page_by_page(int keep_cx);
#else
int flash_erase_page_by_page(ErasePage keep_cx);
#endif
#endif

int flash_unlock(void);
int getFirmwareVersion(u16 * fw_vers, u16* config_id);
int getFWdata(const char* pathToFile, u8** data, int *size);
int parseBinFile(u8* fw_data, int fw_size, Firmware *fw, int keep_cx);
int readFwFile(const char* path, Firmware *fw, int keep_cx);
int flash_burn(Firmware fw, int force_burn, int keep_cx);
int flashProcedure(const char* path, int force, int keep_cx);

#endif

/** @}*/

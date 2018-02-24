/*

 **************************************************************************
 **                        STMicroelectronics                            **
 **************************************************************************
 **                        marco.cali@st.com                             **
 **************************************************************************
 *                                                                        *
 *                      FTS API for Flashing the IC                       *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */


/*!
* \file ftsFlash.c
* \brief Contains all the functions to handle the FW update process
*/

#include "ftsCore.h"
#include "ftsCompensation.h"
#include "ftsError.h"
#include "ftsFlash.h"
#include "ftsFrame.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"
#include "../fts.h"					//needed for including the define FW_H_FILE


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>


#ifdef FW_H_FILE
#include "../fts_fw.h"
#endif


extern SysInfo systemInfo;														///< forward declaration of the global variable of containing System Info Data


/**
* Retrieve the actual FW data from the system (bin file or header file)
* @param pathToFile name of FW file to load or "NULL" if the FW data should be loaded by a .h file
* @param data pointer to the pointer which will contains the FW data
* @param size pointer to a variable which will contain the size of the loaded data
* @return OK if success or an error code which specify the type of error encountered
*/
int getFWdata(const char* pathToFile, u8** data, int *size){
	const struct firmware *fw = NULL;
	struct device *dev = NULL;
	int res,from=0;
	char* path = (char*)pathToFile;

	logError(1, "%s getFWdata starting ...\n", tag);
	if(strncmp(pathToFile,"NULL",4)==0){
		from =1;
		path = PATH_FILE_FW;
	}
	//keep the switch case because if the argument passed is null but the option from .h is not set we still try to load from bin
	switch(from){
#ifdef FW_H_FILE
		case 1:
			logError(1, "%s Read FW from .h file!\n", tag);
			*size = FW_SIZE_NAME;
			*data = (u8*) kmalloc((*size) * sizeof (u8), GFP_KERNEL);
			if(*data==NULL){
				logError(1, "%s getFWdata: Impossible to allocate memory! ERROR %08X\n", tag, ERROR_ALLOC);
				return ERROR_ALLOC;
			}
			memcpy(*data, (u8 *) FW_ARRAY_NAME , (*size));

		break;
#endif
		default:
			logError(1, "%s Read FW from BIN file %s !\n", tag, path);
			dev = getDev();

			if (dev != NULL){
				res = request_firmware(&fw, path, dev);
				if(res == 0){
					*size = fw->size;
					*data = (u8*) kmalloc((*size) * sizeof (u8), GFP_KERNEL);
					if(*data==NULL){
						logError(1, "%s getFWdata: Impossible to allocate memory! ERROR %08X\n", tag, ERROR_ALLOC);
						release_firmware(fw);
						return ERROR_ALLOC;
					}
					memcpy(*data, (u8 *) fw->data , (*size));
					release_firmware(fw);
				}else{
					logError(1, "%s getFWdata: No File found! ERROR %08X\n", tag, ERROR_FILE_NOT_FOUND);
					return ERROR_FILE_NOT_FOUND;
				}

			}else {
				logError(1, "%s getFWdata: No device found! ERROR %08X\n", tag, ERROR_OP_NOT_ALLOW);
				return ERROR_OP_NOT_ALLOW;
			}

	}

	logError(1, "%s getFWdata Finished!\n", tag);
	return OK;


}


/**
* Perform all the steps to read the FW that should be burnt in the IC from the system and parse it in order to fill a Firmware struct with the relevant info
* @param path name of FW file to load or "NULL" if the FW data should be loaded by a .h file
* @param fw pointer to a Firmware variable which will contains the FW data and info
* @param keep_cx if 1, the CX area will be loaded otherwise will be skipped
* @return OK if success or an error code which specify the type of error encountered
*/
int readFwFile(const char* path, Firmware *fw, int keep_cx) {
    int res;
    int orig_size;
    u8* orig_data = NULL;


    res = getFWdata(path, &orig_data, &orig_size);
    if (res < OK) {
        logError(1, "%s readFwFile: impossible retrieve FW... ERROR %08X\n", tag, ERROR_MEMH_READ);
        return (res | ERROR_MEMH_READ);
    }
    res = parseBinFile(orig_data, orig_size, fw, keep_cx);
    if (res < OK) {
        logError(1, "%s readFwFile: impossible parse ERROR %08X\n", tag, ERROR_MEMH_READ);
        return (res | ERROR_MEMH_READ);
    }

    return OK;

}

/**
* Perform all the steps necessary to burn the FW into the IC
* @param path name of FW file to load or "NULL" if the FW data should be loaded by a .h file
* @param force if 1, the flashing procedure will be forced and executed regardless the additional info, otherwise the FW in the file will be burnt only if it is newer than the one running in the IC
* @param keep_cx if 1, the CX area will be loaded and burnt otherwise will be skipped and the area will be untouched
* @return OK if success or an error code which specify the type of error encountered
*/
int flashProcedure(const char* path, int force, int keep_cx) {
    Firmware fw;
    int res;

    fw.data = NULL;
    logError(0, "%s Reading Fw file... \n", tag);
    res = readFwFile(path, &fw, keep_cx);
    if (res < OK) {
        logError(1, "%s flashProcedure: ERROR %08X \n", tag, (res | ERROR_FLASH_PROCEDURE));
	kfree(fw.data);
        return (res | ERROR_FLASH_PROCEDURE);
    }
    logError(0, "%s Fw file read COMPLETED! \n", tag);

    logError(0, "%s Starting flashing procedure... \n", tag);
    res = flash_burn(fw, force, keep_cx);
    if (res < OK && res != (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)) {
        logError(1, "%s flashProcedure: ERROR %08X \n", tag, ERROR_FLASH_PROCEDURE);
		kfree(fw.data);
        return (res | ERROR_FLASH_PROCEDURE);
    }
    logError(0, "%s flashing procedure Finished!\n", tag);
    kfree(fw.data);

    return res;
}

/**
* Poll the Flash Status Registers after the execution of a command to check if the Flash becomes ready within a timeout
* @param type register to check according to the previous command sent
* @return OK if success or an error code which specify the type of error encountered
*/
int wait_for_flash_ready(u8 type) {

    u8 cmd[5] = {FTS_CMD_HW_REG_R, 0x20, 0x00, 0x00, type};

    u8 readData[2] ={0};
    int i, res = -1;

    logError(0, "%s Waiting for flash ready ... \n", tag);
    for (i = 0; i < FLASH_RETRY_COUNT && res != 0; i++) {
		res = fts_writeRead(cmd, ARRAY_SIZE(cmd), readData, 2);
        if (res < OK) {
            logError(1, "%s wait_for_flash_ready: ERROR % 08X\n", tag, ERROR_BUS_W);
        } else{
#ifdef I2C_INTERFACE  //in case of spi there is a dummy byte
			res = readData[0] & 0x80;
#else
			res = readData[1] & 0x80;
#endif

			logError(0, "%s flash status = %d  \n", tag, res);
		}
        mdelay(FLASH_WAIT_BEFORE_RETRY);
    }

    if (i == FLASH_RETRY_COUNT && res != 0) {
        logError(1, "%s Wait for flash TIMEOUT! ERROR %08X \n", tag, ERROR_TIMEOUT);
        return ERROR_TIMEOUT;
    }

    logError(0, "%s Flash READY! \n", tag);
    return OK;
}


/**
 * Put the M3 in hold
 * @return OK if success or an error code which specify the type of error encountered
 */
int hold_m3(){
	int ret;
    u8 cmd[1] = {0x01};

    logError(0, "%s Command m3 hold... \n", tag);
	ret = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_SYSTEM_RESET, cmd, 1);
    if(ret<OK){
        logError(1, "%s hold_m3: ERROR %08X\n", tag, ret);
        return ret;
    }
    logError(0, "%s Hold M3 DONE! \n", tag);

#if !defined(I2C_INTERFACE) && defined(SPI4_WIRE)
	//configure manually SPI4 because when no fw is running the chip use SPI3 by default
	logError(0, "%s Setting SPI4 mode... \n", tag);
	cmd[0] = 0x10;
	ret = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_GPIO_DIRECTION, cmd, 1);
    if(ret<OK){
        logError(1, "%s hold_m3: can not set gpio dir ERROR %08X\n", tag, ret);
        return ret;
    }

	cmd[0] = 0x02;
	ret = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_GPIO_PULLUP, cmd, 1);
    if(ret<OK){
        logError(1, "%s hold_m3: can not set gpio pull-up ERROR %08X\n", tag, ret);
        return ret;
    }

	cmd[0] = 0x07;
	ret = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_GPIO_CONFIG_REG2, cmd, 1);
    if(ret<OK){
        logError(1, "%s hold_m3: can not set gpio config ERROR %08X\n", tag, ret);
        return ret;
    }

	cmd[0] = 0x30;
	ret = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_GPIO_CONFIG_REG0, cmd, 1);
    if(ret<OK){
        logError(1, "%s hold_m3: can not set gpio config ERROR %08X\n", tag, ret);
        return ret;
    }

	cmd[0] = SPI4_MASK;
	ret = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_ICR, cmd, 1);
    if(ret<OK){
        logError(1, "%s hold_m3: can not set spi4 mode ERROR %08X\n", tag, ret);
        return ret;
    }
	mdelay(1);		//wait for the GPIO to stabilize
#endif

    return OK;

}




/**
* Parse the raw data read from a FW file in order to fill properly the fields of a Firmware variable
* @param fw_data raw FW data loaded from system
* @param fw_size size of fw_data
* @param fwData pointer to a Firmware variable which will contain the processed data
* @param keep_cx if 1, the CX area will be loaded and burnt otherwise will be skipped and the area will be untouched
* @return OK if success or an error code which specify the type of error encountered
*/
int parseBinFile(u8* fw_data, int fw_size, Firmware *fwData, int keep_cx)
{

	int dimension, index = 0;
	u32 temp;
	int res, i;

	//the file should contain at least the header plus the content_crc
	if (fw_size < FW_HEADER_SIZE + FW_BYTES_ALLIGN || fw_data == NULL)
	{
		logError(1, "%s parseBinFile: Read only %d instead of %d... ERROR %08X\n", tag, fw_size, FW_HEADER_SIZE + FW_BYTES_ALLIGN, ERROR_FILE_PARSE);
		res = ERROR_FILE_PARSE;
		goto END;
	} else
	{
		//start parsing of bytes
		u8ToU32(&fw_data[index], &temp);
		if (temp != FW_HEADER_SIGNATURE)
		{
			logError(1, "%s parseBinFile: Wrong Signature %08X ... ERROR %08X\n", tag, temp, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}
		logError(0, "%s parseBinFile: Fw Signature OK!\n", tag);
		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		if (temp != FW_FTB_VER)
		{
			logError(1, "%s parseBinFile: Wrong ftb_version %08X ... ERROR %08X\n", tag, temp, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}
		logError(0, "%s parseBinFile: ftb_version OK!\n", tag);
		index += FW_BYTES_ALLIGN;
		if (fw_data[index] != DCHIP_ID_0 || fw_data[index + 1] != DCHIP_ID_1)
		{
			logError(1, "%s parseBinFile: Wrong target %02X != %02X  %02X != %02X ... ERROR %08X\n", tag, fw_data[index], DCHIP_ID_0, fw_data[index + 1], DCHIP_ID_1, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}
		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		logError(1, "%s parseBinFile: FILE SVN REV = %08X\n", tag, temp);

		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		fwData->fw_ver = temp;
		logError(1, "%s parseBinFile: FILE Fw Version = %04X\n", tag, fwData->fw_ver);

		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		fwData->config_id = temp;
		logError(1, "%s parseBinFile: FILE Config Project ID = %08X\n", tag, temp);

		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		logError(1, "%s parseBinFile: FILE Config Version = %08X\n", tag, temp);

		index += FW_BYTES_ALLIGN * 2; //skip reserved data

		index += FW_BYTES_ALLIGN;
		pr_info("%s parseBinFile: File External Release =  ", tag);
		for (i = 0; i < EXTERNAL_RELEASE_INFO_SIZE; i++)
		{
			fwData->externalRelease[i] = fw_data[index++];
			pr_cont("%02X ", fwData->externalRelease[i]);
		}
		pr_cont("\n");

		//index+=FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		fwData->sec0_size = temp;
		logError(1, "%s parseBinFile:  sec0_size = %08X (%d bytes)\n", tag, fwData->sec0_size, fwData->sec0_size);

		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		fwData->sec1_size = temp;
		logError(1, "%s parseBinFile:  sec1_size = %08X (%d bytes)\n", tag, fwData->sec1_size, fwData->sec1_size);

		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		fwData->sec2_size = temp;
		logError(1, "%s parseBinFile:  sec2_size = %08X (%d bytes) \n", tag, fwData->sec2_size, fwData->sec2_size);

		index += FW_BYTES_ALLIGN;
		u8ToU32(&fw_data[index], &temp);
		fwData->sec3_size = temp;
		logError(1, "%s parseBinFile:  sec3_size = %08X (%d bytes) \n", tag, fwData->sec3_size, fwData->sec3_size);

		index += FW_BYTES_ALLIGN; //skip header crc

		//if (!keep_cx)
		//{
			dimension = fwData->sec0_size + fwData->sec1_size + fwData->sec2_size + fwData->sec3_size;
			temp = fw_size;
		/*} else
		{
			dimension = fwData->sec0_size + fwData->sec1_size; //sec2 may contain cx data (future implementation) sec3 atm not used
			temp = fw_size - fwData->sec2_size - fwData->sec3_size;
			fwData->sec2_size = 0;
			fwData->sec3_size = 0;
		}*/

		if (dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN != temp)
		{
			logError(1, "%s parseBinFile: Read only %d instead of %d... ERROR %08X\n", tag, fw_size, dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}

		fwData->data = (u8*) kmalloc(dimension * sizeof (u8), GFP_KERNEL);
		if (fwData->data == NULL)
		{
			logError(1, "%s parseBinFile: ERROR %08X\n", tag, ERROR_ALLOC);
			res = ERROR_ALLOC;
			goto END;
		}

		index += FW_BYTES_ALLIGN;
		memcpy(fwData->data, &fw_data[index], dimension);
		if(fwData->sec2_size!=0){
			 u8ToU16(&fwData->data[fwData->sec0_size+fwData->sec1_size+FW_CX_VERSION],&fwData->cx_ver);

		}else{
			 logError(1, "%s parseBinFile: Initialize cx_ver to default value! \n", tag);
			 fwData->cx_ver=systemInfo.u16_cxVer;
		}

		logError(1, "%s parseBinFile: CX Version = %04X \n", tag, fwData->cx_ver);

		fwData->data_size = dimension;

		logError(0, "%s READ FW DONE %d bytes!\n", tag, fwData->data_size);
		res = OK;
		goto END;
	}

END:
	kfree(fw_data);
	return res;
}

/**
* Unlock the flash to be programmed
* @return OK if success or an error code which specify the type of error encountered
*/
int flash_unlock() {
    u8 cmd[6] = {FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_UNLOCK_CODE0, FLASH_UNLOCK_CODE1};

    logError(0, "%s Command unlock ... \n", tag);
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
        logError(1, "%s flash_unlock: ERROR %08X\n", tag, ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    logError(0, "%s Unlock flash DONE! \n", tag);

    return OK;
}

/**
* Unlock the flash to be erased
* @return OK if success or an error code which specify the type of error encountered
*/
int flash_erase_unlock() {
    u8 cmd[6] = {FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_UNLOCK_CODE0, FLASH_ERASE_UNLOCK_CODE1};

    logError(0, "%s Try to erase unlock flash... \n", tag);

    logError(0, "%s Command erase unlock ... \n", tag);
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < 0) {
        logError(1, "%s flash_erase_unlock: ERROR %08X\n", tag, ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    logError(0, "%s Erase Unlock flash DONE! \n", tag);

    return OK;
}

/**
* Erase the full flash
* @return OK if success or an error code which specify the type of error encountered
*/
int flash_full_erase() {
    int status;

	u8 cmd1[6] = {FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0+1, 0x00 };
    u8 cmd[6] = {FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0, FLASH_ERASE_CODE1};

	if (fts_write(cmd1, ARRAY_SIZE(cmd1)) < OK) {
		logError(1, "%s flash_erase_page_by_page: ERROR %08X\n", tag, ERROR_BUS_W);
		return ERROR_BUS_W;
    }

    logError(0, "%s Command full erase sent ... \n", tag);
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
        logError(1, "%s flash_full_erase: ERROR %08X\n", tag, ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    status = wait_for_flash_ready(FLASH_ERASE_CODE0);

    if (status != OK) {
        logError(1, "%s flash_full_erase: ERROR %08X\n", tag, ERROR_FLASH_NOT_READY);
        return (status | ERROR_FLASH_NOT_READY); //Flash not ready within the chosen time, better exit!
    }

    logError(0, "%s Full Erase flash DONE! \n", tag);

    return OK;

}

/**
* Erase the flash page by page, giving the possibility to skip the CX area and maintain therefore its value
* @param keep_cx if SKIP_PANEL_INIT the Panel Init pages will be skipped, if > SKIP_PANEL_CX_INIT Cx and Panel Init pages otherwise all the pages will be deleted
* @return OK if success or an error code which specify the type of error encountered
*/
int flash_erase_page_by_page(ErasePage keep_cx) {

    u8 status,i=0;
	u8 cmd1[6] = {FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0+1, 0x00 };
	u8 cmd[6] = {FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0, 0xA0 };
	u8 cmd2[9] = {FTS_CMD_HW_REG_W,  0x20, 0x00, 0x01, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
	u8 mask[4]={0};

    for(i=FLASH_CX_PAGE_START; i<=FLASH_CX_PAGE_END && keep_cx>=SKIP_PANEL_CX_INIT; i++){
			logError(0, "%s Skipping erase CX page %d! \n", tag,i);
			fromIDtoMask(i, mask, 4);
	}


	for(i=FLASH_PANEL_PAGE_START; i<=FLASH_PANEL_PAGE_END && keep_cx>=SKIP_PANEL_INIT; i++){
		logError(0, "%s Skipping erase Panel Init page %d! \n", tag,i);
		fromIDtoMask(i, mask, 4);
	}


	pr_info("%s Setting the page mask = ", tag);
	for(i=0; i<4; i++){
		cmd2[5+i] = cmd2[5+i]&(~mask[i]);
		pr_cont("%02X ", cmd2[5+i]);
	}

	pr_cont("\n%s Writing page mask... \n", tag);
	if (fts_write(cmd2, ARRAY_SIZE(cmd2)) < OK) {
		logError(1, "%s flash_erase_page_by_page: Page mask ERROR %08X\n", tag, ERROR_BUS_W);
		return ERROR_BUS_W;
    }

	if (fts_write(cmd1, ARRAY_SIZE(cmd1)) < OK) {
		logError(1, "%s flash_erase_page_by_page: Disable info ERROR %08X\n", tag, ERROR_BUS_W);
		return ERROR_BUS_W;
	}

	logError(0, "%s Command erase pages sent ... \n", tag);
	if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
		logError(1, "%s flash_erase_page_by_page: Erase ERROR %08X\n", tag, ERROR_BUS_W);
		return ERROR_BUS_W;
	}

	status = wait_for_flash_ready(FLASH_ERASE_CODE0);

	if (status != OK) {

		logError(1, "%s flash_erase_page_by_page: ERROR % 08X\n", tag, ERROR_FLASH_NOT_READY);
		return (status | ERROR_FLASH_NOT_READY); //Flash not ready within the chosen time, better exit!
	}

     logError(0, "%s Erase flash page by page DONE! \n", tag);

    return OK;
}

/**
* Start the DMA procedure which actually transfer and burn the data loaded from memory into the Flash
* @return OK if success or an error code which specify the type of error encountered
*/
int start_flash_dma(void)
{
    int status;
    u8 cmd[6] = {FLASH_CMD_WRITE_REGISTER, 0x20, 0x00, 0x00, FLASH_DMA_CODE0, FLASH_DMA_CODE1}; //write the command to erase the flash

    logError(0, "%s Command flash DMA ... \n", tag);
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
        logError(1, "%s start_flash_dma: ERROR %08X\n", tag, ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    status = wait_for_flash_ready(FLASH_DMA_CODE0);

    if (status != OK) {
        logError(1, "%s start_flash_dma: ERROR %08X\n", tag, ERROR_FLASH_NOT_READY);
        return (status | ERROR_FLASH_NOT_READY); //Flash not ready within the chosen time, better exit!
    }

    logError(0, "%s flash DMA DONE! \n", tag);

    return OK;
}

/**
* Copy the FW data that should be burnt in the Flash into the memory and then the DMA will take care about burning it into the Flash
* @param address address in memory where to copy the data, possible values are FLASH_ADDR_CODE, FLASH_ADDR_CONFIG, FLASH_ADDR_CX
* @param data pointer to an array of byte which contain the data that should be copied into the memory
* @param size size of data
* @return OK if success or an error code which specify the type of error encountered
*/
int fillFlash(u32 address, u8 *data, int size) {

    int remaining = size, index = 0;
    int toWrite = 0;
    int byteBlock = 0;
    int wheel = 0;
    u32 addr = 0;
    int res;
    int delta;
    u8* buff = NULL;
    u8 buff2[12] = {0};


    buff = (u8*) kmalloc((DMA_CHUNK + 5) * sizeof (u8), GFP_KERNEL);
	if (buff == NULL) {
		logError(1, "%s fillFlash: ERROR %08X\n", tag, ERROR_ALLOC);
		return ERROR_ALLOC;
	}

    while (remaining > 0) {
		byteBlock = 0;
		addr = 0x00100000;

        while (byteBlock < FLASH_CHUNK && remaining > 0) {
			index = 0;
            if (remaining >= DMA_CHUNK) {
                if ((byteBlock + DMA_CHUNK) <= FLASH_CHUNK) {
					//logError(1, "%s fillFlash: 1\n", tag);
                    toWrite = DMA_CHUNK;
                    remaining -= DMA_CHUNK;
                    byteBlock += DMA_CHUNK;
                } else {
					//logError(1, "%s fillFlash: 2\n", tag);
                    delta = FLASH_CHUNK - byteBlock;
                    toWrite = delta;
                    remaining -= delta;
                    byteBlock += delta;
                }
            } else {
                if ((byteBlock + remaining) <= FLASH_CHUNK) {
					//logError(1, "%s fillFlash: 3\n", tag);
                    toWrite = remaining;
					byteBlock += remaining;
                    remaining = 0;

                } else {
					//logError(1, "%s fillFlash: 4\n", tag);
                    delta = FLASH_CHUNK - byteBlock;
                    toWrite = delta;
                    remaining -= delta;
                    byteBlock += delta;
                }
            }

			buff[index++] = FTS_CMD_HW_REG_W;
			buff[index++] = (u8) ((addr & 0xFF000000) >> 24);
			buff[index++] = (u8) ((addr & 0x00FF0000) >> 16);
			buff[index++] = (u8) ((addr & 0x0000FF00) >> 8);
            buff[index++] = (u8) (addr & 0x000000FF);

            memcpy(&buff[index], data, toWrite);
            //logError(0, "%s Command = %02X , address = %02X %02X , bytes = %d, data =  %02X %02X, %02X %02X \n", tag, buff[0], buff[1], buff[2], toWrite, buff[3], buff[4], buff[3 + toWrite-2], buff[3 + toWrite-1]);
            if (fts_write(buff, index + toWrite) < OK) {
                logError(1, "%s fillFlash: ERROR %08X\n", tag, ERROR_BUS_W);
				kfree(buff);
                return ERROR_BUS_W;
            }

			//mdelay(5);
            addr += toWrite;
            data += toWrite;
        }


        //configuring the DMA
        byteBlock = byteBlock / 4 - 1;
		index = 0;

        buff2[index++] = FLASH_CMD_WRITE_REGISTER;
		buff2[index++] = 0x20;
        buff2[index++] = 0x00;
		buff2[index++] = 0x00;
        buff2[index++] = FLASH_DMA_CONFIG;
        buff2[index++] = 0x00;
        buff2[index++] = 0x00;

		addr = address +((wheel * FLASH_CHUNK)/4);
        buff2[index++] = (u8) ((addr & 0x000000FF));
        buff2[index++] = (u8) ((addr & 0x0000FF00) >> 8);
        buff2[index++] = (u8) (byteBlock & 0x000000FF);
        buff2[index++] = (u8) ((byteBlock & 0x0000FF00)>> 8);
		buff2[index++] = 0x00;

		logError(0, "%s DMA Command = %02X , address = %02X %02X, words =  %02X %02X \n", tag, buff2[0], buff2[8], buff2[7], buff2[10], buff2[9]);

		if (fts_write(buff2, index) < OK) {
            logError(1, "%s   Error during filling Flash! ERROR %08X \n", tag, ERROR_BUS_W);
			kfree(buff);
            return (ERROR_BUS_W);
        }

        res = start_flash_dma();
        if (res < OK) {
            logError(1, "%s   Error during flashing DMA! ERROR %08X \n", tag, res);
			kfree(buff);
            return res;
        }
        wheel++;
    }
    kfree(buff);
    return OK;
}


/*
 * Execute the procedure to burn a FW on FTM5/FTI IC
 *
 * @param fw - structure which contain the FW to be burnt
 * @param force_burn - if >0, the flashing procedure will be forced and executed
 *	regardless the additional info, otherwise the FW in the file will be
 *	burned only if it is different from the one running in the IC
 * @param keep_cx - if 1, the function preserves the CX/Panel Init area.
 *	Otherwise, it will be cleared.
 *
 * @return OK if success or an error code which specifies the type of error
 *	encountered
 */
int flash_burn(Firmware fw, int force_burn, int keep_cx) {
	int res;

	if (!force_burn) {
		/* Compare firmware, config, and CX versions */
		if (fw.fw_ver != (uint32_t)systemInfo.u16_fwVer ||
		    fw.config_id != (uint32_t)systemInfo.u16_cfgVer ||
		    fw.cx_ver != (uint32_t)systemInfo.u16_cxVer)
			goto start;

		for (res = EXTERNAL_RELEASE_INFO_SIZE - 1; res >= 0; res--) {
			if (fw.externalRelease[res] !=
			    systemInfo.u8_releaseInfo[res])
				goto start;
		}

		logError(1,
			 "%s flash_burn: Firmware in the chip matches the firmware to flash! NO UPDATE ERROR %08X\n",
			 tag, ERROR_FW_NO_UPDATE);
		return (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED);
	} else if (force_burn == CRC_CX && fw.sec2_size == 0) {
		/* burn procedure to update the CX memory, if not present just
		 * skip it!
		 */
		for (res = EXTERNAL_RELEASE_INFO_SIZE - 1; res >= 0; res--) {
			if (fw.externalRelease[res] > systemInfo.u8_releaseInfo[res])
				// Avoid loading the CX because it is missing in the bin file, it just need to update to last fw+cfg because a new release
				force_burn = 0;
				goto start;
		}
		logError(1,
			 "%s flash_burn: CRC in CX but fw does not contain CX data! NO UPDATE ERROR %08X\n",
			 tag, ERROR_FW_NO_UPDATE);
		return (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED);
	}

	/* Programming procedure start */
start:
	logError(0, "%s Programming Procedure for flashing started:\n", tag);

	logError(0, "%s 1) SYSTEM RESET:\n", tag);
	res = fts_system_reset();
	if (res < 0) {
		logError(1, "%s    system reset FAILED!\n", tag);
		/* If there is no firmware, there is no controller ready event
		 * and there will be a timeout, we can keep going. But if
		 * there is an I2C error, we must exit.
		 */
		if (res != (ERROR_SYSTEM_RESET_FAIL | ERROR_TIMEOUT))
			return (res | ERROR_FLASH_BURN_FAILED);
	} else
		logError(0, "%s   system reset COMPLETED!\n", tag);

	logError(0, "%s 2) HOLD M3 :\n", tag);
	res = hold_m3();
	if (res < OK) {
		logError(1, "%s    hold_m3 FAILED!\n", tag);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	logError(0, "%s    hold_m3 COMPLETED!\n", tag);

	logError(0, "%s 3) FLASH UNLOCK:\n", tag);
	res = flash_unlock();
	if (res < OK) {
		logError(1, "%s   flash unlock FAILED! ERROR %08X\n", tag,
			 ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	logError(0, "%s   flash unlock COMPLETED!\n", tag);

	logError(0, "%s 4) FLASH ERASE UNLOCK:\n", tag);
	res = flash_erase_unlock();
	if (res < 0) {
		logError(1, "%s   flash unlock FAILED! ERROR %08X\n", tag,
			 ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	logError(0, "%s   flash unlock COMPLETED!\n", tag);

	logError(0, "%s 5) FLASH ERASE:\n", tag);
	if (keep_cx > 0) {
		if (fw.sec2_size != 0)
			res = flash_erase_page_by_page(SKIP_PANEL_INIT);
		else
			res = flash_erase_page_by_page(SKIP_PANEL_CX_INIT);
	} else {
		res = flash_erase_page_by_page(SKIP_PANEL_INIT);
		if(fw.sec2_size==0)
			logError(1, "%s WARNING!!! Erasing CX memory but no CX in fw file! touch will not work right after fw update! \n", tag);
	}

	if (res < OK) {
		logError(1, "%s   flash erase FAILED! ERROR %08X\n", tag,
			 ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	logError(0, "%s   flash erase COMPLETED!\n", tag);

	logError(0, "%s 6) LOAD PROGRAM:\n", tag);
	res = fillFlash(FLASH_ADDR_CODE, &fw.data[0], fw.sec0_size);
	if (res < OK) {
		logError(1, "%s   load program ERROR %08X\n", tag,
			 ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	logError(1, "%s   load program DONE!\n", tag);

	logError(0, "%s 7) LOAD CONFIG:\n", tag);
	res = fillFlash(FLASH_ADDR_CONFIG, &(fw.data[fw.sec0_size]),
			fw.sec1_size);
	if (res < OK) {
		logError(1, "%s   load config ERROR %08X\n", tag,
			 ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	logError(1, "%s   load config DONE!\n", tag);

	if (fw.sec2_size != 0 && (force_burn == CRC_CX || keep_cx <= 0)) {
		logError(0, "%s 7.1) LOAD CX:\n", tag);
		res = fillFlash(FLASH_ADDR_CX,
				&(fw.data[fw.sec0_size + fw.sec1_size]),
				fw.sec2_size);
		if (res < OK) {
			logError(1, "%s   load cx ERROR %08X\n", tag,
				 ERROR_FLASH_BURN_FAILED);
			return (res | ERROR_FLASH_BURN_FAILED);
		}
		logError(1, "%s   load cx DONE!\n", tag);
	}

	logError(0, "%s   Flash burn COMPLETED!\n", tag);

	logError(0, "%s 8) SYSTEM RESET:\n", tag);
	res = fts_system_reset();
	if (res < 0) {
		logError(1, "%s    system reset FAILED! ERROR %08X\n", tag,
			 ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	logError(0, "%s   system reset COMPLETED!\n", tag);

	logError(0, "%s 9) FINAL CHECK:\n", tag);
	res = readSysInfo(0);
	if (res < 0) {
		logError(1,
			 "%s flash_burn: Unable to retrieve Chip INFO! ERROR %08X\n",
			 tag, ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}

	for (res = 0; res < EXTERNAL_RELEASE_INFO_SIZE; res++) {
		if (fw.externalRelease[res] != systemInfo.u8_releaseInfo[res]) {
			/* External release is printed during readSysInfo */
			logError(1,
				 "%s  Firmware in the chip different from the one that was burn!\n",
				 tag);
			return ERROR_FLASH_BURN_FAILED;
		}
	}

	logError(0, "%s   Final check OK!\n", tag);

	return OK;
}

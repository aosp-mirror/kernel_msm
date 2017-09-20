/*

**************************************************************************
**                        STMicroelectronics							**
**************************************************************************
**                        marco.cali@st.com								**
**************************************************************************
*                                                                        *
*							HW related data								 *
*                                                                        *
**************************************************************************
**************************************************************************

*/

/*!
* \file ftsHardware.h
* \brief Contains all the definitions and information related to the IC from an hardware point of view
*/

#ifndef FTS_HARDWARE_H
#define FTS_HARDWARE_H

//type of IC

//#define FTM3_CHIP																///< Use with FTM3
#ifndef FTM3_CHIP
#define FTI																		///< Use with FTM5 onward
#ifdef FTI
//#define SONIA																	///< Use with FTM5 automotive
#endif
#endif


//DIGITAL CHIP INFO
#ifdef FTM3_CHIP
#define DCHIP_ID_0								0x39							///< LSB chip ID for FTM3
#define DCHIP_ID_1								0x6C							///< MSB chip ID for FTM3
#else
#ifdef FTI
#ifndef SONIA
#define DCHIP_ID_0								0x36							///< LSB chip ID for FTM5
#define DCHIP_ID_1								0x39							///< MSB chip ID for FTM5
#else
#define DCHIP_ID_0								0x36							///< LSB chip ID for SONIA
#define DCHIP_ID_1								0x58							///< MSB chip ID for SONIA
#endif
#else
#define DCHIP_ID_0								0x36							///< LSB chip ID for FTM4
#define DCHIP_ID_1								0x70							///< MSB chip ID for FTM4
#endif
#endif


#define DCHIP_FW_VER_BYTE						2								///< number of bytes of the fw versions

//CHUNKS
#define READ_CHUNK								1024							///< chunk dimension of a single i2c read, max allowed value is 2kB
#define WRITE_CHUNK								1024							///< chunk dimension of a single i2c write, max allowed value is 2kB
#define MEMORY_CHUNK							1024							///< chunk dimenasion of a single i2c write on mem, max allowed value is 2kB

//PROTOCOL INFO
#define I2C_INTERFACE															///< comment if the chip use SPI bus
#ifdef I2C_INTERFACE
#define I2C_SAD									0x49							///< slave address of the IC
#else
#define SPI4_WIRE																///< comment if the master is SPI3 wires (MOSI and MISO share same line)
#define SPI_DELAY_CS							10								///< time in usec to wait before rising the CS
#define SPI_CLOCK_FREQ							7000000							///< clock frequency in Hz of the SPI bus
#endif

//#define ICR_ADDR								0x0024							///< address of Device control register to set the comunication protocol (SPI/I2C)
//#define ICR_SPI_VALUE							0x02							///< value to write in ICR_ADDR to enable the SPI4 protocol


#define IER_ENABLE								0x41							///< value to write in IER_ADDR to enable the interrupts
#define IER_DISABLE								0x00							///< value to write in IER_ADDR to disable the interrupts

//FLASH COMMAND
/** @defgroup flash_command Flash Commands
 *	All the commands that works with the flash of the IC
 *	@{
 */
#define FLASH_CMD_UNLOCK						0xF7
#ifdef FTM3_CHIP
#define FLASH_CMD_WRITE_LOWER_64                0xF0
#define FLASH_CMD_WRITE_UPPER_64                0xF1
#define FLASH_CMD_BURN							0xF2
#define FLASH_CMD_ERASE							0xF3
#define FLASH_CMD_READSTATUS                    0xF4
#else
#define FLASH_CMD_WRITE_64K                     0xF8
#ifdef FTI
#define FLASH_CMD_READ_REGISTER                 0xFA
#else
#define FLASH_CMD_READ_REGISTER                 0xF9
#endif
#define FLASH_CMD_WRITE_REGISTER				0xFA
#endif

//FLASH UNLOCK PARAMETER
#ifdef FTI
#define FLASH_UNLOCK_CODE0						0x25
#define FLASH_UNLOCK_CODE1						0x20
#else
#define FLASH_UNLOCK_CODE0						0x74
#define FLASH_UNLOCK_CODE1						0x45
#endif


#ifndef FTM3_CHIP
//FLASH ERASE and DMA PARAMETER
#define FLASH_ERASE_START						0x80
#define FLASH_ERASE_CODE1                       0xC0
#define FLASH_DMA_CODE1                         0xC0
#ifdef FTI
#define FLASH_ERASE_UNLOCK_CODE0				0xDE
#define FLASH_ERASE_UNLOCK_CODE1				0x03
#define FLASH_ERASE_CODE0                       0x6A
#define FLASH_DMA_CODE0				0x71
#define FLASH_DMA_CONFIG                        0x72
#define FLASH_NUM_PAGE							32								///< number of pages in main flash
#ifndef SONIA
#define FLASH_CX_PAGE_START						28								///< starting page which contain Cx data
#define FLASH_CX_PAGE_END						30								///< last page which contain Cx data
#define FLASH_PANEL_PAGE_START					26								///< starting page which contain Panel Init data
#define FLASH_PANEL_PAGE_END					27								///< last page which contain Panel Init data
#else
#define FLASH_CX_PAGE_START						27								///< starting page which contain Cx data
#define FLASH_CX_PAGE_END						30								///< last page which contain Cx data
#define FLASH_PANEL_PAGE_START					26								///< starting page which contain Panel Init data
#define FLASH_PANEL_PAGE_END					26								///< last page which contain Panel Init data
#endif
#else
#define FLASH_ERASE_UNLOCK_CODE0				0x72
#define FLASH_ERASE_UNLOCK_CODE1				0x03
#define FLASH_ERASE_UNLOCK_CODE2				0x02
#define FLASH_ERASE_CODE0                       0x02
#define FLASH_DMA_CODE0                         0x05
#define FLASH_DMA_CONFIG                        0x06
#define FLASH_NUM_PAGE							64								///< number of pages in main flash
#define FLASH_CX_PAGE_START						59								///< starting page which contain Cx data
#define FLASH_CX_PAGE_END						62								///< last page which contain Cx data
#define FLASH_PANEL_PAGE_START					58								///< starting page which contain Panel Init data
#define FLASH_PANEL_PAGE_END					58								///< last page which contain Panel Init data
#endif
#endif
/** @} */

//FLASH ADDRESS
#ifdef FTM3_CHIP
#define FLASH_ADDR_SWITCH_CMD                   0x00010000						///< address which remark the first 64k in flash
#define FLASH_ADDR_CODE							0x00000000						///< starting address in the flash of the code
#define FLASH_ADDR_CONFIG						0x0001E800						///< starting address in the flash of the config
#define FLASH_ADDR_CX							0x0001F000						///< starting address in the flash of the CX
#else
#define FLASH_ADDR_CODE							0x00000000						///< starting address (words) in the flash of the code in FTM4
#ifdef FTI
#ifndef SONIA
#define FLASH_ADDR_CONFIG						0x00007C00						///< starting address (words) in the flash of the config in FTI
#define FLASH_ADDR_CX							0x00007000						///< starting address (words) in the flash of the Init data in FTI
#else
#define FLASH_ADDR_CONFIG						0x00007C00						///< starting address (words) in the flash of the config in FTI
#define FLASH_ADDR_CX							0x00006C00						///< starting address (words) in the flash of the Init data in FTI
#endif
#else
#define ADDR_WARM_BOOT                          0x001E							///< address of the System Reset control register in FTM4
#define WARM_BOOT_VALUE                         0x38							///< value to write in ADDR_WARM_BOOT to performa a warm boot
#define FLASH_ADDR_CONFIG						0x0000FC00						///< starting address (words) in the flash of the config in FTM4
#define FLASH_ADDR_CX							0x0000EC00						///< starting address (words) in the flash of the Init data in FTM4
#endif


#endif


//SIZES FW, CODE, CONFIG, MEMH
/** @defgroup fw_file FW file info
 *	All the info related to the fw file
 *	@{
 */
#ifdef FTM3_CHIP
#define FW_HEADER_SIZE							32												///< fw header dimension in bytes
#define FW_SIZE									(int)(128*1024)									///< full fw size in bytes
#define FW_CODE_SIZE							(int)(122*1024)									///< code size in bytes
#define FW_CONFIG_SIZE							(int)(2*1024)									///< config size in bytes
#define FW_CX_SIZE								(int)(FW_SIZE-FW_CODE_SIZE-FW_CONFIG_SIZE)		///< CX size in bytes
#define FW_VER_MEMH_BYTE1						193
#define FW_VER_MEMH_BYTE0						192
#define FW_OFF_CONFID_MEMH_BYTE1				2												///< offset of the config id MSB in the config memh
#define FW_OFF_CONFID_MEMH_BYTE0				1												///< offset of the config id LSB in the config
#define FW_BIN_VER_OFFSET						4												///< offset of the fw version in the bin file
#define FW_BIN_CONFIG_VER_OFFSET				(FW_HEADER_SIZE+FW_CODE_SIZE+1)					///< offset of the config id in the bin file
#else
#define FW_HEADER_SIZE							64												///< dimension of the header of the .fts file
#define FW_HEADER_SIGNATURE						0xAA55AA55										///< header signature
#define FW_FTB_VER								0x00000001										///< .ftb version
#define FW_BYTES_ALLIGN							4												///< allignment of the info
#define FW_BIN_VER_OFFSET						16												///< offset of the fw version in the .ftb file
#define FW_BIN_CONFIG_ID_OFFSET					20												///< offset of the config id in the .ftb file
#define FW_CX_VERSION							(16+4)											///< offset of CX version in the sec2 of FW file
#endif
/** @} */


//FIFO
#define FIFO_EVENT_SIZE							8								///< number of bytes of one event
#ifdef FTM3_CHIP
#define FIFO_DEPTH								32								///< max number of events that the FIFO can collect before going in overflow in FTM3
#else
#ifdef FTI
#define FIFO_DEPTH								32								///< max number of events that the FIFO can collect before going in overflow in FTM5
#else
#define FIFO_DEPTH								64								///< max number of events that the FIFO can collect before going in overflow in FTM4
#endif
#endif

#ifndef FTI
#define FIFO_CMD_READONE						0x85							///< commad to read one event from FIFO
#define FIFO_CMD_READALL						0x86							///< command to read all the events in the FIFO
#define FIFO_CMD_LAST							0x87							///< command to obtain the last event of the FIFO (most recent)
#define FIFO_CMD_FLUSH							0xA1							///< command to flush the FIFO
#else
#ifdef I2C_INTERFACE
#define FIFO_CMD_READALL						0x86							///< command to read all the events in the FIFO
#else
#define FIFO_CMD_READALL						0x87							///< command to read all the events in the FIFO
#endif
#define FIFO_CMD_READONE						FIFO_CMD_READALL				///< commad to read one event from FIFO

#endif

//OP CODES FOR MEMORY (based on protocol)
#ifndef FTI
#ifdef I2C_INTERFACE
#define FTS_CMD_HW_REG_R						0xB6							///< command to read an hw register
#define FTS_CMD_HW_REG_W						0xB6							///< command to write an hw register
#define FTS_CMD_FRAMEBUFFER_R					0xD0							///< command to read the framebuffer
#define FTS_CMD_CONFIG_R						0xD0							///< command to read the config memory
#define FTS_CMD_CONFIG_W						0xD0							///< command to write the config memory
#else
#define FTS_CMD_HW_REG_R						0xB4							///< command to read an hw register
#define FTS_CMD_HW_REG_W						0xB6							///< command to write an hw register
#define FTS_CMD_FRAMEBUFFER_R					0xD0							///< command to read the framebuffer
#define FTS_CMD_CONFIG_R						0xD0							///< command to read the config memory
#define FTS_CMD_CONFIG_W						0xD0							///< command to write the config memory
#endif
#else
#ifdef I2C_INTERFACE
#define FTS_CMD_HW_REG_R						0xFA							///< command to read an hw register if FTI
#define FTS_CMD_HW_REG_W						0xFA							///< command to write an hw register if FTI
#define FTS_CMD_FRAMEBUFFER_R					0xA6							///< command to read the framebuffer if FTI
#define FTS_CMD_CONFIG_R						0xA8							///< command to read the config memory if FTI
#define FTS_CMD_CONFIG_W						0xA8							///< command to write the config memory if FTI
#else
#define FTS_CMD_HW_REG_R						0xFB							///< command to read an hw register if FTI
#define FTS_CMD_HW_REG_W						0xFA							///< command to write an hw register if FTI
#define FTS_CMD_FRAMEBUFFER_R					0xA7							///< command to read the framebuffer if FTI
#define FTS_CMD_CONFIG_R						0xA9							///< command to read the config memory if FTI
#define FTS_CMD_CONFIG_W						0xA8							///< command to write the config memory if FTI
#endif
#endif

//DUMMY BYTES DATA
#ifdef FTI
#ifndef I2C_INTERFACE
#define DUMMY_HW_REG							1								///< 1 if the first byte read from HW register is dummy
#define DUMMY_FRAMEBUFFER						1								///< 1 if the first byte read from Frame buffer is dummy
#define DUMMY_CONFIG							1								///< 1 if the first byte read from Config Memory is dummy
#define DUMMY_FIFO								1								///< 1 if the first byte read from FIFO is dummy
#else
#define DUMMY_HW_REG							0								///< 1 if the first byte read from HW register is dummy
#define DUMMY_FRAMEBUFFER						0								///< 1 if the first byte read from Frame buffer is dummy
#define DUMMY_CONFIG							0								///< 1 if the first byte read from Config Memory is dummy
#define DUMMY_FIFO								0								///< 1 if the first byte read from FIFO is dummy
#endif
#else
#define DUMMY_HW_REG							1								///< 1 if the first byte read from HW register is dummy
#define DUMMY_FRAMEBUFFER						1								///< 1 if the first byte read from Frame buffer is dummy
#define DUMMY_CONFIG							1								///< 1 if the first byte read from Config Memory is dummy
#define DUMMY_FIFO								0								///< 1 if the first byte read from FIFO is dummy
#endif

/** @defgroup hw_adr HW Address
 * @ingroup address
 * Important addresses of hardware registers (and sometimes their important values)
 * @{
 */

//IMPORTANT HW ADDRESSES (u64)
#define ADDR_FRAMEBUFFER			(u64)0x0000000000000000						///< frame buffer address in memory
#define ADDR_ERROR_DUMP				(u64)0x000000000000EF80						///< start address dump error log

//SYSTEM RESET INFO
#ifdef FTM3_CHIP
#define ADDR_SYSTEM_RESET			(u64)0x0000000000000023						///< address of System control register in FTM3
#define SYSTEM_RESET_VALUE						0x01							///< value to write in SYSTEM_RESET_ADDRESS to perform a system reset in FTM3
#else
#ifdef FTI
#define ADDR_SYSTEM_RESET			(u64)0x0000000020000024						///< address of System control register in FTI
#else
#define ADDR_SYSTEM_RESET			(u64)0x0000000000000028						///< address of System control register in FTM4
#endif
#define SYSTEM_RESET_VALUE						0x80							///< value to write in SYSTEM_RESET_ADDRESS to perform a system reset in FTM4
#endif

//REMAP REGISTER
#ifdef FTI
#define ADDR_BOOT_OPTION			(u64)0x0000000020000025						///< address of Boot option register in SONIA
#endif

//INTERRUPT INFO
#ifdef FTM3_CHIP
#define ADDR_IER					(u64)0x000000000000001C						///< address of the Interrupt enable register in FTM3
#else
#ifdef FTI
#define ADDR_IER					(u64)0x0000000020000029						///< address of the Interrupt enable register in FTMI
#else
#define ADDR_IER					(u64)0x000000000000002C						///< address of the Interrupt enable register in FTM4
#endif
#endif

#ifdef FTM3_CHIP
#define ADDR_DCHIP_ID				(u64)0x0000000000000007						///< chip id address for FTM3
#define ADDR_DCHIP_FW_VER			(u64)0x000000000000000A						///< fw version address for FTM3
#else
#ifdef FTI
#define ADDR_DCHIP_ID				(u64)0x0000000020000000						///< chip id address for FTI
#define ADDR_DCHIP_FW_VER			(u64)0x0000000020000004						///< fw version address for FTI
#else
#define ADDR_DCHIP_ID				(u64)0x0000000000000004						///< chip id address for FTM4
#define ADDR_DCHIP_FW_VER			(u64)0x0000000000000008						///< fw version address for FTM4
#endif
#endif

//INTERFACE REGISTER
#ifdef FTM3_CHIP
#define ADDR_ICR					(u64)0x0000000000000024						///< address of Device control register to set the comunication protocol (SPI/I2C)
#define SPI4_MASK					0x02										///< bit to set spi4
#else
#ifdef FTI
#define ADDR_ICR					(u64)0x000000002000002D						///< address of Device control register to set the comunication protocol (SPI/I2C)
#define SPI4_MASK					0x02										///< bit to set spi4
#else
#define ADDR_ICR					(u64)0x0000000000000031						///< address of Device control register to set the comunication protocol (SPI/I2C)
#define SPI4_MASK					0x02										///< bit to set spi4
#endif
#endif

//CRC ADDR
#ifdef FTM3_CHIP
#define ADDR_CRC					(u64)0x0000000000000086						///< address of the FLASH status register in FTM3
#define CRC_MASK								0x02							///< bitmask which reveal if there is a CRC error in the flash
#else
#ifdef FTI
#define ADDR_CRC					(u64)0x0000000020000078						///< address of the CRC control register in FTI
#else
#define ADDR_CRC					(u64)0x0000000000000074						///< address of the CRC control register in FTM4
#endif
#define CRC_MASK								0x03							///< bitmask which reveal if there is a CRC error in the flash
#endif

#ifndef FTI
#define ADDR_CONFIG_OFFSET			(u64)0x000000000000F000						///< config address in memory
#else
#define ADDR_CONFIG_OFFSET			(u64)0x0000000000000000						///< config address in memory if FTI
#endif

#ifdef FTI
#define ADDR_GPIO_INPUT				(u64)0x0000000020000030						///< address of GPIO input register
#define ADDR_GPIO_DIRECTION			(u64)0x0000000020000032						///< address of GPIO direction register
#define ADDR_GPIO_PULLUP			(u64)0x0000000020000034						///< address of GPIO pullup register
#define ADDR_GPIO_CONFIG_REG0		(u64)0x000000002000003D						///< address of GPIO config register
#define ADDR_GPIO_CONFIG_REG2		(u64)0x000000002000003F						///< address of GPIO config register
#endif

/**@}*/


#endif

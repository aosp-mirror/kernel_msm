/*
  *
  **************************************************************************
  **                        STMicroelectronics				 **
  **************************************************************************
  **                        marco.cali@st.com				*
  **************************************************************************
  *                                                                        *
  *                     Utilities published in /proc/fts		*
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file fts_proc.c
  * \brief contains the function and variables needed to publish a file node in
  * the file system which allow to communicate with the IC from userspace
  */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsCore.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"


#define DRIVER_TEST_FILE_NODE	"driver_test"	/* /< name of file node
						 * published */
#define CHUNK_PROC		1024	/* /< Max chunk of data printed on
					 * the sequential file in each
					 * iteration */
#define DIAGNOSTIC_NUM_FRAME	10	/* /< number of frames reading
					 * iterations during the diagnostic
					 * test */



/** @defgroup proc_file_code	 Proc File Node
  * @ingroup file_nodes
  * The /proc/fts/driver_test file node provide expose the most important API
  * implemented into the driver to execute any possible operation into the IC \n
  * Thanks to a series of Operation Codes, each of them, with a different set of
  * parameter, it is possible to select a function to execute\n
  * The result of the function is usually returned into the shell as an ASCII
  * hex string where each byte is encoded in two chars.\n
  * @{
  */

/* Bus operations */
#define CMD_READ				0x00	/* /< I2C/SPI read: need
							 * to pass: byteToRead1
							 * byteToRead0
							 * (optional) dummyByte
							 * */
#define CMD_WRITE				0x01	/* /< I2C/SPI write:
							 * need to pass: cmd[0]
							 *  cmd[1] …
							 * cmd[cmdLength-1] */
#define CMD_WRITEREAD				0x02	/* /< I2C/SPI writeRead:
							 * need to pass: cmd[0]
							 *  cmd[1] …
							 * cmd[cmdLength-1]
							 * byteToRead1
							 * byteToRead0 dummyByte
							 * */
#define CMD_WRITETHENWRITEREAD			0x03	/* /< I2C/SPI write then
							 * writeRead: need to
							 * pass: cmdSize1
							 * cmdSize2 cmd1[0]
							 * cmd1[1] …
							 * cmd1[cmdSize1-1]
							 * cmd2[0] cmd2[1] …
							 * cmd2[cmdSize2-1]
							 *  byteToRead1
							 * byteToRead0 */
#define CMD_WRITEU8UX				0x04
						/* /< I2C/SPI
						 * writeU8UX:
						 * need to pass: cmd
						 * addrSize addr[0] …
						 * addr[addrSize-1]
						 * data[0] data[1] … */
#define CMD_WRITEREADU8UX			0x05	/* /< I2C/SPI
							 * writeReadU8UX: need
							 * to pass: cmd addrSize
							 * addr[0] …
							 * addr[addrSize-1]
							 * byteToRead1
							 * byteToRead0
							 * hasDummyByte */
#define CMD_WRITEU8UXTHENWRITEU8UX		0x06
						/* /< I2C/SPI writeU8UX
						 * then writeU8UX: need
						 * to pass: cmd1
						 * addrSize1 cmd2
						 * addrSize2 addr[0] …
						 * addr[addrSize1+
						 *      addrSize2-1]
						 * data[0] data[1] … */
#define CMD_WRITEU8UXTHENWRITEREADU8UX		0x07	/* /< I2C/SPI writeU8UX
							 *  then writeReadU8UX:
							 * need to pass: cmd1
							 * addrSize1 cmd2
							 * addrSize2 addr[0] …
							 * addr[addrSize1+
							 *      addrSize2-1]
							 *  byteToRead1
							 * byteToRead0
							 * hasDummybyte */
#define CMD_GETLIMITSFILE			0x08	/* /< Get the Production
							 * Limits File and print
							 * its content into the
							 * shell: need to pass:
							 * path(optional)
							 * otherwise select the
							 * approach chosen at
							 * compile time */
#define CMD_GETFWFILE				0x09	/* /< Get the FW file
							 * and print its content
							 * into the shell: need
							 * to pass: path
							 * (optional) otherwise
							 * select the approach
							 * chosen at compile
							 * time */
#define CMD_VERSION				0x0A	/* /< Get the driver
							 * version and other
							 * driver setting info
							 * */
#define CMD_READCONFIG				0x0B	/* /< Read The config
							 * memory, need to pass:
							 * addr[0] addr[1]
							 * byteToRead0
							 * byteToRead1 */


/* GUI utils byte ver */
#define CMD_READ_BYTE				0xF0	/* /< Byte output
							 * version of I2C/SPI
							 * read @see CMD_READ */
#define CMD_WRITE_BYTE				0xF1	/* /< Byte output
							 * version of I2C/SPI
							 * write @see CMD_WRITE
							 * */
#define CMD_WRITEREAD_BYTE			0xF2	/* /< Byte output
							 * version of I2C/SPI
							 * writeRead @see
							 * CMD_WRITEREAD */
#define CMD_WRITETHENWRITEREAD_BYTE		0xF3
						/* /< Byte output
						 * version of I2C/SPI
						 * write then writeRead
						 * @see
						 * CMD_WRITETHENWRITEREAD
						 * */
#define CMD_WRITEU8UX_BYTE			0xF4	/* /< Byte output
							 * version of I2C/SPI
							 * writeU8UX @see
							 * CMD_WRITEU8UX */
#define CMD_WRITEREADU8UX_BYTE			0xF5	/* /< Byte output
							 * version of I2C/SPI
							 * writeReadU8UX @see
							 * CMD_WRITEREADU8UX */
#define CMD_WRITEU8UXTHENWRITEU8UX_BYTE		0xF6
						/* /< Byte output
						 * version of I2C/SPI
						 * writeU8UX then
						 * writeU8UX @see
						 * CMD_WRITEU8UXTHENWRITEU8UX
						 * */
#define CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE	0xF7
						/* /< Byte output
						* version of I2C/SPI
						* writeU8UX  then
						* writeReadU8UX @see
						* CMD_WRITEU8UXTHENWRITEREADU8UX
						* */
#define CMD_GETLIMITSFILE_BYTE			0xF8	/* /< Byte output
							 * version of Production
							 * Limits File @see
							 * CMD_GETLIMITSFILE */
#define CMD_GETFWFILE_BYTE			0xF9	/* /< Byte output
							 * version of FW file
							 * need to pass: @see
							 * CMD_GETFWFILE */
#define CMD_VERSION_BYTE			0xFA	/* /< Byte output
							 * version of driver
							 * version and setting
							 * @see CMD_VERSION */
#define CMD_CHANGE_OUTPUT_MODE			0xFF	/* /< Select the output
							 * mode of the
							 * scriptless protocol,
							 * need to pass:
							 * bin_output = 1 data
							 * returned as binary,
							 * bin_output =0 data
							 * returned as hex
							 * string */

/* Core/Tools */
#define CMD_POLLFOREVENT			0x11	/* /< Poll the FIFO for
							 * an event: need to
							 * pass: eventLength
							 * event[0] event[1] …
							 * event[eventLength-1]
							 * timeToWait1
							 * timeToWait0 */
#define CMD_SYSTEMRESET				0x12	/* /< System Reset */
#define CMD_CLEANUP				0x13	/* /< Perform a system
							 * reset and optionally
							 * re-enable the
							 * scanning, need to
							 * pass: enableTouch */
#define CMD_POWERCYCLE				0x14	/* /< Execute a power
							 * cycle toggling the
							 * regulators */
#define CMD_READSYSINFO				0x15	/* /< Read the System
							 * Info information from
							 * the framebuffer, need
							 * to pass: doRequest */
#define CMD_FWWRITE				0x16
						/* /< Write a FW
						 * command: need to
						 * pass: cmd[0]  cmd[1]
						 * … cmd[cmdLength-1] */
#define CMD_INTERRUPT				0x17	/* /< Allow to enable or
							 * disable the
							 * interrupts, need to
							 * pass: enable (if 1
							 * will enable the
							 * interrupt) */
#define CMD_SETSCANMODE				0x18	/* /< set Scan Mode
							 * need to pass:
							 * scanType option
							 */
#define CMD_SAVEMPFLAG				0x19	/* /< save manually a
							 * value in the MP flag
							 * need to pass: mpflag
							 */

/* Frame */
#define CMD_GETFORCELEN				0x20	/* /< Get the number of
							 * Force channels */
#define CMD_GETSENSELEN				0x21	/* /< Get the number of
							 * Sense channels */
#define CMD_GETMSFRAME				0x23	/* /< Get a MS frame:
							 * need to pass:
							 * MSFrameType */
#define CMD_GETSSFRAME				0x24	/* /< Get a SS frame:
							 * need to pass:
							 * SSFrameType */
#define CMD_GETSYNCFRAME			0x25	/* /< Get a SS frame:
							 * need to pass:
							 * SSFrameType
							 */

/* Compensation */
#define CMD_REQCOMPDATA				0x30	/* /< Request Init data:
							 * need to pass: type */
#define CMD_READCOMPDATAHEAD			0x31	/* /< Read Init data
							 * header: need to pass:
							 * type */
#define CMD_READMSCOMPDATA			0x32	/* /< Read MS Init data:
							 * need to pass: type */
#define CMD_READSSCOMPDATA			0x33	/* /< Read SS Init data:
							 * need to pass: type */
#define CMD_READTOTMSCOMPDATA			0x35	/* /< Read Tot MS Init
							 * data: need to pass:
							 * type */
#define CMD_READTOTSSCOMPDATA			0x36	/* /< Read Tot SS Init
							 * data: need to pass:
							 * type */
#define CMD_READSENSCOEFF			0x37	/* /< Read MS and SS
							 * Sensitivity
							 * Calibration
							 * Coefficients */

/* FW Update */
#define CMD_GETFWVER				0x40	/* /< Get the FW version
							 * of the IC */
#define CMD_FLASHUNLOCK				0x42	/* /< Unlock the flash
							 * */
#define CMD_READFWFILE				0x43	/* /< Try to read the FW
							 * file, need to pass:
							 * keep_cx */
#define CMD_FLASHPROCEDURE			0x44	/* /< Perform a full
							 * flashing procedure:
							 * need to pass: force
							 * keep_cx */
#define CMD_FLASHERASEUNLOCK			0x45	/* /< Unlock the erase
							 * of the flash */
#define CMD_FLASHERASEPAGE			0x46
						/* /< Erase page by page
						 * the flash, need to
						 * pass: keep_cx, if
						 * keep_cx>SKIP_PANEL_INIT
						 * Panel Init Page will
						 * be skipped, if
						 * >SKIP_PANEL_CX_INIT
						 * Cx and Panel Init
						 * Pages will be skipped
						 * otherwise if
						 * =ERASE_ALL all the
						 * pages will be deleted
						 * */

/* MP test */
#define CMD_ITOTEST				0x50	/* /< Perform an ITO
							 * test */
#define CMD_INITTEST				0x51	/* /< Perform an
							 * Initialization test:
							 * need to pass: type */
#define CMD_MSRAWTEST				0x52	/* /< Perform MS raw
							 * test: need to pass
							 * stop_on_fail */
#define CMD_MSINITDATATEST			0x53	/* /< Perform MS Init
							 * data test: need to
							 * pass stop_on_fail */
#define CMD_SSRAWTEST				0x54	/* /< Perform SS raw
							 * test: need to pass
							 * stop_on_fail */
#define CMD_SSINITDATATEST			0x55	/* /< Perform SS Init
							 * data test: need to
							 * pass stop_on_fail */
#define CMD_MAINTEST				0x56	/* /< Perform a full
							 * Mass production test:
							 * need to pass
							 * stop_on_fail saveInit
							 * */
#define CMD_FREELIMIT				0x57	/* /< Free (if any)
							 * limit file which was
							 * loaded during any
							 * test procedure */

/* Diagnostic */
#define CMD_DIAGNOSTIC				0x60	/* /< Perform a series
							 * of commands and
							 * collect severals data
							 * to detect any
							 * malfunction */

#define CMD_CHANGE_SAD				0x70	/* /< Allow to change
							 * the SAD address (for
							 * debugging) */

/* Debug functionalities requested by Google for B1 Project */
#define CMD_TRIGGER_FORCECAL			0x80	/* /< Trigger manually
							 * forcecal for MS and
							 * SS */
#define CMD_BASELINE_ADAPTATION			0x81	/* /< Enable/Disable
							 * Baseline adaptation,
							 * need to pass: enable
							 * */
#define CMD_FREQ_HOP				0x82	/* /< Enable/Disable
							 * Frequency hopping,
							 * need to pass: enable
							 * */
#define CMD_SET_OPERATING_FREQ			0x83	/* /< Set a defined
							 * scanning frequency in
							 * Hz passed as 4 bytes
							 * in big endian, need
							 * to pass: freq3 freq2
							 * freq1 freq0 */
#define CMD_READ_SYNC_FRAME			0x84
						/* /< Read Sync Frame
						 * which contain MS and
						 * SS data, need to
						 * pass: frameType (this
						 * parameter can be
						 * LOAD_SYNC_FRAME_STRENGTH
						 * or LOAD_SYNC_FRAME_BASELINE)
						 * */


#define CMD_TP_SENS_MODE			0x90	/* /< Enter/Exit in the
							 * TP Sensitivity
							 * Calibration mode,
							 * need to pass: enter
							 * (optional)saveGain */
#define CMD_TP_SENS_SET_SCAN_MODE		0x91	/* /< Set scan mode type
							 * which should be used
							 * for the test before
							 * the stimpad is down,
							 * need to pass: type */
#define CMD_TP_SENS_PRECAL_SS			0x92	/* /< Perform Pre
							 * Calibration for SS
							 * steps when stimpad is
							 * down */
#define CMD_TP_SENS_PRECAL_MS			0x93	/* /< Perform Pre
							 * Calibration for MS
							 * steps when stimpad is
							 * down */
#define CMD_TP_SENS_POSTCAL_MS			0x94	/* /< Perform Post
							 * Calibration for MS
							 * steps when stimpad is
							 * down */
#define CMD_TP_SENS_STD				0x95	/* /< Compute the
							 * Standard deviation of
							 * a certain number of
							 * frames, need to pass:
							 * numFrames */

#define CMD_FORCE_TOUCH_ACTIVE			0xA0	/* /< Prevent the driver
							 * from transitioning
							 * the ownership of the
							 * bus to SLPI
							 */

static u8 bin_output;		/* /< Select the output type of the scriptless
				 * protocol (binary = 1  or hex string = 0) */
/** @}*/

/** @defgroup scriptless Scriptless Protocol
  * @ingroup proc_file_code
  * Scriptless Protocol allows ST Software (such as FingerTip Studio etc) to
  * communicate with the IC from an user space.
  * This mode gives access to common bus operations (write, read etc) and
  * support additional functionalities. \n
  * The protocol is based on exchange of binary messages included between a
  * start and an end byte
  * @{
  */

#define MESSAGE_START_BYTE	0x7B	/* /< start byte of each message
					 * transferred in Scriptless Mode */
#define MESSAGE_END_BYTE	0x7D	/* /< end byte of each message
					 * transferred in Scriptless Mode */
#define MESSAGE_MIN_HEADER_SIZE 8	/* /< minimun number of bytes of the
					 * structure of a messages exchanged
					 * with host (include start/end byte,
					 * counter, actions, msg_size) */

/**
  * Possible actions that can be requested by an host
  */
typedef enum {
	ACTION_WRITE				= (u16) 0x0001,	/* /< Bus Write
								 * */
	ACTION_READ				= (u16) 0x0002,	/* /< Bus Read
								 * */
	ACTION_WRITE_READ			= (u16) 0x0003,	/* /< Bus Write
								 * followed by a
								 * Read */
	ACTION_GET_VERSION			= (u16) 0x0004,	/* /< Get
								 * Version of
								 * the protocol
								 * (equal to the
								 * first 2 bye
								 * of driver
								 * version) */
	ACTION_WRITEU8UX			= (u16) 0x0011,	/* /< Bus Write
								 * with support
								 * to different
								 * address size
								 * */
	ACTION_WRITEREADU8UX			= (u16) 0x0012,	/* /< Bus
								 * writeRead
								 * with support
								 * to different
								 * address size
								 * */
	ACTION_WRITETHENWRITEREAD		= (u16) 0x0013,	/* /< Bus write
								 * followed by a
								 * writeRead */
	ACTION_WRITEU8XTHENWRITEREADU8UX	= (u16) 0x0014,	/* /< Bus write
								 * followed by a
								 * writeRead
								 * with support
								 * to different
								 * address size
								 * */
	ACTION_WRITEU8UXTHENWRITEU8UX		= (u16) 0x0015,	/* /< Bus write
								 * followed by a
								 * write with
								 * support to
								 * different
								 * address size
								 * */
	ACTION_GET_FW				= (u16) 0x1000,	/* /< Get Fw
								 * file content
								 * used by the
								 * driver */
	ACTION_GET_LIMIT			= (u16) 0x1001	/* /< Get Limit
								 * File content
								 * used by the
								 * driver */
} Actions;

/**
  * Struct used to contain info of the message received by the host in
  * Scriptless mode
  */
typedef struct {
	u16 msg_size;	/* /< total size of the message in bytes */
	u16 counter;	/* /< counter ID to identify a message */
	Actions action;	/* /< type of operation requested by the host @see
			 * Actions */
	u8 dummy;	/* /< (optional)in case of any kind of read operations,
			 * specify if the first byte is dummy */
} Message;

/** @}*/



extern TestToDo tests;
extern SysInfo systemInfo;

static int limit;	/* /< store the amount of data to print into the
			 * shell */
static int chunk;	/* /< store the chuk of data that should be printed in
			 * this iteration */
static int printed;	/* /< store the amount of data already printed in the
			 * shell */
static struct proc_dir_entry *fts_dir;		/* /< reference to the directory
						 * fts under /proc */
static u8 *driver_test_buff;		/* /< pointer to an array of bytes used
					 * to store the result of the function
					 * executed */
char buf_chunk[CHUNK_PROC] = { 0 };	/* /< buffer used to store the message
					 * info received */
static Message mess = { 0 };	/* /< store the information of the Scriptless
				 * message received */


/************************ SEQUENTIAL FILE UTILITIES **************************/
/**
  * This function is called at the beginning of the stream to a sequential file
  * or every time into the sequential were already written PAGE_SIZE bytes and
  * the stream need to restart
  * @param s pointer to the sequential file on which print the data
  * @param pos pointer to the offset where write the data
  * @return NULL if there is no data to print or the pointer to the beginning of
  * the data that need to be printed
  */
static void *fts_seq_start(struct seq_file *s, loff_t *pos)
{
	pr_info("%s: Entering start(), pos = %lld limit = %d printed = %d\n",
		__func__, *pos, limit, printed);

	if (driver_test_buff == NULL && *pos == 0) {
		int size = 13 * sizeof(u8);

		pr_info("%s: No data to print!\n", __func__);
		driver_test_buff = (u8 *)kmalloc(size, GFP_KERNEL);
		limit = scnprintf(driver_test_buff,
				  size,
				  "{ %08X }\n", ERROR_OP_NOT_ALLOW);
		/* pr_err("%s: len = %d driver_test_buff = %s\n",
		 * __func__, limit, driver_test_buff); */
	} else {
		if (*pos != 0)
			*pos += chunk - 1;

		if (*pos >= limit)
			/* pr_err("%s: Apparently, we're done.\n", __func__); */
			return NULL;
	}

	chunk = CHUNK_PROC;
	if (limit - *pos < CHUNK_PROC)
		chunk = limit - *pos;
	/* pr_err("%s: In start(),
	 *	updated pos = %Ld limit = %d printed = %d chunk = %d\n",
	 *	__func__, *pos, limit, printed, chunk); */
	memset(buf_chunk, 0, CHUNK_PROC);
	memcpy(buf_chunk, &driver_test_buff[(int)*pos], chunk);

	return buf_chunk;
}

/**
  * This function actually print a chunk amount of data in the sequential file
  * @param s pointer to the sequential file where to print the data
  * @param v pointer to the data to print
  * @return 0
  */
static int fts_seq_show(struct seq_file *s, void *v)
{
	/* pr_err("%s: In show()\n", __func__); */
	seq_write(s, (u8 *)v, chunk);
	printed += chunk;
	return 0;
}

/**
  * This function update the pointer and the counters to the next data to be
  * printed
  * @param s pointer to the sequential file where to print the data
  * @param v pointer to the data to print
  * @param pos pointer to the offset where write the next data
  * @return NULL if there is no data to print or the pointer to the beginning of
  * the next data that need to be printed
  */
static void *fts_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	/* int* val_ptr; */
	/* pr_err("%s: In next(), v = %X, pos = %Ld.\n", __func__,
	 * v, *pos); */
	(*pos) += chunk;/* increase my position counter */
	chunk = CHUNK_PROC;

	/* pr_err("%s: In next(),
	 *	updated pos = %Ld limit = %d printed = %d\n",
	 *	__func__, *pos, limit,printed); */
	if (*pos >= limit)	/* are we done? */
		return NULL;
	else if (limit - *pos < CHUNK_PROC)
		chunk = limit - *pos;


	memset(buf_chunk, 0, CHUNK_PROC);
	memcpy(buf_chunk, &driver_test_buff[(int)*pos], chunk);
	return buf_chunk;
}


/**
  * This function is called when there are no more data to print  the stream
  *need to be terminated or when PAGE_SIZE data were already written into the
  *sequential file
  * @param s pointer to the sequential file where to print the data
  * @param v pointer returned by fts_seq_next
  */
static void fts_seq_stop(struct seq_file *s, void *v)
{
	/* pr_err("%s: Entering stop().\n", __func__); */

	if (v) {
		/* pr_err("%s: v is %X.\n", __func__, v); */
	} else {
		/* pr_err("%s: v is null.\n", __func__); */
		limit = 0;
		chunk = 0;
		printed = 0;
		if (driver_test_buff != NULL) {
		/* pr_err("%s: Freeing and clearing driver_test_buff.\n",
		 *   __func__); */
			kfree(driver_test_buff);
			driver_test_buff = NULL;
		} else {
		/* pr_err("%s: driver_test_buff is already null.\n",
		 *   __func__); */
		}
	}
}

/**
  * Struct where define and specify the functions which implements the flow for
  * writing on a sequential file
  */
static const struct seq_operations fts_seq_ops = {
	.start	= fts_seq_start,
	.next	= fts_seq_next,
	.stop	= fts_seq_stop,
	.show	= fts_seq_show
};

/**
  * This function open a sequential file
  * @param inode Inode in the file system that was called and triggered this
  * function
  * @param file file associated to the file node
  * @return error code, 0 if success
  */
static int fts_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &fts_seq_ops);
};


/*****************************************************************************/

/**************************** DRIVER TEST ************************************/

/** @addtogroup proc_file_code
  * @{
  */

/**
  * Receive the OP code and the inputs from shell when the file node is called,
  * parse it and then execute the corresponding function
  * echo cmd+parameters > /proc/fts/driver_test to execute the select command
  * cat /proc/fts/driver_test			to obtain the result into the
  * shell \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * the answer content and format strictly depend on the cmd executed. In
  * general can be: an HEX string or a byte array (e.g in case of 0xF- commands)
  * \n
  * } = end byte \n
  */
static ssize_t fts_driver_test_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
	int numberParam = 0;
	struct fts_ts_info *info = dev_get_drvdata(getDev());
	char *p = NULL;
	char pbuf[count];
	char path[100] = { 0 };
	int res = -1, j, index = 0;
	u8 report = 0;
	int size = 6;
	int temp, byte_call = 0;
	u16 byteToRead = 0;
	u32 fileSize = 0;
	u8 *readData = NULL;
	u8 *cmd = NULL;	/* worst case needs count bytes */
	u32 funcToTest[((count + 1) / 3)];
	u64 addr = 0;
	MutualSenseFrame frameMS;
	MutualSenseFrame deltas;
	SelfSenseFrame frameSS;

	DataHeader dataHead;
	MutualSenseData compData;
	SelfSenseData comData;
	TotMutualSenseData totCompData;
	TotSelfSenseData totComData;
	MutualSenseCoeff msCoeff;
	SelfSenseCoeff ssCoeff;
	int meanNorm = 0, meanEdge = 0;

	u64 address;

	Firmware fw;
	LimitFile lim;

	mess.dummy = 0;
	mess.action = 0;
	mess.msg_size = 0;

	if (fts_set_bus_ref(info, FTS_BUS_REF_SYSFS, true) < 0) {
		res = ERROR_BUS_WR;
		pr_err("%s: bus is not accessible.\n", __func__);
		if (driver_test_buff)
			limit = scnprintf(driver_test_buff, size, "{ %08X }\n",
					  res);
		fts_set_bus_ref(info, FTS_BUS_REF_SYSFS, false);
		goto ERROR;
	}

	/*for(temp = 0; temp<count; temp++){
	  *      pr_err("p[%d] = %02X\n", temp, p[temp]);
	  * }*/
	if (access_ok(VERIFY_READ, buf, count) < OK ||
	    copy_from_user(pbuf, buf, count) != 0) {
		res = ERROR_ALLOC;
		goto END;
	}

	cmd = (u8 *)kmalloc_array(count, sizeof(u8), GFP_KERNEL);
	if (cmd == NULL) {
		res = ERROR_ALLOC;
		pr_err("%s: Impossible allocate memory... ERROR %08X!\n",
			__func__, res);
		goto ERROR;
	}

	p = pbuf;
	if (count > MESSAGE_MIN_HEADER_SIZE - 1 && p[0] == MESSAGE_START_BYTE) {
		pr_info("Enter in Byte Mode!\n");
		byte_call = 1;
		mess.msg_size = (p[1] << 8) | p[2];
		mess.counter = (p[3] << 8) | p[4];
		mess.action = (p[5] << 8) | p[6];
		pr_info("Message received: size = %d, counter_id = %d, action = %04X\n",
			mess.msg_size, mess.counter, mess.action);
		size = MESSAGE_MIN_HEADER_SIZE + 2;	/* +2 error code */
		if (count < mess.msg_size || p[count - 2] != MESSAGE_END_BYTE) {
			pr_err("number of byte received or end byte wrong! msg_size = %d != %zu, last_byte = %02X != %02X ... ERROR %08X\n",
				mess.msg_size, count, p[count - 1],
				MESSAGE_END_BYTE, ERROR_OP_NOT_ALLOW);
			res = ERROR_OP_NOT_ALLOW;
			goto END;
		} else {
			numberParam = mess.msg_size - MESSAGE_MIN_HEADER_SIZE +
				      1;	/* +1 because put the internal
						 * op code */
			size = MESSAGE_MIN_HEADER_SIZE + 2;	/* +2 send also
								 * the first 2
								 * lsb of the
								 * error code */
			switch (mess.action) {
			case ACTION_READ:
				/* numberParam =
				 * mess.msg_size-MESSAGE_MIN_HEADER_SIZE+1; */
				cmd[0] = funcToTest[0] = CMD_READ_BYTE;
				break;

			case ACTION_WRITE:
				cmd[0] = funcToTest[0] = CMD_WRITE_BYTE;
				break;

			case ACTION_WRITE_READ:
				cmd[0] = funcToTest[0] = CMD_WRITEREAD_BYTE;
				break;

			case ACTION_GET_VERSION:
				cmd[0] = funcToTest[0] = CMD_VERSION_BYTE;
				break;

			case ACTION_WRITETHENWRITEREAD:
				cmd[0] = funcToTest[0] =
						 CMD_WRITETHENWRITEREAD_BYTE;
				break;

			case ACTION_WRITEU8UX:
				cmd[0] = funcToTest[0] = CMD_WRITEU8UX_BYTE;
				break;

			case ACTION_WRITEREADU8UX:
				cmd[0] = funcToTest[0] = CMD_WRITEREADU8UX_BYTE;
				break;

			case ACTION_WRITEU8UXTHENWRITEU8UX:
				cmd[0] = funcToTest[0] =
					 CMD_WRITEU8UXTHENWRITEU8UX_BYTE;
				break;

			case ACTION_WRITEU8XTHENWRITEREADU8UX:
				cmd[0] = funcToTest[0] =
					 CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE;
				break;

			case ACTION_GET_FW:
				cmd[0] = funcToTest[0] = CMD_GETFWFILE_BYTE;
				break;

			case ACTION_GET_LIMIT:
				cmd[0] = funcToTest[0] = CMD_GETLIMITSFILE_BYTE;
				break;

			default:
				pr_err("Invalid Action = %d ... ERROR %08X\n",
					mess.action, ERROR_OP_NOT_ALLOW);
				res = ERROR_OP_NOT_ALLOW;
				goto END;
			}

			if (numberParam - 1 != 0)
				memcpy(&cmd[1], &p[7], numberParam - 1);
			/* -1 because i need to exclude the cmd[0] */
		}
	} else {
		if (((count + 1) / 3) >= 1) {
			if (sscanf(p, "%02X ", &funcToTest[0]) == 1) {
				p += 3;
				cmd[0] = (u8)funcToTest[0];
				numberParam = 1;
			}
		} else {
			res = ERROR_OP_NOT_ALLOW;
			goto END;
		}

		pr_info("functionToTest[0] = %02X cmd[0]= %02X\n",
			 funcToTest[0], cmd[0]);
		switch (funcToTest[0]) {
		case CMD_GETFWFILE:
		case CMD_GETLIMITSFILE:
			if (count - 2 - 1 > 1) {
				if (sscanf(p, "%100s", path) == 1) {
					numberParam = 2;
					/* the first byte is an hex string
					 * coded in three byte (2  chars for
					 * hex and the space) and -1 for the
					 * space at the end
					 */
				}
			}
			break;

		default:
			for (; numberParam < (count + 1) / 3; numberParam++) {
				if (sscanf(p, "%02X ",
					&funcToTest[numberParam]) == 1) {
					p += 3;
					cmd[numberParam] =
						(u8)funcToTest[numberParam];
					pr_info("functionToTest[%d] = %02X cmd[%d]= %02X\n",
					numberParam, funcToTest[numberParam],
					numberParam, cmd[numberParam]);
				}
			}
		}
	}


	fw.data = NULL;
	lim.data = NULL;

	pr_info("Number of Parameters = %d\n", numberParam);

	/* elaborate input */
	if (numberParam >= 1) {
		switch (funcToTest[0]) {
		case CMD_VERSION_BYTE:
			pr_info("%s: Get Version Byte\n", __func__);
			byteToRead = 2;
			mess.dummy = 0;
			readData = (u8 *)kmalloc(byteToRead * sizeof(u8),
						 GFP_KERNEL);
			size += byteToRead;
			if (readData != NULL) {
				readData[0] = (u8)(FTS_TS_DRV_VER >> 24);
				readData[1] = (u8)(FTS_TS_DRV_VER >> 16);
				pr_info("%s: Version = %02X%02X\n",
					 __func__, readData[0], readData[1]);
				res = OK;
			} else {
				res = ERROR_ALLOC;
				pr_err("%s: Impossible allocate memory... ERROR %08X\n",
					__func__, res);
			}
			break;


		case CMD_VERSION:
			byteToRead = 2 * sizeof(u32);
			mess.dummy = 0;
			readData = (u8 *)kmalloc(byteToRead * sizeof(u8),
						 GFP_KERNEL);
			u32ToU8_be(FTS_TS_DRV_VER, readData);
			fileSize = 0;
			/* first two bytes bitmask of features enabled in the
			 * IC, second two bytes bitmask of features enabled in
			 * the driver */

#ifdef FW_H_FILE
			fileSize |= 0x00010000;
#endif

#ifdef LIMITS_H_FILE
			fileSize |= 0x00020000;
#endif

#ifdef USE_ONE_FILE_NODE
			fileSize |= 0x00040000;
#endif

#ifdef FW_UPDATE_ON_PROBE
			fileSize |= 0x00080000;
#endif

#ifdef PRE_SAVED_METHOD
			fileSize |= 0x00100000;
#endif

#ifdef COMPUTE_INIT_METHOD
			fileSize |= 0x00200000;
#endif

#ifdef USE_GESTURE_MASK
			fileSize |= 0x00100000;
#endif

#ifdef I2C_INTERFACE
			fileSize |= 0x00200000;
#endif

#ifdef SPI4_WIRE
			fileSize |= 0x00400000;
#endif

#ifdef PHONE_KEY	/* it is a feature enabled in the config of the chip */
			fileSize |= 0x00000100;
#endif

#ifdef GESTURE_MODE
			fromIDtoMask(FEAT_SEL_GESTURE, (u8 *)&fileSize, 4);
#endif


#ifdef GRIP_MODE
			fromIDtoMask(FEAT_SEL_GRIP, (u8 *)&fileSize, 4);
#endif

#ifdef CHARGER_MODE
			fromIDtoMask(FEAT_SEL_CHARGER, (u8 *)&fileSize, 4);
#endif

#ifdef GLOVE_MODE
			fromIDtoMask(FEAT_SEL_GLOVE, (u8 *)&fileSize, 4);
#endif


#ifdef COVER_MODE
			fromIDtoMask(FEAT_SEL_COVER, (u8 *)&fileSize, 4);
#endif

#ifdef STYLUS_MODE
			fromIDtoMask(FEAT_SEL_STYLUS, (u8 *)&fileSize, 4);
#endif

			u32ToU8_be(fileSize, &readData[4]);
			res = OK;
			size += (byteToRead * sizeof(u8));
			break;

		case CMD_WRITEREAD:
		case CMD_WRITEREAD_BYTE:
			if (numberParam >= 5) {	/* need to pass: cmd[0]  cmd[1]
						 * … cmd[cmdLength-1]
						 * byteToRead1 byteToRead0
						 * dummyByte */
				temp = numberParam - 4;
				if (cmd[numberParam - 1] == 0)
					mess.dummy = 0;
				else
					mess.dummy = 1;

				u8ToU16_be(&cmd[numberParam - 3], &byteToRead);
				pr_info("bytesToRead = %d\n",
					byteToRead + mess.dummy);

				readData = (u8 *)kmalloc((byteToRead +
							  mess.dummy) *
							 sizeof(u8),
							 GFP_KERNEL);
				res = fts_writeRead_heap(&cmd[1], temp,
					readData, byteToRead + mess.dummy);
				size += (byteToRead * sizeof(u8));
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITE:
		case CMD_WRITE_BYTE:
			if (numberParam >= 2) {	/* need to pass: cmd[0]  cmd[1]
						 * … cmd[cmdLength-1] */
				temp = numberParam - 1;

				res = fts_write_heap(&cmd[1], temp);
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READ:
		case CMD_READ_BYTE:
			if (numberParam >= 3) {	/* need to pass: byteToRead1
						 * byteToRead0 (optional)
						 * dummyByte */
				if (numberParam == 3 ||
				     (numberParam == 4 &&
				      cmd[numberParam - 1] == 0))
					mess.dummy = 0;
				else
					mess.dummy = 1;
				u8ToU16_be(&cmd[1], &byteToRead);
				readData = (u8 *)kmalloc((byteToRead +
							  mess.dummy) *
							 sizeof(u8),
							 GFP_KERNEL);
				res = fts_read_heap(readData, byteToRead +
						mess.dummy);
				size += (byteToRead * sizeof(u8));
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITETHENWRITEREAD:
		case CMD_WRITETHENWRITEREAD_BYTE:
			/* need to pass: cmdSize1 cmdSize2 cmd1[0] cmd1[1] …
			 * cmd1[cmdSize1-1] cmd2[0] cmd2[1] … cmd2[cmdSize2-1]
			 *  byteToRead1 byteToRead0 */
			if (numberParam >= 6) {
				u8ToU16_be(&cmd[numberParam - 2], &byteToRead);
				readData = (u8 *)kmalloc(byteToRead *
							 sizeof(u8),
							 GFP_KERNEL);
				res = fts_writeThenWriteRead_heap(
						&cmd[3], cmd[1],
						&cmd[3 + (int)cmd[1]], cmd[2],
						readData, byteToRead);
				size += (byteToRead * sizeof(u8));
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITEU8UX:
		case CMD_WRITEU8UX_BYTE:
			/* need to pass:
			 *    cmd addrSize addr[0] … addr[addrSize-1]
			 *    data[0] data[1] … */
			if (numberParam >= 4) {
				if (cmd[2] <= sizeof(u64)) {
					u8ToU64_be(&cmd[3], &addr, cmd[2]);
					pr_info("addr = %llx\n", addr);
					res = fts_writeU8UX(cmd[1], cmd[2],
						addr,
						&cmd[3 + cmd[2]],
						(numberParam - cmd[2] - 3));
				} else {
					pr_err("Wrong address size!\n");
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_WRITEREADU8UX:
		case CMD_WRITEREADU8UX_BYTE:
			/* need to pass:
			 *    cmd addrSize addr[0] … addr[addrSize-1]
			 *    byteToRead1 byteToRead0 hasDummyByte */
			if (numberParam >= 6) {
				if (cmd[2] <= sizeof(u64)) {
					u8ToU64_be(&cmd[3], &addr, cmd[2]);
					u8ToU16_be(&cmd[numberParam - 3],
						   &byteToRead);
					readData = (u8 *)kmalloc(byteToRead *
								 sizeof(u8),
								 GFP_KERNEL);
					pr_info("addr = %llx byteToRead = %d\n",
						addr, byteToRead);
					res = fts_writeReadU8UX(cmd[1], cmd[2],
							addr, readData,
							byteToRead,
							cmd[numberParam - 1]);
					size += (byteToRead * sizeof(u8));
				} else {
					pr_err("Wrong address size!\n");
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITEU8UXTHENWRITEU8UX:
		case CMD_WRITEU8UXTHENWRITEU8UX_BYTE:
			/* need to pass:
			 *    cmd1 addrSize1 cmd2 addrSize2 addr[0] …
			 *    addr[addrSize1+addrSize2-1] data[0] data[1] … */
			if (numberParam >= 6) {
				if ((cmd[2] + cmd[4]) <= sizeof(u64)) {
					u8ToU64_be(&cmd[5], &addr, cmd[2] +
						   cmd[4]);

					pr_info("addr = %llx\n", addr);
					res = fts_writeU8UXthenWriteU8UX(cmd[1],
						cmd[2], cmd[3],
						cmd[4], addr,
						&cmd[5 + cmd[2] + cmd[4]],
						(numberParam - cmd[2]
							- cmd[4] - 5));
				} else {
					pr_err("Wrong address size!\n");
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITEU8UXTHENWRITEREADU8UX:
		case CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE:
			/* need to pass:
			 * cmd1 addrSize1 cmd2 addrSize2 addr[0] …
			 * addr[addrSize1+addrSize2-1]  byteToRead1 byteToRead0
			 * hasDummybyte */
			if (numberParam >= 8) {
				if ((cmd[2] + cmd[4]) <= sizeof(u64)) {
					u8ToU64_be(&cmd[5], &addr, cmd[2] +
						   cmd[4]);
					pr_info("%s: cmd[5] = %02X, addr =  %llx\n",
						__func__, cmd[5], addr);
					u8ToU16_be(&cmd[numberParam - 3],
						   &byteToRead);
					readData = (u8 *)kmalloc(byteToRead *
								 sizeof(u8),
								 GFP_KERNEL);
					res = fts_writeU8UXthenWriteReadU8UX(
						cmd[1], cmd[2], cmd[3], cmd[4],
						addr,
						readData, byteToRead,
						cmd[numberParam - 1]);
					size += (byteToRead * sizeof(u8));
				} else {
					pr_err("Wrong total address size!\n");
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_CHANGE_OUTPUT_MODE:
			/* need to pass: bin_output */
			if (numberParam >= 2) {
				bin_output = cmd[1];
				pr_info("Setting Scriptless output mode: %d\n",
					bin_output);
				res = OK;
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FWWRITE:
			if (numberParam >= 3) {	/* need to pass: cmd[0]  cmd[1]
						 * … cmd[cmdLength-1] */
				if (numberParam >= 2) {
					temp = numberParam - 1;
					res = fts_writeFwCmd_heap(&cmd[1],
								temp);
				} else {
					pr_err("Wrong parameters!\n");
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_INTERRUPT:
			/* need to pass: enable */
			if (numberParam >= 2) {
				if (cmd[1] == 1)
					res = fts_enableInterrupt();
				else
					res = fts_disableInterrupt();
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SETSCANMODE:
			/* need to pass: scanMode option */
			if (numberParam >= 3)
				res = setScanMode(cmd[1], cmd[2]);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SAVEMPFLAG:
			/* need to pass: mpflag */
			if (numberParam == 2)
				res = saveMpFlag(cmd[1]);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READCONFIG:
			if (numberParam == 5) {	/* need to pass: addr[0]
						 *  addr[1] byteToRead1
						 * byteToRead0 */
				byteToRead = ((funcToTest[3] << 8) |
					      funcToTest[4]);
				readData = (u8 *)kmalloc(byteToRead *
							 sizeof(u8),
							 GFP_KERNEL);
				res = readConfig((u16)((((u8)funcToTest[1] &
								0x00FF) << 8) +
						       ((u8)funcToTest[2] &
								0x00FF)),
						 readData, byteToRead);
				size += (byteToRead * sizeof(u8));
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_POLLFOREVENT:
			if (numberParam >= 5) {	/* need to pass: eventLength
						 * event[0] event[1] …
						 * event[eventLength-1]
						 * timeTowait1 timeTowait0 */
				temp = (int)funcToTest[1];
				if (numberParam == 5 + (temp - 1) &&
					temp != 0) {
					readData = (u8 *)kmalloc(
						FIFO_EVENT_SIZE * sizeof(u8),
						GFP_KERNEL);
					res = pollForEvent(
						(int *)&funcToTest[2], temp,
						readData,
						((funcToTest[temp + 2] &
							0x00FF) << 8) +
						(funcToTest[temp + 3] &
							0x00FF));
					if (res >= OK)
						res = OK;	/* pollForEvent
								 * return the
								 * number of
								 * error found
								 * */
					size += (FIFO_EVENT_SIZE * sizeof(u8));
					byteToRead = FIFO_EVENT_SIZE;
				} else {
					pr_err("Wrong parameters!\n");
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SYSTEMRESET:
			res = fts_system_reset();

			break;

		case CMD_READSYSINFO:
			if (numberParam == 2)	/* need to pass: doRequest */
				res = readSysInfo(funcToTest[1]);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_CLEANUP:/* TOUCH ENABLE/DISABLE */
			if (numberParam == 2)	/* need to pass: enableTouch */
				res = cleanUp(funcToTest[1]);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_GETFORCELEN:	/* read number Tx channels */
			temp = getForceLen();
			if (temp < OK)
				res = temp;
			else {
				size += (1 * sizeof(u8));
				res = OK;
			}
			break;

		case CMD_GETSENSELEN:	/* read number Rx channels */
			temp = getSenseLen();
			if (temp < OK)
				res = temp;
			else {
				size += (1 * sizeof(u8));
				res = OK;
			}
			break;


		case CMD_GETMSFRAME:
			if (numberParam == 2) {
				pr_info("Get 1 MS Frame\n");
				/* setScanMode(SCAN_MODE_ACTIVE, 0xFF);
				 * mdelay(WAIT_FOR_FRESH_FRAMES);
				 * setScanMode(SCAN_MODE_ACTIVE, 0x00);
				 * mdelay(WAIT_AFTER_SENSEOFF);
				 */
				/* flushFIFO(); //delete the events related to
				 * some touch (allow to call this function while
				 * touching the screen without having a flooding
				 * of the FIFO) */
				res = getMSFrame3((MSFrameType)cmd[1],
						  &frameMS);
				if (res < 0)
					pr_err("Error while taking the MS frame... ERROR %08X\n",
						res);

				else {
					pr_info("The frame size is %d words\n",
						res);
					size += (res * sizeof(short) + 2);
					/* +2 to add force and sense channels
					 * set res to OK because if getMSFrame
					 * is successful
					 *	res = number of words read
					 */
					res = OK;
					print_frame_short("MS frame =",
						array1dTo2d_short(
						    frameMS.node_data,
						    frameMS.node_data_size,
						    frameMS.header.sense_node),
						frameMS.header.force_node,
						frameMS.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		/*read self raw*/
		case CMD_GETSSFRAME:
			if (numberParam == 2) {
				/* pr_info("Get 1 SS Frame\n"); */
				/* flushFIFO(); //delete the events related to
				 * some touch (allow to call this function while
				 * touching the screen without having a flooding
				 * of the FIFO) */
				/* setScanMode(SCAN_MODE_ACTIVE, 0xFF);
				 * mdelay(WAIT_FOR_FRESH_FRAMES);
				 * setScanMode(SCAN_MODE_ACTIVE, 0x00);
				 * mdelay(WAIT_AFTER_SENSEOFF);
				 */
				res = getSSFrame3((SSFrameType)cmd[1],
						  &frameSS);

				if (res < OK)
					pr_err("Error while taking the SS frame... ERROR %08X\n",
						res);

				else {
					pr_info("The frame size is %d words\n",
						res);
					size += (res * sizeof(short) + 2);
					/* +2 to add force and sense channels
					 * set res to OK because if getMSFrame
					 * is successful
					 *	res = number of words read
					 */
					res = OK;
					print_frame_short("SS force frame =",
						  array1dTo2d_short(
						    frameSS.force_data,
						    frameSS.header.force_node,
						    1),
						  frameSS.header.force_node, 1);
					print_frame_short("SS sense frame =",
						array1dTo2d_short(
						  frameSS.sense_data,
						  frameSS.header.sense_node,
						  frameSS.header.sense_node),
						  1,
						frameSS.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETSYNCFRAME:
			/* need to pass: frameType (this parameter can be
			 * one of LOAD_SYNC_FRAME_X)
			 */
			if (numberParam == 2) {
				pr_info("Reading Sync Frame...\n");
				res = getSyncFrame(cmd[1], &frameMS, &frameSS);
				if (res < OK)
					pr_err("Error while taking the Sync Frame frame... ERROR %08X\n",
						res);

				else {
					pr_info("The total frames size is %d words\n",
						 res);
					size += (res * sizeof(short) + 4);
					/* +4 to add force and sense channels
					 * for MS and SS.
					 * Set res to OK because if getSyncFrame
					 * is successful res = number of words
					 * read
					 */
					res = OK;

					print_frame_short("MS frame =",
						array1dTo2d_short(
						    frameMS.node_data,
						    frameMS.node_data_size,
						    frameMS.header.sense_node),
						frameMS.header.force_node,
						frameMS.header.sense_node);
					print_frame_short("SS force frame =",
						array1dTo2d_short(
						    frameSS.force_data,
						    frameSS.header.force_node,
						    1),
						frameSS.header.force_node,
						1);
					print_frame_short("SS sense frame =",
						array1dTo2d_short(
						    frameSS.sense_data,
						    frameSS.header.sense_node,
						    frameSS.header.sense_node),
						1,
						frameSS.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_REQCOMPDATA:	/* request comp data */
			if (numberParam == 2) {
				pr_info("Requesting Compensation Data\n");
				res = requestCompensationData(cmd[1]);

				if (res < OK)
					pr_err("Error requesting compensation data ERROR %08X\n",
						res);
				else
					pr_info("Requesting Compensation Data Finished!\n");
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READCOMPDATAHEAD:	/* read comp data header */
			if (numberParam == 2) {
				pr_info("Requesting Compensation Data\n");
				res = requestCompensationData(cmd[1]);
				if (res < OK)
					pr_err("Error requesting compensation data ERROR %08X\n",
						res);
				else {
					pr_info("Requesting Compensation Data Finished!\n");
					res = readCompensationDataHeader(
						(u8)funcToTest[1], &dataHead,
						&address);
					if (res < OK)
						pr_err("Read Compensation Data Header ERROR %08X\n",
							res);
					else {
						pr_info("Read Compensation Data Header OK!\n");
						size += (1 * sizeof(u8));
					}
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_READMSCOMPDATA:/* read mutual comp data */
			if (numberParam == 2) {
				pr_info("Get MS Compensation Data\n");
				res = readMutualSenseCompensationData(cmd[1],
							      &compData);

				if (res < OK)
					pr_err("Error reading MS compensation data ERROR %08X\n",
						res);
				else {
					pr_info("MS Compensation Data Reading Finished!\n");
					size = ((compData.node_data_size + 10) *
						sizeof(i8));
					print_frame_i8("MS Data (Cx2) =",
						array1dTo2d_i8(
						  compData.node_data,
						  compData.node_data_size,
						  compData.header.sense_node),
						compData.header.force_node,
						compData.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READSSCOMPDATA:
			if (numberParam == 2) {	/* read self comp data */
				pr_info("Get SS Compensation Data...\n");
				res = readSelfSenseCompensationData(cmd[1],
								    &comData);
				if (res < OK)
					pr_err("Error reading SS compensation data ERROR %08X\n",
						res);
				else {
					pr_info("SS Compensation Data Reading Finished!\n");
					size = ((comData.header.force_node +
						 comData.header.sense_node) *
						2 + 13) *
					       sizeof(i8);
					print_frame_i8("SS Data Ix2_fm = ",
						array1dTo2d_i8(
						  comData.ix2_fm,
						  comData.header.force_node,
						  comData.header.force_node),
						1,
						comData.header.force_node);
					print_frame_i8("SS Data Cx2_fm = ",
						array1dTo2d_i8(
						  comData.cx2_fm,
						  comData.header.force_node,
						  comData.header.force_node),
						1,
						comData.header.force_node);
					print_frame_i8("SS Data Ix2_sn = ",
						array1dTo2d_i8(
						  comData.ix2_sn,
						  comData.header.sense_node,
						  comData.header.sense_node),
						1,
						comData.header.sense_node);
					print_frame_i8("SS Data Cx2_sn = ",
						array1dTo2d_i8(
						  comData.cx2_sn,
						  comData.header.sense_node,
						  comData.header.sense_node),
						1,
						comData.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READTOTMSCOMPDATA:	/* read mutual comp data */
			if (numberParam == 2) {
				pr_info("Get TOT MS Compensation Data\n");
				res = readTotMutualSenseCompensationData(cmd[1],
								&totCompData);

				if (res < OK)
					pr_err("Error reading TOT MS compensation data ERROR %08X\n",
						res);
				else {
					pr_info("TOT MS Compensation Data Reading Finished!\n");
					size = (totCompData.node_data_size *
						sizeof(short) + 9);
					print_frame_short("MS Data (TOT Cx) =",
					  array1dTo2d_short(
					      totCompData.node_data,
					      totCompData.node_data_size,
					      totCompData.header.sense_node),
					  totCompData.header.force_node,
					  totCompData.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READTOTSSCOMPDATA:
			if (numberParam == 2) {	/* read self comp data */
				pr_info("Get TOT SS Compensation Data...\n");
				res = readTotSelfSenseCompensationData(cmd[1],
								&totComData);
				if (res < OK)
					pr_err("Error reading TOT SS compensation data ERROR %08X\n",
						res);
				else {
					pr_info("TOT SS Compensation Data Reading Finished!\n");
					size = ((totComData.header.force_node +
						 totComData.header.sense_node) *
						2 *
						sizeof(short) + 9);
					print_frame_u16("SS Data TOT Ix_fm = ",
						array1dTo2d_u16(
						  totComData.ix_fm,
						  totComData.header.force_node,
						  totComData.header.force_node),
						1,
						totComData.header.force_node);
					print_frame_short(
						"SS Data TOT Cx_fm = ",
						array1dTo2d_short(
						  totComData.cx_fm,
						  totComData.header.force_node,
						  totComData.header.force_node),
						1,
						totComData.header.force_node);
					print_frame_u16("SS Data TOT Ix_sn = ",
						array1dTo2d_u16(
						  totComData.ix_sn,
						  totComData.header.sense_node,
						  totComData.header.sense_node),
						1,
						totComData.header.sense_node);
					print_frame_short(
						"SS Data TOT Cx_sn = ",
						array1dTo2d_short(
						  totComData.cx_sn,
						  totComData.header.sense_node,
						  totComData.header.sense_node),
						1,
						totComData.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READSENSCOEFF:
			/* read MS and SS Sensitivity Coefficients */
			pr_info("Get Sensitivity Calibration Coefficients...\n");
			res = readSensitivityCoefficientsData(&msCoeff,
							      &ssCoeff);
			if (res < OK)
				pr_err("Error reading Sensitivity Calibration Coefficients ERROR %08X\n",
					res);
			else {
				pr_info("Sensitivity Calibration Coefficients Reading Finished!\n");
				size += (((msCoeff.node_data_size) +
					  ssCoeff.header.force_node +
					  ssCoeff.header.sense_node) *
					 sizeof(u8) + 4);
				print_frame_u8("MS Sensitivity Coeff = ",
					       array1dTo2d_u8(msCoeff.ms_coeff,
							      msCoeff.
							      node_data_size,
							      msCoeff.header.
							      sense_node),
					       msCoeff.header.force_node,
					       msCoeff.header.sense_node);
				print_frame_u8("SS Sensitivity Coeff force = ",
					       array1dTo2d_u8(
						       ssCoeff.ss_force_coeff,
						       ssCoeff.header.
						       force_node, 1),
					       ssCoeff.header.force_node, 1);
				print_frame_u8("SS Sensitivity Coeff sense = ",
					       array1dTo2d_u8(
						       ssCoeff.ss_sense_coeff,
						       ssCoeff.header.
						       sense_node,
						       ssCoeff.header.
						       sense_node), 1,
					       ssCoeff.header.sense_node);
			}
			break;

		case CMD_GETFWVER:
			size += (EXTERNAL_RELEASE_INFO_SIZE)*sizeof(u8);
			break;

		case CMD_FLASHUNLOCK:
			res = flash_unlock();
			if (res < OK)
				pr_err("Impossible Unlock Flash ERROR %08X\n",
					res);
			else
				pr_info("Flash Unlock OK!\n");
			break;

		case CMD_READFWFILE:
			if (numberParam == 2) {	/* read fw file */
				pr_info("Reading FW File...\n");
				res = readFwFile(info->board->fw_name, &fw,
						 funcToTest[1]);
				if (res < OK)
					pr_err("Error reading FW File ERROR %08X\n",
						res);
				else
					pr_info("Read FW File Finished!\n");
				kfree(fw.data);
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FLASHPROCEDURE:
			if (numberParam == 3) {	/* flashing procedure */
				pr_info("Starting Flashing Procedure...\n");
				res = flashProcedure(info->board->fw_name,
						     cmd[1],
						     cmd[2]);
				if (res < OK)
					pr_err("Error during flash procedure ERROR %08X\n",
						res);
				else
					pr_info("Flash Procedure Finished!\n");
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FLASHERASEUNLOCK:
			res = flash_erase_unlock();
			if (res < OK)
				pr_err("Error during flash erase unlock... ERROR %08X\n",
					res);
			else
				pr_info("Flash Erase Unlock Finished!\n");
			break;

		case CMD_FLASHERASEPAGE:
			if (numberParam == 2) {	/* need to pass: keep_cx */
				pr_info("Starting Flashing Page Erase...\n");
				res = flash_erase_page_by_page(cmd[1]);
				if (res < OK)
					pr_err("Error during flash page erase... ERROR %08X\n",
						res);
				else
					pr_info("Flash Page Erase Finished!\n");
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		/*ITO TEST*/
		case CMD_ITOTEST:
			frameMS.node_data = NULL;
			res = production_test_ito(LIMITS_FILE, &tests,
				&frameMS);

			if (frameMS.node_data != NULL) {
				size += (frameMS.node_data_size *
						sizeof(short) + 2);
				report = 1;
			}
			break;

		/*Initialization*/
		case CMD_INITTEST:
			if (numberParam == 2)
				res = production_test_initialization(cmd[1]);

			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_MSRAWTEST:	/* MS Raw DATA TEST */
			if (numberParam == 2)	/* need to specify if stopOnFail
						 * */
				res = production_test_ms_raw(LIMITS_FILE,
							     cmd[1], &tests);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_MSINITDATATEST:/* MS CX DATA TEST */
			if (numberParam == 2)	/* need stopOnFail */
				res = production_test_ms_cx(LIMITS_FILE, cmd[1],
							    &tests);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SSRAWTEST:	/* SS RAW DATA TEST */
			if (numberParam == 2) /* need stopOnFail */
				res = production_test_ss_raw(LIMITS_FILE,
							     cmd[1], &tests);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SSINITDATATEST:/* SS IX CX DATA TEST */
			if (numberParam == 2)	/* need stopOnFail */
				res = production_test_ss_ix_cx(LIMITS_FILE,
							       cmd[1], &tests);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		/*PRODUCTION TEST*/
		case CMD_MAINTEST:
			if (numberParam >= 3)	/* need to specify if stopOnFail
						 * saveInit and
						 * mpflag(optional)
						 */
				if (numberParam == 3)
					res = production_test_main(LIMITS_FILE,
							   cmd[1],
							   cmd[2], &tests,
							   MP_FLAG_OTHERS);
				else
					res = production_test_main(LIMITS_FILE,
							   cmd[1],
							   cmd[2], &tests,
							   cmd[3]);
			else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FREELIMIT:
			res = freeCurrentLimitsFile();
			break;

		case CMD_POWERCYCLE:
			res = fts_chip_powercycle(info);
			break;

		case CMD_GETLIMITSFILE:
			/* need to pass: path(optional) return error code +
			 * number of byte read otherwise GUI could not now how
			 * many byte read */
			if (numberParam >= 1) {
				lim.data = NULL;
				lim.size = 0;
				if (numberParam == 1)
					res = getLimitsFile(LIMITS_FILE, &lim);
				else
					res = getLimitsFile(path, &lim);
				readData = lim.data;
				fileSize = lim.size;
				size += (fileSize * sizeof(u8));
				if (byte_call == 1)
					size += sizeof(u32);	/* transmit as
								 * first 4 bytes
								 * the size */
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETLIMITSFILE_BYTE:
			/* need to pass: byteToRead1 byteToRead0 */
			if (numberParam >= 3) {
				lim.data = NULL;
				lim.size = 0;

				u8ToU16_be(&cmd[1], &byteToRead);
				addr = ((u64)byteToRead) * 4;	/* number of
								 * words */

				res = getLimitsFile(LIMITS_FILE, &lim);

				readData = lim.data;
				fileSize = lim.size;

				if (fileSize > addr) {
					pr_err("Limits dimension expected by Host is less than actual size: expected = %d, real = %d\n",
						byteToRead, fileSize);
					res = ERROR_OP_NOT_ALLOW;
				}

				size += (addr * sizeof(u8));
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETFWFILE:
			/* need to pass: from (optional) otherwise select the
			 * approach chosen at compile time */
			if (numberParam >= 1) {
				if (numberParam == 1)
					res = getFWdata(info->board->fw_name,
							&readData, &fileSize);
				else
					res = getFWdata(path, &readData,
							&fileSize);

				size += (fileSize * sizeof(u8));
				if (byte_call == 1)
					size += sizeof(u32);	/* transmit as
								 * first 4 bytes
								 * the size */
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETFWFILE_BYTE:
			/* need to pass: byteToRead1 byteToRead0 */
			if (numberParam == 3) {
				u8ToU16_be(&cmd[1], &byteToRead);
				addr = ((u64)byteToRead) * 4;	/* number of
								 * words */
				res = getFWdata(info->board->fw_name, &readData,
						&fileSize);
				if (fileSize > addr) {
					pr_err("FW dimension expected by Host is less than actual size: expected = %d, real = %d\n",
						byteToRead, fileSize);
					res = ERROR_OP_NOT_ALLOW;
				}

				size += (addr * sizeof(u8));	/* return always
								 * the amount
								 * requested by
								 * host, if real
								 * size is
								 * smaller, the
								 * data are
								 * padded to
								 * zero */
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		/* finish all the diagnostic command with a goto ERROR in order
		 * to skip the modification on driver_test_buff
		 * remember to set properly the limit and printed variables in
		 * order to make the seq_file logic to work */
		case CMD_DIAGNOSTIC:
			index = 0;
			size = 0;
			fileSize = 256 * 1024 * sizeof(char);
			driver_test_buff = (u8 *)kzalloc(fileSize, GFP_KERNEL);
			readData = (u8 *)kmalloc((ERROR_DUMP_ROW_SIZE *
						  ERROR_DUMP_COL_SIZE) *
						 sizeof(u8), GFP_KERNEL);
			if (driver_test_buff == NULL || readData == NULL) {
				res = ERROR_ALLOC;
				pr_err("Impossible allocate memory for buffers! ERROR %08X!\n",
					res);
				goto END;
			}
			j = scnprintf(&driver_test_buff[index],
				      fileSize - index,
				      "DIAGNOSTIC TEST:\n1) I2C Test: ");
			index += j;

			res = fts_writeReadU8UX(FTS_CMD_HW_REG_R,
						ADDR_SIZE_HW_REG, ADDR_DCHIP_ID,
						(u8 *)&temp, 2,
						DUMMY_HW_REG);
			if (res < OK) {
				pr_err("Error during I2C test: ERROR %08X!\n",
					res);
				j = scnprintf(&driver_test_buff[index],
					      fileSize - index, "ERROR %08X\n",
					      res);
				index += j;
				res = ERROR_OP_NOT_ALLOW;
				goto END_DIAGNOSTIC;
			}

			temp &= 0xFFFF;
			pr_info("Chip ID = %04X!\n", temp);
			j = scnprintf(&driver_test_buff[index],
				      fileSize - index,
				      "DATA = %04X, expected = %02X%02X\n",
				      temp, DCHIP_ID_1,
				      DCHIP_ID_0);
			index += j;
			if (temp != ((DCHIP_ID_1 << 8) | DCHIP_ID_0)) {
				pr_err("Wrong CHIP ID, Diagnostic failed!\n");
				res = ERROR_OP_NOT_ALLOW;
				goto END_DIAGNOSTIC;
			}

			j = scnprintf(&driver_test_buff[index],
				      fileSize - index,
				      "Present Driver Mode: %08X\n",
				      info->mode);
			index += j;

			j = scnprintf(&driver_test_buff[index],
				      fileSize - index,
				      "2) FW running: Sensing On...");
			index += j;
			pr_info("Sensing On!\n");
			readData[0] = FTS_CMD_SCAN_MODE;
			readData[1] = SCAN_MODE_ACTIVE;
			readData[2] = 0x1;
			fts_write_heap(readData, 3);
			res = checkEcho(readData, 3);
			if (res < OK) {
				pr_err("No Echo received.. ERROR %08X !\n",
					res);
				j = scnprintf(&driver_test_buff[index],
					      fileSize - index,
					      "No echo found... ERROR %08X!\n",
					      res);
				index += j;
				goto END_DIAGNOSTIC;
			} else {
				pr_info("Echo FOUND... OK!\n");
				j = scnprintf(&driver_test_buff[index],
					      fileSize - index,
					      "Echo FOUND... OK!\n");
				index += j;
			}

			pr_info("Reading Frames...!\n");
			j = scnprintf(&driver_test_buff[index],
				      fileSize - index,
				      "3) Read Frames:\n");
			index += j;
			for (temp = 0; temp < DIAGNOSTIC_NUM_FRAME; temp++) {
				pr_info("Iteration n. %d...\n", temp + 1);
				j = scnprintf(&driver_test_buff[index],
					      fileSize - index,
					      "Iteration n. %d...\n",
					      temp + 1);
				index += j;
				for (addr = 0; addr < 3; addr++) {
					switch (addr) {
					case 0:
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "MS RAW FRAME =");
						index += j;
						res |= getMSFrame3(MS_RAW,
								   &frameMS);
						break;
					case 2:
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "MS STRENGTH FRAME =");
						index += j;
						res |= getMSFrame3(MS_STRENGTH,
								   &frameMS);
						break;
					case 1:
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "MS BASELINE FRAME =");
						index += j;
						res |= getMSFrame3(MS_BASELINE,
								   &frameMS);
						break;
					}
					if (res < OK) {
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "No data! ERROR %08X\n",
						    res);
						index += j;
					} else {
						for (address = 0; address <
						    frameMS.node_data_size;
						  address++) {
							if (address %
							    frameMS.header.
							      sense_node == 0) {
								j = scnprintf(
							       &driver_test_buff
									[index],
							       fileSize	-
									  index,
							       "\n");
								index += j;
							}
							j = scnprintf(
							    &driver_test_buff
								[index],
							    fileSize - index,
							    "%5d, ",
							    frameMS.
							    node_data[address]);
							index += j;
						}
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index, "\n");
						index += j;
					}
					if (frameMS.node_data != NULL)
						kfree(frameMS.node_data);
				}
				for (addr = 0; addr < 3; addr++) {
					switch (addr) {
					case 0:
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "SS RAW FRAME =\n");
						index += j;
						res |= getSSFrame3(SS_RAW,
								   &frameSS);
						break;
					case 2:
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "SS STRENGTH FRAME =\n");
						index += j;
						res |= getSSFrame3(SS_STRENGTH,
								   &frameSS);
						break;
					case 1:
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "SS BASELINE FRAME =\n");
						index += j;
						res |= getSSFrame3(SS_BASELINE,
								   &frameSS);
						break;
					}
					if (res < OK) {
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "No data! ERROR %08X\n",
						    res);
						index += j;
					} else {
						int num;
						short *data;

						num = frameSS.header.force_node;
						data = frameSS.force_data;
						for (address = 0;
							address < num;
							address++) {
						    j = scnprintf(
						       &driver_test_buff[index],
						       fileSize - index,
						       "%d\n",
						       data[address]);
						    index += j;
						}

						num = frameSS.header.sense_node;
						data = frameSS.sense_data;
						for (address = 0;
							address < num;
							address++) {
						    j = scnprintf(
						       &driver_test_buff[index],
						       fileSize - index,
						       "%d, ",
						       data[address]);
						    index += j;
						}
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index, "\n");
						index += j;
					}
					if (frameSS.force_data != NULL)
						kfree(frameSS.force_data);
					if (frameSS.sense_data != NULL)
						kfree(frameSS.sense_data);
				}
			}


			pr_info("Reading error info...\n");
			j = scnprintf(&driver_test_buff[index],
				      fileSize - index,
				      "4) FW INFO DUMP: ");
			index += j;
			temp = dumpErrorInfo(readData, ERROR_DUMP_ROW_SIZE *
					     ERROR_DUMP_COL_SIZE);
			/* OR to detect if there are failures also in the
			 * previous reading of frames and write the correct
			 * result */
			if (temp < OK) {
				pr_err("Error during dump: ERROR %08X!\n",
					res);
				j = scnprintf(&driver_test_buff[index],
					      fileSize - index, "ERROR %08X\n",
					      temp);
				index += j;
			} else {
				pr_info("DUMP OK!\n");
				for (temp = 0; temp < ERROR_DUMP_ROW_SIZE *
				     ERROR_DUMP_COL_SIZE; temp++) {
					if (temp % ERROR_DUMP_COL_SIZE == 0) {
						j = scnprintf(
						    &driver_test_buff[index],
						    fileSize - index,
						    "\n%2d - ",
						    temp / ERROR_DUMP_COL_SIZE);
						index += j;
					}
					j = scnprintf(&driver_test_buff[index],
						      fileSize - index, "%02X ",
						      readData[temp]);
					index += j;
				}
			}
			res |= temp;

END_DIAGNOSTIC:
			if (res < OK) {
				j = scnprintf(&driver_test_buff[index],
					      fileSize - index,
					      "\nRESULT = FAIL\n");
				index += j;
			} else {
				j = scnprintf(&driver_test_buff[index],
					      fileSize - index,
					      "\nRESULT = FINISHED\n");
				index += j;
			}
			/* the sting is already terminated with the null char by
			 * scnprintf */
			limit = index;
			printed = 0;
			goto ERROR;
			break;

#ifdef I2C_INTERFACE
		case CMD_CHANGE_SAD:
			res = changeSAD(cmd[1]);
			break;
#endif

		case CMD_TRIGGER_FORCECAL:
			cmd[0] = CAL_MS_TOUCH | CAL_SS_TOUCH;
			cmd[1] = 0x00;
			fts_disableInterrupt();
			res = writeSysCmd(SYS_CMD_FORCE_CAL, cmd, 2);
			res |= fts_enableInterrupt();
			if (res < OK)
				pr_err("can not trigger Force Cal! ERROR %08X\n",
					res);
			else
				pr_info("MS and SS force cal triggered!\n");
			break;

		case CMD_BASELINE_ADAPTATION:
			/* need to pass: enable */
			if (numberParam == 2) {
				if (cmd[1] == 0x01)
					pr_info("Enabling Baseline adaptation...\n");
				else {
					pr_info("Disabling Baseline adaptation...\n");
					cmd[1] = 0x00;	/* set to zero to
							 * disable baseline
							 * adaptation */
				}

				res = writeConfig(ADDR_CONFIG_AUTOCAL, &cmd[1],
						  1);
				if (res < OK)
					pr_err("Baseline adaptation operation FAILED! ERROR %08X\n",
						res);
				else
					pr_info("Baseline adaptation operation OK!\n");
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FREQ_HOP:
			/* need to pass: enable */
			if (numberParam == 2) {
				pr_info("Reading MNM register...\n");
				res = readConfig(ADDR_CONFIG_MNM, &cmd[2], 1);
				if (res < OK) {
					pr_err("Reading MNM register... ERROR %08X!\n",
						res);
					break;
				}

				if (cmd[1] == 0x01) {
					pr_info("Enabling Frequency Hopping... %02X => %02X\n",
						cmd[2], cmd[2] | 0x01);
					cmd[2] |= 0x01;	/* set bit 0 to enable
							 * Frequency Hopping */
				} else {
					pr_info("Disabling Frequency Hopping... %02X => %02X\n",
						cmd[2], cmd[2] & (~0x01));
					cmd[2] &= (~0x01);	/* reset bit 0
								 * to disable
								 * Frequency
								 * Hopping */
				}

				res = writeConfig(ADDR_CONFIG_MNM, &cmd[2], 1);
				if (res < OK)
					pr_err("Frequency Hopping operation FAILED! ERROR %08X\n",
						res);
				else
					pr_info("Frequency Hopping operation OK!\n");
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READ_SYNC_FRAME:
			/* need to pass: frameType (this parameter can be
			 * LOAD_SYNC_FRAME_STRENGTH or LOAD_SYNC_FRAME_BASELINE)
			 * */
			if (numberParam == 2) {
				pr_info("Reading Sync Frame...\n");
				res = getSyncFrame(cmd[1], &frameMS, &frameSS);
				if (res < OK)
					pr_err("Error while taking the Sync Frame frame... ERROR %08X\n",
						res);

				else {
					pr_info("The total frames size is %d words\n",
						res);
					size += (res * sizeof(short) + 4);
					/* +4 to add force and sense channels
					 * for MS and SS
					 * set res to OK because if getSyncFrame
					 * is successful
					 *	res = number of words read
					 */
					res = OK;

					print_frame_short("MS frame =",
						array1dTo2d_short(
						    frameMS.node_data,
						    frameMS.node_data_size,
						    frameMS.header.sense_node),
						frameMS.header.force_node,
						frameMS.header.sense_node);
					print_frame_short("SS force frame =",
						array1dTo2d_short(
						    frameSS.force_data,
						    frameSS.header.force_node,
						    1),
						frameSS.header.force_node, 1);
					print_frame_short("SS sense frame =",
						array1dTo2d_short(
						    frameSS.sense_data,
						    frameSS.header.sense_node,
						    frameSS.header.sense_node),
						1,
						frameSS.header.sense_node);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SET_OPERATING_FREQ:
			/* need to pass: freq3 freq2 freq1 freq0 */
			if (numberParam == 5) {
				res = fts_disableInterrupt();
				if (res >= OK) {
					pr_info("Setting Scan Freq...\n");
					u8ToU32_be(&cmd[1], &fileSize);
					/* fileSize is used just as container
					 * variable, sorry for the name! */

					res = setActiveScanFrequency(fileSize);
					if (res < OK)
						pr_err("Error while setting the scan frequency... ERROR %08X\n",
							res);
					else {
						/* setActiveScan Frequency leave
						 * the chip in reset state but
						 * with the new scan freq set */
						/* need to enable the scan mode
						 * and re-enable the interrupts
						 * */
						res |= setScanMode(
							SCAN_MODE_LOCKED,
							LOCKED_ACTIVE);
						/* this is a choice to force
						 * the IC to use the freq set */
						res |= fts_enableInterrupt();
						pr_info("Setting Scan Freq... res = %08X\n",
							res);
					}
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_TP_SENS_MODE:
			/* need to pass: enter (optional)saveGain */
			if (numberParam >= 2) {
				if (numberParam == 2)
					cmd[2] = 0;	/* by default never save
							 * the gain (used only
							 * when exit) */

				res = tp_sensitivity_mode(cmd[1], cmd[2]);
				if (res < OK)
					pr_err("Error while setting TP Sens mode... ERROR %08X\n",
						res);
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_TP_SENS_SET_SCAN_MODE:
			/* need to pass: scan_type, enableGains */
			if (numberParam == 3) {
				if (cmd[1] == 0x07) { /* To match the api for
						       * C2/F2
						       */
					res = tp_sensitivity_set_scan_mode(
					    LOCKED_SINGLE_ENDED_ONLY_MUTUAL_0,
					    cmd[2]);
					/* this force the IC to lock in a scan
					 * mode
					 */
					if (res < OK)
						pr_err("Error while setting TP Sens scan mode... ERROR %08X\n",
						res);
				} else {
					pr_err("Wrong parameter!\n");
					 res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_TP_SENS_PRECAL_SS:
			/* need to pass: target1 target0 percentage(optional) */
			if (numberParam >= 3) {
				if (numberParam > 3)
					temp = cmd[3];
				else
					temp = SENS_TEST_PERC_TARGET_PRECAL;

				pr_info("Setting target = %d and percentage = %d\n",
					(cmd[1] << 8 | cmd[2]), temp);

				res = tp_sensitivity_test_pre_cal_ss(&frameSS,
							     (cmd[1] << 8 |
									cmd[2]),
							      temp);
				if (res < OK)
					pr_err("Error while setting the scan frequency... ERROR %08X\n",
						res);

				if ((frameSS.force_data != NULL) &&
				    (frameSS.sense_data != NULL)) {
					size += ((frameSS.header.force_node +
						  frameSS.header.sense_node) *
						 sizeof(short) + 2);
					/*make error code positive to print the
					 * frame*/
					res &= (~0x80000000);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_TP_SENS_PRECAL_MS:
			/* need to pass: target1 target0 calibrate
			 * percentage(optional) */
			if (numberParam >= 4) {
				if (numberParam > 4)
					temp = cmd[4];
				else
					temp = SENS_TEST_PERC_TARGET_PRECAL;

				pr_info("Setting target = %d and percentage = %d\n",
					(cmd[1] << 8 | cmd[2]), temp);

				res = tp_sensitivity_test_pre_cal_ms(&frameMS,
							     (cmd[1] << 8 |
									cmd[2]),
							      temp);
				if (res < OK)
					pr_err("Error during TP Sensitivity Precal ... ERROR %08X\n",
						res);

				if (cmd[3] != 0) {
					pr_info("Computing gains with target = %d and saveGain = %d\n",
						(cmd[1] << 8 | cmd[2]), 0);
					temp = tp_sensitivity_compute_gains(
						&frameMS, (cmd[1] << 8 |
							   cmd[2]),
						0);
					if (temp < OK)
						pr_err("Error during TP Sensitivity Calibration... ERROR %08X\n",
							temp);
					res |= temp;
				}

				if (frameMS.node_data != NULL) {
					size += (frameMS.node_data_size *
						 sizeof(short) + 2);
					/*make error code positive to print the
					 * frame*/
					res &= (~0x80000000);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_TP_SENS_POSTCAL_MS:
			/* need to pass: target1 target0 executeTest
			 * percentage(optional) */
			if (numberParam >= 4) {
				if (cmd[3] != 0) {
					if (numberParam > 4)
						temp = cmd[4];
					else
						temp =
						SENS_TEST_PERC_TARGET_POSTCAL;
				} else
					temp = -1;

				pr_info("Setting target = %d and percentage = %d\n",
					(cmd[1] << 8 | cmd[2]), temp);

				res = tp_sensitivity_test_post_cal_ms(&frameMS,
							      &deltas,
							      (cmd[1] << 8 |
									cmd[2]),
							      temp,
							      &meanNorm,
							      &meanEdge);
				if (res < OK)
					pr_err("Error during TP Sensitivity Post Cal ... ERROR %08X\n",
						res);

				/* processing for a proper printing on the shell
				 * */
				if ((frameMS.node_data != NULL) &&
				    (deltas.node_data != NULL)) {
					size += ((frameMS.node_data_size +
						  deltas.node_data_size) *
						 sizeof(short) +
						 2 + 8);/* +2 force and
							 * sense len, +8
							 * mean_normal/edge
							 * */
					/*make error code positive to print the
					 * frame*/
					res &= (~0x80000000);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_TP_SENS_STD:
			/* need to pass: numFrames */
			if (numberParam >= 2) {
				res =  tp_sensitivity_test_std_ms(cmd[1],
								  &frameMS);
				if (res < OK)
					pr_err("Error during TP Sensitivity STD... ERROR %08X\n",
						res);

				/* processing for a proper printing on the shell
				 * */
				if (frameMS.node_data != NULL) {
					size += ((frameMS.node_data_size) *
						 sizeof(short) + 2);
					/* +2 force and sense len */
					/*make error code positive to print the
					 * frame*/
					res &= (~0x80000000);
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FORCE_TOUCH_ACTIVE:
			/* Single parameter indicates force touch state */
			if (numberParam == 2) {
				if (cmd[1] > 1) {
					pr_err("Parameter should be 1 or 0\n");
					res = ERROR_OP_NOT_ALLOW;
				} else {
					fts_set_bus_ref(info,
						FTS_BUS_REF_FORCE_ACTIVE,
						cmd[1]);
					res = OK;
				}
			} else {
				pr_err("Wrong number of parameters!\n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		default:
			pr_err("COMMAND ID NOT VALID!!!\n");
			res = ERROR_OP_NOT_ALLOW;
			break;
		}

		/* res2 = fts_enableInterrupt();
		 * the interrupt was disabled on purpose in this node because it
		 * can be used for testing procedure and between one step and
		 * another the interrupt wan to be kept disabled
		 * if (res2 < 0) {
		 *      pr_err("stm_driver_test_show: ERROR %08X\n",
		 * (res2 | ERROR_ENABLE_INTER));
		 * }*/
	} else {
		pr_err("NO COMMAND SPECIFIED!!! do: 'echo [cmd_code] [args] > stm_fts_cmd' before looking for result!\n");
		res = ERROR_OP_NOT_ALLOW;
	}

END:	/* here start the reporting phase, assembling the data to send in the
	 * file node */
	if (driver_test_buff != NULL) {
		pr_info("Consecutive echo on the file node, free the buffer with the previous result\n");
		kfree(driver_test_buff);
	}

	if (byte_call == 0) {
		size *= 2;
		size += 2;	/* add \n and \0 (terminator char) */
	} else {
		if (bin_output != 1) {
			size *= 2; /* need to code each byte as HEX string */
			size -= 1;	/* start byte is just one, the extra
					 * byte of end byte taken by \n */
		} else
			size += 1;	/* add \n */
	}

	pr_info("Size = %d\n", size);
	driver_test_buff = (u8 *)kzalloc(size, GFP_KERNEL);
	pr_info("Finish to allocate memory!\n");
	if (driver_test_buff == NULL) {
		pr_err("Unable to allocate driver_test_buff! ERROR %08X\n",
			ERROR_ALLOC);
		goto ERROR;
	}

	if (byte_call == 0) {
		index = 0;
		index += scnprintf(&driver_test_buff[index],
				   size - index, "{ ");
		index += scnprintf(&driver_test_buff[index],
				   size - index, "%08X", res);
		if (res >= OK || report) {
			/*all the other cases are already fine printing only the
			 * res.*/
			switch (funcToTest[0]) {
			case CMD_VERSION:
			case CMD_READ:
			case CMD_WRITEREAD:
			case CMD_WRITETHENWRITEREAD:
			case CMD_WRITEREADU8UX:
			case CMD_WRITEU8UXTHENWRITEREADU8UX:
			case CMD_READCONFIG:
			case CMD_POLLFOREVENT:
				/* pr_err("Data = "); */
				if (mess.dummy == 1)
					j = 1;
				else
					j = 0;
				for (; j < byteToRead + mess.dummy; j++) {
					/* pr_err("%02X ", readData[j]); */
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X", readData[j]);
					/* this approach is much more faster */
				}
				/* pr_err("\n"); */
				break;
			case CMD_GETFWFILE:
			case CMD_GETLIMITSFILE:
				pr_info("Start To parse!\n");
				for (j = 0; j < fileSize; j++) {
					/* pr_err("%02X ", readData[j]); */
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X", readData[j]);
				}
				pr_info("Finish to parse!\n");
				break;
			case CMD_GETFORCELEN:
			case CMD_GETSENSELEN:
				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   (u8)temp);
				break;
			case CMD_GETMSFRAME:
			case CMD_TP_SENS_PRECAL_MS:
			case CMD_TP_SENS_POSTCAL_MS:
			case CMD_TP_SENS_STD:
			case CMD_ITOTEST:

				if (frameMS.node_data == NULL)
					break;

				if (res != OK)
					driver_test_buff[2] = '8';
				/* convert back error code to negative */

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameMS.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameMS.header.sense_node);

				for (j = 0; j < frameMS.node_data_size; j++) {
					index += scnprintf(
					   &driver_test_buff[index],
					   size - index,
					   "%02X%02X",
					   (frameMS.node_data[j] & 0xFF00) >> 8,
					   frameMS.node_data[j] & 0xFF);
				}

				kfree(frameMS.node_data);

				if (funcToTest[0] == CMD_TP_SENS_POSTCAL_MS) {
					/* print also mean and deltas */
					for (j = 0; j < deltas.node_data_size;
					     j++) {
						index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (deltas.node_data[j] &
							 0xFF00) >> 8,
						    deltas.node_data[j] &
							0xFF);
					}
					kfree(deltas.node_data);

					index += scnprintf(
						     &driver_test_buff[index],
						     size - index,
						     "%08X", meanNorm);

					index += scnprintf(
						     &driver_test_buff[index],
						     size - index,
						     "%08X", meanEdge);
				}
				break;
			case CMD_GETSSFRAME:
			case CMD_TP_SENS_PRECAL_SS:
				if (res != OK)
					driver_test_buff[2] = '8';
				/* convert back error code to negative */
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameSS.header.force_node);
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameSS.header.sense_node);
				/* Copying self raw data Force */
				for (j = 0; j < frameSS.header.force_node;
				     j++) {
					index += scnprintf(
					  &driver_test_buff[index],
					  size - index,
					  "%02X%02X",
					  (frameSS.force_data[j] & 0xFF00) >> 8,
					  frameSS.force_data[j] & 0xFF);
				}

				/* Copying self raw data Sense */
				for (j = 0; j < frameSS.header.sense_node;
				     j++) {
					index += scnprintf(
					  &driver_test_buff[index],
					  size - index,
					  "%02X%02X",
					  (frameSS.sense_data[j] & 0xFF00) >> 8,
					  frameSS.sense_data[j] & 0xFF);
				}

				kfree(frameSS.force_data);
				kfree(frameSS.sense_data);
				break;

			case CMD_GETSYNCFRAME:
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameMS.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameMS.header.sense_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameSS.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameSS.header.sense_node);

				/* Copying mutual data */
				for (j = 0; j < frameMS.node_data_size; j++) {
					index += scnprintf(
						&driver_test_buff[index],
						size - index,
						"%02X%02X",
						(frameMS.node_data[j] &
							0xFF00) >> 8,
						frameMS.node_data[j] & 0xFF);
				}

				/* Copying self data Force */
				for (j = 0; j < frameSS.header.force_node;
				     j++) {
					index += scnprintf(
						&driver_test_buff[index],
						size - index,
						"%02X%02X",
						(frameSS.force_data[j] &
							0xFF00) >> 8,
						frameSS.force_data[j] & 0xFF);
				}

				/* Copying self  data Sense */
				for (j = 0; j < frameSS.header.sense_node;
				     j++) {
					index += scnprintf(
						&driver_test_buff[index],
						size - index,
						"%02X%02X",
						(frameSS.sense_data[j] &
							0xFF00) >> 8,
						frameSS.sense_data[j] & 0xFF);
				}

				kfree(frameMS.node_data);
				kfree(frameSS.force_data);
				kfree(frameSS.sense_data);
				break;

			case CMD_READMSCOMPDATA:
				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   (u8)compData.header.type);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)compData.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)compData.header.sense_node);

				/* Cpying CX1 value */
				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   compData.cx1 & 0xFF);

				/* Copying CX2 values */
				for (j = 0; j < compData.node_data_size; j++) {
					index += scnprintf(
						&driver_test_buff[index],
						size - index,
						"%02X",
						compData.node_data[j] & 0xFF);
				}

				kfree(compData.node_data);
				break;

			case CMD_READSSCOMPDATA:
				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   (u8)comData.header.type);

				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   comData.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   comData.header.sense_node);

				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   comData.f_ix1 & 0xFF);

				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   comData.s_ix1 & 0xFF);

				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   comData.f_cx1 & 0xFF);

				index += scnprintf(&driver_test_buff[index],
						   size - index, "%02X",
						   comData.s_cx1 & 0xFF);

				/* Copying IX2 Force */
				for (j = 0; j < comData.header.force_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X",
						    comData.ix2_fm[j] & 0xFF);
				}

				/* Copying IX2 Sense */
				for (j = 0; j < comData.header.sense_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X",
						    comData.ix2_sn[j] & 0xFF);
				}

				/* Copying CX2 Force */
				for (j = 0; j < comData.header.force_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X",
						    comData.cx2_fm[j] & 0xFF);
				}

				/* Copying CX2 Sense */
				for (j = 0; j < comData.header.sense_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X",
						    comData.cx2_sn[j] & 0xFF);
				}

				kfree(comData.ix2_fm);
				kfree(comData.ix2_sn);
				kfree(comData.cx2_fm);
				kfree(comData.cx2_sn);
				break;



			case CMD_READTOTMSCOMPDATA:
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)totCompData.header.type);

				index += scnprintf(&driver_test_buff[index],
					    size - index, "%02X",
					    (u8)totCompData.header.force_node);

				index += scnprintf(&driver_test_buff[index],
					    size - index, "%02X",
					    (u8)totCompData.header.sense_node);

				/* Copying TOT CX values */
				for (j = 0; j < totCompData.node_data_size;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (totCompData.node_data[j] &
							0xFF00) >> 8,
						    totCompData.node_data[j] &
							0xFF);
				}

				kfree(totCompData.node_data);
				break;

			case CMD_READTOTSSCOMPDATA:
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)totComData.header.type);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						totComData.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						totComData.header.sense_node);

				/* Copying TOT IX Force */
				for (j = 0; j < totComData.header.force_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (totComData.ix_fm[j] &
							0xFF00) >> 8,
						    totComData.ix_fm[j] &
							0xFF);
				}

				/* Copying TOT IX Sense */
				for (j = 0; j < totComData.header.sense_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (totComData.ix_sn[j] &
							0xFF00) >> 8,
						    totComData.ix_sn[j] &
							0xFF);
				}

				/* Copying TOT CX Force */
				for (j = 0; j < totComData.header.force_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (totComData.cx_fm[j] &
							0xFF00) >> 8,
						    totComData.cx_fm[j] &
							0xFF);
				}

				/* Copying CX2 Sense */
				for (j = 0; j < totComData.header.sense_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (totComData.cx_sn[j] &
							0xFF00) >> 8,
						    totComData.cx_sn[j] &
							0xFF);
				}

				kfree(totComData.ix_fm);
				kfree(totComData.ix_sn);
				kfree(totComData.cx_fm);
				kfree(totComData.cx_sn);
				break;

			case CMD_READSENSCOEFF:
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)msCoeff.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)msCoeff.header.sense_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)ssCoeff.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)ssCoeff.header.sense_node);

				/* Copying MS Coefficients */
				for (j = 0; j < msCoeff.node_data_size; j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X",
						    msCoeff.ms_coeff[j] & 0xFF);
				}

				/* Copying SS force Coefficients */
				for (j = 0; j < ssCoeff.header.force_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X",
						    ssCoeff.ss_force_coeff[j] &
							0xFF);
				}

				/* Copying SS sense Coefficients */
				for (j = 0; j < ssCoeff.header.sense_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X",
						    ssCoeff.ss_sense_coeff[j] &
							0xFF);
				}

				kfree(msCoeff.ms_coeff);
				kfree(ssCoeff.ss_force_coeff);
				kfree(ssCoeff.ss_sense_coeff);
				break;

			case CMD_GETFWVER:
				for (j = 0; j < EXTERNAL_RELEASE_INFO_SIZE;
				     j++) {
					index += scnprintf(
						  &driver_test_buff[index],
						  size - index,
						  "%02X",
						  systemInfo.u8_releaseInfo[j]);
				}
				break;

			case CMD_READCOMPDATAHEAD:
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						dataHead.type);
				break;

			case CMD_READ_SYNC_FRAME:
				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameMS.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameMS.header.sense_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameSS.header.force_node);

				index += scnprintf(&driver_test_buff[index],
						size - index, "%02X",
						(u8)frameSS.header.sense_node);

				/* Copying mutual data */
				for (j = 0; j < frameMS.node_data_size; j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (frameMS.node_data[j] &
							0xFF00) >> 8,
						    frameMS.node_data[j] &
							0xFF);
				}

				/* Copying self data Force */
				for (j = 0; j < frameSS.header.force_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (frameSS.force_data[j] &
							0xFF00) >> 8,
						    frameSS.force_data[j] &
							0xFF);
				}


				/* Copying self  data Sense */
				for (j = 0; j < frameSS.header.sense_node;
				     j++) {
					index += scnprintf(
						    &driver_test_buff[index],
						    size - index,
						    "%02X%02X",
						    (frameSS.sense_data[j] &
							0xFF00) >> 8,
						    frameSS.sense_data[j] &
							0xFF);
				}

				kfree(frameMS.node_data);
				kfree(frameSS.force_data);
				kfree(frameSS.sense_data);
				break;


			default:
				break;
			}
		}

		index += scnprintf(&driver_test_buff[index],
				   size - index, " }\n");
		limit = size - 1;/* avoid to print \0 in the shell */
		printed = 0;
	} else {
		/* start byte */
		driver_test_buff[index++] = MESSAGE_START_BYTE;
		if (bin_output == 1) {
			/* msg_size */
			driver_test_buff[index++] = (size & 0xFF00) >> 8;
			driver_test_buff[index++] = (size & 0x00FF);
			/* counter id */
			driver_test_buff[index++] =
				(mess.counter & 0xFF00) >> 8;
			driver_test_buff[index++] = (mess.counter & 0x00FF);
			/* action */
			driver_test_buff[index++] = (mess.action & 0xFF00) >> 8;
			driver_test_buff[index++] = (mess.action & 0x00FF);
			/* error */
			driver_test_buff[index++] = (res & 0xFF00) >> 8;
			driver_test_buff[index++] = (res & 0x00FF);
		} else {
			if (funcToTest[0] == CMD_GETLIMITSFILE_BYTE ||
			    funcToTest[0] == CMD_GETFWFILE_BYTE)
				index += scnprintf(&driver_test_buff[index],
					   size - index,
					   "%02X%02X",
					   (((fileSize + 3) / 4) & 0xFF00) >> 8,
					   ((fileSize + 3) / 4) & 0x00FF);
			else
				index += scnprintf(&driver_test_buff[index],
					    size - index,
					    "%02X%02X", (size & 0xFF00) >> 8,
					    size & 0xFF);

			index += scnprintf(&driver_test_buff[index],
					   size - index, "%04X",
					   (u16)mess.counter);
			index += scnprintf(&driver_test_buff[index],
					   size - index, "%04X",
					   (u16)mess.action);
			index += scnprintf(&driver_test_buff[index],
					   size - index,
					   "%02X%02X", (res & 0xFF00) >> 8,
					   res & 0xFF);
		}

		switch (funcToTest[0]) {
		case CMD_VERSION_BYTE:
		case CMD_READ_BYTE:
		case CMD_WRITEREAD_BYTE:
		case CMD_WRITETHENWRITEREAD_BYTE:
		case CMD_WRITEREADU8UX_BYTE:
		case CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE:
			if (bin_output == 1) {
				if (mess.dummy == 1)
					memcpy(&driver_test_buff[index],
					       &readData[1], byteToRead);
				else
					memcpy(&driver_test_buff[index],
					       readData, byteToRead);
				index += byteToRead;
			} else {
				j = mess.dummy;
				for (; j < byteToRead + mess.dummy; j++)
					index += scnprintf(
						&driver_test_buff[index],
						size - index,
						"%02X",
						(u8)readData[j]);
			}
			break;

		case CMD_GETLIMITSFILE_BYTE:
		case CMD_GETFWFILE_BYTE:
			if (bin_output == 1) {
				/* override the msg_size with dimension in words
				 * */
				driver_test_buff[1] = (
					((fileSize + 3) / 4) & 0xFF00) >> 8;
				driver_test_buff[2] = (
					((fileSize + 3) / 4) & 0x00FF);

				if (readData != NULL)
					memcpy(&driver_test_buff[index],
					       readData, fileSize);
				else
					pr_err("readData = NULL... returning junk data!");
				index += addr;	/* in this case the byte to read
						 * are stored in addr because it
						 * is a u64 end byte need to be
						 * inserted at the end of the
						 * padded memory */
			} else {
				/* snprintf(&driver_test_buff[1], 3, "%02X",
				 * (((fileSize + 3) / 4)&0xFF00) >> 8); */
				/* snprintf(&driver_test_buff[3], 3, "%02X",
				 * ((fileSize + 3) / 4)&0x00FF); */
				for (j = 0; j < fileSize; j++)
					index += scnprintf(
						&driver_test_buff[index],
						size - index,
						"%02X",
						(u8)readData[j]);
				for (; j < addr; j++)
					index += scnprintf(
						&driver_test_buff[index],
						size - index,
						"%02X", 0);	/* pad memory
								 * with 0x00 */
			}
			break;
		default:
			break;
		}

		index += scnprintf(&driver_test_buff[index],
				  size - index, "%c\n", MESSAGE_END_BYTE);
		/*for(j=0; j<size; j++){
		  *      pr_err("%c", driver_test_buff[j]);
		  * }*/
		limit = size;
		printed = 0;
	}
ERROR:
	numberParam = 0;/* need to reset the number of parameters in order to
			 * wait the next command, comment if you want to repeat
			 * the last command sent just doing a cat */

	/* pr_err(0,"numberParameters = %d\n", numberParam); */

	kfree(readData);
	kfree(cmd);

	fts_set_bus_ref(info, FTS_BUS_REF_SYSFS, false);

	return count;
}

/** @}*/

/**
  * file_operations struct which define the functions for the canonical
  * operation on a device file node (open. read, write etc.)
  */
static struct file_operations fts_driver_test_ops = {
	.open		= fts_open,
	.read		= seq_read,
	.write		= fts_driver_test_write,
	.llseek		= seq_lseek,
	.release	= seq_release
};

/*****************************************************************************/

/**
  * This function is called in the probe to initialize and create the directory
  * proc/fts and the driver test file node DRIVER_TEST_FILE_NODE into the /proc
  * file system
  * @return OK if success or an error code which specify the type of error
  */
int fts_proc_init(void)
{
	struct proc_dir_entry *entry;

	int retval = 0;


	fts_dir = proc_mkdir_data("fts", 0777, NULL, NULL);
	if (fts_dir == NULL) {	/* directory creation failed */
		retval = -ENOMEM;
		goto out;
	}

	entry = proc_create(DRIVER_TEST_FILE_NODE, 0777, fts_dir,
			    &fts_driver_test_ops);

	if (entry)
		pr_info("%s: proc entry CREATED!\n", __func__);
	else {
		pr_err("%s: error creating proc entry!\n", __func__);
		retval = -ENOMEM;
		goto badfile;
	}
	return OK;
badfile:
	remove_proc_entry("fts", NULL);
out:
	return retval;
}

/**
  * Delete and Clean from the file system, all the references to the driver test
  * file node
  * @return OK
  */
int fts_proc_remove(void)
{
	remove_proc_entry(DRIVER_TEST_FILE_NODE, fts_dir);
	remove_proc_entry("fts", NULL);
	return OK;
}

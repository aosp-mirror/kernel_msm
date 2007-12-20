#ifndef QDSP5AUDPLAYCMDI_H
#define QDSP5AUDPLAYCMDI_H

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

       Q D S P 5  A U D I O   P L A Y  T A S K   C O M M A N D S

GENERAL DESCRIPTION
  Command Interface for AUDPLAYTASK on QDSP5

REFERENCES
  None

EXTERNALIZED FUNCTIONS

  audplay_cmd_dec_data_avail
    Send buffer to AUDPLAY task
	
  
Copyright(c) 1992 - 2009 by QUALCOMM, Incorporated.

This software is licensed under the terms of the GNU General Public
License version 2, as published by the Free Software Foundation, and
may be copied, distributed, and modified under those terms.
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*===========================================================================

                      EDIT HISTORY FOR FILE

This section contains comments describing changes made to this file.
Notice that changes are listed in reverse chronological order.

$Header: //source/qcom/qct/multimedia2/Audio/drivers/QDSP5Driver/QDSP5Interface/main/latest/qdsp5audplaycmdi.h#2 $
  
===========================================================================*/

#define AUDPLAY_CMD_BITSTREAM_DATA_AVAIL		0x0000
#define AUDPLAY_CMD_BITSTREAM_DATA_AVAIL_LEN	\
	sizeof(audplay_cmd_bitstream_data_avail)

/* Type specification of dec_data_avail message sent to AUDPLAYTASK
*/
typedef struct {
  /*command ID*/
  unsigned int cmd_id;        

  /* Decoder ID for which message is being sent */
  unsigned int decoder_id;        

  /* Start address of data in ARM global memory */
  unsigned int buf_ptr;    

  /* Number of 16-bit words of bit-stream data contiguously available at the
   * above-mentioned address 
   */
  unsigned int buf_size;
          
  /* Partition number used by audPlayTask to communicate with DSP's RTOS
   * kernel 
  */
  unsigned int partition_number;    

} __attribute__((packed)) audplay_cmd_bitstream_data_avail;

#endif /* QDSP5AUDPLAYCMD_H */

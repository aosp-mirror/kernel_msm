/*
 * Copyright 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php, or by writing to the Free
 * Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Sensor RPC handler for 47720
 *
 * tabstop = 8
 */


#ifdef __KERNEL__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/printk.h>
#include "bbd.h"

/* function to inject sensor data into SHMD */
ssize_t bbd_sensor_write(const char *buf, size_t size);
/* function to notify standby */
ssize_t bbd_request_mcu(bool on);
#else

/* self-test with built-in main() */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


#define pr_info		printf
#define pr_warn		printf
#define pr_err		printf
#define WARN_ON(x)	if (x) printf("error in %s:%d\n", __func__, __LINE__)

#endif



/**  Escape byte. If present, the following byte is un-escaped (i.e. interpreted) */
#define ESC_BYTE			0xE0
/** Special Byte that will be interpreted if preceeded by an ESC_BYTE */
#define SOP_BYTE			0x00
#define EOP_BYTE			0x01
#define ESCAPED_ESC_BYTE	0x02
#define ESCAPED_XON_BYTE	0x03
#define ESCAPED_XOFF_BYTE   0x04
#define XON_BYTE			0x11
#define XOFF_BYTE		   0x13

/** BitMask definition for the "Flags" byte */
#define TRANSPORT_FLAG_RELIABLE			(1<<0)   /* only @hs flag used. Indicates that this is a reliable message (i.e. the Flag details will contain the reliable SeqId */
#define TRANSPORT_FLAG_ACK				 (1<<1)   /* Acknowledgement. The Flag details will contain the acknowledged SeqId */
#define TRANSPORT_FLAG_RELIABLE_SEQ_ERROR  (1<<2)   /* Wrong Reliable SeqId detected. The Flag details will contain the last reliable SeqId received */
#define TRANSPORT_FLAG_MSG_LOST			(1<<3)   /* ESW lost messages. The Flag details will contains the number of lost messages (based on SeqId) */
#define TRANSPORT_FLAG_MSG_GARBAGE		 (1<<4)   /* ESW detected garbage. The Flag details will contains the number of lost bytes */
#define TRANSPORT_FLAG_LAST				TRANSPORT_FLAG_MSG_GARBAGE


#define ESW_RPC_HDR_SZ   3 /*ESW transaction header size which equals the size of class id, method len and method id that goes with each RPC */

/* states for the Escaping */
typedef enum
{
	 WAIT_FOR_ESC_SOP = 0
	,WAIT_FOR_SOP
	,WAIT_FOR_MESSAGE_COMPLETE
	,WAIT_FOR_EOP
}esc_state_type;



/*___________________shared_rpc.h_______________________________________________________*/

/** Macro to cosntruct ESW class id */
#define ESWRPC_CLASS(klass)  ESWRPC_##klass

/** Macro to cosntruct GLL class id */
#define GLLRPC_CLASS(klass)  GLLRPC_##klass

/** Macro to cosntruct method id */
#define RPC_METHOD(klass,method)  RPC_##klass##_##method

/** List of the classes and their IDs */
#define ESW_RPC_BASE_ABS	0
#define GLL_RPC_BASE_ABS	16
typedef enum
{
	 ESWRPC_CLASS(gcm) = ESW_RPC_BASE_ABS   /**< Global Configuration Manager */
	,ESWRPC_CLASS(stm)					  /**< built-in Self Test Manager Id */
	,ESWRPC_CLASS(hrm)					  /**< Hardware Registers Manager class Id */
	,ESWRPC_CLASS(tim)					  /**< Time Manager Id */
	,ESWRPC_CLASS(jd)					   /**< Job Director class Id  */
	,ESWRPC_CLASS(rm)					   /**< Resource Manager class Id  */
	,ESWRPC_CLASS(sat)					  /**< Search And Track class Id  */
	,ESWRPC_CLASS(geomgr)				   /**< Geofence Mgr class Id */
	,ESWRPC_CLASS(esamgr)				   /**< ESA Mgr class Id */
	,ESWRPC_CLASS(reserved2)				/**< Reserved for future extension */
	,ESW_RPC_TOP_ABS


	,GLLRPC_CLASS(misc) = GLL_RPC_BASE_ABS  /**< Misc for now, until we have better place to put it!*/
	,GLLRPC_CLASS(satrpt)				   /**< SAT Msmt Report (Search, Sniff, Track) */
	,GLLRPC_CLASS(satevt)				   /**< SAT Event Report (Resource allocation, Sm Report, ..) */
	,GLLRPC_CLASS(utils)					/**< Utils, contains utilities like assert, printf, ...*/
	,GLLRPC_CLASS(sysevt)				   /**< System events from ESW to GLL, like hb, syncin, pps, rtc capture,...*/
	,GLLRPC_CLASS(rmrpt)					/**< Resource Manager Reports from ESW to GLL */
	,GLLRPC_CLASS(georpt)				   /**< Geofence Rpt Mgr from ESW to GLL */
	,GLLRPC_CLASS(esarpt)				   /**< ESA Rpt Mgr from ESW to GLL */
	,GLLRPC_CLASS(reserved2)				/**< Reserved for future extension */
	,GLL_RPC_TOP_ABS
}ClassList;

/* WARNING: we couldn't add the georpt classId as it breaks the FW for B2 due to gllrpc_priorities_type structure which uses GLL_RPC_TOP
 * what we do instead is "share" the classId with misc.
 * we have to be carefull as to have the methodId different, so we start the methodId for georpt in the upper part
 * Also, that means there will be a Hack on the Host side in the TL layer to check whether it is a georpt or misc RPC that is being received
 * As a result, all the upper layer will not see any issue, and it will be transparent
 */
#define GLLRPC_georpt GLLRPC_misc	  /*< Geofence Rpt Mgr from ESW to GLL */

/* Same trick with sensor hub RPC than with geofence */
#define GLLRPC_esarpt GLLRPC_misc		   /*< Sensor Rpt Mgr from ESW to GLL */

typedef enum
{
	RPC_METHOD(esamgr,host2sh)  = 0,

}esamgr_rpc_methods;

typedef enum
{
	 RPC_METHOD(misc, echo_response)   = 0
	,RPC_METHOD(misc, hrm_response)
	,RPC_METHOD(misc, version_number)
	,RPC_METHOD(misc, nvram_response)
	,RPC_METHOD(misc, gpio_response)
	,RPC_METHOD(misc, mem_dump_response)
	,LAST_OF_MISC_RPC_METHOD_ID /* KEEP LAST! Used for hack due to lack of ClassId flexibility */
}misc_rpc_methods;

typedef enum
{
	RPC_METHOD(georpt,fixes) = LAST_OF_MISC_RPC_METHOD_ID, /* We are sharing the classId with misc as we cannot have a separate ClassId, so we start the methodId right after the methodId from misc */
	RPC_METHOD(georpt,internal_state),
	RPC_METHOD(georpt,measurements),
	RPC_METHOD(georpt,area_triggered),
	RPC_METHOD(georpt,debug_msg),
	RPC_METHOD(georpt,host_request),
	RPC_METHOD(georpt,raw_rpcs),
	LAST_OF_GEORPT_RPC_METHOD_ID /* KEEP LAST! Used for hack due to lack of ClassId flexibility */
}georpt_rpc_methods;

typedef enum
{
	ESA2HOST_CMD_SHOFF = 0,
	ESA2HOST_DEBUG
} esa2host_msg_type;

typedef enum
{
	RPC_METHOD(esarpt,sh2host) = LAST_OF_GEORPT_RPC_METHOD_ID,
	RPC_METHOD(esarpt,esa2host)
}esarpt_rpc_methods;

typedef struct
{
	unsigned int ulRxBytes;
	unsigned int ulTxBytes;
	unsigned int ulGarbageBytes;
	unsigned int ulRxMessage;
	unsigned int ulTxMessage;
	unsigned int ulRxSeqErrors;
	unsigned int ulTxSeqErrors;
	unsigned int aulLatencyHistogram[9]; /* 0=[0-250[,1=[250-500[...,7=[1750-2000[,8=2000+ */
}sPacketStats;

static struct
{
	sPacketStats otPublic;
	unsigned short usMaxTxBufUse;
	unsigned short usMaxReliableTxBufUse;
	unsigned short usMaxRxBufUse;
	unsigned short usMaxReliableRxElements;
	struct sLatency
	{
		unsigned int ulTimestamp;
		unsigned char ucSeqId;
	}otLatency[10];
} m_otStats;


/*________________________sh_ifc.h_________________________________________________________ */

/** List of the Sensor Hub Manager RPC methods */

#define SH_MODE_NONE						(0   ) /* Disabling sensor hub */
#define SH_MODE_ACC						 (1<<0) /* Accelerometer reading enabled */
#define SH_MODE_GYRO						(1<<1) /* Gyro reading enabled */

#define MIN_MESSAGE_LENGTH  6		 /* minimum message length (before escaping) */
#define MAX_S2H_MESSAGE_LENGTH  4096  /* maximum sardine to host packet size (before esacping) */
#define MAX_H2S_MESSAGE_LENGTH  800   /* maximum host to sardine packet size (before escaping) */
#define MAX_H2C_MESSAGE_LENGTH  1200  /* maximum host to carp packet size (before escaping) */

typedef struct
{
	unsigned char sh_flags;
	short acc_sampling_rate_hz;
	short gyro_sampling_rate_hz;
} sh_mode_type;

typedef struct
{
	short x;
	short y;
	short z;
} sh_xyz_type;

/*______________________GlMeSrdPacketManager________________________________________________*/

/* GlMeSrdPacketManager */
/* buffer for the received data */
static unsigned char m_aucRxMessageBuf[MAX_S2H_MESSAGE_LENGTH];	/*holder of current packet */
static unsigned char m_aucTxMessageBuf[MAX_S2H_MESSAGE_LENGTH];	/*holder of current packet - sensor rpc */
static unsigned int m_uiRxLen;				/* holder of packet length */
/*include flag+flagbytes+rpcs, not include sop, eop, crc*/
static unsigned char m_ucRxCrc;

static unsigned char *m_pucTxMsg;
static unsigned int m_usTxSize;		/* length from ESC SOP to ESC EOP */
static unsigned char m_ucTxCrc;

static unsigned int m_uiState;
static unsigned char m_ucRxSeqId;
static unsigned char m_ucLastAckSeqId;




/**
 * CRC table - from GlUtlCrc::ucCrcTable
 *
 */
static const unsigned char crc_table [] =
{
	0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
	0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
	0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
	0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
	0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
	0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
	0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
	0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
	0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
	0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
	0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
	0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
	0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
	0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
	0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
	0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
};

/**
 * crc_calc from GlUtlCrc::GlUtlCrcCalc
 *
 *
 */

static inline unsigned char crc_calc(unsigned char *m_ucCrcState, unsigned char ucData) /* update with new data byte and get result */
{
	*m_ucCrcState = crc_table[*m_ucCrcState ^ ucData];
	return *m_ucCrcState;
}


/**
 * crc_calc_many - from GlUtlCrc::GlUtlCrcCalc
 *
 */
static unsigned char crc_calc_many(unsigned char *m_ucCrcState, const unsigned char *pucData, unsigned short usLen)
{
	while (usLen--)
	{
		*m_ucCrcState =  crc_table[*m_ucCrcState ^ (*pucData++)];
	}
	return *m_ucCrcState;
}

/**
 * escape_and_append - GlMeSrdPacketManager::EscapeAndAppend
 *
 */
static void escape_and_append(unsigned char ucByte)
{
	if (ucByte == ESC_BYTE)
	{
		*m_pucTxMsg++ = ESC_BYTE;
		*m_pucTxMsg++ = ESCAPED_ESC_BYTE;
	}
	else if (ucByte == XON_BYTE)
	{
		*m_pucTxMsg++ = ESC_BYTE;
		*m_pucTxMsg++ = ESCAPED_XON_BYTE;
	}
	else if (ucByte == XOFF_BYTE)
	{
		*m_pucTxMsg++ = ESC_BYTE;
		*m_pucTxMsg++ = ESCAPED_XOFF_BYTE;
	}
	else
	{
		*m_pucTxMsg++ = ucByte;
	}
}

/**
 * clear_packet - from GlMeSrdPacketManager::Clear
 *
 *
 */
static void clear_packet(void)
{
	m_pucTxMsg = m_aucTxMessageBuf;
	*m_pucTxMsg++ = ESC_BYTE;
	*m_pucTxMsg++ = SOP_BYTE;
	m_ucTxCrc = 0;
 	/*Unlike GlMeSrdPacketManager::Clear, we don't set tx seq id here. */
}

/**
 * append_data - from GlMeSrdPacketManager::AppendData
 *
 */
/* Must be called to concatenate data */
static void append_data(unsigned char *pucData, unsigned short usLen)
{
	while (usLen-- > 0)
	{
		crc_calc(&m_ucTxCrc, *pucData);
		escape_and_append(*pucData++);
	}
}


/**
 * parse_method - from SardineGllRpcParserImp::ParseMethod
 *
 * Return: 
 *   true	for sensor rpc
 *   false	for other rpc
 */
static bool parse_method(unsigned char ucClassId, unsigned char ucMethodId, char *pucMethodData, unsigned short usMethodLen)
{
	bool ret = false;
	unsigned short idx=0;

	switch(ucClassId)
	{
		case GLLRPC_CLASS(esarpt):
				switch(ucMethodId)
		{
					case RPC_METHOD(esarpt,sh2host):
			bbd_sensor_write(pucMethodData, usMethodLen);
						break;
					case RPC_METHOD(esarpt,esa2host):
			/* Byte 0: Protocol version (not at 0)
			 * Byte 1: Command (ESA2HOST_CMD_SHOFF)
			 * Byte 2..5: Timestamp
			 */
			printk("Received RPC_METHOD(esarpt,esa2host) payload byte[0]=%02X, byte[1]=%02X\n", pucMethodData[0], pucMethodData[1]);
			if (pucMethodData[0]==0 && pucMethodData[1]==ESA2HOST_CMD_SHOFF)
							bbd_request_mcu(false);
			break;
				};
		break;
		}
		return ret;
}


/**
 * on_packet - from GlMeSrdPacketManagerCallbackImpl::OnPacket
 *
 *		When this call return, m_aucRxMessageBuf[] has non-sensor packet (seqid, flag, flagbytes, non-sensor rpc). Please give it to gpsd
 * @pucData
 * @usLen
 */
static bool on_packet(char *pucData, unsigned short usLen)
{
	unsigned short usOffset;

	for (usOffset=0; usOffset<usLen ;)
	{
		unsigned short usMethodLen = pucData[usOffset];
		unsigned char ucClassId = pucData[usOffset+1];
		unsigned char ucMethodId = pucData[usOffset+2];
		char *pucMethodData = &pucData[usOffset+3];

		/* the 2 MSb of the classId are used as the bit8 and bit9 of the method leni */
		usMethodLen += (ucClassId & 0xc0)<<2;
		/* bit 5 of classId is used as bit10 of the method len */
		usMethodLen += (ucClassId & 0x20)<<5;
		ucClassId &= 0x1f;

		/* Send DAC data to UDP socket  //FIXME: What is DAC?? */
		/*m_rTransactionManager.m_rTransportCallback.OnDacData(&pucData[usOffset], usMethodLen + ESW_RPC_HDR_SZ); //1 for the len itself, 3 for classId, instanceId, methodId */
	if (!parse_method(ucClassId,ucMethodId, pucMethodData,usMethodLen)) {
			append_data(pucData+usOffset, usMethodLen+ESW_RPC_HDR_SZ);
	}

	/* jump to next rpc messge */
		usOffset += usMethodLen  + ESW_RPC_HDR_SZ; /*1 for the len itself, 2 for classId, methodId */
	}

	return true;
}


/**
 * check_packet_sanity - from GlMeSrdPacketManager::CheckPacketSanity
 *
 *
 */
static bool check_packet_sanity(void)
{
	int i;
	bool bSanity = false;
	/* the first thing we want to check is if the packet seems valid. We already had the CRC passing,
	 * now we can try to see if the packets Flag section is valid, and if the transaction is valid as well
	 */
	char ucFlags = m_aucRxMessageBuf[1];
	const unsigned char ucValidFlags = TRANSPORT_FLAG_ACK | TRANSPORT_FLAG_RELIABLE | TRANSPORT_FLAG_MSG_LOST | TRANSPORT_FLAG_MSG_GARBAGE | TRANSPORT_FLAG_RELIABLE_SEQ_ERROR;
	if ((ucFlags&(~ucValidFlags)) == 0) /* only some bits should be set in the flags */
	{
		unsigned int uiBitCnt = 0;
		for (i=0; i<8 && ucFlags != 0; i++)
		{
			if (((1<<i)&ucFlags) != 0)
			{
				uiBitCnt++;
			}
		}

		if (m_uiRxLen >= uiBitCnt+2+ESW_RPC_HDR_SZ) /* SeqId, Flags, and at least BitCnt of extra flag info + at least one RPC */
		{
			unsigned short usLen = m_uiRxLen - 2 - uiBitCnt;		/* Len for all RPCs */
			char *pucData = &m_aucRxMessageBuf[2+uiBitCnt]; /* buffer for all RPCs */
			unsigned short usOffset;

			bSanity = true; /* assume true unless proved otherwise */
			for (usOffset=0; usOffset<usLen ;)
			{
				unsigned short usMethodLen = pucData[usOffset];
				unsigned char ucClassId = pucData[usOffset+1];

				/* the 3 MSb of the classId are used as the bit8 and bit9 of the method len */
				usMethodLen += (ucClassId & 0xc0)<<2;
				usMethodLen += (ucClassId & 0x20)<<5;
				ucClassId &= 0x1f;

#if 0
		/*FIXME: we don't check RPC class valid because we just intercept GLLRPC_CLASS(misc) and this should get free pass. See GlMeSrdAsicFeature::IsRpcClassValid */
				if( !m_rAsicFeatureIfc.IsRpcClassValid(ucClassId) )
				{
					bSanity = false;
				}
#endif
				usOffset += usMethodLen  + ESW_RPC_HDR_SZ; //1 for the len itself, 2 for classId, methodId
			}

			if (usOffset != usLen)
			{
				bSanity = false;
			}
		}
		else if (m_uiRxLen == (2 + uiBitCnt)) /* no RPC */
		{
			bSanity = true;
			if (uiBitCnt == 0) /* no flags either, this is an empty message */
			{
				/* This can happen in some cases in ESW when the "super" transaction is empty because some tracks don't have COPs to run yet
				 * In any case, this is very rare, so we decided it was ok for GLL to receive them, only some extra IO
				 */
				/* pr_warn("PacketMgr::Received Empty Packet from ESW!\n"); */
			}
		}
	}
	return bSanity;
}




/**
 * packet_received - from SardineGllRpcparserImp::ParseMethod
 *
 */
static bool packet_received(void)
{
	bool bRet = true;
	if (!check_packet_sanity()) {
		bRet = false;
	} else {
	char *pucData;
	short sLen;
	unsigned char ucFlags;
	bool bReliableNack, bAckReceived;
	unsigned int i;

	append_data(&m_aucRxMessageBuf[0], 1);	/* seqid */

		if (m_ucRxSeqId != m_aucRxMessageBuf[0]) {
			unsigned short usReceivedLost = (m_aucRxMessageBuf[0]-m_ucRxSeqId)&0xFF;
			m_otStats.otPublic.ulRxSeqErrors += usReceivedLost;
		}
		m_ucRxSeqId = (m_aucRxMessageBuf[0]+1)&0xFF;

		pucData = &m_aucRxMessageBuf[1];
		sLen = m_uiRxLen-1;

	append_data(&m_aucRxMessageBuf[1], 1);	/* flags */

		/* in every packet, we start with the flags */
		ucFlags = *pucData++; sLen--;
		bReliableNack = (ucFlags&TRANSPORT_FLAG_RELIABLE_SEQ_ERROR) == TRANSPORT_FLAG_RELIABLE_SEQ_ERROR;
		bAckReceived = (ucFlags&TRANSPORT_FLAG_ACK) == TRANSPORT_FLAG_ACK;

		for (i=0; i<8 && ucFlags != 0; i++)
		{
			if ( ((1<<i)&ucFlags) == TRANSPORT_FLAG_ACK)
			{
		unsigned char ucExpectedAckSeqId;
		unsigned char ucDeltaAckSeqId;
				unsigned char ucAckedSeqId = *pucData++; sLen--; ucFlags &= ~TRANSPORT_FLAG_ACK;

		append_data(&ucAckedSeqId, 1);	/*flags*/


				ucExpectedAckSeqId = (m_ucLastAckSeqId + 1) & 0xFF;
				ucDeltaAckSeqId = (ucAckedSeqId - ucExpectedAckSeqId) & 0xFF;
				if (ucDeltaAckSeqId)
				{
					pr_warn("PacketMgr::Acknowlegment lost(%u)!\n", ucDeltaAckSeqId);
				}
#if 0
				UpdateLatencyStatsOnAckReceived(ucAckedSeqId, ucDeltaAckSeqId);
#endif

				m_ucLastAckSeqId = ucAckedSeqId;
				pr_warn("PacketMgr::Acknowledgement Received for SeqId %u\n",ucAckedSeqId);

				/* we have received an Ack. It might be acknowledging a reliable message, so let's pass the info to the reliable part.
				 * make sure it's a Ack that is not complaining about a reliable Seq Error
				 */
				if (!bReliableNack)
				{
#if 0
					ReliableAckReceived(m_ucLastAckSeqId);
#endif
				}
			}
			else if (((1<<i)&ucFlags) == TRANSPORT_FLAG_RELIABLE) /* shouldn't happen from ESW, but unittest is using the transport to decode data coming from GLL! */
			{

		unsigned char c = *pucData;

				pucData++; sLen--; ucFlags &= ~TRANSPORT_FLAG_RELIABLE;

		append_data(&c, 1);

			}
			else if (((1<<i)&ucFlags) == TRANSPORT_FLAG_MSG_LOST)
			{
				unsigned char ucMsgLost = *pucData++; sLen--; ucFlags &= ~TRANSPORT_FLAG_MSG_LOST;

		append_data(&ucMsgLost, 1);	/* msg_lost */

				m_otStats.otPublic.ulTxSeqErrors += ucMsgLost;
			}
			else if (((1<<i)&ucFlags) == TRANSPORT_FLAG_MSG_GARBAGE)
			{
				unsigned char ucMsgGarbage = *pucData++; sLen--; ucFlags &= ~TRANSPORT_FLAG_MSG_GARBAGE;

		append_data(&ucMsgGarbage, 1);	/* msg_lost */
			}
			else if (((1<<i)&ucFlags) == TRANSPORT_FLAG_RELIABLE_SEQ_ERROR)
			{
				unsigned char ucLastReliableSeqId = *pucData++; sLen--; ucFlags &= ~TRANSPORT_FLAG_RELIABLE_SEQ_ERROR;

		append_data(&ucLastReliableSeqId, 1);	/* msg_lost */

				/* we have just been nacked the following */
				/*pr_warn("PacketMgr::ESW reported Wrong Reliable SeqId(Last Valid: %u)!\n", ucLastReliableSeqId);*/

				/* we have received a reliable ack. This is because the wrong reliable SeqId was received, usually indicating that a reliable packet was lost
				 * in that case, we do not want to wait for the timer to kick in, we do resend the packets right away
				 */
				WARN_ON(!bAckReceived); /* This NACK should only be recveived with an ACK as it is also ACKing a packet. */
#if 0
				ReliableNackReceived(m_ucLastAckSeqId, ucLastReliableSeqId);
#endif
			}
		}
		WARN_ON(ucFlags != 0); /* make sure we processed all flags otherwise we might not be at the right place in the buffer! */
		WARN_ON(sLen < 0); /* looking for usLen negative (rollover) which indicates */


		m_otStats.otPublic.ulRxMessage++;

		if (sLen > 0)
		{
			/* Should we call this before taking all the action on the first
		 * bytes? in case CRC passes but it is actually garbage!
		 */
			if (!on_packet(pucData, (unsigned short)sLen))
			{
				bRet = false;
			}

		/* prepare CRC ESC EOP - copied from
		 * GlMeSrdPacketManager::SendPacket
		 */
			{
				unsigned char crc = m_ucTxCrc;
			    /* CRC has its nibble inverted for stronger CRC (as CRC of a packet with itself is always 0, if EoP is not detected, taht always reset the CRC) */
				crc = ((crc&0x0F)<<4) | ((crc&0xF0)>>4);
				escape_and_append(crc);
				*m_pucTxMsg++ = ESC_BYTE;
				*m_pucTxMsg++ = EOP_BYTE;
				 m_usTxSize = m_pucTxMsg-m_aucTxMessageBuf;
			 }

		}
	}

	return bRet;
}



/**
 * bbd_parse_asic_data  - from GlMeSrdPacketManager::ParseAsicData
 *
 * @pucData
 * @usLen
 */
void bbd_parse_asic_data(unsigned char *pucData, unsigned short usLen, void (*to_gpsd)(unsigned char *packet, unsigned short len, void* priv), void* priv)
{
	bool bGarbage = false;
	unsigned int usIdx=0;
	unsigned int usGarbageIdx=0;

	while (usIdx != usLen)
	{
		unsigned char ucData = pucData[usIdx++];
		m_otStats.otPublic.ulRxBytes++;

		if (ucData == XON_BYTE || ucData == XOFF_BYTE)
		{
			bGarbage = true;
			continue;
		}

		switch(m_uiState)
		{
			case WAIT_FOR_ESC_SOP:
			{
				if (ucData == ESC_BYTE)
				{
					m_uiState = WAIT_FOR_SOP;
				}
#if 0
				else if (m_bBurstModeEnabled && ucData == m_ucBurstModeDummyByte)
#else
		/* FIXME: need to support burst mode on/off and configurable dummy byte. For now, hardcoded as 0x00 which is used by GLL */
				else if (ucData == 0x00)
#endif
				{
					/* do nothing, it is expected */
				}
				else
				{
					bGarbage = true;
				}
			}
			break;

			case WAIT_FOR_SOP:
			{
				if (ucData == SOP_BYTE)
				{
					m_uiState = WAIT_FOR_MESSAGE_COMPLETE;
					if (m_uiRxLen > m_otStats.usMaxRxBufUse)
					{
						m_otStats.usMaxRxBufUse = m_uiRxLen;
					}
					m_uiRxLen = 0;

					clear_packet();

				}
				else
				{
					bGarbage = true;
					if (ucData != ESC_BYTE)
					{
						m_uiState = WAIT_FOR_ESC_SOP;
					}
				}
			}
			break;

			case WAIT_FOR_MESSAGE_COMPLETE:
			{
				if (ucData == ESC_BYTE)
				{
					m_uiState = WAIT_FOR_EOP;
				}
				else if (m_uiRxLen < MAX_S2H_MESSAGE_LENGTH)
				{
					m_aucRxMessageBuf[m_uiRxLen++] = ucData;
				}
				else
				{
					bGarbage = true;
					m_uiState = WAIT_FOR_ESC_SOP;
				}
			}
			break;

			case WAIT_FOR_EOP:
			{
				if (ucData == EOP_BYTE)
				{
					if (m_uiRxLen >= 3) /* minimum is seqId and Crc and one byte of data (flags) */
					{
						/* got message */
						unsigned char crc=0;
						crc = crc_calc_many(&crc, m_aucRxMessageBuf, --m_uiRxLen);
						/* CRC has its nibble inverted for stronger CRC (as CRC of a packet with itself is always 0, if EoP is not detected, taht always reset the CRC) */
						crc = ((crc&0x0F)<<4) | ((crc&0xF0)>>4);
						if (crc == m_aucRxMessageBuf[m_uiRxLen])
						{
							/* should we notify if a packet was lost?? (seqId not continuous) and that we detected some garbage?? */
							if (bGarbage)
							{
								bGarbage = false;
								m_otStats.otPublic.ulGarbageBytes += (usIdx-usGarbageIdx);
							}

							if (packet_received())
							{

								/* After packet_received() call, non-sensor RPC messages are prepared as a packet in m_aucTxMessageBuf with length m_usTxSize;
								 * Give it to upper layer
								 */
				if (to_gpsd)
					to_gpsd(m_aucTxMessageBuf, m_usTxSize, priv);

							}
							else
							{
								m_otStats.otPublic.ulGarbageBytes += (usIdx-usGarbageIdx);
							}

							usGarbageIdx = usIdx;
						}
						else
						{
							bGarbage = true;
						}
					}
					else
					{
						bGarbage = true;
					}

					m_uiState = WAIT_FOR_ESC_SOP;
				}
				else if (ucData == ESCAPED_ESC_BYTE)
				{
					if (m_uiRxLen < MAX_S2H_MESSAGE_LENGTH)
					{
						m_aucRxMessageBuf[m_uiRxLen++] = ESC_BYTE;
						m_uiState = WAIT_FOR_MESSAGE_COMPLETE;
					}
					else
					{
						bGarbage = true;
						m_uiState = WAIT_FOR_ESC_SOP;
					}
				}
				else if (ucData == ESCAPED_XON_BYTE)
				{
					if (m_uiRxLen < MAX_S2H_MESSAGE_LENGTH)
					{
						m_aucRxMessageBuf[m_uiRxLen++] = XON_BYTE;
						m_uiState = WAIT_FOR_MESSAGE_COMPLETE;
					}
					else
					{
						bGarbage = true;
						m_uiState = WAIT_FOR_ESC_SOP;
					}
				}
				else if (ucData == ESCAPED_XOFF_BYTE)
				{
					if (m_uiRxLen < MAX_S2H_MESSAGE_LENGTH)
					{
						m_aucRxMessageBuf[m_uiRxLen++] = XOFF_BYTE;
						m_uiState = WAIT_FOR_MESSAGE_COMPLETE;
					}
					else
					{
						bGarbage = true;
						m_uiState = WAIT_FOR_ESC_SOP;
					}
				}
				else if (ucData == ESC_BYTE)
				{
					bGarbage = true;
					m_uiState = WAIT_FOR_SOP;
				}
				else if (ucData == SOP_BYTE)
				{
					bGarbage = true;
					/* we probably missed the ESC EOP, but we start receiving a new packet! */
					m_uiState = WAIT_FOR_MESSAGE_COMPLETE;
					/* init the parser! */
					if (m_uiRxLen > m_otStats.usMaxRxBufUse)
					{
						m_otStats.usMaxRxBufUse = m_uiRxLen;
					}
					m_uiRxLen = 0;
				}
				else
				{
					bGarbage = true;
					m_uiState = WAIT_FOR_ESC_SOP;
				}
			}
			break;
		}
	}

	/* should we notify if a packet was lost?? (seqId not continuous) and that we detected some garbage?? */
	if (bGarbage)
	{
		m_otStats.otPublic.ulGarbageBytes += (usIdx-usGarbageIdx);
	}
}
EXPORT_SYMBOL(bbd_parse_asic_data);

#ifndef __KERNEL__

/* sample packet 1~4 are from FW4773-145 and it doesn't have sensor RPC.
 * Therefore parse_asic_data() should return packet same with input.
 */
unsigned char sample1[] = { 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x01 };
unsigned char sample2[] = { 0xE0, 0x00, 0x01, 0x01, 0x00, 0x04, 0x02, 0x02, 0xB4, 0xAC, 0xCE, 0x55, 0x04, 0x02, 0x00, 0x05, 0x00, 0x00, 0xA5, 0x04, 0x02, 0x00, 0x05, 0x00, 0x00, 0xA6, 0x00, 0x02, 0x03, 0xEE, 0xE0, 0x01 };
unsigned char sample3[] = {0xE0, 0x00, 0x02, 0x01, 0x01, 0x44, 0x80, 0x01, 0x7E, 0xCE, 0xDC, 0xAF, 0xC5, 0x29, 0x0D, 0x52, 0xD3, 0x7C, 0xAA, 0xA3, 0x04, 0x5D, 0xCB, 0xED, 0x00, 0x4A, 0x8D, 0x32, 0x17, 0x33, 0xD0, 0x7C, 0x03, 0x44, 0x01, 0x45, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x94, 0x10, 0x20, 0x00, 0x03, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x64, 0x80, 0x00, 0x00, 0x64, 0x80, 0x00, 0x00, 0x64, 0x80, 0x00, 0x00, 0x64, 0x80, 0x00, 0x00, 0x64, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xE0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xE0, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0xE0, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xE0, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0xE0, 0x02, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xE0, 0x02, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xE0, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB, 0xE0, 0x02, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF, 0xE0, 0x02, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0xE0, 0x02, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0xE0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6E, 0xE0, 0x02, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xE0, 0x02, 0xE0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xE0, 0x02, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0xE0, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0xE0, 0x02, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0xE0, 0x02, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xE0, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xE0, 0x02, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xE0, 0x02, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0xE0, 0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4A, 0xE0, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0xE0, 0x02, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0xE0, 0x02, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xE0, 0x02, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xE0, 0x02, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE0, 0x02, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x02, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xE0, 0x02, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B, 0xE0, 0x02, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xE0, 0x02, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0xE0, 0x02, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0xE0, 0x02, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xE0, 0x02, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xE0, 0x02, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x02, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB9, 0xE0, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0xC0, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x9E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x01, 0xC0, 0x1D, 0x00, 0x00, 0x00, 0xC6, 0x64, 0x61, 0xC0, 0x5C, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x01, 0xC0, 0x5D, 0x00, 0x00, 0x00, 0xD0, 0x34, 0x61, 0xC0, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x01, 0xC0, 0x9D, 0x00, 0x00, 0x00, 0xCA, 0x54, 0x61, 0xC0, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xC0, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xC0, 0x9E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xC0, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xC0, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xC0, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xC0, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xC0, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xC0, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1E, 0xE0, 0x01};
unsigned char sample4[] = {0xE0, 0x00, 0x03, 0x01, 0x02, 0x03, 0x03, 0x08, 0x01, 0x00, 0x05, 0x02, 0x00, 0x0A, 0x00, 0x06, 0x04, 0x03, 0x00, 0x02, 0x00, 0x01, 0x00, 0x02, 0x00, 0x0A, 0x07, 0x01, 0x06, 0x00, 0x09, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x32, 0xE0, 0x01};

void to_gpsd(unsigned char *packet, unsigned short len, void* priv)
{
	int i = 0;
	unsigned char* org_packet = (unsigned char *) priv;;

	for (i = 0; i < len; i++) {
		if (packet[i]!=org_packet[i]) {
			printf("packet[%d]=0x%02X is differ from org_packet[%d]=0x%02X", i, packet[i], i, org_packet[i]);
			break;
		}

		printf("%02X ", packet[i]);
	}
	printf("\n\n");
}

int main(void)
{
	/* Check if bbd_parse_asic_data breaks non-sensor packet */
	bbd_parse_asic_data(sample1, sizeof(sample1), to_gpsd, sample1);
	bbd_parse_asic_data(sample2, sizeof(sample2), to_gpsd, sample2);
	bbd_parse_asic_data(sample3, sizeof(sample3), to_gpsd, sample3);
	bbd_parse_asic_data(sample4, sizeof(sample4), to_gpsd, sample4);
	return 0;
}

#endif

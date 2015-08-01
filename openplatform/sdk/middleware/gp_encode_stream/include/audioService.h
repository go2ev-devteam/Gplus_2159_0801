/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2013 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park,                      *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
/**
 * @file audioService.h
 * @brief audio service header file
 * @author Yu Tseng Liao (ytliao)
 */
 
#ifndef _AUDIO_H_
#define _AUDIO_H_

#include "AAC_enc.h"
#include "audioEncService.h"
#include "gp_audio_stream_api.h"

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define AUDIO_BUFFER_COUNT 32
//#define AUDIO_SAMPLE_RATE	16000
#define AUDIO_CHANNEL		1
//#define AUDIO_BUFFER_SIZE (AAC_INPUT_FRAME_LEN * AUD_AAC_SAMPLE_TIMES*AUDIO_CHANNEL*2)
#define AUDIO_BUFFER_SIZE 1440*2
#define AUDIO_AAC_BUFFER_SIZE (AAC_INPUT_FRAME_LEN * AUDIO_CHANNEL * 2)
//#define AUDIO_PCM_BUFFER_SIZE 96*2
//#define AUDIO_G711_BUFFER_SIZE 184*2
//#define AUDIO_G726_BUFFER_SIZE 360*2
#define AUDIO_PCM_BUFFER_SIZE AUDIO_BUFFER_SIZE
#define AUDIO_G711_BUFFER_SIZE AUDIO_BUFFER_SIZE
#define AUDIO_G726_BUFFER_SIZE AUDIO_BUFFER_SIZE
//#define AUDIO_BUFFER_SIZE 	(AUDIO_SAMPLE_RATE*0.02*AUDIO_CHANNEL*2)
//#define AUDIO_BUFFER_TIME	(1000*AUDIO_BUFFER_SIZE/(AUDIO_SAMPLE_RATE*AUDIO_CHANNEL*2))

enum {
	AUDIO_CMD_START = 0,
	AUDIO_CMD_STOP,
	AUDIO_CMD_CLOSE,
	AUDIO_CMD_ENABLE,
	AUDIO_CMD_DISABLE,
	AUDIO_CMD_AEC_EN,
	AUDIO_CMD_AEC_DIS,
	AUDIO_CMD_ANR_EN,
	AUDIO_CMD_ANR_DIS,
	AUDIO_CMD_BUFFER,
};
/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct audioInfo_s {
	//int fifo;
	int dsp;
	int mixer;
	bufferHandle_t *bufferHandle;
	int pcmBufferSize;
	audioEncInfo_t audioEncInfo;
	pthread_t audioThread;
	mqd_t audioMsgQ;
	mqd_t audioBufMsgQ;
	IPC_ADev_Attr_s dev_attr;

	pthread_mutex_t a_mutex;
	bufferInfo_t *userbufInfo;
	
	int mute;
} audioInfo_t;

typedef struct audioMsg_s {
	int cmd;
	//void *handle;
} audioMsg_t;

typedef struct audioBufMsg_s {
	int cmd;
	bufferInfo_t *bufferInfo;
	unsigned int nBytes;
	unsigned int pts;
	unsigned int bitwidth;
	unsigned int bitrate;
	unsigned int duration;
	unsigned int seq;
} audioBufMsg_t;
/**************************************************************************
 *                 E X T E R N A L    R E F E R E N C E S                 *
 **************************************************************************/



/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/

int audioCommand(int cmd);
int audioServiceInit();
void audioServiceUnInit();
SINT32 audioServiceSetVolume(audioInfo_t *pAinfo,  int volume );
SINT32 audioServiceEnable( audioInfo_t *pAinfo );
SINT32 audioServiceDisable( audioInfo_t *pAinfo );
SINT32 audioServiceEnableAec( audioInfo_t *pAinfo );
SINT32 audioServiceDisableAec( audioInfo_t *pAinfo );
//SINT32 audioServiceStart(audioInfo_t pAinfo);
SINT32 audioServiceEnableAnr( audioInfo_t *pAinfo );
SINT32 audioServiceDisableAnr( audioInfo_t *pAinfo );
/**
 * start audio devive.
 * @pAinfo[in]: audioInfo_t *.
 * return: GP_SUCCESS: set success
 */
SINT32 audioServiceStart(audioInfo_t *pAinfo);
SINT32 audioServiceStop(audioInfo_t *pAinfo);
#endif

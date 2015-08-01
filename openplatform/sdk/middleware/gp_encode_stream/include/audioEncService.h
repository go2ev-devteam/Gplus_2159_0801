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
 * @file audioEncService.h
 * @brief audio encoder service header file
 * @author Yu Tseng Liao (ytliao)
 */

#ifndef _AUDIOENC_H_
#define _AUDIOENC_H_

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
enum {
	AUDIOENC_CMD_PROCESS = 0,
	AUDIOENC_CMD_STOP,
	AUDIOENC_CMD_CLOSE,
};
/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct audioEncInfo_s {
	bufferHandle_t *bufferHandle;
	bufferInfo_t** inBuffer;
	pthread_t audioEncThread;
	mqd_t audioEncMsgQ;
	int wIdx;
	int rIdx;
	void* pWorkMem;
    int format;
	int samplerate;
	int channel;
	int bitrate;
    int buffersize;
    int buffer_time;
    char *extData;
    int extDataLen;
	int inVol;			/*audio recoder volume*/
} audioEncInfo_t;

typedef struct audioEncMsg_s {
	int cmd;
	bufferInfo_t *bufferInfo;
	int audio_time;
	int buffer_size;
	void *handle;
} audioEncMsg_t; 

/**************************************************************************
 *                 E X T E R N A L    R E F E R E N C E S                 *
 **************************************************************************/
extern audioEncInfo_t *audioEncInfo;

 /**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

int audioEncCommand(int cmd);
int audioEncodeProcess(bufferInfo_t* bufferInfo, int buffer_size, int audio_time);
int audioEncServiceInit();
void audioEncServiceUnInit();
int audioSendToBuffer( bufferInfo_t *pAinfo, UINT32 nBytes, UINT32 pts, UINT32 bitrate, UINT32 duration, UINT32 seq);

#ifdef __cplusplus 
} /* extern "C" */
#endif
#endif

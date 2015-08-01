/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2010 by Generalplus Inc.                         *
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
 * @file videoService.h
 * @brief videoService header file
 * @author 
 */

#ifndef _VIDEO_SERVICE_H_
#define _VIDEO_SERVICE_H_

#include "gp_on2.h"
#include "ceva.h"
#include "bufferManager.h"

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/


enum {
	VIDEO_ENCODE_PROCESS = 0,
	VIDEO_ENCODE_FINISH,
	VIDEO_ENCODE_CLOSE,
	VIDEO_ENCODE_I_FRAME,
	VIDEO_ENCODE_JPG,
	VIDEO_ENCODE_STRM_RESET,
};

enum {
	VIDEO_STAT_JUST_RESET = 0,
	VIDEO_STAT_RUNNING, 
	VIDEO_STAT_LIVE_STREAMING,
	VIDEO_STAT_WAIT_RESET,
};


/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct videoInfo_s {
	//bufferHandle_t *bufferHandle;
} videoInfo_t;

typedef struct PefInfo_s {
	unsigned int process_time;
	unsigned int process_frame;
	unsigned int total_time;
	unsigned int total_frame;	
	unsigned int total_size;
	unsigned int total_skip;
	float current_fps;
	int stable;
}PefInfo_t;

typedef struct videoEncoderHandle_s {
	int loop_flag;
	cevaEncode_t vdt;
	unsigned int timeStamp;		/* unit : ms */
	bufferHandle_t *bufferHandle;
	bufferHandle_t *thumbHandle;
	mqd_t MsgQ;
	pthread_t pThread;
	pthread_mutex_t mutex;
	PefInfo_t pf;
	char *pps_buf;
	int pps_size;
	int un_connect;
	int target_bitrate;
} videoEncoderHandle_t;

typedef struct videoMsg_s {
	int cmd;
	void *handle;
	void *inputImage;
	long long pts;
	cevaEncode_t *vdt;
} videoMsg_t;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
int videoServiceInit(void);
void videoServiceUnInit(void);

void *videoEncoderCreate(int id,cevaEncode_t *vdt,int cache_buf);
void videoEncoderDelete(void *handle);
int videoEncoderProcess(void *streamHandle, void *inputImage, long long pts);
void videoEncoderFinish(void *streamHandle);
//int videoEncoder_GetPSP(void *hanlde,char **psp_buf,int *size);
int videoEncoderReInitial(void *streamHandle,int bitrate,int framerate,int gopLen,int bitrateMode);
void videoEncoderBufferFree(bufferInfo_t *bufferInfo);
void videoEncoderKeyframe(void *streamHandle);
void videoEncoderStreamReset(void *streamHandle);
//int KeyFrame_include_PPS(char *buf,int size);
int JPGEncoder(void *streamHandle,cevaEncode_t *vdt);
#endif //endif _VIDEO_SERVICE_H_

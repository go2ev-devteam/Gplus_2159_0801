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
 * @file scaleService.h
 * @brief scale service header file
 * @author 
 */

#ifndef _SCALE_SERVICE_H_
#define _SCALE_SERVICE_H_

#include <mqueue.h>
#include "bufferManager.h"
#include "csiService.h"

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
enum {
	SCALE_CMD_PROCESS = 0,
	SCALE_CMD_FINISH,
	SCALE_CMD_CLOSE,
};

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct scaleInfo_s {
	//int devHandle;
	//int devHandle2;
	//bufferHandle_t *bufferHandle;
} scaleInfo_t;

typedef struct scaleHandle_s {
	int loop_flag;
	int inputWidth;
	int inputHeight;
	int outputWidth;
	int outputHeight;
	unsigned int total_time;
	unsigned int total_frame;
	bufferHandle_t *bufferHandle;
	mqd_t MsgQ;
	pthread_t pThread;
	int un_connect;
} scaleHandle_t;

typedef struct scaleMsg_s {
	int cmd;
	void *handle;
	void *inputImage;
	long long pts;
	int zoom;
} scaleMsg_t;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
int scaleServiceInit(void);
void scaleServiceUnInit();

void *scaleCreate(int id, int inputWidth, int inputHeight, int outputWidth, int outputHeight);
int scaleProcess(void *streamHandle, void /*csiImage_t*/ *csiImage, long long pts, int zoom);
void scaleFinish(void *streamHandle);
void scaleBufferFree(bufferInfo_t *bufferInfo);
void scaleDelete(void *handle);

#endif //endif _SCALE_SERVICE_H_

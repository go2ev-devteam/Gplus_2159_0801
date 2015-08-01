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
 * @file csiInput.h
 * @brief csi input header file
 * @author 
 */

#ifndef _CSI_INPUT_H_
#define _CSI_INPUT_H_

#include <linux/videodev2.h>
#include "pthread.h"
#include <mqueue.h>
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define  MD_EN 1
#define  LDW_EN	1
 
#define CSI_STATE_OFF 0x0
#define CSI_STATE_ON 0x1
#define CSI_STATE_ERR 0x2

#define CSI_CDSP	0
#define CSI_V4L2	1

enum {
	CSI_CMD_STREAM_ON = 0,
	CSI_CMD_STREAM_OFF,
	CSI_CMD_STREAM_CLOSE,
	CSI_CMD_SERVICE_CLOSE,
	CSI_CMD_SNAP_SHOT,
	CSI_CMD_SNAP_SHOT_FINISH,
	CSI_CMD_RESOLUTION,
	CSI_CMD_QR_SCAN,
	CSI_CMD_QR_SCAN_STOP,
	CSI_CMD_RESTART_STREAM,
};

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct csiInfo_s {
	int csiHandle;
	int mipiHandle;
	unsigned int sensorAddr;
	int width;
	int height;
	int crop_height;
	int cdsp;
	int fps;
	int csiState;
	mqd_t csiMsgQ;
	pthread_t csiThread;
	int buffer_size;
	int mem_type;
	char name[64];
	unsigned int source;
	int display_target;
	int raw_mode;
} csiInfo_t;

typedef struct csiImage_s {
	int id;
	pthread_mutex_t mutex;
	struct v4l2_buffer v4l2Buffer;
	unsigned int refCount;
	csiInfo_t* info;
} csiImage_t;

typedef struct csiMsg_s {
	int cmd;
	void *handle;
} csiMsg_t;

typedef struct csiQRcode_s {
	int notify;
	char* code;
	int len;
} csiQRcode_t;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
typedef int csiMD_work_f(void *buf);
int csiServiceInit();
void csiServiceUnInit();
void csiStreamPlay(unsigned int source);
void csiStreamStop(unsigned int source);
void csiStreamClose(void*);
void csiServiceClose(void *streamHandle);
void csiGetResolution(unsigned int source, int *width, int *height);
void csiImageAddRef(csiImage_t *csiImage);
void csiImageFree(csiImage_t *csiImage);
void csiSnapShot(unsigned int source, void* notify);
void csiSnapShotFinish(unsigned int source);
void csiSetResolution(int res);
void csiQRcodeScan(csiQRcode_t* QR);
void csiQRcodeScanStop();
int csiQueryStatus(unsigned int source);
#ifdef ENABLE_MULTI_SENSOR
void csiSetPIPMode(int mode);
#endif
void csiRestartCSIStream();
void csiRestartV4L2Stream();

//#if  MD_EN 
int csiMD_Register(csiMD_work_f *md_fun);
//#endif
#endif //endif _CSI_INPUT_H_

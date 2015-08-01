#ifndef __JPGSERVICE__H__
#define __JPGSERVICE__H__

#include "gp_on2.h"
#include "ceva.h"
#include <mqueue.h>
#include "bufferManager.h"

enum {
	JPG_ENCODE_PROCESS = 0,
	JPG_ENCODE_PROCESS_SLICE,
	JPG_ENCODE_THUMBNAIL,
	JPG_ENCODE_ENABLE,
	JPG_ENCODE_FINISH,
	JPG_ENCODE_CLOSE,
};

typedef struct jpgMsg_s {
	int cmd;
	void *handle;
	void *inputImage;
	int slice_idx;
	cevaEncode_t *vdt;
} jpgMsg_t;

typedef struct JpgEncoderHandle_s
{
	int loop_flag;
	cevaEncode_t vdt;
	int un_connect;
	int enable_timestamp;
	mqd_t MsgQ;
	pthread_t pThread;
	int JPG_Encode_Enable;
	bufferHandle_t *bufferHandle;
}JpgEncoderHandle_t;

/************************************************************
funtion list
*************************************************************/
int JPGServiceInit(void);
void JPGServiceUnInit(void);
void *JPGEncoderCreate(int id);
void jpgEncoderDelete(void *arg);
int JPG_capture(void *streamHandle,cevaEncode_t *vdt);
int jpgEncoderProcess(void *streamHandle, void *inputImage, unsigned int pts);
int jpgEncoderProcessSlice(void *streamHandle, void *inputImage, int slice_idx);
int jpgEncoderThumbnail(void *streamHandle,	void *inputImage);
void jpgEncoderFinish(void *streamHandle);
#endif

#ifndef __GP__VIDEO__CHANNEL__H__
#define __GP__VIDEO__CHANNEL__H__

#include "mach/typedef.h"
#include "gp_video_stream.h"
#include "pthread.h"
#include "gp_encode_env.h"
#include "gp_video_stream_api.h"
#include "gp_multi_sensor.h"

//typedef struct gpIPC_VCHN_STAT_S {
//    UINT32 enable;      /*channel enable flag*/
//    UINT32 framerate;   /*channel current fame rate*/
//    UINT32 VbFail;      /*channel video buffer malloc fail*/
//    UINT32 picWidth;    /*channel pic width*/
//    UINT32 picHeight;   /*channel pic height*/
//}IPC_VChn_Stat_s;
//
//
//typedef struct gpIPC_MD_AREA_S {
//    gp_rect_t rect;     /*md area*/
//    UINT32 MdId;        /*md id*/
//}IPC_Md_Area_s;


/***********************************
this struct use for inter code
***********************************/
typedef struct vidCh_info_s
{
	vid_streamInfo_t stream_info;
	void *stream_handle;
	UINT64  seq;
	UINT64  cur_read_seq; //for record the read seq
	SINT64  last_pts;   
	pthread_mutex_t mutex;
	sem_t sem;
	int  frame_disable;
	UINT64 total_size;
	struct gpIPC_VIDEO_FRAME_S frame[0];
}vidCh_info_t;

void* get_empty_frame(void* arg);
int write_frame(void *arg, bufferInfo_t *bufferInfo, unsigned int size, long long pts, int frame_type, void* thumb, int thumb_size,	void* out_frame);
#endif

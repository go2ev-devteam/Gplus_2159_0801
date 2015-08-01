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
 * @file stream.h
 * @brief stream header file
 * @author 
 */

#ifndef __GP__VIDEO__STREAM__H__
#define __GP__VIDEO__STREAM__H__

#include "scaleService.h"
#include "videoService.h"
#include "list.h"
#include "bufferManager.h"
#include "jpgService.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/


/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct vid_streamInfo_s {
	char *name;
	int id;
	int width;
	int height;
	int bitrate;
	int framerate;
	int target_fps;
	int bitrateMode;
	int gopLen;           /*channel I/P ratio*/
	int cache_buf_num;
	int scaler_type;
	int vstb_offset;
	int enable_thumb;
	unsigned int source;
	int reset_length;
	int max_size;
	int sps_pps;
} vid_streamInfo_t;



typedef int vid_stream_callback(void *argv,bufferInfo_t *bufferInfo,unsigned int size, long long pts,int frame_type, void* thumb, int thumb_size);
typedef struct vid_streamHandle_s {
	struct list_head list;
	struct list_head osd_list;
	int enable_osd;
	int delay_osd;
	pthread_mutex_t osd_mutex;
	int width,height;
	scaleHandle_t *scaleHandle;
	videoEncoderHandle_t *videoHandle;
	bufferHandle_t *bufferHandle;
	JpgEncoderHandle_t *jpgHandle;
	int frame_rate;
	int double_fps;
	int bit_rate;
	int frame_count;
	int time_lapse_count;
	unsigned int skip_pat;
	int display;
	vid_stream_callback *callback;
	void *argv;
	int scaler_type;
	int lbp_md;
	int enable_scale;
	int enable_thumb;
	int cache_count;
	int slice_height;
	unsigned int source;
	int reset_length;
	int max_size;
	int current_size;
	int sps_pps;
	long long stream_start_time;
	int frame_interval;
	int frame_time;
} vid_streamHandle_t;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/

void* vid_streamCreate(vid_streamInfo_t info,vid_stream_callback *op,void *arg);
int vid_streamDelete(void *handle);
int vid_streamEnable(void *arg,int enable);
int vid_streamEncodeChange(void *streamHandle,int bitrate,int framerate,int gopLen,int bitrateMode);
void osd_init(void);
void osd_exit(void);
void osd_draw(vid_streamHandle_t* stream, UINT8* dst);

extern struct list_head gStreamList;

#endif //endif _STREAM_H_

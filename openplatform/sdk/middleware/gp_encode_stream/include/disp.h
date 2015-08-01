#ifndef _DISP_H
#define _DISP_H

#include "mach/typedef.h"
#include "gp_video_channel.h"

#define DISP_LAYER_PRIMARY 0
#define DISP_LAYER_OSD 1

#define DISP_TARGET_NONE	-1
#define DISP_TARGET_PRIMARY	0
#define DISP_TARGET_SECONDARY	1
#define DISP_TARGET_LEFT	2
#define DISP_TARGET_RIGHT	3

UINT32 dispCreate(HANDLE *pHandle, UINT32 layer, int aspect_w, int aspect_h);
UINT32 dispDestroy(HANDLE handle);
void dispGetResolution(HANDLE handle, gp_size_t *resolution);
void* dispGetFramebuffer(HANDLE handle);
void dispFlip(HANDLE handle, gp_bitmap_t* fb);

enum {
	DISPLAY_CMD_PROCESS = 0,
	DISPLAY_CMD_STOP,
	DISPLAY_CMD_SNAP_SHOT,
	DISPLAY_CMD_SNAP_SHOT_FINISH,
	DISPLAY_CMD_CLOSE,
};

typedef struct displayMsg_s {
	int cmd;
	void *buffer;
	int width;
	int height;
	//void *handle;
	int zoom;
	int target;
} displayMsg_t;	

int init_disp(int is4x3);
int uinit_disp();
int init_disp2(int is4x3, display_param_t* param);
int uinit_disp2();
int displayCommand(int cmd);
int displayClose();
int displayBuffer(void* img, int width, int height, int zoom, int target);
int displaySnapShot(void* img, int width, int height, int zoom, int target);
int displaySnapShotFinish(void);

#endif

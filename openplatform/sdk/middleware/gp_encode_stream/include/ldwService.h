#ifndef _LDW_H
#define _LDW_H

#include "mach/typedef.h"
#include "gp_video_channel.h"
#include "gp_cvr.h"

#define	USE_SCALE	0
#define	PLAY_SOUND	0

enum {
	LDW_CMD_PROCESS = 0,
	LDW_CMD_CLOSE,
};

enum {
	SOUND_ALARM = 0,
	SOUND_TURN_ON,
	SOUND_TURN_OFF,
	SOUND_OFF,
};

typedef struct ldwMsg_s {
	int cmd;
	void *buffer;
	int width;
	int height;
} ldwMsg_t;	

SINT32 ldwInit();
void ldwUnInit();
SINT32 ldwStart();
void ldwStop();
//int lbpClose();
int ldwSendBuffer(void* img, int width, int height);
void ldwSet(LDW_param_t *param);
void ldwSetResolution(int width, int height);
void LDW_GetLine(LDW_DISPLINE_t *para);

#endif

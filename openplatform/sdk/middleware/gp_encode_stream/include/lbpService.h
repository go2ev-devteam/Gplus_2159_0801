#ifndef _LBP_H
#define _LBP_H

#include "mach/typedef.h"

enum {
	LBP_CMD_PROCESS = 0,
	LBP_CMD_CLOSE,
};

typedef struct lbpMsg_s {
	int cmd;
	void *buffer;
	int width;
	int height;
} lbpMsg_t;	

SINT32 lbpInit(int width, int height, int threshold);
void lbpUninit();

int lbpClose();
int lbpSendBuffer(void* img, int width, int height);

#endif

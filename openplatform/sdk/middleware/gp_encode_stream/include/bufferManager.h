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
 * @file bufferManager.h
 * @brief buffer manager header file
 * @author 
 */

#ifndef _BUFFER_MANAGER_H_
#define _BUFFER_MANAGER_H_

#include <semaphore.h>
#include <pthread.h>
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define ID_BUFFER_MANAGER	0
#define ID_CSI_IMAGE		1
#define ID_CSI_JPG_IMAGE	2



/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct bufferInfo_s {
	int id;
	void *buf;
	int ref;
	void *bufferHandle;
	int idx;
} bufferInfo_t;

typedef struct bufferHandle_s {
	unsigned int size;
	unsigned int count;
	unsigned int used_count; //-1 meant can not malloc buffer.
	pthread_mutex_t mutex;
	sem_t sem;
	void *pbuf;
	bufferInfo_t *info;
	int wait;
} bufferHandle_t;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void *bufferCreate(unsigned int size, unsigned int count);
void *bufferCreate2(unsigned int size,	unsigned int count,	unsigned int phy_addr);
void bufferDelete(void* handle);

bufferInfo_t* bufferMalloc(void *handle, int time_wait);
void bufferAddRef(bufferInfo_t* info);
void bufferFree(bufferInfo_t* info);
int can_buffer_delete(void *handle);

#ifdef __cplusplus 
} /* extern "C" */
#endif
#endif //endif _BUFFER_MANAGER_H_

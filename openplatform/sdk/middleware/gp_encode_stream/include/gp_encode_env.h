#ifndef __GP__ENCODE__ENV__H__
#define __GP__ENCODE__ENV__H__

//#include "list.h"
#include <pthread.h>
#include "gp_video_stream_api.h"

typedef struct gpIPC_Config_s
{
	int enable_display;
	IPC_Scale_mode_e display_scaler_type; /*0: integraton 1:bilinear (faster)*/
	int display_skip; //skip 1 frame for every n frames.
#ifdef ENABLE_MULTI_SENSOR
	int display_skip2;
	IPC_Scale_mode_e pip_scaler_type;
#endif	
	stream_mode_e  stream_mode;
	int timestamp;
	int time_format;
	int zoom;
	int enable_ldw;
	int enable_sensor2;
	int PIP_mode;
	void *sensor_calibrate;
	void *cdsp_user_preference;
	int raw_mode;
	int dynamic_fps;
	int evaluation_only;
}gpIPC_Config_t;

/********************************
globe value for user
*********************************/
extern gpIPC_Config_t gpConfig;
extern int g_mem_fd;
extern pthread_mutex_t g_list_mutex;
extern pthread_mutex_t g_disp_mutex;
extern pthread_mutex_t g_md_mutex;
extern struct list_head gStreamList;


int debug_log_open(char* name);
void debug_log_close();
int debug_log_print(char* string);

#define CSI_STAT 0
#define SCL_STAT 1
#define VID_STAT 2
#define LBP_STAT 3
#define DIS_STAT 4
#define AUD_STAT 5
#define AEN_STAT 6
#define RDF_STAT 7
#define RDA_STAT 8
#define STATUS_NUMBER 9

extern unsigned int env_status[STATUS_NUMBER];

#define _STATE(idx, st)	env_status[idx] = st;
#endif

#ifndef __GP_MULTI_SENSOR_H__
#define __GP_MULTI_SENSOR_H__

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
//enable sensor2 flag in gp_IPC_VStream_Open()
#define CFG_SENSOR2	0x1000 

//Display Picture in Picture mode select, set in gp_IPC_Set_PIPMode()
#define CVR_PIP_MAIN_ONLY	0
#define CVR_PIP_SECOND_ONLY	1
#define CVR_PIP_MAIN_1_SECOND_2	2
#define CVR_PIP_MAIN_2_SECOND_1	3
#define CVR_PIP_SPLIT_SCREEN	4

//source value for IPC_Chn_Attr_s structure, 0: for cdsp sensor 1: for pc-camera
#define CVR_CAMERA_MAIN	0
#define CVR_CAMERA_SECONDARY 1

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/

#ifdef ENABLE_MULTI_SENSOR 
//Set PIP mode
int gp_IPC_Set_PIPMode(int mode, IPC_Scale_mode_e type);
//alternative to gp_capture_stream_create() for taking pictures from sensor2 (pc-camera)
void * gp_capture_stream_create_2();
int gp_IPC_VStream2_Open();
int gp_IPC_VStream2_Close(void);
int gp_IPC_Vstream2_GetResolution(int* w, int* h);
int gp_IPC_Screen2_Skip(int skip_every_n_frame);
#endif

#endif
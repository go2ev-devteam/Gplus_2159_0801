#ifndef __OSD__RESOURCE__H__
#define __OSD__RESOURCE__H__

#include "list.h"


/********************************
register the resource file
***********************************/
void * reg_resource_file(char *filename);

/*********************************
register the resource from buf
**********************************/
void *reg_resource_buf(char *buf,int size);

/***************************************
free the resource 
***************************************/
void* free_resource(void *handle);

/**********************************************
add the osd resource to stream
************************************************/
int   add_osd(void *streamHandle,void *image, int width,int height);


/*******************************************
reg the resource to the VChn.
argument :1.CHandle:  Video chanel
		  2.resource handle
		  3.x,y the postion
		  
return :  index.	  
*********************************************/
int   gp_IPC_VChn_reg_resource(void *CHandle, void *resource_handle,int x,int y);	

/**********************************
unreg the resource
argument:
          1. CHanel: vidoe channel
          2. the return value of fucntion gp_IPC_VChn_reg_resource;
***********************************/
int gp_IPC_VChn_unreg_resource(void *CHandle,int index);

/**********************************
free the resource of Video channel
************************************/
int gp_IPC_VChn_unreg_all_resource(void *CHandle);	

/*********************************
enable the osd add to the video channel.
**********************************/
int gp_IPC_VChn_enable_osd(void *CHandle,int enable);
/***************************************
used by inter code
****************************************/
int add_osd(void *streamHandle,void *image, int width,int height);
#endif

#ifndef __GP__PNG__DECODE__H__
#define _GP__PNG__DECODE__H__
int  pngDecoder_buf(void *inputbuf,int input_size,int *width,int *height,char **outbuf);
int pngDecoder_filename(char* name, int *width,int *height,char **outbuf);

#endif 

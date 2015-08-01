/***************************************************************************
 * Name: audioEncoder.h
 *
 * Purpose:
 *
 * Developer:
 *     deyueli, 2014-1-24
 *
 ***************************************************************************/
#ifndef __AUDIOENCODER_H__
#define __AUDIOENCODER_H__
	 
/***************************************************************************
 * Header Files
 ***************************************************************************/
 #include "error_code.h"
/**
 * get instance size for audio encode.Call this function to get encode size.
 * @EncodeFormat[in]: encode format.
 * return: instance size. -1: error.
 */
SINT32 AudioEncoderInstanceSize( UINT32 EncodeFormat );

/**
 * init audio encode.
 * @pAEncInfo[in]: the piont of AudioEncInfo_t. Used pWorkMem, format, samplerate,
 * channel, bitrat.
 * return: GP_SUCCESS: init success. other value: error code.
 */
SINT32 AudioEncoderInit(audioEncInfo_t *pAEncInfo);

/**
 * audio encoder function.
 * @pAEncInfo[in]: the AudioEncInfo_t *.
 * @iBuf[in]: encoder input buffer data.
 * @oBuf: encoder output buffer
 * @outSize: to save encode size.
 * return: > 0: encoder ok. <=0: encode error.
 */

SINT32 AudioEncoderRun(audioEncInfo_t* pAEncInfo, bufferInfo_t *iBuf, bufferInfo_t*oBuf, int *outSize);

/**
 * audio encoder uninit.
 * @pAEncInfo[in]: the AudioEncInfo_t *.
 * return : GP_SUCCESS: GP_FAIL:
 */
SINT32 AudioEncoderUninit(audioEncInfo_t *pAEncInfo);

#endif  // __AUDIOENCODER_H__


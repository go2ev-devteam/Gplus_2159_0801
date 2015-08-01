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
 *                                                                        *
 **************************************************************************/
/**
 * @file error_code.h
 * @brief ipcamera error code header file
 * @author 
 */

#ifndef _ERROR_CODE_H_
#define _ERROR_CODE_H_

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#define GP_SUCCESS              (0)
#define GP_FAIL                 (-1)
#define GP_ERR_ILLEGAL_PARAM	(-2)    /*parameter illegal*/
#define GP_ERR_BUFFER_UNEXIST	(-3)    /*buffer unexist*/
#define GP_ERR_NULL_PTR 		(-4)	/*parameter NULL pointer*/
#define GP_ERR_NOT_PERM 		(-5)	/*setting not permit*/
#define GP_ERR_NOMEM			(-6)	/*alloc memory error*/
#define GP_ERR_NOTREADY 		(-7)	/*system not ready*/
#define GP_ERR_BUSY				(-8)    /*system busy.*/

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/

#endif //endif _ERROR_CODE_H_

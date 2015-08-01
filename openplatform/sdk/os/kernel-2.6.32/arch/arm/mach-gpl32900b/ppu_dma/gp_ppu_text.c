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
 *  3F, No.8, Dusing Rd., Hsinchu Science Park,                           *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
/**
 * @file    gp_ppu.c
 * @brief   Implement of PPU module driver.
 * @author  Cater Chen
 * @since   2010-10-27
 * @date    2010-10-27
 */
 
#include <linux/io.h>
#include <linux/module.h> 
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/hdreg.h> 		/* HDIO_GETGEO */
#include <linux/blkdev.h>
#include <mach/gp_ppu.h>
#include <mach/gp_chunkmem.h>
#if PPU_HARDWARE_MODULE == MODULE_ENABLE
#include <mach/hal/hal_ppu.h>
#endif

#if 1
	#define DEBUG	printk
#else
	#define DEBUG(...)
#endif

/**************************************************************************
 *                           C O N S T A N T S                            *
**************************************************************************/

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/

/**************************************************************************
 *                 E X T E R N A L    R E F E R E N C E S                 *
 **************************************************************************/

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
void gp_ppu_text_number_array_update_flag_clear(void);
/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static unsigned char text_charnum_update_flag[4];
 /**************************************************************************
 *             F U N C T I O N    I M P L E M E N T A T I O N S           *
 **************************************************************************/
/**
 * @brief 		PPU text register flag set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @return 	SUCCESS/ERROR_ID.
*/
static void 
gp_ppu_text_set_update_reg_flag(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index
)
{
	// Notify PPU driver to update text registers
	if (text_index == C_PPU_TEXT1) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT1;
	} else if (text_index == C_PPU_TEXT2) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT2;
	} else if (text_index == C_PPU_TEXT3) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT3;
	} else if (text_index == C_PPU_TEXT4) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT4;
	}
}

/**
 * @brief 		PPU text initial function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_init(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index
)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -ENOIOCTLCMD;
	}

	text_charnum_update_flag[text_index] = 1;

	if (text_index == C_PPU_TEXT3) {
		p_register_set->text3_25d_y_compress = 0x10;
	}
	gp_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

/**
 * @brief 		PPU text enable function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @param 	value [in]: value:0=disable 1=enable.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_enable_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index, 
unsigned int value
)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -ENOIOCTLCMD;
	}

	if (value) {
		p_register_set->text[text_index].control |= TXN_ENABLE;
	} else {
		p_register_set->text[text_index].control &= ~TXN_ENABLE;
	}
	
	p_register_set->ppu_enable &= ~HVCMP_DISABLE;
	p_register_set->ppu_enable &= ~TX_DIRECT_ADDRESS;
	p_register_set->text[text_index].control &= ~TXN_WALL_ENABLE;
	p_register_set->text[text_index].control &= ~TXN_MVE_ENABLE;
	p_register_set->text[text_index].control &= ~MASK_TXN_MODE;
	p_register_set->text[text_index].attribute &= ~MASK_TXN_HS;
	p_register_set->text[text_index].attribute &= ~MASK_TXN_VS;
	p_register_set->text[text_index].attribute &= ~MASK_TXN_DEPTH;
		
	p_register_set->text[text_index].control |= TXN_BMP;
	p_register_set->text[text_index].control |= TXN_REGMODE;
	p_register_set->text[text_index].segment = 0;
	p_register_set->text[text_index].position_x = 0;
	p_register_set->text[text_index].position_y = 0;

	// Notify PPU driver to update text registers
	gp_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

/**
 * @brief 		PPU text size set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @param 	value [in]: value:0(512x256) 1(512x512) ... 7(4096x4096).
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_size_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index, 
unsigned int value
)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || value>7) {
		return -ENOIOCTLCMD;
	}

	p_register_set->text[text_index].attribute &= ~MASK_TXN_SIZE;
	p_register_set->text[text_index].attribute |= (value<<B_TXN_SIZE) & MASK_TXN_SIZE;

	// Notify PPU driver to update text registers
	gp_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

/**
 * @brief 		PPU text color mode set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @param 	rgb_mode:[in]: rgb_mode:0:palette mode,1:high color mode.
* @param 	color:[in]: color=0:1555,1:565,2:RGBG,3:YUYV.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_color_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index, 
unsigned int rgb_mode, 
unsigned int color
)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || rgb_mode>1 || color>3) {
		return -ENOIOCTLCMD;
	}

	if (rgb_mode) {
		p_register_set->text[text_index].control |= TXN_RGB15P;
	} else {
		p_register_set->text[text_index].control &= ~TXN_RGB15P;
	}
	p_register_set->text[text_index].attribute &= ~MASK_TXN_COLOR;
	p_register_set->text[text_index].attribute |= (color<<B_TXN_COLOR) & MASK_TXN_COLOR;

	// Notify PPU driver to update text registers
	gp_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

/**
 * @brief 		PPU text position set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @param 	pos_x:[in]: pos_x:0~0xFFF.
* @param 	pos_y:[in]: pos_y:0~0xFFF.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_position_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index, 
unsigned int pos_x, 
unsigned int pos_y
)
{

	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -ENOIOCTLCMD;
	}

	p_register_set->text[text_index].position_x = (unsigned short)(pos_x & 0xFFF);
	p_register_set->text[text_index].position_y = (unsigned short)(pos_y & 0xFFF);

	// Notify PPU driver to update text registers
	gp_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

/**
* @brief 	PPU text number ram ptr set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @param 	value:[in]: value: 32-bit segment address.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_number_array_set_ptr(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index, 
unsigned int value
)
{
	unsigned int temp,temp1;
	
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -ENOIOCTLCMD;
	}
  
    temp = (unsigned int)gp_user_va_to_pa((unsigned short *)value);
    temp1 = (unsigned int)gp_chunk_va((unsigned int)temp);
    p_register_set->text[text_index].n_ptr = temp1;
    p_register_set->text[text_index].n_ptr_pa = temp;
  
	// Notify PPU driver to update text registers
	gp_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

/**
* @brief 		PPU text number ram ptr update flag clear set function.
* @return		None.
*/
void 
gp_ppu_text_number_array_update_flag_clear(
void
)
{
	text_charnum_update_flag[C_PPU_TEXT1] = 0;
	text_charnum_update_flag[C_PPU_TEXT2] = 0;
	text_charnum_update_flag[C_PPU_TEXT3] = 0;
	text_charnum_update_flag[C_PPU_TEXT4] = 0;
}

/**
 * @brief 		PPU text calculate number ram set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @param 	photo_width:[in]: photo width.
* @param 	photo_height:[in]: photo height.
* @param 	data_ptr:[in]: data_ptr: 32-bit segment address.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_calculate_number_array(
PPU_REGISTER_SETS *p_register_set, 
unsigned int text_index, 
unsigned int photo_width, 
unsigned int photo_height, 
unsigned int data_ptr
)
{
	unsigned short i, text_height;
	unsigned int  *p32,temp;
	unsigned int  number_value,bitmap_offset;
  
	temp= (unsigned int)gp_user_va_to_pa((unsigned short *)data_ptr);	
	p32 = (unsigned int *) p_register_set->text[text_index].n_ptr;
	if (!p32) {
		return -1;
	}
	
	// Normal bitmap mode
	bitmap_offset = photo_width;
	// YUYV or RGBG
	switch ((p_register_set->text[text_index].attribute & MASK_TXN_COLOR) >> B_TXN_COLOR) {
		case 0:	
			bitmap_offset <<= 1;		// Each pixel contains 2-byte data 
			break;
		case 1:		
			bitmap_offset <<= 1;		// Each pixel contains 2-byte data 
			break;
		case 2:		
		  if(p_register_set->ppu_misc & TEXT_RGBA_ENABLE)
			bitmap_offset <<= 2;		// RGBA8888 color mode
		  else
			bitmap_offset <<= 1;		// Each pixel contains 2-byte data  
			break;
		case 3:			// 8-bit color mode
		    bitmap_offset <<= 1;		// Each pixel contains 2-byte data  
			break;
	}			  			  

	number_value = temp;//data_ptr;
	text_height = photo_height;
	for (i=0; i<text_height; i++) {
		*p32 = number_value;
		p32++;
		number_value += bitmap_offset;
	}

	text_charnum_update_flag[text_index] = 1;

	return 0;
}

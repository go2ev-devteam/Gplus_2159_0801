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
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <mach/general.h>
#include <mach/kernel.h>
#include <mach/module.h>
#include <mach/cdev.h>
#include <linux/cdev.h>
#include <mach/diag.h>
#include <mach/gp_ppu.h>
#include <mach/gp_chunkmem.h>
#include <mach/hal/hal_clock.h>
#include <mach/gp_board.h>
#include <mach/clk/gp_clk_core.h>
#if PPU_HARDWARE_MODULE == MODULE_ENABLE
#include <mach/hal/hal_ppu.h>
#endif

/**************************************************************************
 *                           C O N S T A N T S                            *
**************************************************************************/

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#define DERROR	printk
#if 1
	#define DEBUG	printk
#else
	#define DEBUG(...)
#endif
#define VIC_PPU                  20

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
static unsigned int ppu_init_state = 0;

/**************************************************************************
 *                 E X T E R N A L    R E F E R E N C E S                 *
 **************************************************************************/

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
int gp_ppu_open(struct inode *inode, struct file *filp);
int gp_ppu_release(struct inode *inode, struct file *filp);
int gp_ppu_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static unsigned int gp_ppu_poll(struct file *filp, struct poll_table_struct *poll);
static irqreturn_t gp_ppu_irq_handler(int irq, void *dev_id);

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#define RETURN(x, msg)\
{\
	nRet = x;\
	DERROR(msg);\
	goto __return;\
}\
/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
struct file_operations ppu_fops = {
	.owner = THIS_MODULE,
	.poll = gp_ppu_poll,
	.open = gp_ppu_open,
	.ioctl = gp_ppu_ioctl,
	.release = gp_ppu_release,
};

typedef struct gp_ppu_dev_s {
	struct miscdevice c_dev;
	struct semaphore sem;           /*!< @brief mutex semaphore for scale ops */
	wait_queue_head_t ppu_wait_queue;
	bool done;
	int ppu_major;
	struct class *ppu_class;
} gp_ppu_dev_t;

static gp_ppu_dev_t *ppu_devices = NULL;
unsigned int PPU_GO_REENTRY = TRUE,PPU_FIFO_GO_REENTRY = TRUE;
static PPU_REGISTER_SETS *ppu_register_set;

 /**************************************************************************
 *             F U N C T I O N    I M P L E M E N T A T I O N S           *
 **************************************************************************/
//PPU Module Register
/**
 * @brief   PPU clock enable/disable
 * @param   enable [in] 0:disable, 1:enable
 * @return  None
 * @see
 */
static void ppu_clock_enable(int enable)
{
	DEBUG("[%s][%d]\n", __FUNCTION__, __LINE__);
	gp_enable_clock( (int*)"PPU_SPR", enable );			   
}

/**
 * @brief 	PPU module isr function.
* @return 	isr number.
 */
signed int 
gp_ppu_module_isr(
void
)
{
	signed int temp=0;

	temp = gpHalPPUIsr();

	return temp;
}

/**
 * @brief 		PPU driver initial function.
* @param 	p_register_set [in]: PPU struct value initiation.
* @return 	SUCCESS/ERROR_ID.
 */
signed int 
gp_ppu_init(
PPU_REGISTER_SETS *p_register_set
)
{
	int nRet = 0;

	gpHalPPUIrqCtrl(1);

	return nRet;
}

/**
 * @brief 		PPU module enable function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	value [in]: value:0=disable 1=enable.
* @return 	SUCCESS/ERROR_ID.
 */
signed int 
gp_ppu_set_enable(
PPU_REGISTER_SETS *p_register_set, 
unsigned int value
)
{
	if (!p_register_set) {
		return -ENOIOCTLCMD;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~PPU_ENABLE;
	}
	
	p_register_set->ppu_enable &= ~PPU_VGA;
	p_register_set->ppu_enable &= ~PPU_VGA_NONINTL;
	p_register_set->ppu_enable &= ~SAVE_ROM_ENABLE;
	p_register_set->ppu_enable &= ~MASK_TFT_SIZE;
	p_register_set->ppu_enable &= ~PPU_CM_ENABLE;
	p_register_set->ppu_enable &= ~PPU_DEFEN_ENABLE;
	//p_register_set->ppu_enable &= ~PPU_LONG_BURST;
	p_register_set->ppu_enable &= ~FB_SEL1;
	
	p_register_set->ppu_misc &= ~TXT_NEWCMP_ENABLE;
	p_register_set->ppu_misc &= ~TV_FLIP_ENABLE;
	p_register_set->ppu_misc &= ~TFT_FLIP_ENABLE;
	p_register_set->ppu_misc &= ~TXT_DELGO_ENABLE;
	p_register_set->ppu_misc &= ~TXT_TFTVTQ_ENABLE;
	p_register_set->ppu_misc &= ~TXT_TVLB_ENABLE;	

	p_register_set->ppu_enable |= TX_BOT2UP;
	p_register_set->ppu_enable |= PPU_FRAME_BASE;
	p_register_set->ppu_enable |= FB_SEL1;
	p_register_set->ppu_enable |= PPU_LONG_BURST;
	//p_register_set->ppu_enable |= PPU_VGA_NONINTL;
	
	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

/**
 * @brief 		PPU line or frame buffer color mode set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	format [in]: format:0=yuyv or rgbg mode disable 1= yuyv or rgbg mode enable.
* @param 	mono [in]: mono: format:0(mono:0=RGB565 1=Mono 2=4-color 3=16-color) 1(mono:0=RGBG 1=YUYV 2=RGBG 3=YUYV).
* @return 	SUCCESS/ERROR_ID.
 */
signed int 
gp_ppu_fb_format_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int format, 
unsigned int mono
)
{
	if (!p_register_set || mono>3) {
		return -ENOIOCTLCMD;
	}

	if (format) {
		p_register_set->ppu_enable |= PPU_RGBG;
	} else {
		p_register_set->ppu_enable &= ~PPU_RGBG;
	}
	p_register_set->ppu_enable &= ~MASK_FB_MONO;
	p_register_set->ppu_enable |= (mono<<B_FB_MONO) & MASK_FB_MONO;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

/**
 * @brief 		PPU test and sprite color type mode set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	value [in]: value: 0=BGRG/VYUY 1=GBGR/YVYU 2=RGBG/UYVY 3=GRGB/YUYV, value[2]:0=UV is unsigned(YCbCr) 1=UV is signed(YUV).
* @return 	SUCCESS/ERROR_ID.
 */
signed int 
gp_ppu_yuv_type_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int value
)
{
	if (!p_register_set || value>7) {
		return -ENOIOCTLCMD;
	}

	p_register_set->ppu_enable &= ~MASK_YUV_TYPE;
	p_register_set->ppu_enable |= (value<<B_YUV_TYPE) & MASK_YUV_TYPE;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

/**
 * @brief 		PPU rgb565 or yuyv and rgbg mode set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	enable [in]: enable:0=disable 1=enable.
* @param 	value [in]: value:0~0xFFFF.
* @return 	SUCCESS/ERROR_ID.
 */
signed int 
gp_ppu_rgb565_transparent_color_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int enable, 
unsigned int value
)
{
	if (!p_register_set || value>0xFFFFFF) {
		return -ENOIOCTLCMD;
	}
	if (enable) {
		p_register_set->ppu_rgb565_transparent_color = (1<<24) | value;
	} else {
		p_register_set->ppu_rgb565_transparent_color = value;
	}

	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

/**
 * @brief 		This function returns when PPU registers are updated and operation is done.
* @param 	p_register_set [in]: PPU struct value set.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_go(
PPU_REGISTER_SETS *p_register_set
)
{
    signed int result = 0;
	unsigned int temp;
	
    if(PPU_GO_REENTRY)
    {
	    ppu_devices->done = 0x00;
        PPU_GO_REENTRY = FALSE;
        #if PPU_HARDWARE_MODULE == MODULE_ENABLE
	       temp = (unsigned int)gp_user_va_to_pa((unsigned short *)p_register_set->buffer_user_define);
		   p_register_set->buffer_user_define = temp;
		   result = gpHalPPUSimpleGo(p_register_set);
	       if (result == 0) {
		       gp_ppu_text_number_array_update_flag_clear();
	       }
       #else
           result=-ENOIOCTLCMD;
       #endif	 
	   PPU_GO_REENTRY = TRUE; 
    }
    return result;
}

/**
 * @brief 	PPU free size set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	INTL [in]: INTL:0=TFT Display 1=TV Display.
* @param 	H_size [in]: H_size:16~1920.
* @param 	V_size [in]: V_size:1~1024.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_free_size_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned short INTL, 
unsigned short H_size, 
unsigned short V_size
)
{
	if (!p_register_set) {
		return -ENOIOCTLCMD;
	}
  if(INTL)
     p_register_set->ppu_free_mode = (((INTL << B_FREE_INIT) & MASK_FREE_INIT_SIZE)|((H_size << B_FREE_H_SIZE) & MASK_FREE_H_SIZE )|((V_size>>1) & MASK_FREE_V_SIZE));
  else
     p_register_set->ppu_free_mode = (((INTL << B_FREE_INIT) & MASK_FREE_INIT_SIZE)|((H_size << B_FREE_H_SIZE) & MASK_FREE_H_SIZE )|(V_size & MASK_FREE_V_SIZE));

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;	
}

/**
 * @brief 	PPU rgba color mode for text set function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	value [in]: value:0=disable 1=enable.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_text_rgba_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int value
)
{
	
	if (!p_register_set) {
		return -ENOIOCTLCMD;
	}

	if (value) {
		p_register_set->ppu_misc |= TEXT_RGBA_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TEXT_RGBA_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;	
}

/**
 * @brief 	PPU argb888 mode enable function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	value [in]: value:0=disable 1=enable.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_argb888_enable_set(
PPU_REGISTER_SETS *p_register_set, 
unsigned int value
)
{
	if (!p_register_set) {
		return -ENOIOCTLCMD;
	}

	if (value) {
		p_register_set->ppu_misc |= PPU_ARGB888_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~PPU_ARGB888_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

/**
 * @brief 	PPU irq function.
* @return 	SUCCESS/ERROR_ID.
*/
irqreturn_t 
gp_ppu_irq_handler(
	int irq, 
	void *dev_id
)
{     
		signed int ret=-1;

		#if PPU_HARDWARE_MODULE == MODULE_ENABLE
			ret = gpHalPPUIsr();
		#endif

		if(ret!=PPU_MODULE_PPU_VBLANK)
			return IRQ_NONE;            
                   
        if(!gpHalPPUInterlaceEnget())
        {
		    ppu_devices->done = 1;
		    wake_up_interruptible(&ppu_devices->ppu_wait_queue);
	    }
	      
	    return IRQ_HANDLED;	
}

/**
* @brief	ppu poll function
* @param	filp [in]:
* @param	poll [in]:
* @return	SUCCESS/ERROR_ID
*/
static unsigned int 
gp_ppu_poll(
	struct file *filp, 
	struct poll_table_struct *poll
)
{
	unsigned int mask = 0;

	/* wait_event_interruptible(p_cdsp_dev->cdsp_wait_queue, (p_cdsp_dev->done != 0)); */
	poll_wait(filp, &ppu_devices->ppu_wait_queue, poll);
	if(ppu_devices->done != 0)
	{
		ppu_devices->done = 0;
		mask = POLLIN | POLLRDNORM;
	}
	
	return mask;
}

/**
 * @brief 	PPU fb-lock set.
 * @param 	fb_lock [in]: fb-lock set type.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_fblock_set(
PPU_FBLOCK_STRUCT *fb_lock
)
{
	#if PPU_HARDWARE_MODULE == MODULE_ENABLE
		gpHalPPUFblock(fb_lock);
	#else
		return -1;
	#endif 
		
	return 0;
}

/**
* @brief 	PPU irq state get.
* @param 	state_name [in]: state get number.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_irq_state_get(
unsigned int state_name
)
{
	int state_temp;
		
	state_temp = -1;
	#if PPU_HARDWARE_MODULE == MODULE_ENABLE
		state_temp = gpHalPPUGetIrqStatus();
        if(state_temp & state_name)
        {
            state_temp = 1;
            gpHalPPUClearIrqFlag(state_name);
        }
        else
        {
            state_temp = 0;
	    }
    #endif 
		
	return state_temp;
}

/**
 * @brief display device release                                              
 */                                                                         
static void
ppu_device_release(
	struct device *dev
)
{
	DEBUG("remove ppu device ok\n");
}

static struct platform_device ppu_module_device = {
	.name	= "gp-ppu-module",
	.id		= 0,
	.dev	= {
		.release = ppu_device_release,
	},
};

#ifdef CONFIG_PM
static void gp_ppu_suspend_set( void ){
	gpHalPPUEn(0);
	ppu_clock_enable(0);
}

static void gp_ppu_resume_set( void ){
	ppu_clock_enable(1);
	gpHalPPUEn(1);
}

static int
ppu_suspend(
	struct platform_device *pdev,
	pm_message_t state
)
{
	if(ppu_init_state)
		 gp_ppu_suspend_set();	 
	
	return 0;
}

static int
ppu_resume(
	struct platform_device *pdev
)
{
	if(ppu_init_state)
		 gp_ppu_resume_set();
	
	return 0;
}
#else
#define ppu_suspend NULL
#define	ppu_resume NULL
#endif

/**                                                                         
 * @brief audio driver define                                               
 */                                                                         
static struct platform_driver ppu_module_driver = {
	.suspend = ppu_suspend,
	.resume = ppu_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "gp-ppu-easy-module"
	}
};

/**
* @brief 	PPU fb-lock set.
* @param 	fb_lock [in]: fb-lock set type.
* @return 	SUCCESS/ERROR_ID.
*/
signed int 
gp_ppu_enable_set(
void
)
{
	#if PPU_HARDWARE_MODULE == MODULE_ENABLE
		gpHalPPUEnset();
	#else
		return -1;
	#endif 
		
	return 0;
}

/**
 * \brief Open ppu device
 */
int
gp_ppu_open(
	struct inode *inode,
	struct file *filp
)
{
	int nRet=0;
	
	/* Success */

	if(ppu_init_state == 0)
	{
		ppu_clock_enable(1);
		gpHalPPUEn(1);			
	}
	ppu_init_state++;
  
  DEBUG(KERN_WARNING "PPU open \n");
	
	return nRet;
}
int
gp_ppu_release(
	struct inode *inode,
	struct file *filp
)
{
	ppu_init_state--;
	if(ppu_init_state == 0)
	{
			gpHalPPUEn(0);	
			ppu_clock_enable(0);
    }
	/* Success */
	DEBUG(KERN_WARNING "PPU release \n");
		
	return 0;
}

int
gp_ppu_ioctl(
	struct inode *inode,
	struct file *filp,
	unsigned int cmd,
	unsigned long arg
)
{
  int ret = 0;

  /* initial ppu register parameter set structure */
  ppu_register_set = (PPU_REGISTER_SETS *)arg;
  
  switch(cmd)
  {
      case PPUIO_SET_INIT:
           ret = gp_ppu_init(ppu_register_set);
           break;
           
      case PPUIO_SET_ENABLE:     
           ret = gp_ppu_set_enable(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode);
           break;
        
      case PPUIO_SET_FB_FORMAT:     
           ret = gp_ppu_fb_format_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_select_type_mode);
           break;
      
      case PPUIO_SET_YUV_TYPE:     
           ret = gp_ppu_yuv_type_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_select_type_mode);
           break; 
                      
      case PPUIO_SET_RGB565_TRANSPARENT_COLOR:     
           ret = gp_ppu_rgb565_transparent_color_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_select_type_mode);
           break;  
         
      case PPUIO_PPUGO:     
           ret = gp_ppu_go(ppu_register_set);
           break;            
           
      case PPUIO_SET_FREE_SIZE:     
           ret = gp_ppu_free_size_set(ppu_register_set, (unsigned short)ppu_register_set->ppu_set.PPU_select_type_mode, 
           (unsigned short)ppu_register_set->ppu_set.PPU_hsize_mode, (unsigned short)ppu_register_set->ppu_set.PPU_vsize_mode);
           break;
      
      case PPUIO_SET_TEXT_RGBA:     
           ret = gp_ppu_text_rgba_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode);
           break;           
           
      case PPUIO_SET_PPU_FBLOCK: 
		   ret = gp_ppu_fblock_set((PPU_FBLOCK_STRUCT *)&ppu_register_set->fblock_set);
           break;
           
      case PPUIO_SET_PPU_ENABLE:              
           ret = gp_ppu_enable_set();
           break;
           
      case PPUIO_SET_PPU_ILLEGAL_WRITE_STATE_GET:              
           ret = gp_ppu_irq_state_get(PPU_MODULE_ILLEGAL_WRITE);
           break;                        
        
      case PPUIO_SET_PPU_ARGB888_ENABLE:              
           ret = gp_ppu_argb888_enable_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode);
           break;                                 
                                                                                 	         	                        
      // Text Moudle Set
      case PPUIO_TEXT_SET_INIT:
           ret = gp_ppu_text_init(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode);
           break;
           
      case PPUIO_TEXT_SET_ENABLE:     
           ret = gp_ppu_text_enable_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_select_type_mode);
           break;
      
	  case PPUIO_TEXT_SET_TEXT_SIZE:     
           ret = gp_ppu_text_size_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_select_type_mode);
           break; 
		   
      case PPUIO_TEXT_SET_COLOR:     
           ret = gp_ppu_text_color_set(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_select_type_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_select_type_value_mode);
           break;             
      
	  case PPUIO_TEXT_SET_POSITION:     
           ret=gp_ppu_text_position_set(ppu_register_set, (unsigned int)(ppu_register_set->ppu_set.PPU_enable_mode), 
           (unsigned int)ppu_register_set->ppu_set.PPU_hsize_mode, (unsigned int)ppu_register_set->ppu_set.PPU_vsize_mode);
           break;  
		   
      case PPUIO_TEXT_SET_NUMBER_ARRAY_PTR:     
           ret = gp_ppu_text_number_array_set_ptr(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_buffer_ptr);
           break;  
 
      case PPUIO_TEXT_CALCULATE_NUMBER_ARRAY:     
           ret = gp_ppu_text_calculate_number_array(ppu_register_set, (unsigned int)ppu_register_set->ppu_set.PPU_enable_mode, 
           (unsigned int)ppu_register_set->ppu_set.PPU_hsize_mode, (unsigned int)ppu_register_set->ppu_set.PPU_vsize_mode, (unsigned int)ppu_register_set->ppu_set.PPU_buffer_ptr);
           break; 
                      
 		  default:
			     ret = -ENOIOCTLCMD;
			     break;                      
  }
	
	return ret;
}

void __exit
gp_ppu_module_exit(
	void
)
{
	misc_deregister(&ppu_devices->c_dev);
	free_irq(VIC_PPU, ppu_devices);
	kfree(ppu_devices);
	ppu_devices = NULL;

	platform_device_unregister(&ppu_module_device);
	platform_driver_unregister(&ppu_module_driver);

  DEBUG(KERN_WARNING "PPU module exit \n"); 
}

/**
 * \brief Initialize display device
 */
int __init
gp_ppu_module_init(
	void
)
{
	int nRet;

	DEBUG(KERN_WARNING "ModuleInit: PPU DMA\n");
	ppu_devices = (gp_ppu_dev_t *)kzalloc(sizeof(gp_ppu_dev_t), GFP_KERNEL);
	if(ppu_devices == NULL)
		RETURN(-ENOMEM, "PPU kmalloc fail\n");

	/* reguest irq */
	nRet = request_irq(VIC_PPU,
					  gp_ppu_irq_handler,
					  IRQF_SHARED,
					  "PPU_IRQ",
					  (void *)ppu_devices);
	if(nRet < 0)
		RETURN(-ENXIO, "PPU request irq fail\n");

	/* initialize */
	init_MUTEX(&ppu_devices->sem);
	init_waitqueue_head(&ppu_devices->ppu_wait_queue);

	ppu_devices->c_dev.name  = "ppu0";
	ppu_devices->c_dev.minor = MISC_DYNAMIC_MINOR;
	ppu_devices->c_dev.fops  = &ppu_fops;

	/* register device */
	nRet = misc_register(&ppu_devices->c_dev);
	if(nRet != 0)
		RETURN(-ENXIO, "PPU device register fail\n");

	nRet = 0;
	platform_device_register(&ppu_module_device);
	platform_driver_register(&ppu_module_driver);

__return:
	if(nRet < 0)
	{
			DERROR(KERN_WARNING "PPUInitFail\n");
			free_irq(VIC_PPU, ppu_devices);
			kfree(ppu_devices);
			ppu_devices = NULL;
	}
	return nRet;
}

module_init(gp_ppu_module_init);
module_exit(gp_ppu_module_exit);

/**************************************************************************
 *                  M O D U L E    D E C L A R A T I O N                  *
 **************************************************************************/

MODULE_AUTHOR("Generalplus");
MODULE_DESCRIPTION("Generalplus PPU Driver");
MODULE_LICENSE_GP;
	

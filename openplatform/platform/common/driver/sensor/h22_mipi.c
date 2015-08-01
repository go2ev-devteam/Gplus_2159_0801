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
 
/**************************************************************************
 *                         H E A D E R   F I L E S						  *
 **************************************************************************/
#include <linux/module.h>
#include <linux/fs.h> /* everything... */
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <linux/delay.h>
#include <mach/gp_gpio.h>
#include <mach/gp_i2c_bus.h>
#include <mach/gp_mipi.h>
#include <mach/sensor_mgr.h>

#if (defined CONFIG_ARCH_GPL32900B)
#include <mach/gp_ti2c_bus.h>
#endif

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define SOI_H22_ID		0x60

#define TI2C_RETRY		5

// I2C mode
#define	GPIO_I2C		0x00 
#define HW_I2C			0x01
#define HW_TI2C			0x02
#if (defined CONFIG_ARCH_GPL32900)
#define I2C_MODE		HW_I2C
#elif (defined CONFIG_ARCH_GPL32900B)
#define I2C_MODE		HW_TI2C
#else
#define I2C_MODE		GPIO_I2C
#endif


#define MAX_ANALOG_GAIN					2.5

#define H22_MAX_EXPOSURE_TIME			(256*3)//0x30E//782
#define H22_MIN_EXPOSURE_TIME			0x08
#define H22_MAX_ANALOG_GAIN				(MAX_ANALOG_GAIN*256) //(3*256)	//4x
#define H22_MIN_ANALOG_GAIN				(1*256)
#define H22_MAX_DIGITAL_GAIN			(1.5*256)//0x3=4x
#define H22_MIN_DIGITAL_GAIN 			(1*256)//0:1x

#define H22_50HZ_EXP_TIME_TOTAL			212
#define	H22_50HZ_INIT_EV_IDX			140
#define H22_50HZ_NIGHT_EV_IDX			153
#define H22_50HZ_DAY_EV_IDX				100
#define H22_50HZ_MAX_EXP_IDX			(H22_50HZ_EXP_TIME_TOTAL-24)  // <= H22_50HZ_EXP_TIME_TOTAL

#define H22_60HZ_EXP_TIME_TOTAL			214
#define	H22_60HZ_INIT_EV_IDX			140
#define H22_60HZ_NIGHT_EV_IDX			153
#define H22_60HZ_DAY_EV_IDX				51
#define H22_60HZ_MAX_EXP_IDX			(H22_60HZ_EXP_TIME_TOTAL-22)  // <= H22_60HZ_EXP_TIME_TOTAL

#define OV2710_30FPS_50HZ_EXP_TIME_TOTAL		(258-13)

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct regval_list_s 
{
	unsigned char reg_num;
	unsigned char value;
} regval8_t;

typedef struct sensor_dev_s 
{
	struct v4l2_subdev sd;
	sensor_fmt_t *fmt;	/* Current format */
} sensor_dev_t;

/**************************************************************************
 *                 E X T E R N A L    R E F E R E N C E S                 *
 **************************************************************************/

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
static void h22_set_exp_freq(int freq);

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static int current_frame_rate;

static int *p_expTime_table;
static int sensor_fps, sensor_total_ev, sensor_swith_time;

static sensor_dev_t			gH22Dev;
static sensor_exposure_t 	h22_seInfo;
#if (I2C_MODE == HW_I2C)
static int 					g_i2c_handle;
#elif (I2C_MODE == HW_TI2C)
static ti2c_set_value_t 	g_ti2c_handle;
#endif

static char *param[] = {"0", "PORT0", "0", "NONE", "0", "NONE"};
static int nstrs = 6;
module_param_array(param, charp, &nstrs, S_IRUGO);

#if 0
static const unsigned short h22_r_b_gain[71][2] = 
{
	{24, 141}, // 2
	{28, 140},
	{32, 139},
	{36, 138},
	{39, 137},
	{42, 136},
	{46, 135},
	{49, 134},
	{51, 133},
	{54, 132}, // 3
	{56, 131},
	{59, 130},
	{61, 129},
	{63, 127},
	{65, 126},
	{67, 125}, // 3.6
	{68, 124},
	{70, 123},
	{71, 122},
	{72, 120}, // 4
	{74, 119},
	{75, 118},
	{76, 117},
	{77, 115},
	{77, 114},
	{78, 113},
	{79, 111},
	{79, 110},
	{80, 109},
	{80, 107}, // 5
	{81, 106},
	{81, 105},
	{81, 103},
	{82, 102},
	{82, 100},// 5.5
	{82, 99}, 
	{82, 98}, 
	{83, 96}, 
	{83, 95}, 
	{83, 93}, // 6
	{83, 92}, 
	{83, 90}, 
	{84, 88}, 
	{84, 87}, 
	{84, 85},  // 6.5
	{84, 84}, 
	{85, 82}, 
	{85, 81}, 
	{85, 79}, 
	{86, 77}, 
	{86, 76}, 
	{87, 74}, 
	{87, 72}, 
	{88, 71}, 
	{89, 69}, 
	{90, 67}, 
	{91, 65}, 
	{92, 64}, 
	{93, 62}, 
	{94, 60}, 
	{95, 58}, 
	{97, 56}, 
	{98, 55}, 
	{100, 53},
	{102, 51},
	{104, 49},
	{106, 47},
	{108, 45},
	{111, 43},
	{113, 41},
	{116, 39} 
};

static const unsigned int h22_gamma_045_table[] =
{
	0x04510d, 0x051112, 0x111417, 0x14451b, 0x111120, 0x044425, 0x111129, 0x05112d, 
	0x044432, 0x044436, 0x04443a, 0x04443e, 0x044442, 0x110446, 0x111149, 0x04444d, 
	0x111051, 0x044154, 0x111058, 0x10445b, 0x04415e, 0x011062, 0x011065, 0x010468, 
	0x01046b, 0x01046e, 0x041071, 0x041074, 0x104176, 0x010479, 0x04107c, 0x00417e, 
	0x041081, 0x010183, 0x101086, 0x040488, 0x01018a, 0x10408d, 0x10108f, 0x041091, 
	0x040493, 0x040495, 0x040497, 0x040499, 0x10109b, 0x00109d, 0x00409f, 0x0101a0, 
	0x1004a2, 0x0010a4, 0x0100a6, 0x1004a7, 0x0040a9, 0x1001aa, 0x0040ac, 0x1001ad, 
	0x0100af, 0x0004b0, 0x0400b2, 0x0040b3, 0x0004b4, 0x0400b6, 0x0100b7, 0x0010b8, 
	0x0004b9, 0x1000bb, 0x0400bc, 0x0100bd, 0x0040be, 0x0010bf, 0x0004c0, 0x0004c1, 
	0x0001c2, 0x0001c3, 0x0000c5, 0x0000c6, 0x0000c7, 0x0000c8, 0x0000c9, 0x0000ca, 
	0x0000cb, 0x0000cc, 0x0000cd, 0x0000ce, 0x0000cf, 0x0000d0, 0x0001d0, 0x0001d1, 
	0x0001d2, 0x0001d3, 0x0001d4, 0x0001d5, 0x0001d6, 0x0001d7, 0x0001d8, 0x0001d9, 
	0x0000db, 0x0000dc, 0x1000dd, 0x0400de, 0x0400df, 0x0100e0, 0x0040e1, 0x0010e2, 
	0x0004e3, 0x1000e5, 0x0100e6, 0x0040e7, 0x0004e8, 0x1000ea, 0x0100eb, 0x0004ec, 
	0x0400ee, 0x0040ef, 0x1001f0, 0x0040f2, 0x1004f3, 0x0040f5, 0x0401f6, 0x0010f8, 
	0x0100fa, 0x0404fb, 0x1010fd, 0x0000ff, 0x0000ff, 0x0000ff, 0x0000ff, 0x0000ff
};

static const short h22_color_matrix4gamma045[9] = 
{	
	(short) (1.289461127029244000*64),
	(short) (-0.15633132671308050*64),
	(short) (-0.13312980031616348*64),
	(short) (-0.28924304019577968*64),
	(short) (1.448004578314210100*64),
	(short) (-0.15876153811843055*64),
	(short) (-0.02364080679490283*64),
	(short) (-1.03955844263753080*64),
	(short) (2.063199249432433800*64)
};

static const short h22_awb_thr[19] = 
{
	200, // awbwinthr
	
	0*64, // sindata
	1*64, // cosdata 
	
	 30, // Ythr0
	 90, // Ythr1
	150, // Ythr2 
	200, // Ythr3 
	
	-6, // UL1N1 
	 5, // UL1P1 
	-6, // VL1N1 
	 5, // VL1P1 
	
	-6, // UL1N2
	 6, // UL1P2
	-6, // VL1N2
	 6, // VL1P2
	
	-7, // UL1N3
	 7, // UL1P3
	-7, // VL1N3
	 7, // VL1P3
};

static const sensor_calibration_t h22_cdsp_calibration =
{
	NULL,						//ob
	NULL, 						//lenscmp
	h22_r_b_gain,				//wb_gain
	h22_gamma_045_table,		//gamma1
	h22_color_matrix4gamma045,	//color_matrix1
	h22_gamma_045_table,		//gamma2
	h22_color_matrix4gamma045,	//color_matrix2
	h22_awb_thr					//awb_thr	
};
#endif

static const int h22_exp_time_gain_60Hz[H22_60HZ_EXP_TIME_TOTAL][3] = 
{ // {time, analog gain, digital gain}
{8,	    (int)(1.00*256), (int)(1.00*256)},
{9,	    (int)(1.00*256), (int)(1.00*256)},
{9,	    (int)(1.00*256), (int)(1.00*256)},
{9,	    (int)(1.00*256), (int)(1.00*256)},
{10,	(int)(1.00*256), (int)(1.00*256)},
{10,	(int)(1.00*256), (int)(1.00*256)},
{10,	(int)(1.00*256), (int)(1.00*256)},
{11,	(int)(1.00*256), (int)(1.00*256)},
{11,	(int)(1.00*256), (int)(1.00*256)},
{12,	(int)(1.00*256), (int)(1.00*256)},
{12,	(int)(1.00*256), (int)(1.00*256)},
{12,	(int)(1.00*256), (int)(1.00*256)},
{13,	(int)(1.00*256), (int)(1.00*256)},
{13,	(int)(1.00*256), (int)(1.00*256)},
{14,	(int)(1.00*256), (int)(1.00*256)},
{14,	(int)(1.00*256), (int)(1.00*256)},
{15,	(int)(1.00*256), (int)(1.00*256)},
{15,	(int)(1.00*256), (int)(1.00*256)},
{16,	(int)(1.00*256), (int)(1.00*256)},
{16,	(int)(1.00*256), (int)(1.00*256)},
{17,	(int)(1.00*256), (int)(1.00*256)},
{18,	(int)(1.00*256), (int)(1.00*256)},
{18,	(int)(1.00*256), (int)(1.00*256)},
{19,	(int)(1.00*256), (int)(1.00*256)},
{19,	(int)(1.00*256), (int)(1.00*256)},
{20,	(int)(1.00*256), (int)(1.00*256)},
{21,	(int)(1.00*256), (int)(1.00*256)},
{22,	(int)(1.00*256), (int)(1.00*256)},
{22,	(int)(1.00*256), (int)(1.00*256)},
{23,	(int)(1.00*256), (int)(1.00*256)},
{24,	(int)(1.00*256), (int)(1.00*256)},
{25,	(int)(1.00*256), (int)(1.00*256)},
{26,	(int)(1.00*256), (int)(1.00*256)},
{27,	(int)(1.00*256), (int)(1.00*256)},
{28,	(int)(1.00*256), (int)(1.00*256)},
{29,	(int)(1.00*256), (int)(1.00*256)},
{30,	(int)(1.00*256), (int)(1.00*256)},
{31,	(int)(1.00*256), (int)(1.00*256)},
{32,	(int)(1.00*256), (int)(1.00*256)},
{33,	(int)(1.00*256), (int)(1.00*256)},
{34,	(int)(1.00*256), (int)(1.00*256)},
{35,	(int)(1.00*256), (int)(1.00*256)},
{36,	(int)(1.00*256), (int)(1.00*256)},
{38,	(int)(1.00*256), (int)(1.00*256)},
{39,	(int)(1.00*256), (int)(1.00*256)},
{40,	(int)(1.00*256), (int)(1.00*256)},
{42,	(int)(1.00*256), (int)(1.00*256)},
{43,	(int)(1.00*256), (int)(1.00*256)},
{45,	(int)(1.00*256), (int)(1.00*256)},
{46,	(int)(1.00*256), (int)(1.00*256)},
{48,	(int)(1.00*256), (int)(1.00*256)},
{50,	(int)(1.00*256), (int)(1.00*256)},
{51,	(int)(1.00*256), (int)(1.00*256)},
{53,	(int)(1.00*256), (int)(1.00*256)},
{55,	(int)(1.00*256), (int)(1.00*256)},
{57,	(int)(1.00*256), (int)(1.00*256)},
{59,	(int)(1.00*256), (int)(1.00*256)},
{61,	(int)(1.00*256), (int)(1.00*256)},
{63,	(int)(1.00*256), (int)(1.00*256)},
{66,	(int)(1.00*256), (int)(1.00*256)},
{68,	(int)(1.00*256), (int)(1.00*256)},
{70,	(int)(1.00*256), (int)(1.00*256)},
{73,	(int)(1.00*256), (int)(1.00*256)},
{75,	(int)(1.00*256), (int)(1.00*256)},
{78,	(int)(1.00*256), (int)(1.00*256)},
{81,	(int)(1.00*256), (int)(1.00*256)},
{84,	(int)(1.00*256), (int)(1.00*256)},
{87,	(int)(1.00*256), (int)(1.00*256)},
{90,	(int)(1.00*256), (int)(1.00*256)},
{93,	(int)(1.00*256), (int)(1.00*256)},
{96,	(int)(1.00*256), (int)(1.00*256)},
{99,	(int)(1.00*256), (int)(1.00*256)},
{103,	(int)(1.00*256), (int)(1.00*256)},
{107,	(int)(1.00*256), (int)(1.00*256)},
{110,	(int)(1.00*256), (int)(1.00*256)},
{114,	(int)(1.00*256), (int)(1.00*256)},
{118,	(int)(1.00*256), (int)(1.00*256)},
{122,	(int)(1.00*256), (int)(1.00*256)},
{127,	(int)(1.00*256), (int)(1.00*256)},
{131,	(int)(1.00*256), (int)(1.00*256)},
{136,	(int)(1.00*256), (int)(1.00*256)},
{141,	(int)(1.00*256), (int)(1.00*256)},
{145,	(int)(1.00*256), (int)(1.00*256)},
{151,	(int)(1.00*256), (int)(1.00*256)},
{156,	(int)(1.00*256), (int)(1.00*256)},
{161,	(int)(1.00*256), (int)(1.00*256)},
{167,	(int)(1.00*256), (int)(1.00*256)},
{173,	(int)(1.00*256), (int)(1.00*256)},
{179,	(int)(1.00*256), (int)(1.00*256)},
{185,	(int)(1.00*256), (int)(1.00*256)},
{192,	(int)(1.00*256), (int)(1.00*256)},
{199,	(int)(1.00*256), (int)(1.00*256)},
{206,	(int)(1.00*256), (int)(1.00*256)},
{213,	(int)(1.00*256), (int)(1.00*256)},
{213,	(int)(1.00*256), (int)(1.04*256)},
{213,	(int)(1.06*256), (int)(1.00*256)},
{213,	(int)(1.13*256), (int)(1.00*256)},
{213,	(int)(1.13*256), (int)(1.00*256)},
{213,	(int)(1.19*256), (int)(1.00*256)},
{213,	(int)(1.25*256), (int)(1.00*256)},
{213,	(int)(1.25*256), (int)(1.00*256)},
{213,	(int)(1.31*256), (int)(1.00*256)},
{213,	(int)(1.38*256), (int)(1.00*256)},
{213,	(int)(1.44*256), (int)(1.00*256)},
{213,	(int)(1.44*256), (int)(1.00*256)},
{213,	(int)(1.50*256), (int)(1.00*256)},
{213,	(int)(1.56*256), (int)(1.00*256)},
{213,	(int)(1.63*256), (int)(1.00*256)},
{213,	(int)(1.69*256), (int)(1.00*256)},
{213,	(int)(1.75*256), (int)(1.00*256)},
{213,	(int)(1.81*256), (int)(1.00*256)},
{213,	(int)(1.88*256), (int)(1.00*256)},
{213,	(int)(1.94*256), (int)(1.00*256)},
{426,	(int)(1.00*256), (int)(1.00*256)},
{426,	(int)(1.00*256), (int)(1.04*256)},
{426,	(int)(1.06*256), (int)(1.00*256)},
{426,	(int)(1.13*256), (int)(1.00*256)},
{426,	(int)(1.13*256), (int)(1.00*256)},
{426,	(int)(1.19*256), (int)(1.00*256)},
{426,	(int)(1.25*256), (int)(1.00*256)},
{426,	(int)(1.25*256), (int)(1.00*256)},
{426,	(int)(1.31*256), (int)(1.00*256)},
{426,	(int)(1.38*256), (int)(1.00*256)},
{426,	(int)(1.44*256), (int)(1.00*256)},
{426,	(int)(1.44*256), (int)(1.00*256)},
{426,	(int)(1.50*256), (int)(1.00*256)},
{426,	(int)(1.56*256), (int)(1.00*256)},
{426,	(int)(1.63*256), (int)(1.00*256)},
{426,	(int)(1.69*256), (int)(1.00*256)},
{426,	(int)(1.75*256), (int)(1.00*256)},
{426,	(int)(1.81*256), (int)(1.00*256)},
{426,	(int)(1.88*256), (int)(1.00*256)},
{426,	(int)(1.94*256), (int)(1.00*256)},
{639,	(int)(1.31*256), (int)(1.01*256)},
{639,	(int)(1.38*256), (int)(1.00*256)},
{639,	(int)(1.44*256), (int)(1.00*256)},
{639,	(int)(1.44*256), (int)(1.00*256)},
{639,	(int)(1.50*256), (int)(1.00*256)},
{639,	(int)(1.56*256), (int)(1.00*256)},
{639,	(int)(1.63*256), (int)(1.00*256)},
{639,	(int)(1.69*256), (int)(1.00*256)},
{639,	(int)(1.75*256), (int)(1.00*256)},
{639,	(int)(1.81*256), (int)(1.00*256)},
{639,	(int)(1.88*256), (int)(1.00*256)},
{639,	(int)(1.94*256), (int)(1.00*256)},
{852,	(int)(1.50*256), (int)(1.00*256)},
{852,	(int)(1.56*256), (int)(1.00*256)},
{852,	(int)(1.63*256), (int)(1.00*256)},
{852,	(int)(1.69*256), (int)(1.00*256)},
{852,	(int)(1.75*256), (int)(1.00*256)},
{852,	(int)(1.81*256), (int)(1.00*256)},
{852,	(int)(1.88*256), (int)(1.00*256)},
{852,	(int)(1.94*256), (int)(1.00*256)},
{852,	(int)(2.00*256), (int)(1.00*256)},
{852,	(int)(2.00*256), (int)(1.04*256)},
{852,	(int)(2.13*256), (int)(1.00*256)},
{852,	(int)(2.25*256), (int)(1.00*256)},
{852,	(int)(2.25*256), (int)(1.00*256)},
{852,	(int)(2.38*256), (int)(1.00*256)},
{852,	(int)(2.50*256), (int)(1.00*256)},
{852,	(int)(2.50*256), (int)(1.00*256)},
{852,	(int)(2.63*256), (int)(1.00*256)},
{852,	(int)(2.75*256), (int)(1.00*256)},
{852,	(int)(2.88*256), (int)(1.00*256)},
{852,	(int)(2.88*256), (int)(1.00*256)},
{852,	(int)(3.00*256), (int)(1.00*256)},
{852,	(int)(3.13*256), (int)(1.00*256)},
{852,	(int)(3.25*256), (int)(1.00*256)},
{852,	(int)(3.38*256), (int)(1.00*256)},
{852,	(int)(3.50*256), (int)(1.00*256)},
{852,	(int)(3.63*256), (int)(1.00*256)},
{852,	(int)(3.75*256), (int)(1.00*256)},
{852,	(int)(3.88*256), (int)(1.00*256)},
{852,	(int)(4.00*256), (int)(1.00*256)},
{852,	(int)(4.00*256), (int)(1.04*256)},
{852,	(int)(4.25*256), (int)(1.00*256)},
{852,	(int)(4.25*256), (int)(1.04*256)},
{852,	(int)(4.50*256), (int)(1.00*256)},
{852,	(int)(4.75*256), (int)(1.00*256)},
{852,	(int)(5.00*256), (int)(1.00*256)},
{852,	(int)(5.00*256), (int)(1.04*256)},
{852,	(int)(5.25*256), (int)(1.00*256)},
{852,	(int)(5.50*256), (int)(1.00*256)},
{852,	(int)(5.50*256), (int)(1.04*256)},
{852,	(int)(5.75*256), (int)(1.02*256)},
{852,	(int)(6.00*256), (int)(1.00*256)},
{852,	(int)(6.25*256), (int)(1.00*256)},
{852,	(int)(6.50*256), (int)(1.00*256)},
{852,	(int)(6.75*256), (int)(1.00*256)},
{852,	(int)(7.00*256), (int)(1.00*256)},
{852,	(int)(7.00*256), (int)(1.04*256)},
{852,	(int)(7.00*256), (int)(1.07*256)},
{852,	(int)(7.00*256), (int)(1.11*256)},
{852,	(int)(7.00*256), (int)(1.15*256)},
{852,	(int)(7.00*256), (int)(1.19*256)},
{852,	(int)(7.00*256), (int)(1.23*256)},
{852,	(int)(7.00*256), (int)(1.27*256)},
{852,	(int)(7.00*256), (int)(1.32*256)},
{852,	(int)(7.00*256), (int)(1.37*256)},
{852,	(int)(7.00*256), (int)(1.41*256)},
{852,	(int)(7.00*256), (int)(1.46*256)},
{852,	(int)(7.00*256), (int)(1.52*256)},
{852,	(int)(7.00*256), (int)(1.57*256)},
{852,	(int)(7.00*256), (int)(1.62*256)},
{852,	(int)(7.00*256), (int)(1.68*256)},
{852,	(int)(7.00*256), (int)(1.74*256)},
{852,	(int)(7.00*256), (int)(1.80*256)},
{852,	(int)(7.00*256), (int)(1.87*256)},
{852,	(int)(7.00*256), (int)(1.93*256)},
{852,	(int)(7.00*256), (int)(2.00*256)},
{852,	(int)(7.25*256), (int)(2.00*256)},
{852,	(int)(7.50*256), (int)(2.00*256)},
{852,	(int)(7.75*256), (int)(2.00*256)},
{852,	(int)(8.00*256), (int)(2.00*256)}
};

static const  int h22_exp_time_gain_50Hz[H22_50HZ_EXP_TIME_TOTAL][3] = 
{ // {time, analog gain, digital gain}
{8, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
{12, (int)(1.00*256), (int)(1.00*256)},
{12, (int)(1.00*256), (int)(1.00*256)},
{13, (int)(1.00*256), (int)(1.00*256)},
{13, (int)(1.00*256), (int)(1.00*256)},
{13, (int)(1.00*256), (int)(1.00*256)},
{14, (int)(1.00*256), (int)(1.00*256)},
{14, (int)(1.00*256), (int)(1.00*256)},
{15, (int)(1.00*256), (int)(1.00*256)},
{15, (int)(1.00*256), (int)(1.00*256)},
{16, (int)(1.00*256), (int)(1.00*256)},
{17, (int)(1.00*256), (int)(1.00*256)},
{17, (int)(1.00*256), (int)(1.00*256)},
{18, (int)(1.00*256), (int)(1.00*256)},
{18, (int)(1.00*256), (int)(1.00*256)},
{19, (int)(1.00*256), (int)(1.00*256)},
{20, (int)(1.00*256), (int)(1.00*256)},
{20, (int)(1.00*256), (int)(1.00*256)},
{21, (int)(1.00*256), (int)(1.00*256)},
{22, (int)(1.00*256), (int)(1.00*256)},
{23, (int)(1.00*256), (int)(1.00*256)},
{23, (int)(1.00*256), (int)(1.00*256)},
{24, (int)(1.00*256), (int)(1.00*256)},
{25, (int)(1.00*256), (int)(1.00*256)},
{26, (int)(1.00*256), (int)(1.00*256)},
{27, (int)(1.00*256), (int)(1.00*256)},
{28, (int)(1.00*256), (int)(1.00*256)},
{29, (int)(1.00*256), (int)(1.00*256)},
{30, (int)(1.00*256), (int)(1.00*256)},
{31, (int)(1.00*256), (int)(1.00*256)},
{32, (int)(1.00*256), (int)(1.00*256)},
{33, (int)(1.00*256), (int)(1.00*256)},
{34, (int)(1.00*256), (int)(1.00*256)},
{36, (int)(1.00*256), (int)(1.00*256)},
{37, (int)(1.00*256), (int)(1.00*256)},
{38, (int)(1.00*256), (int)(1.00*256)},
{39, (int)(1.00*256), (int)(1.00*256)},
{41, (int)(1.00*256), (int)(1.00*256)},
{42, (int)(1.00*256), (int)(1.00*256)},
{44, (int)(1.00*256), (int)(1.00*256)},
{45, (int)(1.00*256), (int)(1.00*256)},
{47, (int)(1.00*256), (int)(1.00*256)},
{49, (int)(1.00*256), (int)(1.00*256)},
{50, (int)(1.00*256), (int)(1.00*256)},
{52, (int)(1.00*256), (int)(1.00*256)},
{54, (int)(1.00*256), (int)(1.00*256)},
{56, (int)(1.00*256), (int)(1.00*256)},
{58, (int)(1.00*256), (int)(1.00*256)},
{60, (int)(1.00*256), (int)(1.00*256)},
{62, (int)(1.00*256), (int)(1.00*256)},
{64, (int)(1.00*256), (int)(1.00*256)},
{66, (int)(1.00*256), (int)(1.00*256)},
{69, (int)(1.00*256), (int)(1.00*256)},
{71, (int)(1.00*256), (int)(1.00*256)},
{74, (int)(1.00*256), (int)(1.00*256)},
{76, (int)(1.00*256), (int)(1.00*256)},
{79, (int)(1.00*256), (int)(1.00*256)},
{82, (int)(1.00*256), (int)(1.00*256)},
{84, (int)(1.00*256), (int)(1.00*256)},
{87, (int)(1.00*256), (int)(1.00*256)},
{91, (int)(1.00*256), (int)(1.00*256)},
{94, (int)(1.00*256), (int)(1.00*256)},
{97, (int)(1.00*256), (int)(1.00*256)},
{100, (int)(1.00*256), (int)(1.00*256)},
{104, (int)(1.00*256), (int)(1.00*256)},
{108, (int)(1.00*256), (int)(1.00*256)},
{111, (int)(1.00*256), (int)(1.00*256)},
{115, (int)(1.00*256), (int)(1.00*256)},
{119, (int)(1.00*256), (int)(1.00*256)},
{124, (int)(1.00*256), (int)(1.00*256)},
{128, (int)(1.00*256), (int)(1.00*256)},
{133, (int)(1.00*256), (int)(1.00*256)},
{137, (int)(1.00*256), (int)(1.00*256)},
{142, (int)(1.00*256), (int)(1.00*256)},
{147, (int)(1.00*256), (int)(1.00*256)},
{152, (int)(1.00*256), (int)(1.00*256)},
{158, (int)(1.00*256), (int)(1.00*256)},
{163, (int)(1.00*256), (int)(1.00*256)},
{169, (int)(1.00*256), (int)(1.00*256)},
{175, (int)(1.00*256), (int)(1.00*256)},
{181, (int)(1.00*256), (int)(1.00*256)},
{187, (int)(1.00*256), (int)(1.00*256)},
{194, (int)(1.00*256), (int)(1.00*256)},
{201, (int)(1.00*256), (int)(1.00*256)},
{208, (int)(1.00*256), (int)(1.00*256)},
{215, (int)(1.00*256), (int)(1.00*256)},
{223, (int)(1.00*256), (int)(1.00*256)},
{231, (int)(1.00*256), (int)(1.00*256)},
{239, (int)(1.00*256), (int)(1.00*256)},
{247, (int)(1.00*256), (int)(1.00*256)},
{256, (int)(1.00*256), (int)(1.00*256)},
{256, (int)(1.00*256), (int)(1.04*256)},
{256, (int)(1.06*256), (int)(1.00*256)},
{256, (int)(1.13*256), (int)(1.00*256)},
{256, (int)(1.13*256), (int)(1.00*256)},
{256, (int)(1.19*256), (int)(1.00*256)},
{256, (int)(1.25*256), (int)(1.00*256)},
{256, (int)(1.25*256), (int)(1.00*256)},
{256, (int)(1.31*256), (int)(1.00*256)},
{256, (int)(1.38*256), (int)(1.00*256)},
{256, (int)(1.44*256), (int)(1.00*256)},
{256, (int)(1.44*256), (int)(1.00*256)},
{256, (int)(1.50*256), (int)(1.00*256)},
{256, (int)(1.56*256), (int)(1.00*256)},
{256, (int)(1.63*256), (int)(1.00*256)},
{256, (int)(1.69*256), (int)(1.00*256)},
{256, (int)(1.75*256), (int)(1.00*256)},
{256, (int)(1.81*256), (int)(1.00*256)},
{256, (int)(1.88*256), (int)(1.00*256)},
{256, (int)(1.94*256), (int)(1.00*256)},
{512, (int)(1.00*256), (int)(1.00*256)},
{512, (int)(1.00*256), (int)(1.04*256)},
{512, (int)(1.06*256), (int)(1.00*256)},
{512, (int)(1.13*256), (int)(1.00*256)},
{512, (int)(1.13*256), (int)(1.00*256)},
{512, (int)(1.19*256), (int)(1.00*256)},
{512, (int)(1.25*256), (int)(1.00*256)},
{512, (int)(1.25*256), (int)(1.00*256)},
{512, (int)(1.31*256), (int)(1.00*256)},
{512, (int)(1.38*256), (int)(1.00*256)},
{512, (int)(1.44*256), (int)(1.00*256)},
{512, (int)(1.44*256), (int)(1.00*256)},
{512, (int)(1.50*256), (int)(1.00*256)},
{512, (int)(1.56*256), (int)(1.00*256)},
{512, (int)(1.63*256), (int)(1.00*256)},
{512, (int)(1.69*256), (int)(1.00*256)},
{512, (int)(1.75*256), (int)(1.00*256)},
{512, (int)(1.81*256), (int)(1.00*256)},
{512, (int)(1.88*256), (int)(1.00*256)},
{512, (int)(1.94*256), (int)(1.00*256)},
{768, (int)(1.31*256), (int)(1.00*256)},
{768, (int)(1.38*256), (int)(1.00*256)},
{768, (int)(1.44*256), (int)(1.00*256)},
{768, (int)(1.44*256), (int)(1.00*256)},
{768, (int)(1.50*256), (int)(1.00*256)},
{768, (int)(1.56*256), (int)(1.00*256)},
{768, (int)(1.63*256), (int)(1.00*256)},
{768, (int)(1.69*256), (int)(1.00*256)},
{768, (int)(1.75*256), (int)(1.00*256)},
{768, (int)(1.81*256), (int)(1.00*256)},
{768, (int)(1.88*256), (int)(1.00*256)},
{768, (int)(1.94*256), (int)(1.00*256)},
{768, (int)(2.0*256), (int)(1.00*256)},
{768, (int)(2.00*256), (int)(1.04*256)},
{768, (int)(2.13*256), (int)(1.00*256)},
{768, (int)(2.25*256), (int)(1.00*256)},
{768, (int)(2.25*256), (int)(1.00*256)},
{768, (int)(2.38*256), (int)(1.00*256)},
{768, (int)(2.50*256), (int)(1.00*256)},
{768, (int)(2.50*256), (int)(1.00*256)},
{768, (int)(2.63*256), (int)(1.00*256)},
{768, (int)(2.75*256), (int)(1.00*256)},
{768, (int)(2.88*256), (int)(1.00*256)},
{768, (int)(2.88*256), (int)(1.00*256)},
{768, (int)(3.00*256), (int)(1.00*256)},
{768, (int)(3.13*256), (int)(1.00*256)},
{768, (int)(3.25*256), (int)(1.00*256)},
{768, (int)(3.38*256), (int)(1.00*256)},
{768, (int)(3.50*256), (int)(1.00*256)},
{768, (int)(3.63*256), (int)(1.00*256)},
{768, (int)(3.75*256), (int)(1.00*256)},
{768, (int)(3.88*256), (int)(1.00*256)},
{768, (int)(4.00*256), (int)(1.00*256)},
{768, (int)(4.00*256), (int)(1.04*256)},
{768, (int)(4.25*256), (int)(1.00*256)},
{768, (int)(4.25*256), (int)(1.04*256)},
{768, (int)(4.50*256), (int)(1.00*256)},
{768, (int)(4.75*256), (int)(1.00*256)},
{768, (int)(5.00*256), (int)(1.00*256)},
{768, (int)(5.00*256), (int)(1.04*256)},
{768, (int)(5.25*256), (int)(1.00*256)},
{768, (int)(5.50*256), (int)(1.00*256)},
{768, (int)(5.50*256), (int)(1.04*256)},
{768, (int)(5.75*256), (int)(1.02*256)},
{768, (int)(6.00*256), (int)(1.00*256)},
{768, (int)(6.25*256), (int)(1.00*256)},
{768, (int)(6.50*256), (int)(1.00*256)},
{768, (int)(6.75*256), (int)(1.00*256)},
{768, (int)(7.00*256), (int)(1.00*256)},
{768, (int)(7.00*256), (int)(1.04*256)},
{768, (int)(7.00*256), (int)(1.07*256)},
{768, (int)(7.00*256), (int)(1.11*256)},
{768, (int)(7.00*256), (int)(1.15*256)},
{768, (int)(7.00*256), (int)(1.19*256)},
{768, (int)(7.00*256), (int)(1.23*256)},
{768, (int)(7.00*256), (int)(1.27*256)},
{768, (int)(7.00*256), (int)(1.32*256)},
{768, (int)(7.00*256), (int)(1.37*256)},
{768, (int)(7.00*256), (int)(1.41*256)},
{768, (int)(7.00*256), (int)(1.46*256)},
{768, (int)(7.00*256), (int)(1.52*256)},
{768, (int)(7.00*256), (int)(1.57*256)},
{768, (int)(7.00*256), (int)(1.62*256)},
{768, (int)(7.00*256), (int)(1.68*256)},
{768, (int)(7.00*256), (int)(1.74*256)},
{768, (int)(7.00*256), (int)(1.80*256)},
{768, (int)(7.00*256), (int)(1.87*256)},
{768, (int)(7.00*256), (int)(1.93*256)},
{768, (int)(7.00*256), (int)(2.00*256)},
{768, (int)(7.25*256), (int)(2.00*256)},
{768, (int)(7.50*256), (int)(2.00*256)},
{768, (int)(7.75*256), (int)(2.00*256)},
{768, (int)(8.00*256), (int)(2.00*256)}
};

static const int h22_analog_gain_table[65] = 
{
	// coarse gain = 0
	(int)(1.00*256+0.5), (int)(1.06*256+0.5), (int)(1.13*256+0.5), (int)(1.19*256+0.5), 
	(int)(1.25*256+0.5), (int)(1.31*256+0.5), (int)(1.38*256+0.5), (int)(1.44*256+0.5), 
	(int)(1.50*256+0.5), (int)(1.56*256+0.5), (int)(1.63*256+0.5), (int)(1.69*256+0.5), 
	(int)(1.75*256+0.5), (int)(1.81*256+0.5), (int)(1.88*256+0.5), (int)(1.94*256+0.5),
	
	// coarse gain = 1
	(int)(2.00*256+0.5), (int)(2.13*256+0.5), (int)(2.25*256+0.5), (int)(2.38*256+0.5), 
	(int)(2.50*256+0.5), (int)(2.63*256+0.5), (int)(2.75*256+0.5), (int)(2.88*256+0.5),
	(int)(3.00*256+0.5), (int)(3.13*256+0.5), (int)(3.25*256+0.5), (int)(3.38*256+0.5),
	(int)(3.50*256+0.5), (int)(3.63*256+0.5), (int)(3.75*256+0.5), (int)(3.88*256+0.5),

	// coarse gain = 3
	(int)(4.00*256+0.5), (int)(4.25*256+0.5), (int)(4.50*256+0.5), (int)(4.75*256+0.5), 
	(int)(5.00*256+0.5), (int)(5.25*256+0.5), (int)(5.50*256+0.5), (int)(5.75*256+0.5), 
	(int)(6.00*256+0.5), (int)(6.25*256+0.5), (int)(6.50*256+0.5), (int)(6.75*256+0.5), 
	(int)(7.00*256+0.5), (int)(7.25*256+0.5), (int)(7.50*256+0.5), (int)(7.75*256+0.5), 
	
	// coarse gain = 7
	(int)(8.00*256+0.5), (int)(8.50*256+0.5), (int)(9.00*256+0.5), (int)(9.50*256+0.5), 
	(int)(10.00*256+0.5), (int)(10.50*256+0.5), (int)(11.00*256+0.5), (int)(11.50*256+0.5), 
	(int)(12.00*256+0.5), (int)(12.50*256+0.5), (int)(13.00*256+0.5), (int)(13.50*256+0.5), 
	(int)(14.00*256+0.5), (int)(14.50*256+0.5), (int)(15.00*256+0.5), (int)(15.50*256+0.5), 
	
	// coarse gain = 15
	(int)(16.00*256+0.5)
};


static sensor_fmt_t gH22FmtTable[] =
{
	{
		.desc		= "preview=1280*720,crop=1280*720",
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.bpp 		= 1,
		.mclk_src = MODE_MCLK_SRC_96M,
		.mclk = 24000000,
		.hpixel = 1280,
		.hoffset = 0,
		.vline = 720,
		.voffset = 0,
		.cdsp_calibration = 0,
	},
	{
		.desc		= "capture=1280*800,crop=1280*800",
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.bpp 		= 1,
		.mclk_src = MODE_MCLK_SRC_96M,
		.mclk = 24000000,
		.hpixel = 1280,
		.hoffset = 0,
		.vline = 800,
		.voffset = 0,
		.cdsp_calibration = 0,
	},
	{
		.desc		= "record=1280*720,crop=1280*720",
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.bpp 		= 1,
		.mclk_src = MODE_MCLK_SRC_96M,
		.mclk = 24000000,
		.hpixel = 1280,
		.hoffset = 0,
		.vline = 720,
		.voffset = 0,
		.cdsp_calibration = 0,
	},
};

#define C_SENSOR_FMT_MAX	sizeof(gH22FmtTable)/sizeof(sensor_fmt_t)

static mipi_config_t h22_mipi_setting = 
{
	.mipi_sep_clk_en = ENABLE,
	.mipi_sep_clk = 200000000,
	.mipi_sep_clk_src = MIPI_D_PHY_CLK,
	.byte_clk_edge = D_PHY_SAMPLE_POS,
	.low_power_en = DISABLE,
	.lane_num = MIPI_1_LANE,
	.ecc_check_en = ENABLE,
	.ecc_order = MIPI_ECC_ORDER3,
#ifdef CONFIG_FPGA_TEST	
	.data_mask_time = 270, //ns
#else
	.data_mask_time = 100, //ns
#endif
#if 1
	.check_hs_seq = MIPI_CHECK_LP_00,	//for mipi clock no stop 
#else 
	.check_hs_seq = MIPI_CHECK_HS_SEQ,	//for mipi clock auto stop 
#endif
};

static sensor_config_t h22_config_table =
{
	.sensor_timing_mode = MODE_CCIR_601,
	.sensor_data_mode = MODE_DATA_YUV,
	.sensor_interlace_mode = MODE_NONE_INTERLACE,
	.sensor_pclk_mode = MODE_POSITIVE_EDGE,
	.sensor_hsync_mode = MODE_ACTIVE_HIGH,
	.sensor_vsync_mode = MODE_ACTIVE_LOW,
	.sensor_fmt_num = C_SENSOR_FMT_MAX,
	.fmt = gH22FmtTable,
	.mipi_config = &h22_mipi_setting,
};

/**************************************************************************
 *             Sensor Setting Table								          *
 **************************************************************************/
//@@ MIPI-10bit_640x480_60fps 
const regval8_t SOI_H22_MIPI_VGA_f60[] =
{
	//Sleep Mode
	{0x12,0x40},
	//Gain
	{0x0E,0x1D}, //for MCLK=24Mhz, please change to 1C for MCLK=12Mhz
	{0x0F,0x09},
	{0x10,0x20},
	{0x11,0x80},
	//AEC/AGC
	{0x14,0x80},
	{0x16,0xA0},
	{0x17,0x40},
	{0x18,0xD5},
	{0x19,0x00},
	//DVP
	{0x1D,0x00},
	{0x1E,0x1C},
	{0x1F,0x10},
	//Frame
	{0x20,0x5C},
	{0x21,0x03},
	{0x22,0xEA},
	{0x23,0x02},
	//Window
	{0x24,0x84},	//enhance +4
	{0x25,0xE2},	//enhance +2
	{0x26,0x12},
	{0x27,0xC1},
	{0x28,0x0D},
	{0x29,0x00},
	//Physical
	{0x2C,0x28},
	{0x2D,0x28},
	{0x2E,0xA4},
	{0x2F,0x20},
	{0x37,0x62},
	{0x38,0x10},
	//MIPI
	{0x67,0x30},//Mipi sleep mode disable
	{0x6c,0x10},//Mipi active
	{0x76,0x20},//word counter low
	{0x77,0x03},//word counter high
	{0x12,0x00},
	{0xFF,0xFF}
};

//@@ MIPI-10bit_1280x720_30fps
const regval8_t SOI_H22_MIPI_720P_f30[] =
{
	//Sleep Mode
	{0x12,0x40},
	//Gain
	{0x0E,0x1D}, //for MCLK=24Mhz, please change to 1C for MCLK=12Mhz
	{0x0F,0x09},
	{0x10,0x20},
	{0x11,0x80},
	//AEC/AGC
	{0x14,0x80},
	{0x16,0xA0},
	{0x17,0x40},
	{0x18,0xD5},
	{0x19,0x00},
	//DVP
	{0x1D,0x00},
	{0x1E,0x1C},
	//Frame
	{0x20,0xDC},
	{0x21,0x05},
	{0x22,0x56},
	{0x23,0x03},
	//Window
	{0x24,0x04},	//enhance +4
	{0x25,0xD2},	//enhance +2
	{0x26,0x25},
	{0x27,0xC1},
	{0x28,0x0D},
	{0x29,0x00},
	//Physical
	{0x2C,0x00},
	{0x2D,0x0A},
	{0x2E,0xC2},
	{0x2F,0x20},
	{0x37,0x40},
	{0x38,0x98},
	//MIPI
	{0x67,0x30},//Mipi sleep mode disable
	{0x6c,0x10},//Mipi active
	{0x76,0x40},//word counter low
	{0x77,0x06},//word counter high
	{0x12,0x00},
	{0xFF,0xFF}
};

//@@ MIPI-10bit_1280x800_30fps
const regval8_t SIO_H22_MIPI_1280_800_f30[] =
{
	//---Sleep Mode
	{0x12,0x40},
	//---Gain
	{0x0E,0x1D}, //for MCLK=24Mhz, please change to 1C for MCLK=12Mhz
	{0x0F,0x09},
	{0x10,0x20},
	{0x11,0x80},
	//---AEC/AGC
	{0x14,0x80},
	{0x16,0xA0},
	{0x17,0x40},
	{0x18,0xD5},
	{0x19,0x00},
	//---DVP
	{0x1D,0x00},
	{0x1E,0x1C},
	//---Frame
	{0x20,0xDC},
	{0x21,0x05},
	{0x22,0x55},
	{0x23,0x03},
	//---Window
	{0x24,0x04},	//enhance +4 for cdsp interpolation
	{0x25,0x22},	//enhance +2 for cdsp interpolation
	{0x26,0x35},
	{0x27,0xC1},
	{0x28,0x0D},
	{0x29,0x00},
	//---Physical
	{0x2C,0x00},
	{0x2D,0x00},
	{0x2E,0xCC},
	{0x2F,0x20},
	//;;;;;
	{0x37,0x4A},
	{0x38,0x70},
	// MIPI
	{0x67,0x30},//Mipi sleep mode disable
	{0x6c,0x10},//Mipi active
	{0x76,0x40},//word counter low
	{0x77,0x06},//word counter high
	{0x12,0x00},
	{0xFF,0xFF}
};

/**************************************************************************
 *             F U N C T I O N    I M P L E M E N T A T I O N S           *
 **************************************************************************/
static int
sensor_i2c_open(
	unsigned int slave_id,
	unsigned int scl_speed
)
{
#if (I2C_MODE == HW_I2C)
	g_i2c_handle = gp_i2c_bus_request(slave_id, scl_speed);
	if((g_i2c_handle == 0) ||(g_i2c_handle == -ENOMEM)) {
		printk(KERN_WARNING "i2cReqFail %d\n", g_i2c_handle);
		return -1;
	}

	return 0;

#elif (I2C_MODE == HW_TI2C)
	g_ti2c_handle.pDeviceString = "H22_MIPI";		
	g_ti2c_handle.slaveAddrMode = TI2C_NORMAL_SLAVEADDR_8BITS;		
	g_ti2c_handle.slaveAddr = slave_id;		
	g_ti2c_handle.clockRate = scl_speed;
	if (gp_ti2c_bus_request(&g_ti2c_handle) != 0) {
		printk("H22_MIPI ti2c request failed\n");
		return -1;
	}
	//gp_ti2c_bus_int_mode_en(0);
	return 0;
#endif
}

static int
sensor_i2c_close(
	void
)
{
#if (I2C_MODE == HW_I2C)
	return gp_i2c_bus_release(g_i2c_handle);
#elif (I2C_MODE == HW_TI2C)
	return gp_ti2c_bus_release(&g_ti2c_handle);
#endif
}

static int 
h22_read(
	unsigned char reg, 
	unsigned char *value
)
{
#if (I2C_MODE == HW_I2C)
	char addr[1], data[1];
	int nRet;
	
	addr[0] = reg & 0xFF;
	nRet = gp_i2c_bus_write(g_i2c_handle, addr, 1);
	if(nRet <= 0) {
		return nRet;
	}
	
	nRet = gp_i2c_bus_read(g_i2c_handle, data, 1);
	*value = data[0];
	return nRet;
	
#elif (I2C_MODE == HW_TI2C)
	unsigned char addr[1], data[1];
	int nRet;
	int retry = 0;
	
	addr[0] = reg & 0xFF;	
	g_ti2c_handle.transmitMode = TI2C_NORMAL_WRITE_MODE;	
	g_ti2c_handle.pBuf = addr;	
	g_ti2c_handle.dataCnt = 1;	
	nRet = gp_ti2c_bus_xfer(&g_ti2c_handle);
	while(nRet<0) {
		retry++;
		if(nRet == -88)
			return nRet;
		nRet = gp_ti2c_bus_xfer(&g_ti2c_handle);
		if(nRet<0 && retry>TI2C_RETRY) {
			printk("retry too much arr\n");
			return nRet;
		}
	}
	
	g_ti2c_handle.transmitMode = TI2C_NORMAL_READ_MODE;	
	g_ti2c_handle.pBuf = data;	
	g_ti2c_handle.dataCnt = 1;	
	nRet = gp_ti2c_bus_xfer(&g_ti2c_handle);
	*value = data[0];
	return nRet;
#endif
}

static int 
h22_write(
	unsigned char reg, 
	unsigned char value
)
{
#if (I2C_MODE == HW_I2C)
	char data[2];
	
	data[0] = reg & 0xFF;
	data[1] = value;	
	return gp_i2c_bus_write(g_i2c_handle, data, 2);
	
#elif (I2C_MODE == HW_TI2C)
	unsigned char data[2];
	int ret, retry = 0;

	data[0] = reg & 0xFF;
	data[1] = value;	
	g_ti2c_handle.transmitMode = TI2C_NORMAL_WRITE_MODE;	
	g_ti2c_handle.pBuf = data;	
	g_ti2c_handle.dataCnt = 2;		
	ret = gp_ti2c_bus_xfer(&g_ti2c_handle);
	while(ret<0) {
		retry++;
		if(ret == -88) {
			printk("ti2c conflict\r\n");
			return ret;
		}
		ret = gp_ti2c_bus_xfer(&g_ti2c_handle);
		if(ret<0 && retry>TI2C_RETRY) {
			printk("retry too much aw\n");
			return ret;
		}
	}
	return ret;
#endif	
}

static int
H22WrTable(
	regval8_t *pTable
)
{
	int ret = 0;

	while(1) {
		if((pTable->reg_num == 0xFF) && (pTable->value == 0xFF)) {
			break;
		}
		
		ret = h22_write(pTable->reg_num, pTable->value);
		if(ret < 0) {
			return -1;
		}
		
		printk("[0x%x] = 0x%x\n", pTable->reg_num, pTable->value);
		pTable ++;
	}
	
	return ret;
}

static int 
h22_init(
	struct v4l2_subdev *sd,
	u32 val
)
{
	h22_seInfo.time = 0x10;
	h22_seInfo.analog_gain = H22_MIN_ANALOG_GAIN;
	h22_seInfo.digital_gain = H22_MIN_DIGITAL_GAIN;

	h22_seInfo.max_time = H22_MAX_EXPOSURE_TIME;
	h22_seInfo.max_analog_gain = H22_MAX_ANALOG_GAIN;
	h22_seInfo.max_digital_gain = H22_MAX_DIGITAL_GAIN;


	h22_seInfo.min_time = H22_MIN_EXPOSURE_TIME;
	h22_seInfo.min_analog_gain = H22_MIN_ANALOG_GAIN;
	h22_seInfo.min_digital_gain = H22_MIN_DIGITAL_GAIN;

	h22_seInfo.userISO = DISABLE;
	h22_set_exp_freq(50);

	sensor_fps = V4L2_TC_TYPE_30FPS;
	sensor_total_ev = OV2710_30FPS_50HZ_EXP_TIME_TOTAL;
	sensor_swith_time = 2000;
	current_frame_rate= 30;

	printk("%s\n", __FUNCTION__);
	return 0;
}

static int 
h22_preview(
	void
)
{
	printk("%s\n", __FUNCTION__);
	return H22WrTable((regval8_t *)SOI_H22_MIPI_720P_f30);
}

static int 
h22_capture(
	void
)
{
	printk("%s\n", __FUNCTION__);
	return H22WrTable((regval8_t *)SIO_H22_MIPI_1280_800_f30);
}

static int 
h22_record(
	void
)
{
	printk("%s\n", __FUNCTION__);
	return H22WrTable((regval8_t *)SOI_H22_MIPI_720P_f30);
}

static int 
h22_reset(
	struct v4l2_subdev *sd, 
	u32 val
)
{
	return 0;
}

#if 0
static int h22_get_real_agc_gain(int agc_gain)
{
	int real_agc_gain;

	real_agc_gain = 0x10 + (agc_gain & 0x0f);
	real_agc_gain = real_agc_gain * (1 + ((agc_gain >> 4) & 1)) * (1 + ((agc_gain >> 5) & 1))
			* (1 + ((agc_gain >> 6) & 1)) * (1 + ((agc_gain >> 7) & 1)) * (1 + ((agc_gain >> 8) & 1));
	
	return real_agc_gain;
}
#endif

static int h22_cvt_analog_gain(int analog_gain)
{
int i;
	int coarse_gain, fine_gain;
	int *p_analog_gain_table = (int *)h22_analog_gain_table;

	for(i=0; i<65; i++)
	{
		if(analog_gain >= p_analog_gain_table[i] && analog_gain < p_analog_gain_table[i+1])
			break;
	} 

	if(i < 16) {
		coarse_gain = 0;
		fine_gain = i;
	}
	else if(i < (16 + 16)) {
		coarse_gain = 1;
		fine_gain = (i - 16); 
	}
	else if(i < (16 + 16 + 16)) {
		coarse_gain = 3;
		fine_gain = (i - 16 - 16);
	}
	else if(i < (16 + 16 + 16 + 16)) {
		coarse_gain = 7;
		fine_gain = (i - 16 - 16 -16);
	}
	else {
		coarse_gain = 15;
		fine_gain = 0;
	}

	return ((coarse_gain << 4) | fine_gain);
}

static int h22_set_exposure_time(sensor_exposure_t *se)
{
	unsigned char cvt_gain;

	// AGC Gain / analog gain
	cvt_gain = h22_cvt_analog_gain(se->analog_gain);
	h22_write(0x00, cvt_gain);

	// set exposure time
	h22_write(0x01, se->time & 0xFF);
	h22_write(0x02, (se->time >> 8) ^ 0xFF);
	return 0;
}

static int h22_set_xfps_exposure_time(sensor_exposure_t *si)
{
	int idx;

	si->sensor_ev_idx += si->ae_ev_idx;
	if(si->sensor_ev_idx >= si->max_ev_idx) {
		si->sensor_ev_idx = si->max_ev_idx;
	}
	
	if(si->sensor_ev_idx < 0) {
		si->sensor_ev_idx = 0;
	}
	
	idx = si->sensor_ev_idx * 3;
	si->time = p_expTime_table[idx];
	si->analog_gain = p_expTime_table[idx+1];
	si->digital_gain = p_expTime_table[idx+2];

	return 0;
}

static int h22_get_exposure_time(sensor_exposure_t *se)
{	
	return 0;
}

static void h22_set_exp_freq(int freq)
{
	if(freq == 50)
	{
		h22_seInfo.sensor_ev_idx = H22_50HZ_INIT_EV_IDX;
		h22_seInfo.ae_ev_idx = 0;
		h22_seInfo.daylight_ev_idx= H22_50HZ_DAY_EV_IDX;
		h22_seInfo.night_ev_idx= H22_50HZ_NIGHT_EV_IDX;			
		h22_seInfo.max_ev_idx = H22_50HZ_MAX_EXP_IDX - 1;
		
		p_expTime_table = (int *)h22_exp_time_gain_50Hz;
		
		if(h22_seInfo.sensor_ev_idx > h22_seInfo.max_ev_idx) {
			h22_seInfo.sensor_ev_idx = h22_seInfo.max_ev_idx;
		}
	}
	else if(freq == 60)
	{
		h22_seInfo.sensor_ev_idx = H22_60HZ_INIT_EV_IDX;
		h22_seInfo.ae_ev_idx = 0;
		h22_seInfo.daylight_ev_idx= H22_60HZ_DAY_EV_IDX;
		h22_seInfo.night_ev_idx= H22_60HZ_NIGHT_EV_IDX;
		h22_seInfo.max_ev_idx = H22_60HZ_MAX_EXP_IDX - 1;
		
		p_expTime_table = (int *)h22_exp_time_gain_60Hz;
	
		if(h22_seInfo.sensor_ev_idx > h22_seInfo.max_ev_idx) {
				h22_seInfo.sensor_ev_idx = h22_seInfo.max_ev_idx;
		}
	}
}

static int 
h22_queryctrl(
	struct v4l2_subdev *sd,
	struct v4l2_queryctrl *qc
)
{
	/* Fill in min, max, step and default value for these controls. */
	switch(qc->id)
	{
	case V4L2_CID_AUTO_WHITE_BALANCE:
		qc->minimum = 0;
		qc->maximum = 1;
		qc->step = 1;
		qc->default_value = 1;
		break;

	case V4L2_CID_POWER_LINE_FREQUENCY:
		qc->minimum = 50;
		qc->maximum = 60;
		qc->step = 10;
		qc->default_value = 50;
		break;

	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		qc->minimum = 0;
		qc->maximum = 3;
		qc->step = 1;
		qc->default_value = 0;
		break;

	case V4L2_CID_BACKLIGHT_COMPENSATION:
		qc->minimum = 0;
		qc->maximum = 1;
		qc->step = 1;
		qc->default_value = 0;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int 
h22_g_ctrl(
	struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl
)
{	
	switch(ctrl->id)
	{
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;

	case V4L2_CID_POWER_LINE_FREQUENCY:
		break;

	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		break;

	case V4L2_CID_BACKLIGHT_COMPENSATION:	
		break;
	case V4L2_CID_EXPOSURE:
		if(p_expTime_table == 0)
		{
			h22_get_exposure_time(&h22_seInfo);
		}
		ctrl->value = (int)&h22_seInfo;
		break;
		
	case V4L2_CID_USER_CLASS:
		// for Comi testing
		h22_get_exposure_time(&h22_seInfo);
		ctrl->value = (int)&h22_seInfo;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int 
h22_s_ctrl(
	struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl
)
{
	int nRet = 0;

	switch(ctrl->id)
	{
	case V4L2_CID_AUTO_WHITE_BALANCE:
		printk("WBAUTO = %d\n", ctrl->value);
		break;
		
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		printk("WB = %d\n", ctrl->value);
		if(ctrl->value == 0) {	// SUNSHINE 
			
		} else if(ctrl->value == 1) {	// CLOUDY
			
		} else if(ctrl->value == 2) {	// FLUORESCENCE
			
		} else if(ctrl->value == 3) {	// INCANDESCENCE

		}
		break; 
 
	case V4L2_CID_BACKLIGHT_COMPENSATION:
		printk("NightMode = %d\n", ctrl->value);
		if(ctrl->value) {	// NIGH MODE ON
			
		} else {	// NIGH MODE OFF
			
		}
		break;
	case V4L2_CID_EXPOSURE:
		{
		sensor_exposure_t *si;
		si = (sensor_exposure_t *) ctrl->value;
		
		h22_seInfo.userISO = si->userISO;
		if(p_expTime_table != 0)
		{
			h22_seInfo.ae_ev_idx = si->ae_ev_idx;
			nRet = h22_set_xfps_exposure_time(&h22_seInfo);
		}
		else
		{
			h22_seInfo.time = si->time;
			h22_seInfo.analog_gain = si->analog_gain;
			nRet = h22_set_exposure_time(&h22_seInfo);
		}
		
		
		}
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		{
			enum v4l2_power_line_frequency line_freq;
			line_freq = ctrl->value;
			sensor_swith_time = 2000;
			current_frame_rate= 30;
			
			if(sensor_fps == V4L2_TC_TYPE_30FPS)
			{
				if(line_freq == V4L2_CID_POWER_LINE_FREQUENCY_DISABLED 
					|| line_freq == V4L2_CID_POWER_LINE_FREQUENCY_50HZ)
				{
					h22_set_exp_freq(50);
				}
				else if(line_freq == V4L2_CID_POWER_LINE_FREQUENCY_60HZ)
				{
					h22_set_exp_freq(60);
				}
				else	
					return -EINVAL;
			}
			else if(sensor_fps == V4L2_TC_TYPE_25FPS_LDW)
			{
				// for ldw use only
				if(line_freq == V4L2_CID_POWER_LINE_FREQUENCY_DISABLED 
					|| line_freq == V4L2_CID_POWER_LINE_FREQUENCY_50HZ)
				{
					
					
				}
				else if(line_freq == V4L2_CID_POWER_LINE_FREQUENCY_60HZ)
				{
					
					
				}
				else 
					return -EINVAL;
			}
		}
		break;
	default:
		return -EINVAL;
	}
	
	return nRet; 
}

static int 
h22_querystd(
	struct v4l2_subdev *sd,
	v4l2_std_id *std
)
{
	return 0;
}

static int 
h22_enum_fmt(
	struct v4l2_subdev *sd, 
	struct v4l2_fmtdesc *fmtdesc
)
{
	printk("%s\n", __FUNCTION__);
	if(fmtdesc->index >= C_SENSOR_FMT_MAX)
		return -EINVAL;

	fmtdesc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	memcpy((void *)fmtdesc->description, (void *)gH22FmtTable[fmtdesc->index].desc, 32);
	fmtdesc->pixelformat = gH22FmtTable[fmtdesc->index].pixelformat;
	return 0;
}

static int 
h22_g_fmt(
	struct v4l2_subdev *sd, 
	struct v4l2_format *fmt
)
{
	printk("%s\n", __FUNCTION__);
	fmt->fmt.pix.width = gH22Dev.fmt->hpixel;
	fmt->fmt.pix.height = gH22Dev.fmt->vline;
	fmt->fmt.pix.pixelformat = gH22Dev.fmt->pixelformat;
	fmt->fmt.pix.field = V4L2_FIELD_NONE;
	fmt->fmt.pix.bytesperline = gH22Dev.fmt->hpixel * gH22Dev.fmt->bpp;
	fmt->fmt.pix.sizeimage = fmt->fmt.pix.bytesperline * gH22Dev.fmt->vline;

	return 0;
}

static int 
h22_try_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_format *fmt
)
{
	return 0;
}

static int 
h22_s_fmt(
	struct v4l2_subdev *sd, 
	struct v4l2_format *fmt
)
{
	int ret;

	printk("%s = %d\n", __FUNCTION__, fmt->fmt.pix.priv);
	switch(fmt->fmt.pix.priv)
	{
	case 0: 
		ret = h22_preview();
		break;

	case 1: 
		ret = h22_capture();
		break;

	case 2: 
		ret = h22_record();
		break;	

	default:
		ret = -1;
	}

	gH22Dev.fmt = &gH22FmtTable[fmt->fmt.pix.priv];
	return ret;
}

static int 
h22_cropcap(
	struct v4l2_subdev *sd,
	struct v4l2_cropcap *cc
)
{
	return 0;
}

static int 
h22_g_crop(
	struct v4l2_subdev *sd,
	struct v4l2_crop *crop
)
{
	return 0;
}

static int 
h22_s_crop(
	struct v4l2_subdev *sd,
	struct v4l2_crop *crop
)
{
	return 0;
}

static int 
h22_g_parm(
	struct v4l2_subdev *sd,
	struct v4l2_streamparm *parms
)
{
	return 0;
}

static int 
h22_s_parm(
	struct v4l2_subdev *sd,
	struct v4l2_streamparm *param
)
{
	return 0;
}

static int 
h22_s_interface(
	struct v4l2_subdev *sd,
	struct v4l2_interface *interface
)
{
	return 0;
}

static int 
h22_suspend(
	struct v4l2_subdev *sd
)
{
	return 0;
}

static int 
h22_resume(
	struct v4l2_subdev *sd
)
{
	return 0;
}

static const struct v4l2_subdev_core_ops h22_core_ops = 
{
	.init = h22_init,
	.reset = h22_reset,
	.queryctrl = h22_queryctrl,
	.g_ctrl = h22_g_ctrl,
	.s_ctrl = h22_s_ctrl,
};

static const struct v4l2_subdev_video_ops h22_video_ops = 
{
	.querystd = h22_querystd,
	.enum_fmt = h22_enum_fmt,
	.g_fmt = h22_g_fmt,
	.try_fmt = h22_try_fmt,
	.s_fmt = h22_s_fmt,
	.cropcap = h22_cropcap,
	.g_crop = h22_g_crop,
	.s_crop = h22_s_crop,
	.g_parm = h22_g_parm,
	.s_parm = h22_s_parm,
};

static const struct v4l2_subdev_ext_ops h22_ext_ops = 
{
	.s_interface = h22_s_interface,
	.suspend = h22_suspend,
	.resume = h22_resume,
};

static const struct v4l2_subdev_ops h22_ops = 
{
	.core = &h22_core_ops,
	.video = &h22_video_ops,
	.ext = &h22_ext_ops
};

static int __init 
h22_module_init(
		void
)
{
	if(sensor_i2c_open(SOI_H22_ID, 50) < 0) {
		printk(KERN_WARNING "i2cReqFail\n");
		return -1;
	}

	printk(KERN_WARNING "ModuleInit: h22 mipi\n");
	gH22Dev.fmt = &gH22FmtTable[0];
	v4l2_subdev_init(&(gH22Dev.sd), &h22_ops);
	strcpy(gH22Dev.sd.name, "sensor_h22_mipi");
	register_sensor(&gH22Dev.sd, (int *)&param[0], &h22_config_table);
	return 0;
}

static void __exit
h22_module_exit(
		void
)
{
	sensor_i2c_close();
	unregister_sensor(&(gH22Dev.sd));
}

module_init(h22_module_init);
module_exit(h22_module_exit);

/**************************************************************************
 *                  M O D U L E    D E C L A R A T I O N                  *
 **************************************************************************/
MODULE_AUTHOR("Generalplus");
MODULE_DESCRIPTION("Generalplus h22 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");


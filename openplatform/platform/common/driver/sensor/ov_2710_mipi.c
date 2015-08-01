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
#define OV2710_MIPI_CLK_NO_STOP_EN	1

#define OV2710_ID		0x6C
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


#define OV2710_MAX_EXPOSURE_TIME			0x0438 // depend on 0x350C/0x350D
#define OV2710_MIN_EXPOSURE_TIME			0x0008
#define OV2710_MAX_ANALOG_GAIN			(16*256)
#define OV2710_MIN_ANALOG_GAIN			(1*256)
#define OV2710_MAX_DIGITAL_GAIN			(1*256)
#define OV2710_MIN_DIGITAL_GAIN 			(1*256)


// 30fps
#define OV2710_30FPS_50HZ_INIT_EV_IDX 			129
#define OV2710_30FPS_50HZ_DAY_EV_IDX 			129
#define OV2710_30FPS_50HZ_NIGHT_EV_IDX		178
#define OV2710_30FPS_50HZ_EXP_TIME_TOTAL		(258-13)
#define OV2710_30FPS_50HZ_MAX_EV_IDX			(OV2710_30FPS_50HZ_EXP_TIME_TOTAL - 27) //15)


#define OV2710_30FPS_60HZ_INIT_EV_IDX 			129
#define OV2710_30FPS_60HZ_DAY_EV_IDX 			129
#define OV2710_30FPS_60HZ_NIGHT_EV_IDX		181
#define OV2710_30FPS_60HZ_EXP_TIME_TOTAL		(261-13)
#define OV2710_30FPS_60HZ_MAX_EV_IDX			(OV2710_30FPS_60HZ_EXP_TIME_TOTAL - 27) //15)



#define OV2710_MIN_D_GAIN			1.5


/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct regval_list_s 
{
	unsigned short reg_num;
	unsigned char value;
}regval16_t;

typedef struct sensor_dev_s 
{
	struct v4l2_subdev sd;
	sensor_fmt_t *fmt;	/* Current format */
}sensor_dev_t;

/**************************************************************************
 *                 E X T E R N A L    R E F E R E N C E S                 *
 **************************************************************************/

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static int current_frame_rate;

static int *p_expTime_table;
static int sensor_fps, sensor_total_ev, sensor_swith_time;

static sensor_dev_t	gOV2710Dev;
static sensor_exposure_t 	ov2710_seInfo;
#if (I2C_MODE == HW_I2C)
static int g_i2c_handle;
#elif (I2C_MODE == HW_TI2C)
static ti2c_set_value_t g_ti2c_handle;
#endif

static char *param[] = {"0", "PORT0", "0", "NONE", "0", "NONE"};
static int nstrs = 6;
module_param_array(param, charp, &nstrs, S_IRUGO);


static const unsigned short g_ov2710_lenscmp_table[] = 
{
	0x100, 0x100, 0x101, 0x101, 0x101, 0x101, 0x102, 0x103, 
	0x103, 0x103, 0x104, 0x104, 0x104, 0x105, 0x105, 0x106, 
	0x106, 0x107, 0x106, 0x106, 0x107, 0x107, 0x108, 0x108, 
	0x109, 0x109, 0x10a, 0x10a, 0x10a, 0x10b, 0x10b, 0x10c, 
	0x10c, 0x10d, 0x10e, 0x10f, 0x110, 0x110, 0x110, 0x110, 
	0x111, 0x111, 0x112, 0x112, 0x113, 0x114, 0x114, 0x115, 
	0x115, 0x116, 0x116, 0x117, 0x118, 0x119, 0x11a, 0x11b, 
	0x11b, 0x11c, 0x11d, 0x11e, 0x11f, 0x121, 0x121, 0x122, 
	0x123, 0x124, 0x126, 0x127, 0x128, 0x129, 0x12a, 0x12c, 
	0x12d, 0x12e, 0x12f, 0x131, 0x132, 0x132, 0x134, 0x135, 
	0x137, 0x138, 0x139, 0x13b, 0x13c, 0x13d, 0x13f, 0x140, 
	0x141, 0x143, 0x144, 0x145, 0x147, 0x149, 0x149, 0x14b, 
	0x14d, 0x14d, 0x14f, 0x150, 0x152, 0x154, 0x155, 0x157, 
	0x157, 0x159, 0x15b, 0x15c, 0x15e, 0x15f, 0x161, 0x162, 
	0x163, 0x164, 0x166, 0x167, 0x168, 0x169, 0x16b, 0x16c, 
	0x16e, 0x16f, 0x171, 0x172, 0x173, 0x175, 0x175, 0x178, 
	0x179, 0x17a, 0x17c, 0x17d, 0x17f, 0x180, 0x182, 0x183, 
	0x185, 0x187, 0x188, 0x18a, 0x18d, 0x18e, 0x190, 0x192, 
	0x194, 0x197, 0x19a, 0x19e, 0x19f, 0x1a2, 0x1a1, 0x177, 
	0x151, 0x12e, 0x10e, 0x0f1, 0x0d7, 0x0bf, 0x0aa, 0x097, 
	0x085, 0x075, 0x067, 0x05a, 0x04f, 0x045, 0x03c, 0x034, 
	0x02d, 0x027, 0x021, 0x01c, 0x018, 0x014, 0x011, 0x00e, 
	0x00c, 0x00a, 0x008, 0x006, 0x005, 0x004, 0x003, 0x002, 
	0x001, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 

};


static const unsigned short g_ov2710_r_b_gain[71][2] = 
{ // { r gain, b gain }
{8, 160},
{14, 160},
{20, 159},
{26, 159},
{31, 158},
{36, 157},
{40, 157},
{45, 156},
{49, 155},
{53, 154},
{56, 154},
{60, 153},
{63, 152},
{66, 151},
{68, 150},
{71, 149},
{73, 148},
{75, 147},
{77, 146},
{79, 144},
{80, 143},
{82, 142},
{83, 141},
{84, 139},
{85, 138},
{86, 137},
{87, 135},
{87, 134},
{88, 132},
{89, 131},
{89, 129},
{89, 128},
{90, 126},
{90, 124},
{90, 123},
{90, 121},
{90, 119},
{91, 117},
{91, 115},
{91, 114},
{91, 112},
{91, 110},
{91, 108},
{91, 106},
{92, 104},
{92, 101},
{92, 99},
{93, 97},
{93, 95},
{94, 93},
{94, 90},
{95, 88},
{96, 86},
{97, 83},
{98, 81},
{100, 79},
{101, 76},
{103, 74},
{104, 71},
{106, 68},
{108, 66},
{111, 63},
{113, 60},
{116, 58},
{119, 55},
{122, 52},
{125, 49},
{129, 46},
{132, 43},
{137, 40},
{141, 37}
};



static const unsigned int g_ov2710_gamma_045_20_90_table[] = 
{
0x000000, 0x040400, 0x111102, 0x114506, 0x11510c, 0x151512, 0x154519, 0x154520, 
0x151526, 0x04452c, 0x044131, 0x110435, 0x044138, 0x11103c, 0x10443f, 0x044142, 
0x041145, 0x011049, 0x11044c, 0x11044f, 0x010452, 0x010455, 0x011058, 0x04105b, 
0x10415d, 0x104160, 0x010463, 0x104066, 0x010468, 0x04106b, 0x01046d, 0x101070, 
0x040472, 0x004174, 0x101077, 0x040479, 0x01047b, 0x01017d, 0x004080, 0x004082, 
0x004084, 0x004086, 0x004088, 0x00408a, 0x00408c, 0x00108e, 0x010090, 0x001091, 
0x040093, 0x004094, 0x000195, 0x040097, 0x004098, 0x001099, 0x00019a, 0x00009c, 
0x10009d, 0x10009e, 0x04009f, 0x0400a0, 0x1000a1, 0x1000a2, 0x0000a3, 0x0000a4, 
0x0001a4, 0x0010a5, 0x0040a6, 0x0400a7, 0x0000a8, 0x0000a9, 0x0010a9, 0x0100aa, 
0x0000ab, 0x0000ac, 0x0004ac, 0x0100ad, 0x0000ae, 0x0000af, 0x0010af, 0x0100b0, 
0x0000b1, 0x0000b2, 0x0004b2, 0x0040b3, 0x0400b4, 0x0000b5, 0x0000b6, 0x0001b6, 
0x0010b7, 0x0040b8, 0x0040b9, 0x0100ba, 0x0100bb, 0x0100bc, 0x0040bd, 0x0040be, 
0x0010bf, 0x0004c0, 0x1001c1, 0x0400c3, 0x0040c4, 0x0004c5, 0x0400c7, 0x0040c8, 
0x1001c9, 0x0040cb, 0x0401cc, 0x0010ce, 0x0040d0, 0x0401d1, 0x0404d3, 0x1010d5, 
0x1010d7, 0x1010d9, 0x1010db, 0x0404dd, 0x0104df, 0x0041e1, 0x1010e4, 0x0104e6, 
0x1010e9, 0x0104eb, 0x0410ee, 0x0411f0, 0x1041f3, 0x1041f6, 0x1041f9, 0x0441fc
};

static const short g_ov2710_color_matrix4gamma045_20_90[9] = 
{		
	(short) ((1.2250362030761943000 *64) + 0.5),
	(short) ((-0.118072848118916540 *64) + 0.5),
	(short) ((-0.106963354957277710 *64) + 0.5),
	(short) ((-0.241112945438339890 *64) + 0.5),
	(short) ((1.4356035309807371000 *64) + 0.5),
	(short) ((-0.194490585542397230 *64) + 0.5),
	(short) ((-0.059437423749203792 *64) + 0.5),
	(short) ((-0.895170869603835960 *64) + 0.5),
	(short) ((1.9546082933530398000 *64) + 0.5)
};




static const short g_ov2710_awb_thr[19] = 
{
	200, // awbwinthr
	
	0*64, // sindata
	1*64, // cosdata 
	
	 50, // Ythr0
	100, // Ythr1
	146,	// Ythr2 
	210, // Ythr3 
	
	-7, // UL1N1 
	 9, // UL1P1 
	-5, // VL1N1 
	 6, // VL1P1 
	
	 -10, //UL1N2
	  11, //UL1P2
	 -6, //VL1N2
	  7, // VL1P2
	
	 -11, // UL1N3
	  11,	 //UL1P3
	 -9, // VL1N3
	  9, //VL1P3
};


static const short g_ov2710_ob_table[8] = 
{
	0, // obautoen	
	1, // obmanuen
	17, // maunob
	1, // wboffseten
	1, // wbo_r
	1, // wbo_gr
	0, // wbo_gb
	1 // wbo_b
};
/*
sensor_calibration_t ov2710_cdsp_calibration = 
{
	.ob = g_ov2710_ob_table,
	.lenscmp = g_ov2710_lenscmp_table,
	.wb_gain = g_ov2710_r_b_gain,
	.gamma1 = g_ov2710_gamma_045_20_90_table,
	.color_matrix1 = g_ov2710_color_matrix4gamma045_20_90,
	.awb_thr = g_ov2710_awb_thr
};*/

static const  int ov2710_30fps_exp_time_gain_60Hz[OV2710_30FPS_60HZ_EXP_TIME_TOTAL][3] = 
{
{4, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{8, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{8, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{8, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{8, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{9, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{9, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{9, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{10, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{10, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{10, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{11, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{11, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{12, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{12, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{12, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{13, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{13, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{14, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{14, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{15, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{15, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{16, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{16, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{17, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{17, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{18, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{19, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{19, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{20, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{21, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{21, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{22, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{23, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{24, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{25, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{26, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{26, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{27, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{28, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{29, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{30, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{31, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{33, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{34, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{35, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{36, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{37, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{39, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{40, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{41, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{43, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{44, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{46, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{48, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{49, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{51, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{53, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{55, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{57, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{59, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{61, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{63, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{65, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{67, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{70, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{72, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{75, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{77, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{80, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{83, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{86, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{89, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{92, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{95, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{99, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{102, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{106, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{109, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{113, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{117, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{121, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{126, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{130, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{135, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{140, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{144, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{150, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{155, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{160, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{166, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{172, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{178, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{184, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{191, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{197, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{204, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{211, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{219, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{227, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{235, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{243, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{251, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{260, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{269, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.1875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.3125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.5*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.5625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.6875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.8125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{279, (int)(1.9375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.1875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.3125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.5*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.5625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.6875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.8125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{558, (int)(1.9375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.3125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.5*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.5625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.6875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.8125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{837, (int)(1.9375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.5*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.5625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.6875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.8125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(1.9375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.0*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.07*OV2710_MIN_D_GAIN*256), (int)(1.04*256)},
{1112, (int)(2.14*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.22*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.30*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.38*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.46*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.55*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.64*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.83*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(2.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.03*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.14*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.36*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.48*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.61*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(3.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(4.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(4.14*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(4.29*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(4.44*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(4.59*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(4.76*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(4.92*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(5.10*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(5.28*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(5.46*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(5.66*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(5.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(6.06*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(6.28*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(6.50*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(6.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(6.96*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(7.21*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(7.46*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(7.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(8.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(8.28*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(8.57*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(8.88*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(9.19*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(9.51*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(9.85*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(10.20*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(10.56*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(10.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(11.31*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(11.71*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(12.13*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(12.55*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(13.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(13.45*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(13.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(14.42*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(14.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(15.45*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(16.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(16.56*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(17.15*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(17.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(18.83*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(19.03*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(19.70*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(20.39*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
/*{1112, (int)(21.11*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(21.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(22.63*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(23.43*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(24.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(25.11*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(25.99*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(26.91*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(27.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(28.84*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(29.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(30.91*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1112, (int)(32.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)}*/
};


static const  int ov2710_30fps_exp_time_gain_50Hz[OV2710_30FPS_50HZ_EXP_TIME_TOTAL][3] = 
{ // {time, analog gain, digital gain}
{4, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{5, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{6, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{7, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{8, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{8, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{8, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{9, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{9, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{9, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{9, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{10, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{10, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{10, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{11, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{11, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{12, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{12, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{12, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{13, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{13, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{14, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{14, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{15, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{15, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{16, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{16, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{17, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{18, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{18, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{19, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{20, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{20, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{21, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{22, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{22, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{23, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{24, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{25, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{26, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{27, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{28, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{29, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{30, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{31, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{32, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{33, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{34, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{35, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{36, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{38, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{39, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{40, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{42, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{43, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{45, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{46, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{48, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{50, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{52, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{53, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{55, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{57, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{59, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{61, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{63, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{66, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{68, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{70, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{73, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{75, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{78, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{81, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{84, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{87, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{90, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{93, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{96, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{100, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{103, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{107, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{111, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{114, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{118, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{123, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{127, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{131, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{136, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{141, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{146, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{151, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{156, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{162, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{168, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{173, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{180, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{186, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{192, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{199, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{206, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{213, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{221, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{229, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{237, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{245, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{254, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{263, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{272, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{282, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{292, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{302, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{313, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{324, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.1875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.3125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.5*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.5625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.6875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.8125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{335, (int)(1.9375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.0625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.1875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.3125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.5*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.5625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.6875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.8125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{670, (int)(1.9375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.3125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.4375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.5*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.5625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.625*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.6875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.8125*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.875*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(1.9375*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.0*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.07*OV2710_MIN_D_GAIN*256), (int)(1.04*256)},
{1005, (int)(2.14*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.22*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.30*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.38*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.46*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.55*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.64*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.83*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(2.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.03*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.14*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.36*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.48*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.61*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(3.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(4.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(4.14*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(4.29*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(4.44*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(4.59*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(4.76*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(4.92*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(5.10*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(5.28*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(5.46*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(5.66*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(5.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(6.06*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(6.28*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(6.50*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(6.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(6.96*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(7.21*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(7.46*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(7.73*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(8.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(8.28*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(8.57*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(8.88*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(9.19*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(9.51*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(9.85*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(10.20*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(10.56*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(10.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(11.31*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(11.71*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(12.13*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(12.55*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(13.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(13.45*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(13.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(14.42*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(14.93*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(15.45*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(16.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(16.56*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(17.15*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(17.75*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(18.83*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(19.03*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(19.70*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(20.39*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
/*{1005, (int)(21.11*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(21.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(22.63*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(23.43*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(24.25*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(25.11*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(25.99*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(26.91*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(27.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(28.84*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(29.86*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(30.91*OV2710_MIN_D_GAIN*256), (int)(1.00*256)},
{1005, (int)(32.00*OV2710_MIN_D_GAIN*256), (int)(1.00*256)}*/

};

static sensor_fmt_t gOV2710FmtTable[] =
{
	{
		.desc		= "preview=1280*720",
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.bpp 		= 1,
		.mclk_src = MODE_MCLK_SRC_96M,
		.mclk = 24000000,
		.hpixel = 1280,
		.hoffset = 0,
		.vline = 720,
		.voffset = 0,
		.cdsp_calibration = 0,//&ov2710_cdsp_calibration,
	},
	{
		.desc		= "capture=1920*1082,crop=1920*1080",
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.bpp 		= 1,
		.mclk_src = MODE_MCLK_SRC_96M,
		.mclk = 27000000,
		.hpixel = 1920,
		.hoffset = 0,
		.vline = 1082,
		.voffset = 0,
		.cdsp_calibration = 0,//&ov2710_cdsp_calibration,
	},
	{
		.desc		= "record=1920*1080",
		.pixelformat = V4L2_PIX_FMT_SBGGR8,
		.bpp 		= 1,
		.mclk_src = MODE_MCLK_SRC_96M,
		.mclk = 27000000,
		.hpixel = 1920,
		.hoffset = 0,
		.vline = 1082,
		.voffset = 0,
		.cdsp_calibration = 0,//&ov2710_cdsp_calibration,
	},
	
};

#define C_SENSOR_FMT_MAX	sizeof(gOV2710FmtTable)/sizeof(sensor_fmt_t)

static mipi_config_t ov2710_mipi_setting = 
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
#if OV2710_MIPI_CLK_NO_STOP_EN == 1 
	.check_hs_seq = MIPI_CHECK_LP_00,	//for mipi clock no stop 
#else 
	.check_hs_seq = MIPI_CHECK_HS_SEQ,	//for mipi clock auto stop 
#endif
};

static sensor_config_t ov2710_config_table =
{
	.sensor_timing_mode = MODE_CCIR_601,
	.sensor_data_mode = MODE_DATA_YUV,
	.sensor_interlace_mode = MODE_NONE_INTERLACE,
	.sensor_pclk_mode = MODE_POSITIVE_EDGE,
	.sensor_hsync_mode = MODE_ACTIVE_HIGH,
	.sensor_vsync_mode = MODE_ACTIVE_LOW,
	.sensor_fmt_num = C_SENSOR_FMT_MAX,
	.fmt = gOV2710FmtTable,
	.mipi_config = &ov2710_mipi_setting,
};

/**************************************************************************
 *             Sensor Setting Table								          *
 **************************************************************************/
//@@ MIPI-10bit_1920x1080_30fps 
const regval16_t ov2710_Mipi_Raw10_1080P[] =
{
	{ 0x3103, 0x93 },            
	{ 0x3008, 0x82 },            
	{ 0x3017, 0x7f },            
	{ 0x3018, 0xfc },            
	{ 0x3706, 0x61 },            
	{ 0x3712, 0x0c },            
	{ 0x3630, 0x6d },            
	{ 0x3801, 0xb4 },            
	{ 0x3621, 0x04 },            
	{ 0x3604, 0x60 },            
	{ 0x3603, 0xa7 },            
	{ 0x3631, 0x26 },            
	{ 0x3600, 0x04 },            
	{ 0x3620, 0x37 },            
	{ 0x3623, 0x00 },            
	{ 0x3702, 0x9e },            
	{ 0x3703, 0x5c },            
	{ 0x3704, 0x40 },            
	{ 0x370d, 0x0f },            
	{ 0x3713, 0x9f },            
	{ 0x3714, 0x4c },            
	{ 0x3710, 0x9e },            
	{ 0x3801, 0xc4 },            
	{ 0x3605, 0x05 },            
	{ 0x3606, 0x3f },            
	{ 0x302d, 0x90 },            
	{ 0x370b, 0x40 },            
	{ 0x3716, 0x31 },            
	{ 0x3707, 0x52 },            
	{ 0x380d, 0x74 },            
	{ 0x5181, 0x20 },            
	{ 0x518f, 0x00 },            
	{ 0x4301, 0xff },            
	{ 0x4303, 0x00 },            
	{ 0x3a00, 0x78 },            
	{ 0x300f, 0x88 },            
	{ 0x3011, 0x28 },            
	{ 0x3a1a, 0x06 },            
	{ 0x3a18, 0x00 },            
	{ 0x3a19, 0x7a },            
	{ 0x3a13, 0x54 },            
	{ 0x382e, 0x0f },            
	{ 0x381a, 0x1a },            
	{ 0x401d, 0x02 },            
	{ 0x5688, 0x03 },            
	{ 0x5684, 0x07 },            
	{ 0x5685, 0xa0 },            
	{ 0x5686, 0x04 },            
	{ 0x5687, 0x43 },            
	{ 0x3011, 0x0a },            
	{ 0x300f, 0x8a },            
	{ 0x3017, 0x00 },            
	{ 0x3018, 0x00 },            
	{ 0x300e, 0x04 },
	{ 0x4801, 0x0f },            
	{ 0x300f, 0xc3 },            
	{ 0x3a0f, 0x40 },            
	{ 0x3a10, 0x38 },            
	{ 0x3a1b, 0x48 },            
	{ 0x3a1e, 0x30 },            
	{ 0x3a11, 0x90 },            
	{ 0x3a1f, 0x10 },            


	// The OV2710 sensor has an image array of 1952 columns by 1092 rows (2,131,584 pixels).
	
	//R1928x1088                 
	{ 0x3820, 0x0  },            
	{ 0x3821, 0x00 },            
	{ 0x381c, 0x00 },            
	{ 0x381d, 0x2  },            
	{ 0x381e, 0x4  },            
	{ 0x381f, 0x44 },            

	// horizontal start {0x3800, 0x3801}, HS[11:8] = 0x3800, HS[7:0] = 0x3801    
	{ 0x3800, 0x1  },        
	{ 0x3801, 0xC4 },           

	// horizontal width {0x3804, 0x3805} HW[11:8] = 0x3804, HW[7:0] = 0x3805
	{ 0x3804, 0x7  },            
	{ 0x3805, 0x88 },     

	// vertical starta (VS can only be an even number) {0x3802, 0x3803} VS[11:8] = 0x3802, VS[7:0] = 0x3803       
	{ 0x3802, 0x0  },     
	{ 0x3803, 0xA  },       


	// vertical height {0x3806, 0x3807} VH[11:8] = 0x3806, VH[7:0] = 0x3807
	{ 0x3806, 0x4  },            
	{ 0x3807, 0x40},

	// 0x350C AEC PK VTS 0x00 RW Bit[7:0]: AEC VTS[15:8]
	// 0x350D AEC PK VTS 0x00 RW Bit[7:0]: AEC VTS[7:0]
	//{ 0x350c, 0x4 },
	//{ 0x350d, 0x40},

	// exposure time
	// The exposure value in registers 0x3500~0x3502 is in units of line*16 - the low 4 bits (0x3502[3:0]) is
	// the fraction of line, the maximum value in 0x350C/0x350D is in unit of line.
	{ 0x3500, 0x00}, // Bit[7:4]: Not used, Bit[3:0]: AEC exposure[19:16]
	{ 0x3501, 0x1c}, // AEC exposure[15:8]
	{ 0x3502, 0x00}, // AEC exposure[7:0]


	// Gain = (0x350A[0]+1) กั (0x350B[7]+1) กั(0x350B[6]+1) กั (0x350B[5]+1) กั(0x350B[4]+1) กั (0x350B[3:0]/16+1)
	{ 0x350a, 0x0}, // Bit[0]: Gain high bit
	{ 0x350b, 0x08}, // Bit[7:0]: Gain low bits

	
	{ 0x3808, 0x7  },  // DVP output Width Hight byte          
	{ 0x3809, 0x88 }, // DVP output Width Low byte                     
	{ 0x380a, 0x4  },  // DVP output Height Hight byte                     
	{ 0x380b, 0x40 }, // DVP output Height Low byte                                
	{ 0x380c, 0x9  }, // HTS High byte           
	{ 0x380d, 0x74 },// HTS Low byte           
	{ 0x380e, 0x4  }, // VTS High byte
	{ 0x380f, 0x5E }, // VTS Low byte
	{ 0x3810, 0x8  },            
	{ 0x3811, 0x2  },            
	                             
	//{ 0x370d, 0x07 },            
	       
#if 1
	{ 0x3621, 0x14 },			 
	{ 0x3622, 0x8  },	  
	{ 0x3818, 0xE0 },          //flip 
	{ 0x3803, 0x09 },          //flip 
#else
	{ 0x3621, 0x04 },			 
	{ 0x3622, 0x8  },	  
	{ 0x3818, 0x80 },          // Mirror off, flip off  
#endif
	//{ 0x370d, 0x07 },            
	                             
	//AVG windows                
	{ 0x5688, 0x03 },            
	{ 0x5684, 0x07 },            
	{ 0x5685, 0x88 },            
	{ 0x5686, 0x04 },            
	{ 0x5687, 0x40 },            
	                             
	{ 0x3a08, 0x14 },            
	{ 0x3a09, 0xB3 },            
	{ 0x3a0a, 0x11 },            
	{ 0x3a0b, 0x40 },            
	{ 0x3a0e, 0x3  },            
	{ 0x3a0d, 0x4  },            
	{ 0x401c, 0x08 },
	
	//blc  ---------szk
	{ 0x4000, 0x05 },
	{ 0x4006, 0x00 },
	{ 0x4007, 0x20 },
	{ 0x401D, 0x22 },
	
#if 1 
	//off 3A                     
	{ 0x3503, 0x37 }, //;off aec
	{ 0x5001, 0x4e }, //;off awb 
	{ 0x5000, 0x5f }, //;off lenc
#else
	// 3A enable
	{ 0x3503, 0x00 }, 
	{ 0x5001, 0x4f }, 
	{ 0x5000, 0x5f }, 
#endif
#if OV2710_MIPI_CLK_NO_STOP_EN == 0 
	{ 0x4800, 0x24 }, //set mipi clock auto stop
#endif

	//mclk=27mhz pclk=81mhz, 30fps
	{0x3011, 0x09}, // default is 0x0a
	{0x380c, 0x09},
	{0x380d, 0x90}, // (default is 0x74)


	{ 0x0000, 0x00 }, //end
};

//@@ MIPI-10bit_1280x720_60fps or 30fps
const regval16_t ov2710_Mipi_Raw10_720P[] =
{
	{ 0x3103, 0x93 },
	{ 0x3008, 0x82 },
	{ 0x3017, 0x7f },
	{ 0x3018, 0xfc },
	{ 0x3706, 0x61 },
	{ 0x3712, 0x0c },
	{ 0x3630, 0x6d },
	{ 0x3801, 0xb4 },
	{ 0x3621, 0x04 },
	{ 0x3604, 0x60 },
	{ 0x3603, 0xa7 },
	{ 0x3631, 0x26 },
	{ 0x3600, 0x04 },
	{ 0x3620, 0x37 },
	{ 0x3623, 0x00 },
	{ 0x3702, 0x9e },
	{ 0x3703, 0x5c },
	{ 0x3704, 0x40 },
	{ 0x370d, 0x0f },
	{ 0x3713, 0x9f },
	{ 0x3714, 0x4c },
	{ 0x3710, 0x9e },
	{ 0x3801, 0xc4 },
	{ 0x3605, 0x05 },
	{ 0x3606, 0x3f },
	{ 0x302d, 0x90 },
	{ 0x370b, 0x40 },
	{ 0x3716, 0x31 },
	{ 0x3707, 0x52 },
	{ 0x380d, 0x74 },
	{ 0x5181, 0x20 },
	{ 0x518f, 0x00 },
	{ 0x4301, 0xff },
	{ 0x4303, 0x00 },
	{ 0x3a00, 0x78 },
	{ 0x300f, 0x88 },
	{ 0x3011, 0x28 },
	{ 0x3a1a, 0x06 },
	{ 0x3a18, 0x00 },
	{ 0x3a19, 0x7a },
	{ 0x3a13, 0x54 },
	{ 0x382e, 0x0f },
	{ 0x381a, 0x1a },
	{ 0x401d, 0x02 },
	{ 0x381c, 0x10 },
	{ 0x381d, 0xb8 },
	{ 0x381e, 0x02 },
	{ 0x381f, 0xdc },
	{ 0x3820, 0x0a },
	{ 0x3821, 0x29 },
	{ 0x3804, 0x05 },
	{ 0x3805, 0x00 },
	{ 0x3806, 0x02 },
	{ 0x3807, 0xd0 },
	{ 0x3808, 0x05 },
	{ 0x3809, 0x00 },
	{ 0x380a, 0x02 },
	{ 0x380b, 0xd0 },
	{ 0x380e, 0x02 },
	{ 0x380f, 0xe8 },
	{ 0x380c, 0x07 },
	{ 0x380d, 0x00 },
	{ 0x5688, 0x03 },
	{ 0x5684, 0x05 },
	{ 0x5685, 0x00 },
	{ 0x5686, 0x02 },
	{ 0x5687, 0xd0 },
	{ 0x3a08, 0x1b },
	{ 0x3a09, 0xe6 },
	{ 0x3a0a, 0x17 },
	{ 0x3a0b, 0x40 },
	{ 0x3a0e, 0x01 },
	{ 0x3a0d, 0x02 },
	{ 0x3011, 0x0a },
	{ 0x300f, 0x8a },
	{ 0x3017, 0x00 },
	{ 0x3018, 0x00 },
	{ 0x300e, 0x04 },
	{ 0x4801, 0x0f },
	{ 0x300f, 0xc3 },
	{ 0x3a0f, 0x40 },
	{ 0x3a10, 0x38 },
	{ 0x3a1b, 0x48 },
	{ 0x3a1e, 0x30 },
	{ 0x3a11, 0x90 },
	{ 0x3a1f, 0x10 },

#if 1	//;30FPS
	{ 0x3615, 0xf0 },
	{ 0x3010, 0x10 },
#endif

#if 1 //;off 3A
	{ 0x3503, 0x07 }, //;off aec/agc
	{ 0x5001, 0x4e }, //;off awb
	{ 0x5000, 0x5f }, //;off lenc
#endif
#if OV2710_MIPI_CLK_NO_STOP_EN == 0 
	{ 0x4800, 0x24 }, //set mipi clock auto stop
#endif
	{ 0x0000, 0x00 }, //end
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
	g_ti2c_handle.pDeviceString = "OV2710_MIPI";		
	g_ti2c_handle.slaveAddrMode = TI2C_NORMAL_SLAVEADDR_8BITS;		
	g_ti2c_handle.slaveAddr = slave_id;		
	g_ti2c_handle.clockRate = scl_speed;
	if (gp_ti2c_bus_request(&g_ti2c_handle) != 0) {
		printk("OV2710_MIPI ti2c request failed\n");
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
ov2710_read(
	unsigned short reg, 
	unsigned char *value
)
{
#if (I2C_MODE == HW_I2C)
	char addr[2], data[0];
	int nRet;
	
	addr[0] = (reg >> 8) & 0xFF;
	addr[1] = reg & 0xFF;
	nRet = gp_i2c_bus_write(g_i2c_handle, addr, 2);
	if(nRet <= 0) {
		return nRet;
	}
	
	nRet = gp_i2c_bus_read(g_i2c_handle, data, 1);
	*value = data[0];
	return nRet;
	
#elif (I2C_MODE == HW_TI2C)
	unsigned char addr[2], data[1];
	int nRet;
	int retry = 0;
	
	addr[0] = (reg >> 8) & 0xFF;
	addr[1] = reg & 0xFF;	
	g_ti2c_handle.transmitMode = TI2C_NORMAL_WRITE_MODE;	
	g_ti2c_handle.pBuf = addr;	
	g_ti2c_handle.dataCnt = 2;	
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
ov2710_write(
	unsigned short reg, 
	unsigned char value
)
{
#if (I2C_MODE == HW_I2C)
	char data[3];
	
	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;	
	return gp_i2c_bus_write(g_i2c_handle, data, 3);
	
#elif (I2C_MODE == HW_TI2C)
	unsigned char data[3];
	int ret, retry = 0;

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;	
	g_ti2c_handle.transmitMode = TI2C_NORMAL_WRITE_MODE;	
	g_ti2c_handle.pBuf = data;	
	g_ti2c_handle.dataCnt = 3;		
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


static int OV2710_get_plck(void)
{
	int ret;
	int pclk;
	unsigned char tmp;
	int val, t1, t2, t3, t4, t5;
	int mclk = gOV2710FmtTable[1].mclk;
	

	ret = ov2710_read(0x300f, &tmp);
	if(ret < 0) return -1;
	

	val = tmp & 0x03;
	if(val <= 1 ) t1 = 1;
	else if(val == 2) t1 = 4;
	else t1 = 5;

	val = (tmp >> 6) & 0x03;
	if(val <= 1 ) t5 = 1*2;
	else if(val == 2) t5 = 2*2;
	else t5 = (int)(2.5*2);

	

	ret = ov2710_read(0x3010, &tmp);
	if(ret < 0) return -1;
	
	t4 = (tmp >> 4) & 0x0f;
	t4 += 1;

	
	
	ret = ov2710_read(0x3011, &tmp);
	if(ret < 0) return -1;
	
	t2 = tmp & 0x1f;

	

	ret = ov2710_read(0x3012, &tmp);
	if(ret < 0) return -1;
	

	val = tmp & 0x07;
	switch(val)
	{
		case 0: t3 = 1 * 2; break;
		case 1: t3 = (int)(1.5 * 2); break;
		case 2: t3 = 2 * 2; break;
		case 3: t3 = (int)(2.5 * 2); break;
		case 4: t3 = 3 * 2; break;
		case 5: t3 = 4 * 2; break;
		case 6: t3 = 6 * 2; break;
		case 7: t3 = 8 * 2; break;
	}
	

	pclk = mclk * t1 * t2 / t3;
	pclk /= t4;
	pclk /= t5;

	return pclk;
}

static int
OV2710WrTable(
	regval16_t *pTable
)
{
	int ret = 0;

	while(1) {
		if(pTable->reg_num == 0 && pTable->value == 0) {
			break;
		}
		
		ret = ov2710_write(pTable->reg_num, pTable->value);
		if(ret < 0) {
			return -1;
		}
		
		//printk("[0x%x] = 0x%x\n", pTable->reg_num, pTable->value);
		pTable ++;
	}

	printk("**** OV2710 PCLK = %d ****\r\n", OV2710_get_plck());
	
	return ret;
}

static int 
ov2710_init(
	struct v4l2_subdev *sd,
	u32 val
)
{
	ov2710_seInfo.time = 0x10;
	ov2710_seInfo.analog_gain = OV2710_MIN_ANALOG_GAIN;
	ov2710_seInfo.digital_gain = OV2710_MIN_DIGITAL_GAIN;

	ov2710_seInfo.max_time = OV2710_MAX_EXPOSURE_TIME;
	ov2710_seInfo.max_analog_gain = OV2710_MAX_ANALOG_GAIN;
	ov2710_seInfo.max_digital_gain = OV2710_MAX_DIGITAL_GAIN;


	ov2710_seInfo.min_time = OV2710_MIN_EXPOSURE_TIME;
	ov2710_seInfo.min_analog_gain = OV2710_MIN_ANALOG_GAIN;
	ov2710_seInfo.min_digital_gain = OV2710_MIN_DIGITAL_GAIN;

	ov2710_seInfo.userISO = DISABLE;
	ov2710_seInfo.sensor_ev_idx = OV2710_30FPS_50HZ_INIT_EV_IDX;
	ov2710_seInfo.ae_ev_idx = 0;
	ov2710_seInfo.daylight_ev_idx= OV2710_30FPS_50HZ_DAY_EV_IDX;
	ov2710_seInfo.night_ev_idx= OV2710_30FPS_50HZ_NIGHT_EV_IDX;
	ov2710_seInfo.max_ev_idx = OV2710_30FPS_50HZ_MAX_EV_IDX;
	p_expTime_table = ov2710_30fps_exp_time_gain_50Hz;
	sensor_fps = V4L2_TC_TYPE_30FPS;
	sensor_total_ev = OV2710_30FPS_50HZ_EXP_TIME_TOTAL;
	sensor_swith_time = 2000;
	current_frame_rate= 30;

	
	printk("%s\n", __FUNCTION__);
	return 0;
}

static int 
ov2710_preview(
	void
)
{
	printk("%s\n", __FUNCTION__);
	return OV2710WrTable((regval16_t *)ov2710_Mipi_Raw10_720P);
}

static int 
ov2710_capture(
	void
)
{
	printk("%s\n", __FUNCTION__);
	return OV2710WrTable((regval16_t *)ov2710_Mipi_Raw10_1080P);
}

static int 
ov2710_record(
	void
)
{
	printk("%s\n", __FUNCTION__);
	return OV2710WrTable((regval16_t *)ov2710_Mipi_Raw10_1080P);
}

static int 
ov2710_reset(
	struct v4l2_subdev *sd, 
	u32 val
)
{
	return 0;
}


static int ov2710_get_real_agc_gain(int agc_gain)
{
	int real_agc_gain;

	real_agc_gain = 0x10 + (agc_gain & 0x0f);
	real_agc_gain = real_agc_gain * (1 + ((agc_gain >> 4) & 1)) * (1 + ((agc_gain >> 5) & 1))
			* (1 + ((agc_gain >> 6) & 1)) * (1 + ((agc_gain >> 7) & 1)) * (1 + ((agc_gain >> 8) & 1));
	
	return real_agc_gain;
}


static int ov2710_cvt_agc_gain(int agc_gain)
{
	int ov2710_agc_gain, i;

	ov2710_agc_gain = 0;
	i = 5;
	do {
		if(agc_gain <= 0x1f) 
			break;
		
		agc_gain >>= 1;
		ov2710_agc_gain <<= 1;
		ov2710_agc_gain |= 0x10;
		
		i--;
	} while(i != 0);

	agc_gain -= 0x10;
	if(agc_gain < 0) agc_gain = 0;
	ov2710_agc_gain += agc_gain;
	
	return ov2710_agc_gain;
}



static int ov2710_set_exposure_time(sensor_exposure_t *se)
{
	unsigned char t1, t2;
	int temp;


	temp = se->analog_gain >> 4;
	temp = ov2710_cvt_agc_gain(temp);
	t1 = temp & 0x00ff;
	t2 = (temp >> 8) & 0x01;

	ov2710_write(0x3212, 0x0); // Group hold enable 

	// Gain = (0x350A[0]+1) กั (0x350B[7]+1) กั(0x350B[6]+1) กั (0x350B[5]+1) กั(0x350B[4]+1) กั (0x350B[3:0]/16+1)
	ov2710_write(0x350b, t1);
	ov2710_write(0x350a, t2);


	// exposure time	
	//printk("%s: time = 0x%x, analog gain = 0x%x\n", __FUNCTION__, se->time, se->analog_gain);
	temp = se->time;
	t1 = (temp & 0x0f) << 4;
	t2 = (temp >> 4) & 0x00ff;
	ov2710_write(0x3502, t1);
	ov2710_write(0x3501, t2);
	t1 =  (temp >> 12) & 0x000f;
	ov2710_write(0x3500, t1);

	ov2710_write(0x3212, 0x10); //Group latch end
	ov2710_write(0x3212, 0xa0); // Group Latch Launch

	return 0;
}


static int ov2710_set_xfps_exposure_time(sensor_exposure_t *si)
{
	unsigned char t1, t2;
	int idx, temp, ret;


	si->sensor_ev_idx += si->ae_ev_idx;
	if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
	if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;

	idx = si->sensor_ev_idx * 3;
	si ->time = p_expTime_table[idx];
	si ->analog_gain = p_expTime_table[idx+1];
	si ->digital_gain = p_expTime_table[idx+2];


	temp = si->analog_gain >> 4;
	temp = ov2710_cvt_agc_gain(temp);
	t1 = temp & 0x00ff;
	t2 = (temp >> 8) & 0x01;

	//printk("EV[%d]: time = %d, analog gain =0x%x, cvt_gain = 0x%x\n", si->sensor_ev_idx, si->time, si->analog_gain>>4, temp);

	ret = ov2710_write(0x3212, 0x0); // Group hold enable 
	if(ret < 0) return ret;

	// Gain = (0x350A[0]+1) กั (0x350B[7]+1) กั(0x350B[6]+1) กั (0x350B[5]+1) กั(0x350B[4]+1) กั (0x350B[3:0]/16+1)
	ret = ov2710_write(0x350b, t1);
	if(ret < 0) return ret;
	ret = ov2710_write(0x350a, t2);
	if(ret < 0) return ret;


	// exposure time		
	temp = si->time;
	t1 = (temp & 0x0f) << 4;
	t2 = (temp >> 4) & 0x00ff;
	ret = ov2710_write(0x3502, t1);
	if(ret < 0) return ret;
	ret = ov2710_write(0x3501, t2);
	if(ret < 0) return ret;
	t1 =  (temp >> 12) & 0x000f;
	ret = ov2710_write(0x3500, t1);
	if(ret < 0) return ret;

	ret = ov2710_write(0x3212, 0x10); //Group latch end
	if(ret < 0) return ret;
	ret = ov2710_write(0x3212, 0xa0); // Group Latch Launch
	if(ret < 0) return ret;

	return 0;
}



static int ov2710_get_exposure_time(sensor_exposure_t *se)
{
	unsigned char t1, t2;
	int temp;
	int ret;

	ret = ov2710_read(0x3500, &t1);
	if(ret < 0) return ret;
	ret = ov2710_read(0x3501, &t2);
	if(ret < 0) return ret;
	temp = (t1 & 0x0f) << 12;
	temp = temp | ((t2 & 0x00ff) << 4);
	ret = ov2710_read(0x3502, &t1);
	if(ret < 0) return ret;
	se->time = temp | ((t1  & 0x00F0) >> 4);


	ret = ov2710_read(0x350b, &t1);
	if(ret < 0) return ret;
	ret = ov2710_read(0x350a, &t2);
	if(ret < 0) return ret;
	
	temp = (t1 & 0x00ff) | ((t2 & 0x01) << 8);
	temp = ov2710_get_real_agc_gain(temp);
	se->analog_gain = temp << 4;

	// Get Yavg
	ret = ov2710_read(0x5690, &t1);
	if(ret < 0) return ret;
	se->digital_gain = t1;

	//printk("%s: time = 0x%x, analog gain = 0x%x\n", __FUNCTION__, se->time, se->analog_gain);
	
	return 0;
}



static int 
ov2710_queryctrl(
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
ov2710_g_ctrl(
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
			ov2710_get_exposure_time(&ov2710_seInfo);
		}
		ctrl->value = (int)&ov2710_seInfo;
		break;
		
	case V4L2_CID_USER_CLASS:
		// for Comi testing
		ov2710_get_exposure_time(&ov2710_seInfo);
		ctrl->value = (int)&ov2710_seInfo;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int 
ov2710_s_ctrl(
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
		
		ov2710_seInfo.userISO = si->userISO;
		if(p_expTime_table != 0)
		{
			ov2710_seInfo.ae_ev_idx = si->ae_ev_idx;
			nRet = ov2710_set_xfps_exposure_time(&ov2710_seInfo);
		}
		else
		{
			ov2710_seInfo.time = si->time;
			ov2710_seInfo.analog_gain = si->analog_gain;
			nRet = ov2710_set_exposure_time(&ov2710_seInfo);
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
					p_expTime_table = ov2710_30fps_exp_time_gain_50Hz;		
					
					ov2710_seInfo.ae_ev_idx = 0;
					ov2710_seInfo.daylight_ev_idx= OV2710_30FPS_50HZ_DAY_EV_IDX;
					ov2710_seInfo.night_ev_idx= OV2710_30FPS_50HZ_NIGHT_EV_IDX;
					ov2710_seInfo.max_ev_idx= OV2710_30FPS_50HZ_MAX_EV_IDX;
					sensor_total_ev = OV2710_30FPS_50HZ_EXP_TIME_TOTAL;
					
					if(ov2710_seInfo.sensor_ev_idx > ov2710_seInfo.max_ev_idx) ov2710_seInfo.sensor_ev_idx = ov2710_seInfo.max_ev_idx;
					
				}
				else if(line_freq == V4L2_CID_POWER_LINE_FREQUENCY_60HZ)
				{
					p_expTime_table = ov2710_30fps_exp_time_gain_60Hz;
					
					ov2710_seInfo.sensor_ev_idx = OV2710_30FPS_60HZ_INIT_EV_IDX;
					ov2710_seInfo.ae_ev_idx = 0;
					ov2710_seInfo.daylight_ev_idx= OV2710_30FPS_60HZ_DAY_EV_IDX;
					ov2710_seInfo.night_ev_idx= OV2710_30FPS_60HZ_NIGHT_EV_IDX;
					ov2710_seInfo.max_ev_idx= OV2710_30FPS_50HZ_MAX_EV_IDX;
					sensor_total_ev = OV2710_30FPS_60HZ_EXP_TIME_TOTAL;
					if(ov2710_seInfo.sensor_ev_idx > ov2710_seInfo.max_ev_idx) ov2710_seInfo.sensor_ev_idx = ov2710_seInfo.max_ev_idx;
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
ov2710_querystd(
	struct v4l2_subdev *sd,
	v4l2_std_id *std
)
{
	return 0;
}

static int 
ov2710_enum_fmt(
	struct v4l2_subdev *sd, 
	struct v4l2_fmtdesc *fmtdesc
)
{
	printk("%s\n", __FUNCTION__);
	if(fmtdesc->index >= C_SENSOR_FMT_MAX)
		return -EINVAL;

	fmtdesc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	memcpy((void *)fmtdesc->description, (void *)gOV2710FmtTable[fmtdesc->index].desc, 32);
	fmtdesc->pixelformat = gOV2710FmtTable[fmtdesc->index].pixelformat;
	return 0;
}

static int 
ov2710_g_fmt(
	struct v4l2_subdev *sd, 
	struct v4l2_format *fmt
)
{
	printk("%s\n", __FUNCTION__);
	fmt->fmt.pix.width = gOV2710Dev.fmt->hpixel;
	fmt->fmt.pix.height = gOV2710Dev.fmt->vline;
	fmt->fmt.pix.pixelformat = gOV2710Dev.fmt->pixelformat;
	fmt->fmt.pix.field = V4L2_FIELD_NONE;
	fmt->fmt.pix.bytesperline = gOV2710Dev.fmt->hpixel * gOV2710Dev.fmt->bpp;
	fmt->fmt.pix.sizeimage = fmt->fmt.pix.bytesperline * gOV2710Dev.fmt->vline;

	return 0;
}

static int 
ov2710_try_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_format *fmt
)
{
	return 0;
}

static int 
ov2710_s_fmt(
	struct v4l2_subdev *sd, 
	struct v4l2_format *fmt
)
{
	int ret;

	printk("%s = %d\n", __FUNCTION__, fmt->fmt.pix.priv);
	switch(fmt->fmt.pix.priv)
	{
	case 0: 
		ret = ov2710_preview();
		break;

	case 1: 
		ret = ov2710_capture();
		break;

	case 2: 
		ret = ov2710_record();
		break;	

	default:
		ret = -1;
	}

	gOV2710Dev.fmt = &gOV2710FmtTable[fmt->fmt.pix.priv];
	return ret;
}

static int 
ov2710_cropcap(
	struct v4l2_subdev *sd,
	struct v4l2_cropcap *cc
)
{
	return 0;
}

static int 
ov2710_g_crop(
	struct v4l2_subdev *sd,
	struct v4l2_crop *crop
)
{
	return 0;
}

static int 
ov2710_s_crop(
	struct v4l2_subdev *sd,
	struct v4l2_crop *crop
)
{
	return 0;
}

static int 
ov2710_g_parm(
	struct v4l2_subdev *sd,
	struct v4l2_streamparm *parms
)
{
	return 0;
}

static int 
ov2710_s_parm(
	struct v4l2_subdev *sd,
	struct v4l2_streamparm *param
)
{
	return 0;
}

static int 
ov2710_s_interface(
	struct v4l2_subdev *sd,
	struct v4l2_interface *interface
)
{
	return 0;
}

static int 
ov2710_suspend(
	struct v4l2_subdev *sd
)
{
	return 0;
}

static int 
ov2710_resume(
	struct v4l2_subdev *sd
)
{
	return 0;
}

static const struct v4l2_subdev_core_ops ov2710_core_ops = 
{
	.init = ov2710_init,
	.reset = ov2710_reset,
	.queryctrl = ov2710_queryctrl,
	.g_ctrl = ov2710_g_ctrl,
	.s_ctrl = ov2710_s_ctrl,
};

static const struct v4l2_subdev_video_ops ov2710_video_ops = 
{
	.querystd = ov2710_querystd,
	.enum_fmt = ov2710_enum_fmt,
	.g_fmt = ov2710_g_fmt,
	.try_fmt = ov2710_try_fmt,
	.s_fmt = ov2710_s_fmt,
	.cropcap = ov2710_cropcap,
	.g_crop = ov2710_g_crop,
	.s_crop = ov2710_s_crop,
	.g_parm = ov2710_g_parm,
	.s_parm = ov2710_s_parm,
};

static const struct v4l2_subdev_ext_ops ov2710_ext_ops = 
{
	.s_interface = ov2710_s_interface,
	.suspend = ov2710_suspend,
	.resume = ov2710_resume,
};

static const struct v4l2_subdev_ops ov2710_ops = 
{
	.core = &ov2710_core_ops,
	.video = &ov2710_video_ops,
	.ext = &ov2710_ext_ops
};

static int __init 
ov2710_module_init(
		void
)
{
	if(sensor_i2c_open(OV2710_ID, 50) < 0) {
		printk(KERN_WARNING "i2cReqFail\n");
		return -1;
	}

	printk(KERN_WARNING "ModuleInit: ov2710 mipi\n");
	gOV2710Dev.fmt = &gOV2710FmtTable[0];
	v4l2_subdev_init(&(gOV2710Dev.sd), &ov2710_ops);
	strcpy(gOV2710Dev.sd.name, "sensor_ov2710_mipi");
	register_sensor(&gOV2710Dev.sd, (int *)&param[0], &ov2710_config_table);
	return 0;
}

static void __exit
ov2710_module_exit(
		void
)
{
	sensor_i2c_close();
	unregister_sensor(&(gOV2710Dev.sd));
}

module_init(ov2710_module_init);
module_exit(ov2710_module_exit);

/**************************************************************************
 *                  M O D U L E    D E C L A R A T I O N                  *
 **************************************************************************/
MODULE_AUTHOR("Generalplus");
MODULE_DESCRIPTION("Generalplus ov2710 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");


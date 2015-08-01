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
#include <mach/kernel.h>
#include <mach/module.h>
#include <mach/cdev.h>
#include <mach/diag.h>
#include <mach/hardware.h>
#include <linux/delay.h>

#include <mach/hal/hal_i2c_bus.h>
#include <mach/gp_i2c_bus.h>
#include <mach/gp_board.h>

//----------------------------------
#include <linux/init.h>
#include <linux/fs.h> 
#include <mach/gp_display.h>
#include <mach/general.h>

#include <mach/gp_gpio.h>
#include <linux/delay.h> 
#include <mach/hal/hal_gpio.h>
#include <linux/interrupt.h> 


#if (defined CONFIG_ARCH_GPL32900B)
#include <mach/gp_ti2c_bus.h>
#endif
#define TI2C_MODE 0
#define I2C_MODE 1
#define I2C_DEV  I2C_MODE
#define Z_INVERSE 0//1// z inverse

#define VIEW_ANGLE_ZVALUE_CAR 135 
#define VIEW_ANGLE_ZVALUE_SUV 134	
#define VIEW_ANGLE_ZVALUE_TRUCK 133
/*************************************************
MAC define
*************************************************/
#define ERROR(fmt, arg...) printk( "[%s:%d] Error! "fmt, __FUNCTION__, __LINE__, ##arg)
#define RETURN(x)		{ret = x; goto Return;}
//#define CHECK_(x, msg, errid) if(!(x)) {/*ERROR("%s, %s\n", msg, #x)i*/; RETURN(errid);}
#define CHECK_(x, msg, errid) if(!(x)) {RETURN(errid);}
#define CHECK(x)		CHECK_(x, "Check failed", -1)
#define CHECK_PRG(x)	CHECK_(x, "Program Error", -EIO)
#define CHECK_VAL(x)	CHECK_(x, "Value Error", -1)



#define ASENSOR_IOCTL_ID	'G'
#define ASENSOR_IOCTL_GET_INT_ACTIVE	_IOR(ASENSOR_IOCTL_ID, 0x01, int)//获取是否碰撞，如果碰撞，则录影文件进行lock动作
#define ASENSOR_IOCTL_SET_SENSITIVE	_IOW(ASENSOR_IOCTL_ID, 0x02, int)//设置碰撞锁文件的灵敏度等级。碰撞是通过读取两次gvalue对比是否大于某个阀值的判定方式
#define ASENSOR_IOCTL_PARK_MODE		_IOW(ASENSOR_IOCTL_ID, 0x04, int)//设置parking mode，并设置parking mode等级。
#define ASENSOR_IOCTL_PARK_MODE_INT	_IOR(ASENSOR_IOCTL_ID, 0x08, int)//获取是否是parking mode 导致的开机。
#define ASENSOR_IOCTL_GET_G_VALUE	_IOR(ASENSOR_IOCTL_ID, 0x10, int)//LDW获取加速度数据
//以下为adas calibrate使用到
#define ASENSOR_IOCTL_SET_INIT_OFFSET	_IOW(ASENSOR_IOCTL_ID, 0x20, int)//设置初始gsnsor的偏移，并开启timer
#define ASENSOR_IOCTL_GET_INIT_OFFSET	_IOR(ASENSOR_IOCTL_ID, 0x40, int)//获取初始gsnsor的偏移，防止并发，会关闭timer
#define ASENSOR_IOCTL_CALIBRATE_ENABLE		_IOW(ASENSOR_IOCTL_ID, 0x82, int)//准备进行倾斜角度校正，这时传给ldw的gvalue =0.
#define ASENSOR_IOCTL_CALIBRATE_ONOFF	    _IOW(ASENSOR_IOCTL_ID, 0x80, int)//根据车型设置不同的倾斜角度，开始进行校正
#define ASENSOR_IOCTL_GET_CALIBRATE_OFFSET	_IOW(ASENSOR_IOCTL_ID, 0x81, int)//timer进行读取10次求均值的方式产生偏移数据,传给UI使用

#define ASENSOR_IOCTL_SET_INIT_LEVEL	_IOR(ASENSOR_IOCTL_ID, 0x84, int)

#define DA380_SLAVE_ID 0x4E
#define A_SENSOR_ADDR DA380_SLAVE_ID
#define A_SENSOR_CLK 300

#if 0
//working ,when read xyz>threshold,lock file
#define DMT_THRSHOLD_HIGH 	800
#define DMT_THRSHOLD_MID 	1500
#define DMT_THRSHOLD_LOW 	1900

//when sleep,set resloution must 14bit +-8g [if set +-16g,int not work --from DA380 FAE ]
#define DMT_SENSITIVE_P_LOW	  (0x80)
#define DMT_SENSITIVE_P_MID	  (0x40)
#define DMT_SENSITIVE_P_HIGH  (0x18)

#endif
//working ,when read xyz>threshold,lock file   logic 1=32g/(2^14)=1/512
#define DMT_THRSHOLD_HIGH 	0x100//0.5g
#define DMT_THRSHOLD_MID 	0x400//2g
#define DMT_THRSHOLD_LOW 	0x800//4g

//when sleep,set resloution must 14bit +-8g [if set +-16g,int not work --from DA380 FAE ]
#define DMT_SENSITIVE_P_LOW	  (0x20)//2g
#define DMT_SENSITIVE_P_MID	  (0x10)//1g
#define DMT_SENSITIVE_P_HIGH  (0x08)//0.5g


static short x_init=0,y_init=0,z_init=0;//record init xyzout value  
static short gvalus_read[6];//备份2次数据，ldw需要时为之前两次数据的平均值减去初始offset
static int parking_mode_pwron =0;
static short gsensor_calibrate_z_read[10];//保存10次数据，然后排序去掉最大最小值，剩余8个数据求平均
static short gsensor_calibrate_z_off;//
static char read_cnt=0;//muti read ，average ，then know how length  读取多次求平均值以便知道偏转箭头该多长
static unsigned char ViewAngle_Zvalue;//according different car type ,set different ViewAngle[gvalue] 
static int gsensor_calibrate_enable=0;//calibrate enable,if 0,ldw get gvalue =0
static char b_is_first = 0;//是否是第一次读取数据
static int gsensor_calibrate_flag=0;

int da380_error = 0; //0: no error, >0: error count  <0: init error;
int IntFlag = 0;

//========================DA380 inter face ============================
                                               
#define NSA_REG_SPI_I2C                 0x00
#define NSA_REG_WHO_AM_I                0x01
#define NSA_REG_ACC_X_LSB               0x02
#define NSA_REG_ACC_X_MSB               0x03
#define NSA_REG_ACC_Y_LSB               0x04
#define NSA_REG_ACC_Y_MSB               0x05
#define NSA_REG_ACC_Z_LSB               0x06
#define NSA_REG_ACC_Z_MSB               0x07
#define NSA_REG_MOTION_FLAG				0x09
#define NSA_REG_G_RANGE                 0x0f
//0x0f: Resolution bit[3:2] -- 00:14bit 
//                        01:12bit 
//                        10:10bit
//                        11:8bit 

// FS bit[1:0]         -- 00:+/-2g 
//                        01:+/-4g 
//                        10:+/-8g
//                        11:+/-16g 

#define NSA_REG_ODR_AXIS_DISABLE        0x10
#define NSA_REG_POWERMODE_BW            0x11
#define NSA_REG_SWAP_POLARITY           0x12
#define NSA_REG_FIFO_CTRL               0x14
#define NSA_REG_INTERRUPT_SETTINGS1     0x16
#define NSA_REG_INTERRUPT_SETTINGS2     0x17
#define NSA_REG_INTERRUPT_MAPPING1      0x19
#define NSA_REG_INTERRUPT_MAPPING2      0x1a
#define NSA_REG_INTERRUPT_MAPPING3      0x1b
#define NSA_REG_INT_PIN_CONFIG          0x20
#define NSA_REG_INT_LATCH               0x21
#define NSA_REG_ACTIVE_DURATION         0x27
#define NSA_REG_ACTIVE_THRESHOLD        0x28
#define NSA_REG_TAP_DURATION            0x2A
#define NSA_REG_TAP_THRESHOLD           0x2B
#define NSA_REG_CUSTOM_OFFSET_X         0x38
#define NSA_REG_CUSTOM_OFFSET_Y         0x39
#define NSA_REG_CUSTOM_OFFSET_Z         0x3a
#define NSA_REG_ENGINEERING_MODE        0x7f
#define NSA_REG_SENSITIVITY_TRIM_X      0x80
#define NSA_REG_SENSITIVITY_TRIM_Y      0x81
#define NSA_REG_SENSITIVITY_TRIM_Z      0x82
#define NSA_REG_COARSE_OFFSET_TRIM_X    0x83
#define NSA_REG_COARSE_OFFSET_TRIM_Y    0x84
#define NSA_REG_COARSE_OFFSET_TRIM_Z    0x85
#define NSA_REG_FINE_OFFSET_TRIM_X      0x86
#define NSA_REG_FINE_OFFSET_TRIM_Y      0x87
#define NSA_REG_FINE_OFFSET_TRIM_Z      0x88
#define NSA_REG_SENS_COMP               0x8c
#define NSA_REG_SENS_COARSE_TRIM        0xd1


static int A_sensor_write(unsigned char reg,unsigned char data);
static int A_sensor_read(unsigned char reg);
static int A_sensor_read_buff(unsigned char reg, unsigned char *data, unsigned int len);
int cycle_read_xyz(short *x,short *y, short *z, int ns);
static void gsensor_xyz_read(unsigned long arg);

	

/*return value: 0: is ok    other: is failed*/
int     i2c_read_byte_data( unsigned char addr, unsigned char *data){
	
		int ret = 0;
		short pdata = 0;
		
		*data = A_sensor_read(addr); 
		
		//*data = (unsigned char)pdata;
		
		return ret;
}

/*return value: 0: is ok    other: is failed*/
int     i2c_write_byte_data( unsigned char addr, unsigned char data){
		
		int ret = 0;
		
		A_sensor_write(addr,data);
	
		return 0;
}

/*return value: 0: is count    other: is failed*/
int     i2c_read_block_data( unsigned char base_addr, unsigned char count, unsigned char *data){
	int i = 0;
		
	for(i = 0; i < count;i++)
	{
				if(i2c_read_byte_data(base_addr+i,(data+i)))
				{
					return -1;		
				}
	}	
		
	return count;
}

int mir3da_register_read( unsigned char addr, unsigned char *data){
    int     res = 0;

    res = i2c_read_byte_data(addr, data);
    if(res != 0) {
          return res;
    }	

    return res;
}

int mir3da_register_write( unsigned char addr, unsigned char data){
    int     res = 0;

    res = i2c_write_byte_data(addr, data);
    if(res != 0) {
         return res;
    }

    return res;
}

int mir3da_register_read_continuously( unsigned char addr, unsigned char count, unsigned char *data)
{
    int     res = 0;

    res = (count==i2c_read_block_data(addr, count, data)) ? 0 : 1;
    if(res != 0) {
         return res;
    }

    return res;
}

int mir3da_register_mask_write(unsigned char addr, unsigned char mask, unsigned char data){
    int     res = 0;
    unsigned char      tmp_data;

    res = mir3da_register_read(addr, &tmp_data);
    if(res) {
        return res;
    }

    tmp_data &= ~mask; 
    tmp_data |= data & mask;
    res = mir3da_register_write(addr, tmp_data);

    return res;
}

/*return value: 0: is ok    other: is failed*/
char mir3da_init(void){
	int res;
	unsigned char data=0;

	mir3da_register_read(NSA_REG_WHO_AM_I,&data);
	if(data != 0x13){
	    printk("------DA380 read chip id  error= %x-----\r\n",data); 
		da380_error = -1;
		return 1;
	}

	printk("------DA380 chip id = %x-----\r\n",data); 

	res =  mir3da_register_mask_write(NSA_REG_SPI_I2C, 0x24, 0x24);

	mdelay(5);

	res |= mir3da_register_mask_write(NSA_REG_G_RANGE, 0x03, 0x03);
	res |= mir3da_register_mask_write(NSA_REG_POWERMODE_BW, 0xFF, 0x1E);
	res |= mir3da_register_mask_write(NSA_REG_ODR_AXIS_DISABLE, 0xFF, 0x07);
	
	res |= mir3da_register_mask_write(NSA_REG_INT_PIN_CONFIG, 0x0F, 0x00);//set int_pin level
	res |= mir3da_register_mask_write(NSA_REG_INT_LATCH, 0x8F, 0x86);//clear latch and set latch mode
	
	res |= mir3da_register_mask_write(NSA_REG_ENGINEERING_MODE, 0xFF, 0x83);
	res |= mir3da_register_mask_write(NSA_REG_ENGINEERING_MODE, 0xFF, 0x69);
	res |= mir3da_register_mask_write(NSA_REG_ENGINEERING_MODE, 0xFF, 0xBD);
	res |= mir3da_register_mask_write(NSA_REG_SWAP_POLARITY, 0xFF, 0x00);

	cycle_read_xyz(&x_init,&y_init, &z_init, 1);//read 1，for test
	printk("==========org DA380 init read1========[%d.%d.%d]",x_init,y_init,z_init);
	cycle_read_xyz(&x_init,&y_init, &z_init, 8);
	printk("==========org DA380 init aver8========[%d.%d.%d]\n\n\n",x_init,y_init,z_init);
	x_init -= 2*(127-128);//x初始偏移变为127，
	z_init -= 2*(135-128);//z初始偏移变为135	
	memset(gsensor_calibrate_z_read,0,sizeof(short)*10);
	return res;	    	
}

int mir3da_set_enable(char enable)
{
		int res = 0;
		if(enable)
		res = mir3da_register_mask_write(NSA_REG_POWERMODE_BW,0xC0,0x40);
		else	
		res = mir3da_register_mask_write(NSA_REG_POWERMODE_BW,0xC0,0x80);
	
	return res;	
}

int mir3da_open_interrupt(int level){
	int   res = 0;
		
	switch(level){
		case 3:
			res = mir3da_register_write(NSA_REG_ACTIVE_THRESHOLD,DMT_SENSITIVE_P_LOW );
			break;
		case 2:
			res = mir3da_register_write(NSA_REG_ACTIVE_THRESHOLD,DMT_SENSITIVE_P_MID );
			break;
		case 1:
			res = mir3da_register_write(NSA_REG_ACTIVE_THRESHOLD,DMT_SENSITIVE_P_HIGH );
			break;	
		default:
			break;
	}
	
	res |= mir3da_register_mask_write(NSA_REG_POWERMODE_BW, 0xFF, 0x5E);
	res |= mir3da_register_mask_write(NSA_REG_ODR_AXIS_DISABLE, 0xFF, 0x06);
	res |= mir3da_register_mask_write(NSA_REG_G_RANGE, 0x03, 0x02);
res |= mir3da_register_mask_write(NSA_REG_INT_PIN_CONFIG, 0x0F, 0x00);//set int_pin level
res |= mir3da_register_mask_write(NSA_REG_INT_LATCH, 0x8F, 0x86);//clear latch and set latch mode
	res = mir3da_register_write(NSA_REG_INTERRUPT_MAPPING1,0x04 );
	res = mir3da_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x03);
	res = mir3da_register_write(NSA_REG_ACTIVE_DURATION,0x03 );
	

	return res;
}

int mir3da_close_interrupt(int num){
	int   res = 0;

	res = mir3da_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x00 );
			
	switch(num){

		case 0:
			res = mir3da_register_write(NSA_REG_INTERRUPT_MAPPING1,0x00 );
			break;

		case 1:
			res = mir3da_register_write(NSA_REG_INTERRUPT_MAPPING3,0x00 );
			break;
	}

	return res;
}

/*return value: 0: is ok    other: is failed*/
int mir3da_read_data(short *x, short *y, short *z)
{
    unsigned char    tmp_data[6] = {0};
	int ret;

    /*if (mir3da_register_read_continuously(NSA_REG_ACC_X_LSB, 6, tmp_data) != 0) {
        return -1;
    }*/
	ret = A_sensor_read_buff(NSA_REG_ACC_X_LSB, tmp_data, 6);
	if(ret == -1) {
		printk("[%s:%d]read data error!!!!!!\n", __FUNCTION__, __LINE__);
		return -1;
	}
    
    *x = ((short)(tmp_data[1] << 8 | tmp_data[0]))>> 4;
    *y = ((short)(tmp_data[3] << 8 | tmp_data[2]))>> 4;
    *z = ((short)(tmp_data[5] << 8 | tmp_data[4]))>> 4;
//    printk("rd=%d %d %d\n",*x,*y,*z); 	

    return 0;
}

int cycle_read_xyz(short *x,short *y, short *z, int ns)
{
	int i = 0;
	short tx= 0,ty = 0, tz = 0;
	*x=*y=*z=0;
	for(i = 0;i < ns;i++)
	{
		mdelay(10);
		
		mir3da_read_data(&tx,&ty,&tz);
		*x += tx;	
		*y += ty;	
		*z += tz;		
	}

	*x = *x/ns;
	*y = *y/ns;
	*z = *z/ns;
}
int mir3da_calibrate(void)
{
		short x = 0, y = 0, z = 0;
		short x_off= 0,y_off = 0, z_off = 0;
		
		if(mir3da_read_data(&x,&y,&z) == -1) {
			printk("%s:%d error!!!\n", __FUNCTION__, __LINE__);
			return -1;	
		}
				
		//x_off = (x - x_init)/2;
		//y_off = (y - y_init)/2;
		z_off = (z - z_init)/2;
#if Z_INVERSE
		z_off=-z_off;
#endif
		if(z_off > 127)
			z_off = 127;
		if(z_off < -128)
			z_off = -128;
		
		gsensor_calibrate_z_read[read_cnt] = (unsigned char)(z_off + 128);
		if(read_cnt>=9)
		{
			unsigned char i;
			short max,min,sum;

			min=max=gsensor_calibrate_z_read[0];
			sum =0;
			for(i=0;i<10;i++)
			{
				if(max<gsensor_calibrate_z_read[i])
					max=gsensor_calibrate_z_read[i];
				if(min>gsensor_calibrate_z_read[i])
					min=gsensor_calibrate_z_read[i];					
				sum += gsensor_calibrate_z_read[i];				
				printk("%d ",gsensor_calibrate_z_read[i]);
			}			
			gsensor_calibrate_z_off=ViewAngle_Zvalue-(sum-max-min)/8;
			
			printk("min=%d,max=%d,sum=%d,aver=%d\n",min,max,sum,gsensor_calibrate_z_off);				
			
			read_cnt=0;
		}		
        else
        {
           read_cnt++; 
        }
		return 0;
}

int mir3da_off_calc(unsigned char *ux_off ,unsigned char *uy_off,unsigned char *uz_off)
{
	short x_off= 0,y_off = 0, z_off = 0;

	if(gsensor_calibrate_enable)
	{//calibrateing ,LDW gvalue=0
		gvalus_read[0]=gvalus_read[1]=gvalus_read[2]=gvalus_read[3]=gvalus_read[4]=gvalus_read[5]=0;
		b_is_first =0;
		*ux_off=*uy_off=*uz_off=0;
		return;
	}

	x_off = ((gvalus_read[0]+gvalus_read[3])/2 - x_init)/2;
	y_off = ((gvalus_read[1]+gvalus_read[4])/2 - y_init)/2;
	z_off = ((gvalus_read[2]+gvalus_read[5])/2 - z_init)/2;

#if Z_INVERSE
	z_off=-z_off;
#endif

	if(z_off > 127)
		z_off = 127;
	if(z_off < -128)
		z_off = -128;

	if(y_off > 127)
		y_off = 127;
	if(y_off < -128)
		y_off = -128;

	if(x_off > 127)
		x_off = 127;
	if(x_off < -128)
		x_off = -128;
	//printk("offset1 x y z %d %d %d\n",*x_off,*y_off,*z_off); 

	*ux_off = (unsigned char)(x_off + 128+64);
	*uy_off = (unsigned char)(y_off + 128);	
	*uz_off = (unsigned char)(z_off + 128);
//	printk("(%d,%d,%d)(%d.%d,%d)--[%d,%d,%d]\n",gvalus_read[0],gvalus_read[1],gvalus_read[2],gvalus_read[3],gvalus_read[4],gvalus_read[5],*ux_off,*uy_off,*uz_off);
}

int mir3da_read_int_status(void)
{
	char data = 0;

	mir3da_register_read(NSA_REG_MOTION_FLAG,&data);
	if(data&0x04)
		return 1;

	return 0;
}

int mir3da_check_collision(int threhold)
{
		short x = 0, y = 0, z = 0;
	
		if(mir3da_read_data(&x,&y,&z) == -1) {
			printk("%s:%d error!!!\n", __FUNCTION__, __LINE__);
			gvalus_read[0]=gvalus_read[1]=gvalus_read[2]=gvalus_read[3]=gvalus_read[4]=gvalus_read[5]=0;
			b_is_first =0;
			return -1;	
		}
		
		gvalus_read[3]=gvalus_read[0];
		gvalus_read[4]=gvalus_read[1];
		gvalus_read[5]=gvalus_read[2];
		gvalus_read[0]=x;
		gvalus_read[1]=y;
		gvalus_read[2]=z;
		
		
		if(b_is_first == 0)
		{
			gvalus_read[3] = x;
			gvalus_read[4] = y;
			gvalus_read[5] = z;
			
			b_is_first = 1;				
			return 0;					
		}
		
		if((abs(gvalus_read[5] - gvalus_read[2]) > threhold)||(abs(gvalus_read[4] - gvalus_read[1]) > threhold)||(abs(gvalus_read[3] - gvalus_read[0]) > threhold))
		{
			return 1;
		}
		//printk("%d-rn%d-[%d,%d,%d(%d)]===%d,%d,%d",gvalue_flag,is_collision,abs(x - prev_x),abs(y - prev_y),abs(z - prev_z),threhold,x,y,z);	
		return 0;
}
//========================DA380 inter face end ==================================
#if I2C_DEV == TI2C_MODE
typedef struct gsensor_info_s 
{		
	struct miscdevice dev;	
	struct ti2c_set_value_s g_ti2c_handle;
	struct delayed_work work;
	struct workqueue_struct *wq;
}
gsensor_info_t;

static gsensor_info_t gsensor_data;

static int A_sensor_write(unsigned char reg,unsigned char data)
{
	int ret=0;	
	
	gsensor_data.g_ti2c_handle.transmitMode = TI2C_NORMAL_WRITE_MODE;	
	gsensor_data.g_ti2c_handle.slaveAddrMode =TI2C_NORMAL_SLAVEADDR_8BITS;
	gsensor_data.g_ti2c_handle.slaveAddr = (unsigned short)A_SENSOR_ADDR;
	gsensor_data.g_ti2c_handle.subAddrMode = TI2C_NORMAL_SUBADDR_8BITS;
	gsensor_data.g_ti2c_handle.pSubAddr = (unsigned short *)&reg;
	gsensor_data.g_ti2c_handle.pBuf = &data;	
	gsensor_data.g_ti2c_handle.dataCnt = 1;	

	ret=gp_ti2c_bus_xfer(&gsensor_data.g_ti2c_handle);	

	return ret;
}
static int A_sensor_read(unsigned char reg,unsigned char *value)
{
	int ret=0;	

	gsensor_data.g_ti2c_handle.transmitMode = TI2C_BURST_READ_STOP_MODE;	
	
	gsensor_data.g_ti2c_handle.slaveAddrMode =TI2C_NORMAL_SLAVEADDR_8BITS;
	gsensor_data.g_ti2c_handle.slaveAddr = (unsigned short)A_SENSOR_ADDR;
	gsensor_data.g_ti2c_handle.subAddrMode = TI2C_NORMAL_SUBADDR_8BITS;	
	gsensor_data.g_ti2c_handle.pSubAddr =(unsigned short *) &reg;
	gsensor_data.g_ti2c_handle.pBuf = value;
	gsensor_data.g_ti2c_handle.dataCnt = 1;
	gsensor_data.g_ti2c_handle.apbdmaEn =0;	
	ret=gp_ti2c_bus_xfer(&gsensor_data.g_ti2c_handle);

	return ret;
}
#else
typedef struct gsensor_info_s 
{		
	struct miscdevice dev;	
	int g_ti2c_handle;
	int gsensor_threshold;
	int int_active;

	struct timer_list s_timer;
}
gsensor_info_t;
static gsensor_info_t gsensor_data;

static int A_sensor_write(unsigned char reg,unsigned char data)
{
	int ret=0xff;	
	char buf[2];
	buf[0] = reg;
	buf[1] = data;
	CHECK(gp_i2c_bus_write(gsensor_data.g_ti2c_handle, &buf, 2)>=0);
	printk( "[%s:%d] write reg = 0x%x,data = %d\n ", __FUNCTION__, __LINE__,reg,data);
	return ret;	
	Return:
	printk( "[%s:%d] Error!reg = %d,data = %d\n ", __FUNCTION__, __LINE__,reg,data);	
	
}
static int A_sensor_read(unsigned char reg)
{
	int ret;
	char value;
	CHECK(gp_i2c_bus_write(gsensor_data.g_ti2c_handle, &reg, 1)>=0);
	CHECK(gp_i2c_bus_read(gsensor_data.g_ti2c_handle, &value, 1)>=0);
	//printk( "[%s:%d] read reg = 0x%x,data = %d\n ", __FUNCTION__, __LINE__,reg,value);
	return value;
	Return:
	printk( "[%s:%d] Error!reg = %d\n ", __FUNCTION__, __LINE__,reg);

}
static int A_sensor_read_buff(unsigned char reg, unsigned char *data, unsigned int len)
{
	int ret;
	char value = 0;
	CHECK(gp_i2c_bus_write(gsensor_data.g_ti2c_handle, &reg, 1)>=0);
	CHECK(gp_i2c_bus_read(gsensor_data.g_ti2c_handle, data, len)>=0);
	//printk( "[%s:%d] read reg buf = 0x%x,data = %d\n ", __FUNCTION__, __LINE__,reg,data[0]);
	return value;
	Return:
	printk( "[%s:%d] Error!reg = %d\n ", __FUNCTION__, __LINE__,reg);
	return -1;

}
#endif

static char A_Sensor_Init(unsigned char level)
{
	UINT8 temp= 0xff;
	UINT8 i=0;
	int ret;

	parking_mode_pwron = 0;
	temp = A_sensor_read(NSA_REG_MOTION_FLAG);
	if((temp != 0xff) && (temp & 0x04)) 
	{
		parking_mode_pwron = 1;
	}
	else
	{
		parking_mode_pwron = 0;
	}
	printk("%s:%d open with %d\n", __FUNCTION__, __LINE__, parking_mode_pwron);
//////////////
	ret= mir3da_init();

	if(ret)
	{//if init DA380 err, reinit 8times
		for(i=0;i<8;i++)
		{
			mdelay(20);
			printk("###########DA380 init err retry %d times\n",(i+1));
				
			ret= mir3da_init();			
			if(!ret) 
				break;
		}
	}
	return ret;
}
/**
 *soft reset. if Init error!
 * addr reset device id. 0~addr
 *
 * */
static int A_Sensor_Soft_Reset(void)
{
	char dev_id[5] = {0,1, 6,7, A_SENSOR_ADDR};
	int i = 0;
	int ret = 0;
	gp_i2c_bus_release(gsensor_data.g_ti2c_handle);

	for(i=0; i<5; i++) 
	{

		printk("soft reset DA380...dev_id(%d)\n", dev_id[i]);
		gsensor_data.g_ti2c_handle = gp_i2c_bus_request(dev_id[i],10); // da380 addr
		if(gsensor_data.g_ti2c_handle == -ENOMEM)
		{
			printk(KERN_ALERT"gsensor_data.g_ti2c_handle = 0x%x fail\n",gsensor_data.g_ti2c_handle);	
			return -1;
		}
		ret = A_sensor_write(0x00, 0x24); //soft reset
		mdelay(1);
		ret = A_sensor_write(0x00, 0x24); //soft reset
		mdelay(10);
		gp_i2c_bus_release(gsensor_data.g_ti2c_handle);
		gsensor_data.g_ti2c_handle = gp_i2c_bus_request(A_SENSOR_ADDR,10); // da380 addr
		if(gsensor_data.g_ti2c_handle == -ENOMEM)
		{
			printk(KERN_ALERT"gsensor_data.g_ti2c_handle = 0x%x fail\n",gsensor_data.g_ti2c_handle);	
			return -1;
		}
		ret = A_sensor_read(NSA_REG_WHO_AM_I); 
		if(ret == 0x13) {
			ret = 0;
		}

		gp_i2c_bus_release(gsensor_data.g_ti2c_handle);
	}
	if(i == 5) {
		ret = -1;
	}
	gsensor_data.g_ti2c_handle = gp_i2c_bus_request(A_SENSOR_ADDR,A_SENSOR_CLK); // da380 addr
	if(gsensor_data.g_ti2c_handle == -ENOMEM)
	{
		printk(KERN_ALERT"gsensor_data.g_ti2c_handle = 0x%x fail\n",gsensor_data.g_ti2c_handle);	
		return -1;
	}
		
	return ret;
	
}

void A_sensor_uninit(unsigned char level) //for power off.
{
	printk("DA380 Del Timer,timer_pending=%d\n",timer_pending(&gsensor_data.s_timer));
	del_timer(&gsensor_data.s_timer);
	if(level)
		mir3da_open_interrupt(level);
	da380_error = 0;
}

static long A_sensor_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{	
	//int temp=0xff;	
	int g_value=0;
	short buf[3];
	unsigned char x_off = 0,y_off = 0,z_off = 0;
	
	switch(cmd)	
	{	
		case ASENSOR_IOCTL_GET_INT_ACTIVE:	
				if(da380_error > 2) {
					//printk("[%s][%d]:da380_error is [%d]\r\n",__FUNCTION__,__LINE__,da380_error);
					if(copy_to_user((void __user *)arg,&da380_error,sizeof(int)))			
						return -EFAULT;
				}
				else {
					if(copy_to_user((void __user *)arg,&gsensor_data.int_active,sizeof(int)))			
						return -EFAULT;
					gsensor_data.int_active = 0;
				}
			break;	
		case ASENSOR_IOCTL_SET_SENSITIVE:	
			printk("[%s][%d]:level is [%d]\r\n",__FUNCTION__,__LINE__,arg);
			switch(arg)
				{
					case 3://low
					//	A_sensor_write(ACTIVE_THS,DMT_SENSITIVE_LOW);
						gsensor_data.gsensor_threshold = DMT_THRSHOLD_LOW;
					break;
					case 2://mid
			        //	A_sensor_write(ACTIVE_THS,DMT_SENSITIVE_MID);
						gsensor_data.gsensor_threshold = DMT_THRSHOLD_MID;
					break;
					case 1://high
			       	// 	A_sensor_write(ACTIVE_THS,DMT_SENSITIVE_HIGH);
						gsensor_data.gsensor_threshold = DMT_THRSHOLD_HIGH;
					break;
				}
			break;
		case ASENSOR_IOCTL_PARK_MODE:
				A_sensor_uninit(arg);
			break;
		case ASENSOR_IOCTL_SET_INIT_LEVEL:
            if(arg == 0 && da380_error == 0) {
                //mir3da_register_mask_write(NSA_REG_INT_PIN_CONFIG, 0x0F, 0x05);//set int_pin level
				IntFlag = 1;
            }
            else if(da380_error == 0){
                //mir3da_register_mask_write(NSA_REG_INT_PIN_CONFIG, 0x0F, 0x00);//set int_pin level
				IntFlag = 3;
            }
			break;
		case ASENSOR_IOCTL_PARK_MODE_INT:
				if(copy_to_user((void __user *)arg,&parking_mode_pwron,sizeof(int)))			
				return -EFAULT;		
			break;
		case ASENSOR_IOCTL_GET_G_VALUE:
			//printk("%s:%d=======\n", __FUNCTION__, __LINE__);
			if(da380_error <=2 && da380_error>=0) {
				mir3da_off_calc(&x_off,&y_off,&z_off);
			}
			else {
				x_off = y_off = z_off = 0;
			}
		
			g_value = x_off;			
			g_value <<=8;
			g_value =g_value+y_off;
			g_value <<=8;
			g_value =g_value+z_off;
		//	printk("DA380gvalue%x--(%d,%d,%d)\n",g_value,x_off,y_off,z_off);					
			if(copy_to_user((void __user *)arg,&g_value,sizeof(UINT32)))			
					return -EFAULT;	
			break;
		case ASENSOR_IOCTL_SET_INIT_OFFSET:		
			if(copy_from_user(buf, (void __user*)arg, sizeof(short)*3) < 0) {
				return -EINVAL;
			}
			x_init = buf[0],
			y_init = buf[1], 
			z_init = buf[2],
			printk("--%d,%d,%d--set init offset,timer en=%d\n",x_init,y_init,z_init,timer_pending(&gsensor_data.s_timer));
			if(0==timer_pending(&gsensor_data.s_timer))
			{//timer 未开启
				init_timer(&gsensor_data.s_timer);
				gsensor_data.s_timer.function = &gsensor_xyz_read;
				gsensor_data.s_timer.expires = jiffies + HZ;//delay1s timer 起作用
				add_timer(&gsensor_data.s_timer);	
				printk("DA380 reinit timer add timer\n");
			}
			break;
			
		case ASENSOR_IOCTL_GET_INIT_OFFSET:				
			if(timer_pending(&gsensor_data.s_timer))
			{// timer 运行中，则del timer
				del_timer(&gsensor_data.s_timer);
			}
			cycle_read_xyz(&buf[0],&buf[1], &buf[2], 8);	
			
			//printk("--%d,%d,%d--get init offset[%d,%d,%d]\n",x_init,y_init,z_init,buf[0],buf[1],buf[2]);
			
			if(copy_to_user((void __user *)arg,buf,sizeof(short)*3))			
				return -EFAULT;					
			break;
		case ASENSOR_IOCTL_CALIBRATE_ENABLE://when set 1,ldw gvalue =0
			if(copy_from_user(&gsensor_calibrate_enable, (void __user*)arg, sizeof(int)) < 0) {
				return -EINVAL;
			}
			break;
		case ASENSOR_IOCTL_CALIBRATE_ONOFF:
			if(copy_from_user(&gsensor_calibrate_flag, (void __user*)arg, sizeof(int)) < 0) {
				return -EINVAL;
			}
			//printk("set onoff=%d\n",gsensor_calibrate_flag);
			if(gsensor_calibrate_flag==0x04)
			{
				ViewAngle_Zvalue = VIEW_ANGLE_ZVALUE_TRUCK;
				gsensor_calibrate_flag=1;
			}
			else if(gsensor_calibrate_flag==0x02)
			{
				ViewAngle_Zvalue = VIEW_ANGLE_ZVALUE_SUV;
				gsensor_calibrate_flag=1;
			}
			else if(gsensor_calibrate_flag==0x01)
			{
				ViewAngle_Zvalue = VIEW_ANGLE_ZVALUE_CAR;
				gsensor_calibrate_flag=1;
			}
			else if(gsensor_calibrate_flag==0)
			{
				gsensor_calibrate_flag=0;
			}
			//printk("viewAngle=%d\n",ViewAngle_Zvalue);
			break;
			
		case ASENSOR_IOCTL_GET_CALIBRATE_OFFSET:
			if(copy_to_user((void __user *)arg,&gsensor_calibrate_z_off,sizeof(short)))		
				return -EFAULT;			
			break;
	
		default:	
			return -EINVAL;		
	}	
	return 0;
}

struct file_operations gsensor_fops = 
{	
	.owner		= THIS_MODULE,	
	.unlocked_ioctl = A_sensor_dev_ioctl,
};


//===============================tasklet==================
void gsensor_xyz_read_do_tasklet();

DECLARE_TASKLET(gsensor_read_tasklet,gsensor_xyz_read_do_tasklet,0);

void gsensor_xyz_read_do_tasklet()
{
	int ret = 0;
	//printk("%s:%d %d\n", __FUNCTION__, __LINE__, gsensor_calibrate_flag);
	if(IntFlag == 1) {
		IntFlag = 2;
		mir3da_register_mask_write(NSA_REG_INT_PIN_CONFIG, 0x0F, 0x05);//set int_pin level
	}
	else if(IntFlag == 3) {
		IntFlag = 0;
        mir3da_register_mask_write(NSA_REG_INT_PIN_CONFIG, 0x0F, 0x00);//set int_pin level
	}
	if(gsensor_calibrate_flag)
	{
		ret = mir3da_calibrate();
		if(ret == -1) {
			if(da380_error<1000)
				da380_error++;
		}
		else {
			da380_error = 0;
		}
	}
	else {
		ret = mir3da_check_collision(gsensor_data.gsensor_threshold);
		if(ret == -1) {
			if(da380_error<1000)
				da380_error++;
			gsensor_data.int_active= 0;
		}
		else if(!gsensor_data.int_active && ret == 1)
		{
			da380_error = 0;
			gsensor_data.int_active= 1;
		}
	}
	//printk("# ");
}

	


//===================================================


static void gsensor_xyz_read(unsigned long arg)
{
/*	if(gsensor_calibrate_flag)
	{
		mir3da_calibrate();
	}else
	{
		int int_flag;
		int_flag = mir3da_check_collision(gsensor_data.gsensor_threshold);
		if(!gsensor_data.int_active && int_flag==1)
		{
			gsensor_data.int_active= 1;
		}
	}*/
	//printk("%s:%d %d\n", __FUNCTION__, __LINE__, gsensor_calibrate_flag);
	if(da380_error >=0 && da380_error<=2) {
		mod_timer(&gsensor_data.s_timer,jiffies+HZ/50);
	}
	else {
		mod_timer(&gsensor_data.s_timer,jiffies+HZ*5); //HZ*5
		printk("%s:%d DA380 error %d\n", __FUNCTION__, __LINE__, da380_error);
	}
	tasklet_schedule(&gsensor_read_tasklet);
	//printk("@ ");
}

static int __init gp_asensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	#if I2C_DEV == TI2C_MODE
	gsensor_data.g_ti2c_handle.pDeviceString = "Asensor_DA380";	
	gsensor_data.g_ti2c_handle.slaveAddrMode = TI2C_NORMAL_SLAVEADDR_8BITS;
	gsensor_data.g_ti2c_handle.slaveAddr = (unsigned short)A_SENSOR_ADDR;
	gsensor_data.g_ti2c_handle.clockRate = A_SENSOR_CLK;
	gsensor_data.g_ti2c_handle.apbdmaEn = 0;	/* open ti2c */	
	ret = gp_ti2c_bus_request(&gsensor_data.g_ti2c_handle);
	#else
	gsensor_data.g_ti2c_handle = gp_i2c_bus_request(A_SENSOR_ADDR,A_SENSOR_CLK);
	if(gsensor_data.g_ti2c_handle == -ENOMEM)
	{
		printk(KERN_ALERT"gsensor_data.g_ti2c_handle = 0x%x fail\n",gsensor_data.g_ti2c_handle);	
		return;
	}
	printk(KERN_ALERT"gsensor_data.g_ti2c_handle = 0x%x\n",gsensor_data.g_ti2c_handle);	
	#endif
	if(A_Sensor_Init(2))
	{
		if(A_Sensor_Soft_Reset() != 0) {
        	da380_error = 3;
			printk("DA380...reset Init error!!!! \n");
		}
		else {
			if(A_Sensor_Init(2)) {
        		da380_error = 3;
			}
		}
	}
	printk("da380 init ok!!!\n");
	
	gsensor_data.dev.name = "gp_asensor";	
	gsensor_data.dev.fops = &gsensor_fops;	
	gsensor_data.dev.minor = MISC_DYNAMIC_MINOR;
	
	ret = misc_register(&gsensor_data.dev);	
	if(ret)	
	{		
		printk(KERN_ALERT"gsensor probe register faile\n");		
		goto err_reg;	
	}
	platform_set_drvdata(pdev,&gsensor_data);

	gsensor_data.gsensor_threshold = 10000;

	init_timer(&gsensor_data.s_timer);
	gsensor_data.s_timer.function = &gsensor_xyz_read;
	gsensor_data.s_timer.expires = jiffies + HZ;//delay1s timer 起作用
	add_timer(&gsensor_data.s_timer);


	
	printk(KERN_ALERT"gsensor kxtf9 probe ok\n");	
	return 0;
err_reg:	
	#if I2C_DEV == TI2C_MODE
	gp_ti2c_bus_release(&gsensor_data.g_ti2c_handle);
	#else
	gp_i2c_bus_release(gsensor_data.g_ti2c_handle);
	#endif
	return ret;	
}

static int gp_asensor_remove(struct platform_device *pdev)
{

	int ret;
	printk("[%s][%d]\r\n",__FUNCTION__,__LINE__);	
	
	//A_sensor_uninit(2);
	A_sensor_write(NSA_REG_POWERMODE_BW,0x5e);
	#if I2C_DEV == TI2C_MODE
	gp_ti2c_bus_release(&gsensor_data.g_ti2c_handle);
	#else
	gp_i2c_bus_release(gsensor_data.g_ti2c_handle);
	#endif
	ret = misc_deregister(&gsensor_data.dev);	
	if(ret)	
	{		
		printk(KERN_ALERT"gsensor rmmod register faile\n");		
		return ret;
	}
	
	return 0;
}

static int gp_asensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("[%s][%d]\r\n",__FUNCTION__,__LINE__);
	return 0;
}
static int gp_asensor_resume(struct platform_device *pdev)
{
	printk("[%s][%d]\r\n",__FUNCTION__,__LINE__);
	return 0;
}

static void gp_asensor_device_release(struct device *dev)
{
	printk("[%s][%d]\r\n",__FUNCTION__,__LINE__);
}



static struct platform_device gp_asensor_device = {
	.name = "gp_asensor",
	.id   = -1,
	.dev	= {
		.release = gp_asensor_device_release,
	}
};

static struct platform_driver gp_asensor_driver = {
       .driver         = {
	       .name   = "gp_asensor",
	       .owner  = THIS_MODULE,
       },
       .probe          = gp_asensor_probe,
       .remove         = gp_asensor_remove,
       .suspend        = gp_asensor_suspend,
       .resume         = gp_asensor_resume,

};

static int __init gp_gsensor_module_init(void)
{
	int rc;
	printk("gp_asensor_module_init \n");
	platform_device_register(&gp_asensor_device);
	rc = platform_driver_register(&gp_asensor_driver);
	printk("gp_asensor_module_init  end\n");
	return rc;
}

static void __exit gp_gsensor_module_exit(void)
{
	platform_device_unregister(&gp_asensor_device);
	platform_driver_unregister(&gp_asensor_driver);
}

module_init(gp_gsensor_module_init);
module_exit(gp_gsensor_module_exit);
MODULE_LICENSE_GP;






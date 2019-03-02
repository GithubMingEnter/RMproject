/**
  ******************************************************************************
  * @file    mpu.c
  * @author  langgo
  * @version V1.0
  * @date    2015-11-19
  * @brief   mpu6050/mpu6500/mpu9250 module driver.
  *          This file provides motion processing uint APIs to 
  *          Configuration MPU6500 or MPU 9250 and Read the Accelerator
  *			 			and Gyrometer data using SPI interface of f103 SPI2
	* @update		2016年2月25日22:21:01
	*						##增加了IIC 对 MPU6050和MPU6500，MPU9250的支持
							6500及9250可以使用SPI进行读取，前提是已经初始化SPI和nCS的IO引脚
							在mpu.h里更改需要使用的mpu type和interface type(6500/9250only!)
	* @update2  2016年3月7日20:09:35
							修复了两个兼容性BUG，所有SPI使用阻塞式发送！！
  *	@caution 使用SPI阻塞发送测试通过， 使用IT 或 DMA 操作均有问题，
						所以推荐使用Polling Mode
  *         使用该驱动前确保SPI硬件已经初始化！！
	*	@注意			在iic delay那个函数里，为了配合F4的高速IO翻转加了很多延时，
							F1实际上可以不需要那么多延时
  * @verbatim you should comment with english avoid diff text encoder cause ?!$@#%^@$%&$*
  ****************************************************************************
***/

#include "stm32f4xx_hal.h"
#include "mpu.h"
#include "ist8310_reg.h"	//magenet meter
#include <math.h>
#include "calibrate.h"
#include "pid.h"
//#include "mytype.h"
//#include "param.h"
#define USE_FREE_RTOS
#ifdef USE_FREE_RTOS
	#include "cmsis_os.h"
	#define MPU_DELAY(x)		osDelay(x)
#else
	#define MPU_DELAY(x)		HAL_Delay(x)
#endif


#define MPU6500
//#define MPU6050
#define MPU6500_USE_SPI
//#define MPU6500_USE_IIC

//??AD0?(9?)??,IIC???0X68(??????).
//???V3.3,?IIC???0X69(??????).
//6500 and 6050 is the same addr
#define MPU_ADDR				0X68	

#if	defined (MPU6500) || defined (MPU9250)
	#if !defined(MPU6500_USE_IIC) && !defined(MPU6500_USE_SPI)
		#error "fuck! define a interface MPU6500_USE_IIC/MPU6500_USE_SPI for MPU6500"
	#endif
#endif

#if defined(MPU6500) || defined(MPU9250)
	#include "mpu6500_reg.h"
#elif defined(MPU6050)
	#include "mpu6050_reg.h"
#endif


#if defined(MPU6500) || defined(MPU9250)	//use SPI interface to read data
	#if	defined (MPU6500_USE_SPI)
		#include "spi.h"
		#define MPU_HSPI			hspi5
		#define MPU_NSS_LOW		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
		#define MPU_NSS_HIGH	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

		static u8  tx, rx;
		static u8 tx_buff[14]={0xff};
	#elif defined (MPU6500_USE_IIC)
		#include "myiic.h"
	#endif
#elif	defined(MPU6050)
	#include "myiic.h"			//use iic to read

#endif



u8  mpu_buff[14];
MPU_OriginData  mpu_data;
		imu_t	imu;
//MPU6050_Class	mpu6050;
/*The firstbit of the first byte contains 
the Read/Write bit and indicates 
the Read (1) or Write (0) operation
so SPI data MSB = 1 when read*/

u8 mpu_write_reg(u8 const reg, u8 const data){
#if defined(MPU6500) || defined(MPU9250)
	#if	defined (MPU6500_USE_SPI)
		MPU_NSS_LOW;
		tx = reg & 0x7F;
		HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1 ,55);
		tx = data;
		HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1 ,55);
		MPU_NSS_HIGH;
		return 0;
	#elif defined (MPU6500_USE_IIC)
		u8 res = IIC_Write_Reg(MPU_ADDR, reg, data);
		return res;		//0 if success!
	#endif
#elif defined (MPU6050)
	u8 res = IIC_Write_Reg(MPU_ADDR, reg, data);
	return res;		//0 if success!
#endif
}

u8	mpu_read_reg(u8 const reg){
#if defined (MPU6500) || defined (MPU9250)
	#if	defined (MPU6500_USE_SPI)
		MPU_NSS_LOW;
		tx = reg | 0x80;
	
		HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
		HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
		MPU_NSS_HIGH;
		return rx;
	#elif defined (MPU6500_USE_IIC)
		u8 res = IIC_Read_Reg(MPU_ADDR, reg);
		return res;
	#endif
	
#elif defined (MPU6050)
	u8 res = IIC_Read_Reg(MPU_ADDR, reg);
	return res;
#endif
}

u8 mpu_read_regs(u8 const regAddr, u8 *pData, u8 len){
#if defined (MPU6500) || defined (MPU9250)
	#if	defined (MPU6500_USE_SPI)
		MPU_NSS_LOW;
		tx = regAddr | 0x80;
		tx_buff[0] = tx;
		HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
		HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
		MPU_NSS_HIGH;
		return 0;
	#elif defined (MPU6500_USE_IIC)
		u8 res = IIC_Read_Bytes(MPU_ADDR, regAddr, pData, len);
		return res;
	#else
		#error "define a interface spi or iic for 6500!!!"
	#endif
	
#elif defined (MPU6050)

	u8 res = IIC_Read_Bytes(MPU_ADDR, regAddr, pData, len);
	return res;
#endif
}


/**
* @fn       ist_reg_write_by_mpu
* @brief    Write IST8310 register through MPU6500's I2C Master
* @param    Register address¡êoreg_address, Register content: reg_data
* @retval   void
* @note     ist8310_init need to be called before.
*/
static void ist_reg_write_by_mpu(u8 addr, u8 data){
	//turn off slave 1 at first 
	mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);	
	MPU_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_REG, addr);
	MPU_DELAY(2);
	mpu_write_reg(MPU6500_I2C_SLV1_DO, data);
	MPU_DELAY(2);
	//turn on slave 1 with one byte transmitting
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);	
	//wait longer to ensure the data is transmitted from slave 1
	MPU_DELAY(10);
}

/**
* @fn       ist_reg_read_by_mpu
* @brief    Write IST8310 register through MPU6500's I2C Master
* @param    Register address¡êoreg_address
* @retval   void
* @note     ist8310_init need to be called before.
*/
static u8 ist_reg_read_by_mpu(u8 addr)
{
	u8 retval;
	mpu_write_reg(MPU6500_I2C_SLV4_REG, addr);
	MPU_DELAY(10);
	mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x80);
	MPU_DELAY(10);
	retval = mpu_read_reg(MPU6500_I2C_SLV4_DI);
	//turn off slave4 after read
	mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
	MPU_DELAY(10);
	return retval;
}


/**
* @fn       mpu_mst_i2c_auto_read_config
* @brief    Initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    Slave device address, Address[6:0]
* @retval   void
* @note     
*/
static void mpu_mst_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    //configure the device address of the IST8310
    //use slave1,auto transmit single measure mode.
	mpu_write_reg(MPU6500_I2C_SLV1_ADDR, device_address );
	MPU_DELAY(2);
	mpu_write_reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  MPU_DELAY(2);
  mpu_write_reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);   
	MPU_DELAY(2);   
    
  //use slave0,auto read data
	mpu_write_reg(MPU6500_I2C_SLV0_ADDR, 0x80|device_address );
	MPU_DELAY(2);
	mpu_write_reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  MPU_DELAY(2);
    
	//every eight mpu6500 internal samples one i2c master read
	mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x03);   
	MPU_DELAY(2);    
	//enable slave 0 and 1 access delay 
	mpu_write_reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01|0x02);  
	MPU_DELAY(2);   
	//enable slave 1 auto transmit
  mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);  
	MPU_DELAY(6);                            //Wait 6ms (minimum waiting time for 16 times internal average setup)
	//enable slave 0 with data_num bytes reading 
  mpu_write_reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);   
	MPU_DELAY(2);
}

u8 ist8310_init(){
	mpu_write_reg(MPU6500_USER_CTRL, 0x30);//enable iic master mode
	MPU_DELAY(10);
	mpu_write_reg(MPU6500_I2C_MST_CTRL, 0x0d);//enable iic 400khz
	MPU_DELAY(10);
	
	//turn on slave 1 for ist write and slave 4 to ist read 
	mpu_write_reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);//enable iic 400khz
	MPU_DELAY(10);
	mpu_write_reg(MPU6500_I2C_SLV4_ADDR, 0x80|IST8310_ADDRESS);//enable iic 400khz
	MPU_DELAY(10);
	
	//IST8310_R_CONFB 0x01	= device rst
	ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);//soft rst
	MPU_DELAY(10);
	if ( IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I) ) 
		return 1;//wrong
	
	ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);	//rst
	MPU_DELAY(10);
	
	ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);	//config as ready mode to access reg
	if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00) 
		return 2;
	MPU_DELAY(10);
	
	ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);	//normal state, no int
	if(ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
		return 3;
	MPU_DELAY(10);
	//config  low noise mode, x,y,z axis 16 time 1 avg, 
	ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24);	//100100
	if(ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
		return 4;
	MPU_DELAY(10);
	
	//Set/Reset pulse duration setup,normal mode
	ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);	
	if(ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
		return 5;
	MPU_DELAY(10);
	
	//turn off slave1 & slave 4
	mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
	MPU_DELAY(10);
	mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
	MPU_DELAY(10);
	
	//configure and turn on slave 0 
	mpu_mst_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06); 
	MPU_DELAY(100);
	return 0;
}

u8 ist_buff[6];
float test_yaw1,test_yaw2;
void ist8310_get_data(u8 *buff) 
{   
   mpu_read_regs(MPU6500_EXT_SENS_DATA_00,buff,6);   //¶ÁÈ¡µ½´ÅÁ¦¼ÆÊý¾Ý
}

int16_t MPU6050_FIFO[6][11] = {0};
void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]Îª×î½ü10´ÎÊý¾Ý [10]Îª10´ÎÊý¾ÝµÄÆ½¾ùÖµ  ¾ùÖµÂË²¨
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[0][9]=ax;//½«ÐÂµÄÊý¾Ý·ÅÖÃµ½ Êý¾ÝµÄ×îºóÃæ
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//Çóµ±Ç°Êý×éµÄºÏ£¬ÔÙÈ¡Æ½¾ùÖµ
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}
int16_t eff,eff2;
int16_t gy_z;

float C_Gyo_z;
int16_t Eff[120] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
									 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
									 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
									 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 
									 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
									 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
									 60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
									 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
									 80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
	                 90, 91,92,93,94,95,96,97,98,99,
	                 100,101,102,103,104,105,106,107,108,109,
	                 110,111,112,113,114,115,116,117,118,119
                                                          }; 
int16_t accgyroval[6]; //´æ´¢ÍÓÂÝÒÇÊý¾Ý

void mpu_get_data(){
	
#if defined (MPU6500) || defined (MPU9250)
	mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
#elif defined (MPU6050)
	mpu_read_regs(MPU_ACCEL_XOUTH_REG, mpu_buff, 14);
#endif
	u8 i,j;
	mpu_data.ax = mpu_buff[0]<<8 | mpu_buff[1];
	mpu_data.ay = mpu_buff[2]<<8 | mpu_buff[3];
	mpu_data.az = mpu_buff[4]<<8 | mpu_buff[5];
	mpu_data.temp = mpu_buff[6]<<8 | mpu_buff[7];
	if(gAppParam.ImuCaliList[CALI_GYRO].isAlreadyCalied == 0x55){
		mpu_data.gx = mpu_buff[8]<<8 | mpu_buff[9] - gAppParam.ImuCaliList[CALI_GYRO].offset[0];
		mpu_data.gy = mpu_buff[10]<<8 | mpu_buff[11] - gAppParam.ImuCaliList[CALI_GYRO].offset[0];
		mpu_data.gz = mpu_buff[12]<<8 | mpu_buff[13] - gAppParam.ImuCaliList[CALI_GYRO].offset[0];
	}
	ist8310_get_data(ist_buff);
	memcpy(&mpu_data.mx, ist_buff, 6);
	
	memcpy(&imu.ax, &mpu_data.ax, 6*sizeof(s16));
	imu.temp = 21 + mpu_data.temp/333.87f;
	//imu.wx = mpu_data.gx/16.384f/57.3f;	//2000dps -> rad/s
	//imu.wy = mpu_data.gy/16.384f/57.3f;	//2000dps -> rad/s
	//imu.wz = mpu_data.gz/16.384f/57.3f;	//2000dps -> rad/s
	
	test_yaw1 = 57.3f * atan2f(mpu_data.my, mpu_data.mx);
	MPU6050_DataSave(mpu_data.ax,mpu_data.ay,mpu_data.az,mpu_data.gx,mpu_data.gy,mpu_data.gz);  

  	mpu_data.ax1  =MPU6050_FIFO[0][10];
		mpu_data.ay1  =MPU6050_FIFO[1][10];
		mpu_data.az1 = MPU6050_FIFO[2][10];
		mpu_data.gx1 = MPU6050_FIFO[3][10];
		mpu_data.gy1 = MPU6050_FIFO[4][10];
		mpu_data.gz1 = MPU6050_FIFO[5][10];
	for(i = 0; i < 120; i++)
		{
				if(mpu_data.gx1 ==  Eff[i])
				{
						eff = Eff[i];
						break;
				}
					if(-mpu_data.gx1 == Eff[i])
				{
						eff = -Eff[i];
						break;
				}
		}		
	for(j = 0; j < 120; j++)
		{
				if(mpu_data.gz1 ==  Eff[j])
				{
						eff2 = Eff[j];
						break;
				}
					if(-mpu_data.gz1 == Eff[j])
				{
						eff2 = -Eff[j];
						break;
				}
		}				
		mpu_data.gx1= mpu_data.gx1;//-eff;
		mpu_data.gz1= mpu_data.gz1-eff2;
		imu.wz=mpu_data.gz1/32.8f;
		imu.wy += -imu.wz * 0.02f; //角速度积分得出角度
		if(imu.wy >180)		imu.wy=imu.wy-355;	//角度变为连续
		if(imu.wy < -180)		imu.wy=imu.wy+355 ;
	imuCaliHook(CALI_GYRO, &mpu_data.gx);
	imuCaliHook(CALI_ACC, &mpu_data.ax);
	imuCaliHook(CALI_MAG, &mpu_data.mx);

}


//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
#if	defined (MPU6050) 
	return mpu_write_reg(MPU_GYRO_CFG_REG, fsr<<3);
#elif defined (MPU6500) || defined (MPU9250) 
	return mpu_write_reg(MPU6500_GYRO_CONFIG, fsr<<3);

#endif
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
#if defined (MPU6050) 
	return mpu_write_reg(MPU_ACCEL_CFG_REG, fsr<<3);//?????????????
#elif defined (MPU6500)|| defined (MPU9250)
	return mpu_write_reg(MPU6500_ACCEL_CONFIG, fsr<<3);//?????????????
#endif
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
//6050 和 6500 通用
u8 MPU_Set_Gyro_LPF(u16 lpf)
{
#if defined (MPU6050)
	u8 data=0;
	if(lpf>=188)data=1;	
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return mpu_write_reg(MPU_CFG_REG, data);//????????? 
#elif defined (MPU6500) || defined(MPU9250)	
	u8 data=0;
	if(lpf>250)data=7;				//fcut  = 3600hz
	else if(lpf>=184)data=0;	//fcut = 250hz
	else if(lpf>=92)data=1;	
	else if(lpf>=41)data=2;
	else if(lpf>=20)data=3;
	else if(lpf>=10)data=4;
	else if(lpf>=5)data=5;
	else data=6; 

	return mpu_write_reg(MPU6500_CONFIG, data);//????????? 
#endif
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 

#if defined(MPU6050)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
		float temp;
		IIC_Read_Bytes(MPU_ADDR, MPU_TEMP_OUTH_REG, buf, 2); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
#endif

u8 id;
u8 mpu_device_init( void )
{
#if defined (MPU6500)|| defined (MPU9250)
	#if defined (MPU6500_USE_IIC)
		IIC_Init();	//very important!!!
	#endif

	MPU_DELAY(100);

	id = mpu_read_reg(MPU6500_WHO_AM_I);
  u8 i = 0;
  u8 MPU6500_Init_Data[10][2] = {
      {MPU6500_PWR_MGMT_1, 		0x80},    	// Reset Device
      {MPU6500_PWR_MGMT_1, 		0x03},    	// Clock Source - Gyro-Z
      {MPU6500_PWR_MGMT_2, 		0x00},     	// Enable Acc & Gyro
      {MPU6500_CONFIG, 			0x04},     	// LPF 41Hz//gyro bandwidth 41Hz
      {MPU6500_GYRO_CONFIG, 	0x18},    	// +-2000dps	
      {MPU6500_ACCEL_CONFIG,	0x10},    	// +-8G
      {MPU6500_ACCEL_CONFIG_2,0x02}, 		// enable LowPassFilter  Set Acc LPF
      {MPU6500_USER_CTRL,			0x20},      // Enable AUX
	};
	for(i=0; i<10; i++)
	{
		mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}
	
	MPU_Set_Gyro_Fsr(3);					//??????,0=250,1=500,2=1000,3=Ѳ000dps
	MPU_Set_Accel_Fsr(2);					//??????,0=Ѳg,1=4g, 2=8g, 3=16g
	
	ist8310_init();
	
	return 0;
#elif defined (MPU6050)
	IIC_Init();	//very important!!!
	id = IIC_Read_Reg(MPU_ADDR, MPU_DEVICE_ID_REG);
	IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG,	0X80);	//??MPU6050
  MPU_DELAY(100);
	IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG,	0X00);	//??MPU6050 
	MPU_Set_Gyro_Fsr(3);					//??????,0=250,1=500,2=1000,3=Ѳ000dps
	MPU_Set_Accel_Fsr(2);					//??????,0=Ѳg,1=4g, 2=8g, 3=16g
	IIC_Write_Reg(MPU_ADDR, MPU_INT_EN_REG,	0X00);	//??????
	IIC_Write_Reg(MPU_ADDR, MPU_USER_CTRL_REG,	0X00);	//I2C?????
	IIC_Write_Reg(MPU_ADDR, MPU_FIFO_EN_REG,	0X00);	//??FIFO
	IIC_Write_Reg(MPU_ADDR, MPU_INTBP_CFG_REG,	0X80);	//INT???????	
	if(id == MPU6050_ID)//??ID??
	{
		IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG,0X01);	//??CLKSEL,PLL X????
		IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT2_REG,0X00);	//??????????
 	}
	else 
		return 1;
	return 0;
#endif
}
/**	@bref put in mpu_read_data() function,if dont need calibrate, will just return
	*
	*
	*/
//void mpu_calibrate_hook(){

//}


#ifdef __cplusplus
MPU6050_Class::MPU6050_Class(){
	
}

void MPU6050_Class::init(void){
		mpu_init();
	
		readMpuOffset();
}

void MPU6050_Class::readAccGyroData(void){
//	s16 ax,ay,az,gx,gy,gz;

	mpu_get_data();
	
	accAdc.x = mpu_data.ax;
	accAdc.y = mpu_data.ay;
	accAdc.z = mpu_data.az;
	gyroAdc.x = mpu_data.gx;
	gyroAdc.y = mpu_data.gy;
	gyroAdc.z = mpu_data.gz;
	
	accCalibrate();
	gyroCalibrate(); 
}

Vector3f MPU6050_Class::getAcc(void){
	s16 t1 = (s16)accAdc.x - (s16)AccOffset.x;
	s16 t2 = (s16)accAdc.y - (s16)AccOffset.y;
	s16 t3 = (s16)accAdc.z - (s16)AccOffset.z;

	return Vector3f((float)t1, (float)t2, (float)t3);

}

Vector3f MPU6050_Class::getGyro(void){
	s16 t1 = gyroAdc.x - GyroOffset.x;
	s16 t2 = gyroAdc.y - GyroOffset.y;
	s16 t3 = gyroAdc.z - GyroOffset.z;
	return Vector3f(t1, t2, t3);
}

Vector3f MPU6050_Class::getGyroInRadps(void){
	Vector3f tmp = getGyro();
	tmp.x *= MPU6050G_s2000Radps;
	tmp.y *= MPU6050G_s2000Radps;
	tmp.z *= MPU6050G_s2000Radps;
	return tmp;
}

void MPU6050_Class::accCalibrate(){
	if(accNeedCalibrate == true){
		static Vector3f tmpAcc;
		static uint16_t cnt_a=0;
	
		//?????1?G ?????
		//ACC_Z_ADC_1G
		if(cnt_a == 0){
			AccOffset(0, 0, 0);
			tmpAcc(0, 0, 0);
			cnt_a = 1;
//      flyUnlock = false;
			return;
		}
		tmpAcc += this->accAdc;
		if(cnt_a == 1000){	//a thousand times calibrate cycles
			AccOffset.x = tmpAcc.x / cnt_a;
			AccOffset.y = tmpAcc.y / cnt_a;
			AccOffset.z = tmpAcc.z / cnt_a - ACC_Z_ADC_1G;
			cnt_a = 0;
			accNeedCalibrate = false;
			saveMpuAccGyroOffset();
			return;
		}
		cnt_a++;
	}
}

void MPU6050_Class::gyroCalibrate(){
	if(gyroNeedCalibrate == true){
		static Vector3f tmpGyro;
		static uint16_t cnt_g=0;
			
		if(cnt_g == 0){
			GyroOffset(0, 0, 0);
			tmpGyro(0, 0, 0);
			cnt_g = 1;
			return;
		}
		tmpGyro += this->gyroAdc;
		if(cnt_g == 1000){	//a thousand times calibrate cycles
			
			GyroOffset.x = tmpGyro.x / cnt_g;
			GyroOffset.y = tmpGyro.y / cnt_g;
			GyroOffset.z = tmpGyro.z / cnt_g;
			
			cnt_g = 0;
			gyroNeedCalibrate = false;
			saveMpuAccGyroOffset();
			return;
		}
		cnt_g++;
	}

}
#endif


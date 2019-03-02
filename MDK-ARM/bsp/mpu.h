#ifndef _MPU_H_
#define _MPU_H_

//#include "my_math.h"
#include "mytype.h"



typedef struct{
	s16 ax;
	s16 ay;
	s16 az;
			s16 ax1;
	s16 ay1;
	s16 az1;
		s16 gx1;
	s16 gy1;
	s16 gz1;
	s16 mx;
	s16 my;
	s16 mz;
	
	s16 temp;
	
	s16 gx;
	s16 gy;
	s16 gz;
	
}MPU_OriginData;

typedef struct{
	s16 ax;
	s16 ay;
	s16 az;
	
	s16 mx;
	s16 my;
	s16 mz;
	
	float temp;
	
	float wx;	//omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s
	float wy;
	float wz;
	
	float rol;
	float pit;
	float yaw;
}imu_t;

u8 mpu_device_init(void);
void mpu_get_data(void);
extern MPU_OriginData  mpu_data;
extern imu_t	imu;
#ifdef __cplusplus
class MPU6050_Class
{
public:
	MPU6050_Class();
	
	bool accNeedCalibrate;
	bool gyroNeedCalibrate;

	Vector3s16	AccOffset,GyroOffset;

	//???6500
	void init(void);

//	void readAccData(void);
	void readAccGyroData(void);
	//read and return value!
	Vector3f getAcc(void);

	Vector3f getGyro(void);

	//????? ???
	Vector3f getGyroInRadps(void);
	
private:
	
	Vector3f accAdc, gyroAdc;
	Vector3f gyroInDps;

	void accCalibrate(void);
	void gyroCalibrate(void);

};

extern MPU6050_Class	mpu6050;
#endif
#endif

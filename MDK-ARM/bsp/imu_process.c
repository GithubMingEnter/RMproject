#include "imu_process.h"


//#include "tinyEKF.h"
#include "tiny_ekf.h"
#include "mpu.h"
#include "math.h"
#include "cmsis_os.h"
#define deltaT   0.005


//kf_t myKF;
float acc[3];
ekf myEKF;

static void copyToMatrix(double * dst,double * src,int n)
{
	for(int i= 0; i < n; ++i)
	{
		 dst[i] = src[i];
	}
}
void acc_average_filter(uint8_t count,s16 acc[], s16 filtered_acc[]);
/********************************************************************
* * Description ： imu_task
* * Arguments	 :  void * p_arg
* * Returns	 :  void
* * CreateTime  :  2015/11/18
* * Creator     :  Joseph_Chen
*********************************************************************/
void imu_task(const void * p_arg)
{

	p_arg = p_arg;

//	static int 		imuTime[10] = {0};
//	static float   yawDelta = 0;
//	static float  rollDelta = 0;
//	static float  pitchDelta = 0;
	static float  deltaTime = 0;
	static int    msCnt = 0;

	//椭球拟合参数  0526
	//x1 0.0121  y1 0 z1 = -0.0268
	//kx 0.9989  ky 0.9961 kz 1
	//a  -0.0021  1.5248  1.5736
	//模型  U_test = K * U_real + B
	//这里已经换算过了, Ureal = K_-1 * (Utest - B)
//	myImu.Acc.Kx = 0.9925;  
//	myImu.Acc.Ky = 0.9962;  
//	myImu.Acc.Kz = 1; 
//	
//	myImu.Acc.alpha = -0.0062;
//	myImu.Acc.beta  = 0;
//	myImu.Acc.gama  = 0;

//	myImu.Acc.Bx = 0.0097;
//	myImu.Acc.By = 0.0011;
//	myImu.Acc.Bz = -0.0288;

	

	/* EKF结构体初始化  */
	/* 非线性系统模型 */
	// X(k) = f( X(k-1),U(k) ) + w
	// Z(k) = h( X(k) ) + v

	ekf_init(&myEKF,myEKF.n,myEKF.m);

	//const matrix
	double P[] = {1,	0,	0,	0,
								0,	1,	0,	0,
								0,	0,	1,	0,
								0,	0,	0,	1,};
	
	double Q[] = {0.001,	0,			0,			0,
								0,			0.001,	0,			0,
								0,			0,			0.001,	0,
								0,			0,			0,			0.001,};
	
	double R[] = {16,	0,	0,	
								0,	16,	0,	
								0,	0,	16,	
								};  
	

	copyToMatrix(myEKF.P,P,eN*eN);  
	copyToMatrix(myEKF.Q,Q,eN*eN);
	copyToMatrix(myEKF.R,R,eM*eM);
	
	
//	myImu.Pitch = 0;
//	myImu.Roll  = 0;
//	myImu.Yaw   = 0;
	
								
	myEKF.x[0] = 1;	//q0
	myEKF.x[1] = 0;	//q1
	myEKF.x[2] = 0;	//q2
	myEKF.x[3] = 0;	//q3
	
	osDelay(5000);
	while(1)
	{
		

		/* 得到原始数据 */
//		IMU_GetRawData(&myImu);  //1.3ms
//		/* 数据预处理   */
//		
//		/* 陀螺仪 */
//		myImu.Gyro.X = myImu.Gyro_Real.X - myImu.Gyro.X_Zero;
//		myImu.Gyro.Y=  myImu.Gyro_Real.Y - myImu.Gyro.Y_Zero;
//		myImu.Gyro.Z = myImu.Gyro_Real.Z - myImu.Gyro.Z_Zero;  
//		myImu.Wx =  GYRO_NORMAL(myImu.Gyro.X);  //单位是rad/s 以下所有计算都以这为准，最后换算
//		myImu.Wy =  GYRO_NORMAL(myImu.Gyro.Y);
//		myImu.Wz =  GYRO_NORMAL(myImu.Gyro.Z);

//		/* 磁力计内部标定 读取flash里数据 */
//		myImu.Mag.X = (int)(myImu.Mag.Kx * myImu.Mag_Real.X + myImu.Mag.X_Zero);
//		myImu.Mag.Y = (int)(myImu.Mag.Ky * myImu.Mag_Real.Y + myImu.Mag.Y_Zero);     
//		myImu.Mag.Z = (int)(myImu.Mag.Kz * myImu.Mag_Real.Z + myImu.Mag.Z_Zero);    
		/* 椭球拟合标定 */
//      myImu.Mag.X = 0.8342 * (myImu.Mag_Real.X + 0.1446 * 599.5) - 
//                    0.0121 * (myImu.Mag_Real.Z + 0.0874 * 599.5);
//      myImu.Mag.Y = 0.8490 * (myImu.Mag_Real.Y + 0.0565 * 599.5) -
//                    0.0187 * (myImu.Mag_Real.Z + 0.0874 * 599.5);
//      myImu.Mag.Z = myImu.Mag_Real.Z + 0.0874 * 599.5;
		

		 /*  磁力计  */
		/* Hn = C_nb *Hb  */
		/* Yn = sinP * sinR * Xb + cosP * Yb - sinP * cosR *Zb  */
		/* Xn = Xb * cosRoll + Zb * sinRoll  */
//		 myImu.MXn = myImu.Mag.X * myCos(myImu.Roll) + myImu.Mag.Z * mySin(myImu.Roll);
//		 
//		 myImu.MYn = myImu.Mag.X * mySin(myImu.Pitch) * mySin(myImu.Roll) + myImu.Mag.Y * myCos(myImu.Pitch)
//								 - mySin(myImu.Pitch) * myCos(myImu.Roll) * myImu.Mag.Z;
//		 
//		 myImu.Yaw_Ob[1] = myImu.Yaw_Ob[0];
//		 myImu.Yaw_Ob[0] = -atan2( myImu.MXn, myImu.MYn)*180.0/PI; 

		

		/* 加速度计10次均值滤波 */
		//acc_average_filter(10, &imu.ax, &imu.ax);
		/* 加速度计椭球拟合标定  */
		
//		myImu.Ax =  1.0076 * (myImu.Acc_Real.X/16384.0f - myImu.Acc.Bx)  
//								-0.0062* (myImu.Acc_Real.Z/16384.0f - myImu.Acc.Bz);
//								
//		myImu.Ay =  1.0038 * (myImu.Acc_Real.Y/16384.0f - myImu.Acc.By) +
//								0.0006 * (myImu.Acc_Real.Z/16384.0f - myImu.Acc.Bz);

//		myImu.Az =  1 * (myImu.Acc_Real.Z/16384.0f - myImu.Acc.Bz);

//		myImu.A_Error = (1.0 - myImu.Ax*myImu.Ax - myImu.Ay*myImu.Ay - myImu.Az*myImu.Az)*100.0; //error 在2%以内
//		
//		myImu.Acc.X = (int)(myImu.Ax * 16384.0);
//		myImu.Acc.Y = (int)(myImu.Ay * 16384.0);
//		myImu.Acc.Z = (int)(myImu.Az * 16384.0);
		 mpu_get_data();
		acc[0] = imu.ax/1024.0f;
		acc[1] = imu.ay/1024.0f;
		acc[2] = imu.az/1024.0f;
		
		
//		myImu.Pitch_Ob[1] = myImu.Pitch_Ob[0];
//		myImu.Pitch_Ob[0] = safe_asin(myImu.Ay) * 180.0/PI;
//		imu.pit = safe_asin(acc[1]) * 57.3f;
////		myImu.Roll_Ob[1] = myImu.Roll_Ob[0];
////		myImu.Roll_Ob[0] = -atan2(myImu.Ax,myImu.Az) * 180.0/PI;   
//		imu.rol = -atan2(acc[0],acc[2]) *57.3f;   
		/* 扩展卡尔曼滤波 */
		/* 改变状态变量为四元数 */
		//姣曞崱绠楁硶锛屽洓闃惰繎浼硷紝 鐪嬨�婃儻鎬у鑸��
		deltaTime = 0.005f;
		double deltaX = deltaTime * imu.wx;
		double deltaY = deltaTime * imu.wy;
		double deltaZ = deltaTime * imu.wz;
		double delta2 = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
		double temp1  = 1 - delta2/8.0 + delta2 * delta2 /384.0;
		double temp2  = 0.5 - delta2 /48.0;
		
		/*  output vector */
//		myEKF.z[0] = myImu.Ax;
//		myEKF.z[1] = myImu.Ay;
//		myEKF.z[2] = myImu.Az;
		myEKF.z[0] = (float)(imu.ax/1024.0f);
		myEKF.z[1] = (float)(imu.ay/1024.0f);
		myEKF.z[2] = (float)(imu.az/1024.0f);
 
		
		/*  fx  状态方程,四阶比卡算法离散化 dQ/dt = 1/2 *W*Q  */
		//F
		myEKF.F[0][0] = temp1;
		myEKF.F[0][1] = -temp2 * deltaX;
		myEKF.F[0][2] = -temp2 * deltaY;
		myEKF.F[0][3] = -temp2 * deltaZ;
		
		myEKF.F[1][0] = temp2 * deltaX;
		myEKF.F[1][1] = temp1;
		myEKF.F[1][2] = temp2 * deltaZ;
		myEKF.F[1][3] = -temp2* deltaY;
		
		myEKF.F[2][0] = temp2 * deltaY;
		myEKF.F[2][1] = -temp2* deltaZ;
		myEKF.F[2][2] = temp1;
		myEKF.F[2][3] = temp2 * deltaX;
		
		myEKF.F[3][0] = temp2 * deltaZ;
		myEKF.F[3][1] = temp2 * deltaY;
		myEKF.F[3][2] = -temp2* deltaX;
		myEKF.F[3][3] = temp1;
		
		//f(x) = A*x
		myEKF.fx[0] = myEKF.x[0] * myEKF.F[0][0] + myEKF.x[1] * myEKF.F[0][1] + myEKF.x[2] * myEKF.F[0][2] + myEKF.x[3] * myEKF.F[0][3];
		myEKF.fx[1] = myEKF.x[0] * myEKF.F[1][0] + myEKF.x[1] * myEKF.F[1][1] + myEKF.x[2] * myEKF.F[1][2] + myEKF.x[3] * myEKF.F[1][3];      
		myEKF.fx[2] = myEKF.x[0] * myEKF.F[2][0] + myEKF.x[1] * myEKF.F[2][1] + myEKF.x[2] * myEKF.F[2][2] + myEKF.x[3] * myEKF.F[2][3];     
		myEKF.fx[3] = myEKF.x[0] * myEKF.F[3][0] + myEKF.x[1] * myEKF.F[3][1] + myEKF.x[2] * myEKF.F[3][2] + myEKF.x[3] * myEKF.F[3][3];
		
		/* 输出方程 */
		//Z(k) = h(x(k)) + V
		//hx
//		double temp3 =  myEKF.x[0] * myEKF.x[0] - myEKF.x[1] * myEKF.x[1] + myEKF.x[2] * myEKF.x[2] - myEKF.x[3] * myEKF.x[3]; 
//		double temp4 =  -2.0 *(myEKF.x[1] * myEKF.x[2] - myEKF.x[0] * myEKF.x[3]);
//		double temp5 =  (temp4*temp4/temp3/temp3 + 1);
		
		myEKF.hx[0] = 2*(myEKF.x[1] * myEKF.x[3] - myEKF.x[0] * myEKF.x[2]);
		myEKF.hx[1] = 2*(myEKF.x[2] * myEKF.x[3] + myEKF.x[0] * myEKF.x[1]);
		myEKF.hx[2] = myEKF.x[0] * myEKF.x[0] - myEKF.x[1] * myEKF.x[1] - myEKF.x[2] * myEKF.x[2] + myEKF.x[3] * myEKF.x[3];
 
			
	 //H  pitch != 90  temp3 != 0
		myEKF.H[0][0] = -2 * myEKF.x[2];
		myEKF.H[1][0] =  2 * myEKF.x[1];
		myEKF.H[2][0] =  2 * myEKF.x[0];
 
		
		myEKF.H[0][1] = 2 * myEKF.x[3];
		myEKF.H[1][1] = 2 * myEKF.x[0];
		myEKF.H[2][1] = -2* myEKF.x[1];
 
//((2*q2)/(q0^2 - q1^2 + q2^2 - q3^2) - (2*q1*(2*q0*q3 - 2*q1*q2))/(q0^2 - q1^2 + q2^2 - q3^2)^2)/((2*q0*q3 - 2*q1*q2)^2/(q0^2 - q1^2 + q2^2 - q3^2)^2 + 1)  
			
		myEKF.H[0][2] = -2 * myEKF.x[0];
		myEKF.H[1][2] = 2  * myEKF.x[3];
		myEKF.H[2][2] = -2 * myEKF.x[2];
 
//((2*q1)/(q0^2 - q1^2 + q2^2 - q3^2) + (2*q2*(2*q0*q3 - 2*q1*q2))/(q0^2 - q1^2 + q2^2 - q3^2)^2)/((2*q0*q3 - 2*q1*q2)^2/(q0^2 - q1^2 + q2^2 - q3^2)^2 + 1)
			
		myEKF.H[0][3] =  2 * myEKF.x[1];
		myEKF.H[1][3] =  2 * myEKF.x[2];
		myEKF.H[2][3] =  2 * myEKF.x[3];
 
//-((2*q0)/(q0^2 - q1^2 + q2^2 - q3^2) + (2*q3*(2*q0*q3 - 2*q1*q2))/(q0^2 - q1^2 + q2^2 - q3^2)^2)/((2*q0*q3 - 2*q1*q2)^2/(q0^2 - q1^2 + q2^2 - q3^2)^2 + 1)
			 
	 if(ekf_step(&myEKF,myEKF.z))
	 {
		  msCnt ++;
	 };
	 //归一化处理  
	 //褰掍竴鍖�
	 float sum = 0;
	 sum =  myEKF.x[0]*myEKF.x[0] + myEKF.x[1]*myEKF.x[1] + myEKF.x[2]*myEKF.x[2] + myEKF.x[3] * myEKF.x[3];
	 float norm = inv_sqrt(sum);       
		myEKF.x[0]  *= norm;
		myEKF.x[1]  *= norm;
		myEKF.x[2]  *= norm;
		myEKF.x[3]  *= norm;
		
		
		
		//输出欧拉角  
//		imu.pit =  safe_asin(2.0*(myEKF.x[2] * myEKF.x[3] + myEKF.x[0] * myEKF.x[1]))* 180/PI;
//		imu.rol  =  -atan2(2.0*(myEKF.x[1]*myEKF.x[3] - myEKF.x[0]*myEKF.x[2]),myEKF.x[0]*myEKF.x[0] -  myEKF.x[1]*myEKF.x[1]- myEKF.x[2]*myEKF.x[2] + myEKF.x[3] * myEKF.x[3])* 180/PI; 
//		imu.yaw   =  atan2(2.0 *( myEKF.x[1]*myEKF.x[2] - myEKF.x[0]*myEKF.x[3]),myEKF.x[0]*myEKF.x[0] - myEKF.x[1]*myEKF.x[1] + myEKF.x[2]*myEKF.x[2]- myEKF.x[3] * myEKF.x[3])* 180/PI;     

		
		osDelay(5);
	}
 
}

/********************************************************************
* * Description ：  inv_sqrt ,快速计算1/sqrt(x)
* * Arguments	 :  
* * Returns	 :  
* * CreateTime  :  2015/11/26
* * Creator     :  Joseph_Chen
*********************************************************************/
float inv_sqrt(float x)
{
	float half_x = 0.5 * x;
	float y = x;
	long i = *(long *)&x;
	i = 0x5f3759df - ( i >> 1);
	y = *(float *)&i;
	y = y *(1.5 - half_x * y *y);
	
	return y;
}



float safe_asin(float x)
{
 
 if(isnan(x))
 {
	 return 0.0;
 }
 if(x >= 1.0f)
 {
	 return PI/2;
 }
 if(x <= -1.0f)
 {
		return  -PI/2;
 }
 return asin(x);
}
/********************************************************************
* * Description ：  姿态解算
* * Arguments	 :  当前测量值,陀螺仪、加速度计、磁力计
									陀螺仪单位为rad/s，加速度、磁力计不用管
* * Returns	 :  
* * CreateTime  :  2015/11/26
* * Creator     :  Joseph_Chen
*********************************************************************/
/*
void  IMU_AHRSUpdate(IMU_TypeDef *imu)
{
	 float ax,ay,az,gx,gy,gz,mx,my,mz;
	 float hx,hy,hz,bx,bz;
	 float vx,vy,vz,wx,wy,wz;
	 float ex,ey,ez;
	 float norm = 0;
	 float pitch =0,roll = 0,yaw = 0;

	 float q0q0 = q0*q0;
	 float q0q1 = q0*q1;
	 float q0q2 = q0*q2;
	 float q0q3 = q0*q3;     
	 float q1q1 = q1*q1;
	 float q1q2 = q1*q2;
	 float q1q3 = q1*q3;
	 float q2q2 = q2*q2;
	 float q2q3 = q2*q3;
	 float q3q3 = q3*q3;
	 
	 ax = (float)(imu->Acc.X/16384.0); //为了下一步计算方便,单位G,可不转换
	 ay = (float)(imu->Acc.Y/16384.0);
	 az = (float)(imu->Acc.Z/16384.0); 
	 
	 //重力，磁力计归一化处理
	 norm = inv_sqrt(ax * ax + ay * ay + az * az);
	 ax = ax * norm;     //三个轴分量与加速度矢量的夹角余弦值
	 ay = ay * norm;
	 az = az * norm;
	 
	 

	 
	 norm = inv_sqrt(imu->Mag.X * imu->Mag.X + imu->Mag.Y * imu->Mag.Y + imu->Mag.Z * imu->Mag.Z);
	 mx = imu->Mag.X * norm;
	 my = imu->Mag.Y * norm;
	 mz = imu->Mag.Z * norm;   
	 //计算参考磁通方向
	 //   hx        mx
	 //  [hy] = C*[ my]
	 //   hz        mz
	 //C为坐标旋转变换阵，从刚体固连坐标系/载体系b到坐标系n(导航坐标系/地理坐标系)
	 //即把刚体固连坐标系b上测量值转换到导航坐标系中.所有传感器测量值都是基于刚体坐标系
	 hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2-q0q3) + 2*mz*(q1q3 + q0q2);
	 hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	 hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
	 
	 bx = sqrt((hx * hx) + (hy*hy));
	 bz = hz;
	 //vx,vy,vz是陀螺仪积分后的姿态推算出来的重力向量
	 //ax,ay,az是测量得到的重力向量
	 //他们之间的误差向量，就是陀螺仪积分后的姿态和加速度计测出来的姿态之间的误差
	 //向量间的误差，可以用叉积表示，叉积向量仍旧位于机体坐标系，陀螺仪积分误差也在机体坐标系上，
	 //叉积大小与陀螺仪积分误差成正比 
	 
	 vx = 2*(q1q3 - q0q2);    //T31,参考系Z轴与载体系X轴之间方向余弦向量
	 vy = 2*(q0q1 + q2q3);    //T32  参考系Z轴与载体系Y轴之间方向余弦向量
	 vz = q0q0 - q1q1 -q2q2 + q3q3; //T33  参考系Z轴与载体系Z轴之间方向余弦向量
	 
	 //        bx
	 // W = C[ 0 ]
	 //        bz
	 wx = 2*bx*(0.5 - q2q2 - q3q3) +2*bz*(q1q3 - q0q2);
	 wy = 2*bx*(q1q2  - q0q3) + 2*bz*(q0q1 + q2q3);
	 wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
	 //误差是向量叉积
	 ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	 ey = (az*vx - ax*vz) + (mz*wx - mx*wz);             
	 ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	//角速度单位要转换为rad/s
//    gx = (float)(imu->Gyro.X)/16.384*PI/180.0; 
//    gy = (float)(imu->Gyro.Y)/16.384*PI/180.0; 
//    gz = (float)(imu->Gyro.Z)/16.384*PI/180.0;
	 gx = imu->Wx;
	 gy = imu->Wy;
	 gz = imu->Wz;
	 
	 //integral error scaled integral gain
	if(ex != 0.0 && ey != 0.0 && ez != 0.0)

	{
	 exInt += ex * KiDef*halfT;
	 eyInt += ey * KiDef*halfT;
	 ezInt += ez * KiDef*halfT;
	 
	 //修正陀螺仪测量值
	 gx = gx + KpDef * ex + exInt;
	 gy = gy + KpDef * ey + eyInt;
	 gz = gz + KpDef * ez + ezInt;
	}
	
	//四元数微分方程
	//
	//                 0 -wx -wy  -wz
	//                 wx 0  wz   -wy
	// [dQ/dt] = 0.5*[ wy -wz 0   wx ] *Q
	//                 wz wy -wx  0
	//比卡算法，一阶近似, 定时采样增量法
	//角增量阵delta = []*dT,  Q[k+1] = (I + 0.5*delta)*Q[k]
	//Q[k+1] = Q[k] + 0.5*[ ]*dT*Q[k],dT采样周期,所以有halfT
	 q0 = q0 + (-q1 * gx - q2 *gy - q3 * gz) * halfT;
	 q1 = q1 +(q0 * gx + q2*gz - q3*gy)*halfT;
	 q2 = q2 +(q0*gy - q1*gz + q3*gx)*halfT;
	 q3 = q3 +(q0*gz + q1*gy -q2*gx)*halfT;
	
	 
	 norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	 q0 *= norm;
	 q1 *= norm;
	 q2 *= norm;
	 q3 *= norm;
	 
	 //更新姿态角
													
	 //ZXY,坐标转换对应到MPU6050坐标系，Y指向北，X指向东
	 yaw = atan2(2.0 *( q1*q2 - q0*q3),1 - 2*q1*q1 - 2*q3*q3)* 180/PI;
	 pitch = safe_asin(2.0*(q2*q3 + q0*q1))* 180/PI;     
	 roll = -atan2(2.0*(q1*q3 - q0*q2),1 - 2.0*(q1*q1 + q2*q2))* 180/PI;

	 imu->Pitch = pitch;
	 imu->Roll = roll;
	 imu->Yaw = yaw; 


}
	*/ 

// void IMU_FreeUpdate(IMU_TypeDef * imu)
// {
//   
//      float norm;
//      float vx,vy,vz;
//      float ex,ey,ez;
//      float ax,ay,az,gx,gy,gz;
//      
//     ax = (float)(imu->Acc_X/16384.0);
//     ay = (float)(imu->Acc_Y/16384.0);
//     az = (float)(imu->Acc_Z/16384.0); 
//    
//     //归一化
//     norm = inv_sqrt(ax * ax +ay * ay + az * az);
//     ax = ax * norm;     //三个轴分量与加速度矢量的夹角余弦值
//     ay = ay * norm;
//     az = az * norm;
//      
//       //比力f = g[T31,T32,T33]
//      //dT内加速度速度增量dV,近似有
//      //dV/dT/g = [T31,T32,T33]
//      vx = 2*(q1*q3 - q0*q2);     //T31
//      vy = 2*(q0*q1 + q2*q3);     //T32
//      vz = q0*q0 - q1*q1- q2*q2 + q3*q3; //T33
//      
//      
//      //正交轴的误差, 正交理论误差为0,
//      //Dij = Ci*Cj_T,
//      //修正Ci = Ci - 0.5*Dij*Cj
//      //    Cj = Cj - 0.5*Dij*Ci
//      ex = ay*vz - az*vy;    //    ex      0   -az ay       vx
//      ey = az*vx - ax*vz;    //  [ ey ] = [az  0   -ax]   [ vy]
//      ez = ax*vy - ay*vx;    //    ez      -ay  ax  0       vz
//      
//      exInt += ex*KiDef;
//      eyInt += ey*KiDef;
//      ezInt += ez*KiDef;
//      
//      
//
//      gx = (imu->Gyro_X - imu->Gyro_X_Zero)/16.384*PI/180; 
//      gy = (imu->Gyro_Y - imu->Gyro_Y_Zero)/16.384*PI/180; 
//      gz = (imu->Gyro_Z - imu->Gyro_Z_Zero)/16.384*PI/180;
//      
//      gx = gx + KpDef * ex + exInt;
//      gy = gy + KpDef * ey + eyInt;
//      gz = gz + KpDef * ez + ezInt; 
//      
//     q0 = q0 + (-q1 * gx - q2 *gy - q3 * gz) * halfT;
//     q1 = q1 +(q0 * gx + q2*gz - q3*gy)*halfT;
//     q2 = q2 +(q0*gy - q1*gz + q3*gx)*halfT;
//     q3 = q3 +(q0*gz + q1*gy -q2*gx)*halfT;
//     
//     norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//     q0 *= norm;
//     q1 *= norm;
//     q2 *= norm;
//     q3 *= norm;
//     
//     //更新姿态角
//     imu->Pitch = safe_asin(2.0*(q2*q3 + q0*q1))* 180/PI;     
//     imu->Roll = -atan2(2.0*(q1*q3 - q0*q2),1 - 2.0*(q1*q1 + q2*q2))* 180/PI;
//     imu->Yaw = atan2(2.0 *( q1*q2 - q0*q3),q0*q0 - q1*q1 + q2*q2 - q3*q3)* 180/PI;     
// }




/********************************************************************
* * Description ： 加速度滑动平均滤波
* * Arguments	 :  count < 50次
* * Returns	 :  
* * CreateTime  :  2016/01/12
* * Creator     :  Joseph_Chen
*********************************************************************/
void acc_average_filter(uint8_t count,s16 acc[], s16 filtered_acc[])
{
 
	static s16 acc_tmp[3][10];
	static u8 idx=0,i;
	s32 acc_sum[3]={0};
	//static s16 filted_acc[3];
	
	idx = idx++ > 10? 0 : idx;
	acc_tmp[0][idx] = acc[0];
	acc_tmp[1][idx] = acc[1];
	acc_tmp[2][idx] = acc[2];
	
	for(i=0; i<10; i++){
		acc_sum[0] += acc_tmp[0][i];
		acc_sum[1] += acc_tmp[1][i];
		acc_sum[2] += acc_tmp[2][i];
	}
	filtered_acc[0] = acc_sum[0]/10;
	filtered_acc[1] = acc_sum[1]/10;
	filtered_acc[2] = acc_sum[2]/10;
	
//	return filted_acc;
	

}


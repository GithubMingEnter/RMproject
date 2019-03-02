
/*
* AHRS
* Copyright 2010 S.O.H. Madgwick
*
* This program is free software: you can redistribute it and/or
* modify it under the terms of the GNU Lesser Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser Public License for more details.
*
* You should have received a copy of the GNU Lesser Public License
* along with this program.  If not, see
* <http://www.gnu.org/licenses/>.
*/
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRS_Update()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//

//#include "ahrs_att_est.h"
#include "mahonyAHRS.h"
#include "math.h"
#include "mpu.h"
#include "mytype.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "cmsis_os.h"
#include "bt.h"
#include "bsp_can.h"

#include "mpu.h"
#include "error_task.h"
#include <math.h>
#include "chassis.h"
#include "gimbal.h"
#include "calibrate.h"
#include "kb.h"
#include "judge_sys.h"
#include "sys.h"
#include "bt.h"
//#include "app_estimator.h"

static volatile float exInt, eyInt, ezInt;  // 误差积分
static volatile float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // 全局四元数
static volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
//static float rate_x, rate_y, rate_z;

float Kp=      1.0f ;   // when acc not only gravity, proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki =     0.000f;    // integral gain governs rate of convergence of gyroscope biases//没有积分效果更好
#define HALF_T  0.0025f; // half of operate period


/**
 * 更新四元数
 * Gyroscope units are radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
 *
 * @param gx gy gz // unit rad/s
 * @param ax ay az
 * @param mx my mz
 */
static void AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // 先把这些用得到的值算好
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    //单位化数据
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0) return;
    mx /= norm;
    my /= norm;
    mz /= norm;

    // 求出重力在机体坐标系的轴向分量(旋转矩阵的最后一列)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 将磁场矢量转换至地理坐标系e(乘以旋转矩阵)
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
    // 求出在磁场矢量在地理坐标系e中XoY平面和Z轴的投影
    // (地理坐标系X朝向磁场北极，所以by为0, 磁场矢量: [bx 0 bz]T)
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // 将磁场矢量转换回机体坐标系(利用旋转矩阵的逆(转置))
    wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);

    // 计算总误差：重力误差和磁场误差
    // 误差的含义是：当前四元数得到的姿态与当前传感器测量到的姿态的差，利用叉积得到
    // 为什么不用 h x b ? 因为要在机体坐标系中运算，才会得到在机体坐标系中的误差
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    //误差积分
    exInt += ex * Ki;// * HALF_T;
    eyInt += ey * Ki;// * HALF_T;
    ezInt += ez * Ki;// * HALF_T;

    //将误差补偿到陀螺仪
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

//    rate_x = gx;
//    rate_y = gy;
//    rate_z = gz;

    //利用四元数的微分方程，更新四元数
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * HALF_T;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * HALF_T;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * HALF_T;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * HALF_T;

    //单位化四元数
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm == 0) return;
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void IMU_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    //
    //四元数运算需要用到的值
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    //单位化处理
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0) return;
    ax /= norm;
    ay /= norm;
    az /= norm;
    //
    //提取旋转矩阵中重力在机身坐标轴的三个分量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //坐标系和重力叉积运算 1*1*sin(theta)
    //代表估计的重力方向 与 加计测量的重力方向 之间的角度，即误差
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    //积分运算
    exInt += ex * Ki;
    eyInt += ey * Ki;
    ezInt += ez * Ki;

    //将误差经过PI运算加到角速度中
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

//    rate_x = gx;
//    rate_y = gy;
//    rate_z = gz;

    //利用四元数的微分方程，更新四元数
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * HALF_T;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * HALF_T;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * HALF_T;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * HALF_T;

    //单位化处理
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm == 0) return;
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

/**
 * 将四元数转换成方向余弦矩阵
 *
 * @param R 3*3的矩阵，用二维数组代替,逆为转置，对调下表即可实现反向转换
 */
void Quaternion_ToDCM(float R[3][3])
{
    float aSq = q0 * q0;
    float bSq = q1 * q1;
    float cSq = q2 * q2;
    float dSq = q3 * q3;
    R[0][0] = aSq + bSq - cSq - dSq;
    R[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    R[0][2] = 2.0f * (q0 * q2 + q1 * q3);
    R[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    R[1][1] = aSq - bSq + cSq - dSq;
    R[1][2] = 2.0f * (q2 * q3 - q0 * q1);
    R[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    R[2][1] = 2.0f * (q0 * q1 + q2 * q3);
    R[2][2] = aSq - bSq - cSq + dSq;

}

float euler[3];
float aeuler[3];
/**
 * 获取姿态角和角速率
 */

void AHRS_GetAttitude(const void *argu)
{
//    CPU_SR_ALLOC();
//    static float last_yaw;
//    float yaw, yaw_diff;
//    //将陀螺仪的测量值转成弧度每秒
//    //加速度和磁力计保持原值　不需要转换
	imu.wy=0;//
   while(1)
{
	
	  //mpu_get_data();
    AHRS_Update( 	mpu_data.gx * 0.001065264436031695f,
									mpu_data.gy * 0.001065264436031695f,
									mpu_data.gz * 0.001065264436031695f,
									
									mpu_data.ax, mpu_data.ay, mpu_data.az,
									mpu_data.mx, mpu_data.my, mpu_data.mz
               );
		
//    // /*不融合磁力计更新四元数*/
//			IMU_Update(mpu_data.gx * 0.001065264436031695f, // (gyro / (32768/2000) * M_PI / 180)
//									mpu_data.gy * 0.001065264436031695f,
//									mpu_data.gz * 0.001065264436031695f,
//									mpu_data.ax, mpu_data.ay, mpu_data.az
//								 );
			
			//四元数更新完毕，开始计算欧拉角，
			euler[0] = 180.0f/3.1415926f * -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1); // yaw
			euler[1] = 180.0f/3.1415926f * -asin(-2 * q1 * q3 + 2 * q0 * q2); // pitch
			euler[2] = 180.0f/3.1415926f * atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1); // roll
			aeuler
			[0] = atan2(mpu_data.ay, mpu_data.az)*57.3f;
			aeuler[1] = atan2(mpu_data.ax, mpu_data.az)*57.3f;
			uart_send2pc(&BT_HUART, RealRol, euler[1]); 
			uart_send2pc(&BT_HUART, RealPit, aeuler[1]); 
//    float rad2degree = 180.0f / M_PI;
//    CPU_CRITICAL_ENTER();
//    // xyz-右前上 坐标系
//    gMc_AttPos.roll  = -euler[1] * rad2degree;
//    gMc_AttPos.pitch =  euler[2] * rad2degree;
//    yaw              = -euler[0] * rad2degree + 90;
//    // 将范围统一到-180~+180, 配合上位机显示姿态
//    if (yaw < -180)
//        yaw += 360;
//    if (yaw > 180)
//        yaw -= 360;
  osDelay(5);
}
}




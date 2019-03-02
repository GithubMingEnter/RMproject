#ifndef __IMU_PROCESS_H
#define __IMU_PROCESS_H

#include <stdint.h>


#define PI  		3.1415926535
#define halfT   0.0025   //2ms

//#define GYRO_NORMAL(x)  ((float)(x/16.384f)*PI/180.0f)   //rad/s
//#define angleToDegree(x)   ((float)(x)*180.0f/PI)                    //dps
//#define angleToRad(x)      ((float)(x)*PI/180.0f)


//#define myCos(x)        (cos((float)x*PI/180.0))
//#define mySin(x)        (sin((float)x*PI/180.0))
//#define myTan(x)        (tan((float)x*PI/180.0))

//#define vN  6 //状态变量6个
//#define vM  3 //观测量3

//typedef struct {

//    int n;
//    int m;
//    
//    double x[vN];    /* state vector */
//    double u[vN];   /* input vector */
//    double z[vM];   //观测量

//    double P[vN][vN];  /* prediction error covariance */
//    double Q[vN*vN];  /* process noise covariance */
//    double R[vM*vM];  /* measurement error covariance */

//    double G[vN][vM];  /* Kalman gain; a.k.a. K */

//    double A[vN*vN];   //系统矩阵
//    double H[vM*vN];   //输出矩阵
//    double B[vN*vN];   // 控制矩阵  
//    

//    double Ht[vN][vM];
//    double At[vN][vN];
//    double Pp[vN][vN];

// 

//    /* temporary storage */
//    double tmp1[vN][vM];
//    double tmp2[vM][vN];
//    double tmp3[vM][vM];
//    double tmp4[vM][vM];
//    double tmp5[vM]; 
//    double tmp6[vN][vN];
//    double tmp7[vN];
//    double tmp8[vM];

//} kf_t;


#define eN   4   //状态变量4
#define eM   3   //观测量4
typedef struct {

    int n;          /* number of state values */
    int m;          /* number of observables */

    double x[eN];    /* state vector */
    double z[eM];    /* output vector */
    
    double P[eN*eN];  /* prediction error covariance */
    double Q[eN*eN];  /* process noise covariance */
    double R[eM*eM];  /* measurement error covariance */

    double G[eN][eM];  /* Kalman gain; a.k.a. K */

    double F[eN][eN];  /* Jacobian of process model */
    double H[eM][eN];  /* Jacobian of measurement model */

    double Ht[eN][eM]; /* transpose of measurement Jacobian */
    double Ft[eN][eN]; /* transpose of process Jacobian */
    double Pp[eN][eN]; /* P, post-prediction, pre-update */

    double fx[eN];   /* output of user defined f() state-transition function */
    double hx[eM];   /* output of user defined h() measurement function */

    /* temporary storage */
    double tmp1[eN][eM];
    double tmp2[eM][eN];
    double tmp3[eM][eM];
    double tmp4[eM][eM];
    double tmp5[eM]; 

} ekf;   

//void acc_average_filter(uint8_t count);


void imu_task(const void *p_arg);
float inv_sqrt(float x);
float safe_asin(float x);
//void  IMU_AHRSUpdate(IMU_TypeDef *imu);
//void IMU_FreeUpdate(IMU_TypeDef * imu);

#endif

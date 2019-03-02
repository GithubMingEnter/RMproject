#ifndef __UNDERPAN_TASK_H__
#define __UNDERPAN_TASK_H__

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define WHEEL_DIAMETER 		( 50.0f )               //定位轮直径，单位:mm 
#define WHEEL_PERIMETER 	157.07963267949f //(PI*WHEEL_DIAMETER)       //定位轮周长，单位:mm 
#define CLOSEDLOOPRATIO -1
#ifdef EC60
	#define Max_WheelSpeed 2500  //最大速度
#else
	#define Max_WheelSpeed 850
#endif
#define 	Gyro_Ratio  1 // (1.651376146f)  //陀螺仪比例系数  如果不是1就不对啊
#define 	PI 							(3.1415926535898f)

typedef enum
{
	Ctrl_Mode_RemoteControl = 1,  //只有遥控器模式
	Ctrl_Mode_KeyBoard_OpenLoop = 3, //鼠标键盘 角速度开环     
	Ctrl_Mode_KeyBoard_CloseLoop = 2, //鼠标键盘 角速度闭环
}Undepan_Mode; 

typedef union
{
 uint8_t u8_form[8];
 int16_t s16_form[4];
}underpan_speed_uniontype;

typedef union{
	uint8_t 	ui8t[4];
	uint32_t 	ui32t;
	int32_t 	i32t;
	float 		f32t;
}format_union;

typedef struct 
{
	float my_normal_speed;   //底盘合速度
	float my_speedx;         //x分速度
	float my_speedy;				//y分速度
	float my_omega;					//w角速度  to calc matrix, target omega
	float target_angle;//地盘目标角度
	float angle_from_gyro;
	
	float omega_from_gyro; //measure omega
	float last_angle;
	format_union 	angle_from_gimbal;
	int		is_snipe_mode;	//	1 = yes 0 = no
	/*new add*/
  underpan_speed_uniontype underpan_speed;
}Underpan;  //底盘状态


void CalculateSpeed(float myVx,float myVy,float omega);
void UnderpanTask(void const * argument);
void resetGryo(void);
void initUnderpanPID(void);

//extern float realtime_chassis_power;
//extern float realtime_chassis_current;
//extern float realtime_chassis_voltage;

extern Underpan myUnderpan;
#endif

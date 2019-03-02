/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h" 
#include "bsp_can.h"
#include "can.h"
#include "pid.h"
#include "chassis.h"
#include "bt.h"
#include "error_task.h"
#include "gimbal.h"
#include "kb.h"
#include <math.h>
#include "judge_sys.h"
#include "sys.h"
#include "forbidden.h"

#define MyAbs(x) 	( (x>0) ? (x) : (-x) )
// #define MAX_WHEEL_SPEED					 3600//300
#define MAX_CHASSIS_VX_SPEED			3200//150
#define MAX_CHASSIS_VY_SPEED			4000//150
#define MAX_CHASSIS_OMEGA_SPEED		3600//300
#define CHASSIS_OLD
#define MAX_FOLLOW_SPEED           6000
//#define DEBUG_REVERSE		//2 diff chassis

float MAX_WHEEL_SPEED =0;

typedef enum{
	MOVE_NONE=0,	//no move, just move accoding by rc.
	MOVE_Y_LINE,//move a straight line, then stop,
	MOVE_SQUARE,
	MOVE_CIRCLE,
}MOVE_STA_e;
MOVE_STA_e	chassis_move_sta = MOVE_NONE;
extern int last_sw1;

chassis_t chassis;	//main chassis object


s16 buff_3510iq[4];
volatile double V_previous,V_Current;
volatile double dV;
volatile double iV;
volatile double V1;	
volatile double round_cnt;
		volatile double V2_previous,V2_Current;
volatile double dV2;
volatile double iV2;
volatile double V2;	
volatile double round_cnt2;
		volatile double V3_previous,V3_Current;
volatile double dV3;
volatile double iV3;
volatile double V3;	
volatile double round_cnt3;
		volatile double V4_previous,V4_Current;
volatile double dV4;
volatile double iV4;
volatile double V4;	
volatile double round_cnt4;
float power_date;
int qqq,k1,j,oj,k2,k3,k4,oj2,oj3,oj4,cm1,cm2,cm3,cm4;
float pidd(float px1,float px2)
{

   const float v_p =2;//5
    const float v_d =4;//7
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    
    error_v[0] = error_v[1];
    error_v[1] = px1-px2;
   

    output = error_v[1] * v_p
   
         		+ (error_v[1] - error_v[0]) * v_d;
     
    if(output > 5000)
    {
        output = 5000;
    }
    
    if(output < -5000)
    {
        output = -5000;
    }
		 qqq=-output;
		return -output;
}
float pidd22(float px3,float px4)
{

   const float v2_p = 2;
    const float v2_d = 4;
    
    static float error2_v[2] = {0.0,0.0};
    static float output2 = 0;
		
 
    error2_v[0] = error2_v[1];//v0 存上一次的误差，v1 存本次的误差
    error2_v[1] = px3-px4;
    
    output2 = error2_v[1] * v2_p
             + (error2_v[1] - error2_v[0]) * v2_d;
     
    if(output2 > 5000)
    {
        output2 = 5000;
    }
    
    if(output2 < -5000)
    {
        output2 = -5000;
    }
		 
		return -output2;
}

float pidd33(float px5,float px6)
{

    const float v3_p =2;
    const float v3_d =4;
    
    static float error3_v[2] = {0.0,0.0};
    static float output3 = 0;
		
 
    error3_v[0] = error3_v[1];//v0 存上一次的误差，v1 存本次的误差
    error3_v[1] = px5-px6;
    
    output3 = error3_v[1] * v3_p
             + (error3_v[1] - error3_v[0]) * v3_d;
     
    if(output3 > 5000)
    {
        output3 = 5000;
    }
    
    if(output3 < -5000)
    {
        output3 = -5000;
    }
		 
		return -output3;
}
float pidd44(float px7,float px8)
{

   const float v4_p = 2;//45,0.2
    const float v4_d = 4;
    
    static float error4_v[2] = {0.0,0.0};
    static float output4 = 0;
		
 
    error4_v[0] = error4_v[1];//v0 存上一次的误差，v1 存本次的误差
    error4_v[1] = px7-px8;
    
    output4 = error4_v[1] * v4_p
             + (error4_v[1] - error4_v[0]) * v4_d;
     
    if(output4 > 5000)
    {
        output4 = 5000;
    }
    
    if(output4 < -5000)
    {
        output4 = -5000;
    }
		 
		return -output4;
}
float pidd55(float px9,float px10)
{

   const float v5_p = 0.8;//45,0.2
    const float v5_d = 0;
    
    static float error5_v[2] = {0.0,0.0};
    static float output5 = 0;
		
 
    error5_v[0] = error5_v[1];//v0 存上一次的误差，v1 存本次的误差
    error5_v[1] = px9-px10;
    
    output5 = error5_v[1] * v5_p
             + (error5_v[1] - error5_v[0]) * v5_d;
     
    if(output5 > 15)
    {
        output5 = 15;
    }
    
    if(output5 < -15)
    {
        output5 = -15;
    }
		 
		return -output5;
}
//chassis move measure
int ysigma,xsigma,asigma,last_ysigma,last_xsigma;
int ydelta,xdelta,adelta;
int xpos,ypos,apos;
int xx,yy,xtotal,ytotal;
float fai,theta;
int ma_total[4];	//forward = +, back = -\
extern float ZGyroModuleAngle;

void move_measure_hook(){
	
	
}


//actually this belong to gimbal will better
void reset_zgyro()
{
	while(ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_TX);
	ZGYRO_CAN.pTxMsg->StdId = 0x404;
	ZGYRO_CAN.pTxMsg->IDE = CAN_ID_STD;
	ZGYRO_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	ZGYRO_CAN.pTxMsg->DLC = 0x08;
	ZGYRO_CAN.pTxMsg->Data[0] = 0;
	ZGYRO_CAN.pTxMsg->Data[1] = 1;
	ZGYRO_CAN.pTxMsg->Data[2] = 2;
	ZGYRO_CAN.pTxMsg->Data[3] = 3;
	ZGYRO_CAN.pTxMsg->Data[4] = 4;
	ZGYRO_CAN.pTxMsg->Data[5] = 5;
	ZGYRO_CAN.pTxMsg->Data[6] = 6;
	ZGYRO_CAN.pTxMsg->Data[7] = 7;
	HAL_CAN_Transmit(&ZGYRO_CAN, 1000);
}


/**
	* @function 麦轮解算函数， 输入x和y方向速度， 旋转角度， 输出到4个轮子
	*	@bref omega: + = cw, - = ccw
	* @output wheel_speed[4];
	* @note : 1:FR, 2=FL 3=BL, 4=BR, ↑↓=Vy， ←→=Vx
		@map TYPE=1
			2	%++++++%	1
					++++
					++++
			3	%++++++%	4    ↑=+Vy  →=+Vx
	*/
void mecanum_calc(float vx, float vy, float omega, const int each_max_spd, s16 speed[]){
	s16 buf[4];
	int i;
	float max=0, rate;
	
	vx = vx > MAX_CHASSIS_VX_SPEED ? MAX_CHASSIS_VX_SPEED : vx;
	vx = vx < -MAX_CHASSIS_VX_SPEED ? -MAX_CHASSIS_VX_SPEED : vx;	
	vy = vy > MAX_CHASSIS_VY_SPEED ? MAX_CHASSIS_VY_SPEED : vy;
	vy = vy < -MAX_CHASSIS_VY_SPEED ? -MAX_CHASSIS_VY_SPEED : vy;
	omega = omega > MAX_CHASSIS_OMEGA_SPEED ? MAX_CHASSIS_OMEGA_SPEED : omega;
	omega = omega < -MAX_CHASSIS_OMEGA_SPEED ? -MAX_CHASSIS_OMEGA_SPEED : omega;
	
#ifdef DEBUG_REVERSE
	buf[0] = ( -vx + vy + omega );
	buf[1] = ( -vx - vy + omega );
	buf[2] = ( +vx + vy + omega );
	buf[3] = ( +vx - vy + omega );  //因为轮子方向的关系
#endif	
	
	
#ifdef CHASSIS_OLD
	buf[0] = ( -vx + vy - omega );
	buf[1] = ( -vx - vy - omega );
	buf[2] = ( +vx - vy - omega );
	buf[3] = ( +vx + vy - omega );  //因为轮子方向的关系
#else //这个不一定是正确的  等待修改
	buf[0] = ( vx + vy + omega );
	buf[1] = -( vx - vy - omega );
	buf[2] = ( vx - vy + omega );
	buf[3] = -( vx + vy - omega );  //因为轮子方向的关系
#endif
	//find max item
	for(i=0; i<4; i++){
		if ( MyAbs(buf[i]) > max )
			max = MyAbs(buf[i]) ;
	}
	//等比例缩放， 都是对于绝对值。 不允许单个轮子速度超过最大值，否则合成不正常
	if (max > each_max_spd){
		rate = each_max_spd / max;
		for(i=0; i<4; i++)
			buf[i] *= rate;
	}
	//output
	memcpy(speed, buf, sizeof(s16)*4); 
}
/**
	* @bref   send 4 calculated current to driver
	* @assume 3510 driver in current mode
	* @attention Do NOT change this function.for safety purpose.	//btw, i think you can't ~ ~
*/
void set_cm_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){
	
	memcpy((hcan->pTxMsg), pack_can_msg(iq1,iq2,iq3,iq4), sizeof(CanTxMsgTypeDef));
		
	HAL_CAN_Transmit(hcan, 1000);
	
}	

void get_chassis_mode_set_ref(RC_Type* rc){
	
	chassis.last_mode = chassis.mode;
	
	switch(rc->sw2){
		case RC_UP:
			chassis.mode = CHASSIS_FOLLOW_GIMBAL_ENCODER;	
			break;
		
		case RC_MI:
			chassis.mode = CHASSIS_FOLLOW_GIMBAL_ENCODER;	//release
			break;
		
		case RC_DN:
			//chassis.mode = CHASSIS_CLOSE_GYRO_LOOP;
			chassis.mode = CHASSIS_CLOSE_GYRO_LOOP;	//release
			
			break;
		default:
			
			break;
	}
	#ifdef SAFE_CHASSIS
			chassis.mode = CHASSIS_RELAX;	
	#endif
	
	
	switch(chassis.mode){
		case CHASSIS_CLOSE_GYRO_LOOP:
			{
				
				//chassis.vy = rc->ch2*0.5f*CHASSIS_RC_MOVE_RATIO_Y + km.vy*CHASSIS_PC_MOVE_RATIO_Y + tu_rc.bits.ch2;	//last item is or
				//chassis.vx = rc->ch1*0.5f*CHASSIS_RC_MOVE_RATIO_X + km.vx*CHASSIS_PC_MOVE_RATIO_X + tu_rc.bits.ch1;
			   chassis.vx = 0;
				 chassis.vy = 0;

			}
			break;
		case CHASSIS_OPEN_LOOP:
			{
//				chassis.vy = rc->ch2*CHASSIS_RC_MOVE_RATIO_Y + km.vy;
//				chassis.vx = rc->ch1*CHASSIS_RC_MOVE_RATIO_X + km.vx;
//				//chassis.omega = 0;
//				chassis.omega = rc->ch3/4 + rc->mouse.x*10;	//remove this will better
			}
			break;
		case CHASSIS_FOLLOW_GIMBAL_ENCODER:
			{
				chassis.vy = rc->ch2*0.5f*CHASSIS_RC_MOVE_RATIO_Y + km.vy*CHASSIS_PC_MOVE_RATIO_Y + tu_rc.bits.ch2;	//last item is or
				chassis.vx = rc->ch1*0.5f*CHASSIS_RC_MOVE_RATIO_X + km.vx*CHASSIS_PC_MOVE_RATIO_X + tu_rc.bits.ch1;
			//chassis.vz = rc->ch3*0.5f + km.vz + tu_rc.bits.ch3;			
				//omega according to yaw encoder pid calc
				//chassis.target_angle += rc->ch3 * 0.001f;
			}
			break;	
			
	}			
	
	
}

float scope_param[2];

void chassis_task(void const * argu){
	int i=0;
	chassis.vy2=0;
		
	for(int k=0; k<4; k++){
			/*	max current = 20000, it may cause deadly injury !!! just like me today*/
			PID_struct_init(&pid_spd[k], POSITION_PID, 20000, 20000,		//origin = 20000
									4,	0.05f,	0.0f	);  //4,	0.05f,	0.0f
	}
	PID_struct_init(&pid_chassis_angle, POSITION_PID, 300, 300,
									0.2f,	0.0f,	1.7f	);  
	PID_struct_init(&pid_vy, POSITION_PID, 200, 200,
									1.0f,	0.0f,	0.0f	);  
	chassis.vybiaoshi=0;
	//pid_chassis_angle.max_err = 80*22.75f;	//err angle > 60 cut the output,原来是60
	//pid_chassis_angle.deadband = 50;	//err angle <10 cut the output
	
	HAL_Delay(1000);
	//TODO : chassis follows gimbal. chassis follow gimbal encoder.
	while(1){
	 MAX_WHEEL_SPEED = speed_max_use.max_wheel_speed ;
		get_chassis_mode_set_ref(&rc);
		/*TODO: refactory this code, combo to omega*/
		
		//move_measure_hook();//you need to implement by yourself
		/* chassis follows gimbal yaw, just keep encoder = 0, ==> pid.set = 0*/
		
		/* CHASSIS_CLOSE_GYRO_LOOP actually NOT used yet# */
		if(chassis.mode == CHASSIS_CLOSE_GYRO_LOOP){	
			if(chassis.last_mode != CHASSIS_CLOSE_GYRO_LOOP){
				chassis.target_angle = chassis.angle_from_gyro;
			}
			//chassis.omega = -pid_calc(&pid_chassis_angle, chassis.angle_from_gyro, chassis.target_angle);	
		}
		/* CHASSIS_CLOSE_GYRO_LOOP actually NOT used yet */
		
		if(chassis.mode == CHASSIS_FOLLOW_GIMBAL_ENCODER
			&& rc.sw2 != RC_MI){	//need to change range(-xxxx -> 0 -> +xxxx)//done
			chassis.omega = pid_calc(&pid_chassis_angle, yaw_relative_pos, 0);	//chassis follow yaw, keep relative pos = 0 is good
		}
		else{
	 chassis.omega =0;      // pid_calc(&pid_chassis_angle, yaw_relative_pos, 0);
		}
		if ( fabs(chassis.vx) < 10) chassis.vx = 0;	//avoid rc stick have little offset
		if ( fabs(chassis.vy) < 10) chassis.vy = 0;
		if (chassis.is_snipe_mode) chassis.omega = 0;	//|| ABS(chassis.omega) < 10
		
		
		if (chassis.anti_attack)//escape gesture
		{
			chassis.omega = pid_calc(&pid_chassis_angle, yaw_relative_pos, 20);	
			osDelay(100);
			chassis.omega = pid_calc(&pid_chassis_angle, yaw_relative_pos, -20);	
		}

		
	
//		if(chassis.vy-chassis.vy1>0)chassis.vy2+=10;
//		if(chassis.vy2>40)chassis.vy2=40;
		
//				if(chassis.vy2>80)mecanum_calc(chassis.vx*30,chassis.vy1*2*30,chassis.omega*(-30), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)
//		if(chassis.vy2<80)

/*Beifen
				if(chassis.WW>3&&chassis.WW<200)chassis.vy2=chassis.WW;
		//if(chassis.WW<3)chassis.vy2=chassis.vy2;
    if(chassis.vy>0)
		{
			pid_calc(&pid_vy,chassis.vy2, chassis.vy/4);
			chassis.vy1=chassis.vy/4-pid_vy.pos_out;
		}
		if(chassis.vy<0)
		{
			pid_calc(&pid_vy,chassis.vy2, -chassis.vy/4);
			chassis.vy1=chassis.vy/4+pid_vy.pos_out;
		}
		if(chassis.vy==0)chassis.vy1=pid_vy.pos_out=chassis.vy;
   	if(chassis.vy2>80)mecanum_calc(chassis.vx*30,chassis.vy1*2*30,chassis.omega*(-30), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)
		if(chassis.vy2<80)mecanum_calc(chassis.vx*30,chassis.vy*30,chassis.omega*(-30), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)
		buff_3510iq[0] =chassis.wheel_speed.s16_fmt[0]+pidd(moto_chassis[0].speed_rpm,chassis.wheel_speed.s16_fmt[0]);
		buff_3510iq[1] =chassis.wheel_speed.s16_fmt[1]+pidd22(moto_chassis[1].speed_rpm,chassis.wheel_speed.s16_fmt[1]);
		buff_3510iq[2] =chassis.wheel_speed.s16_fmt[2]+pidd33(moto_chassis[2].speed_rpm,chassis.wheel_speed.s16_fmt[2]);
		buff_3510iq[3] =chassis.wheel_speed.s16_fmt[3]+pidd44(moto_chassis[3].speed_rpm,chassis.wheel_speed.s16_fmt[3]);


   	if(chassis.vy2>80)mecanum_calc(chassis.vx*30,chassis.vy1*4*30,chassis.omega*(-30), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)
		if(chassis.vy2<80)
		
		
		
		
		    if(chassis.vy>0)chassis.vy4=chassis.vy-chassis.vy3;
		if(chassis.vy<0)chassis.vy4=-chassis.vy+chassis.vy3;
		if(chassis.vy==0)chassis.vy4=chassis.vy;

		if(chassis.WW>3&&chassis.WW<200)chassis.vy2=chassis.WW;
		//if(chassis.WW<3)chassis.vy2=chassis.vy2;
    if(chassis.vy>0)
		{
			//pid_calc(&pid_vy,chassis.vy2, chassis.vy/4);
			chassis.vy1=chassis.vy/4+pidd55(chassis.vy2,chassis.vy/4);
			chassis.vx1=pidd55(chassis.vy2,chassis.vy/4);
		}
		if(chassis.vy<0)
		{
			//pid_calc(&pid_vy,chassis.vy2, -chassis.vy/4);
			chassis.vy1=chassis.vy/4-pidd55(chassis.vy2,-chassis.vy/4);
			chassis.vx1=pidd55(chassis.vy2,-chassis.vy/4);
		}
		if(chassis.vy==0)chassis.vy1=pid_vy.pos_out=chassis.vy;
		
    if(chassis.vy2>=70||chassis.vy4>=10)
		{
			
			mecanum_calc(chassis.vx*30,chassis.vy1*4*30,chassis.omega*(-30), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)
			chassis.vybiaoshi=1;
		}
		if(chassis.vy2<70&&chassis.vy4<10)
		{
			mecanum_calc(chassis.vx*30,chassis.vy*30,chassis.omega*(-30), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)
			chassis.vybiaoshi=0;
		}
*/
		
//		if(chassis.WW>80)chassis.vy1=chassis.vy/1.5f;
//		if(chassis.WW<=80)chassis.vy1=chassis.vy;chassis.vy
//		chassis.vy1=(chassis.vy)/(chassis.WW);
    chassis.vvv=abs(buff_3510iq[0]);

		if(chassis.WW>3&&chassis.WW<200)chassis.vy3=chassis.WW;
		if(chassis.vy3>=140||chassis.vvv>8000||(abs(buff_3510iq[1]))>8000||(abs(buff_3510iq[2]))>8000||(abs(buff_3510iq[3]))>8000)chassis.vy2++;
		if(chassis.vy3<140&&chassis.vvv<=8000&&(abs(buff_3510iq[1]))<=8000&&(abs(buff_3510iq[2]))<=8000&&(abs(buff_3510iq[3]))<=8000)chassis.vy2=0;
		//if(chassis.WW<3)chassis.vy2=chassis.vy2;
    if(chassis.vy2>50)
		{
      chassis.vy1=0;
			chassis.vx1=0;
			chassis.omega1=0;
			chassis.vybiaoshi=1;
			chassis.vxbiaoshi=1;
			chassis.omega_biaoshi=1;
		}
		
		if(chassis.vybiaoshi==1)chassis.vy1=0;;
		if(chassis.vy==0)chassis.vybiaoshi=0;		
		if(chassis.vy2<=50&&chassis.vybiaoshi==0)chassis.vy1=chassis.vy;
		
//		if(chassis.omega_biaoshi==1)chassis.omega1=0;
//		if(chassis.omega==0)chassis.omega_biaoshi=0;	
//		if(chassis.vy2<=50&&chassis.omega_biaoshi==0)chassis.omega1=chassis.omega;
		
		if(chassis.vxbiaoshi==1)chassis.vx1=0;;
		if(chassis.vx==0)chassis.vxbiaoshi=0;	
		if(chassis.vy2<=50&&chassis.vxbiaoshi==0)chassis.vx1=chassis.vx;

			mecanum_calc(chassis.vx1*30,chassis.vy1*30,chassis.omega*(-30), MAX_WHEEL_SPEED, chassis.wheel_speed.s16_fmt);//chassis.omega*(-30)

		buff_3510iq[0] =chassis.wheel_speed.s16_fmt[0]+pidd(moto_chassis[0].speed_rpm,chassis.wheel_speed.s16_fmt[0]);
		buff_3510iq[1] =chassis.wheel_speed.s16_fmt[1]+pidd22(moto_chassis[1].speed_rpm,chassis.wheel_speed.s16_fmt[1]);
		buff_3510iq[2] =chassis.wheel_speed.s16_fmt[2]+pidd33(moto_chassis[2].speed_rpm,chassis.wheel_speed.s16_fmt[2]);
		buff_3510iq[3] =chassis.wheel_speed.s16_fmt[3]+pidd44(moto_chassis[3].speed_rpm,chassis.wheel_speed.s16_fmt[3]);

		
		
		power_date=game_info_use.realChassisOutV*game_info_use.realChassisOutA;
		scope_param[0] = pid_spd[0].set[0];
		scope_param[1] = pid_spd[0].get[0];
	
		set_cm_current(	&CHASSIS_CAN,
										buff_3510iq[0], 
										buff_3510iq[1], 
										buff_3510iq[2], 
										buff_3510iq[3]);
		

		osDelay(10);
		
		
	}
}



#if 0	//this version is speed + power limit loop
void chassis_task(void const * argu){
	
	//0-3 FL FR BL BR
	speed_out_sum = 0;
	s16 tmp_real_current[4]={0};
	int real_current_sum = 0, left_current=0;
	
	memcpy(tmp_real_current, RealTimeCurrent.u8_fmt, 8);
  
	real_current_sum = tmp_real_current[0] + tmp_real_current[1] 
									+ tmp_real_current[2] + tmp_real_current[3];

	left_current = 3200 - real_current_sum;	//80W
	if(left_current < 0)left_current = 0;
	for(int k=0; k<4; k++){
		/*get realtime signed current assume Voltage = 25V */
		/*if averege , every moto can get 20W, I = 0.8A */
		tmp_real_current[k] = tmp_real_current[k] < 50? 0 : tmp_real_current[k];
		//对于绝对值小于50mA则清零
		if(moto_measure[k].speed_rpm < 0 && tmp_real_current[k] > 0)
			tmp_real_current[k] *= -1;	//规定测量电流正负与电机同方向

		/*limit accelerate of moto speed*/
		if(target_4moto_speed[k] > last_target_4moto_speed[k]	//speed up
			&& target_4moto_speed[k] > last_target_4moto_speed[k] + abs_target_speed_acc)
			target_4moto_speed[k] = last_target_4moto_speed[k] + abs_target_speed_acc;
		else if(target_4moto_speed[k] < last_target_4moto_speed[k] 	//speed down
			&& target_4moto_speed[k] < last_target_4moto_speed[k] - abs_target_speed_acc)
			target_4moto_speed[k] = last_target_4moto_speed[k] - abs_target_speed_acc;
		
#if 0
		pid_3510_speed[k].target = target_4moto_speed[k]*10;	//大约只有1000，扩大到10000
		last_target_4moto_speed[k] = target_4moto_speed[k];
		if(k%2 == 0) 
			buff_3510iq[k] 
			= pid_3510_speed[k].f_cal_pid(&pid_3510_speed[k], moto_measure[k].speed_rpm);
		else	//两外两个电机  
			buff_3510iq[k] 
			= -pid_3510_speed[k].f_cal_pid(&pid_3510_speed[k], -moto_measure[k].speed_rpm);
#endif
		
#if 1
		//pid_3510_speed[k].target = target_4moto_speed[k]*10;	//大约只有1000，扩大到10000
		
		last_target_4moto_speed[k] = target_4moto_speed[k];
		if(k%2 == 0) 
			buff_3510iq[k] = pid_speed[k].f_cal_pid(
			&pid_speed[k], moto_measure[k].speed_rpm, target_4moto_speed[k]*10);
		else	//两外两个电机  
			buff_3510iq[k] = -pid_speed[k].f_cal_pid(
			&pid_speed[k], -moto_measure[k].speed_rpm, target_4moto_speed[k]*10);
#endif

		const int max_current_target = 800;	//贪心！

		if(buff_3510iq[k] > (max_current_target + left_current/4) )
				buff_3510iq[k]  = (max_current_target + left_current/4);
		if(buff_3510iq[k] < -(max_current_target + left_current/4) )
			buff_3510iq[k]  = -(max_current_target + left_current/4);
		
		final_buff[k] = pid_curet[k].f_cal_pid(&pid_curet[k], tmp_real_current[k], buff_3510iq[k]);
//		final_sum += final_buff[k];	//求出计算出所需的总和
	
	}//end of for(k=0;k<4...) 
	

	if(HAL_GetTick() - latest_get_target_time > 200)
		target_chassis_speed_timeout = 1;
	else
		target_chassis_speed_timeout = 0;
	
	if(target_chassis_speed_timeout)
		memset(final_buff, 0, sizeof(final_buff));
	if(in_supply_mode)
		final_buff[2] = final_buff[3] = 0;
	
	can_send_msg();

}
#endif




/* 初始化底盘PID */
void chassis_pid_init(void)
{

}


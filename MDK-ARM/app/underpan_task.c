/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "bsp_can.h"
#include "can.h"
#include "dogconfig.h"
#include "underpan_task.h"
#include "pid.h"
//#include "cloud_task.h"
#include "bsp_flash.h"
#include "mytype.h"
#include "hero.h"
#include <math.h>
#include "i2c_oled.h"

#define MyAbs(x) 	( (x>0) ? (x) : (-x) )

volatile float Last_Spd[4] = {0,0,0,0};
const int Move_CalculateMAT[4][3] = {
{1, 1, 1},
{1,-1,-1},
{1,-1, 1},
{1, 1,-1}};//麦轮解算矩阵
/*  wheel speed 
spd[0] = ( vx + vy + omega );
spd[1] = ( vx - vy - omega );
spd[2] = ( vx - vy + omega )
spd[3] = ( vx + vy - omega )
*/

//UserConfig myConfig;
Underpan myUnderpan;


PID_TypeDef pid_3510_speed[4];
PID_TypeDef pid_3510_current[4];

PID_TypeDef	pid_gyro_omega;	//omega inner loop of double loop for chassis
PID_TypeDef pid_gyro_angle;	//angle outer loop of double loop 

s16 buff_3510_speed[4];


/*for debug*/
float look1;
float look2;
float look3;
float look4;
float y=0;
int lun_normal_flag[4]={0,0,0,0};


int ytest=0;
int only_one=1;
s16 final_buff[4];
float rc_total_angle=0;
float mouse_total_angle=0;
char is_chassis_slow_mode=0;
float afg_without_gyro;//不包含陀螺仪的角度的！它的陀螺仪角度跟我是反的！！！
float afg_with_gyro;  //从云台发过来的角度，这角度里包含了底盘陀螺仪。use!
//#define CLOSE_ANGLE
#define CLOSE_ANGLE_OMEGA
//#define CLOSE_OMEGA --done!
/*if angle = 10, in my code, then is -10 in hzw code, afg =  20 + angle*/

void UnderpanTask(void const * argument)
{
  //不要复位陀螺仪！！！
  osDelay(3000);//wait gyro reset
	initUnderpanPID();
	
  for(;;)
  {

		if(RC_UP == rc.switch_left){	//闭环hn
//      if(only_one){
//        resetGryo();
//				osDelay(1000);
//        only_one = 0;
//      }
		//myUnderpan.target_angle = myUnderpan.mouse_x_integral;//改变底盘目标角度
			rc_total_angle += rc.ch3*0.003;
			mouse_total_angle += rc.mouse.x * 0.02;
			
			if(myUnderpan.is_snipe_mode){	//狙击模式就不跟随云台
        //myUnderpan.target_angle = rc_total_angle + kb.angle + mouse_total_angle;
				
			}else{//不是狙击模式就跟踪炮的云台 ,因为志伟发过来的和我是反的！所以加符号
        myUnderpan.target_angle = -1 * myUnderpan.angle_from_gimbal.f32t;
				//target_omega = pid_gyro_angle.f_cal_pid(&pid_gyro_angle, myUnderpan.angle_from_gimbal.f32t);//底盘陀螺仪PID计算
			}
      
      pid_gyro_angle.target = myUnderpan.target_angle;
      float target_omega = pid_gyro_angle.f_cal_pid(&pid_gyro_angle, myUnderpan.angle_from_gyro);//底盘陀螺仪PID计算
		#ifdef CLOSE_ANGLE
			myUnderpan.my_omega = target_omega;
		#elif defined CLOSE_ANGLE_OMEGA
			pid_gyro_omega.target = target_omega;
			myUnderpan.my_omega = pid_gyro_omega.f_cal_pid(&pid_gyro_omega, myUnderpan.omega_from_gyro);
		#endif
    
		#ifdef CLOSE_OMEGA
			pid_gyro_omega.target = rc.ch3 * 0.8;
			myUnderpan.my_omega = pid_gyro_omega.f_cal_pid(&pid_gyro_omega, myUnderpan.omega_from_gyro);
		#endif
			//uart_send2pc(&BT_HUART, Monitor1, myUnderpan.omega_from_gyro);
			//uart_send2pc(&BT_HUART, Monitor2, pid_gyro_omega.target);
			//uart_send2pc(&BT_HUART, Monitor3, pid_gyro_omega.output);
      
      if(eLandingState >= StandOnGNDReadyLanding &&
        eLandingState <= CatchEggsDoneSeatOnIsland)
      {
        is_chassis_slow_mode = 1;  
      }
      else
        is_chassis_slow_mode = 0;
      
      float errK = MyAbs(pid_gyro_angle.err);
      errK = exp(errK * -0.03f) ;
      //errK = 1.0f;
      if(is_chassis_slow_mode){
        myUnderpan.my_speedx = 0.4f * (rc.ch2*0.8 + kb.speed_x);
        myUnderpan.my_speedy = 0.4f * (rc.ch1*0.8 + kb.speed_y);
        }else{  //normal mode. 加这个是因为上岛后要慢
        myUnderpan.my_speedx = rc.ch2*0.8 + kb.speed_x;
        myUnderpan.my_speedy = rc.ch1*0.8*errK + kb.speed_y;
          
        
       // uart_send2pc(&BT_HUART, Monitor2, myUnderpan.my_speedy);
        //uart_send2pc(&BT_HUART, Monitor3, myUnderpan.my_speedx);
      }
    }
    else if (RC_MI == rc.switch_left){ //open loop
      myUnderpan.my_speedx = rc.ch2*0.7 + kb.speed_x;
      myUnderpan.my_speedy = rc.ch1*0.7 + kb.speed_y;
      myUnderpan.my_omega	= rc.ch3*0.6;
			only_one = 1;
    }
		else{
			myUnderpan.my_speedx = 0;
      myUnderpan.my_speedy = 0;
      myUnderpan.my_omega	= 0;
      
			
		}
		
    
    
    
   // uart_send2pc(&BT_HUART, Monitor4, myUnderpan.angle_from_gyro );
    //uart_send2pc(&BT_HUART, Monitor5, pid_gyro_angle.target);
   // uart_send2pc(&BT_HUART, Monitor6, myUnderpan.angle_from_gimbal.f32t);
//		uart_send2pc(&BT_HUART, Monitor5, pid_3510_current[0].target);	
//		uart_send2pc(&BT_HUART, Monitor6, pid_3510_current[0].output);	
//    
		//麦轮方程解算出四个轮子的速度
		CalculateSpeed(myUnderpan.my_speedx, myUnderpan.my_speedy, myUnderpan.my_omega);

		s16 RealCurrent[4];
		memcpy((void*)RealCurrent, (void*)RealTimeCurrent.U16type, 8);
		for(int k=0; k<4; k++){
			pid_3510_speed[k].target = myUnderpan.underpan_speed.s16_form[k] * 10;	//大约只有1000，扩大到10000
			if(k%2 == 0) 
				buff_3510_speed[k] = pid_3510_speed[k].f_cal_pid(&pid_3510_speed[k], Moto3510Measure[k].speed_rpm);
			else	//两外两个电机  
				buff_3510_speed[k] = -pid_3510_speed[k].f_cal_pid(&pid_3510_speed[k], -Moto3510Measure[k].speed_rpm);
		}
    float max_current_target=1200;
    //float max_current_target[4] = {1400, 1400, 1400, 1400};
    
    float total_need_current = (float)buff_3510_speed[0]+(float)buff_3510_speed[1]+
      (float)buff_3510_speed[2]+(float)buff_3510_speed[3];
    //前面两个功率限制的小 后面两个功率大
		for(int k=0; k<4; k++){
      //max_current_target= (float)buff_3510_speed[k]/(total_need_current/5500.0f) ;
			//if(max_current_target>3500)
        //max_current_target=3500;
      //else if(max_current_target<-3500)
         //max_current_target=-3500;      
      //电流值限幅
			if(buff_3510_speed[k] >max_current_target)
				buff_3510_speed[k]  = max_current_target;
			if(buff_3510_speed[k] <- max_current_target)
				buff_3510_speed[k]  = -max_current_target;
      
			//pid_3510_current[k].target=y;
			pid_3510_current[k].target = buff_3510_speed[k];      	
    
      RealCurrent[k] = RealCurrent[k] < 50? 0 : RealCurrent[k];
      
			if(Moto3510Measure[k].speed_rpm < 0 && RealCurrent[k] > 0)
				RealCurrent[k] *= -1;	//规定测量电流正负与电机同方向
      //if(Moto3510Measure[k].speed_rpm==0)//轮子不转没电流
        //RealCurrent[k]=0;
			final_buff[k] = pid_3510_current[k].f_cal_pid(&pid_3510_current[k], RealCurrent[k]);
      //输出电流很大 可是测量电流很小
			if( MyAbs(pid_3510_current[k].output) > 9000 && MyAbs(pid_3510_current[k].measure) < 200)
      {
        pid_3510_speed[k].target=0;
        pid_3510_current[k].iout=0;
        pid_3510_current[k].output=0;
        pid_3510_speed[k].iout=0;
        pid_3510_speed[k].output=0;
        final_buff[k]=0;
        lun_normal_flag[k]++;
      }
      else if(lun_normal_flag[k]>0)
      {
        lun_normal_flag[k]++;//还处于电调保护之中
        if(lun_normal_flag[k]<20)
        {
          pid_3510_speed[k].target=0;
          pid_3510_current[k].iout=0;
          pid_3510_current[k].output=0;
          pid_3510_speed[k].iout=0;
          pid_3510_speed[k].output=0;
          final_buff[k]=0;
        }
        else
          lun_normal_flag[k]=0;
      }
      else
        lun_normal_flag[k]=0;
		}		
    uart_send2pc(&BT_HUART, Monitor4, pid_3510_speed[0].measure);       
    uart_send2pc(&BT_HUART, Monitor5, pid_3510_current[0].measure);	
    uart_send2pc(&BT_HUART, Monitor6, pid_3510_current[0].output);	 
//    int total = RealCurrent[0] + RealCurrent[1] + RealCurrent[2] + RealCurrent[3];
		//rcDataTable[Monitor4].F32type = RealCurrent[0];	//measure	
		

    if(RC_DN == rc.switch_left){
			final_buff[0] = final_buff[1] = final_buff[2] = final_buff[3] = 0;
		}
    
    
			CAN_Send_Message(&CHASSIS_CAN, CAN_3510Moto_four_ID, final_buff);
//		}
		//look1 = sendMessageToCANQueue(CAN_Four_Motor_ID, myUnderpan.underpan_speed.s16_form);
		osDelay(5); 
		
  }
}     

/*麦轮解算函数 左前0 右前1 左后2 右后3 */
void CalculateSpeed(float myVx,float myVy,float omega)
{
	unsigned char ii = 0, jj = 0;
	float temp_mat[3] = {0,0,0};
	float max_spd = 0,temp_speed = 0;
	float temp_ration = 0;	
	float temp_spd[4] = {0,0,0,0};
	float my_spd[4] = {0,0,0,0};
  int under_acc = 15;
	temp_mat[0] = myVx;
	temp_mat[1] = myVy;
	temp_mat[2] = omega;
	for(ii = 0;ii<4;ii++)
	{
		for(jj = 0;jj<3;jj++)
		{
			my_spd[ii] += temp_mat[jj]*Move_CalculateMAT[ii][jj];    
		}
		if(my_spd[ii] >= Last_Spd[ii])  //加速过程， 之前这里写的好像有毒，已经重写
		{
     if(my_spd[ii] - Last_Spd[ii] > under_acc)
        my_spd[ii] = Last_Spd[ii] + under_acc;
      temp_speed = my_spd[ii];
		}
		else if(my_spd[ii] < Last_Spd[ii])  //减速过程 必须限制，否则导致旋转时后退！！！
		{
      if(my_spd[ii] - Last_Spd[ii] < -under_acc)
        my_spd[ii] = Last_Spd[ii] - under_acc;
      temp_speed = my_spd[ii];
      //限减速
		}
		my_spd[ii] = temp_speed;
		Last_Spd[ii] = my_spd[ii];
	}
	for(ii = 0;ii<4;ii++)  //找到速度最大值
	{
		temp_speed = MyAbs(temp_spd[ii]);
		max_spd = (max_spd>temp_speed) ? max_spd:temp_speed;
	}
	if( max_spd > Max_WheelSpeed )//按比例缩小
	{
		temp_ration = Max_WheelSpeed/max_spd;
		for(ii = 0;ii<4;ii++)
		{
			my_spd[ii] = my_spd[ii]*temp_ration; 
		}	
	}
	for(ii = 0;ii<4;ii++)//设置发送速度
	{
		myUnderpan.underpan_speed.s16_form[ii] = (int)my_spd[ii]; 
	} 
	
}


void resetGryo(void)//复位陀螺仪
{
	myUnderpan.angle_from_gyro=0;
	myUnderpan.target_angle=0;
	CAN_Send_Message(&hcan1, CAN_GyroReset_ID, NULL);
}


/* 初始化底盘PID */
void initUnderpanPID(void)
{
	PID_Init(  //陀螺仪 angle pid初始化
						&pid_gyro_angle,	//* pid
						PID_Gryo,			//id
						500,					// maxout  这里为速度上下限
						65535,				// intergral_limit
						0,						// deadband,
						10,						//period
						30000,				//max_err,
						0,    			//target,
						8.5,					//float 	kp  10
						0,					//float 	ki
						300);				//float 	kd 100
	
	PID_Init(  //陀螺仪 omega pid初始化
						&pid_gyro_omega,	//* pid
						PID_Gryo,			//id
						500,					// maxout  这里为速度上下限
						65535,				// intergral_limit
						0,						// deadband,
						10,						//period
						30000,				//max_err,
						0,    			//target,
						0.75,					//float 	kp  10 0.75
						0,					//float 	ki
						0);				//float 	kd 100
	
	for(int i=0; i<4; i++){
		PID_Init(  //3510 speed pid初始化
							&pid_3510_speed[i],	//* pid
							PID_3510_1,	//id
							1200,				// maxout  这里为abs速度上下限
							1150,				// intergral_limit, abs
							0,					// deadband,
							5,					//period
							30000,			//max_err,
							0,    			//target,
							8.8,				//float 	kp  10 20 10 8
							0.01,				//float 	ki 0.7 0.7
						  60 );			//float 	kd 100 100 80 40
		pid_3510_speed[i].max_acc=250;
		PID_Init(  //3510 scurrent pid初始化
			
    &pid_3510_current[i],	//* pid
							PID_3510_Current1,	//id
							14000,			// maxout  这里为abs速度上下限
							13000,			// intergral_limit, abs
							0,					// deadband,
							5,					//period
							30000,			//max_err,
							0,    			//target,
							0.3,					//float 	kp  10
							0.15,					//float 	ki 0.12
							0);					//float 	kd 100
    //pid_3510_current[i].max_acc=500;
	}

}


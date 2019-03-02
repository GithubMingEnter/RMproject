/********************************************************************************************************\
*                                     DJI FreeRTOS TASK PART 
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
\********************************************************************************************************/
/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  langgo
  * @version V1.0
  * @date    2016-11-30
  * @brief   
	* @update		
		@update 2   After this version, gimbal's encoder feedback position range [-xxxx , +xxxx], symmetric by 0，
								after call @function s16 get_relative_pos(s16 raw_ecd, s16 center_offset) will
								get a position relative to the center is center_encoder/offset= GimbalCaliData.GimbalPit/YawOffset;
								all input refence range also = [-xxxx, xxxx], Symmetric by 0
		@update 3   when rc sw2 = RC_DN, stop everything
	*	@attention			
  * @verbatim you should comment with english avoid diff text encoder cause ?!$@#%^@$%&$*
	****************************************************************************
	*/

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


#define VAL_LIMIT(x,min,max)	\
do{									\
	if( (x) < (min) )	\
		(x) = (min);		\
	if( (x) > (max) )	\
		(x) = (max);		\
}while(0)						\


float yaw_pos_ref = 0;
float pit_pos_rc_int = 0;		//rc_integral 
float pit_pos_mouse_int;		//mouse_integral 
float pit_pos_ref =  0;			//rc_integral + mouse_integral 
float yaw_pos_rc_int = 0;		//rc_integral 
float yaw_pos_mouse_int;		//mouse_integral 
int pit_center_offset = 0;	//read from flash. == ecd value when gimbal pit in center position
int yaw_center_offset = 0;	//read from motor esc when yaw axis in center.		//this will read from flash in the future. 
int flag_poker=0;
int flag_yaw=1;            //大幅模式flag
int flag_pit=1;            //大幅模式flag
int xxx,jjj;
float datepit;
float dateyaw;
float date_real_pit;
/*gimbal move delta position, init value = 0, range (-MOVE_RANGE, MOVE_RANGE)*/
s16 pit_relative_pos,yaw_relative_pos;	//unit : encoder 
s16 yaw_speed,yaw_angle1,yaw_angle2;
float watch_param[4];	//for jscope watch
float watch_pit[4];
int last_sw1;
int fric_wheel_run=0;	//run or not 
int poke_spd_ref,poke_pos_ref;	//poke bullet speed or 
int enter_stanby_mode_cnt;
u8  mouse_left_sta=0;
u8  mouse_right_sta=0;
float imu_tmp,imu_tmp_ref=40;
gimbal_yaw_t gYaw;
extern float euler[];
int reset_sequence=0;
int poke_stuck_cnt=0;
int poke_dir=1;//1
float pit_top_max;
float pit_dow_max;
float yaw_lift_max;
float yaw_righ_max;
float pit_change_range;
float yaw_change_range;

/*===============auto shoot big buff part ===============*/

s16 big_buff_table[9][2];					//this table represents 9 points,point format = [yaw,pit]
u8 manifold_cv_cmd;	//watch which mode pc is
u8 autoshot_cmd=0;
// u8  mainflod_use_date[2];//used for shot bullet when big buff num changed, such as 1->5
s16 img_center_yaw,img_center_pit;	//read from flash.when camera assemble position changed, need cali 
u8	is_in_big_buff_mode;		//1 = //unsed now
/*===============auto shoot big buff part ===============*/

#define CMD_TEST_UART		0xF1
#define CMD_FIND_NUM		0xF3
#define CMD_CALI_5			0XF4
#define CMD_CALI_9			0xF5
#define CMD_DEBUG				0xFF	//return num, yaw+pit
/**
	*	@param cmd is upon	5 CMD xxxx
	*/


void shot_big_buff_init(){

	if(gAppParam.CameraCali.isAlreadyCalied == CALIED_FLAG){
		img_center_yaw = gAppParam.CameraCali.GimbalYawOffset;
		img_center_pit = gAppParam.CameraCali.GimbalPitOffset;
	}
	/*TODO : if camera NOT calied before, DO NOT allow enter big buff mode*/
}
/** @map  
		yaw+	
		<-----0						1		2		3
					|						4		5		6
					↓	/pit+			7		8		9,  5=(0,0) + offset(y5,p5);
	*	@note this pit yaw is always relative!*/
void update_gimbal_position_table(s16 y5,s16 p5, s16 y9,s16 p9 ){
	u8 i;
	s16 deltaP,deltaY;
	
	deltaP = abs(p9-p5);
	deltaY = abs(y9-y5);
	//9 line 2 col
	//this table represents 9 points,point format = [yaw,pit]
	s16 tab[9][2] = 
	{
		deltaY,-deltaP,			0, -deltaP,			-deltaY, -deltaP,
		deltaY,			 0,			0, 			 0,			-deltaY, 			 0,
		deltaY, deltaP,			0,  deltaP,			-deltaY,  deltaP,
	};
	for(i=0; i<9; i++){
		big_buff_table[i][0] = tab[i][0] + y5;	//point5 is offset
		big_buff_table[i][1] = tab[i][1] + p5;
	}	
}

void gimbal_point2target(u8 num){
	if (num>8) return;
	gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ENCODER;
// 	yaw_pos_ref = big_buff_table[num][0];
//	pit_pos_ref = big_buff_table[num][1] - 20;
	//pit add -20, because of gravity make gun a little down(pit)
}

//eg: full image = 1280*720, so image center = 640*360
//it have offset of cause, so add offset will be [xxx,xxx]
//#define IMG_CENTER_YAW	775	//this 2 offset read from flash when calied before.
//#define IMG_CENTER_PIT	459


int t[4];	//just for test
int ex=999;
int ey=999;
int done5,done9;
enum{
	STA_5=1,
	STA_9,
	STA_AUTO,
};
void AUTO_buff_shot()
{
pit_change_range=abs(pit_top_max- pit_dow_max);
	yaw_change_range=abs(yaw_lift_max-yaw_righ_max);
	
	datepit= abs(CV_bigbuff_PandY.pit3_3-CV_bigbuff_PandY.pit2_2);
	dateyaw= abs(CV_bigbuff_PandY.yaw3_3-CV_bigbuff_PandY.yaw2_2);
	date_real_pit=abs(CV_bigbuff_PandY.pit_rea_3_3 - CV_bigbuff_PandY.pit_rea_2_2);
	if(1)
	{
		CV_bigbuff_PandY.yaw1_1=CV_bigbuff_PandY.yaw2_2+dateyaw;
	  CV_bigbuff_PandY.pit1_1=CV_bigbuff_PandY.pit2_2-datepit;
	//  CV_bigbuff_PandY.pit_rea_1_1=	CV_bigbuff_PandY.pit_rea_2_2-date_real_pit;
	}
	
	if(1)
	{
		CV_bigbuff_PandY.yaw1_2=CV_bigbuff_PandY.yaw2_2;
	  CV_bigbuff_PandY.pit1_2= CV_bigbuff_PandY.pit2_2-datepit;
		//CV_bigbuff_PandY.pit_rea_1_1=	CV_bigbuff_PandY.pit_rea_2_2-date_real_pit;
	}
	if(1)
	{
		CV_bigbuff_PandY.yaw1_3=CV_bigbuff_PandY.yaw2_2-dateyaw;
	  CV_bigbuff_PandY.pit1_3=CV_bigbuff_PandY.pit2_2-datepit;
		//CV_bigbuff_PandY.pit_rea_1_1=	CV_bigbuff_PandY.pit_rea_2_2-date_real_pit;
	}
	if(1)
	{
		CV_bigbuff_PandY.yaw2_1=CV_bigbuff_PandY.yaw2_2+dateyaw;
	  CV_bigbuff_PandY.pit2_1=CV_bigbuff_PandY.pit2_2;
		//CV_bigbuff_PandY.pit_rea_1_1=	CV_bigbuff_PandY.pit_rea_2_2;
	}
	if(rc.kb.bit.Q&&rc.kb.bit.SHIFT)
	{
		CV_bigbuff_PandY.yaw2_2= gYaw.zgyro_target;
	  CV_bigbuff_PandY.pit2_2= pit_pos_ref;
		//CV_bigbuff_PandY.pit_rea_2_2=pit_relative_pos;
		
	}
	if(1)
	{
		CV_bigbuff_PandY.yaw2_3=CV_bigbuff_PandY.yaw2_2-dateyaw;
	  CV_bigbuff_PandY.pit2_3=CV_bigbuff_PandY.pit2_2;
		//CV_bigbuff_PandY.pit_rea_1_1=	CV_bigbuff_PandY.pit_rea_2_2;
	}
	if(1)
	{
		CV_bigbuff_PandY.yaw3_1=CV_bigbuff_PandY.yaw2_2+dateyaw;
	  CV_bigbuff_PandY.pit3_1=CV_bigbuff_PandY.pit2_2+datepit;
		//CV_bigbuff_PandY.pit_rea_1_1=	CV_bigbuff_PandY.pit_rea_2_2+date_real_pit;
	}
	if(1)
	{
		CV_bigbuff_PandY.yaw3_2=CV_bigbuff_PandY.yaw2_2;
	  CV_bigbuff_PandY.pit3_2=CV_bigbuff_PandY.pit2_2+datepit;
			//CV_bigbuff_PandY.pit_rea_1_1=	CV_bigbuff_PandY.pit_rea_2_2+date_real_pit;
	}
	
	if(rc.kb.bit.E&&rc.kb.bit.SHIFT)
	{
		CV_bigbuff_PandY.yaw3_3=gYaw.zgyro_target;
	  CV_bigbuff_PandY.pit3_3=pit_pos_ref;
		//CV_bigbuff_PandY.pit_rea_3_3=pit_relative_pos;
	}
	
	
	
	
	
			if( cv_big_buff.cmd_use_control == 0xf1){	//cv track big buff
					
				  gYaw.zgyro_target =CV_bigbuff_PandY.yaw1_1;
	        pit_pos_ref =CV_bigbuff_PandY.pit1_1;
			//	  pit_relative_pos=CV_bigbuff_PandY.pit_rea_1_1;
	
              if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw1_1&&  pit_pos_ref == CV_bigbuff_PandY.pit1_1){
								
							  osDelay(50);
								flag_poker=1;
						//		cv_big_buff.cmd_use_control=0x00;
								
							}
		  //    flag_poker=1;
			} 		
	if( cv_big_buff.cmd_use_control == 0xf2){	//cv track big buff
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw1_2;
	        pit_pos_ref=CV_bigbuff_PandY.pit1_2;
			//pit_relative_pos=CV_bigbuff_PandY.pit_rea_1_2;
			 if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw1_2&& pit_pos_ref == CV_bigbuff_PandY.pit1_2){
							  osDelay(50);
								flag_poker=1;
				  //  cv_big_buff.cmd_use_control=0x00;
								
							}
		}
	 if( cv_big_buff.cmd_use_control == 0xf3){	//cv track big buff
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw1_3;
	        pit_pos_ref =CV_bigbuff_PandY.pit1_3;
		 //pit_relative_pos=CV_bigbuff_PandY.pit_rea_1_3;
		  if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw1_3&&  pit_pos_ref == CV_bigbuff_PandY.pit1_3){
		
							  osDelay(50);
								flag_poker=1;
			  //    	cv_big_buff.cmd_use_control=0x00;
								
							}
	}
		if( cv_big_buff.cmd_use_control== 0xE4){	         //有bug
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw2_1;
	        pit_pos_ref =CV_bigbuff_PandY.pit2_1;
		//	pit_relative_pos=CV_bigbuff_PandY.pit_rea_2_1;
			 if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw2_1&&  pit_pos_ref == CV_bigbuff_PandY.pit2_1){
				 
							  osDelay(50);
								flag_poker=1;
			//cv_big_buff.cmd_use_control=0x00;
								
							}
		}
  if( cv_big_buff.cmd_use_control == 0xf5){	//cv track big buff
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw2_2;
	        pit_pos_ref =CV_bigbuff_PandY.pit2_2;		
		//pit_relative_pos=CV_bigbuff_PandY.pit_rea_2_2;
		 if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw2_2&&  pit_pos_ref == CV_bigbuff_PandY.pit2_2){
			 
							  osDelay(50);
								flag_poker=1;
			//cv_big_buff.cmd_use_control=0x00;
								
							}
}
	 if( cv_big_buff.cmd_use_control == 0xf6){	//cv track big buff
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw2_3;
	        pit_pos_ref=CV_bigbuff_PandY.pit2_3;
		// pit_relative_pos=CV_bigbuff_PandY.pit_rea_2_3;
		  if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw2_3&&  pit_pos_ref == CV_bigbuff_PandY.pit2_3){
				     
							  osDelay(50);
								flag_poker=1;
			    // cv_big_buff.cmd_use_control=0x00;
								
							}
	}
	 if( cv_big_buff.cmd_use_control == 0xf7){	//cv track big buff
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw3_1;
	       pit_pos_ref =CV_bigbuff_PandY.pit3_1;
		// pit_relative_pos=CV_bigbuff_PandY.pit_rea_3_1;
		  if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw3_1&&  pit_pos_ref == CV_bigbuff_PandY.pit3_1){
				   
							  osDelay(50);
								flag_poker=1;
			  // cv_big_buff.cmd_use_control=0x00;
								
							}
	}
		if( cv_big_buff.cmd_use_control == 0xf8){	//cv track big buff
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw3_2;
	       pit_pos_ref =CV_bigbuff_PandY.pit3_2;
		//	pit_relative_pos=CV_bigbuff_PandY.pit_rea_3_2;
			 if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw3_2&& pit_pos_ref == CV_bigbuff_PandY.pit3_2){
							  osDelay(50);
								flag_poker=1;
				   //  cv_big_buff.cmd_use_control=0x00;
								
							}
		}
		if( cv_big_buff.cmd_use_control == 0xf9){	//cv track big buff
					gYaw.zgyro_target =CV_bigbuff_PandY.yaw3_3;
	       pit_pos_ref =CV_bigbuff_PandY.pit3_3;
		//	pit_relative_pos=CV_bigbuff_PandY.pit_rea_3_3;
			 if(gYaw.zgyro_target ==CV_bigbuff_PandY.yaw3_3&&  pit_pos_ref == CV_bigbuff_PandY.pit3_3){
							  osDelay(50);
								flag_poker=1;
				  //   cv_big_buff.cmd_use_control=0x00;
								
							}
			}

			
			   if (rc.sw2 != RC_DN )
	 {
		 	flag_poker=0;
		 cv_big_buff.cmd_use_control=0x00; 
	 }
  if ( flag_poker&&rc.sw2 == RC_DN  ){
		poke_pos_ref += 135863*poke_dir;	//60'理论值=136530 实际测100圈 get 135863 stable as dog    1.2
		  osDelay(10);
		 	cv_big_buff.cmd_use_control=0x00;
			flag_poker=0;
     if(poke_stuck_cnt > 30)	//1.5s
		{
			poke_stuck_cnt=0;
			moto_poke.round_cnt = 0;
			poke_pos_ref = 0;
    	poke_dir = -poke_dir;
		  }
		 } 
	
		 
}
void autoshot_big_buff(){
	//first move laser to center(5), remember here as img y5,p5.
	//send CMD_CALI_5 enter cali 5 mode, manifold return image position ,
	//control gimbal move to image y5,p5, record p y relative pos,
	//control gimbal move to image y9 p9, record p y relative pos.\
	//call @func update_gimbal_position_table(...)
	//switch cmd to CMD_FIND_NUM, ready to shoot
	//a key to triger big buff mode.
	//ps. wait to add more protect code when cali 5+9
	/*============== accoding to camera cali_point_5_9 ==============*/
	
	
	
//////////////////////////////////////////////////////////////////////////////////
	// deal_info_date();
	
	
	
	if(rc.kb.bit.V&&rc.sw2 == RC_DN)
	{
		osDelay(70);
	if(rc.kb.bit.V&&rc.sw2 == RC_DN)
	{	
		
	flag_yaw=-flag_yaw; 
	flag_pit=-flag_pit;	
	}	
		osDelay(100);
	if (flag_yaw<0)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
	
	}
if (flag_yaw>0)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);

}
	}
	

	/*============== accoding to camera cali_point_5_9 ==============*/
	
	
	update_gimbal_position_table(cv_big_buff.calied_y5,
																	cv_big_buff.calied_p5,
																	cv_big_buff.calied_y9,
																	cv_big_buff.calied_p9);
	/*============== start recognise and shooting ==============*/

		


}


/**
	*@aim 		get position relative to center,center = GimbalCaliData.GimbalPit/Yaw Offset;
	*@range  	gimbal move range = [center-MOVE_RANGE, center+MOVE_RANGE]
	*@note	 	process when angle cross 0 !
	*/
s16 get_relative_pos(s16 raw_ecd, s16 center_offset){
	s16 tmp=0;
	if(center_offset >= 4096){
		if(raw_ecd > center_offset - 4096)
			tmp = raw_ecd - center_offset; 
		else
			tmp = raw_ecd + 8192 - center_offset;
		
	}
	else{
		if(raw_ecd > center_offset + 4096)
			tmp = raw_ecd - 8192 - center_offset ; 
		else
			tmp = raw_ecd - center_offset;
	}
	return tmp;
}



void can_send_gimbal_iq(s16 yaw_iq, s16 pit_iq, s16 poke_iq){
	
	GIMBAL_CAN.pTxMsg->StdId = 0x1ff;
	GIMBAL_CAN.pTxMsg->IDE = CAN_ID_STD;
	GIMBAL_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	GIMBAL_CAN.pTxMsg->DLC = 8;
	GIMBAL_CAN.pTxMsg->Data[0] = yaw_iq>>8;
	GIMBAL_CAN.pTxMsg->Data[1] = yaw_iq;
	GIMBAL_CAN.pTxMsg->Data[2] = pit_iq>>8;
	GIMBAL_CAN.pTxMsg->Data[3] = pit_iq;
	GIMBAL_CAN.pTxMsg->Data[4] = poke_iq>>8;
	GIMBAL_CAN.pTxMsg->Data[5] = poke_iq;
	GIMBAL_CAN.pTxMsg->Data[6] = 0;
	GIMBAL_CAN.pTxMsg->Data[7] = 0;
	HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
	
}








/**
	*@bref - pitch + yaw gimbal double loop control
	*@TODO : ADD speed pid loop, done!
	*/
void gimbal_task(const void *argu){
//PID_struct_init(&pid_pit, POSITION_PID, 4000, 1000,
//									17.0f,	0,	25.1f);	//15.0f,	0.08f,	10.1f     //15 0 0/10.1f
//	PID_struct_init(&pid_pit_omg, POSITION_PID, 4000, 400,
//									5.0f,	-1.8,	0.0f	);
		PID_struct_init(&pid_pit, POSITION_PID, 4000, 1000,
									15.0f,	0,	10.1f);	//15.0f,	0.08f,	10.1f     //15 0 0/10.1f
	PID_struct_init(&pid_pit_omg, POSITION_PID, 4000, 400,
									12.0f,	0.8,	0.0f	);
/*PID_struct_init(&pid_pit, POSITION_PID, 4000, 1000,
								 3.0f,	0,	10.1f);	//15.0f,	0.08f,	10.1f     //15 0 0/10.1f
	PID_struct_init(&pid_pit_omg, POSITION_PID, 4000, 400,
									20.0f,	0.08,	0.0f	);*/
	

	
	PID_struct_init(&pid_poke_omg, POSITION_PID, 5000, 1000,		//omega   8
									1,	0,	0); 	
	PID_struct_init(&pid_poke, POSITION_PID, 5000, 1000,//position
									0.6,	0.0f,	0);   //0.45

	PID_struct_init(&pid_imu_tmp, POSITION_PID, 999, 999,//position
									180,	0.1f,	0);	
	PID_struct_init(&pid_cali_bby, POSITION_PID, 999, 999,//position
									0.4,		0.03f,	0);	
	PID_struct_init(&pid_cali_bbp, POSITION_PID, 999, 999,//position
									0.4,		0.03f,	0);	
	/* mpu6500 & istxxx magnet meter init ,for angular rate measure*/
	mpu_device_init();
	shot_big_buff_init();
	//read gimbal offset from gAppParam.//if not calied before , not allow gimbal run
//	if(gAppParam.GimbalCaliData.isAlreadyCalied == CALIED_FLAG){
//		yaw_center_offset = gAppParam.GimbalCaliData.GimbalYawOffset;
//		pit_center_offset = gAppParam.GimbalCaliData.GimbalPitOffset;
//	}
//	
  yaw_center_offset= 3890; //云台电机编码器中间值   //need change //5000，原来是3790,5630，6300
	gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ENCODER;//hide encoder mode!!!
	gYaw.zgyro_offset = get_relative_pos(moto_yaw.angle, yaw_center_offset)/22.75f;	//get offset修改陀螺仪中间值、moto_yaw.angle观测
	yaw_speed=0;//
	
	//transform  absolute ecd range [0,8192] to relative range [-MOVE_RANGE, MOVE_RANGE],
//	yaw_relative_pos = get_relative_pos(moto_yaw.angle, yaw_center_offset);moto_yaw.angle是y观测量
//	pit_relative_pos = get_relative_pos(moto_pit.angle, pit_center_offset);
	imu.wx=yaw_zgyro_angle;
	
	while(1)
	{
		AUTO_buff_shot();
		manifold_uart_init(); 
		//keep imu in a constant temperature
		imu_tmp = 21 + mpu_data.temp/333.87f;
		pid_calc(&pid_imu_tmp, imu_tmp, imu_tmp_ref);		//keep in 40 degree
		TIM3->CCR2 = pid_imu_tmp.pos_out;
		/* get angular speed, for gimbal speed loop */
		//mpu_get_data();
		mpu_get_data();
		CalibrateHook(CALI_GIMBAL);
	//cameraCaliHook(cv_big_buff.y5, cv_big_buff.p5);
		pit_pos_rc_int += -rc.ch4 *0.05f* GIMBAL_RC_MOVE_RATIO_PIT;  
		pit_pos_mouse_int += rc.mouse.y*0.5f* GIMBAL_PC_MOVE_RATIO_PIT;//p轴俯仰变化、0.3
		VAL_LIMIT(pit_pos_rc_int, -PIT_MOVE_UP, PIT_MOVE_DOWN);    //need change
		VAL_LIMIT(pit_pos_mouse_int, -PIT_MOVE_UP, PIT_MOVE_DOWN);
		if (flag_pit>0){
		pit_pos_ref = pit_pos_rc_int + pit_pos_mouse_int;
		}
		VAL_LIMIT(pit_pos_ref, -PIT_MOVE_UP, PIT_MOVE_DOWN);
		
		  gYaw.last_mode = gYaw.ctrl_mode;					
			//gYaw.zgyro_angle = yaw_zgyro_angle + gYaw.zgyro_offset;
//		if((gYaw.last_zgyro_target-gYaw.zgyro_target)==0)imu.wx=imu.wx;
//		if((gYaw.last_zgyro_target-gYaw.zgyro_target)!=0)imu.wx=yaw_zgyro_angle;
		if(abs(imu.wz)<1)imu.wx=imu.wx;//+= imu.wz * 0.05f;;
		if(abs(imu.wz)>=1)imu.wx=yaw_zgyro_angle;		
     gYaw.zgyro_angle = imu.wx + gYaw.zgyro_offset;
			//9: TODO: limit yaw gyro mode target.2016-12-14 21:23:30 DONE！
			if (  (yaw_relative_pos>700 && rc.ch3>0)
				|| (yaw_relative_pos<-700 && rc.ch3<0) 
			  ||  (yaw_relative_pos>700&& rc.mouse.x >0	)
			  ||  (yaw_relative_pos<-700&& rc.mouse.x <0	)
				|| fabs(yaw_relative_pos)<700	)
			
			
			//(yaw_relative_pos>700 && rc.ch3>0)|| (yaw_relative_pos<-700 && rc.ch3<0) 	|| fabs(yaw_relative_pos)<600	
			{	//limit position, means angle between gimbal and chassis is too large!!!  bug
				
								//yaw轴限位
			if(flag_yaw>0)
				{
			gYaw.zgyro_rc += (-rc.ch3*0.001f)*GIMBAL_RC_MOVE_RATIO_YAW;//0.001f
			gYaw.zgyro_mouse += -rc.mouse.x * 0.01f*GIMBAL_PC_MOVE_RATIO_YAW;//0.01f
					
			
					}
			
			}
				if(flag_yaw>0)
					{
		gYaw.zgyro_target = gYaw.zgyro_rc + gYaw.zgyro_mouse;
				}
					
	  	/*	if (gYaw.zgyro_target>40.0)
			{
      gYaw.zgyro_rc=40.0;     			
		
					}
			
				if (gYaw.zgyro_target<-34.0)
			{
           gYaw.zgyro_rc=-34.0; 			
			
					}	*/
		//	 VAL_LIMIT(gYaw.zgyro_target, -YAW_MOVE_RANGE, YAW_MOVE_RANGE);
		switch(rc.sw2){
			case (RC_UP):
//				if(gYaw.last_mode == GIMBAL_RELAX){	//开机后不允许直接处于陀螺仪模式
//					gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;	
//				}
		//		if(HAL_GetTick() > 4000 ){
					gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
					
		//}	
				break;
			case (RC_MI):
				gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
				break;
			case (RC_DN):
				gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
//				gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ENCODER;//hide encoder mode!!!
				//gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ENCODER;//no matter how gimbal work, chassis follow gimbal
				break;
			default : break;
		}


if(cv_mainfold_yandp.flag==0xA0){	//cv track big buff
		  
	 cv_mainfold_yandp.yawnow= gYaw.zgyro_target;
   cv_mainfold_yandp.pitnow= pit_pos_ref;
	

	
	
			}
					
	autoshot_big_buff();
			
			

		//fuck yuanlai s sw1 poke 
		/*shoot poke moto task*/
		if ((fric_wheel_run && (autoshot_cmd || KeyStateMachine(&mouse_left_sta, rc.mouse.l) ) )||(fric_wheel_run && autoshot_cmd  )){
			poke_pos_ref += 135863*poke_dir;	//60'理论值=136530 实际测100圈 get 135863 stable as dog    1.2   
			autoshot_cmd = 0;
		}
		if((rc.sw2 == RC_UP && fric_wheel_run && rc.sw1 == RC_DN)||(rc.sw2 == RC_DN && fric_wheel_run && rc.sw1 == RC_DN))
			poke_pos_ref += POKE_MOTOR_SPEED*poke_dir;
		
		/*add poke protect*/
		if(abs(pid_poke.pos_out) > 4000)
			poke_stuck_cnt ++ ;
		else
			poke_stuck_cnt = 0;
		
		if(poke_stuck_cnt > 30)	//1.5s
		{
			poke_stuck_cnt=0;
			moto_poke.round_cnt = 0;
			poke_pos_ref = 0;
			poke_dir = -poke_dir;
		}
			
		
		if(KeyStateMachine(&mouse_right_sta, rc.mouse.r) || last_sw1 == RC_MI && rc.sw1 == RC_UP &&1){ //rc.sw2 == RC_UP)||rc.sw2 == RC_DN{
			fric_wheel_run = !fric_wheel_run;//摩擦轮
		}
		last_sw1 = rc.sw1;
		if ((fric_wheel_run && rc.sw2 == RC_UP && !gRxErr.err_list[DbusTOE].err_exist)||(fric_wheel_run && rc.sw2 == RC_DN && !gRxErr.err_list[DbusTOE].err_exist)){
			TIM12->CCR1 = TIM12->CCR2 = SHOT_FRIC_WHEEL_SPEED;	//if fric run faster, gyro will have too much noise
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
			//open laser  开启激光
		}
		else{
			TIM12->CCR1 = TIM12->CCR2 = 1000;
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
			//close laser
		}
		
		//transform  absolute ecd range [0,8192] to relative range [-MOVE_RANGE, MOVE_RANGE],
		yaw_relative_pos = get_relative_pos(moto_yaw.angle, yaw_center_offset);//修改800，此值为云台在中间位置时yaw电机编码器反馈值
		pit_relative_pos = get_relative_pos(moto_pit.angle, 6460);//5600,3号车是750
		yaw_speed=moto_pit.angle-yaw_angle1;
		
	//eliminate the illustration
		if(abs(yaw_relative_pos)<=5)
		{
			yaw_relative_pos=0;
		}
	if(cv_mainfold_yandp.mod_flag_yaw==0&&abs(imu.wz)>1)
		{
	  PID_struct_init(&pid_yaw, POSITION_PID, 4000, 400,
									180,	0,	0	);  //1.8f,	0.032f,	65	//230   //180 0 0
  	PID_struct_init(&pid_yaw_omg, POSITION_PID, 4000, 400,
									15,	0,0);  
		pid_calc(&pid_yaw, gYaw.zgyro_angle, gYaw.zgyro_target); 
		pid_calc(&pid_yaw_omg, mpu_data.gz/10.0f,-pid_yaw.pos_out/10.0f); //我想除以10会有一种误差比较小的感觉
		}
			if(cv_mainfold_yandp.mod_flag_yaw==0&&abs(imu.wz)<=1)
		{
	  PID_struct_init(&pid_yaw, POSITION_PID, 4000, 400,
									180,	0,	300	);  //1.8f,	0.032f,	65	//230   //180 0 0
  	PID_struct_init(&pid_yaw_omg, POSITION_PID, 4000, 400,
									15,	-5,0);  
		pid_calc(&pid_yaw, gYaw.zgyro_angle, gYaw.zgyro_target); 
		pid_calc(&pid_yaw_omg, mpu_data.gz1/10.0f,-pid_yaw.pos_out/10.0f); //我想除以10会有一种误差比较小的感觉
		}
   if( cv_mainfold_yandp.mod_flag_yaw==1)
    {
    pid_calc(&pid_yaw, yaw_relative_pos/10.0f, gYaw.zgyro_target); 
		pid_calc(&pid_yaw_omg, mpu_data.gz/10.0f,-pid_yaw.pos_out/10.0f); //我想除以10会有一种误差比较小的感觉
		PID_struct_init(&pid_yaw, POSITION_PID, 4000, 400,
									60,	0,	0	);  //1.8f,	0.032f,	65	//230   //180 0 0
	 PID_struct_init(&pid_yaw_omg, POSITION_PID, 4000, 400,
									25,	0,0);  

    }   



		pid_calc(&pid_pit, pit_relative_pos, pit_pos_ref); 
  	pid_calc(&pid_pit_omg, mpu_data.gx/10.0f, pid_pit.pos_out/10.0f); //我想除以10会有一种误差比较小的感觉
		
		pid_calc(&pid_poke, moto_poke.total_angle/100, poke_pos_ref/100);
		pid_calc(&pid_poke_omg, moto_poke.speed_rpm, pid_poke.pos_out); 
		
		//final output, for safe protect purpose
		if ((rc.sw2 == RC_UP && gAppParam.GimbalCaliData.isAlreadyCalied == 0x55
			&& gYaw.ctrl_mode != GIMBAL_RELAX
			&& !gRxErr.err_list[DbusTOE].err_exist 	
			&& !gRxErr.err_list[GimbalYawTOE].err_exist
			&& !gRxErr.err_list[GimbalPitTOE].err_exist)||(rc.sw2 == RC_DN && gAppParam.GimbalCaliData.isAlreadyCalied == 0x55
			&& gYaw.ctrl_mode != GIMBAL_RELAX
			&& !gRxErr.err_list[DbusTOE].err_exist 	
			&& !gRxErr.err_list[GimbalYawTOE].err_exist
			&& !gRxErr.err_list[GimbalPitTOE].err_exist))
		{

			
			
			
			can_send_gimbal_iq(pid_yaw_omg.pos_out, -pid_pit_omg.pos_out, pid_poke.pos_out);  //pid_yaw_omg.pos_out, -pid_pit_omg.pos_out, pid_poke.pos_out
		}
		else{ 
			
			
			pid_poke.iout = 0;
			can_send_gimbal_iq(0, 0, 0);	//relax state
		}
		/* just for debug function*/
		//extern void AHRS_GetAttitude(void);
		//AHRS_GetAttitude();
		uart_send2pc(&BT_HUART, Monitor1, ZGyroModuleAngle);
		/* just for debug function*/
		/* for jscope watch */
		watch_param[0] = pid_yaw_omg.get[0];
		watch_param[1] = pid_yaw_omg.set[0];
		watch_param[2] = pid_yaw.get[0];
		watch_param[3] = pid_yaw.set[0];
		
		watch_pit[0] = pid_pit_omg.get[0];
		watch_pit[1] = pid_pit_omg.set[0];
		watch_pit[2] = pid_pit.get[0];
		watch_pit[3] = pid_pit.set[0];
		/* for jscope watch */
    gYaw.last_zgyro_target=gYaw.zgyro_target;
		yaw_angle1=moto_pit.angle;
		osDelay(5);
		
	}
}


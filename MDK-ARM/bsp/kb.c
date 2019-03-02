#include "kb.h"
#include "mytype.h"
#include "cmsis_os.h"
#include "bt.h"
#include "chassis.h"
extern CV_mainfold_yandp cv_mainfold_yandp;
 u8  mainflod_use_date[2];


/************************* 全局变量 *******************************/
SPEED_USE speed_max_use ;
km_control_t  km;
u16 key_code=0;
vu8 kb_move_mode = 0;
u16 max_kb_move_speed = 1200;//200
int anti_on=0;
u8 rc_R_sta;
int key_use_flag = 0 ;

/************************* 全局变量 *******************************/


enum{
    KEY_IDLE = 0,					//没有按键按下的空闲状态
    KEY_DOWN_WAIT_CONFIRM,		//表示可能处于抖动状态，等待第二次扫描确认
    KEY_DOWN_WAIT_RELEASE,		//按键确认按下，等待释放进入空闲
}KeyStateTypeDef;

//used for avoid mouse  press shake 
u8 KeyStateMachine(u8* psta, u8 condition){
    u8 ret=0;	//temp var must init! or it is a random 
    switch(*psta){
        case KEY_IDLE:
            if(condition)
                *psta = KEY_DOWN_WAIT_CONFIRM;
            break;
            
        case KEY_DOWN_WAIT_CONFIRM:
            if(condition){
                *psta = KEY_DOWN_WAIT_RELEASE;
                ret = 1;	//这句放在这里的话 就是按下立即触发。如果放下面就是释放才触发
            }
            else
                *psta = KEY_IDLE;
            break;
            
        case KEY_DOWN_WAIT_RELEASE:
            if(!condition)
                *psta = KEY_IDLE;
            break;
    }
    return ret;
}



void set_manifold_vision_mode(u8 cmd){

		
		HAL_UART_Transmit(&huart6, mainflod_use_date, 2, 10);
	
}


/**
*@public:  in the future, delete kb.c this file/task, combine to chassis 
**/
void key_mouse_task(void const * argument)
{
	TIM2->CCR1=2200;
  /* USER CODE BEGIN KeyDealTask */
  osDelay(1000);//等待稳定，不允许操作(主要是要等待陀螺仪二次复位稳定)
  /* Infinite loop */
  for(;;)
  {
		 
    key_code = rc.kb.key_code;
    //if(rc.kb.bit.R)
     // chassis.is_snipe_mode = 1;
    //else
      //chassis.is_snipe_mode = 0;
    
    /*根据shitf或者ctrl键的情况来选择底盘前后左右加速减速模式*/
//    if(key_code & SHIFT)
//      kb_move_mode = ShiftQuicklyMode;
//    else if(key_code & CTRL)
//      kb_move_mode = CtrlSlowlyMode;
//    else
//      kb_move_mode = NormalMode;
				if(rc.kb.bit.W)  
{
   km.vy += 6;
	 speed_max_use.max_wheel_speed=3600;

if (rc.kb.bit.SHIFT)
	{
     if(km.vy==0)
{
			 osDelay(40);
		 
		 }
	 speed_max_use.max_wheel_speed=4500; 
	 km.vy+=4; 
	}
	
}
  else if(rc.kb.bit.S)
	{
     km.vy += -6;
		 speed_max_use.max_wheel_speed=3600;
	 if(rc.kb.bit.SHIFT)
	 {
		  if(km.vy==0)
    {
			 osDelay(40);
		 
		 }
	 speed_max_use.max_wheel_speed=4500; 
	 km.vy -=4; 
	 }
	}
	
	else
	{
      km.vy = 0;
      speed_max_use.max_wheel_speed=3600;
	}
		
	 if(rc.kb.bit.A)
      km.vx += -5;
    else if(rc.kb.bit.D)
      km.vx += 5;
    else
      km.vx = 0;
	
	
		  if(km.vx > max_kb_move_speed)
      km.vx = max_kb_move_speed;
    if(km.vx < -max_kb_move_speed)
      km.vx = -max_kb_move_speed;
    
    if(km.vy > max_kb_move_speed)
      km.vy = max_kb_move_speed;
    if(km.vy < -max_kb_move_speed)
      km.vy = -max_kb_move_speed;
		
		if (rc.kb.bit.Z&&rc.sw2 == RC_DN)
		{
			osDelay(70);
			if(rc.kb.bit.Z&&rc.sw2 == RC_DN)
			{
		key_use_flag=!key_use_flag;
			
    	if(key_use_flag)
   	{
	   cv_mainfold_yandp.mod_flag_yaw=1;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
	
	   }
   else 
    {
    cv_mainfold_yandp.mod_flag_yaw=0;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
   }
		
		}
			osDelay(100);
	}
		
		
		if(KeyStateMachine(&rc_R_sta, rc.kb.bit.R))
			
			anti_on=!anti_on;
						
		if(anti_on)
		{
			chassis.anti_attack=1;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
		}
		else
		{
			chassis.anti_attack=0;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		}
			if (rc.kb.bit.Q&&rc.kb.bit.CTRL)
{
  osDelay(70) ;
	if(rc.kb.bit.Q&&rc.kb.bit.CTRL){
		mainflod_use_date[0]=0xA1;
		mainflod_use_date[1]=~mainflod_use_date[0];		
    set_manifold_vision_mode(0XA1);
	}
 osDelay(100);
}
if (rc.kb.bit.E&&rc.kb.bit.CTRL)
{
osDelay(70) ;
	if( rc.kb.bit.E&&rc.kb.bit.CTRL){
		mainflod_use_date[0]=0xA2;
		mainflod_use_date[1]=~mainflod_use_date[0];
  set_manifold_vision_mode(0XA2);
	}
	osDelay(100);
}

if (rc.kb.bit.C&&rc.kb.bit.CTRL)
{
  osDelay(70) ;
	if (rc.kb.bit.C&&rc.kb.bit.CTRL)
	{
		mainflod_use_date[0]=0xA3;
		mainflod_use_date[1]=~mainflod_use_date[0];
 set_manifold_vision_mode(0XA3);
	}
	
	osDelay(100);
}

    if(rc.kb.bit.G) 
		{			
      TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=3000;
		  TIM2->CCR1=700;
		}
	 if(rc.kb.bit.F)   //舵机
	 {
     TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=8000;
	   TIM2->CCR1=2200;
	 }

    osDelay(10);
  }
  /* USER CODE END KeyDealTask */
}

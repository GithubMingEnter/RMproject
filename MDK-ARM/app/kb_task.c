




#include "dogconfig.h"

#define MAX_KB_SLOWLY_SPEED     150
#define MAX_KB_NORMAL_SPEED     350
#define MAX_KB_QUICKLY_SPEED    450

//#define MAX_

enum{
  NormalMode = 0, 
  ShiftQuicklyMode,
  CtrlSlowlyMode,
}KB_MoveMode;

/************************* 全局变量 *******************************/
//extern RC_Type rc; 
kb_control_t  kb;
int hello, look, fuck;
u16 key_code=0;
u8 openKeyenceDetect = 0;
vu8 kb_move_mode = 0;
u16 max_kb_move_speed;
u16 open_bullet_door_cnt=0;
/************************* 全局变量 *******************************/

/**
*@public:
**/
void KeyDealTask_Func(void const * argument)
{
  /* USER CODE BEGIN KeyDealTask */
	//RemoteControl_Param_Init();
	//Cloud_PID_Param2Hand_Mode();//上电手动模式
	osDelay(1000);//等待稳定，不允许操作(主要是要等待陀螺仪二次复位稳定)
  /* Infinite loop */
  for(;;)
  {
    key_code = rc.keyBoard.key_code;
    if(key_code & ClickE)
      myUnderpan.is_snipe_mode = 1;
    else
      myUnderpan.is_snipe_mode = 0;
    
    /*根据shitf或者ctrl键的情况来选择底盘前后左右加速减速模式*/
    if(key_code & ClickShift)
      kb_move_mode = ShiftQuicklyMode;
    else if(key_code & ClickCtrl)
      kb_move_mode = CtrlSlowlyMode;
    else
      kb_move_mode = NormalMode;
    /*上岛之后速度减慢*/
    if(   eLandingState == SeatOnIslandFirst
        || eLandingState == StandOnIslandToCatchEggs
        || eLandingState == CatchEggsDoneSeatOnIsland)
      kb_move_mode = CtrlSlowlyMode;
    /*根据速度模式选择按键积分的最大速度*/
    switch(kb_move_mode){
        case NormalMode:
          max_kb_move_speed = MAX_KB_NORMAL_SPEED;break;
        case ShiftQuicklyMode:
          max_kb_move_speed = MAX_KB_QUICKLY_SPEED;break;
        case CtrlSlowlyMode:
          max_kb_move_speed = MAX_KB_SLOWLY_SPEED;break;
        default:
          max_kb_move_speed = MAX_KB_NORMAL_SPEED;break;
    }
    
    
    
    if(key_code & ClickW)   //慢慢加速到400
      kb.speed_x += 2;
    else if(key_code & ClickS)
      kb.speed_x += -2;
    else
      kb.speed_x = 0;
    
    if(key_code & ClickA)
      kb.speed_y += -2;
    else if(key_code & ClickD)
      kb.speed_y += 2;
    else
      kb.speed_y = 0;
    
    
    
    
    if(kb.speed_x > max_kb_move_speed)
      kb.speed_x = max_kb_move_speed;
    if(kb.speed_x < -max_kb_move_speed)
      kb.speed_x = -max_kb_move_speed;
    
    if(kb.speed_y > max_kb_move_speed)
      kb.speed_y = max_kb_move_speed;
    if(kb.speed_y < -max_kb_move_speed)
      kb.speed_y = -max_kb_move_speed;
    
		switch(key_code)
		{
      //switch-case 相当于if else if 结构，导致wsad不能形成组合键！应该使用if同步判断
//			case(ClickW):// 这相当于四个要一起按。我日

			case(ClickE):	//enter snipe mode 
//myUnderpan.is_snipe_mode = 1;
				break;
			case(ClickR):
        PULL_CATCH_EGG();
				break;
			case(ClickR|ClickCtrl):
        PUSH_CATCH_EGG();
				break;
      
      case(ClickF):
        openKeyenceDetect = 1;
        PUSH_CATCH_EGG();
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigStart);
				break;
      case (ClickF | ClickShift):
        AIR_FRONT_LONG;
        break;
      case (ClickF | ClickCtrl):
        AIR_FRONT_SHORT;
        break;
      case (ClickB | ClickShift):
        AIR_BACK_LONG;
        break;
      case (ClickB | ClickCtrl):
        AIR_BACK_SHORT;
        break;
      
			case(ClickZ):
				
					//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoUp);
				break;
			case(ClickX):
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoOut);
				break;
			/*点击C 持续50ms 相当于发五次 伸出取弹机构的命令，伸出去*/
			case(ClickC):
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoDown);
				break;  
			case(ClickV):
        
				break;
			case(ClickB):	//just like break-
        open_bullet_door_cnt = 200;
        osDelay(1000);
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigStop);
				break;
			case(ClickShift):
				break;
			case(ClickCtrl):
				break;
			
			/*combine keys */
			case(ClickZ|ClickCtrl):
				/*持续按住Ctrl，将会让抓蛋机构持续往下动，直到合适的时候放开*/
				//CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*) &DigGoDown);
				break;
			
			case(ClickQ|ClickShift):
        kb.angle -= 0.5f;  //1s = 20'
				break;
			case(ClickE|ClickShift):
        kb.angle += 0.5f;  //1s = 20'
				break;
			case(ClickShift|ClickCtrl):
				break;
			
			default :
				break;
		}
		
    if(open_bullet_door_cnt){
      OPEN_BULLET_DOOR();
      open_bullet_door_cnt--;
    }
    else
      CLOSE_BULLET_DOOR();
    
		osDelay(10);
  }
  /* USER CODE END KeyDealTask */
}




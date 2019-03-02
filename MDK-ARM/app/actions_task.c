
#include "dogconfig.h"
#include "actions_task.h"
#include "bsp_can.h"
#include "bt.h"
#include "cmsis_os.h"



#define READ_AIR_BACK			  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)
#define READ_AIR_FRONT			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)
#define READ_AIR_CENTER			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)



#define IN_RANGE				0
#define NOT_IN_RANGE		1				//default state = pull up = 1

/*############## --- Debug Macros ---- #############*/
#define STAND_FOR_CATCH_EGG
//#define STN_SIT_DEBUG_MODE  //只是测试，右遥控拨上就站起来，中间就降下来
//#undef  GO_UPSTAIRS //上台阶模式
/*############## --- Debug Macros ---- #############*/

int16_t zero=0;
char rcKeyRight,rcKeyLeft;
char eLandingState = 0;
extern u8 openKeyenceDetect;  //只有按下按键V把这个打开后  才检测基恩士 防止看到两个桥柱！

LandingStatusUnionType	landing_total_status;
char* state_str[TotalStateLength]={
  "Sit\\Gnd",	//seat on ground
  "StndGnd",	//ready go island
  "Land50%",	//landing half complete
  "SitIlan",	//seat on island 
  "Stn&Cat",	//stand island for catch eggs
  "CtOkSit",	//catch done and seat on ground
	"Levi50%",	//leaving gnd
	"FSitGnd",	//finally sit gnd
};

void AirSwitch_init(void){
  AIR_FRONT_SHORT;
	AIR_BACK_SHORT;
	AIR_CENTER_IN;
  AIR_SIDE_SHORT;
  PULL_CATCH_EGG();
  CLOSE_BULLET_DOOR();
}


void StartActionTask(void const * argu){
	
	AIR_FRONT_SHORT;
	AIR_BACK_SHORT;
	AIR_CENTER_IN;
	AIR_SIDE_SHORT;
	while(1){
		
		rcKeyRight = rc.switch_right;
		rcKeyLeft = rc.switch_left;
		
		landing_total_status.bit.key_bl = KEY_BL;
		landing_total_status.bit.key_br = KEY_BR;
		landing_total_status.bit.key_ml = KEY_ML;
		landing_total_status.bit.key_mr = KEY_MR;
		landing_total_status.bit.key_fl = KEY_FL;
		landing_total_status.bit.key_fr = KEY_FR;
		landing_total_status.bit.landing_status = eLandingState;
		landing_total_status.bit.air_front = READ_AIR_FRONT;
		landing_total_status.bit.air_back = READ_AIR_BACK;
		landing_total_status.bit.air_center = READ_AIR_CENTER;
		
		uart_send2pc(&BT_HUART, LandingStatus, landing_total_status.ftotal);

		if(RC_DN == rcKeyLeft)
			osDelay(100);
		
		//right键初始化是下面
		if(SeatOnTheGround == eLandingState &&  RC_MI == rcKeyRight){
			AIR_FRONT_LONG;
			AIR_BACK_LONG;
      AIR_SIDE_LONG;
			AIR_CENTER_OUT;	//伸出来
			//四脚升起来
			osDelay(1000);
			eLandingState = StandOnGNDReadyLanding;
		}
		//然后开向台阶。进入自动模式 等待后面两个开关进入距离
		if(StandOnGNDReadyLanding == eLandingState  && RC_DN == rcKeyRight){
      AIR_SIDE_SHORT;
      osDelay(200);
			//在站立准备上岛时如果播下开关则立即降下，且切换到seat
			AIR_FRONT_SHORT;	
			AIR_BACK_SHORT;
      
			AIR_CENTER_IN;
			eLandingState = SeatOnTheGround;
		}
    
#if !defined STN_SIT_DEBUG_MODE
		//if(1 == eLandingState && RC_UP == rcKeyRight){
		if(StandOnGNDReadyLanding == eLandingState 
			&& openKeyenceDetect 				//按V键后才可以登录资源岛，否则不可！
			&& IN_RANGE == KEY_BL && IN_RANGE == KEY_BR)
		{
//			osDelay(500);	//等待完全弹出再收后两个轮子
			AIR_BACK_SHORT;
      PUSH_CATCH_EGG();
			eLandingState = LandingHalfComplete;	//完成一半了
		}
		//if(2 == eLandingState && RC_MI == rcKeyRight){
		if(LandingHalfComplete == eLandingState 
			&& IN_RANGE == KEY_ML && IN_RANGE == KEY_MR
			&& IN_RANGE == KEY_FL && IN_RANGE == KEY_FR)
		{
			AIR_FRONT_SHORT; 
			AIR_CENTER_IN;
      
			eLandingState = SeatOnIslandFirst;
      osDelay(3000);//为了不至于刚上岛的时候，
      //因为摇晃导致的车以为要下岛导致前两个要收缩又伸出，
      //所以加延时，3s的时间内不判断状态跳转
			//不出意外的话现在车已经上了资源岛
		}
    
    /*#################这部分后面要改成自动#################*/
		if(SeatOnIslandFirst == eLandingState && RC_UP == rcKeyRight){
			AIR_FRONT_LONG;
			AIR_BACK_LONG;
      
			eLandingState = StandOnIslandToCatchEggs;
		}
		
		if(StandOnIslandToCatchEggs == eLandingState && RC_MI == rcKeyRight){
			AIR_FRONT_SHORT;
			AIR_BACK_SHORT; 
      DOT_LASER_OFF();
      LINE_LASER_ON();
			eLandingState = CatchEggsDoneSeatOnIsland;

      osDelay(1000);  //1s内不检测要等降下后，传感器在地面上 才检测要下岛的部分传感器
		}
    
    //下次要注意，就是Short或者LONG都不能瞬时就做完动作，
    //这时候要考虑传感器立马会响应！！导致的状态机错误
		/*#################这部分后面要改成自动#################*/
//		if(SeatOnIsland == eLandingState
//				&& IN_RANGE == KEY_BL && IN_RANGE == KEY_BR )
//		{
//			AIR_FRONT_LONG;
//			AIR_BACK_LONG;
//			eLandingState = StandOnIslandToCatchEggs;
//		}
//		
//		if(StandOnIslandToCatchEggs == eLandingState
//				&& IN_RANGE == KEY_BL && IN_RANGE == KEY_BR )
//		{
//			AIR_FRONT_SHORT;
//			AIR_BACK_SHORT;
//			eLandingState = SeatOnIsland;
//		}
		
		//准备下台阶，在台上的时候，两个开关应该在距离之内 = 0
//		if(3 == eLandingState && RC_DN == rcKeyRight){
		//########注意 这里可能有安全隐患，万一照到柱子上？！
		if((CatchEggsDoneSeatOnIsland == eLandingState)
			&& NOT_IN_RANGE == KEY_FL 
			&& NOT_IN_RANGE == KEY_FR)
		{
			AIR_FRONT_LONG;
			AIR_CENTER_OUT;	//伸出 为了安全放下
      
			eLandingState = LeavingIslandHalf;
		}
//		if(4 == eLandingState && RC_MI == rcKeyRight){
		if(LeavingIslandHalf == eLandingState 
			&& NOT_IN_RANGE == KEY_FL && NOT_IN_RANGE == KEY_FR
			&& NOT_IN_RANGE == KEY_ML && NOT_IN_RANGE == KEY_MR)
		{
			AIR_BACK_LONG;
      //这样下岛之后不需要重新伸缩一次，节约气也防止车磨损
			//eLandingState = StandOnTheGround;
      eLandingState = FinalStandOnTheGround; //
      osDelay(3000);  //3s后再检测状态切换，先保证后轮两个放下来，BL和BR不会照到！
      //注意！因为这里将要landing， 却发现BL和BR在距离之内！！因为后轮两个是缩着的！！
      //导致状态一下子变成landing half！
		}
		if(FinalStandOnTheGround == eLandingState && RC_DN == rcKeyRight){
			//AIR_CENTER_IN;
      AIR_SIDE_SHORT;
      osDelay(200); //下降的时候两边的支撑必须先收！！！
			AIR_FRONT_SHORT;
			AIR_BACK_SHORT;
      PULL_CATCH_EGG();
			AIR_CENTER_IN;//收回
      DOT_LASER_ON();
      LINE_LASER_OFF();
			eLandingState = SeatOnTheGround;	//back to reset state
			//复位
		}
#endif		
		osDelay(10);

	}
}


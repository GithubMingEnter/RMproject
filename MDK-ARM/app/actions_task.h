#ifndef __ACTION_TASK
#define __ACTION_TASK

#include "mytype.h"
#include "gpio.h"
#include "bsp_can.h"


//distance key front  middle back
#define KEY_FL		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)	//front left
#define KEY_FR		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)
#define KEY_ML		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)
#define KEY_MR		HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)
#define KEY_BL		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)
#define KEY_BR		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)

//注意！换了三极管驱动电磁阀之后， short状态是SET！！！
#define	AIR_BACK_LONG			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define	AIR_BACK_SHORT		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define	AIR_FRONT_LONG		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define	AIR_FRONT_SHORT		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define	AIR_CENTER_OUT		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define	AIR_CENTER_IN			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
//两侧用于上岛的支撑
#define	AIR_SIDE_LONG		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define	AIR_SIDE_SHORT		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)


#define TRIGERED				GPIO_PIN_RESET	//触发时是0
#define UNTRIGERED			GPIO_PIN_SET		//默认情况就是上拉高电平 1

#define ReleaseBulletServo			(TIM3->CCR2)
#define StepMotoPWM							(TIM3->CCR1)

//typedef __packed union{
//	u8 keyence;
//	
//	struct{
//		u8 fl :1;	//
//		u8 fr :1;
//		u8 ml :1;
//		u8 mr :1;
//		u8 bl :1;
//		u8 br :1;
//	
//		u8 rsvd:2;
//	}bit;
//}KeyenceSensorUnionType;

//需要新加状态，登陆小岛后要再次站立，后面可以改成基恩士接近后自动站立
enum{
	SeatOnTheGround=0,
	StandOnGNDReadyLanding, 				//准备登陆资源岛。
	LandingHalfComplete,				//登陆一半
	SeatOnIslandFirst, 					//第一次1@登陆成功，坐在岛上 
	
	StandOnIslandToCatchEggs,		//new added
	CatchEggsDoneSeatOnIsland,
	
	LeavingIslandHalf,					//前脚下地，离开完成一半
	FinalStandOnTheGround, 					//离开小岛完成,站在地上
		
  TotalStateLength,
	
};

typedef __packed union{
		u32 	total;
		float ftotal;
		__packed struct __bit {
		//KeyenceSensorUnionType keyence;
		u8 key_fl :1;	//keyence
		u8 key_fr :1;
		u8 key_ml :1;
		u8 key_mr :1;
		u8 key_bl :1;
		u8 key_br :1;
		u8 key_rsvd	:2;
		
		u8 air_front	:1;
		u8 air_back		:1;
		u8 air_center	:1;
		u8 air_rsvd:5;
		
		u8 rsvd;
		
		u8 landing_status;
		
	}bit ;
}LandingStatusUnionType;



void AirSwitch_init(void);
void StartActionTask(void const * argu);


extern LandingStatusUnionType landing_total_status;
extern MotoControlTypeDef motoLiftBridge;
extern MotoControlTypeDef motoLiftProvideBullet;
extern char eLandingState;
extern char* state_str[];
#endif


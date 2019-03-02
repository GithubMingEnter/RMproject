
#ifndef _RC_KB_
#define _RC_KB_

#include "stm32f4xx_hal.h"
#include "dogconfig.h"


/*
	测试下模式切换说明：
	遥控器两个开关为主模式，优先级跟高
	ctrl+ Q W A S 经典 云台分离1 云台分离2 云台分离3
	shift+ Q W A S 全手动 云台自动 底盘自动 全自动
	切换模式时，不能按其他键
*/

/********************** 用户手感设置 ************************/
#define User_A_K 400
#define User_D_K 400
#define User_W_K 400
#define User_S_K 400
#define User_Q_K 400
#define User_E_K 400

#define User_MouseX_K 400.0f
#define User_MouseY_K 400.0f
#define User_MouseZ_K 400.0f

#define User_L_Ver 900
#define User_L_Lev 900
#define User_R_Ver 900
#define User_R_Lev 900
/********************** 用户手感设置 ************************/


/************************ 按键地址 *************************/

/**********************************************************************************
   * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/
#define ClickW 			0x0001		//bit 0
#define ClickS 			0x0002
#define ClickA 			0x0004
#define ClickD 			0x0008
#define ClickShift 	0x0010
#define ClickCtrl 	0x0020
#define ClickQ 			0x0040
#define ClickE			0x0080
#define ClickR 			0x0100
#define ClickF 			0x0200
#define ClickG 			0x0400
#define ClickZ 			0x0800
#define ClickX 			0x1000
#define ClickC 			0x2000
#define ClickV 			0x4000		//bit 15
#define ClickB			0x8000
/************************ 按键地址 *************************/

typedef struct{
  float speed_x;
  float speed_y;
  float angle;
  int acc;
}kb_control_t;

extern kb_control_t kb;

void KeyDealTask_Func(const void*);

#endif



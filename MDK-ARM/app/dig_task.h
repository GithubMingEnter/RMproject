// Header:	
// File Name: dig task
// Author:	langgo
// Date: 2016Äê6ÔÂ3ÈÕ11:28:45


#ifndef __DIG_TASK_
#define __DIG_TASK_

#include "mytype.h"



//task function
void DigTask(const void *argu);

extern u8 DigFeedBackStatus;

extern const s16 DigGoOut;
extern const s16 DigGoUp;
extern const s16 DigGoDown;
extern const s16 DigStop;
extern const s16 DigStart;

#define DOT_LASER_ON()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)
#define DOT_LASER_OFF()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)
#define DOT_LASER_TOGGLE()	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6)
#define LINE_LASER_TOGGLE()	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8)
#define LINE_LASER_ON()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define LINE_LASER_OFF()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
#define PUSH_CATCH_EGG()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define PULL_CATCH_EGG()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
#define OPEN_GOLF_DOOR()	TIM12->CCR1 = 2300
#define CLOSE_GOLF_DOOR()	TIM12->CCR1 = 1300
#define OPEN_BULLET_DOOR()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define CLOSE_BULLET_DOOR()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#endif


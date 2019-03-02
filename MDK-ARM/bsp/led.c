#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "led.h"

//void ledShowError(){
//	while(1){
//		
//		#ifdef USE_FREE_RTOS
//			LED_R_ON;
//			osDelay(50);
//			LED_R_OFF;
//			osDelay(50);
//		#else
//			LED_R_ON;
//			HAL_Delay(50);
//			LED_R_OFF;
//			HAL_Delay(50);
//		#endif
//		
//	}
//}

void led_fly_status(char is_ready){
	if(is_ready){
		LED_G_ON;
		LED_B_OFF;
		LED_R_OFF;
	}else{
		LED_R_ON;
		LED_B_OFF;
		LED_G_OFF;
	}
}

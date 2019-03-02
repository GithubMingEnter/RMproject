

#include "dogconfig.h"
#include "cmsis_os.h"
#include "dig_task.h"
#include "error_task.h"
#define ToString(x)		#x	
#define ADD(x)				"当时我就念了两句诗"##ToString(x)

/*
	dig protocal
	data[0]   0x99  AA  BB   CC    DD 		DLC = 1
*/
const s16 DigGoOut			= 0x99;
const s16 DigGoUp				= 0xAA;
const s16 DigGoDown 		= 0xBB;
const s16 DigStop				= 0xCC;
const s16 DigStart			= 0xDD;

#define tell_under_out    0x11 
#define tell_under_in     0x22
#define open_golf_door    0x33
#define close_golf_door   0x44
#define open_line_laser   0x55
#define close_line_laser  0x66
#define blink_line_laser  0x77
#define release_laser_control 0x88

#define bullet_gate   //PA1


u8 DigFeedBackStatus;    //update in bsp_can, rx callback
int DigActionStatus = 0;

short dig_cmd=0;
u8 blink_laser = 0;
u16 blink_cnt;

void dig_cmd_action(u8 cmd){
	switch(cmd){
		case tell_under_out:
			PUSH_CATCH_EGG();
			break;
		case tell_under_in:
			PULL_CATCH_EGG();
			break;
		case open_golf_door:
			OPEN_GOLF_DOOR();
			break;
		case close_golf_door:
			CLOSE_GOLF_DOOR();
			break;
		case open_line_laser:
			DOT_LASER_OFF();
			LINE_LASER_ON();
			blink_laser = 0;
			break;
		case close_line_laser:
			DOT_LASER_OFF();
			LINE_LASER_OFF();
			blink_laser = 0;	
			break;
		case blink_line_laser:
			blink_laser = 1;	
			break;
	}

}

void DigTask(const void *argu){

	LINE_LASER_ON();	
	DOT_LASER_OFF();
	CLOSE_GOLF_DOOR();
	PULL_CATCH_EGG();
	
	for(;;){
		
		
		if(blink_laser){
			if(blink_cnt++>30){
				blink_cnt = 0;
				
				LINE_LASER_TOGGLE();
			}
		}	
//		dig_cmd = DigUp;
//		if(DigActionStatus == 0){
//			if(rc.switch_left == RC_MI)
//				CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*)&DigGoOut);
//		}
//		if(DigActionStatus == 1){
//			CAN_Send_Message(&hcan1, CAN_DigBulletCmd_ID, (s16*)&DigGoDown);
//			
//		}
		
		
		//每次app变大， 就不能用！fuck！！ find problem in pcdog！！！
		//每次app变大， 就不能用！fuck！！ find problem in pcdog！！！
		//每次app变大， 就不能用！fuck！！ find problem in pcdog！！！
//		bt_printf(&BT_HUART, ADD(我感觉你们还要提高自身的姿势水平\r\n)  );
		bt_printf(ADD(苟利国家生死以@岂因祸福避趋之\r\n)  );
		DBG_PRINT("fuckyou! %d", dig_cmd);
//		bt_printf(&BT_HUART, ADD(我感觉你们还要提高自身的姿势水平\r\n)  );
//		bt_printf(&BT_HUART, ADD(苟利国家生死以@岂因祸福避趋之\r\n)  );
//		bt_printf(&BT_HUART, ADD(苟利国家生死以@岂因祸福避趋之\r\n)  );
////		
		
		osDelay(10);
	}

}


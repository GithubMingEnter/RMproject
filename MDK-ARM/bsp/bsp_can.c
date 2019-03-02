/**
  ******************************************************************************
  * @file			my_can.c
  * @author		Ginger
  * @version	V1.0.0
  * @date			2015/11/30
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "error_task.h"
#include "chassis.h"

moto_measure_t moto_pit;
moto_measure_t moto_yaw;
moto_measure_t moto_poke;	//拨单电机
moto_measure_t moto_chassis[4] = {0};//4 chassis moto
float yaw_zgyro_angle;

void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

/*******************************************************************************************
  * @Func			my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
	static CanTxMsgTypeDef		Tx1Message;
	static CanRxMsgTypeDef 		Rx1Message;
	static CanTxMsgTypeDef		Tx2Message;
	static CanRxMsgTypeDef 		Rx2Message;

	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.BankNumber = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		//err_deadloop(); //show error!
	}

	//filter config for can2 
	//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterNumber = 14;
	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
	//err_deadloop();
	}

	if(_hcan == &hcan1){
		_hcan->pTxMsg = &Tx1Message;
		_hcan->pRxMsg = &Rx1Message;
	}


	if(_hcan == &hcan2){
		_hcan->pTxMsg = &Tx2Message;
		_hcan->pRxMsg = &Rx2Message;
	}

}

/*******************************************************************************************
  * @Func			void can_filter_recv_special(CAN_HandleTypeDef* _hcan, s16 id)
  * @Brief    待测试！！！
  * @Param		只接收filtered id，其他的全屏蔽。
  * @Retval		eg： 	CAN1_FilterConfiguration(0, HOST_CONTROL_ID);
										CAN1_FilterConfiguration(1, SET_CURRENT_ID);
										CAN1_FilterConfiguration(2, SET_VOLTAGE_ID);
										CAN1_FilterConfiguration(3, ESC_CAN_DEVICE_ID);
										CAN1_FilterConfiguration(4, SET_POWER_ID);
										CAN1_FilterConfiguration(5, SET_LIMIT_RECOVER_ID);
  * @Date     2016年11月11日
 *******************************************************************************************/
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id)
{
	CAN_FilterConfTypeDef   cf;
	cf.FilterNumber = filter_number;	//过滤器组编号
	cf.FilterMode = CAN_FILTERMODE_IDMASK;	//id屏蔽模式
	cf.FilterScale = CAN_FILTERSCALE_32BIT;	//32bit 滤波
	cf.FilterIdHigh = (filtered_id<<21) >> 16;	//high 16 bit		其实这两个结构体成员变量是16位宽
	cf.FilterIdLow = filtered_id<<21;	//low 16bit
	cf.FilterMaskIdHigh = 0xFFFF;
	cf.FilterMaskIdLow = 0xFFF8;	//IDE[2], RTR[1] TXRQ[0] 低三位不考虑。
	cf.FilterFIFOAssignment = CAN_FilterFIFO0;
	cf.BankNumber = 14;	//can1(0-13)和can2(14-27)分别得到一半的filter
	cf.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hcan, &cf);
} 



HAL_StatusTypeDef can_send_msg()
{ 
//	if(_hcan->Instance->ESR){
//		//can error occured, sleep can and reset!
//		_hcan->Instance->MCR |= 0x02;
//		_hcan->Instance->MCR &= ~(0x02);
//	}//这个是志伟试过的可以解决can错误  有待验证！
	return HAL_OK;
}

float ZGyroModuleAngle;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    这是一个回调函数,都不用声明
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	//ignore can1 or can2.
	switch(_hcan->pRxMsg->StdId){
		case CAN_3510Moto1_ID:
		case CAN_3510Moto2_ID:
		case CAN_3510Moto3_ID:
		case CAN_3510Moto4_ID:
			{
				static u8 i;
				i = _hcan->pRxMsg->StdId - CAN_3510Moto1_ID;
				
				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], _hcan) : get_moto_measure(&moto_chassis[i], _hcan);
				//get_moto_measure(&moto_chassis[i], _hcan);
				err_detector_callback(ChassisMoto1TOE + i);
			}break;
		case CAN_YAW_FEEDBACK_ID:
			{
				get_moto_measure(&moto_yaw, _hcan);
				err_detector_callback(GimbalYawTOE);
			}
			break;
		case CAN_PIT_FEEDBACK_ID:
			{
				get_moto_measure(&moto_pit, _hcan);
				err_detector_callback(GimbalPitTOE);
			}
			break;
		case CAN_POKE_FEEDBACK_ID:
			{
				moto_poke.msg_cnt++;
				
				moto_poke.msg_cnt <= 10? get_moto_offset(&moto_poke, _hcan) : get_moto_measure(&moto_poke, _hcan);//10

				err_detector_callback(PokeMotoTOE);
			}
			break;
			
		case CAN_ZGYRO_FEEDBACK_MSG_ID:
			{
					//chassis.angle_from_gyro
					
					yaw_zgyro_angle = 0.01f * ((s32)(_hcan->pRxMsg->Data[0]<<24) | 
																						(_hcan->pRxMsg->Data[1]<<16) | 
																						(_hcan->pRxMsg->Data[2]<<8)  | 
																						(_hcan->pRxMsg->Data[3])); 
					err_detector_callback(ChassisGyroTOE);
					
					chassis.angle_from_gyro = yaw_zgyro_angle;
			}
			break;
	}
		

	//hcan1.Instance->IER|=0x00008F02;
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;
	
	/*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
	ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])/-5;
	ptr->hall = hcan->pRxMsg->Data[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}



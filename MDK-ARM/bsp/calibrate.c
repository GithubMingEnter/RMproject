/*
*********************************************************************************************************
*                                     DJI BOARD SUPPORT PACKAGE
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/
/**
  ******************************************************************************
  * @file    calibrate.c
  * @author  langgo
  * @version V1.0
  * @date    16-12-1
  * @brief   
  *          This file provides gimbal offset calibrate + mpu_data + mag_data calibrate and save 2 flash function
  * @verbatim you should comment with english avoid diff text encoder cause ?!$@#%^@$%&$*
  ****************************************************************************
***/

#include "bsp_can.h"
#include "calibrate.h"
#include "bsp_flash.h"
AppParam_t gAppParam;

void CalibrateHook(CALI_ID_e cali_id){
	if(gAppParam.GimbalCaliData.GimbalNeedCali){
		gAppParam.GimbalCaliData.GimbalPitOffset = moto_pit.angle;
		gAppParam.GimbalCaliData.GimbalYawOffset = moto_yaw.angle;
		gAppParam.GimbalCaliData.isAlreadyCalied = CALIED_FLAG;	//55 better than 1
		gAppParam.GimbalCaliData.GimbalNeedCali = 0;
		AppParamSave2Flash();
	}
}
//input is when laser in center(num5), yaw and pit return from camera.
//point center(5) is used for aim to big buff center(aka. 5)
void cameraCaliHook(s16 camera_y5,s16 camera_p5){
	if(gAppParam.CameraCali.GimbalNeedCali){
		gAppParam.CameraCali.GimbalPitOffset = camera_p5;
		gAppParam.CameraCali.GimbalYawOffset = camera_y5;
		gAppParam.CameraCali.isAlreadyCalied = CALIED_FLAG;	//55 better than 1
		gAppParam.CameraCali.GimbalNeedCali = 0;
		AppParamSave2Flash();
	}
}

//this func have bug!!!
void imuCaliHook(CALI_ID_e cali_id, s16 raw_xyz[]){
	static int sum[3];
	static int cnt=0;	//global / static var init = 0
	//how to avoid two sensor cali together???
	if(gAppParam.ImuCaliList[cali_id].NeedCali){
		gAppParam.ImuCaliList[cali_id].isAlreadyCalied = 0;
		sum[0] += raw_xyz[0];
		sum[1] += raw_xyz[1];
		sum[2] += raw_xyz[2];
		if (++cnt >= 100){
			cnt=0;
			gAppParam.ImuCaliList[cali_id].offset[0] = sum[0]/100.0f;
			gAppParam.ImuCaliList[cali_id].offset[1] = sum[1]/100.0f;
			gAppParam.ImuCaliList[cali_id].offset[2] = sum[2]/100.0f;
			sum[0]=sum[1]=sum[2]=0;
			gAppParam.ImuCaliList[cali_id].NeedCali=0;
			gAppParam.ImuCaliList[cali_id].isAlreadyCalied = CALIED_FLAG;
			AppParamSave2Flash();
		}
	}
	
}
u8 test[] = "0123456789";

void AppParamSave2Flash(void){

	BSP_FLASH_Write((u8*)&gAppParam, sizeof(AppParam_t));

}

void AppParamReadFromFlash(){
	//AppParam_t tmp;
	memcpy((void*)&gAppParam, (void*)PARAM_SAVED_START_ADDRESS, sizeof(AppParam_t));
}




#include "mytype.h"


//enum the cali result
typedef enum
{
    CALI_STATE_ERR,
    CALI_STATE_IN,
    CALI_STATE_DONE,
}CALI_STATE_e;

typedef enum
{
	CALI_GYRO,
	CALI_ACC,
	CALI_MAG,
	CALI_IMU_LIST_LEN,
	CALI_GIMBAL,
	//add more...
}CALI_ID_e;

#define PARAM_SAVED_FLAG                            0x5A   //header of the structure
#define PARAM_CALI_DONE                             0x5A 		
#define PARAM_CALI_NONE                             0x00
#define CALIED_FLAG																	0x55
#define CALI_START_FLAG_GYRO                  ((uint32_t)1<<1)
#define CALI_END_FLAG_GYRO                    ((uint32_t)1<<2)
#define CALI_START_FLAG_ACC                   ((uint32_t)1<<3)
#define CALI_START_FLAG_MAG                   ((uint32_t)1<<4)
#define CALI_END_FLAG_MAG                     ((uint32_t)1<<5)
#define CALI_START_FLAG_GIMBAL                ((uint32_t)1<<6)
#define CALI_END_FLAG_GIMBAL                  ((uint32_t)1<<7)
#define CALI_FLAG_PID         				  			((uint32_t)1<<8)
#define CALI_FLAG_PITCH_SPEED_PID             ((uint32_t)1<<9)
#define CALI_FLAG_YAW_POSITION_PID            ((uint32_t)1<<10)
#define CALI_FLAG_YAW_SPEED_PID               ((uint32_t)1<<11)

typedef __packed struct
{
    int16_t     GimbalYawOffset;
    int16_t     GimbalPitOffset;
    uint8_t     GimbalNeedCali;
		uint8_t			isAlreadyCalied;		//already calied
}GimbalCaliStruct_t;

typedef __packed struct
{
    int16_t     offset[3];					//x,y,z
    uint8_t     NeedCali;						//1=need, 0=not
		uint8_t			isAlreadyCalied;		//0x55 = already calied, else not
}ImuCaliStruct_t;

typedef __packed struct 
{
  uint8_t     ParamSavedFlag;    				//header 
  uint32_t    FirmwareVersion;    			//version
  GimbalCaliStruct_t GimbalCaliData;          //gimbal pitch yaw encoder offset
//  GyroCaliStruct_t   GyroCaliData;            //gyro offset data
//  AccCaliStruct_t    AccCaliData;    		    //ACC offset data
//  MagCaliStruct_t    MagCaliData;				//Mag offset data
	ImuCaliStruct_t 	 ImuCaliList[CALI_IMU_LIST_LEN];	// in fact =3
	GimbalCaliStruct_t CameraCali;				//adjust yaw/pit make camera img move to center, record y/p
}AppParam_t;


extern AppParam_t gAppParam;
void AppParamSave2Flash(void);
void AppParamReadFromFlash(void);
void CalibrateHook(CALI_ID_e cali_id);
void imuCaliHook(CALI_ID_e cali_id, s16 raw_xyz[]);
void cameraCaliHook(s16 y,s16 p);

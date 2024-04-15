#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "HI229.h"




/* 电机类型枚举 */
typedef enum 
{
  M_3508 = 0,
  GM_6020_YAW ,
  GM_6020_PIT ,
  M_2006  ,
  FRIC_3508 ,
  MOTOR_TYPE_CNT,
}Motor_Type_t;


/* 云台电机ID信息 */
typedef enum 
{
	GIMBAL_ALL_ID = 0x1FE,   //云台电调ID
  GIM_YAW_ID 	  = 0x205,   //YAW
  GIM_PIT_ID    = 0x206,   //PITCH
}GIMBAL_MOTOR_ID;


/* 拨盘电机ID信息 */
typedef enum 
{
	DRIVER_ALL_ID = 0x1FF,
  DRIVER_ID     = 0x207,   
}DRIVER_MOTOR_ID;


/* 摩擦轮电机ID信息 */
typedef enum 
{
  FRIC_ALL_ID = 0x200,
  FRIC_L_ID   = 0x201,   
  FRIC_R_ID   = 0x202,
}FRIC_MOTOR_ID;


/*电机信息结构体*/
typedef struct 
{
  Motor_Type_t Motor_Type;
}Motor_Info_t;


/*CAN接收到的数据 结构体*/
typedef __packed struct 
{
  int16_t Motor_Angle;
	int16_t Motor_Speed;
  int16_t Motor_ELC;
  uint8_t Motor_Temp;   
}CAN_GET_DATA_t;


/*串级PID的数据 结构体*/
typedef  struct 
{
  float angle;//电机机械角度
	float lastAngle;//电机上一刻机械角度
	float totalAngle;//电机总的机械角度
	float conversion_angle;//电机转换角度
}Relative_Angle_t;


/*电机IMU接收到的数据 结构体*/
typedef __packed struct
{
  float IMU_Speed;
  float IMU_Angle;
}IMU_CAN_DATA_t;

typedef struct
{
	CAN_GET_DATA_t CAN_GetData;
	IMU_CAN_DATA_t IMU_GetData;
  /* 参与计算的pid数据 */
  float PID_Speed;
  float PID_Speed_target;
  float PID_Angle; 
  float PID_Angle_target; 
}Motor_Data_t;



extern IMU_GET_DATA_t IMU_Get_Data;
extern PID_Parameter_t PID_Speed_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT];
extern PID_Parameter_t PID_Angle_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT];









void get_motor_measure(CAN_GET_DATA_t *ptr, uint8_t *data);


void CAN_cmd_gimbal_yaw(int16_t yaw);
void CAN_cmd_shoot( int16_t shoot1,int16_t shoot2);
void CAN_cmd_gimbal_pitch(int16_t pitch,int16_t driver);

void CAN_cmd_RC1(int16_t ch0,int16_t ch1,int16_t ch2,int16_t ch3);
void CAN_cmd_RC2(uint8_t s1,uint8_t s2,int16_t sw,uint16_t key);

void get_total_angle(CAN_GET_DATA_t *motor_get);


#endif


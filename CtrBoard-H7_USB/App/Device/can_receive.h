#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "HI229.h"




/* �������ö�� */
typedef enum 
{
  M_3508 = 0,
  GM_6020_YAW ,
  GM_6020_PIT ,
  M_2006  ,
  FRIC_3508 ,
  MOTOR_TYPE_CNT,
}Motor_Type_t;


/* ��̨���ID��Ϣ */
typedef enum 
{
	GIMBAL_ALL_ID = 0x1FE,   //��̨���ID
  GIM_YAW_ID 	  = 0x205,   //YAW
  GIM_PIT_ID    = 0x206,   //PITCH
}GIMBAL_MOTOR_ID;


/* ���̵��ID��Ϣ */
typedef enum 
{
	DRIVER_ALL_ID = 0x1FF,
  DRIVER_ID     = 0x207,   
}DRIVER_MOTOR_ID;


/* Ħ���ֵ��ID��Ϣ */
typedef enum 
{
  FRIC_ALL_ID = 0x200,
  FRIC_L_ID   = 0x201,   
  FRIC_R_ID   = 0x202,
}FRIC_MOTOR_ID;


/*�����Ϣ�ṹ��*/
typedef struct 
{
  Motor_Type_t Motor_Type;
}Motor_Info_t;


/*CAN���յ������� �ṹ��*/
typedef __packed struct 
{
  int16_t Motor_Angle;
	int16_t Motor_Speed;
  int16_t Motor_ELC;
  uint8_t Motor_Temp;   
}CAN_GET_DATA_t;


/*����PID������ �ṹ��*/
typedef  struct 
{
  float angle;//�����е�Ƕ�
	float lastAngle;//�����һ�̻�е�Ƕ�
	float totalAngle;//����ܵĻ�е�Ƕ�
	float conversion_angle;//���ת���Ƕ�
}Relative_Angle_t;


/*���IMU���յ������� �ṹ��*/
typedef __packed struct
{
  float IMU_Speed;
  float IMU_Angle;
}IMU_CAN_DATA_t;

typedef struct
{
	CAN_GET_DATA_t CAN_GetData;
	IMU_CAN_DATA_t IMU_GetData;
  /* ��������pid���� */
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


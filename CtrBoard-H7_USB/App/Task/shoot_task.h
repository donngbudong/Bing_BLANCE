#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "can_receive.h"
#include "System.h"


//#define ECD_FULL_ROUND 8192
//typedef uint8_t 	u8;
//typedef uint16_t 	u16;
//typedef uint32_t 	u32;



//拨盘电机射击模式枚举类型
typedef enum
{
	Single		=1,
	Continous	=2,
//	Stop			=3,
}Fire_Mode_t;


/* 拨盘电机数组下标 */
typedef enum 
{
  FRIC_L = 0,//左
	FRIC_R = 1,
	DRIVER = 2,
  RIFLE_MOTOR_CNT = 3,
}SHOOT_Motor_cnt_t;

typedef struct
{
  Motor_Info_t Motor_Info;
  Motor_Data_t Motor_Data;
  PID_Info_t 	PID;
}Shoot_Motor_t;


typedef struct
{
	System_State_t State;
	System_Ctrl_Mode_t Ctrl_Mode;
	Shoot_Motor_t Motor_Data[RIFLE_MOTOR_CNT];
	PID_Type_t PID_Type;
	Fire_Mode_t Mode;
}Shoot_Date_t;



void Shoot_Task(void);
void Shoot_GET_Info(void);
void Shoot_Get_PID_Type(Shoot_Motor_t *strSho);
void Shoot_Set_PID_Type(void);

void Shoot_InitPID(void);

void Shoot_PID_Switch(Shoot_Motor_t *strSho);
void Shoot_RC_Ctrl(void);
void Shoot_KEY_Ctrl(void);

void Shoot_Ctrl_FRIC_L(Shoot_Motor_t *str,int32_t speed);
void Shoot_Ctrl_FRIC_R(Shoot_Motor_t *str,int32_t speed);
void Shoot_Ctrl_DRIVER(Shoot_Motor_t *str);

float Shoot_GetOutPut(Shoot_Motor_t *str);
void Shoot_Ctrl_FRIC(Shoot_Motor_t *str);

void Shoot_CanOutPut(void);
void Shoot_Stop(void);

void driver_out(void);
void Shoot_DRIVER_CanOutPut_0(void);
void Shoot_DRIVER_CanOutPut(void);

void Shoot_KEY_F(void);
void Shoot_KEY_Fire(void);

extern Shoot_Date_t Shoot;


#endif

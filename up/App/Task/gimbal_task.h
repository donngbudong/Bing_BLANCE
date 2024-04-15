#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "can_receive.h"
#include "System.h"
#define MECH_Mid_Angle   	5410



#define IMU_90          	90
#define IMU_180        	 	180
#define IMU_360         	360

#define Motor_90          2048
#define Motor_180         4096
#define Motor_360         8192

#define PITCH_MAX					21
#define PITCH_MIX					-20

/* 云台电机数组下标 */
typedef enum 
{
  GIM_YAW = 0,
  GIM_PITCH = 1,
  GIM_MOTOR_CNT = 2,
}GIM_Motor_cnt_t;

typedef struct
{
  float Angle_Inc;
  float Angle_k;//角度增加的幅度调节因子
  float Slow_Inc;
}Gimbal_Move_t;

typedef struct
{
  Motor_Info_t  Motor_Info;
  Motor_Data_t  Motor_Data;
  PID_Info_t    PID;
	
	Gimbal_Move_t RC_Move;
	Gimbal_Move_t	KEY_Move;
	
}Gimbal_Info_t;


typedef struct 
{
	System_State_t State;
	System_Ctrl_Mode_t Ctrl_Mode;
	Gimbal_Info_t YAW;
	Gimbal_Info_t PITCH;
  PID_Type_t PID_Type;
}Gimbal_Date_t;


void Gimbal_Task(void);

void Gimbal_GET_Info(void);
void Gimbal_Get_PID_Type(Gimbal_Info_t *strGim);
void Gimbal_Set_PID_type(void);
void Gimbal_InitPID(void);
void Gimbal_PID_Switch(Gimbal_Info_t *strGim);

void Gimbal_RC_Ctrl(void);
void Gimbal_KRY_Ctrl(void);


void RC_Ctrl_YAW(Gimbal_Info_t *str);
void RC_Ctrl_PITCH(Gimbal_Info_t *str);
void KEY_Ctrl_YAW(Gimbal_Info_t *str);
void KEY_Ctrl_PITCH(Gimbal_Info_t *str);
void DEBUG_Ctrl_YAW(Gimbal_Info_t *str);
void DEBUG_Ctrl_PITCH(Gimbal_Info_t *str);

float YAW_MotorAngle_Proc(int16_t Angle);
float YAW_Angle_Over_Zero(float *Angle);
float PITCH_Angle_Limit(float *Angle);

float Gimbal_GetOutPut(Gimbal_Info_t *str);
void Gimbal_CanOutPut(void);
void Gimbal_Stop(void);

void Gimbal_FirstYawAngle(Gimbal_Info_t *str);
void Gimbal_FirstPitchAngle(Gimbal_Info_t *str);

void IMU_YawData_Report(Motor_Data_t *str);
void IMU_PitData_Report(Motor_Data_t *str);
void IMU_Data_Report(void);
bool JudgeFireFlag(Gimbal_Date_t *AimTra);



extern Gimbal_Date_t Gimbal;
#endif

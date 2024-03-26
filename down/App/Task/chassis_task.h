#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
/************************************** Includes **************************************/
//#include "can_receive.h"

#include "Device.h"
#include "system.h"
//#include "pid.h"

//PI 
#ifndef PI
#define PI 3.14159265358979f
#endif

/********************************* Exported functions *********************************/
#define IMU_90          	90
#define IMU_180        	 	180
#define IMU_360         	360

#define Diameter_weel 			0.190f

#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 75


/* 底盘电机枚举 */
typedef enum 
{
  MF9025_R = 0,
  MF9025_L = 1,
  CHAS_MOTOR_CNT = 2,
}CHAS_Motor_cnt_t;

/*前后切换枚举*/
typedef enum
{
	CHASSIS_FRONT	= 0,
	CHASSIS_BACK	= 1,
}chassis_direction_e;




/* 底盘数据结构体*/
typedef struct
{
  System_State_t State;
	System_Ctrl_Mode_t Ctrl_Mode;
	Motor_Info_t  Motor_Info;

	CAN_MF9025_DATE_T Motor_Date[CHAS_MOTOR_CNT];					//电机结构体
	PID_Chassis_Info_t PID;											//PID
	PID_Type_t PID_Type;											//PID类型
	
	uint16_t HEAD;
	float torque_const;												//转矩系数

	float follow_gimbal_zero;									//正方向

	float speed_x,target_speed_x;									//速度、目标速度
	float pose_x,target_pose_x;										//位移、目标位移
	float Pitch;													//前倾角
	float Yaw;														//
	float angle_z;
	float Gyo_y;													//???????????
	float Gyo_z;													//????????????

	float GIM_Yaw;												//????????????yaw??6020???
	float GIM_Yaw_t;												//????????????yaw??6020???

	float omega_z;													//????????????

	float X_Speed_k ,Y_Speed_k,Z_Speed_k; 							//???->??? ?仯????????????
	float X_Target,Y_Target,Z_Target;								//??????

	uint8_t chassis_direction;										//??????????(CHASSIS_FRONT,CHASSIS_BACK)

	float move_speed,move_direction;								//?????????????????

	float torque_speed;												//??????????
	float torque_balance;											//????????
	float torque_revolve;											//??????????
	float iqControl[2];												//?????????

uint8_t flag_clear_pose;											//里程计停止清零标志(0,清零;1,停止清零)

}CHASSIS_Date_t;




extern CHASSIS_Date_t Chassis;


void Chassis_Task(void);
void Chassis_Static(void);

void Chassis_Balance(void);
void Chassis_Normal(void);
void Chassis_Normal_s(void);
void Chassis_Normal_z(void);


void Chassis_Get_PID_Type(PID_Info_t *str);
void Chassis_Set_PID_Type(void);
void Chassis_InitPID(void);
void Chassis_Stop(void);
void Chassis_PID_Set(PID_Chassis_Info_t *str);




#endif

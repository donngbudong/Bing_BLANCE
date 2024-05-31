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

#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 5710


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

	float speed_x,target_speed_x;							//速度、目标速度
	float pose_x,target_pose_x;								//位移、目标位移
	float Pitch;															//前倾角
	float Yaw;																//航向角
	float angle_z;
	float Gyo_y;															//底盘倾斜角速度
	float Gyo_z;															//底盘旋转角速度

	float GIM_Yaw;														//????????????yaw??6020???
	float GIM_Yaw_t;													//????????????yaw??6020???

	float omega_z;														//底盘旋转线速度

	float X_Speed_k ,Y_Speed_k,Z_Speed_k; 					//速度系数
	float X_Target,Y_Target,Z_Target;								//

	uint8_t chassis_direction;								//底盘正方向(CHASSIS_FRONT,CHASSIS_BACK)
	float move_speed,move_direction;						//底盘平移速度，平移方向
	
	uint8_t ctrl_mode;												//底盘模式
	uint8_t ctrl_mode_last;										//上次底盘模式
	
	float torque_speed;												//前进动力相应转矩
	float torque_balance;											//旋转动力相应转矩
	float torque_revolve;											//平衡动力相应转矩
	float iqControl[2];												//底盘电机目标电流

	uint8_t flag_clear_pose;											//里程计停止清零标志(0,清零;1,停止清零)

}CHASSIS_Date_t;




extern CHASSIS_Date_t Chassis;
typedef enum
{
	MODE_STATIC 		= 0,
	MODE_NORMAL 		= 1,
	MODE_WEAK 			= 2,
	MODE_STOP 			= 3,
	MODE_BALANCE 		= 4,
	MODE_BALANCE_SPEED = 5,
	MODE_SLIPPED		= 6,
}chassis_mode_e;

void mode_switch_chassis(uint8_t mode);


void Chassis_Task(void);
void Chassis_Static(void);

void Chassis_Balance(void);
void Chassis_Normal(void);
void Chassis_Normal_Speed(void);
void Chassis_Normal_s(void);
void Chassis_Normal_z(void);


void Chassis_Get_PID_Type(PID_Info_t *str);
void Chassis_Set_PID_Type(void);
void Chassis_InitPID(void);
void Chassis_Stop(void);
void Chassis_PID_Set(PID_Chassis_Info_t *str);


void Chassis_KEY_C(void);


#endif

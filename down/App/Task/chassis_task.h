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


/* ���̵��ö�� */
typedef enum 
{
  MF9025_R = 0,
  MF9025_L = 1,
  CHAS_MOTOR_CNT = 2,
}CHAS_Motor_cnt_t;

/*ǰ���л�ö��*/
typedef enum
{
	CHASSIS_FRONT	= 0,
	CHASSIS_BACK	= 1,
}chassis_direction_e;




/* �������ݽṹ��*/
typedef struct
{
  System_State_t State;
	System_Ctrl_Mode_t Ctrl_Mode;
	Motor_Info_t  Motor_Info;

	CAN_MF9025_DATE_T Motor_Date[CHAS_MOTOR_CNT];					//����ṹ��
	PID_Chassis_Info_t PID;											//PID
	PID_Type_t PID_Type;											//PID����
	
	uint16_t HEAD;
	float torque_const;												//ת��ϵ��

	float follow_gimbal_zero;									//������

	float speed_x,target_speed_x;							//�ٶȡ�Ŀ���ٶ�
	float pose_x,target_pose_x;								//λ�ơ�Ŀ��λ��
	float Pitch;															//ǰ���
	float Yaw;																//�����
	float angle_z;
	float Gyo_y;															//������б���ٶ�
	float Gyo_z;															//������ת���ٶ�

	float GIM_Yaw;														//????????????yaw??6020???
	float GIM_Yaw_t;													//????????????yaw??6020???

	float omega_z;														//������ת���ٶ�

	float X_Speed_k ,Y_Speed_k,Z_Speed_k; 					//�ٶ�ϵ��
	float X_Target,Y_Target,Z_Target;								//

	uint8_t chassis_direction;								//����������(CHASSIS_FRONT,CHASSIS_BACK)
	float move_speed,move_direction;						//����ƽ���ٶȣ�ƽ�Ʒ���
	
	uint8_t ctrl_mode;												//����ģʽ
	uint8_t ctrl_mode_last;										//�ϴε���ģʽ
	
	float torque_speed;												//ǰ��������Ӧת��
	float torque_balance;											//��ת������Ӧת��
	float torque_revolve;											//ƽ�⶯����Ӧת��
	float iqControl[2];												//���̵��Ŀ�����

	uint8_t flag_clear_pose;											//��̼�ֹͣ�����־(0,����;1,ֹͣ����)

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

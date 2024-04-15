#ifndef __PID_H_
#define __PID_H_
/************************************** Includes **************************************/
//#include "Device.h"

/********************************* Exported functions *********************************/

typedef enum
{
	Clear_Away  =0,	    //���
	KEY         =1,			//����
	RC          =2,			//ң��
	PID_TYPE_CNT,
}PID_Type_t;


/* PID���� */
typedef __packed struct 
{
  float P,I,D,F;
}PID_Parameter_t;


typedef __packed struct
{
  PID_Parameter_t PID_Param;
	float Target;
	float Last_Target;
	float Actual;
  float Err;
  float Last_Err;
	
  float I_Limit;				//I�����޷�
  float I_Limit_Max;	
	
  float PID_P_Out;
  float PID_I_Out;
  float PID_D_Out;
	float PID_F_Out;
  float PID_Output;
  float PID_Err_Dead;			//����
  float PID_Output_Max;		//����޷�
  float PID_I_Out_Max;		
  float PID_P_Out_Max;
}PID_Loop_t;

/*����PID���ݽṹ*/
typedef struct 
{  
	PID_Loop_t PID_b1;
	PID_Loop_t PID_b2;
	
  PID_Loop_t PID_b3;
	PID_Loop_t PID_b4;
	
	PID_Loop_t PID_b5;
	PID_Loop_t PID_b6;
	
	PID_Loop_t Chasssis_OUT;
  PID_Type_t PID_Type;
}PID_Chassis_Info_t;

/*��̨PID���ݽṹ��*/
typedef struct 
{
  PID_Loop_t Speed_Loop;
	PID_Loop_t Angle_Loop;
  PID_Type_t PID_Type;
}PID_Info_t;


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


float PID_Position(PID_Loop_t *pid, float target, float actual);

void Motor_Init(void);
void GM_6020_Init(void);
void M_3508_ParamInit(PID_Loop_t *str);
void Fric_3508_Init(void);
		
void M_2006_ParamInit(void);
void M_2006_Init(void);

//void GIM_YawParamInit(Gimbal_Info_t *str);
//void GIM_PitParamInit(Gimbal_Info_t *str);


#endif

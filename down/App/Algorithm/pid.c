#include "pid.h"
#include "Device.h"


//反限幅（死区）
float anti_constrain(float amt,float high,float low)
{
	if (amt > low && amt < high)
		return 0;
	else
		return amt;
}


/* 用于调试PID参数的函数 */
PID_Parameter_t PID_SpeedDebug = {
  .P = 0,
  .I = 0,
  .D = 0,
};
PID_Parameter_t PID_AngleDebug = {
  .P = 0,
  .I = 0,
  .D = 0,
};
void PID_Debug(PID_Info_t *str)
{
  str->Speed_Loop.PID_Param.P = PID_SpeedDebug.P;
  str->Speed_Loop.PID_Param.I = PID_SpeedDebug.I;
  str->Speed_Loop.PID_Param.D = PID_SpeedDebug.D;
  
  str->Angle_Loop.PID_Param.P = PID_AngleDebug.P;
  str->Angle_Loop.PID_Param.I = PID_AngleDebug.I;
  str->Angle_Loop.PID_Param.D = PID_AngleDebug.D;
}
			
/**
  * @brief  PID(速度/角度)环控制器
  * @param  目标-真实
  * @retval None
  */
float PID_Position(PID_Loop_t *pid, float target, float actual)
{
	pid->Target=target;
	pid->Actual=actual;
	//误差=目标值-实际值
	pid->Err = target - actual;
  	/*死区*/
  pid->Err = anti_constrain(pid->Err,pid->PID_Err_Dead,-pid->PID_Err_Dead);
	//积分限幅
	pid->I_Limit = pid->I_Limit + pid->Err;
	LimitMax(pid->I_Limit,pid->I_Limit_Max);

	pid->PID_P_Out = pid->PID_Param.P * pid->Err;
	pid->PID_I_Out = pid->PID_Param.I * pid->I_Limit;
	pid->PID_D_Out = pid->PID_Param.D * (pid->Err - pid->Last_Err);
	//I输出限幅
	LimitMax(pid->PID_I_Out,pid->PID_I_Out_Max);
	pid->PID_Output = pid->PID_P_Out+pid->PID_I_Out+pid->PID_D_Out;
	
	pid->Last_Err = pid->Err;
	//总输出限幅
	LimitMax(pid->PID_Output, pid->PID_Output_Max);
	
	return pid->PID_Output;
}




/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(PID_Loop_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Err = 0.0f;
    pid->PID_Output = pid->PID_P_Out = pid->PID_I_Out = pid->PID_D_Out = 0.0f;
    pid->Target = pid->Actual = 0.0f;
}







//PID_Parameter_t Param_1 = {800,	0, 110000};
PID_Parameter_t Param_1 = {0,	0, 0};
PID_Parameter_t Param_2 = {450,18, 0};

PID_Parameter_t Param_3 = {280,	0,	0};
PID_Parameter_t Param_4 = {18,	0,	0};

PID_Parameter_t Param_5 = {5 	,	0,	0};
PID_Parameter_t Param_6 = {3	,	0,	0};

PID_Parameter_t Chasssis_OUT = {1,0,0};


/**
 * @brief 总电机初始化
 * @param 
 */
void Motor_Init(void)
{
  MF_9025_Init();
}


void MF_9025_Init(void)
{
	Chassis.Motor_Info.Motor_Type = MF_9025_Right;
	MF_9028_Balance_1(&Chassis.PID.PID_b1);
	MF_9028_Balance_2(&Chassis.PID.PID_b2);
	MF_9028_Balance_Kp(&Chassis.PID.PID_b3);
	MF_9028_Balance_Kd(&Chassis.PID.PID_b4);
	MF_9028_PID_56(&Chassis.PID.PID_b5);
	MF_9028_PID_56(&Chassis.PID.PID_b6);

	MF_9028_Chasssis_OUT(&Chassis.PID.Chasssis_OUT);

}


/**
 * @brief PID_1限幅设置 
 * @param 
 */
void MF_9028_Balance_1(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 0;
  str->PID_I_Out_Max = 8000; 
  str->PID_P_Out_Max = 16000;
  str->PID_Output_Max = 3000;
  str->PID_Err_Dead = 0;
}
/**
 * @brief PID_12限幅设置 
 * @param 
 */
void MF_9028_Balance_2(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 60;
  str->PID_I_Out_Max = 2500; 
  str->PID_P_Out_Max = 16000;
  str->PID_Output_Max = 5000;
  str->PID_Err_Dead = 0;
}
/**
 * @brief PID限幅设置 
 * @param 
 */

void MF_9028_Balance_Kp(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 10000;
  str->PID_I_Out_Max = 8000; 
  str->PID_P_Out_Max = 16000;
  str->PID_Output_Max = 16000;
  str->PID_Err_Dead = 0;
}

/**
 * @brief PID限幅设置 
 * @param 
 */
void MF_9028_Balance_Kd(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 10000;
  str->PID_I_Out_Max = 0; 
  str->PID_P_Out_Max = 16000;
  str->PID_Output_Max = 16000;
  str->PID_Err_Dead = 0;
}

/**
 * @brief PID限幅设置 
 * @param 
 */
void MF_9028_PID_56(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 2000;
  str->PID_I_Out_Max = 0; 
  str->PID_P_Out_Max = 16000;
  str->PID_Output_Max = 12000;
  str->PID_Err_Dead = 5;
}
/**
 * @brief PID限幅设置 
 * @param 
 */
void MF_9028_Chasssis_OUT(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 10000;
  str->PID_I_Out_Max = 30; 
  str->PID_P_Out_Max = 50000;
  str->PID_Output_Max = 2000;
  str->PID_Err_Dead = 0;
}




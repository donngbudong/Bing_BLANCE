/***      
《 云台控制代码 》
***/
#include "gimbal_task.h"
#include "Device.h"
#include "math.h"



/* Init start */
Gimbal_Date_t Gimbal ={
	.State = SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
	.PID_Type = Clear_Away,		//Clear_Away
};

/*Init end*/
static bool fire_flag = 0; // 开火标志位
static uint32_t start_fire_time = 0; // 开始开火的时间
extern int16_t Shoot_pid_out;				//拨弹M2006

static PID_Type_t YAW_prev_pid_type = Clear_Away; 
/**
 * @brief Gimbal总控
 * @param 
 */
void Gimbal_Task(void)
{
	Gimbal_GET_Info();

	if(RC_S1== RC_SW_UP)
	{
		Gimbal_RC_Ctrl();
	}
	else if(RC_S1 == RC_SW_DOWN)
	{
		Gimbal_KRY_Ctrl();
	}
	else if(RC_S1 == RC_SW_MID)
	{
			if(YAW_prev_pid_type != Gimbal.PID_Type)
		{/* 第一次进入模式 */
			YAW_prev_pid_type = Gimbal.PID_Type;
		}
		Gimbal_Stop();
	}
}


/**
 * @brief 云台信息获取
 * @param 
 */
void Gimbal_GET_Info(void)
{
	Gimbal.State = System.System_State;
	Gimbal.PID_Type = System.System_Pid;
	Gimbal_Set_PID_type();
	Gimbal_InitPID();
	IMU_Data_Report();
}



/**
 * @brief PID种类获取
 * @note  本地同步
 */
void Gimbal_Get_PID_Type(Gimbal_Info_t *strGim)
{
  strGim->PID.PID_Type = Gimbal.PID_Type;
}

void Gimbal_Set_PID_type(void)
{
  Gimbal_Get_PID_Type(&Gimbal.YAW);
  Gimbal_Get_PID_Type(&Gimbal.PITCH);    
}

/**
 * @brief 云台PID设置总函数
 * @param 
 */
void Gimbal_InitPID(void)
{  
  Gimbal_PID_Switch(&Gimbal.YAW);
  Gimbal_PID_Switch(&Gimbal.PITCH); 
}



/**
 * @brief 云台PID切换
 * @param 
 */
void Gimbal_PID_Switch(Gimbal_Info_t *strGim)
{
	PID_Info_t *str = &(strGim->PID);
	Motor_Info_t *strx = &(strGim->Motor_Info);

	static int length = sizeof(PID_Parameter_t);
 switch (strx->Motor_Type)
 {
	case GM_6020_YAW:{
		switch (str->PID_Type)
      {
        case RC:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_YAW][RC],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_YAW][RC],length);
        }break;
        case KEY:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_YAW][KEY],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_YAW][KEY],length);
        }break;
         default:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_YAW][Clear_Away],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_YAW][Clear_Away],length);
        }break;
			}
	}
	 case GM_6020_PIT:{
		 	switch (str->PID_Type)
      {
        case RC:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_PIT][RC],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_PIT][RC],length);
        }break;
        case KEY:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_PIT][KEY],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_PIT][KEY],length);
        }break;
         default:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_PIT][Clear_Away],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_PIT][Clear_Away],length);
        }break;
			}
   }	 
	}
}




/**
 * @brief 遥控器控制云台
 * @param 
 */
void Gimbal_RC_Ctrl(void)
{
	RC_Ctrl_YAW(&Gimbal.YAW);
	RC_Ctrl_PITCH(&Gimbal.PITCH);

	Gimbal_CanOutPut();
}

/**
 * @brief Gimbal_YAW轴的控制 遥控器遥控模式
 * @param 
 */
void RC_Ctrl_YAW(Gimbal_Info_t *str)
{
//	PID_Debug(&str->PID);
	if(YAW_prev_pid_type != Gimbal.PID_Type)
		{/* 第一次进入模式 */
			YAW_prev_pid_type = Gimbal.PID_Type;
			Gimbal_FirstYawAngle(&Gimbal.YAW);
			Gimbal_FirstPitchAngle(&Gimbal.PITCH);
		}
	
	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * (-RC_CH0);
		
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;
//	if(RC_S2 == 1){
//	str->Motor_Data.PID_Angle_target = 100;}
//	if(RC_S2 == 3){
//	str->Motor_Data.PID_Angle_target = 80;}
//	if(RC_S2 == 2){
//	str->Motor_Data.PID_Angle_target = 60;}
	YAW_Angle_Over_Zero(&str->Motor_Data.PID_Angle_target);
	
	/*同步陀螺仪与YAW数据*/
	str->Motor_Data.PID_Speed = str->Motor_Data.IMU_GetData.IMU_Speed;
	str->Motor_Data.PID_Angle = str->Motor_Data.IMU_GetData.IMU_Angle;
}
/**
 * @brief Gimbal_PITCH轴的控制遥控器遥控模式
 * @param 
 */
void RC_Ctrl_PITCH(Gimbal_Info_t *str)
{
//	PID_Debug(&str->PID);
	if(YAW_prev_pid_type != Gimbal.PID_Type)
	{/* 第一次进入模式 */
		YAW_prev_pid_type = Gimbal.PID_Type;
		Gimbal_FirstPitchAngle(&Gimbal.PITCH);
	}

	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * RC_CH1;
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;
//	if(RC_S2 == 1){
//	str->Motor_Data.PID_Angle_target = -20;}
//	if(RC_S2 == 3){
//	str->Motor_Data.PID_Angle_target = 0;}
//	if(RC_S2 == 2){
//	str->Motor_Data.PID_Angle_target = 20;}
	PITCH_Angle_Limit(&str->Motor_Data.PID_Angle_target);
	
	/*同步陀螺仪与YAW数据*/
	str->Motor_Data.PID_Speed = str->Motor_Data.IMU_GetData.IMU_Speed;
	str->Motor_Data.PID_Angle = str->Motor_Data.IMU_GetData.IMU_Angle;
}

/**
 * @brief 键鼠控制云台
 * @param 
 */
void Gimbal_KRY_Ctrl(void)
{
	KEY_Ctrl_YAW(&Gimbal.YAW);
	KEY_Ctrl_PITCH(&Gimbal.PITCH);

	System.Flag_State.fire_flag = JudgeFireFlag(&Gimbal);
	
	Gimbal_CanOutPut();
}
/**
 * @brief Gimbal_YAW轴的控制 KEY遥控模式
 * @param 
 */

void KEY_Ctrl_YAW(Gimbal_Info_t *str)
{
	if(YAW_prev_pid_type != Gimbal.PID_Type)
		{/* 第一次进入模式 */
			YAW_prev_pid_type = Gimbal.PID_Type;
			Gimbal_FirstYawAngle(&Gimbal.YAW);
			Gimbal_FirstPitchAngle(&Gimbal.PITCH);
		}
	if(MOUSE_RIGH == 0 || RC_S1 != RC_SW_DOWN)
	{		
		str->KEY_Move.Angle_Inc = str->KEY_Move.Angle_k * (MOUSE_X_MOVE_SPEED);
		str->Motor_Data.PID_Angle_target -= str->KEY_Move.Angle_Inc;
	}
	else if(MOUSE_RIGH == 1 || RC_S1 == RC_SW_DOWN)/*开启自瞄*/
	{
		if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 0)
		{
			str->KEY_Move.Angle_Inc = str->KEY_Move.Angle_k * (MOUSE_X_MOVE_SPEED);
			str->Motor_Data.PID_Angle_target -= str->KEY_Move.Angle_Inc;
		}
		else if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 1)
		{
			str->Motor_Data.PID_Angle_target = st.yaw; //- st.v_yaw * 10;
		}

	}
	YAW_Angle_Over_Zero(&str->Motor_Data.PID_Angle_target);
	/*同步陀螺仪与YAW数据*/
	str->Motor_Data.PID_Speed = str->Motor_Data.IMU_GetData.IMU_Speed;
	str->Motor_Data.PID_Angle = str->Motor_Data.IMU_GetData.IMU_Angle;
}

/**
 * @brief Gimbal_PITCH轴的控制KEY遥控模式
 * @param 
 */
void KEY_Ctrl_PITCH(Gimbal_Info_t *str)
{
	if(YAW_prev_pid_type != Gimbal.PID_Type)
	{/* 第一次进入模式 */
		YAW_prev_pid_type = Gimbal.PID_Type;
		Gimbal_FirstPitchAngle(&Gimbal.PITCH);
	}
	if(MOUSE_RIGH == 0 || RC_S1 != RC_SW_DOWN)
	{
		str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * MOUSE_Y_MOVE_SPEED;
		str->Motor_Data.PID_Angle_target -= str->RC_Move.Angle_Inc;
	}
	else if(MOUSE_RIGH == 1 || RC_S1 == RC_SW_DOWN)/*开启自瞄*/
	{
		if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 0)
		{
			str->KEY_Move.Angle_Inc = str->KEY_Move.Angle_k * (MOUSE_X_MOVE_SPEED);
			str->Motor_Data.PID_Angle_target -= str->KEY_Move.Angle_Inc;
		}
		else if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 1)
		{
			str->Motor_Data.PID_Angle_target = st.pitch ;
		}
	}
	
	PITCH_Angle_Limit(&str->Motor_Data.PID_Angle_target);
	/*同步陀螺仪与YAW数据*/
	str->Motor_Data.PID_Speed = str->Motor_Data.IMU_GetData.IMU_Speed;
	str->Motor_Data.PID_Angle = str->Motor_Data.IMU_GetData.IMU_Angle;
}

/**
 * @brief 底盘YAW轴跟随处理
 * @param 
 */
float YAW_MotorAngle_Proc(int16_t Angle)
{
	if(Angle < MECH_Mid_Angle - Motor_180)
		Angle += Motor_360;
	if(Angle > MECH_Mid_Angle + Motor_180)
		Angle -= Motor_360;
	
  return (float)Angle;
}

/**
 * @brief YAW轴角度过零处理
 * @param 
 */
float YAW_Angle_Over_Zero(float *Angle)
{
	if(*Angle - Gimbal.YAW.Motor_Data.IMU_GetData.IMU_Angle > IMU_180)    
	{
		*Angle =*Angle - IMU_360;        
	}
	else if(*Angle - Gimbal.YAW.Motor_Data.IMU_GetData.IMU_Angle < -IMU_180)
	{
		*Angle =*Angle + IMU_360;
	}
	return *Angle;
}


/**
 * @brief PICTH轴角度限位
 * @param 
 */
float PITCH_Angle_Limit(float *Angle)
{
	if(*Angle > PITCH_MAX)    
	{
		*Angle = PITCH_MAX;        
	}
	if(*Angle < PITCH_MIX)
	{
		*Angle =PITCH_MIX;
	}
	return *Angle;
}





/**
 * @brief 获取串级PID输出
 * @param 
 */
float Gimbal_GetOutPut(Gimbal_Info_t *str)
{
  PID_Info_t *PID = &(str->PID);
  Motor_Data_t *DATA = &(str->Motor_Data);
  float res;
   
  /*速度环期望 = 角度环PID输出*/
  DATA->PID_Speed_target = PID_Position(&PID->Angle_Loop,
																				str->Motor_Data.PID_Angle_target ,\
																				str->Motor_Data.PID_Angle); \
  /*返回值 = 速度环PID输出*/
  res = PID_Position(&PID->Speed_Loop,
										str->Motor_Data.PID_Speed_target,\
										str->Motor_Data.PID_Speed);\
  return res;
}



/**
 * @brief Gimbal电机输出
 * @param 
 */
void Gimbal_CanOutPut(void)
{
	static int16_t Gimbal_pid_out[4] = {0,0,0,0};

	Gimbal_pid_out[GIM_YAW]	= Gimbal_GetOutPut(&Gimbal.YAW);
	Gimbal_pid_out[GIM_PITCH]= Gimbal_GetOutPut(&Gimbal.PITCH);
	
	CAN_cmd_gimbal_yaw(Gimbal_pid_out[GIM_YAW]);
	CAN_cmd_gimbal_pitch(Gimbal_pid_out[GIM_PITCH],Shoot_pid_out);

//	CAN_cmd_gimbal_pitch(Gimbal_pid_out[GIM_PITCH],Shoot_pid_out);
}









/**
 * @brief 卸力函数
 * @param 
 */
void Gimbal_Stop(void)
{
	static int16_t pid_out[2] = {0, 0};
	
	/* 速度环最终输出 */
  pid_out[GIM_YAW] 	 = 0;
  pid_out[GIM_PITCH] = 0;
  
	CAN_cmd_gimbal_yaw(pid_out[GIM_YAW]);
	CAN_cmd_gimbal_pitch(pid_out[GIM_PITCH],0);
}
/**
 * @brief 初始化陀螺仪模式YAW轴角度
 * @param 
 */
void Gimbal_FirstYawAngle(Gimbal_Info_t *str)
{
  str->Motor_Data.PID_Angle_target = str->Motor_Data.IMU_GetData.IMU_Angle;
}

/**
 * @brief 初始化陀螺仪模式PITCH轴角度
 * @param 
 */
void Gimbal_FirstPitchAngle(Gimbal_Info_t *str)
{
  str->Motor_Data.PID_Angle_target = str->Motor_Data.IMU_GetData.IMU_Angle;
}


/**
 * @brief 获取IMU数据
 * @param 
 */
void IMU_YawData_Report(Motor_Data_t *str)
{
  str->IMU_GetData.IMU_Speed = IMU_Get_Data.IMU_Gyo[Z]/10;
  str->IMU_GetData.IMU_Angle = IMU_Get_Data.IMU_Eular[YAW];
}
void IMU_PitData_Report(Motor_Data_t *str)
{
  str->IMU_GetData.IMU_Speed = IMU_Get_Data.IMU_Gyo[Y]/10;
  str->IMU_GetData.IMU_Angle = IMU_Get_Data.IMU_Eular[PITCH];
}
void IMU_Data_Report(void)
{
  IMU_YawData_Report(&Gimbal.YAW.Motor_Data);
  IMU_PitData_Report(&Gimbal.PITCH.Motor_Data);
}


/**
 * @brief 计算是否符合开火条件
 * @param AimTra:in 瞄准的弹道数据
 * @param gyro_angle_rad:in 当前的陀螺仪数据
 */

bool JudgeFireFlag(Gimbal_Date_t *AimTra)
{
    // 当到达范围内时更新开始开火时间
    // 在之后的0.5s（500ms）内保持开火位为true
    float error_pitch, error_yaw;
    error_pitch = fabs(AimTra->PITCH.Motor_Data.PID_Angle - AimTra->PITCH.Motor_Data.PID_Angle_target);
    error_yaw = fabs(AimTra->YAW.Motor_Data.PID_Angle - AimTra->YAW.Motor_Data.PID_Angle_target);
    bool fire = 0;
    uint32_t now_time = HAL_GetTick();
    if ((error_pitch < 2.0f) &&
        (error_yaw < 0.3f))
    {
        fire = true;
        start_fire_time = now_time;
    }
    else if (now_time - start_fire_time < 500)
    {
        fire = true;
    }
    else
    {
        fire = false;
    }

    return fire;
}


/**
 * @brief   获取开火标志符(1可以开火，0不可以开火)
 * @param   none
 * @return  开火标志符
 */
bool GetFireFlag(void)
{
    return fire_flag;
}

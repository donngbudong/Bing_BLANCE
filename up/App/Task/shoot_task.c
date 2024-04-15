/***      
《 发射机构代码 》
***/
#include "shoot_task.h"
#include "Device.h"



Shoot_Date_t Shoot = {
	.State = SYSTEM_LOST,
	.Ctrl_Mode = RC_CTRL_MODE,
	.PID_Type = RC,
	.Mode = Continous,
};
extern Relative_Angle_t Relative_Angle;

/**
 * @brief Ammo-Booster总控
 * @param 
 */
void Shoot_Task(void)
{
	Shoot_GET_Info();

	if(RC_S1== RC_SW_UP)
	{
		Shoot_RC_Ctrl();
	}
	else if(RC_S1 == RC_SW_DOWN)
	{
		Shoot_KEY_Ctrl();
	}
	else if(RC_S1 == RC_SW_MID)
	{
		Shoot_Stop();
	}
}




/**
* @brief 射击信息获取
* @param 
*/
void Shoot_GET_Info(void)
{
	Shoot.State = System.System_State;
//	Shoot.PID_Type = System.System_Pid;
	Shoot_Set_PID_Type();
	Shoot_InitPID();
}



/**
 * @brief PID种类获取
 * @note  本地同步
 */
void Shoot_Get_PID_Type(Shoot_Motor_t *strSho)
{
  strSho->PID.PID_Type = Shoot.PID_Type;
}

void Shoot_Set_PID_Type(void)
{
  Shoot_Get_PID_Type(&Shoot.Motor_Data[FRIC_L]);
  Shoot_Get_PID_Type(&Shoot.Motor_Data[FRIC_R]);
	Shoot_Get_PID_Type(&Shoot.Motor_Data[DRIVER]);
}


/**
 * @brief 射击PID设置总函数
 * @param 
 */
void Shoot_InitPID(void)
{
  Shoot_PID_Switch(&Shoot.Motor_Data[FRIC_L]);
	Shoot_PID_Switch(&Shoot.Motor_Data[FRIC_R]);
  Shoot_PID_Switch(&Shoot.Motor_Data[DRIVER]);

}

/**
 * @brief 拨盘电机PID切换
 * @param 
 */
void Shoot_PID_Switch(Shoot_Motor_t *strSho)
{
	PID_Info_t *str = &(strSho->PID);
	Motor_Info_t *strx = &(strSho->Motor_Info);
	static int length = sizeof(PID_Parameter_t);
 switch (strx->Motor_Type)
 {
	case M_2006:{
			switch (str->PID_Type)
			{
				
		  case RC:{
				memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_2006][RC],length);
				memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[M_2006][RC],length);					 
			}break;
			case KEY:{
				if(Shoot.Mode == Continous)
				{
					memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_2006][KEY],length);
					memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[M_2006][Clear_Away],length);					 
				}
				if(Shoot.Mode == Single)
				{
					memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_2006][KEY],length);
					memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[M_2006][KEY],length);
				}
			}break;
			default :{
				memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_2006][Clear_Away],length);
				memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[M_2006][Clear_Away],length);					 
			}break;
			}
		}
	 case FRIC_3508:{
		 	switch (str->PID_Type)
      {
        case Clear_Away:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[FRIC_3508][Clear_Away],length);
        }break;
         default:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[FRIC_3508][RC],length);
        }break;
			}
   }	 
	}
}

int16_t Shoot_pid_out;//拨盘can_out
/**e
 * @brief 遥控器射击
 * @param 
 */
void Shoot_RC_Ctrl(void)
{
	Shoot_Ctrl_FRIC_L(&Shoot.Motor_Data[FRIC_L],6800);
	Shoot_Ctrl_FRIC_R(&Shoot.Motor_Data[FRIC_R],-6800);
	Shoot_Ctrl_DRIVER(&Shoot.Motor_Data[DRIVER]);

		if(RC_SW == 660)
	{
		Shoot_pid_out = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[DRIVER]);
	}
	else 
	{
		Shoot_pid_out = 0;
	}
	Shoot_CanOutPut();
//	CAN_cmd_shoot_driver(Shoot_pid_out);

}

/**
 * @brief 键盘射击
 * @param 
 */
void Shoot_KEY_Ctrl(void)
{
	Shoot_KEY_F();
	Shoot_Ctrl_FRIC_L(&Shoot.Motor_Data[FRIC_L],6800);
	Shoot_Ctrl_FRIC_R(&Shoot.Motor_Data[FRIC_R],-6800);
	Shoot_Ctrl_DRIVER(&Shoot.Motor_Data[DRIVER]);
	Shoot_KEY_Fire();

	Shoot_CanOutPut();
}


void Shoot_Ctrl_FRIC_L(Shoot_Motor_t *str,int16_t speed)
{
	str->Motor_Data.PID_Speed_target = speed;
	str->Motor_Data.PID_Speed = str->Motor_Data.CAN_GetData.Motor_Speed;
}
void Shoot_Ctrl_FRIC_R(Shoot_Motor_t *str,int16_t speed)
{
	str->Motor_Data.PID_Speed_target = speed;
	str->Motor_Data.PID_Speed = str->Motor_Data.CAN_GetData.Motor_Speed;
}
void Shoot_Ctrl_DRIVER(Shoot_Motor_t *str)
{
	if(KEY_C == 1)
	{
		str->Motor_Data.PID_Speed_target = 5000;
	}
	else{
		str->Motor_Data.PID_Speed_target = -5000;
	}
	str->Motor_Data.PID_Speed = str->Motor_Data.CAN_GetData.Motor_Speed;
}

/**
 * @brief PID输出
*/
float Shoot_GetOutPut(Shoot_Motor_t *str)
{
	PID_Info_t *PID = &(str->PID);
  float res;
	  /*返回值 = 速度环PID输出*/
  res = PID_Position(&PID->Speed_Loop,  										 \
										str->Motor_Data.PID_Speed_target,				 \
										str->Motor_Data.PID_Speed);							 \
  return res;
}
/**
 * @brief Shoot电机输出
 * @param 
 */
void Shoot_CanOutPut(void)
{
	static int16_t pid_out[2] = {0,0};
	pid_out[FRIC_L] = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[FRIC_L]);
	pid_out[FRIC_R] = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[FRIC_R]);

	CAN_cmd_shoot (pid_out[FRIC_L],	pid_out[FRIC_R]);
}


/**
 * @brief 卸力函数
 * @param 
 */
void Shoot_Stop(void)
{
	static int16_t pid_out[3] = {0, 0, 0};
	Shoot_Ctrl_FRIC_L(&Shoot.Motor_Data[FRIC_L],0);
	Shoot_Ctrl_FRIC_R(&Shoot.Motor_Data[FRIC_R],0);
	/* 速度环最终输出 */
  pid_out[FRIC_L] = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[FRIC_L]);
  pid_out[FRIC_R] = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[FRIC_R]);
  pid_out[DRIVER] = 0;
	
	CAN_cmd_shoot (pid_out[FRIC_L],	pid_out[FRIC_R]);
//	CAN_cmd_shoot_driver(pid_out[DRIVER]);

}
void driver_out(void)
{

	if(RC_S2==1)
	{
		Shoot_DRIVER_CanOutPut();
	}
	if(RC_S2==3){
		memset(&Relative_Angle,0,sizeof(Relative_Angle_t));
	}
}
/**
 *	@brief	拨盘自锁
 */
void Shoot_DRIVER_CanOutPut_0(void)
{
	get_total_angle(&Shoot.Motor_Data[DRIVER].Motor_Data.CAN_GetData);
	Shoot_pid_out = PID_Position(&Shoot.Motor_Data[DRIVER].PID.Speed_Loop,
	PID_Position(&Shoot.Motor_Data[DRIVER].PID.Angle_Loop,0.0f*36,Relative_Angle.conversion_angle),
	Shoot.Motor_Data[DRIVER].Motor_Data.CAN_GetData.Motor_Speed);
	
}
/**
 *	@brief	单发模式
 */
void Shoot_DRIVER_CanOutPut(void)
{
	get_total_angle(&Shoot.Motor_Data[DRIVER].Motor_Data.CAN_GetData);
	Shoot_pid_out = PID_Position(&Shoot.Motor_Data[DRIVER].PID.Speed_Loop,
	PID_Position(&Shoot.Motor_Data[DRIVER].PID.Angle_Loop,-45.0f*36,Relative_Angle.conversion_angle),
	Shoot.Motor_Data[DRIVER].Motor_Data.CAN_GetData.Motor_Speed);
	
}

/**
 *	@brief	射击模式切换
 */
void Shoot_KEY_F(void)
{
	static uint8_t press_flag=1;
	if (press_flag==1&&KEY_F)
		{	
			HAL_Delay(20);		
			if (KEY_F)
			{	
				press_flag=0;
				if(Shoot.Mode == Continous){
				Shoot.Mode = Single;}
				else{
				Shoot.Mode = Continous;}
			}
		}
	if(press_flag == 0 && KEY_F == 0)
		{
			press_flag=1;
		}
}



void Shoot_KEY_Fire(void)
{
	if(Shoot.Mode == Continous)
	{
		if(System.Flag_State.fire_control == 0&& MOUSE_LEFT == 1)		//左键开火
		{
			Shoot_pid_out = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[DRIVER]);
		}
		else if(System.Flag_State.fire_control == 1 && System.Flag_State.fire_flag == 1 && MOUSE_LEFT == 1)		//火控
		{
			Shoot_pid_out = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[DRIVER]);
		}
		else 
		{
			Shoot_pid_out = 0;
		}
	}
	else if(Shoot.Mode ==Single)
	{
//		PID_Debug(&Shoot.Motor_Data[DRIVER].PID);
		if(MOUSE_LEFT == 1)
		{
			Shoot_DRIVER_CanOutPut();
		}
		else 
		{
			memset(&Relative_Angle,0,sizeof(Relative_Angle_t));
			Shoot_DRIVER_CanOutPut_0();
		}
	}
	
}
//void Shoot_Fire_F(void)
//{
//	switch (Shoot.Mode)
//		case Single:
//			
//	break;
//		case Continous:
//	break;

//}

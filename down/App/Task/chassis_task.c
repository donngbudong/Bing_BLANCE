/***      
《 底盘控制代码 》
***/
#include "chassis_task.h"
#include "ChassisControl_Task.h"

#include "Device.h"
#include "math.h"
#include "cmsis_os.h"


extern CAN_GET_DATA_t Gimbal_YAW;
extern float angle;//旋转角度
bool mode = 1;
/* Init start */
CHASSIS_Date_t Chassis ={
	.State=SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
	.PID_Type = RC,
};


/**
  * @brief  底盘切换任务
  */
void ChassisSwitch_Task(void const * argument)
{
	vTaskDelay(1000);
	mode_switch_chassis(MODE_STOP);
//	Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
	for(;;)
	{
		vTaskDelay(3);
		if(RC_S1 == RC_SW_UP)
		{
		//遥控器拨杆位置
			if(RC_S2 == RC_SW_MID)
			{
			//无前进速度	
				if(Chassis.move_speed == 0)
				{
				//底盘从倾倒状态恢复
					if((Chassis.ctrl_mode == MODE_STOP || Chassis.ctrl_mode == MODE_WEAK) && fabs(Chassis.Pitch)>10.0/57.3)
					{
						mode_switch_chassis(MODE_BALANCE);	
						vTaskDelay(500);
						mode_switch_chassis(MODE_NORMAL);	
						vTaskDelay(300);
					}
	//				mode_switch_chassis(MODE_STATIC);	
				}
				else
				{
					mode_switch_chassis(MODE_NORMAL);
				}			
			}
			else if(RC_S2 == RC_SW_UP){
				mode_switch_chassis(MODE_STATIC);	
			}
			else if(RC_S2 == RC_SW_DOWN){
				mode_switch_chassis(MODE_STOP);	
			}
		}
		else if(RC_S1 == RC_SW_DOWN)
		{
			if(KEY_Z == 1){
				mode_switch_chassis(MODE_BALANCE_SPEED);
				Chassis.Z_Target = 0;

			}
			if(KEY_C == 1){
				Chassis.Z_Target = 0;
				mode_switch_chassis(MODE_BALANCE);	
				vTaskDelay(500);
				mode_switch_chassis(MODE_NORMAL);	
				vTaskDelay(300);
			}
			if(KEY_X == 1){
				Chassis.Z_Target = 1000;
				mode_switch_chassis(MODE_STATIC);
				
				
			}
		}
		else if(RC_S1 == RC_SW_MID)
		{
				mode_switch_chassis(MODE_STOP);	
		}
 }
} 


/**
  * @brief  底盘模式切换
  * @param  新底盘模式
  */
void mode_switch_chassis(uint8_t mode)
{
	if(mode == Chassis.ctrl_mode)return;
	else 
	{
		Chassis.ctrl_mode_last = Chassis.ctrl_mode;
		Chassis.ctrl_mode = mode;
	}
}


/**
  * @brief  底盘静态模式
  */
void Chassis_Static(void)
{
//	Chassis.PID.PID_b2.PID_Param.I = 0;
//	Chassis.PID.PID_b2.PID_I_Out_Max = 0;

	PID_clear(&Chassis.PID.PID_b2);

	if(fabs(Chassis.speed_x)>0.20f&&fabs(Chassis.speed_x)<0.20f&&Chassis.flag_clear_pose == 0)
	{
		Chassis.target_pose_x=Chassis.pose_x;
		Chassis.flag_clear_pose = 1;
	}
	Chassis.X_Target = 0;

	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.X_Target, Chassis.speed_x);
	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);

	
	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output - Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定
	PID_Position(&Chassis.PID.Chasssis_OUT, Chassis.torque_speed + Chassis.torque_balance,0);
	
	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
	Chassis.iqControl[1] = (Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
}

/**
  * @brief  底盘平衡模式
  */
void Chassis_Balance(void)
{

	Chassis.pose_x = 0;
	Chassis.target_pose_x=0;
	Chassis.flag_clear_pose = 0;
	PID_clear(&Chassis.PID.PID_b1);
	PID_clear(&Chassis.PID.PID_b2);
	Chassis.angle_z=deadband_limit(angle_z_err_get((Gimbal_YAW.Motor_Angle/ 22.75556f)),5.0f);

	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);
	
	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output + Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定
	
	Chassis_RefereePowerControl(&PowerLimit_balance);

	PID_Position(&Chassis.PID.Chasssis_OUT,Chassis.torque_balance,0);
	

	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output );
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output );
		
	CAN_cmd_chassis(Chassis.iqControl[0],Chassis.iqControl[1]);

}



/**
  * @brief  底盘正常模式
  */
void Chassis_Normal(void)
{
//	Chassis.PID.PID_b2.PID_Param.I = 20;
//	Chassis.PID.PID_b2.PID_I_Out_Max = 80;
	if(KEY_A == 1)
	{
		Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO - 2048;
	}
	else{Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;}
	
	Chassis.target_pose_x=Chassis.pose_x;
	Chassis.flag_clear_pose = 0;
	Chassis.target_speed_x=speed_control(Chassis.move_speed,Chassis.move_direction);
	//位移、速度
	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.X_Target, Chassis.speed_x);
	Chassis.angle_z=deadband_limit(angle_z_err_get((Gimbal_YAW.Motor_Angle/ 22.75556f)),2.0f);
	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,1,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);

	Chassis.torque_speed   = (-Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定
	
	Chassis_RefereePowerControl(&PowerLimit_speed);
	Chassis_RefereePowerControl(&PowerLimit_k5);

	PID_Position(&Chassis.PID.Chasssis_OUT,Chassis.torque_speed + Chassis.torque_balance,0);
	

	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
	CAN_cmd_chassis(Chassis.iqControl[0],Chassis.iqControl[1]);

}

/**
  * @brief  底盘倒地自救模式
  */
void Chassis_Normal_Speed(void)
{

	Chassis.pose_x = 0;
	Chassis.target_pose_x=0;
	Chassis.flag_clear_pose = 0;
	PID_clear(&Chassis.PID.PID_b1);
	PID_clear(&Chassis.PID.PID_b2);
	Chassis.angle_z=deadband_limit(angle_z_err_get((Gimbal_YAW.Motor_Angle/ 22.75556f)),5.0f);
	Chassis.target_speed_x=speed_control(Chassis.move_speed,Chassis.move_direction);
	//位移、速度
	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.X_Target, Chassis.speed_x);
	/*旋转*/
	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);
	
	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output + Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定
	
	Chassis_RefereePowerControl(&PowerLimit_balance);

	PID_Position(&Chassis.PID.Chasssis_OUT,Chassis.torque_speed,0);
	

	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output );
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output );
		
	CAN_cmd_chassis(Chassis.iqControl[0],Chassis.iqControl[1]);

}

/**
  * @brief  底盘小陀螺模式
  */ 
void Chassis_Normal_s(void)
{
//	Chassis.PID.PID_b2.PID_Param.I = 0;
//	Chassis.PID.PID_b2.PID_I_Out_Max = 0;
//	Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
	PID_clear(&Chassis.PID.PID_b2);

	Chassis.target_pose_x=Chassis.pose_x;
	Chassis.flag_clear_pose = 0;
	Chassis.target_speed_x=speed_control(Chassis.move_speed,Chassis.move_direction);
	
	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.X_Target, Chassis.speed_x);
	Chassis.angle_z=deadband_limit(angle_z_min_err_get((Gimbal_YAW.Motor_Angle/ 22.75556f)),2.0f);

	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
	if (RC_S1 == RC_SW_DOWN)
	{
		if(RC_S2 == RC_SW_UP)
		{
			PID_clear(&Chassis.PID.PID_b5);
			Chassis.PID.PID_b5.PID_Output = 0;
		}
	}
	if(Chassis.Z_Target != 0)
	{
		PID_clear(&Chassis.PID.PID_b1);
		PID_clear(&Chassis.PID.PID_b2);
		PID_clear(&Chassis.PID.PID_b5);
	  float speed_temp = -Chassis.Z_Target;	
		Chassis.PID.PID_b5.PID_Output = ramp_control(Chassis.Gyo_z,speed_temp,0.25);
	}
	Chassis_RefereePowerControl(&PowerLimit_k5);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);

	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output - Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定

	PID_Position(&Chassis.PID.Chasssis_OUT, Chassis.torque_speed + Chassis.torque_balance,0);
	
	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);

	CAN_cmd_chassis(Chassis.iqControl[0],Chassis.iqControl[1]);
}



/**
  * @brief  底盘正常模式
  */
void Chassis_Normal_z(void)
{
//	Chassis.PID.PID_b2.PID_Param.I = 50;
//	Chassis.PID.PID_b2.PID_I_Out_Max = 0.1;
//	Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;

	Chassis.target_pose_x=Chassis.pose_x;
	Chassis.flag_clear_pose = 0;
//	Chassis.target_speed_x=fabs(cos(Chassis.PID.PID_b5.Err)) * speed_control(Chassis.move_speed,Chassis.move_direction);

	Chassis.angle_z=deadband_limit(angle_z_err_get(-Chassis.move_direction+(Gimbal_YAW.Motor_Angle / 1303.7973f)),0.02f);

	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.target_speed_x, Chassis.speed_x);
//	PID_Position(&Chassis.PID.PID_b2, Chassis.target_speed_x, Chassis.speed_x);

	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
//	PID_Position(&Chassis.PID.PID_b5,Chassis.GIM_Yaw,CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO);
	
	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);
//	
//	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
//	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);

	
	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output - Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve =  -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定

	PID_Position(&Chassis.PID.Chasssis_OUT, Chassis.torque_speed + Chassis.torque_balance,0);
	
	
	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
//	Chassis.iqControl[0] = (-Chassis.torque_revolve);
//	Chassis.iqControl[1] = (-Chassis.torque_revolve);
//	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output - Chassis.torque_revolve);
//	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output - Chassis.torque_revolve);

}


/**
  * @brief  重力加速度测量
  * @param  none
  * @retval 重力加速度
  */
//float accel_g_solve()
//{
//	float q0 = chassis_ctrl.INS_quat[0];
//	float q1 = chassis_ctrl.INS_quat[1];
//	float q2 = chassis_ctrl.INS_quat[2];
//	float q3 = chassis_ctrl.INS_quat[3];
//	float a0 = chassis_ctrl.INS_accel[0];
//	float a1 = chassis_ctrl.INS_accel[1];
//	float a2 = chassis_ctrl.INS_accel[2];
//	return a0 * (2 * q1 * q3 - 2 * q0 * q2) + a1 * (2 * q2 * q3 + 2 * q0 * q1) + a2 * (1 - 2 * q1 * q1 - 2 * q2 * q2);
//}

/**
* @brief 底盘PID设置总函数
* @param 
*/
void Chassis_InitPID(void)
{
	Chassis_PID_Set(&Chassis.PID);
}


/**
 * @brief PID初始化
 * @param 
 */
void Chassis_PID_Set(PID_Chassis_Info_t *str)
{
  static int length = sizeof(PID_Parameter_t);
	memcpy(&(str->PID_b1),&Param_1,length);
	memcpy(&(str->PID_b2),&Param_2,length);
	memcpy(&(str->PID_b3),&Param_3,length);
	memcpy(&(str->PID_b4),&Param_4,length);
	memcpy(&(str->PID_b5),&Param_5,length);
	memcpy(&(str->PID_b6),&Param_6,length);
	memcpy(&(str->Chasssis_OUT),&Chasssis_OUT,length);
};



/**
 * @brief 卸力函数
 * @param 
 */
void Chassis_Stop(void)
{
	static int16_t pid_out[CHAS_MOTOR_CNT] = {0,0};
	/* 速度环最终输出 */
	pid_out[MF9025_R] = 0;
	pid_out[MF9025_L] = 0;
	Chassis.iqControl[0] = pid_out[MF9025_R];
	Chassis.iqControl[1] = pid_out[MF9025_L];
	CAN_cmd_chassis(Chassis.iqControl[0],Chassis.iqControl[1]);

}


/**
 *	@brief	底盘模式切换
 */
void Chassis_KEY_C(void)
{
	static uint8_t press_flag=1;
	if (press_flag==1&&KEY_C)
		{	
			HAL_Delay(20);		
			if (KEY_C)
			{	
				press_flag=0;
				if(mode == 1){
				mode = 0;}
				else{
				mode = 1;}
			}
		}
	if(press_flag == 0 && KEY_C == 0)
		{
			press_flag=1;
		}
}

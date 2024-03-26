/***      
《 底盘控制代码 》
***/
#include "chassis_task.h"
#include "ChassisControl_Task.h"

#include "Device.h"
#include "math.h"


extern CAN_GET_DATA_t Gimbal_YAW;
extern float angle;//旋转角度

/* Init start */
CHASSIS_Date_t Chassis ={
	.State=SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
	.PID_Type = RC,
};



/**
 * @brief 底盘总控
 * @param 
 */
void Chassis_Task(void)
{
	Chassis_GET_Info();
	if(RC_S2 == 1)
	{
		Chassis_Normal_s();
		CAN_cmd_chassis(Chassis.iqControl[0],Chassis.iqControl[1]);
	}
	else if(RC_S2 == 2)
	{
		Chassis_Normal();

		CAN_cmd_chassis(Chassis.iqControl[0],Chassis.iqControl[1]);
	}
	else{
		Chassis_Stop();
	}
}




/**
  * @brief  底盘静态模式
  */
void Chassis_Static(void)
{
	Chassis.PID.PID_b2.PID_Param.I = 0;
	Chassis.PID.PID_b2.PID_I_Out_Max = 0;

	PID_clear(&Chassis.PID.PID_b2);

	if(fabs(Chassis.speed_x)>0.20f&&fabs(Chassis.speed_x)<0.20f&&Chassis.flag_clear_pose == 0)
	{
		Chassis.target_pose_x=Chassis.pose_x;
		Chassis.flag_clear_pose = 1;
	}
	Chassis.X_Target = 0;

	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.speed_x, Chassis.X_Target);
	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
//	PID_Position(&Chassis.PID.PID_b5,angle,Chassis.Yaw);
	PID_Position(&Chassis.PID.PID_b5,Chassis.GIM_Yaw,CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO);
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
	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
	PID_Position(&Chassis.PID.PID_b5,Chassis.GIM_Yaw,CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);
	
	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output + Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定
	PID_Position(&Chassis.PID.Chasssis_OUT,Chassis.torque_balance,0);
	

	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output - Chassis.torque_revolve);
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output - Chassis.torque_revolve);
//	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output);
//	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output);
////	Chassis.iqControl[0] = (-Chassis.torque_revolve);
////	Chassis.iqControl[1] = (-Chassis.torque_revolve);
}


/**
  * @brief  底盘正常模式
  */
void Chassis_Normal(void)
{
//	Chassis.PID.PID_b2.PID_Param.I = 50;
//	Chassis.PID.PID_b2.PID_I_Out_Max = 0.1;
//	Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;

	Chassis.target_pose_x=Chassis.pose_x;
	Chassis.flag_clear_pose = 0;
	Chassis.target_speed_x=speed_control(Chassis.move_speed,Chassis.move_direction);

	Chassis.angle_z=deadband_limit(angle_z_err_get(-Chassis.move_direction+(Gimbal_YAW.Motor_Angle / 1303.7973f)),0.02f);

	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.X_Target, Chassis.speed_x);
//	PID_Position(&Chassis.PID.PID_b2, Chassis.target_speed_x, Chassis.speed_x);

	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
//	PID_Position(&Chassis.PID.PID_b5,Chassis.GIM_Yaw,CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO);
	
	PID_Position(&Chassis.PID.PID_b5,Chassis.GIM_Yaw,Chassis.follow_gimbal_zero);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);
//	
//	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
//	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);

	
	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output - Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定

	PID_Position(&Chassis.PID.Chasssis_OUT, Chassis.torque_speed + Chassis.torque_balance,0);
	
	
	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
//	Chassis.iqControl[0] = (-Chassis.torque_revolve);
//	Chassis.iqControl[1] = (-Chassis.torque_revolve);
//	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output - Chassis.torque_revolve);
//	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output - Chassis.torque_revolve);

}


/**
  * @brief  底盘小陀螺模式
  */
void Chassis_Normal_s(void)
{
//	Chassis.PID.PID_b2.PID_Param.I = 50;
//	Chassis.PID.PID_b2.PID_I_Out_Max = 0.1;
//	Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;

	Chassis.target_pose_x=Chassis.pose_x;
	Chassis.flag_clear_pose = 0;
	Chassis.target_speed_x=speed_control(Chassis.move_speed,Chassis.move_direction);
	
	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
	PID_Position(&Chassis.PID.PID_b2, Chassis.X_Target, Chassis.speed_x);
//	PID_Position(&Chassis.PID.PID_b2, Chassis.target_speed_x, Chassis.speed_x);

	/*平衡*/
	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	/*旋转*/
	PID_Position(&Chassis.PID.PID_b5,0,40);
	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);
//	
//	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
//	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);

	
	Chassis.torque_speed   = (-Chassis.PID.PID_b1.PID_Output - Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output - Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定

	PID_Position(&Chassis.PID.Chasssis_OUT, Chassis.torque_speed + Chassis.torque_balance,0);
	
	
	Chassis.iqControl[0] = (-Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
	Chassis.iqControl[1] = ( Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve);
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
	Chassis.target_speed_x=fabs(cos(Chassis.PID.PID_b5.Err)) * speed_control(Chassis.move_speed,Chassis.move_direction);

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
//void Chassis_Balance(void)
//{
//	
//	Chassis.pose_x =0; 
//	PID_Position(&Chassis.PID.PID_b1, Chassis.pose_x, Chassis.target_pose_x);
//	PID_Position(&Chassis.PID.PID_b2, Chassis.speed_x, Chassis.X_Target);
//	/*平衡*/
//	PID_Position(&Chassis.PID.PID_b3,0.5,Chassis.Pitch);
//	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
//	/*旋转*/
//	PID_Position(&Chassis.PID.PID_b5,angle,Chassis.Yaw);
//	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);

//	Chassis.torque_balance = (Chassis.PID.PID_b3.PID_Output+Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
//	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定
//	PID_Position(&Chassis.PID.Chasssis_OUT,Chassis.torque_balance,0);
//	Chassis.iqControl[0] = -Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve;
//	Chassis.iqControl[1] =  Chassis.PID.Chasssis_OUT.PID_Output + Chassis.torque_revolve;
////	Chassis.iqControl[0] = Chassis.torque_revolve;
////	Chassis.iqControl[1] = Chassis.torque_revolve;
////	Chassis.iqControl[0] = -Chassis.PID.Chasssis_OUT.PID_Output;
////	Chassis.iqControl[1] =  Chassis.PID.Chasssis_OUT.PID_Output;
//}
// void Chassis_Normal(void)
// {
// //	Chassis.PID.PID_b2.PID_Param.I = 0.01f;//0.01f;
// //	Chassis.PID.PID_b2.PID_Output_Max = 25.0f;//25.0f;
// 	Chassis.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
// 	Chassis.target_pose_x=Chassis.pose_x;
// //	Chassis.flag_clear_pose = 0;
// 	Chassis.target_speed_x=speed_control(Chassis.move_speed,Chassis.move_direction);
// 	Chassis.angle_z=deadband_limit(angle_z_err_get(-Chassis.move_direction+(Gimbal_YAW.Motor_Angle / 1303.7973f)),0.02f);


// 	PID_Position(&Chassis.PID.PID_b1, Chassis.target_pose_x, Chassis.pose_x);
// 	PID_Position(&Chassis.PID.PID_b2, Chassis.target_speed_x, Chassis.speed_x);
// 	//Blance
// 	PID_Position(&Chassis.PID.PID_b3,0,Chassis.Pitch);
// 	PID_Position(&Chassis.PID.PID_b4,0,Chassis.Gyo_y);
	
// 	/*旋转*/
// 	PID_Position(&Chassis.PID.PID_b5,0,Chassis.angle_z);
// 	PID_Position(&Chassis.PID.PID_b6,Chassis.PID.PID_b5.PID_Output,Chassis.Gyo_z);
	
	
// 	Chassis.torque_speed 	 = (Chassis.PID.PID_b1.PID_Output + Chassis.PID.PID_b2.PID_Output) * Chassis.torque_const;
// 	Chassis.torque_balance = (-Chassis.PID.PID_b3.PID_Output -	Chassis.PID.PID_b4.PID_Output) * Chassis.torque_const;
// 	Chassis.torque_revolve = -Chassis.PID.PID_b6.PID_Output * Chassis.torque_const;				//航行锁定
	
		
// 	PID_Position(&Chassis.PID.Chasssis_OUT, Chassis.torque_speed + Chassis.torque_balance, 0);

// //	PID_Position(&Chassis.PID.Chasssis_OUT, 0, Chassis.torque_speed);

// 	Chassis.iqControl[0] = (Chassis.PID.Chasssis_OUT.PID_Output );
// 	Chassis.iqControl[1] = (-Chassis.PID.Chasssis_OUT.PID_Output );
	
	
// //	Chassis.iqControl[0] = -Chassis.PID.Chasssis_OUT.PID_Output;
// //	Chassis.iqControl[1] =  Chassis.PID.Chasssis_OUT.PID_Output;
// }

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
  * @brief  正方向切换
  * @param  none
  * @retval 正方向
  */
	
	
	
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
	CAN_cmd_chassis(pid_out[MF9025_R],pid_out[MF9025_L]);	  
}



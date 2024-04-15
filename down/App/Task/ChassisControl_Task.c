#include "ChassisControl_Task.h"
#include "chassis_task.h"

#include "math.h"




extern CHASSIS_Date_t Chassis;
/**
* @brief 底盘信息获取
* @param 
*/
void Chassis_GET_Info(void)
{
	Chassis.torque_const = 1;		//转矩系数
	Chassis.State	= System.System_State;

	Chassis.Pitch 	= IMU_Get_Data.IMU_Eular[0];
	Chassis.Yaw 	= IMU_Get_Data.IMU_Eular[2];

	Chassis.Gyo_y	= IMU_Get_Data.IMU_Gyo[1]/10;//除10
	Chassis.Gyo_z = IMU_Get_Data.IMU_Gyo[2]/10;
	Chassis.speed_x = -(Chassis.Motor_Date[0].Speed - Chassis.Motor_Date[1].Speed) /57.29578f*Diameter_weel/2.0f/2.0f;
	Chassis.omega_z =  (Chassis.Motor_Date[0].Speed + Chassis.Motor_Date[1].Speed) /57.29578f*Diameter_weel/2.0f/2.0f;
	Chassis.pose_x += 	Chassis.speed_x * 0.01f;
	if(RC_S1 == 1)
	{
		Chassis_RC_Ctrl();
		
	}else if(RC_S1 == 2)
	{
		Chassis_KEY_Ctrl();
	}
	

	Chassis.move_speed = sqrt(Chassis.X_Target*Chassis.X_Target + Chassis.Y_Target*Chassis.Y_Target);
	if(Chassis.chassis_direction == CHASSIS_BACK)Chassis.move_speed *= -1;
	Chassis.move_direction = atan2f(Chassis.Y_Target,Chassis.X_Target);

	Chassis.GIM_Yaw_t = Gimbal_YAW.Motor_Angle*360/8192;
	Chassis.GIM_Yaw = YAW_Angle_Over_Zero(&Chassis.GIM_Yaw_t);
}



void XY_Speed(float *speed, float MAX,float MIX)
{
	if(*speed > MAX){
		*speed = MAX;
	}
	if(*speed < MIX){ 
	*speed = MIX;
	}
}
/**
  * @brief  底盘遥控控制
  * @param  none
  * @retval none
  */
void Chassis_RC_Ctrl(void)
{
	
	Chassis.X_Target = 	RC_CH3 * 8.0 / 660 ;
	Chassis.Y_Target =	RC_CH2 * 2.5 / 660;
	Chassis.Z_Target =  RC_CH0 * 2.5 / 660;
}
/**
  * @brief  底盘键鼠控制
  * @param  none
  * @retval none
  */
void Chassis_KEY_Ctrl(void)
{
	/*速度控制*/
	if(KEY_W == 1)
	{
		Chassis.X_Target += 0.03f;
	}else if(KEY_S == 1)
	{
		Chassis.X_Target -=0.03f;
	}else
	{
		if(Chassis.X_Target > 0)
		{
			Chassis.X_Target-=0.01f;
		}
		else if(Chassis.X_Target < 0)
		{
			Chassis.X_Target+=0.01f;
			
		}
		else if(Chassis.X_Target == 0){
			Chassis.X_Target = 0;
		}
	}
	
//	if(REF.Power_Heat_Data.chassis_power == 45
//	|| REF.Power_Heat_Data.chassis_power == 50
//	|| REF.Power_Heat_Data.chassis_power == 50
//	|| REF.Power_Heat_Data.chassis_power == 60
//	)
//	{	
//		XY_Speed(&Chassis.X_Target,3,-3);
//	}
//	else if(REF.Power_Heat_Data.chassis_power >= 80){

//		XY_Speed(&Chassis.X_Target,3.5,-3.5);
//	}
	XY_Speed(&Chassis.X_Target,4.5,-4.5);
	/*四面切换*/
	
//252
	if(KEY_A == 1)
	{
		Chassis.follow_gimbal_zero = -19;
	}
	else if(KEY_D == 1)
	{
		Chassis.follow_gimbal_zero = 161;
	}
	else{
		Chassis.follow_gimbal_zero = 70;}
	
	
	
}
/**
  * @brief  底盘状态清除
  * @param  none
  * @retval none
  */
void chassis_state_clear()
{
	Chassis.pose_x = 0;
	Chassis.target_pose_x=0;
	Chassis.target_speed_x=0;
	PID_clear(&Chassis.PID.PID_b1);
	PID_clear(&Chassis.PID.PID_b2);
	PID_clear(&Chassis.PID.PID_b3);
	PID_clear(&Chassis.PID.PID_b4);
	PID_clear(&Chassis.PID.PID_b5);
	PID_clear(&Chassis.PID.PID_b6);
	PID_clear(&Chassis.PID.Chasssis_OUT);
}


/**
  * @brief  速度控制
  * @param  输入速度,运动方向
  * @retval 输出速度
  */
float speed_control(float speed_in,float direction)
{
	float accel_K1;
	float accel_K2;
	if(fabs(direction)>1.39&&fabs(direction)<1.75)
	{
		accel_K1 = 0.2f;
		accel_K2 = 0.6f;
	}
	else 
	{
		accel_K1 = 1.0f;
		accel_K2 = 1.0f;
	}
	if (speed_in >= 0)
	{
		float accel_1 = accel_K1*(speed_in - Chassis.speed_x);
		float accel_2 = accel_K2*(speed_in - Chassis.speed_x);
		if (Chassis.speed_x >= 0)
		{
			return Chassis.speed_x + accel_1;
		}
		else if (Chassis.speed_x < 0)
			return Chassis.speed_x + accel_2;
	}
	if (speed_in < 0)
	{
		float accel_1 = -accel_K1*(speed_in - Chassis.speed_x);
		float accel_2 = -accel_K2*(speed_in - Chassis.speed_x);
		if (Chassis.speed_x < 0)
		{
			return Chassis.speed_x - accel_1;
		}
		else if (Chassis.speed_x >= 0)
			return Chassis.speed_x - accel_2;
	}
	
}


/**
  * @brief  速度灵敏度调节
  * @retval 速度灵敏度
  */


//float speed_sen_adjust()
//{
//	float speed_sen;
//	if(Game_Robot_State.chassis_power_limit == 0)
//	{
//		speed_sen = SPEED_UNLIMIT/660.0f;
//	}
//	else if(Game_Robot_State.chassis_power_limit <= 60)
//	{
//		speed_sen = SPEED_LIMIT_60/660.0f;
//	}
//	else if(Game_Robot_State.chassis_power_limit <= 80)
//	{
//		speed_sen = SPEED_LIMIT_80/660.0f;
//	}
//	else if(Game_Robot_State.chassis_power_limit <= 100)
//	{
//		speed_sen = SPEED_LIMIT_100/660.0f;
//	}
//	else if(Game_Robot_State.chassis_power_limit <= 65535)
//	{
//		speed_sen = SPEED_UNLIMIT/660.0f;
//	}
//		
//	return speed_sen;
//}


	
/**
  * @brief  底盘方向偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_err_get(float target_ang)
{
	float AngErr_front,AngErr_back;
	AngErr_front=limit_pi(Chassis.follow_gimbal_zero / 1303.7973f - target_ang);
	AngErr_back	=limit_pi((Chassis.follow_gimbal_zero + 4096) / 1303.7973f - target_ang);
	if(fabs(AngErr_front)>fabs(AngErr_back))
	{
		Chassis.chassis_direction = CHASSIS_BACK;
		return AngErr_back;
	}
	else 
	{
		Chassis.chassis_direction = CHASSIS_FRONT;
		return AngErr_front;
	}
}

/**
  * @brief  底盘方向最小偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_min_err_get(float target_ang)
{
	float AngErr_front,AngErr_back,AngErr_left,AngErr_right;
	AngErr_front=limit_pi(Chassis.follow_gimbal_zero / 1303.7973f - target_ang);
	AngErr_back	=limit_pi((Chassis.follow_gimbal_zero + 4096) / 1303.7973f - target_ang);
	AngErr_left =limit_pi((Chassis.follow_gimbal_zero + 2048) / 1303.7973f - target_ang);
	AngErr_right=limit_pi((Chassis.follow_gimbal_zero - 2048) / 1303.7973f - target_ang);
	float min_err = AngErr_front;
	if(fabs(min_err)>fabs(AngErr_back))min_err=AngErr_back;
	if(fabs(min_err)>fabs(AngErr_left))min_err=AngErr_left;
	if(fabs(min_err)>fabs(AngErr_right))min_err=AngErr_right;
	return min_err;
}

/**
  * @brief  四面对敌
  * @param  目标方向
  * @retval 方向偏差
  */
//void angle_z()
//{
//	float AngErr_front,AngErr_back,AngErr_left,AngErr_right;
//	
//	
//}
/**
  * @brief  梯形控制
  * @param  反馈；设定；加速度
  * @retval 输出值
  */
float ramp_control(float ref ,float set,float accel)
{
	float ramp = limit(accel,0,1)*(set - ref);
	return ref + ramp;
}

/**
  * @brief  死区限制
  * @param  输入信号;死区
  * @retval 输出信号
  */
float deadband_limit(float rc_in , float deadband )
{
	if(fabs(rc_in)<deadband)return 0;
	return rc_in;
}

/**
  * @brief  转角限制到±PI
  * @param  输入转角
  * @retval 输出转角
  */
float limit_pi(float in)
{
	while(in < -PI || in > PI)
	{
		if (in < -PI)
			in = in + PI + PI;
		if (in > PI)
			in = in - PI - PI;
	}
	return in;
}

/**
  * @brief  数据范围限制
  * @param  输入数据,最小值,最大值
  * @retval 输出数据
  */
float limit(float data, float min, float max)
{
	if (data >= max)
		return max;
	if (data <= min)
		return min;
	return data;
}

/**
 * @brief YAW轴角度过零处理
 * @param 
 */
float YAW_Angle_Over_Zero(float *Angle)
{
	if(*Angle < CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO - 180)    
	{
		*Angle += 360;
	}
	else if(*Angle > CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO + 180)
	{
		*Angle -= 360;
	}
	return *Angle;
}

/**
 * @brief 底盘YAW轴跟随处理
 * @param 
 */
float YAW_MotorAngle_Proc(int16_t Angle)
{
	if(Angle < CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO - 4096)
		Angle += 8192;
	if(Angle > CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO + 4096)
		Angle -= 8192;
	
  return (float)Angle;
}

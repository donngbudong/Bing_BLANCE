#include "ChassisControl_Task.h"
#include "chassis_task.h"
#include "math.h"
#include "cmsis_os.h"

extern CHASSIS_Date_t Chassis;
//定义功率限制对象
chassis_power_t PowerLimit_speed =		{	.control_obj = &Chassis.torque_speed,
																				.warning_buffer = 30.0f,
																				.no_limit 	= 4000.0f,
																				.base_limit	= 4000.0f,
																				.add_limit 	= 1000.0f,	};

chassis_power_t PowerLimit_balance =	{	.control_obj = &Chassis.torque_balance,
																				.warning_buffer = 30.0f,
																				.no_limit 	= 5000.0f,
																				.base_limit	= 3000.0f,
																				.add_limit 	= 2000.0f,	};

chassis_power_t PowerLimit_k5 =				{	.control_obj = &Chassis.PID.PID_b5.PID_Output,
																				.warning_buffer = 30.0f,
																				.no_limit 	= 1000.0f,
																				.base_limit	= 2000.0f,
																				.add_limit 	= 30.0f,	};

/**
  * @brief  底盘控制任务
  */
void ChassisControl_Task(void const * argument)
{
	vTaskDelay(2000);
	for(;;)
	{
		Chassis_GET_Info();
		
		if(Chassis.ctrl_mode == MODE_STATIC)
		{
			Chassis_Normal_s();
		}
		if(Chassis.ctrl_mode == MODE_NORMAL)
		{
			Chassis_Normal();
		}
		if(Chassis.ctrl_mode == MODE_STOP)	
		{
			Chassis_Stop();
		}
		if(Chassis.ctrl_mode == MODE_BALANCE)	
		{
			Chassis_Balance();
		}
		if(Chassis.ctrl_mode == MODE_BALANCE_SPEED)	
		{
			Chassis_Normal_Speed();
		}
		vTaskDelay(1);
		
	}
}

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
	Chassis.pose_x += 	Chassis.speed_x * 0.001f;
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
	if(Chassis.chassis_direction == CHASSIS_BACK)
	{
		Chassis.X_Target =  RC_CH3 * 8 / 660 ;
	}
	
	if(Chassis.chassis_direction == CHASSIS_FRONT)
	{
		Chassis.X_Target = 	-RC_CH3 * 8 / 660 ;
	}
		Chassis.Y_Target = RC_CH2 * 15 / 660 ;
		Chassis.Z_Target = RC_CH0 * 1000 / 660;

}
/**
  * @brief  底盘键鼠控制
  * @param  none
  * @retval none
  */
int16_t vx_set;
void Chassis_KEY_Ctrl(void)
{
	vx_set = limit(vx_set,-660,660);
	
		if(Chassis.chassis_direction == CHASSIS_BACK)
	{
		if(KEY_W == 1){
		vx_set ++;
		Chassis.X_Target = ramp_control(Chassis.speed_x, vx_set * speed_sen_adjust(),0.001);
	}else if(KEY_S == 1){
		vx_set --;
		Chassis.X_Target = ramp_control(Chassis.speed_x, vx_set * speed_sen_adjust(),0.001);
	}else{
		vx_set=0;
		Chassis.X_Target = ramp_control(Chassis.speed_x,vx_set,1);;
	}

	}
	
	if(Chassis.chassis_direction == CHASSIS_FRONT)
	{
		if(KEY_W == 1){
		vx_set --;
		Chassis.X_Target = ramp_control(Chassis.speed_x, vx_set * speed_sen_adjust(),0.001);
	}else if(KEY_S == 1){
		vx_set ++;
		Chassis.X_Target = ramp_control(Chassis.speed_x, vx_set * speed_sen_adjust(),0.001);
	}else{
		vx_set=0;
		Chassis.X_Target = ramp_control(Chassis.speed_x,vx_set,1);;
	}	}
	
	
	
	/*速度控制*/
//	if(KEY_W == 1){
//		vx_set ++;
//		Chassis.X_Target = ramp_control(Chassis.speed_x, vx_set * speed_sen_adjust(),0.001);
//	}else if(KEY_S == 1){
//		vx_set --;
//		Chassis.X_Target = ramp_control(Chassis.speed_x, vx_set * speed_sen_adjust(),0.001);
//	}else{
//		vx_set=0;
//		Chassis.X_Target = ramp_control(Chassis.speed_x,vx_set,1);;
//	}
//	if(Chassis.chassis_direction == CHASSIS_BACK)
//	{
		
//		if(KEY_W == 1){
//			Chassis.X_Target = speed_sen_adjust();
//			}else if(KEY_S == 1){
//			Chassis.X_Target = -speed_sen_adjust();
//			}else{
//		Chassis.X_Target = ramp_control(Chassis.speed_x,0,0.4);;
//		}
//	}
//	
//	if(Chassis.chassis_direction == CHASSIS_FRONT)
//	{
//		if(KEY_W == 1){
//			Chassis.X_Target = -speed_sen_adjust();
//		}else if(KEY_S == 1){
//			Chassis.X_Target = speed_sen_adjust();
//		}else{
//			Chassis.X_Target = ramp_control(Chassis.speed_x,0,0.6);;
//		}	
//	}
	
	/*四面切换*/
////252
//	if(KEY_A == 1)
//	{
//		Chassis.follow_gimbal_zero = -19;
//	}
//	else if(KEY_D == 1)
//	{
//		Chassis.follow_gimbal_zero = 161;
//	}
//	else{
//		Chassis.follow_gimbal_zero = 70;}
	
//			Chassis.Z_Target = RC_CH0 * 1000 / 660;

	
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
float speed_sen_adjust(void)
{
	float speed_sen;
	if(REF.Robot_Status.chassis_power_limit == 0)
	{
		speed_sen = SPEED_UNLIMIT;
	}
	else if(REF.Robot_Status.chassis_power_limit <= 60)
	{
		speed_sen = SPEED_LIMIT_60;
	}
	else if(REF.Robot_Status.chassis_power_limit <= 80)
	{
		speed_sen = SPEED_LIMIT_80;
	}
	else if(REF.Robot_Status.chassis_power_limit <= 100)
	{
		speed_sen = SPEED_LIMIT_100;
	}
	else if(REF.Robot_Status.chassis_power_limit <= 65535)
	{
		speed_sen = SPEED_UNLIMIT/660.0f;
	}
		
	return speed_sen;
}
/**
  * @brief  裁判系统功率限制
  * @param  调节对象指针
  */
void Chassis_RefereePowerControl(chassis_power_t *power_obj)
{
	float total_current_limit = 0.0f;
	float total_current       = 0.0f;
	float power_scale         = 0.0f;
	float current_scale       = 0.0f;
			
	if(REF.Robot_Status.chassis_power_limit == 0 || REF.Robot_Status.chassis_power_limit == 65535)
	{
		total_current_limit = power_obj -> no_limit;
	}
  else
	{
		if(REF.Power_Heat_Data.buffer_energy < power_obj -> warning_buffer)
		{
			if(REF.Power_Heat_Data.buffer_energy > 5.0f)
			{
					power_scale = REF.Power_Heat_Data.buffer_energy / power_obj -> warning_buffer;
			}
			else
			{
					power_scale = 5.0f / power_obj -> warning_buffer;
			}
			total_current_limit = power_obj -> base_limit * power_scale;
		}
		else
		{
			if(REF.Power_Heat_Data.chassis_power > REF.Robot_Status.chassis_power_limit * 0.75f)
			{
				if(REF.Power_Heat_Data.chassis_power < REF.Robot_Status.chassis_power_limit)
				{
					power_scale = (REF.Robot_Status.chassis_power_limit - REF.Power_Heat_Data.chassis_power) / 
					(REF.Robot_Status.chassis_power_limit - REF.Robot_Status.chassis_power_limit*0.75f);	
				}
				else
				{
					power_scale = 0.0f;
				}
				
				total_current_limit = power_obj -> base_limit + power_obj -> add_limit * power_scale;
			}
			else
			{
				total_current_limit = power_obj -> base_limit + power_obj -> add_limit;
			}
		}
	}
	
	total_current += fabs(*(power_obj -> control_obj));

  if(total_current > total_current_limit)
  {
		current_scale = total_current_limit / total_current;
		*(power_obj -> control_obj) *= current_scale;
  }
}

	
/**
  * @brief  底盘方向偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_err_get(float target_ang)
{
	float AngErr_front,AngErr_back;
	AngErr_front=limit_pi(Chassis.follow_gimbal_zero / 22.75556f - target_ang);
	AngErr_back	=limit_pi((Chassis.follow_gimbal_zero + 4096) / 22.75556f - target_ang);
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
	AngErr_front=limit_pi(Chassis.follow_gimbal_zero / 22.75556f - target_ang);
	AngErr_back	=limit_pi((Chassis.follow_gimbal_zero + 4096) / 22.75556f - target_ang);
	AngErr_left =limit_pi((Chassis.follow_gimbal_zero + 2048) / 22.75556f - target_ang);
	AngErr_right=limit_pi((Chassis.follow_gimbal_zero - 2048) / 22.75556f - target_ang);
	float min_err = AngErr_front;
	if(fabs(min_err)>fabs(AngErr_back))min_err=AngErr_back;
	if(fabs(min_err)>fabs(AngErr_left))min_err=AngErr_left;
	if(fabs(min_err)>fabs(AngErr_right))min_err=AngErr_right;
	return min_err;
}


/**
  * @brief  梯形控制
  * @param  反馈；设定；加速度
  * @retval 输出值
  */
float ramp_control(float ref ,float set,float accel)
{
	float ramp = limit(accel,0,5)*(set - ref);
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
	while(in < -IMU_180 || in > IMU_180)
	{
		if (in < -IMU_180)
			in = in + IMU_180 + IMU_180;
		if (in > IMU_180)
			in = in - IMU_180 - IMU_180;
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

/**
 * @brief YAW轴角度过零处理
 * @param 
 */
float YAW_Angle_Over_Zero_1(float *Angle)
{
	if(*Angle - Chassis.GIM_Yaw_t > IMU_180)    
	{
		*Angle =*Angle - IMU_360;        
	}
	else if(*Angle - Chassis.GIM_Yaw_t < -IMU_180)
	{
		*Angle =*Angle + IMU_360;
	}
	return *Angle;
}

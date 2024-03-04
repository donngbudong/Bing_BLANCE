#ifndef __CHASSISCONTROL_TASK_H
#define __CHASSISCONTROL_TASK_H


//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif



/**
  * @brief  死区限制
  * @param  输入信号;死区
  * @retval 输出信号
  */
float deadband_limit(float rc_in , float deadband );

/**
  * @brief  转角限制到±PI
  * @param  输入转角
  * @retval 输出转角
  */
float limit_pi(float in);

/**
  * @brief  数据范围限制
  * @param  输入数据,最小值,最大值
  * @retval 输出数据
  */
float limit(float data, float min, float max);

/**
  * @brief  底盘状态更新
  */
void Chassis_GET_Info(void);

/**
  * @brief  底盘状态清除
  * @param  none
  * @retval none
  */
void chassis_state_clear(void);
	
/**
  * @brief  速度控制
  * @param  输入速度,运动方向
  * @retval 输出速度
  */
float speed_control(float speed_in,float direction);

/**
  * @brief  速度灵敏度调节
  * @retval 速度灵敏度
  */
//float speed_sen_adjust();

/**
  * @brief  底盘方向偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_err_get(float target_ang);

/**
  * @brief  底盘方向最小偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_min_err_get(float target_ang);

/**
  * @brief  梯形控制
  * @param  反馈；设定；加速度
  * @retval 输出值
  */
float ramp_control(float ref ,float set,float accel);

float YAW_Angle_Over_Zero(float *Angle);

#endif

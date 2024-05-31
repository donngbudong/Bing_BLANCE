#ifndef __CHASSISCONTROL_TASK_H
#define __CHASSISCONTROL_TASK_H

#include "stm32f4xx_hal.h"

//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif

//轮子直径
#define Diameter_weel 			0.190f
//定速控制
#define SPEED_LIMIT_60  		10.0f
#define SPEED_LIMIT_80  		15.0f
#define SPEED_LIMIT_100 		20.0f
#define SPEED_UNLIMIT 	 		6600.0f


//功率限制结构体
typedef struct
{
	float* control_obj;								//控制对象
	float no_limit;										//无限制允许输出
	float base_limit;									//基础限制允许输出
	float add_limit;									//增益限制允许输出
	float warning_buffer;							//缓冲能量预警值
	
}chassis_power_t;

extern chassis_power_t PowerLimit_speed;
extern chassis_power_t PowerLimit_balance;
extern chassis_power_t PowerLimit_k5;

/**
  * @brief  裁判系统功率限制
  * @param  调节对象指针
  */
void Chassis_RefereePowerControl(chassis_power_t *power_obj);
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
float speed_sen_adjust(void);

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
float YAW_MotorAngle_Proc(int16_t Angle);

void Chassis_RC_Ctrl(void);
void Chassis_KEY_Ctrl(void);

#endif

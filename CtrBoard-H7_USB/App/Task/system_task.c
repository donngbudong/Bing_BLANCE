#include "system_task.h"
//#include "tim.h"
#include "System.h"
#include "Device.h"
#include "cmsis_os.h"

__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}
//获取系统时间，单位us
uint32_t micros(void)
{
  return getCurrentMicros();
}






Sys_Info_t System = {
    .Rc_State  = RC_LOST,
    .Imu_State = IMU_LOST,
    .System_State = SYSTEM_LOST,
		.System_Pid = Clear_Away,

};

uint64_t Remote_time = 0;//遥控器
uint64_t Imu_time = 0;//裁判系统

//设备连接初始化
void Time_Init(void)
{
  Remote_time = micros();
  Imu_time = micros();
}


void System_Task(void)
{
	System_State();
	RC_State_Report();
	IMU_State_Report();
	PID_Switch();
	shoot_pc_R();
}


void PID_Switch(void)
{
	if(RC_S1==1)
	{
		System.System_Pid = RC;
	}
	if(RC_S1==3)
	{
		System.System_Pid = Clear_Away;
	}
	if(RC_S1==2)
	{
		System.System_Pid = KEY;
	}
}

uint16_t xy123=1000;
void System_State(void)
{
	CAN_cmd_RC1(RC_CH0,RC_CH1,RC_CH2,RC_CH3);
	CAN_cmd_RC2(RC_S1,RC_S2,RC_SW,RC_Ctrl.kb.key);
    if(System.Rc_State ==RC_ERR || System.Imu_State ==IMU_ERR)
  {
    System.System_State = SYSTEM_ERR;
  } 
    if(System.Rc_State ==RC_NORMAL && System.Imu_State ==IMU_NORMAL)
  {
    System.System_State = SYSTEM_NORMAL;
  }
  if(System.Rc_State ==RC_LOST || System.Imu_State ==IMU_LOST)
  {
    System.System_State = SYSTEM_LOST;
  }
	
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1800); //相当于一个周期内（20ms）有0.5ms高脉冲
	
}


/**弹舱开关**/
//780关//1800开

uint16_t magazine=1;//开关弹舱
void shoot_pc_R(void)
{
	static uint8_t helm=1;
	if (helm==1&&KEY_R)
		{	
			HAL_Delay(20);		
			if (KEY_R)
			{	
				helm=0;
				if(magazine==0)
				{magazine=1;}
				else	
				{magazine=0;}
			}
		}
		if(helm==0 && RC_Ctrl.kb.bit.R==0)
		{
			helm=1;
		}
//			if(magazine==0)
//		{
//			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,750);
//		}
//		else
//		{
//			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1800);
//		}	
}

#include "system_task.h"
#include "tim.h"
#include "System.h"
#include "Device.h"

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


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
		HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_1);
}
 


uint16_t static RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);
    
    for (uint16_t i = 0;i < 8;i++)
    {
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
    }
}
void ws2812_blue(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x00, 0x22, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(num_data));
}
uint8_t i=100;
uint8_t x,y,z;
void State_LED(void)
{
	 // X
	 if (x == 0)
	 {
		ws2812_set_RGB(i, 0x00, 0x00, 0);
		ws2812_set_RGB(i, 0x00, 0x00, 1);
	 }
	 else
	 {
		ws2812_set_RGB(0x00, i, 0x00, 0);
		ws2812_set_RGB(0x00, i, 0x00, 1);
	 }
	 // Y
	 if (y == 0)
	 {
		ws2812_set_RGB(0x00, i, 0x00, 2);
		ws2812_set_RGB(0x00, i, 0x00, 3);
	 }
	 else
	 {
		ws2812_set_RGB(0x00, 0x00, i, 2);
		ws2812_set_RGB(0x00, 0x00, i, 3);
	 }
	 // Z
	 if (z == 0)
	 {
		ws2812_set_RGB(0x00, 0x00, i, 4);
		ws2812_set_RGB(0x00, 0x00, i, 5);
	 }
	 else
	 {
		ws2812_set_RGB(i, 0x00, 0x00, 4);
		ws2812_set_RGB(i, 0x00, 0x00, 5);
	 }

//    ws2812_set_RGB(0x00, 0x22, 0x00, 1);
//    ws2812_set_RGB(0x00, 0x00, 0x22, 2);
//    ws2812_set_RGB(0x22, 0x00, 0x00, 3);
//    ws2812_set_RGB(0x00, 0x22, 0x00, 4);
//    ws2812_set_RGB(0x00, 0x00, 0x22, 5);
//		ws2812_set_RGB(0x22, 0x00, 0x00, 6);
//    ws2812_set_RGB(0x00, 0x22, 0x00, 7);
    HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(232)); //272 = 80 + 24 * LED_NUMS(6)
//    HAL_Delay(50);
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
			if(magazine==0)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,750);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1800);
		}	
}

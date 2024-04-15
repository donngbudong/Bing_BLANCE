#ifndef __SYSTEM_TASK_H
#define	__SYSTEM_TASK_H


#include "stdint.h"
#define ONE_PULSE        (142)                           //1 
#define ZERO_PULSE       (67)                           //0 
#define RESET_PULSE      (80)                           //80 
#define LED_NUMS         (6)                            //led 
#define LED_DATA_LEN     (24)                           //led 
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812




void Time_Init(void);
void System_Task(void);
void System_State(void);
void PID_Switch(void);




void shoot_pc_R(void);


#endif

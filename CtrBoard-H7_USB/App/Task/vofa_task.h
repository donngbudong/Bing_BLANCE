#ifndef _VOFA_TASK_H
#define _VOFA_TASK_H
#include "stm32h7xx_hal.h"





/*ÁªºÏÌå*/
typedef union 
{
	float		receive;
	uint8_t	send[4];
}Vofa_Date; 




#endif


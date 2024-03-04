#include "vofa_task.h"
#include "usart.h"
#include "can_receive.h"

static uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};




void Motor_Date(CAN_GET_DATA_t *str)
{
	
}


void VOFA_SendDate(void)
{
	
	
	/*֡帧头*/
 	for(uint8_t i; i<4; i++){
		HAL_UART_Transmit(&huart6,&tail[i],1,100);
	}
//	HAL_UART_Transmit(&huart6,&tail[0],1,100);
//	HAL_UART_Transmit(&huart6,&tail[1],1,100);
//	HAL_UART_Transmit(&huart6,&tail[2],1,100);
//	HAL_UART_Transmit(&huart6,&tail[3],1,100);
}

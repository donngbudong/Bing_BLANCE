/************************************** Includes **************************************/
#include "driver.h"
/* Private typedef -----------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	Çý¶¯³õÊ¼»¯
 */
void Driver_Init(void)
{
	FDCAN1_Config();
	FDCAN2_Config();

	USART_Init();
}

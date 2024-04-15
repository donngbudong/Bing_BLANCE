#include "drv_usart.h"
#include "Device.h"
#include "usart.h"

/******** ���ڽ������鶨�� ********/
uint8_t usart5_buf[USART5_BUFLEN];
uint8_t IMU_Buffer  [128];



/**
  * @brief          UART��ʼ��
  * @param[in]      none
  * @retval         none
  */
void USART_Init(void)
{
	IMU_Init();//
	uart_receive_init(&huart5);
	__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);//IMU
	HAL_UART_Receive_DMA(&huart7,IMU_Buffer,128);
}





void UART_IRQHandler_IT(UART_HandleTypeDef *huart)
{
 if(huart==&huart7)//Imu
	{
		 if(__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE)!=RESET)   //�ж��Ƿ��ǿ����ж�
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart7);                    		 //��������жϱ�־�������һֱ���Ͻ����жϣ�
			HAL_UART_DMAStop(&huart7);
			//imu���ݽ���
			for(size_t i=0;i<128;i++){
				Packet_Decode(IMU_Buffer[i]);
			}
			HAL_UART_Receive_DMA(&huart7,IMU_Buffer,128);	
			memset(IMU_Buffer, 0, 128);
			Imu_time=micros() + 30000;
		}
	}
	if(huart == &huart5) 
	{
		uart_receive_handler(&huart5);
	}
}

/******** ���ڿ����жϴ����� ********/
void usart5_callback_handler(uint8_t *buff)
{
	sbus_to_rc(&RC_Ctrl, buff);
}
/**
  * @brief      ���ص�ǰDMAͨ����ʣ������ݸ���
  * @param[in]  dma_stream: DMAͨ��
  * @retval     DMAͨ����ʣ������ݸ���
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  return ((uint16_t)(dma_stream->NDTR));
}

/**
  * @brief	�ڽ��յ�һ֡����֮�����һ֡����ʱ��֮��������
	*					���������˻ص�����,�˺�������������жϱ�־λ
  * @param	huart: UART���ָ��
  * @retval
  */
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5)
  {
//    //�ж������Ƿ�Ϊ�����ĳ��� �粻���򲻽���ص����� ֱ�ӿ�����һ�ν���
//    if ((USART5_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == USART5_BUFLEN)
//    {
      /* ��������жϻص����� */
      usart5_callback_handler(usart5_buf);
//    }

    /* ����DMA�������ݵĳ��� */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, USART5_MAX_LEN);
  }

}

/**
  * @brief	�����ڷ����жϵ�ʱ����˺���
  * @param	huart: UART���ָ��
  * @retval	��stm32f4xx_it.c�����
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
  /* __HAL_UART_GET_FLAG	���ָ����UART���б�־λ�Ƿ񴥷� */
  /* __HAL_UART_GET_IT_SOURCEG	���ָ����UART�����ж��Ƿ񴥷� */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
      __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
  {
    /* ��������жϱ�־λ */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* �ص�DMA */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* ��������жϴ����� */
    uart_rx_idle_callback(huart);

    /* ����DMA���� */
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
}

/**
  * @brief      ����ʹ��DMA����(�������жϽ���)
  * @param[in]  huart: UART���ָ��
  * @param[in]  pData: receive buff
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
  uint32_t tmp = 0;
  tmp = huart->RxState;

  /* �жϴ����Ƿ��Ѿ���ʼ����� */
  if (tmp == HAL_UART_STATE_READY)
  {
    /* ����û�����������Ƿ���ȷ */
    if ((pData == NULL) || (Size == 0))
      return HAL_ERROR;

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* ʹ��DMAͨ�� */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)pData, Size);

    /* ����DMA���� ��UART CR3 �Ĵ����е� DMARλ �ø� */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    return HAL_OK;
  }
  else
    return HAL_BUSY;
}

/**
  * @brief	�����жϳ�ʼ������
  * @param	huart:UART���ָ��
  * @retval	none
  */
void uart_receive_init(UART_HandleTypeDef *huart)
{
  if (huart == &huart5)
  {
    /* ��������жϱ�־λ */
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    /* �������ڿ����ж� */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    /* ����DMA���� ָ�����ճ��Ⱥ����ݵ�ַ */
    uart_receive_dma_no_it(&huart5, usart5_buf, USART5_MAX_LEN);
  }
}


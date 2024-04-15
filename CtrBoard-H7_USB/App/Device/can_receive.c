#include "can_receive.h"
#include "Device.h"
//#include "chassis_task.h"


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

void CAN_Date_Decode(CAN_GET_DATA_t *str, uint8_t *Data,uint32_t data_len)                                    
{                         
	str->Motor_Angle = (uint16_t)((Data)[0] << 8 | (Data)[1]);            
	str->Motor_Speed = (uint16_t)((Data)[2] << 8 | (Data)[3]);      
	str->Motor_ELC   = (int16_t)((Data)[4] << 8 | (Data)[5]);  
	str->Motor_Temp  = (Data)[6];                                   
}




void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//����ǰ���������	
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			
			switch(RxHeader1.Identifier)
			{//�������IDΪ0
        case GIM_PIT_ID :
					CAN_Date_Decode(&Gimbal.PITCH.Motor_Data.CAN_GetData, g_Can1RxData,RxHeader1.DataLength);
				break;
				
        case FRIC_L_ID :
					CAN_Date_Decode(&Shoot.Motor_Data[FRIC_L].Motor_Data.CAN_GetData, g_Can1RxData,RxHeader1.DataLength);
				break;			
				
        case FRIC_R_ID :
					CAN_Date_Decode(&Shoot.Motor_Data[FRIC_R].Motor_Data.CAN_GetData, g_Can1RxData,RxHeader1.DataLength);
				break;				
				
        case DRIVER_ID :
					CAN_Date_Decode(&Shoot.Motor_Data[DRIVER].Motor_Data.CAN_GetData, g_Can1RxData,RxHeader1.DataLength);
				break;				
				
				default: break;
			}			
	  }
  }
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN2)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			switch(RxHeader2.Identifier)
			{ //�������IDΪ0
        case GIM_YAW_ID :
					CAN_Date_Decode(&Gimbal.YAW.Motor_Data.CAN_GetData, g_Can2RxData,RxHeader2.DataLength);
				break;
				
				default: break;
			}	
    }
  }
}
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
  if(len<=8)	
	{
	  TxHeader.DataLength = len<<16;     // ���ͳ��ȣ�8byte
	}
	else  if(len==12)	
	{
	   TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else  if(len==16)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	
	}
  else  if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}		
	else  if(len==24)	
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}else  if(len==48)
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}else  if(len==64)
   {
		 TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	 }
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//�������л��رգ�
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CAN2.0
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0x00;//��Ϣ���

	 
	 HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
//   // ����CANָ��
//  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
//  {
//       // ����ʧ�ܴ���
//       Error_Handler();      
//  }
//	 return 0;
}


/**
 * @brief CAN2_YAW�������
*/
void CAN_cmd_gimbal_yaw(int16_t yaw)
{
	uint8_t data[2];
	uint16_t id = GIMBAL_ALL_ID;
	
	data[0] = yaw >> 8;
	data[1] = yaw;
	canx_send_data(&hfdcan2, id, data, 2);
}




/**
 * @brief CAN1_PITCH_DRIVER����
*/

void CAN_cmd_gimbal_pitch(int16_t pitch,int16_t driver)
{
	uint8_t data[8];
	uint16_t id = DRIVER_ALL_ID;
	data[0] = 0;
	data[1] = 0;	
	data[2] = pitch >> 8;
	data[3] = pitch;
	data[4] = driver >> 8;
	data[5] = driver;
	data[6] = 0;
	data[7] = 0;
	canx_send_data(&hfdcan1, id, data, 8);
}

/**
 * @brief CAN2_SHOOT����
*/
void CAN_cmd_shoot( int16_t shoot1,int16_t shoot2)
{
	uint8_t data[8];
	uint16_t id = FRIC_ALL_ID;
	
	data[0] = shoot1 >> 8;
	data[1] = shoot1;
	data[2] = shoot2 >> 8;
	data[3] = shoot2;
	
	canx_send_data(&hfdcan1, id, data, 8);

}


/**
* @brief ң��������
* @param 
*/
void CAN_cmd_RC1(int16_t ch0,int16_t ch1,int16_t ch2,int16_t ch3)
{
	uint8_t data[8];
	uint16_t id = 0x567;
	
	data[0] = ch0 >> 8;
	data[1] = ch0;
	data[2] = ch1 >> 8;
	data[3] = ch1;
	data[4] = ch2 >> 8;
	data[5] = ch2;
	data[6] = ch3 >> 8;
	data[7] = ch3;
	
	canx_send_data(&hfdcan2, id, data, 8);
}

/**
* @brief ң��������
* @param 
*/
void CAN_cmd_RC2(uint8_t s1,uint8_t s2,int16_t sw,uint16_t key)
{
	uint8_t data[8];
	uint16_t id = 0x520;
	
	data[0] = s1;
	data[1] = s2;
	data[2] = sw >> 8;
	data[3] = sw;
	data[4] = key >> 8;
	data[5] = key;
	
	canx_send_data(&hfdcan2, id, data, 8);

}
/**
 *	@brief	����ϵ�Ƕ�=0��֮���������������2006�������Կ�����Ϊ0������ԽǶȡ�������ת��ת���ĽǶ����м��ٱ���Ŀ��Ƕ�Ҫ�˼��ٱȣ������á�
 */
Relative_Angle_t Relative_Angle;
void get_total_angle(CAN_GET_DATA_t *motor_get)
{
	Relative_Angle.angle=(float)motor_get->Motor_Angle;
	float res1, res2, delta;
	if(Relative_Angle.angle < Relative_Angle.lastAngle){			        //���ܵ����
		res1 = Relative_Angle.angle + 8192.0f - Relative_Angle.lastAngle;	//��ת  
		res2 = Relative_Angle.angle - Relative_Angle.lastAngle;				  //��ת	
	}else{	
		res1 = Relative_Angle.angle - 8192.0f -Relative_Angle.lastAngle ; //��ת	
		res2 = Relative_Angle.angle - Relative_Angle.lastAngle;				  //��ת	
	}
	
	/*��������ת���϶���ת�ĽǶ�С���Ǹ������*/ 
	//->ÿ����8000ת��ÿ����133ת��ÿ����0.133ת
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;
	Relative_Angle.totalAngle += delta;
	Relative_Angle.conversion_angle=Relative_Angle.totalAngle/8191.0f*360.0f;
	Relative_Angle.lastAngle = Relative_Angle.angle;
}

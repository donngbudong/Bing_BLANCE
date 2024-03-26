#include "can_receive.h"
#include "Device.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

static CAN_TxHeaderTypeDef  down_uptx_message;
static uint8_t              capacitance_can_send_data[8];

static CAN_TxHeaderTypeDef  capacitance_tx_message;
static uint8_t              capacitance_can_send_data[8];

CAN_GET_DATA_t Gimbal_YAW;

void CAN_MF9025_Decode(CAN_MF9025_DATE_T *str,uint8_t *Date)
{
	str->Last_Encoder = str->Encoder;
	str->Cmd						= (Date)[0];
	str->Temp					  = (Date)[1];
	str->Torque_Current = (int16_t)((Date)[3] << 8 | (Date)[2]);
	str->Speed 					= (int16_t)((Date)[5] << 8 | (Date)[4]);
	str->Encoder				= ((uint16_t)(Date)[7]<< 8 | (Date)[6]);
}

void CAN_Date_Decode(CAN_GET_DATA_t *str, uint8_t *Data)                                    
{                         
	str->Motor_Angle = (uint16_t)((Data)[0] << 8 | (Data)[1]);            
	str->Motor_Speed = (uint16_t)((Data)[2] << 8 | (Data)[3]);      
	str->Motor_ELC   = (uint16_t)((Data)[4] << 8 | (Data)[5]);  
	str->Motor_Temp  = (Data)[6];                                   
}


super_capacitor_t super_capacitor;
int16_t s1;
int16_t s2;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	static uint8_t rx_data[8];
	if(hcan->Instance==CAN1)
	{
	
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_MF9025_1_ID:
						CAN_MF9025_Decode(&Chassis.Motor_Date[MF9025_R], rx_data);                                    
					break;
        case CAN_MF9025_2_ID:
						CAN_MF9025_Decode(&Chassis.Motor_Date[MF9025_L], rx_data);                                    
					break;
				case 0x666:
					s1 = (int16_t)(rx_data[0]<<8 | rx_data[1]);
					s2 = (int16_t)(rx_data[2]<<8 | rx_data[3]);

				default: {break;}	
		}
	}
	if(hcan->Instance==CAN2)
	{
	
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
			case GIM_YAW_ID:
				CAN_Date_Decode(&Gimbal_YAW, rx_data); 
				break;
			case 0x567:
				RC_Ctrl.rc.ch0 = ((int16_t)(rx_data[0] << 8	| rx_data[1]));
				RC_Ctrl.rc.ch1 = ((int16_t)(rx_data[2] << 8 | rx_data[3]));
				RC_Ctrl.rc.ch2 = ((int16_t)(rx_data[4] << 8 | rx_data[5]));
				RC_Ctrl.rc.ch3 = ((int16_t)(rx_data[6] << 8 | rx_data[7]));
				break;
			case 0x520:
				RC_Ctrl.rc.s1 = ((uint8_t)(rx_data[0]));
				RC_Ctrl.rc.s2 = ((uint8_t)(rx_data[1]));
				RC_Ctrl.rc.sw = ((int16_t)(rx_data[2] << 8 	| rx_data[3]));
				RC_Ctrl.kb.key = ((int16_t)(rx_data[4] << 8 | rx_data[5]));
				break;
			default: {break;}	

		}
  }
}



void CAN_cmd_capacitance(int16_t size)
{
	uint32_t send_mail_box;
	capacitance_tx_message.StdId = 0x333;
	capacitance_tx_message.IDE = CAN_ID_STD;
	capacitance_tx_message.RTR = CAN_RTR_DATA;
	capacitance_tx_message.DLC = 0x08;
	
	capacitance_can_send_data[0]=size>>8;
	capacitance_can_send_data[1]=size;
	
	HAL_CAN_AddTxMessage(&hcan1, &capacitance_tx_message, capacitance_can_send_data, &send_mail_box);
//		/*找到空的发送邮箱，把数据发送出去*/
//	if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//	if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
//	{
//		HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
//	}
//	}
}
/**
* @brief 底盘电机发送
* @param 
*/
void CAN_cmd_chassis(int16_t Right, int16_t Left)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_MF9025_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	
	chassis_can_send_data[0] = Right;
	chassis_can_send_data[1] = Right >> 8;
	chassis_can_send_data[2] = Left;
	chassis_can_send_data[3] = Left >>8;
//	chassis_can_send_data[4] = motor3 >> 8;
//	chassis_can_send_data[5] = motor3;
//	chassis_can_send_data[6] = motor4 >> 8;
//	chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


int Cacapacitance_Rong(void)
{
	  return super_capacitor.rong;
}




///**
//* @brief 遥控器发送
//* @param 
//*/
//void CAN_cmd_RC1(int16_t ch0,int16_t ch1,int16_t ch2,int16_t ch3)
//{
//    rc_tx_message.StdId = 0x567;
//    rc_tx_message.IDE = CAN_ID_STD;
//    rc_tx_message.RTR = CAN_RTR_DATA;
//    rc_tx_message.DLC = 0x08;
//    
//    rc_can_send_data[0] = ch0 >> 8;
//    rc_can_send_data[1] = ch0;
//		rc_can_send_data[2] = ch1 >> 8;
//		rc_can_send_data[3] = ch1;
//		rc_can_send_data[4] = ch2 >> 8;
//		rc_can_send_data[5] = ch2;
//		rc_can_send_data[6] = ch3 >> 8;
//		rc_can_send_data[7] = ch3;

//	//找到空的发送邮箱，把数据发送出去
//	if(HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//	if(HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
//	{
//		HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
//	}
//	}
//}

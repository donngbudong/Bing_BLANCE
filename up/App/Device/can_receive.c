#include "can_receive.h"
#include "Device.h"
//#include "chassis_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];

static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

static CAN_TxHeaderTypeDef  rc_tx_message;
static uint8_t              rc_can_send_data[8];

void CAN_Date_Decode(CAN_GET_DATA_t *str, uint8_t *Data)                                    
{                         
	str->Motor_Angle = (uint16_t)((Data)[0] << 8 | (Data)[1]);            
	str->Motor_Speed = (uint16_t)((Data)[2] << 8 | (Data)[3]);      
	str->Motor_ELC   = (int16_t)((Data)[4] << 8 | (Data)[5]);  
	str->Motor_Temp  = (Data)[6];                                   
}




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		if(hcan->Instance==CAN1)
		{
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
			{
				case GIM_PIT_ID:
					CAN_Date_Decode(&Gimbal.PITCH.Motor_Data.CAN_GetData, rx_data);                                    
				break;

				case FRIC_L_ID:
					CAN_Date_Decode(&Shoot.Motor_Data[FRIC_L].Motor_Data.CAN_GetData, rx_data);
				break;

				case FRIC_R_ID:
					CAN_Date_Decode(&Shoot.Motor_Data[FRIC_R].Motor_Data.CAN_GetData, rx_data);
				break;
				case DRIVER_ID:
					CAN_Date_Decode(&Shoot.Motor_Data[DRIVER].Motor_Data.CAN_GetData, rx_data);
				break;
				default: {break;}	


			}
		}
		if(hcan->Instance==CAN2)
		{
			HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
			{
				case GIM_YAW_ID:
					CAN_Date_Decode(&Gimbal.YAW.Motor_Data.CAN_GetData, rx_data); 
				break;
				case 0x666:
				REF.Robot_Status.shooter_barrel_heat_limit = ((int16_t)(rx_data[0] << 8	| rx_data[1])); 
				REF.Power_Heat_Data.shooter_17mm_1_barrel_heat = ((int16_t)(rx_data[2] << 8 | rx_data[3]));
				break;
				default: {break;}	

			}
		}
}
/**
 * @brief CAN2_YAW电机发送
*/
void CAN_cmd_gimbal_yaw(int16_t yaw)
{
    gimbal_tx_message.StdId = GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    
    gimbal_can_send_data[0] = yaw >> 8;
    gimbal_can_send_data[1] = yaw;

	
	//找到空的发送邮箱，把数据发送出去
	if(HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message,gimbal_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
	if(HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message,gimbal_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
	{
		HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message,gimbal_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
	}
	}
}


/**
 * @brief CAN1_SHOOT发送
*/
void CAN_cmd_shoot( int16_t shoot1,int16_t shoot2)
{
    shoot_tx_message.StdId = FRIC_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    
   	shoot_can_send_data[0] = shoot1 >> 8;
    shoot_can_send_data[1] = shoot1;
		shoot_can_send_data[2] = shoot2 >> 8;
    shoot_can_send_data[3] = shoot2;
	//找到空的发送邮箱，把数据发送出去
		if(HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message,shoot_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
		{
		if(HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message,shoot_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
		{
			HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message,shoot_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
		}
		}
}

/**
 * @brief CAN1_PITCH发送
*/
void CAN_cmd_gimbal_pitch(int16_t pitch,int16_t driver)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = DRIVER_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    
    gimbal_can_send_data[2] = pitch >> 8;
    gimbal_can_send_data[3] = pitch;
		gimbal_can_send_data[4] = driver >> 8;
		gimbal_can_send_data[5] = driver;

    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


/**
* @brief 遥控器发送
* @param 
*/
void CAN_cmd_RC1(int16_t ch0,int16_t ch1,int16_t ch2,int16_t ch3)
{
    rc_tx_message.StdId = 0x567;
    rc_tx_message.IDE = CAN_ID_STD;
    rc_tx_message.RTR = CAN_RTR_DATA;
    rc_tx_message.DLC = 0x08;
    
    rc_can_send_data[0] = ch0 >> 8;
    rc_can_send_data[1] = ch0;
		rc_can_send_data[2] = ch1 >> 8;
		rc_can_send_data[3] = ch1;
		rc_can_send_data[4] = ch2 >> 8;
		rc_can_send_data[5] = ch2;
		rc_can_send_data[6] = ch3 >> 8;
		rc_can_send_data[7] = ch3;

	//找到空的发送邮箱，把数据发送出去
	if(HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
	if(HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
	{
		HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
	}
	}
}
/**
* @brief 遥控器发送
* @param 
*/
void CAN_cmd_RC2(uint8_t s1,uint8_t s2,int16_t sw,uint16_t key)
{
    rc_tx_message.StdId = 0x520;
    rc_tx_message.IDE = CAN_ID_STD;
    rc_tx_message.RTR = CAN_RTR_DATA;
    rc_tx_message.DLC = 0x06;
    
    rc_can_send_data[0] = s1;
    rc_can_send_data[1] = s2;
		rc_can_send_data[2] = sw >> 8;
		rc_can_send_data[3] = sw;
		rc_can_send_data[4] = key >> 8;
		rc_can_send_data[5] = key;
	if(HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
	if(HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
	{
		HAL_CAN_AddTxMessage(&hcan2, &rc_tx_message,rc_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
	}
	}
}
/**
 *	@brief	电机上电角度=0，之后用这个函数更新2006电机的相对开机后（为0）的相对角度。即计算转子转过的角度若有减速比则目标角度要乘减速比，否则不用。
 */
Relative_Angle_t Relative_Angle;
void get_total_angle(CAN_GET_DATA_t *motor_get)
{
	Relative_Angle.angle=(float)motor_get->Motor_Angle;
	float res1, res2, delta;
	if(Relative_Angle.angle < Relative_Angle.lastAngle){			        //可能的情况
		res1 = Relative_Angle.angle + 8192.0f - Relative_Angle.lastAngle;	//正转  
		res2 = Relative_Angle.angle - Relative_Angle.lastAngle;				  //反转	
	}else{	
		res1 = Relative_Angle.angle - 8192.0f -Relative_Angle.lastAngle ; //反转	
		res2 = Relative_Angle.angle - Relative_Angle.lastAngle;				  //正转	
	}
	
	/*不管正反转，肯定是转的角度小的那个是真的*/ 
	//->每分钟8000转，每秒钟133转，每毫秒0.133转
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;
	Relative_Angle.totalAngle += delta;
	Relative_Angle.conversion_angle=Relative_Angle.totalAngle/8191.0f*360.0f;
	Relative_Angle.lastAngle = Relative_Angle.angle;
}

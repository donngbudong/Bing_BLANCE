#include "referee.h"

// 裁判系统信息
Referee_info_t 	REF ;

//bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用

/**
  * @brief  读取裁判数据
  * @param  缓存数据
  */
void Referee_Decode(uint8_t *referee_buf )
{
	uint16_t judge_length;//统计一帧数据长度 
	uint16_t cmd_id = 0;//数据命令码解析	

	memcpy(&REF.FrameHeader,referee_buf,LEN_HEADER);   //储存帧头数据
	if(referee_buf[SOF] == JUDGE_FRAME_HEADER)                   //判断帧头是否为0xa5
	{
		if(Verify_CRC8_Check_Sum( referee_buf, LEN_HEADER ) == TRUE)  //帧头CRC8校验
		{
			judge_length = referee_buf[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;	//统计一帧数据长度,用于CR16校验
			
			if(Verify_CRC16_Check_Sum(referee_buf,judge_length) == TRUE)//帧尾CRC16校验
			{
				cmd_id = (referee_buf[6] << 8 | referee_buf[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				switch(cmd_id)
				{
					case ID_game_status:     //0x0001
							memcpy(&REF.Game_Status, (referee_buf + DATA), LEN_game_status);
							break;
					
					case ID_game_result:    //0x0002
							memcpy(&REF.Game_Result, (referee_buf + DATA), LEN_game_result);
							break;
					
					case ID_game_robot_HP:    //0x0003
							memcpy(&REF.Game_Robot_HP,(referee_buf + DATA), LEN_game_robot_HP);
							break;
			
					case ID_event_data:    //0x0101
							memcpy(&REF.Event_Data, (referee_buf + DATA), LEN_event_data);
							break;
					
					case ID_supply_projectile_action:    //0x0102
							memcpy(&REF.Supply_Projectile_Action, (referee_buf + DATA), LEN_supply_projectile_action);
							break;
					
					case ID_referee_warning:    //0x0104
							memcpy(&REF.Referee_Warning, (referee_buf + DATA), LEN_referee_warning);
							break;
					
					case ID_dart_remaining_time:    //0x0105
							memcpy(&REF.Dart_Remaining_Time, (referee_buf + DATA), LEN_dart_remaining_time);
							break;
					
					case ID_robot_status:    //0x0201
               				Determine_ID();
							memcpy(&REF.Robot_Status, (referee_buf + DATA), LEN_robot_status);
							break;
				
					case ID_power_heat_data:    //0x0202
							memcpy(&REF.Power_Heat_Data, (referee_buf + DATA), LEN_power_heat_data);
							break;
					
					case ID_robot_pos:    //0x0203
							memcpy(&REF.Robot_Pos, (referee_buf + DATA), LEN_robot_pos);
							break;
					
					case ID_buff:    //0x0204
							memcpy(&REF.Buff, (referee_buf + DATA), LEN_buff);
							break;
					
					case ID_air_support_data:    //0x0205
							memcpy(&REF.Air_Support_date, (referee_buf + DATA), LEN_air_support_data);
							break;
					
					case ID_hurt_data:      			//0x0206
							memcpy(&REF.Hurt_Date, (referee_buf + DATA), LEN_hurt_data);
							break;
					case ID_shoot_data:      			//0x0207
							memcpy(&REF.Shoot_Data, (referee_buf + DATA), LEN_shoot_data);
					     	break;	
					
					case ID_projectile_allowance:    //0x0208
							memcpy(&REF.Projectile_allowance,(referee_buf + DATA), LEN_projectile_allowance);
							break;
          
					case ID_rfid_status: //0x0209
               				memcpy(&REF.Rfid_Status, (referee_buf+DATA), LEN_rfid_status);
               				break;
	
				}
					
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(referee_buf + sizeof(xFrameHeader) + LEN_CMDID + REF.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Referee_Decode(referee_buf + sizeof(xFrameHeader) + LEN_CMDID + REF.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}


/***************************裁判系统串口初始化***********************************/

//extern uint32_t Refer_time ;//裁判系统


/**
  * @brief  串口一次发送一个字节数据
  * @param  自己打包好的要发给裁判的数据
  * @retval void
  * @attention  串口移位发送
  */
//void UART5_SendChar(uint8_t cData)
//{
//	while (USART_GetFlagStatus( UART5, USART_FLAG_TC ) == RESET);
//	
//	USART_SendData( UART5, cData );   
//}

///**
// * @brief 判断裁判系统是否连接
// * @param 
// */
//bool Judge_IF_Refer_Normal(void)
//{
//  bool res = true;
//  if(micros() >= Refer_time)
//  {
//    res = false;
//  }
//  return res;
//}

//void Judge_IF_REF_ONL(void)
//{
//  REF.IF_REF_ONL = Judge_IF_Refer_Normal();
//}


////------------PowerHeatData------------//
//float REF_Report_CHAS_Power(void)   //4
//{
//  return REF.PowerHeatData.chassis_power;
//}

//uint16_t REF_Report_CHAS_PowerBuffer(void)   //2
//{
//  return REF.PowerHeatData.chassis_power_buffer;
//}  

//uint16_t REF_Report_Shooter_Heat(void)    //2
//{
//  return REF.PowerHeatData.shooter_heat0;
//}
////------------ShootInfo----------------//
//float REF_Report_RealShootSpeed(void)
//{
//  return REF.ShootData.bullet_speed;
//}
//uint8_t REF_Report_IF_shooter_output(void)
//{
//  return REF.GameRobotStat.mains_power_shooter_output;
//}

////------------GameRobotStat------------//
//uint8_t REF_Report_robot_ID(void)   //1
//{
//  return REF.GameRobotStat.robot_id;
//}

//uint8_t REF_Report_Shoot_SpeedLimit(void)  //1
//{
//  return REF.GameRobotStat.shooter1_17mm_speed_limit;
//}

//uint16_t REF_Report_Shoot_CoolingLimit(void)  //2
//{
//  return REF.GameRobotStat.shooter1_17mm_cooling_limit;
//}

//uint16_t REF_Report_Shoot_CoolingRate(void)  //2
//{
//  return REF.GameRobotStat.shooter1_17mm_cooling_rate;
//}

//uint16_t REF_Report_CHAS_MaxPower(void)  //2
//{
//  return REF.GameRobotStat.max_chassis_power;
//}
////------------------RFID-------------------------//
//uint8_t REF_Report_RFID_State(void)
//{
//  uint8_t res = 0;
//  res = res | REF.rfid_status.rfid_status;
//  return res;
////  return (uint8_t)REF.rfid_status.rfid_status;
//}


////------------------------------------------------------------------------

void Determine_ID(void)//判断自己是哪个队伍
{
	if(REF.Robot_Status.robot_id < 10)//本机器人的ID，红方
	{ 
		REF.ids.teammate_hero 		 	= 1;
		REF.ids.teammate_engineer  	= 2;
		REF.ids.teammate_infantry3  = 3;
		REF.ids.teammate_infantry4  = 4;
		REF.ids.teammate_infantry5  = 5;
		REF.ids.teammate_plane		 	= 6;
		REF.ids.teammate_sentry		  = 7;
		
		REF.ids.client_hero 		 	= 0x0101;
		REF.ids.client_engineer  = 0x0102;
		REF.ids.client_infantry3 = 0x0103;
		REF.ids.client_infantry4 = 0x0104;
		REF.ids.client_infantry5 = 0x0105;
		REF.ids.client_plane			= 0x0106;
		
		if     (REF.Robot_Status.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			REF.self_client = REF.ids.client_hero;
		else if(REF.Robot_Status.robot_id == engineer_red)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.Robot_Status.robot_id == infantry3_red)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.Robot_Status.robot_id == infantry4_red)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.Robot_Status.robot_id == infantry5_red)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.Robot_Status.robot_id == plane_red)
			REF.self_client = REF.ids.client_plane;
	}
	else //蓝方
	{
		REF.ids.teammate_hero 		 = 101;
		REF.ids.teammate_engineer  = 102;
		REF.ids.teammate_infantry3 = 103;
		REF.ids.teammate_infantry4 = 104;
		REF.ids.teammate_infantry5 = 105;
		REF.ids.teammate_plane		 = 106;
		REF.ids.teammate_sentry		 = 107;
		
		REF.ids.client_hero 		 	 = 0x0165;
		REF.ids.client_engineer    = 0x0166;
		REF.ids.client_infantry3	 = 0x0167;
		REF.ids.client_infantry4 	 =0x0168;
		REF.ids.client_infantry5 	 = 0x0169;
		REF.ids.client_plane			 = 0x016A;
		
		if     (REF.Robot_Status.robot_id == hero_blue)
			REF.self_client = REF.ids.client_hero;
		else if(REF.Robot_Status.robot_id == engineer_blue)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.Robot_Status.robot_id == infantry3_blue)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.Robot_Status.robot_id == infantry4_blue)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.Robot_Status.robot_id == infantry5_blue)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.Robot_Status.robot_id == plane_blue)
			REF.self_client = REF.ids.client_plane;
		
	}
}


/***      
《 视觉接收代码 》
***/
#include "vision_task.h"
#include "Device.h"
#include "usart.h"
extern DMA_HandleTypeDef hdma_uart4_tx;

//extern uint8_t visual_mode;								//视觉模式（普通|哨兵）
//extern uint8_t Visual_Date_t;							//视觉是否识别

//Visual_data From_Vision_Data;
//Visual_data Visual_Data;
Vision_Date_t Vision = {
	.Vs_State = VS_LOST,
	.ratio = 0.6,
	.MAX =  10,
	.MIN = -10,
};



void Visual_Task(void)
{
//	VSIION_State_Report();//离线判断
	Visual_SendData();
}
//static uint8_t exe[7] = {0x80,0x01,0x02,0x02,0x03,0x04,0x7f}; 
void Visual_SendData(void)
{
	Vision.TX_Pitch.output = IMU_Get_Data.IMU_Eular[PITCH];
	Vision.TX_Yaw.output = IMU_Get_Data.IMU_Eular[YAW];
	uint8_t exe[13] = {0x80,1,30,
							Vision.TX_Pitch.input[0],Vision.TX_Pitch.input[1],Vision.TX_Pitch.input[2],Vision.TX_Pitch.input[3],
							Vision.TX_Yaw.input[0],Vision.TX_Yaw.input[1],Vision.TX_Yaw.input[2],Vision.TX_Yaw.input[3],
							112,0x7f}; 
	for(uint8_t i = 0; i<13; i++)
	{
		HAL_UART_Transmit(&huart4,&exe[i],1,100);
	}
}


void Vision_read_data(uint8_t *data)
{
	 if(*data==0x70 && (*(data+12))==0x6F)
	 {		//yaw
			for(uint8_t i=0;i<4;i++){
				Vision.yaw.input[i]=*((data+1)+i);
			}//pitch
			 for(int i=0;i<4;i++){
				Vision.pitch.input[i]=*((data+5)+i);
			}
			Vision.distance[0]=*(data+9);
			Vision.distance[1]=*(data+10);
			Vision.distance[2]=Vision.distance[0]*100+Vision.distance[1];
			Vision.STATE = *(data+11);
			Vision.RX_Pitch = Vision.pitch.output ;
			Vision.RX_Yaw	  = Vision.yaw.output ;
	 }
	 //YAW
		 if(Vision.RX_Yaw > Vision.MAX)
		{
			Vision.RX_Yaw = Vision.MAX;
		}
		else if(Vision.RX_Yaw < Vision.MIN)
		{
			Vision.RX_Yaw = Vision.MIN;
		}
	 //PITCH
		 if(Vision.RX_Pitch > Vision.MAX)
		{
			Vision.RX_Pitch = Vision.MAX;
		}
		else if(Vision.RX_Pitch < Vision.MIN)
		{
			Vision.RX_Pitch = Vision.MIN;
		}
}


//void VSIION_State_Report(void)
//{
//	Vision.Vs_State = VS_Check();
//}

//Vs_State_t VS_Check(void)
//{
//	static Vs_State_t res;
//	res = VS_NORMAL;
//	if(IF_VS_LOST)
//		res = VS_LOST;
//	return res;
//}
/**
 * @brief vision离线判断
 * @param 
 */
//bool Judge_VISISON_Lost(void)
//{
//  static bool res ;
//  	res = false;
//  if(micros() <= Vision_time)
//    res = true;
//  return res;
//}

//void visual_date(uint8_t *data)
//{
//	 if(*data==0x70 && (*(data+12))==0x6F)
// {
//	 visual_rc visual_pitch;
//	 visual_rc visual_yaw;

//	 for(int i=0;i<4;i++)
//	 {
//		 visual_yaw.date_t[i]=*(data+1+i);
//	 }
//	 
//	 	 for(int i=0;i<4;i++)
//	 {
//		 visual_pitch.date_t[i]=*(data+5+i);
//	 }

//	From_Vision_Data.distance[0]=*(data+9);
//	From_Vision_Data.distance[1]=*(data+10);
//	From_Vision_Data.distance[2]=From_Vision_Data.distance[0]*100+From_Vision_Data.distance[1];
//	From_Vision_Data.Visual_State=*(data+11);//状态  默认0		1自启成功无数据		2有数据
//	From_Vision_Data.Vision_Pitch = visual_pitch.date*Vision_Coff;
//	From_Vision_Data.Vision_Yaw  =visual_yaw.date*Vision_Coff; 

////Yaw
//		 if(From_Vision_Data.Vision_Yaw<-10)
//		{
//			Visual_Data.Vision_Yaw=-10;
//		}
//		else if(From_Vision_Data.Vision_Yaw>10)
//				{
//			Visual_Data.Vision_Pitch=10;
//		}
//		else
//		{
//			Visual_Data.Vision_Yaw=From_Vision_Data.Vision_Yaw;
//		}
////Pitch
//		if(From_Vision_Data.Vision_Pitch<-10)
//		{
//			Visual_Data.Vision_Pitch=-10;
//		}
//		else if(From_Vision_Data.Vision_Pitch>10)
//				{
//			Visual_Data.Vision_Pitch=10;
//		}
//		else
//		{
//			Visual_Data.Vision_Pitch=From_Vision_Data.Vision_Pitch;
//		}
//		
// }
//}

//visual_send send[3];
//void visual_send_date(void)
//{
//	uint8_t state;
//	float speed;
//	uint16_t i;
//	send[0].date=Eular[0];
//	send[1].date=Eular[2];
//	if(REF.GameRobotStat.robot_id < 10){
//		state=BLUE;//0//BLUR
//	}
//	else{
//		state=RED;//1//RED
//	}
//		
//	speed=(uint8_t)(REF_FOOT_Shoot_Speed()*10);//实时弹速
//	uint8_t exe[13] = {0x80,state,speed,
//								send[0].date_t[0],send[0].date_t[1],send[0].date_t[2],send[0].date_t[3],
//								send[1].date_t[0],send[1].date_t[1],send[1].date_t[2],send[1].date_t[3],
//										visual_mode,0x7f}; 
////百位 1正常   2 哨兵   十位 1 普通   2 预测    个位 1  15m/s    2  30m/s 
//	for(i = 0; i<13; i++)
//	{
//		HAL_UART_Transmit(&huart4,&exe[i],1,100);
//	}
//}

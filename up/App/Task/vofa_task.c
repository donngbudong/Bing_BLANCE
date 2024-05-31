#include "vofa_task.h"
#include "usart.h"
#include "can_receive.h"
#include "Device.h"

static uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};




Vofa_Date yaw_actual;
Vofa_Date yaw_target;


Vofa_Date pitch_actual;
Vofa_Date pitch_target;

void vofa_SendDate(void)
{
	uint16_t i=0;

	yaw_actual.receive = Gimbal.YAW.Motor_Data.PID_Angle;
	yaw_target.receive = Gimbal.YAW.Motor_Data.PID_Angle_target;
	pitch_actual.receive = Gimbal.PITCH.Motor_Data.PID_Angle;
	pitch_target.receive = Gimbal.PITCH.Motor_Data.PID_Angle_target;
	
	for(i = 0; i<4; i++)	
	{
		HAL_UART_Transmit(&huart4,&yaw_actual.send[i],1,100);
	}
	for(i = 0; i<4; i++)	
	{
		HAL_UART_Transmit(&huart4,&yaw_target.send[i],1,100);
	}
	for(i = 0; i<4; i++)	
	{
		HAL_UART_Transmit(&huart4,&pitch_actual.send[i],1,100);
	}
	for(i = 0; i<4; i++)	
	{
		HAL_UART_Transmit(&huart4,&pitch_target.send[i],1,100);
	}
	
	
	/*֡帧尾*/
 	for(uint8_t i=0; i<4; i++){
		HAL_UART_Transmit(&huart4,&tail[i],1,100);
	}
//	HAL_UART_Transmit(&huart4,&tail[0],1,100);
//	HAL_UART_Transmit(&huart6,&tail[1],1,100);
//	HAL_UART_Transmit(&huart6,&tail[2],1,100);
//	HAL_UART_Transmit(&huart6,&tail[3],1,100);
}

//void vofa_send(void)
//{
//	uint16_t i=0;

////	vofa_date_t[1].c=Eular[0];
////	vofa_date_t[2].c=Eular[2];
////	vofa_date_t[3].c=pid_gimbal[1].set;
////	vofa_date_t[4].c=pid_gimbal[3].set;
////chassis
////	vofa_date_t[0].c=4000;
////	vofa_date_t[1].c=motor_chassis[1].speed_rpm;
////	vofa_date_t[2].c=motor_chassis[2].speed_rpm;
////	vofa_date_t[3].c=motor_chassis[3].speed_rpm;
////	vofa_date_t[4].c=motor_chassis[4].speed_rpm;

////gimbal
//	vofa_date_t[1].c=Eular[0];
////		vofa_date_t[1].c=From_Vision_Data.Vision_Yaw;

////	vofa_date_t[2].c=Eular[2];
////	vofa_date_t[3].c=x;
////	vofa_date_t[4].c=z;
//	vofa_date_t[3].c=pid_gimbal[1].set;
////	vofa_date_t[4].c=pid_gimbal[3].set;
//	
//	
////shoot
////	vofa_date_t[0].c=pid_shoot[1].set;
////	vofa_date_t[1].c=motor_shoot[1].speed_rpm;
////	vofa_date_t[2].c=motor_shoot[2].speed_rpm;
////	vofa_date_t[5].c=pid_shoot[3].set;
////	vofa_date_t[3].c=motor_shoot[3].speed_rpm;
///******************************0**************************/
////	for(i = 0; i<4; i++)	
////	{
////		HAL_UART_Transmit(&huart6,&vofa_date_t[0].b[i],1,100);
////	}
///******************************1**************************/
//	for(i = 0; i<4; i++)
//	{
//		HAL_UART_Transmit(&huart6,&vofa_date_t[1].b[i],1,100);
//	}
///******************************2**************************/
//	for(i = 0; i<4; i++)
//	{
//		HAL_UART_Transmit(&huart6,&vofa_date_t[2].b[i],1,100);
//	}
///******************************3**************************/
////	for(i = 0; i<4; i++)
////	{
////		HAL_UART_Transmit(&huart6,&vofa_date_t[3].b[i],1,100);
////	}
/////******************************4**************************/
////		for(i = 0; i<4; i++)
////	{
////		HAL_UART_Transmit(&huart6,&vofa_date_t[4].b[i],1,100);
////	}


//}


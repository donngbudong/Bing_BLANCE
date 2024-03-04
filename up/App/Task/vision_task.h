#ifndef VISION_TASK_H
#define VISION_TASK_H
#include "System.h"

#define VISUAL_BUFFER_LEN		13



/*VISION×´Ì¬Ã¶¾Ù*/
typedef enum
{
	VS_ERR		= 0, 
	VS_NORMAL   = 1, 
	VS_LOST     = 2, 
}Vs_State_t;

typedef union 
{
	uint8_t input[4];
	float		output;
}Union_t; 

typedef struct 
{
	Vs_State_t Vs_State;
//	uint16_t distance[3];									//ÊÓ¾õ²â¾à
//	uint8_t Visual_State;
	Union_t	pitch;
	Union_t	yaw;
	Union_t	TX_Pitch;
	Union_t	TX_Yaw;
	
	float RX_Pitch;
	float RX_Yaw;

	float ratio;
	float MAX;
	float MIN;
	float distance[3];
	uint8_t STATE;
}Vision_Date_t;

void Visual_Task(void);
void Visual_SendData(void);
void Vision_read_data(uint8_t *data);

void VSIION_State_Report(void);
Vs_State_t VS_Check(void);
bool Judge_VISISON_Lost(void);

//#define    IF_VS_LOST      Judge_VISISON_Lost()


extern Vision_Date_t Vision;
#endif

#ifndef VISION_TASK_H
#define VISION_TASK_H
#include "System.h"

#define VISION_TX_BUFFER_LEN	28
#define VISION_FRAME_HEADER		(0x5A)

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


/* 帧头字节偏移 */
typedef enum {
	 SOF_Vision    =  0 ,//帧头偏移
	 DATA_Vision	 = 1 , //数据起始偏移
	 LEN_FRAME_HEADER = 1	,	// 帧头长度
	 LEN_FRAME_TAILER = 2	,	// 帧尾CRC16 
}Frame_Header_Offset_t;



/* 接收数据长度信息 */
typedef struct {
	/* Std */
	uint8_t LEN_RX_DATA 			;	// 接收数据段长度
  uint8_t LEN_RX_PACKET	    ;	// 接收包整包长度
} DataRX_Length_t;

/* 发送数据长度信息 */
typedef struct {
	/* Std */
  uint8_t TX_CRC16          ;//crc16偏移 
	uint8_t LEN_TX_DATA 		  ;	// 发送数据段长度
	uint8_t LEN_TX_PACKET	    ;	// 发送包整包长度
} DataTX_Length_t;


/* 接收数据段格式 */
typedef __packed struct 
{
//	bool tracking : 1;
//  uint8_t id : 3;          	// 0-outpost 6-guard 7-base
//  uint8_t armors_num : 3;  	// 2-balance 3-outpost 4-normal
//  uint8_t reserved : 1;			//保留
	uint8_t check;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
} AutoAim_Rx_Data_t;

/* 打符发送数据段格式 */
typedef __packed struct
{
	uint8_t detect_color  ;  // 0-red 1-blue
//  bool reset_tracker : 1;
//  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
} AutoAim_Tx_Data_t;


/* 帧尾格式 */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16校验码
} Vision_Frame_Tailer_t;

/* 自瞄发送包格式 */
typedef __packed struct
{
	uint8_t  			sof;		// 同步头
	AutoAim_Tx_Data_t	  TxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾		
} AutoAim_Tx_Packet_t;


/* 自瞄接收包格式 */
typedef __packed struct 
{
	uint8_t  			sof;		// 同步头
	AutoAim_Rx_Data_t	  RxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾	
} AutoAim_Rx_Packet_t;


/*发送端的信息*/
typedef struct
{
  AutoAim_Tx_Packet_t Packet;
  DataTX_Length_t  LEN;
}AutoAim_Tx_Info_t;

/*接收端的信息*/
typedef struct
{
  AutoAim_Rx_Packet_t Packet;
  DataRX_Length_t  LEN;
}AutoAim_Rx_Info_t;


/*电控<-->视觉 通信信息*/
typedef struct
{
  AutoAim_Tx_Info_t AutoAim_Tx;
  AutoAim_Rx_Info_t AutoAim_Rx;
}VisionRTx_t;



/*视觉任务总控结构体*/
typedef struct
{
  VisionRTx_t  VisionRTx;
}Vision_Info_t;


void Visual_Task(void);
void Visual_SendData(void);
void Vision_read_data(uint8_t *data);


void rm_vision(uint8_t *data);


void VSIION_State_Report(void);
Vs_State_t VS_Check(void);
bool Judge_VISISON_Lost(void);

//#define    IF_VS_LOST      Judge_VISISON_Lost()

void Vision_Init(void);
void VISION_SendData(void);
void VISION_ReadData(uint8_t *rxBuf);
void AUTO_AIM_Ctrl(void);

void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length);

extern Vision_Date_t Vision;
#endif

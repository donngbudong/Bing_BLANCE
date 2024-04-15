#ifndef VISION_TASK_H
#define VISION_TASK_H
#include "stm32h7xx_hal.h"

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
//	uint16_t distance[3];									//
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

#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78f
typedef unsigned char uint8_t;
enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};


//设置参数
struct SolveTrajectoryParams
{
    float k;             //弹道系数

    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v;      //当前弹速
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw

    //目标参数
    float xw;             //ROS坐标系下的x
    float yw;             //ROS坐标系下的y
    float zw;             //ROS坐标系下的z
    float vxw;            //ROS坐标系下的vx
    float vyw;            //ROS坐标系下的vy
    float vzw;            //ROS坐标系下的vz
    float tar_yaw;        //目标yaw
    float v_yaw;          //目标yaw速度
    float r1;             //目标中心到前后装甲板的距离
    float r2;             //目标中心到左右装甲板的距离
    float dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;        //偏置时间
    float s_bias;         //枪口前推的距离
    float z_bias;         //yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵 
    enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
		
		float yaw;
		float pitch;
};

//用于存储目标装甲板的信息
struct tar_pos
{
    float x;           //装甲板在世界坐标系下的x
    float y;           //装甲板在世界坐标系下的y
    float z;           //装甲板在世界坐标系下的z
    float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
};
//单方向空气阻力模型
extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
//完全空气阻力模型
extern float completeAirResistanceModel(float s, float v, float angle);
//pitch弹道补偿
extern float pitchTrajectoryCompensation(float s, float y, float v);
//根据最优决策得出被击打装甲板 自动解算弹道
extern void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);


/* 帧头字节偏移 */
typedef enum {
	 SOF_Vision    		= 0 ,	//帧头偏移
	 DATA_Vision	 		= 1 , //数据起始偏移
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

/* 发送数据段格式 */ 
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
void AUTO_AIM_Ctrl(void);

void Vision_read_data(uint8_t *data);


void rm_vision(uint8_t *data);


void VSIION_State_Report(void);
Vs_State_t VS_Check(void);

//#define    IF_VS_LOST      Judge_VISISON_Lost()

void Vision_Init(void);
void VISION_SendData(void);
void VISION_ReadData(uint8_t *rxBuf);
void AUTO_AIM_Ctrl(void);
void RM_Vision_Init(void);

void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length);

extern Vision_Date_t Vision;
#endif

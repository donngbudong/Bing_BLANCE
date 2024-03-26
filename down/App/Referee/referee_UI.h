#ifndef __REFEREE_UI_H
#define __REFEREE_UI_H
#include "System.h"

#include "referee.h"

#define HEADER_SOF                  0xA5
#define STUDENT_INTERACTIVE_DATA_CMD_ID 0x0301
/* UI绘制内容cmdID */
#define UI_DataID_Delete   0x0100 //客户端删除图形
#define UI_DataID_Draw1    0x0101 //客户端绘制1个图形
#define UI_DataID_Draw2    0x0102 //客户端绘制2个图形
#define UI_DataID_Draw5    0x0103 //客户端绘制5个图形
#define UI_DataID_Draw7    0x0104 //客户端绘制7个图形
#define UI_DataID_DrawChar 0x0110 //客户端绘制字符图形

/* UI删除操作 */
#define UI_Delete_Invalid 0 //空操作
#define UI_Delete_Layer   1 //删除图层
#define UI_Delete_All     2 //删除所有

/* UI图形操作 */
#define UI_Graph_invalid 0 //空操作
#define UI_Graph_Add     1 //增加图形
#define UI_Graph_Change  2 //修改图形
#define UI_Graph_Delete  3 //删除图形

/* UI图形类型 */
#define UI_Graph_Line      0 //直线
#define UI_Graph_Rectangle 1 //矩形
#define UI_Graph_Circle    2 //整圆
#define UI_Graph_Ellipse   3 //椭圆
#define UI_Graph_Arc       4 //圆弧
#define UI_Graph_Float     5 //浮点型
#define UI_Graph_Int       6 //整形
#define UI_Graph_String    7 //字符型

/* UI图形颜色 */
#define UI_Color_Main   0 //红蓝主色
#define UI_Color_Yellow 1 //黄色
#define UI_Color_Green  2 //绿色
#define UI_Color_Orange 3 //橙色
#define UI_Color_Purple 4 //紫色
#define UI_Color_Pink   5 //粉色
#define UI_Color_Cyan   6 //青色
#define UI_Color_Black  7 //黑色
#define UI_Color_White  8 //白色


typedef __packed struct
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

//typedef _packed struct 
//{ 
//  uint16_t data_cmd_id; 
//  uint16_t sender_id; 
//  uint16_t receiver_id; 
//  uint8_t user_data[x]; 
//}robot_interaction_data_t;


/* 自定义绘制UI结构体 -------------------------------------------------------*/
typedef __packed struct  //绘制UI UI图形数据
{
	uint8_t figure_name[3];  
	uint32_t operate_tpye:3;  
	uint32_t figure_tpye:3;  
	uint32_t layer:4;  
	uint32_t color:4;  
	uint32_t details_a:9; 
	uint32_t details_b:9; 
	uint32_t width:10;  
	uint32_t start_x:11;  
	uint32_t start_y:11;  
	uint32_t details_c:10;  
	uint32_t details_d:11;  
	uint32_t details_e:11;  	
} graphic_data_struct_t;

typedef __packed struct  //绘制UI UI字符串数据
{
	uint8_t  string_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t null;
	uint8_t stringdata[30];
} string_data_struct_t;

typedef __packed struct  //绘制UI UI删除图形数据
{
	uint8_t operate_tpye;
	uint8_t layer;
} delete_data_struct_t;

typedef __packed struct //绘制UI 绘制1个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;		//帧头
	uint16_t CMD_ID;																	//命令马		
	robot_interaction_data_t Interactive_Header;			//数据段头结构
	graphic_data_struct_t Graphic[1];									//数据段
	uint16_t CRC16;																		//帧尾
} UI_Graph1_t;

typedef __packed struct //绘制UI 绘制2个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //绘制UI 绘制5个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //绘制UI 绘制7个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //绘制UI 绘制1字符串完整结构体
{ 
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	string_data_struct_t String;
	uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //绘制UI UI删除图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;

void  referee_usart1_task(void);

#endif

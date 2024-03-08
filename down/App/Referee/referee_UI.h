//#ifndef __REFEREE_UI_H
//#define __REFEREE_UI_H
//#include "System.h"

//#include "referee.h"

//typedef __packed struct
//{
//  uint8_t  SOF;
//  uint16_t data_length;
//  uint8_t  seq;
//  uint8_t  CRC8;
//} frame_header_struct_t;


///* 自定义绘制UI结构体 -------------------------------------------------------*/
//typedef __packed struct  //绘制UI UI图形数据
//{
//	uint8_t  graphic_name[3];
//	uint32_t operate_tpye:3;
//	uint32_t graphic_tpye:3;
//	uint32_t layer:4;
//	uint32_t color:4;
//	uint32_t start_angle:9;
//	uint32_t end_angle:9;
//	uint32_t width:10;
//	uint32_t start_x:11;
//	uint32_t start_y:11;
//	uint32_t radius:10;
//	uint32_t end_x:11;
//	uint32_t end_y:11;
//} graphic_data_struct_t;

//typedef __packed struct  //绘制UI UI字符串数据
//{
//	uint8_t  string_name[3];
//	uint32_t operate_tpye:3;
//	uint32_t graphic_tpye:3;
//	uint32_t layer:4;
//	uint32_t color:4;
//	uint32_t start_angle:9;
//	uint32_t end_angle:9;
//	uint32_t width:10;
//	uint32_t start_x:11;
//	uint32_t start_y:11;
//	uint32_t null;
//	uint8_t stringdata[30];
//} string_data_struct_t;

//typedef __packed struct  //绘制UI UI删除图形数据
//{
//	uint8_t operate_tpye;
//	uint8_t layer;
//} delete_data_struct_t;

//typedef __packed struct //绘制UI 绘制1个图形完整结构体
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[1];
//	uint16_t CRC16;
//} UI_Graph1_t;

//typedef __packed struct //绘制UI 绘制2个图形完整结构体
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[2];
//	uint16_t CRC16;
//} UI_Graph2_t;

//typedef __packed struct //绘制UI 绘制5个图形完整结构体
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[5];
//	uint16_t CRC16;
//} UI_Graph5_t;

//typedef __packed struct //绘制UI 绘制7个图形完整结构体
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[7];
//	uint16_t CRC16;
//} UI_Graph7_t;

//typedef __packed struct //绘制UI 绘制1字符串完整结构体
//{ 
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	string_data_struct_t String;
//	uint16_t CRC16;
//} UI_String_t;

//typedef __packed struct  //绘制UI UI删除图形完整结构体
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	delete_data_struct_t Delete;
//	uint16_t CRC16;
//} UI_Delete_t;


//#endif

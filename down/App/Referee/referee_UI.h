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


///* �Զ������UI�ṹ�� -------------------------------------------------------*/
//typedef __packed struct  //����UI UIͼ������
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

//typedef __packed struct  //����UI UI�ַ�������
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

//typedef __packed struct  //����UI UIɾ��ͼ������
//{
//	uint8_t operate_tpye;
//	uint8_t layer;
//} delete_data_struct_t;

//typedef __packed struct //����UI ����1��ͼ�������ṹ��
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[1];
//	uint16_t CRC16;
//} UI_Graph1_t;

//typedef __packed struct //����UI ����2��ͼ�������ṹ��
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[2];
//	uint16_t CRC16;
//} UI_Graph2_t;

//typedef __packed struct //����UI ����5��ͼ�������ṹ��
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[5];
//	uint16_t CRC16;
//} UI_Graph5_t;

//typedef __packed struct //����UI ����7��ͼ�������ṹ��
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	graphic_data_struct_t Graphic[7];
//	uint16_t CRC16;
//} UI_Graph7_t;

//typedef __packed struct //����UI ����1�ַ��������ṹ��
//{ 
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	string_data_struct_t String;
//	uint16_t CRC16;
//} UI_String_t;

//typedef __packed struct  //����UI UIɾ��ͼ�������ṹ��
//{
//	frame_header_struct_t Referee_Transmit_Header;
//	uint16_t CMD_ID;
//	robot_interaction_data_t Interactive_Header;
//	delete_data_struct_t Delete;
//	uint16_t CRC16;
//} UI_Delete_t;


//#endif

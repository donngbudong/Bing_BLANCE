#ifndef __REFEREE_UI_H
#define __REFEREE_UI_H
#include "System.h"

#include "referee.h"

#define HEADER_SOF                  0xA5
#define STUDENT_INTERACTIVE_DATA_CMD_ID 0x0301
/* UI��������cmdID */
#define UI_DataID_Delete   0x0100 //�ͻ���ɾ��ͼ��
#define UI_DataID_Draw1    0x0101 //�ͻ��˻���1��ͼ��
#define UI_DataID_Draw2    0x0102 //�ͻ��˻���2��ͼ��
#define UI_DataID_Draw5    0x0103 //�ͻ��˻���5��ͼ��
#define UI_DataID_Draw7    0x0104 //�ͻ��˻���7��ͼ��
#define UI_DataID_DrawChar 0x0110 //�ͻ��˻����ַ�ͼ��

/* UIɾ������ */
#define UI_Delete_Invalid 0 //�ղ���
#define UI_Delete_Layer   1 //ɾ��ͼ��
#define UI_Delete_All     2 //ɾ������

/* UIͼ�β��� */
#define UI_Graph_invalid 0 //�ղ���
#define UI_Graph_Add     1 //����ͼ��
#define UI_Graph_Change  2 //�޸�ͼ��
#define UI_Graph_Delete  3 //ɾ��ͼ��

/* UIͼ������ */
#define UI_Graph_Line      0 //ֱ��
#define UI_Graph_Rectangle 1 //����
#define UI_Graph_Circle    2 //��Բ
#define UI_Graph_Ellipse   3 //��Բ
#define UI_Graph_Arc       4 //Բ��
#define UI_Graph_Float     5 //������
#define UI_Graph_Int       6 //����
#define UI_Graph_String    7 //�ַ���

/* UIͼ����ɫ */
#define UI_Color_Main   0 //������ɫ
#define UI_Color_Yellow 1 //��ɫ
#define UI_Color_Green  2 //��ɫ
#define UI_Color_Orange 3 //��ɫ
#define UI_Color_Purple 4 //��ɫ
#define UI_Color_Pink   5 //��ɫ
#define UI_Color_Cyan   6 //��ɫ
#define UI_Color_Black  7 //��ɫ
#define UI_Color_White  8 //��ɫ


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


/* �Զ������UI�ṹ�� -------------------------------------------------------*/
typedef __packed struct  //����UI UIͼ������
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

typedef __packed struct  //����UI UI�ַ�������
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

typedef __packed struct  //����UI UIɾ��ͼ������
{
	uint8_t operate_tpye;
	uint8_t layer;
} delete_data_struct_t;

typedef __packed struct //����UI ����1��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;		//֡ͷ
	uint16_t CMD_ID;																	//������		
	robot_interaction_data_t Interactive_Header;			//���ݶ�ͷ�ṹ
	graphic_data_struct_t Graphic[1];									//���ݶ�
	uint16_t CRC16;																		//֡β
} UI_Graph1_t;

typedef __packed struct //����UI ����2��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //����UI ����5��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //����UI ����7��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //����UI ����1�ַ��������ṹ��
{ 
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	string_data_struct_t String;
	uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //����UI UIɾ��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;

void  referee_usart1_task(void);

#endif

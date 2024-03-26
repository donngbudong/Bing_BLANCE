#include "referee_UI.h"
#include "stm32f4xx.h"                  // Device header
#include "usart.h"

/*==============================================================================
              ##### UI基本图形绘制函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 绘制直线 UI_Draw_Line
      (+) 绘制矩形 UI_Draw_Rectangle
      (+) 绘制整圆 UI_Draw_Circle
      (+) 绘制椭圆 UI_Draw_Ellipse
      (+) 绘制圆弧 UI_Draw_Arc
      (+) 绘制小数 UI_Draw_Float
      (+) 绘制整数 UI_Draw_Int
      (+) 绘制字符 UI_Draw_String
*/
void UI_Draw_Line(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                char                   GraphName[3], //图形名 作为客户端的索引
									uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									uint8_t                Layer,        //UI图形图层 [0,9]
									uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									uint16_t               Width,        //线宽
									uint16_t               StartX,       //起始坐标X
									uint16_t               StartY,       //起始坐标Y
									uint16_t               EndX,         //截止坐标X
									uint16_t               EndY)         //截止坐标Y
{
	Graph->figure_name[0]  = GraphName[0];
	Graph->figure_name[1]  = GraphName[1];
	Graph->figure_name[2]  = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->figure_tpye		 = UI_Graph_Line;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->details_d       = EndX;
	Graph->details_e       = EndY;
}

void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                     char                   GraphName[3], //图形名 作为客户端的索引
									     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									     uint8_t                Layer,        //UI图形图层 [0,9]
							     	 	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
							     	   uint16_t               Width,        //线宽
							     		 uint16_t               StartX,       //起始坐标X
							     		 uint16_t               StartY,       //起始坐标Y
							     		 uint16_t               EndX,         //截止坐标X
							     		 uint16_t               EndY)         //截止坐标Y
{
	Graph->figure_name[0] = GraphName[0];
	Graph->figure_name[1] = GraphName[1];
	Graph->figure_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->figure_tpye     = UI_Graph_Rectangle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->details_d       = EndX;
	Graph->details_d       = EndY;
}

void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                  char                   GraphName[3], //图形名 作为客户端的索引
									  uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									  uint8_t                Layer,        //UI图形图层 [0,9]
							     	uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t               Width,        //线宽
										uint16_t               CenterX,      //圆心坐标X
							      uint16_t               CenterY,      //圆心坐标Y
										uint16_t               Radius)       //半径
{
	Graph->figure_name[0] = GraphName[0];
	Graph->figure_name[1] = GraphName[1];
	Graph->figure_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->figure_tpye    = UI_Graph_Circle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->details_c          = Radius;
}

void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                   char                   GraphName[3], //图形名 作为客户端的索引
									   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									   uint8_t                Layer,        //UI图形图层 [0,9]
							     	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										 uint16_t               Width,        //线宽
										 uint16_t               CenterX,      //圆心坐标X
							       uint16_t               CenterY,      //圆心坐标Y
										 uint16_t               XHalfAxis,    //X半轴长
										 uint16_t               YHalfAxis)    //Y半轴长
{
	Graph->figure_name[0] = GraphName[0];
	Graph->figure_name[1] = GraphName[1];
	Graph->figure_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->figure_tpye    = UI_Graph_Ellipse;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->details_d           = XHalfAxis;
	Graph->details_e           = YHalfAxis;
}

void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               StartAngle,   //起始角度 [0,360]
								 uint16_t               EndAngle,     //截止角度 [0,360]
								 uint16_t               Width,        //线宽
								 uint16_t               CenterX,      //圆心坐标X
							   uint16_t               CenterY,      //圆心坐标Y
								 uint16_t               XHalfAxis,    //X半轴长
								 uint16_t               YHalfAxis)    //Y半轴长
{
	Graph->figure_name[0] = GraphName[0];
	Graph->figure_name[1] = GraphName[1];
	Graph->figure_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->figure_tpye    = UI_Graph_Arc;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->details_a     = StartAngle;
	Graph->details_b       = EndAngle;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->details_d           = XHalfAxis;
	Graph->details_e           = YHalfAxis;
}

void UI_Draw_Float(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                 char                   GraphName[3], //图形名 作为客户端的索引
							     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								   uint8_t                Layer,        //UI图形图层 [0,9]
							     uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									 uint16_t               NumberSize,   //字体大小
									 uint16_t               Significant,  //有效位数
									 uint16_t               Width,        //线宽
							     uint16_t               StartX,       //起始坐标X
							     uint16_t               StartY,       //起始坐标Y
									 float                  FloatData)    //数字内容
{
	Graph->figure_name[0] = GraphName[0];
	Graph->figure_name[1] = GraphName[1];
	Graph->figure_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->figure_tpye    = UI_Graph_Float;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->details_a     = NumberSize;
	Graph->details_b       = Significant;//无效位
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->details_c        = (IntData & 0x000003ff) >>  0;
	Graph->details_d        = (IntData & 0x001ffc00) >> 10;
	Graph->details_e        = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_Int(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               NumberSize,   //字体大小
								 uint16_t               Width,        //线宽
							   uint16_t               StartX,       //起始坐标X
							   uint16_t               StartY,       //起始坐标Y
								 int32_t                IntData)      //数字内容
{
	Graph->figure_name[0] = GraphName[0];
	Graph->figure_name[1] = GraphName[1];
	Graph->figure_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->figure_tpye    = UI_Graph_Int;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->details_a     = NumberSize;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->details_c          = (IntData & 0x000003ff) >>  0;
	Graph->details_d           = (IntData & 0x001ffc00) >> 10;
	Graph->details_e           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_String(graphic_data_struct_t *String,        //UI图形数据结构体指针
	                  char                  StringName[3], //图形名 作为客户端的索引
							      uint8_t               StringOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								    uint8_t               Layer,         //UI图形图层 [0,9]
							      uint8_t               Color,         //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t              CharSize,      //字体大小
									  uint16_t              StringLength,  //字符串长度
									  uint16_t              Width,         //线宽
							      uint16_t              StartX,        //起始坐标X
							      uint16_t              StartY,        //起始坐标Y
										char                 *StringData)    //字符串内容
{
	String->figure_name[0] = StringName[0];
	String->figure_name[1] = StringName[1];
	String->figure_name[2] = StringName[2];
	String->operate_tpye   = StringOperate;
	String->figure_tpye   = UI_Graph_String;
	String->layer          = Layer;
	String->color          = Color;
	String->details_a    = CharSize;
	String->details_b      = StringLength;
	String->width          = Width;
	String->start_x        = StartX;
	String->start_y        = StartY;
//	for(int i = 0; i < StringLength; i ++) String->stringdata[i] = *StringData ++;
}

/*==============================================================================
              ##### UI完整图案推送函数 #####
  ==============================================================================
    [..]  该部分提供如下函数:
		  (+) 推送图案 UI_PushUp_Graphs
			(+) 推送字符 UI_PushUp_String
			(+) 删除图层 UI_PushUp_Delete
*/
void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* 与Counter相一致的UI_Graphx结构体头指针 */, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //假设只发一个基本图形
	
	/* 填充 frame_header */
	Graph->Referee_Transmit_Header.SOF  = HEADER_SOF;
	     if(Counter == 1) Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if(Counter == 2) Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if(Counter == 5) Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if(Counter == 7) Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq  = Graph->Referee_Transmit_Header.seq + 1;
	Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);
	
	/* 填充 cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* 填充 student_interactive_header */
	     if(Counter == 1) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if(Counter == 2) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if(Counter == 5) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if(Counter == 7) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;

	Graph->Interactive_Header.sender_id   = RobotID ;      //当前机器人ID
	
	Graph->Interactive_Header.receiver_id = RobotID+256; //对应操作手ID
	/* 填充 frame_tail 即CRC16 */
	     if(Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Graph1->CRC16 = CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
	}
	else if(Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
	}
	else if(Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Graph5->CRC16 = CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
	}
	else if(Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
	}
	
	/* 使用串口PushUp到裁判系统 */
	     if(Counter == 1) HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if(Counter == 2) HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if(Counter == 5) HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if(Counter == 7) HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Graph, sizeof(UI_Graph7_t));
}

void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* 填充 frame_header */
	String->Referee_Transmit_Header.SOF  = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq  = String->Referee_Transmit_Header.seq + 1;
	String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);
	
	/* 填充 cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* 填充 student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_id   = RobotID ;      //当前机器人ID
	String->Interactive_Header.receiver_id = RobotID + 256; //对应操作手ID
	
	/* 填充 frame_tail 即CRC16 */
	String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);
	
	/* 使用串口PushUp到裁判系统 */
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)String, sizeof(UI_String_t));
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* 填充 frame_header */
	Delete->Referee_Transmit_Header.SOF  = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq  = Delete->Referee_Transmit_Header.seq + 1;
	Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);
	
	/* 填充 cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* 填充 student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_id   = RobotID ;      //当前机器人ID
	Delete->Interactive_Header.receiver_id = RobotID + 256; //对应操作手ID
	
	/* 填充 frame_tail 即CRC16 */
	Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);
	
	/* 使用串口PushUp到裁判系统 */
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Delete, sizeof(UI_Delete_t));
}


/* Private define ------------------------------------------------------------*/
#define Max(a,b) ((a) > (b) ? (a) : (b))
/* 机器人ID */

#define Robot_ID_Red_Infantry3    3 //红方步兵3
#define Robot_ID_Red_Infantry4    4 //红方步兵4
#define Robot_ID_Red_Infantry5    5 //红方步兵5

#define Robot_ID_Blue_Infantry3 103 //蓝方步兵3
#define Robot_ID_Blue_Infantry4 104 //蓝方步兵4
#define Robot_ID_Blue_Infantry5 105 //蓝方步兵5

#define Robot_ID_Current Robot_ID_Red_Infantry4
/* 绘制UI专用结构体 */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_Delete_t UI_Delete;

/* 动态UI数据变量 */
uint8_t UI_AutoAim_Flag = 0;    //是否开启自瞄标志位
float   UI_Kalman_Speed = 0;    //卡尔曼预测速度
float   UI_Gimbal_Pitch = 0.0f; //云台Pitch轴角度
float   UI_Gimbal_Yaw   = 0.0f; //云台Yaw轴角度
uint8_t UI_Capacitance  = 10;   //电容剩余容量
uint8_t UI_fric_is_on   = 0;    //摩擦轮是否开启

/* 中央标尺高度变量 */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;

uint8_t autoaim_mode;//2:normal,3:small energy,4:big energy
uint8_t autoaim_armor;//0x10:auto,0x20:big,0x30:small
uint8_t if_predict;

uint16_t UI_PushUp_Counter = 261;
float    Capacitance_X;
void  referee_usart1_task(void)
{
  for(;;)
  {
		/* UI更新 */
		UI_PushUp_Counter++;
		if(UI_PushUp_Counter % 301 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  40,   50,  80,   150); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 5,  950,   666,  500,   1000); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //第一行十字竖
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  990,   y02, 1050,   y02); //第二行右横线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
//		if(UI_PushUp_Counter % 311 == 0) //静态UI预绘制 中央标尺2
//		{
//			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //第三行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //第三行中心点
//			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //第三行右横线
//			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //第四行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //第四行中心点
//			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //第四行右横线
//			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //第四行下竖线
//			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 321 == 0) //静态UI预绘制 小陀螺预警线
//		{
//			UI_Draw_Line(&UI_Graph5.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
//			UI_Draw_Line(&UI_Graph5.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
//			UI_Draw_Line(&UI_Graph5.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
//			UI_Draw_Line(&UI_Graph5.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
//			UI_Draw_Line(&UI_Graph5.Graphic[4], "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
//			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 331 == 0) //动态UI预绘制 图形
//		{
//			UI_Draw_Float (&UI_Graph2.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith轴角度
//			UI_Draw_Line  (&UI_Graph2.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Orange, 20, 1829, 330, 1870, 334);      //电容容量
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 341 == 0) //动态UI预绘制 字符串1
//		{
//			UI_Draw_String(&UI_String.String,     "203", UI_Graph_Add, 2, UI_Color_Black,  22, 8, 3,  400, 632, "Fric OFF"); //摩擦轮是否开启
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 21 == 0) //动态UI更新 字符串1
//		{
//			if(UI_fric_is_on == 1) 
//			{
//				if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==0)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre  NO");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==0)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre  NO");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==0)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre  NO");
//				}		
//				else if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==1)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre YES");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==1)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre YES");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==1)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre YES");
//				}		
//				
//				else if(autoaim_mode==0x03)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nXFu\n        \n       ");
//				}
//				else if(autoaim_mode==0x04)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nDFu\n        \n       ");
//				}
//			}
//			if(UI_fric_is_on == 0) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Black, 22, 8+4+9+8, 3,  100, 700, "Fric OFF\n   \n        \n       ");
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 10 == 0)  //动态UI更新 图形
//		{
//			/* Pitch轴当前角度 */
//			UI_Draw_Float(&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
//			
//			/* 超级电容容量 */
//			UI_Capacitance = Max(UI_Capacitance, 30);
//			Capacitance_X  = 1870.0f - 4.1f * UI_Capacitance;
//			if(50 < UI_Capacitance && UI_Capacitance <= 100) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Green , 20, Capacitance_X, 334, 1870, 334);
//			if(35 < UI_Capacitance && UI_Capacitance <=  50) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Yellow, 20, Capacitance_X, 334, 1870, 334);
//			if(0  < UI_Capacitance && UI_Capacitance <=  35) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Orange, 20, Capacitance_X, 334, 1870, 334);
//			
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
//			continue;
//		}
	}
}
	
//void referee_usart_task(void const * argument)
//{
//	/* 动态UI控制变量 */
//	uint16_t UI_PushUp_Counter = 261;
//	float    Capacitance_X;
//	
//	/* 裁判系统初始化 */
////	vTaskDelay(300);
//	
//	/* new UI */
//	while(1)
//	{

//		/* UI更新 */
//		UI_PushUp_Counter++;
//		if(UI_PushUp_Counter % 301 == 0) //静态UI预绘制 中央标尺1
//		{
//			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //第一行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //第一行十字横
//			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //第一行右横线
//			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //第一行十字竖
//			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
//			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  990,   y02, 1050,   y02); //第二行右横线
//			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 311 == 0) //静态UI预绘制 中央标尺2
//		{
//			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //第三行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //第三行中心点
//			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //第三行右横线
//			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //第四行左横线
//			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //第四行中心点
//			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //第四行右横线
//			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //第四行下竖线
//			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 321 == 0) //静态UI预绘制 小陀螺预警线
//		{
//			UI_Draw_Line(&UI_Graph5.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
//			UI_Draw_Line(&UI_Graph5.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
//			UI_Draw_Line(&UI_Graph5.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
//			UI_Draw_Line(&UI_Graph5.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
//			UI_Draw_Line(&UI_Graph5.Graphic[4], "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
//			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 331 == 0) //动态UI预绘制 图形
//		{
//			UI_Draw_Float (&UI_Graph2.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith轴角度
//			UI_Draw_Line  (&UI_Graph2.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Orange, 20, 1829, 330, 1870, 334);      //电容容量
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
//			continue;
//		}
////		if(UI_PushUp_Counter % 341 == 0) //动态UI预绘制 字符串1
////		{
////			UI_Draw_String(&UI_String.String,     "203", UI_Graph_Add, 2, UI_Color_Black,  22, 8, 3,  400, 632, "Fric OFF"); //摩擦轮是否开启
////			UI_PushUp_String(&UI_String, Robot_ID_Current);
////			continue;
////		}
////		if(UI_PushUp_Counter % 21 == 0) //动态UI更新 字符串1
////		{
////			if(UI_fric_is_on == 1) 
////			{
////				if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==0)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre  NO");
////				}
////				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==0)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre  NO");
////				}
////				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==0)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre  NO");
////				}		
////				else if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==1)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre YES");
////				}
////				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==1)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre YES");
////				}
////				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==1)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre YES");
////				}		
////				
////				else if(autoaim_mode==0x03)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nXFu\n        \n       ");
////				}
////				else if(autoaim_mode==0x04)
////				{
////					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nDFu\n        \n       ");
////				}
////			}
////			if(UI_fric_is_on == 0) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Black, 22, 8+4+9+8, 3,  100, 700, "Fric OFF\n   \n        \n       ");
////			UI_PushUp_String(&UI_String, Robot_ID_Current);
////			continue;
////		}
//		if(UI_PushUp_Counter % 10 == 0)  //动态UI更新 图形
//		{
//			/* Pitch轴当前角度 */
//			UI_Draw_Float(&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
//			
//			/* 超级电容容量 */
//			UI_Capacitance = Max(UI_Capacitance, 30);
//			Capacitance_X  = 1870.0f - 4.1f * UI_Capacitance;
//			if(50 < UI_Capacitance && UI_Capacitance <= 100) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Green , 20, Capacitance_X, 334, 1870, 334);
//			if(35 < UI_Capacitance && UI_Capacitance <=  50) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Yellow, 20, Capacitance_X, 334, 1870, 334);
//			if(0  < UI_Capacitance && UI_Capacitance <=  35) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Orange, 20, Capacitance_X, 334, 1870, 334);
//			
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
//			continue;
//		}
//	}
//}



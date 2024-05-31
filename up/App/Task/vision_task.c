/***      
《 视觉处理 》
***/
#include "vision_task.h"
#include "Device.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "usbd_cdc_if.h"


Vision_Date_t Vision = {
	.Vs_State = VS_LOST,
	.ratio = 0.6,
	.MAX =  10,
	.MIN = -10,
};
Vision_Info_t Vision_cj;
//------------------------驱动层------------------------
void Vision_Init(void)
{
  Vision_cj.VisionRTx.AutoAim_Rx.LEN.LEN_RX_DATA = sizeof(AutoAim_Rx_Data_t);
  Vision_cj.VisionRTx.AutoAim_Rx.LEN.LEN_RX_PACKET = sizeof(AutoAim_Rx_Packet_t);
  
  Vision_cj.VisionRTx.AutoAim_Tx.LEN.LEN_TX_DATA = sizeof(AutoAim_Tx_Data_t);
  Vision_cj.VisionRTx.AutoAim_Tx.LEN.LEN_TX_PACKET = sizeof(AutoAim_Tx_Packet_t);
  Vision_cj.VisionRTx.AutoAim_Tx.LEN.TX_CRC16 = sizeof(AutoAim_Tx_Packet_t) - 2;
}


uint8_t Vision_Tx_Buffer[VISION_TX_BUFFER_LEN];
AutoAim_Tx_Info_t  VISON;
struct SolveTrajectoryParams st;

void Visual_Task(void)
{
	if(KEY_Q == 1)
	{
		st.bias_time = 5;
	}
	if(KEY_E == 1)
	{
		st.bias_time = 100;
	}
	if(KEY_SHIFT == 1)
	{st.z_bias = st.z_bias + 0.001;}
	if(KEY_CTRL == 1)
	{st.z_bias = st.z_bias - 0.001;}
	AUTO_AIM_Ctrl();
	VISION_SendData();
	RM_Vision_Init();
	
	autoSolveTrajectory(&st.current_pitch,&st.current_yaw,&st.xw,&st.yw,&st.zw);
	st.pitch =  st.current_pitch * 180 / PI;
	st.yaw = st.current_yaw * 180 / PI;

}


/**
 *	@brief	cj发送数据填充
 *	@note		USB发送
 */
void AUTO_AIM_Ctrl(void)	
{	
	//发送
//	if(REF.Robot_Status.robot_id < 10)// 0-red 1-blue
//	{	Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.detect_color = 0x01;
//	}
//	else{Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.detect_color = 0x00;}
	Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.detect_color = 0x01;
  Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.roll = IMU_Get_Data.IMU_Eular[1]*PI/180;
  Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.pitch = -IMU_Get_Data.IMU_Eular[0]*PI/180;
	Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.yaw = IMU_Get_Data.IMU_Eular[2]*PI/180;
	
	Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.aim_x = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.x;
	Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.aim_y = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.y;
	Vision_cj.VisionRTx.AutoAim_Tx.Packet.TxData.aim_z = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.z;
}


void RM_Vision_Init(void)
{
	st.xw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.x;
	st.yw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.y;
	st.zw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.z;

	st.vxw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.vx;
	st.vyw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.vy;
	st.vzw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.vz;
	
	st.v_yaw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.v_yaw;
	st.tar_yaw = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.yaw;
	
	st.r1 = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.r1;
	st.r2 = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.r2;
	st.dz = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.dz;
	
	st.armor_id = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.id;
	st.armor_num = Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.armors_num;
	
}
/**
 *	@brief	cj读取视觉通信数据
 *	@note		USB中IRQ调用
 */
void VISION_ReadData(uint8_t *rxBuf)
{
  AutoAim_Rx_Info_t *AUTO = &Vision_cj.VisionRTx.AutoAim_Rx;
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF_Vision] == 0xA5) 
	{	
			if(Verify_CRC16_Check_Sum( rxBuf, AUTO->LEN.LEN_RX_PACKET ))
      {    /* 数据正确则拷贝接收包 */
				memcpy(&AUTO->Packet, rxBuf, AUTO->LEN.LEN_RX_PACKET);

      }
  }
}


/**
 *	@brief	cj发送视觉通信数据
 *	@note	USB发送
 */
void VISION_SendData(void)
{
  AutoAim_Tx_Info_t *AUTO = &Vision_cj.VisionRTx.AutoAim_Tx;
  int len = 0;

	len = AUTO->LEN.LEN_TX_PACKET;
	/* 写入帧头 */
	Vision_Tx_Buffer[SOF_Vision] = 0x5A;
	/* 数据段填充 */
	memcpy( &Vision_Tx_Buffer[DATA_Vision], 
					&AUTO->Packet.TxData, 
					 AUTO->LEN.LEN_TX_DATA );
	/* 写入帧尾CRC16 */
	Append_CRC16_Check_Sum(Vision_Tx_Buffer, len);
	/* 数据同步 */
//	memcpy(&AUTO->Packet , Vision_Tx_Buffer , len);
  /* 发送数据 */
  /* USB发送  */
	CDC_Transmit_HS(Vision_Tx_Buffer,len);
	/* 发送数据包清零 */
//	memset(Vision_Tx_Buffer, 0, VISION_TX_BUFFER_LEN);
}
/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间
/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
		if(t < 0)
				{
						//由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
//						printf("[WRAN]: Exceeding the maximum range!\n");
						//重置t，防止下次调用会出现nan
						t = 0;
						return 0;
				}	
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}


/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
//    for (i = 0; i < 20; i++)
//    {
//        angle_pitch = atan2(z_temp, s); // rad
//        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
//        dz = 0.3f*(z - z_actual);
//        z_temp = z_temp + dz;
//        if (fabsf(dz) < 0.00001f)
//        {
//            break;
//        }
//    }
			for (i = 0; i < 20; i++)
				{
						angle_pitch = atan2(z_temp, s); // rad
						z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
						if(z_actual == 0)
						{
								angle_pitch = 0;
								break;
						}
						dz = 0.3*(z - z_actual);
						z_temp = z_temp + dz;
//						printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
//								i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
						if (fabsf(dz) < 0.00001)
						{
								break;
						}
				}
		
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
    // 线性预测
    float timeDelay = st.bias_time/1000.0 + t;
    st.tar_yaw += st.v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
		int use_1 = 1;
		int i = 0;
    int idx = 0; // 选择的装甲板
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0f;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        	int idx = 0;
        	for (i = 1; i<4; i++)
        	{
        		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        		if (temp_dis_diff < dis_diff_min)
        		{
        			dis_diff_min = temp_dis_diff;
        			idx = i;
        		}
        	}

            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }

    }

    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    //这里符号给错了
//    *pitch = pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
//            *aim_z + st.z_bias, st.current_v);
//    *yaw = (float)(atan2(*aim_y, *aim_x));
    float temp_pitch = pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);
    if(temp_pitch)
        *pitch = temp_pitch;
    if(*aim_x || *aim_y)
        *yaw = (float)(atan2(*aim_y, *aim_x));
}



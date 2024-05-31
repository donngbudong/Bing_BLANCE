#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H


/** 《遥控器信息图》
 * ┌──────────────────────────────────────────┐
 * │ ┌───┐1  660                                       ┌──┐1                   │
 * │ │S 1   │3   Λ                              Λ       │S 2 │3 			        		 │
 * │ └───┘2       │                              │       └──┘2 					  			 │
 * │               │                              │                                   │
 * │ CH2<─────┼─────>660  CH0<─────┼─────>660                     │
 * │               │                              │                                   │
 * │               │                              │          	                       │
 * │                V                               V                                   │
 * │              CH3                              CH1                                  │
 * └──────────────────────────────────────────┘
 */
#include "stm32f4xx_hal.h"
#include "System.h"

#define abs(x) ((x)>0? (x):(-(x)))       //绝对值宏定义



#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u



/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
	 struct
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		uint8_t s1;
		uint8_t s2;
		int16_t sw;
	} rc;
	 struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;
	
	union {
    uint16_t key;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
} RC_Ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */
extern RC_Ctrl_t RC_Ctrl;
extern uint64_t Remote_time ;//遥控器

/* ----------------------- RC Channel Definition---------------------------- */
//#define RC_CH_VALUE_MIN ((uint16_t)-660 )
//#define RC_CH_VALUE_OFFSET ((uint16_t)0)
//#define RC_CH_VALUE_MAX ((uint16_t)660)
#define RC_CH0              RC_Ctrl.rc.ch0
#define RC_CH1              RC_Ctrl.rc.ch1
#define RC_CH2              RC_Ctrl.rc.ch2
#define RC_CH3              RC_Ctrl.rc.ch3
#define RC_S1               RC_Ctrl.rc.s1      
#define RC_S2               RC_Ctrl.rc.s2
#define RC_SW               RC_Ctrl.rc.sw

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define RC_SW_ERR ((uint16_t)4)
/* -----------------------PC Key-------------------------------- */
#define CONTROL 1
#if(CONTROL == 1)
#define KEY_W               RC_Ctrl.kb.bit.W		
#define KEY_S               RC_Ctrl.kb.bit.S		
#define KEY_A               RC_Ctrl.kb.bit.A		
#define KEY_D               RC_Ctrl.kb.bit.D	
#define KEY_SHIFT           RC_Ctrl.kb.bit.SHIFT	
#define KEY_CTRL            RC_Ctrl.kb.bit.CTRL		
#define KEY_Q               RC_Ctrl.kb.bit.Q		
#define KEY_E               RC_Ctrl.kb.bit.E		
#define KEY_R               RC_Ctrl.kb.bit.R		
#define KEY_F               RC_Ctrl.kb.bit.F		
#define KEY_G               RC_Ctrl.kb.bit.G		
#define KEY_Z               RC_Ctrl.kb.bit.Z		
#define KEY_X               RC_Ctrl.kb.bit.X		
#define KEY_C               RC_Ctrl.kb.bit.C		
#define KEY_V               RC_Ctrl.kb.bit.V		
#define KEY_B               RC_Ctrl.kb.bit.B		

#define KEY_ALL_CODE        RC_Ctrl.kb.key_code

/*鼠标三轴的移动速度*/
#define    MOUSE_X_MOVE_SPEED    (RC_Ctrl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (RC_Ctrl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (RC_Ctrl.mouse.z)
/* 检测鼠标按键状态 */
#define    MOUSE_LEFT    				(RC_Ctrl.mouse.press_l )
#define    MOUSE_RIGH    				(RC_Ctrl.mouse.press_r )
/* ----------------------- Internal Functions ----------------------------------- */
#elif(CONTROL == 2)
#define KEY_W               REF.Renote_Control.kb.bit.W	
#define KEY_S               REF.Renote_Control.kb.bit.W		
#define KEY_A               REF.Renote_Control.kb.bit.W		
#define KEY_D               REF.Renote_Control.kb.bit.W	
#define KEY_SHIFT           REF.Renote_Control.kb.bit.SHIFT	
#define KEY_CTRL            REF.Renote_Control.kb.bit.CTRL		
#define KEY_Q               REF.Renote_Control.kb.bit.Q		
#define KEY_E               REF.Renote_Control.kb.bit.E		
#define KEY_R               REF.Renote_Control.kb.bit.R		
#define KEY_F               REF.Renote_Control.kb.bit.F		
#define KEY_G               REF.Renote_Control.kb.bit.G		
#define KEY_Z               REF.Renote_Control.kb.bit.Z		
#define KEY_X               REF.Renote_Control.kb.bit.X		
#define KEY_C               REF.Renote_Control.kb.bit.C		
#define KEY_V               REF.Renote_Control.kb.bit.V		
#define KEY_B               REF.Renote_Control.kb.bit.B		

#define KEY_ALL_CODE        RC_Ctrl.kb.key_code

/*鼠标三轴的移动速度*/
#define    MOUSE_X_MOVE_SPEED    (REF.Renote_Control.mouse_x)
#define    MOUSE_Y_MOVE_SPEED    (REF.Renote_Control.mouse_y)
#define    MOUSE_Z_MOVE_SPEED    (REF.Renote_Control.mouse_z)
/* 检测鼠标按键状态 */
#define    MOUSE_LEFT    				(REF.Renote_Control.left_button_down)
#define    MOUSE_RIGH    				(REF.Renote_Control.right_button_down)
#endif
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_Ctrl_t *RC_Ctrl);


void RC_State_Report(void);
Rc_State_t RC_Check(void);

bool Judge_RC_DataErr(void);
bool Judge_RC_Lost(void);




#define    IF_RC_DATAERR   Judge_RC_DataErr()
#define    IF_RC_LOST      Judge_RC_Lost()
#define    IF_RC_NORMAL    !(IF_RC_DATAERR || IF_RC_LOST)

#endif

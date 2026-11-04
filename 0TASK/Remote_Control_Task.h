#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__
#include <stm32f4xx.h>

#define KEY_PRESSED_OFFSET_W      ((uint32_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S      ((uint32_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A      ((uint32_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D      ((uint32_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT  ((uint32_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL   ((uint32_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q      ((uint32_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E      ((uint32_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R      ((uint32_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F      ((uint32_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G      ((uint32_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z      ((uint32_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X      ((uint32_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C      ((uint32_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V      ((uint32_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B      ((uint32_t)0x01<<15)
#define RC_FRAME_LENGTH 18u

#define V_Forword   1000
#define V_Back     -1000
#define V_R         1000
#define V_L        -1000
#define W_R         100
#define W_L        -100
#define V_STOP       0

#define		FRAME_HEADER_1		0xA9
#define		FRAME_HEADER_2		0x53
#define		MODE_SW_C 	0
#define		MODE_SW_N 	1
#define		MODE_SW_S 	2

#define KEY_LOW_LIMIT      				 			25	//按键检测下限0.025s
#define KEY_SHORT_LONG_BOUNDARY		 				500	//长短按分界0.5s	

#define CH4_Dead_Zone								50

//#define RmtCtrl_Sensitivity								0.0001f
extern float RmtCtrl_Sensitivity;
#define Mouse_Sensitivity								0.001f

typedef enum{
	DR16_Remote=0,
	VTM_Remote,
}RC_Type;

/*****************************老接收机**********************************/

typedef enum{
	Keep      =0,
	L_2_M       ,
	M_2_L       ,
	M_2_H       ,
	H_2_M       ,
}Clicker_Action_e;			// 遥控器上开关状态枚举

typedef __packed enum {
    NONE,              // 无输入
    SHORT_PRESS,       // 单击:按下后松开时置1一个时间周期
    LONG_PRESS,        // 按住时间较长
    TAP_AND_HOLD,      // 长按：按下后还未松开置1，松开后置0
} Key_Mouse_Action_e;  // 按键动作枚举

typedef __packed struct
{
	int Cnt;			//检测到下降后开始计数，用于防止多次触发
	int Hold_Cnt;		//按键保持计数器，用于判断长按和短按
	uint8_t  Flag;		//当前按键状态
	uint8_t  Flag_Last;	//上一次按键状态
	Key_Mouse_Action_e Key_Mouse_Action;//按键动作
}Key_Mouse_Action_t;


typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int8_t s1;
    //1RC,3KeyMouse,2RELAX
	int8_t s2;
	u8 s1_last;
	u8 s2_last;
	Clicker_Action_e s1_Action;
	Clicker_Action_e s2_Action;
	u8 Trigger_H;
	u8 Trigger_L;
	int16_t	Trigger_H_Cnt;
	int16_t Trigger_L_Cnt;
	uint8_t Trigger_H_Hold;
	uint8_t Trigger_L_Hold;
}Remote_t;						//遥控数据结构体（仅遥控器上的）


typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;		//上一次左键状态
	uint8_t last_press_r;		//上一次右键状态
	uint8_t press_l;			//左键状态
	uint8_t press_r;			//右键状态
	float Mouse_W;//鼠标下的角速度
	Key_Mouse_Action_t Press_L_Action;	//鼠标左键动作
	Key_Mouse_Action_t Press_R_Action;	//鼠标右键动作
}Mouse_t;						//鼠标数据结构体

typedef	__packed struct
{
	uint16_t v;//Bitmask,Key states pack
	uint16_t last_v;
	float Key_V_Y;//按键下Y轴速度
	float Key_V_X;//按键下X轴速度
	Key_Mouse_Action_t Key_W_Action;	//W键动作
	Key_Mouse_Action_t Key_S_Action;	//同上...
	Key_Mouse_Action_t Key_A_Action;
	Key_Mouse_Action_t Key_D_Action;
	Key_Mouse_Action_t Key_Q_Action;
	Key_Mouse_Action_t Key_E_Action;
	Key_Mouse_Action_t Key_R_Action;
	Key_Mouse_Action_t Key_F_Action;
	Key_Mouse_Action_t Key_G_Action;
	Key_Mouse_Action_t Key_Z_Action;	
	Key_Mouse_Action_t Key_X_Action;
	Key_Mouse_Action_t Key_C_Action;
	Key_Mouse_Action_t Key_V_Action;
	Key_Mouse_Action_t Key_B_Action;
	Key_Mouse_Action_t Key_SHIFT_Action;
	Key_Mouse_Action_t Key_CTRL_Action;
}Key_t;							//键盘数据结构体

typedef __packed struct
{
	Remote_t Remote_Control;
	Mouse_t Remote_Mouse;
	Key_t Remote_Key;
}Remote;						//整体的遥控数据结构体（遥控+键盘+鼠标）

typedef enum
{
	OFFLINE=0,
    RC_RELAX,
	REMOTE,
	KEY_MOUSE,
}Control_Mode_e;



extern Remote Remote_data;	//遥控数据
extern Control_Mode_e RemoteCtrl_Mode;	//控制模式（遥控器or键鼠）
extern RC_Type RC_Type_e;
void Rmt_Input_Mode(void);
void Get_Remote_Data(uint8_t *RemoteData);;
void Key_Mouse_Action_Get(Mouse_t *_Mouse,Key_t *_Key);
void Cmd_Get(void);

void RmtCtrl_Input_Interpreter(void);


#endif

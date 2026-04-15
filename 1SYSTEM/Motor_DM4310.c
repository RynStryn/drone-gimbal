#include "main.h"
DM4310_t DM4310_Pitch;

void DM4310_Init(uint8_t ID,eDM4310_Mode Mode,DM4310_Init_State State)	//DM4310电机初始化
{
	uint8_t DATA[8];
	memset(DATA,0xFF,sizeof(DATA));
	uint16_t ID_offset=0;
	switch(Mode)
	{
		case DM4310_Mode_MIT:
			ID_offset=0;
		break;
		case DM4310_Mode_PV:
			ID_offset=0x100;
		break;
		case DM4310_Mode_V:
			ID_offset=0X200;
		break;
		default:
			
		break;
	}
	switch(State)
	{
		case DM4310_Init_Enable:
		DATA[7]=0xFC;
		CAN2_Transmit(ID+ID_offset,8,DATA);//enable signal.try once(tap)
		break;
		case DM4310_Init_Disable:
		DATA[7]=0xFD;
		CAN2_Transmit(ID+ID_offset,8,DATA);//disable signal.try once(tap)
		break;
		case DM4310_Init_Set_Zero_Position:
		DATA[7]=0xFE;
		CAN2_Transmit(ID+ID_offset,8,DATA);
		break;
		case DM4310_Init_Clear_Error:
		DATA[7]=0xFB;
		CAN2_Transmit(ID+ID_offset,8,DATA);
		break;
		default:
		DATA[7]=0xFD;									//状态未知则失能，防止意外；
		CAN2_Transmit(ID+ID_offset,8,DATA);
		break;
	}
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	MIT_Ctrl: MIT模式下的电机控制函数
* @param[in]:   ID:				电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void DM4310_MIT_Ctrl(uint8_t ID, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t DATA[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	//16,12,12,12,12
	vel/=6.0f;

	pos_tmp = float_to_uint(pos,  -3.14,	3.14,	16);
	vel_tmp = float_to_uint(vel,  -120,  	120,  	12);
	kp_tmp  = float_to_uint(kp,   0, 		500, 	12);
	kd_tmp  = float_to_uint(kd,   0, 		5, 		12);
	tor_tmp = float_to_uint(torq, -10,  	10,  	12);

	//16,12,12,12,12
	DATA[0] = ( pos_tmp 	>>8);
	DATA[1] =   pos_tmp;
	DATA[2] = ( vel_tmp 	>>4);
	DATA[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	DATA[4] =   kp_tmp;
	DATA[5] = ( kd_tmp 		>>4);
	DATA[6] = ((kd_tmp&0xF)	<<4)|(tor_tmp>>8);
	DATA[7] =   tor_tmp;
	
	CAN2_Transmit(ID,8,DATA);
}

//position,velocity||rad,rad/s
//??max(velMax)=12.566370614359171830206674001655rad/s
void DM4310_PV_Ctrl(uint8_t ID, float _pos, float _velMax)
{
	uint8_t DATA[8];
	_pos*=DEG2RAD;
    _velMax/=6.0f;//达妙的,暂时不动
    memcpy(DATA,&_pos,4);
	memcpy(&DATA[4],&_velMax,4);
    CAN2_Transmit(ID+0x100,0x08,DATA);
}
//rad/s
void DM4310_V_Ctrl(uint8_t ID,float _vel)
{
	uint8_t DATA[8];
	_vel/=6.0f;
    memcpy(DATA,&_vel,4);
    CAN2_Transmit(ID+0x200,0x08,DATA);
}
	
void DM4310_Receive_Data(DM4310_t* DM4310_Pitch,uint8_t *DATA)
{
	DM4310_Pitch->ID 		= (DATA[0])&0x0F;
	DM4310_Pitch->ERR 		= (DATA[0])>>4;
	DM4310_Pitch->POS 		= uint_to_float((DATA[1]<<8)|DATA[2], 		-180, 	180, 	16);
	DM4310_Pitch->VEL 		= uint_to_float((DATA[3]<<4)|(DATA[4]>>4), 	-120.0, 120, 	12);
	DM4310_Pitch->T 		= uint_to_float(((DATA[4]&0xF)<<8)|DATA[5], -18, 	18, 	12);
	DM4310_Pitch->T_MOS 	= (float)(DATA[6]);
	DM4310_Pitch->T_Rotor	= (float)(DATA[7]);
}



#include "main.h"

DJI6020_t DJI6020_Yaw;

void DJI6020_Receive_Data(DJI6020_t* My_Motor,uint8_t* Data)
{
	(*My_Motor).speed_rpm=(int16_t)((Data[2] << 8) | Data[3]);
	(*My_Motor).pos_deg=((Data[0] << 8) | Data[1])/8192.0f*360.0f;
	(*My_Motor).I_ofTorq=(int16_t)((Data[4] << 8) | Data[5]);
	(*My_Motor).Temp=Data[6];
	if((*My_Motor).Init_Flag==0&&(CH040_Data.yaw_agl_get!=0||CH040_Data.pitch_agl_get!=0))
	{
		(*My_Motor).Init_deg=(*My_Motor).pos_deg-236+30-120;
		(*My_Motor).Init_Flag=1;
		CH040_Data.Init_Flag=1;
	} 
	else if((*My_Motor).Init_Flag==1&&(*My_Motor).pos_deg_last>(*My_Motor).pos_deg&&((*My_Motor).pos_deg_last-(*My_Motor).pos_deg)>320)
	{
		(*My_Motor).pos_turns++;
	}
	else if((*My_Motor).Init_Flag==1&&(*My_Motor).pos_deg_last<(*My_Motor).pos_deg&&((*My_Motor).pos_deg-(*My_Motor).pos_deg_last)>320)
	{
		(*My_Motor).pos_turns--;
	}
	(*My_Motor).multi_turns_deg=(*My_Motor).pos_deg+360*(*My_Motor).pos_turns-236+30-120;
	(*My_Motor).pos_deg_last=(*My_Motor).pos_deg;
}

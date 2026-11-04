#include "main.h"

DJI2006_t DJI2006_Pock;
DJI2006_t DJI2006_Fric[3];

void DJI2006_Receive_Data(DJI2006_t* My_Motor,uint8_t* DATA)
{
	(*My_Motor).speed_rpm=(int16_t)((DATA[2] << 8) | DATA[3]);
	(*My_Motor).speed_deg=(*My_Motor).speed_rpm*6;
	
	(*My_Motor).pos_deg=((DATA[0] << 8) | DATA[1])/8192.0f*360.0f;
	
	(*My_Motor).Torq=(int16_t)(DATA[4] << 8) | DATA[5];
	
	if((*My_Motor).pos_deg_last>(*My_Motor).pos_deg&&((*My_Motor).pos_deg_last-(*My_Motor).pos_deg)>180)
	{
		(*My_Motor).pos_turns++;
	}
	else if((*My_Motor).pos_deg_last<(*My_Motor).pos_deg&&((*My_Motor).pos_deg-(*My_Motor).pos_deg_last)>180)
	{
		(*My_Motor).pos_turns--;
	}
	(*My_Motor).multi_turns_deg=(*My_Motor).pos_deg+360*(*My_Motor).pos_turns;
	(*My_Motor).pos_deg_last=(*My_Motor).pos_deg;
	
	(*My_Motor).out_deg=(*My_Motor).multi_turns_deg;
}

void DJI2006_Receive_Reversed_Data(DJI2006_t* My_Motor,uint8_t* DATA)
{
	(*My_Motor).speed_rpm=-(int16_t)((DATA[2] << 8) | DATA[3]);
	(*My_Motor).speed_deg=(*My_Motor).speed_rpm*6;
	
	(*My_Motor).pos_deg=-((DATA[0] << 8) | DATA[1])/8192.0f*360.0f;
	
	(*My_Motor).Torq=(int16_t)(DATA[4] << 8) | DATA[5];
	
	if((*My_Motor).pos_deg_last>(*My_Motor).pos_deg&&((*My_Motor).pos_deg_last-(*My_Motor).pos_deg)>180)
	{
		(*My_Motor).pos_turns++;
	}
	else if((*My_Motor).pos_deg_last<(*My_Motor).pos_deg&&((*My_Motor).pos_deg-(*My_Motor).pos_deg_last)>180)
	{
		(*My_Motor).pos_turns--;
	}
	(*My_Motor).multi_turns_deg=(*My_Motor).pos_deg+360*(*My_Motor).pos_turns;
	(*My_Motor).pos_deg_last=(*My_Motor).pos_deg;
	
	(*My_Motor).out_deg=(*My_Motor).multi_turns_deg;
}

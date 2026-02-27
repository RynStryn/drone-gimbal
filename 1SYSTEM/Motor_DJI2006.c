#include "main.h"

DJI2006_t DJI2006_Pock;
DJI2006_t DJI2006_Fric[3];

void DJI2006_Receive_Data(DJI2006_t* My_Motor,uint8_t* DATA)
{
	(*My_Motor).speed_rpm=(int16_t)((DATA[2] << 8) | DATA[3]);
	(*My_Motor).speed_degs=(*My_Motor).speed_rpm*6;
	
	(*My_Motor).pos_degs=((DATA[0] << 8) | DATA[1])/8192.0f*360.0f;
	
	(*My_Motor).N_m=(int16_t)(DATA[4] << 8) | DATA[5];
	
	if((*My_Motor).pos_degs_last>(*My_Motor).pos_degs&&((*My_Motor).pos_degs_last-(*My_Motor).pos_degs)>180)
	{
		(*My_Motor).pos_turns++;
	}
	else if((*My_Motor).pos_degs_last<(*My_Motor).pos_degs&&((*My_Motor).pos_degs-(*My_Motor).pos_degs_last)>180)
	{
		(*My_Motor).pos_turns--;
	}
	(*My_Motor).mult_degs=(*My_Motor).pos_degs+360*(*My_Motor).pos_turns;
	(*My_Motor).pos_degs_last=(*My_Motor).pos_degs;
	
	(*My_Motor).out_degs=(*My_Motor).mult_degs;
}

void DJI2006_Receive_Reversed_Data(DJI2006_t* My_Motor,uint8_t* DATA)
{
	(*My_Motor).speed_rpm=-(int16_t)((DATA[2] << 8) | DATA[3]);
	(*My_Motor).speed_degs=(*My_Motor).speed_rpm*6;
	
	(*My_Motor).pos_degs=-((DATA[0] << 8) | DATA[1])/8192.0f*360.0f;
	
	(*My_Motor).N_m=(int16_t)(DATA[4] << 8) | DATA[5];
	
	if((*My_Motor).pos_degs_last>(*My_Motor).pos_degs&&((*My_Motor).pos_degs_last-(*My_Motor).pos_degs)>180)
	{
		(*My_Motor).pos_turns++;
	}
	else if((*My_Motor).pos_degs_last<(*My_Motor).pos_degs&&((*My_Motor).pos_degs-(*My_Motor).pos_degs_last)>180)
	{
		(*My_Motor).pos_turns--;
	}
	(*My_Motor).mult_degs=(*My_Motor).pos_degs+360*(*My_Motor).pos_turns;
	(*My_Motor).pos_degs_last=(*My_Motor).pos_degs;
	
	(*My_Motor).out_degs=(*My_Motor).mult_degs;
}

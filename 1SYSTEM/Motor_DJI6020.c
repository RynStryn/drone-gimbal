#include "main.h"

DJI6020_t DJI6020_Yaw;

void DJI6020_Receive_Data(DJI6020_t* DJI6020_Yaw,uint8_t* Data)//
{
	(DJI6020_Yaw)->speed_rpm=(int16_t)((Data[2] << 8) | Data[3]);
	(DJI6020_Yaw)->pos_deg=((Data[0] << 8) | Data[1])/8192.0f*360.0f;
	(DJI6020_Yaw)->I_ofTorq=(int16_t)((Data[4] << 8) | Data[5]);
	(DJI6020_Yaw)->Temp=Data[6];
	if((DJI6020_Yaw)->Init_Flag==0&&(CH040_Data.yaw_agl_get!=0||CH040_Data.pitch_agl_get!=0))
	{
		(DJI6020_Yaw)->Init_deg=(DJI6020_Yaw)->pos_deg-236+30+30;
		(DJI6020_Yaw)->Init_Flag=1;
	    //__
		CH040_Data.Init_Flag=1;
	}
    else if ((DJI6020_Yaw)->Init_Flag == 1)
    {
        if ((DJI6020_Yaw)->pos_deg_last > (DJI6020_Yaw)->pos_deg
            && ((DJI6020_Yaw)->pos_deg_last - (DJI6020_Yaw)->pos_deg) > 320)
        {
            (DJI6020_Yaw)->pos_turns++;
        }
        else if ((DJI6020_Yaw)->pos_deg_last < (DJI6020_Yaw)->pos_deg
            && ((DJI6020_Yaw)->pos_deg - (DJI6020_Yaw)->pos_deg_last) > 320)
        {
            (DJI6020_Yaw)->pos_turns--;
        }
    }
	(DJI6020_Yaw)->multi_turns_deg=(DJI6020_Yaw)->pos_deg+360*(DJI6020_Yaw)->pos_turns-236+30+30;
	(DJI6020_Yaw)->pos_deg_last=(DJI6020_Yaw)->pos_deg;
}
//pos=355,init=30
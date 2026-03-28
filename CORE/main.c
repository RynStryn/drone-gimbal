#include "main.h"

int main()
{
 	Drone_akaFighter_init();
#ifdef TEST
    Remote_data.Remote_Control.s2=1;
    Gimbal.Mode=GIMBAL_RELAX;
#endif
	while (1)
	{
		
	

 }
}
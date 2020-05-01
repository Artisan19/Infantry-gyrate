#include "headfile.h"
/*************************************
函数名;remote_task
入口参数：无
出口参数：无
功能：mydata.Cortol_ch0：控制前进的速度
      mydata.Cortol_ch1： 控制横移速度
			mydata.Cortol_ch2：控制云台左右运动速度
			mydata.Cortol_ch3：控制云台上下运动速度

*************************************/
void remote_task(void)
{
	mydata.Cortol_ch0 = (mydata.dbus.remote.ch0 - 1024) * 7;
	mydata.Cortol_ch1 = (mydata.dbus.remote.ch1 - 1024) * 7;
	mydata.Cortol_ch2 -= (mydata.dbus.remote.ch2 - 1024) * 0.0004f;
	mydata.Cortol_ch3 += (mydata.dbus.remote.ch3 - 1024) * 0.005f;
	
}



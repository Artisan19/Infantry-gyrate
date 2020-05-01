#include "Auto_attackTask.h"


AUTODATA AutoData; 
 uint8_t mainfocus_rx_buffer[2][100]={0};
AUTODATA_From_MainFocus Auto_Origion ={0}; 


char  SendBuff[4];
void send_data(char mode, u8 cmd2)
{
	static unsigned char i = 0;
	
	SendBuff[0] = 0xFF;
	SendBuff[1] = mode;
	SendBuff[2] = cmd2;
	SendBuff[3] = 0xFE;
	
	for(i = 0; i < 4 ;i++)//循环发送数据
	{	 
		 while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);  //等待上次传输完成
		 USART_SendData(USART3,(uint8_t)SendBuff[i]);		 
	}
}

uint16_t rx_fail_cnt = 0;
void receiveMainFocusData(uint8_t *buff) 
{
    int i = 0;
		
	  for(i =0;i< 100;i++)
	  {
		  if(buff[i]== 0xFF && buff[i+7]== 0xFE)
			{
			  memcpy(&Auto_Origion,(u8 *)&buff[i+1],6);
			  break;
			}
			else
			{
				rx_fail_cnt++;		   
			}
	 }	
		AutoData.YawAxiaAngle   = (float)Auto_Origion.YawAxiaAngle /100.0f * 0.0174f ; //rad/s
		AutoData.PitchAxiaAngle = (float)-Auto_Origion.PitchAxiaAngle /100.0f * 0.0174f;
		AutoData.Mode = Auto_Origion.Mode;	
		
		
   if(AutoData.Mode == FPS_IS_120)
	{	
		AutoData.PitchAxiaAngle = AutoData.PitchAxiaAngle ;// 
	}
	if(AutoData.Mode == FPS_IS_60)
	{
		AutoData.PitchAxiaAngle = AutoData.PitchAxiaAngle ;// 
	}
}	



AUTODATA *GetAutoDataPoint()
{
   return &AutoData; 
}

bool iSIdentifySuccess()
{
   if(Auto_Origion.YawAxiaAngle !=0 ||Auto_Origion.PitchAxiaAngle!=0)
   {
      return true;
   }
   else 
   {
	  return false;
   }
}



#include "headfile.h"
#include "pid_modify.h"


 /*
void CMControlCalc(CMCONTORL *CmCotrol)
{
	if(GetWorkState() == PREPARE_STATE) //上电初始化时，设定底盘初始角度为0
	{
		CmCotrol->Angle = 0;
	}
	else //初始化完成后根据Y轴电机的机械角得到底盘相对云台的角度值
	{
		CmCotrol->Angle = (CmCotrol->Encoder_Init - CmCotrol->Encoder) * 360 / 8192;
	}
	
	CmCotrol->Control_CH2 = CmCotrol->Angle * 220;//角度值*200后得到底盘电机的控制值
	
	Power_Limit(CmCotrol);
	
	ID201_PID.SetSpeed = CmCotrol->Control_CH1 + CmCotrol->Control_CH2 + CmCotrol->Control_CH0; //麦克纳姆轮运动公式
	ID202_PID.SetSpeed = -(CmCotrol->Control_CH1 - CmCotrol->Control_CH2 - CmCotrol->Control_CH0);
	ID203_PID.SetSpeed = CmCotrol->Control_CH1 + CmCotrol->Control_CH2 - CmCotrol->Control_CH0;
	ID204_PID.SetSpeed = -(CmCotrol->Control_CH1 - CmCotrol->Control_CH2 + CmCotrol->Control_CH0);
	
	ID201_PID.Calc(&ID201_PID);//进行增量式PID计算
	ID202_PID.Calc(&ID202_PID);
	ID203_PID.Calc(&ID203_PID);
	ID204_PID.Calc(&ID204_PID);
	
	
	if((Dog.rc == Lose) || (GetWorkState() == PREPARE_STATE) || (GetWorkState() == STOP_STATE))//有模块离线时，底盘速度清零
	{
		Set_CM_Speed(CAN1, 0, 0, 0, 0);
	}
	else 
	{
		Set_CM_Speed(CAN1, ID201_PID.output, ID202_PID.output, ID203_PID.output, ID204_PID.output);//发送电机转矩电流控制值
	}
}
*/




void Power_Limit(CMCONTORL *CmCotrol);



void Fire_Motor_Pidparams_init()
{
   int i =0;	
 for(i=0;i<4;i++)
	{
	  PID_struct_init(&CAN2_Motor_PID[i],DELTA_PID,7000,500,12,1.8f,0); 
	//  PID_struct_init(&CAN2_Motor_PID[1],DELTA_PID,7000,500,12,1.8f,0); 
	}
	PID_struct_init(&CAN1_Motor_PID[6],DELTA_PID,7000,500,1,0.5f,0); 
}

void CM_Motor_Pidparams_init()
{
  int i =0;
	for(i=0;i<4;i++)
	{
	  PID_struct_init(&CAN1_Motor_PID[i],DELTA_PID,7000,500,12,1.0f,0.5);
	}
}

void CMControlCalc(char cm_mode)
{

	int i=0;
	static char TurnLeftFlag = 0;
	
	if(GetWorkState() != NORMAL_STATE ) //上电初始化时，设定底盘初始角度为0
	{
		CmCotrol.Angle = 0;
		CmCotrol.Control_CH2  = CmCotrol.Angle;
	}
	else //初始化完成后根据Y轴电机的机械角得到底盘相对云台的角度值
	{
		if(cm_mode == 0)  
		{
	     CmCotrol.Angle = (CmCotrol.Encoder_Init - CmCotrol.Encoder) * 360 / 8192;	
			 CmCotrol.Control_CH2 = CmCotrol.Angle * 200;
		}
	  else	if(cm_mode == 1)
		{
			  if(TurnLeftFlag == 0)
				{
		        CmCotrol.Angle = (CmCotrol.Encoder_Init - CmCotrol.Encoder  + 1000) * 360 / 8192;  
            if(CmCotrol.Angle == 0)	TurnLeftFlag = 1;						
				}
				else
				{
				    CmCotrol.Angle = (CmCotrol.Encoder_Init - CmCotrol.Encoder  - 1000) * 360 / 8192;  
            if(CmCotrol.Angle == 0)	TurnLeftFlag = 0;		
				}
					CmCotrol.Control_CH2 = CmCotrol.Angle * 120;
		}		
	}
	
	
	Power_Limit(&CmCotrol);
	
	
	CAN1_Motor_PID[0].set = CmCotrol.Control_CH1 + CmCotrol.Control_CH2 + CmCotrol.Control_CH0; //麦克纳姆轮运动公式
	CAN1_Motor_PID[1].set = -(CmCotrol.Control_CH1 - CmCotrol.Control_CH2 - CmCotrol.Control_CH0);
	CAN1_Motor_PID[2].set = CmCotrol.Control_CH1 + CmCotrol.Control_CH2 - CmCotrol.Control_CH0;
	CAN1_Motor_PID[3].set = -(CmCotrol.Control_CH1 - CmCotrol.Control_CH2 + CmCotrol.Control_CH0);
	
	
	for(i=0;i<4;i++)
	{
    pid_calc(&CAN1_Motor_PID[i]);	
	
	}
	
	for(i=0;i<4;i++)
	{
	   CAN1_Motor_PID[i].out = constrain(CAN1_Motor_PID[i].out ,-6500,6500);  
	}	

}

int   limit_ch1 =0;
int   limit_ch0 =0;
int   limit_ch2 =0;


void Power_Limit(CMCONTORL *CmCotrol)
{
		 	 
	  power.SetPower   = 65;	
			
		if(g_Referee_flag == 1)
		{
		   g_Referee_flag = 0 ; 
			
			 if(CmCotrol->Control_CH1 > 0  )
					{
								power.Kp   = 8.0f;
								power.output = (int16_t) (power.SetPower   - power.average) * power.Kp ;
							
							if(power.output>0)
							 {
										limit_ch1=6500;		
							 }
							 else 
							 {
									if(CmCotrol->Control_CH0 == 0 && CmCotrol->Control_CH2 == 0 )
											{
												 limit_ch1 =  limit_ch1 +  power.output;
												 limit_ch1 = constrain(limit_ch1,1500,6500);
											}	
									else
										 {
												 limit_ch1 =  limit_ch1 +  power.output;
												 limit_ch1 = constrain(limit_ch1,1500,6200);							 
										 }
									if(CmCotrol->Control_CH1>limit_ch1 ) CmCotrol->Control_CH1  = limit_ch1;		 							 
							 } 		 
					 }	
					else if(CmCotrol->Control_CH1 < 0)
						{	
						
								power.Kp   = 8.0f;
								power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
								
								if(power.output>0)
								{
									 limit_ch1=-6200;
								}
								else
								{
										if(CmCotrol->Control_CH0 == 0 && CmCotrol->Control_CH2 == 0 )
												{
													 limit_ch1 =  limit_ch1 -  power.output;
													 limit_ch1 = constrain(limit_ch1,-6500,-1500);
													if(CmCotrol->Control_CH1>limit_ch1 ) CmCotrol->Control_CH1  = limit_ch1;
												}	
										else
											 {
													 limit_ch1 =  limit_ch1 -  power.output;
													 limit_ch1 = constrain(limit_ch1,-6200,-1500);		
													 if(CmCotrol->Control_CH1<limit_ch1 ) CmCotrol->Control_CH1  = limit_ch1;
												}											 
								}
										
						 }
					
					if(CmCotrol->Control_CH0 > 0  )
					{
							power.Kp   = 8.0f;
							power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
						
						 if(power.output>0)limit_ch0=5500;
							else
							{
								limit_ch0 =  limit_ch0 +  power.output;
								limit_ch0 = constrain(limit_ch0,1500,5500);
								if(CmCotrol->Control_CH0>limit_ch0 ) CmCotrol->Control_CH0  = limit_ch0;			
							}

						 
					}
					
					else if(CmCotrol->Control_CH0 < 0)
					{	
							power.Kp   = 8.0f;
							power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
						
						 if(power.output>0)limit_ch0=-5500;
							else
							{
									limit_ch0 = limit_ch0 -power.output;	
									limit_ch0 = constrain(limit_ch0,-5500,-1500);		
								 if(CmCotrol->Control_CH0< limit_ch0 ) CmCotrol->Control_CH0  = limit_ch0;		
							}
						
					}
					
					
				if(CmCotrol->Control_CH2 > 0  )
					{
							power.Kp   = 1.2f;
							power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
						
							if(power.output>0)limit_ch2=10000;
							else
							{
								limit_ch2 =  limit_ch2 +  power.output;
								limit_ch2 = constrain(limit_ch2,9500,10000);
								if(CmCotrol->Control_CH2>limit_ch2 ) CmCotrol->Control_CH2  = limit_ch2;
							}

						 
					}
					
					else if(CmCotrol->Control_CH2 < 0)
					{	
							power.Kp   = 1.2f;
							power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
						
						if(power.output>0)limit_ch2=-10000;
							else
							{
									limit_ch2 = limit_ch2 -power.output;	
									limit_ch2 = constrain(limit_ch2,-10000,-9500);		
								 if(CmCotrol->Control_CH2< limit_ch2 ) CmCotrol->Control_CH2  = limit_ch2;
							}
					 }									
				}									
}


float Fire_speed_position(long int fre) 
{
  	float position_out = 0;
	  float velocity_out =0;
	
	 
	  CAN1_Motor_Position_PID[0].posi_kp= 2.0f;
	  CAN1_Motor_Position_PID[0].vec_kp = 1.0f;
	  CAN1_Motor_Position_PID[0].position_set  = fre;
	
	  CAN1_Motor_Position_PID[0].err[0] =  CAN1_Motor_Position_PID[0].position_set - (CAN1_Motor_Position_PID[0].position_get - CAN1_Motor_Position_PID[0].init_position);
	
	  position_out =  CAN1_Motor_Position_PID[0].posi_kp * ( CAN1_Motor_Position_PID[0].err[0]); 
	  velocity_out =  CAN1_Motor_Position_PID[0].vec_kp * (position_out - CAN1_Motor_PID[6].get);
	
	  velocity_out= constrain(velocity_out,-6000,6000);
	
	  return velocity_out;  
}



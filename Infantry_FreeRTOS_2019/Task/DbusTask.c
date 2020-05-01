#include "headfile.h"

static volatile InputMode_e inputmode = REMOTE_INPUT;   //输入模式设定
static void MouseKeyboardControlPress(MyData *mydata);

InputMode_e GetInputMode()
{
	return inputmode;
}

void ReceiveDbusData(MyData *mydata)			
{
	Dog.DbusReceive = 1;
	global_cout[LOST_COUNTER_INDEX_RC] = Online; 
	mydata->dbus.remote.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; 						//!< Channel 0
	mydata->dbus.remote.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; 		//!< Channel 1
	mydata->dbus.remote.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	mydata->dbus.remote.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	mydata->dbus.remote.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;															//!< Switch left
	mydata->dbus.remote.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); 																	//!< Switch right	
	mydata->dbus.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); 											//!< Mouse X axis
	mydata->dbus.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);										  //!< Mouse Y axis
	mydata->dbus.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); 										//!< Mouse Z axis
	mydata->dbus.mouse.press_l = sbus_rx_buffer[12]; 																					//!< Mouse Left Is Press ?
	mydata->dbus.mouse.press_r = sbus_rx_buffer[13]; 																					//!< Mouse Right Is Press ?
	mydata->dbus.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); 											//!< KeyBoard value
	
	SetInputMode(mydata);
	switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{		
			RemoteControlProcess(mydata);
			Bsp.OpenTheLaser(1);		
		}break;
		case KEY_MOUSE_INPUT:
		{
			MouseKeyboardControlPress(mydata);

      			
			Bsp.OpenTheLaser(1);
		}
		break;
		case STOP:
		{
          
		}break; 
	}
}

void RemoteControlProcess(MyData *mydata)
{
    if(GetWorkState()!=PREPARE_STATE)
		{
				mydata->Cortol_ch0 = (mydata->dbus.remote.ch0 - 1024) * 22;
				mydata->Cortol_ch1 = (mydata->dbus.remote.ch1 - 1024) * 22;
				mydata->Cortol_ch2 -= (mydata->dbus.remote.ch2 - 1024) * 0.0045f;
				mydata->Cortol_ch3 += (mydata->dbus.remote.ch3 - 1024) * 0.2f;
			
			 if(mydata->Cortol_ch3 < -250) 
						mydata->Cortol_ch3 = -250;
	     else 	if(mydata->Cortol_ch3 > 720) 
						mydata->Cortol_ch3 = 720;
		}
		
		
		if(mydata->dbus.remote.s1 == MOVE_UP)
		{
			 Set_Fire_Fre(0); 
       Set_fire_speed(0);  
			 Bsp.TurnOnTheShootMotor(0);
		}
		else if(mydata->dbus.remote.s1 == MOVE_MID)
		{
		    Set_Fire_Fre(0);
       Set_fire_speed(5500);				
			 Bsp.TurnOnTheShootMotor(1);
		}
		else  if(mydata->dbus.remote.s1 == MOVE_DOWM)
		{
			 Set_Fire_Fre(1500);//调拨弹电机
       Set_fire_speed(5500);	//调摩擦轮电机速度
			Bsp.TurnOnTheShootMotor(1);
		
		}
		
}

char Control_Mode = 0;

static void MouseKeyboardControlPress(MyData *mydata)
{
	static int time = 0;
	static char flag = 1,flag1 =0 ;
	mydata->dbus.mouse.x = constrain(mydata->dbus.mouse.x,-30,30);
	mydata->dbus.mouse.y = constrain(mydata->dbus.mouse.y,-30,30);
	
   mydata->Cortol_ch2 -= mydata->dbus.mouse.x   *  0.07f;
	 mydata->Cortol_ch3 -= mydata->dbus.mouse.y   *  1.8f;
	 
	 switch(mydata->dbus.key.v)
	 {
	    case  KEY_PRESSED_OFFSET_W:
		                            mydata->Cortol_ch0=0;if(mydata->Cortol_ch1 <= 7500) mydata->Cortol_ch1   += 550;break; 
			case  KEY_PRESSED_OFFSET_S:
																mydata->Cortol_ch0=0;if(mydata->Cortol_ch1 >= -7500) mydata->Cortol_ch1  -= 550;break;
			case  KEY_PRESSED_OFFSET_A:
																mydata->Cortol_ch1=0;if(mydata->Cortol_ch0 >= -5500) mydata->Cortol_ch0  -= 550;break;
		  case  KEY_PRESSED_OFFSET_D:
																mydata->Cortol_ch1=0;if(mydata->Cortol_ch0  <= 5500) mydata->Cortol_ch0  += 550;break;
			case KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_A:
				                      if(mydata->Cortol_ch1 <= 4500) mydata->Cortol_ch1   += 550;
                              if(mydata->Cortol_ch0 >= -2500) mydata->Cortol_ch0  -= 550;
                      			break;
				case KEY_PRESSED_OFFSET_W + KEY_PRESSED_OFFSET_D:
				                      if(mydata->Cortol_ch0  <= 2500) mydata->Cortol_ch0  += 550;
                              if(mydata->Cortol_ch1 <= 4500) mydata->Cortol_ch1   += 550;
                      			break;	
			case KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_A:
				                      if(mydata->Cortol_ch1 >= -4500) mydata->Cortol_ch1  -= 550;;
                              if(mydata->Cortol_ch0 >= -2500) mydata->Cortol_ch0  -= 550;
                      			break;
				case KEY_PRESSED_OFFSET_S + KEY_PRESSED_OFFSET_D:
				                      if(mydata->Cortol_ch0  <= 2500) mydata->Cortol_ch0  += 550;
                              if(mydata->Cortol_ch1 >= -4500) mydata->Cortol_ch1  -= 550;
                      			break;				
				case  KEY_PRESSED_OFFSET_Z:  flag = 1;							               
					     break;
				case  KEY_PRESSED_OFFSET_X:  flag = 2;
				      break;
				case  KEY_PRESSED_OFFSET_C:  flag = 3;
				      break;
				case  KEY_PRESSED_OFFSET_V:  flag = 4;
				      break;
			  case  KEY_PRESSED_OFFSET_B:  flag = 5;
				      break;
				
								
			default: 
			       if( mydata->Cortol_ch0 >0) 	  mydata->Cortol_ch0 -= 550	;
             else if( mydata->Cortol_ch0 <0)mydata->Cortol_ch0 += 550	;	
						 if( mydata->Cortol_ch1 >0) 	  mydata->Cortol_ch1 -= 550	;
             else if( mydata->Cortol_ch1 <0)mydata->Cortol_ch1 += 550	;	
				     break;
				
	 }
	 
//	    if(mydata->dbus.mouse.x != 0)
//			{
//			   if(mydata->Cortol_ch1 > 0)mydata->Cortol_ch1 = 5000;
//				 if(mydata->Cortol_ch1  < 0)mydata->Cortol_ch1 = -5000;
//			}
	 
	 
	     if(mydata->dbus.key.v == KEY_PRESSED_OFFSET_E)
			 {
			    Control_Mode = 1;
			 }
			 
			 else if(mydata->dbus.key.v == KEY_PRESSED_OFFSET_Q)
			 {
			    Control_Mode = 0;
			 }
			 
			 			 
	 
	 		if(mydata->Cortol_ch3 < -220) 
						mydata->Cortol_ch3 = -220;
	      else 	if(mydata->Cortol_ch3 > 750) 
						mydata->Cortol_ch3 = 750;
	 
	 if( workState != NORMAL_STATE)
			{
			  Set_fire_speed(0); 
				Set_Fire_Fre(0);
			}
			else
			{
			  	 if(mydata->dbus.mouse.press_l)
						 {	
	
									 switch(flag)
									 {
										 case 1:
											 Set_Fire_Mode(H_SPEED_A_L_FRE);  
											 break;
										 case 2:
											 Set_Fire_Mode(L_SPEED_A_H_FRE);
											 break;
										 case 3:
											 Set_Fire_Mode(M_SPEED_A_M_FRE);
											 break;
										 case 4:
											 Set_Fire_Mode(H_SPEED_A_H_FRE);
											 break;
										 case 5: flag1 = 1;break;
										 default:break;
									 
									 }							   
										
						  }			 
						 else
							 {
										 if(flag1 == 1)
										 {
											  if(flag != 5)
												{
												  flag1  = 0;
												}
												
												switch(Referee_date1.GameRobotState_t.robotLevel)
												{
												  case 1:
																	Set_Fire_Fre(2200);
																	Set_fire_speed(4500);		
																	ShootMotor_Left = ShootMotor_Right = 122;															 
														break;
													case 2:
																	Set_Fire_Fre(2300);
																	Set_fire_speed(4500);		
																	ShootMotor_Left = ShootMotor_Right = 125;															
														break;
													case 3:
																	Set_Fire_Fre(2300);
																	Set_fire_speed(4500);		
																	ShootMotor_Left = ShootMotor_Right = 130;																
														break;
													default:
																	Set_Fire_Fre(2200);
																	Set_fire_speed(4500);		
																	ShootMotor_Left = ShootMotor_Right = 122;	
													  break;
														
												}							 
											 time++;
											 if(time >= 250)
											 {
													 time = 0;
													 flag1 = 0;
												   flag = 1;
													 Set_Fire_Fre(0);
													 Set_fire_speed(4500);
													 ShootMotor_Left = ShootMotor_Right = 120;
											 }				 
										 }
										 else
										 {
												Set_Fire_Fre(0);
												Set_fire_speed(4500);		
												ShootMotor_Left = ShootMotor_Right = 120;									 
										 }									               
							 }
						  Heat2SetFire();
			}
}

void SetInputMode(MyData *mydata)
{
	if(mydata->dbus.remote.s2 == MOVE_UP)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(mydata->dbus.remote.s2 == MOVE_MID)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(mydata->dbus.remote.s2 == MOVE_DOWM)
	{
		inputmode = STOP;
	}	
}

u8 RemoteSelfCheck(void)
{
	if( (mydata.dbus.remote.ch1 != 1024) ||(mydata.dbus.remote.ch2 != 1024) || (mydata.dbus.remote.ch3 != 1024))
	{
		return false;
	}
	else 
		return true;
}

void DMA2_Stream5_IRQHandler(void)	
{

  if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET) 
	{
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		ReceiveDbusData(&mydata);				
	}
}

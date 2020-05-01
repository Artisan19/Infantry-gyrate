#include "headfile.h"
#include "pid_modify.h"


u8 GimbalInit = 0;


/*************************************************************************
函数名：YawControlLoop_Init
入口参数：
         GIMBALCONTORL *YawControl：传入Y轴的控制结构体指针，得到控制参数
				 
出口参数：Y轴速度环限流后的输出
功能：上电云台Y轴归中控制
说明：云台采用串级控制，有位置环（控制角度），速度环（控制速度），电流环（控制电流）
位置环相当于队长，指挥调节速度环，速度环相当于组长， 指挥调节电流环，层层指导，
Y轴编码器提供位置反馈，MPU6500提供角速度反馈，形成双环PID控制，电流环由驱动板控制。
云台的稳定性由结构本身确定，可通过调节PID参数提高稳定性，步兵采用纯比例控制
*************************************************************************/
float YawControlLoop_Init(GIMBALCONTORL *YawControl) //计算
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;

	
	YawControl->Limit += 1;//让电流值递增，防止归中时电流过大
	
	if(YawControl->Limit > 1500)		YawControl->Limit = 1500;
	
	 
	PositionOutPut = 	- (
													(
	                           YawControl->Encoder 
                           - YawControl->Encoder_Init
	                        )
                        	* 360 / 8192
	                    )  
	                         * YawControl->Position_kp;//Y轴位置环，注意符号的选择，应使误差向着减小的方向
		
	VelocityOutPut = -(PositionOutPut + YawControl->Gyro) * YawControl->Velocity_kp;//Y轴速度环，注意符号的选择，应使误差向着减小的方向
		
	yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//限制电流输出
	
				
	
	return yawCal;
}
static float yaw_ierror = 0;
/*************************************************************************
函数名：YawControlLoop_Work
入口参数：
         GIMBALCONTORL *YawControl：传入Y轴的控制结构体指针，得到控制参数
				 
出口参数：Y轴速度环限流后的输出
功能：云台Y轴正常工作时的控制
说明：云台采用串级控制，有位置环（控制角度），速度环（控制速度），电流环（控制电流）
位置环相当于队长，指挥调节速度环，速度环相当于组长， 指挥调节电流环，层层指导，
陀螺仪Z轴积分得到位置反馈，MPU6500提供角速度反馈，形成双环PID控制，电流环由驱动板控制。
云台的稳定性由结构本身确定，可通过调节PID参数提高稳定性，步兵采用纯比例控制
*************************************************************************/
float YawControlLoop_Work(GIMBALCONTORL *YawControl) //计算
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;
	
	static const float Position_Ki = 0.001;
	
	static float perror = 0 ;
	
	
	 
	//yaw_ierror += perror;
	
	YawControl->Limit = 4500;

	//PositionOutPut = -(perror * YawControl->Position_kp + Position_Ki * yaw_ierror);//Y轴位置环控制
  if(Control_Mode == 0)  
	{
		perror = YawControl->ControlData + YawControl->Angle;
			PositionOutPut =  -(perror * YawControl->Position_kp );	
				
			VelocityOutPut = (PositionOutPut - YawControl->Gyro) * YawControl->Velocity_kp;//Y轴速度环控制
				
			
	}
	else
	{
	  YawControl->ControlData = -YawControl->Angle ;
		
		if(AutoData.YawAxiaAngle != 0)
		PositionOutPut = (AutoData.YawAxiaAngle - 3.73f) * 8.2f;
		else
		AutoData.YawAxiaAngle = 0;
		
		
		VelocityOutPut = -(PositionOutPut + YawControl->Gyro) * 55.0f;//Y轴速度环控制
			
	}

	  yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//限制电流值	
	return yawCal;
}
/*************************************************************************
函数名：YawControlCalc
入口参数：
         GIMBALCONTORL *YawControl：传入Y轴的控制结构体指针，得到控制参数
				 
出口参数：
功能：云台Y轴根据状态选择输出并对初始遥控值清零
*************************************************************************/
void YawControlCalc(GIMBALCONTORL *YawControl)
{
	if(GetWorkState() != NORMAL_STATE )
	{
		mydata.Cortol_ch2 = mydata.imudata.Angle_Yaw = 0; //遥控值清零
		yaw_ierror = 0;
		
		YawControl->output = YawControlLoop_Init(YawControl);//上电得到Y轴输出
	}
	else 
	{
	    YawControl->output = YawControlLoop_Work(YawControl);//正常工作时的Y轴输出
	}
		
}

/*************************************************************************
函数名：PitchControlCalc
入口参数：
         GIMBALCONTORL *PitchControl：传入P轴的控制结构体指针，得到控制参数
				 
出口参数：无
功能：上电云台P轴归中控制
说明：云台采用串级控制，有位置环（控制角度），速度环（控制速度），电流环（控制电流）
位置环相当于队长，指挥调节速度环，速度环相当于组长， 指挥调节电流环，层层指导，
P轴编码器提供位置反馈，MPU6500提供角速度反馈，形成双环PID控制，电流环由驱动板控制。
云台的稳定性由结构本身确定，可通过调节PID参数提高稳定性，步兵采用纯比例控制
*************************************************************************/
void PitchControlCalc(GIMBALCONTORL *PitchControl) //计算
{
		float PositionOutPut = 0.0f, VelocityOutPut = 0.0f;
		static float Encoder_init = 0, Encoder_E, Gimbal_MaxOutPut = 0;
		static u8 mode = 0;
		static int cout = 0;
	
	  static char angle2zero = 0;

		Gimbal_MaxOutPut += 1;
		if(Gimbal_MaxOutPut > 5000)		Gimbal_MaxOutPut = 5000;

	if(Control_Mode == 0)
	{
		 PositionOutPut = 	(
													(
															PitchControl->Encoder
															- PitchControl->Encoder_Init  	//初始化的机械角度
															+ PitchControl->ControlData //控制量
													) 
													* 360 / 8192   //角度转换
											) 
											* PitchControl->Position_kp;
													
	
		VelocityOutPut = (PositionOutPut - PitchControl->Gyro) * PitchControl->Velocity_kp;//P轴角速度
	
	
	  if(VelocityOutPut > 5000)VelocityOutPut =5000;
		 if(VelocityOutPut < -5000)VelocityOutPut =-5000;
		 
		  
	}
	else
	{
	   PitchControl->ControlData =  Encoder_init- PitchControl->Encoder;
		
		if(AutoData.PitchAxiaAngle != 0)
		{
		  PositionOutPut = (AutoData.PitchAxiaAngle + 2.6) * 8.0f;
		}
		else
		{
		   PositionOutPut = 0;
		}
	   
	   VelocityOutPut = (PositionOutPut - PitchControl->Gyro) * 35.0f;
	}
	
	 PitchControl->output =VelocityOutPut;	

	//	PitchControl->output = constrain(VelocityOutPut, -4500, 4500);
}


int16_t Get_angle(void)
{
	int16_t angle = 0;
   angle = (( PitchControl.Encoder	- PitchControl.Encoder_Init ) *360 /8192 ) ; 	//初始化的机械角
	
	 if(angle < -150)
		  angle += 360;
	 return angle;
}



int16_t FireControlLoop(int16_t SetSpeed)//拨弹电机控制 
{
	   	CAN1_Motor_PID[6].set  =  SetSpeed;
	    pid_calc(&CAN1_Motor_PID[6]);	
	    CAN1_Motor_PID[6].out = constrain(CAN1_Motor_PID[6].out,-8000,8000);
	    return CAN1_Motor_PID[6].out;
	//return -(mydata.usemotor.rm2310_fire.speed - SetSpeed) * 0.60f;
}


void Set_Fire_Fre(int16_t fre)
{
	static int cnt,   cnt1 = 0;
	static char flag = 0;
	
	int16_t fire_set_speed =  fre;

	if(fire_set_speed != 0)
	{
		if(	CAN1_Motor_PID[6].get < 100)
				{
					 cnt++;
					 if(cnt > 40)
					 {
							flag = 1;
							cnt = 0;
					 }
				}
				
		if(flag == 1)
					 {
							 cnt1++;
							 fire_set_speed = -1500; //电机反转
							 if(cnt1 > 10)
							 {
								 flag = 0;
								 cnt1 = 0;
								 fire_set_speed = fre;
							 }
					 }	
	}
	
	FireControlLoop(fire_set_speed);
	
}



void Fire_speed_Control(int16_t speed)
{
    CAN2_Motor_PID[0].set =  -speed;
	  CAN2_Motor_PID[1].set =  speed;
	  pid_calc(&CAN2_Motor_PID[0] );
	  pid_calc(&CAN2_Motor_PID[1] );
	
	 CAN2_Motor_PID[0].out =  constrain(CAN2_Motor_PID[0].out,-7000,7000);
	 CAN2_Motor_PID[1].out =  constrain(CAN2_Motor_PID[1].out,-7000,7000);
 
}

void Set_fire_speed(int16_t speed) //设置摩擦轮速度
{
  Fire_speed_Control(speed);
}


void Set_Fire_Mode(Fire_Mode mode) 
{
	
	switch(mode)                                                       //fre: 88 :av:3， speed:4500 av:13
		{                                                                 //  :2800 av:9,  speed:6800 av: 27
	   case H_SPEED_A_L_FRE:
			        Set_Fire_Fre(800); // 射频:2发/s
              Set_fire_speed(6500);  //射速：
		          ShootMotor_Left = ShootMotor_Right = 150;
			 break;
	   case L_SPEED_A_H_FRE:
			        Set_Fire_Fre(2300); // 射频:8发/s
              Set_fire_speed(4500);;  //射速：
		          ShootMotor_Left = ShootMotor_Right = 120;
			 break;
	   case M_SPEED_A_M_FRE:
			        Set_Fire_Fre(1500); // 射频:5发/s
              Set_fire_speed(5500); //射速：
		           ShootMotor_Left = ShootMotor_Right = 130;
			 break;
	   case H_SPEED_A_H_FRE:
			        Set_Fire_Fre(2800); // 射频:8发/s
              Set_fire_speed(6500);  //射速：
		          ShootMotor_Left = ShootMotor_Right = 150;
			 break;
		 case Fire_STOP:
			 			 	Set_Fire_Fre(0);
			 break;	 
	 }		 
}


void Heat2SetFire()  //根据等级和热量设置拨弹轮
{ 
  switch(Referee_date1.GameRobotState_t.robotLevel)
	{
		case LEVEL1:
			  if( Referee_date1.powerHeatData.shooterHeat0 > 90 - 25)
				{
				     Set_Fire_Mode(Fire_STOP); 
             					
				}
	//			else
	//			{
	//			   Set_Fire_Mode(H_SPEED_A_L_FRE);
	//			}
			break;
				
		case LEVEL2:
			  if( Referee_date1.powerHeatData.shooterHeat0 > ( 180 - 20))
				{
				     Set_Fire_Mode(Fire_STOP);   
			  }
//				else
//				{
	//			   Set_Fire_Mode(M_SPEED_A_M_FRE);
	//			}
			break;

	 case LEVEL3:
			  if( Referee_date1.powerHeatData.shooterHeat0 > ( 360 - 24))
				{
				     Set_Fire_Mode(Fire_STOP);   
				}
 	//			else
	//			{
	//			   Set_Fire_Mode(L_SPEED_A_H_FRE);
	//			}
			break;
				
	 default://Set_Fire_Mode(M_SPEED_A_M_FRE);
		 break;
	}		
}


#include "headfile.h"
#include <math.h>
#include "pid_modify.h"


CMCONTORL     CmCotrol     = CMcontrolconfig;//底盘控制参数赋值 
GIMBALCONTORL YawControl   = yawcontrolconfig; //云台Y轴控制参数赋值
GIMBALCONTORL PitchControl = pitchcontrolconfig;//云台P轴控制参数赋值

 uint32_t  time_1ms = 0;  
 

volatile int16_t g_fire_speed = 0; 

WorkState_e workState     = PREPARE_STATE;//上电初始化状态，4-5s左右
WorkState_e lastWorkState = PREPARE_STATE;

void Gimbal_Params_Init()
{
	YawControl.Encoder_Init = 7909  ;//获取初始码盘值 R2:7787  R3:6313
	  YawControl.Position_kp = 8.5f;//设定位置环控制参数
	  YawControl.Velocity_kp = 42.0f;//设定速度环控制参数
	
	PitchControl.Encoder_Init = 7205     ;  // R2:6955   R3:409          ;	
	  PitchControl.Position_kp = 9.5f;
	  PitchControl.Velocity_kp = 48.0f;	
}


void WorkStateFSM(void);

WorkState_e GetWorkState(void)//获取工作状态
{
		return workState;
}

void Set_WorkState(WorkState_e state)
{
   workState  = state ;
}


static void WorkStateSwitchProcess(void)
{
    if(lastWorkState != workState  && workState == PREPARE_STATE)
		{
		     time_1ms = 0;
		}
}
/***************************************************
函数名:CMControlLoop
入口参数：无
出口参数：无
功能： 获取遥控参数值和云台Y轴码盘值，更新底盘控制参数
**************************************************/
void CMControlLoop(void)
{
	static char dance_flag = 0; 
	
	CM_Motor_Pidparams_init();
	CmCotrol.Control_CH0 = mydata.Cortol_ch0;//控制底盘前后移动
	CmCotrol.Control_CH1 = mydata.Cortol_ch1;//控制底盘左右横移
	CmCotrol.Encoder = mydata.usemotor.rm6623_y.encoder;//获取Y轴码盘值（经过连续化处理），用于计算角度差值，实现底盘跟随
	CmCotrol.Encoder_Init = YawControl.Encoder_Init;//获取云台初始化位置
 
	if(mydata.dbus.key.v == KEY_PRESSED_OFFSET_R)
	{
	   dance_flag  = 1;
	}
	else if(mydata.dbus.key.v == KEY_PRESSED_OFFSET_F)
	{
	   dance_flag  = 0;
	}
	
	if(dance_flag == 1)
	{
	   CmCotrol.Calc(1);
	}
	else
	{
	  CmCotrol.Calc(0);
	}
		
	
}
/***************************************************
函数名:YawControlLoop
入口参数：无
出口参数：无
功能： 获取Y轴电机码盘值和陀螺仪Z轴角速度，Z轴积分绕轴角
度值后进行串级控制
**************************************************/
void YawControlLoop(void)
{
	YawControl.Angle = mydata.imudata.Angle_Yaw; //获取绕轴角度
	YawControl.Gyro = mydata.imudata.Gyro_Z;//获取绕轴角速度
	
	YawControl.ControlData = mydata.Cortol_ch2;//获取绕轴控制值
	
	YawControl.Encoder = mydata.usemotor.rm6623_y.encoder;//获取Y轴码盘值

	YawControl.Calc(&YawControl);//云台Y轴控制
}
/***************************************************
函数名:PitchControlLoop
入口参数：无
出口参数：无
功能： 获取P轴电机码盘值和陀螺仪X轴角速度
度值后进行串级控制
**************************************************/
void PitchControlLoop(void)
{
	PitchControl.Gyro = mydata.imudata.Gyro_X;
	
	PitchControl.ControlData  = mydata.Cortol_ch3;
	PitchControl.Encoder      = mydata.usemotor.rm6623_p.encoder;
	
	PitchControl.Calc(&PitchControl);
}

u8 start =	1;
 
volatile long int fire_fre = 0;
 /***************************************************
函数名:GimbalMotorOutput
入口参数：无
出口参数：无
功能： 根据状态控制云台电机，当遥控值无法接收或模块离线时，云
台电机电流清零
**************************************************/
void GimbalMotorOutput(void)
{
	if(start == 1)
	{
		
			
	  WorkStateFSM();
		WorkStateSwitchProcess();
		
		YawControlLoop();
		PitchControlLoop();
				
		CMControlLoop();
		
		
		if( (Dog.rc == Lose) ||(GetWorkState() == STOP_STATE))
		{
			  Set_Gimbal_Current(CAN1, 0, 0, 0);
        Set_CM_Speed(CAN1,0,0,0,0);		
        CAN2_Set_FireSpeed(0,0,0,0);	
			ShootMotor_Left = ShootMotor_Right = 100;
		}
		else 
		{		
			Set_Gimbal_Current(CAN1, YawControl.output , PitchControl.output,CAN1_Motor_PID[6].out); //云台输出 R2:+ + +;R3 - + +
			Set_CM_Speed(CAN1,CAN1_Motor_PID[0].out, CAN1_Motor_PID[1].out, CAN1_Motor_PID[2].out, CAN1_Motor_PID[3].out); //底盘输出
		//	CAN2_Set_FireSpeed(-CAN2_Motor_PID[0].out,CAN2_Motor_PID[1].out,0,0);//调节摩擦轮方向
		}
	}		
}



void WorkStateFSM(void)
{
    lastWorkState =   workState;
	  static char cali_success_flag = 0;
    static char cnt = 0,cnt1 = 0;
	 switch(workState) 
	 {
		 case PREPARE_STATE:
			   time_1ms ++;
			   if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;
				 
				 }
				 else if(time_1ms > PREPARE_TIME_TICK_MS)
				 {
					  workState =  NORMAL_STATE;
            time_1ms = 0;				 
				 }
			 break;
		 case STANDBY_STATE:
			  if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;			 
				 }
			 
		 break;
		 case  NORMAL_STATE:
			 if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;
				 }
			 break;
		 
		 case STOP_STATE:
		     //  time_1ms = 0 ;
			     if(GetInputMode() != STOP)
					 {
					     workState =  PREPARE_STATE;  
						  //   workState = CALI_STATE;						   
					 }
			 break;
		 case CALI_STATE:
  /*          do{
						  	MPU6500_InitGyro_offset();
							  delay_ms(10);
							  cnt1++;
                if(gy_data > -3 && gy_data  < 3)
								{
								   cnt++;
								   cali_success_flag = 0;
									 if(cnt >=6)
									 {
									    cnt = 0;
										  cali_success_flag = 1;
									 }										 
								}						     							  
						  }			 
          while(cali_success_flag == 0 && cnt1 < 100);
					workState =  PREPARE_STATE; 
          time_1ms = 0;								
					cnt1 = 0; */
			 break;
		 default:break;
	 }
	
}


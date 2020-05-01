#include "headfile.h"
#include "pid_modify.h"
static int16_t cout_p = 0, cout_y = 0;
static int16_t new_data_p = 4096, old_data_p;
static int16_t new_data_y = 4096, old_data_y;

static long int old_data_m,new_data_m = 4096,cout_m;

int16_t watch =0,watch1=0;;

/* Get the value of the continuous encoder of the pitch axis */
/*************************************************************************
函数名：encoder_value_for_pitch
入口参数：
         input_data：P轴接收到的机械角
出口参数：无
功能：把P轴的机械角进行连续化处理
说明：当码盘值到临界时，比如上个数据是10,下个数据是-8000，说明已经过了临界角，
需进行连续化处理，否则得到的码盘值是错的，连续化处理的方法是当到达临界角时，顺
时针经过临界角时，圈数加1，逆时针转过时减1
*************************************************************************/
int encoder_value_for_pitch(int16_t input_data)  
{
	old_data_p = new_data_p;
	new_data_p = input_data;
	
	if((new_data_p - old_data_p) < -5200)
		cout_p++;
	else if((new_data_p - old_data_p) > 5200)
		cout_p--;
	watch1= cout_p;
	return new_data_p + cout_p * 8191;
}

/* Get the value of the continuous encoder of the yaw axis */
/*************************************************************************
函数名：encoder_value_for_yaw
入口参数：
         input_data：Y轴接收到的机械角
出口参数：无
功能：把Y轴的机械角进行连续化处理
*************************************************************************/
int encoder_value_for_yaw(int16_t input_data)
{
	old_data_y = new_data_y;
	new_data_y = input_data;
	
	if((new_data_y - old_data_y) < -5200)
		cout_y++;
	else if((new_data_y - old_data_y) > 5200)
		cout_y--;
	watch= cout_y;
	return new_data_y + cout_y * 8191;
}

long int encoder_value_for_2006(int16_t input_data)
{
	old_data_m = new_data_m;
	new_data_m = input_data;
	
	if((new_data_m - old_data_m) < -7000)
		cout_m++;
	else if((new_data_m - old_data_m) > 7000)
		cout_m--;
	return (new_data_m + cout_m * 8191);
}

/* Get the motor all the motor parameters */
/*************************************************************************
函数名：ReceiveCAN1Data
入口参数：无
出口参数：无
功能：根据电调ID接收电调的转速及机械角数据
注意：
电调发送频率1kHz；
机械角度值的范围：0~8191(0x1FFF)
实际电流测量值范围：-13000 ~ 13000
*************************************************************************/
int origion_yaw_encoder_val  =0;
int origion_pith_encoder_val =0;
int init_flag = 0;

void ReceiveCAN1Data(MyData *mydata)
{
	CanRxMsg rx_message; 
	CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
	
	switch(rx_message.StdId)
	{	
		case CAN_BUS1_MOTOR1:
		{             
			global_cout[LOST_COUNTER_INDEX_MOTOR1] = Online;
			CAN1_Motor_PID[0].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
		}
		break;
		
		case CAN_BUS1_MOTOR2:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR2] = Online;
			CAN1_Motor_PID[1].get= (rx_message.Data[2]<<8) + rx_message.Data[3];
		}	
		break;
			
		case CAN_BUS1_MOTOR3:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR3] = Online;
			CAN1_Motor_PID[2].get = (rx_message.Data[2]<<8) + rx_message.Data[3];			
		}
		break;
			
		case CAN_BUS1_MOTOR4:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR4] = Online;
			CAN1_Motor_PID[3].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
		}
		break;			
		
		case CAN_BUS1_MOTOR5:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR5] = Online;
			origion_yaw_encoder_val = (rx_message.Data[0] << 8) | rx_message.Data[1];
			
			if(workState == PREPARE_STATE)
			{
			   if(origion_yaw_encoder_val -  YawControl.Encoder_Init > 5000)
				 {
				    mydata->usemotor.rm6623_y.encoder = origion_yaw_encoder_val - 8192;  
				 }
				 else if(origion_yaw_encoder_val -  YawControl.Encoder_Init < -5000)
				 {
				    mydata->usemotor.rm6623_y.encoder = origion_yaw_encoder_val + 8192; 
				 }
				 else
				 {
				      mydata->usemotor.rm6623_y.encoder  = origion_yaw_encoder_val ; 
				 }
			}
			else
			{
			   mydata->usemotor.rm6623_y.encoder = encoder_value_for_yaw(origion_yaw_encoder_val);//码盘值连续化处理 
			}

		}
		break;
		
		case CAN_BUS1_MOTOR6:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR6] = Online;
			origion_pith_encoder_val  = (rx_message.Data[0] << 8) | rx_message.Data[1];
			
			
		if(workState == PREPARE_STATE)
			{
			   if(origion_pith_encoder_val -  PitchControl.Encoder_Init > 5000)
				 {
				    mydata->usemotor.rm6623_p.encoder = origion_pith_encoder_val - 8192;  
				 }
				 else if(origion_pith_encoder_val -   PitchControl.Encoder_Init  < -5000)
				 {
				    mydata->usemotor.rm6623_p.encoder = origion_pith_encoder_val + 8192; 
				 }
				 else
				 {
				      mydata->usemotor.rm6623_p.encoder  = origion_pith_encoder_val  ; 
				 }
			}
			else
			{
			  mydata->usemotor.rm6623_p.encoder = encoder_value_for_pitch(origion_pith_encoder_val);//码盘值连续化处理
			}

		}
		break;
		
		case CAN_BUS1_MOTOR7:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR7] = Online; 
			mydata->usemotor.rm2310_fire.speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
			CAN1_Motor_PID[6].get =  (rx_message.Data[2] << 8) | rx_message.Data[3];
			CAN1_Motor_Position_PID[0].position_get = ((encoder_value_for_2006((rx_message.Data[0] << 8) | rx_message.Data[1]))/ 36);	
			if(init_flag == 0)
			{
			   CAN1_Motor_Position_PID[0].init_position  = CAN1_Motor_Position_PID[0].position_get; 
				 init_flag = 1;
			}
			
			
		}
		break;
		
		default:break;
	}
}

/*       CAN1_RX0_IRQHandler           */
/*************************************************************************
函数名：CAN1_RX0_IRQHandler
入口参数：无
出口参数：无
功能：CAN接收中断接收电调发送的机械角及转速数据
注意：
电调发送频率1kHz；
机械角度值的范围：0~8191(0x1FFF)
实际电流测量值范围：-13000 ~ 13000
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		ReceiveCAN1Data(&mydata);
	}
}

void CAN1_TX_IRQHandler()
{
  if(CAN_GetITStatus(CAN1,CAN_IT_TME) !=RESET)
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	 }
}
/****************************************************************************************************************************
函数原型：void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
入口参数：
1.CAN_TypeDef *CANx 指定是主CAN1还是从CAN2
2. int16_t gimbal_yaw_iq  云台ID为205的Y轴电调发送的转矩电流数据
3. int16_t gimbal_pitch_iq云台ID为206的Y轴电调发送的转矩电流数据
4. int16_t gimbal_fire_iq 云台ID为207的拨弹电调发送的转矩电流数据
5.int16_t cm4_iq 底盘ID为204的电调发送的转矩电流数据
出口参数：无
功能：发送云台电调的转矩电流数据
*****************************************************************************************************************************/
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
{
    CanTxMsg tx_message;  

      
    tx_message.StdId = 0x1FF;//发送标准ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//发送数据帧
    tx_message.DLC = 0x08;//数据帧发送8个字节的数据
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = (unsigned char)(gimbal_fire_iq >> 8);
    tx_message.Data[5] = (unsigned char)gimbal_fire_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

/*************************************************************************************************************
函数原型：void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
入口参数：
1.CAN_TypeDef *CANx 指定是主CAN1还是从CAN2
2.int16_t cm1_iq 底盘ID为201的电调发送的转矩电流数据
3.int16_t cm2_iq 底盘ID为202的电调发送的转矩电流数据
4.int16_t cm3_iq 底盘ID为203的电调发送的转矩电流数据
5.int16_t cm4_iq 底盘ID为204的电调发送的转矩电流数据
出口参数：无
功能：发送底盘电调的转矩电流数据
**************************************************************************************************************/

void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;//标准ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//发送数据帧
    tx_message.DLC = 0x08;//发送八个字节的数据
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//数据高八位
    tx_message.Data[1] = (uint8_t)cm1_iq;         //数据低八位
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
	
	
    CAN_Transmit(CANx,&tx_message);
}


void ReceiveCAN2Data()
{
	CanRxMsg rx_message; 
	CAN_Receive(CAN2, CAN_FIFO0, &rx_message);     
	switch(rx_message.StdId)
	{	
		case CAN_BUS2_MOTOR1:
		{             
			global_cout[LOST_COUNTER_INDEX_MOTOR8] = Online;
      CAN2_Motor_PID[0].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
		}
		break;
		
		case CAN_BUS2_MOTOR2:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR9] = Online;
			CAN2_Motor_PID[1].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
			
		}	
		break;
				case CAN_BUS2_MOTOR3:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR10] = Online;
			CAN2_Motor_PID[2].get= (rx_message.Data[2]<<8) + rx_message.Data[3];	
			
		}	
		break;
				case CAN_BUS2_MOTOR4: 
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR11] = Online;
			CAN2_Motor_PID[3].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
			
		}	
		break;
		
		default:break;
	}
}


void CAN2_RX0_IRQHandler(void)
{   
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 
    ReceiveCAN2Data();
	}
	
}


void CAN2_TX_IRQHandler()
{
		if(CAN_GetITStatus(CAN2,CAN_IT_TME) !=RESET)
		{
			 CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		 }  
}


void CAN2_Set_FireSpeed( int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	  int i=0;
	  u8 mbox;
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;//标准ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//发送数据帧
    tx_message.DLC = 0x08;//发送八个字节的数据
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//数据高八位
    tx_message.Data[1] = (uint8_t)cm1_iq;         //数据低八位
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    mbox = CAN_Transmit(CAN2,&tx_message);
	
	// while(CAN_TransmitStatus(CAN2,mbox) ==CAN_TxStatus_Failed && i++<0xffff);

}

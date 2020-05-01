#include "MotorCAN.h"
#include "detect_task.h"

//声明电机变量
static motor_measure_t motor_yaw, motor_pit, motor_trigger[3], motor_chassis[4],motor_fric[2];

static void MotorCAN1_Init()
{
	GPIO_InitTypeDef       gpio;  	
	NVIC_InitTypeDef       nvic;
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;		        									
	gpio.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOD, &gpio);
	
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 
	
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 5;    
	nvic.NVIC_IRQChannelSubPriority = 0;          
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);	
	
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 5;    
	nvic.NVIC_IRQChannelSubPriority = 0;           
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN1);
	CAN_StructInit(&can);
  
	can.CAN_TTCM = DISABLE;		 
	can.CAN_ABOM = DISABLE;		 
	can.CAN_AWUM = DISABLE;			 
	can.CAN_NART = DISABLE;			 
	can.CAN_RFLM = DISABLE;			 
	can.CAN_TXFP = ENABLE;			 
	can.CAN_Mode = CAN_Mode_Normal;		 
	can.CAN_SJW  = CAN_SJW_1tq; 
	can.CAN_BS1 = CAN_BS2_6tq;
	can.CAN_BS2 = CAN_BS1_8tq;
	can.CAN_Prescaler = 3;   //CAN BaudRate 45/(1+6+8)/3=1Mbps
	CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0; 
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit; 
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;                    
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0; 
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);	
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		 
//	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		 
	
}

static void MotorCAN2_Init()
{
	GPIO_InitTypeDef       gpio;  	
	NVIC_InitTypeDef       nvic;
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;		        									
	gpio.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &gpio);
	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 
	
	nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 4;    
	nvic.NVIC_IRQChannelSubPriority = 0;          
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);	
	
	nvic.NVIC_IRQChannel = CAN2_TX_IRQn;					 
	nvic.NVIC_IRQChannelPreemptionPriority = 6;    
	nvic.NVIC_IRQChannelSubPriority = 0;           
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN2);
	CAN_StructInit(&can);
  
	can.CAN_TTCM = DISABLE;		 
	can.CAN_ABOM = DISABLE;		 
	can.CAN_AWUM = DISABLE;			 
	can.CAN_NART = DISABLE;			 
	can.CAN_RFLM = DISABLE;			 
	can.CAN_TXFP = ENABLE;			 
	can.CAN_Mode = CAN_Mode_Normal;		 
	can.CAN_SJW  = CAN_SJW_1tq; 
	can.CAN_BS1 = CAN_BS2_6tq;
	can.CAN_BS2 = CAN_BS1_8tq;
	can.CAN_Prescaler = 3;   //CAN BaudRate 45/(1+6+8)/3=1Mbps
	CAN_Init(CAN2, &can);

	can_filter.CAN_FilterNumber=14; 
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit; 
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;                    
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0; 
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);	
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);		 
 	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE); 	
}


void MoterCanInit()
{
	MotorCAN1_Init();
	MotorCAN2_Init();
}

void CanSendMess(CAN_TypeDef* CANx,uint32_t SendID,int16_t *message)
{
	int i=0;
	u8 mbox;
	
    CanTxMsg tx_message;
    tx_message.StdId = SendID; 
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data; 
    tx_message.DLC = 0x08; 
	
    tx_message.Data[0] = (uint8_t)(message[0] >> 8); 
    tx_message.Data[1] = (uint8_t) message[0];          
    tx_message.Data[2] = (uint8_t)(message[1] >> 8);
    tx_message.Data[3] = (uint8_t) message[1];
    tx_message.Data[4] = (uint8_t)(message[2] >> 8);
    tx_message.Data[5] = (uint8_t) message[2];
    tx_message.Data[6] = (uint8_t)(message[3] >> 8);
    tx_message.Data[7] = (uint8_t) message[3];
	
    mbox = CAN_Transmit(CANx,&tx_message);
 while(CAN_TransmitStatus(CANx,mbox) ==CAN_TxStatus_Failed && i++<0xffff);	
	
}


//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}

//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}

//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Fric_Motor_Measure_Point(uint8_t i)
{
    return &motor_fric[(i & 0x03)];
}

//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_shoot_Motor_Measure_Point(uint8_t i)
{
    return &motor_trigger[(i & 0x03)];
}


//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
	
//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
 void CAN_hook(CAN_TypeDef* CANx,CanRxMsg *rx_message) 
{

    switch (rx_message->StdId)
    {
		case CAN_YAW_MOTOR_ID:
		{
			//处理电机数据宏函数
			get_gimbal_motor_measuer(&motor_yaw, rx_message);
			//记录时间
			DetectHook(YawGimbalMotorTOE);
			break;
		}
		
		
		case CAN_PIT_MOTOR_ID:
		{
			//处理电机数据宏函数
			get_gimbal_motor_measuer(&motor_pit, rx_message);
			DetectHook(PitchGimbalMotorTOE);
			break;
		}
		
		case CAN_TRIGGER_42_MOTOR_ID:
		{
			//处理电机数据宏函数
			get_motor_measure(&motor_trigger[2], rx_message);
			//记录时间
			DetectHook(TriggerMotorTOE);
			break;
		}
		
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
			if(CANx == CAN1)
			{
				static uint8_t i = 0;
				//处理电机ID号
				i = rx_message->StdId - CAN_3508_M1_ID;
				//处理电机数据宏函数
				get_motor_measure(&motor_chassis[i], rx_message);
				//记录时间
				DetectHook(ChassisMotor1TOE + i);
				break;
			
			}
			else
			{
				 if(rx_message->StdId == CAN_SUPPLY_MOTOR_ID)
				 {
					//处理电机数据宏函数
					get_motor_measure(&motor_trigger[0], rx_message);
					//记录时间
					DetectHook(TriggerMotorTOE);
					break;
				 }
				 else if(rx_message->StdId == CAN_TRIGGER_15_MOTOT_ID)
				 {
					//处理电机数据宏函数
					get_motor_measure(&motor_trigger[1], rx_message);
					//记录时间
					DetectHook(TriggerMotorTOE);
					break;					   
				 }
				 else if(rx_message->StdId == CAN_FRIC_R_MOTOR_ID)
				 {
					//处理电机数据宏函数
					get_motor_measure(&motor_fric[0], rx_message);
					//记录时间
					DetectHook(TriggerMotorTOE);
					break;					    
				 }
				 else if(rx_message->StdId == CAN_FRIC_L_MOTOR_ID)
				 {
					//处理电机数据宏函数
					get_motor_measure(&motor_fric[1], rx_message);
					//记录时间
					DetectHook(TriggerMotorTOE);
					break;					    
				 }
			}

		}

		default:
		{
			break;
		}		
   }
}

	



#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "freertostask.h"   
#include "BasicPeripherals.h"
#include "FrictionMoterPWM.h"
#include "RemotDbus.h"
#include "MotorCAN.h"
#include "MainFocus_Usart.h"
#include "Referee.h"
#include "power_ctrl.h"
#include "start_task.h"
#include "buzzer.h"
#include "timer.h"
#include "adc.h"
#include "calibrate_task.h"
#include "usart6.h"

void BSP_Init(void);
//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

int main()
{
    BSP_Init();
  	StartTask();
    vTaskStartScheduler();
	/*Never arrive here*/
	while(1)
	{  
 	
   	}
} 

void BSP_Init()
{	
	/*设置优先级分组4，即4位抢占优先级，0位相应优先级*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//初始化滴答定时器
    delay_init(180);		  
	//初始化led,key,laser 
	BasicPreiph_Init(); 
	//初始化蜂鸣器
	buzzer_init(30000,90);
	//初始化摩擦轮控制pwm
    FrictionMoterPWM_Init(); 
	//初始化打印串口
	uart_init(115200);
	//初始化can1,can2
	MoterCanInit();
	//初始化串口用于妙算通讯
	MainFocusConfig();
	//初始化串口用于裁判系统通讯	
	//Referee_init();
	USART6_Init();
	USART6_DMA_Tx_Init();
	USART6_DMA_Rx_Init();
	//初始化can1,can2
	MoterCanInit();
    //定时器6 初始化
//    TIM6_Init(60000, 90);
    //stm32 板载温度传感器初始化
    temperature_ADC_init();	
	//24输出控制口 初始化
	power_ctrl_configuration();
	    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }	
	//初始化遥控DBUS
	RemotDbus_Init();
	//flash读取函数，把校准值放回对应参数
    cali_param_init();
}




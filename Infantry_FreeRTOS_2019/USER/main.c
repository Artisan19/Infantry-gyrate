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
//�ĸ�24v ��� ���ο��� ��� 709us
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
	/*�������ȼ�����4����4λ��ռ���ȼ���0λ��Ӧ���ȼ�*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//��ʼ���δ�ʱ��
    delay_init(180);		  
	//��ʼ��led,key,laser 
	BasicPreiph_Init(); 
	//��ʼ��������
	buzzer_init(30000,90);
	//��ʼ��Ħ���ֿ���pwm
    FrictionMoterPWM_Init(); 
	//��ʼ����ӡ����
	uart_init(115200);
	//��ʼ��can1,can2
	MoterCanInit();
	//��ʼ��������������ͨѶ
	MainFocusConfig();
	//��ʼ���������ڲ���ϵͳͨѶ	
	//Referee_init();
	USART6_Init();
	USART6_DMA_Tx_Init();
	USART6_DMA_Rx_Init();
	//��ʼ��can1,can2
	MoterCanInit();
    //��ʱ��6 ��ʼ��
//    TIM6_Init(60000, 90);
    //stm32 �����¶ȴ�������ʼ��
    temperature_ADC_init();	
	//24������ƿ� ��ʼ��
	power_ctrl_configuration();
	    //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }	
	//��ʼ��ң��DBUS
	RemotDbus_Init();
	//flash��ȡ��������У׼ֵ�Żض�Ӧ����
    cali_param_init();
}




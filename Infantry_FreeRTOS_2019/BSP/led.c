#include "stm32f4xx.h"
#include "led.h"

/****************************
函数名：LEDConfig
入口参数：无
出口参数：无
功能：初始化LED_GPIO
说明：PE7 --红色LED
      PE14 --绿色LED
			低电平点亮
******************************/
void LEDConfig(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		 //PE7                                                                                                                                                                                                                                                                                         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		//PF14  
	GPIO_Init(GPIOF,&GPIO_InitStructure);  
	
	GPIO_SetBits(GPIOF,GPIO_Pin_14);
	GPIO_SetBits(GPIOE,GPIO_Pin_7);
}



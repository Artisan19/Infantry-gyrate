#include "sys.h"
#include "usart.h"	
#include "main.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOSConfig.h"					
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//__use_no_semihosting was requested, but _ttywrch was 
int _ttywrch(int ch) 
{ 
   ch = ch; 
   return 0;
} 

//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	
#if ( USART_FOR_PRINT == 2)   	
	USART2->DR = (u8) ch;
    while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
#elif ( USART_FOR_PRINT == 3)
	USART3->DR = (u8) ch;
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕 
#else
	USART6->DR = (u8) ch;
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕 
#endif
	return ch;
}
#endif
 
//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound)
{
   //GPIO端口设置
   GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
#if EN_USART_RX
	NVIC_InitTypeDef NVIC_InitStructure;
#endif
	
#if ( USART_FOR_PRINT == 2)	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD5复用为USART2TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD6复用为USART2RX

#elif ( USART_FOR_PRINT == 3)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟	

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD5复用为USART3TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOD6复用为USART3RX
#else 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOG时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART6,ENABLE);//使能USART6时钟	

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART6); //GPIOD5复用为 USART6RX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_USART6); //GPIOD6复用为USART6TX
#endif

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
#if ( USART_FOR_PRINT == 2)	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;  
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
 #elif ( USART_FOR_PRINT == 3)  
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;  
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
#else
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;  
	GPIO_Init(GPIOG,&GPIO_InitStructure); 
#endif


   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
#if ( USART_FOR_PRINT == 2)	
  USART_Init(USART2, &USART_InitStructure);  
   USART_Cmd(USART2, ENABLE);  
#elif ( USART_FOR_PRINT == 3)  
  USART_Init(USART3, &USART_InitStructure);  
   USART_Cmd(USART3, ENABLE);  
#else
  USART_Init(USART6, &USART_InitStructure); 
  USART_Cmd(USART6, ENABLE); 	  
#endif	

#if EN_USART1_RX	
		#if ( USART_FOR_PRINT == 2)	
		   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
		   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
		#elif ( USART_FOR_PRINT == 3)  
		   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
		   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
		#else
		   USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断
		   NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口6中断通道 
		#endif	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
#endif
}


#if EN_USART_RX   //如果使能了接收
//串口中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		 
    } 

} 
#endif	

 




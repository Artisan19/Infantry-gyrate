#include "MainFocus_Usart.h"
#include "Auto_attackTask.h"
/****************************
函数名：MainFocusConfig
入口参数：无
出口参数：无
功能：配置串口3接收妙算数据
******************************/
void MainFocusConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART6，GPIOG ,DMA2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	//USART6_TX   GPIOA9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8 ; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStruct); 
	

	DMA_DeInit(DMA1_Stream1);   //将DMA的通道1寄存器重设为缺省值

	DMA_Cmd(DMA1_Stream1, DISABLE);						
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mainfocus_rx_buffer[0];//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
	DMA_InitStructure.DMA_BufferSize = 100;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);//初始化DMA Stream
	//开启双缓冲，首先存储DMA_Memory_0
	DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)mainfocus_rx_buffer[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
	DMA_Cmd(DMA1_Stream1, DISABLE); //Add a disable
	DMA_Cmd(DMA1_Stream1, ENABLE);


   //USART 初始化设置
    USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate = 115200;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	USART_ClearFlag(USART3, USART_FLAG_IDLE);	
    USART_ITConfig(USART3,USART_IT_TC,DISABLE);       
    USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);    
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART3, ENABLE); 	
	 
	 NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; 
     NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;                
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         
     NVIC_Init(&NVIC_InitStructure);        
}

void Send_MessToMainFocus(uint8_t cmd1,uint8_t cmd2)
{
	uint8_t sendbuf[4];
	
	sendbuf[0] = 0xFF;
	sendbuf[1] = cmd1;
	sendbuf[2] = cmd2;
	sendbuf[3] = 0xFE;
	
	for(char i=0;i<4;i++)
	{
	   	USART3->DR = sendbuf[i];
	   while((USART3->SR&0X40)==0);//循环发送,直到发送完毕
	}

}
	

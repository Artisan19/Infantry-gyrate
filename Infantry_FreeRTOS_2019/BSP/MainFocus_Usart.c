#include "MainFocus_Usart.h"
#include "Auto_attackTask.h"
/****************************
��������MainFocusConfig
��ڲ�������
���ڲ�������
���ܣ����ô���3������������
******************************/
void MainFocusConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART6��GPIOG ,DMA2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	//USART6_TX   GPIOA9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8 ; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStruct); 
	

	DMA_DeInit(DMA1_Stream1);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMA_Cmd(DMA1_Stream1, DISABLE);						
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;//DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mainfocus_rx_buffer[0];//DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//�洢��������ģʽ
	DMA_InitStructure.DMA_BufferSize = 100;//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// ʹ��ѭ��ģʽ 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);//��ʼ��DMA Stream
	//����˫���壬���ȴ洢DMA_Memory_0
	DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)mainfocus_rx_buffer[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
	DMA_Cmd(DMA1_Stream1, DISABLE); //Add a disable
	DMA_Cmd(DMA1_Stream1, ENABLE);


   //USART ��ʼ������
    USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate = 115200;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
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
	   while((USART3->SR&0X40)==0);//ѭ������,ֱ���������
	}

}
	

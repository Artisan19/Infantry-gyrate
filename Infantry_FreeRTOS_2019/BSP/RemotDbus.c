#include "RemotDbus.h"
#include "stddef.h"

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
volatile unsigned char sbus_rx_buffer[2][SBUS_RX_BUF_NUM];
RC_ctrl_t rc_ctrl;
 
void RemotDbus_Init()
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
     
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 , ENABLE);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	                                                                                                                                                                                                                                                                                                         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);
	
	USART_DeInit(USART1);
	
	USART_InitStructure.USART_BaudRate = 100000;	//SBUS 100K baudrate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even; //偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);	
	
	USART_ClearFlag(USART1, USART_FLAG_IDLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
											
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART1, ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA2_Stream2, DISABLE);	
	 while (DMA2_Stream2->CR & DMA_SxCR_EN); //禁止数据流
	 
	DMA_DeInit(DMA2_Stream2);	
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);	
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer[0];			
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;									
	DMA_InitStructure.DMA_BufferSize = 36;	//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界															
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;							
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)sbus_rx_buffer[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream2, DISABLE); //Add a disable
	DMA_Cmd(DMA2_Stream2, ENABLE);
		 
}

void RC_unable(void)
{
	USART_Cmd(USART1, DISABLE);
}
void RC_restart(uint16_t dma_buf_num)
{
	USART_Cmd(USART1, DISABLE);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num);

	USART_ClearFlag(USART1, USART_FLAG_IDLE);

	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

//返回遥控器控制变量，通过指针传递方式传递信息
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}




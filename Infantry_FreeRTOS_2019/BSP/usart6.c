#include "usart6.h"
#include "stdio.h"
#include "sys.h"
#include <stdarg.h>
#include <string.h>
#include "circular_buffer.h"
#include "Detect_Task.h"


#define USART6_BUF_SIZE         1300
#define MAX_RING_BUF_SIZE       3500
#define DMA_DOUBLE_BUFFER_MODE  1

// buffer for DMA transport
unsigned char USART6_RxBuf0[ USART6_BUF_SIZE ] = {0};
unsigned char USART6_RxBuf1[ USART6_BUF_SIZE ] = {0};
unsigned char USART6_TxBuf[ 300 ]  = {0};
//buffer for Circular buffer
#ifndef CIRC_BUFF_MALLOC
unsigned char USART6_TxBuff[ MAX_RING_BUF_SIZE ] = {0};
unsigned char USART6_RxBuff[ MAX_RING_BUF_SIZE ] = {0};
#endif


CircBuf_t USART6_RxCBuf, USART6_TxCBuf;

void USART6_Init(void)
{
    USART_DeInit(USART6);

    /**
     * @name PORT_CONFIG
     * @{ */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);

    GPIO_InitTypeDef GPIO_Cfg = { GPIO_Pin_9, GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP };
    GPIO_Init(GPIOG, &GPIO_Cfg);            // TX

    GPIO_Cfg.GPIO_Pin   = GPIO_Pin_14;
    GPIO_Cfg.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &GPIO_Cfg);            // RX
    /**  @} */


    /**
     * @name USART_CONFIG
     * @{ */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    USART_InitTypeDef UsartCfg;

    USART_StructInit( &UsartCfg );
    UsartCfg.USART_BaudRate = 115200;
	UsartCfg.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
    USART_Init(USART6, &UsartCfg);
    /**  @} */


    /**
     * @name NVIC_CONFIG
     * @{ */
    NVIC_InitTypeDef NVIC_Cfg = { USART6_IRQn, 1, 0, ENABLE };
    NVIC_Init(&NVIC_Cfg);
    /**  @} */


    /**
     * @name IRQ_CONFIG
     * @{ */
    USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART6, USART_IT_TC,   DISABLE);
    USART_ITConfig(USART6, USART_IT_TXE,  DISABLE);
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  // 使能IDLE中断

    USART_Cmd(USART6, ENABLE);
    /**  @} */

}

void USART6_DMA_Tx_Init(void)
{
    DMA_InitTypeDef DMA_Cfg;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /**
     * @name DMA_CONFIG
     * @{ */
    DMA_DeInit(DMA2_Stream6);
    DMA_StructInit(&DMA_Cfg);
    DMA_Cfg.DMA_Channel            = DMA_Channel_5;
    DMA_Cfg.DMA_PeripheralBaseAddr = (uint32_t) &(USART6->DR);
    DMA_Cfg.DMA_Memory0BaseAddr    = (uint32_t) USART6_TxBuf;
    DMA_Cfg.DMA_BufferSize         = USART6_BUF_SIZE;
    DMA_Cfg.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_Cfg.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_Cfg.DMA_Mode               = DMA_Mode_Normal;
    DMA_Cfg.DMA_Priority           = DMA_Priority_High;

    DMA_Init(DMA2_Stream6, &DMA_Cfg);
    /**  @} */

    /**
     * @name NVIC_CONFIG
     * @{ */
    NVIC_InitTypeDef NVIC_Cfg = { DMA2_Stream6_IRQn, 1, 1, ENABLE };
    NVIC_Init(&NVIC_Cfg);
    /**  @} */

    /**
     * @name IRQ_CONFIG
     * @{ */
    DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA2_Stream6, DISABLE);
    /**  @} */

#ifndef CIRC_BUFF_MALLOC
    CircBuf_Init(&USART6_TxCBuf, USART6_TxBuff, MAX_RING_BUF_SIZE);  // Allocate memery on heap, bigger buffer
#else
    CircBuf_Alloc(&USART6_TxCBuf, 64);                              // Allocate memery on stack
#endif
}

void USART6_DMA_Rx_Init(void)
{
    DMA_InitTypeDef DMA_Cfg;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /**
     * @name DMA_CONFIG
     * @{ */
    DMA_DeInit(DMA2_Stream1);
    DMA_StructInit(&DMA_Cfg);
    DMA_Cfg.DMA_Channel            = DMA_Channel_5;
    DMA_Cfg.DMA_PeripheralBaseAddr = (uint32_t) &(USART6->DR);
    DMA_Cfg.DMA_Memory0BaseAddr    = (uint32_t) USART6_RxBuf0;
    DMA_Cfg.DMA_BufferSize         = USART6_BUF_SIZE;
    DMA_Cfg.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_Cfg.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_Cfg.DMA_Mode               = DMA_Mode_Circular;
    DMA_Cfg.DMA_Priority           = DMA_Priority_VeryHigh;

#ifdef DMA_DOUBLE_BUFFER_MODE
    DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t) USART6_RxBuf1, DMA_Memory_0);
    DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
#endif

    DMA_Init(DMA2_Stream1, &DMA_Cfg);
    /**  @} */


    /**
     * @name NVIC_CONFIG
     * @{ */
    NVIC_InitTypeDef NVIC_Cfg = { DMA2_Stream1_IRQn, 1, 0, ENABLE };
    NVIC_Init(&NVIC_Cfg);
    /**  @} */

    /**
     * @name IRQ_CONFIG
     * @{ */
    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA2_Stream1, ENABLE);
    /**  @} */

#ifndef CIRC_BUFF_MALLOC
    CircBuf_Init(&USART6_RxCBuf, USART6_RxBuff, MAX_RING_BUF_SIZE);  // Allocate memery on heap, bigger buffer
#else
    CircBuf_Alloc(&USART6_RxCBuf, 64);                              // Allocate memery on stack
#endif
}

void USART6_Tx_IRQ_Callback(void)
{
    if(USART_GetITStatus(USART6, USART_IT_TXE) == RESET)
    {
        USART_ITConfig(USART6, USART_IT_TC, DISABLE);
    }
}

void USART6_Rx_IRQ_Callback(void)
{
    unsigned int len = 0;

    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART6);  // 清USART_IT_IDLE标志

			  DetectHook(RefereeSystemTOE);
        DMA_Cmd(DMA2_Stream1, DISABLE);
        DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);

#ifdef DMA_DOUBLE_BUFFER_MODE
        if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)
        {
            len = USART6_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);
            CircBuf_Push(&USART6_RxCBuf, USART6_RxBuf0, len);
        }
        else
        {
            len = USART6_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);
            CircBuf_Push(&USART6_RxCBuf, USART6_RxBuf1, len);
        }
#else
        // DMA_CIRCULAR_MODE
        len = USART6_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);
        CircBuf_Push(&USART6_RxCBuf, USART6_RxBuf0, len);
#endif

        DMA_SetCurrDataCounter(DMA2_Stream1, USART6_BUF_SIZE);
        DMA_Cmd(DMA2_Stream1, ENABLE);
    }
}

void USART6_IRQHandler(void)
{
    USART6_Rx_IRQ_Callback();

    USART6_Tx_IRQ_Callback();
}

void DMA2_Stream1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);

#ifdef DMA_DOUBLE_BUFFER_MODE
        if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)
        {
            CircBuf_Push(&USART6_RxCBuf, USART6_RxBuf0, USART6_BUF_SIZE);
            DMA_SetCurrDataCounter(DMA2_Stream1, 0);
        }
        else
        {
            CircBuf_Push(&USART6_RxCBuf, USART6_RxBuf1, USART6_BUF_SIZE);
            DMA_SetCurrDataCounter(DMA2_Stream1, 0);
        }
#else
        // DMA_CIRCULAR_MODE
        len = USART6_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);
        CircBuf_Push(&USART6_RxCBuf, USART6_RxBuf0, len);
#endif

    }
}

void DMA2_Stream6_IRQHandler(void)
{
    int len = 0;
    if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream6, DMA_IT_TCIF6);
        DMA_Cmd(DMA2_Stream6, DISABLE);

        // 打开发送完成中断,确保最后一个字节发送成功
        USART_ITConfig(USART6, USART_IT_TC, ENABLE);

        if(CircBuf_GetUsedSize(&USART6_TxCBuf) > 0)
        {
            len = CircBuf_Pop(&USART6_TxCBuf, USART6_TxBuf, USART6_BUF_SIZE);
            DMA_SetCurrDataCounter(DMA2_Stream6, (uint16_t)len);
            DMA_Cmd(DMA2_Stream6, ENABLE);
        }
    }				
}


void USART6_Send_(unsigned char *data, unsigned short len)
{
	memcpy(&USART6_TxBuf, data, len);
	DMA_SetCurrDataCounter(DMA2_Stream6, len);
	DMA_Cmd(DMA2_Stream6, ENABLE);
}


unsigned int USART6_Send(unsigned char *data, unsigned short len)
{
    unsigned int result = 0;

    unsigned int isBuffNotEmpty = CircBuf_GetUsedSize(&USART6_TxCBuf);

    result = CircBuf_Push(&USART6_TxCBuf, data, len);

    if(isBuffNotEmpty == 0)
    {
        // Circular Buffer empty, prepare data for DMA to send
        len = CircBuf_Pop(&USART6_TxCBuf, USART6_TxBuf, USART6_BUF_SIZE);
        DMA_SetCurrDataCounter(DMA2_Stream6, len);
    }

    DMA_Cmd(DMA2_Stream6, ENABLE);

    return result;
}

unsigned int USART6_Recv(unsigned char *data, unsigned short len)
{
    unsigned int result = 0;

    if(data != NULL)
        result = CircBuf_Pop(&USART6_RxCBuf, data, len);

    return result;
}

unsigned char USART6_At( unsigned short offset)
{
    return CircBuf_At(&USART6_RxCBuf, offset);
}

void USART6_Drop( unsigned short LenToDrop)
{
    CircBuf_Drop(&USART6_RxCBuf, LenToDrop);
}

unsigned int USART6_GetDataCount( void )
{
    return CircBuf_GetUsedSize(&USART6_RxCBuf);
}

void USART6_Free(void)
{
    CircBuf_Free(&USART6_RxCBuf);
}


#ifndef __USART6_H__
#define __USART6_H__

// void USART6_Configuration(void);
// void USART6_SendChar(unsigned char b);

void USART6_Init(void);
void USART6_DMA_Tx_Init(void);
void USART6_DMA_Rx_Init(void);

unsigned int  USART6_Send(unsigned char *data, unsigned short len);
unsigned int  USART6_Recv(unsigned char *data, unsigned short len);
unsigned char USART6_At( unsigned short offset);
void USART6_Drop( unsigned short LenToDrop);
unsigned int USART6_GetDataCount( void );

void USART6_Free(void);
void USART6_Send_(unsigned char *data, unsigned short len);
#endif

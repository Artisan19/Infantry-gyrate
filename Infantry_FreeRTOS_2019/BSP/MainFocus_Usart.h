#ifndef _MAINFOCUS_USART_H_
#define _MAINFOCUS_USART_H_
#include "sys.h"

void MainFocusConfig(void);
void Send_MessToMainFocus(uint8_t cmd1,uint8_t cmd2);

#endif 


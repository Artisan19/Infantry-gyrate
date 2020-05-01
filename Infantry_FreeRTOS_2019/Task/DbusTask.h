#ifndef __DBUSTASK_H__
#define __DBUSTASK_H__

#include <stm32f4xx.h>
#include "mydata.h"
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;

#define MOVE_UP 1
#define MOVE_MID 3
#define MOVE_DOWM 2

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN    ((uint16_t)364 )			//通道最小值
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)			//通道中间值
#define RC_CH_VALUE_MAX 	 ((uint16_t)1684)			//通道最大值
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP 		((uint16_t)1)
#define RC_SW_MID 	((uint16_t)3)
#define RC_SW_DOWN 	((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)

/* ----------------------- Data Struct ------------------------------------- */




void RemoteControlProcess(MyData *mydata);
u8 RemoteSelfCheck(void);
void SetInputMode(MyData *mydata);
InputMode_e GetInputMode(void);

extern char Control_Mode ;

#endif


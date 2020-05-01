#ifndef _AUTO_ATTACKTASK_H_
#define _AUTO_ATTACKTASK_H_

#include "stm32f4xx.h"
#include "stdbool.h"
#include "string.h"

void USART3_Configuration(void);

#define MAX_RECEIVE 32

#define red_car 0
#define blue_car 1

#define FPS_IS_120 1 
#define FPS_IS_60 2

#define our_mode_720 1
#define our_mode_480 0
#define our_mode 4
#define send_data_mode 2
#define Armor_mode 0
#define Rune_mode 1

#define Remote 2
#define AUTO_Fire 1
#define AUTO_Move 0
#define mode_is_480 0
#define mode_is_720 1


typedef struct _AUTODATA_
{
  float  YawAxiaAngle;
  float  PitchAxiaAngle;
  int16_t  Mode;
	float area;
	u8 _480_or_720;
	u8 Mode_;
}AUTODATA;

typedef __packed struct 
{
  int16_t  YawAxiaAngle;
  int16_t  PitchAxiaAngle;
  int16_t  Mode;
}AUTODATA_From_MainFocus;


extern AUTODATA AutoData;
extern uint8_t mainfocus_rx_buffer[2][100];

void TerminalCommand_receive_AUTO(void);
void send_data(char mode, u8 cmd2);

void ARMOR_MODE_RED(void);
void ARMOR_MODE_BLUE(void);
void RUND_MODE(void);

void MainFocusConfig(void);
void receiveMainFocusData(uint8_t *buf);
AUTODATA *GetAutoDataPoint(void);
extern bool iSIdentifySuccess(void);

#endif


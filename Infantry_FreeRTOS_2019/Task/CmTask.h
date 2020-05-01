#ifndef __CMTASK_H__
#define __CMTASK_H__

#include <stm32f4xx.h>


#define CMcontrolconfig \
{\
	.Angle = 0,\
	.Control_CH0 = 0,\
	.Control_CH1 = 0,\
	.Control_CH2 = 0,\
	.Encoder_Init = 5663,\
	.Encoder = 0,\
	.time = 0,\
	.Calc = &CMControlCalc,\
}\

typedef struct _CMCONTORL
{
	volatile float Angle;
	volatile	float Control_CH0;
	volatile	float Control_CH1;
	volatile	float Control_CH2;
	volatile int16_t Encoder_Init;
	volatile int16_t Encoder;
	int time;
	void (*Calc)(char cm_mode);
}CMCONTORL; 

void CMControlCalc(char cm_mode);
void CM_Motor_Pidparams_init(void);
void Fire_Motor_Pidparams_init(void);
float Fire_speed_position(long int fre);

extern  CMCONTORL CmCotrol;
extern int   limit_ch1  ;
extern int   limit_ch0 ;
extern int   limit_ch2 ;
#endif

#ifndef _FRICTIONMOTERPWM_H_
#define _FRICTIONMOTERPWM_H_
#include "main.h"
#define FRICTION_MOTER_L TIM2->CCR1
#define FRICTION_MOTER_R TIM2->CCR2

#define Fric_UP 1250
#define Fric_DOWN 1150 //1150// 1230
#define Fric_OFF 1000

void FrictionMoterPWM_Init(void);
void fric_off(void);
void fric1_on(uint16_t cmd);
void fric2_on(uint16_t cmd);

#endif


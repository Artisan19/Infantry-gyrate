#ifndef __CANTASK_H__
#define __CANTASK_H__
#include <stm32f4xx.h>

#define CAN_BUS1_MOTOR1           0x201
#define CAN_BUS1_MOTOR2           0x202 
#define CAN_BUS1_MOTOR3           0x203
#define CAN_BUS1_MOTOR4           0x204
#define CAN_BUS1_MOTOR5           0x205
#define CAN_BUS1_MOTOR6           0x206
#define CAN_BUS1_MOTOR7           0x207

#define CAN_BUS2_MOTOR1           0x201
#define CAN_BUS2_MOTOR2           0x202 
#define CAN_BUS2_MOTOR3           0x203
#define CAN_BUS2_MOTOR4           0x204
#define CAN_BUS2_MOTOR5           0x205
#define CAN_BUS2_MOTOR6           0x206 
#define CAN_BUS2_MOTOR7           0x207



void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);

void CAN2_Set_FireSpeed( int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);


extern int origion_yaw_encoder_val ;
extern int origion_pith_encoder_val;
extern  int16_t watch ,watch1;
#endif

#ifndef __PID_H__
#define __PID_H__

#include <stm32f4xx.h>

#define motor_id201 \
{\
	.SetSpeed = 0,\
	.NextSpeed = 0,\
	.Limit = 7000,\
	.Kp = 4,\
	.Ki = 1.0f,\
	.Kd = 1,\
	.output = 0,\
	.Calc = &IncrementalPidCalc,\
}\

typedef struct _IncrementalPid
{
	int16_t SetSpeed;
	int16_t NextSpeed;
	int16_t Limit;
	float Kp;	
	float Ki;
	float Kd;	   
	int16_t output;
	void (*Calc)(struct _IncrementalPid *pid);
}IncrementalPid;



#define motor_id208 \
{\
	.SetPoint = 0,\
	.NextPoint = 10000,\
	.Limit = 0,\
	.Kp = 1,\
	.Ki = 0.0f,\
	.Kd = 0,\
	.output = 0,\
	.Calc = &PositionPidCalc,\
}\

typedef struct PositionPid
{
	int16_t SetPoint;
	int16_t NextPoint;
	int16_t Limit;
	float Kp;	
	float Ki;
	float Kd;
	int16_t output;
	void (*Calc)(struct PositionPid *pid);
}PositionPid; 


extern IncrementalPid ID201_PID;
extern IncrementalPid ID202_PID;
extern IncrementalPid ID203_PID;
extern IncrementalPid ID204_PID;
extern IncrementalPid ID207_PID;




void PositionPidCalc(PositionPid *pid);
void IncrementalPidCalc(IncrementalPid * pid);

int constrain(int amt, int low, int high);


#endif

#ifndef _PID_MODIFY_H
#define _PID_MODIFY_H

#include "stm32f4xx.h"

enum

{

  LLAST = 0,

  LAST,

  NOW,

  POSITION_PID,

  DELTA_PID,

};

typedef struct pid_t

{

  float p;

  float i;

  float d;
	
	float vec_kp;
	float posi_kp;
	
	long int position_get;
	int16_t init_position;
  long int position_set;


  int16_t set;

  int16_t get;

  float err[3];



  float pout;

  float iout;

  float dout;

  float out;



  float input_max_err;    //input max err;

  float output_deadband;  //output deadband; 

  

  uint32_t pid_mode;

  uint32_t max_out;

  uint32_t integral_limit;



  void (*f_param_init)(struct pid_t *pid, 

                       uint32_t      pid_mode,

                       uint32_t      max_output,

                       uint32_t      inte_limit,

                       float         p,

                       float         i,

                       float         d);

  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);

 
} pid_t;

extern pid_t CAN1_Motor_PID[8];
extern pid_t CAN2_Motor_PID[8];
extern pid_t CAN1_Motor_Position_PID[1];
extern pid_t power_linit_s;

void PID_struct_init( pid_t*   pid,uint32_t mode,uint32_t maxout, uint32_t intergral_limit,float kp, float ki, float kd);
float pid_calc(pid_t *pid);

#endif


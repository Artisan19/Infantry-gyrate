#include "headfile.h"

IncrementalPid ID201_PID = motor_id201;
IncrementalPid ID202_PID = motor_id201;
IncrementalPid ID203_PID = motor_id201;
IncrementalPid ID204_PID = motor_id201;
IncrementalPid ID207_PID = motor_id201;




/**************************************************************************************************
函数名：IncrementalPidCalc
入口参数：
				IncrementalPid * pid ：增量式PID结构体指针
出口参数：无
功能：传入一个结构体地址，计算出误差值，对误差进行PID控制
说明：
		增量式PID计算公式：out = Kp *(e[n]-e[n-1])  +  Ki*e[n] +Kd*(e[n]  -  2*e[n-1]  +  e[n-2])
*******************************************************************************************************/
void IncrementalPidCalc(IncrementalPid * pid)
{
	  int16_t Error, pError, dError, incrementSpeed, output = 0;
		static int16_t Error_Last, Error_Next;
	
		Error = pid->SetSpeed - pid->NextSpeed;
		pError = Error - Error_Last;
		dError = Error - 2 * Error_Next + Error_Last;
	
		incrementSpeed = pid->Kp * pError 
												+ pid->Ki * Error 
														+ pid->Kd * dError;
		output += incrementSpeed;
		pid->output = constrain(output, -pid->Limit, pid->Limit);
		Error_Last = Error_Next;
		Error_Next = Error;
}

PositionPid ID207;
/**************************************************************************************************
函数名：PositionPidCalc
入口参数：
				PositionPid *pid ：位置式PID结构体指针
出口参数：无
功能：传入一个结构体指针，计算出误差值，对误差进行PID控制
说明：
		位置式PID计算公式：out = Kp * (e[n]  +  Ki * sum（e[n]）  -  Kd*(e[n]  -  e[n-1]))
*******************************************************************************************************/
void PositionPidCalc(PositionPid *pid)
{
    static int16_t Error_Last;
    int16_t Error, iError, dError, output;
		
    Error = pid->SetPoint - pid->NextPoint;
		iError += Error;
		dError = Error - Error_Last;
		Error_Last = Error;
	
		output = Error * pid->Kp
								+ Error * pid->Ki 
										+ dError * pid->Kd;
	
    pid->output = constrain(output, -pid->Limit, pid->Limit);
		
}

/**************************************************************************************************
函数名：constrain
入口参数：
				1. int amt 需要限幅的整型数据
        2. int low 最小值
        3. int high最大值 
出口参数：
				amt:限幅后的参数
功能： 传入一个参数,当这个参数大于最大值或小于最小值时，让它等于最大值或最小值
*******************************************************************************************************/
int constrain(int amt, int low, int high)   //限幅
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}



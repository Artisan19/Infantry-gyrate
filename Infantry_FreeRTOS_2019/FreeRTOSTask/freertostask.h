#ifndef  __FREERTOS_TASK_H
#define  __FREERTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "imu.h"
#include "MotorCAN.h"
#include "event_groups.h"

struct IMU_EXT{
	int16_t Gyo_X;
	int16_t Gyo_Z;
	u8 state;
	float angle;
	uint32_t onlinecnt;
};

void FreeRTOS_init(void);

extern  xSemaphoreHandle DbusParseSem;
extern  xSemaphoreHandle ImuDataSem;

extern  QueueHandle_t Can1ReceiveQueue;
extern  QueueHandle_t Can2ReceiveQueue;

extern IMUDATA  Mpu6500Data;

extern struct IMU_EXT Imu_ext;

extern Motor_SpeedLoopData_t CM_Motor[4];
extern Motor_SpeedLoopData_t Golf_Motor[4];
extern Motor_Posi_A_Speed_LoopData_t YawMotor;
extern Motor_Posi_A_Speed_LoopData_t PitchMotor;
extern Motor_SpeedLoopData_t FireMotor;
extern EventGroupHandle_t CmGimbalOutputEnt;

static void Can1ReceiveTask(void *pvParameters);
#endif


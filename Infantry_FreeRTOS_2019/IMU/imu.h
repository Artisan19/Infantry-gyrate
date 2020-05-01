#ifndef __IMU_H__
#define __IMU_H__
#include "stm32f4xx.h"

typedef struct IMUDATA_T
{
  float Accel_X;   
  float Accel_Y;  
  float Accel_Z;   
  float Gyro_X;    
  float Gyro_Y;    
  float Gyro_Z;    
  float Angle_pitch;
  float Angle_Yaw;
  float Angle_Roll;
  int32_t Gx_offset;
  int32_t Gy_offset;
  int32_t Gz_offset;
  int16_t onlintcntl;
}IMUDATA;


#define M_PI  (float)3.1415926535

void IMU_getValues(volatile float * values);

float invSqrt(float x);

void IMU_getYawPitchRoll(volatile float * angles, volatile float *mygetqval);
	
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

 void GetPitchYawGxGyGz(IMUDATA *mydata);


	
#endif


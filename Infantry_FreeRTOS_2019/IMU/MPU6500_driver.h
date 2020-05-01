#ifndef _MPU6500_DRIVER_H
#define _MPU6500_DRIVER_H

#include <stm32f4xx.h>
#include "sys.h"
#include "imu.h"

#define MPU6500     PFout(6)

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
	
  int16_t Gx_offset;
  int16_t Gy_offset;
  int16_t Gz_offset;
  
}IMUDataTypedef;

void MPU6500Config(void);

uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);

uint8_t MPU6500_Read_Reg(uint8_t const reg);

void MPU6500_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz,IMUDATA *offset);

int16_t Correct_Data_For_GXYZ(int16_t RawDate);

uint8_t IST8310_Init(void);

extern IMUDataTypedef imu_data;
 extern int16_t gy_data;
extern int16_t Gx_offset ,Gy_offset ,Gz_offset ;

float Get_Yaw_Angle(int16_t Gyro_Z);

void MPU6500_InitGyro_offset(IMUDATA *offset);
uint32_t Get_Time_Micros(void);


#endif


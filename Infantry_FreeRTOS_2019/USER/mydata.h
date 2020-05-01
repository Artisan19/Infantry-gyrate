#ifndef __MYDATA_H__
#define __MYDATA_H__

#include <stm32f4xx.h>
typedef struct IMUDATA
{
  float Accel_X;  //转换成实际的X轴加速度，
  float Accel_Y;  //转换成实际的Y轴加速度，
  float Accel_Z;  //转换成实际的Z轴加速度，
  float Gyro_X;   //转换成实际的X轴角加速度，
  float Gyro_Y;   //转换成实际的Y轴角加速度，
  float Gyro_Z;   //转换成实际的Z轴角加速度
	float Angle_pitch;
	float Angle_Yaw;
	float Angle_Roll;
}IMUDATA;

typedef struct Remote
{
  int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

typedef struct Mouse
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	struct Key
{
	uint16_t v;
}Key;

typedef	struct Dubs
{
	Remote remote;
	Mouse mouse;
	Key key;
}Dubs;

typedef struct MotorData
{
	int encoder;  				/*The motor feedback back to the encoder value*/
	int16_t speed;  			/*The speed of the feedback from the motor*/
	int16_t control_val;
}MotorData;

typedef struct _UseMotor
{
	MotorData rm6623_p;				
	MotorData rm6623_y;
	MotorData rm3510_id201;		/*ID is 0x201 of the large Xinjiang motor*/
	MotorData rm3510_id202;	
	MotorData rm3510_id203;		
	MotorData rm3510_id204;		
	MotorData rm2310_fire;
  MotorData rm3510_fire_L;
  MotorData rm3510_fire_R;		
}UseMotor;

typedef	struct Manifold
{
	float  yaw;         		/*The angle value of the pitch axis*/
  float  pitch;						/*The angle value of the yaw axis*/
	int16_t mode;						/*Mode selection: NULL or 640 * 480 or 1280 * 720*/
}Manifold;

typedef	struct MyData
{
	IMUDATA imudata;
	Dubs dbus;
	UseMotor usemotor;
	Manifold manifold;
	float Cortol_ch0;
	float Cortol_ch1;
	float Cortol_ch2;
	float Cortol_ch3;
}MyData;

#endif




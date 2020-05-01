#ifndef __MAIN_H__
#define __MAIN_H__
#include "sys.h"

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#define USART_FOR_PRINT 2    //设置串口打印使用的串口，2 --uart2  3 --uart3  other--uart6

#define SysCoreClock 180

#define RC_NVIC 4

#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5

#define Latitude_At_ShenZhen 22.57025f

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#endif


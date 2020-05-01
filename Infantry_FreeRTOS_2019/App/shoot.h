/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能，其中射击的初始化，以及循环都是在云台任务中调用，故而此文件
  *             不是freeRTOS任务，射击分关闭状态，准备状态，射击状态，以及完成状态
  *             关闭状态是关闭摩擦轮以及激光，准备状态是将子弹拨到微型开关处，射击状
  *             态是将子弹射出，判断微型开关值，进入完成状态，完成状态通过判断一定时间
  *             微型开关无子弹认为已经将子弹射出。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"
#include "MotorCan.h"
#include "RemotDbus.h"
#include "user_lib.h"
#include "Gimbal_Task.h"
#include "Referee.h"
//射击发射开关通道数据
#define Shoot_RC_Channel    1
//云台模式使用的开关通道
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Z
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_X

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 10
//鼠标长按判断
#define PRESS_LONG_TIME 400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define Half_ecd_range 4096
#define ecd_range 8191
//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
//电机编码器值变化成输出轴上的弧度
#define Motor_ECD_TO_ANGLE_36 0.000021305288720633905968306772076277f
#define Motor_ECD_TO_ANGLE_19 0.000040347450657894736842105263157895f
#define FULL_COUNT_36 18
#define FULL_COUNT_19 9.5f
//拨弹速度
#define TRIGGER_SPEED 8.0f
#define Ready_Trigger_Speed 8.0f
#define ADD_BULLET_SPEED 3.0f

#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1

//卡单时间 以及反转时间
#define BLOCK_TIME 500
#define REVERSE_TIME 600
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Eigth 0.39269908169872415480783042290994f
#define PI_Five 0.6283185307179586476925286766559f

//大弹丸摩擦轮电机PID
#define FRIC_R_MOTOR_ANGLE_PID_KP 1000.0f
#define FRIC_R_MOTOR_ANGLE_PID_KI 0.25f
#define FRIC_R_MOTOR_ANGLE_PID_KD 0.0f

#define FRIC_R_MOTOR_ANGLE_PID_MAX_OUT 8000.0f
#define FRIC_R_MOTOR_ANGLE_PID_MAX_IOUT 2000.0f


#define FRIC_L_MOTOR_ANGLE_PID_KP 1000.0f
#define FRIC_L_MOTOR_ANGLE_PID_KI 0.25f
#define FRIC_L_MOTOR_ANGLE_PID_KD 0.0f

#define FRIC_L_MOTOR_ANGLE_PID_MAX_OUT 8000.0f
#define FRIC_L_MOTOR_ANGLE_PID_MAX_IOUT 2000.0f
//供弹电机PID
#define SUPPLY_MOTOR_ANGLE_PID_KP 2000.0f
#define SUPPLY_MOTOR_ANGLE_PID_KI 0.15f
#define SUPPLY_MOTOR_ANGLE_PID_KD 0.0f

#define SUPPLY_MOTOR_ANGLE_PID_MAX_OUT 8000.0f
#define SUPPLY_MOTOR_ANGLE_PID_MAX_IOUT 2000.0f

#define SUPPLY_MOTOR_READY_PID_MAX_OUT 5000.0f
#define SUPPLY_MOTOR_READY_PID_MAX_IOUT 1500.0f

//小弹丸拨弹轮电机PID
#define TRIGGER_15_ANGLE_PID_KP 1800.0f
#define TRIGGER_15_ANGLE_PID_KI 0.001f
#define TRIGGER_15_ANGLE_PID_KD 0.0f

#define TRIGGER_15_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_15_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_15_READY_PID_MAX_OUT 5000.0f
#define TRIGGER_15_READY_PID_MAX_IOUT 2500.0f

//高尔夫拨弹轮电机PID
#define TRIGGER_42_ANGLE_PID_KP 1500.0f
#define TRIGGER_42_ANGLE_PID_KI 0.1f
#define TRIGGER_42_ANGLE_PID_KD 0.0f

#define TRIGGER_42_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_42_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_42_READY_PID_MAX_OUT 5000.0f
#define TRIGGER_42_READY_PID_MAX_IOUT 2500.0f

//摩擦轮电机速度 单位rad/s
#define FRIC_MOTOR_SPEED 8.0f

typedef struct
{
    ramp_function_source_t fric1_ramp;
    ramp_function_source_t fric2_ramp;
    const motor_measure_t *shoot_motor_measure;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    bool_t move_flag;
    uint32_t cmd_time;
    uint32_t run_time;
    bool_t key;
    uint16_t key_time;
    bool_t shoot_done;
    uint8_t shoot_done_time;
    int16_t BulletShootCnt;
    int16_t last_butter_count;
	Referee_Date *referee_data;
} Shoot_Motor_t;

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
	SHOOT_BULLET_15,
	SHOOT_BULLET_42,
    SHOOT_DONE,
	SHOOT_DONE_15,
	SHOOT_DONE_42
} shoot_mode_e;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t *shoot_control_loop(void);

#endif

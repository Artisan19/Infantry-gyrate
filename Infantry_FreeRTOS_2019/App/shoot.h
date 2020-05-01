/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܣ���������ĳ�ʼ�����Լ�ѭ����������̨�����е��ã��ʶ����ļ�
  *             ����freeRTOS��������ֹر�״̬��׼��״̬�����״̬���Լ����״̬
  *             �ر�״̬�ǹر�Ħ�����Լ����⣬׼��״̬�ǽ��ӵ�����΢�Ϳ��ش������״
  *             ̬�ǽ��ӵ�������ж�΢�Ϳ���ֵ���������״̬�����״̬ͨ���ж�һ��ʱ��
  *             ΢�Ϳ������ӵ���Ϊ�Ѿ����ӵ������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
//������俪��ͨ������
#define Shoot_RC_Channel    1
//��̨ģʽʹ�õĿ���ͨ��
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Z
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_X

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME 10
//��곤���ж�
#define PRESS_LONG_TIME 400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME 80
//�����������ֵ��Χ
#define Half_ecd_range 4096
#define ecd_range 8191
//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
//���������ֵ�仯��������ϵĻ���
#define Motor_ECD_TO_ANGLE_36 0.000021305288720633905968306772076277f
#define Motor_ECD_TO_ANGLE_19 0.000040347450657894736842105263157895f
#define FULL_COUNT_36 18
#define FULL_COUNT_19 9.5f
//�����ٶ�
#define TRIGGER_SPEED 8.0f
#define Ready_Trigger_Speed 8.0f
#define ADD_BULLET_SPEED 3.0f

#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TIME 500
#define REVERSE_TIME 600
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Eigth 0.39269908169872415480783042290994f
#define PI_Five 0.6283185307179586476925286766559f

//����Ħ���ֵ��PID
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
//�������PID
#define SUPPLY_MOTOR_ANGLE_PID_KP 2000.0f
#define SUPPLY_MOTOR_ANGLE_PID_KI 0.15f
#define SUPPLY_MOTOR_ANGLE_PID_KD 0.0f

#define SUPPLY_MOTOR_ANGLE_PID_MAX_OUT 8000.0f
#define SUPPLY_MOTOR_ANGLE_PID_MAX_IOUT 2000.0f

#define SUPPLY_MOTOR_READY_PID_MAX_OUT 5000.0f
#define SUPPLY_MOTOR_READY_PID_MAX_IOUT 1500.0f

//С���貦���ֵ��PID
#define TRIGGER_15_ANGLE_PID_KP 1800.0f
#define TRIGGER_15_ANGLE_PID_KI 0.001f
#define TRIGGER_15_ANGLE_PID_KD 0.0f

#define TRIGGER_15_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_15_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_15_READY_PID_MAX_OUT 5000.0f
#define TRIGGER_15_READY_PID_MAX_IOUT 2500.0f

//�߶��򲦵��ֵ��PID
#define TRIGGER_42_ANGLE_PID_KP 1500.0f
#define TRIGGER_42_ANGLE_PID_KI 0.1f
#define TRIGGER_42_ANGLE_PID_KD 0.0f

#define TRIGGER_42_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_42_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_42_READY_PID_MAX_OUT 5000.0f
#define TRIGGER_42_READY_PID_MAX_IOUT 2500.0f

//Ħ���ֵ���ٶ� ��λrad/s
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

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern int16_t *shoot_control_loop(void);

#endif

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

#include "Shoot.h"
#include "MotorCan.h"
#include "gimbal_behaviour.h"
#include "Detect_Task.h"
#include "pid.h"
#include "BasicPeripherals.h"
#include "arm_math.h"
#include "user_lib.h"
#include "FrictionMoterPWM.h"

#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Referee_DispatchTask.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off() fric_off()         //�ر�����Ħ����

#define shoot_laser_on() laser_on()   //���⿪���궨��
#define shoot_laser_off() laser_off() //����رպ궨��

static const RC_ctrl_t *shoot_rc; //ң����ָ��

 PidTypeDef trigger_motor_pid[3],fric_motor_pid[2];         //���PID
 Shoot_Motor_t trigger_motor[3],fric_motor[2];          //�������
 shoot_mode_e shoot_mode = SHOOT_STOP; //���״̬��

float    freOfSmallBullet =  4.5,    freOfBigBullet = 8.0f,speedOfBigBullet = 15.0f;
int16_t  speedOfSmallBullet =  Fric_DOWN;

char only_one_bulet_flag = 0;

extern void getTriggerMotorMeasure(motor_measure_t *motor);



/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void);
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void);
static void LevelToStopBulet(void);
static void SetShootFreAndSpeed(void);
/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3][3] = {{SUPPLY_MOTOR_ANGLE_PID_KP, SUPPLY_MOTOR_ANGLE_PID_KI, SUPPLY_MOTOR_ANGLE_PID_KD},\
											     {TRIGGER_15_ANGLE_PID_KP, TRIGGER_15_ANGLE_PID_KI, TRIGGER_15_ANGLE_PID_KD},\
												 {TRIGGER_42_ANGLE_PID_KP, TRIGGER_42_ANGLE_PID_KI, TRIGGER_42_ANGLE_PID_KD},};
    
												 
    static const fp32 fric_motot_speed_pid[2][3] = {{FRIC_R_MOTOR_ANGLE_PID_KP, FRIC_R_MOTOR_ANGLE_PID_KI, FRIC_R_MOTOR_ANGLE_PID_KD},\
											        {FRIC_L_MOTOR_ANGLE_PID_KP, FRIC_L_MOTOR_ANGLE_PID_KI, FRIC_L_MOTOR_ANGLE_PID_KD}};												 
    //ң����ָ��
    shoot_rc = get_remote_control_point();
    //���ָ��
  //  for(char i = 0;i<3;i++)
	// {
	     trigger_motor[1].shoot_motor_measure = get_shoot_Motor_Measure_Point(2);
		 trigger_motor[1].referee_data = GetRefereeDataPoint();//��ȡ����ϵͳ����
	// }
	 
	 for(char i = 0;i<2;i++)
	 {
	     fric_motor[i].shoot_motor_measure = get_Fric_Motor_Measure_Point(i);	
	 }
	 
    //��ʼ��PID
    PID_Init(&trigger_motor_pid[0], PID_POSITION, &Trigger_speed_pid[0][0], SUPPLY_MOTOR_ANGLE_PID_MAX_OUT, SUPPLY_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_Init(&trigger_motor_pid[1], PID_POSITION, &Trigger_speed_pid[1][0], TRIGGER_15_BULLET_PID_MAX_OUT, TRIGGER_15_BULLET_PID_MAX_IOUT);
	PID_Init(&trigger_motor_pid[2], PID_POSITION, &Trigger_speed_pid[2][0], TRIGGER_42_BULLET_PID_MAX_OUT, TRIGGER_42_BULLET_PID_MAX_IOUT);	 

	PID_Init(&fric_motor_pid[0], PID_POSITION, &fric_motot_speed_pid[0][0], FRIC_R_MOTOR_ANGLE_PID_MAX_OUT, FRIC_R_MOTOR_ANGLE_PID_MAX_IOUT);	 
	PID_Init(&fric_motor_pid[1], PID_POSITION, &fric_motot_speed_pid[1][0], FRIC_L_MOTOR_ANGLE_PID_MAX_OUT, FRIC_L_MOTOR_ANGLE_PID_MAX_IOUT);	 
    //��������
    Shoot_Feedback_Update();
	 
	ramp_init(&trigger_motor[0].fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF); 
    ramp_init(&trigger_motor[0].fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);
	
	
	 for(char i = 0;i<3;i++)
	 {
		trigger_motor[i].ecd_count = 0;
		trigger_motor[i].given_current = 0;
		trigger_motor[i].move_flag = 0;
		trigger_motor[i].set_angle = trigger_motor[i].angle;
		trigger_motor[i].speed = 0.0f;
		trigger_motor[i].speed_set = 0.0f;
		trigger_motor[i].BulletShootCnt = 0;	 
    }
	 for(char i =1;i<2;i++)
	    trigger_motor[i].angle = trigger_motor[i].shoot_motor_measure->ecd * Motor_ECD_TO_ANGLE_36;
	
	  trigger_motor[0].angle = trigger_motor[0].shoot_motor_measure->ecd * Motor_ECD_TO_ANGLE_19;
}
/**
  * @brief          ���ѭ��
  * @author         RM
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t *shoot_control_loop(void)
{
   static  int16_t shoot_CAN_Set_Current[5]; //���ص�canֵ

    Shoot_Set_Mode();        //����״̬��
    Shoot_Feedback_Update(); //��������

    //����״̬����
    if (shoot_mode == SHOOT_BULLET)
    {
		trigger_motor_pid[0].max_out  =  SUPPLY_MOTOR_ANGLE_PID_MAX_OUT;
        trigger_motor_pid[0].max_iout =  SUPPLY_MOTOR_ANGLE_PID_MAX_IOUT;
        trigger_motor_pid[1].max_out  =  TRIGGER_15_BULLET_PID_MAX_OUT;
        trigger_motor_pid[1].max_iout =  TRIGGER_15_BULLET_PID_MAX_IOUT;
        trigger_motor_pid[2].max_out  =  TRIGGER_42_BULLET_PID_MAX_OUT;
        trigger_motor_pid[2].max_iout =  TRIGGER_42_BULLET_PID_MAX_IOUT;
		
		shoot_bullet_control();	
    }
    //�������״̬����
    else if (shoot_mode == SHOOT_DONE)
    {
        shoot_done_control();
    }
    //����׼��״̬����
    else if (shoot_mode == SHOOT_READY)
    {
		trigger_motor_pid[0].max_out  = SUPPLY_MOTOR_READY_PID_MAX_OUT;
        trigger_motor_pid[0].max_iout = SUPPLY_MOTOR_READY_PID_MAX_IOUT;
        trigger_motor_pid[1].max_out  = TRIGGER_15_READY_PID_MAX_OUT;
        trigger_motor_pid[1].max_iout = TRIGGER_15_READY_PID_MAX_IOUT;
        trigger_motor_pid[2].max_out  = TRIGGER_42_READY_PID_MAX_OUT;
        trigger_motor_pid[2].max_iout = TRIGGER_42_READY_PID_MAX_IOUT;
		
        shoot_ready_control();
    }

    if (shoot_mode == SHOOT_STOP)
    {

		trigger_motor[0].fric1_ramp.out = Fric_OFF;
		trigger_motor[0].fric2_ramp.out = Fric_OFF;	
		

        shoot_fric_off();
        shoot_laser_off();
		for(char i = 0;i<5;i++)
           shoot_CAN_Set_Current[i] = 0;
    }
    else
    {
		//С���貦���ֿ���
		if(trigger_motor[1].press_l == 1 || switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
		{
			if(trigger_motor[1].move_flag == 0)
			{
				trigger_motor[1].set_angle = rad_format(trigger_motor[1].set_angle + PI_Eigth);
				trigger_motor[1].cmd_time  = xTaskGetTickCount();
				trigger_motor[1].move_flag = 1;		
			} 
			
			if (rad_format(trigger_motor[1].set_angle - trigger_motor[1].angle) > 0.05f)
			{
				//�Ƕȴﵽ�ж�
				trigger_motor[1].speed_set = freOfSmallBullet;
				trigger_motor[1].run_time  = xTaskGetTickCount();
				//��ת�ж�
				if (trigger_motor[1].run_time - trigger_motor[1].cmd_time > BLOCK_TIME && trigger_motor[1].run_time - trigger_motor[1].cmd_time < 500 + BLOCK_TIME && fabs(trigger_motor[1].speed) < (5.5f * 0.3f))
				{
					trigger_motor[1].speed_set = -Ready_Trigger_Speed;
				}
				else if (trigger_motor[1].run_time - trigger_motor[1].cmd_time > 500 + BLOCK_TIME || fabs(trigger_motor[1].speed) > REVERSE_SPEED_LIMIT)
				{
					trigger_motor[1].cmd_time = xTaskGetTickCount();
				}
			}
			else
			{
				trigger_motor[1].move_flag = 0;
			}		   
		}
		else
		{
			trigger_motor[1].set_angle = trigger_motor[1].angle;
			trigger_motor[1].move_flag = 0;
			trigger_motor[1].speed_set = 0;	     			
		}
		
		//���ݵȼ���׼��������ʱֹͣ������
		LevelToStopBulet();
		
		//�����л�������Ƶ
		SetShootFreAndSpeed();
		
        //Ħ����pwm
        static uint16_t fric_pwm1 = Fric_OFF;
        static uint16_t fric_pwm2 = Fric_OFF;
		
		//����3510Ħ����
        fric_motor[0].speed_set =  -speedOfBigBullet;
		fric_motor[1].speed_set =   speedOfBigBullet;

        shoot_laser_on();       //���⿪��


        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&trigger_motor[0].fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

        if(trigger_motor[0].fric1_ramp.out == trigger_motor[0].fric1_ramp.max_value)
        {
            ramp_calc(&trigger_motor[0].fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }

        if( trigger_motor[0].fric2_ramp.out != trigger_motor[0].fric2_ramp.max_value)
        {
            trigger_motor[0].speed_set = 0.0f;
        }


//����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
//        static uint16_t up_time = 0;
//        if (trigger_motor[0].press_r)
//        {
//            up_time = UP_ADD_TIME;
//        }

//        if (up_time > 0)
//        {
            trigger_motor[0].fric1_ramp.max_value = speedOfSmallBullet;//Fric_UP;
            trigger_motor[0].fric2_ramp.max_value = speedOfSmallBullet;//Fric_UP;
//            up_time--;
//        }
//        else
//        {
//            trigger_motor[0].fric1_ramp.max_value = Fric_DOWN;
//            trigger_motor[0].fric2_ramp.max_value = Fric_DOWN;
//        }

        fric_pwm1 = (uint16_t)(trigger_motor[0].fric1_ramp.out);
        fric_pwm2 = (uint16_t)(trigger_motor[0].fric2_ramp.out);

        shoot_fric1_on(fric_pwm1);
        shoot_fric2_on(fric_pwm2);

        //���㲦���ֵ��PID
		for(char i=0;i<3;i++)
			PID_Calc(&trigger_motor_pid[i], trigger_motor[i].speed, trigger_motor[i].speed_set);
         //����Ħ���ֵ��PID
		for(char i=0;i<2;i++)
		   PID_Calc(&fric_motor_pid[i], fric_motor[i].speed, fric_motor[i].speed_set);
       
		for(char i=0;i<3;i++)
		{
		   trigger_motor[i].given_current = (int16_t)(trigger_motor_pid[i].out);
           shoot_CAN_Set_Current[i] = trigger_motor[i].given_current;		
		}
		for(char i= 0;i<2;i++)
		{
			fric_motor[i].given_current = (int16_t)(fric_motor_pid[i].out);
			shoot_CAN_Set_Current[i+3] = fric_motor[i].given_current;
		}		
    }

    return shoot_CAN_Set_Current;
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP) || (shoot_rc->key.v & SHOOT_OFF_KEYBOARD))
    {
        shoot_mode = SHOOT_STOP;
    }

    //�����е��� ����ʹ�ü��̿���Ħ����
    if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY;
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_mode == SHOOT_READY)
    {
        shoot_mode = SHOOT_STOP;
    }

    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_mode = SHOOT_STOP;
    }

    if (shoot_mode == SHOOT_READY)
    {
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_down(last_s)) ||(trigger_motor[0].press_r && trigger_motor[0].last_press_r == 0))
        {
            shoot_mode = SHOOT_BULLET;
			trigger_motor[2].last_butter_count = trigger_motor[2].BulletShootCnt;
        }
			
		
        //��곤��һֱ�������״̬ ��������
        if (((trigger_motor[0].press_r_time == PRESS_LONG_TIME)) || (trigger_motor[0].rc_s_time == RC_S_LONG_TIME))
        {
            if (shoot_mode != SHOOT_DONE && trigger_motor[0].key == SWITCH_TRIGGER_ON)
            {
                shoot_mode = SHOOT_BULLET;
            }
        }	
    }
	
    last_s = shoot_rc->rc.s[Shoot_RC_Channel];
}
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1[3] = {0.0f};
    static fp32 speed_fliter_2[3] = {0.0f};
    static fp32 speed_fliter_3[3] = {0.0f};

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3][3] = {{1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f},\
										  {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f},\
										  {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f}};

    //���׵�ͨ�˲�
    for(char i = 0;i<3;i++)
	{
		speed_fliter_1[i] = speed_fliter_2[i];
		speed_fliter_2[i] = speed_fliter_3[i];
		speed_fliter_3[i] = speed_fliter_2[i] * fliter_num[i][0] + speed_fliter_1[i] * fliter_num[i][1] + (trigger_motor[i].shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[i][2];
		trigger_motor[i].speed = speed_fliter_3[i];	
	}

	for(char i=0;i<2 ;i++)
	{
	  fric_motor[i].speed = fric_motor[i].shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED;
	}
	
    for(char i =0;i<3;i++)
	{
		//���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
		if (trigger_motor[i].shoot_motor_measure->ecd - trigger_motor[i].shoot_motor_measure->last_ecd > Half_ecd_range)
		{
			trigger_motor[i].ecd_count--;
		}
		else if (trigger_motor[i].shoot_motor_measure->ecd - trigger_motor[i].shoot_motor_measure->last_ecd < -Half_ecd_range)
		{
			trigger_motor[i].ecd_count++;
		}

		//΢������
		trigger_motor[i].key = Butten_Trig_Pin;
		//��갴��
		trigger_motor[i].last_press_l = trigger_motor[i].press_l;
		trigger_motor[i].last_press_r = trigger_motor[i].press_r;
		trigger_motor[i].press_l = shoot_rc->mouse.press_l;
		trigger_motor[i].press_r = shoot_rc->mouse.press_r;	

		//������ʱ
		if (trigger_motor[i].press_l)
		{
			if (trigger_motor[i].press_l_time < PRESS_LONG_TIME)
			{
				trigger_motor[i].press_l_time++;
			}
		}
		else
		{
			trigger_motor[i].press_l_time = 0;
		}	

		if (trigger_motor[i].press_r)
		{
			if (trigger_motor[i].press_r_time < PRESS_LONG_TIME)
			{
				trigger_motor[i].press_r_time++;
			}
		}
		else
		{
			trigger_motor[i].press_r_time = 0;
		}
		//��������µ�ʱ���ʱ
		if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
		{

			if (trigger_motor[i].rc_s_time < RC_S_LONG_TIME)
			{
				trigger_motor[i].rc_s_time++;
			}
		}
		else
		{
			trigger_motor[i].rc_s_time = 0;
		}
		
			//��������µ�ʱ���ʱ
		if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
		{

			if (trigger_motor[i].rc_s_time < RC_S_LONG_TIME)
			{
				trigger_motor[i].rc_s_time++;
			}
		}
		else
		{
			trigger_motor[i].rc_s_time = 0;
		}	
	}
	
		//���������Ƕ� ��Χ-3.14~3.14
	for(char i=1;i<3;i++)
	{
		if (trigger_motor[i].ecd_count == FULL_COUNT_36)
		{
			trigger_motor[i].ecd_count = -(FULL_COUNT_36 - 1);
		}
		else if (trigger_motor[i].ecd_count == -FULL_COUNT_36)
		{
			trigger_motor[i].ecd_count = FULL_COUNT_36 - 1;
		}		
		trigger_motor[i].angle = (trigger_motor[i].ecd_count * ecd_range + trigger_motor[i].shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE_36;
	}
		
	if (trigger_motor[0].ecd_count == FULL_COUNT_19)
	{
		trigger_motor[0].ecd_count = -(FULL_COUNT_19 - 1);
	}
	else if (trigger_motor[0].ecd_count == -FULL_COUNT_19)
	{
		trigger_motor[0].ecd_count = FULL_COUNT_19 - 1;
	}
	trigger_motor[0].angle = (trigger_motor[0].ecd_count * ecd_range + trigger_motor[0].shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE_19;
}
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
	static  char bullet_empty = 0; 
    //�ӵ�����ж�
    if (trigger_motor[2].key == SWITCH_TRIGGER_OFF)
    {
        trigger_motor[2].shoot_done = 1;
        trigger_motor[2].shoot_done_time = 0;

        shoot_mode = SHOOT_DONE;
        trigger_motor[2].set_angle = trigger_motor[2].angle;
	}


    //ÿ�β��� 1/4PI�ĽǶ�
    if (trigger_motor[2].move_flag == 0 && shoot_mode == SHOOT_BULLET)
    {
        trigger_motor[2].set_angle = rad_format(trigger_motor[2].set_angle + PI_Four);
        trigger_motor[2].cmd_time  = xTaskGetTickCount();
        trigger_motor[2].move_flag = 1;
			  
    }	
	

	if (rad_format(trigger_motor[2].set_angle - trigger_motor[2].angle) > 0.05f)
	{
		//û����һֱ������ת�ٶ�
		trigger_motor[2].speed_set = freOfBigBullet;
		trigger_motor[2].run_time  = xTaskGetTickCount();

		//��ת�ж�
		if (trigger_motor[2].run_time - trigger_motor[2].cmd_time > BLOCK_TIME && trigger_motor[2].run_time - trigger_motor[2].cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor[2].speed) < TRIGGER_SPEED * 0.5f)
		{
			trigger_motor[2].speed_set = -TRIGGER_SPEED;
		}
		else if (trigger_motor[2].run_time - trigger_motor[2].cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor[0].speed) > REVERSE_SPEED_LIMIT)
		{
			trigger_motor[2].cmd_time = xTaskGetTickCount();
		}
	}
	else
	{
		trigger_motor[2].move_flag = 0;	
		//ֻ��һ���ӵ�
		if(trigger_motor[2].key == SWITCH_TRIGGER_ON)
		{
		    bullet_empty++;
		}
		
		if(bullet_empty > 5)
		{			
			bullet_empty = 0;
			only_one_bulet_flag = 1;
			shoot_mode = SHOOT_READY;
		}
	}
}
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void)
{
	//���һ����������������ٶ�����
    trigger_motor[2].speed_set = 0.0f;
    //�������жϣ��ж�΢������һ��ʱ�����ӵ�
    if (trigger_motor[2].key == SWITCH_TRIGGER_OFF)
    {
        if (trigger_motor[2].shoot_done_time < SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor[2].shoot_done_time++;
        }
        else if (trigger_motor[2].shoot_done_time == SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor[2].BulletShootCnt++;
            shoot_mode = SHOOT_READY;
        }
    }
    else
    {
        shoot_mode = SHOOT_BULLET;
    }
}
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void)
{
 static char add_bullet_num = 0,add_bullet_flag = 0,only_one_cnt = 0;
    if (trigger_motor[2].shoot_done)
    {
        trigger_motor[2].shoot_done = 0;
    }

    if (trigger_motor[2].key == SWITCH_TRIGGER_ON && only_one_bulet_flag == 0)
    {
        //�ж��ӵ�����΢�����ش�
        trigger_motor[2].set_angle = trigger_motor[2].angle;
        trigger_motor_pid[2].out = 0.0f;
        trigger_motor_pid[2].Iout = 0.0f;
        trigger_motor[2].speed_set = 0.0f;		
        trigger_motor[2].move_flag = 0;
        trigger_motor[2].key_time = 0;
		
		only_one_bulet_flag = 0;
		add_bullet_flag = 0;
		add_bullet_num  = 0;
		trigger_motor[0].speed_set  = 0;
		trigger_motor[0].move_flag = 0;
		trigger_motor[0].set_angle = trigger_motor[0].angle;
    }
    else if (trigger_motor[2].key == SWITCH_TRIGGER_OFF || only_one_bulet_flag == 1)
    {     
		if(trigger_motor[2].move_flag == 0)
		{
			trigger_motor[2].set_angle = rad_format(trigger_motor[2].set_angle + PI_Four);
			trigger_motor[2].cmd_time  = xTaskGetTickCount();
			trigger_motor[2].move_flag = 1;		
		}
		
        if (rad_format(trigger_motor[2].set_angle - trigger_motor[2].angle) > 0.05f)
        {		
            //�Ƕȴﵽ�ж�
            trigger_motor[2].speed_set = ADD_BULLET_SPEED;
            trigger_motor[2].run_time = xTaskGetTickCount();
            //��ת�ж�
            if (trigger_motor[2].run_time - trigger_motor[2].cmd_time > BLOCK_TIME && trigger_motor[2].run_time - trigger_motor[2].cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor[2].speed) < ADD_BULLET_SPEED*0.5f)
            {
                trigger_motor[2].speed_set = -Ready_Trigger_Speed;
            }
            else if (trigger_motor[2].run_time - trigger_motor[2].cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor[2].speed) > REVERSE_SPEED_LIMIT)
            {
                trigger_motor[2].cmd_time = xTaskGetTickCount();
            }
        }
        else
        {
			trigger_motor[2].move_flag = 0;	
            add_bullet_num++;
            if(add_bullet_num >= 3)
			{
			  
			   add_bullet_num = 0;
			   add_bullet_flag = 1;
			}				      
		}	
	 
		if(add_bullet_flag == 1)
		{
			if(trigger_motor[0].move_flag == 0)
			{
				trigger_motor[0].set_angle = rad_format(trigger_motor[0].set_angle + PI_Five);
				trigger_motor[0].cmd_time  = xTaskGetTickCount();
				trigger_motor[0].move_flag = 1;		
			} 
			
			if (rad_format(trigger_motor[0].set_angle - trigger_motor[0].angle) > 0.05f)
			{
				//�Ƕȴﵽ�ж�
				trigger_motor[0].speed_set = 2.05f;//ADD_BULLET_SPEED; 
				trigger_motor[0].run_time = xTaskGetTickCount();
				//��ת�ж�
				if (trigger_motor[0].run_time - trigger_motor[0].cmd_time > BLOCK_TIME && trigger_motor[0].run_time - trigger_motor[0].cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor[0].speed) < 1.5f)
				{
					trigger_motor[0].speed_set = -Ready_Trigger_Speed;
				}
				else if (trigger_motor[0].run_time - trigger_motor[0].cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor[0].speed) > REVERSE_SPEED_LIMIT)
				{
					trigger_motor[0].cmd_time = xTaskGetTickCount();
				}
			}
			else
			{
				trigger_motor[0].move_flag = 0;
				if( only_one_bulet_flag == 1)
				{
					only_one_cnt++;
				}
				if(only_one_cnt >= 10)
				{
				   only_one_bulet_flag = 0;
				}
			}
			
		}
    }
}


static void LevelToStopBulet()
{
			if(trigger_motor[1].referee_data->power_heat_data.shooter_heat0 > trigger_motor[1].referee_data->game_robot_state.shooter_heat0_cooling_limit-100)
			{
				trigger_motor[1].speed_set = 0;
			}	
}

static uint8_t fire_mode  = 0;
static void SetShootFreAndSpeed()
{
	//C�����ٸ���Ƶģʽ
   if(shoot_rc->key.v == KEY_PRESSED_OFFSET_C)
   {
	   fire_mode = 0;
	   freOfSmallBullet   =  8.5f;
	   speedOfSmallBullet = 1150; //1150;//1230;
	   freOfBigBullet     =  11.0f;
	   speedOfBigBullet   =  15.0f;          
   }
   else if(shoot_rc->key.v == KEY_PRESSED_OFFSET_V)
   {
	   fire_mode = 1;
	   //V������������Ƶģʽ
 	   freOfSmallBullet   =  6.5f;
	   speedOfSmallBullet =  1200;//1200;//1250;
	   freOfBigBullet     =  9.5f;
	   speedOfBigBullet   =  15.0f;   
   }
   else if(shoot_rc->key.v == KEY_PRESSED_OFFSET_B)
   {
	   fire_mode = 2;
	   //B�������ٵ���Ƶģʽ
 	   freOfSmallBullet   =  4.5f;
	   speedOfSmallBullet = 1230;// 1230;//1300;
	   freOfBigBullet     =  8.5f;
	   speedOfBigBullet   =  15.0f;		
   }
}

uint8_t getFireMode()
{
	return fire_mode;
}
	

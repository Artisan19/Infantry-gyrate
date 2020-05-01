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

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义

static const RC_ctrl_t *shoot_rc; //遥控器指针

 PidTypeDef trigger_motor_pid[3],fric_motor_pid[2];         //电机PID
 Shoot_Motor_t trigger_motor[3],fric_motor[2];          //射击数据
 shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机

float    freOfSmallBullet =  4.5,    freOfBigBullet = 8.0f,speedOfBigBullet = 15.0f;
int16_t  speedOfSmallBullet =  Fric_DOWN;

char only_one_bulet_flag = 0;

extern void getTriggerMotorMeasure(motor_measure_t *motor);



/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
/**
  * @brief          射击完成控制，判断微动开关一段时间无子弹来判断一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void);
/**
  * @brief          射击准备控制，将子弹送到微动开关处，
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void);
static void LevelToStopBulet(void);
static void SetShootFreAndSpeed(void);
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3][3] = {{SUPPLY_MOTOR_ANGLE_PID_KP, SUPPLY_MOTOR_ANGLE_PID_KI, SUPPLY_MOTOR_ANGLE_PID_KD},\
											     {TRIGGER_15_ANGLE_PID_KP, TRIGGER_15_ANGLE_PID_KI, TRIGGER_15_ANGLE_PID_KD},\
												 {TRIGGER_42_ANGLE_PID_KP, TRIGGER_42_ANGLE_PID_KI, TRIGGER_42_ANGLE_PID_KD},};
    
												 
    static const fp32 fric_motot_speed_pid[2][3] = {{FRIC_R_MOTOR_ANGLE_PID_KP, FRIC_R_MOTOR_ANGLE_PID_KI, FRIC_R_MOTOR_ANGLE_PID_KD},\
											        {FRIC_L_MOTOR_ANGLE_PID_KP, FRIC_L_MOTOR_ANGLE_PID_KI, FRIC_L_MOTOR_ANGLE_PID_KD}};												 
    //遥控器指针
    shoot_rc = get_remote_control_point();
    //电机指针
  //  for(char i = 0;i<3;i++)
	// {
	     trigger_motor[1].shoot_motor_measure = get_shoot_Motor_Measure_Point(2);
		 trigger_motor[1].referee_data = GetRefereeDataPoint();//获取裁判系统数据
	// }
	 
	 for(char i = 0;i<2;i++)
	 {
	     fric_motor[i].shoot_motor_measure = get_Fric_Motor_Measure_Point(i);	
	 }
	 
    //初始化PID
    PID_Init(&trigger_motor_pid[0], PID_POSITION, &Trigger_speed_pid[0][0], SUPPLY_MOTOR_ANGLE_PID_MAX_OUT, SUPPLY_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_Init(&trigger_motor_pid[1], PID_POSITION, &Trigger_speed_pid[1][0], TRIGGER_15_BULLET_PID_MAX_OUT, TRIGGER_15_BULLET_PID_MAX_IOUT);
	PID_Init(&trigger_motor_pid[2], PID_POSITION, &Trigger_speed_pid[2][0], TRIGGER_42_BULLET_PID_MAX_OUT, TRIGGER_42_BULLET_PID_MAX_IOUT);	 

	PID_Init(&fric_motor_pid[0], PID_POSITION, &fric_motot_speed_pid[0][0], FRIC_R_MOTOR_ANGLE_PID_MAX_OUT, FRIC_R_MOTOR_ANGLE_PID_MAX_IOUT);	 
	PID_Init(&fric_motor_pid[1], PID_POSITION, &fric_motot_speed_pid[1][0], FRIC_L_MOTOR_ANGLE_PID_MAX_OUT, FRIC_L_MOTOR_ANGLE_PID_MAX_IOUT);	 
    //更新数据
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
  * @brief          射击循环
  * @author         RM
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t *shoot_control_loop(void)
{
   static  int16_t shoot_CAN_Set_Current[5]; //返回的can值

    Shoot_Set_Mode();        //设置状态机
    Shoot_Feedback_Update(); //更新数据

    //发射状态控制
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
    //发射完成状态控制
    else if (shoot_mode == SHOOT_DONE)
    {
        shoot_done_control();
    }
    //发射准备状态控制
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
		//小弹丸拨弹轮控制
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
				//角度达到判断
				trigger_motor[1].speed_set = freOfSmallBullet;
				trigger_motor[1].run_time  = xTaskGetTickCount();
				//堵转判断
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
		
		//根据等级在准备超热量时停止拨弹轮
		LevelToStopBulet();
		
		//按键切换射速射频
		SetShootFreAndSpeed();
		
        //摩擦轮pwm
        static uint16_t fric_pwm1 = Fric_OFF;
        static uint16_t fric_pwm2 = Fric_OFF;
		
		//开启3510摩擦轮
        fric_motor[0].speed_set =  -speedOfBigBullet;
		fric_motor[1].speed_set =   speedOfBigBullet;

        shoot_laser_on();       //激光开启


        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&trigger_motor[0].fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

        if(trigger_motor[0].fric1_ramp.out == trigger_motor[0].fric1_ramp.max_value)
        {
            ramp_calc(&trigger_motor[0].fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }

        if( trigger_motor[0].fric2_ramp.out != trigger_motor[0].fric2_ramp.max_value)
        {
            trigger_motor[0].speed_set = 0.0f;
        }


//鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
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

        //计算拨弹轮电机PID
		for(char i=0;i<3;i++)
			PID_Calc(&trigger_motor_pid[i], trigger_motor[i].speed, trigger_motor[i].speed_set);
         //计算摩擦轮电机PID
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
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP) || (shoot_rc->key.v & SHOOT_OFF_KEYBOARD))
    {
        shoot_mode = SHOOT_STOP;
    }

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_mode == SHOOT_READY)
    {
        shoot_mode = SHOOT_STOP;
    }

    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_mode = SHOOT_STOP;
    }

    if (shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_down(last_s)) ||(trigger_motor[0].press_r && trigger_motor[0].last_press_r == 0))
        {
            shoot_mode = SHOOT_BULLET;
			trigger_motor[2].last_butter_count = trigger_motor[2].BulletShootCnt;
        }
			
		
        //鼠标长按一直进入射击状态 保持连发
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
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1[3] = {0.0f};
    static fp32 speed_fliter_2[3] = {0.0f};
    static fp32 speed_fliter_3[3] = {0.0f};

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3][3] = {{1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f},\
										  {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f},\
										  {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f}};

    //二阶低通滤波
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
		//电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
		if (trigger_motor[i].shoot_motor_measure->ecd - trigger_motor[i].shoot_motor_measure->last_ecd > Half_ecd_range)
		{
			trigger_motor[i].ecd_count--;
		}
		else if (trigger_motor[i].shoot_motor_measure->ecd - trigger_motor[i].shoot_motor_measure->last_ecd < -Half_ecd_range)
		{
			trigger_motor[i].ecd_count++;
		}

		//微动开关
		trigger_motor[i].key = Butten_Trig_Pin;
		//鼠标按键
		trigger_motor[i].last_press_l = trigger_motor[i].press_l;
		trigger_motor[i].last_press_r = trigger_motor[i].press_r;
		trigger_motor[i].press_l = shoot_rc->mouse.press_l;
		trigger_motor[i].press_r = shoot_rc->mouse.press_r;	

		//长按计时
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
		//射击开关下档时间计时
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
		
			//射击开关下档时间计时
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
	
		//计算输出轴角度 范围-3.14~3.14
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
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
	static  char bullet_empty = 0; 
    //子弹射出判断
    if (trigger_motor[2].key == SWITCH_TRIGGER_OFF)
    {
        trigger_motor[2].shoot_done = 1;
        trigger_motor[2].shoot_done_time = 0;

        shoot_mode = SHOOT_DONE;
        trigger_motor[2].set_angle = trigger_motor[2].angle;
	}


    //每次拨动 1/4PI的角度
    if (trigger_motor[2].move_flag == 0 && shoot_mode == SHOOT_BULLET)
    {
        trigger_motor[2].set_angle = rad_format(trigger_motor[2].set_angle + PI_Four);
        trigger_motor[2].cmd_time  = xTaskGetTickCount();
        trigger_motor[2].move_flag = 1;
			  
    }	
	

	if (rad_format(trigger_motor[2].set_angle - trigger_motor[2].angle) > 0.05f)
	{
		//没到达一直设置旋转速度
		trigger_motor[2].speed_set = freOfBigBullet;
		trigger_motor[2].run_time  = xTaskGetTickCount();

		//堵转判断
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
		//只有一个子弹
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
  * @brief          射击完成控制，判断微动开关一段时间无子弹来判断一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void)
{
	//完成一次射击，将拨弹轮速度清零
    trigger_motor[2].speed_set = 0.0f;
    //射击完成判断，判断微动开关一段时间无子弹
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
  * @brief          射击准备控制，将子弹送到微动开关处，
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
        //判断子弹到达微动开关处
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
            //角度达到判断
            trigger_motor[2].speed_set = ADD_BULLET_SPEED;
            trigger_motor[2].run_time = xTaskGetTickCount();
            //堵转判断
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
				//角度达到判断
				trigger_motor[0].speed_set = 2.05f;//ADD_BULLET_SPEED; 
				trigger_motor[0].run_time = xTaskGetTickCount();
				//堵转判断
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
	//C键低速高射频模式
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
	   //V键中射速中射频模式
 	   freOfSmallBullet   =  6.5f;
	   speedOfSmallBullet =  1200;//1200;//1250;
	   freOfBigBullet     =  9.5f;
	   speedOfBigBullet   =  15.0f;   
   }
   else if(shoot_rc->key.v == KEY_PRESSED_OFFSET_B)
   {
	   fire_mode = 2;
	   //B键高射速低射频模式
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
	

#include "gimbal_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h" 
#include "INS_task.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "MotorCAN.h"
#include "shoot.h"

		int16_t gimbal_set_current[4];

//电机编码值规整 0—8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }
	
static void J_scope_gimbal_test(void);

//发送的can 指令
 int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;

//云台控制所有相关数据
 Gimbal_Control_t gimbal_control;
//初始化云台 
 static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
 //初始化云台位置环PID参数
 static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
//将pid误差值，设定值，输出值清零
 static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
//更新云台控制参数
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//根据拨杆选择云台控制模式，再根据云台状态机选择电机状态机	
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);	
//云台状态切换保存，用于状态切换过渡
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//云台控制状态使用不同控制pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);

uint8_t time_delay = 1; 	
void GIMBAL_task(void *pvParameters)
{

	
	int16_t *shoot_out;
	TickType_t PreviousWakeTime;
	const TickType_t TimerIncrement =  pdMS_TO_TICKS(GIMBAL_CONTROL_TIME);
	PreviousWakeTime = xTaskGetTickCount();
	
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);	
	 //云台初始化
    GIMBAL_Init(&gimbal_control);
	//射击初始化
    shoot_init();
	
	//判断电机是否都上线
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE)/* || toe_is_error(TriggerMotorTOE) */)
    {
        vTaskDelay(GIMBAL_CONTROL_TIME);
        GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
    }
	
	while(1)  
	{
		 GIMBAL_Set_Mode(&gimbal_control);                    //设置云台控制模式
		 GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //控制模式切换 控制数据过渡
		 GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
		 GIMBAL_Set_Contorl(&gimbal_control);                 //设置云台控制量
		 GIMBAL_Control_loop(&gimbal_control);                //云台控制PID计算
		 shoot_out = shoot_control_loop();
      		

		int16_t shoot_motot_set[4] = {0};
		//云台在遥控器掉线状态即relax 状态，can指令为0，不使用current设置为零的方法，是保证遥控器掉线一定使得云台停止
//        if ((toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE)/* && toe_is_error(TriggerMotorTOE)*/))
//        {
            if (toe_is_error(DBUSTOE))
            {			
				 for(char i = 0;i<4;i++)
				  {
						gimbal_set_current[i]  = 0;
				  } 
            }
            else
            {
				#if YAW_TURN
				   gimbal_set_current[0] = - gimbal_control.gimbal_yaw_motor.given_current; 
				#else
				   gimbal_set_current[0] = gimbal_control.gimbal_yaw_motor.given_current; 
				#endif
				
				#if PITCH_TURN
				  gimbal_set_current[1] = 0;// - gimbal_control.gimbal_pitch_motor.given_current;
				#else 
				  gimbal_set_current[1] =   gimbal_control.gimbal_pitch_motor.given_current;
				#endif
				gimbal_set_current[2] = shoot_out[1];
				gimbal_set_current[3] = 0;
				
				shoot_motot_set[0] = shoot_out[3]; 
				shoot_motot_set[1] = shoot_out[4];
				shoot_motot_set[2] = shoot_out[1];
				shoot_motot_set[3] = shoot_out[0];
            }
			
			CanSendMess(CAN1,SEND_ID205_207,gimbal_set_current); 
			CanSendMess(CAN2,SEND_ID201_204,shoot_motot_set); 
	//   }
		//vTaskDelay(time_delay);
      vTaskDelayUntil(&PreviousWakeTime,TimerIncrement);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif	

#if (GIMBAL_TEST_MODE )	
         J_scope_gimbal_test();
#endif	   
    }
}
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}


static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set =  GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
 //   gimbal_motor->motor_gyro_set = 0;
	gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
extern 	uint8_t getFireMode();

//云台控制状态使用不同控制pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
	static char last_aim_flag = 0,auto_flag = 0;

	
    if (gimbal_control_loop == NULL)
    {
        return;
    }
    //yaw不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro角度控制	 
		if(gimbal_control_loop->auto_aim_flag == 1 &&  last_aim_flag == 0  && auto_flag == 0 )
		{
		     auto_flag = 1;
		}
		else if(gimbal_control_loop->auto_aim_flag == 0 && last_aim_flag == 1 && auto_flag == 1)
		{
			//发生模式切换时，将设定值设为反馈值
		    auto_flag = 0;
			
			   gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kp = YAW_GYRO_ABSOLUTE_PID_KP ;
				 gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp = YAW_SPEED_PID_KP;
			
		    gimbal_control_loop->gimbal_yaw_motor.absolute_angle_set =  gimbal_control_loop->gimbal_yaw_motor.absolute_angle; 				
		}
		
			
		if(auto_flag == 1)
		{		
			gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kp = 15.0f ;	
			gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp = 12000.0f;	
			
			
			static float yaw_add =  2.3f /180.0f * 3.14f;//3.0f /180.0f * 3.14f;
			if(getFireMode() == 2)
			{
			   yaw_add =  2.1f /180.0f * 3.14f;//3.0f /180.0f * 3.14f;
			}
			else
			{
				yaw_add =  2.3f /180.0f * 3.14f;//3.0f /180.0f * 3.14f;
			}
				
				
				
			gimbal_control_loop->gimbal_yaw_motor.absolute_angle_set = 0.0f;
			//识别到的坐标系可能跟装甲的位置存在偏差，所以需加上一个偏移量，未识别到目标时需把该量清零
			gimbal_control_loop->gimbal_yaw_motor.absolute_angle = rad_format(gimbal_control_loop->autodata->YawAxiaAngle + yaw_add);	
			
			if(gimbal_control_loop->autodata->YawAxiaAngle == 0.0f)
			{
				gimbal_control_loop->gimbal_yaw_motor.absolute_angle = 0.0f;
			}		
		}
			
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde角度控制zx
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }

    //pitch不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro角度控制
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //gyro角度控制	 

	   if(gimbal_control_loop->auto_aim_flag == 0 && last_aim_flag == 1)
		{
		   gimbal_control_loop->gimbal_pitch_motor.relative_angle_set =  gimbal_control_loop->gimbal_pitch_motor.relative_angle; 
		}
		

		if(auto_flag == 1)
		{		
            //辅助射击模式下反馈置由妙算结算的坐标提供，设定值为零			
			
			static float pitch_add =  4.20f/180.0f * 3.14f;//6.0f/180.0f * 3.14f;//2.0f/180.0f * 3.14f;
			
			if(getFireMode() == 0) 
					pitch_add = 3.10f/180.0f * 3.14f;
			else if(getFireMode() == 1)
					pitch_add = 3.60f/180.0f * 3.14f;
			else
					pitch_add = 3.40f/180.0f * 3.14f;
			gimbal_control_loop->gimbal_pitch_motor.relative_angle_set = 0.0f;
			gimbal_control_loop->gimbal_pitch_motor.relative_angle = -rad_format(gimbal_control_loop->autodata->PitchAxiaAngle   + pitch_add);	
		
			if(gimbal_control_loop->autodata->PitchAxiaAngle == 0.0f)
			{
				gimbal_control_loop->gimbal_pitch_motor.relative_angle = 0.0f;
			}
		}		
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
	
	last_aim_flag = gimbal_control_loop->auto_aim_flag;
}


//陀螺仪 控制量限制
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            //计算出一个最大的添加角度，
          //  add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
					
					//如果是小陀螺模式，就用下面这一句
					add= gimbal_motor->relative_angle -bias_angle;
        }
    }
		//max_relative_angle最大机械角度
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
           // add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
			   	add=	gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}

//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
    //yaw电机模式控制
    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    //pitch电机模式控制
    if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

//云台状态切换保存，用于状态切换过渡
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}


static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}


//初始化pid 数据指针
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{

    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //电机数据指针获取
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();  
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point(); 
    //陀螺仪数据指针获取
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    //清除所有PID
    gimbal_total_pid_clear(gimbal_init);

    //更新云台反馈数据
    GIMBAL_Feedback_Update(gimbal_init);
	
	//初始化妙算数据
	gimbal_init->autodata = GetAutoDataPoint();
	

    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;


    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
}

static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_ROLL_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = *(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);

    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro =  -*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
	//-(arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET))\
                                                            - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)));
}


/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @author         RM
  * @param[in]      yaw 中值
  * @param[in]      pitch 中值
  * @param[in]      yaw 最大相对角度
  * @param[in]      yaw 最小相对角度
  * @param[in]      pitch 最大相对角度
  * @param[in]      pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}




//校准计算，相对最大角度，云台中值
static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_ecd = 0;//temp_max_ecd = 0, temp_min_ecd = 0, 

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

//    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
//    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
//    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
//    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

//    ECD_Format(temp_max_ecd);
//    ECD_Format(temp_min_ecd);

//    temp_ecd = temp_max_ecd - temp_min_ecd;

//    if (temp_ecd > Half_ecd_range)
//    {
//        temp_ecd -= ecd_range;
//    }
//    else if (temp_ecd < -Half_ecd_range)
//    {
//        temp_ecd += ecd_range;
//    }

//    if (temp_max_ecd > temp_min_ecd)
//    {
//        temp_min_ecd += ecd_range;
//    }

//    temp_ecd = temp_max_ecd - temp_ecd / 2;

//    ECD_Format(temp_ecd);

//    *pitch_offset = temp_ecd;

//    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
//    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);


    temp_ecd = gimbal_cali->max_pitch_ecd - gimbal_cali->min_pitch_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_pitch_ecd - (temp_ecd / 2);
	
    ECD_Format(temp_ecd);
    *pitch_offset = temp_ecd;	
    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#else
//    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
//    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
//    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
//    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

//    ECD_Format(temp_max_ecd);
//    ECD_Format(temp_min_ecd);

//    temp_ecd = temp_max_ecd - temp_min_ecd;

//    if (temp_ecd > Half_ecd_range)
//    {
//        temp_ecd -= ecd_range;
//    }
//    else if (temp_ecd < -Half_ecd_range)
//    {
//        temp_ecd += ecd_range;
//    }

    temp_ecd = gimbal_cali->max_pitch_ecd - gimbal_cali->min_pitch_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_pitch_ecd - (temp_ecd / 2);
	
    ECD_Format(temp_ecd);
    *pitch_offset = temp_ecd;	
    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *yaw_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *yaw_offset);

//    temp_ecd = temp_max_ecd - temp_ecd / 2;

//    ECD_Format(temp_ecd);

//    *pitch_offset = temp_ecd;

//    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
 //   *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
  * @brief          云台校准计算，将校准记录的最大 最小值 来计算云台 中值和最大最小机械角度
  * @author         RM
  * @param[in]      yaw 中值 指针
  * @param[in]      pitch 中值 指针
  * @param[in]      yaw 最大相对角度 指针
  * @param[in]      yaw 最小相对角度 指针
  * @param[in]      pitch 最大相对角度 指针
  * @param[in]      pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        //保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;

        return 1;
    }
    else
    {
        return 0;
    }
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

int8_t getPitchAngle()
{
   return   (int8_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 180.0f / 3.14f);
}
int8_t getEenmyColor()
{
   static char  switch_enemy_color = 0;
   if((gimbal_control.gimbal_rc_ctrl->key.v & SwitchEnemyColor_Red_KeyBoard) !=  0  && switch_enemy_color == 1)
   {
       switch_enemy_color = 0;
   }
   else if((gimbal_control.gimbal_rc_ctrl->key.v & SwitchEnemyColor_Blue_KeyBoard) !=  0 && switch_enemy_color == 0)
   {
       switch_enemy_color = 1;
   }
   return switch_enemy_color;
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif


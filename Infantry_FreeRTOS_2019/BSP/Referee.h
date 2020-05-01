#ifndef __REFEREE__H
#define __REFEREE__H

#include "stdio.h"
#include "sys.h"

#define AHRS_RC_LEN  50 	//定义接收字节数 512

enum{
	Competition_Satus_e  = 0x0001,
	Competition_Result_e = 0x0002,
	Robot_Survive_Data_e = 0x0003,
	Site_Event_Data_e    = 0x0101,
	Supply_Station_Data_e  = 0x0102,
	Request_Bullet_Data_e  = 0x0103,
	Robot_Status_Data_e    = 0x0201,
	Power_Heat_Data_e      = 0x0202,
	Robot_Position         = 0x0203,
	Robot_Gain_Data_e      = 0x0204,
    Air_Robot_Power_Data_e = 0x0205,
    Hurt_data_e            = 0x0206,
    Shoot_Data_e           = 0x0207,
    Robot_Interaction      = 0x0301	
};

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

typedef __packed struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
}ext_game_state_t;

typedef __packed struct
{
uint8_t winner;
} ext_game_result_t;


typedef __packed struct
{
uint16_t robot_legion;
} ext_game_robot_survivors_t;

typedef __packed struct
{
uint32_t event_type;
} ext_event_data_t;

typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_num;
} ext_supply_projectile_booking_t;


typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_heat0_cooling_rate;
uint16_t shooter_heat0_cooling_limit;
uint16_t shooter_heat1_cooling_rate;
uint16_t shooter_heat1_cooling_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_heat0;
uint16_t shooter_heat1;
} ext_power_heat_data_t;

typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_musk_t;

typedef __packed struct
{
uint8_t energy_point;
uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;



typedef __packed struct
{
uint8_t bullet_type;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

typedef struct __Referee_Date__
{		
	frame_header_t    frame_header;
    int16_t CmdID;	  
	ext_game_state_t  game_state;
	ext_game_result_t game_result;
	ext_game_robot_survivors_t game_robot_survivors;
	ext_event_data_t      event_data;
	ext_supply_projectile_action_t supply_projectile_action;
	ext_supply_projectile_booking_t supply_projectile_booking;
	ext_game_robot_state_t game_robot_state;
	ext_power_heat_data_t  power_heat_data;
	ext_game_robot_pos_t   game_robot_pos;
	ext_buff_musk_t  buff_musk;
	aerial_robot_energy_t aerial_robot_energy;
	ext_robot_hurt_t   robot_hurt;
	ext_shoot_data_t   shoot_data;
	ext_student_interactive_header_data_t student_interactive_header_data;	
}Referee_Date;


typedef struct _power
{
	volatile  int16_t SetPower;
	float curr_power;
	int16_t Limit;
	float Kp;	
	float Ki;
	float Kd;
  volatile  float average ;	
	volatile  int16_t output;
	float pout;
	float iout;
	float dout;
	float error[3];
}Power;

typedef __packed struct _Client
{
	frame_header_t head;
	uint16_t cmd_id;
	ext_student_interactive_header_data_t student_interactive_header_data;
	float customize_data1;
	float customize_data2;
	float customize_data3;
	uint8_t Indicator_light : 6;
	uint8_t reverse:2;
	uint16_t crc16;
}clientData;


extern  Referee_Date Referee_date1;
extern  u16 Usart1_RX_Cou  ;
extern u8 Usart1_RX_Flag  ;
extern char  g_Referee_flag ;
extern Power power ; 

void Referee_init(void);

#endif

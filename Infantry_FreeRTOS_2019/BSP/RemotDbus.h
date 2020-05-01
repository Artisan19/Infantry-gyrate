#ifndef  __REMOTDBUS_H_
#define __REMOTDBUS_H_
#include "stm32f4xx.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W 		 ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S		 ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A 		 ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D 		 ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q 		 ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R         ((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F         ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G 		 ((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z 		 ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X  		 ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C  		 ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V  		 ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B 		 ((uint16_t)0x01<<15)

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

struct RC
{
    int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
};

struct MouseKey
{
	int16_t x; //!< Byte 6-7
	int16_t y; //!< Byte 8-9
	int16_t z; //!< Byte 10-11
	uint8_t press_l; //!< Byte 12
	uint8_t press_r; //!< Byte 13
	
	uint16_t v;
};

struct CMControlValue
{
   int16_t forwardOrbackward;
   int16_t left_right;
   int16_t horizoltal;
   int16_t DnfanceAngle;
   int16_t DnfanceSpeed;	
};

struct GimbalControlValue
{
   float updown;
   float left_right;
};

struct ControlValue
{
  struct CMControlValue CMControl;
  struct GimbalControlValue GimbalControl;
};



struct RemotDataPack
{
	struct RC rc;
	struct MouseKey mousekey;
	u8 RC_State;
    uint16_t	RC_onlineCnt;
};

void RemotDbus_Init(void);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);
static int16_t RC_abs(int16_t value);
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
uint8_t RC_data_is_error(void);
void slove_RC_lost(void);
void slove_data_error(void);
const RC_ctrl_t *get_remote_control_point(void);

extern volatile unsigned char sbus_rx_buffer[2][SBUS_RX_BUF_NUM];
extern  RC_ctrl_t rc_ctrl;

#endif

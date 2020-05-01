#include "MPU6500_driver.h"
#include "spi.h"
#include "mpu6500_reg.h"
#include "delay.h"
#include "IST8310_reg.h"
#include "usart.h"
#include "imu.h"
#include "freertostask.h"

IMUDataTypedef imu_data;

IMUDataTypedef imu_data_offest;

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Tx;
  
  MPU6500 = 0;
  
  MPU_Tx = reg&0x7f;
  SPI5_ReadWriteByte(MPU_Tx);
  MPU_Tx = data;
  SPI5_ReadWriteByte(MPU_Tx);
  MPU6500 = 1;
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500=0;
  
  MPU_Tx = reg|0x80;
  SPI5_ReadWriteByte(MPU_Tx);
  MPU_Rx=SPI5_ReadWriteByte(reg|0x80);
  
  MPU6500=1;
  return  MPU_Rx;
}
uint8_t MPU6500_Read_Regs(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  MPU_Tx = reg|0x80;
  MPU_Rx=SPI5_ReadWriteByte(MPU_Tx);
  return  MPU_Rx;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  delay_ms(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_ms(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  delay_ms(7);
}

//Initialize the IST8310
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  delay_ms(100);
  return 0;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}

static int16_t MPU6500_FIFO[6][11] = {0};
void MPU6500_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) 
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i = 1; i < 10; i ++)
	{
		MPU6500_FIFO[0][i-1] = MPU6500_FIFO[0][i];
		MPU6500_FIFO[1][i-1] = MPU6500_FIFO[1][i];
		MPU6500_FIFO[2][i-1] = MPU6500_FIFO[2][i];
		MPU6500_FIFO[3][i-1] = MPU6500_FIFO[3][i];
		MPU6500_FIFO[4][i-1] = MPU6500_FIFO[4][i];
		MPU6500_FIFO[5][i-1] = MPU6500_FIFO[5][i];
	}
	
	MPU6500_FIFO[0][9] = ax;
	MPU6500_FIFO[1][9] = ay;
	MPU6500_FIFO[2][9] = az;
	MPU6500_FIFO[3][9] = gx;
	MPU6500_FIFO[4][9] = gy;
	MPU6500_FIFO[5][9] = gz;
	
	for(i=0;i<10;i++)
	{	
		 sum += MPU6500_FIFO[0][i];
	}
	MPU6500_FIFO[0][10] = sum / 10;

	sum=0;
	for(i=0;i<10;i++){
		 sum += MPU6500_FIFO[1][i];
	}
	MPU6500_FIFO[1][10] = sum /10;

	sum=0;
	for(i=0;i<10;i++){
		 sum += MPU6500_FIFO[2][i];
	}
	MPU6500_FIFO[2][10] = sum / 10;

	sum=0;
	for(i=0;i<10;i++){
		 sum += MPU6500_FIFO[3][i];
	}
	MPU6500_FIFO[3][10] = sum /10;

	sum=0;
	for(i=0;i<10;i++){
		 sum += MPU6500_FIFO[4][i];
	}
	MPU6500_FIFO[4][10]= sum / 10;

	sum=0;
	for(i=0;i<10;i++){
		 sum += MPU6500_FIFO[5][i];
	}
	MPU6500_FIFO[5][10] = sum / 10;
}

					 
int16_t Gx_offset = 0,Gy_offset = 0,Gz_offset = 0;

int16_t MPU6500_Lastax, MPU6500_Lastay, MPU6500_Lastaz,
				MPU6500_Lastgx, MPU6500_Lastgy, MPU6500_Lastgz;

void MPU6500_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz,IMUDATA *offset)
{
	uint8_t mpu_buff[14];
	
	MPU6500 = 0;
	SPI5_ReadWriteByte(MPU6500_ACCEL_XOUT_H|0x80);

  mpu_buff[0]=MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H); 
  mpu_buff[1]=MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_L); 
  MPU6500_Lastax = mpu_buff[0]<<8 |mpu_buff[1];
	mpu_buff[2]=MPU6500_Read_Regs(MPU6500_ACCEL_YOUT_H); 
  mpu_buff[3]=MPU6500_Read_Regs(MPU6500_ACCEL_YOUT_L); 
  MPU6500_Lastay = mpu_buff[2]<<8 |mpu_buff[3];
	mpu_buff[4]=MPU6500_Read_Regs(MPU6500_ACCEL_ZOUT_H); 
  mpu_buff[5]=MPU6500_Read_Regs(MPU6500_ACCEL_ZOUT_L); 	
  MPU6500_Lastaz = mpu_buff[4]<<8 |mpu_buff[5];
	
	mpu_buff[6]=MPU6500_Read_Regs(MPU6500_TEMP_OUT_H); 
  mpu_buff[7]=MPU6500_Read_Regs(MPU6500_TEMP_OUT_L); 	
  imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
	
	mpu_buff[8]=MPU6500_Read_Regs(MPU6500_GYRO_XOUT_H);
  mpu_buff[9]=MPU6500_Read_Regs(MPU6500_GYRO_XOUT_L); 	
  MPU6500_Lastgx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;
	
	mpu_buff[10]=MPU6500_Read_Regs(MPU6500_GYRO_YOUT_H); 
  mpu_buff[11]=MPU6500_Read_Regs(MPU6500_GYRO_YOUT_L); 	
  MPU6500_Lastgy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
	
	mpu_buff[12]=MPU6500_Read_Regs(MPU6500_GYRO_ZOUT_H); 
  mpu_buff[13]=MPU6500_Read_Regs(MPU6500_GYRO_ZOUT_L);
  MPU6500_Lastgz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;

	MPU6500 = 1;
	
	MPU6500_DataSave(MPU6500_Lastax, MPU6500_Lastay, MPU6500_Lastaz, MPU6500_Lastgx, MPU6500_Lastgy, MPU6500_Lastgz);
	
	*ax = MPU6500_FIFO[0][10];
	*ay = MPU6500_FIFO[1][10];
	*az = MPU6500_FIFO[2][10];
	
	*gx = MPU6500_FIFO[3][10]  - offset->Gx_offset;//Gx_offset   RED3: 21  0 5  R2:-34 20 -22
	*gy = MPU6500_FIFO[4][10]  - offset->Gy_offset ;//Gy_offset
	*gz = MPU6500_FIFO[5][10]  - offset->Gz_offset;//Gz_offset
	//gy_data=*gz;
	//*gz = Correct_Data_For_GXYZ(MPU6500_FIFO[5][10] - Gz_offset + 5);
}

void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	*ax  =MPU6500_FIFO[0][10];
	*ay  =MPU6500_FIFO[1][10];
	*az = MPU6500_FIFO[2][10];
	*gx  =MPU6500_FIFO[3][10] - 16; 
	*gy = MPU6500_FIFO[4][10] +5;
	*gz = MPU6500_FIFO[5][10] +34;
}

void MPU6500_InitGyro_offset(IMUDATA *offset)
{
	u8 i = 0;
	int16_t temp[6];
	int32_t	tempgx = 0,tempgy = 0,tempgz = 0;
	int32_t	tempax = 0,tempay = 0,tempaz = 0;
	
	for(i = 0; i < 50; i++)
	{
		delay_us(100);
		MPU6500_GetMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5],&Mpu6500Data);
	}
	
	for(i = 0; i < 100; i++)
	{
		delay_us(200);
		MPU6500_GetMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5],&Mpu6500Data);
		tempax += temp[0];
		tempay += temp[1];
		tempaz += temp[2];
		tempgx += temp[3];
		tempgy += temp[4];
		tempgz += temp[5];
	}
	
	    offset->Gx_offset = tempgx / 100;
		offset->Gy_offset = tempgy / 100;
		offset->Gz_offset = tempgz / 100;
	
}

int16_t Error_Data[20] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
													10, 11, 12, 13, 14, 15, 16, 17, 18, 19}; 
/*---------------------------------------------------------
输入数据:经过均值滤波的陀螺仪角速度值
函数功能:除去陀螺仪角速度静止状态的偏差值，使其积分不会偏移	
				 常用于校准yaw角速度值
输出数据:经过矫正的角速度值	
										
很简单效果也很明显~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~非常的nice！
妈妈再也不用担心积分后yaw角度会漂移了  (*^__^*) 						
----------------------------------------------------------*/				
int16_t Correct_Data_For_GXYZ(int16_t RawDate)
{
		u8 i;	
		int16_t error_data;		//偏差值
		int16_t Correct_Data;	//修正值
		for(i = 0; i < 8; i++)
		{
				if(RawDate ==  Error_Data[i])
				{
						error_data = Error_Data[i];
						break;
				}
		}		
		Correct_Data = RawDate - error_data;
		return Correct_Data;
}



uint8_t MPU6500_Init(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x10},      // +-1000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-2G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
		{MPU6500_INT_ENABLE,    0x01},      //Enable INT
		{MPU6500_INT_PIN_CFG,   0x9c},
  };
	
  delay_ms(100);
	
	SPIConfig();
	
	
  delay_ms(100);
	
	if(MPU6500_Read_Reg(MPU6500_WHO_AM_I) != MPU6500_ID)
	{
		printf("error 1A\r\n");
		return 0xff;
	}
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    delay_ms(1);
  }
  return 0;
}

void TIM5_Config(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 90 - 1;	           
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	 
    tim.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_ARRPreloadConfig(TIM5, ENABLE);	 
    TIM_TimeBaseInit(TIM5, &tim);

    TIM_Cmd(TIM5,ENABLE);	
}

void MPU6500Config(void)
{
	while(MPU6500_Init() == 0xff)
	{
		delay_ms(200);
	}
	TIM5_Config();
}
uint32_t Get_Time_Micros(void)
{
	return TIM5->CNT;
}


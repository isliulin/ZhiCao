#ifndef __MPU6050DMP_H
#define __MPU6050DMP_H

/*在使用前，先在预编译符号定义中定义MPU6050和EMPL_TARGET_STM32*/


#include "stm32f10x.h"
#include "I2CRoutines.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

//****************************************
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU6050_CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU6050_Addr	0xD0	//IIC写入时的地址字节数据，+1为读取
#define MPU6050_I2C I2C1 //MPU6050 I2C端口

/* MPU6050 Structure definition */
struct MPU6050_RawData_s
{
  int16_t ax;
  int16_t ay;
  int16_t az;
	int16_t temperature;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

int InitMPU6050(void);
uint16_t MPU6050_GetData(uint8_t REG_Address);
int MPU6050_ReadRawData(struct MPU6050_RawData_s *data);
int MPU6050_Read_Ext_Sens_Data(uint8_t RegAddr,uint8_t *buff, uint8_t length);


int stm32_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
int stm32_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);

//定时延时程序
/*
在main.c的NVIC_Configuration函数中
设置SysTick_Config(720)，每10us产生一次SysTick
在stm32f10x_it.c中的SysTick_Handler处理SysTick中断，
执行TimingDelay_Decrement();
并且每100个10us使TimeStamp_ms加1
*/
static __IO uint32_t TimingDelay; //SysTick计数变量
extern __IO uint32_t TimeStamp_ms;

void TimingDelay_Decrement(void);//获取节拍程序


void Delay_us(__IO uint32_t nTime);//定时延时程序 10us为单位
void Delay_ms(__IO uint32_t nTime);//定时延时程序 1ms为单位

int stm32_get_clock_ms(unsigned long *count); //获取时钟毫秒数
//定时延时程序结束


unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx);
unsigned short inv_row_2_scale(const signed char *row);
void mpu6050_run_self_test(void);

//姿态结构定义
struct dmpGravity_s
{
  float x;
	float y;
	float z;
};

struct dmpQuaternion_s
{
	float w;
  float x;
	float y;
	float z;
};

struct dmpEuler_s
{
	float psi;
  float theta;
	float phi;
};

struct dmpYawPitchRoll_s
{
	float yaw;
  float pitch;
	float roll;
};

//姿态获取函数定义
int dmpGetGravity(struct dmpGravity_s *v, struct dmpQuaternion_s *q);
int dmpGetEuler(struct dmpEuler_s *data, struct dmpQuaternion_s *q);
int dmpGetYawPitchRoll(struct dmpYawPitchRoll_s *data, struct dmpQuaternion_s *q, struct dmpGravity_s *gravity);
		
#endif

#include "HMC5883.h"
#include "math.h"
#include "MPU6050DMP.h"

uint8_t HMC5883_Buffer_Tx[2];
uint8_t HMC5883_Buffer_Rx[6];

Status InitHMC5883(void)
{
		//寄存器A，D7必须为0，D6D5在每次测量输出中选择采样平均数（1-8）00=1; 01=2;10=4; 11=8(缺省)；
	//D4D3D2数据输出速率位000~110依次升高；D1D0测量配置位，00-正常，01正偏压，10负偏压
		HMC5883_Buffer_Tx[0]=0x00;
		HMC5883_Buffer_Tx[1]=0x10;
// 	Buffer_Tx1[1]=0x01;  //自测试
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
	
			//寄存器B，增益配置，高3位从000~111依次减少，低5位必须为0
		HMC5883_Buffer_Tx[0]=0x01;
		HMC5883_Buffer_Tx[1]=0x00;
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
	
			//模式寄存器，D7~D2必须为0，D1D0模式选择，00连续测量，01单次测量，10或11闲置
		HMC5883_Buffer_Tx[0]=0x02;
		HMC5883_Buffer_Tx[1]=0x00; //连续测量
// 	Buffer_Tx1[1]=0x01; //单次测量
	  return I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
	
}

Status HMC5883_GetData(struct HMC5883_s *var)
{
// 		Buffer_Tx1[0]=0x03;
// 	  if(I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Polling, 0x3C) == Error)
// 			return Error;
// 		I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, 0x3C);
// 		x=Buffer_Rx1[0]<<8;
// 		
// 		Buffer_Tx1[0]=0x04;
// 	  if(I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Polling, 0x3C) == Error)
// 			return Error;
// 		I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, 0x3C);
// 		x |= Buffer_Rx1[0];

// 		Buffer_Tx1[0]=0x05;
// 	  if(I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Polling, 0x3C) == Error)
// 			return Error;		
// 		I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, 0x3C);
// 		z=Buffer_Rx1[0]<<8;

// 		Buffer_Tx1[0]=0x06;
// 	  if(I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Polling, 0x3C) == Error)
// 			return Error;
// 		I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, 0x3C);
// 		z |= Buffer_Rx1[0];

// 		Buffer_Tx1[0]=0x07;
// 	  if(I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Polling, 0x3C) == Error)
// 			return Error;		
// 		I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, 0x3C);
// 		y=Buffer_Rx1[0]<<8;

// 		Buffer_Tx1[0]=0x08;
// 	  if(I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Polling, 0x3C) == Error)
// 			return Error;
// 		I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, 0x3C);
// 		y |= Buffer_Rx1[0];
// 		
// 		return Success;
		HMC5883_Buffer_Tx[0]=0x03;
	  if(I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,1,Polling, HMC5883_ADDR) == Error)
			return Error;
		I2C_Master_BufferRead(HMC5883_I2C,HMC5883_Buffer_Rx,6,Polling, HMC5883_ADDR);
		var->x=HMC5883_Buffer_Rx[0]<<8 | HMC5883_Buffer_Rx[1];
		var->z=HMC5883_Buffer_Rx[2]<<8 | HMC5883_Buffer_Rx[3];
		var->y=HMC5883_Buffer_Rx[4]<<8 | HMC5883_Buffer_Rx[5];
		var->angle = atan2((double)(var->y+240),(double)(var->x-9)) * (180/3.1415926)+180;
		if(var->angle>180) var->angle -=360;
		return Success;
}

Status HMC5883_GetDataBuff(uint8_t *buff,uint8_t buff_len)
{
		uint8_t i, i_end;

		HMC5883_Buffer_Tx[0]=0x03;
	  if(I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,1,Polling, HMC5883_ADDR) == Error)
			return Error;
		
		if(I2C_Master_BufferRead(HMC5883_I2C,HMC5883_Buffer_Rx,6,Polling, HMC5883_ADDR) == Error)
			return Error;
		
		i_end = 6;
		if(buff_len<i_end) i_end = buff_len;
		
		for(i=0;i<buff_len;i++)
		{
			buff[i] = HMC5883_Buffer_Rx[i];
		}
		
		return Success;
}

Status HMC5883_RunSelfTest(struct HMC5883_s *var)
{
		Delay_ms(100);
		HMC5883_Buffer_Tx[0]=0x00;
		HMC5883_Buffer_Tx[1]=0x11;  //自测试，正极性
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
	
			//寄存器B，增益配置，高3位从000~111依次减少，低5位必须为0
		HMC5883_Buffer_Tx[0]=0x01;
		HMC5883_Buffer_Tx[1]=0x20;
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
	
		Delay_ms(100);
			//模式寄存器，D7~D2必须为0，D1D0模式选择，00连续测量，01单次测量，10或11闲置
		HMC5883_Buffer_Tx[0]=0x02;
		HMC5883_Buffer_Tx[1]=0x01; //单次测量
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
		
		Delay_ms(100);
		HMC5883_Buffer_Tx[0]=0x03;
	  if(I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,1,Polling, HMC5883_ADDR) == Error)
			return Error;
		
		if(I2C_Master_BufferRead(HMC5883_I2C,HMC5883_Buffer_Rx,6,Polling, HMC5883_ADDR) == Error)
			return Error;
		
		HMC5883_Buffer_Tx[0]=0x00;
		HMC5883_Buffer_Tx[1]=0x10;  //正常
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
		HMC5883_Buffer_Tx[0]=0x02;
		HMC5883_Buffer_Tx[1]=0x00; //连续测量
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
		
		Delay_ms(1000);
		
		HMC5883_Buffer_Tx[0]=0x00;
		HMC5883_Buffer_Tx[1]=0x12;  //自测试，负极性
		//模式寄存器，D7~D2必须为0，D1D0模式选择，00连续测量，01单次测量，10或11闲置
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
					
		//寄存器B，增益配置，高3位从000~111依次减少，低5位必须为0
		HMC5883_Buffer_Tx[0]=0x01;
		HMC5883_Buffer_Tx[1]=0x20;
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
		
		Delay_ms(100);
		
		HMC5883_Buffer_Tx[0]=0x02;
		HMC5883_Buffer_Tx[1]=0x01; //单次测量
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,2,Polling, HMC5883_ADDR);
		
		Delay_ms(100);
		HMC5883_Buffer_Tx[0]=0x03;
	  if(I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,1,Polling, HMC5883_ADDR) == Error)
			return Error;
		
		if(I2C_Master_BufferRead(HMC5883_I2C,HMC5883_Buffer_Rx,6,Polling, HMC5883_ADDR) == Error)
			return Error;
		
		InitHMC5883(); //重新初始化为原来模式
		return Success;
}

//获取HMC 5883状态寄存器，
// D1：数据输出寄存器锁存。当六个数据输出寄存器上的一些但不是全部数据被
// 读取时，该位置位。当此位置位时，六个数据输出寄存器被锁定且任何新
// 的数据将不会被更新至这些寄存器中，除非符合以下三个条件之一： 一，
// 所有6个寄存器已被读取或模式改变，二，模式发生变化，三，测量配置发
// 生变化。
// D0：准备就绪位。当数据都写入了6个数据寄存器，该位置位。在一个或几个数
// 据写入输出寄存器以后且在装置开始向数据输出寄存器写入数据时该位被
// 清除。当RDY位已清除， RDY应保持清除状态至少250微秒。 DRDY引脚可被
// 用来作为一种替代的状态寄存器的监测装置为测量数据。
Status HMC5883_GetStatus(uint8_t *status)
{
		HMC5883_Buffer_Tx[0]=0x09;
	  I2C_Master_BufferWrite(HMC5883_I2C, HMC5883_Buffer_Tx,1,Polling, HMC5883_ADDR);
	if(I2C_Master_BufferRead(HMC5883_I2C,HMC5883_Buffer_Rx,1,Polling, HMC5883_ADDR) == Success)
	{
		*status = HMC5883_Buffer_Rx[0];
		return Success;
	}
	else
	{	
		return Error;
	}
}

Status InitMMA7660(void)
{
		HMC5883_Buffer_Tx[0]=0x07;
		HMC5883_Buffer_Tx[1]=0x01;
	  return I2C_Master_BufferWrite(I2C1, HMC5883_Buffer_Tx,2,Polling, 0x98);
}

Status MMA7660_GetData(struct MMA7660_s *var)
{
		float temp;
		HMC5883_Buffer_Tx[0]=0x00;
	  I2C_Master_BufferWrite(I2C1, HMC5883_Buffer_Tx,1,Polling, 0x98);
		if(I2C_Master_BufferRead(I2C1,HMC5883_Buffer_Rx,3,Polling, 0x98) == Error)
			return Error;
		
		var->x = HMC5883_Buffer_Rx[0];
		if(HMC5883_Buffer_Rx[0] & 0x20) var->x = var->x | 0xC0;
		var->y = HMC5883_Buffer_Rx[1];
		if(HMC5883_Buffer_Rx[1] & 0x20) var->y = var->y | 0xC0;
		var->z = HMC5883_Buffer_Rx[2];
		if(HMC5883_Buffer_Rx[2] & 0x20) var->z = var->z | 0xC0;
		
		temp = -(float)(var->x)/0x15;
		if(temp<-1) temp=-1;
		if(temp>1) temp = 1;
		var->PitchAngle = acos(temp) * (180/3.1415926)-90;
		
		temp = -(float)(var->y)/0x15;
		if(temp<-1) temp=-1;
		if(temp>1) temp = 1;
		var->RollAngle = acos(temp) * (180/3.1415926)-90;
		
		temp = -(float)(var->z)/0x15;
		if(temp<-1) temp=-1;
		if(temp>1) temp = 1;
		var->TiltAngle = acos(temp) * (180/3.1415926)-90;
		
		return Success;
}

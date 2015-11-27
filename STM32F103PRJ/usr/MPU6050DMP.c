#include "MPU6050DMP.h"
#include "stdio.h"
#include "math.h"


__IO uint32_t TimeStamp_ms=0; //毫秒绝对值
/* Buffer of data to be received by I2C1 */
uint8_t IIC_Buffer_Rx[25];
/* Buffer of data to be transmitted by I2C1 */
uint8_t IIC_Buffer_Tx[25] = {0x5, 0x6,0x8,0xA};

//**************************************
//初始化MPU6050
//**************************************
int InitMPU6050()
{
	Status rtn;
	//Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
	IIC_Buffer_Tx[0]=PWR_MGMT_1;
	IIC_Buffer_Tx[1]=0x01;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	//Single_WriteI2C(SMPLRT_DIV, 0x07);
	IIC_Buffer_Tx[0]=SMPLRT_DIV;
	IIC_Buffer_Tx[1]=0x31;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	
	//Single_WriteI2C(MPU6050_CONFIG, 0x06);
	IIC_Buffer_Tx[0]=MPU6050_CONFIG;
	IIC_Buffer_Tx[1]=0x06;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	//Single_WriteI2C(GYRO_CONFIG, 0x18);
	IIC_Buffer_Tx[0]=GYRO_CONFIG;
	IIC_Buffer_Tx[1]=0x18;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	//Single_WriteI2C(ACCEL_CONFIG, 0x01);
	IIC_Buffer_Tx[0]=ACCEL_CONFIG;
	IIC_Buffer_Tx[1]=0x01;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	//配置Motion Interrupt
	//Single_WriteI2C(ACCEL_CONFIG, 0x01);
	IIC_Buffer_Tx[0]=0x38;
	IIC_Buffer_Tx[1]=0x41;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	//Single_WriteI2C(ACCEL_CONFIG, 0x01);
	IIC_Buffer_Tx[0]=0x1F;
	IIC_Buffer_Tx[1]=0x05;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	//Single_WriteI2C(ACCEL_CONFIG, 0x01);
	IIC_Buffer_Tx[0]=0x20;
	IIC_Buffer_Tx[1]=0x07;
  rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	
	return !rtn;
}

//**************************************
//读指定地址数据
//**************************************
uint16_t MPU6050_GetData(uint8_t REG_Address)
{
	Status rtn;
	char H,L;
	//H=Single_ReadI2C(REG_Address);
		IIC_Buffer_Tx[0]=REG_Address;
	  I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
		rtn=I2C_Master_BufferRead(MPU6050_I2C,IIC_Buffer_Rx,1,Polling, MPU6050_Addr);	
		if(rtn == Success) H=IIC_Buffer_Rx[0];
		else H=0;
	//L=Single_ReadI2C(REG_Address+1);
		IIC_Buffer_Tx[0]=REG_Address+1;
	  I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
		rtn=I2C_Master_BufferRead(MPU6050_I2C,IIC_Buffer_Rx,1,Polling, MPU6050_Addr);
		if(rtn == Success) L=IIC_Buffer_Rx[0];
		else L=0;
	return (H<<8)+L;   //合成数据
}

//**************************************
//连续读ax,ay,az,temperature,gx,gy,gz数据
//**************************************
int MPU6050_ReadRawData(struct MPU6050_RawData_s *s_IMUVar)
{
	Status rtn;
	char H,L;

	IIC_Buffer_Tx[0]=ACCEL_XOUT_H;
  I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
	rtn=I2C_Master_BufferRead(MPU6050_I2C,IIC_Buffer_Rx,14,Polling, MPU6050_Addr);	
	if(rtn == Success) 
	{
		H=IIC_Buffer_Rx[0];
		L=IIC_Buffer_Rx[1];
		s_IMUVar->ax = (H<<8)+L;
		
		H=IIC_Buffer_Rx[2];
		L=IIC_Buffer_Rx[3];
		s_IMUVar->ay = (H<<8)+L;
		
		H=IIC_Buffer_Rx[4];
		L=IIC_Buffer_Rx[5];
		s_IMUVar->az = (H<<8)+L;
		
		H=IIC_Buffer_Rx[6];
		L=IIC_Buffer_Rx[7];
		s_IMUVar->temperature = (H<<8)+L;
		
		H=IIC_Buffer_Rx[8];
		L=IIC_Buffer_Rx[9];
		s_IMUVar->gx = (H<<8)+L;
		
		H=IIC_Buffer_Rx[10];
		L=IIC_Buffer_Rx[11];
		s_IMUVar->gy = (H<<8)+L;
		
		H=IIC_Buffer_Rx[12];
		L=IIC_Buffer_Rx[13];
		s_IMUVar->gz = (H<<8)+L;

	}
	return !rtn;
}


int stm32_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data)
{
    unsigned char i;
		Status rtn;
    if (!length)
        return 0;
		
		if(length > 24) return -1;
		
		IIC_Buffer_Tx[0] = reg_addr;
		for(i=0;i<length;i++)
		{
			IIC_Buffer_Tx[i+1]=data[i];
		}
		rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,length+1,Polling, slave_addr);
    if(rtn==Success)return 0;
		else return -1;
}


int stm32_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{
		Status rtn;
    if (!length)
        return 0;

		rtn=I2C_Master_BufferWrite(MPU6050_I2C, &reg_addr,1,Polling, slave_addr);
		if(rtn!=Success) return -1;
		rtn=I2C_Master_BufferRead(MPU6050_I2C, (uint8_t*)data,length,Polling, slave_addr);
    if(rtn==Success)
		{
			
			return 0;
		}
		else 
		{
			return -1;
		}
}

/****************************************************************************
* 名    称：void TimingDelay_Decrement(void)
* 功    能：获取节拍程序
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/  
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
/****************************************************************************
* 名    称：void Delay_us(__IO uint32_t nTime)
* 功    能：定时延时程序 10us为单位
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/  
void Delay_us(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

void Delay_ms(__IO uint32_t nTime)
{ 
	uint32_t i;
	i=nTime;
	while(i--)
		Delay_us(100);
}

int stm32_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = TimeStamp_ms;
    return 0;
}
unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

void mpu6050_run_self_test(void)
{
    int rtn;
    long gyro[3], accel[3];
		float sens;
		unsigned short accel_sens;

    rtn = mpu_run_self_test(gyro, accel);
    if (rtn == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */

        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
			printf("\r\nSelf Test Passed!\r\n");
    }
}

//
int dmpGetGravity(struct dmpGravity_s *v, struct dmpQuaternion_s *q) {
    v->x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v->y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v->z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

int dmpGetEuler(struct dmpEuler_s *data, struct dmpQuaternion_s *q) {
    data->psi = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data->theta = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data->phi = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}

int dmpGetYawPitchRoll(struct dmpYawPitchRoll_s *data, struct dmpQuaternion_s *q, struct dmpGravity_s *gravity) {
    // yaw: (about Z axis) 偏航角
    data->yaw = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis) 俯仰角
    data->pitch = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis) 横滚角
    data->roll = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}


//读取MPU6050 I2C Slave0 的数据
int MPU6050_Read_Ext_Sens_Data(uint8_t RegAddr,uint8_t *buff, uint8_t length)
{
	Status rtn;
	IIC_Buffer_Tx[0] = RegAddr;  
	rtn = I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
	rtn = I2C_Master_BufferRead(MPU6050_I2C, buff,length,Polling, MPU6050_Addr);
	return rtn;
}

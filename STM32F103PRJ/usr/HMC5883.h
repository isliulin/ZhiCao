#ifndef __HMC5883_H
#define __HMC5883_H

#include "stm32f10x.h"
#include "I2CRoutines.h"

#define HMC5883_ADDR 0x3C
#define HMC5883_I2C I2C1

/* HMC5883 Structure definition */
struct HMC5883_s
{
  int16_t x;
  int16_t y;
  int16_t z;
	int8_t status;
	float angle;
};

struct MMA7660_s
{
  int8_t x;
  int8_t y;
  int8_t z;
	int8_t status;
	
	int16_t RollAngle; //ºá¹ö½Ç
	int16_t PitchAngle; //¸©Ñö½Ç
	int16_t TiltAngle; //¸©Ñö½Ç
};

//struct HMC5883_s s_HMC5883_Var;

Status InitHMC5883(void);
Status HMC5883_GetData(struct HMC5883_s *var);
Status HMC5883_RunSelfTest(struct HMC5883_s *var);
Status HMC5883_GetDataBuff(uint8_t *buff,uint8_t buff_len);
Status HMC5883_GetStatus(uint8_t *status);

Status InitMMA7660(void);
Status MMA7660_GetData(struct MMA7660_s *var);
#endif

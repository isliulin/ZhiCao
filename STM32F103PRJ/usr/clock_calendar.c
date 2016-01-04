/**
  ******************************************************************************
  * @file RTC/src/clock_calendar.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Clock Calendar basic routines
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 
/*
BKP_DR1保存是否已经配置日历时间
BKP_DR2、3、4保存月、日、年
BKP_DR8、9、10保存闹钟月、日、年
BKP_DR11~24保存周一到周五的工作时间（起始时分和结束时分）
*/

/* Includes ------------------------------------------------------------------*/
#include "clock_calendar.h"
#include "stdio.h"

//#define RTCClockSource_LSI   /* 用内置的32K 时钟晶振源 */
#define RTCClockSource_LSE   /* 用外置的32.768K 时钟晶振源 */

/* Private variables--------------------------------------------------------- */
// uint8_t ClockSource;
// uint8_t *MonthsNames[]={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug",\
// "Sep","Oct","Nov","Dec"};
// const uint8_t CalibrationPpm[128]={0,1,2,3,4,5,6,7,8,9,10,10,11,12,13,14,15,16,17,\
//                          18,19,20,21,22,23,24,25,26,27,28,29,30,31,31,32,33,34,\
//                          35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,\
//                          52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,\
//                          70,71,72,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,\
//                          87,88,89,90,91,92,93,93,94,95,96,97,98,99,100,101,102,\
//                          103,104,105,106,107,108,109,110,111,112,113,113,114,\
//                          115,116,117,118,119,120,121};
/*Structure variable declaration for system time, system date,
alarm time, alarm date */
struct Time_s s_TimeStructVar;
struct AlarmTime_s s_AlarmStructVar;
struct Date_s s_DateStructVar;
struct AlarmDate_s s_AlarmDateStructVar;
struct WorkTime_s s_WorkTimeVar[7];

// extern uint16_t SummerTimeCorrect;

/** @addtogroup RTC
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  COnfiguration of RTC Registers, Selection and Enabling of 
  *   RTC clock
  * @param  None
  * @retval : None
  */
void RTC_Configuration()
{
		uint8_t i;
	/* 使能 PWR 和 BKP 的时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
  /* 允许访问BKP区域 */
  PWR_BackupAccessCmd(ENABLE);
	
	if(BKP_ReadBackupRegister(BKP_DR1) != CONFIGURATION_DONE)		     //判断保存在备份寄存器的RTC标志是否已经被配置过
  {
//     printf("\r\n\n RTC not yet configured....");
//  /* 使能 PWR 和 BKP 的时钟 */
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
//   
//   /* 允许访问BKP区域 */
//   PWR_BackupAccessCmd(ENABLE);

  /* 复位BKP */
  BKP_DeInit();

#ifdef RTCClockSource_LSI
  /* 使能内部RTC时钟 */ 
  RCC_LSICmd(ENABLE);
  /* 等待RTC内部时钟就绪 */
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }
  /* 选择RTC内部时钟为RTC时钟 */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  
#elif defined	RTCClockSource_LSE  
  /* 使能RTC外部时钟 */
  RCC_LSEConfig(RCC_LSE_ON);
  /* 等待RTC外部时钟就绪 */
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {	    
  }

  /* 选择RTC外部时钟为RTC时钟 */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
#endif

  /* 使能RTC时钟 */
  RCC_RTCCLKCmd(ENABLE);

#ifdef RTCClockOutput_Enable  
  /* Disable the Tamper Pin */
  BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                               functionality must be disabled */
                               
  /* 使能在TAMPER脚输出RTC时钟 */
  BKP_RTCCalibrationClockOutputCmd(ENABLE);
#endif 

  /* 等待RTC寄存器同步 */
  RTC_WaitForSynchro();

  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask();
  
  /* 使能RTC秒中断 */  
  RTC_ITConfig(RTC_IT_SEC, ENABLE);

  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask();
  
  /* 设置RTC预分频 */
#ifdef RTCClockSource_LSI
  RTC_SetPrescaler(31999);            /* RTC period = RTCCLK/RTC_PR = (32.000 KHz)/(31999+1) */
#elif defined	RTCClockSource_LSE
  RTC_SetPrescaler(32767);            /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
#endif
  
  /* 等待写RTC寄存器完成 */
  RTC_WaitForLastTask();
//printf("\r\n RTC configured....");

	BKP_WriteBackupRegister(BKP_DR1, CONFIGURATION_DONE);    	      //RTC设置后，将已配置标志写入备份数据寄存器 
	
	//设置默认日期、时间、工作时间
	s_DateStructVar.Month=DEFAULT_MONTH ;
	s_DateStructVar.Day=DEFAULT_DAY;
	s_DateStructVar.Year=DEFAULT_YEAR;
	//SummerTimeCorrect = OCTOBER_FLAG_SET;
	//BKP_WriteBackupRegister(BKP_DR7,SummerTimeCorrect);
	BKP_WriteBackupRegister(BKP_DR2,s_DateStructVar.Month);
	BKP_WriteBackupRegister(BKP_DR3,s_DateStructVar.Day);
	BKP_WriteBackupRegister(BKP_DR4,s_DateStructVar.Year);
	SetTime(DEFAULT_HOURS,DEFAULT_MINUTES,DEFAULT_SECONDS);
	for(i=0;i<7;i++)
	{
		s_WorkTimeVar[i].StartHour = 9;
		s_WorkTimeVar[i].StartMin = 0;
		s_WorkTimeVar[i].EndHour = 15;
		s_WorkTimeVar[i].EndMin = 0;
	}
	SaveWorkTime();
	
  }
  else
  {	     
//      if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)	  //检查是否掉电重启
//      {
//        printf("\r\n\n Power On Reset occurred....");
//      }												     
//      else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) //检查是否reset复位
//      {
//        printf("\r\n\n External Reset occurred....");
//      }
//      printf("\r\n No need to configure RTC....");  
		
    RTC_WaitForSynchro();								   //等待RTC寄存器被同步 
    RTC_ITConfig(RTC_IT_SEC, ENABLE);					   //使能秒中断
    RTC_WaitForLastTask();								   //等待写入完成
		
		LoadWorkTime();
  }
  RCC_ClearFlag();										   //清除复位标志
	
  /* Check if how many days are elapsed in power down/Low Power Mode-
   Updates Date that many Times*/
  CheckForDaysElapsed();
//ClockSource = BKP_ReadBackupRegister(BKP_DR6);
//TamperNumber = BKP_ReadBackupRegister(BKP_DR5);
  s_DateStructVar.Month = BKP_ReadBackupRegister(BKP_DR2);
  s_DateStructVar.Day = BKP_ReadBackupRegister(BKP_DR3);
  s_DateStructVar.Year = BKP_ReadBackupRegister(BKP_DR4);
//   SummerTimeCorrect = BKP_ReadBackupRegister(BKP_DR7);
//   s_AlarmDateStructVar.Month = BKP_ReadBackupRegister(BKP_DR8);
//   s_AlarmDateStructVar.Day = BKP_ReadBackupRegister(BKP_DR9);
//   s_AlarmDateStructVar.Year = BKP_ReadBackupRegister(BKP_DR10);
	
}



/**
  * @brief  Summer Time Correction routine
  * @param  None
  * @retval : None
  */
// void SummerTimeCorrection(void)
// {
//   uint8_t CorrectionPending=0;
//   uint8_t CheckCorrect=0;
//   
//   if((SummerTimeCorrect & OCTOBER_FLAG_SET)!=0)
//   {
//     if((s_DateStructVar.Month==10) && (s_DateStructVar.Day >24 ))
//     {
//       for(CheckCorrect = 25;CheckCorrect <=s_DateStructVar.Day;CheckCorrect++)
//       {
//         if(WeekDay(s_DateStructVar.Year,s_DateStructVar.Month,CheckCorrect )==0)
//         {
//           if(CheckCorrect == s_DateStructVar.Day)
//           {
//             /* Check if Time is greater than equal to 1:59:59 */
//             if(RTC_GetCounter()>=7199)
//             {
//               CorrectionPending=1;
//             }
//           }
//           else
//           {
//             CorrectionPending=1;
//           }
//          break;
//        }
//      }
//    }
//    else if((s_DateStructVar.Month > 10))
//    {
//      CorrectionPending=1;
//    }
//    else if(s_DateStructVar.Month < 3)
//    {
//      CorrectionPending=1;
//    }
//    else if(s_DateStructVar.Month == 3)
//    {
//      if(s_DateStructVar.Day<24)
//      {
//        CorrectionPending=1;
//      }
//      else
//      {
//        for(CheckCorrect=24;CheckCorrect<=s_DateStructVar.Day;CheckCorrect++)
//        {
//          if(WeekDay(s_DateStructVar.Year,s_DateStructVar.Month,CheckCorrect)==0)
//          {
//            if(CheckCorrect == s_DateStructVar.Day)
//            {
//              /*Check if Time is less than 1:59:59 and year is not the same in which
//                 March correction was done */
//              if((RTC_GetCounter() < 7199) && ((SummerTimeCorrect & 0x3FFF) != \
//                                    s_DateStructVar.Year))
//              {
//                CorrectionPending=1;
//              }
//              else
//              {
//                CorrectionPending=0;
//              }
//              break;
//             }
//             else
//             {
//               CorrectionPending=1;
//             }
//           }
//         }
//       }
//     }
//   }
//   else if((SummerTimeCorrect & MARCH_FLAG_SET)!=0)
//   {
//     if((s_DateStructVar.Month == 3) && (s_DateStructVar.Day >24 ))
//     {
//       for(CheckCorrect = 25;CheckCorrect <=s_DateStructVar.Day;\
//          CheckCorrect++)
//       {
//         if(WeekDay(s_DateStructVar.Year,s_DateStructVar.Month,\
//            CheckCorrect )==0)
//         {
//           if(CheckCorrect == s_DateStructVar.Day)
//           {
//             /*Check if time is greater than equal to 1:59:59 */
//             if(RTC_GetCounter()>=7199)
//             {
//               CorrectionPending=1;
//             }
//           }
//           else
//           {
//             CorrectionPending=1;
//           }
//         break;
//         }
//       }
//     }
//     else if((s_DateStructVar.Month > 3) && (s_DateStructVar.Month < 10 ))
//     {
//       CorrectionPending=1;
//     }
//     else if(s_DateStructVar.Month ==10)
//     {
//       if(s_DateStructVar.Day<24)
//       {
//         CorrectionPending=1;
//       }
//       else
//       {
//         for(CheckCorrect=24;CheckCorrect<=s_DateStructVar.Day;\
//           CheckCorrect++)
//         {
//           if(WeekDay(s_DateStructVar.Year,s_DateStructVar.Month,\
//             CheckCorrect)==0)
//           {
//             if(CheckCorrect == s_DateStructVar.Day)
//             {
//               /*Check if Time is less than 1:59:59 and year is not the same in
//               which March correction was done */
//               if((RTC_GetCounter() < 7199) && \
//                 ((SummerTimeCorrect & 0x3FFF) != s_DateStructVar.Year))
//               {
//                 CorrectionPending=1;
//               }
//               else
//               {
//                 CorrectionPending=0;
//               }
//             break;
//             }
//           }
//         }
//       }
//     }
//   }

//   if(CorrectionPending==1)
//   {
//     if((SummerTimeCorrect & OCTOBER_FLAG_SET)!=0)
//     {
//       /* Subtract 1 hour from the current time */
//       RTC_SetCounter(RTC_GetCounter() - 3599);
//       /* Reset October correction flag */
//       SummerTimeCorrect &= 0xBFFF;
//       /* Set March correction flag  */
//       SummerTimeCorrect |= MARCH_FLAG_SET;
//       SummerTimeCorrect |= s_DateStructVar.Year;
//       BKP_WriteBackupRegister(BKP_DR7,SummerTimeCorrect);
//     }
//     else if((SummerTimeCorrect & MARCH_FLAG_SET)!=0)
//     {
//      /* Add 1 hour to current time */
//      RTC_SetCounter(RTC_GetCounter() + 3601);
//      /* Reset March correction flag */
//      SummerTimeCorrect &= 0x7FFF;
//      /* Set October correction flag  */
//      SummerTimeCorrect |= OCTOBER_FLAG_SET;
//      SummerTimeCorrect |= s_DateStructVar.Year;
//      BKP_WriteBackupRegister(BKP_DR7,SummerTimeCorrect);
//     }
//   }
// }

/**
  * @brief  Apllication Initialisation Routine
  * @param  None
  * @retval : None
  */
// void ApplicationInit(void)
// {
//   /* System Clocks Configuration */
//   RCC_Configuration();
//   /*Enables the clock to Backup and power interface peripherals    */
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR,ENABLE);
//   
//   /* SysTick Configuration*/
//   SysTickConfig();
//   
//   /*Initialisation of TFT LCD */
//   STM3210B_LCD_Init();
//   
//   /* Unlock the Flash Program Erase controller */
//   FLASH_Unlock();
//   /*RTC_NVIC Configuration */
//   RTC_NVIC_Configuration();
//   
//   /* RTC Configuration*/
//   RTC_Configuration();
//   BKP_RTCOutputConfig(BKP_RTCOutputSource_None);
//   
//   /* General Purpose I/O Configuration */
//   GPIO_Configuration();
//   
//   /* Battery Removal Emulation   */
//   GPIO_SetBits(GPIOC, GPIO_Pin_8);
//   
//   while(!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)))
//   {
//     TamperEvent = 1;
//   }
//   
//   /* Joystick NVIC Configuration  */
//   NVIC_JoyStickConfig();

//   /* Tamper pin NVIC Configuration  */
//   Tamper_NVIC_Configuration();

//   /* Configure PVD Supervisor to disable the Tamper Interrupt when voltage drops 
//   below 2.5 volts*/
//   PWR_PVDCmd(ENABLE);
//   PWR_PVDLevelConfig(PWR_PVDLevel_2V5);
//   PWR_BackupAccessCmd(ENABLE);

//   /* Only JoyStick Sel Interrupt is enabled on startup */
//   SelIntExtOnOffConfig(ENABLE);

//   /* Tamper FeatureRTC  -   Enable Tamper Interrupt and configure for Low level */
//   BKP_ITConfig(ENABLE);

//   /* Enable Tamper Pin for Active low level: Tamper level detected for low level*/
//   BKP_TamperPinLevelConfig(BKP_TamperPinLevel_Low);

//   /* Enable tamper Pin Interrupt */
//   BKP_TamperPinCmd(ENABLE);

//   /*  Menu Initialisation  */
//   MenuInit();
// }



/**
  * @brief  Sets the RTC Current Counter Value
  * @param Hour, Minute and Seconds data
  * @retval : None
  */
void SetTime(uint8_t Hour,uint8_t Minute,uint8_t Seconds)
{
  uint32_t CounterValue;

  CounterValue=((Hour * 3600)+ (Minute * 60)+Seconds);

  RTC_WaitForLastTask();
  RTC_SetCounter(CounterValue);
  RTC_WaitForLastTask();
}



/**
  * @brief  Sets the RTC Alarm Register Value
  * @param Hours, Minutes and Seconds data
  * @retval : None
  */
void SetAlarm(uint8_t Hour,uint8_t Minute, uint8_t Seconds)
{
  uint32_t CounterValue;
  
  CounterValue=((Hour * 3600)+ (Minute * 60)+Seconds);
  
  if(CounterValue == 0)
  {
    CounterValue = SECONDS_IN_DAY;
  }

  RTC_WaitForLastTask();
  RTC_SetAlarm(CounterValue);
  RTC_WaitForLastTask();
}



/**
  * @brief  Sets the RTC Date(DD/MM/YYYY)
  * @param DD,MM,YYYY
  * @retval : None
  */
void SetDate(uint8_t Day, uint8_t Month, uint16_t Year)
{
  uint32_t DateTimer;
  
//   RightLeftIntExtOnOffConfig(DISABLE);
//   UpDownIntOnOffConfig(DISABLE);

  /*Check if the date entered by the user is correct or not, Displays an error
    message if date is incorrect  */
  if((( Month==4 || Month==6 || Month==9 || Month==11) && Day ==31) \
    || (Month==2 && Day==31)|| (Month==2 && Day==30)|| \
      (Month==2 && Day==29 && (CheckLeap(Year)==0)))
  {
//     LCD_Clear(Red);
//     LCD_SetBackColor(Red);
//     LCD_DisplayString(Line3,Column2,"INCORRECT DATE");
//     LCD_DisplayString(Line6,Column1,"PLEASE RECONFIGURE");
    DateTimer=RTC_GetCounter();

    while((RTC_GetCounter()-DateTimer)<2)
    {
    }
  }
  /* if date entered is correct then set the date*/
  else
  {
 //   if(AlarmDate==0)
    {
      BKP_WriteBackupRegister(BKP_DR2,Month);
      BKP_WriteBackupRegister(BKP_DR3,Day);
      BKP_WriteBackupRegister(BKP_DR4,Year);
			s_DateStructVar.Month=BKP_ReadBackupRegister(BKP_DR2);
			s_DateStructVar.Year=BKP_ReadBackupRegister(BKP_DR4);
			s_DateStructVar.Day=BKP_ReadBackupRegister(BKP_DR3);
 //     DisplayDateFlag=1;
    }
//     else
//     {
//      s_AlarmDateStructVar.Day = Day;
//      s_AlarmDateStructVar.Month = Month;
//      s_AlarmDateStructVar.Year = Year;
//      BKP_WriteBackupRegister(BKP_DR8,Month);
//      BKP_WriteBackupRegister(BKP_DR9,Day);
//      BKP_WriteBackupRegister(BKP_DR10,Year);
//      AlarmDate=0;
//     }
  }
}



/**
  * @brief  RTC Application runs in while loop
  * @param  None
  * @retval : None
  */
// void RTC_Application(void)
// {
//   uint8_t ReturnValue;

//   CalculateTime();
//   
//   if(TimeDateDisplay==0)
//   {
//     DisplayTime();
//   }

//   if(DisplayDateFlag==1 && TimeDateDisplay==0)
//   {
//     DisplayDate();
//     DisplayDateFlag=0;
//   }

//   if(AlarmStatus==1)
//   {
//     DisplayAlarm();
//     GPIO_SetBits(GPIOC, GPIO_Pin_6);
//     Dummy=RTC_GetCounter();
//     AlarmStatus=2;
//   }
//   if(((RTC_GetCounter()-Dummy) == 4) && (AlarmStatus==2))
//   {
//     GPIO_ResetBits(GPIOC, GPIO_Pin_6);
//     SelIntExtOnOffConfig(ENABLE);
//     RightLeftIntExtOnOffConfig(ENABLE);
//     UpDownIntOnOffConfig(ENABLE);
//     MenuInit();
//     Dummy=0;
//     AlarmStatus = 0;
//   }

//   if(TamperEvent == 1) /* Tamper event is detected */
//   {
//     TamperNumber = TamperNumber + 1;
//     BKP_WriteBackupRegister(BKP_DR5,TamperNumber);
//     BKP_WriteBackupRegister(BKP_DR1, CONFIGURATION_DONE);
//     BKP_WriteBackupRegister(BKP_DR2,s_DateStructVar.Month);
//     BKP_WriteBackupRegister(BKP_DR3,s_DateStructVar.Day);
//     BKP_WriteBackupRegister(BKP_DR4,s_DateStructVar.Year);
//     BKP_WriteBackupRegister(BKP_DR9,s_AlarmDateStructVar.Day);
//     BKP_WriteBackupRegister(BKP_DR8,s_AlarmDateStructVar.Month);
//     BKP_WriteBackupRegister(BKP_DR10,s_AlarmDateStructVar.Year);
//     BKP_WriteBackupRegister(BKP_DR6,ClockSource);
//     BKP_WriteBackupRegister(BKP_DR7,SummerTimeCorrect);
//     ReturnValue=EE_WriteVariable(TamperNumber);
//     
//     if(ReturnValue==PAGE_FULL)
//     {
//       ReturnValue=EE_WriteVariable(TamperNumber);
//     }
//     
//     ReturnValue=EE_WriteVariable(s_DateStructVar.Day);
//     ReturnValue=EE_WriteVariable(s_DateStructVar.Month);
//     ReturnValue=EE_WriteVariable((s_DateStructVar.Year)/100);
//     ReturnValue=EE_WriteVariable((s_DateStructVar.Year)%100);
//     ReturnValue=EE_WriteVariable((s_TimeStructVar.HourHigh*10)+\
//                     s_TimeStructVar.HourLow);
//     ReturnValue=EE_WriteVariable((s_TimeStructVar.MinHigh*10)+\
//                     s_TimeStructVar.MinLow);
//     ReturnValue=EE_WriteVariable((s_TimeStructVar.SecHigh*10)+\
//                     s_TimeStructVar.SecLow);
//     while(!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)))
//     {
//     }
//     
//     TamperEvent=0;
//     BKP_TamperPinCmd(ENABLE);
//   }
//   
//   if(BatteryRemoved != 0) /* Battery tamper is detected */
//   {
//     TamperNumber = TamperNumber + 1;
//     BKP_WriteBackupRegister(BKP_DR5,TamperNumber);
//     ReturnValue=EE_WriteVariable(TamperNumber);
//     
//     if(ReturnValue==PAGE_FULL)
//     {
//       ReturnValue=EE_WriteVariable(TamperNumber);
//     }
//     
//     ReturnValue=EE_WriteVariable(s_DateStructVar.Day);
//     ReturnValue=EE_WriteVariable(s_DateStructVar.Month);
//     ReturnValue=EE_WriteVariable((s_DateStructVar.Year)/100);
//     ReturnValue=EE_WriteVariable((s_DateStructVar.Year)%100);
//     ReturnValue=EE_WriteVariable((s_TimeStructVar.HourHigh*10)\
//                     +s_TimeStructVar.HourLow);
//     ReturnValue=EE_WriteVariable((s_TimeStructVar.MinHigh*10)\
//                     +s_TimeStructVar.MinLow);
//     if(BatteryRemoved==1)
//     {
//       ReturnValue=EE_WriteVariable(BATTERY_REMOVED);
//       BatteryRemoved=0;
//     }
//     else if(BatteryRemoved==2)
//     {
//       ReturnValue=EE_WriteVariable(BATTERY_RESTORED);
//       BatteryRemoved=0;
//     }

//     BKP_TamperPinCmd(ENABLE);
//   }
// }



/**
  * @brief  This function is executed after wakeup from STOP mode
  * @param  None
  * @retval : None
  */
// void ReturnFromStopMode(void)
// {
//   /* RCC Configuration has to be called after waking from STOP Mode*/
//   RCC_Configuration();
//   /*Enables the clock to Backup and power interface peripherals after Wake Up */
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR,ENABLE);
//   /* Enable access to Backup Domain */
//   PWR_BackupAccessCmd(ENABLE);
//   /* LCD Reinitialisation */
//   STM3210B_LCD_Init();
//   /* LED D2 goes off */
//   GPIO_ResetBits(GPIOC, GPIO_Pin_9); 
//   /* Enable Sel interrupt */
//   SelIntExtOnOffConfig(ENABLE);
//   /* Menu initialisation */
//   MenuInit();
//   /* Time display enable */
//   TimeDateDisplay=0;
//   /* Since Sel is used to exit from STOP mode, hence when STOP mode is exited
//   initial value of MenuLevelPointer is 0 */
//   MenuLevelPointer=0xFF;
// }



/**
  * @brief Updates the Date (This function is called when 1 Day has elapsed
  * @param None
  * @retval :None
  */
void DateUpdate(void)
{
  s_DateStructVar.Month=BKP_ReadBackupRegister(BKP_DR2);
  s_DateStructVar.Year=BKP_ReadBackupRegister(BKP_DR4);
  s_DateStructVar.Day=BKP_ReadBackupRegister(BKP_DR3);
  
  if(s_DateStructVar.Month == 1 || s_DateStructVar.Month == 3 || \
    s_DateStructVar.Month == 5 || s_DateStructVar.Month == 7 ||\
     s_DateStructVar.Month == 8 || s_DateStructVar.Month == 10 \
       || s_DateStructVar.Month == 12)
  {
    if(s_DateStructVar.Day < 31)
    {
      s_DateStructVar.Day++;
    }
    /* Date structure member: s_DateStructVar.Day = 31 */
    else
    {
      if(s_DateStructVar.Month != 12)
      {
        s_DateStructVar.Month++;
        s_DateStructVar.Day = 1;
      }
     /* Date structure member: s_DateStructVar.Day = 31 & s_DateStructVar.Month =12 */
      else
      {
        s_DateStructVar.Month = 1;
        s_DateStructVar.Day = 1;
        s_DateStructVar.Year++;
      }
    }
  }
  else if(s_DateStructVar.Month == 4 || s_DateStructVar.Month == 6 \
            || s_DateStructVar.Month == 9 ||s_DateStructVar.Month == 11)
  {
    if(s_DateStructVar.Day < 30)
    {
      s_DateStructVar.Day++;
    }
    /* Date structure member: s_DateStructVar.Day = 30 */
    else
    {
      s_DateStructVar.Month++;
      s_DateStructVar.Day = 1;
    }
  }
  else if(s_DateStructVar.Month == 2)
  {
    if(s_DateStructVar.Day < 28)
    {
      s_DateStructVar.Day++;
    }
    else if(s_DateStructVar.Day == 28)
    {
      /* Leap Year Correction */
      if(CheckLeap(s_DateStructVar.Year))
      {
        s_DateStructVar.Day++;
      }
      else
      {
        s_DateStructVar.Month++;
        s_DateStructVar.Day = 1;
      }
    }
    else if(s_DateStructVar.Day == 29)
    {
      s_DateStructVar.Month++;
      s_DateStructVar.Day = 1;
    }
  }
  
  BKP_WriteBackupRegister(BKP_DR2,s_DateStructVar.Month);
  BKP_WriteBackupRegister(BKP_DR3,s_DateStructVar.Day);
  BKP_WriteBackupRegister(BKP_DR4,s_DateStructVar.Year);
//   DisplayDateFlag=1;
}



/**
  * @brief  Checks whether the passed year is Leap or not.
  * @param  None
  * @retval : 1: leap year
  *   0: not leap year
  */
uint8_t CheckLeap(uint16_t Year)
{
  if((Year%400)==0)
  {
    return LEAP;
  }
  else if((Year%100)==0)
  {
    return NOT_LEAP;
  }
  else if((Year%4)==0)
  {
    return LEAP;
  }
  else
  {
    return NOT_LEAP;
  }
}



/**
  * @brief Determines the weekday
  * @param Year,Month and Day
  * @retval :Returns the CurrentWeekDay Number 0- Sunday 6- Saturday
  */
uint16_t WeekDay(uint16_t CurrentYear,uint8_t CurrentMonth,uint8_t CurrentDay)
{
  uint16_t Temp1,Temp2,Temp3,Temp4,CurrentWeekDay;
  
  if(CurrentMonth < 3)
  {
    CurrentMonth=CurrentMonth + 12;
    CurrentYear=CurrentYear-1;
  }
  
  Temp1=(6*(CurrentMonth + 1))/10;
  Temp2=CurrentYear/4;
  Temp3=CurrentYear/100;
  Temp4=CurrentYear/400;
  CurrentWeekDay=CurrentDay + (2 * CurrentMonth) + Temp1 \
     + CurrentYear + Temp2 - Temp3 + Temp4 +1;
  CurrentWeekDay = CurrentWeekDay % 7;
  
  return(CurrentWeekDay);
}



// /**
//   * @brief Displays the Date(DD/MM/YY and DAY ) on LCD
//   * @param None
//   * @retval :None
//   */
// void DisplayDate(void)
// {
//   s_DateStructVar.Year=(BKP_ReadBackupRegister(BKP_DR4));
//   s_DateStructVar.Month=(BKP_ReadBackupRegister(BKP_DR2));
//   s_DateStructVar.Day=(BKP_ReadBackupRegister(BKP_DR3));
//   LCD_DisplayChar(Line5,Column5,(s_DateStructVar.Day/10) + 0x30);
//   LCD_DisplayChar(Line5,Column6,(s_DateStructVar.Day%10) + 0x30);
//   LCD_DisplayString(Line5,Column8,MonthsNames[s_DateStructVar.Month-1]);
//   LCD_DisplayChar(Line5,Column12,((s_DateStructVar.Year/1000) + 0x30));
//   LCD_DisplayChar(Line5,Column13,((s_DateStructVar.Year/100)%10) + 0x30);
//   LCD_DisplayChar(Line5,Column14,((s_DateStructVar.Year/10)%10) + 0x30);
//   LCD_DisplayChar(Line5,Column15,(s_DateStructVar.Year%10) + 0x30);
//   
//   switch(WeekDay(s_DateStructVar.Year,s_DateStructVar.Month,s_DateStructVar.Day))
//   {
//     case 0: LCD_DisplayString(Line6,Column8,"Sun");
//             break;
//     case 1: LCD_DisplayString(Line6,Column8,"Mon");
//             break;
//     case 2: LCD_DisplayString(Line6,Column8,"Tue");
//             break;
//     case 3: LCD_DisplayString(Line6,Column8,"Wed");
//             break;
//     case 4: LCD_DisplayString(Line6,Column8,"Thu");
//             break;
//     case 5: LCD_DisplayString(Line6,Column8,"Fri");
//             break;
//     case 6: LCD_DisplayString(Line6,Column8,"Sat");
//             break;
//   }
// }



// /**
//   * @brief Displays the Time on LCD
//   * @param None
//   * @retval :None
//   */
// void DisplayTime(void)
// {
//   LCD_DisplayChar(Line4,Column6,s_TimeStructVar.HourHigh + 0x30);
//   LCD_DisplayChar(Line4,Column7,s_TimeStructVar.HourLow + 0x30);
//   LCD_DisplayChar(Line4,Column9,s_TimeStructVar.MinHigh + 0x30);
//   LCD_DisplayChar(Line4,Column10,s_TimeStructVar.MinLow + 0x30);
//   LCD_DisplayChar(Line4,Column12,s_TimeStructVar.SecHigh + 0x30);
//   LCD_DisplayChar(Line4,Column13,s_TimeStructVar.SecLow + 0x30);
// }



/**
  * @brief  Displays Alarm on TFT LCD
  * @param  None
  * @retval : None
  */
// void DisplayAlarm(void)
// {
//   SelIntExtOnOffConfig(DISABLE);
//   RightLeftIntExtOnOffConfig(DISABLE);
//   UpDownIntOnOffConfig(DISABLE);
//   TimeDateDisplay=1;
//   LCD_Clear(Red);
//   LCD_SetTextColor(Yellow);
//   LCD_SetBackColor(Red);
//   LCD_DisplayString(Line4,Column8,"ALARM");
// }



/**
  * @brief Calcuate the Time (in hours, minutes and seconds  derived from
  *   COunter value
  * @param None
  * @retval :None
  */
void CalculateTime(void)
{
  uint32_t TimeVar;
  
  TimeVar=RTC_GetCounter();
  TimeVar=TimeVar % 86400;
  s_TimeStructVar.HourHigh=(uint8_t)(TimeVar/3600)/10;
  s_TimeStructVar.HourLow=(uint8_t)(TimeVar/3600)%10;
  s_TimeStructVar.MinHigh=(uint8_t)((TimeVar%3600)/60)/10;
  s_TimeStructVar.MinLow=(uint8_t)((TimeVar%3600)/60)%10;
  s_TimeStructVar.SecHigh=(uint8_t)((TimeVar%3600)%60)/10;
  s_TimeStructVar.SecLow=(uint8_t)((TimeVar %3600)%60)%10;
}



/**
  * @brief Chaeks is counter value is more than 86399 and the number of
  *   elapsed and updates date that many times
  * @param None
  * @retval :None
  */
void CheckForDaysElapsed(void)
{
  uint8_t DaysElapsed;
 
  if((RTC_GetCounter() / SECONDS_IN_DAY) != 0)
  {
    for(DaysElapsed = 0; DaysElapsed < (RTC_GetCounter() / SECONDS_IN_DAY)\
         ;DaysElapsed++)
    {
      DateUpdate();
    }

    RTC_SetCounter(RTC_GetCounter() % SECONDS_IN_DAY);
  }
}



/**
  * @brief  Calibration of External crystal oscillator manually
  * @param  None
  * @retval : None
  */
// void ManualClockCalibration(void)
// {
//   UpDownIntOnOffConfig(ENABLE);
//   RightLeftIntExtOnOffConfig(ENABLE);
//   SelIntExtOnOffConfig(DISABLE);
//   LCD_Clear(Blue2);
//   BKP_TamperPinCmd(DISABLE);
//   BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
//   LCD_DisplayString(Line1,Column5,"Calibration");
//   LCD_DisplayString(Line3,Column0,"LSE/64 is available");
//   LCD_DisplayString(Line4,Column0,"on PC13.Measure the");
//   LCD_DisplayString(Line5,Column0,"the frequency and");
//   LCD_DisplayString(Line6,Column0,"press Sel to proceed");
//   
//   while(ReadKey()!=SEL)
//   {
//   }
//   
//   LCD_Clear(Blue2);
//   BKP_RTCOutputConfig(BKP_RTCOutputSource_None);
//   LCD_DisplayString(Line1,Column4,"Please enter");
//   LCD_DisplayString(Line2,Column2,"Calibration Value");
//   LCD_DisplayCount(Line4,Column6,1,ArrayTime[0]+0x30);
//   LCD_DisplayCount(Line4,Column7,0,ArrayTime[1]+0x30);
//   LCD_DisplayCount(Line4,Column8,0,ArrayTime[2]+0x30);
//   LCD_DisplayString(Line6,Column6,"(0-121)");
//   LCD_SetBackColor(Green);
//   SelIntExtOnOffConfig(ENABLE);
// }



/**
  * @brief  Calibration of External crystal oscillator auto(through Timer
  *   
  * @param  None
  * @retval : None
  */
// void AutoClockCalibration(void)
// {
//   RCC_ClocksTypeDef ClockValue;
//   uint16_t TimerPrescalerValue=0x0003;
//   uint16_t CountWait;
//   uint16_t DeviationInteger;
//   uint32_t CalibrationTimer;
//   float f32_Deviation;
//   TIM_ICInitTypeDef  TIM_ICInitStructure;
//   
//   TIM_DeInit(TIM2);
//   BKP_TamperPinCmd(DISABLE);
//   BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
//   
//   /* TIM2 configuration: PWM Input mode ------------------------
//      The external signal is connected to TIM2 CH2 pin (PA.01),
//      The Rising edge is used as active edge,
//      The TIM2 CCR2 is used to compute the frequency value
//      The TIM2 CCR1 is used to compute the duty cycle value
//   ------------------------------------------------------------ */
//   TIM_PrescalerConfig(TIM2,TimerPrescalerValue,TIM_PSCReloadMode_Immediate);
//   TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//   TIM_ICInitStructure.TIM_ICFilter = 0x00;
//   TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);
//   TIM_ICInit(TIM2, &TIM_ICInitStructure);
//   /* Select the TIM2 Input Trigger: TI2FP2 */
//   TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
//   /* Select the slave Mode: Reset Mode */
//   TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
//   /* Enable the Master/Slave Mode */
//   TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
//   /* TIM enable Counter */
//   TIM_Cmd(TIM2, ENABLE);
//   LCD_Clear(Blue2);
//   LCD_DisplayString(Line4,Column1,"Please Wait.....");
//   /* Wait for 2 seconds */
//   CalibrationTimer = RTC_GetCounter();
//  
//   while((RTC_GetCounter() - CalibrationTimer) < 2)
//   {
//   }
//   
//   RCC_GetClocksFreq(&ClockValue);
//   TimerFrequency=(ClockValue.PCLK1_Frequency * 2)/(TimerPrescalerValue+1);
//    /* Enable the CC2 Interrupt Request */
//   TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
//    /* Wait for 2 seconds */
//   CalibrationTimer = RTC_GetCounter();
//   
//   while((RTC_GetCounter() - CalibrationTimer) < 2)
//   {
//   }

//   if(!(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC1)))
//    /* There is no signal at the timer TIM2 peripheral input */
//   {
//     LCD_Clear(Blue2);
//     LCD_DisplayString(Line3,Column0,"Please connect wire");
//     LCD_DisplayString(Line4,Column0,"link between PC13");
//     LCD_DisplayString(Line5,Column0,"and PA1");
//     LCD_DisplayString(Line7,Column0,"No calibration done");
//   }
//   else
//   {
//     /* Calulate Deviation in ppm  using the formula :
//     Deviation in ppm = (Deviation from 511.968/511.968)*1 million*/
//     if(f32_Frequency > 511.968)
//     {
//       f32_Deviation=((f32_Frequency-511.968)/511.968)*1000000;
//     }
//     else
//     {
//       f32_Deviation=((511.968-f32_Frequency)/511.968)*1000000;
//     }
//      DeviationInteger = (uint16_t)f32_Deviation;
//     
//     if(f32_Deviation >= (DeviationInteger + 0.5))
//     {
//       DeviationInteger = ((uint16_t)f32_Deviation)+1;
//     }
//     
//    CountWait=0;

//    /* Frequency deviation in ppm should be les than equal to 121 ppm*/
//    if(DeviationInteger <= 121)
//    {
//      while(CountWait<128)
//      {
//        if(CalibrationPpm[CountWait] == DeviationInteger)
//        break;
//        CountWait++;
//      }

//      BKP_SetRTCCalibrationValue(CountWait);
//      LCD_Clear(Blue2);
//      LCD_DisplayString(Line4,Column1,"Calibration Value");
//      LCD_DisplayChar(Line5,Column10,(CountWait%10)+0x30);
//      CountWait=CountWait/10;
//      LCD_DisplayChar(Line5,Column9,(CountWait%10)+0x30);
//      CountWait=CountWait/10;
//    
//      if(CountWait>0)
//      {
//        LCD_DisplayChar(Line5,Column8,(CountWait%10)+0x30);
//      }
//    }
//    else /* Frequency deviation in ppm is more than 121 ppm, hence calibration
//            can not be done */
//    {
//      LCD_Clear(Blue2);
//      LCD_DisplayString(Line3,Column1,"Out Of Calibration");
//      LCD_DisplayString(Line4,Column4,"Range");
//    }
//   }
//   
//   BKP_RTCOutputConfig(BKP_RTCOutputSource_None);
//   TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
//   TIM_Cmd(TIM2, DISABLE);
//   TIM_DeInit(TIM2);
//   CalibrationTimer=RTC_GetCounter();
//   
//   /*  Wait for 2 seconds  */
//   while((RTC_GetCounter() - CalibrationTimer) < 5)
//   {
//   }
//   
//   MenuInit();
// }



/**
  * @brief  Configures the Tamper Interrupts
  * @param  None
  * @retval : None
  */
// void Tamper_NVIC_Configuration(void)
// {
//   NVIC_InitTypeDef NVIC_InitStructure;
//   NVIC_InitStructure.NVIC_IRQChannel = TAMPER_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);
// }



/**
  * @brief  Configures RTC Interrupts
  * @param  None
  * @retval : None
  */
// void RTC_NVIC_Configuration(void)
// {
//   NVIC_InitTypeDef NVIC_InitStructure;
//   EXTI_InitTypeDef EXTI_InitStructure;
//   
//   EXTI_DeInit();

//   /* Configure one bit for preemption priority */
//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//   /* Enable the RTC Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);

//   /* Enable the EXTI Line17 Interrupt */
//   EXTI_ClearITPendingBit(EXTI_Line17);
//   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//   EXTI_InitStructure.EXTI_Line = EXTI_Line17;
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//   EXTI_Init(&EXTI_InitStructure);

//   /* Enable the RTC Alarm Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);

//   EXTI_ClearITPendingBit(EXTI_Line16 );
//   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//   EXTI_InitStructure.EXTI_Line = EXTI_Line16;
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//   EXTI_Init(&EXTI_InitStructure);

//   NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_Init(&NVIC_InitStructure);

//   /* Enable the TIM2 global Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);
// }

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

void SaveWorkTime(void)
{
	BKP_WriteBackupRegister(BKP_DR11, (s_WorkTimeVar[0].StartHour << 8) + s_WorkTimeVar[0].StartMin );
	BKP_WriteBackupRegister(BKP_DR12, (s_WorkTimeVar[0].EndHour << 8) + s_WorkTimeVar[0].EndMin );
	
	BKP_WriteBackupRegister(BKP_DR13, (s_WorkTimeVar[1].StartHour << 8) + s_WorkTimeVar[1].StartMin );
	BKP_WriteBackupRegister(BKP_DR14, (s_WorkTimeVar[1].EndHour << 8) + s_WorkTimeVar[1].EndMin );
	
	BKP_WriteBackupRegister(BKP_DR15, (s_WorkTimeVar[2].StartHour << 8) + s_WorkTimeVar[2].StartMin );
	BKP_WriteBackupRegister(BKP_DR16, (s_WorkTimeVar[2].EndHour << 8) + s_WorkTimeVar[2].EndMin );
	
	BKP_WriteBackupRegister(BKP_DR17, (s_WorkTimeVar[3].StartHour << 8) + s_WorkTimeVar[3].StartMin );
	BKP_WriteBackupRegister(BKP_DR18, (s_WorkTimeVar[3].EndHour << 8) + s_WorkTimeVar[3].EndMin );
	
	BKP_WriteBackupRegister(BKP_DR19, (s_WorkTimeVar[4].StartHour << 8) + s_WorkTimeVar[4].StartMin );
	BKP_WriteBackupRegister(BKP_DR20, (s_WorkTimeVar[4].EndHour << 8) + s_WorkTimeVar[4].EndMin );
	
	BKP_WriteBackupRegister(BKP_DR21, (s_WorkTimeVar[5].StartHour << 8) + s_WorkTimeVar[5].StartMin );
	BKP_WriteBackupRegister(BKP_DR22, (s_WorkTimeVar[5].EndHour << 8) + s_WorkTimeVar[5].EndMin );
	
	BKP_WriteBackupRegister(BKP_DR23, (s_WorkTimeVar[6].StartHour << 8) + s_WorkTimeVar[6].StartMin );
	BKP_WriteBackupRegister(BKP_DR24, (s_WorkTimeVar[6].EndHour << 8) + s_WorkTimeVar[6].EndMin );

}

void LoadWorkTime(void)
{
	s_WorkTimeVar[0].StartHour = BKP_ReadBackupRegister(BKP_DR11)>>8;
	s_WorkTimeVar[0].StartMin = BKP_ReadBackupRegister(BKP_DR11) & 0x00FF;
	s_WorkTimeVar[0].EndHour = BKP_ReadBackupRegister(BKP_DR12)>>8;
	s_WorkTimeVar[0].EndMin = BKP_ReadBackupRegister(BKP_DR12)& 0x00FF;
	
	s_WorkTimeVar[1].StartHour = BKP_ReadBackupRegister(BKP_DR13)>>8;
	s_WorkTimeVar[1].StartMin = BKP_ReadBackupRegister(BKP_DR13) & 0x00FF;
	s_WorkTimeVar[1].EndHour = BKP_ReadBackupRegister(BKP_DR14)>>8;
	s_WorkTimeVar[1].EndMin = BKP_ReadBackupRegister(BKP_DR14)& 0x00FF;
	
	s_WorkTimeVar[2].StartHour = BKP_ReadBackupRegister(BKP_DR15)>>8;
	s_WorkTimeVar[2].StartMin = BKP_ReadBackupRegister(BKP_DR15) & 0x00FF;
	s_WorkTimeVar[2].EndHour = BKP_ReadBackupRegister(BKP_DR16)>>8;
	s_WorkTimeVar[2].EndMin = BKP_ReadBackupRegister(BKP_DR16)& 0x00FF;
	
	s_WorkTimeVar[3].StartHour = BKP_ReadBackupRegister(BKP_DR17)>>8;
	s_WorkTimeVar[3].StartMin = BKP_ReadBackupRegister(BKP_DR17) & 0x00FF;
	s_WorkTimeVar[3].EndHour = BKP_ReadBackupRegister(BKP_DR18)>>8;
	s_WorkTimeVar[3].EndMin = BKP_ReadBackupRegister(BKP_DR18)& 0x00FF;
	
	s_WorkTimeVar[4].StartHour = BKP_ReadBackupRegister(BKP_DR19)>>8;
	s_WorkTimeVar[4].StartMin = BKP_ReadBackupRegister(BKP_DR19) & 0x00FF;
	s_WorkTimeVar[4].EndHour = BKP_ReadBackupRegister(BKP_DR20)>>8;
	s_WorkTimeVar[4].EndMin = BKP_ReadBackupRegister(BKP_DR20)& 0x00FF;
	
	s_WorkTimeVar[5].StartHour = BKP_ReadBackupRegister(BKP_DR21)>>8;
	s_WorkTimeVar[5].StartMin = BKP_ReadBackupRegister(BKP_DR21) & 0x00FF;
	s_WorkTimeVar[5].EndHour = BKP_ReadBackupRegister(BKP_DR22)>>8;
	s_WorkTimeVar[5].EndMin = BKP_ReadBackupRegister(BKP_DR22)& 0x00FF;
	
	s_WorkTimeVar[6].StartHour = BKP_ReadBackupRegister(BKP_DR23)>>8;
	s_WorkTimeVar[6].StartMin = BKP_ReadBackupRegister(BKP_DR23) & 0x00FF;
	s_WorkTimeVar[6].EndHour = BKP_ReadBackupRegister(BKP_DR24)>>8;
	s_WorkTimeVar[6].EndMin = BKP_ReadBackupRegister(BKP_DR24)& 0x00FF;

}

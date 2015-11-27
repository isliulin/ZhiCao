#include "stm32f10x.h"
#include "misc.h"
#include "Lcd_Driver.h"
#include "SysTickDelay.h"
#include "GUI.h"
#include "key.h"
#include "Encoder.h"
#include "TSL1401CCD.h"
#include "controller.h"

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void RCC_Configuration(void);
/* Private functions ---------------------------------------------------------*/

// extern uint8_t TimeSecTick;


u8 SYS_Status;   //0-stop,1-pause,2-run
u8 CCD_Pixel_Last[128];  //保存CCD数据
bool bPausedFlag[4];
bool bShowLine;
int main(void)
{	
	u8 i,j;
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();

	Key_Config();
	Lcd_Init();
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK32(16,6,BLUE,GRAY0,"长度：");
	Gui_DrawFont_Num32_Int(112,6,RED,GRAY0,0,4);	
	Gui_DrawFont_GBK32(256,6,BLUE,GRAY0,"米");
	Gui_DrawLine(95,49,224,49,BLACK);
	Gui_DrawLine(95,49,95,178,BLACK);
	Gui_DrawLine(96,178,224,178,BLACK);
	Gui_DrawLine(224,49,224,178,BLACK);

	ENC_Init();
	CCD_Init();
	Controller_Init(10);
	j=i=0;
	SYS_Status = 1;
	bShowLine = TRUE;
	while(1)
	{

// 		if(Key_Press(KEY_UP)) ENC_Init();
// 		if(Key_Press(KEY_DOWN)) i=10;
// 		if(Key_Press(KEY_LEFT)) i=100;
// 		if(Key_Press(KEY_RIGHT)) i=1000;

// 		if(Key_Press(KEY_OK)) i=20000;
		
		while(!Key_Press(KEY_STOP))
		{
// 			SYS_Status=1;
			;
		}
		
		//显示长度
		Gui_DrawFont_Num32_Int(112,6,RED,GRAY0,nCurrentMeter,4);
		
		if(Key_Press(KEY_OK) && Key_Press(KEY_SET))
		{
			bShowLine = (bool)!bShowLine;
		}
		
		if(bShowLine)
		{
			//清除上次画的曲线
				for(i=0;i<128;i++)
				{
					j=CCD_Pixel_Last[i]>>1;
					Gui_DrawPoint(96+i,177-j,GRAY0);
				}
				//绘制CCD曲线
				for(i=0;i<128;i++)
				{
					CCD_Pixel_Last[i]=CCD_Pixel[i];
				}
				for(i=0;i<128;i++)
				{
					j=CCD_Pixel_Last[i]>>1;
					Gui_DrawPoint(96+i,177-j,RED);
				}		
		}
		
		if(SYS_Status !=2)
			LED_ON;
		else
			LED_OFF;
		
		switch(SYS_Status)
		{
			case 0:
				Controller_Stop();
				Gui_DrawFont_GBK32(16,100,BLUE,GRAY0,"断线");
				Gui_DrawFont_GBK32(16,200,BLUE,GRAY0,"按确认键继续");
				if(Key_Press(KEY_OK))
				{
					Gui_DrawDefaultScreen();
					SYS_Status=2;
				}
				if(Key_Press(KEY_SET))
				{
					ENC_Init();
					nCurrentMeter = ENC_ReadCNT();
					Gui_DrawFont_Num32_Int(112,6,RED,GRAY0,nCurrentMeter,4);
					for(i=0;i<4;i++)
					{
						bPausedFlag[i]=FALSE;
					}
				}
				break;
			
			case 1:
				Controller_Stop();
				Gui_DrawFont_GBK32(16,100,BLUE,GRAY0,"暂停");
				Gui_DrawFont_GBK32(16,200,BLUE,GRAY0,"按确认键继续");
				if(Key_Press(KEY_OK))
				{
					Gui_DrawDefaultScreen();
					SYS_Status=2;
				}
				if(Key_Press(KEY_SET))
				{
					ENC_Init();
					nCurrentMeter = ENC_ReadCNT();
					Gui_DrawFont_Num32_Int(112,6,RED,GRAY0,nCurrentMeter,4);
					for(i=0;i<4;i++)
					{
						bPausedFlag[i]=FALSE;
					}
				}
				break;
				
			case 2:
				if(Key_Press(KEY_SET))
				{
					SYS_Status = 1;
				}		

				if(Controller_is_Line_Detected())
				{
					;
				}
				else
				{
					SYS_Status = 0;
				}
				
				

				//模拟编码器信号
				GPIOB->BSRR = GPIO_Pin_10;
				Delay_ms(2);
				GPIOB->BSRR = GPIO_Pin_11;
				Delay_ms(2);

				GPIOB->BRR = GPIO_Pin_10;
				Delay_ms(2);
				GPIOB->BRR = GPIO_Pin_11;
				Delay_ms(2);
				//模拟编码器信号
				
				
				for(i=0;i<4;i++)
				{
					if(!bPausedFlag[i] && nCurrentMeter > nPauseMeter[i])
					{
						SYS_Status = 1;
						bPausedFlag[i] = TRUE;
					}
				}
				
				if(nCurrentMeter< nPauseMeter[0])
					Controller_SetSpeed(0);
				else if (nCurrentMeter< nPauseMeter[1])
					Controller_SetSpeed(1);
				else if(nCurrentMeter< nPauseMeter[2])
					Controller_SetSpeed(2);
				else
					Controller_SetSpeed(3);
				
				
				if(nCurrentMeter > nStopMeter)
				{
					SYS_Status = 1;
				}
				
				break;
				
				default:
					break;

		
		//Delay_ms(100);
		
		
		//Gui_DrawFont_Num32_Int(112,10,RED,GRAY0,i++,5);

		}

	}
	

}

void RCC_Configuration(void)
{ 
        RCC_DeInit();

//        SystemInit();//源自system_stm32f10x.c文件,只需要调用此函数,则可完成RCC的配置.具体请看2_RCC
        RCC_HSEConfig(RCC_HSE_OFF);

        RCC_HSICmd(ENABLE);                        //打开内部时钟
        
        while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)        
        {        
        }

//        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

//        FLASH_SetLatency(FLASH_Latency_2);
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        //APB2
        RCC_PCLK2Config(RCC_HCLK_Div1);
        //APB1
        RCC_PCLK1Config(RCC_HCLK_Div1);
        //PLL 倍频
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);        //内部时钟倍频*16
        RCC_PLLCmd(ENABLE);                        //使能倍频
                                                                                                         
   //等待指定的 RCC 标志位设置成功 等待PLL初始化成功
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);        

        /**************************************************
        获取RCC的信息,调试用
        请参考RCC_ClocksTypeDef结构体的内容,当时钟配置完成后,
        里面变量的值就直接反映了器件各个部分的运行频率
        ***************************************************/
                
        while(RCC_GetSYSCLKSource() != 0x08){}
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
// 	EXTI_InitTypeDef EXTI_InitStructure;	

  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* 优先级组 说明了抢占优先级所用的位数，和子优先级所用的位数   在这里是1， 7 */    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		   
  
  /* Enable the USART1 Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//设置串口1中断
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//抢占优先级 0
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为0
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
//   NVIC_Init(&NVIC_InitStructure);
	
	  /* Enable the USART2 Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			     	//设置串口2中断
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//抢占优先级 0
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为0
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
//   NVIC_Init(&NVIC_InitStructure);
	
	
	  /* Enable the RTC Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;					//配置外部中断源（秒中断） 
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);  
	
// 	//NRF24L01中断配置
//   NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;					//NRF24L01 中断响应
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		    //抢占优先级 0
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//子优先级为1
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
//   NVIC_Init(&NVIC_InitStructure);		

// 			/*使能外部中断要用到的AFIO的时钟*/
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	

//   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);	   //NRF24L01 IRQ  PA0
//   
//   EXTI_InitStructure.EXTI_Line = EXTI_Line5;					   //NRF24L01 IRQ PA0
//   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			   //EXTI中断
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		   //下降沿触发
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;						   //使能
//   EXTI_Init(&EXTI_InitStructure);	
// 	//NRF24L01中断配置结束
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
		if (SysTick_Config(640))		   //时钟节拍中断时10us一次  用于定时 
		{ 
			/* Capture error */ 
			while (1);
		} 
}


void GPIO_Configuration(void){
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	

	//PB10,PB11模拟编码器输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
}


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
u8 CCD_Pixel_Last[128];  //����CCD����
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
	Gui_DrawFont_GBK32(16,6,BLUE,GRAY0,"���ȣ�");
	Gui_DrawFont_Num32_Int(112,6,RED,GRAY0,0,4);	
	Gui_DrawFont_GBK32(256,6,BLUE,GRAY0,"��");
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
		
		//��ʾ����
		Gui_DrawFont_Num32_Int(112,6,RED,GRAY0,nCurrentMeter,4);
		
		if(Key_Press(KEY_OK) && Key_Press(KEY_SET))
		{
			bShowLine = (bool)!bShowLine;
		}
		
		if(bShowLine)
		{
			//����ϴλ�������
				for(i=0;i<128;i++)
				{
					j=CCD_Pixel_Last[i]>>1;
					Gui_DrawPoint(96+i,177-j,GRAY0);
				}
				//����CCD����
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
				Gui_DrawFont_GBK32(16,100,BLUE,GRAY0,"����");
				Gui_DrawFont_GBK32(16,200,BLUE,GRAY0,"��ȷ�ϼ�����");
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
				Gui_DrawFont_GBK32(16,100,BLUE,GRAY0,"��ͣ");
				Gui_DrawFont_GBK32(16,200,BLUE,GRAY0,"��ȷ�ϼ�����");
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
				
				

				//ģ��������ź�
				GPIOB->BSRR = GPIO_Pin_10;
				Delay_ms(2);
				GPIOB->BSRR = GPIO_Pin_11;
				Delay_ms(2);

				GPIOB->BRR = GPIO_Pin_10;
				Delay_ms(2);
				GPIOB->BRR = GPIO_Pin_11;
				Delay_ms(2);
				//ģ��������ź�
				
				
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

//        SystemInit();//Դ��system_stm32f10x.c�ļ�,ֻ��Ҫ���ô˺���,������RCC������.�����뿴2_RCC
        RCC_HSEConfig(RCC_HSE_OFF);

        RCC_HSICmd(ENABLE);                        //���ڲ�ʱ��
        
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
        //PLL ��Ƶ
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);        //�ڲ�ʱ�ӱ�Ƶ*16
        RCC_PLLCmd(ENABLE);                        //ʹ�ܱ�Ƶ
                                                                                                         
   //�ȴ�ָ���� RCC ��־λ���óɹ� �ȴ�PLL��ʼ���ɹ�
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);        

        /**************************************************
        ��ȡRCC����Ϣ,������
        ��ο�RCC_ClocksTypeDef�ṹ�������,��ʱ��������ɺ�,
        ���������ֵ��ֱ�ӷ�ӳ�������������ֵ�����Ƶ��
        ***************************************************/
                
        while(RCC_GetSYSCLKSource() != 0x08){}
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
// 	EXTI_InitTypeDef EXTI_InitStructure;	

  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* ���ȼ��� ˵������ռ���ȼ����õ�λ�����������ȼ����õ�λ��   ��������1�� 7 */    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		   
  
  /* Enable the USART1 Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//���ô���1�ж�
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//��ռ���ȼ� 0
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//�����ȼ�Ϊ0
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ��
//   NVIC_Init(&NVIC_InitStructure);
	
	  /* Enable the USART2 Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			     	//���ô���2�ж�
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//��ռ���ȼ� 0
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//�����ȼ�Ϊ0
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ��
//   NVIC_Init(&NVIC_InitStructure);
	
	
	  /* Enable the RTC Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;					//�����ⲿ�ж�Դ�����жϣ� 
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);  
	
// 	//NRF24L01�ж�����
//   NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;					//NRF24L01 �ж���Ӧ
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		    //��ռ���ȼ� 0
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//�����ȼ�Ϊ1
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ��
//   NVIC_Init(&NVIC_InitStructure);		

// 			/*ʹ���ⲿ�ж�Ҫ�õ���AFIO��ʱ��*/
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	

//   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);	   //NRF24L01 IRQ  PA0
//   
//   EXTI_InitStructure.EXTI_Line = EXTI_Line5;					   //NRF24L01 IRQ PA0
//   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			   //EXTI�ж�
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		   //�½��ش���
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;						   //ʹ��
//   EXTI_Init(&EXTI_InitStructure);	
// 	//NRF24L01�ж����ý���
	
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
	
		if (SysTick_Config(640))		   //ʱ�ӽ����ж�ʱ10usһ��  ���ڶ�ʱ 
		{ 
			/* Capture error */ 
			while (1);
		} 
}


void GPIO_Configuration(void){
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	

	//PB10,PB11ģ����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
}


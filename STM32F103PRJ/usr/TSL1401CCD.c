#include "TSL1401CCD.h"
#include "math.h"
/* 曝光时间，单位ms */
u8 CCD_IntegrationTime = 10;

void CCD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	

	//配置A0=SI, A1=CLK, 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//配置PA2模拟输入，A2=AO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // PA2,输入时不用设置速率
	
		//ADC配置
	/* Resets ADC1 */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	ADC_DeInit(ADC1);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;               //规定了顺序进行规则转换的ADC通道的数目。这个数目的取值范围是1到16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration [规则模式通道配置]*/ 

	//ADC1 规则通道配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);	  //通道16样时间 239.5周期
	

	//使能ADC1 DMA 
	//ADC_DMACmd(ADC1, ENABLE);
	//使能ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	// 初始化ADC1校准寄存器
	ADC_ResetCalibration(ADC1);
	//检测ADC1校准寄存器初始化是否完成
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//开始校准ADC1
	ADC_StartCalibration(ADC1);
	//检测是否完成校准
	while(ADC_GetCalibrationStatus(ADC1));
	
	//ADC1转换启动
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
	
}

void CCD_StartIntegration(void) 
{

    unsigned char i;

    SI_SetVal();            /* SI  = 1 */
    CCD_SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    CCD_SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    CCD_SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */
    for(i=0; i<127; i++) {
        CCD_SamplingDelay();
        CCD_SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        CCD_SamplingDelay();
        CCD_SamplingDelay();
        CLK_ClrVal();       /* CLK = 0 */
    }
    CCD_SamplingDelay();
    CCD_SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    CCD_SamplingDelay();
    CCD_SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */
}

void CCD_ImageCapture(unsigned char * ImageData)
{
    unsigned char i;
    extern u8 AtemP ;

    SI_SetVal();            /* SI  = 1 */
    CCD_SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    CCD_SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    CCD_SamplingDelay();

    //Delay 10us for sample the first pixel
    /**/
    for(i = 0; i < 250; i++) {                    //更改250，让CCD的图像看上去比较平滑，
      CCD_SamplingDelay() ;  //200ns                  //把该值改大或者改小达到自己满意的结果。
    }

    //Sampling Pixel 1
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
		while(ADC_GetSoftwareStartConvStatus(ADC1));
		
    *ImageData =  ADC_GetConversionValue(ADC1)>>4;
    ImageData ++ ;
    CLK_ClrVal();           /* CLK = 0 */

    for(i=0; i<127; i++) {
        CCD_SamplingDelay();
        CCD_SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        CCD_SamplingDelay();
        CCD_SamplingDelay();
        //Sampling Pixel 2~128
				ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
				while(ADC_GetSoftwareStartConvStatus(ADC1));
       *ImageData =  ADC_GetConversionValue(ADC1)>>4;
        ImageData ++ ;
        CLK_ClrVal();       /* CLK = 0 */
    }
    CCD_SamplingDelay();
    CCD_SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    CCD_SamplingDelay();
    CCD_SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */	
}

void CCD_CalculateIntegrationTime(u8 *Pixel) 
{
/* 128个像素点的平均AD值 */
u8 PixelAverageValue;
/* 128个像素点的平均电压值的10倍 */
u8 PixelAverageVoltage;
/* 设定目标平均电压值，实际电压的10倍 */
s16 TargetPixelAverageVoltage = 25;
/* 设定目标平均电压值与实际值的偏差，实际电压的10倍 */
s16 PixelAverageVoltageError = 0;
/* 设定目标平均电压值允许的偏差，实际电压的10倍 */
s16 TargetPixelAverageVoltageAllowError = 2;

    /* 计算128个像素点的平均AD值 */
    PixelAverageValue = CCD_PixelAverage(128,Pixel);
    /* 计算128个像素点的平均电压值,实际值的10倍 */
    PixelAverageVoltage = (unsigned char)((int)PixelAverageValue * 25 / 194);

    PixelAverageVoltageError = TargetPixelAverageVoltage - PixelAverageVoltage;
    if(PixelAverageVoltageError < -TargetPixelAverageVoltageAllowError)
    {
      PixelAverageVoltageError = 0- PixelAverageVoltageError ;
      PixelAverageVoltageError /= 2;
      if(PixelAverageVoltageError > 10 )
         PixelAverageVoltageError = 10 ;
       CCD_IntegrationTime -= PixelAverageVoltageError;
    }
    if(PixelAverageVoltageError > TargetPixelAverageVoltageAllowError)
    { 
        PixelAverageVoltageError /= 2;
        if(PixelAverageVoltageError > 10 )
           PixelAverageVoltageError = 10 ;
        CCD_IntegrationTime += PixelAverageVoltageError;}
 
    if(CCD_IntegrationTime <= 1)
        CCD_IntegrationTime = 1;
    if(CCD_IntegrationTime >= 100)
        CCD_IntegrationTime = 100;
}

u8 CCD_PixelAverage(u8 len, u8 *data) 
{
  unsigned char i;
  unsigned int sum = 0;
  for(i = 0; i<len; i++) {
    sum = sum + *data++;
  }
  return ((unsigned char)(sum/len));
}

void CCD_SamplingDelay(void)
{
   u8 i ;
   for(i=0;i<12;i++);
   
}

bool CCD_DetectLine(u8 *Pixel, u8 sen)
{
	unsigned char i,nCount;
	int8_t nErr;
	nCount=0;
	for(i=10;i<120;i++)
	{
		nErr = Pixel[i] - Pixel[i-1] + Pixel[i+1] - Pixel[i];
		if(nErr > sen/2 || nErr < -sen/2) nCount++;
	}
	
	if(nCount > sen) return TRUE;
	else return FALSE;
}

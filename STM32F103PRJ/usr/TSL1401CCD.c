#include "TSL1401CCD.h"
#include "math.h"
/* �ع�ʱ�䣬��λms */
u8 CCD_IntegrationTime = 10;

void CCD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	

	//����A0=SI, A1=CLK, 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//����PA2ģ�����룬A2=AO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // PA2,����ʱ������������
	
		//ADC����
	/* Resets ADC1 */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	ADC_DeInit(ADC1);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		//ģ��ת��������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת��������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;               //�涨��˳����й���ת����ADCͨ������Ŀ�������Ŀ��ȡֵ��Χ��1��16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration [����ģʽͨ������]*/ 

	//ADC1 ����ͨ������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);	  //ͨ��16��ʱ�� 239.5����
	

	//ʹ��ADC1 DMA 
	//ADC_DMACmd(ADC1, ENABLE);
	//ʹ��ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	// ��ʼ��ADC1У׼�Ĵ���
	ADC_ResetCalibration(ADC1);
	//���ADC1У׼�Ĵ�����ʼ���Ƿ����
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//��ʼУ׼ADC1
	ADC_StartCalibration(ADC1);
	//����Ƿ����У׼
	while(ADC_GetCalibrationStatus(ADC1));
	
	//ADC1ת������
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
    for(i = 0; i < 250; i++) {                    //����250����CCD��ͼ����ȥ�Ƚ�ƽ����
      CCD_SamplingDelay() ;  //200ns                  //�Ѹ�ֵ�Ĵ���߸�С�ﵽ�Լ�����Ľ����
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
/* 128�����ص��ƽ��ADֵ */
u8 PixelAverageValue;
/* 128�����ص��ƽ����ѹֵ��10�� */
u8 PixelAverageVoltage;
/* �趨Ŀ��ƽ����ѹֵ��ʵ�ʵ�ѹ��10�� */
s16 TargetPixelAverageVoltage = 25;
/* �趨Ŀ��ƽ����ѹֵ��ʵ��ֵ��ƫ�ʵ�ʵ�ѹ��10�� */
s16 PixelAverageVoltageError = 0;
/* �趨Ŀ��ƽ����ѹֵ�����ƫ�ʵ�ʵ�ѹ��10�� */
s16 TargetPixelAverageVoltageAllowError = 2;

    /* ����128�����ص��ƽ��ADֵ */
    PixelAverageValue = CCD_PixelAverage(128,Pixel);
    /* ����128�����ص��ƽ����ѹֵ,ʵ��ֵ��10�� */
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

/****************************************************************************
*						   									�ŷ�ʵս����	
*1.����֧��QQ:3403129447  �ֻ�:15618987206
*2.΢�Ź��ں�:�ŷ�ʵս����
*3.�Ա�����:https://shop105175919.taobao.com/
*4.����:�ŷ�ʵս����								  
*���������ѧϰʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
*****************************************************************************/
#include "stm32f37x.h"
#include "ADC.h"
#include "Sys.h"

uint8_t UnderVolage = 0;
uint8_t UnderVolageTimes = 0;
extern float ADC_Value;
/*********************************************************
������: void Init_ADC(void)
��  ��: adc��ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void Init_ADC(void)
{
    ADC_InitTypeDef ADC_InitStructure;
	
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC->AHBENR |= (1<<17);		//I/O port A clock enabled
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* ADC1 Periph clock enable */
		RCC_ADCCLKConfig(RCC_PCLK2_Div4);
		ADC_DeInit(ADC1);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* Configure the ADC1 in continuous mode */
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		//��ɨ��,������
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;		//�Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 1;			
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Convert the ADC1 Channel 9 with 55.5 Cycles as sampling time */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
		
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);

    while(ADC_GetResetCalibrationStatus(ADC1));

    /* ADC1 calibration start */
    ADC_StartCalibration(ADC1);

    while(ADC_GetCalibrationStatus(ADC1));

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
}

/*********************************************************
������: int16_t Get_Adc(int8_t ch)   
��  ��: ��ȡadc���β������
����ֵ: ����ͨ�� ADC_Channel_0~ADC_Channel_16
���ֵ: ��
����ֵ:ת�����
**********************************************************/
int16_t Get_Adc(int8_t ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_55Cycles5 );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}
/*********************************************************
������: int16_t Get_Adc_Average(int8_t ch,int8_t times)  
��  ��: ��ȡadc��β���ֵ����ƽ��
����ֵ: ����ͨ�� ADC_Channel_0~ADC_Channel_16,��������
���ֵ: ��
����ֵ:��β���ƽ�����
**********************************************************/
float Get_Adc_Average(int8_t ch,int8_t times)
{
	int32_t temp_val=0;
	int8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		Delay_us(1000);
	}
	
	return (temp_val/times)/ 2048.0f * 3.0f;
}
/*********************************************************
������: void VoltageProt(void)
��  ��: Ƿѹ����
����ֵ: ��
���ֵ: ��
����ֵ: ��
**********************************************************/
void VoltageProt(void)
{
	if(ADC_Value < 3.0f)
	{
		UnderVolageTimes++;      //�͵�ѹ����������ʼ����
	}
	else 
	{
		UnderVolageTimes = 0;
	}	
	if(UnderVolageTimes>60)    //3s
	{
		UnderVolageTimes = 0;
		UnderVolage = 1;       //�͵�ѹ����
	}
}
	





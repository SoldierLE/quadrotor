/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
*****************************************************************************/
#include "stm32f37x.h"
#include "ADC.h"
#include "Sys.h"

uint8_t UnderVolage = 0;
uint8_t UnderVolageTimes = 0;
extern float ADC_Value;
/*********************************************************
函数名: void Init_ADC(void)
描  述: adc初始化
输入值: 无
输出值: 无
返回值: 无 
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
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		//不扫面,不连续
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;		//右对齐
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
函数名: int16_t Get_Adc(int8_t ch)   
描  述: 获取adc单次采样结果
输入值: 采样通道 ADC_Channel_0~ADC_Channel_16
输出值: 无
返回值:转换结果
**********************************************************/
int16_t Get_Adc(int8_t ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_55Cycles5 );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
/*********************************************************
函数名: int16_t Get_Adc_Average(int8_t ch,int8_t times)  
描  述: 获取adc多次采样值并求平均
输入值: 采样通道 ADC_Channel_0~ADC_Channel_16,采样次数
输出值: 无
返回值:多次采样平均结果
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
函数名: void VoltageProt(void)
描  述: 欠压处理
输入值: 无
输出值: 无
返回值: 无
**********************************************************/
void VoltageProt(void)
{
	if(ADC_Value < 3.0f)
	{
		UnderVolageTimes++;      //低电压报警触发开始计数
	}
	else 
	{
		UnderVolageTimes = 0;
	}	
	if(UnderVolageTimes>60)    //3s
	{
		UnderVolageTimes = 0;
		UnderVolage = 1;       //低电压报警
	}
}
	





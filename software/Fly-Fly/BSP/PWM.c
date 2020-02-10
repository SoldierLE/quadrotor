/****************************************************************************
*						   									�ŷ�ʵս����	
*1.����֧��QQ:3403129447  �ֻ�:15618987206
*2.΢�Ź��ں�:�ŷ�ʵս����
*3.�Ա�����:https://shop105175919.taobao.com/
*4.����:�ŷ�ʵս����								  
*���������ѧϰʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
*****************************************************************************/
#include "stm32f37x.h"
#include "pwm.h"
/*********************************************************
������: void Tim2_init(void)
��  ��: pwm��ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
��ע:AHB1 Time Clock Ϊ72M  4��Ƶ:0��ʱ�򲻷�Ƶ��1��Ƶ,
*		 1��ʱ��2��Ƶ....3��ʱ��Ϊ4��Ƶ   72/4 = 18M  1/(1/18M *1000)��ΪƵ��
**********************************************************/
void Tim2_init(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	

	TIM2->ARR=1000;//�趨�������Զ���װֵ
	TIM2->PSC=3;//4��Ƶ
	TIM2->CCMR1|=6<<4; //CH1 PWM2ģʽ
	TIM2->CCMR1|=1<<3; //CH1Ԥװ��ʹ��
//	TIM2->CCMR1|=6<<12; //CH2 PWM2ģʽ
//	TIM2->CCMR1|=1<<11; //CH2Ԥװ��ʹ��
//	TIM2->CCMR2|=6<<4; //CH3 PWM2ģʽ
//	TIM2->CCMR2|=1<<3; //CH3Ԥװ��ʹ��
//	TIM2->CCMR2|=6<<12; //CH4 PWM2ģʽ
//	TIM2->CCMR2|=1<<11; //CH4Ԥװ��ʹ��
	TIM2->CCER|=0x1111; //CH1234 ���ʹ��
	TIM2->CR1|=0x01; //ʹ�ܶ�ʱ��2			
	
	TIM2->CCR1=0; //PWMռ�ձ�
//	TIM2->CCR2=0;
//	TIM2->CCR3=0;
//	TIM2->CCR4=0;
}
/*********************************************************
������: void Tim5_init(void)
��  ��: pwm��ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
��ע:AHB1 Time Clock Ϊ72M  4��Ƶ:0��ʱ�򲻷�Ƶ��1��Ƶ,
*		 1��ʱ��2��Ƶ....3��ʱ��Ϊ4��Ƶ   72/4 = 18M  1/(1/18M *1000)��ΪƵ��
**********************************************************/
void Tim5_init(void)//PWM����
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	TIM5->ARR=1000;//�趨�������Զ���װֵ
	TIM5->PSC=3;//4��Ƶ
	TIM5->CCMR1|=6<<4; //CH1 PWM2ģʽ
	TIM5->CCMR1|=1<<3; //CH1Ԥװ��ʹ��
//	TIM5->CCMR1|=6<<12; //CH2 PWM2ģʽ
//	TIM5->CCMR1|=1<<11; //CH2Ԥװ��ʹ��
//	TIM5->CCMR2|=6<<4; //CH3 PWM2ģʽ
//	TIM5->CCMR2|=1<<3; //CH3Ԥװ��ʹ��
//	TIM5->CCMR2|=6<<12; //CH4 PWM2ģʽ
//	TIM5->CCMR2|=1<<11; //CH4Ԥװ��ʹ��
	TIM5->CCER|=0x1111; //CH1234 ���ʹ��
	TIM5->CR1|=0x01; //ʹ�ܶ�ʱ��2			
	
	TIM5->CCR1=0; //PWMռ�ձ�
}

/*********************************************************
������: void Tim12_init(void)
��  ��: pwm��ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
��ע:AHB1 Time Clock Ϊ72M  4��Ƶ:0��ʱ�򲻷�Ƶ��1��Ƶ,
*		 1��ʱ��2��Ƶ....3��ʱ��Ϊ4��Ƶ   72/4 = 18M  1/(1/18M *1000)��ΪƵ��
**********************************************************/
void Tim12_init(void)//PWM����    �⺯����
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);	
	TIM12->ARR=1000;//�趨�������Զ���װֵ
	TIM12->PSC=3;//4��Ƶ
	TIM12->CCMR1|=6<<4; //CH1 PWM2ģʽ
	TIM12->CCMR1|=1<<3; //CH1Ԥװ��ʹ��
	TIM12->CCMR1|=6<<12; //CH2 PWM2ģʽ
	TIM12->CCMR1|=1<<11; //CH2Ԥװ��ʹ��
//	TIM2->CCMR2|=6<<4; //CH3 PWM2ģʽ
//	TIM2->CCMR2|=1<<3; //CH3Ԥװ��ʹ��
//	TIM2->CCMR2|=6<<12; //CH4 PWM2ģʽ
//	TIM2->CCMR2|=1<<11; //CH4Ԥװ��ʹ��
	TIM12->CCER|=0x1111; //CH1234 ���ʹ��
	TIM12->CR1|=0x01; //ʹ�ܶ�ʱ��2			
	
	TIM12->CCR1=0; //PWMռ�ձ�
	TIM12->CCR2=0;
//	TIM2->CCR3=0;
//	TIM2->CCR4=0;
	
}
/*********************************************************
������: void pwm_init(void)
��  ��: pwm��ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void pwm_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/**********************LED*****************************/
	RCC->AHBENR |= (1<<20);		//I/O port B clock enabled
	RCC->AHBENR |= (1<<17);		//I/O port A clock enabled
		
	/**********************Moto*****************************/
	//PB14  PB15  PA8 PA15   ���ų�ʼ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14, GPIO_AF_9);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15, GPIO_AF_9);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //TIM2_PWM

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //TIM2_PWM
	Tim2_init();
	Tim5_init();
	Tim12_init();
}

/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
*****************************************************************************/
#include "stm32f37x.h"
#include "pwm.h"
/*********************************************************
函数名: void Tim2_init(void)
描  述: pwm初始化
输入值: 无
输出值: 无
返回值: 无 
备注:AHB1 Time Clock 为72M  4分频:0的时候不分频即1分频,
*		 1的时候2分频....3的时候为4分频   72/4 = 18M  1/(1/18M *1000)即为频率
**********************************************************/
void Tim2_init(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	

	TIM2->ARR=1000;//设定计数器自动重装值
	TIM2->PSC=3;//4分频
	TIM2->CCMR1|=6<<4; //CH1 PWM2模式
	TIM2->CCMR1|=1<<3; //CH1预装载使能
//	TIM2->CCMR1|=6<<12; //CH2 PWM2模式
//	TIM2->CCMR1|=1<<11; //CH2预装载使能
//	TIM2->CCMR2|=6<<4; //CH3 PWM2模式
//	TIM2->CCMR2|=1<<3; //CH3预装载使能
//	TIM2->CCMR2|=6<<12; //CH4 PWM2模式
//	TIM2->CCMR2|=1<<11; //CH4预装载使能
	TIM2->CCER|=0x1111; //CH1234 输出使能
	TIM2->CR1|=0x01; //使能定时器2			
	
	TIM2->CCR1=0; //PWM占空比
//	TIM2->CCR2=0;
//	TIM2->CCR3=0;
//	TIM2->CCR4=0;
}
/*********************************************************
函数名: void Tim5_init(void)
描  述: pwm初始化
输入值: 无
输出值: 无
返回值: 无 
备注:AHB1 Time Clock 为72M  4分频:0的时候不分频即1分频,
*		 1的时候2分频....3的时候为4分频   72/4 = 18M  1/(1/18M *1000)即为频率
**********************************************************/
void Tim5_init(void)//PWM产生
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	TIM5->ARR=1000;//设定计数器自动重装值
	TIM5->PSC=3;//4分频
	TIM5->CCMR1|=6<<4; //CH1 PWM2模式
	TIM5->CCMR1|=1<<3; //CH1预装载使能
//	TIM5->CCMR1|=6<<12; //CH2 PWM2模式
//	TIM5->CCMR1|=1<<11; //CH2预装载使能
//	TIM5->CCMR2|=6<<4; //CH3 PWM2模式
//	TIM5->CCMR2|=1<<3; //CH3预装载使能
//	TIM5->CCMR2|=6<<12; //CH4 PWM2模式
//	TIM5->CCMR2|=1<<11; //CH4预装载使能
	TIM5->CCER|=0x1111; //CH1234 输出使能
	TIM5->CR1|=0x01; //使能定时器2			
	
	TIM5->CCR1=0; //PWM占空比
}

/*********************************************************
函数名: void Tim12_init(void)
描  述: pwm初始化
输入值: 无
输出值: 无
返回值: 无 
备注:AHB1 Time Clock 为72M  4分频:0的时候不分频即1分频,
*		 1的时候2分频....3的时候为4分频   72/4 = 18M  1/(1/18M *1000)即为频率
**********************************************************/
void Tim12_init(void)//PWM产生    库函数版
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);	
	TIM12->ARR=1000;//设定计数器自动重装值
	TIM12->PSC=3;//4分频
	TIM12->CCMR1|=6<<4; //CH1 PWM2模式
	TIM12->CCMR1|=1<<3; //CH1预装载使能
	TIM12->CCMR1|=6<<12; //CH2 PWM2模式
	TIM12->CCMR1|=1<<11; //CH2预装载使能
//	TIM2->CCMR2|=6<<4; //CH3 PWM2模式
//	TIM2->CCMR2|=1<<3; //CH3预装载使能
//	TIM2->CCMR2|=6<<12; //CH4 PWM2模式
//	TIM2->CCMR2|=1<<11; //CH4预装载使能
	TIM12->CCER|=0x1111; //CH1234 输出使能
	TIM12->CR1|=0x01; //使能定时器2			
	
	TIM12->CCR1=0; //PWM占空比
	TIM12->CCR2=0;
//	TIM2->CCR3=0;
//	TIM2->CCR4=0;
	
}
/*********************************************************
函数名: void pwm_init(void)
描  述: pwm初始化
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void pwm_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/**********************LED*****************************/
	RCC->AHBENR |= (1<<20);		//I/O port B clock enabled
	RCC->AHBENR |= (1<<17);		//I/O port A clock enabled
		
	/**********************Moto*****************************/
	//PB14  PB15  PA8 PA15   引脚初始化
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

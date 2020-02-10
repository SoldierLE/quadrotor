/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
*****************************************************************************/
#include "stm32f37x.h"
#include "GPIO.h"
#include "Sys.h"
/*********************************************************
函数名: Init_GPIO(void)
描  述: IO口初始化
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void Init_GPIO(void)
{
		//GPIO_InitTypeDef GPIO_InitStructure;
		/**********************LED*****************************/
		RCC->AHBENR |= (1<<18);		//I/O port C clock enabled
		RCC->AHBENR |= (1<<19);		//I/O port D clock enabled
		RCC->AHBENR |= (1<<20);		//I/O port B clock enabled
		RCC->AHBENR |= (1<<17);		//I/O port A clock enabled
		RCC->AHBENR |= (1<<21);		//I/O port E clock enabled 
		
		GPIOB->MODER = 0; //Reset register
	  GPIOB->OSPEEDR = 0;//Reset register
		GPIOB->PUPDR = 0;//Reset register
		//GPIOB_Pin2 GPIOB_Pin4
		GPIOB->MODER |= 0x00000110; //输出模式  推挽输出
	  GPIOB->OSPEEDR |= 0x00000330;//高速
		GPIOB->ODR |= 0x0014;				//输出高 
		//GPIOE_Pin8 GPIOE_Pin9
		GPIOE->MODER |= 0x00050000; //输出模式  推挽输出
	  GPIOE->OSPEEDR |= 0x000F0000;//高速
		GPIOE->ODR |= 0x0300;				//输出高 	
		//GPIOD_Pin8
		GPIOD->MODER |= 0x00010000; //输出模式  推挽输出
	  GPIOD->OSPEEDR |= 0x00030000;//高速
		GPIOD->ODR |= 0x0100;				//输出高 	
		//GPIOC_Pin15 
		GPIOC->MODER |= 0x40000000; //输出模式  推挽输出
	  GPIOC->OSPEEDR |= 0xC0000000;//高速
		GPIOC->ODR |= 0x8000;				//输出高	
		/*
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOE, &GPIO_InitStructure);
			
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		GPIO_Init(GPIOC, &GPIO_InitStructure);*/
}
/*********************************************************
函数名: void UnderVoltageFlick(void)
描  述: 闪烁
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void UnderVoltageFlick(void)
{
	LED1_Flick;
	LED2_Flick;
	LED3_Flick;
	LED4_Flick;
}
/*********************************************************
函数名: void LostSignalFlick(void)
描  述: 闪烁
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void LostSignalFlick(void)
{
	LED1_Flick;
	LED2_Flick;
}
/*********************************************************
函数名: void SysRun_Led(void)
描  述: 系统运行灯闪烁
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void SysUnRun_Led(void)
{
	LED5_Flick;
}
/*********************************************************
函数名: void SysRun_Led(void)
描  述: 系统运行灯常亮
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void SysRun_LED(void)
{
	LED5_ON;
}

/*********************************************************
函数名: void allLedOn(void)
描  述: 打开LED
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void allLedOn(void)
{
	LED1_ON 				//输出低 	
	LED2_ON 				//输出低 		
	LED5_ON						//输出低 	
	LED6_ON					//输出低 
}
/*********************************************************
函数名: void allLedOff(void)
描  述: 关闭LED
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void allLedOff(void)
{
	LED1_OFF					//输出高 	
	LED2_OFF				//输出高 
	LED5_OFF				//输出高 	
	LED6_OFF					//输出高 
}

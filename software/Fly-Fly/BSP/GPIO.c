/****************************************************************************
*						   									�ŷ�ʵս����	
*1.����֧��QQ:3403129447  �ֻ�:15618987206
*2.΢�Ź��ں�:�ŷ�ʵս����
*3.�Ա�����:https://shop105175919.taobao.com/
*4.����:�ŷ�ʵս����								  
*���������ѧϰʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
*****************************************************************************/
#include "stm32f37x.h"
#include "GPIO.h"
#include "Sys.h"
/*********************************************************
������: Init_GPIO(void)
��  ��: IO�ڳ�ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
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
		GPIOB->MODER |= 0x00000110; //���ģʽ  �������
	  GPIOB->OSPEEDR |= 0x00000330;//����
		GPIOB->ODR |= 0x0014;				//����� 
		//GPIOE_Pin8 GPIOE_Pin9
		GPIOE->MODER |= 0x00050000; //���ģʽ  �������
	  GPIOE->OSPEEDR |= 0x000F0000;//����
		GPIOE->ODR |= 0x0300;				//����� 	
		//GPIOD_Pin8
		GPIOD->MODER |= 0x00010000; //���ģʽ  �������
	  GPIOD->OSPEEDR |= 0x00030000;//����
		GPIOD->ODR |= 0x0100;				//����� 	
		//GPIOC_Pin15 
		GPIOC->MODER |= 0x40000000; //���ģʽ  �������
	  GPIOC->OSPEEDR |= 0xC0000000;//����
		GPIOC->ODR |= 0x8000;				//�����	
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
������: void UnderVoltageFlick(void)
��  ��: ��˸
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void UnderVoltageFlick(void)
{
	LED1_Flick;
	LED2_Flick;
	LED3_Flick;
	LED4_Flick;
}
/*********************************************************
������: void LostSignalFlick(void)
��  ��: ��˸
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void LostSignalFlick(void)
{
	LED1_Flick;
	LED2_Flick;
}
/*********************************************************
������: void SysRun_Led(void)
��  ��: ϵͳ���е���˸
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void SysUnRun_Led(void)
{
	LED5_Flick;
}
/*********************************************************
������: void SysRun_Led(void)
��  ��: ϵͳ���еƳ���
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void SysRun_LED(void)
{
	LED5_ON;
}

/*********************************************************
������: void allLedOn(void)
��  ��: ��LED
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void allLedOn(void)
{
	LED1_ON 				//����� 	
	LED2_ON 				//����� 		
	LED5_ON						//����� 	
	LED6_ON					//����� 
}
/*********************************************************
������: void allLedOff(void)
��  ��: �ر�LED
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void allLedOff(void)
{
	LED1_OFF					//����� 	
	LED2_OFF				//����� 
	LED5_OFF				//����� 	
	LED6_OFF					//����� 
}

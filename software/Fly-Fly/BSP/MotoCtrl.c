/****************************************************************************
*						   									�ŷ�ʵս����	
*1.����֧��QQ:3403129447  �ֻ�:15618987206
*2.΢�Ź��ں�:�ŷ�ʵս����
*3.�Ա�����:https://shop105175919.taobao.com/
*4.����:�ŷ�ʵս����								  
*���������ѧϰʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
*****************************************************************************/
#include "stm32f37x.h"
#include "motoctrl.h"
#include "Sys.h"
#include "mpu6500.h"
float PitchOutput,RollOutput,YawOutput; 
uint16_t PWM1,PWM2,PWM3,PWM4;
extern uint16_t AccelerateValue;
extern float ADC_Value;
extern uint8_t FlyStatus;
float idle_acc_mod=0.0f;  //����ʱ���ٶ�ģ 


/*********************************************
*									��ͷ
*					PWM3						PWM2					
*
*							
*								
*
*					PWM1						PWM4
*Yaw:ˮƽ����ת
*ROLL:����(ǰ�󷭹�)
*Pitch:���ҷ���
*********************************************/
/*********************************************************
������: void  motor_ctrl(void)
��  ��: PWM���㼰�������
����ֵ: ��
���ֵ: ��
����ֵ: ��
**********************************************************/
void  motor_ctrl(void)
{
	AccelerateValue = AccelerateValue * (4.2f/ADC_Value);//4.2Ϊ��ص�ѹ��׼,�ֲ��ڵ�ѹ������ĵ��ת�ٽ���
	
		PWM1 = AccelerateValue + PitchOutput + RollOutput + YawOutput;//
		PWM2 = AccelerateValue - PitchOutput - RollOutput + YawOutput;//
		PWM3 = AccelerateValue - PitchOutput + RollOutput - YawOutput;//
		PWM4 = AccelerateValue + PitchOutput - RollOutput - YawOutput;//

		if(FlyStatus==FlyIdle)
    {
      PWM_Set(40,40,40,40);
      idle_acc_mod=get_acc_mod();
    }
    else if(FlyStatus==FlyStart) 
    {
        PWM_Set(PWM3,PWM1,PWM4,PWM2);       //ע������˳��
    }
    else 
    { 
      PWM_Set(0,0,0,0);
    }
}
/*********************************************************
������: void PWM_Set(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
��  ��: PWM����
����ֵ: ��·PWMֵ
���ֵ: ��
����ֵ: ��
**********************************************************/
void PWM_Set(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{	
	
		if(MOTO1_PWM>PWMMAX)  MOTO1_PWM = PWMMAX; //PWM�޷� PWMMIN��PWMMAX
		if(MOTO2_PWM>PWMMAX)	MOTO2_PWM = PWMMAX;
		if(MOTO3_PWM>PWMMAX)	MOTO3_PWM = PWMMAX;
		if(MOTO4_PWM>PWMMAX)	MOTO4_PWM = PWMMAX;
		
		if(MOTO1_PWM<PWMMIN)	MOTO1_PWM = PWMMIN;
		if(MOTO2_PWM<PWMMIN)	MOTO2_PWM = PWMMIN;
		if(MOTO3_PWM<PWMMIN)	MOTO3_PWM = PWMMIN;
		if(MOTO4_PWM<PWMMIN)	MOTO4_PWM = PWMMIN;

		TIM12->CCR1 = MOTO1_PWM;           //��ֵ���ĸ����ͨ��
		TIM12->CCR2 = MOTO2_PWM;
		TIM5->CCR1 = MOTO3_PWM;
		TIM2->CCR1 = MOTO4_PWM;	
}

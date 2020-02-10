/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
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
float idle_acc_mod=0.0f;  //怠速时加速度模 


/*********************************************
*									机头
*					PWM3						PWM2					
*
*							
*								
*
*					PWM1						PWM4
*Yaw:水平面旋转
*ROLL:俯仰(前后翻滚)
*Pitch:左右翻滚
*********************************************/
/*********************************************************
函数名: void  motor_ctrl(void)
描  述: PWM计算及电机控制
输入值: 无
输出值: 无
返回值: 无
**********************************************************/
void  motor_ctrl(void)
{
	AccelerateValue = AccelerateValue * (4.2f/ADC_Value);//4.2为电池电压基准,弥补在电压低引起的电机转速降低
	
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
        PWM_Set(PWM3,PWM1,PWM4,PWM2);       //注意电机号顺序
    }
    else 
    { 
      PWM_Set(0,0,0,0);
    }
}
/*********************************************************
函数名: void PWM_Set(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
描  述: PWM设置
输入值: 四路PWM值
输出值: 无
返回值: 无
**********************************************************/
void PWM_Set(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{	
	
		if(MOTO1_PWM>PWMMAX)  MOTO1_PWM = PWMMAX; //PWM限幅 PWMMIN～PWMMAX
		if(MOTO2_PWM>PWMMAX)	MOTO2_PWM = PWMMAX;
		if(MOTO3_PWM>PWMMAX)	MOTO3_PWM = PWMMAX;
		if(MOTO4_PWM>PWMMAX)	MOTO4_PWM = PWMMAX;
		
		if(MOTO1_PWM<PWMMIN)	MOTO1_PWM = PWMMIN;
		if(MOTO2_PWM<PWMMIN)	MOTO2_PWM = PWMMIN;
		if(MOTO3_PWM<PWMMIN)	MOTO3_PWM = PWMMIN;
		if(MOTO4_PWM<PWMMIN)	MOTO4_PWM = PWMMIN;

		TIM12->CCR1 = MOTO1_PWM;           //赋值到四个电机通道
		TIM12->CCR2 = MOTO2_PWM;
		TIM5->CCR1 = MOTO3_PWM;
		TIM2->CCR1 = MOTO4_PWM;	
}

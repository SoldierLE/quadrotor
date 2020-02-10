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
#include "pwm.h"
#include "ADC.h"
#include "UART.h"
#include "I2C.h"
#include "PI.h"
#include "mpu6500.h"
uint8_t InitSuccess =0;
float ADC_Value =0;
extern uint8_t LostSignal;
extern uint8_t InitOffset;
extern uint8_t FlyStatus; 
extern float_XYZ EXP_ANGLE;
extern float_XYZ acce_f;
extern uint8_t FlyStatus;
extern uint16_t AccelerateValue;

extern uint8_t UnderVolage;
extern uint8_t RF_Mode;
/*********************************************************
������: int main(void)
��  ��: ������
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
int main()
{
	uint16_t tempcount = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);/*!< 4 bits for pre-emption priority 0 bits for subpriority */
	SysTick_Init();			//ϵͳ�δ�ʱ����ʼ��
	Init_GPIO();				//��Ƭ��IO��ģ���ʼ��
	pwm_init();					//PWMģ���ʼ��
	Init_ADC();					//ADCģ���ʼ��
	Init_Uart();				//���ڳ�ʼ��
	imu_i2c_init();			//I2C��ʼ��
	mpu6500_init();			//MPU6500��ʼ��
	PID_Init();					//PID������ʼ��
	//��ȡIMUƫ����,��ȡ mpu6500,�ȴ� �����������ȶ�
	//������������ݲ���̫���������
	while (!InitOffset)      
	{ 		
    READ_MPU6500(); 
		Delay_ms(5);
		tempcount++;
		if(tempcount%50 == 0)		//����LED����˸ʱ��
    {
			SysUnRun_Led(); 
		}
		if(tempcount>=1000)	
			tempcount = 0;
	}
	tempcount = 0;
	Get_GRYOffset();		//��������������ƫ����
	allLedOff();				//�ر�����LED��
	//������źŲ��������
	while(LostSignal)				
	{
		Delay_ms(500);
		LostSignalFlick();
	}
	allLedOff();				//�ر�����LED��
	calc_curve();				//������������
	RF_Mode = RFMODE_6AXIE;	//����ģʽ
	InitSuccess = 1;			//��ʼ���ɹ���־
	while(1)
	{
		ADC_Value = Get_Adc_Average(0,5);			//�ɼ���ص�ѹ
		if((LostSignal)||(UnderVolage))				//�ж��Ƿ��źŻ���Ƿѹ
		{
			SysUnRun_Led();											//ϵͳ����˸
			if(LostSignal)											//���źŴ���
			{
				LostSignalFlick();
			}else if(UnderVolage)								//Ƿѹ����
			{
				UnderVoltageFlick();
			}
		}else																//��Ƿѹ�����źŴ���
		{
			allLedOn();
			SysRun_LED();
		}
		Delay_ms(300);
	}
}

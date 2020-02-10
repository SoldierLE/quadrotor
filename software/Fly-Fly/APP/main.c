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
函数名: int main(void)
描  述: 主函数
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
int main()
{
	uint16_t tempcount = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);/*!< 4 bits for pre-emption priority 0 bits for subpriority */
	SysTick_Init();			//系统滴答定时器初始化
	Init_GPIO();				//单片机IO口模块初始化
	pwm_init();					//PWM模块初始化
	Init_ADC();					//ADC模块初始化
	Init_Uart();				//串口初始化
	imu_i2c_init();			//I2C初始化
	mpu6500_init();			//MPU6500初始化
	PID_Init();					//PID参数初始化
	//获取IMU偏移量,读取 mpu6500,等待 陀螺仪数据稳定
	//如果陀螺仪数据波动太大不允许起飞
	while (!InitOffset)      
	{ 		
    READ_MPU6500(); 
		Delay_ms(5);
		tempcount++;
		if(tempcount%50 == 0)		//设置LED灯闪烁时间
    {
			SysUnRun_Led(); 
		}
		if(tempcount>=1000)	
			tempcount = 0;
	}
	tempcount = 0;
	Get_GRYOffset();		//计算陀螺仪数据偏移量
	allLedOff();				//关闭所有LED灯
	//如果丢信号不允许起飞
	while(LostSignal)				
	{
		Delay_ms(500);
		LostSignalFlick();
	}
	allLedOff();				//关闭所有LED灯
	calc_curve();				//计算油门曲线
	RF_Mode = RFMODE_6AXIE;	//六轴模式
	InitSuccess = 1;			//初始化成功标志
	while(1)
	{
		ADC_Value = Get_Adc_Average(0,5);			//采集电池电压
		if((LostSignal)||(UnderVolage))				//判断是否丢信号或者欠压
		{
			SysUnRun_Led();											//系统灯闪烁
			if(LostSignal)											//丢信号处理
			{
				LostSignalFlick();
			}else if(UnderVolage)								//欠压处理
			{
				UnderVoltageFlick();
			}
		}else																//不欠压不丢信号处理
		{
			allLedOn();
			SysRun_LED();
		}
		Delay_ms(300);
	}
}

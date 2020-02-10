/**
  ******************************************************************************
  * @file    Project/STM32F37x_StdPeriph_Templates/stm32f37x_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x_it.h"
#include "GPIO.h"
#include "Sys.h"
#include "pwm.h"
#include "ADC.h"
#include "UART.h"
#include "I2C.h"
#include "math.h"
#include "mpu6500.h"
#include "motoctrl.h"

float rollRateDesired;
float pitchRateDesired;
float yawRateDesired;
extern float PitchOutput;
extern float	RollOutput;
extern float YawOutput; 
extern uint8_t LostSignal;
extern uint16_t AccelerateValue;
extern uint32_t LostSignalTime;
extern uint8_t InitSuccess;
uint8_t task_count = 0;
float_RPY Q_ANGLE;

extern PID Pitch;
extern PID Roll;
extern PID PitchRate;
extern PID RollRate;
extern PID YawRate;
extern float_RPY Q_Rad;
extern float_XYZ acce_f;
extern float_XYZ gyro_f;        //IMU ����
extern float_XYZ EXP_ANGLE;
extern uint8_t UnderVolage;
extern float YawRateGol;
extern uint8_t FlyStatus;
extern uint8_t FlipFlag;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
	
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//SysTick����1ms�ж�һ��
void SysTick_Handler(void)
{
	if(InitSuccess)				//ϵͳ��ʼ���ɹ�
	{
		task_count ++;			//����ʱ��ȥ����ϵͳ����
		if(!UnderVolage)		//��Ƿѹ
		{
			READ_MPU6500();    //��ȡIMUԭʼ����
			Cal_TsData();      //IMUԭʼ���ݼ���ת��
			if(task_count%5==0)		//5ms����һ��ң�������� ��������Ԫ��ŷ����
			{
				RfDataDecode();
				attitude_quat_5ms_task();
				PID_EurLoop(&Pitch,EXP_ANGLE.Y * 1.1f,Q_ANGLE.Pitch,  &pitchRateDesired);		//�⻷�ǶȻ�����
				PID_EurLoop(&Roll ,EXP_ANGLE.X * 1.1f,Q_ANGLE.Roll,  &rollRateDesired); 		//��߸��ұ���
			}	 
			if(task_count%3==0)			//����ͷ���� ˼�룺���ӵ�����ͷָ���ɻ���һ���Ľ��ٶȺ�����ȥ��תһ����ʱ�䡣
			{
				flip_3ms_task();
			}
			attitude_pid_1ms_task();			//�ڻ�PID����   �⻷PID���ƽ�����뵽�ڻ������ڻ�PID����
			motor_ctrl();									//�����ڻ�PID���ƽ��ȥ���Ƶ��
		}
		
		//ϵͳ��������ִ�� 50msִ��һ��
    if(task_count%50==0)
    {
      task_count=0;
			LostSignalTime++;				
			if(LostSignalTime>40)//2Sû���ݱ�ʾ���ź�
			{
				LostSignalTime = 0;
				LostSignal = 1;					//���źű�־
			}
			VoltageProt();			//Ƿѹ����
			if((LostSignal)||(UnderVolage))
			{
				EXP_ANGLE.X=0.0f;
				EXP_ANGLE.Y=0.0f;
				EXP_ANGLE.Z=0.0f;
				YawRateGol=0.0f;
				if(AccelerateValue>250.0f)		
				 AccelerateValue=250.0f;       //�����Ž���
				if(acce_f.Z>15)            //�����׺� ����ֹͣ
				{     
					 FlyStatus = FlyStop;
				}
			}
			if(AccelerateValue<30.0f)			//�ɻ�����С��һ��ֵ֮��λ��ת��ͷ��־
				FlipFlag = 0; 
		}
	}
}

/******************************************************************************/
/*                 STM32F37x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f37x.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

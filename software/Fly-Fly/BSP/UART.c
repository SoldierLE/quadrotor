/****************************************************************************
*						   									�ŷ�ʵս����	
*1.����֧��QQ:3403129447  �ֻ�:15618987206
*2.΢�Ź��ں�:�ŷ�ʵս����
*3.�Ա�����:https://shop105175919.taobao.com/
*4.����:�ŷ�ʵս����								  
*���������ѧϰʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
*****************************************************************************/
#include "UART.h"
#include "stm32f37x.h"
#include "Sys.h"
#include "math.h"


uint8_t UnderVoltage = 0;		//Ƿѹ
uint8_t StartReceiveData = 0;//��ʼ��������
uint8_t ReceiveDataNum = 0;//���������ֽ���
uint8_t ReceiveDataOver = 0;//һ֡���ݽ������
uint8_t RFdata[13] = {0};		//��Ž��յ�����
uint8_t KeyStatus = 0;			//�ɻ�����״̬
uint8_t FlyStatus = 0;			//�ɻ�����״̬
uint8_t LostSignal = 1;			//��ʧ�źű�־
uint16_t AccelerateValue = 0;		//����
uint32_t LostSignalTime = 0;			//��ʧ�źż�ʱ
uint8_t RF_Mode = 0;					//��Ƶģʽ
uint8_t flip_direction =0;  // ����ͷ����
uint16_t flip_count = 0;	//����ͷ��ʱ
uint8_t FlipFlag = 0;			//����ͷ��־
uint8_t LastFlipFlag = 0;	//��¼����ͷ��λ 
float YawRateGol;						
extern float ADC_Value;		//��ص�ѹ		
float_XYZ TempEXP_ANGLE;			
float_XYZ EXP_ANGLE;

/*********************************************************
������: void Init_Uart(void) 
��  ��: ��ʼ������1
����ֵ: ��
���ֵ: ��
����ֵ: ��
**********************************************************/
void Init_Uart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);  //??USART1,GPIOA??
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_7);        
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_7);

  // USART GPIO Configuration     
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;  //Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;   // Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);   

  // USART Parameter Configuration 
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART1, &USART_InitStructure);

	// USART NVIC Configuration 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	   
	
	// Enable USART RXNE Interrupt 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   
	// Enable USART
  USART_Cmd(USART1, ENABLE); 
}
/*********************************************************
������: void uart1_send_byte(unsigned char data)
��  ��: ����1����1���ֽ�����
����ֵ: ����
���ֵ: ��
����ֵ: ��
**********************************************************/
void uart1_send_byte(unsigned char data)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1,data);
}
/*********************************************************
������: void Init_Uart(void) 
��  ��: ���Ͷ���ֽ�����
����ֵ: �洢���ݵĵ�ַ,�ֽ���
���ֵ: ��
����ֵ: ��
**********************************************************/
void uart1_send_nbyte(unsigned char* pbuffer,unsigned int number)
{
	unsigned int count =0;
	for(count =0;count<number;count++)
	{
		uart1_send_byte(pbuffer[count]);
	}
}
/*********************************************************
*        M4			M1
*						       
*       		
*        M3		 	M2
*M1,M2,M3,M4�ֱ�����ĸ����
*M2,M3ת������,M1,M4ת�ٲ���ɻ���ǰ��,��֮����
*M3,M4ת������,M1,M2ת�ٲ���ɻ����ҷ�,��֮�����
*M1,M3ת������,M4,M2ת�ٲ���ɻ�˳ʱ����ת,��֮��ʱ����ת
*********************************************************/
void RfDataDecode(void)
{
	float TargetLen;
	if(ReceiveDataOver)
	{
			//����ֵ  171<->853 - 171   *(4.2f/ADC_Value) Ϊ�˽��е�ѹ����				ע�� - �� | �����ȼ�
		AccelerateValue = ((RFdata[1]<<8) | RFdata[2]) - 171;
		AccelerateValue = curve_ctrl(AccelerateValue);      //�����������
		//Yawֵ	512�ǵ�λ������ֵ		/8.0f���ƽǶ����ֵ		
		TempEXP_ANGLE.X = -((((RFdata[3]<<8) + RFdata[4]) - 512) / 8.0f);
		//Pitchֵ /8.0f��Ϊ�����ƽǶ����ֵ
		TempEXP_ANGLE.Y = (((RFdata[5]<<8) + RFdata[6]) - 512) / 8.0f;	
		//	512�ǵ�λ������ֵ	
		TempEXP_ANGLE.Z = ((RFdata[7]<<8) + RFdata[8]) - 512;
		EXP_ANGLE.Z = applyDeadband(constrain_float((TempEXP_ANGLE.Z*fabs(TempEXP_ANGLE.Z)/350.0f),-370.0f,370.0f),35.0f);
		//���ݷ�Χ171-853
		TargetLen= TempEXP_ANGLE.X*TempEXP_ANGLE.X + TempEXP_ANGLE.Y*TempEXP_ANGLE.Y;
		EXP_ANGLE.X=TempEXP_ANGLE.X*fabs(TempEXP_ANGLE.X)/sqrtf(TargetLen+1.0f);   //+1.0f ��ֹ��ĸΪ0
		EXP_ANGLE.Y=TempEXP_ANGLE.Y*fabs(TempEXP_ANGLE.Y)/sqrtf(TargetLen+1.0f);
		YawRateGol=EXP_ANGLE.Z;
		EXP_ANGLE.X = applyDeadband(EXP_ANGLE.X, 5.0f);
		EXP_ANGLE.Y = applyDeadband(EXP_ANGLE.Y, 5.0f);
		if(!FlipFlag)
		{
			FlipFlag = 1;
			LastFlipFlag = RFdata[9];
		}
		if(RFdata[9] != LastFlipFlag)      //����ȥ  
		{ 	 
			if( RF_Mode == RFMODE_FLIP)
				 RF_Mode = RFMODE_6AXIE;
			 else
				 RF_Mode = RFMODE_FLIP; 
			 LastFlipFlag = RFdata[9];
		}
		 
		if(RF_Mode == RFMODE_FLIP)
		{
			if(fabs(EXP_ANGLE.Y)>15.0f)  //ǰ���з���ͷָ��
      {
         if(EXP_ANGLE.Y>15.0f)
           flip_direction =1;  // forward
         else if(EXP_ANGLE.Y<-15.0f)
           flip_direction =2;  // back
         flip_count = 0;
      }else if(fabs(EXP_ANGLE.X)>15.0f)  //�����з���ͷָ��
      {
          if(EXP_ANGLE.X>15.0f)
           flip_direction =3;  // left
					else if(EXP_ANGLE.X<-15.0f)
           flip_direction =4;  // right	
         flip_count = 0;
      }
      EXP_ANGLE.X=0.0f;
      EXP_ANGLE.Y=0.0f;
		}
		//�ɻ�����״̬
		KeyStatus = RFdata[10];
		if(KeyStatus)
		{
			if(AccelerateValue<30.0f)
			{
				FlyStatus = FlyIdle;
			}
		}else
		{
			 FlyStatus  = FlyStop;
		}
		if((FlyStatus)&&(!UnderVoltage)&&(AccelerateValue>30.0f) && (FlyStatus  == 1))
			FlyStatus = FlyStart;
			
		ReceiveDataOver = 0;
	}
}
/*********************************************************
������: void USART1_IRQHandler(void)
��  ��: ���ڽ����жϷ�����
����ֵ: ��
���ֵ: ��
����ֵ: ��
**********************************************************/
void USART1_IRQHandler(void)
{
	/*receive nrf51822 data*/
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	
		RFdata[ReceiveDataNum] = USART_ReceiveData(USART1);
		LostSignal = 0;
		LostSignalTime = 0;
		if(StartReceiveData)
		{
			ReceiveDataNum++;
			if(ReceiveDataNum == 13)
			{
				ReceiveDataOver = 1;
				StartReceiveData = 0;				
				ReceiveDataNum = 0;
			}
		}else
		{
			if(!ReceiveDataOver)
			{
				if(RFdata[0] == 0x55)          //bluetooth
				{	
					StartReceiveData = 1;
					ReceiveDataNum++;
				}else if(RFdata[0] == 0x66)          //2.4G
				{		
					StartReceiveData = 1;	
					ReceiveDataNum++;					
				}else
				{
					ReceiveDataNum = 0;
					ReceiveDataOver = 0;	
					StartReceiveData = 0;
				}
			}
		}
	}
}



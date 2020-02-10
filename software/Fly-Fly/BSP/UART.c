/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
*****************************************************************************/
#include "UART.h"
#include "stm32f37x.h"
#include "Sys.h"
#include "math.h"


uint8_t UnderVoltage = 0;		//欠压
uint8_t StartReceiveData = 0;//开始接受数据
uint8_t ReceiveDataNum = 0;//接收数据字节数
uint8_t ReceiveDataOver = 0;//一帧数据接收完毕
uint8_t RFdata[13] = {0};		//存放接收的数据
uint8_t KeyStatus = 0;			//飞机开关状态
uint8_t FlyStatus = 0;			//飞机飞行状态
uint8_t LostSignal = 1;			//丢失信号标志
uint16_t AccelerateValue = 0;		//油门
uint32_t LostSignalTime = 0;			//丢失信号计时
uint8_t RF_Mode = 0;					//射频模式
uint8_t flip_direction =0;  // 翻跟头方向
uint16_t flip_count = 0;	//翻跟头计时
uint8_t FlipFlag = 0;			//翻跟头标志
uint8_t LastFlipFlag = 0;	//记录翻跟头键位 
float YawRateGol;						
extern float ADC_Value;		//电池电压		
float_XYZ TempEXP_ANGLE;			
float_XYZ EXP_ANGLE;

/*********************************************************
函数名: void Init_Uart(void) 
描  述: 初始化串口1
输入值: 无
输出值: 无
返回值: 无
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
函数名: void uart1_send_byte(unsigned char data)
描  述: 串口1发送1个字节数据
输入值: 数据
输出值: 无
返回值: 无
**********************************************************/
void uart1_send_byte(unsigned char data)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1,data);
}
/*********************************************************
函数名: void Init_Uart(void) 
描  述: 发送多个字节数据
输入值: 存储数据的地址,字节数
输出值: 无
返回值: 无
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
*M1,M2,M3,M4分别代表四个电机
*M2,M3转速增大,M1,M4转速不变飞机向前飞,反之向后飞
*M3,M4转速增大,M1,M2转速不变飞机向右飞,反之向左飞
*M1,M3转速增大,M4,M2转速不变飞机顺时针旋转,反之逆时针旋转
*********************************************************/
void RfDataDecode(void)
{
	float TargetLen;
	if(ReceiveDataOver)
	{
			//油门值  171<->853 - 171   *(4.2f/ADC_Value) 为了进行电压补偿				注意 - 和 | 的优先级
		AccelerateValue = ((RFdata[1]<<8) | RFdata[2]) - 171;
		AccelerateValue = curve_ctrl(AccelerateValue);      //油门曲线拟合
		//Yaw值	512是电位器的中值		/8.0f限制角度最大值		
		TempEXP_ANGLE.X = -((((RFdata[3]<<8) + RFdata[4]) - 512) / 8.0f);
		//Pitch值 /8.0f是为了限制角度最大值
		TempEXP_ANGLE.Y = (((RFdata[5]<<8) + RFdata[6]) - 512) / 8.0f;	
		//	512是电位器的中值	
		TempEXP_ANGLE.Z = ((RFdata[7]<<8) + RFdata[8]) - 512;
		EXP_ANGLE.Z = applyDeadband(constrain_float((TempEXP_ANGLE.Z*fabs(TempEXP_ANGLE.Z)/350.0f),-370.0f,370.0f),35.0f);
		//数据范围171-853
		TargetLen= TempEXP_ANGLE.X*TempEXP_ANGLE.X + TempEXP_ANGLE.Y*TempEXP_ANGLE.Y;
		EXP_ANGLE.X=TempEXP_ANGLE.X*fabs(TempEXP_ANGLE.X)/sqrtf(TargetLen+1.0f);   //+1.0f 防止分母为0
		EXP_ANGLE.Y=TempEXP_ANGLE.Y*fabs(TempEXP_ANGLE.Y)/sqrtf(TargetLen+1.0f);
		YawRateGol=EXP_ANGLE.Z;
		EXP_ANGLE.X = applyDeadband(EXP_ANGLE.X, 5.0f);
		EXP_ANGLE.Y = applyDeadband(EXP_ANGLE.Y, 5.0f);
		if(!FlipFlag)
		{
			FlipFlag = 1;
			LastFlipFlag = RFdata[9];
		}
		if(RFdata[9] != LastFlipFlag)      //按下去  
		{ 	 
			if( RF_Mode == RFMODE_FLIP)
				 RF_Mode = RFMODE_6AXIE;
			 else
				 RF_Mode = RFMODE_FLIP; 
			 LastFlipFlag = RFdata[9];
		}
		 
		if(RF_Mode == RFMODE_FLIP)
		{
			if(fabs(EXP_ANGLE.Y)>15.0f)  //前后有翻跟头指令
      {
         if(EXP_ANGLE.Y>15.0f)
           flip_direction =1;  // forward
         else if(EXP_ANGLE.Y<-15.0f)
           flip_direction =2;  // back
         flip_count = 0;
      }else if(fabs(EXP_ANGLE.X)>15.0f)  //左右有翻跟头指令
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
		//飞机开关状态
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
函数名: void USART1_IRQHandler(void)
描  述: 串口接收中断服务函数
输入值: 无
输出值: 无
返回值: 无
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



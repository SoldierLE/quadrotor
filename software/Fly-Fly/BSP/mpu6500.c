/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
*****************************************************************************/
#include "stm32f37x.h"
#include "mpu6500.h"
#include "I2C.h"
#include "Sys.h"
#include "math.h"


u8 InitOffset = 0;
u16 turbulen = 0;
u16 Peace, PeaceTime; 
int16_XYZ ACC_Offset,GRY_Offset;     // 陀螺仪偏移量
float_XYZ acce_f,gyro_f;        //IMU 数据
int16_XYZ ACC_RealData,GRY_RealData;
 
bool Gyro_Offset_Flag=false;   // IMU偏移量获取标志
bool GRY_Stable_Flag=false;     // 陀螺仪数据稳定标志
/*********************************************************
函数名: void Init_Uart(void) 
描  述: 初始化串口1
输入值: 无
输出值: 无
返回值: 无
**********************************************************/
void mpu6500_init(void)
{
	i2c_reg_write(IMU_I2C,0x80,MPU6500_Addr,PWR_MGMT_1);	//复位MPU6500	
	Delay_ms(100);
	i2c_reg_write(IMU_I2C,0x07,MPU6500_Addr,MPU6500_SIGNAL_PATH_RESET);//复位信号路径
	
	i2c_reg_write(IMU_I2C,0x03,MPU6500_Addr,PWR_MGMT_1);//自动选择最佳时钟源    
	i2c_reg_write(IMU_I2C,0x00,MPU6500_Addr,SMPLRT_DIV);  //采样率分频    
	i2c_reg_write(IMU_I2C,0x01,MPU6500_Addr,CONFIG);  			//DLPF_CFG = 1;	  数字低通滤波器配置	
	i2c_reg_write(IMU_I2C,0x05,MPU6500_Addr,MPU6500_ACCEL_CONFIG2); //A_DLPF_CFG = 5; 数字低通滤波器配置
	i2c_reg_write(IMU_I2C,0x20,MPU6500_Addr,MPU6500_INT_PIN_CFG);//Interrupt status is cleared if any read operation is performed.
	i2c_reg_write(IMU_I2C,0x01,MPU6500_Addr,MPU6500_INT_ENABLE);//Enable interrupt for wake on motion to propagate to interrupt pin.
	  
	i2c_reg_write(IMU_I2C,3<<3,MPU6500_Addr,GYRO_CONFIG);  //+/-2000dps   角速度     度/S
	i2c_reg_write(IMU_I2C,1<<3,MPU6500_Addr,ACCEL_CONFIG); //+/-4g				加速度	
}
/*********************************************************
函数名: void READ_MPU6500(void) 
描  述: 读数据
输入值: 无
输出值: 无
返回值: 无
**********************************************************/
void READ_MPU6500(void)
{
	u8 BUF_6500_buff[14];
	int16_XYZ Old_GRY;

	Old_GRY.X=GRY_RealData.X;
	Old_GRY.Y=GRY_RealData.Y;
	Old_GRY.Z=GRY_RealData.Z;

	i2c_read(IMU_I2C,BUF_6500_buff,MPU6500_Addr,ACCEL_XOUT_H,14);			//0x3B-0x48 详见数据手册
	ACC_RealData.X=(( ((int16_t)BUF_6500_buff[0]) <<8) |BUF_6500_buff[1]);
	ACC_RealData.Y=(( ((int16_t)BUF_6500_buff[2]) <<8) |BUF_6500_buff[3]);	
	ACC_RealData.Z=(( ((int16_t)BUF_6500_buff[4]) <<8) |BUF_6500_buff[5]);			//加速度数据

	GRY_RealData.X=(( ((int16_t)BUF_6500_buff[8]) <<8) |BUF_6500_buff[9]);
	GRY_RealData.Y=(( ((int16_t)BUF_6500_buff[10]) <<8) |BUF_6500_buff[11]);	
	GRY_RealData.Z=(( ((int16_t)BUF_6500_buff[12]) <<8) |BUF_6500_buff[13]);		//角度数据
	//校正偏移量
	if(Gyro_Offset_Flag==true)
	{ 	
		GRY_RealData.X -= GRY_Offset.X;
		GRY_RealData.Y -= GRY_Offset.Y;
		GRY_RealData.Z -= GRY_Offset.Z;
  }
	turbulen=fabs(Old_GRY.X-GRY_RealData.X)+fabs(Old_GRY.Y-GRY_RealData.Y)+fabs(Old_GRY.Z-GRY_RealData.Z);	
	
	if(turbulen < 20) 
		Peace++;  
	else 
	{
		Peace = 0; 
		GRY_Stable_Flag = false;
	} 
	if(Peace > 100) 
		GRY_Stable_Flag = true;

	if(InitOffset == 0)
	{	   
		if(Peace > 100) 
			InitOffset = 1;
	}
}
/*********************************************************
函数名: void Get_GRYOffset(void)
描  述: 在静止的状态下计算数据取30次平均值计算飞机的偏移量,就是在静止状态下数据的变化
输入值: 无
输出值: 无
返回值: 无
**********************************************************/
void Get_GRYOffset(void)
{ 
	u8 i;
	int_fast32_t  GRY_XTemp=0,GRY_YTemp=0,GRY_ZTemp=0;
	Gyro_Offset_Flag=false;
	
  for(i=0;i<30;i++)	
	{
		READ_MPU6500();
		GRY_XTemp+=GRY_RealData.X;
		GRY_YTemp+=GRY_RealData.Y;
		GRY_ZTemp+=GRY_RealData.Z;
		Delay_ms(5);	
 	}		
	GRY_XTemp/=30;
	GRY_YTemp/=30;
	GRY_ZTemp/=30;

 	GRY_Offset.X=GRY_XTemp;
 	GRY_Offset.Y=GRY_YTemp;
 	GRY_Offset.Z=GRY_ZTemp;	
	Gyro_Offset_Flag=true;
}
/*********************************************************
函数名: void Cal_TsData(void) 
描  述: 更新四元数数据	
输入值: 无
输出值: 无
返回值: 无
**********************************************************/
void Cal_TsData(void) 
{ 	 
	//16位寄存器输出-32767-32767  相当于 (4G * 9.8)m/s的重力加速度对应32768  每m/s对应 32767 / (4G * 9.8)  ACC_RealData.X/(32767 / (4G * 9.8))
	acce_f.X = ACC_RealData.X * ACC_Gain;			
  acce_f.Y = ACC_RealData.Y * ACC_Gain;
	acce_f.Z = ACC_RealData.Z * ACC_Gain;
  //16位寄存器输出-32767-32767  相当于 2000dps对应32768  每dps对应 32767 / 2000     GRY_RealData.X/(32767 / 2000)    
	gyro_f.X = GRY_RealData.X * Gyr_Gain;			
	gyro_f.Y = GRY_RealData.Y * Gyr_Gain;
	gyro_f.Z = GRY_RealData.Z * Gyr_Gain;
}
/*********************************************************
函数名: float get_acc_mod(void)
描  述: 得到静止情况下加速度的输出
输入值: 无
输出值: 无
返回值: 无
*为什么 不用单个的数据去判断？
*当无人机从不同的角度降落的时候,或者地面不平的时候,保证飞机平稳降落
**********************************************************/
float get_acc_mod(void)
{
  float AccMod = sqrtf(acce_f.X*acce_f.X+acce_f.Y*acce_f.Y+acce_f.Z*acce_f.Z); 
  return AccMod;
}
 


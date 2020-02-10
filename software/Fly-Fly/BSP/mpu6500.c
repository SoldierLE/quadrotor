/****************************************************************************
*						   									�ŷ�ʵս����	
*1.����֧��QQ:3403129447  �ֻ�:15618987206
*2.΢�Ź��ں�:�ŷ�ʵս����
*3.�Ա�����:https://shop105175919.taobao.com/
*4.����:�ŷ�ʵս����								  
*���������ѧϰʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
*****************************************************************************/
#include "stm32f37x.h"
#include "mpu6500.h"
#include "I2C.h"
#include "Sys.h"
#include "math.h"


u8 InitOffset = 0;
u16 turbulen = 0;
u16 Peace, PeaceTime; 
int16_XYZ ACC_Offset,GRY_Offset;     // ������ƫ����
float_XYZ acce_f,gyro_f;        //IMU ����
int16_XYZ ACC_RealData,GRY_RealData;
 
bool Gyro_Offset_Flag=false;   // IMUƫ������ȡ��־
bool GRY_Stable_Flag=false;     // �����������ȶ���־
/*********************************************************
������: void Init_Uart(void) 
��  ��: ��ʼ������1
����ֵ: ��
���ֵ: ��
����ֵ: ��
**********************************************************/
void mpu6500_init(void)
{
	i2c_reg_write(IMU_I2C,0x80,MPU6500_Addr,PWR_MGMT_1);	//��λMPU6500	
	Delay_ms(100);
	i2c_reg_write(IMU_I2C,0x07,MPU6500_Addr,MPU6500_SIGNAL_PATH_RESET);//��λ�ź�·��
	
	i2c_reg_write(IMU_I2C,0x03,MPU6500_Addr,PWR_MGMT_1);//�Զ�ѡ�����ʱ��Դ    
	i2c_reg_write(IMU_I2C,0x00,MPU6500_Addr,SMPLRT_DIV);  //�����ʷ�Ƶ    
	i2c_reg_write(IMU_I2C,0x01,MPU6500_Addr,CONFIG);  			//DLPF_CFG = 1;	  ���ֵ�ͨ�˲�������	
	i2c_reg_write(IMU_I2C,0x05,MPU6500_Addr,MPU6500_ACCEL_CONFIG2); //A_DLPF_CFG = 5; ���ֵ�ͨ�˲�������
	i2c_reg_write(IMU_I2C,0x20,MPU6500_Addr,MPU6500_INT_PIN_CFG);//Interrupt status is cleared if any read operation is performed.
	i2c_reg_write(IMU_I2C,0x01,MPU6500_Addr,MPU6500_INT_ENABLE);//Enable interrupt for wake on motion to propagate to interrupt pin.
	  
	i2c_reg_write(IMU_I2C,3<<3,MPU6500_Addr,GYRO_CONFIG);  //+/-2000dps   ���ٶ�     ��/S
	i2c_reg_write(IMU_I2C,1<<3,MPU6500_Addr,ACCEL_CONFIG); //+/-4g				���ٶ�	
}
/*********************************************************
������: void READ_MPU6500(void) 
��  ��: ������
����ֵ: ��
���ֵ: ��
����ֵ: ��
**********************************************************/
void READ_MPU6500(void)
{
	u8 BUF_6500_buff[14];
	int16_XYZ Old_GRY;

	Old_GRY.X=GRY_RealData.X;
	Old_GRY.Y=GRY_RealData.Y;
	Old_GRY.Z=GRY_RealData.Z;

	i2c_read(IMU_I2C,BUF_6500_buff,MPU6500_Addr,ACCEL_XOUT_H,14);			//0x3B-0x48 ��������ֲ�
	ACC_RealData.X=(( ((int16_t)BUF_6500_buff[0]) <<8) |BUF_6500_buff[1]);
	ACC_RealData.Y=(( ((int16_t)BUF_6500_buff[2]) <<8) |BUF_6500_buff[3]);	
	ACC_RealData.Z=(( ((int16_t)BUF_6500_buff[4]) <<8) |BUF_6500_buff[5]);			//���ٶ�����

	GRY_RealData.X=(( ((int16_t)BUF_6500_buff[8]) <<8) |BUF_6500_buff[9]);
	GRY_RealData.Y=(( ((int16_t)BUF_6500_buff[10]) <<8) |BUF_6500_buff[11]);	
	GRY_RealData.Z=(( ((int16_t)BUF_6500_buff[12]) <<8) |BUF_6500_buff[13]);		//�Ƕ�����
	//У��ƫ����
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
������: void Get_GRYOffset(void)
��  ��: �ھ�ֹ��״̬�¼�������ȡ30��ƽ��ֵ����ɻ���ƫ����,�����ھ�ֹ״̬�����ݵı仯
����ֵ: ��
���ֵ: ��
����ֵ: ��
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
������: void Cal_TsData(void) 
��  ��: ������Ԫ������	
����ֵ: ��
���ֵ: ��
����ֵ: ��
**********************************************************/
void Cal_TsData(void) 
{ 	 
	//16λ�Ĵ������-32767-32767  �൱�� (4G * 9.8)m/s���������ٶȶ�Ӧ32768  ÿm/s��Ӧ 32767 / (4G * 9.8)  ACC_RealData.X/(32767 / (4G * 9.8))
	acce_f.X = ACC_RealData.X * ACC_Gain;			
  acce_f.Y = ACC_RealData.Y * ACC_Gain;
	acce_f.Z = ACC_RealData.Z * ACC_Gain;
  //16λ�Ĵ������-32767-32767  �൱�� 2000dps��Ӧ32768  ÿdps��Ӧ 32767 / 2000     GRY_RealData.X/(32767 / 2000)    
	gyro_f.X = GRY_RealData.X * Gyr_Gain;			
	gyro_f.Y = GRY_RealData.Y * Gyr_Gain;
	gyro_f.Z = GRY_RealData.Z * Gyr_Gain;
}
/*********************************************************
������: float get_acc_mod(void)
��  ��: �õ���ֹ����¼��ٶȵ����
����ֵ: ��
���ֵ: ��
����ֵ: ��
*Ϊʲô ���õ���������ȥ�жϣ�
*�����˻��Ӳ�ͬ�ĽǶȽ����ʱ��,���ߵ��治ƽ��ʱ��,��֤�ɻ�ƽ�Ƚ���
**********************************************************/
float get_acc_mod(void)
{
  float AccMod = sqrtf(acce_f.X*acce_f.X+acce_f.Y*acce_f.Y+acce_f.Z*acce_f.Z); 
  return AccMod;
}
 


#include "stm32f37x.h"
#include "GlobalVal.h"

 
#define Gyr_Gain 	0.06103f        // +-2000 DEG/s     2000 deg/s/32768
#define ACC_Gain 	0.0011963f      // +-4G							4g * 9.8 / 32768

float idle_acc_mod=0.0f;  //����ʱ���ٶ�ģ       
float_XYZ acce_f,gyro_f;        //IMU ����

int16_XYZ ACC_Offset,GRY_Offset;     // ������ƫ����
float Vol_Real=0.0f,bat_adc=2450.0f;    //��ص�ѹֵ

float_RPY Q_ANGLE,Q_Rad;

float IMU_P = 0.8;

float_XYZ acc_f,gry_f;        //IMU ����
float_XYZ EXP_ANGLE,EXP_RateOffset;
float YawRateGol;


















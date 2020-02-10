#include "stm32f37x.h"
#include "GlobalVal.h"

 
#define Gyr_Gain 	0.06103f        // +-2000 DEG/s     2000 deg/s/32768
#define ACC_Gain 	0.0011963f      // +-4G							4g * 9.8 / 32768

float idle_acc_mod=0.0f;  //怠速时加速度模       
float_XYZ acce_f,gyro_f;        //IMU 数据

int16_XYZ ACC_Offset,GRY_Offset;     // 陀螺仪偏移量
float Vol_Real=0.0f,bat_adc=2450.0f;    //电池电压值

float_RPY Q_ANGLE,Q_Rad;

float IMU_P = 0.8;

float_XYZ acc_f,gry_f;        //IMU 数据
float_XYZ EXP_ANGLE,EXP_RateOffset;
float YawRateGol;


















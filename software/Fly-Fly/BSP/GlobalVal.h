#ifndef _GLOBALVAL_H_
#define _GLOBALVAL_H_

#ifdef	__cplusplus
extern "C" {
#endif
#include "stm32f37x.h"
#include "Sys.h"
	
typedef enum
{
  false = 0, true  = !false,
}bool;	
	
typedef enum
{
  RFMODE_3AXIE =0,
  RFMODE_FLIP  =1,
  RFMODE_6AXIE =2
}RF_MODE;




typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
} int16_XYZ;  


typedef struct
{
	float X;
	float Y;
	float Z;
} float_XYZ; 


typedef struct
{
	float Roll;
	float Pitch;
	float Yaw;
} float_RPY; 


typedef struct PID
{
    float kp,ki,kd,PreErr,Pout,Iout,Dout,I_sum,Dt,Imax;
}PID;


//RF数据结构
typedef struct
{
	float thr;
	float rud;
	float ele;
	float ail;
	uint8_t sw1;
	uint8_t sw2;
} RFDATA_t;



typedef enum
{
	FlyStop  = 0,
	FlyIdle  = 1,
	FlyStart = 2
}FLY_STATE;

typedef enum
{
  NORMAL    =0,
	LowPower1 =1,
  LowPower2 =2,
}VOLTAGE_STATE;



//------------全局变量声明------------
extern uint8_t rf_raw_data[13];

extern uint16_t rf_bind_count;


extern float idle_acc_mod;
extern float throttle;
extern FLY_STATE fly_state;
extern VOLTAGE_STATE voltage_state;
extern float_XYZ acce_f,gyro_f;




extern volatile uint32_t systick;   //系统时钟变量
extern int16_XYZ ACC_Offset,GRY_Offset;
extern bool Gyro_Offset_Flag;
extern bool GRY_Stable_Flag;

extern float_RPY Q_ANGLE,Q_Rad;	

extern float_XYZ ACC_Est;
extern float vel ;
extern float EstAlt;

extern PID Pitch,Roll;
extern PID PitchRate,RollRate,YawRate,Thr_Alt,AltAcc,AltVel;
extern float rollRateDesired;
extern float pitchRateDesired;
extern float yawRateDesired;

extern float ACC_EstZsum,ACC_EstZCount;


extern u8 RF_Mode;

extern float IMU_P;

extern float_XYZ EXP_ANGLE, EXP_RateOffset;
extern float YawRateGol;

extern float PitchOutput,RollOutput,YawOutput;    

extern volatile uint16_t PWM1,PWM2,PWM3,PWM4;

extern bool startup_init;

extern float ADC_Value;	
	
	
	
#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */




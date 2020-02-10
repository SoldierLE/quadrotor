/* 
 * File:   Sys.h
 * Author: Administrator
 *
 * Created on 2016?12?10?, ??9:31
 */
#ifndef SYS_H
#define	SYS_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "stm32f37x.h"	
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

typedef enum
{
	FlyStop  = 0,
	FlyIdle  = 1,
	FlyStart = 2
}FLY_STATE;
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
    float kp,ki,kd,PreErr,Pout,Iout,Dout,I_sum,Dt,Imax,Err,LErr,LLerr;
}PID;
	
	void SysTick_Init(void);
void Delay_us(unsigned int data);
	void Delay_ms(unsigned int data);
float applyDeadband(float value, float deadband);
float constrain_float(float amt, float low, float high);

float curve_ctrl(uint16_t raw_data);
static float Get_64K(float Ax,float Ay,float Bx,float By);
static float Get_64B(float Ax,float Ay,float Bx,float By);
void calc_curve(void);
void attitude_pid_1ms_task(void);
void flip_3ms_task(void);
void attitude_quat_5ms_task(void);
void PID_GRYLoop(PID* pid,float EurDesir,float measure,float* Output);
void PID_EurLoop(PID* pid,float EurDesir,float measure,float* Output);
void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw);
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static float invSqrt(float x);
void PID_6X_HS(void);
void PID_6X_LS(void);
void PID_Init(void);
#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */


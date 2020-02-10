#include "quaternion.h"
#include "var_global.h"
#include <math.h>



float q0=1, q1=0, q2=0, q3=0;  

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{ 
	float twoKp ;
  
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;                
  
	gx = gx * 0.0174f;
	gy = gy * 0.0174f;
	gz = gz * 0.0174f;			//度转弧度
  
	twoKp=IMU_P;
	
	// Normalise accelerometer measurement
	//单位化加速度计测量值
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	
	// Estimated direction of gravity and vector perpendicular to magnetic flux
	//机体坐标系下的Z方向向量
	halfvx = q1 * q3 - q0 * q2;
	halfvy = q0 * q1 + q2 * q3;
	halfvz = q0 * q0 - 0.5f + q3 * q3;
	
	// Error is sum of cross product between estimated and measured direction of gravity
	//加速计读取的方向与垂力加速计方向的差值，用向量差乘计算
	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);
  
	//Apply proportional feedback
	//误差累计
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;
  
	//Integrate rate of change of quaternion
	gx *= (0.5f * dt);   // pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	//gx = (gx + twoKp * halfex) * 0.5f * dt;
	//gx = gx * 0.5f * dt + twoKp * halfex * 0.5f * dt; 
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);
  
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
}

//四元数转换成欧拉角
void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw)
{
	float gx, gy, gz; // estimated gravity direction

	gx = 2.0f * (q1*q3 - q0*q2);
	gy = 2.0f * (q0*q1 + q2*q3);
	gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	*pitch = atanf(gx / sqrt(gy*gy + gz*gz)) * 57.3f;
	*roll  = atanf(gy / sqrt(gx*gx + gz*gz)) * 57.3f;
	*yaw   = atan2f(2.0f*q1*q2 - 2.0f*q0*q3, 2.0f*q0*q0 + 2.0f*q1*q1 - 1) * -57.3f;
}




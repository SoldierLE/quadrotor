#include "stm32f37x.h"
#include "Sys.h"
#include "PI.h"


/*
void IncPIDCalc(unsigned int TargetValue,unsigned int PresentValue,float* Output)
{
	Error = TargetValue - PresentValue; 
  *Output = (Kp * Error) - (Ki * LastError) + (Kd * LastLastError);
	LastError = Error;
	LastLastError = LastError; 
*/

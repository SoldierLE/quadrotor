/* 
 * File:   Sys.h
 * Author: Administrator
 *
 * Created on 2016?12?10?, ??9:31
 */
#ifndef ADC_H
#define	ADC_H
#include "stm32f37x.h"
#ifdef	__cplusplus
extern "C" {
#endif

void Init_ADC(void);
int16_t Get_Adc(int8_t ch);
float Get_Adc_Average(int8_t ch,int8_t times);
	void VoltageProt(void);
#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */



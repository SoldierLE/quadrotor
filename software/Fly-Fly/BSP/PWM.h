/* 
 * File:   Sys.h
 * Author: Administrator
 *
 * Created on 2016?12?10?, ??9:31
 */
#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

void pwm_init(void);
void Tim12_init(void);//PWM����    �⺯����
void Tim5_init(void);//PWM����
void Tim2_init(void);//PWM����

#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */


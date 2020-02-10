/* 
 * File:   Sys.h
 * Author: Administrator
 *
 * Created on 2016?12?10?, ??9:31
 */
#ifndef MOTOCTRL_H
#define	MOTOCTRL_H

#ifdef	__cplusplus
extern "C" {
#endif
#define PWMMAX   853
#define PWMMIN   0
#define PWMRANGE 853

	
	void  motor_ctrl(void);
	void PWM_Set(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);

#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */


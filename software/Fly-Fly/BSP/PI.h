/* 
 * File:   Sys.h
 * Author: Administrator
 *
 * Created on 2016?12?10?, ??9:31
 */
#ifndef PI_H
#define	PI_H

#ifdef	__cplusplus
extern "C" {
#endif
    
void IncPIDCalc(unsigned int TargetValue,unsigned int PresentValue,float* Output);

#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */


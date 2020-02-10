/* 
 * File:   Sys.h
 * Author: Administrator
 *
 * Created on 2016?12?10?, ??9:31
 */
#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif

void Init_Uart(void);
	void uart1_send_byte(unsigned char data);
	void uart1_send_nbyte(unsigned char* pbuffer,unsigned int number);
void RfDataDecode(void);
#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */


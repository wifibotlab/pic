/* 
 * File:   uart.h
 * Author: rbeal
 *
 * Created on May 5, 2017, 11:41 AM
 */

#ifndef UART_H
#define	UART_H


void InitUART1();
void InitUART2();

void WriteUART1(unsigned char c);
void WriteUART2(unsigned char c);
unsigned char ReadUART1(void);
unsigned char ReadUART2(void);

void UART_str(char *str);

#endif	/* UART_H */


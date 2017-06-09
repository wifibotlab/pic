/* 
 * File:   sht21.h
 * Author: rbeal
 *
 * Created on May 9, 2017, 09:47 AM
 */

#ifndef SHT21_H
#define	SHT21_H

#define SHT21_ADDR        0x80
#define SHT21_GET_T_HOLD  0xE3
#define SHT21_GET_RH_HOLD 0xE5

void init_SHT21(void);
float get_SHT21_temp(void);
double get_SHT21_humi(void);

#endif	/* SHT21_H */


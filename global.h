/* 
 * File:   global.h
 * Author: rbeal
 *
 * Created on May 5, 2017, 11:25 AM
 */

#ifndef GLOBAL_H
#define	GLOBAL_H

#define FCY 60000000

struct modbus {
    char enable;
    short wtd;
    unsigned short speed_L;
    unsigned short speed_R;
};

extern struct modbus modbus;

#endif	/* GLOBAL_H */


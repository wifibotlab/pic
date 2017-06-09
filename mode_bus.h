/* 
 * File:   mode_bus.h
 * Author: rbeal
 *
 * Created on May 9, 2017, 5:47 PM
 */

#ifndef MODE_BUS_H
#define	MODE_BUS_H

/*
 * SYNC BIT (165)
 * SIZE
 * CMD
 * DATA ...
 * CRC_L
 * CDC_H
 */

#define MODBUS_SYNC_BYTE 254

// ############ CMD LIST ###############

#define MODE_BUS     1
#define MODE_PAS_BUS 0
/* 
 * 8- Reserved
 */

#define MODBUS_CMD_SET_WTD    0x2
/*
 * DATA OCT (1): wtd_h
 * DATA OCT (2): wtd_l
 */

#define MODBUS_CMD_SPEED 0x3
/*
 * DATA OCT (1): Left_L
 * DATA OCT (2): Left_H
 * DATA OCT (3): Right_L
 * DATA OCT (4): Right_H
 */

#define MODBUS_CMD_WTD 0x4
/*
 * DATA OCT (1): WTD_L
 * DATA OCT (2): WTD_H
 */

#define MODBUS_DEBUG      0xF0
#define MODBUS_STREAM     0xF1


void mode_bus(unsigned char *buff);

#endif	/* MODE_BUS_H */


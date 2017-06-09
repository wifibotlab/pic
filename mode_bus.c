#include "mode_bus.h"
#include "global.h"
#include "uart.h"
#include "vrac.h"

extern short maxspeed;
extern short speedmarge;

//Motors variable
extern short aux;
extern short aux2;

extern char ch1;
extern char ch2;
extern char ch3;
extern char ch4;

extern int dog;

void mode_bus(unsigned char *buff) {
    
    
    
    unsigned char cmdflag;
    
    switch (buff[2]) { // CMD
            
        case MODBUS_CMD_SPEED:
            
            modbus.speed_L = (unsigned short) buff[3] + (unsigned short) (buff[4]<<8);
            modbus.speed_R = (unsigned short) buff[5] + (unsigned short) (buff[6]<<8);
            
            if (modbus.speed_L > (maxspeed - speedmarge))
                modbus.speed_L =  maxspeed - speedmarge;
            
            if (modbus.speed_R > (maxspeed - speedmarge))
                modbus.speed_R =  maxspeed - speedmarge;
            
            aux  = modbus.speed_L;
            aux2 = modbus.speed_R;
            
            cmdflag = buff[7];

            if(cmdflag&0x01) ch1=1;
            else if(!(cmdflag&0x01)) ch1=0;

            if(cmdflag&0x02) ch2=1;
            else if(!(cmdflag&0x02)) ch2=0;

            if(cmdflag&0x04) ch3=1;
            else if(!(cmdflag&0x04)) ch3=0;

            if(cmdflag&0x08) ch4=1;
            else if(!(cmdflag&0x08)) ch4=0;
            
            dog = 0;
            
            break;
            
        case MODBUS_CMD_WTD:
            
            modbus.wtd = (unsigned short) buff[3] + (unsigned short) (buff[4]<<8);
            break;
                        
    }
}
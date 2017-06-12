#include "global.h"
#include "sht21.h"
#include "I2C.h"
#include "math.h"

void init_SHT21(void) {
    
    I2C_write_byte(SHT21_ADDR, 0xE6, 0x01); // default mode
}

double get_SHT21_temp(void) {
    
    unsigned int raw;
    unsigned char raw_8b[3];
    
    I2C_read_multibytes(SHT21_ADDR, SHT21_GET_T_HOLD, raw_8b, 3);
    
    raw = raw_8b[0]<<8 | (raw_8b[1] & 0xFFFC);
    
    return ( -46.85 + 175.72 / 65536.0 * raw );
}

double get_SHT21_hygro(void) {
    
    unsigned int raw;
    unsigned char raw_8b[3];
    
    I2C_read_multibytes(SHT21_ADDR, SHT21_GET_RH_HOLD, raw_8b, 3);
    
    raw = raw_8b[0]<<8 | (raw_8b[1] & 0xFFFC);
    
    return ( -6.0 + 125.0 / 65536.0 * raw );
}

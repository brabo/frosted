#ifndef INC_I2C
#define INC_I2C

#include "gpio.h"

/* TX, RX, RTS, CTS, CK*/
#define MAX_I2C_PINS 2

struct i2c_addr {
    uint8_t devidx;
    uint32_t base;
    uint32_t irq;
    uint32_t rcc;
    uint32_t freq;
    uint8_t address;
};


void i2c_init(struct fnode *dev, const struct i2c_addr i2c_addrs[], int num_i2cs);

#endif


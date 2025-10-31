#ifndef MAX_H
#define MAX_H

#include "hardware/i2c.h"

typedef struct MAX30100{ //! i2c addr, temp, ir, red, int,
    uint8_t address;
    i2c_inst_t *instance;
    float ltemp;
    uint16_t lred;
    uint16_t lir;
    float temperature;
    uint16_t red;
    uint16_t ir;
    uint8_t interrupts;
} MAX30100_t;

//@brief returns 0 if everyting went smooth, reads ir, red, and temperature data
bool MAX30100_init(MAX30100_t *sens, i2c_inst_t *inst, uint8_t addr);

void MAX30100_cfg(MAX30100_t *sens, uint8_t mode, uint8_t pulse_width, uint8_t sample, uint8_t ir_current, uint8_t red_current, bool hi_res);

bool MAX30100_read(MAX30100_t *sens);

void MAX30100_FIFO_CLR(MAX30100_t * sens);

void MAX30100_Start_temp(MAX30100_t *sens);

bool MAX30100_read_temp(MAX30100_t *sens);

#endif
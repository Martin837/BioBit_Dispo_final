#include "Max.h"
#include "hardware/i2c.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"

//Registers
const uint8_t INT_STATUS = 0x0;
const uint8_t INT_ENABLE = 0x1;
const uint8_t FIFO_WR_PTR = 0x2;
const uint8_t FIFO_OVR_COUNTER = 0x3;
const uint8_t FIFO_RD_PTR = 0x4;
const uint8_t FIFO_DATA = 0x5;
const uint8_t MODE_CFG = 0x6;
const uint8_t SPO2_CFG = 0x7;
const uint8_t LED_CFG = 0x9;
const uint8_t TEMP_INT = 0x16;
const uint8_t TEMP_FRAC = 0x17;

void set_pulse_width(i2c_inst_t *inst,uint8_t addr, uint8_t width){
    //leer, modificar
    uint8_t prev = 0;

    i2c_write_blocking(inst, addr, &SPO2_CFG, 1, true);
    i2c_read_blocking(inst, addr, &prev, 1, false);

    uint8_t data1[] = {SPO2_CFG, (prev & 0xfc) | width};
    i2c_write_blocking(inst, addr, data1, 2, false);
}

void set_sample_rate(i2c_inst_t *inst,uint8_t addr, uint8_t sr){
    //leer, modificar
    uint8_t prev = 0;

    i2c_write_blocking(inst, addr, &SPO2_CFG, 1, true);
    i2c_read_blocking(inst, addr, &prev, 1, false);

    uint8_t data1[] = {SPO2_CFG, (prev & 0xe3) | (sr << 2)};
    i2c_write_blocking(inst, addr, data1, 2, false);
}

void set_hi_res(i2c_inst_t *inst,uint8_t addr, bool enabled){
    //leer, modificar
    uint8_t prev = 0;

    i2c_write_blocking(inst, addr, &SPO2_CFG, 1, true);
    i2c_read_blocking(inst, addr, &prev, 1, false);


    uint8_t data1[] = {SPO2_CFG, 0};

    if(enabled)
        data1[1] = prev | (1 << 6); 
    else
        data1[1] = prev & ~(1<<6);
    i2c_write_blocking(inst, addr, data1, 2, false);
}

void MAX30100_Start_temp(MAX30100_t *sens){
    uint8_t prev = 0;

    i2c_write_blocking(sens->instance, sens->address, &MODE_CFG, 1, true);
    i2c_read_blocking(sens->instance, sens->address, &prev, 1, false);

    uint8_t data[] = {MODE_CFG, prev | 1<<3};

    i2c_write_blocking(sens->instance, sens->address, data, 2, true);
}

bool MAX30100_read_temp(MAX30100_t *sens){
    uint8_t prev = 0;
    uint8_t buf[2] = {0};

    i2c_write_blocking(sens->instance, sens->address, &MODE_CFG, 1, true);
    i2c_read_blocking(sens->instance, sens->address, &prev, 1, false);

    if(!((prev>>3)&1)){
        sens->ltemp=sens->temperature;
        i2c_write_blocking(sens->instance, sens->address, &TEMP_INT, 1, true);
        i2c_read_blocking(sens->instance, sens->address, buf, 2, false);

        sens->temperature=buf[1]*0.0625+buf[0];
        return true;
    }

    return false;
}



bool MAX30100_init(MAX30100_t *sens, i2c_inst_t *inst, uint8_t addr){ //TODO: clear power up interrupt?
    sens->address = addr;
    sens->instance = inst;

    /*

    uint8_t rx = 0;
    uint8_t data = 0xFF;

    i2c_write_blocking(inst, addr, &data, 1, true);
    i2c_read_blocking(inst, addr, &rx, 1, false);
    
    if(rx!=0x11)
        return false;

    uint8_t data1[] = {MODE_CFG, 0b10};
    i2c_write_blocking(inst, addr, data1, 2, true);

    data = 0x03;
    set_pulse_width(inst, addr, data);

    data = 1;
    set_sample_rate(inst, addr, data);

    data = (0x0f<<4) | 0x0f;
    uint8_t send[2] = {LED_CFG, data};
    i2c_write_blocking(inst, addr, send, 2, false);

    set_hi_res(inst, addr, true); */

    return true;
}

void MAX30100_cfg(MAX30100_t *sens, uint8_t mode, uint8_t pulse_width, uint8_t sample, uint8_t ir_current, uint8_t red_current, bool hi_res){
    uint8_t data[2] = {MODE_CFG, mode};
    i2c_write_blocking(sens->instance, sens->address, data, 2, false);

    set_pulse_width(sens->instance, sens->address, pulse_width);

    set_sample_rate(sens->instance, sens->address, sample);

    data[0] = LED_CFG;
    data[1] = (red_current << 4) | ir_current;
    i2c_write_blocking(sens->instance, sens->address, data, 2, false);

    set_hi_res(sens->instance, sens->address, hi_res);

    return;
}

bool MAX30100_read(MAX30100_t *sens){

    sens->lir = sens->ir;
    sens->red = sens->red;
    int16_t temp = 0;
    uint8_t data[5] = {0};
    uint8_t wr_ptr = 0, rd_ptr = 0;

    uint8_t buffer[0x10*4];
    uint8_t toRead = 0;



    i2c_write_blocking(sens->instance, sens->address, &FIFO_WR_PTR, 1, true);
    i2c_read_blocking(sens->instance, sens->address, &wr_ptr, 1, false);

    i2c_write_blocking(sens->instance, sens->address, &FIFO_RD_PTR, 1, true);
    i2c_read_blocking(sens->instance, sens->address, &rd_ptr, 1, false);

    toRead = (wr_ptr - rd_ptr) & (0x10-1);

    if(toRead){
        i2c_write_blocking(sens->instance, sens->address, &FIFO_DATA, 1, true);
        i2c_read_blocking(sens->instance, sens->address, buffer, 4 * toRead, false);


        for (uint8_t i=0 ; i < toRead ; ++i) {
            // Warning: the values are always left-aligned
            sens->ir = (uint16_t)((buffer[i*4] << 8) | buffer[i*4 + 1]);
            sens->red = (uint16_t)((buffer[i*4 + 2] << 8) | buffer[i*4 + 3]);           
        }

        return 1;
    }
    else
        return 0;
}

void MAX30100_FIFO_CLR(MAX30100_t * sens){
    uint8_t data[2] = {FIFO_WR_PTR, 0};

    i2c_write_blocking(sens->instance, sens->address, data, 2, false);

    data[0] = FIFO_RD_PTR;
    data[1] = 0;
    i2c_write_blocking(sens->instance, sens->address, data, 2, false);

    data[0] = FIFO_OVR_COUNTER;
    data[1] = 0;
    i2c_write_blocking(sens->instance, sens->address, data, 2, false);
}
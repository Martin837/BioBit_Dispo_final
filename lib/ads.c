#include "ads.h"


void ads_init (ADS_t *sensor, i2c_inst_t *instance, uint8_t address){
    sensor->inst = instance;
    sensor->addr = address;
    sensor->valor0 = 0;
    sensor->valor1 = 0;

    // Disable auto sequence by default
    uint8_t data1[] = {AUTO_SEQ_CHEN, 0};
    i2c_write_blocking(sensor->inst, sensor->addr, data1, 2, false);
}


bool ads_start(ADS_t *sensor){
    uint8_t data[] = {DATA_BUFFER_OPMODE, 1}; // enable data buffer
    int written = i2c_write_blocking(sensor->inst, sensor->addr, data, 2, false);
    if (written != 2) return false;

    uint8_t data1[] = {START_SEQUENCE, 1};
    written = i2c_write_blocking(sensor->inst, sensor->addr, data1, 2, false);
    return (written == 2);
}

bool ads_stop(ADS_t *sensor){
    uint8_t data[] = {DATA_BUFFER_OPMODE, 0}; // disable data buffer
    int written = i2c_write_blocking(sensor->inst, sensor->addr, data, 2, false);
    if (written != 2) return false;

    uint8_t data1[] = {ABORT_SEQUENCE, 1};
    written = i2c_write_blocking(sensor->inst, sensor->addr, data1, 2, false);
    return (written == 2);
}

bool ads_config_channel0(ADS_t *sensor){
    // Configure input so only channel 0 is enabled in the channel input configuration register.
    // The exact bit layout depends on the ADS7142 datasheet. This example writes 0x01 to
    // CHANNEL_INPUT_CFG (enable CH0 only) — adjust if your device requires a different value.
    uint8_t cfg[] = {CHANNEL_INPUT_CFG, 0x01};
    int written = i2c_write_blocking(sensor->inst, sensor->addr, cfg, 2, false);
    return (written == 2);
}

bool ads_config_ch0_single_ended(ADS_t *sensor){
    // Many ADCs have a channel input configuration register where each channel
    // can be enabled and single/ differential mode selected. This implementation
    // enables CH0 as single-ended with AINCOM as the negative reference.
    // The exact bitfields depend on the ADS7142 datasheet; here we write 0x01
    // to enable CH0 only. If your device requires a different value (e.g., a
    // separate bit for single-ended mode), update this accordingly.
    uint8_t cfg[] = {CHANNEL_INPUT_CFG, 0x01};
    int written = i2c_write_blocking(sensor->inst, sensor->addr, cfg, 2, false);
    return (written == 2);
}

bool ads_read_channel0(ADS_t *sensor, int16_t *out){
    // Read accumulator MSB/LSB for channel 0. ACC_CH0_MSB and ACC_CH0_LSB are defined.
    // We'll read two bytes starting at ACC_CH0_MSB.
    uint8_t reg = ACC_CH0_MSB;
    uint8_t rx[2] = {0,0};
    int written = i2c_write_timeout_us(sensor->inst, sensor->addr, &reg, 1, true, 1000);
    if (written == PICO_ERROR_TIMEOUT) return false;
    int read = i2c_read_timeout_us(sensor->inst, sensor->addr, rx, 2, false, 1000);
    if (read == PICO_ERROR_TIMEOUT) return false;

    uint16_t val = (uint16_t)((rx[0] << 8) | rx[1])>>4;
    *out = val;
    return true;
}

bool ads_readFirst (ADS_t *sensor){
    // Read four bytes: CH0 MSB, CH0 LSB, CH1 MSB, CH1 LSB
    uint8_t reg = ACC_CH0_MSB;
    uint8_t rx[4] = {0,0,0,0};
    int written = i2c_write_blocking(sensor->inst, sensor->addr, &reg, 1, true);
    if (written != 1) return false;
    int read = i2c_read_blocking(sensor->inst, sensor->addr, rx, 4, false);
    if (read != 4) return false;

    sensor->valor0 = (int16_t)((rx[0] << 8) | rx[1]);
    sensor->valor1 = (int16_t)((rx[2] << 8) | rx[3]);
    return true;
}
#ifndef ADS_H
#define ADS_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

//* Registros
#define WKEY                      0x17
#define DEVICE_RESET              0x14
#define OFFSET_CAL                0x15
#define OPMODE_SEL                0x1C
#define OPMODE_I2CMODE_STATUS     0x00
#define CHANNEL_INPUT_CFG         0x24
#define AUTO_SEQ_CHEN             0x20
#define START_SEQUENCE            0x1E
#define ABORT_SEQUENCE            0x1F
#define SEQUENCE_STATUS           0x04
#define OSC_SEL                   0x18
#define nCLK_SEL                  0x19
#define DATA_BUFFER_OPMODE        0x2C
#define DOUT_FORMAT_CFG           0x28
#define DATA_BUFFER_STATUS        0x01
#define ACC_EN                    0x30
#define ACC_CH0_LSB               0x08
#define ACC_CH0_MSB               0x09
#define ACC_CH1_LSB               0x0A
#define ACC_CH1_MSB               0x0B
#define ACCUMULATOR_STATUS        0x02
#define ALERT_DWC_EN              0x37
#define ALERT_CHEN                0x34
#define DWC_HTH_CH0_MSB           0x39
#define DWC_HTH_CH0_LSB           0x38
#define DWC_LTH_CH0_MSB           0x3B
#define DWC_LTH_CH0_LSB           0x3A
#define DWC_HYS_CH0               0x40
#define DWC_HTH_CH1_MSB           0x3D
#define DWC_HTH_CH1_LSB           0x3C
#define DWC_LTH_CH1_MSB           0x3F
#define DWC_LTH_CH1_LSB           0x3E
#define DWC_HYS_CH1               0x41
#define PRE_ALT_MAX_EVENT_COUNT   0x36
#define ALERT_TRIG_CHID           0x03
#define ALERT_LOW_FLAGS           0x0C
#define ALERT_HIGH_FLAGS          0x0E
//* Registros

typedef struct ADS{
    i2c_inst_t *inst;
    uint8_t addr;
    /* accumulator / channel values are 16-bit */
    int16_t valor0;
    int16_t valor1;
} ADS_t;

void ads_init (ADS_t *sensor, i2c_inst_t *instance, uint8_t address);
bool ads_start(ADS_t *sensor);
bool ads_stop(ADS_t *sensor);
/** Configure the device to read only channel 0 (if device supports channel selection).
 * Returns true on success. */
bool ads_config_channel0(ADS_t *sensor);

/** Configure CH0 as single-ended positive input (AIN0 = positive, AINCOM = negative).
 * Returns true on success.
 */
bool ads_config_ch0_single_ended(ADS_t *sensor);

/** Read the 16-bit signed accumulator value for channel 0 and store it in *out.
 * Returns true on success. */
bool ads_read_channel0(ADS_t *sensor, int16_t *out);

/** Read both channel 0 and channel 1 accumulators into sensor->valor0/valor1.
 * Returns true on success. */
bool ads_readFirst (ADS_t *sensor);


#endif
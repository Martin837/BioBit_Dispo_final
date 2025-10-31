#ifndef MAX30100_SPO2_H
#define MAX30100_SPO2_H

#include <stdint.h>
#include <stdbool.h>

#define SPO2_WINDOW_SIZE 500   // 10 seconds @ 50 Hz (adjust for your sample rate)

// Tunable parameters
#define IR_DC_MIN      10000.0f
#define PI_VALID_MIN   0.008f
#define PI_VALID_MAX   0.08f
#define VALID_HYST     5

// LED current control limits (register values 0–15)
#define LED_CURRENT_MIN 0x01
#define LED_CURRENT_MAX 0x0F
#define LED_ADJ_STEP    1

typedef struct {
    float ir_dc, red_dc;
    float ir_ac_avg, red_ac_avg;
    float spo2_filtered;
    float signal_quality;
    bool  spo2_valid;

    // Adaptive LED control
    uint8_t ir_current;
    uint8_t red_current;
    int quality_low_count;
    int quality_high_count;

    // Validity tracking
    float last_pi;
    int valid_counter;
    int invalid_counter;

    // Moving window buffer
    float spo2_window[SPO2_WINDOW_SIZE];
    int window_idx;
    int window_count;
} spo2_filter_t;

void spo2_filter_init(spo2_filter_t *f, uint8_t ir_current_init, uint8_t red_current_init);
float spo2_process_batch(spo2_filter_t *f, const uint16_t *ir, const uint16_t *red, int n);
bool spo2_adjust_led_current(spo2_filter_t *f, void (*set_led_fn)(uint8_t ir, uint8_t red));

#endif

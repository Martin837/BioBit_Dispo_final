#include "max30100_spo2.h"
#include <math.h>
#include <string.h>

#define ALPHA_DC    0.96f
#define ALPHA_AC    0.85f
#define ALPHA_SPO2  0.8f
#define SPO2_A      110.0f
#define SPO2_B      25.0f

static inline float lowpass(float in, float prev, float alpha) {
    return alpha * prev + (1.0f - alpha) * in;
}

void spo2_filter_init(spo2_filter_t *f, uint8_t ir_current_init, uint8_t red_current_init) {
    memset(f, 0, sizeof(*f));
    f->spo2_valid = false;
    f->ir_current = ir_current_init;
    f->red_current = red_current_init;
}

float spo2_process_batch(spo2_filter_t *f, const uint16_t *ir, const uint16_t *red, int n) {
    float spo2 = f->spo2_filtered;

    for (int i = 0; i < n; i++) {
        float irf = (float)ir[i];
        float redf = (float)red[i];

        // DC filtering
        f->ir_dc  = lowpass(irf,  f->ir_dc,  ALPHA_DC);
        f->red_dc = lowpass(redf, f->red_dc, ALPHA_DC);

        // AC envelope
        float ir_ac  = irf  - f->ir_dc;
        float red_ac = redf - f->red_dc;
        f->ir_ac_avg  = lowpass(fabsf(ir_ac),  f->ir_ac_avg,  ALPHA_AC);
        f->red_ac_avg = lowpass(fabsf(red_ac), f->red_ac_avg, ALPHA_AC);

        // Perfusion index
        float pi = f->ir_ac_avg / f->ir_dc;
        f->signal_quality = fminf(1.0f, pi / PI_VALID_MAX);

        // Signal validity
        bool valid_signal = (f->ir_dc > IR_DC_MIN) && (pi > PI_VALID_MIN && pi < PI_VALID_MAX);

        if (valid_signal) {
            if (++f->valid_counter > VALID_HYST) {
                f->spo2_valid = true;
                f->invalid_counter = 0;
                f->valid_counter = VALID_HYST;
            }
        } else {
            if (++f->invalid_counter > VALID_HYST) {
                f->spo2_valid = false;
                f->valid_counter = 0;
                f->invalid_counter = VALID_HYST;
            }
        }

        // SpO2 computation
        if (f->spo2_valid) {
            float ratio = (f->red_ac_avg / f->red_dc) / (f->ir_ac_avg / f->ir_dc);
            spo2 = SPO2_A - SPO2_B * ratio;
            if (spo2 > 100.0f) spo2 = 100.0f;
            if (spo2 < 70.0f)  spo2 = 70.0f;

            // Store in moving window
            f->spo2_window[f->window_idx] = spo2;
            f->window_idx = (f->window_idx + 1) % SPO2_WINDOW_SIZE;
            if (f->window_count < SPO2_WINDOW_SIZE) f->window_count++;

            // Compute mean
            float sum = 0;
            for (int j = 0; j < f->window_count; j++)
                sum += f->spo2_window[j];
            float avg = sum / f->window_count;

            // Smooth result
            f->spo2_filtered = lowpass(avg, f->spo2_filtered, ALPHA_SPO2);
        }

        f->last_pi = pi;
    }

    return f->spo2_filtered;
}

bool spo2_adjust_led_current(spo2_filter_t *f, void (*set_led_fn)(uint8_t ir, uint8_t red)) {
    bool updated = false;

    if (f->spo2_valid) {
        if (f->signal_quality > 0.9f && f->ir_current > LED_CURRENT_MIN) {
            if (++f->quality_high_count > 20) {
                f->ir_current -= LED_ADJ_STEP;
                f->red_current -= LED_ADJ_STEP;
                if (f->ir_current < LED_CURRENT_MIN) f->ir_current = LED_CURRENT_MIN;
                if (f->red_current < LED_CURRENT_MIN) f->red_current = LED_CURRENT_MIN;
                f->quality_high_count = 0;
                updated = true;
            }
        } else if (f->signal_quality < 0.5f && f->ir_current < LED_CURRENT_MAX) {
            if (++f->quality_low_count > 10) {
                f->ir_current += LED_ADJ_STEP;
                f->red_current += LED_ADJ_STEP;
                if (f->ir_current > LED_CURRENT_MAX) f->ir_current = LED_CURRENT_MAX;
                if (f->red_current > LED_CURRENT_MAX) f->red_current = LED_CURRENT_MAX;
                f->quality_low_count = 0;
                updated = true;
            }
        }
    }

    if (updated && set_led_fn) {
        set_led_fn(f->ir_current, f->red_current);
    }

    return updated;
}

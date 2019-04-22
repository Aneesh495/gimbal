/*
 * Gimbal Multi-Stage Adaptive Mechanical Notch Filter Bank
 * Suppresses gimbal structural mechanical resonance frequencies (e.g. 180 Hz, 320 Hz).
 */

#include "sensors.h"

void notch_bank_init(notch_bank_axis_t *bank, q16_t fs_hz) {
    if (!bank) return;
    
    /* Nominal resonant peaks: 180 Hz, 320 Hz, 550 Hz, 820 Hz */
    q16_t default_freqs[NOTCH_STAGES_PER_AXIS] = {
        q16_from_float(180.0f),
        q16_from_float(320.0f),
        q16_from_float(550.0f),
        q16_from_float(820.0f)
    };
    q16_t default_bw = q16_from_float(25.0f);
    
    for (int i = 0; i < NOTCH_STAGES_PER_AXIS; i++) {
        bank->center_freq_hz[i] = default_freqs[i];
        bank->bandwidth_hz[i] = default_bw;
        bank->enabled[i] = true;
        biquad_init_notch(&bank->stages[i], default_freqs[i], default_bw, fs_hz);
    }
}

q16_t notch_bank_process(notch_bank_axis_t *bank, q16_t input) {
    if (!bank) return input;
    
    q16_t signal = input;
    for (int i = 0; i < NOTCH_STAGES_PER_AXIS; i++) {
        if (bank->enabled[i]) {
            signal = biquad_process_sample(&bank->stages[i], signal);
        }
    }
    return signal;
}

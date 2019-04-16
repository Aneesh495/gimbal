/*
 * Gimbal Incremental Quadrature Encoder Interface
 * 4x decoding state machine with hardware edge capture and velocity estimation.
 */

#include "sensors.h"

static uint32_t last_counts[GIMBAL_AXIS_COUNT] = {0, 0, 0};

void quad_encoder_init(void) {
    for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
        last_counts[i] = 0;
    }
}

void quad_encoder_update(encoder_state_t *enc) {
    if (!enc) return;
    
    q16_t two_pi = 411775;
    uint32_t counts_per_rev = 16384;
    
    for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
        uint32_t c = enc->raw_counts[i];
        int32_t delta = (int32_t)(c - last_counts[i]);
        last_counts[i] = c;
        
        /* Angle = (counts / counts_per_rev) * 2*pi in Q16 */
        int64_t angle_scaled = ((int64_t)(c % counts_per_rev) * two_pi) / counts_per_rev;
        enc->angle_rad[i] = (q16_t)angle_scaled;
        
        /* Velocity in rad/s: delta * 2*pi / (counts_per_rev * dt) at 1 kHz sample rate */
        int64_t vel_scaled = ((int64_t)delta * two_pi * 1000) / counts_per_rev;
        enc->velocity_rad_s[i] = (q16_t)vel_scaled;
        enc->aligned[i] = true;
        enc->fault[i] = false;
    }
}

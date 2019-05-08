/*
 * Gimbal Motor Cogging Torque Feedforward Compensator
 * Spatial Fourier harmonic lookup table indexed by mechanical rotor angle.
 */

#include "foc.h"

static q16_t cogging_tables[GIMBAL_AXIS_COUNT][COGGING_HARMONICS];

void foc_anti_cogging_set_table(uint8_t axis, const q16_t *harmonic_table) {
    if (axis >= GIMBAL_AXIS_COUNT || !harmonic_table) return;
    for (int i = 0; i < COGGING_HARMONICS; i++) {
        cogging_tables[axis][i] = harmonic_table[i];
    }
}

q16_t foc_anti_cogging_evaluate(uint8_t axis, q16_t mechanical_angle) {
    if (axis >= GIMBAL_AXIS_COUNT) return 0;
    
    q16_t two_pi = 411775;
    while (mechanical_angle < 0) mechanical_angle += two_pi;
    while (mechanical_angle >= two_pi) mechanical_angle -= two_pi;
    
    /* Map angle to index in [0, COGGING_HARMONICS - 1] */
    int32_t idx = (int32_t)(((int64_t)mechanical_angle * COGGING_HARMONICS) / two_pi);
    if (idx < 0) idx = 0;
    if (idx >= COGGING_HARMONICS) idx = COGGING_HARMONICS - 1;
    
    return cogging_tables[axis][idx];
}

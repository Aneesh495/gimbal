/*
 * Gimbal Space Vector Pulse Width Modulation (SVPWM) Timing Generator
 * 6-sector space vector decomposition and 100 MHz timer compare counts.
 */

#include "foc.h"

/* sqrt(3) in Q16 = 1.7320508 * 65536 = 113512 */
#define SQRT3_Q16 113512

void foc_svpwm_calculate(q16_t v_alpha, q16_t v_beta, q16_t v_bus, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c, uint8_t *sector_out) {
    if (v_bus <= 0) {
        if (pwm_a) *pwm_a = PWM_TIMER_TICKS >> 1;
        if (pwm_b) *pwm_b = PWM_TIMER_TICKS >> 1;
        if (pwm_c) *pwm_c = PWM_TIMER_TICKS >> 1;
        if (sector_out) *sector_out = 0;
        return;
    }
    
    /* Normalized voltage vectors: u_alpha = v_alpha * sqrt(3) / v_bus */
    int64_t ua_scaled = ((int64_t)v_alpha * SQRT3_Q16) >> FIXED_Q16_SHIFT;
    q16_t u_alpha = q16_div((q16_t)ua_scaled, v_bus);
    q16_t u_beta  = q16_div(v_beta, v_bus);
    
    /* Project onto 60-degree non-orthogonal axes */
    q16_t v1 = u_beta;
    q16_t v2 = (q16_mul(SQRT3_Q16, u_alpha) - u_beta) >> 1;
    q16_t v3 = (-q16_mul(SQRT3_Q16, u_alpha) - u_beta) >> 1;
    
    /* Determine Sector */
    uint8_t sector = 0;
    if (v1 > 0) sector |= 1;
    if (v2 > 0) sector |= 2;
    if (v3 > 0) sector |= 4;
    
    q16_t t1 = 0, t2 = 0;
    switch (sector) {
        case 1: /* Sector 2 */
            t1 = -v2; t2 = -v3; sector = 2; break;
        case 2: /* Sector 6 */
            t1 = -v3; t2 = -v1; sector = 6; break;
        case 3: /* Sector 1 */
            t1 = v2; t2 = v1; sector = 1; break;
        case 4: /* Sector 4 */
            t1 = -v1; t2 = -v2; sector = 4; break;
        case 5: /* Sector 3 */
            t1 = v1; t2 = v3; sector = 3; break;
        case 6: /* Sector 5 */
            t1 = v3; t2 = v2; sector = 5; break;
        default:
            t1 = 0; t2 = 0; sector = 1; break;
    }
    
    /* Over-modulation scaling */
    q16_t t_sum = t1 + t2;
    if (t_sum > FIXED_Q16_ONE) {
        t1 = q16_div(t1, t_sum);
        t2 = q16_div(t2, t_sum);
    }
    
    q16_t t0 = (FIXED_Q16_ONE - t1 - t2) >> 1;
    q16_t ta = 0, tb = 0, tc = 0;
    
    switch (sector) {
        case 1:
            ta = t1 + t2 + t0; tb = t2 + t0; tc = t0; break;
        case 2:
            ta = t1 + t0; tb = t1 + t2 + t0; tc = t0; break;
        case 3:
            ta = t0; tb = t1 + t2 + t0; tc = t2 + t0; break;
        case 4:
            ta = t0; tb = t1 + t0; tc = t1 + t2 + t0; break;
        case 5:
            ta = t2 + t0; tb = t0; tc = t1 + t2 + t0; break;
        case 6:
            ta = t1 + t2 + t0; tb = t0; tc = t1 + t0; break;
    }
    
    if (pwm_a) *pwm_a = (uint16_t)(((int64_t)ta * PWM_TIMER_TICKS) >> FIXED_Q16_SHIFT);
    if (pwm_b) *pwm_b = (uint16_t)(((int64_t)tb * PWM_TIMER_TICKS) >> FIXED_Q16_SHIFT);
    if (pwm_c) *pwm_c = (uint16_t)(((int64_t)tc * PWM_TIMER_TICKS) >> FIXED_Q16_SHIFT);
    if (sector_out) *sector_out = sector;
}

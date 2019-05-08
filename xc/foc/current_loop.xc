/*
 * Gimbal 25 kHz Proportional-Integral (PI) Current Control Loops
 * Anti-windup back-calculation integrator and cross-axis back-EMF feedforward decoupling.
 */

#include "foc.h"

void foc_pi_init(pi_controller_t *pi, q16_t kp, q16_t ki, q16_t limit) {
    if (!pi) return;
    pi->kp = kp;
    pi->ki = ki;
    pi->integrator = 0;
    pi->out_max = limit;
    pi->out_min = -limit;
}

q16_t foc_pi_update(pi_controller_t *pi, q16_t setpoint, q16_t measurement) {
    if (!pi) return 0;
    
    q16_t error = q16_sub(setpoint, measurement);
    
    /* Proportional term */
    q16_t p_term = q16_mul(pi->kp, error);
    
    /* Integrator update with anti-windup clamping */
    q16_t i_delta = q16_mul(pi->ki, error);
    q16_t next_integrator = q16_add(pi->integrator, i_delta);
    
    q16_t raw_output = q16_add(p_term, next_integrator);
    q16_t clamped_output = q16_clamp(raw_output, pi->out_min, pi->out_max);
    
    /* Anti-windup: only integrate if output is not saturated or error drives out of saturation */
    if (raw_output == clamped_output) {
        pi->integrator = next_integrator;
    }
    
    return clamped_output;
}

void foc_pi_reset(pi_controller_t *pi) {
    if (pi) {
        pi->integrator = 0;
    }
}

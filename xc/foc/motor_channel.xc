/*
 * Gimbal 3-Axis Motor Channel Coordinator
 * Executes Clarke, Park, PI current regulation, cogging feedforward, and SVPWM for one axis.
 */

#include "foc.h"

void foc_motor_channel_init(motor_axis_channel_t *channel, uint8_t axis) {
    if (!channel) return;
    
    channel->params.pole_pairs = 14;
    channel->params.phase_resistance_ohms = q16_from_float(1.2f);
    channel->params.phase_inductance_henries = q16_from_float(0.00045f);
    channel->params.torque_constant_nm_a = q16_from_float(0.085f);
    channel->params.back_emf_constant_v_rad_s = q16_from_float(0.085f);
    channel->params.max_current_amps = q16_from_float(4.0f);
    channel->params.max_voltage_bus_v = q16_from_float(24.0f);
    
    /* PI current gains: Kp = 4.5, Ki = 0.05 */
    q16_t v_limit = channel->params.max_voltage_bus_v;
    foc_pi_init(&channel->pi_d, q16_from_float(4.5f), q16_from_float(0.05f), v_limit);
    foc_pi_init(&channel->pi_q, q16_from_float(4.5f), q16_from_float(0.05f), v_limit);
    
    channel->electrical_angle = 0;
    channel->target_iq = 0;
    channel->target_id = 0;
    channel->enabled = true;
}

void foc_motor_channel_update(motor_axis_channel_t *channel, q16_t ia, q16_t ib, q16_t ic, q16_t mech_angle, q16_t mech_vel) {
    if (!channel || !channel->enabled) return;
    
    /* 1. Calculate electrical angle: theta_e = (mech_angle * pole_pairs) mod 2*pi */
    q16_t two_pi = 411775;
    int64_t th_e = (int64_t)mech_angle * channel->params.pole_pairs;
    channel->electrical_angle = (q16_t)(th_e % two_pi);
    
    /* 2. Clarke transform */
    foc_clarke_transform(ia, ib, ic, &channel->state.i_alpha, &channel->state.i_beta);
    
    /* 3. Park transform */
    foc_park_transform(channel->state.i_alpha, channel->state.i_beta, channel->electrical_angle, &channel->state.i_d, &channel->state.i_q);
    
    /* 4. Anti-cogging feedforward torque addition to Iq target */
    q16_t i_cog = foc_anti_cogging_evaluate(0, mech_angle);
    q16_t effective_target_iq = q16_add(channel->target_iq, i_cog);
    
    /* 5. PI current controllers */
    channel->state.v_d = foc_pi_update(&channel->pi_d, channel->target_id, channel->state.i_d);
    channel->state.v_q = foc_pi_update(&channel->pi_q, effective_target_iq, channel->state.i_q);
    
    /* 6. Inverse Park transform */
    foc_inv_park_transform(channel->state.v_d, channel->state.v_q, channel->electrical_angle, &channel->state.v_alpha, &channel->state.v_beta);
    
    /* 7. SVPWM dwell times and timer compare generation */
    foc_svpwm_calculate(channel->state.v_alpha, channel->state.v_beta, channel->params.max_voltage_bus_v,
                        &channel->state.pwm_a, &channel->state.pwm_b, &channel->state.pwm_c, &channel->state.sector);
}

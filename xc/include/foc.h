#ifndef GIMBAL_FOC_H
#define GIMBAL_FOC_H

#include "gimbal_types.h"
#include "fixed_math.h"

#define PWM_FREQUENCY_HZ 25000
#define PWM_TIMER_TICKS  4000  /* 100 MHz clock / 25 kHz = 4000 ticks */
#define COGGING_HARMONICS 32

typedef struct {
    q16_t kp;
    q16_t ki;
    q16_t integrator;
    q16_t out_min;
    q16_t out_max;
} pi_controller_t;

typedef struct {
    q16_t phase_resistance_ohms;
    q16_t phase_inductance_henries;
    q16_t torque_constant_nm_a;
    q16_t back_emf_constant_v_rad_s;
    uint8_t pole_pairs;
    q16_t max_current_amps;
    q16_t max_voltage_bus_v;
} motor_params_t;

typedef struct {
    pi_controller_t pi_d;
    pi_controller_t pi_q;
    motor_params_t params;
    foc_state_t state;
    q16_t electrical_angle;
    q16_t target_iq;
    q16_t target_id;
    bool enabled;
} motor_axis_channel_t;

void foc_clarke_transform(q16_t ia, q16_t ib, q16_t ic, q16_t *i_alpha, q16_t *i_beta);
void foc_park_transform(q16_t i_alpha, q16_t i_beta, q16_t theta_e, q16_t *i_d, q16_t *i_q);
void foc_inv_park_transform(q16_t v_d, q16_t v_q, q16_t theta_e, q16_t *v_alpha, q16_t *v_beta);

void foc_svpwm_calculate(q16_t v_alpha, q16_t v_beta, q16_t v_bus, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c, uint8_t *sector);

void foc_pi_init(pi_controller_t *pi, q16_t kp, q16_t ki, q16_t limit);
q16_t foc_pi_update(pi_controller_t *pi, q16_t setpoint, q16_t measurement);
void foc_pi_reset(pi_controller_t *pi);

q16_t foc_anti_cogging_evaluate(uint8_t axis, q16_t mechanical_angle);
void foc_anti_cogging_set_table(uint8_t axis, const q16_t *harmonic_table);

void foc_field_weakening_update(motor_axis_channel_t *axis, q16_t electrical_speed);

void foc_motor_channel_init(motor_axis_channel_t *channel, uint8_t axis);
void foc_motor_channel_update(motor_axis_channel_t *channel, q16_t ia, q16_t ib, q16_t ic, q16_t mech_angle, q16_t mech_vel);

#endif

/*
 * Gimbal Multi-Rate State Estimator Coordinator
 * Fuses 10 kHz gyro data, 1 kHz EKF attitude prediction, and disturbance observer telemetry.
 */

#include "estimation.h"

static es_ekf_t ekf_instance;
static comp_filter_t comp_instance;
static disturbance_observer_t dob_instance;

void state_estimator_init(nav_state_t *state) {
    if (!state) return;
    
    ekf_init(&ekf_instance);
    comp_filter_init(&comp_instance, q16_from_float(0.02f));
    
    q16_t inertia[3] = {
        q16_from_float(0.008f),
        q16_from_float(0.006f),
        q16_from_float(0.004f)
    };
    dob_init(&dob_instance, q16_from_float(40.0f), inertia);
    
    state->q_inertial_to_body = quat_identity();
    state->rate_body_rad_s = vec3_zero();
    state->gyro_bias_rad_s = vec3_zero();
    state->accel_bias_m_s2 = vec3_zero();
    state->disturbance_torque_nm = vec3_zero();
    state->converged = true;
}

void state_estimator_update(nav_state_t *state, const conditioned_sensor_msg_t *sensors, q16_t dt_sec) {
    if (!state || !sensors) return;
    
    /* 1. Predict EKF */
    ekf_predict(&ekf_instance, sensors->gyro_filtered, dt_sec);
    
    /* 2. Update with accelerometer gravity reference */
    ekf_update_accel(&ekf_instance, sensors->accel_filtered);
    
    /* 3. Fast complementary observer backup */
    comp_filter_update(&comp_instance, sensors->gyro_filtered, sensors->accel_filtered, dt_sec);
    
    /* 4. Update disturbance observer */
    vec3_q16_t cmd_torque = vec3_zero();
    state->disturbance_torque_nm = dob_update(&dob_instance, cmd_torque, sensors->joint_velocities_rad_s, dt_sec);
    
    /* 5. Fill state message */
    state->q_inertial_to_body = ekf_instance.q_nominal;
    state->rate_body_rad_s = vec3_sub(sensors->gyro_filtered, ekf_instance.gyro_bias);
    state->gyro_bias_rad_s = ekf_instance.gyro_bias;
    state->last_update_tick = sensors->timestamp_ticks;
}

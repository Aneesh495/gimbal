/*
 * Gimbal Low-Latency 2 kHz Complementary Attitude Observer
 * Accelerometer tilt gravity vector fusion with dynamic acceleration gating.
 */

#include "estimation.h"

void comp_filter_init(comp_filter_t *cf, q16_t alpha) {
    if (!cf) return;
    cf->q_att = quat_identity();
    cf->alpha = alpha;
}

void comp_filter_update(comp_filter_t *cf, vec3_q16_t gyro_rad_s, vec3_q16_t accel_m_s2, q16_t dt_sec) {
    if (!cf) return;
    
    /* 1. Integrate gyro rates */
    cf->q_att = quat_integrate_rates(cf->q_att, gyro_rad_s, dt_sec);
    
    /* 2. Check linear acceleration magnitude */
    q16_t a_mag = vec3_mag(accel_m_s2);
    q16_t g_1g = (q16_t)(9.80665f * 65536.0f);
    q16_t diff = q16_abs(a_mag - g_1g);
    
    /* Only fuse if acceleration is close to 1g (+/- 1.5 m/s^2) */
    if (diff < 98304) {
        vec3_q16_t a_norm = vec3_normalize(accel_m_s2);
        
        /* Gravity vector predicted by current attitude */
        mat3_q16_t R = quat_to_matrix(cf->q_att);
        vec3_q16_t g_pred = vec3_create(R.m[2][0], R.m[2][1], R.m[2][2]);
        
        /* Correction error: error = a_norm x g_pred */
        vec3_q16_t error = vec3_cross(a_norm, g_pred);
        vec3_q16_t correction = vec3_scale(error, cf->alpha);
        
        quat_q16_t dq = quat_create(FIXED_Q16_ONE, correction.x >> 1, correction.y >> 1, correction.z >> 1);
        cf->q_att = quat_normalize(quat_mul(cf->q_att, dq));
    }
}

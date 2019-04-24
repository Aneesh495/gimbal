/*
 * Gimbal Sensor Calibration and Thermal Compensation Model
 * 3x3 misalignment matrix, scale factor corrections, and 2nd-order polynomial thermal drift model.
 */

#include "sensors.h"

void sensor_cal_init_defaults(sensor_cal_params_t *params) {
    if (!params) return;
    
    params->accel_misalignment = mat3_identity();
    params->gyro_misalignment = mat3_identity();
    
    params->accel_scale = vec3_create(FIXED_Q16_ONE, FIXED_Q16_ONE, FIXED_Q16_ONE);
    params->gyro_scale = vec3_create(FIXED_Q16_ONE, FIXED_Q16_ONE, FIXED_Q16_ONE);
    
    params->accel_bias_t0 = vec3_zero();
    params->gyro_bias_t0 = vec3_zero();
    
    params->accel_temp_coeff = vec3_zero();
    params->gyro_temp_coeff = vec3_zero();
    
    params->reference_temp_c = 25 * FIXED_Q16_ONE;
}

void sensor_cal_apply(const sensor_cal_params_t *params, const imu_sample_t *raw, imu_sample_t *cal_out) {
    if (!params || !raw || !cal_out) return;
    
    *cal_out = *raw;
    
    q16_t delta_t = raw->temperature - params->reference_temp_c;
    
    /* 1. Subtract thermal bias */
    vec3_q16_t gyro_bias = vec3_add(params->gyro_bias_t0, vec3_scale(params->gyro_temp_coeff, delta_t));
    vec3_q16_t accel_bias = vec3_add(params->accel_bias_t0, vec3_scale(params->accel_temp_coeff, delta_t));
    
    vec3_q16_t g_unbiased = vec3_sub(raw->gyro_raw, gyro_bias);
    vec3_q16_t a_unbiased = vec3_sub(raw->accel_raw, accel_bias);
    
    /* 2. Scale factor */
    vec3_q16_t g_scaled = vec3_create(
        q16_mul(g_unbiased.x, params->gyro_scale.x),
        q16_mul(g_unbiased.y, params->gyro_scale.y),
        q16_mul(g_unbiased.z, params->gyro_scale.z)
    );
    vec3_q16_t a_scaled = vec3_create(
        q16_mul(a_unbiased.x, params->accel_scale.x),
        q16_mul(a_unbiased.y, params->accel_scale.y),
        q16_mul(a_unbiased.z, params->accel_scale.z)
    );
    
    /* 3. Misalignment matrix */
    cal_out->gyro_cal = mat3_mul_vec(params->gyro_misalignment, g_scaled);
    cal_out->accel_cal = mat3_mul_vec(params->accel_misalignment, a_scaled);
}

/*
 * Gimbal Error-State Extended Kalman Filter (ES-EKF) on SO(3)
 * High-precision 1 kHz attitude observer propagating nominal quaternions
 * and 6-state error vector (attitude angle error and gyro bias).
 */

#include "estimation.h"

void ekf_init(es_ekf_t *ekf) {
    if (!ekf) return;
    ekf->q_nominal = quat_identity();
    ekf->gyro_bias = vec3_zero();
    
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            ekf->P[i][j] = (i == j) ? q16_from_float(0.01f) : 0;
        }
    }
    
    /* Process noise: attitude random walk, bias instability */
    ekf->Q_diag[0] = q16_from_float(0.0001f);
    ekf->Q_diag[1] = q16_from_float(0.0001f);
    ekf->Q_diag[2] = q16_from_float(0.0001f);
    ekf->Q_diag[3] = q16_from_float(0.00001f);
    ekf->Q_diag[4] = q16_from_float(0.00001f);
    ekf->Q_diag[5] = q16_from_float(0.00001f);
    
    /* Measurement noise: accelerometer gravity reference */
    ekf->R_accel_diag[0] = q16_from_float(0.05f);
    ekf->R_accel_diag[1] = q16_from_float(0.05f);
    ekf->R_accel_diag[2] = q16_from_float(0.05f);
    
    ekf->last_predict_tick = 0;
    ekf->initialized = true;
}

void ekf_predict(es_ekf_t *ekf, vec3_q16_t omega_raw, q16_t dt_sec) {
    if (!ekf || !ekf->initialized) return;
    
    /* 1. Unbiased angular rate */
    vec3_q16_t omega_unbiased = vec3_sub(omega_raw, ekf->gyro_bias);
    
    /* 2. Propagate nominal quaternion */
    ekf->q_nominal = quat_integrate_rates(ekf->q_nominal, omega_unbiased, dt_sec);
    
    /* 3. Error state transition matrix F = I + [ -[omega]x  -I ; 0  0 ] * dt */
    q16_t F[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            F[i][j] = (i == j) ? FIXED_Q16_ONE : 0;
        }
    }
    
    q16_t wx_dt = q16_mul(omega_unbiased.x, dt_sec);
    q16_t wy_dt = q16_mul(omega_unbiased.y, dt_sec);
    q16_t wz_dt = q16_mul(omega_unbiased.z, dt_sec);
    
    F[0][1] =  wz_dt; F[0][2] = -wy_dt;
    F[1][0] = -wz_dt; F[1][2] =  wx_dt;
    F[2][0] =  wy_dt; F[2][1] = -wx_dt;
    
    F[0][3] = -dt_sec;
    F[1][4] = -dt_sec;
    F[2][5] = -dt_sec;
    
    /* 4. Propagate covariance */
    ekf_cov_predict(ekf->P, F, ekf->Q_diag, dt_sec);
}

void ekf_update_accel(es_ekf_t *ekf, vec3_q16_t accel_meas) {
    if (!ekf || !ekf->initialized) return;
    
    /* Normalize accelerometer reading */
    vec3_q16_t a_norm = vec3_normalize(accel_meas);
    if (vec3_mag_squared(a_norm) == 0) return;
    
    /* Predicted gravity in body frame: g_body = R(q)^T * [0, 0, 1]^T */
    mat3_q16_t R = quat_to_matrix(ekf->q_nominal);
    vec3_q16_t g_pred = vec3_create(R.m[2][0], R.m[2][1], R.m[2][2]);
    
    /* Measurement residual: y = a_norm - g_pred */
    vec3_q16_t residual = vec3_sub(a_norm, g_pred);
    
    /* Measurement Jacobian H = [ [g_pred]x   0 ] */
    q16_t H[3][6];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 6; j++) H[i][j] = 0;
    }
    H[0][1] = -g_pred.z; H[0][2] =  g_pred.y;
    H[1][0] =  g_pred.z; H[1][2] = -g_pred.x;
    H[2][0] = -g_pred.y; H[2][1] =  g_pred.x;
    
    /* Compute Kalman gain K (simplified diagonal approximation for real-time DSP) */
    q16_t K[6][3];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0;
        }
    }
    
    for (int i = 0; i < 3; i++) {
        q16_t s = ekf->P[i][i] + ekf->R_accel_diag[i];
        if (s > 0) {
            K[i][i] = q16_div(ekf->P[i][i], s);
        }
    }
    
    /* Error state injection */
    vec3_q16_t delta_theta = vec3_create(
        q16_mul(K[0][0], residual.x),
        q16_mul(K[1][1], residual.y),
        q16_mul(K[2][2], residual.z)
    );
    
    /* Retract nominal quaternion: q_nom = q_nom * exp(0.5 * delta_theta) */
    quat_q16_t dq = quat_create(FIXED_Q16_ONE, delta_theta.x >> 1, delta_theta.y >> 1, delta_theta.z >> 1);
    ekf->q_nominal = quat_normalize(quat_mul(ekf->q_nominal, dq));
    
    /* Update covariance using Joseph-form */
    ekf_cov_update_joseph(ekf->P, H, K, ekf->R_accel_diag);
}

#ifndef GIMBAL_ESTIMATION_H
#define GIMBAL_ESTIMATION_H

#include "gimbal_types.h"
#include "gimbal_channels.h"
#include "vector3.h"
#include "matrix3x3.h"
#include "quaternion.h"

#define EKF_STATE_DIM 6  /* 3 attitude error, 3 gyro bias */

typedef struct {
    quat_q16_t q_nominal;
    vec3_q16_t gyro_bias;
    q16_t P[EKF_STATE_DIM][EKF_STATE_DIM];
    q16_t Q_diag[EKF_STATE_DIM];
    q16_t R_accel_diag[3];
    uint32_t last_predict_tick;
    bool initialized;
} es_ekf_t;

typedef struct {
    quat_q16_t q_att;
    q16_t alpha; /* Filter weight in Q16 */
} comp_filter_t;

typedef struct {
    vec3_q16_t estimated_torque_nm;
    vec3_q16_t z_state;
    q16_t cutoff_freq_rad_s;
    q16_t inertia_kg_m2[3];
} disturbance_observer_t;

void ekf_init(es_ekf_t *ekf);
void ekf_predict(es_ekf_t *ekf, vec3_q16_t omega_raw, q16_t dt_sec);
void ekf_update_accel(es_ekf_t *ekf, vec3_q16_t accel_meas);

void ekf_cov_predict(q16_t P[6][6], const q16_t F[6][6], const q16_t Q_diag[6], q16_t dt);
void ekf_cov_update_joseph(q16_t P[6][6], const q16_t H[3][6], const q16_t K[6][3], const q16_t R_diag[3]);

void comp_filter_init(comp_filter_t *cf, q16_t alpha);
void comp_filter_update(comp_filter_t *cf, vec3_q16_t gyro_rad_s, vec3_q16_t accel_m_s2, q16_t dt_sec);

void dob_init(disturbance_observer_t *dob, q16_t cutoff_rad_s, const q16_t inertia[3]);
vec3_q16_t dob_update(disturbance_observer_t *dob, vec3_q16_t commanded_torque, vec3_q16_t measured_vel, q16_t dt_sec);

void state_estimator_init(nav_state_t *state);
void state_estimator_update(nav_state_t *state, const conditioned_sensor_msg_t *sensors, q16_t dt_sec);

#endif

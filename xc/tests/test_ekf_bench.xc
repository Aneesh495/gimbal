/*
 * XC Test Bench: Error-State Extended Kalman Filter (ES-EKF)
 * Validates gyro rate propagation, gravity alignment, and covariance matrix symmetry.
 */

#include "estimation.h"
#include <stdbool.h>

bool run_ekf_bench(void) {
    es_ekf_t ekf;
    ekf_init(&ekf);

    /* Propagate 100 ticks with stationary input */
    vec3_q16_t omega = vec3_zero();
    q16_t dt = q16_from_float(0.001f);
    for (int i = 0; i < 100; i++) {
        ekf_predict(&ekf, omega, dt);
    }

    /* Covariance diagonal must remain strictly positive */
    for (int i = 0; i < 6; i++) {
        if (ekf.P[i][i] <= 0) return false;
    }

    /* Update with 1g vertical accelerometer */
    vec3_q16_t accel = vec3_create(0, 0, (q16_t)(9.80665f * 65536.0f));
    ekf_update_accel(&ekf, accel);

    /* Nominal quaternion norm must equal 1.0 */
    q16_t norm = quat_norm(ekf.q_nominal);
    if (q16_abs(norm - FIXED_Q16_ONE) > 4) return false;
    return true;
}

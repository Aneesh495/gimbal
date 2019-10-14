/*
 * Unit Test Suite: Error-State Extended Kalman Filter (ES-EKF)
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "estimation.h"

int main(void) {
    printf("Running test_ekf... ");
    
    es_ekf_t ekf;
    ekf_init(&ekf);
    
    /* Propagate 50 cycles */
    q16_t dt = q16_from_float(0.001f);
    vec3_q16_t omega = vec3_create(q16_from_float(0.05f), 0, 0);
    for (int i = 0; i < 50; i++) {
        ekf_predict(&ekf, omega, dt);
    }
    
    /* Update with gravity */
    vec3_q16_t accel = vec3_create(0, 0, q16_from_float(9.80665f));
    ekf_update_accel(&ekf, accel);
    
    /* Diagonal covariance must be strictly positive */
    for (int i = 0; i < 6; i++) {
        assert(ekf.P[i][i] > 0);
    }
    
    printf("PASSED\n");
    return 0;
}

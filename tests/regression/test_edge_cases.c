#include <stdlib.h>
/*
 * Regression Test Suite: Singularities, Wrap-Arounds, and Hardware Faults
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "kinematics.h"
#include "fdir.h"

int main(void) {
    printf("Running test_edge_cases... ");
    
    /* 1. Gimbal lock singularity at +/- 90 degrees pitch */
    vec3_q16_t angles;
    quat_q16_t q_extreme = quat_from_axis_angle(vec3_create(0, FIXED_Q16_ONE, 0), q16_from_float(1.56f)); /* ~89.4 deg */
    bool ik_ok = gimbal_inverse_kinematics(q_extreme, &angles);
    assert(ik_ok);
    assert(fabs(q16_to_float(angles.y)) <= 1.484f); /* Properly clamped to 85 deg limit */
    
    /* 2. Soft limit violation detection */
    limit_checker_t lc;
    limit_checker_init(&lc);
    vec3_q16_t ang_bad = vec3_create(0, q16_from_float(2.0f), 0); /* Tilt > 45 deg max */
    bool ok = limit_checker_verify(&lc, &ang_bad, NULL, NULL);
    assert(!ok);
    assert(lc.limit_tripped[GIMBAL_AXIS_TILT]);
    
    /* 3. Watchdog trip */
    watchdog_channel_t wd;
    watchdog_init(&wd, 50);
    watchdog_feed(&wd, 10);
    assert(!watchdog_check(&wd, 75)); /* Exceeded 50 ticks elapsed */
    
    printf("PASSED\n");
    return 0;
}

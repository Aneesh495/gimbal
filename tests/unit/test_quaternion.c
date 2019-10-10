/*
 * Unit Test Suite: SO(3) Unit Quaternion Kinematics and Interpolation
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "quaternion.h"

int main(void) {
    printf("Running test_quaternion... ");
    
    quat_q16_t q_id = quat_identity();
    assert(q_id.w == FIXED_Q16_ONE && q_id.x == 0 && q_id.y == 0 && q_id.z == 0);
    
    /* Axis-angle rotation 90 deg around Z */
    q16_t pi_half = q16_from_float(1.570796f);
    quat_q16_t q_z90 = quat_from_axis_angle(vec3_create(0, 0, FIXED_Q16_ONE), pi_half);
    
    vec3_q16_t vx = vec3_create(FIXED_Q16_ONE, 0, 0);
    vec3_q16_t vy = quat_rotate_vec(q_z90, vx);
    assert(abs(vy.x) < 64 && abs(vy.y - FIXED_Q16_ONE) < 64);
    
    /* Quaternion norm preservation */
    q16_t n = quat_norm(q_z90);
    assert(abs(n - FIXED_Q16_ONE) < 32);
    
    /* SLERP midpoint */
    quat_q16_t q_mid = quat_slerp(q_id, q_z90, q16_from_float(0.5f));
    q16_t y, p, r;
    quat_to_euler_zyx(q_mid, &y, &p, &r);
    assert(fabs(q16_to_float(y) - 0.785398f) < 0.01f);
    
    printf("PASSED\n");
    return 0;
}

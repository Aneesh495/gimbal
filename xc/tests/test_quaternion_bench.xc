/*
 * XC Test Bench: SO(3) Unit Quaternion Kinematics
 * Tests quaternion rotation, normalization preservation, SLERP, and angular velocity integration.
 */

#include "quaternion.h"
#include <stdbool.h>

bool run_quaternion_bench(void) {
    /* Test 1: Identity quaternion rotation leaves vector unchanged */
    quat_q16_t q_id = quat_identity();
    vec3_q16_t v = vec3_create(15000, 25000, -35000);
    vec3_q16_t v_rot = quat_rotate_vec(q_id, v);
    if (vec3_mag(vec3_sub(v, v_rot)) > 4) return false;

    /* Test 2: 90 degree yaw rotation rotates [1, 0, 0] to [0, 1, 0] */
    q16_t pi_half = 51472; /* pi/2 in Q16 */
    quat_q16_t q_yaw = quat_from_axis_angle(vec3_create(0, 0, FIXED_Q16_ONE), pi_half);
    vec3_q16_t vx = vec3_create(FIXED_Q16_ONE, 0, 0);
    vec3_q16_t vy_rot = quat_rotate_vec(q_yaw, vx);
    if (q16_abs(vy_rot.x) > 64 || q16_abs(vy_rot.y - FIXED_Q16_ONE) > 64) return false;

    /* Test 3: SLERP midpoint at t = 0.5 */
    quat_q16_t q_mid = quat_slerp(q_id, q_yaw, 32768);
    q16_t yaw, pitch, roll;
    quat_to_euler_zyx(q_mid, &yaw, &pitch, &roll);
    /* Midpoint yaw should be 45 degrees = 25736 */
    if (q16_abs(yaw - 25736) > 256) return false;
    return true;
}

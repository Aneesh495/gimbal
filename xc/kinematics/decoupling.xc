/*
 * Gimbal Dynamic Kinematic Decoupling Matrix
 * Projects carrier vehicle 3D angular rates onto gimbal joint axes to cancel base disturbances.
 */

#include "kinematics.h"

vec3_q16_t kinematics_decouple_base_motion(vec3_q16_t carrier_body_rates, vec3_q16_t gimbal_angles) {
    /*
     * For Pan (psi), Tilt (theta), Roll (phi) gimbal geometry:
     * Carrier rates [p, q, r] in body frame project through joint rotations.
     * Feedforward rates must negate the projected base rates.
     */
    q16_t pan = gimbal_angles.x;
    q16_t tilt = gimbal_angles.y;
    
    q16_t s_pan, c_pan, s_tilt, c_tilt;
    q16_cordic_sin_cos(pan, &s_pan, &c_pan);
    q16_cordic_sin_cos(tilt, &s_tilt, &c_tilt);
    
    vec3_q16_t ff;
    
    /* Decoupled feedforward rate commands */
    q16_t p = carrier_body_rates.x;
    q16_t q = carrier_body_rates.y;
    q16_t r = carrier_body_rates.z;
    
    /* Pan feedforward: primarily yaw axis */
    ff.x = -r;
    
    /* Tilt feedforward: pitch axis rotated by pan angle */
    ff.y = -(q16_mul(q, c_pan) - q16_mul(p, s_pan));
    
    /* Roll feedforward: roll axis rotated through pan and tilt */
    ff.z = -(q16_mul(p, c_pan) + q16_mul(q, s_pan));
    
    return ff;
}

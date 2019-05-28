/*
 * Gimbal Geodetic and Terrestrial Coordinate Transformations
 * Transformations between WGS-84, ECEF, Local NED, and Aircraft Body Frames.
 */

#include "kinematics.h"

void frames_ecef_to_ned(vec3_q16_t ecef_pos, vec3_q16_t ecef_ref, q16_t lat_rad, q16_t lon_rad, vec3_q16_t *ned_out) {
    if (!ned_out) return;
    
    vec3_q16_t delta = vec3_sub(ecef_pos, ecef_ref);
    
    q16_t s_lat, c_lat, s_lon, c_lon;
    q16_cordic_sin_cos(lat_rad, &s_lat, &c_lat);
    q16_cordic_sin_cos(lon_rad, &s_lon, &c_lon);
    
    /* Rotation matrix from ECEF to NED:
     * R = [ -s_lat*c_lon  -s_lat*s_lon   c_lat ]
     *     [ -s_lon         c_lon         0     ]
     *     [ -c_lat*c_lon  -c_lat*s_lon  -s_lat ]
     */
    mat3_q16_t R;
    R.m[0][0] = -q16_mul(s_lat, c_lon);
    R.m[0][1] = -q16_mul(s_lat, s_lon);
    R.m[0][2] =  c_lat;
    
    R.m[1][0] = -s_lon;
    R.m[1][1] =  c_lon;
    R.m[1][2] =  0;
    
    R.m[2][0] = -q16_mul(c_lat, c_lon);
    R.m[2][1] = -q16_mul(c_lat, s_lon);
    R.m[2][2] = -s_lat;
    
    *ned_out = mat3_mul_vec(R, delta);
}

void frames_ned_to_body(vec3_q16_t ned_vec, quat_q16_t q_ned_to_body, vec3_q16_t *body_out) {
    if (!body_out) return;
    *body_out = quat_rotate_vec(q_ned_to_body, ned_vec);
}

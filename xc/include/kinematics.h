#ifndef GIMBAL_KINEMATICS_H
#define GIMBAL_KINEMATICS_H

#include "gimbal_types.h"
#include "vector3.h"
#include "matrix3x3.h"
#include "quaternion.h"

typedef struct {
    q16_t pos_target;
    q16_t pos_current;
    q16_t vel_current;
    q16_t acc_current;
    q16_t max_vel;
    q16_t max_acc;
    q16_t max_jerk;
    bool active;
} scurve_profile_t;

typedef struct {
    q16_t latitude_deg;
    q16_t longitude_deg;
    q16_t altitude_m;
} geodetic_coord_t;

void frames_ecef_to_ned(vec3_q16_t ecef_pos, vec3_q16_t ecef_ref, q16_t lat_rad, q16_t lon_rad, vec3_q16_t *ned_out);
void frames_ned_to_body(vec3_q16_t ned_vec, quat_q16_t q_ned_to_body, vec3_q16_t *body_out);

void scurve_init(scurve_profile_t *prof, q16_t max_v, q16_t max_a, q16_t max_j);
void scurve_set_target(scurve_profile_t *prof, q16_t target_pos);
void scurve_step(scurve_profile_t *prof, q16_t dt_sec);

vec3_q16_t kinematics_decouple_base_motion(vec3_q16_t carrier_body_rates, vec3_q16_t gimbal_angles);

void geo_pointing_calculate(geodetic_coord_t aircraft, quat_q16_t q_aircraft_att, geodetic_coord_t target, q16_t *azimuth_rad, q16_t *elevation_rad, q16_t *slant_range_m);

mat3_q16_t gimbal_forward_kinematics(vec3_q16_t joint_angles);
bool gimbal_inverse_kinematics(quat_q16_t desired_pointing, vec3_q16_t *joint_angles_out);

#endif

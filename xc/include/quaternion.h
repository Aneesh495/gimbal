#ifndef GIMBAL_QUATERNION_H
#define GIMBAL_QUATERNION_H

#include "gimbal_types.h"
#include "vector3.h"
#include "matrix3x3.h"

quat_q16_t quat_identity(void);
quat_q16_t quat_create(q16_t w, q16_t x, q16_t y, q16_t z);
quat_q16_t quat_from_axis_angle(vec3_q16_t axis, q16_t angle_rad);
quat_q16_t quat_from_euler_zyx(q16_t yaw, q16_t pitch, q16_t roll);
void quat_to_euler_zyx(quat_q16_t q, q16_t *yaw, q16_t *pitch, q16_t *roll);
mat3_q16_t quat_to_matrix(quat_q16_t q);
quat_q16_t quat_from_matrix(mat3_q16_t m);
quat_q16_t quat_conjugate(quat_q16_t q);
q16_t quat_norm(quat_q16_t q);
quat_q16_t quat_normalize(quat_q16_t q);
quat_q16_t quat_mul(quat_q16_t a, quat_q16_t b);
vec3_q16_t quat_rotate_vec(quat_q16_t q, vec3_q16_t v);
quat_q16_t quat_integrate_rates(quat_q16_t q, vec3_q16_t omega, q16_t dt_sec);
quat_q16_t quat_slerp(quat_q16_t a, quat_q16_t b, q16_t t);
vec3_q16_t quat_error_vector(quat_q16_t q_target, quat_q16_t q_current);

#endif

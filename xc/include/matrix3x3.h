#ifndef GIMBAL_MATRIX3X3_H
#define GIMBAL_MATRIX3X3_H

#include "gimbal_types.h"
#include "vector3.h"

mat3_q16_t mat3_identity(void);
mat3_q16_t mat3_zero(void);
mat3_q16_t mat3_transpose(mat3_q16_t a);
mat3_q16_t mat3_add(mat3_q16_t a, mat3_q16_t b);
mat3_q16_t mat3_sub(mat3_q16_t a, mat3_q16_t b);
mat3_q16_t mat3_scale(mat3_q16_t a, q16_t s);
mat3_q16_t mat3_mul(mat3_q16_t a, mat3_q16_t b);
vec3_q16_t mat3_mul_vec(mat3_q16_t m, vec3_q16_t v);
q16_t mat3_determinant(mat3_q16_t m);
bool mat3_inverse(mat3_q16_t m, mat3_q16_t *inv_out);
mat3_q16_t mat3_from_euler_zyx(q16_t yaw, q16_t pitch, q16_t roll);
void mat3_to_euler_zyx(mat3_q16_t m, q16_t *yaw, q16_t *pitch, q16_t *roll);
mat3_q16_t mat3_orthonormalize(mat3_q16_t m);

#endif

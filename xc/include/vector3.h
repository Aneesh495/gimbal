#ifndef GIMBAL_VECTOR3_H
#define GIMBAL_VECTOR3_H

#include "gimbal_types.h"
#include "fixed_math.h"

vec3_q16_t vec3_create(q16_t x, q16_t y, q16_t z);
vec3_q16_t vec3_zero(void);
vec3_q16_t vec3_add(vec3_q16_t a, vec3_q16_t b);
vec3_q16_t vec3_sub(vec3_q16_t a, vec3_q16_t b);
vec3_q16_t vec3_scale(vec3_q16_t v, q16_t s);
q16_t vec3_dot(vec3_q16_t a, vec3_q16_t b);
vec3_q16_t vec3_cross(vec3_q16_t a, vec3_q16_t b);
q16_t vec3_mag_squared(vec3_q16_t v);
q16_t vec3_mag(vec3_q16_t v);
vec3_q16_t vec3_normalize(vec3_q16_t v);
vec3_q16_t vec3_lerp(vec3_q16_t a, vec3_q16_t b, q16_t t);
vec3_q16_t vec3_clamp(vec3_q16_t v, q16_t min_val, q16_t max_val);

#endif

#ifndef GIMBAL_FIXED_MATH_H
#define GIMBAL_FIXED_MATH_H

#include "gimbal_types.h"

#define CORDIC_ITERATIONS 16
#define CORDIC_GAIN_Q16   39797 /* 0.607252935 * 65536 */

q16_t q16_from_float(float val);
float q16_to_float(q16_t val);
q16_t q16_add(q16_t a, q16_t b);
q16_t q16_sub(q16_t a, q16_t b);
q16_t q16_mul(q16_t a, q16_t b);
q16_t q16_div(q16_t a, q16_t b);
q16_t q16_clamp(q16_t val, q16_t min_val, q16_t max_val);
q16_t q16_abs(q16_t val);
q16_t q16_sqrt(q16_t val);
void q16_cordic_sin_cos(q16_t theta_rad, q16_t *sin_out, q16_t *cos_out);
q16_t q16_cordic_atan2(q16_t y, q16_t x);

q31_t q31_mul(q31_t a, q31_t b);
q31_t q31_add_sat(q31_t a, q31_t b);
q31_t q31_sub_sat(q31_t a, q31_t b);

#endif

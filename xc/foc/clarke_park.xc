/*
 * Gimbal Forward and Inverse Clarke and Park Transformations
 * Fixed-point Q16.16 transformations between 3-phase, stationary 2-phase, and rotating reference frames.
 */

#include "foc.h"

/* 1/sqrt(3) in Q16 = 0.577350269 * 65536 = 37837 */
#define ONE_OVER_SQRT3_Q16 37837

void foc_clarke_transform(q16_t ia, q16_t ib, q16_t ic, q16_t *i_alpha, q16_t *i_beta) {
    /*
     * Simplified Clarke transform assuming ia + ib + ic = 0:
     * i_alpha = ia
     * i_beta = (ia + 2*ib) / sqrt(3) = (ib - ic) / sqrt(3)
     */
    if (i_alpha) *i_alpha = ia;
    if (i_beta) {
        int64_t diff = (int64_t)ib - (int64_t)ic;
        *i_beta = (q16_t)(((diff * ONE_OVER_SQRT3_Q16) + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    }
}

void foc_park_transform(q16_t i_alpha, q16_t i_beta, q16_t theta_e, q16_t *i_d, q16_t *i_q) {
    /*
     * Rotating reference frame transformation:
     * i_d =  i_alpha * cos(theta_e) + i_beta * sin(theta_e)
     * i_q = -i_alpha * sin(theta_e) + i_beta * cos(theta_e)
     */
    q16_t sin_t, cos_t;
    q16_cordic_sin_cos(theta_e, &sin_t, &cos_t);
    
    if (i_d) {
        int64_t d = (int64_t)i_alpha * cos_t + (int64_t)i_beta * sin_t;
        *i_d = (q16_t)((d + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    }
    if (i_q) {
        int64_t q = -(int64_t)i_alpha * sin_t + (int64_t)i_beta * cos_t;
        *i_q = (q16_t)((q + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    }
}

void foc_inv_park_transform(q16_t v_d, q16_t v_q, q16_t theta_e, q16_t *v_alpha, q16_t *v_beta) {
    /*
     * Inverse Park transformation:
     * v_alpha = v_d * cos(theta_e) - v_q * sin(theta_e)
     * v_beta  = v_d * sin(theta_e) + v_q * cos(theta_e)
     */
    q16_t sin_t, cos_t;
    q16_cordic_sin_cos(theta_e, &sin_t, &cos_t);
    
    if (v_alpha) {
        int64_t a = (int64_t)v_d * cos_t - (int64_t)v_q * sin_t;
        *v_alpha = (q16_t)((a + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    }
    if (v_beta) {
        int64_t b = (int64_t)v_d * sin_t + (int64_t)v_q * cos_t;
        *v_beta = (q16_t)((b + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    }
}

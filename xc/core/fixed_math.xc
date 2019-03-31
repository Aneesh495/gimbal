/*
 * Gimbal Core Fixed-Point Mathematics and CORDIC Engine
 *
 * Implements deterministic Q16.16 and Q31 fixed-point arithmetic routines
 * optimized for multi-core xCORE hardware tiles without floating-point units.
 */

#include "fixed_math.h"

/* Precomputed CORDIC angles: atan(2^-i) in Q16.16 format */
static const q16_t cordic_atan_table[16] = {
    51472, /* atan(1.0) = 0.785398 rad = 51472 in Q16 */
    30386, /* atan(0.5) = 0.463648 rad = 30386 in Q16 */
    16055, /* atan(0.25) = 0.244979 rad */
    8150,  /* atan(0.125) */
    4091,  /* atan(0.0625) */
    2047,  /* atan(0.03125) */
    1024,  /* atan(0.015625) */
    512,
    256,
    128,
    64,
    32,
    16,
    8,
    4,
    2
};

q16_t q16_from_float(float val) {
    return (q16_t)(val * 65536.0f);
}

float q16_to_float(q16_t val) {
    return ((float)val) / 65536.0f;
}

q16_t q16_add(q16_t a, q16_t b) {
    int64_t sum = (int64_t)a + (int64_t)b;
    if (sum > 2147483647LL) return 2147483647;
    if (sum < -2147483648LL) return -2147483648;
    return (q16_t)sum;
}

q16_t q16_sub(q16_t a, q16_t b) {
    int64_t diff = (int64_t)a - (int64_t)b;
    if (diff > 2147483647LL) return 2147483647;
    if (diff < -2147483648LL) return -2147483648;
    return (q16_t)diff;
}

q16_t q16_mul(q16_t a, q16_t b) {
    int64_t prod = ((int64_t)a * (int64_t)b) + FIXED_Q16_HALF;
    return (q16_t)(prod >> FIXED_Q16_SHIFT);
}

q16_t q16_div(q16_t a, q16_t b) {
    if (b == 0) {
        return (a >= 0) ? 2147483647 : -2147483648;
    }
    int64_t num = ((int64_t)a) << FIXED_Q16_SHIFT;
    return (q16_t)(num / b);
}

q16_t q16_clamp(q16_t val, q16_t min_val, q16_t max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

q16_t q16_abs(q16_t val) {
    if (val < 0) {
        if (val == -2147483648) return 2147483647;
        return -val;
    }
    return val;
}

q16_t q16_sqrt(q16_t val) {
    if (val <= 0) return 0;
    
    /* Newton-Raphson square root initial guess */
    uint64_t x = ((uint64_t)val) << FIXED_Q16_SHIFT;
    uint64_t root = 65536;
    
    for (int i = 0; i < 16; i++) {
        uint64_t next_root = (root + (x / root)) >> 1;
        if (next_root == root || next_root == root - 1) {
            break;
        }
        root = next_root;
    }
    return (q16_t)root;
}

void q16_cordic_sin_cos(q16_t theta_rad, q16_t *sin_out, q16_t *cos_out) {
    q16_t pi = 205887;     /* pi in Q16 = 3.14159265 * 65536 */
    q16_t two_pi = 411775; /* 2*pi in Q16 */
    
    /* Normalize theta into [-pi, pi] */
    while (theta_rad > pi) theta_rad -= two_pi;
    while (theta_rad < -pi) theta_rad += two_pi;
    
    q16_t x = CORDIC_GAIN_Q16;
    q16_t y = 0;
    q16_t z = theta_rad;
    
    /* Pre-rotation to handle [-pi, -pi/2] and [pi/2, pi] */
    if (z > (pi >> 1)) {
        z -= pi;
        x = -x;
    } else if (z < -(pi >> 1)) {
        z += pi;
        x = -x;
    }
    
    for (int i = 0; i < CORDIC_ITERATIONS; i++) {
        q16_t x_new, y_new;
        q16_t angle = cordic_atan_table[i];
        
        if (z >= 0) {
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            z -= angle;
        } else {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            z += angle;
        }
        x = x_new;
        y = y_new;
    }
    
    if (cos_out) *cos_out = x;
    if (sin_out) *sin_out = y;
}

q16_t q16_cordic_atan2(q16_t y, q16_t x) {
    if (x == 0 && y == 0) return 0;
    
    q16_t pi = 205887;
    q16_t pi_half = 102944;
    
    if (x == 0) {
        return (y > 0) ? pi_half : -pi_half;
    }
    
    q16_t z = 0;
    q16_t x_curr = x;
    q16_t y_curr = y;
    
    /* Pre-rotation into quadrant 1 or 4 */
    if (x_curr < 0) {
        if (y_curr >= 0) {
            x_curr = -x;
            y_curr = -y;
            z = pi;
        } else {
            x_curr = -x;
            y_curr = -y;
            z = -pi;
        }
    }
    
    for (int i = 0; i < CORDIC_ITERATIONS; i++) {
        q16_t x_new, y_new;
        q16_t angle = cordic_atan_table[i];
        
        if (y_curr > 0) {
            x_new = x_curr + (y_curr >> i);
            y_new = y_curr - (x_curr >> i);
            z += angle;
        } else {
            x_new = x_curr - (y_curr >> i);
            y_new = y_curr + (x_curr >> i);
            z -= angle;
        }
        x_curr = x_new;
        y_curr = y_new;
    }
    
    return z;
}

q31_t q31_mul(q31_t a, q31_t b) {
    int64_t prod = ((int64_t)a * (int64_t)b) >> 31;
    return (q31_t)prod;
}

q31_t q31_add_sat(q31_t a, q31_t b) {
    int64_t sum = (int64_t)a + (int64_t)b;
    if (sum > 0x7FFFFFFFLL) return 0x7FFFFFFF;
    if (sum < -0x80000000LL) return (q31_t)0x80000000LL;
    return (q31_t)sum;
}

q31_t q31_sub_sat(q31_t a, q31_t b) {
    int64_t diff = (int64_t)a - (int64_t)b;
    if (diff > 0x7FFFFFFFLL) return 0x7FFFFFFF;
    if (diff < -0x80000000LL) return (q31_t)0x80000000LL;
    return (q31_t)diff;
}

#include <stdlib.h>
/*
 * Unit Test Suite: Fixed-Point Arithmetic and CORDIC Precision
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "fixed_math.h"

int main(void) {
    printf("Running test_fixed_math... ");
    
    /* 1. Basic arithmetic */
    q16_t a = q16_from_float(3.5f);
    q16_t b = q16_from_float(2.25f);
    q16_t sum = q16_add(a, b);
    assert(fabs(q16_to_float(sum) - 5.75f) < 0.001f);
    
    q16_t diff = q16_sub(a, b);
    assert(fabs(q16_to_float(diff) - 1.25f) < 0.001f);
    
    q16_t prod = q16_mul(a, b);
    assert(fabs(q16_to_float(prod) - 7.875f) < 0.002f);
    
    q16_t quot = q16_div(a, b);
    assert(fabs(q16_to_float(quot) - 1.5555f) < 0.002f);
    
    /* 2. Newton-Raphson sqrt */
    q16_t val = q16_from_float(25.0f);
    q16_t root = q16_sqrt(val);
    assert(fabs(q16_to_float(root) - 5.0f) < 0.005f);
    
    /* 3. CORDIC sin / cos across 16 angles */
    for (int i = 0; i < 16; i++) {
        float rad = (float)i * 0.392699f; /* pi/8 steps */
        q16_t q_rad = q16_from_float(rad);
        q16_t s, c;
        q16_cordic_sin_cos(q_rad, &s, &c);
        assert(fabs(q16_to_float(s) - sinf(rad)) < 0.005f);
        assert(fabs(q16_to_float(c) - cosf(rad)) < 0.005f);
    }
    
    /* 4. CORDIC atan2 */
    q16_t ang = q16_cordic_atan2(q16_from_float(1.0f), q16_from_float(1.0f));
    assert(fabs(q16_to_float(ang) - 0.785398f) < 0.005f);
    
    printf("PASSED\n");
    return 0;
}

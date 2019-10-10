/*
 * Unit Test Suite: 3D Vector Algebra and 3x3 Matrix Routines
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "vector3.h"
#include "matrix3x3.h"

int main(void) {
    printf("Running test_vector_matrix... ");
    
    /* Vector operations */
    vec3_q16_t v1 = vec3_create(FIXED_Q16_ONE, 0, 0);
    vec3_q16_t v2 = vec3_create(0, FIXED_Q16_ONE, 0);
    vec3_q16_t v3 = vec3_cross(v1, v2);
    assert(v3.x == 0 && v3.y == 0 && abs(v3.z - FIXED_Q16_ONE) < 4);
    
    q16_t dot = vec3_dot(v1, v2);
    assert(dot == 0);
    
    q16_t mag = vec3_mag(vec3_create(q16_from_float(3.0f), q16_from_float(4.0f), 0));
    assert(fabs(q16_to_float(mag) - 5.0f) < 0.01f);
    
    /* Matrix operations */
    mat3_q16_t I = mat3_identity();
    mat3_q16_t A = mat3_from_euler_zyx(q16_from_float(0.5f), q16_from_float(0.2f), q16_from_float(-0.3f));
    mat3_q16_t A_inv;
    bool inv_ok = mat3_inverse(A, &A_inv);
    assert(inv_ok);
    
    mat3_q16_t prod = mat3_mul(A, A_inv);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            q16_t expected = (i == j) ? FIXED_Q16_ONE : 0;
            assert(abs(prod.m[i][j] - expected) < 128);
        }
    }
    
    printf("PASSED\n");
    return 0;
}

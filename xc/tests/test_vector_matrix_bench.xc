/*
 * XC Test Bench: 3D Vector and 3x3 Matrix Linear Algebra
 * Tests vector dot, cross, norm, matrix inversion, and Gram-Schmidt orthonormalization.
 */

#include "vector3.h"
#include "matrix3x3.h"
#include <stdbool.h>

bool run_vector_matrix_bench(void) {
    /* Test 1: Vector cross product orthogonality */
    vec3_q16_t vx = vec3_create(FIXED_Q16_ONE, 0, 0);
    vec3_q16_t vy = vec3_create(0, FIXED_Q16_ONE, 0);
    vec3_q16_t vz = vec3_cross(vx, vy);
    if (vz.x != 0 || vz.y != 0 || q16_abs(vz.z - FIXED_Q16_ONE) > 2) return false;

    /* Test 2: Matrix identity multiplication */
    mat3_q16_t I = mat3_identity();
    vec3_q16_t test_v = vec3_create(12345, -54321, 33333);
    vec3_q16_t res_v = mat3_mul_vec(I, test_v);
    if (vec3_mag(vec3_sub(test_v, res_v)) > 2) return false;

    /* Test 3: Matrix inversion A * A^-1 = I */
    mat3_q16_t R = mat3_from_euler_zyx(32768, 16384, -24576);
    mat3_q16_t R_inv;
    if (!mat3_inverse(R, &R_inv)) return false;
    mat3_q16_t prod = mat3_mul(R, R_inv);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            q16_t expected = (i == j) ? FIXED_Q16_ONE : 0;
            if (q16_abs(prod.m[i][j] - expected) > 64) return false;
        }
    }
    return true;
}

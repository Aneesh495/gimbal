/*
 * Gimbal Core 3x3 Matrix Mathematics
 * Rotation matrices, coordinate transformations, and matrix inversions in Q16.16 format.
 */

#include "matrix3x3.h"

mat3_q16_t mat3_identity(void) {
    mat3_q16_t out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.m[i][j] = (i == j) ? FIXED_Q16_ONE : 0;
        }
    }
    return out;
}

mat3_q16_t mat3_zero(void) {
    mat3_q16_t out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.m[i][j] = 0;
        }
    }
    return out;
}

mat3_q16_t mat3_transpose(mat3_q16_t a) {
    mat3_q16_t out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.m[i][j] = a.m[j][i];
        }
    }
    return out;
}

mat3_q16_t mat3_add(mat3_q16_t a, mat3_q16_t b) {
    mat3_q16_t out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.m[i][j] = q16_add(a.m[i][j], b.m[i][j]);
        }
    }
    return out;
}

mat3_q16_t mat3_sub(mat3_q16_t a, mat3_q16_t b) {
    mat3_q16_t out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.m[i][j] = q16_sub(a.m[i][j], b.m[i][j]);
        }
    }
    return out;
}

mat3_q16_t mat3_scale(mat3_q16_t a, q16_t s) {
    mat3_q16_t out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.m[i][j] = q16_mul(a.m[i][j], s);
        }
    }
    return out;
}

mat3_q16_t mat3_mul(mat3_q16_t a, mat3_q16_t b) {
    mat3_q16_t out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int64_t sum = 0;
            for (int k = 0; k < 3; k++) {
                sum += (int64_t)a.m[i][k] * (int64_t)b.m[k][j];
            }
            out.m[i][j] = (q16_t)((sum + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
        }
    }
    return out;
}

vec3_q16_t mat3_mul_vec(mat3_q16_t m, vec3_q16_t v) {
    vec3_q16_t out;
    int64_t rx = (int64_t)m.m[0][0]*v.x + (int64_t)m.m[0][1]*v.y + (int64_t)m.m[0][2]*v.z;
    int64_t ry = (int64_t)m.m[1][0]*v.x + (int64_t)m.m[1][1]*v.y + (int64_t)m.m[1][2]*v.z;
    int64_t rz = (int64_t)m.m[2][0]*v.x + (int64_t)m.m[2][1]*v.y + (int64_t)m.m[2][2]*v.z;
    out.x = (q16_t)((rx + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    out.y = (q16_t)((ry + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    out.z = (q16_t)((rz + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    return out;
}

q16_t mat3_determinant(mat3_q16_t m) {
    int64_t t0 = (int64_t)m.m[1][1] * m.m[2][2] - (int64_t)m.m[1][2] * m.m[2][1];
    int64_t t1 = (int64_t)m.m[1][0] * m.m[2][2] - (int64_t)m.m[1][2] * m.m[2][0];
    int64_t t2 = (int64_t)m.m[1][0] * m.m[2][1] - (int64_t)m.m[1][1] * m.m[2][0];
    
    int64_t det = (int64_t)m.m[0][0] * ((t0 + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT)
                - (int64_t)m.m[0][1] * ((t1 + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT)
                + (int64_t)m.m[0][2] * ((t2 + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
                
    return (q16_t)((det + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
}

bool mat3_inverse(mat3_q16_t m, mat3_q16_t *inv_out) {
    q16_t det = mat3_determinant(m);
    if (q16_abs(det) < 16) {
        return false; /* Singular or near-singular matrix */
    }
    
    mat3_q16_t adj;
    adj.m[0][0] = q16_mul(m.m[1][1], m.m[2][2]) - q16_mul(m.m[1][2], m.m[2][1]);
    adj.m[0][1] = q16_mul(m.m[0][2], m.m[2][1]) - q16_mul(m.m[0][1], m.m[2][2]);
    adj.m[0][2] = q16_mul(m.m[0][1], m.m[1][2]) - q16_mul(m.m[0][2], m.m[1][1]);
    
    adj.m[1][0] = q16_mul(m.m[1][2], m.m[2][0]) - q16_mul(m.m[1][0], m.m[2][2]);
    adj.m[1][1] = q16_mul(m.m[0][0], m.m[2][2]) - q16_mul(m.m[0][2], m.m[2][0]);
    adj.m[1][2] = q16_mul(m.m[0][2], m.m[1][0]) - q16_mul(m.m[0][0], m.m[1][2]);
    
    adj.m[2][0] = q16_mul(m.m[1][0], m.m[2][1]) - q16_mul(m.m[1][1], m.m[2][0]);
    adj.m[2][1] = q16_mul(m.m[0][1], m.m[2][0]) - q16_mul(m.m[0][0], m.m[2][1]);
    adj.m[2][2] = q16_mul(m.m[0][0], m.m[1][1]) - q16_mul(m.m[0][1], m.m[1][0]);
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inv_out->m[i][j] = q16_div(adj.m[i][j], det);
        }
    }
    return true;
}

mat3_q16_t mat3_from_euler_zyx(q16_t yaw, q16_t pitch, q16_t roll) {
    q16_t cy, sy, cp, sp, cr, sr;
    q16_cordic_sin_cos(yaw, &sy, &cy);
    q16_cordic_sin_cos(pitch, &sp, &cp);
    q16_cordic_sin_cos(roll, &sr, &cr);
    
    mat3_q16_t out;
    out.m[0][0] = q16_mul(cy, cp);
    out.m[0][1] = q16_mul(cy, q16_mul(sp, sr)) - q16_mul(sy, cr);
    out.m[0][2] = q16_mul(cy, q16_mul(sp, cr)) + q16_mul(sy, sr);
    
    out.m[1][0] = q16_mul(sy, cp);
    out.m[1][1] = q16_mul(sy, q16_mul(sp, sr)) + q16_mul(cy, cr);
    out.m[1][2] = q16_mul(sy, q16_mul(sp, cr)) - q16_mul(cy, sr);
    
    out.m[2][0] = -sp;
    out.m[2][1] = q16_mul(cp, sr);
    out.m[2][2] = q16_mul(cp, cr);
    
    return out;
}

void mat3_to_euler_zyx(mat3_q16_t m, q16_t *yaw, q16_t *pitch, q16_t *roll) {
    q16_t sp = -m.m[2][0];
    sp = q16_clamp(sp, -FIXED_Q16_ONE, FIXED_Q16_ONE);
    
    q16_t p = q16_cordic_atan2(sp, q16_sqrt(FIXED_Q16_ONE - q16_mul(sp, sp)));
    q16_t y, r;
    
    if (q16_abs(m.m[2][0]) < 64880) { /* Not near +/- 90 degrees */
        y = q16_cordic_atan2(m.m[1][0], m.m[0][0]);
        r = q16_cordic_atan2(m.m[2][1], m.m[2][2]);
    } else {
        /* Gimbal lock singularity fallback */
        y = 0;
        r = q16_cordic_atan2(-m.m[0][1], m.m[1][1]);
    }
    
    if (yaw) *yaw = y;
    if (pitch) *pitch = p;
    if (roll) *roll = r;
}

mat3_q16_t mat3_orthonormalize(mat3_q16_t m) {
    vec3_q16_t x = vec3_create(m.m[0][0], m.m[1][0], m.m[2][0]);
    vec3_q16_t y = vec3_create(m.m[0][1], m.m[1][1], m.m[2][1]);
    
    x = vec3_normalize(x);
    q16_t proj = vec3_dot(x, y);
    y = vec3_sub(y, vec3_scale(x, proj));
    y = vec3_normalize(y);
    vec3_q16_t z = vec3_cross(x, y);
    
    mat3_q16_t out;
    out.m[0][0] = x.x; out.m[0][1] = y.x; out.m[0][2] = z.x;
    out.m[1][0] = x.y; out.m[1][1] = y.y; out.m[1][2] = z.y;
    out.m[2][0] = x.z; out.m[2][1] = y.z; out.m[2][2] = z.z;
    return out;
}

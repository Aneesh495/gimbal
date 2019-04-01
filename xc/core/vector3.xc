/*
 * Gimbal Core 3D Fixed-Point Vector Library
 * Deterministic vector operations for 3-axis kinematics and coordinate transformations.
 */

#include "vector3.h"

vec3_q16_t vec3_create(q16_t x, q16_t y, q16_t z) {
    vec3_q16_t out;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

vec3_q16_t vec3_zero(void) {
    vec3_q16_t out = {0, 0, 0};
    return out;
}

vec3_q16_t vec3_add(vec3_q16_t a, vec3_q16_t b) {
    vec3_q16_t out;
    out.x = q16_add(a.x, b.x);
    out.y = q16_add(a.y, b.y);
    out.z = q16_add(a.z, b.z);
    return out;
}

vec3_q16_t vec3_sub(vec3_q16_t a, vec3_q16_t b) {
    vec3_q16_t out;
    out.x = q16_sub(a.x, b.x);
    out.y = q16_sub(a.y, b.y);
    out.z = q16_sub(a.z, b.z);
    return out;
}

vec3_q16_t vec3_scale(vec3_q16_t v, q16_t s) {
    vec3_q16_t out;
    out.x = q16_mul(v.x, s);
    out.y = q16_mul(v.y, s);
    out.z = q16_mul(v.z, s);
    return out;
}

q16_t vec3_dot(vec3_q16_t a, vec3_q16_t b) {
    int64_t sum = ((int64_t)a.x * (int64_t)b.x) +
                  ((int64_t)a.y * (int64_t)b.y) +
                  ((int64_t)a.z * (int64_t)b.z);
    return (q16_t)((sum + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
}

vec3_q16_t vec3_cross(vec3_q16_t a, vec3_q16_t b) {
    vec3_q16_t out;
    int64_t cx = ((int64_t)a.y * (int64_t)b.z) - ((int64_t)a.z * (int64_t)b.y);
    int64_t cy = ((int64_t)a.z * (int64_t)b.x) - ((int64_t)a.x * (int64_t)b.z);
    int64_t cz = ((int64_t)a.x * (int64_t)b.y) - ((int64_t)a.y * (int64_t)b.x);
    out.x = (q16_t)((cx + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    out.y = (q16_t)((cy + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    out.z = (q16_t)((cz + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    return out;
}

q16_t vec3_mag_squared(vec3_q16_t v) {
    return vec3_dot(v, v);
}

q16_t vec3_mag(vec3_q16_t v) {
    q16_t mag_sq = vec3_mag_squared(v);
    return q16_sqrt(mag_sq);
}

vec3_q16_t vec3_normalize(vec3_q16_t v) {
    q16_t m = vec3_mag(v);
    if (m <= 1) {
        return vec3_zero();
    }
    vec3_q16_t out;
    out.x = q16_div(v.x, m);
    out.y = q16_div(v.y, m);
    out.z = q16_div(v.z, m);
    return out;
}

vec3_q16_t vec3_lerp(vec3_q16_t a, vec3_q16_t b, q16_t t) {
    vec3_q16_t diff = vec3_sub(b, a);
    vec3_q16_t scaled = vec3_scale(diff, t);
    return vec3_add(a, scaled);
}

vec3_q16_t vec3_clamp(vec3_q16_t v, q16_t min_val, q16_t max_val) {
    vec3_q16_t out;
    out.x = q16_clamp(v.x, min_val, max_val);
    out.y = q16_clamp(v.y, min_val, max_val);
    out.z = q16_clamp(v.z, min_val, max_val);
    return out;
}

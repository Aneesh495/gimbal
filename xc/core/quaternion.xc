/*
 * Gimbal Core SO(3) Unit Quaternion Kinematics
 * Singularity-free 3D attitude representations, angular velocity integration,
 * and SLERP spherical linear interpolation.
 */

#include "quaternion.h"

quat_q16_t quat_identity(void) {
    quat_q16_t q;
    q.w = FIXED_Q16_ONE;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    return q;
}

quat_q16_t quat_create(q16_t w, q16_t x, q16_t y, q16_t z) {
    quat_q16_t q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}

quat_q16_t quat_from_axis_angle(vec3_q16_t axis, q16_t angle_rad) {
    vec3_q16_t u = vec3_normalize(axis);
    q16_t half_angle = angle_rad >> 1;
    q16_t s, c;
    q16_cordic_sin_cos(half_angle, &s, &c);
    
    quat_q16_t q;
    q.w = c;
    q.x = q16_mul(u.x, s);
    q.y = q16_mul(u.y, s);
    q.z = q16_mul(u.z, s);
    return quat_normalize(q);
}

quat_q16_t quat_from_euler_zyx(q16_t yaw, q16_t pitch, q16_t roll) {
    q16_t cy, sy, cp, sp, cr, sr;
    q16_cordic_sin_cos(yaw >> 1, &sy, &cy);
    q16_cordic_sin_cos(pitch >> 1, &sp, &cp);
    q16_cordic_sin_cos(roll >> 1, &sr, &cr);
    
    quat_q16_t q;
    q.w = q16_mul(cr, q16_mul(cp, cy)) + q16_mul(sr, q16_mul(sp, sy));
    q.x = q16_mul(sr, q16_mul(cp, cy)) - q16_mul(cr, q16_mul(sp, sy));
    q.y = q16_mul(cr, q16_mul(sp, cy)) + q16_mul(sr, q16_mul(cp, sy));
    q.z = q16_mul(cr, q16_mul(cp, sy)) - q16_mul(sr, q16_mul(sp, cy));
    return quat_normalize(q);
}

void quat_to_euler_zyx(quat_q16_t q, q16_t *yaw, q16_t *pitch, q16_t *roll) {
    /* Roll (x-axis rotation) */
    int64_t sinr_cosp = 2 * ((int64_t)q.w * q.x + (int64_t)q.y * q.z);
    int64_t cosr_cosp = (int64_t)FIXED_Q16_ONE * FIXED_Q16_ONE - 2 * ((int64_t)q.x * q.x + (int64_t)q.y * q.y);
    q16_t r = q16_cordic_atan2((q16_t)(sinr_cosp >> FIXED_Q16_SHIFT), (q16_t)(cosr_cosp >> FIXED_Q16_SHIFT));
    
    /* Pitch (y-axis rotation) */
    int64_t sinp = 2 * ((int64_t)q.w * q.y - (int64_t)q.z * q.x);
    q16_t sinp_q16 = (q16_t)(sinp >> FIXED_Q16_SHIFT);
    q16_t p;
    if (q16_abs(sinp_q16) >= FIXED_Q16_ONE) {
        p = (sinp_q16 > 0) ? 102944 : -102944; /* +/- pi/2 */
    } else {
        p = q16_cordic_atan2(sinp_q16, q16_sqrt(FIXED_Q16_ONE - q16_mul(sinp_q16, sinp_q16)));
    }
    
    /* Yaw (z-axis rotation) */
    int64_t siny_cosp = 2 * ((int64_t)q.w * q.z + (int64_t)q.x * q.y);
    int64_t cosy_cosp = (int64_t)FIXED_Q16_ONE * FIXED_Q16_ONE - 2 * ((int64_t)q.y * q.y + (int64_t)q.z * q.z);
    q16_t y = q16_cordic_atan2((q16_t)(siny_cosp >> FIXED_Q16_SHIFT), (q16_t)(cosy_cosp >> FIXED_Q16_SHIFT));
    
    if (yaw) *yaw = y;
    if (pitch) *pitch = p;
    if (roll) *roll = r;
}

mat3_q16_t quat_to_matrix(quat_q16_t q) {
    mat3_q16_t m;
    q16_t qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    q16_t x2 = qx + qx;
    q16_t y2 = qy + qy;
    q16_t z2 = qz + qz;
    
    q16_t xx = q16_mul(qx, x2);
    q16_t xy = q16_mul(qx, y2);
    q16_t xz = q16_mul(qx, z2);
    q16_t yy = q16_mul(qy, y2);
    q16_t yz = q16_mul(qy, z2);
    q16_t zz = q16_mul(qz, z2);
    q16_t wx = q16_mul(qw, x2);
    q16_t wy = q16_mul(qw, y2);
    q16_t wz = q16_mul(qw, z2);
    
    m.m[0][0] = FIXED_Q16_ONE - (yy + zz);
    m.m[0][1] = xy - wz;
    m.m[0][2] = xz + wy;
    
    m.m[1][0] = xy + wz;
    m.m[1][1] = FIXED_Q16_ONE - (xx + zz);
    m.m[1][2] = yz - wx;
    
    m.m[2][0] = xz - wy;
    m.m[2][1] = yz + wx;
    m.m[2][2] = FIXED_Q16_ONE - (xx + yy);
    
    return m;
}

quat_q16_t quat_conjugate(quat_q16_t q) {
    quat_q16_t out;
    out.w = q.w;
    out.x = -q.x;
    out.y = -q.y;
    out.z = -q.z;
    return out;
}

q16_t quat_norm(quat_q16_t q) {
    int64_t sum = ((int64_t)q.w * q.w) +
                  ((int64_t)q.x * q.x) +
                  ((int64_t)q.y * q.y) +
                  ((int64_t)q.z * q.z);
    q16_t norm_sq = (q16_t)((sum + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    return q16_sqrt(norm_sq);
}

quat_q16_t quat_normalize(quat_q16_t q) {
    q16_t n = quat_norm(q);
    if (n <= 1) {
        return quat_identity();
    }
    quat_q16_t out;
    out.w = q16_div(q.w, n);
    out.x = q16_div(q.x, n);
    out.y = q16_div(q.y, n);
    out.z = q16_div(q.z, n);
    return out;
}

quat_q16_t quat_mul(quat_q16_t a, quat_q16_t b) {
    quat_q16_t out;
    int64_t w = (int64_t)a.w*b.w - (int64_t)a.x*b.x - (int64_t)a.y*b.y - (int64_t)a.z*b.z;
    int64_t x = (int64_t)a.w*b.x + (int64_t)a.x*b.w + (int64_t)a.y*b.z - (int64_t)a.z*b.y;
    int64_t y = (int64_t)a.w*b.y - (int64_t)a.x*b.z + (int64_t)a.y*b.w + (int64_t)a.z*b.x;
    int64_t z = (int64_t)a.w*b.z + (int64_t)a.x*b.y - (int64_t)a.y*b.x + (int64_t)a.z*b.w;
    
    out.w = (q16_t)((w + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    out.x = (q16_t)((x + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    out.y = (q16_t)((y + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    out.z = (q16_t)((z + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
    return quat_normalize(out);
}

vec3_q16_t quat_rotate_vec(quat_q16_t q, vec3_q16_t v) {
    /* v_rot = q * (0, v) * q^-1 */
    quat_q16_t p = quat_create(0, v.x, v.y, v.z);
    quat_q16_t q_inv = quat_conjugate(q);
    quat_q16_t rotated = quat_mul(quat_mul(q, p), q_inv);
    
    vec3_q16_t out;
    out.x = rotated.x;
    out.y = rotated.y;
    out.z = rotated.z;
    return out;
}

quat_q16_t quat_integrate_rates(quat_q16_t q, vec3_q16_t omega, q16_t dt_sec) {
    /* q_next = q + 0.5 * q * omega * dt */
    q16_t half_dt = dt_sec >> 1;
    vec3_q16_t delta_angle = vec3_scale(omega, half_dt);
    
    quat_q16_t dq;
    dq.w = 0;
    dq.x = delta_angle.x;
    dq.y = delta_angle.y;
    dq.z = delta_angle.z;
    
    quat_q16_t q_dot = quat_mul(q, dq);
    
    quat_q16_t next;
    next.w = q16_add(q.w, q_dot.w);
    next.x = q16_add(q.x, q_dot.x);
    next.y = q16_add(q.y, q_dot.y);
    next.z = q16_add(q.z, q_dot.z);
    return quat_normalize(next);
}

quat_q16_t quat_slerp(quat_q16_t a, quat_q16_t b, q16_t t) {
    q16_t cos_half_theta = q16_mul(a.w, b.w) + q16_mul(a.x, b.x) + q16_mul(a.y, b.y) + q16_mul(a.z, b.z);
    
    quat_q16_t b_target = b;
    if (cos_half_theta < 0) {
        b_target.w = -b.w;
        b_target.x = -b.x;
        b_target.y = -b.y;
        b_target.z = -b.z;
        cos_half_theta = -cos_half_theta;
    }
    
    if (cos_half_theta >= 64880) { /* > 0.99 in Q16: Linear interpolation */
        quat_q16_t out;
        q16_t one_minus_t = FIXED_Q16_ONE - t;
        out.w = q16_mul(a.w, one_minus_t) + q16_mul(b_target.w, t);
        out.x = q16_mul(a.x, one_minus_t) + q16_mul(b_target.x, t);
        out.y = q16_mul(a.y, one_minus_t) + q16_mul(b_target.y, t);
        out.z = q16_mul(a.z, one_minus_t) + q16_mul(b_target.z, t);
        return quat_normalize(out);
    }
    
    q16_t half_theta = q16_cordic_atan2(q16_sqrt(FIXED_Q16_ONE - q16_mul(cos_half_theta, cos_half_theta)), cos_half_theta);
    q16_t sin_half_theta;
    q16_cordic_sin_cos(half_theta, &sin_half_theta, NULL);
    
    q16_t sin_a, sin_b;
    q16_cordic_sin_cos(q16_mul(FIXED_Q16_ONE - t, half_theta), &sin_a, NULL);
    q16_cordic_sin_cos(q16_mul(t, half_theta), &sin_b, NULL);
    
    q16_t ratio_a = q16_div(sin_a, sin_half_theta);
    q16_t ratio_b = q16_div(sin_b, sin_half_theta);
    
    quat_q16_t out;
    out.w = q16_mul(a.w, ratio_a) + q16_mul(b_target.w, ratio_b);
    out.x = q16_mul(a.x, ratio_a) + q16_mul(b_target.x, ratio_b);
    out.y = q16_mul(a.y, ratio_a) + q16_mul(b_target.y, ratio_b);
    out.z = q16_mul(a.z, ratio_a) + q16_mul(b_target.z, ratio_b);
    return quat_normalize(out);
}

vec3_q16_t quat_error_vector(quat_q16_t q_target, quat_q16_t q_current) {
    /* delta_q = q_target * q_current^-1 */
    quat_q16_t q_err = quat_mul(q_target, quat_conjugate(q_current));
    if (q_err.w < 0) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }
    /* Small angle approximation: delta_theta = 2 * (x, y, z) */
    vec3_q16_t out;
    out.x = q_err.x << 1;
    out.y = q_err.y << 1;
    out.z = q_err.z << 1;
    return out;
}

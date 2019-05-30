/*
 * Gimbal Jerk-Limited S-Curve Motion Profile Generator
 * 7-phase trajectory profiler producing continuous position, velocity, and acceleration.
 */

#include "kinematics.h"

void scurve_init(scurve_profile_t *prof, q16_t max_v, q16_t max_a, q16_t max_j) {
    if (!prof) return;
    prof->pos_target = 0;
    prof->pos_current = 0;
    prof->vel_current = 0;
    prof->acc_current = 0;
    prof->max_vel = max_v;
    prof->max_acc = max_a;
    prof->max_jerk = max_j;
    prof->active = false;
}

void scurve_set_target(scurve_profile_t *prof, q16_t target_pos) {
    if (!prof) return;
    prof->pos_target = target_pos;
    prof->active = true;
}

void scurve_step(scurve_profile_t *prof, q16_t dt_sec) {
    if (!prof || !prof->active) return;
    
    q16_t error = prof->pos_target - prof->pos_current;
    q16_t abs_err = q16_abs(error);
    
    if (abs_err < 10 && q16_abs(prof->vel_current) < 10) {
        prof->pos_current = prof->pos_target;
        prof->vel_current = 0;
        prof->acc_current = 0;
        prof->active = false;
        return;
    }
    
    /* Stopping distance: d = v^2 / (2 * a_max) */
    int64_t v_sq = ((int64_t)prof->vel_current * prof->vel_current) >> FIXED_Q16_SHIFT;
    q16_t stop_dist = (q16_t)(v_sq / ((prof->max_acc << 1) >> FIXED_Q16_SHIFT));
    
    q16_t desired_acc = 0;
    if (abs_err > stop_dist) {
        /* Accelerate towards max velocity */
        q16_t desired_vel = (error > 0) ? prof->max_vel : -prof->max_vel;
        q16_t vel_diff = desired_vel - prof->vel_current;
        desired_acc = (vel_diff > 0) ? prof->max_acc : -prof->max_acc;
    } else {
        /* Decelerate to zero at target */
        desired_acc = (prof->vel_current > 0) ? -prof->max_acc : prof->max_acc;
    }
    
    /* Jerk-limited acceleration update */
    q16_t acc_diff = desired_acc - prof->acc_current;
    q16_t max_jerk_step = q16_mul(prof->max_jerk, dt_sec);
    acc_diff = q16_clamp(acc_diff, -max_jerk_step, max_jerk_step);
    prof->acc_current = q16_add(prof->acc_current, acc_diff);
    
    /* Velocity integration */
    prof->vel_current = q16_add(prof->vel_current, q16_mul(prof->acc_current, dt_sec));
    prof->vel_current = q16_clamp(prof->vel_current, -prof->max_vel, prof->max_vel);
    
    /* Position integration */
    prof->pos_current = q16_add(prof->pos_current, q16_mul(prof->vel_current, dt_sec));
}

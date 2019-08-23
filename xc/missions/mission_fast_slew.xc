/*
 * Gimbal Autonomous Mission Profile: Agile Multi-Target Rapid Slew
 * Stress-tests jerk-limited S-curve trajectory profiling and flux weakening under high slew accelerations.
 */

#include "kinematics.h"

typedef struct {
    vec3_q16_t target_waypoints[4];
    uint8_t current_wp_idx;
    scurve_profile_t profiles[GIMBAL_AXIS_COUNT];
    uint32_t dwell_ticks;
} mission_fast_slew_t;

void mission_fast_slew_init(mission_fast_slew_t *fs) {
    if (!fs) return;
    
    /* 4 target waypoints in Pan, Tilt, Roll (rad) */
    fs->target_waypoints[0] = vec3_create(q16_from_float(1.2f), q16_from_float(0.4f), 0);
    fs->target_waypoints[1] = vec3_create(q16_from_float(-1.0f), q16_from_float(-0.3f), 0);
    fs->target_waypoints[2] = vec3_create(q16_from_float(0.8f), q16_from_float(-0.5f), 0);
    fs->target_waypoints[3] = vec3_create(0, 0, 0);
    
    fs->current_wp_idx = 0;
    fs->dwell_ticks = 0;
    
    q16_t max_v = q16_from_float(3.5f);  /* 200 deg/s high-speed slew */
    q16_t max_a = q16_from_float(15.0f); /* 860 deg/s^2 */
    q16_t max_j = q16_from_float(80.0f);
    
    for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
        scurve_init(&fs->profiles[i], max_v, max_a, max_j);
    }
    
    scurve_set_target(&fs->profiles[0], fs->target_waypoints[0].x);
    scurve_set_target(&fs->profiles[1], fs->target_waypoints[0].y);
    scurve_set_target(&fs->profiles[2], fs->target_waypoints[0].z);
}

void mission_fast_slew_step(mission_fast_slew_t *fs, q16_t dt_sec) {
    if (!fs) return;
    
    bool all_settled = true;
    for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
        scurve_step(&fs->profiles[i], dt_sec);
        if (fs->profiles[i].active) all_settled = false;
    }
    
    if (all_settled) {
        fs->dwell_ticks++;
        if (fs->dwell_ticks > 50) { /* Dwell 500 ms at target */
            fs->dwell_ticks = 0;
            fs->current_wp_idx = (fs->current_wp_idx + 1) % 4;
            vec3_q16_t next_wp = fs->target_waypoints[fs->current_wp_idx];
            scurve_set_target(&fs->profiles[0], next_wp.x);
            scurve_set_target(&fs->profiles[1], next_wp.y);
            scurve_set_target(&fs->profiles[2], next_wp.z);
        }
    }
}

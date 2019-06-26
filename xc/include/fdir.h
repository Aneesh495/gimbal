#ifndef GIMBAL_FDIR_H
#define GIMBAL_FDIR_H

#include "gimbal_types.h"

typedef struct {
    uint32_t last_heartbeat_tick;
    uint32_t timeout_threshold_ticks;
    uint32_t missed_heartbeat_count;
    bool trip_alarm;
} watchdog_channel_t;

typedef struct {
    q16_t winding_temp_c;
    q16_t fet_temp_c;
    q16_t r_th_winding; /* Thermal resistance in Q16 */
    q16_t c_th_winding; /* Thermal capacitance in Q16 */
    q16_t ambient_temp_c;
    q16_t max_winding_temp_c;
} thermal_model_t;

typedef struct {
    q16_t min_angle_rad[GIMBAL_AXIS_COUNT];
    q16_t max_angle_rad[GIMBAL_AXIS_COUNT];
    q16_t max_speed_rad_s[GIMBAL_AXIS_COUNT];
    q16_t max_current_amps[GIMBAL_AXIS_COUNT];
    bool limit_tripped[GIMBAL_AXIS_COUNT];
} limit_checker_t;

typedef struct {
    gimbal_status_flags_t active_faults;
    gimbal_mode_t previous_mode;
    uint32_t fault_latch_tick;
    bool emergency_stow_requested;
} safety_manager_t;

void watchdog_init(watchdog_channel_t *wd, uint32_t timeout_ticks);
void watchdog_feed(watchdog_channel_t *wd, uint32_t current_tick);
bool watchdog_check(watchdog_channel_t *wd, uint32_t current_tick);

void thermal_model_init(thermal_model_t *tm);
void thermal_model_update(thermal_model_t *tm, q16_t phase_current_amps, q16_t dt_sec);

void limit_checker_init(limit_checker_t *lc);
bool limit_checker_verify(limit_checker_t *lc, const vec3_q16_t *angles, const vec3_q16_t *velocities, const q16_t currents[3]);

void safety_manager_init(safety_manager_t *sm);
void safety_manager_update(safety_manager_t *sm, gimbal_status_flags_t new_faults, gimbal_mode_t *active_mode);

#endif

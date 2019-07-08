/*
 * Gimbal Autonomous Safety Manager and FDIR State Machine
 * Transitions between normal operation, degraded modes, motor power cut, and automated stow.
 */

#include "fdir.h"

void safety_manager_init(safety_manager_t *sm) {
    if (!sm) return;
    sm->active_faults = GIMBAL_STATUS_OK;
    sm->previous_mode = GIMBAL_MODE_OFF;
    sm->fault_latch_tick = 0;
    sm->emergency_stow_requested = false;
}

void safety_manager_update(safety_manager_t *sm, gimbal_status_flags_t new_faults, gimbal_mode_t *active_mode) {
    if (!sm || !active_mode) return;
    
    sm->active_faults = (gimbal_status_flags_t)(sm->active_faults | new_faults);
    
    /* Critical faults require immediate transition to FAULT / RETRACT mode */
    if (sm->active_faults & (GIMBAL_STATUS_OVERCURRENT | GIMBAL_STATUS_OVERTEMP | GIMBAL_STATUS_WATCHDOG_TRIP)) {
        if (*active_mode != GIMBAL_MODE_FAULT) {
            sm->previous_mode = *active_mode;
            *active_mode = GIMBAL_MODE_FAULT;
        }
    } else if (sm->active_faults & (GIMBAL_STATUS_IMU_FAULT | GIMBAL_STATUS_ENC_FAULT)) {
        /* Sensor degradation: fallback to rate hold or direct damping */
        if (*active_mode == GIMBAL_MODE_GEO_POINT || *active_mode == GIMBAL_MODE_TARGET_TRACK) {
            *active_mode = GIMBAL_MODE_RATE_HOLD;
        }
    }
}

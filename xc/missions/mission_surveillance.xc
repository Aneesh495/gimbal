/*
 * Gimbal Autonomous Mission Profile: Wide-Area Optical Surveillance and Target Handoff
 * Sequences wide-area rosette search pattern, detects target centroid, locks line-of-sight,
 * and maintains continuous optical tracking.
 */

#include "tracking.h"
#include "kinematics.h"
#include "fdir.h"

typedef enum {
    SURV_STATE_INIT = 0,
    SURV_STATE_SEARCH = 1,
    SURV_STATE_ACQUIRE = 2,
    SURV_STATE_TRACK = 3,
    SURV_STATE_COAST = 4
} surv_mission_state_t;

typedef struct {
    surv_mission_state_t state;
    search_generator_t searcher;
    centroid_tracker_t tracker;
    uint32_t track_duration_ticks;
} mission_surveillance_t;

void mission_surveillance_init(mission_surveillance_t *ms) {
    if (!ms) return;
    ms->state = SURV_STATE_SEARCH;
    search_generator_init(&ms->searcher, SEARCH_PATTERN_ROSETTE, q16_from_float(0.4f), q16_from_float(0.4f), q16_from_float(0.3f));
    tracker_init(&ms->tracker);
    ms->track_duration_ticks = 0;
}

vec3_q16_t mission_surveillance_step(mission_surveillance_t *ms, bool target_detected, q16_t det_x, q16_t det_y, q16_t dt_sec) {
    if (!ms) return vec3_zero();
    
    tracker_update(&ms->tracker, target_detected, det_x, det_y, dt_sec);
    
    switch (ms->state) {
        case SURV_STATE_SEARCH:
            if (target_detected) {
                ms->state = SURV_STATE_TRACK;
                return los_controller_compute_rates(&ms->tracker, q16_from_float(0.1f));
            }
            return search_generator_step(&ms->searcher, dt_sec);
            
        case SURV_STATE_TRACK:
            if (!ms->tracker.target_locked) {
                ms->state = SURV_STATE_SEARCH;
                return search_generator_step(&ms->searcher, dt_sec);
            }
            ms->track_duration_ticks++;
            return los_controller_compute_rates(&ms->tracker, q16_from_float(0.1f));
            
        default:
            return vec3_zero();
    }
}

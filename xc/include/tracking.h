#ifndef GIMBAL_TRACKING_H
#define GIMBAL_TRACKING_H

#include "gimbal_types.h"
#include "vector3.h"

typedef struct {
    q16_t x_pos_norm; /* Normalized image coordinates [-1.0, 1.0] */
    q16_t y_pos_norm;
    q16_t x_vel_norm_s;
    q16_t y_vel_norm_s;
    q16_t alpha;
    q16_t beta;
    uint32_t last_detection_tick;
    uint32_t missed_frames;
    bool target_locked;
} centroid_tracker_t;

typedef enum {
    SEARCH_PATTERN_RASTER = 0,
    SEARCH_PATTERN_SECTOR = 1,
    SEARCH_PATTERN_ROSETTE= 2
} search_pattern_type_t;

typedef struct {
    search_pattern_type_t pattern_type;
    q16_t scan_width_rad;
    q16_t scan_height_rad;
    q16_t scan_rate_rad_s;
    q16_t time_accum_sec;
    vec3_q16_t pattern_offset_rad;
} search_generator_t;

void tracker_init(centroid_tracker_t *trk);
void tracker_update(centroid_tracker_t *trk, bool detection_valid, q16_t meas_x, q16_t meas_y, q16_t dt_sec);

vec3_q16_t los_controller_compute_rates(const centroid_tracker_t *trk, q16_t fov_diag_rad);

void search_generator_init(search_generator_t *gen, search_pattern_type_t type, q16_t width, q16_t height, q16_t rate);
vec3_q16_t search_generator_step(search_generator_t *gen, q16_t dt_sec);

#endif

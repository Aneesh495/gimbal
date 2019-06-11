/*
 * Gimbal Line-of-Sight Rate Stabilization Controller
 * Drives gimbal slew rates to null optical centroid tracking errors.
 */

#include "tracking.h"

vec3_q16_t los_controller_compute_rates(const centroid_tracker_t *trk, q16_t fov_diag_rad) {
    if (!trk || !trk->target_locked) {
        return vec3_zero();
    }
    
    /* Control gain: proportional rate command per normalized pixel offset */
    q16_t k_p = q16_from_float(2.5f);
    q16_t k_d = q16_from_float(0.4f);
    
    /* Rate commands in optical frame: Pan (azimuth) = +X pixel, Tilt (elevation) = -Y pixel */
    q16_t cmd_pan = q16_mul(k_p, trk->x_pos_norm) + q16_mul(k_d, trk->x_vel_norm_s);
    q16_t cmd_tilt = -(q16_mul(k_p, trk->y_pos_norm) + q16_mul(k_d, trk->y_vel_norm_s));
    
    vec3_q16_t rates;
    rates.x = cmd_pan;
    rates.y = cmd_tilt;
    rates.z = 0;
    return rates;
}

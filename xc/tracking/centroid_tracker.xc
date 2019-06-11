/*
 * Gimbal Optical Target Centroid Filter
 * Alpha-Beta tracker with target loss coasting and occlusion prediction.
 */

#include "tracking.h"

void tracker_init(centroid_tracker_t *trk) {
    if (!trk) return;
    trk->x_pos_norm = 0;
    trk->y_pos_norm = 0;
    trk->x_vel_norm_s = 0;
    trk->y_vel_norm_s = 0;
    trk->alpha = q16_from_float(0.6f);
    trk->beta  = q16_from_float(0.15f);
    trk->last_detection_tick = 0;
    trk->missed_frames = 0;
    trk->target_locked = false;
}

void tracker_update(centroid_tracker_t *trk, bool detection_valid, q16_t meas_x, q16_t meas_y, q16_t dt_sec) {
    if (!trk) return;
    
    if (detection_valid) {
        trk->missed_frames = 0;
        trk->target_locked = true;
        
        /* 1. Prediction error (residual) */
        q16_t pred_x = q16_add(trk->x_pos_norm, q16_mul(trk->x_vel_norm_s, dt_sec));
        q16_t pred_y = q16_add(trk->y_pos_norm, q16_mul(trk->y_vel_norm_s, dt_sec));
        
        q16_t res_x = q16_sub(meas_x, pred_x);
        q16_t res_y = q16_sub(meas_y, pred_y);
        
        /* 2. Alpha-Beta state update */
        trk->x_pos_norm = q16_add(pred_x, q16_mul(trk->alpha, res_x));
        trk->y_pos_norm = q16_add(pred_y, q16_mul(trk->alpha, res_y));
        
        trk->x_vel_norm_s = q16_add(trk->x_vel_norm_s, q16_div(q16_mul(trk->beta, res_x), dt_sec));
        trk->y_vel_norm_s = q16_add(trk->y_vel_norm_s, q16_div(q16_mul(trk->beta, res_y), dt_sec));
    } else {
        /* Coasting phase: extrapolate using last velocity */
        trk->missed_frames++;
        if (trk->missed_frames > 30) { /* 1 second at 30 fps */
            trk->target_locked = false;
            trk->x_vel_norm_s = 0;
            trk->y_vel_norm_s = 0;
        } else {
            trk->x_pos_norm = q16_add(trk->x_pos_norm, q16_mul(trk->x_vel_norm_s, dt_sec));
            trk->y_pos_norm = q16_add(trk->y_pos_norm, q16_mul(trk->y_vel_norm_s, dt_sec));
        }
    }
}

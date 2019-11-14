/*
 * Gimbal Kinematic and Electrical Travel Limit Checkers
 * Enforces software soft limits, mechanical bumper margins, and overspeed/overcurrent thresholds.
 */

#include "fdir.h"
#include "fixed_math.h"

void limit_checker_init(limit_checker_t *lc) {
    if (!lc) return;
    
    /* Pan continuous (+/- 180 deg) */
    lc->min_angle_rad[GIMBAL_AXIS_PAN] = -205887;
    lc->max_angle_rad[GIMBAL_AXIS_PAN] =  205887;
    
    /* Tilt (-90 deg to +45 deg) */
    lc->min_angle_rad[GIMBAL_AXIS_TILT] = -102944;
    lc->max_angle_rad[GIMBAL_AXIS_TILT] =   51472;
    
    /* Roll (+/- 45 deg) */
    lc->min_angle_rad[GIMBAL_AXIS_ROLL] = -51472;
    lc->max_angle_rad[GIMBAL_AXIS_ROLL] =  51472;
    
    for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
        lc->max_speed_rad_s[i] = q16_from_float(5.2f); /* ~300 deg/s */
        lc->max_current_amps[i] = q16_from_float(4.5f);
        lc->limit_tripped[i] = false;
    }
}

bool limit_checker_verify(limit_checker_t *lc, const vec3_q16_t *angles, const vec3_q16_t *velocities, const q16_t currents[3]) {
    if (!lc) return false;
    
    bool all_ok = true;
    const q16_t ang_arr[3] = {angles ? angles->x : 0, angles ? angles->y : 0, angles ? angles->z : 0};
    const q16_t vel_arr[3] = {velocities ? velocities->x : 0, velocities ? velocities->y : 0, velocities ? velocities->z : 0};
    
    for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
        /* Check angle limit */
        if (ang_arr[i] < lc->min_angle_rad[i] || ang_arr[i] > lc->max_angle_rad[i]) {
            lc->limit_tripped[i] = true;
            all_ok = false;
        }
        /* Check speed limit */
        if (q16_abs(vel_arr[i]) > lc->max_speed_rad_s[i]) {
            lc->limit_tripped[i] = true;
            all_ok = false;
        }
        /* Check current limit */
        if (currents && q16_abs(currents[i]) > lc->max_current_amps[i]) {
            lc->limit_tripped[i] = true;
            all_ok = false;
        }
    }
    return all_ok;
}

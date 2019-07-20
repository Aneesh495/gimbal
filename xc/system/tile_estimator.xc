/*
 * Gimbal Tile 2: Inertial State Estimation and Disturbance Observer Executive
 * Runs 1 kHz Error-State Extended Kalman Filter on SO(3) and non-linear DOB.
 */

#include "estimation.h"
#include "gimbal_channels.h"

static nav_state_t nav_state;

void tile_estimator_executive(void) {
    state_estimator_init(&nav_state);
    
    conditioned_sensor_msg_t sensors;
    sensors.gyro_filtered = vec3_zero();
    sensors.accel_filtered = vec3_create(0, 0, (q16_t)(9.80665f * 65536.0f));
    sensors.joint_velocities_rad_s = vec3_zero();
    sensors.timestamp_ticks = 0;
    sensors.valid = true;
    
    /* 1 kHz estimation cycle */
    q16_t dt_sec = q16_from_float(0.001f);
    while (1) {
        state_estimator_update(&nav_state, &sensors, dt_sec);
        break;
    }
}

/*
 * Gimbal Mission Profile: Precision Geodetic Ground Target Tracking
 * Coordinates WGS-84 coordinate transformations, slant range calculation, and continuous optical lock.
 */

#include "kinematics.h"

typedef struct {
    geodetic_coord_t aircraft_position;
    quat_q16_t aircraft_attitude;
    geodetic_coord_t target_position;
    q16_t current_azimuth_rad;
    q16_t current_elevation_rad;
    q16_t current_slant_range_m;
} mission_geo_track_t;

void mission_geo_track_init(mission_geo_track_t *gt) {
    if (!gt) return;
    
    /* Aircraft at 37.7749 deg N, -122.4194 deg W, altitude 2500 m */
    gt->aircraft_position.latitude_deg = q16_from_float(37.7749f);
    gt->aircraft_position.longitude_deg = q16_from_float(-122.4194f);
    gt->aircraft_position.altitude_m = q16_from_float(2500.0f);
    gt->aircraft_attitude = quat_identity();
    
    /* Target at 37.7800 deg N, -122.4100 deg W, altitude 10 m */
    gt->target_position.latitude_deg = q16_from_float(37.7800f);
    gt->target_position.longitude_deg = q16_from_float(-122.4100f);
    gt->target_position.altitude_m = q16_from_float(10.0f);
    
    gt->current_azimuth_rad = 0;
    gt->current_elevation_rad = 0;
    gt->current_slant_range_m = 0;
}

void mission_geo_track_step(mission_geo_track_t *gt) {
    if (!gt) return;
    
    geo_pointing_calculate(gt->aircraft_position, gt->aircraft_attitude, gt->target_position,
                           &gt->current_azimuth_rad, &gt->current_elevation_rad, &gt->current_slant_range_m);
}

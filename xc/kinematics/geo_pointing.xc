/*
 * Gimbal Geodetic Target Line-of-Sight Pointing Engine
 * Computes LOS vectors, azimuth, elevation, and slant range to ground targets.
 */

#include "kinematics.h"

void geo_pointing_calculate(geodetic_coord_t aircraft, quat_q16_t q_aircraft_att, geodetic_coord_t target, q16_t *azimuth_rad, q16_t *elevation_rad, q16_t *slant_range_m) {
    /* Conversion from geodetic deltas to approximate local NED vector in meters */
    /* 1 degree latitude ~= 111,139 meters = 7,283,446 in Q16 */
    /* 1 degree longitude ~= 111,139 * cos(lat) meters */
    
    q16_t deg_to_rad = 1143; /* pi / 180 in Q16 = 0.01745 * 65536 */
    q16_t meters_per_deg_lat = q16_from_float(111139.0f);
    
    q16_t lat_rad = q16_mul(aircraft.latitude_deg, deg_to_rad);
    q16_t c_lat;
    q16_cordic_sin_cos(lat_rad, NULL, &c_lat);
    q16_t meters_per_deg_lon = q16_mul(meters_per_deg_lat, c_lat);
    
    q16_t delta_lat = target.latitude_deg - aircraft.latitude_deg;
    q16_t delta_lon = target.longitude_deg - aircraft.longitude_deg;
    q16_t delta_alt = target.altitude_m - aircraft.altitude_m;
    
    vec3_q16_t los_ned;
    los_ned.x = q16_mul(delta_lat, meters_per_deg_lat);  /* North */
    los_ned.y = q16_mul(delta_lon, meters_per_deg_lon);  /* East */
    los_ned.z = -delta_alt;                             /* Down */
    
    q16_t range = vec3_mag(los_ned);
    if (slant_range_m) *slant_range_m = range;
    
    /* Transform LOS vector into Aircraft Body Frame */
    vec3_q16_t los_body;
    frames_ned_to_body(los_ned, q_aircraft_att, &los_body);
    
    /* Azimuth = atan2(y, x) in aircraft body */
    if (azimuth_rad) {
        *azimuth_rad = q16_cordic_atan2(los_body.y, los_body.x);
    }
    
    /* Elevation = atan2(-z, sqrt(x^2 + y^2)) */
    if (elevation_rad) {
        q16_t ground_dist = q16_sqrt(q16_mul(los_body.x, los_body.x) + q16_mul(los_body.y, los_body.y));
        *elevation_rad = q16_cordic_atan2(-los_body.z, ground_dist);
    }
}

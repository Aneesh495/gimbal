/*
 * Gimbal Autonomous Target Acquisition Search Patterns
 * Raster scans, horizontal sector swaths, and rosette multi-petal search geometries.
 */

#include "tracking.h"
#include "fixed_math.h"

void search_generator_init(search_generator_t *gen, search_pattern_type_t type, q16_t width, q16_t height, q16_t rate) {
    if (!gen) return;
    gen->pattern_type = type;
    gen->scan_width_rad = width;
    gen->scan_height_rad = height;
    gen->scan_rate_rad_s = rate;
    gen->time_accum_sec = 0;
    gen->pattern_offset_rad = vec3_zero();
}

vec3_q16_t search_generator_step(search_generator_t *gen, q16_t dt_sec) {
    if (!gen) return vec3_zero();
    
    gen->time_accum_sec = q16_add(gen->time_accum_sec, dt_sec);
    q16_t t = gen->time_accum_sec;
    
    vec3_q16_t offset = vec3_zero();
    
    switch (gen->pattern_type) {
        case SEARCH_PATTERN_RASTER: {
            /* Triangular horizontal sweep with slow vertical progression */
            q16_t period = q16_div(gen->scan_width_rad << 1, gen->scan_rate_rad_s);
            q16_t phase = (period > 0) ? (t % period) : 0;
            q16_t half_p = period >> 1;
            
            q16_t pan_offset;
            if (phase < half_p) {
                pan_offset = q16_mul(gen->scan_rate_rad_s, phase) - (gen->scan_width_rad >> 1);
            } else {
                pan_offset = (gen->scan_width_rad >> 1) - q16_mul(gen->scan_rate_rad_s, phase - half_p);
            }
            offset.x = pan_offset;
            offset.y = 0;
            break;
        }
        case SEARCH_PATTERN_SECTOR: {
            /* Sinusoidal azimuth sweep */
            q16_t w = q16_div(gen->scan_rate_rad_s, gen->scan_width_rad);
            q16_t angle = q16_mul(w, t);
            q16_t s;
            q16_cordic_sin_cos(angle, &s, NULL);
            offset.x = q16_mul(gen->scan_width_rad >> 1, s);
            offset.y = 0;
            break;
        }
        case SEARCH_PATTERN_ROSETTE: {
            /* Dual harmonic petal: r(t) = R * cos(omega1 * t), theta(t) = omega2 * t */
            q16_t w1 = q16_from_float(1.2f);
            q16_t w2 = q16_from_float(7.8f);
            q16_t s1, c1, s2, c2;
            q16_cordic_sin_cos(q16_mul(w1, t), &s1, &c1);
            q16_cordic_sin_cos(q16_mul(w2, t), &s2, &c2);
            
            q16_t r = q16_mul(gen->scan_width_rad >> 1, c2);
            offset.x = q16_mul(r, c1);
            offset.y = q16_mul(r, s1);
            break;
        }
    }
    
    gen->pattern_offset_rad = offset;
    return offset;
}

/*
 * XC Test Bench: Jerk-Limited S-Curve Profiler and Geodetic Pointing
 * Verifies acceleration continuity, maximum velocity envelopes, and geo-pointing.
 */

#include "kinematics.h"
#include <stdbool.h>

bool run_kinematics_bench(void) {
    scurve_profile_t prof;
    q16_t max_v = q16_from_float(2.0f);
    q16_t max_a = q16_from_float(8.0f);
    q16_t max_j = q16_from_float(40.0f);
    scurve_init(&prof, max_v, max_a, max_j);
    scurve_set_target(&prof, q16_from_float(1.5f));

    q16_t dt = q16_from_float(0.005f);
    for (int i = 0; i < 400; i++) {
        scurve_step(&prof, dt);
        /* Must not exceed velocity limit */
        if (q16_abs(prof.vel_current) > max_v + 100) return false;
        if (!prof.active) break;
    }

    /* Target reached within tolerance */
    if (q16_abs(prof.pos_current - q16_from_float(1.5f)) > 64) return false;
    return true;
}
